#!/usr/bin/env python3
"""THE INNERMOST COLUMN EACH ARM CAN PUT A PAD IN, over the full path.

    python3 scripts/sim_session.py --stack moveit -- \
        python3 scripts/measure_pad_columns.py --repeats 10

WHY A SEPARATE FILE. `measure_centre_vs_height.py` answers "is the CENTRE
reachable" and it answers it well, but it re-tests only ONE cell per arm over
the full path -- the innermost that survived the optimistic grasp-pose stage.
When that cell fails the full path, as the right arm's did on 2026-08-16
(x = 0.275, clearance 0.1173 m against a 0.150 floor), the run reports the
failure and stops. It never says where the arm CAN go.

That is the number a layout needs, so this walks outward from the failing
column, testing the WHOLE pick path at N=10 with the wearer measured
geometrically, and stops at the first column that clears the floor. The pads
go there.

CONTROLS, and there is no report without them:

    a column deep inside the wearer must FAIL, so a pass means something
    a column far outboard must PASS, so a fail is not the rig being broken
    the clearance model must go NEGATIVE inside the torso
    the arms must be AT HOME, because reach is measured from the IK seed and
    a run taken from somewhere else answers a different question
"""
import argparse
import json
import math
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))

from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                  # noqa: E402
from measure_centre_vs_height import full_path, _wrist           # noqa: E402
from measure_grasp_approach import as_msg                        # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import master_calibration as mc                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/pad_columns.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--z", type=float, default=1.12)
    ap.add_argument("--y", type=float, nargs="*",
                    default=[0.200, 0.240, 0.280, 0.320])
    ap.add_argument("--x-from", type=float, default=0.250)
    ap.add_argument("--x-to", type=float, default=0.700)
    ap.add_argument("--step", type=float, default=0.025)
    ap.add_argument("--slot-dy", type=float, default=0.030,
                    help="the place path's per-cube slot offset in y")
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        off, j = n.home_ok(arm)
        if off > HOME_TOL_RAD:
            print("REFUSING: %s arm is %.4f rad from home at joint_%d"
                  % (arm, off, j))
            return 3
    rig = Rig(n, "t1", 1)
    # `_wrist` branches on these: None means "the PINNED anchor", which is what
    # every teleop mode commands and therefore the only orientation a pad
    # column may be measured at. measure_centre_vs_height sets them the same
    # way for its default run.
    rig.pad_ee = None
    rig.pad_rot = None
    # SOLVE AT THE PINNED ANCHOR, NOT AT WHATEVER THE ARM IS RESTING IN.
    #
    # `Rig.__init__` sets `self.quat = node.ee_quat(arm)` -- the LIVE
    # end-effector orientation, read off TF at construction, which is the HOME
    # orientation. `Rig.solve_joints` then uses it for every query, and
    # `measure_centre_vs_height.full_path` goes through `solve_joints`. So
    # every reachability number those paths produce is measured at the HOME
    # wrist, while `run_abc.send()` commands `WORKSPACE_ORIENT` -- the pinned
    # 30.7 deg near-side anchor -- for every waypoint of every task.
    #
    # That was survivable while home and anchor were similar. It is not
    # survivable now: the 2026-08-16 home puts the tool axis LEVEL and
    # FORWARD, and the anchor is 30.7 deg above horizontal arriving from the
    # near side. Measured, the difference is the whole result -- this file
    # first reported the pads safe at (0.425, 0.270) with both slots clear,
    # and T1 then lost 14 waypoints at exactly those two slots.
    #
    # Overriding rig.quat here is the minimal fix: `full_path` reads it, so
    # every solve below is at the orientation the task actually commands.
    for _a in ("left", "right"):
        rig.quat[_a] = as_msg(tuple(mc.WORKSPACE_ORIENT[_a]))

    # ------------------------------------------------------------ controls
    ctl = {}
    ok_in, c_in, _ = full_path(rig, "left", [0.05, 0.30, a.z], 1)
    ctl["deep_inside_the_wearer_fails"] = dict(
        want="not reachable-and-clear",
        got="%s / %s" % (ok_in, None if c_in is None else round(c_in, 4)))
    good_in = (not ok_in) or (c_in is not None and c_in < a.floor)

    ok_out, c_out, _ = full_path(rig, "left", [0.60, 0.24, a.z], 1)
    ctl["far_outboard_passes"] = dict(
        want="reachable and >= floor",
        got="%s / %s" % (ok_out, None if c_out is None else round(c_out, 4)))
    good_out = ok_out and c_out is not None and c_out >= a.floor

    jt = rig.solve_joints("left", [0.05, -0.02, 1.22], avoid=False)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = dict(
        want="< 0", got=None if c_t is None else round(c_t, 4))
    good_t = c_t is not None and c_t < 0.0

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-32s want %-26s got %s" % (k, v["want"], v["got"]))
    if not (good_in and good_out and good_t):
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    # -------------------------------------------------------------- sweep
    xs = [round(a.x_from + a.step * i, 4)
          for i in range(int((a.x_to - a.x_from) / a.step) + 1)]
    print("\nFULL PICK PATH, N=%d, wearer geometric, floor %.3f m"
          % (a.repeats, a.floor))
    print("   %-6s %-6s %-7s %-10s %-9s %s"
          % ("arm", "x", "y", "reachable", "clear", "verdict"))
    res, inner = {}, {}
    for arm in ("left", "right"):
        rows = []
        found = None
        for x in xs:
            sx = x if arm == "left" else -x
            hit = None
            for y in a.y:
                # THE PAD CENTRE IS NOT A WAYPOINT. The place path gives each
                # of the two cubes sharing a pad its own slot at +/-SLOT_DY in
                # y, and those slots are what the arm is actually commanded
                # to. Testing the centre alone is why the first two layouts
                # this file produced both failed T1: the centre passed and a
                # slot 30 mm away did not. Both slots must hold, or the pad
                # cannot go here.
                per = []
                for dy in (-a.slot_dy, +a.slot_dy):
                    ok, c, who = full_path(rig, arm, [sx, y + dy, a.z],
                                           a.repeats)
                    per.append((y + dy, ok, c, who))
                ok = all(p[1] for p in per)
                cs = [p[2] for p in per if p[2] is not None]
                c = min(cs) if cs else None
                who = next((p[3] for p in per
                            if p[2] is not None and p[2] == c), None)
                rows.append(dict(x=x, y=y, slots=[p[0] for p in per],
                                 reachable=bool(ok),
                                 clearance_m=None if c is None else round(c, 4),
                                 to=who,
                                 per_slot=[dict(y=p[0], reachable=bool(p[1]),
                                                clearance_m=None if p[2] is None
                                                else round(p[2], 4))
                                           for p in per]))
                verdict = ("SAFE" if ok and c is not None and c >= a.floor
                           else ("reach only" if ok else "unreachable"))
                print("   %-6s %-6.3f %-7.3f %-10s %-9s %s   slots %s"
                      % (arm, x, y, ok,
                         "----" if c is None else "%.4f" % c, verdict,
                         " ".join("%.3f:%s" % (p[0],
                                               "-" if p[2] is None
                                               else "%.4f" % p[2])
                                  for p in per)))
                if verdict == "SAFE" and hit is None:
                    hit = dict(x=x, y=y, clearance_m=round(c, 4), to=who,
                               slots=[p[0] for p in per])
            if hit is not None:
                found = hit
                break
        res[arm] = rows
        inner[arm] = found
        if found:
            print("   -> %s arm innermost SAFE pad column |x| = %.3f "
                  "(y = %.3f, clearance %.4f m to %s)"
                  % (arm, found["x"], found["y"], found["clearance_m"],
                     found["to"]))
        else:
            print("   -> %s arm: NO safe column in %.3f..%.3f"
                  % (arm, a.x_from, a.x_to))

    print("\nANSWER")
    for arm in ("left", "right"):
        f = inner[arm]
        print("   %-5s innermost safe pad column |x| = %s"
              % (arm, "NONE" if f is None else "%.3f m (%d mm off centre)"
                 % (f["x"], round(f["x"] * 1000))))
    json.dump(dict(controls=ctl, floor=a.floor, z=a.z, ys=a.y,
                   repeats=a.repeats, rows=res, innermost=inner),
              open(a.out, "w"), indent=2)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
