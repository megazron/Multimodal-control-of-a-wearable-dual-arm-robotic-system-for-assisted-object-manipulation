#!/usr/bin/env python3
"""WHERE CAN THE ARM WORK **AND** KEEP THE WEARER CLEARANCE FLOOR?

    python3 scripts/measure_clearance_region.py [--repeats 1] [--step 0.05]

THE GAP THIS EXISTS TO CLOSE. Every workspace number in this repository is an
IK number. `survey_work_surface.py` asks /compute_ik with `avoid_collisions`
and writes down the cells that solve; the marking drawn in every clip is that
file; T1 stage 2 samples positions out of it. Not one of those steps ever
asks how close the metal gets to the person.

It cannot, either, and that is the point. HARD CONSTRAINT 11 says the
clearance floor is the last thing between the arms and a person's chest, and
the SRDF permanently excludes torso/harness/backpack against each arm's
base, shoulder and half_arm_1 -- the pairs a shoulder-mounted arm actually
threatens. So MoveIt returns "valid" for poses with the tube inside the
wearer, exactly as `find_presentation_pose.py` records, and a survey built on
`avoid_collisions` inherits that silence.

Measured here on the FIRST WALK OF THE DIRECTION SWEEP: the innermost cell of
the left arm's own surveyed region, (0.300, 0.175), leaves **23 mm** between
the forearm and the wearer's arm against a **150 mm** floor. It is inside the
marking a participant would be told to work in.

WHAT IS MEASURED. For each cell, the whole pick path -- standoff, descend,
lift -- solved collision-aware, and each solution's worst arm-to-wearer
distance under the mount guard's own capsule model. A cell passes only if
every waypoint on it solves AND every solution clears the floor.

THE HONEST LIMIT, STATED UP FRONT. /compute_ik returns ONE of many solutions
for a 7-DOF arm and TRAC-IK restarts randomly, so the clearance reported for
a cell is the clearance of the solution the solver happened to return -- which
is also the pose the follower would be commanded to, so it is the right
quantity, but it is a sample and not an infimum. Cells near the floor should
be re-measured at higher N before anything is built on them.

CONTROLS, and no report without them:
    a pose inside the torso            must measure NEGATIVE clearance
    the home pose                      must clear the floor
    a cell 0.6 m outboard              must clear the floor by a wide margin
"""
import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/clearance_region.json")
Z_WORK = CT.BENCH_TOP + 0.02
STANDOFF, LIFT = 0.10, 0.08


def path_for(ee):
    pre = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.04) + densify([ee, up], 0.04)


def cell(rig, arm, obj, repeats):
    """(reachable, worst_clearance, which_body) over the whole pick path."""
    worst, who = 1e9, None
    for w in path_for(CT.ee_for(obj, arm)):
        j = None
        for _ in range(repeats):
            j = rig.solve_joints(arm, w, avoid=True)
            if j is None:
                return False, None, None
        c, k = rig.clearance(arm, j)
        if c is None:
            return False, None, None
        if c < worst:
            worst, who = c, k
    return True, worst, who


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--x", type=float, nargs=2, default=(0.20, 0.70))
    ap.add_argument("--y", type=float, nargs=2, default=(0.10, 0.45))
    ap.add_argument("--z", type=float, default=Z_WORK)
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
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)

    # ---------------------------------------------------------- controls
    ctl = {}
    jt = rig.solve_joints("left", [0.05, -0.02, 1.22], avoid=False)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = dict(
        want="< 0", got=None if c_t is None else round(c_t, 4))
    home = [n.js.get(k, 0.0) for k in n.names("left")]
    c_h = rig.clearance("left", home)[0]
    ctl["home_clears_floor"] = dict(want=">= %.2f" % a.floor,
                                    got=round(c_h, 4))
    okc, c_o, _ = cell(rig, "left", [0.60, 0.175, a.z], 1)
    ctl["outboard_cell_clears"] = dict(want=">= %.2f" % a.floor,
                                       got=None if c_o is None
                                       else round(c_o, 4), reachable=okc)
    # A DIFFERENTIAL CONTROL, PER ARM, AND IT IS THE ONE THAT MATTERS.
    #
    # The first run of this script reported 80 right-arm cells with ONE
    # distinct clearance value, 0.1610 m, while the left arm's varied from
    # -0.003 to 0.161. That was not an asymmetric robot; it was
    # `positions[:7]` on an IK response that returns BOTH arms with the left
    # one first, so every right-arm clearance was the left arm at home. A map
    # of one value and a map that was never computed are the same picture.
    #
    # So each arm must show its OWN clearance FALLING as the target comes in
    # toward the wearer. A constant is a failed control, not a finding.
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        ok_in, c_in, _ = cell(rig, arm, [sgn * 0.30, 0.15, a.z], 1)
        ok_out, c_out, _ = cell(rig, arm, [sgn * 0.65, 0.15, a.z], 1)
        ctl["%s_varies_with_distance" % arm] = dict(
            want="inboard < outboard",
            got="%s vs %s" % (None if c_in is None else round(c_in, 4),
                              None if c_out is None else round(c_out, 4)),
            inboard_reachable=ok_in, outboard_reachable=ok_out)
    varies = all(
        ctl["%s_varies_with_distance" % arm]["got"].count("None") == 0
        and float(ctl["%s_varies_with_distance" % arm]["got"].split(" vs ")[0])
        < float(ctl["%s_varies_with_distance" % arm]["got"].split(" vs ")[1])
        for arm in ("left", "right"))
    good = (c_t is not None and c_t < 0.0
            and c_h >= a.floor
            and okc and c_o is not None and c_o >= a.floor
            and varies)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-26s want %-20s got %s" % (k, v["want"], v["got"]))
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    def frange(lo, hi, st):
        out, v = [], lo
        while v <= hi + 1e-9:
            out.append(round(v, 4))
            v += st
        return out

    xs, ys = frange(*a.x, a.step), frange(*a.y, a.step)
    res = {}
    print("\nPICK PATH REACHABLE **AND** CLEAR OF THE WEARER  (floor %.2f m, "
          "N=%d, %d x %d cells)" % (a.floor, a.repeats, len(xs), len(ys)))
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        ik_cells, clear_cells, grid = [], [], {}
        for x in xs:
            for y in ys:
                ok, c, who = cell(rig, arm, [sgn * x, y, a.z], a.repeats)
                if not ok:
                    continue
                ik_cells.append([round(sgn * x, 4), y])
                grid["%+.3f,%.3f" % (sgn * x, y)] = dict(
                    clearance_m=round(c, 4), to=who)
                if c >= a.floor:
                    clear_cells.append([round(sgn * x, 4), y])
        res[arm] = dict(ik_cells=ik_cells, clear_cells=clear_cells,
                        clearance=grid)
        if ik_cells:
            cs = [grid["%+.3f,%.3f" % (p[0], p[1])]["clearance_m"]
                  for p in ik_cells]
            print("   %-5s IK-reachable %3d   ALSO clear of the floor %3d "
                  "(%3d%%)   worst clearance %.4f m"
                  % (arm, len(ik_cells), len(clear_cells),
                     round(100.0 * len(clear_cells) / len(ik_cells)),
                     min(cs)))
            if clear_cells:
                X = [p[0] for p in clear_cells]
                Y = [p[1] for p in clear_cells]
                print("         the CLEAR region: x %+.3f..%+.3f  y %.3f..%.3f"
                      % (min(X), max(X), min(Y), max(Y)))
        else:
            print("   %-5s NO reachable cell in this box" % arm)

    out = dict(z=a.z, step=a.step, repeats=a.repeats, floor=a.floor,
               x_range=list(a.x), y_range=list(a.y), controls=ctl,
               per_arm=res, ik_calls=rig.calls,
               caveat="clearance is the clearance of the solution "
                      "/compute_ik returned, which is the pose the follower "
                      "would be commanded to, but is a sample and not an "
                      "infimum over the null space")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
