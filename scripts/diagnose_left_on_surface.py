#!/usr/bin/env python3
"""WHY CAN THE RIGHT ARM WORK ON THE SURFACE AND THE LEFT ARM NOT AT ALL?

    python3 scripts/diagnose_left_on_surface.py

`search_t1_centre.py --stage b` confirmed the RIGHT arm picking an object
RESTING on a table at |x| = 0.275, y = 0.300, table top 0.90, over the full
pick path at N=10 with 0.1504 m of wearer clearance -- and found NOTHING for
the left arm anywhere in the same sweep.

That is a large asymmetry and this project's standing rule says it is a
question about the INSTRUMENT until the instrument has been cleared. Two
things could produce it that are not the robot:

  1. THE HEADING SIGN. The fan mirrors the heading per arm so that "50 deg
     inboard" means the same thing on both. If that mirror is backwards for
     one arm, that arm was measured reaching AWAY from the centreline and the
     result is about a question nobody asked.
  2. THE FAN CAP. Ten of fourteen surviving approaches went forward and four
     were dropped, all of them at heading +25. If the left arm's only working
     approach was among them, the sweep never tried it.

So this takes the RIGHT arm's winning cell, mirrors it exactly, and asks the
left arm the same question -- then walks `measure_what_binds`'s ladder at that
cell so the answer has a MECHANISM attached rather than a count, and finally
re-tries the left arm over the WHOLE fan including the dropped members.

There is no report if the controls fail, and the controls include the right
arm's own winning cell, which must come back reachable: if it does not, the
scene here is not the scene the sweep measured and nothing else in the output
means anything.
"""
import argparse
import json
import os
import sys

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from search_t1_centre import (put_table, clear_table, solve_cell,  # noqa: E402
                              build_fan, frange, CUBE)
import grasp_frames as GF                                     # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/left_arm_on_surface.json")

# The right arm's confirmed cell, from t1_centre_stage_b.json.
#
# TABLE TOP 0.95, NOT 0.90, AND THE DIFFERENCE IS THE WHOLE POINT OF A MARGIN.
# The sweep's headline cell was the same column over a 0.90 table and it read
# 0.1504 m against a 0.150 floor. Re-measured twice here it read 0.1509 and
# 0.1462 -- TRAC-IK restarts randomly, so N=10 is ten different postures and
# the worst of them moves between runs. A cell whose verdict flips on a
# re-run is MARGINAL, and this project's own rule is that marginal is not
# usable.
#
# The same column over a 0.95 table read 0.1610 in the sweep, which is not
# "better by 11 mm": 0.1610 is `base_link`'s own distance from the torso, the
# mount's clearance, which no joint can move. It is the CEILING. A cell
# reading 0.1610 is one where the arm is never the closest thing to the
# wearer at any waypoint, and that cannot flip on a re-run.
WIN = dict(x=0.275, y=0.300, top=0.95, elev=-10.0, head=-50.0, roll=0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
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
    fk = FK(n)
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    z_obj = round(WIN["top"] + CUBE / 2.0, 4)
    near = round(max(0.02, WIN["y"] - CUBE / 2.0), 4)
    rig.set_furniture(False)
    put_table(rig, WIN["top"], near)

    q = {"left": GF.q_from_axis(GF.axis_for(WIN["elev"], WIN["head"]),
                                np.radians(WIN["roll"])),
         "right": GF.q_from_axis(GF.axis_for(WIN["elev"], -WIN["head"]),
                                 np.radians(WIN["roll"]))}
    for arm in ("left", "right"):
        e, h = GF.elev_head_of(q[arm])
        ax = GF.tool_axis(q[arm])
        inboard = (ax[0] < 0) if arm == "left" else (ax[0] > 0)
        print("   %-5s tool axis %s  elev %+.1f head %+.1f  points %s"
              % (arm, [round(float(v), 3) for v in ax], e, h,
                 "INBOARD" if inboard else "OUTBOARD -- THE MIRROR IS WRONG"))

    # ------------------------------------------------------------- control
    obj_r = [-WIN["x"], WIN["y"], z_obj]
    ok_r, c_r, who_r, _ = solve_cell(rig, "right", obj_r, q["right"],
                                     pad_mid["right"], a.repeats, True)
    print("\nCONTROL -- the right arm's own winning cell, re-measured here")
    print("   %s  reachable=%s clearance=%s to %s"
          % (obj_r, ok_r, None if c_r is None else round(c_r, 4), who_r))
    if not (ok_r and c_r is not None and c_r >= a.floor):
        print("\nREFUSING TO REPORT: the control cell does not reproduce, so "
              "this scene is not the one the sweep measured.")
        clear_table(rig)
        return 6

    # ------------------------------------------------- the mirrored question
    obj_l = [WIN["x"], WIN["y"], z_obj]
    ok_l, c_l, who_l, _ = solve_cell(rig, "left", obj_l, q["left"],
                                     pad_mid["left"], a.repeats, True)
    print("\nTHE SAME CELL, MIRRORED, ON THE LEFT ARM")
    print("   %s  reachable=%s clearance=%s to %s"
          % (obj_l, ok_l, None if c_l is None else round(c_l, 4), who_l))

    # ------------------------------------------------------------ the ladder
    # WHAT STOPS IT, by name. The wrist pose the pads would need, walked
    # through collision-aware -> furniture out -> collisions off -> wrist free.
    ee_l = GF.wrist_for(obj_l, q["left"], pad_mid["left"])
    saved = rig.quat["left"]
    rig.quat["left"] = as_msg(q["left"])
    verdict, why, clr, who = rig.classify("left", ee_l)
    rig.quat["left"] = saved
    # PUT THE SCENE BACK. `Rig.classify` walks its ladder by switching
    # clip_scene's OWN furniture on and off, and it leaves it ON -- so
    # everything after this call ran with the shipped table at 0.980 AND the
    # search slab at 0.900 in the scene at once. Measured, and it is the
    # "results depend on run order" row of CLAUDE.md's table: the first
    # version of this script reported the left arm reaching the cell over the
    # FULL path and then 0 of 181 approaches reaching the GRASP POSE inside
    # it, which cannot both be true of one scene.
    rig.set_furniture(False)
    put_table(rig, WIN["top"], near)
    print("\nTHE LADDER at the left arm's grasp WRIST pose %s"
          % [round(v, 4) for v in ee_l])
    print("   BINDS: %-11s %s" % (verdict, why))
    print("   clearance %s to %s"
          % ("----" if clr is None else "%.4f" % clr, who))

    # ---------------------------------------- the whole fan, left arm, no cap
    print("\nTHE WHOLE FAN ON THE LEFT ARM at this cell -- including the four "
          "members the sweep's cap dropped")
    fan = build_fan(rig, elevs=[-20, -15, -10, -5, 0, 5, 10, 15, 20],
                    heads=[-50, -25, 0, 25, 50], rolls=[0, 45, 90, 135])
    grasp_ok = []
    for name, quats in fan:
        ok, c, who, _ = solve_cell(rig, "left", obj_l, quats["left"],
                                   pad_mid["left"], 1, False)
        if ok:
            grasp_ok.append((name, None if c is None else round(c, 4), who))
    print("   %d of %d approaches reach the GRASP POSE at all" % (len(grasp_ok),
                                                                  len(fan)))
    for name, c, who in grasp_ok[:20]:
        print("      %-20s clearance %s to %s" % (name, c, who))

    full_ok = []
    for name, c, _who in grasp_ok:
        if c is None or c < a.floor:
            continue
        quats = dict(fan)[name]
        ok, c2, who2, _ = solve_cell(rig, "left", obj_l, quats["left"],
                                     pad_mid["left"], a.repeats, True)
        if ok and c2 is not None and c2 >= a.floor:
            full_ok.append((name, round(c2, 4), who2))
    print("   %d of those survive the FULL path at N=%d and the floor"
          % (len(full_ok), a.repeats))
    for name, c, who in full_ok:
        print("      %-20s clearance %.4f to %s" % (name, c, who))

    # ------------------------------- how far out must the left arm go instead
    # AND THE NUMBER THAT MATTERS: the left arm reaches this cell but sits
    # 12.5 mm inside the floor there, so the question is not "can it" but "how
    # far out before the floor is clear".
    print("\nHOW FAR OUT THE LEFT ARM MUST GO, at the best approach it has")
    # THE MIRRORED APPROACH FIRST, because that is the one the right arm won
    # with and the comparison only means something if both arms are asked the
    # same question. Fall back to whatever the fan found only if it fails.
    _mirror = "e%+05.1f/h%+05.1f/r%05.1f" % (WIN["elev"], WIN["head"],
                                             WIN["roll"])
    _names = [x[0] for x in full_ok] or [x[0] for x in grasp_ok]
    best = (_mirror if _mirror in _names
            else (_names[0] if _names else None))
    reach = None
    if best is not None:
        quats = dict(fan)[best]
        for x in frange(WIN["x"], 0.90, 0.025):
            obj = [x, WIN["y"], z_obj]
            ok, c, who, _ = solve_cell(rig, "left", obj, quats["left"],
                                       pad_mid["left"], a.repeats, True)
            if ok and c is not None and c >= a.floor:
                reach = dict(x=x, clearance=round(c, 4), to=who,
                             approach=best)
                print("   innermost |x| = %.3f at approach %s, clearance "
                      "%.4f to %s" % (x, best, c, who))
                break
        if reach is None:
            print("   NOTHING from |x| = %.3f to 0.900 at approach %s."
                  % (WIN["x"], best))
    else:
        print("   the left arm cannot reach this cell under ANY member of the "
              "fan, so there is no best approach to walk outward with.")
    clear_table(rig)

    out = dict(win=WIN, control_right=dict(ok=ok_r, clearance=c_r, to=who_r),
               left_at_mirror=dict(ok=ok_l, clearance=c_l, to=who_l),
               ladder=dict(verdict=verdict, detail=why, clearance=clr, to=who),
               left_grasp_ok=[[a_, b_, c_] for a_, b_, c_ in grasp_ok],
               left_full_ok=[[a_, b_, c_] for a_, b_, c_ in full_ok],
               left_innermost=reach, repeats=a.repeats, floor=a.floor,
               ik_calls=rig.calls)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2, default=float)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
