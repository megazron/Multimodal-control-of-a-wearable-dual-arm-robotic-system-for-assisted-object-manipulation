#!/usr/bin/env python3
"""WHAT LIMITS EACH ARM AT EACH PAD COLUMN, AND WHERE IT STOPS BEING THE PERSON.

    python3 scripts/measure_pad_columns.py

T1's two pads sit as near the centreline as they can. "As near as they can" was
being decided by whether the place pose SOLVED, and solving is not the
constraint -- every column from |x| = 0.120 outward solves 10 of 10. What
changes across those columns is how close the arm passes to the WEARER, and
that is what this measures: the place pose for a pad slot at each column, on
each arm, N=10, with the wearer measured geometrically by the mount guard's own
model and the table in the planning scene.

WHY IT WAS WORTH MEASURING RATHER THAN ARGUING. The shipped pair was 320 mm
(left) and 165 mm (right) off the centreline, and the composed path walked
three times gave the right arm 0.1611, 0.1787 and 0.1483 m against a 0.150 m
floor at one waypoint -- one run in three inside it, with 0 IK failures every
time. A column with no margin cannot absorb the null-space branch scatter a
7-DOF arm has, and the fix is a column with margin, not a re-run that comes out
better.

WHAT THE ANSWER LOOKS LIKE. Both arms saturate at the MOUNT's own clearance
(0.2202 m against this wearer, a link no joint moves), and they saturate at
different columns: the left arm is already mount-limited at 0.260, the right
arm is limited by the wearer's own UPPER ARM until 0.255. So there is a band
where the closest thing to the person is the mount rather than the arm, and a
pad in it cannot be made worse by a bad branch.

The reading to look for is `closest`: while it says `R-upperarm` the number is
about the person, and once it says `torso` at 0.2202 the number is about the
mount and the arm has stopped being the limit.
"""
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
from search_t1_centre import controls                         # noqa: E402
import t1_task as T1                                          # noqa: E402
import grasp_frames as GF                                     # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_pad_columns.json")
# The shipped slots are in here on purpose -- 0.260 and 0.320 for a pad with
# two cubes on it, 0.230, 0.290 and 0.350 for a pad with three: a test asserts
# every slot the task uses appears in this record, so a slot that moves without
# being measured fails rather than passing on a neighbouring column.
COLUMNS = [0.120, 0.135, 0.165, 0.195, 0.225, 0.230, 0.255, 0.260, 0.285,
           0.290, 0.315, 0.320, 0.345, 0.350, 0.375, 0.405, 0.440]
N = 10


def main():
    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)" % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    fk = FK(n)
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)
    ctl, good = controls(rig, pad_mid, CLEAR_FLOOR)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-38s %s" % (k, v))
    if not good:
        print("\nREFUSING: a control failed.")
        return 6
    rig.set_furniture(True)

    pz = round(T1.T1_Z + T1.PAD_T, 4)
    res = {"controls": ctl, "floor": CLEAR_FLOOR, "z": pz,
           "row_y": T1.ROW_Y, "repeats": N, "arms": {}}
    print("\nPLACE POSE AT EACH PAD COLUMN, N=%d, floor %.3f m" % (N, CLEAR_FLOOR))
    print("arm    |x|     solved   worst clearance   closest body")
    for arm in ("left", "right"):
        rows = []
        q = T1.APPROACH[arm]
        qm = as_msg(q)
        for col in COLUMNS:
            x = col if arm == "left" else -col
            ee = GF.wrist_for([x, T1.ROW_Y, pz], q)
            ok, worst, who = 0, 1e9, None
            for _ in range(N):
                j = n.solve_arm_joints(arm, list(ee), qm, avoid=True, tries=6)
                rig.calls += 1
                if j is None:
                    continue
                ok += 1
                c, k = rig.clearance(arm, j)
                if c is not None and c < worst:
                    worst, who = c, k
            rows.append(dict(column=col, solved=ok, of=N,
                             worst_clearance=None if worst > 1e8 else round(worst, 4),
                             closest=who,
                             mount_limited=who in ("torso",) and worst > 0.21))
            print("%-5s %.3f   %2d/%d     %s   %s"
                  % (arm, col, ok, N,
                     "------" if worst > 1e8 else "%.4f" % worst, who))
        res["arms"][arm] = rows

    # WHERE EACH ARM STOPS BEING THE LIMIT. Reported rather than left to the
    # reader, because it is the single number the layout is chosen on.
    for arm in ("left", "right"):
        sat = [r["column"] for r in res["arms"][arm]
               if r["worst_clearance"] is not None
               and r["worst_clearance"] >= 0.2200]
        res["arms"][arm + "_mount_limited_from"] = min(sat) if sat else None
        print("%-5s mount-limited from |x| = %s"
              % (arm, res["arms"][arm + "_mount_limited_from"]))
    res["shipped_planes"] = [list(p) for p in T1.T1_PLANES]
    res["slot_dx"] = T1.SLOT_DX

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, OUT))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
