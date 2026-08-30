#!/usr/bin/env python3
"""Pull the arm back off the table and point the gripper straight DOWN.

    python3 scripts/retract_look_down.py

The pick kept failing because the arm ended up too far forward with the wrist
pointing ACROSS the table instead of down it.  A wrist camera aimed sideways
sees the table's edge and the room beyond, never the cube, and the plane fit
then latches onto the edge -- which is how the hand got reported as "-2 mm
above the table" while it was in mid air.

This does the two things that fixes:
  * retract horizontally toward the arm's own base, so the hand is over the
    near part of the table rather than out past it,
  * command the TOOL AXIS to point down, so the camera looks at the surface.

After each step it looks for the cube and stops as soon as it finds it.
"""
import argparse
import math
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from pick_from_top import cube_points, look_down_R  # noqa: E402
from servo_pick_left import CUBE_GRIP, Eye, GRIP_OPEN  # noqa: E402
from srl_sequence import ik_relaxed  # noqa: E402
from srl_fk import FK  # noqa: E402

UP = np.array([0.0, 0.0, 1.0])


def state(node, fk):
    q = node.fresh_q()
    Tl, Tr, Tee = fk.poses("left", q,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=CUBE_GRIP)
    Tb, = fk.poses("left", q, ["base_link"])
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    axis = Tee[:3, :3] @ np.array([0.0, 0.0, 1.0])
    down_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(axis @ -UP)))))
    return q, mid, Tee, Tb, axis, down_deg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pull-mm", type=float, default=120.0)
    ap.add_argument("--rise-mm", type=float, default=50.0)
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Eye("left")
    node.fk = fk = FK()
    if not node.preflight():
        raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()

    q, mid, Tee, Tb, axis, dd = state(node, fk)
    print("pads at world (%+.3f, %+.3f, %+.3f)" % tuple(mid))
    print("tool axis (%+.3f, %+.3f, %+.3f) -> %.0f deg away from straight down"
          % (*axis, dd))
    Pk = cube_points(node, fk)
    print("cube in view now: %s"
          % ("yes, %d pts" % len(Pk) if Pk is not None else "no"))
    if args.dry_run:
        return
    if Pk is not None and len(Pk) >= 80 and dd < 35:
        print("already looking down with the cube in view -- nothing to do")
        return

    node.hold_gripper(GRIP_OPEN, 1.5, "OPEN gripper")
    horiz = mid - Tb[:3, 3]
    horiz[2] = 0.0
    horiz /= np.linalg.norm(horiz)
    print("retracting along (%+.2f, %+.2f, 0) -- toward the arm's own base"
          % (-horiz[0], -horiz[1]))

    for i in range(1, args.steps + 1):
        q, mid, Tee, Tb, axis, dd = state(node, fk)
        target = mid - horiz * (args.pull_mm / 1000.0) + UP * (args.rise_mm / 1000.0)
        Rd = look_down_R(Tee, UP)
        qn, ep, tl = ik_relaxed(target, Rd, q, CUBE_GRIP, UP, fk)
        print("  step %d: retract %.0f mm, rise %.0f mm, look down "
              "(IK residual %.1f mm, tilt %.0f deg)"
              % (i, args.pull_mm, args.rise_mm, ep * 1000, tl))
        if ep > 0.012:
            print("     unreachable -- stopping")
            break
        if node.goto(qn, "retract-%d" % i) is None:
            print("     collision guard fired -- stopping")
            break
        q, mid, Tee, Tb, axis, dd = state(node, fk)
        print("     pads (%+.3f, %+.3f, %+.3f), tool %.0f deg from down"
              % (*mid, dd))
        Pk = cube_points(node, fk)
        if Pk is not None and len(Pk) >= 80:
            c = np.median(Pk, 0)
            print("     CUBE FOUND: %d points at (%+.3f, %+.3f, %+.3f), "
                  "range %.3f m" % (len(Pk), *c, np.linalg.norm(c)))
            print("\nnow run:  python3 scripts/pick_from_top.py")
            return
        print("     no cube in view yet")
    print("\ncube still not in view.")


if __name__ == "__main__":
    main()
