#!/usr/bin/env python3
"""Drive the LEFT arm to a saved joint pose, and wait until it arrives.

Used to return to the operator's hand-set pick pose before planning, so every
run starts from the same place rather than from wherever the last run stopped.

Arrival is WAITED FOR, not timed: the bridge closes error with a proportional
law (kp = 0.5, ~2 s time constant), so a fixed settle window reports a miss on
a move that was still converging.  That is exactly what aborted the first pick.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy

from execute_pick_left import (ARRIVE_TOL_DEG, CONVERGE_MAX_S, DEG_PER_S,
                               Executor, GRIP_OPEN, RATE, STALL_MIN_DEG,
                               STALL_S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", required=True,
                    help="json with {'left': {'rad': [7 values]}}")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--open-gripper", action="store_true",
                    help="open the hand on the way, so the pads are clear")
    args = ap.parse_args()

    d = json.load(open(args.pose))
    q_target = np.array(d["left"]["rad"], float)
    print("target pose (deg):", [round(math.degrees(x), 2) for x in q_target])

    rclpy.init()
    node = Executor()
    if not node.preflight():
        raise SystemExit(2)
    if args.dry_run:
        print("dry run: nothing commanded")
        return

    node.set_deadband(0.10)
    print("start q (deg):", [round(math.degrees(x), 2) for x in node.q])
    node._hold = node.q.copy()
    delta = math.degrees(np.abs(node.q - q_target).max())
    print("worst joint must move %.2f deg" % delta)
    if delta < ARRIVE_TOL_DEG:
        print("already there; nothing to do")
        return

    if args.open_gripper:
        node.hold_gripper(GRIP_OPEN, 2.0, "OPEN gripper")

    err = node.goto(q_target, "SAVED-POSE")
    if err is None:
        raise SystemExit("arm did not respond; see the message above")
    if err > ARRIVE_TOL_DEG * 2:
        raise SystemExit("did not reach the saved pose (%.2f deg out)" % err)

    # hold briefly so the arm does not sag between programs
    t0 = time.time()
    while time.time() - t0 < 1.5:
        node.send_arm(node._hold)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / RATE)
    print("AT SAVED POSE")


if __name__ == "__main__":
    main()
