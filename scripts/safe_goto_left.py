#!/usr/bin/env python3
"""Move the LEFT arm to a saved joint pose WITHOUT sweeping through the table.

WHY THIS EXISTS
---------------
The first attempt to return to the saved pose interpolated straight through
joint space from a pose hovering low over the table.  Joint-space straight
lines say nothing about where the hand goes, and the gripper was driven into
the table; the arm stalled 50 deg short of target, pressing against it.

So every path is now CHECKED before it is commanded: the whole hand is walked
through the interpolation in FK and every sample must clear the measured table
plane.  If the direct path does not clear it, a LIFT-FIRST detour is tried --
rise along the table normal, traverse high, descend -- and if that does not
clear either, nothing is commanded at all.

The table plane comes from the depth measurement of the real scene
(recordings/baselines/cube_located_v2.json): a plane fitted at sub-millimetre
RMS, expressed in world, so it is the same surface the grasp was planned
against rather than a nominal number.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from srl_fk import FK  # noqa: E402

from execute_pick_left import (ARRIVE_TOL_DEG, Executor, GRIP_OPEN,  # noqa: E402
                               RATE, ang_wrap)

# Every part of the hand that could touch the table.
HAND_LINKS = [
    "end_effector_link",
    "robotiq_85_base_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link",
    "robotiq_85_left_knuckle_link",
    "robotiq_85_right_knuckle_link",
]
TRANSIT_MARGIN_M = 0.040   # keep this much air under the hand while moving
LIFT_M = 0.110             # how far to rise for the detour
CHECK_STEPS = 60


class TableCheck:
    def __init__(self, cube_json):
        d = json.load(open(cube_json))
        n = np.array(d["table_normal_world"], float)
        self.n = n / np.linalg.norm(n)
        cube = np.array(d["grasp_world"], float)
        # the grasp centre sits half a cube above the surface
        on_plane = cube - self.n * (d["height_mm"] / 2000.0)
        self.d = -float(self.n @ on_plane)

    def height(self, p):
        return float(self.n @ np.asarray(p) + self.d)

    def min_hand_height(self, fk, q, grip=0.0):
        Ts = fk.poses("left", q, HAND_LINKS, gripper=grip)
        return min(self.height(T[:3, 3]) for T in Ts)


def path_clear(fk, tab, q0, q1, margin, steps=CHECK_STEPS, grip=0.0):
    """Lowest point the hand reaches over a straight joint interpolation."""
    worst = float("inf")
    worst_a = 0.0
    for k in range(steps + 1):
        a = k / steps
        q = np.array(q0) + (np.array(q1) - np.array(q0)) * a
        h = tab.min_hand_height(fk, q, grip)
        if h < worst:
            worst, worst_a = h, a
    return worst >= margin, worst, worst_a


def lift_pose(fk, tab, q_seed, lift_m):
    """IK a pose that is q_seed raised `lift_m` along the table normal."""
    from plan_pick_left import ik  # reuse the same solver the grasp uses
    Tee, = fk.poses("left", q_seed, ["end_effector_link"])
    lf, rf = fk.poses("left", q_seed,
                      ["robotiq_85_left_finger_tip_link",
                       "robotiq_85_right_finger_tip_link"])
    mid = (lf[:3, 3] + rf[:3, 3]) / 2.0
    target = mid + tab.n * lift_m
    q, ep, er, it = ik(target, Tee[:3, :3], q_seed)
    return q, ep, er


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", required=True)
    ap.add_argument("--arm-key", default="left")
    ap.add_argument("--cube", default="/home/gausms/kortex_ws/recordings/"
                                      "baselines/cube_located_v2.json")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    fk = FK()
    tab = TableCheck(args.cube)
    q_goal = np.array(json.load(open(args.pose))[args.arm_key]["rad"], float)
    print("goal pose (deg):", [round(math.degrees(x), 2) for x in q_goal])
    print("goal hand clearance above the table: %.1f mm"
          % (tab.min_hand_height(fk, q_goal) * 1000))

    rclpy.init()
    node = Executor()
    if not node.preflight():
        raise SystemExit(2)
    q_now = node.q.copy()
    print("start pose (deg):", [round(math.degrees(x), 2) for x in q_now])
    print("start hand clearance above the table: %.1f mm"
          % (tab.min_hand_height(fk, q_now) * 1000))

    if math.degrees(np.abs(ang_wrap(q_now - q_goal)).max()) < ARRIVE_TOL_DEG:
        print("already at the goal; nothing to do")
        return

    # 1) is the direct path clear?
    ok, worst, at = path_clear(fk, tab, q_now, q_goal, TRANSIT_MARGIN_M)
    print("\ndirect path: lowest hand point %.1f mm above the table "
          "(at %.0f%% along)%s"
          % (worst * 1000, at * 100, "" if ok else "   <-- WOULD HIT"))
    waypoints = []
    if ok:
        waypoints = [("GOAL", q_goal)]
    else:
        # 2) lift-first detour
        qa, epa, _ = lift_pose(fk, tab, q_now, LIFT_M)
        qb, epb, _ = lift_pose(fk, tab, q_goal, LIFT_M)
        print("detour: lift %.0f mm at both ends (IK residual %.2f / %.2f mm)"
              % (LIFT_M * 1000, epa * 1000, epb * 1000))
        legs = [("LIFT", q_now, qa), ("TRAVERSE", qa, qb),
                ("DESCEND", qb, q_goal)]
        all_ok = True
        for nm, a, b in legs:
            o, w, p = path_clear(fk, tab, a, b, TRANSIT_MARGIN_M)
            print("  %-9s lowest %.1f mm%s" % (nm, w * 1000,
                                               "" if o else "   <-- WOULD HIT"))
            all_ok = all_ok and o
        if not all_ok:
            raise SystemExit(
                "\nNo clear path found. NOTHING COMMANDED.\n"
                "Move the arm clear of the table by hand, or raise LIFT_M.")
        waypoints = [("LIFT", qa), ("TRAVERSE", qb), ("GOAL", q_goal)]

    print("\nplan: " + " -> ".join(n for n, _ in waypoints))
    if args.dry_run:
        print("dry run: nothing commanded")
        return

    node.set_deadband(0.10)
    node._hold = node.q.copy()
    node.hold_gripper(GRIP_OPEN, 2.0, "OPEN gripper")
    for nm, qt in waypoints:
        print("\n%s" % nm)
        err = node.goto(np.array(qt), nm)
        if err is None or err > ARRIVE_TOL_DEG * 2:
            raise SystemExit("stopped at %s (%s deg out)"
                             % (nm, "n/a" if err is None else "%.2f" % err))
    t0 = time.time()
    while time.time() - t0 < 1.5:
        node.send_arm(node._hold)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / RATE)
    print("\nAT THE IDEAL POSE; hand %.1f mm above the table"
          % (tab.min_hand_height(fk, node.q) * 1000))


if __name__ == "__main__":
    main()
