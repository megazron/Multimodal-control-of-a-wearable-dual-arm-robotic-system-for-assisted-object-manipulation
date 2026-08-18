#!/usr/bin/env python3
"""DRIVE BOTH ARMS TO THE TWO PADS AND HOLD, so a still shows the work.

    python3 scripts/pose_arms_at_pads.py --hold 600

A picture of the scene at HOME shows where the pads are. It does not show that
both arms can WORK there, which is the claim being made -- so this solves each
arm's own pad slot through the same solver the sweeps use, refuses if either
side does not solve or sits inside the 150 mm floor, and only then commands it.

REFUSING IS THE POINT. A render that quietly falls back to the home pose when
IK fails is a render that illustrates a claim it did not test. If either arm
cannot be posed at its pad, nothing is commanded and the exit code says so.
"""
import argparse
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver                          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg   # noqa: E402
from measure_centre_gap import clearance_named                 # noqa: E402
import search_centre_geometry as SCG                           # noqa: E402
import grasp_frames as GF                                      # noqa: E402
import t1_task as T1                                           # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", type=float, default=600.0)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--slot", default="inner",
                    choices=("inner", "outer", "centre"))
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(6.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    rig = Rig(n, "t1", 1)
    SCG.scene(rig, T1.TABLE_TOP, T1.TABLE_NEAR_Y)
    fk = FK(n)
    z = round(T1.TABLE_TOP + T1.CUBE_M / 2.0, 4)
    row = round(T1.TABLE_NEAR_Y + T1.CUBE_M / 2.0, 4)

    sol = {}
    for i, arm in ((0, "left"), (1, "right")):
        px = T1.T1_PLANES[i][0]
        off = 0.0 if a.slot == "centre" else (
            -T1.SLOT_DX if (px > 0) == (a.slot == "inner") else T1.SLOT_DX)
        obj = [round(px + off, 4), row, z]
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid, _ = pad_mid_in_ee(fk, arm, home)
        w = GF.wrist_for(obj, T1.APPROACH[arm], pad_mid)
        j = n.solve_arm_joints(arm, list(w), as_msg(T1.APPROACH[arm]),
                               avoid=True, tries=8)
        if j is None:
            print("REFUSING: %s arm cannot reach its pad slot at x %+.3f"
                  % (arm, obj[0]))
            return 4
        c, who, seg, _p = clearance_named(fk.cli, n, arm, j)
        print("%-5s pad slot x %+7.3f  clearance %.4f m to %s at %s"
              % (arm, obj[0], c, who, seg))
        if c is None or c < a.floor:
            print("REFUSING: %s arm is inside the %.3f m floor at its pad."
                  % (arm, a.floor))
            return 5
        sol[arm] = [float(v) for v in j[:7]]

    pubs = {arm: n.create_publisher(
        JointTrajectory, "/%s_arm_controller/joint_trajectory" % arm, 10)
        for arm in ("left", "right")}
    n.spin(1.0)
    for arm, j in sol.items():
        t = JointTrajectory()
        t.joint_names = list(n.names(arm))
        p = JointTrajectoryPoint()
        p.positions = j
        p.time_from_start = Duration(sec=4)
        t.points = [p]
        for _ in range(4):
            pubs[arm].publish(t)
            n.spin(0.3)
    print("commanded; holding %.0f s" % a.hold)
    end = time.time() + a.hold
    while time.time() < end:
        n.spin(0.5)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
