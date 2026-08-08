#!/usr/bin/env python3
"""Is every task actually performable in sim? Per target, per arm, with reasons.

    python3 scripts/verify_task_scenes.py                 # all five tasks
    python3 scripts/verify_task_scenes.py --task t2
    python3 scripts/verify_task_scenes.py --sweep-table    # what height works

Needs a running sim (move_group) and the arms AT HOME. The home assertion is
not optional: this measures reach FROM HOME, and a run taken with the arms
parked elsewhere answers a different question under the same name -- which
has already invalidated one reachability run and half of one front-reach run
in this project.

Each target is solved with `/compute_ik` exactly as `ik_follower_node` does:
collision-aware, seeded from the current state, with the joint_3 redundancy
re-seed and retry. A target that fails is then re-solved with collisions OFF
and with yaw sampled, which separates three very different findings:

    fails collision-aware, succeeds collision-free -> the SCENE blocks it
    fails both, succeeds with yaw free             -> ORIENTATION binds
    fails everything                               -> genuinely out of reach

Only the last is a statement about the arm. The first is a statement about
where the table was put, and it is fixable with a rule and some tape.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.time
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "config"))
BIM = os.path.join(ROOT, "src", "srl_experiments", "experiments", "bimanual")
HOME_TOL_RAD = 0.05
TASKS = ["t1", "t2", "t3", "t4", "t5"]


def layout_dir(task):
    for d in sorted(os.listdir(BIM)):
        if d.startswith(task + "_"):
            return os.path.join(BIM, d)
    raise FileNotFoundError(task)


class Solver(Node):
    def __init__(self):
        super().__init__("verify_task_scenes")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def names(self, arm):
        return ["%s_joint_%d" % (arm, i) for i in range(1, 8)]

    def home_ok(self, arm):
        import home_positions as hp
        tgt = hp.load_home_radians(arm)
        cur = [self.js.get(k, 0.0) for k in self.names(arm)]
        off = [abs((cur[i] - tgt[i] + math.pi) % (2 * math.pi) - math.pi)
               for i in range(7)]
        return max(off), off.index(max(off)) + 1

    def ee_quat(self, arm):
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
            return t.transform.rotation
        except Exception:                                    # noqa: BLE001
            return None

    def solve(self, arm, xyz, quat, avoid=True, tries=7):
        seed = [self.js.get(k, 0.0) for k in self.names(arm)]
        for k in range(tries):
            s2 = list(seed)
            if k:
                s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request()
            r = PositionIKRequest()
            r.group_name = "%s_arm" % arm
            rs = RobotState()
            rs.joint_state.name = self.names(arm)
            rs.joint_state.position = [float(x) for x in s2]
            r.robot_state = rs
            r.avoid_collisions = avoid
            ps = PoseStamped()
            ps.header.frame_id = "world"
            p = Pose()
            p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
            p.orientation = quat
            ps.pose = p
            r.pose_stamped = ps
            r.timeout.nanosec = 100_000_000
            req.ik_request = r
            fut = self.ik.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.002)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                return True
        return False

    def solve_yaw_free(self, arm, xyz, quat, avoid=True):
        for yaw in np.linspace(-math.pi, math.pi, 8, endpoint=False):
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            q = type(quat)()
            q.x = quat.x * cy - quat.y * sy
            q.y = quat.x * sy + quat.y * cy
            q.z = quat.z * cy + quat.w * sy
            q.w = quat.w * cy - quat.z * sy
            if self.solve(arm, xyz, q, avoid=avoid, tries=3):
                return True
        return False


def classify(n, arm, xyz, quat):
    if n.solve(arm, xyz, quat, avoid=True):
        return "REACHABLE", ""
    if n.solve(arm, xyz, quat, avoid=False):
        return "BLOCKED", "the SCENE blocks it (table/object collision)"
    if n.solve_yaw_free(arm, xyz, quat, avoid=True):
        return "ORIENT", "reachable only with yaw free -- orientation binds"
    if n.solve_yaw_free(arm, xyz, quat, avoid=False):
        return "ORIENT+SCENE", "yaw free AND collisions off"
    return "UNREACHABLE", "no configuration reaches it at all"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="all")
    ap.add_argument("--sweep-table", action="store_true")
    ap.add_argument("--skip-home-check", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Solver()
    n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=15.0):
        print("no /compute_ik -- start the sim first")
        return 2

    print("TASK SCENE REACHABILITY")
    ok_home = True
    for arm in ("left", "right"):
        worst, j = n.home_ok(arm)
        flag = "ok" if worst <= HOME_TOL_RAD else "NOT AT HOME"
        print("  %-5s home offset %.4f rad (%.2f deg) on joint_%d  [%s]"
              % (arm, worst, math.degrees(worst), j, flag))
        if worst > HOME_TOL_RAD:
            ok_home = False
    if not ok_home and not a.skip_home_check:
        print("\n  REFUSING: this measures reach FROM HOME. Restart the sim "
              "with no followers running, or pass --skip-home-check and "
              "treat the numbers as invalid.")
        return 3

    quats = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quats.values()):
        print("  no tf2 for an end effector")
        return 2

    tasks = TASKS if a.task == "all" else [a.task]
    grand = {}
    for t in tasks:
        d = layout_dir(t)
        spec = yaml.safe_load(open(os.path.join(d, "layout.yaml")))
        print("\n  === %s ===  table top z=%.3f"
              % (os.path.basename(d), spec["table"]["z"]))
        res = {}
        for name, tg in spec["targets"].items():
            arm = tg["arm"]
            verdict, why = classify(n, arm, tg["xyz"], quats[arm])
            res[name] = verdict
            print("    %-18s %-5s %-14s %s"
                  % (name, arm, verdict, why))
        good = sum(1 for v in res.values() if v == "REACHABLE")
        grand[t] = (good, len(res))
        print("    -> %d/%d targets reachable  %s"
              % (good, len(res),
                 "TASK PERFORMABLE" if good == len(res)
                 else "TASK NOT PERFORMABLE AS LAID OUT"))

    if a.sweep_table:
        print("\n  === TABLE HEIGHT SWEEP === (t2 targets, offset as a block)")
        spec = yaml.safe_load(open(os.path.join(layout_dir("t2"),
                                                "layout.yaml")))
        base = spec["table"]["z"]
        for dz in (0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
            good = 0
            for name, tg in spec["targets"].items():
                xyz = list(tg["xyz"])
                xyz[2] += dz
                v, _ = classify(n, tg["arm"], xyz, quats[tg["arm"]])
                good += (v == "REACHABLE")
            print("    table z %.3f (+%.2f)  ->  %d/%d reachable"
                  % (base + dz, dz, good, len(spec["targets"])))

    print("\n  SUMMARY")
    for t, (g, tot) in grand.items():
        print("    %-4s %d/%d" % (t, g, tot))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
