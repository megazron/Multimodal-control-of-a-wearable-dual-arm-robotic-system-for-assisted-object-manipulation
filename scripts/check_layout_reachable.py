#!/usr/bin/env python3
"""Is every marked position in a task layout actually reachable? Walk to it.

    python3 scripts/check_layout_reachable.py

Seeds each 1 cm step from the previous solution, exactly as the follower
does, so this answers the teleop question -- can the arm GET there from home
-- not the pointwise one.
"""
import math, sys, time
import numpy as np, rclpy, rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

# (label, arm, x, y, z) -- as written in each protocol.md
LAYOUT = [
    ("T2 box lip grasp",   "right", -0.28, 0.34, 0.905),
    ("T2 block A pick",    "left",   0.18, 0.30, 0.873),
    ("T2 block B pick",    "left",   0.26, 0.30, 0.880),
    ("T2 block C pick",    "left",   0.34, 0.30, 0.888),
    ("T3 tray grip -x",    "right", -0.15, 0.34, 0.885),
    ("T3 tray grip +x",    "left",   0.15, 0.34, 0.885),
    ("T3 lift wp -x",      "right", -0.15, 0.34, 1.010),
    ("T3 lift wp +x",      "left",   0.15, 0.34, 1.010),
    ("T3 place -x",        "right", -0.15, 0.16, 0.885),
    ("T3 place +x",        "left",   0.15, 0.16, 0.885),
    ("T5 tool cradle",     "left",   0.30, 0.32, 0.884),
    ("T5 receive point",   "left",   0.16, 0.18, 1.020),
]


class L(Node):
    def __init__(self, arm):
        super().__init__('layout_%s' % arm)
        self.arm = arm; self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.buf = tf2_ros.Buffer(); self.lis = tf2_ros.TransformListener(self.buf, self)
        self.names = ['%s_joint_%d' % (arm, i) for i in range(1, 8)]

    def _js(self, m): self.js = dict(zip(m.name, m.position))

    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def ee(self):
        try:
            t = self.buf.lookup_transform('world', '%s_end_effector_link' % self.arm,
                                          rclpy.time.Time())
            v, r = t.transform.translation, t.transform.rotation
            p = Pose(); p.position.x, p.position.y, p.position.z = v.x, v.y, v.z
            p.orientation = r; return p
        except Exception: return None

    def solve(self, pose, seed, tries=7):
        for k in range(tries):
            s2 = list(seed)
            if k: s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request(); r = PositionIKRequest()
            r.group_name = '%s_arm' % self.arm
            rs = RobotState(); rs.joint_state.name = list(self.names)
            rs.joint_state.position = [float(x) for x in s2]
            r.robot_state = rs; r.avoid_collisions = True
            ps = PoseStamped(); ps.header.frame_id = 'world'; ps.pose = pose
            r.pose_stamped = ps; r.timeout.nanosec = 100_000_000
            req.ik_request = r
            fut = self.ik.call_async(req); t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.002)
            res = fut.result()
            if res and res.error_code.val == 1:
                idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
                return [res.solution.joint_state.position[idx[n]] for n in self.names]
        return None


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--raise-z', type=float, default=0.0,
                    help='raise every layout point by this many metres')
    ap.add_argument('--pull-y', type=float, default=0.0,
                    help='bring every point this much closer in y')
    args = ap.parse_args()
    rclpy.init()
    print('TASK LAYOUT REACHABILITY -- walked from home, 1 cm steps')
    print('raise_z=%+.2f m  pull_y=%+.2f m\n' % (args.raise_z, args.pull_y))
    print('  %-20s %-6s %8s %8s  %s' % ('point', 'arm', 'dist m', 'got m', 'verdict'))
    nodes = {}
    for arm in ('left', 'right'):
        n = L(arm); n.spin(2.0); n.ik.wait_for_service(timeout_sec=10.0)
        home = None
        for _ in range(20):
            home = n.ee()
            if home: break
            n.spin(0.5)
        nodes[arm] = (n, home, [n.js.get(k, 0.0) for k in n.names])
    bad = 0
    for label, arm, x, y, z in LAYOUT:
        z += args.raise_z
        y -= args.pull_y
        n, home, q_home = nodes[arm]
        if home is None:
            print('  %-20s %-6s   no tf2' % (label, arm)); continue
        p0 = np.array([home.position.x, home.position.y, home.position.z])
        tp = np.array([x, y, z]); d = float(np.linalg.norm(tp - p0))
        v = (tp - p0) / max(d, 1e-9)
        q = list(q_home); got = 0.0; k = 1; step = 0.01
        while k * step <= d + 1e-9:
            p = p0 + v * min(k * step, d)
            tgt = Pose(); tgt.orientation = home.orientation
            tgt.position.x, tgt.position.y, tgt.position.z = p
            sol = n.solve(tgt, q)
            if sol is None: break
            q = sol; got = min(k * step, d); k += 1
        ok = got >= d - 1e-6
        if not ok: bad += 1
        print('  %-20s %-6s %8.3f %8.3f  %s'
              % (label, arm, d, got, 'reachable' if ok else
                 'UNREACHABLE (stops %.0f%% of the way)' % (100 * got / d)))
    for n, _, _ in nodes.values(): n.destroy_node()
    print('\n  %d of %d layout points UNREACHABLE' % (bad, len(LAYOUT)))
    rclpy.shutdown(); return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
