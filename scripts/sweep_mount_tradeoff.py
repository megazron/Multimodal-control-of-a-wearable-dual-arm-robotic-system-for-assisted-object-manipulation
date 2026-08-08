#!/usr/bin/env python3
"""Does mount rotation trade BACK reach for FRONT reach? Report only.

    python3 scripts/sweep_mount_tradeoff.py --arm both

NOTHING IS APPLIED. A mount change invalidates P_HOME, the workspace anchor
and every clearance figure, so this reports the tradeoff curve and stops.

METHOD, and why no URDF edit is needed: rotating the arm's base by delta is
exactly equivalent to rotating the TARGETS by -delta about the mount origin.
So one running move_group can evaluate every candidate, which is the same
trick used for the earlier mount-angle study.
"""
import argparse, math, sys, time
import numpy as np, rclpy, rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

DIRS = {'front': (0, 1, 0), 'back': (0, -1, 0), 'up': (0, 0, 1), 'down': (0, 0, -1)}
MOUNT = {'left': np.array([0.20, -0.32, 1.25]),
         'right': np.array([-0.20, -0.32, 1.25])}


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class M(Node):
    def __init__(self, arm):
        super().__init__('mount_sweep_%s' % arm)
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

    def solve(self, pose, seed, tries=5):
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


def reach(n, home, p0, q_home, v, delta, mount, step=0.02, maxd=0.90):
    """Reach along v with the base rotated by `delta` (targets by -delta)."""
    R = rot_x(-delta); q = list(q_home); best = 0.0; k = 1
    while k * step <= maxd + 1e-9:
        p = p0 + v * (k * step)
        p = mount + R @ (p - mount)
        tgt = Pose(); tgt.orientation = home.orientation
        tgt.position.x, tgt.position.y, tgt.position.z = p
        sol = n.solve(tgt, q)
        if sol is None: break
        q = sol; best = k * step; k += 1
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', default='both', choices=['left', 'right', 'both'])
    ap.add_argument('--deltas', default='-40,-20,-10,0,10,20,40')
    a = ap.parse_args()
    deltas = [math.radians(float(x)) for x in a.deltas.split(',')]
    rclpy.init()
    print('MOUNT ROTATION TRADEOFF  (pitch about the mount x-axis)')
    print('NOTHING IS APPLIED -- report only. A mount change invalidates')
    print('P_HOME, the workspace anchor and every clearance figure.\n')
    for arm in (['left', 'right'] if a.arm == 'both' else [a.arm]):
        n = M(arm); n.spin(2.5); n.ik.wait_for_service(timeout_sec=10.0)
        home = None
        for _ in range(20):
            home = n.ee()
            if home: break
            n.spin(0.5)
        if home is None:
            print('  %s: no tf2' % arm); continue
        q_home = [n.js.get(k, 0.0) for k in n.names]
        p0 = np.array([home.position.x, home.position.y, home.position.z])
        print('  === %s ARM ===' % arm.upper())
        print('    %8s %8s %8s %8s %8s   %s'
              % ('delta', 'front', 'back', 'up', 'down', 'front+back'))
        for d in deltas:
            r = {k: reach(n, home, p0, q_home, np.array(v, float), d, MOUNT[arm])
                 for k, v in DIRS.items()}
            print('    %7.0f deg %8.3f %8.3f %8.3f %8.3f   %8.3f'
                  % (math.degrees(d), r['front'], r['back'], r['up'],
                     r['down'], r['front'] + r['back']))
        n.destroy_node()
    rclpy.shutdown(); return 0


if __name__ == '__main__':
    sys.exit(main())
