#!/usr/bin/env python3
"""How often does IK refuse a target it can solve? -- the "arm sticks" bug.

    python3 scripts/measure_ik_stickiness.py --arm both

TRAC-IK uses random restarts, so the same target from the same seed is not
guaranteed to solve. In a reachability sweep that shows up as a rare short
direction and is only a statistics problem. IN LIVE TELEOPERATION IT IS A
SMOOTHNESS PROBLEM: the arm refuses to continue in a direction it managed a
moment ago, and to the operator that is the arm sticking.

METHOD. For each direction, walk out to 80% of the measured median reach --
a target known to be solvable -- keeping the seed the follower would have.
Then call /compute_ik for that SAME target from that SAME seed N times and
count how many fail. Any failure is by construction a false negative.

Repeated for several request timeouts, recording solve time, because the fix
has to fit the control budget: at 21 Hz a cycle is 47.6 ms and IK is only one
part of it.
"""
import argparse
import json
import math
import statistics
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros

DIRS = []
for dx in (-1, 0, 1):
    for dy in (-1, 0, 1):
        for dz in (-1, 0, 1):
            if (dx, dy, dz) != (0, 0, 0):
                v = np.array([dx, dy, dz], float)
                DIRS.append(v / np.linalg.norm(v))
NAMES = {(0, 1, 0): 'front', (0, -1, 0): 'back', (-1, 0, 0): 'left',
         (1, 0, 0): 'right', (0, 0, 1): 'up', (0, 0, -1): 'down'}


def dname(v):
    key = tuple(int(round(x)) for x in v * (1 / max(abs(v).max(), 1e-9)))
    return NAMES.get(key, "%+d%+d%+d" % key)


class IKProbe(Node):
    def __init__(self, arm):
        super().__init__('ik_stickiness_%s' % arm)
        self.arm = arm
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.names = ['%s_joint_%d' % (arm, i) for i in range(1, 8)]

    def _js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def ee(self):
        try:
            t = self.buf.lookup_transform('world',
                                          '%s_end_effector_link' % self.arm,
                                          rclpy.time.Time())
            v, r = t.transform.translation, t.transform.rotation
            p = Pose()
            p.position.x, p.position.y, p.position.z = v.x, v.y, v.z
            p.orientation = r
            return p
        except Exception:                                        # noqa: BLE001
            return None

    def solve(self, pose, seed, timeout_s):
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = '%s_arm' % self.arm
        rs = RobotState()
        rs.joint_state.name = list(self.names)
        rs.joint_state.position = [float(x) for x in seed]
        r.robot_state = rs
        r.avoid_collisions = True
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose = pose
        r.pose_stamped = ps
        r.timeout.sec = int(timeout_s)
        r.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1e9)
        req.ik_request = r
        t0 = time.monotonic()
        fut = self.cli.call_async(req)
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.002)
        dt = (time.monotonic() - t0) * 1000.0
        res = fut.result()
        if res is None:
            return None, dt
        if res.error_code.val != 1:
            return None, dt
        idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
        return [res.solution.joint_state.position[idx[n]]
                for n in self.names], dt


def walk_to(n, home_pose, p0, v, dist, step, seed):
    """Reproduce the follower's seeding: walk out one cm at a time."""
    q = list(seed)
    k = 1
    while k * step <= dist + 1e-9:
        tgt = Pose()
        tgt.orientation = home_pose.orientation
        p = p0 + v * (k * step)
        tgt.position.x, tgt.position.y, tgt.position.z = p
        sol, _ = n.solve(tgt, q, 0.2)
        if sol is None:
            return None, None
        q = sol
        k += 1
    tgt = Pose()
    tgt.orientation = home_pose.orientation
    p = p0 + v * dist
    tgt.position.x, tgt.position.y, tgt.position.z = p
    return tgt, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', default='both', choices=['left', 'right', 'both'])
    ap.add_argument('--repeats', type=int, default=30)
    ap.add_argument('--timeouts', default='0.005,0.02,0.05,0.1')
    ap.add_argument('--baseline',
                    default='recordings/baselines/workspace_n10_20260806.json')
    ap.add_argument('--fracs', default='0.8',
                    help='fractions of median reach to probe')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    tos = [float(x) for x in a.timeouts.split(',')]
    base = json.load(open(a.baseline))
    rclpy.init()
    print('IK STICKINESS -- false negatives on a KNOWN-SOLVABLE target')
    print('%d calls per direction per timeout, seed held fixed\n' % a.repeats)
    out = {}
    for arm in (['left', 'right'] if a.arm == 'both' else [a.arm]):
        n = IKProbe(arm)
        n.spin(2.5)
        n.cli.wait_for_service(timeout_sec=10.0)
        home_pose = n.ee()
        q_home = [n.js.get(k, 0.0) for k in n.names]
        if home_pose is None:
            print('  %s: no tf2' % arm)
            continue
        p0 = np.array([home_pose.position.x, home_pose.position.y,
                       home_pose.position.z])
        print('  === %s ARM ===' % arm.upper())
        per_to = {}
        for to, frac in [(t, f) for t in tos for f in
                         [float(x) for x in a.fracs.split(',')]]:
            fails = {}
            times = []
            for v in DIRS:
                nm = dname(v)
                med = base[arm][nm]['median']
                d = frac * med
                if d < 0.02:
                    continue
                tgt, seed = walk_to(n, home_pose, p0, v, d, 0.01, q_home)
                if tgt is None:
                    continue
                f = 0
                for _ in range(a.repeats):
                    sol, dt = n.solve(tgt, seed, to)
                    times.append(dt)
                    if sol is None:
                        f += 1
                fails[nm] = f / a.repeats
            if not fails:
                continue
            allf = float(np.mean(list(fails.values())))
            worst = sorted(fails.items(), key=lambda kv: -kv[1])[:4]
            per_to['%g@%g' % (to, frac)] = dict(
                mean_fail=allf, per_dir=fails,
                t_med=float(np.median(times)),
                t_p95=float(np.percentile(times, 95)))
            print('    timeout %5.0f ms  reach %4.0f%% of median  ->  '
                  'false negatives %5.2f%%   solve med %5.1f ms'
                  % (1000 * to, 100 * frac, 100 * allf, np.median(times)))
            nz = [(k, x) for k, x in worst if x > 0]
            if nz:
                print('       worst directions: %s'
                      % ', '.join('%s %.0f%%' % (k, 100 * x) for k, x in nz))
            else:
                print('       no direction failed')
        out[arm] = per_to
        n.destroy_node()
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)
        print('\n  -> %s' % a.json)
    print('\n  CONTROL BUDGET: at 21 Hz a cycle is 47.6 ms; at 50 Hz it is '
          '20.0 ms.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
