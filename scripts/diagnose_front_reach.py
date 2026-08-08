#!/usr/bin/env python3
"""WHY does the front direction stop so early? Name the binding constraint.

    python3 scripts/diagnose_front_reach.py --arm both

`collision/IK (-31)` is NO_IK_SOLUTION, which conflates three very different
causes, and they have three different fixes:

  WEARER COLLISION   the pose is reachable but the arm would hit the person.
                     Fixed by geometry: mount, layout or posture.
  JOINT LIMIT        a joint runs out of travel. Fixed only by a different
                     arm or a different approach.
  IK INFEASIBLE      no configuration reaches the pose at all. Reach limit.

Disambiguated by re-solving each failed step with avoid_collisions OFF:

  fails with collisions ON, succeeds with them OFF   -> WEARER COLLISION,
        and /check_state_validity then names the exact link pair.
  fails BOTH ways                                    -> IK INFEASIBLE or a
        JOINT LIMIT; the returned joint vector (if any) is checked against
        the URDF limits to separate them.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "config"))

DIRS = {'front': (0, 1, 0), 'back': (0, -1, 0), 'up': (0, 0, 1),
        'down': (0, 0, -1), 'left': (-1, 0, 0), 'right': (1, 0, 0)}

# Gen3 7-DOF limits (rad). Continuous joints 1/3/5/7 are unlimited.
LIMITED = {2: 2.41, 4: 2.66, 6: 2.23}
HOME_TOL_RAD = 0.05


class Diag(Node):
    def __init__(self, arm):
        super().__init__('diagnose_front_%s' % arm)
        self.arm = arm
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self._js, 20)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.sv = self.create_client(GetStateValidity, '/check_state_validity')
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

    def solve(self, pose, seed, avoid=True, tries=7):
        for k in range(tries):
            s2 = list(seed)
            if k:
                s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request()
            r = PositionIKRequest()
            r.group_name = '%s_arm' % self.arm
            rs = RobotState()
            rs.joint_state.name = list(self.names)
            rs.joint_state.position = [float(x) for x in s2]
            r.robot_state = rs
            r.avoid_collisions = avoid
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose = pose
            r.pose_stamped = ps
            r.timeout.nanosec = 100_000_000
            req.ik_request = r
            fut = self.ik.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.002)
            res = fut.result()
            if res is None:
                continue
            if res.error_code.val == 1:
                idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
                return [res.solution.joint_state.position[idx[n]]
                        for n in self.names]
        return None

    def contacts(self, q):
        req = GetStateValidity.Request()
        rs = RobotState()
        rs.joint_state.name = list(self.names)
        rs.joint_state.position = [float(x) for x in q]
        req.robot_state = rs
        req.group_name = '%s_arm' % self.arm
        fut = self.sv.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.002)
        res = fut.result()
        if res is None:
            return None, []
        return res.valid, [(c.contact_body_1, c.contact_body_2, c.depth)
                           for c in res.contacts]


def diagnose(arm, dirs, step, maxd):
    n = Diag(arm)
    n.spin(2.5)
    n.ik.wait_for_service(timeout_sec=10.0)
    n.sv.wait_for_service(timeout_sec=10.0)
    home = None
    for _ in range(20):
        home = n.ee()
        if home is not None:
            break
        n.spin(0.5)
    if home is None:
        print('\n  === %s ARM ===  no tf2 for %s_end_effector_link'
              % (arm.upper(), arm))
        n.destroy_node()
        return
    q_home = [n.js.get(k, 0.0) for k in n.names]
    # REFUSE UNLESS THE ARM IS AT HOME.
    #
    # This tool measures "how far can the arm get FROM HOME", and it takes
    # its start pose from wherever the arm happens to be. On 2026-08-07 an
    # indexing test had left the left arm displaced and this reported
    # 0.000 m front and down -- a wrong answer under the right name. The
    # right arm, which WAS at home, reproduced its baseline exactly, which is
    # how the discrepancy was caught. measure_workspace.py has had this
    # assertion since the same failure invalidated a reachability run;
    # diagnose_front_reach.py did not.
    try:
        import home_positions as hp
        target = hp.load_home_radians(arm)
    except Exception:                                        # noqa: BLE001
        target = None
    if target is not None:
        off = [abs((q_home[i] - target[i] + math.pi) % (2 * math.pi) - math.pi)
               for i in range(7)]
        worst = max(off)
        print("  home check: worst joint offset %.4f rad (%.2f deg)"
              % (worst, math.degrees(worst)))
        if worst > HOME_TOL_RAD:
            print("  REFUSING: %s arm is %.3f rad (%.1f deg) from home on "
                  "joint_%d, over the %.2f rad tolerance. This tool measures "
                  "reach FROM HOME; from anywhere else it answers a "
                  "different question under the same name. Restart the sim "
                  "with no followers running, or re-home."
                  % (arm, worst, math.degrees(worst),
                     off.index(worst) + 1, HOME_TOL_RAD))
            n.destroy_node()
            return
    p0 = np.array([home.position.x, home.position.y, home.position.z])
    print('\n  === %s ARM ===' % arm.upper())
    for name in dirs:
        v = np.array(DIRS[name], float)
        q = list(q_home)
        reached = 0.0
        k = 1
        while k * step <= maxd + 1e-9:
            tgt = Pose()
            tgt.orientation = home.orientation
            p = p0 + v * (k * step)
            tgt.position.x, tgt.position.y, tgt.position.z = p
            sol = n.solve(tgt, q, avoid=True)
            if sol is None:
                break
            q = sol
            reached = k * step
            k += 1
        # The step that FAILED. Re-solve it with collisions off.
        tgt = Pose()
        tgt.orientation = home.orientation
        p = p0 + v * (reached + step)
        tgt.position.x, tgt.position.y, tgt.position.z = p
        free = n.solve(tgt, q, avoid=False)
        if free is not None:
            valid, cts = n.contacts(free)
            pairs = ', '.join('%s <-> %s (%.3f m deep)' % c for c in cts[:3])
            cause = 'WEARER COLLISION'
            detail = pairs or 'reported invalid, no contact list'
        else:
            # No solution even ignoring the wearer. Reach or joint limit?
            lim_hits = []
            for j, lim in LIMITED.items():
                if abs(q[j - 1]) > 0.95 * lim:
                    lim_hits.append('joint_%d at %.2f rad of +/-%.2f'
                                    % (j, q[j - 1], lim))
            span = float(np.linalg.norm(p - p0))
            cause = 'JOINT LIMIT' if lim_hits else 'IK INFEASIBLE'
            detail = ('; '.join(lim_hits) if lim_hits
                      else 'no configuration reaches it (target %.3f m from '
                           'home EE)' % span)
        print('    %-6s reached %.3f m  ->  %-18s %s'
              % (name, reached, cause, detail))
        # Which joints are nearest their limits at the stopping pose?
        near = sorted(((abs(q[j - 1]) / lim, j) for j, lim in LIMITED.items()),
                      reverse=True)[:2]
        print('           limited joints at the stop: %s'
              % ', '.join('joint_%d %.0f%% of limit' % (j, 100 * f)
                          for f, j in near))
        # IS IT THE POSITION OR THE ORIENTATION? The sweep holds the wrist at
        # the HOME orientation because that is what orientation_mode:fixed
        # does in teleop. Re-solving the same POSITION with yaw sampled tells
        # us whether the arm cannot get there at all, or only cannot get
        # there while holding that wrist angle.
        best = reached
        for extra in range(1, 40):
            p2 = p0 + v * (reached + extra * step)
            ok = False
            for yaw in np.linspace(-math.pi, math.pi, 8, endpoint=False):
                t2 = Pose()
                cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                t2.orientation.x = home.orientation.x * cy - home.orientation.y * sy
                t2.orientation.y = home.orientation.x * sy + home.orientation.y * cy
                t2.orientation.z = home.orientation.z * cy + home.orientation.w * sy
                t2.orientation.w = home.orientation.w * cy - home.orientation.z * sy
                t2.position.x, t2.position.y, t2.position.z = p2
                if n.solve(t2, q, avoid=True, tries=3) is not None:
                    ok = True
                    break
            if not ok:
                break
            best = reached + extra * step
        gain = best - reached
        print('           with YAW FREE it reaches %.3f m  (%+.3f m, %s)'
              % (best, gain,
                 'ORIENTATION is the binding constraint' if gain > 0.05
                 else 'position limit is genuine'))
    n.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', default='both', choices=['left', 'right', 'both'])
    ap.add_argument('--dirs', default='front,back,up,down,left,right')
    ap.add_argument('--step', type=float, default=0.01)
    ap.add_argument('--max', type=float, default=0.90)
    a = ap.parse_args()
    rclpy.init()
    print('FRONT-REACH DIAGNOSIS -- naming the constraint that binds')
    for arm in (['left', 'right'] if a.arm == 'both' else [a.arm]):
        diagnose(arm, a.dirs.split(','), a.step, a.max)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
