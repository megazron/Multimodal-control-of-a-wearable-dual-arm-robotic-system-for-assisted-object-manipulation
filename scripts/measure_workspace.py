#!/usr/bin/env python3
"""Measure the CONTINUOUSLY reachable volume from home. No master needed.

    python3 scripts/measure_workspace.py --arm left
    python3 scripts/measure_workspace.py --arm both --step 0.01 --max 0.60

Sweeps outward from the home EE pose in 26 directions in `step` increments,
SEEDING EACH STEP FROM THE PREVIOUS SOLUTION exactly as ik_follower_node
does. That is the measurement that matters for teleop: pointwise IK asks
"could the arm get there from anywhere", which it can, while teleop asks "can
it get there from where it already is, without a branch switch".

Reports, per direction, the distance reached and the BINDING CONSTRAINT --
IK failure, collision, or the step guard -- plus isotropy (min/max) and the
usable fraction of the master's mapped range.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import RobotState, PositionIKRequest
from rclpy.node import Node
from sensor_msgs.msg import JointState

import tf2_ros

# THE ASSERTION THAT WAS MISSING.
# A previous run measured "reachability from home" while both arms were parked
# 155-166 deg away from it, left over from a capture. Every number it produced
# was meaningless, and nothing said so. The sweep now refuses to sample unless
# the arm is actually at home.
HOME_TOL_RAD = 0.05

# 26 directions: 6 face, 12 edge, 8 corner, on the unit sphere.
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


class WS(Node):
    def __init__(self, arm):
        super().__init__('measure_workspace_%s' % arm)
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
            rclpy.spin_once(self, timeout_sec=0.01)

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

    def seed_state(self, q):
        rs = RobotState()
        rs.joint_state.name = list(self.names)
        rs.joint_state.position = [float(x) for x in q]
        return rs

    def solve_with_retry(self, pose, seed, tries=7):
        """Solve exactly as ik_follower_node does: RE-SEED AND RETRY.

        Measured false-negative rate on a known-solvable target is 0.0-0.4%
        per call. A single call is therefore reliable, but a sweep is a CHAIN
        of ~70 calls and any one failure ends the direction early -- which is
        why 9 of 26 left directions showed min << median over 10 sweeps. The
        compounding, not the per-call rate, was the problem.

        The follower never suffers this because on failure it re-seeds the
        redundant DOF (joint_3) and retries up to `redundancy_samples` (6).
        Without the same retry, this tool was MORE brittle than the system it
        is supposed to characterise, and reported the difference as workspace.
        """
        q, why = self.solve(pose, seed)
        if q is not None:
            return q, ''
        for k in range(1, tries):
            s2 = list(seed)
            # fan out either side of the natural posture, as the follower does
            s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            q, why = self.solve(pose, s2)
            if q is not None:
                return q, ''
        return None, why

    def solve(self, pose, seed, timeout=2.0):
        """Returns (q, reason). reason is '' on success."""
        if not self.ik.wait_for_service(timeout_sec=timeout):
            return None, 'no /compute_ik'
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = '%s_arm' % self.arm
        r.robot_state = self.seed_state(seed)
        r.avoid_collisions = True
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose = pose
        r.pose_stamped = ps
        r.timeout.sec = 0
        r.timeout.nanosec = 100_000_000
        req.ik_request = r
        fut = self.ik.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
        res = fut.result()
        if res is None:
            return None, 'ik timeout'
        code = res.error_code.val
        if code != 1:
            return None, ('collision/IK (%d)' % code)
        idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
        q = [res.solution.joint_state.position[idx[n]] for n in self.names]
        return q, ''


def check_at_home(n, arm, tol=HOME_TOL_RAD):
    """Refuse unless the arm is AT its home joint vector.

    Returns (ok, worst_rad, detail). The measurement is defined as reach FROM
    HOME; starting anywhere else measures a different question and silently
    reports it under the same name.
    """
    import os
    import sys as _s
    _s.path.insert(0, os.path.expanduser('~/kortex_ws/config'))
    import home_positions
    home = list(home_positions.load_home_radians(arm))
    q = [n.js.get(k) for k in n.names]
    if any(v is None for v in q):
        return False, float('inf'), 'no /joint_states for %s' % arm
    d = [abs(a - b) for a, b in zip(q, home)]
    worst = max(d)
    j = d.index(worst)
    return (worst <= tol), worst, ('joint_%d is %.1f deg from home'
                                   % (j + 1, math.degrees(worst)))


def sweep_once(n, q_home, home_pose, p0, step, maxd, max_step_rad):
    """One full 26-direction sweep. Returns {name: (reach, reason)}."""
    out = {}
    for v in DIRS:
        q = list(q_home)
        reached = 0.0
        reason = 'max range'
        k = 1
        while k * step <= maxd + 1e-9:
            tgt = Pose()
            tgt.orientation = home_pose.orientation
            p = p0 + v * (k * step)
            tgt.position.x, tgt.position.y, tgt.position.z = p
            sol, why = n.solve_with_retry(tgt, q)
            if sol is None:
                reason = why
                break
            d = max(abs(np.array(sol) - np.array(q)))
            if d > max_step_rad:
                reason = 'step guard (%.2f rad > %.2f)' % (d, max_step_rad)
                break
            q = sol
            reached = k * step
            k += 1
        out[dname(v)] = (reached, reason)
    return out


def run(arm, step, maxd, max_step_rad, tol=HOME_TOL_RAD, repeats=1):
    n = WS(arm)
    n.spin(2.5)
    ok, worst, why = check_at_home(n, arm, tol)
    print('  %s: home check -- worst %.4f rad (%.2f deg), tol %.2f rad ... %s'
          % (arm, worst, math.degrees(worst) if math.isfinite(worst) else float('nan'),
             tol, 'OK' if ok else 'REFUSED'))
    if not ok:
        print('     %s' % why)
        print('     REFUSING to sample: this measures reach FROM HOME, and the '
              'arm is not at home. Relaunch the sim and stop anything that '
              'commands the arm (the IK followers).')
        n.destroy_node()
        return None
    home_pose = n.ee()
    if home_pose is None:
        print('  %s: no tf2 for %s_end_effector_link' % (arm, arm))
        n.destroy_node()
        return None
    q_home = [n.js.get(k, 0.0) for k in n.names]
    p0 = np.array([home_pose.position.x, home_pose.position.y,
                   home_pose.position.z])
    # REPEATS. TRAC-IK uses random restarts, so one sweep is an ESTIMATE, not
    # a measurement: two runs from an identical home pose gave left isotropy
    # 0.28 then 0.12 and worst-direction 0.140 m then 0.060 m. Anything
    # derived from a single sample -- the motion scale above all -- inherits
    # that factor-of-two.
    runs = []
    for r in range(repeats):
        t0 = time.monotonic()
        runs.append(sweep_once(n, q_home, home_pose, p0, step, maxd,
                               max_step_rad))
        print('     sweep %d/%d  (%.0f s)' % (r + 1, repeats,
                                              time.monotonic() - t0),
              flush=True)
    out = {}
    for name in runs[0]:
        vals = np.array([r[name][0] for r in runs], float)
        reasons = [r[name][1] for r in runs]
        # binding constraint = the most common reason at the MEDIAN outcome
        med = float(np.median(vals))
        why = max(set(reasons), key=reasons.count)
        out[name] = dict(median=med, min=float(vals.min()),
                         max=float(vals.max()),
                         q1=float(np.percentile(vals, 25)),
                         q3=float(np.percentile(vals, 75)),
                         iqr=float(np.percentile(vals, 75) -
                                   np.percentile(vals, 25)),
                         n=int(vals.size), reason=why,
                         truncated=bool(np.any(vals >= maxd - 1e-9)))
    n.destroy_node()
    return out


def report(arm, res, maxd):
    if not res:
        return None
    faces = ['front', 'back', 'left', 'right', 'up', 'down']
    order = faces + sorted((k for k in res if k not in faces),
                           key=lambda k: -res[k]['median'])
    print('\n  %s ARM   (n=%d sweeps per direction)'
          % (arm.upper(), res[order[0]]['n']))
    print('    %-10s %8s %8s %8s %8s   %s'
          % ('direction', 'median', 'min', 'max', 'IQR', 'binding constraint'))
    for k in order:
        r = res[k]
        trunc = '  TRUNCATED at sweep limit' if r['truncated'] else ''
        print('    %-10s %8.3f %8.3f %8.3f %8.3f   %s%s'
              % (k, r['median'], r['min'], r['max'], r['iqr'],
                 r['reason'], trunc))
    med = np.array([res[k]['median'] for k in res])
    iqr = np.array([res[k]['iqr'] for k in res])
    trunc = [k for k in res if res[k]['truncated']]
    print('    ISOTROPY on MEDIANS (min/max): %.2f' % (med.min() / med.max()))
    print('    worst direction (median): %.3f m     mean of medians: %.3f m'
          % (med.min(), med.mean()))
    print('    median IQR across directions: %.3f m   worst IQR: %.3f m'
          % (float(np.median(iqr)), float(iqr.max())))
    if trunc:
        print('    %d direction(s) STILL truncated at %.2f m: %s'
              % (len(trunc), maxd, ', '.join(sorted(trunc))))
        print('    -> isotropy is a LOWER BOUND; the true max is larger.')
    else:
        print('    no direction truncated -- every one bound on its own limit')
    return med


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', default='both', choices=['left', 'right', 'both'])
    ap.add_argument('--step', type=float, default=0.01)
    ap.add_argument('--max', type=float, default=0.50)
    ap.add_argument('--max-step-rad', type=float, default=0.35,
                    help="ik_follower_node's guard; a step larger than this "
                         "is slewed, not committed, so it bounds continuity")
    ap.add_argument('--home-tol-rad', type=float, default=HOME_TOL_RAD)
    ap.add_argument('--repeats', type=int, default=1,
                    help='sweeps per direction; use >=10 before deriving a scale')
    ap.add_argument('--json', default='', help='write full results here')
    ap.add_argument('--master-range', type=float, default=0.272,
                    help='measured master ball radius, for the usable fraction')
    a = ap.parse_args()
    rclpy.init()
    print('CONTINUOUS REACHABILITY FROM HOME')
    print('26 directions, %.0f cm steps, each seeded from the previous '
          'solution' % (100 * a.step))
    arms = ['left', 'right'] if a.arm == 'both' else [a.arm]
    allres = {}
    refused = []
    for arm in arms:
        res = run(arm, a.step, a.max, a.max_step_rad,
                  a.home_tol_rad, a.repeats)
        allres[arm] = res
        if res is None:
            refused.append(arm)
            continue
        dd = report(arm, res, a.max)
        if dd is not None:
            worst, med50 = float(dd.min()), float(np.median(dd))
            s_worst = worst / a.master_range
            s_med = med50 / a.master_range
            need = int((dd < med50).sum())
            print('    SCALE OPTIONS (master ball %.3f m):' % a.master_range)
            print('      to the WORST direction : %.2f  '
                  '(every direction in one sweep; shrinks everything to the '
                  'narrowest axis)' % s_worst)
            print('      to the MEDIAN direction: %.2f  '
                  '(%d of %d directions need a second sweep -- the clutch '
                  'indexes, proven unbounded)' % (s_med, need, dd.size))
    if a.json:
        import json as _j
        _j.dump(allres, open(a.json, 'w'), indent=1)
        print('\nfull results -> %s' % a.json)
    rclpy.shutdown()
    if refused:
        print('\nREFUSED for: %s -- no numbers produced, deliberately.'
              % ', '.join(refused))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
