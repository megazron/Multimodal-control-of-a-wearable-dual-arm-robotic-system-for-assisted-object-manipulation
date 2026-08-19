#!/usr/bin/env python3
"""
calibrate_operator_yaw.py -- measure where the operator is standing, in one angle.

THE PROBLEM IT SOLVES. The controller poses arrive in the WebXR reference
space, whose heading is fixed to the room when the session starts -- i.e. by
wherever the headset happened to be pointing. Nothing relates that to the
robot's world frame. `vr_pose_mapper.align_yaw_deg` is the one number that
does, and until now it was a guess.

Guessing is specifically dangerous here. The operator sits ACROSS THE ROOM
FACING THE WEARER, and the intuitive correction for that -- "they're facing
me, so mirror it" -- is a REFLECTION. A reflection mirrors every orientation
as well as every position, so the gripper rolls the wrong way while the
positions look right. The mapper cannot express a reflection: the alignment is
one yaw angle. So the only question left is which angle, and this measures it.

HOW. The operator moves their hand along ONE direction that is unambiguous to
both people in the room: STRAIGHT TOWARDS THE ROBOT. In the robot's world
frame that direction is known -- the wearer faces the operator, so
operator-to-wearer is the wearer's BACKWARD, world -y. One known direction is
exactly enough to solve one unknown angle.

A second motion (the operator's own right) is recorded as a CHECK, not as
input: after the solved yaw is applied, the two motions must still be
perpendicular and horizontal. If they are not, the operator's frame is not a
pure yaw relative to the robot's -- they are standing on a slope, or the
headset is tilted on its shelf -- and the script says so instead of returning
a confident wrong number.

    python3 scripts/calibrate_operator_yaw.py
    python3 scripts/calibrate_operator_yaw.py --apply     # set it live
    python3 scripts/calibrate_operator_yaw.py --selftest  # no ROS, no headset

NOTHING MOVES. This reads /vr/controller_pose_<hand> directly and never
engages the clutch, so no arm is commanded during calibration.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, 'recordings', 'baselines', 'operator_yaw.json')

# The direction the operator is asked to move in, expressed in the ROBOT's
# world frame (x = wearer's right, y = wearer's forward, z = up).
# The wearer faces the operator, so moving from the operator towards the
# wearer travels along the wearer's -y.
TOWARDS_ROBOT_WORLD = np.array([0.0, -1.0, 0.0])


# ------------------------------------------------------------------ solving
def solve_yaw_deg(measured_xy, target_xy=TOWARDS_ROBOT_WORLD[:2]):
    """Yaw (deg) that rotates `measured` onto `target`, both horizontal.

    Signed, and about world +z, matching vr_pose_mapper.yaw_matrix.
    """
    m = np.asarray(measured_xy, float)
    t = np.asarray(target_xy, float)
    nm, nt = np.linalg.norm(m), np.linalg.norm(t)
    if nm < 1e-9 or nt < 1e-9:
        return None
    a = math.atan2(m[1], m[0])
    b = math.atan2(t[1], t[0])
    return math.degrees((b - a + math.pi) % (2 * math.pi) - math.pi)


def excursion(points):
    """Largest displacement from the start, not last-minus-first.

    The operator returns their hand, and end-to-end would then read zero --
    which would silently calibrate against noise.
    """
    P = np.asarray(points, float)
    if len(P) < 3:
        return None
    d = P - P[0]
    i = int(np.argmax(np.linalg.norm(d, axis=1)))
    return d[i]


def assess(d_towards, d_right, yaw):
    """Everything that says whether the number is trustworthy."""
    from math import degrees
    out = {}
    horiz = float(np.linalg.norm(d_towards[:2]))
    out['travel_m'] = round(float(np.linalg.norm(d_towards)), 4)
    out['horizontal_travel_m'] = round(horiz, 4)
    out['vertical_component_m'] = round(float(d_towards[2]), 4)
    # A motion that is mostly vertical carries almost no heading information.
    out['tilt_deg'] = round(degrees(math.atan2(abs(d_towards[2]), max(horiz, 1e-9))), 2)
    if d_right is not None:
        R = np.array([[math.cos(math.radians(yaw)), -math.sin(math.radians(yaw)), 0],
                      [math.sin(math.radians(yaw)), math.cos(math.radians(yaw)), 0],
                      [0, 0, 1.0]])
        a = R @ d_towards
        b = R @ d_right
        na, nb = np.linalg.norm(a[:2]), np.linalg.norm(b[:2])
        if na > 1e-6 and nb > 1e-6:
            cosang = float(np.dot(a[:2], b[:2]) / (na * nb))
            out['angle_between_motions_deg'] = round(
                degrees(math.acos(max(-1.0, min(1.0, cosang)))), 2)
        out['right_travel_m'] = round(float(np.linalg.norm(d_right)), 4)
    return out


def verdict(a):
    bad = []
    if a.get('horizontal_travel_m', 0) < 0.10:
        bad.append('the hand moved less than 100 mm horizontally -- too little '
                   'to fix a heading; move further')
    if a.get('tilt_deg', 0) > 35:
        bad.append('the motion was %.0f deg off horizontal; a mostly-vertical '
                   'motion carries almost no heading information'
                   % a['tilt_deg'])
    ab = a.get('angle_between_motions_deg')
    if ab is not None and abs(ab - 90.0) > 25.0:
        bad.append('the two motions are %.0f deg apart, not ~90 -- the '
                   'operator frame is not a pure yaw relative to the robot '
                   '(tilted headset, or one of the motions was not the one '
                   'asked for)' % ab)
    return bad


# ----------------------------------------------------------------- selftest
def selftest():
    fails = []

    def check(name, cond, detail=''):
        print('  %-56s %s %s' % (name, 'PASS' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    print('\nsolve_yaw_deg -- ground truth is constructed geometry')
    # operator already aligned: hand towards robot reads as world -y
    check('already aligned -> 0 deg',
          abs(solve_yaw_deg([0.0, -1.0])) < 1e-9,
          '(%.3f)' % solve_yaw_deg([0.0, -1.0]))
    # operator facing the wearer: their "towards robot" reads as world +y
    check('facing the wearer -> 180 deg',
          abs(abs(solve_yaw_deg([0.0, 1.0])) - 180.0) < 1e-9,
          '(%.3f)' % solve_yaw_deg([0.0, 1.0]))
    check('operator turned 90 deg -> 90',
          abs(solve_yaw_deg([1.0, 0.0]) - (-90.0)) < 1e-9 or
          abs(solve_yaw_deg([1.0, 0.0]) - 270.0) < 1e-9,
          '(%.3f)' % solve_yaw_deg([1.0, 0.0]))
    # round trip: solving then rotating must land on the target
    for ang in (0, 37, 180, -122):
        r = math.radians(ang)
        meas = [math.sin(r) * -1.0 * 0 + math.cos(r) * 0 - math.sin(r) * -1.0,
                math.sin(r) * 0 + math.cos(r) * -1.0]
        y = solve_yaw_deg(meas)
        M = np.array([[math.cos(math.radians(y)), -math.sin(math.radians(y))],
                      [math.sin(math.radians(y)), math.cos(math.radians(y))]])
        got = M @ np.array(meas)
        check('round trip at %+5.1f deg lands on -y' % ang,
              np.allclose(got / np.linalg.norm(got), [0, -1], atol=1e-9),
              str(np.round(got, 4)))
    check('a zero-length motion returns None, not 0',
          solve_yaw_deg([0.0, 0.0]) is None)

    print('\nexcursion')
    there_back = [[0.3 * i / 25, 0, 0] for i in range(26)] + \
                 [[0.3 * (25 - i) / 25, 0, 0] for i in range(26)]
    e = excursion(there_back)
    check('a hand that returns still reports 0.300 m, not 0',
          abs(e[0] - 0.3) < 1e-9, str(np.round(e, 4)))

    print('\nverdict -- must REFUSE bad input')
    good = assess(np.array([0.0, 0.42, 0.02]), np.array([0.38, 0.0, 0.01]), 180.0)
    check('a clean pair is accepted', verdict(good) == [], str(verdict(good)))
    tiny = assess(np.array([0.0, 0.03, 0.0]), np.array([0.03, 0.0, 0.0]), 180.0)
    check('a 30 mm motion is REFUSED', len(verdict(tiny)) > 0)
    vert = assess(np.array([0.0, 0.05, 0.40]), np.array([0.38, 0.0, 0.0]), 180.0)
    check('a mostly-vertical motion is REFUSED', len(verdict(vert)) > 0)
    par = assess(np.array([0.0, 0.42, 0.0]), np.array([0.0, 0.40, 0.0]), 180.0)
    check('two PARALLEL motions are REFUSED', len(verdict(par)) > 0,
          str(par.get('angle_between_motions_deg')))

    print('\n' + '-' * 68)
    if fails:
        print('SELFTEST FAILED: %s' % ', '.join(fails))
        return 1
    print('SELFTEST PASS: the solver answers constructed geometry and refuses '
          'motions that cannot determine a heading.')
    return 0


# ------------------------------------------------------------------- live
def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy

    hand = args.hand

    class Cal(Node):
        def __init__(self):
            super().__init__('calibrate_operator_yaw')
            self.rec = False
            self.pts = []
            self.a_edge = threading.Event()
            self._prev = False
            self._block = 0.0
            self.lock = threading.Lock()
            self.create_subscription(
                PoseStamped, '/vr/controller_pose_%s' % hand, self._pose, 200)
            self.create_subscription(
                Joy, '/vr/controller_joy_%s' % hand, self._joy, 200)

        def _pose(self, m):
            with self.lock:
                if self.rec:
                    self.pts.append([m.pose.position.x, m.pose.position.y,
                                     m.pose.position.z])

        def _joy(self, m):
            a = bool(m.buttons and m.buttons[0])
            if a and not self._prev and time.monotonic() > self._block:
                self.a_edge.set()
            self._prev = a

        def capture(self, prompt, timeout=120.0):
            print('\n  >>> %s' % prompt)
            print('      press A on the %s controller when done' % hand,
                  flush=True)
            with self.lock:
                self.pts = []
                self.rec = True
            self.a_edge.clear()
            self._block = time.monotonic() + 1.0
            t0 = time.monotonic()
            while not self.a_edge.is_set():
                if time.monotonic() - t0 > timeout:
                    print('      (timed out)')
                    break
                time.sleep(0.1)
            with self.lock:
                self.rec = False
                pts = list(self.pts)
            self._block = time.monotonic() + 1.0
            print('      %d samples' % len(pts))
            return pts

    rclpy.init()
    n = Cal()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)

    print('=' * 72)
    print('OPERATOR YAW CALIBRATION -- nothing moves, the clutch is not used.')
    print('=' * 72)

    pts_to = n.capture(
        'Hold the controller out and move your hand about 40 cm STRAIGHT '
        'TOWARDS THE ROBOT, keeping it level. Then stop.')
    pts_rt = n.capture(
        'Now move your hand about 40 cm to YOUR OWN RIGHT, level. Then stop. '
        '(this one is a cross-check, not an input)')

    ex.shutdown()
    rclpy.shutdown()

    d_to = excursion(pts_to)
    d_rt = excursion(pts_rt)
    if d_to is None:
        print('\nNO DATA for the towards-the-robot motion. Is the session '
              'running and the controller tracked?')
        return 2

    yaw = solve_yaw_deg(d_to[:2])
    if yaw is None:
        print('\nThe hand did not move horizontally at all. Nothing to solve.')
        return 2
    a = assess(d_to, d_rt, yaw)
    bad = verdict(a)

    print('\n' + '=' * 72)
    print('  towards-robot motion, world axes : %s' % np.round(d_to, 4))
    if d_rt is not None:
        print('  operator-right motion, world axes: %s' % np.round(d_rt, 4))
    for k, v in a.items():
        print('  %-32s %s' % (k, v))
    print()
    print('  SOLVED align_yaw_deg = %+.2f' % yaw)
    print()
    if bad:
        print('  NOT TRUSTWORTHY:')
        for b in bad:
            print('    * %s' % b)
        print('\n  Refusing to write it. Re-run with larger, level motions.')
        return 1

    rec = dict(align_yaw_deg=round(yaw, 2), hand=hand,
               towards_robot_world=[round(float(x), 4) for x in d_to],
               operator_right_world=([round(float(x), 4) for x in d_rt]
                                     if d_rt is not None else None),
               assessment=a,
               convention=('align_yaw_deg rotates the operator displacement '
                           'into the robot world frame about +z; it is a '
                           'rotation and can never be a mirror'))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        json.dump(rec, f, indent=2)
    print('  written: %s' % OUT)

    if args.apply:
        os.system("bash -lc \"ros2 param set /vr_pose_mapper align_yaw_deg %f\""
                  % yaw)
        print('  applied to the running vr_pose_mapper.')
    else:
        print('  to apply:  ros2 param set /vr_pose_mapper align_yaw_deg %.2f'
              % yaw)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hand', default='right', choices=['left', 'right'])
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    sys.exit(selftest() if a.selftest else run(a))


if __name__ == '__main__':
    main()
