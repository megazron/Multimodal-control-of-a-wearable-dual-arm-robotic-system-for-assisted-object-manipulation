#!/usr/bin/env python3
"""
mount_guard_node.py — FAIL LOUDLY at startup if the home pose is in collision
or any arm link is within the clearance floor of any wearer link.

WHY THIS EXISTS. A mount rotation was applied that buried both arm bases 75 mm
inside the wearer's torso and put a link in collision, and the way that was
discovered was a SCREENSHOT. Nothing in the stack objected: MoveIt's
/check_state_validity returned `valid=True` because the offending pairs were
SRDF-excluded, and the offline check that approved the mount excluded the same
pairs by the same argument. An exclusion silences the alarm; it does not move
the metal.

So this guard deliberately does NOT read the SRDF. It measures geometry from
TF and the collision meshes, over EVERY arm link against EVERY wearer link,
with NO exclusions. If the geometry is wrong it says so, names the pair, and
by default refuses to let the stack proceed.

    ros2 run srl_teleop mount_guard_node
    ros2 launch srl_teleop teleop.launch.py            # runs it automatically

Parameters:
    min_clearance_m   0.15   floor every arm link must keep from the wearer
    fail_on_violation true   exit non-zero, and latch the e-stop
    check_period_s    0.0    0 = check once at startup and exit; >0 = keep
                             checking, for use during development
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

import tf2_ros

from . import wearer_posture

# Wearer collision primitives, in the WORLD frame, as
# (name, kind, dimensions, centre, rpy).
#
# They are BUILT here rather than parsed from the URDF so the guard still works
# if the description fails to load — a guard that needs the thing it is
# guarding is not a guard. They come from `wearer_posture`, which is also what
# writes the URDF's arm block, so the two cannot drift: see
# `test_wearer_posture_has_one_source`.
#
# THE ARM POSTURE IS A VARIABLE and this list follows it. `SRL_WEARER_ARMS`
# picks it and the default is `down`, the arms-at-the-sides model every earlier
# workspace number was measured against.
POSTURE = wearer_posture.posture_from_env()
WEARER = wearer_posture.wearer_model(POSTURE)

# THE MEASURED BODY REACHES THIS CHECK NOW, AND ONLY EVER MAKES IT BIGGER.
#
# The scene camera can measure the wearer, and until now that measurement
# reached MoveIt's planning scene and NOT this file -- so the thing MoveIt
# planned against and the thing the CLEARANCE FLOOR was enforced against were
# two different bodies. This is HARD CONSTRAINT 11's own check, and it was
# the one still looking at the mannequin.
#
# `wearer_tracking.fuse()` keeps whichever of the tracked and mannequin
# primitive is CLOSER to the robot, per part, so subscribing to the estimate
# cannot loosen this guard however wrong the camera is. When the topic is
# silent, stale, or every segment is gated out, `fuse()` returns the
# mannequin unchanged -- which is exactly what this file did before.
try:
    from srl_perception import wearer_tracking as _WT
except Exception:                                             # noqa: BLE001
    _WT = None

# The arm as a CHAIN OF CAPSULES between consecutive link origins.
#
# A point test on link ORIGINS is what MISSED the 75 mm torso penetration:
# shoulder_link's origin is nowhere near its contact point. But inflating each
# origin by the link's whole bounding radius (up to 0.135 m) is so pessimistic
# that it fires on a geometry with 0.166 m of real clearance - measured, it
# did exactly that. A guard that cries wolf gets switched off, which is worse
# than no guard.
#
# So: sample along the SEGMENT between consecutive origins and inflate by the
# Gen3 tube radius. That is a faithful capsule for a serial arm, still
# conservative (the true mesh lies inside it almost everywhere), and it agrees
# with the mesh-accurate offline model to a few millimetres.
CHAIN = ['base_link', 'shoulder_link', 'half_arm_1_link', 'half_arm_2_link',
         'forearm_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link',
         'bracelet_link', 'end_effector_link',
         'robotiq_85_left_finger_tip_link']
TUBE_R = 0.050          # Gen3 tube is r=0.046; 4 mm of margin
SAMPLES = 8             # points per segment


def dist_point(p, kind, prm, ctr, rpy=(0.0, 0.0, 0.0)):
    """Signed distance from point `p` to one wearer primitive.

    `rpy` rotates the primitive. It used to be absent, which was fine while
    every wearer link was axis-aligned and silently wrong the moment the arm
    posture became a variable: a forearm folded across the chest is a cylinder
    lying on its side, and an unrotated test measures a different solid.
    """
    q = np.asarray(p, float) - np.asarray(ctr, float)
    if any(abs(a) > 1e-12 for a in rpy):
        R = np.asarray(wearer_posture.rot_matrix(rpy), float)
        q = R.T @ q
    if kind == 'box':
        half = np.asarray(prm, float) * 0.5
        d = np.maximum(np.abs(q) - half, 0.0)
        n = float(np.linalg.norm(d))
        return n if n > 0 else -float(np.min(half - np.abs(q)))
    if kind == 'cylinder':
        r, L = prm
        radial = math.hypot(q[0], q[1]) - r
        axial = abs(q[2]) - L * 0.5
        if radial <= 0 and axial <= 0:
            return max(radial, axial)
        return math.hypot(max(radial, 0.0), max(axial, 0.0))
    return float(np.linalg.norm(q)) - prm[0]


class MountGuard(Node):
    def __init__(self):
        super().__init__('mount_guard_node')
        self.declare_parameter('arms', ['left', 'right'])
        self.declare_parameter('min_clearance_m', 0.15)
        self.declare_parameter('fail_on_violation', True)
        self.declare_parameter('check_period_s', 0.0)
        self.declare_parameter('settle_s', 6.0)

        self.arms = list(self.get_parameter('arms').value)
        self.floor = float(self.get_parameter('min_clearance_m').value)
        self.fail = bool(self.get_parameter('fail_on_violation').value)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.report_pub = self.create_publisher(String, '/mount_guard', 10)
        # WHICH BODY IS BEING ENFORCED, said out loud on its own topic. The
        # operator has to be able to tell "the floor is measured against the
        # person" from "the floor is measured against a mannequin" without
        # reading a log.
        self.body_pub = self.create_publisher(String, '/wearer/enforced', 10)
        self.declare_parameter('use_tracked_wearer', True)
        self._est = None
        self._est_t = None
        self._body_note = 'mannequin (no scene camera seen)'
        if _WT is not None:
            self.create_subscription(String, '/wearer/estimate',
                                     self._on_wearer, 10)
            self.create_timer(2.0, self._say_body)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.done = False
        period = float(self.get_parameter('check_period_s').value)
        self.create_timer(float(self.get_parameter('settle_s').value), self._first)
        if period > 0:
            self.create_timer(period, lambda: self.check(quiet=True))

    # ------------------------------------------------ the body being used
    def _on_wearer(self, msg):
        try:
            self._est = json.loads(msg.data)
            self._est_t = time.monotonic()
        except Exception:                                     # noqa: BLE001
            self._est = None

    def wearer(self):
        """The primitives this guard enforces against, right now.

        NEVER LESS CONSERVATIVE THAN `WEARER`. Every path out of here either
        returns the mannequin or returns `fuse()`'s output, and `fuse()` is
        defined to keep whichever primitive is closer to the robot.
        """
        # NOBODY IN THE RIG is checked FIRST, before tracking, before the
        # mannequin, before anything. It is the one condition under which
        # this guard has nothing to guard.
        if not wearer_posture.wearer_present():
            self._body_note = 'NO WEARER -- declared absent'
            self.get_logger().warn(wearer_posture.absence_banner(),
                                   throttle_duration_sec=10.0)
            return []
        if _WT is None or not bool(
                self.get_parameter('use_tracked_wearer').value):
            self._body_note = 'mannequin (tracking not enabled here)'
            return WEARER
        est, t = self._est, self._est_t
        if est is None or t is None:
            self._body_note = 'mannequin (no scene camera seen)'
            return WEARER
        age = time.monotonic() - t
        if age > _WT.MAX_AGE_S * 4:
            self._body_note = ('mannequin (the scene camera stopped %.1f s '
                               'ago)' % age)
            return WEARER
        segs = []
        for name, d in (est.get('segments') or {}).items():
            if not d.get('usable'):
                continue
            segs.append(_WT.Segment(name, d.get('kind', 'cylinder'),
                                    d.get('dims', (0.0, 0.0)),
                                    d.get('centre', (0.0, 0.0, 0.0)),
                                    d.get('rpy', (0.0, 0.0, 0.0)),
                                    float(d.get('conf', 0.0))))
        if not segs:
            self._body_note = 'mannequin (nothing measurable in view)'
            return WEARER
        e = _WT.Estimate(segs, stamp=time.monotonic(), source='scene camera')
        fused, dec = _WT.fuse(WEARER, e, time.monotonic())
        n, _tot, text = _WT.summarise(dec)
        self._body_note = ('%s (%d part%s measured)'
                           % ('measured body' if n else 'mannequin', n,
                              '' if n == 1 else 's'))
        return fused

    def _say_body(self):
        prims = self.wearer()
        self.body_pub.publish(String(data=json.dumps(dict(
            enforcing=self._body_note,
            n_parts=len(prims),
            floor_m=self.floor,
            note=('The clearance floor is enforced against this body. It is '
                  'never smaller than the mannequin: the camera may only '
                  'make the wearer bigger.')))))

    def _first(self):
        if not self.done:
            self.check()
            self.done = True

    def check(self, quiet=False):
        pairs = []
        missing = []
        for arm in self.arms:
            origins = []
            for ln in CHAIN:
                frame = f'{arm}_{ln}'
                try:
                    t = self.buf.lookup_transform('world', frame, rclpy.time.Time())
                    origins.append((ln, np.array([t.transform.translation.x,
                                                  t.transform.translation.y,
                                                  t.transform.translation.z])))
                except Exception:
                    missing.append(frame)
            for i in range(len(origins)):
                ln, a = origins[i]
                b = origins[i + 1][1] if i + 1 < len(origins) else a
                pts = [a + (b - a) * (k / float(SAMPLES)) for k in range(SAMPLES + 1)]
                for nm, kind, prm, ctr, rpy in self.wearer():
                    d = min(dist_point(q, kind, prm, ctr, rpy) for q in pts) - TUBE_R
                    pairs.append((d, arm, ln, nm))
        if missing and len(missing) == len(CHAIN) * len(self.arms):
            self.get_logger().warn(
                'no arm TF yet - guard has not run. This is NOT a pass.',
                throttle_duration_sec=10.0)
            self.done = False
            return
        pairs.sort()
        colliding = [p for p in pairs if p[0] < 0.0]
        close = [p for p in pairs if 0.0 <= p[0] < self.floor]

        msg = {
            'checked_pairs': len(pairs),
            'colliding': len(colliding),
            'below_floor': len(close),
            'floor_m': self.floor,
            'worst': ('%s %s <-> %s %.4f m' % (pairs[0][1], pairs[0][2],
                                               pairs[0][3], pairs[0][0])) if pairs else '',
        }
        out = String()
        out.data = str(msg)
        self.report_pub.publish(out)

        if not colliding and not close:
            if not quiet:
                self.get_logger().info(
                    'MOUNT GUARD PASS: %d link/wearer pairs checked, worst '
                    'clearance %.4f m (floor %.2f m).'
                    % (len(pairs), pairs[0][0], self.floor))
            return True

        banner = '=' * 72
        self.get_logger().error(banner)
        self.get_logger().error('MOUNT GUARD FAILED — the home geometry is wrong.')
        self.get_logger().error(banner)
        if colliding:
            self.get_logger().error(
                '%d COLLIDING link pairs (conservative bounding-sphere test):'
                % len(colliding))
            for d, arm, ln, nm in colliding[:12]:
                self.get_logger().error(
                    '   %-6s %-32s <-> %-12s  penetration %.4f m'
                    % (arm, ln, nm, -d))
        if close:
            self.get_logger().error(
                '%d pairs inside the %.2f m clearance floor:' % (len(close), self.floor))
            for d, arm, ln, nm in close[:12]:
                self.get_logger().error(
                    '   %-6s %-32s <-> %-12s  clearance %.4f m' % (arm, ln, nm, d))
        self.get_logger().error(banner)
        self.get_logger().error(
            'This check deliberately IGNORES the SRDF. An SRDF exclusion '
            'silences the alarm; it does not move the metal. If a pair here '
            'is genuinely structural, fix the MOUNT, do not widen the guard.')
        self.get_logger().error(banner)

        if self.fail:
            b = Bool()
            b.data = True
            self.estop_pub.publish(b)
            self.get_logger().error(
                'fail_on_violation is set: latching the e-stop and exiting 1.')
            rclpy.shutdown()
            sys.exit(1)
        return False


def main():
    rclpy.init()
    n = MountGuard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        raise
    finally:
        if rclpy.ok():
            n.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
