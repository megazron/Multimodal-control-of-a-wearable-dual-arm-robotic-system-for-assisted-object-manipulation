#!/usr/bin/env python3
"""
vr_safety_node.py — the interlocks that only VR needs.

THE ONE THAT MATTERS: **the operator cannot see the real arm.** Inside a
headset there is no peripheral view of a 17 kg robot moving near a person. The
mannequin operator can at least look at it. So an INDEPENDENT OBSERVER E-STOP,
held by someone who is not wearing the headset, is MANDATORY, and this node
refuses to enable real-arm control until its presence is confirmed for the
session. That is not a policy note; it is enforced here.

The rest:
  * tracking loss > 0.2 s        -> freeze
  * network dropout              -> freeze (same watchdog, same threshold)
  * headset asleep / backgrounded-> looks identical to a dropout, and is
                                    handled by the same path on purpose
  * workspace boundary           -> warn in-headset BEFORE the arm reaches its
                                    limit, because the operator cannot see it
                                    approaching
  * collision-aware IK + the same clearance floors as the mannequin path

FREEZE means "publish nothing". The follower holds its last commanded pose.
Publishing a "safe" pose instead would be commanding a motion the operator did
not ask for, which is worse.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

import tf2_ros


class VrSafety(Node):
    def __init__(self):
        super().__init__('vr_safety_node')
        self.declare_parameter('arms', ['left', 'right'])
        self.declare_parameter('tracking_timeout_s', 0.2)
        self.declare_parameter('require_observer_estop', True)
        self.declare_parameter('allow_real_arm', False)
        # Robot workspace bounds, world frame. Warn before the limit, not at it.
        self.declare_parameter('ws_min', [-1.10, -0.10, 0.70])
        self.declare_parameter('ws_max', [1.10, 0.90, 1.60])
        self.declare_parameter('ws_warn_margin_m', 0.12)

        self.arms = list(self.get_parameter('arms').value)
        self.timeout = float(self.get_parameter('tracking_timeout_s').value)
        self.last_pose_t = 0.0
        self.observer_ok = False
        self.frozen = True
        self.freeze_reason = 'startup'
        self.n_freezes = 0

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.freeze_pub = self.create_publisher(Bool, '/vr/freeze', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop', 10)
        self.status_pub = self.create_publisher(String, '/vr/safety', 10)
        self.warn_pub = self.create_publisher(String, '/vr/boundary_warning', 10)

        self.create_subscription(Bool, '/vr/observer_estop_present',
                                 self._on_observer, 10)
        self.create_subscription(Bool, '/vr/tracking_ok', self._on_track, 10)
        for h in ('left', 'right'):
            self.create_subscription(
                PoseStamped, f'/vr/controller_pose_{h}',
                lambda m: setattr(self, 'last_pose_t', time.monotonic()), 20)
        self.create_service(Trigger, '/vr/enable_real_arm', self._srv_enable)
        self.create_timer(0.05, self._tick)
        self.get_logger().warn(
            'vr_safety_node up. Real-arm control is REFUSED until an '
            'independent observer e-stop is confirmed: the operator in a '
            'headset cannot see the arm.')

    def _on_observer(self, m):
        if m.data and not self.observer_ok:
            self.observer_ok = True
            self.get_logger().info('observer e-stop CONFIRMED present for this session')
        elif not m.data and self.observer_ok:
            self.observer_ok = False
            self._freeze('observer e-stop withdrawn')

    def _on_track(self, m):
        if not m.data:
            self._freeze('headset tracking lost')

    def _freeze(self, why):
        if not self.frozen:
            self.n_freezes += 1
            self.get_logger().error('VR FREEZE: %s. Publishing no command; the '
                                    'follower holds its last pose.' % why)
        self.frozen = True
        self.freeze_reason = why

    def _srv_enable(self, req, res):
        if bool(self.get_parameter('require_observer_estop').value) and not self.observer_ok:
            res.success = False
            res.message = ('REFUSED: no observer e-stop confirmed. Publish '
                           '/vr/observer_estop_present after physically '
                           'handing it to someone not wearing the headset.')
            self.get_logger().error(res.message)
            return res
        self.set_parameters([rclpy.parameter.Parameter(
            'allow_real_arm', rclpy.Parameter.Type.BOOL, True)])
        res.success = True
        res.message = 'real-arm control enabled for this VR session'
        return res

    def _boundary(self):
        lo = np.array(self.get_parameter('ws_min').value, float)
        hi = np.array(self.get_parameter('ws_max').value, float)
        margin = float(self.get_parameter('ws_warn_margin_m').value)
        warns = []
        for arm in self.arms:
            try:
                t = self.tf_buf.lookup_transform(
                    'world', f'{arm}_end_effector_link', rclpy.time.Time())
            except Exception:
                continue
            p = np.array([t.transform.translation.x, t.transform.translation.y,
                          t.transform.translation.z])
            near_lo = p - lo
            near_hi = hi - p
            d = float(min(near_lo.min(), near_hi.min()))
            if d < margin:
                ax = int(np.argmin(np.concatenate([near_lo, near_hi])))
                warns.append(dict(arm=arm, margin_m=round(d, 4),
                                  axis='xyz'[ax % 3],
                                  side='min' if ax < 3 else 'max'))
        return warns

    def _tick(self):
        age = time.monotonic() - self.last_pose_t if self.last_pose_t else 1e9
        if age > self.timeout:
            self._freeze('no controller pose for %.2f s (dropout, sleep or '
                         'backgrounded app all look identical here, and all '
                         'must freeze)' % age)
        elif self.frozen and self.observer_ok:
            self.frozen = False
            self.freeze_reason = ''
            self.get_logger().info('VR unfrozen: poses flowing, observer present')
        elif self.frozen and not self.observer_ok:
            self.freeze_reason = 'awaiting observer e-stop confirmation'

        b = Bool()
        b.data = bool(self.frozen)
        self.freeze_pub.publish(b)

        warns = self._boundary()
        if warns:
            w = String()
            w.data = json.dumps(warns)
            self.warn_pub.publish(w)

        s = String()
        s.data = json.dumps(dict(frozen=self.frozen, reason=self.freeze_reason,
                                 observer_estop=self.observer_ok,
                                 real_arm_allowed=bool(
                                     self.get_parameter('allow_real_arm').value),
                                 pose_age_s=round(min(age, 999.0), 3),
                                 freezes=self.n_freezes,
                                 boundary_warnings=warns))
        self.status_pub.publish(s)


def main():
    rclpy.init()
    n = VrSafety()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
