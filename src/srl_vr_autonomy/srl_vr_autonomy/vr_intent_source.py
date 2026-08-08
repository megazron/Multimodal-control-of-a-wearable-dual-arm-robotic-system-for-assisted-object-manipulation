#!/usr/bin/env python3
"""
vr_intent_source.py — publishes the pointing vector srl_autonomy already
consumes, sourced from the Quest instead of the mannequin's IMU.

THE WHOLE POINT: the inference code is NOT duplicated. srl_autonomy's
`intent_inference` subscribes to `/master_pointing_<arm>`; this node publishes
that topic from the controller. Same Bayesian update, same forgetting factor,
same three hard cases, same tests. The only difference is where the vector
comes from, which is exactly the experimental manipulation E6 needs: if the
inference code differed between modalities, a difference in inference accuracy
would be partly the code and nobody could separate it.

WHY THE VR SIGNAL IS CLEANER, stated so the comparison is honest:
  mannequin  elevation from gravity (absolute), azimuth from ONE pot (j1),
             optionally gyro-aided; yaw is not observable from gravity at all.
  VR         full 3-D pointing from a tracked 6-DOF pose at 72 Hz. No dead
             channels, no azimuth coupling, no drift.
So E6 is not "the same signal through two pipelines" - it is a deliberately
BETTER signal through the same pipeline, which is the manipulation.

Hand velocity is published alongside as the secondary cue, replacing the
mannequin's commanded-EE-velocity cue. On the mannequin that cue was
contaminated by pot dropouts freezing the command; here it is clean.
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import String


def q_rot(q, v):
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ np.asarray(v, float)


class VrIntentSource(Node):
    def __init__(self):
        super().__init__('vr_intent_source')
        self.declare_parameter('hands', ['left', 'right'])
        self.declare_parameter('left_controller_arm', 'left')
        self.declare_parameter('right_controller_arm', 'right')
        # The controller's local axis that "points". On a Quest controller the
        # natural pointing ray is -z in the grip frame; exposed because it
        # differs between controller generations.
        self.declare_parameter('point_axis', [0.0, 0.0, -1.0])
        self.declare_parameter('vel_tau_s', 0.1)

        self.hands = list(self.get_parameter('hands').value)
        self.arm_of = {'left': self.get_parameter('left_controller_arm').value,
                       'right': self.get_parameter('right_controller_arm').value}
        self.axis = np.array(self.get_parameter('point_axis').value, float)
        self.prev = {h: None for h in self.hands}
        self.vel = {h: np.zeros(3) for h in self.hands}
        self.pub, self.vpub, self.st = {}, {}, {}
        for h in self.hands:
            arm = self.arm_of[h]
            self.pub[h] = self.create_publisher(
                Vector3Stamped, f'/master_pointing_{arm}', 10)
            self.vpub[h] = self.create_publisher(
                Vector3Stamped, f'/vr/hand_velocity_{arm}', 10)
            self.st[h] = self.create_publisher(String, f'/vr/intent_source_{h}', 10)
            self.create_subscription(PoseStamped, f'/vr/controller_pose_{h}',
                                     lambda m, h=h: self._on(h, m), 20)
        self.get_logger().info(
            'vr_intent_source up. Publishing /master_pointing_<arm> so '
            'srl_autonomy runs UNCHANGED under VR.')

    def _on(self, hand, m):
        q = np.array([m.pose.orientation.x, m.pose.orientation.y,
                      m.pose.orientation.z, m.pose.orientation.w])
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        u = q_rot(q, self.axis)
        n = float(np.linalg.norm(u))
        if n < 1e-9:
            return
        u = u / n
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev[hand] is not None:
            pt, pp = self.prev[hand]
            dt = t - pt
            if 1e-4 < dt < 0.5:
                k = dt / (float(self.get_parameter('vel_tau_s').value) + dt)
                self.vel[hand] = (1 - k) * self.vel[hand] + k * (p - pp) / dt
        self.prev[hand] = (t, p)

        v = Vector3Stamped()
        v.header.stamp = m.header.stamp
        v.header.frame_id = 'world'
        v.vector.x, v.vector.y, v.vector.z = (float(c) for c in u)
        self.pub[hand].publish(v)

        vv = Vector3Stamped()
        vv.header = v.header
        vv.vector.x, vv.vector.y, vv.vector.z = (float(c) for c in self.vel[hand])
        self.vpub[hand].publish(vv)

        s = String()
        s.data = json.dumps(dict(hand=hand, arm=self.arm_of[hand],
                                 pointing=[round(float(c), 4) for c in u],
                                 speed_mps=round(float(np.linalg.norm(self.vel[hand])), 4),
                                 source='quest_6dof',
                                 yaw_observable=True))
        self.st[hand].publish(s)


def main():
    rclpy.init()
    n = VrIntentSource()
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
