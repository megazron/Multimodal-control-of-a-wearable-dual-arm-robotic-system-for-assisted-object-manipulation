#!/usr/bin/env python3
"""
vr_mock_publisher.py — a desktop stand-in for the Quest, so the ENTIRE ROS
side is testable without a headset.

This is what makes the VR path verifiable tonight. It publishes exactly the
topics quest_bridge_node publishes, at the same rate, in the same frames, with
the same button semantics, so every node downstream of the bridge is exercised
for real. The only thing it cannot test is the headset itself and the
WebSocket transit.

It also injects the failures that matter:
    --dropout-at / --dropout-for   simulate a network drop or headset sleep
    --tracking-glitch              simulate tracking loss
so the freeze path is tested rather than assumed.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray, String


class VrMock(Node):
    def __init__(self, a):
        super().__init__('vr_mock_publisher')
        self.a = a
        self.pose_pub, self.joy_pub = {}, {}
        for h in ('left', 'right'):
            self.pose_pub[h] = self.create_publisher(PoseStamped, f'/vr/controller_pose_{h}', 20)
            self.joy_pub[h] = self.create_publisher(Joy, f'/vr/controller_joy_{h}', 20)
        self.track_pub = self.create_publisher(Bool, '/vr/tracking_ok', 10)
        self.lat_pub = self.create_publisher(Float64MultiArray, '/vr/latency', 10)
        self.t0 = time.monotonic()
        self.n = 0
        self.grip = {'left': a.grip, 'right': a.grip}
        self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(
            'vr_mock_publisher up at %.0f Hz. Everything DOWNSTREAM of the '
            'bridge is now exercised; the headset and the WebSocket are not.'
            % a.rate)

    def tick(self):
        t = time.monotonic() - self.t0
        # HOLD STILL for `settle` seconds first. vr_pose_mapper refuses to
        # latch a reference off a MOVING hand (that is the classic source of a
        # re-engage jump), so a mock that moves from frame zero can never
        # engage the clutch - which is correct behaviour, and was observed.
        moving = t > self.a.settle
        self.n += 1
        dropped = (self.a.dropout_at > 0 and
                   self.a.dropout_at <= t < self.a.dropout_at + self.a.dropout_for)
        ok = not dropped and not (self.a.tracking_glitch > 0 and
                                  self.a.tracking_glitch <= t < self.a.tracking_glitch + 0.5)
        b = Bool(); b.data = bool(ok); self.track_pub.publish(b)
        if dropped:
            return                       # publish NOTHING: a real dropout is silence
        now = self.get_clock().now().to_msg()
        for h, sgn in (('left', 1.0), ('right', -1.0)):
            # a slow figure-of-eight in the play space, ~0.25 m across
            tm = (t - self.a.settle) if moving else 0.0
            p = np.array([0.25 * math.sin(2 * math.pi * 0.08 * tm),
                          0.15 * math.sin(2 * math.pi * 0.16 * tm) + sgn * 0.20,
                          1.10 + 0.10 * math.sin(2 * math.pi * 0.05 * tm)])
            ang = 0.4 * math.sin(2 * math.pi * 0.06 * tm)
            q = np.array([0.0, 0.0, math.sin(ang / 2), math.cos(ang / 2)])
            m = PoseStamped(); m.header.stamp = now; m.header.frame_id = 'vr_play_space'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = map(float, q)
            self.pose_pub[h].publish(m)
            j = Joy(); j.header.stamp = now
            j.axes = [float(self.a.trigger), float(self.grip[h]), 0.0, 0.0]
            j.buttons = [0, 0]
            self.joy_pub[h].publish(j)
        if self.n % 20 == 0:
            lm = Float64MultiArray()
            lm.data = [float(self.a.rate), 0.0, 0.0, 0.0, 0.0, float(self.n)]
            self.lat_pub.publish(lm)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=72.0)
    ap.add_argument('--grip', type=float, default=1.0, help='1 = clutch engaged')
    ap.add_argument('--trigger', type=float, default=0.0)
    ap.add_argument('--dropout-at', type=float, default=-1.0)
    ap.add_argument('--dropout-for', type=float, default=1.0)
    ap.add_argument('--tracking-glitch', type=float, default=-1.0)
    ap.add_argument('--settle', type=float, default=2.5,
                    help='hold still this long so the clutch can engage')
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = VrMock(a)
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
