#!/usr/bin/env python3
"""
fake_pot.py — Simulate the Teensy for testing without hardware
===============================================================
Publishes fake pot values to a virtual serial-like topic so you can
test the full teleoperation pipeline (pot -> joint -> arm) in sim,
before the real Teensy is connected.

This version sweeps k1j7 back and forth automatically so you can
watch joint 7 of the simulated arm move in RViz.

Run:
  ros2 run srl_teleop fake_pot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math


class FakePot(Node):
    def __init__(self):
        super().__init__("fake_pot")
        self.pub = self.create_publisher(String, "/fake_serial", 10)
        self.t = 0.0
        # 20 Hz, same rate a Teensy would stream
        self.create_timer(0.05, self.tick)
        self.get_logger().info("fake_pot started — sweeping k1j7 0..360")

    def tick(self):
        self.t += 0.05
        # Sweep 0..360 with a slow sine so it's smooth and visible
        raw = 180.0 + 170.0 * math.sin(self.t * 0.5)   # ranges ~10..350
        # Build a Teensy-style line; other joints zero (unconnected)
        line = (
            f"k1j1:0.0,k1j2:0.0,k1j3:0.0,k1j4:0.0,"
            f"k1j5:0.0,k1j6:0.0,k1j7:{raw:.1f},"
            f"k2j1:0.0,k2j2:0.0,k2j3:0.0,k2j4:0.0,"
            f"k2j5:0.0,k2j6:0.0,k2j7:0.0"
        )
        msg = String()
        msg.data = line
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

