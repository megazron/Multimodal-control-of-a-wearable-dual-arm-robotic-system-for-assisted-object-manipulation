#!/usr/bin/env python3
"""
mock_real_stack.py — a stand-in for the real arm, so the whole gated flow can
be exercised with no hardware attached.

It does the three things the real stack does that the rest of the system can
observe, and nothing else:
  * publishes /real/joint_states at 100 Hz
  * accepts /real/<arm>_arm_controller/joint_trajectory
  * moves toward the commanded position at a finite rate, like a real
    JointTrajectoryController tracking a setpoint

and one thing the real stack does only when something is wrong:
  * `stall:=true` freezes it while still publishing, which is exactly the
    failure the lag monitor exists to catch. Without a way to produce that
    failure on demand, "the lag monitor is armed" is an untested claim.

It deliberately starts AWAY from home (start_offset_deg), so homing has real
work to do and the arrival tolerance means something.

  ros2 run srl_teleop mock_real_stack --ros-args -p arm:=left
  ros2 param set /mock_real_stack stall true      # trip the lag monitor
"""
import math
import os
import sys
import threading
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, os.path.expanduser("~/kortex_ws/config"))
import home_positions                                        # noqa: E402

from srl_teleop.kortex_convention import wrap_rad_pi         # noqa: E402

CONTINUOUS_IDX = (0, 2, 4, 6)


class MockReal(Node):
    def __init__(self):
        super().__init__("mock_real_stack")
        self.declare_parameter("arm", "left")
        self.declare_parameter("rate_hz", 100.0)
        # How fast the mock controller can chase its setpoint. Well above the
        # 0.05 rad/s the bridge commands, so the mock is not itself the
        # bottleneck -- if the lag monitor trips, it is because of `stall`.
        self.declare_parameter("track_vel_rad_s", 0.5)
        self.declare_parameter("stall", False)
        # Start away from home so homing has to actually work.
        self.declare_parameter("start_offset_deg",
                               [25.0, -18.0, 40.0, 12.0, -60.0, 15.0, -22.0])
        self.declare_parameter("start_at_home", False)

        self.arm = self.get_parameter("arm").value
        self.rate = float(self.get_parameter("rate_hz").value)
        self.track_vel = float(self.get_parameter("track_vel_rad_s").value)
        self.names = [f"{self.arm}_joint_{i}" for i in range(1, 8)]

        home = list(home_positions.load_home_radians(self.arm))
        if bool(self.get_parameter("start_at_home").value):
            self.q = list(home)
        else:
            off = list(self.get_parameter("start_offset_deg").value)
            self.q = [wrap_rad_pi(home[i] + math.radians(off[i]))
                      if i in CONTINUOUS_IDX else home[i] + math.radians(off[i])
                      for i in range(7)]
        self.setpoint = list(self.q)

        self.pub = self.create_publisher(JointState, "/real/joint_states", 20)
        self.create_subscription(
            JointTrajectory,
            f"/real/{self.arm}_arm_controller/joint_trajectory", self.on_cmd, 20)
        self.create_timer(1.0 / self.rate, self.step)
        self.last = time.monotonic()
        self.get_logger().info(
            "mock real arm up. start deg: %s"
            % " ".join("%.1f" % math.degrees(v) for v in self.q))

    def on_cmd(self, m):
        if not m.points:
            return
        idx = {n: i for i, n in enumerate(m.joint_names)}
        p = m.points[-1].positions
        for j, n in enumerate(self.names):
            if n in idx and idx[n] < len(p):
                self.setpoint[j] = p[idx[n]]

    def step(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        if not bool(self.get_parameter("stall").value):
            cap = self.track_vel * dt
            for i in range(7):
                e = self.setpoint[i] - self.q[i]
                if i in CONTINUOUS_IDX:
                    e = wrap_rad_pi(e)
                if e > cap:
                    e = cap
                elif e < -cap:
                    e = -cap
                q = self.q[i] + e
                self.q[i] = wrap_rad_pi(q) if i in CONTINUOUS_IDX else q
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(self.names)
        m.position = list(self.q)
        # A tiny deterministic dither, because a real encoder always has some
        # and a perfectly constant channel is the signature of a DEAD one --
        # this project has been bitten by exactly that (see CLAUDE.md).
        m.velocity = [0.0] * 7
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = MockReal()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
