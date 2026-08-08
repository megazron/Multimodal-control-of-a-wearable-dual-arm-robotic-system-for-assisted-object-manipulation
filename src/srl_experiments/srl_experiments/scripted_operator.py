#!/usr/bin/env python3
"""
scripted_operator.py — a synthetic operator, so every experiment runs end to
end with no human and no hardware.

This is what makes the pipeline testable. A study harness that can only be
exercised by running a participant through it is a harness whose bugs are
found on participants.

It drives the same topics the real master publishes:
    /master_arm_pose_<arm>     PoseStamped   position command
    /master_pointing_<arm>     Vector3Stamped  pointing direction
    /master_arm_raw_<arm>      Float64MultiArray  pots + IMU, so channel
                                                  health and dropouts are
                                                  exercised too
and it can be told to inject the faults the real rig has, which is how the
"abort on channel dropout" path gets tested without waiting for a pot to fail.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class ScriptedOperator(Node):
    def __init__(self):
        super().__init__("scripted_operator")
        self.declare_parameter("arm", "left")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("home", [-0.52, 0.21, 1.39])
        # Reach speed, m/s. The real operator manages roughly this.
        self.declare_parameter("speed_mps", 0.12)
        # Fault injection, for testing the abort path.
        self.declare_parameter("dropout_channel", -1)     # 0-based j index
        self.declare_parameter("dropout_fraction", 0.0)
        self.declare_parameter("dropout_burst_len", 25)

        self.arm = self.get_parameter("arm").value
        self.hz = float(self.get_parameter("rate_hz").value)
        self.home = np.array(self.get_parameter("home").value, float)
        self.speed = float(self.get_parameter("speed_mps").value)

        self.pose_pub = self.create_publisher(
            PoseStamped, f"/master_arm_pose_{self.arm}", 10)
        self.point_pub = self.create_publisher(
            Vector3Stamped, f"/master_pointing_{self.arm}", 10)
        self.raw_pub = self.create_publisher(
            Float64MultiArray, f"/master_arm_raw_{self.arm}", 10)
        self.status_pub = self.create_publisher(
            String, f"/scripted_operator_{self.arm}", 10)
        # Mirror the real master's status layout so the CLUTCH column is
        # populated in scripted runs too. Without it the column is silently
        # empty in every pilot and a real clutch bug would not be caught.
        self.master_status_pub = self.create_publisher(
            Float64MultiArray, f"/master_status_{self.arm}", 10)
        self.declare_parameter("clutch_engaged", True)
        self.create_subscription(String, f"/scripted_operator_cmd_{self.arm}",
                                 self._on_cmd, 10)

        self.pos = self.home.copy()
        self.goal = self.home.copy()
        self.pointing_at = None
        self.hold = 0.0
        self.burst = 0
        self.rng = np.random.default_rng(0)
        self.n = 0
        self.create_timer(1.0 / self.hz, self._tick)

    # ----------------------------------------------------------- commands
    def go_to(self, p, point_at=None):
        self.goal = np.asarray(p, float)
        self.pointing_at = None if point_at is None else np.asarray(point_at, float)

    def _on_cmd(self, msg):
        """`x,y,z[;px,py,pz]` — move here, optionally pointing at that."""
        try:
            parts = msg.data.split(";")
            p = [float(v) for v in parts[0].split(",")]
            q = [float(v) for v in parts[1].split(",")] if len(parts) > 1 else None
            self.go_to(p, q)
        except (ValueError, IndexError):
            self.get_logger().warn("bad command %r" % msg.data)

    @property
    def at_goal(self):
        return float(np.linalg.norm(self.goal - self.pos)) < 0.005

    # -------------------------------------------------------------- loop
    def _tick(self):
        self.n += 1
        step = self.speed / self.hz
        d = self.goal - self.pos
        n = float(np.linalg.norm(d))
        if n > step:
            # Constant-speed straight-line reach. Deliberately NOT a
            # minimum-jerk profile: a scripted operator that moves more
            # smoothly than a person would make the analysis look better than
            # it is.
            self.pos = self.pos + d / n * step
        else:
            self.pos = self.goal.copy()

        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            (float(v) for v in self.pos)
        m.pose.orientation.w = 1.0
        self.pose_pub.publish(m)

        target = self.pointing_at if self.pointing_at is not None else self.goal
        v = np.asarray(target, float) - self.pos
        nv = float(np.linalg.norm(v))
        u = v / nv if nv > 1e-6 else np.array([0.0, 1.0, 0.0])
        pm = Vector3Stamped()
        pm.header.stamp = m.header.stamp
        pm.header.frame_id = "world"
        pm.vector.x, pm.vector.y, pm.vector.z = (float(c) for c in u)
        self.point_pub.publish(pm)

        self.raw_pub.publish(self._raw())
        st = Float64MultiArray()
        # [clutch, scale, 0, 0, 0, valid, n_bad] -- master_pose_node's layout
        st.data = [1.0 if bool(self.get_parameter("clutch_engaged").value) else 0.0,
                   1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.master_status_pub.publish(st)

    def _raw(self):
        """Synthetic pot + IMU frame, with optional bursty dropouts.

        Bursty and not independent per frame, because the real fault is
        bursty: right j4's 12.9% of zero frames are concentrated, e.g. 51.1%
        of one recorded sweep. Independent dropouts would make the abort logic
        look far more robust than it is.
        """
        j = [30.0 * math.sin(0.01 * self.n + i) for i in range(7)]
        ch = int(self.get_parameter("dropout_channel").value)
        frac = float(self.get_parameter("dropout_fraction").value)
        blen = int(self.get_parameter("dropout_burst_len").value)
        if 0 <= ch < 7 and frac > 0.0:
            if self.burst > 0:
                self.burst -= 1
                j[ch] = 0.0
            elif self.rng.random() < frac / max(1, blen):
                self.burst = blen
                j[ch] = 0.0
        # accel: gravity for an arm at rest, plus the elevation the reach
        # implies, so the elevation channel is exercised rather than constant
        el = math.atan2(self.pos[2] - self.home[2], 0.3)
        accel = [0.0, math.cos(el), math.sin(el)]
        gyro = [0.0, 0.0, 0.0]
        m = Float64MultiArray()
        m.data = [float(x) for x in j] + accel + gyro
        return m


def main():
    rclpy.init()
    n = ScriptedOperator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
