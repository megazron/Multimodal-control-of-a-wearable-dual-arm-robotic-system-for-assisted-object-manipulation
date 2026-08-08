#!/usr/bin/env python3
"""
pointing_direction_node.py — POINTING DIRECTION as a first-class topic.

This is the signal srl_autonomy's intent inference is built on, so it gets its
own node and its own topic rather than being derived inside the consumer.

WHY POINTING AND NOT COMMANDED VELOCITY. Pointing is measured directly: its
elevation comes from gravity and is absolute and drift-free. Commanded
end-effector velocity is a derivative of the pot chain and inherits every pot
fault the master has — on this rig right j4 drops out on 12.9% of frames in
bursts, and a rejected frame FREEZES the command, so velocity reads zero when
the operator is in fact moving. Inferring intent from that is inferring from
the fault.

WHAT IS AND IS NOT TRUSTWORTHY, published explicitly:
  elevation  absolute, from gravity. Trust it.
  azimuth    NOT observable from gravity. Taken from j1, a directly measured,
             zero-referenced pot, optionally aided by integrating yaw rate
             about the measured vertical. Confidence is published separately
             and drops as gyro aiding accumulates drift.

Topics:
  /master_pointing_<arm>         geometry_msgs/Vector3Stamped, wearer frame
  /master_pointing_state_<arm>   JSON: elevation, azimuth, per-axis confidence
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray, String

from srl_teleop import master_calibration as mc
from srl_teleop.imu_orientation import elevation_from_accel, pointing_direction

ARMS = ("left", "right")
DEG = 180.0 / math.pi


class PointingNode(Node):
    def __init__(self):
        super().__init__("pointing_direction_node")
        self.declare_parameter("arms", ["left", "right"])
        #   "j1"   -- azimuth from the shoulder-roll pot alone. Absolute, and
        #             the default: it is one directly measured quantity.
        #   "gyro" -- j1 plus integrated yaw rate about the measured vertical,
        #             pulled slowly back to j1 so a long segment cannot run
        #             away. Measured drift: -5.0 deg/min left, +44.0 deg/min
        #             right (harness fault). NOT the default for that reason.
        self.declare_parameter("azimuth_mode", "j1")
        self.declare_parameter("azimuth_blend_tau_s", 20.0)
        self.declare_parameter("accel_gate_g", 0.15)
        for a in ARMS:
            self.declare_parameter(f"{a}_gyro_bias", [0.0, 0.0, 0.0])
            self.declare_parameter(f"{a}_j1_zero_deg", 0.0)

        self.arms = [a for a in self.get_parameter("arms").value if a in ARMS]
        self.mode = self.get_parameter("azimuth_mode").value
        if self.mode not in ("j1", "gyro"):
            raise ValueError("azimuth_mode must be 'j1' or 'gyro'")
        self.tau = float(self.get_parameter("azimuth_blend_tau_s").value)
        self.gate = float(self.get_parameter("accel_gate_g").value)

        self.a_hat, self.bias, self.j1_zero = {}, {}, {}
        for a in self.arms:
            self.a_hat[a] = self._load_a_hat(a)
            self.bias[a] = np.radians(
                np.array(self.get_parameter(f"{a}_gyro_bias").value, float))
            self.j1_zero[a] = float(self.get_parameter(f"{a}_j1_zero_deg").value)

        self.az = {a: None for a in self.arms}
        self.u_hat = {a: None for a in self.arms}   # last trusted world-up
        self.t_last = {a: None for a in self.arms}
        self.drift = {a: 0.0 for a in self.arms}

        self.vec_pub = {a: self.create_publisher(
            Vector3Stamped, f"/master_pointing_{a}", 10) for a in self.arms}
        self.st_pub = {a: self.create_publisher(
            String, f"/master_pointing_state_{a}", 10) for a in self.arms}
        for a in self.arms:
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda m, a=a: self._on_raw(a, m), 20)
        self.get_logger().info(
            "pointing_direction_node up, azimuth_mode=%s. Elevation is "
            "absolute; azimuth confidence is published separately because "
            "gravity carries no yaw information." % self.mode)

    def _load_a_hat(self, arm):
        p = mc.CONFIG_DIR / f"master_zero_{arm}.txt"
        try:
            for line in p.read_text().splitlines():
                if line.strip().startswith("a_hat"):
                    v = np.array([float(x) for x in
                                  line.split(":", 1)[1].replace(",", " ").split()])
                    return v / np.linalg.norm(v)
        except Exception:
            pass
        self.get_logger().warn(
            "[%s] no a_hat captured; elevation cannot be computed and this "
            "arm will publish nothing rather than a guess. Run "
            "`bash scripts/calibrate.sh zero`." % arm)
        return None

    def _on_raw(self, arm, msg):
        d = list(msg.data)
        if len(d) < 13 or self.a_hat[arm] is None:
            return
        accel = np.array(d[7:10], float)
        gyro = np.radians(np.array(d[10:13], float)) - self.bias[arm]
        j1 = math.radians(d[0] - self.j1_zero[arm])

        elev = elevation_from_accel(self.a_hat[arm], accel)
        if elev is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.t_last[arm] is None else min(0.5, now - self.t_last[arm])
        self.t_last[arm] = now

        if self.mode == "j1" or self.az[arm] is None:
            self.az[arm] = j1
            self.drift[arm] = 0.0
            az_conf = 1.0
        else:
            # Yaw rate about the MEASURED vertical. Both vectors are already
            # in the sensor frame, so this needs no mount rotation -- which is
            # why the failed IMU mount calibration is irrelevant here.
            mag = float(np.linalg.norm(accel))
            if abs(mag - 1.0) <= self.gate and mag > 1e-6:
                self.u_hat[arm] = accel / mag
            if self.u_hat[arm] is not None:
                yaw_rate = float(np.dot(gyro, self.u_hat[arm]))
                self.az[arm] += yaw_rate * dt
                self.drift[arm] += abs(yaw_rate) * dt
            # slow pull back to the absolute j1 reading
            k = dt / (self.tau + dt)
            err = math.atan2(math.sin(j1 - self.az[arm]), math.cos(j1 - self.az[arm]))
            self.az[arm] += k * err
            # confidence decays with accumulated |yaw| since the last reset;
            # 90 deg of integrated rotation is the point at which j1 should
            # simply be believed instead.
            az_conf = float(max(0.0, 1.0 - self.drift[arm] / (math.pi / 2)))

        u = pointing_direction(elev, self.az[arm])
        v = Vector3Stamped()
        v.header.stamp = self.get_clock().now().to_msg()
        v.header.frame_id = "world"
        v.vector.x, v.vector.y, v.vector.z = (float(c) for c in u)
        self.vec_pub[arm].publish(v)

        st = String()
        st.data = json.dumps({
            "arm": arm,
            "elevation_deg": elev * DEG,
            "azimuth_deg": self.az[arm] * DEG,
            "azimuth_mode": self.mode,
            "elevation_confidence": 1.0,          # absolute, from gravity
            "azimuth_confidence": round(az_conf, 3),
            "unit_vector": [float(c) for c in u],
        })
        self.st_pub[arm].publish(st)

    def reset_azimuth(self, arm=None):
        """Called on clutch engage: re-anchor azimuth to the absolute j1."""
        for a in ([arm] if arm else self.arms):
            self.az[a] = None
            self.drift[a] = 0.0


def main():
    rclpy.init()
    n = PointingNode()
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
