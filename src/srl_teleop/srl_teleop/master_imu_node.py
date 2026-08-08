#!/usr/bin/env python3
"""
master_imu_node.py — IMU-PRIMARY orientation for the master arm.

Consumes /master_arm_raw_<arm> ([j1..j7 deg, ax, ay, az, gx, gy, gz]) and runs
a complementary filter per arm. Publishes a proper sensor_msgs/Imu with a
fused orientation, plus the two quantities the rest of the system actually
wants: elevation (absolute) and the filter's health.

TWO-IMU PATH, DEFAULT OFF. `two_imu` adds an upper-arm sensor and derives the
ELBOW ANGLE from the two segments' relative orientation, with no pot at all.
It stays off until a second sensor is physically present, and it degrades to
the pot silently rather than publishing an elbow angle from a sensor that is
not there.

  WIRING for the second IMU. The MPU-6050/9250 family has exactly TWO
  I2C addresses, 0x68 and 0x69, selected by the AD0 pin, and BOTH ARE
  ALREADY TAKEN on this rig (one IMU per arm). A third sensor therefore
  cannot simply be added to the same bus. The options, in increasing order of
  work:
    1. SECOND I2C BUS. The Teensy 4.1 has three (Wire, Wire1, Wire2).
       Wire1 is pins 16/17, Wire2 is 24/25. Put the two new upper-arm IMUs on
       Wire1 at 0x68 and 0x69 and change nothing else electrically. This is
       the recommended route: no extra parts, and the buses stay short.
    2. TCA9548A I2C multiplexer on the existing bus: one part, address
       0x70-0x77, eight channels, each carrying a 0x68/0x69 pair. Costs an
       extra transaction per read, which at 50 Hz is irrelevant.
    3. A sensor with more address options (e.g. BNO055 at 0x28/0x29), which
       also moves the fusion onto the sensor and out of this file.
  The frame format must gain the second sensor's fields; this node reads
  them from indices 13..18 of /master_arm_raw_<arm> when `two_imu` is true,
  and reports "second IMU expected but frame is too short" rather than
  silently using the first sensor twice.
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

from srl_teleop import master_calibration as mc
from srl_teleop.imu_orientation import (ComplementaryFilter, elbow_angle_from_two,
                                        elevation_from_accel, q_to_rpy)

ARMS = ("left", "right")
DEG = 180.0 / math.pi


class MasterImuNode(Node):
    def __init__(self):
        super().__init__("master_imu_node")
        self.declare_parameter("arms", ["left", "right"])
        # Complementary crossover. Below tau the gyro rules (good through fast
        # motion), above it the accelerometer rules (bounds drift).
        self.declare_parameter("tau_s", 1.0)
        # |accel| outside 1 g +/- this is contaminated by the arm's own
        # acceleration and is not gravity; the correction is suspended.
        self.declare_parameter("accel_gate_g", 0.15)
        self.declare_parameter("two_imu", False)
        self.declare_parameter("publish_rate_hz", 50.0)
        # Per-arm gyro bias, rad/s. MUST be captured from a SETTLED stream:
        # reopening the serial port resets the Teensy and the IMU emits
        # nonsense while settling, which once cost a factor of 34 in measured
        # drift. Use capture_gyro_bias against /master_arm_raw_*, not the port.
        for a in ARMS:
            self.declare_parameter(f"{a}_gyro_bias", [0.0, 0.0, 0.0])

        self.arms = [a for a in self.get_parameter("arms").value if a in ARMS]
        self.tau = float(self.get_parameter("tau_s").value)
        self.two_imu = bool(self.get_parameter("two_imu").value)

        self.filt, self.filt2, self.a_hat = {}, {}, {}
        for a in self.arms:
            b = [float(v) for v in self.get_parameter(f"{a}_gyro_bias").value]
            self.filt[a] = ComplementaryFilter(self.tau,
                                               float(self.get_parameter("accel_gate_g").value),
                                               b)
            self.filt2[a] = ComplementaryFilter(self.tau,
                                                float(self.get_parameter("accel_gate_g").value))
            self.a_hat[a] = self._load_a_hat(a)

        self.imu_pub = {a: self.create_publisher(Imu, f"/master_imu_{a}", 10)
                        for a in self.arms}
        self.state_pub = {a: self.create_publisher(String, f"/master_imu_state_{a}", 10)
                          for a in self.arms}
        self.elbow_pub = {a: self.create_publisher(
            Float64MultiArray, f"/master_elbow_{a}", 10) for a in self.arms}
        self.t_last = {a: None for a in self.arms}
        self.short_frame_warned = set()
        for a in self.arms:
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda m, a=a: self._on_raw(a, m), 20)

        self.get_logger().info(
            "master_imu_node up: tau=%.2f s, gate=%.2f g, two_imu=%s. "
            "Accelerometer gives roll/pitch absolutely; YAW IS GYRO-ONLY and "
            "is published with its accumulated drift so a consumer can "
            "discount it." % (self.tau, self.filt[self.arms[0]].gate, self.two_imu))

    def _load_a_hat(self, arm):
        """Arm long axis in the sensor frame, from the zero capture."""
        try:
            z = mc.load_zero(arm) if hasattr(mc, "load_zero") else None
            if z and "a_hat" in z:
                v = np.asarray(z["a_hat"], float)
                return v / np.linalg.norm(v)
        except Exception:
            pass
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
            "[%s] no a_hat in config/master_zero_%s.txt; elevation will be "
            "published as NaN rather than guessed. Run "
            "`bash scripts/calibrate.sh zero`." % (arm, arm))
        return None

    def _on_raw(self, arm, msg):
        d = list(msg.data)
        if len(d) < 13:
            return
        accel = np.array(d[7:10], float)
        gyro = np.radians(np.array(d[10:13], float))
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.02 if self.t_last[arm] is None else now - self.t_last[arm]
        self.t_last[arm] = now

        q = self.filt[arm].update(accel, gyro, dt)
        roll, pitch, yaw = q_to_rpy(q)

        elev = float("nan")
        if self.a_hat[arm] is not None:
            e = elevation_from_accel(self.a_hat[arm], accel)
            if e is not None:
                elev = e

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f"master_{arm}_wrist"
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = \
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        # Roll and pitch are observable from gravity; YAW IS NOT. A large
        # yaw variance is the honest way to say so in a standard message,
        # and consumers that read the covariance will do the right thing.
        m.orientation_covariance = [1e-4, 0.0, 0.0,
                                    0.0, 1e-4, 0.0,
                                    0.0, 0.0, 1e6]
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
            float(gyro[0]), float(gyro[1]), float(gyro[2])
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
            (float(v) * 9.80665 for v in accel)
        self.imu_pub[arm].publish(m)

        elbow = float("nan")
        elbow_src = "j4_pot"
        if self.two_imu:
            if len(d) >= 19:
                a2 = np.array(d[13:16], float)
                g2 = np.radians(np.array(d[16:19], float))
                q2 = self.filt2[arm].update(a2, g2, dt)
                elbow = elbow_angle_from_two(q2, q)
                elbow_src = "two_imu"
            elif arm not in self.short_frame_warned:
                self.short_frame_warned.add(arm)
                self.get_logger().warn(
                    "[%s] two_imu is enabled but the frame carries %d values, "
                    "not the 19 a second IMU needs. Falling back to the j4 "
                    "pot. See the wiring note in master_imu_node.py." %
                    (arm, len(d)))
        em = Float64MultiArray()
        em.data = [elbow, 1.0 if elbow_src == "two_imu" else 0.0]
        self.elbow_pub[arm].publish(em)

        st = String()
        st.data = json.dumps({
            "arm": arm,
            "roll_deg": roll * DEG, "pitch_deg": pitch * DEG,
            "yaw_deg": yaw * DEG,
            "yaw_is_gyro_only": True,
            "yaw_drift_deg": self.filt[arm].yaw_drift * DEG,
            "elevation_deg": elev * DEG if elev == elev else None,
            "accel_gated_fraction": round(self.filt[arm].gated_fraction, 4),
            "tau_s": self.tau,
            "elbow_deg": elbow * DEG if elbow == elbow else None,
            "elbow_source": elbow_src,
        })
        self.state_pub[arm].publish(st)


def main():
    rclpy.init()
    n = MasterImuNode()
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
