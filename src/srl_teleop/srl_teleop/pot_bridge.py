#!/usr/bin/env python3
"""
pot_bridge.py DUAL ARM
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
import math


class PotBridge(Node):
    def __init__(self):
        super().__init__("pot_bridge")
        self.declare_parameter("use_fake", False)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.use_fake = self.get_parameter("use_fake").value
        self.home = {
            "left":  [-1.7623, -1.4366, -1.6102, -1.2891, -2.8955, 0.4796, 0.9645],
            "right": [-0.9835,  1.345,   1.7204,  1.0222, -0.748,  0.6351, 2.7002],
        }
        self.target = {"left": list(self.home["left"]), "right": list(self.home["right"])}
        self.connected_pots = {"k1j7": ("right", 3)}
        self.pot_noise_floor = 3.0
        self.joint_names = {
            "left":  [f"left_joint_{i+1}"  for i in range(7)],
            "right": [f"right_joint_{i+1}" for i in range(7)],
        }
        self.pub = {
            "left":  self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10),
            "right": self.create_publisher(JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10),
        }
        self.ser = None
        if self.use_fake:
            self.create_subscription(String, "/fake_serial", self.on_fake_line, 10)
            self.get_logger().info("Input: FAKE topic /fake_serial")
        else:
            try:
                import serial
                port = self.get_parameter("serial_port").value
                baud = self.get_parameter("baud_rate").value
                self.ser = serial.Serial(port, baud, timeout=0.01)
                self.create_timer(0.01, self.read_serial)
                self.get_logger().info(f"Input: REAL serial {port} @ {baud}")
            except Exception as e:
                self.get_logger().warn(f"Serial failed ({e}) holding home")
        self.create_timer(0.05, self.publish_targets)
        self.get_logger().info("pot_bridge DUAL started.")
        self.get_logger().info(f"Connected pots: {self.connected_pots}")

    def raw_to_rad(self, raw_deg):
        if raw_deg < self.pot_noise_floor:
            return None
        deg = raw_deg - 180.0
        deg = max(-175.0, min(175.0, deg))
        return math.radians(deg)

    def parse_line(self, line):
        out = {}
        for token in line.strip().lower().split(","):
            if ":" in token:
                tag, _, val = token.partition(":")
                try:
                    out[tag.strip()] = float(val)
                except ValueError:
                    pass
        return out

    def apply_pots(self, line):
        pots = self.parse_line(line)
        for tag, (arm, jidx) in self.connected_pots.items():
            if tag not in pots:
                continue
            rad = self.raw_to_rad(pots[tag])
            if rad is None:
                continue
            self.target[arm][jidx] = rad

    def read_serial(self):
        if self.ser is None:
            return
        try:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            return
        if line:
            self.apply_pots(line)

    def on_fake_line(self, msg):
        self.apply_pots(msg.data)

    def publish_targets(self):
        for arm in ("left", "right"):
            msg = JointTrajectory()
            msg.joint_names = self.joint_names[arm]
            pt = JointTrajectoryPoint()
            pt.positions = list(self.target[arm])
            pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
            msg.points = [pt]
            self.pub[arm].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
