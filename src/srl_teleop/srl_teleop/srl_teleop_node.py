#!/usr/bin/env python3
"""
srl_teleop_node.py — Combined SRL teleoperation (ROS2)
=======================================================
Homing + countdown + pot teleop, publishing to BOTH arm controllers.

On startup, ASKS you to choose a mode in the terminal:
  1. SIM  — fake serial topic (/fake_serial), test without hardware
  2. REAL — real Teensy serial (/dev/ttyACM0)
  3. BOTH — real Teensy, but you're also running sim (arms mirror)

Publishes to the MoveIt-generated controllers:
  /left_arm_controller/joint_trajectory
  /right_arm_controller/joint_trajectory

RELATIVE (zero-reference) POT MAPPING:
  A pot has no idea what angle the arm is at when it powers on, so we
  never map pot value -> joint angle directly (that causes a jump: arm
  at 90 deg, pot happens to read 0, turning pot to 90 would send the
  arm to 180). Instead, on the FIRST valid reading of each pot AFTER
  teleop starts, that raw value is captured as the pot's zero
  reference and the joint's CURRENT angle (= home) is captured as its
  reference too. Every reading after that applies only the CHANGE:
      target = joint_ref + (pot_now - pot_zero)
  wrap-safe via deg_wrap_diff(). No jump, ever, at any power-on pot
  position.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from srl_teleop.serial_port import find_port
import math
import sys

# ══════════════════════════════════════════════════════════════════════════════
#  HOME POSITIONS — real Web App values (0-360 deg)
# ══════════════════════════════════════════════════════════════════════════════
LEFT_HOME_DEG  = [259.03, 277.69, 267.74, 286.14, 194.10, 27.48, 55.26]
RIGHT_HOME_DEG = [303.65, 77.06, 98.57, 58.57, 317.14, 36.39, 154.71]

# ══════════════════════════════════════════════════════════════════════════════
#  CONTROLLER TOPICS — match MoveIt Setup Assistant output
# ══════════════════════════════════════════════════════════════════════════════
LEFT_TOPIC  = "/left_arm_controller/joint_trajectory"
RIGHT_TOPIC = "/right_arm_controller/joint_trajectory"

# ══════════════════════════════════════════════════════════════════════════════
#  POT CONFIG
# ══════════════════════════════════════════════════════════════════════════════
SINGLE_POT_MODE = True
ACTIVE_POT   = "k1j7"
ACTIVE_ARM   = "right"
ACTIVE_JOINT = 7
MULTI_POTS = {
    "k1j7": ("right", 6),
}

POT_NOISE_FLOOR = 3.0
COUNTDOWN       = 10


def deg_to_rad(deg):
    d = deg - 360.0 if deg > 180.0 else deg
    return math.radians(d)


def deg_wrap_diff(a, b):
    """Shortest signed difference a-b in degrees, wrapped to [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def choose_mode():
    """Ask the user which mode to run in."""
    print("\n" + "=" * 50)
    print("  SRL Teleoperation — choose mode:")
    print("=" * 50)
    print("  1. SIM   — fake pot (test in RViz, no hardware)")
    print("  2. REAL  — real Teensy serial (/dev/ttyACM0)")
    print("  3. BOTH  — real Teensy + running sim together")
    print("=" * 50)
    while True:
        choice = input("Enter 1, 2, or 3: ").strip()
        if choice == "1":
            return "sim"
        elif choice == "2":
            return "real"
        elif choice == "3":
            return "both"
        print("Invalid — type 1, 2, or 3.")


class SrlTeleop(Node):
    def __init__(self, mode):
        super().__init__("srl_teleop")
        self.mode = mode

        # "auto" sniffs for the board (it moves between ACM0/ACM1 on every
        # usbipd re-attach); an explicit path bypasses detection.
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("do_homing", True)
        self.do_homing = self.get_parameter("do_homing").value

        self.home = {
            "left":  [deg_to_rad(d) for d in LEFT_HOME_DEG],
            "right": [deg_to_rad(d) for d in RIGHT_HOME_DEG],
        }
        self.target = {
            "left":  list(self.home["left"]),
            "right": list(self.home["right"]),
        }

        # ── Relative pot calibration state (per pot tag) ──────────────────
        # pot_zero[tag]  = raw pot value (deg) at first valid sample
        # joint_ref[tag] = joint angle (rad) at that same moment (= home)
        self.pot_zero = {}
        self.joint_ref = {}

        if SINGLE_POT_MODE:
            self.connected_pots = {ACTIVE_POT: (ACTIVE_ARM, ACTIVE_JOINT - 1)}
        else:
            self.connected_pots = dict(MULTI_POTS)

        self.joint_names = {
            "left":  [f"left_joint_{i+1}"  for i in range(7)],
            "right": [f"right_joint_{i+1}" for i in range(7)],
        }

        self.pub = {
            "left":  self.create_publisher(JointTrajectory, LEFT_TOPIC, 10),
            "right": self.create_publisher(JointTrajectory, RIGHT_TOPIC, 10),
        }

        self.homing_done = not self.do_homing
        self.countdown = COUNTDOWN

        # Input source depends on mode
        self.ser = None
        if self.mode == "sim":
            # fake pot topic
            self.create_subscription(String, "/fake_serial",
                                     self.on_fake_line, 10)
            self.get_logger().info("MODE: SIM — input from /fake_serial")
        else:
            # real or both → real Teensy serial
            # Fails loudly rather than silently holding home -- choose mode
            # 1/SIM to run deliberately without a Teensy.
            import serial
            baud = self.get_parameter("baud_rate").value
            port = find_port(self.get_parameter("serial_port").value, baud,
                             logger=self.get_logger())
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.create_timer(0.01, self.read_serial)
            self.get_logger().info(
                f"MODE: {self.mode.upper()} — real serial {port} @ {baud}")

        self.create_timer(0.05, self.publish_targets)

        if self.do_homing:
            self.create_timer(1.0, self.countdown_tick)
            self.get_logger().info(f"HOMING — teleop starts in {COUNTDOWN}s.")
        else:
            self.get_logger().info("Homing skipped — teleop active now.")

        m = "SINGLE-POT" if SINGLE_POT_MODE else "MULTI-POT"
        self.get_logger().info(f"Started ({m}). Pots: {self.connected_pots}")
        self.get_logger().info("Other joints LOCKED at home.")

    def countdown_tick(self):
        if self.homing_done:
            return
        if self.countdown > 0:
            self.get_logger().info(f"  teleop in {self.countdown} ...")
            self.countdown -= 1
        else:
            self.homing_done = True
            self.get_logger().info("TELEOP ACTIVE — move the pot.")
            self.get_logger().info(
                "NOTE: each pot's CURRENT position is captured as its zero "
                "point on its first reading now — hold it steady/neutral.")

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
        if not self.homing_done:
            return
        pots = self.parse_line(line)
        for tag, (arm, jidx) in self.connected_pots.items():
            if tag not in pots:
                continue
            raw = pots[tag]

            # Ignore noise floor / disconnected reads — hold last target
            if raw < POT_NOISE_FLOOR:
                continue

            # ── CALIBRATION: first valid sample after teleop = zero point ──
            # No motion on this sample; it only sets the reference so the
            # very next reading's delta starts at 0.
            if tag not in self.pot_zero:
                self.pot_zero[tag] = raw
                self.joint_ref[tag] = self.target[arm][jidx]   # == home
                self.get_logger().info(
                    f"[CAL] {tag}: pot {raw:.1f} -> {arm} J{jidx+1} "
                    f"@ {math.degrees(self.joint_ref[tag]):.1f} deg (home)")
                continue

            # ── RELATIVE mapping: home + shortest signed pot delta ─────────
            delta_deg = deg_wrap_diff(raw, self.pot_zero[tag])
            # Same gimbal-lock guard used elsewhere in the project.
            delta_deg = max(-175.0, min(175.0, delta_deg))
            self.target[arm][jidx] = self.joint_ref[tag] + math.radians(delta_deg)

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
            if self.homing_done:
                pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
            else:
                pt.time_from_start = Duration(sec=2, nanosec=0)
            msg.points = [pt]
            self.pub[arm].publish(msg)


def main(args=None):
    # Ask mode BEFORE starting ROS (clean terminal prompt)
    mode = choose_mode()

    rclpy.init(args=args)
    node = SrlTeleop(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
