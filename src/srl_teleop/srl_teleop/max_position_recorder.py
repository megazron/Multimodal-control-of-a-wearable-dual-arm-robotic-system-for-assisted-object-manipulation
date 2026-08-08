#!/usr/bin/env python3
"""
max_position_recorder.py — walks you through a fixed sequence of movements,
capturing ONE snapshot (max position reached) per prompt.
=============================================================================
No typing labels. The program tells you what to do; you move the arm to
its max extent in that direction, press ENTER once, and it captures a
single snapshot (raw pot values, computed master pose, sim joint state --
both arms) at that instant. Repeats for the full sequence, then saves
everything into one combined CSV.

Run:
  ros2 run srl_teleop max_position_recorder
"""
import csv
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARMS = ["left", "right"]
SIM_JOINT_NAMES = {a: [f"{a}_joint_{i}" for i in range(1, 8)] for a in ARMS}
RAW_JOINT_NAMES = {a: [f"raw_{'k1j' if a=='left' else 'k2j'}{i}" for i in range(1, 8)]
                   for a in ARMS}

CSV_COLUMNS = ["timestamp", "label"]
for a in ARMS:
    CSV_COLUMNS += RAW_JOINT_NAMES[a] + [f"raw_{a}_ax", f"raw_{a}_ay", f"raw_{a}_az"]
    CSV_COLUMNS += [f"master_{a}_x", f"master_{a}_y", f"master_{a}_z",
                    f"master_{a}_qx", f"master_{a}_qy", f"master_{a}_qz", f"master_{a}_qw"]
    CSV_COLUMNS += [f"sim_{n}" for n in SIM_JOINT_NAMES[a]]

# Fixed sequence of prompts -- edit this list if you want different/more tests.
SEQUENCE = []
for arm in ["left", "right"]:
    for direction in ["UP", "DOWN", "LEFT", "RIGHT", "FRONT"]:
        SEQUENCE.append(f"{arm}_{direction.lower()}")
    for direction in ["GRIPPER_LEFT", "GRIPPER_RIGHT"]:
        SEQUENCE.append(f"{arm}_{direction.lower()}")


class MaxPositionNode(Node):
    def __init__(self):
        super().__init__("max_position_recorder")
        self.latest_pose = {a: None for a in ARMS}
        self.latest_raw = {a: None for a in ARMS}
        self.latest_sim_joints = {
            a: {n: None for n in SIM_JOINT_NAMES[a]} for a in ARMS
        }

        for a in ARMS:
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}",
                lambda msg, arm=a: self.on_pose(arm, msg), 10)
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda msg, arm=a: self.on_raw(arm, msg), 10)

        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)

        self.get_logger().info("max_position_recorder ready.")

    def on_pose(self, arm, msg):
        self.latest_pose[arm] = msg

    def on_raw(self, arm, msg):
        self.latest_raw[arm] = msg.data

    def on_joint_state(self, msg):
        for name, pos in zip(msg.name, msg.position):
            for a in ARMS:
                if name in self.latest_sim_joints[a]:
                    self.latest_sim_joints[a][name] = pos

    def get_row(self, label):
        row = {"timestamp": time.time(), "label": label}
        for a in ARMS:
            raw = self.latest_raw[a]
            if raw is not None:
                for i, name in enumerate(RAW_JOINT_NAMES[a]):
                    row[name] = raw[i]
                row[f"raw_{a}_ax"] = raw[7]
                row[f"raw_{a}_ay"] = raw[8]
                row[f"raw_{a}_az"] = raw[9]
            else:
                for name in RAW_JOINT_NAMES[a] + [f"raw_{a}_ax", f"raw_{a}_ay", f"raw_{a}_az"]:
                    row[name] = ""

            pose = self.latest_pose[a]
            if pose is not None:
                p = pose.pose
                row[f"master_{a}_x"] = p.position.x
                row[f"master_{a}_y"] = p.position.y
                row[f"master_{a}_z"] = p.position.z
                row[f"master_{a}_qx"] = p.orientation.x
                row[f"master_{a}_qy"] = p.orientation.y
                row[f"master_{a}_qz"] = p.orientation.z
                row[f"master_{a}_qw"] = p.orientation.w
            else:
                for suffix in ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                    row[f"master_{a}_{suffix}"] = ""

            for n in SIM_JOINT_NAMES[a]:
                v = self.latest_sim_joints[a][n]
                row[f"sim_{n}"] = v if v is not None else ""
        return row


def main(args=None):
    rclpy.init(args=args)
    node = MaxPositionNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    downloads_dir = os.path.join(os.path.expanduser("~"), "Downloads")
    os.makedirs(downloads_dir, exist_ok=True)

    print("=" * 60)
    print("  MAX POSITION RECORDER")
    print(f"  {len(SEQUENCE)} prompts total. For each: move the arm to")
    print("  its max extent in that direction, then press ENTER.")
    print("  Press Ctrl+C at any point to stop early and save what")
    print("  you've captured so far.")
    print("=" * 60)

    rows = []
    try:
        for i, label in enumerate(SEQUENCE, 1):
            input(f"\n[{i}/{len(SEQUENCE)}] Move {label.replace('_', ' ').upper()} "
                  f"as far as possible, then press ENTER...")
            row = node.get_row(label)
            rows.append(row)
            print(f"  Captured '{label}'.")

    except KeyboardInterrupt:
        print("\nStopped early.")

    finally:
        if rows:
            filename = f"max_positions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            filepath = os.path.join(downloads_dir, filename)
            with open(filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)
            print(f"\nSaved {len(rows)} snapshot(s) to {filepath}")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
