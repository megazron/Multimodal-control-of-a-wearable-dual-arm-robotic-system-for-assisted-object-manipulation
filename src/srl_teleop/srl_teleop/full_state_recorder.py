#!/usr/bin/env python3
"""
full_state_recorder.py — guided snapshot recorder capturing EVERYTHING:
pots (raw joint degrees), FSR, buttons, computed master pose, and sim
joint state -- both arms.
=============================================================================
Same workflow as max_position_recorder.py: the program tells you what to
do, you do it, press ENTER once, it captures a single snapshot. Now also
includes gripper (FSR) and button prompts, not just arm movement.

Run:
  ros2 run srl_teleop full_state_recorder
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
CSV_COLUMNS += ["fsr1", "fsr2", "btn1", "btn2"]

# Fixed sequence -- arm movements (as before) PLUS gripper/button prompts.
# fsr1/btn1 = left arm's gripper/button; fsr2/btn2 = right arm's.
SEQUENCE = []
for arm in ["left", "right"]:
    for direction in ["UP", "DOWN", "LEFT", "RIGHT", "FRONT"]:
        SEQUENCE.append(("move", f"{arm}_{direction.lower()}"))
    for direction in ["GRIPPER_LEFT", "GRIPPER_RIGHT"]:
        SEQUENCE.append(("move", f"{arm}_{direction.lower()}"))
    SEQUENCE.append(("fsr_open", f"{arm}_fsr_open"))
    SEQUENCE.append(("fsr_close", f"{arm}_fsr_close"))
    SEQUENCE.append(("button", f"{arm}_button_press"))


PROMPT_TEXT = {
    "move": "Move {desc} as far as possible",
    "fsr_open": "OPEN the {desc} gripper (release the FSR sensor)",
    "fsr_close": "CLOSE/squeeze the {desc} gripper (press the FSR sensor hard)",
    "button": "PRESS the {desc} button",
}


class FullStateNode(Node):
    def __init__(self):
        super().__init__("full_state_recorder")
        self.latest_pose = {a: None for a in ARMS}
        self.latest_raw = {a: None for a in ARMS}
        self.latest_sim_joints = {
            a: {n: None for n in SIM_JOINT_NAMES[a]} for a in ARMS
        }
        self.latest_fsr_btn = None  # [fsr1, fsr2, btn1, btn2]

        for a in ARMS:
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}",
                lambda msg, arm=a: self.on_pose(arm, msg), 10)
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda msg, arm=a: self.on_raw(arm, msg), 10)

        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons", self.on_fsr_btn, 10)
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)

        self.get_logger().info("full_state_recorder ready.")

    def on_pose(self, arm, msg):
        self.latest_pose[arm] = msg

    def on_raw(self, arm, msg):
        self.latest_raw[arm] = msg.data

    def on_fsr_btn(self, msg):
        self.latest_fsr_btn = msg.data

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

        if self.latest_fsr_btn is not None:
            row["fsr1"] = self.latest_fsr_btn[0]
            row["fsr2"] = self.latest_fsr_btn[1]
            row["btn1"] = self.latest_fsr_btn[2]
            row["btn2"] = self.latest_fsr_btn[3]
        else:
            for name in ["fsr1", "fsr2", "btn1", "btn2"]:
                row[name] = ""

        return row


def main(args=None):
    rclpy.init(args=args)
    node = FullStateNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    downloads_dir = os.path.join(os.path.expanduser("~"), "Downloads")
    os.makedirs(downloads_dir, exist_ok=True)

    print("=" * 60)
    print("  FULL STATE RECORDER (pots, FSR, buttons, pose, sim joints)")
    print(f"  {len(SEQUENCE)} prompts total.")
    print("  Ctrl+C at any point saves what you've captured so far.")
    print("=" * 60)

    rows = []
    try:
        for i, (kind, label) in enumerate(SEQUENCE, 1):
            desc = label.replace("_", " ").upper()
            prompt = PROMPT_TEXT[kind].format(desc=desc)
            input(f"\n[{i}/{len(SEQUENCE)}] {prompt}, then press ENTER...")
            row = node.get_row(label)
            rows.append(row)
            print(f"  Captured '{label}'.")

    except KeyboardInterrupt:
        print("\nStopped early.")

    finally:
        if rows:
            filename = f"full_state_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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
