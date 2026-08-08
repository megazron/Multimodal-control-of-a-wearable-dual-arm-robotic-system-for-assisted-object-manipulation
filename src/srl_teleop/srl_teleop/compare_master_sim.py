#!/usr/bin/env python3
"""
compare_master_sim.py — records MASTER pose vs SIMULATOR joint state for
BOTH arms simultaneously, side by side, for labeled test movements.
=============================================================================
Captures both left AND right arm data in every sample -- so whichever arm
you actually move during a test, its data is there, and we can also see
whether moving one arm affects the OTHER arm's readings (crosstalk check).

For each labeled test (type what you're about to do, e.g. "right_arm_up"),
press ENTER to start recording, do the movement, press ENTER again to
stop. Saved as its own CSV file per labeled test, in your Downloads folder.

Uses keyboard ENTER (not the physical button) as the start/stop trigger --
the physical button's state comes over serial, which master_pose_node.py
already holds exclusively; opening a second serial connection here would
recreate the same port-contention problem fixed earlier.

Run:
  ros2 run srl_teleop compare_master_sim
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


class ComparisonNode(Node):
    def __init__(self):
        super().__init__("compare_master_sim")
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

        self.get_logger().info(
            "compare_master_sim ready -- capturing BOTH arms.")

    def on_pose(self, arm, msg):
        self.latest_pose[arm] = msg

    def on_raw(self, arm, msg):
        self.latest_raw[arm] = msg.data  # [jN..jN deg (7), ax, ay, az]

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
    node = ComparisonNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0)  # let subscriptions connect before we start prompting

    downloads_dir = os.path.join(os.path.expanduser("~"), "Downloads")
    os.makedirs(downloads_dir, exist_ok=True)
    print(f"Each labeled test saves as its own file in: {downloads_dir}\n")

    print("=" * 60)
    print("  MASTER vs SIMULATOR COMPARISON RECORDER (both arms)")
    print("  Type a label, press ENTER to start recording,")
    print("  do the movement, press ENTER again to stop.")
    print("  Type 'quit' instead of a label to exit.")
    print("=" * 60)

    try:
        while True:
            label = input("\nLabel for this test (or 'quit'): ").strip()
            if label.lower() in ("quit", "exit"):
                break
            if not label:
                print("  Please enter a label.")
                continue

            rows = []
            recording = threading.Event()

            def record_loop():
                while recording.is_set():
                    rows.append(node.get_row(label))
                    time.sleep(0.05)  # ~20 Hz sampling

            input(f"  Ready. Press ENTER to START recording '{label}'...")
            recording.set()
            rec_thread = threading.Thread(target=record_loop, daemon=True)
            rec_thread.start()

            input(f"  >>> RECORDING '{label}' -- do the movement now, "
                  f"then press ENTER to STOP...")
            recording.clear()
            rec_thread.join()

            safe_label = "".join(
                c if c.isalnum() or c in "_-" else "_" for c in label)
            filename = (f"compare_{safe_label}_"
                        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            filepath = os.path.join(downloads_dir, filename)
            with open(filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)

            print(f"  <<< STOPPED. Captured {len(rows)} samples.")
            print(f"  Saved to {filepath}")

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
