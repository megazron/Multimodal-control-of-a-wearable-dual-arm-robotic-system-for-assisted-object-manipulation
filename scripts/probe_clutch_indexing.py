#!/usr/bin/env python3
"""Is the reachable set UNBOUNDED across repeated clutch cycles?

    python3 scripts/probe_clutch_indexing.py --arm both --cycles 8

Sim only. The point of a clutch is indexing: move to the edge of comfortable
master travel, disengage, bring your arm back, re-engage, continue. If that
works, the robot's reachable set is bounded by the ROBOT, not by one ball of
master travel around home. If it does not, the operator is trapped in a
0.27 m ball no matter how many times they re-index.

WHAT THIS DOES AND DOES NOT TEST. The clutch state machine (button edges,
quasi-static gate, averaged reference) lives in master_pose_node and needs a
Teensy and physical button presses, so it cannot run unattended. What CAN be
tested without any of that is the property that actually matters downstream:
that successive anchored offsets accumulate, so commanding
home + 1*d, then home + 2*d, ... walks the arm outward instead of snapping it
back. That is exactly what a re-engage produces -- a fresh reference with the
anchor left where the arm already is.

Each "cycle" commands one master-ball worth of displacement, waits for the
arm to arrive, and then treats the arrival point as the new origin.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import tf2_ros


class Index(Node):
    def __init__(self, arm):
        super().__init__("probe_index_%s" % arm)
        self.arm = arm
        self.ik = None
        self.create_subscription(Float64MultiArray, "/ik_status_%s" % arm,
                                 lambda m: setattr(self, "ik", list(m.data)), 10)
        self.pub = self.create_publisher(PoseStamped,
                                         "/master_arm_pose_%s" % arm, 20)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def spin(self, t):
        t0 = time.monotonic()
        while time.monotonic() - t0 < t and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def ee(self):
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % self.arm, rclpy.time.Time())
            v = t.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:                                        # noqa: BLE001
            return None

    def send(self, p):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def glide(self, a, b, seconds, rate=50.0):
        steps = max(1, int(seconds * rate))
        for i in range(steps):
            self.send(a + (b - a) * (i + 1) / steps)
            self.spin(1.0 / rate)


def run(arm, cycles, ball, direction):
    n = Index(arm)
    n.spin(3.0)
    origin = n.ee()
    if origin is None:
        print("  %s: no tf2" % arm)
        return None
    d = np.array(direction, float)
    d /= np.linalg.norm(d)
    print("\n  %s ARM   start %.3f %.3f %.3f   indexing %.2f m per cycle "
          "along (%.1f %.1f %.1f)"
          % (arm.upper(), *origin, ball, *d))
    print("    cycle   commanded_total   EE_total_from_start   step_gain")
    ref = origin.copy()
    total_cmd = 0.0
    prev = origin.copy()
    rows = []
    for c in range(1, cycles + 1):
        # One clutch cycle: engage HERE, sweep one ball, disengage.
        target = ref + d * ball
        n.glide(ref, target, 3.0)
        n.spin(1.0)
        now = n.ee()
        total_cmd += ball
        moved_total = float(np.linalg.norm(now - origin))
        step = float(np.linalg.norm(now - prev))
        rows.append((c, total_cmd, moved_total, step))
        print("    %5d %15.3f %20.3f %12.3f"
              % (c, total_cmd, moved_total, step))
        # RE-ENGAGE: the new reference is wherever the arm actually is. This
        # is the re-basing a real clutch performs; without it the next cycle
        # would command from the old origin and simply repeat itself.
        prev = now.copy()
        ref = now.copy()
    n.destroy_node()
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both", choices=["left", "right", "both"])
    ap.add_argument("--cycles", type=int, default=8)
    ap.add_argument("--ball", type=float, default=0.12,
                    help="EE displacement per clutch cycle, metres")
    ap.add_argument("--dir", default="0,-1,0",
                    help="world direction to index along (default: back)")
    a = ap.parse_args()
    direction = [float(x) for x in a.dir.split(",")]
    rclpy.init()
    print("CLUTCH INDEXING PROBE -- is the reachable set unbounded?")
    ok = True
    for arm in (["left", "right"] if a.arm == "both" else [a.arm]):
        rows = run(arm, a.cycles, a.ball, direction)
        if not rows:
            ok = False
            continue
        totals = [r[2] for r in rows]
        one_ball = a.ball
        grew = totals[-1] > 2.0 * one_ball
        # Growth must continue past the first cycle, not saturate immediately.
        late = [r[3] for r in rows[len(rows) // 2:]]
        still = sum(1 for s in late if s > 0.2 * one_ball)
        print("    total EE travel over %d cycles: %.3f m  (one ball = %.2f m)"
              % (len(rows), totals[-1], one_ball))
        print("    cycles in the second half still advancing: %d of %d"
              % (still, len(late)))
        if grew and still:
            print("    UNBOUNDED: travel exceeds a single ball and keeps "
                  "advancing -- indexing works")
        else:
            ok = False
            print("    BOUNDED: the arm saturated near one ball. Re-engaging "
                  "does not extend reach, so the operator cannot index.")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
