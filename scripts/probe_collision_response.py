#!/usr/bin/env python3
"""Drive the commanded EE INTO the wearer and watch what the follower does.

    python3 scripts/probe_collision_response.py --arm both

Needs only the sim: move_group, the controllers and ik_follower_node. The
master is not involved -- this publishes /master_arm_pose_<arm> directly, so
there is no clutch and no Teensy in the loop.

WHAT IS BEING TESTED. The commanded pose is ramped from the arm's current EE
straight at a wearer body centre and held there. A correct response is
GRADUATED: redundancy re-seeding and slewing carry the arm as far as it can
safely go, the clearance floor then refuses to publish further, and the arm
HOLDS. Two failures are equally bad:

  * BREACH   -- measured clearance drops below the hard floor
  * LOCKUP   -- the follower stops processing poses altogether, so when the
                command is withdrawn the arm never comes back. Blocking is
                correct; staying blocked after the cause is gone is not.

So the probe pushes in, then RETREATS, and requires the arm to resume.
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

# Wearer primitives, from clearance.py / human_backpack.xacro, expressed in
# world via the torso frame at (0, 0, 1.05).
ZONES = {
    "head": (0.0, 0.0, 1.295),
    "torso": (0.0, 0.0, 1.220),
}


class Probe(Node):
    def __init__(self, arm):
        super().__init__("probe_collision_%s" % arm)
        self.arm = arm
        self.ik = None
        self.create_subscription(Float64MultiArray, "/ik_status_%s" % arm,
                                 self._ik, 10)
        self.pub = self.create_publisher(PoseStamped,
                                         "/master_arm_pose_%s" % arm, 20)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _ik(self, m):
        self.ik = list(m.data)

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

    def counters(self):
        if not self.ik or len(self.ik) < 8:
            return None
        return dict(success=self.ik[0], fail=self.ik[1], clear=self.ik[5],
                    blocks=self.ik[6])


def ramp(n, start, target, seconds, rate=50.0):
    """Walk the command from start to target, sampling clearance."""
    steps = int(seconds * rate)
    worst = float("inf")
    for i in range(steps):
        p = start + (target - start) * (i + 1) / steps
        n.send(p)
        n.spin(1.0 / rate)
        c = n.counters()
        if c and math.isfinite(c["clear"]) and c["clear"] >= 0:
            worst = min(worst, c["clear"])
    return worst


def probe(arm, floor, hold_s, out):
    n = Probe(arm)
    n.spin(3.0)
    start = n.ee()
    if start is None:
        print("  %s: no tf2" % arm)
        return
    print("\n  %s ARM   (start EE %.3f %.3f %.3f)" % (arm.upper(), *start))
    for zone, centre in ZONES.items():
        c0 = n.counters()
        tgt = np.array(centre, float)
        worst_in = ramp(n, start, tgt, 8.0)
        # HOLD the command inside the body: the floor must keep refusing.
        t0 = time.monotonic()
        worst_hold = float("inf")
        while time.monotonic() - t0 < hold_s:
            n.send(tgt)
            n.spin(0.02)
            c = n.counters()
            if c and math.isfinite(c["clear"]) and c["clear"] >= 0:
                worst_hold = min(worst_hold, c["clear"])
        c1 = n.counters()
        # RETREAT and require the arm to resume -- blocking must not latch.
        ramp(n, tgt, start, 6.0)
        n.spin(1.5)
        c2 = n.counters()
        worst = min(worst_in, worst_hold)
        blocks = (c1["blocks"] - c0["blocks"]) if (c0 and c1) else float("nan")
        resumed = (c2["success"] - c1["success"]) if (c1 and c2) else 0
        breach = worst < floor
        status = ("BREACH" if breach else "held") + \
                 ("" if resumed > 0 else "  LOCKED UP")
        print("    %-6s -> min clearance %.4f m   floor %.2f m   "
              "clearance-blocks +%d   IK after retreat +%d   %s"
              % (zone, worst, floor, blocks, resumed, status))
        out.append(dict(arm=arm, zone=zone, worst=worst, floor=floor,
                        blocks=blocks, resumed=resumed,
                        ok=(not breach) and resumed > 0))
    n.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both", choices=["left", "right", "both"])
    ap.add_argument("--floor", type=float, default=0.05,
                    help="ik_follower_node min_clearance_m in sim")
    ap.add_argument("--hold", type=float, default=6.0)
    a = ap.parse_args()
    rclpy.init()
    print("GRADUATED COLLISION RESPONSE PROBE")
    print("commanded EE driven INTO the wearer, held, then withdrawn")
    out = []
    for arm in (["left", "right"] if a.arm == "both" else [a.arm]):
        probe(arm, a.floor, a.hold, out)
    print()
    bad = [o for o in out if not o["ok"]]
    for o in out:
        if o["worst"] < o["floor"]:
            print("  FAIL %s/%s breached the floor (%.4f < %.2f)"
                  % (o["arm"], o["zone"], o["worst"], o["floor"]))
        if o["resumed"] <= 0:
            print("  FAIL %s/%s did not resume after the command withdrew"
                  % (o["arm"], o["zone"]))
    print("  %s" % ("ALL ZONES HELD, NONE LOCKED UP" if not bad
                    else "%d zone(s) FAILED" % len(bad)))
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
