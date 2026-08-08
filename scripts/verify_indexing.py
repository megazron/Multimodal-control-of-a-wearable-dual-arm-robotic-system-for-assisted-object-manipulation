#!/usr/bin/env python3
"""PART 2 VERIFICATION: 10 clutch index cycles, jump per cycle, total travel.

    python3 scripts/verify_indexing.py --cycles 10

Requires a stack running against the VIRTUAL TEENSY, because the cycle needs
deterministic button edges and a deterministic master trajectory -- neither
of which a human at a mannequin can supply repeatably.

WHAT IS BEING TESTED, precisely:

  engage -> move the master to its limit -> press (FREEZE) -> reposition the
  master freely -> press (RESUME from where it froze, new master pose becomes
  the reference) -> repeat.

Two numbers decide it:

  RE-ENGAGE JUMP   the commanded EE displacement across the resume edge. It
                   must be ~0: the arm resumes from where it froze, and the
                   master's new pose is the new reference. Anything else is
                   the operator's repositioning leaking into the command.

  TOTAL TRAVEL     must keep growing with cycles. If it saturates, indexing
                   is not indexing and the reachable set is bounded by the
                   master's range -- which is the whole reason indexing
                   exists.

The master trajectory is driven by writing directly to the virtual board's
pty, so the input is exactly known.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Watch(Node):
    def __init__(self, arm):
        super().__init__("verify_indexing")
        self.arm = arm
        self.pose = []
        self.clutch = []
        self.reach = []
        self.create_subscription(
            PoseStamped, "/master_arm_pose_%s" % arm, self._pose, 50)
        self.create_subscription(
            Float64MultiArray, "/master_status_%s" % arm, self._status, 50)

    def _pose(self, m):
        p = m.pose.position
        self.pose.append((time.monotonic(), p.x, p.y, p.z))

    def _status(self, m):
        if m.data:
            self.clutch.append((time.monotonic(), float(m.data[0])))
        if len(m.data) > 4:
            self.reach.append((time.monotonic(), float(m.data[4])))

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

    def at(self, t):
        """Commanded position nearest a timestamp."""
        if not self.pose:
            return None
        i = min(range(len(self.pose)), key=lambda k: abs(self.pose[k][0] - t))
        return np.array(self.pose[i][1:], float)

    def reach_at(self, t):
        r = [v for (ts, v) in self.reach if ts <= t]
        return r[-1] if r else float("nan")

    def clutch_at(self, t):
        c = [v for (ts, v) in self.clutch if ts <= t]
        return c[-1] if c else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=10)
    ap.add_argument("--arm", default="left")
    ap.add_argument("--portfile", default="/tmp/virtual_teensy_port")
    ap.add_argument("--hold", type=float, default=1.2)
    a = ap.parse_args()

    if not os.path.exists(a.portfile):
        print("no %s -- start scripts/virtual_teensy.py first" % a.portfile)
        return 2
    port = open(a.portfile).read().strip()
    ctl = os.environ.get("VT_CTL", "/tmp/virtual_teensy_ctl")

    rclpy.init()
    n = Watch(a.arm)
    n.spin(3.0)
    if not n.pose:
        print("no /master_arm_pose_%s -- is the stack up on %s?"
              % (a.arm, port))
        return 2

    def cmd(s):
        with open(ctl, "w") as f:
            f.write(s)
        time.sleep(0.05)

    print("INDEXING CHECK -- %d cycles, arm=%s" % (a.cycles, a.arm))
    print("  %-6s %14s %14s %16s" %
          ("cycle", "moved (mm)", "jump (mm)", "cumulative (mm)"))

    btn = 2 if a.arm == "left" else 1
    jumps, moved, cum = [], [], 0.0
    # start engaged, reference already latched at boot
    cmd("j7=0")
    n.spin(1.0)
    for c in range(a.cycles):
        t_a = time.monotonic()
        p0 = n.at(t_a)
        # 1. drive the master out to its limit on the reach channel
        cmd("j7=+22")
        n.spin(a.hold)
        t_b = time.monotonic()
        p1 = n.at(t_b)
        d_move = float(np.linalg.norm(p1 - p0)) * 1000.0
        # 2. press -> FREEZE
        cmd("btn%d=1" % btn)
        n.spin(0.25)
        cmd("btn%d=0" % btn)
        n.spin(0.6)
        p_frozen = n.at(time.monotonic())
        # 3. reposition the master freely while frozen
        cmd("j7=-22")
        n.spin(a.hold)
        p_still = n.at(time.monotonic())
        drift = float(np.linalg.norm(p_still - p_frozen)) * 1000.0
        # 4. press -> RESUME, new master pose becomes the reference
        cmd("btn%d=1" % btn)
        n.spin(0.25)
        cmd("btn%d=0" % btn)
        n.spin(1.2)                    # quasi-static gate + 5-frame average
        p_res = n.at(time.monotonic())
        jump = float(np.linalg.norm(p_res - p_frozen)) * 1000.0
        jumps.append(jump)
        moved.append(d_move)
        cum += d_move
        print("  %-6d %14.2f %14.3f %16.2f   reach %.3f->%.3f->%.3f m  "
              "clutch %s->%s  (drift %.3f mm)"
              % (c + 1, d_move, jump, cum,
                 n.reach_at(t_a), n.reach_at(t_b), n.reach_at(time.monotonic()),
                 n.clutch_at(t_b), n.clutch_at(time.monotonic()), drift))

    j = np.array(jumps)
    m = np.array(moved)
    print("\n  RE-ENGAGE JUMP  mean %.3f mm   max %.3f mm   (target ~0)"
          % (j.mean(), j.max()))
    print("  TOTAL TRAVEL    %.1f mm over %d cycles" % (cum, a.cycles))
    second = m[len(m) // 2:].sum()
    print("  second half contributed %.1f mm -- %s"
          % (second,
             "UNBOUNDED (still advancing)" if second > 0.25 * cum
             else "SATURATING: indexing is NOT extending the reachable set"))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
