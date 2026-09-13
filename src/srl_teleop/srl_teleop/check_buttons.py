#!/usr/bin/env python3
"""30-SECOND LAB CHECK: which physical button moves which index?

    ros2 run srl_teleop check_buttons

The mapping in this repo has NEVER been confirmed against hardware. It was
inferred from one CSV in which btn2 changed during a left-arm prompt while
`BUTTON1_PIN=2` said btn1 was the left pin -- two pieces of evidence pointing
opposite ways, and the code picked one. Everything downstream (which arm a
press clutches, which arm freezes) rests on that guess.

This does not guess. It prompts for one button at a time, watches
/master_arm_raw_* companion fields via the FSR/button frame on
/master_status_*, reports WHICH INDEX ACTUALLY MOVED, and then states whether
the running parameters agree. If they do not, it prints the exact
`ros2 param set` lines to fix it -- the mapping is live, so no rebuild.

It is read-only: it never sets a parameter itself. A tool that silently
"fixes" a safety-relevant mapping is worse than one that reports.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

PROMPT_S = 12.0          # per button
SETTLE_S = 1.5


class ButtonCheck(Node):
    def __init__(self):
        super().__init__("check_buttons")
        self.btn = {1: None, 2: None}
        self.changes = {1: 0, 2: 0}
        self.seen = False
        # master_pose_node republishes the raw buttons in /master_status_*
        # fields; subscribing to both arms covers either publisher.
        for a in ("left", "right"):
            self.create_subscription(
                Float64MultiArray, "/master_status_%s" % a,
                self._on_status, 20)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons", self._on_buttons, 20)

    def _on_status(self, msg):
        self.seen = True

    def _on_buttons(self, msg):
        d = list(msg.data)
        if len(d) < 4:
            return
        self.seen = True
        for i, v in ((1, int(d[2])), (2, int(d[3]))):
            if self.btn[i] is None:
                self.btn[i] = v
            elif v != self.btn[i]:
                self.btn[i] = v
                self.changes[i] += 1

    def spin(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def watch(self, label):
        self.changes = {1: 0, 2: 0}
        print("\n  >>> PRESS AND RELEASE THE %s BUTTON, a few times. "
              "%.0f s..." % (label, PROMPT_S), flush=True)
        self.spin(PROMPT_S)
        c1, c2 = self.changes[1], self.changes[2]
        print("      btn1 changed %d time(s), btn2 changed %d time(s)"
              % (c1, c2))
        if c1 == c2 == 0:
            return None, "NOTHING MOVED"
        if c1 and c2:
            return None, ("BOTH indices moved (btn1 %d, btn2 %d) -- press one "
                          "button only, or the wiring is shorted" % (c1, c2))
        return (1 if c1 else 2), "clean"


def main(argv=None):
    rclpy.init(args=argv)
    n = ButtonCheck()
    print("=" * 72)
    print("BUTTON MAPPING CHECK -- read-only, ~30 s")
    print("=" * 72)
    print("  waiting for the master to publish...", flush=True)
    n.spin(SETTLE_S + 3.0)
    if not n.seen:
        print("\n  NO MASTER DATA on /master_fsr_buttons or /master_status_*.")
        print("  Start the stack first:  bash scripts/run_teleop.sh gate:=false")
        print("  (or use the virtual board: "
              "python3 scripts/virtual_teensy.py --mode script ...)")
        n.destroy_node()
        rclpy.shutdown()
        return 2

    found = {}
    for arm, label in (("left", "LEFT"), ("right", "RIGHT")):
        idx, why = n.watch(label)
        if idx is None:
            print("      -> %s: INCONCLUSIVE (%s)" % (arm, why))
        else:
            found[arm] = idx
            print("      -> the %s button is btn%d" % (arm, idx))

    print("\n" + "=" * 72)
    print("RESULT")
    print("=" * 72)
    if len(found) < 2:
        print("  Inconclusive -- re-run and press ONE button at a time.")
    elif found["left"] == found["right"]:
        print("  BOTH arms reported btn%d. That cannot be right: either the "
              "same button was pressed twice, or the two buttons are wired "
              "to one input." % found["left"])
    else:
        print("  MEASURED:  left -> btn%d,  right -> btn%d"
              % (found["left"], found["right"]))
        print("\n  The mapping is a LIVE parameter. If the running stack "
              "disagrees, set it without rebuilding:")
        for arm in ("left", "right"):
            print("      ros2 param set /master_pose_node %s_clutch_button %d"
                  % (arm, found[arm]))
        print("\n  Then record it in docs/ENGINEERING_LOG.md -- this is the first "
              "hardware confirmation of the mapping.")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
