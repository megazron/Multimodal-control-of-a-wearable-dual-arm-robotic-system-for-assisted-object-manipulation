#!/usr/bin/env python3
"""
live_monitor.py — every raw Teensy channel, 5 Hz, in its own terminal.

READS ROS TOPICS, NEVER THE SERIAL PORT. master_pose_node owns /dev/ttyACM*;
a second reader would steal bytes from it and both would see corrupt frames.
So this subscribes to /master_arm_raw_<arm> and /master_fsr_buttons, which
carry the RAW values before validation or substitution -- exactly what you
want when the question is "is this channel alive".

COLOUR POLICY, deliberately MODE-AWARE (same rule as dashboard.py):
  red    = exact 0.0 on a channel THIS MODE ACTUALLY USES  -> a real dropout
  yellow = railed (>=359.5), or exact 0.0 on a channel this mode ignores
  green  = healthy
In spherical mode only j1/j2/j4 are consumed, so a dead j3 or j5 is
informational, not an error. Coluring it red would train the operator to
ignore red, which is worse than not colouring it at all.

  ros2 run srl_teleop live_monitor
  ros2 run srl_teleop live_monitor --ros-args -p position_mode:=fk
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from srl_teleop.master_pose_node import ZERO_CHECK_IDX, SPHERICAL_CHECK_IDX

R = "\033[0m"; BOLD = "\033[1m"; DIM = "\033[2m"
RED = "\033[31m"; GRN = "\033[32m"; YEL = "\033[33m"; CYN = "\033[36m"

RAILED = 359.5
ACCEL_GATE = 0.15          # |a| must be within this of 1 g to trust elevation


def c(t, col):
    return "%s%s%s" % (col, t, R)


class Mon(Node):
    def __init__(self):
        super().__init__("live_monitor")
        self.declare_parameter("position_mode", "spherical")
        self.declare_parameter("rate_hz", 5.0)
        self.mode = self.get_parameter("position_mode").value
        self.used = (SPHERICAL_CHECK_IDX if self.mode == "spherical"
                     else ZERO_CHECK_IDX)

        self.raw = {}
        self.last = {}
        self.fsr = None
        self.fsr_t = 0.0
        for a in ("left", "right"):
            self.create_subscription(
                Float64MultiArray, "/master_arm_raw_%s" % a,
                lambda m, arm=a: self._raw(arm, m), 20)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons", self._fsr, 20)
        self.t0 = time.monotonic()
        self.tty = sys.stdout.isatty()
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                          self.show)

    def _raw(self, arm, m):
        self.raw[arm] = list(m.data)
        self.last[arm] = time.time()

    def _fsr(self, m):
        self.fsr = list(m.data)
        self.fsr_t = time.monotonic()

    # ---------------- rendering ----------------

    def pots(self, d):
        out = []
        for i in range(7):
            v = d[i] if i < len(d) else float("nan")
            if v == 0.0:
                col = RED if i in self.used else YEL
            elif v >= RAILED:
                col = YEL
            else:
                col = GRN
            out.append(c("j%d %6.1f" % (i + 1, v), col))
        return "  ".join(out)

    def imu(self, d):
        """accel + gyro. Short payloads are reported, not crashed on."""
        if len(d) < 10:
            return c("   imu   (no accel in payload, len=%d)" % len(d), DIM)
        ax, ay, az = d[7], d[8], d[9]
        mag = (ax * ax + ay * ay + az * az) ** 0.5
        ok = abs(mag - 1.0) < ACCEL_GATE
        lines = ["   accel %7.3f %7.3f %7.3f   %s"
                 % (ax, ay, az,
                    c("|a|=%.3f g" % mag, GRN if ok else RED)
                    + ("" if ok else c("  OUTSIDE GATE - elevation HELD", RED)))]
        if len(d) >= 13:
            gx, gy, gz = d[10], d[11], d[12]
            spin = (gx * gx + gy * gy + gz * gz) ** 0.5
            lines.append("   gyro  %7.2f %7.2f %7.2f   %s"
                         % (gx, gy, gz,
                            c("|w|=%.2f deg/s" % spin,
                              DIM if spin < 5.0 else CYN)))
        else:
            lines.append(c("   gyro  (not in payload)", DIM))
        return "\n".join(lines)

    def show(self):
        L = []
        L.append(c("MASTER ARM LIVE", BOLD)
                 + c("   mode=%s   uptime %.0fs   %s   (Ctrl-C to quit)"
                     % (self.mode, time.monotonic() - self.t0,
                        time.strftime("%H:%M:%S")), DIM))
        L.append(c("  channels this mode consumes: %s"
                   % ", ".join("j%d" % (i + 1) for i in self.used), DIM))
        L.append("")
        for a in ("left", "right"):
            d = self.raw.get(a)
            L.append(c("  %s ARM" % a.upper(), BOLD + CYN))
            if not d:
                L.append(c("   (no /master_arm_raw_%s -- is master_pose_node "
                           "running?)" % a, DIM))
                L.append("")
                continue
            age = time.monotonic() - self.last.get(a, 0)
            if age > 1.0:
                L.append(c("   STALE: last frame %.1f s ago" % age, RED))
            L.append("   pots  " + self.pots(d))
            L.append(self.imu(d))
            L.append("")
        if self.fsr and len(self.fsr) >= 4:
            age = time.monotonic() - self.fsr_t
            f1, f2, b1, b2 = self.fsr[0], self.fsr[1], self.fsr[2], self.fsr[3]
            L.append("  fsr1 %7.1f   fsr2 %7.1f      btn1 %s   btn2 %s%s"
                     % (f1, f2,
                        c("DOWN", GRN) if b1 else c("up", DIM),
                        c("DOWN", GRN) if b2 else c("up", DIM),
                        c("   (stale %.1fs)" % age, RED) if age > 1.0 else ""))
        else:
            L.append(c("  (no /master_fsr_buttons)", DIM))
        L.append("")
        L.append(c("  %s dropout on a USED channel   %s railed / unused-zero  "
                   "%s healthy" % (c("red", RED), c("yellow", YEL),
                                   c("green", GRN)), DIM))

        if self.tty:
            sys.stdout.write("\033[H" + "\n".join("%s\033[K" % x for x in L)
                             + "\033[J")
        else:
            sys.stdout.write("\n".join(L) + "\n" + "-" * 70 + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    n = Mon()
    if n.tty:
        sys.stdout.write("\033[2J")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.tty:
            sys.stdout.write("\033[?25h\n")
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
