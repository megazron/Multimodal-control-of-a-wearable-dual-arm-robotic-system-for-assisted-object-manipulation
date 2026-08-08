#!/usr/bin/env python3
"""
dashboard.py — one-terminal live status for the teleop stack, 5 Hz.

Plain ANSI, no GUI toolkit and no curses screen setup, so it works over SSH
and in WSL with no X server. When stdout is NOT a tty (e.g. captured by
`ros2 launch`), it degrades to periodic plain-text blocks instead of cursor
control, so the launch log stays readable rather than filling with escape
codes.

Colour policy for the raw pots is deliberately MODE-AWARE: a channel reading
exactly 0.0 is only red if the CURRENT position_mode actually consumes it.
In spherical mode j3/j5/j6/j7 are unused, so a dead j5 is informational, not
an error -- flagging it red would train the operator to ignore red.

  red    = exact 0.0 on a channel this mode uses  (a real dropout)
  yellow = railed (>=359.5), or exact 0.0 on a channel this mode ignores
  green  = healthy

Run:
  ros2 run srl_teleop dashboard
  ros2 launch srl_teleop teleop.launch.py dashboard:=true
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

from srl_teleop.master_pose_node import ZERO_CHECK_IDX, SPHERICAL_CHECK_IDX

R = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
GRN = "\033[32m"
YEL = "\033[33m"
CYN = "\033[36m"
DIM = "\033[2m"
HOME = "\033[H"
CLEAR = "\033[2J"

RAILED = 359.5
ACCEL_GATE = 0.15


def c(text, colour):
    return "%s%s%s" % (colour, text, R)


class Dashboard(Node):
    def __init__(self):
        super().__init__("teleop_dashboard")
        self.declare_parameter("arm", "both")
        self.declare_parameter("position_mode", "spherical")
        self.declare_parameter("rate_hz", 5.0)
        a = self.get_parameter("arm").value
        self.arms = ["left", "right"] if a == "both" else [a]
        self.mode = self.get_parameter("position_mode").value
        self.used = (SPHERICAL_CHECK_IDX if self.mode == "spherical"
                     else ZERO_CHECK_IDX)

        self.raw = {k: None for k in self.arms}
        self.cmd = {k: None for k in self.arms}
        self.mstat = {k: None for k in self.arms}
        self.istat = {k: None for k in self.arms}
        self.istat_prev = {k: None for k in self.arms}
        self.rstat = {k: None for k in self.arms}
        self.fsr = None

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

        for arm in self.arms:
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{arm}",
                lambda m, k=arm: self.raw.__setitem__(k, list(m.data)), 20)
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{arm}",
                lambda m, k=arm: self.cmd.__setitem__(k, m), 20)
            self.create_subscription(
                Float64MultiArray, f"/master_status_{arm}",
                lambda m, k=arm: self.mstat.__setitem__(k, list(m.data)), 20)
            self.create_subscription(
                Float64MultiArray, f"/ik_status_{arm}",
                lambda m, k=arm: self.istat.__setitem__(k, list(m.data)), 20)
            self.create_subscription(
                Float64MultiArray, f"/real_status_{arm}",
                lambda m, k=arm: self.rstat.__setitem__(k, list(m.data)), 20)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: setattr(self, "fsr", list(m.data)), 20)

        self.tty = sys.stdout.isatty()
        self.period = 1.0 / float(self.get_parameter("rate_hz").value)
        # 5 Hz is right for a redrawn terminal, but 5 blocks/second of plain
        # text would swamp a shared launch log, so throttle that path to 1 Hz.
        self.every = 1 if self.tty else max(1, int(round(
            float(self.get_parameter("rate_hz").value))))
        self.tick = 0
        self.t0 = time.monotonic()
        if self.tty:
            sys.stdout.write(CLEAR)
        self.create_timer(self.period, self.draw)

    # ---------------- rendering helpers ----------------

    def pots(self, arm):
        raw = self.raw[arm]
        if not raw or len(raw) < 7:
            return c("  (no data)", DIM)
        out = []
        for i in range(7):
            v = raw[i]
            if v == 0.0:
                col = RED if i in self.used else YEL
            elif v >= RAILED:
                col = YEL
            else:
                col = GRN
            out.append(c("j%d:%6.1f" % (i + 1, v), col))
        return "  ".join(out)

    def accel(self, arm):
        raw = self.raw[arm]
        if not raw or len(raw) < 10:
            return c("(no imu)", DIM)
        ax, ay, az = raw[7], raw[8], raw[9]
        mag = (ax * ax + ay * ay + az * az) ** 0.5
        ok = abs(mag - 1.0) < ACCEL_GATE
        return "%s  %s" % (
            c("|a|=%.3f g" % mag, GRN if ok else RED),
            "" if ok else c("OUTSIDE GATE - elevation held", RED))

    def ee(self, arm):
        try:
            t = self.buf.lookup_transform(
                "world", f"{arm}_end_effector_link",
                rclpy.time.Time()).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def arm_block(self, arm):
        L = []
        L.append(c("  %s ARM" % arm.upper(), BOLD + CYN))
        L.append("   pots  " + self.pots(arm))
        L.append("   imu   " + self.accel(arm))

        ms = self.mstat[arm]
        if ms and len(ms) >= 8:
            clutch = c("ENGAGED", GRN) if ms[0] else c("DISENGAGED", YEL)
            held = c(" elev HELD", YEL) if ms[7] else ""
            L.append("   state clutch=%s  scale=%.2f  elev=%+.1f deg  "
                     "azim=%+.1f deg  reach=%.3f m%s"
                     % (clutch, ms[1], ms[2], ms[3], ms[4], held))
            if ms[6]:
                L.append(c("         %d dropout(s) on used channels this frame"
                           % int(ms[6]), RED))
        else:
            L.append(c("   state (no /master_status_%s)" % arm, DIM))

        cmd = self.cmd[arm]
        ee = self.ee(arm)
        if cmd is not None:
            p = cmd.pose.position
            cs = "%+.3f %+.3f %+.3f" % (p.x, p.y, p.z)
        else:
            cs = "   --      --      --  "
        if ee is not None:
            es = "%+.3f %+.3f %+.3f" % ee
        else:
            es = "   --      --      --  "
        if cmd is not None and ee is not None:
            err = ((p.x - ee[0]) ** 2 + (p.y - ee[1]) ** 2 + (p.z - ee[2]) ** 2) ** 0.5
            ec = GRN if err < 0.02 else (YEL if err < 0.08 else RED)
            errs = c("%.3f m" % err, ec)
        else:
            errs = c("--", DIM)
        L.append("   cmd   %s      actual  %s      err %s" % (cs, es, errs))

        st = self.istat[arm]
        if st and len(st) >= 5:
            succ, fail, direct, slewed, rej = st[:5]
            tot = succ + fail
            rate = 100.0 * succ / tot if tot else 0.0
            rc = GRN if rate > 90 else (YEL if rate > 50 else RED)
            L.append("   ik    success %s   (%d ok / %d fail)"
                     % (c("%.0f%%" % rate, rc), int(succ), int(fail)))
            L.append("   guard published %d  (direct %d / %s)  %s"
                     % (int(direct + slewed), int(direct),
                        c("slewed %d" % int(slewed), YEL if slewed else GRN),
                        c("rejected %d" % int(rej), RED if rej else GRN)))
        else:
            L.append(c("   ik    (no /ik_status_%s - follower not running)" % arm, DIM))
        rs = self.rstat.get(arm)
        if rs and len(rs) >= 23:
            at = rs[22] >= 0.5
            L.append("   real  kortex %s" % " ".join("%.0f" % v for v in rs[7:14]))
            L.append("   home  max delta %s  %s"
                     % (c("%.3f rad" % rs[21], GRN if at else RED),
                        c("AT SIM HOME", GRN) if at
                        else c("NOT AT HOME - run real_homing_node", RED)))
        return L

    def draw(self):
        self.tick += 1
        if self.tick % self.every:
            return
        lines = []
        up = time.monotonic() - self.t0
        lines.append(c("SRL TELEOP DASHBOARD", BOLD)
                     + c("   mode=%s  uptime %.0fs  %s"
                         % (self.mode, up, time.strftime("%H:%M:%S")), DIM))
        lines.append(c("  validated channels: %s"
                       % ", ".join("j%d" % (i + 1) for i in self.used), DIM))
        lines.append("")
        for arm in self.arms:
            lines += self.arm_block(arm)
            lines.append("")
        if self.fsr and len(self.fsr) >= 4:
            lines.append("  fsr1=%.1f fsr2=%.1f   btn1=%d btn2=%d"
                         % (self.fsr[0], self.fsr[1],
                            int(self.fsr[2]), int(self.fsr[3])))
        else:
            lines.append(c("  (no /master_fsr_buttons)", DIM))

        if self.tty:
            # Pad so shrinking content does not leave stale text behind.
            body = "\n".join("%s\033[K" % ln for ln in lines)
            sys.stdout.write(HOME + body + "\033[J")
        else:
            sys.stdout.write("\n".join(lines) + "\n" + "-" * 70 + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    n = Dashboard()
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
    sys.exit(main())
