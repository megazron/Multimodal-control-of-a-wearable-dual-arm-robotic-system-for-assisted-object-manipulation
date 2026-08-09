#!/usr/bin/env python3
"""Live capability selection: run the best mapping the healthy channels allow.

    ros2 run srl_teleop capability_node
    ros2 topic echo /master_capability_left

WHAT THIS ADDS OVER channel_manager. channel_manager answers "which channels
are healthy". That is necessary and not sufficient: it cannot say which
mapping should therefore be running, what that mapping costs, or which repair
would actually buy anything. Fixing j3 while both bend joints are dead moves
the operator up no rung at all, and a health table cannot say so because the
answer depends on the OTHER channels.

Health is re-evaluated every cycle from a rolling window, so a channel that
comes back is picked up automatically without a restart. The rung can move in
either direction and every transition is logged with the channel that caused
it.
"""
import collections
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from srl_teleop import capability as cap

CH = ("j1", "j2", "j3", "j4", "j5", "j6", "j7")


class ChannelHealth:
    """Rolling per-channel verdict, over DISTINCT updates.

    Statistics must be computed between distinct, time-adjacent sensor
    updates. At the frame rate 82-88% of consecutive rows repeat, so anything
    computed across rows is dominated by duplicates and every channel returns
    a median absolute difference of exactly zero regardless of its health.
    """

    def __init__(self, window=120, jump_deg=60.0, jump_frac=0.05,
                 drop_frac=0.02, min_range_deg=20.0):
        self.window = window
        self.jump_deg = jump_deg
        self.jump_frac = jump_frac
        self.drop_frac = drop_frac
        self.min_range = min_range_deg
        self.vals = {c: collections.deque(maxlen=window) for c in CH}
        self.last = {c: None for c in CH}
        self.drops = {c: collections.deque(maxlen=window) for c in CH}

    def push(self, ch, deg):
        bad = (deg is None) or (deg != deg) or deg == 0.0 or not (0.0 <= deg <= 360.0)
        self.drops[ch].append(1 if bad else 0)
        if bad:
            return
        if self.last[ch] is None or abs(deg - self.last[ch]) > 1e-9:
            self.vals[ch].append(deg)          # DISTINCT updates only
            self.last[ch] = deg

    @staticmethod
    def _circular_range(v):
        """Smallest arc containing every sample.

        Unwrap-and-subtract is invalid on a rotary sensor: across a dropout
        the unwrap cannot know how far the joint moved and the offset runs
        away, which once produced 314363 deg on a 360 deg sensor.
        """
        if len(v) < 2:
            return 0.0
        s = sorted(x % 360.0 for x in v)
        gaps = [(s[i + 1] - s[i]) for i in range(len(s) - 1)]
        gaps.append(360.0 - s[-1] + s[0])
        return 360.0 - max(gaps)

    def verdict(self, ch):
        v = list(self.vals[ch])
        d = list(self.drops[ch])
        if len(d) < 20:
            return "unknown"
        drop = sum(d) / float(len(d))
        rng = self._circular_range(v)
        if rng < 5.0 or drop > 0.90:
            return "dead"
        if len(v) >= 10:
            jumps = sum(1 for i in range(1, len(v))
                        if min(abs(v[i] - v[i - 1]),
                               360 - abs(v[i] - v[i - 1])) > self.jump_deg)
            if jumps / float(len(v) - 1) > self.jump_frac:
                return "incoherent"
        if drop > self.drop_frac:
            return "intermittent"
        if rng < self.min_range:
            return "restricted"
        return "alive"

    def healthy(self):
        """Channels usable for a mapping.

        `restricted` counts as healthy: a channel with a small but coherent
        range still carries information, and the reduced-DOF fallback is built
        on exactly such a channel. `intermittent` also counts, because a
        dropout holds the last good value rather than injecting a wrong one.
        `unknown` does NOT count -- not yet knowing is not the same as healthy.
        """
        return {c for c in CH
                if self.verdict(c) in ("alive", "restricted", "intermittent")}


class CapabilityNode(Node):

    def __init__(self):
        super().__init__("capability_node")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("min_dwell_s", 1.0)
        self.health = {a: ChannelHealth() for a in ("left", "right")}
        self.imu_ok = {a: True for a in ("left", "right")}
        self.level = {a: None for a in ("left", "right")}
        self.since = {a: 0.0 for a in ("left", "right")}
        self.pub = {a: self.create_publisher(String,
                                             "/master_capability_%s" % a, 10)
                    for a in ("left", "right")}
        for a in ("left", "right"):
            self.create_subscription(
                Float32MultiArray, "/master_arm_raw_%s" % a,
                (lambda m, arm=a: self.on_raw(arm, m)), 20)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self.tick)
        if cap.COST_M:
            self.get_logger().info(
                "capability ladder loaded with measured costs: %s"
                % ", ".join("%s %.3f m" % (k, v["mean_m"])
                            for k, v in sorted(cap.COST_M.items())))
        else:
            self.get_logger().warn(
                "no measured rung costs found -- capability will report "
                "'unmeasured' rather than invent numbers. Run "
                "scripts/measure_capability_ladder.py")

    def on_raw(self, arm, msg):
        d = list(msg.data)
        for i, c in enumerate(CH):
            self.health[arm].push(c, d[i] if i < len(d) else None)
        if len(d) >= 10:
            ax, ay, az = d[7], d[8], d[9]
            g = math.sqrt(ax * ax + ay * ay + az * az)
            # The gate is on MAGNITUDE, not presence: a sensor reading a
            # plausible-looking constant that is not 1 g is not measuring
            # gravity, and elevation built on it is wrong rather than noisy.
            self.imu_ok[arm] = 0.3 < g < 3.0 if g > 0 else False

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dwell = float(self.get_parameter("min_dwell_s").value)
        for arm in ("left", "right"):
            h = self.health[arm]
            healthy = h.healthy()
            lvl, why = cap.select(healthy, self.imu_ok[arm])
            if self.level[arm] is None:
                self.level[arm], self.since[arm] = lvl, now
                self.get_logger().info("[%s] capability %s -- %s"
                                       % (arm, cap.describe(lvl), why))
            elif lvl != self.level[arm] and now - self.since[arm] >= dwell:
                # Dwell stops a channel flickering on the health boundary from
                # switching the mapping every cycle, which would be worse for
                # the operator than either rung.
                direction = "RECOVERED to" if lvl < self.level[arm] else "fell to"
                self.get_logger().warn(
                    "[%s] capability %s %s -- %s"
                    % (arm, direction, cap.LEVELS[lvl]["key"], why))
                self.level[arm], self.since[arm] = lvl, now

            cur = self.level[arm]
            regain = {c: dict(level=cap.LEVELS[l]["key"], improves=bool(imp))
                      for c, (l, imp) in cap.regain(healthy,
                                                    self.imu_ok[arm]).items()}
            cost = cap.COST_M.get(cap.LEVELS[cur]["key"])
            m = String()
            m.data = json.dumps(dict(
                arm=arm,
                level=cur, key=cap.LEVELS[cur]["key"],
                name=cap.LEVELS[cur]["name"],
                position_available=cap.position_available(cur),
                reason=why,
                healthy=sorted(healthy),
                verdicts={c: h.verdict(c) for c in CH},
                imu_ok=self.imu_ok[arm],
                cost_m=cost,
                cost_note=(None if cost else "unmeasured"),
                regain=regain,
                worth_fixing=sorted(c for c, r in regain.items()
                                    if r["improves"])))
            self.pub[arm].publish(m)


def main(argv=None):
    rclpy.init(args=argv)
    n = CapabilityNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
