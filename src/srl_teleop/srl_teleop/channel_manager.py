#!/usr/bin/env python3
"""
channel_manager.py — every master-arm channel individually disableable at
runtime, with the capability lost by each stated explicitly.

WHY THIS EXISTS. The pots are being replaced and must stop being load-bearing.
Until they are, channels keep failing mid-session: measured on this rig, right
j4 drops out on 12.9% of frames in bursts, and right j3/j5/j7 are dead. The
old behaviour was to reject the whole frame, which freezes the command — for
half of one recorded sweep. That is worse than degrading, because the operator
cannot tell a frozen command from a stationary one.

POLICY: DEGRADE, NEVER FAIL. Disabling a channel is always allowed. What it
costs is published on /master_capability_<arm> and logged once, so an
experiment can record which capability was live in every trial — E4 depends
on exactly that.

    ros2 param set /channel_manager left_j4_enabled false
    ros2 topic echo /master_capability_left

`disable_all_pots` is the endpoint of the Part 3 argument: with it set, the
rig is IMU-only and position is UNAVAILABLE. The node says so rather than
inventing a position, because inventing one is the single most dangerous
failure mode on a robot bolted to a person.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

ARMS = ("left", "right")
CHANNELS = [f"j{i}" for i in range(1, 8)] + ["accel", "gyro", "imu2"]

# What each channel supplies, and what is lost when it goes. Written from the
# MEASURED behaviour of the shipped spherical pipeline, not from intent.
CAPABILITY = {
    "j1": ("azimuth (lateral direction)",
           "lateral direction becomes gyro-only and drifts; with the gyro also "
           "disabled, azimuth is FROZEN and lateral teleop is unusable"),
    "j2": ("shoulder bend, feeds reach magnitude",
           "reach magnitude degrades; measured tip error rises (see the "
           "reduction ladder in CLAUDE.md)"),
    "j3": ("upper-arm roll; contributes almost nothing to reach magnitude",
           "negligible - already passed through as 0 when dead"),
    "j4": ("elbow bend, the dominant term in reach magnitude",
           "reach magnitude degrades badly unless a second IMU supplies the "
           "elbow angle instead"),
    "j5": ("forearm roll, unused in spherical mode",
           "nothing in spherical mode; wrist roll in fk mode"),
    "j6": ("wrist bend, unused in spherical mode",
           "nothing in spherical mode"),
    "j7": ("wrist roll, unused in spherical mode",
           "nothing in spherical mode; moves the tip 0.000 m regardless"),
    "accel": ("gravity vector -> elevation, roll, pitch. Absolute, drift-free",
              "ELEVATION IS LOST. Up/down teleop stops working and the "
              "complementary filter has nothing to correct toward, so the "
              "gyro runs open-loop"),
    "gyro": ("rotation rate, including yaw",
             "yaw-rate aiding of azimuth is lost (azimuth falls back to j1) "
             "and the complementary filter degrades to accel-only, which is "
             "noisier under fast motion"),
    "imu2": ("second IMU on the upper arm -> elbow angle without a pot",
             "elbow angle must come from the j4 pot again"),
}


class ChannelManager(Node):
    def __init__(self):
        super().__init__("channel_manager")
        for a in ARMS:
            for c in CHANNELS:
                self.declare_parameter(f"{a}_{c}_enabled", True)
        # One switch for the endpoint of the argument: IMUs only.
        self.declare_parameter("disable_all_pots", False)
        # Auto-disable a channel that has been reading a firmware-clamped 0.0
        # for this many consecutive frames. A stuck channel that keeps being
        # accepted is worse than one that is honestly dead.
        #
        # DEFAULT WAS 0 - i.e. OFF - and fault injection caught it: a channel
        # could drop out for the whole trial and nothing would ever mark it
        # dead. 25 frames is 0.5 s at 50 Hz, which is the measured length of a
        # real right-j4 dropout burst, so a genuine burst trips it and a
        # single bad frame does not.
        self.declare_parameter("auto_disable_after_frames", 25)

        self.pub = {a: self.create_publisher(String, f"/master_capability_{a}", 10)
                    for a in ARMS}
        self.zero_run = {a: {c: 0 for c in CHANNELS} for a in ARMS}
        # PART 2a: an auto-disabled channel that the CURRENT MODE depends on
        # must abort the trial, not merely be logged.
        self.abort_pub = self.create_publisher(String, "/participant/abort", 10)
        self.declare_parameter("mode_required_channels",
                               ["j1", "j2", "j4", "accel"])
        self.auto_off = {a: set() for a in ARMS}
        self._last = {}
        for a in ARMS:
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda m, a=a: self._on_raw(a, m), 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            "channel_manager up. Every channel is individually disableable; "
            "disabling degrades, it never stops the node.")

    # ---- runtime state ----
    def enabled(self, arm, ch):
        if ch in self.auto_off[arm]:
            return False
        if ch.startswith("j") and bool(self.get_parameter("disable_all_pots").value):
            return False
        return bool(self.get_parameter(f"{arm}_{ch}_enabled").value)

    def _on_raw(self, arm, msg):
        d = list(msg.data)
        if len(d) < 10:
            return
        n = int(self.get_parameter("auto_disable_after_frames").value)
        for i in range(7):
            ch = f"j{i+1}"
            if d[i] == 0.0:
                self.zero_run[arm][ch] += 1
            else:
                self.zero_run[arm][ch] = 0
                self.auto_off[arm].discard(ch)
            if n > 0 and self.zero_run[arm][ch] >= n and ch not in self.auto_off[arm]:
                self.auto_off[arm].add(ch)
                self.get_logger().warn(
                    "[%s] %s auto-disabled after %d consecutive zero frames. "
                    "LOST: %s" % (arm, ch, n, CAPABILITY[ch][1]))
                if ch in list(self.get_parameter("mode_required_channels").value):
                    rec = dict(t=time.strftime("%Y-%m-%dT%H:%M:%S"), kind="abort",
                               arm=arm, cause="required channel %s dropped out "
                               "mid-trial (%d consecutive zero frames)" % (ch, n),
                               lost=CAPABILITY[ch][1])
                    am = String(); am.data = json.dumps(rec)
                    self.abort_pub.publish(am)
                    self.get_logger().error(
                        "[%s] ABORT: %s is REQUIRED by the current mode. The "
                        "trial is INVALID and must not be silently continued."
                        % (arm, ch))
        self._last[arm] = d

    def _publish(self):
        for a in ARMS:
            state = {c: self.enabled(a, c) for c in CHANNELS}
            lost = [CAPABILITY[c][1] for c in CHANNELS if not state[c]]
            pos_ok = any(state[c] for c in ("j1", "j2", "j4"))
            payload = {
                "arm": a,
                "channels": state,
                "auto_disabled": sorted(self.auto_off[a]),
                "position_available": pos_ok,
                "elevation_available": state["accel"],
                "azimuth_available": state["j1"] or state["gyro"],
                "elbow_from_imu": state["imu2"],
                "capability_lost": lost,
            }
            m = String()
            m.data = json.dumps(payload)
            self.pub[a].publish(m)
            if not pos_ok:
                self.get_logger().warn(
                    "[%s] no position-bearing pot is enabled. POSITION IS "
                    "UNAVAILABLE and is reported as such rather than "
                    "invented. IMUs give orientation only." % a,
                    throttle_duration_sec=10.0)


def main():
    rclpy.init()
    n = ChannelManager()
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
