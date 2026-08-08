#!/usr/bin/env python3
"""
fsr_gripper_node.py — squeeze the master FSR, close the Robotiq 2F-85.

fsr1 drives the LEFT gripper, fsr2 the RIGHT. Both are published on
/master_fsr_buttons as [fsr1, fsr2, btn1, btn2].

MEASURED characteristics (2026-07-31):
  unsqueezed  6-191 counts
  squeezed    3603 (left) / 3842 (right)
  cross-talk  3.7% (left) / 0.4% (right), under load only
So the two channels are cleanly separable and a simple proportional map is
enough -- no mixing matrix needed.

Three things stop the raw signal being usable directly:
  * DEADBAND at 250 counts. Unsqueezed rests up to 191, so anything below
    250 must read as fully open or the gripper creeps shut on noise.
  * HYSTERESIS on the output. The FSR is noisy under light load, and a
    joint_trajectory message per frame at 50 Hz with a jittering target
    makes the fingers buzz.
  * PER-ARM RANGE, because the two pads differ by ~7% at full squeeze.

Run:
  ros2 run srl_teleop fsr_gripper_node
"""
import sys
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Robotiq 2F-85 driven knuckle: 0 rad open, ~0.8 rad closed.
CLOSED_RAD = 0.8

DEFAULTS = {
    "left":  dict(fsr_index=0, open_counts=191.0, closed_counts=3603.0),
    "right": dict(fsr_index=1, open_counts=191.0, closed_counts=3842.0),
}


class FSRGripper(Node):
    def __init__(self):
        super().__init__("fsr_gripper_node")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("deadband_counts", 250.0)
        self.declare_parameter("hysteresis_frac", 0.02)
        # ---- LATCH ----
        # Once the hand closes on an object it must STAY closed; hand fatigue
        # must not drop the load. Thresholds are in raw FSR counts, from the
        # measured ranges (unsqueezed 6-191, squeezed 3603 left / 3842 right):
        #   CLOSE at 1200 -- comfortably above the 191 rest ceiling and above
        #     the 250 deadband, but only ~33% of full squeeze, so a deliberate
        #     grasp latches without needing a hard squeeze.
        #   RELEASE at 400 -- just above the rest ceiling, so letting go fully
        #     releases while a tiring grip (which sags toward mid-range) does
        #     NOT. The wide 1200/400 gap is the anti-fatigue margin.
        self.declare_parameter("latch_close_counts", 1200.0)
        self.declare_parameter("latch_open_counts", 400.0)
        # Release only after the low reading is SUSTAINED, so a momentary dip
        # cannot drop the object.
        self.declare_parameter("latch_release_hold_s", 0.5)
        self.declare_parameter("closed_rad", CLOSED_RAD)
        self.declare_parameter("rate_hz", 20.0)
        for a, d in DEFAULTS.items():
            self.declare_parameter(f"{a}_open_counts", d["open_counts"])
            self.declare_parameter(f"{a}_closed_counts", d["closed_counts"])

        self.arms = list(self.get_parameter("arms").value)
        self.deadband = float(self.get_parameter("deadband_counts").value)
        self.hyst = float(self.get_parameter("hysteresis_frac").value)
        self.closed = float(self.get_parameter("closed_rad").value)
        self.latch_close = float(self.get_parameter("latch_close_counts").value)
        self.latch_open = float(self.get_parameter("latch_open_counts").value)
        self.latch_hold = float(self.get_parameter("latch_release_hold_s").value)
        self.latched = {a: False for a in self.arms}
        self.latch_frac = {a: 0.0 for a in self.arms}
        self.below_since = {a: None for a in self.arms}

        self.fsr = None
        self.last_cmd = {a: None for a in self.arms}
        self.pub = {
            a: self.create_publisher(
                JointTrajectory, f"/{a}_gripper_controller/joint_trajectory", 10)
            for a in self.arms
        }
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: setattr(self, "fsr", list(m.data)), 20)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                          self.tick)
        self.create_timer(5.0, self.log_state)

        for a in self.arms:
            self.get_logger().info(
                "[FSR:%s] fsr%d -> %s_robotiq_85_left_knuckle_joint, "
                "open %.0f counts, closed %.0f, deadband %.0f"
                % (a, DEFAULTS[a]["fsr_index"] + 1, a,
                   self.get_parameter(f"{a}_open_counts").value,
                   self.get_parameter(f"{a}_closed_counts").value,
                   self.deadband))

    def fraction(self, arm):
        """0.0 open .. 1.0 closed, or None if no data."""
        if not self.fsr or len(self.fsr) < 2:
            return None
        raw = float(self.fsr[DEFAULTS[arm]["fsr_index"]])
        if raw < self.deadband:
            return 0.0
        lo = float(self.get_parameter(f"{arm}_open_counts").value)
        hi = float(self.get_parameter(f"{arm}_closed_counts").value)
        if hi - lo < 1.0:
            return 0.0
        return max(0.0, min(1.0, (raw - lo) / (hi - lo)))

    def update_latch(self, arm, raw, frac, now):
        """Returns the commanded fraction after latch logic.

        Proportional on the way IN so the operator can close gently; once
        past the close threshold the grip is held at its deepest value and
        only a sustained, deliberate release opens it.
        """
        if not self.latched[arm]:
            if raw >= self.latch_close:
                self.latched[arm] = True
                self.latch_frac[arm] = frac
                self.below_since[arm] = None
                self.get_logger().info(
                    "[FSR:%s] LATCHED at raw=%.0f frac=%.2f -- will hold until "
                    "raw stays under %.0f for %.1f s"
                    % (arm, raw, frac, self.latch_open, self.latch_hold))
            return frac

        # Latched: keep the firmest grip seen, so squeezing harder tightens
        # but easing off does not slacken.
        self.latch_frac[arm] = max(self.latch_frac[arm], frac)
        if raw < self.latch_open:
            if self.below_since[arm] is None:
                self.below_since[arm] = now
            elif now - self.below_since[arm] >= self.latch_hold:
                self.latched[arm] = False
                self.latch_frac[arm] = 0.0
                self.below_since[arm] = None
                self.get_logger().info(
                    "[FSR:%s] RELEASED (raw under %.0f for %.1f s)"
                    % (arm, self.latch_open, self.latch_hold))
                return 0.0
        else:
            self.below_since[arm] = None
        return self.latch_frac[arm]

    def tick(self):
        now = time.monotonic()
        for a in self.arms:
            frac = self.fraction(a)
            if frac is None:
                continue
            raw = float(self.fsr[DEFAULTS[a]["fsr_index"]])
            frac = self.update_latch(a, raw, frac, now)
            # Hysteresis: only re-command on a meaningful change, so a noisy
            # pad does not stream a new setpoint every cycle.
            if (self.last_cmd[a] is not None
                    and abs(frac - self.last_cmd[a]) < self.hyst):
                continue
            self.last_cmd[a] = frac
            t = JointTrajectory()
            t.joint_names = [f"{a}_robotiq_85_left_knuckle_joint"]
            p = JointTrajectoryPoint()
            p.positions = [frac * self.closed]
            p.time_from_start = Duration(sec=0, nanosec=100_000_000)
            t.points = [p]
            self.pub[a].publish(t)

    def log_state(self):
        if not self.fsr:
            self.get_logger().warn("[FSR] no /master_fsr_buttons yet",
                                   throttle_duration_sec=10.0)
            return
        parts = []
        for a in self.arms:
            f = self.fraction(a)
            parts.append("%s raw=%.0f frac=%.2f %s" % (
                a, self.fsr[DEFAULTS[a]["fsr_index"]], f if f is not None else -1,
                "LATCHED" if self.latched[a] else "open"))
        self.get_logger().info("[FSR] " + "  ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    n = FSRGripper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
