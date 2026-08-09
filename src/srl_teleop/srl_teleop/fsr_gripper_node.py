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
import json
import signal
import sys
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    from srl_teleop import gripper_state as gs
except ImportError:                                   # pragma: no cover
    import gripper_state as gs

# Robotiq 2F-85 driven knuckle: 0 rad open, ~0.8 rad closed.
CLOSED_RAD = gs.CLOSED_RAD

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

        # ---- STARTUP / SHUTDOWN OPEN ----
        # See gripper_state.py. The short version: the 2F-85 holds position
        # when its commanding process dies, so the fingers found at startup
        # are a LEFTOVER, not a reference. Neither end may assume.
        self.declare_parameter("open_on_startup", True)
        self.declare_parameter("open_on_shutdown", True)
        self.declare_parameter("startup_confirm_s", 3.0)
        self.declare_parameter("startup_open_tol_rad", gs.OPEN_RAD)

        self.fsr = None
        self._shutting_down = False
        self.knuckle = {a: None for a in self.arms}
        self.startup_done = {a: False for a in self.arms}
        self.startup_state = {a: "pending" for a in self.arms}
        self.hold_on_startup = {a: False for a in self.arms}
        self.startup_action = {a: None for a in self.arms}
        self.startup_t0 = time.monotonic()
        self.last_cmd = {a: None for a in self.arms}
        self.pub = {
            a: self.create_publisher(
                JointTrajectory, f"/{a}_gripper_controller/joint_trajectory", 10)
            for a in self.arms
        }
        # Published so the startup verdict is READABLE rather than only
        # logged. A per-arm decision that only exists in a log line cannot be
        # asserted on by the console, the preflight or a test.
        self.state_pub = self.create_publisher(String, "/gripper_reference", 10)
        # The deliberate way out of an inherited grip, for when squeezing the
        # pad is not available or not wanted (the operator has taken the
        # master off, or the object should be set down before anyone touches
        # anything). A service and not a topic: releasing a load is a request
        # that must be acknowledged, not a message that may be dropped.
        for a in self.arms:
            self.create_service(Trigger, "/gripper_release_%s" % a,
                                lambda rq, rs, a=a: self.srv_release(a, rq, rs))
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: setattr(self, "fsr", list(m.data)), 20)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                          self.tick)
        self.create_timer(5.0, self.log_state)
        self.create_timer(0.2, self.startup_tick)

        for a in self.arms:
            self.get_logger().info(
                "[FSR:%s] fsr%d -> %s_robotiq_85_left_knuckle_joint, "
                "open %.0f counts, closed %.0f, deadband %.0f"
                % (a, DEFAULTS[a]["fsr_index"] + 1, a,
                   self.get_parameter(f"{a}_open_counts").value,
                   self.get_parameter(f"{a}_closed_counts").value,
                   self.deadband))
            if gs.was_latched(a):
                age = gs.marker_age_s(a)
                self.get_logger().warn(
                    "[FSR:%s] LATCH MARKER PRESENT (%s, %.0f s old) -- a "
                    "previous run took a grip and did not let go. Deciding "
                    "from the knuckle before commanding anything."
                    % (a, gs.marker_path(a), age if age is not None else -1))

    # ------------------------------------------------------------ startup
    def on_joints(self, m):
        for a in self.arms:
            name = f"{a}_robotiq_85_left_knuckle_joint"
            if name in m.name:
                self.knuckle[a] = float(m.position[m.name.index(name)])

    def startup_tick(self):
        """Assert the open reference once, per arm, and CONFIRM it.

        Commanding an open and assuming it happened is the same mistake as
        assuming the leftover position was open -- one stage later. So the
        confirmation is read back from /joint_states, and a timeout is
        reported as a timeout rather than as success.
        """
        if self._shutting_down:
            return
        if not bool(self.get_parameter("open_on_startup").value):
            for a in self.arms:
                if not self.startup_done[a]:
                    self.startup_done[a] = True
                    self.startup_state[a] = "skipped"
            return
        waited = time.monotonic() - self.startup_t0
        tol = float(self.get_parameter("startup_open_tol_rad").value)
        deadline = float(self.get_parameter("startup_confirm_s").value)

        for a in self.arms:
            if self.startup_done[a]:
                continue

            # DECIDE ONCE, FROM A STATIONARY HAND, THEN COMMIT.
            # "Was this hand holding something when I started?" has exactly
            # one answer, and it must be read before anything is commanded.
            # Re-deciding every tick reads the hand WHILE IT IS MOVING under
            # our own open command: measured, a gripper correctly found at
            # 0.800 rad (free_air, stale marker, safe to open) was re-judged
            # 0.2 s later at 0.55 rad -- mid-ramp, inside the holding band --
            # and the node concluded it was gripping something. It then
            # latched onto its own open. The hand is genuinely static at
            # startup, because a dead process commands nothing, so the first
            # reading is the trustworthy one.
            if self.startup_action[a] is None:
                action, why = gs.startup_decision(a, self.knuckle[a])
                if action != "unknown":
                    self.startup_action[a] = (action, why)
            if self.startup_action[a] is not None:
                action, why = self.startup_action[a]
            else:
                action, why = gs.startup_decision(a, self.knuckle[a])

            if action == "unknown":
                # No feedback YET is not the same as no feedback. Keep waiting
                # until the deadline, then say so and refuse to command.
                if waited < deadline:
                    continue
                self.startup_done[a] = True
                self.startup_state[a] = "no_feedback"
                self.get_logger().error(
                    "[FSR:%s] STARTUP REFERENCE UNKNOWN after %.1f s -- %s "
                    "Commanding nothing: an open on no evidence could drop a "
                    "held object." % (a, waited, why))
                continue

            if action == "hold":
                self.startup_done[a] = True
                self.startup_state[a] = "holding"
                self.hold_on_startup[a] = True
                # Adopt the grip rather than fighting it, so the operator's
                # own release is what opens the hand.
                self.latched[a] = True
                self.latch_frac[a] = min(1.0, max(0.0, self.knuckle[a] / self.closed))
                self.last_cmd[a] = self.latch_frac[a]
                self.get_logger().warn(
                    "[FSR:%s] STARTUP: HOLDING, NOT OPENING -- %s" % (a, why))
                continue

            # action == "open"
            if self.last_cmd[a] != 0.0:
                self.command(a, 0.0)
                self.get_logger().info("[FSR:%s] STARTUP: %s" % (a, why))
            if self.knuckle[a] is not None and self.knuckle[a] < tol:
                self.startup_done[a] = True
                self.startup_state[a] = "open_confirmed"
                gs.clear_latched(a)
                self.get_logger().info(
                    "[FSR:%s] OPEN REFERENCE CONFIRMED from /joint_states at "
                    "%.4f rad (< %.3f). Travel range is 0.000 .. %.3f rad."
                    % (a, self.knuckle[a], tol, self.closed))
            elif waited >= deadline:
                self.startup_done[a] = True
                self.startup_state[a] = "open_UNCONFIRMED"
                self.get_logger().error(
                    "[FSR:%s] OPEN COMMANDED BUT NOT CONFIRMED after %.1f s "
                    "-- knuckle reads %s. The controller may not be active. "
                    "Do NOT trust the travel range until this is resolved."
                    % (a, waited,
                       "no data" if self.knuckle[a] is None
                       else "%.4f rad" % self.knuckle[a]))
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(dict(
            startup={a: self.startup_state[a] for a in self.arms},
            latched={a: bool(self.latched[a]) for a in self.arms},
            marker={a: bool(gs.was_latched(a)) for a in self.arms},
            knuckle={a: self.knuckle[a] for a in self.arms},
            closed_rad=self.closed))))

    def srv_release(self, arm, _req, resp):
        was = self.latched[arm]
        self.latched[arm] = False
        self.latch_frac[arm] = 0.0
        self.below_since[arm] = None
        self.hold_on_startup[arm] = False
        gs.clear_latched(arm)
        self.command(arm, 0.0)
        resp.success = True
        resp.message = ("%s released deliberately (was %s); commanded full open"
                        % (arm, "LATCHED" if was else "not latched"))
        self.get_logger().warn("[FSR:%s] %s" % (arm, resp.message))
        return resp

    def command(self, arm, frac):
        """Publish one knuckle setpoint. The ONLY place that publishes."""
        self.last_cmd[arm] = frac
        t = JointTrajectory()
        t.joint_names = [f"{arm}_robotiq_85_left_knuckle_joint"]
        p = JointTrajectoryPoint()
        p.positions = [frac * self.closed]
        p.time_from_start = Duration(sec=0, nanosec=100_000_000)
        t.points = [p]
        self.pub[arm].publish(t)

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
                # WRITTEN HERE, NOT AT SHUTDOWN. kill -9 runs no exit code, so
                # the evidence that this hand may be holding something must
                # already be on disk before the kill lands.
                gs.set_latched(arm, frac, "fsr_gripper_node")
                self.get_logger().info(
                    "[FSR:%s] LATCHED at raw=%.0f frac=%.2f -- will hold until "
                    "raw stays under %.0f for %.1f s"
                    % (arm, raw, frac, self.latch_open, self.latch_hold))
            return frac

        # Latched: keep the firmest grip seen, so squeezing harder tightens
        # but easing off does not slacken.
        self.latch_frac[arm] = max(self.latch_frac[arm], frac)

        # AN INHERITED GRIP CANNOT BE RELEASED BY A RESTING PAD.
        # After a crash recovery the operator's hand is, by definition, not on
        # the pad -- so the ordinary "raw stayed low for 0.5 s" release would
        # fire immediately and drop the object about half a second after the
        # node came back. That is the exact failure the hold exists to
        # prevent, arriving through the recovery path instead of the fatigue
        # path. So a grip this node did not itself take can only be let go by
        # someone who first demonstrates they have the pad (one squeeze past
        # the latch threshold), or by the explicit release service.
        if self.hold_on_startup[arm]:
            if raw >= self.latch_close:
                self.hold_on_startup[arm] = False
                self.get_logger().info(
                    "[FSR:%s] inherited grip HANDED BACK -- operator squeezed "
                    "past %.0f, normal release rules resume."
                    % (arm, self.latch_close))
            else:
                self.below_since[arm] = None
            return self.latch_frac[arm]

        if raw < self.latch_open:
            if self.below_since[arm] is None:
                self.below_since[arm] = now
            elif now - self.below_since[arm] >= self.latch_hold:
                self.latched[arm] = False
                self.latch_frac[arm] = 0.0
                self.below_since[arm] = None
                self.hold_on_startup[arm] = False
                gs.clear_latched(arm)
                self.get_logger().info(
                    "[FSR:%s] RELEASED (raw under %.0f for %.1f s)"
                    % (arm, self.latch_open, self.latch_hold))
                return 0.0
        else:
            self.below_since[arm] = None
        return self.latch_frac[arm]

    def tick(self):
        if self._shutting_down:
            return
        now = time.monotonic()
        for a in self.arms:
            # The startup open owns the gripper until it has resolved. Letting
            # the FSR map run first would command a position derived from a
            # reference that has not been established yet -- which is the bug
            # this whole pass exists to close.
            if not self.startup_done[a]:
                continue
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
            self.command(a, frac)

    # ----------------------------------------------------------- shutdown
    def open_on_exit(self):
        """Open the hand on the way out -- EXCEPT where that would drop a load.

        Called from the clean path AND from the exception path, because an
        exception is the commoner way for a node to end and leaving the
        fingers wherever a traceback found them is exactly the leftover state
        this pass removes.

        A LATCHED ARM IS LEFT CLOSED, DELIBERATELY. The latch exists so hand
        fatigue cannot drop a load; a node crash is not the operator letting
        go, and opening on it would turn every software fault into a dropped
        object. The marker stays on disk so the next start inherits the claim
        rather than guessing.
        """
        if not bool(self.get_parameter("open_on_shutdown").value):
            return
        # ONE OWNER FROM HERE ON. Measured: without this the drain loop below
        # spins the node, `tick()` fires once more, re-reads a pad the
        # operator is still touching and re-commands the old position -- the
        # setpoint stream after SIGTERM was [0.0, 0.166] and the hand closed
        # again after being told to open. Same shape as the scripted-grasp /
        # frac-schedule collision fixed in the 2026-08-08 grasp work: two
        # writers on one gripper in the same cycle, last one wins.
        self._shutting_down = True
        for a in self.arms:
            if self.latched[a]:
                self.get_logger().warn(
                    "[FSR:%s] SHUTDOWN: LATCHED -- leaving the grip closed so "
                    "the load is not dropped. Marker %s stays; the next start "
                    "will see it." % (a, gs.marker_path(a)))
                continue
            try:
                self.command(a, 0.0)
                self.get_logger().info(
                    "[FSR:%s] SHUTDOWN: commanded FULL OPEN." % a)
            except Exception as e:                    # noqa: BLE001
                self.get_logger().error(
                    "[FSR:%s] SHUTDOWN open FAILED: %s" % (a, e))
        # Give the message a moment to leave the process. A publish followed
        # immediately by destroy_node() is a publish that may never go out.
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def log_state(self):
        if not self.fsr:
            self.get_logger().warn("[FSR] no /master_fsr_buttons yet",
                                   throttle_duration_sec=10.0)
            return
        parts = []
        for a in self.arms:
            f = self.fraction(a)
            parts.append("%s raw=%.0f frac=%.2f knuckle=%s %s [%s]" % (
                a, self.fsr[DEFAULTS[a]["fsr_index"]], f if f is not None else -1,
                "--" if self.knuckle[a] is None else "%.3f" % self.knuckle[a],
                "LATCHED" if self.latched[a] else "open",
                self.startup_state[a]))
        self.get_logger().info("[FSR] " + "  ".join(parts))
        self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    n = FSRGripper()
    # SIGTERM as well as SIGINT. `ros2 launch` shutdown, a systemd stop and
    # the GUI's own process-group teardown all send SIGTERM, and Python's
    # default handler for it exits without unwinding -- so `finally:` alone
    # would not run and the hand would be left wherever it was.
    def _bail(signum, _frame):
        raise KeyboardInterrupt("signal %d" % signum)
    try:
        signal.signal(signal.SIGTERM, _bail)
    except ValueError:                                # not the main thread
        pass
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except Exception:                                 # noqa: BLE001
        # The exception path matters MORE than the clean one: a node that
        # dies on a traceback is the commoner way to leave the fingers half
        # shut. Open first, then re-raise so the fault is still visible.
        try:
            n.open_on_exit()
        finally:
            n.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        raise
    try:
        n.open_on_exit()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
