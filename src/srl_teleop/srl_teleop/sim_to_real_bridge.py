#!/usr/bin/env python3
"""
sim_to_real_bridge.py — replay SIM joint angles onto the REAL arm, delayed.

THE CASCADE. The operator drives the SIM. The sim is the thing that can be
watched, collision-checked and undone. The real arm follows the sim's
recorded history, preview_delay_s behind, so there is always that much
warning: whatever the real arm is about to do has already been visible in
the sim for a full second.

WHY THE HOMES MUST MATCH. This node replays sim joint ANGLES directly. If the
sim home and the real home were different poses, the very first command after
enabling would be the difference between them -- a jump. That is why
config/home_positions_*.txt now hold the legacy real-robot values, and why
the real arm is homed to that same pose before this node is enabled.

FOUR INDEPENDENT LIMITS, all active at once:
  * preview_delay_s   the real arm plays sim history, never sim present
  * max_vel_rad_s     per-joint rate cap on what is commanded
  * max_step_rad      per-cycle displacement cap (an acceleration ceiling)
  * lag monitor       |sim_delayed - real_actual| over lag_trip_rad trips the
                      e-stop; that means the real arm is NOT keeping up, and
                      continuing to feed it commands it cannot execute is how
                      a slow fault becomes a fast one

The SIM is rate-limited to the same cap via /cascade_active, which the IK
followers listen to. Without that the sim would outrun the real arm and the
delay would grow instead of staying constant.

Collision checking uses the same wearer model as teleop, against the REAL
arm's prefixed TF frames (real_<arm>_<link>).

  ros2 run srl_teleop sim_to_real_bridge --ros-args -p arm:=left
  ros2 service call /bridge_enable  std_srvs/srv/Trigger {}
  ros2 service call /bridge_disable std_srvs/srv/Trigger {}
"""
import math
import os
import sys
import threading
import time
from collections import deque

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.expanduser("~/kortex_ws/config"))
import home_positions                                        # noqa: E402

from srl_teleop.kortex_convention import (                   # noqa: E402
    pose_delta_rad, wrap_rad_pi)
from srl_teleop.clearance import (                           # noqa: E402
    ClearanceModel, DISTAL_LINKS, REAL_ROBOT_PAD_M)

# ONE SOURCE, since 2026-08-23. This tuple -- which joints are
# type="continuous" and can therefore wind up -- had SIX identical
# definitions across this package. They agreed, which is luck rather
# than design: it is the same shape as two home poses, and the day one
# of them is edited the others describe a different robot.
from srl_teleop.motion_generator import CONTINUOUS_IDX     # noqa: E402,F401


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class SimToRealBridge(Node):
    def __init__(self):
        super().__init__("sim_to_real_bridge")
        self.declare_parameter("arm", "left")
        self.declare_parameter("real_ns", "/real")
        self.declare_parameter("preview_delay_s", 1.0)
        self.declare_parameter("max_vel_rad_s", 0.15)
        self.declare_parameter("max_step_rad", 0.05)
        # 0.15 rad had NO WORKING MARGIN: a mock run tripped at 0.155 rad
        # during ordinary motion, a 3% overshoot, with nothing wrong. A
        # protection that fires on normal operation gets disabled by the
        # operator, which is strictly worse than a looser one that fires only
        # on a real divergence. 0.5 rad = 28.6 deg is still far short of any
        # pose the arm could reach before the clearance floor catches it.
        self.declare_parameter("lag_trip_rad", 0.5)
        # ENABLE GATE, separate from the TRIP threshold on purpose. Enabling
        # at exactly the trip limit means the very first monitor tick after
        # the grace period trips. Enabling requires the gap to be inside
        # `enable_gap_rad`, leaving (lag_trip - enable_gap) of headroom for
        # the transient the enable itself causes.
        self.declare_parameter("enable_gap_rad", 0.3)
        self.declare_parameter("min_clearance_m", 0.12)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("enabled", False)
        # Guard: never enable unless the real arm is actually at the home
        # pose, because the first replayed sample is the sim AT home.
        self.declare_parameter("require_homed", True)
        self.declare_parameter("home_tolerance_rad", 0.05)
        # The lag monitor needs the real arm to have been fed for a moment
        # before its error means anything.
        self.declare_parameter("lag_grace_s", 3.0)
        self.declare_parameter("auto_enable_timeout_s", 420.0)

        self.arm = self.get_parameter("arm").value
        self.ns = self.get_parameter("real_ns").value.rstrip("/")
        self.delay = float(self.get_parameter("preview_delay_s").value)
        self.max_vel = float(self.get_parameter("max_vel_rad_s").value)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        self.lag_trip = float(self.get_parameter("lag_trip_rad").value)
        self.enable_gap = float(self.get_parameter("enable_gap_rad").value)
        self.min_clear = float(self.get_parameter("min_clearance_m").value)
        self.rate = float(self.get_parameter("rate_hz").value)
        self.require_homed = bool(self.get_parameter("require_homed").value)
        self.tol = float(self.get_parameter("home_tolerance_rad").value)
        self.grace = float(self.get_parameter("lag_grace_s").value)

        self.names = [f"{self.arm}_joint_{i}" for i in range(1, 8)]
        self.home = list(home_positions.load_home_radians(self.arm))

        self.sim_hist = deque(maxlen=4000)     # (t, [q7]) sim history
        # THE DEQUE IS BOUNDED, SO AN APPEND EVICTS, AND AN EVICTION DURING
        # A SCAN RAISES. `on_sim` appends on the ROS executor thread while
        # `sim_delayed` is scanned from the auto-enable thread; measured on
        # the rig 2026-08-29 as `RuntimeError: deque mutated during
        # iteration`, which killed the ONLY thread that will ever enable
        # this relay. The refusals it had been printing were asking the
        # operator to bring the arm to home, and after they did, nothing was
        # left alive to notice. The relay looked healthy and was permanently
        # dead. Reader takes a snapshot; writer honours the same lock, since
        # a snapshot under a lock only helps if the writer takes it too.
        self._hist_lock = threading.Lock()
        self.real_js = {}
        self.last_cmd = None
        self.enabled = False
        self.enabled_at = 0.0
        self.estopped = False
        self.trip_reason = None
        self.max_lag_seen = 0.0
        self.min_clear_seen = float("inf")

        self.create_subscription(JointState, "/joint_states", self.on_sim, 50)
        self.create_subscription(JointState, f"{self.ns}/joint_states",
                                 self.on_real, 50)
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: setattr(self, "estopped", m.data), 10)

        self.pub = self.create_publisher(
            JointTrajectory,
            f"{self.ns}/{self.arm}_arm_controller/joint_trajectory", 10)
        # PER ARM. This was a single global /cascade_active, and with
        # arm:=both there are TWO bridges with independent enable states
        # publishing to it. Each heartbeat overwrote the other, so BOTH
        # followers chattered between the cascade cap and the sim cap every
        # 0.5 s -- measured. The sim then ran at 0.6 rad/s half the time
        # against a real arm at 0.05, divergence was guaranteed, and the lag
        # monitor tripped on the design rather than on a fault.
        self.cascade_pub = self.create_publisher(
            Bool, "/cascade_active_%s" % self.arm, 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self._rebase = self.create_client(Trigger, "/master_rebase")
        # See real_homing_node: the enable gate opens the moment the real arm
        # is within home tolerance, which is EARLIER than homing finishes.
        # Enabling then puts two publishers on the real arm's trajectory
        # topic at once.
        self.homing_active = False
        self.create_subscription(
            Bool, "/homing_active_%s" % self.arm,
            lambda m: setattr(self, "homing_active", bool(m.data)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(
            Float64MultiArray, f"/bridge_status_{self.arm}", 10)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.clearance = ClearanceModel()

        # PER ARM. One bridge node runs per arm, so an unprefixed name is two
        # registrations of one service -- and "disable the bridge" reaching
        # only one arm, non-deterministically, leaves the other still driving
        # a real robot. Sixth instance of this class in this project.
        self.create_service(Trigger, "/bridge_enable_%s" % self.arm,
                            self.srv_enable)
        self.create_service(Trigger, "/bridge_disable_%s" % self.arm,
                            self.srv_disable)

        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(0.5, self.heartbeat)

        if bool(self.get_parameter("enabled").value):
            threading.Thread(target=self._auto_enable, daemon=True).start()

        self.get_logger().info(
            "bridge ready: delay %.2f s, vmax %.3f rad/s, step %.3f rad, "
            "lag trip %.2f rad. DISABLED until /bridge_enable."
            % (self.delay, self.max_vel, self.max_step, self.lag_trip))

    def _auto_enable(self):
        """Gated-launch path: keep retrying until the arm reaches home.

        enable() refuses while the real arm is outside home_tolerance, which
        is exactly the window in which homing is still running. So a single
        attempt would always lose the race; poll instead, and give up loudly
        rather than enabling into a pose that would jump.
        """
        deadline = time.monotonic() + float(self.get_parameter("auto_enable_timeout_s").value)
        last = ""
        nagged = 0.0
        while time.monotonic() < deadline:
            # THE ATTEMPT IS WRAPPED, AND THE LOOP CONTINUES. Catching and
            # then falling out of the loop would be the same defect with a
            # log line in front of it: this loop is the only thing that will
            # ever enable the relay, so an exception here must cost one
            # attempt, not the session.
            try:
                ok, msg = self._enable_attempt()
            except Exception as exc:                         # noqa: BLE001
                self.get_logger().error(
                    "auto-enable attempt raised (%s: %s) -- the relay is "
                    "still DISABLED and this loop is still retrying."
                    % (type(exc).__name__, exc))
                time.sleep(1.0)
                continue
            # ok is None when the inputs are not ready yet. That is a
            # PRECONDITION, not a refusal: printing it as one, or storing it
            # as `last`, would report "no sim /joint_states" as the reason a
            # relay gave up ten minutes later.
            if ok is not None:
                if ok:
                    self.get_logger().info(msg)
                    print("\n" + "=" * 62)
                    print("  REAL ARMS LIVE - move the master arm.")
                    print("=" * 62 + "\n", flush=True)
                    return
                # Say WHY it is not enabling, every few seconds. A silent
                # refusal is indistinguishable from a hung node, and the
                # operator is the one who has to fix it (by bringing the
                # master back towards home).
                if time.monotonic() - nagged > 5.0:
                    nagged = time.monotonic()
                    self.get_logger().warn(
                        "waiting to enable, still DISABLED: %s" % msg)
                last = msg
            time.sleep(1.0)
        self.get_logger().error(
            "bridge auto-enable GAVE UP after %.0f s; the relay is still "
            "DISABLED. Last refusal: %s"
            % (float(self.get_parameter("auto_enable_timeout_s").value),
               last or "the inputs never became ready"))

    def _enable_attempt(self):
        """One try, with the preconditions separated from the refusals.

        Returns (None, None) when the inputs are not ready yet -- no real
        joint states, or not enough sim history to have anything to replay.
        That is not the same thing as a refusal and the caller must not
        report it as one. Otherwise returns whatever `enable()` returned.
        """
        if self.real_current() is None:
            return (None, None)
        with self._hist_lock:
            have = len(self.sim_hist)
        if have <= 10:
            return (None, None)
        return self.enable()

    # ---------------- inputs ----------------

    def on_sim(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.names):
            with self._hist_lock:
                self.sim_hist.append(
                    (time.monotonic(), [d[n] for n in self.names]))

    def on_real(self, m):
        for n, p in zip(m.name, m.position):
            self.real_js[n] = p

    def real_current(self):
        if not all(n in self.real_js for n in self.names):
            return None
        return [self.real_js[n] for n in self.names]

    def sim_delayed(self):
        """The sim sample from preview_delay_s ago."""
        # SNAPSHOT, taken under the lock, then iterated outside it. Copying
        # a few thousand tuples is far cheaper than holding a lock across a
        # scan that a 50 Hz subscriber is waiting on.
        with self._hist_lock:
            hist = list(self.sim_hist)
        if not hist:
            return None
        want = time.monotonic() - self.delay
        best = None
        for t, q in hist:
            if t <= want:
                best = q
            else:
                break
        # Before enough history has accumulated, hold at the oldest sample
        # rather than jumping to the present -- the present is exactly what
        # the delay exists to avoid commanding.
        return best if best is not None else hist[0][1]

    # ---------------- enable / disable ----------------

    def srv_enable(self, req, resp):
        ok, msg = self.enable()
        resp.success = ok; resp.message = msg
        return resp

    def srv_disable(self, req, resp):
        self.disable("service request")
        resp.success = True; resp.message = "bridge disabled"
        return resp

    def enable(self):
        real = self.real_current()
        if real is None:
            return False, "no %s/joint_states -- real stack not up" % self.ns
        if not self.sim_hist:
            return False, "no sim /joint_states"
        if self.estopped:
            return False, "e-stop is latched -- reset it first"
        if self.homing_active:
            return False, ("REFUSED: %s homing is still running. Enabling now "
                           "would put the bridge and the homing law on the "
                           "same trajectory topic at once." % self.arm)
        if self.require_homed:
            d = pose_delta_rad(self.home, real, CONTINUOUS_IDX)
            worst = max(abs(x) for x in d)
            if worst > self.tol:
                # SAY WHY, NOT JUST HOW FAR. On 2026-08-15 the SIM home was
                # changed to the presentation pose and the real arms were not
                # recaptured, so this refusal is the expected state of the rig
                # rather than an operator error -- and a bare number reads
                # like a fault to whoever meets it in the lab. The recapture
                # is the fix; the runbook is named so nobody has to re-derive
                # anything from a joint index.
                return False, (
                    "REFUSED: real %s arm is %.3f rad (%.2f deg) from the "
                    "loaded home on joint_%d. The bridge replays sim angles "
                    "starting at home, so enabling now would command that "
                    "difference as a jump.\n"
                    "  LIKELY CAUSE: the SIM home was changed (2026-08-15, to "
                    "the presentation pose) and the REAL arms have not been "
                    "recaptured yet. A gap of about 1.9 rad on joint_7 is "
                    "exactly that case, not a mis-homed arm.\n"
                    "  FIX: follow 'CAPTURE THE NEW HOME ON THE REAL ARMS' in "
                    "docs/NEXT_SESSION.md -- it carries the target in Kortex "
                    "degrees and radians. Then home the arm and retry.\n"
                    "  The pose this bridge is comparing against is "
                    "config/home_positions_%s.txt."
                    % (self.arm, worst, math.degrees(worst),
                       max(range(7), key=lambda i: abs(d[i])) + 1, self.arm))
        # AND the sim must already agree with the real arm. Checking the real
        # arm against home is not sufficient: the operator is driving the sim
        # the whole time the real arm is homing, so the sim can be a long way
        # from home by the time homing finishes. Enabling then would command
        # the real arm to chase that gap at 0.05 rad/s and the lag monitor
        # would trip within its grace period -- an e-stop caused purely by
        # enabling, which teaches the operator to distrust the monitor.
        tgt = self.sim_delayed()
        if tgt is not None:
            gap = pose_delta_rad(tgt, real, CONTINUOUS_IDX)
            worst_gap = max(abs(x) for x in gap)
            if worst_gap > self.enable_gap:
                return False, (
                    "REFUSED: sim is %.3f rad (%.2f deg) from the real arm on "
                    "joint_%d, over the %.2f rad ENABLE gap (the trip limit "
                    "is %.2f rad; enabling needs %.2f rad of headroom for the "
                    "transient). Bring the master back towards home first."
                    % (worst_gap, math.degrees(worst_gap),
                       max(range(7), key=lambda i: abs(gap[i])) + 1,
                       self.enable_gap, self.lag_trip,
                       self.lag_trip - self.enable_gap))

        # Seed from the real arm's ACTUAL position so the first command is a
        # no-op rather than a step to wherever the sim happens to be.
        self.last_cmd = list(real)
        self.enabled = True
        self.enabled_at = time.monotonic()
        self.trip_reason = None
        self.max_lag_seen = 0.0
        self.min_clear_seen = float("inf")
        self.cascade_pub.publish(Bool(data=True))
        # REBASE THE MASTER REFERENCE AT ENABLE. The bridge seeds from the
        # real arm's ACTUAL position, so the master's reference must be
        # re-latched at the same instant or the operator's current pose maps
        # to wherever it mapped at node startup -- and they will have moved
        # in between. Non-blocking: a missing service must not stop the
        # bridge enabling, and on a sim-only stack it legitimately may not
        # exist. (Do not use wait_for_service here; that is the 4 s e-stop
        # stall all over again.)
        if self._rebase.service_is_ready():
            self._rebase.call_async(Trigger.Request())
            self.get_logger().info(
                "master reference rebased -- the master's CURRENT pose now "
                "maps to the arm's home")
        else:
            self.get_logger().warn(
                "/master_rebase not available; the master reference is "
                "whatever was latched at master_pose_node startup")
        return True, ("bridge ENABLED -- real arm follows sim %.2f s behind, "
                      "seeded from actual position" % self.delay)

    def disable(self, why):
        if self.enabled:
            self.get_logger().warn("bridge DISABLED: %s" % why)
        self.enabled = False
        self.trip_reason = why
        self.cascade_pub.publish(Bool(data=False))

    def trip_estop(self, why):
        self.get_logger().error("LAG MONITOR TRIPPING E-STOP: %s" % why)
        self.estop_pub.publish(Bool(data=True))
        self.disable(why)

    # ---------------- the loop ----------------

    def measure_clearance(self):
        pts = {}
        for part in ("torso", "head", "hips"):
            got = []
            for link in DISTAL_LINKS:
                try:
                    t = self.tf_buf.lookup_transform(
                        part, f"real_{self.arm}_{link}",
                        rclpy.time.Time()).transform.translation
                    got.append((t.x, t.y, t.z))
                except Exception:
                    continue
            if got:
                pts[part] = got
        if not pts:
            return float("inf"), None
        return self.clearance.clearance(pts, REAL_ROBOT_PAD_M)

    def tick(self):
        if not self.enabled:
            return
        if self.estopped:
            self.disable("e-stop latched")
            return

        target = self.sim_delayed()
        real = self.real_current()
        if target is None or real is None:
            return

        # --- lag monitor, against the REAL arm's actual position ---
        lag = pose_delta_rad(target, real, CONTINUOUS_IDX)
        worst_lag = max(abs(x) for x in lag)
        self.max_lag_seen = max(self.max_lag_seen, worst_lag)
        if time.monotonic() - self.enabled_at > self.grace and \
                worst_lag > self.lag_trip:
            self.trip_estop(
                "|sim_delayed - real| = %.3f rad on joint_%d, over the "
                "%.2f rad limit. The real arm is not keeping up."
                % (worst_lag, max(range(7), key=lambda i: abs(lag[i])) + 1,
                   self.lag_trip))
            return

        # --- collision, same model as teleop ---
        clear, part = self.measure_clearance()
        self.min_clear_seen = min(self.min_clear_seen, clear)
        if clear < self.min_clear:
            self.trip_estop("clearance %.3f m from %s under the %.2f m floor"
                            % (clear, part, self.min_clear))
            return

        # --- rate + step limited command, off the LAST COMMAND ---
        dt = 1.0 / self.rate
        cmd = []
        step_cap = min(self.max_vel * dt, self.max_step)
        for i in range(7):
            err = (wrap_rad_pi(target[i] - self.last_cmd[i])
                   if i in CONTINUOUS_IDX else target[i] - self.last_cmd[i])
            q = self.last_cmd[i] + clip(err, -step_cap, step_cap)
            cmd.append(wrap_rad_pi(q) if i in CONTINUOUS_IDX else q)
        self.last_cmd = cmd

        t = JointTrajectory()
        t.joint_names = self.names
        p = JointTrajectoryPoint()
        p.positions = cmd
        p.time_from_start = Duration(
            sec=int(dt), nanosec=int((dt - int(dt)) * 1e9))
        t.points = [p]
        self.pub.publish(t)

    def heartbeat(self):
        # Republish cascade state so a follower that starts late still learns
        # the sim is rate-limited.
        self.cascade_pub.publish(Bool(data=self.enabled))
        m = Float64MultiArray()
        real = self.real_current()
        tgt = self.sim_delayed()
        lag = 0.0
        if real is not None and tgt is not None:
            lag = max(abs(x) for x in
                      pose_delta_rad(tgt, real, CONTINUOUS_IDX))
        m.data = [1.0 if self.enabled else 0.0, self.delay, lag,
                  self.max_lag_seen,
                  self.min_clear_seen if self.min_clear_seen < 1e9 else -1.0,
                  float(len(self.sim_hist))]
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = SimToRealBridge()
    ex = SingleThreadedExecutor(); ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
