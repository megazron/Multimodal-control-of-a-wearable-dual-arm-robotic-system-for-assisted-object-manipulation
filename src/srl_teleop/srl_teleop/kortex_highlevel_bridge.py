#!/usr/bin/env python3
"""
kortex_highlevel_bridge.py — drive the REAL arm over the HIGH-LEVEL Kortex API.

WHY THIS EXISTS. ros2_control's Kortex hardware interface commands through the
CYCLIC path, which is designed for a 1 kHz loop on a dedicated link. Over WSL2
every cyclic write is a network round trip costing ~10 ms, measured:

    Read time: 217 us, Update: 282 us, Write time: 10365 us

At 100 Hz the budget is 10 ms, so the write alone consumes the entire cycle.
The controller manager overran permanently, no command ever landed, and the
arm sat still serving perfectly live feedback -- which looks exactly like a
frozen driver and is not one. Dropping the CM rate does not fix it: the write
is a round trip whatever rate you ask for.

This node replaces that final hop. SendJointSpeedsCommand is a HIGH-LEVEL TCP
call on the same session as the feedback read. It pays the same ~10 ms, but it
is a velocity command: it does not need to be re-sent at 1 kHz to hold a
motion, so 30 Hz is ample. Measured on this link the arm tracked a commanded
5 deg/s to within 6%.

ACHIEVED RATE. rate_hz is the target; this link delivers ~18-20 Hz, because a
cycle is TWO sequential round trips (GetMeasuredJointAngles, then
SendJointSpeedsCommand) and back-to-back RPCs cost ~25 ms each rather than the
~10 ms an isolated call costs. The loop free-runs when it cannot hit the
target, and logs what it actually achieved -- never assume the configured rate.
That is still far more than this motion needs: at vmax 0.05 rad/s a joint
moves 2.8 mrad per cycle at 18 Hz, against a 1 deg (17 mrad) deadband.

EVERYTHING UPSTREAM IS UNCHANGED. master -> sim -> sim_to_real_bridge is the
same architecture, and this node subscribes to the same JointTrajectory topic
the arm controller used to serve. The lag monitor and the collision check stay
where they are, in sim_to_real_bridge, and still see /real/joint_states --
which this node publishes. Only the metal-facing hop changed.

THE CONTROL LAW is real_homing_node's, deliberately:

    speed[j] = clip(kp * angle_diff(target[j], actual[j]), -vmax, +vmax)

The clip is on SPEED, not on error, so a large error cannot produce a fast
move -- it produces a long slow one. Inside the deadband the joint is
commanded to exactly zero rather than left to dither against encoder noise.

SAFETY. Three independent things command zero speed, and all three are
cheaper than the driver-level e-stop they replace:
  * the watchdog, if no target arrives for watchdog_s
  * /estop_state going true
  * shutdown, on every path including exceptions

That last point is the reason the old e-stop was one-way: deactivating a
Kortex component tore down its API router and on_activate could not rebuild
it. Zeroing a velocity does no such damage, so this e-stop is a SOFT stop and
the arm is recoverable without relaunching.

ONE SESSION. The arm permits exactly one, and a leaked session blocks the next
run until it times out. Read and write share this node's single session, and
shutdown closes it on every exit path.

  Run it with the kortex venv interpreter, which is the only one that has both
  rclpy and kortex_api:
    ~/kortex_ws/.kortex_venv/bin/python -m srl_teleop.kortex_highlevel_bridge \
        --ros-args -p arm:=left
"""
import json
import math
import signal
import sys
import threading
import time

# EVERY interval in this file uses time.monotonic(), never time.time().
# The WSL wall clock steps backwards when it resyncs with the host, which
# produced send latencies of -2321 ms in the first run of this node -- an
# interval measured across a step. The same stepping is what makes
# real.robot_state_publisher log "Moved backwards in time, re-publishing joint
# transforms" every ~30 s; that warning is a WSL clock artefact, unrelated to
# the cyclic-write fault this node replaces.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from srl_teleop.kortex_convention import (
    kortex_deg_to_ros_rad, pose_delta_rad)

try:
    import kortex_api.autogen.client_stubs.BaseClientRpc as BaseClient
    import kortex_api.autogen.messages.Base_pb2 as Base_pb2
    import kortex_api.autogen.messages.Session_pb2 as Session_pb2
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
except ImportError:
    sys.stderr.write(
        "kortex_api not importable. This node MUST run under the kortex venv:\n"
        "  ~/kortex_ws/.kortex_venv/bin/python -m "
        "srl_teleop.kortex_highlevel_bridge\n"
        "Do NOT pip install kortex_api into user site -- it pins protobuf\n"
        "3.5.1, which is broken on Python 3.12 and shadows the protobuf ROS\n"
        "needs.\n")
    raise

CONTINUOUS_IDX = (0, 2, 4, 6)
NJ = 7


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class KortexHighLevelBridge(Node):
    def __init__(self):
        super().__init__("kortex_highlevel_bridge")
        self.declare_parameter("arm", "left")
        self.declare_parameter("robot_ip", "192.168.1.10")
        self.declare_parameter("port", 10000)
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")
        self.declare_parameter("real_ns", "/real")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("vmax_rad_s", 0.05)
        self.declare_parameter("deadband_deg", 1.0)
        self.declare_parameter("watchdog_s", 0.5)
        # Full travel of the driven knuckle in radians, matching
        # fsr_gripper_node's CLOSED_RAD. The Kortex gripper takes a
        # NORMALISED 0..1 (0 open, 1 closed), so the two must agree or the
        # grip is silently scaled.
        self.declare_parameter("gripper_closed_rad", 0.8)
        # Resend only on a real change: SendGripperCommand is a round trip on
        # the same session as the joint speeds, and re-sending an unchanged
        # target every cycle would spend the loop's budget achieving nothing.
        self.declare_parameter("gripper_min_delta", 0.02)
        self.declare_parameter("gripper_enabled", True)
        self.declare_parameter("report_every_s", 5.0)

        self.arm = self.get_parameter("arm").value
        self.ns = self.get_parameter("real_ns").value.rstrip("/")
        self.rate = float(self.get_parameter("rate_hz").value)
        self.kp = float(self.get_parameter("kp").value)
        self.vmax = float(self.get_parameter("vmax_rad_s").value)
        self.deadband = math.radians(
            float(self.get_parameter("deadband_deg").value))
        self.watchdog = float(self.get_parameter("watchdog_s").value)
        self.report_every = float(self.get_parameter("report_every_s").value)

        self.names = [f"{self.arm}_joint_{i}" for i in range(1, NJ + 1)]

        # Shared between the ROS callbacks and the control thread.
        self._lock = threading.Lock()
        self._target = None
        self._target_t = 0.0
        self._target_vel = None       # velocities field, when the sender fills it
        self._prev_target = None
        self._prev_target_t = 0.0
        self._estopped = False
        self._running = True
        self._commanding = False      # False until the first target arrives

        # metrics
        self._lat = []
        self._loop_dt = []
        self._last_err = [0.0] * NJ

        # --- Kortex session: exactly one, shared by read and write ---
        self._tr = None
        self._ss = None
        self.base = None
        self._connect()

        self.js_pub = self.create_publisher(
            JointState, f"{self.ns}/joint_states", 10)
        self.stat_pub = self.create_publisher(
            Float64MultiArray, f"/highlevel_status_{self.arm}", 10)
        # Session health, published so recovery_manager can see a session die
        # WITHOUT inferring it from joint feedback -- which goes flat rather
        # than absent and is therefore useless as a liveness signal.
        self.session_pub = self.create_publisher(String, "/real/session_state", 10)
        self.create_timer(0.5, self._publish_session_state)
        self.create_service(Trigger, "/real/session_recover", self._srv_recover)
        self._grip_target = None       # normalised 0..1, None = never commanded
        self._grip_sent = None
        self._grip_fail = 0
        self._session_ok = True
        self._session_err = ""
        self._recoveries = 0

        self.create_subscription(
            JointTrajectory,
            f"{self.ns}/{self.arm}_arm_controller/joint_trajectory",
            self.on_target, 10)
        self.create_subscription(Bool, "/estop_state", self.on_estop, 10)
        # GRIPPER, over the SAME session. fsr_gripper_node already publishes a
        # JointTrajectory here with the driven-knuckle angle in radians, and it
        # owns all the behaviour that was tuned on real pads: the 250-count
        # deadband, per-arm range, output hysteresis, latch close at 1200 /
        # release at 400 sustained 0.5 s, and the latch surviving a clutch
        # disengage. Subscribing DOWNSTREAM of it means none of that is
        # reimplemented here and none of it can drift out of step -- this is
        # purely a transport from the knuckle angle to the Kortex gripper.
        self.create_subscription(
            JointTrajectory,
            f"/{self.arm}_gripper_controller/joint_trajectory",
            self.on_gripper, 10)

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            "high-level bridge up: %.0f Hz, kp %.2f, vmax %.3f rad/s "
            "(%.2f deg/s), deadband %.2f deg, watchdog %.2f s"
            % (self.rate, self.kp, self.vmax, math.degrees(self.vmax),
               math.degrees(self.deadband), self.watchdog))

    # ---------------- session ----------------

    def _connect(self):
        ip = self.get_parameter("robot_ip").value
        port = int(self.get_parameter("port").value)
        self.get_logger().info("connecting to %s:%d ..." % (ip, port))
        self._tr = TCPTransport()
        self._tr.connect(ip, port)
        self._rt = RouterClient(
            self._tr, lambda e: self.get_logger().warn("[router] %s" % e))
        self._ss = SessionManager(self._rt)
        info = Session_pb2.CreateSessionInfo()
        info.username = self.get_parameter("username").value
        info.password = self.get_parameter("password").value
        info.session_inactivity_timeout = 60000
        info.connection_inactivity_timeout = 2000
        self._ss.CreateSession(info)
        self.base = BaseClient.BaseClient(self._rt)
        state = Base_pb2.ArmState.Name(self.base.GetArmState().active_state)
        self.get_logger().info("session created -- arm state %s" % state)
        if state != "ARMSTATE_SERVOING_READY":
            self.get_logger().warn(
                "arm is NOT servoing-ready (%s); motion may be refused" % state)

    def _read_angles_rad(self):
        """Kortex degrees -> ROS radians, via the one conversion module."""
        fb = self.base.GetMeasuredJointAngles()
        deg = [0.0] * NJ
        for ja in fb.joint_angles:
            if 0 <= ja.joint_identifier < NJ:
                deg[ja.joint_identifier] = float(ja.value)
        return [kortex_deg_to_ros_rad(d) for d in deg]

    def on_gripper(self, msg):
        """Knuckle radians -> normalised Kortex gripper position."""
        if not msg.points or not msg.points[0].positions:
            return
        rad = float(msg.points[0].positions[0])
        closed = float(self.get_parameter("gripper_closed_rad").value)
        self._grip_target = max(0.0, min(1.0, rad / max(1e-6, closed)))

    def _send_gripper(self):
        """Push the gripper target on the EXISTING session. Never opens one.

        The arm permits exactly one session, so this deliberately reuses
        self.base rather than connecting: a second session would be refused
        and would leave the first unusable.
        """
        if not bool(self.get_parameter("gripper_enabled").value):
            return
        t = self._grip_target
        if t is None or self.base is None:
            return
        if self._grip_sent is not None and \
                abs(t - self._grip_sent) < float(
                    self.get_parameter("gripper_min_delta").value):
            return
        cmd = Base_pb2.GripperCommand()
        cmd.mode = Base_pb2.GRIPPER_POSITION
        f = cmd.gripper.finger.add()
        f.finger_identifier = 1
        f.value = float(t)
        try:
            self.base.SendGripperCommand(cmd)
            self._grip_sent = t
            self._grip_fail = 0
        except Exception as e:                                  # noqa: BLE001
            # Do NOT let a gripper fault take down the arm loop: the arm is
            # the safety-relevant path and must keep being commanded.
            self._grip_fail += 1
            if self._grip_fail in (1, 20):
                self.get_logger().error(
                    "gripper command failed (%d): %s. The ARM loop is "
                    "unaffected; grippers are on the internal bus of this "
                    "same session." % (self._grip_fail, e))

    def _send_speeds_rad(self, speeds_rad):
        """rad/s -> Kortex deg/s. Returns send latency in ms."""
        cmd = Base_pb2.JointSpeeds()
        for i in range(NJ):
            js = cmd.joint_speeds.add()
            js.joint_identifier = i
            js.value = float(math.degrees(speeds_rad[i]))
        t0 = time.monotonic()
        self.base.SendJointSpeedsCommand(cmd)
        return (time.monotonic() - t0) * 1000.0

    # ---------------- inputs ----------------

    def on_target(self, msg):
        if not msg.points:
            return
        pt = msg.points[0]
        pos = list(pt.positions)
        if len(pos) < NJ:
            return
        vel = list(pt.velocities) if pt.velocities else None
        # Trust joint_names when present rather than positional order.
        if msg.joint_names and len(msg.joint_names) >= NJ:
            idx = {n: i for i, n in enumerate(msg.joint_names)}
            if all(n in idx for n in self.names):
                pos = [pt.positions[idx[n]] for n in self.names]
                if vel is not None and len(vel) >= NJ:
                    vel = [pt.velocities[idx[n]] for n in self.names]
        with self._lock:
            self._prev_target = self._target
            self._prev_target_t = self._target_t
            self._target = pos[:NJ]
            self._target_vel = (vel[:NJ] if vel is not None and len(vel) >= NJ
                                else None)
            self._target_t = time.monotonic()
            self._commanding = True

    def on_estop(self, msg):
        with self._lock:
            was = self._estopped
            self._estopped = bool(msg.data)
        if self._estopped and not was:
            self.get_logger().error(
                "E-STOP -- commanding zero speed (SOFT stop, recoverable)")

    # ---------------- the loop ----------------

    def _control_loop(self):
        period = 1.0 / self.rate
        next_t = time.monotonic()
        last_report = time.monotonic()
        while self._running:
            loop_start = time.monotonic()
            try:
                actual = self._read_angles_rad()
            except Exception as e:
                self.get_logger().error("read failed: %s" % e)
                time.sleep(period)
                continue

            # Publish feedback FIRST, so downstream (bridge lag monitor,
            # robot_state_publisher, homing) sees fresh state even on a cycle
            # where we end up commanding zero.
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.names)
            js.position = list(actual)
            self.js_pub.publish(js)

            with self._lock:
                target = list(self._target) if self._target else None
                t_vel = (list(self._target_vel) if self._target_vel
                         else None)
                prev = list(self._prev_target) if self._prev_target else None
                prev_dt = self._target_t - self._prev_target_t
                t_age = time.monotonic() - self._target_t
                estopped = self._estopped
                commanding = self._commanding

            reason = None
            if estopped:
                speeds = [0.0] * NJ
                reason = "estop"
            elif not commanding or target is None:
                speeds = [0.0] * NJ
                reason = "no target yet"
            elif t_age > self.watchdog:
                speeds = [0.0] * NJ
                reason = "watchdog (%.2f s since last target)" % t_age
            else:
                # FEEDFORWARD + FEEDBACK, and the feedforward is not optional.
                #
                # Both senders publish an INCREMENTAL setpoint: real_homing_node
                # publishes cur + v*dt, and sim_to_real_bridge advances its own
                # last_cmd by at most max_step. So the position error this node
                # sees is one cycle of travel -- about 1.7 mrad at vmax -- which
                # is an order of magnitude INSIDE the 1 deg (17 mrad) deadband.
                # A pure position law therefore commands exactly zero forever:
                # the arm never moves, the setpoint never advances because it
                # tracks measured position, and homing stalls at its starting
                # error. That is precisely what happened on the first run --
                # "worst joint err 0.0025 rad" while the true error was 2.9 rad.
                #
                # So take the setpoint's own RATE as the command, and use the
                # position error only as a correction:
                #   speed = clip(v_ff + kp*err, -vmax, +vmax)
                # real_homing_node fills velocities[] with the speeds its own
                # law computed, so that path is exact -- the law runs once, in
                # homing, and this node executes it. sim_to_real_bridge sends
                # positions only, so differentiate its setpoint stream instead.
                delta = pose_delta_rad(target, actual, CONTINUOUS_IDX)
                self._last_err = delta

                if t_vel is not None:
                    v_ff = [clip(v, -self.vmax, self.vmax) for v in t_vel]
                elif prev is not None and prev_dt > 1e-6:
                    step = pose_delta_rad(target, prev, CONTINUOUS_IDX)
                    v_ff = [clip(s / prev_dt, -self.vmax, self.vmax)
                            for s in step]
                else:
                    v_ff = [0.0] * NJ

                speeds = []
                for i in range(NJ):
                    # The deadband suppresses the CORRECTION only. Applying it
                    # to the total would reintroduce the deadlock above.
                    fb = 0.0 if abs(delta[i]) <= self.deadband \
                        else self.kp * delta[i]
                    speeds.append(clip(v_ff[i] + fb, -self.vmax, self.vmax))

            try:
                lat = self._send_speeds_rad(speeds)
                self._lat.append(lat)
            except Exception as e:
                self.get_logger().error("send failed: %s" % e)

            # AFTER the arm speeds, never before: the arm is the
            # safety-relevant path and must not be delayed by the gripper.
            # _send_gripper() is a no-op unless the target actually changed,
            # so a still hand costs nothing on the wire.
            self._send_gripper()

            if reason and reason != "no target yet":
                self.get_logger().warn("zero speed: %s" % reason,
                                       throttle_duration_sec=2.0)

            now = time.monotonic()
            self._loop_dt.append(now - loop_start)
            if now - last_report >= self.report_every:
                self._report()
                last_report = now

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # Fell behind: resync rather than accumulate debt.
                next_t = time.monotonic()

    def _report(self):
        if not self._lat:
            return
        lat, dts = self._lat[-300:], self._loop_dt[-300:]
        achieved = 1.0 / (sum(dts) / len(dts)) if dts else 0.0
        worst = max(abs(e) for e in self._last_err) if self._last_err else 0.0
        self.get_logger().info(
            "rate %.1f Hz (target %.0f) | send latency min %.1f avg %.1f "
            "max %.1f ms | worst joint err %.4f rad (%.2f deg)"
            % (achieved, self.rate, min(lat), sum(lat) / len(lat), max(lat),
               worst, math.degrees(worst)))
        m = Float64MultiArray()
        m.data = [achieved, min(lat), sum(lat) / len(lat), max(lat), worst]
        self.stat_pub.publish(m)
        self._lat = self._lat[-300:]
        self._loop_dt = self._loop_dt[-300:]

    # ---------------- shutdown ----------------

    # ------------------------------------------------- session recovery
    def _publish_session_state(self):
        m = String()
        m.data = json.dumps(dict(
            arm=self.arm, connected=bool(self._session_ok and self.base),
            error=self._session_err, recoveries=self._recoveries))
        self.session_pub.publish(m)

    def _close_session(self):
        """Close cleanly. THE ARM PERMITS EXACTLY ONE SESSION.

        A leaked session blocks the next connect, so every step is attempted
        even if an earlier one raised -- a failed Stop() must not skip the
        CloseSession that frees the slot.
        """
        # The gripper is deliberately NOT opened here. A latched grip may be
        # holding something, and dropping it on shutdown would be a worse
        # failure than leaving it closed. Stopping the arm is what matters.
        for what, fn in (("zero speeds", lambda: self._send_speeds_rad([0.0] * NJ)),
                         ("Stop()", lambda: self.base.Stop()),
                         ("CloseSession", lambda: self._ss.CloseSession()),
                         ("disconnect", lambda: self._tr.disconnect())):
            try:
                if fn is not None:
                    fn()
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warn("recover: %s failed: %s" % (what, e))
        self.base = None
        self._ss = None
        self._rt = None
        self._tr = None

    def _srv_recover(self, req, res):
        """Recover the session IN PROCESS. Never a relaunch.

        The hardware component is deliberately NOT deactivated: deactivating
        tears down the API router and on_activate then fails permanently with
        `KBasicException: Router is not active`, so the driver-level e-stop is
        one-way and recovery through it means a relaunch. A participant
        session cannot absorb a relaunch -- the arm re-homes, the condition
        order is lost and the block is discarded. So recovery is: close the
        one permitted session cleanly, then create a fresh one.
        """
        self.get_logger().warn(
            "SESSION RECOVERY requested: closing the old session first "
            "(the arm permits only one; a leaked session refuses the next "
            "connect), then creating a fresh one.")
        self._session_ok = False
        self._close_session()
        try:
            self._connect()
            self._session_ok = True
            self._session_err = ""
            self._recoveries += 1
            res.success = True
            res.message = ("fresh session created without a relaunch "
                           "(recovery #%d)" % self._recoveries)
            self.get_logger().warn("SESSION RECOVERED: %s" % res.message)
        except Exception as e:                                  # noqa: BLE001
            self._session_err = str(e)
            res.success = False
            res.message = "reconnect failed: %s" % e
            self.get_logger().error(
                "SESSION RECOVERY FAILED: %s. The old session was closed, so "
                "the slot is free; retry is safe." % e)
        return res

    def shutdown(self):
        """Zero speeds, Stop(), close the session. Safe to call twice."""
        if not self._running:
            return
        self._running = False
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        # Zero speed FIRST and unconditionally -- this is the one call that
        # actually stops the arm, so it must not be skipped by an earlier
        # failure in the teardown.
        for attempt in ("zero speeds", "Stop()"):
            try:
                if attempt == "zero speeds":
                    self._send_speeds_rad([0.0] * NJ)
                else:
                    self.base.Stop()
            except Exception as e:
                sys.stderr.write("shutdown: %s failed: %s\n" % (attempt, e))
        try:
            self._ss.CloseSession()
        except Exception as e:
            sys.stderr.write("shutdown: CloseSession failed: %s\n" % e)
        try:
            self._tr.disconnect()
        except Exception as e:
            sys.stderr.write("shutdown: disconnect failed: %s\n" % e)
        sys.stderr.write("kortex session closed cleanly\n")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KortexHighLevelBridge()

        # SIGINT/SIGTERM must reach shutdown() even though the control thread
        # is blocked in a socket call -- rclpy's default handler would only
        # break the spin.
        def _sig(signum, frame):
            if node is not None:
                node.shutdown()
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
