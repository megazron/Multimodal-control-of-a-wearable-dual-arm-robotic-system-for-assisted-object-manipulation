#!/usr/bin/env python3
"""
master_teleop_node.py -- THE MASTER TELEOPERATION PATH. Smoothness is the
point of this file, and every constant in it was measured.

A SEPARATE NODE, DELIBERATELY. `ik_follower_node` is the baseline condition of
every experiment in this repo and it is not touched: it keeps its topics, its
parameters and its recordings, and a run that compares the two must be able to
launch either without the other changing. This node publishes the same
JointTrajectory on the same controller topics, so it is a DROP-IN REPLACEMENT
for the follower and not an addition to it. Run exactly one of the two.

    ros2 run srl_teleop master_teleop --ros-args -p arms:=left,right

WHY IT EXISTS -- MEASURED ON recordings/trajectory_capture/capture_20260901_151026
(28/28 segments, both arms, clutch pinned engaged the whole way):

                              left        right
    command step, median      0.052 mm    0.043 mm
    command step, p95         1.99 mm     2.60 mm
    command step, p99         4.57 mm    16.06 mm
    command step, MAX        24.80 mm   135.78 mm      <-- the problem
    master frame_valid        99.8 %      91.9 %
    guard rejections               0          13

The median step is 43-52 MICRONS and the worst is 136 MILLIMETRES, which at
the 50 Hz the master publishes is 6.8 m/s of commanded hand speed. Those
outliers are 500-2700x the median. They are not motion, they are the serial
and sensing faults this repo has documented for months arriving as a position,
and they are what the operator feels as a snatch.

THE ORDER OF THE STAGES IS THE WHOLE DESIGN, and it is what a single filter
gets wrong. A low pass fed a 136 mm outlier does not remove it, it SPREADS it
over the filter's time constant -- the arm still goes there, just more slowly
and for longer. So the outlier must be REJECTED before anything smooths, and
the smoothing must then be told nothing happened:

  1. VALIDITY   stale or unstamped master frames are dropped. The right arm
                published 8.1% invalid frames in the capture above.
  2. SLEW LIMIT the target moves toward the master at most `max_step_m` per
                sample. A RATE LIMIT, not a rejector: the arm always gets
                where the operator is going, it just never jumps to it. The
                first version of this file rejected instead, and replaying
                the capture showed it took the right arm's worst step from
                135.77 mm to 134.46 mm -- rejecting a SUSTAINED offset only
                defers the lurch.
  3. ONE EURO   position and orientation, from `smoothing.py`. Chosen over the
                EMA because its cutoff opens with speed: steady when the hand
                is still, and NOT laggy when it is not. The EMA is one
                constant and has to pick.
  4. IK         /compute_ik, the same solver and the same seeding convention
                the rest of the repo uses.
  5. RUCKIG     `motion_generator.MotionGenerator` -- jerk-limited, phase
                synchronised, reading the arm's own joint_limits.yaml. This is
                what makes the seven joints ARRIVE TOGETHER; per-joint
                clamping is what put the hand 51.3 mm off the line the IK
                implied.

Stages 3 and 5 are both smoothers and they are not redundant. One Euro
smooths the OPERATOR'S HAND in Cartesian space, where tremor lives. Ruckig
smooths the ROBOT'S JOINTS, where the limits live. Neither can do the other's
job: a Cartesian filter cannot honour a jerk limit, and a joint-space
generator cannot tell tremor from intent.

SAFETY. This node adds no safety layer of its own and removes none:

  * `/estop_state` latched -> it stops commanding and resyncs. HARD
    CONSTRAINT 6's node is untouched and still the only path out.
  * the clutch, read live from `/master_status_<arm>`. Disengaged means HOLD,
    and re-engaging RE-ANCHORS rather than snapping to wherever the hand
    drifted to.
  * `/mount_guard` reporting a breach -> refuse to command. The guard is a
    watchdog that trips `/estop`, so it is reactive by construction; refusing
    on its report is cheaper than being stopped by it.
  * `motion_enabled` defaults FALSE (HARD CONSTRAINT 8) and `real_enabled`
    is a SECOND, separate parameter. Nothing reaches a real arm until both
    are set by hand.
  * a watchdog on the master itself: `master_timeout_s` of silence holds.

Every refusal is counted and named on `/master_teleop/status`, because a
teleop path that quietly stops following is indistinguishable from a dead one.

  python3 -m srl_teleop.master_teleop_node --self-test
"""
import argparse
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from srl_teleop import smoothing
from srl_teleop.motion_generator import MotionGenerator, MotionError

ARMS = ("left", "right")
DOF = 7

#: /master_status_<arm> index 5. master_pose_node names this layout in one
#: place (ST_FRAME_VALID); it is repeated here rather than imported so this
#: node keeps its own explicit contract with the topic, and the name is used
#: on both sides so an index cannot drift silently.
ST_CLUTCH = 0
ST_FRAME_VALID = 5

# ---------------------------------------------------------------------------
# MEASURED DEFAULTS. Every number here has a capture behind it; change one and
# say which measurement replaced it.
# ---------------------------------------------------------------------------

#: THE SLEW LIMIT: the largest step the target may take in one master sample,
#: in metres. This is a RATE LIMIT, not a rejector, and the difference is the
#: whole design.
#:
#: The first version of this file REJECTED an oversized step and held the last
#: good one. That is right for a single-sample outlier and WRONG for the fault
#: this rig actually has, which is a SUSTAINED displacement: hold through it
#: and the arm lurches the moment the signal is accepted again. Replayed
#: through capture_20260901_151026 the rejector took the right arm's worst
#: step from 135.77 mm to 134.46 mm -- i.e. it did nothing. Measured, not
#: reasoned: that replay is why this is a slew limit.
#:
#: SWEPT on that capture (frame_valid gate + slew + one euro), left arm:
#:
#:     slew    max step    p99      lag median   lag p95
#:     none    24.80 mm    4.57 mm      -            -
#:      2 mm    2.70 mm    2.16 mm   0.49 mm     11.56 mm
#:      4 mm    5.09 mm    3.33 mm   0.48 mm      3.87 mm
#:      6 mm    7.05 mm    3.42 mm   0.48 mm      3.71 mm   <-- the knee
#:      8 mm    9.23 mm    3.39 mm   0.48 mm      3.66 mm
#:     20 mm   12.56 mm    3.38 mm   0.48 mm      3.65 mm
#:
#: 6 mm is where the lag stops improving: below it p95 lag climbs steeply
#: (11.56 mm at 2 mm) for no further gain in smoothness. At the master's
#: 50 Hz, 6 mm/sample is 0.30 m/s of commanded hand speed, which is above
#: every p99 in the capture, so ordinary motion is untouched.
DEFAULT_MAX_STEP_M = 0.006

#: One Euro, in METRES. `smoothing.OneEuro`'s docstring records what a beta
#: copied from the paper's pixel examples costs here: 55.8 mm of lag at
#: 0.40 m/s. These are that module's own measured defaults.
DEFAULT_MIN_CUTOFF_HZ = 0.5
DEFAULT_BETA = 100.0

#: A master frame older than this is not a position, it is history.
DEFAULT_MAX_FRAME_AGE_S = 0.20

#: Silence longer than this holds the arm where it is.
DEFAULT_MASTER_TIMEOUT_S = 0.50

#: Control cycle. The master publishes at ~50 Hz and the high-level bridge
#: consumes at 30 Hz; generating at 100 Hz keeps Ruckig's profile smooth
#: without inventing master data that does not exist.
DEFAULT_RATE_HZ = 100.0


class SlewGate:
    """Rate-limit a position stream. Never a jump; always bounded motion.

    Returns the limited position and whether the limit bound this sample, so
    a caller can report how hard the operator is pushing against it.

    PURE, and separate from the node, so the self-test drives it with a signal
    whose ground truth is arithmetic.

    WHY NOT A REJECTOR. A rejector answers "this step is too big" with "then
    do not move", which is correct for one bad sample and wrong for a
    sustained offset -- the arm sits still and then lurches when the signal is
    finally accepted. A slew limit answers the same question with "then get
    there at a rate I am willing to command", which is right in both cases and
    is the only one of the two that BOUNDS the output by construction.
    """

    def __init__(self, max_step_m=DEFAULT_MAX_STEP_M):
        if not (max_step_m > 0.0) or not math.isfinite(max_step_m):
            raise ValueError(
                "max_step_m must be a positive finite number of metres, got "
                "%r -- zero would freeze the arm" % (max_step_m,))
        self.max_step_m = float(max_step_m)
        self.reset()

    def reset(self):
        self._last = None
        self.limited = 0
        self.passed = 0

    def __call__(self, p):
        p = np.asarray(p, float)
        if self._last is None:
            self._last = p.copy()
            self.passed += 1
            return p.copy(), False
        d = p - self._last
        n = float(np.linalg.norm(d))
        if n <= self.max_step_m:
            self._last = p.copy()
            self.passed += 1
            return p.copy(), False
        self._last = self._last + d * (self.max_step_m / n)
        self.limited += 1
        return self._last.copy(), True


class ArmChannel:
    """Everything one arm needs. Two of these, never shared."""

    def __init__(self, arm, node):
        self.arm = arm
        self.node = node
        self.gate = SlewGate(node.p_max_step)
        self.pos_filt = smoothing.make(
            "one_euro", min_cutoff=node.p_min_cutoff, beta=node.p_beta)
        self.rot_filt = smoothing.OneEuroQuat()
        try:
            self.gen = MotionGenerator(arm, velocity_cap_rad_s=node.p_vmax)
        except MotionError as exc:
            node.get_logger().error(
                "[%s] motion generator refused to build: %s" % (arm, exc))
            raise
        self.target = None            # filtered Cartesian target
        self.target_q = None
        self.last_rx = None           # monotonic time of the last good frame
        self.clutch = None
        self.frame_valid = None
        self.n_invalid = 0
        self.anchor = None            # hand position when the clutch engaged
        self.cmd_anchor = None        # commanded position at that moment
        self.ik_q = None              # newest IK solution
        self.ik_inflight = False
        self.seeded = False
        self.n_ik_fail = 0
        self.n_stale = 0
        self.n_held = 0
        self.n_published = 0
        self.last_reason = "starting"

    # ------------------------------------------------------------ input
    def on_pose(self, msg):
        now = time.monotonic()
        st = msg.header.stamp
        if st.sec == 0 and st.nanosec == 0:
            # UNSTAMPED IS NOT FRESH. master_pose_node republishes its last
            # good frame at full rate when the master degrades, so arrival
            # time cannot see a frozen-but-talking master. estop_node learned
            # this the same way; the rule belongs here too.
            self.n_stale += 1
            self.last_reason = "master frame carries no timestamp"
            return
        age = (self.node.get_clock().now()
               - rclpy.time.Time.from_msg(st)).nanoseconds / 1e9
        if age > self.node.p_max_age:
            self.n_stale += 1
            self.last_reason = "master frame %.3f s old" % age
            return

        if self.frame_valid is False:
            # THE MASTER ITSELF SAYS THIS FRAME IS NOT VALID, and until
            # 2026-09-01 nothing downstream read that bit.
            #
            # MEASURED on capture_20260901_151026: the right arm published
            # frame_valid=0 on 8.1% of samples, and ALL 27 of its reach
            # discontinuities over 50 mm have frame_valid=0 on one side --
            # 27 of 27, not a tendency. On an invalid frame the derived reach
            # collapses from ~0.26 m to ~0.09 m, and that 170 mm collapse
            # rendered as a POSITION is the 135.78 mm command step this node
            # was written to remove. It is not tremor and no filter removes
            # it: a low pass fed a teleport still goes there.
            #
            # So it is dropped, not smoothed. The arm holds the last valid
            # pose, which is the honest reading of "the master cannot
            # currently tell me where the hand is".
            self.n_invalid += 1
            self.last_reason = "master reports frame_valid=0"
            return

        p = np.array([msg.pose.position.x, msg.pose.position.y,
                      msg.pose.position.z], float)
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w], float)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)):
            self.n_stale += 1
            self.last_reason = "master frame is not finite"
            return

        p, _limited = self.gate(p)

        dt = 0.02 if self.last_rx is None else max(1e-3, now - self.last_rx)
        self.last_rx = now
        self.target = self.pos_filt(p, dt)
        self.target_q = self.rot_filt(q, dt)

    def on_status(self, msg):
        """/master_status_<arm>: element 0 is the clutch."""
        if not msg.data:
            return
        if len(msg.data) > ST_FRAME_VALID:
            self.frame_valid = bool(msg.data[ST_FRAME_VALID] > 0.5)
        engaged = bool(msg.data[ST_CLUTCH] > 0.5)
        if self.clutch is not None and engaged and not self.clutch:
            # RE-ENGAGE RE-ANCHORS. Without this the arm leaps to wherever the
            # operator's hand drifted while the clutch was out, which is the
            # single most alarming thing a worn rig can do.
            self.anchor = None
            self.pos_filt.reset()
            self.rot_filt.reset()
            self.gate.reset()
            self.node.get_logger().info(
                "[%s] clutch re-engaged -- re-anchored" % self.arm)
        self.clutch = engaged


class MasterTeleop(Node):

    def __init__(self):
        super().__init__("master_teleop_node")

        self.declare_parameter("arms", "left,right")
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)
        self.declare_parameter("max_step_m", DEFAULT_MAX_STEP_M)
        self.declare_parameter("min_cutoff_hz", DEFAULT_MIN_CUTOFF_HZ)
        self.declare_parameter("beta", DEFAULT_BETA)
        self.declare_parameter("max_frame_age_s", DEFAULT_MAX_FRAME_AGE_S)
        self.declare_parameter("master_timeout_s", DEFAULT_MASTER_TIMEOUT_S)
        # Velocity cap handed to the generator. None/0 means "the yaml's own".
        self.declare_parameter("vmax_rad_s", 0.0)
        # HARD CONSTRAINT 8. Both default FALSE and neither implies the other.
        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("real_enabled", False)
        self.declare_parameter("real_ns", "/real")
        self.declare_parameter("ik_timeout_s", 0.05)
        self.declare_parameter("report_every_s", 5.0)

        # ACCEPTS "both", because that is what teleop.launch.py's `arm`
        # argument carries and what every other node in this stack is handed.
        # A node that silently means something different by the same word is
        # how one arm ends up unmanaged with nothing saying so.
        raw = str(self.get_parameter("arms").value).strip()
        arms = (list(ARMS) if raw.lower() in ("both", "", "all") else
                [a.strip() for a in raw.split(",") if a.strip()])
        bad = [a for a in arms if a not in ARMS]
        if bad:
            raise ValueError(
                "unknown arm(s) %s -- expected 'both', or left and/or right "
                "as a comma-separated list" % ", ".join(bad))
        self.arms = arms
        self.rate = float(self.get_parameter("rate_hz").value)
        self.p_max_step = float(self.get_parameter("max_step_m").value)
        self.p_min_cutoff = float(self.get_parameter("min_cutoff_hz").value)
        self.p_beta = float(self.get_parameter("beta").value)
        self.p_max_age = float(self.get_parameter("max_frame_age_s").value)
        self.p_timeout = float(self.get_parameter("master_timeout_s").value)
        cap = float(self.get_parameter("vmax_rad_s").value)
        self.p_vmax = cap if cap > 0.0 else None
        self.ns = str(self.get_parameter("real_ns").value)

        self.estop = False
        self.guard_breach = ""
        self.js = {}
        self._last_report = 0.0

        self.ch = {}
        for a in self.arms:
            self.ch[a] = ArmChannel(a, self)

        # ---- publishers: the SAME topics the follower uses, so this is a
        # ---- replacement and not a second commander.
        self.cmd_pub = {
            a: self.create_publisher(
                JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 10)
            for a in self.arms}
        self.real_pub = {
            a: self.create_publisher(
                JointTrajectory,
                "%s/%s_arm_controller/joint_trajectory" % (self.ns, a), 10)
            for a in self.arms}
        self.status_pub = self.create_publisher(
            String, "/master_teleop/status", 10)

        cb = MutuallyExclusiveCallbackGroup()
        for a in self.arms:
            self.create_subscription(
                PoseStamped, "/master_arm_pose_%s" % a,
                self.ch[a].on_pose, 20, callback_group=cb)
            self.create_subscription(
                Float64MultiArray, "/master_status_%s" % a,
                self.ch[a].on_status, 20, callback_group=cb)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20,
                                 callback_group=cb)
        self.create_subscription(Bool, "/estop_state", self.on_estop, 10,
                                 callback_group=cb)
        self.create_subscription(String, "/mount_guard", self.on_guard, 10,
                                 callback_group=cb)

        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

        # RE-ANCHOR ON DEMAND. The operator's own way of saying "I have moved,
        # start from here" without stopping the stack.
        self.create_service(Trigger, "/master_teleop/reanchor",
                            self.srv_reanchor)

        self.timer = self.create_timer(1.0 / self.rate, self.tick,
                                       callback_group=cb)
        self.get_logger().info(
            "master teleop up on %s | slew %.0f mm/sample, one-euro "
            "min_cutoff %.2f Hz beta %.0f | %s | motion_enabled=%s "
            "real_enabled=%s"
            % (", ".join(self.arms), self.p_max_step * 1000.0,
               self.p_min_cutoff, self.p_beta,
               self.ch[self.arms[0]].gen.backend,
               self.get_parameter("motion_enabled").value,
               self.get_parameter("real_enabled").value))

    # ---------------------------------------------------------------- input
    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def on_estop(self, m):
        was = self.estop
        self.estop = bool(m.data)
        if self.estop and not was:
            self.get_logger().error(
                "E-STOP LATCHED -- master teleop has stopped commanding. "
                "Clear with /estop_reset.")
            for c in self.ch.values():
                c.seeded = False

    def on_guard(self, m):
        """The mount guard's own report. Refuse while it names a breach."""
        txt = m.data or ""
        self.guard_breach = "" if ("BREACH" not in txt.upper()) else txt[:120]

    def srv_reanchor(self, req, resp):
        for c in self.ch.values():
            c.anchor = None
            c.seeded = False
            c.pos_filt.reset()
            c.rot_filt.reset()
            c.gate.reset()
        resp.success = True
        resp.message = "re-anchored %s" % ", ".join(self.arms)
        return resp

    def measured_q(self, arm):
        names = ["%s_joint_%d" % (arm, i) for i in range(1, DOF + 1)]
        if not all(n in self.js for n in names):
            return None
        return [float(self.js[n]) for n in names]

    # ----------------------------------------------------------------- loop
    def tick(self):
        now = time.monotonic()
        dt = 1.0 / self.rate
        lines = []
        for a in self.arms:
            lines.append(self.tick_arm(a, now, dt))
        if now - self._last_report >= float(
                self.get_parameter("report_every_s").value):
            self._last_report = now
            msg = String()
            msg.data = " | ".join(lines)
            self.status_pub.publish(msg)

    def tick_arm(self, arm, now, dt):
        c = self.ch[arm]
        q_meas = self.measured_q(arm)

        # -- the refusals, in the order that makes the reason legible --------
        why = None
        if self.estop:
            why = "E-STOP LATCHED"
        elif self.guard_breach:
            why = "mount guard: %s" % self.guard_breach
        elif q_meas is None:
            why = "no /joint_states for %s" % arm
        elif c.target is None:
            why = "no master pose yet"
        elif c.last_rx is None or (now - c.last_rx) > self.p_timeout:
            why = "master silent %.2f s" % (
                0.0 if c.last_rx is None else now - c.last_rx)
        elif c.clutch is False:
            why = "clutch disengaged -- holding"

        if why is not None:
            # HOLD, and RESYNC. A generator that keeps integrating while its
            # output is not published believes the arm followed commands that
            # were never sent, and the next accepted command starts from a
            # fiction. `resync` is the documented way back to reality.
            if q_meas is not None:
                c.gen.resync(q_meas)
                c.seeded = True
            c.n_held += 1
            c.last_reason = why
            return "%s HOLD (%s)" % (arm, why)

        if not c.seeded:
            c.gen.resync(q_meas)
            c.seeded = True

        # -- IK, one in flight at a time -------------------------------------
        self.request_ik(c, q_meas)
        if c.ik_q is None:
            c.last_reason = "waiting for the first IK solution"
            return "%s WAIT (no IK yet)" % arm

        # -- Ruckig ----------------------------------------------------------
        try:
            step = c.gen.step(c.ik_q, dt)
        except MotionError as exc:
            c.gen.resync(q_meas)
            c.last_reason = "motion generator: %s" % exc
            return "%s REFUSED (%s)" % (arm, exc)

        if not bool(self.get_parameter("motion_enabled").value):
            # ARMED SEPARATELY, and it still computes so the whole path can be
            # watched on /master_teleop/status before anything moves.
            c.gen.resync(q_meas)
            c.last_reason = "motion_enabled=false"
            return ("%s DRY (motion_enabled=false, would move %.4f rad)"
                    % (arm, step.travel_rad))

        self.publish(arm, step)
        c.n_published += 1
        c.last_reason = "following"
        return ("%s FOLLOW invalid %d slewed %d/%d ikfail %d stale %d "
                "limited_by %s"
                % (arm, c.n_invalid, c.gate.limited,
                   c.gate.limited + c.gate.passed,
                   c.n_ik_fail, c.n_stale, step.limited_by))

    def request_ik(self, c, q_meas):
        if c.ik_inflight or not self.ik_cli.service_is_ready():
            return
        req = GetPositionIK.Request()
        req.ik_request.group_name = "%s_arm" % c.arm
        req.ik_request.avoid_collisions = False
        req.ik_request.robot_state.joint_state.name = [
            "%s_joint_%d" % (c.arm, i) for i in range(1, DOF + 1)]
        req.ik_request.robot_state.joint_state.position = list(q_meas)
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(c.target[0])
        ps.pose.position.y = float(c.target[1])
        ps.pose.position.z = float(c.target[2])
        q = c.target_q if c.target_q is not None else np.array([0., 0., 0., 1.])
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = Duration(
            sec=0, nanosec=int(float(
                self.get_parameter("ik_timeout_s").value) * 1e9))

        c.ik_inflight = True
        fut = self.ik_cli.call_async(req)

        def done(f, ch=c):
            ch.ik_inflight = False
            try:
                res = f.result()
            except Exception:                                # noqa: BLE001
                ch.n_ik_fail += 1
                return
            if res is None or res.error_code.val != 1:
                ch.n_ik_fail += 1
                return
            names = list(res.solution.joint_state.name)
            pos = list(res.solution.joint_state.position)
            want = ["%s_joint_%d" % (ch.arm, i) for i in range(1, DOF + 1)]
            try:
                ch.ik_q = [float(pos[names.index(n)]) for n in want]
            except ValueError:
                ch.n_ik_fail += 1
        fut.add_done_callback(done)

    def publish(self, arm, step):
        t = JointTrajectory()
        t.joint_names = ["%s_joint_%d" % (arm, i) for i in range(1, DOF + 1)]
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in step.position]
        pt.velocities = [float(v) for v in step.velocity]
        dt = 1.0 / self.rate
        pt.time_from_start = Duration(sec=0, nanosec=int(dt * 1e9))
        t.points = [pt]
        self.cmd_pub[arm].publish(t)
        if bool(self.get_parameter("real_enabled").value):
            self.real_pub[arm].publish(t)


# ===========================================================================
#  SELF-TEST. Constructed signals only -- the ground truth is arithmetic, so
#  a failure here is a failure of this file and not of a rig or a renderer.
#
#  AND EVERY CHECK IS ONE THAT CAN FAIL. The spike-gate checks below are run
#  against a signal built to contain the exact outlier measured in
#  capture_20260901_151026, and the smoothing checks compare against the
#  unfiltered signal, so "no improvement" fails rather than passing quietly.
# ===========================================================================
def self_test(verbose=True):                                  # noqa: C901
    fails = []

    def check(name, cond, detail=""):
        if verbose:
            print("  %-58s %s" % (name, "ok" if cond else "FAIL"))
            if not cond and detail:
                print("      %s" % detail)
        if not cond:
            fails.append(name)

    print("SlewGate")
    g = SlewGate(0.006)
    pts = [np.array([0.001 * i, 0.0, 0.0]) for i in range(50)]
    out = [g(p)[0] for p in pts]
    check("a 1 mm/sample ramp is never limited", g.limited == 0)
    check("and it is passed through unchanged", np.allclose(out[-1], pts[-1]))

    # THE MEASURED FAULT: a sustained 135.78 mm displacement, not one sample.
    g = SlewGate(0.006)
    g(np.zeros(3))
    far = np.array([0.13578, 0.0, 0.0])
    steps = [g(far)[0] for _ in range(60)]
    first = float(np.linalg.norm(steps[0]))
    check("the measured 135.78 mm jump is capped at the slew limit",
          first <= 0.006 + 1e-12, "first step %.5f m" % first)
    check("and the output BOUNDS every step by construction",
          max(float(np.linalg.norm(b - a))
              for a, b in zip(steps, steps[1:])) <= 0.006 + 1e-12)
    check("and it still ARRIVES -- a rate limit is not a rejector",
          np.allclose(steps[-1], far, atol=1e-6),
          "reached %r of %r" % (steps[-1], far))
    check("arrival takes ceil(135.78/6) = 23 samples",
          sum(1 for a, b in zip([np.zeros(3)] + steps, steps)
              if not np.allclose(a, b)) == 23,
          "took %d" % sum(1 for a, b in zip([np.zeros(3)] + steps, steps)
                          if not np.allclose(a, b)))

    # A REJECTOR WOULD NOT HAVE. This is the claim the redesign rests on, so
    # it is measured here rather than asserted in a comment.
    held = np.zeros(3)
    for _ in range(60):
        held = held if float(np.linalg.norm(far - held)) > 0.006 else far
    check("a pure rejector would still be at the origin after 60 samples",
          np.allclose(held, np.zeros(3)),
          "rejector reached %r" % (held,))

    try:
        SlewGate(0.0)
        check("max_step_m=0 is refused", False)
    except ValueError:
        check("max_step_m=0 is refused", True)

    print("stage ordering")
    # 6. gate-then-filter beats filter-alone on the REAL failure shape:
    #    a still hand with one outlier in it.
    rng = np.random.default_rng(3)
    n = 400
    dt = 0.02
    truth = np.zeros((n, 3))
    sig = truth + rng.normal(0.0, 0.0002, (n, 3))     # 0.2 mm tremor
    sig[200] += np.array([0.13578, 0.0, 0.0])         # the measured outlier

    f1 = smoothing.make("one_euro", min_cutoff=0.5, beta=100.0)
    filt_only = np.array([f1(x, dt) for x in sig])

    g = SlewGate(0.006)
    f2 = smoothing.make("one_euro", min_cutoff=0.5, beta=100.0)
    gated = np.array([f2(g(x)[0], dt) for x in sig])

    e_filt = float(np.abs(filt_only - truth).max())
    e_gate = float(np.abs(gated - truth).max())
    check("slew+filter beats filter alone on a spiked still signal",
          e_gate < e_filt,
          "gate+filter %.4f m vs filter alone %.4f m" % (e_gate, e_filt))
    check("and the residual excursion is under 1 mm",
          e_gate < 0.001, "%.5f m" % e_gate)

    # 7. the filter must still TRACK. A smoother that wins by not moving is
    #    not a smoother, and this is the check that catches it.
    ramp = np.stack([np.linspace(0, 0.40, n), np.zeros(n), np.zeros(n)], 1)
    g = SlewGate(0.006)
    f3 = smoothing.make("one_euro", min_cutoff=0.5, beta=100.0)
    tracked = np.array([f3(g(x)[0], dt) for x in ramp])
    lag = float(np.linalg.norm(ramp[-1] - tracked[-1]))
    check("a 0.40 m sweep is tracked to under 10 mm of lag",
          lag < 0.010, "lag %.4f m" % lag)
    check("and the 0.40 m sweep was not fighting the slew limit",
          g.limited < 0.5 * (g.limited + g.passed),
          "limited on %d of %d samples" % (g.limited, g.limited + g.passed))

    print("motion generator wiring")
    try:
        gen = MotionGenerator("left")
        gen.resync([0.0] * DOF)
        st = gen.step([0.1] * DOF, 0.01)
        check("generator builds and steps for the left arm",
              st is not None and len(st.position) == DOF)
        check("and it reports which joint is limiting",
              st.limited_by.startswith("left_joint_"))
        v = max(abs(x) for x in st.velocity)
        vmax = max(gen.limits.velocity)
        check("first cycle does not exceed the yaml velocity limit",
              v <= vmax + 1e-9, "%.4f vs %.4f rad/s" % (v, vmax))
    except MotionError as exc:
        check("generator builds and steps for the left arm", False, str(exc))

    print()
    if fails:
        print("SELF-TEST FAILED: %d" % len(fails))
        for f in fails:
            print("   - %s" % f)
        return 1
    print("SELF-TEST PASSED")
    return 0


def main(args=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--self-test", action="store_true")
    known, rest = ap.parse_known_args()
    if known.self_test:
        return self_test()

    rclpy.init(args=args)
    node = MasterTeleop()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ex.shutdown()
        except Exception:                                    # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:                                    # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
