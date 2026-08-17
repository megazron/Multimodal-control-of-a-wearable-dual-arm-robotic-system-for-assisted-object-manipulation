#!/usr/bin/env python3
"""
ik_follower_node.py — continuously moves the sim left_arm to match
/master_arm_pose, using MoveIt's existing /compute_ik service.
=============================================================================
Simpler than MoveIt Servo: reuses the /compute_ik service that move_group
already runs (no new package/config needed). For each incoming pose, asks
MoveIt to solve IK for left_arm, then publishes the resulting joint angles
straight to the left_arm_controller -- the same controller topic proven
working all session.

CONTINUOUS-JOINT WIND-UP
  joint_1/3/5/7 are type="continuous" in the Gen3 7-DOF URDF, so
  /joint_states reports them UNWRAPPED and they accumulate revolutions.
  TRAC-IK runs solve_type: Distance, which seeds from that unwrapped value
  and minimises distance from the seed -- so once a joint drifts a turn
  out, every later solution stays out there and it never comes back. The
  right arm reached joint_1 = -15.6 rad, joint_7 = -21.6 rad this way.

  Four guards, all below:
    - the IK seed is wrapped, so the solver never sees a wound-up seed
    - the returned solution is wrapped before it is published
    - solutions that jump more than max_step_rad in one cycle are rejected
    - time_from_start is derived from the TRUE unwrapped distance the
      controller actually has to travel, so a big traverse is given the
      time it needs instead of a fixed 0.1 s
  Plus a startup unwind: if the arm is already wound up when this node
  comes up, it is walked slowly back inside +/-pi before tracking starts.

  Expect the [STATS] success rate to DROP once rejections become visible.
  That is the guard working, not a regression.

Run:
  ros2 run srl_teleop ik_follower_node
"""
import sys
import os
import math
import time    # monotonic ONLY -- never time.time() for an interval, see CLAUDE.md
sys.path.insert(0, os.path.expanduser("~/kortex_ws/config"))
import home_positions  # loads from ~/kortex_ws/config/home_positions_<arm>.txt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Bool
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from tf2_ros import Buffer, TransformListener
from srl_teleop.block_monitor import BlockMonitor
from srl_teleop.clearance import (ClearanceModel, DISTAL_LINKS,
                                  REAL_ROBOT_PAD_M)

TWO_PI = 2.0 * math.pi

# 0-based indices of the joints declared type="continuous" in
# gen3_macro.xacro: joint_1, joint_3, joint_5, joint_7. Only these can
# wind up -- 2/4/6 are revolute and hard-limited well inside one turn,
# so wrapping them would be meaningless at best and wrong at worst.
CONTINUOUS_IDX = (0, 2, 4, 6)

# Joints perturbed when searching the Gen3's redundant DOF. joint_3 is the
# upper-arm roll: turning it swings the ELBOW around the shoulder-wrist axis
# while leaving the gripper pose alone, which is exactly the freedom needed
# to route the elbow away from the wearer.
REDUNDANT_IDX = (2,)


def joint_names_for(arm):
    return [f"{arm}_joint_{i}" for i in range(1, 8)]


def wrap_pi(a):
    """Wrap an angle into (-pi, pi]."""
    return math.pi - ((math.pi - a) % TWO_PI)


def ang_diff(a, b):
    """Shortest signed distance a-b, in (-pi, pi]. Used so that a step
    across the +/-pi boundary reads as the small move it really is,
    rather than as a spurious ~2pi jump."""
    return wrap_pi(a - b)


def wrap_continuous(positions):
    """Wrap only the continuous joints; leave the limited ones alone."""
    out = list(positions)
    for i in CONTINUOUS_IDX:
        out[i] = wrap_pi(out[i])
    return out


def joint_steps(solution, current_wrapped, continuous_idx=CONTINUOUS_IDX):
    """Per-joint move from current to solution, shortest-way for rolls."""
    return [ang_diff(t, c) if i in continuous_idx else t - c
            for i, (t, c) in enumerate(zip(solution, current_wrapped))]


def clamp_towards(solution, current_wrapped, max_step, continuous_idx=CONTINUOUS_IDX):
    """One bounded step from current toward solution.

    This is what breaks the deadlock. Rejecting a solution because it is far
    from the current state is self-defeating: the arm then does not move, so
    the next solution is exactly as far, forever (observed as reject_count
    == success_count, with the arm never leaving home). Walking toward it
    max_step at a time converges in ceil(distance / max_step) cycles.
    """
    out = []
    for i, (tgt, cur) in enumerate(zip(solution, current_wrapped)):
        d = ang_diff(tgt, cur) if i in continuous_idx else tgt - cur
        d = max(-max_step, min(max_step, d))
        v = cur + d
        out.append(wrap_pi(v) if i in continuous_idx else v)
    return out


def quat_angle(qa, qb):
    """Absolute angle between two (x,y,z,w) quaternions, in radians."""
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def pose_delta(a, b):
    """(metres, radians) between two (pos, quat) pairs. inf if either is None."""
    if a is None or b is None:
        return float("inf"), float("inf")
    dist = math.sqrt(sum((p - q) ** 2 for p, q in zip(a[0], b[0])))
    return dist, quat_angle(a[1], b[1])


def _duration(sec_float):
    sec = int(sec_float)
    return Duration(sec=sec, nanosec=int(round((sec_float - sec) * 1e9)))


class IKFollowerNode(Node):
    def __init__(self):
        super().__init__("ik_follower_node")
        self.declare_parameter("arm", "left")  # "left" or "right"
        # Largest joint move committed in one cycle. A solution further away
        # than this is now SLEWED toward (clamped and published), not
        # discarded -- see clamp_towards().
        self.declare_parameter("max_step_rad", 0.35)
        # A solution is only treated as a redundancy flip (and rejected)
        # when the TARGET POSE barely moved yet the SOLUTION jumped. These
        # deadbands define "barely moved".
        self.declare_parameter("pose_deadband_m", 0.002)
        self.declare_parameter("pose_deadband_rad", 0.02)
        # Backstop: never reject forever. If the solver keeps returning the
        # same far branch for a static target this many cycles running,
        # accept and slew to it. Without this, rejecting flips would just
        # recreate the deadlock in a different place.
        self.declare_parameter("flip_reject_limit", 10)
        # POSTURE BIAS. solve_type Distance seeds from the live joint state,
        # so the arm keeps whatever IK branch it drifted into and never comes
        # back -- the "gets stuck in a cramped region" failure recorded in
        # this project's notes. Biasing the SEED (not the solution) a little
        # toward the nominal home pulls the null space back over seconds
        # without ever jumping. w=0 disables.
        self.declare_parameter("posture_bias_w", 0.02)
        # ---- collision safety ----
        # HARD FLOOR: a solution whose clearance from the wearer is below this
        # is never published. The arm is bolted to the operator's back, so a
        # bad solution reaches a person, not just scenery.
        self.declare_parameter("min_clearance_m", 0.05)
        # Redundancy search. The Gen3 is 7-DOF: the elbow can swing without
        # moving the gripper. When the nearest solution is unsafe we re-seed
        # the redundant DOF and keep the SAFEST solution that still hits the
        # commanded pose, rather than giving up on the pose.
        self.declare_parameter("redundancy_samples", 6)
        self.declare_parameter("redundancy_span_rad", 2.0)
        # real_robot mode -- see CLAUDE.md. Slower, stricter, opt-in motion.
        self.declare_parameter("real_robot", False)
        # time_from_start = unwrapped_delta / max_vel, floored at min_time.
        self.declare_parameter("max_vel_rad_s", 0.6)
        self.declare_parameter("min_time_s", 0.05)
        # Startup unwind: slower than tracking, and stepped so the
        # controller gets a path rather than one huge setpoint.
        self.declare_parameter("unwind_vel_rad_s", 0.3)
        self.declare_parameter("unwind_step_rad", 0.25)

        self.arm = self.get_parameter("arm").value
        self.joint_names = joint_names_for(self.arm)
        self.max_step = self.get_parameter("max_step_rad").value
        self.pose_db_m = self.get_parameter("pose_deadband_m").value
        self.pose_db_rad = self.get_parameter("pose_deadband_rad").value
        self.flip_limit = int(self.get_parameter("flip_reject_limit").value)
        self.posture_w = float(self.get_parameter("posture_bias_w").value)
        self.min_clear = float(self.get_parameter("min_clearance_m").value)
        self.redundancy_n = int(self.get_parameter("redundancy_samples").value)
        self.redundancy_span = float(self.get_parameter("redundancy_span_rad").value)
        self.real_robot = bool(self.get_parameter("real_robot").value)
        # ORDER MATTERS HERE. These four MUST be read from parameters BEFORE
        # the real_robot block, for two reasons:
        #   1. the block reads self.max_vel, so reading it after was an
        #      AttributeError that crashed every real_robot:=true launch --
        #      which is why real_robot mode had never actually run;
        #   2. re-reading max_vel_rad_s afterwards silently DISCARDED the
        #      0.10 rad/s safety cap the block had just applied.
        # A safety clamp that is overwritten one line later is worse than no
        # clamp, because the warning still prints.
        self.max_vel = float(self.get_parameter("max_vel_rad_s").value)
        self.min_time = self.get_parameter("min_time_s").value
        self.unwind_vel = self.get_parameter("unwind_vel_rad_s").value
        self.unwind_step = self.get_parameter("unwind_step_rad").value

        # REAL ROBOT SAFETY MODE. Sim defaults are unchanged; this only ever
        # makes things slower and stricter, never the reverse.
        self.declare_parameter("motion_enabled", not self.real_robot)
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        # `motion_enabled` IS READ LIVE, AND UNTIL 2026-08-17 IT WAS NOT.
        #
        # The value above was snapshotted into `self.motion_enabled` at
        # construction and nothing ever re-read it. `set_parameters` therefore
        # changed the parameter server's copy and the follower carried on
        # publishing -- so every caller that "paused the follower" by lowering
        # this parameter paused nothing, and the blocker's own recovery text
        # ("ros2 param set motion_enabled true") could not work either.
        #
        # MEASURED, with both controls, on a live stack:
        #     followers ARMED   commanded +60 mm -> the arm moved 0.0600 m
        #     followers PAUSED  commanded -60 mm -> the arm moved 0.0600 m
        # The pause reported success on both followers and changed nothing.
        #
        # WHAT IT COST. `stage_presentation_pose.py` has documented this pause
        # since 2026-08-15 as its answer to "the follower wins", and
        # `vision_grasp.observe_and_detect` was given the same treatment for
        # T1's look. Neither worked. The arm settled at the edge of
        # `Vision.stage()`'s 0.02 rad tolerance in a tug of war with the
        # follower and never stopped moving, which is why T1's cubes
        # deprojected 31.7-35.7 mm off inside the sweep and 1.3-3.0 mm
        # standalone on a fresh stack -- on a fresh stack the follower has no
        # target yet, so there is nothing to fight.
        #
        # HARD CONSTRAINT 8 IS SERVED BY THIS, NOT BREACHED BY IT. The rule is
        # that real_robot mode must be armed BY HAND; the parameter is that
        # hand, and honouring it is what makes the documented arming path
        # real. Every transition is logged at WARN, in both directions, so an
        # arm that was disarmed by a script and not restored says so.
        self.add_on_set_parameters_callback(self._on_set_parameters)
        if self.real_robot:
            # Real arms move next to a person: slower, wider margins, and a
            # ramp so the first seconds after arming are gentler still.
            self.max_vel = min(self.max_vel, 0.10)      # from 0.6 rad/s
            self.min_clear = max(self.min_clear, 0.12)  # from 0.05 m
            self.max_step = min(self.max_step, 0.10)    # accel cap << 8.6
            self.ramp_s = 5.0
            self.ramp_frac = 0.25
            self.get_logger().warn(
                "[REAL ROBOT] max_vel capped to %.2f rad/s, clearance floor "
                "raised to %.2f m, motion DISABLED until "
                "`ros2 param set /%s motion_enabled true`."
                % (self.max_vel, self.min_clear, self.get_name()))

        # CASCADE RATE LIMIT. When the sim->real bridge is live the real arm
        # replays sim angles at cascade_vel_rad_s. If the SIM were allowed to
        # move faster the real arm would fall progressively further behind and
        # the "1.0 s preview" would stop meaning anything. So the sim gets the
        # SAME cap, and only while the cascade is active -- applying it always
        # made the sim visually frozen during ordinary sim-only teleop.
        self.declare_parameter("cascade_vel_rad_s", 0.15)
        # The rate this follower publishes at, used ONLY to convert the
        # cascade velocity into a per-cycle step. Declared rather than
        # measured so the cap is deterministic at startup.
        self.declare_parameter("cascade_rate_hz", 50.0)
        self.cascade_vel = float(self.get_parameter("cascade_vel_rad_s").value)
        self.cascade_rate = float(self.get_parameter("cascade_rate_hz").value)
        self.sim_max_step = self.max_step
        self.sim_max_vel = self.max_vel          # remembered, restored on exit
        self.cascade_active = False
        # WHICH ARM THE CLEARANCE FIGURE IS ABOUT: "sim", "real" or "none".
        # Published in /ik_status so it cannot be lost between here and a
        # reader, and logged on every change.
        self._clear_src = "sim"
        self._clear_src_said = None
        # Per-arm: a follower must only listen to ITS OWN bridge.
        self.create_subscription(Bool, "/cascade_active_%s" % self.arm,
                                 self.on_cascade, 10)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("Waiting for /compute_ik service...")
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("  still waiting...")
        self.get_logger().info("Connected to /compute_ik.")

        self.pub = self.create_publisher(
            JointTrajectory, f"/{self.arm}_arm_controller/joint_trajectory", 10)
        # CUMULATIVE guard/solver counters, published rather than logged, so
        # a recorder can attribute slews and rejections to a time window
        # without scraping [STATS]/[GUARD] text.
        # [success, fail, direct, slewed, rejected]
        self.status_pub = self.create_publisher(
            Float64MultiArray, f"/ik_status_{self.arm}", 10)
        self.tot = dict(success=0, fail=0, direct=0, slewed=0, rejected=0)
        # Clearance monitoring: the arm is bolted to a person's back, so the
        # actual measured distance to the wearer is an interlock, not a
        # diagnostic. Sourced from TF, i.e. where the robot REALLY is.
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.clearance_model = ClearanceModel()
        self.min_clearance = float("inf")
        self.clearance_blocks = 0
        self.redundancy_used = 0
        self.redundancy_try = 0
        self.pending_target_msg = None
        self.ramp_s = getattr(self, "ramp_s", 0.0)
        self.ramp_frac = getattr(self, "ramp_frac", 1.0)
        self.enabled_at = None
        self.estopped = False
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: setattr(self, "estopped", m.data), 10)
        # STARTUP GATE (real hardware only). real_homing_node publishes the
        # real arm's delta from the sim home on /real_status_<arm>; the last
        # two fields are max|delta| and an at_home flag. Refuse to command a
        # real arm that is not where the sim thinks it is.
        self.home_gate_ok = None
        self.home_gate_worst = None
        self.declare_parameter("home_tolerance_rad", 0.05)
        self.home_tol = float(self.get_parameter("home_tolerance_rad").value)
        self.create_subscription(Float64MultiArray, f"/real_status_{self.arm}",
                                 self.on_real_status, 10)
        self.create_timer(0.02, self.publish_status)

        self.create_subscription(
            PoseStamped, f"/master_arm_pose_{self.arm}", self.on_pose, 10)

        # AUTONOMY INPUT -- modes 4, 5 and 6.
        #
        # This subscription is the whole fix for "autonomy cannot command the
        # arm". handover_arbiter and autonomy_executive published
        # /autonomy/assist_pose_<arm> and NOTHING SUBSCRIBED TO IT, so three
        # of the four study modes computed a correct pose and dropped it.
        #
        # IT ENTERS AT request_ik, WHICH IS THE SAME DOOR TELEOP USES. on_pose
        # maps a MASTER-frame displacement through the anchor and scale and
        # then calls request_ik; an autonomy pose is already world-frame, so
        # it skips the mapping and nothing else. Everything downstream --
        # collision-aware IK, the redundancy re-seed, the clearance floor, the
        # step guard, the flip reject, the e-stop and every BlockMonitor
        # blocker -- is literally the same code. There is deliberately NO
        # second path to the controller: a private route for autonomy would be
        # a different robot wearing the same name, and it is the highest
        # autonomy mode where nobody is watching.
        self.autonomy_pose_t = 0.0
        self.declare_parameter("accept_autonomy_pose", True)
        self.declare_parameter("autonomy_pose_timeout_s", 0.5)
        self.create_subscription(
            PoseStamped, f"/autonomy/assist_pose_{self.arm}",
            self.on_autonomy_pose, 10)

        # Track the arm's CURRENT real joint state, to use as the IK
        # seed -- without this, the numerical (KDL) solver has nothing
        # to start searching from and fails with NO_IK_SOLUTION (-31)
        # even for perfectly reachable poses, as we confirmed directly.
        # Stored UNWRAPPED, exactly as /joint_states reports it: the
        # unwrapped value is what the controller will actually travel
        # from, so it is what the timing calc needs. Wrapping happens
        # at each point of use instead.
        home_radians = home_positions.load_home_radians(self.arm)
        self.nominal_home = wrap_continuous(list(home_radians))
        self.current_joint_positions = dict(zip(self.joint_names, home_radians))
        self.joint_state_seen = False
        self.get_logger().info(
            f"Loaded home position for {self.arm} arm from "
            f"~/kortex_ws/config/home_positions_{self.arm}.txt")
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10)

        # PART 1 HARDENING. Every mechanism in this node that can stop motion
        # registers here, so blocking is loud, named, escalating and visible in
        # ONE place. All four historical silent-stall bugs lived in this file.
        self.blocks = BlockMonitor(self, "ik_follower_%s" % self.arm)
        for nm, what, rec in (
            ("estop", "e-stop latched",
             "publish /estop_reset, or clear the trigger and reset"),
            ("startup_unwind", "startup unwind still running",
             "completes on its own within a few seconds"),
            ("home_gate", "real arm is not at the sim home",
             "run real_homing_node for this arm"),
            ("motion_disarmed", "real_robot mode with motion_enabled false",
             "ros2 param set motion_enabled true"),
            ("ik_inflight", "an IK call has not returned",
             "force-released after pending_timeout_s and retried"),
            ("ik_failed", "/compute_ik returned no solution",
             "redundancy re-seeds joint_3 and retries; then the next pose"),
            ("state_unknown",
             "another blocker EXPIRED - the loop asserting it stopped running, "
             "so its condition is unknown and motion is refused",
             "the asserting unit must run again and either re-assert or clear "
             "the expired blocker; check /blocking_summary for which unit"),
            ("autonomy_has_control",
             "an autonomy mode is commanding this arm; master input ignored",
             "stops on its own once /autonomy/assist_pose_<arm> goes quiet "
             "for autonomy_pose_timeout_s, or set accept_autonomy_pose false"),
            ("clearance_floor", "arm is inside the clearance floor of the wearer",
             "move the commanded pose away; the arm holds, it does not retreat"),
            ("flip_reject", "solution rejected as a redundancy flip",
             "force-accepted after flip_reject_limit consecutive rejections"),
            ("no_joint_state", "no /joint_states yet, cannot seed IK",
             "arrives when the controllers come up"),
        ):
            self.blocks.register(nm, what, rec)

        self.pending = False  # simple guard: don't pile up overlapping calls
        self.pending_since = self.get_clock().now()
        # How long an in-flight /compute_ik call may block new ones before
        # the guard is force-released. Generous vs the 0.2 s solver timeout.
        self.declare_parameter("pending_timeout_s", 1.0)
        self.pending_timeout = float(self.get_parameter("pending_timeout_s").value)
        self.success_count = 0
        self.fail_count = 0
        self.reject_count = 0
        self.slew_count = 0
        self.direct_count = 0
        # Target pose of the in-flight request, and of the last one acted on,
        # so a redundancy flip (solution jumps while the target stands still)
        # can be told apart from a legitimate large move.
        self.pending_target = None
        self.last_target_acted = None
        self.last_solution = None
        self.consec_flip_rejects = 0
        self.create_timer(5.0, self.log_stats)

        # Tracking stays off until the startup unwind has finished (or
        # has confirmed there is nothing to unwind).
        self.tracking_enabled = False
        self.unwind_timer = None
        self.startup_timer = self.create_timer(0.5, self.startup_check)

        self.get_logger().info(
            "ik_follower_node up. Checking for wind-up before tracking.")

    # ---------------- state ----------------

    def on_cascade(self, m):
        """Bridge is live: hold the SIM to the same cap as the real arm.

        Without this the sim runs at sim speed, the real arm replays at
        cascade_vel, and the gap between them grows without bound -- the
        preview delay would drift instead of staying at its configured value,
        and the lag monitor would trip on the operator rather than on a fault.
        """
        new = bool(m.data)
        if new == self.cascade_active:
            return
        self.cascade_active = new
        self.max_vel = self.cascade_vel if new else self.sim_max_vel
        # AND THE PER-CYCLE STEP. max_vel alone does NOT rate-limit this
        # follower: it only stretches `time_from_start`, i.e. how long the
        # controller is TOLD to take. What bounds how far the joints
        # actually move is `max_step_rad` in clamp_towards(), and at the sim
        # default of 0.35 rad per cycle at 50 Hz that is 17.5 rad/s -- 117x
        # the 0.15 rad/s the cascade was capping. So the sim outran the real
        # arm no matter what max_vel said.
        #
        # This is why raising lag_trip_rad 0.15 -> 0.5 (3.3x) let the
        # observed divergence grow 0.155 -> 0.890 rad (5.7x) instead of
        # holding roughly constant: nothing was limiting the sim's
        # joint-space rate, so the threshold only chose when to notice.
        self.max_step = (min(self.sim_max_step,
                             self.cascade_vel / max(self.cascade_rate, 1e-6))
                         if new else self.sim_max_step)
        self.get_logger().warn(
            "CASCADE %s -- sim max_vel now %.3f rad/s, max_step now "
            "%.4f rad/cycle (= %.3f rad/s at %.0f Hz)"
            % ("ACTIVE" if new else "OFF", self.max_vel, self.max_step,
               self.max_step * self.cascade_rate, self.cascade_rate))

    def current_raw(self):
        """Current joint positions, unwrapped, in joint_names order."""
        return [self.current_joint_positions[n] for n in self.joint_names]

    def on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = pos
                self.joint_state_seen = True

    def publish_status(self):
        # The e-stop blocker is maintained HERE, on a timer, not only in
        # on_pose.
        #
        # It used to be set and cleared exclusively inside on_pose, which made
        # it invisible in the one situation that matters most. The commonest
        # reason for the e-stop to latch is the dead-man, and the dead-man
        # fires precisely BECAUSE the master went silent -- so on_pose stops
        # being called at the same moment, and the blocker announcing "the
        # arm is stopped because the e-stop is latched" was never published.
        # From the aggregated view the follower simply went quiet, which is
        # indistinguishable from a crashed node.
        if self.estopped:
            self.blocks.block("estop", "latched")
        else:
            self.blocks.clear("estop")
        m = Float64MultiArray()
        m.data = ([float(self.tot[k]) for k in
                   ("success", "fail", "direct", "slewed", "rejected")]
                  + [float(self.min_clearance if math.isfinite(self.min_clearance)
                           else -1.0),
                     float(self.clearance_blocks),
                     float(self.redundancy_used),
                     # [8] EFFECTIVE max_vel and [9] cascade state. The
                     # declared parameter stays at its sim value, so
                     # `ros2 param get max_vel_rad_s` reports 0.6 whether or
                     # not the cascade limit is applied -- which makes the one
                     # thing worth checking unobservable. Published here so it
                     # can be MEASURED rather than inferred from a log line.
                     float(self.max_vel),
                     1.0 if self.cascade_active else 0.0,
                     # [10] EFFECTIVE max_step. max_vel alone never told the
                     # truth about how fast this follower moves.
                     float(self.max_step),
                     # [11] WHICH ARM FIELD [5] IS ABOUT: 0 sim, 1 real,
                     # -1 unknown. A clearance number without its source is
                     # exactly the defect this field exists to close -- under
                     # the cascade the sim leads the real arm by the bridge
                     # delay, so "sim" and "real" are different questions and
                     # a reader must not have to guess which was answered.
                     {"sim": 0.0, "real": 1.0}.get(self._clear_src, -1.0)])
        self.status_pub.publish(m)

    def log_stats(self):
        total = self.success_count + self.fail_count
        if total == 0 and self.reject_count == 0:
            return
        pct = 100.0 * self.success_count / total if total else 0.0
        self.get_logger().info(
            f"[STATS] {self.success_count}/{total} IK solves succeeded "
            f"({pct:.0f}%) in the last 5s window.")
        # Reported separately from [STATS], and slewed separately from
        # rejected. Conflating them made a working solver look like a
        # failing one: every solution was counted as a rejection while IK
        # itself was succeeding 100% of the time.
        commanded = self.direct_count + self.slew_count
        self.get_logger().info(
            f"[GUARD] published {commanded} "
            f"(direct {self.direct_count} / slewed {self.slew_count}) / "
            f"rejected {self.reject_count} as redundancy flips "
            f"(max_step {self.max_step:.2f} rad).")
        self.success_count = 0
        self.fail_count = 0
        self.reject_count = 0
        self.slew_count = 0
        self.direct_count = 0

    # ---------------- live parameters ----------------

    def _on_set_parameters(self, params):
        """Honour a live change to `motion_enabled`. See the note where the
        callback is registered for what a snapshot cost.

        ONLY `motion_enabled` is applied. Every other parameter in this node
        is read once at construction, some of them into derived values that a
        late change could not reach consistently -- `max_vel` is clamped by
        real_robot mode one line after it is read, and re-applying the raw
        value would silently discard that clamp, which is a bug this file has
        already had once. Anything else is reported as ignored rather than
        accepted and dropped.
        """
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name != "motion_enabled":
                self.get_logger().warn(
                    "parameter %r is read at startup only; setting it now has "
                    "NO EFFECT on this node. Restart it to change %s."
                    % (p.name, p.name))
                continue
            want = bool(p.value)
            if want != self.motion_enabled:
                self.get_logger().warn(
                    "[MOTION] %s -> %s"
                    % ("ENABLED" if self.motion_enabled else "DISABLED",
                       "ENABLED" if want else "DISABLED"))
            self.motion_enabled = want
        return SetParametersResult(successful=True)

    # ---------------- startup unwind ----------------

    def startup_check(self):
        """Runs until /joint_states has been heard, then unwinds if needed."""
        if not self.joint_state_seen:
            self.get_logger().info(
                "Waiting for /joint_states before startup wind-up check...",
                throttle_duration_sec=5.0)
            return

        self.startup_timer.cancel()
        self.startup_timer = None

        current = self.current_raw()
        target = wrap_continuous(current)
        wound = [
            (self.joint_names[i], current[i], target[i])
            for i in CONTINUOUS_IDX
            if abs(current[i] - target[i]) > 1e-6
        ]

        if not wound:
            self.get_logger().info(
                "No wind-up detected -- all continuous joints already "
                "inside +/-pi. Tracking enabled.")
            self.tracking_enabled = True
            return

        if self.real_robot:
            # On real hardware an unwind is a large unattended motion. Refuse
            # to start rather than execute it with a person wearing the rig.
            self.get_logger().error(
                "[REAL ROBOT] REFUSING TO START: %s is wound up beyond +/-pi "
                "(%s). Unwind it in sim or by hand first."
                % (self.arm, ", ".join("%s=%.2f" % (n, c) for n, c, _ in wound)))
            self.tracking_enabled = False
            return

        for name, cur, tgt in wound:
            turns = (cur - tgt) / TWO_PI
            self.get_logger().warn(
                f"[UNWIND] {name} is at {cur:.2f} rad ({turns:+.2f} turns "
                f"out) -- unwinding to {tgt:.2f} rad.")

        traj, total_time = self.build_unwind_trajectory(current, target)
        self.pub.publish(traj)
        self.get_logger().warn(
            f"[UNWIND] {len(traj.points)} waypoints over {total_time:.1f}s at "
            f"{self.unwind_vel} rad/s. Tracking starts when it completes.")

        # Re-enable tracking once the move has had time to finish, plus a
        # small settle margin.
        self.unwind_timer = self.create_timer(
            total_time + 1.0, self.finish_unwind)

    def build_unwind_trajectory(self, current, target):
        """Walk from current to target in small steps so the controller
        gets a path it can follow, not a single enormous setpoint."""
        deltas = [t - c for c, t in zip(current, target)]
        max_delta = max(abs(d) for d in deltas)
        n_points = max(1, int(math.ceil(max_delta / self.unwind_step)))
        per_step_time = max(
            (max_delta / n_points) / self.unwind_vel, self.min_time)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        elapsed = 0.0
        for k in range(1, n_points + 1):
            frac = float(k) / n_points
            pt = JointTrajectoryPoint()
            pt.positions = [c + frac * d for c, d in zip(current, deltas)]
            elapsed += per_step_time
            pt.time_from_start = _duration(elapsed)
            traj.points.append(pt)
        return traj, elapsed

    def finish_unwind(self):
        self.unwind_timer.cancel()
        self.unwind_timer = None
        settled = self.current_raw()
        still_out = [
            f"{self.joint_names[i]}={settled[i]:.2f}"
            for i in CONTINUOUS_IDX
            if abs(settled[i]) > math.pi + 0.05
        ]
        if still_out:
            self.get_logger().warn(
                f"[UNWIND] finished but still outside +/-pi: "
                f"{', '.join(still_out)}. Enabling tracking anyway; the "
                f"wrap guards below will hold it from here.")
        else:
            self.get_logger().info(
                "[UNWIND] complete -- all continuous joints inside +/-pi. "
                "Tracking enabled.")
        self.tracking_enabled = True

    # ---------------- tracking ----------------

    def on_real_status(self, m):
        """[7 ros rad][7 kortex deg][7 delta rad][max|delta|][at_home]"""
        if len(m.data) < 23:
            return
        deltas = list(m.data[14:21])
        self.home_gate_worst = (
            max(range(7), key=lambda i: abs(deltas[i])), deltas)
        self.home_gate_ok = bool(m.data[22])

    def _clearance_from(self, prefix):
        """Distal-link points relative to the wearer, from ONE frame tree."""
        pts = {}
        # Every part the model knows, not a hardcoded three. The wearer's own
        # arms were added to ClearanceModel.PARTS and a fixed triple here would
        # have kept them out of the live check while the model claimed them.
        for part in self.clearance_model.PARTS:
            got = []
            for link in DISTAL_LINKS:
                try:
                    t = self.tf_buf.lookup_transform(
                        prefix + part, f"{prefix}{self.arm}_{link}",
                        rclpy.time.Time()).transform.translation
                    got.append((t.x, t.y, t.z))
                except Exception:
                    continue
            if got:
                pts[part] = got
        return pts

    def measure_clearance(self):
        """Smallest distance from the arm's distal links to the wearer.

        Returns (distance, part, SOURCE) and the source is the point of this
        function's existence.

        IT USED TO MEASURE THE WRONG ARM AND SAY NOTHING. The lookup was
        `f"{self.arm}_{link}"` -- unprefixed, which is the SIM tree -- while
        the real arm publishes under `real_*` (mock_real.launch.py and
        real_arms.launch.py both run robot_state_publisher with
        frame_prefix="real_"). With the cascade running, the sim leads the real
        arm by the bridge delay, 1.0 s by default. So every clearance figure
        reported during a real run described a pose the arm had not reached
        yet, under a name that reads as a measurement of the arm. A number that
        is silently about something else is worse than no number: this one
        would have been quoted as "0.12 m from the wearer" while the arm was
        somewhere else entirely.
        
        WHEN THE CASCADE IS ACTIVE THE REAL TREE IS THE ONLY ANSWER, and if it
        is not there the answer is UNKNOWN -- never a quiet fall back to the
        sim tree, which is the bug wearing a different hat. Unknown still does
        not block, for the reason it never did: an interlock that fires on
        missing data makes the arm undriveable every time TF hiccups. It is
        loud instead, and the source travels with the number all the way into
        /ik_status so a consumer cannot lose it either.
        """
        want_real = bool(self.cascade_active)
        pts = self._clearance_from("real_" if want_real else "")
        if not pts:
            if want_real:
                self._clear_src = "none"
                if self._clear_src_said != "none":
                    self.get_logger().error(
                        "[CLEARANCE] cascade is ACTIVE and no real_* TF is "
                        "available -- clearance is UNKNOWN for the REAL arm. "
                        "Not falling back to the sim tree: that number would "
                        "describe a pose %.1f s ahead of the arm."
                        % 1.0)
                    self._clear_src_said = "none"
            else:
                self._clear_src = "none"
            return float("inf"), None
        self._clear_src = "real" if want_real else "sim"
        if self._clear_src != self._clear_src_said:
            self.get_logger().info(
                "[CLEARANCE] measuring the %s arm (%s frames)"
                % (self._clear_src.upper(),
                   "real_*" if want_real else "unprefixed"))
            self._clear_src_said = self._clear_src
        pad = REAL_ROBOT_PAD_M if self.real_robot else 0.0
        return self.clearance_model.clearance(pts, pad)

    def on_pose(self, msg: PoseStamped):
        # ONE SOURCE AT A TIME. If autonomy is commanding this arm, the master
        # does not also get to: interleaving two sources on one arm is the
        # two-publishers-on-one-topic bug wearing a different hat, and on a
        # rig bolted to a person it would be worse than either source alone.
        if self.autonomy_is_driving():
            self.blocks.block("autonomy_has_control")
            return
        self.blocks.clear("autonomy_has_control")
        if not self.tracking_enabled:
            self.blocks.block("startup_unwind")
            return  # startup unwind still in progress
        self.blocks.clear("startup_unwind")
        if self.estopped:
            # Blocker state itself is maintained by publish_status on a timer,
            # so it is still reported when the master has stopped publishing
            # and this callback is no longer running at all.
            return
        if self.real_robot and self.home_gate_ok is False:
            i, d = self.home_gate_worst
            out = ", ".join("joint_%d %+.1f deg" % (k + 1, math.degrees(d[k]))
                            for k in range(7) if abs(d[k]) > self.home_tol)
            self.get_logger().error(
                "[HOME GATE] real %s arm is NOT at the sim home -- %s. "
                "Run the homing routine (ros2 run srl_teleop real_homing_node "
                "--ros-args -p arm:=%s) before enabling motion."
                % (self.arm, out, self.arm), throttle_duration_sec=10.0)
            self.blocks.block("home_gate", out)
            return
        self.blocks.clear("home_gate")
        if not self.motion_enabled:
            self.blocks.block("motion_disarmed", "motion_enabled is false")
            return
        self.blocks.clear("motion_disarmed")
        if self.pending:
            # WATCHDOG. `pending` is cleared in the response callback, so a
            # call that never returns -- /compute_ik restarted, move_group
            # disturbed -- would leave it True forever and every later pose
            # would be skipped in silence. Observed exactly that: the node
            # alive, poses arriving at 50 Hz, and zero IK calls. Time it out
            # and carry on rather than stalling permanently.
            waited = (self.get_clock().now() - self.pending_since).nanoseconds / 1e9
            if waited < self.pending_timeout:
                self.blocks.block("ik_inflight", "waiting %.2f s" % waited)
                return
            self.fail_count += 1
            self.tot["fail"] += 1
            self.get_logger().warn(
                f"IK call did not return within {waited:.1f}s -- releasing the "
                f"in-flight guard and retrying. (/compute_ik may have "
                f"restarted.)", throttle_duration_sec=5.0)
        self.blocks.clear("ik_inflight")
        self.redundancy_try = 0
        self.request_ik(msg)

    def on_autonomy_pose(self, msg: PoseStamped):
        """A WORLD-frame target from the autonomy stack.

        Applies exactly the gates on_pose applies before handing to the solver.
        They are repeated rather than shared because on_pose's remaining body
        is the master-frame mapping, which must NOT run for a world-frame
        target -- routing an autonomy pose through it would scale a world
        coordinate as if it were a hand displacement.
        """
        if not bool(self.get_parameter("accept_autonomy_pose").value):
            return
        if not self.tracking_enabled:
            self.blocks.block("startup_unwind")
            return
        self.blocks.clear("startup_unwind")
        if self.estopped:
            return                      # blocker maintained by publish_status
        if self.real_robot and self.home_gate_ok is False:
            return
        now = time.monotonic()
        if self.autonomy_pose_t == 0.0:
            self.get_logger().info(
                "[SOURCE:%s] AUTONOMY is now driving this arm via "
                "/autonomy/assist_pose_%s. Same IK, same clearance floor, "
                "same guard, same e-stop as teleop."
                % (self.arm, self.arm))
        self.autonomy_pose_t = now
        self.request_ik(msg)

    def autonomy_is_driving(self):
        """True while a fresh autonomy pose is arriving.

        Used to suppress the master input rather than let two sources
        interleave on one arm -- the same failure as two publishers on one
        topic, which has already cost this project a measurement.
        """
        if self.autonomy_pose_t == 0.0:
            return False
        t = float(self.get_parameter("autonomy_pose_timeout_s").value)
        return (time.monotonic() - self.autonomy_pose_t) < t

    def request_ik(self, msg, seed_perturb=0):
        """Fire one /compute_ik call.

        seed_perturb > 0 offsets the REDUNDANT joints, exploiting the Gen3's
        7th DOF: the elbow can swing through a null space without moving the
        gripper. TRAC-IK with solve_type Distance returns the solution
        nearest its seed, so moving the seed is how you ask for a different
        elbow configuration for the same commanded pose. Offsets alternate
        sign and grow, so the search fans out either side of the natural
        posture instead of drifting one way.
        """
        self.pending = True
        self.pending_since = self.get_clock().now()
        p, o = msg.pose.position, msg.pose.orientation
        self.pending_target = ((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        self.pending_target_msg = msg

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = f"{self.arm}_arm"
        req.ik_request.pose_stamped = msg
        req.ik_request.ik_link_name = f"{self.arm}_end_effector_link"
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        # The wearer is in the collision model (human_backpack.xacro now
        # carries <collision>, and the SRDF only excludes the mount-adjacent
        # proximal links), so this is what keeps the arm off the operator.
        req.ik_request.avoid_collisions = True

        seed = wrap_continuous(self.current_raw())
        # Bias the SEED toward nominal home. Only the seed: the solution
        # still has to hit the commanded pose exactly, so this steers which
        # of the infinitely many 7-DOF solutions is chosen, and cannot move
        # the gripper away from where the operator asked.
        w = float(self.get_parameter("posture_bias_w").value)
        if w > 0.0 and self.nominal_home is not None:
            seed = [c + w * ang_diff(h, c) if i in CONTINUOUS_IDX
                    else c + w * (h - c)
                    for i, (c, h) in enumerate(zip(seed, self.nominal_home))]
        if seed_perturb:
            k = (seed_perturb + 1) // 2
            sign = 1.0 if seed_perturb % 2 else -1.0
            step = sign * self.redundancy_span * k / max(1, self.redundancy_n // 2)
            for i in REDUNDANT_IDX:
                seed[i] = wrap_pi(seed[i] + step)

        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = seed

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.on_ik_response)

    def on_ik_response(self, future):
        self.pending = False
        # Clear the in-flight blocker HERE, not in on_pose. on_pose only
        # reaches its clear() when it finds pending already False, so on a
        # healthy system that line is never executed and the blocker latched
        # "active" forever after its first firing - a false alarm produced by
        # the alarm itself. Caught by fault injection: 65 s held, reason
        # "waiting 0.00 s".
        self.blocks.clear("ik_inflight")
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f"IK call failed: {e}")
            return

        if resp.error_code.val != 1:  # 1 == moveit_msgs.msg.MoveItErrorCodes.SUCCESS
            # REDUNDANCY RETRY. With avoid_collisions on, a failure often
            # means "this elbow configuration collides", not "this pose is
            # unreachable". The Gen3 has one redundant DOF, so re-seed it and
            # ask again -- the gripper can hold its pose while the elbow
            # swings clear of the wearer.
            if (self.pending_target is not None
                    and self.redundancy_try < self.redundancy_n):
                self.redundancy_try += 1
                self.redundancy_used += 1
                self.request_ik(self.pending_target_msg,
                                seed_perturb=self.redundancy_try)
                return
            self.redundancy_try = 0
            self.fail_count += 1; self.tot["fail"] += 1
            self.blocks.block("ik_failed", "error code %d after %d redundancy "
                              "retries" % (resp.error_code.val, self.redundancy_n))
            return

        self.success_count += 1; self.tot["success"] += 1

        name_to_pos = dict(zip(resp.solution.joint_state.name,
                                resp.solution.joint_state.position))
        try:
            positions = [name_to_pos[n] for n in self.joint_names]
        except KeyError:
            self.get_logger().warn("IK solution missing expected joint names.")
            return

        positions = wrap_continuous(positions)
        self.blocks.clear("ik_failed")

        # Step guard, measured against the WRAPPED current state. For the
        # continuous joints the comparison is the shortest angular
        # distance, so a legitimate small move across +/-pi is not
        # mistaken for a 2pi leap.
        current = self.current_raw()
        current_wrapped = wrap_continuous(current)
        steps = joint_steps(positions, current_wrapped)
        worst = max(range(len(steps)), key=lambda i: abs(steps[i]))

        target = self.pending_target
        d_m, d_rad = pose_delta(target, self.last_target_acted)
        target_moved = d_m > self.pose_db_m or d_rad > self.pose_db_rad

        if abs(steps[worst]) <= self.max_step:
            commanded = positions
            self.direct_count += 1; self.tot["direct"] += 1
            self.consec_flip_rejects = 0
        else:
            # Far from the current state. Two very different causes:
            #   (a) the target genuinely moved, or the arm simply has ground
            #       to cover -- walk toward it, bounded, and converge.
            #   (b) the target stood still but the SOLUTION jumped to another
            #       IK branch -- a redundancy flip, which solve_type Distance
            #       exists to avoid. Reject that.
            sol_jump = 0.0
            if self.last_solution is not None:
                sol_jump = max(abs(s) for s in
                               joint_steps(positions, self.last_solution))
            flip = (not target_moved
                    and self.last_solution is not None
                    and sol_jump > self.max_step)

            if flip and self.consec_flip_rejects < self.flip_limit:
                self.reject_count += 1; self.tot["rejected"] += 1
                self.consec_flip_rejects += 1
                self.get_logger().warn(
                    f"[GUARD] rejected redundancy flip: target moved "
                    f"{d_m*1000:.1f} mm / {math.degrees(d_rad):.1f} deg but "
                    f"the solution jumped {sol_jump:+.3f} rad "
                    f"({self.joint_names[worst]}).",
                    throttle_duration_sec=1.0)
                self.last_solution = positions
                self.blocks.block("flip_reject",
                                  "%d consecutive; force-accept at %d"
                                  % (self.consec_flip_rejects, self.flip_limit))
                return
            self.blocks.clear("flip_reject")

            if flip:
                self.get_logger().warn(
                    f"[GUARD] accepting a flip after {self.consec_flip_rejects} "
                    f"consecutive rejections -- the solver is not coming back. "
                    f"Slewing to it; expect a slow re-orientation.")
            commanded = clamp_towards(positions, current_wrapped, self.max_step)
            self.slew_count += 1; self.tot["slewed"] += 1
            self.consec_flip_rejects = 0
            self.get_logger().info(
                f"[GUARD] slewing: {self.joint_names[worst]} wants "
                f"{steps[worst]:+.3f} rad, committing "
                f"{max(-self.max_step, min(self.max_step, steps[worst])):+.3f}.",
                throttle_duration_sec=2.0)

        self.last_solution = positions
        self.last_target_acted = target

        # Timing from the ACTUAL COMMANDED distance, measured from the
        # controller's true unwrapped position -- a clamped step must still
        # execute at bounded speed, not be crammed into the old full-travel
        # time.
        travel = max(abs(t - c) for t, c in zip(commanded, current))
        # RAMP: for the first ramp_s after arming, run at ramp_frac of the
        # velocity cap. time_from_start is STRETCHED (not clamped), so the
        # trajectory is genuinely slower rather than merely truncated.
        vel = self.max_vel
        if self.ramp_s > 0.0:
            if self.enabled_at is None:
                self.enabled_at = self.get_clock().now()
            el = (self.get_clock().now() - self.enabled_at).nanoseconds / 1e9
            if el < self.ramp_s:
                vel = self.max_vel * self.ramp_frac
        time_from_start = max(travel / vel, self.min_time)
        positions = commanded

        # HARD FLOOR. Never let a command through while the arm is already
        # inside the safety margin of the wearer. Holding position is always
        # a safe action; publishing is not.
        clear, part = self.measure_clearance()
        self.min_clearance = clear
        if clear < self.min_clear:
            self.clearance_blocks += 1
            self.blocks.block("clearance_floor",
                              "%.1f cm from %s, floor %.0f cm"
                              % (clear * 100, part, self.min_clear * 100))
            return
        self.blocks.clear("clearance_floor")

        # AN EXPIRED BLOCKER IS NOT PERMISSION TO MOVE.
        #
        # BlockMonitor auto-expires a blocker nobody has re-asserted for
        # EXPIRE_S, which is what stops the unreachable-clear() pattern
        # latching a block forever. But an expiry means the loop that was
        # asserting it STOPPED RUNNING, so the condition it guarded is
        # UNKNOWN -- not gone. Reading that as an all-clear would turn a
        # crashed guard into a green light, which is a worse failure than the
        # latch the expiry was introduced to fix.
        #
        # Placed HERE, at the publish, and deliberately not at the top of the
        # callback: every clear() above is what resolves an expiry, so
        # refusing at entry would prevent the re-assertion that clears the
        # unknown and would deadlock exactly as the original latch did.
        # `state_unknown` is itself a blocker and can expire like any other, so
        # it is excluded from its own trigger -- otherwise it re-arms itself
        # forever, which is the same latch in a new costume.
        unknown = [n for n in self.blocks.expired_names if n != "state_unknown"]
        if unknown:
            self.blocks.block("state_unknown", "expired: %s" % ",".join(unknown))
            return
        self.blocks.clear("state_unknown")

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = _duration(time_from_start)
        traj.points = [pt]
        self.pub.publish(traj)
        self.redundancy_try = 0


def main(args=None):
    rclpy.init(args=args)
    node = IKFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
