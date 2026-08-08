#!/usr/bin/env python3
"""
master_pose_node.py — computes the master (left) arm's end-effector pose
=============================================================================
POSITION: from Forward Kinematics using the 7 pot joint angles (k1j1-k1j7),
  same math/link-lengths as the verified kinematics.cpp, ported to Python.

ORIENTATION:
  Roll & Pitch  <- from the wrist IMU's accelerometer (gravity vector).
                   Physically solid, drift-free -- an accelerometer can
                   ALWAYS tell you which way is down.
  Yaw           <- from k1j7 (the wrist roll pot) instead of the IMU.
                   An accelerometer CANNOT measure yaw (rotation about the
                   vertical axis) -- gravity looks the same regardless of
                   heading. The IMU has no magnetometer, so gyro-only yaw
                   would drift over time. Using the pot instead gives a
                   drift-free yaw at zero extra cost, since it's already
                   accurately measured.

GRACEFUL DEGRADATION (three stages, in this order, per frame):
  1. VALIDATION -- several pot channels are known bad or intermittent. The
     Teensy firmware clamps sub-ADC_MIN readings to exactly 0.0, so a
     dropout is indistinguishable from a real bottom-of-travel reading.
     A frame that fails validation reuses the last good joint vector; it
     is never allowed to reach the FK. Dead channels are NEVER modelled,
     interpolated or filled -- the frame is rejected instead.
  2. SMOOTHING -- exponential moving average on the FK tip POSITION only
     (never orientation), applied before the workspace mapping.
  3. CLUTCH -- physical button toggles tracking off, so the operator can
     recentre their arm without the robot following. Re-engaging captures
     a fresh reference so the robot does not jump.

Publishes the combined pose to /master_arm_pose (PoseStamped), ready to
feed into MoveIt Servo for real-time, collision-aware execution -- see
master_pose_node's docstring continuation for the Servo wiring (next
stage, once this stage's numbers are verified sane).

Run (test with the fake serial simulator or the real Teensy):
  ros2 run srl_teleop master_pose_node --ros-args -p serial_port:=/dev/ttyACM1
"""
import os
import re
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from srl_teleop import master_calibration as mc
from srl_teleop.serial_port import find_port, claim_exclusive
from srl_teleop import degraded_mode as dg
import tf_transformations  # quaternion helpers (roll,pitch,yaw -> quaternion)
import re as _re

# FSR + buttons are GLOBAL values in the frame (not per-arm) -- parsed
# once per line, regardless of which arm(s) this node is handling.
FSR_BTN_PATTERN = _re.compile(
    r"fsr1:(-?\d+\.?\d*),fsr2:(-?\d+\.?\d*),btn1:(\d+),btn2:(\d+)"
)

# ---- Channel validation policy (see CLAUDE.md "Known-bad pot channels") ----
# Joints whose exact-0.0 reading means "dropout", not "real reading".
# j6 is EXEMPT: it is mounted at the bottom of its travel and legitimately
# clamps to 0. j7 is EXEMPT: it is railed at 360 and is a terminal roll
# that moves the tip 0.000000 m, so a bad value there cannot corrupt
# position. j1-j5 are all load-bearing for tip position.
ZERO_CHECK_IDX = (0, 1, 2, 3, 4)  # j1..j5 -- position_mode "fk"

# position_mode "spherical" uses only j1 (azimuth) and the j2/j4 bends
# (reach magnitude). Direction comes from the wrist IMU instead of the
# distal joints, so j3, j5, j6 and j7 are no longer load-bearing and a
# dropout on any of them must NOT reject the frame. j3 still feeds the
# reach magnitude when it is alive, but magnitude barely depends on it, so
# a dead j3 is passed through as 0 rather than rejected or invented.
SPHERICAL_CHECK_IDX = (0, 1, 3)   # j1, j2, j4

# Motion scaling, chosen from MEASURED IK reachability over the recorded
# master trajectory (not from geometry): at 1.0 the commanded span is
# ~0.44 m with 92% (left) / 95% (right) of poses reachable, orientation
# fixed. 1:1 is also the most predictable mapping for the operator.
# Re-read every frame, so `ros2 param set` retunes live without a jump.
SCALE_DEFAULT = {"left": 1.0, "right": 1.0}
# 0-based index of the roll joint supplying azimuth, and of j3.
J1_IDX, J3_IDX = 0, 2


def parse_fsr_buttons(line):
    m = FSR_BTN_PATTERN.search(line)
    if not m:
        return None
    fsr1, fsr2, btn1, btn2 = m.groups()
    return float(fsr1), float(fsr2), int(btn1), int(btn2)


def zero_dropouts(joints_deg, idx=ZERO_CHECK_IDX):
    """Indices of load-bearing channels reading a firmware-clamped 0.0.

    Which channels count as load-bearing depends on position_mode -- see
    ZERO_CHECK_IDX / SPHERICAL_CHECK_IDX.
    """
    return [i for i in idx if joints_deg[i] == 0.0]


def circ_diff_deg(a, b):
    """Shortest signed a-b for pot degrees, in (-180, 180]. The pots wrap
    at 0/360, so a plain subtraction reads 359->1 as a 358 deg glitch
    instead of the 2 deg move it really is."""
    return ((a - b + 180.0) % 360.0) - 180.0

# ============================================================================
# Link lengths (meters), shaft-center to shaft-center -- SAME verified
# values as kinematics.cpp. J1 roll, J2 bend, J3 roll, J4 bend, J5 roll,
# J6 bend, J7 roll (confirmed Kinova Gen3 joint pattern).
# ============================================================================
L1, L2, L3, L4, L5, L6, L7 = 0.043, 0.037, 0.043, 0.037, 0.043, 0.036, 0.033


def mat4_identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def mat4_mult(a, b):
    r = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            r[i][j] = sum(a[i][k]*b[k][j] for k in range(4))
    return r


def rot_z(theta):
    r = mat4_identity()
    c, s = math.cos(theta), math.sin(theta)
    r[0][0], r[0][1] = c, -s
    r[1][0], r[1][1] = s, c
    return r


def rot_x(theta):
    r = mat4_identity()
    c, s = math.cos(theta), math.sin(theta)
    r[1][1], r[1][2] = c, -s
    r[2][1], r[2][2] = s, c
    return r


def translate_z(d):
    r = mat4_identity()
    r[2][3] = d
    return r


def forward_kinematics_position(q):
    """q = [q1..q7] in radians. Returns (x, y, z) of the master hand,
    relative to the base (J1) mount frame. Same math as kinematics.cpp,
    verified there against hand-computed test cases.

    NOTE: no longer on the live path -- the node calls mc.fk() so that the
    FK, the axis map and the workspace mapping all come from one module.
    Kept because it is the rot_x-bend reference implementation, and the
    two conventions differ by a 90 deg rotation about z."""
    T = mat4_identity()
    T = mat4_mult(T, rot_z(q[0])); T = mat4_mult(T, translate_z(L1))
    T = mat4_mult(T, rot_x(q[1])); T = mat4_mult(T, translate_z(L2))
    T = mat4_mult(T, rot_z(q[2])); T = mat4_mult(T, translate_z(L3))
    T = mat4_mult(T, rot_x(q[3])); T = mat4_mult(T, translate_z(L4))
    T = mat4_mult(T, rot_z(q[4])); T = mat4_mult(T, translate_z(L5))
    T = mat4_mult(T, rot_x(q[5])); T = mat4_mult(T, translate_z(L6))
    T = mat4_mult(T, rot_z(q[6])); T = mat4_mult(T, translate_z(L7))
    return T[0][3], T[1][3], T[2][3]


def roll_pitch_from_accel(ax, ay, az):
    """Standard gravity-vector tilt calculation. Drift-free, but cannot
    give yaw (see module docstring)."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    return roll, pitch


# ---- Serial frame parsing, parameterized by which arm we're reading ----
# "left" reads k1jN + K1IMU (master's left side); "right" reads k2jN + K2IMU.
def build_parsers(arm):
    prefix = "k1j" if arm == "left" else "k2j"
    imu_tag = "K1IMU" if arm == "left" else "K2IMU"
    joint_patterns = {i: re.compile(rf"{prefix}{i}:(-?\d+\.?\d*)") for i in range(1, 8)}
    imu_pattern = re.compile(
        rf"{imu_tag}:(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),"
        rf"(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)"
    )
    return joint_patterns, imu_pattern


def parse_arm(line, joint_patterns, imu_pattern):
    joints_deg = []
    for i in range(1, 8):
        m = joint_patterns[i].search(line)
        if not m:
            return None
        joints_deg.append(float(m.group(1)))
    imu_m = imu_pattern.search(line)
    if not imu_m:
        return None
    ax, ay, az, gx, gy, gz = (float(v) for v in imu_m.groups())
    # Gyro is returned, not discarded: it is the ONLY sensor that observes
    # rotation about the vertical. The accelerometer cannot (gravity looks
    # identical at any heading) and j1 cannot (its lever arm scales with the
    # arm's bend). Teensy sends deg/s.
    return joints_deg, (ax, ay, az), (gx, gy, gz)


class MasterPoseNode(Node):
    def __init__(self):
        super().__init__("master_pose_node")
        # "auto" sniffs for the board (it moves between ACM0/ACM1 on every
        # usbipd re-attach); an explicit path bypasses detection.
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("arm", "both")  # "left", "right", or "both"

        self.arm = self.get_parameter("arm").value
        # "both" is the correct mode for normal use -- ONE serial reader,
        # since only one process can reliably own the port at a time
        # (running two readers against the same port causes contention:
        # garbled/dropped reads for both, which shows up as exactly the
        # kind of slow, laggy response you saw).
        self.arms = ["left", "right"] if self.arm == "both" else [self.arm]

        self.parsers = {a: build_parsers(a) for a in self.arms}
        self.calib = {a: mc.PotCalibration.load(a) for a in self.arms}
        # Independent calibration state per arm -- each has its own IMU
        # mounting offset and needs its own zero-reference baseline.
        self.roll_zero = {a: None for a in self.arms}
        self.pitch_zero = {a: None for a in self.arms}
        self.yaw_zero = {a: None for a in self.arms}

        # ---- validation / smoothing / clutch parameters ----
        # VELOCITY limit, not a fixed per-frame step. 1500 deg/s is far
        # beyond human arm speed and reproduces the old 30 deg tolerance
        # at the 50 Hz frame rate. It must scale with elapsed time: a
        # fixed step deadlocks, because while a channel is dropped out
        # the arm keeps moving, so the recovery frame sits further than
        # one frame's worth from the last good one and is rejected as a
        # glitch -- and the reference never advances, so every later
        # frame is further still.
        self.declare_parameter("max_joint_velocity_deg_per_s", 1500.0)
        # Cap, so a long gap does not widen the gate to "anything goes".
        # Note |circ_diff_deg| <= 180 by construction, so a threshold at
        # 180 accepts everything -- reached after 180/1500 = 0.12 s.
        self.declare_parameter("max_joint_step_cap_deg", 180.0)
        self.declare_parameter("consecutive_reject_warn", 20)
        # Hard backstop: force-accept the next dropout-free frame after
        # this many consecutive rejections. One visible jump beats a
        # silently frozen arm.
        self.declare_parameter("force_resync_after", 25)
        # EMA on tip position. alpha=1.0 disables filtering; smaller is
        # smoother but laggier. Re-read every frame so `ros2 param set`
        # takes effect live.
        self.declare_parameter("ema_alpha", 0.3)
        # POSITION MODE.
        #   "spherical" (default) -- direction from the wrist IMU + j1, reach
        #       magnitude from the j2/j4 bends. Survives dead j3/j5/j6/j7.
        #   "fk"        -- the original full 7-DOF FK path, kept intact and
        #       switchable live with `ros2 param set` for A/B comparison.
        # The 7-joint FK collapsed directionally (93.4% of positional
        # variance on one axis, "down" landing higher than "up"), which is
        # why direction now comes from gravity instead.
        self.declare_parameter("position_mode", "spherical")
        # Gravity is only isolable when the arm is not accelerating. Outside
        # this band the elevation update is skipped and the last value held.
        self.declare_parameter("accel_gate_g", 0.15)
        # See reject_incoherent(). 0 disables the check.
        self.declare_parameter("max_channel_rate_deg_s", 800.0)
        # AZIMUTH SOURCE.
        #   "j1"   -- j1 alone (default until the gyro model is validated
        #             against a MOVING master; see CLAUDE.md)
        #   "gyro" -- integrate yaw rate about the measured vertical, reset to
        #             the j1 estimate at every clutch engage, and blend slowly
        #             back toward j1 so a long segment cannot run away.
        self.declare_parameter("azimuth_mode", "j1")
        # Time constant of the slow pull toward the j1 estimate. Long enough
        # that a fast lateral sweep is governed by the gyro (which is the
        # whole point), short enough that integration drift cannot accumulate
        # without bound across a long clutch segment.
        self.declare_parameter("azimuth_blend_tau_s", 20.0)
        self.declare_parameter("clutch_enabled", True)
        # Startup clutch state is logged explicitly below. Set true to pin
        # the clutch ENGAGED and ignore the buttons entirely -- the button
        # mapping is unconfirmed, so this rules it out in one command.
        self.declare_parameter("force_clutch_engaged", False)
        # --- clutch re-engage quality ---
        # Button bounce can double-toggle a single press.
        self.declare_parameter("clutch_debounce_s", 0.05)
        # Latch the reference only while the master is QUASI-STATIC. Latching
        # mid-motion captures a moving reference and every later command
        # inherits that offset.
        self.declare_parameter("clutch_static_m", 0.002)
        self.declare_parameter("clutch_static_window_s", 0.1)
        # Average the reference over several frames rather than trusting one.
        self.declare_parameter("clutch_ref_frames", 5)
        # WHICH PHYSICAL BUTTON DRIVES WHICH ARM IS UNVERIFIED. Recorded
        # data showed btn2 changing during the left-arm prompt and btn1
        # during the right-arm prompt, despite BUTTON1_PIN=2 being the
        # left button -- i.e. they appear to be wired backwards. These
        # defaults encode the OBSERVED behaviour, not the wiring intent.
        # Confirm with `ros2 topic echo /master_fsr_buttons` before
        # trusting them; swap with -p left_clutch_button:=1 if wrong.
        self.declare_parameter("left_clutch_button", 2)
        self.declare_parameter("right_clutch_button", 1)

        self.max_vel_deg_s = self.get_parameter("max_joint_velocity_deg_per_s").value
        self.step_cap_deg = self.get_parameter("max_joint_step_cap_deg").value
        self.consec_warn = self.get_parameter("consecutive_reject_warn").value
        self.force_resync_after = self.get_parameter("force_resync_after").value
        self.clutch_enabled = self.get_parameter("clutch_enabled").value
        self.force_clutch = self.get_parameter("force_clutch_engaged").value
        self.clutch_debounce = float(self.get_parameter("clutch_debounce_s").value)
        self.clutch_static_m = float(self.get_parameter("clutch_static_m").value)
        self.clutch_static_win = float(self.get_parameter("clutch_static_window_s").value)
        self.clutch_ref_frames = int(self.get_parameter("clutch_ref_frames").value)
        self.clutch_button = {
            a: self.get_parameter(f"{a}_clutch_button").value for a in self.arms
        }

        self.position_mode = self.get_parameter("position_mode").value
        if self.position_mode not in ("fk", "spherical"):
            raise ValueError(
                f"position_mode must be 'fk' or 'spherical', got "
                f"{self.position_mode!r}")
        # Which channels validation is allowed to reject a frame over, and
        # the master-frame rest position the workspace mapping is relative
        # to. In spherical mode rest is the arm HANGING DOWN (-z), which is
        # the posture the zeros and a_hat are captured in; in fk mode it is
        # the straight-out neutral the FK path has always used. Getting this
        # wrong offsets every commanded pose by the full arm length.
        if self.position_mode == "spherical":
            self.check_idx = SPHERICAL_CHECK_IDX
            self.neutral = mc.NEUTRAL_SPHERICAL
        else:
            self.check_idx = ZERO_CHECK_IDX
            self.neutral = mc.NEUTRAL

        # ---- DEGRADED MODE ----
        # 7 of 14 channels are INCOHERENT. An incoherent channel is not
        # noisy, it is wrong, and no filter recovers a signal that carries no
        # information about the joint. Freeze those channels at their
        # zero-reference so they cannot inject motion, and shrink the
        # per-arm validation set so a dead channel stops costing the LIVE
        # channels their frames. See degraded_mode.py.
        self.declare_parameter("degraded_mode", "auto")
        self.declare_parameter("channel_baseline", dg.default_baseline_path())
        self.declare_parameter("degraded_min_coherent", 12)
        dmode = str(self.get_parameter("degraded_mode").value)
        if dmode not in ("auto", "on", "off"):
            raise ValueError("degraded_mode must be auto|on|off, got %r"
                             % dmode)
        self.dg_baseline = dg.load_baseline(
            str(self.get_parameter("channel_baseline").value))
        self.dg_min = int(self.get_parameter("degraded_min_coherent").value)
        self.degraded, self.frozen_idx, dg_reason = dg.decide(
            self.dg_baseline, dmode, self.dg_min)
        # Per-arm validation set: a frozen channel is constant, so checking
        # it for dropouts or step violations can only produce false
        # rejections -- and a rejection freezes the WHOLE command vector.
        self.check_idx_arm = {
            a: tuple(i for i in self.check_idx
                     if i not in self.frozen_idx.get(a, []))
            for a in self.arms}
        # Freeze VALUE is the raw zero-reference reading, which
        # PotCalibration.apply() maps to exactly 0 rad. Freezing at 0.0 raw
        # would be wrong -- that is the firmware's dropout clamp and
        # zero_dropouts() reads it as a fault marker.
        self.freeze_val = {a: list(self.calib[a].zeros) for a in self.arms}
        self.frozen_counts = {a: 0 for a in self.arms}

        # ---- RADIAL FALLBACK ----
        self.declare_parameter("reach_fallback", "auto")
        self.declare_parameter("reach_fallback_rate_m_s", 0.12)
        self.declare_parameter("reach_fallback_deadband_deg", 4.0)
        self.declare_parameter("reach_min_m", 0.04)
        self.declare_parameter("reach_max_m", float(mc.FULL_EXT))
        rf_mode = str(self.get_parameter("reach_fallback").value)
        self.rf_rate = float(self.get_parameter("reach_fallback_rate_m_s").value)
        self.rf_dead = math.radians(
            float(self.get_parameter("reach_fallback_deadband_deg").value))
        self.rf_min = float(self.get_parameter("reach_min_m").value)
        self.rf_max = float(self.get_parameter("reach_max_m").value)
        self.rf_on, self.rf_ch, self.rf_why = {}, {}, {}
        self.rf_reach, self.rf_t = {}, {}
        for a in self.arms:
            on, ch, why = dg.reach_fallback_plan(
                self.dg_baseline, a, self.frozen_idx.get(a, []), rf_mode)
            self.rf_on[a], self.rf_ch[a], self.rf_why[a] = on, ch, why
            # Start mid-range so the operator can go both in and out.
            self.rf_reach[a] = 0.5 * (self.rf_min + self.rf_max)
            self.rf_t[a] = None
            # NOT `(warn if on else info)(...)`: rclpy caches severity per
            # CALL SITE, so one line emitting warn for the left arm and info
            # for the right raises "Logger severity cannot be changed between
            # calls" and kills the node. It respawned every 6.3 s, which from
            # the log looked like a serial fault rather than a logging bug.
            msg = "[REACH:%s] %s" % (a, why)
            if on:
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)
            if on:
                self.get_logger().warn(
                    "[REACH:%s] j%d deflection beyond %.1f deg commands "
                    "d(reach)/dt up to %.3f m/s, clamped to %.3f..%.3f m. "
                    "RATE not absolute: j%d spans only ~26 deg, so an "
                    "absolute map would give 26 deg of resolution over the "
                    "whole radial range and would peg reach to a wrist angle "
                    "the operator has to hold."
                    % (a, ch + 1, math.degrees(self.rf_dead), self.rf_rate,
                       self.rf_min, self.rf_max, ch + 1))
        for line in dg.banner(self.dg_baseline, self.degraded,
                              self.frozen_idx, dg_reason, self.dg_min).split("\n"):
            if self.degraded:
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)
        if self.degraded:
            for a in self.arms:
                if not self.check_idx_arm[a]:
                    self.get_logger().error(
                        "[DEGRADED:%s] EVERY channel this position_mode "
                        "validates is frozen. Position for this arm is "
                        "elevation-only and reach is constant. This is "
                        "reported, not hidden -- see the banner above." % a)
        self.accel_gate = float(self.get_parameter("accel_gate_g").value)
        self.azimuth_mode = self.get_parameter("azimuth_mode").value
        if self.azimuth_mode not in ("j1", "gyro"):
            raise ValueError("azimuth_mode must be 'j1' or 'gyro', got %r"
                             % self.azimuth_mode)
        self.azim_tau = float(self.get_parameter("azimuth_blend_tau_s").value)

        # WORKSPACE MAPPING (pipeline step 6, previously skipped -- this is
        # exactly why IK was failing every time). The master's own FK output
        # is tiny (~0.27m total reach) and centered near its own local
        # origin -- nowhere near where the real Kinova arm actually sits in
        # the world frame. We shift + scale the master's motion into a
        # region we already PROVED reachable earlier this session
        # (0.3, 0.0, 1.2) was a confirmed successful target for left_arm.
        #
        # offset = where the master's own "resting" position should land
        # scale  = how much to amplify master motion (its ~0.27m reach vs
        #          the Kinova's much larger workspace)
        # Independent, per-arm offset/scale parameters. Right arm's
        # offset defaults to a MIRRORED guess (negative X), not an
        # independently-proven reachable point like left's (0.3,0,1.2)
        # was -- will very likely need live tuning, same as left did.
        #
        # NOTE: these offsets are still the UNVERIFIED originals. The
        # measured-anchor work is unfinished (POTS_FRONT measured, P_HOME
        # not), so nothing here has been re-derived from measurement.
        for _a in ("left", "right"):
            _c = mc.WORKSPACE_CENTRE[_a]
            self.declare_parameter(f"{_a}_offset_x", _c[0])
            self.declare_parameter(f"{_a}_offset_y", _c[1])
            self.declare_parameter(f"{_a}_offset_z", _c[2])
            self.declare_parameter(f"{_a}_scale", SCALE_DEFAULT[_a])
            # Orientation half of the anchor, (x,y,z,w).
            _q = mc.WORKSPACE_ORIENT[_a]
            for _n, _v in zip(("qx", "qy", "qz", "qw"), _q):
                self.declare_parameter(f"{_a}_anchor_{_n}", _v)

        # ORIENTATION MODE.
        #   "anchored" (default) -- command anchor_quat * relative rotation,
        #       the exact mirror of how position is anchored. At rest this is
        #       the arm's own home orientation, which is reachable.
        #   "relative" -- the previous behaviour: command the master's
        #       relative rotation raw, so rest is the IDENTITY quaternion.
        # Measured over a 243-pose sweep around the anchor, /compute_ik
        # seeded from the live joint state:
        #       left   relative  5.3% ->  anchored 92.6%
        #       right  relative  0.8% ->  anchored 62.6%
        # Position was already anchored; orientation was not, and that
        # asymmetry is what made position and orientation jointly
        # unreachable. Revert with -p orientation_mode:=relative.
        # "fixed" is the DEFAULT because the master wrist is not currently
        # measurable (see the fixed branch below for the reachability
        # numbers). Switch to "anchored" once j6/j7 (left) and j3/j5/j7
        # (right) are repaired and orientation can be trusted.
        # "tilt" adds roll+pitch from the wrist IMU (drift-free gravity
        # vector) while leaving YAW uncommanded -- yaw is unobservable from
        # an accelerometer and j7 is railed left / dead right. The follower
        # sends it as an OrientationConstraint with a wide yaw tolerance
        # rather than an exact pose; an exact pose is what crushed IK to
        # 5.3% before.
        self.declare_parameter("orientation_mode", "fixed")

        self.offsets = {
            a: (
                self.get_parameter(f"{a}_offset_x").value,
                self.get_parameter(f"{a}_offset_y").value,
                self.get_parameter(f"{a}_offset_z").value,
                self.get_parameter(f"{a}_scale").value,
            )
            for a in self.arms
        }
        self.orientation_mode = self.get_parameter("orientation_mode").value
        if self.orientation_mode not in ("fixed", "tilt", "anchored", "relative"):
            raise ValueError(
                f"orientation_mode must be 'fixed', 'tilt', 'anchored' or "
                f"'relative', got {self.orientation_mode!r}")
        self.anchor_quat = {
            a: np.array([self.get_parameter(f"{a}_anchor_{n}").value
                         for n in ("qx", "qy", "qz", "qw")], dtype=float)
            for a in self.arms
        }

        # ---- per-arm runtime state ----
        # last_good: last joint vector (degrees) that passed validation.
        # A rejected frame reuses it rather than publishing a corrupt pose.
        self.last_good = {a: None for a in self.arms}
        # Per-CHANNEL last accepted value and its time, for the rate check.
        self.last_chan = {a: [None] * 7 for a in self.arms}
        self.last_chan_t = {a: [None] * 7 for a in self.arms}
        self.incoherent = {a: [0] * 7 for a in self.arms}
        # Timestamp of that frame -- the step tolerance is scaled by how
        # long ago it was, so a dropout cannot deadlock the gate.
        self.last_good_time = {a: None for a in self.arms}
        self.tip_filt = {a: None for a in self.arms}   # EMA state (local FK metres)
        self.frames = {a: 0 for a in self.arms}
        self.rejects = {a: 0 for a in self.arms}
        self.resyncs = {a: 0 for a in self.arms}
        self.reject_by_ch = {a: {} for a in self.arms}
        self.consec_rejects = {a: 0 for a in self.arms}
        self.consec_warned = {a: False for a in self.arms}
        # Spherical mode: last trusted elevation, held while the arm is
        # accelerating and gravity cannot be isolated.
        self.last_elev = {a: None for a in self.arms}
        self.elev_holds = {a: 0 for a in self.arms}
        self.recovered_by_ch = {a: {} for a in self.arms}
        self.last_scale = {a: self.offsets[a][3] for a in self.arms}
        self.last_tilt = {a: None for a in self.arms}
        self.pos_anchor_base = {
            a: np.array(self.offsets[a][:3], dtype=float) for a in self.arms}
        self.anchor_ref = {
            a: np.array(self.offsets[a][:3], dtype=float) for a in self.arms}
        # Gyro-aided azimuth state, per arm.
        self.azim_gyro = {a: None for a in self.arms}   # integrated estimate
        self.last_u_hat = {a: None for a in self.arms}  # last trusted vertical
        self.last_gyro_t = {a: None for a in self.arms}
        self.gyro_rest_warned = {a: False for a in self.arms}
        # Clutch re-engage support: recent master displacements (for the
        # quasi-static test) and the frames being averaged into the reference.
        self.recent_p = {a: [] for a in self.arms}
        self.ref_accum = {a: [] for a in self.arms}
        self.last_btn_edge_t = {a: 0.0 for a in self.arms}
        # Latest per-frame diagnostics, republished on /master_status_<arm>
        # so a recorder can log clutch/scale/elevation without scraping logs.
        self.status = {a: [0.0] * 8 for a in self.arms}

        # Clutch. Starts ENGAGED and tracks button CHANGES only, so
        # start-up behaviour is identical to before the clutch existed --
        # nothing latches until the operator actually presses something.
        # anchor + (d - d_ref) keeps the output continuous across a
        # re-engage: at the instant of re-engage d == d_ref, so the
        # commanded pose is exactly the held one.
        self.clutch_on = {a: True for a in self.arms}
        self.btn_last = {a: None for a in self.arms}
        self.pending_engage = {a: False for a in self.arms}
        # BOOT REFERENCE. This used to be zeros(3), i.e. the master-frame
        # ORIGIN, so the very first command was
        #     pos_anchor + scale * (tip - 0) = home + tip
        # and the arm jumped by the master's whole tip vector (~0.22 m) in
        # whatever direction the master happened to be lying. That is why the
        # operator had to hold the master at a calibrated neutral before
        # enabling: the only master pose that did not jump was the one where
        # tip == 0.
        #
        # None means "not latched yet". The first VALID frame per arm latches
        # it, so wherever the master is at startup maps to the arm's home.
        self.d_ref = {a: None for a in self.arms}
        self.boot_ref_latched = {a: False for a in self.arms}
        self.pos_anchor = {
            a: np.array(self.offsets[a][:3], dtype=float) for a in self.arms
        }
        # Last pose actually commanded, so a disengage can freeze exactly
        # there. Seeded to the anchor so the very first frame behaves the
        # same as it did before the clutch existed.
        self.last_pos = {
            a: np.array(self.offsets[a][:3], dtype=float) for a in self.arms
        }
        self.last_quat = {a: None for a in self.arms}

        self.pubs = {
            a: self.create_publisher(PoseStamped, f"/master_arm_pose_{a}", 10)
            for a in self.arms
        }
        # Also publish the raw joint angles (degrees) + raw IMU accel,
        # separately from the computed pose -- needed to diagnose whether
        # a mismatch originates from the raw pot/IMU reading itself, or
        # from something in the FK/orientation math downstream.
        # Published so the GUI and any observer can see the channel set
        # WITHOUT reading a log. "I could not tell degraded mode was on" is
        # the same class of failure as a blocker with no name.
        self.degraded_pub = self.create_publisher(
            String, "/master_channel_state",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self.publish_channel_state)
        self.create_service(Trigger, "/master_rebase", self.srv_rebase)

        self.raw_pubs = {
            a: self.create_publisher(Float64MultiArray, f"/master_arm_raw_{a}", 10)
            for a in self.arms
        }
        # FSR (fsr1=left gripper, fsr2=right gripper) + buttons
        # (btn1=left, btn2=right) -- global, published once per line.
        self.fsr_btn_pub = self.create_publisher(
            Float64MultiArray, "/master_fsr_buttons", 10)
        # Per-frame state a recorder needs, published rather than logged:
        # [clutch_on, scale, elev_deg, azim_deg, reach_m,
        #  frame_valid, n_dropouts, elev_held]
        self.status_pubs = {
            a: self.create_publisher(
                Float64MultiArray, f"/master_status_{a}", 10)
            for a in self.arms
        }

        self.create_timer(5.0, self.log_health)

        # ---- startup state, stated explicitly rather than inferred ----
        # The clutch has always initialised ENGAGED and only flips on a
        # button CHANGE, so it cannot latch DISENGAGED at startup -- but
        # that was previously only visible by reading the code, so say it.
        for a in self.arms:
            self.get_logger().info(
                f"[CLUTCH:{a}] startup state = "
                f"{'ENGAGED' if self.clutch_on[a] else 'DISENGAGED'}"
                f"{' (FORCED, buttons ignored)' if self.force_clutch else ''}, "
                f"clutch_enabled={self.clutch_enabled}, "
                f"button=btn{self.clutch_button.get(a)} (MEASURED 2026-07-31: "
                f"left->btn2, right->btn1)")

        self.get_logger().info(
            f"[MODE] position_mode={self.position_mode} "
            f"(validating j{[i+1 for i in self.check_idx]}, "
            f"accel gate 1 +/- {self.accel_gate:.2f} g)")

        if self.position_mode == "spherical":
            missing = [a for a in self.arms if self.calib[a].a_hat is None]
            if missing:
                raise RuntimeError(
                    "position_mode=spherical needs a_hat, and "
                    + ", ".join(f"master_zero_{a}.txt" for a in missing)
                    + " has none. Capture it with the arm HANGING STRAIGHT "
                      "DOWN:\n    python3 "
                      "src/srl_teleop/srl_teleop/capture_zero.py <arm> <port>\n"
                      "Or run with -p position_mode:=fk to use the old path.")
            for a in self.arms:
                self.get_logger().info(
                    "[MODE] %s a_hat = [%+.4f %+.4f %+.4f]"
                    % (a, *self.calib[a].a_hat))

        # Port resolution FAILS LOUDLY. The old path set self.ser = None and
        # spun at 50 Hz reading nothing, which is indistinguishable from a
        # dead board -- and cost a long diagnosis once already.
        import serial
        baud = self.get_parameter("baud_rate").value
        port = find_port(self.get_parameter("serial_port").value, baud,
                         logger=self.get_logger())
        self.port = port
        self.read_fail = 0
        self.empty_reads = 0
        # ~2 s of silence at 50 Hz before we conclude the port is gone.
        self.read_fail_limit = 25
        self.empty_read_limit = 100
        self.ser = serial.Serial(port, baud, timeout=0.05)
        # EXCLUSIVE. A second reader silently splits the frame stream; see
        # serial_port.claim_exclusive(). This raises rather than degrading.
        claim_exclusive(self.ser, self.get_logger())
        self.create_timer(0.02, self.read_serial)  # 50 Hz poll
        self.get_logger().info(
            f"Reading master {self.arm} arm from {port} @ {baud}")

    # ---------------- validation ----------------

    def step_threshold_deg(self, arm, now):
        """Tolerance for check (b), as a velocity budget over the time
        actually elapsed since the last good frame -- NOT a fixed
        per-frame step. During a dropout the arm keeps moving, so the
        gate has to widen at the rate the arm can physically move,
        otherwise the recovery frame is rejected and the reference can
        never advance again."""
        t0 = self.last_good_time[arm]
        if t0 is None:
            return self.step_cap_deg
        dt = max((now - t0).nanoseconds / 1e9, 0.0)
        return min(self.max_vel_deg_s * dt, self.step_cap_deg)

    def reject_incoherent(self, arm, joints_deg, now):
        """Mark as DROPOUT any channel whose implied rate is not physical.

        A failing pot does not go quiet -- it returns numbers that are not a
        trajectory. Measured on the 2026-08-06 capture, 7 of 14 channels had
        more than 5% of updates jumping over 60 deg between consecutive
        sensor updates, which at the sensor's ~15 Hz is about 2500 deg/s.
        No hand does that, and passing it downstream puts a discontinuity
        into the commanded pose that the sim reproduces and the real arm
        copies a second later.

        THRESHOLD, and how it was derived. Steps between distinct updates on
        the four channels that ARE coherent (r_j1, r_j2, r_j6, l_j7), n=7126:

            p50   0.40 deg      p90   6.25 deg  (about 93 deg/s)
            p99 102.60 deg  ->  1531 deg/s      (already the glitch tail)

        `max_channel_rate_deg_s` sits between them at 800 deg/s: far above
        any deliberate sweep, far below the glitch tail. It is expressed as a
        RATE and evaluated against the actual elapsed time, so it does not
        silently change meaning if the sensor rate does.

        A rejected value is set to 0.0 -- the firmware's existing dropout
        marker -- so it flows through the mode-scoped validation already in
        place rather than inventing a parallel mechanism. Nothing is
        fabricated: a rejected channel is reported missing, not guessed.
        """
        lim = float(self.get_parameter("max_channel_rate_deg_s").value)
        if lim <= 0:
            return joints_deg
        prev = self.last_chan[arm]
        prev_t = self.last_chan_t[arm]
        out = list(joints_deg)
        t = now.nanoseconds * 1e-9
        for i, v in enumerate(out):
            if v == 0.0:
                continue                       # already a dropout
            p, pt = prev[i], prev_t[i]
            if p is not None and pt is not None:
                dt = max(1e-3, t - pt)
                d = abs(v - p)
                d = min(d, 360.0 - d)          # shortest way round the seam
                if d / dt > lim:
                    self.incoherent[arm][i] += 1
                    out[i] = 0.0
                    self.get_logger().warn(
                        "[INCOHERENT:%s] j%d %.1f -> %.1f in %.3f s = "
                        "%.0f deg/s, over the %.0f deg/s limit. Marked as a "
                        "dropout, not passed downstream."
                        % (arm, i + 1, p, v, dt, d / dt, lim),
                        throttle_duration_sec=5.0)
                    continue
            prev[i], prev_t[i] = v, t
        return out

    def validate_frame(self, arm, joints_deg, now):
        """Returns (ok, [offending joint indices], [human-readable reasons]).
        Never modifies or fills the data -- a bad frame is rejected whole.

        Only the channels this position_mode actually consumes can reject a
        frame. Under spherical mode that is j1, j2 and j4; a dead j3, j5, j6
        or j7 is simply unused, which is the whole point of the design."""
        bad, reasons = [], []

        # (a) exact 0.0 on a load-bearing channel == firmware clamp ==
        # dropout. Exact compare is deliberate: the firmware writes
        # literal 0.0, so a real reading that happens to be near zero
        # will not be a bit-exact 0.0.
        for i in zero_dropouts(joints_deg, self.check_idx_arm[arm]):
            bad.append(i)
            reasons.append(f"j{i+1} exactly 0.0")

        # (b) motion faster than an arm can physically move.
        last = self.last_good[arm]
        if last is not None:
            thresh = self.step_threshold_deg(arm, now)
            for i in self.check_idx_arm[arm]:
                step = abs(circ_diff_deg(joints_deg[i], last[i]))
                if step > thresh:
                    if i not in bad:
                        bad.append(i)
                    reasons.append(
                        f"j{i+1} moved {step:.1f} deg (budget {thresh:.1f})")

        return (not bad), bad, reasons

    def note_rejection(self, arm, bad, reasons):
        self.rejects[arm] += 1
        for i in bad:
            self.reject_by_ch[arm][i] = self.reject_by_ch[arm].get(i, 0) + 1
        self.consec_rejects[arm] += 1
        if (self.consec_rejects[arm] > self.consec_warn
                and not self.consec_warned[arm]):
            self.consec_warned[arm] = True
            self.get_logger().warn(
                f"[HEALTH:{arm}] {self.consec_rejects[arm]} CONSECUTIVE frames "
                f"rejected -- a channel has likely failed outright, not just "
                f"glitched ({'; '.join(reasons)}). Holding last good pose.")

    def srv_rebase(self, req, resp):
        """Re-latch the boot reference at the master's CURRENT pose.

        Called by sim_to_real_bridge when it enables, so "wherever the master
        is now maps to the arm's home" holds at the moment real control
        starts, not merely at node startup -- the operator will have moved in
        between, and the bridge seeds from the real arm's actual position.
        """
        for a in self.arms:
            self.d_ref[a] = None            # next valid frame re-latches
            self.boot_ref_latched[a] = False
        resp.success = True
        resp.message = ("reference cleared on %s; the next valid frame "
                        "re-latches at the master's actual pose"
                        % ",".join(self.arms))
        self.get_logger().warn("[REF] %s" % resp.message)
        return resp

    def publish_channel_state(self):
        """One line per arm, latched, so a late subscriber still gets it."""
        bits = ["DEGRADED" if self.degraded else "FULL",
                "%d/14 coherent" % dg.n_coherent(self.dg_baseline)]
        for a in self.arms:
            frz = self.frozen_idx.get(a, [])
            bits.append("%s: use j%s frozen j%s %s" % (
                a,
                "".join(str(i + 1) for i in range(7) if i not in frz) or "-",
                "".join(str(i + 1) for i in frz) or "-",
                ("reach=j%d @ %.3f m" % (self.rf_ch[a] + 1, self.rf_reach[a]))
                if self.rf_on.get(a) else "reach=pots"))
        m = String()
        m.data = " | ".join(bits)
        self.degraded_pub.publish(m)

    def log_health(self):
        for a in self.arms:
            if self.frames[a] == 0:
                continue
            per_ch = " ".join(
                f"j{i+1}:{n}" for i, n in sorted(self.reject_by_ch[a].items()))
            per_ch = f" ({per_ch})" if per_ch else ""
            resync = f" resync {self.resyncs[a]}" if self.resyncs[a] else ""
            clutch = "ENGAGED" if self.clutch_on[a] else "DISENGAGED"
            if self.force_clutch:
                clutch += "(forced)"
            # Elevation holds are NOT rejections: the frame is good, but the
            # arm was accelerating so gravity could not be isolated and the
            # last trusted elevation was reused. Reported separately so a
            # fast-motion window is never mistaken for a channel failure.
            held = f" elev_held {self.elev_holds[a]}" if self.elev_holds[a] else ""
            rec = " ".join(f"j{i+1}:{n}"
                           for i, n in sorted(self.recovered_by_ch[a].items()))
            rec = f" recovered {rec}" if rec else ""
            pct = 100.0 * self.rejects[a] / self.frames[a] if self.frames[a] else 0.0
            self.get_logger().info(
                f"[HEALTH:{a}] mode={self.position_mode} frames {self.frames[a]} "
                f"rejected {self.rejects[a]} ({pct:.1f}%){per_ch}{resync}"
                f"{held}{rec} clutch={clutch}")
            self.frames[a] = 0
            self.rejects[a] = 0
            self.resyncs[a] = 0
            self.elev_holds[a] = 0
            self.reject_by_ch[a] = {}
            self.recovered_by_ch[a] = {}

    # ---------------- spherical position estimate ----------------

    # ---------------- gyro-aided azimuth ----------------

    def gyro_azimuth(self, arm, azim_j1, accel, gyro, quasi_static):
        """Azimuth by integrating yaw rate about the measured vertical.

        The gyro is the only sensor here that observes rotation about the
        vertical: the accelerometer cannot (gravity looks identical at any
        heading), and j1 cannot on its own because its lever arm scales with
        the arm's bend. It drifts, so it is bounded three ways:
          * RESET to the j1 estimate at every clutch engage -- short segments
            are what make integration viable at all;
          * a slow first-order pull toward j1 (azimuth_blend_tau_s), so a long
            segment cannot run away, while a fast lateral sweep is still
            governed by the gyro;
          * the stored per-arm bias is subtracted every frame.

        When |accel| leaves the gate, u_hat is unreliable -- but that is
        exactly the fast motion the gyro exists to cover, so the LAST trusted
        u_hat is held and integration continues, rather than being skipped.
        """
        cal = self.calib[arm]
        now = self.get_clock().now()

        if quasi_static:
            self.last_u_hat[arm] = np.asarray(accel, dtype=float)
        u = self.last_u_hat[arm]
        if u is None or cal.gyro_bias is None:
            # No trusted vertical yet, or no bias captured: fall back to j1
            # rather than integrating something meaningless.
            self.azim_gyro[arm] = azim_j1
            self.last_gyro_t[arm] = now
            return azim_j1

        if self.azim_gyro[arm] is None or self.last_gyro_t[arm] is None:
            self.azim_gyro[arm] = azim_j1
            self.last_gyro_t[arm] = now
            return azim_j1

        dt = (now - self.last_gyro_t[arm]).nanoseconds / 1e9
        self.last_gyro_t[arm] = now
        if dt <= 0.0 or dt > 0.5:          # first frame, or a serial stall
            return self.azim_gyro[arm]

        rate = mc.yaw_rate_from_gyro(gyro, cal.gyro_bias, u)
        a = self.azim_gyro[arm] + rate * dt

        # Slow pull toward j1. Weight is dt/tau, so it is frame-rate
        # independent; the shortest-angle difference keeps it wrap-safe.
        w = min(1.0, dt / max(self.azim_tau, 1e-6))
        a += w * math.atan2(math.sin(azim_j1 - a), math.cos(azim_j1 - a))

        # Stationary-bias health check.
        if quasi_static and not self.gyro_rest_warned[arm]:
            still = float(np.linalg.norm(
                np.asarray(gyro, float) - np.asarray(cal.gyro_bias, float)))
            if still > mc.GYRO_REST_WARN_DPS * 3.0:
                pass                        # moving; not a bias problem
        self.azim_gyro[arm] = a
        return a

    def spherical_tip(self, arm, joints_deg, joints_rad, accel, gyro):
        """Master tip position in the master frame, IMU-primary.

        ELEVATION from gravity: drift-free, mount-independent, and the only
        source of up/down here -- the 7-joint FK could not separate them.
        AZIMUTH from j1 alone, already zero-referenced by calib.apply().
        REACH  from the magnitude of fk([j1,j2,j3,j4,0,0,0]). Magnitude is
               dominated by the j2/j4 bends and is far more robust than FK
               direction; j3 contributes very little, so a dead j3 is passed
               as 0 rather than invented.

        Returns None when no trustworthy elevation exists yet, so the caller
        skips the frame instead of publishing a fabricated direction.
        """
        a_hat = self.calib[arm].a_hat

        # Gravity is only isolable when the arm is not accelerating.
        mag = float(np.linalg.norm(accel))
        quasi_static = abs(mag - 1.0) < self.accel_gate
        if quasi_static:
            elev = mc.elevation_from_accel(a_hat, accel)
            self.last_elev[arm] = elev
        else:
            if self.last_elev[arm] is None:
                self.elev_holds[arm] += 1
                self.get_logger().warn(
                    f"[HEALTH:{arm}] |accel| = {mag:.3f} g outside the "
                    f"1 +/- {self.accel_gate:.2f} g gate and no trusted "
                    f"elevation yet -- cannot publish. Hold the arm still.",
                    throttle_duration_sec=2.0)
                return None
            elev = self.last_elev[arm]
            self.elev_holds[arm] += 1

        azim_j1 = joints_rad[J1_IDX]
        azim = azim_j1
        if self.azimuth_mode == "gyro":
            azim = self.gyro_azimuth(arm, azim_j1, accel, gyro, quasi_static)

        # j3 feeds reach only. If the channel is dropped out, use 0 -- never
        # interpolate a dead channel, and never let it reject the frame.
        q3 = joints_rad[J3_IDX]
        if float(joints_deg[J3_IDX]) == 0.0:
            q3 = 0.0
            self.recovered_by_ch[arm][J3_IDX] = \
                self.recovered_by_ch[arm].get(J3_IDX, 0) + 1
        reach_q = [joints_rad[0], joints_rad[1], q3, joints_rad[3], 0.0, 0.0, 0.0]
        reach = float(np.linalg.norm(mc.fk(reach_q)))

        # RADIAL FALLBACK. With j2 and j4 both frozen the FK magnitude above
        # is a CONSTANT -- measured R^2 of true reach on every remaining live
        # channel is 0.133 on the left arm, i.e. no reach observable exists.
        # The operator drives radius directly instead.
        if self.rf_on.get(arm):
            ch = self.rf_ch[arm]
            now_s = time.monotonic()
            prev = self.rf_t[arm]
            self.rf_t[arm] = now_s
            if prev is not None:
                dt = min(0.2, max(0.0, now_s - prev))
                defl = joints_rad[ch]
                if abs(defl) > self.rf_dead:
                    # Deflection past the deadband, normalised on a 45 deg
                    # span so a 26 deg channel still reaches full rate near
                    # its own limit.
                    frac = max(-1.0, min(1.0, (abs(defl) - self.rf_dead)
                                         / math.radians(45.0)))
                    frac = math.copysign(frac, defl)
                    want = self.rf_reach[arm] + self.rf_rate * frac * dt
                    self.rf_reach[arm] = min(self.rf_max,
                                             max(self.rf_min, want))
                    # SAY SO AT THE CLAMP. Otherwise the operator keeps
                    # pushing the wrist and the arm simply does not move --
                    # a check that can only say no, with nothing said. This
                    # was measured: with reach pinned at a limit, four
                    # consecutive index cycles produced 0.00 mm of travel and
                    # nothing in the logs explained it.
                    if want > self.rf_max + 1e-9 or want < self.rf_min - 1e-9:
                        self.get_logger().warn(
                            "[REACH:%s] at the %s limit (%.3f m). Pushing j%d "
                            "further does nothing -- reverse the wrist to "
                            "come back in, or raise reach_%s_m."
                            % (arm, "OUTER" if want > self.rf_max else "INNER",
                               self.rf_reach[arm], ch + 1,
                               "max" if want > self.rf_max else "min"),
                            throttle_duration_sec=2.0)
            reach = self.rf_reach[arm]

        self.status[arm][2] = math.degrees(elev)
        self.status[arm][3] = math.degrees(azim)
        self.status[arm][4] = reach
        self.status[arm][7] = 0.0 if abs(mag - 1.0) < self.accel_gate else 1.0

        return mc.spherical_position(reach, elev, azim)

    def master_is_static(self, arm):
        """True when the master has moved under clutch_static_m over the
        recent window. Uses the displacement history kept per frame."""
        h = self.recent_p[arm]
        if len(h) < 3:
            return False
        span = np.array([p for _, p in h])
        return float(np.linalg.norm(span.max(axis=0) - span.min(axis=0))) \
            < self.clutch_static_m

    # ---------------- clutch ----------------

    def update_clutch(self, arm, btn_values, frame_valid, d_now):
        """btn_values is (btn1, btn2). The buttons are firmware toggles, so
        we act on CHANGES to the reported value, one flip per press."""
        if not self.clutch_enabled:
            return
        if self.force_clutch:
            # Pinned ENGAGED: ignore the buttons entirely. The mapping is
            # unconfirmed, so this rules the clutch out as a cause of a
            # frozen arm without having to trust btn1/btn2 at all.
            self.clutch_on[arm] = True
            self.pending_engage[arm] = False
            return
        # LIVE mapping. Re-read every frame so `ros2 param set` swaps it
        # without a rebuild -- the mapping has never been confirmed against
        # hardware, so it must be changeable while the rig is in your hands.
        new_idx = int(self.get_parameter("%s_clutch_button" % arm).value)
        if new_idx != self.clutch_button.get(arm):
            self.get_logger().warn(
                "[CLUTCH:%s] button mapping changed btn%s -> btn%d"
                % (arm, self.clutch_button.get(arm), new_idx))
            self.clutch_button[arm] = new_idx
            self.btn_last[arm] = None        # resync: do not act on the swap
        idx = new_idx
        if idx not in (1, 2):
            return
        val = btn_values[idx - 1]

        if self.btn_last[arm] is None:
            self.btn_last[arm] = val          # first sight: record, don't act
            return
        if val != self.btn_last[arm]:
            self.btn_last[arm] = val
            # DEBOUNCE: one physical press must not toggle twice.
            tnow = self.get_clock().now().nanoseconds / 1e9
            if tnow - self.last_btn_edge_t[arm] < self.clutch_debounce:
                self.get_logger().info(
                    f"[CLUTCH:{arm}] bounce ignored (<{self.clutch_debounce*1000:.0f} ms)")
                return
            self.last_btn_edge_t[arm] = tnow
            if self.clutch_on[arm]:
                # Disengage is always safe -- it only freezes output.
                # Freeze at the last pose actually commanded.
                self.clutch_on[arm] = False
                self.pending_engage[arm] = False
                self.pos_anchor[arm] = np.array(self.last_pos[arm], dtype=float)
                self.get_logger().info(
                    f"[CLUTCH:{arm}] DISENGAGED (btn{idx}) -- holding pose "
                    f"({self.pos_anchor[arm][0]:+.3f}, "
                    f"{self.pos_anchor[arm][1]:+.3f}, "
                    f"{self.pos_anchor[arm][2]:+.3f}).")
            elif self.pending_engage[arm]:
                # A second press while an engage is still PENDING cancels it.
                # Without this the operator has no way to back out of a
                # deferred engage: the gate keeps refusing (frame invalid, or
                # master still moving) and every further press just re-arms
                # the same pending state, which looks exactly like a dead
                # button.
                self.pending_engage[arm] = False
                self.ref_accum[arm] = []
                self.get_logger().info(
                    "[CLUTCH:%s] pending engage CANCELLED (btn%d). Press "
                    "again to retry." % (arm, idx))
            else:
                # Re-engage must latch a reference from a TRUSTED frame.
                # Latching a corrupt one would silently offset every
                # subsequent command, which is worse than not clutching.
                self.pending_engage[arm] = True

        if self.pending_engage[arm]:
            if not frame_valid:
                self.get_logger().warn(
                    f"[CLUTCH:{arm}] re-engage deferred -- frame failed "
                    f"validation, refusing to latch a corrupt reference.",
                    throttle_duration_sec=1.0)
                return
            # QUASI-STATIC GATE. Latching while the master is moving captures
            # a moving reference, and every command afterwards inherits that
            # offset -- one of the three causes of the re-engage glitch.
            if not self.master_is_static(arm):
                self.get_logger().warn(
                    f"[CLUTCH:{arm}] re-engage deferred -- master still moving "
                    f"(>{self.clutch_static_m*1000:.0f} mm in "
                    f"{self.clutch_static_win*1000:.0f} ms). Hold still.",
                    throttle_duration_sec=1.0)
                return
            # AVERAGE the reference over several frames instead of trusting
            # a single sample of a noisy signal.
            self.ref_accum[arm].append(np.array(d_now, dtype=float))
            if len(self.ref_accum[arm]) < self.clutch_ref_frames:
                return
            ref = np.mean(np.array(self.ref_accum[arm]), axis=0)
            self.ref_accum[arm] = []
            self.pending_engage[arm] = False
            self.clutch_on[arm] = True
            self.d_ref[arm] = ref
            # RESET the EMA. It still holds pre-disengage state, so without
            # this the first frames after engage blend a stale position into
            # the command -- the third cause of the glitch.
            self.tip_filt[arm] = None
            # RESET the integrated azimuth at every engage. Bounded segments
            # are what make gyro integration usable at all -- drift only ever
            # accumulates from the most recent engage, never across a session.
            self.azim_gyro[arm] = None
            self.last_gyro_t[arm] = None
            self.get_logger().info(
                f"[CLUTCH:{arm}] ENGAGED (btn{idx}) -- new reference latched, "
                f"resuming from ({self.pos_anchor[arm][0]:+.3f}, "
                f"{self.pos_anchor[arm][1]:+.3f}, "
                f"{self.pos_anchor[arm][2]:+.3f}).")

    # ---------------- main loop ----------------

    def read_serial(self):
        if self.ser is None:
            return
        try:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
        except Exception as e:
            # DO NOT swallow this. The Teensy re-enumerates often (ACM0 <-> ACM1
            # on every usbipd re-attach) and the port is resolved ONCE, in the
            # constructor. Silently returning here left the node alive holding a
            # dead file descriptor: no poses, no trajectories, an arm that looks
            # broken, and respawn never firing because nothing crashed.
            # Exiting lets launch respawn us and re-run find_port().
            self.read_fail += 1
            if self.read_fail >= self.read_fail_limit:
                self.get_logger().error(
                    f"Serial read failed {self.read_fail} times ({e}). The "
                    f"Teensy has probably re-enumerated to another /dev/ttyACM*. "
                    f"EXITING so launch respawns and re-detects the port.")
                raise SystemExit(1)
            return
        if not line:
            # An always-empty read is the same fault presenting differently:
            # the device node vanished but the fd is still open.
            self.empty_reads += 1
            if (self.empty_reads >= self.empty_read_limit
                    and not os.path.exists(self.port)):
                self.get_logger().error(
                    f"{self.port} no longer exists and {self.empty_reads} reads "
                    f"returned nothing. EXITING so launch respawns and "
                    f"re-detects the port.")
                raise SystemExit(1)
            return
        self.read_fail = 0
        self.empty_reads = 0

        fsr_btn = parse_fsr_buttons(line)
        btn_values = None
        if fsr_btn is not None:
            fsr1, fsr2, btn1, btn2 = fsr_btn
            btn_values = (btn1, btn2)
            msg = Float64MultiArray()
            msg.data = [fsr1, fsr2, float(btn1), float(btn2)]
            self.fsr_btn_pub.publish(msg)

        # One line contains BOTH arms' data -- process each arm this
        # node is handling from the SAME line, no extra serial reads.
        for a in self.arms:
            joint_patterns, imu_pattern = self.parsers[a]
            parsed = parse_arm(line, joint_patterns, imu_pattern)
            if parsed is None:
                continue
            joints_deg, (ax, ay, az), (gx, gy, gz) = parsed

            # ---- stage -1: DEGRADED MODE FREEZE ----
            # Before incoherence rejection and before validation, so a
            # frozen channel cannot trip either. It is replaced by its
            # zero-reference raw value, which calib.apply() maps to 0 rad.
            frz = self.frozen_idx.get(a, ())
            if frz:
                joints_deg = list(joints_deg)
                for i in frz:
                    joints_deg[i] = self.freeze_val[a][i]
                self.frozen_counts[a] += 1

            self.frames[a] += 1

            # ---- stage 0: INCOHERENCE REJECTION ----
            now = self.get_clock().now()
            joints_deg = self.reject_incoherent(a, joints_deg, now)

            # ---- stage 1: validation ----
            ok, bad, reasons = self.validate_frame(a, joints_deg, now)

            # Hard recovery. Only ever onto a frame with no dropout
            # markers -- resyncing onto a clamped 0.0 would latch garbage
            # as the new reference, which is exactly what validation
            # exists to prevent.
            forced = (not ok
                      and self.consec_rejects[a] >= self.force_resync_after
                      and not zero_dropouts(joints_deg,
                                            self.check_idx_arm[a]))
            if forced:
                ok = True
                self.resyncs[a] += 1
                self.get_logger().warn(
                    f"[HEALTH:{a}] FORCED RESYNC after {self.consec_rejects[a]} "
                    f"consecutive rejections -- accepting a frame that failed "
                    f"({'; '.join(reasons)}) and resetting the reference. "
                    f"Expect one visible jump.")

            if ok:
                self.last_good[a] = list(joints_deg)
                self.last_good_time[a] = now
                self.consec_rejects[a] = 0
                self.consec_warned[a] = False
                joints_used = joints_deg
            else:
                self.note_rejection(a, bad, reasons)
                if self.last_good[a] is None:
                    # Nothing trustworthy has ever arrived for this arm --
                    # publishing anything here would be inventing data.
                    self.get_logger().warn(
                        f"[HEALTH:{a}] no good frame yet, cannot publish "
                        f"({'; '.join(reasons)}).", throttle_duration_sec=2.0)
                    continue
                joints_used = self.last_good[a]

            # THE STAMP IS THE AGE OF THE DATA, NOT THE AGE OF THE MESSAGE.
            #
            # This branch republishes the last GOOD joint vector at full rate.
            # Stamping it `now` (which it used to do, unconditionally, below)
            # made a frozen command indistinguishable from a live one: the
            # topic never goes silent and the timestamp is never stale, so
            # NEITHER an arrival-based NOR a timestamp-based dead-man can fire.
            # That is the partial-failure mode -- a Teensy emitting garbage
            # rather than vanishing -- and it is the dangerous one, because a
            # clean unplug already exits the node and lets launch respawn it.
            #
            # Carrying the source time forward means "stale" is observable by
            # anyone downstream with no extra channel: /estop_node's dead-man
            # keys on exactly this field.
            src_time = self.last_good_time[a] if ok is False else now
            data_age_s = (now - src_time).nanoseconds * 1e-9

            joints_rad = self.calib[a].apply(joints_used)

            # ---- stage 2: position estimate, then EMA smoothing ----
            if self.position_mode == "spherical":
                tip = self.spherical_tip(a, joints_used, joints_rad,
                                         (ax, ay, az), (gx, gy, gz))
                if tip is None:
                    continue
            else:
                tip = np.asarray(mc.fk(joints_rad), dtype=float)
            alpha = float(self.get_parameter("ema_alpha").value)
            alpha = min(max(alpha, 0.0), 1.0)
            if self.tip_filt[a] is None:
                self.tip_filt[a] = tip
            else:
                self.tip_filt[a] = alpha * tip + (1.0 - alpha) * self.tip_filt[a]

            _p = mc.to_robot_frame(self.tip_filt[a]) - self.neutral

            # ---- stage 3: clutch ----
            # Scale is re-read EVERY FRAME so `ros2 param set` takes effect
            # live. It multiplies the displacement from the reference latched
            # at clutch-engage time -- NOT from the workspace centre. Scaling
            # about the centre would make any scale change instantly re-map
            # the arm's current position and teleport it; about the engage
            # reference, the term (_p - d_ref) is zero at the engage point,
            # so changing scale pivots around where the operator actually is
            # and the arm does not jump.
            # Anchor re-read every frame, same as scale, so it can be
            # re-anchored live -- and so the startup self-test can nudge it
            # to prove the whole master->IK->controller->TF path moves.
            self.pos_anchor_base[a] = np.array(
                [self.get_parameter(f"{a}_offset_{c}").value for c in "xyz"],
                dtype=float)
            scale = float(self.get_parameter(f"{a}_scale").value)
            if scale != self.last_scale[a]:
                # Re-base the anchor so the CURRENTLY commanded pose is
                # unchanged by the scale change itself. Without this the
                # command jumps by (new-old)*displacement the instant the
                # parameter is set -- worst exactly when the operator is
                # far from the engage point and the jump is largest.
                d = _p - self.d_ref[a]
                self.pos_anchor[a] = (self.pos_anchor[a]
                                      + (self.last_scale[a] - scale) * d)
                self.get_logger().info(
                    f"[SCALE:{a}] {self.last_scale[a]:.2f} -> {scale:.2f}, "
                    f"anchor re-based by "
                    f"{np.linalg.norm((self.last_scale[a]-scale)*d):.3f} m "
                    f"so the commanded pose is continuous.")
                self.last_scale[a] = scale

            tnow = self.get_clock().now().nanoseconds / 1e9
            self.recent_p[a].append((tnow, np.array(_p, dtype=float)))
            self.recent_p[a] = [(t, p) for (t, p) in self.recent_p[a]
                                if tnow - t <= self.clutch_static_win]
            if btn_values is not None:
                self.update_clutch(a, btn_values, ok, _p)

            # BOOT REFERENCE: latch on the first valid frame for this arm.
            # Deliberately requires `ok` -- latching a reference from a frame
            # that failed validation would offset every later command, which
            # is the same failure the re-engage path already refuses.
            if self.d_ref[a] is None:
                if not ok:
                    continue
                self.d_ref[a] = np.array(_p, dtype=float)
                self.boot_ref_latched[a] = True
                self.get_logger().warn(
                    "[REF:%s] BOOT REFERENCE latched at the master's ACTUAL "
                    "pose (%+.3f, %+.3f, %+.3f). This pose now maps to the "
                    "arm's home -- you do NOT have to hold the master at a "
                    "calibrated neutral." % (a, *self.d_ref[a]))

            if self.clutch_on[a]:
                # Absolute against a FIXED reference latched at engage
                # time -- not an accumulation of per-frame increments,
                # which would drift without bound over a long session.
                anchor_delta = self.pos_anchor_base[a] - self.anchor_ref[a]
                pos = (self.pos_anchor[a] + anchor_delta
                       + scale * (_p - self.d_ref[a]))
            else:
                pos = (self.pos_anchor[a]
                       + self.pos_anchor_base[a] - self.anchor_ref[a])
            self.last_pos[a] = np.array(pos, dtype=float)

            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

            quasi_static_last = {a: abs(float(np.linalg.norm((ax, ay, az))) - 1.0)
                                 < self.accel_gate}
            raw_roll, raw_pitch = roll_pitch_from_accel(ax, ay, az)
            raw_yaw = joints_rad[6]

            if self.roll_zero[a] is None:
                self.roll_zero[a] = raw_roll
                self.pitch_zero[a] = raw_pitch
                self.yaw_zero[a] = raw_yaw
                self.get_logger().info(
                    f"[CAL:{a}] orientation zero-referenced: "
                    f"roll={math.degrees(raw_roll):.1f} deg, "
                    f"pitch={math.degrees(raw_pitch):.1f} deg, "
                    f"yaw={math.degrees(raw_yaw):.1f} deg.")
                continue

            roll = raw_roll - self.roll_zero[a]
            pitch = raw_pitch - self.pitch_zero[a]
            yaw = raw_yaw - self.yaw_zero[a]

            q_rel = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            if self.orientation_mode == "tilt":
                # roll/pitch from gravity, yaw taken from the anchor (the
                # follower is told to ignore yaw via a ~pi tolerance).
                ar, ap, ay = tf_transformations.euler_from_quaternion(
                    self.anchor_quat[a])
                if quasi_static_last[a]:
                    self.last_tilt[a] = (roll, pitch)
                r_, p_ = self.last_tilt[a] if self.last_tilt[a] else (0.0, 0.0)
                quat = tf_transformations.quaternion_from_euler(
                    ar + r_, ap + p_, ay)
            elif self.orientation_mode == "fixed":
                # Orientation pinned to the anchor: POSITION-ONLY teleop.
                # The master wrist cannot currently be measured -- left j7 is
                # railed and j6 clamps to 0, right j3/j5/j7 are dead -- so the
                # commanded orientation was swinging with roll/pitch while yaw
                # stayed frozen, which made position and orientation jointly
                # unreachable. Measured over the recorded trajectory:
                #   left  10% -> 100% reachable at scale 0.6 (8% -> 92% at 1.0)
                #   right 62% ->  98% reachable at scale 0.6 (75% -> 95% at 1.0)
                quat = np.array(self.anchor_quat[a], dtype=float)
            elif self.orientation_mode == "anchored":
                # Same structure as the position anchor: at rest q_rel is
                # identity, so the command is exactly the anchor orientation.
                quat = tf_transformations.quaternion_multiply(
                    self.anchor_quat[a], q_rel)
            else:
                quat = q_rel
            # A disengaged clutch holds the whole pose, orientation
            # included -- otherwise the wrist would keep following while
            # the arm was supposedly parked.
            if self.clutch_on[a] or self.last_quat[a] is None:
                self.last_quat[a] = quat
            else:
                quat = self.last_quat[a]

            msg = PoseStamped()
            msg.header.frame_id = self.get_parameter("target_frame").value
            # Source time of the joint data, NOT publication time. See the
            # comment where src_time is computed -- on a substituted frame this
            # is deliberately in the past, and that is the only signal a
            # downstream dead-man has that the master has stopped supplying
            # fresh data while continuing to publish.
            msg.header.stamp = src_time.to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]

            self.pubs[a].publish(msg)

            raw_msg = Float64MultiArray()
            # [k1j1..k1j7 in degrees, ax, ay, az] -- the RAW line as read,
            # never the validated/substituted vector, so this stays usable
            # for diagnosing the channels themselves.
            raw_msg.data = list(joints_deg) + [ax, ay, az, gx, gy, gz]
            self.raw_pubs[a].publish(raw_msg)

            self.status[a][0] = 1.0 if self.clutch_on[a] else 0.0
            self.status[a][1] = scale
            self.status[a][5] = 1.0 if ok else 0.0
            self.status[a][6] = float(len(bad))
            # Age of the joint data behind the pose just published. 0.0 on a
            # good frame; grows while the master is substituting.
            self.status[a][7] = data_age_s
            st = Float64MultiArray()
            st.data = list(self.status[a])
            self.status_pubs[a].publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = MasterPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
