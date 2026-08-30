#!/usr/bin/env python3
"""
vr_pose_mapper.py — Quest controller pose -> robot end-effector target.

WHAT VR CAN DO THAT THE MANNEQUIN CANNOT. The Quest gives full 6-DOF at 72 Hz
with no dead channels, so this path commands ORIENTATION as well as position.
The mannequin path cannot: gravity gives roll and pitch but never yaw, and on
that rig j7 is railed and j5/j6 are dead, which is why `orientation_mode` is
pinned to `fixed` there. Everything below that differs from
`master_pose_node` traces back to that one fact.

RELATIVE, NOT ABSOLUTE. The controller's pose is never mapped to a robot pose
directly — the play space and the robot workspace have no common origin and no
common scale. Instead, at every clutch ENGAGE the current controller pose and
the current robot pose are latched as a matched pair, and thereafter

    p_cmd = p_anchor + scale * R_align * (p_controller - p_ref)
    q_cmd = dq * q_anchor,   dq = q_controller * q_ref^-1   (in the robot frame)

so the operator's hand can be anywhere; only its motion SINCE engage matters.
That is what makes indexing work and what makes the re-engage jump zero by
construction rather than by tuning.

IT MUST BE RESET BETWEEN RUNS, AND IT DID NOT USED TO BE. Everything below
is per-RUN state: the latched references, the anchors, the EMA/speed-limited
`filt`, and `scale`, which the thumbstick moves. Carried into a second run
they are wrong by however far the first run ended up, and the symptom
measured on 02_vr_teleop was a grasp that missed by 88, 115, 156 and 206 mm
against a 30 mm gate -- ACCUMULATING, which is what distinguishes it from a
constant scale error. The harness worked around it by starting a fresh
process per clip; `reset()` and the `/vr/reset` service fix it here, where the
state is.

`filt` is the one that accumulates. It is rate-limited to `max_speed_mps`, so
whenever the command moves faster than that it falls behind and never catches
up while the motion continues -- a lag, not an offset, which is why the miss
grows. That lag is now MEASURED and published as `lag_m`, so the next time it
happens it is visible instead of being inferred from four cube misses.

R_ALIGN IS REAL NOW, AND IT IS A ROTATION. The formula above always claimed
an `R_align` and the code never had one -- the controller displacement was
added raw, so the controller frame WAS the world frame by assumption. That is
true only if the operator stands behind the wearer facing the same way. In the
setup this rig is actually used in, the operator sits ACROSS THE ROOM FACING
THE WEARER, so their frame is turned roughly 180 deg about the vertical.

It is a YAW, never a mirror. Facing someone and copying them is a reflection,
and a reflection has det = -1: it would mirror every ORIENTATION as well as
position, so the gripper would roll the wrong way while the positions looked
right. That is the single hardest class of bug to see, and it is why
`align_yaw_deg` is an angle rather than a set of per-axis sign flips. There is
no configuration of this node that can produce a reflection.

Calibrate it, do not guess it: `scripts/calibrate_operator_yaw.py` solves the
angle from a recorded hand motion.

SCALE CHANGES RE-BASE THE ANCHOR. Changing scale without re-basing moves the
arm by (new-old) x displacement, worst exactly when the operator is far from
the engage point. The same fix the mannequin path uses:

    p_anchor += (old_scale - new_scale) * (p_controller - p_ref)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import Trigger

import tf2_ros

from . import vr_smoothing as smooth

HANDS = ('left', 'right')


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_norm(q):
    n = float(np.linalg.norm(q))
    return np.array([0.0, 0, 0, 1.0]) if n < 1e-12 else np.asarray(q, float) / n


def yaw_quat(deg):
    """Rotation about world +z (up) as (x, y, z, w). A proper rotation for
    every input, which a per-axis sign flip is not."""
    h = math.radians(float(deg)) / 2.0
    return np.array([0.0, 0.0, math.sin(h), math.cos(h)])


def yaw_matrix(deg):
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def q_rot(q, v):
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ np.asarray(v, float)


class VrPoseMapper(Node):
    def __init__(self):
        super().__init__('vr_pose_mapper')
        self.declare_parameter('hands', ['left', 'right'])
        # Which robot arm each controller drives. One controller per arm.
        self.declare_parameter('left_controller_arm', 'left')
        self.declare_parameter('right_controller_arm', 'right')
        # OPERATOR YAW. The angle from the operator's frame to the robot's,
        # about the vertical. 0 = operator faces the same way as the wearer
        # (standing behind them); 180 = operator faces the wearer across the
        # room, which is how this rig is actually driven.
        # 180 DEGREES, AND IT IS DERIVED, NOT TUNED.
        #
        # The operator stands ACROSS THE ROOM FACING THE WEARER. Work the
        # frames rather than guessing signs:
        #   world  +y is the direction the WEARER faces, +z up, so the
        #          wearer's RIGHT is +x (face north, right is east).
        #   the operator faces the wearer, i.e. faces -y, so the OPERATOR's
        #          right is -x (face south, right is west).
        # The controller displacement arrives in the OPERATOR's frame, so
        # +x_op is physically -x_world and +y_op is physically -y_world. That
        # is exactly a 180 deg yaw, and nothing else.
        #
        # AT THE OLD DEFAULT OF 0 THIS WAS MEASURED IN THE LAB, 2026-08-21:
        # left/right was inverted, forward/back was inverted, and ONLY UP AND
        # DOWN WORKED -- because a yaw does not touch z, so z was the one axis
        # a zero yaw could not get wrong. "Only up/down works" is the
        # signature of a missing yaw, not of a broken controller.
        #
        # IT IS A ROTATION AND NEVER A MIRROR. Facing someone and copying them
        # is a reflection (det -1); that would leave positions looking right
        # while every ORIENTATION came out mirrored. No value of this
        # parameter can produce one, which is why it is an angle and not a set
        # of per-axis sign flips.
        self.declare_parameter('align_yaw_deg', 180.0)
        # 1:1. At 0.5 the operator moved 200 mm to command 100 mm, which
        # reads as a sluggish arm rather than as a scale, and it doubles the
        # reach the operator needs for a workspace the arm can already cover.
        # `vr_bringup`'s clean-scale check reads `scale_default` out of the
        # mapper's own state rather than hardcoding a number, so raising this
        # cannot make a freshly reset mapping report as dirty.
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('scale_min', 0.1)
        self.declare_parameter('scale_max', 2.0)
        # Thumbstick y adjusts scale live; VR's play space is much larger than
        # the robot's workspace, so scaling matters more here than for the
        # mannequin, where 1:1 was the right default.
        self.declare_parameter('scale_step_per_s', 0.5)
        self.declare_parameter('command_orientation', True)
        # ------------------------------------------------------- SMOOTHING
        # `one_euro` (default), `ema` (what shipped before), or `none`.
        #
        # THE EMA COULD NOT WIN. A fixed low pass buys stillness and lag with
        # one constant and they trade directly: measured against a 1.5 mm-rms
        # still hand and a 0.40 m/s reach, the shipped ema(0.6) left 1.74 mm
        # of tremor and 3.66 mm of lag, and an EMA retuned to the 1-Euro's
        # 0.51 mm stillness carries 64.4 mm of lag. The adaptive filter is off
        # the fixed filter's trade-off curve, which is the only thing that
        # justifies replacing a control law that was working.
        #
        # `ema` is kept so recordings made before this change reproduce, the
        # same reason `motion_generator:=legacy` exists on the follower side.
        self.declare_parameter('smoothing', 'one_euro')
        self.declare_parameter('ema_alpha', 0.6)
        # Position, in metres. beta is Hz PER METRE PER SECOND -- the 1-Euro
        # paper's example values are for PIXELS and are ~1000x too small here.
        self.declare_parameter('min_cutoff_hz', 0.5)
        self.declare_parameter('beta', 100.0)
        self.declare_parameter('d_cutoff_hz', 0.2)
        # ORIENTATION WAS NOT FILTERED AT ALL until 2026-08-26: the raw
        # controller quaternion went straight to IK while position got an EMA.
        # Wrist tremor therefore reached the arm unattenuated, and it is the
        # wrist that the pads hang off. beta here is Hz per (rad/s).
        self.declare_parameter('smooth_orientation', True)
        self.declare_parameter('rot_min_cutoff_hz', 0.5)
        self.declare_parameter('rot_beta', 8.0)
        self.declare_parameter('rot_d_cutoff_hz', 0.5)
        self.declare_parameter('rate_hz', 100.0)
        # THE RATE LIMIT WAS THE OTHER HALF OF "SLOW". 0.35 m/s is slower
        # than an ordinary reach, so any brisk hand movement hit the limiter
        # and the command fell behind the hand -- published as `lag_m`, and
        # the miss ACCUMULATES while the motion continues, which is what cost
        # mode 02 every grasping task. Raised to 1.20 m/s: still well under
        # the follower's own limit, so the follower and not this node remains
        # the thing that bounds arm speed.
        self.declare_parameter('max_speed_mps', 1.20)
        self.declare_parameter('quiet_engage_m', 0.020)
        self.declare_parameter('quiet_window_s', 0.10)
        # FEED THE DEAD-MAN WHILE NOT DRIVING. On a real-arm launch
        # estop_node latches an e-stop when /master_arm_pose_<arm> goes
        # stale for 0.5 s x 5 checks -- INCLUDING for an arm that has never
        # published. This mapper used to publish only while the clutch was
        # engaged, so every clutch release, every freeze, and the whole run
        # for a one-handed session ended in a LATCHED e-stop that outlived
        # its cause and needed /estop_reset from a terminal. While the
        # mapper is alive but not driving a hand, it now commands the arm's
        # OWN live pose: zero motion by construction (target == where the
        # arm already is), so a freeze still stops the arm -- it just no
        # longer converts into a latched e-stop. The dead-man still catches
        # what it exists for: this PROCESS dying.
        self.declare_parameter('hold_when_idle', True)
        self.declare_parameter('hold_rate_hz', 20.0)

        self.hands = list(self.get_parameter('hands').value)
        self.arm_of = {'left': self.get_parameter('left_controller_arm').value,
                       'right': self.get_parameter('right_controller_arm').value}
        self.scale = float(self.get_parameter('scale').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)

        self.ctrl = {h: None for h in HANDS}
        self.ctrl_hist = {h: [] for h in HANDS}
        self.joy = {h: None for h in HANDS}
        self.engaged = {h: False for h in HANDS}
        # WHY the last squeeze did not engage, per hand. The
        # refusals were logger.warn only, and the mapper runs as a
        # detached subprocess with stdout at DEVNULL -- so the
        # operator squeezed the grip, nothing moved, and the reason
        # existed nowhere they could see it.
        self.engage_refused = {h: None for h in HANDS}
        self.grip_held = {h: False for h in HANDS}
        self.p_ref = {h: None for h in HANDS}
        self.q_ref = {h: None for h in HANDS}
        self.p_anchor = {h: None for h in HANDS}
        self.q_anchor = {h: None for h in HANDS}
        self.filt = {h: None for h in HANDS}
        # The smoothers, one pair per hand. Built here and REBUILT on reset,
        # because a filter carries the last run's state in exactly the way
        # `filt` did -- and that was the accumulating miss that cost mode 02
        # every grasping task.
        self.pfilt = {h: self._make_pfilt() for h in HANDS}
        self.qfilt = {h: self._make_qfilt() for h in HANDS}
        # The key the filters above were built from, so `_smooth_sync` can
        # tell a changed parameter from an unchanged one.
        self._smooth_cur = self._smooth_key()
        self._smooth_bad = None
        self.last_cmd = {h: None for h in HANDS}
        self.tracking_ok = False
        # PER-HAND tracking, separate from the global stream watchdog.
        # /vr/tracking_ok answers "is the link alive"; this answers "is THIS
        # controller actually being seen". Off-head operation makes the second
        # question the one that matters, because an occluded controller is
        # routine there and the link stays perfectly healthy throughout.
        # Defaults True so a bridge that does not publish it (an older one)
        # behaves exactly as before rather than freezing everything.
        self.hand_tracked = {h: True for h in HANDS}
        self.safety_frozen = False
        self.jump = {h: [] for h in HANDS}
        self.lag = {h: 0.0 for h in HANDS}
        self.lag_max = {h: 0.0 for h in HANDS}
        self.n_resets = 0
        self.last_hold_t = {h: 0.0 for h in HANDS}

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.cmd_pub, self.st_pub = {}, {}
        for h in self.hands:
            self.create_subscription(PoseStamped, f'/vr/controller_pose_{h}',
                                     lambda m, h=h: self._on_pose(h, m), 20)
            self.create_subscription(Joy, f'/vr/controller_joy_{h}',
                                     lambda m, h=h: self._on_joy(h, m), 20)
            arm = self.arm_of[h]
            self.cmd_pub[h] = self.create_publisher(
                PoseStamped, f'/master_arm_pose_{arm}', 10)
            self.st_pub[h] = self.create_publisher(String, f'/vr/mapper_{h}', 10)
        self.create_subscription(Bool, '/vr/tracking_ok', self._on_tracking, 10)
        # HONOUR THE SAFETY NODE. vr_safety_node computed a freeze and
        # published it, and nothing consumed it -- so 'the observer e-stop was
        # withdrawn' and 'the tracking reference moved' were both states the
        # system could be in while still commanding the arm. Defaults False
        # and only ever set by a message actually received, so a stack running
        # without vr_safety_node (the recording sweep) behaves as before.
        self.create_subscription(Bool, '/vr/freeze', self._on_safety_freeze, 10)
        for h in HANDS:
            self.create_subscription(
                Bool, '/vr/controller_valid_%s' % h,
                lambda m, h=h: self._on_hand_tracked(h, bool(m.data)), 10)
        # BOTH a service and a topic on purpose. The service is what a harness
        # should call, because it gets an answer back; the topic is for a
        # scripted run that must not block on a service that may not be up.
        self.create_service(Trigger, '/vr/reset', self._srv_reset)
        self.create_subscription(Empty, '/vr/reset_request',
                                 lambda _m: self.reset('/vr/reset_request'), 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        # A HEARTBEAT WHILE DISENGAGED, at 2 Hz.
        #
        # The state topic was published ONLY inside the engaged branch, so
        # while the clutch was out the mapper said nothing at all -- and
        # "disengaged", "reset", "frozen" and "the process died" were one
        # observation. In particular `/vr/reset` could not be CHECKED: the
        # thing it clears is only visible when it is not cleared.
        #
        # It carries the same keys as the engaged message so a consumer needs
        # no second shape, with `engaged: false` and no displacement.
        self.create_timer(0.5, self._idle_state)
        self.get_logger().info(
            'vr_pose_mapper up. Publishes /master_arm_pose_<arm>, the SAME '
            'topic the mannequin path uses, so the robot side is untouched '
            'and only one input may be running at a time.')

    # --------------------------------------------------------------- reset
    # ----------------------------------------------------------- smoothing
    #: THE PARAMETERS A SMOOTHING FILTER IS BUILT FROM. Named as one tuple
    #: because they have to be re-read TOGETHER: the filters are rebuilt when
    #: any of them changes, and a list that drifts out of step with
    #: `_make_pfilt` is a slider that silently stops working.
    SMOOTHING_PARAMS = ('smoothing', 'ema_alpha', 'min_cutoff_hz', 'beta',
                        'd_cutoff_hz', 'smooth_orientation',
                        'rot_min_cutoff_hz', 'rot_beta', 'rot_d_cutoff_hz')

    def _smooth_key(self):
        """The current value of every smoothing parameter, as one key."""
        return tuple(self.get_parameter(n).value
                     for n in self.SMOOTHING_PARAMS)

    def _smooth_sync(self):
        """Rebuild the filters if any smoothing parameter has changed.

        WITHOUT THIS THE SLIDERS ARE DECORATION. These constants used to be
        read only when a filter was CONSTRUCTED, and nothing constructed one
        again after start-up, so `ros2 param set min_cutoff_hz` -- and the
        window's steadiness slider on top of it -- set a number this node had
        already copied and would never look at again. That is the same defect
        as the START REAL ARM TELEOP button that set a parameter nothing
        read: a control that reports success and changes nothing.

        Rebuilding rather than mutating is deliberate. A filter's state is
        only meaningful under the law that produced it, so a law change must
        re-seed. An unknown name is REFUSED by `smooth.make`; that refusal is
        caught here so a typo mid-session costs the operator a log line
        rather than the node, and the filters in force stay in force.
        """
        try:
            key = self._smooth_key()
        except Exception as exc:                             # noqa: BLE001
            self.get_logger().error(
                '[SMOOTH] could not read the smoothing parameters (%s); '
                'keeping the filters in force.' % exc)
            return False
        if key == getattr(self, '_smooth_cur', None):
            return False
        if key == getattr(self, '_smooth_bad', None):
            return False                    # already refused, already said so
        try:
            pos = {h: self._make_pfilt() for h in HANDS}
            rot = {h: self._make_qfilt() for h in HANDS}
        except ValueError as exc:
            self._smooth_bad = key
            self.get_logger().error(
                '[SMOOTH] %s -- keeping the filters in force. Fix the '
                'parameter and they will rebuild.' % exc)
            return False
        if getattr(self, '_smooth_cur', None) is not None:
            self.get_logger().info(
                '[SMOOTH] parameters changed -- filters re-seeded (%s)'
                % ', '.join('%s=%s' % (n, v)
                            for n, v in zip(self.SMOOTHING_PARAMS, key)))
        self.pfilt, self.qfilt = pos, rot
        self._smooth_cur = key
        self._smooth_bad = None
        return True

    def _make_pfilt(self):
        """The position smoother named by the `smoothing` parameter.

        An unknown name RAISES rather than quietly falling back: a typo
        selecting a different control law is how a session gets spent
        comparing two conditions that were secretly the same one.
        """
        kind = str(self.get_parameter('smoothing').value)
        return smooth.make(
            kind,
            alpha=float(self.get_parameter('ema_alpha').value),
            min_cutoff=float(self.get_parameter('min_cutoff_hz').value),
            beta=float(self.get_parameter('beta').value),
            d_cutoff=float(self.get_parameter('d_cutoff_hz').value))

    def _make_qfilt(self):
        if not bool(self.get_parameter('smooth_orientation').value) \
                or str(self.get_parameter('smoothing').value) == 'none':
            return None
        return smooth.OneEuroQuat(
            float(self.get_parameter('rot_min_cutoff_hz').value),
            float(self.get_parameter('rot_beta').value),
            float(self.get_parameter('rot_d_cutoff_hz').value))

    def reset(self, why='request'):
        """Return the mapper to its just-started state.

        EVERYTHING per-run, including `scale`, which the operator moves with
        the thumbstick and which therefore does NOT survive a run either. The
        value restored is the declared parameter, not whatever the last
        operator left it at.
        """
        carried = {h: (None if self.filt[h] is None or self.last_cmd[h] is None
                       else round(float(np.linalg.norm(self.lag[h])), 5))
                   for h in HANDS}
        for h in HANDS:
            self.engaged[h] = False
            self.p_ref[h] = None
            self.q_ref[h] = None
            self.p_anchor[h] = None
            self.q_anchor[h] = None
            self.filt[h] = None
            # REBUILT, not just cleared. A smoother holds the last run's
            # position, velocity estimate and adaptive cutoff; carried into a
            # second run they are wrong by however far the first run ended up.
            # This is the same class of defect as `filt` and was found the
            # same way.
            self.pfilt[h] = self._make_pfilt()
            self.qfilt[h] = self._make_qfilt()
            self.last_cmd[h] = None
            self.ctrl_hist[h] = []
            self.jump[h] = []
            self.lag[h] = 0.0
            self.lag_max[h] = 0.0
            # hand_tracked is NOT reset: it reflects what the bridge is
            # reporting right now, not per-run history, and clearing it to
            # True would let a run start on an occluded controller.
        old_scale, self.scale = self.scale, float(self.get_parameter('scale').value)
        self.n_resets += 1
        self.get_logger().info(
            'RESET (%s): clutch out, references and anchors dropped, filter '
            'cleared, scale %.3f -> %.3f. Lag carried at reset: %s'
            % (why, old_scale, self.scale, carried))
        return carried

    def _srv_reset(self, req, res):
        carried = self.reset('service')
        res.success = True
        res.message = ('mapper reset; lag discarded per hand (m): %s' % carried)
        return res

    def _on_hand_tracked(self, hand, ok):
        was = self.hand_tracked[hand]
        self.hand_tracked[hand] = ok
        if was and not ok:
            # Same reasoning as a full tracking loss, applied to one arm:
            # hold the clutch across an occlusion and the reference is latched
            # against a hand that has since moved, so on regain the arm sweeps
            # to catch up at max_speed_mps -- a motion nobody commanded.
            if self.engaged[hand]:
                self.get_logger().error(
                    '[%s] controller OCCLUDED / emulated - clutch dropped, '
                    'this arm freezes. Re-grip in view to re-latch.' % hand)
            self.engaged[hand] = False
            self.filt[hand] = None
            self.pfilt[hand].reset()
            if self.qfilt[hand] is not None:
                self.qfilt[hand].reset()

    def _on_safety_freeze(self, m):
        was = self.safety_frozen
        self.safety_frozen = bool(m.data)
        if self.safety_frozen and not was:
            for h in HANDS:
                if self.engaged[h]:
                    self.get_logger().error(
                        '[%s] safety FREEZE - clutch dropped. The arm holds '
                        'its last pose; re-grip once the cause is cleared.' % h)
                self.engaged[h] = False
                self.filt[h] = None
                self.pfilt[h].reset()
                if self.qfilt[h] is not None:
                    self.qfilt[h].reset()

    def _on_tracking(self, m):
        was = self.tracking_ok
        self.tracking_ok = bool(m.data)
        if was and not self.tracking_ok:
            # DISENGAGE, do not merely stop publishing. Holding the clutch in
            # across a dropout means resuming from a filter latched before the
            # loss against a controller that has since moved, and the clutch's
            # 'zero jump by construction' guarantee is void for exactly the
            # motion nobody saw. Dropping the clutch costs a re-grip -- which
            # _on_joy retries automatically while the grip is still held -- and
            # buys back a re-latch, so the jump is zero again.
            for h in HANDS:
                if self.engaged[h]:
                    self.get_logger().error(
                        '[%s] tracking LOST while engaged - clutch dropped. '
                        'Re-grip to re-latch; the arm holds its last pose.' % h)
                self.engaged[h] = False
                self.filt[h] = None
                self.pfilt[h].reset()
                if self.qfilt[h] is not None:
                    self.qfilt[h].reset()

    # ------------------------------------------------------------ callbacks
    def _on_pose(self, hand, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        q = q_norm([m.pose.orientation.x, m.pose.orientation.y,
                    m.pose.orientation.z, m.pose.orientation.w])
        self.ctrl[hand] = (p, q)
        t = self.get_clock().now().nanoseconds * 1e-9
        self.ctrl_hist[hand].append((t, p))
        cut = t - float(self.get_parameter('quiet_window_s').value)
        self.ctrl_hist[hand] = [(tt, pp) for tt, pp in self.ctrl_hist[hand] if tt >= cut]

    def _on_joy(self, hand, m):
        prev = self.joy[hand]
        self.joy[hand] = m
        if not m.axes:
            return
        grip = m.axes[1]
        was = (prev.axes[1] > 0.6) if (prev and len(prev.axes) > 1) else False
        now = grip > 0.6
        self.grip_held[hand] = bool(now)
        if now and not self.engaged[hand]:
            # RETRY while the grip is HELD, rather than only on the rising
            # edge. Engage can legitimately be refused - no pose yet, no robot
            # TF, or the hand still moving - and consuming the edge on a
            # refusal means the operator must release and re-grip for reasons
            # they cannot see. Observed exactly that: the first Joy message
            # arrived before the first pose, the edge was consumed, and the
            # clutch never engaged again.
            self._engage(hand)
        elif was and not now:
            self.engage_refused[hand] = None
            self._disengage(hand)

    # --------------------------------------------------------------- clutch
    def _quiet(self, hand):
        """Is the controller quasi-static? Latching a MOVING reference is the
        classic source of a re-engage jump."""
        h = self.ctrl_hist[hand]
        if len(h) < 3:
            return False
        pts = np.array([p for _, p in h])
        return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))) < \
            float(self.get_parameter('quiet_engage_m').value)

    def _robot_pose(self, arm):
        try:
            t = self.tf_buf.lookup_transform('world', f'{arm}_end_effector_link',
                                             rclpy.time.Time())
        except Exception:
            return None, None
        v, r = t.transform.translation, t.transform.rotation
        return (np.array([v.x, v.y, v.z]), q_norm([r.x, r.y, r.z, r.w]))

    def _engage(self, hand):
        if self.ctrl[hand] is None:
            self.engage_refused[hand] = 'no controller pose yet'
            self.get_logger().warn('[%s] engage ignored: no controller pose' % hand)
            return
        if self.safety_frozen:
            self.engage_refused[hand] = 'safety freeze held'
            self.get_logger().warn(
                '[%s] engage REFUSED: the safety node is holding a freeze.'
                % hand)
            return
        if not self.hand_tracked[hand]:
            self.engage_refused[hand] = 'controller not tracked'
            self.get_logger().warn(
                '[%s] engage REFUSED: the controller is not tracked. Latching '
                'a reference off an emulated pose anchors the whole run to a '
                'position the runtime invented.' % hand)
            return
        if not self._quiet(hand):
            self.engage_refused[hand] = 'hand moving at engage - hold still'
            self.get_logger().warn(
                '[%s] engage while the controller is MOVING - the reference '
                'would be latched off a moving hand. Hold still and re-grip.'
                % hand)
            return
        arm = self.arm_of[hand]
        pr, qr = self._robot_pose(arm)
        if pr is None:
            self.engage_refused[hand] = 'no robot TF for this arm'
            self.get_logger().warn('[%s] engage ignored: no robot TF' % hand)
            return
        p, q = self.ctrl[hand]
        self.p_ref[hand] = p.copy()
        self.q_ref[hand] = q.copy()
        self.p_anchor[hand] = pr.copy()
        self.q_anchor[hand] = qr.copy()
        # EMA reset. Without it, pre-disengage state bleeds into the first
        # frames after engage and shows up as a small jump.
        self.filt[hand] = None
        self.engaged[hand] = True
        self.engage_refused[hand] = None
        # WHAT THIS NUMBER ACTUALLY IS, because it was reported under the
        # wrong name. The command at engage is set to `pr`, the arm's OWN
        # current pose read from TF, so the jump the ARM experiences is zero
        # by construction -- there is nothing to measure there.
        #
        # `last_cmd - pr` is a different quantity entirely: the distance
        # between what was last COMMANDED and where the arm actually GOT TO.
        # That is the FOLLOWER'S TRACKING ERROR at the moment of engage. It is
        # worth recording -- a large value means the arm is a long way behind
        # its target -- but calling it a "re-engage jump" would put a number
        # in the write-up that says the clutch is bad when the clutch is fine
        # and the follower is lagging. Measured 2026-08-19: 150 mm of it, on a
        # run whose actual arm motion across the engage was zero.
        if self.last_cmd[hand] is not None:
            self.jump[hand].append(float(np.linalg.norm(self.last_cmd[hand] - pr)))
        self.get_logger().info('[%s] CLUTCH ENGAGED, anchored at %s'
                               % (hand, np.round(pr, 4)))

    def _disengage(self, hand):
        if self.engaged[hand]:
            self.engaged[hand] = False
            self.get_logger().info('[%s] CLUTCH RELEASED - robot frozen' % hand)

    # ----------------------------------------------------------------- loop
    def _idle_state(self):
        """Say what the mapper is, while it is not driving anything."""
        for hand in self.hands:
            if self.engaged[hand]:
                continue                      # the engaged path is richer
            try:
                yaw = float(self.get_parameter('align_yaw_deg').value)
            except Exception:                                 # noqa: BLE001
                yaw = 0.0
            self.st_pub[hand].publish(String(data=json.dumps(dict(
                hand=hand, arm=self.arm_of[hand], engaged=False,
                scale=round(self.scale, 3),
                # WHAT A CLEAN SCALE ACTUALLY IS, published rather than left
                # for a consumer to assume. The declared default is 0.5, not
                # 1.0, and a bring-up check that assumed 1.0 reported a
                # freshly reset mapper as "still holding settings from an
                # earlier run" -- for ever, because the fix it offered was
                # the reset that had just produced the value it objected to.
                scale_default=round(
                    float(self.get_parameter('scale').value), 3),
                has_reference=self.p_ref[hand] is not None,
                has_anchor=self.p_anchor[hand] is not None,
                filter_primed=self.filt[hand] is not None,
                hand_tracked=self.hand_tracked[hand],
                tracking_ok=self.tracking_ok,
                safety_frozen=self.safety_frozen,
                align_yaw_deg=round(yaw, 2),
                resets=self.n_resets,
                grip_held=self.grip_held[hand],
                engage_refused=self.engage_refused[hand],
                # Whether this hand's arm topic is being kept alive with
                # hold-in-place commands (the dead-man feed). False means an
                # idle hand STARVES /master_arm_pose_<arm> on a real run.
                feeding_deadman=bool(
                    self.get_parameter('hold_when_idle').value)))))

    def _publish_hold(self, hand):
        """Command the arm's own live pose: zero motion by construction.

        This is what keeps /master_arm_pose_<arm> a live source for the
        e-stop dead-man while the clutch is out or a freeze stands. Silence
        here does not stop the arm any harder than a hold does -- the
        follower holds its last command either way -- but silence LATCHES
        the e-stop, and the latch outlives the freeze that caused it.
        """
        if not bool(self.get_parameter('hold_when_idle').value):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_hold_t[hand] < 1.0 / max(
                1e-3, float(self.get_parameter('hold_rate_hz').value)):
            return
        arm = self.arm_of[hand]
        p, q = self._robot_pose(arm)
        if p is None:
            return                       # no TF yet: nothing true to say
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'world'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = map(float, q)
        self.cmd_pub[hand].publish(m)
        self.last_hold_t[hand] = now

    def _tick(self):
        dt = 1.0 / float(self.get_parameter('rate_hz').value)
        for hand in self.hands:
            j = self.joy[hand]
            if j is not None and len(j.axes) > 3 and abs(j.axes[3]) > 0.5:
                step = (float(self.get_parameter('scale_step_per_s').value)
                        * j.axes[3] * dt)
                self._set_scale(hand, self.scale + step)
            if not self.engaged[hand] or self.ctrl[hand] is None:
                self._publish_hold(hand)
                continue
            if self.safety_frozen or not self.tracking_ok \
                    or not self.hand_tracked[hand]:
                # Freeze. The arm must not MOVE -- so command where it
                # already is, rather than going silent and feeding the
                # dead-man latch. See _publish_hold.
                self.get_logger().error(
                    '[%s] %s - FROZEN (commanding hold-in-place)'
                    % (hand, 'safety freeze' if self.safety_frozen
                       else 'link lost' if not self.tracking_ok
                       else 'controller not tracked'),
                    throttle_duration_sec=1.0)
                self._publish_hold(hand)
                continue
            p, q = self.ctrl[hand]
            d = p - self.p_ref[hand]
            # R_align: the operator's frame -> the robot's frame.
            yaw = float(self.get_parameter('align_yaw_deg').value)
            R = yaw_matrix(yaw)
            p_cmd = self.p_anchor[hand] + self.scale * (R @ d)
            # Live: the smoothing sliders take effect on the next sample,
            # not on the next run.
            self._smooth_sync()
            if self.filt[hand] is None:
                self.filt[hand] = p_cmd.copy()
                self.pfilt[hand].reset()
                self.pfilt[hand](p_cmd, dt)
            else:
                # SMOOTH FIRST, THEN RATE LIMIT. The order matters and it is
                # not interchangeable: rate-limiting a noisy signal makes the
                # limiter fire on tremor, and a limiter that has fired cannot
                # give the distance back while the motion continues -- that is
                # the accumulating `lag_m`. Filtering first means the limiter
                # only ever sees smooth motion, so it fires on genuinely fast
                # hands and nothing else.
                nxt = np.asarray(self.pfilt[hand](p_cmd, dt), float)
                step = nxt - self.filt[hand]
                mx = float(self.get_parameter('max_speed_mps').value) * dt
                n = float(np.linalg.norm(step))
                if n > mx:
                    step *= mx / n
                self.filt[hand] = self.filt[hand] + step
            p_out = self.filt[hand]
            # THE ACCUMULATING MISS, MADE VISIBLE. This is the distance
            # between what the operator's hand asks for and what is actually
            # commanded. A rate limiter cannot give it back while the motion
            # continues, so it grows -- which is why 02's cube misses grew by
            # 27, 41 and 50 mm instead of repeating one number.
            self.lag[hand] = float(np.linalg.norm(p_cmd - p_out))
            self.lag_max[hand] = max(self.lag_max[hand], self.lag[hand])
            if self.lag[hand] > 0.030:
                self.get_logger().warn(
                    '[%s] command LAGS the hand by %.1f mm (rate limit %.2f '
                    'm/s). This is the miss that accumulates.'
                    % (hand, self.lag[hand] * 1000.0,
                       float(self.get_parameter('max_speed_mps').value)),
                    throttle_duration_sec=2.0)

            if bool(self.get_parameter('command_orientation').value):
                dq = q_mul(q, q_conj(self.q_ref[hand]))
                # The SAME alignment must reach the orientation, conjugated:
                # a rotation expressed in the operator's frame becomes
                # qz * dq * qz^-1 in the robot's. Rotating the position and
                # not the orientation puts them in two different frames, and
                # IK is then asked for a pose that does not exist.
                if abs(yaw) > 1e-9:
                    qz = yaw_quat(yaw)
                    dq = q_mul(q_mul(qz, dq), q_conj(qz))
                q_out = q_norm(q_mul(dq, self.q_anchor[hand]))
                # SMOOTHED, WHICH IT NEVER WAS. Until 2026-08-26 this line
                # published the controller's rotation raw while position went
                # through a low pass, so wrist tremor reached the arm
                # unattenuated -- and the wrist is what the pads hang off, so
                # 0.74 deg of hand tremor is ~1.4 mm at the fingertips plus
                # whatever chatter it induces in the IK.
                #
                # Slerped, not component-averaged, and sign-canonicalised
                # against the previous output: q and -q are the same rotation,
                # and a filter that misses that interpolates the long way
                # round and swings the wrist through 360 deg on an input that
                # was perfectly valid.
                if self.qfilt[hand] is not None:
                    q_out = q_norm(self.qfilt[hand](q_out, dt))
            else:
                q_out = self.q_anchor[hand]

            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'world'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_out)
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = map(float, q_out)
            self.cmd_pub[hand].publish(m)
            self.last_cmd[hand] = p_out.copy()

            s = String()
            s.data = json.dumps(dict(
                hand=hand, arm=self.arm_of[hand], engaged=True,
                scale=round(self.scale, 3),
                orientation_commanded=bool(
                    self.get_parameter('command_orientation').value),
                displacement_m=round(float(np.linalg.norm(d)), 4),
                # RENAMED. These are the follower's tracking error at the
                # moment of engage, NOT the jump the arm makes. The arm's
                # jump is zero by construction; see _engage().
                mean_follower_lag_at_engage_m=(
                    round(float(np.mean(self.jump[hand])), 5)
                    if self.jump[hand] else None),
                max_follower_lag_at_engage_m=(
                    round(float(np.max(self.jump[hand])), 5)
                    if self.jump[hand] else None),
                reengage_jump_m=0.0,
                n_engages=len(self.jump[hand]),
                hand_tracked=self.hand_tracked[hand],
                grip_held=self.grip_held[hand],
                engage_refused=None,
                align_yaw_deg=round(yaw, 2),
                safety_frozen=self.safety_frozen,
                lag_m=round(self.lag[hand], 5),
                max_lag_m=round(self.lag_max[hand], 5),
                # WHAT THE SMOOTHER IS DOING RIGHT NOW. The adaptive cutoff is
                # the whole mechanism, so leaving it invisible would make
                # "feels laggy" and "feels jittery" unfalsifiable in exactly
                # the way the old fixed alpha was: `cutoff_hz` near
                # min_cutoff means it is treating the hand as still, and high
                # means it is tracking a fast reach.
                smoothing=str(self.get_parameter('smoothing').value),
                cutoff_hz=round(float(getattr(self.pfilt[hand], 'cutoff',
                                              float('nan'))), 2),
                hand_speed_mps=round(float(getattr(self.pfilt[hand], 'speed',
                                                   float('nan'))), 4),
                rot_smoothed=self.qfilt[hand] is not None,
                rot_cutoff_hz=(None if self.qfilt[hand] is None else
                               round(float(self.qfilt[hand].cutoff), 2)),
                resets=self.n_resets))
            self.st_pub[hand].publish(s)

    def _set_scale(self, hand, new):
        lo = float(self.get_parameter('scale_min').value)
        hi = float(self.get_parameter('scale_max').value)
        new = max(lo, min(hi, new))
        if abs(new - self.scale) < 1e-6:
            return
        # RE-BASE so the commanded pose is continuous. Without this the arm
        # jumps by (new-old) x displacement, worst exactly when the operator
        # is far from the engage point.
        R = yaw_matrix(float(self.get_parameter('align_yaw_deg').value))
        for h in self.hands:
            if self.engaged[h] and self.ctrl[h] is not None:
                d = R @ (self.ctrl[h][0] - self.p_ref[h])
                self.p_anchor[h] = self.p_anchor[h] + (self.scale - new) * d
        self.scale = new


def main():
    rclpy.init()
    n = VrPoseMapper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
