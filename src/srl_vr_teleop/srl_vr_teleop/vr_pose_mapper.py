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
        self.declare_parameter('scale', 0.5)
        self.declare_parameter('scale_min', 0.1)
        self.declare_parameter('scale_max', 2.0)
        # Thumbstick y adjusts scale live; VR's play space is much larger than
        # the robot's workspace, so scaling matters more here than for the
        # mannequin, where 1:1 was the right default.
        self.declare_parameter('scale_step_per_s', 0.5)
        self.declare_parameter('command_orientation', True)
        # First-order low pass on the command. The Quest is a high-rate, low-
        # noise source, so this is much lighter than the mannequin's 0.3 EMA.
        self.declare_parameter('ema_alpha', 0.6)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('max_speed_mps', 0.35)
        self.declare_parameter('quiet_engage_m', 0.003)
        self.declare_parameter('quiet_window_s', 0.10)

        self.hands = list(self.get_parameter('hands').value)
        self.arm_of = {'left': self.get_parameter('left_controller_arm').value,
                       'right': self.get_parameter('right_controller_arm').value}
        self.scale = float(self.get_parameter('scale').value)
        self.alpha = float(self.get_parameter('ema_alpha').value)

        self.ctrl = {h: None for h in HANDS}
        self.ctrl_hist = {h: [] for h in HANDS}
        self.joy = {h: None for h in HANDS}
        self.engaged = {h: False for h in HANDS}
        self.p_ref = {h: None for h in HANDS}
        self.q_ref = {h: None for h in HANDS}
        self.p_anchor = {h: None for h in HANDS}
        self.q_anchor = {h: None for h in HANDS}
        self.filt = {h: None for h in HANDS}
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
        self.jump = {h: [] for h in HANDS}
        self.lag = {h: 0.0 for h in HANDS}
        self.lag_max = {h: 0.0 for h in HANDS}
        self.n_resets = 0

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
        self.get_logger().info(
            'vr_pose_mapper up. Publishes /master_arm_pose_<arm>, the SAME '
            'topic the mannequin path uses, so the robot side is untouched '
            'and only one input may be running at a time.')

    # --------------------------------------------------------------- reset
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
            self.get_logger().warn('[%s] engage ignored: no controller pose' % hand)
            return
        if not self.hand_tracked[hand]:
            self.get_logger().warn(
                '[%s] engage REFUSED: the controller is not tracked. Latching '
                'a reference off an emulated pose anchors the whole run to a '
                'position the runtime invented.' % hand)
            return
        if not self._quiet(hand):
            self.get_logger().warn(
                '[%s] engage while the controller is MOVING - the reference '
                'would be latched off a moving hand. Hold still and re-grip.'
                % hand)
            return
        arm = self.arm_of[hand]
        pr, qr = self._robot_pose(arm)
        if pr is None:
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
        # The command at engage is EXACTLY the current robot pose, so the jump
        # is zero by construction. Recorded anyway, because "by construction"
        # has been wrong before.
        if self.last_cmd[hand] is not None:
            self.jump[hand].append(float(np.linalg.norm(self.last_cmd[hand] - pr)))
        self.get_logger().info('[%s] CLUTCH ENGAGED, anchored at %s'
                               % (hand, np.round(pr, 4)))

    def _disengage(self, hand):
        if self.engaged[hand]:
            self.engaged[hand] = False
            self.get_logger().info('[%s] CLUTCH RELEASED - robot frozen' % hand)

    # ----------------------------------------------------------------- loop
    def _tick(self):
        dt = 1.0 / float(self.get_parameter('rate_hz').value)
        for hand in self.hands:
            j = self.joy[hand]
            if j is not None and len(j.axes) > 3 and abs(j.axes[3]) > 0.5:
                step = (float(self.get_parameter('scale_step_per_s').value)
                        * j.axes[3] * dt)
                self._set_scale(hand, self.scale + step)
            if not self.engaged[hand] or self.ctrl[hand] is None:
                continue
            if not self.tracking_ok or not self.hand_tracked[hand]:
                # Freeze. Publishing nothing is the correct behaviour: the
                # follower holds its last command.
                self.get_logger().error(
                    '[%s] %s - FREEZING (publishing no command)'
                    % (hand, 'link lost' if not self.tracking_ok
                       else 'controller not tracked'),
                    throttle_duration_sec=1.0)
                continue
            p, q = self.ctrl[hand]
            d = p - self.p_ref[hand]
            p_cmd = self.p_anchor[hand] + self.scale * d
            if self.filt[hand] is None:
                self.filt[hand] = p_cmd.copy()
            else:
                a = self.alpha
                nxt = a * p_cmd + (1 - a) * self.filt[hand]
                # speed limit, same spirit as the follower's max_vel
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
                q_out = q_norm(q_mul(dq, self.q_anchor[hand]))
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
                mean_reengage_jump_m=(round(float(np.mean(self.jump[hand])), 5)
                                      if self.jump[hand] else None),
                max_reengage_jump_m=(round(float(np.max(self.jump[hand])), 5)
                                     if self.jump[hand] else None),
                n_engages=len(self.jump[hand]),
                hand_tracked=self.hand_tracked[hand],
                lag_m=round(self.lag[hand], 5),
                max_lag_m=round(self.lag_max[hand], 5),
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
        for h in self.hands:
            if self.engaged[h] and self.ctrl[h] is not None:
                d = self.ctrl[h][0] - self.p_ref[h]
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
