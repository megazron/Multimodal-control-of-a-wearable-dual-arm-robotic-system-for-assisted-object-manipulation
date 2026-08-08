#!/usr/bin/env python3
"""
vr_handover_arbiter.py — assistance for an input device that is NOT
DOF-deficient.

THE DESIGN QUESTION, AND HOW IT IS RESOLVED.

On the mannequin, the autonomy supplies wrist ORIENTATION, because the master
physically cannot measure it: gravity gives roll and pitch but never yaw, j7 is
railed, j5/j6 are dead. Assistance there RECOVERS A LOST CAPABILITY — without
it the pose is simply unreachable.

On VR the operator already has full 6-DOF at 72 Hz. Supplying orientation would
be supplying something they already have, and worse, it would FIGHT them: the
operator commands a wrist angle, the autonomy overrides it, and the operator
feels the device stop obeying. That is the single most likely way to make
assistance feel bad.

So the VR arbiter assists differently, and the difference is the scientifically
interesting part:

    mannequin   ASSIST supplies the orientation DOFs the input cannot express.
                Benefit = capability recovered. Measured by E4 as a CATEGORICAL
                result: impossible becomes possible.

    VR          ASSIST supplies PRECISION and reduces WORKLOAD. The operator's
                pose is snapped onto the nearest VALIDATED grasp pose once they
                are inside a threshold, correcting the last centimetre and the
                last few degrees. Benefit = fewer failed attempts, less
                fine-positioning time, lower workload. Measured as a
                CONTINUOUS improvement, not a categorical one.

Concretely, in ASSIST this node blends the operator's pose toward the validated
grasp with a weight that grows as they get CLOSER — a "funnel" rather than a
takeover:

    w = clamp((d_engage - d) / (d_engage - d_full), 0, 1)
    p_cmd = (1-w) p_operator + w p_grasp
    q_cmd = slerp(q_operator, q_grasp, w)

At the engage distance w = 0 and the operator has full authority; only at
`snap_full_m` (a few centimetres) does the autonomy dominate. There is no
discontinuity anywhere, and backing off returns authority smoothly.

WHY THIS IS STILL "DISCRETE AND OPERATOR-CONFIRMED", not continuous blending in
the sense the mannequin design rejects: the STATE is discrete and visible
(DIRECT / ASSIST / GRASPED), entry requires the same P-threshold and proximity
test, and it is cancellable at any moment. What varies continuously inside
ASSIST is only how much of the final centimetre the autonomy contributes, which
the operator can see rendered in the headset. The mannequin path blends nothing
because it has nothing to blend — the operator cannot express orientation at
all.

THE GRIPPER IS NEVER CLOSED BY THIS NODE. Same rule as everywhere.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

DIRECT, ASSIST, GRASPED = 'DIRECT', 'ASSIST', 'GRASPED'


def slerp(q0, q1, t):
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(max(-1.0, min(1.0, d)))
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * math.cos(th0 * t) + q2 * math.sin(th0 * t)


class VrHandoverArbiter(Node):
    def __init__(self):
        super().__init__('vr_handover_arbiter')
        self.declare_parameter('arms', ['left', 'right'])
        self.declare_parameter('p_threshold', 0.60)
        self.declare_parameter('engage_distance_m', 0.20)
        self.declare_parameter('snap_full_m', 0.03)
        self.declare_parameter('cancel_distance_m', 0.30)
        self.declare_parameter('gripper_closed_rad', 0.55)
        self.declare_parameter('rate_hz', 60.0)
        # If the operator is actively rotating the controller, back off: they
        # are expressing an orientation intention and the assist must not
        # fight it. Unique to VR - the mannequin operator cannot do this.
        self.declare_parameter('operator_override_rad_s', 1.2)

        self.arms = list(self.get_parameter('arms').value)
        self.state = {a: DIRECT for a in self.arms}
        self.op_pose = {a: None for a in self.arms}
        self.grasp = {a: None for a in self.arms}
        self.intent = {a: None for a in self.arms}
        self.gripper = {a: 0.0 for a in self.arms}
        self.grip_at_assist = {a: None for a in self.arms}
        self.w = {a: 0.0 for a in self.arms}
        self.prev_q = {a: None for a in self.arms}
        self.omega = {a: 0.0 for a in self.arms}
        self.reason = {a: 'startup' for a in self.arms}
        self.cancelled = {a: False for a in self.arms}

        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.cmd_pub, self.state_pub, self.dbg_pub = {}, {}, {}
        for a in self.arms:
            self.create_subscription(PoseStamped, f'/master_arm_pose_{a}',
                                     lambda m, a=a: self.op_pose.__setitem__(a, m), 20)
            self.create_subscription(PoseStamped, f'/autonomy/grasp_{a}',
                                     lambda m, a=a: self.grasp.__setitem__(a, m), 10)
            self.create_subscription(String, f'/autonomy/intent_debug_{a}',
                                     lambda m, a=a: self._on_intent(a, m), 10)
            self.create_subscription(Bool, f'/autonomy/cancel_{a}',
                                     lambda m, a=a: self._on_cancel(a, m), 10)
            self.cmd_pub[a] = self.create_publisher(
                PoseStamped, f'/vr/assisted_pose_{a}', 10)
            self.state_pub[a] = self.create_publisher(String, f'/autonomy/state_{a}', 10)
            self.dbg_pub[a] = self.create_publisher(String, f'/vr/arbiter_{a}', 10)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            'vr_handover_arbiter up. VR assistance supplies PRECISION, not '
            'missing DOFs - the operator already has full 6-DOF. See the '
            'module docstring for why that differs from the mannequin path.')

    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        for a in self.arms:
            v = d.get(f'{a}_robotiq_85_left_knuckle_joint')
            if v is not None:
                self.gripper[a] = float(v)

    def _on_intent(self, arm, m):
        try:
            self.intent[arm] = json.loads(m.data)
        except ValueError:
            self.intent[arm] = None

    def _on_cancel(self, arm, m):
        self.cancelled[arm] = bool(m.data)
        if m.data and self.state[arm] == ASSIST:
            self._to_direct(arm, 'operator cancel')

    def _to_direct(self, arm, why):
        self.state[arm] = DIRECT
        self.w[arm] = 0.0
        self.grip_at_assist[arm] = None
        self.reason[arm] = why
        self.get_logger().info('[%s] ASSIST -> DIRECT: %s' % (arm, why))

    @staticmethod
    def _pq(msg):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        return p, q

    def _tick(self):
        p_thr = float(self.get_parameter('p_threshold').value)
        d_eng = float(self.get_parameter('engage_distance_m').value)
        d_full = float(self.get_parameter('snap_full_m').value)
        d_cancel = float(self.get_parameter('cancel_distance_m').value)
        closed = float(self.get_parameter('gripper_closed_rad').value)
        om_max = float(self.get_parameter('operator_override_rad_s').value)

        for arm in self.arms:
            if self.op_pose[arm] is None:
                continue
            po, qo = self._pq(self.op_pose[arm])
            if self.prev_q[arm] is not None:
                dq = abs(1.0 - abs(float(np.dot(qo, self.prev_q[arm]))))
                self.omega[arm] = math.sqrt(max(0.0, dq)) * 2.0 / self.dt
            self.prev_q[arm] = qo.copy()

            g = self.grasp[arm]
            it = self.intent[arm] or {}
            d = None
            if g is not None:
                pg, qg = self._pq(g)
                d = float(np.linalg.norm(po - pg))

            st = self.state[arm]
            if st == DIRECT:
                if (g is not None and d is not None and d < d_eng
                        and float(it.get('top_p', 0.0)) > p_thr
                        and not bool(it.get('ambiguous', True))
                        and not self.cancelled[arm]):
                    self.state[arm] = ASSIST
                    self.grip_at_assist[arm] = self.gripper[arm]
                    self.reason[arm] = ('P=%.2f, d=%.3f < %.3f'
                                        % (it.get('top_p', 0), d, d_eng))
                    self.get_logger().info('[%s] DIRECT -> ASSIST: %s'
                                           % (arm, self.reason[arm]))
                else:
                    self.reason[arm] = 'waiting'
            elif st == ASSIST:
                if g is None:
                    self._to_direct(arm, 'grasp withdrawn')
                elif d is not None and d > d_cancel:
                    self._to_direct(arm, 'moved away (d=%.3f)' % d)
                elif bool(it.get('ambiguous', False)):
                    self._to_direct(arm, 'intent became ambiguous')
                elif self.omega[arm] > om_max:
                    # UNIQUE TO VR. The operator is rotating the controller,
                    # i.e. deliberately commanding orientation. Back off rather
                    # than fight - they can express it, so let them.
                    self._to_direct(arm, 'operator is commanding orientation '
                                         '(%.2f rad/s) - assist yields'
                                         % self.omega[arm])
                else:
                    if self.grip_at_assist[arm] is None:
                        self.grip_at_assist[arm] = self.gripper[arm]
                    if (self.gripper[arm] > closed and
                            self.gripper[arm] > self.grip_at_assist[arm] + 0.05):
                        self.state[arm] = GRASPED
                        self.reason[arm] = 'gripper closed by the OPERATOR'
            elif st == GRASPED:
                if self.gripper[arm] < closed * 0.5:
                    self._to_direct(arm, 'gripper released')

            # ---- the funnel ----
            out = PoseStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'world'
            if self.state[arm] in (ASSIST, GRASPED) and g is not None:
                pg, qg = self._pq(g)
                w = 0.0 if d is None else \
                    max(0.0, min(1.0, (d_eng - d) / max(1e-6, d_eng - d_full)))
                self.w[arm] = w
                p_cmd = (1 - w) * po + w * pg
                q_cmd = slerp(qo, qg, w)
            else:
                self.w[arm] = 0.0
                p_cmd, q_cmd = po, qo
            out.pose.position.x, out.pose.position.y, out.pose.position.z = \
                (float(v) for v in p_cmd)
            (out.pose.orientation.x, out.pose.orientation.y,
             out.pose.orientation.z, out.pose.orientation.w) = (float(v) for v in q_cmd)
            self.cmd_pub[arm].publish(out)

            s = String()
            s.data = self.state[arm]
            self.state_pub[arm].publish(s)
            dbg = String()
            dbg.data = json.dumps(dict(
                arm=arm, state=self.state[arm], funnel_weight=round(self.w[arm], 3),
                distance_m=(round(d, 4) if d is not None else None),
                reason=self.reason[arm], operator_omega_rad_s=round(self.omega[arm], 3),
                assistance_kind='precision_and_workload',
                note='VR: the operator already has 6-DOF; assist refines the '
                     'final approach rather than supplying missing DOFs'))
            self.dbg_pub[arm].publish(dbg)


def main():
    rclpy.init()
    n = VrHandoverArbiter()
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
