#!/usr/bin/env python3
"""
handover_arbiter.py — DIRECT / ASSIST / GRASPED. Discrete, operator-confirmed.

NOT continuous blending. Blending an autonomous velocity into the operator's
command means the operator can never tell whose motion they are watching, and
the literature on shared control is equivocal about whether it even helps.
This is a discrete handover with a visible state, and the operator keeps FULL
POSITION CONTROL in every state.

The contract, in full:

  DIRECT   -> ASSIST   when P(object) > `p_threshold` AND the end effector is
                       within `distance_threshold_m` of the grasp. Both, not
                       either. And never while the intent estimate reports
                       `ambiguous`, because assisting toward the wrong one of
                       two adjacent objects is worse than not assisting.
  ASSIST               wrist ORIENTATION servos to the grasp orientation over
                       `servo_time_s` (~0.5 s), SLERP, never a snap. POSITION
                       stays 100% the operator's, always.
  gripper              closes ONLY on the operator's FSR command. There is no
                       code path in this file that closes it.
  cancel               moving away past `cancel_distance_m`, or the cancel
                       button. Immediate return to DIRECT with NO JUMP: the
                       orientation blend is simply frozen where it is and
                       released, so the commanded pose is continuous.
  ASSIST   -> GRASPED  when the gripper reports closed on the object.

Every threshold is a live ROS parameter; E5 sets them empirically.

Publishes:
  /autonomy/state_<arm>        DIRECT | ASSIST | GRASPED
  /autonomy/assist_pose_<arm>  the pose the follower should track
  /autonomy/arbiter_<arm>      JSON with why the state is what it is
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

DIRECT, ASSIST, GRASPED = "DIRECT", "ASSIST", "GRASPED"


def slerp(q0, q1, t):
    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0.0:                      # take the short way round
        q1 = -q1
        d = -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    th0 = math.acos(max(-1.0, min(1.0, d)))
    th = th0 * t
    q2 = q1 - q0 * d
    q2 /= np.linalg.norm(q2)
    return q0 * math.cos(th) + q2 * math.sin(th)


class HandoverArbiter(Node):
    def __init__(self):
        super().__init__("handover_arbiter")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("p_threshold", 0.60)
        self.declare_parameter("distance_threshold_m", 0.25)
        self.declare_parameter("cancel_distance_m", 0.35)
        self.declare_parameter("servo_time_s", 0.5)
        self.declare_parameter("gripper_closed_rad", 0.55)
        self.declare_parameter("rate_hz", 50.0)

        self.arms = list(self.get_parameter("arms").value)
        self.state = {a: DIRECT for a in self.arms}
        self.master = {a: None for a in self.arms}
        self.grasp = {a: None for a in self.arms}
        self.intent = {a: None for a in self.arms}
        self.blend = {a: 0.0 for a in self.arms}
        self.q_at_engage = {a: None for a in self.arms}
        self.q_held = {a: None for a in self.arms}
        self.gripper = {a: 0.0 for a in self.arms}
        self.gripper_at_assist = {a: None for a in self.arms}
        self.cancelled = {a: False for a in self.arms}
        self.reason = {a: "startup" for a in self.arms}

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.state_pub, self.pose_pub, self.dbg_pub = {}, {}, {}
        for a in self.arms:
            self.create_subscription(PoseStamped, f"/master_arm_pose_{a}",
                                     lambda m, a=a: self.master.__setitem__(a, m), 10)
            self.create_subscription(PoseStamped, f"/autonomy/grasp_{a}",
                                     lambda m, a=a: self.grasp.__setitem__(a, m), 10)
            self.create_subscription(String, f"/autonomy/intent_debug_{a}",
                                     lambda m, a=a: self._on_intent(a, m), 10)
            self.create_subscription(Bool, f"/autonomy/cancel_{a}",
                                     lambda m, a=a: self._on_cancel(a, m), 10)
            self.state_pub[a] = self.create_publisher(String, f"/autonomy/state_{a}", 10)
            self.pose_pub[a] = self.create_publisher(
                PoseStamped, f"/autonomy/assist_pose_{a}", 10)
            self.dbg_pub[a] = self.create_publisher(String, f"/autonomy/arbiter_{a}", 10)
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            "handover_arbiter up. Discrete handover only: the operator keeps "
            "full POSITION control in every state, and the gripper is never "
            "closed by this node.")

    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        for a in self.arms:
            v = d.get(f"{a}_robotiq_85_left_knuckle_joint")
            if v is not None:
                self.gripper[a] = float(v)

    def _on_intent(self, arm, msg):
        try:
            self.intent[arm] = json.loads(msg.data)
        except ValueError:
            self.intent[arm] = None

    def _on_cancel(self, arm, msg):
        if msg.data and self.state[arm] == ASSIST:
            self._to_direct(arm, "operator cancel button")
        self.cancelled[arm] = bool(msg.data)

    def _to_direct(self, arm, why):
        # NO JUMP. The blended orientation is frozen where it is and simply
        # released; the commanded pose is continuous across the transition.
        # Snapping back to the master's raw orientation here is the single
        # most likely way to make a cancel feel dangerous.
        self.q_held[arm] = self._blended_quat(arm)
        self.state[arm] = DIRECT
        self.blend[arm] = 0.0
        self.reason[arm] = why
        self.get_logger().info("[%s] ASSIST -> DIRECT: %s" % (arm, why))

    def _master_quat(self, arm):
        m = self.master[arm]
        if m is None:
            return None
        o = m.pose.orientation
        return np.array([o.x, o.y, o.z, o.w])

    def _grasp_quat(self, arm):
        g = self.grasp[arm]
        if g is None:
            return None
        o = g.pose.orientation
        return np.array([o.x, o.y, o.z, o.w])

    def _blended_quat(self, arm):
        q0 = self.q_at_engage[arm]
        q1 = self._grasp_quat(arm)
        if q0 is None or q1 is None:
            return self.q_held[arm] if self.q_held[arm] is not None \
                else self._master_quat(arm)
        return slerp(q0, q1, self.blend[arm])

    def _distance(self, arm):
        m, g = self.master[arm], self.grasp[arm]
        if m is None or g is None:
            return None
        return float(np.linalg.norm(
            np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]) -
            np.array([g.pose.position.x, g.pose.position.y, g.pose.position.z])))

    def _tick(self):
        p_thr = float(self.get_parameter("p_threshold").value)
        d_thr = float(self.get_parameter("distance_threshold_m").value)
        c_thr = float(self.get_parameter("cancel_distance_m").value)
        servo = max(1e-3, float(self.get_parameter("servo_time_s").value))
        closed = float(self.get_parameter("gripper_closed_rad").value)

        for arm in self.arms:
            m = self.master[arm]
            if m is None:
                continue
            it = self.intent[arm] or {}
            d = self._distance(arm)
            st = self.state[arm]

            if st == DIRECT:
                ok_p = float(it.get("top_p", 0.0)) > p_thr
                ok_amb = not bool(it.get("ambiguous", True))
                ok_d = d is not None and d < d_thr
                ok_g = self.grasp[arm] is not None
                if ok_p and ok_amb and ok_d and ok_g and not self.cancelled[arm]:
                    self.state[arm] = ASSIST
                    self.blend[arm] = 0.0
                    self.gripper_at_assist[arm] = self.gripper[arm]
                    self.q_at_engage[arm] = self._master_quat(arm)
                    self.reason[arm] = ("P=%.2f > %.2f and d=%.3f < %.3f"
                                        % (it.get("top_p", 0), p_thr, d, d_thr))
                    self.get_logger().info("[%s] DIRECT -> ASSIST: %s"
                                           % (arm, self.reason[arm]))
                else:
                    self.reason[arm] = ("waiting: P=%.2f%s%s"
                                        % (float(it.get("top_p", 0.0)),
                                           ", ambiguous" if not ok_amb else "",
                                           ", d=%.3f" % d if d is not None else
                                           ", no grasp"))
            elif st == ASSIST:
                if d is not None and d > c_thr:
                    self._to_direct(arm, "moved away: d=%.3f > %.3f" % (d, c_thr))
                elif self.grasp[arm] is None:
                    self._to_direct(arm, "grasp withdrawn")
                elif bool(it.get("ambiguous", False)):
                    self._to_direct(arm, "intent became ambiguous")
                else:
                    self.blend[arm] = min(1.0, self.blend[arm] + self.dt / servo)
                    if self.gripper_at_assist[arm] is None:
                        self.gripper_at_assist[arm] = self.gripper[arm]
                    # A CLOSING TRANSITION, not a level. The mock gripper boots
                    # at 0.79 rad - already past any sensible "closed"
                    # threshold - so a level test reports a grasp that never
                    # happened, and in the pilot it latched GRASPED for the
                    # whole session.
                    closing = (self.gripper[arm] > closed and
                               self.gripper[arm] > self.gripper_at_assist[arm] + 0.05)
                    if closing:
                        self.state[arm] = GRASPED
                        self.reason[arm] = "gripper closed by the OPERATOR"
                        self.get_logger().info("[%s] ASSIST -> GRASPED (operator "
                                               "FSR command)" % arm)
            elif st == GRASPED:
                if self.gripper[arm] < closed * 0.5:
                    self.state[arm] = DIRECT
                    self.blend[arm] = 0.0
                    self.gripper_at_assist[arm] = None
                    self.reason[arm] = "gripper released"

            self._publish(arm)

    def _publish(self, arm):
        m = self.master[arm]
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        # POSITION IS ALWAYS THE OPERATOR'S. This line is the whole safety
        # argument of the design and must never become conditional.
        out.pose.position = m.pose.position
        q = self._blended_quat(arm) if self.state[arm] in (ASSIST, GRASPED) \
            else (self.q_held[arm] if self.q_held[arm] is not None
                  else self._master_quat(arm))
        if q is not None:
            (out.pose.orientation.x, out.pose.orientation.y,
             out.pose.orientation.z, out.pose.orientation.w) = (float(v) for v in q)
        self.pose_pub[arm].publish(out)

        s = String()
        s.data = self.state[arm]
        self.state_pub[arm].publish(s)

        dbg = String()
        it = self.intent[arm] or {}
        dbg.data = json.dumps(dict(
            arm=arm, state=self.state[arm], blend=round(self.blend[arm], 3),
            reason=self.reason[arm], distance_m=self._distance(arm),
            top=it.get("top"), top_p=it.get("top_p"),
            ambiguous=it.get("ambiguous"),
            gripper_rad=round(self.gripper[arm], 4),
            position_authority="operator",           # always
            orientation_authority=("blended" if self.state[arm] == ASSIST
                                   else "operator")))
        self.dbg_pub[arm].publish(dbg)


def main():
    rclpy.init()
    n = HandoverArbiter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
