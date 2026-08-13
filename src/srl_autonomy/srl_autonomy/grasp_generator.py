#!/usr/bin/env python3
"""
grasp_generator.py — offers ONLY grasps that /compute_ik has already accepted.

The rule this node exists to enforce: an unreachable grasp must never be
offered. In a user study an offered-then-failed grasp is worse than no offer,
because the participant attributes the failure to the autonomy and the trial
is contaminated in a way analysis cannot undo.

So every candidate from grasp_library is validated against the live
/compute_ik with `avoid_collisions=True` BEFORE it is published, and both the
pre-grasp standoff and the grasp itself must pass — a grasp you cannot
approach is not a grasp.

Publishes:
  /autonomy/grasp_<arm>          the validated grasp pose (PoseStamped)
  /autonomy/pregrasp_<arm>       its standoff pose
  /autonomy/grasp_status_<arm>   JSON: which object, how many candidates were
                                 tried, why the rejected ones failed
"""
import json
import math

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from srl_autonomy import grasp_library as gl


class GraspGenerator(Node):
    def __init__(self):
        super().__init__("grasp_generator")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("ik_timeout_s", 0.05)
        self.declare_parameter("rate_hz", 4.0)
        self.declare_parameter("n_yaw", 8)

        self.arms = list(self.get_parameter("arms").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout_s").value)
        self.n_yaw = int(self.get_parameter("n_yaw").value)

        self.joints = {}
        self.objects = []
        self.target = {a: None for a in self.arms}

        # The IK calls happen INSIDE a timer callback. Calling
        # spin_until_future_complete() there re-enters the executor and raises
        # "Executor is already spinning", which killed this node outright in
        # the first pilot - the arbiter then sat in DIRECT reporting
        # "P=1.00, no grasp" forever and nothing looked broken from outside.
        # A reentrant group plus a MultiThreadedExecutor lets the blocking
        # call() work while the rest of the node keeps servicing callbacks.
        self.cbg = ReentrantCallbackGroup()
        self.ik = self.create_client(GetPositionIK, "/compute_ik",
                                     callback_group=self.cbg)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Detection3DArray, "/perception/objects",
                                 self._on_objects, 10)
        self.g_pub, self.pg_pub, self.st_pub = {}, {}, {}
        for a in self.arms:
            self.create_subscription(
                String, f"/autonomy/intent_debug_{a}",
                lambda m, a=a: self._on_intent(a, m), 10)
            self.g_pub[a] = self.create_publisher(PoseStamped, f"/autonomy/grasp_{a}", 10)
            self.pg_pub[a] = self.create_publisher(PoseStamped, f"/autonomy/pregrasp_{a}", 10)
            self.st_pub[a] = self.create_publisher(String, f"/autonomy/grasp_status_{a}", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value),
                          self._tick, callback_group=self.cbg)
        self.get_logger().info(
            "grasp_generator up. Nothing is published until /compute_ik has "
            "accepted BOTH the grasp and its pre-grasp standoff.")

    def _on_js(self, m):
        self.joints = dict(zip(m.name, m.position))

    def _on_objects(self, m):
        objs = []
        for d in m.detections:
            if not d.results:
                continue
            p = d.results[0].pose.pose
            # YAW = 0 AND YAW UNKNOWN ARE DIFFERENT ANSWERS.
            #
            # An identity quaternion arrives from any detector that did not
            # measure orientation, and `_yaw_of` turns it into 0.0 -- so a
            # cube sitting at 30 degrees was grasped square and the status
            # message said nothing at all. The unknown case is now carried
            # through to the status, where an operator can see it, instead of
            # being laundered into a number.
            objs.append((d.id or d.results[0].hypothesis.class_id,
                         np.array([p.position.x, p.position.y, p.position.z]),
                         _yaw_of(p.orientation),
                         _yaw_known(p.orientation)))
        self.objects = objs

    def _on_intent(self, arm, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        # Only chase a goal the estimator is not ambiguous about. Generating a
        # grasp for a coin-flip goal wastes IK calls and, worse, makes the
        # arbiter's job look easier than it is.
        self.target[arm] = None if s.get("ambiguous", True) else s.get("top")

    def _seed(self, arm):
        rs = RobotState()
        names = [f"{arm}_joint_{i+1}" for i in range(7)]
        rs.joint_state.name = names
        rs.joint_state.position = [float(self.joints.get(n, 0.0)) for n in names]
        return rs

    def _ik_ok(self, arm, pos, quat):
        if not self.ik.service_is_ready():
            return False, "compute_ik not ready"
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = f"{arm}_arm"
        r.ik_link_name = f"{arm}_end_effector_link"
        r.avoid_collisions = True
        r.timeout.sec = 0
        r.timeout.nanosec = int(self.ik_timeout * 1e9)
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
            (float(v) for v in pos)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in quat)
        r.pose_stamped = ps
        r.robot_state = self._seed(arm)
        req.ik_request = r
        try:
            res = self.ik.call(req)
        except Exception as e:                                   # noqa: BLE001
            return False, "ik call failed: %s" % type(e).__name__
        if res is None:
            return False, "ik call returned nothing"
        return res.error_code.val == 1, "error_code %d" % res.error_code.val

    def _tick(self):
        by_id = {o[0]: o for o in self.objects}
        for arm in self.arms:
            tgt = self.target[arm]
            status = dict(arm=arm, target=tgt, offered=False)
            if tgt is None or tgt not in by_id:
                status["reason"] = "no unambiguous target"
                self._publish_status(arm, status)
                continue
            oid, pos, yaw, yaw_known = by_id[tgt]
            # SAID OUT LOUD ON EVERY OFFER. Without this a square grasp on a
            # turned object is indistinguishable from a correct grasp on a
            # square one, which is the `object_rotated_30deg` row of the
            # lab-day fault table and is marked SILENT there.
            status["yaw_deg"] = round(math.degrees(yaw), 1)
            status["yaw_source"] = ("measured" if yaw_known
                                    else "UNKNOWN, assuming square")
            ok_width, width = gl.is_graspable(oid)
            status["width_m"] = round(width, 4)
            if not ok_width:
                # Say so once, clearly. An object wider than the stroke is not
                # a grasp problem, it is a "wrong object" problem.
                status["reason"] = ("object is %.0f mm across, wider than the "
                                    "%.0f mm safe stroke" %
                                    (width * 1000, gl.GRIPPER_SAFE_WIDTH_M * 1000))
                self._publish_status(arm, status)
                continue
            tried = []
            for cand in gl.candidates(oid, pos, yaw, self.n_yaw):
                pre = gl.pregrasp(cand)
                ok_pre, why_pre = self._ik_ok(arm, pre["position"], pre["quat"])
                if not ok_pre:
                    tried.append(dict(yaw=round(math.degrees(cand["yaw_offset"]), 1),
                                      stage="pregrasp", why=why_pre))
                    continue
                ok_g, why_g = self._ik_ok(arm, cand["position"], cand["quat"])
                if not ok_g:
                    tried.append(dict(yaw=round(math.degrees(cand["yaw_offset"]), 1),
                                      stage="grasp", why=why_g))
                    continue
                self._publish_pose(self.g_pub[arm], cand)
                self._publish_pose(self.pg_pub[arm], pre)
                status.update(offered=True, object=cand["name"],
                              yaw_offset_deg=round(math.degrees(cand["yaw_offset"]), 1),
                              candidates_tried=len(tried) + 1, rejected=tried)
                self._publish_status(arm, status)
                break
            else:
                status["reason"] = ("all %d candidates failed IK - not offered"
                                    % len(tried))
                status["rejected"] = tried
                self._publish_status(arm, status)

    def _publish_pose(self, pub, g):
        m = PoseStamped()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            (float(v) for v in g["position"])
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = (float(v) for v in g["quat"])
        pub.publish(m)

    def _publish_status(self, arm, s):
        m = String()
        m.data = json.dumps(s)
        self.st_pub[arm].publish(m)


def _yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _yaw_known(q, tol=1e-6):
    """False for an identity quaternion, which carries no measurement.

    Deliberately NOT folded into `_yaw_of`: callers that only want a number
    keep getting one, and the caller that needs to know whether anybody
    measured it has to ask separately and say what it did about the answer.
    """
    return not (abs(q.x) < tol and abs(q.y) < tol and abs(q.z) < tol
                and abs(abs(q.w) - 1.0) < tol)


def main():
    rclpy.init()
    n = GraspGenerator()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
