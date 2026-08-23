#!/usr/bin/env python3
"""THE LIVE HALF OF THE CALIBRATION STAGE: arm, camera, TF, and one voice.

Everything that needs a running stack is here, and nothing that decides what
the robot DOES is. The sweep order lives in `calibration_sweep`, the fusion in
`world_model`, the words in `narration` -- all three pure and tested offline.
This file is the part that cannot be tested without a robot, so it is kept as
small and as dull as possible.

WHY IT REFUSES A STALE FRAME
----------------------------
The camera publishes whether or not the arm moved. A capture that takes
whatever is in the buffer will record the PREVIOUS cell's picture at this
cell's pose, and nothing downstream can tell: the map fuses six copies of one
view and reports a confident surface. `fresh_depth` requires a frame stamped
after the move, and says so when there is not one.

WHY THE CAMERA POSE COMES OFF THE FRAME AND NOT OFF TF
------------------------------------------------------
A pixel has to be deprojected with the pose the camera was AT WHEN THE FRAME
WAS TAKEN. Looking the pose up on TF afterwards asks a different question --
where is the camera now -- and the two agree only if the arm has not moved
since. `mock_rgbd_camera` publishes `/<arm>_camera/render_pose` stamped
IDENTICALLY to the frame precisely so a consumer cannot get this wrong, with
a comment recording that the mistake once cost all four T1 grasps.

I MADE IT ANYWAY. `camera_pose` looked up `lookup_transform(..., Time())` --
latest available -- and the map came back with the cubes REPLICATED, one copy
per view, each displaced by about one cell of the sweep: 13 objects where the
scene has 6, at x = 0.334, 0.396, 0.419, 0.487, 0.521, 0.576 for two cubes at
0.420 and 0.480. Two runs of the same command produced different maps. The
displacement being a cell step is the signature: every view was deprojected
with a pose from a different cell.

`frame_pose` now takes the pose whose stamp MATCHES the depth frame's and
refuses when there is none within `POSE_STAMP_TOL_S`. TF is the fallback for a
real camera, which publishes no such topic -- and the map records which was
used, because a TF-posed map carries this error and must not look like one
that does not.

WHY THE CAMERA'S PROVENANCE IS PUBLISHED
----------------------------------------
No camera has ever been attached to this host, so every run so far is against
`mock_rgbd_camera` -- which renders a scene this repository also wrote. That
is a perfectly good test of the pipeline and no test at all of the room, and
the difference has to travel with the map rather than living in somebody's
memory. `camera_provenance` names the publishers it actually found.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# How far apart a depth frame's stamp and a render_pose's stamp may be and
# still describe the same instant. The mock stamps them from ONE clock read,
# so a match is exact; the tolerance exists so the check is a check and not an
# equality test on floats.
POSE_STAMP_TOL_S = 0.02


def _q_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ProbeNode(Node):
    def __init__(self):
        super().__init__("environment_calibrator")
        self.js = {}
        self.depth = {}
        self.color = {}
        self.info = {}
        self.render_pose = {}
        self.pose_source = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 20)
        for arm in ("left", "right"):
            base = "/%s_camera" % arm
            self.create_subscription(
                Image, base + "/depth/image_raw",
                lambda m, a=arm: self._on_depth(a, m), 5)
            # THE COLOUR FRAME, because object identity comes from the
            # PICTURE now and not from clustering the depth. See
            # `srl_perception.segment_lift`.
            self.create_subscription(
                Image, base + "/color/image_raw",
                lambda m, a=arm: self.color.__setitem__(a, m), 5)
            self.create_subscription(
                CameraInfo, base + "/color/camera_info",
                lambda m, a=arm: self.info.__setitem__(a, m), 5)
            self.create_subscription(
                PoseStamped, base + "/render_pose",
                lambda m, a=arm: self.render_pose.setdefault(a, []).append(m),
                20)
        self.pub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 5)
            for a in ("left", "right")}
        self.grip_pub = {a: self.create_publisher(
            JointTrajectory, "/%s_gripper_controller/joint_trajectory" % a, 10)
            for a in ("left", "right")}
        # THE ROBOT'S VOICE. One topic, JSON, so the GUI and anything else
        # read the same sentence the log carries.
        self.say_pub = self.create_publisher(String, "/robot_say", 10)
        self.map_pub = self.create_publisher(String, "/world_map", 1)
        # WHAT THE ROBOT MEASURED, DRAWN. Without this a recording of the run
        # shows two arms moving in an empty room: the operator can see the
        # motion and not the thing it is about. The clip verifier already
        # learned this once -- a scene came up with no table, no cubes and no
        # pads and was filed OK, because the WEARER is 18% of the frame.
        # TRANSIENT LOCAL, to match the RViz display. Without it a viewer
        # that subscribes after the markers were published sees an empty
        # scene and the clip is of an arm reaching for nothing -- which is
        # indistinguishable from the markers never having been built.
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/world_map_markers",
            QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf, self)

    # ------------------------------------------------------------- plumbing
    def _on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def _on_depth(self, arm, m):
        self.depth[arm] = m

    def spin(self, secs):
        t = time.time()
        while time.time() - t < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def names(self, arm):
        return ["%s_joint_%d" % (arm, i) for i in range(1, 8)]

    def wait_ready(self, arm, timeout_s, need_camera=True):
        """Wait for what the CALLER actually needs.

        `need_camera=False` for the jobs that read a stored map: planning a
        pick from a map the robot measured earlier needs the arm and IK and
        nothing else, and demanding a camera made it refuse on a stack that
        could have done the job.
        """
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            self.spin(0.2)
            ok = (self.ik.service_is_ready()
                  and all(n in self.js for n in self.names(arm)))
            if ok and need_camera:
                ok = (arm in self.depth and arm in self.info
                      and arm in self.color)
            if ok:
                return True
        return False

    def missing(self, arm):
        """Which of the inputs has not arrived. For a refusal that helps."""
        out = []
        if not self.ik.service_is_ready():
            out.append("/compute_ik")
        if not all(n in self.js for n in self.names(arm)):
            out.append("/joint_states for the %s arm" % arm)
        for k, d in (("depth", self.depth), ("camera_info", self.info),
                     ("colour", self.color)):
            if arm not in d:
                out.append("%s on /%s_camera" % (k, arm))
        return out

    # ------------------------------------------------------------- the voice
    def say(self, phase, text, speech):
        m = String()
        m.data = json.dumps(dict(phase=phase, text=text, speech=speech,
                                 t=time.time()))
        self.say_pub.publish(m)

    def publish_map_markers(self, doc, ns="world_map"):
        """The measured surface and every mapped object, as RViz markers.

        Drawn from the MAP, never from a task file -- so what appears is what
        the robot measured, and an object it invented shows up as a box
        standing where nothing is. Graspable objects are drawn green and
        refused ones red, with the refusal as a floating label, so a clip
        carries the reason and not just the geometry.
        """
        arr = MarkerArray()
        surf = doc.get("surface", {})
        objs = doc.get("objects", [])
        stamp = self.get_clock().now().to_msg()

        def base(i, kind, ns_):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = ns_
            m.id = int(i)
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        if surf.get("z_m") is not None:
            xs = [o["centre"][0] for o in objs] or [0.0]
            ys = [o["centre"][1] for o in objs] or [0.4]
            m = base(0, Marker.CUBE, ns + "_surface")
            m.pose.position.x = float((min(xs) + max(xs)) / 2.0)
            m.pose.position.y = float((min(ys) + max(ys)) / 2.0)
            m.pose.position.z = float(surf["z_m"]) - 0.005
            m.scale.x = float(max(0.40, max(xs) - min(xs) + 0.30))
            m.scale.y = float(max(0.30, max(ys) - min(ys) + 0.30))
            m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.85, 0.9, 0.45
            arr.markers.append(m)

        for i, o in enumerate(objs):
            c, e = o["centre"], o["extents"]
            ok = bool(o.get("graspable"))
            m = base(i, Marker.CUBE, ns + "_objects")
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                (float(v) for v in c)
            # extents are [long, short, height]; drawn as a box of that size.
            m.scale.x = float(max(e[0], 0.005))
            m.scale.y = float(max(e[1], 0.005))
            m.scale.z = float(max(e[2], 0.005))
            m.color.r = 0.1 if ok else 0.9
            m.color.g = 0.9 if ok else 0.2
            m.color.b = 0.2
            m.color.a = 0.75
            arr.markers.append(m)

            t = base(i, Marker.TEXT_VIEW_FACING, ns + "_labels")
            t.pose.position.x = float(c[0])
            t.pose.position.y = float(c[1])
            t.pose.position.z = float(c[2]) + 0.07
            t.scale.z = 0.028
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = ("%d: %.0f mm%s"
                      % (i, o["width_m"] * 1000,
                         "" if ok else "  REFUSED: " + o.get("why", "")[:40]))
            arr.markers.append(t)
        self.marker_pub.publish(arr)
        return len(arr.markers)

    def publish_map(self, doc):
        m = String()
        m.data = json.dumps(doc)
        self.map_pub.publish(m)

    # --------------------------------------------------------------- the arm
    def anchor_quat(self, arm):
        """The wrist orientation every task commands, as a message.

        `master_calibration.WORKSPACE_ORIENT`, not the live EE orientation --
        reading the arm's current wrist and calling it the anchor is a defect
        this repository has found in about twenty scripts.
        """
        from srl_teleop import master_calibration as MC
        q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
        q = q / np.linalg.norm(q)
        m = Quaternion()
        m.x, m.y, m.z, m.w = (float(v) for v in q)
        return m

    def anchor_axis(self, arm):
        """The direction the wrist camera looks, as a unit vector in world.

        The camera looks along the TOOL AXIS, which is column 2 of the
        anchor's rotation. On this rig that is 30.76 deg ABOVE horizontal, so
        anything that assumes the camera points down is wrong before it
        starts.
        """
        from srl_teleop import master_calibration as MC
        q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
        q = q / np.linalg.norm(q)
        return _q_matrix(q)[:, 2]

    def look_at_quat(self, cam_xyz, target_xyz):
        """A wrist orientation whose TOOL AXIS points from the camera at the target.

        THE CAMERA LOOKS ALONG THE TOOL AXIS, and the grasp anchor points that
        axis 30.76 deg ABOVE horizontal because the hand grasps from below. A
        camera carrying the anchor can therefore never see the TOP of a
        horizontal surface: every ray in its field of view goes upward, so
        from beneath a table it sees the underside and from above it sees the
        room.

        MEASURED, on the second live run of the calibration stage: the map came
        back with a support surface at **z = 1.2154 m**, flat to 0.18 deg,
        while the table is declared at 1.2500. The table is a 35 mm slab
        centred at 1.2325 -- so its underside is at **1.2150**, and the robot
        had measured that to 0.4 mm. The number was right; the thing it was
        looking at was wrong.

        `solve_observe_pose` already knew: the observe pose it solved has its
        tool axis at **-13.75 deg**, pointing DOWN, from a viewpoint above and
        outboard. This builds the same kind of orientation per cell.
        """
        import sys as _sys
        import os as _os
        _abc = _os.path.join(_os.path.dirname(_os.path.dirname(
            _os.path.abspath(__file__))), "src/srl_experiments/experiments/abc")
        if _abc not in _sys.path:
            _sys.path.insert(0, _abc)
        import grasp_frames as _GF
        a = np.asarray(target_xyz, float) - np.asarray(cam_xyz, float)
        n = float(np.linalg.norm(a))
        if n < 1e-6:
            return None
        q = _GF.q_from_axis(a / n)
        m = Quaternion()
        m.x, m.y, m.z, m.w = (float(v) for v in q)
        return m

    def solve(self, arm, xyz, quat, tries=6):
        seed = [self.js.get(k, 0.0) for k in self.names(arm)]
        for k in range(tries):
            s2 = list(seed)
            if k:
                s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request()
            r = PositionIKRequest()
            r.group_name = "%s_arm" % arm
            rs = RobotState()
            rs.joint_state.name = self.names(arm)
            rs.joint_state.position = [float(v) for v in s2]
            r.robot_state = rs
            r.avoid_collisions = True
            r.ik_link_name = "%s_end_effector_link" % arm
            r.pose_stamped.header.frame_id = "world"
            r.pose_stamped.pose.position.x = float(xyz[0])
            r.pose_stamped.pose.position.y = float(xyz[1])
            r.pose_stamped.pose.position.z = float(xyz[2])
            r.pose_stamped.pose.orientation = quat
            r.timeout.sec = 1
            req.ik_request = r
            fut = self.ik.call_async(req)
            end = time.time() + 3.0
            while time.time() < end and not fut.done():
                rclpy.spin_once(self, timeout_sec=0.02)
            if not fut.done():
                continue
            resp = fut.result()
            if resp is None or resp.error_code.val != 1:
                continue
            got = dict(zip(resp.solution.joint_state.name,
                           resp.solution.joint_state.position))
            if all(n in got for n in self.names(arm)):
                return [float(got[n]) for n in self.names(arm)]
        return None

    def send(self, arm, q, secs):
        m = JointTrajectory()
        m.joint_names = self.names(arm)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(secs)
        p.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        m.points = [p]
        self.pub[arm].publish(m)

    def grip(self, arm, rad):
        """Command the gripper, on the SAME topic and joint `run_abc` uses.

        One knuckle joint per arm and the mimic chain follows it -- see
        `srl_fk.poses`, which walks that chain. Named identically to the
        runner's so the two cannot drift apart.
        """
        t = JointTrajectory()
        t.joint_names = ["%s_robotiq_85_left_knuckle_joint" % arm]
        p = JointTrajectoryPoint()
        p.positions = [float(rad)]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = 250_000_000
        t.points = [p]
        self.grip_pub[arm].publish(t)

    def at(self, arm, q, tol=0.02):
        cur = [self.js.get(n) for n in self.names(arm)]
        if any(v is None for v in cur):
            return False
        return max(abs(((a - b + math.pi) % (2 * math.pi)) - math.pi)
                   for a, b in zip(cur, q)) <= tol

    # ------------------------------------------------------------ the camera
    def _depth_now(self, arm, max_age_s):
        """The newest depth frame if it is fresh, without waiting."""
        m = self.depth.get(arm)
        info = self.info.get(arm)
        if m is None or info is None:
            return None, None, 0.0
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > max_age_s:
            return None, None, 0.0
        buf = np.frombuffer(bytes(m.data), dtype=np.uint16)
        if buf.size != m.height * m.width:
            return None, None, 0.0
        d = buf.reshape(m.height, m.width).astype(float) / 1000.0
        return d, [info.k[0], info.k[4], info.k[2], info.k[5]], stamp

    def fresh_depth(self, arm, max_age_s=1.5):
        """(depth_metres, K, stamp) or (None, None, 0).

        16UC1 in MILLIMETRES is what the Kinova driver emits and what the mock
        reproduces, so the conversion happens here rather than in four places.
        """
        end = time.time() + max_age_s
        while time.time() < end and rclpy.ok():
            self.spin(0.05)
            m = self.depth.get(arm)
            info = self.info.get(arm)
            if m is None or info is None:
                continue
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - stamp > max_age_s:
                continue
            buf = np.frombuffer(bytes(m.data), dtype=np.uint16)
            if buf.size != m.height * m.width:
                continue
            d = buf.reshape(m.height, m.width).astype(float) / 1000.0
            K = [info.k[0], info.k[4], info.k[2], info.k[5]]
            return d, K, stamp
        return None, None, 0.0

    def frame_pose(self, arm, stamp):
        """4x4 of the camera pose THIS FRAME WAS TAKEN FROM, or (None, why).

        Returns (matrix, source). `source` is "render_pose" when the pose
        travelled with the frame and "tf" when it had to be looked up, and the
        difference belongs in the map: a TF-posed cloud is deprojected with
        wherever the arm happens to be now.
        """
        best, bd = None, 1e9
        for m in self.render_pose.get(arm, [])[-200:]:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            d = abs(t - stamp)
            if d < bd:
                best, bd = m, d
        if best is not None and bd <= POSE_STAMP_TOL_S:
            M = np.eye(4)
            r = best.pose.orientation
            M[:3, :3] = _q_matrix([r.x, r.y, r.z, r.w])
            M[0, 3] = best.pose.position.x
            M[1, 3] = best.pose.position.y
            M[2, 3] = best.pose.position.z
            self.pose_source[arm] = "render_pose"
            return M, "render_pose"
        M = self.camera_pose(arm)
        if M is None:
            return None, ("no render_pose within %.3f s of the frame (nearest "
                          "%.3f s) and no TF either"
                          % (POSE_STAMP_TOL_S, bd if best is not None else -1))
        self.pose_source[arm] = self.pose_source.get(arm, "tf")
        if self.render_pose.get(arm):
            # A publisher EXISTS and none of its poses matches this frame.
            # That is not a fallback, it is a desynchronised camera, and
            # silently using TF here is the defect this method was written
            # for.
            return None, ("the camera publishes render_pose but none is "
                          "within %.3f s of this frame (nearest %.3f s)"
                          % (POSE_STAMP_TOL_S, bd))
        self.pose_source[arm] = "tf"
        return M, "tf"

    def fresh_frame(self, arm, max_age_s=1.5):
        """(depth_m, bgr, K, stamp) taken at the SAME instant, or Nones.

        BOTH IMAGES OR NEITHER. Segmenting one frame and deprojecting another
        puts the masks over the wrong pixels, and the result is a confident
        object at a place nothing is -- the same class of defect as reading
        the camera pose off TF, one step further along.
        """
        import cv2
        end = time.time() + max_age_s
        while time.time() < end and rclpy.ok():
            self.spin(0.05)
            dm, K, stamp = self._depth_now(arm, max_age_s)
            if dm is None:
                continue
            c = self.color.get(arm)
            if c is None:
                continue
            cs = c.header.stamp.sec + c.header.stamp.nanosec * 1e-9
            if abs(cs - stamp) > POSE_STAMP_TOL_S:
                continue
            buf = np.frombuffer(bytes(c.data), dtype=np.uint8)
            if buf.size != c.height * c.width * 3:
                continue
            img = buf.reshape(c.height, c.width, 3)
            bgr = (cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                   if c.encoding == "rgb8" else img.copy())
            return dm, bgr, K, stamp
        return None, None, None, 0.0

    def camera_pose(self, arm):
        """4x4 of the camera's OPTICAL frame in `world`, from TF.

        THE LATEST TF, which is where the camera is NOW and not where it was
        when any particular frame was taken. Only `frame_pose` should call
        this, and only when the camera publishes no per-frame pose.
        """
        for frame in ("%s_camera_color_frame" % arm,
                      "%s_camera_link" % arm,
                      "%s_camera_depth_frame" % arm):
            try:
                t = self.buf.lookup_transform("world", frame,
                                              rclpy.time.Time())
            except Exception:                                  # noqa: BLE001
                continue
            M = np.eye(4)
            r = t.transform.rotation
            M[:3, :3] = _q_matrix([r.x, r.y, r.z, r.w])
            M[0, 3] = t.transform.translation.x
            M[1, 3] = t.transform.translation.y
            M[2, 3] = t.transform.translation.z
            return M
        return None

    def camera_is_mock(self):
        return any("mock_rgbd" in n for n, _ in self.get_node_names_and_namespaces())

    def camera_provenance(self, arm):
        """Who was actually publishing the depth this map was built from.

        Named rather than assumed: a map whose source is a renderer this
        repository also wrote is a test of the pipeline and not of the room,
        and that distinction has to travel with the map.
        """
        nodes = [n for n, _ in self.get_node_names_and_namespaces()]
        mock = [n for n in nodes if "mock_rgbd" in n]
        return dict(
            topic="/%s_camera/depth/image_raw" % arm,
            mock_publishers=mock,
            is_mock=bool(mock),
            caveat=("SYNTHETIC: the depth was rendered by mock_rgbd_camera, "
                    "so this map tests the pipeline and not the room"
                    if mock else
                    "depth came from a non-mock publisher"),
            pose_source=self.pose_source.get(arm, "none"),
            pose_source_note=(
                "render_pose: the camera pose travelled WITH each frame, so "
                "the deprojection used the pose the frame was taken from"
                if self.pose_source.get(arm) == "render_pose" else
                "tf: the pose was looked up separately and describes where "
                "the camera was when the lookup happened, NOT when the frame "
                "was taken. Every point in this map carries that error."),
            camera_link_verified_against_hardware=False,
            camera_link_note=("the camera pose is forward kinematics, exact "
                              "given the URDF, and the URDF's camera_link has "
                              "never been checked against the physical module"))
