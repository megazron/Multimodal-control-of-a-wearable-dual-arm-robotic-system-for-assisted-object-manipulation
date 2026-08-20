#!/usr/bin/env python3
"""wearer_tracker_node.py -- the wearer's actual body, into the planning scene.

    bash scripts/run_wearer_tracker.sh          # THE WAY TO START IT

MUST RUN UNDER .venv_pose. MediaPipe is not installed system-wide and must not
be: it wants a numpy the rest of this workspace does not, and the working
system is not worth breaking for a new one. `scripts/run_wearer_tracker.sh`
starts this file with the venv's interpreter, which can import rclpy,
cv_bridge and mediapipe at once -- measured, all four import cleanly with
numpy pinned to the system's 1.26.4.

WHAT IT PUBLISHES
    /wearer/estimate       String, JSON. Every segment, its confidence, and
                           the reason it was rejected if it was.
    /wearer/overlay        String, JSON. Image-space skeleton and the
                           PROJECTED robot arms, for the GUI to draw.
    /planning_scene        moveit_msgs/PlanningScene, a diff, ONE writer,
                           rate-limited.

WHAT IT DOES NOT DO, and both are deliberate:

  * IT DOES NOT CORRECT THE ARMS. The robot's own encoders are a far better
    estimate of where the arms are than a camera two metres away looking at a
    partly occluded gripper. The arms are PROJECTED into the picture from
    forward kinematics, which localises them for the operator and, more
    usefully, checks the extrinsic: if the drawn arm does not sit on the real
    arm, the camera transform is wrong and every body position is wrong with
    it.
  * IT DOES NOT MAKE THE WEARER SMALLER. `wearer_tracking.fuse` keeps
    whichever of the tracked and mannequin primitives is CLOSER to the robot,
    per part, always. A camera that says the person has stepped back changes
    nothing. That asymmetry is the safety case, and it is tested by injection
    in test_wearer_tracking.py.

THE FIVE WAYS IT REFUSES, all of them loudly and by name:
    no camera / not calibrated / frame unusable (dark, blank, two people) /
    body could not be placed / every segment gated out.
In all five the planning scene keeps the mannequin, which is exactly what the
system does today with no camera at all.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

_HERE = os.path.dirname(os.path.abspath(__file__))


def _find_ws(start):
    """The workspace root, found by LOOKING for it rather than counting.

    The first version counted four directories up from this file and landed
    one above the workspace, so every relative path -- the pose model, both
    calibration files -- resolved into the home directory. It did not crash:
    it refused with "no pose model at /home/gausms/models/..." which is a
    perfectly clear message about the wrong path. Counting ".." is a guess
    about a layout; looking for a landmark is not.
    """
    d = os.path.abspath(start)
    for _ in range(6):
        if os.path.isdir(os.path.join(d, "src")) and \
                os.path.isfile(os.path.join(d, "CLAUDE.md")):
            return d
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return os.path.abspath(os.path.join(start, "..", "..", ".."))


_WS = os.environ.get("SRL_WS") or _find_ws(_HERE)
for _p in (os.path.join(_WS, "src/srl_perception"),
           os.path.join(_WS, "src/srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from srl_perception import scene_geometry as SG          # noqa: E402
from srl_perception import wearer_tracking as WT         # noqa: E402

# MediaPipe pose landmark indices -> the joint names wearer_tracking wants.
# LEFT AND RIGHT ARE THE PERSON'S OWN, which is MediaPipe's convention too.
LM = {0: "nose", 11: "L-shoulder", 12: "R-shoulder", 13: "L-elbow",
      14: "R-elbow", 15: "L-wrist", 16: "R-wrist", 19: "L-hand-tip",
      20: "R-hand-tip", 23: "L-hip", 24: "R-hip"}
# The landmarks used to PLACE the body. More than the segment joints, because
# PnP is better conditioned the more of the body it spans -- the ankles and
# knees are worth a lot here even though no segment is built from them.
PNP_LM = sorted(set(list(LM) + [25, 26, 27, 28, 7, 8]))


class WearerTracker(Node):

    def __init__(self):
        super().__init__("wearer_tracker")
        self.declare_parameter("model", "models/mediapipe/pose_landmarker_full.task")
        self.declare_parameter("intrinsics", "config/scene_camera_intrinsics.yaml")
        self.declare_parameter("extrinsics", "config/scene_camera_extrinsics.yaml")
        self.declare_parameter("publish_scene", True)
        self.declare_parameter("scene_hz", 2.0)
        self.declare_parameter("min_landmark_vis", 0.5)
        self.declare_parameter("probe_xyz", [0.0, 0.30, 1.20])

        self.frames = self._load_frames()
        self.det, self.det_why = self._load_model()

        self.est_pub = self.create_publisher(String, "/wearer/estimate", 10)
        self.ovl_pub = self.create_publisher(String, "/wearer/overlay", 4)
        self.scene_pub = None
        self.create_subscription(Image, "/scene_camera/image_raw",
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/scene_camera/camera_info",
                                 self.on_info, 10)

        self.prev_pts, self.prev_t = None, None
        self.last_est = None
        self.last_dec = {}
        self.n_frames, self.n_placed = 0, 0
        self.proc_ms = []
        self.motion_to_scene_ms = None
        self._last_scene = 0.0
        self._scene_sig = None
        self._tf = None
        self._fk = None
        self.reason = "waiting for the scene camera"

        self.create_timer(1.0, self.heartbeat)
        self.get_logger().info(
            "wearer tracker up. %s. %s"
            % (("model ready" if self.det else "NO MODEL: " + self.det_why),
               ("calibrated" if self.frames.calibrated
                else "NOT CALIBRATED: " + self.frames.why_not())))

    # ------------------------------------------------------------ set-up
    def _abs(self, p):
        return p if os.path.isabs(p) else os.path.join(_WS, p)

    def _load_frames(self):
        K = D = T = None
        isrc = esrc = ""
        try:
            import yaml
            ip = self._abs(self.get_parameter("intrinsics").value)
            if os.path.exists(ip):
                d = yaml.safe_load(open(ip))
                K = np.array(d["camera_matrix"], float).reshape(3, 3)
                D = np.array(d.get("distortion", [0] * 5), float).ravel()
                isrc = ip
            ep = self._abs(self.get_parameter("extrinsics").value)
            if os.path.exists(ep):
                d = yaml.safe_load(open(ep))
                T = np.array(d["T_world_camera"], float).reshape(4, 4)
                esrc = ep
        except Exception as e:                                # noqa: BLE001
            self.get_logger().error("calibration unreadable: %r" % (e,))
        return SG.SceneFrames(K, D, T, isrc, esrc)

    def _load_model(self):
        path = self._abs(self.get_parameter("model").value)
        if not os.path.exists(path):
            return None, "no pose model at %s" % path
        try:
            import mediapipe as mp
            from mediapipe.tasks import python as tp
            from mediapipe.tasks.python import vision
            self._mp = mp
            opts = vision.PoseLandmarkerOptions(
                base_options=tp.BaseOptions(model_asset_path=path),
                running_mode=vision.RunningMode.IMAGE,
                # FOUR, NOT TWO. The gate that refuses when more than one
                # person is in view can only fire if the detector is ALLOWED
                # to report more than one, and at num_poses=2 a real
                # two-person photograph came back with one -- so the gate
                # never ran and the refusal came from somewhere else
                # entirely. A safety gate that is never reached passes every
                # test for the wrong reason.
                num_poses=4,
                min_pose_detection_confidence=0.5)
            return vision.PoseLandmarker.create_from_options(opts), ""
        except Exception as e:                                # noqa: BLE001
            return None, ("mediapipe is not available in this interpreter "
                          "(%r). Start with scripts/run_wearer_tracker.sh."
                          % (e,))

    def on_info(self, m):
        if self.frames.K is None and any(m.k):
            self.frames.K = np.array(m.k, float).reshape(3, 3)
            self.frames.D = np.array(m.d, float).ravel() if m.d else np.zeros(5)
            self.frames.intrinsics_src = "/scene_camera/camera_info"

    # ------------------------------------------------------------- frame
    def on_image(self, msg):
        t0 = time.perf_counter()
        self.n_frames += 1
        try:
            import cv2
            buf = np.frombuffer(msg.data, np.uint8)
            bgr = buf.reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                bgr = bgr[:, :, ::-1]
        except Exception as e:                                # noqa: BLE001
            self.reason = "the camera frame could not be read: %r" % (e,)
            return

        # ---- gate the FRAME before reading a body out of it -----------
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mean_luma = float(gray.mean())
        luma_span = float(np.percentile(gray, 97) - np.percentile(gray, 3))
        if self.det is None:
            self._fail("no pose model: " + self.det_why, msg)
            return
        img = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                             data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        res = self.det.detect(img)
        n_people = len(res.pose_landmarks) if res.pose_landmarks else 0
        ok, why = WT.frame_is_usable(mean_luma, luma_span, n_people)
        if not ok:
            self._fail(why, msg, overlay_only=bgr.shape)
            return
        if not self.frames.calibrated:
            self._fail(self.frames.why_not(), msg, overlay_only=bgr.shape)
            return

        # ---- place the body ------------------------------------------
        L = res.pose_landmarks[0]
        W = res.pose_world_landmarks[0]
        vmin = float(self.get_parameter("min_landmark_vis").value)
        obj, img_pts = [], []
        for i in PNP_LM:
            if i >= len(L) or L[i].visibility < vmin:
                continue
            obj.append((W[i].x, W[i].y, W[i].z))
            img_pts.append((L[i].x * msg.width, L[i].y * msg.height))
        cam_pts, resid = SG.lift_skeleton(obj, img_pts, self.frames.K,
                                          self.frames.D)
        if cam_pts is None:
            self._fail("the wearer is in view but too little of them is "
                       "clearly visible to place: %s" % resid, msg,
                       overlay_only=bgr.shape)
            return
        # index back from the filtered list to landmark ids
        used = [i for i in PNP_LM
                if i < len(L) and L[i].visibility >= vmin]
        cam_by_lm = {i: cam_pts[k] for k, i in enumerate(used)}

        pts, vis = {}, {}
        for i, name in LM.items():
            if i not in cam_by_lm:
                continue
            pts[name] = tuple(self.frames.camera_to_world(
                cam_by_lm[i].reshape(1, 3))[0])
            vis[name] = float(L[i].visibility)

        now = time.monotonic()
        dt = 0.0 if self.prev_t is None else (now - self.prev_t)
        est = WT.segments_from_joints(pts, vis, prev=self.prev_pts, dt=dt,
                                      now=now, source="scene camera")
        est.note = "PnP residual %.2f px over %d landmarks" % (resid,
                                                               len(used))
        self.prev_pts, self.prev_t = pts, now
        self.last_est = est
        self.n_placed += 1
        self.reason = "tracking"
        self._publish(est, msg, bgr.shape, L)
        self.proc_ms.append((time.perf_counter() - t0) * 1e3)
        del self.proc_ms[:-200]

    # ------------------------------------------------------------ output
    def _mannequin(self):
        try:
            from srl_teleop import wearer_posture as wp
            return wp.wearer_model()
        except Exception:                                     # noqa: BLE001
            return []

    def _publish(self, est, msg, shape, landmarks=None):
        man = self._mannequin()
        probe = tuple(self.get_parameter("probe_xyz").value)
        fused, dec = WT.fuse(man, est, time.monotonic(), probe=probe)
        self.last_dec = dec
        n, total, text = WT.summarise(dec)
        self.est_pub.publish(String(data=json.dumps(dict(
            state=text, n_tracked=n, n_parts=total, note=est.note,
            source=est.source, stamp=est.stamp,
            segments={k: dict(conf=round(s.conf, 3), usable=s.usable,
                              kind=s.kind,
                              dims=[round(x, 4) for x in s.dims],
                              centre=[round(x, 4) for x in s.centre],
                              reasons=s.reasons)
                      for k, s in est.segments.items()},
            decisions={k: list(v) for k, v in dec.items()},
        ))))
        self._overlay(shape, landmarks, dec)
        if bool(self.get_parameter("publish_scene").value):
            self._write_scene(fused, dec)

    def _overlay(self, shape, landmarks, dec):
        """Image-space skeleton plus the PROJECTED robot arms, for the GUI."""
        h, w = shape[0], shape[1]
        sk = []
        if landmarks is not None:
            for a, b in ((11, 13), (13, 15), (12, 14), (14, 16), (11, 12),
                         (11, 23), (12, 24), (23, 24), (0, 11), (0, 12)):
                if a < len(landmarks) and b < len(landmarks):
                    sk.append([landmarks[a].x * w, landmarks[a].y * h,
                               landmarks[b].x * w, landmarks[b].y * h,
                               round(min(landmarks[a].visibility,
                                         landmarks[b].visibility), 3)])
        arms = {}
        for arm in ("left", "right"):
            pw = self._arm_points(arm)
            if pw is not None:
                arms[arm] = [[round(u, 1), round(v, 1)] for u, v in
                             SG.arm_polyline(self.frames, pw, (w, h))]
        self.ovl_pub.publish(String(data=json.dumps(dict(
            width=w, height=h, skeleton=sk, arms=arms,
            decisions={k: v[0] for k, v in dec.items()}))))

    def _arm_points(self, arm):
        """The arm's link origins in world, from TF. None if TF is not up."""
        try:
            import tf2_ros
            import rclpy.time
            if self._tf is None:
                self._tf = tf2_ros.Buffer()
                self._tf_l = tf2_ros.TransformListener(self._tf, self)
                return None
            out = []
            for link in SG.ARM_CHAIN:
                tr = self._tf.lookup_transform(
                    "world", "%s_%s" % (arm, link), rclpy.time.Time())
                v = tr.transform.translation
                out.append((v.x, v.y, v.z))
            return np.array(out)
        except Exception:                                     # noqa: BLE001
            return None

    # ------------------------------------------------------ planning scene
    def _write_scene(self, prims, dec):
        """ONE writer, rate-limited, and only when something moved.

        TWO CLIENTS WRITING ONE PLANNING SCENE cost this project a day: a
        verifier overlapping a sweep read 18 of 171 obstructed where two clean
        runs read 0, because both were applying and removing geometry in the
        same scene. So this publishes a DIFF containing exactly the wearer's
        parts, at a couple of hertz, and only when the geometry has actually
        changed -- never a stream of identical scenes that MoveIt has to
        re-plan against mid-solve.
        """
        now = time.monotonic()
        hz = max(0.2, float(self.get_parameter("scene_hz").value))
        if now - self._last_scene < 1.0 / hz:
            return
        sig = json.dumps([[n, k, [round(x, 3) for x in d],
                           [round(x, 3) for x in c]]
                          for n, k, d, c, _r in prims])
        if sig == self._scene_sig:
            self._last_scene = now
            return
        try:
            from moveit_msgs.msg import CollisionObject, PlanningScene
            from shape_msgs.msg import SolidPrimitive
            from geometry_msgs.msg import Pose
        except Exception:                                     # noqa: BLE001
            return
        if self.scene_pub is None:
            self.scene_pub = self.create_publisher(PlanningScene,
                                                   "/planning_scene", 4)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        for name, kind, dims, centre, rpy in prims:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = "wearer_%s" % name
            co.operation = CollisionObject.ADD
            sp = SolidPrimitive()
            if kind == "box":
                sp.type = SolidPrimitive.BOX
                sp.dimensions = [float(x) for x in dims]
            elif kind == "cylinder":
                sp.type = SolidPrimitive.CYLINDER
                sp.dimensions = [float(dims[1]), float(dims[0])]
            else:
                sp.type = SolidPrimitive.SPHERE
                sp.dimensions = [float(dims[0])]
            p = Pose()
            p.position.x, p.position.y, p.position.z = [float(x) for x in centre]
            qx, qy, qz, qw = _rpy_to_quat(*rpy)
            p.orientation.x, p.orientation.y = qx, qy
            p.orientation.z, p.orientation.w = qz, qw
            co.primitives.append(sp)
            co.primitive_poses.append(p)
            ps.world.collision_objects.append(co)
        self.scene_pub.publish(ps)
        if self._scene_sig is not None:
            self.motion_to_scene_ms = (now - self._last_scene) * 1e3
        self._scene_sig = sig
        self._last_scene = now

    # ------------------------------------------------------------ refuse
    def _fail(self, why, msg=None, overlay_only=None):
        """Say it, publish the mannequin decision, and change nothing else."""
        if why != self.reason:
            self.get_logger().warn("wearer tracking OFF: %s" % why)
        self.reason = why
        self.prev_pts, self.prev_t = None, None
        man = self._mannequin()
        _, dec = WT.fuse(man, None, time.monotonic())
        self.last_dec = dec
        self.last_est = None
        self.est_pub.publish(String(data=json.dumps(dict(
            state="MANNEQUIN -- %s" % why, n_tracked=0,
            n_parts=len(dec), note=why, source="", segments={},
            decisions={k: list(v) for k, v in dec.items()}))))
        if overlay_only:
            self._overlay(overlay_only, None, dec)

    def heartbeat(self):
        if self.n_frames == 0:
            self.get_logger().warn(
                "no frames from the scene camera. %s"
                % ("Is scene_camera_node running?"
                   if os.path.exists("/dev/video0")
                   else "No camera is attached -- run "
                        "scripts/attach_scene_camera.sh."),
                throttle_duration_sec=15.0)


def _rpy_to_quat(r, p, y):
    import math
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def main(argv=None):
    rclpy.init(args=argv)
    n = WearerTracker()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
