#!/usr/bin/env python3
"""
apriltag_detector.py — PRIMARY object detector. Deliberately boring.

The contribution of this project is the autonomy and the human study, not
perception. A flaky detector destroys user-study data in a way that cannot be
recovered in analysis, so this node optimises for PREDICTABLE FAILURE:

  * A tag is either decoded or it is not. There is no confident-but-wrong
    mode, which is exactly what a colour segmenter has and why colour is
    relegated to a clearly-marked fallback.
  * Pose comes from solvePnP on four known corners of a known-size square —
    full 6-DOF, no depth camera needed, no learned component.
  * Every detection carries a confidence and a timestamp, and confidence is
    derived from measurable things (decision margin, apparent tag size,
    reprojection error), never asserted.

Implementation note: the AprilTag families ship inside OpenCV's ArUco module
(DICT_APRILTAG_36h11 and friends), so there is no extra dependency to install
and no separate detector process to keep alive.

Publishes vision_msgs/Detection3DArray on:
    /perception/detections/<arm>    per camera, in the camera optical frame
and, if `publish_world` is set, transformed into `world` via tf2.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import (BoundingBox3D, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)

import tf2_ros

FAMILIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__("apriltag_detector")
        self.declare_parameter("arm", "left")
        self.declare_parameter("family", "36h11")
        # Physical edge length of the black square, metres. Getting this wrong
        # scales every pose linearly, so it is a parameter and not a constant.
        self.declare_parameter("tag_size_m", 0.040)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("publish_world", True)
        self.declare_parameter("world_frame", "world")
        # A tag smaller than this in the image is decodable but its pose is
        # noise. Reported as low confidence rather than dropped, so the
        # detection-rate figure stays honest.
        self.declare_parameter("min_tag_px", 20.0)
        self.declare_parameter("max_reproj_px", 4.0)
        # Corner refinement. MEASURED on 1280x720 synthetic tags at 0.35 m:
        #   none      4.2 ms   pos 2.43 mm   rot 27.7 deg  <- rotation unusable
        #   subpix    3.3 ms   pos 0.65 mm   rot  0.92 deg <- DEFAULT
        #   apriltag  141.7 ms pos 0.46 mm   rot  0.82 deg
        # APRILTAG refinement costs 43x the time for 0.2 mm and 0.1 deg. At
        # 141.7 ms the detector runs at 7 Hz, which is slower than the arm
        # moves; subpix runs at 300 Hz and is the right default. Switch to
        # apriltag only if a measurement proves the extra accuracy is needed.
        self.declare_parameter("corner_refine", "subpix")

        arm = self.get_parameter("arm").value
        self.arm = arm
        img = self.get_parameter("image_topic").value or \
            f"/{arm}_camera/color/image_raw"
        info = self.get_parameter("camera_info_topic").value or \
            f"/{arm}_camera/color/camera_info"
        self.tag_size = float(self.get_parameter("tag_size_m").value)
        self.min_px = float(self.get_parameter("min_tag_px").value)
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.publish_world = bool(self.get_parameter("publish_world").value)

        fam = self.get_parameter("family").value
        if fam not in FAMILIES:
            raise ValueError("family must be one of %s" % list(FAMILIES))
        self.dictionary = cv2.aruco.Dictionary_get(FAMILIES[fam])
        self.params = cv2.aruco.DetectorParameters_create()
        # Corner refinement is what turns a decoded tag into a usable POSE.
        # Without it the pose is fine in POSITION but the ORIENTATION is not
        # recoverable at all (measured 27.7 deg mean, p95 179 deg - the
        # square-marker planar ambiguity flips outright).
        refine = {"none": cv2.aruco.CORNER_REFINE_NONE,
                  "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
                  "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG}
        self.params.cornerRefinementMethod = refine[
            self.get_parameter("corner_refine").value]

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(CameraInfo, info, self._on_info, 1)
        self.create_subscription(Image, img, self._on_image, qos)
        self.pub = self.create_publisher(
            Detection3DArray, f"/perception/detections/{arm}", 10)
        self.diag = self.create_publisher(String, f"/perception/diag/{arm}", 10)

        self.n_frames = 0
        self.n_with_tag = 0
        self.lat = []
        self.create_timer(2.0, self._report)
        self.get_logger().info(
            "apriltag_detector[%s] family=%s tag=%.3f m  image=%s"
            % (arm, fam, self.tag_size, img))

    def _on_info(self, m):
        self.K = np.array(m.k, float).reshape(3, 3)
        self.D = np.array(m.d, float) if len(m.d) else np.zeros(5)

    def _on_image(self, msg):
        t0 = time.monotonic()
        self.n_frames += 1
        if self.K is None:
            self.get_logger().warn("no CameraInfo yet - cannot compute pose",
                                   throttle_duration_sec=5.0)
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warn("cv_bridge: %s" % e, throttle_duration_sec=5.0)
            return
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(grey, self.dictionary,
                                                  parameters=self.params)
        out = Detection3DArray()
        out.header = msg.header
        if ids is not None and len(ids):
            self.n_with_tag += 1
            for c, i in zip(corners, ids.flatten()):
                d = self._pose_of(c, int(i), msg.header)
                if d is not None:
                    out.detections.append(d)
        self.pub.publish(out)
        self.lat.append((time.monotonic() - t0) * 1000.0)
        if len(self.lat) > 500:
            self.lat = self.lat[-500:]

    def _pose_of(self, corner, tag_id, header):
        h = self.tag_size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
        img_pts = corner.reshape(4, 2).astype(float)
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, self.K, self.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.D)
        reproj = float(np.linalg.norm(proj.reshape(4, 2) - img_pts, axis=1).mean())
        side = float(np.linalg.norm(img_pts[0] - img_pts[1]))

        # Confidence from measurable quantities only. A tag that is small in
        # the image or reprojects badly is reported with LOW confidence rather
        # than discarded, so the detection rate stays a detection rate.
        c_size = min(1.0, side / max(self.min_px, 1.0) / 3.0)
        c_reproj = max(0.0, 1.0 - reproj / max(self.max_reproj, 1e-6))
        conf = max(0.0, min(1.0, 0.5 * c_size + 0.5 * c_reproj))

        R, _ = cv2.Rodrigues(rvec)
        q = _mat_to_quat(R)
        det = Detection3D()
        det.header = header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "tag_%d" % tag_id
        hyp.hypothesis.score = conf
        p = hyp.pose.pose
        p.position.x, p.position.y, p.position.z = (float(v) for v in tvec.flatten())
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        det.results.append(hyp)
        det.bbox = BoundingBox3D()
        det.bbox.center = p
        det.bbox.size.x = det.bbox.size.y = self.tag_size
        det.bbox.size.z = 0.001
        det.id = "tag_%d" % tag_id

        if self.publish_world:
            w = self._to_world(det, header)
            if w is not None:
                return w
            return None
        return det

    def _to_world(self, det, header):
        try:
            tf = self.tf_buf.lookup_transform(
                self.world_frame, header.frame_id, rclpy.time.Time())
        except Exception:
            self.get_logger().warn(
                "no tf %s -> %s; dropping detection rather than publishing it "
                "in the wrong frame" % (self.world_frame, header.frame_id),
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        R = _quat_to_mat((r.x, r.y, r.z, r.w))
        p = det.results[0].pose.pose
        v = R @ np.array([p.position.x, p.position.y, p.position.z]) + \
            np.array([t.x, t.y, t.z])
        q = _quat_mul((r.x, r.y, r.z, r.w),
                      (p.orientation.x, p.orientation.y,
                       p.orientation.z, p.orientation.w))
        p.position.x, p.position.y, p.position.z = (float(c) for c in v)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        det.header.frame_id = self.world_frame
        det.bbox.center = p
        return det

    def _report(self):
        rate = self.n_with_tag / self.n_frames if self.n_frames else 0.0
        lat = float(np.mean(self.lat)) if self.lat else float("nan")
        m = String()
        m.data = ('{"arm": "%s", "frames": %d, "frames_with_tag": %d, '
                  '"detection_rate": %.4f, "mean_latency_ms": %.2f}'
                  % (self.arm, self.n_frames, self.n_with_tag, rate, lat))
        self.diag.publish(m)


def _mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, \
            (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, \
            (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, \
            0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, \
            (R[1, 2] + R[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def _quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def main():
    rclpy.init()
    n = AprilTagDetector()
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
