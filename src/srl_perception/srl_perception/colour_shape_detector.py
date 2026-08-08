#!/usr/bin/env python3
"""
colour_shape_detector.py — CLEARLY-SECONDARY FALLBACK. Off by default.

Read this before enabling it. Colour segmentation has the failure mode
AprilTag does not: it is confidently wrong. A tag either decodes or it does
not; a colour blob under changed lighting produces a plausible pose for the
wrong thing, and downstream code cannot tell. That is why this exists only as
a fallback, why it must be enabled explicitly, and why every detection it
emits is capped at `max_confidence` (default 0.45) so it can never outrank an
AprilTag detection in the tracker's fusion.

It also cannot give full 6-DOF: with no depth camera, range comes from the
apparent size of a blob of ASSUMED physical width, and orientation comes from
the 2-D minor axis with roll unobservable. Both are recorded in the message
(low score) rather than presented as equivalent to a tag pose.
"""
import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import (BoundingBox3D, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)

# (name, HSV low, HSV high). Two red bands because red wraps the hue circle.
DEFAULT_COLOURS = [
    ("red", (0, 120, 70), (10, 255, 255)),
    ("red", (170, 120, 70), (180, 255, 255)),
    ("green", (40, 80, 60), (85, 255, 255)),
    ("blue", (95, 120, 60), (130, 255, 255)),
    ("yellow", (20, 120, 100), (35, 255, 255)),
]


class ColourShapeDetector(Node):
    def __init__(self):
        super().__init__("colour_shape_detector")
        self.declare_parameter("arm", "left")
        self.declare_parameter("enabled", False)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("object_width_m", 0.05)
        self.declare_parameter("min_area_px", 400)
        # Hard ceiling. AprilTag detections routinely exceed this, so the
        # tracker's max() fusion will always prefer a tag when one exists.
        self.declare_parameter("max_confidence", 0.45)

        self.arm = self.get_parameter("arm").value
        self.enabled = bool(self.get_parameter("enabled").value)
        self.width = float(self.get_parameter("object_width_m").value)
        self.min_area = int(self.get_parameter("min_area_px").value)
        self.max_conf = float(self.get_parameter("max_confidence").value)
        img = self.get_parameter("image_topic").value or \
            f"/{self.arm}_camera/color/image_raw"
        info = self.get_parameter("camera_info_topic").value or \
            f"/{self.arm}_camera/color/camera_info"

        self.bridge = CvBridge()
        self.K = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(CameraInfo, info, self._on_info, 1)
        self.create_subscription(Image, img, self._on_image, qos)
        self.pub = self.create_publisher(
            Detection3DArray, f"/perception/detections/{self.arm}", 10)
        self.diag = self.create_publisher(
            String, f"/perception/fallback_diag/{self.arm}", 10)
        if not self.enabled:
            self.get_logger().warn(
                "colour_shape_detector[%s] loaded but DISABLED. It is a "
                "fallback: colour segmentation fails by being confidently "
                "wrong, which AprilTag does not. Set enabled:=true only "
                "deliberately." % self.arm)

    def _on_info(self, m):
        self.K = np.array(m.k, float).reshape(3, 3)

    def _on_image(self, msg):
        if not bool(self.get_parameter("enabled").value) or self.K is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:                                       # noqa: BLE001
            return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        out = Detection3DArray()
        out.header = msg.header
        found = []
        for name, lo, hi in DEFAULT_COLOURS:
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                (cx, cy), (w, h), ang = cv2.minAreaRect(c)
                px = max(w, h)
                if px < 5:
                    continue
                fx = self.K[0, 0]
                # Range from apparent size of an ASSUMED physical width. If the
                # assumption is wrong, so is the range, linearly.
                z = fx * self.width / px
                x = (cx - self.K[0, 2]) * z / fx
                y = (cy - self.K[1, 2]) * z / self.K[1, 1]
                d = Detection3D()
                d.header = msg.header
                d.id = "colour_%s" % name
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = d.id
                # Confidence rises with blob size but is HARD-CAPPED.
                hyp.hypothesis.score = float(
                    min(self.max_conf, self.max_conf * min(1.0, area / 5000.0)))
                p = hyp.pose.pose
                p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
                # Orientation: only the 2-D minor axis is observable. Roll
                # about the optical axis is NOT, and is left at identity
                # rather than fabricated.
                p.orientation.w = 1.0
                d.results.append(hyp)
                d.bbox = BoundingBox3D()
                d.bbox.center = p
                d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = self.width
                out.detections.append(d)
                found.append(dict(colour=name, area=int(area),
                                  range_m=round(float(z), 3),
                                  minor_axis_deg=round(float(ang), 1)))
        self.pub.publish(out)
        m = String()
        m.data = json.dumps({"arm": self.arm, "fallback": True,
                             "note": "range from assumed object width; "
                                     "roll unobservable",
                             "detections": found})
        self.diag.publish(m)


def main():
    rclpy.init()
    n = ColourShapeDetector()
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
