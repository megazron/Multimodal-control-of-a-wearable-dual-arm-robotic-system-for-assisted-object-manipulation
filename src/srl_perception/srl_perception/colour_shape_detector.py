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
apparent size of a blob of ASSUMED physical width, and orientation is the
in-image angle only, with out-of-plane tilt unobservable. Both are recorded in
the message (low score) rather than presented as equivalent to a tag pose.

WHAT CHANGED, AND IT WAS A SILENT FAULT. This detector measured the blob's
in-image angle and published IDENTITY anyway, so `grasp_generator` -- which
reads yaw straight off the quaternion -- saw yaw = 0 for every object however
it was turned. An object at 30 degrees got a square grasp and no check
anywhere could tell that from a square object. It now publishes the angle it
measured, and publishes identity ONLY when the blob is too close to square
for the angle to mean anything, so identity now means "not measured"
throughout. See `scene_fingerprint.yaw_from_quat`.
"""
import json
import math

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

# SIZE AT RANGE -- the gate that stops a PAD being called a CUBE.
#
# Added 2026-08-16 after a measured failure. T1's coloured pads are 210 x 130
# mm in exactly the colours of the 40 mm cubes, and this detector had a
# `min_area_px` FLOOR with no upper bound and no size check of any kind. At the
# observe pose it locked onto the pads and scored 0 of 4 cubes: one wrong
# colour and three missed, because the pads are the biggest blobs of cube
# colour in the frame.
#
# The check is not "is the blob small". It is "is the blob the size this object
# WOULD BE at the range it appears to be at". Two forms, because two callers
# have different information:
#
#   NO DEPTH (this node). Range is derived from the apparent size of an
#   ASSUMED width, so a size test against that range is circular. What is NOT
#   circular is the range itself: a 210 mm pad read as a 40 mm cube yields
#   z = fx * 0.04 / 184 = 0.134 m, i.e. "a cube 134 mm from the lens". Reject
#   anything whose implied range falls outside the window the arm actually
#   works in and the pad disappears, while a real cube at 0.5 m (49 px,
#   z = 0.502) passes untouched.
#
#   WITH DEPTH (verify_colour_vision). Compare the blob's pixel size against
#   fx * size / measured_depth directly. That is the honest test and it is what
#   the harness uses.
EXPECTED_PX_TOL = (0.55, 1.80)


def expected_px(fx, object_size_m, range_m):
    """How many pixels across an object of this size should be at this range."""
    return float(fx) * float(object_size_m) / max(1e-6, float(range_m))


def size_at_range_ok(px, fx, object_size_m, range_m, tol=EXPECTED_PX_TOL):
    """Is a blob of `px` across the right size for `object_size_m` at `range_m`?"""
    e = expected_px(fx, object_size_m, range_m)
    return tol[0] * e <= float(px) <= tol[1] * e


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
        # THE WORKING RANGE WINDOW. A blob whose IMPLIED range falls outside
        # this is not the object it claims to be -- see the note on
        # size_at_range_ok. Defaults span the wrist camera's usable band: the
        # depth module is blind below 0.25 m and the arm cannot hold anything
        # further than ~1.5 m from its own wrist.
        self.declare_parameter("range_window_m", [0.25, 1.50])
        # Hard ceiling. AprilTag detections routinely exceed this, so the
        # tracker's max() fusion will always prefer a tag when one exists.
        self.declare_parameter("max_confidence", 0.45)
        # Below this width-to-height ratio the minAreaRect angle is
        # degenerate and no yaw is published. 1.15 on a 40 mm cube is a
        # 6 mm difference between the two sides, which is comfortably above
        # the contour noise and well below any real elongation.
        self.declare_parameter("min_aspect_for_yaw", 1.15)

        self.arm = self.get_parameter("arm").value
        self.enabled = bool(self.get_parameter("enabled").value)
        self.width = float(self.get_parameter("object_width_m").value)
        self.min_area = int(self.get_parameter("min_area_px").value)
        self.range_window = [float(v) for v in
                             self.get_parameter("range_window_m").value]
        self.max_conf = float(self.get_parameter("max_confidence").value)
        self.min_aspect_for_yaw = float(
            self.get_parameter("min_aspect_for_yaw").value)
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
                # SIZE AT RANGE. Reject a blob whose implied range is outside
                # the band this camera can work in: that means it is not an
                # object of `object_width_m` at all. This is what stops the
                # 210 mm pads being reported as 40 mm cubes.
                if not (self.range_window[0] <= z <= self.range_window[1]):
                    continue
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
                # ORIENTATION: THE IN-IMAGE ANGLE IS PUBLISHED, BECAUSE IT
                # WAS MEASURED. The old comment here said roll about the
                # optical axis is unobservable and left w = 1.0; that is
                # backwards. Rotation about the optical axis is precisely
                # what minAreaRect measures -- it is the OUT-OF-PLANE tilt
                # that this detector cannot see. So the measured angle was
                # being computed, written into a debug string, and thrown
                # away, while identity went out on the wire.
                #
                # The cost of that was not cosmetic. `grasp_generator` reads
                # yaw straight off this quaternion, and identity reads as
                # yaw = 0, so an object sitting at 30 degrees got a square
                # grasp with nothing anywhere able to tell "the object is
                # square" from "nobody looked". That is the rotated-object
                # fault listed as SILENT in the lab-day table.
                #
                # A NEAR-SQUARE BLOB STILL GETS IDENTITY, and that is the
                # honest answer rather than a lazy one: minAreaRect's angle
                # is degenerate when the two sides are the same length, so
                # publishing it would be inventing a direction. Identity now
                # means "not measured" everywhere downstream -- see
                # scene_fingerprint.yaw_from_quat.
                lo_ax, hi_ax = min(w, h), max(w, h)
                if lo_ax > 0 and hi_ax / lo_ax >= self.min_aspect_for_yaw:
                    a = math.radians(float(ang))
                    p.orientation.z = math.sin(a / 2.0)
                    p.orientation.w = math.cos(a / 2.0)
                    yaw_known = True
                else:
                    p.orientation.w = 1.0
                    yaw_known = False
                d.results.append(hyp)
                d.bbox = BoundingBox3D()
                d.bbox.center = p
                d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = self.width
                out.detections.append(d)
                found.append(dict(colour=name, area=int(area),
                                  range_m=round(float(z), 3),
                                  minor_axis_deg=round(float(ang), 1),
                                  aspect=round(float(hi_ax / lo_ax), 2)
                                  if lo_ax else None,
                                  # PUBLISHED, or explicitly not. A reader of
                                  # this diagnostic can tell a square object
                                  # from an unmeasured one.
                                  yaw_deg=(round(float(ang), 1)
                                           if yaw_known else None),
                                  yaw_known=yaw_known))
        self.pub.publish(out)
        m = String()
        m.data = json.dumps({"arm": self.arm, "fallback": True,
                             "note": "range from assumed object width; "
                                     "in-image yaw measured, out-of-plane "
                                     "tilt unobservable",
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
