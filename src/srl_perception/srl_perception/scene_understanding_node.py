#!/usr/bin/env python3
"""EVERY camera runs computer vision, in ONE node, and says how it knew.

    ros2 run srl_perception scene_understanding_node

This is `scripts/srl_scene_understanding.py` promoted into the package: the
same four cameras, the same table-first pipeline, publishing on topics the
planner's seam (`map_obstacles_node`) can consume instead of drawing tiles.
The standalone script stays for the operator's eyes; this node is for the
stack.

WHY TABLE FIRST, AND NOT COLOUR FIRST -- colour alone cannot tell a cube from
a shirt; what separates an object from the room is that it STANDS ON A
SURFACE. So each depth camera, per cycle: RANSAC the dominant plane, keep the
points 8-300 mm above it, cluster them IN the plane, and measure each cluster.
The name is DERIVED from the measurement, never declared.

FRAMES ARE PART OF THE ANSWER. Only the wrist cameras can convert to the
robot frame -- their pose travels WITH the frame (`render_pose`; the sweep of
2026-08-17 measured 31-36 mm of grasp miss from looking a pose up later) or
comes from TF down a single chain. The room cameras have no measured
extrinsic, so their 3-D is reported in their OWN frame and labelled
`camera:<name>`; inventing an extrinsic would be worse than doing without.

THE ROBOT ARMS, THE PERSON AND THE MANNEQUIN ARE DELIBERATELY NOT DETECTED
HERE. The arms' poses come from their own encoders plus the URDF -- asking a
camera to find the arm is strictly worse. The person is `wearer_tracker_node`'s
job (MediaPipe under .venv_pose) and it already feeds `fuse()`; a second
person-detector would be a second writer of the one quantity whose failure
mode is somebody's chest. What THIS node adds on the scene camera is 2-D
open-vocabulary detection ("person", "robot arm", "table", objects) as a
labelled picture -- pixels and provenance, never metres, because that camera
publishes a ZERO K until it is calibrated and a zero K cannot deproject.

REFUSE RATHER THAN INVENT: a camera with no frames says "no frames"; a zero K
names the missing calibration; a failed plane fit names the camera. Nothing
republishes a last good frame as if it were new.
"""
from __future__ import annotations

import json
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from srl_perception.prompt_detector import PromptDetector, _colour_matches

# What counts as "standing on the table" -- same values the standalone script
# and `table_scene` use; below 8 mm is depth noise, above 300 mm is the room.
ABOVE_MIN, ABOVE_MAX = 0.008, 0.30
CLUSTER_M = 0.035
MIN_PTS = 40
RANGE_M = (0.10, 4.0)

#: name -> topic lists. LISTS, because the mock and the real driver disagree
#: on where camera_info lives (the mock publishes color/camera_info only, the
#: real wrist driver has a depth/camera_info too), and /scene_camera/image_raw
#: is an alias for the same USB device scene/usb reads.
CAMS = {
    "scene/usb": dict(
        colour=["/scene/usb/image_raw", "/scene_camera/image_raw"],
        depth=[], info=["/scene_camera/camera_info"],
        pose=[], metric=False),
    "scene/rs": dict(
        colour=["/scene/rs/color/image_raw"],
        depth=["/scene/rs/depth/image_raw"],
        info=["/scene/rs/color/camera_info", "/scene/rs/depth/camera_info"],
        pose=[], metric=True),
    "gripper/left": dict(
        colour=["/left_camera/color/image_raw"],
        depth=["/left_camera/depth/image_raw"],
        info=["/left_camera/color/camera_info", "/left_camera/depth/camera_info"],
        pose=["/left_camera/render_pose"], metric=True),
    "gripper/right": dict(
        colour=["/right_camera/color/image_raw"],
        depth=["/right_camera/depth/image_raw"],
        info=["/right_camera/color/camera_info", "/right_camera/depth/camera_info"],
        pose=["/right_camera/render_pose"], metric=True),
}

# The operator's label set for the colour-only scene camera, plus the colour
# probes the HSV fallback can actually answer when no neural backend imports.
OPEN_VOCAB_LABELS = ["person", "robot arm", "table", "object"]
COLOUR_PROBES = ["red object", "green object", "blue object", "yellow object"]
COLOUR_NAMES = ["red", "green", "blue", "yellow", "white", "black"]


# ---------------------------------------------------------------- pure parts
# Everything that decides a NUMBER is a module-level function taking arrays,
# so the known-answer test drives it with constructed geometry and no ROS.

def fit_plane(P, iters=140, tol=0.008, seed=0):
    """Dominant plane by RANSAC, refined by PCA. (n, d, inlier mask) or None.

    Same algorithm as scripts/srl_scene_understanding.py: the normal is
    pointed AWAY from the camera (d > 0) so "above the table" is a sign, not
    a guess -- the camera is at the origin of this frame.
    """
    if len(P) < 200:
        return None
    rng = np.random.default_rng(seed)
    best, bc = None, -1
    for _ in range(iters):
        p = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n = n / L
        d = -n @ p[0]
        c = int((np.abs(P @ n + d) < tol).sum())
        if c > bc:
            bc, best = c, (n, d)
    if best is None or bc < 150:
        return None
    n, d = best
    Q = P[np.abs(P @ n + d) < tol]
    cen = Q.mean(0)
    _, V = np.linalg.eigh(np.cov((Q - cen).T))
    n = V[:, 0] / np.linalg.norm(V[:, 0])
    d = -float(n @ cen)
    if d < 0:
        n, d = -n, -d
    return n, d, np.abs(P @ n + d) < tol


def cluster(xy, radius=CLUSTER_M):
    """Greedy spatial clustering in the table plane. Labels, -1 for noise."""
    lab = -np.ones(len(xy), int)
    nxt = 0
    for i in range(len(xy)):
        if lab[i] >= 0:
            continue
        stack, lab[i], nxt = [i], nxt, nxt + 1
        while stack:
            j = stack.pop()
            near = np.nonzero((lab < 0) &
                              (np.linalg.norm(xy - xy[j], axis=1) < radius))[0]
            lab[near] = lab[i]
            stack.extend(near.tolist())
    return lab


def name_for(width_mm, height_mm):
    """What a thing of this size IS, and whether the 85 mm jaws close on it."""
    if width_mm < 15:
        return "speck", False
    if width_mm <= 90 and height_mm <= 90:
        return "cube", width_mm <= 85
    if width_mm <= 200:
        return "box", False
    return "large object", False


def colour_name(mean_bgr):
    """A colour word for a region's MEAN colour, or 'plain'.

    Region means, not per-pixel hue: per-pixel filters picked the room's teal
    robots for the green cube three times (hue 76 against 74)."""
    b, g, r = [float(v) for v in mean_bgr]
    for term in COLOUR_NAMES:
        if _colour_matches(term, b, g, r):
            return term
    return "plain"


def quat_to_R(q):
    """(x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_world(P, t, q):
    """Camera-frame points -> world, given the camera's world pose (t, quat).

    The pose is the one that travelled WITH the frame (`render_pose`), never
    one looked up later: the 2026-08-17 sweep measured 31-36 mm of grasp miss
    from exactly that substitution."""
    R = quat_to_R(q)
    P = np.asarray(P, float).reshape(-1, 3)
    return (R @ P.T).T + np.asarray(t, float)


def plane_to_world(n, d, t, q):
    """A plane fitted in the camera frame, expressed in world."""
    R = quat_to_R(q)
    nw = R @ np.asarray(n, float)
    dw = float(d) - float(nw @ np.asarray(t, float))
    return nw, dw


def analyse_metric(bgr, depth_m, K4, above=(ABOVE_MIN, ABOVE_MAX),
                   cluster_m=CLUSTER_M, min_pts=MIN_PTS, max_objects=8):
    """One RGB-D frame -> {"table": ..., "objects": [...]} or {"refusal": ...}.

    Coordinates are in the CAMERA frame the depth came from -- whose frame
    that is in world terms is the caller's problem, and the caller must say.
    A refusal names its cause; it is never an empty result.
    """
    fx, fy, cx, cy = [float(v) for v in K4]
    if fx <= 0.0 or fy <= 0.0:
        return {"refusal": "camera_info K is zero -- an uncalibrated camera "
                           "cannot deproject pixels to metres"}
    dep = np.asarray(depth_m, float)
    v, u = np.nonzero((dep > RANGE_M[0]) & (dep < RANGE_M[1]) & np.isfinite(dep))
    if len(v) == 0:
        return {"refusal": "no depth between %.2f and %.2f m in this frame -- "
                           "nothing to measure" % RANGE_M}
    if len(v) < 400:
        return {"refusal": "only %d depth returns between %.2f and %.2f m -- "
                           "not enough to fit a plane" % (len(v), *RANGE_M)}
    step = max(1, len(v) // 20000)          # keep the cycle real-time
    v, u = v[::step], u[::step]
    Z = dep[v, u]
    P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
    pl = fit_plane(P)
    if pl is None:
        return {"refusal": "no dominant plane in the depth -- a support "
                           "surface must be visible before things ON it can "
                           "be named"}
    n, d, inl = pl
    h = P @ n + d
    table = dict(normal=[round(float(x), 4) for x in n],
                 d_m=round(float(d), 4), inliers=int(inl.sum()),
                 rms_mm=round(float(np.sqrt((h[inl] ** 2).mean()) * 1000), 2))
    on = (h > above[0]) & (h < above[1])
    if int(on.sum()) < min_pts:
        return {"table": table, "objects": [], "plane": (n, d)}
    # Coordinates IN the plane, so clustering is 2-D and objects that touch in
    # the image but sit apart on the table stay separate.
    e1 = np.cross(n, [0, 0, 1.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [0, 1.0, 0])
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    Q = P[on]
    uv = np.stack([u[on], v[on]], 1)
    xy = np.stack([Q @ e1, Q @ e2], 1)
    lab = cluster(xy, radius=cluster_m)
    objs = []
    for k in range(lab.max() + 1 if len(lab) else 0):
        sel = lab == k
        if int(sel.sum()) < min_pts:
            continue
        C = Q[sel]
        hh = h[on][sel]
        s1, s2 = float(np.ptp(C @ e1)), float(np.ptp(C @ e2))
        width_mm = min(s1, s2) * 1000.0
        height_mm = float(hh.max()) * 1000.0
        nm, pickable = name_for(width_mm, height_mm)
        if nm == "speck":
            continue
        # Colour is sampled at the DEPTH pixels the cluster came from. On an
        # unaligned sensor pair those are the wrong colour pixels (the wrist
        # pair sits ~27 mm apart), so the caller passes bgr=None there and
        # the colour is reported as unknown rather than sampled wrongly.
        if bgr is not None and bgr.shape[:2] == dep.shape[:2]:
            px = uv[sel]
            mean = bgr[px[:, 1], px[:, 0]].reshape(-1, bgr.shape[2]).mean(0)
            col = colour_name(mean[:3])
        else:
            col = "unknown (colour not aligned to depth)"
        objs.append(dict(
            label=nm, pickable=bool(pickable), colour=col,
            centre=[round(float(x), 4) for x in C.mean(0)],
            extents_m=[round(max(s1, s2), 4), round(min(s1, s2), 4),
                       round(float(hh.max()), 4)],
            width_mm=round(width_mm, 1), height_mm=round(height_mm, 1),
            n_points=int(sel.sum()),
            bbox_depth_px=[int(uv[sel][:, 0].min()), int(uv[sel][:, 1].min()),
                           int(uv[sel][:, 0].max()), int(uv[sel][:, 1].max())]))
    objs.sort(key=lambda o: -o["n_points"])
    return {"table": table, "objects": objs[:max_objects], "plane": (n, d)}


def segmenter_available():
    """(usable, why-not). Checked ONCE, reported in every cycle's provenance.

    `segment_lift.objects_in_view` needs torch (via ultralytics), the FastSAM
    weights, and `pick_from_table` off scripts/. On the system interpreter
    torch does not exist -- it lives in .venv_vision only -- so this node
    normally runs the geometric path and SAYS so, rather than crashing or
    silently degrading.
    """
    try:
        import os
        from srl_perception.segment_lift import _weights_path
        _weights_path()
        scripts = os.path.expanduser("~/kortex_ws/scripts")
        if os.path.isdir(scripts) and scripts not in sys.path:
            sys.path.append(scripts)
        import ultralytics                                    # noqa: F401
        import pick_from_table                                # noqa: F401
        return True, ""
    except Exception as e:                                    # noqa: BLE001
        return False, "%s: %s" % (type(e).__name__, e)


def markers_for(objects, frame="world"):
    """Robot-frame objects -> MarkerArray. DELETEALL first, so two cycles do
    not stack on screen (the lesson `scene_markers` records)."""
    arr = MarkerArray()
    wipe = Marker()
    wipe.action = Marker.DELETEALL
    arr.markers.append(wipe)
    for i, o in enumerate(objects):
        m = Marker()
        m.header.frame_id = frame
        m.ns = "scene_objects"
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        c = o["centre"]
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(c[0]), float(c[1]), float(c[2])
        m.pose.orientation.w = 1.0
        ex = o.get("extents_m", [0.05, 0.05, 0.05])
        # Orientation in the plane is not carried, so the drawn box is the
        # circumscribed square in x-y: it may only overstate the object.
        m.scale.x = m.scale.y = max(float(ex[0]), 0.01)
        m.scale.z = max(float(ex[2]), 0.01)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 0.3, 0.5
        arr.markers.append(m)
    return arr


def colour_report(bgr, detector, labels=None, probes=None):
    """The colour-only camera's answer: pixel boxes, labels, and PROVENANCE.

    No metric fields, ever -- the caller attaches `why_no_metric` naming the
    missing calibration. A label the running backend cannot answer is listed
    as not-detectable with the reason, instead of silently returning nothing.
    """
    labels = labels if labels is not None else OPEN_VOCAB_LABELS
    dets, undetectable = [], []
    for lb in labels:
        try:
            hits = detector.detect(bgr, lb)
        except ValueError as e:
            # The HSV fallback needs a colour word; "person" has none.
            undetectable.append(dict(label=lb, why=str(e)))
            continue
        for h in hits[:4]:
            dets.append(dict(label=lb, bbox_px=list(h.bbox),
                             score=round(h.score, 3), backend=h.backend))
    note = ""
    if detector.backend == "colour":
        note = ("open-vocabulary backend unavailable (no torch on this "
                "interpreter) -- HSV colour fallback: only coloured objects "
                "are detectable")
        for lb in (probes if probes is not None else COLOUR_PROBES):
            try:
                hits = detector.detect(bgr, lb)
            except ValueError:
                continue
            for h in hits[:4]:
                dets.append(dict(label=lb, bbox_px=list(h.bbox),
                                 score=round(h.score, 3), backend=h.backend))
    return dict(detections=dets, not_detectable=undetectable,
                backend=detector.backend, backend_note=note)


# ------------------------------------------------------------------ overlays
# The operator's window shows the cameras DOING vision, not a JSON blob: each
# cycle's results are burned onto the frame they came from. Pure functions --
# cv2 rectangle/putText only -- so the known-answer test drives them with
# constructed frames and no ROS.

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _wrap_text(text, width=70):
    """Greedy word wrap, so a long refusal stays readable at 640 px."""
    words, lines, cur = str(text).split(), [], ""
    for w in words:
        cand = (cur + " " + w).strip()
        if len(cand) > width and cur:
            lines.append(cur)
            cur = w
        else:
            cur = cand
    if cur:
        lines.append(cur)
    return lines or [""]


def _banner(img, lines, scale=0.45):
    """Burn `lines` across the top on a filled strip; returns strip height."""
    y = 0
    for ln in lines:
        (_, th), base = cv2.getTextSize(ln, _FONT, scale, 1)
        strip = th + base + 8
        cv2.rectangle(img, (0, y), (img.shape[1], y + strip), (0, 0, 0), -1)
        cv2.putText(img, ln, (4, y + 4 + th), _FONT, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += strip
    return y


def _label_box(img, text, x, y, colour):
    """A one-line label on its own background strip, just above (x, y)."""
    (tw, th), base = cv2.getTextSize(text, _FONT, 0.4, 1)
    ty = max(y - 3, th + base)
    cv2.rectangle(img, (x, ty - th - base - 2), (x + tw + 4, ty + 2),
                  (0, 0, 0), -1)
    cv2.putText(img, text, (x + 2, ty - base + 1), _FONT, 0.4, colour, 1,
                cv2.LINE_AA)


def _horizon_endpoints(n, k4, w, h):
    """The fitted plane's vanishing line in pixel space, clipped to the image.

    A pixel (u, v) looks along ((u-cx)/fx, (v-cy)/fy, 1); the horizon is
    where that ray is parallel to the plane, n . dir = 0 -- a straight line
    in (u, v), by arithmetic. Fronto-parallel planes have no horizon in
    frame and return None, which is correct, not a failure.
    """
    fx, fy, cx, cy = [float(v) for v in k4]
    a, b = float(n[0]) / fx, float(n[1]) / fy
    c = float(n[2]) - float(n[0]) * cx / fx - float(n[1]) * cy / fy
    pts = []
    for u in (0.0, w - 1.0):
        if abs(b) > 1e-12:
            v = -(a * u + c) / b
            if 0.0 <= v <= h - 1.0:
                pts.append((u, v))
    for v in (0.0, h - 1.0):
        if abs(a) > 1e-12:
            u = -(b * v + c) / a
            if 0.0 <= u <= w - 1.0:
                pts.append((u, v))
    uniq = []
    for p in pts:
        if all(abs(p[0] - q[0]) + abs(p[1] - q[1]) > 1e-6 for q in uniq):
            uniq.append(p)
    return (uniq[0], uniq[1]) if len(uniq) >= 2 else None


def draw_overlay(bgr, entry, name="", k4=None, depth_shape=None):
    """One camera's cycle result burned onto its own frame. Returns a NEW
    bgr image; the input is never touched.

    `entry` is exactly what `_camera_entry` reports, so the picture cannot
    disagree with the JSON. A refusing camera gets its refusal VERBATIM in
    the banner -- the operator sees WHY, on the frame that refused. Metric
    bboxes are in DEPTH pixels; `depth_shape` scales them onto a colour
    frame of a different size (approximate on an unaligned pair, and only
    used for drawing, never for measurement). `k4` (depth-scaled intrinsics)
    lets the plane's horizon line be drawn when it crosses the frame.
    """
    img = np.ascontiguousarray(np.asarray(bgr, np.uint8)[:, :, :3]).copy()
    h, w = img.shape[:2]
    if "refusal" in entry:
        _banner(img, _wrap_text("%s: REFUSED -- %s"
                                % (name or "camera", entry["refusal"])))
        return img
    sx = sy = 1.0
    dh, dw = (depth_shape if depth_shape is not None else (h, w))
    if (dh, dw) != (h, w) and dh and dw:
        sy, sx = h / float(dh), w / float(dw)
    dets = entry.get("detections")
    if dets is not None:                       # the colour-only scene camera
        _banner(img, _wrap_text(
            "%s: %d detections, backend=%s"
            % (name or "camera", len(dets), entry.get("backend", "?"))))
        for d in dets:
            x0, y0, x1, y1 = [int(round(v)) for v in d["bbox_px"]]
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 200, 255), 2)
            _label_box(img, "%s %.2f" % (d["label"], d.get("score", 0.0)),
                       x0, y0, (0, 200, 255))
        return img
    objs = entry.get("objects", [])
    _banner(img, _wrap_text("%s: %d objects, %s"
                            % (name or "camera", len(objs),
                               entry.get("method", "?"))))
    table = entry.get("table")
    if table is not None and k4 is not None:
        ends = _horizon_endpoints(table["normal"], k4, dw, dh)
        if ends is not None:
            (u0, v0), (u1, v1) = ends
            cv2.line(img, (int(round(u0 * sx)), int(round(v0 * sy))),
                     (int(round(u1 * sx)), int(round(v1 * sy))),
                     (255, 160, 0), 1, cv2.LINE_AA)
    for o in objs:
        bb = o.get("bbox_depth_px")
        if bb is None:               # segment_lift objects carry no 2-D box
            continue
        x0, y0, x1, y1 = [int(round(v)) for v in bb]
        x0, x1 = int(round(x0 * sx)), int(round(x1 * sx))
        y0, y1 = int(round(y0 * sy)), int(round(y1 * sy))
        cv2.rectangle(img, (x0, y0), (x1, y1), (60, 220, 60), 2)
        _label_box(img, "%s %.0fmm %s"
                   % (o["label"], o.get("width_mm", 0.0),
                      o.get("colour", "?")), x0, y0, (60, 220, 60))
    return img


# ------------------------------------------------------------------ the node

class SceneUnderstanding(Node):
    def __init__(self):
        super().__init__("scene_understanding")
        self.declare_parameter("rate_hz", 2.0)
        # A camera that stopped speaking must not go on being "seen" through
        # its last frame -- stale-republished-with-a-new-timestamp is the
        # instrument-failure table's third row.
        self.declare_parameter("stale_after_s", 2.0)
        # The cameras VISIBLY doing vision: each cycle's entry burned onto
        # the frame it came from. No extra rate, no re-published stale frame.
        self.declare_parameter("publish_overlays", True)
        self.col, self.dep, self.info, self.pose = {}, {}, {}, {}
        self.t = {}
        self._overlay_t = {}
        self._tf = None
        self._tf_l = None
        q = qos_profile_sensor_data
        for name, cfg in CAMS.items():
            for tp in cfg["colour"]:
                self.create_subscription(
                    Image, tp, lambda m, k=name: (
                        self.col.__setitem__(k, m),
                        self.t.__setitem__(k, time.time())), q)
            for tp in cfg["depth"]:
                self.create_subscription(
                    Image, tp, lambda m, k=name: self.dep.__setitem__(k, m), q)
            for tp in cfg["info"]:
                # First topic that speaks wins; the mock and the real driver
                # publish on different ones and never both.
                self.create_subscription(
                    CameraInfo, tp,
                    lambda m, k=name: self.info.__setitem__(k, m), q)
            for tp in cfg["pose"]:
                self.create_subscription(
                    PoseStamped, tp,
                    lambda m, k=name: self.pose.__setitem__(k, m), q)
        self.pub = self.create_publisher(String, "/perception/scene/objects", 10)
        self.mark_pub = self.create_publisher(
            MarkerArray, "/perception/scene/markers", 4)
        self.overlay_pubs = {
            name: self.create_publisher(
                Image, "/perception/scene/overlay/%s"
                % name.replace("/", "_"), 2)
            for name in CAMS}
        self.detector = PromptDetector(backend="auto")
        self.seg_ok, self.seg_why = segmenter_available()
        rate = max(0.2, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "scene understanding up; segment_lift %s"
            % ("available" if self.seg_ok
               else "unavailable (%s) -- geometric cluster path" % self.seg_why))

    @staticmethod
    def _bgr(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else a

    @staticmethod
    def _depth_m(m):
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        return d.astype(np.float32) * 0.001

    def _k4(self, name, dm):
        """(fx, fy, cx, cy) scaled to the depth image, and a status word."""
        ci = self.info.get(name)
        if ci is None:
            return None, "missing"
        k = list(ci.k)
        if not any(abs(v) > 1e-9 for v in k):
            return None, "zero"
        sx = dm.width / float(ci.width) if ci.width else 1.0
        sy = dm.height / float(ci.height) if ci.height else 1.0
        return (k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy), "ok"

    def _wrist_pose(self, name):
        """The camera's world pose: render_pose first (it travelled with the
        frame), TF down the arm's own chain second, else None -- and None
        means the objects stay labelled in the camera's frame."""
        p = self.pose.get(name)
        if p is not None:
            t = p.pose.position
            q = p.pose.orientation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        if not name.startswith("gripper/"):
            return None
        try:
            import tf2_ros
            import rclpy.time
            if self._tf is None:
                self._tf = tf2_ros.Buffer()
                self._tf_l = tf2_ros.TransformListener(self._tf, self)
                return None
            arm = name.split("/")[1]
            tr = self._tf.lookup_transform(
                "world", "%s_camera_color_frame" % arm, rclpy.time.Time())
            t, q = tr.transform.translation, tr.transform.rotation
            return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        except Exception:                                     # noqa: BLE001
            return None

    def _try_segment(self, bgr, dep, K4, pose, plane):
        """segment_lift when it can run; the NAMED refusal when it cannot.

        Returns (objects-in-world, note) or (None, why) -- the caller then
        takes the geometric path and the cycle's provenance says which ran.
        """
        try:
            from srl_perception.segment_lift import SegRefusal, objects_in_view
            from srl_perception.table_scene import Plane
            nw, dw = plane_to_world(plane[0], plane[1], pose[0], pose[1])
            # table_scene.Plane measures height as p . n - offset; this
            # node's fit satisfies p . n + d = 0, so offset = -d.
            segs = objects_in_view(bgr, dep, K4, _pose_T(pose),
                                   Plane(nw, -dw, None, 0),
                                   source="scene_understanding")
            out = [dict(label=name_for(s.width_m * 1000,
                                       s.extents[2] * 1000)[0],
                        pickable=bool(s.graspable),
                        colour=colour_name(s.mean_bgr),
                        centre=[round(float(v), 4) for v in s.centre],
                        extents_m=[round(float(v), 4) for v in s.extents],
                        width_mm=round(s.width_m * 1000, 1),
                        height_mm=round(s.extents[2] * 1000, 1),
                        n_points=s.n_points) for s in segs]
            return out, ""
        except SegRefusal as e:
            return None, str(e)
        except Exception as e:                                # noqa: BLE001
            # Named, not swallowed: a segmenter that failed for a new reason
            # must show up in the report, and the geometric answer still ships.
            return None, "%s: %s" % (type(e).__name__, e)

    def tick(self):
        rep = {"t": time.time(),
               "segment_lift": ("available" if self.seg_ok
                                else "unavailable: %s" % self.seg_why),
               "cameras": {}}
        world_objs = []
        for name, cfg in CAMS.items():
            ent = self._camera_entry(name, cfg, world_objs)
            rep["cameras"][name] = ent
            self._publish_overlay(name, ent)
        self.pub.publish(String(data=json.dumps(rep)))
        self.mark_pub.publish(markers_for(world_objs))

    def _camera_entry(self, name, cfg, world_objs):
        cm = self.col.get(name)
        age = (time.time() - self.t[name]) if name in self.t else None
        ent = {"stamp": self.t.get(name),
               "age_s": None if age is None else round(age, 2)}
        if cm is None:
            ent["refusal"] = ("no frames on %s -- this camera has never "
                              "spoken" % " or ".join(cfg["colour"]))
            return ent
        stale = float(self.get_parameter("stale_after_s").value)
        if age is not None and age > stale:
            # NOT analysed. An old frame analysed fresh every cycle is the
            # last-good-frame republish this node exists to refuse.
            ent["refusal"] = ("%s last spoke %.1f s ago (gate %.1f s) -- "
                              "refusing to report a stale picture as current"
                              % (name, age, stale))
            return ent
        bgr = self._bgr(cm)
        if not cfg["metric"]:
            return self._colour_only_entry(name, ent, bgr)
        dm = self.dep.get(name)
        if dm is None:
            ent["refusal"] = ("colour arrived but no depth on %s"
                              % " or ".join(cfg["depth"]))
            return ent
        K4, kstat = self._k4(name, dm)
        ent["k_status"] = kstat
        if K4 is None:
            ent["refusal"] = (
                "camera_info K is %s for %s -- cannot deproject, so no "
                "metric claims from this camera" % (kstat, name))
            return ent
        dep = self._depth_m(dm)
        aligned = bgr.shape[:2] == dep.shape[:2]
        res = analyse_metric(bgr if aligned else None, dep, K4)
        if "refusal" in res:
            ent["refusal"] = "%s: %s" % (name, res["refusal"])
            return ent
        pose = self._wrist_pose(name) if name.startswith("gripper/") else None
        ent["method"] = "plane_cluster"
        if self.seg_ok and pose is not None and aligned:
            segged, why = self._try_segment(bgr, dep, K4, pose, res["plane"])
            if segged is not None:
                ent["method"] = "segment_lift(FastSAM)"
                ent["frame"] = "robot"
                ent["table"] = res["table"]
                ent["objects"] = segged
                world_objs.extend(segged)
                return ent
            ent["segment_note"] = why
        objs = res["objects"]
        if pose is not None:
            for o in objs:
                o["centre"] = [round(float(v), 4) for v in
                               to_world(o["centre"], pose[0], pose[1])[0]]
            ent["frame"] = "robot"
            world_objs.extend(objs)
        else:
            # No extrinsic exists for the room cameras, and the wrist pose can
            # be absent too (no render_pose, TF not up). Labelled, not guessed.
            ent["frame"] = "camera:%s" % name
        ent["table"] = res["table"]
        ent["objects"] = objs
        return ent

    def _publish_overlay(self, name, ent):
        """This cycle's entry, burned onto this camera's LATEST frame.

        Publishes on /perception/scene/overlay/<name> at the cycle rate --
        but only when a frame has ARRIVED since the last publish. A camera
        with no frames, a stale camera, or a camera that stopped between
        cycles publishes NOTHING: re-drawing the last frame every cycle is
        the last-good-frame republish this node exists to refuse. A camera
        that refuses for any other reason (no depth, zero K, no plane) still
        gets its overlay, with the refusal burned in verbatim, so the
        operator SEES why.
        """
        if not bool(self.get_parameter("publish_overlays").value):
            return
        cm, st = self.col.get(name), self.t.get(name)
        if cm is None or st is None:
            return
        if time.time() - st > float(self.get_parameter("stale_after_s").value):
            return
        if self._overlay_t.get(name) == st:
            return                       # no fresh frame since the last one
        k4, dshape = None, None
        dm = self.dep.get(name)
        if dm is not None:
            k4, _ = self._k4(name, dm)
            dshape = (dm.height, dm.width)
        img = draw_overlay(self._bgr(cm), ent, name=name, k4=k4,
                           depth_shape=dshape)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = "bgr8"
        msg.step = int(img.shape[1] * 3)
        msg.data = img.tobytes()
        self.overlay_pubs[name].publish(msg)
        self._overlay_t[name] = st

    def _colour_only_entry(self, name, ent, bgr):
        ci = self.info.get(name)
        if ci is None:
            why = ("no camera_info for %s -- intrinsics never calibrated"
                   % name)
        elif not any(abs(v) > 1e-9 for v in ci.k):
            why = ("scene_camera_node publishes a ZERO K until calibrated -- "
                   "pixel boxes only, no metres")
        else:
            why = ("intrinsics exist but no depth sensor and no measured "
                   "extrinsic to the robot")
        ent.update(colour_report(bgr, self.detector))
        ent["frame"] = "camera:%s (pixels)" % name
        ent["why_no_metric"] = why
        return ent


def _pose_T(pose):
    """(t, quat) -> 4x4, the shape `pick_from_table` expects."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(pose[1])
    T[:3, 3] = np.asarray(pose[0], float)
    return T


def main(args=None):
    rclpy.init(args=args)
    n = SceneUnderstanding()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
