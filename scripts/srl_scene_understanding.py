#!/usr/bin/env python3
"""Find the TABLE, then what is ON it -- named, in 3D, in every depth camera.

    python3 scripts/srl_scene_understanding.py            # window
    python3 scripts/srl_scene_understanding.py --save out.png

WHY TABLE FIRST, AND NOT COLOUR FIRST
-------------------------------------
Colour alone cannot tell a cube from a shirt. The mannequin is green in
places, the backdrop is green under some white balance, and a colour gate
locked onto a 268 mm "cube" on this rig for exactly that reason. What
separates an object from the room is that it STANDS ON A SURFACE: fit the
dominant plane, keep the points above it, cluster those, and every cluster is
a thing on the table whether or not anybody predicted its colour.

So each depth camera does, per frame:

    1. RANSAC the dominant plane            -> the TABLE
    2. keep points 8 mm .. 300 mm above it  -> things standing on it
    3. cluster them in the plane            -> one cluster per object
    4. measure each cluster                 -> centre, extents, colour, name

The name is DERIVED from the measurement, never declared: a cluster 20-90 mm
across its narrowest axis and under 90 mm tall is a `cube`; wider than 100 mm
is a `box`; the plane itself is the `table`. The gripper's own jaws close on
85 mm, so `cube` and `box` is exactly the distinction that decides whether a
thing is pickable, and it is measured rather than assumed.

WHAT THE CAMERAS SAY TO EACH OTHER
----------------------------------
Each camera reports in ITS OWN frame, and only the wrist cameras can convert
to robot coordinates -- their frame's pose is FK down a single chain. The
room cameras have no measured extrinsic to the robot, so their 3D is reported
in their own frame and clearly labelled that way. They still matter: they see
the table and the cube whether or not an arm is pointed at it, and they are
the independent witness that the wrist camera's answer is about the right
object. Fusing them into one world frame needs a calibration this rig has not
measured, and inventing one would be worse than doing without.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import cv2  # noqa: E402
from srl_fk import FK  # noqa: E402

TILE_W, TILE_H = 640, 360
ABOVE_MIN, ABOVE_MAX = 0.008, 0.30      # what counts as "standing on it"
CLUSTER_M = 0.035                        # merge radius in the table plane
MIN_PTS = 40

#: (tile, colour topic, depth topic, info topic, metric?)
CAMS = [
    ("scene/usb", "/scene/usb/image_raw", None, None, False),
    ("scene/rs", "/scene/rs/color/image_raw", "/scene/rs/depth/image_raw",
     "/scene/rs/color/camera_info", True),
    ("gripper/left", "/left_camera/color/image_raw",
     "/left_camera/depth/image_raw", "/left_camera/depth/camera_info", True),
    ("gripper/right", "/right_camera/color/image_raw",
     "/right_camera/depth/image_raw", "/right_camera/depth/camera_info", True),
]
PALETTE = [("green", (40, 70, 40), (85, 255, 255)),
           ("blue", (95, 90, 40), (130, 255, 255)),
           ("red", (0, 110, 60), (8, 255, 255)),
           ("red", (170, 110, 60), (179, 255, 255)),
           ("yellow", (22, 110, 90), (34, 255, 255))]


def fit_plane(P, iters=140, tol=0.008, seed=0):
    """Dominant plane by RANSAC, refined by PCA. (n, d, inliers) or None."""
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
    # POINT THE NORMAL AWAY FROM THE CAMERA so "above the table" is a sign
    # rather than a guess. The camera is at the origin of this frame.
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
    """What a thing of this size IS, in one word, and whether it is pickable.

    The jaws close on 85 mm, so this is the distinction that decides whether
    the robot can pick it -- measured, not declared.
    """
    if width_mm < 15:
        return "speck", False
    if width_mm <= 90 and height_mm <= 90:
        return "cube", width_mm <= 85
    if width_mm <= 200:
        return "box", False
    return "large object", False


class Scene(Node):
    def __init__(self):
        super().__init__("srl_scene_understanding")
        self.col, self.dep, self.K, self.t = {}, {}, {}, {}
        self.ck = {}          # COLOUR intrinsics, for projecting boxes
        self.js = {}
        # THE WRIST DEPTH AND COLOUR ARE SEPARATE, UNALIGNED SENSORS.
        #
        # 480x270 depth against 1280x720 colour, ~27 mm apart. Projecting a
        # depth-derived point into the colour image by scaling pixel
        # coordinates puts every box in the wrong place -- measured here: the
        # cube's box landed in the top-left corner while the cube sat bottom
        # centre. The URDF carries the baseline between the two frames, so the
        # points are TRANSFORMED, not rescaled.
        #
        # The RealSense is different: this rig's node ALIGNS depth to colour
        # before publishing, so for that camera the transform is identity and
        # the colour K is the depth K.
        self.fk = FK()
        from sensor_msgs.msg import JointState
        self.create_subscription(
            JointState, "/real/joint_states",
            lambda m: self.js.update(dict(zip(m.name, m.position))), 20)
        for name, ct, dt, it, metric in CAMS:
            self.create_subscription(
                Image, ct, lambda m, k=name: (self.col.__setitem__(k, m),
                                              self.t.__setitem__(k, time.time())),
                qos_profile_sensor_data)
            if dt:
                self.create_subscription(
                    Image, dt, lambda m, k=name: self.dep.__setitem__(k, m),
                    qos_profile_sensor_data)
            if it:
                self.create_subscription(
                    CameraInfo, it, lambda m, k=name: self.K.__setitem__(k, m),
                    qos_profile_sensor_data)
                self.create_subscription(
                    CameraInfo, ct.rsplit("/", 1)[0] + "/camera_info",
                    lambda m, k=name: self.ck.__setitem__(k, m),
                    qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/srl/scene", 10)

    @staticmethod
    def _bgr(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else a

    def to_colour_px(self, name, C):
        """3-D points in the DEPTH frame -> pixels in the COLOUR image.

        For a wrist camera this is a real transform: the URDF carries the
        baseline between camera_depth_frame and camera_color_frame, and the
        two sensors have different resolutions and different intrinsics.
        Rescaling pixel coordinates instead -- which is what this did -- put
        the cube's box in the opposite corner of the frame from the cube.

        For the RealSense the node publishes depth ALREADY ALIGNED to colour,
        so the transform is identity by construction.
        """
        ck = self.ck.get(name)
        if ck is None:
            return None, None
        P = np.asarray(C, float)
        if name.startswith("gripper/"):
            arm = name.split("/")[1]
            names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
            q = [self.js.get(x) for x in names]
            if all(v is not None for v in q):
                Td, Tc = self.fk.poses(arm, np.array(q),
                                       ["camera_depth_frame",
                                        "camera_color_frame"])
                T = np.linalg.inv(Tc) @ Td
                P = (T @ np.c_[P, np.ones(len(P))].T).T[:, :3]
        z = np.maximum(P[:, 2], 1e-6)
        uu = P[:, 0] * ck.k[0] / z + ck.k[2]
        vv = P[:, 1] * ck.k[4] / z + ck.k[5]
        return uu, vv

    def colour_at(self, bgr, uu, vv):
        """The palette name at these pixels, or 'plain'."""
        h, w = bgr.shape[:2]
        u = np.clip(uu.astype(int), 0, w - 1)
        v = np.clip(vv.astype(int), 0, h - 1)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        px = hsv[v, u]
        best, bn = 0, "plain"
        for nm, lo, hi in PALETTE:
            m = np.all((px >= np.array(lo)) & (px <= np.array(hi)), axis=1)
            if m.sum() > best:
                best, bn = int(m.sum()), nm
        return bn if best > len(u) * 0.25 else "plain"

    def analyse(self, name):
        """Table + objects for one camera. Returns (table, [objects])."""
        cm, dm, ki = self.col.get(name), self.dep.get(name), self.K.get(name)
        if cm is None or dm is None or ki is None:
            return None, []
        bgr = self._bgr(cm)
        dep = np.frombuffer(dm.data, np.uint16).reshape(
            dm.height, dm.width).astype(np.float32) * 0.001
        fx, fy, cx, cy = ki.k[0], ki.k[4], ki.k[2], ki.k[5]
        # scale the intrinsics if the depth image is a different size
        sx, sy = dm.width / ki.width, dm.height / ki.height
        fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
        v, u = np.nonzero((dep > 0.10) & (dep < 4.0))
        if len(v) < 400:
            return None, []
        step = max(1, len(v) // 20000)          # keep it real-time
        v, u = v[::step], u[::step]
        Z = dep[v, u]
        P = np.stack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z], 1)
        pl = fit_plane(P)
        if pl is None:
            return None, []
        n, d, inl = pl
        h = P @ n + d
        table = dict(n=[round(float(x), 4) for x in n], d=round(float(d), 4),
                     inliers=int(inl.sum()),
                     rms_mm=round(float(np.sqrt((h[inl] ** 2).mean()) * 1000), 2))
        on = (h > ABOVE_MIN) & (h < ABOVE_MAX)
        if on.sum() < MIN_PTS:
            return table, []
        # coordinates IN the plane, so clustering is 2-D and objects that
        # touch in the image but sit apart on the table stay separate
        e1 = np.cross(n, [0, 0, 1.0])
        e1 = e1 / (np.linalg.norm(e1) + 1e-9)
        e2 = np.cross(n, e1)
        Q = P[on]
        xy = np.stack([Q @ e1, Q @ e2], 1)
        lab = cluster(xy)
        objs = []
        for k in range(lab.max() + 1 if len(lab) else 0):
            sel = lab == k
            if sel.sum() < MIN_PTS:
                continue
            C = Q[sel]
            hh = h[on][sel]
            ex1, ex2 = C @ e1, C @ e2
            width_mm = float(min(ex1.ptp(), ex2.ptp()) * 1000)
            height_mm = float(hh.max() * 1000)
            # EXACT, in the frame the points came from. No transform, so no
            # transform error: these pixels are where the depth camera itself
            # saw the object. The colour projection below is kept for
            # reference and is the one that carries the ~8.45 deg mount error
            # this rig documents.
            du = C[:, 0] * fx / C[:, 2] + cx
            dv = C[:, 1] * fy / C[:, 2] + cy
            uu, vv = self.to_colour_px(name, C)
            if uu is None:
                uu, vv = du * (1.0 / sx), dv * (1.0 / sy)
            nm, pickable = name_for(width_mm, height_mm)
            if nm == "speck":
                continue
            objs.append(dict(
                name=nm, pickable=bool(pickable),
                colour=self.colour_at(bgr, uu, vv),
                centre_cam=[round(float(x), 4) for x in C.mean(0)],
                width_mm=round(width_mm, 1), height_mm=round(height_mm, 1),
                n_points=int(sel.sum()),
                bbox=[int(uu.min()), int(vv.min()),
                      int(uu.max()), int(vv.max())],
                bbox_depth=[int(du.min()), int(dv.min()),
                            int(du.max()), int(dv.max())],
                depth_wh=[int(dm.width), int(dm.height)]))
        objs.sort(key=lambda o: -o["n_points"])
        return table, objs[:8]

    ARM_LINKS = ["base_link", "shoulder_link", "half_arm_1_link",
                 "half_arm_2_link", "forearm_link", "spherical_wrist_1_link",
                 "spherical_wrist_2_link", "bracelet_link",
                 "end_effector_link"]

    def arm_links_world(self, arm):
        """Every joint link's 3-D position in WORLD, from FK.

        THIS IS THE ONE THING ON THIS RIG THAT NEEDS NO CAMERA AND NO
        CALIBRATION. The arm's own encoders plus the URDF give each link's
        pose exactly; asking a camera to find the arm would be strictly worse
        and would need an extrinsic nobody has measured.
        """
        names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        q = [self.js.get(x) for x in names]
        if any(v is None for v in q):
            return None
        T = self.fk.poses(arm, np.array(q), self.ARM_LINKS)
        return [dict(link=ln, p=[round(float(v), 4) for v in M[:3, 3]])
                for ln, M in zip(self.ARM_LINKS, T)]

    def wearer_world(self):
        """The mannequin/person's body parts in WORLD, from the URDF.

        The body is IN the robot model -- torso, head, hips and both arms are
        real links with real poses. So its 3-D position is known exactly and
        does not depend on a camera seeing it, which matters because the
        clearance floor is enforced against this body and a camera that
        fails to see somebody must never read as "nobody is there".
        """
        parts = ["torso", "head", "hips",
                 "human_left_upper_arm", "human_right_upper_arm",
                 "human_left_lower_arm", "human_right_lower_arm",
                 "human_left_hand", "human_right_hand"]
        out = []
        for pnm in parts:
            if pnm not in self.fk.links:
                continue
            try:
                M = self.fk.poses("left", np.zeros(7), [pnm])[0]
            except Exception:                                 # noqa: BLE001
                continue
            out.append(dict(part=pnm,
                            p=[round(float(v), 4) for v in M[:3, 3]]))
        return out

    def tick(self):
        rep = {"t": time.time(), "cameras": {}}
        rep["arms"] = {a: self.arm_links_world(a) for a in ("left", "right")}
        rep["wearer"] = self.wearer_world()
        for name, _, dt, _, metric in CAMS:
            if not metric:
                rep["cameras"][name] = dict(metric=False, table=None,
                                            objects=[])
                continue
            table, objs = self.analyse(name)
            rep["cameras"][name] = dict(
                metric=True, frame="%s camera frame" % name,
                table=table, objects=objs)
        self.pub.publish(String(data=json.dumps(rep)))
        return rep


def draw(node, rep, name, on_depth=True):
    """One tile. On a metric camera the DEPTH image is drawn, because the
    boxes were measured in it -- so they land exactly, with no transform in
    between. Projecting them into the colour image instead is what left every
    box offset from its object."""
    c = rep["cameras"].get(name, {})
    dm = node.dep.get(name)
    use_depth = on_depth and c.get("metric") and dm is not None
    canvas = np.zeros((TILE_H, TILE_W, 3), np.uint8)
    if use_depth:
        d = np.frombuffer(dm.data, np.uint16).reshape(
            dm.height, dm.width).astype(np.float32) * 0.001
        vis = np.clip(d / 2.5 * 255, 0, 255).astype(np.uint8)
        img = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
        img[d <= 0] = 0
    else:
        cm = node.col.get(name)
        if cm is None:
            cv2.putText(canvas, "%s: no frame" % name, (14, TILE_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 200), 2)
            return canvas
        img = node._bgr(cm)
    sx, sy = TILE_W / img.shape[1], TILE_H / img.shape[0]
    canvas = cv2.resize(img, (TILE_W, TILE_H))
    tbl, objs = c.get("table"), c.get("objects", [])
    for o in objs:
        box = o.get("bbox_depth") if use_depth else o.get("bbox")
        if not box:
            continue
        x1, y1, x2, y2 = [int(v * (sx if i % 2 == 0 else sy))
                          for i, v in enumerate(box)]
        col = ((70, 230, 70) if o["pickable"] else (60, 170, 235))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), col, 2)
        txt = "%s %s  %.0fx%.0fmm  %.2fm" % (
            o["colour"], o["name"], o["width_mm"], o["height_mm"],
            o["centre_cam"][2])
        cv2.rectangle(canvas, (x1, max(0, y1 - 16)), (x1 + 8 * len(txt), y1),
                      (0, 0, 0), -1)
        cv2.putText(canvas, txt, (x1 + 2, max(11, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
        if o["pickable"]:
            cv2.putText(canvas, "PICKABLE", (x1 + 2, y2 + 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (70, 230, 70), 1)
    age = time.time() - node.t.get(name, 0)
    live = age < 1.5
    head = "%s%s  %s" % (name, " [DEPTH]" if use_depth else "",
                         "LIVE" if live else "STALE %.0fs" % age)
    if c.get("metric"):
        head += ("  |  TABLE rms %.1fmm, %d pts" % (tbl["rms_mm"],
                                                    tbl["inliers"])
                 if tbl else "  |  no table found")
        head += "  |  %d on it" % len(objs)
    else:
        head += "  |  colour only, no depth"
    cv2.rectangle(canvas, (0, 0), (TILE_W, 22), (0, 0, 0), -1)
    cv2.putText(canvas, head, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (90, 235, 90) if live else (60, 60, 220), 1, cv2.LINE_AA)
    return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default="")
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Scene()
    t0 = time.time()
    while time.time() - t0 < 4:
        rclpy.spin_once(n, timeout_sec=0.05)
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            rep = n.tick()
            grid = np.vstack([
                np.hstack([draw(n, rep, "scene/usb"), draw(n, rep, "scene/rs")]),
                np.hstack([draw(n, rep, "gripper/left"),
                           draw(n, rep, "gripper/right")])])
            if a.save:
                if time.time() - last > 1.0:
                    cv2.imwrite(a.save, grid)
                    last = time.time()
                if a.once:
                    for cam, c in rep["cameras"].items():
                        if c.get("table"):
                            print("%-14s table rms %.1f mm | %d object(s)"
                                  % (cam, c["table"]["rms_mm"],
                                     len(c["objects"])))
                            for o in c["objects"]:
                                print("     %-6s %-6s %5.0f x %5.0f mm  "
                                      "%.2f m  %s"
                                      % (o["colour"], o["name"], o["width_mm"],
                                         o["height_mm"], o["centre_cam"][2],
                                         "PICKABLE" if o["pickable"] else ""))
                    break
            else:
                cv2.imshow("SRL scene understanding", grid)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
