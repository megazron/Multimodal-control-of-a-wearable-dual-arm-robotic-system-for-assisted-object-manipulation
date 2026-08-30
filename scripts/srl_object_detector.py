#!/usr/bin/env python3
"""Detect objects continuously, in every camera this rig has, at once.

    python3 scripts/srl_object_detector.py --arm right

Publishes `/srl/detections` (JSON in a std_msgs/String) at 5 Hz for as long as
it runs.  One message carries what every camera currently sees, each detection
tagged with the camera that produced it and -- this is the point -- WHAT FRAME
ITS COORDINATES ARE IN, because the three cameras answer three different
questions and are trustworthy to very different degrees.

  scene/usb    colour only, no calibration.  Gives PIXELS.  Answers "is there
               a green object on the table at all", which is worth having
               because it is true whether or not the arm is looking.
  scene/rs     colour + metric depth, intrinsics read from the device.  Gives
               METRES IN THE REALSENSE'S OWN FRAME.  There is no measured
               extrinsic from that frame to the robot, so these coordinates
               are NOT robot coordinates and are never treated as such.
  gripper      the wrist camera.  Gives METRES IN THE WRIST DEPTH FRAME, and
               that frame's pose in world is FK down a single chain, so it is
               the only one that converts to a robot coordinate.

WHY THE SCENE CAMERAS ARE STILL WORTH RUNNING WITH NO EXTRINSIC.  The wrist
camera cannot see the table unless the arm is already pointed at it, so
"there is nothing there" and "I am not looking" are the same observation to
it -- and that ambiguity is what made an earlier scan report no cube at 14
viewpoints in a row.  A scene camera resolves it from across the room without
moving the robot.  It is a PRESENCE and COUNT sensor here, not a metric one,
and the schema says so per detection with `metric: true|false`.

THE HSV GATE IS PER CAMERA ON PURPOSE.  The three sensors have different white
balance -- the RealSense runs noticeably cooler than the webcam on this rig --
so one shared threshold either loses the cube in one image or floods another.
"""
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import cv2  # noqa: E402

# (name, lo_hsv, hi_hsv). OpenCV hue is 0-179.
PALETTE = [
    ("green", (40, 70, 40), (85, 255, 255)),
    ("blue", (95, 90, 40), (130, 255, 255)),
    ("red", (0, 110, 60), (8, 255, 255)),
    ("red2", (170, 110, 60), (179, 255, 255)),
    ("yellow", (22, 110, 90), (34, 255, 255)),
]
MIN_AREA_PX = 90


def blobs(bgr, gate_v=0):
    """Coloured blobs in one image, largest first, as pixel detections."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = []
    for name, lo, hi in PALETTE:
        lo = (lo[0], lo[1], max(lo[2], gate_v))
        m = cv2.inRange(hsv, lo, hi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
        for i in range(1, n):
            a = int(stats[i, cv2.CC_STAT_AREA])
            if a < MIN_AREA_PX:
                continue
            w, h = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
            out.append({
                "colour": "red" if name == "red2" else name,
                "u": float(cent[i][0]), "v": float(cent[i][1]),
                "area_px": a, "w_px": w, "h_px": h,
                # a cube is roughly square in the image; a cable or a shadow
                # edge is not, and this is the cheapest way to say so
                "aspect": float(w) / max(h, 1),
                "mask_i": i, "_lab": lab,
            })
    out.sort(key=lambda d: -d["area_px"])
    return out


class Detector(Node):
    def __init__(self, arm="right", hz=5.0):
        super().__init__("srl_object_detector")
        self.arm = arm
        self.msg = {}
        self.q = None

        def sub(T, t, k):
            self.create_subscription(
                T, t, lambda m, k=k: self.msg.__setitem__(k, m),
                qos_profile_sensor_data)

        sub(Image, "/scene/usb/image_raw", "usb")
        sub(Image, "/scene/rs/color/image_raw", "rs_c")
        sub(Image, "/scene/rs/depth/image_raw", "rs_d")
        sub(CameraInfo, "/scene/rs/color/camera_info", "rs_k")
        sub(Image, "/%s_camera/color/image_raw" % arm, "gc")
        sub(CameraInfo, "/%s_camera/color/camera_info" % arm, "gck")
        sub(Image, "/%s_camera/depth/image_raw" % arm, "gd")
        sub(CameraInfo, "/%s_camera/depth/camera_info" % arm, "gdk")
        self.create_subscription(JointState, "/real/joint_states", self._js, 10)

        self.pub = self.create_publisher(String, "/srl/detections", 10)
        self.fk = None
        self.names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        self.create_timer(1.0 / hz, self.tick)
        self.n = 0

    def _js(self, m):
        z = dict(zip(m.name, m.position))
        if all(n in z for n in self.names):
            self.q = np.array([z[n] for n in self.names])

    @staticmethod
    def _img(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        return a

    @staticmethod
    def _depth_m(m):
        return np.frombuffer(m.data, np.uint16).reshape(
            m.height, m.width).astype(np.float32) * 0.001

    def _deproject(self, dets, dep, K, near, far):
        """Attach metres to pixel detections using an ALIGNED depth image.

        The depth taken is the MEDIAN over the blob's own mask, not the value
        at the centroid.  A centroid pixel that happens to land on a depth
        dropout returns 0 and reads as "the object is at the camera", which is
        a detection that looks perfectly well-formed and is nonsense.
        """
        fx, fy, cx, cy = K.k[0], K.k[4], K.k[2], K.k[5]
        out = []
        for d in dets:
            lab = d.pop("_lab")
            i = d.pop("mask_i")
            if dep.shape != lab.shape:
                continue
            z = dep[(lab == i) & (dep > near) & (dep < far)]
            if z.size < 12:
                d["metric"] = False
                out.append(d)
                continue
            Z = float(np.median(z))
            d.update(metric=True, z_m=Z,
                     xyz_cam=[float((d["u"] - cx) * Z / fx),
                              float((d["v"] - cy) * Z / fy), Z],
                     depth_px=int(z.size))
            out.append(d)
        return out

    def tick(self):
        rep = {"t": time.time(), "seq": self.n, "cameras": {}}
        self.n += 1

        m = self.msg.get("usb")
        if m is not None:
            ds = blobs(self._img(m))
            for d in ds:
                d.pop("_lab", None)
                d.pop("mask_i", None)
                d["metric"] = False
            rep["cameras"]["scene/usb"] = {
                "frame": "pixels", "metric": False, "n": len(ds),
                "detections": ds[:8]}

        if self.msg.get("rs_c") is not None and self.msg.get("rs_d") is not None \
                and self.msg.get("rs_k") is not None:
            col = self._img(self.msg["rs_c"])
            dep = self._depth_m(self.msg["rs_d"])
            ds = self._deproject(blobs(col), dep, self.msg["rs_k"], 0.20, 6.0)
            rep["cameras"]["scene/rs"] = {
                "frame": "scene_rs_color_frame", "metric": True, "n": len(ds),
                "detections": ds[:8]}

        if self.msg.get("gc") is not None and self.msg.get("gd") is not None \
                and self.msg.get("gdk") is not None and self.msg.get("gck") is not None:
            col = self._img(self.msg["gc"])
            dep = self._depth_m(self.msg["gd"])
            ds = blobs(col)
            # The wrist colour and depth images are DIFFERENT SIZES (1280x720
            # against 480x270) and are not aligned, so a colour pixel is not a
            # depth pixel.  Project the blob centroid through the colour K into
            # the depth image using the URDF baseline between the two frames.
            gck, gdk = self.msg["gck"], self.msg["gdk"]
            out = []
            for d in ds:
                d.pop("_lab", None)
                d.pop("mask_i", None)
                uu = d["u"] / gck.width * gdk.width
                vv = d["v"] / gck.height * gdk.height
                r = 6
                y0, y1 = int(max(vv - r, 0)), int(min(vv + r, dep.shape[0]))
                x0, x1 = int(max(uu - r, 0)), int(min(uu + r, dep.shape[1]))
                patch = dep[y0:y1, x0:x1]
                z = patch[(patch > 0.08) & (patch < 1.5)]
                if z.size >= 8:
                    Z = float(np.median(z))
                    fx, fy = gdk.k[0], gdk.k[4]
                    cx, cy = gdk.k[2], gdk.k[5]
                    d.update(metric=True, z_m=Z,
                             xyz_cam=[float((uu - cx) * Z / fx),
                                      float((vv - cy) * Z / fy), Z],
                             depth_px=int(z.size))
                else:
                    d["metric"] = False
                out.append(d)
            rep["cameras"]["gripper/%s" % self.arm] = {
                "frame": "%s_camera_depth_frame" % self.arm,
                "metric": True, "n": len(out), "detections": out[:8]}

        rep["arm_q_deg"] = (None if self.q is None
                            else [round(float(np.degrees(v)), 2) for v in self.q])
        self.pub.publish(String(data=json.dumps(rep)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="right")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--print", action="store_true",
                    help="also print a one-line summary per report")
    a = ap.parse_args()
    rclpy.init()
    node = Detector(arm=a.arm, hz=a.hz)
    if a.print:
        def show(m):
            r = json.loads(m.data)
            bits = []
            for cam, c in r["cameras"].items():
                top = c["detections"][0] if c["detections"] else None
                bits.append("%s n=%d%s" % (
                    cam, c["n"],
                    "" if not top else " top=%s@%.0f,%.0f%s" % (
                        top["colour"], top["u"], top["v"],
                        "" if not top.get("metric") else
                        " %.3fm" % top["z_m"])))
            print("[%4d] %s" % (r["seq"], " | ".join(bits)), flush=True)
        node.create_subscription(String, "/srl/detections", show, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
