#!/usr/bin/env python3
"""Every camera, live, with what the robot is detecting drawn on it.

    python3 scripts/srl_vision_view.py

One window, one tile per camera, annotated continuously:

    scene/usb        the room camera -- sees the table, the cube and the arms
    scene/rs         the RealSense -- same view, WITH metric depth
    gripper/left     the left wrist camera -- what that hand sees
    gripper/right    the right wrist camera

WHAT IS DRAWN, AND WHY EACH THING IS THERE
------------------------------------------
  CUBE-SIZED BLOBS    boxed in green with their measured size. On a metric
                      camera the distance is printed with them, so "green
                      thing" and "green thing 3.2 m away" are distinguishable
                      -- which matters, because the mannequin's shirt and the
                      backdrop are also green and a 268 mm "cube" was locked
                      onto once for exactly this reason.
  EVERYTHING ELSE     boxed dimly. A detector that only draws what it likes
                      cannot be caught mistaking one thing for another; the
                      rejects are the evidence that the gate is working.
  THE REJECT REASON   written on the box. "too big", "too small", "no depth".

NOTHING HERE COMMANDS ANYTHING. It is a window onto `/srl/detections` and the
raw image topics, so it can be left running beside a pick without being in
the pick's way -- and if it dies, nothing on the robot changes.
"""
import argparse
import json
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

TILE_W, TILE_H = 640, 360
CUBE_MM = (20.0, 90.0)

#: (tile name, image topic, camera_info topic or None)
FEEDS = [
    ("scene/usb", "/scene/usb/image_raw", None),
    ("scene/rs", "/scene/rs/color/image_raw", "/scene/rs/color/camera_info"),
    ("gripper/left", "/left_camera/color/image_raw",
     "/left_camera/color/camera_info"),
    ("gripper/right", "/right_camera/color/image_raw",
     "/right_camera/color/camera_info"),
]
COLOURS = {"green": (80, 220, 80), "blue": (230, 140, 60),
           "red": (60, 60, 235), "yellow": (60, 220, 235)}


class Viewer(Node):
    def __init__(self):
        super().__init__("srl_vision_view")
        self.img = {}
        self.det = None
        self.t = {}
        # THE FOCAL LENGTH IS READ FROM THE CAMERA, NOT ASSUMED.
        #
        # This tile used a hard-coded 600 px to turn a pixel width into
        # millimetres. The Kinova wrist colour camera is fx = 1297.67 -- more
        # than double -- so a 40 mm cube was drawn as "129mm" and labelled
        # NOT CUBE-SIZED. The detection was right; the annotation was lying
        # about it, which is the worst way for a display to be wrong.
        self.fx = {}
        for name, topic, info in FEEDS:
            self.create_subscription(
                Image, topic,
                lambda m, k=name: (self.img.__setitem__(k, m),
                                   self.t.__setitem__(k, time.time())),
                qos_profile_sensor_data)
            if info:
                self.create_subscription(
                    CameraInfo, info,
                    lambda m, k=name: self.fx.__setitem__(k, float(m.k[0])),
                    qos_profile_sensor_data)
        self.create_subscription(String, "/srl/detections", self._det, 10)

    def _det(self, m):
        try:
            self.det = json.loads(m.data)
        except Exception:                                     # noqa: BLE001
            pass

    @staticmethod
    def _bgr(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else a

    def tile(self, name):
        """One annotated tile, always the same size so the grid never jumps."""
        m = self.img.get(name)
        canvas = np.zeros((TILE_H, TILE_W, 3), np.uint8)
        if m is None:
            cv2.putText(canvas, "%s: no frame" % name, (14, TILE_H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 200), 2)
            return canvas
        img = self._bgr(m)
        sx, sy = TILE_W / img.shape[1], TILE_H / img.shape[0]
        canvas = cv2.resize(img, (TILE_W, TILE_H))

        cams = (self.det or {}).get("cameras", {})
        cam = cams.get(name) or {}
        n_cube = 0
        for d in cam.get("detections", []):
            u, v = d["u"] * sx, d["v"] * sy
            w, h = max(6, d["w_px"] * sx), max(6, d["h_px"] * sy)
            p1 = (int(u - w / 2), int(v - h / 2))
            p2 = (int(u + w / 2), int(v + h / 2))
            metric = d.get("metric") and d.get("z_m")
            # IS THIS CUBE-SIZED? On a metric camera the answer is a real
            # measurement; without depth it can only be guessed from pixels,
            # and saying which is which is the point.
            label, ok = d["colour"], False
            if metric:
                fx = self.fx.get(name)
                if not fx:
                    label = "%s %.2fm (no K yet)" % (d["colour"], d["z_m"])
                    cv2.rectangle(canvas, p1, p2, (150, 150, 150), 1)
                    continue
                mm = d["w_px"] * d["z_m"] / fx * 1000.0
                ok = CUBE_MM[0] <= mm <= CUBE_MM[1]
                label = "%s %.0fmm %.2fm" % (d["colour"], mm, d["z_m"])
                if not ok:
                    label += "  (not cube-sized)"
            else:
                label = "%s %dpx (no depth)" % (d["colour"], d["w_px"])
            col = COLOURS.get(d["colour"], (180, 180, 180))
            if not ok:
                col = tuple(int(c * 0.45) for c in col)      # dim the rejects
            else:
                n_cube += 1
            cv2.rectangle(canvas, p1, p2, col, 2 if ok else 1)
            cv2.putText(canvas, label, (p1[0], max(12, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

        age = time.time() - self.t.get(name, 0)
        head = "%s   %s   %d det  %d cube-sized" % (
            name, "LIVE" if age < 1.5 else "STALE %.0fs" % age,
            cam.get("n", 0), n_cube)
        cv2.rectangle(canvas, (0, 0), (TILE_W, 22), (0, 0, 0), -1)
        cv2.putText(canvas, head, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (90, 235, 90) if age < 1.5 else (60, 60, 220), 1,
                    cv2.LINE_AA)
        return canvas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default="",
                    help="also write the grid here as a PNG, once a second")
    a = ap.parse_args()
    rclpy.init()
    n = Viewer()
    headless = a.save != ""
    last = 0.0
    print("showing 4 cameras. q or Esc to quit.")
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            grid = np.vstack([
                np.hstack([n.tile("scene/usb"), n.tile("scene/rs")]),
                np.hstack([n.tile("gripper/left"), n.tile("gripper/right")])])
            if headless:
                if time.time() - last > 1.0:
                    cv2.imwrite(a.save, grid)
                    last = time.time()
            else:
                cv2.imshow("SRL cameras -- live detection", grid)
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
