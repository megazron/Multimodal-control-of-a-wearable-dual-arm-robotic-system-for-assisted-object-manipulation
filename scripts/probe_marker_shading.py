#!/usr/bin/env python3
"""WHAT COLOUR DOES A MARKER ACTUALLY RENDER AS? Measured, not assumed.

    python3 scripts/probe_marker_shading.py --out /tmp/shading

THE QUESTION. `clip_scene.OAK` asks for (0.94, 0.94, 0.95) and TASK_SPEC calls
the table WHITE. Sampled out of the shipped 2026-08-15 T1 clip, the table top
renders at RGB (118, 118, 118) -- mid grey -- while the arm's own meshes in
the same frame reach (208, 207, 210). So "white" is being requested and grey
is being delivered, and no check in this repository looks at that: the clip
verifier keys on channel DIFFERENCES (R-B, G-R) and a neutral fires none of
them, whatever its brightness.

WHAT IT MEASURES. One flat slab where the table is, at a series of requested
colours, one frame grabbed per colour off the same Xvfb display the clips are
recorded from, and the modal colour of the slab read back. That gives the
renderer's actual response curve and says plainly whether a white table is
available at all or whether the ceiling is lower.

CONTROLS:
    a BLACK slab            must render darker than a white one, or the probe
                            is reading the background rather than the slab
    the background          must be the RViz background, not the slab
    a RED slab              must come back with R > G, so the probe is reading
                            a colour and not a fixed pixel
"""
import argparse
import collections
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import record_rviz as rr                                       # noqa: E402
import clip_scene as CS                                        # noqa: E402

SWATCHES = [
    ("black", (0.0, 0.0, 0.0)),
    ("shipped_oak", (0.94, 0.94, 0.95)),
    ("full_white", (1.0, 1.0, 1.0)),
    ("red", (1.0, 0.0, 0.0)),
    # OVER-UNITY, AND IT IS A REAL QUESTION RATHER THAN A TRICK. An up-facing
    # marker face gets the ambient term alone, which is half the requested
    # colour, so 1.0 renders 127 and a white table top is not available at
    # any colour a person would write down. std_msgs/ColorRGBA is float32
    # with no stated upper bound, and Ogre materials do not clamp before
    # lighting -- so if RViz passes the value through, 2.0 asks for an
    # ambient of 1.0 and the top saturates at 255. If it clamps, these two
    # come back identical to full_white and the ceiling is real.
    ("over_1p5", (1.5, 1.5, 1.5)),
    ("over_2p0", (2.0, 2.0, 2.0)),
]


class Pub(Node):
    def __init__(self):
        super().__init__("probe_marker_shading")
        q = QoSProfile(depth=4)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, "/task_objects", q)

    def show(self, rgb):
        A = MarkerArray()
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "scene"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        tyc = (CS.TABLE_NEAR_Y + CS.TABLE_FAR_Y) / 2.0
        m.pose.position.x = 0.0
        m.pose.position.y = float(tyc)
        m.pose.position.z = float(CS.TABLE_TOP - CS.TABLE_THICK / 2.0)
        m.pose.orientation.w = 1.0
        m.scale.x = float(2 * CS.TABLE_HALF_X)
        m.scale.y = float(CS.TABLE_FAR_Y - CS.TABLE_NEAR_Y)
        m.scale.z = float(CS.TABLE_THICK)
        m.color.r, m.color.g, m.color.b = (float(v) for v in rgb)
        m.color.a = 1.0
        A.markers.append(m)
        for _ in range(4):
            self.pub.publish(A)
            rclpy.spin_once(self, timeout_sec=0.05)


def grab(disp, path):
    subprocess.run([rr.FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
                    "-video_size", "%dx%d" % (rr.VW, rr.VH),
                    "-i", "%s.0" % disp, "-frames:v", "1", path],
                   capture_output=True)
    return os.path.exists(path)


def modal(path, box):
    from PIL import Image
    im = Image.open(path).convert("RGB")
    px = im.load()
    c = collections.Counter()
    for y in range(box[1], box[3]):
        for x in range(box[0], box[2]):
            c[px[x, y]] += 1
    return c.most_common(4)


def faces(path, bg=(45, 45, 48)):
    """(brightest slab pixel, most common slab pixel) down one column.

    TWO NUMBERS, NOT ONE, AND THE DIFFERENCE IS THE WHOLE ANSWER. The slab's
    TOP faces up and its near edge faces the camera, and they do not render
    the same: at full white the top comes back 127 and the near edge 221. A
    probe that reports only the modal colour reports the top -- it is most of
    the pixels -- and would have concluded that a marker simply cannot exceed
    half brightness, when what cannot exceed it is a face the headlight
    grazes.
    """
    from PIL import Image
    im = Image.open(path).convert("RGB")
    px = im.load()
    w, h = im.size
    col = w // 2
    vals = collections.Counter()
    best = None
    for y in range(int(h * 0.55), h - 30):
        p = px[col, y]
        if p == bg or sum(p) < 12:
            continue
        vals[p] += 1
        if best is None or sum(p) > sum(best):
            best = p
    return best, (vals.most_common(1)[0][0] if vals else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        "/tmp/scratch/"
        "56c2ed37-1c90-48cc-88af-370fd1cba1c5/scratchpad", "shading"))
    ap.add_argument("--view", default="front")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rr.ensure_display(os.path.join(a.out, "rviz"), gripper_arm="left",
                      task="t1")
    disp = rr.VIEWS[a.view][0]
    rclpy.init()
    n = Pub()
    time.sleep(2.0)
    rows = []
    # The lower third of the frame, where the table slab fills the view and
    # neither the wearer nor the arms reach.
    box = (60, int(rr.VH * 0.72), rr.VW - 60, rr.VH - 40)
    for name, rgb in SWATCHES:
        n.show(rgb)
        time.sleep(2.5)
        p = os.path.join(a.out, "swatch_%s.png" % name)
        if not grab(disp, p):
            print("FAILED to grab %s" % disp)
            return 2
        top = modal(p, box)
        bright, common = faces(p)
        rows.append((name, rgb, top, bright, common))
        print("   %-12s requested %-18s up-facing %-16s camera-facing %s"
              % (name, "(%.2f, %.2f, %.2f)" % rgb, common, bright))
    n.destroy_node()
    rclpy.shutdown()

    def lum(row):
        return sum(row[2][0][0]) / 3.0
    black = [r for r in rows if r[0] == "black"][0]
    white = [r for r in rows if r[0] == "full_white"][0]
    red = [r for r in rows if r[0] == "red"][0]
    ok = lum(black) < lum(white) and red[2][0][0][0] > red[2][0][0][1]
    print("\nCONTROLS  black darker than white: %s   red reads red: %s"
          % (lum(black) < lum(white), red[2][0][0][0] > red[2][0][0][1]))
    if not ok:
        print("REFUSING TO REPORT: the probe is not reading the slab.")
        return 6
    print("\nCEILING at full white: an UP-FACING marker face reaches %s of "
          "255, a CAMERA-FACING one %s."
          % (None if white[4] is None else white[4][0],
             None if white[3] is None else white[3][0]))
    print("A table top is an up-facing face, so that first number is the "
          "whitest a table top can be through the Marker path.")
    print("Frames in %s -- LOOK at them." % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
