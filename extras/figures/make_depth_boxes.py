#!/usr/bin/env python3
"""Raw frame beside its depth map, both with the cube and box boxed and the
box annotated with its measured distance -- so a reader can see what the
depth panel is a depth map OF, not just an abstract heatmap.

    python3 extras/figures/make_depth_boxes.py

Same real frame as make_pilot_figures.py's siblings use for cv_pipeline.pdf:
recordings/vision_thesis/20260830_073141/raw/left_gripper/{colour.png,depth.npy}.
depth.npy is float32 metres, 0 = no return (confirmed against
data/left_gripper/15_raw_depth.json's own stage log).
"""

import json
import os
import sys

import cv2
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 2))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from srl_object_detector import blobs  # noqa: E402

RAW = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141", "raw", "left_gripper")
OUT = os.path.join(os.path.dirname(__file__), "vision", "depth_boxes.pdf")

BLUE, GREEN = "#2c6fbb", "#3f9142"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})

BOX_HSV = ((0, 90, 50), (25, 255, 200))       # cardboard box, measured from this frame
WORKING_BAND_M = (0.08, 1.20)                  # this camera's declared working band


def box_mask(bgr, lo, hi):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, lo, hi)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h = stats[idx, cv2.CC_STAT_LEFT], stats[idx, cv2.CC_STAT_TOP], \
        stats[idx, cv2.CC_STAT_WIDTH], stats[idx, cv2.CC_STAT_HEIGHT]
    return x, y, w, h


def depth_at(depth, colour_shape, box_px, scale):
    x, y, w, h = box_px
    dx0, dy0 = int(x * scale), int(y * scale)
    dx1, dy1 = int((x + w) * scale), int((y + h) * scale)
    patch = depth[dy0:dy1, dx0:dx1]
    valid = patch[(patch > WORKING_BAND_M[0]) & (patch < WORKING_BAND_M[1])]
    return float(np.median(valid)) if valid.size else float("nan")


def main():
    bgr = cv2.imread(os.path.join(RAW, "colour.png"))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = np.load(os.path.join(RAW, "depth.npy"))
    scale = depth.shape[0] / bgr.shape[0]

    cube = sorted(blobs(bgr), key=lambda d: -d["area_px"])[0]
    cube_box = (cube["u"] - cube["w_px"] / 2, cube["v"] - cube["h_px"] / 2,
                cube["w_px"], cube["h_px"])
    box_box = box_mask(bgr, *BOX_HSV)

    cube_mm = 1000 * depth_at(depth, bgr.shape, cube_box, scale)
    box_mm = 1000 * depth_at(depth, bgr.shape, box_box, scale)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.6, 3.0))

    ax0.imshow(rgb)
    ax0.set_title("(a) raw frame")
    ax0.axis("off")
    ax0.add_patch(Rectangle((box_box[0], box_box[1]), box_box[2], box_box[3],
                             fill=False, edgecolor=BLUE, lw=1.4))
    ax0.add_patch(Rectangle((cube_box[0], cube_box[1]), cube_box[2], cube_box[3],
                             fill=False, edgecolor=GREEN, lw=1.4))

    d_clip = np.clip(depth * 1000, 0, 1500)
    d_clip = np.ma.masked_where(depth <= 0, d_clip)
    im = ax1.imshow(d_clip, cmap="viridis")
    ax1.set_title("(b) depth, boxed and measured")
    ax1.axis("off")
    for box, colour, label, mm in (
        (box_box, BLUE, "box", box_mm), (cube_box, GREEN, "cube", cube_mm),
    ):
        x, y, w, h = [v * scale for v in box]
        ax1.add_patch(Rectangle((x, y), w, h, fill=False, edgecolor=colour, lw=1.4))
        ax1.text(x + w / 2, y - 6, "%s: %.0f mm" % (label, mm), color=colour,
                  fontsize=7.5, ha="center", va="bottom", fontweight="bold")
    cb = fig.colorbar(im, ax=ax1, fraction=0.046, pad=0.03)
    cb.set_label("depth (mm), capped at 1500", fontsize=8)
    cb.ax.tick_params(labelsize=7)

    fig.subplots_adjust(left=0.02, right=0.92, top=0.90, bottom=0.03, wspace=0.10)
    fig.savefig(OUT)
    print("wrote", OUT, "cube=%.0fmm box=%.0fmm" % (cube_mm, box_mm))


if __name__ == "__main__":
    main()
