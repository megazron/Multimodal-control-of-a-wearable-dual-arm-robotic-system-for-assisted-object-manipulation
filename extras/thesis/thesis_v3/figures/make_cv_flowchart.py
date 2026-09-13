#!/usr/bin/env python3
"""The picture-to-grasp pipeline as a flowchart of real stage images.

    python3 extras/thesis/thesis_v3/figures/make_cv_flowchart.py

Six stages of one real left-wrist-camera frame (recordings/vision_thesis/
20260830_073141), left to right with arrows: the picture, its depth map, the
table plane found in it, the two task objects picked out, the oriented boxes
round them, and the map the planner is handed (the point cloud from above).
Each stage carries its measured number. Reuses the gallery's verified loaders
(figures/gallery/vision/make_vision_gallery.py); nothing is synthesised.
"""

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, FancyArrowPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "gallery", "vision"))
import make_vision_gallery as G  # noqa: E402
import cv2  # noqa: E402

OUT = os.path.join(HERE, "vision", "cv_flowchart.pdf")
GREEN, RED, BLUE = G.GREEN, G.RED, G.BLUE
CAM = "left_gripper"


def main():
    bgr = cv2.imread(os.path.join(G.RUN, "raw", CAM, "colour.png"))
    im = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    d = G.depth(CAM)
    lo, hi = G.band(CAM)
    P, v, u = G.cloud(CAM)
    n, off, tol, frac_stored, rms = G.plane(CAM)
    h = P @ n + off
    inl = np.abs(h) < tol
    cube_pts, cube_m = G.cube_rect(bgr)
    box_pts, box_m = G.box_rect(bgr)
    rows = G.boxes(CAM)
    m15 = G.stage(CAM, "15_raw_depth")["m"]
    ret = 100 * m15["pixels with a return"] / (m15["depth width"] * m15["depth height"])

    fig = plt.figure(figsize=(7.4, 7.6))
    gs = fig.add_gridspec(3, 2, hspace=0.42, wspace=0.14,
                          left=0.02, right=0.98, top=0.95, bottom=0.04)
    axes = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(6)]

    # 1 picture
    axes[0].imshow(im)
    # 2 depth
    shown = np.ma.masked_where(d <= 0, np.clip(d * 1000, lo * 1000, hi * 1000))
    axes[1].imshow(shown, cmap="viridis")
    # 3 plane
    canvas = np.stack([0.55 + 0.35 * np.clip(d / hi, 0, 1)] * 3, -1)
    canvas[d <= 0] = 1.0
    canvas[v[inl], u[inl]] = matplotlib.colors.to_rgb(GREEN)
    canvas[v[~inl], u[~inl]] = matplotlib.colors.to_rgb(RED)
    axes[2].imshow(canvas)
    # 4 objects picked out
    seg = im.astype(float)
    seg[cube_m] = 0.5 * seg[cube_m] + 0.5 * np.array(matplotlib.colors.to_rgb(GREEN)) * 255
    seg[box_m] = 0.5 * seg[box_m] + 0.5 * np.array(matplotlib.colors.to_rgb(BLUE)) * 255
    axes[3].imshow(seg.astype(np.uint8))
    # 5 oriented boxes
    axes[4].imshow(im)
    axes[4].add_patch(Polygon(cube_pts, closed=True, fill=False, edgecolor=GREEN, lw=1.6))
    axes[4].add_patch(Polygon(box_pts, closed=True, fill=False, edgecolor=BLUE, lw=1.6))
    # 6 plan view
    ax = axes[5]
    keep = h * 1000 > -20
    order = np.argsort(h[keep])
    ax.scatter(P[keep][order, 0], P[keep][order, 2], c=np.clip(h[keep][order] * 1000, 0, 90),
               s=0.6, cmap="viridis", rasterized=True)
    for b in rows:
        x, z = float(b["x m"]), float(b["z m"])
        L, W = float(b["length mm"]) / 1000, float(b["width mm"]) / 1000
        yaw = np.deg2rad(float(b["yaw deg"]))
        c, s = np.cos(yaw), np.sin(yaw)
        corners = np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2]])
        rot = corners @ np.array([[c, -s], [s, c]]).T
        ax.add_patch(Polygon(rot + [x, z], closed=True, fill=False,
                             edgecolor=GREEN if b["graspable"] == "yes" else RED, lw=1.2))
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

    grasp_ok = sum(1 for b in rows if b["graspable"] == "yes")
    big = max(rows, key=lambda b: float(b["length mm"]))
    titles = [
        ("1  the picture", "left wrist camera, 1280x720"),
        ("2  the depth map", "%.0f%% of pixels have a reading" % ret),
        ("3  find the flat table", "%.0f%% of points on one plane, %.2f mm" % (frac_stored, rms)),
        ("4  pick out the task objects", "cube by colour; box by its own hue"),
        ("5  a tilted box round each", "true width of a turned object"),
        ("6  the map the planner gets", "%d objects; %d fit the 85 mm hand"
         % (len(rows), grasp_ok)),
    ]
    for ax, (t1, t2) in zip(axes, titles):
        if ax is not axes[5]:
            ax.axis("off")
        ax.set_title(t1, fontsize=10, loc="left", pad=3)
        ax.text(0.0, -0.04, t2, transform=ax.transAxes, fontsize=8.5, color="0.3", va="top")

    # arrows between stages in the same row, from the grid cells (panel 6 has
    # an equal-aspect axes box, so axes positions are not comparable)
    def arrow(i, j):
        pa = gs[i // 2, i % 2].get_position(fig); pb = gs[j // 2, j % 2].get_position(fig)
        y = (pa.y0 + pa.y1) / 2
        fig.add_artist(FancyArrowPatch((pa.x1 - 0.012, y), (pb.x0 + 0.012, y),
                                       arrowstyle="-|>", mutation_scale=10, color="0.35", lw=1.0))
    arrow(0, 1); arrow(2, 3); arrow(4, 5)
    print("widest object %.0f mm" % float(big["length mm"]))
    fig.savefig(OUT)
    print("wrote", OUT, "| stored inlier %.2f%%, recomputed %.2f%%" % (frac_stored, 100 * inl.mean()))


if __name__ == "__main__":
    main()
