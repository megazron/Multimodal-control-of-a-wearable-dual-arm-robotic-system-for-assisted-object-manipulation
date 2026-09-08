#!/usr/bin/env python3
"""What each of the four real cameras sees and is for: one column per camera,
RGB on top, depth below, from the frames of the 2026-08-30 session
(recordings/vision_thesis/20260830_073141/raw/<camera>/). The HD webcam has
no depth sensor, and that panel says so rather than showing something.

    python3 thesis_v3/figures/make_camera_roles.py
"""
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
RAW = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141", "raw")
OUT = os.path.join(HERE, "vision", "camera_roles")

CAMS = [
    ("left_gripper", "Left wrist camera", "RGB-D (Kinova vision module)",
     "on the left gripper; fits the table plane\nand finds the objects to grasp"),
    ("right_gripper", "Right wrist camera", "RGB-D (Kinova vision module)",
     "on the right gripper; same role for the\nright arm"),
    ("scene_rs", "Room depth camera", "RGB-D (RealSense D435i)",
     "across the room; tracks the wearer's body\nfor the clearance floor"),
    ("scene_hd", "Room HD camera", "RGB only (webcam)",
     "across the room; the operator's view and\nthe record of each session"),
]


def main():
    fig, axes = plt.subplots(2, 4, figsize=(12.5, 6.0))
    for j, (cam, title, kind, role) in enumerate(CAMS):
        d = os.path.join(RAW, cam)
        rgb = plt.imread(os.path.join(d, "colour.png"))
        # every panel at 16:9: the D435i frame is 4:3, so its middle 360 rows are shown
        def to169(a):
            h, w = a.shape[:2]
            th = int(round(w * 9 / 16))
            if th < h:
                o = (h - th) // 2
                return a[o:o + th]
            return a
        rgb = to169(rgb)
        ax = axes[0, j]
        ax.imshow(rgb)
        ax.set_title("%s\n%s" % (title, kind), fontsize=10)
        ax.set_axis_off()
        ax = axes[1, j]
        dp = os.path.join(d, "depth.npy")
        if os.path.exists(dp):
            depth = to169(np.load(dp).astype(float))
            valid = depth > 0
            shown = np.where(valid, depth, np.nan)
            vmax = np.percentile(depth[valid], 97) if valid.any() else 1.0
            im = ax.imshow(shown, cmap="viridis", vmin=0.0, vmax=vmax)
            ax.text(0.02, 0.04, "%.0f%% of pixels return a depth" % (100 * valid.mean()),
                    transform=ax.transAxes, fontsize=8, color="white",
                    bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=2))
            cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
            cb.set_label("distance (m)", fontsize=8)
            cb.ax.tick_params(labelsize=7)
        else:
            ax.text(0.5, 0.5, "no depth sensor", ha="center", va="center", fontsize=11, color="0.35")
            ax.set_facecolor("0.93")
            ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
        if os.path.exists(dp):
            ax.set_axis_off()
        else:
            ax.set_aspect(9 / 16)
        fig.text(0.04 + (0.95 / 4) * (j + 0.5) - 0.012, 0.075, role, ha="center", va="top", fontsize=8.5, color="0.25")
    axes[0, 0].text(-0.08, 0.5, "colour", transform=axes[0, 0].transAxes, rotation=90, va="center", ha="right", fontsize=11)
    axes[1, 0].text(-0.08, 0.5, "depth", transform=axes[1, 0].transAxes, rotation=90, va="center", ha="right", fontsize=11)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.92, bottom=0.10, wspace=0.12, hspace=0.02)
    fig.savefig(OUT + ".pdf")
    fig.savefig(OUT + "_300.png", dpi=300)
    fig.savefig(OUT + ".png", dpi=130)
    print("wrote", OUT + ".pdf")


if __name__ == "__main__":
    main()
