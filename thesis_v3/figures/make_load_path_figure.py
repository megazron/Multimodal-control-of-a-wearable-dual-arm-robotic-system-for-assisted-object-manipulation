#!/usr/bin/env python3
"""Where the load goes on this platform: the simulation's own front view of
both arms on the wearer, annotated with the load path of each arm, which
closes through the wearer's back rather than through the environment.

Base image: a recorded RViz front frame of the circuit-box task
(recordings/verification/01_master_teleop/T3/.../rviz_front.mp4, via the
gallery's frame cache).

    python3 thesis_v3/figures/make_load_path_figure.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME = os.path.join(HERE, "gallery", "verification", "frames", "01_master_teleop_T3_front_simtasks_T3_0.png")
OUT_PDF = os.path.join(HERE, "sim", "load_path.pdf")
OUT_PNG = os.path.expanduser("~/overleaf_thesis/figures/raster/bracing_vs_worn.png")
W = 6.76
RED, AMBER = "#e5433a", "#f0b429"
plt.rcParams.update({"font.size": 8.5, "savefig.bbox": "tight"})
BOX = dict(boxstyle="round,pad=0.3", fc="white", ec="0.4", lw=0.6, alpha=0.95)
# pixel coordinates in the 772 x 410 frame: gripper -> elbow -> shoulder -> mount, per arm
LEFT = [(112, 245), (120, 200), (196, 137), (252, 150)]
RIGHT = [(518, 222), (562, 196), (481, 137), (426, 152)]


def load_path(ax, pts):
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=12, lw=2.0, ls="--",
                                     color=RED, shrinkA=0, shrinkB=0, zorder=5))


def main():
    im = Image.open(FRAME).convert("RGB"); w, h = im.size
    # paint out the mouse cursor RViz left on the torso, with the shirt colour beside it
    im.paste(im.getpixel((389, 222)), (380, 190, 398, 214))
    fig, ax = plt.subplots(figsize=(W, W * h / w))
    ax.imshow(im); ax.set_xlim(0, w); ax.set_ylim(h, 0); ax.axis("off")
    load_path(ax, LEFT); load_path(ax, RIGHT)
    ax.plot([252, 428], [150, 150], "o", ms=9, mfc=AMBER, mec="0.2", zorder=6)
    ax.text(340, 22, "mounts on the wearer's back:\nboth load paths end here, in the wearer", ha="center", va="top",
            fontsize=8, bbox=BOX, zorder=7)
    ax.annotate("", xy=(258, 148), xytext=(300, 52), arrowprops=dict(arrowstyle="-|>", color="0.3", lw=0.9), zorder=6)
    ax.annotate("", xy=(422, 148), xytext=(380, 52), arrowprops=dict(arrowstyle="-|>", color="0.3", lw=0.9), zorder=6)
    ax.text(20, 70, "left SRL, 8.2 kg,\nfree in space", fontsize=8, ha="left", va="center", bbox=BOX, zorder=7)
    ax.text(752, 70, "right SRL, 8.2 kg,\nfree in space", fontsize=8, ha="right", va="center", bbox=BOX, zorder=7)
    ax.text(752, 292, "work surface: not contacted,\nnothing is braced against it", fontsize=8, ha="right", va="center", bbox=BOX, zorder=7)
    ax.text(20, 292, "the wearer carries every reaction load\nand does not issue the commands", fontsize=8, ha="left",
            va="center", color=RED, bbox=BOX, zorder=7)
    ax.text(386, 385, "the operator commands both arms from across the room", fontsize=8, ha="center", va="center",
            bbox=BOX, zorder=7)
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    os.makedirs(os.path.dirname(OUT_PDF), exist_ok=True)
    fig.savefig(OUT_PDF); fig.savefig(OUT_PNG, dpi=300); plt.close(fig); print("wrote", OUT_PDF, "and", OUT_PNG)


if __name__ == "__main__":
    main()
