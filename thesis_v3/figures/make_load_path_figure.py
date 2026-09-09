#!/usr/bin/env python3
"""Where the load goes: a bracing supernumerary limb against the worn,
free-in-space arms of this thesis. Drawn natively; no data.

    python3 thesis_v3/figures/make_load_path_figure.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle, FancyArrowPatch, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_PDF = os.path.join(HERE, "sim", "load_path.pdf")
OUT_PNG = os.path.expanduser("~/overleaf_thesis/figures/raster/bracing_vs_worn.png")
W = 6.76
GREEN, RED, GREY, SKIN, SHIRT, STEEL = "#2e8b3d", "#c1392b", "#4a4a4a", "#e6c9a8", "#4b6fa6", "#8c8c8c"
plt.rcParams.update({"font.size": 8.5, "savefig.bbox": "tight"})


def person(ax, x, commands):
    """Side view: head, torso, legs, one own arm hanging."""
    ax.add_patch(Circle((x, 8.25), 0.52, fc=SKIN, ec="0.3", lw=0.8, zorder=3))
    ax.add_patch(FancyBboxPatch((x - 0.62, 4.7), 1.24, 2.85, boxstyle="round,pad=0.02,rounding_size=0.25",
                                fc=SHIRT, ec="0.3", lw=0.8, zorder=3))
    for dx in (-0.28, 0.28):
        ax.plot([x + dx, x + dx * 1.1], [4.7, 1.65], color="#2b3a55", lw=5, solid_capstyle="round", zorder=2)
    ax.plot([x - 0.45, x - 0.75, x - 0.55], [7.2, 5.9, 4.9], color=SKIN, lw=3.2, solid_capstyle="round", zorder=4)
    ax.text(x, 8.95, "wearer\n(" + ("commands the limb" if commands else "does not command") + ")", ha="center", va="bottom", fontsize=8)


def limb(ax, pts, color=STEEL):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=6, solid_capstyle="round", solid_joinstyle="round", zorder=5)
    ax.plot(xs, ys, color="0.95", lw=2.2, solid_capstyle="round", solid_joinstyle="round", zorder=6)
    for px, py in pts[1:-1]:
        ax.add_patch(Circle((px, py), 0.17, fc="0.25", ec="none", zorder=7))
    ax.add_patch(Circle(pts[0], 0.2, fc="0.25", ec="none", zorder=7))


def surface(ax, x0, x1, contacted):
    ax.add_patch(Rectangle((x0, 3.55), x1 - x0, 0.32, fc="0.85", ec="0.35", lw=0.8, zorder=2))
    for lx in (x0 + 0.3, x1 - 0.3):
        ax.plot([lx, lx], [3.55, 1.65], color="0.35", lw=1.6, zorder=1)
    if contacted:
        ax.text(x1 - 0.7, 3.15, "work surface", ha="right", va="top", fontsize=8)
    else:
        ax.text((x0 + x1) / 2, 3.15, "work surface\n(not contacted)", ha="center", va="top", fontsize=8)


def floor(ax):
    ax.plot([0.2, 9.8], [1.65, 1.65], color="0.3", lw=1.2, zorder=1)
    for gx in np.arange(0.4, 9.8, 0.45):
        ax.plot([gx, gx - 0.25], [1.65, 1.3], color="0.6", lw=0.7, zorder=1)


def path(ax, pts, color, label=None, label_xy=None, rad=0.0):
    """The load path as a dashed arrow through the given points."""
    for (ax0, ay0), (ax1, ay1) in zip(pts[:-1], pts[1:]):
        ax.add_patch(FancyArrowPatch((ax0, ay0), (ax1, ay1), arrowstyle="-|>", mutation_scale=11, lw=1.6, ls="--",
                                     color=color, zorder=8, connectionstyle="arc3,rad=%.2f" % rad, shrinkA=0, shrinkB=0))
    if label:
        ax.text(*label_xy, label, color=color, fontsize=8, ha="center", va="center", style="italic",
                bbox=dict(fc="white", ec="none", pad=1.5), zorder=9)


def main():
    fig, axes = plt.subplots(1, 2, figsize=(W, 3.25))
    for ax in axes:
        ax.set_xlim(0, 10); ax.set_ylim(0.9, 10.4); ax.set_aspect("equal"); ax.axis("off")
        floor(ax)

    # (a) bracing: mount at the waist, limb pressed on the surface, load closes through the environment
    ax = axes[0]
    ax.set_title("(a) a bracing supernumerary limb", loc="left", fontsize=9, fontweight="bold")
    person(ax, 2.5, commands=True)
    surface(ax, 5.4, 9.4, contacted=True)
    pts = [(3.15, 5.4), (4.7, 6.6), (6.3, 5.3), (7.0, 3.87)]
    limb(ax, pts)
    ax.text(6.3, 7.3, "supernumerary limb\nbraced on the surface", ha="center", va="bottom", fontsize=8)
    ax.add_patch(Polygon([[6.7, 3.87], [7.3, 3.87], [7.0, 3.55]], fc="0.25", ec="none", zorder=7))
    path(ax, [(3.0, 5.4), (4.7, 6.6), (6.3, 5.3), (7.0, 3.87), (7.0, 2.4), (7.0, 1.7)], GREEN)
    ax.text(8.55, 5.6, "reaction load\nleaves the body\nthrough the surface\nand the floor", color=GREEN, fontsize=8,
            ha="center", va="center", style="italic", zorder=9)

    # (b) this thesis: mount on the back, arm free in space, load closes through the wearer
    ax = axes[1]
    ax.set_title("(b) this thesis", loc="left", fontsize=9, fontweight="bold")
    person(ax, 2.5, commands=False)
    surface(ax, 5.4, 9.4, contacted=False)
    pts = [(3.15, 6.9), (4.6, 8.3), (6.4, 7.4), (7.4, 6.2)]
    limb(ax, pts)
    ax.add_patch(Rectangle((7.35, 5.65), 0.5, 0.5, fc="#3f9142", ec="0.25", lw=0.6, zorder=6))
    ax.text(6.7, 9.35, "two 8.2 kg arms, free in space", ha="center", va="bottom", fontsize=8)
    # weight of the arm and the reaction path back into the wearer
    ax.add_patch(FancyArrowPatch((4.7, 7.75), (4.7, 6.35), arrowstyle="-|>", mutation_scale=11, lw=1.6, color=RED, zorder=8))
    ax.text(4.7, 6.1, "weight and\ncontact forces", color=RED, fontsize=7.5, ha="center", va="top", style="italic")
    path(ax, [(7.4, 6.2), (6.4, 7.4), (4.6, 8.3), (3.15, 6.9), (2.6, 6.2)], RED)
    ax.text(7.4, 4.75, "every reaction load is carried by\na person who did not command it", color=RED,
            fontsize=8, ha="center", va="center", style="italic", zorder=9)
    ax.annotate("operator commands\nfrom across the room", xy=(9.8, 7.3), xytext=(8.6, 8.35), fontsize=7.5, ha="center",
                va="center", color="0.3", arrowprops=dict(arrowstyle="-|>", color="0.45", lw=1.0))
    fig.subplots_adjust(left=0.01, right=0.99, top=0.9, bottom=0.02, wspace=0.05)
    os.makedirs(os.path.dirname(OUT_PDF), exist_ok=True)
    fig.savefig(OUT_PDF); fig.savefig(OUT_PNG, dpi=300); plt.close(fig); print("wrote", OUT_PDF, "and", OUT_PNG)


if __name__ == "__main__":
    main()
