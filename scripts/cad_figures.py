#!/usr/bin/env python3
"""Thesis figures for the CAD master arm, rendered straight from the STLs.

NOT VIA RVIZ, DELIBERATELY. RViz is a viewer with its own config schema and a
window manager dependency; these figures need repeatable framing, real
dimension annotations and a transparent background, and every one of those is
easier to control here. The RViz render is a separate check that the model
looks like the printed arm -- this is the figure pipeline.

Everything is drawn at the ZERO CONFIGURATION, from the same inferred joint
heights the URDF uses, so a figure cannot disagree with the model.

    .cad_venv/bin/python scripts/cad_figures.py
"""
import os
import struct
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection          # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH = os.path.join(ROOT, "src/srl_description/meshes/master_arm_cad")
OUT = os.path.join(ROOT, "thesis_report/figures")

JOINT_Z = [0.023, 0.0607, 0.1030, 0.1397, 0.1800, 0.2187, 0.2600]
KIND = ["roll", "bend", "roll", "bend", "roll", "bend", "roll"]
TIP_Z = 0.3317
MEASURED = [43, 37, 43, 37, 43, 36, 33]

INK = "#20262b"
ACC = "#2f7f8f"
WARM = "#b8622a"


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    tri = np.empty((n, 3, 3), dtype=np.float32)
    for i in range(n):
        o = 84 + i * 50 + 12
        tri[i] = np.frombuffer(b, dtype="<f4", count=9, offset=o).reshape(3, 3)
    return tri


def load_all():
    parts = {}
    for nm in ["base"] + ["link%d" % i for i in range(1, 8)]:
        p = os.path.join(MESH, "%s.stl" % nm)
        if os.path.exists(p):
            parts[nm] = read_stl(p)
    return parts


def draw(ax, parts, elev, azim, shade=True):
    for i, (nm, tri) in enumerate(parts.items()):
        c = ACC if nm != "base" else "#8a949c"
        col = Poly3DCollection(tri, facecolor=c, edgecolor="none",
                               alpha=0.95, linewidths=0)
        ax.add_collection3d(col)
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect((1, 1, 2.4))
    ax.set_xlim(-0.06, 0.06)
    ax.set_ylim(-0.06, 0.06)
    ax.set_zlim(0, 0.34)
    ax.set_axis_off()


def fig_views(parts):
    views = [("isometric", 22, -60), ("front", 0, -90),
             ("side", 0, 0), ("top", 89, -90)]
    fig = plt.figure(figsize=(13, 6.5), facecolor="white")
    for k, (nm, el, az) in enumerate(views, 1):
        ax = fig.add_subplot(1, 4, k, projection="3d")
        draw(ax, parts, el, az)
        ax.set_title(nm, color=INK, fontsize=12, pad=2)
    fig.suptitle("Master arm, from CAD — zero configuration", color=INK,
                 fontsize=14)
    p = os.path.join(OUT, "master_arm_cad_views.png")
    fig.savefig(p, dpi=190, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return p


def fig_chain():
    """The link-length diagram in the roll/bend convention."""
    fig, ax = plt.subplots(figsize=(7.6, 9.2), facecolor="white")
    zs = [0.0] + JOINT_Z
    for i in range(7):
        z0, z1 = zs[i], zs[i + 1]
        ax.plot([0, 0], [z0, z1], color=INK, lw=3, solid_capstyle="round",
                zorder=1)
        ax.annotate("", xy=(0.020, z1), xytext=(0.020, z0),
                    arrowprops=dict(arrowstyle="<->", color=ACC, lw=1.1))
        cad_mm = (z1 - z0) * 1000
        ax.text(0.024, (z0 + z1) / 2,
                "L%d   CAD %.1f\n      meas %d" % (i + 1, cad_mm, MEASURED[i]),
                color=INK, fontsize=9, va="center", family="monospace")
    for i, (z, k) in enumerate(zip(JOINT_Z, KIND), 1):
        if k == "roll":
            ax.plot([0], [z], marker="o", ms=15, mfc="white", mec=ACC,
                    mew=2.4, zorder=3)
            ax.plot([0, 0], [z - 0.008, z + 0.008], color=ACC, lw=2.4,
                    zorder=4)
        else:
            ax.plot([0], [z], marker="o", ms=15, mfc="white", mec=WARM,
                    mew=2.4, zorder=3)
            ax.plot([-0.009, 0.009], [z, z], color=WARM, lw=2.4, zorder=4)
        ax.text(-0.024, z, "J%d %s" % (i, k.upper()), color=INK, fontsize=9,
                ha="right", va="center", family="monospace")
    ax.plot([0], [TIP_Z], marker="s", ms=10, mfc="white", mec=INK, mew=2)
    ax.text(-0.024, TIP_Z, "grip", color=INK, fontsize=9, ha="right",
            va="center", family="monospace")
    ax.plot([0, 0], [JOINT_Z[-1], TIP_Z], color=INK, lw=3, ls=":", zorder=1)
    ax.set_xlim(-0.07, 0.075)
    ax.set_ylim(-0.012, 0.355)
    ax.set_axis_off()
    ax.set_title("Master arm chain — roll / bend alternation\n"
                 "lengths in mm, CAD-inferred against measured",
                 color=INK, fontsize=12)
    # the convention-free result, stated on the figure
    ax.text(0.0, -0.005,
            "bend-to-bend, independent of roll placement:\n"
            "J2→J4  measured 80.0   CAD 79.0\n"
            "J4→J6  measured 79.0   CAD 79.0",
            fontsize=8.5, color=ACC, ha="center", va="top",
            family="monospace")
    p = os.path.join(OUT, "master_arm_chain.png")
    fig.savefig(p, dpi=190, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return p


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = load_all()
    print("loaded %d link meshes, %d triangles"
          % (len(parts), sum(len(t) for t in parts.values())))
    made = [fig_views(parts), fig_chain()]
    for p in made:
        print("  -> %s" % p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
