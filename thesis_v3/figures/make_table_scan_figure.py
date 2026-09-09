#!/usr/bin/env python3
"""The table-scanning sweep and the map it produced, drawn natively:
(a) the planned sweep, one fixed wrist attitude per pass, serpentine rows,
two heights; (b) which cells the arms could reach and which views were
captured; (c) the fused map against the scene's true geometry.

Data: recordings/calibration/20260823_202409/world_map.json (both arms, 144
planned cells) and srl_perception.calibration_sweep for the planned order.

    python3 thesis_v3/figures/make_table_scan_figure.py
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_perception"))
from srl_perception import calibration_sweep as CS  # noqa: E402

RUN = os.path.join(ROOT, "recordings", "calibration", "20260823_202409", "world_map.json")
OUT = os.path.join(HERE, "sim", "table_scan_collage.pdf")
W = 6.76
plt.rcParams.update({"font.size": 9, "axes.titlesize": 9.5, "axes.labelsize": 9, "xtick.labelsize": 8.5,
                     "ytick.labelsize": 8.5, "legend.fontsize": 8, "savefig.bbox": "tight",
                     "axes.spines.top": False, "axes.spines.right": False})
BLUE, RED, GREEN, AMBER, GREY = "#2c6fbb", "#c1392b", "#3f9142", "#c98a1a", "#595959"
# the scene the map was measured against (the T1 layout, declared in the task file)
TRUTH = [((0.290, 0.500), (0.140, 0.160), "pad"), ((-0.290, 0.500), (0.140, 0.160), "pad"),
         ((0.420, 0.450), (0.040, 0.040), "cube"), ((0.480, 0.450), (0.040, 0.040), "cube"),
         ((-0.420, 0.450), (0.040, 0.040), "cube"), ((-0.480, 0.450), (0.040, 0.040), "cube")]


def main():
    w = json.load(open(RUN))
    sw = w["sweep"]; prov = w["provenance"]
    fig = plt.figure(figsize=(W, 5.6))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 0.7], hspace=0.25, wspace=0.3)

    # (a) the planned sweep for one arm: serpentine rows, one attitude per pass, two layers.
    # The run's own record keeps the grid shape (cols, rows, step, layers, facings) but not
    # the bounds; the box swept in this run was x 0.30-0.58, y 0.30-0.52 (left arm), mirrored
    # in x for the right arm, and plan_volume reproduces the run's cell order.
    BOUNDS = {"left": (0.30, 0.58, 0.30, 0.52), "right": (-0.58, -0.30, 0.30, 0.52)}
    surface = w["surface"]["z_m"]
    per_arm = {pa["arm"]: pa for pa in sw["per_arm"]}
    pa = per_arm["left"]
    def plan(arm):
        x0, x1, y0, y1 = BOUNDS[arm]
        return CS.plan_volume(x0, x1, y0, y1, surface, step_m=pa["step_m"], layers_m=tuple(pa["layers_m"]),
                              facings_deg=tuple(pa["facings_deg"]))["passes"]
    passes = plan("left")
    ax = fig.add_subplot(gs[0, 0], projection="3d")
    cells = passes[0]["cells"]
    zs = sorted({c[2] for c in cells})
    x0, x1, y0, y1 = BOUNDS["left"]
    # the measured surface as a translucent sheet under the sweep
    XX, YY = np.meshgrid([x0 - 0.05, x1 + 0.05], [y0 - 0.05, y1 + 0.05])
    ax.plot_surface(XX, YY, np.full_like(XX, surface), color=AMBER, alpha=0.25, linewidth=0)
    for zi, z in enumerate(zs):
        pts = [(c[0], c[1], c[2]) for c in cells if c[2] == z]
        xs, ys, zz = zip(*pts)
        col = [BLUE, RED][zi % 2]
        ax.plot(xs, ys, zz, "-", color=col, lw=1.3, label="camera %.2f m above the surface" % (z - surface))
        ax.scatter(xs, ys, zz, s=12, color=col, depthshade=False)
        ax.quiver(xs[0], ys[0], zz[0], xs[1] - xs[0], ys[1] - ys[0], 0, color=col, arrow_length_ratio=0.4, lw=1.5)
    for (xa, ya, za) in [(c[0], c[1], c[2]) for c in cells if c[2] == zs[0]][:4]:
        ax.plot([xa, xa], [ya, ya], [surface, za], color="0.7", lw=0.5, ls=":")
    ax.set_xlabel("x (m)", labelpad=-4); ax.set_ylabel("y (m)", labelpad=-4); ax.set_zlabel("z (m)", labelpad=-2)
    ax.tick_params(labelsize=7, pad=-1)
    ax.view_init(elev=26, azim=-58); ax.set_box_aspect((1.3, 1.0, 0.55), zoom=1.35)
    ax.legend(loc="upper left", frameon=False, fontsize=7, bbox_to_anchor=(0.0, 0.98), ncol=1)
    ax.set_title("(a) one pass of the sweep, left arm", loc="left", fontsize=9, fontweight="bold", pad=0)

    # (b) every planned cell position of both arms as a grid: how many of its six views were captured
    ax = fig.add_subplot(gs[0, 1])
    captured = set(prov["sources"])
    step = pa["step_m"]
    for arm, cmap in (("left", plt.cm.Blues), ("right", plt.cm.Reds)):
        pss = plan(arm); n_per = len(pss[0]["cells"]); count = {}
        for pi, ps in enumerate(pss):
            for ci, c in enumerate(ps["cells"]):
                key = "%s_p%d_c%02d" % (arm, pi + 1, pi * n_per + ci + 1)
                kk = (round(c[0], 3), round(c[1], 3))
                count.setdefault(kk, [0, 0]); count[kk][1] += 1
                if key in captured: count[kk][0] += 1
        for (x, y), (n, tot) in count.items():
            ax.add_patch(Rectangle((x - step * 0.42, y - step * 0.42), step * 0.84, step * 0.84,
                                   fc=cmap(0.2 + 0.7 * n / tot), ec="white", lw=1))
            ax.text(x, y, "%d" % n, ha="center", va="center", fontsize=7.5, color="white" if n / tot > 0.5 else "0.2")
    ax.set_xlim(-0.66, 0.66); ax.set_ylim(0.23, 0.60); ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.text(-0.44, 0.585, "right arm", ha="center", fontsize=7.5, color=RED); ax.text(0.44, 0.585, "left arm", ha="center", fontsize=7.5, color=BLUE)
    ax.set_title("(b) views captured per cell, of 6", loc="left", fontsize=9, fontweight="bold")

    # (c) the map against the truth
    ax = fig.add_subplot(gs[1, :])
    for (cx, cy), (ex, ey), kind in TRUTH:
        ax.add_patch(Rectangle((cx - ex / 2, cy - ey / 2), ex, ey, fc="none", ec=GREY, lw=1.2, ls="--", label="true object" if kind == "pad" and cx > 0 else None))
    k = 0
    for o in sorted(w["objects"], key=lambda o: o["centre"][0]):
        cx, cy, _ = o["centre"]; ex, ey, ez = o["extents"]
        col = GREEN if o.get("graspable") else AMBER
        ax.add_patch(Rectangle((cx - ex / 2, cy - ey / 2), ex, ey, fc=col, alpha=0.35, ec=col, lw=1.0))
        lab = "%.0f x %.0f x %.0f mm" % (1000 * ex, 1000 * ey, 1000 * ez)
        if not o.get("graspable"):
            ax.text(cx, cy, lab, ha="center", va="center", fontsize=7.5, color="0.25")
    for sign in (-1, 1):
        cubes = [o for o in w["objects"] if o.get("graspable") and (o["centre"][0] > 0) == (sign > 0)]
        if cubes:
            xm = sum(o["centre"][0] for o in cubes) / len(cubes)
            zs_ = sorted({round(1000 * o["extents"][2]) for o in cubes}); zl = "%d" % zs_[0] if len(zs_) == 1 else "%d-%d" % (zs_[0], zs_[-1])
            ax.text(xm, 0.415, "cubes: 42 x 41 x %s mm" % zl,
                    ha="center", va="top", fontsize=7.5, color=GREEN)
    ax.add_patch(Rectangle((0, 0), 0, 0, fc=GREEN, alpha=0.35, label="measured, fits the 85 mm jaw"))
    ax.add_patch(Rectangle((0, 0), 0, 0, fc=AMBER, alpha=0.35, label="measured, too wide to grasp"))
    ax.set_xlim(-0.6, 0.6); ax.set_ylim(0.38, 0.60); ax.set_aspect("equal")
    ax.set_xlabel("x (m), positive to the wearer's left"); ax.set_ylabel("y (m), forward")
    ax.legend(frameon=False, loc="upper center", ncol=3, fontsize=8, bbox_to_anchor=(0.5, -0.32))
    n_true = len(TRUTH); hits = 0
    for (cx, cy), _, _ in TRUTH:
        if any(abs(o["centre"][0] - cx) < 0.01 and abs(o["centre"][1] - cy) < 0.01 for o in w["objects"]): hits += 1
    ax.set_title("(c) the fused map (filled) against the true scene (dashed); surface %.4f m, true %.3f m" % (w["surface"]["z_m"], 1.250),
                 loc="left", fontsize=9, fontweight="bold")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT); plt.close(fig); print("wrote", OUT)
    print("surface", w["surface"], "sweep", {k: sw[k] for k in ("cells", "cells_reachable", "views_used", "travel_m", "stillness_refusals")})
    for o in w["objects"]:
        print("   ", [round(c, 3) for c in o["centre"]], [round(1000 * e) for e in o["extents"]], o.get("graspable"), o.get("why", ""))


if __name__ == "__main__":
    main()
