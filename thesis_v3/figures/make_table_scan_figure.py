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
    fig = plt.figure(figsize=(W, 5.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 0.7], hspace=0.45, wspace=0.3)

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
    ax = fig.add_subplot(gs[0, 0])
    cells = passes[0]["cells"]
    zs = sorted({c[2] for c in cells})
    for zi, z in enumerate(zs):
        pts = [(c[0], c[1]) for c in cells if c[2] == z]
        xs, ys = zip(*pts)
        ax.plot(xs, ys, "-", color=[BLUE, RED][zi % 2], lw=1.0, alpha=0.9, label="camera %.2f m above the surface" % (z - surface))
        ax.scatter(xs, ys, s=10, color=[BLUE, RED][zi % 2], zorder=3)
        ax.annotate("", xy=pts[1], xytext=pts[0], arrowprops=dict(arrowstyle="->", color=[BLUE, RED][zi % 2], lw=1.4))
    ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.margins(0.15)
    ax.legend(loc="upper left", frameon=False, fontsize=7.5, bbox_to_anchor=(0, 1.0))
    ax.set_title("(a) one pass, left arm",
                 loc="left", fontsize=9, fontweight="bold")

    # (b) every planned cell position of both arms: how many of its six views (2 heights x 3
    # facings) were captured, from the run's own record of captured sources
    ax = fig.add_subplot(gs[0, 1])
    captured = set(prov["sources"])
    for arm, cmap in (("left", plt.cm.Blues), ("right", plt.cm.Reds)):
        pss = plan(arm); n_per = len(pss[0]["cells"]); count = {}
        for pi, ps in enumerate(pss):
            for ci, c in enumerate(ps["cells"]):
                key = "%s_p%d_c%02d" % (arm, pi + 1, pi * n_per + ci + 1)
                count.setdefault((round(c[0], 3), round(c[1], 3)), [0, 0])
                count[(round(c[0], 3), round(c[1], 3))][1] += 1
                if key in captured: count[(round(c[0], 3), round(c[1], 3))][0] += 1
        for (x, y), (n, tot) in count.items():
            ax.scatter([x], [y], s=150, color=cmap(0.25 + 0.7 * n / tot), edgecolor="0.4", lw=0.5, zorder=3)
            ax.text(x, y, "%d" % n, ha="center", va="center", fontsize=7.5, color="white" if n / tot > 0.5 else "0.2")
    ax.set_aspect("auto"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.margins(0.12, 0.25)
    ax.set_title("(b) %d planned, %d reachable, %d captured" % (sw["cells"], sw["cells_reachable"], sw["views_used"]),
                 loc="left", fontsize=9, fontweight="bold")

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
    ax.set_title("(c) the fused map against the true scene (dashed): surface %.4f m, true %.3f m" % (w["surface"]["z_m"], 1.250),
                 loc="left", fontsize=9, fontweight="bold")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT); plt.close(fig); print("wrote", OUT)
    print("surface", w["surface"], "sweep", {k: sw[k] for k in ("cells", "cells_reachable", "views_used", "travel_m", "stillness_refusals")})
    for o in w["objects"]:
        print("   ", [round(c, 3) for c in o["centre"]], [round(1000 * e) for e in o["extents"]], o.get("graspable"), o.get("why", ""))


if __name__ == "__main__":
    main()
