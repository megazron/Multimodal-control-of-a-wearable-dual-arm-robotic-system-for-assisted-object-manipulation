#!/usr/bin/env python3
"""The table-scanning sweep and the map it produced, drawn natively:
(a) one pass of the planned sweep in 3-D over the measured surface, with
the scene's objects drawn on it; (b) how many of each cell's six views were
captured, both arms; (c) the fused map against the scene's true geometry,
every object numbered; (d) what each numbered object is.

Data: recordings/calibration/20260823_202409/world_map.json (both arms, 144
planned cells) and srl_perception.calibration_sweep for the planned order.

    python3 extras/thesis/thesis_v3/figures/make_table_scan_figure.py
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 4))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_perception"))
from srl_perception import calibration_sweep as CS  # noqa: E402

RUN = os.path.join(ROOT, "recordings", "calibration", "20260823_202409", "world_map.json")
OUT = os.path.join(HERE, "sim", "table_scan_collage.pdf")
W = 6.76
plt.rcParams.update({"font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "xtick.labelsize": 8,
                     "ytick.labelsize": 8, "legend.fontsize": 7.5, "savefig.bbox": "tight",
                     "axes.spines.top": False, "axes.spines.right": False})
BLUE, RED, GREEN, AMBER, GREY = "#2c6fbb", "#c1392b", "#3f9142", "#c98a1a", "#595959"
# the scene the map was measured against (the pick-and-place layout, declared in the task file)
TRUTH = [((0.290, 0.500), (0.140, 0.160), "pad"), ((-0.290, 0.500), (0.140, 0.160), "pad"),
         ((0.420, 0.450), (0.040, 0.040), "cube"), ((0.480, 0.450), (0.040, 0.040), "cube"),
         ((-0.420, 0.450), (0.040, 0.040), "cube"), ((-0.480, 0.450), (0.040, 0.040), "cube")]
# the run's own record keeps the grid shape but not the bounds; this run swept x 0.30-0.58,
# y 0.30-0.52 with the left arm, mirrored in x for the right, and plan_volume reproduces its order
BOUNDS = {"left": (0.30, 0.58, 0.30, 0.52), "right": (-0.58, -0.30, 0.30, 0.52)}


def main():
    w = json.load(open(RUN))
    sw = w["sweep"]; prov = w["provenance"]; surface = w["surface"]["z_m"]
    per_arm = {pa["arm"]: pa for pa in sw["per_arm"]}
    pa = per_arm["left"]

    def plan(arm):
        x0, x1, y0, y1 = BOUNDS[arm]
        return CS.plan_volume(x0, x1, y0, y1, surface, step_m=pa["step_m"], layers_m=tuple(pa["layers_m"]),
                              facings_deg=tuple(pa["facings_deg"]))["passes"]

    fig = plt.figure(figsize=(W, 6.9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0], height_ratios=[1.15, 1.0], hspace=0.30, wspace=0.10,
                          left=0.03, right=0.99, top=0.96, bottom=0.05)

    # (a) one pass of the left arm's sweep in 3-D, over the surface, with the scene on it
    ax = fig.add_subplot(gs[0, 0], projection="3d")
    x0, x1, y0, y1 = BOUNDS["left"]
    XX, YY = np.meshgrid([x0 - 0.08, x1 + 0.06], [y0 - 0.02, y1 + 0.06])
    ax.plot_surface(XX, YY, np.full_like(XX, surface), color=AMBER, alpha=0.18, linewidth=0)
    labelled = set()
    for (cx, cy), (ex, ey), kind in TRUTH:
        if x0 - 0.08 <= cx <= x1 + 0.06:
            h = 0.04 if kind == "cube" else 0.01
            col = GREEN if kind == "cube" else GREY
            xs = [cx - ex / 2, cx + ex / 2, cx + ex / 2, cx - ex / 2, cx - ex / 2]
            ys = [cy - ey / 2, cy - ey / 2, cy + ey / 2, cy + ey / 2, cy - ey / 2]
            ax.plot(xs, ys, [surface + h] * 5, color=col, lw=1.2)
            for xa, ya in zip(xs[:4], ys[:4]):
                ax.plot([xa, xa], [ya, ya], [surface, surface + h], color=col, lw=0.8)
            if kind not in labelled:
                ax.text(cx + ex / 2 + 0.01, cy, surface + h, "cubes" if kind == "cube" else "pad", fontsize=7.5, color=col)
                labelled.add(kind)
    cells = plan("left")[0]["cells"]
    zs = sorted({c[2] for c in cells})
    for zi, z in enumerate(zs):
        pts = [(c[0], c[1], c[2]) for c in cells if c[2] == z]
        xs, ys, zz = zip(*pts)
        col = [BLUE, RED][zi % 2]
        ax.plot(xs, ys, zz, "-", color=col, lw=1.4, label="camera path %.2f m above the surface" % (z - surface))
        ax.scatter(xs, ys, zz, s=12, color=col, depthshade=False)
        ax.quiver(xs[0], ys[0], zz[0], xs[1] - xs[0], ys[1] - ys[0], 0, color=col, arrow_length_ratio=0.35, lw=1.6)
    for (xa, ya, za) in [(c[0], c[1], c[2]) for c in cells if c[2] == zs[0]]:
        ax.plot([xa, xa], [ya, ya], [surface, za], color="0.7", lw=0.5, ls=":")
    ax.set_xlabel("x (m)", labelpad=-3); ax.set_ylabel("y (m)", labelpad=-3); ax.set_zlabel("z (m)", labelpad=0)
    ax.set_zticks([surface, zs[0], zs[1]]); ax.set_zticklabels(["%.2f" % surface, "%.2f" % zs[0], "%.2f" % zs[1]])
    ax.set_xticks([0.3, 0.4, 0.5, 0.6]); ax.set_yticks([0.3, 0.4, 0.5])
    ax.tick_params(labelsize=7.5, pad=-1)
    ax.view_init(elev=22, azim=-58); ax.set_box_aspect((1.25, 1.0, 0.7), zoom=1.12)
    ax.legend(loc="lower center", frameon=False, fontsize=7.5, bbox_to_anchor=(0.5, -0.06), ncol=1)
    ax.set_title("(a) one pass of the sweep, left arm, 12 stops", loc="left", fontsize=9, fontweight="bold", pad=-4, x=-0.02)

    # (b) views captured per cell, both arms, one grid each stacked so the cells are readable
    captured = set(prov["sources"])
    gsb = gs[0, 1].subgridspec(2, 1, hspace=0.6)
    for row, (arm, cmap, col) in enumerate((("right", plt.cm.Reds, RED), ("left", plt.cm.Blues, BLUE))):
        ax = fig.add_subplot(gsb[row, 0])
        pss = plan(arm); n_per = len(pss[0]["cells"]); count = {}
        for pi, ps in enumerate(pss):
            for ci, c in enumerate(ps["cells"]):
                key = "%s_p%d_c%02d" % (arm, pi + 1, pi * n_per + ci + 1)
                kk = (round(c[0], 3), round(c[1], 3))
                count.setdefault(kk, [0, 0]); count[kk][1] += 1
                if key in captured: count[kk][0] += 1
        xsu = sorted({k[0] for k in count}); ysu = sorted({k[1] for k in count})
        for (x, y), (n, tot) in count.items():
            i, j = xsu.index(x), ysu.index(y)
            ax.add_patch(Rectangle((i, j), 1, 1, fc=cmap(0.2 + 0.7 * n / tot), ec="white", lw=1.5))
            ax.text(i + 0.5, j + 0.5, "%d" % n, ha="center", va="center", fontsize=8.5, color="white" if n / tot > 0.5 else "0.2")
        ax.set_xlim(0, len(xsu)); ax.set_ylim(0, len(ysu)); ax.set_aspect("equal")
        ax.set_xticks([i + 0.5 for i in range(len(xsu))]); ax.set_xticklabels(["%.2f" % v for v in xsu], fontsize=7.5)
        ax.set_yticks([j + 0.5 for j in range(len(ysu))]); ax.set_yticklabels(["%.2f" % v for v in ysu], fontsize=7.5)
        ax.tick_params(length=0)
        for s in ax.spines.values(): s.set_visible(False)
        ax.set_ylabel("y (m)", fontsize=8); ax.set_xlabel("x (m)", fontsize=8, labelpad=1)
        n_arm = sum(v[0] for v in count.values()); t_arm = sum(v[1] for v in count.values())
        ax.set_title("%s arm: %d of %d views captured" % (arm, n_arm, t_arm), fontsize=8.5, color=col, loc="left", pad=2)
        if row == 0:
            ax.text(-0.28, 1.32, "(b) views captured per cell, of 6 planned", transform=ax.transAxes, fontsize=9,
                    fontweight="bold", va="bottom")

    # (c) the fused map against the truth, objects numbered
    gsc = gs[1, :].subgridspec(1, 2, width_ratios=[1.55, 1.0], wspace=0.02)
    ax = fig.add_subplot(gsc[0, 0])
    for (cx, cy), (ex, ey), kind in TRUTH:
        ax.add_patch(Rectangle((cx - ex / 2, cy - ey / 2), ex, ey, fc="none", ec="0.25", lw=1.3, ls="--"))
    objs = sorted(w["objects"], key=lambda o: (not o.get("graspable"), o["centre"][0]))
    rows = []
    for n, o in enumerate(objs, 1):
        cx, cy, _ = o["centre"]; ex, ey, ez = o["extents"]
        col = GREEN if o.get("graspable") else AMBER
        ax.add_patch(Rectangle((cx - ex / 2, cy - ey / 2), ex, ey, fc=col, alpha=0.3, ec=col, lw=1.2))
        ax.text(cx, cy + (0.045 if o.get("graspable") else 0.0), "%d" % n, ha="center", va="center", fontsize=8,
                fontweight="bold", color="0.15", bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec=col, lw=0.8))
        near = min(TRUTH, key=lambda t: (t[0][0] - cx) ** 2 + (t[0][1] - cy) ** 2)
        off = 1000 * ((near[0][0] - cx) ** 2 + (near[0][1] - cy) ** 2) ** 0.5
        rows.append((n, "%.0f x %.0f x %.0f mm" % (1000 * ex, 1000 * ey, 1000 * ez), off, o.get("graspable"), near[2]))
    ax.set_xlim(-0.62, 0.62); ax.set_ylim(0.36, 0.64); ax.set_aspect("equal")
    ax.set_xlabel("x (m), positive to the wearer's left", labelpad=1); ax.set_ylabel("y (m), forward")
    ax.add_patch(Rectangle((0, 0), 0, 0, fc="none", ec="0.25", ls="--", label="true object"))
    ax.add_patch(Rectangle((0, 0), 0, 0, fc=GREEN, alpha=0.3, ec=GREEN, label="measured, fits the 85 mm jaw"))
    ax.add_patch(Rectangle((0, 0), 0, 0, fc=AMBER, alpha=0.3, ec=AMBER, label="measured, too wide to grasp"))
    ax.legend(frameon=False, loc="upper center", ncol=2, fontsize=7.5, bbox_to_anchor=(0.5, -0.28),
              handletextpad=0.4, columnspacing=1.0)
    ax.set_title("(c) the fused map against the true scene", loc="left", fontsize=9, fontweight="bold")

    # (d) what each numbered object is
    ax = fig.add_subplot(gsc[0, 1]); ax.axis("off")
    ax.set_title("(d) what the map holds", loc="left", fontsize=9, fontweight="bold")
    lines = [("surface height", "%.4f m, true 1.250 m" % surface)]
    for n, size, off, g, kind in rows:
        if g:
            lines.append(("%d  cube" % n, "%s\n%.0f mm from its true centre" % (size, off)))
        elif off < 60:
            lines.append(("%d  pad" % n, "%s\n%.0f mm off; merged with a\nfragment of its own outline" % (size, off)))
        else:
            lines.append(("%d  spurious strip" % n, "%s\nno object there" % size))
    y = 0.98
    for head, body in lines:
        ax.text(0.02, y, head, transform=ax.transAxes, fontsize=8, fontweight="bold", va="top", color="0.15")
        ax.text(0.42, y, body, transform=ax.transAxes, fontsize=7.5, va="top", color="0.25", linespacing=1.15)
        y -= 0.075 + 0.055 * body.count("\n")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT); plt.close(fig); print("wrote", OUT)
    for r in rows: print("  ", r)


if __name__ == "__main__":
    main()
