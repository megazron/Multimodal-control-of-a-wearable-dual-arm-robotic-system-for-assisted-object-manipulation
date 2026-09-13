#!/usr/bin/env python3
"""The pilot's overall result in one figure: four outcome measures, direct
control against shared autonomy, one point per operator (n = 5).

Every point is one operator's mean over that operator's trials in that
condition, so each box holds five values and the design stays balanced:
five operators, two tasks, two conditions. Grey lines join the same
operator across the two conditions, because the comparison is within
subject.

The box is the interquartile range with the median, and the whiskers span
the full range, because at n = 5 a 1.5 IQR fence calls ordinary operators
outliers. Checked by verify_overall_results_figure.py.

Data: extras/thesis/thesis_v3/figures/pilot/data/study_summary.json (duration, robot
hand path, re-grips) and tlx_trials_long.csv (raw NASA-TLX).

    python3 extras/thesis/thesis_v3/figures/make_overall_results_figure.py
"""
import collections
import csv
import json
import os
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "pilot", "data")
OUTS = [os.path.join(HERE, "pilot", "overall_results_box.pdf"),
        "/mnt/c/Users/Gausms/Downloads/overall_results_box.pdf"]
PNG = "/mnt/c/Users/Gausms/Downloads/overall_results_box.png"

W = 6.76
BLUE, GREEN = "#2c6fbb", "#3f9142"          # validated pair, direct / shared
INK, MUTED, GRID = "#1a1a1a", "#5c5c5c", "#d8d8d8"
plt.rcParams.update({
    "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8, "legend.fontsize": 8.5,
    "savefig.bbox": "tight", "axes.spines.top": False, "axes.spines.right": False,
    "axes.edgecolor": "#8a8a8a", "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
})
OPS = ["P1", "P2", "P3", "P4", "P5"]


def per_operator():
    """One mean per operator per condition, for each measure."""
    sess = json.load(open(os.path.join(DATA, "study_summary.json")))["sessions"]
    g = collections.defaultdict(list)
    for s in sess:
        g[(s["participant"], s["condition"])].append(s)
    out = {}
    for key, field in (("time", "duration_s"), ("path", "ee_path_m"), ("regrip", "clutch_engagements")):
        out[key] = {c: [st.mean(x[field] for x in g[(p, c)]) for p in OPS] for c in ("Direct", "Shared")}

    rows = list(csv.DictReader(open(os.path.join(DATA, "tlx_trials_long.csv"))))
    t = collections.defaultdict(list)
    for r in rows:
        t[(r["participant"], r["condition"])].append(float(r["rtlx"]))
    out["rtlx"] = {"Direct": [st.mean(t[(p, "VR")]) for p in OPS],
                   "Shared": [st.mean(t[(p, "SharedAutonomy")]) for p in OPS]}
    return out


PANELS = [("time",   "(a) time to finish", "seconds per trial",     "%.0f",  "s"),
          ("regrip", "(b) re-grips",       "clutch engagements",    "%.1f",  ""),
          ("rtlx",   "(c) reported workload", "NASA-TLX (RTLX, 0-100)", "%.1f", ""),
          ("path",   "(d) the robot's hand path", "metres per trial", "%.2f", "m")]


def panel(ax, direct, shared, title, ylab, fmt, unit):
    pairs = [direct, shared]
    # whis=(0, 100): the whiskers span the full range. With five values per box
    # a 1.5 IQR fence calls ordinary operators outliers -- in the robot-path
    # panel it would exclude two of the five -- so every point sits inside them.
    bp = ax.boxplot(pairs, positions=[0, 1], widths=0.46, showfliers=False, whis=(0, 100),
                    medianprops=dict(color=INK, lw=1.6, solid_capstyle="butt"),
                    boxprops=dict(lw=1.0), whiskerprops=dict(lw=1.0, color="#8a8a8a"),
                    capprops=dict(lw=1.0, color="#8a8a8a"), patch_artist=True)
    for box, col in zip(bp["boxes"], (BLUE, GREEN)):
        box.set(facecolor=col, alpha=0.16, edgecolor=col, lw=1.2)
    # the same operator, joined: this is a within-subject comparison
    for a, b in zip(direct, shared):
        ax.plot([0.18, 0.82], [a, b], "-", color="#b4b4b4", lw=0.9, zorder=2)
    rng = np.random.default_rng(4)
    for x, vals, col in ((0.18, direct, BLUE), (0.82, shared, GREEN)):
        jit = x + rng.uniform(-0.045, 0.045, len(vals))
        ax.scatter(jit, vals, s=30, color=col, zorder=4, linewidths=1.2, edgecolors="white")
    lo, hi = min(direct + shared), max(direct + shared)
    pad = 0.16 * (hi - lo)
    ax.set_ylim(lo - pad, hi + 2.4 * pad)
    ax.set_xlim(-0.55, 1.55)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["direct\ncontrol", "shared\nautonomy"])
    ax.set_ylabel(ylab)
    ax.set_title(title, loc="left", fontweight="bold", pad=6)
    ax.yaxis.grid(True, color=GRID, lw=0.7); ax.set_axisbelow(True)
    md, ms = st.mean(direct), st.mean(shared)
    arrow = "↓" if ms < md else "↑"
    ax.text(0.5, 0.985, ("mean " + fmt + " " + arrow + " " + fmt + " %s") % (md, ms, unit),
            transform=ax.transAxes, ha="center", va="top", fontsize=8, color=MUTED)


def main():
    d = per_operator()
    fig, axes = plt.subplots(2, 2, figsize=(W, 6.0))
    for ax, (key, title, ylab, fmt, unit) in zip(axes.ravel(), PANELS):
        panel(ax, d[key]["Direct"], d[key]["Shared"], title, ylab, fmt, unit)
    handles = [plt.Line2D([], [], marker="o", ls="", ms=6, mfc=BLUE, mec="white", mew=1.2, label="direct control"),
               plt.Line2D([], [], marker="o", ls="", ms=6, mfc=GREEN, mec="white", mew=1.2, label="shared autonomy"),
               plt.Line2D([], [], color="#b4b4b4", lw=0.9, label="the same operator, both conditions")]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.5, 0.005), handletextpad=0.5, columnspacing=1.6)
    fig.text(0.5, 0.995, "Five operators, two tasks, two conditions: one point per operator (n = 5)",
             ha="center", va="top", fontsize=9, color=MUTED)
    fig.subplots_adjust(hspace=0.46, wspace=0.34, top=0.915, bottom=0.115)
    for o in OUTS:
        fig.savefig(o)
    fig.savefig(PNG, dpi=300)
    plt.close(fig)
    for o in OUTS + [PNG]:
        print("wrote", o)
    for key, title, *_ in PANELS:
        a, b = d[key]["Direct"], d[key]["Shared"]
        print("  %-7s direct %s  shared %s" % (key, ["%.1f" % v for v in a], ["%.1f" % v for v in b]))


if __name__ == "__main__":
    main()
