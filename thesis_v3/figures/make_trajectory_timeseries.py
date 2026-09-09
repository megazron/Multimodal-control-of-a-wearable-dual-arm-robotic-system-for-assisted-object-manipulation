#!/usr/bin/env python3
"""Readable trajectories: position against time, three lines (left-right,
forward-back, up-down) per panel, one panel per source -- (a) the master arm
or operator's hand, (b) the simulated robot's hand, (c) the real robot's hand
from its own encoders -- one row for a VR session, one for a mannequin-master
session. Panels (b) and (c) share their vertical scale so a reader can see
they match.

    python3 thesis_v3/figures/make_trajectory_timeseries.py

Reads the FK'd data the trajectory gallery extracted
(figures/gallery/trajectories/trajectories.json); no ROS needed.
"""

import json
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "gallery", "trajectories", "trajectories.json")
OUT = os.path.join(HERE, "pilot", "trajectory_timeseries.pdf")

BLUE, RED, GREEN = "#2c6fbb", "#c1392b", "#3f9142"
plt.rcParams.update({"font.size": 16.9, "axes.titlesize": 19.0,
                     "axes.spines.top": False, "axes.spines.right": False})

PICK = {"VR": ("P5", "Pick and place", "Direct"),
        "master": ("M2", "Target reaching", "Direct")}


def pick(sessions, cohort):
    p, task, cond = PICK[cohort]
    c = [s for s in sessions if s["cohort"] == cohort and s["participant"] == p
         and s["task"] == task and s["condition"] == cond]
    return max(c, key=lambda s: len(s["t"]))


def main():
    d = json.load(open(DATA))
    fig, axes = plt.subplots(2, 3, figsize=(7.4, 4.6), sharex="row")
    cols = [("op", "(a) master arm / operator's hand"),
            ("sim", "(b) simulated robot's hand"),
            ("real", "(c) real robot's hand (encoders)")]
    for r, (cohort, cname) in enumerate([("VR", "VR controllers"), ("master", "mannequin master")]):
        s = pick(d["sessions"], cohort)
        t = np.array(s["t"], float); t = t - t[0]
        arrs = {}
        for key, _ in cols:
            P = np.array(s[key], float)
            arrs[key] = P
        lo = min(np.nanmin(arrs["sim"]), np.nanmin(arrs["real"]))
        hi = max(np.nanmax(arrs["sim"]), np.nanmax(arrs["real"]))
        pad = 0.05 * (hi - lo)
        gaps = [math.dist(a, b) for a, b in zip(s["sim"], s["real"]) if a and b]
        gap = 1000 * (sum(g * g for g in gaps) / len(gaps)) ** 0.5
        for c, (key, title) in enumerate(cols):
            ax = axes[r, c]
            P = arrs[key]
            for j, (lab, colr) in enumerate([("left-right (x)", BLUE), ("forward-back (y)", GREEN), ("up-down (z)", RED)]):
                ax.plot(t, P[:, j], color=colr, lw=1.0, label=lab if (r == 0 and c == 0) else None)
            if key != "op":
                ax.set_ylim(lo - pad, hi + pad)
            if r == 0:
                ax.set_title(title)
            if c == 0:
                ax.set_ylabel("%s\nposition (m)" % cname)
            if r == 1:
                ax.set_xlabel("time into session (s)")
            ax.grid(True, lw=0.3, alpha=0.5)
        axes[r, 2].text(0.03, 0.97, "gap between (b) and (c): %.0f mm" % gap,
                        transform=axes[r, 2].transAxes, ha="left", va="top", fontsize=7, color="0.35",
                        bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1))
        print(cohort, s["participant"], s["task"], s["condition"], "gap %.1f mm" % gap, "T=%.0f s" % t[-1])
    fig.legend(loc="lower center", ncol=3, frameon=False, fontsize=8, bbox_to_anchor=(0.5, -0.01))
    fig.subplots_adjust(left=0.09, right=0.99, top=0.92, bottom=0.17, wspace=0.28, hspace=0.25)
    fig.savefig(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
