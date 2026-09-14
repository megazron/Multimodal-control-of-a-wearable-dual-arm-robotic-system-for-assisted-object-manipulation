#!/usr/bin/env python3
"""Three separate panels per cohort: (a) the master arm / operator's hand,
(b) the simulated robot's hand (what RViz shows), (c) the real robot's hand
(forward kinematics from its own joint encoders). One row for the VR cohort,
one for the mannequin-master cohort. Each path is coloured from the start
(dark) to the end (yellow) of the session; start and end are marked.

    python3 extras/figures/make_trajectory_abc.py

Reads the FK'd data the trajectory gallery already extracted
(figures/gallery/trajectories/trajectories.json) so no ROS is needed.
"""

import json
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "gallery", "trajectories", "trajectories.json")
OUT = os.path.join(HERE, "pilot", "trajectory_abc.pdf")

plt.rcParams.update({"font.size": 8, "axes.titlesize": 9})

# VR: P4, object tracking, direct (the session already used elsewhere in the
# report); mannequin: M2, target reaching, direct (the longest master session).
PICK = {"VR": ("P5", "Pick and place", "Direct"),
        "master": ("M2", "Target reaching", "Direct")}


def pick(sessions, cohort):
    p, task, cond = PICK[cohort]
    cands = [s for s in sessions if s["cohort"] == cohort and s["participant"] == p
             and s["task"] == task and s["condition"] == cond]
    return max(cands, key=lambda s: len(s["t"]))


def path_len(P):
    return float(sum(math.dist(a, b) for a, b in zip(P[:-1], P[1:])))


def draw(ax, P, t, title):
    P = np.array(P, float)
    ok = ~np.isnan(P).any(1)
    P, t = P[ok], np.array(t, float)[ok]
    segs = np.stack([P[:-1], P[1:]], 1)
    lc = Line3DCollection(segs, cmap="viridis", linewidths=1.3)
    lc.set_array((t[:-1] - t[0]) / max(t[-1] - t[0], 1e-9))
    ax.add_collection3d(lc)
    ax.scatter(*P[0], color="black", marker="o", s=28, zorder=5)
    ax.scatter(*P[-1], color="black", marker="s", s=28, zorder=5)
    lo, hi = P.min(0), P.max(0)
    span = max((hi - lo).max(), 0.05)
    mid = (lo + hi) / 2
    for f, m in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), mid):
        f(m - span / 2, m + span / 2)
    ax.set_xlabel("x (m)", labelpad=-1)
    ax.set_ylabel("y (m)", labelpad=-1)
    ax.set_zlabel("z (m)", labelpad=-3)
    ax.tick_params(labelsize=6, pad=0)
    ax.set_title(title, pad=2)
    return lc, path_len(P), t[-1] - t[0]


def main():
    d = json.load(open(DATA))
    fig = plt.figure(figsize=(7.4, 5.6))
    rows = [("VR", "VR controllers"), ("master", "mannequin master")]
    cols = [("op", "(a) master arm / operator's hand"),
            ("sim", "(b) simulated robot's hand (RViz)"),
            ("real", "(c) real robot's hand (its own encoders)")]
    lc = None
    for r, (cohort, cname) in enumerate(rows):
        s = pick(d["sessions"], cohort)
        for c, (key, label) in enumerate(cols):
            ax = fig.add_subplot(2, 3, r * 3 + c + 1, projection="3d")
            lc, L, T = draw(ax, s[key], s["t"], label if r == 0 else "")
            ax.text2D(0.02, 0.96, "%s, %s: %.2f m in %.0f s" % (
                s["participant"], cname, L, T), transform=ax.transAxes, fontsize=6.5, color="0.3")
        # sim vs real gap for this session, for the caption
        gaps = [math.dist(a, b) for a, b in zip(s["sim"], s["real"]) if a and b]
        print(cohort, s["participant"], s["task"], s["condition"], s["hand"],
              "gap RMS %.1f mm" % (1000 * (sum(g * g for g in gaps) / len(gaps)) ** 0.5))
    fig.subplots_adjust(left=0.02, right=0.98, top=0.93, bottom=0.10, wspace=0.05, hspace=0.12)
    cax = fig.add_axes([0.30, 0.045, 0.40, 0.015])
    cb = fig.colorbar(lc, cax=cax, orientation="horizontal")
    cb.set_label("time into session (start = dark, end = yellow);  circle = start, square = end", fontsize=7)
    cb.set_ticks([0, 1]); cb.set_ticklabels(["start", "end"])
    fig.savefig(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
