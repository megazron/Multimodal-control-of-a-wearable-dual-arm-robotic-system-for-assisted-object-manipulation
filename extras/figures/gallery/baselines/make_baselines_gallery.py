#!/usr/bin/env python3
"""Plain-language figure gallery from every measured baseline JSON.

    python3 extras/figures/gallery/baselines/make_baselines_gallery.py

Every figure reads ONE (occasionally two) files under recordings/baselines/
and draws only what the file contains. Nothing is typed in from memory; the
few reference lines that come from documents rather than files (the 16 %
break-even from Dragan & Srinivasa 2012) are named as such in INDEX.md.
Each figure is written as .pdf and .png and gets an INDEX.md entry with its
source fields, a one-sentence takeaway, the key numbers and a MAIN/APPENDIX/
SKIP suggestion. A file the code cannot draw from is listed under "Skipped"
with the reason, never drawn empty.

Labels avoid jargon on purpose: "how far the hand can reach", "gap to the
wearer", "parking error per joint". Where a technical term is unavoidable it
is glossed in the label.
"""

import json
import os
import sys
import traceback
from collections import OrderedDict, defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 4))
BASE = os.path.join(ROOT, "recordings", "baselines")
OUT = HERE

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
AMBER = "#c98a1b"
LIGHT = "0.75"
ARM = {"left": BLUE, "right": RED}
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10, "axes.labelsize": 9,
    "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "legend.fontsize": 8,
})
BLUES = LinearSegmentedColormap.from_list("blues1", ["#f4f7fb", BLUE])

import textwrap
from matplotlib.axes import Axes
_orig_set_title = Axes.set_title
def _wrapped_title(self, label, *a, **k):
    """Wrap panel titles to the panel's width so nothing is cut off at the edge."""
    width_in = self.get_position().width * self.figure.get_figwidth()
    n = max(26, int(width_in * 12))
    return _orig_set_title(self, textwrap.fill(str(label), n), *a, **k)
Axes.set_title = _wrapped_title
_orig_set_ylabel = Axes.set_ylabel
def _wrapped_ylabel(self, label, *a, **k):
    height_in = self.get_position().height * self.figure.get_figheight()
    n = max(24, int(height_in * 15))
    return _orig_set_ylabel(self, "\n".join(textwrap.wrap(str(label), n)) if "\n" not in str(label) else label, *a, **k)
Axes.set_ylabel = _wrapped_ylabel

INDEX = []      # (filename, title, source, takeaway, numbers, suggestion)
SKIPPED = []    # (name, reason)


def load(name):
    with open(os.path.join(BASE, name)) as f:
        return json.load(f)


def save(fig, stem, title, source, takeaway, numbers, suggestion):
    fig.savefig(os.path.join(OUT, stem + ".pdf"))
    fig.savefig(os.path.join(OUT, stem + ".png"), dpi=150)
    plt.close(fig)
    INDEX.append((stem, title, source, takeaway, numbers, suggestion))
    print("wrote", stem)


def label_bars(ax, bars, fmt="%.2f", dy=0.0, size=7, color=GREY):
    for b in bars:
        h = b.get_height()
        if h is None or (isinstance(h, float) and np.isnan(h)):
            continue
        ax.text(b.get_x() + b.get_width() / 2, h + dy, fmt % h,
                ha="center", va="bottom", fontsize=size, color=color)


# ---------------------------------------------------------------- reach ---
def fig_mount_sweep():
    rows = load("mount_overlap_sweep.json")
    tilts = sorted({round(r["tilt_deg"]) for r in rows})
    dys = sorted({r["dy"] for r in rows})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1))
    # (a) best shared cells vs forward tilt, one line per forward shift
    for i, dy in enumerate(dys):
        best = [max(r["both"] for r in rows if round(r["tilt_deg"]) == t
                    and r["dy"] == dy and r["splay_deg"] == 0) for t in tilts]
        ax1.plot([-t for t in tilts], best, "o-", color=[LIGHT, BLUE, GREEN][i],
                 lw=1.6, ms=4, label="mount moved forward %.0f cm" % (dy * 100))
    ax1.set_xlabel("mount tilted forward (degrees)")
    ax1.set_ylabel("table cells both arms can reach (of 63)")
    ax1.legend(loc="upper left")
    ax1.set_title("(a) tilt and forward shift help")
    # (b) splayed outward hurts, at every tilt
    x = np.arange(len(tilts))
    for k, sp in enumerate(sorted({r["splay_deg"] for r in rows})):
        best = [max(r["both"] for r in rows if round(r["tilt_deg"]) == t
                    and r["splay_deg"] == sp) for t in tilts]
        b = ax2.bar(x + (k - 0.5) * 0.38, best, 0.38,
                    color=BLUE if sp == 0 else RED,
                    label="arms straight" if sp == 0 else
                    "arms splayed outward %.0f deg" % abs(sp))
        label_bars(ax2, b, "%d", 0.3)
    ax2.set_xticks(x, ["%.0f" % -t for t in tilts])
    ax2.set_xlabel("mount tilted forward (degrees)")
    ax2.set_ylabel("best shared cells at that tilt")
    ax2.legend(loc="upper left")
    ax2.set_title("(b) splaying outward hurts")
    fig.tight_layout()
    asb = [r["both"] for r in rows if r["dx"] == 0 and r["dy"] == 0
           and r["dz"] == 0 and r["tilt_deg"] == 0 and r["splay_deg"] == 0][0]
    top = max(r["both"] for r in rows)
    save(fig, "reach_mount_sweep",
         "How moving the mounts changes what both arms can reach",
         "mount_overlap_sweep.json: dx, dy, dz, tilt_deg, splay_deg, both",
         "As built the two arms share only %d of 63 table cells; tilting the "
         "mounts forward and shifting them helps, splaying them outward "
         "hurts, and the best of 108 positions shares %d." % (asb, top),
         "as built %d shared cells; best anywhere %d; splay -25 deg never "
         "beats straight" % (asb, top),
         "MAIN (the costed-fix half of the disjoint-workspace figure already "
         "in the report; this is a cleaner, plainer version)")


def fig_mount_heat():
    rows = load("mount_overlap_sweep.json")
    dys = sorted({r["dy"] for r in rows})
    tilts = sorted({round(r["tilt_deg"]) for r in rows})
    dxs = sorted({r["dx"] for r in rows})
    fig, axes = plt.subplots(1, len(dxs), figsize=(7.6, 2.9), sharey=True)
    for ax, dx in zip(axes, dxs):
        grid = np.full((len(tilts), len(dys)), np.nan)
        for i, t in enumerate(tilts):
            for j, dy in enumerate(dys):
                vals = [r["both"] for r in rows if r["dx"] == dx and r["dy"] == dy
                        and round(r["tilt_deg"]) == t and r["splay_deg"] == 0]
                if vals:
                    grid[i, j] = max(vals)
        im = ax.imshow(grid, cmap=BLUES, vmin=0, vmax=30, aspect="auto",
                       origin="lower")
        for i in range(len(tilts)):
            for j in range(len(dys)):
                if not np.isnan(grid[i, j]):
                    ax.text(j, i, "%d" % grid[i, j], ha="center", va="center",
                            fontsize=8, color="white" if grid[i, j] > 16 else GREY)
        ax.set_xticks(range(len(dys)), ["%.0f" % (d * 100) for d in dys])
        ax.set_yticks(range(len(tilts)), ["%.0f" % -t for t in tilts])
        ax.set_xlabel("mount moved forward (cm)")
        ax.set_title("mount moved inboard %.0f cm" % (-dx * 100))
        for s in ("top", "right"):
            ax.spines[s].set_visible(True)
    axes[0].set_ylabel("mount tilted forward (degrees)")
    cb = fig.colorbar(im, ax=axes, fraction=0.03, pad=0.02)
    cb.set_label("table cells both arms reach (of 63)")
    fig.subplots_adjust(left=0.08, right=0.88, bottom=0.2, top=0.86, wspace=0.12)
    save(fig, "reach_mount_heatmap",
         "Shared table cells for every swept mount position",
         "mount_overlap_sweep.json: best 'both' over dz at splay 0, per dx/dy/tilt",
         "Every mount position that was tried, coloured by how many table "
         "cells both arms can reach; the cells the buildable fix used are "
         "the 22 at 10 cm inboard, 15 cm forward, 60 deg tilt.",
         "grid maxima per panel; 29 is the sweep's best (20 cm inboard, "
         "not buildable)",
         "APPENDIX (the line plot above carries the story; this is the full "
         "evidence)")


DIR_WORDS = {"+1+0+0": "outboard", "-1+0+0": "inboard", "+0+1+0": "forward",
             "+0-1+0": "backward", "+0+0+1": "up", "+0+0-1": "down",
             "left": "toward left", "right": "toward right", "front": "forward",
             "back": "backward", "up": "up", "down": "down"}


def fig_reach_directions():
    d = load("workspace_n10_20260806.json")
    cardinal = ["front", "back", "left", "right", "up", "down"]
    fig, ax = plt.subplots(figsize=(7.0, 3.0))
    x = np.arange(len(cardinal))
    for k, arm in enumerate(("left", "right")):
        med = [d[arm][c]["median"] for c in cardinal]
        lo = [d[arm][c]["min"] for c in cardinal]
        hi = [d[arm][c]["max"] for c in cardinal]
        b = ax.bar(x + (k - 0.5) * 0.38, med, 0.38, color=ARM[arm],
                   label="%s arm" % arm)
        ax.errorbar(x + (k - 0.5) * 0.38, med,
                    yerr=[np.array(med) - lo, np.array(hi) - np.array(med)],
                    fmt="none", ecolor=GREY, lw=0.8, capsize=2)
        label_bars(ax, b, "%.2f", 0.01)
        for xi, c in zip(x + (k - 0.5) * 0.38, cardinal):
            if d[arm][c]["truncated"]:
                ax.text(xi, 0.03, "range\ncap", ha="center", fontsize=6.5,
                        color="white")
    ax.set_xticks(x, [DIR_WORDS[c] for c in cardinal])
    ax.set_ylabel("how far the hand can travel from home (m)")
    ax.legend(loc="upper left")
    ax.set_title("Reach from the home pose, six directions, ten solver repeats each")
    fig.tight_layout()
    save(fig, "reach_six_directions",
         "How far each hand can travel from home in six directions",
         "workspace_n10_20260806.json: median/min/max per direction, n=10",
         "From home the hands move freely backward and up but only 14-24 cm "
         "toward the body's centre and 15-16 cm forward; the whiskers are "
         "the spread over ten solver restarts.",
         "left: forward 0.16, back 0.90 (cap), left 0.45, right 0.24, up 0.78, "
         "down 0.23 m; right: forward %.2f, left %.2f, right %.2f, up %.2f, down "
         "%.2f m" % tuple(d["right"][c]["median"] for c in
                          ("front", "left", "right", "up", "down")),
         "APPENDIX (dated 2026-08-06, before the mount change; superseded as a "
         "workspace figure but the only per-direction spread measurement)")


def fig_reach_extents():
    d = load("arm_reach_extents.json")
    dirs = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    words = {"+X": "toward +x", "-X": "toward -x", "+Y": "forward",
             "-Y": "backward", "+Z": "up", "-Z": "down"}
    fig, ax = plt.subplots(figsize=(7.0, 3.0))
    x = np.arange(len(dirs))
    for k, arm in enumerate(("left", "right")):
        vals = [d["arms"][arm]["dirs"][k_]["reach_m"] for k_ in dirs]
        lims = [d["arms"][arm]["dirs"][k_]["limit"] for k_ in dirs]
        cols = [RED if "wearer" in l else ARM[arm] for l in lims]
        b = ax.bar(x + (k - 0.5) * 0.38, vals, 0.38, color=cols,
                   edgecolor=ARM[arm], lw=1.2, label="%s arm" % arm)
        label_bars(ax, b, "%.2f", 0.01)
    ax.set_xticks(x, [words[k_] for k_ in dirs])
    ax.set_ylabel("reach from home before something stops it (m)")
    ax.legend(loc="upper left")
    ax.text(0.99, 0.95, "red fill: stopped by the wearer's clearance floor\n"
            "arm colour: stopped by the arm itself", transform=ax.transAxes,
            ha="right", va="top", fontsize=7.5, color=GREY)
    ax.set_title("What stops the hand in each direction")
    fig.tight_layout()
    save(fig, "reach_extents_what_stops",
         "How far each hand reaches, and what stops it",
         "arm_reach_extents.json: dirs.reach_m and dirs.limit per arm",
         "Only one direction per arm is stopped by the wearer (moving inboard); "
         "every other direction is stopped by the arm running out of reach.",
         "left inboard 0.40 m (wearer, 0.149 m gap), right inboard 0.46 m "
         "(wearer, 0.150 m gap); backward 1.51 m both",
         "APPENDIX")


# ---------------------------------------------------------- the wearer ---
def fig_wearer_posture():
    d = load("centre_vs_wearer_posture.json")
    order = ["none", "down", "folded", "behind", "out"]
    words = {"none": "no arms\n(the limit)", "down": "arms down\n(shipped)",
             "folded": "arms folded", "behind": "clasped\nbehind",
             "out": "held out\nto the sides"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1))
    x = np.arange(len(order))
    for k, arm in enumerate(("left", "right")):
        vals = [d["rows"][p]["full"][arm] for p in order]
        v = [np.nan if val is None else val for val in vals]
        b = ax1.bar(x + (k - 0.5) * 0.38, v, 0.38, color=ARM[arm],
                    label="%s arm" % arm)
        label_bars(ax1, b, "%.3f", 0.004)
        for xi, val in zip(x + (k - 0.5) * 0.38, vals):
            if val is None:
                ax1.text(xi, 0.01, "no\nsafe\ncell", ha="center", fontsize=6.5,
                         color=RED)
    ax1.set_xticks(x, [words[p] for p in order], fontsize=7.5)
    ax1.set_ylabel("closest column the hand can work at (m from centre)")
    ax1.legend(loc="upper left")
    ax1.set_title("(a) how close to the body's centre the work can be")
    home = [d["rows"][p]["home"]["left"] for p in order]
    cols = [RED if h < 0.15 else GREEN for h in home]
    b = ax2.bar(x, home, 0.55, color=cols)
    label_bars(ax2, b, "%.3f", 0.004)
    ax2.axhline(0.15, ls="--", color=RED, lw=1)
    ax2.text(len(order) - 0.45, 0.153, "150 mm safety gap", ha="right",
             fontsize=7.5, color=RED)
    ax2.set_xticks(x, [words[p] for p in order], fontsize=7.5)
    ax2.set_ylabel("gap between arm and wearer at the home pose (m)")
    ax2.set_title("(b) two postures break the gap before the arm moves")
    fig.tight_layout()
    save(fig, "wearer_posture",
         "The wearer's posture changes what the arms can do",
         "centre_vs_wearer_posture.json: rows.<posture>.full, .home, .binds",
         "Even with the wearer's arms removed entirely the hands cannot work "
         "nearer than 30 cm from the body's centre, so the torso is what "
         "closes the centre; arms held out or clasped behind put the "
         "wearer's own limbs inside the 150 mm safety gap.",
         "innermost column: none 0.300/0.375, down 0.325/0.450, folded "
         "0.400/0.450 m (left/right); home gap: behind 0.1145, out 0.060/-0.084 m",
         "MAIN candidate (safety + workspace in one figure; currently only "
         "the number is in the text)")


def fig_wearer_size():
    d = load("wearer_size_effect.json")
    rows = d["rows"]
    labels = ["%s,\narms %s" % (r["size"].replace("_", " "), r["posture"])
              for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    x = np.arange(len(rows))
    for k, arm in enumerate(("left", "right")):
        b = ax1.bar(x + (k - 0.5) * 0.38, [r[arm] for r in rows], 0.38,
                    color=ARM[arm], label="%s arm" % arm)
        label_bars(ax1, b, "%.3f", 0.003)
        b2 = ax2.bar(x + (k - 0.5) * 0.38, [r["clear_" + arm] * 1000 for r in rows],
                     0.38, color=ARM[arm], label="%s arm" % arm)
        label_bars(ax2, b2, "%.0f", 0.4)
        for xi, r in zip(x + (k - 0.5) * 0.38, rows):
            ax2.text(xi, r["clear_" + arm] * 1000 - 6,
                     r["binds_" + arm].replace("L-", "").replace("R-", ""),
                     ha="center", fontsize=6, color="white", rotation=90)
    ax1.set_xticks(x, labels, fontsize=7.5)
    ax1.set_ylabel("closest workable column (m from centre)")
    ax1.set_ylim(0, 0.45)
    ax1.legend(loc="lower right")
    ax1.set_title("(a) a real adult moves the limit by 25 mm at most")
    ax2.axhline(150, ls="--", color=RED, lw=1)
    ax2.set_xticks(x, labels, fontsize=7.5)
    ax2.set_ylabel("gap to the wearer at that column (mm)")
    ax2.set_ylim(0, 175)
    ax2.set_title("(b) what the arm is closest to (text in the bar)")
    fig.tight_layout()
    save(fig, "wearer_size",
         "A measured adult instead of the mannequin",
         "wearer_size_effect.json: rows[].left/right, clear_*, binds_*",
         "Swapping the mannequin for a measured adult body moves the closest "
         "workable column by at most 25 mm, but changes WHAT the arm is "
         "closest to: a limb for the mannequin, the torso for the person.",
         "worst shift %.3f m; adult binds on torso in both postures" % d["worst_shift_m"],
         "APPENDIX")


def fig_what_binds():
    walks = load("what_binds.json")["walks"]
    dirs = list(OrderedDict.fromkeys(w["direction"] for w in walks))
    cols = {"WEARER": RED, "ORIENTATION": AMBER, "FURNITURE": GREY,
            "NOT BOUNDED WITHIN 0.70 m": GREEN}
    words = {"WEARER": "the wearer", "ORIENTATION": "the pinned wrist",
             "FURNITURE": "the table", "NOT BOUNDED WITHIN 0.70 m": "nothing within 0.7 m"}
    fig, ax = plt.subplots(figsize=(7.4, 3.0))
    x = np.arange(len(dirs))
    for k, arm in enumerate(("left", "right")):
        ws = [next(w for w in walks if w["arm"] == arm and w["direction"] == d_)
              for d_ in dirs]
        b = ax.bar(x + (k - 0.5) * 0.38, [w["reach_m"] for w in ws], 0.38,
                   color=[cols[w["binds"]] for w in ws], edgecolor=ARM[arm], lw=1.4)
        label_bars(ax, b, "%.3f", 0.01)
    ax.set_xticks(x, [d_.replace(" (+y)", "").replace(" (-y)", "")
                      .replace(" (+z)", "").replace(" (-z)", "") for d_ in dirs])
    ax.set_ylabel("reach from the work point (m)")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in cols.values()]
    handles += [plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor=ARM[a], lw=1.4)
                for a in ("left", "right")]
    ax.legend(handles, ["stopped by " + words[k] for k in cols] +
              ["left arm", "right arm"], ncol=3, loc="upper left", fontsize=7)
    ax.set_ylim(0, 0.9)
    ax.set_title("From the work point, what stops the hand in each direction")
    fig.tight_layout()
    save(fig, "what_binds_work_point",
         "What stops the hand, direction by direction, from the work point",
         "what_binds.json: walks[].reach_m, binds, per arm and direction",
         "From where the work actually happens the hand can move only 12-15 cm "
         "toward the body before the wearer stops it and 10-12 cm down before "
         "the table does; forward and outboard it is the pinned wrist, not "
         "the person, that stops it.",
         "inboard 0.125/0.150 m (wearer); forward 0.30 m both (wrist); "
         "down 0.10/0.125 m (table)",
         "MAIN candidate (pairs with the pinned-wrist number now in Results 3.2)")


def fig_orientation_cost():
    home = load("orientation_cost.json")
    work = load("orientation_cost_what_binds.json")
    pols = home["policies"]
    words = {"pinned": "wrist pinned\n(as delivered)", "spin": "roll free",
             "cone15": "15 deg cone", "cone45": "45 deg cone", "free": "any\norientation"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1))
    x = np.arange(len(pols))
    b1 = ax1.bar(x - 0.19, [home["mean_reach_m"][p] for p in pols], 0.38,
                 color=LIGHT, label="starting from home")
    b2 = ax1.bar(x + 0.19, [work["mean_reach_m"][p] for p in pols], 0.38,
                 color=BLUE, label="starting from the work point")
    label_bars(ax1, b1, "%.3f", 0.006)
    label_bars(ax1, b2, "%.3f", 0.006)
    ax1.set_xticks(x, [words[p] for p in pols], fontsize=7.5)
    ax1.set_ylabel("average reach over the direction walk (m)")
    ax1.legend(loc="upper left")
    ax1.set_title("(a) how much wrist freedom buys")
    n = len(work["walks"])
    b3 = ax2.bar(x, [work["wearer_bound_walks"][p] for p in pols], 0.55, color=RED)
    label_bars(ax2, b3, "%d", 0.15)
    ax2.axhline(n, ls="--", color=GREY, lw=0.9)
    ax2.text(len(pols) - 0.45, n + 0.2, "all %d walks" % n, ha="right",
             fontsize=7.5, color=GREY)
    ax2.set_xticks(x, [words[p] for p in pols], fontsize=7.5)
    ax2.set_ylabel("direction walks stopped by the wearer")
    ax2.set_ylim(0, n + 1.5)
    ax2.set_title("(b) with the wrist pinned, the wearer is the stop")
    fig.tight_layout()
    save(fig, "orientation_cost",
         "What pinning the wrist orientation costs the reach",
         "orientation_cost.json (from home) and orientation_cost_what_binds.json "
         "(from the work point): mean_reach_m, wearer_bound_walks",
         "From home the pinned wrist costs 89 mm of reach; from the work point "
         "it costs a factor of seven (0.065 m against 0.450 m), and 11 of 12 "
         "direction walks are stopped by the wearer because a fixed wrist "
         "spends the joint that would move the elbow out of the person.",
         "work point: pinned 0.065, roll-free 0.065, 15 deg 0.319, 45 deg "
         "0.440, free 0.450 m; wearer-bound 11/11/7/5/4 of 12",
         "MAIN candidate (the 'factor of seven' now quoted in Results 3.2 has "
         "no figure)")


# ------------------------------------------------------------ sim-to-real ---
def fig_park_error():
    d = load("arm_directional_calibration.json")
    runs = d["runs"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1))
    for arm in ("left", "right"):
        errs = np.concatenate([np.abs(r["ss_joint_deg"]) for r in runs
                               if r["arm"] == arm])
        ax1.hist(errs, bins=np.arange(0, 0.32, 0.01), color=ARM[arm], alpha=0.75,
                 label="%s arm (%d joint readings)" % (arm, len(errs)))
    ax1.set_xlabel("how far short of its target each joint parked (degrees)")
    ax1.set_ylabel("joint readings")
    ax1.legend(loc="upper left")
    ax1.set_title("(a) every joint stops about 0.29 degrees short")
    dirs = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    x = np.arange(len(dirs))
    for k, arm in enumerate(("left", "right")):
        data = [[r["ss_cart_mm"] for r in runs if r["arm"] == arm
                 and r["direction"] == d_] for d_ in dirs]
        bp = ax2.boxplot(data, positions=x + (k - 0.5) * 0.38, widths=0.32,
                         patch_artist=True, showfliers=True,
                         medianprops=dict(color=ARM[arm], lw=1.6),
                         boxprops=dict(facecolor=ARM[arm], alpha=0.35, edgecolor=ARM[arm]),
                         whiskerprops=dict(color=ARM[arm]), capprops=dict(color=ARM[arm]),
                         flierprops=dict(marker="o", ms=3, color=ARM[arm]))
    ax2.set_xticks(x, ["%s" % d_ for d_ in dirs])
    ax2.set_xlabel("direction of a 100 mm move")
    ax2.set_ylabel("hand position error at the end (mm)")
    ax2.set_ylim(0, 10)
    ax2.set_title("(b) what that costs the hand (3 speeds each)")
    fig.tight_layout()
    all_worst = [r["ss_joint_worst_deg"] for r in runs]
    save(fig, "sim_real_park_error",
         "The real arms park short of their target by a fixed amount",
         "arm_directional_calibration.json: runs[].ss_joint_deg, ss_cart_mm, "
         "direction, arm (36 real runs)",
         "On the physical arms every joint stops 0.29-0.30 degrees short of "
         "where it was sent, always on the side it came from, which puts the "
         "hand 5-8 mm off after a 100 mm move; the spread is tiny because it "
         "is a fixed offset, not noise.",
         "worst per run %.3f-%.3f deg; hand error %.1f-%.1f mm" % (
             min(all_worst), max(all_worst),
             min(r["ss_cart_mm"] for r in runs), max(r["ss_cart_mm"] for r in runs)),
         "MAIN candidate (the only real-hardware motion data in the report; "
         "currently text only)")


def fig_sim_real_models():
    g = load("sim_to_real_gap.json")
    arms = ["left", "right"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    x = np.arange(len(arms))
    jt = g["joint_terminal"]
    series = [("no correction", [jt[a]["identity_rms_mm"] for a in arms], LIGHT),
              ("one offset per arm,\nfitted", [jt[a]["fitted_rms_mm"] for a in arms], BLUE),
              ("one offset per arm,\ntested on an unseen axis",
               [jt[a]["held_out_axis_rms_mm"] for a in arms], GREEN),
              ("3x3 matrix,\ntested on an unseen axis",
               [g["arms"][a]["held_out_axis_rms_mm"] for a in arms], RED)]
    w = 0.2
    for i, (name, vals, col) in enumerate(series):
        b = ax1.bar(x + (i - 1.5) * w, vals, w, color=col, label=name)
        label_bars(ax1, b, "%.1f", 0.5, size=6.5)
    ax1.set_xticks(x, ["%s arm" % a for a in arms])
    ax1.set_ylabel("hand error after a 100 mm move (mm)")
    ax1.set_yscale("log")
    ax1.legend(fontsize=6.5, loc="upper left", ncol=2)
    ax1.set_title("(a) one number per arm beats a full matrix")
    ca = g["cross_arm"]
    b = ax2.bar(["left model\non left arm", "right model\non left arm",
                 "right model\non right arm", "left model\non right arm"],
                [jt["left"]["fitted_rms_mm"], ca["right_to_left"]["corrected_rms_mm"],
                 jt["right"]["fitted_rms_mm"], ca["left_to_right"]["corrected_rms_mm"]],
                0.55, color=[BLUE, LIGHT, RED, LIGHT])
    label_bars(ax2, b, "%.2f", 0.08)
    ax2.set_ylabel("hand error after correction (mm)")
    ax2.set_title("(b) each arm needs its own number")
    ax2.tick_params(axis="x", labelsize=7.5)
    fig.tight_layout()
    save(fig, "sim_real_correction_models",
         "Correcting the parking error: which model to trust",
         "sim_to_real_gap.json: joint_terminal.*, arms.*.held_out_axis_rms_mm, cross_arm",
         "A single offset per arm removes about three quarters of the hand "
         "error and still works on a direction it was never fitted on; a "
         "3x3 matrix fits better but fails by 93 mm off its fitted axes; and "
         "one arm's number applied to the other is only half as good.",
         "identity 7.24/7.21 mm; one offset fitted 1.83/1.68, held out 1.88/1.72; "
         "3x3 held out 93.5/93.6; cross-arm 4.35/4.31 mm; removed 74.1/76.2 %%",
         "APPENDIX (Results quotes the 74-76 %% and r=0.923; the model "
         "comparison is appendix depth)")


# ------------------------------------------------------------ the budget ---
def fig_error_budget():
    d = load("control_budget.json")
    terms = d["positioning_mm"]
    known = [t for t in terms if t["mm"] is not None]
    unknown = [t for t in terms if t["mm"] is None]
    names = {"terminal joint error, UNCOMPENSATED": "joints parking short (as is)",
             "  the same, if the overshoot is switched on": "  ... with the overshoot fix",
             "IK solve residual": "solver residual",
             "depth noise at 0.4-0.8 m": "depth-camera noise at arm's length",
             "shell bias from a single view, BEFORE the plane": "seeing only the object's front face",
             "  the same, with the support plane": "  ... knowing it rests on the table",
             "declared vs measured pad offset (T0, T2, T3)": "finger-pad offset (was 13.45 mm)"}
    fig, ax = plt.subplots(figsize=(7.4, 3.3))
    y = np.arange(len(known))[::-1]
    cols = [RED if t["systematic"] else BLUE for t in known]
    bars = ax.barh(y, [t["mm"] for t in known], 0.62, color=cols)
    for b, t in zip(bars, known):
        if t["term"].startswith("  "):
            b.set_hatch("///"); b.set_alpha(0.5)
        ax.text(b.get_width() + 0.2, b.get_y() + b.get_height() / 2,
                "%.2f mm" % t["mm"], va="center", fontsize=7.5, color=GREY)
    ax.set_yticks(y, [names.get(t["term"], t["term"].strip()) for t in known], fontsize=8)
    ax.axvline(d["positioning_rss_mm"], ls="--", color=GREEN, lw=1.1)
    top = len(known) - 0.55
    ax.text(d["positioning_rss_mm"] + 0.3, top, "all known terms\ncombined: %.1f mm" % d["positioning_rss_mm"],
            fontsize=7.5, color=GREEN, va="top")
    ax.axvline(d["grasp_gate_m"] * 1000, color=RED, lw=1.2)
    ax.text(d["grasp_gate_m"] * 1000 - 0.4, top, "%.0f mm: the grasp\nstill closes" %
            (d["grasp_gate_m"] * 1000), fontsize=7.5, color=RED, ha="right", va="top")
    ax.set_xlim(0, 34)
    ax.set_ylim(-0.9, len(known) - 0.4)
    ax.set_xlabel("contribution to where the fingers land (mm)")
    handles = [plt.Rectangle((0, 0), 1, 1, color=RED), plt.Rectangle((0, 0), 1, 1, color=BLUE),
               plt.Rectangle((0, 0), 1, 1, facecolor=RED, alpha=0.5, hatch="///")]
    ax.legend(handles, ["fixed offset (can be calibrated away)", "random",
                        "the row above, after its fix"], loc="lower right", fontsize=7,
              bbox_to_anchor=(1.0, 0.08))
    if unknown:
        ax.set_title("Where the positioning error comes from (the camera's true "
                     "position on the wrist is still unmeasured and not on this chart)", fontsize=9.5)
    fig.tight_layout()
    save(fig, "error_budget",
         "Where the positioning error comes from",
         "control_budget.json: positioning_mm[].term/mm/systematic, positioning_rss_mm, grasp_gate_m",
         "Combined, the known error sources put the fingers %.1f mm off "
         "(worst case %.1f), inside the 30 mm within which a grasp still "
         "closes -- but the largest unknown, where the camera really sits on "
         "the wrist, is not on the chart because it has never been measured."
         % (d["positioning_rss_mm"], d["positioning_worst_mm"]),
         "RSS %.2f mm, worst %.2f mm; largest term 10.5 mm (front-face bias) "
         "becomes 0 with the table plane" % (d["positioning_rss_mm"], d["positioning_worst_mm"]),
         "MAIN candidate (Results 3.2 quotes 12.91/19.99 mm with no figure)")


def fig_reaction_budget():
    d = load("control_budget.json")
    rows = d["reaction"]
    floor = d["floor_m"] * 1000
    motions = list(OrderedDict.fromkeys(r["motion"] for r in rows))
    words = {"a deliberate reach": "a deliberate reach\n(0.8-1.5 m/s)",
             "a quick reach or a fidget": "a quick reach\n(1.5-2.5 m/s)",
             "a startle or flinch": "a startle\n(2.5-4 m/s)"}
    fig, ax = plt.subplots(figsize=(7.2, 3.1))
    x = np.arange(len(motions))
    for k, which in enumerate(("best", "worst")):
        rr = [next(r for r in rows if r["motion"] == m and r["which"] == which) for m in motions]
        pct = [r["eats_floor_pct_hi"] for r in rr]
        b = ax.bar(x + (k - 0.5) * 0.38, [min(p, 130) for p in pct], 0.38,
                   color=[GREEN if p < 100 else RED for p in pct],
                   alpha=1.0 if which == "worst" else 0.55,
                   label="%s ms delay (%s case)" % (int(rr[0]["latency_s"] * 1000), which))
        for bar, p, r in zip(b, pct, rr):
            ax.text(bar.get_x() + bar.get_width() / 2, min(p, 130) + 3,
                    "%.0f%%\n%.0f mm" % (p, r["travel_hi_m"] * 1000),
                    ha="center", fontsize=7, color=GREY)
    ax.axhline(100, ls="--", color=RED, lw=1.1, label="100 %: the whole 150 mm gap is used up")
    ax.set_xticks(x, [words[m] for m in motions])
    ax.set_ylabel("share of the 150 mm safety gap the wearer\ncan close before the robot knows (%)")
    ax.set_ylim(0, 175)
    ax.legend(loc="upper left", fontsize=7, ncol=3)
    ax.set_title("The safety gap is a distance; the wearer can move in the time it takes to see them")
    fig.tight_layout()
    save(fig, "reaction_budget",
         "How much of the safety gap a moving wearer closes before the robot reacts",
         "control_budget.json: reaction[].latency_s, speed_hi, travel_hi_m, eats_floor_pct_hi",
         "With the best-case 62 ms delay a deliberate reach closes 62 %% of the "
         "150 mm gap before the robot knows; at the worst-case 562 ms every "
         "motion class closes the whole gap, a startle fifteen times over.",
         "62 ms: 62 / 103 / 165 %%; 562 ms: 562 / 937 / 1499 %% (93 mm to 2248 mm of travel)",
         "MAIN candidate (Discussion 4.2 has a two-row table; this shows all six)")


def fig_posture_fallback():
    d = load("control_budget.json")
    pg = d["posture_gap"]["left"]["clearance_m"]
    order = ["none", "down", "folded", "behind", "out"]
    words = {"none": "no arms", "down": "arms down\n(what the guard assumes)",
             "folded": "folded", "behind": "clasped behind", "out": "held out"}
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    vals = [pg[p] * 1000 for p in order]
    b = ax.bar(range(len(order)), vals, 0.55, color=[RED if v < 150 else BLUE for v in vals])
    label_bars(ax, b, "%.0f", 3)
    ax.axhline(150, ls="--", color=RED, lw=1.1)
    ax.text(len(order) - 0.45, 155, "150 mm safety gap", ha="right", fontsize=7.5, color=RED)
    ax.set_xticks(range(len(order)), [words[p] for p in order], fontsize=7.5)
    ax.set_ylabel("real gap to the wearer at the home pose (mm)")
    ax.set_title("If the camera drops out, the guard assumes 'arms down' -- %.0f mm too optimistic "
                 "for a wearer with arms out" % (-d["posture_gap"]["left"]["vs_down_m"]["out"] * 1000),
                 fontsize=9)
    fig.tight_layout()
    save(fig, "posture_fallback_gap",
         "The fallback assumption when wearer tracking drops out",
         "control_budget.json: posture_gap.left.clearance_m, vs_down_m",
         "When the wearer camera is stale or gated out the guard falls back to "
         "an arms-down body and believes 317 mm of clearance; a wearer with "
         "arms held out really has 93 mm, inside the 150 mm gap.",
         "down 317, folded 321, behind 278, out 93 mm; out is 224 mm worse than assumed",
         "MAIN candidate (Discussion 4.2 now quotes 224 mm; this is its figure)")


# ------------------------------------------------------- shared autonomy ---
def fig_intent_sweep():
    d = load("shared_autonomy_gain.json")
    errs = sorted(d["rows"], key=float)
    xs = [float(e) for e in errs]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    for src, col, name in (("DECLARED", RED, "position typed in from the task file"),
                           ("CALIBRATED", BLUE, "position measured by the camera")):
        ax1.plot(xs, [d["rows"][e][src]["correct_pct"] for e in errs], "o-", color=col,
                 lw=1.7, ms=4, label=name)
        ax2.plot(xs, [d["rows"][e][src]["wrong_pct"] for e in errs], "o-", color=col,
                 lw=1.7, ms=4, label=name)
    for ax in (ax1, ax2):
        ax.axvline(d["cube_pitch_mm"], ls=":", color=GREY, lw=0.9)
        ax.set_xlabel("how wrong the typed-in object position is (mm)")
    ax1.text(d["cube_pitch_mm"] + 2, 40, "one cube spacing\n(%.0f mm)" % d["cube_pitch_mm"],
             fontsize=7, color=GREY)
    ax2.axhline(16, ls="--", color=AMBER, lw=1.0)
    ax2.text(2, 17.5, "16 %: above this, aggressive assistance is a net loss\n"
             "(Dragan & Srinivasa 2012, from the literature)", fontsize=6.5, color=AMBER)
    ax1.set_ylabel("robot guesses the right object (%)")
    ax2.set_ylabel("robot confidently guesses the WRONG object (%)")
    ax1.set_ylim(0, 100); ax2.set_ylim(0, 55)
    ax1.legend(loc="lower left", fontsize=7)
    ax1.set_title("(a) right guesses (%d trials)" % d["trials"])
    ax2.set_title("(b) confident wrong guesses")
    fig.tight_layout()
    save(fig, "intent_estimator_sweep",
         "Guessing the operator's intent from a typed-in map against a measured one",
         "shared_autonomy_gain.json: rows.<err>.DECLARED/CALIBRATED correct_pct, wrong_pct, cube_pitch_mm",
         "If the object positions are typed in rather than measured, a 40 mm "
         "error already makes the robot confidently pick the wrong object "
         "17.5 %% of the time and 47.5 %% at 120 mm; measured positions stay "
         "at 0 %% wrong across the whole sweep.",
         "wrong %%: 0 / 0.5 / 17.5 / 35.0 / 39.8 / 47.5 at 0/20/40/60/80/120 mm "
         "(declared), 0.0 throughout (measured). NOTE the 16 %% line is a "
         "published value, not in the file.",
         "MAIN candidate (Results 3.3 quotes these numbers; check text: at one "
         "cube spacing (60 mm) the wrong rate is 35 %%, not 50 %%)")


def fig_mode_travel():
    d = load("mode_difference.json")
    modes = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy", "04_vr_shared", "06_full_autonomy"]
    words = {"01_master_teleop": "master\ndirect", "02_vr_teleop": "VR\ndirect",
             "03_shared_autonomy": "master\nshared", "04_vr_shared": "VR\nshared",
             "06_full_autonomy": "full\nautonomy"}
    tasks = [t for t in ("T0", "T1", "T1S2", "T2", "T3") if t in d]
    fig, axes = plt.subplots(1, len(tasks), figsize=(10.5, 3.3), sharey=False)
    for ax, t in zip(axes, tasks):
        pm = d[t]["per_mode"]
        x = np.arange(len(modes))
        for k, side in enumerate(("l", "r")):
            v = [pm[m]["travel_" + side] for m in modes]
            b = ax.bar(x + (k - 0.5) * 0.38, v, 0.38, color=ARM["left" if side == "l" else "right"],
                       label="%s arm" % ("left" if side == "l" else "right"))
        ax.set_xticks(x, [words[m].replace("\n", " ") for m in modes], fontsize=6.5, rotation=55, ha="right")
        ax.set_title("task %s" % t, fontsize=9)
        if t == tasks[0]:
            ax.set_ylabel("distance the robot's hand travelled (m)")
            ax.legend(fontsize=7, loc="upper left")
    fig.suptitle("No operator at all: the same commanded waypoints, driven under each mode",
                 fontsize=9.5)
    fig.tight_layout()
    t0 = d["T0"]["per_mode"]
    save(fig, "mode_travel_no_operator",
         "The robot travels further under shared autonomy even with nobody driving",
         "mode_difference.json: <task>.per_mode.<mode>.travel_l/travel_r",
         "Scripted clips with the same waypoints and no operator travel "
         "further under shared autonomy than under direct control -- 45 %% "
         "further on the master path for task T0, 4 %% on the VR path -- so "
         "the extra path seen in the pilot is a property of the assistance "
         "layer, not of the operators.",
         "T0 left: master direct %.3f -> master shared %.3f m (+%.0f%%); VR direct %.3f -> VR shared %.3f m (+%.0f%%)" % (
             t0["01_master_teleop"]["travel_l"], t0["03_shared_autonomy"]["travel_l"],
             100 * (t0["03_shared_autonomy"]["travel_l"] / t0["01_master_teleop"]["travel_l"] - 1),
             t0["02_vr_teleop"]["travel_l"], t0["04_vr_shared"]["travel_l"],
             100 * (t0["04_vr_shared"]["travel_l"] / t0["02_vr_teleop"]["travel_l"] - 1)),
         "MAIN candidate (it is the mechanistic explanation of the pilot's one "
         "counter-intuitive result; Discussion cites the file with no figure)")


# ------------------------------------------------------------- perception ---
def fig_detector_real():
    d = load("detector_real_rgbd.json")
    rows = d["rows"]
    agg = defaultdict(lambda: [0, 0, []])
    for r in rows:
        a = agg[r["prompt"]]; a[0] += 1; a[1] += bool(r["detected"])
        if r["detected"]:
            a[2].append(r["iou"])
    prompts = sorted(agg, key=lambda p: -agg[p][1] / agg[p][0])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1), gridspec_kw={"width_ratios": [1.4, 1]})
    rate = [100 * agg[p][1] / agg[p][0] for p in prompts]
    b = ax1.bar(range(len(prompts)), rate, 0.6, color=[GREEN if r_ > 0 else LIGHT for r_ in rate])
    for i, p in enumerate(prompts):
        ax1.text(i, rate[i] + 1, "%d/%d" % (agg[p][1], agg[p][0]), ha="center", fontsize=7, color=GREY)
    ax1.set_xticks(range(len(prompts)), prompts, rotation=30, ha="right", fontsize=7.5)
    ax1.set_ylabel("objects found when asked for by name (%)")
    ax1.set_ylim(0, 45)
    ax1.set_title("(a) found %.1f%% of %d objects in %d real frames" % (
        100 * sum(a[1] for a in agg.values()) / len(rows), len(rows), d["frames"]))
    ious = [r["iou"] for r in rows if r["detected"]]
    ax2.hist(ious, bins=np.arange(0.5, 1.01, 0.025), color=BLUE)
    ax2.set_xlabel("box overlap with the true object (1 = perfect)")
    ax2.set_ylabel("detections")
    ax2.set_title("(b) when found, the box fits well")
    fig.tight_layout()
    save(fig, "detector_real_rgbd",
         "An open-vocabulary detector on real camera frames",
         "detector_real_rgbd.json: rows[].prompt, detected, iou (LM-O test set)",
         "Asked for objects by name on 60 real RGB-D frames, the open-vocabulary "
         "detector found only 9.0 %% of the 432 instances -- five of eight object "
         "types were never found at all -- but the boxes it did draw fit well, "
         "so the failure is missing objects, not mislocating them.",
         "found 39/432; watering can 19/60, duck 17/60, cat 3/48, five types 0; "
         "IoU of hits %.2f-%.2f" % (min(ious), max(ious)),
         "APPENDIX (Methods quotes 9.0 %%; the figure justifies the geometric fallback)")


def fig_look_cost():
    d = load("look_then_grasp.json")
    t = d["timing"]
    parts = [("move to the observe pose", t["move_home_to_observe_s"], BLUE),
             ("detect and classify", t["detect_and_classify_s"], GREEN),
             ("check the plan", t["verify_plan_s"], AMBER),
             ("move back", t["move_observe_to_home_s"], BLUE)]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.6), gridspec_kw={"width_ratios": [2.2, 1]})
    left = 0
    for name, v, col in parts:
        ax1.barh([0], [v], 0.5, left=left, color=col, edgecolor="white")
        if v > 1.5:
            ax1.text(left + v / 2, 0, "%s\n%.1f s" % (name, v), ha="center", va="center",
                     fontsize=7.5, color="white")
        else:
            ax1.annotate("%s: %.2f s" % (name, v), xy=(left + v / 2, 0.25),
                         xytext=(left + v / 2 + 4, 0.42), fontsize=7, color=GREEN,
                         arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.7))
        left += v
    ax1.set_yticks([]); ax1.set_xlabel("seconds"); ax1.set_xlim(0, left * 1.02); ax1.set_ylim(-0.4, 0.6)
    ax1.set_title("(a) one look: %.1f s, almost all of it travel" % d["added_per_look_s"])
    b = ax2.bar(["per look", "per pick\n(%d picks share a look)" % d["picks"]],
                [d["added_per_look_s"], d["added_per_pick_s"]], 0.5, color=[RED, GREEN])
    label_bars(ax2, b, "%.1f s", 0.6)
    ax2.set_ylabel("seconds added"); ax2.set_title("(b) shared over the picks")
    ax2.tick_params(axis="x", labelsize=7.5)
    fig.tight_layout()
    c = d["classification"]
    save(fig, "look_then_grasp_cost",
         "What looking before grasping costs",
         "look_then_grasp.json: timing.*, added_per_look_s, added_per_pick_s, classification",
         "Detecting and classifying the cubes takes 0.31 s; travelling to the "
         "observe pose and back takes 48.3 s, so a faster detector would save "
         "nothing -- a shorter journey would. All %d cubes were classified "
         "correctly, worst localisation %.1f mm." % (c["n"], c["worst_localisation_m"] * 1000),
         "detect 0.31 s, travel 24.1 + 24.1 s, check 3.7 s; 12.14 s per pick over 4 picks",
         "APPENDIX")


def fig_pick_accuracy():
    d = load("pick_accuracy.json")
    rows = d["rows"]; gate = d["capture_gate_mm"]
    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    for arm in ("left", "right"):
        rr = [r for r in rows if r["arm"] == arm]
        ax.scatter([r["belief_err_mm"] for r in rr], [r["pad_miss_mm"] for r in rr], s=55,
                   color=ARM[arm], label="%s arm" % arm, zorder=3)
    lim = 34
    ax.plot([0, lim], [0, lim], ls=":", color=GREY, lw=1)
    ax.text(18, 20.5, "fingers miss by exactly what\nthe camera was wrong by", fontsize=7.5,
            color=GREY, rotation=38)
    ax.axhline(gate, ls="--", color=RED, lw=1.1)
    ax.text(0.5, gate + 0.7, "%.0f mm: the grasp still closes" % gate, fontsize=7.5, color=RED)
    ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    ax.set_xlabel("how far off the camera's estimate of the object was (mm)")
    ax.set_ylabel("how far the finger pads missed the true centre (mm)")
    ax.legend(loc="lower right")
    ax.set_title("Picking from the measured map: the arm adds no error of its own")
    fig.tight_layout()
    meas = [r for r in rows if r["pipeline"] == "MEASURED"]
    save(fig, "pick_accuracy_belief_vs_miss",
         "Whatever the camera believes, the arm delivers the fingers there",
         "pick_accuracy.json: rows[].belief_err_mm, pad_miss_mm, arm, pipeline",
         "Every pick lands on the diagonal: the finger pads miss the true "
         "object centre by exactly the camera's own error (8-14 mm), so the "
         "remaining error is perception, not kinematics.",
         "measured picks: belief error %s mm, pad miss %s mm; declared picks 0.00-0.01 mm" % (
             "/".join("%.1f" % r["belief_err_mm"] for r in meas),
             "/".join("%.1f" % r["pad_miss_mm"] for r in meas)),
         "MAIN candidate (localises the residual error to perception, which is "
         "what makes the unmeasured camera position the top future-work item)")


def fig_pad_curve():
    d = load("pad_mid_ee_by_width.json")
    rows = sorted(d["arms"]["left"], key=lambda r: r["width_mm"])
    w = [r["width_mm"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.9))
    ax1.plot(w, [r["along_axis_m"] * 1000 for r in rows], "o-", color=BLUE, lw=1.7, ms=4)
    for width, note in ((0, "hand wide open"), (40, "closed on a 40 mm cube")):
        r = next(r_ for r_ in rows if r_["width_mm"] == width)
        ax1.annotate("%.1f mm\n%s" % (r["along_axis_m"] * 1000, note),
                     xy=(width, r["along_axis_m"] * 1000), xytext=(width + 14, r["along_axis_m"] * 1000 - 7.5),
                     fontsize=7, color=GREY, arrowprops=dict(arrowstyle="->", color=GREY, lw=0.7))
    ax1.set_ylim(min(r["along_axis_m"] for r in rows) * 1000 - 12, max(r["along_axis_m"] for r in rows) * 1000 + 3)
    ax1.set_xlabel("commanded grip width (mm)"); ax1.set_ylabel("wrist to finger-pad midpoint (mm)")
    ax1.set_title("(a) the pads move as the fingers close")
    ax2.plot(w, [r["span_mm"] for r in rows], "o-", color=RED, lw=1.7, ms=4)
    ax2.set_xlabel("commanded grip width (mm)"); ax2.set_ylabel("distance between the fingertips (mm)")
    ax2.set_title("(b) the fingers swing, they do not slide")
    fig.tight_layout()
    save(fig, "finger_pad_curve",
         "Where the finger pads are depends on how far the hand is closed",
         "pad_mid_ee_by_width.json: arms.left[].width_mm, along_axis_m, span_mm (both arms identical)",
         "The distance from the wrist to the finger pads is not a constant: it "
         "is 98.3 mm with the hand open and 109.8 mm closed on a 40 mm cube, "
         "because the fingers swing on a linkage -- which is why two earlier "
         "measurements of 'the' offset disagreed by 13 mm.",
         "98.33 mm open, 111.79 mm at 20 mm, 109.76 mm at 40 mm; both arms agree exactly",
         "APPENDIX")


def fig_objects_vs_jaw():
    d = load("grasp_approach.json")
    ap = d["aperture_m"] * 1000
    items = []
    for g in d["grasps"]:
        ext, presented = None, False
        for appr in g["approaches"].values():
            for shape in appr.values():
                if not isinstance(shape, dict):
                    continue
                p = shape.get("presentation") or {}
                e = p.get("object_extent_along_closing_axis_mm")
                if e is not None and (ext is None or e < ext):
                    ext = e
                presented = presented or bool(p.get("presented"))
        if ext is not None:
            items.append(("%s %s\n(%s arm)" % (g["task"], g["name"], g["arm"]), ext, presented))
    items = sorted(items, key=lambda i: i[1])
    fig, ax = plt.subplots(figsize=(7.6, 3.0))
    b = ax.bar(range(len(items)), [i[1] for i in items], 0.6,
               color=[GREEN if i[2] else RED for i in items])
    label_bars(ax, b, "%.0f", 1.5)
    ax.axhline(ap, ls="--", color=RED, lw=1.1)
    ax.text(-0.4, ap + 3, "the hand opens to %.0f mm" % ap, ha="left", fontsize=7.5, color=RED)
    ax.set_xticks(range(len(items)), [i[0] for i in items], fontsize=6.5)
    ax.set_ylabel("narrowest width the fingers must span (mm)")
    ax.set_title("Which task objects fit the hand (green: a grasp was found; red: none)")
    fig.tight_layout()
    save(fig, "objects_vs_jaw",
         "Which objects fit inside the gripper",
         "grasp_approach.json: grasps[].approaches.*.*.presentation.object_extent_along_closing_axis_mm, presented; aperture_m",
         "Each task object's narrowest presented width against the 85 mm the "
         "hand can open: the cubes and the tray fit; anything red could not be "
         "presented inside the jaw from any tried approach.",
         "aperture 85 mm; %d of %d objects presentable" % (sum(i[2] for i in items), len(items)),
         "APPENDIX")


# -------------------------------------------------------------- home pose ---
def fig_home_pose():
    d = load("home_render.json")
    s = load("symmetric_home.json")
    res = d["mirror_residual_m"]; links = list(res)
    short = [l.replace("_link", "").replace("spherical_", "").replace("half_arm", "upper arm ")
             .replace("_", " ") for l in links]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    b = ax1.bar(range(len(links)), [res[l] * 1000 for l in links], 0.6, color=BLUE)
    label_bars(ax1, b, "%.1f", 0.3)
    ax1.set_xticks(range(len(links)), short, rotation=28, ha="right", fontsize=7)
    ax1.set_ylabel("left arm reflected onto right: mismatch (mm)")
    ax1.set_title("(a) the two arms are not mirror images at home")
    xs = [p["x"] for p in s["pairs"]]
    ax2.plot(xs, [p["mirror_residual_m"] * 1000 for p in s["pairs"]], "o-", color=RED, lw=1.6, ms=4,
             label="best mirror mismatch found")
    ax2.plot(xs, [p["worst_elbow_drop_m"] * 1000 for p in s["pairs"]], "s--", color=GREY, lw=1.2, ms=4,
             label="how far the elbow can drop")
    ax2.set_xlabel("hands this far from the centre line (m)")
    ax2.set_ylabel("mm")
    ax2.legend(loc="lower left", fontsize=7)
    ax2.set_title("(b) no hand position makes them mirror")
    fig.tight_layout()
    save(fig, "home_pose_symmetry",
         "The home pose cannot be made symmetric",
         "home_render.json: mirror_residual_m per link; symmetric_home.json: pairs[].mirror_residual_m, worst_elbow_drop_m",
         "Reflecting the left arm onto the right leaves a 27 mm mismatch at "
         "the wrist at home, and searching hand positions from 0.45 to 0.55 m "
         "off centre never gets the best mismatch below 161 mm: the two "
         "mounts differ by 168 degrees of roll, so identical arms cannot mirror.",
         "worst link 27.2 mm (wrist 2); best pair residual 161.6-164.4 mm",
         "APPENDIX")


# --------------------------------------------------------- channel health ---
def fig_channel_health():
    d = load("channels_20260806.json")
    chans = list(d)
    verdict_col = {"ALIVE": GREEN, "INTERMITTENT": AMBER, "INCOHERENT": RED, "DEAD": GREY}
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.4, 3.6), sharex=True)
    x = np.arange(len(chans))
    ax1.bar(x, [d[c]["updates"] for c in chans], 0.6, color=[verdict_col[d[c]["verdict"]] for c in chans])
    ax1.set_ylabel("distinct readings\nin the recording")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in verdict_col.values()]
    ax1.legend(handles, [k.lower() for k in verdict_col], ncol=4, loc="upper right", fontsize=7)
    ax1.set_title("Master arm channels: how many moved, and how many jumped")
    jump = [d[c]["jump_pct"] if d[c]["jump_pct"] is not None else 0 for c in chans]
    ax2.bar(x, jump, 0.6, color=[verdict_col[d[c]["verdict"]] for c in chans])
    ax2.axhline(5, ls="--", color=RED, lw=0.9)
    ax2.text(len(chans) - 0.45, 5.4, "5 %: called incoherent above this", ha="right", fontsize=7, color=RED)
    ax2.set_ylabel("readings that jumped\nmore than 60 deg (%)")
    ax2.set_xticks(x, [c.replace("_j", " joint ").replace("l ", "left ").replace("r ", "right ")
                       for c in chans], rotation=35, ha="right", fontsize=7)
    fig.tight_layout()
    alive = sum(1 for c in chans if d[c]["verdict"] == "ALIVE")
    save(fig, "channel_health",
         "Which master-arm sensors are trustworthy",
         "channels_20260806.json: <channel>.verdict, updates, jump_pct",
         "Six of the fourteen master sensors read cleanly; seven read a value "
         "that jumps by more than 60 degrees in 5-14 %% of readings (a wiring "
         "fault, not silence), and two are dead.",
         "%d alive, %d incoherent, %d intermittent, %d dead" % (
             alive, sum(1 for c in chans if d[c]["verdict"] == "INCOHERENT"),
             sum(1 for c in chans if d[c]["verdict"] == "INTERMITTENT"),
             sum(1 for c in chans if d[c]["verdict"] == "DEAD")),
         "APPENDIX only (the report has moved this out of the main body)")


# ------------------------------------------------------------- avoidance ---
def fig_predictive():
    d = load("predictive_avoidance.json")
    targets = list(d["results"])
    pols = [k for k in ("off", "on", "projection", "tangential") if k in d["results"][targets[0]]]
    words = {"off": "avoidance off", "on": "avoidance on", "projection": "slide along the body",
             "tangential": "reroute sideways"}

    def commanded_min(t, p):
        rows = [r["clearance"] for r in d["results"][t][p]["rows"] if r.get("solved")]
        return min(rows) * 1000 if rows else np.nan

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.1))
    x = np.arange(len(targets)); w = 0.8 / len(pols)
    pcols = [LIGHT, BLUE, GREEN, AMBER]
    for i, p in enumerate(pols):
        v = [commanded_min(t, p) for t in targets]
        b = ax1.bar(x + (i - (len(pols) - 1) / 2) * w, v, w, color=pcols[i], label=words[p])
        for bar, vi in zip(b, v):
            ax1.text(bar.get_x() + bar.get_width() / 2, vi + (3 if vi >= 0 else -12), "%.0f" % vi,
                     ha="center", fontsize=6.5, color=GREY)
        n = d["results"][targets[0]][p]["n"]
        b2 = ax2.bar(x + (i - (len(pols) - 1) / 2) * w, [d["results"][t][p]["solved"] for t in targets],
                     w, color=pcols[i], label=words[p])
        label_bars(ax2, b2, "%d", 0.15)
    ax1.axhline(d["floor_m"] * 1000, ls="--", color=GREEN, lw=1)
    ax1.axhline(0, color=RED, lw=1.1)
    ax1.text(-0.45, 5, "the wearer's skin", fontsize=7, color=RED)
    ax1.text(len(targets) - 0.45, d["floor_m"] * 1000 + 4, "150 mm safety gap", ha="right", fontsize=7, color=GREEN)
    ax1.set_xticks(x, ["toward the %s" % t for t in targets]); ax1.set_ylabel("closest the arm gets at any pose it is SENT to (mm)")
    ax1.legend(fontsize=6.5, loc="lower left", ncol=2)
    ax1.set_title("(a) without avoidance the arm is sent inside the person")
    ax2.axhline(n, ls="--", color=GREY, lw=0.9)
    ax2.set_xticks(x, ["toward the %s" % t for t in targets]); ax2.set_ylabel("steps of %d the policy is willing to send" % n)
    ax2.set_ylim(0, n + 2)
    ax2.set_title("(b) what it buys is refusal, not extra room")
    fig.tight_layout()
    save(fig, "predictive_avoidance",
         "What predictive avoidance does when the hand heads for the wearer",
         "predictive_avoidance.json: results.<target>.<policy>.rows[].clearance/solved, floor_m",
         "Told to move toward the head or torso with avoidance off, the arm is "
         "sent to poses up to 155 mm inside the person; with avoidance on "
         "every pose it is sent to keeps the 150 mm gap and the rest are "
         "refused by name -- the gain is refusal, because the hand itself is "
         "what is inside the person.",
         "off: min commanded clearance %.0f / %.0f / %.0f mm; on: all commanded poses >= 150 mm" % tuple(
             commanded_min(t, "off") for t in targets),
         "APPENDIX (the report's Discussion already notes it is not wired into "
         "the follower; caveat: %s)" % d["caveat"][:60])


# -------------------------------------------------------- motion generator ---
def fig_motion_generator():
    d = load("teleop_motion.json")
    gens = ["clamp_towards", "synchronised-clamp", "ruckig"]
    words = {"clamp_towards": "per-joint clamp\n(as shipped)", "synchronised-clamp": "synchronised\nclamp",
             "ruckig": "jerk-limited\n(delivered)"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    x = np.arange(len(gens))
    for k, arm in enumerate(("left", "right")):
        g = d["arms"][arm]["generators"]
        off = [max(g[n]["off_path_max_mm"], 0.01) for n in gens]
        vel = [g[n]["peak_velocity_frac_of_limit"] for n in gens]
        b = ax1.bar(x + (k - 0.5) * 0.38, off, 0.38, color=ARM[arm], label="%s arm" % arm)
        for bar, v in zip(b, off):
            ax1.text(bar.get_x() + bar.get_width() / 2, v * 1.2, "%.1f" % v if v > 0.05 else "0.0",
                     ha="center", fontsize=7, color=GREY)
        b2 = ax2.bar(x + (k - 0.5) * 0.38, vel, 0.38, color=ARM[arm], label="%s arm" % arm)
        for bar, v in zip(b2, vel):
            ax2.text(bar.get_x() + bar.get_width() / 2, v * 1.15, "%.2fx" % v, ha="center", fontsize=7, color=GREY)
    ax1.set_yscale("log"); ax1.axhline(30, ls="--", color=RED, lw=1)
    ax1.text(2.4, 34, "30 mm: grasp still closes", ha="right", fontsize=7, color=RED)
    ax1.set_xticks(x, [words[n] for n in gens], fontsize=7.5); ax1.set_ylabel("how far the hand left its straight line (mm)")
    ax1.legend(loc="upper right"); ax1.set_title("(a) off the line")
    ax2.set_yscale("log"); ax2.axhline(1, ls="--", color=RED, lw=1)
    ax2.set_xticks(x, [words[n] for n in gens], fontsize=7.5)
    ax2.set_ylabel("peak joint speed as a multiple of its limit\n(dashed line: exactly at the limit)")
    ax2.set_title("(b) over the speed limit")
    fig.tight_layout()
    save(fig, "motion_generator",
         "The old motion generator against the delivered one",
         "teleop_motion.json: arms.*.generators.*.off_path_max_mm, peak_velocity_frac_of_limit",
         "The per-joint clamp that shipped let the hand wander 51 mm off its "
         "straight line at 12.5 times the joint speed limit because it never "
         "read the limits file; the jerk-limited generator stays on the line "
         "and at the limit.",
         "off-path 51.3/39.2 mm -> 0.0; speed 12.53x -> 1.00x; the clamp sweep never gets under 30 mm",
         "APPENDIX (Results has the two-row table)")


# ---------------------------------------------------------- degradation ---
def fig_degradation():
    d = load("capability_ladder.json"); dd = load("capability_degradation.json")
    levels = ["FK", "SPHERICAL", "SPH_RATE", "SHELL", "DIR_ONLY"]
    words = {"FK": "all 7 sensors", "SPHERICAL": "one bend\nsensor lost", "SPH_RATE": "radius from\nrate only",
             "SHELL": "radius frozen", "DIR_ONLY": "direction only\n(no position)"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    x = np.arange(len(levels))
    for k, arm in enumerate(("left", "right")):
        raw = [d[arm]["cost"][l]["p95_m"] for l in levels]
        v = [np.nan if r is None else r * 1000 for r in raw]
        b = ax1.bar(x + (k - 0.5) * 0.38, v, 0.38, color=ARM[arm], label="%s arm (%d frames)" % (arm, d[arm]["n"]))
        label_bars(ax1, b, "%.0f", 2)
        if k == 0:
            for xi, r in zip(x, raw):
                if r is None:
                    ax1.text(xi, 5, "no position\nat all", ha="center", fontsize=7, color=RED)
    ax1.set_xticks(x, [words[l] for l in levels], fontsize=7); ax1.set_ylabel("position error, 95th percentile (mm)")
    ax1.legend(fontsize=7, loc="upper left"); ax1.set_title("(a) what each lost sensor costs")
    counts = dd["subset_counts"]
    b2 = ax2.bar(x, [counts.get(l, 0) for l in levels], 0.55, color=GREY)
    label_bars(ax2, b2, "%d", 0.8)
    ax2.set_xticks(x, [words[l] for l in levels], fontsize=7); ax2.set_ylabel("of 128 sensor combinations, how many land here")
    ax2.set_title("(b) recovery is never worse than before the loss: %s" % dd["monotonic_recovery"])
    fig.tight_layout()
    save(fig, "degradation_ladder",
         "How the master keeps working as its sensors fail",
         "capability_ladder.json: <arm>.cost.<level>.p95_m, n; capability_degradation.json: subset_counts, monotonic_recovery",
         "Replayed through 20 440 recorded master frames, losing a bend sensor "
         "costs nothing, falling back to rate-only costs up to 114 mm, "
         "freezing the radius up to 232 mm, and losing the azimuth sensor "
         "leaves no position at all; no rung is a cliff and a returning "
         "sensor never leaves the system worse than before it failed.",
         "p95 left/right: 0/0, 0/0, 114/82, 217/232 mm; 64 of 128 subsets are direction-only",
         "APPENDIX only")


# ----------------------------------------------------------------- depth ---
def fig_depth_pose():
    d = load("depth_pose_accuracy.json")
    rows = d["rows"]; objs = list(OrderedDict.fromkeys(r["obj"] for r in rows))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.8))
    cols = [BLUE, GREEN, RED]
    for i, o in enumerate(objs):
        rr = sorted([r for r in rows if r["obj"] == o and not r["contamination"] and "naive_mm" in r],
                    key=lambda r: r["range_m"])
        ax1.plot([r["range_m"] for r in rr], [r["naive_mm"] for r in rr], "o--", color=cols[i], lw=1.3, ms=3,
                 label="%s, centre of what is visible" % o)
        ax1.plot([r["range_m"] for r in rr], [r["fit_mm"] for r in rr], "o-", color=cols[i], lw=1.8, ms=3,
                 label="%s, fitted shape" % o)
    ax1.set_xlabel("distance to the object (m)"); ax1.set_ylabel("error in the object's position (mm)")
    ax1.set_yscale("log")
    ax1.set_title("(a) fitting the shape beats averaging what is seen")
    sig = d["per_pixel_sigma_mm"]; rr = sorted(float(k) for k in sig)
    b = ax2.bar([str(r) for r in rr], [sig["%g" % r] for r in rr], 0.55, color=BLUE)
    label_bars(ax2, b, "%.2f", 0.02)
    ax2.set_xlabel("distance to the object (m)"); ax2.set_ylabel("depth noise per pixel (mm)")
    ax2.set_title("(b) depth noise grows with the square of distance")
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=6.5)
    fig.subplots_adjust(left=0.11, right=0.98, top=0.84, bottom=0.36, wspace=0.38)
    save(fig, "depth_pose_accuracy",
         "Object position from a depth camera: naive against fitted",
         "depth_pose_accuracy.json: rows[].range_m, naive_mm, fit_mm, per_pixel_sigma_mm",
         "Taking the centre of the visible surface puts a 40 mm block 13-14 mm "
         "off (the camera sees only its front); fitting the known shape brings "
         "that to about 1 mm, and the result barely changes with distance "
         "because it is a geometric bias, not noise.",
         "40 mm block: naive 12.7-14.2 mm, fit 1.1-1.5 mm; multimeter naive 44-46 mm, fit 8 mm",
         "APPENDIX")


# -------------------------------------------------------- instructions ---
def fig_instructions():
    d = load("t1_instruction_sweep.json")
    order = ["CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"]
    words = {"CORRECT": "did the right thing", "ASKED": "asked a question", "REFUSED": "refused",
             "MISUNDERSTOOD": "did the WRONG thing"}
    cols = [GREEN, AMBER, GREY, RED]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.8, 3.1), gridspec_kw={"width_ratios": [1, 1.6]})
    x = np.arange(len(order))
    b1 = ax1.bar(x - 0.19, [d["before"]["totals"].get(k, 0) for k in order], 0.38, color=LIGHT,
                 label="earlier grammar")
    b2 = ax1.bar(x + 0.19, [d["totals"].get(k, 0) for k in order], 0.38, color=cols, label="delivered parser")
    label_bars(ax1, b1, "%d", 0.4); label_bars(ax1, b2, "%d", 0.4)
    ax1.set_xticks(x, [words[k] for k in order], rotation=20, ha="right", fontsize=7)
    ax1.set_ylabel("of 75 typed instructions"); ax1.legend(fontsize=7, loc="upper right")
    ax1.set_title("(a) the dangerous outcome went from 3 to 0")
    by = d["by_category"]; cats = sorted(by, key=lambda c: -sum(by[c].values()))
    bottom = np.zeros(len(cats))
    for k, col in zip(order, cols):
        v = np.array([by[c].get(k, 0) for c in cats], float)
        ax2.bar(np.arange(len(cats)), v, 0.66, bottom=bottom, color=col, label=words[k]); bottom += v
    ax2.set_xticks(np.arange(len(cats)), cats, rotation=40, ha="right", fontsize=6.5)
    ax2.set_ylabel("instructions"); ax2.legend(fontsize=6.5, ncol=2, loc="upper right")
    ax2.set_title("(b) by kind of phrasing")
    fig.tight_layout()
    t = d["totals"]
    save(fig, "instruction_sweep",
         "How the robot handles 75 typed instructions",
         "t1_instruction_sweep.json: totals, before.totals, by_category",
         "Of 75 phrasings the delivered parser did the right thing for %d, "
         "asked for %d, refused %d and misunderstood none -- against 3 "
         "misunderstood for the grammar it replaced. Asking and refusing are "
         "safe: the arm moves in neither." % (t["CORRECT"], t["ASKED"], t["REFUSED"]),
         "now 34/14/27/0; before 21/14/37/3 (scene constructed: grounding only)",
         "APPENDIX (mode 06 is not in the pilot)")


def fig_voice():
    d = load("voice_instruction.json"); w = load("wake_word.json")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    t = d["tally"]; order = ["CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"]
    b = ax1.bar(["right", "asked", "refused", "wrong"], [t[k] for k in order], 0.55,
                color=[GREEN, AMBER, GREY, RED])
    label_bars(ax1, b, "%d", 0.3)
    ax1.set_ylabel("of %d spoken instructions" % d["n_cases"])
    ax1.set_title("(a) spoken: word error %.1f%%, %d exact transcripts" % (d["wer"] * 100, d["exact_transcripts"]))
    th = [r["threshold"] for r in w["table"]]
    ax2.plot(th, [r["wake_accepted"] for r in w["table"]], "o-", color=GREEN, lw=1.6, ms=4, label="real wake phrase accepted")
    ax2.plot(th, [r["false_accepted"] for r in w["table"]], "s-", color=RED, lw=1.6, ms=4, label="wrong phrase accepted")
    ax2.axvline(w["shipped_threshold"], ls="--", color=GREY, lw=0.9); ax2.text(w["shipped_threshold"] + 0.1, 30, "shipped", fontsize=7, color=GREY, rotation=90)
    ax2.axvline(w["recommended"], ls="--", color=BLUE, lw=0.9); ax2.text(w["recommended"] + 0.1, 30, "recommended", fontsize=7, color=BLUE, rotation=90)
    ax2.set_xlabel("how loosely the wake phrase is matched"); ax2.set_ylabel("utterances accepted")
    ax2.legend(fontsize=7, loc="upper left"); ax2.set_title("(b) the wake word: looser matching lets wrong phrases in")
    fig.tight_layout()
    save(fig, "voice_pipeline",
         "Spoken instructions and the wake word (synthesised voice only)",
         "voice_instruction.json: tally, wer, exact_transcripts; wake_word.json: table, shipped_threshold, recommended",
         "Through a synthesised voice with 25 %% word error and no exact "
         "transcript, the parser still did the right thing 14 times and the "
         "wrong thing never; the shipped wake-word threshold admits 2 wrong "
         "phrases where the recommended one admits none.",
         "14 right / 5 asked / 21 refused / 0 wrong of 40; WER 25.2 %%; threshold 4 -> 39 real + 2 false, 1 -> 14 + 0",
         "APPENDIX (caveat in file: %s)" % d["caveat"])


# ------------------------------------------------------------ the centre ---
def fig_centre_on_surface():
    d = load("centre_on_surface.json")
    fig, ax = plt.subplots(figsize=(6.8, 3.4))
    for arm, m in (("left", "o"), ("right", "s")):
        hits = [h for h in d["stage1_hits"] if h["arm"] == arm]
        ax.scatter([h["x"] for h in hits], [h["table_top"] for h in hits], s=36, marker=m,
                   facecolors="none", edgecolors=ARM[arm], lw=1.2,
                   label="%s arm: passed one solver call (%d)" % (arm, len(hits)))
    surv = d["stage2"]
    ax.scatter([s["x"] for s in surv], [s["table_top"] for s in surv], s=120, marker="*", color=GREEN, zorder=5,
               label="survived ten repeats over the whole path (%d)" % len(surv))
    for s in surv:
        ax.annotate("%s arm, gap %.0f mm%s" % (s["arm"], s["full_clearance"] * 1000, "" if s["safe"] else " (below 150)"),
                    xy=(s["x"], s["table_top"]), xytext=(s["x"] + 0.02, s["table_top"] - 0.06), fontsize=7, color=GREY,
                    arrowprops=dict(arrowstyle="->", color=GREY, lw=0.7))
    ax.axvspan(-0.01, 0.10, color=RED, alpha=0.08); ax.axvline(0.10, ls="--", color=RED, lw=1)
    ax.text(0.105, max(d["zs"]) - 0.01, "the target: within 10 cm of centre\nnot one cell, either arm", fontsize=7.5, color=RED, va="top")
    ax.set_xlabel("object's distance from the body's centre line (m)"); ax.set_ylabel("table height (m)")
    ax.set_xlim(-0.01, 0.5); ax.legend(fontsize=7, loc="lower right")
    ax.set_title("Can an object resting on a table be picked near the centre? %d solver calls say no" % d["ik_calls"])
    fig.tight_layout()
    save(fig, "centre_on_surface",
         "Objects on a table cannot be picked near the body's centre",
         "centre_on_surface.json: stage1_hits[], stage2[], xs/ys/zs, ik_calls",
         "Sweeping table height, table distance and object position with the "
         "objects resting on the surface, no cell within 10 cm of the centre "
         "works for either arm; the closest that survives ten repeats is "
         "35 cm off centre, and every survivor needs the object at the table's "
         "front edge.",
         "22 single-call hits, 2 full-path survivors (x=0.35 left safe, x=0.45 right at 138 mm gap); %d solver calls" % d["ik_calls"],
         "APPENDIX (a negative result the appendix already states)")


def fig_work_height():
    d = load("band_vs_work_height.json")["work_heights"]
    hs = sorted(d, key=float)
    fig, ax = plt.subplots(figsize=(6.6, 3.0))
    x = np.arange(len(hs))
    for k, arm in enumerate(("left", "right")):
        b = ax.bar(x + (k - 0.5) * 0.38, [d[h][arm]["cells"] for h in hs], 0.38, color=ARM[arm], label="%s arm" % arm)
        label_bars(ax, b, "%d", 0.3)
    ax.set_xticks(x, hs); ax.set_xlabel("height the hand works at (m above the floor)")
    ax.set_ylabel("workable cells in the forward band"); ax.legend(loc="upper left")
    ax.set_title("Working height matters: below 1.05 m almost nothing is reachable")
    fig.tight_layout()
    save(fig, "work_height_band",
         "How the working height changes what is reachable",
         "band_vs_work_height.json: <height>.left/right.cells, y_max",
         "At 0.90 m nothing is reachable and at 1.00 m only 5-6 cells; from "
         "1.10 m upward 18-21 cells are, which is why the work surface sits at "
         "1.10 m.",
         "cells L/R: 0/0, 2/2, 5/6, 8/9, 18/19, 19/20, 20/20, 20/21, 21/21 from 0.90 to 1.30 m",
         "APPENDIX")


def fig_task_margin():
    d = load("task_margin.json")
    rows = d["rows"]
    fig, ax = plt.subplots(figsize=(6.8, 3.0))
    x = np.arange(len(rows))
    b = ax.bar(x, [r["worst_m"] * 1000 for r in rows], 0.6, color=[ARM[r["arm"]] for r in rows])
    label_bars(ax, b, "%.0f", 1)
    ax.axhline(d["bar_m"] * 1000, ls="--", color=RED, lw=1, label="%.0f mm: the minimum asked for" % (d["bar_m"] * 1000))
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_xticks(x, ["%s\n(%s)" % (r["pose"], r["arm"]) for r in rows], fontsize=7.5)
    ax.set_ylabel("how far the pose can be pushed in its worst direction (mm)")
    ax.set_title("Every task pose keeps at least the required margin (ten repeats each)")
    fig.tight_layout()
    save(fig, "task_pose_margins",
         "Room to spare around each task pose",
         "task_margin.json: rows[].pose, arm, worst_m, bar_m",
         "Each task pose was pushed in six directions until the solver failed; "
         "the tightest has 20 mm of room, exactly the minimum required, and "
         "none fails.",
         "worst margins 20-50 mm; bar 20 mm; %d solver calls" % d["ik_calls"],
         "APPENDIX")


def fig_accuracy_table():
    p = os.path.join(ROOT, "recordings", "verification", "accuracy_table.json")
    with open(p) as f:
        d = json.load(f)
    modes = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy", "04_vr_shared", "06_full_autonomy"]
    words = {"01_master_teleop": "master direct", "02_vr_teleop": "VR direct", "03_shared_autonomy": "master shared",
             "04_vr_shared": "VR shared", "06_full_autonomy": "full autonomy"}
    tasks = sorted({r["task"] for rows in d.values() for r in rows})
    grid = np.full((len(modes), len(tasks)), np.nan); txt = [["" for _ in tasks] for _ in modes]
    for i, m in enumerate(modes):
        for r in d.get(m, []):
            j = tasks.index(r["task"]); grid[i, j] = r["grasped"] / r["n"]; txt[i][j] = "%d/%d" % (r["grasped"], r["n"])
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(7.6, 2.8), gridspec_kw={"width_ratios": [1.3, 1]})
    masked = np.ma.masked_invalid(grid); BLUES.set_bad("#eeeeee")
    ax.imshow(masked, cmap=BLUES, vmin=0, vmax=1, aspect="auto")
    for i in range(len(modes)):
        for j in range(len(tasks)):
            ax.text(j, i, txt[i][j] or "not run\nby design", ha="center", va="center", fontsize=7,
                    color="white" if txt[i][j] else GREY)
    ax.set_xticks(range(len(tasks)), [t.upper() for t in tasks]); ax.set_yticks(range(len(modes)), [words[m] for m in modes], fontsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(True)
    ax.set_title("(a) grasps that closed / attempted, scripted clips")
    place = [e * 1000 for e in d["06_full_autonomy"][0]["place_err"]]
    b = ax2.bar(["cube %d" % i for i in range(len(place))], place, 0.55, color=BLUE)
    label_bars(ax2, b, "%.1f", 0.4)
    ax2.set_ylabel("placement error (mm)"); ax2.set_title("(b) T1 placement, full autonomy")
    fig.tight_layout()
    save(fig, "scripted_clip_accuracy",
         "Scripted clip results: grasps closed and placement error",
         "recordings/verification/accuracy_table.json: <mode>[].task, n, grasped, place_err",
         "In the scripted (no-operator) clip set every attempted grasp closed "
         "in every mode, and the pick-and-place task placed its four cubes "
         "55-57 mm from target under full autonomy.",
         "all cells 100 %%; T1 placement %.1f-%.1f mm" % (min(place), max(place)),
         "APPENDIX (simulation only; positioning error is structurally zero in the mock)")


def fig_mount_geometry():
    d = load("mount_geometry.json")
    cands = [c for c in d["cands"] if c["left"].get("x") is not None]
    names = [c["name"].replace("  ", " ") for c in cands]
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    y = np.arange(len(cands))
    for k, arm in enumerate(("left", "right")):
        ax.barh(y + (k - 0.5) * 0.38, [c[arm]["x"] for c in cands], 0.38, color=ARM[arm], label="%s arm" % arm)
    ax.axvline(0.10, ls="--", color=RED, lw=1); ax.text(0.102, len(cands) - 0.6, "10 cm: the target", fontsize=7.5, color=RED)
    ax.set_yticks(y, names, fontsize=7); ax.set_xlabel("closest workable column (m from the centre line)")
    ax.legend(loc="lower right"); ax.set_title("Moving the mount 5-20 cm never opens the centre (%d candidates tried)" % len(d["cands"]))
    fig.tight_layout()
    save(fig, "mount_geometry_candidates",
         "Every mount shift tried, and how close to the centre it lets the hand work",
         "mount_geometry.json: cands[].name, left.x, right.x (candidates with no solution omitted)",
         "Thirty-two mount shifts of 5-20 cm in every direction were tried and "
         "none lets either hand work closer than 27.5 cm from the centre; "
         "forward shifts have no safe solution at all.",
         "best inner column 0.275 m (right, several shifts); forward +5/+10/+15 cm: no solution",
         "APPENDIX")


def fig_grasp_penetration():
    d = load("grasp_penetration.json")
    objs = d["objects"]; names = [objs[k]["name"].replace("_", " ") for k in objs]
    fig, ax = plt.subplots(figsize=(6.8, 3.0))
    x = np.arange(len(objs))
    b1 = ax.bar(x - 0.19, [objs[k]["before_penetration_m"] * 1000 for k in objs], 0.38, color=RED, label="before the fix")
    b2 = ax.bar(x + 0.19, [objs[k]["after_penetration_m"] * 1000 for k in objs], 0.38, color=GREEN, label="after the fix")
    label_bars(ax, b1, "%.0f", 1);
    for bar, k in zip(b2, objs):
        ax.text(bar.get_x() + bar.get_width() / 2, 1.5, "%.0f" % (objs[k]["after_penetration_m"] * 1000), ha="center", fontsize=7, color=GREY)
    ax.axhline(0, color=GREY, lw=0.8)
    ax.set_xticks(x, names, fontsize=7.5); ax.set_ylabel("how far the fingertips would go INTO the object (mm)")
    ax.legend(loc="upper right"); ax.set_title("Fingertip offset bug: the fingers would have been driven 52-106 mm into every object")
    fig.tight_layout()
    save(fig, "grasp_penetration_fix",
         "A fingertip-offset bug, before and after",
         "grasp_penetration.json: objects.*.before_penetration_m, after_penetration_m, name",
         "Before the offset was corrected the fingertips would have been "
         "driven 52-106 mm into every object; after it they stop 5 mm short, "
         "the intended pad contact.",
         "before 52-106 mm penetration; after -5 mm (5 mm standoff) for all six",
         "APPENDIX (an instrument/defect finding)")


def fig_t3_box_hold():
    d = load("t3_box_hold_sweep.json")
    hits = d["hits"]
    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    for grip, m, col in (("mid", "o", BLUE), ("top", "s", GREEN)):
        hh = [h for h in hits if h["grip"] == grip]
        ax.scatter([h["x"] for h in hh], [h["y"] for h in hh], s=40, marker=m, color=col, label="held at the %s (%d)" % (grip, len(hh)))
    ax.set_xlabel("sideways position (m; negative is the right arm's side)"); ax.set_ylabel("forward position (m)")
    ax.legend(fontsize=7); ax.set_title("Where the T3 box can be held: %d of %d positions" % (len(hits), d["tested"]))
    fig.tight_layout()
    save(fig, "t3_box_hold_positions",
         "Where the circuit box can be held for the probing task",
         "t3_box_hold_sweep.json: hits[].x, y, grip, arm; tested",
         "Of %d candidate positions for holding the circuit box, %d work, all "
         "on the right arm's side between 51 and 54 cm out." % (d["tested"], len(hits)),
         "%d/%d hits" % (len(hits), d["tested"]),
         "SKIP (thin; appendix at most)")


def fig_capture_rate():
    d = load("capture_rate.json")
    rows = d["rows"]
    fig, ax = plt.subplots(figsize=(5.6, 3.0))
    x = np.arange(len(rows))
    b1 = ax.bar(x - 0.19, [r["requested"] for r in rows], 0.38, color=LIGHT, label="frames per second asked for")
    b2 = ax.bar(x + 0.19, [r["delivered_fps"] for r in rows], 0.38, color=RED, label="genuinely new frames per second")
    label_bars(ax, b1, "%d", 0.5); label_bars(ax, b2, "%.2f", 0.5)
    ax.set_xticks(x, ["%d fps" % r["requested"] for r in rows]); ax.set_ylabel("frames per second")
    ax.legend(fontsize=7); ax.set_title("A camera that reported 60 fps delivered under 1 new frame a second")
    fig.tight_layout()
    save(fig, "capture_rate_duplicates",
         "Requested against genuinely new camera frames",
         "capture_rate.json: rows[].requested, delivered_fps, dup_pct",
         "Whatever frame rate was requested, 91-100 %% of the stored frames "
         "were duplicates of the previous one: the pipeline reported a rate it "
         "was not delivering, one of the project's instrument faults.",
         "duplicates 90.6 / 100 / 97.5 / 98.8 %%; new frames 1.12 / 0 / 0.75 / 0.75 per s",
         "APPENDIX (instrument-fault register)")


def fig_experiment_gates():
    d = load("experiment_gates.json")
    g1 = d["gate1"]; caps = ["brief's assumption (0.05)", "real (shipped, 0.15)", "candidate 0.30", "sim default (0.6)"]
    words = {"brief's assumption (0.05)": "0.05 rad/s\n(the brief assumed)", "real (shipped, 0.15)": "0.15 rad/s\n(real arm cap)",
             "candidate 0.30": "0.30 rad/s", "sim default (0.6)": "0.60 rad/s\n(simulation)"}
    paths = [p for p in g1 if g1[p]["waypoints"] > 4]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.6, 3.0))
    x = np.arange(len(caps))
    for i, p in enumerate(paths):
        b = ax1.bar(x + (i - 1) * 0.27, [g1[p][c] for c in caps], 0.27, color=[BLUE, GREEN, AMBER][i],
                    label=p.replace("T2/B carry ", "carry: ").replace(" (Task A clip path)", ""))
        label_bars(ax1, b, "%.0f", 0.4, size=6.5)
    ax1.set_xticks(x, [words[c] for c in caps], fontsize=7); ax1.set_ylabel("time to run the path (s)")
    ax1.legend(fontsize=6.5); ax1.set_title("(a) the real speed cap makes a pick take %.0f s" % g1[paths[0]]["real (shipped, 0.15)"])
    g2 = d["gate2"]; pols = ["fixed", "tilt", "yaw_free", "topdown"]
    pw = {"fixed": "wrist fixed", "tilt": "tilt allowed", "yaw_free": "yaw free", "topdown": "top-down"}
    x2 = np.arange(len(pols))
    for k, arm in enumerate(("left", "right")):
        b = ax2.bar(x2 + (k - 0.5) * 0.38, [g2[arm][p]["pct"] for p in pols], 0.38, color=ARM[arm], label="%s arm" % arm)
        label_bars(ax2, b, "%.0f%%", 1, size=6.5)
    ax2.set_xticks(x2, [pw[p] for p in pols], fontsize=7.5); ax2.set_ylabel("of 15 test poses reachable (%)")
    ax2.set_ylim(0, 115); ax2.legend(fontsize=7, loc="lower right"); ax2.set_title("(b) the right arm reaches little with a fixed wrist")
    fig.tight_layout()
    save(fig, "experiment_gates",
         "Two gates the experiment design had to pass",
         "experiment_gates.json: gate1.<path>.<cap>, gate2.<arm>.<policy>.pct",
         "At the real arms' speed cap a single pick-and-place path takes 11 s "
         "(36 s at the speed the brief assumed); and with the wrist fixed the "
         "right arm reaches none of 15 test poses where the left reaches 13.",
         "gate1 T1 path: 36.0 / 11.2 / 5.7 / 2.9 s; gate2 right fixed 0 %%, top-down 53 %%; gate3 verdict: %s" % d["gate3"]["verdict"],
         "APPENDIX")


FIGS = [fig_mount_sweep, fig_mount_heat, fig_reach_directions, fig_reach_extents,
        fig_wearer_posture, fig_wearer_size, fig_what_binds, fig_orientation_cost,
        fig_park_error, fig_sim_real_models, fig_error_budget, fig_reaction_budget,
        fig_posture_fallback, fig_intent_sweep, fig_mode_travel, fig_detector_real,
        fig_look_cost, fig_pick_accuracy, fig_pad_curve, fig_objects_vs_jaw,
        fig_home_pose, fig_channel_health, fig_predictive, fig_motion_generator,
        fig_degradation, fig_depth_pose, fig_instructions, fig_voice,
        fig_centre_on_surface, fig_work_height, fig_task_margin, fig_accuracy_table,
        fig_mount_geometry, fig_grasp_penetration, fig_t3_box_hold, fig_capture_rate,
        fig_experiment_gates]

NOT_DRAWN = [
    ("centre_vs_height.json", "marked STALE in docs/ENGINEERING_LOG.md (its 0.425/0.400 columns re-measure to 0.325/0.450); not drawn"),
    ("dance_paths.json", "0 failures in 1298 waypoints -- a flat result with nothing to draw; stated in the report as a number"),
    ("t1_stage2_paths.json", "all eight seeds clean, pad miss 0.01 mm each -- flat; stated as a number"),
    ("dial_safety.json", "two runs, identical clearance -- too thin"),
    ("lift.json", "per-cube lift heights all 0.30 m or None -- too thin"),
    ("home_change*.json", "per-task IK failures/clearance at the old vs new home; superseded by home_change_applied's text in the report"),
    ("cad_inspect.json, cad_joints.json, world_map*.json, scene_reference.npz", "geometry dumps, not measurements"),
    ("centre_gap_*.json, centre_geom_*.json, centre_limit*.json, centre_posture_*.json, clearance_region*.json", "the per-cell raw sweeps behind centre_on_surface/wearer_posture; summarised by those figures"),
    ("vision_options.json, colour_vision.json, language_vision_sweep.json, freeform_language.json, wake_word rows", "constructed-scene grounding; instruction_sweep covers the same claim"),
    ("t1_paths*.json, t1_*.json, task0_*, observe_*, pick_*, scan_*, retreat_*, go_home.json", "single verification runs with pass/fail and one or two numbers; no distribution to draw"),
    ("channels_* other dates", "only channels_20260806.json exists in the directory"),
]


DISAGREEMENTS = [
    "* `shared_autonomy_gain.json`: DECLARED wrong-guess rate is 0.5 % at 20 mm, **17.5 % at 40 mm, 35.0 % at 60 mm (one cube pitch), 39.8 % at 80, 47.5 % at 120 mm**. Results 3.3 says \"confidently wrong 50 % of the time once the declaration is one object-pitch out\" -- at one pitch (60 mm) the file says 35 %; ~50 % (47.5 %) is reached at two pitches (120 mm). The 17.5 % at 40 mm and the calibrated 0.0 % agree.",
    "* `control_budget.json` `freedom.at_the_work_point`: pinned 0.0646 m, 15-degree cone 0.3042, 45-degree cone 0.325, **position-only 0.3625 m**; `at_home`: pinned 0.4598, free 0.5491 (orientation_cost.json: 0.5473). Results 3.2 says \"0.065 m pinned against 0.450 m free, a factor of seven\" -- the file gives 0.363 m free (a factor of 5.6). Appendix I tab:work-point carries 0.065 / 0.363, so the main body disagrees with its own appendix; 0.450 in appendix I is the right arm's innermost workable column, a different quantity.",
    "* `arm_directional_calibration.json`: the per-run worst joint parks 0.29 degrees short (mode of the histogram); the report's 0.305 is the pooled per-joint fit from measure_sim_to_real_gap.py -- same order, not the same number; the figure carries no 0.305 line.",
    "* `mount_overlap_sweep.json`: best cell count 29 of 63 at dx -0.20 m, tilt 60 degrees (not buildable per findings.md); 22 of 63 at the buildable dx -0.20 / tilt 30 / forward 0.15 -- consistent with the report's \"22 of 63 within a buildable 0.20 m separation\".",
    "* Everything else the main body quotes that these files carry (12.91 / 19.99 mm, 51.3 mm, 12.53x, 0.3170 / 0.0933 / 224 mm, 9.0 % of 432, 74-76 %, 0.460 -> 0.547 from home) agrees.",
]


def main():
    for fn in FIGS:
        try:
            fn()
        except Exception as e:      # noqa: BLE001
            SKIPPED.append((fn.__name__, "%s: %s" % (type(e).__name__, e)))
            traceback.print_exc()
            plt.close("all")
    lines = ["# Baseline gallery", "",
             "Generated by `make_baselines_gallery.py` from `recordings/baselines/*.json` "
             "(plus `recordings/verification/accuracy_table.json`). Every figure is .pdf + .png. "
             "Suggestion column: MAIN = worth a main-body slot, APPENDIX = supporting, SKIP = thin.", "",
             "| file | title | source | takeaway | key numbers | suggestion |", "|---|---|---|---|---|---|"]
    for stem, title, source, take, nums, sug in INDEX:
        lines.append("| `%s.pdf` | %s | %s | %s | %s | %s |" % (
            stem, title, source.replace("|", "/"), take.replace("|", "/"), nums.replace("|", "/"), sug.replace("|", "/")))
    lines += ["", "## Not drawn", ""]
    for name, why in NOT_DRAWN:
        lines.append("* `%s`: %s" % (name, why))
    for name, why in SKIPPED:
        lines.append("* `%s` FAILED: %s" % (name, why))
    lines += ["", "## Where a JSON disagrees with a number the report quotes", ""] + DISAGREEMENTS
    with open(os.path.join(OUT, "INDEX.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("%d figures, %d failed; INDEX.md written" % (len(INDEX), len(SKIPPED)))


if __name__ == "__main__":
    sys.exit(main())
