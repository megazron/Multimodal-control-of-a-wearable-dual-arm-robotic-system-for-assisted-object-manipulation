#!/usr/bin/env python3
"""Two result collages drawn natively at the report's text width (6.76 in),
so every label prints at 7 pt or larger: the NASA-TLX questionnaire and the
direct-versus-shared comparison. Data: figures/pilot/data/tlx_trials_long.csv
and figures/pilot/data/study_summary.json.

    python3 thesis_v3/figures/make_results_collages.py
"""
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "pilot")
W = 6.76
plt.rcParams.update({"font.size": 7.5, "axes.titlesize": 8, "axes.labelsize": 7.5, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "legend.fontsize": 7, "figure.dpi": 150, "savefig.bbox": "tight",
                     "axes.spines.top": False, "axes.spines.right": False})
BLUE, RED, PURPLE, GREEN, AMBER, GREY = "#2c6fbb", "#c1392b", "#8e44ad", "#3f9142", "#e0a800", "0.35"


def panel_letter(ax, s):
    ax.text(0.0, 1.04, s, transform=ax.transAxes, fontsize=9, fontweight="bold", va="bottom", ha="left")


# --------------------------------------------------------------------------
# 1. NASA-TLX
# --------------------------------------------------------------------------
def tlx():
    rows = list(csv.DictReader(open(os.path.join(HERE, "pilot", "data", "tlx_trials_long.csv"))))
    COND = [("VR", "VR direct"), ("MM", "mannequin\ndirect"), ("SA", "VR + shared\nautonomy")]
    TASK = [("PP", "pick and place"), ("TR", "target reaching")]
    SUB = ["mental", "physical", "temporal", "performance", "effort", "frustration"]
    COL = {"VR": BLUE, "MM": PURPLE, "SA": RED}
    PARTS = sorted({r["participant"] for r in rows})
    OUTC = {"Success_picked_and_placed": ("placed", GREEN), "Success_reached": ("reached", GREEN),
            "Partial_picked_not_placed": ("picked\nonly", AMBER), "Fail_no_pick_reach_only": ("reach\nonly", RED)}
    sel = lambda c=None, t=None: [r for r in rows if (c is None or r["condition_code"] == c) and (t is None or r["task_code"] == t)]
    fig = plt.figure(figsize=(W, 7.4))
    gs = fig.add_gridspec(3, 5, height_ratios=[1.0, 1.0, 0.95], hspace=0.6, wspace=0.9)
    # (a) RTLX per condition, each task
    for k, (tc, tl) in enumerate(TASK):
        ax = fig.add_subplot(gs[0, 0:3] if k == 0 else gs[0, 3:5])
        for i, (cc, cl) in enumerate(COND):
            v = [float(r["rtlx"]) for r in sel(cc, tc)]
            ax.bar(i, np.mean(v), 0.6, color=COL[cc], alpha=0.35)
            ax.errorbar(i, np.mean(v), yerr=np.std(v, ddof=1), color=COL[cc], capsize=3, lw=1)
            xs = np.linspace(-0.16, 0.16, len(v))
            ax.scatter(i + xs, v, s=16, color=COL[cc], zorder=3)
            for r, x in zip(sel(cc, tc), xs):
                ax.text(i + x + 0.04, float(r["rtlx"]), r["participant"], fontsize=6.5, ha="left", va="center", color=GREY)
        ax.set_xticks(range(3)); ax.set_xticklabels([c[1] for c in COND]); ax.set_ylim(0, 100); ax.set_title(tl)
        if k == 0:
            ax.set_ylabel("raw NASA-TLX workload (0-100)"); panel_letter(ax, "(a)")
    # (b) subscales
    ax = fig.add_subplot(gs[1, :])
    x = np.arange(len(SUB)); w = 0.26
    for i, (cc, cl) in enumerate(COND):
        means = [np.mean([float(r[s]) for r in sel(cc)]) for s in SUB]
        sds = [np.std([float(r[s]) for r in sel(cc)], ddof=1) for s in SUB]
        ax.bar(x + (i - 1) * w, means, w, yerr=sds, color=COL[cc], alpha=0.85, capsize=2, error_kw={"lw": 0.8}, label=cl.replace("\n", " "))
    ax.set_xticks(x); ax.set_xticklabels(SUB); ax.set_ylabel("rating (0-100)"); ax.set_ylim(0, 100)
    ax.legend(frameon=False, ncol=3, loc="upper right"); panel_letter(ax, "(b)")
    # (c) outcomes
    ax = fig.add_subplot(gs[2, 0:3])
    cols = [(tc, cc) for tc, _ in TASK for cc, _ in COND]
    for j, (tc, cc) in enumerate(cols):
        for i, p in enumerate(PARTS):
            r = [r for r in rows if r["participant"] == p and r["task_code"] == tc and r["condition_code"] == cc][0]
            lab, col = OUTC[r["task_outcome"]]
            ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor=col, edgecolor="white", lw=1.5))
            ax.text(j + 0.5, i + 0.5, lab, ha="center", va="center", fontsize=7, color="white")
    ax.set_xlim(0, len(cols)); ax.set_ylim(len(PARTS), 0)
    ax.set_xticks(np.arange(len(cols)) + 0.5)
    ax.set_xticklabels(["%s\n%s" % ("pick+place" if tc == "PP" else "reach", {"VR": "VR", "MM": "mannequin", "SA": "shared"}[cc]) for tc, cc in cols], fontsize=6.5)
    exp = {p: [r for r in rows if r["participant"] == p][0] for p in PARTS}
    ax.set_yticks(np.arange(len(PARTS)) + 0.5)
    ax.set_yticklabels(["%s (VR %s, robot %s)" % (p, exp[p]["vr_experience"].lower(), exp[p]["robot_experience"].lower()) for p in PARTS], fontsize=7)
    for s in ax.spines.values(): s.set_visible(False)
    ax.tick_params(length=0); panel_letter(ax, "(c)")
    # (d) workload against experience
    ax = fig.add_subplot(gs[2, 3:5])
    lv = {1: "novice", 2: "intermediate", 3: "experienced"}
    for cc, cl in COND:
        xs, ys = [], []
        for p in PARTS:
            rr = [r for r in rows if r["participant"] == p and r["condition_code"] == cc]
            xs.append(int(rr[0]["robot_exp_level"])); ys.append(np.mean([float(r["rtlx"]) for r in rr]))
        ax.scatter(np.array(xs) + {"VR": -0.08, "MM": 0, "SA": 0.08}[cc], ys, color=COL[cc], s=24, label=cl.replace("\n", " "))
    ax.set_xticks([1, 2, 3]); ax.set_xticklabels([lv[1], lv[2], lv[3]]); ax.set_ylim(0, 100)
    ax.set_xlabel("prior robot experience"); ax.set_ylabel("mean RTLX per operator"); ax.legend(frameon=False, loc="lower left")
    panel_letter(ax, "(d)")
    fig.savefig(os.path.join(OUT, "tlx_collage.pdf")); plt.close(fig)


# --------------------------------------------------------------------------
# 2. direct control against shared autonomy
# --------------------------------------------------------------------------
def dvs():
    D = json.load(open(os.path.join(HERE, "pilot", "data", "study_summary.json")))
    S = D["sessions"]; P = D["paired"]
    TASKS = ["Pick and place", "Target reaching", "Unspecified"]
    TL = {"Pick and place": "pick &\nplace", "Target reaching": "target\nreaching", "Unspecified": "P3*"}
    fig = plt.figure(figsize=(W, 5.4))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.15], hspace=0.5, wspace=0.42)
    metrics = [("duration_s", "time to finish (s)", 1.0), ("ee_path_m", "robot's hand path (m)", 1.0), ("clutch_engagements", "re-grips", 1.0)]
    for k, (m, lab, sc) in enumerate(metrics):
        ax = fig.add_subplot(gs[0, k])
        pos = 0; ticks = []
        for t in TASKS:
            for cond, col, off in (("Direct", BLUE, -0.2), ("Shared", RED, 0.2)):
                v = [s[m] * sc for s in S if s["task"] == t and s["condition"] == cond and s.get(m) is not None]
                bp = ax.boxplot([v], positions=[pos + off], widths=0.34, patch_artist=True, showfliers=True,
                                medianprops={"color": "black", "lw": 1}, flierprops={"marker": ".", "markersize": 3})
                for b in bp["boxes"]: b.set(facecolor=col, alpha=0.45, edgecolor=col)
                for w_ in bp["whiskers"] + bp["caps"]: w_.set(color=col, lw=0.9)
            ticks.append(pos); pos += 1
        ax.set_xticks(ticks); ax.set_xticklabels([TL[t] for t in TASKS]); ax.set_ylabel(lab)
        ax.set_title(["(a)", "(b)", "(c)"][k], loc="left", fontsize=9, fontweight="bold")
        if k == 0:
            ax.legend(handles=[Patch(facecolor=BLUE, alpha=0.45, label="direct"), Patch(facecolor=RED, alpha=0.45, label="shared autonomy")],
                      frameon=False, loc="upper right")
    slopes = [("ee_path_m", "how far the robot's hand moved (m)"), ("controller_path_m", "how far the operator's hand moved (m)"), ("clutch_engagements", "times the clutch was re-engaged")]
    for k, (m, lab) in enumerate(slopes):
        ax = fig.add_subplot(gs[1, k])
        a_all = [p[m]["direct"] for p in P]; b_all = [p[m]["shared"] for p in P]
        for p, a, b in zip(P, a_all, b_all):
            ax.plot([0, 1], [a, b], color=GREY, lw=1.0); ax.scatter([0], [a], color=BLUE, s=26, zorder=3); ax.scatter([1], [b], color=RED, s=26, zorder=3)
            ax.text(-0.08, a, p["participant"], fontsize=6.5, ha="right", va="center", color=GREY)
        up = sum(1 for a, b in zip(a_all, b_all) if b > a)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["direct", "with\nassistance"]); ax.set_xlim(-0.4, 1.25); ax.set_ylabel(lab)
        ax.set_title("%s  %d of %d higher with assistance" % (["(d)", "(e)", "(f)"][k], up, len(P)), loc="left", fontsize=7.5, color=GREY)
    fig.savefig(os.path.join(OUT, "dvs_collage.pdf")); plt.close(fig)


if __name__ == "__main__":
    tlx(); dvs()
    print("wrote", os.path.join(OUT, "tlx_collage.pdf"), "and dvs_collage.pdf")
