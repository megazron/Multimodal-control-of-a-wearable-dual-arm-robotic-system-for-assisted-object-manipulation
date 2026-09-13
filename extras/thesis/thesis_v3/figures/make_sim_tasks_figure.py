#!/usr/bin/env python3
"""The four scripted tasks in simulation, one row each, four moments per
row, plus the tray's tilt over time in every mode.

Stills come from the recorded RViz front-camera clips of each verification
cell (recordings/verification/<mode>/<task>/*/rviz_front.mp4) at the logged
event times, through the gallery's own cached frame extractor; the tilt
series from each T2 cell's scene_events.json.

    python3 extras/thesis/thesis_v3/figures/make_sim_tasks_figure.py
"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "gallery", "verification"))
import make_verification_gallery as G  # noqa: E402

OUT = os.path.join(HERE, "sim", "sim_tasks_collage.pdf")
W = 6.76
BLUE, RED, GREEN, AMBER, GREY = "#2c6fbb", "#c1392b", "#3f9142", "#c98a1a", "#595959"
MODE_TINY = {"01_master_teleop": "mannequin direct", "02_vr_teleop": "VR direct",
             "03_shared_autonomy": "mannequin + assist", "04_vr_shared": "VR + assist",
             "06_full_autonomy": "full autonomy"}
ROWS = [("T0", "01_master_teleop", "reach the target\n(three targets per arm)"),
        ("T2", "01_master_teleop", "carry the tray\n(both arms, ball on top)"),
        ("T3", "01_master_teleop", "circuit box +\nmultimeter probe"),
        ("T1", "06_full_autonomy", "pick and place\n(four cubes, two arms)")]


def moments(mode, task):
    off, span, evs, e = G.clip_times(mode, task)
    gr = [t for t, x in evs if x["ev"] == "GRASPED"]; rl = [t for t, x in evs if x["ev"] == "RELEASED"]
    if gr and task == "T1":
        return [(off + 0.5, "start"), (gr[0] + 0.3, "first grasp"), (gr[-1] + 0.3, "last grasp"), (off + span - 0.5, "end")]
    if gr:
        return [(off + 0.5, "start"), (gr[0] + 0.3, "grasp"), ((gr[-1] + rl[0]) / 2 if rl else gr[-1] + 1, "carry"),
                ((rl[-1] + 0.5) if rl else off + span - 0.5, "end")]
    return [(off + 0.5, "start"), (off + span * .33, "one third"), (off + span * .66, "two thirds"), (off + span - 0.5, "end")]


def main():
    fig = plt.figure(figsize=(W, 8.0))
    gs = fig.add_gridspec(5, 4, height_ratios=[1, 1, 1, 1, 1.3], hspace=0.22, wspace=0.03,
                          left=0.09, right=0.99, top=0.975, bottom=0.06)
    for r, (task, mode, name) in enumerate(ROWS):
        for c, (t, lab) in enumerate(moments(mode, task)):
            ax = fig.add_subplot(gs[r, c])
            ax.imshow(Image.open(G.frame(mode, task, "front", t, "simtasks_%s_%d" % (task, c)))); ax.axis("off")
            if c == 0:
                ax.set_title("(%s) %s" % ("abcd"[r], name.replace("\n", " ")), loc="left", fontsize=8.5, pad=3)
            ax.text(0.03, 0.95, lab, transform=ax.transAxes, ha="left", va="top", fontsize=7.5, color="white",
                    bbox=dict(boxstyle="round,pad=0.2", fc="black", ec="none", alpha=0.55))
    # (e) the tray's tilt over time, every mode
    ax = fig.add_subplot(gs[4, :])
    for mode, c in zip(G.MODES, [BLUE, RED, GREEN, AMBER, GREY]):
        d = G.cell_dir(mode, "T2")
        if not d: continue
        e = G.load(os.path.join(d, "scene_events.json")); cs = e.get("carry_series") or []
        ax.plot([r["t"] for r in cs], [r["tilt_deg"] for r in cs], color=c, lw=1.1, label=MODE_TINY[mode])
    ax.axhline(6.8, color=RED, ls="--", lw=0.9); ax.axhline(-6.8, color=RED, ls="--", lw=0.9)
    ax.text(1, 7.4, "6.8 deg: the ball rolls off", fontsize=7.5, color=RED)
    ax.set_xlabel("time in the run (s)", labelpad=1); ax.set_ylabel("tray tilt (deg)"); ax.set_ylim(-9, 12.5)
    ax.legend(fontsize=7.5, ncol=5, loc="upper center", frameon=False, bbox_to_anchor=(0.5, 0.99), handlelength=1.4, columnspacing=1.0)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.set_title("(e) the tray's tilt through the carry, every mode", loc="left", fontsize=8.5, pad=3)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, bbox_inches="tight"); plt.close(fig); print("wrote", OUT)


if __name__ == "__main__":
    main()
