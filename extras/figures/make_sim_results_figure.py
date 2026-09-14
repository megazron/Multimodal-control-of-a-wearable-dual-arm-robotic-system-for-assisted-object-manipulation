#!/usr/bin/env python3
"""The simulation results in one figure, drawn natively at text width:
(a) grasp and set-down error against the 30 mm capture gate, every scripted
run; (b) run duration by mode; (c) the intent estimator from a typed-in map
against a measured one; (d) how full autonomy handled 75 typed instructions.

Data: recordings/verification/accuracy_table.json, the clip_meta.json of
each verification cell, recordings/baselines/shared_autonomy_gain.json and
recordings/baselines/t1_instruction_sweep.json.

    python3 extras/figures/make_sim_results_figure.py
"""
import glob, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 2))
V = os.path.join(ROOT, "recordings", "verification")
B = os.path.join(ROOT, "recordings", "baselines")
OUT = os.path.join(HERE, "sim", "sim_results_collage.pdf")
W = 6.76
plt.rcParams.update({"font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "xtick.labelsize": 8,
                     "ytick.labelsize": 8, "legend.fontsize": 8, "savefig.bbox": "tight",
                     "axes.spines.top": False, "axes.spines.right": False})
BLUE, RED, GREEN, AMBER, GREY, PURPLE = "#2c6fbb", "#c1392b", "#3f9142", "#c98a1a", "#595959", "#8e44ad"
MODES = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy", "04_vr_shared", "06_full_autonomy"]
MODE_TINY = {"01_master_teleop": "mannequin direct", "02_vr_teleop": "VR direct",
             "03_shared_autonomy": "mannequin + assist", "04_vr_shared": "VR + assist",
             "06_full_autonomy": "full autonomy"}
MODE_COL = {"01_master_teleop": BLUE, "02_vr_teleop": RED, "03_shared_autonomy": GREEN,
            "04_vr_shared": AMBER, "06_full_autonomy": GREY}
TASK_TINY = {"T3": "circuit box", "T1": "pick and place", "T1S2": "pick and place, random"}
GATE = 30.0


def load(p):
    return json.load(open(p))


def cell_dir(mode, task):
    hits = [h for h in glob.glob(os.path.join(V, mode, task, "*")) if os.path.isdir(h)]
    return hits[0] if hits else None


def panel_letter(ax, s):
    ax.set_title(s, loc="left", fontsize=9, fontweight="bold")


def main():
    acc = load(os.path.join(V, "accuracy_table.json"))
    fig = plt.figure(figsize=(W, 5.6))
    gs = fig.add_gridspec(2, 2, hspace=0.55, wspace=0.4)

    # (a) set-down error across the four recorded versions of the pick-and-place
    ax = fig.add_subplot(gs[0, 0])
    sets = [("first\nlayout", "extras/archive/recordings/superseded_20260813_prespec/verification/accuracy_table.json"),
            ("rework", "extras/archive/recordings/superseded_20260813_prespec/verification_20260811_t1rework/accuracy_table.json"),
            ("before the\naccuracy pass", "extras/archive/recordings/verification_20260823_pre_accuracy_pass/accuracy_table.json"),
            ("delivered", "recordings/verification/accuracy_table.json")]
    for k, (lab, path) in enumerate(sets):
        dd = load(os.path.join(ROOT, path))
        vals = [1000 * v for r in dd.get("06_full_autonomy", []) if r["task"] == "t1" for v in r["place_err"]]
        ax.scatter([k] * len(vals), vals, color=BLUE, s=24, zorder=3, label="one cube" if k == 0 else None)
        m = sum(vals) / len(vals)
        ax.plot([k - .28, k + .28], [m, m], color=RED, lw=2, label="mean" if k == 0 else None)
        ax.text(k + 0.3, m, "%.0f" % m, ha="left", va="center", fontsize=7.5, color=RED)
    ax.axhline(GATE, color=GREY, ls="--", lw=1); ax.text(-0.4, GATE + 14, "30 mm capture gate", ha="left", fontsize=7, color=GREY)
    ax.set_xticks(range(len(sets))); ax.set_xticklabels([sname for sname, _ in sets], fontsize=7.5)
    ax.set_ylabel("cube set-down error (mm)"); ax.set_ylim(0, 680); ax.legend(frameon=False, loc="upper left", fontsize=7)
    panel_letter(ax, "(a) set-down error by version")

    # (b) run duration by mode
    ax = fig.add_subplot(gs[0, 1]); tasks = ["T0", "T2", "T3"]
    TN = {"T0": "reach the\ntarget", "T2": "carry the\ntray", "T3": "circuit box +\nmultimeter"}
    for j, task in enumerate(tasks):
        for i, mode in enumerate(MODES):
            d = cell_dir(mode, task)
            if not d: continue
            m = load(os.path.join(d, "clip_meta.json")); span = m["grab_t1"] - m["grab_t0"]
            ax.scatter(j + (i - 2) * 0.14, span, color=MODE_COL[mode], s=30, zorder=3,
                       label=MODE_TINY[mode] if j == 0 else None)
    ax.set_xticks(range(len(tasks))); ax.set_xticklabels([TN[t] for t in tasks]); ax.set_xlim(-0.6, len(tasks) - 0.4)
    ax.set_ylabel("time to run the task (s)"); ax.set_ylim(0, 80)
    ax.legend(frameon=False, ncol=2, loc="upper left", fontsize=7, handletextpad=0.2, columnspacing=0.8)
    panel_letter(ax, "(b) run duration by mode")

    # (c) intent estimator sweep
    d = load(os.path.join(B, "shared_autonomy_gain.json"))
    errs = sorted(d["rows"], key=float); x = [float(e) for e in errs]
    ax = fig.add_subplot(gs[1, 0])
    for src, col, name in (("DECLARED", RED, "object position typed in"), ("CALIBRATED", BLUE, "object position measured by camera")):
        ax.plot(x, [d["rows"][e][src]["wrong_pct"] for e in errs], "o-", color=col, lw=1.6, ms=4, label=name)
    ax.axhline(16, ls="--", color=AMBER, lw=1); ax.text(124, 18, "16 %: above this, assistance is a net loss", fontsize=7, color=AMBER, ha="right")
    ax.axvline(d["cube_pitch_mm"], ls=":", color=GREY, lw=0.9); ax.text(d["cube_pitch_mm"] + 1.5, 4, "one cube\nspacing", fontsize=7, color=GREY)
    ax.set_xlabel("error in the typed-in object position (mm)"); ax.set_ylabel("confidently wrong guesses (%)")
    ax.set_ylim(0, 55); ax.legend(frameon=False, loc="upper left", fontsize=7)
    panel_letter(ax, "(c) shared autonomy: guessing the target")

    # (d) instruction handling
    d = load(os.path.join(B, "t1_instruction_sweep.json"))
    order = ["CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"]
    words = {"CORRECT": "did the\nright thing", "ASKED": "asked a\nquestion", "REFUSED": "refused", "MISUNDERSTOOD": "did the\nwrong thing"}
    cols = [GREEN, AMBER, GREY, RED]
    ax = fig.add_subplot(gs[1, 1]); xx = np.arange(4)
    b1 = ax.bar(xx - 0.19, [d["before"]["totals"].get(k, 0) for k in order], 0.38, color="0.8", label="earlier grammar")
    b2 = ax.bar(xx + 0.19, [d["totals"].get(k, 0) for k in order], 0.38, color=cols, label="delivered parser")
    for bars in (b1, b2):
        for b in bars:
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 0.6, "%d" % b.get_height(), ha="center", fontsize=7)
    ax.set_xticks(xx); ax.set_xticklabels([words[k] for k in order]); ax.set_ylabel("of 75 typed instructions"); ax.set_ylim(0, 52)
    ax.legend(frameon=False, loc="upper left", fontsize=7)
    panel_letter(ax, "(d) full autonomy: 75 typed instructions")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT); plt.close(fig); print("wrote", OUT)


if __name__ == "__main__":
    main()
