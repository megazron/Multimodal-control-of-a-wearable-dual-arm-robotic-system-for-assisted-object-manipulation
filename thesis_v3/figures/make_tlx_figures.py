#!/usr/bin/env python3
"""NASA-TLX questionnaire results of the pilot: five operators, three
conditions (VR controllers, mannequin master, shared autonomy), two tasks.
Raw workload (RTLX) is the unweighted mean of the six subscales, 0-100.

Reads figures/pilot/data/tlx_trials_long.csv (one row per operator x
condition x task, with the task outcome) and writes:
  figures/pilot/tlx_rtlx_by_condition.pdf   RTLX per condition and task, every operator shown
  figures/pilot/tlx_subscales.pdf           the six subscales per condition
  figures/pilot/tlx_outcomes.pdf            what each operator achieved, per task and condition
  figures/pilot/tlx_experience.pdf          workload against prior experience
  figures/pilot/tlx_table.tex               the means table for the report
  figures/pilot/data/tlx_all.csv            the tidy data, one file
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "pilot", "data", "tlx_trials_long.csv")
OUT = os.path.join(HERE, "pilot")
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8, "xtick.labelsize": 7.5,
                     "ytick.labelsize": 7.5, "legend.fontsize": 7, "figure.dpi": 150, "savefig.bbox": "tight",
                     "axes.spines.top": False, "axes.spines.right": False})
rows = list(csv.DictReader(open(D)))
COND = [("VR", "VR controllers,\ndirect"), ("MM", "mannequin master,\ndirect"), ("SA", "VR controllers,\nshared autonomy")]
TASK = [("PP", "pick and place"), ("TR", "target reaching")]
SUB = [("mental", "mental"), ("physical", "physical"), ("temporal", "temporal"), ("performance", "performance"),
       ("effort", "effort"), ("frustration", "frustration")]
COL = {"VR": "#2c6fbb", "MM": "#8e44ad", "SA": "#c1392b"}
PARTS = sorted({r["participant"] for r in rows})
OUTCOME = {"Success_picked_and_placed": ("picked and placed", "#3f9142"), "Success_reached": ("reached", "#3f9142"),
           "Partial_picked_not_placed": ("picked, not placed", "#e0a800"), "Fail_no_pick_reach_only": ("reached only", "#c1392b")}

def sel(cond=None, task=None):
    return [r for r in rows if (cond is None or r["condition_code"] == cond) and (task is None or r["task_code"] == task)]

# 1. RTLX by condition and task, every operator as a point
fig, axes = plt.subplots(1, 2, figsize=(6.4, 2.6), sharey=True)
for ax, (tc, tl) in zip(axes, TASK):
    for i, (cc, cl) in enumerate(COND):
        v = [float(r["rtlx"]) for r in sel(cc, tc)]
        ax.bar(i, np.mean(v), 0.6, color=COL[cc], alpha=0.35)
        ax.errorbar(i, np.mean(v), yerr=np.std(v, ddof=1), color=COL[cc], capsize=3, lw=1)
        ax.scatter(np.full(len(v), i) + np.linspace(-0.15, 0.15, len(v)), v, s=14, color=COL[cc], zorder=3)
        for r, x in zip(sel(cc, tc), np.linspace(-0.15, 0.15, len(v))):
            ax.text(i + x, float(r["rtlx"]) + 1.5, r["participant"], fontsize=5, ha="center", color="0.3")
    ax.set_xticks(range(3)); ax.set_xticklabels([c[1] for c in COND]); ax.set_title(tl); ax.set_ylim(0, 100)
axes[0].set_ylabel("raw NASA-TLX workload (0-100)")
fig.savefig(os.path.join(OUT, "tlx_rtlx_by_condition.pdf")); plt.close(fig)

# 2. the six subscales per condition, both tasks pooled
fig, ax = plt.subplots(figsize=(6.4, 2.6))
x = np.arange(len(SUB)); w = 0.26
for i, (cc, cl) in enumerate(COND):
    means = [np.mean([float(r[s]) for r in sel(cc)]) for s, _ in SUB]
    sds = [np.std([float(r[s]) for r in sel(cc)], ddof=1) for s, _ in SUB]
    ax.bar(x + (i - 1) * w, means, w, yerr=sds, color=COL[cc], alpha=0.85, capsize=2, error_kw={"lw": 0.8}, label=cl.replace("\n", " "))
ax.set_xticks(x); ax.set_xticklabels([l for _, l in SUB]); ax.set_ylabel("rating (0-100)"); ax.set_ylim(0, 100)
ax.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.18))
fig.savefig(os.path.join(OUT, "tlx_subscales.pdf")); plt.close(fig)

# 3. outcomes: operator x (task, condition)
fig, ax = plt.subplots(figsize=(6.4, 2.2))
cols = [(tc, cc) for tc, _ in TASK for cc, _ in COND]
for j, (tc, cc) in enumerate(cols):
    for i, p in enumerate(PARTS):
        r = [r for r in rows if r["participant"] == p and r["task_code"] == tc and r["condition_code"] == cc][0]
        lab, col = OUTCOME[r["task_outcome"]]
        ax.add_patch(plt.Rectangle((j, i), 1, 1, facecolor=col, edgecolor="white", lw=2))
        ax.text(j + 0.5, i + 0.5, lab, ha="center", va="center", fontsize=6, color="white")
ax.set_xlim(0, len(cols)); ax.set_ylim(len(PARTS), 0)
ax.set_xticks(np.arange(len(cols)) + 0.5); ax.set_xticklabels(["%s\n%s" % (dict(TASK)[tc], dict(COND)[cc].replace("\n", " ")) for tc, cc in cols], fontsize=6.5)
ax.set_yticks(np.arange(len(PARTS)) + 0.5); ax.set_yticklabels(["%s (%s, robot %s)" % (p, [r for r in rows if r["participant"] == p][0]["vr_experience"].lower(), [r for r in rows if r["participant"] == p][0]["robot_experience"].lower()) for p in PARTS])
for s in ax.spines.values(): s.set_visible(False)
ax.tick_params(length=0)
fig.savefig(os.path.join(OUT, "tlx_outcomes.pdf")); plt.close(fig)

# 4. workload against experience (robot experience level 1 novice .. 3 experienced)
fig, axes = plt.subplots(1, 2, figsize=(6.4, 2.5))
lv = {"1": "novice", "2": "intermediate", "3": "experienced"}
for ax, key, title in ((axes[0], "robot_exp_level", "prior robot experience"), (axes[1], "vr_exp_level", "prior VR experience")):
    for cc, cl in COND:
        xs, ys = [], []
        for p in PARTS:
            rr = [r for r in rows if r["participant"] == p and r["condition_code"] == cc]
            xs.append(int(rr[0][key])); ys.append(np.mean([float(r["rtlx"]) for r in rr]))
        ax.scatter(np.array(xs) + {"VR": -0.08, "MM": 0, "SA": 0.08}[cc], ys, color=COL[cc], s=22, label=cl.replace("\n", " "))
    ax.set_xticks([1, 2, 3]); ax.set_xticklabels([lv["1"], lv["2"], lv["3"]]); ax.set_title(title); ax.set_ylim(0, 100)
axes[0].set_ylabel("mean RTLX per operator (0-100)")
axes[1].legend(frameon=False, loc="lower left", fontsize=6.5)
fig.savefig(os.path.join(OUT, "tlx_experience.pdf")); plt.close(fig)

# 5. table and tidy csv
lines = []
for cc, cl in COND:
    for tc, tl in TASK:
        v = [float(r["rtlx"]) for r in sel(cc, tc)]
        subs = [np.mean([float(r[s]) for r in sel(cc, tc)]) for s, _ in SUB]
        lines.append("    %s & %s & %.1f & %.1f & %s \\\\" % (cl.replace("\n", " "), tl, np.mean(v), np.std(v, ddof=1), " & ".join("%.0f" % m for m in subs)))
    v = [float(r["rtlx"]) for r in sel(cc)]
    subs = [np.mean([float(r[s]) for r in sel(cc)]) for s, _ in SUB]
    lines.append("    %s & both & \\textbf{%.1f} & %.1f & %s \\\\" % (cl.replace("\n", " "), np.mean(v), np.std(v, ddof=1), " & ".join("%.0f" % m for m in subs)))
    lines.append("    \\addlinespace")
tex = r"""\begin{table}[htbp]
  \centering
  \small
  \caption[NASA-TLX workload by condition and task]{Raw NASA-TLX workload
  (RTLX, the unweighted mean of the six subscales, 0--100) reported by the
  five operators after each trial: mean and standard deviation across
  operators, and the mean of each subscale. Lower is less workload; the
  performance subscale is scored so that lower means better perceived
  performance.}
  \label{tab:tlx}
  \begin{tabular}{p{3.1cm}p{1.9cm}rrrrrrrr}
    \toprule
    Condition & Task & RTLX & s.d. & Mental & Physical & Temporal & Perform. & Effort & Frustr. \\
    \midrule
%s
    \bottomrule
  \end{tabular}
\end{table}
""" % "\n".join(lines[:-1])
open(os.path.join(OUT, "tlx_table.tex"), "w").write(tex)
with open(os.path.join(HERE, "pilot", "data", "tlx_all.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
# numbers for the text
for cc, cl in COND:
    v = [float(r["rtlx"]) for r in sel(cc)]; print("%-30s RTLX %.1f +- %.1f" % (cl.replace("\n", " "), np.mean(v), np.std(v, ddof=1)))
for lvl in ("1", "2", "3"):
    v = [float(r["rtlx"]) for r in rows if r["robot_exp_level"] == lvl]
    if v: print("robot experience level %s: n=%d operators, RTLX %.1f" % (lvl, len({r['participant'] for r in rows if r['robot_exp_level'] == lvl}), np.mean(v)))
print("wrote tlx figures, table, csv")
