"""Master-arm cohort sim-to-real joint tracking, one bar per session, house
style. Run from the repository root:

    python3 thesis_v3/figures/make_master_figure.py
"""
import json, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.join(HERE, "pilot", "data")  # the derived session tables, kept in the repo
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot")

plt.rcParams.update({
    "font.size": 9.0, "axes.titlesize": 9.5, "axes.labelsize": 9.0,
    "xtick.labelsize": 8.0, "ytick.labelsize": 8.5,
    "figure.dpi": 150, "savefig.bbox": "tight", "axes.spines.top": False,
    "axes.spines.right": False,
})
ODD = "M1"  # the one operator whose tracking error stands out
COLORS = {"M1": "#c1392b", "M2": "#2c6fbb", "M3": "#2c6fbb"}

data = json.load(open(os.path.join(SCRATCH, "master_arm_sessions.json")))
data = [d for d in data if d["joint_rms_deg"] is not None]
data.sort(key=lambda d: d["joint_rms_deg"])

SHORT = {"Pick and place": "pick and\nplace", "Target reaching": "target\nreaching"}
labels = [f"{SHORT.get(d['task'], 'trial')}\n{d['condition'].lower()}" for d in data]
vals = [d["joint_rms_deg"] for d in data]
colors = [COLORS[d["participant"]] for d in data]

fig, ax = plt.subplots(figsize=(6.4, 2.8))
x = np.arange(len(data))
ax.bar(x, vals, color=colors, alpha=0.85)
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=7)
ax.set_ylabel("sim-real joint RMS (deg)")
handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in ("#2c6fbb", "#c1392b")]
ax.legend(handles, ["trials tracking within a few degrees", "the one operator whose error stands out"],
          frameon=False, fontsize=8.5, loc="upper left")
ax.set_xlabel("mannequin-master trials, sorted by tracking error")
fig.savefig(os.path.join(OUT, "master_tracking.pdf"))
plt.close(fig)
print("wrote master_tracking.pdf")
