"""Master-arm cohort sim-to-real joint tracking, one bar per session, house
style. Run from the repository root:

    python3 thesis_v3/figures/make_master_figure.py
"""
import json, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRATCH = "/tmp/claude-1000/-home-gausms-kortex-ws/bbebc7ab-9df1-4fc1-80b7-ee284d9eaac6/scratchpad"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot")

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "xtick.labelsize": 6.5, "ytick.labelsize": 7,
    "figure.dpi": 150, "savefig.bbox": "tight", "axes.spines.top": False,
    "axes.spines.right": False,
})
COLORS = {"M1": "#2c6fbb", "M2": "#c1392b", "M3": "0.45"}

data = json.load(open(os.path.join(SCRATCH, "master_arm_sessions.json")))
data = [d for d in data if d["joint_rms_deg"] is not None]
data.sort(key=lambda d: d["joint_rms_deg"])

labels = [f"{d['participant']}\n{d['task'].split()[0].lower()} {d['condition'][0]}" for d in data]
vals = [d["joint_rms_deg"] for d in data]
colors = [COLORS[d["participant"]] for d in data]

fig, ax = plt.subplots(figsize=(6.0, 2.6))
x = np.arange(len(data))
ax.bar(x, vals, color=colors, alpha=0.85)
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=6)
ax.set_ylabel("sim-real joint RMS (deg)")
handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in COLORS.values()]
ax.legend(handles, ["M1", "M2", "M3"],
          frameon=False, fontsize=6.5, loc="upper left")
fig.savefig(os.path.join(OUT, "master_tracking.pdf"))
plt.close(fig)
print("wrote master_tracking.pdf")
