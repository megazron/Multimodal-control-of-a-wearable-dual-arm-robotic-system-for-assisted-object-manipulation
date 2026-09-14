"""Master/VR-vs-sim-vs-real tracking figure, best-tracking pilot session
(lowest sim-to-real joint RMS across all 16 sessions: P4, object tracking,
direct control -- 0.0185 rad = 1.06 deg RMS over 7 joints, real hardware
polled throughout). House style: muted grey/blue/red, font.size 8, PDF.

Run from the repository root:

    python3 extras/figures/make_tracking_figure.py
"""
import csv, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot")
os.makedirs(OUT, exist_ok=True)
SESSION = "20260904_172909_Feifan_VRDIrect_ObjecttTracking"  # P4, direct, object tracking
HAND = "left"
PATH = os.path.join("recordings", "sessions", SESSION, "trail.csv")

plt.rcParams.update({
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7,
    "figure.dpi": 150, "savefig.bbox": "tight", "axes.spines.top": False,
    "axes.spines.right": False,
})
GREY, BLUE, RED = "0.35", "#2c6fbb", "#c1392b"


def f(x):
    try:
        return float(x) if x not in (None, "") else None
    except ValueError:
        return None


t, sim_j4, real_j4 = [], [], []
mx, my, mz, ex, ey, ez = [], [], [], [], [], []
with open(PATH) as fh:
    for row in csv.DictReader(fh):
        tv = f(row.get("t"))
        sj = f(row.get(f"sim_{HAND}_j4"))
        rj = f(row.get(f"real_{HAND}_j4"))
        if tv is not None and sj is not None and rj is not None:
            t.append(tv); sim_j4.append(sj); real_j4.append(rj)
        mxv, myv, mzv = f(row.get(f"master_{HAND}_x")), f(row.get(f"master_{HAND}_y")), f(row.get(f"master_{HAND}_z"))
        exv, eyv, ezv = f(row.get(f"ee_{HAND}_x")), f(row.get(f"ee_{HAND}_y")), f(row.get(f"ee_{HAND}_z"))
        if None not in (mxv, myv, mzv, exv, eyv, ezv):
            mx.append(mxv); my.append(myv); mz.append(mzv)
            ex.append(exv); ey.append(eyv); ez.append(ezv)

t = np.array(t); sim_j4 = np.array(sim_j4); real_j4 = np.array(real_j4)
mx, my, mz = np.array(mx), np.array(my), np.array(mz)
ex, ey, ez = np.array(ex), np.array(ey), np.array(ez)

# ---------------------------------------------------------------------------
# (a) joint 4, sim vs real, over time
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(3.1, 2.3))
ax.plot(t, np.degrees(sim_j4), color=BLUE, linewidth=0.8, label="simulated")
ax.plot(t, np.degrees(real_j4), color=RED, linewidth=0.8, linestyle="--", label="real arm")
ax.set_xlabel("time into session (s)")
ax.set_ylabel("joint 4 angle (deg)")
ax.legend(frameon=False, fontsize=6.5, loc="upper right", handlelength=1.4)
fig.savefig(os.path.join(OUT, "track_joint.pdf"))
plt.close(fig)

# ---------------------------------------------------------------------------
# (b) joint tracking error, all 7 joints RMS, over time
# ---------------------------------------------------------------------------
rows_err = []
with open(PATH) as fh:
    for row in csv.DictReader(fh):
        tv = f(row.get("t"))
        diffs = []
        ok = True
        for j in range(1, 8):
            sv = f(row.get(f"sim_{HAND}_j{j}"))
            rv = f(row.get(f"real_{HAND}_j{j}"))
            if sv is None or rv is None:
                ok = False
                break
            diffs.append((sv - rv) ** 2)
        if ok and tv is not None:
            rows_err.append((tv, np.degrees(np.sqrt(np.mean(diffs)))))
te = np.array([r[0] for r in rows_err])
je = np.array([r[1] for r in rows_err])

fig, ax = plt.subplots(figsize=(3.1, 2.3))
ax.plot(te, je, color=GREY, linewidth=0.6)
ax.axhline(np.sqrt(np.mean(je**2)), color=RED, linewidth=0.8, linestyle=":")
ax.set_xlabel("time into session (s)")
ax.set_ylabel("sim-real joint RMS (deg)")
fig.savefig(os.path.join(OUT, "track_joint_error.pdf"))
plt.close(fig)

# ---------------------------------------------------------------------------
# (c) relative Cartesian path, master (operator's hand) vs sim (achieved),
#     each centred on its own mean so the two coordinate frames overlay --
#     an absolute overlay is meaningless here, since the master's raw pose
#     and the robot's end-effector pose are naturally in different places.
# ---------------------------------------------------------------------------
mxr, mzr = mx - mx.mean(), mz - mz.mean()
exr, ezr = ex - ex.mean(), ez - ez.mean()

fig, ax = plt.subplots(figsize=(3.1, 2.6))
ax.plot(mxr, mzr, color=BLUE, linewidth=0.6, label="operator's hand (VR)")
ax.plot(exr, ezr, color=RED, linewidth=0.6, label="robot hand (sim)")
ax.set_xlabel("sideways, centred (m)")
ax.set_ylabel("height, centred (m)")
ax.legend(frameon=False, fontsize=6.5, loc="upper right", handlelength=1.4)
ax.set_aspect("equal", adjustable="datalim")
fig.savefig(os.path.join(OUT, "track_cartesian.pdf"))
plt.close(fig)

print("wrote track_joint.pdf, track_joint_error.pdf, track_cartesian.pdf")
print(f"joint RMS overall: {np.sqrt(np.mean(je**2)):.3f} deg over {len(je)} samples")
