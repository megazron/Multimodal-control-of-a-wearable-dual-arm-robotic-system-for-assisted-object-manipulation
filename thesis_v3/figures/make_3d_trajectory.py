"""A genuine 3D trajectory figure (not a 2D projection): the operator's raw
VR-hand path and the simulated robot hand's achieved path, for the
best-tracking pilot session, in one 3D axes object. House style: muted
grey/blue/red, font.size 8, PDF.

Run from the repository root:
    python3 thesis_v3/figures/make_3d_trajectory.py
"""
import csv, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
import sys as _sys
_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from session_paths import session_dir  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot")
SESSION = "20260904_172909"  # P4, pick and place, direct
HAND = "left"
PATH = os.path.join(session_dir(SESSION), "trail.csv")

plt.rcParams.update({
    "font.size": 8, "axes.labelsize": 7.5, "axes.titlesize": 8.5,
    "figure.dpi": 150, "savefig.bbox": "tight",
})
GREY, BLUE, RED = "0.35", "#2c6fbb", "#c1392b"


def f(x):
    try:
        return float(x) if x not in (None, "") else None
    except ValueError:
        return None


mx, my, mz, ex, ey, ez = [], [], [], [], [], []
with open(PATH) as fh:
    for row in csv.DictReader(fh):
        mxv, myv, mzv = f(row.get(f"master_{HAND}_x")), f(row.get(f"master_{HAND}_y")), f(row.get(f"master_{HAND}_z"))
        exv, eyv, ezv = f(row.get(f"ee_{HAND}_x")), f(row.get(f"ee_{HAND}_y")), f(row.get(f"ee_{HAND}_z"))
        if None not in (mxv, myv, mzv, exv, eyv, ezv):
            mx.append(mxv); my.append(myv); mz.append(mzv)
            ex.append(exv); ey.append(eyv); ez.append(ezv)

mx, my, mz = np.array(mx), np.array(my), np.array(mz)
ex, ey, ez = np.array(ex), np.array(ey), np.array(ez)

fig = plt.figure(figsize=(6.4, 3.1))

ax1 = fig.add_subplot(1, 2, 1, projection="3d")
ax1.plot(mx, my, mz, color=BLUE, linewidth=0.6)
ax1.scatter([mx[0]], [my[0]], [mz[0]], color=GREY, s=16)
ax1.scatter([mx[-1]], [my[-1]], [mz[-1]], color="black", marker="s", s=16)
ax1.set_title("operator's hand (VR controller)", fontsize=8)
ax1.set_xlabel("x (m)", labelpad=0)
ax1.set_ylabel("y (m)", labelpad=0)
ax1.set_zlabel("z (m)", labelpad=0)
ax1.tick_params(labelsize=6)

ax2 = fig.add_subplot(1, 2, 2, projection="3d")
ax2.plot(ex, ey, ez, color=RED, linewidth=0.6)
ax2.scatter([ex[0]], [ey[0]], [ez[0]], color=GREY, s=16, label="start")
ax2.scatter([ex[-1]], [ey[-1]], [ez[-1]], color="black", marker="s", s=16, label="end")
ax2.set_title("robot hand (simulated, achieved)", fontsize=8)
ax2.set_xlabel("x (m)", labelpad=0)
ax2.set_ylabel("y (m)", labelpad=0)
ax2.set_zlabel("z (m)", labelpad=0)
ax2.tick_params(labelsize=6)
ax2.legend(frameon=False, fontsize=6.5, loc="upper right")

fig.savefig(os.path.join(OUT, "track_3d.pdf"))
plt.close(fig)
print("wrote track_3d.pdf")
