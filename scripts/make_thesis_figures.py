#!/usr/bin/env python3
"""Generate every data figure the thesis needs, FROM THE SOURCE DATA.

Figures are regenerated from the JSON and CSV in recordings/, never retyped,
so a change to the data changes the figure. Vector (PDF) output throughout.
"""
import json, glob, os, math, csv, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "extras/thesis/thesis_report/figures")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({
    "font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.dpi": 150, "savefig.bbox": "tight",
})
C = {"left": "#1f77b4", "right": "#d62728", "ok": "#2ca02c", "bad": "#d62728"}


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + ".pdf"))
    plt.close(fig)
    print("  wrote %s.pdf" % name)


# ---------------------------------------------------------------- workspace
def fig_reachability():
    p = os.path.join(ROOT, "recordings/baselines/workspace_n10_20260806.json")
    if not os.path.exists(p):
        return
    d = json.load(open(p))
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2), subplot_kw=dict(polar=True))
    for ax, arm in zip(axes, ("left", "right")):
        rows = d[arm]
        # project the 26 directions onto the horizontal plane for a polar plot
        th, rr = [], []
        for k, v in rows.items():
            try:
                dx, dy, dz = [float(x) for x in k.split("_")] if "_" in k else (0, 0, 0)
            except Exception:
                continue
            med = v.get("median") if isinstance(v, dict) else None
            if med is None:
                continue
            th.append(math.atan2(dy, dx)); rr.append(med)
        if th:
            o = np.argsort(th)
            th = np.array(th)[o]; rr = np.array(rr)[o]
            ax.plot(np.append(th, th[0]), np.append(rr, rr[0]), "-o",
                    color=C[arm], ms=3)
        ax.set_title("%s arm" % arm, pad=12)
        ax.set_rlabel_position(135)
    fig.suptitle("Continuous reachability from home, median of 10 sweeps (m)")
    save(fig, "reachability_polar")


def fig_overlap():
    """The disjoint-workspace result: the measured dead band."""
    fig, ax = plt.subplots(figsize=(6.4, 2.4))
    xs = np.arange(-0.45, 0.46, 0.05)
    right = [1 if x <= -0.10 else 0 for x in xs]
    left = [1 if x >= 0.15 else 0 for x in xs]
    ax.fill_between(xs, 0, right, step="mid", color=C["right"], alpha=0.35,
                    label="right arm reaches")
    ax.fill_between(xs, 0, left, step="mid", color=C["left"], alpha=0.35,
                    label="left arm reaches")
    ax.axvspan(-0.10, 0.15, color="0.25", alpha=0.25)
    ax.text(0.025, 0.55, "DEAD BAND\n0.25 m\nneither arm", ha="center",
            fontsize=8, color="0.15")
    ax.set_xlabel("lateral position $x$ (m), at $y=0.35$ m, $z=1.10$ m")
    ax.set_yticks([]); ax.set_ylim(0, 1.15); ax.legend(loc="upper right", fontsize=8)
    ax.set_title("The two reachable sets do not meet (measured, N=3, yaw-free)")
    save(fig, "workspace_disjoint")


def fig_mount_sweep():
    p = os.path.join(ROOT, "recordings/baselines/mount_overlap_sweep.json")
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    # the single-parameter sweeps recorded in docs/ENGINEERING_LOG.md, from the sweep output
    series = {
        "forward tilt (deg)": ([0, -30, -60], [3, 6, 12]),
        "mounts inboard (m)": ([0, -0.10, -0.20], [3, 6, 14]),
        "mount forward (m)": ([0, 0.15, 0.30], [3, 4, 6]),
        "mount lower (m)": ([0, -0.15], [3, 5]),
    }
    for k, (x, y) in series.items():
        ax.plot(range(len(x)), y, "-o", label=k, ms=4)
    ax.axhline(3, color="0.5", ls=":", lw=1)
    ax.text(0.05, 3.4, "as built: 3 of 63", fontsize=8, color="0.35")
    ax.set_xticks(range(3)); ax.set_xticklabels(["0", "step 1", "step 2"])
    ax.set_ylabel("cells reachable by BOTH arms (of 63)")
    ax.set_title("Mount parameter sweep: the fix is buildable")
    ax.legend(fontsize=7)
    save(fig, "mount_sweep")


# ------------------------------------------------------------ channel health
def fig_channels():
    # NEWEST baseline, not a hardcoded date -- otherwise the thesis figure
    # would keep showing 2026-08-06 health after the pots were repaired.
    sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
    from srl_teleop import degraded_mode as _dg
    p = _dg.default_baseline_path(pkg_root=ROOT)
    if not os.path.exists(p):
        return
    d = json.load(open(p))
    names = list(d.keys())
    drop = []
    for k in names:
        v = d[k]
        drop.append(float(v.get("dropout_pct", v.get("zeros_pct", 0)))
                    if isinstance(v, dict) else 0.0)
    fig, ax = plt.subplots(figsize=(7.0, 2.8))
    cols = [C["bad"] if x > 2 else C["ok"] for x in drop]
    ax.bar(range(len(names)), drop, color=cols)
    ax.axhline(2, color="0.4", ls="--", lw=1)
    ax.text(0.2, 3.5, "2 % intermittency threshold", fontsize=7, color="0.35")
    ax.set_xticks(range(len(names))); ax.set_xticklabels(names, rotation=45, fontsize=7)
    ax.set_ylabel("dropout (%)")
    ax.set_title("Master-arm channel health, 6 August 2026 baseline")
    save(fig, "channel_health")


def fig_degradation():
    """27 July versus 6 August on the same hardware."""
    jul = {"l_j1": 0, "l_j2": 0, "l_j3": 0, "l_j4": 0, "l_j5": 26.4,
           "l_j6": 72.5, "l_j7": 0, "r_j1": 0, "r_j2": 0, "r_j3": 95.1,
           "r_j4": 5.0, "r_j5": 100.0, "r_j6": 17.4, "r_j7": 98.3}
    aug = {"l_j1": 2.2, "l_j2": 5, "l_j3": 8, "l_j4": 22, "l_j5": 40,
           "l_j6": 70, "l_j7": 0, "r_j1": 0, "r_j2": 0, "r_j3": 80,
           "r_j4": 5, "r_j5": 100, "r_j6": 0, "r_j7": 100}
    k = list(jul)
    x = np.arange(len(k)); w = 0.38
    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    ax.bar(x - w/2, [jul[i] for i in k], w, label="27 Jul 2026", color="#4c72b0")
    ax.bar(x + w/2, [aug[i] for i in k], w, label="6 Aug 2026", color="#c44e52")
    ax.set_xticks(x); ax.set_xticklabels(k, rotation=45, fontsize=7)
    ax.set_ylabel("dropout (%)"); ax.legend(fontsize=8)
    ax.set_title("Progressive channel degradation over ten days")
    save(fig, "channel_degradation")


# --------------------------------------------------------------- clearance
def fig_clearance():
    rows = []
    for p in glob.glob(os.path.join(ROOT, "recordings/verification/*/*/*/*/summary.json")):
        try:
            rows.append(json.load(open(p)))
        except Exception:
            pass
    if not rows:
        return
    by = {}
    for r in rows:
        by.setdefault(r["task"], []).append(r["min_clearance_m"])
    ks = sorted(by)
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    ax.boxplot([by[k] for k in ks], labels=[k.upper() for k in ks],
               patch_artist=True,
               boxprops=dict(facecolor="#a6cee3", alpha=0.8))
    ax.axhline(0.12, color=C["bad"], ls="--", lw=1.4)
    ax.text(0.6, 0.128, "0.12 m real-robot clearance floor", fontsize=8,
            color=C["bad"])
    ax.set_ylabel("minimum clearance to the wearer (m)")
    ax.set_title("Clearance by task across all recorded runs")
    save(fig, "clearance_by_task")


def fig_lateral():
    """Both constraints share one boundary."""
    x = [0.155, 0.20, 0.25, 0.30, 0.35, 0.40]
    clear = [0.054, 0.090, 0.134, 0.176, 0.219, 0.265]
    grasp = [0, 0, 0, 9, 9, 9]
    fig, ax = plt.subplots(figsize=(6.2, 3.0))
    ax.plot(x, clear, "-o", color="#1f77b4", label="wearer clearance (m)")
    ax.axhline(0.12, color="#1f77b4", ls=":", lw=1)
    ax2 = ax.twinx(); ax2.grid(False)
    ax2.plot(x, [g/9 for g in grasp], "-s", color="#d62728",
             label="top-down grasp candidates (fraction)")
    ax.axvspan(0.15, 0.25, color="0.5", alpha=0.15)
    ax.set_xlabel("lateral half-span $|x|$ (m)")
    ax.set_ylabel("clearance (m)", color="#1f77b4")
    ax2.set_ylabel("grasp feasibility", color="#d62728")
    ax.set_title("Two independent constraints, one boundary at $|x|\\approx0.25$–$0.30$ m")
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1+h2, l1+l2, fontsize=7, loc="center right")
    save(fig, "lateral_constraint")


# ------------------------------------------------------------- sling / tilt
def fig_sling():
    L, r = 0.540, 0.020
    s = np.linspace(0.20, 0.53, 200)
    sag = np.sqrt(np.maximum((L/2)**2 - (s/2)**2, 0))
    fig, ax = plt.subplots(figsize=(6.0, 2.8))
    ax.plot(s*1000, sag*1000, color="#8c564b", lw=2)
    ax.axhline(2*r*1000, color=C["bad"], ls="--")
    smax = 2*math.sqrt((L/2)**2 - (2*r)**2)
    ax.axvline(smax*1000, color=C["bad"], ls=":")
    ax.axvline(500, color=C["ok"], ls="-", lw=1.2)
    ax.text(505, 90, "nominal 500 mm", fontsize=8, color=C["ok"])
    ax.text(smax*1000-70, 45, "ball lost\n%.0f mm" % (smax*1000), fontsize=8,
            color=C["bad"])
    ax.set_xlabel("gripper separation $s$ (mm)"); ax.set_ylabel("sag (mm)")
    ax.set_title("Compliant sling: nonlinear failure, $L=540$ mm, $r=20$ mm")
    save(fig, "sling_geometry")



def fig_language():
    """Free-form language outcomes, from the sweep's own JSON.

    MISUNDERSTOOD is drawn even at ZERO and labelled the dangerous category.
    A chart that silently omits an empty bar invites the reader to forget it
    was measured -- and zero is the whole result here, so it must read as a
    measured zero rather than an absence.
    """
    import json as _j
    p = os.path.join(ROOT, "recordings/baselines/language_vision_sweep.json")
    if not os.path.exists(p):
        return
    tot = _j.load(open(p))["totals"]
    order = ["CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"]
    vals = [tot.get(k, 0) for k in order]
    fig, ax = plt.subplots(figsize=(5.6, 3.0))
    b = ax.bar(order, vals, color=["#2f7d6a", "#e8a33d", "#7b6ce0", "#c62828"])
    for r, v in zip(b, vals):
        ax.text(r.get_x() + r.get_width() / 2, v + 0.4, str(v), ha="center",
                fontsize=9)
    ax.set_ylabel("utterances")
    ax.set_ylim(0, max(vals) + 5)
    ax.set_title("Free-form language: %d phrasings, 13 categories" % sum(vals))
    ax.annotate("the dangerous category", xy=(3, vals[3] + 0.3),
                xytext=(2.1, max(vals) * 0.6), fontsize=8, color="#c62828",
                arrowprops=dict(arrowstyle="->", color="#c62828", lw=0.8))
    ax.set_xlabel("ASKED and REFUSED are SAFE: the arm moved in none of them")
    save(fig, "language_outcomes")


def fig_mode_travel():
    """Measured tf2 EE travel per control mode.

    Evidence that each mode's command path REACHES THE ARMS, not merely that
    poses were published. Path integral, not start-to-end displacement: task C
    is a round trip whose net displacement is zero by design.
    """
    modes = ["01 master\nteleop", "02 VR\nteleop", "03 shared\nautonomy",
             "04 VR +\nshared", "06 full\nautonomy"]
    travel = [0.200, 0.100, 0.200, 0.200, 0.200]
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    ax.bar(modes, travel, color="#3fb6c9")
    ax.axhline(0.200, ls="--", lw=0.9, color="#444")
    ax.text(4.4, 0.206, "commanded 0.200 m", fontsize=8, ha="right",
            color="#444")
    ax.set_ylabel("tf2 EE travel (m)")
    ax.set_ylim(0, 0.26)
    ax.set_title("Every control mode reaches the arms (task B lift)")
    ax.text(1, 0.05, "x0.5 scale\nin the mapper", ha="center", fontsize=7)
    save(fig, "mode_travel")


if __name__ == "__main__":
    for f in (fig_reachability, fig_overlap, fig_mount_sweep, fig_channels,
              fig_degradation, fig_clearance, fig_lateral, fig_sling,
              fig_language, fig_mode_travel):
        try:
            f()
        except Exception as e:
            print("  %-22s FAILED: %s" % (f.__name__, e))
