#!/usr/bin/env python3
"""
analyse_teleop.py — turn a teleop_recorder CSV into numbers and plots
=============================================================================
The central output is the 3x3 GAIN MATRIX: robot EE displacement regressed
against commanded master displacement, per axis, over the Phase A sweeps.

  G[i][j] = d(robot_i) / d(master_j)

A correct mapping is DIAGONAL and POSITIVE. A negative diagonal term is an
inverted axis. A large off-diagonal term is a crossed axis. If the matrix is
a clean signed permutation, AXIS_MAP can express it. If every axis has large
off-diagonal energy, the mount rotation is not a multiple of 90 deg and
AXIS_MAP cannot express it at all -- that needs a measured rotation matrix,
and this script says so explicitly rather than fitting a permutation anyway.

Usage:
  python3 -m srl_teleop.analyse_teleop <recording.csv> [--arm left]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

AXES = ("x", "y", "z")
# Phase A segments, in the order the protocol drives them. Each pair is the
# axis the OPERATOR moved along; the regression discovers where it landed.
PHASE_A = ("left", "right", "up", "down", "forward", "back")


def load(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit("empty recording: %s" % path)
    return rows


def fnum(row, key):
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def vec(row, prefix, comps=AXES):
    out = [fnum(row, "%s%s" % (prefix, c)) for c in comps]
    return None if any(v is None for v in out) else np.array(out)


def segments(rows, arm):
    """Group rows by segment label."""
    out = {}
    for r in rows:
        out.setdefault(r["segment"], []).append(r)
    return out


def gain_matrix(rows, arm):
    """Regress robot EE displacement on commanded displacement.

    Uses every Phase A sample for this arm, expressed as a displacement from
    that segment's own starting point, so a per-segment offset cannot bias
    the fit. Least squares over all axes at once yields the full 3x3.
    """
    segs = segments(rows, arm)
    M, R = [], []
    used = []
    for name, rs in segs.items():
        if not name.startswith(f"{arm}_A_"):
            continue
        pts = [(vec(r, f"{arm}_cmd_"), vec(r, f"{arm}_ee_")) for r in rs]
        pts = [(c, e) for c, e in pts if c is not None and e is not None]
        if len(pts) < 5:
            continue
        c0, e0 = pts[0]
        for c, e in pts[1:]:
            M.append(c - c0)
            R.append(e - e0)
        used.append(name)
    if len(M) < 20:
        return None, used, 0
    M = np.array(M)
    R = np.array(R)
    G, *_ = np.linalg.lstsq(M, R, rcond=None)     # M @ G ~= R
    return G.T, used, len(M)                      # G.T[i][j] = d robot_i / d master_j


def classify(G):
    """Is the gain matrix a clean signed permutation, or a real rotation?"""
    notes = []
    if G is None:
        return ["no usable Phase A data"]
    for i in range(3):
        row = G[i]
        dom = int(np.argmax(np.abs(row)))
        mag = abs(row[dom])
        off = math.sqrt(sum(row[j] ** 2 for j in range(3) if j != dom))
        frac = off / mag if mag > 1e-9 else float("inf")
        notes.append(
            "robot %s <- master %s  gain %+.3f, off-axis %.0f%% of dominant"
            % (AXES[i], AXES[dom], row[dom], 100 * frac))
    return notes


def is_permutation(G, tol=0.5):
    """True when each robot axis is dominated by ONE master axis."""
    if G is None:
        return False
    doms = []
    for i in range(3):
        row = G[i]
        d = int(np.argmax(np.abs(row)))
        mag = abs(row[d])
        off = math.sqrt(sum(row[j] ** 2 for j in range(3) if j != d))
        if mag < 1e-6 or off / mag > tol:
            return False
        doms.append(d)
    return len(set(doms)) == 3


def ranges(rows, arm):
    """Per-axis travel of command and robot, over the whole session."""
    C = [vec(r, f"{arm}_cmd_") for r in rows]
    E = [vec(r, f"{arm}_ee_") for r in rows]
    C = np.array([c for c in C if c is not None])
    E = np.array([e for e in E if e is not None])
    if not len(C) or not len(E):
        return None, None
    return (C.max(axis=0) - C.min(axis=0)), (E.max(axis=0) - E.min(axis=0))


def lag(rows, arm, hz=50.0, max_lag=100):
    """Cross-correlation lag, commanded vs actual, per axis."""
    out = {}
    for i, ax in enumerate(AXES):
        c = [fnum(r, f"{arm}_cmd_{ax}") for r in rows]
        e = [fnum(r, f"{arm}_ee_{ax}") for r in rows]
        pair = [(a, b) for a, b in zip(c, e) if a is not None and b is not None]
        if len(pair) < 200:
            out[ax] = None
            continue
        a = np.array([p[0] for p in pair])
        b = np.array([p[1] for p in pair])
        a = a - a.mean()
        b = b - b.mean()
        if a.std() < 1e-9 or b.std() < 1e-9:
            out[ax] = None
            continue
        best, bl = -2.0, 0
        for L in range(0, max_lag):
            n = len(a) - L
            v = float(np.corrcoef(a[:n], b[L:L + n])[0, 1])
            if not math.isnan(v) and v > best:
                best, bl = v, L
        out[ax] = (bl / hz, best)
    return out


def tracking_error(rows, arm):
    errs = []
    for r in rows:
        c, e = vec(r, f"{arm}_cmd_"), vec(r, f"{arm}_ee_")
        if c is not None and e is not None:
            errs.append(float(np.linalg.norm(c - e)))
    if not errs:
        return None, None
    return float(np.sqrt(np.mean(np.square(errs)))), float(np.max(errs))


def clutch_jumps(rows, arm, hz=50.0, window_s=0.2):
    """EE displacement in the first 200 ms after each clutch re-engage."""
    out = []
    prev = None
    n = int(window_s * hz)
    for i, r in enumerate(rows):
        c = fnum(r, f"{arm}_clutch")
        if c is None:
            continue
        if prev is not None and prev == 0.0 and c == 1.0:
            a = vec(rows[i], f"{arm}_ee_")
            j = min(i + n, len(rows) - 1)
            b = vec(rows[j], f"{arm}_ee_")
            if a is not None and b is not None:
                out.append((float(fnum(rows[i], "t") or 0.0),
                            float(np.linalg.norm(b - a))))
        prev = c
    return out


def dropouts(rows, arm):
    counts = {}
    for j in range(1, 8):
        k = f"{arm}_j{j}"
        n = sum(1 for r in rows if fnum(r, k) == 0.0)
        counts[k] = n
    return counts


def button_analysis(path):
    """Which physical button drives which reported index (Task 5).

    The recorder MEASURES this at the start of every run and writes it to a
    sidecar, so it is read back here rather than re-inferred from the data.
    """
    meta_path = path + ".meta.json"
    if not os.path.exists(meta_path):
        return None
    try:
        with open(meta_path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def j1_analysis(rows, arm):
    seg = f"{arm}_shoulder_rot"
    rs = [r for r in rows if r["segment"] == seg]
    out = {}
    for j in range(1, 8):
        vals = [fnum(r, f"{arm}_j{j}") for r in rs]
        vals = [v for v in vals if v is not None]
        if vals:
            out[f"j{j}"] = (len(set(vals)), max(vals) - min(vals))
    return out


def plots(rows, arm, base):
    made = []
    t = [fnum(r, "t") for r in rows]
    C = {ax: [fnum(r, f"{arm}_cmd_{ax}") for r in rows] for ax in AXES}
    E = {ax: [fnum(r, f"{arm}_ee_{ax}") for r in rows] for ax in AXES}

    # 1. commanded vs actual, stacked
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for k, ax in enumerate(AXES):
        axs[k].plot(t, C[ax], label="commanded", lw=1.0)
        axs[k].plot(t, E[ax], label="actual EE", lw=1.0)
        axs[k].set_ylabel(f"{ax} (m)")
        axs[k].grid(alpha=0.3)
    axs[0].legend(loc="upper right")
    axs[0].set_title(f"{arm}: commanded vs actual EE")
    axs[-1].set_xlabel("t (s)")
    p = f"{base}_{arm}_tracking.png"
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig); made.append(p)

    # 2. 3D trajectory
    fig = plt.figure(figsize=(9, 8))
    a3 = fig.add_subplot(111, projection="3d")
    cc = [(x, y, z) for x, y, z in zip(C["x"], C["y"], C["z"])
          if None not in (x, y, z)]
    ee = [(x, y, z) for x, y, z in zip(E["x"], E["y"], E["z"])
          if None not in (x, y, z)]
    if cc:
        a3.plot(*zip(*cc), label="commanded", lw=0.8)
    if ee:
        a3.plot(*zip(*ee), label="actual", lw=0.8)
    a3.set_xlabel("x"); a3.set_ylabel("y"); a3.set_zlabel("z")
    a3.set_title(f"{arm}: trajectory"); a3.legend()
    p = f"{base}_{arm}_traj3d.png"
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig); made.append(p)

    # 3. tracking error with clutch shading
    err, cl = [], []
    for r in rows:
        c, e = vec(r, f"{arm}_cmd_"), vec(r, f"{arm}_ee_")
        err.append(float(np.linalg.norm(c - e)) if c is not None and e is not None else None)
        cl.append(fnum(r, f"{arm}_clutch"))
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, err, lw=1.0, color="crimson")
    inseg = False
    for i in range(len(rows)):
        if cl[i] == 0.0 and not inseg:
            s, inseg = t[i], True
        elif cl[i] != 0.0 and inseg:
            ax.axvspan(s, t[i], color="grey", alpha=0.25)
            inseg = False
    if inseg:
        ax.axvspan(s, t[-1], color="grey", alpha=0.25)
    ax.set_xlabel("t (s)"); ax.set_ylabel("|cmd - actual| (m)")
    ax.set_title(f"{arm}: tracking error (grey = clutch DISENGAGED)")
    ax.grid(alpha=0.3)
    p = f"{base}_{arm}_error.png"
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig); made.append(p)
    return made


def gain_heatmap(G, arm, base):
    if G is None:
        return []
    fig, ax = plt.subplots(figsize=(6, 5))
    lim = float(np.abs(G).max()) or 1.0
    im = ax.imshow(G, cmap="RdBu_r", vmin=-lim, vmax=lim)
    ax.set_xticks(range(3), [f"master {a}" for a in AXES])
    ax.set_yticks(range(3), [f"robot {a}" for a in AXES])
    for i in range(3):
        for j in range(3):
            ax.text(j, i, "%+.3f" % G[i][j], ha="center", va="center",
                    color="black", fontsize=12)
    ax.set_title(f"{arm}: gain matrix  d(robot)/d(master)")
    fig.colorbar(im)
    p = f"{base}_{arm}_gain.png"
    fig.tight_layout(); fig.savefig(p, dpi=110); plt.close(fig)
    return [p]


def analyse(path, arms):
    rows = load(path)
    base = os.path.splitext(path)[0]
    made = []
    print("=" * 72)
    print("ANALYSIS of %s  (%d rows)" % (path, len(rows)))
    print("=" * 72)

    meta = button_analysis(path)
    print("\n--- CLUTCH BUTTON MAPPING (Task 5) ---")
    if not meta:
        print("  no sidecar .meta.json beside the CSV - mapping unknown")
    else:
        m = meta.get("button_mapping", {})
        measured = meta.get("button_mapping_measured", False)
        for side, idx in sorted(m.items()):
            print("  %-6s button -> btn%s" % (side, idx))
        if measured:
            print("  -> MEASURED this run, not inferred.")
            if m.get("left") == 2 and m.get("right") == 1:
                print("     Confirms the suspected reversal: the left button")
                print("     reports btn2 despite BUTTON1_PIN=2 being the left pin.")
            elif m.get("left") == 1 and m.get("right") == 2:
                print("     Straight mapping - the old CSV inference was wrong.")
        else:
            print("  -> NOT measured (both buttons drove the same channel); "
                  "fell back to btn1=left, btn2=right. Treat as unconfirmed.")
        segs = meta.get("segments", [])
        if segs:
            weak = [s for s in segs
                    if s.get("motion_m", 0) < 0.02 and "rest" not in s["label"]
                    and "clutch" not in s["label"]]
            print("\n  %d segments recorded" % len(segs))
            if weak:
                print("  %d with under 2 cm of master motion - these will "
                      "contribute little to the gain fit:" % len(weak))
                for s in weak:
                    print("     %-26s %.3f m over %.1f s"
                          % (s["label"], s["motion_m"], s["seconds"]))

    for arm in arms:
        print("\n" + "=" * 72)
        print("ARM: %s" % arm)
        print("=" * 72)

        j1 = j1_analysis(rows, arm)
        if j1:
            print("\n--- SHOULDER ROTATION, channel response (Task 1) ---")
            for k, (nd, spread) in sorted(j1.items()):
                flag = "  <== FROZEN" if nd <= 1 else ""
                print("  %-4s %3d distinct, spread %7.2f deg%s" % (k, nd, spread, flag))

        G, used, n = gain_matrix(rows, arm)
        print("\n--- GAIN MATRIX  d(robot)/d(master)  [%d samples, %d sweeps] ---"
              % (n, len(used)))
        if G is None:
            print("  insufficient Phase A data for %s" % arm)
        else:
            print("            master_x  master_y  master_z")
            for i in range(3):
                print("  robot_%s   %+8.3f  %+8.3f  %+8.3f" % (AXES[i], *G[i]))
            print()
            for line in classify(G):
                print("  " + line)
            perm = is_permutation(G)
            print("\n  clean signed permutation: %s" % ("YES" if perm else "NO"))
            if not perm:
                print("  -> AXIS_MAP CANNOT express this. It is a permutation-only")
                print("     construct; this needs a measured rotation matrix.")
            else:
                diag_signs = []
                for i in range(3):
                    d = int(np.argmax(np.abs(G[i])))
                    diag_signs.append((AXES[i], AXES[d], G[i][d]))
                print("  -> mapping: " + ", ".join(
                    "robot_%s = %s master_%s" % (a, "+" if g > 0 else "-", b)
                    for a, b, g in diag_signs))
            made += gain_heatmap(G, arm, base)

        cr, er = ranges(rows, arm)
        if cr is not None:
            print("\n--- RANGE / EFFECTIVE SCALE ---")
            for i, ax in enumerate(AXES):
                eff = er[i] / cr[i] if cr[i] > 1e-6 else float("nan")
                print("  %s: master cmd range %.3f m, robot range %.3f m, ratio %.2f"
                      % (ax, cr[i], er[i], eff))
            span = float(np.max(cr))
            if span > 1e-6:
                rec = 0.60 / span
                print("  recommended WORKSPACE_SCALE ~ %.2f "
                      "(to sweep ~0.60 m of robot workspace)" % rec)

        lg = lag(rows, arm)
        print("\n--- TRACKING LAG (cross-correlation) ---")
        for ax in AXES:
            v = lg.get(ax)
            print("  %s: %s" % (ax, "n/a" if v is None
                                else "%.0f ms (r=%.2f)" % (1000 * v[0], v[1])))

        rms, mx = tracking_error(rows, arm)
        print("\n--- TRACKING ERROR |cmd - actual| ---")
        print("  RMS %s   max %s"
              % ("n/a" if rms is None else "%.4f m" % rms,
                 "n/a" if mx is None else "%.4f m" % mx))

        cj = clutch_jumps(rows, arm)
        print("\n--- CLUTCH RE-ENGAGE JUMP (first 200 ms) ---")
        if not cj:
            print("  no re-engage transitions seen")
        for t, d in cj:
            print("  t=%.1fs  EE moved %.4f m%s"
                  % (t, d, "   <== JUMP" if d > 0.02 else ""))

        dp = dropouts(rows, arm)
        print("\n--- CHANNEL DROPOUTS (exact 0.0 samples) ---")
        tot = len(rows)
        for k, v in dp.items():
            print("  %-10s %6d / %d  (%.1f%%)" % (k, v, tot, 100.0 * v / tot))

        made += plots(rows, arm, base)

    print("\n" + "=" * 72)
    print("PLOTS")
    for p in made:
        print("  %s" % p)
    print("=" * 72)
    return made


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--arm", action="append", default=None)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    analyse(a.csv, a.arm or ["left", "right"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
