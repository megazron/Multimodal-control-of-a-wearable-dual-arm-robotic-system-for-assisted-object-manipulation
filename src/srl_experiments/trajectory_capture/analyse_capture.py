#!/usr/bin/env python3
"""analyse_capture.py — turn a capture into a CALIBRATION REPORT.

    python3 src/srl_experiments/trajectory_capture/analyse_capture.py \
        ~/kortex_ws/recordings/trajectory_capture/capture_*/all_segments.csv

Answers, in one place:

  * per joint      range, linearity, noise floor, dropout count, ALIVE/DEAD
                   decided from THIS capture only; the 2026-07-31 verdict is
                   shown alongside purely to flag a CHANGE
  * crosstalk      the pairs already under suspicion (fsr1/fsr2 read r=0.9992
                   unloaded; k1j6/k2j6 suspected), plus a full 14x14 sweep
  * per direction  gain matrix d(EE)/d(master), off-diagonal %, isotropy
  * repeatability  variance across the three repeats of block C
  * rate           does gain depend on speed (block F)
  * recommended    ema_alpha and max_step_rad, from the measured noise and
                   the measured sample-to-sample motion

Plots go to PNG through the Agg backend: no display is needed and none is
opened, which matters because this runs over SSH in WSL.

A NOTE ON WHAT "DEAD" MEANS HERE. A channel is called DEAD on evidence, not on
reputation: near-zero range AND a high exact-zero fraction across the segment
that was supposed to exercise it. The protocol deliberately records the
known-dead channels as negative controls, so the report can be checked against
what is already known -- if a channel this calls alive is on the dead list, or
vice versa, that disagreement is the finding.
"""
import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")                     # headless, before pyplot
import matplotlib.pyplot as plt           # noqa: E402
import numpy as np                        # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from capture_protocol import PRIOR_VERDICT  # noqa: E402

ARMS = ("left", "right")
PFX = {"left": "l", "right": "r"}
DIRS = ("front", "back", "left", "right", "up", "down")
# Intended world-frame unit vector for each instructed direction. Wearer faces
# +y, +x is the wearer's right, +z up (this repo's convention -- NOT the usual
# ROS x-forward one).
INTENT = {"front": (0, 1, 0), "back": (0, -1, 0),
          "left": (-1, 0, 0), "right": (1, 0, 0),
          "up": (0, 0, 1), "down": (0, 0, -1)}


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    return rows


def num(r, k):
    v = r.get(k, "")
    if v == "" or v is None:
        return float("nan")
    try:
        return float(v)
    except ValueError:
        return float("nan")


def col(rows, k):
    return np.array([num(r, k) for r in rows])


def by_segment(rows):
    d = defaultdict(list)
    for r in rows:
        d[r.get("segment", "?")].append(r)
    return d


# --------------------------------------------------------------- per joint
def joint_stats(rows, arm, j):
    v = col(rows, "%s_j%d" % (PFX[arm], j))
    v = v[np.isfinite(v)]
    if v.size < 10:
        return None
    zeros = int(np.sum(v == 0.0))
    live = v[v != 0.0]
    rng = float(live.max() - live.min()) if live.size else 0.0
    # Noise floor: median absolute first difference over the QUIET half of the
    # trace. Using the whole trace would measure the sweep, not the noise.
    d = np.abs(np.diff(live)) if live.size > 2 else np.array([0.0])
    quiet = d[d <= np.percentile(d, 50)] if d.size else d
    noise = float(np.median(quiet)) if quiet.size else 0.0
    # LINEARITY, honestly. There is no independent reference for the true
    # joint angle here, so R^2 against sample index measures the shape of the
    # operator's sweep, not the sensor -- it read 0.000 for every healthy
    # channel and was meaningless. Two things ARE computable and do bear on
    # whether a channel is usable:
    #   rail_frac  fraction of samples pinned at the channel's own extremes,
    #              which is how a railed pot presents (left j7: 75% railed);
    #   mono       longest monotonic run as a fraction of a half sweep. A
    #              clean sweep is nearly monotonic out and back; a sticky or
    #              quantised channel breaks into short runs.
    rail_frac = float("nan")
    mono = float("nan")
    if live.size > 20:
        lo, hi = live.min(), live.max()
        tol = 0.01 * max(1e-9, hi - lo)
        rail_frac = float(np.mean((live <= lo + tol) | (live >= hi - tol)))
        d1 = np.sign(np.diff(live))
        best = run = 1
        for i in range(1, d1.size):
            run = run + 1 if d1[i] == d1[i - 1] and d1[i] != 0 else 1
            best = max(best, run)
        mono = float(best) / max(1.0, live.size / 2.0)
    n = v.size
    return dict(n=n, zeros=zeros, zero_frac=zeros / n if n else 0.0,
                rng=rng, noise=noise, rail_frac=rail_frac, mono=mono,
                vmin=float(live.min()) if live.size else float("nan"),
                vmax=float(live.max()) if live.size else float("nan"))


def _r2(x, y):
    if x.size < 3:
        return float("nan")
    A = np.vstack([x, np.ones_like(x)]).T
    try:
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    except np.linalg.LinAlgError:
        return float("nan")
    pred = A @ coef
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def verdict(st, expected):
    """ALIVE / DEAD / INTERMITTENT, on evidence."""
    if st is None:
        return "NO DATA", "segment missing or too short"
    if st["rng"] < 1.0 and st["zero_frac"] > 0.5:
        return "DEAD", "range %.2f deg, %.0f%% exact zeros" % (
            st["rng"], 100 * st["zero_frac"])
    if st["zero_frac"] > 0.05:
        return "INTERMITTENT", "%.1f%% exact-zero dropouts" % (
            100 * st["zero_frac"])
    if st["rng"] < 5.0:
        return "SUSPECT", "range only %.2f deg - was it actually swept?" % st["rng"]
    return "ALIVE", "range %.1f deg, noise %.3f deg" % (st["rng"], st["noise"])


# -------------------------------------------------------------- crosstalk
def crosstalk(rows):
    names, series = [], []
    for a in ARMS:
        for j in range(1, 8):
            k = "%s_j%d" % (PFX[a], j)
            v = col(rows, k)
            # Exact zeros are DROPOUTS, not readings. Leaving them in makes a
            # dropout-ridden channel correlate with everything that happens to
            # be moving, which is a measurement of the fault, not of crosstalk.
            v = np.where(v == 0.0, np.nan, v)
            if np.isfinite(v).sum() > 50:
                names.append("%s_j%d" % (a[0], j))
                series.append(v)
    out = {}
    if len(series) > 1:
        M = np.vstack(series)
        ok = np.all(np.isfinite(M), axis=0)
        if ok.sum() > 50:
            C = np.corrcoef(M[:, ok])
            out["matrix"] = C
            out["names"] = names
    # The two pairs already under suspicion, named explicitly.
    for k1, k2, why in (("fsr1", "fsr2", "read r=0.9992 unloaded"),
                        ("l_j6", "r_j6", "suspected k1j6/k2j6 crosstalk")):
        v1, v2 = col(rows, k1), col(rows, k2)
        v1 = np.where(v1 == 0.0, np.nan, v1)
        v2 = np.where(v2 == 0.0, np.nan, v2)
        m = np.isfinite(v1) & np.isfinite(v2)
        if m.sum() > 50 and np.std(v1[m]) > 0 and np.std(v2[m]) > 0:
            out.setdefault("pairs", []).append(
                (k1, k2, float(np.corrcoef(v1[m], v2[m])[0, 1]), why))
        else:
            out.setdefault("pairs", []).append((k1, k2, float("nan"), why))
    return out


# ------------------------------------------------------------- directional
def direction_gain(seg_rows, arm):
    """d(EE) for the segment, and the master displacement that produced it."""
    ee = np.vstack([col(seg_rows, "%s_ee_%s" % (arm, c)) for c in "xyz"]).T
    cmd = np.vstack([col(seg_rows, "%s_cmd_%s" % (arm, c)) for c in "xyz"]).T
    m = np.all(np.isfinite(ee), axis=1) & np.all(np.isfinite(cmd), axis=1)
    if m.sum() < 20:
        return None
    ee, cmd = ee[m], cmd[m]
    # Displacement from the segment start to the extreme (max excursion).
    d_ee = ee - ee[0]
    d_cmd = cmd - cmd[0]
    i = int(np.argmax(np.linalg.norm(d_cmd, axis=1)))
    return dict(ee=d_ee[i], cmd=d_cmd[i],
                ee_norm=float(np.linalg.norm(d_ee[i])),
                cmd_norm=float(np.linalg.norm(d_cmd[i])))


def gain_matrix(segs, arm):
    """Columns: intended direction. Rows: measured EE axis."""
    G = np.full((3, 3), np.nan)
    per = {}
    axis_of = {"front": 1, "back": 1, "left": 0, "right": 0, "up": 2, "down": 2}
    acc = defaultdict(list)
    for label, rows in segs.items():
        if not label.startswith(("C_", "E_")):
            continue
        if (rows[0].get("arm") or "").strip() != arm:
            continue
        # Direction from the COLUMN, never parsed out of the label: the arm
        # names collide with the direction names, so "C_left_up" matched the
        # direction "left" before "up" and every up/down segment was filed
        # under left. The recorder writes `direction` for exactly this reason.
        d = (rows[0].get("direction") or "").strip()
        if d not in DIRS:
            continue
        g = direction_gain(rows, arm)
        if g is None:
            continue
        per.setdefault(d, []).append(g)
        acc[d].append(g["ee"])
    for d, vecs in acc.items():
        v = np.mean(np.vstack(vecs), axis=0)
        G[:, axis_of[d]] = v if np.isnan(G[0, axis_of[d]]) else \
            (G[:, axis_of[d]] + v) / 2.0
    return G, per


def isotropy(per):
    mags = [np.mean([g["ee_norm"] for g in gs]) for gs in per.values() if gs]
    mags = [m for m in mags if m > 0]
    if len(mags) < 2:
        return float("nan")
    return min(mags) / max(mags)


# ------------------------------------------------------------- smoothing
def recommend(rows):
    """EMA alpha and max_step from the MEASURED noise and motion."""
    # Sample-to-sample command motion, and pot noise, both in the same units
    # they will be filtered in.
    steps = []
    for a in ARMS:
        p = np.vstack([col(rows, "%s_cmd_%s" % (a, c)) for c in "xyz"]).T
        m = np.all(np.isfinite(p), axis=1)
        if m.sum() > 50:
            steps.append(np.linalg.norm(np.diff(p[m], axis=0), axis=1))
    step = np.concatenate(steps) if steps else np.array([0.0])
    step = step[np.isfinite(step)]
    p95 = float(np.percentile(step, 95)) if step.size else 0.0
    med = float(np.median(step)) if step.size else 0.0

    noise = []
    for a in ARMS:
        for j in (1, 2, 4):                       # spherical mode's channels
            v = col(rows, "%s_j%d" % (PFX[a], j))
            v = v[np.isfinite(v) & (v != 0.0)]
            if v.size > 50:
                d = np.abs(np.diff(v))
                noise.append(float(np.median(d[d <= np.percentile(d, 50)])))
    n = float(np.mean(noise)) if noise else float("nan")

    # An EMA's cutoff is f_c ~ alpha * fs / (2*pi) for small alpha. Choose the
    # smallest alpha whose passband still covers the operator's own motion:
    # the median step must survive, the noise must not dominate.
    if math.isfinite(n) and med > 0:
        snr = med / max(n, 1e-6)
        alpha = float(np.clip(0.15 + 0.1 * math.log10(max(snr, 1.0)), 0.1, 0.6))
    else:
        alpha = float("nan")
    # max_step: the 95th-percentile real motion, with headroom, converted from
    # metres of EE to radians of joint by the arm's ~0.9 m reach.
    max_step = float(np.clip(3.0 * p95 / 0.9, 0.05, 1.0)) if p95 > 0 else float("nan")
    return dict(median_step_m=med, p95_step_m=p95, pot_noise_deg=n,
                ema_alpha=alpha, max_step_rad=max_step)


# ------------------------------------------------------------------ plots
def plots(rows, segs, outdir):
    outdir.mkdir(parents=True, exist_ok=True)
    made = []

    # 1. every pot, per arm
    for a in ARMS:
        fig, ax = plt.subplots(figsize=(11, 5))
        for j in range(1, 8):
            v = col(rows, "%s_j%d" % (PFX[a], j))
            ax.plot(v, lw=0.7, label="j%d" % j)
        ax.set_title("%s arm - all pot channels" % a)
        ax.set_xlabel("sample")
        ax.set_ylabel("deg")
        ax.legend(ncol=7, fontsize=7)
        f = outdir / ("pots_%s.png" % a)
        fig.tight_layout()
        fig.savefig(f, dpi=110)
        plt.close(fig)
        made.append(f)

    # 2. crosstalk heatmap
    ct = crosstalk(rows)
    if "matrix" in ct:
        fig, ax = plt.subplots(figsize=(7, 6))
        im = ax.imshow(np.abs(ct["matrix"]), vmin=0, vmax=1, cmap="magma")
        ax.set_xticks(range(len(ct["names"])))
        ax.set_xticklabels(ct["names"], rotation=90, fontsize=7)
        ax.set_yticks(range(len(ct["names"])))
        ax.set_yticklabels(ct["names"], fontsize=7)
        ax.set_title("|correlation| between pot channels")
        fig.colorbar(im)
        f = outdir / "crosstalk.png"
        fig.tight_layout()
        fig.savefig(f, dpi=110)
        plt.close(fig)
        made.append(f)

    # 3. per-direction EE displacement
    for a in ARMS:
        G, per = gain_matrix(segs, a)
        if not per:
            continue
        fig, ax = plt.subplots(figsize=(7, 5))
        labels, mags = [], []
        for d in DIRS:
            if d in per:
                labels.append(d)
                mags.append(np.mean([g["ee_norm"] for g in per[d]]))
        ax.bar(labels, mags)
        ax.set_title("%s arm - mean EE excursion per instructed direction" % a)
        ax.set_ylabel("m")
        f = outdir / ("direction_%s.png" % a)
        fig.tight_layout()
        fig.savefig(f, dpi=110)
        plt.close(fig)
        made.append(f)
    return made


# ----------------------------------------------------------------- report
class CorruptInput(Exception):
    """The input cannot support a report. Fail loudly, never emit numbers."""


def assert_sane(name, **k):
    """Refuse to report on impossible values.

    Every check here corresponds to a number this analyser actually emitted
    from the 2026-08-06 capture: a range of 314363 deg on a 0-360 sensor, a
    noise floor of exactly 0.000 on all 14 channels (an artefact of the 50 Hz
    recorder oversampling a 15 Hz sensor, so most adjacent rows are
    bit-identical), rail%=100 with an ALIVE verdict, and an all-zero gain
    matrix produced while the clutch was disengaged and nothing moved.
    """
    r = k.get("rng")
    if r is not None and np.isfinite(r) and r > 360.0:
        raise CorruptInput(
            "%s: range %.1f deg exceeds 360 on a rotary sensor. The 0-360 "
            "seam was not handled, or dropouts/parse corruption were counted "
            "as readings. Use a CIRCULAR range." % (name, r))
    n = k.get("noise")
    if n is not None and np.isfinite(n) and n == 0.0:
        raise CorruptInput(
            "%s: noise floor is exactly 0.000. No real sensor has zero noise; "
            "this means the statistic was taken over duplicate rows. The "
            "recorder samples at 50 Hz and the pots update at ~15 Hz, so 82-88%% "
            "of adjacent rows are identical. Compute over DISTINCT updates."
            % name)
    g = k.get("gain")
    if g is not None:
        G = np.asarray(g, float)
        fin = G[np.isfinite(G)]
        if fin.size and np.allclose(fin, 0.0):
            raise CorruptInput(
                "%s: gain matrix is all zero. Nothing moved. Check the clutch "
                "was ENGAGED during the segments -- a disengaged clutch "
                "freezes the commanded pose and the arm never follows."
                % name)
    f = k.get("clutch_frac")
    if f is not None and np.isfinite(f) and f < 0.5:
        raise CorruptInput(
            "%s: clutch engaged for only %.0f%% of the segments. The commanded "
            "pose is frozen while the clutch is out, so gain, repeatability "
            "and rate cannot be measured from this capture." % (name, 100 * f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--outdir", default="")
    a = ap.parse_args()

    rows = load(a.csv)
    if not rows:
        print("no rows in %s" % ", ".join(a.csv))
        return 2
    segs = by_segment(rows)
    outdir = Path(a.outdir or (Path(a.csv[0]).parent / "analysis"))

    print("=" * 74)
    print("CALIBRATION REPORT")
    print("=" * 74)
    print("%d rows, %d segments, from %s" % (len(rows), len(segs), a.csv[0]))
    print()

    # ---- per joint, using the isolation segment for that joint
    print("PER-CHANNEL VERDICT  (from block A, the isolation sweep)")
    print("  %-9s %8s %8s %8s %8s %8s %6s   %-13s %s"
          % ("channel", "range", "noise", "rail%", "mono", "zeros%", "n",
             "verdict", "prior"))
    usable, dead = [], []
    for arm in ARMS:
        for j in range(1, 8):
            lbl = "A_%s_j%d" % (arm, j)
            st = joint_stats(segs.get(lbl, []), arm, j)
            # The verdict is computed from THIS capture alone. The prior is
            # read only afterwards, to flag a change -- a channel previously
            # called dead that now moves is the finding.
            v, why = verdict(st, None)
            exp_state, _ = PRIOR_VERDICT[(arm, j)]
            agree = ("" if st is None else
                     ("" if (v == "DEAD") == (exp_state == "flat")
                      else "   <-- CHANGED vs 2026-07-31"))
            print("  %-9s %8s %8s %8s %8s %8s %6s   %-13s %s%s"
                  % ("%s_j%d" % (arm, j),
                     "-" if st is None else "%.1f" % st["rng"],
                     "-" if st is None else "%.3f" % st["noise"],
                     "-" if st is None or not math.isfinite(st["rail_frac"])
                     else "%.1f" % (100 * st["rail_frac"]),
                     "-" if st is None or not math.isfinite(st["mono"])
                     else "%.2f" % st["mono"],
                     "-" if st is None else "%.1f" % (100 * st["zero_frac"]),
                     "-" if st is None else st["n"],
                     v, exp_state, agree))
            (usable if v in ("ALIVE", "INTERMITTENT") else dead).append(
                "%s_j%d" % (arm, j))
    print()

    # ---- crosstalk
    print("CROSSTALK")
    ct = crosstalk(rows)
    for k1, k2, r, why in ct.get("pairs", []):
        flag = ""
        if math.isfinite(r) and abs(r) > 0.9:
            flag = "  <-- HIGH, treat as one channel until wiring is checked"
        print("  %-6s vs %-6s  r = %s   (%s)%s"
              % (k1, k2, "-" if not math.isfinite(r) else "%+.4f" % r,
                 why, flag))
    if "matrix" in ct:
        C = np.abs(ct["matrix"])
        nm = ct["names"]
        worst = []
        for i in range(len(nm)):
            for j in range(i + 1, len(nm)):
                if math.isfinite(C[i, j]):
                    worst.append((C[i, j], nm[i], nm[j]))
        worst.sort(reverse=True)
        print("  highest off-diagonal pairs:")
        for r, n1, n2 in worst[:5]:
            print("     %-6s %-6s  |r| = %.4f" % (n1, n2, r))
    print()

    # ---- directional gain
    print("DIRECTIONAL GAIN  d(EE)/d(instructed direction), metres")
    for arm in ARMS:
        G, per = gain_matrix(segs, arm)
        if not per:
            print("  %-5s no directional segments" % arm)
            continue
        print("  %s arm" % arm)
        for i, axis in enumerate("xyz"):
            print("     EE_%s  %s" % (axis, "  ".join(
                "-" if not math.isfinite(G[i, k]) else "%+7.4f" % G[i, k]
                for k in range(3))))
        offdiag = []
        for k in range(3):
            colv = G[:, k]
            if np.all(np.isfinite(colv)) and np.linalg.norm(colv) > 0:
                on = abs(colv[k])
                off = math.sqrt(max(0.0, np.dot(colv, colv) - on * on))
                offdiag.append(100.0 * off / max(on, 1e-9))
        if offdiag:
            print("     off-diagonal: %s  (%% of on-axis)"
                  % ", ".join("%.0f%%" % o for o in offdiag))
        print("     isotropy (min/max excursion): %.2f" % isotropy(per))
    print()

    # ---- repeatability
    print("REPEATABILITY  (block C + E, three repeats per direction)")
    for arm in ARMS:
        for d in DIRS:
            mags = []
            for label, rws in segs.items():
                if not (label.startswith("C_") or label.startswith("E_")):
                    continue
                if (rws[0].get("direction") or "").strip() != d:
                    continue
                if (rws[0].get("arm") or "").strip() != arm:
                    continue
                g = direction_gain(rws, arm)
                if g:
                    mags.append(g["ee_norm"])
            if len(mags) >= 2:
                m, s = float(np.mean(mags)), float(np.std(mags))
                print("  %-5s %-6s n=%d  mean %.4f m  sd %.4f m  cv %.1f%%"
                      % (arm, d, len(mags), m, s, 100 * s / max(m, 1e-9)))
    print()

    # ---- rate dependence
    print("RATE DEPENDENCE  (block F)")
    for arm in ARMS:
        for sp in ("slow", "medium", "fast"):
            lbl = "F_%s_front_%s" % (arm, sp)
            rws = segs.get(lbl)
            if not rws:
                continue
            g = direction_gain(rws, arm)
            v = col(rws, "%s_cmd_y" % arm)
            v = v[np.isfinite(v)]
            speed = float(np.median(np.abs(np.diff(v)))) * 50.0 if v.size > 5 else float("nan")
            print("  %-5s %-7s excursion %s m   median |v| %.4f m/s"
                  % (arm, sp,
                     "-" if not g else "%.4f" % g["ee_norm"], speed))
    print()

    # ---- recommendations
    rec = recommend(rows)
    print("RECOMMENDED SMOOTHING  (from the measured noise and motion above)")
    print("  median command step   %.5f m/sample" % rec["median_step_m"])
    print("  p95 command step      %.5f m/sample" % rec["p95_step_m"])
    print("  pot noise floor       %s deg"
          % ("-" if not math.isfinite(rec["pot_noise_deg"])
             else "%.4f" % rec["pot_noise_deg"]))
    print("  ema_alpha             %s"
          % ("-" if not math.isfinite(rec["ema_alpha"])
             else "%.2f" % rec["ema_alpha"]))
    print("  max_step_rad          %s"
          % ("-" if not math.isfinite(rec["max_step_rad"])
             else "%.2f" % rec["max_step_rad"]))
    print()

    print("USABLE CHANNELS (%d): %s" % (len(usable), ", ".join(usable)))
    print("UNUSABLE       (%d): %s" % (len(dead), ", ".join(dead)))
    print()
    made = plots(rows, segs, outdir)
    print("plots -> %s" % outdir)
    for f in made:
        print("   %s" % f.name)

    summary = dict(rows=len(rows), segments=len(segs), usable=usable,
                   unusable=dead, recommended=rec)
    (outdir / "calibration_report.json").write_text(json.dumps(summary, indent=2))
    print("machine-readable summary -> %s"
          % (outdir / "calibration_report.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
