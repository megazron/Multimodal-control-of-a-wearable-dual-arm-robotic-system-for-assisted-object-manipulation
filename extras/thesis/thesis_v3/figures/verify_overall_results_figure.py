#!/usr/bin/env python3
"""Known-answer check on the overall-results box plot.

Checks, in order, and every one of them can fail:

  1. the two derived session files agree with each other, session by session
  2. the pooled statistics recomputed from the sessions reproduce the
     `pooled_stats` block those files already carry, which was written by a
     different script
  3. the NASA-TLX per-operator means reproduce `tlx_summary_means.csv`
  4. every operator has both conditions, so each box holds exactly five
     values and the pairing is complete
  5. the artists the figure actually DRAWS -- box edges, medians, whiskers,
     plotted points and pairing lines -- equal quartiles computed here from
     the data, independently of matplotlib's own boxplot call
  6. the printed mean in each panel equals the mean of the plotted points
  7. a hand-computed example with quartiles worked out on paper

    python3 extras/thesis/thesis_v3/figures/verify_overall_results_figure.py
    python3 extras/thesis/thesis_v3/figures/verify_overall_results_figure.py --break-it

`--break-it` perturbs one operator's value by 1 mm / 1 s after the data is
read and before it is drawn: a check that cannot fail on a broken input is
not a check, so the second command must report FAILED.
"""
import argparse
import collections
import csv
import json
import os
import statistics as st
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_overall_results_figure as M  # noqa: E402

DATA = M.DATA
TOL = 1e-9
FAILS = []
NCHECK = 0


def check(ok, label, detail=""):
    global NCHECK
    NCHECK += 1
    if not ok:
        FAILS.append("%s %s" % (label, detail))
    print("  [%s] %-58s %s" % ("PASS" if ok else "FAIL", label, detail))


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------- 1, 2, 3
def check_sources():
    print("1. the two derived session files agree")
    a = json.load(open(os.path.join(DATA, "study_summary.json")))["sessions"]
    b = json.load(open(os.path.join(DATA, "vr_study_sessions.json")))
    ka = {x["session"]: x for x in a}
    kb = {x["session"]: x for x in b}
    check(set(ka) == set(kb), "same sessions in both files", "%d each" % len(ka))
    fields = ["participant", "condition", "task", "duration_s", "ee_path_m", "clutch_engagements"]
    bad = [(s, f) for s in ka for f in fields if ka[s][f] != kb[s][f]]
    check(not bad, "every field agrees, session by session", "%d sessions x %d fields" % (len(ka), len(fields)))

    print("2. pooled statistics reproduce the stored block")
    stored = json.load(open(os.path.join(DATA, "study_summary.json")))["pooled_stats"]
    for cond in ("Direct", "Shared"):
        rows = [x for x in a if x["condition"] == cond]
        for field in ("duration_s", "ee_path_m", "clutch_engagements"):
            vals = [x[field] for x in rows]
            s = stored[cond][field]
            ok = (s["n"] == len(vals) and close(s["mean"], st.mean(vals), 5e-4)
                  and close(s["median"], st.median(vals), 5e-4))
            check(ok, "%s / %s" % (cond, field),
                  "n=%d mean=%.4f median=%.4f" % (len(vals), st.mean(vals), st.median(vals)))

    print("3. NASA-TLX means reproduce tlx_summary_means.csv")
    rows = list(csv.DictReader(open(os.path.join(DATA, "tlx_trials_long.csv"))))
    t = collections.defaultdict(list)
    for r in rows:
        t[(r["participant"], r["condition"])].append(float(r["rtlx"]))
    stored = {r["condition"]: float(r["rtlx_mean"])
              for r in csv.DictReader(open(os.path.join(DATA, "tlx_summary_means.csv")))
              if r["task_code"] == "ALL"}
    for cond in ("VR", "SharedAutonomy"):
        per_op = [st.mean(t[(p, cond)]) for p in M.OPS]
        check(close(st.mean(per_op), stored[cond], 5e-3), "%s mean of per-operator means" % cond,
              "%.3f against the stored %.2f" % (st.mean(per_op), stored[cond]))


# ------------------------------------------------------------------ 4, 5, 6
def quartiles(v):
    """Q1, median, Q3 and the whisker ends, computed here, not by matplotlib."""
    q1, q2, q3 = (float(x) for x in np.percentile(v, [25, 50, 75]))
    # the figure draws whiskers at the full range, not Tukey fences: with five
    # values per box a 1.5 IQR fence flags ordinary points as outliers
    return q1, q2, q3, float(min(v)), float(max(v))


def read_panel(ax):
    """What was actually drawn: box edges, median, whiskers, points, pair lines."""
    boxes, medians, whisk, pairs = [], [], [], []
    for p in ax.patches:
        ys = [pt[1] for pt in p.get_path().vertices]
        xs = [pt[0] for pt in p.get_path().vertices]
        boxes.append((round(float(np.mean(xs))), min(ys), max(ys)))
    for ln in ax.lines:
        x, y = ln.get_xdata(), ln.get_ydata()
        if len(x) != 2:
            continue
        if close(float(y[0]), float(y[1])) and abs(x[1] - x[0]) > 0.3 and ln.get_linewidth() > 1.4:
            medians.append((round(float(np.mean(x))), float(y[0])))       # median bar
        elif close(float(x[0]), float(x[1])):
            whisk.append((round(float(x[0])), float(y[0]), float(y[1])))  # whisker / cap
        elif abs(x[1] - x[0]) > 0.5:
            pairs.append((float(y[0]), float(y[1])))                      # operator pairing
    pts = {}
    for c in ax.collections:
        off = c.get_offsets()
        if len(off) == 0:
            continue
        side = round(float(np.mean([o[0] for o in off])))
        pts.setdefault(side, []).extend(float(o[1]) for o in off)
    return boxes, medians, whisk, pairs, pts


def check_drawing(data, truth, label):
    print("4. every operator contributes to both boxes")
    for key in ("time", "regrip", "rtlx", "path"):
        d, s = truth[key]["Direct"], truth[key]["Shared"]
        check(len(d) == 5 and len(s) == 5, "%s holds five values per box" % key,
              "n = %d and %d" % (len(d), len(s)))

    print("5. what the figure draws equals quartiles computed here%s" % label)
    for key, title, ylab, fmt, unit in M.PANELS:
        fig, ax = plt.subplots()
        # drawn from `data`; every expected value below comes from `truth`,
        # which is read again from the source files
        M.panel(ax, data[key]["Direct"], data[key]["Shared"], title, ylab, fmt, unit)
        d, s = truth[key]["Direct"], truth[key]["Shared"]
        boxes, medians, whisk, pairs, pts = read_panel(ax)
        ok = True
        detail = []
        for side, vals in ((0, d), (1, s)):
            q1, q2, q3, lo, hi = quartiles(vals)
            box = [b for b in boxes if b[0] == side]
            med = [m for m in medians if m[0] == side]
            wk = [w for w in whisk if w[0] == side]
            drawn_ends = sorted({round(y, 6) for w in wk for y in (w[1], w[2])})
            ok &= len(box) == 1 and close(box[0][1], q1, 1e-6) and close(box[0][2], q3, 1e-6)
            ok &= len(med) == 1 and close(med[0][1], q2, 1e-6)
            ok &= bool(drawn_ends) and close(min(drawn_ends), lo, 1e-6) and close(max(drawn_ends), hi, 1e-6)
            ok &= sorted(round(v, 6) for v in pts.get(side, [])) == sorted(round(v, 6) for v in vals)
            detail.append("Q1 %.3f med %.3f Q3 %.3f whisk %.3f-%.3f" % (q1, q2, q3, lo, hi))
        check(ok, "%s: boxes, medians, whiskers, points" % key, " | ".join(detail))
        # the grey lines must join the SAME operator's two values
        want = sorted((round(a, 6), round(b, 6)) for a, b in zip(d, s))
        got = sorted((round(a, 6), round(b, 6)) for a, b in pairs)
        check(want == got, "%s: pairing lines join the same operator" % key, "%d lines" % len(pairs))
        # 6. the printed mean equals the mean of the points
        txt = [t.get_text() for t in ax.texts if t.get_text().startswith("mean ")]
        raw = txt[0].replace("mean ", "") if txt else ""
        for ch in ("\u2193", "\u2191"):
            raw = raw.replace(ch, " ")
        if unit:
            raw = raw.replace(unit, " ")   # replace("") would split every character
        nums = [float(x) for x in raw.split()]
        ok = len(nums) == 2 and close(nums[0], float(fmt % st.mean(d)), 1e-9) \
            and close(nums[1], float(fmt % st.mean(s)), 1e-9)
        check(ok, "%s: printed mean equals the mean of the points" % key,
              "%s  (exact %.3f, %.3f)" % (txt[0] if txt else "MISSING", st.mean(d), st.mean(s)))
        plt.close(fig)


# ---------------------------------------------------------------------- 7
def check_known_answer():
    print("7. a hand-computed example")
    # v = 1,2,3,4,10 -> Q1 2, median 3, Q3 4, IQR 2, upper fence 7, so 10 is
    # beyond the fence and the upper whisker stops at 4; lower whisker at 1.
    v = [1.0, 2.0, 3.0, 4.0, 10.0]
    q1, q2, q3, lo, hi = quartiles(v)
    check((q1, q2, q3, lo, hi) == (2.0, 3.0, 4.0, 1.0, 10.0), "quartiles of 1,2,3,4,10 worked out on paper",
          "Q1 %.1f med %.1f Q3 %.1f whisk %.1f-%.1f" % (q1, q2, q3, lo, hi))
    fig, ax = plt.subplots()
    M.panel(ax, v, [5.0, 5.0, 5.0, 5.0, 5.0], "t", "y", "%.1f", "")
    boxes, medians, whisk, pairs, pts = read_panel(ax)
    b0 = [b for b in boxes if b[0] == 0][0]
    m0 = [m for m in medians if m[0] == 0][0]
    ends = sorted({round(y, 6) for w in whisk if w[0] == 0 for y in (w[1], w[2])})
    check(close(b0[1], 2.0) and close(b0[2], 4.0) and close(m0[1], 3.0)
          and close(min(ends), 1.0) and close(max(ends), 10.0),
          "the figure draws that example the same way",
          "box %.1f-%.1f median %.1f whisk %.1f-%.1f" % (b0[1], b0[2], m0[1], min(ends), max(ends)))
    check(sorted(round(x, 6) for x in pts[0]) == v, "all five points are plotted",
          "%s" % sorted(pts[0]))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--break-it", action="store_true",
                    help="perturb one drawn value so the checks must fail")
    a = ap.parse_args()
    check_sources()
    data = M.per_operator()          # what the figure is drawn from
    truth = M.per_operator()         # read again, as the expected values
    label = ""
    if a.break_it:
        data["time"]["Shared"][2] += 1.0      # 1 second
        data["path"]["Direct"][0] += 0.001    # 1 millimetre
        data["rtlx"]["Direct"][4] += 0.01
        label = "   [DELIBERATELY BROKEN: the drawing no longer matches the data]"
    check_drawing(data, truth, label)
    check_known_answer()
    print("\n%d checks, %d failed" % (NCHECK, len(FAILS)))
    for f in FAILS:
        print("  FAILED:", f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
