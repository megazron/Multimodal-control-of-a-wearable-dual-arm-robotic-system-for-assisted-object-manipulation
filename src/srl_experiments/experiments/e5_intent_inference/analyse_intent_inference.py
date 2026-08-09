#!/usr/bin/env python3
"""
analyse_intent_inference.py — E5, intent characterisation.

Inference accuracy, and the measured COST of a wrong inference, which is what sets the Part 5 thresholds.

The statistical test is the one PRE-REGISTERED in
docs/research/02_baseline_and_hypotheses.md. It is not chosen after looking at
the data, and where an assumption fails the fallback is also pre-registered.
"""
import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from srl_experiments.validity import partition, assert_analysable

EXPERIMENT = "E5 intent inference"

try:
    from scipy import stats as _st
except ImportError:          # analysis must still run without scipy
    _st = None


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    return rows


def valid(rows):
    """Invalid trials are EXCLUDED, and how many is always reported.

    Silently dropping them would hide a rig fault as a small n."""
    good = [r for r in rows if r.get("valid", "1") not in ("0", "")]
    return good, len(rows) - len(good)


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def by(rows, key):
    d = defaultdict(list)
    for r in rows:
        d[r.get(key, "")].append(r)
    return d


def describe(vals):
    v = np.array([x for x in vals if x == x])
    if not v.size:
        return dict(n=0, mean=float("nan"), sd=float("nan"), median=float("nan"))
    return dict(n=int(v.size), mean=float(v.mean()), sd=float(v.std(ddof=1)) if v.size > 1 else 0.0,
                median=float(np.median(v)))


def friedman_or_rm(groups, labels):
    """Pre-registered: repeated-measures ANOVA if normality holds, Friedman
    otherwise. With no scipy, report descriptives and say the test was skipped.
    """
    if _st is None:
        return dict(test="skipped", why="scipy not installed")
    arrs = [np.array([x for x in g if x == x]) for g in groups]
    n = min(len(a) for a in arrs) if arrs else 0
    if n < 3 or len(arrs) < 2:
        return dict(test="skipped", why="n=%d too small" % n)
    arrs = [a[:n] for a in arrs]
    normal = all(_st.shapiro(a).pvalue > 0.05 for a in arrs if 3 <= len(a) <= 5000)
    if normal:
        f, p = _st.f_oneway(*arrs)
        return dict(test="one-way RM ANOVA (approx)", statistic=float(f),
                    p=float(p), n_per_cell=n, labels=labels,
                    note="normality held (Shapiro-Wilk p>0.05)")
    s, p = _st.friedmanchisquare(*arrs)
    return dict(test="Friedman", statistic=float(s), p=float(p),
                n_per_cell=n, labels=labels,
                note="normality violated; pre-registered nonparametric fallback")


def holm(pvals):
    """DEPRECATED HERE. The shared implementation, with the post-hoc stage it
    was written for, lives in srl_experiments.posthoc. This local copy existed
    in five analysers and was called by none of them, so every E-series result
    was reported uncorrected."""
    from srl_experiments.posthoc import holm as _holm
    return _holm(pvals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("summary", nargs="+")
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    _all = load(a.summary)
    rows, _bad = partition(_all)
    # Excludes as pre-registered, and REFUSES to analyse a session that is
    # entirely or mostly invalid -- empty means read exactly like a null
    # finding, and only one of those is about the hypothesis.
    rows = assert_analysable(EXPERIMENT, rows, _bad)
    n_bad = len(_bad)
    print("E5 intent inference — %d valid trials (%d excluded)" % (len(rows), n_bad))
    print()
    acc = [num(r, "intent_correct_at_handover") for r in rows]
    acc = [x for x in acc if x == x]
    print("intent correct at handover: %d/%d = %.1f%%"
          % (int(sum(acc)), len(acc), 100 * (sum(acc) / len(acc)) if acc else 0.0))
    print("mean intent switches per trial: %.2f"
          % describe([num(r, "intent_switches") for r in rows])["mean"])
    print()
    print("COST OF A WRONG INFERENCE — the number that sets the threshold:")
    wrong = [r for r in rows if num(r, "intent_correct_at_handover") == 0]
    right = [r for r in rows if num(r, "intent_correct_at_handover") == 1]
    tw = describe([num(r, "completion_time_s") for r in wrong])
    tr = describe([num(r, "completion_time_s") for r in right])
    print("   correct inference   n=%3d  mean %.3f s" % (tr["n"], tr["mean"]))
    print("   wrong inference     n=%3d  mean %.3f s" % (tw["n"], tw["mean"]))
    if tr["n"] and tw["n"]:
        print("   COST                       %+.3f s per wrong ASSIST"
              % (tw["mean"] - tr["mean"]))
    print()
    print("by condition cell (spacing x distractors):")
    for c, g in sorted(by(rows, "condition").items()):
        ac = [num(r, "intent_correct_at_handover") for r in g]
        ac = [x for x in ac if x == x]
        print("   %-22s n=%3d  correct=%5.1f%%  cancels=%.2f"
              % (c, len(g), 100 * sum(ac) / len(ac) if ac else float("nan"),
                 describe([num(r, "assist_cancelled") for r in g])["mean"]))
    print()
    print("RECOMMENDED THRESHOLDS follow from the cost above: raise")
    print("p_threshold until the wrong-ASSIST rate costs less than the delay")
    print("that waiting for confidence costs. Write the winner into")
    print("srl_autonomy/handover_arbiter.py's defaults.")
    if a.plot and rows:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        cells = sorted(by(rows, "condition").items())
        labels = [c for c, _ in cells]
        ys = []
        for _, g in cells:
            ac = [num(r, "intent_correct_at_handover") for r in g]
            ac = [x for x in ac if x == x]
            ys.append(sum(ac) / len(ac) if ac else 0.0)
        ax.bar(range(len(ys)), ys)
        ax.set_xticks(range(len(ys)))
        ax.set_xticklabels(labels, rotation=60, ha="right", fontsize=6)
        ax.set_ylabel("intent correct at handover"); ax.set_ylim(0, 1.05)
        ax.set_title("E5 intent accuracy by cell")
        fig.tight_layout(); fig.savefig(a.plot, dpi=130)
        print("plot -> %s" % a.plot)

    # NO INFERENTIAL TEST IS RUN IN THIS ANALYSER. It defines
    # friedman_or_rm() and holm() and calls NEITHER, so everything printed
    # above is descriptive only. Saying so at the end is the minimum: a table
    # of condition means with no test beside it reads like a result, and a
    # reader has no way to tell that no hypothesis was evaluated.
    #
    # Wiring the omnibus in blind was declined deliberately -- the correct
    # test depends on this experiment's design and on which series pair with
    # which, and guessing that is how a wrong p-value enters a thesis.
    print()
    print("  !! DESCRIPTIVE ONLY: no inferential test was run. The omnibus "
          "and the")
    print("     Holm correction are defined in this file and called nowhere. "
          "Do NOT")
    print("     read the differences above as significant or as null.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
