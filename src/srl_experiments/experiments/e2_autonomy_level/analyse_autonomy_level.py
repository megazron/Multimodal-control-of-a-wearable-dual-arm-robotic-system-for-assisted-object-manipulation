#!/usr/bin/env python3
"""
analyse_autonomy_level.py — E2, the core comparison.

Per-condition descriptives and the pre-registered omnibus test on completion time.

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

EXPERIMENT = "E2 autonomy level"

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
    print("E2 autonomy level — %d valid trials (%d excluded as invalid)"
          % (len(rows), n_bad))
    groups = by(rows, "condition")
    order = [c for c in ("direct", "shared", "full_auto") if c in groups]
    print()
    print("%-12s %5s %10s %10s %10s %10s %10s"
          % ("condition", "n", "time_s", "success", "path_ratio", "interv.", "err_m"))
    series = {}
    for c in order:
        g = groups[c]
        t = [num(r, "completion_time_s") for r in g]
        series[c] = t
        print("%-12s %5d %10.3f %10.3f %10.3f %10.3f %10.4f"
              % (c, len(g), describe(t)["mean"],
                 describe([num(r, "grasp_success") for r in g])["mean"],
                 describe([num(r, "path_ratio") for r in g])["mean"],
                 describe([num(r, "intervention_count") for r in g])["mean"],
                 describe([num(r, "positioning_error_m") for r in g])["mean"]))
    print()
    print("H2.1 completion time differs by autonomy level (pre-registered):")
    print("   ", json.dumps(friedman_or_rm([series[c] for c in order], order)))
    # POST-HOC, which is what holm() was written for. The omnibus
    # above says something differs across conditions; it does not
    # say WHICH PAIR, and with three conditions the pair is the
    # question. Without this stage the correction had nothing to
    # correct, which is why it sat uncalled in five analysers.
    from srl_experiments.posthoc import pairwise, summarise
    print(summarise(pairwise([series[c] for c in order], order)))
    if a.plot and order:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(9, 4))
        ax[0].boxplot([[x for x in series[c] if x == x] for c in order], labels=order)
        ax[0].set_ylabel("completion time (s)"); ax[0].set_title("E2 time by condition")
        succ = [describe([num(r, "grasp_success") for r in groups[c]])["mean"] for c in order]
        ax[1].bar(order, succ); ax[1].set_ylim(0, 1.05)
        ax[1].set_ylabel("grasp success rate"); ax[1].set_title("E2 success by condition")
        fig.tight_layout(); fig.savefig(a.plot, dpi=130)
        print("plot -> %s" % a.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
