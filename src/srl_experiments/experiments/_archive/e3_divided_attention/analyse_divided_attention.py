#!/usr/bin/env python3
"""
analyse_divided_attention.py — E3, divided attention.

The crossed design, and the INTERACTION term that is the result of record.

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

EXPERIMENT = "E3 divided attention"

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
    print("E3 divided attention — %d valid trials (%d excluded)" % (len(rows), n_bad))
    cells = defaultdict(list)
    for r in rows:
        cond = r.get("condition", "")
        base, _, att = cond.partition("|")
        cells[(base, att or "single_task")].append(r)
    conds = sorted({k[0] for k in cells})
    atts = sorted({k[1] for k in cells})
    print()
    print("mean completion time (s), condition x attention")
    print("%-12s %s" % ("", "".join("%14s" % x for x in atts)))
    for c in conds:
        line = "%-12s" % c
        for at in atts:
            line += "%14.3f" % describe(
                [num(r, "completion_time_s") for r in cells.get((c, at), [])])["mean"]
        print(line)
    print()
    print("THE RESULT OF INTEREST IS THE INTERACTION: does autonomy help MORE")
    print("when attention is divided? Reported as the difference of differences.")
    for c in conds:
        if c == "direct":
            continue
        d = {}
        for at in atts:
            base = describe([num(r, "completion_time_s")
                             for r in cells.get(("direct", at), [])])["mean"]
            cur = describe([num(r, "completion_time_s")
                            for r in cells.get((c, at), [])])["mean"]
            d[at] = base - cur
        if len(atts) == 2:
            print("   %-10s benefit single=%.3f s  dual=%.3f s  INTERACTION=%+.3f s"
                  % (c, d[atts[0]], d[atts[1]], d[atts[1]] - d[atts[0]]))
    print()
    print("Pre-registered test: 2-way repeated-measures ANOVA (autonomy x attention),")
    print("with the interaction term as the hypothesis of record; Aligned Rank")
    print("Transform if residuals are non-normal.")
    if a.plot and conds:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        x = np.arange(len(conds)); w = 0.8 / max(1, len(atts))
        for i, at in enumerate(atts):
            ys = [describe([num(r, "completion_time_s")
                            for r in cells.get((c, at), [])])["mean"] for c in conds]
            ax.bar(x + i * w, ys, w, label=at)
        ax.set_xticks(x + w * (len(atts) - 1) / 2); ax.set_xticklabels(conds)
        ax.set_ylabel("completion time (s)"); ax.legend()
        ax.set_title("E3 autonomy x attention")
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
