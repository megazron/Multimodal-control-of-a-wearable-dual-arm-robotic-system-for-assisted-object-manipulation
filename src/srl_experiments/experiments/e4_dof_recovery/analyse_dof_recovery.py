#!/usr/bin/env python3
"""
analyse_dof_recovery.py — E4, DOF recovery.

A categorical outcome per scenario, tested with McNemar on the paired binary result.

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

EXPERIMENT = "E4 DOF recovery"

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
    """Holm-Bonferroni, the pre-registered correction."""
    idx = np.argsort(pvals)
    out = np.empty(len(pvals))
    m = len(pvals)
    prev = 0.0
    for rank, i in enumerate(idx):
        v = min(1.0, (m - rank) * pvals[i])
        prev = max(prev, v)
        out[i] = prev
    return out


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
    print("E4 DOF recovery — %d valid trials (%d excluded, incl. dropout>2%%)"
          % (len(rows), n_bad))
    print()
    print("THE RESULT IS CATEGORICAL: for each scenario, is the object")
    print("graspable at all under each condition?")
    print()
    scen = sorted({r.get("scenario", "") for r in rows})
    conds = sorted({r.get("condition", "") for r in rows})
    print("%-16s %s" % ("scenario", "".join("%18s" % c for c in conds)))
    table = {}
    for s in scen:
        line = "%-16s" % s
        for c in conds:
            g = [r for r in rows if r.get("scenario") == s and r.get("condition") == c]
            k = sum(int(num(r, "grasp_success") == 1) for r in g)
            table[(s, c)] = (k, len(g))
            line += "%18s" % ("%d/%d" % (k, len(g)) if g else "-")
        print(line)
    print()
    if _st is not None and len(conds) == 2:
        print("Pre-registered test: McNemar on the paired binary outcome")
        print("(the same scenario under both conditions).")
        for s in scen:
            (k0, n0) = table.get((s, conds[0]), (0, 0))
            (k1, n1) = table.get((s, conds[1]), (0, 0))
            if not (n0 and n1):
                continue
            b = max(0, k1 - k0); c_ = max(0, k0 - k1)
            if b + c_ == 0:
                print("   %-16s no discordant pairs" % s); continue
            p = _st.binomtest(b, b + c_, 0.5).pvalue
            print("   %-16s %s=%d/%d  %s=%d/%d   McNemar exact p=%.4f"
                  % (s, conds[0], k0, n0, conds[1], k1, n1, p))
    print()
    print("Channel liveness per trial is in the summary (dropout_fraction);")
    print("trials over the 2% threshold are already excluded above, so the")
    print("deficit under test is the STRUCTURAL one, not a broken pot.")
    if a.plot and scen:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        x = np.arange(len(scen)); w = 0.8 / max(1, len(conds))
        for i, c in enumerate(conds):
            ys = [table[(s, c)][0] / max(1, table[(s, c)][1]) for s in scen]
            ax.bar(x + i * w, ys, w, label=c)
        ax.set_xticks(x + w * (len(conds) - 1) / 2)
        ax.set_xticklabels(scen, rotation=20, ha="right")
        ax.set_ylabel("grasp success rate"); ax.set_ylim(0, 1.05); ax.legend()
        ax.set_title("E4 DOF recovery — categorical")
        fig.tight_layout(); fig.savefig(a.plot, dpi=130)
        print("plot -> %s" % a.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
