#!/usr/bin/env python3
"""Post-hoc pairwise comparisons with the pre-registered Holm correction.

WHY THIS EXISTS. Every E-series analyser defined `holm()` and none of them
called it. The correction was not merely unused: there was nothing to correct,
because each analyser ran ONE omnibus test (Friedman or repeated-measures
ANOVA) and stopped there. An omnibus result says that something differs across
conditions. It does not say which pair, and with three or more conditions the
pair is usually the question.

So the gap was not a missing call. It was the missing post-hoc stage that
`holm` was written for. Adding it in one shared place also removes the five
identical copies, which could otherwise drift apart.

WHY HOLM AND NOT BONFERRONI. Holm-Bonferroni is uniformly more powerful and
controls the same family-wise error rate, so there is no reason to prefer
plain Bonferroni. It was pre-registered.

WHY PAIRED TESTS. The design is within subjects: the same dyad performs every
condition. Treating the conditions as independent samples would discard the
pairing and inflate the variance, which loses exactly the sensitivity the
within-subjects design was chosen to buy.
"""
import math

import numpy as np


def holm(pvals):
    """Holm-Bonferroni step-down adjusted p-values, in the input order.

    Adjusted values are made monotone non-decreasing in rank order, which is
    what makes the procedure valid: without that a later, larger raw p can
    otherwise emerge with a smaller adjusted value.
    """
    p = np.asarray(list(pvals), dtype=float)
    if p.size == 0:
        raise ValueError("holm() over an empty family. A correction with "
                         "nothing to correct is a silent no-op, and this "
                         "project has been bitten by exactly that shape.")
    idx = np.argsort(p)
    out = np.empty(p.size)
    m = p.size
    prev = 0.0
    for rank, i in enumerate(idx):
        v = min(1.0, (m - rank) * p[i])
        prev = max(prev, v)
        out[i] = prev
    return out


def _paired(a, b):
    """Drop pairs where either side is missing, and keep the pairing."""
    A, B = np.asarray(a, float), np.asarray(b, float)
    n = min(A.size, B.size)
    A, B = A[:n], B[:n]
    keep = ~(np.isnan(A) | np.isnan(B))
    return A[keep], B[keep]


def pairwise(series, labels, alpha=0.05):
    """All pairs, paired Wilcoxon, Holm-corrected.

    `series` is a list of per-condition sequences in `labels` order. Returns a
    list of dicts, each carrying the RAW and the ADJUSTED p so a reader can
    see the correction rather than take it on trust, plus the effect size.

    A pair with too few usable observations is reported with p=None and
    EXCLUDED from the correction family, because including it would inflate
    the family size and weaken every other comparison on the strength of a
    test that was never run.
    """
    try:
        from scipy import stats as st
    except ImportError:                                      # noqa: BLE001
        return [dict(error="scipy unavailable; no post-hoc computed")]

    rows, family = [], []
    for i in range(len(labels)):
        for j in range(i + 1, len(labels)):
            A, B = _paired(series[i], series[j])
            row = dict(a=labels[i], b=labels[j], n=int(A.size))
            if A.size < 6 or np.allclose(A, B):
                row.update(p_raw=None, p_adj=None, effect_r=None,
                           note="too few paired observations to test"
                                if A.size < 6 else "identical series")
            else:
                s, p = st.wilcoxon(A, B)
                # Matched-pairs rank-biserial effect size: interpretable
                # without assuming normality, unlike Cohen's d here.
                z = abs(st.norm.ppf(p / 2.0)) if 0 < p < 1 else 0.0
                row.update(p_raw=float(p), statistic=float(s),
                           effect_r=float(z / math.sqrt(A.size)) if A.size else None,
                           median_diff=float(np.median(A - B)))
                family.append(row)
            rows.append(row)

    if family:
        adj = holm([r["p_raw"] for r in family])
        for r, q in zip(family, adj):
            r["p_adj"] = float(q)
            r["significant"] = bool(q < alpha)
    return rows


def summarise(rows, alpha=0.05):
    """One line per pair, showing raw and adjusted side by side."""
    out = []
    tested = [r for r in rows if r.get("p_raw") is not None]
    out.append("  post-hoc: %d pair(s) tested, Holm-corrected over that family"
               % len(tested))
    for r in rows:
        if r.get("p_raw") is None:
            out.append("    %-10s vs %-10s  n=%-3d  not tested: %s"
                       % (r["a"], r["b"], r["n"], r.get("note", "")))
            continue
        out.append("    %-10s vs %-10s  n=%-3d  p=%.4f  p_holm=%.4f  r=%.2f  %s"
                   % (r["a"], r["b"], r["n"], r["p_raw"], r["p_adj"],
                      r["effect_r"] or 0.0,
                      "SIGNIFICANT" if r["significant"] else ""))
    if not tested:
        out.append("    NOTHING WAS TESTED. Do not read this as 'no "
                   "differences'; it means no pair had enough paired data.")
    return "\n".join(out)
