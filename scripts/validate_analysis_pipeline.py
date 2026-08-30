#!/usr/bin/env python3
"""DOES THE PRE-REGISTERED ANALYSIS RECOVER AN EFFECT IT IS GIVEN?

    python3 scripts/validate_analysis_pipeline.py
    python3 scripts/validate_analysis_pipeline.py --self-test

WHY THIS EXISTS.  The study of `docs/research/` is powered for a
within-subjects contrast of d = 0.73 at n = 16 dyads, and that number came
out of a power calculation rather than out of the analysis that will
actually be run.  Those are not the same thing: the pipeline applies a
Holm-Bonferroni correction across a family of pairwise comparisons, and a
correction costs power that a two-sample formula does not know about.

So this program constructs data whose answer is known -- a within-subjects
data set with an effect of exactly the declared size, and a null data set
with no effect at all -- pushes both through the SAME code the study will
use, and reports how often the effect is recovered and how often the null
is falsely rejected.  Both directions are checked, because a test that
always rejects has perfect power and is worthless.

This is the one place in this project where synthetic data is legitimate:
the ground truth is CONSTRUCTED (a number added to a sample), not RENDERED.
Nothing here is a claim about the platform.  It is a claim about the
analysis.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments"))

from srl_experiments.posthoc import holm            # noqa: E402

ALPHA = 0.05
N_DYADS = 16
EFFECT_D = 0.73          # the effect the design is powered for
N_TASKS = 5              # T1..T5; T2 carries no mode contrast, see the thesis
SEED = 20260830

#: WITHIN-CONDITION NOISE, CHOSEN SO THAT d MEANS WHAT THE DESIGN SAYS.
#: In a within-subjects design the effect size that the power calculation
#: refers to is d_z, the standardised mean of the PAIRED DIFFERENCES. Two
#: independent residuals of unit standard deviation give a difference of
#: standard deviation sqrt(2), which would silently deflate every effect by
#: a factor of 1.41 and make the pipeline look 30 percentage points weaker
#: than it is. Setting each residual to 1/sqrt(2) makes the difference unit
#: variance, so d in this file is d_z.
SIGMA_W = 1.0 / math.sqrt(2.0)


def paired_t(a, b):
    """Two-sided paired t-test. Written out so the pipeline has no runtime
    dependency the study machine might not have, and so the degrees of
    freedom are visible rather than implied."""
    d = np.asarray(a, float) - np.asarray(b, float)
    n = d.size
    if n < 2:
        return float("nan"), float("nan"), n - 1
    sd = d.std(ddof=1)
    if sd == 0:
        return float("inf") if d.mean() else 0.0, 0.0, n - 1
    t = d.mean() / (sd / math.sqrt(n))
    df = n - 1
    return t, _t_sf(abs(t), df) * 2.0, df


def _t_sf(t, df):
    """Upper tail of Student's t, by the incomplete beta function.

    Implemented here rather than imported so that the p-values in the thesis
    do not depend on which SciPy happens to be installed on the machine that
    runs the study.  Checked against known values in --self-test.
    """
    x = df / (df + t * t)
    return 0.5 * _betainc(0.5 * df, 0.5, x)


def _betainc(a, b, x):
    """Regularised incomplete beta, by the continued fraction of Lentz."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbeta = (math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
             + a * math.log(x) + b * math.log1p(-x))
    if x < (a + 1.0) / (a + b + 2.0):
        return math.exp(lbeta) * _betacf(a, b, x) / a
    return 1.0 - math.exp(lbeta) * _betacf(b, a, 1.0 - x) / b


def _betacf(a, b, x, itmax=200, eps=3e-16):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    if abs(d) < 1e-300:
        d = 1e-300
    d = 1.0 / d
    h = d
    for m in range(1, itmax + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        c = 1.0 + aa / c
        if abs(d) < 1e-300:
            d = 1e-300
        if abs(c) < 1e-300:
            c = 1e-300
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        c = 1.0 + aa / c
        if abs(d) < 1e-300:
            d = 1e-300
        if abs(c) < 1e-300:
            c = 1e-300
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < eps:
            break
    return h


def one_study(rng, d, n=N_DYADS, k=N_TASKS):
    """One simulated study: k paired contrasts, each with true effect d.

    The dyad is the unit. A per-dyad offset is drawn and added to BOTH
    conditions, which is what makes the design within subjects: it is the
    between-dyad variance the pairing removes, and omitting it would
    overstate the power of every within-subjects design ever simulated.
    """
    dyad = rng.normal(0.0, 1.0, size=(n, 1))       # between-dyad variance
    a = dyad + rng.normal(0.0, SIGMA_W, size=(n, k))
    b = dyad + rng.normal(0.0, SIGMA_W, size=(n, k)) + d
    ps = [paired_t(b[:, j], a[:, j])[1] for j in range(k)]
    return holm(ps)


def run(trials=4000, seed=SEED):
    rng = np.random.RandomState(seed)
    out = {}
    for label, d in (("effect", EFFECT_D), ("null", 0.0)):
        rng2 = np.random.RandomState(seed + (0 if d else 1))
        any_hit, per_contrast, uncorrected = 0, np.zeros(N_TASKS), 0
        for _ in range(trials):
            dyad = rng2.normal(0.0, 1.0, size=(N_DYADS, 1))
            a = dyad + rng2.normal(0.0, SIGMA_W, size=(N_DYADS, N_TASKS))
            b = dyad + rng2.normal(0.0, SIGMA_W, size=(N_DYADS, N_TASKS)) + d
            raw = [paired_t(b[:, j], a[:, j])[1] for j in range(N_TASKS)]
            adj = holm(raw)
            hits = np.array(adj) < ALPHA
            any_hit += bool(hits.any())
            per_contrast += hits
            uncorrected += bool(np.array(raw)[0] < ALPHA)
        out[label] = dict(
            trials=trials,
            true_effect_d=d,
            family_wise_rate=any_hit / trials,
            per_contrast_rate=float(per_contrast.mean() / trials),
            single_contrast_uncorrected=uncorrected / trials,
        )
    del rng
    return out


def power_curve(ns=(8, 10, 12, 14, 16, 18, 20, 24, 28, 32), trials=6000,
                seed=SEED + 99):
    """Power against the number of dyads, corrected and uncorrected.

    Reported as a curve rather than as one number because the single number
    the design carries -- 80 per cent at 16 dyads -- is the UNCORRECTED
    power of one contrast, and the analysis that will actually be run
    corrects across five.
    """
    rows = []
    for n in ns:
        rng = np.random.RandomState(seed + n)
        hit_c = hit_u = hit_f = 0
        for _ in range(trials):
            dyad = rng.normal(0.0, 1.0, size=(n, 1))
            a = dyad + rng.normal(0.0, SIGMA_W, size=(n, N_TASKS))
            b = dyad + rng.normal(0.0, SIGMA_W, size=(n, N_TASKS)) + EFFECT_D
            raw = [paired_t(b[:, j], a[:, j])[1] for j in range(N_TASKS)]
            adj = holm(raw)
            hit_u += sum(1 for q in raw if q < ALPHA)
            hit_c += sum(1 for q in adj if q < ALPHA)
            hit_f += bool(min(adj) < ALPHA)
        rows.append(dict(dyads=n,
                         uncorrected=hit_u / (trials * N_TASKS),
                         corrected=hit_c / (trials * N_TASKS),
                         family_wise=hit_f / trials))
    return rows


def self_test():
    """Known answers. No study, no data, no random numbers where a closed
    form exists."""
    ok = fail = 0

    def check(name, cond, got=""):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  ok    %s %s" % (name, got))
        else:
            fail += 1
            print("  FAIL  %s %s" % (name, got))

    # 1. The t distribution, against values that can be looked up.
    check("t(0, df=10) has p = 1", abs(_t_sf(0.0, 10) * 2 - 1.0) < 1e-12)
    check("t = 2.228, df = 10 is the 5 per cent two-sided point",
          abs(_t_sf(2.228, 10) * 2 - 0.05) < 5e-4,
          "p = %.5f" % (_t_sf(2.228, 10) * 2))
    check("t = 2.131, df = 15 is the 5 per cent two-sided point",
          abs(_t_sf(2.131, 15) * 2 - 0.05) < 5e-4,
          "p = %.5f" % (_t_sf(2.131, 15) * 2))

    # 2. A paired test on a constructed difference of known size.
    x = np.arange(16, dtype=float)
    t, p, df = paired_t(x + 1.0, x)          # difference exactly 1, sd 0
    check("a constant difference gives df = n-1", df == 15)
    check("a difference with no variance gives p = 0", p == 0.0)

    # 3. Holm must not be an identity, and must not over-correct.
    adj = holm([0.01, 0.02, 0.03])
    check("Holm scales the smallest p by the family size",
          abs(adj[0] - 0.03) < 1e-12, "%.4f" % adj[0])
    check("Holm is monotone", adj[0] <= adj[1] <= adj[2])

    # 4. THE CHECK THAT CAN FAIL. A null study must be rejected at about
    #    alpha and NOT more often; a pipeline that always rejects would
    #    pass every power check and be worthless.
    r = run(trials=1500, seed=7)
    fw = r["null"]["family_wise_rate"]
    check("a null study is rejected at about alpha, family-wise",
          fw < 0.09, "%.3f" % fw)
    pw = r["effect"]["family_wise_rate"]
    check("a study with the designed effect is recovered family-wise",
          pw > 0.90, "%.3f" % pw)
    un = r["effect"]["single_contrast_uncorrected"]
    check("one uncorrected contrast reproduces the power calculation",
          0.72 < un < 0.86, "%.3f" % un)
    pc = r["effect"]["per_contrast_rate"]
    check("the correction costs power, and the cost is reported",
          pc < un, "%.3f corrected vs %.3f uncorrected" % (pc, un))

    print("\n%d checks, %d failed" % (ok + fail, fail))
    return 1 if fail else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--trials", type=int, default=4000)
    ap.add_argument("--out", default=os.path.join(
        WS, "recordings", "baselines", "analysis_pipeline_validation.json"))
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()

    res = run(a.trials)
    res["power_curve"] = power_curve()
    res["design"] = dict(dyads=N_DYADS, contrasts=N_TASKS, alpha=ALPHA,
                         effect_d=EFFECT_D,
                         correction="Holm-Bonferroni, pre-registered")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(res, fh, indent=2)
    print(json.dumps(res, indent=2))
    print("\nwritten to %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
