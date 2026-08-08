#!/usr/bin/env python3
"""T1 analysis — Fitts throughput, one arm vs two. The attention bottleneck.

    python3 analyse_t1.py <summary.csv> ...

The headline is the RATIO of bimanual summed throughput to single-arm
throughput. 1.0 means two arms are free; 0.5 means the operator is strictly
serialising. Stagger is reported alongside because a large stagger with a
ratio near 0.5 is direct evidence of time-slicing rather than parallelism.
"""
import argparse, csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from srl_experiments.validity import partition, assert_analysable  # noqa: E402

EXPERIMENT = "T1 bimanual reach"
CONDITIONS = ("direct", "assisted", "shared")


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def fit_fitts(rows, arm):
    """MT = a + b.ID ; throughput = 1/b (bits/s)."""
    ids, mts = [], []
    for r in rows:
        i, m = num(r, "id_bits"), num(r, "%s_mt_s" % arm)
        if i == i and m == m and m > 0:
            ids.append(i); mts.append(m)
    if len(ids) < 3:
        return None
    A = np.vstack([np.array(ids), np.ones(len(ids))]).T
    (b, a), *_ = np.linalg.lstsq(A, np.array(mts), rcond=None)
    pred = A @ np.array([b, a])
    ss = float(np.sum((np.array(mts) - pred) ** 2))
    st = float(np.sum((np.array(mts) - np.mean(mts)) ** 2))
    return dict(a=float(a), b=float(b), tp=(1.0 / b) if b > 1e-9 else float("nan"),
                r2=(1 - ss / st) if st > 0 else float("nan"), n=len(ids))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    rows = []
    for p in a.csv:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    good, bad = partition(rows)
    nd = sum(1 for r in bad if "arm_not_driven" in (r.get("invalid_reason") or ""))
    if nd:
        print("EXCLUDED: %d trial(s) where an arm was never driven "
              "(not an infinitely slow reach)\n" % nd)
    good = assert_analysable(EXPERIMENT, good, bad)

    single = [r for r in good if (r.get("mode") or "").lower() == "single"]
    bim = [r for r in good if (r.get("mode") or "").lower() != "single"]
    if not single:
        print("NO SINGLE-ARM BASELINE BLOCK -- the bottleneck cannot be")
        print("measured without it. Report throughput only, not the ratio.")

    print("FITTS FIT  (MT = a + b.ID,  throughput = 1/b)")
    print("  %-10s %-6s %6s %8s %8s %8s" % ("mode", "arm", "n", "a", "b", "TP"))
    tp = {}
    for label, rs in (("single", single), ("bimanual", bim)):
        for arm in ("left", "right"):
            f = fit_fitts(rs, arm)
            if not f:
                continue
            tp[(label, arm)] = f["tp"]
            print("  %-10s %-6s %6d %8.3f %8.3f %8.2f  (R2 %.2f)"
                  % (label, arm, f["n"], f["a"], f["b"], f["tp"], f["r2"]))

    s_sum = sum(tp.get(("single", a2), float("nan")) for a2 in ("left", "right"))
    b_sum = sum(tp.get(("bimanual", a2), float("nan")) for a2 in ("left", "right"))
    if s_sum == s_sum and b_sum == b_sum and s_sum > 0:
        print("\nATTENTION BOTTLENECK")
        print("  single-arm summed throughput   %.2f bits/s" % s_sum)
        print("  bimanual summed throughput     %.2f bits/s" % b_sum)
        print("  RATIO %.2f  -- 1.0 = two arms are free, 0.5 = strict "
              "serialisation" % (b_sum / s_sum))

    print("\nSTAGGER  |t_arrive_left - t_arrive_right| (s)")
    for c in CONDITIONS:
        v = np.array([num(r, "stagger_s") for r in bim
                      if (r.get("condition") or "").lower() == c])
        v = v[np.isfinite(v)]
        if v.size:
            print("  %-10s median %.2f  p90 %.2f" % (c, np.median(v),
                                                     np.percentile(v, 90)))
    print("\n  A large stagger WITH a ratio near 0.5 is time-slicing, not")
    print("  parallelism, and that is the finding rather than the throughput.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
