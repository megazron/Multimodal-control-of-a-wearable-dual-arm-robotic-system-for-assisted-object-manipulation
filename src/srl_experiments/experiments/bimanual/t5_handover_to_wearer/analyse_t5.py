#!/usr/bin/env python3
"""T5 analysis — handover latency, reported as a DISTRIBUTION.

    python3 analyse_t5.py <summary.csv> ...

A mean latency hides the failure this task is most likely to have: a receive
point at the edge of the reachable set makes the distribution bimodal, and the
mean lands between the two modes where no trial ever occurred.
"""
import argparse, csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from srl_experiments.validity import partition, assert_analysable  # noqa: E402
from bimanual_metrics import PRESENCE_FAULTS                       # noqa: E402

EXPERIMENT = "T5 handover to the wearer"
CONDITIONS = ("direct", "assisted", "shared")


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def bimodal_hint(v):
    """Crude gap test: a clear empty band between two clusters."""
    v = np.sort(np.array([x for x in v if x == x], float))
    if v.size < 6:
        return ""
    gaps = np.diff(v)
    i = int(np.argmax(gaps))
    if gaps[i] > 2.0 * np.median(gaps) and 1 < i < v.size - 2:
        return ("   BIMODAL? gap %.1f-%.1f s -- check the receive point is "
                "inside MEDIAN reach, not just max" % (v[i], v[i + 1]))
    return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    rows = []
    for p in a.csv:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    good, bad = partition(rows)
    faults = defaultdict(int)
    for r in bad:
        why = (r.get("invalid_reason") or "")
        for f in PRESENCE_FAULTS:
            if f in why:
                faults[f] += 1
    if faults:
        print("SCENE FAULTS (not operator error):")
        for k, v in sorted(faults.items()):
            print("   %-22s x%d" % (k, v))
        print()
    good = assert_analysable(EXPERIMENT, good, bad)
    by = defaultdict(list)
    for r in good:
        by[(r.get("condition") or "?").lower()].append(r)

    print("\nHANDOVER LATENCY  (t_press - t_arrival)")
    print("  %-10s %5s %8s %8s %8s %8s" % ("cond", "n", "median", "IQR",
                                           "min", "max"))
    for c in CONDITIONS:
        v = [num(r, "handover_latency_s") for r in by.get(c, [])]
        v = np.array([x for x in v if x == x])
        if not v.size:
            continue
        print("  %-10s %5d %8.2f %8.2f %8.2f %8.2f%s"
              % (c, v.size, np.median(v),
                 np.percentile(v, 75) - np.percentile(v, 25), v.min(), v.max(),
                 bimodal_hint(v)))

    print("\nSTATION DRIFT while waiting (mm) -- a drifting robot makes the")
    print("wearer chase it, which inflates latency for an unrelated reason")
    for c in CONDITIONS:
        v = [num(r, "station_drift_mm") for r in by.get(c, [])]
        v = np.array([x for x in v if x == x])
        if v.size:
            print("  %-10s median %.1f  max %.1f" % (c, np.median(v), v.max()))

    print("\nFAILED HANDOVERS (timeouts, counted NOT discarded)")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if rs:
            f = sum(1 for r in rs if num(r, "handover_latency_s") != num(r, "handover_latency_s"))
            print("  %-10s %d of %d" % (c, f, len(rs)))

    print("\nWEARER TASK INTERFERENCE (peg rate)")
    for c in CONDITIONS:
        rs = by.get(c, [])
        b = np.array([num(r, "peg_rate_before") for r in rs])
        d = np.array([num(r, "peg_rate_during") for r in rs])
        m = np.isfinite(b) & np.isfinite(d) & (b > 0)
        if m.any():
            print("  %-10s during/before = %.2f" % (c, float((d[m] / b[m]).mean())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
