#!/usr/bin/env python3
"""T2 analysis — box disturbance first, then the discrete placement outcome.

    python3 analyse_t2.py <summary.csv> [<summary.csv> ...]

A block that lands in a box that was dragged 40 mm is not a success, so the
holding arm's drift is reported first and the placement count second.

COLUMN NAMES WERE WRONG UNTIL 2026-08-08. This read `box_drift_max_mm`,
`drops` and `time_per_block_s`; the runner writes `hold_disturbance_max_mm`,
`blocks_dropped` and `mean_cycle_time_s`. Every one resolved to NaN, so the
analysis printed `n=0` across the board -- which is indistinguishable from a
session in which nothing happened. Names now match the runner, and
`assert_analysable` still fails loudly when nothing survives.
"""
import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from srl_experiments.validity import partition, assert_analysable  # noqa: E402
from bimanual_metrics import PRESENCE_FAULTS                       # noqa: E402

EXPERIMENT = "T2 hold and fill"
CONDITIONS = ("direct", "assisted", "shared")


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            rows += list(csv.DictReader(f))
    return rows


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def describe(v):
    v = np.array([x for x in v if x == x], float)
    if not v.size:
        return "n=0"
    return ("n=%d  median %.1f  IQR %.1f  max %.1f"
            % (v.size, np.median(v),
               np.percentile(v, 75) - np.percentile(v, 25), v.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    rows = load(a.csv)
    good, bad = partition(rows)

    # Scene faults are NOT participant failures and must be visible as such
    # before anything is excluded.
    faults = defaultdict(int)
    for r in bad:
        why = (r.get("invalid_reason") or "").strip()
        for f in PRESENCE_FAULTS:
            if f in why:
                faults[f] += 1
    if faults:
        print("SCENE FAULTS (object missing/knocked, NOT operator error):")
        for k, v in sorted(faults.items()):
            print("   %-22s x%d" % (k, v))
        print()

    good = assert_analysable(EXPERIMENT, good, bad)
    by = defaultdict(list)
    for r in good:
        by[(r.get("condition") or "?").lower()].append(r)

    print("\nBOX DISTURBANCE — the primary outcome")
    print("  %-10s %-46s %s" % ("condition", "max drift (mm)", "RMS drift (mm)"))
    for c in CONDITIONS:
        rs = by.get(c, [])
        if not rs:
            continue
        print("  %-10s %-46s %s"
              % (c, describe([num(r, "hold_disturbance_max_mm") for r in rs]),
                 describe([num(r, "hold_disturbance_rms_mm") for r in rs])))

    print("\nPLACEMENT — the discrete outcome T2 exists for")
    print("  %-10s %8s %8s %8s %9s  %s"
          % ("condition", "placed", "missed", "dropped", "success", "s/block"))
    for c in CONDITIONS:
        rs = by.get(c, [])
        if not rs:
            continue
        pl = np.nanmean([num(r, "blocks_placed") for r in rs])
        ms = np.nansum([num(r, "blocks_missed") for r in rs])
        dr = np.nansum([num(r, "blocks_dropped") for r in rs])
        sr = np.nanmean([num(r, "success_rate") for r in rs])
        tb = describe([num(r, "mean_cycle_time_s") for r in rs])
        print("  %-10s %8.1f %8.0f %8.0f %8.0f%%  %s"
              % (c, pl, ms, dr, 100 * sr, tb))
    # MISSED and DROPPED are kept apart deliberately: one is an aim that was
    # off, the other is a grip that let go, and merging them would hide which
    # of the two assistance actually helps.

    # The comparison the task exists for.
    d = [num(r, "hold_disturbance_rms_mm") for r in by.get("direct", [])]
    s = [num(r, "hold_disturbance_rms_mm") for r in by.get("assisted", [])]
    d = np.array([x for x in d if x == x])
    s = np.array([x for x in s if x == x])
    if d.size and s.size:
        print("\nDIRECT -> ASSISTED, box drift RMS")
        print("  direct   median %.1f mm" % np.median(d))
        print("  assisted median %.1f mm" % np.median(s))
        print("  reduction %.0f%%  -- this is the cost of dividing attention "
              "across two arms" % (100 * (1 - np.median(s) / max(np.median(d), 1e-9))))
        if min(d.size, s.size) < 5:
            print("  (n<5 per condition; report the effect, not a p-value)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
