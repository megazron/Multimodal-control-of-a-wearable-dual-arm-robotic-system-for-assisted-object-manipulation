#!/usr/bin/env python3
"""T8 analysis — COORDINATION between two people, not reach by one.

    python3 analyse_t8.py <summary.csv> [<summary.csv> ...]

T8 is the only task in the set whose outcome is a property of the DYAD. The
robot metrics (tracking, clearance) are hygiene; the finding is whether two
people got faster at working together and whether the wearer began to move
before being asked.

THE HEADLINE IS `anticipation_s`, AND ITS SIGN IS THE WHOLE POINT.

    anticipation_s = t_wearer_starts_moving - t_operator_requests

    positive  the wearer waited to be asked        -- reactive
    zero      they moved on the request            -- reactive, fast
    NEGATIVE  the wearer moved BEFORE being asked  -- PREDICTIVE

A negative value means the wearer built a model of the operator's intent
while having no control over the limbs and no proprioception of them. That is
the substantive claim (H3) and it is why the sign is reported before the
magnitude.

WHY LATENCY ALONE WOULD MISLEAD. A dyad can cut coordination latency simply by
the wearer repositioning constantly and pre-emptively, which is not learning
and is worse for the wearer. `repositions_per_trial` is therefore reported
alongside, and a latency improvement accompanied by a rise in repositions is
reported as over-correction rather than as coordination.
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

EXPERIMENT = "T8 wearer-assisted reach"
CONDITIONS = ("direct", "assisted", "shared")


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def describe(v):
    v = np.array([x for x in v if x == x], float)
    if not v.size:
        return "n=0  (NOT RECORDED)"
    return ("n=%d  median %+.2f  IQR %.2f  [%.2f, %.2f]"
            % (v.size, np.median(v),
               np.percentile(v, 75) - np.percentile(v, 25), v.min(), v.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    rows = []
    for p in a.csv:
        rows += list(csv.DictReader(open(p)))
    good, bad = partition(rows)
    good = assert_analysable(EXPERIMENT, good, bad)

    # The dyadic fields are entered by the experimenter. If they are empty the
    # session did not capture coordination, and saying so is the only honest
    # output -- an empty column must never analyse to "no effect".
    have = sum(1 for r in good if num(r, "coordination_latency_s") ==
               num(r, "coordination_latency_s"))
    print("\n%s — %d valid trials, %d with dyadic data"
          % (EXPERIMENT, len(good), have))
    if have == 0:
        print("\n  NO COORDINATION DATA IN THESE FILES.")
        print("  coordination_latency_s / anticipation_s / initiator are")
        print("  entered by the experimenter against the trial clock. Their")
        print("  absence means the dyadic measures were not captured -- it")
        print("  does NOT mean coordination took zero time.")
        return 1

    by = defaultdict(list)
    for r in good:
        by[(r.get("condition") or "?").lower()].append(r)

    print("\nANTICIPATION (s) — negative means the wearer moved BEFORE being asked")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if rs:
            v = [num(r, "anticipation_s") for r in rs]
            n_pred = sum(1 for x in v if x == x and x < 0)
            print("  %-10s %s   predictive on %d/%d trials"
                  % (c, describe(v), n_pred, len([x for x in v if x == x])))

    print("\nCOORDINATION LATENCY (s) — request to reposition complete")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if rs:
            print("  %-10s %s" % (c, describe([num(r, "coordination_latency_s")
                                               for r in rs])))

    print("\nLEARNING — does latency fall across trials within a condition?")
    for c in CONDITIONS:
        rs = sorted(by.get(c, []), key=lambda r: int(r.get("trial_index") or 0))
        v = np.array([num(r, "coordination_latency_s") for r in rs], float)
        m = np.isfinite(v)
        if m.sum() >= 4:
            x = np.arange(len(v))[m]
            slope = float(np.polyfit(x, v[m], 1)[0])
            print("  %-10s slope %+.3f s/trial over %d trials  %s"
                  % (c, slope, m.sum(),
                     "converging" if slope < -0.01 else
                     "no trend" if abs(slope) <= 0.01 else "DIVERGING"))
        else:
            print("  %-10s n=%d — too few for a trend" % (c, int(m.sum())))

    print("\nWHO INITIATED")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if not rs:
            continue
        init = [(r.get("initiator") or "").strip().lower() for r in rs]
        w = sum(1 for i in init if i == "wearer")
        o = sum(1 for i in init if i == "operator")
        print("  %-10s wearer %d, operator %d, unrecorded %d"
              % (c, w, o, len(init) - w - o))

    print("\nOVER-CORRECTION CHECK — latency can fall for the wrong reason")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if rs:
            print("  %-10s repositions/trial %s   comm events %s"
                  % (c, describe([num(r, "repositions_per_trial") for r in rs]),
                     describe([num(r, "comm_events") for r in rs])))

    print("\nWEARER LOAD (Borg CR10) — the constraint on session length")
    for c in CONDITIONS:
        rs = by.get(c, [])
        if rs:
            print("  %-10s %s" % (c, describe([num(r, "wearer_borg")
                                               for r in rs])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
