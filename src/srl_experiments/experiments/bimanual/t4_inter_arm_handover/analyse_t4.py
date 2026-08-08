#!/usr/bin/env python3
"""T4 analysis — and the guard that stops stepped and oriented being pooled.

    python3 analyse_t4.py <summary.csv> ...

T4 IS BLOCKED pending wrist orientation (see protocol.md). This script exists
so that when the wrist channels are repaired the analysis is ready, and so
that the pooling error cannot be made in the meantime.
"""
import argparse, csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from srl_experiments.validity import partition, assert_analysable  # noqa: E402
from bimanual_metrics import handover_ordering_ok                  # noqa: E402

EXPERIMENT = "T4 inter-arm handover"
CONDITIONS = ("direct", "assisted", "shared")


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


class PooledVariants(Exception):
    """Stepped and oriented trials are not the same experiment."""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--variant", default="", choices=["", "oriented", "stepped"])
    a = ap.parse_args()
    rows = []
    for p in a.csv:
        with open(p) as f:
            rows += list(csv.DictReader(f))

    variants = {(r.get("variant") or "oriented").strip().lower() for r in rows}
    if len(variants) > 1 and not a.variant:
        raise PooledVariants(
            "these files contain BOTH %s trials. The stepped-block variant "
            "removes the orientation requirement and therefore measures the "
            "timing of a sequential exchange, NOT an inter-arm regrasp -- the "
            "thing T4 exists to measure. Pooling them would average two "
            "different experiments. Re-run with --variant oriented or "
            "--variant stepped." % " and ".join(sorted(variants)))
    if a.variant:
        rows = [r for r in rows
                if (r.get("variant") or "oriented").strip().lower() == a.variant]
    variant = a.variant or (variants.pop() if variants else "oriented")
    if variant == "stepped":
        print("WARNING: STEPPED variant. Orientation was not required, so")
        print("these results do NOT speak to inter-arm regrasp capability.\n")

    good, bad = partition(rows)
    dropped = sum(1 for r in bad
                  if "dropped_at_transfer" in (r.get("invalid_reason") or ""))
    if dropped:
        print("%d trial(s) dropped AT TRANSFER (block fell through the "
              "exchange, distinct from a failed grasp)\n" % dropped)
    good = assert_analysable("%s [%s]" % (EXPERIMENT, variant), good, bad)

    by = defaultdict(list)
    for r in good:
        by[(r.get("condition") or "?").lower()].append(r)

    print("HANDOVER TIMING")
    print("  %-10s %5s %12s %12s %10s" % ("cond", "n", "duration s",
                                          "both-hold s", "drops"))
    for c in CONDITIONS:
        rs = by.get(c, [])
        if not rs:
            continue
        d = np.array([num(r, "handover_duration_s") for r in rs])
        b = np.array([num(r, "both_holding_s") for r in rs])
        dr = int(np.nansum([num(r, "drop") for r in rs]))
        print("  %-10s %5d %12.2f %12.2f %10d"
              % (c, len(rs), np.nanmedian(d), np.nanmedian(b), dr))

    print("\nORDERING CHECK -- receiver closes BEFORE giver opens")
    bad_order = 0
    for r in good:
        if not handover_ordering_ok(num(r, "t_receiver_closed"),
                                    num(r, "t_giver_opened")):
            bad_order += 1
    print("  %d of %d trials had giver-opens-first: a caught drop, not a "
          "handover" % (bad_order, len(good)))

    print("\nARMS FIGHTING (EE separation drift while both hold, mm)")
    for c in CONDITIONS:
        v = np.array([num(r, "transfer_force_proxy_mm") for r in by.get(c, [])])
        v = v[np.isfinite(v)]
        if v.size:
            print("  %-10s median %.1f  max %.1f" % (c, np.median(v), v.max()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
