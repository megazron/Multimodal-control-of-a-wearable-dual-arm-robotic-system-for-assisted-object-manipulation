#!/usr/bin/env python3
"""Disable channels one at a time and report what still works at each level.

Two sweeps, because they answer different questions:

  ORDERED   remove channels in a fixed order, so the ladder is walked top to
            bottom and each rung's entry condition is visible.
  EXHAUSTIVE every one of the 128 subsets of the seven channels, so no rung
            is claimed to be unreachable without having tried all the ways in.

Pure: no stack, no hardware. The ladder is a function of the healthy set.
"""
import itertools
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_teleop import capability as cap          # noqa: E402

CH = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
OUT = os.path.join(ROOT, "recordings/baselines/capability_degradation.json")


def cost_str(key):
    c = cap.COST_M.get(key)
    if c is None:
        return "unmeasured" if key not in ("DIR_ONLY", "NONE") else "no position"
    return "%.3f m mean / %.3f m p95" % (c["mean_m"], c["p95_m"])


def main():
    print("=" * 78)
    print("CAPABILITY UNDER CHANNEL LOSS")
    print("=" * 78)
    if not cap.COST_M:
        print("WARNING: no measured costs loaded; rungs will read 'unmeasured'")

    # ------------------------------------------------------------- ordered
    print("\nORDERED SWEEP -- remove one channel at a time, worst case last")
    print("  removed          healthy  rung        position  cost")
    order = ["j5", "j3", "j6", "j7", "j2", "j4", "j1"]
    healthy = set(CH)
    rows = []
    for i, ch in enumerate([None] + order):
        if ch:
            healthy.discard(ch)
        lvl, why = cap.select(healthy, imu_ok=True)
        k = cap.LEVELS[lvl]["key"]
        rows.append(dict(removed=ch, healthy=sorted(healthy), level=k,
                         position=cap.position_available(lvl), why=why))
        print("  %-15s %7d  %-10s %-8s  %s"
              % (ch or "(none)", len(healthy), k,
                 "yes" if cap.position_available(lvl) else "NO", cost_str(k)))
        if ch:
            print("      -> %s" % why)

    # --------------------------------------------------------- exhaustive
    print("\nEXHAUSTIVE SWEEP -- all %d subsets of the seven channels"
          % (2 ** len(CH)))
    counts = {}
    first_seen = {}
    for r in range(len(CH) + 1):
        for sub in itertools.combinations(CH, r):
            lvl, why = cap.select(set(sub), imu_ok=True)
            k = cap.LEVELS[lvl]["key"]
            counts[k] = counts.get(k, 0) + 1
            if k not in first_seen:
                first_seen[k] = sorted(sub)
    print("  rung        subsets  smallest healthy set reaching it")
    for lv in sorted(cap.LEVELS, key=lambda x: x):
        k = cap.LEVELS[lv]["key"]
        if k in counts:
            print("  %-10s %7d  %s" % (k, counts[k],
                                       first_seen[k] or "(none)"))
    missing = [cap.LEVELS[l]["key"] for l in cap.LEVELS
               if cap.LEVELS[l]["key"] not in counts]
    if missing:
        print("  UNREACHABLE with a healthy IMU: %s" % ", ".join(missing))
        print("    (expected: NONE is only reachable with the IMU down)")

    # ------------------------------------------------------- IMU down case
    print("\nIMU DOWN -- elevation has no other source")
    for sub in (tuple(CH), ("j1", "j2"), ("j1",), ()):
        lvl, why = cap.select(set(sub), imu_ok=False)
        print("  %-28s -> %-8s  position %s"
              % (str(sorted(sub)) if sub else "[]", cap.LEVELS[lvl]["key"],
                 "yes" if cap.position_available(lvl) else "NO"))

    # ------------------------------------------------------------ recovery
    print("\nRECOVERY -- a channel returning to health must move the rung UP")
    seq = [{"j1"}, {"j1", "j7"}, {"j1", "j7", "j4"},
           {"j1", "j7", "j4", "j2"}, set(CH)]
    prev = None
    ok = True
    for h in seq:
        lvl, _ = cap.select(h, True)
        k = cap.LEVELS[lvl]["key"]
        arrow = ""
        if prev is not None:
            if lvl > prev:
                arrow = "  <-- WENT DOWN, wrong"
                ok = False
            elif lvl < prev:
                arrow = "  <-- recovered"
        print("  healthy %-28s %-10s%s" % (sorted(h), k, arrow))
        prev = lvl
    print("  monotonic recovery: %s" % ("PASS" if ok else "FAIL"))

    # -------------------------------------------------- what to fix first
    print("\nWHAT TO FIX FIRST, from the measured left-arm health")
    left_healthy = {"j1", "j7"}          # measured 2026-08-06 baseline
    lvl, why = cap.select(left_healthy, True)
    print("  current: %s (%s)" % (cap.LEVELS[lvl]["key"], why))
    for ch, (l, imp) in sorted(cap.regain(left_healthy, True).items()):
        print("    fix %-3s -> %-10s %s" % (ch, cap.LEVELS[l]["key"],
                                            "WORTH IT" if imp else "no change"))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(ordered=rows, subset_counts=counts,
                   first_seen=first_seen, monotonic_recovery=ok),
              open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
