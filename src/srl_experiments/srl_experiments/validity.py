#!/usr/bin/env python3
"""validity.py — one definition of "analysable", shared by every analyser.

Two separate jobs, and conflating them is how a rig fault becomes a result:

  1. EXCLUDE invalid trials, exactly as pre-registered. An invalid trial is
     still written out -- it is data about the RIG even when it is not data
     about the participant -- so exclusion happens at analysis time, from the
     `valid` column, and never by deleting rows.

  2. REFUSE to produce a result set that is empty or mostly empty.
     A session where every trial was invalidated used to analyse cleanly:
     `0 valid trials (24 excluded)` followed by empty means and NaN
     confidence intervals, which reads exactly like a null finding. A null
     finding and "the rig broke for the whole session" must not look alike,
     because only one of them is about the hypothesis.

The pre-registered rule lives here so all six experiments cannot drift apart.
"""
import sys
from collections import Counter

# Above this fraction the session is not a degraded dataset, it is a broken
# session. Chosen so a normal run with one or two rig hiccups still analyses.
MAX_INVALID_FRACTION = 0.5


def partition(rows):
    """Split rows into (valid, invalid). `valid` defaults to 1 when absent."""
    good, bad = [], []
    for r in rows:
        (bad if str(r.get("valid", "1")) in ("0", "") else good).append(r)
    return good, bad


def causes(invalid_rows):
    """How many of each invalid_reason, so the cause is never lost."""
    return Counter((r.get("invalid_reason") or "unrecorded").strip()
                   for r in invalid_rows)


def report(experiment, good, bad, stream=sys.stdout):
    total = len(good) + len(bad)
    frac = (len(bad) / total) if total else 0.0
    print("%s — %d valid trials, %d EXCLUDED as invalid (%.0f%% of %d)"
          % (experiment, len(good), len(bad), 100 * frac, total), file=stream)
    for reason, n in causes(bad).most_common():
        print("    excluded x%-3d %s" % (n, reason), file=stream)
    return frac


def assert_analysable(experiment, good, bad, stream=sys.stdout):
    """Exclude as pre-registered, then refuse to analyse a broken session.

    Exits NON-ZERO rather than returning, so a scripted pipeline cannot carry
    an empty result set forward as though it were a finding.
    """
    total = len(good) + len(bad)
    frac = report(experiment, good, bad, stream=stream)

    if total == 0:
        print("\nREFUSING TO ANALYSE: no trials at all in this summary. "
              "This is a missing or empty session file, not a null result.",
              file=stream)
        sys.exit(2)

    if not good:
        print("\nREFUSING TO ANALYSE %s: EVERY ONE of the %d trials was "
              "invalidated. This is a BROKEN SESSION, not a null finding, and "
              "producing empty means for it would be indistinguishable from "
              "one. Causes above; fix the rig and re-run the session."
              % (experiment, total), file=stream)
        sys.exit(2)

    if frac > MAX_INVALID_FRACTION:
        print("\nREFUSING TO ANALYSE %s: %.0f%% of trials were invalidated, "
              "above the pre-registered ceiling of %.0f%%. The surviving %d "
              "trials are not a sample of the intended design -- exclusion at "
              "this rate is selection, not cleaning."
              % (experiment, 100 * frac, 100 * MAX_INVALID_FRACTION, len(good)),
              file=stream)
        sys.exit(2)
    return good
