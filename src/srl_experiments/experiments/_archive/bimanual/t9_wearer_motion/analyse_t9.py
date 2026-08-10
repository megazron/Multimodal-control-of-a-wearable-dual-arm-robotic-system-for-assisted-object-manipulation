#!/usr/bin/env python3
"""T9 analysis — does autonomy compensate a disturbance the operator cannot feel?

    python3 analyse_t9.py <summary.csv> [<summary.csv> ...]

THE HYPOTHESIS IS AN INTERACTION, NOT A MAIN EFFECT (H2).

    error ~ b0 + b_auto*autonomy + b_sway*amplitude + b_int*(autonomy x amplitude)

`b_int` is the finding. A main effect of autonomy would be unsurprising and
could come from anywhere; the prediction specific to a two-person system is
that autonomy's advantage GROWS with disturbance amplitude, because the
operator has no efference copy of a disturbance originating in someone else's
body and sees it only through a camera mounted on that moving body.

THE MANIPULATION CHECK COMES FIRST AND CAN VETO THE ANALYSIS.
`sway_amplitude_mm` is what the metronome ASKED for; `wearer_motion_rms_mm` is
what the wearer's IMU MEASURED. If the second does not track the first, the
independent variable did not happen and nothing downstream means anything. A
wearer who tires and stops swaying produces a beautiful null result.

H2-null IS PRE-REGISTERED. If the operator can simply watch the sway and
compensate visually, there is no interaction. The metronome rates straddle
plausible visual-tracking bandwidth so that outcome is informative rather
than a failure to build the thing.
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

EXPERIMENT = "T9 reach under wearer motion"
CONDITIONS = ("direct", "assisted", "shared")
AUTONOMY_LEVEL = {"direct": 0.0, "assisted": 1.0, "shared": 2.0}


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError, TypeError):
        return float("nan")


def describe(v):
    v = np.array([x for x in v if x == x], float)
    if not v.size:
        return "n=0 (NOT RECORDED)"
    return "n=%d  median %.1f  IQR %.1f" % (
        v.size, np.median(v), np.percentile(v, 75) - np.percentile(v, 25))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    rows = []
    for p in a.csv:
        rows += list(csv.DictReader(open(p)))
    good, bad = partition(rows)
    good = assert_analysable(EXPERIMENT, good, bad)
    print("\n%s — %d valid trials" % (EXPERIMENT, len(good)))

    # ---------------------------------------------- manipulation check FIRST
    print("\nMANIPULATION CHECK — did the wearer actually sway as instructed?")
    pairs = [(num(r, "sway_amplitude_mm"), num(r, "wearer_motion_rms_mm"))
             for r in good]
    pairs = [(a_, b_) for a_, b_ in pairs if a_ == a_ and b_ == b_]
    if len(pairs) < 4:
        print("  wearer_motion_rms_mm NOT RECORDED (%d usable rows)."
              % len(pairs))
        print("  The IV is COMMANDED but UNMEASURED. Without the wearer's own")
        print("  IMU the sway condition is an instruction, not a variable, and")
        print("  a null result cannot be distinguished from a wearer who")
        print("  stopped swaying. Analysis continues but is NOT confirmatory.")
        confirmed = False
    else:
        A = np.array([p[0] for p in pairs])
        B = np.array([p[1] for p in pairs])
        r = float(np.corrcoef(A, B)[0, 1])
        slope = float(np.polyfit(A, B, 1)[0])
        span = float(B.max() - B.min())
        cspan = float(A.max() - A.min())
        # CORRELATION ALONE IS NOT ENOUGH, and a known-answer test is what
        # showed it: a wearer who tires and PLATEAUS at 20 mm while being
        # asked for 20/60/100 still correlates r = +0.71 with the command,
        # because a plateau is still monotonic. The check that catches it is
        # the SLOPE (measured must actually grow with commanded) and the
        # SPAN (the measured range must cover a useful fraction of the
        # commanded range). Correlation is reported, not relied upon.
        cover = span / cspan if cspan > 0 else 0.0
        print("  commanded vs measured: r = %+.3f, slope %.2f, "
              "measured span %.0f mm of %.0f mm commanded (%.0f%%)  (n=%d)"
              % (r, slope, span, cspan, 100 * cover, len(pairs)))
        confirmed = (r > 0.7) and (slope > 0.5) and (cover > 0.5)
        if confirmed:
            print("  MANIPULATION CONFIRMED")
        else:
            why = []
            if r <= 0.7:
                why.append("measured motion does not follow the command")
            if slope <= 0.5:
                why.append("measured grows far slower than commanded "
                           "(slope %.2f)" % slope)
            if cover <= 0.5:
                why.append("measured span covers only %.0f%% of the "
                           "commanded range -- a PLATEAU, which is what a "
                           "tiring wearer looks like" % (100 * cover))
            print("  MANIPULATION FAILED: %s." % "; ".join(why))
            print("  Do not interpret the interaction below as confirmatory.")

    by = defaultdict(list)
    for r_ in good:
        by[((r_.get("condition") or "?").lower(),
            num(r_, "sway_amplitude_mm"))].append(r_)

    print("\nTRACKING ERROR (mm RMS) by condition x sway amplitude")
    amps = sorted({k[1] for k in by if k[1] == k[1]})
    print("  %-10s %s" % ("condition",
                          "  ".join("%5.0f mm" % a_ for a_ in amps)))
    for c in CONDITIONS:
        cells = []
        for a_ in amps:
            v = [num(r_, "rms_error_mm_left") for r_ in by.get((c, a_), [])]
            v = [x for x in v if x == x]
            cells.append("%8.1f" % np.median(v) if v else "       -")
        print("  %-10s %s" % (c, "  ".join(cells)))

    # ------------------------------------------------------ the interaction
    X, y = [], []
    for (c, amp), rs in by.items():
        if c not in AUTONOMY_LEVEL or amp != amp:
            continue
        for r_ in rs:
            e = num(r_, "rms_error_mm_left")
            if e != e:
                continue
            au = AUTONOMY_LEVEL[c]
            X.append([1.0, au, amp, au * amp])
            y.append(e)
    print("\nINTERACTION — the T9 hypothesis (H2)")
    if len(y) < 8:
        print("  n=%d — too few trials to fit. Need >= 8 spanning both "
              "factors." % len(y))
        return 1
    X = np.asarray(X)
    y = np.asarray(y)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ beta
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    print("  b_autonomy   %+8.2f mm per level" % beta[1])
    print("  b_sway       %+8.4f mm per mm of sway" % beta[2])
    print("  b_INTERACT   %+8.4f mm per (level x mm)   <-- THE FINDING"
          % beta[3])
    print("  R2           %8.3f   n=%d" % (1 - ss_res / max(ss_tot, 1e-9),
                                           len(y)))
    print()
    if beta[3] < -1e-4:
        print("  NEGATIVE interaction: autonomy's advantage GROWS with sway,")
        print("  which is H2 as predicted.")
    elif abs(beta[3]) <= 1e-4:
        print("  NO interaction: H2-null. The operator compensates visually at")
        print("  these amplitudes and rates. This is an informative result,")
        print("  not a failure — see 02_baseline_and_hypotheses.md §6.3.")
    else:
        print("  POSITIVE interaction: autonomy gets RELATIVELY WORSE as sway")
        print("  grows. Check the clearance and block traces before believing")
        print("  it — an autonomy that backs off under disturbance would look")
        print("  exactly like this.")
    if not confirmed:
        print("\n  REMINDER: the manipulation check did not pass. The above is")
        print("  exploratory only.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
