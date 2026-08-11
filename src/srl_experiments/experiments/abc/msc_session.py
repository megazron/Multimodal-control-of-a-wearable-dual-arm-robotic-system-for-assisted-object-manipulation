#!/usr/bin/env python3
"""THE DATA COLLECTION SEQUENCE for the MSc set: what runs, in what order.

    python3 src/srl_experiments/experiments/abc/msc_session.py

`session_timeline.py` is the SUPERSEDED A/B/C plan -- two modes, tasks A/B/C --
and it is kept because its arithmetic and its caps are still the source those
numbers come from. This file is the current set: five tasks (T0, T1 stage 1,
T1 stage 2, T2, T3) across five control modes.

THE PROBLEM THIS FILE EXISTS TO SOLVE IS THAT 5 x 5 DOES NOT FIT.
Twenty-five task-mode cells at even four minutes each is 100 minutes of
trials before a single questionnaire, consent, brief, fitting, break or
debrief -- against a 120 min session cap and a 12 min continuous pack limit
for a wearer carrying over 17 kg with no gravity compensation. Pretending
otherwise is how a protocol slips on the day and loses its last block, which
is always the one with the interesting comparison in it.

So the design is stated as a choice with its cost, not presented as a fit:

  * EVERY TASK IS RUN IN THE TWO ANCHOR MODES -- 01_master_teleop, the
    baseline every other mode is read against, and 06_full_autonomy, the far
    end of the ladder. That pair is what the headline comparison needs.
  * THE THREE MIDDLE MODES (03_shared_autonomy, 02_vr_teleop, 04_vr_shared)
    are run on a SUBSET chosen for what each is FOR: shared autonomy on the
    tasks with a grasp to assist (T1 s1, T3), and the two VR modes on the
    tasks where the pinned wrist is the binding constraint (T1 s1, T1 s2),
    since VR commands 6-DOF and master teleop does not.
  * T2 RUNS IN THE ANCHORS ONLY. It is the coupling task; its metrics are
    joint (tilt, separation) and its claim -- remove an arm and it is
    impossible -- does not need five modes to make.

WHAT THAT COSTS, PLAINLY: the mode axis is FULLY crossed only for T0 and
T1 stage 1. For T2, T3 and T1 stage 2 the missing cells cannot be estimated,
and any statement of the form "mode X beat mode Y on task Z" must name whether
Z was run in both. A half-crossed design analysed as if it were full is the
error this paragraph exists to prevent.

ORDER WITHIN THE SESSION is fixed by three things that are not negotiable:
  1. BENCH BEFORE WORN, and worn blocks kept short and separated, because
     fatigue is collinear with condition order and the master arm has no
     gravity compensation;
  2. MODE ORDER COUNTERBALANCED ACROSS DYADS by a Williams square, which
     balances immediate sequence as well as position -- `conditions.py`
     already implements it and this file does not reimplement it;
  3. T1 STAGE 1 BEFORE STAGE 2, always, because stage 2 is stage 1 done twice
     at once and a participant who has not done stage 1 is learning the task
     and the simultaneity at the same time.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SESSION_CAP_MIN = 120
PACK_CONTINUOUS_MIN = 12
PACK_TOTAL_MIN = 48

ANCHORS = ("01_master_teleop", "06_full_autonomy")
MIDDLE = ("03_shared_autonomy", "02_vr_teleop", "04_vr_shared")

# task -> (modes it runs in, trials, seconds per trial incl. reset, worn?)
#
# Trial counts are the task's own `repeats` where it declares one, and the
# seconds are measured clip durations rounded up for reset -- not guesses.
# Measured from the 2026-08-11 recordings: T0 ~15 s of motion, T1 stage 1
# ~29 s, T2 ~12 s, T3 ~20 s.
PLAN = {
    "T0":       (ANCHORS + MIDDLE, 6, 20, False),
    "T1_s1":    (ANCHORS + MIDDLE, 4, 35, False),
    "T1_s2":    (ANCHORS + ("02_vr_teleop", "04_vr_shared"), 3, 45, False),
    "T2":       (ANCHORS, 3, 20, True),
    "T3":       (ANCHORS + ("03_shared_autonomy",), 2, 30, True),
}
SETUP_S = 60          # per task-mode cell: scene, home, brief the pair

# The fixed, non-trial blocks. Same sources as session_timeline.py.
OVERHEAD = [
    (4, "arrival; roles assigned"),
    (8, "CONSENT, separately, in different rooms"),
    (7, "baselines: EDA fitted, Borg baseline, proprioceptive drift"),
    (8, "SAFETY BRIEF + STOP DRILL -- the wearer presses twice"),
    (6, "familiarisation, both anchor modes, no data"),
    (6, "BREAK, pack off, NASA-TLX + trust"),
    (5, "questionnaires + pack off + Borg"),
    (6, "final Borg, proprioceptive drift re-test, NASA-TLX + trust"),
    (10, "debrief, both together"),
]


def cells():
    """Every (task, mode) cell that runs, in session order.

    BENCH FIRST, then worn, and T1 stage 1 before stage 2 within a mode.
    """
    order = ["T0", "T1_s1", "T1_s2", "T2", "T3"]
    out = []
    for worn in (False, True):
        for task in order:
            modes, trials, secs, w = PLAN[task]
            if w != worn:
                continue
            for mode in modes:
                out.append((task, mode, trials, secs, w))
    return out


def trial_minutes():
    return sum((trials * secs + SETUP_S) / 60.0
               for _t, _m, trials, secs, _w in cells())


def overhead_minutes():
    return sum(m for m, _ in OVERHEAD)


def pack_minutes():
    return sum((trials * secs + SETUP_S) / 60.0
               for _t, _m, trials, secs, w in cells() if w)


def check():
    """Arithmetic, not assertion. Returns [(label, ok, detail)]."""
    rows = []
    t, o, p = trial_minutes(), overhead_minutes(), pack_minutes()
    total = t + o
    rows.append(("session fits %d min" % SESSION_CAP_MIN,
                 total <= SESSION_CAP_MIN,
                 "%.0f min = %.0f trials + %.0f overhead" % (total, t, o)))
    rows.append(("pack-on total <= %d min" % PACK_TOTAL_MIN,
                 p <= PACK_TOTAL_MIN, "%.0f min for the wearer" % p))
    longest = max(((trials * secs + SETUP_S) / 60.0
                   for _t, _m, trials, secs, w in cells() if w), default=0.0)
    rows.append(("longest continuous pack-on <= %d min" % PACK_CONTINUOUS_MIN,
                 longest <= PACK_CONTINUOUS_MIN,
                 "%.1f min" % longest))
    return rows


def caveats():
    """Design consequences that are TRUE BY CHOICE, not failures.

    Reported separately and loudly, and deliberately NOT counted as failing
    checks. A gate that goes red for a decision somebody made on purpose is
    switched off within a day, and then the real failures go with it -- the
    same reasoning as the test allowlist. What must not happen is that this
    goes UNSAID, because a half-crossed design analysed as a full one is a
    real error.
    """
    full = [k for k, v in PLAN.items() if set(v[0]) >= set(ANCHORS + MIDDLE)]
    part = {k: sorted(set(ANCHORS + MIDDLE) - set(v[0]))
            for k, v in PLAN.items() if k not in full}
    out = [("mode axis fully crossed for %s ONLY" % ", ".join(sorted(full)),
            "every other task has missing cells that CANNOT be estimated")]
    for k, missing in sorted(part.items()):
        out.append(("%s does not run in %s" % (k, ", ".join(missing)),
                    "any claim of the form 'mode X beat mode Y on %s' must "
                    "say whether %s was run in both" % (k, k)))
    return out


def main():
    print("=" * 74)
    print("MSc DATA COLLECTION SEQUENCE")
    print("=" * 74)
    print("\n%-8s %-20s %6s %8s %6s" % ("task", "mode", "trials", "s/trial",
                                        "worn"))
    for task, mode, trials, secs, w in cells():
        print("%-8s %-20s %6d %8d %6s"
              % (task, mode, trials, secs, "WORN" if w else "bench"))
    print("\n%d cells, %.0f min of trials, %.0f min overhead"
          % (len(cells()), trial_minutes(), overhead_minutes()))
    print("\nCHECKS -- arithmetic against the caps")
    bad = 0
    for label, ok, detail in check():
        print("   %-38s %-4s %s" % (label, "OK" if ok else "FAIL", detail))
        bad += not ok
    print("\nCAVEATS -- true by choice, and they must reach the analysis")
    for label, detail in caveats():
        print("   %s\n      %s" % (label, detail))
    print("\n%d of %d checks fail, %d caveats"
          % (bad, len(check()), len(caveats())))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
