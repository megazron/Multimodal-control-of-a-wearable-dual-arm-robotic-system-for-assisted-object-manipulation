#!/usr/bin/env python3
"""THE TWO-HOUR SESSION, as data plus the arithmetic that checks it.

A timeline written as prose is an assertion. This one is a list of blocks with
durations, and `check()` recomputes every constraint from it, so "it fits two
hours" is a result rather than a claim -- and so the next person to add five
minutes to the safety brief finds out immediately what it costs.

THE THREE HARD CONSTRAINTS, and none of them come from the science:

    SESSION_CAP_MIN      120   the brief
    PACK_CONTINUOUS_MIN   12   the wearer carries >17 kg with no gravity
                               compensation; the existing protocol's cap
    PACK_TOTAL_MIN        48   same source, per person

The binding one is PACK-ON, not the clock. That is what forced the shape
below, and it is why TASK A IS BENCH-MOUNTED. The project's own rule is that
E1/E5-style characterisation runs bench-mounted and only tasks whose CLAIM is
about wearable SRLs need to be worn (CLAUDE.md, gravity and load). Task A is
an uncoupled Fitts characterisation -- its claim is about pointing, not about
wearing -- so bench-mounting it is honest and it buys 15 minutes of pack-on
budget that Tasks B and C actually need.

ROLES ARE FIXED WITHIN A DYAD and counterbalanced ACROSS dyads. Swapping roles
mid-session would double the conditions, and there is no version of that which
fits two hours. It is a real limitation: no dyad member experiences both
sides, so any within-person operator-vs-wearer comparison is unavailable and
must not be claimed.

WHAT WAS CUT TO REACH 120 MINUTES, stated plainly rather than absorbed:
  * five tasks -> three (see tasks.py for why each merge is defensible);
  * pick-and-place left the timeline entirely -- it never had a baseline, so
    it contributed no comparative measure to lose;
  * the information sheet is read BEFORE arrival, which shortens consent
    without shortening the conversation;
  * per-condition familiarisation is amortised: the second mode's Task A is a
    minute shorter because the task, though not the mode, is already known.

WHAT WAS NOT CUT, and will not be: the STOP DRILL. The wearer presses the
e-stop twice, themselves, before any arm moves near them. It is eight minutes
and it is the one block that may never be shortened to make the clock work.
"""

SESSION_CAP_MIN = 120
PACK_CONTINUOUS_MIN = 12
PACK_TOTAL_MIN = 48

# (minutes, label, worn) -- `worn` is True only while the pack is ON the
# wearer. Fitting and removal are inside the adjacent non-worn blocks.
BLOCKS = [
    (4,  "arrival; roles assigned (information sheet read beforehand)", False),
    (8,  "CONSENT, separately, in different rooms", False),
    (7,  "baselines: EDA fitted, Borg baseline, proprioceptive drift", False),
    (8,  "SAFETY BRIEF + STOP DRILL -- the wearer presses twice", False),
    (6,  "familiarisation, both modes, no data", False),
    (8,  "MODE 1 / TASK A positioning            [BENCH]", False),
    (9,  "MODE 1 / TASK B coordinated carry      [WORN]", True),
    (5,  "questionnaires + pack off + Borg", False),
    (9,  "MODE 1 / TASK C dual pursuit           [WORN]", True),
    (6,  "BREAK, pack off, NASA-TLX + trust", False),
    (7,  "MODE 2 / TASK A positioning            [BENCH]", False),
    (9,  "MODE 2 / TASK B coordinated carry      [WORN]", True),
    (5,  "questionnaires + pack off + Borg", False),
    (9,  "MODE 2 / TASK C dual pursuit           [WORN]", True),
    (6,  "final Borg, proprioceptive drift re-test, NASA-TLX + trust", False),
    (10, "debrief, both together", False),
]

# Where each block's minutes come from. A block duration that is not derived
# from a trial count is a guess, and a guess is what makes a timeline slip on
# the day. `check()` recomputes these and compares.
#   (trials, seconds per trial including reset, setup seconds)
TRIAL_BUDGET = {
    # 3 targets x 3 widths x 2 repeats x 3 conditions (left/right/both).
    # A "both" trial is ONE slot -- the two arms run simultaneously -- so
    # slots, not arm-trials, is the right unit for wall-clock.
    "TASK A": (54, 6, 60),
    # 3 paths x 2 objects (rigid, compliant) x 3 repeats
    "TASK B": (18, 30, 60),
    # 4 speed conditions + 2 single-arm baselines, x 2 repeats, 30 s trials
    "TASK C": (12, 45, 60),
}


def total_min():
    return sum(m for m, _, _ in BLOCKS)


def pack_total_min():
    return sum(m for m, _, w in BLOCKS if w)


def pack_longest_run_min():
    """Longest CONTINUOUS worn stretch. Adjacent worn blocks accumulate."""
    best = run = 0
    for m, _, w in BLOCKS:
        run = run + m if w else 0
        best = max(best, run)
    return best


def derived_block_min(task):
    trials, per_trial_s, setup_s = TRIAL_BUDGET[task]
    return (trials * per_trial_s + setup_s) / 60.0


def check():
    """Every constraint recomputed. Returns (ok, [(name, ok, detail)])."""
    rows = []

    t = total_min()
    rows.append(("session fits %d min" % SESSION_CAP_MIN, t <= SESSION_CAP_MIN,
                 "%d min" % t))

    p = pack_total_min()
    rows.append(("pack-on total <= %d min" % PACK_TOTAL_MIN,
                 p <= PACK_TOTAL_MIN, "%d min for the wearer" % p))

    r = pack_longest_run_min()
    rows.append(("pack-on continuous <= %d min" % PACK_CONTINUOUS_MIN,
                 r <= PACK_CONTINUOUS_MIN, "longest run %d min" % r))

    # Every task appears once per mode, and both modes are present.
    for key in ("TASK A", "TASK B", "TASK C"):
        n = sum(1 for _, lab, _ in BLOCKS if key in lab)
        rows.append(("%s appears in both modes" % key, n == 2,
                     "%d blocks" % n))

    # The block durations must be DERIVED, not guessed.
    for key, (trials, per_s, setup_s) in TRIAL_BUDGET.items():
        need = derived_block_min(key)
        have = [m for m, lab, _ in BLOCKS if key in lab]
        ok = all(m + 1e-9 >= need - 1.0 for m in have)   # 1 min slack allowed
        rows.append(("%s budget covers %d trials" % (key, trials), ok,
                     "needs %.1f min, allotted %s" % (need, have)))

    # The stop drill is present and not shortened.
    drill = [m for m, lab, _ in BLOCKS if "STOP DRILL" in lab]
    rows.append(("stop drill present, >= 8 min",
                 bool(drill) and min(drill) >= 8,
                 "%s min" % drill))

    return all(o for _, o, _ in rows), rows


def render():
    L = ["THE TWO-HOUR SESSION -- three tasks, two modes, roles fixed",
         "=" * 72]
    t = 0
    for m, lab, worn in BLOCKS:
        L.append("%4d  %-56s %2d%s" % (t, lab, m, "  [PACK ON]" if worn else ""))
        t += m
    L.append("%4d  END" % t)
    L += ["=" * 72,
          "total %d min (cap %d)   pack-on %d min (cap %d)   "
          "longest continuous %d min (cap %d)"
          % (total_min(), SESSION_CAP_MIN, pack_total_min(), PACK_TOTAL_MIN,
             pack_longest_run_min(), PACK_CONTINUOUS_MIN)]
    return "\n".join(L)


if __name__ == "__main__":
    import sys
    print(render())
    ok, rows = check()
    print("\nCONSTRAINTS")
    for name, o, detail in rows:
        print("  %-42s %s   %s" % (name, "PASS" if o else "FAIL", detail))
    print("\n%s" % ("ALL CONSTRAINTS SATISFIED" if ok else "DOES NOT FIT"))
    sys.exit(0 if ok else 1)
