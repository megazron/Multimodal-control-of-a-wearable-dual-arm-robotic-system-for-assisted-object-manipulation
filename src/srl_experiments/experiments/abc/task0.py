#!/usr/bin/env python3
"""TASK 0 -- FOUR COLOURED SPHERES PER ARM.

The pointing primitive the rest of the programme is read against: no objects
to grasp, no coupling, no vision required in the teleoperated conditions.  It
is the first thing a dyad does and the last thing that still runs when
anything else is unavailable.

WHY TWO SETS AND NOT ONE
------------------------
There is no set of targets both arms can use.  Measured on this platform, over
a 7 x 3 x 3 grid of the frontal volume: **0 of 63 cells are reachable by both
arms**, 16 by the left only (all at x >= +0.4), 18 by the right only (all at
x <= -0.4), and the entire centreline x in [-0.2, +0.2] by neither.  A shared
A/B/C/D would therefore be unreachable by one arm or the other by
construction.  Each arm gets its OWN four spheres, inside its OWN reachable
set, and the two sets are mirror images so the arms are compared fairly.

WHY |x| >= 0.30
---------------
A TOP-DOWN pose is what a sphere target implies and what the autonomy
conditions command.  Measured: 0 of 9 top-down candidates feasible at
x = 0.25, 9 of 9 at x = 0.30, and the failure persists with collision checking
OFF -- so it is wrist orientation, not obstruction.  0.30 is a cliff edge, not
a preference.

WHY THE RADIUS IS FIXED AND THE DISTANCE VARIES
-----------------------------------------------
Fitts' ID = log2(2A/W).  Task A varied W (60/30/15 mm) at three fixed
amplitudes, which makes the ID range an artefact of the TOLERANCE: the widths
are a scoring rule this rig imposes, and a 15 mm width is close enough to the
system's own tracking error that "difficulty" and "measurement noise" are the
same axis.  Here W is FIXED at one value for every target and the whole ID
range comes from AMPLITUDE, which is real geometry.  The spread in bits is
then log2(A_max/A_min) and depends on nothing else -- in particular it does
not change if W is later revised, which is the property that makes the
regression interpretable.

TIME-TO-FIRST-MOTION IS NOT PART OF MOVEMENT TIME
------------------------------------------------
Reported separately, always.  Full autonomy takes a MEASURED 2008 ms from
utterance to motion, of which 2000 ms is the deliberate confirmation wait --
a safety choice, not a control-quality result.  Folding it into movement time
would inflate autonomy's Fitts intercept by two seconds and flatter every
direct mode for a reason that has nothing to do with pointing.  So:

    t_dispatch   command issued  ->  first motion of the commanded arm
    t_movement   first motion    ->  target entered and HELD for dwell_s

Fitts' law is fitted to t_movement ONLY.  t_dispatch is reported as its own
per-mode figure with its fixed and variable parts named.

CONDITIONS DIFFER BY WHAT IS VISIBLE, AND THAT IS DELIBERATE
------------------------------------------------------------
    teleoperated modes   ONE sphere shown at a time.  The operator has no
                         selection problem, only a pointing problem, so the
                         measurement is Fitts' law and nothing else.
    full autonomy        ALL FOUR shown, and the command is "go to B".  The
                         selection problem is the point: naming one of four
                         visible alternatives is what the language and vision
                         path is for, and it cannot be exercised with a single
                         target on screen.

That asymmetry means the two are NOT a like-for-like pointing comparison, and
any comparison between them carries a selection cost on the autonomy side.
Stated here so it cannot be quietly dropped from the analysis.

OPEN: TASK 0 AND TASK A OVERLAP, AND SOMEBODY MUST DECIDE
---------------------------------------------------------
Both are Fitts pointing tasks, and their designs CONTRADICT each other: Task A
varies W at fixed amplitudes, Task 0 fixes W and varies amplitude, and the
paragraph above says why the second is the better of the two.  Running both
puts two incompatible Fitts characterisations in one session and spends
13 minutes doing it.

This is NOT decided here, because it is a protocol decision and not a
geometric one.  It is flagged rather than resolved so the choice is visible:
either Task 0 REPLACES Task A -- which frees 15 min and removes the
contradiction -- or both run and the write-up must say which regression is the
headline and why the other exists.  The session timeline currently carries
BOTH, and lands at exactly 120 min against a 120 min cap, so the decision is
also what buys back the only slack in the session.
"""

import itertools
import math

Y = 0.35                     # the only fore/aft band both arms can work in

# Target diameter.  30 mm, matching the middle width Task A already verified,
# so this is not a new tolerance nobody has measured against.
TARGET_W_M = 0.030
TARGET_R_M = TARGET_W_M / 2.0

# The box the LEFT arm's spheres live in.  MEASURED, with the bench in the
# scene, by scripts/survey_task0_band.py -- not reasoned out.  The first
# version of this band ran from z = 1.03 and was largely INSIDE the bench
# slab (BENCH_TOP 1.10, BENCH_THICK 0.04, so the slab is z 1.06..1.10);
# sphere A came back unreachable and the verifier refused to report, which is
# what it is for.
#
# Worth knowing, because it is the same trap one layer up: TASK_A declares
# targets at z = 1.05, also under the bench top, and passes -- because
# verify_abc_scenarios checks the study tasks in FREE SPACE and applies the
# furniture only for the clip tasks.  So no study task's band had ever been
# measured against the bench at all.
#
# And the band must be common to BOTH ARMS, not one arm's.  The two arms are
# parked asymmetrically -- CLAUDE.md records the residual as 1.3837 m and
# proves it independent of the mount -- so the right arm's reachable band is
# NOT the mirror of the left's.  A set derived from the left alone did
# exactly what that predicts: left 4/4 spheres and 0 waypoint failures, right
# sphere A unreachable and 8 waypoint failures around it.  The sets stay
# mirrored so the arms are compared fairly, which means the BAND has to be
# common ground.
#
# Survey (y = 0.35, k = 3, bench in scene, x mirrored for the right arm):
#     left only    feasible from z = 1.22
#     BOTH arms    feasible from z = 1.28; x 0.26..0.40 at z 1.28-1.31,
#                  widening to x 0.26..0.52 from z = 1.34
# The band below is the largest rectangle inside the BOTH-arms region, held
# 20 mm inside the last cell that passed -- this project's rule that N finds
# the boundary and MARGIN is what keeps you off it.
BAND = dict(x=(0.30, 0.50), y=Y, z=(1.36, 1.44))

# The survey also finds x = 0.26 and 0.28 feasible, which is NOT a
# contradiction of the |x| >= 0.30 rule: the survey solves for the arm's own
# anchor orientation, while the 0/9-at-0.25 finding is for a TOP-DOWN wrist.
# A sphere target implies a top-down pose, so 0.30 stays as the design rule.

DWELL_S = 0.5                # must be HELD inside the sphere

COLOURS = {"A": "red", "B": "green", "C": "blue", "D": "yellow"}


def fitts_id(amplitude_m: float, width_m: float = TARGET_W_M) -> float:
    """Shannon formulation, ID = log2(A/W + 1).

    Shannon rather than Welford's 2A/W: it cannot go negative for A < W/2,
    and it is the form the ISO 9241-9 throughput convention assumes.  Which
    form is used must be stated or the intercepts are not comparable with
    anyone else's; this is that statement.
    """
    return math.log2(amplitude_m / width_m + 1.0)


def amplitudes(points: dict) -> dict:
    """Every ordered pair's amplitude.  Movements run BETWEEN spheres, so the
    six unordered pairs are the six IDs -- four targets buy six conditions."""
    keys = sorted(points)
    return {(a, b): math.dist(points[a], points[b])
            for a, b in itertools.combinations(keys, 2)}


def mirror(points: dict) -> dict:
    """Right-arm set: the left set reflected in the sagittal plane x=0.

    Mirroring the POSITIONS, never negating joint angles -- both arms
    instantiate the same non-mirrored Gen3 macro, so a joint-space mirror is
    not the same pose.
    """
    return {k: [-p[0], p[1], p[2]] for k, p in points.items()}


# --------------------------------------------------------------------------
# THE SPHERES.  Chosen by search, not by hand -- see choose_targets() and
# scripts/choose_task0_targets.py.  The objective is an even spread of the six
# pairwise IDs, because a Fitts regression is only as good as the spacing of
# its x-axis, and four hand-placed points in a 0.16 x 0.25 m box will always
# produce two amplitudes that are nearly equal and one condition wasted.
# --------------------------------------------------------------------------
# choose_targets() output over the MEASURED band, adopted verbatim.  The
# hand-placed set it replaced had two IDs at 2.77 and 2.81 -- one of the six
# conditions was a duplicate and bought nothing.  Unevenness of the ID gaps:
# 0.449 by hand, 0.068 by search.
#
# Six IDs spanning 1.54 bits: 1.43, 1.80, 2.07, 2.37, 2.67, 2.98.  That span
# is smaller than a bench-top Fitts study would want, and the reason is
# geometric, not a choice: with the bench in the scene the whole feasible
# band for this arm is 0.08 x 0.20 m, so the largest amplitude available is
# 0.215 m and the ID span is capped at log2(A_max/A_min) whatever is done
# with W.  It is reported that way rather than widened by shrinking W, which
# would only move the range onto the tolerance axis again.
TARGETS_LEFT = {
    "A": [0.3500, Y, 1.3600],
    "B": [0.3000, Y, 1.3700],
    "C": [0.5000, Y, 1.4200],
    "D": [0.3750, Y, 1.4300],
}
TARGETS_RIGHT = mirror(TARGETS_LEFT)


def choose_targets(band=BAND, n_grid=9, w=TARGET_W_M):
    """Pick four points in the band whose six pairwise IDs are as evenly
    spaced as possible, subject to the smallest being a real movement.

    Pure geometry over a grid, so the answer is arithmetic and reproducible:
    the standing rule allows synthetic input only where the ground truth is
    CONSTRUCTED rather than rendered, and two points 200 mm apart really are
    200 mm apart.  Reachability is NOT decided here -- that is
    verify_task0_scenarios.py's job, against a live solver with the bench in
    the scene.
    """
    xs = [band["x"][0] + i * (band["x"][1] - band["x"][0]) / (n_grid - 1)
          for i in range(n_grid)]
    zs = [band["z"][0] + i * (band["z"][1] - band["z"][0]) / (n_grid - 1)
          for i in range(n_grid)]
    cells = [(x, band["y"], z) for x in xs for z in zs]

    best, best_score = None, None
    for combo in itertools.combinations(cells, 4):
        d = sorted(math.dist(a, b)
                   for a, b in itertools.combinations(combo, 2))
        if d[0] < 0.05:                     # smaller than this is not a reach
            continue
        ids = [fitts_id(x, w) for x in d]
        gaps = [b - a for a, b in zip(ids, ids[1:])]
        # even spacing == small variance in the gaps; wide range == large
        # total span.  Maximise span, penalise unevenness.
        span = ids[-1] - ids[0]
        mean_gap = sum(gaps) / len(gaps)
        unevenness = sum((g - mean_gap) ** 2 for g in gaps) ** 0.5
        score = span - 3.0 * unevenness
        if best_score is None or score > best_score:
            best_score, best = score, combo
    return {k: [round(v, 4) for v in p]
            for k, p in zip("ABCD", sorted(best, key=lambda p: (p[2], p[0])))}


# --------------------------------------------------------------------------
# The task record itself
# --------------------------------------------------------------------------
TASK_0 = dict(
    key="0",
    name="sphere_pointing",
    role="pointing primitive; Fitts characterisation; selection under "
         "autonomy; the reference every other task is read against",
    bimanual="NO. Two SEPARATE per-arm sets, because 0 of 63 frontal cells "
             "are reachable by both arms -- a shared set is not an option "
             "that was rejected, it is one that does not exist.",
    arms_used="left | right | both",
    conditions=("left_only", "right_only", "both"),
    targets=dict(left=TARGETS_LEFT, right=TARGETS_RIGHT),
    colours=COLOURS,
    target_width_m=TARGET_W_M,
    target_radius_m=TARGET_R_M,
    dwell_s=DWELL_S,
    visibility=dict(
        teleoperated="one sphere at a time",
        full_autonomy="all four, commanded by name (\"go to B\")"),
    fitts_form="Shannon, ID = log2(A/W + 1)",
    repeats=2,
    metrics=("t_dispatch_s", "t_movement_s", "throughput_bits_s",
             "overshoot_mm", "dwell_failures", "dual_task_cost",
             "wrong_target_rate"),
    reporting_rule="t_dispatch is NEVER folded into t_movement; Fitts is "
                   "fitted to t_movement alone.",
)


if __name__ == "__main__":
    import json
    print("chosen by search:",
          json.dumps(choose_targets(), indent=2))
    print("\nshipped left set:", json.dumps(TARGETS_LEFT))
    print("\n%-8s %10s %8s" % ("pair", "amplitude", "ID"))
    for (a, b), amp in sorted(amplitudes(TARGETS_LEFT).items(),
                              key=lambda kv: kv[1]):
        print("%-8s %9.4f m %7.2f" % ("%s-%s" % (a, b), amp, fitts_id(amp)))
