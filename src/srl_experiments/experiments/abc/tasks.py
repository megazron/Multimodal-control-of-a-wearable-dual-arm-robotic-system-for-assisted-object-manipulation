#!/usr/bin/env python3
"""TASKS A, B AND C. The three-task set, consolidated from five.

WHY THREE, AND WHY THESE THREE
------------------------------
The binding constraint is not scientific, it is the WEARER. One person carries
>17 kg, the protocol caps pack-on at 12 minutes continuous and 48 minutes
total, and the session must fit TWO HOURS. Two modes x five tasks was measured
at 167 minutes and does not fit. Something had to go, and the honest way to
choose is by what each task uniquely measures.

Two consolidations, each already justified by a recorded finding:

  * TASKS 3 AND 4 WERE NEVER TWO MECHANISMS. CLAUDE.md records it plainly:
    "Tasks 3 and 4 are the same mechanism -- a coupled object spanning the
    dead band -- run with a rigid and a compliant object." Rigid-versus-
    compliant is the scientific contrast, so it belongs INSIDE one task as a
    within-task factor, not across two. Merging costs nothing and saves a
    whole condition's worth of familiarisation and questionnaires.

  * TASK 2 CARRIED NO COMPARATIVE MEASURE AT ALL. It has no DIRECT and no VR
    condition, because a top-down grasp needs 169.7 deg (left) / 164.6 deg
    (right) of wrist rotation from the anchor `orientation_mode: fixed` pins
    to, and nothing in the master measures the wrist to command it. A task
    that cannot be compared across modes cannot contribute to a mode
    comparison. It is kept below as an OPTIONAL demonstration, explicitly
    outside the two-hour timeline and outside every statistic.

What survives is one task per distinct thing being measured:

    A  POSITIONING       individual and divided attention, no coupling
    B  COORDINATED CARRY physical coupling through a shared object
    C  DUAL PURSUIT      simultaneity in two DISJOINT reachable sets

THE BINDING GEOMETRIC FACT, unchanged and load-bearing. The two arms'
reachable sets are DISJOINT: at y = 0.35, z = 1.10 the right arm reaches
x <= -0.10 and the left x >= +0.15, with a 0.25 m dead band between. So no
task may require both arms at the SAME point -- inter-arm handover is
impossible on this platform. A task can require both arms only if (a) a rigid
or tensioned object SPANS the gap and must be held at two points at once, or
(b) two targets in the two disjoint sets must be satisfied SIMULTANEOUSLY.
B takes route (a); C takes route (b). A is deliberately neither: it is the
uncoupled control that the other two are read against.

EVERY COORDINATE HERE IS VERIFIED AT N=10 OVER THE DENSIFIED FULL PATH by
`scripts/verify_abc_scenarios.py`. Endpoints are not enough -- the arm flies
the segments between declared waypoints -- and one IK call is not enough,
because TRAC-IK restarts randomly and a pose at the edge of the feasible set
is a coin flip.
"""

Y = 0.35                    # the only fore/aft band both arms can work in

# --------------------------------------------------------------------------
# SHARED GEOMETRY. Carried over from the five-task respec, with the reasoning,
# because both numbers were derived against two independent constraints and
# re-deriving them casually is how a verified geometry gets broken.
# --------------------------------------------------------------------------
# 500 mm puts each grip at |x| = 0.25. That is simultaneously (i) outside the
# 0.25 m dead band between the disjoint sets -- the superseded 310 mm span put
# each grip at |x| = 0.155, INSIDE it, and failed 10 of 25 waypoints -- and
# (ii) the smallest half-span clearing the 120 mm wearer floor, measured at
# 0.134 m clearance with 6/6 reachable.
TRAY_SEP = 0.500
# The sling must lengthen with the span or it is taut before the trial starts.
# Retention needs sag >= 2r, i.e. L >= 2*sqrt((s/2)^2 + (2r)^2). For s = 0.500
# and r = 0.020 that is L >= 0.5064, so 0.540 leaves a 34 mm margin AND keeps
# the failure threshold inside the reachable band -- which is the property
# that lets the task fail at all. A longer sling could never fail and would
# measure nothing.
SLING_L = 0.540
BALL_R = 0.020

# TRAP FOR WHOEVER WIRES THIS UP. The superseded packages carry their own
# spans: bimanual_metrics.TRAY_SEPARATION_M = 0.300 is the DEFAULT argument of
# tilt_deg(), and coupled_metrics.SLING_NOMINAL_SEP = 0.310. Feeding this
# spec's heights to those functions without passing the separation explicitly
# yields a tilt over the wrong baseline -- wrong by 5/3 and entirely
# plausible-looking. ALWAYS pass TRAY_SEP.


# --------------------------------------------------------------------------
# TASK A -- POSITIONING
# --------------------------------------------------------------------------
# No objects, no grasp, no coupling. Three blocks: left alone, right alone,
# both simultaneously. The single-versus-both contrast measures the attention
# bottleneck directly and needs nothing to exist, which makes it both the
# safest warm-up and the most robust measure in the set -- it is the one task
# that still runs if the grippers, the vision or the objects are unavailable.
#
# It is also the only task that survives every degradation of the master:
# with reach frozen the targets still differ in elevation and azimuth.
TASK_A = dict(
    key="A",
    name="positioning",
    role="uncoupled control; Fitts characterisation; warm-up",
    bimanual="NO -- the both-arms block is an ATTENTION contrast, not a "
             "coupling constraint. One arm could do either half alone.",
    arms_used="left | right | both",
    objects=None,
    conditions=("left_only", "right_only", "both"),
    targets={
        "left":  [[0.30, Y, 1.05], [0.30, Y, 1.25], [0.45, Y, 1.15]],
        "right": [[-0.30, Y, 1.05], [-0.30, Y, 1.25], [-0.45, Y, 1.15]],
    },
    widths_mm=(60, 30, 15),          # three IDs at each amplitude
    dwell_s=0.5,                     # must be HELD inside the target
    # TWO repeats, not three. 3 targets x 3 widths x 3 conditions x 3 repeats
    # is 81 trial slots = 9.1 min, and session_timeline.check() showed that
    # does not fit the block it was given -- which is the point of holding the
    # timeline as arithmetic rather than prose. Two repeats gives 54 slots
    # (6.4 min) and 9 ID conditions per arm-condition. Per-participant
    # throughput is thin at 2 repeats per ID; the Fitts regression is a
    # GROUP-level fit (n dyads x 2 modes per ID), and that is how it must be
    # reported. Do not present a per-participant throughput from this.
    repeats=2,
    metrics=("movement_time_s", "throughput_bits_s", "overshoot_mm",
             "dwell_failures", "dual_task_cost"),
)

# --------------------------------------------------------------------------
# TASK B -- COORDINATED CARRY, rigid AND compliant
# --------------------------------------------------------------------------
# A coupled object is held at two points TRAY_SEP apart, one gripper on each
# end. It SPANS the dead band between the reachable sets, which is exactly why
# it needs both arms: no single arm can hold a body at two separated points,
# and no amount of time changes that.
#
# THE OBJECT IS A WITHIN-TASK FACTOR, not a second task. Rigid coupling
# transmits coordination error instantly and proportionally; compliant
# coupling absorbs it and then fails abruptly. Running both against the SAME
# paths, in the same block, with the same grip separation, is what turns that
# into a contrast rather than two unrelated numbers.
#
#   RIGID     failure is TILT. Over 500 mm, 20 mm of height difference is
#             2.3 deg (visible wobble) and 60 mm is 6.8 deg, at which the ball
#             rolls off. Rescaled from the superseded 310 mm span, where the
#             same millimetres read 3.7 and 11.0 deg -- the ANGLE is the
#             criterion, so it MUST be recomputed whenever the span changes.
#   COMPLIANT failure is SEPARATION. sag(s) = sqrt((L/2)^2 - (s/2)^2), and the
#             ball is retained while sag >= 2r, so s_max = 534.0 mm.
#
# TRANSPORT IS VERTICAL, and that is a measured constraint, not a preference:
# the feasible band is only 0.35-0.40 m deep in y, so a fore-aft carry leaves
# it immediately (measured 0/20 carrying toward the wearer, against 20/20 for
# the grips and the vertical lift).
TASK_B = dict(
    key="B",
    name="coordinated_carry",
    role="physical coupling through a shared object; rigid vs compliant",
    bimanual="YES, route (a) -- a body held at two points 500 mm apart spans "
             "the dead band between the disjoint reachable sets.",
    arms_used="both, simultaneously",
    object_factor=("rigid", "compliant"),
    objects=dict(
        tray=dict(size=(TRAY_SEP + 0.06, 0.26, 0.02), grip_sep=TRAY_SEP,
                  colour="tan", mass_g=180),
        sling=dict(length=SLING_L, grip_sep=TRAY_SEP, colour="brown"),
        ball=dict(radius=BALL_R, colour="yellow", mass_g=15),
    ),
    # Identical paths for both objects. All vertical; see the note above.
    paths=dict(
        S1_short_lift=[[0.0, Y, 1.15], [0.0, Y, 1.20]],
        S2_full_lift=[[0.0, Y, 1.10], [0.0, Y, 1.30]],
        S3_detour=[[0.0, Y, 1.15], [0.0, 0.38, 1.20],
                   [0.0, 0.38, 1.25], [0.0, Y, 1.30]],
    ),
    grip_sep=TRAY_SEP,
    fail_tilt_deg=6.8,               # rigid:    60 mm over 500 mm
    fail_sep_m=0.5340,               # compliant: sag == 2r
    repeats=3,
    metrics=("tilt_rms_deg", "tilt_max_deg", "time_above_fail_tilt_s",
             "sep_err_rms_mm", "sep_err_max_mm", "min_sag_mm",
             "height_diff_rms_mm", "path_efficiency", "object_retained"),
)

# --------------------------------------------------------------------------
# TASK C -- SYNCHRONISED DUAL-TARGET PURSUIT
# --------------------------------------------------------------------------
# Each arm tracks its own moving target inside its own reachable set. The
# targets sit in the two DISJOINT sets, so one arm can never satisfy both --
# not sequentially, not with unlimited time. Success requires both within
# tolerance SIMULTANEOUSLY for a sustained fraction of the trial.
#
# TARGET SPEEDS VARY INDEPENDENTLY, and that is not a detail. With the two
# speeds yoked,
#
#     err_A ~ b0 + b_self*speed_A + b_cross*speed_B + b_int*speed_A*speed_B
#
# the self and cross terms are perfectly collinear and NEITHER is estimable.
# b_cross is the cross-arm interference -- the attention bottleneck measured
# directly -- and it is the headline number of this task.
#
# WHAT b_cross MEANS, so it is not over-read: the TOTAL speed sensitivity of
# one arm's RMS tracking error to the OTHER arm's target speed, in mm per
# (m/s). Total, because tracking error contains an attentional amplitude term
# AND a lag term (a fixed reaction delay L becomes positional error v*L), and
# from RMS error alone the two are NOT separable. `phase_lag_s` is reported
# beside it for exactly that reason: b_cross large with a flat phase lag means
# the effect is attentional; both rising together means it is not.
TASK_C = dict(
    key="C",
    name="dual_pursuit",
    role="simultaneity across the disjoint sets; cross-arm interference",
    bimanual="YES, route (b) -- the two targets lie in DISJOINT reachable "
             "sets, so one arm can never reach both, and simultaneity is the "
             "success criterion.",
    arms_used="both, simultaneously",
    objects=dict(target=dict(radius=0.050, colour="green", virtual=True)),
    centres={"left": [0.35, Y, 1.15], "right": [-0.35, Y, 1.15]},
    amplitude_m=0.08,
    # Lissajous, so the marker must stay inside a SPHERE of this radius. The
    # 1.06 factor caught a real bug before it reached a participant: the
    # figure exceeded its own amplitude by sqrt(1 + 0.35^2), so the marker
    # could leave the shell that had been verified reachable.
    shell_radius_m=0.08,
    speeds_m_s=dict(S1_both_slow=(0.05, 0.05), S2_one_fast=(0.20, 0.05),
                    S3_both_fast=(0.20, 0.20), S4_asymmetric=(0.30, 0.08)),
    baselines=dict(B1_left_only=(0.20, 0.0), B2_right_only=(0.0, 0.20)),
    tolerance_mm=40,
    trial_s=30,
    metrics=("rms_error_mm_left", "rms_error_mm_right", "frac_both_on_target",
             "phase_lag_s", "b_cross", "b_self", "dual_task_cost"),
)

ALL = [TASK_A, TASK_B, TASK_C]
BY_KEY = {t["key"]: t for t in ALL}


# --------------------------------------------------------------------------
# OPTIONAL, AND OUTSIDE THE TIMELINE AND EVERY STATISTIC
# --------------------------------------------------------------------------
# Pick and place. Retained because it is the only discrete success/failure
# outcome and the only grasp in the programme, and because a capability
# demonstration is worth having. It is NOT part of the mode comparison and
# never can be: with no DIRECT and no VR condition there is no baseline, so no
# completion time, error, workload or learning comparison exists for it.
#
# Run it only if the session is ahead of schedule, and report it as a
# demonstration plus a refusal. See docs/research/09_task2_grasping_finding.md.
#
# Pick points sit at |x| >= 0.30 because that is where a TOP-DOWN grasp is
# kinematically feasible: 0/9 candidates at x = 0.25 and 9/9 at x = 0.30, and
# the failure persists with collision checking OFF, so it is wrist
# orientation, not obstruction.
OPTIONAL_PICK_PLACE = dict(
    key="D-optional",
    name="pick_and_place",
    in_timeline=False,
    comparative=False,
    modes="4+ only (no DIRECT, no VR)",
    arms_used="left, then right (sequential)",
    objects=dict(block=dict(size=(0.040, 0.040, 0.040), mass_g=25,
                            colour="orange")),
    bins=dict(left=[0.30, Y, 1.02], right=[-0.30, Y, 1.02]),
    picks={"left":  [[0.32, Y, 1.15], [0.38, Y, 1.15], [0.44, Y, 1.15]],
           "right": [[-0.32, Y, 1.15], [-0.38, Y, 1.15], [-0.44, Y, 1.15]]},
    standoff_m=0.10,
    metrics=("blocks_placed", "blocks_dropped", "success_rate",
             "cycle_time_s", "grasp_attempts"),
)


def why_this_task(task):
    """The one-line justification that this task earns its slot."""
    return {
        "positioning":
            "the only uncoupled measure, the only one that survives every "
            "master degradation, and the reference the other two are read "
            "against",
        "coordinated_carry":
            "the only task where the two arms are PHYSICALLY coupled, and "
            "the rigid/compliant factor is the one contrast in the set that "
            "changes the FORM of the failure rather than its size",
        "dual_pursuit":
            "the only task that requires simultaneity rather than coupling, "
            "and the only source of b_cross",
    }[task["name"]]
