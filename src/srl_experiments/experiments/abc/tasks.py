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
# TASK B -- T3 COORDINATED CARRY (rigid only)
# --------------------------------------------------------------------------
# T3 REPLACES THE MERGED TASK B (2026-08-11).  The slot keeps the key "B"
# because that key is the dispatcher's, the GUI's and run_experiment.sh's --
# renaming it is exactly the "button labelled T3 runs something else" failure
# the archive README exists to prevent.  The CONTENT is T3 and only T3.
#
# What was removed: the COMPLIANT (sling) half.  The merged task ran the same
# three paths twice, once with a rigid tray and once with a 540 mm sling, as
# a within-task factor -- 18 trial slots where T3 alone is 9.  The contrast
# was real and is not being called worthless; it is being spent elsewhere.
# The superseded spec is kept verbatim, with the arithmetic that supported
# it, at experiments/_archive/task_b_rigid_and_compliant/.
#
# WHICH T3 THIS IS, because there are two and only one is usable.  The
# archived bimanual T3 declares its grips at |x| = 0.155 on a 310 mm span --
# INSIDE the 0.25 m dead band between the disjoint reachable sets, where it
# failed 10 of 25 waypoints.  Reviving those coordinates would reinstate a
# regression this repository already documents.  So what is reinstated is
# T3's MECHANISM -- two grippers on one rigid body, a loose ball making the
# coordination error visible -- at the CURRENT verified 500 mm geometry.
#
# A coupled object is held at two points TRAY_SEP apart, one gripper on each
# end.  It SPANS the dead band, which is exactly why it needs both arms: no
# single arm can hold a body at two separated points, and no amount of time
# changes that.
#
# FAILURE IS TILT.  Over 500 mm, 20 mm of height difference is 2.3 deg
# (visible wobble) and 60 mm is 6.8 deg, at which the ball rolls off.
# Rescaled from the superseded 310 mm span, where the same millimetres read
# 3.7 and 11.0 deg -- the ANGLE is the criterion, so it MUST be recomputed
# whenever the span changes.
#
# TRANSPORT IS VERTICAL, and that is a measured constraint, not a preference:
# the feasible band is only 0.35-0.40 m deep in y, so a fore-aft carry leaves
# it immediately (measured 0/20 carrying toward the wearer, against 20/20 for
# the grips and the vertical lift).
TASK_B = dict(
    key="B",
    name="t3_coordinated_carry",
    supersedes="coordinated_carry (rigid AND compliant); the merged spec is "
               "kept at experiments/_archive/task_b_rigid_and_compliant/",
    role="physical coupling through a shared rigid body",
    # WHY COUPLING AND NOT TRANSFER, in one measured number.  The smallest
    # distance between an end-effector pose the LEFT arm can reach and one the
    # RIGHT arm can reach is 0.200 m, over 1488 poses per arm per orientation
    # with the bench in scene and all four solo controls non-zero.  A cube
    # crossing between two container rims needs about 0.115 m (75 mm container
    # + 40 mm cube).  Transfer fails by a factor of 1.7, and that one number
    # settles rim-to-rim, tilted and stacked together, because every transfer
    # geometry needs the grippers NEAR each other.
    #
    # The 500 mm span is therefore not a workaround for that result -- it is
    # the same geometry used the other way round.  Each grip sits at
    # |x| = 0.25, OUTSIDE the dead band, so the object SPANS the gap instead
    # of asking the two arms to meet inside it.  Remove one arm and the task
    # is impossible rather than slower, which is the stronger claim: under
    # simultaneity a participant working sequentially merely degrades, and
    # contaminates the dependent variable with strategy.
    why_not_transfer="min inter-arm EE separation 0.200 m vs 0.115 m needed; "
                     "see docs/research/13_two_arm_transfer_is_not_achievable.md",
    bimanual="YES, route (a) -- a body held at two points 500 mm apart spans "
             "the dead band between the disjoint reachable sets.",
    # THE DISTINCTION THAT MUST NOT BE BLURRED IN THE WRITE-UP.
    # This project now has TWO tasks that use both arms, and they support
    # DIFFERENT claims:
    #
    #   T2 (this task)  PHYSICAL COUPLING. One body, two grips, and neither
    #                   arm's pose is free given the other's -- the tray is
    #                   rigid, so a height difference at one grip IS a tilt at
    #                   the other. Remove one arm and the task becomes
    #                   IMPOSSIBLE, not slower. Tilt RMS and separation error
    #                   are JOINT metrics: no single arm can produce them,
    #                   because they are defined on the relationship between
    #                   the two.
    #
    #   T3              BIMANUAL BY ROLE. Two objects, two places, two arms --
    #                   the right holds the circuit box, the left presents the
    #                   meter. Neither arm's pose constrains the other's. One
    #                   arm could do both sequentially; it would be slower and
    #                   the wearer would wait, but it would not be impossible.
    #
    # Coupling is the stronger claim, and only T2 supports it. A sentence like
    # "the bimanual tasks showed X" is therefore ambiguous and must not be
    # written: name the task, and say which kind of bimanual it is.
    bimanual_kind="physical coupling -- the stronger claim; see T3 for the "
                  "weaker by-role sense, and do not merge the two",
    arms_used="both, simultaneously",
    objects=dict(
        tray=dict(size=(TRAY_SEP + 0.06, 0.26, 0.02), grip_sep=TRAY_SEP,
                  colour="tan", mass_g=180, lip_mm=8),
        ball=dict(radius=BALL_R, colour="yellow", mass_g=15),
    ),
    # RE-SPECIFIED 2026-08-11.  THE OLD BAND WAS INSIDE THE BENCH.
    #
    # The declared band was z 1.10-1.30 and it had never been checked against
    # the bench it is carried over: verify_abc_scenarios verifies the STUDY
    # tasks in FREE SPACE and applies the furniture only to the clip tasks.
    # Two checks therefore disagreed -- 0 failures against 21 -- and the free-
    # space one was the broken one.  Settled three ways:
    #
    #   ARITHMETIC     two declared grip poses, S2's start at (+/-0.25, 0.35,
    #                  1.10), lie INSIDE the slab (z 1.06-1.10, y 0.245-0.63).
    #                  No solver involved, so it cannot be a solver artefact.
    #   PER WAYPOINT   N=10, both arm assignments, bench in scene:
    #                  S1 4/4 fail, S2 9/11 fail, S3 8/10 fail = 21, exactly
    #                  the number the regression block reported.
    #   CLEAR BAND     feasible from z = 1.28 at y = 0.35, and from z = 1.30
    #                  at y = 0.38, up to at least 1.40.
    #
    # So the band moves to 1.32-1.40, holding 20 mm inside the lowest clear z
    # at each y -- this project's rule that N finds the boundary and MARGIN is
    # what keeps you off it.  The tilt threshold is unaffected: 6.8 deg is
    # 60 mm over the 500 mm SPAN and does not depend on height.
    # RAISED 2026-08-11. THE OLD TOP WAS NOT A LIMIT, IT WAS WHERE THE SURVEY
    # STOPPED LOOKING.
    #
    # The band was re-spec'd from 1.10-1.30 to 1.32-1.40 by a survey that
    # established the BOTTOM properly and recorded the top only as "up to at
    # least 1.40", because 1.40 was as high as it sampled. The consequence was
    # that T2's entire commanded motion was EIGHTY MILLIMETRES -- the whole
    # coordinated carry, the task whose claim is that neither arm's pose is
    # free given the other's, is not visible in a clip at that scale.
    #
    # Measured upward the same way -- both grippers 500 mm apart at y = 0.35,
    # BOTH arm assignments, N=10, T2's own scene, control at z = 2.20
    # correctly unreachable (scripts/measure_t2_band_top.py):
    #
    #     z = 1.32 .. 1.70 in 20 mm steps: EVERY ONE PASSES
    #
    # so the ceiling is at least 1.70 and was never 1.40. The paths below use
    # 1.60, which is 100 mm inside the last z that passed N/N -- this project's
    # rule that N locates the boundary and MARGIN is what keeps you off it --
    # and turns the full lift from 80 mm into 280 mm.
    #
    # The tilt threshold is UNCHANGED and must be: 6.8 deg is 60 mm over the
    # 500 mm SPAN, and does not depend on height.
    paths=dict(
        S1_short_lift=[[0.0, Y, 1.32], [0.0, Y, 1.42]],
        S2_full_lift=[[0.0, Y, 1.32], [0.0, Y, 1.60]],
        S3_detour=[[0.0, Y, 1.32], [0.0, 0.38, 1.42],
                   [0.0, 0.38, 1.52], [0.0, Y, 1.60]],
    ),
    band_z=(1.32, 1.60),
    band_rationale="lowest clear z measured 1.28 (y=0.35) / 1.30 (y=0.38) "
                   "with the bench in scene, +20 mm margin; top measured "
                   "clear to 1.70 at N=10, held 100 mm inside it",
    grip_sep=TRAY_SEP,
    fail_tilt_deg=6.8,               # 60 mm over 500 mm
    repeats=3,
    metrics=("tilt_rms_deg", "tilt_max_deg", "time_above_fail_tilt_s",
             "height_diff_rms_mm", "height_diff_max_mm",
             "sep_err_rms_mm", "sep_err_max_mm",
             "path_efficiency", "object_retained", "corrective_movements",
             "collisions", "near_collisions"),
    # TILT AND SEPARATION ARE LOGGED CONTINUOUSLY, not scored pass/fail at the
    # end.  The continuous trace IS the measurement: a carry that ends level
    # having been 9 deg out in the middle is not a successful carry, and a
    # single end-state sample cannot tell the two apart.  The sample rate is
    # the trial logger's, and `tilt_max_deg` comes from the trace rather than
    # from the endpoint.
    log_continuously=("tilt_deg", "sep_err_mm", "height_diff_mm"),
)

# The compliant half, kept as ARITHMETIC rather than as a task, because the
# sling threshold is still the reference the rigid tilt threshold is read
# against and deleting it would strand that comparison.
#   sag(s) = sqrt((L/2)^2 - (s/2)^2); the ball is retained while sag >= 2r,
#   so with L = 0.540 and r = 0.020, s_max = 0.5340 m.
ARCHIVED_COMPLIANT = dict(
    sling_length_m=SLING_L, ball_radius_m=BALL_R, grip_sep_m=TRAY_SEP,
    fail_sep_m=0.5340,
    where="experiments/_archive/task_b_rigid_and_compliant/",
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
        "t3_coordinated_carry":
            "the only task where the two arms are PHYSICALLY coupled -- a "
            "rigid body held at two points 500 mm apart, which no single arm "
            "can do and no amount of time can substitute for",
        "dual_pursuit":
            "the only task that requires simultaneity rather than coupling, "
            "and the only source of b_cross",
    }[task["name"]]
