#!/usr/bin/env python3
"""THE FIVE-TASK SET. Consolidated from nine.

Nine tasks could not be run on a participant. These five are chosen so that
each earns its place by measuring something the others cannot.

THE BINDING GEOMETRIC FACT. The two arms' reachable sets are DISJOINT: at
y = 0.35, z = 1.10 the right arm reaches x <= -0.10 and the left reaches
x >= +0.15, with a 0.25 m dead band between them (measured, N=3, yaw-free).
Nothing may therefore require both arms at the SAME point -- inter-arm
handover is impossible on this platform. A task can only require both arms if
either (a) a rigid object SPANS the gap and must be held at two points at
once, or (b) two targets in the two disjoint sets must be satisfied
SIMULTANEOUSLY.

Both routes are used below. Tasks 3 and 4 take route (a); task 5 takes route
(b). In every case one arm alone cannot complete the task given unlimited
time -- which is the criterion that was set.
"""

Y = 0.35                    # the only fore/aft band both arms can work in
# RESPEC'D 2026-08-08. The 310 mm span put each grip at |x| = 0.155, which a
# home-verified reachability run shows is inside the dead band between the two
# disjoint sets -- 10 of 25 waypoints failed. 500 mm puts each grip at
# |x| = 0.25, which the clearance probe independently identified as the
# smallest half-span clearing the 120 mm wearer floor (0.134 m at 6/6
# reachable). One change satisfies both constraints.
TRAY_SEP = 0.500
# The sling must lengthen with the span or it is taut before the trial starts:
# retention needs sag >= 2r, i.e. L >= 2*sqrt((s/2)^2 + (2r)^2). For s = 0.50
# and r = 0.020 that is L >= 0.5064, so 0.540 leaves a 34 mm margin and keeps
# the failure threshold INSIDE the reachable band, which is the property that
# makes the task able to fail at all.
SLING_L = 0.540

# ---------------------------------------------------------------------------
# TASK 1 -- POSITIONING (Fitts characterisation and warm-up)
# ---------------------------------------------------------------------------
# No objects. Three blocks: left alone, right alone, both simultaneously. The
# single-versus-both contrast is the attention bottleneck measured directly,
# and it needs no object to exist, so it is also the safest warm-up.
#
# Target positions sit inside each arm's own verified set. Widths give three
# indices of difficulty: ID = log2(2A/W).
TASK1 = dict(
    name="positioning",
    arms_used="left | right | both",
    objects=None,
    conditions=("left_only", "right_only", "both"),
    targets={
        "left":  [[0.30, Y, 1.05], [0.30, Y, 1.25], [0.45, Y, 1.15]],
        "right": [[-0.30, Y, 1.05], [-0.30, Y, 1.25], [-0.45, Y, 1.15]],
    },
    widths_mm=(60, 30, 15),          # three IDs at each amplitude
    dwell_s=0.5,                     # must be held inside the target
    metrics=("movement_time_s", "throughput_bits_s", "overshoot_mm",
             "dwell_failures", "dual_task_cost"),
)

# ---------------------------------------------------------------------------
# TASK 2 -- PICK AND PLACE, ONE ARM AT A TIME
# ---------------------------------------------------------------------------
# Left picks and places, then right. NOT simultaneous. Gives the single-arm
# manipulation baseline and a discrete success/failure outcome, which none of
# the continuous-metric tasks provide.
#
# Pick points sit at |x| >= 0.30 because that is where a TOP-DOWN grasp is
# kinematically feasible: measured 0/9 candidates at x = 0.25 and 9/9 at
# x = 0.30, and the failure persists with collision checking off, so it is
# wrist orientation and not obstruction.
TASK2 = dict(
    name="pick_and_place",
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

# ---------------------------------------------------------------------------
# TASK 3 -- RIGID COORDINATED CARRY  (genuinely bimanual, route (a))
# ---------------------------------------------------------------------------
# A rigid tray is held at two points 310 mm apart, one gripper on each end.
# The object SPANS the dead band between the two reachable sets, which is
# exactly why it needs both arms: no single arm can hold a rigid body at two
# separated points, and no amount of time changes that.
#
# The failure mode is TILT: the tray is rigid, so any height difference
# between the grippers transmits instantly. 20 mm over a 300 mm span is
# 3.8 deg (visible wobble); 60 mm is 11.3 deg and the ball rolls off.
TASK3 = dict(
    name="rigid_carry",
    arms_used="both, simultaneously",
    objects=dict(tray=dict(size=(TRAY_SEP + 0.06, 0.26, 0.02),
                           grip_sep=TRAY_SEP, colour="tan"),
                 ball=dict(radius=0.020, colour="yellow")),
    paths=dict(
        S1_short_lift=[[0.0, Y, 1.15], [0.0, Y, 1.20]],
        S2_full_lift=[[0.0, Y, 1.10], [0.0, Y, 1.30]],
        S3_detour=[[0.0, Y, 1.15], [0.0, 0.38, 1.20],
                   [0.0, 0.38, 1.25], [0.0, Y, 1.30]],
    ),
    # tilt rescales with the span: 60 mm over 500 mm is 6.8 deg, not 11.3
    fail_tilt_deg=6.8,
    metrics=("tilt_rms_deg", "tilt_max_deg", "time_above_11_3_s",
             "height_diff_rms_mm", "path_efficiency", "ball_retained"),
)

# ---------------------------------------------------------------------------
# TASK 4 -- COMPLIANT SLING CARRY  (the contrast with Task 3)
# ---------------------------------------------------------------------------
# Identical paths, identical grip separation, DIFFERENT object. A flexible
# sling of length L held s apart hangs with sag
#
#       sag(s) = sqrt((L/2)^2 - (s/2)^2)
#
# and retains a ball of radius r while sag >= 2r, giving
# s_max = 2*sqrt((L/2)^2 - (2r)^2) = 534.0 mm for L = 540, r = 20.
#
# L = 540 mm is correct PRECISELY BECAUSE the failure threshold sits INSIDE
# the reachable band (the measured reachable separation band): a longer sling could never fail and would
# measure nothing. Rigid coupling transmits error instantly; compliant
# coupling absorbs it and then fails abruptly, and that difference is the
# scientific point of running 3 and 4 together.
TASK4 = dict(
    name="compliant_carry",
    arms_used="both, simultaneously",
    objects=dict(sling=dict(length=SLING_L, colour="brown"),
                 ball=dict(radius=0.020, colour="yellow")),
    paths=None,                      # identical to TASK3["paths"]
    nominal_sep=TRAY_SEP,
    fail_sep_m=0.5340,
    metrics=("sep_err_rms_mm", "sep_err_max_mm", "min_sag_mm",
             "time_above_threshold_s", "ball_retained"),
)

# ---------------------------------------------------------------------------
# TASK 5 -- SYNCHRONISED DUAL-TARGET PURSUIT  (route (b))
# ---------------------------------------------------------------------------
# Each arm tracks its own moving target inside its own reachable set. The
# targets are in the two DISJOINT sets, so one arm can never satisfy both --
# not sequentially, not with unlimited time. Success requires both to be
# within tolerance SIMULTANEOUSLY for a sustained fraction of the trial.
#
# Target speeds vary INDEPENDENTLY across trials. That is not a detail: with
# the two speeds yoked, the self and cross terms in
#
#   err_A ~ b0 + b_self*speed_A + b_cross*speed_B + b_int*speed_A*speed_B
#
# are perfectly collinear and neither is estimable. b_cross is the cross-arm
# interference -- the attention bottleneck measured directly.
TASK5 = dict(
    name="dual_pursuit",
    arms_used="both, simultaneously",
    objects=dict(target=dict(radius=0.050, colour="green")),
    centres={"left": [0.35, Y, 1.15], "right": [-0.35, Y, 1.15]},
    amplitude_m=0.08,
    speeds_m_s=dict(S1_both_slow=(0.05, 0.05), S2_one_fast=(0.20, 0.05),
                    S3_both_fast=(0.20, 0.20), S4_asymmetric=(0.30, 0.08)),
    baselines=dict(B1_left_only=(0.20, 0.0), B2_right_only=(0.0, 0.20)),
    tolerance_mm=40,
    metrics=("rms_error_mm_left", "rms_error_mm_right", "frac_both_on_target",
             "phase_lag_s", "b_cross", "dual_task_cost"),
)

ALL = [TASK1, TASK2, TASK3, TASK4, TASK5]


def why_bimanual(task):
    """The one-line justification that a task needs both arms."""
    return {
        "positioning": "not bimanual by requirement; the both-arms block is "
                       "the attention contrast, not a coupling constraint",
        "pick_and_place": "not bimanual by requirement; single-arm baseline",
        "rigid_carry": "a rigid body held at two points 500 mm apart, which "
                       "spans the dead band between the disjoint reachable "
                       "sets. One arm cannot hold two separated points",
        "compliant_carry": "as rigid_carry; the sling must be tensioned "
                           "between two grippers to retain the ball at all",
        "dual_pursuit": "the two targets lie in DISJOINT reachable sets, so "
                        "one arm can never reach both, and simultaneity is "
                        "the success criterion",
    }[task["name"]]
