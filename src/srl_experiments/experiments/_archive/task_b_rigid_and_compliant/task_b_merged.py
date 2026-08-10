"""SUPERSEDED 2026-08-11 -- the merged Task B, rigid AND compliant.

Kept verbatim, exactly as it stood in tasks.py at commit e642506, so the
arithmetic behind its two failure thresholds survives with it.  Replaced
in the live set by T3 (rigid only); see the README beside this file.

IMPORTS NOTHING AND IS IMPORTED BY NOTHING.  It is a record, not a
module -- the constants it references (TRAY_SEP, SLING_L, BALL_R, Y)
live in the live tasks.py and are NOT redefined here on purpose, so
this file cannot quietly become a second source for them.
"""

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

