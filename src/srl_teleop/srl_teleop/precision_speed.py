#!/usr/bin/env python3
"""ONE DIAL: precision at one end, speed at the other. Safety at neither.

The operator should not have to think in three parameters. Fine work wants a
small motion scale, heavy smoothing and a low velocity cap; covering ground
wants the opposite. Those three always move together, so they are one control.

    dial = 0.0   PRECISION   small scale, heavy smoothing, slow
    dial = 1.0   SPEED       full scale, light smoothing, fast

THE INVARIANT, AND IT IS THE WHOLE POINT
----------------------------------------
`SAFETY_PARAMS` is the list of things this dial MUST NOT TOUCH, and
`assert_safety_unconditional()` raises if a proposed settings dict contains
any of them. It is not a comment: the dial is an operator convenience, and the
clearance floor, the e-stop, the collision checks and the dead-man are what
stand between the arms and the chest of someone who did not choose the motion.
An operator in a hurry must not be able to trade those away, and must not be
able to do so BY ACCIDENT because a future edit added a key to the dict.

Note what this means at dial=1.0: the arm moves faster and therefore reaches
the floor sooner, but the floor is in exactly the same place. Speed changes
how quickly a wrong motion happens, not how far it is allowed to go.

WHY THE MAPPING IS NOT LINEAR IN SCALE. Perceived precision goes as the
motion per unit of hand travel, and the useful part of the range is bunched at
the low end -- 0.2 to 0.4 is a large change in feel, 0.8 to 1.0 is barely
noticeable. The scale is therefore geometric between its endpoints, which
spreads the useful resolution across the dial instead of crowding it.
"""
import math

# What the dial IS allowed to move, with its endpoints.
#   name: (value at dial=0 PRECISION, value at dial=1 SPEED)
RANGE = {
    "scale":         (0.20, 1.00),    # commanded metres per master metre
    "ema_alpha":     (0.08, 0.50),    # smoothing: low = heavy
    "max_vel_rad_s": (0.10, 0.60),    # follower joint velocity cap
    "max_step_rad":  (0.10, 0.35),    # per-cycle slew limit
}

# What it is NEVER allowed to move. Adding a key here is free; removing one
# is a safety change and should look like one in the diff.
SAFETY_PARAMS = frozenset({
    "min_clearance_m",          # the hard floor against the wearer
    "wearer_pad_m",
    "avoid_collisions",         # collision-aware IK
    "redundancy_samples",       # graduated avoidance before the floor
    "stale_timeout_s",          # dead-man
    "deadman_enabled",
    "estop_enabled",
    "flip_reject_limit",
    "pending_timeout_s",        # in-flight IK watchdog
})


def clamp01(x):
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else float(x)


def settings(dial):
    """The three-or-four numbers for a dial position in [0, 1]."""
    d = clamp01(dial)
    out = {}
    for k, (lo, hi) in RANGE.items():
        if k == "scale":
            # geometric, so the low end gets the resolution
            out[k] = float(lo * (hi / lo) ** d)
        else:
            out[k] = float(lo + (hi - lo) * d)
    return out


def assert_safety_unconditional(s):
    """Raise if a settings dict would move anything safety-relevant."""
    bad = SAFETY_PARAMS.intersection(s)
    if bad:
        raise ValueError(
            "the precision/speed dial tried to set %s. Safety does not scale "
            "with the dial: the clearance floor, the collision checks, the "
            "dead-man and the e-stop are identical at every dial position. "
            "If this needs to change it is a safety change and belongs in a "
            "review, not in an operator control." % sorted(bad))
    return True


def rebase_anchor(pos_anchor, current, reference, old_scale, new_scale):
    """Move the anchor so changing scale does NOT move the arm.

    The commanded pose is `pos_anchor + scale * (current - reference)`. Change
    the scale alone and the arm jumps by `(new - old) * displacement`, worst
    exactly when the operator is furthest from where they engaged -- which is
    when they are least able to predict it. Absorbing the difference into the
    anchor makes the command continuous through the change.

    This is the same correction the motion-scale parameter already used; the
    dial reuses it rather than inventing a second one that could disagree.
    """
    disp = [c - r for c, r in zip(current, reference)]
    return [a + (old_scale - new_scale) * d for a, d in zip(pos_anchor, disp)]


def describe(dial):
    s = settings(dial)
    end = ("PRECISION" if dial < 0.34 else
           "SPEED" if dial > 0.66 else "BALANCED")
    return ("dial %.2f %-9s scale %.2f  smoothing %.2f  vel %.2f rad/s  "
            "step %.2f rad  | floor UNCHANGED"
            % (dial, end, s["scale"], s["ema_alpha"], s["max_vel_rad_s"],
               s["max_step_rad"]))
