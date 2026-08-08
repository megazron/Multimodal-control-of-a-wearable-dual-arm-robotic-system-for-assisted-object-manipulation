#!/usr/bin/env python3
"""
kortex_convention.py — the ONE place Kortex angles become ROS angles.

Kortex reports joint positions in DEGREES over [0, 360). ROS uses RADIANS,
and this project wraps continuous joints into (-pi, pi]. Getting this wrong
does not produce a small error: 350 deg read as +350 instead of -10 commands
very nearly a full revolution on a real arm standing next to a person.

Every conversion in the project goes through here so there is exactly one
place to audit, and it is unit-tested at the seam (0 / 180 / 360) where the
mistakes actually live.

RULES
  * Kortex -> ROS : wrap into (-180, 180] first, THEN convert to radians.
  * ROS -> Kortex : convert to degrees, THEN fold into [0, 360).
  * MOVING between two poses uses shortest_delta(), never a plain
    subtraction, so a move from 359 deg to 1 deg is +2 deg and not -358.
"""
import math

TWO_PI = 2.0 * math.pi


def wrap_deg_180(d):
    """Fold any degree value into (-180, 180]."""
    return 180.0 - ((180.0 - d) % 360.0)


def wrap_rad_pi(a):
    """Fold any radian value into (-pi, pi]."""
    return math.pi - ((math.pi - a) % TWO_PI)


def kortex_deg_to_ros_rad(deg):
    """Kortex [0,360) degrees -> ROS (-pi, pi] radians."""
    return math.radians(wrap_deg_180(float(deg)))


def ros_rad_to_kortex_deg(rad):
    """ROS radians -> Kortex [0, 360) degrees."""
    return math.degrees(float(rad)) % 360.0


def kortex_list_to_ros(degs):
    return [kortex_deg_to_ros_rad(d) for d in degs]


def ros_list_to_kortex(rads):
    return [ros_rad_to_kortex_deg(r) for r in rads]


def shortest_delta_rad(target, current):
    """Signed shortest move current -> target, in (-pi, pi].

    This is what stops a homing move taking the long way round the seam.
    """
    return wrap_rad_pi(float(target) - float(current))


def shortest_delta_deg(target, current):
    return wrap_deg_180(float(target) - float(current))


def pose_delta_rad(target, current, continuous_idx=(0, 2, 4, 6)):
    """Per-joint shortest move for a 7-DOF arm.

    Continuous joints (1/3/5/7, i.e. indices 0/2/4/6 in the Gen3) may take
    the short way round; the LIMITED joints must not, because for them
    "wrapping" would mean driving through a hard stop.
    """
    out = []
    for i, (t, c) in enumerate(zip(target, current)):
        out.append(shortest_delta_rad(t, c) if i in continuous_idx
                   else float(t) - float(c))
    return out
