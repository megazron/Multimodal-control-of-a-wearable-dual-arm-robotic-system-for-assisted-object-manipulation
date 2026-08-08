#!/usr/bin/env python3
"""
clearance.py — distance from a candidate arm posture to the wearer.

MoveIt's /check_state_validity reports whether a state collides, but not by
how much, and the thing we actually need is a MARGIN: "do not publish if the
gripper is within 5 cm of the operator". So this asks the service with a
padded collision model and, separately, computes a geometric distance from
the arm's distal links to the wearer's body primitives via TF.

The geometric path is deliberately simple and conservative -- capsules and
boxes taken straight from human_backpack.xacro. It is a floor, not a
simulation: it never reports MORE clearance than the true value for the
shapes it models.
"""
import math

import numpy as np

# Wearer body primitives, in the `torso` frame, straight from
# human_backpack.xacro. (kind, params, origin)
#   sphere:   radius
#   cylinder: (radius, length)  -- axis along z
#   box:      (sx, sy, sz)
TORSO_BODY = [
    ("box", (0.36, 0.22, 0.48), (0.0, 0.0, 0.17)),          # torso
]
HEAD_BODY = [
    ("sphere", (0.105,), (0.0, 0.0, 0.245)),
    ("cylinder", (0.055, 0.16), (0.0, 0.0, 0.10)),
]
HIPS_BODY = [
    ("box", (0.32, 0.21, 0.18), (0.0, 0.0, 0.17)),
]

# Arm links that must stay clear. Proximal links are bolted to the harness
# and are excluded in the SRDF, so checking them would only produce noise.
DISTAL_LINKS = ["forearm_link", "spherical_wrist_1_link",
                "spherical_wrist_2_link", "bracelet_link",
                "end_effector_link"]


def _dist_point_box(p, half):
    """Distance from a point to an axis-aligned box centred at the origin."""
    d = np.maximum(np.abs(p) - np.asarray(half, dtype=float), 0.0)
    inside = float(np.min(np.asarray(half) - np.abs(p)))
    n = float(np.linalg.norm(d))
    return n if n > 0.0 else -inside


def _dist_point_cyl(p, radius, length):
    """Distance from a point to a z-axis capsule-ish cylinder at the origin."""
    radial = math.hypot(p[0], p[1]) - radius
    axial = abs(p[2]) - length * 0.5
    if radial <= 0 and axial <= 0:
        return max(radial, axial)
    return math.hypot(max(radial, 0.0), max(axial, 0.0))


def _dist_point_sphere(p, radius):
    return float(np.linalg.norm(p)) - radius


# Real operators differ in build from the mannequin these primitives were
# traced from, so real_robot mode inflates every shape by this margin.
REAL_ROBOT_PAD_M = 0.05


def point_clearance(p_local, prims, pad=0.0):
    """Smallest distance from a point to a list of primitives, same frame."""
    best = float("inf")
    for kind, params, origin in prims:
        q = np.asarray(p_local, dtype=float) - np.asarray(origin, dtype=float)
        if kind == "box":
            d = _dist_point_box(q, [c * 0.5 + pad for c in params])
        elif kind == "cylinder":
            d = _dist_point_cyl(q, params[0] + pad, params[1] + 2 * pad)
        else:
            d = _dist_point_sphere(q, params[0] + pad)
        best = min(best, d)
    return best


class ClearanceModel:
    """Clearance of a set of arm link origins from the wearer.

    Points are supplied in each body part's own frame by the caller (which
    has TF), so this stays pure geometry and is unit-testable.
    """

    PARTS = {"torso": TORSO_BODY, "head": HEAD_BODY, "hips": HIPS_BODY}

    def clearance(self, points_by_part, pad=0.0):
        """points_by_part: {part_name: [p, ...]} in that part's frame."""
        best = float("inf")
        worst_part = None
        for part, pts in points_by_part.items():
            prims = self.PARTS.get(part)
            if not prims:
                continue
            for p in pts:
                d = point_clearance(p, prims, pad)
                if d < best:
                    best, worst_part = d, part
        return best, worst_part
