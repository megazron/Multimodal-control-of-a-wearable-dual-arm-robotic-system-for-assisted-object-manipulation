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

try:
    from srl_teleop import wearer_posture as _WP
except Exception:                                              # noqa: BLE001
    from . import wearer_posture as _WP

# Wearer body primitives, in the `torso` frame, straight from
# human_backpack.xacro. (kind, params, origin)
#   sphere:   radius
#   cylinder: (radius, length)  -- axis along z
#   box:      (sx, sy, sz)
# THE WEARER'S SIZE REACHED THIS FILE THROUGH NOTHING, AND THIS IS THE LIVE
# CHECK.
#
# `SRL_WEARER_SIZE` has been a variable since 2026-08-15 and docs/ENGINEERING_LOG.md
# describes it as going "through the SAME one source the posture uses". It
# reached `mount_guard_node.WEARER`, which is an OFFLINE guard, and it did not
# reach here -- and this module is what `ik_follower_node`, `real_homing_node`
# and `sim_to_real_bridge` enforce a floor with. Measured before the fix:
#
#     SRL_WEARER_SIZE=measured_adult
#     mount_guard torso box    0.430 x 0.220 x 0.503     follows the profile
#     clearance.py torso box   0.360 x 0.220 x 0.480     the mannequin
#     clearance.py upper arm   0.300 m long              the mannequin
#
# So the arm was stopped at 150 mm from a body 70 mm narrower than the one
# configured, and every consumer reported the floor as held. That is docs/ENGINEERING_LOG.md
# rule 11's "a posture that reaches one and not the other measures the old
# wearer under a new name", one level down and on the live path.
#
# The dimensions are now built from `wearer_posture`, per instance, at
# construction. The ORIGINS are unchanged and stay hardcoded: they are offsets
# within each body part's OWN LINK FRAME, TF places the frames, and they are
# not a function of size.
def parts_for(size=None):
    """Every wearer primitive, in its own link frame, at the given size.

    `size` is a profile name, a path or a dict; None means SRL_WEARER_SIZE,
    whose default is the shipped mannequin -- so a caller that asks for
    nothing gets exactly the body this file has always carried.
    """
    # A dict is a profile already. Same idiom as `wearer_posture.arm_links`
    # and `torso_parts_for`, so a caller can hand any of the three the same
    # thing -- a name, a path, or a body it built itself.
    s = size if isinstance(size, dict) else _WP.size_profile(size)
    out = {
        "torso": [("box", (s["chest_w"], s["chest_d"], s["chest_h"]),
                   (0.0, 0.0, 0.17))],
        "head": [("sphere", (s["head_r"],), (0.0, 0.0, 0.245)),
                 ("cylinder", (0.055, 0.16), (0.0, 0.0, 0.10))],
        "hips": [("box", (s["hips_w"], s["hips_d"], 0.18),
                  (0.0, 0.0, 0.17))],
    }
    for name, kind, dims, _len, _mat in _WP.segments_for(s):
        for side in ("left", "right"):
            out["human_%s_%s" % (side, name)] = [
                (kind, tuple(dims), (0.0, 0.0, 0.0))]
    return out


# The shipped body, kept as module constants because callers import them.
# They are DERIVED from the mannequin profile rather than retyped, so there is
# no second copy to drift.
_MANNEQUIN = parts_for("mannequin")
TORSO_BODY = _MANNEQUIN["torso"]
HEAD_BODY = _MANNEQUIN["head"]
HIPS_BODY = _MANNEQUIN["hips"]

# THE WEARER'S OWN ARMS, AND THEY WERE NOT HERE.
#
# This module is the check the LIVE stack runs -- ik_follower_node's hard
# floor, real_homing_node and sim_to_real_bridge all import it -- and until
# now it modelled a torso, a head and a pair of hips. The offline geometric
# survey has always included the wearer's arms, and on the inboard side those
# arms are the thing that binds: the forearm hangs at |x| = 0.21 and is what
# every "innermost safe column" number in this project is measured against. So
# the planner's wearer and the follower's wearer were different people, and the
# follower's had no arms.
#
# Each primitive is expressed in ITS OWN LINK FRAME, where it sits at the
# origin with its axis along z. That is why this table needs no posture: the
# posture moves the LINK, TF carries it, and the shape in the link frame is the
# same whatever the wearer is doing with their arms. The dimensions come from
# `wearer_posture.SEGMENTS`, which is also what writes the URDF.
WEARER_ARM_PARTS = {k: v for k, v in _MANNEQUIN.items()
                    if k.startswith("human_")}

# Arm links that must stay clear. Proximal links are bolted to the harness
# and are excluded in the SRDF, so checking them would only produce noise.
DISTAL_LINKS = ["forearm_link", "spherical_wrist_1_link",
                "spherical_wrist_2_link", "bracelet_link",
                "end_effector_link"]

# THE PROXIMAL LINKS ARE THE ONES A SHOULDER MOUNT ACTUALLY SWINGS THROUGH.
#
# DISTAL_LINKS starts at the forearm, so the shoulder and both half-arm tubes
# -- the segments that sweep across the wearer's head and chest when the arm
# rotates about its base -- were never measured against the body at all. The
# hand can be a metre clear while the upper tube is inside somebody's neck,
# and the check would report the hand's distance and call it clearance.
#
# This is the SRDF trap one layer down: docs/ENGINEERING_LOG.md hard constraint 11 says the
# SRDF excludes the 44 proximal pairs a shoulder mount threatens, and the
# homing check then reproduced the same blind spot in its own link list.
#
# base_link is deliberately NOT here. It is rigid to the mount at 0.1610 m
# from the torso and no joint moves it, so including it would peg every
# reading at 0.1610 and mask the arm entirely -- a constant dressed as a
# measurement.
PROXIMAL_LINKS = ["shoulder_link", "half_arm_1_link", "half_arm_2_link"]

# What the wearer check should sweep: everything that moves.
WEARER_CHECK_LINKS = PROXIMAL_LINKS + DISTAL_LINKS


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

    #: The shipped body, for callers that read the class rather than an
    #: instance. An INSTANCE built with a size overrides it.
    PARTS = dict(_MANNEQUIN)

    def __init__(self, size=None):
        """`size` is a wearer profile. None means SRL_WEARER_SIZE.

        The body is fixed at construction rather than looked up per call, so
        one node cannot measure two different people during a run.
        """
        self.PARTS = parts_for(size)
        self.size = size if isinstance(size, dict) else _WP.size_profile(size)

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
