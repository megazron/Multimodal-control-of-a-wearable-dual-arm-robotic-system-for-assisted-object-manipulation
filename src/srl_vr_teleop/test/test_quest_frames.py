#!/usr/bin/env python3
"""Quest -> world frame conversion, pinned against known rotations.

A handedness error does not look like an error. It looks like an arm that
turns the wrong way only for SOME motions, which is exactly the class of bug
that took this project 21.79 deg of "planar ambiguity" to notice once before.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_vr_teleop.quest_vendor_bridge import (          # noqa: E402
    quest_to_world_position, quest_to_world_quat)


def R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_up_maps_to_z():
    """Unity y is UP; this repo's world z is UP."""
    p = quest_to_world_position({"x": 0.0, "y": 1.0, "z": 0.0})
    assert np.allclose(p, [0.0, 0.0, 1.0])


def test_forward_maps_to_y():
    """Unity z is FORWARD; this repo's world y is FORWARD."""
    p = quest_to_world_position({"x": 0.0, "y": 0.0, "z": 1.0})
    assert np.allclose(p, [0.0, 1.0, 0.0])


def test_right_is_unchanged():
    p = quest_to_world_position({"x": 1.0, "y": 0.0, "z": 0.0})
    assert np.allclose(p, [1.0, 0.0, 0.0])


def test_identity_rotation_stays_identity():
    q = quest_to_world_quat({"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
    assert np.allclose(R(q), np.eye(3), atol=1e-9)


def test_conversion_is_a_proper_rotation():
    """det(R) must be +1 for every input. A reflection here would mirror the
    operator's wrist, which reads as 'orientation is wrong sometimes'."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        v = rng.normal(size=4)
        v /= np.linalg.norm(v)
        q = quest_to_world_quat({"x": v[0], "y": v[1], "z": v[2], "w": v[3]})
        assert abs(np.linalg.det(R(q)) - 1.0) < 1e-9


def test_yaw_about_unity_up_becomes_yaw_about_world_z():
    """A turn about the vertical stays a turn about the vertical, same
    magnitude, with the SIGN NEGATED -- and that negation is correct.

    Unity is LEFT-handed: +theta about its up axis is clockwise seen from
    above. This repo's world frame is right-handed: clockwise from above is
    -theta about z. So the same PHYSICAL motion is +theta in Unity and
    -theta in world, and a conversion that preserved the sign would turn the
    operator's wrist the wrong way.

    The property that actually matters -- position and orientation ending up
    in the SAME frame -- is pinned by test_position_and_rotation_agree; if
    these two ever disagree, IK is asked for a pose that does not exist.
    """
    for ang in (0.3, 1.0, -0.7, 2.5):
        h = ang / 2.0
        q = quest_to_world_quat({"x": 0.0, "y": math.sin(h),
                                 "z": 0.0, "w": math.cos(h)})
        M = R(q)
        # the vertical axis is preserved: it is still a pure yaw
        assert np.allclose(M @ np.array([0, 0, 1.0]), [0, 0, 1.0], atol=1e-9)
        got = math.atan2(M[1, 0], M[0, 0])
        err = math.atan2(math.sin(got + ang), math.cos(got + ang))
        assert abs(err) < 1e-6, (ang, got)


def test_position_and_rotation_agree():
    """Rotating a point in Unity then converting must equal converting then
    rotating. If these disagree, position and orientation are in different
    frames and IK will be asked for a pose that does not exist."""
    for ang in (0.4, -1.1):
        h = ang / 2.0
        qu = {"x": 0.0, "y": math.sin(h), "z": 0.0, "w": math.cos(h)}
        pu = {"x": 0.7, "y": 0.2, "z": 0.5}
        c, s = math.cos(ang), math.sin(ang)
        # Unity y-axis rotation acting on (x,y,z), left-handed about up
        rot_u = {"x": c * pu["x"] + s * pu["z"], "y": pu["y"],
                 "z": -s * pu["x"] + c * pu["z"]}
        a = quest_to_world_position(rot_u)
        b = R(quest_to_world_quat(qu)) @ quest_to_world_position(pu)
        assert np.allclose(a, b, atol=1e-9), (a, b)
