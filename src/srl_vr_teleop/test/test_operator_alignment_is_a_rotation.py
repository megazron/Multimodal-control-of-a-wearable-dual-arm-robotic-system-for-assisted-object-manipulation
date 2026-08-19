#!/usr/bin/env python3
"""The operator-to-robot alignment must be a ROTATION, never a mirror.

THE SETUP THIS EXISTS FOR. The operator sits across the room FACING the
wearer, holding the controllers as a motion-capture input. Their frame is
turned about the vertical relative to the robot's -- roughly 180 deg.

THE TRAP. "The operator faces the wearer, so mirror it" is the intuitive fix
and it is wrong. Facing someone and copying them is a REFLECTION (det = -1).
Applied to position alone it looks correct; applied to the whole pose it
mirrors every ORIENTATION, so the gripper rolls the wrong way while the
positions look right. This project has already lost time to exactly this
class of error once -- see test_quest_frames.py's note on 21.79 deg of
"planar ambiguity" -- and a wrist that turns the wrong way only for SOME
motions is the hardest possible thing to notice from a video.

So the alignment is exposed as ONE ANGLE, not as per-axis sign flips. No
value of `align_yaw_deg` can produce a reflection, and these tests pin that.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_vr_teleop.vr_pose_mapper import (                       # noqa: E402
    q_conj, q_mul, q_norm, yaw_matrix, yaw_quat)


def R_of(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ------------------------------------------------------- it is a rotation
@pytest.mark.parametrize("deg", [0, 45, 90, 180, 270, -137.5, 360])
def test_yaw_matrix_is_a_proper_rotation(deg):
    M = yaw_matrix(deg)
    assert abs(np.linalg.det(M) - 1.0) < 1e-12, "a reflection would mirror " \
        "every orientation while the positions still looked right"
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-12)


@pytest.mark.parametrize("deg", [0, 30, 180, -90])
def test_yaw_quat_matches_yaw_matrix(deg):
    """Position and orientation must be aligned by the SAME transform. If
    these two ever disagree, IK is asked for a pose that does not exist."""
    assert np.allclose(R_of(yaw_quat(deg)), yaw_matrix(deg), atol=1e-12)


def test_yaw_preserves_the_vertical():
    """Whatever the operator's heading, up is still up. A yaw that tilted the
    vertical would send the arm into the table or the wearer."""
    for deg in (0, 37, 180, -95):
        assert np.allclose(yaw_matrix(deg) @ np.array([0, 0, 1.0]),
                           [0, 0, 1.0], atol=1e-12)


def test_yaw_preserves_length():
    rng = np.random.default_rng(0)
    for _ in range(50):
        v = rng.normal(size=3)
        for deg in (0, 61, 180, -212):
            assert abs(np.linalg.norm(yaw_matrix(deg) @ v)
                       - np.linalg.norm(v)) < 1e-12


# ------------------------------------------- the facing-the-wearer case
def test_180_reverses_both_horizontal_axes_not_just_one():
    """The actual answer for an operator facing the wearer.

    It flips left/right AND forward/back, together. That is what makes it a
    rotation. Flipping only left/right is the reflection this file exists to
    prevent, so it is asserted here that we do NOT do that.
    """
    M = yaw_matrix(180)
    assert np.allclose(M @ np.array([1.0, 0, 0]), [-1, 0, 0], atol=1e-12)
    assert np.allclose(M @ np.array([0, 1.0, 0]), [0, -1, 0], atol=1e-12)
    assert np.allclose(M @ np.array([0, 0, 1.0]), [0, 0, 1], atol=1e-12)

    mirror = np.diag([-1.0, 1.0, 1.0])          # "just flip left/right"
    assert abs(np.linalg.det(mirror) + 1.0) < 1e-12
    assert not np.allclose(M, mirror), (
        "if these were ever equal, the yaw would be a mirror")


def test_no_yaw_can_produce_a_reflection():
    """The property that makes the parameter safe to hand to an operator."""
    for deg in np.linspace(-720, 720, 289):
        assert np.linalg.det(yaw_matrix(deg)) > 0.999999


# ------------------------------------------------- position and rotation
def test_position_and_orientation_are_aligned_consistently():
    """Rotate-then-align must equal align-then-rotate, which is what makes
    the commanded position and orientation describe the same physical pose."""
    rng = np.random.default_rng(3)
    for deg in (0, 55, 180, -120):
        qz = yaw_quat(deg)
        M = yaw_matrix(deg)
        for _ in range(20):
            v = rng.normal(size=4)
            dq = q_norm(v / np.linalg.norm(v))
            p = rng.normal(size=3)
            # the mapper's own conjugation
            dq_aligned = q_mul(q_mul(qz, dq), q_conj(qz))
            a = R_of(dq_aligned) @ (M @ p)
            b = M @ (R_of(dq) @ p)
            assert np.allclose(a, b, atol=1e-9)


def test_identity_yaw_is_exactly_the_old_behaviour():
    """0 deg must be a no-op to the last bit, so the default cannot silently
    change what every existing measurement was taken with."""
    assert np.allclose(yaw_matrix(0.0), np.eye(3), atol=0.0)
    assert np.allclose(yaw_quat(0.0), [0.0, 0.0, 0.0, 1.0], atol=0.0)


# ------------------------------------------------------- wired into _tick
def test_the_mapper_actually_applies_it_to_both():
    """A parameter that is stored and never read is this repo's most repeated
    instrument failure. Check the consumer, not the declaration."""
    import inspect
    from srl_vr_teleop import vr_pose_mapper
    src = inspect.getsource(vr_pose_mapper.VrPoseMapper._tick)
    assert "R = yaw_matrix(yaw)" in src
    assert "self.scale * (R @ d)" in src, "position must be aligned"
    assert "q_mul(q_mul(qz, dq), q_conj(qz))" in src, \
        "orientation must be aligned by the SAME yaw, conjugated"
    reb = inspect.getsource(vr_pose_mapper.VrPoseMapper._set_scale)
    assert "R @ (" in reb, "the scale re-base must use the aligned displacement"
