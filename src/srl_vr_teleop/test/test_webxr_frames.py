#!/usr/bin/env python3
"""WebXR -> world frame conversion, pinned against known rotations.

WHY THIS FILE EXISTS. `test_quest_frames.py` pins the UNITY conversion in
`quest_vendor_bridge` (the USB/adb route) and nothing pinned the WebXR one in
`quest_bridge_node` (the wifi route). They are not the same conversion and
must not be:

    Unity   is LEFT-handed:  +x right, +y up, +z FORWARD
    WebXR   is RIGHT-handed: +x right, +y up, +z BACKWARD (toward the viewer)

The WebXR bridge shipped with `[-z, -x, y]`, which lands in the ROS
x-forward / y-left convention -- NOT this repo's world frame (x = the
wearer's right, y = forward, z = up; vr_bringup.md section 6). That matters
because `vr_pose_mapper` adds the controller displacement STRAIGHT onto the
robot's world-frame pose:

    p_cmd = p_anchor + scale * (p_controller - p_ref)

with no aligning rotation anywhere. So a controller frame that is not the
world frame is a rotation error applied to every command. With the shipped
mapping, pushing the controller FORWARD moved the arm to the wearer's RIGHT
-- 90 degrees off, on the exact path this test now covers.

The first two tests below fail against the shipped mapping. That is the
point: a check that cannot fail on the broken input is not a check.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_vr_teleop.quest_bridge_node import (                  # noqa: E402
    webxr_to_world_position, webxr_to_world_quat)


def R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------- the 3 axes
def test_webxr_up_maps_to_world_z():
    """WebXR +y is UP; this repo's world +z is UP."""
    assert np.allclose(webxr_to_world_position([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])


def test_webxr_forward_maps_to_world_y():
    """WebXR FORWARD is -z (+z points at the viewer). World +y is forward.

    This is the one the shipped mapping got wrong: it returned
    [1, 0, 0] -- the wearer's RIGHT -- for a hand pushed straight ahead.
    """
    assert np.allclose(webxr_to_world_position([0.0, 0.0, -1.0]), [0.0, 1.0, 0.0])


def test_webxr_right_is_world_x_unchanged():
    assert np.allclose(webxr_to_world_position([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0])


# ------------------------------------------------------------- the rotation
def test_identity_rotation_stays_identity():
    assert np.allclose(R(webxr_to_world_quat([0.0, 0.0, 0.0, 1.0])),
                       np.eye(3), atol=1e-9)


def test_conversion_is_a_proper_rotation():
    """det(R) = +1 for every input. WebXR is already right-handed, so unlike
    the Unity path there is NO handedness flip to carry and w must not be
    negated. A reflection here mirrors the operator's wrist."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        v = rng.normal(size=4)
        v /= np.linalg.norm(v)
        q = webxr_to_world_quat(v)
        assert abs(np.linalg.det(R(q)) - 1.0) < 1e-9


def test_w_is_not_negated():
    """Explicit, because the Unity conversion next door DOES negate w and
    copying it here would invert every rotation."""
    q = webxr_to_world_quat([0.1, 0.2, 0.3, 0.9])
    assert q[3] == 0.9


def test_yaw_about_webxr_up_becomes_the_same_yaw_about_world_z():
    """A turn about the vertical stays a turn about the vertical, same
    magnitude AND SAME SIGN -- both frames are right-handed here, so unlike
    the Unity path there is no negation."""
    for ang in (0.3, 1.0, -0.7, 2.5):
        h = ang / 2.0
        q = webxr_to_world_quat([0.0, math.sin(h), 0.0, math.cos(h)])
        M = R(q)
        assert np.allclose(M @ np.array([0, 0, 1.0]), [0, 0, 1.0], atol=1e-9)
        got = math.atan2(M[1, 0], M[0, 0])
        err = math.atan2(math.sin(got - ang), math.cos(got - ang))
        assert abs(err) < 1e-6, (ang, got)


def test_position_and_rotation_agree():
    """Rotate in WebXR then convert == convert then rotate. If these disagree
    the position and the orientation are in different frames and IK is asked
    for a pose that does not exist."""
    for ang in (0.4, -1.1):
        h = ang / 2.0
        qx = [0.0, math.sin(h), 0.0, math.cos(h)]      # yaw about WebXR +y
        pu = [0.7, 0.2, 0.5]
        c, s = math.cos(ang), math.sin(ang)
        # right-handed rotation about +y acting on (x, y, z)
        rot = [c * pu[0] + s * pu[2], pu[1], -s * pu[0] + c * pu[2]]
        a = webxr_to_world_position(rot)
        b = R(webxr_to_world_quat(qx)) @ webxr_to_world_position(pu)
        assert np.allclose(a, b, atol=1e-9), (a, b)


# ------------------------------------------- the two bridges must NOT agree
def test_the_two_transports_have_different_conversions_on_purpose():
    """Unity is left-handed and WebXR is right-handed, so the same numbers
    off the wire mean different physical directions. If someone ever
    'unifies' these, this fails and says why."""
    from srl_vr_teleop.quest_vendor_bridge import quest_to_world_position
    unity_fwd = quest_to_world_position({"x": 0.0, "y": 0.0, "z": 1.0})
    webxr_same_numbers = webxr_to_world_position([0.0, 0.0, 1.0])
    assert np.allclose(unity_fwd, [0.0, 1.0, 0.0])
    assert np.allclose(webxr_same_numbers, [0.0, -1.0, 0.0])


def test_the_bridge_owns_the_AXES_and_the_mapper_owns_the_HEADING():
    """The frame question this file used to defer, now answered.

    An R_align WAS introduced (2026-08-19), because the operator sits across
    the room facing the wearer rather than standing behind them. That splits
    the job in two, and the split is deliberate:

      quest_bridge_node  WebXR axes -> world AXES.   A fixed relabelling.
                         Known-answer tested above. Never calibrated.
      vr_pose_mapper     the operator's HEADING within those axes, as one
                         yaw angle, measured per setup by
                         scripts/calibrate_operator_yaw.py.

    Keeping them separate is what stops a calibration from editing a constant
    that known-answer tests pin. If the bridge absorbed the yaw, every
    re-calibration would move a number this file asserts, and the two would
    drift apart with nothing to catch it.

    So: the bridge's conversion must stay heading-free, and the mapper must
    apply the heading to the displacement rather than adding it raw.
    """
    import inspect
    from srl_vr_teleop import vr_pose_mapper

    src = inspect.getsource(vr_pose_mapper.VrPoseMapper._tick)
    assert 'self.scale * (R @ d)' in src, (
        'the mapper must align the controller displacement by the operator '
        'yaw before adding it to the anchor')

    # The bridge's conversion carries NO heading: it is pure axis relabelling,
    # so it must be exactly the constants asserted at the top of this file and
    # must not consult any parameter.
    bsrc = inspect.getsource(webxr_to_world_position)
    assert 'yaw' not in bsrc.lower() and 'align' not in bsrc.lower(), (
        'the axis conversion has acquired a heading term; it belongs in '
        'vr_pose_mapper.align_yaw_deg, where it can be calibrated')
    assert np.allclose(webxr_to_world_position([0.0, 0.0, -1.0]),
                       [0.0, 1.0, 0.0])
