#!/usr/bin/env python3
"""A gripper closed on NOTHING must never read as holding something.

`mock_components/GenericSystem` boots the knuckle at 0.7929 rad -- fully shut.
Every threshold in the set is below that, so a test written as "closed enough
for this object" passes at startup for every object, with the arm still at
home. That is how the 2026-08-10 re-record logged task A's grasp at t=0.0 and
then reported the REAL grasp, 56 s later, as being 51.3 s outside the clip.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from srl_teleop import gripper_state as g                    # noqa: E402

WIDTHS = (20, 30, 40, 45, 50)
BOOT = 0.7929               # what the mock actually boots at, measured


def test_the_mock_boot_value_is_not_a_grasp_of_anything():
    assert BOOT >= g.FREE_AIR_RAD, "the premise moved; re-measure the mock"
    for w in WIDTHS:
        assert not g.holding(BOOT, w), (
            "%d mm reads as held by a gripper closed on nothing" % w)


def test_the_mechanical_limit_is_not_a_grasp_either():
    for w in WIDTHS:
        assert not g.holding(g.MECH_LIMIT_RAD, w)


def test_a_real_grip_at_the_object_width_still_holds():
    """The upper bound must not break the case the lower bound exists for."""
    for w in WIDTHS:
        k = g.grip_for(w)
        assert g.holding(k, w), "%d mm at its own grip angle is not held" % w
        assert k < g.FREE_AIR_RAD, (
            "%d mm needs %.4f, at or past free air %.4f -- the bound would "
            "make this object ungraspable" % (w, k, g.FREE_AIR_RAD))


def test_an_open_gripper_is_not_holding():
    for w in WIDTHS:
        assert not g.holding(0.0, w)
        assert not g.holding(g.OPEN_RAD, w)


def test_the_two_branches_agree_about_free_air():
    """The width and no-width branches must not disagree at the top end. The
    no-width branch always excluded free air; the width branch did not, and a
    single owner of the constants that contradicts itself is worse than two."""
    assert not g.holding(BOOT)
    assert not g.holding(BOOT, 40)
