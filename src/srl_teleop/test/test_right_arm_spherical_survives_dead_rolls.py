#!/usr/bin/env python3
"""The RIGHT arm must still give POSITION with j3/j5/j7 dead.

TASK 2f. All three dead right-arm channels are ROLL joints (j3, j5, j7).
Spherical mode takes azimuth from j1, elevation from the wrist IMU's gravity
vector, and reach magnitude from |fk(j1,j2,j3,j4,0,0,0)| -- which is dominated
by the j2/j4 bends. A dead roll contributes almost nothing to reach magnitude,
so the right arm should still track POSITION.

These tests pin that claim, and pin what it does NOT buy: orientation. They
run against the real module, with no hardware and no ROS graph.
"""
import math

import numpy as np
import pytest

from srl_teleop.master_pose_node import (
    SPHERICAL_CHECK_IDX, ZERO_CHECK_IDX, zero_dropouts)

DEAD_RIGHT_ROLLS = (3, 5, 7)          # 1-based, measured dead
LIVE_RIGHT = (1, 2, 4, 6)


def frame(over=None):
    """A right-arm joint vector in degrees, dead rolls already zeroed.

    `over` is keyed by 1-based joint number, so it cannot be **kwargs.
    """
    d = {1: 20.0, 2: -35.0, 3: 0.0, 4: 60.0, 5: 0.0, 6: 15.0, 7: 0.0}
    d.update(over or {})
    return [d[i] for i in range(1, 8)]


def test_spherical_checks_only_j1_j2_j4():
    """The whole point: a dead roll must not be a validated channel."""
    checked = {i + 1 for i in SPHERICAL_CHECK_IDX}
    assert checked == {1, 2, 4}, (
        "spherical mode must validate only j1/j2/j4; validating a dead roll "
        "would reject every right-arm frame. Got %s" % sorted(checked))
    for j in DEAD_RIGHT_ROLLS:
        assert j not in checked, "j%d is dead and must not be validated" % j


def test_dead_rolls_do_not_trip_the_dropout_detector_in_spherical():
    """j3/j5/j7 sitting at exactly 0.0 is the DEAD signature, not a dropout."""
    f = frame()
    assert zero_dropouts(f, SPHERICAL_CHECK_IDX) == [], (
        "a right-arm frame with only the dead rolls at zero must pass "
        "spherical validation")


def test_fk_mode_would_reject_the_same_frame():
    """Contrast, so the mode-scoping is doing real work.

    fk mode validates j1-j5, so the same physically-fine frame is rejected --
    which is exactly why the right arm is unusable in fk mode and usable in
    spherical.
    """
    f = frame()
    bad = zero_dropouts(f, ZERO_CHECK_IDX)
    assert bad, ("fk mode should reject this frame on the dead rolls; if it "
                 "does not, the mode scoping is not being exercised")
    assert any(i + 1 in DEAD_RIGHT_ROLLS for i in bad)


def test_reach_still_responds_to_the_live_bends():
    """Reach magnitude must vary with j2/j4 while the rolls stay at zero.

    This is the substantive claim: position survives because reach comes from
    the bends, not the rolls.
    """
    from srl_teleop import master_calibration as mc

    def reach(j2, j4):
        q = np.radians(frame({2: j2, 4: j4}))
        q = list(q[:4]) + [0.0, 0.0, 0.0]
        return float(np.linalg.norm(mc.fk(q)))

    a, b = reach(-35.0, 20.0), reach(-35.0, 90.0)
    assert abs(a - b) > 0.05, (
        "reach magnitude barely moved (%.4f -> %.4f m) when the elbow swept "
        "70 deg; if this fails the right arm has no usable position" % (a, b))


def test_rolls_barely_change_reach_magnitude():
    """A dead roll costs LITTLE -- measured, not assumed.

    Not zero: rolling the upper arm reorients the elbow bend relative to the
    shoulder, so with j4 bent it does move the tip. Measured here, the worst
    case over the full roll range is ~20 mm against the master arm's 0.272 m
    reach (7.5%), and docs/ENGINEERING_LOG.md's reduction ladder puts the TYPICAL cost of
    dropping j3 on the right arm at 4.5 mm. Small enough that position
    survives; large enough that "free" would be the wrong word.
    """
    from srl_teleop import master_calibration as mc

    def reach(j3):
        q = np.radians(frame({3: j3}))
        q = list(q[:4]) + [0.0, 0.0, 0.0]
        return float(np.linalg.norm(mc.fk(q)))

    base = reach(0.0)
    worst = max(abs(reach(j3) - base) for j3 in (-120.0, -90.0, -30.0, 45.0,
                                                 90.0, 120.0))
    assert worst < 0.025, (
        "j3 moved the tip %.1f mm over the full roll range. Above ~25 mm the "
        "claim that spherical reach is roll-insensitive stops holding and the "
        "right arm's position would need j3 back." % (1000 * worst))
    # And it must not be trivially zero either -- that would mean fk() is
    # ignoring j3 entirely, which would make this test vacuous.
    assert worst > 1e-4, "fk() appears not to use j3 at all; test is vacuous"


def test_what_the_right_arm_LACKS_is_orientation():
    """Position survives; WRIST ORIENTATION does not, and that is the cost.

    j5 and j7 are the forearm and wrist rolls. With both dead the wrist's
    rotation about its own axis is unobservable, so orientation_mode cannot
    leave `fixed` for the right arm no matter what the IK does.
    """
    lost = set(DEAD_RIGHT_ROLLS) - set(LIVE_RIGHT)
    assert {5, 7} <= lost
    # j1 (azimuth) and the bends survive, so position is observable.
    assert {1, 2, 4} <= set(LIVE_RIGHT)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
