#!/usr/bin/env python3
"""KNOWN ANSWERS FOR HOW LONG THE STAGING MOVE IS GIVEN.

Every clip is supposed to open on the presentation pose. It is commanded as a
single-point joint trajectory, and the duration on that point was a FIXED
2.5 s whatever the distance -- so a move of 2.23 rad asked for 0.89 rad/s
sustained and did not arrive. Measured on the 2026-08-15 sweep, printed by
the staging script itself:

    left  presentation pose, worst joint error 1.0315 rad
    right presentation pose, worst joint error 2.2282 rad
    DID NOT ARRIVE within 8.0 s

A retry did not rescue it, because the retry re-commanded the same impossible
2.5 s. Three clips opened on the HOME pose because of that -- and the home
wrist points up by +85/+79 degrees, which is real, correct, and reads as a
fault to anyone who has not been told.

A fixed duration is only right for a fixed distance, and this distance is not
fixed: the arm starts wherever the previous task left it.
"""

import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))


def _ms():
    try:
        import stage_presentation_pose
    except ImportError as e:                                # pragma: no cover
        pytest.skip("stage_presentation_pose needs rclpy (%s)" % e)
    return stage_presentation_pose.move_seconds


# ---- the two measurements that produced the bug -------------------------
def test_the_RIGHT_ARMS_REAL_FAILURE_now_gets_enough_time():
    """2.2282 rad is the move that failed. It must no longer be given 2.5 s."""
    got = _ms()(2.2282)
    assert got > 4.0, got
    # and the rate it implies must stay slow: this is a joint-space move near
    # a person with the followers paused.
    assert 2.2282 / got <= 0.5


def test_the_LEFT_ARMS_REAL_FAILURE_too():
    assert _ms()(1.0315) > 2.5


# ---- the floor, which must not be lost ----------------------------------
def test_a_SHORT_move_still_gets_the_floor_not_a_tiny_number():
    """0.1 rad at 0.45 rad/s is 0.22 s. Commanding that would be a jerk.

    The floor is what stops the fix turning every small correction into a
    snap.
    """
    assert _ms()(0.1) == 2.5


def test_an_arm_ALREADY_THERE_gets_the_floor_and_not_zero():
    assert _ms()(0.0) == 2.5


# ---- absent is not zero -------------------------------------------------
def test_NO_JOINT_STATE_falls_back_to_the_floor():
    """None means the joint state has not arrived, not that the arm is here.

    Guessing a long move from no information would make every clip wait; the
    floor is the honest default and the arrival check still has to pass.
    """
    assert _ms()(None) == 2.5


# ---- and it cannot run away ---------------------------------------------
def test_an_ABSURD_distance_is_capped_rather_than_stalling_the_sweep():
    """A bad joint reading must not park the sweep for ten minutes."""
    assert _ms()(1000.0) == 30.0


def test_it_is_monotonic_in_the_distance():
    f = _ms()
    seq = [f(x) for x in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0)]
    assert seq == sorted(seq)
    # and it genuinely varies -- a rule that returns one number for every
    # distance is the bug it replaces.
    assert len(set(seq)) > 1
