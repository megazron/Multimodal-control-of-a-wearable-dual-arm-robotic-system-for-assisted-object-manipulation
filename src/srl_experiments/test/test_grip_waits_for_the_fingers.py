"""The run must wait for the FINGERS, by the same test its evidence uses.

WHY THIS FILE EXISTS. 06_full_autonomy/T1 was recorded with two of its four
cubes left on the table, and the run reported exit 0. From clip_scene's own
closest-approach numbers:

    cube_0   pads reached 11.0 mm of it -- knuckle 0.0007, hand OPEN --
             and the fingers did not read closed until 48.8 mm away
    cube_1   picked, knuckle 0.3968 against a needed 0.3812

The grip COMMAND was already gated on arrival. What was not gated was the
schedule: it walked on after a fixed number of ticks while the fingers were
still moving. The same run launched by hand, with nothing recording, picked
all four -- so the dwell was tuned without the recording load and the result
depended on what else was running.

These are known answers: the numbers below are the ones measured in that clip.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ABC = os.path.join(HERE, "..", "experiments", "abc")
sys.path.insert(0, os.path.abspath(ABC))

run_abc = pytest.importorskip("run_abc")
from srl_teleop.gripper_state import grip_for, holding   # noqa: E402

WIDTH = 40
WANT = grip_for(WIDTH)


def test_the_open_hand_is_not_settled():
    """cube_0's actual knuckle at its closest approach. This is the case
    that cost the clip: 0.0007 rad is a hand that has not moved."""
    assert run_abc.grip_settled(0.0007, WANT, WIDTH) is False


def test_the_closed_hand_is_settled():
    """cube_1's actual knuckle when the scene recorded GRASPED."""
    assert run_abc.grip_settled(0.3968, WANT, WIDTH) is True


def test_it_agrees_with_the_scene_at_every_knuckle():
    """THE POINT OF THE CHANGE. The run and clip_scene must not disagree
    about whether the hand is closed -- that disagreement is the defect.

    Swept rather than spot-checked, because a threshold that agrees at two
    points and differs at a third is exactly the drift this guards against.
    """
    for i in range(0, 801):
        kn = i / 1000.0
        assert run_abc.grip_settled(kn, WANT, WIDTH) is bool(
            holding(kn, WIDTH)), "disagree at knuckle %.3f" % kn


def test_a_release_settles_only_when_the_hand_is_actually_open():
    """An OPEN is owed too. Releasing over the pad and moving off before the
    fingers part drops the cube somewhere nobody chose."""
    assert run_abc.grip_settled(WANT, 0.0, WIDTH) is False
    assert run_abc.grip_settled(0.0, 0.0, WIDTH) is True


def test_no_readback_does_not_stall_the_run():
    """A missing knuckle must not hang a run. It is reported elsewhere; it
    is not a reason to sit on a waypoint until the deadline."""
    assert run_abc.grip_settled(None, WANT, WIDTH) is True


def test_the_check_can_fail():
    """A check that cannot fail on a deliberately broken input is not a
    check. `holding` accepts a band; a knuckle far past the object is a
    hand closed on AIR and must not read as settled."""
    assert run_abc.grip_settled(0.79, WANT, WIDTH) is False
