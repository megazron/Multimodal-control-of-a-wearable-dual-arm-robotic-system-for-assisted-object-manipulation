#!/usr/bin/env python3
"""The grasp thresholds must have exactly ONE definition.

They had six, across the files that decide whether a grasp happened:
`gripper_state.py`, `record_rviz.py`, `clip_tasks.py`, `fsr_gripper_node.py`,
`vr_gripper_node.py` and `verify_gripper_shutdown.py`. Two had already
drifted -- `verify_gripper_shutdown` called 0.80 "FREE_AIR_RAD" while
`gripper_state.FREE_AIR_RAD` is 0.74, the same name for two different
quantities six hundredths apart, and `vr_gripper_node` called 0.0 "OPEN_RAD"
against a classification bound of 0.10.

A drifted grasp threshold does not crash. It silently reclassifies trials: a
grip that one module scores as `holding` another scores as `free_air`, so the
"object retained" column and the attachment in the clip disagree about the
same instant, and nothing says so.

THESE TESTS COMPARE IDENTITY, NOT VALUE. Asserting the numbers are equal
passes the moment after someone re-copies them, which is the state this is
meant to prevent -- they were equal when they were copied, too.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))

from srl_teleop import gripper_state as gs                   # noqa: E402


def test_the_ordering_is_asserted_at_import():
    """command-to-open < open bound < free-air bound <= mechanical stop.

    The gap between FREE_AIR_RAD and MECH_LIMIT_RAD is the whole margin that
    makes "closed on nothing" distinguishable from "closed on something"; if
    they ever meet, every empty hand reads as a grip.
    """
    assert gs.CMD_OPEN_RAD < gs.OPEN_RAD < gs.FREE_AIR_RAD <= gs.MECH_LIMIT_RAD


def test_clip_tasks_imports_grip_for_rather_than_restating_it():
    import clip_tasks as CT
    assert CT.grip_for is gs.grip_for
    assert CT.OPEN is gs.CMD_OPEN_RAD


def test_record_rviz_imports_grip_for_and_holding():
    import record_rviz as rr
    assert rr.grip_for is gs.grip_for
    assert rr.holding is gs.holding
    assert (rr.GRIP_HOLD_MIN, rr.GRIP_FREE_AIR) == (gs.OPEN_RAD,
                                                    gs.FREE_AIR_RAD)


def test_holding_needs_the_fingers_to_reach_the_object():
    """The band alone is not a grasp.

    A 40 mm block needs 0.4235 rad. Merely leaving the open position -- 0.11,
    inside the holding band -- attached the object while the fingers were
    still visibly open, so the clip showed the block jump to the hand before
    it was touched.
    """
    need = gs.grip_for(40)
    assert gs.holding(0.11) is True             # band alone: yes
    assert gs.holding(0.11, 40) is False        # against the object: no
    assert gs.holding(0.90 * need, 40) is True
    assert gs.holding(0.89 * need, 40) is False


def test_holding_is_false_for_nothing_and_for_NaN():
    """A missing reading must never read as a grip."""
    assert gs.holding(None) is False
    assert gs.holding(float("nan")) is False
    assert gs.holding(None, 40) is False


def test_a_drifted_copy_would_FAIL_this(monkeypatch):
    """The negative control: these tests must be able to fail.

    Rebinding one module's grip_for to an equal-valued COPY -- exactly what a
    re-paste produces -- has to be caught, or the identity checks above are
    decoration.
    """
    import clip_tasks as CT

    def copy_of_grip_for(width_mm):
        return max(0.12, min(0.70, 0.8 * (1.0 - float(width_mm) / 85.0)))

    assert copy_of_grip_for(40) == gs.grip_for(40)      # equal in value
    monkeypatch.setattr(CT, "grip_for", copy_of_grip_for)
    try:
        test_clip_tasks_imports_grip_for_rather_than_restating_it()
    except AssertionError:
        return
    raise AssertionError("an equal-valued copy was not detected")
