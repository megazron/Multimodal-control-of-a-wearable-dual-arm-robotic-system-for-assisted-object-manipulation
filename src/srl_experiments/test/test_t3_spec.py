#!/usr/bin/env python3
"""T3's spec properties, asserted rather than trusted."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "abc"))

import task3 as T3                                        # noqa: E402
from srl_experiments.task_actions import Verb             # noqa: E402


def test_the_box_is_where_a_PINNED_wrist_can_hold_it():
    """|x| >= 0.51 is the measured boundary of the right arm's pinned-wrist
    graspable strip. A box inboard of it cannot be HELD in MASTER_TELEOP,
    which is the mode the whole layout is constrained by."""
    assert abs(T3.BOX_OBJ[0]) >= 0.51, T3.BOX_OBJ
    assert T3.BOX_ARM == "right"


def test_the_meter_is_in_the_LEFT_arms_measured_strip():
    assert 0.30 <= abs(T3.METER_OBJ[0]) <= 0.39, T3.METER_OBJ
    assert 0.15 <= T3.METER_OBJ[1] <= 0.23, T3.METER_OBJ
    assert T3.METER_ARM == "left"


def test_the_two_objects_use_DIFFERENT_arms():
    """They are two objects at two places; if one arm had to do both, the
    task would be sequential and the coordination measure would vanish."""
    assert T3.BOX_ARM != T3.METER_ARM


def test_measurement_points_FORCE_repositioning():
    """A trial with zero requests must be evidence of protocol violation, not
    of good coordination -- so the geometry has to make at least one request
    unavoidable."""
    assert T3.MIN_EXPECTED_REQUESTS >= 2, T3.MIN_EXPECTED_REQUESTS
    unserved = [p for p in T3.MEASUREMENT_POINTS
                if not p["served_by_initial_presentation"]]
    assert len(unserved) == T3.MIN_EXPECTED_REQUESTS
    for p in unserved:
        assert p.get("needs"), "%s must say WHY it forces a request" % p["id"]
    assert any(p["face"] == "far" for p in T3.MEASUREMENT_POINTS)


def test_the_multimeter_is_the_respecd_size():
    """30 x 70 x 45. The original bound three ways at once and the respec is
    what made the pose fit, the capture window and sigma all pass."""
    assert T3.METER_SIZE == (0.030, 0.070, 0.045)


def test_commands_are_mode_free_and_use_RETURN_for_the_return_leg():
    cmds = T3.commands()
    assert len(cmds) == 6
    verbs = [c.verb for c in cmds]
    assert verbs.count(Verb.PICK_OBJECT) == 2
    assert verbs.count(Verb.PLACE_OBJECT) == 2
    assert verbs.count(Verb.RETURN) == 2, (
        "the return leg is scored separately and its destination is not the "
        "participant's choice, so it must be RETURN and not PLACE_OBJECT")
    for c in cmds:
        assert "MODE" not in str(c.params)
        assert c.arm in ("left", "right")


def test_every_declared_pose_is_offered_to_the_verifier():
    """A pose the spec declares but the verifier never sees is a pose nobody
    has checked."""
    declared = {tuple(T3.BOX_OBJ), tuple(T3.BOX_PRESENT),
                tuple(T3.METER_OBJ), tuple(T3.METER_PRESENT)}
    offered = {tuple(p) for _, p in T3.poses_to_verify()}
    assert declared == offered, declared ^ offered


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
