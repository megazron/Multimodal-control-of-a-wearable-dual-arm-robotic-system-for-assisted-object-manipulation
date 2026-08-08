#!/usr/bin/env python3
"""The six modes, the shared safety stack, and mode 6's extra requirements."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "srl_teleop"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_teleop import operating_modes as om                # noqa: E402
from srl_autonomy.autonomy_executive import (               # noqa: E402
    behind_wearer, inside_wearer)

FULL = set(om.SAFETY_STACK)


def test_all_six_modes_exist_and_describe():
    assert [m.value for m in om.Mode] == [1, 2, 3, 4, 5, 6]
    for m in om.Mode:
        assert om.describe(m)


def test_mode_lookup_is_strict():
    assert om.mode_from("full_autonomy") is om.Mode.FULL_AUTONOMY
    assert om.mode_from(6) is om.Mode.FULL_AUTONOMY
    assert om.mode_from("6") is om.Mode.FULL_AUTONOMY
    with pytest.raises(ValueError):
        om.mode_from("fast")            # a typo must not silently default
    with pytest.raises(ValueError):
        om.mode_from(True)


def test_every_mode_requires_the_whole_shared_stack():
    """No mode may bypass it -- that is the invariant the design rests on."""
    for m in om.Mode:
        assert FULL <= set(om.required_safety(m)), m


def test_dropping_any_one_mechanism_is_refused_in_every_mode():
    for m in om.Mode:
        for drop in om.SAFETY_STACK:
            active = set(om.required_safety(m)) - {drop}
            with pytest.raises(RuntimeError) as ei:
                om.assert_safety_invariant(m, active)
            assert drop in str(ei.value)


def test_full_stack_is_accepted():
    for m in om.Mode:
        assert om.assert_safety_invariant(m, om.required_safety(m))


def test_autonomous_modes_demand_more_than_teleop():
    extra = set(om.required_safety(om.Mode.FULL_AUTONOMY)) - FULL
    assert {"action_timeout", "confidence_floor", "voice_stop",
            "decision_log", "spoken_confirmation",
            "world_model_agreement"} <= extra
    # mode 1 must NOT demand them, or teleop could never start
    assert set(om.required_safety(om.Mode.DIRECT_MANNEQUIN)) == FULL


def test_autonomy_is_slower_than_teleop():
    """No operator is watching, so the only bound on a wrong motion is how
    long it takes to happen."""
    assert (om.MAX_VEL_RAD_S[om.Mode.FULL_AUTONOMY]
            < om.MAX_VEL_RAD_S[om.Mode.SUPERVISED_AUTO]
            < om.MAX_VEL_RAD_S[om.Mode.DIRECT_MANNEQUIN])


def test_teleop_modes_decide_nothing():
    assert om.ROBOT_DECIDES[om.Mode.DIRECT_MANNEQUIN] == frozenset()
    assert om.ROBOT_DECIDES[om.Mode.DIRECT_VR] == frozenset()


def test_only_modes_5_and_6_may_decide_the_grasp():
    for m in om.Mode:
        if m in (om.Mode.SUPERVISED_AUTO, om.Mode.FULL_AUTONOMY):
            assert "grasp" in om.ROBOT_DECIDES[m]
        else:
            assert "grasp" not in om.ROBOT_DECIDES[m]


def test_only_mode_6_may_choose_the_target_without_a_human():
    assert "target" in om.ROBOT_DECIDES[om.Mode.FULL_AUTONOMY]
    assert om.INPUT_SOURCE[om.Mode.FULL_AUTONOMY] == "voice"


# ---------------------------------------------------------- wearer keep-out
def test_behind_the_wearer_is_refused_not_routed_around():
    """+y is FORWARD. Anything at y < 0 is behind the person, and the arms
    mount on the back, so 'behind' is where the wearer IS."""
    assert behind_wearer([0.0, -0.30, 1.20]) is True
    assert behind_wearer([0.0, +0.30, 1.20]) is False


def test_head_and_torso_are_keep_out_volumes():
    bad, part = inside_wearer([0.0, 0.0, 1.295])
    assert bad and part == "head"
    bad, part = inside_wearer([0.0, -0.06, 1.05])
    assert bad and part == "torso"
    bad, _ = inside_wearer([0.60, 0.40, 0.95])
    assert not bad


def test_padding_widens_the_keep_out():
    p = [0.0, 0.0, 1.295 + 0.17]
    assert inside_wearer(p, pad=0.0)[0] is False
    assert inside_wearer(p, pad=0.05)[0] is True
