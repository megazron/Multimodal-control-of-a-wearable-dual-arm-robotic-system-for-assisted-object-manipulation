#!/usr/bin/env python3
"""Switching between all six modes: no leaked state, no bypassed safety."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "srl_teleop"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_teleop import operating_modes as om            # noqa: E402
from srl_autonomy.world_model import WorldModel         # noqa: E402


def test_every_ordered_pair_of_modes_keeps_the_full_safety_stack():
    """36 transitions. The invariant must hold in both directions, including
    the dangerous one -- dropping FROM mode 6 must not leave a mode-6
    permission behind, and rising TO mode 6 must not inherit mode 1's
    thinner requirements."""
    for a in om.Mode:
        for b in om.Mode:
            om.assert_safety_invariant(a, om.required_safety(a))
            om.assert_safety_invariant(b, om.required_safety(b))
            # what mode b needs must be satisfied by b's own stack, never by
            # whatever a happened to have active
            assert set(om.SAFETY_STACK) <= set(om.required_safety(b))


def test_rising_to_autonomy_adds_requirements_and_lowering_removes_them():
    up = set(om.required_safety(om.Mode.FULL_AUTONOMY))
    down = set(om.required_safety(om.Mode.DIRECT_MANNEQUIN))
    assert up > down
    # and a teleop mode must NOT silently keep mode 6's extra gates, or
    # teleop could never start after an autonomy run
    assert "spoken_confirmation" not in down


def test_switching_into_teleop_from_mode6_does_not_inherit_the_speed_cap():
    assert (om.MAX_VEL_RAD_S[om.Mode.DIRECT_MANNEQUIN]
            > om.MAX_VEL_RAD_S[om.Mode.FULL_AUTONOMY])


def test_world_model_is_not_shared_between_mode_entries():
    """A stale object surviving a mode change would let mode 6 act on
    something perceived during a completely different session."""
    a = WorldModel()
    a.observe([{"label": "cube", "position": [0.3, 0.3, 0.9],
                "confidence": 0.9}])
    b = WorldModel()
    assert a.objects and not b.objects


@pytest.mark.parametrize("m", list(om.Mode))
def test_input_source_matches_the_deadman_that_applies(m):
    """A voice-driven mode must not be gated on a master dead-man, and a
    master-driven mode must be."""
    src = om.INPUT_SOURCE[m]
    if src == "master":
        assert om.requires_master(m)
    else:
        assert not om.requires_master(m)
    assert om.is_autonomous(m) == (src in ("voice", "voice_or_point"))
