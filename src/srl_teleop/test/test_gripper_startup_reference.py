#!/usr/bin/env python3
"""The startup/shutdown open reference: what may be assumed, and what may not.

The end-to-end proof is `scripts/verify_gripper_shutdown.py`, which actually
SIGKILLs the node mid-grasp against a fake 2F-85 that holds position. These
tests pin the decision table underneath it, which is where the two opposite
mistakes live: opening on a grip (drops the object) and trusting a leftover
position as the open reference (wrong travel range for the whole session).
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
from srl_teleop import gripper_state as gs             # noqa: E402

ARM = "test_left"


@pytest.fixture(autouse=True)
def clean_marker():
    gs.clear_latched(ARM)
    yield
    gs.clear_latched(ARM)


# ------------------------------------------------------------------ bands
def test_the_three_states_are_distinguished():
    assert gs.classify(0.00) == gs.OPEN
    assert gs.classify(0.42) == gs.HOLDING          # a 40 mm block
    assert gs.classify(0.80) == gs.FREE_AIR         # closed on nothing


@pytest.mark.parametrize("bad", [None, "", float("nan"), float("inf")])
def test_absence_is_never_a_position(bad):
    """`unknown` must not collapse into `open`. Absence read as a value is
    the mechanism behind the frozen /real/joint_states and the dead j7 pot."""
    assert gs.classify(bad) == gs.UNKNOWN


# --------------------------------------------------------- decision table
def test_no_marker_and_part_closed_opens():
    action, why = gs.startup_decision(ARM, 0.42)
    assert action == "open"
    assert "previous run" in why


def test_marker_plus_holding_does_NOT_open():
    """The one case where opening would drop a participant's object."""
    gs.set_latched(ARM, 0.52)
    action, why = gs.startup_decision(ARM, 0.42)
    assert action == "hold"
    assert "drop" in why


def test_marker_plus_free_air_is_STALE_and_opens():
    """A marker outlives its object. Requiring BOTH the marker and a holding
    knuckle is what stops a stale file freezing an empty gripper forever."""
    gs.set_latched(ARM, 0.52)
    action, why = gs.startup_decision(ARM, 0.80)
    assert action == "open"
    assert "stale" in why


def test_marker_plus_open_is_STALE_and_opens():
    gs.set_latched(ARM, 0.52)
    action, _ = gs.startup_decision(ARM, 0.0)
    assert action == "open"


def test_no_feedback_commands_NOTHING():
    """Neither open nor hold. An open on no evidence could drop a load; a
    hold on no evidence freezes a working gripper. The honest answer is to
    say the reference is unknown and command nothing."""
    gs.set_latched(ARM, 0.52)
    action, why = gs.startup_decision(ARM, None)
    assert action == "unknown"
    assert "joint_states" in why


def test_an_already_open_hand_is_still_commanded_open():
    """Asserted, not assumed. 'It looks open' is the assumption this pass
    removes -- the command is issued and then confirmed from feedback."""
    action, why = gs.startup_decision(ARM, 0.0)
    assert action == "open"
    assert "asserted, not assumed" in why


# -------------------------------------------------------------- the marker
def test_marker_is_written_at_latch_time_not_at_exit():
    """The whole design rests on this. kill -9 runs no shutdown code, so the
    claim 'I may be holding something' has to already be on disk."""
    assert not gs.was_latched(ARM)
    gs.set_latched(ARM, 0.5, "unit test")
    assert gs.was_latched(ARM)
    assert os.path.exists(gs.marker_path(ARM))
    assert gs.marker_age_s(ARM) is not None


def test_clearing_a_marker_that_is_not_there_is_not_an_error():
    assert gs.clear_latched(ARM) is True
    assert gs.clear_latched(ARM) is True


def test_the_marker_does_not_survive_a_reboot():
    """A grip cannot outlive the machine, and a marker that did would be a
    latch nobody could clear."""
    assert gs.MARKER_DIR.startswith("/run") or gs.MARKER_DIR == "/tmp"
