#!/usr/bin/env python3
"""The capture gate is the button on the arm you are MOVING, and the pin is
what makes that safe.

This replaces test_capture_gate_is_opposite_button.py, which asserted the
exact opposite. The old invariant was not wrong, it was a WORKAROUND: an
arm's own button toggles that arm's clutch, so gating a left sweep on the
left button disengaged the clutch exactly when the sweep started, and in
capture_20260806_130205 41 of 42 directional segments recorded with the
clutch out and the sim arm still. Gating on the OTHER arm's button avoided
that -- at the price of an operator holding the left master arm reaching
across to press with their right hand, 28 times.

The fix is to stop the button touching the clutch at all.
`force_clutch_engaged` pins the clutch ENGAGED and ignores the buttons for
clutch purposes, while their VALUES still publish on /master_fsr_buttons for
the gate to read. So the invariant that replaces "never the own button" is:

    own-arm gating IMPLIES the pin, and a pin that cannot be CONFIRMED
    refuses the recording.

The confirmation matters more than it looks. `force_clutch_engaged` was read
once in __init__ until 2026-08-30, so setting it reported success and changed
nothing -- a pin "confirmed" by reading the parameter back would have been
the parameter agreeing with itself while the capture recorded another still
arm. See test_master_smoothing.py for the behavioural half.
"""
import inspect
import sys
from pathlib import Path

import pytest

CAP = Path(__file__).resolve().parents[1] / "trajectory_capture"
sys.path.insert(0, str(CAP))
import record_trajectories as rt  # noqa: E402

SRC = (CAP / "record_trajectories.py").read_text()

# [fsr1, fsr2, btn1, btn2]
BTN1, BTN2 = 2, 3
OWN_CLUTCH_BUTTON = {"left": BTN2, "right": BTN1}   # MEASURED 2026-07-31


def test_the_default_gate_is_the_arm_you_are_moving():
    for arm, own in OWN_CLUTCH_BUTTON.items():
        assert rt.GATE_INDEX[arm] == own, (
            "%s segments are not gated by the %s arm's own button; the "
            "operator has to reach across for every segment" % (arm, arm))


def test_the_old_mapping_is_still_reachable():
    """--gate opposite is the fallback the refusal tells the operator to use,
    so it has to exist and has to be the other arm's button."""
    for arm, own in OWN_CLUTCH_BUTTON.items():
        assert rt.GATE_INDEX_OPPOSITE[arm] != own
    assert rt.GATE_INDEX_OPPOSITE["left"] == BTN1
    assert rt.GATE_INDEX_OPPOSITE["right"] == BTN2


def test_own_arm_gating_refuses_to_record_without_a_confirmed_pin():
    body = SRC[SRC.index("def main("):]
    i = body.index('a.gate == "own"')
    block = body[i:body.index("meta = dict", i)]
    assert "pin_clutch(True" in block, "own-arm gating does not pin the clutch"
    assert "return 1" in block, (
        "a clutch pin that cannot be confirmed does not refuse to record -- "
        "which is the 2026-08-06 defect with an extra step")


def test_the_pin_is_confirmed_behaviourally_and_not_by_readback_alone():
    fn = inspect.getsource(rt.pin_clutch)
    assert "node.status" in fn, (
        "pin_clutch never observes the clutch state; reading the parameter "
        "back is the parameter agreeing with itself")
    assert "DISENGAGED" in fn, "no failure message for a clutch that stayed out"


def test_the_pin_is_released_on_every_exit_path():
    body = SRC[SRC.index("def main("):]
    fin = body[body.index("finally:"):]
    assert "pin_clutch(False" in fin, (
        "the pin is not released in finally, so Ctrl-C leaves the next "
        "operator's buttons dead with nothing saying why")


def test_a_dropped_clutch_is_caught_from_the_rows_written():
    rows = [{"left_clutch": "1"}] * 8 + [{"left_clutch": "0"}] * 3
    bad, why = rt.clutch_was_out(rows, "left")
    assert bad and "3 of 11" in why
    assert rt.clutch_was_out([{"left_clutch": "1"}] * 11, "left")[0] is False
    # a column that was never recorded is an ABSENCE, not a clean segment
    assert rt.clutch_was_out([{}] * 11, "left")[0] is False


def test_the_prompt_names_the_button_the_gate_waits_on():
    class _N:
        gate_index = dict(rt.GATE_INDEX)
    assert rt.gate_word(_N, "left") == "LEFT"
    assert rt.gate_word(_N, "right") == "RIGHT"
    _N.gate_index = dict(rt.GATE_INDEX_OPPOSITE)
    assert rt.gate_word(_N, "left") == "RIGHT"
    assert rt.gate_word(_N, "right") == "LEFT"


def test_the_metadata_sentence_matches_the_mapping_it_ships_with():
    """The derived string and the English beside it must agree.

    They did not: the mapping was flipped to own-arm and the sentence still
    read "each arm is gated by the OPPOSITE button", one line below a comment
    explaining that this exact inversion is what made the 2026-08-06 capture
    unusable to anyone reading its metadata.
    """
    body = SRC[SRC.index("gate_mapping="):]
    sent = body[:body.index("pre_labelling")]
    assert 'a.gate == "own"' in sent, \
        "the metadata sentence is fixed text and does not follow --gate"
    assert "ITS OWN button" in sent and "PINNED" in sent, \
        "the own-arm branch does not say the pin is what makes it safe"
    assert "OPPOSITE arm's" in sent, "the --gate opposite branch is missing"


def test_a_refused_run_leaves_nothing_on_disk():
    """An aborted capture used to leave a directory holding a header-only
    all_segments.csv and a manifest. At the analysis that is
    indistinguishable from a session that ran and recorded nothing, which is
    a different fault with a different cause."""
    body = SRC[SRC.index("def main("):]
    mk = body.index("out.mkdir(")
    for guard in ("PRE-CAPTURE CHECK FAILED", "CANNOT PIN THE CLUTCH"):
        assert body.index(guard) < mk, \
            "%r can refuse AFTER the output directory is created" % guard


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
