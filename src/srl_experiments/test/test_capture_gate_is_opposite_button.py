#!/usr/bin/env python3
"""The capture gate must be the OPPOSITE arm's button.

An arm's own button toggles that arm's clutch. Gating a sweep on it
disengages the clutch exactly when the sweep starts, so the commanded pose
freezes and the sim arm does not move -- the recording then looks like a
capture but contains no motion.

That is not hypothetical: it happened. In capture_20260806_130205 the mapping
was inverted and 41 of 42 directional segments recorded with the clutch out.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "trajectory_capture"))
from record_trajectories import GATE_INDEX  # noqa: E402

# [fsr1, fsr2, btn1, btn2]
BTN1, BTN2 = 2, 3
OWN_CLUTCH_BUTTON = {"left": BTN2, "right": BTN1}   # MEASURED


def test_gate_is_not_the_arms_own_button():
    for arm, own in OWN_CLUTCH_BUTTON.items():
        assert GATE_INDEX[arm] != own, (
            "%s segments are gated by btn%d, which is %s's OWN clutch button; "
            "starting a sweep would disengage the clutch under test"
            % (arm, own - 1, arm))


def test_gate_is_exactly_the_other_arms_button():
    assert GATE_INDEX["left"] == BTN1
    assert GATE_INDEX["right"] == BTN2


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
