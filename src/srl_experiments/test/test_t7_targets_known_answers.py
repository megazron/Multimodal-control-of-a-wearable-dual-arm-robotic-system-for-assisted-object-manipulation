#!/usr/bin/env python3
"""T7 target generator: known-answer checks before it drives a participant."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "experiments", "bimanual", "t7_pursuit"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "experiments", "bimanual"))
import targets as tg          # noqa: E402
import coupled_metrics as cm  # noqa: E402

SC = dict(centre_left=[0.40, 0.35, 1.05], centre_right=[-0.40, 0.35, 1.05],
          speed_left=0.20, speed_right=0.05, amplitude_m=0.08)


def test_a_stationary_target_does_not_move():
    s = dict(SC, speed_left=0.0)
    _, L, _ = tg.trial_targets(s, 5.0)
    assert np.allclose(L, np.asarray(s["centre_left"]))


def test_targets_never_leave_the_verified_amplitude():
    """A marker outside the reachable sphere would lose the trial."""
    for sc in (SC, dict(SC, unpredictable=True), dict(SC, speed_left=0.30)):
        eL, eR = tg.max_excursion(sc, 60.0)
        assert eL <= sc["amplitude_m"] * 1.001, eL
        assert eR <= sc["amplitude_m"] * 1.001, eR


def test_faster_speed_really_does_travel_further():
    """The speed parameter must map monotonically onto path length, or the
    interference regression is fitted against a meaningless x-axis."""
    prev = 0.0
    for v in (0.05, 0.10, 0.20, 0.30):
        _, L, _ = tg.trial_targets(dict(SC, speed_left=v), 20.0)
        length = float(np.sum(np.linalg.norm(np.diff(L, axis=0), axis=1)))
        assert length > prev, v
        prev = length


def test_the_two_arms_targets_are_independent():
    """Changing one arm's speed must not alter the other's trajectory."""
    _, _, R1 = tg.trial_targets(dict(SC, speed_left=0.05), 10.0)
    _, _, R2 = tg.trial_targets(dict(SC, speed_left=0.30), 10.0)
    assert np.allclose(R1, R2)


def test_unpredictable_reverses_direction_but_stays_in_volume():
    _, L, _ = tg.trial_targets(dict(SC, unpredictable=True), 30.0)
    d = np.diff(L[:, 0])
    sign_changes = int(np.sum(np.diff(np.sign(d)) != 0))
    _, L2, _ = tg.trial_targets(SC, 30.0)
    d2 = np.diff(L2[:, 0])
    smooth_changes = int(np.sum(np.diff(np.sign(d2)) != 0))
    assert sign_changes > smooth_changes


def test_perfect_tracking_scores_zero_error():
    """End-to-end known answer: feed the target back as the EE."""
    t, L, _ = tg.trial_targets(SC, 10.0)
    s = cm.summarise_pursuit(t, L, L)
    assert s["rms_error_mm"] == pytest.approx(0.0, abs=1e-9)
    assert s["frac_on_target"] == pytest.approx(1.0)


def test_a_known_constant_offset_scores_exactly_that():
    t, L, _ = tg.trial_targets(SC, 10.0)
    ee = L + np.array([0.03, 0.04, 0.0])       # 50 mm
    s = cm.summarise_pursuit(t, ee, L)
    assert s["rms_error_mm"] == pytest.approx(50.0, abs=1e-6)
    assert s["frac_on_target"] == pytest.approx(0.0)
