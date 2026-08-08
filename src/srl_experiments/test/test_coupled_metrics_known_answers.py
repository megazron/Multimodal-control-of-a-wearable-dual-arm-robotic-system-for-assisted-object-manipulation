#!/usr/bin/env python3
"""KNOWN-ANSWER TESTS. The standing rule, applied.

Three times a "system failure" in this project was the measuring instrument:
the 0.000 noise floor (oversampled duplicate rows), isotropy swinging 2x
(min-over-directions with rare tail failures), and detection at 0-4% (an
out-of-distribution synthetic renderer). Each was caught only by checking the
tool against a known-good reference.

So every metric below is fed an input whose answer is known by CONSTRUCTION
-- geometry and arithmetic, not rendering. That distinction matters: a
constructed ground truth (two points 300 mm apart really are 300 mm apart) is
valid, whereas a RENDERED ground truth is only as good as the renderer, which
is exactly what produced the 0-4% figure.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "experiments", "bimanual"))
import coupled_metrics as cm          # noqa: E402


# ----------------------------------------------------------- sling geometry
@pytest.mark.parametrize("sep_mm,expect_mm", [
    (200, 143.6), (280, 105.0), (330, 58.3), (340, 41.5), (349, 13.2)])
def test_sag_matches_the_closed_form_used_to_spec_the_object(sep_mm, expect_mm):
    """These are the numbers the object spec was derived from. If the code
    and the spec ever disagree, the fabricated sling is the wrong length."""
    assert abs(1000 * cm.sag(sep_mm / 1000.0) - expect_mm) < 0.2


def test_retention_threshold_is_where_the_spec_says():
    assert abs(1000 * cm.max_secure_separation() - 340.7) < 0.5
    assert cm.sling_retains(0.330) is True
    assert cm.sling_retains(0.349) is False


def test_sag_is_zero_when_the_sling_is_pulled_straight():
    assert cm.sag(cm.SLING_L) == 0.0
    assert cm.sag(cm.SLING_L + 0.05) == 0.0        # over-extended, not NaN


# ------------------------------------------------------------- separation
def test_separation_error_sign_and_magnitude():
    L = [0.0, 0.0, 1.0]
    R = [0.35, 0.0, 1.0]                      # 350 mm apart, nominal 300
    assert abs(cm.separation_error_mm(L, R, 0.30) - 50.0) < 1e-6
    R2 = [0.25, 0.0, 1.0]
    assert abs(cm.separation_error_mm(L, R2, 0.30) + 50.0) < 1e-6


def test_height_difference_keeps_its_sign():
    """The ball rolls to the LOW side, so an unsigned metric would lose the
    information that says which way it escaped."""
    assert cm.height_difference_mm([0, 0, 1.05], [0.3, 0, 1.00]) == pytest.approx(50.0)
    assert cm.height_difference_mm([0, 0, 1.00], [0.3, 0, 1.05]) == pytest.approx(-50.0)


# ------------------------------------------------------------------- tilt
def test_tilt_matches_hand_computed_angles():
    """20 mm over 300 mm is 3.81 deg; 60 mm is 11.31 deg -- the two
    thresholds the protocol uses."""
    assert cm.tilt_deg([0, 0, 1.02], [0.3, 0, 1.00]) == pytest.approx(
        math.degrees(math.atan2(0.020, 0.300)), abs=1e-9)
    assert cm.tilt_deg([0, 0, 1.06], [0.3, 0, 1.00]) == pytest.approx(11.31, abs=0.01)


def test_tilt_is_height_independent():
    """Moving the task from a 0.885 m table to a 1.15 m stand must not change
    the metric -- only the reachability of the path changes."""
    a = cm.tilt_deg([0, 0, 0.905], [0.3, 0, 0.885])
    b = cm.tilt_deg([0, 0, 1.170], [0.3, 0, 1.150])
    assert a == pytest.approx(b, abs=1e-12)


# ------------------------------------------------------- path efficiency
def test_path_efficiency_is_1_for_a_straight_line():
    pts = [[0, 0, 1.0], [0.1, 0, 1.0], [0.2, 0, 1.0]]
    assert cm.path_efficiency(pts) == pytest.approx(1.0)


def test_path_efficiency_of_a_known_detour():
    """Out 1 m, across 1 m, back 1 m: straight line sqrt(2)/3 of the path."""
    pts = [[0, 0, 0], [1, 0, 0], [1, 1, 0]]
    assert cm.path_efficiency(pts) == pytest.approx(math.sqrt(2) / 2.0)


# ----------------------------------------------------------------- pursuit
def test_tracking_error_of_a_known_offset():
    assert cm.tracking_error_mm([0, 0, 0], [0.03, 0.04, 0]) == pytest.approx(50.0)


def test_phase_lag_recovers_an_injected_delay():
    """THE key known-answer test for T7: build a signal with a lag put in on
    purpose and confirm the estimator returns it."""
    dt = 0.02
    t = np.arange(0, 10, dt)
    target = np.sin(2 * math.pi * 0.5 * t)
    lag_samples = 7
    ee = np.concatenate([np.zeros(lag_samples), target])[:len(t)]
    got = cm.phase_lag_s(t, ee, target)
    assert got == pytest.approx(lag_samples * dt, abs=dt)


def test_time_on_target_counts_the_right_samples():
    dt = 0.02
    t = np.arange(0, 1.0, dt)
    ee = [[0, 0, 0]] * len(t)
    # half the samples 10 mm away, half 100 mm away
    tgt = ([[0.01, 0, 0]] * (len(t) // 2)) + ([[0.10, 0, 0]] * (len(t) - len(t) // 2))
    s = cm.summarise_pursuit(t, ee, tgt, tol_mm=40.0)
    assert s["frac_on_target"] == pytest.approx(0.5, abs=0.02)


# ------------------------------------------------------------ interference
def test_interference_coefficient_recovers_an_injected_effect():
    """Synthesise trials with a KNOWN cross-arm coefficient and confirm the
    fit returns it. Without this the headline T7 number would be an
    unvalidated regression."""
    rng = np.random.default_rng(0)
    B_SELF, B_CROSS, B_INT = 12.0, 7.5, 2.0
    rows = []
    for sa in (0.05, 0.10, 0.20, 0.30):
        for sb in (0.05, 0.10, 0.20, 0.30):
            err = 20 + B_SELF * sa + B_CROSS * sb + B_INT * sa * sb
            rows.append({"speed_left": sa, "speed_right": sb,
                         "rms_error_mm_left": err + rng.normal(0, 0.05)})
    fit = cm.interference_coefficient(rows, "left")
    assert fit["b_cross"] == pytest.approx(B_CROSS, abs=0.5)
    assert fit["b_self"] == pytest.approx(B_SELF, abs=0.5)
    assert fit["r2"] > 0.99


def test_interference_is_zero_when_the_other_arm_does_not_matter():
    rows = [{"speed_left": sa, "speed_right": sb,
             "rms_error_mm_left": 20 + 10 * sa}
            for sa in (0.05, 0.1, 0.2, 0.3) for sb in (0.05, 0.1, 0.2, 0.3)]
    fit = cm.interference_coefficient(rows, "left")
    assert abs(fit["b_cross"]) < 1e-6


def test_yoked_speeds_cannot_identify_interference():
    """If both targets always move at the same speed, b_self and b_cross are
    collinear. The design REQUIRES independent speeds, and this is why."""
    rows = [{"speed_left": s, "speed_right": s, "rms_error_mm_left": 20 + 10 * s}
            for s in (0.05, 0.1, 0.2, 0.3)]
    fit = cm.interference_coefficient(rows, "left")
    # under collinearity the split between self and cross is arbitrary;
    # what must NOT happen is a confident wrong answer being reported
    assert not (abs(fit["b_cross"]) < 1e-6 and abs(fit["b_self"] - 10) < 1e-6)


def test_dual_task_cost_is_a_proportion():
    assert cm.dual_task_cost(150.0, 100.0) == pytest.approx(0.5)
    assert cm.dual_task_cost(100.0, 100.0) == pytest.approx(0.0)
    assert math.isnan(cm.dual_task_cost(100.0, 0.0))


# ------------------------------------------------------ trial roll-up
def test_summarise_transport_rigid_vs_compliant_use_different_thresholds():
    dt = 0.02
    t = np.arange(0, 2.0, dt)
    # grippers 310 mm apart, left 70 mm high -> tilt 12.7 deg, over the 11.3
    # rigid threshold; separation stays inside the compliant one.
    L = [[0.0, 0.35, 1.15 + 0.070]] * len(t)
    R = [[0.31, 0.35, 1.15]] * len(t)
    rigid = cm.summarise_transport(t, L, R, 0.310, "rigid")
    comp = cm.summarise_transport(t, L, R, 0.310, "compliant")
    assert rigid["time_above_threshold_s"] == pytest.approx(2.0, abs=0.05)
    assert comp["time_above_threshold_s"] == pytest.approx(0.0, abs=0.05)
    assert comp["min_sag_mm"] > 2 * 1000 * cm.BALL_R
