#!/usr/bin/env python3
"""KNOWN-ANSWER test for T9's interaction fit — the headline of the task.

Standing rule (CLAUDE.md): before a metric touches participant data it must
recover an answer known by construction. T7's `b_cross` earned this the hard
way — the pilot injected 90/60 and the fit returned 358/289, and it took a
separate calibration block to show that the fit was right and the claim about
it was wrong.

T9's `b_interaction` is the same shape of statistic and gets the same
treatment BEFORE any participant is run. The data here is constructed
arithmetic, not a rendered or simulated operator: an error that is linear in
autonomy level and sway amplitude by construction, so the coefficients are
known exactly.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
BIM = os.path.join(os.path.dirname(HERE), "experiments", "_archive", "bimanual")
sys.path.insert(0, BIM)

AUTONOMY_LEVEL = {"direct": 0.0, "assisted": 1.0, "shared": 2.0}


def fit(rows):
    """The same design matrix analyse_t9.py builds."""
    X, y = [], []
    for au, amp, err in rows:
        X.append([1.0, au, amp, au * amp])
        y.append(err)
    X, y = np.asarray(X), np.asarray(y)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ beta
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return beta, 1 - ss_res / max(ss_tot, 1e-9)


def synth(b0, b_auto, b_sway, b_int):
    rows = []
    for au in (0.0, 1.0, 2.0):
        for amp in (0.0, 20.0, 60.0, 100.0):
            rows.append((au, amp,
                         b0 + b_auto * au + b_sway * amp + b_int * au * amp))
    return rows


def test_recovers_an_injected_interaction_exactly():
    beta, r2 = fit(synth(10.0, -2.0, 0.40, -0.05))
    assert abs(beta[0] - 10.0) < 1e-8
    assert abs(beta[1] - (-2.0)) < 1e-8
    assert abs(beta[2] - 0.40) < 1e-8
    assert abs(beta[3] - (-0.05)) < 1e-8
    assert r2 > 0.999


def test_a_pure_main_effect_gives_ZERO_interaction():
    """Autonomy helps equally at every amplitude -> b_int must be 0.

    This is the H2-null case, and it must come out as an unambiguous zero
    rather than as a small number that invites over-reading.
    """
    beta, _ = fit(synth(10.0, -3.0, 0.40, 0.0))
    assert abs(beta[3]) < 1e-9
    assert abs(beta[1] - (-3.0)) < 1e-8


def test_the_SIGN_of_the_interaction_is_the_direction_of_the_claim():
    """Negative = autonomy's advantage GROWS with sway. That is H2."""
    grows, _ = fit(synth(10.0, -2.0, 0.40, -0.05))
    shrinks, _ = fit(synth(10.0, -2.0, 0.40, +0.05))
    assert grows[3] < 0 < shrinks[3]


def test_noise_does_not_move_the_estimate_much():
    rng = np.random.default_rng(7)
    base = synth(10.0, -2.0, 0.40, -0.05)
    noisy = [(au, amp, e + rng.normal(0, 0.5)) for au, amp, e in base] * 6
    beta, _ = fit(noisy)
    assert abs(beta[3] - (-0.05)) < 0.01


def test_a_yoked_design_is_UNIDENTIFIABLE_and_must_not_look_fine():
    """If autonomy and sway are varied together, the fit is degenerate.

    Encoded so nobody 'simplifies' the design into uninterpretability: with
    amplitude perfectly determined by condition, the columns are collinear and
    the coefficients are not separately estimable. lstsq will still return
    numbers -- that is exactly the danger -- so the test asserts the rank
    deficiency rather than the values.
    """
    rows = [(au, 50.0 * au, 10.0 - 2.0 * au) for au in (0.0, 1.0, 2.0)] * 4
    X = np.asarray([[1.0, au, amp, au * amp] for au, amp, _ in rows])
    assert np.linalg.matrix_rank(X) < 4


def check(commanded, measured):
    """The manipulation check exactly as analyse_t9.py applies it."""
    r = float(np.corrcoef(commanded, measured)[0, 1])
    slope = float(np.polyfit(commanded, measured, 1)[0])
    cover = (measured.max() - measured.min()) / (commanded.max()
                                                 - commanded.min())
    return (r > 0.7) and (slope > 0.5) and (cover > 0.5)


def test_the_manipulation_check_catches_a_wearer_who_stopped_swaying():
    """A TIRING WEARER IS THE FAILURE THIS CHECK EXISTS FOR.

    They produce a beautiful null result that has nothing to do with
    autonomy. And CORRELATION ALONE DOES NOT CATCH THEM: a wearer who
    plateaus at 20 mm while being asked for 20/60/100 still correlates
    r = +0.71, because a plateau is monotonic. This test found that, which is
    why the shipped check also requires slope and span coverage.
    """
    commanded = np.array([0.0, 20.0, 60.0, 100.0] * 3)
    honest = commanded * 0.9 + 2.0
    tired = np.array([0.0, 18.0, 20.0, 19.0] * 3)

    assert check(commanded, honest), "an honest wearer must pass"
    assert not check(commanded, tired), "a plateaued wearer must FAIL"
    # and the specific reason correlation was insufficient, pinned:
    assert float(np.corrcoef(commanded, tired)[0, 1]) > 0.7


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
