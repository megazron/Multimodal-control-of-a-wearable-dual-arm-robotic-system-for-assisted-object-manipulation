#!/usr/bin/env python3
"""Divergence: the number, and the four ways it can be a lie.

The reading this module exists to produce is "0.0000 rad", and that is also
what a broken comparison produces. Every test below is about keeping those
two apart.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from srl_teleop import divergence as dv                      # noqa: E402

N = dv.joint_names("left")
T = 1000.0


def side(vals, t=T, name="s"):
    s = dv.Side(name)
    s.update(N, vals, now=t)
    return s


# ------------------------------------------------- the zero vs the blank
def test_a_true_zero_and_a_missing_arm_do_not_render_the_same():
    """THE HEADLINE PROPERTY. Subtracting a missing thing gives 0.000, which
    renders as flawless tracking -- the same shape as 'IK BLOCKED, 0% success'
    computed over zero attempts."""
    a = dv.compare(side([0.1] * 7), side([0.1] * 7), N, now=T)
    b = dv.compare(side([0.1] * 7), dv.Side("real"), N, now=T)
    assert a.status == dv.OK and a.max_rad == 0.0 and a.measured
    assert b.status == dv.NO_REAL and not b.measured
    assert a.text() != b.text()
    assert b.text() == "--"


def test_the_module_ships_its_own_demonstration_of_that():
    ok, _, _ = dv.self_test()
    assert ok


def test_no_real_says_so_in_words():
    r = dv.compare(side([0.0] * 7), dv.Side("real"), N, now=T)
    assert "NOT zero divergence" in r.reason


# ---------------------------------------------------------- the freeze
def test_a_frozen_real_stream_is_caught_even_at_full_rate():
    """/real/joint_states publishes a CACHE at 100 Hz when the hardware
    component goes inactive -- measured, one distinct value per joint,
    peak-to-peak 0.000e+00. Arrival-rate freshness cannot see that."""
    real = dv.Side("real")
    sim = dv.Side("sim")
    for i in range(600):                       # 6 s at 100 Hz
        t = T + i * 0.01
        real.update(N, [0.2] * 7, now=t)       # never changes
        sim.update(N, [0.2 + 0.001 * i] * 7, now=t)   # sim is moving
    r = dv.compare(sim, real, N, now=T + 6.0, sim_is_moving=True)
    assert r.status == dv.REAL_FROZEN
    assert not r.measured
    assert real.n_msgs == 600 and real.n_changes == 1


def test_a_STILL_arm_beside_a_STILL_sim_is_not_called_frozen():
    """A false FROZEN costs a glance; crying wolf on every pause is how an
    indicator gets ignored."""
    real, sim = dv.Side("real"), dv.Side("sim")
    for i in range(600):
        t = T + i * 0.01
        real.update(N, [0.2] * 7, now=t)
        sim.update(N, [0.2] * 7, now=t)
    r = dv.compare(sim, real, N, now=T + 6.0, sim_is_moving=False)
    assert r.status == dv.OK


def test_unknown_motion_is_treated_as_moving():
    """The conservative direction: a missed FROZEN costs the belief that the
    arm is tracking."""
    real, sim = dv.Side("real"), dv.Side("sim")
    for i in range(600):
        real.update(N, [0.2] * 7, now=T + i * 0.01)
        sim.update(N, [0.2] * 7, now=T + i * 0.01)
    assert dv.compare(sim, real, N, now=T + 6.0,
                      sim_is_moving=None).status == dv.REAL_FROZEN


# ------------------------------------------------------------- coverage
def test_partial_joint_coverage_refuses_to_report_a_maximum():
    """A maximum over three joints of seven is silent about the four that
    might be anywhere."""
    real = dv.Side("real")
    real.update(N[:3], [0.0, 0.0, 0.0], now=T)
    r = dv.compare(side([0.5] * 7), real, N, now=T)
    assert r.status == dv.PARTIAL
    assert not r.measured
    assert r.n_compared == 3 and r.n_expected == 7
    assert "not a maximum" in r.reason


def test_staleness_on_either_side_blocks_the_reading():
    assert dv.compare(side([0.0] * 7, t=T - 5), side([0.0] * 7), N,
                      now=T).status == dv.SIM_STALE
    assert dv.compare(side([0.0] * 7), side([0.0] * 7, t=T - 5), N,
                      now=T).status == dv.REAL_STALE


# ------------------------------------------------------------ the number
def test_the_worst_joint_is_identified_not_just_the_magnitude():
    real = dv.Side("real")
    vals = [0.0] * 7
    vals[4] = -0.3
    real.update(N, vals, now=T)
    r = dv.compare(side([0.0] * 7), real, N, now=T)
    assert r.measured
    assert r.max_joint == "left_joint_5"
    assert r.max_rad == pytest.approx(0.3)


def test_ee_distance_never_falls_back_to_zero():
    """The distance between a pose and a missing pose is not zero."""
    assert dv.ee_distance(None, (1, 2, 3))[0] is None
    assert dv.ee_distance((1, 2, 3), None)[0] is None
    assert dv.ee_distance(None, None)[0] is None
    d, why = dv.ee_distance((0, 0, 0), (0.03, 0.04, 0.0))
    assert d == pytest.approx(0.05)


# -------------------------------------------------------------- the band
@pytest.mark.parametrize("v,want", [(0.0, "ok"), (0.10, "ok"), (0.20, "watch"),
                                    (0.35, "near"), (0.50, "trip"),
                                    (0.90, "trip")])
def test_the_band_follows_the_monitor_threshold(v, want):
    """`band()` lives here rather than in the GUI so the panel and the lag
    monitor cannot quietly disagree about where the threshold is."""
    assert dv.band(v, 0.5) == want


def test_an_unknown_divergence_has_its_own_band():
    assert dv.band(None, 0.5) == "unknown"
    assert dv.band(0.1, 0.0) == "unknown"
