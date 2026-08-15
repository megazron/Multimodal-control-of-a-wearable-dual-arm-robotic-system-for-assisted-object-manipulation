"""Predictive avoidance, on constructed inputs with known answers.

The arithmetic here is CONSTRUCTED, not rendered: straight-line extrapolation
and a max over a list. That is the one case this project's standing rule
allows synthetic data for, because the right answer is known by construction
rather than by another measurement.

What these do NOT test is whether the sampled solutions actually differ in the
null space -- that needs a solver and a wearer, and it is
`scripts/verify_predictive_avoidance.py`.
"""
import math

from srl_teleop.predictive_avoidance import (
    AvoidanceDecision, Lookahead, choose, ee_residual, seed_fan)


# ----------------------------------------------------------------- lookahead
def test_first_sample_predicts_nothing():
    """Absence of an estimate must not read as an estimate of no motion."""
    la = Lookahead(horizon_s=0.3)
    assert la.update(0.0, (0.0, 0.0, 0.0)) is None


def test_constant_velocity_extrapolates_exactly():
    # dt must be INSIDE stale_s, or the staleness guard correctly refuses to
    # extrapolate. The first version of this test used dt = 1.0 s against the
    # 0.5 s default and failed -- the test was wrong, not the guard.
    la = Lookahead(horizon_s=0.5, stale_s=0.5)
    la.update(0.0, (0.0, 0.0, 0.0))
    pred, vel = la.update(0.1, (0.02, 0.0, 0.0))   # 0.2 m/s in x
    assert abs(vel[0] - 0.2) < 1e-9
    # 0.02 now + 0.2 m/s * 0.5 s = 0.12
    assert abs(pred[0] - 0.12) < 1e-9
    assert abs(pred[1]) < 1e-9 and abs(pred[2]) < 1e-9


def test_a_stale_gap_predicts_nothing():
    """A paused master must not keep flying on its last velocity."""
    la = Lookahead(horizon_s=0.3, stale_s=0.5)
    la.update(0.0, (0.0, 0.0, 0.0))
    assert la.update(2.0, (0.2, 0.0, 0.0)) is None


def test_zero_timestep_predicts_nothing():
    la = Lookahead()
    la.update(1.0, (0.0, 0.0, 0.0))
    assert la.update(1.0, (0.1, 0.0, 0.0)) is None


# ------------------------------------------------------------------ seed fan
def test_the_fan_starts_with_the_unperturbed_seed():
    """So "nothing better" costs one solve and returns today's answer."""
    seed = [0.1] * 7
    fan = seed_fan(seed, 5)
    assert fan[0] == seed
    assert len(fan) == 5


def test_the_fan_is_symmetric_about_the_seed():
    """An asymmetric fan biases the search to one side of the wearer."""
    seed = [0.0] * 7
    fan = seed_fan(seed, 5, spread_rad=0.4, joints=(2,))
    offs = sorted(round(f[2], 6) for f in fan)
    assert offs[0] == -offs[-1]
    assert offs[1] == -offs[-2]


def test_a_fan_of_one_is_just_the_seed():
    seed = [0.3] * 7
    assert seed_fan(seed, 1) == [seed]


# -------------------------------------------------------------------- choose
FLOOR, TRIGGER = 0.05, 0.15


def test_clear_of_the_trigger_uses_the_unperturbed_solution():
    """No avoidance when nothing is near. The operator must not feel it."""
    cands = [([1] * 7, 0.30, 0.0), ([2] * 7, 0.40, 0.0)]
    d = choose(cands, FLOOR, TRIGGER)
    assert d.action == "clear"
    assert d.joints == [1] * 7          # NOT the clearer one


def test_inside_the_trigger_takes_the_clearest():
    cands = [([1] * 7, 0.08, 0.0), ([2] * 7, 0.22, 0.001), ([3] * 7, 0.11, 0.0)]
    d = choose(cands, FLOOR, TRIGGER)
    assert d.action == "avoided"
    assert d.joints == [2] * 7
    assert abs(d.clearance - 0.22) < 1e-9


def test_refuses_only_when_nothing_clears_the_floor():
    cands = [([1] * 7, 0.01, 0.0), ([2] * 7, 0.03, 0.0)]
    d = choose(cands, FLOOR, TRIGGER)
    assert d.action == "refuse"
    assert abs(d.clearance - 0.03) < 1e-9   # reports the best it could do


def test_a_solution_between_floor_and_trigger_is_used_not_refused():
    """THE POINT OF THE WHOLE MODULE. Degrade, never freeze."""
    cands = [([1] * 7, 0.06, 0.0), ([2] * 7, 0.09, 0.0)]
    d = choose(cands, FLOOR, TRIGGER)
    assert d.action == "avoided"
    assert d.clearance >= FLOOR


def test_no_candidates_refuses_rather_than_crashing():
    d = choose([], FLOOR, TRIGGER)
    assert d.action == "refuse"


def test_unsolvable_candidates_are_skipped_not_scored():
    """A None clearance is 'not measured', never 'infinitely clear'."""
    cands = [([1] * 7, None, None), ([2] * 7, 0.20, 0.0)]
    d = choose(cands, FLOOR, TRIGGER)
    assert d.action == "avoided"
    assert d.joints == [2] * 7


def test_all_unsolvable_refuses():
    d = choose([([1] * 7, None, None)], FLOOR, TRIGGER)
    assert d.action == "refuse"


# ------------------------------------------------------------- ee residual
def test_ee_residual_is_a_distance():
    assert abs(ee_residual((0, 0, 0), (0.003, 0.004, 0.0)) - 0.005) < 1e-12


def test_decision_repr_does_not_crash():
    assert "AvoidanceDecision" in repr(AvoidanceDecision("clear", 0.2))
