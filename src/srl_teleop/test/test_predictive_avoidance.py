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


# ---------------------------------------------------------------------------
# THE NULL SPACE, ON A CONSTRUCTED ARM WHOSE ANSWER IS KNOWN
#
# The seed-fan sampler was measured useless on the real arm and the replacement
# is a Jacobian null-space projection. Its correctness cannot be argued from
# the real robot -- if it moves the elbow there, that is evidence about the
# Gen3 and not about the arithmetic -- so it is tested on a THREE-LINK PLANAR
# arm, where the redundancy is one-dimensional and the answer is written down.
#
# For a planar 3R arm with unit links the end-effector POSITION task is
# 2-dimensional, so the null space of the 2 x 3 position Jacobian is exactly
# one direction, and moving along it must leave the tip where it is. That is
# the whole claim, and it is checkable to machine precision.
import math

import numpy as np
import pytest

from srl_teleop.predictive_avoidance import (            # noqa: E402
    NullSpaceRetreat, clearance_gradient, jacobian, null_projector,
    pose_error)

L = (1.0, 1.0, 1.0)


def planar_fk(q):
    """Tip position and orientation of a 3R planar arm, in the module's form."""
    a = 0.0
    x = y = 0.0
    for qi, li in zip(q, L):
        a += float(qi)
        x += li * math.cos(a)
        y += li * math.sin(a)
    c, s = math.cos(a), math.sin(a)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (np.array([x, y, 0.0]), R)


def planar_elbow(q):
    """Position of the second joint -- the thing the retreat should move."""
    return np.array([L[0] * math.cos(q[0]), L[0] * math.sin(q[0]), 0.0])


def test_jacobian_matches_the_analytic_one():
    q = [0.3, 0.7, -0.4]
    J = jacobian(planar_fk, q)
    # analytic position rows for a planar 3R
    a1, a2, a3 = q[0], q[0] + q[1], q[0] + q[1] + q[2]
    dx = [-(L[0] * math.sin(a1) + L[1] * math.sin(a2) + L[2] * math.sin(a3)),
          -(L[1] * math.sin(a2) + L[2] * math.sin(a3)),
          -(L[2] * math.sin(a3))]
    dy = [L[0] * math.cos(a1) + L[1] * math.cos(a2) + L[2] * math.cos(a3),
          L[1] * math.cos(a2) + L[2] * math.cos(a3),
          L[2] * math.cos(a3)]
    assert np.allclose(J[0], dx, atol=1e-6)
    assert np.allclose(J[1], dy, atol=1e-6)
    # the tip's rotation about z is the sum of the joints, so that row is 1,1,1
    assert np.allclose(J[5], [1.0, 1.0, 1.0], atol=1e-6)


def test_null_space_motion_does_not_move_the_tip():
    """The claim, stated exactly: J qdot = 0 leaves the end effector alone."""
    q = np.array([0.3, 0.7, -0.4])
    Jpos = jacobian(planar_fk, q)[:2]        # position task only, 2 x 3
    N = null_projector(Jpos)
    d = N @ np.array([1.0, -2.0, 0.5])       # any direction, projected
    assert np.linalg.norm(d) > 1e-6, "the 3R position task must be redundant"
    moved = planar_fk(q + d * 1e-4)[0] - planar_fk(q)[0]
    assert np.linalg.norm(moved[:2]) < 1e-8, moved


def test_the_projector_is_a_projector():
    q = [0.2, -0.5, 1.1]
    N = null_projector(jacobian(planar_fk, q)[:2])
    assert np.allclose(N @ N, N, atol=1e-9)          # idempotent
    assert np.allclose(N, N.T, atol=1e-9)            # symmetric


def test_retreat_moves_the_elbow_and_not_the_hand():
    """The whole point: elbow away from an obstacle, tip unchanged.

    The obstacle is a point beside the elbow and `clearance` is the distance
    to it, so the gradient is well defined and the right answer is obvious:
    the elbow should end up further from it and the tip should not move.
    """
    q0 = np.array([0.9, -1.2, 0.6])
    obstacle = planar_elbow(q0) + np.array([0.05, -0.02, 0.0])

    def clearance(q):
        return float(np.linalg.norm(planar_elbow(q) - obstacle))

    def fk_pos_only(q):
        p, R = planar_fk(q)
        return (p, np.eye(3))        # orientation free, so the null space is 1D

    before = clearance(q0)
    r = NullSpaceRetreat(max_steps=12, step_rad=0.05,
                         max_ee_drift_m=0.0005)
    q, c, resid, steps, why = r.retreat(fk_pos_only, clearance, q0,
                                        target_m=before + 0.10)
    assert steps > 0, why
    assert c > before + 0.01, (before, c, why)
    # microns, not millimetres: the open-loop version of this step drifted
    # 5.5 mm on this same arm, which is what the task-space correction fixes
    assert resid < 1e-4, resid
    assert np.linalg.norm(planar_elbow(q) - planar_elbow(q0)) > 0.01


def test_retreat_reports_when_the_null_space_offers_nothing():
    """A non-redundant task must stop, and SAY it stopped, not loop."""
    q0 = np.array([0.4, 0.2, -0.3])

    def clearance(_q):
        return 0.10                                   # flat: zero gradient

    r = NullSpaceRetreat(max_steps=5)
    q, c, resid, steps, why = r.retreat(planar_fk, clearance, q0,
                                        target_m=0.50)
    assert steps == 0
    assert resid < 1e-9
    assert why in ("null space offers no direction", "no improvement",
                   "gradient unavailable"), why


def test_gradient_is_none_if_any_probe_is_none():
    def clearance(q):
        return None if q[0] > 0.5 else 0.2
    assert clearance_gradient(clearance, [0.5, 0.0, 0.0]) is None


def test_pose_error_is_zero_for_the_same_pose():
    p = planar_fk([0.1, 0.2, 0.3])
    assert np.allclose(pose_error(p, p), np.zeros(6), atol=1e-12)


def test_tangential_removes_the_inward_part_and_keeps_the_rest():
    """Constructed: heading straight at the wearer, and across them.

    `away` is +x. A step of (-1, 0, 0) is straight in and must be cancelled; a
    step of (0, 1, 0) is straight across and must survive untouched; a step of
    (-1, 1, 0) must keep its across part and lose its inward part.
    """
    from srl_teleop.predictive_avoidance import tangential_target
    away = (1.0, 0.0, 0.0)
    p = (0.0, 0.0, 0.0)
    assert np.allclose(tangential_target(p, (-1.0, 0.0, 0.0), away),
                       (0.0, 0.0, 0.0))
    assert np.allclose(tangential_target(p, (0.0, 1.0, 0.0), away),
                       (0.0, 1.0, 0.0))
    assert np.allclose(tangential_target(p, (-1.0, 1.0, 0.0), away),
                       (0.0, 1.0, 0.0))


def test_tangential_never_blocks_motion_that_moves_AWAY():
    """Retreating is always allowed. A guard that also stops escape is a trap."""
    from srl_teleop.predictive_avoidance import tangential_target
    got = tangential_target((0.0, 0.0, 0.0), (0.7, 0.2, 0.0), (1.0, 0.0, 0.0))
    assert np.allclose(got, (0.7, 0.2, 0.0))


def test_tangential_can_be_allowed_a_bounded_inward_creep():
    from srl_teleop.predictive_avoidance import tangential_target
    got = tangential_target((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
                            (1.0, 0.0, 0.0), max_into_m=0.05)
    assert np.allclose(got, (-0.05, 0.0, 0.0))


def test_tangential_with_no_direction_is_a_pass_through():
    """No nearest-point estimate means no basis to project onto. Say so by
    doing nothing, rather than inventing an axis."""
    from srl_teleop.predictive_avoidance import tangential_target
    got = tangential_target((1.0, 2.0, 3.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert np.allclose(got, (1.1, 2.0, 3.0))
