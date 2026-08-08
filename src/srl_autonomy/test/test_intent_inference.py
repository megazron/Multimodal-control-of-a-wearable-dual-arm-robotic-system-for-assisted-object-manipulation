"""The three hard cases for intent inference, as executable tests.

These are the cases the brief singles out, and the third one -- the operator
changing their mind mid-reach -- is the one that matters: latching onto a
wrong goal and not releasing is worse than having no inference at all.
"""
import numpy as np

from srl_autonomy.intent_inference import IntentEstimator

EE = np.array([0.0, 0.0, 1.0])


def _dir(target, ee=EE):
    v = np.asarray(target, float) - ee
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- case 1
def test_no_objects_is_reported_not_faked():
    est = IntentEstimator()
    p = est.update([], EE, np.zeros((0, 3)), pointing=np.array([0, 1.0, 0]))
    assert len(p) == 0
    s = est.summary()
    assert s["state"] == "no_objects"
    assert s["top"] is None and s["top_p"] == 0.0
    assert s["ambiguous"] is False


def test_objects_disappearing_clears_the_estimate():
    """A stale distribution over objects that are gone is worse than none."""
    est = IntentEstimator()
    ids = ["tag_0", "tag_1"]
    pos = np.array([[0.4, 0.5, 1.0], [-0.4, 0.5, 1.0]])
    for _ in range(50):
        est.update(ids, EE, pos, pointing=_dir(pos[0]))
    assert est.summary()["top"] == "tag_0"
    est.update([], EE, np.zeros((0, 3)), pointing=_dir(pos[0]))
    assert est.summary()["state"] == "no_objects"


# ---------------------------------------------------------------- case 2
def test_equally_likely_objects_report_a_tie():
    """Two objects placed symmetrically about the pointing ray must stay
    near 1/N and be flagged ambiguous, NOT resolved by noise."""
    est = IntentEstimator()
    ids = ["a", "b"]
    pos = np.array([[0.3, 0.6, 1.0], [-0.3, 0.6, 1.0]])
    point = np.array([0.0, 1.0, 0.0])            # exactly between them
    for _ in range(100):
        est.update(ids, EE, pos, pointing=point)
    s = est.summary()
    assert abs(s["top_p"] - 0.5) < 0.02, s
    assert s["ambiguous"] is True
    assert s["margin"] < 0.15


def test_three_way_tie_stays_uniform():
    est = IntentEstimator()
    ids = ["a", "b", "c"]
    # three objects on a circle about the pointing axis: all equidistant and
    # all at the same angle from it
    pos = np.array([[0.3, 0.6, 1.0], [-0.15, 0.6, 1.26], [-0.15, 0.6, 0.74]])
    for _ in range(100):
        est.update(ids, EE, pos, pointing=np.array([0.0, 1.0, 0.0]))
    s = est.summary()
    assert abs(s["top_p"] - 1.0 / 3) < 0.03, s
    assert s["ambiguous"] is True


# ---------------------------------------------------------------- case 3
def test_operator_changes_mind_mid_reach():
    """Point at A until confident, then point at B. The estimate MUST switch,
    and within a bounded number of frames."""
    est = IntentEstimator()
    ids = ["A", "B"]
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])

    for _ in range(200):
        est.update(ids, EE, pos, pointing=_dir(pos[0]))
    s = est.summary()
    assert s["top"] == "A" and s["top_p"] > 0.9, s

    switched_at = None
    for k in range(1, 201):
        est.update(ids, EE, pos, pointing=_dir(pos[1]))
        if est.summary()["top"] == "B" and switched_at is None:
            switched_at = k
    s = est.summary()
    assert s["top"] == "B", "estimate LATCHED on the abandoned goal: %s" % s
    assert switched_at is not None
    # At 20 Hz this is the switch latency in seconds. Bounded by the
    # forgetting factor, which is exactly why forget > 0.
    assert switched_at < 60, "switch took %d frames" % switched_at


def test_forgetting_bounds_confidence():
    """With forget > 0 no object can reach probability 1, which is what makes
    the change-of-mind case recoverable at all."""
    est = IntentEstimator(forget=0.02)
    ids = ["A", "B"]
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])
    for _ in range(2000):
        est.update(ids, EE, pos, pointing=_dir(pos[0]))
    assert est.summary()["top_p"] < 0.999


def test_zero_forgetting_latches_which_is_why_it_is_not_the_default():
    """Documents the failure mode the default avoids."""
    est = IntentEstimator(forget=0.0)
    ids = ["A", "B"]
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])
    for _ in range(400):
        est.update(ids, EE, pos, pointing=_dir(pos[0]))
    latched = est.summary()["top_p"]
    for _ in range(60):
        est.update(ids, EE, pos, pointing=_dir(pos[1]))
    assert latched > 0.999
    # It does eventually move, but far slower than the default configuration.
    assert est.summary()["top"] == "A" or est.summary()["top_p"] < 0.6


# ---------------------------------------------------------------- cues
def test_pointing_is_used_when_velocity_is_frozen():
    """The pot-dropout case: commanded velocity reads ZERO while the operator
    is moving. Pointing must still drive the estimate."""
    est = IntentEstimator()
    ids = ["A", "B"]
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])
    for _ in range(100):
        est.update(ids, EE, pos, pointing=_dir(pos[1]),
                   ee_vel=np.zeros(3))          # frozen command
    s = est.summary()
    assert s["top"] == "B" and s["top_p"] > 0.8, s


def test_no_evidence_leaves_the_estimate_untouched():
    est = IntentEstimator()
    ids = ["A", "B"]
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])
    for _ in range(50):
        est.update(ids, EE, pos, pointing=_dir(pos[0]))
    before = est.p.copy()
    est.update(ids, EE, pos, pointing=None, ee_vel=None)
    assert np.allclose(before, est.p)


def test_new_object_can_still_win():
    """An object detected late starts at the uniform prior, not at zero."""
    est = IntentEstimator()
    pos_a = np.array([[0.5, 0.5, 1.0]])
    for _ in range(100):
        est.update(["A"], EE, pos_a, pointing=_dir(pos_a[0]))
    pos = np.array([[0.5, 0.5, 1.0], [-0.5, 0.5, 1.0]])
    for _ in range(80):
        est.update(["A", "B"], EE, pos, pointing=_dir(pos[1]))
    assert est.summary()["top"] == "B", est.summary()
