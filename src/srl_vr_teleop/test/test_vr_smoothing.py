#!/usr/bin/env python3
"""The VR command smoother: what it must do, and what it must not carry.

Two separate obligations are tested here and they fail in different ways:

  THE CONTROL LAW must be better than the fixed EMA it replaced -- and better
  on BOTH axes at once, because beating a fixed filter on either one alone is
  free (retune it the other way and it wins that one back).

  THE STATE must not survive a run. A smoother holds a position, a velocity
  estimate and an adaptive cutoff; carried into a second run they are wrong by
  however far the first run ended up. That is exactly the defect that cost
  02_vr_teleop every grasping task through `filt`, and adding two more
  stateful objects to this node re-opens it unless reset() rebuilds them.
"""
import math

import numpy as np
import pytest

from srl_vr_teleop import vr_smoothing as sm

DT = 1.0 / 72.0


def _still(n=900, sigma=0.0015, seed=7):
    rng = np.random.default_rng(seed)
    return rng.normal(0.0, sigma, (n, 3))


def _ramp(n=900, v=0.40):
    t = np.arange(n) * DT
    return np.stack([v * t, np.zeros(n), np.zeros(n)], 1)


def _run(f, sig):
    f.reset()
    return np.array([f(sig[i], DT) for i in range(len(sig))])


def _still_rms(f, noise):
    return float(np.sqrt((_run(f, noise)[200:] ** 2).sum(1).mean())) * 1000


def _lag_mm(f, ramp, noise, v=0.40):
    out = _run(f, ramp + noise)
    return float(np.mean(ramp[400:, 0] - out[400:, 0])) * 1000


# ------------------------------------------------------------- the control law
def test_one_euro_dominates_the_ema_it_replaced():
    """Equal stillness, less lag. This is the whole justification.

    A filter that is merely quieter is not an improvement -- ema(0.08) is
    quieter still and lags 64 mm. The claim is DOMINANCE: pin the EMA to the
    1-Euro's own stillness and it must then lag more.
    """
    noise, ramp = _still(), _ramp()
    oe = sm.OneEuro()
    oe_still = _still_rms(oe, noise)
    oe_lag = _lag_mm(sm.OneEuro(), ramp, noise)

    lo, hi = 1e-4, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _still_rms(sm.LegacyEma(mid), noise) > oe_still:
            hi = mid
        else:
            lo = mid
    matched = 0.5 * (lo + hi)
    ema_lag = _lag_mm(sm.LegacyEma(matched), ramp, noise)

    assert oe_lag < ema_lag, (
        "1-Euro lagged %.2f mm against %.2f mm for an EMA at the same %.2f mm "
        "stillness -- it is no longer off the fixed filter's trade-off curve, "
        "so there is no reason to prefer it" % (oe_lag, ema_lag, oe_still))
    # and by a wide margin, not by a rounding error
    assert ema_lag > 5.0 * oe_lag


def test_the_shipped_tuning_beats_the_shipped_ema_on_both_axes():
    noise, ramp = _still(), _ramp()
    oe_s = _still_rms(sm.OneEuro(), noise)
    oe_l = _lag_mm(sm.OneEuro(), ramp, noise)
    em_s = _still_rms(sm.LegacyEma(0.6), noise)
    em_l = _lag_mm(sm.LegacyEma(0.6), ramp, noise)
    assert oe_s < em_s, "%.2f vs %.2f mm still" % (oe_s, em_s)
    assert oe_l < em_l, "%.2f vs %.2f mm lag" % (oe_l, em_l)


def test_lag_does_not_depend_on_the_tick_rate():
    """The bug in the filter this replaces.

    The EMA applied a constant alpha per TICK, so the amount of filtering
    depended on the scheduler. The mapper's timer runs at 100 Hz and the Quest
    delivers at 72; neither is regular under load.
    """
    def lag_at(rate, mk):
        d, k, v = 1.0 / rate, int(4.0 * rate), 0.40
        f = mk()
        f.reset()
        out = [f(np.array([v * i * d, 0.0, 0.0]), d) for i in range(k)]
        return (v * (k - 1) * d - out[-1][0]) * 1000

    a, b = lag_at(72.0, sm.OneEuro), lag_at(144.0, sm.OneEuro)
    assert abs(a - b) < 0.5 * abs(a) + 0.5, "%.2f vs %.2f mm" % (a, b)

    # the control: the EMA really did have this defect, so a future change
    # that accidentally made LegacyEma dt-aware would be caught here rather
    # than silently invalidating every pre-2026-08-26 recording
    c, d = lag_at(72.0, lambda: sm.LegacyEma(0.6)), \
        lag_at(144.0, lambda: sm.LegacyEma(0.6))
    assert abs(c - d) > 0.2 * abs(c)


def test_a_held_target_is_actually_reached():
    """A filter that never converges quietly shrinks the workspace."""
    f = sm.OneEuro()
    f.reset()
    f(np.zeros(3), DT)
    tgt = np.array([0.20, 0.0, 0.0])
    for _ in range(int(3.0 / DT)):
        out = f(tgt, DT)
    assert float(np.linalg.norm(tgt - out)) * 1000 < 1.0


# ------------------------------------------------------------------ quaternion
def test_a_quaternion_sign_flip_is_not_a_360_degree_swing():
    """q and -q are the SAME rotation, and the runtime may flip at any time."""
    q = sm.q_norm(np.array([0.1, 0.2, 0.3, 0.9]))
    f = sm.OneEuroQuat()
    f.reset()
    for _ in range(21):
        f(q, DT)
    for _ in range(20):
        out = f(-q, DT)
    assert math.degrees(sm.q_angle(out, q)) < 0.5


def test_orientation_is_smoothed_at_all():
    """It was not, until 2026-08-26: the raw quaternion went straight to IK."""
    rng = np.random.default_rng(11)
    base = sm.q_norm(np.array([0.0, 0.0, 0.0, 1.0]))
    f = sm.OneEuroQuat()
    f.reset()
    raw, out = [], []
    for _ in range(900):
        p = sm.q_norm(base + rng.normal(0, 0.004, 4))
        o = f(p, DT)
        raw.append(math.degrees(sm.q_angle(p, base)))
        out.append(math.degrees(sm.q_angle(o, base)))
    assert float(np.mean(out[200:])) < 0.7 * float(np.mean(raw[200:]))


# ------------------------------------------------------------------- refusals
def test_an_unknown_smoothing_name_raises():
    """A typo must not silently select a different control law.

    That is how a session gets spent comparing two conditions that were the
    same one, and the comparison looks perfectly clean while it happens.
    """
    with pytest.raises(ValueError):
        sm.make("one-euro")
    with pytest.raises(ValueError):
        sm.make("")


def test_the_legacy_ema_is_bit_for_bit_what_it_was():
    """`ema` must reproduce pre-change recordings, so it is pinned.

    Same reason `motion_generator:=legacy` is pinned: a recording made before
    a control change has to stay reproducible or the change cannot be
    evaluated against it.
    """
    f = sm.LegacyEma(0.6)
    f.reset()
    xs = [np.array([0.0, 0, 0]), np.array([1.0, 0, 0]), np.array([1.0, 0, 0])]
    got = [f(x, DT)[0] for x in xs]
    # y0 = 0 ; y1 = .6*1 + .4*0 = .6 ; y2 = .6*1 + .4*.6 = .84
    assert got == pytest.approx([0.0, 0.6, 0.84])
    # and it is dt-BLIND on purpose -- the same sequence at a different dt
    f2 = sm.LegacyEma(0.6)
    f2.reset()
    assert [f2(x, DT * 3.0)[0] for x in xs] == pytest.approx(got)


# ----------------------------------------------------------- state, not carried
def test_reset_clears_every_piece_of_carried_state():
    """The 02_vr_teleop defect, guarded for the two new stateful objects.

    Drive the filter a long way from the origin, reset it, and it must behave
    exactly as a freshly built one -- not merely 'close'.
    """
    for mk, sig in ((sm.OneEuro, np.array([0.5, -0.3, 0.9])),
                    (sm.OneEuroQuat,
                     sm.q_norm(np.array([0.3, 0.1, 0.2, 0.9])))):
        used = mk()
        used.reset()
        for _ in range(400):
            used(sig, DT)
        used.reset()
        fresh = mk()
        fresh.reset()
        probe = (np.array([0.0, 0.0, 0.0]) if mk is sm.OneEuro
                 else sm.q_norm(np.array([0.0, 0.0, 0.0, 1.0])))
        a = [np.asarray(used(probe, DT)).copy() for _ in range(50)][-1]
        b = [np.asarray(fresh(probe, DT)).copy() for _ in range(50)][-1]
        assert np.allclose(a, b, atol=1e-12), (
            "%s carried state across reset(): %s vs %s" % (mk.__name__, a, b))


def test_the_first_sample_after_a_reset_is_passed_through():
    """Seeding at the origin would command the arm from (0,0,0) on re-engage.

    The mapper latches a fresh anchor at every clutch engage precisely so the
    re-engage jump is zero by construction; a filter that starts at zero would
    put the jump straight back.
    """
    f = sm.OneEuro()
    f.reset()
    p = np.array([0.42, -0.18, 1.10])
    assert np.allclose(f(p, DT), p)

    g = sm.OneEuroQuat()
    g.reset()
    q = sm.q_norm(np.array([0.2, 0.3, 0.1, 0.9]))
    assert np.allclose(g(q, DT), q)
