#!/usr/bin/env python3
"""Smoothing for a human-held input device driving a robot arm.

    python3 -m srl_teleop.smoothing --self-test

BOTH input paths use this one implementation. It lives HERE and not in
`srl_vr_teleop` because the dependency arrow runs one way (HARD CONSTRAINT 7):
`srl_teleop` imports nothing in-repo, `srl_vr_teleop` already imports
`srl_teleop`, so the shared maths can only sit on this side of the arrow.
`srl_vr_teleop.vr_smoothing` is a re-export shim over this module -- the
filter that shipped there on 2026-08-26 is this code, moved, not forked.

WHY A FIXED EMA IS THE WRONG SHAPE FOR THIS JOB
-----------------------------------------------
`vr_pose_mapper` smoothed position with a first-order EMA at a fixed alpha
(and `master_pose_node` did the same to the master arm's FK tip) and neither
smoothed orientation at all.  A fixed alpha buys jitter rejection and
lag with ONE constant, and they trade directly against each other:

    alpha high -- the controller's tremor reaches the arm.  On a Quest the
                  reported position of a still controller wanders a few
                  millimetres, and a still hand should give a still arm.
    alpha low  -- a hand moving at constant speed produces a CONSTANT position
                  offset behind it, because a first-order low pass has a phase
                  lag proportional to velocity.  The operator is across the
                  room watching the metal, so that reads as sponginess.

There is no value that fixes both, because they are not the same signal: the
tremor is high frequency at low amplitude, the motion is low frequency at high
amplitude.

WHAT THIS USES INSTEAD
----------------------
The 1-Euro filter (Casiez, Roussel & Vogel, CHI 2012), which is the standard
answer for interactive pointing and hand tracking.  It is a first-order low
pass whose cutoff RISES WITH SPEED:

    cutoff = min_cutoff + beta * |estimated speed|

Slow or still, the cutoff is low and the tremor is filtered hard.  Moving, the
cutoff rises and the lag collapses.  One filter, two behaviours, and the knee
between them is `beta`.

IT IS dt-CORRECT, WHICH THE OLD ONE WAS NOT
-------------------------------------------
The EMA applied a constant alpha once per timer tick.  The mapper's timer runs
at 100 Hz and the Quest delivers at 72, and neither is regular under load, so
the amount of filtering actually applied varied with whatever the scheduler
did.  Every alpha here is computed from the MEASURED dt, so the filter's
frequency response is a property of the filter rather than of the machine's
load.

ORIENTATION IS FILTERED TOO, AND IT IS NOT A COMPONENT-WISE EMA
---------------------------------------------------------------
Averaging quaternion components and renormalising is only approximately a
rotation average, and it fails outright across a sign flip: q and -q are the
same rotation, so a filter that sees the components jump sign chases a 360 deg
excursion that never physically happened.  This slerps, and canonicalises the
sign against the previous output first.  The "speed" driving the adaptive
cutoff is the angular rate in rad/s.

NOTHING HERE IMPORTS ROS.  That is deliberate: the filter is the part with the
maths in it, so it is tested directly against constructed signals whose answer
is known, rather than through a node.
"""
from __future__ import annotations

import math

import numpy as np

TWO_PI = 2.0 * math.pi


def alpha_for(dt, cutoff_hz):
    """The one-pole coefficient giving `cutoff_hz` at a step of `dt`.

    tau = 1/(2 pi fc);  alpha = dt / (tau + dt).  Returned in (0, 1]: alpha=1
    is "no filtering", which is what an infinite cutoff or a huge dt means and
    is the safe direction to fail in -- a filter that stops filtering passes
    the operator's hand through, while one that clamps the other way would
    freeze the arm.
    """
    if dt <= 0.0:
        return 1.0
    if cutoff_hz <= 0.0:
        return 0.0
    tau = 1.0 / (TWO_PI * cutoff_hz)
    return float(dt / (tau + dt))


class LowPass:
    """One-pole low pass over a scalar or a vector, driven by measured dt."""

    def __init__(self):
        self.y = None

    def reset(self):
        self.y = None

    def __call__(self, x, a):
        x = np.asarray(x, float)
        if self.y is None:
            self.y = x.copy()
        else:
            self.y = a * x + (1.0 - a) * self.y
        return self.y


class OneEuro:
    """1-Euro filter over a vector (position, in metres).

    min_cutoff  cutoff in Hz when the hand is still. LOWER = steadier when
                still, and the only cost is lag at low speed, where lag is
                cheap. 1.0 Hz is the paper's starting point.
    beta        how fast the cutoff opens with speed, in Hz PER METRE PER
                SECOND. HIGHER = less lag when moving fast, at the price of
                letting more tremor through during fast motion, where it is
                not visible anyway.

                MIND THE UNITS. The 1-Euro paper's worked examples are in
                PIXELS and quote beta around 0.001-1. This signal is in
                METRES, which is ~1000x smaller for the same physical motion,
                so a beta copied from the paper leaves the cutoff pinned at
                min_cutoff and the filter degenerates into a very heavy fixed
                low pass. Measured here with beta=0.35: 55.8 mm of lag at
                0.40 m/s, against 3.7 mm for the EMA it was meant to beat.
                The self-test below is what caught that, and it is why the
                lag comparison is a CHECK and not a printed number.
    d_cutoff    cutoff of the low pass on the speed ESTIMATE itself. The speed
                estimate is a finite difference and so is noisier than the
                signal; without this the cutoff would be modulated by noise.
    """

    def __init__(self, min_cutoff=0.5, beta=100.0, d_cutoff=0.2):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x = LowPass()
        self._dx = LowPass()
        self._prev = None
        self.speed = 0.0
        self.cutoff = float(min_cutoff)

    def reset(self):
        self._x.reset()
        self._dx.reset()
        self._prev = None
        self.speed = 0.0
        self.cutoff = self.min_cutoff

    def __call__(self, x, dt):
        x = np.asarray(x, float)
        if self._prev is None:
            # FIRST SAMPLE IS PASSED THROUGH, not blended toward zero. Seeding
            # the filter at the origin would command the arm from (0,0,0) to
            # the hand on the first tick after every engage.
            self._prev = x.copy()
            self._x.y = x.copy()
            return x.copy()
        dx = (x - self._prev) / dt if dt > 0 else np.zeros_like(x)
        self._prev = x.copy()
        dxh = self._dx(dx, alpha_for(dt, self.d_cutoff))
        self.speed = float(np.linalg.norm(dxh))
        self.cutoff = self.min_cutoff + self.beta * self.speed
        return self._x(x, alpha_for(dt, self.cutoff)).copy()


# ------------------------------------------------------------------ quaternion
def q_norm(q):
    q = np.asarray(q, float)
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def q_canon(q, ref):
    """`q` with the sign that puts it on the same hemisphere as `ref`.

    q and -q ARE THE SAME ROTATION. A filter that does not do this sees the
    controller's quaternion flip sign -- which the runtime is free to do at
    any moment -- and interpolates the long way round, swinging the wrist
    through 360 deg for no physical reason. It is the single nastiest bug in
    quaternion smoothing because the input is perfectly valid.
    """
    q = np.asarray(q, float)
    return -q if float(np.dot(q, np.asarray(ref, float))) < 0.0 else q


def q_slerp(a, b, t):
    """Shortest-arc slerp; falls back to lerp when the two are nearly equal."""
    a, b = q_norm(a), q_canon(q_norm(b), a)
    d = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if d > 0.9995:
        return q_norm(a + t * (b - a))
    th = math.acos(d)
    s = math.sin(th)
    return (math.sin((1.0 - t) * th) / s) * a + (math.sin(t * th) / s) * b


def q_angle(a, b):
    """Angle between two rotations, radians, in [0, pi]."""
    d = abs(float(np.dot(q_norm(a), q_norm(b))))
    return 2.0 * math.acos(float(np.clip(d, -1.0, 1.0)))


class OneEuroQuat:
    """1-Euro over a rotation. Blends by slerp; speed is rad/s."""

    def __init__(self, min_cutoff=0.5, beta=8.0, d_cutoff=0.5):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.y = None
        self._prev = None
        self._w = LowPass()
        self.speed = 0.0
        self.cutoff = float(min_cutoff)

    def reset(self):
        self.y = None
        self._prev = None
        self._w.reset()
        self.speed = 0.0
        self.cutoff = self.min_cutoff

    def __call__(self, q, dt):
        q = q_norm(q)
        if self.y is None:
            self.y = q.copy()
            self._prev = q.copy()
            return q.copy()
        q = q_canon(q, self.y)
        w = q_angle(q, self._prev) / dt if dt > 0 else 0.0
        self._prev = q.copy()
        self.speed = float(self._w(np.array([w]), alpha_for(dt, self.d_cutoff))[0])
        self.cutoff = self.min_cutoff + self.beta * self.speed
        self.y = q_norm(q_slerp(self.y, q, alpha_for(dt, self.cutoff)))
        return self.y.copy()


class LegacyEma:
    """The filter this replaced, kept so old recordings reproduce byte for byte.

    Deliberately dt-BLIND, because that is what it was: one constant alpha per
    tick regardless of how long the tick took. `motion_generator:=legacy`
    exists for the same reason on the follower side -- a recording made before
    a control change has to stay reproducible, or the change cannot be
    evaluated against it.
    """

    def __init__(self, alpha=0.6):
        self.alpha = float(alpha)
        self.y = None

    def reset(self):
        self.y = None

    def __call__(self, x, dt):
        x = np.asarray(x, float)
        if self.y is None:
            self.y = x.copy()
        else:
            self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y.copy()


def make(kind, **kw):
    """Position filter by name. Unknown names RAISE rather than defaulting.

    A typo in a parameter silently selecting a different control law is how a
    session gets spent comparing two conditions that were the same one.
    """
    if kind == "one_euro":
        return OneEuro(kw.get("min_cutoff", 0.5), kw.get("beta", 100.0),
                       kw.get("d_cutoff", 0.2))
    if kind == "ema":
        return LegacyEma(kw.get("alpha", 0.6))
    if kind == "none":
        return _Passthrough()
    raise ValueError(
        "unknown smoothing %r -- expected one_euro, ema or none" % (kind,))


class _Passthrough:
    def reset(self):
        pass

    def __call__(self, x, dt):
        return np.asarray(x, float).copy()


# ===========================================================================
#  SELF-TEST.  Constructed signals only: the ground truth is arithmetic, so a
#  failure here is a failure of the filter and not of a renderer or a rig.
# ===========================================================================
def _self_test():
    rng = np.random.default_rng(7)
    dt = 1.0 / 72.0
    n = 900
    fails = []

    n_checks = [0]

    def check(name, cond, detail=""):
        # COUNTED HERE, not written as a literal at the bottom. A hand-kept
        # total drifts the moment a check is added, and a suite that says
        # "10 checks" while running 11 is a suite nobody can audit.
        n_checks[0] += 1
        print("  %-58s %s%s" % (name, "PASS" if cond else "FAIL",
                                "" if not detail else "   " + detail))
        if not cond:
            fails.append(name)

    print("1-Euro vs the fixed EMA it replaces (dt = 1/72 s)")
    print("-" * 72)

    # ---- A. a STILL hand with tremor. Less output motion is better.
    still = np.zeros((n, 3))
    noise = rng.normal(0.0, 0.0015, (n, 3))         # 1.5 mm rms, a real Quest
    for name, f, store in (("one_euro", OneEuro(0.5, 100.0, 0.2), []),
                           ("ema(0.6)", LegacyEma(0.6), []),
                           ("none", _Passthrough(), [])):
        f.reset()
        for i in range(n):
            store.append(f(still[i] + noise[i], dt))
        r = float(np.sqrt((np.asarray(store)[200:] ** 2).sum(1).mean())) * 1000
        print("    still-hand output rms: %-10s %6.2f mm" % (name, r))
        if name == "one_euro":
            oe_still = r
        elif name == "ema(0.6)":
            ema_still = r
        else:
            raw_still = r
    check("still hand: 1-Euro is quieter than the EMA",
          oe_still < ema_still * 0.5, "%.2f vs %.2f mm" % (oe_still, ema_still))
    # 0.30 is not arbitrary: the parameter sweep over min_cutoff, beta and
    # d_cutoff put the achievable floor at ~0.19 of raw, and the shipped
    # tuning trades a little of that back for lag. A filter that has drifted
    # above 0.30 has been detuned toward passthrough.
    check("still hand: 1-Euro removes most of the tremor",
          oe_still < raw_still * 0.30, "%.2f vs %.2f mm raw" % (oe_still, raw_still))

    # ---- B. a hand moving at CONSTANT SPEED. Less lag is better.
    #        Truth is a straight line, so the lag is exactly computable.
    v = 0.40                                          # m/s, a brisk reach
    t = np.arange(n) * dt
    ramp = np.stack([v * t, np.zeros(n), np.zeros(n)], 1)
    lags = {}
    for name, f in (("one_euro", OneEuro(0.5, 100.0, 0.2)),
                    ("ema(0.6)", LegacyEma(0.6))):
        f.reset()
        out = np.array([f(ramp[i] + noise[i], dt) for i in range(n)])
        # steady-state lag along the direction of travel
        lags[name] = float(np.mean(ramp[400:, 0] - out[400:, 0])) * 1000
        print("    constant-speed lag:    %-10s %6.2f mm at %.2f m/s"
              % (name, lags[name], v))
    check("moving hand: 1-Euro lags less than the EMA",
          lags["one_euro"] < lags["ema(0.6)"],
          "%.2f vs %.2f mm" % (lags["one_euro"], lags["ema(0.6)"]))

    # ---- C. THE POINT OF THE WHOLE THING: DOMINANCE, not a single win.
    #
    #        Beating the shipped EMA on stillness alone proves nothing -- any
    #        heavier filter does that, and pays in lag. Beating it on lag
    #        alone proves less than nothing: ema(0.9) has tau = 1.5 ms, a
    #        cutoff of 103 Hz against a 36 Hz Nyquist, i.e. it is very nearly
    #        a passthrough, and comparing lag against a passthrough is not a
    #        comparison. The claim worth making is DOMINANCE: retune the EMA
    #        until it is as STILL as the 1-Euro, and its lag must then be
    #        worse. That says the adaptive filter is off the fixed filter's
    #        trade-off curve entirely, which is the only thing that justifies
    #        replacing it.
    def ema_still(alpha):
        f = LegacyEma(alpha)
        f.reset()
        o = np.array([f(still[i] + noise[i], dt) for i in range(n)])
        return float(np.sqrt((o[200:] ** 2).sum(1).mean())) * 1000

    def ema_lag(alpha):
        f = LegacyEma(alpha)
        f.reset()
        o = np.array([f(ramp[i] + noise[i], dt) for i in range(n)])
        return float(np.mean(ramp[400:, 0] - o[400:, 0])) * 1000

    # bisect for the alpha matching the 1-Euro's stillness (lower alpha is
    # stiller, so the function is monotonic and this terminates)
    lo_a, hi_a = 1e-4, 1.0
    for _ in range(60):
        mid = 0.5 * (lo_a + hi_a)
        if ema_still(mid) > oe_still:
            hi_a = mid
        else:
            lo_a = mid
    a_match = 0.5 * (lo_a + hi_a)
    matched_still, matched_lag = ema_still(a_match), ema_lag(a_match)
    print("    an EMA retuned to the SAME stillness: alpha=%.4f" % a_match)
    print("      ema(%.4f):  still %5.2f mm, lag %6.2f mm"
          % (a_match, matched_still, matched_lag))
    print("      1-Euro:      still %5.2f mm, lag %6.2f mm"
          % (oe_still, lags["one_euro"]))
    check("1-Euro DOMINATES the EMA: equal stillness, less lag",
          lags["one_euro"] < matched_lag,
          "%.2f vs %.2f mm at equal %.2f mm stillness"
          % (lags["one_euro"], matched_lag, oe_still))

    # ---- D. dt-correctness. The SAME motion sampled at two rates must come
    #        out with the same shape. The old EMA cannot do this and that is
    #        the bug: its filtering depended on the machine's tick rate.
    def ramp_at(rate, filt):
        d = 1.0 / rate
        k = int(4.0 * rate)
        f = filt()
        out = [f(np.array([v * i * d, 0, 0]), d) for i in range(k)]
        return float(v * (k - 1) * d - out[-1][0]) * 1000
    oe72 = ramp_at(72.0, lambda: OneEuro(0.5, 100.0, 0.2))
    oe144 = ramp_at(144.0, lambda: OneEuro(0.5, 100.0, 0.2))
    em72 = ramp_at(72.0, lambda: LegacyEma(0.6))
    em144 = ramp_at(144.0, lambda: LegacyEma(0.6))
    print("    lag at 72 / 144 Hz:    one_euro %5.2f / %5.2f mm"
          % (oe72, oe144))
    print("                           ema      %5.2f / %5.2f mm"
          % (em72, em144))
    check("1-Euro's lag is a property of the FILTER, not the tick rate",
          abs(oe72 - oe144) < 0.5 * abs(oe72) + 0.5,
          "%.2f vs %.2f mm" % (oe72, oe144))
    check("the EMA's lag DID depend on the tick rate (the bug being fixed)",
          abs(em72 - em144) > 0.2 * abs(em72),
          "%.2f vs %.2f mm" % (em72, em144))

    # ---- E. quaternion: a sign flip must not move the arm.
    q = q_norm(np.array([0.1, 0.2, 0.3, 0.9]))
    f = OneEuroQuat(0.5, 8.0, 0.5)
    f.reset()
    f(q, dt)
    for _ in range(20):
        out = f(q, dt)
    a1 = math.degrees(q_angle(out, q))
    for _ in range(20):
        out = f(-q, dt)                              # SAME rotation, flipped
    a2 = math.degrees(q_angle(out, q))
    print("    after 20 flipped-sign samples: %.4f deg from the truth" % a2)
    check("a quaternion sign flip is not a 360 deg wrist swing",
          a2 < 0.5 and a1 < 0.5, "%.4f deg" % a2)

    # ---- F. quaternion tremor is actually reduced.
    base = q_norm(np.array([0.0, 0.0, 0.0, 1.0]))
    f.reset()
    raw_a, out_a = [], []
    for _ in range(n):
        pert = q_norm(base + rng.normal(0, 0.004, 4))
        o = f(pert, dt)
        raw_a.append(math.degrees(q_angle(pert, base)))
        out_a.append(math.degrees(q_angle(o, base)))
    rr = float(np.mean(raw_a[200:]))
    oo = float(np.mean(out_a[200:]))
    print("    orientation tremor: raw %.3f deg -> filtered %.3f deg" % (rr, oo))
    check("orientation tremor is reduced (it was NOT filtered at all before)",
          oo < rr * 0.7, "%.3f -> %.3f deg" % (rr, oo))

    # AND THE LAG IT COSTS IS BOUNDED. Orientation used to be passed through
    # raw, so this filter can only ADD orientation lag -- that is the price of
    # the line above and it has to be stated, not discovered on a lab day. At
    # 1.5 rad/s, a brisk wrist roll, the shipped tuning lags under 1.5 deg,
    # which at the 0.11 m pad offset is under 3 mm at the fingertips.
    w = 1.5
    truth = [q_norm(np.array([math.sin(w * i * dt / 2), 0.0, 0.0,
                              math.cos(w * i * dt / 2)])) for i in range(n)]
    f.reset()
    outs = [f(q_norm(truth[i] + rng.normal(0, 0.004, 4)), dt) for i in range(n)]
    qlag = float(np.mean([math.degrees(q_angle(outs[i], truth[i]))
                          for i in range(400, n)]))
    print("    orientation lag at %.1f rad/s: %.3f deg (%.2f mm at the pads)"
          % (w, qlag, math.radians(qlag) * 0.11 * 1000))
    check("the orientation lag it costs is bounded", qlag < 1.5,
          "%.3f deg" % qlag)

    # ---- G. a step must still arrive. A filter that never converges is a
    #         filter that quietly limits the workspace.
    f2 = OneEuro(0.5, 100.0, 0.2)
    f2.reset()
    f2(np.zeros(3), dt)
    tgt = np.array([0.20, 0.0, 0.0])
    for _ in range(int(3.0 / dt)):
        o = f2(tgt, dt)
    err = float(np.linalg.norm(tgt - o)) * 1000
    print("    3 s after a 200 mm step: %.3f mm short" % err)
    check("the filter converges to a held target", err < 1.0, "%.3f mm" % err)

    # ---- H. make() refuses a typo rather than silently choosing a law.
    try:
        make("one-euro")
        check("make() refuses an unknown name", False, "it accepted 'one-euro'")
    except ValueError:
        check("make() refuses an unknown name", True)

    print("-" * 72)
    print("%d checks, %d failed" % (n_checks[0], len(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    import sys
    if "--self-test" in sys.argv:
        sys.exit(_self_test())
    print(__doc__)
