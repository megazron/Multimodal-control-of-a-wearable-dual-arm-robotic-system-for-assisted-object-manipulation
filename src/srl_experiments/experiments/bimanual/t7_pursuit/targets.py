#!/usr/bin/env python3
"""T7 moving targets: independent per arm, inside each arm's own reachable set.

Speeds vary INDEPENDENTLY across trials. That is not a detail -- it is what
makes the cross-arm interference coefficient identifiable at all. With the
two speeds yoked, `b_self` and `b_cross` in

    err_A ~ b0 + b_self*speed_A + b_cross*speed_B + b_int*speed_A*speed_B

are perfectly collinear and neither can be estimated.

Motion is a Lissajous inside a sphere of `amplitude` about the arm's verified
centre, so the target never leaves the reachable set and no trial can be lost
to an unreachable marker. `unpredictable` adds direction reversals at
irregular intervals for S4, which breaks anticipatory tracking without
leaving the same volume.
"""
import math

import numpy as np


def target_at(t, centre, speed, amplitude=0.08, unpredictable=False,
              seed=0):
    """World-frame target position at time t. speed in m/s (0 = stationary)."""
    c = np.asarray(centre, float)
    if speed <= 0.0:
        return c.copy()
    # angular rate that gives roughly `speed` along the path
    w = speed / max(amplitude, 1e-6)
    tt = t
    if unpredictable:
        rng = np.random.default_rng(seed)
        # reversal times every 1.5-3.5 s, deterministic per seed
        flips = np.cumsum(rng.uniform(1.5, 3.5, 40))
        sign = 1.0
        prev = 0.0
        acc = 0.0
        for f in flips:
            if t < f:
                break
            acc += sign * (f - prev)
            prev = f
            sign = -sign
        tt = acc + sign * (t - prev)
    # NORMALISED so the peak excursion is exactly `amplitude`. Unnormalised,
    # the y term pushes the maximum radius to sqrt(1 + 0.35^2) = 1.0595 of
    # it, and the marker leaves the sphere that was VERIFIED reachable --
    # which is precisely the "scenario that fails IK on the day and wastes a
    # participant" failure. Caught by
    # test_targets_never_leave_the_verified_amplitude.
    YW = 0.35
    norm = math.sqrt(1.0 + YW * YW)
    return c + (amplitude / norm) * np.array([math.sin(w * tt),
                                              YW * math.sin(0.7 * w * tt + 1.1),
                                              math.cos(w * tt)])


def trial_targets(scenario, duration_s, dt=0.02):
    """(t, left targets, right targets) for a verified scenario dict."""
    n = int(duration_s / dt)
    t = np.arange(n) * dt
    L, R = [], []
    for k in range(n):
        L.append(target_at(t[k], scenario["centre_left"],
                           scenario.get("speed_left", 0.0),
                           scenario.get("amplitude_m", 0.08),
                           scenario.get("unpredictable", False), seed=1))
        R.append(target_at(t[k], scenario["centre_right"],
                           scenario.get("speed_right", 0.0),
                           scenario.get("amplitude_m", 0.08),
                           scenario.get("unpredictable", False), seed=2))
    return t, np.array(L), np.array(R)


def max_excursion(scenario, duration_s=30.0):
    """Furthest any target gets from its centre -- must stay <= amplitude, or
    a trial could be lost to an unreachable marker."""
    _, L, R = trial_targets(scenario, duration_s)
    cl = np.asarray(scenario["centre_left"], float)
    cr = np.asarray(scenario["centre_right"], float)
    return (float(np.max(np.linalg.norm(L - cl, axis=1))),
            float(np.max(np.linalg.norm(R - cr, axis=1))))
