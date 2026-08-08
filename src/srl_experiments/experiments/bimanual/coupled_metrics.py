#!/usr/bin/env python3
"""Metrics for the COUPLED-TRANSPORT pair (T3 rigid, T6 compliant) and for
T7 bimanual pursuit. One definition, shared, so the two coupling conditions
are measured identically -- that is the whole point of running them together.

EVERYTHING HERE IS COMPUTED FROM TF AND /joint_states. No vision.

THE T3/T6 CONTRAST
------------------
Rigid coupling transmits any relative error instantly: the tray tilts the
moment the two grippers differ in height. Compliant coupling absorbs small
errors into the sag and then fails NONLINEARLY -- nothing happens, nothing
happens, the ball leaves. The prediction is that divided attention hurts more
under rigid coupling, and the two tasks differ ONLY in the coupling, so the
comparison is clean.

SLING GEOMETRY, and why L = 350 mm is the right length
------------------------------------------------------
A flexible strip of length L with its ends held `s` apart hangs as a V, so

    sag(s) = sqrt((L/2)^2 - (s/2)^2)

and the ball (radius r) is retained while the sag is at least its diameter:

    s_max = 2 * sqrt((L/2)^2 - (2r)^2)      = 340.7 mm for L=350, r=20

MEASURED against the arms' actual band at the T3 mid-waypoint (0.00, 0.35,
1.15), 3 repeats per separation:

    reachable separation   280 - 360 mm
    ball retained          280 - 340 mm
    OVERLAP                280 - 340 mm     nominal 310 mm

L = 350 mm is correct PRECISELY BECAUSE the failure threshold (340 mm) sits
INSIDE the reachable band (280-360). A longer sling would put failure out of
reach and the task could never fail, which would measure nothing. The arms
cannot close below 280 mm, so the lower half of the geometrically-secure
range is unreachable and the operator works near the threshold -- thin
margin, deliberately.
"""
import math

import numpy as np

SLING_L = 0.350
BALL_R = 0.020
TRAY_SPAN = 0.300
SLING_NOMINAL_SEP = 0.310
SEP_MIN_REACHABLE = 0.280
SEP_MAX_REACHABLE = 0.360


def sag(separation_m, length_m=SLING_L):
    """Sling sag for a given gripper separation. V model, matches the
    closed form used to spec the object."""
    v = (length_m / 2.0) ** 2 - (separation_m / 2.0) ** 2
    return math.sqrt(v) if v > 0 else 0.0


def sling_retains(separation_m, length_m=SLING_L, ball_r=BALL_R):
    return sag(separation_m, length_m) >= 2.0 * ball_r


def max_secure_separation(length_m=SLING_L, ball_r=BALL_R):
    v = (length_m / 2.0) ** 2 - (2.0 * ball_r) ** 2
    return 2.0 * math.sqrt(v) if v > 0 else 0.0


def separation_error_mm(p_left, p_right, nominal_m):
    """Signed: positive means the grippers are FURTHER apart than nominal."""
    if p_left is None or p_right is None:
        return float("nan")
    d = float(np.linalg.norm(np.asarray(p_left) - np.asarray(p_right)))
    return 1000.0 * (d - nominal_m)


def height_difference_mm(p_left, p_right):
    """The ball rolls to the LOW side and escapes there, so the sign matters
    and is kept: positive means the LEFT gripper is higher."""
    if p_left is None or p_right is None:
        return float("nan")
    return 1000.0 * (float(p_left[2]) - float(p_right[2]))


def tilt_deg(p_left, p_right, separation_m=TRAY_SPAN):
    """Height-INDEPENDENT: it uses only the two EE heights and the span, so
    moving the task from a 0.885 m table to a 1.15 m stand does not change
    it. What the height changed is whether the PATH is reachable."""
    if p_left is None or p_right is None:
        return float("nan")
    return math.degrees(math.atan2(abs(float(p_left[2]) - float(p_right[2])),
                                   separation_m))


def path_efficiency(points):
    """Straight-line distance / actual path length, in (0, 1].

    1.0 is a perfect straight line. Reported rather than raw path length
    because trials differ in start and end position across scenarios.
    """
    P = np.asarray([p for p in points if p is not None and
                    all(np.isfinite(p))], float)
    if len(P) < 2:
        return float("nan")
    actual = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
    straight = float(np.linalg.norm(P[-1] - P[0]))
    return straight / actual if actual > 1e-9 else float("nan")


def time_above(series, threshold, dt):
    v = np.asarray([x for x in series if x == x], float)
    return float((np.abs(v) > threshold).sum() * dt)


def summarise_transport(t, p_left, p_right, nominal_sep, coupling):
    """One trial -> the row the analysis reads. Same for T3 and T6."""
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.02
    sep = np.array([separation_error_mm(a, b, nominal_sep)
                    for a, b in zip(p_left, p_right)])
    dh = np.array([height_difference_mm(a, b)
                   for a, b in zip(p_left, p_right)])
    tilt = np.array([tilt_deg(a, b, nominal_sep)
                     for a, b in zip(p_left, p_right)])
    mid = [None if (a is None or b is None)
           else list((np.asarray(a) + np.asarray(b)) / 2.0)
           for a, b in zip(p_left, p_right)]
    out = dict(
        coupling=coupling,
        sep_err_rms_mm=float(np.sqrt(np.nanmean(sep ** 2))),
        sep_err_max_mm=float(np.nanmax(np.abs(sep))),
        height_diff_rms_mm=float(np.sqrt(np.nanmean(dh ** 2))),
        height_diff_max_mm=float(np.nanmax(np.abs(dh))),
        tilt_rms_deg=float(np.sqrt(np.nanmean(tilt ** 2))),
        tilt_max_deg=float(np.nanmax(tilt)),
        path_efficiency=path_efficiency(mid),
        duration_s=float(t[-1] - t[0]) if len(t) > 1 else float("nan"))
    if coupling == "compliant":
        # The failure threshold is a SEPARATION, not a tilt.
        s_abs = np.array([np.nan if (a is None or b is None) else
                          float(np.linalg.norm(np.asarray(a) - np.asarray(b)))
                          for a, b in zip(p_left, p_right)])
        smax = max_secure_separation()
        out["time_above_threshold_s"] = float(
            np.nansum(s_abs > smax) * dt)
        out["min_sag_mm"] = float(1000 * min(
            (sag(x) for x in s_abs if x == x), default=float("nan")))
    else:
        # Rigid: the failure threshold is a TILT (the ball rolls off).
        out["time_above_threshold_s"] = time_above(tilt, 11.3, dt)
    return out


# ------------------------------------------------------------------ T7
def tracking_error_mm(p_ee, p_target):
    if p_ee is None or p_target is None:
        return float("nan")
    return 1000.0 * float(np.linalg.norm(np.asarray(p_ee) -
                                         np.asarray(p_target)))


def phase_lag_s(t, ee_axis, target_axis):
    """Cross-correlation lag of the EE against its target, one axis.

    Positive means the EE LAGS. Reported per arm because a lag that grows
    when the other arm speeds up is interference expressed in time rather
    than in amplitude.
    """
    a = np.asarray(ee_axis, float)
    b = np.asarray(target_axis, float)
    m = np.isfinite(a) & np.isfinite(b)
    a, b = a[m] - np.mean(a[m]), b[m] - np.mean(b[m])
    if len(a) < 8:
        return float("nan")
    c = np.correlate(a, b, mode="full")
    lag = int(np.argmax(c)) - (len(a) - 1)
    dt = float(np.median(np.diff(np.asarray(t, float)[m])))
    return float(lag * dt)


def summarise_pursuit(t, ee, target, tol_mm=40.0):
    e = np.array([tracking_error_mm(a, b) for a, b in zip(ee, target)])
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.02
    E = np.asarray(ee, float)
    T = np.asarray(target, float)
    return dict(
        rms_error_mm=float(np.sqrt(np.nanmean(e ** 2))),
        max_error_mm=float(np.nanmax(e)),
        time_on_target_s=float(np.nansum(e <= tol_mm) * dt),
        frac_on_target=float(np.nanmean(e <= tol_mm)),
        phase_lag_s=phase_lag_s(t, E[:, 0], T[:, 0]),
        duration_s=float(t[-1] - t[0]) if len(t) > 1 else float("nan"))


def dual_task_cost(bimanual_err, unimanual_err):
    """THE CONVENTIONAL BIMANUAL-INTERFERENCE MEASURE.

    The motor-control literature quantifies bimanual interference as the
    difference between performance on a task done alone and the same task
    done alongside another. Reporting it this way makes the number
    comparable to that literature instead of being a bespoke statistic.

    Returned as a proportion: 0.0 = no cost, 0.5 = 50% worse when bimanual.
    """
    if not np.isfinite(unimanual_err) or unimanual_err <= 0:
        return float("nan")
    return float((bimanual_err - unimanual_err) / unimanual_err)


def interference_coefficient(rows, arm):
    """CROSS-ARM INTERFERENCE -- the headline for T7.

    WHAT b_cross MEASURES, IN WORDS
    -------------------------------
    b_cross is the **total speed sensitivity of arm A's RMS tracking error to
    arm B's target speed**, in millimetres of RMS error per (metre/second) of
    the other arm's target.

    "Total" is load-bearing. A participant's tracking error has at least two
    components and BOTH grow with speed:

      * an AMPLITUDE component -- the operator simply tracks less accurately
        when attention is divided;
      * a LAG component -- any fixed reaction delay `L` turns into positional
        error `v * L`, because the target has moved that far by the time the
        hand arrives.

    From RMS error alone these are NOT separable, in this experiment or in
    any other: the two produce the same statistic. So b_cross is DEFINED as
    the total, and it must be reported and interpreted as the total. Reading
    it as "the attentional cost" alone would overstate it by whatever the lag
    contributes.

    `phase_lag_s` is reported alongside precisely so the lag component is
    visible separately. If b_cross is large while the phase lag is flat in
    the other arm's speed, the effect is attentional; if both move together,
    it is not.
    

    Fits, over trials,

        err_A  ~  b0 + b1 * speed_A + b2 * speed_B + b3 * speed_A * speed_B

    and returns b2: how much arm A's error rises per unit of the OTHER
    arm's target speed. That is the attention bottleneck measured directly,
    and it is identifiable only because the two speeds are varied
    INDEPENDENTLY across trials -- with the speeds yoked, b1 and b2 are
    collinear and neither can be estimated.
    """
    other = "right" if arm == "left" else "left"
    X, y = [], []
    for r in rows:
        try:
            sa = float(r["speed_%s" % arm])
            sb = float(r["speed_%s" % other])
            e = float(r["rms_error_mm_%s" % arm])
        except (KeyError, TypeError, ValueError):
            continue
        if not np.isfinite(e):
            continue
        X.append([1.0, sa, sb, sa * sb])
        y.append(e)
    if len(y) < 5:
        return dict(n=len(y), b_self=float("nan"), b_cross=float("nan"),
                    b_interaction=float("nan"), r2=float("nan"))
    X = np.asarray(X)
    y = np.asarray(y)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ beta
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return dict(n=len(y), b_self=float(beta[1]), b_cross=float(beta[2]),
                b_interaction=float(beta[3]),
                r2=float(1 - ss_res / ss_tot) if ss_tot > 0 else float("nan"))
