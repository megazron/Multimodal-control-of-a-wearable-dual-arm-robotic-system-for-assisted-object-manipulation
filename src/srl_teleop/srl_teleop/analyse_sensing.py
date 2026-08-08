#!/usr/bin/env python3
"""
analyse_sensing.py — PART 3 measurements, against the real recordings.

Two questions, answered from `recordings/*.csv` rather than from argument:

  (a) THE POT REDUCTION LADDER. What does each pot removed actually cost, in
      metres of commanded master-tip error, over a real recorded trajectory?
  (c) THE COMPLEMENTARY FILTER. How accurate is it against a static gravity
      reference, and how does that vary with the time constant?

Run:  python3 -m srl_teleop.analyse_sensing recordings/teleop_*.csv

WHAT IS MEASURED, precisely. In spherical mode the commanded master position
is built as (elevation from the wrist IMU, azimuth from j1, reach magnitude
from |fk(j1..j4)|). A pot reduction therefore does NOT move the direction --
that comes from the IMU -- it degrades the REACH MAGNITUDE. So the cost of a
reduction is measured as the change in the COMMANDED 3-D position, using the
recorded, real IMU elevation and the recorded azimuth, with only the reach
term recomputed from the reduced pot set. Measuring the raw 7-DOF FK position
instead would charge every reduction for direction error the shipped pipeline
never incurs.
"""
import csv
import math
import sys

import numpy as np

from srl_teleop import master_calibration as mc
from srl_teleop.imu_orientation import (ComplementaryFilter, angular_error,
                                        pointing_direction, q_conj, q_rot,
                                        roll_pitch_from_accel)

DEG = 180.0 / math.pi


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def _f(r, k, d=float("nan")):
    try:
        return float(r.get(k, ""))
    except (TypeError, ValueError):
        return d


def reach_of(j, keep):
    """Reach magnitude from a reduced pot set; dropped channels pass as 0."""
    jj = [j[i] if i in keep else 0.0 for i in range(7)]
    return float(np.linalg.norm(mc.fk(jj)))


# Each rung: label, pot indices still load-bearing, how the gap is filled.
LADDER = [
    ("R0  j1 j2 j3 j4   SHIPPED spherical", (0, 1, 2, 3), "reference"),
    ("R1  j1 j2 j4      drop j3", (0, 1, 3), "j3 passed as 0"),
    ("R2  j1 j2         drop j3, j4", (0, 1), "elbow lost"),
    ("R3  j1 j4         drop j2, j3", (0, 3), "shoulder lost"),
    ("R4  j1 + 2nd IMU  elbow angle from two IMUs, no j2/j3/j4", (0,), "imu_elbow"),
    ("R5  no pots       IMU only", (), "position unavailable"),
]


def ladder(rows, arm, elbow_sigma_deg=1.2):
    """Commanded-position error for each rung, vs the shipped 4-pot reach."""
    rng = np.random.default_rng(0)
    errs = {name: [] for name, _, _ in LADDER}
    reach_err = {name: [] for name, _, _ in LADDER}
    n = 0
    for r in rows:
        j = [_f(r, f"{arm}_j{i+1}") for i in range(7)]
        elev = _f(r, f"{arm}_elev")
        azim = _f(r, f"{arm}_azim")
        if any(x != x for x in j) or elev != elev or azim != azim:
            continue
        if _f(r, f"{arm}_valid", 1.0) < 0.5:
            continue
        n += 1
        ref = reach_of(j, (0, 1, 2, 3))
        u_ref = pointing_direction(math.radians(elev), math.radians(azim))
        p_ref = ref * u_ref
        for name, keep, how in LADDER:
            if how == "position unavailable":
                continue
            if how == "imu_elbow":
                # A second IMU measures the elbow angle directly. It replaces
                # j4 up to its own error, modelled at the filter's measured
                # static accuracy; j2 and j3 are gone, so the shoulder bend is
                # not recovered.
                jj = list(j)
                jj[3] = j[3] + rng.normal(0.0, elbow_sigma_deg)
                rr = float(np.linalg.norm(mc.fk([jj[0], 0.0, 0.0, jj[3], 0, 0, 0])))
            else:
                rr = reach_of(j, keep)
            u = u_ref if 0 in keep else np.array([0.0, 1.0, 0.0])
            errs[name].append(float(np.linalg.norm(rr * u - p_ref)))
            reach_err[name].append(abs(rr - ref))
    out = []
    for name, keep, how in LADDER:
        e = np.array(errs[name])
        if not e.size:
            out.append((name, len(keep), how, None, None, None, None))
            continue
        d = np.array(reach_err[name])
        out.append((name, len(keep), how, float(e.mean()), float(np.percentile(e, 95)),
                    float(e.max()), float(d.mean())))
    return out, n


def filter_accuracy(rows, arm, taus=(0.2, 0.5, 1.0, 2.0, 5.0),
                    gyro_std_dps=1.57, gyro_bias_dps=-0.075, seed=0):
    """TILT error against a static gravity reference, in degrees.

    The metric is the angle between the filter's predicted DOWN direction and
    the measured one. That is rotation-representation-free: immune to Euler
    wrapping and to yaw, which gravity cannot observe anyway. (An earlier
    version differenced Euler roll/pitch and reported ~200 deg of "error"
    purely from atan2 wrapping near +-pi with the arm hanging.)

    The recordings predate the gyro plumbing, so they carry accel but no gyro.
    The gyro stream is SYNTHESISED from the frame-to-frame change in the
    accel-derived attitude plus this arm's MEASURED bias and noise. The noise
    is real and the motion is real; only the pairing is synthetic. Stated in
    the output, not buried here.
    """
    rng = np.random.default_rng(seed)
    a = []
    for r in rows:
        ax, ay, az = (_f(r, f"{arm}_a{c}") for c in "xyz")
        if any(v != v for v in (ax, ay, az)):
            continue
        a.append((_f(r, "t", 0.0), ax, ay, az))
    if len(a) < 200:
        return None
    a = np.array(a)
    t, acc = a[:, 0], a[:, 1:4]
    rp = np.array([roll_pitch_from_accel(*v) for v in acc])
    mag = np.linalg.norm(acc, axis=1)
    d = np.r_[0.0, np.linalg.norm(np.diff(rp, axis=0), axis=1)]
    static = (np.abs(mag - 1.0) < 0.02) & (d < math.radians(0.2))

    dt = np.r_[0.02, np.diff(t)]
    dt[(dt <= 0) | (dt > 0.5)] = 0.02
    w = np.zeros((len(t), 3))
    w[1:, 0] = np.diff(rp[:, 0]) / dt[1:]
    w[1:, 1] = np.diff(rp[:, 1]) / dt[1:]
    w += rng.normal(0.0, math.radians(gyro_std_dps), w.shape)
    w += math.radians(gyro_bias_dps)

    res = []
    for tau in taus:
        f = ComplementaryFilter(tau)
        err = []
        for i in range(len(t)):
            q = f.update(acc[i], w[i], dt[i])
            if not static[i] or q is None:
                continue
            g_pred = q_rot(q_conj(q), np.array([0.0, 0.0, 1.0]))
            m = float(np.linalg.norm(acc[i]))
            if m < 1e-6:
                continue
            err.append(angular_error(g_pred, acc[i] / m))
        if err:
            e = np.array(err) * DEG
            res.append((tau, float(e.mean()), float(np.percentile(e, 95)),
                        float(e.max()), f.gated_fraction, int(static.sum())))
    return res


def accel_sampling_check(rows, arm):
    """Can the accelerometer's own noise be measured from these recordings?

    NO — and the reason is worth recording. During rest the accel triple
    repeats BIT-IDENTICALLY between consecutive rows, because the 50 Hz
    recorder oversamples a sensor stream that updates more slowly. Any
    "within-static-run noise" computed from this data is therefore exactly
    0.000 deg by construction, which is a property of the sampling, not of
    the sensor. It is the same zero-variance signature that identified the
    dead j7 pot and the frozen /real/joint_states.

    Reported as a repeat fraction so the artefact is visible instead of being
    mistaken for a very good sensor.
    """
    v = []
    for r in rows:
        ax, ay, az = (_f(r, f"{arm}_a{c}") for c in "xyz")
        if any(x != x for x in (ax, ay, az)):
            continue
        v.append((ax, ay, az))
    if len(v) < 200:
        return None
    a = np.array(v)
    same = np.all(np.diff(a, axis=0) == 0.0, axis=1)
    mag = np.linalg.norm(a, axis=1)
    return float(same.mean()), int(len(a)), float(np.abs(mag - 1.0).mean())


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        return
    for p in paths:
        rows = load(p)
        print("=" * 80)
        print("%s   %d rows" % (p, len(rows)))
        for arm in ("left", "right"):
            if not rows or f"{arm}_j1" not in rows[0]:
                continue
            print("-" * 80)
            print("%s arm" % arm.upper())
            res, n = ladder(rows, arm)
            print("  (a) POT REDUCTION LADDER — commanded master-tip error vs the")
            print("      shipped 4-pot reach, over %d valid frames." % n)
            print("      %-46s %4s %9s %9s %9s %9s"
                  % ("rung", "pots", "mean_m", "p95_m", "max_m", "d|reach|"))
            for name, k, how, mean, p95, mx, dr in res:
                if mean is None:
                    print("      %-46s %4d   %s" % (name, k, how.upper()))
                else:
                    print("      %-46s %4d %9.4f %9.4f %9.4f %9.4f"
                          % (name, k, mean, p95, mx, dr))
            b = accel_sampling_check(rows, arm)
            if b:
                print("  (c) accel sampling: %.1f%% of consecutive rows repeat the"
                      " accel triple BIT-IDENTICALLY (%d rows)." % (100 * b[0], b[1]))
                print("      The 50 Hz recorder oversamples the sensor, so sensor"
                      " noise CANNOT be measured from")
                print("      this data - any within-run figure is 0.000 deg by"
                      " construction. mean| |a|-1g | = %.4f g" % b[2])
            fa = filter_accuracy(rows, arm,
                                 gyro_std_dps=1.57 if arm == "left" else 9.74,
                                 gyro_bias_dps=-0.075 if arm == "left" else 0.574)
            if fa:
                print("      COMPLEMENTARY FILTER tilt error vs the same static "
                      "reference;")
                print("      gyro SYNTHESISED with this arm's MEASURED bias/noise "
                      "(%s)" % ("-0.075 +/- 1.57 deg/s" if arm == "left"
                                else "+0.574 +/- 9.74 deg/s"))
                print("      %6s %10s %10s %10s %8s %9s"
                      % ("tau_s", "mean_deg", "p95_deg", "max_deg", "gated", "n_static"))
                for tau, mean, p95, mx, g, ns in fa:
                    print("      %6.2f %10.4f %10.4f %10.4f %8.3f %9d"
                          % (tau, mean, p95, mx, g, ns))


if __name__ == "__main__":
    main()
