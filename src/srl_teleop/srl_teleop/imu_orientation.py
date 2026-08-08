#!/usr/bin/env python3
"""
imu_orientation.py — orientation from the master arm's IMUs.

WHAT AN IMU CAN AND CANNOT DO. These are physical limits, not bugs, and every
design decision below follows from them:

  accelerometer -> ROLL and PITCH, absolute and drift-free. NOT YAW. Gravity
      points the same way regardless of heading, so a rotation about vertical
      leaves the accelerometer reading unchanged. No amount of filtering
      recovers it; there is no magnetometer on this board.
  gyroscope     -> rotation RATE about all three axes, including yaw. Good
      over seconds, drifts over minutes. Measured on this rig: -5.0 deg/min
      (left) and +44.0 deg/min (right, harness fault).
  double integration of acceleration -> POSITION. UNUSABLE. A constant bias b
      grows as 0.5*b*t^2; the measured noise gives ~0.05 m of drift in 1 s
      against a 0.272 m master workspace. This module deliberately provides no
      function that integrates acceleration twice, so it cannot be done by
      accident.
  potentiometers -> the ONLY source of absolute position on this rig.

So: the IMU supplies ORIENTATION and POINTING DIRECTION. The pots supply
POSITION. Autonomy supplies the wrist orientation the master cannot measure.
"""
import math

import numpy as np

GRAVITY_G = 1.0          # the board reports acceleration in g


# ---------------------------------------------------------------- quaternions
def q_mul(a, b):
    """Hamilton product, (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q):
    n = float(np.linalg.norm(q))
    return np.array([1.0, 0, 0, 0]) if n < 1e-12 else np.asarray(q, float) / n


def q_rot(q, v):
    """Rotate v by q (sensor frame -> world frame for a body-to-world q)."""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R @ np.asarray(v, float)


def q_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_q(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


# ------------------------------------------------------------ accel-only tilt
def roll_pitch_from_accel(ax, ay, az):
    """Absolute roll and pitch from gravity. Drift-free. Yaw is unobservable."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch


def elevation_from_accel(a_hat, accel):
    """Arm elevation about the shoulder, from the wrist IMU.

    `a_hat` is the arm's long axis in the SENSOR frame, captured once with the
    arm hanging down. Mount-independent, drift-free, no integration.

    SIGN CONVENTION — do not "fix" it: hanging down = -90 deg, horizontal = 0,
    straight up = +90. The original brief specified both a_hat = -normalize(
    accel_rest) AND elevation = asin(-dot(...)); those compound and put the
    resting arm at +90, flipping z alone. Only one negation is correct.
    """
    a = np.asarray(accel, float)
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        return None
    return math.asin(max(-1.0, min(1.0, float(np.dot(a_hat, a / n)))))


# ----------------------------------------------------- complementary filter
class ComplementaryFilter:
    """Gyro integration corrected toward gravity, with a tunable time constant.

    Structure (Mahony form): the gyro is integrated to predict attitude, and
    the accelerometer pulls the predicted DOWN direction toward the measured
    one. The correction acts only in the tilt plane, so it can never touch
    yaw — which is correct, because gravity carries no yaw information. Yaw is
    therefore pure gyro and is reported as drifting; `yaw_drift_rad` tracks how
    much has accumulated since the last reset so a consumer can discount it.

    tau_s is the crossover: below it the gyro dominates (good for fast motion,
    immune to the linear-acceleration error that corrupts accel tilt), above it
    the accelerometer dominates (bounds drift). 0.5-2 s is the useful range.

    ACCELERATION GATE. When |accel| leaves a band around 1 g the reading is
    contaminated by the arm's own acceleration and is NOT gravity. The
    correction is suspended and integration continues on the gyro alone —
    that fast motion is exactly what the gyro is for.
    """

    def __init__(self, tau_s=1.0, accel_gate_g=0.15, bias=(0.0, 0.0, 0.0)):
        self.tau = float(tau_s)
        self.gate = float(accel_gate_g)
        self.bias = np.asarray(bias, float)
        self.q = None                # body -> world
        self.yaw_drift = 0.0
        self.n_gated = 0
        self.n_total = 0

    def reset(self, q=None):
        self.q = None if q is None else q_norm(q)
        self.yaw_drift = 0.0

    def _seed_from_accel(self, accel):
        r, p = roll_pitch_from_accel(*accel)
        self.q = rpy_to_q(r, p, 0.0)

    def update(self, accel, gyro_rad_s, dt):
        """accel in g (sensor frame), gyro in rad/s (sensor frame)."""
        self.n_total += 1
        accel = np.asarray(accel, float)
        w = np.asarray(gyro_rad_s, float) - self.bias
        if self.q is None:
            self._seed_from_accel(accel)
            return self.q
        if dt <= 0.0 or dt > 0.5:        # a stalled or absurd frame gap
            return self.q

        # --- predict on the gyro
        wn = float(np.linalg.norm(w))
        if wn > 1e-9:
            ang = wn * dt
            axis = w / wn
            dq = np.concatenate([[math.cos(ang / 2)], axis * math.sin(ang / 2)])
            self.q = q_norm(q_mul(self.q, dq))
        self.yaw_drift += float(w[2]) * dt

        # --- correct toward gravity, tilt only
        mag = float(np.linalg.norm(accel))
        if abs(mag - GRAVITY_G) > self.gate or mag < 1e-6:
            self.n_gated += 1
            return self.q
        # An accelerometer at rest measures PROPER acceleration, so it reads
        # +1 g along the UP axis, not down. v_meas is therefore world-UP
        # expressed in the sensor frame, and so is v_pred.
        v_meas = accel / mag
        v_pred = q_rot(q_conj(self.q), np.array([0.0, 0.0, 1.0]))
        # SIGN. The correction is applied as a body-frame rotation, q <- q*dq,
        # which re-expresses world-up in the NEW body frame as R(dq)^T v_pred.
        # To drive that toward v_meas the rotation must carry v_meas onto
        # v_pred, i.e. cross(v_meas, v_pred) -- not the other way round.
        # Getting it backwards does not merely converge slowly: it drives the
        # estimate to the ANTIPODAL attitude and the tilt error sits at a
        # suspiciously stable 176-180 deg, which is how this was caught.
        err = np.cross(v_meas, v_pred)
        k = dt / (self.tau + dt)                   # the complementary weight
        half = 0.5 * k * err
        dq = q_norm(np.concatenate([[1.0], half]))
        self.q = q_norm(q_mul(self.q, dq))
        return self.q

    @property
    def gated_fraction(self):
        return self.n_gated / self.n_total if self.n_total else 0.0


# ------------------------------------------------------- pointing direction
def pointing_direction(elevation_rad, azimuth_rad):
    """Unit vector the operator's forearm points along, in the WEARER frame.

    Wearer frame here matches human_backpack.xacro: +x lateral (wearer's
    right), +y forward, +z up.

    Elevation comes from gravity and is absolute. Azimuth cannot come from
    gravity — see the module docstring — so the caller supplies it, normally
    from j1 (a directly measured, zero-referenced pot) optionally gyro-aided.
    Keeping the two separate is deliberate: a consumer can trust the elevation
    unconditionally and discount the azimuth by its own confidence.
    """
    c = math.cos(elevation_rad)
    return np.array([c * math.sin(azimuth_rad),
                     c * math.cos(azimuth_rad),
                     math.sin(elevation_rad)])


def angular_error(u, v):
    """Angle between two directions, radians."""
    u = np.asarray(u, float)
    v = np.asarray(v, float)
    nu, nv = np.linalg.norm(u), np.linalg.norm(v)
    if nu < 1e-12 or nv < 1e-12:
        return float('nan')
    return math.acos(max(-1.0, min(1.0, float(np.dot(u, v) / (nu * nv)))))


# ------------------------------------------------------------- two-IMU elbow
def elbow_angle_from_two(q_upper, q_fore):
    """Elbow flexion from two segment orientations. NO POTENTIOMETER NEEDED.

    Each segment's orientation comes from its own gravity vector, so each is
    absolute in roll and pitch. The elbow angle is the angle between the two
    segments' long axes, and because it is a RELATIVE quantity the shared yaw
    ambiguity cancels: rotating both segments about vertical leaves it
    unchanged. That is why two IMUs can replace the elbow pot but one cannot.

    Assumes each sensor's +x is along its segment. Re-map before calling if a
    mount differs; capture_zero writes the per-sensor axis.
    """
    ax_u = q_rot(q_upper, np.array([1.0, 0, 0]))
    ax_f = q_rot(q_fore, np.array([1.0, 0, 0]))
    return angular_error(ax_u, ax_f)


def elbow_angle_from_accels(accel_upper, accel_fore, axis=(1.0, 0.0, 0.0)):
    """Elbow flexion from two raw gravity vectors, with no filter at all.

    Cheapest possible form and completely drift-free, but it degenerates when
    both segments are near-vertical (the two gravity vectors become parallel
    and the angle between the segment axes is no longer determined). The
    filtered form above does not degenerate because the gyro carries it
    through. Provided for the static-accuracy check.
    """
    ru, pu = roll_pitch_from_accel(*accel_upper)
    rf, pf = roll_pitch_from_accel(*accel_fore)
    qu = rpy_to_q(ru, pu, 0.0)
    qf = rpy_to_q(rf, pf, 0.0)
    return angular_error(q_rot(qu, axis), q_rot(qf, axis))
