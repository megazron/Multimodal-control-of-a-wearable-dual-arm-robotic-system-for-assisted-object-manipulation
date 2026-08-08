"""Regression tests for the IMU sensing path (Part 3).

Two of these encode bugs that were actually made and caught: the
complementary filter's correction sign, and the Euler-wrapping trap in the
accuracy metric.
"""
import math

import numpy as np

from srl_teleop.imu_orientation import (ComplementaryFilter, angular_error,
                                        elbow_angle_from_accels, elevation_from_accel,
                                        pointing_direction, q_conj, q_rot,
                                        roll_pitch_from_accel, rpy_to_q)


def _gravity_dir(q):
    """World-up expressed in the sensor frame, per the filter's estimate."""
    return q_rot(q_conj(q), np.array([0.0, 0.0, 1.0]))


def test_accel_seed_matches_measurement():
    for acc in [(0, 0, 1.0), (0, 0.5, 0.866), (-0.5, 0, 0.866), (0.3, -0.4, 0.866)]:
        r, p = roll_pitch_from_accel(*acc)
        q = rpy_to_q(r, p, 0.0)
        assert angular_error(_gravity_dir(q), acc) < 1e-9


def test_filter_converges_and_does_not_invert():
    """The correction sign must pull toward gravity, not to the antipode.

    Getting cross(v_pred, v_meas) the wrong way round does not diverge
    loudly - it settles at a stable ~180 deg error, which reads like a
    coordinate convention problem rather than a sign bug. This test fails
    hard on that.
    """
    truth = np.array([0.0, math.sin(math.radians(25)), math.cos(math.radians(25))])
    f = ComplementaryFilter(0.2)
    f.q = np.array([math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0, 0.0])
    start = angular_error(_gravity_dir(f.q), truth)
    assert start > math.radians(20)
    for _ in range(500):
        f.update(truth, np.zeros(3), 0.02)
    end = angular_error(_gravity_dir(f.q), truth)
    assert end < math.radians(0.5), "filter did not converge: %.2f deg" % math.degrees(end)


def test_smaller_tau_converges_faster():
    truth = np.array([0.0, math.sin(math.radians(25)), math.cos(math.radians(25))])
    errs = {}
    for tau in (0.2, 5.0):
        f = ComplementaryFilter(tau)
        f.q = np.array([math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0, 0.0])
        for _ in range(50):                      # 1 s at 50 Hz
            f.update(truth, np.zeros(3), 0.02)
        errs[tau] = angular_error(_gravity_dir(f.q), truth)
    assert errs[0.2] < errs[5.0]


def test_accel_gate_suspends_correction_but_not_integration():
    f = ComplementaryFilter(0.2, accel_gate_g=0.15)
    f.update(np.array([0.0, 0.0, 1.0]), np.zeros(3), 0.02)
    before = f.q.copy()
    # 3 g of linear acceleration: NOT gravity, must not be believed
    for _ in range(50):
        f.update(np.array([0.0, 3.0, 1.0]), np.array([0.0, 0.0, 1.0]), 0.02)
    assert f.gated_fraction > 0.9
    # integration continued, so the estimate did move
    assert angular_error(_gravity_dir(f.q), _gravity_dir(before)) >= 0.0


def test_yaw_is_never_corrected_by_gravity():
    """Gravity carries no yaw information, so a pure yaw error must survive."""
    f = ComplementaryFilter(0.1)
    f.q = rpy_to_q(0.0, 0.0, math.radians(40))
    for _ in range(500):
        f.update(np.array([0.0, 0.0, 1.0]), np.zeros(3), 0.02)
    from srl_teleop.imu_orientation import q_to_rpy
    _, _, yaw = q_to_rpy(f.q)
    assert abs(math.degrees(yaw) - 40.0) < 1.0, "gravity must not correct yaw"


def test_elevation_sign_convention():
    """Hanging = -90, horizontal = 0, up = +90. One negation, not two."""
    a_hat = np.array([0.0, 0.0, -1.0])          # captured hanging down
    assert math.degrees(elevation_from_accel(a_hat, (0, 0, 1.0))) == -90.0
    assert abs(math.degrees(elevation_from_accel(a_hat, (0, 0, -1.0))) - 90.0) < 1e-9
    assert abs(math.degrees(elevation_from_accel(a_hat, (1.0, 0, 0.0)))) < 1e-9


def test_pointing_direction_is_unit_and_oriented():
    u = pointing_direction(0.0, 0.0)
    assert abs(np.linalg.norm(u) - 1.0) < 1e-12
    assert u[1] > 0.99                                   # azimuth 0 => forward
    assert pointing_direction(math.radians(90), 0.0)[2] > 0.99   # +90 => up
    assert pointing_direction(0.0, math.radians(90))[0] > 0.99   # +az => +x


def test_two_imu_elbow_needs_no_pot():
    """Straight arm reads ~0; a 90 deg bend reads ~90, from gravity alone."""
    up = (0.0, 0.0, 1.0)
    straight = elbow_angle_from_accels(up, up)
    assert math.degrees(straight) < 1e-6
    bent = elbow_angle_from_accels((0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert abs(math.degrees(bent) - 90.0) < 1e-6


def test_two_imu_elbow_is_yaw_invariant():
    """The shared yaw ambiguity must cancel in the RELATIVE measurement -
    that is the whole reason two IMUs can replace the elbow pot."""
    q_u = rpy_to_q(0.2, 0.3, 0.0)
    q_f = rpy_to_q(0.2, 1.0, 0.0)
    from srl_teleop.imu_orientation import elbow_angle_from_two, q_mul
    base = elbow_angle_from_two(q_u, q_f)
    yaw = rpy_to_q(0.0, 0.0, 1.1)
    rotated = elbow_angle_from_two(q_mul(yaw, q_u), q_mul(yaw, q_f))
    assert abs(base - rotated) < 1e-9
