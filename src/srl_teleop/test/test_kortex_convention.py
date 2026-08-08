#!/usr/bin/env python3
"""
test_kortex_convention.py — the seam is where this goes wrong, so test the seam.

A convention bug here commands a near-full revolution on a real arm standing
next to a person. These tests use the ACTUAL legacy home values from the real
arms, not synthetic ones.
"""
import math

from srl_teleop.kortex_convention import (
    kortex_deg_to_ros_rad, ros_rad_to_kortex_deg, kortex_list_to_ros,
    ros_list_to_kortex, shortest_delta_deg, shortest_delta_rad,
    pose_delta_rad, wrap_deg_180, wrap_rad_pi,
)

# Legacy real-robot homes, Kortex convention, degrees 0-360.
REAL_LEFT = [259.03, 277.69, 267.74, 286.14, 194.10, 27.48, 55.26]
REAL_RIGHT = [303.65, 77.06, 98.57, 58.57, 317.14, 36.39, 154.71]
# Expected wrapped equivalents, degrees.
REAL_LEFT_WRAPPED = [-100.97, -82.31, -92.26, -73.86, -165.90, 27.48, 55.26]
REAL_RIGHT_WRAPPED = [-56.35, 77.06, 98.57, 58.57, -42.86, 36.39, 154.71]


def test_left_home_wraps_to_the_stated_values():
    got = [math.degrees(kortex_deg_to_ros_rad(d)) for d in REAL_LEFT]
    for g, e in zip(got, REAL_LEFT_WRAPPED):
        assert abs(g - e) < 1e-6, (got, REAL_LEFT_WRAPPED)


def test_right_home_wraps_to_the_stated_values():
    got = [math.degrees(kortex_deg_to_ros_rad(d)) for d in REAL_RIGHT]
    for g, e in zip(got, REAL_RIGHT_WRAPPED):
        assert abs(g - e) < 1e-6, (got, REAL_RIGHT_WRAPPED)


def test_round_trip_is_exact_for_both_real_homes():
    for pose in (REAL_LEFT, REAL_RIGHT):
        back = ros_list_to_kortex(kortex_list_to_ros(pose))
        for b, o in zip(back, pose):
            assert abs(b - o) < 1e-9, (back, pose)


def test_seam_values():
    """0 / 180 / 360 and just either side. This is where bugs live."""
    cases = [
        (0.0, 0.0), (90.0, 90.0), (180.0, 180.0),
        (180.001, -179.999), (270.0, -90.0),
        (359.999, -0.001), (360.0, 0.0),
        (-0.001, -0.001), (-90.0, -90.0),
    ]
    for kortex, expect_deg in cases:
        got = math.degrees(kortex_deg_to_ros_rad(kortex))
        assert abs(got - expect_deg) < 1e-6, (kortex, got, expect_deg)


def test_180_maps_to_positive_pi_not_negative():
    """(-pi, pi] is half-open at the bottom: 180 must be +180, not -180."""
    assert abs(math.degrees(kortex_deg_to_ros_rad(180.0)) - 180.0) < 1e-9
    assert abs(wrap_deg_180(-180.0) - 180.0) < 1e-9
    assert abs(wrap_rad_pi(-math.pi) - math.pi) < 1e-12


def test_ros_to_kortex_folds_negatives():
    assert abs(ros_rad_to_kortex_deg(math.radians(-10.0)) - 350.0) < 1e-9
    assert abs(ros_rad_to_kortex_deg(math.radians(-100.97)) - 259.03) < 1e-9
    assert abs(ros_rad_to_kortex_deg(0.0) - 0.0) < 1e-9


def test_shortest_delta_never_takes_the_long_way():
    """359 -> 1 is +2 deg, NOT -358. This is the homing safety property."""
    assert abs(shortest_delta_deg(1.0, 359.0) - 2.0) < 1e-9
    assert abs(shortest_delta_deg(359.0, 1.0) - (-2.0)) < 1e-9
    assert abs(shortest_delta_deg(-179.0, 179.0) - 2.0) < 1e-9
    assert abs(shortest_delta_deg(179.0, -179.0) - (-2.0)) < 1e-9
    for a, b in ((0, 350), (350, 0), (10, 200), (200, 10)):
        assert abs(shortest_delta_deg(a, b)) <= 180.0 + 1e-9


def test_shortest_delta_rad_bounded():
    for t in (-10.0, -3.5, 0.0, 3.5, 10.0):
        for c in (-10.0, -3.5, 0.0, 3.5, 10.0):
            assert abs(shortest_delta_rad(t, c)) <= math.pi + 1e-12


def test_limited_joints_are_not_wrapped():
    """Only 1/3/5/7 may take the short way. Wrapping a LIMITED joint would
    mean driving through a hard stop."""
    target = [math.radians(179.0)] * 7
    current = [math.radians(-179.0)] * 7
    d = pose_delta_rad(target, current)
    for i in range(7):
        if i in (0, 2, 4, 6):
            assert abs(abs(math.degrees(d[i])) - 2.0) < 1e-6, i  # short way
        else:
            assert abs(abs(math.degrees(d[i])) - 358.0) < 1e-6, i  # honest, refuse later


def test_legacy_left_home_is_within_the_refusal_limit():
    """LEFT is 109.03 deg out on joint_1 -- large, but INSIDE the 120 deg
    refusal, so homing the left arm from the legacy pose is permitted."""
    sim = [150, -94, -53, -65, -123, 41, 121]
    d = [abs(shortest_delta_deg(s, r)) for s, r in zip(sim, REAL_LEFT_WRAPPED)]
    assert 100.0 < max(d) < 120.0, d
    assert abs(max(d) - 109.03) < 0.01, d


def test_legacy_right_home_exceeds_the_refusal_limit():
    """RIGHT is 179.64 deg out on joint_2 -- a LIMITED joint -- so homing the
    right arm from the legacy pose MUST be refused."""
    sim = [28.9, -103.3, -107.3, 62.0, -78.9, 53.6, 66.4]
    d = [abs(shortest_delta_deg(s, r)) for s, r in zip(sim, REAL_RIGHT_WRAPPED)]
    assert max(d) > 120.0, d
    assert d.index(max(d)) == 1, d        # joint_2, and it is NOT continuous


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print("PASS %s" % name)
            except AssertionError as e:
                fails += 1; print("FAIL %s: %s" % (name, e))
    raise SystemExit(1 if fails else 0)
