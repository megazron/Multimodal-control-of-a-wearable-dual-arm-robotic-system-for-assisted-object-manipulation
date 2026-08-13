#!/usr/bin/env python3
"""KNOWN ANSWERS FOR THE T2 CARRY SERIES.

TASK_SPEC.md T2-3 and T2-4: tilt and separation must be logged CONTINUOUSLY
through the carry, not scored pass/fail at the end. `clip_scene` now samples
both every tick and `_carry_summary()` reduces the series.

The ground truth here is CONSTRUCTED, not rendered -- it is trigonometry over
numbers written down in this file -- which is the only kind of synthetic input
CLAUDE.md allows for an instrument check.

THE CASE THAT MATTERS is the last one: a tray that swings past the failure
tilt in the middle of the carry and comes back level. Every end-state check
scores it identically to a tray that never moved, and telling those two apart
is the entire reason the series exists.
"""

import math
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments",
                                "experiments", "abc"))

import tasks as TSK                                            # noqa: E402

THR = TSK.TASK_B["fail_tilt_deg"]


def _cs():
    """_carry_summary, imported without rclpy.

    clip_scene imports rclpy at module scope, so on a machine without a
    sourced ROS this test skips rather than failing for the wrong reason.
    """
    try:
        import clip_scene
    except ImportError as e:                                   # pragma: no cover
        pytest.skip("clip_scene needs a sourced ROS 2 (%s)" % e)
    return clip_scene._carry_summary


def _row(t, tilt_deg, sep_m):
    return dict(t=t, tilt_deg=tilt_deg, sep_m=sep_m,
                sep_err_mm=(sep_m - TSK.TRAY_SEP) * 1000.0,
                height_diff_mm=math.sin(math.radians(tilt_deg)) * sep_m * 1000.0)


def test_no_carry_is_none_not_zero():
    """A task with no carry returns None.

    Zero would say "measured, and it was level". None says "not measured".
    Rendering those two the same is the by_design.py failure.
    """
    assert _cs()([]) is None


def test_level_carry():
    s = _cs()([_row(i * 0.1, 0.0, TSK.TRAY_SEP) for i in range(21)])
    assert s["samples"] == 21
    assert s["tilt_rms_deg"] == 0.0
    assert s["tilt_max_deg"] == 0.0
    assert s["time_above_fail_tilt_s"] == 0.0
    assert s["sep_err_max_mm"] == 0.0


def test_tilt_arithmetic_matches_the_spec_threshold():
    """60 mm of height difference over the 500 mm span IS the threshold.

    Pinning this stops the threshold quietly becoming an angle over some
    other baseline, which tasks.py records as a real past error: feeding the
    sling's 310 mm separation into the rigid tray's tilt is wrong by 5/3.

    WHICH TRIANGLE, stated rather than assumed. The tray is RIGID, so the
    500 mm grip separation is the HYPOTENUSE and the tilt is asin(dz / span)
    -- that is what clip_scene samples. The spec's "6.8 deg" is the same
    60 mm read as atan(dz / 500), 6.843 deg. The two differ by 0.05 deg,
    which is far below anything this rig can resolve, but the convention is
    pinned here so a future edit cannot swap them and move the threshold.
    """
    span = math.degrees(math.asin(0.060 / TSK.TRAY_SEP))       # 6.892
    flat = math.degrees(math.atan(0.060 / 0.500))              # 6.843
    assert TSK.TRAY_SEP == 0.500
    assert abs(flat - THR) < 0.05
    assert abs(span - flat) < 0.06


def test_constant_tilt_above_threshold_is_timed_not_counted():
    """Time above the threshold is integrated from the sample spacing.

    Counted in SAMPLES it would depend on the tick rate, so a slower run
    would report a shorter excursion for the same motion.
    """
    rows = [_row(i * 0.25, THR + 2.0, TSK.TRAY_SEP) for i in range(9)]
    s = _cs()(rows)
    assert s["span_s"] == 2.0
    # 8 intervals of 0.25 s; the first sample opens the window and carries no
    # elapsed time of its own.
    assert abs(s["time_above_fail_tilt_s"] - 2.0) < 1e-6
    assert abs(s["tilt_max_deg"] - (THR + 2.0)) < 1e-6


def test_an_excursion_that_recovers_is_visible():
    """THE CASE THE END-STATE CHECK CANNOT SEE.

    Both trays are level at the first and last sample and both keep the ball.
    One of them swings to 9 deg on the way. A pass/fail at the end scores
    them the same; the series does not.
    """
    steady = [_row(i * 0.1, 0.0, TSK.TRAY_SEP) for i in range(41)]
    swung = list(steady)
    for i in range(15, 26):
        swung[i] = _row(i * 0.1, 9.0, TSK.TRAY_SEP)

    a, b = _cs()(steady), _cs()(swung)
    assert a["tilt_max_deg"] == 0.0 and b["tilt_max_deg"] == 9.0
    assert a["time_above_fail_tilt_s"] == 0.0
    assert b["time_above_fail_tilt_s"] > 0.9
    assert b["tilt_rms_deg"] > a["tilt_rms_deg"]
    # and the thing that makes the test worth having: the end states agree
    assert steady[-1]["tilt_deg"] == swung[-1]["tilt_deg"]


def test_separation_error_keeps_its_sign():
    """The arms drifting APART and TOGETHER are different failures.

    Taking max() of the absolute value would report both as the same number
    and lose which one happened.
    """
    pulled = _cs()([_row(i * 0.1, 0.0, TSK.TRAY_SEP + 0.020)
                    for i in range(11)])
    squeezed = _cs()([_row(i * 0.1, 0.0, TSK.TRAY_SEP - 0.020)
                      for i in range(11)])
    assert pulled["sep_err_max_mm"] == 20.0
    assert squeezed["sep_err_max_mm"] == -20.0
    assert pulled["sep_err_rms_mm"] == squeezed["sep_err_rms_mm"] == 20.0
