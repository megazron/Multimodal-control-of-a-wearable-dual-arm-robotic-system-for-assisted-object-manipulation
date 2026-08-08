#!/usr/bin/env python3
"""Pin the bimanual metrics, especially the presence check.

The presence check is the difference between "the participant failed" and
"the object was missing". Without it the second is silently recorded as the
first, and no amount of later analysis can separate them.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "experiments" / "bimanual"))
from bimanual_metrics import (  # noqa: E402
    gripper_state, object_present, tilt_deg, summarise_tilt,
    handover_ordering_ok, separation_mm)


def test_free_air_closure_is_not_a_grasp():
    assert gripper_state(0.79) == "free_air"
    assert not object_present(0.79), \
        "a gripper closed to free air holds NOTHING; calling that a grasp " \
        "blames the participant for a missing object"


def test_holding_is_between_open_and_free_air():
    assert gripper_state(0.62) == "holding"      # 25 mm block
    assert gripper_state(0.38) == "holding"      # 55 mm block
    assert object_present(0.58)                  # 30 mm tray grip block


def test_open_is_open():
    assert gripper_state(0.0) == "open"
    assert gripper_state(0.05) == "open"


def test_tilt_matches_the_documented_thresholds():
    # 20 mm over a 300 mm tray -> 3.8 deg; 60 mm -> 11.3 deg
    assert abs(tilt_deg(1.000, 1.020) - 3.8) < 0.1
    assert abs(tilt_deg(1.000, 1.060) - 11.3) < 0.1
    assert tilt_deg(1.0, 1.0) == 0.0


def test_tilt_is_sign_independent():
    assert tilt_deg(1.02, 1.00) == tilt_deg(1.00, 1.02)


def test_max_is_dominated_by_one_instant_but_rms_reflects_duration():
    """Why RMS is the primary outcome and max is only reported alongside.

    A trial that spiked once to 40 deg and a trial that sat at 4 deg
    throughout differ by 10x on MAX and are indistinguishable on RMS. The
    ball on the tray integrates the trace, so RMS is the honest primary; max
    is kept because a single 40 deg excursion still throws the ball off.
    """
    steady = summarise_tilt([4.0] * 100)
    spike = summarise_tilt([0.0] * 99 + [40.0])
    assert spike["tilt_max_deg"] == 10.0 * steady["tilt_max_deg"]
    assert abs(spike["tilt_rms_deg"] - steady["tilt_rms_deg"]) < 1e-9

    # And a trial that is bad for LONGER must score worse on RMS.
    longer = summarise_tilt([8.0] * 100)
    assert longer["tilt_rms_deg"] > steady["tilt_rms_deg"]
    assert longer["time_above_3_8_s"] > 0


def test_time_above_thresholds():
    s = summarise_tilt([12.0] * 50, dt=0.02)      # 1 s above both
    assert abs(s["time_above_3_8_s"] - 1.0) < 1e-6
    assert abs(s["time_above_11_3_s"] - 1.0) < 1e-6


def test_handover_ordering_rejects_a_caught_drop():
    assert handover_ordering_ok(t_receiver_closed=1.0, t_giver_opened=1.5)
    assert not handover_ordering_ok(t_receiver_closed=1.5, t_giver_opened=1.0), \
        "giver-opens-first is a drop that was caught, not a handover"


def test_separation_detects_a_lost_grip():
    assert abs(separation_mm((-0.15, 0.3, 0.9), (0.15, 0.3, 0.9)) - 300) < 1e-6


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
