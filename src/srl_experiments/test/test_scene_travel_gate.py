#!/usr/bin/env python3
"""KNOWN ANSWERS FOR THE SWEEP'S SECOND TRAVEL GATE.

WHY THERE IS A SECOND ONE. `run_abc` refuses to exit 0 unless one arm
travelled at least --min-travel-m, and `record_abc_sweep` trusted that exit
code as its only evidence of motion. The two measure the same quantity in
DIFFERENT PROCESSES off different tf2 listeners, so the runner can see motion
the scene node never received. On 2026-08-15 that is what got filed: clips
reporting TRAVEL L 0.00 R 0.00 with four cubes carried 0.000 m, recorded OK.

A stationary arm is this project's oldest failure mode, and the gate against
it lived inside a 300-line loop that needs a stack, an Xvfb and four minutes
to reach -- so it could never be handed a broken input. `scene_travel_verdict`
is that decision pulled out where it can be.

THE GROUND TRUTH IS CONSTRUCTED, not rendered: these are dictionaries written
down in this file, which is the only kind of synthetic input docs/ENGINEERING_LOG.md allows
for an instrument check.

The numbers here are MEASURED, off the 25 clips recorded 2026-08-15, and the
floor is a SEPARATION between two populations rather than a constant: above
the staging move the scene node unavoidably counts (~0.24 m) and above the one
clip that genuinely did not run (0.407 m), below the dimmest clip that did
(1.072 m).

That distinction is the point. The first floor was 0.05 m, set against a
stationary arm reading 0.0000 -- and it would have passed a completely dead
run, because clip_scene starts before the staging move and counts it. A gate
written to catch a silently stationary arm that cannot catch one is the exact
shape of failure this file exists to prevent.
"""

import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))


def _verdict():
    """record_abc_sweep.scene_travel_verdict, or skip saying why.

    The module imports srl_teleop and record_rviz at scope, so on a machine
    without a sourced workspace this skips rather than failing for the wrong
    reason.
    """
    try:
        import record_abc_sweep
    except ImportError as e:                                # pragma: no cover
        pytest.skip("record_abc_sweep needs a sourced workspace (%s)" % e)
    return record_abc_sweep.scene_travel_verdict


# ---- the failure it exists to catch -------------------------------------
def test_the_stationary_arm_that_ended_the_last_session_FAILS():
    """The exact shape of the 2026-08-15 clips. This must be False."""
    ok, why = _verdict()({"ee_travel_m": {"left": 0.0, "right": 0.0}})
    assert ok is False, why
    assert "STATIONARY" in why
    # AND IT NAMES THE NUMBERS. A refusal that does not say what it measured
    # sends the next person back to the file to find out.
    assert "left 0.0000" in why and "right 0.0000" in why


def test_a_crawl_below_the_floor_FAILS_TOO():
    """Not just exactly zero. 30 mm of drift is not a task being performed."""
    ok, _ = _verdict()({"ee_travel_m": {"left": 0.03, "right": 0.021}})
    assert ok is False


def test_THE_STAGING_MOVE_ALONE_DOES_NOT_PASS():
    """THE CASE THE FIRST FLOOR GOT WRONG.

    clip_scene starts BEFORE the presentation-pose move, so its tf integral
    includes it: 0.1903 m of hand travel left and 0.2351 m right. A clip whose
    TASK never moved the arm still reports about that. With the original
    0.05 m floor this passed -- the gate written to catch a silently
    stationary arm could not have caught one.
    """
    ok, why = _verdict()({"ee_travel_m": {"left": 0.1903, "right": 0.2351}})
    assert ok is False, why


def test_THE_ONE_REAL_FAILURE_IN_THE_SET_IS_CAUGHT():
    """04_vr_shared/T0, 2026-08-15: capture gated on timeout, runner exited 1,
    scene read 0.4072 / 0.3297. Two other witnesses called it dead."""
    ok, _ = _verdict()({"ee_travel_m": {"left": 0.4072, "right": 0.3297}})
    assert ok is False


def test_THE_DIMMEST_GOOD_CLIP_STILL_PASSES():
    """01_master_teleop/T2, 1.075 / 1.007 m. The floor must not eat it."""
    ok, _ = _verdict()({"ee_travel_m": {"left": 1.075, "right": 1.007}})
    assert ok is True


# ---- and the good clip it must not fail ---------------------------------
def test_the_measured_good_clip_PASSES():
    ok, why = _verdict()({"ee_travel_m": {"left": 3.0205, "right": 0.6010}})
    assert ok is True, why
    assert "3.0205" in why


def test_ONE_ARM_MOVING_IS_ENOUGH():
    """T1 and T3 work one arm at a time and the idle arm is meant to be still.

    A gate over ALL arms would fail every one-armed task, which is three of
    the five in the set. This is why the rule is max, not min.
    """
    ok, _ = _verdict()({"ee_travel_m": {"left": 3.02, "right": 0.0}})
    assert ok is True


# ---- absent is not zero -------------------------------------------------
def test_a_clip_with_NO_travel_field_is_UNANSWERED_not_failed():
    """by_design.py's rule: a by-design absence must not render as a defect.

    A clip recorded before the field existed says nothing about motion. None
    is "not measured"; False would be "measured, and it did not move".
    """
    ok, why = _verdict()({"items": []})
    assert ok is None
    assert "NOT REPORTED" in why


def test_an_EMPTY_travel_dict_is_also_unanswered():
    ok, _ = _verdict()({"ee_travel_m": {}})
    assert ok is None


def test_travel_that_carries_no_number_is_unanswered():
    """`ee_net_m` writes None for an arm with no track, and the same can
    happen here. None is not a distance and must not be compared to one."""
    ok, why = _verdict()({"ee_travel_m": {"left": None, "right": None}})
    assert ok is None
    assert "no number" in why


def test_one_arm_untracked_and_the_other_moving_still_PASSES():
    ok, _ = _verdict()({"ee_travel_m": {"left": 3.02, "right": None}})
    assert ok is True


def test_one_arm_untracked_and_the_other_STILL_fails():
    ok, _ = _verdict()({"ee_travel_m": {"left": 0.0, "right": None}})
    assert ok is False


# ---- the threshold is a parameter, and the default is the measured one ---
def test_the_floor_is_where_it_says_it_is():
    import record_abc_sweep
    assert record_abc_sweep.MIN_SCENE_TRAVEL_M == 0.60
    # Just under and just over, with the default floor.
    assert _verdict()({"ee_travel_m": {"left": 0.599}})[0] is False
    assert _verdict()({"ee_travel_m": {"left": 0.601}})[0] is True


def test_the_floor_separates_the_two_MEASURED_populations():
    """It is a separation, not a constant: above staging and the one real
    failure, below every clip that did its task."""
    import record_abc_sweep
    f = record_abc_sweep.MIN_SCENE_TRAVEL_M
    assert f > 0.4072, "would pass 04_vr_shared/T0, which did not run"
    assert f < 1.072, "would reject the dimmest clip that did run"
