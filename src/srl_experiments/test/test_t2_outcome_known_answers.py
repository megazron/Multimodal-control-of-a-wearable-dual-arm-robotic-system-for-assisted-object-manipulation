#!/usr/bin/env python3
"""KNOWN-ANSWER tests for the T2 discrete outcome.

The standing rule in CLAUDE.md: before a metric touches participant data it
must recover an answer that is known by construction. These sequences are
constructed, not rendered or simulated -- a gripper angle of 0.05 rad IS open
and a release 200 mm from the opening IS a miss, by arithmetic. That is the
category of synthetic input the rule permits.

Every failure mode encoded here has already been paid for somewhere in this
project:

  * the mock gripper boots at 0.79 rad, which made every trial after the
    first log an instant grasp in the original pilot;
  * a fully closed gripper means it closed on NOTHING, and calling that a
    grasp makes the experiment blame the participant for an empty table;
  * a held pose republished forever is indistinguishable from a live one, so
    a missing joint must read as unknown rather than as open.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
BIM = os.path.join(os.path.dirname(HERE), "experiments", "bimanual")
sys.path.insert(0, BIM)

import t2_metrics as t2m                                     # noqa: E402

HOLD = [-0.15, 0.35, 1.10]
OPEN = [0.15, 0.35, 1.20]
OPEN_RAD, SHUT_RAD, FREE_AIR = 0.05, 0.45, 0.79


def det(tol_mm=40):
    return t2m.T2Outcome(HOLD, OPEN, tol_mm)


def feed(d, rows):
    """rows = (t, fill_knuckle, fill_ee) with the container held still."""
    out = []
    for t, k, ee in rows:
        out.append(d.update(t, k, SHUT_RAD, ee, HOLD))
    return out


def cycle(t0, pick, release, k_carry=SHUT_RAD):
    """One complete pick-carry-release, as four samples."""
    return [(t0 + 0.0, OPEN_RAD, pick),
            (t0 + 0.1, k_carry, pick),
            (t0 + 0.2, k_carry, release),
            (t0 + 0.3, OPEN_RAD, release)]


# ---------------------------------------------------------------- placing
def test_a_clean_placement_is_counted_once():
    d = det()
    feed(d, cycle(0.0, [0.30, 0.35, 1.15], OPEN))
    s = d.summary()
    assert s["blocks_placed"] == 1
    assert s["blocks_missed"] == 0
    assert s["blocks_dropped"] == 0
    assert s["success_rate"] == 1.0


def test_three_placements_are_counted_three_times():
    d = det()
    for i in range(3):
        feed(d, cycle(i * 1.0, [0.30, 0.35, 1.15], OPEN))
    assert d.summary()["blocks_placed"] == 3
    assert d.summary()["picks"] == 3


def test_a_release_outside_the_footprint_is_a_MISS_not_a_placement():
    d = det(tol_mm=40)
    # 200 mm to the side of the opening: unambiguously outside 40 mm
    feed(d, cycle(0.0, [0.30, 0.35, 1.15], [0.35, 0.35, 1.20]))
    s = d.summary()
    assert s["blocks_placed"] == 0
    assert s["blocks_missed"] == 1
    assert s["success_rate"] == 0.0


def test_the_tolerance_boundary_is_where_it_is_declared():
    inside = [OPEN[0] + 0.035, OPEN[1], OPEN[2]]      # 35 mm, inside 40
    outside = [OPEN[0] + 0.045, OPEN[1], OPEN[2]]     # 45 mm, outside 40
    a, b = det(tol_mm=40), det(tol_mm=40)
    feed(a, cycle(0.0, [0.30, 0.35, 1.15], inside))
    feed(b, cycle(0.0, [0.30, 0.35, 1.15], outside))
    assert a.summary()["blocks_placed"] == 1
    assert b.summary()["blocks_missed"] == 1


def test_releasing_below_the_rim_is_not_placing_it_in():
    """Set down ON the container, not IN it."""
    d = det()
    low = [OPEN[0], OPEN[1], OPEN[2] - 0.05]
    feed(d, cycle(0.0, [0.30, 0.35, 1.15], low))
    assert d.summary()["blocks_missed"] == 1


# ------------------------------------------------------ the mock-boot trap
def test_a_gripper_ALREADY_CLOSED_at_trial_start_has_not_picked_anything():
    """THE PILOT BUG. The mock boots at 0.79 rad, past any closed threshold.

    Scoring on a level made every trial after the first log an instant grasp.
    A pick must be an OPEN -> HOLDING transition observed inside the trial.
    """
    d = det()
    d.update(0.0, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, SHUT_RAD, SHUT_RAD, OPEN, HOLD)
    d.update(0.2, OPEN_RAD, SHUT_RAD, OPEN, HOLD)
    s = d.summary()
    assert s["picks"] == 0
    assert s["blocks_placed"] == 0
    assert s["blocks_attempted"] == 0


def test_no_attempt_gives_NaN_not_a_zero_success_rate():
    """An absence of data and a rate of zero are different findings, and only
    one of them is about the participant."""
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    s = d.summary()
    assert s["blocks_attempted"] == 0
    assert s["success_rate"] != s["success_rate"]           # NaN


# --------------------------------------------------------- scene vs person
def test_closing_to_FREE_AIR_is_a_scene_fault_not_a_failed_grasp():
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, FREE_AIR, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    assert "block_absent" in d.summary()["scene_faults"]
    assert t2m.scene_fault_reason(d).startswith("block_absent")
    assert d.summary()["blocks_placed"] == 0


def test_losing_the_container_is_reported_as_box_lost():
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, OPEN_RAD, FREE_AIR, [0.30, 0.35, 1.15], HOLD)
    assert "box_lost" in d.summary()["scene_faults"]


def test_a_block_lost_in_transit_is_DROPPED_not_missed():
    """Dropped and missed are different failures and must not be merged: one
    is a grip that let go, the other is an aim that was off."""
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.2, FREE_AIR, SHUT_RAD, [0.22, 0.35, 1.18], HOLD)
    s = d.summary()
    assert s["blocks_dropped"] == 1
    assert s["blocks_missed"] == 0
    assert s["blocks_placed"] == 0


# ------------------------------------------------- the opening is CARRIED
def test_the_opening_MOVES_with_the_holding_arm():
    """The container is held, not bolted down.

    The same release, scored against a container that has been carried 100 mm
    sideways, must be a MISS -- and the release that follows the container
    must be a PLACEMENT. Scoring against a world-fixed opening gets both
    backwards.
    """
    shifted_hold = [HOLD[0] + 0.10, HOLD[1], HOLD[2]]
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], shifted_hold)
    d.update(0.1, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], shifted_hold)
    d.update(0.2, OPEN_RAD, SHUT_RAD, OPEN, shifted_hold)
    assert d.summary()["blocks_missed"] == 1, "world-fixed opening assumed"

    d2 = det()
    moved_open = [OPEN[0] + 0.10, OPEN[1], OPEN[2]]
    d2.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], shifted_hold)
    d2.update(0.1, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], shifted_hold)
    d2.update(0.2, OPEN_RAD, SHUT_RAD, moved_open, shifted_hold)
    assert d2.summary()["blocks_placed"] == 1


def test_hold_disturbance_is_measured_from_the_trials_own_start():
    d = det()
    for i, dx in enumerate((0.0, 0.01, 0.05, 0.02)):
        d.update(i * 0.1, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15],
                 [HOLD[0] + dx, HOLD[1], HOLD[2]])
    s = d.summary()
    assert abs(s["hold_disturbance_max_mm"] - 50.0) < 1e-6
    assert s["hold_disturbed"] is True


# ----------------------------------------------------------- missing data
def test_an_unknown_knuckle_never_manufactures_a_transition():
    """None is not open. A dead topic must not produce a release."""
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.2, None, SHUT_RAD, OPEN, HOLD)
    s = d.summary()
    assert s["blocks_placed"] == 0
    assert s["blocks_missed"] == 0
    assert s["carrying_at_end"] is True


def test_a_trial_ending_mid_carry_says_so():
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(0.1, SHUT_RAD, SHUT_RAD, [0.25, 0.35, 1.18], HOLD)
    s = d.summary()
    assert s["carrying_at_end"] is True
    assert s["blocks_attempted"] == 0


def test_cycle_time_is_pick_to_release():
    d = det()
    d.update(0.0, OPEN_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(1.0, SHUT_RAD, SHUT_RAD, [0.30, 0.35, 1.15], HOLD)
    d.update(3.5, OPEN_RAD, SHUT_RAD, OPEN, HOLD)
    s = d.summary()
    assert abs(s["mean_cycle_time_s"] - 2.5) < 1e-9
    assert abs(s["time_to_first_place_s"] - 3.5) < 1e-9


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
