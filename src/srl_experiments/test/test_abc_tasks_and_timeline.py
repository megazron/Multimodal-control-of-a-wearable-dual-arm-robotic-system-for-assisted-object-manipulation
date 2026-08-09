#!/usr/bin/env python3
"""Tasks A/B/C: the arithmetic, the geometry, and the two-hour claim.

Reachability is verified separately and against a LIVE solver
(`scripts/verify_abc_scenarios.py`, N=10 over densified paths). These tests
cover what does not need a robot: the internal consistency of the spec, and
the session fitting inside constraints that come from the wearer rather than
from the science.
"""
import math
import os
import sys

import pytest

ABC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "experiments", "abc")
sys.path.insert(0, ABC)
import tasks as T                                            # noqa: E402
import session_timeline as ST                                # noqa: E402


# ----------------------------------------------------------------- the set
def test_three_tasks_and_each_states_why_it_earns_its_slot():
    assert [t["key"] for t in T.ALL] == ["A", "B", "C"]
    for t in T.ALL:
        assert T.why_this_task(t)
        assert t["metrics"]


def test_the_optional_task_is_marked_out_of_the_timeline_AND_out_of_stats():
    """Pick-and-place has no DIRECT or VR condition, so it has no baseline and
    can carry no comparative statistic. Both facts must be on the record where
    someone wiring up an analyser will see them."""
    d = T.OPTIONAL_PICK_PLACE
    assert d["in_timeline"] is False
    assert d["comparative"] is False
    assert d not in T.ALL


def test_exactly_two_tasks_are_genuinely_bimanual_and_by_different_routes():
    """The geometry allows only two routes to a bimanual requirement, and the
    set uses each once. A third 'bimanual' task would be one of these two
    wearing a different name."""
    routes = [t["bimanual"] for t in T.ALL if t["bimanual"].startswith("YES")]
    assert len(routes) == 2
    assert any("route (a)" in r for r in routes)
    assert any("route (b)" in r for r in routes)


# ------------------------------------------------------------ the geometry
def test_the_sling_retains_the_ball_at_the_nominal_span():
    """Sag must exceed the ball's diameter, or it is gone before the trial
    starts -- which is exactly what a stale 350 mm sling did to nine clips."""
    sag = math.sqrt((T.SLING_L / 2) ** 2 - (T.TRAY_SEP / 2) ** 2)
    assert sag > 2 * T.BALL_R
    assert sag == pytest.approx(0.102, abs=5e-4)


def test_the_failure_threshold_is_INSIDE_the_reachable_band():
    """A sling that cannot fail measures nothing. s_max must be beyond the
    nominal span but close enough that a real coordination error reaches it."""
    s_max = 2 * math.sqrt((T.SLING_L / 2) ** 2 - (2 * T.BALL_R) ** 2)
    assert s_max == pytest.approx(T.TASK_B["fail_sep_m"], abs=5e-4)
    assert T.TRAY_SEP < s_max
    assert (s_max - T.TRAY_SEP) == pytest.approx(0.034, abs=1e-3)


def test_the_tilt_threshold_matches_the_span_it_was_computed_for():
    """The ANGLE is the failure criterion, so it must be recomputed whenever
    the span changes. At the superseded 310 mm span the same 60 mm read
    11.0 deg; feeding that number to a 500 mm tray is wrong by 5/3 and looks
    entirely plausible."""
    got = math.degrees(math.atan2(0.060, T.TRAY_SEP))
    assert got == pytest.approx(T.TASK_B["fail_tilt_deg"], abs=0.1)


def test_both_carry_grips_clear_the_dead_band():
    """Each grip sits at |x| = sep/2. The dead band runs to |x| = 0.25 on one
    side and 0.15 on the other; 0.25 is the smallest half-span that also
    clears the 120 mm wearer floor."""
    assert T.TRAY_SEP / 2 >= 0.25


def test_the_pursuit_centres_are_in_the_two_DISJOINT_sets():
    c = T.TASK_C["centres"]
    assert c["left"][0] > 0 and c["right"][0] < 0
    assert abs(c["left"][0]) >= 0.30 and abs(c["right"][0]) >= 0.30


def test_the_pursuit_shell_contains_the_lissajous():
    """The figure exceeds its own amplitude by sqrt(1 + 0.35^2) = 1.06, so a
    shell sized to the amplitude alone would let the marker leave the volume
    that was verified reachable. This caught a real bug before a participant
    ever saw it."""
    assert T.TASK_C["shell_radius_m"] >= T.TASK_C["amplitude_m"]


def test_pursuit_speeds_are_NOT_yoked():
    """With the two speeds yoked, b_self and b_cross are perfectly collinear
    and NEITHER is estimable -- the headline metric of the task would not
    exist."""
    pairs = list(T.TASK_C["speeds_m_s"].values())
    assert any(a != b for a, b in pairs), "every condition has equal speeds"
    ratios = {round(a / b, 4) for a, b in pairs if b}
    assert len(ratios) > 1, "all conditions share one speed ratio: still yoked"


# ------------------------------------------------------------- the session
def test_the_session_fits_two_hours_and_every_wearer_constraint():
    ok, rows = ST.check()
    assert ok, "\n".join("%s: %s" % (n, d) for n, o, d in rows if not o)


def test_the_binding_constraint_is_the_WEARER_not_the_clock():
    """Stated as a test so it stays true. If a change makes the clock bind
    first, the trade-off being made has changed and someone should notice."""
    clock_slack = ST.SESSION_CAP_MIN - ST.total_min()
    pack_slack = ST.PACK_TOTAL_MIN - ST.pack_total_min()
    assert clock_slack >= 0 and pack_slack >= 0
    assert clock_slack <= pack_slack, (
        "the clock now binds before the wearer's load does")


def test_the_stop_drill_cannot_be_shortened_to_make_the_clock_work():
    drill = [m for m, lab, _ in ST.BLOCKS if "STOP DRILL" in lab]
    assert drill and min(drill) >= 8


def test_every_block_duration_is_derived_from_a_trial_count():
    """A block length that is not derived from trials is a guess, and a guess
    is what makes a timeline slip on the day."""
    for key in ("TASK A", "TASK B", "TASK C"):
        need = ST.derived_block_min(key)
        for m, lab, _ in ST.BLOCKS:
            if key in lab:
                assert m >= need - 1.0, "%s: %d min allotted, %.1f needed" % (
                    lab, m, need)


def test_a_longer_session_is_REJECTED_by_the_checker():
    """The negative control. A checker that cannot fail is not a check, and
    'ALL CONSTRAINTS SATISFIED' would then mean nothing."""
    saved = ST.BLOCKS
    try:
        ST.BLOCKS = saved + [(30, "an extra half hour of something", True)]
        ok, rows = ST.check()
        assert not ok
        names = [n for n, o, _ in rows if not o]
        assert any("session fits" in n for n in names)
        assert any("pack-on continuous" in n for n in names)
    finally:
        ST.BLOCKS = saved
    assert ST.check()[0], "the fixture did not restore the real timeline"
