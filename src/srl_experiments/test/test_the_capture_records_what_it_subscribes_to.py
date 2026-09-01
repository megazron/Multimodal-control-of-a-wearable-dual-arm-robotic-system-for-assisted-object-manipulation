"""The capture must record the quantities it subscribes to, and say what it did.

Three defects, all of the same shape -- a value that exists, is read, and
never reaches anybody -- and all three found while preparing the lateral-axis
capture.

  1. `master_pose_node` wrote the accelerometer gate into status index 7 and
     then overwrote index 7 with the data age, every frame, before publishing.
     The dashboard read index 7 and printed "elev HELD" from it, so a STALE
     MASTER was reported under a label naming a different fault entirely.

  2. `record_trajectories` subscribed to /master_status_<arm> and wrote four
     of its nine fields. The three it discarded -- elevation, azimuth and
     reach -- are the decomposition's own outputs and are exactly where the
     left/right defect lives. An analysis without them has to re-derive the
     decomposition, and a re-derivation that differs from the node's by a sign
     or a datum is indistinguishable from a finding.

  3. The gate mapping stamped into every capture's metadata was the INVERSE of
     the mapping the code uses -- and that inversion is the one that made the
     2026-08-06 capture unusable. Anyone reading the metadata to check whether
     it had recurred would have concluded that it had.
"""
import json
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments",
                                "trajectory_capture"))

import capture_protocol as cp                       # noqa: E402
SRC = open(os.path.join(WS, "src", "srl_experiments", "trajectory_capture",
                        "record_trajectories.py")).read()
MPN = open(os.path.join(WS, "src", "srl_teleop", "srl_teleop",
                        "master_pose_node.py")).read()
DASH = open(os.path.join(WS, "src", "srl_teleop", "srl_teleop",
                         "dashboard.py")).read()


# --------------------------------------------------- the status array layout
def test_no_two_status_fields_share_an_index():
    """The array is written by two methods. Every index written must be
    written for one purpose only, or the later writer wins silently."""
    import re
    idx = re.findall(r"self\.status\[\w+\]\[(ST_[A-Z_]+)\]", MPN)
    assert idx, "the status array is being indexed by NUMBER again; the " \
                "whole point of the named layout is that two writers cannot " \
                "collide without it being visible"
    # Every name used must resolve, and no two names may be the same index.
    names = {}
    for line in MPN.splitlines():
        m = re.match(r"^ST_[A-Z_, ]+ = [\d, ]+$", line.strip())
        if m:
            lhs, rhs = line.split("=")
            for k, v in zip([x.strip() for x in lhs.split(",")],
                            [int(x) for x in rhs.split(",")]):
                names[k] = v
    assert names, "no ST_* layout found"
    assert len(set(names.values())) == len(names), \
        "two status fields share an index: %r" % (names,)
    assert names["ST_DATA_AGE_S"] != names["ST_ACCEL_GATED"], \
        "the data age and the accelerometer gate are in one slot again"


def test_the_dashboard_reads_the_gate_and_not_the_data_age():
    assert "ms[ST_ACCEL_GATED]" in DASH, \
        "the dashboard is indexing the status array by number again"
    assert "elev HELD" in DASH
    i = DASH.index("elev HELD")
    window = DASH[max(0, i - 400):i + 200]
    assert "ms[7]" not in window, \
        "'elev HELD' is being driven from index 7, which master_pose_node " \
        "overwrites with the data age"


# ------------------------------------------------- what the recorder records
@pytest.mark.parametrize("field", ["elev_deg", "azim_deg", "reach_m",
                                   "elev_held", "station"])
def test_the_recorder_has_a_column_for(field):
    assert '"%%s_%s" %% a' % field in SRC or '"%s"' % field in SRC, \
        "no column for %s; it is subscribed to and discarded" % field


def test_the_decomposition_outputs_are_written_not_just_read():
    """A column that exists and is never assigned is the same defect one
    level down."""
    for f in ("elev_deg", "azim_deg", "reach_m"):
        assert 'r["%%s_%s" %% a]' % f in SRC, \
            "%s has a column but nothing writes it" % f


def test_a_missing_status_field_stays_empty_rather_than_becoming_zero():
    """A scripted operator publishes a shorter array. A zero there would read
    as 'the elevation was not held', which is a claim, not an absence."""
    i = SRC.index('r["%s_elev_held" % a]')
    assert "len(s) > 8" in SRC[max(0, i - 300):i], \
        "elev_held is written without checking the array is long enough"


# --------------------------------------------------------- the gate mapping
def test_the_gate_metadata_is_derived_from_the_constant():
    assert "GATE_INDEX[a]" in SRC.split("gate_mapping", 1)[1][:400], \
        "the gate mapping in the capture metadata is typed rather than " \
        "derived, which is how it came to state the exact inverse of what " \
        "the code does"


def test_each_arm_is_gated_by_the_other_arms_button():
    """An arm's own button toggles that arm's clutch, so gating a left sweep
    with the left button disengages the clutch exactly when the sweep starts.
    That happened, and it cost 41 of 42 directional segments."""
    ns = {}
    exec(SRC[SRC.index("GATE_INDEX = "):
             SRC.index("\n", SRC.index("GATE_INDEX = "))], ns)
    gate = ns["GATE_INDEX"]
    # [fsr1, fsr2, btn1, btn2]: index 2 is btn1, index 3 is btn2.
    # Measured: the left arm's OWN button is btn2, the right arm's is btn1.
    own = {"left": 3, "right": 2}
    for arm in ("left", "right"):
        assert gate[arm] != own[arm], \
            "%s segments are gated by the %s arm's OWN button" % (arm, arm)


# ------------------------------------------------------------- the protocol
def test_block_g_probes_the_failing_axis_at_every_station():
    g = [s for s in cp.segments() if s["block"] == "G"]
    assert g, "block G is missing"
    for arm in cp.ARMS:
        for st, _ in cp.STATIONS:
            got = {s["direction"] for s in g
                   if s["arm"] == arm and s["station"] == st}
            assert "left_right" in got, \
                "no lateral probe at station %s on the %s arm" % (st, arm)


def test_block_g_carries_a_working_axis_as_a_control():
    """A measurement of a broken axis with no working axis beside it cannot
    separate 'this axis is wrong' from 'this session was wrong'."""
    g = [s for s in cp.segments() if s["block"] == "G"]
    for arm in cp.ARMS:
        for st, _ in cp.STATIONS:
            got = {s["direction"] for s in g
                   if s["arm"] == arm and s["station"] == st}
            assert "up_down" in got, \
                "station %s on the %s arm has no control probe" % (st, arm)


def test_block_g_varies_arm_extension():
    """The mechanism is that the lateral gain scales with how bent the arm is,
    so a grid that does not vary extension cannot see it."""
    names = {s for s, _ in cp.STATIONS}
    assert {"near", "far"} <= names, \
        "block G does not sample a folded and an extended arm, which is the " \
        "one axis the suspected mechanism varies along"


def test_every_g_instruction_names_the_station_and_the_sweep():
    for s in (x for x in cp.segments() if x["block"] == "G"):
        assert "HOLD STILL" in s["instruction"], \
            "%s does not ask for the stationary datum" % s["label"]
        assert s["arm"].upper() in s["instruction"]
