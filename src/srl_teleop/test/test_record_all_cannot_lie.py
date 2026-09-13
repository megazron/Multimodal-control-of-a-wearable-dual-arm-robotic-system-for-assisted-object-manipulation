#!/usr/bin/env python3
"""record_all's summary must FAIL on a recording that captured nothing.

docs/ENGINEERING_LOG.md's standing rule: a check that cannot fail on a deliberately broken
input is not a check. The defect this recorder replaces was precisely a
capture step that ran, exited zero and produced an empty directory -- so the
one thing the summary MUST do is name a channel that never published, rather
than reporting a clean-looking session.

It also pins the freshness convention, which is the repo's own instrument
failure mode: "data fresh but never changes" and "zero variance" are both
DEAD channels, and a recorder that reports a repeated stale value as data has
lied about the run.
"""
import csv
import importlib.util
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCRIPT = os.path.join(WS, "scripts", "record_all.py")


def _load():
    spec = importlib.util.spec_from_file_location("record_all", SCRIPT)
    m = importlib.util.module_from_spec(spec)
    sys.modules["record_all"] = m
    spec.loader.exec_module(m)
    return m


def _write(d, rows, cols):
    with open(os.path.join(d, "trail.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def test_a_recording_that_captured_nothing_is_reported_as_dead():
    m = _load()
    cols = m._trail_columns()
    with tempfile.TemporaryDirectory() as d:
        # Every row present, every channel never fresh: the empty-capture case.
        rows = [{c: "" for c in cols} for _ in range(20)]
        for i, r in enumerate(rows):
            r["t"] = i * 0.05
            for c in cols:
                if c.endswith("_fresh"):
                    r[c] = 0
        _write(d, rows, cols)
        s = m.summarise(d)
    assert s["trail_rows"] == 20
    assert s["channels_live"] == [], s["channels_live"]
    assert "sim_left" in s["channels_DEAD"]
    assert "vr_right" in s["channels_DEAD"]
    # The whole point: it must not come out looking like a good session.
    assert len(s["channels_DEAD"]) == len(
        [c for c in cols if c.endswith("_fresh")])


def test_a_live_channel_is_not_reported_dead():
    """The control. Without it the test above passes on a summary that calls
    EVERYTHING dead, which would be just as useless."""
    m = _load()
    cols = m._trail_columns()
    with tempfile.TemporaryDirectory() as d:
        rows = [{c: "" for c in cols} for _ in range(20)]
        for i, r in enumerate(rows):
            r["t"] = i * 0.05
            for c in cols:
                if c.endswith("_fresh"):
                    r[c] = 0
            r["sim_left_fresh"] = 1
        _write(d, rows, cols)
        s = m.summarise(d)
    assert "sim_left" in s["channels_live"]
    assert "sim_left" not in s["channels_DEAD"]
    assert "real_left" in s["channels_DEAD"]


def test_a_stale_repeated_value_is_still_dead():
    """A channel publishing the SAME value forever is dead, and a channel
    that stopped publishing while its last value persists is dead. Freshness
    is keyed on ARRIVAL, so both read 0 -- this pins that a constant column
    with no arrivals is never counted as live."""
    m = _load()
    cols = m._trail_columns()
    with tempfile.TemporaryDirectory() as d:
        rows = [{c: "" for c in cols} for _ in range(20)]
        for i, r in enumerate(rows):
            r["t"] = i * 0.05
            for c in cols:
                if c.endswith("_fresh"):
                    r[c] = 0
            r["sim_left_j1"] = 0.4242          # a value, republished forever
        _write(d, rows, cols)
        s = m.summarise(d)
    assert "sim_left" in s["channels_DEAD"], \
        "a repeated stale value must not count as a live channel"


def test_clutch_refusals_are_counted_by_reason():
    """The VR complaint is 'the clutch never engages'. The summary has to say
    WHY, by reason, or the next session is spent guessing again."""
    m = _load()
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "events.jsonl"), "w") as f:
            for reason in ["hand moving at engage - hold still"] * 3 + \
                          ["controller not tracked"] * 2:
                f.write(json.dumps(dict(kind="clutch_refused", hand="left",
                                        reason=reason)) + "\n")
            f.write(json.dumps(dict(kind="clutch", hand="left",
                                    engaged=True)) + "\n")
        s = m.summarise(d)
    assert s["clutch_refusals"]["hand moving at engage - hold still"] == 3
    assert s["clutch_refusals"]["controller not tracked"] == 2
    assert s["events_by_kind"]["clutch"] == 1


def test_the_participant_field_refuses_a_name():
    """docs/ENGINEERING_LOG.md hard constraint 12: anonymity is enforced in code."""
    import subprocess
    r = subprocess.run(
        [sys.executable, SCRIPT, "--participant", "someone@example.com",
         "--no-bag", "--no-arms"],
        capture_output=True, text=True, timeout=60, cwd=WS)
    assert r.returncode == 3, (r.returncode, r.stdout, r.stderr)
    assert "REFUSED" in r.stdout


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
