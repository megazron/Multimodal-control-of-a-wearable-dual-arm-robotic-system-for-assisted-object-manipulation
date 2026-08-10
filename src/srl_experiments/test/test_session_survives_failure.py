#!/usr/bin/env python3
"""Break the session on purpose and check it survives, names it, and resumes.

A FAILURE PATH THAT HAS NOT BEEN EXERCISED IS NOT A FAILURE PATH. Every test
here breaks something the way it will actually break with a participant in the
room -- mid-trial, without warning -- rather than checking that the happy path
still works.
"""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from srl_experiments.session import (Session, CONSENT_STEPS,   # noqa: E402
                                     RUNNING, ABORTED, DONE)


def plan(n=6):
    return [("B%d" % (i // 2 + 1), "01_master_teleop", "abc"[i % 3])
            for i in range(n)]


def consented(tmp_path, n=6):
    s = Session("P01", plan(n), path=str(tmp_path / "s.json"))
    for step in CONSENT_STEPS:
        s.give_consent(step)
    s.start()
    return s


# ------------------------------------------------------------------ consent
def test_session_refuses_to_start_without_consent(tmp_path):
    s = Session("P01", plan(), path=str(tmp_path / "s.json"))
    with pytest.raises(RuntimeError) as e:
        s.start()
    assert "consent incomplete" in str(e.value)


def test_every_consent_step_is_required(tmp_path):
    for omit in CONSENT_STEPS:
        s = Session("P01", plan(), path=str(tmp_path / "s.json"))
        for step in CONSENT_STEPS:
            if step != omit:
                s.give_consent(step)
        with pytest.raises(RuntimeError):
            s.start()


def test_identifying_data_is_refused():
    for bad in ("john.name", "a@email", "dob1990"):
        with pytest.raises(ValueError):
            Session(bad, plan())


# ------------------------------------------------- failure in the middle
def test_a_mid_trial_failure_is_recorded_with_its_cause(tmp_path):
    s = consented(tmp_path)
    s.begin_trial()
    s.fail_trial("channel dropout: left j4 dead for 0.5 s")
    t = s.trials[-1]
    assert t.valid is False
    assert "channel dropout" in t.cause
    assert t.ended is not None


def test_an_invalid_trial_is_kept_not_deleted(tmp_path):
    s = consented(tmp_path)
    s.begin_trial()
    s.fail_trial("camera loss")
    s.redo()
    s.begin_trial()
    s.end_trial(valid=True)
    # the failed attempt is still on the record
    assert len(s.trials) == 2
    assert [t.valid for t in s.trials] == [False, True]
    assert s.trials[1].attempt == 2


def test_redo_reruns_the_same_plan_index(tmp_path):
    s = consented(tmp_path)
    a = s.begin_trial()
    s.fail_trial("e-stop tripped")
    s.redo()
    b = s.begin_trial()
    assert b.index == a.index
    assert b.attempt == a.attempt + 1


def test_skip_moves_past_and_leaves_the_failure_recorded(tmp_path):
    s = consented(tmp_path)
    a = s.begin_trial()
    s.fail_trial("process death: ik_follower_left")
    s.skip()
    b = s.begin_trial()
    assert b.index != a.index, "skip did not advance"
    assert s.trials[0].valid is False
    assert "process death" in s.trials[0].cause


def test_abort_keeps_the_data_and_flags_it(tmp_path):
    s = consented(tmp_path)
    s.begin_trial()
    s.abort("participant asked to stop", by="participant")
    assert s.state == ABORTED
    assert "participant" in s.abort_reason
    assert s.trials[-1].valid is False
    d = json.load(open(s.path))
    assert d["state"] == ABORTED
    assert d["trials"], "aborting threw the data away"


# ------------------------------------------------------- crash and resume
def test_state_is_on_disk_after_every_trial(tmp_path):
    s = consented(tmp_path)
    for i in range(3):
        s.begin_trial()
        s.end_trial(valid=True)
        d = json.load(open(s.path))
        assert len([t for t in d["trials"] if t["ended"]]) == i + 1


def test_a_crash_resumes_from_the_last_completed_trial(tmp_path):
    s = consented(tmp_path)
    s.begin_trial(); s.end_trial(valid=True)
    s.begin_trial(); s.end_trial(valid=True)
    s.begin_trial()                       # <- in flight when the GUI dies
    del s                                 # the process is gone

    r = Session.load(str(tmp_path / "s.json"))
    assert r.state == RUNNING
    assert r.done_count == 2
    # the in-flight trial is still open and can be failed or redone
    assert r.current is not None
    r.fail_trial("GUI process died mid-trial")
    assert r.next_index() == 2, "resumed at the wrong place"


def test_resumable_finds_the_interrupted_session(tmp_path):
    s = consented(tmp_path)
    s.begin_trial(); s.end_trial(valid=True)
    found = Session.resumable(str(tmp_path))
    assert [x.sid for x in found] == [s.sid]


def test_a_finished_session_is_not_offered_for_resume(tmp_path):
    s = consented(tmp_path, n=1)
    s.begin_trial(); s.end_trial(valid=True)
    s.begin_trial()                        # -> no plan left, marks DONE
    assert s.state == DONE
    assert Session.resumable(str(tmp_path)) == []


def test_the_save_is_atomic_so_a_kill_cannot_corrupt_it(tmp_path):
    """os.replace is atomic on POSIX: the reader sees the old file or the new
    one, never a half-written one. Simulated by checking no partial file is
    left behind and the live file always parses."""
    s = consented(tmp_path)
    for _ in range(4):
        s.begin_trial()
        s.end_trial(valid=True)
        assert not os.path.exists(s.path + ".tmp"), "temp file left behind"
        json.load(open(s.path))            # must always parse


def test_resume_refuses_an_unknown_schema(tmp_path):
    s = consented(tmp_path)
    s.save()
    d = json.load(open(s.path))
    d["version"] = 999
    json.dump(d, open(s.path, "w"))
    with pytest.raises(ValueError):
        Session.load(s.path)


# ------------------------------------------------------------ the live log
def test_the_log_is_readable_during_the_session(tmp_path):
    s = consented(tmp_path)
    s.begin_trial()
    s.fail_trial("camera loss")
    # readable from disk WHILE the session is still going
    d = json.load(open(s.path))
    texts = " ".join(e["text"] for e in d["log"])
    assert "trial 1" in texts and "INVALID" in texts
    assert any(e["level"] == "bad" for e in d["log"])
