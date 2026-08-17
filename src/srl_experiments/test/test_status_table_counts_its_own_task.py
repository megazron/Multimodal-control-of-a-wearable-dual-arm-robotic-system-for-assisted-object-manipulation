"""A CLIP COUNTS FOR THE TASK IT CONTAINS, NOT THE FOLDER IT SITS IN.

WHY. `status_table.py` reads `<mode>/<TASK>/<scenario>` off the path. That
directory name is a CLAIM, and the file already carries two scars from trusting
task keys: `data()` once substring-matched, so "t0" found inside the trial index
`..._t001_...` credited every task with every other task's rows; and `main()`
once prefix-tested, so `"t1s2".startswith("t1")` let T1 claim T1 stage 2's
clips. Both fixes were about the KEY. Neither looked inside a clip.

So a clip recorded as one task and written into another task's directory would
still have been counted, and the table would have reported a task as covered on
the strength of another task's footage -- with no gap showing anywhere. That is
the "everything matches" row of CLAUDE.md's instrument table again: two
namespaces, one key space.

`clip_scene` stamps the task it was publishing into `scene_events.json`, so the
check costs one string comparison per clip. This file proves the guard can
actually reject, because an audit that finds nothing today proves only that
today is clean -- CLAUDE.md: a check that cannot fail on a deliberately broken
input is not a check.

Audited over all 37 clips on 2026-08-17: 0 mismatched, 0 without a record, 0
groups of byte-identical footage. The guard changes no count; it stops the count
being wrong quietly later.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import status_table as ST                                      # noqa: E402


def _clip(root, mode, task, scen, says_task, n_mp4=2):
    """A clip directory whose folder says `task` and whose record says
    `says_task`. `says_task=None` writes no record at all."""
    d = os.path.join(root, mode, task, scen)
    os.makedirs(d, exist_ok=True)
    for i in range(n_mp4):
        open(os.path.join(d, "rviz_%d.mp4" % i), "wb").close()
    if says_task is not None:
        json.dump({"task": says_task, "items": []},
                  open(os.path.join(d, "scene_events.json"), "w"))
    return d


def _run(tmp_path, monkeypatch):
    monkeypatch.setattr(ST, "WS", str(tmp_path))
    rep = {}
    return ST.clips(rep), rep


def test_an_honest_clip_is_counted(tmp_path, monkeypatch):
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T1S2", "S2_both", "t1s2")
    found, rep = _run(tmp_path, monkeypatch)
    assert ("t1s2", "01_master_teleop") in found
    assert rep["mismatched"] == []
    assert rep["unverified"] == []


def test_a_clip_filed_under_the_wrong_task_is_NOT_counted(tmp_path,
                                                          monkeypatch):
    """THE EXACT SCENARIO: T1's footage sitting in the T1S2 folder. T1S2 must
    read as having no clip, and the mismatch must be named."""
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T1S2", "S2_both", "t1")
    found, rep = _run(tmp_path, monkeypatch)
    assert ("t1s2", "01_master_teleop") not in found, (
        "a clip whose own record says it is T1 was counted as T1S2 coverage")
    assert len(rep["mismatched"]) == 1
    p, want, said = rep["mismatched"][0]
    assert want == "t1s2" and said == "t1"


def test_a_task_with_zero_clips_reads_as_zero(tmp_path, monkeypatch):
    """Only T1 is on disk, so every other task must be absent from the count
    rather than inheriting anything."""
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T1", "S1_left_arm", "t1")
    found, _rep = _run(tmp_path, monkeypatch)
    assert ("t1", "01_master_teleop") in found
    for t in ("t0", "t1s2", "t2", "t3"):
        assert (t, "01_master_teleop") not in found, (
            "%s has no clip on disk and must not be credited with T1's" % t)


def test_t1_does_not_claim_t1s2_and_t1s2_does_not_claim_t1(tmp_path,
                                                           monkeypatch):
    """The prefix pair, both ways, with real records on both sides."""
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T1", "S1_left_arm", "t1")
    _clip(str(root), "01_master_teleop", "T1S2", "S2_both", "t1s2")
    found, rep = _run(tmp_path, monkeypatch)
    assert len(found[("t1", "01_master_teleop")]) == 1
    assert len(found[("t1s2", "01_master_teleop")]) == 1
    assert rep["mismatched"] == []


def test_a_clip_with_no_record_is_counted_but_flagged(tmp_path, monkeypatch):
    """Absence of the record is not evidence of the WRONG task, so dropping it
    would under-report. It is counted and listed as unverified."""
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T2", "S2_full_lift", None)
    found, rep = _run(tmp_path, monkeypatch)
    assert ("t2", "01_master_teleop") in found
    assert len(rep["unverified"]) == 1


def test_a_directory_with_no_video_is_not_a_clip(tmp_path, monkeypatch):
    root = tmp_path / "recordings" / "verification"
    _clip(str(root), "01_master_teleop", "T3", "S1", "t3", n_mp4=0)
    found, _rep = _run(tmp_path, monkeypatch)
    assert found == {}


def test_the_real_tree_has_no_mismatches(tmp_path, monkeypatch):
    """The live audit, so a real cross-credited clip fails the suite rather
    than waiting for somebody to run the reporter."""
    rep = {}
    ST.clips(rep)
    assert rep["mismatched"] == [], (
        "clips are filed under a task their own record contradicts: %s"
        % rep["mismatched"])
