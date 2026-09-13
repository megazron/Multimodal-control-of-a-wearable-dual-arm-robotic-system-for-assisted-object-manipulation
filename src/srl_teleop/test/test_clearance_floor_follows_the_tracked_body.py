"""The clearance floor must enforce against the SAME body MoveIt plans against.

THE GAP THIS CLOSES. The scene camera can measure the wearer, and that
measurement reached MoveIt's planning scene -- but `mount_guard_node`, which
is the geometric check HARD CONSTRAINT 11 actually names, built its wearer at
IMPORT TIME from `wearer_posture.wearer_model()` and never looked again. So
the body MoveIt planned against and the body the 150 mm floor was enforced
against were two different bodies, and the one enforcing the floor was the
mannequin.

WHY IT IS SAFE TO LET A CAMERA NEAR THIS AT ALL. It cannot loosen the guard.
`wearer_tracking.fuse()` keeps whichever of the tracked and mannequin
primitive is CLOSER to the robot, per part, so every path through `wearer()`
returns something at least as large as `WEARER`. The tests below drive the
four ways the camera can be wrong and require the mannequin back each time.
"""
import json
import os
import time

import pytest

from srl_teleop import wearer_posture

WT = pytest.importorskip("srl_perception.wearer_tracking")

MANNEQUIN = wearer_posture.wearer_model("down", "mannequin")
GUARD_SRC = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "srl_teleop", "mount_guard_node.py")


def _estimate_json(forward, conf=1.0, usable=True):
    """A body standing `forward` metres in front of the wearer's centre."""
    segs = {}
    for name, kind, dims, ctr, rpy in MANNEQUIN:
        if name not in WT.TRACKED_PARTS:
            continue
        segs[name] = dict(kind=kind, dims=list(dims),
                          centre=[ctr[0], ctr[1] + forward, ctr[2]],
                          rpy=list(rpy), conf=conf, usable=usable,
                          reasons=[])
    return json.dumps(dict(state="test", segments=segs))


def _fuse_from(payload, age=0.0):
    """What `mount_guard_node.wearer()` computes, without a ROS graph.

    A transcription, kept honest by `test_the_transcription_matches_the_node`
    below -- the node needs rclpy, a TF listener and a live graph, and none of
    that is the thing under test.
    """
    if payload is None or age > WT.MAX_AGE_S * 4:
        return MANNEQUIN
    est = json.loads(payload)
    segs = [WT.Segment(n, d["kind"], d["dims"], d["centre"], d["rpy"],
                       d["conf"], d["reasons"])
            for n, d in (est.get("segments") or {}).items() if d["usable"]]
    if not segs:
        return MANNEQUIN
    now = time.monotonic()
    fused, _dec = WT.fuse(MANNEQUIN, WT.Estimate(segs, stamp=now), now)
    return fused


def test_no_camera_at_all_is_exactly_the_mannequin():
    """The state the rig is in today, and the one it must stay in when
    nothing is publishing."""
    assert _fuse_from(None) == MANNEQUIN


def test_a_stale_estimate_is_the_mannequin():
    assert _fuse_from(_estimate_json(0.30), age=10.0) == MANNEQUIN


def test_every_segment_gated_out_is_the_mannequin():
    assert _fuse_from(_estimate_json(0.30, usable=False)) == MANNEQUIN


def test_a_body_further_from_the_robot_is_the_mannequin():
    """The dangerous direction, and the reason this is safe to wire in.

    A camera that says the wearer stepped back must not shrink the body the
    floor is enforced against."""
    for retreat in (0.10, 0.50, 3.00):
        assert _fuse_from(_estimate_json(-retreat)) == MANNEQUIN, retreat


def test_a_body_nearer_the_robot_does_change_the_floor():
    """The other half. Without it every test above passes on a guard that
    ignores the camera completely."""
    fused = _fuse_from(_estimate_json(0.30))
    assert fused != MANNEQUIN


def test_the_guard_asks_for_the_body_rather_than_reading_the_constant():
    """The wiring, asserted at the call site.

    A `wearer()` method that exists and is never called is the 'feature
    present but does nothing' row of docs/ENGINEERING_LOG.md's instrument table, and it is
    invisible: the guard keeps working, against the wrong body.
    """
    src = open(GUARD_SRC).read()
    assert "for nm, kind, prm, ctr, rpy in self.wearer():" in src, (
        "the distance loop no longer asks wearer() for the body")
    assert "in WEARER:" not in src, (
        "something still iterates the import-time constant directly")


def test_the_transcription_matches_the_node():
    src = open(GUARD_SRC).read()
    for needle in ("def wearer(self)", "_WT.fuse(WEARER", "MAX_AGE_S * 4",
                   "use_tracked_wearer", "/wearer/enforced"):
        assert needle in src, (
            "%r is gone from mount_guard_node -- this file now models code "
            "that does not exist" % needle)


def test_the_guard_says_which_body_it_is_enforcing():
    """A floor enforced against a measured person and a floor enforced
    against a mannequin are different safety cases, and the operator cannot
    tell them apart by watching the arm."""
    src = open(GUARD_SRC).read()
    assert "body_pub" in src and "enforcing" in src
