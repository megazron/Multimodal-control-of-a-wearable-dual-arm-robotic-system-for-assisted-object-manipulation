"""The wearer's SIZE must reach the check that stops the arm, not just the
offline one.

`SRL_WEARER_SIZE` has been a variable since 2026-08-15. docs/ENGINEERING_LOG.md describes it
as going "through the SAME one source the posture uses". Measured on
2026-08-21, with a real arm session behind us in which the arms touched the
wearer, it went to exactly one of the two places that matter:

    SRL_WEARER_SIZE=measured_adult
    mount_guard_node.WEARER   torso 0.430 x 0.220 x 0.503    followed
    clearance.ClearanceModel  torso 0.360 x 0.220 x 0.480    the mannequin
    clearance.ClearanceModel  upper arm 0.300 m long         the mannequin

`mount_guard_node` is an offline geometric guard. `clearance.py` is what
`ik_follower_node` refuses to publish below, what `real_homing_node` halts on
and what `sim_to_real_bridge` gates the real arm with. So the configured body
was bigger and the LIVE floor was still held against a mannequin -- the arm
stopped 150 mm from a chest 70 mm narrower than the one the operator had
selected, and every consumer logged the floor as satisfied.

That is docs/ENGINEERING_LOG.md's "feature present but does nothing" row: the size was
STORED and a consumer did not READ it. These tests are about the reading.
"""
import os
import sys
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from srl_teleop import clearance as CL              # noqa: E402
from srl_teleop import wearer_posture as WP         # noqa: E402


def _torso(model):
    return tuple(model.PARTS["torso"][0][1])


def _upper_arm(model):
    return tuple(model.PARTS["human_left_upper_arm"][0][1])


def test_an_explicit_size_reaches_the_live_model():
    """The defect itself: a profile must change the body that is enforced."""
    mannequin = CL.ClearanceModel(size="mannequin")
    adult = CL.ClearanceModel(size="measured_adult")
    assert _torso(mannequin) != _torso(adult), (
        "the live clearance model returned the same torso for two different "
        "people -- SRL_WEARER_SIZE does not reach it")
    prof = WP.size_profile("measured_adult")
    assert _torso(adult) == pytest.approx(
        (prof["chest_w"], prof["chest_d"], prof["chest_h"]))
    assert _upper_arm(adult) == pytest.approx(
        (prof["upper_arm_rad"], prof["upper_arm_len"]))


def test_the_environment_variable_reaches_it_too(monkeypatch):
    """A node started with SRL_WEARER_SIZE set and nothing passed in code.

    This is how it is actually used: the launch sets the variable, the node
    constructs a ClearanceModel with no arguments.
    """
    monkeypatch.setenv(WP.SIZE_ENV, "measured_adult")
    assert _torso(CL.ClearanceModel()) == _torso(
        CL.ClearanceModel(size="measured_adult"))


def test_the_default_is_still_the_shipped_mannequin(monkeypatch):
    """The regression control. Every clearance figure in this project was
    measured against these numbers, so asking for nothing has to return them
    byte for byte -- otherwise this fix silently re-baselines the archive."""
    monkeypatch.delenv(WP.SIZE_ENV, raising=False)
    m = CL.ClearanceModel()
    assert _torso(m) == (0.36, 0.22, 0.48)
    assert tuple(m.PARTS["hips"][0][1]) == (0.32, 0.21, 0.18)
    assert _upper_arm(m) == (0.050, 0.30)
    assert tuple(m.PARTS["human_right_lower_arm"][0][1]) == (0.045, 0.26)
    assert tuple(m.PARTS["human_left_hand"][0][1]) == (0.09, 0.05, 0.18)
    assert tuple(m.PARTS["head"][0][1]) == (0.105,)


def test_the_module_constants_are_the_same_body():
    """TORSO_BODY and friends are imported by other modules. They must be the
    mannequin still, and they must be DERIVED rather than a second copy."""
    assert CL.TORSO_BODY == CL.parts_for("mannequin")["torso"]
    assert CL.HIPS_BODY == CL.parts_for("mannequin")["hips"]
    assert set(CL.WEARER_ARM_PARTS) == {
        "human_%s_%s" % (s, p)
        for s in ("left", "right")
        for p in ("upper_arm", "lower_arm", "hand")}


def test_the_live_check_and_the_offline_guard_use_one_body(monkeypatch):
    """ONE SOURCE, checked across the two modules that enforce the floor.

    They read it by different routes -- the guard builds world primitives,
    this builds link-frame ones -- so the only thing that can be compared is
    the DIMENSIONS, and those are what the size profile sets.
    """
    monkeypatch.setenv(WP.SIZE_ENV, "measured_adult")
    live = CL.ClearanceModel()
    guard = WP.wearer_model("down", size="measured_adult", present=True)
    by_name = {n: dims for n, _k, dims, _c, _r in guard}
    assert tuple(by_name["torso"]) == _torso(live)
    assert tuple(by_name["hips"]) == tuple(live.PARTS["hips"][0][1])
    assert tuple(by_name["L-upperarm"]) == _upper_arm(live)
    assert tuple(by_name["R-forearm"]) == tuple(
        live.PARTS["human_right_lower_arm"][0][1])


def test_a_bigger_body_never_reports_more_clearance():
    """The direction the whole safety case rests on.

    Not argued from the definition: a point is placed in front of the torso
    and scored against a ladder of chest widths. Every step outward must
    leave the reading the same or SMALLER. A model that grows and reports
    more room is the one failure that cannot be allowed to pass silently.
    """
    point = {"torso": [(0.30, 0.00, 0.17)]}
    prev = None
    for w in (0.30, 0.36, 0.43, 0.50, 0.58):
        prof = dict(WP.SHIPPED_SIZE, chest_w=w)
        d, _part = CL.ClearanceModel(size=prof).clearance(point)
        if prev is not None:
            assert d <= prev + 1e-12, (
                "chest_w %.2f cleared MORE than the narrower body" % w)
        prev = d
    assert prev < 0.30, "the widest body should have swallowed the point"


def test_it_can_still_read_a_hit():
    """A clearance model that cannot go negative cannot report a collision,
    and reporting one is the only reason the floor exists."""
    inside = {"torso": [(0.0, 0.0, 0.17)]}
    d, part = CL.ClearanceModel().clearance(inside)
    assert d < 0.0 and part == "torso"
