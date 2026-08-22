"""THE WRIST-TO-PAD OFFSET IS ONE VECTOR, IN THE FRAME IT BELONGS TO.

`clip_tasks.PAD_OFFSET_BY_ARM` carries two WORLD-frame vectors recorded at the
anchor, 48.3 mm apart, and a comment attributing the difference to the arms
being parked asymmetrically. `grasp_frames.PAD_MID_EE` carries ONE vector in
the END EFFECTOR frame.

Both cannot be independently true, and the one that generalises is the
EE-frame vector: T1 no longer commands the pinned anchor, so a world-frame
offset measured at the anchor is wrong for it by construction.

WHICH OF THE TWO IS RIGHT WAS SETTLED BY MEASUREMENT ON 2026-08-18, AND IT WAS
NOT THE ONE THIS FILE USED TO ASSERT. `PAD_MID_EE` was 0.1118 -- the magnitude
of the world vector -- and this test asserted the two agreed to 0.1 mm, which
they did, because one was arithmetic on the other. `/compute_fk` on the two
finger-tip links says the pad midpoint is 0.09833 m along the tool axis, on
both arms, agreeing to 0.000 mm. The derivation was 13.47 mm long, and it
showed up as `verify_t1.py` reporting a 13.48 mm pad miss on all four cubes of
both arms -- the signature of a constant, not of a path.

So the assertions changed, deliberately and in one direction: the EE-frame
constant is pinned to the MEASUREMENT (`recordings/baselines/pad_mid_ee.json`,
written by `scripts/measure_pad_mid_ee.py`).

AND ON 2026-08-23 `PAD_OFFSET_BY_ARM` WAS MOVED ONTO IT. It had been left
alone on the argument that task and picture agreed with each other -- true,
because `clip_scene` draws each object at `ee_for(obj) + this same offset`, so
the two cancel and the picture is right whatever the constant is. What did
NOT cancel is where the FINGERS go: the pads sit at
`wrist + R(anchor) . PAD_MID_EE`, so a 13.45 mm-too-long offset put them
13.45 mm SHORT of the declared object on every object of T0, T2 and T3, and
that was the largest single term in `scripts/measure_control_budget.py`'s
error budget against a 30 mm capture gate.

`clip_tasks.PAD_OFFSET_BY_ARM` is now DERIVED from `grasp_frames.PAD_MID_EE`
rotated by each arm's own anchor. Task and picture still agree -- both sides
use the one constant -- and the pads now land on the declared coordinate to
0.05 mm, which is the 0.1 mm rounding grid `ee_for` declares coordinates on
and nothing else. What moved is the commanded WRIST, 13.45 mm along the tool
axis toward the object, and every T0/T2/T3 pose was re-verified after it.

These tests now assert the FIXED state, and the legacy vectors are kept in
`clip_tasks.LEGACY_PAD_OFFSET_BY_ARM` so the size and direction of what was
corrected stays checkable rather than becoming folklore.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_experiments",
                                "experiments", "abc"))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_teleop"))

import clip_tasks as CT                                       # noqa: E402
import grasp_frames as GF                                     # noqa: E402
from srl_teleop import master_calibration as MC               # noqa: E402

TOL_M = 0.0001


def test_the_ee_offset_is_the_fk_measurement():
    """Pinned to the record `scripts/measure_pad_mid_ee.py` wrote.

    Not to the other constant. A test that derives its expectation from the
    thing it is checking cannot fail, which is exactly what happened here.
    """
    rec = os.path.join(ROOT, "recordings", "baselines", "pad_mid_ee.json")
    assert os.path.exists(rec), (
        "no pad_mid_ee.json -- run scripts/measure_pad_mid_ee.py against a "
        "live stack before trusting any T1 grasp")
    d = json.load(open(rec))
    for arm in ("left", "right"):
        got = np.asarray(d["arms"][arm]["pad_mid_in_ee"], float)
        want = np.asarray(GF.PAD_MID_EE, float)
        err = float(np.linalg.norm(got - want))
        assert err <= TOL_M, (
            "%s arm: FK measured the pad midpoint at %s in the EE frame and "
            "grasp_frames.PAD_MID_EE says %s, %.3f mm apart"
            % (arm, list(got), list(want), err * 1000.0))


def test_the_world_offset_is_the_ee_measurement_rotated():
    """ONE SOURCE. Rotate the world offset back through the arm's own anchor
    and it must land exactly on `PAD_MID_EE` -- not near it, ON it, because it
    is now that constant rotated rather than a second recording of the same
    quantity."""
    for arm in ("left", "right"):
        w = np.asarray(CT.PAD_OFFSET_BY_ARM[arm], float)
        q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
        in_ee = GF.q_matrix(q / np.linalg.norm(q)).T @ w
        err = float(np.linalg.norm(in_ee - np.asarray(GF.PAD_MID_EE)))
        assert err < 1e-5, (
            "%s arm: the world offset rotates back to %s, and PAD_MID_EE is "
            "%s -- %.4f mm apart. They are meant to be the same constant."
            % (arm, list(np.round(in_ee, 6)), list(GF.PAD_MID_EE), err * 1000))


def test_what_was_corrected_is_still_recorded_and_its_size_is_pinned():
    """The legacy vectors are kept so the correction stays checkable.

    A fix whose size lives only in a commit message becomes folklore. This
    asserts what was wrong: 13.4 to 13.6 mm too long, purely along the tool
    axis, in the direction that made the fingers stop SHORT of the declared
    point.
    """
    for arm in ("left", "right"):
        w = np.asarray(CT.LEGACY_PAD_OFFSET_BY_ARM[arm], float)
        q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
        in_ee = GF.q_matrix(q / np.linalg.norm(q)).T @ w
        off_axis = float(np.linalg.norm(in_ee[:2]))
        assert off_axis < 0.0002, (
            "%s arm: the legacy world offset is %.3f mm off the tool axis, so "
            "it disagreed in DIRECTION as well as length and this test's "
            "premise no longer holds" % (arm, off_axis * 1000))
        extra_mm = (float(in_ee[2]) - float(GF.PAD_MID_EE[2])) * 1000.0
        assert 13.0 <= extra_mm <= 14.0, (
            "%s arm: the legacy offset was %.2f mm longer than the FK "
            "measurement; it was 13.45 to 13.51 mm. One of the two has been "
            "edited." % (arm, extra_mm))


def test_the_pads_now_land_on_the_declared_object():
    """The whole point, stated as the quantity a grasp actually cares about.

    `ee_for` says where the WRIST goes; the pads are `PAD_MID_EE` further
    along the tool axis. Those two must compose to the declared object
    position, and until 2026-08-23 they composed to 13.45 mm short of it.
    The residual is the 0.1 mm grid `ee_for` rounds its coordinates to.
    """
    for arm in ("left", "right"):
        q = np.asarray(MC.WORKSPACE_ORIENT[arm], float)
        R = GF.q_matrix(q / np.linalg.norm(q))
        for obj in ([0.56, 0.12, 1.12], [-0.74, 0.23, 1.10],
                    [0.40, 0.175, 1.12], [0.29, -0.05, 0.98]):
            pads = np.asarray(CT.ee_for(obj, arm), float) + R @ np.asarray(
                GF.PAD_MID_EE)
            miss = float(np.linalg.norm(pads - np.asarray(obj)))
            assert miss < 0.0002, (
                "%s arm at %s: the pads land %.3f mm from the declared object"
                % (arm, obj, miss * 1000))
            legacy = (np.asarray(obj)
                      - np.asarray(CT.LEGACY_PAD_OFFSET_BY_ARM[arm])
                      + R @ np.asarray(GF.PAD_MID_EE))
            was = float(np.linalg.norm(legacy - np.asarray(obj)))
            assert was > 0.013, (
                "the control: the legacy offset should miss by ~13.45 mm and "
                "misses by %.3f mm -- if this stops failing, the fix is "
                "measuring nothing" % (was * 1000))


def test_the_two_arms_differ_only_by_their_anchor():
    """The 48.3 mm is a rotation, and this is the number that says so."""
    l_w = np.asarray(CT.PAD_OFFSET_BY_ARM["left"], float)
    r_w = np.asarray(CT.PAD_OFFSET_BY_ARM["right"], float)
    world_gap = float(np.linalg.norm(l_w - r_w))
    l_e = GF.q_matrix(MC.WORKSPACE_ORIENT["left"]).T @ l_w
    r_e = GF.q_matrix(MC.WORKSPACE_ORIENT["right"]).T @ r_w
    ee_gap = float(np.linalg.norm(l_e - r_e))
    assert world_gap > 0.040, world_gap        # the recorded 48.3 mm
    assert ee_gap < 0.0002, (
        "in the EE frame the two arms' pad offsets differ by %.4f mm, which "
        "would mean the hardware really is asymmetric" % (ee_gap * 1000.0))


def test_the_two_coordinate_converters_now_agree():
    """`clip_tasks.ee_for` and `grasp_frames.wrist_for` are the same map now.

    They differed by exactly 13.45 mm along the tool axis, which is what this
    test used to assert. Two converters that disagree by a constant is how a
    task and its own grasp geometry drift apart, and the residual here is the
    0.1 mm grid `ee_for` rounds to and nothing else.
    """
    for arm in ("left", "right"):
        for obj in ([0.56, 0.12, 1.12], [-0.74, 0.23, 1.10],
                    [0.30, 0.40, 1.0]):
            a = np.asarray(CT.ee_for(obj, arm), float)
            b = np.asarray(GF.wrist_for(obj, MC.WORKSPACE_ORIENT[arm]), float)
            gap = float(np.linalg.norm(a - b))
            assert gap < 0.0002, (
                "%s arm at %s: the two converters are %.3f mm apart"
                % (arm, obj, gap * 1000))


def test_axis_and_quaternion_round_trip():
    """A fan member must point where it was asked to point."""
    for elev in (-90.0, -45.0, -5.0, 0.0, 12.5, 30.8):
        for head in (-50.0, 0.0, 25.0):
            if abs(elev) > 89.0 and head != 0.0:
                continue                 # heading is undefined straight down
            q = GF.q_from_axis(GF.axis_for(elev, head))
            e, h = GF.elev_head_of(q)
            assert abs(e - elev) < 1e-6, (elev, head, e)
            assert abs(h - head) < 1e-6, (elev, head, h)


def test_approach_path_comes_in_along_the_axis():
    """The pre-grasp must be exactly `standoff` back along the approach."""
    q = GF.q_from_axis(GF.axis_for(0.0, 0.0))
    pre, ee = GF.approach_path([0.35, 0.30, 1.02], q, 0.10)
    a = GF.tool_axis(q)
    d = np.asarray(ee) - np.asarray(pre)
    assert abs(float(np.linalg.norm(d)) - 0.10) < 1e-9
    assert float(np.dot(d / np.linalg.norm(d), a)) > 1.0 - 1e-9
    # and the pads land ON the object, not the wrist
    pads = np.asarray(ee) + GF.q_matrix(q) @ np.asarray(GF.PAD_MID_EE)
    assert float(np.linalg.norm(pads - np.array([0.35, 0.30, 1.02]))) < 1e-9
