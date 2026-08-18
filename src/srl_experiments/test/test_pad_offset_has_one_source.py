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
constant is now pinned to the MEASUREMENT
(`recordings/baselines/pad_mid_ee.json`, written by
`scripts/measure_pad_mid_ee.py`), and the disagreement with the legacy world
vectors is asserted EXPLICITLY at its measured size, so it is a recorded fact
rather than a silent one. `PAD_OFFSET_BY_ARM` is not moved: T0, T2 and T3
declare their coordinates through it and `clip_scene` draws their objects
through it, so task and picture agree with each other, and moving it is a
re-derivation of three tasks rather than a fix.
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


def test_the_legacy_world_offset_is_longer_and_by_how_much():
    """The disagreement is a recorded number, not a thing to be discovered again.

    Direction agrees -- rotate the world vector back through the arm's own
    anchor and it lands on +z of the EE frame like the measurement does. Only
    the LENGTH differs, by 13.4 to 13.6 mm, and every task that still declares
    its coordinates through `clip_tasks.ee_for` carries that offset between
    where an object is DECLARED and where it is drawn and grasped.
    """
    for arm in ("left", "right"):
        w = np.asarray(CT.PAD_OFFSET_BY_ARM[arm], float)
        in_ee = GF.q_matrix(MC.WORKSPACE_ORIENT[arm]).T @ w
        off_axis = float(np.linalg.norm(in_ee[:2]))
        assert off_axis < 0.0002, (
            "%s arm: the legacy world offset is %.3f mm off the tool axis, so "
            "it disagrees with the measurement in DIRECTION as well as length "
            "and this test's premise no longer holds" % (arm, off_axis * 1000))
        extra_mm = (float(in_ee[2]) - float(GF.PAD_MID_EE[2])) * 1000.0
        assert 13.0 <= extra_mm <= 14.0, (
            "%s arm: the legacy offset is %.2f mm longer than the FK "
            "measurement; it was 13.45 to 13.51 mm when this was measured. "
            "One of the two has been edited." % (arm, extra_mm))


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


def test_wrist_for_and_ee_for_differ_by_exactly_the_measured_error():
    """The two converters, at the anchor, differ by the 13.45 mm and nothing else.

    This test used to assert they were the SAME function at the anchor, and
    they were, because `wrist_for` was built on the magnitude of `ee_for`'s own
    constant. Now that `PAD_MID_EE` is the FK measurement they differ, and the
    useful assertion is that they differ by exactly the known amount, in
    exactly the known direction: any OTHER difference means a coordinate
    converter has been edited.

    The direction matters as much as the size. `ee_for` puts the wrist further
    back along the tool axis, so its fingers stop short of the declared point;
    that is the sign, and it is asserted rather than described.
    """
    for arm in ("left", "right"):
        axis = GF.q_matrix(MC.WORKSPACE_ORIENT[arm])[:, 2]
        for obj in ([0.56, 0.12, 1.12], [-0.74, 0.23, 1.10], [0.30, 0.40, 1.0]):
            a = np.asarray(CT.ee_for(obj, arm), float)      # legacy
            b = np.asarray(GF.wrist_for(obj, MC.WORKSPACE_ORIENT[arm]), float)
            d = a - b
            along = float(np.dot(d, axis))
            perp = float(np.linalg.norm(d - along * axis))
            assert perp < 0.0002, (
                "%s arm at %s: the two converters differ by %.3f mm ACROSS "
                "the tool axis, which is not the recorded error"
                % (arm, obj, perp * 1000))
            assert -0.0140 <= along <= -0.0130, (
                "%s arm at %s: the legacy converter sits %.2f mm along the "
                "tool axis from the measured one; it was -13.45 mm"
                % (arm, obj, along * 1000))


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
