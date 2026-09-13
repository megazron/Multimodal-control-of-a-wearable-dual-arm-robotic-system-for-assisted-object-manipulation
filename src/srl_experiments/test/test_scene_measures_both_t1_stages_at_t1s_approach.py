"""THE SCENE MUST MEASURE BOTH T1 STAGES AT T1'S OWN APPROACH, NOT THE ANCHOR.

WHY THIS FILE EXISTS. `clip_scene.Scene._pad_offset()` decides where the scene
believes a graspable object is: it is the wrist-to-pad vector, in WORLD, so it
rotates with the hand. T1 does not command the anchor -- it carries its own
approach (`t1_task.APPROACH`, elevation -10 deg, heading -50 inboard) -- and
the function's own docstring says that leaving it at the anchor draws the whole
scene 111.8 mm out.

The guard was written as `if self.task == "t1"`. Stage 2 was then rebuilt on
stage 1's geometry and given stage 1's approach -- `msc_clip_tasks` gives t1
and t1s2 the SAME `orient` -- and an exact match on "t1" is not a match on
"t1s2", so stage 2 silently took the anchor branch.

WHAT IT COST, measured on the first stage-2 clip recorded under 06_full_autonomy:
the runner planned four picks from the camera, commanded all four and all four
places, and BOTH grippers closed (grip trace: left closed 189.7-205.8 s, right
210.1-214.0 s). The scene recorded TWO grasps and filed the clip
"2 OBJECT(S) NEVER MOVED". The arm was fine. The instrument was measuring at
the wrong orientation, and the resulting POSITION error is 99.1 mm on the left
arm and 83.3 mm on the right, against a 30 mm capture gate -- two of the four
happened to land inside it anyway, which is why the clip failed partially
rather than completely and looked like a flaky task instead of a broken
measurement.

Two lengths are easy to confuse here and only one is the error. The anchor
offset is 111.8 mm long and T1's is 98.3 mm; that 111.8 is what separates a
cube's DECLARED position from where the scene reported it. The distance
between the two offset VECTORS -- which is how far the scene is wrong about
the object -- is 99.1 mm (left) and 83.3 mm (right); the mounts are mirrored
in position but differ by 168 deg of roll, so the two arms are not symmetric
here and one number would not have covered both.

This is docs/ENGINEERING_LOG.md's "everything matches" row turned inside out: there the
danger is a prefix matching too much (`t1s2`.startswith(`t1`)), here it is an
equality matching too little. Both come from testing a task key with the wrong
operator.

WHAT IS CHECKED: the offset the scene would use for each stage, against the
offset derived from that task's own commanded orientation. Constructed ground
truth -- it is a quaternion rotating a measured vector, which is arithmetic --
and the anchor value is computed too, so the test states the size of the error
it is preventing rather than asserting an opaque triple.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for _p in (os.path.join(ROOT, "scripts"),
           os.path.join(ROOT, "src/srl_experiments/experiments/abc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


@pytest.fixture(scope="module")
def CS():
    return pytest.importorskip("clip_scene")


@pytest.fixture(scope="module")
def T1():
    import t1_task
    return t1_task


def _offset(CS, task, arm):
    """What the scene would use, driven through the REAL method."""
    class _Fake:
        pass
    f = _Fake()
    f.task = task
    f.seed = 0
    f.buf = None
    f.PAD_DEPTH_M = 0.0
    return np.asarray(CS.Scene._pad_offset(f, arm), dtype=float)


def _t1_truth(T1, arm):
    """T1's own approach, at the opening a 40 mm cube needs.

    `PAD_MID_EE` is the WIDE-OPEN hand and this used it. The Robotiq's fingers
    swing on a four-bar, so the pad sits 0.09833 m from the wrist open and
    0.10976 m closed on a 40 mm cube; `t1_task.ee_for` and
    `clip_scene._pad_offset` both moved onto the 40 mm value on 2026-08-23,
    and a truth function left on the open one would have failed them for
    agreeing with each other.
    """
    import grasp_frames as GF
    return np.asarray(
        GF.q_matrix(T1.APPROACH[arm]) @ np.asarray(
            GF.pad_mid_ee_for(T1.CUBE_M * 1000.0, arm)), dtype=float)


@pytest.mark.parametrize("task", ["t1", "t1s2"])
@pytest.mark.parametrize("arm", ["left", "right"])
def test_both_stages_use_t1s_own_approach(CS, T1, task, arm):
    got = _offset(CS, task, arm)
    want = _t1_truth(T1, arm)
    err = float(np.linalg.norm(got - want))
    assert err < 1e-9, (
        "%s/%s: the scene measures at %s, T1 commands %s -- %.1f mm apart"
        % (task, arm, got.round(4), want.round(4), err * 1000.0))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_anchor_would_have_been_wrong_by_82_to_98_mm(CS, T1, arm):
    """The error this guard prevents, stated as a number.

    If this ever reads ~0 the two orientations have converged and the guard is
    no longer load-bearing -- which is worth knowing, because the guard would
    then be silently untested.

    IT WAS 99.1 / 83.3 mm UNTIL 2026-08-23, and it moved twice that day for
    reasons that are not this guard: `clip_tasks.PAD_OFFSET_BY_ARM` stopped
    being a hand-recorded world vector and became `PAD_MID_EE` rotated by the
    anchor, and T1's own offset moved to the opening a 40 mm cube needs. The
    anchor and T1's approach are still 82-98 mm apart, which is what this test
    is about.
    """
    import clip_tasks as CT
    anchor = np.asarray(CT.PAD_OFFSET_BY_ARM[arm], dtype=float)
    err_mm = float(np.linalg.norm(anchor - _t1_truth(T1, arm))) * 1000.0
    want = {"left": 97.9, "right": 82.2}[arm]
    assert abs(err_mm - want) < 1.0, (
        "expected the anchor to sit ~%.1f mm from T1's approach on the %s "
        "arm, got %.1f mm" % (want, arm, err_mm))
    legacy = np.asarray(CT.LEGACY_PAD_OFFSET_BY_ARM[arm], dtype=float)
    was_mm = float(np.linalg.norm(legacy - _t1_truth(T1, arm))) * 1000.0
    assert abs(was_mm - {"left": 103.7, "right": 86.9}[arm]) < 1.0, (
        "the legacy world offset now sits %.1f mm from T1's approach; it was "
        "103.7 / 86.9 when this was measured" % was_mm)
    assert err_mm > 30.0, (
        "the anchor is now inside the 30 mm capture gate, so this guard no "
        "longer protects anything and the test above is not exercising it")


def test_a_task_that_does_NOT_command_t1s_approach_still_gets_the_anchor(CS):
    """The guard must not have become "always T1's approach".

    A fix that returned T1's offset for everything would pass every assertion
    above and break T0, T2 and T3, which DO command the anchor.
    """
    import clip_tasks as CT
    for task in ("t0", "t2", "t3"):
        got = _offset(CS, task, "left")
        want = np.asarray(CT.PAD_OFFSET_BY_ARM["left"], dtype=float)
        assert np.allclose(got, want), (
            "%s must be measured at the anchor, got %s" % (task, got.round(4)))
