"""The picture and the path must place an object in the SAME spot.

`clip_tasks.ee_for` / `t1_task.ee_for` turn a declared OBJECT position into
the WRIST pose that puts the finger pads on it, by subtracting a wrist-to-pad
offset. `clip_scene` then DRAWS that object at `wrist + the same offset`. When
the two offsets are the same vector the object appears exactly where the task
declared it, and the grasp lands. When they are not, the arm is fine and the
scene is drawing the object somewhere the hand is not — and the clip is filed
"N OBJECT(S) NEVER MOVED".

THAT SENTENCE HAS NOW BEEN PRODUCED THREE TIMES BY THIS ONE DISAGREEMENT:

    49 mm     the scene had a per-arm offset and the task had one number
    111.8 mm  the scene measured stage 2 at the ANCHOR while it commands
              T1's own approach
    11.43 mm  `t1_task.ee_for` moved to the gripper opening a 40 mm cube
              needs (2026-08-23) and `clip_scene._pad_offset` stayed on the
              WIDE-OPEN value

Every time the run exited 0 and the gripper closed. Nothing downstream could
tell the difference, because the scene is also what SCORES the grasp: it
measures the pads against the object IT drew, so a consistent-but-wrong pair
scores perfectly and a task that moved one side scores a miss.

So the invariant is asserted directly, per task, per arm, in the units it
matters in: how far the drawn object is from the declared one. No ROS.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for _p in (os.path.join(ROOT, "scripts"),
           os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc"),
           os.path.join(ROOT, "src", "srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import clip_scene as CS                                        # noqa: E402
import clip_tasks as CT                                        # noqa: E402
import grasp_frames as GF                                      # noqa: E402
import t1_task as T1                                           # noqa: E402

# What a grasp has to hit. The disagreement must be a rounding artefact, not a
# fraction of the gate: a scene 40% of the way to the gate leaves the task no
# budget for the arm.
GATE_MM = 30.0
TOL_MM = 0.2


def _scene_offset(task, arm):
    f = CS.Scene.__new__(CS.Scene)
    f.task, f.seed, f.buf, f.PAD_DEPTH_M = task, 0, None, 0.0
    return np.asarray(CS.Scene._pad_offset(f, arm), float)


@pytest.mark.parametrize("arm", ["left", "right"])
@pytest.mark.parametrize("task", ["t1", "t1s2"])
def test_T1_draws_its_cubes_where_it_declares_them(task, arm):
    """T1 commands its OWN approach and grips a 40 mm cube."""
    off = _scene_offset(task, arm)
    for cx, cy in T1.T1_CUBES:
        obj = np.array([cx, cy, T1.T1_Z])
        drawn = np.asarray(T1.ee_for(list(obj), arm), float) + off
        err = float(np.linalg.norm(drawn - obj)) * 1000.0
        assert err < TOL_MM, (
            "%s/%s: a cube declared at %s is drawn at %s, %.2f mm away. The "
            "scene and the task are using different wrist-to-pad offsets."
            % (task, arm, obj.round(4), drawn.round(4), err))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_other_tasks_draw_their_objects_where_they_declare_them(arm):
    """T0, T2 and T3 go through `clip_tasks.ee_for` at the pinned anchor."""
    off = _scene_offset("t2", arm)
    for obj in ([0.42, 0.45, 1.27], [-0.30, 0.50, 1.10], [0.55, 0.12, 0.98]):
        drawn = np.asarray(CT.ee_for(obj, arm), float) + off
        err = float(np.linalg.norm(drawn - np.asarray(obj))) * 1000.0
        assert err < TOL_MM, (
            "%s arm: an object declared at %s is drawn %.2f mm away"
            % (arm, obj, err))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_T1s_scene_offset_is_at_the_gripping_opening_not_the_open_hand(arm):
    """The specific mistake of 2026-08-23, as its own assertion.

    A CHECK THAT CANNOT FAIL IS NOT A CHECK: this also requires the OPEN-hand
    offset to be measurably different, so it cannot pass by the two values
    having quietly converged.
    """
    off = _scene_offset("t1", arm)
    at_grip = GF.q_matrix(T1.APPROACH[arm]) @ np.asarray(
        GF.pad_mid_ee_for(T1.CUBE_M * 1000.0, arm))
    at_open = GF.q_matrix(T1.APPROACH[arm]) @ np.asarray(GF.PAD_MID_EE)
    assert float(np.linalg.norm(off - at_grip)) * 1000 < TOL_MM
    gap = float(np.linalg.norm(at_grip - at_open)) * 1000
    assert gap == pytest.approx(11.43, abs=0.05), (
        "the open-hand and 40 mm offsets differ by %.2f mm; they were "
        "11.43 mm apart, and if they converge this test stops proving "
        "anything" % gap)


def test_a_disagreement_of_a_third_of_the_gate_would_be_caught():
    """The control. Feed the check the offset it USED to have and require it
    to fail — otherwise the tolerance is not binding anything."""
    arm = "left"
    wrong = GF.q_matrix(T1.APPROACH[arm]) @ np.asarray(GF.PAD_MID_EE)
    obj = np.array([T1.T1_CUBES[0][0], T1.T1_CUBES[0][1], T1.T1_Z])
    drawn = np.asarray(T1.ee_for(list(obj), arm), float) + wrong
    err = float(np.linalg.norm(drawn - obj)) * 1000.0
    assert err > TOL_MM, "the control does not fail; the tolerance is inert"
    assert err < GATE_MM, (
        "the historical error was 11.43 mm, inside the 30 mm gate -- which is "
        "why it scored as a flaky grasp rather than an obvious break")
