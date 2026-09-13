#!/usr/bin/env python3
"""KNOWN ANSWERS FOR WHICH OBJECTS A CLIP IS REQUIRED TO SHOW.

THE COLLISION. Two task families in this repository both have a `t2` and a
`t3`, and they are different tasks with different objects:

    archived nine-task set   t2 = teal container + orange block
                             t3 = tan tray + yellow ball
    MSc set                  T2 = bimanual tray carry
                             T3 = circuit box + multimeter

`verify_rviz_clips` parses <mode>/<task>/<scenario> and lowercases the task,
so an MSc T2 clip was looked up as "t2" and checked for a TEAL CONTAINER --
an object that is not in that scene and never could be. Every symptom of it
looks like a real object failure, which is the substring/prefix family from
docs/ENGINEERING_LOG.md's instrument table.

The two trees are shaped differently and that is what `requirements_for`
reads: the MSc and A/B/C sweeps write <mode>/<task>/..., and a mode is always
NN_name.

THE FLOORS ARE THE OTHER HALF and are asserted here because they were
measured, not chosen. `_tan` fires on the mannequin's SKIN at 288-352 px on a
frame with no tray in it, so the archived floor of 120 would pass every MSc
clip whether the tray rendered or not.
"""

import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))


def _v():
    try:
        import verify_rviz_clips
    except ImportError as e:                                # pragma: no cover
        pytest.skip("verify_rviz_clips needs numpy/PIL (%s)" % e)
    return verify_rviz_clips


# ---- the collision itself ------------------------------------------------
def test_msc_T2_is_NOT_checked_for_the_archived_sets_container():
    """The bug, stated as a test. This is what used to happen."""
    want, _ = _v().requirements_for("01_master_teleop", "t2")
    assert want is not None
    assert "container_teal" not in want and "block_orange" not in want
    assert want == ["msc_tray_tan", "msc_ball_yellow"]


def test_msc_T3_is_NOT_checked_for_the_archived_sets_tray():
    want, _ = _v().requirements_for("06_full_autonomy", "t3")
    assert want == ["msc_green", "msc_meter_yellow"]


def test_the_ARCHIVED_tree_still_gets_the_archived_requirements():
    """The fix must not break the set it is not about.

    The legacy tree has the TASK at the top level, so parts[0] is `t3`, not a
    mode. That must still resolve to the nine-task set's tray and ball.
    """
    want, _ = _v().requirements_for("t3", "s1")
    assert want is None            # `s1` is a scenario, not a task
    want, _ = _v().requirements_for(None, "t3")
    assert want == ["tray_tan", "ball_yellow"]


# ---- unknown is not empty ------------------------------------------------
def test_an_unknown_MSc_task_is_None_and_NOT_an_empty_list():
    """0-of-0 reads exactly like a clean pass, which is the whole problem.

    verify_clip() refuses a None by saying nothing was checked. Handing it []
    instead would silently pass the clip.
    """
    want, _ = _v().requirements_for("01_master_teleop", "t7")
    assert want is None


def test_a_demo_routine_requires_nothing_ON_PURPOSE():
    """[] here is a MEASUREMENT, not an accident: the routines carry no object.

    This is the by_design distinction -- an absence that is declared is not
    the same as an absence nobody looked for -- so d1 must be [] and not None.
    """
    want, _ = _v().requirements_for("06_full_autonomy", "d1")
    assert want == []


# ---- the floors, and why they are not one number -------------------------
def test_the_tan_floor_clears_the_MANNEQUINS_SKIN():
    """Measured: skin alone reads 288-352 px; the tray reads 753-889."""
    v = _v()
    _, floors = v.requirements_for("01_master_teleop", "t2")
    assert floors["msc_tray_tan"] > 352, "would pass on the wearer alone"
    assert floors["msc_tray_tan"] < 753, "would reject a visible tray"


def test_yellow_cannot_be_one_floor_for_the_ball_and_the_meter():
    """The ball is ten times the meter's pixel count.

    Ball 515-963 px, meter 75-83, and the yellow marking outline leaks 21-30
    on a T1 frame. One floor cannot clear the leak and keep the meter unless
    it is set for the meter -- at which point it is far below the ball's own
    noise. Two names, two floors.
    """
    v = _v()
    _, floors = v.requirements_for("01_master_teleop", "t2")
    ball, meter = floors["msc_ball_yellow"], floors["msc_meter_yellow"]
    assert ball > meter
    assert meter > 30, "would fire on T1's yellow marking outline"
    assert meter < 75, "would reject a visible multimeter"


def test_the_blue_floor_keeps_T0s_SMALL_SPHERE():
    """23-24 px is a real object, not noise: T0's blue sphere.

    The wearer's torso is blue and reads 0 px, so there is no leak to clear
    and the floor must not be raised past the smallest true object.
    """
    v = _v()
    _, floors = v.requirements_for("01_master_teleop", "t0")
    assert floors["msc_blue"] < 78, "would reject T1's cubes at 78 px"


# ---- the detector itself -------------------------------------------------
def test_the_blue_detector_fires_on_T1_BLUE_and_not_on_the_WEARER():
    """Constructed colours, not rendered ones: clip_scene.BLUE against the
    mannequin's torso. Both are written down here, so the answer is known."""
    import numpy as np
    v = _v()
    def patch(rgb):
        return np.full((10, 10, 3), rgb, dtype=int)
    cube = patch([int(0.10 * 255), int(0.30 * 255), int(0.90 * 255)])
    assert v._blue(cube).sum() == 100
    # The torso, read off a rendered frame rather than off its rgba, because
    # what matters is what the detector sees.
    torso = patch([45, 62, 120])
    assert v._blue(torso).sum() == 0
    white_table = patch([240, 240, 242])
    assert v._blue(white_table).sum() == 0
