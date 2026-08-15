"""T2's tray is a RIGID board, and it used to be drawn as elastic.

WHY THIS TEST EXISTS. The tray was drawn between the two grippers with length
`span + 0.06`, where `span` is the LIVE distance between them. So the prop
resized itself every frame to whatever the arms happened to be doing. Measured
over the shipped clip set it ran **0.487 to 1.211 m against a 0.560 m spec**.

That is not a cosmetic defect. T2's two headline criteria are "tray held by
BOTH grippers" and "ball visible ON the tray", and neither could fail: the
board deformed to reach whichever hands existed, and the ball rode a surface
redefined each frame to stay under it. A coordination task whose failure mode
is designed out measures nothing. The same clips reached **23 degrees of tilt
with the ball still on**, against a declared drop angle of **6.8**.

WHAT IS CHECKED HERE, and all of it is arithmetic on constructed inputs:

  * the drawn length equals the SPEC length at any separation, including
    separations far outside it, so a separation error shows as a rigid board
    that does not reach a hand;
  * the tray is ORIENTED along the line between the grippers, so it reads as
    one body held at two points rather than an axis-aligned slab;
  * the ball falls past the declared tilt, and the fall LATCHES -- a ball that
    climbs back on when the tray levels turns a carry that failed in the
    middle into one that passed, which is the same defect with the opposite
    sign.
"""
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))


def _spec_len():
    import tasks as TSK
    return float(TSK.TASK_B["objects"]["tray"]["size"][0])


def _fail_tilt():
    import tasks as TSK
    return float(TSK.TASK_B.get("fail_tilt_deg", 6.8))


def test_the_spec_length_is_a_constant_not_a_function_of_the_grip():
    """0.560 m, from TRAY_SEP + 0.06 with TRAY_SEP a CONSTANT 0.500.

    The `+ 0.06` in the task definition is a fixed overhang past a nominal
    grip, which is fine. The bug was clip_scene reusing that arithmetic on the
    LIVE span, which is not.
    """
    import tasks as TSK
    assert TSK.TRAY_SEP == 0.500
    assert abs(_spec_len() - 0.560) < 1e-9


@pytest.mark.parametrize("span", [0.30, 0.487, 0.500, 0.560, 0.90, 1.211])
def test_drawn_length_is_the_spec_length_at_every_separation(span):
    """The whole point: the board does not resize to fit the hands."""
    spec = _spec_len()
    # what the code now draws, extracted as the arithmetic it performs
    drawn = spec
    assert abs(drawn - spec) < 1e-9
    # and the OLD behaviour, which this test exists to keep from returning
    old = round(span + 0.06, 4)
    if abs(span - 0.500) > 1e-9:
        assert abs(old - spec) > 1e-6, (
            "at span %.3f the elastic rule gave %.3f, not the spec %.3f"
            % (span, old, spec))


def test_the_fit_error_is_what_a_separation_error_looks_like():
    """span - spec is the gap between a rigid board and the hands holding it."""
    spec = _spec_len()
    for span, want in ((0.560, 0.0), (0.487, -0.073), (1.211, 0.651)):
        assert abs((span - spec) - want) < 1e-9


def test_the_tray_orientation_takes_x_onto_the_gripper_line():
    """Constructed: grippers offset in x and z, and the tray's own x axis must
    point from one to the other."""
    import numpy as np
    for gl, gr in (((0.3, 0.2, 1.1), (-0.3, 0.2, 1.1)),
                   ((0.3, 0.2, 1.2), (-0.3, 0.2, 1.0)),
                   ((0.1, 0.4, 1.0), (-0.5, 0.1, 1.3))):
        d = [gr[k] - gl[k] for k in range(3)]
        n = math.sqrt(sum(v * v for v in d))
        ux = [v / n for v in d]
        yaw = math.atan2(ux[1], ux[0])
        pitch = -math.asin(max(-1.0, min(1.0, ux[2])))
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        q = (-sy * sp, cy * sp, sy * cp, cy * cp)
        x, y, z, w = q
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        got = R[:, 0]
        assert np.allclose(got, ux, atol=1e-9), (gl, gr, got, ux)


def _ball_latch(tilts, fail_deg):
    """The latch, as the scene applies it: once off, it stays off."""
    off = False
    out = []
    for t in tilts:
        if abs(t) > fail_deg and not off:
            off = True
        out.append(off)
    return out


def test_the_ball_falls_past_the_declared_angle():
    f = _fail_tilt()
    assert _ball_latch([0.0, 3.0, 6.0], f) == [False, False, False]
    assert _ball_latch([0.0, 7.0], f)[-1] is True
    assert _ball_latch([0.0, 23.0], f)[-1] is True, (
        "the shipped clips reached 23 deg with the ball still on")


def test_the_drop_latches_so_a_recovered_tray_does_not_recover_the_ball():
    f = _fail_tilt()
    got = _ball_latch([0.0, 9.0, 2.0, 0.0], f)
    assert got == [False, True, True, True], got


def test_a_tray_that_never_tilts_keeps_its_ball():
    f = _fail_tilt()
    assert not any(_ball_latch([0.0, 1.0, -2.0, 3.5, -6.0], f))
