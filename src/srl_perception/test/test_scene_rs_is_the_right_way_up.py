"""The scene RealSense's orientation has ONE owner, and it is measured.

Three places used to decide independently whether the scene RealSense is
mounted upside down: `cv_pickpose_visuals.grab_scene_rs` said yes,
`real_calibration.scene_cameras.SceneRealSense` said yes, and
`srl_realsense_node` said nothing at all -- so the recorded figures and the
live topics disagreed and neither knew.  Measured on 2026-08-30 against the
HD webcam, which watches the same scene and is rotated by nothing, the
camera is the RIGHT WAY UP.

These tests pin the three things that made that defect possible:

  1. the answer comes from one file, not from a default in a script;
  2. an image can never be rotated without its principal point;
  3. the instrument that decides the orientation can FAIL -- it gets a
     deliberately inverted frame wrong-way-up, and it refuses to answer
     about two pictures with nothing in common.
"""
import os
import sys

import numpy as np
import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "scripts"))

cv2 = pytest.importorskip("cv2")
import scene_rs_orientation as orient          # noqa: E402


def test_the_orientation_is_stored_not_hardcoded():
    """It comes from config/, and the file says how it was measured."""
    o = orient.load_orientation()
    assert o["source"].endswith("scene_rs_orientation.json"), \
        "the shipped config file is missing; the orientation fell back to a " \
        "built-in default, which is how this went wrong the first time"
    assert o["rotate180"] is False
    assert o["measured_on"] and o["method"] and o["evidence"], \
        "an orientation with no recorded evidence is an assumption again"


def test_the_intrinsics_cannot_be_left_behind():
    """rotate() returns the image and K together, or neither."""
    W, H = 640, 480
    K = (603.02, 603.13, 318.87, 231.22)
    img = np.zeros((H, W, 3), np.uint8)
    img[:8, :] = 255
    dep = np.zeros((H, W), np.float32)
    dep[:8, :] = 2.5

    c, d, kc, kd = orient.rotate(img, dep, K, K, apply=True)
    assert c[H - 1, 0, 0] == 255 and c[0, 0, 0] == 0
    assert d[H - 1, 0] == pytest.approx(2.5) and d[0, 0] == 0.0
    assert kc[2] == pytest.approx(W - 1 - K[2])
    assert kc[3] == pytest.approx(H - 1 - K[3])
    assert kd == kc
    assert (kc[0], kc[1]) == (K[0], K[1]), "a rotation is not a zoom"


def test_the_rotation_is_an_involution():
    """--repair undoes a baked-in rotation by applying the same map again."""
    W, H = 424, 240
    K = (380.0, 381.0, 210.4, 118.9)
    rng = np.random.RandomState(0)
    img = rng.randint(0, 255, (H, W, 3), np.uint8)
    dep = rng.rand(H, W).astype(np.float32)
    c, d, kc, _ = orient.rotate(img, dep, K, K, apply=True)
    c2, d2, kc2, _ = orient.rotate(c, d, kc, kc, apply=True)
    assert (c2 == img).all() and (d2 == dep).all()
    assert kc2 == pytest.approx(K)


def _scene(bright_band=(160, 360)):
    """A picture with an unambiguous top and bottom."""
    ref = np.zeros((480, 640, 3), np.uint8)
    ref[:bright_band[0]] = 20
    ref[bright_band[0]:bright_band[1]] = 200
    ref[bright_band[1]:] = 60
    cv2.circle(ref, (320, 250), 60, (255, 255, 255), -1)
    return ref


def test_the_instrument_gets_a_known_upright_frame_right():
    ref = _scene()
    v, a, b = orient.verdict(cv2.resize(ref, (424, 240)), ref)
    assert v == "UPRIGHT" and a > b


def test_the_instrument_can_fail_on_a_deliberately_inverted_frame():
    """A check that only ever answers one way is not a check."""
    ref = _scene()
    flipped = cv2.rotate(cv2.resize(ref, (424, 240)), cv2.ROTATE_180)
    v, a, b = orient.verdict(flipped, ref)
    assert v == "UPSIDE_DOWN" and b > a


def test_two_unrelated_pictures_decide_nothing():
    """No reference in frame must read UNDECIDED, never a confident guess."""
    ref = _scene()
    noise = np.random.RandomState(1).randint(0, 255, (240, 320, 3), np.uint8)
    v, a, b = orient.verdict(noise, ref)
    assert v == "UNDECIDED"
    assert abs(a - b) < orient.MIN_MARGIN


def test_the_recorded_sessions_are_the_right_way_up():
    """The scene_rs frames the thesis figures are drawn from, as they stand.

    Skipped where the recordings are not checked out; where they are, every
    run that has both cameras must read UPRIGHT.  This is the test that
    would have caught the original defect on the day it was recorded.
    """
    import glob
    runs = sorted(glob.glob(os.path.join(WS, "recordings", "vision_thesis",
                                         "*", "raw")))
    pairs = [r for r in runs
             if os.path.exists(os.path.join(r, "scene_rs", "colour.png"))
             and os.path.exists(os.path.join(r, "scene_hd", "colour.png"))]
    if not pairs:
        pytest.skip("no vision_thesis run carries both cameras")
    bad = []
    for r in pairs:
        R = cv2.imread(os.path.join(r, "scene_rs", "colour.png"))
        H = cv2.imread(os.path.join(r, "scene_hd", "colour.png"))
        v, a, b = orient.verdict(R, H)
        if v != "UPRIGHT":
            bad.append("%s: %s (as-saved %+.3f, rotated %+.3f)"
                       % (r, v, a, b))
    assert not bad, "upside-down scene_rs recordings:\n  " + "\n  ".join(bad)
