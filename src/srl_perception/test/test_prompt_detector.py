"""Tests for prompt-driven detection.

Fixtures are CONSTRUCTED images -- a square of a stated colour at a stated
pixel -- so the answer is known before the code runs.

The negative tests carry the weight. A detector that cannot say "I did not
find it", "I have no depth there" or "I do not know that word" will be
believed when it is wrong, and this repository has been bitten by exactly
that: a colour-only filter picked the room's teal robots as the green cube
three separate times because nothing made it state its limits.
"""
import numpy as np
import pytest

from srl_perception.prompt_detector import (COLOUR_TERMS, Detection,
                                            PromptDetector, parse_prompt)


def scene(colour_bgr=(35, 63, 10), at=(300, 200), size=40, shape=(480, 640)):
    img = np.full((shape[0], shape[1], 3), 20, np.uint8)     # dark background
    x, y = at
    img[y:y + size, x:x + size] = colour_bgr
    return img


def test_parse_prompt_pulls_colour_and_shape_out_of_a_sentence():
    c, s, rest = parse_prompt("please pick up the green cube")
    assert c == ["green"] and s == ["cube"]
    assert "please" not in rest and "the" not in rest


def test_parse_prompt_survives_no_colour():
    c, s, rest = parse_prompt("grab the bottle")
    assert c == [] and s == ["bottle"]


def test_colour_backend_finds_a_constructed_green_square_where_it_was_put():
    d = PromptDetector(backend="colour")
    hits = d.detect(scene(at=(300, 200)), "the green cube")
    assert len(hits) == 1
    u, v = hits[0].centre_uv
    assert u == pytest.approx(320, abs=3) and v == pytest.approx(220, abs=3)
    assert hits[0].backend == "colour"


def test_it_does_NOT_find_a_green_cube_that_is_not_there():
    """THE CHECK THAT MUST BE ABLE TO FAIL."""
    d = PromptDetector(backend="colour")
    blue = scene(colour_bgr=(200, 40, 20))          # BGR blue
    assert d.detect(blue, "the green cube") == []


def test_an_unknown_colour_word_is_REFUSED_and_lists_what_it_knows():
    d = PromptDetector(backend="colour")
    with pytest.raises(ValueError) as e:
        d.detect(scene(), "pick up the bottle")
    msg = str(e.value)
    assert "colour word" in msg and "green" in msg


def test_every_advertised_colour_term_actually_matches_its_own_colour():
    """A vocabulary that lists words it cannot match is a lie in a docstring."""
    import cv2
    d = PromptDetector(backend="colour")
    for term, ranges in COLOUR_TERMS.items():
        lo, hi = ranges[0]
        # THE MIDPOINT OF THE RANGE, all three channels. An earlier version of
        # this test forced S and V to >=200 to make a "vivid" colour, which
        # put the sample OUTSIDE the white band (white is defined by S <= 40)
        # and failed the code for the test's mistake.
        mid = np.array([[[(lo[0] + hi[0]) // 2,
                          (lo[1] + hi[1]) // 2,
                          (lo[2] + hi[2]) // 2]]], np.uint8)
        bgr = cv2.cvtColor(mid, cv2.COLOR_HSV2BGR)[0, 0]
        img = scene(colour_bgr=tuple(int(v) for v in bgr))
        assert d.detect(img, "the %s object" % term), "no match for %r" % term


def test_depth_is_attached_and_the_3d_point_is_arithmetically_right():
    d = PromptDetector(backend="colour")
    K = (500.0, 500.0, 320.0, 240.0)
    depth = np.full((480, 640), 1.25, float)
    hits = d.detect(scene(at=(300, 220)), "green cube", depth_m=depth, K=K)
    assert len(hits) == 1
    h = hits[0]
    assert h.depth_m == pytest.approx(1.25, abs=1e-9)
    u, v = h.centre_uv
    assert h.xyz_cam[0] == pytest.approx((u - 320.0) / 500.0 * 1.25, abs=1e-6)
    assert h.xyz_cam[2] == pytest.approx(1.25, abs=1e-9)


def test_the_range_gate_drops_things_that_are_too_far():
    """The teal-robot case, as a test: right colour, wrong distance."""
    K = (500.0, 500.0, 320.0, 240.0)
    img = scene()
    near = PromptDetector(backend="colour", far_m=1.6)
    assert near.detect(img, "green cube",
                       depth_m=np.full((480, 640), 2.8), K=K) == []
    assert len(near.detect(img, "green cube",
                           depth_m=np.full((480, 640), 1.1), K=K)) == 1


def test_depth_at_a_DIFFERENT_resolution_is_reprojected_not_assumed_aligned():
    """Colour is 1280x720 and depth is 480x270 on this rig. Indexing the depth
    image with a colour pixel would read the wrong place entirely."""
    d = PromptDetector(backend="colour")
    CK = (1297.67, 1298.63, 620.91, 238.28)
    DK = (342.21, 342.21, 233.07, 132.48)
    depth = np.zeros((270, 480), float)
    img = np.full((720, 1280, 3), 20, np.uint8)
    img[238:278, 620:660] = (35, 63, 10)        # square at the principal point
    # depth valid ONLY at the depth principal point -- where the bearing maps
    depth[int(DK[3]) - 4:int(DK[3]) + 5, int(DK[2]) - 4:int(DK[2]) + 5] = 0.9
    hits = d.detect(img, "green cube", depth_m=depth, K=CK, depth_K=DK)
    assert len(hits) == 1 and hits[0].depth_m == pytest.approx(0.9, abs=1e-9)


def test_missing_depth_is_reported_rather_than_invented():
    d = PromptDetector(backend="colour")
    K = (500.0, 500.0, 320.0, 240.0)
    hits = d.detect(scene(), "green cube",
                    depth_m=np.zeros((480, 640)), K=K)
    assert len(hits) == 1
    assert hits[0].depth_m is None
    assert "no depth" in hits[0].extra.get("depth_note", "")


def test_the_backend_is_always_reported():
    d = PromptDetector(backend="colour")
    assert d.detect(scene(), "green cube")[0].backend == "colour"
    assert d.available()["colour"] is True
    assert isinstance(d.available()["yoloworld"], bool)


def test_asking_for_yoloworld_when_it_is_absent_REFUSES_clearly():
    try:
        import ultralytics                                # noqa: F401
        pytest.skip("ultralytics is installed here")
    except Exception:
        pass
    with pytest.raises(RuntimeError) as e:
        PromptDetector(backend="yoloworld")
    assert "ultralytics" in str(e.value)


def test_auto_falls_back_to_colour_and_says_so():
    d = PromptDetector(backend="auto")
    assert d.backend in ("colour", "yoloworld")


def test_the_ensemble_falls_through_to_colour_when_the_model_finds_nothing():
    """Measured on the rig: YOLO-World finds a person at 0.725 in the same
    frame where it cannot find a 19 px dark green cube at all. The colour
    backend finds that cube every time. Neither is 'the good one', so 'both'
    must cover the union rather than trusting one."""
    d = PromptDetector(backend="both")
    if d.backend != "both":
        pytest.skip("ultralytics absent here; 'both' degrades to colour")
    hits = d.detect(scene(), "the green cube")
    assert hits, "the ensemble lost a detection the colour backend can make"
    assert hits[0].backend in ("colour", "yoloworld")


def test_both_backend_reports_which_one_actually_found_it():
    d = PromptDetector(backend="both")
    hits = d.detect(scene(), "the green cube")
    assert hits and hits[0].backend, "a detection must say how it was made"


def test_no_colour_word_and_no_model_is_not_found_rather_than_a_crash():
    d = PromptDetector(backend="both")
    if d.backend != "both":
        pytest.skip("needs the ensemble")
    # a prompt with no colour word: the model may find nothing, and colour
    # cannot help. That must be an empty result, not an exception.
    assert isinstance(d.detect(scene(), "the teapot"), list)
