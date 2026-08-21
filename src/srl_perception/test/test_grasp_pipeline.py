"""End-to-end pipeline tests on CONSTRUCTED RGB-D scenes.

A synthetic cube of a known size at a known distance, rendered as a colour
square plus a depth patch -- arithmetic, not a render, so the right answer is
known in advance.

Each stage's REFUSAL is tested by constructing the input that should trigger
it. A pipeline whose failure paths are untested will fail silently on the day
it matters, and the failure will look like a bad grasp rather than a missing
measurement.
"""
import numpy as np
import pytest

from srl_perception.grasp_pipeline import PlanFailure, plan_grasp
from srl_perception.prompt_detector import PromptDetector

CK = (600.0, 600.0, 320.0, 240.0)
DK = (600.0, 600.0, 320.0, 240.0)      # same K -> pixels map 1:1, keeps it readable


def rgbd_cube(size_m=0.05, z=0.60, at=(320, 240), shape=(480, 640),
              colour=(35, 63, 10)):
    """A cube of `size_m` at range `z`, as colour + depth."""
    px = int(round(size_m * CK[0] / z))            # its size in pixels
    img = np.full((shape[0], shape[1], 3), 20, np.uint8)
    dep = np.zeros(shape, np.float32)
    u, v = at
    x0, y0 = u - px // 2, v - px // 2
    img[y0:y0 + px, x0:x0 + px] = colour
    dep[y0:y0 + px, x0:x0 + px] = z
    return img, dep, px


def test_a_known_cube_is_found_and_placed_at_the_right_range():
    img, dep, _ = rgbd_cube(0.05, 0.60)
    g = plan_grasp(img, dep, "the green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert g["detection"]["depth_m"] == pytest.approx(0.60, abs=1e-6)
    # centred in the image at the principal point -> straight ahead
    assert abs(g["centre"][0]) < 5e-3 and abs(g["centre"][1]) < 5e-3
    assert g["centre"][2] == pytest.approx(0.60, abs=5e-3)


def test_the_grasp_is_perpendicular_and_the_pregrasp_is_behind_it():
    img, dep, _ = rgbd_cube(0.05, 0.60)
    g = plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"), standoff_m=0.10)
    assert abs(float(np.dot(g["close_axis"], g["approach"]))) < 1e-6
    d = g["pregrasp"] - g["centre"]
    assert float(np.linalg.norm(d)) == pytest.approx(0.10, abs=1e-9)
    assert float(np.dot(d, g["approach"])) < 0


def test_the_camera_pose_actually_moves_the_answer():
    """A grasp reported in the robot frame must depend on where the camera is,
    or it is being reported in the camera frame under a robot-frame name."""
    img, dep, _ = rgbd_cube(0.05, 0.60)
    kw = dict(detector=PromptDetector(backend="colour"))
    a = plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK, **kw)
    # a REALISTIC camera move: the plausibility gate rejects anything solving
    # outside 0.30-2.00 m, so a 3 m offset would (correctly) be refused.
    off = np.array([0.30, 0.20, 0.40])
    b = plan_grasp(img, dep, "green cube", off, np.eye(3), CK, DK, **kw)
    assert np.allclose(b["centre"] - a["centre"], off, atol=1e-9)


def test_detect_stage_REFUSES_when_the_object_is_not_there():
    img, dep, _ = rgbd_cube(colour=(200, 40, 20))       # blue cube
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, dep, "the green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage == "detect"


def test_depth_stage_REFUSES_when_there_is_no_depth_return():
    img, dep, _ = rgbd_cube()
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, np.zeros_like(dep), "green cube", np.zeros(3),
                   np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage == "depth"
    assert "direction and not a position" in e.value.reason


def test_grasp_stage_REFUSES_an_object_too_wide_for_the_jaw():
    img, dep, _ = rgbd_cube(size_m=0.25, z=0.80)
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage == "grasp"
    assert "85" in e.value.reason


def test_every_refusal_names_its_stage_and_carries_evidence():
    img, dep, _ = rgbd_cube()
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, np.zeros_like(dep), "green cube", np.zeros(3),
                   np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage and e.value.reason
    assert e.value.evidence


def test_the_measured_width_matches_the_constructed_cube():
    img, dep, _ = rgbd_cube(size_m=0.05, z=0.60)
    g = plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    # a flat facing patch has no depth thickness, so the jaw closes across
    # the SMALLER of the two visible extents -- both are the cube's 50 mm
    assert g["width_m"] == pytest.approx(0.05, abs=8e-3)


def test_the_backend_used_is_carried_into_the_plan():
    img, dep, _ = rgbd_cube()
    g = plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert g["backend"] == "colour"
    assert g["n_points"] > 30


def test_too_few_points_says_HOW_CLOSE_to_get():
    """A refusal an autonomy layer can act on.

    Measured on the rig: the cube at 1.5 m gave 18 depth points against the 30
    needed. 'Too few points' is true and useless. Depth area falls as
    1/range^2, so the range that would work is arithmetic.
    """
    # THE REAL SITUATION WAS SPARSE RETURNS, NOT A SMALL FOOTPRINT. On the
    # rig the cube was plainly visible in colour while the depth sensor gave
    # only scattered returns off it -- dark, small, and at the edge of the
    # stereo module's useful range. So the fixture drops most of the depth
    # patch rather than shrinking the object.
    img, dep, px = rgbd_cube(size_m=0.05, z=1.5)
    keep = np.zeros_like(dep, dtype=bool)
    vs, us = np.nonzero(dep)
    keep[vs[::16], us[::16]] = True         # ~1 pixel in 16 returns
    dep = np.where(keep, dep, 0.0).astype(np.float32)
    assert 5 < int(np.count_nonzero(dep)) < 30, int(np.count_nonzero(dep))
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, dep, "green cube", np.zeros(3), np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage == "grasp"
    assert "move to about" in e.value.reason, e.value.reason
    sug = e.value.evidence.get("suggested_range_m")
    assert sug is not None and sug < 2.2, e.value.evidence
    # and the suggested range must actually be closer by the right factor:
    # area falls as 1/range^2, so range scales as sqrt(points/needed)
    n = e.value.evidence["n_points"]
    import math as _m
    assert sug == pytest.approx(1.5 * _m.sqrt(n / 30.0), abs=0.02)


def test_an_object_solving_UNDERGROUND_is_refused():
    """REGRESSION. A near-black segment (mean BGR 4.1, 8.8, 3.9) satisfied the
    green ratio test on noise and solved to z = -0.176 m -- underground -- and
    the pipeline reported it as the cube. Two things were missing: a
    brightness floor in the colour test, and any check on the answer itself."""
    img, dep, _ = rgbd_cube(size_m=0.05, z=0.60)
    # camera low and pointing down, so the object lands below the floor
    with pytest.raises(PlanFailure) as e:
        plan_grasp(img, dep, "green cube", np.array([0.0, 0.0, -0.9]),
                   np.eye(3), CK, DK,
                   detector=PromptDetector(backend="colour"))
    assert e.value.stage == "sanity"
    assert "outside anything this rig can contain" in e.value.reason


def test_a_near_black_region_is_not_green():
    """The ratio test alone passes on sensor noise; the cube's own green
    channel is 52, a false positive's was 8.8."""
    from srl_perception.prompt_detector import _colour_matches
    assert _colour_matches("green", 29.0, 52.0, 10.0) is True    # the cube
    assert _colour_matches("green", 4.1, 8.8, 3.9) is False      # noise
