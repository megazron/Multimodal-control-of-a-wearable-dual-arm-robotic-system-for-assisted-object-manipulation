"""Known-answer tests for scene-camera calibration.

A camera of KNOWN pose and focal is used to project known 3D points into
pixels; the solver must then recover the camera it was given. That is a real
known-answer test: the truth exists before the code runs, and the residual
says whether to believe the answer.

The refusals matter as much. This project already produced a 24 px solve and
nearly used it; a calibrator that cannot say "do not believe me" is worse than
none, because everything downstream inherits the error silently.
"""
import math

import numpy as np
import pytest

from srl_perception import scene_calibration as SCAL


def make_camera(f=900.0, W=1280, H=720, pos=(0.4, 2.2, 1.7),
                look_at=(0.2, 0.25, 1.15)):
    """A camera at `pos` actually LOOKING AT `look_at`.

    An earlier version of this fixture built a rotation from a yaw and a tilt
    and never checked where it pointed: the sample points came out at depths
    of -0.06 to 0.53 m -- some BEHIND the camera -- and projected to pixel
    coordinates up to 207542 on a 1280-wide image. The solver was then blamed
    for failing to fit a camera that could not see its own targets.

    Built from a look-at, the geometry is right by construction.
    """
    C = np.asarray(pos, float)
    fwd = np.asarray(look_at, float) - C
    fwd /= np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd])        # world -> camera
    t = -R @ C
    return R, t, f, (W, H)


def project(R, t, f, WH, pts):
    W, H = WH
    out = []
    for p in np.asarray(pts, float):
        c = R @ p + t
        out.append([f * c[0] / c[2] + W / 2, f * c[1] / c[2] + H / 2])
    return np.array(out)


def sample_points(n=10):
    rng = np.random.default_rng(4)
    return np.column_stack([rng.uniform(-0.5, 0.9, n),
                            rng.uniform(-0.1, 0.6, n),
                            rng.uniform(0.9, 1.5, n)])


def test_the_solver_recovers_a_camera_it_was_given():
    R, t, f, WH = make_camera()
    P = sample_points(12)
    uv = project(R, t, f, WH, P)
    sol = SCAL.solve_camera(P, uv, WH)
    assert sol["reproj_mean_px"] < 0.5
    assert sol["f"] == pytest.approx(f, rel=0.02)
    assert np.allclose(sol["camera_position"], (0.4, 2.2, 1.7), atol=0.02)


def test_check_accepts_a_good_solve_and_REFUSES_a_bad_one():
    R, t, f, WH = make_camera()
    P = sample_points(12)
    sol = SCAL.solve_camera(P, project(R, t, f, WH, P), WH)
    assert SCAL.check(sol) is True
    sol["reproj_mean_px"] = 24.0          # the value this project measured
    with pytest.raises(SCAL.CalibrationRefusal) as e:
        SCAL.check(sol)
    assert "marker" in str(e.value) and "mm" in str(e.value)


def test_too_few_points_is_REFUSED_with_the_arithmetic():
    with pytest.raises(SCAL.CalibrationRefusal) as e:
        SCAL.solve_camera(np.zeros((3, 3)), np.zeros((3, 2)), (1280, 720))
    assert "7 unknowns" in str(e.value)


def test_noise_in_the_pixels_shows_up_in_the_residual():
    """The residual must TRACK the error, or it cannot be used as a verdict."""
    R, t, f, WH = make_camera()
    P = sample_points(14)
    uv = project(R, t, f, WH, P)
    rng = np.random.default_rng(1)
    clean = SCAL.solve_camera(P, uv, WH)["reproj_mean_px"]
    noisy = SCAL.solve_camera(P, uv + rng.normal(0, 6.0, uv.shape),
                              WH)["reproj_mean_px"]
    assert noisy > clean + 1.0, (clean, noisy)


def test_a_pixel_becomes_a_ray_that_points_back_at_its_own_3d_point():
    R, t, f, WH = make_camera()
    P = sample_points(12)
    sol = SCAL.solve_camera(P, project(R, t, f, WH, P), WH)
    target = P[0]
    u, v = project(R, t, f, WH, [target])[0]
    o, d = SCAL.pixel_to_ray(sol, u, v)
    to = target - o
    to /= np.linalg.norm(to)
    assert float(np.dot(to, d)) > 0.9995, float(np.dot(to, d))


def test_a_ray_meets_a_known_plane_at_the_known_point():
    """This is how a scene pixel becomes a POSITION without any depth."""
    R, t, f, WH = make_camera()
    P = sample_points(12)
    sol = SCAL.solve_camera(P, project(R, t, f, WH, P), WH)
    target = P[3]
    u, v = project(R, t, f, WH, [target])[0]
    o, d = SCAL.pixel_to_ray(sol, u, v)
    hit = SCAL.ray_plane(o, d, (0, 0, target[2]), (0, 0, 1.0))
    assert hit is not None
    assert np.allclose(hit, target, atol=6e-3), (hit, target)


def test_a_ray_parallel_to_the_plane_returns_None_rather_than_a_huge_number():
    hit = SCAL.ray_plane((0, 0, 1.0), (1.0, 0, 0), (0, 0, 0.5), (0, 0, 1.0))
    assert hit is None


def test_find_marker_locates_a_constructed_patch_and_returns_None_when_absent():
    import cv2
    img = np.full((480, 640, 3), 30, np.uint8)
    # a saturated magenta patch -- a hue the lab does not already contain
    hsv = np.uint8([[[165, 230, 230]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    img[200:230, 300:330] = bgr
    got = SCAL.find_marker(img, (160, 120, 90), (172, 255, 255))
    assert got is not None
    assert got["uv"][0] == pytest.approx(314.5, abs=2)
    assert got["uv"][1] == pytest.approx(214.5, abs=2)
    blank = np.full((480, 640, 3), 30, np.uint8)
    assert SCAL.find_marker(blank, (160, 120, 90), (172, 255, 255)) is None
