"""Known-answer tests for the RGB-D grasp geometry.

Every fixture here is CONSTRUCTED -- a cube of a stated size at a stated
place -- so the right answer is known before the code runs. That is the only
synthetic data docs/ENGINEERING_LOG.md permits: arithmetic and geometry, never a render.

The refusals are tested as hard as the successes. A grasp planner that
cannot say "too wide" will hand the arm a grasp it discovers by collision.
"""
import math

import numpy as np
import pytest

from srl_perception import rgbd_grasp as G


def box_cloud(size, centre=(0, 0, 1.0), n=12, rot=None):
    """A filled box of the given (sx, sy, sz), as a point cloud."""
    sx, sy, sz = size
    g = np.linspace(-0.5, 0.5, n)
    P = np.array([[x * sx, y * sy, z * sz] for x in g for y in g for z in g])
    if rot is not None:
        P = P @ np.asarray(rot).T
    return P + np.asarray(centre, float)


def test_principal_axes_recover_a_known_box():
    P = box_cloud((0.20, 0.10, 0.04))
    c, V, ext = G.principal_axes(P)
    assert np.allclose(c, [0, 0, 1.0], atol=1e-9)
    # extents come back longest-first and must match what was constructed
    assert ext[0] == pytest.approx(0.20, abs=2e-3)
    assert ext[1] == pytest.approx(0.10, abs=2e-3)
    assert ext[2] == pytest.approx(0.04, abs=2e-3)


def test_grasp_closes_across_the_short_axis():
    """A 200x100x40 box must be gripped across the 40, not the 200."""
    P = box_cloud((0.20, 0.10, 0.04))
    g = G.grasp_from_cloud(P)
    assert g["width_m"] == pytest.approx(0.04, abs=3e-3)
    # the jaw axis must be the box's z, i.e. world z here
    assert abs(abs(float(np.dot(g["close_axis"], [0, 0, 1]))) - 1.0) < 1e-2
    # and the approach must be perpendicular to it, or it is not a grasp
    assert abs(float(np.dot(g["close_axis"], g["approach"]))) < 1e-6


def test_a_40mm_cube_is_graspable_and_the_width_is_right():
    P = box_cloud((0.04, 0.04, 0.04))
    g = G.grasp_from_cloud(P)
    assert g["width_m"] == pytest.approx(0.04, abs=3e-3)


def test_too_wide_is_REFUSED_and_says_the_number():
    """THE CHECK THAT MUST BE ABLE TO FAIL. 120 mm cannot fit an 85 mm jaw."""
    P = box_cloud((0.30, 0.20, 0.12))
    with pytest.raises(G.GraspRefusal) as e:
        G.grasp_from_cloud(P)
    msg = str(e.value)
    assert "120" in msg and "85" in msg, msg


def test_a_slab_too_wide_one_way_is_grasped_the_other_way():
    """Short axis 100 mm (too wide) but MID axis 50 mm -- take the mid."""
    P = box_cloud((0.30, 0.05, 0.10))
    g = G.grasp_from_cloud(P)
    assert g["width_m"] == pytest.approx(0.05, abs=4e-3)


def test_too_few_points_is_REFUSED():
    P = box_cloud((0.04, 0.04, 0.04), n=2)          # 8 points
    with pytest.raises(G.GraspRefusal) as e:
        G.grasp_from_cloud(P)
    assert "noise" in str(e.value)


def test_deproject_matches_a_hand_computed_point():
    K = (1297.6729, 1298.6313, 620.914, 238.28032)   # the rig's real colour K
    p = G.deproject(620.914, 238.28032, 1.5, K)       # exactly the principal point
    assert np.allclose(p, [0, 0, 1.5], atol=1e-9)
    p2 = G.deproject(620.914 + 1297.6729, 238.28032, 2.0, K)   # 45 deg in x
    assert p2[0] == pytest.approx(2.0, abs=1e-9)


def test_cloud_from_mask_uses_only_masked_valid_pixels():
    K = (100.0, 100.0, 50.0, 50.0)
    depth = np.zeros((100, 100), float)
    depth[40:60, 40:60] = 1.0
    mask = np.zeros((100, 100), bool)
    mask[40:60, 40:60] = True
    mask[0:5, 0:5] = True            # masked but depth is 0 -> must be dropped
    pts = G.cloud_from_mask(depth, mask, K)
    assert len(pts) == 400
    assert np.all(pts[:, 2] == 1.0)


def test_rotated_box_still_gives_the_right_width():
    """The answer must not depend on how the object happens to be oriented."""
    th = math.radians(37.0)
    Rz = np.array([[math.cos(th), -math.sin(th), 0],
                   [math.sin(th), math.cos(th), 0], [0, 0, 1]])
    g = G.grasp_from_cloud(box_cloud((0.20, 0.10, 0.04), rot=Rz))
    assert g["width_m"] == pytest.approx(0.04, abs=3e-3)


def test_to_robot_frame_moves_the_grasp_and_keeps_it_a_frame():
    P = box_cloud((0.06, 0.05, 0.04))
    g = G.grasp_from_cloud(P)
    th = math.radians(90.0)
    R = np.array([[math.cos(th), -math.sin(th), 0],
                  [math.sin(th), math.cos(th), 0], [0, 0, 1]])
    t = np.array([1.0, 2.0, 3.0])
    r = G.to_robot_frame(g, t, R)
    assert np.allclose(r["centre"], t + R @ g["centre"], atol=1e-9)
    assert abs(float(np.dot(r["close_axis"], r["approach"]))) < 1e-6
    assert r["width_m"] == g["width_m"]


def test_pregrasp_backs_off_along_the_approach_by_the_stated_distance():
    P = box_cloud((0.05, 0.05, 0.04))
    g = G.grasp_from_cloud(P)
    p = G.pregrasp(g, 0.12)
    assert float(np.linalg.norm(p - g["centre"])) == pytest.approx(0.12, abs=1e-9)
    # and it must be BEHIND the grasp, not in front of it
    assert float(np.dot(p - g["centre"], g["approach"])) < 0


def test_a_cube_is_gripped_ACROSS_A_FACE_not_a_diagonal():
    """REGRESSION. PCA alone returned 43.0 mm for a 40 mm cube because the
    covariance is degenerate and the eigensolver picked a face diagonal. The
    width was 7% wrong; the DIRECTION was the real defect, because a jaw told
    to close on a diagonal approaches a corner and the cube rolls out."""
    P = box_cloud((0.04, 0.04, 0.04))
    g = G.grasp_from_cloud(P)
    assert g["width_m"] == pytest.approx(0.040, abs=1.5e-3)
    # the jaw axis must lie along a face normal: one component ~1, others ~0
    a = np.abs(g["close_axis"])
    assert a.max() > 0.98, "jaw axis is not face-aligned: %s" % g["close_axis"]
    assert np.sort(a)[1] < 0.15


def test_min_width_beats_pca_on_a_degenerate_shape():
    """The refinement must never be WORSE than plain PCA -- checked, not assumed."""
    for size in ((0.04, 0.04, 0.04), (0.06, 0.06, 0.03), (0.09, 0.05, 0.05)):
        P = box_cloud(size)
        _, V, ext = G.principal_axes(P)
        _, _, w_ref, _ = G.min_width_frame(P, V[:, 0])
        assert w_ref <= float(ext[2]) + 1e-6, (size, w_ref, ext[2])


def test_a_SHELL_from_one_depth_view_is_not_gripped_along_the_line_of_sight():
    """REGRESSION, and it would have driven the jaw through the object.

    A depth camera sees the FRONT SURFACE only, so the cloud has ~no
    thickness along the line of sight. Without the view-axis constraint the
    minimum-width search picked exactly that direction and reported a 50 mm
    cube as 0.0 mm wide -- a grasp closing through a surface whose far side
    was never measured.
    """
    # a flat facing patch: 50 x 50 mm at z = 0.6, zero depth thickness
    g0 = np.linspace(-0.025, 0.025, 25)
    P = np.array([[x, y, 0.60] for x in g0 for y in g0])

    bad = G.grasp_from_cloud(P)                    # unconstrained
    assert bad["width_m"] < 1e-6, "fixture no longer reproduces the defect"

    good = G.grasp_from_cloud(P, view_axis=np.array([0.0, 0.0, 1.0]))
    assert good["width_m"] == pytest.approx(0.050, abs=3e-3)
    # the jaw must lie in the image plane, i.e. perpendicular to the sight line
    assert abs(float(np.dot(good["close_axis"], [0, 0, 1]))) < 1e-6
    # and the hand comes in ALONG the sight line
    assert abs(abs(float(np.dot(good["approach"], [0, 0, 1]))) - 1.0) < 1e-6
