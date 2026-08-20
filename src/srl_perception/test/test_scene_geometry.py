"""The camera <-> world transforms, with answers known in advance.

An uncalibrated or mis-framed scene camera does not fail loudly: it produces
smooth, confident body positions in the wrong place. So every transform here
is driven round-trip against constructed geometry, which is the one case
CLAUDE.md's standing rule allows synthetic data for -- the ground truth is
projection arithmetic, not an appearance model.

The specific mistakes these tests exist to catch have all been made once
already in this session:

  * solving PnP in the wrong frame and reading the answer as a world pose
    (the extrinsic tool returned a translation error of exactly the marker's
    height, which is the tell);
  * projecting a point that is BEHIND the camera and drawing it, which puts
    the robot's arm on the wrong side of the picture with nothing to say so.
"""
import math

import numpy as np
import pytest

from srl_perception import scene_geometry as SG


K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])


def frames(t=(0.10, 2.20, 1.40), rpy=(math.radians(-95.0),
                                      math.radians(3.0),
                                      math.radians(182.0)), D=None):
    T = np.eye(4)
    T[:3, :3] = SG.rpy_to_R(*rpy)
    T[:3, 3] = t
    return SG.SceneFrames(K, D if D is not None else np.zeros(5), T,
                          "test", "test")


def test_world_camera_round_trip():
    f = frames()
    pts = np.array([[0.0, 0.0, 1.2], [0.4, 0.3, 1.0], [-0.5, -0.2, 1.6]])
    back = f.camera_to_world(f.world_to_camera(pts))
    assert np.allclose(pts, back, atol=1e-9)


def test_a_point_behind_the_camera_is_reported_as_behind():
    """A finite pixel coordinate for a point behind the lens is the whole
    reason `project` returns depth as well."""
    f = frames()
    # The camera is at y = +2.20 looking back toward the wearer, so a point
    # well beyond it in +y is behind.
    uv, z = f.project(np.array([[0.0, 6.0, 1.4]]))
    assert z[0] < 0, "a point behind the camera reported positive depth"
    assert np.isfinite(uv).all(), "and it still produced a finite pixel"


def test_arm_polyline_stops_rather_than_clamping():
    f = frames()
    chain = np.array([[0.0, 0.0, 1.2], [0.1, 0.1, 1.2], [0.0, 6.0, 1.4],
                      [0.2, 0.2, 1.2]])
    poly = SG.arm_polyline(f, chain, (1280, 720))
    assert len(poly) == 2, ("the polyline must END at the first link that is "
                            "not in front of the camera, not skip it: %r"
                            % (poly,))


def test_an_uncalibrated_camera_refuses_and_says_which_half_is_missing():
    assert SG.SceneFrames(None, None, None).calibrated is False
    why = SG.SceneFrames(None, None, None).why_not()
    assert "intrinsic" in why and "position relative to the robot" in why
    only_k = SG.SceneFrames(K, np.zeros(5), None)
    assert only_k.calibrated is False
    assert "intrinsic" not in only_k.why_not()
    assert SG.arm_polyline(SG.SceneFrames(), [[0, 0, 1]], (640, 480)) == []


def test_lift_recovers_a_known_body_distance():
    """The core of the whole subsystem: a metric skeleton plus its image
    points must come back at the right DISTANCE."""
    # A skeleton in the detector's own frame: origin at the hips.
    skel = np.array([
        [0.19, -0.43, 0.00], [-0.19, -0.43, 0.02], [0.21, -0.15, 0.01],
        [-0.21, -0.15, 0.00], [0.22, 0.08, -0.03], [-0.22, 0.08, -0.02],
        [0.11, 0.00, 0.00], [-0.11, 0.00, 0.00], [0.00, -0.65, -0.05],
        [0.14, 0.45, 0.01], [-0.14, 0.45, 0.02]])
    for true_z in (1.5, 2.2, 3.4):
        cam_true = skel + np.array([0.05, -0.10, true_z])
        img = (K @ cam_true.T).T
        img = img[:, :2] / img[:, 2:3]
        got, resid = SG.lift_skeleton(skel, img, K, np.zeros(5))
        assert got is not None, resid
        assert resid < 0.05, resid
        assert abs(got[:, 2].mean() - cam_true[:, 2].mean()) < 0.01, (
            "body placed at the wrong distance: %.3f vs %.3f"
            % (got[:, 2].mean(), cam_true[:, 2].mean()))


def test_lift_refuses_on_too_few_landmarks():
    got, why = SG.lift_skeleton(np.zeros((3, 3)), np.zeros((3, 2)), K,
                                np.zeros(5))
    assert got is None and "need" in why


def test_lift_scales_with_the_focal_length_which_is_why_calibration_matters():
    """A focal length 10% wrong puts the body 10% out, with no other symptom.

    Asserted rather than described, because it is the argument for refusing
    to publish a guessed intrinsic and it should fail loudly if it ever stops
    being true.
    """
    skel = np.array([
        [0.19, -0.43, 0.0], [-0.19, -0.43, 0.0], [0.21, -0.15, 0.0],
        [-0.21, -0.15, 0.0], [0.11, 0.0, 0.0], [-0.11, 0.0, 0.0],
        [0.0, -0.65, 0.0], [0.14, 0.45, 0.0]])
    cam_true = skel + np.array([0.0, 0.0, 2.0])
    img = (K @ cam_true.T).T
    img = img[:, :2] / img[:, 2:3]
    K_wrong = K.copy()
    K_wrong[0, 0] *= 1.10
    K_wrong[1, 1] *= 1.10
    got, _ = SG.lift_skeleton(skel, img, K_wrong, np.zeros(5))
    err = got[:, 2].mean() - 2.0
    assert err > 0.15, ("a 10%% focal error must move the body; it moved "
                        "%.3f m" % err)


def test_the_arm_chain_is_the_same_one_the_operations_gui_draws():
    """Two pictures of the same robot have to be of the same robot."""
    av = pytest.importorskip("srl_arm_view")
    assert tuple(SG.ARM_CHAIN) == tuple(av.CHAIN)
