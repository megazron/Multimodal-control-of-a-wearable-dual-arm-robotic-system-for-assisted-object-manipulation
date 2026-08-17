"""A camera frame carries the pose it was RENDERED from, and the consumer uses it.

WHAT THIS IS ABOUT. Inside the recording sweep on 2026-08-17 the four T1 cubes
deprojected 31.7 / 32.7 / 34.0 / 35.7 mm from truth against a 30 mm capture
gate, so none of the four was grasped -- and the pad miss at closure was
31.6 / 32.6 / 33.9 / 35.6 mm, the same numbers to 0.1 mm. Standalone on the
same stack the identical code returned 1.3 - 3.0 mm.

The deprojection was looking the camera pose up off LIVE TF after the fact,
which answers "where is the camera now" rather than "where was the camera when
this pixel was captured". Those are the same question only while the arm is
perfectly still, and `Vision.stage()` returns as soon as every joint is within
0.02 rad while `ik_follower_node` streams to the same controller.

`mock_rgbd_camera` now publishes the pose it rendered from with each frame, and
`Vision.cam_pose()` prefers it. Everything below is arithmetic on constructed
inputs -- no stack, no camera.

WHAT COULD SILENTLY GO WRONG, and therefore what is checked:

  * the rotation is republished as a QUATERNION, so a matrix -> quaternion ->
    matrix round trip must be exact. A sign convention error here would rotate
    every deprojected ray and would look like a calibration problem;
  * the consumer must PREFER the stamped pose and FALL BACK to TF, and must
    say which it did -- a run that quietly used the old path would reproduce
    the fault while reporting the fix;
  * translating the camera pose must translate every deprojected point by
    exactly that vector, which is the property the live known-answer test
    (`scripts/verify_frame_pose_gate.py`) leans on.
"""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
for _p in (os.path.join(ROOT, "src", "srl_perception"),
           os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc"),
           os.path.join(ROOT, "scripts"),
           os.path.join(ROOT, "config")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _R(rx, ry, rz):
    """A rotation built from three axis rotations, so it is a real SO(3)."""
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quat_to_R(x, y, z, w):
    """The consumer's own conversion, written out so the round trip is closed
    against the code that actually runs rather than against a library."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize("rpy", [
    (0.0, 0.0, 0.0),
    (np.pi, np.pi, 0.0),            # the camera's own mount, diag(-1, -1, 1)
    (0.3, -1.1, 2.7),
    (np.pi / 2, 0.0, -np.pi / 2),
    (-2.9, 0.05, 3.05),             # near the seam, where a trace branch fails
])
def test_the_rotation_survives_the_round_trip(rpy):
    """Matrix -> quaternion -> matrix, exactly. The publisher uses Shepperd's
    branch precisely so the near-pi cases above do not divide by a small
    trace term."""
    mock = pytest.importorskip("srl_perception.mock_rgbd_camera")
    R = _R(*rpy)
    q = mock.MockRGBD._quat_from_R(R) if hasattr(mock, "MockRGBD") else None
    if q is None:
        # the class name is not load-bearing; find the one that has the helper
        q = next(getattr(v, "_quat_from_R")(R)
                 for v in vars(mock).values()
                 if isinstance(v, type) and hasattr(v, "_quat_from_R"))
    back = _quat_to_R(*q)
    assert np.allclose(back, R, atol=1e-9), (rpy, np.abs(back - R).max())


def test_the_quaternion_is_a_unit_quaternion():
    mock = pytest.importorskip("srl_perception.mock_rgbd_camera")
    cls = next(v for v in vars(mock).values()
               if isinstance(v, type) and hasattr(v, "_quat_from_R"))
    for rpy in [(0, 0, 0), (1.2, -0.4, 2.2), (np.pi, np.pi, 0.0)]:
        q = np.array(cls._quat_from_R(_R(*rpy)), float)
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-9)


def test_the_consumer_prefers_the_stamped_pose_and_says_so():
    """`Vision.cam_pose()` must return the FRAME's pose when one is present.

    Written against the source rather than a live node, because constructing a
    Vision needs a ROS graph and the property under test is a branch, not a
    behaviour of the network.
    """
    src = open(os.path.join(ROOT, "scripts", "verify_colour_vision.py")).read()
    assert "self.render_pose = None" in src
    assert "/render_pose" in src, "the consumer must subscribe to the pose"
    body = src[src.index("def cam_pose"):]
    body = body[:body.index("\n    def ", 5)] if "\n    def " in body[5:] else body
    assert "if self.render_pose is not None:" in body, (
        "cam_pose must PREFER the stamped pose")
    assert "lookup_transform" in body, (
        "the TF lookup must remain as the fallback for a real camera that "
        "does not publish a render pose")
    # and the look must record which path it took
    vg = open(os.path.join(ROOT, "src", "srl_experiments", "experiments",
                           "abc", "vision_grasp.py")).read()
    assert 'frame_pose_is_stamped' in vg, (
        "a run that quietly used the old path would reproduce the fault while "
        "reporting the fix; the timing dict must say which happened")


def test_translating_the_camera_translates_every_point_by_the_same_vector():
    """The property the live known-answer test measures.

    Deprojection is p_cam + R_wc * ray, so a translation of p_cam is a rigid
    translation of the answer. If this were not exact, the live test's
    'shifted by exactly d' assertion would be measuring something else.
    """
    from vision_grasp import deproject

    class Info(object):
        k = [615.0, 0.0, 319.5, 0.0, 615.0, 239.5, 0.0, 0.0, 1.0]

    R = _R(np.pi, np.pi, 0.0)
    p = np.array([0.40, 0.30, 1.20])
    d = np.array([0.030, -0.020, 0.010])
    dets = [dict(u=100.0, v=200.0, depth_m=0.75),
            dict(u=520.0, v=90.0, depth_m=0.83),
            dict(u=319.5, v=239.5, depth_m=0.60)]
    for det in dets:
        a = np.asarray(deproject(det, Info(), p, R), float)
        b = np.asarray(deproject(det, Info(), p + d, R), float)
        assert np.allclose(b - a, d, atol=1e-12), (det, b - a)


def test_the_look_pauses_the_follower_and_restores_it_on_every_path():
    """The other half of the fix. Reading the pose off the frame makes a
    settled-but-wrong arm harmless; it does not make a MOVING arm harmless,
    and the follower is what was moving it."""
    vg = open(os.path.join(ROOT, "src", "srl_experiments", "experiments",
                           "abc", "vision_grasp.py")).read()
    assert "import follower_pause as FP" in vg
    assert "FP.pause(n)" in vg
    body = vg[vg.index("def observe_and_detect"):]
    fin = body.index("    finally:")
    assert "FP.resume(n, paused)" in body[fin:], (
        "the restore must be in the finally block -- a restore that is skipped "
        "on an exception disarms the arm for every run after it")


def test_there_is_ONE_follower_pause_implementation():
    """It was a pair of methods on the staging script and is now a module both
    callers use. Two copies of a routine whose restore failure disarms an arm
    is not a copy worth having."""
    stager = open(os.path.join(ROOT, "scripts",
                               "stage_presentation_pose.py")).read()
    assert "import follower_pause as _FP" in stager
    assert "_FP.pause(self)" in stager and "_FP.resume(self" in stager
    # the old inline bodies must be gone, not merely unused
    assert "from rcl_interfaces.srv import SetParameters" not in stager
