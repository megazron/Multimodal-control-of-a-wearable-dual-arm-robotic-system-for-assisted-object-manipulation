#!/usr/bin/env python3
"""What is on the table, and can it be picked up: the chain, end to end.

THE COMPLAINT THIS ANSWERS, in the operator's words: "it is unable to check
what's on the table, it will just move here and there."

That was accurate and it was a MISSING PIECE, not a broken one. There was a
detector that finds a NAMED thing, a grasp planner that turns ONE given cloud
into ONE grasp, and a task layer commanding coordinates written down in
advance. Nothing looked at a surface and enumerated what was standing on it.

`srl_perception.table_scene` is that piece and `scripts/pick_from_table.py`
is the chain around it. The tests below are the properties that make it worth
trusting, and most of them are about the SUPPORT PLANE, because every one of
the three corrections a hand-built grasp needed on 2026-08-21 came from
ignoring the surface the object rests on.

WHAT IS PROVEN HERE AND WHAT IS NOT. Every check runs against CONSTRUCTED
ground truth -- boxes of known size at known places, and a pinhole projection
of them whose corners are arithmetic. That is the only synthetic this
repository trusts, and it is enough to prove the geometry. It is NOT a claim
about a real table: there is no wrist-camera recording of one in this
repository, every RGB-D capture here is from the scene camera, and the first
real run is a measurement to be checked rather than a result.
"""
import math
import os
import sys

import numpy as np
import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
for p in ("scripts", "config", "src/srl_perception", "src/srl_teleop",
          "scripts/real_calibration"):
    sys.path.insert(0, os.path.join(WS, p))

from srl_perception import table_scene as TS                 # noqa: E402


def _table(z=0.90, n=6000, seed=1):
    return TS._plane_points(z=z, n=n, seed=seed)


def _box(c, size, **kw):
    return TS._box_surface(c, size, **kw)


# ------------------------------------------------------------ the module
def test_the_module_self_test_passes():
    assert TS.self_test(verbose=False)


def test_it_finds_the_surface_and_everything_standing_on_it():
    boxes = [((0.10, 0.05, 0.925), (0.05, 0.05, 0.05)),
             ((-0.12, 0.02, 0.935), (0.04, 0.09, 0.07)),
             ((0.00, -0.15, 0.915), (0.03, 0.03, 0.03))]
    P = np.vstack([_table()] + [_box(c, s, seed=i + 2)
                                for i, (c, s) in enumerate(boxes)])
    sc = TS.analyse(P)
    assert abs(sc["plane"].offset - 0.90) < 0.002
    assert len(sc["objects"]) == 3, (
        "%d objects found where three were built" % len(sc["objects"]))
    for c, _s in boxes:
        d = min(np.linalg.norm(o.centre - np.asarray(c))
                for o in sc["objects"])
        assert d < 0.006, "nearest object to %s is %.1f mm away" % (c, d * 1000)


# ------------------------------------------- the plane is what fixes things
def test_the_plane_fixes_the_shell_bias_that_cost_20_mm():
    """One depth view sees the FRONT of an object, and the centroid of a
    front surface is below the centre of a solid. Measured at 20 mm on
    2026-08-21, against a 30 mm capture gate.

    The object is RESTING on the plane, so its bottom is known without a
    second view. This test requires BOTH that the correction is right and
    that the uncorrected value was wrong -- otherwise it would pass on a
    scene where the bias happens to be zero and prove nothing.
    """
    P = np.vstack([_table(),
                   _box((0.0, 0.0, 0.93), (0.05, 0.05, 0.06), n=1800,
                        seed=9, front_only=True)])
    sc = TS.analyse(P)
    assert len(sc["objects"]) == 1
    o = sc["objects"][0]
    err_corrected = abs(o.centre[2] - 0.93)
    err_raw = abs(o.visible_centroid[2] - 0.93)
    assert err_raw > 0.004, (
        "the raw centroid is only %.1f mm out on this scene, so the "
        "correction has nothing to prove" % (err_raw * 1000))
    assert err_corrected < 0.006, (
        "corrected height is %.1f mm out" % (err_corrected * 1000))
    assert err_corrected < err_raw / 2.0


def test_the_approach_follows_the_surface_not_world_down():
    """A jaw axis 31 deg off vertical would have driven the gripper into the
    table. On a tilted surface the approach must tilt with it."""
    n = np.array([0.0, math.sin(math.radians(12)),
                  math.cos(math.radians(12))])
    tp = TS._plane_points(z=0.90, n=6000, normal=n)
    base = tp[0]
    P = np.vstack([tp, _box(base + n * 0.03, (0.05, 0.05, 0.06), seed=7)])
    sc = TS.analyse(P, max_tilt_deg=30.0)
    assert abs(sc["plane"].tilt_deg - 12.0) < 1.5
    assert sc["graspable"], "nothing graspable on a 12 deg surface"
    g = sc["graspable"][0]["grasp"]
    assert float(g.approach @ (-sc["plane"].normal)) > 0.99


def test_the_jaw_is_perpendicular_to_the_approach():
    """Closing the fingers along the direction they are travelling is the
    third of the three 2026-08-21 corrections. Here it is perpendicular BY
    CONSTRUCTION, and this asserts the construction."""
    P = np.vstack([_table(), _box((0.0, 0.0, 0.93), (0.04, 0.07, 0.06),
                                  seed=3)])
    sc = TS.analyse(P)
    g = sc["graspable"][0]["grasp"]
    assert abs(float(g.jaw @ g.approach)) < 1e-9


def test_the_pad_never_goes_below_the_surface():
    for size in ((0.05, 0.05, 0.05), (0.04, 0.04, 0.025),
                 (0.03, 0.06, 0.10)):
        P = np.vstack([_table(),
                       _box((0.0, 0.0, 0.90 + size[2] / 2), size, seed=6)])
        sc = TS.analyse(P)
        for r in sc["graspable"]:
            h = float(sc["plane"].height(r["grasp"].centre[None, :])[0])
            assert h - TS.PAD_HALF_M >= TS.PLANE_MARGIN_M - 1e-9, (
                "size %s: pad bottom %.1f mm above the plane"
                % (size, (h - TS.PAD_HALF_M) * 1000))


# --------------------------------------------------- any height, any object
@pytest.mark.parametrize("z", [0.40, 0.60, 0.75, 0.90, 1.10, 1.25, 1.40])
def test_the_table_can_be_any_height(z):
    """A real table is whatever height it is. RANSAC finds the plane
    wherever it lies; nothing here assumes a bench height."""
    P = np.vstack([_table(z=z),
                   _box((0.05, 0.03, z + 0.025), (0.05, 0.05, 0.05), seed=3)])
    sc = TS.analyse(P)
    assert abs(sc["plane"].offset - z) < 0.002, (
        "plane at %.4f for a table at %.2f" % (sc["plane"].offset, z))
    assert len(sc["objects"]) == 1
    d = np.linalg.norm(sc["objects"][0].centre
                       - np.array([0.05, 0.03, z + 0.025]))
    assert d < 0.004, "centre %.1f mm out at height %.2f" % (d * 1000, z)


@pytest.mark.parametrize("size,true_width_mm,graspable", [
    ((0.05, 0.05, 0.05), 50, True),        # a cube
    ((0.07, 0.07, 0.06), 70, True),        # a bigger cube -- was REFUSED
    ((0.025, 0.025, 0.12), 25, True),      # tall and thin
    ((0.02, 0.02, 0.20), 20, True),        # a rod
    ((0.09, 0.06, 0.10), 60, True),        # a rectangular block
    ((0.13, 0.13, 0.06), 130, False),      # genuinely wider than the jaws
    ((0.09, 0.06, 0.018), 60, False),      # a slab too flat for the pad
])
def test_the_object_can_be_any_shape_and_the_width_is_the_real_one(
        size, true_width_mm, graspable):
    """THE WIDTH MUST BE THE WIDTH, and it was not.

    It came from PCA on the footprint, and PCA on a SQUARE is degenerate --
    equal eigenvalues, so the "short" axis lands on the DIAGONAL as readily
    as on a side. Measured 2026-08-22: a 50 mm cube reported 68 mm, a 25 mm
    bar 34 mm, a 130 mm block 179 mm. Every one inflated by about sqrt(2).

    That is not cosmetic. The gripper is commanded to the reported width, and
    a 70 mm cube reading 95 mm is REFUSED as too wide for jaws that would
    have closed on it. Rotating calipers over the plane give the width a
    parallel jaw actually needs.
    """
    P = np.vstack([_table(),
                   _box((0.0, 0.0, 0.90 + size[2] / 2), size, n=2400,
                        seed=5)])
    sc = TS.analyse(P)
    assert len(sc["objects"]) == 1, "the object must be FOUND either way"
    o = sc["objects"][0]
    assert abs(o.width_m * 1000 - true_width_mm) < 6, (
        "%s: width read %.0f mm, true short axis %d mm"
        % (size, o.width_m * 1000, true_width_mm))
    assert bool(sc["graspable"]) is graspable, (
        "%s: graspable=%s, reason: %s"
        % (size, bool(sc["graspable"]),
           sc["rows"][0]["why"] if sc["rows"] else "-"))


# ------------------------------------------------------------ the refusals
def test_a_wall_is_not_a_table():
    """A wall is a plane, has plenty of inliers, and is not a support
    surface. Fitting one and grasping against it is worse than saying
    nothing."""
    wall = TS._plane_points(z=0.0, n=3000, normal=(1.0, 0.0, 0.02))
    with pytest.raises(TS.SceneRefusal) as e:
        TS.fit_support_plane(wall)
    assert "no plane within" in str(e.value)


def test_an_object_wider_than_the_jaws_is_reported_and_refused():
    P = np.vstack([_table(), _box((0.0, 0.0, 0.95), (0.13, 0.13, 0.10),
                                  seed=4)])
    sc = TS.analyse(P)
    assert len(sc["objects"]) == 1, "the object must still be REPORTED"
    assert not sc["graspable"]
    assert "jaws close" in sc["rows"][0]["why"]


def test_a_long_thin_box_is_grasped_across_its_short_axis():
    """200 x 50 mm is graspable. An earlier version of this suite assumed it
    was not, which would have 'passed' by refusing something pickable."""
    P = np.vstack([_table(), _box((0.0, 0.0, 0.95), (0.20, 0.05, 0.10),
                                  seed=4)])
    sc = TS.analyse(P)
    assert len(sc["graspable"]) == 1
    assert abs(sc["graspable"][0]["grasp"].width_m - 0.05) < 0.006


def test_an_empty_table_says_so():
    sc = TS.analyse(_table())
    assert not sc["objects"]
    assert "NOTHING standing on it" in TS.describe(sc)


# ---------------------------------------------------- the chain around it
@pytest.fixture(scope="module")
def pft():
    import pick_from_table as P
    return P


def test_the_pick_chain_self_test_passes(pft):
    assert pft.self_test(verbose=False)


def test_naming_an_object_selects_that_one(pft):
    """The check that separates "pick up the red one" from "pick up
    whatever you liked best"."""
    K = (600.0, 600.0, 320.0, 240.0, 640, 480)
    cam = np.array([[1.0, 0, 0, 0.0], [0, -1.0, 0, 0.0],
                    [0, 0, -1.0, 1.40], [0, 0, 0, 1.0]])
    H, W = 480, 640
    depth = np.full((H, W), 0.50)
    vs, us = np.mgrid[0:H, 0:W]
    for (cx, cy, cz), side in (((0.03, 0.02, 0.925), 0.05),
                               ((-0.06, -0.04, 0.930), 0.06)):
        top = cz + side / 2.0
        zc = cam[2, 3] - top
        u = K[2] + (cx - cam[0, 3]) * K[0] / zc
        v = K[3] - (cy - cam[1, 3]) * K[1] / zc
        r = (side / 2.0) * K[0] / zc
        depth[(np.abs(us - u) <= r) & (np.abs(vs - v) <= r)] = zc

    default = pft.plan(depth, K, cam, "left", verbose=False)
    others = [o for o in default["scene"]["objects"]
              if o.index != default["object"].index]
    assert others, "need two objects to prove a choice was made"
    target = others[0]

    _, uv = pft.cloud_in_robot_frame(depth, K, cam, with_pixels=True)
    mask = np.zeros((H, W), bool)
    px = uv[target.pixel_index]
    mask[px[:, 1].astype(int), px[:, 0].astype(int)] = True

    class _Det:
        def detect(self, bgr, prompt, depth_m=None, K=None):
            d = type("D", (), {})()
            d.mask, d.bbox, d.backend, d.score = mask, (0, 0, W, H), "t", 0.9
            d.label = prompt
            return [d]

    got = pft.plan(depth, K, cam, "left", want="that one", detector=_Det(),
                   bgr=np.zeros((H, W, 3), np.uint8), verbose=False)
    assert got["object"].index == target.index
    assert got["object"].index != default["object"].index, (
        "the named object happened to be the default, so this proves "
        "nothing about name matching")
    assert got["matched"]["overlap"] > 0.5


def test_naming_something_absent_refuses_rather_than_substituting(pft):
    K = (600.0, 600.0, 320.0, 240.0, 640, 480)
    cam = np.array([[1.0, 0, 0, 0.0], [0, -1.0, 0, 0.0],
                    [0, 0, -1.0, 1.40], [0, 0, 0, 1.0]])
    H, W = 480, 640
    depth = np.full((H, W), 0.50)
    vs, us = np.mgrid[0:H, 0:W]
    zc = cam[2, 3] - 0.95
    u, v = K[2], K[3]
    r = 0.025 * K[0] / zc
    depth[(np.abs(us - u) <= r) & (np.abs(vs - v) <= r)] = zc

    class _Miss:
        def detect(self, bgr, prompt, depth_m=None, K=None):
            d = type("D", (), {})()
            d.mask = np.zeros((H, W), bool)
            d.bbox, d.backend, d.score, d.label = (0, 0, 2, 2), "t", 0.9, "x"
            return [d]

    with pytest.raises(TS.SceneRefusal) as e:
        pft.plan(depth, K, cam, "left", want="a thing that is not there",
                 detector=_Miss(), bgr=np.zeros((H, W, 3), np.uint8),
                 verbose=False)
    assert "Refusing rather than picking up" in str(e.value)


def test_a_frame_with_no_depth_refuses(pft):
    K = (600.0, 600.0, 320.0, 240.0, 640, 480)
    cam = np.eye(4)
    with pytest.raises(TS.SceneRefusal) as e:
        pft.plan(np.zeros((480, 640)), K, cam, "left", verbose=False)
    assert "no depth" in str(e.value)
