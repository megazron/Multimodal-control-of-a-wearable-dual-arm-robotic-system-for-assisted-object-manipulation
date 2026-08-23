"""Object identity comes from the PICTURE, not from clustering the points.

WHY THIS FILE EXISTS. The calibration stage found objects the way this
repository always has: fit a plane to the fused cloud, cluster the points
above it by Euclidean distance. Run live against the T1 scene it reported

    object 0 at (-0.356, 0.471, 1.272), 140 mm across the jaws, NOT graspable

where two 40 mm cubes are. The cubes sit on a 60 mm pitch, so the GAP between
their faces is 20 mm, and `table_scene.objects_on_plane` clusters at 0.020.
Two cubes 20 mm apart are one cluster. The map then refused an object that
does not exist, with a confident reason naming the gripper.

There is no cluster distance that fixes it: below the depth noise the same
cube splits into several, so the failure moves rather than leaving.

WHAT THE FIELD DOES INSTEAD -- the unseen-object-instance-segmentation line of
work, ZISVFM, and the geometric-grounding half of CLASP -- is to cut instances
out of the IMAGE and lift each mask through the depth. Appearance separates
things that are SIDE BY SIDE; it cannot separate things that are STACKED, and
in this scene a blue cube stands on a blue pad with no edge between them, so
height layering does that half. Neither cue alone is enough, and the two fail
in opposite directions.

These tests are constructed geometry -- arithmetic ground truth, which is what
the standing rule allows synthetic data for. The SEGMENTER itself needs a
picture and is exercised live; what is pinned here is everything that decides
the numbers once the masks exist.
"""
import numpy as np
import pytest

from srl_perception import segment_lift as SL
from srl_perception import table_scene as TS
from srl_perception import world_model as WM


def _cube(centre, size=0.04, n=1500, seed=2):
    """A filled box. `size` may be one number or three.

    THREE, BECAUSE MY FIRST VERSION OF THIS HELPER TOOK ONE and the test below
    then built its "pad" as a 160 mm CUBE -- a block the size of a brick
    standing on the table, with the 40 mm cube buried inside it. The merge
    rule failed that and was right to. A pad is 160 x 140 x 10 mm, and the
    whole question the test is asking is whether a 10 mm slab and a 40 mm
    cube resting on it stay two objects.
    """
    rng = np.random.default_rng(seed)
    c = np.asarray(centre, float)
    sz = np.broadcast_to(np.asarray(size, float), (3,))
    return np.column_stack([c[k] + rng.uniform(-sz[k] / 2, sz[k] / 2, n)
                            for k in range(3)])


def _plane_at(z):
    return TS.Plane(np.array([0.0, 0.0, 1.0]), -z, 4000, 4000)


# ------------------------------------------------- the module's own arithmetic

def test_segment_lift_passes_its_own_self_test():
    assert SL.self_test(verbose=False)


def test_a_square_is_not_measured_by_its_diagonal():
    """THE BUG THE SELF-TEST CAUGHT BEFORE THIS EVER RAN LIVE.

    `_measure` used PCA. The principal axes of a square are degenerate -- both
    horizontal directions have the same variance -- so SVD returns the
    diagonal and a 40 mm cube measured 53 mm, which is 40*sqrt(2). Every cube
    in the scene would have been reported half again as wide as it is.
    """
    _, _, w = SL._measure(_cube((0.42, 0.45, 1.27)))
    assert abs(w - 0.040) < 0.004, "a 40 mm cube measured %.1f mm" % (w * 1000)
    assert abs(w - 0.040 * np.sqrt(2)) > 0.008, "this is the diagonal again"


def test_the_centre_of_a_one_sided_cloud_is_not_its_mean():
    """A wrist camera sees the near face and the top and NOT the far face."""
    c = np.array([0.42, 0.45, 1.27])
    full = _cube(c)
    near = full[(full[:, 1] < c[1]) | (full[:, 2] > c[2] + 0.015)]
    mid, _, _ = SL._measure(near)
    assert abs(near.mean(axis=0)[1] - c[1]) > 0.005      # the mean IS biased
    assert abs(mid[1] - c[1]) < 0.002                    # the mid-extent is not


# ---------------------------------------------------------- the height layers

def test_a_cube_on_a_same_coloured_pad_is_split_by_height():
    """The segmenter returns ONE region and it is right to: in the picture the
    blue cube on the blue pad IS one region. The 40 mm step separates them."""
    pad_h = np.random.default_rng(1).uniform(0.000, 0.010, 3000)
    cube_h = np.random.default_rng(2).uniform(0.010, 0.050, 1500)
    layers = SL._layers(np.concatenate([pad_h, cube_h]))
    assert len(layers) >= 2, ("one region, one layer -- the cube and the pad "
                              "were not separated: %s" % (layers,))


def test_a_single_slab_is_not_split_into_imaginary_layers():
    """THE CONTROL. A splitter that split everything would pass the test above
    and would turn every object in the scene into a stack of phantoms."""
    flat = np.random.default_rng(3).uniform(0.000, 0.010, 4000)
    assert len(SL._layers(flat)) == 1, SL._layers(flat)


# ------------------------------------------------------------- the merge rule

def _seg(centre, size, n=800, seed=0):
    p = _cube(centre, size=size, n=n, seed=seed)
    mid, ext, w = SL._measure(p)
    return SL.Seg(mid, ext, w, p, n, [0, 0, 0], 0.02)


def test_two_views_of_one_large_pad_are_one_object():
    """MEASURED LIVE: two views of one 160 mm pad put its centre 60 mm apart,
    and at a fixed 20 mm merge distance ONE pad came back as FOUR objects."""
    a = _seg((0.29, 0.50, 1.255), (0.160, 0.140, 0.010), seed=4)
    b = _seg((0.32, 0.50, 1.255), (0.160, 0.140, 0.010), seed=5)
    v = [WM.View(a.points, "one", objects=[a]),
         WM.View(b.points, "two", objects=[b])]
    assert WM._overlaps(a.lo, a.hi, b.lo, b.hi)
    assert len(WM._fuse_segments(v, _plane_at(1.250))) == 1


def test_two_cubes_sixty_millimetres_apart_stay_two():
    """THE CASE THAT STARTED ALL OF THIS. Widening the merge to fix the pad
    must not bring this back."""
    a = _seg((0.420, 0.45, 1.270), 0.04, seed=6)
    b = _seg((0.480, 0.45, 1.270), 0.04, seed=7)
    assert not WM._overlaps(a.lo, a.hi, b.lo, b.hi)
    objs = WM._fuse_segments(
        [WM.View(np.vstack([a.points, b.points]), "one", objects=[a, b])],
        _plane_at(1.250))
    assert len(objs) == 2, objs
    for o in objs:
        assert o.width_m < 0.055, "%.0f mm -- two cubes fused" % (o.width_m * 1000)


def test_a_cube_standing_on_a_pad_is_not_merged_into_it():
    """The vertical axis of the overlap test, which is the one that is easy to
    leave out. A horizontal-only test merges the cube into the pad it stands
    on and the map loses the only object anybody wanted to pick up."""
    pad = _seg((0.290, 0.50, 1.2550), (0.160, 0.140, 0.010), seed=8)
    cube = _seg((0.290, 0.50, 1.2800), (0.040, 0.040, 0.040), seed=9)
    objs = WM._fuse_segments(
        [WM.View(np.vstack([pad.points, cube.points]), "one",
                 objects=[pad, cube])], _plane_at(1.250))
    assert len(objs) == 2, ("the cube was absorbed into the pad: %s" % (objs,))


# --------------------------------------------------- the map records the path

def test_the_map_says_which_finder_produced_its_objects():
    """A map built by clustering and one built by segmentation fail
    differently. One that could be either is evidence for nothing."""
    a = _seg((0.420, 0.45, 1.270), 0.04, seed=10)
    table = np.column_stack([
        np.random.default_rng(0).uniform(0.2, 0.7, 4000),
        np.random.default_rng(1).uniform(0.3, 0.6, 4000),
        np.full(4000, 1.250)])
    seg_map = WM.build([WM.View(np.vstack([table, a.points]), "v",
                                objects=[a])])
    cl_map = WM.build([WM.View(np.vstack([table, a.points]), "v")])
    assert "segment_lift" in seg_map.provenance["object_finder"]
    assert "table_scene" in cl_map.provenance["object_finder"]


def test_a_map_will_not_mix_the_two_finders():
    a = _seg((0.420, 0.45, 1.270), 0.04, seed=11)
    table = np.column_stack([
        np.random.default_rng(0).uniform(0.2, 0.7, 4000),
        np.random.default_rng(1).uniform(0.3, 0.6, 4000),
        np.full(4000, 1.250)])
    with pytest.raises(WM.MapRefusal) as e:
        WM.build([WM.View(np.vstack([table, a.points]), "v", objects=[a]),
                  WM.View(np.vstack([table, a.points]), "w")])
    assert "two different ways" in str(e.value)


def test_the_segmenter_refuses_rather_than_falling_back_to_clustering():
    """A silent fallback is how a map ends up with one confidence written over
    objects found two different ways."""
    with pytest.raises(SL.SegRefusal):
        SL._weights_path("no-such-weights-file.pt")


# ------------------------------------ the last check on what the fusion made

def test_a_fused_pair_of_cubes_is_split_back_apart():
    """THE ONE THAT SURVIVED EVERY OTHER FIX.

    With the nested masks suppressed, the pose taken off the frame, the
    minimum-width measure and the non-chaining merge all in place, the live
    map STILL reported one object at (0.450, 0.450) with a long extent of
    0.101 m -- two 40 mm cubes on a 60 mm pitch, at their midpoint, where
    nothing is. It reads 41 mm across its narrowest axis, so it is reported
    GRASPABLE and the arm closes on the gap between two cubes.
    """
    a = _seg((0.420, 0.45, 1.270), (0.04, 0.04, 0.04), n=1500, seed=20)
    b = _seg((0.480, 0.45, 1.270), (0.04, 0.04, 0.04), n=1500, seed=21)
    # A view that saw them as ONE region -- which the segmenter legitimately
    # does, since they are the same colour and adjacent.
    both = np.vstack([a.points, b.points])
    mid, ext, w = SL._measure(both)
    fused = SL.Seg(mid, ext, w, both, 3000, [0, 0, 0], 0.02)
    assert abs(w - 0.04) < 0.006, "the fused pair is 40 mm wide, as warned"
    objs = WM._fuse_segments(
        [WM.View(both, "one", objects=[fused])], _plane_at(1.250))
    assert len(objs) == 2, ("still one object %.0f mm long"
                            % (objs[0].extents[0] * 1000))
    xs = sorted(o.centre[0] for o in objs)
    assert abs(xs[0] - 0.420) < 0.005 and abs(xs[1] - 0.480) < 0.005, xs


def test_one_solid_cube_is_not_split_into_pieces():
    """THE CONTROL. A splitter that split everything would pass the test above
    and would turn every object in the scene into rubble."""
    a = _seg((0.420, 0.45, 1.270), (0.04, 0.04, 0.04), n=2500, seed=22)
    objs = WM._fuse_segments(
        [WM.View(a.points, "one", objects=[a])], _plane_at(1.250))
    assert len(objs) == 1, objs
    assert abs(objs[0].width_m - 0.04) < 0.006


def test_and_a_wide_pad_is_not_split_either():
    """A 160 x 140 mm slab is one object however wide it is."""
    p = _seg((0.290, 0.50, 1.255), (0.160, 0.140, 0.010), n=3000, seed=23)
    objs = WM._fuse_segments(
        [WM.View(p.points, "one", objects=[p])], _plane_at(1.250))
    assert len(objs) == 1, objs


def test_a_cube_cut_in_half_by_the_height_split_is_put_back():
    """The lower band of a cube must not survive as its own object.

    It happens when one mask covers a pad and the cube standing on it: the
    modal height is the pad's, so the cut lands part-way up the cube. Four of
    these were in the live map, all reported graspable, all standing at the
    feet of real cubes.
    """
    low = _seg((0.420, 0.45, 1.2565), (0.040, 0.040, 0.013), n=400, seed=30)
    top = _seg((0.420, 0.45, 1.2765), (0.040, 0.040, 0.027), n=900, seed=31)
    objs = WM._fuse_segments(
        [WM.View(np.vstack([low.points, top.points]), "one",
                 objects=[low, top])], _plane_at(1.250))
    assert len(objs) == 1, ("the cube stayed cut in two: %s" % (objs,))
    assert abs(objs[0].extents[2] - 0.040) < 0.006, objs[0].extents


def test_but_a_cube_on_a_pad_is_still_two_objects():
    """THE CONTROL, and it is the whole reason the rule is a footprint ratio
    and not containment: a cube resting on a pad has the same shape as a cube
    resting on its own lower half. What differs is that the pad is four times
    the footprint."""
    pad = _seg((0.290, 0.50, 1.2550), (0.160, 0.140, 0.010), n=1500, seed=32)
    cube = _seg((0.290, 0.50, 1.2800), (0.040, 0.040, 0.040), n=900, seed=33)
    objs = WM._fuse_segments(
        [WM.View(np.vstack([pad.points, cube.points]), "one",
                 objects=[pad, cube])], _plane_at(1.250))
    assert len(objs) == 2, ("the cube was absorbed into its pad: %s" % (objs,))
