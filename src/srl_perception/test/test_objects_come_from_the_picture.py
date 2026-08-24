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
    """A horizontal support plane at height `z`.

    THE SIGN WAS WRONG HERE FOR THE WHOLE FILE. `Plane.height` is
    `p . n - offset`, so a plane at z carries offset = +z; this helper passed
    -z, which reported every object as ~2.5 m above the surface. Nothing
    failed, because none of the tests using it were sensitive to the magnitude
    -- they only ever asked whether things were above the plane, and with a
    2.5 m answer they always were. A helper that is wrong in a direction no
    test can feel is worth more attention than one that breaks.
    """
    return TS.Plane(np.array([0.0, 0.0, 1.0]), float(z), 4000, 4000)


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


def test_a_view_of_bare_table_is_segmented_not_unsegmented():
    """An empty object list is an ANSWER, not a missing one.

    `View.objects` was set with `list(objects) if objects else None`, which
    collapses "the segmenter ran and found nothing" into "no segmenter ran".
    On the first two-arm sweep one cell of nine returned zero regions and
    `build` refused the entire map for mixing two finders.
    """
    table = np.column_stack([
        np.random.default_rng(0).uniform(0.2, 0.7, 4000),
        np.random.default_rng(1).uniform(0.3, 0.6, 4000),
        np.full(4000, 1.250)])
    v = WM.View(table, "empty_cell", objects=[])
    assert v.objects == [], v.objects
    assert v.objects is not None, "an empty surface read as 'not segmented'"
    # TWO views of the cube, because a single-view detection is now filed as
    # weak on its own account -- see MIN_OBJECT_VIEWS. This test is about the
    # EMPTY view not poisoning the finder, so the object has to be one the
    # evidence rule would keep anyway.
    a = _seg((0.420, 0.45, 1.270), (0.04, 0.04, 0.04), n=3000, seed=40)
    m = WM.build([WM.View(np.vstack([table, a.points]), "seen", objects=[a]),
                  WM.View(np.vstack([table, a.points]), "seen_again",
                          objects=[a]),
                  v])
    assert "segment_lift" in m.provenance["object_finder"]
    assert len(m.objects) == 1, m.objects


def test_a_thirty_point_sliver_seen_once_is_not_an_object():
    """The full two-arm sweep returned 14 objects where 6 exist. The six carry
    110 000 to 500 000 points from 47 to 66 viewpoints; the other eight carry
    25 to 77 points from one or two. Four orders of magnitude of evidence
    separate them, and `pick_from_map` would have planned a grasp on either.
    """
    strong = _seg((0.420, 0.45, 1.270), (0.04, 0.04, 0.04), n=3000, seed=50)
    sliver = _seg((0.60, 0.40, 1.262), (0.006, 0.03, 0.004), n=30, seed=51)
    solid, weak = WM._split_weak(WM._fuse_segments(
        [WM.View(strong.points, "a", objects=[strong]),
         WM.View(strong.points, "b", objects=[strong]),
         WM.View(sliver.points, "c", objects=[sliver])], _plane_at(1.250)))
    assert len(solid) == 1, solid
    assert len(weak) == 1, weak
    assert weak[0].n_points < WM.MIN_OBJECT_POINTS


def test_the_weak_ones_are_kept_and_named_not_deleted():
    """Deleting them silently hides a real return. A map where EVERYTHING is
    weak is a fact the operator needs."""
    table = np.column_stack([
        np.random.default_rng(0).uniform(0.2, 0.7, 4000),
        np.random.default_rng(1).uniform(0.3, 0.6, 4000),
        np.full(4000, 1.250)])
    sliver = _seg((0.60, 0.40, 1.262), (0.006, 0.03, 0.004), n=30, seed=52)
    m = WM.build([WM.View(np.vstack([table, sliver.points]), "only",
                          objects=[sliver])])
    assert m.objects == [], m.objects
    assert len(m.provenance["weak_detections"]) == 1
    assert "points" in m.provenance["weak_rule"]


# ------------------------------- the plane, and where its height is ASKED FOR

def _slab_with_visible_edge(top_z=0.90, thick=0.035, n_top=8000, n_edge=2500):
    """A tabletop AND the front edge face beneath it, which is what the sweep
    actually sees at 38.9 degrees of elevation."""
    r = np.random.default_rng(7)
    top = np.column_stack([r.uniform(0.25, 0.55, n_top),
                           r.uniform(0.35, 0.60, n_top),
                           np.full(n_top, top_z)])
    edge = np.column_stack([r.uniform(0.25, 0.55, n_edge),
                            np.full(n_edge, 0.35),
                            r.uniform(top_z - thick, top_z, n_edge)])
    return np.vstack([top, edge]), top_z


def test_the_surface_height_is_asked_for_where_the_surface_IS():
    """THE LIVE MAP REPORTED THE SURFACE 5.2 mm LOW AND THE PLANE WAS RIGHT.

    `surface_z` returned `offset / n_z` -- the plane's z-intercept at
    x = y = 0. The fit comes out slightly tilted, so its intercept is not its
    height anywhere a table exists, and on this rig the origin is inside the
    WEARER, half a metre from the nearest tabletop.

    I had already written a plane-refinement to correct that "bias" before
    checking whether it was one. It was not.
    """
    pts, top_z = _slab_with_visible_edge()
    m = WM.build([WM.View(pts, "slab")])
    assert abs(m.surface_z - top_z) < 0.003, (m.surface_z, top_z)

    intercept = m.plane.offset / abs(m.plane.normal[2])
    at_table = m.plane_z_at(0.40, 0.475)
    assert abs(at_table - top_z) < 0.003, at_table
    # THE CONTROL: the intercept really is the misleading number, so this test
    # is about something. If the fit came out perfectly level the two would
    # agree and the whole point would be untestable.
    assert m.plane.tilt_deg > 0.05, m.plane.tilt_deg


def test_the_plane_itself_sits_on_the_tabletop_all_along():
    """What was never wrong. The points of the top are ON the plane."""
    pts, top_z = _slab_with_visible_edge()
    m = WM.build([WM.View(pts, "slab")])
    top = pts[np.abs(pts[:, 2] - top_z) < 1e-9]
    assert abs(float(np.median(m.plane.height(top)))) < 0.002


def test_a_tilted_plane_would_promote_the_tabletop_to_an_object():
    """WHY THE TILT MATTERS, which is a different thing from the reporting.

    A support plane tipped to catch the slab's edge face sits BELOW the top
    over part of the table. The height gate is 4 mm, so the table clears its
    own surface there -- and the live map duly reported two slabs of tabletop,
    231 x 392 mm and 79 x 314 mm, as objects.
    """
    pts, top_z = _slab_with_visible_edge()
    loose = TS.fit_support_plane(pts, tol_m=0.008)
    tight = TS.fit_support_plane(pts, tol_m=0.004)
    assert tight.tilt_deg < loose.tilt_deg, (tight.tilt_deg, loose.tilt_deg)
    top = pts[np.abs(pts[:, 2] - top_z) < 1e-9]
    # the loose fit lifts part of the tabletop over the gate; the tight one
    # lifts far less of it
    over_loose = float((loose.height(top) > SL.MIN_HEIGHT_M).mean())
    over_tight = float((tight.height(top) > SL.MIN_HEIGHT_M).mean())
    assert over_tight <= over_loose, (over_tight, over_loose)


def test_the_evidence_rule_scales_with_how_many_views_were_taken():
    """A RE-LOOK VISITS THREE VIEWPOINTS ON PURPOSE.

    `MIN_OBJECT_VIEWS = 2` is right for a 73-view sweep, where a single
    sighting really is a sliver. Applied to a three-view re-look it demoted
    every rearranged object and reported all six GONE -- a fast re-measure
    that concludes the table is empty, which is worse than none because it
    looks like an answer.
    """
    assert WM.views_needed(1) == 1
    assert WM.views_needed(2) == 1
    assert WM.views_needed(3) == WM.MIN_OBJECT_VIEWS
    assert WM.views_needed(73) == WM.MIN_OBJECT_VIEWS

    a = _seg((0.420, 0.45, 1.270), (0.04, 0.04, 0.04), n=3000, seed=60)
    one = [WM.View(a.points, "only", objects=[a])]
    solid, weak = WM._split_weak(WM._fuse_segments(one, _plane_at(1.250)),
                                 len(one))
    assert len(solid) == 1, "a single-view re-look found nothing"
    # THE CONTROL: on a big sweep the same single sighting is still weak.
    solid2, weak2 = WM._split_weak(WM._fuse_segments(one, _plane_at(1.250)), 73)
    assert len(solid2) == 0 and len(weak2) == 1


def test_the_observe_set_can_re_find_every_object_not_just_the_surface():
    """SURFACE COVERAGE IS NOT DETECTION COVERAGE.

    The first observe set covered 99% of the table from three viewpoints, and
    the re-look that used it came back with FIVE objects where the scene has
    six. A viewpoint can contribute points to a patch -- enough to cover it --
    while seeing an object there too obliquely to segment. A short list that
    silently loses an object is worse than no short list.
    """
    plane = _plane_at(0.90)
    wide = _seg((0.40, 0.40, 0.93), (0.04, 0.04, 0.04), n=2000, seed=70)
    lone = _seg((0.65, 0.40, 0.93), (0.04, 0.04, 0.04), n=2000, seed=71)
    # A broad view covering the whole table but detecting only the near cube.
    broad_pts = np.vstack([
        np.column_stack([np.random.default_rng(1).uniform(0.30, 0.70, 6000),
                         np.random.default_rng(2).uniform(0.35, 0.45, 6000),
                         np.full(6000, 0.90)]), wide.points])
    v_broad = WM.View(broad_pts, "broad", objects=[wide],
                      viewpoint=dict(arm="left", cam=[0.5, 0.2, 1.4],
                                     quat=[0, 0, 0, 1], facing_deg=0.0,
                                     layer=0, elev_deg=38.9))
    # A narrow view that is the ONLY one detecting the far cube.
    v_narrow = WM.View(lone.points, "narrow", objects=[lone],
                       viewpoint=dict(arm="left", cam=[0.8, 0.2, 1.4],
                                      quat=[0, 0, 0, 1], facing_deg=0.0,
                                      layer=0, elev_deg=38.9))
    objs = WM._fuse_segments([v_broad, v_narrow], plane)
    chosen, cover = WM.observe_set([v_broad, v_narrow], plane, objects=objs)
    srcs = {c["source"] for c in chosen}
    assert "narrow" in srcs, (
        "the viewpoint that is the ONLY one able to find the far cube was "
        "dropped: %s" % srcs)
    assert any(c.get("reason", "").startswith("detects") for c in chosen)


def test_and_a_redundant_viewpoint_is_still_dropped():
    """THE CONTROL. A rule that kept every viewpoint would pass the test above
    and would not be a short list at all."""
    plane = _plane_at(0.90)
    a = _seg((0.40, 0.40, 0.93), (0.04, 0.04, 0.04), n=2000, seed=72)
    pts = np.vstack([
        np.column_stack([np.random.default_rng(3).uniform(0.30, 0.50, 5000),
                         np.random.default_rng(4).uniform(0.35, 0.45, 5000),
                         np.full(5000, 0.90)]), a.points])
    vs = [WM.View(pts, "v%d" % i, objects=[a],
                  viewpoint=dict(arm="left", cam=[0.5, 0.2, 1.4],
                                 quat=[0, 0, 0, 1], facing_deg=0.0,
                                 layer=0, elev_deg=38.9)) for i in range(4)]
    objs = WM._fuse_segments(vs, plane)
    chosen, _ = WM.observe_set(vs, plane, objects=objs)
    assert len(chosen) == 1, ("four identical viewpoints should collapse to "
                              "one, got %d" % len(chosen))


def test_a_long_thin_sliver_seen_by_many_views_is_not_an_object():
    """A TOTAL POINT COUNT DOES NOT SEPARATE THESE.

    On the table-beside-the-wearer run the map carried three extras at the
    edges of the pads -- 99 x 33 mm at 6 mm tall, 46 x 19, 39 x 5 -- each seen
    by sixteen to eighteen viewpoints. Their totals (226-447) clear the
    200-point floor, and they are nothing: points PER VIEW were 13, 16 and 28
    against 754 to 18 083 for the six real objects.

    A viewpoint that genuinely sees an object returns hundreds of points from
    it. One that catches its edge returns a handful.
    """
    class _O:
        def __init__(self, n, views, w=0.040):
            self.n_points, self.width_m = n, w
            self.seen_by = ["v%d" % i for i in range(views)]
    # WIDTHS AS MEASURED, so this test exercises the points-per-view rule and
    # not the size floor: the fragments were 33, 19 and 5 mm across, all above
    # the 5 mm sensor floor, so only the per-view rule can reject them.
    real = [_O(12071, 16), _O(11101, 2), _O(108497, 6)]
    frag = [_O(447, 16, 0.033), _O(296, 18, 0.019), _O(226, 18, 0.005)]
    solid, weak = WM._split_weak(real + frag, n_views=18)
    assert len(solid) == 3, [o.n_points for o in solid]
    assert len(weak) == 3, [o.n_points for o in weak]
    # THE CONTROL: the fragments DO clear the total-points floor, so the
    # per-view rule is doing work the old rule could not.
    assert all(o.n_points >= WM.MIN_OBJECT_POINTS for o in frag)


def test_a_one_millimetre_object_is_not_a_small_object():
    """The last spurious detection on the table-beside run was 1 mm across.

    At the sweep's working range one camera pixel subtends about 2.1 mm, so an
    extent under two pixels is below what the sensor can resolve -- it is a
    line of points where two surfaces meet, not a measurement of a thing.
    """
    class _O:
        def __init__(self, n, views, w):
            self.n_points, self.width_m = n, w
            self.seen_by = ["v%d" % i for i in range(views)]
    real = _O(11000, 4, 0.040)
    hair = _O(11000, 4, 0.001)          # plenty of points, no width
    solid, weak = WM._split_weak([real, hair], n_views=4)
    assert len(solid) == 1 and solid[0].width_m == 0.040
    assert len(weak) == 1
    # THE CONTROL: a genuinely thin object stays. The rule is a sensor floor,
    # not a dislike of thin things.
    thin = _O(11000, 4, 0.008)
    solid2, _ = WM._split_weak([thin], n_views=4)
    assert len(solid2) == 1, "an 8 mm object should survive"
