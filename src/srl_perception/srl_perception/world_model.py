#!/usr/bin/env python3
"""ONE MAP OF WHAT IS ACTUALLY THERE, BUILT FROM EVERY SOURCE THAT CAN SEE.

    python3 -m srl_perception.world_model          # the self-test

PURE. No ROS, no camera, no robot. It takes point clouds ALREADY IN THE ROBOT
FRAME and returns a map; who moved the arm and who did the TF is somebody
else's problem, which is what makes this testable against constructed scenes.

WHY THIS EXISTS, AND IT IS THE CENTRAL DEFECT OF THE PROJECT
------------------------------------------------------------
Almost everything this repository does is built on DECLARED COORDINATES. T1's
cubes are at (+/-0.42, 0.45) and (+/-0.48, 0.45) because a file says so; the
table is at 1.250 because a file says so; the pads are at (+/-0.29, 0.50)
because a file says so. All of it is verified, N=10, over densified paths --
and none of it survives contact with a real cell, because the real cell has a
different table at a different height with different objects on it.

The pieces that could sense the world instead already exist and are NOT
COMPOSED:

  * `table_scene.analyse` finds a support plane and the objects on it, from
    ONE view, and is used by `pick_from_table` to print a description;
  * `joint_planner.VoxelWorld` lets the planner avoid things that are not the
    wearer -- and is constructed NOWHERE outside its own tests, so the
    planner has never known the table exists;
  * `grasp_pipeline.plan_grasp` plans a grasp from a cloud, for one prompt,
    with no memory between calls;
  * `work_surface.set_measured()` exists so the surface height can come from
    depth, and CLAUDE.md records that nothing calls it.

So the robot has eyes, a planner that can avoid obstacles, and a place to put
the answer -- and no stage that joins them. This module is that stage's
memory: many views in, one map out, and the map is what the planner and the
grasp layer read INSTEAD of a coordinate table.

WHAT IT REFUSES TO DO
---------------------
It does not invent. A map built from one view says so and carries the shell
caveat; a map with no plane REFUSES rather than assuming z; an object whose
top was never observed is reported with `top_seen=False` so a caller can
decide, and is never silently given a made-up height. Every field carries its
PROVENANCE -- which views contributed, and whether the number is measured or
assumed -- because a map that cannot say where a number came from is a
coordinate table with extra steps.

WHAT IS MEASURED HERE AND WHAT IS NOT, ON THIS MACHINE, TODAY
-------------------------------------------------------------
NO CAMERA HAS EVER BEEN ATTACHED TO THIS HOST. `scene_camera_node` refuses
rather than republishing and the scene camera's extrinsic has 0% coverage on
real recordings. So every cloud this module has ever been given came from the
MOCK RGB-D camera or from constructed test data. The fusion, the refusals and
the geometry are real and tested; "the robot has seen the room" is not, and
`Map.provenance` says which sources actually contributed rather than listing
the ones that could have.
"""
from __future__ import annotations

import json
import math

import numpy as np

from srl_perception import table_scene as TS

# A view contributes nothing useful below this many points: the plane fit and
# the clustering both need a population, and a handful of stray returns is
# noise wearing the shape of evidence.
MIN_POINTS_PER_VIEW = 60

# The gripper's own span. Shared with `segment_lift` so one number decides
# graspability wherever it is asked.
JAW_M = 0.085

# Resolution at which a view's FOOTPRINT on the work surface is recorded.
# Coarse on purpose: this answers "which viewpoints see this patch of table",
# not "where is the object", and a fine grid would make the cover problem
# large without making the answer better.
FOOTPRINT_M = 0.05
# How far above and below the plane counts as the working band a view covers.
FOOTPRINT_BAND_M = (-0.01, 0.25)

# Two object centres closer than this are the SAME DETECTION seen twice, not
# two objects. This is a dedup of near-coincident clusters across views and
# nothing else.
#
# IT WAS 0.06 -- the gripper's usable jaw width -- ON THE ARGUMENT THAT THINGS
# CLOSER TOGETHER THAN A JAW SPAN CANNOT BE PICKED SEPARATELY. That argument
# is wrong twice over. Objects 60 mm apart can be picked separately, one after
# the other; and the merge does not just refuse them, it FUSES THEM INTO ONE
# OBJECT with the combined extent, which is then reported as too wide for the
# jaws and refused with a confident reason.
#
# MEASURED, on the third live run of the calibration stage: T1's cubes sit
# 60 mm apart, and the map came back with a "140 mm" object where two 40 mm
# cubes are -- 100 mm of span plus one cube. The pick would have been refused
# on an object that is not there. My own control for the merge rule used
# objects 200 mm apart and could never have caught it.
#
# 20 mm is a dedup distance: two clusters whose centres are within a fifth of
# a cube are one cube seen twice.
MERGE_M = 0.02

# How close two POINTS have to be to belong to the same object. Passed to
# `table_scene.objects_on_plane`, whose default is 0.020 -- and T1's cubes are
# 60 mm apart with 40 mm faces, so the GAP between them is exactly 20 mm and
# the default joins them into one blob. Tighter, so a 20 mm gap separates.
CLUSTER_M = 0.012


class MapRefusal(Exception):
    """No map. Says which stage refused and why, never a partial answer."""


class View:
    """One look: points in the ROBOT frame, and where they came from."""

    def __init__(self, points, source, pose=None, note="", objects=None,
                 viewpoint=None):
        self.points = np.asarray(points, float).reshape(-1, 3)
        self.source = str(source)
        self.pose = None if pose is None else [float(v) for v in pose]
        self.note = note
        # THE INSTANCES THIS VIEW SAW, if a segmenter was run on it.
        #
        # Object identity comes from the PICTURE, one frame at a time -- see
        # `segment_lift`. It is carried per view rather than recomputed on the
        # fused pile because a mask belongs to the frame it was cut from: once
        # nine views are stacked, the only tool left is distance, and distance
        # is exactly what cannot separate two cubes with a 20 mm gap.
        # `None` MEANS NOT SEGMENTED. `[]` MEANS SEGMENTED AND NOTHING THERE.
        #
        # These were collapsed with `if objects else None`, so a view of bare
        # table -- a perfectly good, informative view -- was filed as "no
        # segmenter ran". Measured on the first two-arm sweep: one cell of
        # nine returned 0 regions, and `build` then refused the whole map for
        # mixing two finders, having been handed 8 segmented views and 1
        # supposedly unsegmented one. An empty surface is an ANSWER.
        self.objects = None if objects is None else list(objects)
        # WHERE THIS VIEW WAS TAKEN FROM, well enough to go back.
        #
        # A full sweep is minutes of arm motion, and most of it is spent
        # covering surface that has not changed. Recording the viewpoint makes
        # a view REPEATABLE, which is what turns "sweep everything again" into
        # "go back to the four poses that see this corner".
        self.viewpoint = dict(viewpoint) if viewpoint else None

    @property
    def n(self):
        return int(len(self.points))

    def __repr__(self):
        return "View(%s, %d points)" % (self.source, self.n)


class Obj:
    """One thing found on a surface, with where it was seen from."""

    def __init__(self, centre, extents, width_m, top_seen, n_points, seen_by,
                 graspable, why="", mean_bgr=None):
        self.centre = [round(float(v), 5) for v in centre]
        self.extents = [round(float(v), 5) for v in extents]
        self.width_m = round(float(width_m), 5)
        self.top_seen = bool(top_seen)
        # WHAT COLOUR IT IS, when a segmenter saw it. `None` from the
        # clustering finder, which sees no picture -- and a consumer asking
        # for a colour is then told WHY rather than handed a default.
        self.mean_bgr = (None if mean_bgr is None
                         else [round(float(v), 1) for v in mean_bgr])
        self.n_points = int(n_points)
        self.seen_by = list(seen_by)
        self.graspable = bool(graspable)
        self.why = why

    def as_dict(self):
        return dict(centre=self.centre, extents=self.extents,
                    width_m=self.width_m, top_seen=self.top_seen,
                    n_points=self.n_points, seen_by=self.seen_by,
                    graspable=self.graspable, why=self.why,
                    mean_bgr=self.mean_bgr)

    def __repr__(self):
        return ("Obj(%s, %.0f mm wide, %s, seen by %d view(s))"
                % (self.centre, self.width_m * 1000,
                   "graspable" if self.graspable else "NOT graspable",
                   len(self.seen_by)))


class Map:
    """Surfaces, objects, occupancy -- and where every one of them came from."""

    def __init__(self, plane, objects, points, provenance):
        self.plane = plane
        self.objects = objects
        self.points = points
        self.provenance = provenance

    def plane_z_at(self, x, y):
        """The support surface's height AT (x, y). A plane is not a number."""
        n = self.plane.normal
        return float((self.plane.offset - x * n[0] - y * n[1])
                     / max(abs(n[2]), 1e-9))

    @property
    def surface_z(self):
        """Height of the support surface in the robot frame, MEASURED.

        EVALUATED WHERE THE SURFACE IS, WHICH IS NOT THE ORIGIN.

        This returned `offset / n_z` -- the plane's z-intercept at x = y = 0.
        The support plane is fitted with a tolerance and comes out slightly
        tilted, so its intercept is not its height anywhere the table exists;
        on this rig x = y = 0 is inside the WEARER, half a metre from the
        nearest tabletop.

        MEASURED, on constructed geometry that reproduces the live case -- a
        tabletop at 0.9000 with its 35 mm front edge face in view, which is
        what the sweep sees at 38.9 degrees:

            tolerance   tilt    z at origin   z at the table   top median
            0.008      0.66 deg    0.8934        0.8994        +0.0004
            0.004      0.24 deg    0.8977        0.8998        +0.0002

        The plane was right to under a millimetre the whole time. The reported
        number was 6.6 mm low because it was asked about a place with no table
        in it. The live map's "-5.2 mm surface error" was this, and I had
        already written a plane-refinement to correct a bias that did not
        exist -- which is the standing rule exactly: a surprising measurement
        is evidence about the INSTRUMENT until the instrument is cleared.

        `work_surface.set_measured()` wants the height of the work surface, so
        that is what this gives: the plane evaluated at the centroid of the
        points lying on it.
        """
        pts = self.points
        if len(pts):
            h = self.plane.height(pts)
            near = pts[np.abs(h) <= 0.01]
            if len(near) >= 50:
                c = near.mean(axis=0)
                return round(self.plane_z_at(float(c[0]), float(c[1])), 5)
        return round(float(self.plane.offset
                           / max(abs(self.plane.normal[2]), 1e-9)), 5)

    def graspable(self):
        return [o for o in self.objects if o.graspable]

    def occupancy(self):
        """The points a planner should treat as solid, in the robot frame.

        Everything observed: the surface AND the objects. The planner's own
        `ignore_within_m` is what keeps the gripper from voxelising itself;
        deciding here what the planner may see would be this module guessing
        at somebody else's safety margin.
        """
        return self.points

    def height_histogram(self, bin_m=0.01, lo=None, hi=None):
        """How many points at each height. WHAT SURFACES ARE ACTUALLY THERE.

        The plane fit returns ONE surface -- the biggest -- and reports it
        with a confident number. That is not enough to tell a table from the
        shelf under it, or from a mixture of the two, and the first live run
        of the calibration stage measured a support surface **36.6 mm below**
        the table the mock renders, with no way to see why from the answer
        alone. A histogram is the cheapest thing that distinguishes "the
        table is lower than I thought" from "I fitted a plane across two
        surfaces".
        """
        z = np.asarray(self.points, float)[:, 2]
        if len(z) == 0:
            return []
        lo = float(z.min()) if lo is None else lo
        hi = float(z.max()) if hi is None else hi
        n = max(1, int(math.ceil((hi - lo) / bin_m)))
        counts, edges = np.histogram(z, bins=n, range=(lo, lo + n * bin_m))
        return [dict(z_m=round(float(edges[i]), 4), n=int(counts[i]))
                for i in range(n) if counts[i]]

    def surfaces(self, bin_m=0.01, min_frac=0.05):
        """Every height that holds a substantial share of the points.

        Reported alongside the fitted plane so a caller can SEE whether the
        one it was given is the only candidate.
        """
        h = self.height_histogram(bin_m)
        tot = sum(b["n"] for b in h) or 1
        return [dict(b, frac=round(b["n"] / tot, 4))
                for b in h if b["n"] / tot >= min_frac]

    def as_dict(self):
        return dict(
            heights=self.surfaces(),
            surface=dict(z_m=self.surface_z,
                         normal=[round(float(v), 5) for v in self.plane.normal],
                         tilt_deg=round(float(self.plane.tilt_deg), 3),
                         inliers=int(len(self.plane.inliers))),
            objects=[o.as_dict() for o in self.objects],
            n_points=int(len(self.points)),
            provenance=self.provenance)

    def describe(self):
        """What the robot would SAY it found. One line per fact."""
        out = ["support surface at z = %.4f m, tilted %.2f deg, from %d points"
               % (self.surface_z, self.plane.tilt_deg,
                  len(self.plane.inliers))]
        if not self.objects:
            out.append("nothing on it that is big enough to be an object")
        for i, o in enumerate(self.objects):
            out.append(
                "object %d at (%.3f, %.3f, %.3f), %.0f mm across the jaws, %s%s"
                % (i, o.centre[0], o.centre[1], o.centre[2], o.width_m * 1000,
                   "graspable" if o.graspable else "NOT graspable",
                   "" if o.graspable else " -- " + o.why))
        n = self.provenance["n_views"]
        out.append("built from %d view%s: %s"
                   % (n, "" if n == 1 else "s",
                      ", ".join(self.provenance["sources"])))
        if n == 1:
            out.append("ONE view only, so every object's far side is inferred "
                       "from the support plane and not observed")
        return out

    def __repr__(self):
        return ("Map(surface z=%.4f, %d object(s), %d point(s), %d view(s))"
                % (self.surface_z, len(self.objects), len(self.points),
                   self.provenance["n_views"]))


def _merge(objs):
    """Objects from one clustering, deduplicated across views by proximity.

    The clustering runs on the UNION of the views, so an object seen twice is
    normally already one cluster. This catches the case the union does not:
    two views whose clouds do not quite touch, which leaves two clusters a few
    millimetres apart where there is one object.
    """
    out = []
    for o in objs:
        for k, prev in enumerate(out):
            if math.dist(prev.centre, o.centre) < MERGE_M:
                keep = prev if prev.n_points >= o.n_points else o
                out[k] = Obj(keep.centre, keep.extents, keep.width_m,
                             prev.top_seen or o.top_seen,
                             prev.n_points + o.n_points,
                             sorted(set(prev.seen_by) | set(o.seen_by)),
                             keep.graspable, keep.why)
                break
        else:
            out.append(o)
    return out


def footprint(points, plane, cell_m=FOOTPRINT_M, band=FOOTPRINT_BAND_M):
    """Which patches of the work surface a view actually saw.

    The set of (i, j) cells, at `cell_m`, containing points inside the working
    band above the plane. This is a view's COVERAGE -- what re-measuring from
    that viewpoint would refresh -- and it is what makes it possible to ask
    for the few viewpoints that between them see the whole table.
    """
    pts = np.asarray(points, float).reshape(-1, 3)
    if not len(pts):
        return set()
    h = plane.height(pts)
    sel = pts[(h >= band[0]) & (h <= band[1])]
    if not len(sel):
        return set()
    ij = np.floor(sel[:, :2] / cell_m).astype(np.int64)
    return {(int(a), int(b)) for a, b in np.unique(ij, axis=0)}


def observe_set(views, plane, cover_frac=0.98, cell_m=FOOTPRINT_M):
    """The FEWEST viewpoints that between them see the surface the sweep saw.

    WHY. A full sweep is 116 views and twenty-eight minutes, and almost all of
    it re-measures table that has not moved. But the table is the thing that
    does not move -- what changes is the objects on it, and a handful of well
    chosen viewpoints already see every part of the surface those objects can
    be on. Finding that handful turns "sweep the volume again" into "look from
    six places".

    Greedy set cover: repeatedly take the viewpoint adding the most
    not-yet-covered surface, until `cover_frac` of the swept area is covered.
    Greedy is not optimal and is within a log factor, which is not the
    interesting property here -- the interesting one is that the answer is
    MEASURED from views the arm actually reached, so every viewpoint in it is
    known to be reachable and known to return points.
    """
    fps = []
    for v in views:
        if v.viewpoint is None:
            continue
        fp = footprint(v.points, plane, cell_m=cell_m)
        if fp:
            fps.append((v.source, v.viewpoint, fp))
    if not fps:
        return [], 0.0
    total = set()
    for _s, _vp, fp in fps:
        total |= fp
    want = int(math.ceil(cover_frac * len(total)))
    covered, chosen = set(), []
    remaining = list(fps)
    while remaining and len(covered) < want:
        remaining.sort(key=lambda t: -len(t[2] - covered))
        src, vp, fp = remaining.pop(0)
        gain = fp - covered
        if not gain:
            break
        covered |= gain
        chosen.append(dict(source=src, viewpoint=vp, cells=len(fp),
                           new_cells=len(gain)))
    return chosen, (len(covered) / max(1, len(total)))


def _overlaps(lo_a, hi_a, lo_b, hi_b, pad_m=MERGE_M, frac=0.5):
    """Do two instances occupy the same space, in all three axes?

    A FIXED CENTRE DISTANCE CANNOT DO THIS JOB and the live run showed why.
    Two views of one 160 mm pad put its centre 60 mm apart -- each saw a
    different part of it -- so at a 20 mm merge distance ONE pad came back as
    FOUR objects. Widen the distance to fix that and two 40 mm cubes on a
    60 mm pitch merge again, which is the defect this whole change is about.
    There is no single distance that is right for both, because the right
    answer scales with the object.

    Overlap does scale. Two partial views of one pad overlap heavily; two
    cubes 60 mm apart with 40 mm bodies do not touch.

    ALL THREE AXES, INCLUDING THE VERTICAL, and that is not tidiness. A cube
    STANDS ON a pad: their centres are ~30 mm apart horizontally-nothing, so
    any horizontal-only test merges the cube into the pad it is sitting on and
    the map loses the only object anybody wanted to pick up.

    AND TOUCHING IS NOT OVERLAPPING. A plain box-intersection test still
    merged the cube into the pad, because a cube standing on a pad is INSIDE
    it in x and y and shares a face in z -- the boxes intersect in every axis
    by the letter of the test. What separates them is that the shared part is
    a face and not a volume: the cube occupies 10 to 50 mm above the pad's
    top, the pad occupies 0 to 10, and those two ranges have nothing in
    common. So each axis must overlap by a real FRACTION of the smaller
    object's extent in that axis, not merely fail to be disjoint.

    `pad_m` is the tolerance for depth noise on a thin axis.

    THE GROUP'S BOX IS ITS SEED'S AND IT DOES NOT GROW. Expanding it to the
    union of everything merged so far chains objects together transitively: a
    partial view of a cube joins the cube, the box grows to cover both, and the
    NEXT cube 60 mm away then overlaps the grown box and is swallowed. Two
    cubes measured EXACTLY -- (0.420, 0.450) and (0.480, 0.450), 40.3 mm each
    against a 40 mm truth -- came out of the fusion as one object at
    (0.450, 0.450), their midpoint, where nothing is. Segs are compared
    against the seed, which is the largest member because the list is sorted
    by point count, and the seed is the best-observed instance in the group.
    """
    lo_a, hi_a = np.asarray(lo_a, float), np.asarray(hi_a, float)
    lo_b, hi_b = np.asarray(lo_b, float), np.asarray(hi_b, float)
    inter = np.minimum(hi_a, hi_b) - np.maximum(lo_a, lo_b)
    smaller = np.minimum(hi_a - lo_a, hi_b - lo_b)
    # Relative to the SMALLER object's own extent, with only a floor to keep a
    # perfectly flat thing from dividing by nothing. Padding this by a fixed
    # tolerance is what let a cube share a face with its pad and count as
    # overlapping it.
    need = frac * np.maximum(smaller, 1e-4)
    return bool(np.all(inter >= need))


# The largest gap that is NOT a gap between objects. Below the smallest real
# separation in any scene this rig works with -- T1's cubes leave 20 mm between
# their faces -- and well above the depth noise, so a solid face never splits.
SPLIT_GAP_M = 0.012

# HOW MUCH EVIDENCE AN OBJECT NEEDS BEFORE IT IS ONE.
#
# The full two-arm sweep produced 14 objects where 6 exist. The six are solid
# -- 110 000 to 500 000 points each, seen from 47 to 66 viewpoints. The other
# eight are 25 to 77 points from one or two views: slivers at the feet of real
# cubes, edge returns, a corner of the table caught at a bad angle.
#
# They are not wrong so much as UNSUPPORTED, and the difference between them
# and a real object is four orders of magnitude of evidence. An object the
# whole sweep saw once, with thirty points, is a detection this map should not
# stake a grasp on -- and `pick_from_map` would happily plan one.
#
# Kept in the map under `weak`, because deleting them silently would hide a
# real return, and a scene where EVERYTHING is weak is a fact worth seeing.
MIN_OBJECT_POINTS = 200
MIN_OBJECT_VIEWS = 2


def _split_disconnected(pts, gap_m=SPLIT_GAP_M, min_points=25):
    """One point set -> the separated lumps it is actually made of.

    A LAST CONSISTENCY CHECK ON THE FUSION, and it is not a retreat to
    clustering. Segmentation still decides what an object is, one frame at a
    time. This asks a different and much weaker question of the RESULT: are
    this object's own points in one connected piece?

    IT WAS NEEDED. With everything else fixed, the map still reported one
    object at (0.450, 0.450) with a LONG EXTENT OF 0.101 m -- two 40 mm cubes
    on a 60 mm pitch, fused, sitting at their midpoint where nothing is. It
    reads 41 mm across its narrowest axis, so it is reported as GRASPABLE, and
    the arm would have closed on the gap between two cubes. `segment_lift`'s
    own self-test predicted exactly this: a fused pair keeps the width of one.

    The fusion glues them because in some views the segmenter returns the pair
    as a single region -- legitimately, they are the same colour and adjacent
    -- and that wide instance becomes the group's seed.

    A threshold is unavoidable here, but it is not a tuning value: it has to
    be smaller than the smallest gap between two things anyone would call
    separate, and larger than the depth noise across one face. 12 mm sits
    between 20 mm and about 2 mm with room either side.
    """
    pts = np.asarray(pts, float)
    if len(pts) < 2 * min_points:
        return [pts]
    key = np.floor(pts / gap_m).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    index = {tuple(k): i for i, k in enumerate(uniq)}
    label = np.full(len(uniq), -1, np.int64)
    nbr = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1)
           for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]
    n_lab = 0
    for start in range(len(uniq)):
        if label[start] >= 0:
            continue
        stack = [start]
        label[start] = n_lab
        while stack:
            cur = uniq[stack.pop()]
            for d in nbr:
                j = index.get((cur[0] + d[0], cur[1] + d[1], cur[2] + d[2]))
                if j is not None and label[j] < 0:
                    label[j] = n_lab
                    stack.append(j)
        n_lab += 1
    out = []
    for k in range(n_lab):
        sel = pts[label[inv] == k]
        if len(sel) >= min_points:
            out.append(sel)
    return out or [pts]


def _rejoin_vertical_slices(items, foot_ratio=1.45, gap_m=0.010):
    """Put back together an object the height split cut in half.

    WHAT MAKES THE SLICE. `segment_lift` separates a cube from the pad it
    stands on by cutting the mask at the modal height plus a step. When ONE
    mask covers a pad AND the cube on it, the mode is the pad's top face and
    the cut lands part-way up the cube -- so the cube's lower band is filed
    with the pad and comes out as its own object. The live map had four:
    7 mm, 21 mm, 28 mm and 58 mm pieces standing exactly at the feet of real
    cubes, all reported graspable.

    HOW IT DIFFERS FROM A CUBE ON A PAD, which must NOT be rejoined. The two
    cases look identical to a containment test -- both are one thing resting
    on another, touching, one inside the other's footprint. What separates
    them is the FOOTPRINT RATIO. A slice of a cube has the cube's own
    footprint, because it is the same cube: 40 x 40 under 40 x 40. A cube on
    a pad is 40 x 40 under 160 x 140, four times over.

    So: touching vertically, and horizontal footprints within `foot_ratio` of
    each other, is one object cut in two. Anything else is left alone.
    """
    items = sorted(items, key=lambda t: -len(t[1]))
    used = [False] * len(items)
    out = []
    for i, (oi, pi) in enumerate(items):
        if used[i]:
            continue
        pts = pi
        lo = pts.min(axis=0)
        hi = pts.max(axis=0)
        for j in range(i + 1, len(items)):
            if used[j]:
                continue
            oj, pj = items[j]
            jlo, jhi = pj.min(axis=0), pj.max(axis=0)
            # horizontal overlap, in both axes, by most of the smaller
            inter = np.minimum(hi[:2], jhi[:2]) - np.maximum(lo[:2], jlo[:2])
            small = np.minimum(hi[:2] - lo[:2], jhi[:2] - jlo[:2])
            if not np.all(inter >= 0.6 * np.maximum(small, 1e-4)):
                continue
            # touching in z, one directly under the other
            if min(hi[2], jhi[2]) < max(lo[2], jlo[2]) - gap_m:
                continue
            # THE DISCRIMINATOR: same footprint means same object.
            a = float(np.prod(np.maximum(hi[:2] - lo[:2], 1e-4)))
            b = float(np.prod(np.maximum(jhi[:2] - jlo[:2], 1e-4)))
            if max(a, b) / min(a, b) > foot_ratio:
                continue
            used[j] = True
            pts = np.vstack([pts, pj])
            lo, hi = pts.min(axis=0), pts.max(axis=0)
        out.append((oi, pts))
    return out


def _drop_contained(objs, frac=0.6):
    """Remove objects that are just a PIECE of a bigger one.

    The merge groups instances that overlap its seed; a fragment sitting
    against the seed but not overlapping it -- the bottom band of a cube, say,
    which is BELOW the body the seed measured -- forms its own group and
    survives as an object. The live map reported ten where six exist.

    Whole-object containment is a different question from the merge's, and it
    is asked once, at the end, of the finished list. The bigger object wins
    because it is the better-observed one. A cube standing on a pad is not
    contained in it -- their height ranges are disjoint, which is the axis
    this test has always turned on.
    """
    keep = []
    for o in sorted(objs, key=lambda o: -o.n_points):
        lo = np.array(o.centre) - np.array(o.extents) / 2.0
        hi = np.array(o.centre) + np.array(o.extents) / 2.0
        for b in keep:
            blo = np.array(b.centre) - np.array(b.extents) / 2.0
            bhi = np.array(b.centre) + np.array(b.extents) / 2.0
            if _overlaps(lo, hi, blo, bhi, frac=frac):
                break
        else:
            keep.append(o)
    return keep


def _fuse_segments(usable, plane):
    """Per-view instances -> one object list. The merge is now honest.

    Two Segs are the SAME OBJECT when the space they occupy OVERLAPS -- see
    `_overlaps`. That is a real dedup question, did two views see one thing,
    and it is asked of instances that were already separated in their own
    pictures, so it is no longer doing the job clustering was failing at.
    """
    segs = []
    for v in usable:
        for o in (v.objects or []):
            segs.append((v.source, o))
    segs.sort(key=lambda t: -t[1].n_points)

    groups = []
    for src, o in segs:
        for g in groups:
            if _overlaps(o.lo, o.hi, g["lo"], g["hi"]):
                g["members"].append((src, o))
                break
        else:
            groups.append(dict(lo=np.asarray(o.lo, float),
                               hi=np.asarray(o.hi, float),
                               members=[(src, o)]))

    from srl_perception.segment_lift import _measure
    pieces = []
    for g in groups:
        mem = g["members"]
        allpts = np.vstack([m[1].points for m in mem])
        w = np.array([m[1].n_points for m in mem], float)
        cols = np.array([m[1].mean_bgr for m in mem], float)
        mean_bgr = list((cols * w[:, None]).sum(axis=0) / w.sum())
        srcs = sorted({m[0] for m in mem})
        for pts in _split_disconnected(allpts):
            pieces.append(((srcs, mean_bgr), pts))

    out = []
    for (srcs, mean_bgr), pts in _rejoin_vertical_slices(pieces):
        if True:
            # RE-MEASURED ON THE UNION, so an object seen from two sides has
            # its far face too. The centre still comes from the mid-extent,
            # which is what stops a one-sided view biasing it toward the
            # camera.
            centre, extents, width = _measure(pts)
            top_seen = float(plane.height(pts).max()) > 0.005
            out.append(Obj(centre, extents, width, top_seen, len(pts),
                           srcs,
                           width <= JAW_M,
                           "" if width <= JAW_M else
                           ("%.0f mm across its narrowest axis and the jaws "
                            "close on %.0f mm"
                            % (width * 1000, JAW_M * 1000)),
                           mean_bgr=mean_bgr))
    out = _drop_contained(out)
    out.sort(key=lambda o: (-o.n_points))
    return out


def views_needed(n_views):
    """How many sightings an object needs, given how many were TAKEN.

    THE THRESHOLD CANNOT BE A CONSTANT AND I HAD IT AS ONE.

    `MIN_OBJECT_VIEWS = 2` is right for a full sweep, where 73 views see every
    object dozens of times and a single sighting really is a sliver. It is
    nonsense for a re-look, which visits THREE viewpoints on purpose -- there,
    an object seen once may have been seen by every view that could see it.

    Measured, and it emptied the map: the first re-look found the rearranged
    objects, demoted every one of them to `weak` for having too few sightings,
    and reported all six objects GONE. A fast re-measure that concludes the
    table is empty is worse than no re-measure at all, because it looks like
    an answer.

    So the requirement is relative to the opportunity: with one or two views,
    one sighting is all there is to have.
    """
    return 1 if int(n_views) <= 2 else MIN_OBJECT_VIEWS


def _split_weak(objs, n_views=None):
    """(solid, weak). Evidence, not geometry -- see MIN_OBJECT_POINTS."""
    need = MIN_OBJECT_VIEWS if n_views is None else views_needed(n_views)
    solid, weak = [], []
    for o in objs:
        if o.n_points >= MIN_OBJECT_POINTS and len(o.seen_by) >= need:
            solid.append(o)
        else:
            weak.append(o)
    return solid, weak


def build(views, up=(0.0, 0.0, 1.0), **kw):
    """Many views -> one map. Refuses rather than guessing.

    `views` are `View` objects whose points are ALREADY in the robot frame.
    """
    views = [v for v in views if isinstance(v, View)]
    if not views:
        raise MapRefusal("no views: a map of nothing is not a map")
    usable = [v for v in views if v.n >= MIN_POINTS_PER_VIEW]
    if not usable:
        raise MapRefusal(
            "every one of the %d view(s) has fewer than %d points (largest "
            "%d). A plane fitted to that many returns is noise with a "
            "confident number on it."
            % (len(views), MIN_POINTS_PER_VIEW, max(v.n for v in views)))

    cloud = np.vstack([v.points for v in usable])
    # WHICH VIEW EACH POINT CAME FROM, kept so an object can say what saw it.
    owner = np.concatenate([np.full(v.n, i) for i, v in enumerate(usable)])
    kw.setdefault("cluster_m", CLUSTER_M)
    # A TIGHTER PLANE TOLERANCE, because the tilt is what does the damage.
    #
    # The table is a 35 mm slab and its front edge face is in view at the
    # sweep's elevation. At the shipped 8 mm tolerance the fit tips 0.66 deg
    # to catch the upper strip of that edge; at 4 mm it tips 0.24. A tilted
    # support plane sits BELOW the tabletop over part of the table, and the
    # height gate is 4 mm -- so the table clears its own surface there and the
    # map reports slabs of tabletop as objects. Two of them, 231 x 392 mm and
    # 79 x 314 mm, in the run that prompted this.
    kw.setdefault("tol_m", 0.004)
    try:
        scene = TS.analyse(cloud, up=up, **kw)
    except TS.SceneRefusal as e:
        raise MapRefusal("the support surface could not be fitted: %s" % e)
    plane = scene["plane"]

    # ============================================================
    # WHERE THE OBJECTS COME FROM, and it is recorded in the map
    # ============================================================
    # If the views were segmented, their instances are fused. Otherwise the
    # fused cloud is clustered, which is the old path and is kept because it
    # is the only one that works with no model available -- but the two are
    # NEVER mixed and the map says which ran, because they fail differently.
    segmented = [v for v in usable if v.objects is not None]
    if segmented and len(segmented) == len(usable):
        objs, weak = _split_weak(_fuse_segments(usable, plane), len(usable))
        # THE FEW VIEWPOINTS THAT SEE THE WHOLE SURFACE, learned from the
        # sweep that just ran. `relook` uses these instead of sweeping.
        obs, obs_cover = observe_set(usable, plane)
        prov_finder = "segment_lift: instances cut from each PICTURE by " \
                      "FastSAM and lifted through the depth"
        return Map(plane, objs, cloud,
                   dict(n_views=len(usable),
                        sources=[v.source for v in usable],
                        points_per_view=[v.n for v in usable],
                        objects_per_view=[len(v.objects) for v in usable],
                        observe_set=obs,
                        observe_set_cover=round(obs_cover, 4),
                        observe_set_note=(
                            "the fewest viewpoints covering %.0f%% of the "
                            "surface this sweep saw. Every one was reached "
                            "and returned points, so it is not a prediction."
                            % (obs_cover * 100)),
                        weak_detections=[o.as_dict() for o in weak],
                        weak_rule=("under %d points or seen by fewer than %d "
                                   "views. Kept, not deleted -- a scene where "
                                   "everything is weak is worth seeing."
                                   % (MIN_OBJECT_POINTS, MIN_OBJECT_VIEWS)),
                        discarded_views=[dict(source=v.source, n=v.n)
                                         for v in views
                                         if v.n < MIN_POINTS_PER_VIEW],
                        object_finder=prov_finder,
                        surface_z_m="MEASURED from the fused cloud",
                        single_view_caveat=(len(usable) == 1)))
    if segmented:
        raise MapRefusal(
            "%d of %d views were segmented and the rest were not. A map that "
            "mixed the two would have objects found two different ways with "
            "one confidence written on all of them."
            % (len(segmented), len(usable)))

    objs = []
    for row in scene["rows"]:
        o = row["object"]
        # Which views contributed to THIS object, by nearest-point ownership.
        d = np.linalg.norm(cloud[:, None, :] - o.points[None, :, :], axis=2) \
            if len(o.points) < 400 and len(cloud) < 4000 else None
        if d is not None:
            seen = sorted({usable[owner[k]].source
                           for k in np.unique(np.argmin(d, axis=0))})
        else:
            seen = sorted({v.source for v in usable})
        # THE TOP HAS TO HAVE BEEN SEEN for the height to be a measurement.
        # `table_scene` puts the centre at half the observed height, so an
        # object whose top is out of frame is reported LOW and nothing says
        # so unless this does.
        top_seen = bool(len(o.points) and
                        float(plane.height(o.points).max()) > 0.005)
        objs.append(Obj(o.centre, o.extents if hasattr(o, "extents")
                        else [o.length_m, o.width_m, o.top_m],
                        o.width_m, top_seen, len(o.points), seen,
                        row["grasp"] is not None, row["why"]))
    objs = _merge(objs)

    prov = dict(n_views=len(usable),
                sources=[v.source for v in usable],
                points_per_view=[v.n for v in usable],
                discarded_views=[dict(source=v.source, n=v.n)
                                 for v in views if v.n < MIN_POINTS_PER_VIEW],
                object_finder=("table_scene: the fused cloud clustered at "
                               "%.3f m. Two objects closer than that are ONE."
                               % CLUSTER_M),
                surface_z_m="MEASURED from the fused cloud",
                single_view_caveat=(len(usable) == 1))
    return Map(plane, objs, cloud, prov)


def save(m, path):
    with open(path, "w") as f:
        json.dump(m.as_dict(), f, indent=2, sort_keys=True)
    return path


# ------------------------------------------------------------- the self-test

def _plane_pts(z=0.90, n=4000, seed=1, x=(0.2, 0.7), y=(0.2, 0.6)):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(*x, n), rng.uniform(*y, n),
                            np.full(n, z)])


def _box(centre, size, n=900, seed=2, face=None):
    """A box. `face` gives ONE face only, which is what one camera returns."""
    rng = np.random.default_rng(seed)
    c, s = np.asarray(centre, float), np.asarray(size, float)
    if face == "-y":
        return np.column_stack([
            c[0] + rng.uniform(-s[0] / 2, s[0] / 2, n),
            np.full(n, c[1] - s[1] / 2),
            c[2] + rng.uniform(-s[2] / 2, s[2] / 2, n)])
    return np.column_stack([c[k] + rng.uniform(-s[k] / 2, s[k] / 2, n)
                            for k in range(3)])


def self_test(verbose=True):                                   # noqa: C901
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    Z = 0.90
    box = (0.40, 0.40, Z + 0.03)
    v1 = View(np.vstack([_plane_pts(Z), _box(box, (0.05, 0.05, 0.06))]),
              "wrist_left")
    m = build([v1])
    check("the surface height is MEASURED, not declared",
          abs(m.surface_z - Z) < 0.003, "%.4f m" % m.surface_z)
    check("the object is found", len(m.objects) == 1, repr(m.objects))
    if m.objects:
        o = m.objects[0]
        check("and it is where it really is",
              math.dist(o.centre, box) < 0.01,
              "%s vs %s" % (o.centre, list(box)))
        check("its jaw width is a measurement", 0.03 < o.width_m < 0.09,
              "%.1f mm" % (o.width_m * 1000))

    # TWO VIEWS OF THE SAME OBJECT ARE ONE OBJECT.
    v2 = View(np.vstack([_plane_pts(Z, seed=7), _box(box, (0.05, 0.05, 0.06),
                                                     seed=8)]), "wrist_right")
    m2 = build([v1, v2])
    check("two views of one object give ONE object", len(m2.objects) == 1,
          repr(m2.objects))
    check("and it records that both views saw it",
          m2.objects and len(m2.objects[0].seen_by) == 2,
          str(m2.objects[0].seen_by) if m2.objects else "")
    check("the map names its sources",
          m2.provenance["sources"] == ["wrist_left", "wrist_right"])
    check("a single-view map says it is a single view",
          m.provenance["single_view_caveat"] is True
          and m2.provenance["single_view_caveat"] is False)

    # TWO DIFFERENT OBJECTS STAY TWO. A merge rule that merges everything
    # would pass the test above and be useless.
    far = (0.60, 0.40, Z + 0.03)
    v3 = View(np.vstack([_plane_pts(Z), _box(box, (0.05, 0.05, 0.06)),
                         _box(far, (0.05, 0.05, 0.06), seed=9)]), "scene")
    m3 = build([v3])
    check("two separate objects stay two", len(m3.objects) == 2,
          repr(m3.objects))

    # REFUSALS, each by name.
    try:
        build([])
        check("refuses an empty view list", False)
    except MapRefusal:
        check("refuses an empty view list", True)
    try:
        build([View(np.zeros((5, 3)), "dust")])
        check("refuses a view with too few points", False)
    except MapRefusal as e:
        check("refuses a view with too few points", "fewer than" in str(e))
    try:
        build([View(np.random.default_rng(0).uniform(0, 1, (3000, 3)),
                    "noise")])
        check("refuses a cloud with no plane in it", False)
    except MapRefusal as e:
        check("refuses a cloud with no plane in it",
              "support surface" in str(e), str(e)[:60])

    # THE OCCUPANCY IS WHAT A PLANNER WOULD AVOID, and it must contain the
    # SURFACE -- the whole reason the planner swept through the bench is that
    # it had no idea the bench was there.
    occ = m.occupancy()
    on_plane = int(np.sum(np.abs(occ[:, 2] - Z) < 0.005))
    check("the occupancy includes the support surface", on_plane > 1000,
          "%d points on the plane" % on_plane)

    # A CHECK THAT CANNOT FAIL IS NOT A CHECK: an EMPTY table must produce a
    # map with a surface and NO objects, not a map with a phantom.
    m4 = build([View(_plane_pts(Z), "empty")])
    check("an empty surface gives no objects", not m4.objects,
          repr(m4.objects))
    check("and still gives a usable surface height",
          abs(m4.surface_z - Z) < 0.003)

    # WHAT IT SAYS. The narration reads this, so it has to be sentences.
    lines = m2.describe()
    check("it can describe itself in plain sentences",
          any("support surface at z" in ln for ln in lines)
          and any("object 0 at" in ln for ln in lines), str(lines[:2]))
    check("a one-view map warns that the far side is inferred",
          any("ONE view only" in ln for ln in m.describe()))

    if verbose:
        print("world_model self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
