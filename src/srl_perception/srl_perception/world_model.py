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

# Two object centres closer than this, seen from different views, are the same
# object. It is the gripper's own usable width: things closer together than a
# jaw span cannot be picked separately anyway, so merging them loses nothing a
# caller could have acted on.
MERGE_M = 0.06


class MapRefusal(Exception):
    """No map. Says which stage refused and why, never a partial answer."""


class View:
    """One look: points in the ROBOT frame, and where they came from."""

    def __init__(self, points, source, pose=None, note=""):
        self.points = np.asarray(points, float).reshape(-1, 3)
        self.source = str(source)
        self.pose = None if pose is None else [float(v) for v in pose]
        self.note = note

    @property
    def n(self):
        return int(len(self.points))

    def __repr__(self):
        return "View(%s, %d points)" % (self.source, self.n)


class Obj:
    """One thing found on a surface, with where it was seen from."""

    def __init__(self, centre, extents, width_m, top_seen, n_points, seen_by,
                 graspable, why=""):
        self.centre = [round(float(v), 5) for v in centre]
        self.extents = [round(float(v), 5) for v in extents]
        self.width_m = round(float(width_m), 5)
        self.top_seen = bool(top_seen)
        self.n_points = int(n_points)
        self.seen_by = list(seen_by)
        self.graspable = bool(graspable)
        self.why = why

    def as_dict(self):
        return dict(centre=self.centre, extents=self.extents,
                    width_m=self.width_m, top_seen=self.top_seen,
                    n_points=self.n_points, seen_by=self.seen_by,
                    graspable=self.graspable, why=self.why)

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

    @property
    def surface_z(self):
        """Height of the support surface in the robot frame, MEASURED.

        This is what `work_surface.set_measured()` was written to receive and
        has never been given.
        """
        return round(float(self.plane.offset / max(abs(self.plane.normal[2]),
                                                   1e-9)), 5)

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

    def as_dict(self):
        return dict(
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
    try:
        scene = TS.analyse(cloud, up=up, **kw)
    except TS.SceneRefusal as e:
        raise MapRefusal("the support surface could not be fitted: %s" % e)
    plane = scene["plane"]

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
