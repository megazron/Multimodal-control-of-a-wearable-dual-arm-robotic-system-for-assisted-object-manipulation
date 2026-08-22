#!/usr/bin/env python3
"""What is on the table, and how to pick each of it up.

    python3 -m srl_perception.table_scene        # the self-test

WHY THIS EXISTS. `rgbd_grasp.py` turns ONE mask into ONE grasp, and every
grasp this project has driven was hand-built for a cube whose position was
already known. Asked "what is on the table", the system had no answer: it
had a detector that finds a named thing and a planner that grasps a given
cloud, and nothing in between that looks at a surface and enumerates what is
resting on it. So the arm went where it was told and, when it was told wrong,
it moved around and picked nothing up.

THE SUPPORT PLANE IS NOT AN OPTIONAL EXTRA, and this is the lesson of
2026-08-21. Three separate corrections were needed on a hand-built grasp that
day and all three came from ignoring the surface the object rests on:

  * the jaw axis was 31 deg off vertical and would have driven the gripper
    into the table;
  * the grasp centre was 20 mm low, because a single depth view sees the
    FRONT of an object and the centroid of a front surface is not the centre
    of a solid;
  * the approach was nearly parallel to the jaw axis, which closes the
    fingers along the direction they are travelling.

Segment the plane first and all three become arithmetic. The plane gives the
object's BOTTOM for free -- it is resting on it -- so the centre height is
the plane plus half the visible height, which fixes the shell bias without a
second view. The plane's normal gives the approach. The plane's surface is
what the fingers must not go below.

THE FRAME IS THE ROBOT'S, AND THAT IS THE WHOLE POINT. Points come in already
transformed by the WRIST camera's pose, which is forward kinematics -- exact,
and needing no calibration. The scene camera's extrinsic has 0% coverage on
real recordings (see `docs/system/22_grasping.md`); nothing here depends on
it. That is why this pipeline can be trusted today and a scene-camera one
cannot.

WHAT IT REFUSES, and each refusal is a real failure mode:

  no plane          fewer than `min_inliers` points lie on any plane within
                    `max_tilt`. Fitting a "table" to a wall and grasping
                    against it is worse than saying nothing.
  nothing on it     the plane is there and no cluster stands on it.
  too wide          wider across its short horizontal axis than the jaws
                    open. The arm discovers this by colliding.
  too thin          shorter than the finger tips can reach without the
                    gripper body touching the surface.
  under the plane   a grasp whose fingers would go below the surface.

NO LEARNED MODEL, and that is a decision with a reason rather than a
limitation. A parallel jaw on a rigid object resting on a known plane is
solved geometry, this machine has 4 GB of VRAM shared with the detector, and
a geometric refusal can be READ. `docs/system/22_grasping.md` records where a
learned grasp model would earn its place and what would have to be true
first.
"""
from __future__ import annotations

import math

import numpy as np

# The Robotiq 85 opens 85 mm between the pads. The usable figure is smaller
# because the fingers need somewhere to go.
GRIPPER_MAX_M = 0.085
GRIPPER_MARGIN_M = 0.010
# HALF THE PAD HEIGHT. The Robotiq 85's contact pad is about 37 mm tall, so
# a grasp centred at height h puts the bottom of the pad at h - 18 mm.
#
# THE FIRST VERSION OF THIS WAS WRONG AND THE TEST CAUGHT IT. It was
# `FINGER_REACH_M = 0.035`, modelling the tips as reaching 35 mm BELOW the
# grasp centre, which refused a 30 mm cube -- and a 30 mm cube is the single
# most ordinary thing this rig picks up. You grasp a short object near its
# TOP, not through its middle; the constraint is that the pad's lower edge
# clears the table, not that the centre is a fixed height above it.
PAD_HALF_M = 0.018
# Clearance the fingers keep from the surface.
PLANE_MARGIN_M = 0.004


class SceneRefusal(Exception):
    """A refusal that names what could not be done and why."""


# ------------------------------------------------------------------- plane
class Plane:
    """A support surface: unit normal, offset, and the points on it."""

    def __init__(self, normal, offset, inliers, n_total):
        n = np.asarray(normal, float)
        self.normal = n / max(np.linalg.norm(n), 1e-12)
        self.offset = float(offset)
        self.inliers = inliers
        self.n_total = int(n_total)

    def height(self, pts):
        """Signed distance above the plane. Positive is off the surface."""
        return np.asarray(pts, float) @ self.normal - self.offset

    @property
    def tilt_deg(self):
        return math.degrees(math.acos(max(-1.0, min(1.0,
                                                    abs(self.normal[2])))))

    def __repr__(self):
        return ("Plane(normal=%s, offset=%.4f, %d/%d inliers, tilt %.1f deg)"
                % (np.round(self.normal, 4), self.offset,
                   len(self.inliers), self.n_total, self.tilt_deg))


def fit_support_plane(points, up=(0.0, 0.0, 1.0), tol_m=0.008,
                      max_tilt_deg=30.0, iters=400, min_inliers=200,
                      seed=0):
    """RANSAC a roughly-horizontal plane, and REFUSE if there is not one.

    `up` is the direction the surface's normal is expected to lie near, in
    the frame the points are in. In the robot frame that is world +z, and
    saying so is what stops this fitting the back wall -- which is a plane,
    has plenty of inliers, and is not a table.

    Ties are broken by LOWEST surface, not by most inliers alone: with a
    cluttered table the objects' own tops can out-vote the table, and a
    "plane" 80 mm up is a plane through the middle of the things being
    grasped.
    """
    P = np.asarray(points, float)
    if P.ndim != 2 or P.shape[1] != 3:
        raise SceneRefusal("points must be an (N, 3) array, got %s"
                           % (P.shape,))
    if len(P) < min_inliers:
        raise SceneRefusal(
            "only %d points; a support plane needs at least %d. A plane fitted "
            "to a handful of returns is confident and meaningless."
            % (len(P), min_inliers))
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    cos_max = math.cos(math.radians(max_tilt_deg))
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        idx = rng.choice(len(P), 3, replace=False)
        a, b, c = P[idx]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        if abs(float(n @ up)) < cos_max:
            continue                       # not a horizontal-ish surface
        if float(n @ up) < 0:
            n = -n                         # normal points UP, always
        d = float(n @ a)
        h = P @ n - d
        inl = np.abs(h) <= tol_m
        k = int(inl.sum())
        if k < min_inliers:
            continue
        # LOWEST wins among comparably supported planes. See the docstring.
        key = (k >= 0.9 * (best[0] if best else 0), -d) if best else (True, -d)
        if best is None or k > best[0] * 1.1 or (k > 0.9 * best[0]
                                                 and d < best[2]):
            best = (k, n, d, inl)
    if best is None:
        raise SceneRefusal(
            "no plane within %.0f deg of %s carried %d points in %d "
            "attempts. There is no support surface in this view -- refusing "
            "rather than grasping against a guess."
            % (max_tilt_deg, np.round(up, 2), min_inliers, iters))
    k, n, d, inl = best
    # Refit on the inliers: three random points fix the plane crudely, and a
    # least-squares refit over all of them is worth ~1 mm.
    Q = P[inl]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
    n2 = vt[2]
    if float(n2 @ up) < 0:
        n2 = -n2
    d2 = float(n2 @ c)
    return Plane(n2, d2, np.flatnonzero(inl), len(P))


# ----------------------------------------------------------------- objects
class TableObject:
    """One thing standing on the plane."""

    def __init__(self, points, plane, index=0, pixel_index=None):
        self.index = index
        # WHICH PIXELS THIS CLUSTER CAME FROM, when the caller supplied them.
        # Carried so a 3-D cluster can be matched against a 2-D detector
        # mask -- without it, "pick the red one" degrades to "pick the best
        # one", which is a different instruction wearing the same words.
        self.pixel_index = (None if pixel_index is None
                            else np.asarray(pixel_index, int))
        self.points = np.asarray(points, float)
        self.plane = plane
        h = plane.height(self.points)
        self.top_m = float(h.max())
        self.bottom_seen_m = float(h.min())
        # AXES IN THE PLANE. The vertical is the plane's normal; the two
        # horizontal axes come from the footprint, so "the short axis" is a
        # horizontal width the jaws can close across and never a diagonal
        # that happens to be short because the object is tall.
        self.up = plane.normal
        foot = self.points - np.outer(h, self.up)
        fc = foot.mean(axis=0)
        F = foot - fc
        # Work in a 2-D basis on the plane.
        e1 = np.array([1.0, 0.0, 0.0]) - self.up * float(self.up[0])
        if np.linalg.norm(e1) < 1e-6:
            e1 = np.array([0.0, 1.0, 0.0]) - self.up * float(self.up[1])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(self.up, e1)
        uv = np.stack([F @ e1, F @ e2], axis=1)
        if len(uv) >= 3:
            cov = np.cov(uv.T)
            w, V = np.linalg.eigh(cov)
            long2, short2 = V[:, 1], V[:, 0]
        else:
            long2, short2 = np.array([1.0, 0.0]), np.array([0.0, 1.0])
        self.long_axis = e1 * long2[0] + e2 * long2[1]
        self.short_axis = e1 * short2[0] + e2 * short2[1]
        self.width_m = float(np.ptp(uv @ short2)) if len(uv) else 0.0
        self.length_m = float(np.ptp(uv @ long2)) if len(uv) else 0.0

        # THE SHELL CORRECTION, AND THE PLANE IS WHAT MAKES IT POSSIBLE.
        #
        # One depth view sees the FRONT surface. The centroid of a front
        # surface sits towards the camera and, for an object on a table,
        # BELOW the true centre -- measured at 20 mm on 2026-08-21 with a
        # hand-built planner, against a 30 mm capture gate.
        #
        # The object is RESTING on the plane, so its bottom is the plane.
        # Its top is observed. The centre height is therefore the midpoint of
        # those two, which needs no second view and no symmetry assumption
        # about the horizontal axes.
        self.centre_height_m = self.top_m / 2.0
        self.centre = fc + self.up * self.centre_height_m
        self.visible_centroid = self.points.mean(axis=0)
        self.shell_bias_m = float(np.linalg.norm(self.centre
                                                 - self.visible_centroid))

    @property
    def n_points(self):
        return len(self.points)

    def describe(self):
        return ("object %d: %.0f x %.0f mm footprint, %.0f mm tall, "
                "centre (%.3f, %.3f, %.3f), %d points"
                % (self.index, self.width_m * 1000, self.length_m * 1000,
                   self.top_m * 1000, self.centre[0], self.centre[1],
                   self.centre[2], self.n_points))


def objects_on_plane(points, plane, min_height_m=0.012, max_height_m=0.40,
                     cluster_m=0.020, min_points=40, pixel_uv=None):
    """Everything standing on the surface, as separate objects.

    Clustering is a voxel flood fill rather than a library call: it needs no
    scikit-learn (which is not in the vision environment), it is O(n), and
    its one parameter -- the voxel size -- is the same `cluster_m` that says
    how far apart two things must be to be two things.
    """
    P = np.asarray(points, float)
    h = plane.height(P)
    on = (h > min_height_m) & (h < max_height_m)
    Q = P[on]
    # Indices back into the ORIGINAL point array, so a cluster can name the
    # pixels it came from.
    orig = np.flatnonzero(on)
    if len(Q) < min_points:
        return []
    key = np.floor(Q / cluster_m).astype(np.int64)
    lut = {}
    for i, k in enumerate(map(tuple, key)):
        lut.setdefault(k, []).append(i)
    seen = set()
    out = []
    for start in lut:
        if start in seen:
            continue
        stack, comp = [start], []
        seen.add(start)
        while stack:
            c = stack.pop()
            comp.extend(lut[c])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        nb = (c[0] + dx, c[1] + dy, c[2] + dz)
                        if nb in lut and nb not in seen:
                            seen.add(nb)
                            stack.append(nb)
        if len(comp) >= min_points:
            ci = np.array(comp)
            out.append((Q[ci], orig[ci]))
    # Biggest first: with a cluttered table the thing most worth reporting is
    # the thing most completely seen.
    out.sort(key=lambda t: len(t[0]), reverse=True)
    return [TableObject(c, plane, i,
                        pixel_index=(oi if pixel_uv is not None else None))
            for i, (c, oi) in enumerate(out)]


# ------------------------------------------------------------------ grasps
class Grasp:
    def __init__(self, centre, approach, jaw, width_m, obj, standoff_m=0.10):
        self.centre = np.asarray(centre, float)
        self.approach = np.asarray(approach, float)
        self.jaw = np.asarray(jaw, float)
        self.width_m = float(width_m)
        self.object_index = obj.index
        self.pregrasp = self.centre - self.approach * standoff_m
        self.quality = 0.0

    def describe(self):
        return ("grasp on object %d at (%.3f, %.3f, %.3f), %.0f mm wide, "
                "approach %s, quality %.2f"
                % (self.object_index, self.centre[0], self.centre[1],
                   self.centre[2], self.width_m * 1000,
                   np.round(self.approach, 2), self.quality))


def grasp_for(obj, plane, gripper_max_m=GRIPPER_MAX_M,
              margin_m=GRIPPER_MARGIN_M, standoff_m=0.10):
    """A top-down grasp that closes across the short horizontal axis.

    Every one of the three 2026-08-21 corrections is a line here:

      approach = -plane.normal      not "31 deg off vertical"
      jaw      = short horizontal   perpendicular to the approach BY
                                    CONSTRUCTION, so the fingers never close
                                    along the direction they travel
      centre   = plane + top/2      not the visible-surface centroid
    """
    usable = gripper_max_m - margin_m
    if obj.width_m > usable:
        raise SceneRefusal(
            "object %d is %.0f mm across its short axis and the jaws close "
            "on %.0f mm. The arm would discover this by colliding."
            % (obj.index, obj.width_m * 1000, usable * 1000))
    # The lowest a grasp centre can be and still keep the pad off the table.
    min_centre = PLANE_MARGIN_M + PAD_HALF_M
    if obj.top_m < min_centre:
        raise SceneRefusal(
            "object %d stands %.0f mm proud of the surface. The pad is "
            "%.0f mm tall, so the lowest grasp that clears the table is "
            "%.0f mm up -- above the object."
            % (obj.index, obj.top_m * 1000, 2 * PAD_HALF_M * 1000,
               min_centre * 1000))

    approach = -np.asarray(plane.normal, float)
    jaw = np.asarray(obj.short_axis, float)
    # ORTHOGONALISE, do not assume. The footprint axes are already in the
    # plane, but an accumulation of float error here becomes a gripper that
    # closes slightly into the table.
    jaw = jaw - approach * float(jaw @ approach)
    jaw /= max(np.linalg.norm(jaw), 1e-12)

    centre = obj.centre.copy()
    # THE PAD MUST NOT GO BELOW THE SURFACE. Raise the grasp until it clears,
    # but never above the top of the object -- that would be closing on air.
    hc = float(plane.height(centre[None, :])[0])
    if hc < min_centre:
        hc = min_centre
        centre = obj.centre + plane.normal * (min_centre
                                              - obj.centre_height_m)

    g = Grasp(centre, approach, jaw, obj.width_m, obj, standoff_m)
    # QUALITY, and it is a ranking not a probability. Narrow objects are
    # easier; tall ones give the fingers more to hold; a well-sampled cluster
    # is better understood than a sliver.
    slack = (usable - obj.width_m) / usable
    grip = min(1.0, obj.top_m / 0.06)
    seen = min(1.0, obj.n_points / 600.0)
    g.quality = float(0.45 * slack + 0.35 * grip + 0.20 * seen)
    return g


def analyse(points, up=(0.0, 0.0, 1.0), pixel_uv=None, **kw):
    """The whole answer: the plane, what is on it, and how to grasp each.

    Returns a dict. Objects that cannot be grasped are still REPORTED, with
    the reason -- "there is a thing here I cannot pick up" is an answer and
    silence is not.
    """
    kw["pixel_uv"] = pixel_uv
    plane = fit_support_plane(points, up=up,
                             **{k: v for k, v in kw.items()
                                if k in ("tol_m", "max_tilt_deg", "iters",
                                         "min_inliers", "seed")})
    objs = objects_on_plane(points, plane, pixel_uv=kw.get("pixel_uv"),
                            **{k: v for k, v in kw.items()
                               if k in ("min_height_m", "max_height_m",
                                        "cluster_m", "min_points")})
    rows = []
    for o in objs:
        try:
            g = grasp_for(o, plane)
            rows.append({"object": o, "grasp": g, "why": ""})
        except SceneRefusal as e:
            rows.append({"object": o, "grasp": None, "why": str(e)})
    rows.sort(key=lambda r: (-1.0 if r["grasp"] is None
                             else -r["grasp"].quality))
    return {"plane": plane, "objects": [r["object"] for r in rows],
            "rows": rows,
            "graspable": [r for r in rows if r["grasp"] is not None]}


def describe(scene):
    """What is on the table, in words, including what cannot be picked up."""
    p = scene["plane"]
    if not scene["rows"]:
        return ("a support surface %.0f mm below the sensor origin, tilt "
                "%.1f deg, and NOTHING standing on it."
                % (p.offset * 1000, p.tilt_deg))
    out = ["support surface at %.3f m, tilt %.1f deg, %d/%d points on it"
           % (p.offset, p.tilt_deg, len(p.inliers), p.n_total),
           "%d object(s) on it:" % len(scene["rows"])]
    for r in scene["rows"]:
        o = r["object"]
        line = "  " + o.describe()
        if r["grasp"] is None:
            line += "\n      CANNOT GRASP: " + r["why"]
        else:
            line += "\n      " + r["grasp"].describe()
        out.append(line)
    return "\n".join(out)


# ---------------------------------------------------------------- self-test
def _box_surface(centre, size, n=1400, seed=0, front_only=False,
                 view_dir=(0.0, -1.0, 0.0)):
    """Points on a box. CONSTRUCTED ground truth -- the only synthetic this
    repository trusts."""
    rng = np.random.default_rng(seed)
    c = np.asarray(centre, float)
    s = np.asarray(size, float) / 2.0
    pts = []
    faces = [(0, 1), (1, 1), (2, 1), (0, -1), (1, -1), (2, -1)]
    for ax, sign in faces:
        m = n // 6
        p = rng.uniform(-1, 1, size=(m, 3)) * s
        p[:, ax] = sign * s[ax]
        pts.append(p + c)
    P = np.vstack(pts)
    if front_only:
        v = np.asarray(view_dir, float)
        v /= np.linalg.norm(v)
        # Keep the faces whose outward normal opposes the view: what a single
        # depth camera would actually return.
        keep = ((P - c) @ v) < 0
        P = P[keep]
    return P


def _plane_points(z=0.0, n=4000, extent=0.6, seed=1, normal=None):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-extent, extent, size=(n, 2))
    P = np.column_stack([xy, np.full(n, z)])
    if normal is not None:
        nrm = np.asarray(normal, float)
        nrm /= np.linalg.norm(nrm)
        # tilt the plane about x by whatever `normal` asks for
        P[:, 2] = (z - (P[:, 0] * nrm[0] + P[:, 1] * nrm[1])) / nrm[2]
    return P


def self_test(verbose=True):
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-54s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    def refuses(name, fn, needle=""):
        try:
            fn()
            check(name, False, "it did not refuse")
        except SceneRefusal as e:
            check(name, (needle in str(e)) if needle else True,
                  str(e)[:70] if not needle or needle in str(e)
                  else "wrong reason: %s" % e)

    # ---- 1 a table with three boxes, all constructed
    table = _plane_points(z=0.90, n=6000)
    boxes = [((0.10, 0.05, 0.925), (0.05, 0.05, 0.05)),
             ((-0.12, 0.02, 0.935), (0.04, 0.09, 0.07)),
             ((0.00, -0.15, 0.915), (0.03, 0.03, 0.03))]
    pts = [table] + [_box_surface(c, s, seed=i + 2)
                     for i, (c, s) in enumerate(boxes)]
    P = np.vstack(pts)
    sc = analyse(P)
    check("the support plane is found to 1 mm",
          abs(sc["plane"].offset - 0.90) < 0.001,
          "offset %.4f m (true 0.900)" % sc["plane"].offset)
    check("the plane is horizontal", sc["plane"].tilt_deg < 1.0,
          "tilt %.2f deg" % sc["plane"].tilt_deg)
    check("all three objects are found", len(sc["objects"]) == 3,
          "%d found" % len(sc["objects"]))
    # centres, matched to truth by nearest
    worst = 0.0
    for c, s in boxes:
        d = min(np.linalg.norm(o.centre - np.asarray(c))
                for o in sc["objects"])
        worst = max(worst, d)
    check("every object centre within 6 mm of truth", worst < 0.006,
          "worst %.1f mm" % (worst * 1000))
    check("every object got a grasp", len(sc["graspable"]) == 3,
          "%d graspable" % len(sc["graspable"]))

    # ---- 2 THE SHELL CORRECTION. One view, front faces only.
    front = np.vstack([table,
                       _box_surface((0.0, 0.0, 0.93), (0.05, 0.05, 0.06),
                                    n=1800, seed=9, front_only=True)])
    s2 = analyse(front)
    check("a single-view object is still found", len(s2["objects"]) == 1,
          "%d found" % len(s2["objects"]))
    if s2["objects"]:
        o = s2["objects"][0]
        err_corr = abs(o.centre[2] - 0.93)
        err_raw = abs(o.visible_centroid[2] - 0.93)
        check("the plane fixes the shell bias in height",
              err_corr < 0.006 and err_corr < err_raw,
              "corrected %.1f mm vs raw centroid %.1f mm"
              % (err_corr * 1000, err_raw * 1000))
        check("the raw centroid really was wrong (or this proves nothing)",
              err_raw > 0.004, "raw error %.1f mm" % (err_raw * 1000))

    # ---- 3 the refusals
    # A LONG BOX IS NOT A WIDE ONE. 200 x 50 mm is graspable across its
    # 50 mm short axis, and an earlier version of this test assumed it was
    # not -- which would have "passed" by refusing something perfectly
    # pickable. The refusal must key on the SHORT axis.
    longish = np.vstack([table, _box_surface((0.0, 0.0, 0.95),
                                             (0.20, 0.05, 0.10), seed=4)])
    sl = analyse(longish)
    check("a long thin box is grasped across its SHORT axis",
          len(sl["graspable"]) == 1
          and abs(sl["graspable"][0]["grasp"].width_m - 0.05) < 0.006,
          "width %.0f mm"
          % (sl["graspable"][0]["grasp"].width_m * 1000
             if sl["graspable"] else -1))
    wide = np.vstack([table, _box_surface((0.0, 0.0, 0.95),
                                          (0.13, 0.13, 0.10), seed=4)])
    sw = analyse(wide)
    check("a genuinely too-wide object is reported and NOT grasped",
          len(sw["objects"]) == 1 and sw["rows"][0]["grasp"] is None
          and "jaws close" in sw["rows"][0]["why"],
          sw["rows"][0]["why"][:60] if sw["rows"] else "no object")
    flat = np.vstack([table, _box_surface((0.0, 0.0, 0.9015),
                                          (0.04, 0.04, 0.003), seed=5)])
    sf = analyse(flat)
    check("a flat object is reported and NOT grasped",
          not sf["graspable"],
          (sf["rows"][0]["why"][:60] if sf["rows"] else "nothing found"))

    refuses("a wall is not a table",
            lambda: fit_support_plane(
                _plane_points(z=0.0, n=3000, normal=(1.0, 0.0, 0.02))),
            "no plane within")
    refuses("a handful of points is refused",
            lambda: fit_support_plane(np.zeros((10, 3))), "at least")

    # ---- 4 an empty table says so
    se = analyse(table)
    check("an empty table is reported as empty", not se["objects"]
          and "NOTHING standing on it" in describe(se))

    # ---- 5 a TILTED table is still found, and the approach follows it
    tilt_n = np.array([0.0, math.sin(math.radians(12)),
                       math.cos(math.radians(12))])
    tp = _plane_points(z=0.90, n=6000, normal=tilt_n)
    tb = tp[np.argsort(np.linalg.norm(tp[:, :2], axis=1))[:1]][0]
    tilted = np.vstack([tp, _box_surface(tb + tilt_n * 0.03,
                                         (0.05, 0.05, 0.06), seed=7)])
    st = analyse(tilted, max_tilt_deg=30.0)
    check("a 12 deg tilted surface is found",
          abs(st["plane"].tilt_deg - 12.0) < 1.5,
          "tilt %.1f deg" % st["plane"].tilt_deg)
    if st["graspable"]:
        g = st["graspable"][0]["grasp"]
        check("the approach follows the surface normal, not world down",
              float(g.approach @ (-st["plane"].normal)) > 0.99,
              "approach . -normal = %.4f"
              % float(g.approach @ (-st["plane"].normal)))
        check("the jaw is perpendicular to the approach",
              abs(float(g.jaw @ g.approach)) < 1e-9,
              "dot %.2e" % abs(float(g.jaw @ g.approach)))

    # ---- 6 NO GRASP MAY PUT THE FINGERS BELOW THE SURFACE.
    worst_below = 1.0
    for c, s in boxes:
        pp = np.vstack([table, _box_surface(c, s, seed=11)])
        ss = analyse(pp)
        for r in ss["graspable"]:
            g = r["grasp"]
            tip = float(ss["plane"].height(g.centre[None, :])[0]) \
                - PAD_HALF_M
            worst_below = min(worst_below, tip)
    check("the pad never goes below the surface",
          worst_below >= PLANE_MARGIN_M - 1e-9,
          "closest tip %.1f mm above the plane" % (worst_below * 1000))

    if verbose:
        print("table_scene self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
