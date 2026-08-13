#!/usr/bin/env python3
"""THE WORK SURFACE HEIGHT, MEASURED FROM A DEPTH FRAME. Arithmetic only.

`srl_experiments.work_surface` has had `set_measured()` since it was written
and NOTHING EVER CALLED IT. The height was declared, the declared value was
correct in sim, and on real hardware a table 20 mm out would have been
absorbed silently into every grasp -- which is the failure the module was
created to stop and which it could not stop on its own, because it had no
producer.

This is the producer's arithmetic, deliberately split from the node so it can
be given a CONSTRUCTED depth frame whose true answer is known to the
millimetre. That is the only kind of synthetic input CLAUDE.md permits: the
geometry here is constructed, never rendered, so the known-answer test means
something.

THE ESTIMATOR IS A MODE, NOT A MEAN, and that is the whole design.

A mean over the region is pulled up by everything standing on the surface --
cubes, planes, the multimeter -- and pulled down by any hole. Worse, it moves
as the objects move, so the "measurement" would change between trials with a
motionless table. The mode of a 2 mm histogram answers the question actually
being asked: what height do MOST of these points share. Objects occupy a
minority of the region by construction, so they land in other bins and are
ignored rather than averaged in.

    measure_surface(depth, intrinsics, T_world_cam, region) ->
        SurfaceEstimate | None

IT RETURNS None RATHER THAN A NUMBER IT CANNOT DEFEND. Too few points, no
dominant bin, or a spread that says the camera is not looking at a plane all
produce a refusal with a reason attached. A quiet fall back to the declared
height is exactly the silence this replaces.
"""

import math


# 2 mm bins. The tolerance that matters downstream is work_surface.TOL_M =
# 10 mm, so the bin has to be several times finer than the thing it decides.
BIN_M = 0.002
# Below this, the region is not being seen well enough to say anything.
MIN_POINTS = 200
# The dominant bin has to actually dominate. A flat surface fills one bin with
# a large share of its points; a camera pointed at a wall of clutter does not.
MIN_SHARE = 0.20
# Points further than this from the dominant bin are objects, not the surface,
# and are excluded from the refinement.
INLIER_BAND_M = 0.006


class SurfaceEstimate:
    """A height with everything needed to decide whether to believe it."""

    def __init__(self, z_m, n_points, n_inliers, share, spread_m, region):
        self.z_m = z_m
        self.n_points = n_points
        self.n_inliers = n_inliers
        self.share = share
        self.spread_m = spread_m
        self.region = region

    def note(self):
        """Provenance, in the form work_surface.set_measured() asks for."""
        return ("depth mode over %d points in x %.2f..%.2f y %.2f..%.2f, "
                "%d inliers (%.0f%% in the dominant %.0f mm bin), "
                "spread %.1f mm"
                % (self.n_points, self.region[0][0], self.region[0][1],
                   self.region[1][0], self.region[1][1], self.n_inliers,
                   100.0 * self.share, BIN_M * 1000.0,
                   self.spread_m * 1000.0))

    def as_dict(self):
        return dict(z_m=round(self.z_m, 4), n_points=self.n_points,
                    n_inliers=self.n_inliers, share=round(self.share, 4),
                    spread_mm=round(self.spread_m * 1000.0, 2))


class SurfaceRefusal(Exception):
    """Not enough evidence for a height. Carries why, in words."""


def depth_to_metres(value, encoding):
    """16UC1 is MILLIMETRES; 32FC1 is metres.

    Read from the ENCODING and not from the magnitude. The magnitude
    heuristic elsewhere in this package ("z > 100 looks like millimetres")
    is right for a table at 1.2 m and wrong for anything closer than 100 mm,
    and a rule that works on the common case and fails silently on the close
    one is the shape of fault this repository keeps paying for.
    """
    if encoding in ("16UC1", "mono16"):
        return value / 1000.0
    if encoding in ("32FC1",):
        return float(value)
    raise SurfaceRefusal("depth encoding %r is neither 16UC1 (mm) nor 32FC1 "
                         "(m); refusing to guess the units" % encoding)


def _apply(T, p):
    """T is a 4x4 row-major world<-camera transform."""
    return [T[r][0] * p[0] + T[r][1] * p[1] + T[r][2] * p[2] + T[r][3]
            for r in range(3)]


def deproject(u, v, z_m, intrinsics):
    """Pixel + depth -> a point in the OPTICAL frame (z forward, x right,
    y down). The convention is the camera's, not the world's."""
    fx, fy, ppx, ppy = intrinsics
    return [(u - ppx) * z_m / fx, (v - ppy) * z_m / fy, z_m]


def measure_surface(depth, intrinsics, T_world_cam, region,
                    encoding="16UC1", step=4, z_min_m=0.10, z_max_m=3.0):
    """Estimate the work-surface height from one depth frame.

    `depth`     2-D indexable, depth[v][u]
    `region`    ((x0, x1), (y0, y1)) in WORLD metres -- the footprint to
                measure over. Points outside it are ignored, so the table's
                legs, the floor and the wearer cannot vote.
    `step`      pixel stride. 4 is ~19k samples on a 640x480 frame, which is
                two orders of magnitude more than MIN_POINTS and costs about
                a millisecond.

    Raises SurfaceRefusal rather than returning a value it cannot defend.
    """
    h = len(depth)
    w = len(depth[0]) if h else 0
    if not h or not w:
        raise SurfaceRefusal("empty depth image")
    (x0, x1), (y0, y1) = region
    zs = []
    for v in range(0, h, step):
        row = depth[v]
        for u in range(0, w, step):
            raw = row[u]
            if not raw:
                continue                    # 0 is "no return", not a surface
            z = depth_to_metres(raw, encoding)
            if not (z_min_m <= z <= z_max_m):
                continue
            p = _apply(T_world_cam, deproject(u, v, z, intrinsics))
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                zs.append(p[2])
    if len(zs) < MIN_POINTS:
        raise SurfaceRefusal(
            "only %d depth points fell inside x %.2f..%.2f y %.2f..%.2f "
            "(need %d). The camera is not looking at the work region -- "
            "drive to the scan pose first, recordings/baselines/"
            "scan_pose.json." % (len(zs), x0, x1, y0, y1, MIN_POINTS))

    # THE MODE, over 2 mm bins. See the module docstring for why not a mean.
    bins = {}
    for z in zs:
        bins.setdefault(int(math.floor(z / BIN_M)), []).append(z)
    key = max(bins, key=lambda k: len(bins[k]))
    share = len(bins[key]) / float(len(zs))
    if share < MIN_SHARE:
        raise SurfaceRefusal(
            "no dominant height: the fullest %.0f mm bin holds only %.0f%% of "
            "%d points. That is not a plane, so nothing here is a surface "
            "height." % (BIN_M * 1000.0, 100.0 * share, len(zs)))

    # REFINE ACROSS THE BIN EDGE. A surface landing on a boundary splits its
    # points between two neighbouring bins, so the mode alone can sit up to a
    # full bin off. Averaging the inliers within a band around it recovers the
    # sub-bin position, and the band is narrow enough that a 40 mm cube cannot
    # reach into it.
    centre = (key + 0.5) * BIN_M
    inliers = [z for z in zs if abs(z - centre) <= INLIER_BAND_M]
    z_hat = sum(inliers) / len(inliers)
    spread = math.sqrt(sum((z - z_hat) ** 2 for z in inliers) / len(inliers))
    return SurfaceEstimate(z_hat, len(zs), len(inliers), share, spread,
                           region)
