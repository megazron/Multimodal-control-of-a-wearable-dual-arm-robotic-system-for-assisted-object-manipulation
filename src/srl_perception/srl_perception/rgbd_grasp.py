#!/usr/bin/env python3
"""Object -> grasp, from an RGB-D frame. Pure geometry, no ROS, no model.

WHY THIS IS A SEPARATE, IMPORTABLE MODULE. Everything here is arithmetic on
arrays: a pixel and a depth become a point, points become a cloud, a cloud
becomes a grasp. None of it needs a robot, a camera or a network, so all of
it can be tested against constructed ground truth -- which is the only kind
of synthetic data this project trusts (CLAUDE.md: synthetic only where the
ground truth is CONSTRUCTED, never RENDERED).

THE GRASP IS GEOMETRIC AND DELIBERATELY NOT LEARNED. A parallel jaw on a
rigid object is a solved problem: find the object's principal axes, close
across the SHORT one, approach along the free one. GPG (atenpas/gpg) does
exactly this with no machine learning and it is the right tool here --
a learned grasp model would need a GPU budget we do not have (4 GB, shared
with the detector) and could not explain a refusal.

WHAT REFUSES, AND WHY EACH REFUSAL EXISTS
  no depth        a colour blob with no depth return is a direction, not an
                  object. Guessing a range here is how you drive a gripper
                  into a wall.
  too wide        the Robotiq 85 opens 85 mm. An object wider than that
                  across its short axis cannot be grasped, and reporting a
                  grasp for it would be a lie the arm discovers by colliding.
  too few points  a handful of depth pixels gives a principal axis dominated
                  by noise; the axis would be confident and meaningless.
"""
from __future__ import annotations

import math

import numpy as np

# Robotiq 85: 85 mm between the pads, fully open. The usable figure is
# smaller because the fingers need somewhere to go, so a margin is kept.
GRIPPER_MAX_M = 0.085
GRIPPER_MARGIN_M = 0.010
MIN_POINTS = 30


class GraspRefusal(Exception):
    """Raised with a reason a person can act on."""


def deproject(u, v, z, K):
    """Pixel + depth -> 3D point in the CAMERA frame.

    K is (fx, fy, cx, cy) and must be the FACTORY intrinsics, not nominal
    ones. Measured on this rig: the Kinova colour sensor's principal point is
    cy = 238.3 on a 720-high image, not the 360 that "centre of the image"
    would suggest. Using the nominal value throws every ray by 122 px, which
    at 1 m is 94 mm -- three times the grasp tolerance.
    """
    fx, fy, cx, cy = K
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)


def cloud_from_mask(depth_m, mask, K, z_min=0.05, z_max=3.0):
    """Every masked pixel with a valid depth, as an (N, 3) camera-frame cloud."""
    vs, us = np.nonzero(mask)
    if vs.size == 0:
        return np.zeros((0, 3))
    z = depth_m[vs, us]
    ok = (z > z_min) & (z < z_max) & np.isfinite(z)
    if not np.any(ok):
        return np.zeros((0, 3))
    vs, us, z = vs[ok], us[ok], z[ok]
    fx, fy, cx, cy = K
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)


def principal_axes(pts):
    """(centre, axes, extents) with axes ordered LONGEST first.

    Ordering matters and is not cosmetic: the jaw must close across the
    SHORTEST axis, so the caller needs to know which is which rather than
    trusting whatever order the eigensolver happened to return.
    """
    c = pts.mean(axis=0)
    X = pts - c
    cov = X.T @ X / max(len(X) - 1, 1)
    w, V = np.linalg.eigh(cov)
    order = np.argsort(w)[::-1]
    V = V[:, order]
    proj = X @ V
    extents = proj.max(axis=0) - proj.min(axis=0)
    return c, V, extents


def min_width_frame(pts, long_ax, n_ang=180):
    """Rotate about the long axis to find the TRUE minimum width.

    PCA ALONE IS WRONG FOR A CUBE, AND A CUBE IS THE TARGET. When two or
    three extents are equal the covariance is degenerate and the eigensolver
    returns an arbitrary orientation within that eigenspace -- so the "short
    axis" can come back pointing along a face diagonal. Measured on a
    constructed 40 mm cube: PCA reported 43.0 mm, because the jaw was closing
    across the diagonal rather than the face.

    Three millimetres is survivable; the DIRECTION is not. A jaw told to
    close across a diagonal approaches the corner, and the object rolls out.

    So the two axes perpendicular to the long one are rotated through 180
    steps and the pair giving the smallest projected extent wins. That is the
    rotating-calipers minimum-width answer, and for a cube it lands on a face
    rather than a diagonal.
    """
    a = long_ax / np.linalg.norm(long_ax)
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(tmp, a))) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(a, tmp); e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    X = pts - pts.mean(axis=0)
    p1, p2 = X @ e1, X @ e2
    best = None
    for k in range(n_ang):
        th = math.pi * k / n_ang
        c, s = math.cos(th), math.sin(th)
        u = c * p1 + s * p2                  # candidate jaw direction
        w = float(u.max() - u.min())
        if best is None or w < best[0]:
            v = -s * p1 + c * p2
            best = (w, c * e1 + s * e2, -s * e1 + c * e2,
                    float(v.max() - v.min()))
    width, close_ax, mid_ax, mid_ext = best
    return close_ax, mid_ax, width, mid_ext


def grasp_from_cloud(pts, approach_hint=None, view_axis=None):
    """A parallel-jaw grasp for this cloud, in the cloud's own frame.

    Returns dict(centre, close_axis, approach, width_m, length_m).
    `close_axis` is the direction the jaws travel; `approach` is how the hand
    comes in. They are perpendicular by construction, because a jaw that
    closes along its own approach direction is not a grasp.
    """
    if len(pts) < MIN_POINTS:
        raise GraspRefusal(
            "only %d depth points on the object (need %d) -- the principal "
            "axis would be noise wearing a confident number"
            % (len(pts), MIN_POINTS))
    c, V, ext = principal_axes(pts)
    if view_axis is not None:
        # A SINGLE DEPTH VIEW SEES A SHELL, NOT A SOLID.
        #
        # The camera returns only the front surface, so the cloud has almost
        # no thickness ALONG THE LINE OF SIGHT. An unconstrained minimum-width
        # search therefore picks the view direction every time and reports a
        # width of ~0 -- measured on a constructed 50 mm cube: 0.0 mm. Acting
        # on that would command the jaw to close along the line of sight,
        # squeezing through a surface whose far side was never observed.
        #
        # With one view the only honest grasp closes in the plane
        # PERPENDICULAR to the view axis, where both jaw contact points lie on
        # surface that was actually measured. The hand then approaches along
        # the line of sight, which is also the direction it can reach without
        # passing through the object.
        a = np.asarray(view_axis, float)
        a = a / np.linalg.norm(a)
        long_ax = a
        short_ax, mid_ax, width, mid_ext = min_width_frame(pts, long_ax)
    else:
        long_ax = V[:, 0]
        short_ax, mid_ax, width, mid_ext = min_width_frame(pts, long_ax)
    usable = GRIPPER_MAX_M - GRIPPER_MARGIN_M
    if width > usable:
        # the mid axis may still fit -- try it before refusing
        if mid_ext <= usable:
            short_ax, mid_ax = mid_ax, short_ax
            width = mid_ext
        else:
            raise GraspRefusal(
                "object is %.0f mm across its narrowest axis; the Robotiq 85 "
                "opens %.0f mm usable. It cannot be grasped as it lies."
                % (width * 1000, usable * 1000))
    approach = mid_ax
    if view_axis is not None and approach_hint is None:
        approach_hint = np.asarray(view_axis, float)
    if approach_hint is not None:
        h = np.asarray(approach_hint, float)
        h = h - float(np.dot(h, short_ax)) * short_ax      # keep perpendicular
        if np.linalg.norm(h) > 1e-6:
            approach = h / np.linalg.norm(h)
        if float(np.dot(approach, approach_hint)) < 0:
            approach = -approach
    # re-orthogonalise so the returned frame is a frame
    approach = approach - float(np.dot(approach, short_ax)) * short_ax
    n = np.linalg.norm(approach)
    if n < 1e-9:
        raise GraspRefusal("approach direction collapsed onto the jaw axis")
    approach = approach / n
    return dict(centre=c, close_axis=short_ax, approach=approach,
                width_m=width, length_m=float(ext[0]),
                axes=V, extents=ext)


def to_robot_frame(g, cam_p, cam_R):
    """Move a camera-frame grasp into the robot frame."""
    return dict(
        centre=np.asarray(cam_p, float) + cam_R @ g["centre"],
        close_axis=cam_R @ g["close_axis"],
        approach=cam_R @ g["approach"],
        width_m=g["width_m"], length_m=g["length_m"])


def pregrasp(g, standoff_m=0.12):
    """Where the hand waits before closing: back off ALONG the approach."""
    return g["centre"] - np.asarray(g["approach"], float) * float(standoff_m)
