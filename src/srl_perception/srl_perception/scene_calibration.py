#!/usr/bin/env python3
"""Calibrate the scene camera USING THE ARM AS THE REFERENCE OBJECT.

The scene camera has never been calibrated -- `scene_camera_node` publishes a
ZERO K until it is -- so a pixel in that image is a direction with no scale
and an object in it has no position.

There is no checkerboard in the lab, but there is a robot that knows exactly
where its own gripper is. Drive the gripper to a set of known points, find it
in the image, and solve for the camera. The arm IS the calibration target.

WHY A MARKER IS REQUIRED, MEASURED RATHER THAN ASSUMED.
An earlier attempt located the gripper by rolling joint_7 and taking the
centroid of the CHANGED pixels. It produced 24 px mean reprojection error --
about +-75 mm at 2 m, three times the grasp tolerance -- and the cause is
structural, not statistical: the changed region is the gripper's whole body,
and its centroid moves with the roll angle rather than sitting on the
forward-kinematics hand point. More samples cannot fix a systematic offset.

A single patch of saturated colour on the gripper, in a hue the room does not
already contain, is detected to a pixel or two and sits at a FIXED offset from
the hand. That is what turns this from a plausible idea into a calibration.

`marker_offset_m` is that fixed offset, in the end-effector frame. Leave it
zero and the solve still works; the camera pose is then relative to the marker
rather than the hand, and every position it reports carries the same bias.
"""
from __future__ import annotations

import math

import numpy as np


class CalibrationRefusal(Exception):
    """Raised with the number that failed and what would fix it."""


def find_marker(bgr, hsv_lo, hsv_hi, min_area=60):
    """The marker's pixel, or None. Largest blob in the band."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < min_area:
            continue
        if best is None or a > best[0]:
            best = (a, float(ce[i][0]), float(ce[i][1]))
    if best is None:
        return None
    return dict(uv=(best[1], best[2]), area=best[0])


def _rodrigues(r):
    th = float(np.linalg.norm(r))
    if th < 1e-9:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def solve_camera(points_xyz, pixels_uv, image_wh, f_init=(600., 900., 1300.)):
    """Camera pose + focal from 3D<->2D correspondences.

    Unknowns: 3 rotation + 3 translation + 1 focal = 7. Each correspondence
    gives 2 equations, so 4 points is the bare minimum and more is better.

    THE REPROJECTION ERROR IS RETURNED AND IT IS THE POINT. A solve always
    returns numbers; only the residual says whether to believe them. Under
    about 2 px is a calibration. Twenty is a direction-finder.
    """
    from scipy.optimize import least_squares
    P = np.asarray(points_xyz, float)
    uv = np.asarray(pixels_uv, float)
    if len(P) < 4:
        raise CalibrationRefusal(
            "only %d correspondences; 7 unknowns need at least 4 points and "
            "really want 8 or more spread through the volume" % len(P))
    W, H = image_wh

    def resid(x):
        R, t, f = _rodrigues(x[0:3]), x[3:6], x[6]
        out = []
        for p, (u, v) in zip(P, uv):
            c = R @ p + t
            if c[2] < 1e-3:
                out += [1e3, 1e3]
                continue
            out += [f * c[0] / c[2] + W / 2 - u, f * c[1] / c[2] + H / 2 - v]
        return np.array(out)

    best = None
    for f0 in f_init:
        for yaw in np.linspace(-math.pi, math.pi, 8):
            x0 = np.array([0.0, yaw, 0.0, 0.0, 0.0, 2.5, f0])
            try:
                r = least_squares(resid, x0, method="lm", max_nfev=8000)
            except Exception:                                 # noqa: BLE001
                continue
            if best is None or r.cost < best.cost:
                best = r
    if best is None:
        raise CalibrationRefusal("no solve converged from any starting guess")
    x = best.x
    R, t, f = _rodrigues(x[0:3]), x[3:6], float(x[6])
    e = np.linalg.norm(resid(x).reshape(-1, 2), axis=1)
    return dict(R=R, t=t, f=f, cx=W / 2.0, cy=H / 2.0,
                camera_position=-R.T @ t,
                reproj_mean_px=float(e.mean()), reproj_max_px=float(e.max()),
                n_points=len(P), per_point_px=[float(v) for v in e])


def check(sol, max_mean_px=2.5):
    """Believe it, or say why not."""
    if sol["reproj_mean_px"] > max_mean_px:
        raise CalibrationRefusal(
            "mean reprojection error %.1f px (limit %.1f). At 2 m that is "
            "about %.0f mm of position error, against a 30 mm grasp gate. "
            "The usual cause is that the located pixel is not a fixed point "
            "on the gripper -- use a marker, not a motion blob."
            % (sol["reproj_mean_px"], max_mean_px,
               1000 * 2.0 * sol["reproj_mean_px"] / max(sol["f"], 1.0)))
    return True


def pixel_to_ray(sol, u, v):
    """A scene pixel -> (origin, direction) in the ROBOT frame."""
    d_cam = np.array([(u - sol["cx"]) / sol["f"],
                      (v - sol["cy"]) / sol["f"], 1.0])
    d_cam /= np.linalg.norm(d_cam)
    R = np.asarray(sol["R"], float)
    return np.asarray(sol["camera_position"], float), R.T @ d_cam


def ray_plane(origin, direction, plane_point, plane_normal):
    """Where a ray meets a plane. None if it runs parallel to it.

    This is how a scene pixel becomes a POSITION without depth: the object is
    known to be resting on a surface, and the surface is a plane.
    """
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    n = np.asarray(plane_normal, float)
    den = float(np.dot(d, n))
    if abs(den) < 1e-9:
        return None
    t = float(np.dot(np.asarray(plane_point, float) - o, n)) / den
    if t <= 0:
        return None
    return o + t * d
