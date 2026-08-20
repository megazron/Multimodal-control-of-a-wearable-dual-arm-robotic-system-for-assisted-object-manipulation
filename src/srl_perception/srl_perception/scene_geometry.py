#!/usr/bin/env python3
"""Camera <-> world geometry for the scene camera. Pure arithmetic.

Two jobs, both of which have to be right before any body position means
anything, and both of which are testable without a camera:

  1. LIFT.    A detector gives a metric skeleton in its own frame plus the
              image points it came from. Solving those together against the
              intrinsic puts the skeleton in the CAMERA frame, and the
              extrinsic puts it in the ROBOT WORLD frame.
  2. PROJECT. The robot's own links, whose world positions are known exactly
              from the joint encoders, are projected INTO the image. That is
              how the scene camera "sees" the arms: not by recognising them,
              but by drawing where they must be. It is also the only honest
              check on the extrinsic -- if the drawn arm does not sit on the
              real arm in the picture, the transform is wrong, and no amount
              of confident body tracking will tell you that.

WHY THE ARMS ARE NOT DETECTED VISUALLY. The arm's own sensing is the best
estimate of the arm's position in the system by a wide margin: joint encoders
plus a URDF, against a camera two metres away looking at a partly occluded
gripper. Fusing a visual arm estimate into the arm state would make the arm
state worse. So the camera is used to CONFIRM the frame, never to correct the
arm -- doc 16 section 3, "the arm's own sensing always wins about the arm".
"""
import math

import numpy as np


def rpy_to_R(r, p, y):
    """URDF convention, R = Rz(y) Ry(p) Rx(r). Same as the calibration tool."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


class SceneFrames:
    """Holds the intrinsic and the extrinsic, and refuses without them."""

    def __init__(self, K=None, D=None, T_world_camera=None,
                 intrinsics_src="", extrinsics_src=""):
        self.K = None if K is None else np.asarray(K, float).reshape(3, 3)
        self.D = np.zeros(5) if D is None else np.asarray(D, float).ravel()
        self.T_wc = (None if T_world_camera is None
                     else np.asarray(T_world_camera, float).reshape(4, 4))
        self.intrinsics_src = intrinsics_src
        self.extrinsics_src = extrinsics_src

    @property
    def calibrated(self):
        return self.K is not None and self.T_wc is not None

    def why_not(self):
        missing = []
        if self.K is None:
            missing.append("the camera has no intrinsic calibration")
        if self.T_wc is None:
            missing.append("the camera's position relative to the robot is "
                           "not known")
        return "; ".join(missing)

    # ------------------------------------------------------------- lift
    def camera_to_world(self, pts_cam):
        """(N,3) camera-frame points -> world."""
        p = np.asarray(pts_cam, float).reshape(-1, 3)
        return (self.T_wc[:3, :3] @ p.T).T + self.T_wc[:3, 3]

    def world_to_camera(self, pts_world):
        p = np.asarray(pts_world, float).reshape(-1, 3)
        Rt = self.T_wc[:3, :3].T
        return (Rt @ (p - self.T_wc[:3, 3]).T).T

    def project(self, pts_world):
        """(N,3) world points -> (N,2) pixels and (N,) depth.

        Depth is returned because a point BEHIND the camera still produces a
        finite pixel coordinate, and drawing it would put the robot's arm on
        the wrong side of the picture with nothing to say so. Callers must
        drop anything with depth <= 0.
        """
        cam = self.world_to_camera(pts_world)
        z = cam[:, 2]
        safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
        x = cam[:, 0] / safe
        y = cam[:, 1] / safe
        # Distortion, plumb_bob. Applied because a wide laptop lens has
        # enough of it to move a projected elbow by several pixels, and this
        # projection is used to CHECK the extrinsic.
        k1, k2, p1, p2, k3 = (list(self.D) + [0] * 5)[:5]
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        u = self.K[0, 0] * xd + self.K[0, 2]
        v = self.K[1, 1] * yd + self.K[1, 2]
        return np.stack([u, v], axis=1), z


def lift_skeleton(world_landmarks, image_points, K, D, min_points=6):
    """Put a detector's metric skeleton into the CAMERA frame.

    `world_landmarks` (N,3): metric, in the detector's own frame, origin
    wherever the detector puts it (MediaPipe uses the hip midpoint).
    `image_points` (N,2): the SAME landmarks, in pixels.

    Returns (points_in_camera (N,3), residual_px) or (None, reason).

    This is what supplies the absolute distance that a single camera cannot
    otherwise recover. The detector knows the SHAPE in metres; the image knows
    the DIRECTION; PnP is the only thing that has to be solved, and its
    residual is returned so a bad solve is visible instead of silent.
    """
    import cv2
    obj = np.asarray(world_landmarks, np.float64).reshape(-1, 3)
    img = np.asarray(image_points, np.float64).reshape(-1, 2)
    if len(obj) < min_points or len(obj) != len(img):
        return None, ("only %d usable landmarks -- need %d to place the body"
                      % (len(obj), min_points))
    ok, rvec, tvec = cv2.solvePnP(obj, img.reshape(-1, 1, 2),
                                  np.asarray(K, float).reshape(3, 3),
                                  np.asarray(D, float).ravel(),
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return None, "the body's distance from the camera could not be solved"
    proj, _ = cv2.projectPoints(obj, rvec, tvec,
                                np.asarray(K, float).reshape(3, 3),
                                np.asarray(D, float).ravel())
    resid = float(np.sqrt(((proj.reshape(-1, 2) - img) ** 2)
                          .sum(axis=1)).mean())
    R, _ = cv2.Rodrigues(rvec)
    return (R @ obj.T).T + tvec.ravel(), resid


# --------------------------------------------------------------- the arms
# The link chain drawn for each arm. Same list the operations GUI's ACTUAL
# panel uses, so the two pictures are of the same robot.
ARM_CHAIN = ("base_link", "shoulder_link", "half_arm_1_link",
             "half_arm_2_link", "forearm_link", "spherical_wrist_1_link",
             "spherical_wrist_2_link", "bracelet_link", "end_effector_link")


def arm_polyline(frames, points_world, image_size):
    """Project one arm's link origins, dropping anything not in front.

    Returns [(u, v), ...] for the visible prefix. A link behind the camera or
    outside the frame ends the polyline rather than being clamped, because a
    clamped point draws a straight line to the edge of the picture and reads
    as an arm pointing somewhere it is not.
    """
    if not frames.calibrated or not len(points_world):
        return []
    uv, z = frames.project(points_world)
    w, h = image_size
    out = []
    for i in range(len(uv)):
        if z[i] <= 0.05:
            break
        u, v = float(uv[i][0]), float(uv[i][1])
        if not (-2 * w < u < 3 * w and -2 * h < v < 3 * h):
            break
        out.append((u, v))
    return out
