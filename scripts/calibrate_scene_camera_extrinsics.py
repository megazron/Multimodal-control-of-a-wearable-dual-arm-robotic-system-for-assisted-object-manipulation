#!/usr/bin/env python3
"""Where the scene camera IS, in the robot's world frame.

    python3 scripts/calibrate_scene_camera_extrinsics.py --self-test
    python3 scripts/calibrate_scene_camera_extrinsics.py --live --marker-on-mount
    python3 scripts/calibrate_scene_camera_extrinsics.py --image shot.png \
        --marker-at 0.0 0.55 1.02 --marker-rpy 0 0 0

WHY THIS IS NOT OPTIONAL, restated because it is easy to skip. The tracker
recovers the wearer in the CAMERA's frame. Everything that consumes it -- the
clearance floor, the planning scene, the GUI overlay -- works in the ROBOT's
world frame. Without this transform the body is in the wrong frame, and a body
in the wrong frame is the single most dangerous output this subsystem can
produce: it is confident, it is smooth, it has high per-segment confidence,
and it is 400 mm from where the person is.

THE TARGET. A printed ArUco marker (DICT_APRILTAG_36h11, the same family
`srl_perception/apriltag_detector.py` already uses) whose pose in the world is
KNOWN, by one of two routes:

  --marker-on-mount   the marker is stuck to the backpack frame. Its world
                      pose comes from TF, because world -> torso -> backpack
                      -> mount is a FIXED chain in the URDF. Nothing is
                      measured by hand, so nothing is measured wrong. This is
                      the recommended route.
  --marker-at X Y Z   the marker lies on the table at a place you measured.
                      Honest, and only as good as the tape.

WHAT IT WRITES. config/scene_camera_extrinsics.yaml, holding T_world_camera,
the reprojection residual, and which route produced it. A residual is written
because an extrinsic with no residual beside it cannot be told from a guess.

THE INSTRUMENT IS CHECKED FIRST. `--self-test` places a marker at a known
pose, projects its corners through a known camera at a known extrinsic, and
requires this tool to recover the extrinsic. Constructed, not rendered: the
ground truth is projection arithmetic.
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, "config", "scene_camera_extrinsics.yaml")
INTRINSICS = os.path.join(WS, "config", "scene_camera_intrinsics.yaml")

ARUCO_DICT = cv2.aruco.DICT_APRILTAG_36h11
MARKER_ID = 0
MARKER_M = 0.15          # printed side length, metres. MEASURE THE PRINT.


def rpy_to_R(r, p, y):
    """URDF convention: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) < 1e-8:
        return (math.atan2(-R[1, 2], R[1, 1]), p, 0.0)
    return (math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0]))


def marker_corners_world(centre, rpy, side=MARKER_M):
    """The four corners, in the order cv2.aruco returns them.

    aruco orders corners clockwise from the top-left as seen in the marker's
    own frame with +x right and +y DOWN. Getting this order wrong yields a
    solve that converges to a pose rotated by a multiple of 90 degrees -- it
    does not fail, it answers confidently and wrongly, which is why the
    self-test checks the recovered ROTATION and not only the position.
    """
    h = side * 0.5
    local = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    R = rpy_to_R(*rpy)
    return (R @ local.T).T + np.asarray(centre, dtype=float)


def load_K(path=INTRINSICS):
    if not os.path.exists(path):
        raise SystemExit(
            "REFUSING: no intrinsics at %s. Solve the extrinsic against a "
            "guessed focal length and every body position is scaled by the "
            "error, silently. Run scripts/calibrate_scene_camera.py first."
            % path)
    import yaml
    d = yaml.safe_load(open(path))
    K = np.array(d["camera_matrix"], float).reshape(3, 3)
    D = np.array(d.get("distortion", [0] * 5), float).ravel()
    return K, D


def marker_local_corners(side=MARKER_M):
    """The four corners in the MARKER's own frame, aruco order."""
    h = side * 0.5
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def solve(marker_centre, marker_rpy, corners_img, K, D, side=MARKER_M):
    """(T_world_camera 4x4, residual px). Never returns without a residual.

    SOLVED IN THE MARKER'S OWN FRAME AND THEN COMPOSED, which is not a detail.
    `SOLVEPNP_IPPE_SQUARE` is the right solver for a square fiducial and it
    ASSUMES its four object points are the marker's own corners about the
    origin. Handing it world-frame corners -- the obvious thing, and what this
    did first -- makes it return the camera pose relative to the MARKER while
    the caller reads it as a pose in the world. The self-test caught it as a
    translation error of exactly 1.0500 m against a marker at z = 1.05, which
    is the tell: the answer was right, in the wrong frame.
    """
    obj = marker_local_corners(side)
    ok, rvec, tvec = cv2.solvePnP(
        obj, np.asarray(corners_img, np.float64).reshape(-1, 1, 2),
        K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise SystemExit("solvePnP failed -- the marker was seen but its "
                         "pose could not be resolved")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    resid = float(np.sqrt(((proj.reshape(-1, 2) -
                            np.asarray(corners_img, float).reshape(-1, 2))
                           ** 2).sum(axis=1)).mean())
    R_cm, _ = cv2.Rodrigues(rvec)              # marker -> camera
    T_cm = np.eye(4)
    T_cm[:3, :3] = R_cm
    T_cm[:3, 3] = tvec.ravel()
    T_wm = np.eye(4)                            # marker -> world, KNOWN
    T_wm[:3, :3] = rpy_to_R(*marker_rpy)
    T_wm[:3, 3] = np.asarray(marker_centre, float)
    return T_wm @ np.linalg.inv(T_cm), resid    # camera pose IN the world


def detect(img, K, D):
    """(corners, id) of the calibration marker, or (None, reason)."""
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    try:
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(img)
    except AttributeError:                      # OpenCV < 4.7
        corners, ids, _ = cv2.aruco.detectMarkers(
            img, d, parameters=cv2.aruco.DetectorParameters_create())
    if ids is None or len(ids) == 0:
        return None, "no marker of the calibration family is in view"
    ids = ids.ravel().tolist()
    if MARKER_ID not in ids:
        return None, ("markers %s are in view but not id %d -- this is the "
                      "wrong board" % (ids, MARKER_ID))
    return corners[ids.index(MARKER_ID)].reshape(4, 2), None


# ------------------------------------------------------------- self-test
def self_test(verbose=True):
    K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
    D = np.zeros(5)
    # A camera 2.2 m in front of the wearer, 1.4 m up, looking back at them
    # and rolled a little -- a plausible desk placement rather than a
    # convenient one, so a tool that only works axis-aligned cannot pass.
    t_true = np.array([0.10, 2.20, 1.40])
    rpy_true = (math.radians(-95.0), math.radians(3.0), math.radians(182.0))
    T_true = np.eye(4)
    T_true[:3, :3] = rpy_to_R(*rpy_true)
    T_true[:3, 3] = t_true

    worst_t, worst_r = 0.0, 0.0
    cases = [((0.0, 0.0, 1.05), (0.0, 0.0, 0.0)),
             ((0.18, 0.10, 1.20), (0.0, 0.25, 0.10)),
             ((-0.22, -0.05, 0.95), (0.15, 0.0, -0.30))]
    for centre, rpy in cases:
        cw = marker_corners_world(centre, rpy)
        T_wc_inv = np.linalg.inv(T_true)
        cam = (T_wc_inv[:3, :3] @ cw.T).T + T_wc_inv[:3, 3]
        img_pts = (K @ cam.T).T
        img_pts = img_pts[:, :2] / img_pts[:, 2:3]
        T_est, resid = solve(centre, rpy, img_pts, K, D)
        dt = float(np.linalg.norm(T_est[:3, 3] - t_true))
        dR = T_est[:3, :3].T @ T_true[:3, :3]
        ang = math.degrees(math.acos(
            max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))))
        worst_t, worst_r = max(worst_t, dt), max(worst_r, ang)
        if verbose:
            print("  marker at %-22s -> %.4f m, %.3f deg, residual %.3f px"
                  % (str(centre), dt, ang, resid))
    msg = "worst %.4f m and %.3f deg over %d marker placements" % (
        worst_t, worst_r, len(cases))
    return (worst_t < 0.005 and worst_r < 0.5), msg


def write(T, resid, route, out=OUT, note=""):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    rpy = R_to_rpy(T[:3, :3])
    with open(out, "w") as fh:
        fh.write("# scene camera pose in the ROBOT WORLD frame\n")
        fh.write("# written by scripts/calibrate_scene_camera_extrinsics.py\n")
        fh.write("# route: %s\n" % route)
        if note:
            fh.write("# %s\n" % note)
        fh.write("T_world_camera: %s\n" % json.dumps(
            [round(float(x), 6) for x in T.ravel()]))
        fh.write("translation_xyz: %s\n" % json.dumps(
            [round(float(x), 5) for x in T[:3, 3]]))
        fh.write("rpy: %s\n" % json.dumps([round(float(x), 6) for x in rpy]))
        fh.write("residual_px: %.4f\n" % resid)
        fh.write("marker_side_m: %s\n" % MARKER_M)
        fh.write("route: %s\n" % route)
    print("wrote %s" % out)
    print("  camera at (%.3f, %.3f, %.3f) m, rpy (%.1f, %.1f, %.1f) deg, "
          "residual %.3f px"
          % (T[0, 3], T[1, 3], T[2, 3], math.degrees(rpy[0]),
             math.degrees(rpy[1]), math.degrees(rpy[2]), resid))


def marker_pose_from_tf(frame="backpack", offset=(0.0, -0.16, 0.10)):
    """Marker pose in world, read from TF. Needs a running stack."""
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    rclpy.init()
    n = Node("scene_extrinsic_probe")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    import time
    end = time.time() + 8.0
    tr = None
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tr = buf.lookup_transform("world", frame, rclpy.time.Time())
            break
        except Exception:                                     # noqa: BLE001
            continue
    n.destroy_node()
    rclpy.shutdown()
    if tr is None:
        raise SystemExit(
            "REFUSING: no transform world -> %s. --marker-on-mount reads the "
            "marker's pose from the robot model, so the stack has to be up."
            % frame)
    t = tr.transform.translation
    q = tr.transform.rotation
    R = np.array([
        [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w),
         2 * (q.x * q.z + q.y * q.w)],
        [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2),
         2 * (q.y * q.z - q.x * q.w)],
        [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w),
         1 - 2 * (q.x ** 2 + q.y ** 2)]])
    centre = np.array([t.x, t.y, t.z]) + R @ np.asarray(offset, float)
    return tuple(centre), R_to_rpy(R)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--image", default=None)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--marker-on-mount", action="store_true")
    ap.add_argument("--mount-frame", default="backpack")
    ap.add_argument("--marker-at", nargs=3, type=float, default=None)
    ap.add_argument("--marker-rpy", nargs=3, type=float,
                    default=[0.0, 0.0, 0.0])
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)

    print("== the tool checks itself first ==")
    ok, msg = self_test()
    print(("  SELF-TEST PASS: " if ok else "  SELF-TEST FAILED: ") + msg)
    if not ok:
        return 2
    if a.self_test:
        return 0

    K, D = load_K()
    if a.live:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise SystemExit(
                "REFUSING: no camera. Run scripts/attach_scene_camera.sh.")
        img = None
        for _ in range(12):
            good, f = cap.read()
            if good:
                img = f
        cap.release()
    elif a.image:
        img = cv2.imread(a.image)
    else:
        print("give --image, --live or --self-test")
        return 2
    if img is None:
        raise SystemExit("no image")

    corners, why = detect(img, K, D)
    if corners is None:
        raise SystemExit("REFUSING: %s" % why)

    if a.marker_on_mount:
        centre, rpy = marker_pose_from_tf(a.mount_frame)
        route = "marker on %s, pose from TF" % a.mount_frame
    elif a.marker_at:
        centre, rpy = tuple(a.marker_at), tuple(a.marker_rpy)
        route = "marker placed by hand at %s" % (list(centre),)
    else:
        raise SystemExit("give --marker-on-mount or --marker-at X Y Z")

    T, resid = solve(centre, rpy, corners, K, D)
    if resid > 3.0:
        print("\nWARNING: %.2f px residual. Either the marker is not where "
              "you said, or the printed side is not %.3f m." % (resid,
                                                                MARKER_M))
    write(T, resid, route, a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
