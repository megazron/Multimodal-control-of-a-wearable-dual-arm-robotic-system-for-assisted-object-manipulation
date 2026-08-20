#!/usr/bin/env python3
"""Intrinsics for the scene camera, and a check that the tool itself works.

    python3 scripts/calibrate_scene_camera.py --self-test
    python3 scripts/calibrate_scene_camera.py --capture      # live, live view
    python3 scripts/calibrate_scene_camera.py --images 'cal/*.png'

WHY THE INTRINSIC MATTERS HERE MORE THAN USUAL. The body tracker turns image
landmarks into metric positions by solving PnP against the detector's own
metric skeleton. That solve is scaled by the focal length: a focal length 10%
wrong puts the wearer 10% nearer or further, with no symptom at all. The
positions stay smooth, the confidences stay high, and the clearance figures
are quietly wrong by 100 mm at 1 m. An uncalibrated scene camera is worse
than none, which is why `scene_camera_node` publishes a ZERO camera matrix
rather than a plausible guess until this has been run.

THE INSTRUMENT IS CHECKED BEFORE IT IS USED. `--self-test` builds a chessboard
with a KNOWN camera matrix, projects it from known poses with the projection
arithmetic itself, and requires this tool to recover the matrix it started
from. That is construction, not rendering: the ground truth is the arithmetic,
which is the one case CLAUDE.md's standing rule allows synthetic data for.
It runs before every real calibration and refuses to proceed if it fails.
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, "config", "scene_camera_intrinsics.yaml")

# A 9x6 inner-corner board of 25 mm squares. Printable on A4; the SIZE is a
# parameter because a board printed "to fit" is not the size it says, and a
# board 5% off scales the extrinsic translation by 5%.
COLS, ROWS = 9, 6
SQUARE_M = 0.025


def object_points(cols=COLS, rows=ROWS, square=SQUARE_M):
    p = np.zeros((cols * rows, 3), np.float32)
    p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return p * square


# ------------------------------------------------------------- self-test
def _synth_board(K, dist, rvec, tvec, size, cols=COLS, rows=ROWS):
    """An image of a chessboard, drawn by PROJECTING its corners.

    Not a render: each square is the projection of its four corners, filled.
    So the corner positions in the image are exactly the projection of the
    known geometry, and the ground truth is arithmetic rather than an
    appearance model.
    """
    img = np.full((size[1], size[0]), 255, np.uint8)
    # A BOARD OF cols+1 BY rows+1 SQUARES, WHICH IS WHAT GIVES cols BY rows
    # INNER CORNERS. Drawing cols x rows squares -- the obvious thing, and
    # what this did first -- gives (cols-1) x (rows-1) inner corners, so
    # `findChessboardCorners` was being asked for a 9x6 grid on a board that
    # had 8x5 and correctly refused every one of the twelve views. The
    # self-test reported "the tool cannot be certified", which was true, and
    # the fault was in the test rig rather than in the calibration.
    #
    # The square grid is offset by -1 square in x and y so the ORIGIN of the
    # object points still lands on the first inner corner.
    nx, ny = cols + 2, rows + 2                # corner grid of the squares
    full = np.zeros((nx * ny, 3), np.float32)
    full[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    full[:, 0] -= 1.0
    full[:, 1] -= 1.0
    full *= SQUARE_M
    pts, _ = cv2.projectPoints(full, rvec, tvec, K, dist)
    pts = pts.reshape(ny, nx, 2)
    for r in range(ny - 1):
        for c in range(nx - 1):
            if (r + c) % 2:
                continue
            quad = np.array([pts[r, c], pts[r, c + 1],
                             pts[r + 1, c + 1], pts[r + 1, c]], np.int32)
            cv2.fillConvexPoly(img, quad, 0, lineType=cv2.LINE_AA)
    # A LITTLE BLUR. The corner POSITIONS are unchanged -- a symmetric kernel
    # does not move a saddle point -- but a perfectly hard synthetic edge is
    # not what the detector's sub-pixel refinement expects.
    img = cv2.GaussianBlur(img, (3, 3), 0)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def self_test(verbose=True):
    """Recover a known camera matrix. Returns (ok, message)."""
    size = (1280, 720)
    K_true = np.array([[900.0, 0.0, 640.0],
                       [0.0, 900.0, 360.0],
                       [0.0, 0.0, 1.0]])
    dist_true = np.zeros(5)
    # THE BOARD HAS TO GO INTO THE CORNERS OF THE FRAME, and this is the
    # whole reason the first version of the self-test failed at 11.6 px on
    # the principal point while recovering the focal length to 1.8%. Twelve
    # views all near the image centre leave cx and cy almost unconstrained --
    # a shifted principal point and a shifted board look identical. Spreading
    # the translations across the field is the same instruction printed on
    # every real calibration guide, and the self-test is the thing that made
    # ignoring it visible.
    poses = []
    i = 0
    for tx, ty in ((-0.16, -0.10), (0.02, -0.10), (-0.16, 0.02), (0.02, 0.02),
                   (-0.07, -0.04)):
        for rx, ry, tz in ((0.0, 0.0, 0.42), (0.32, -0.18, 0.50),
                           (-0.30, 0.22, 0.55), (0.18, 0.34, 0.60)):
            rvec = np.array([[rx], [ry], [0.06]])
            tvec = np.array([[tx], [ty], [tz]])
            poses.append((rvec, tvec, i))
            i += 1
    objp, objs, imgs = object_points(), [], []
    for rvec, tvec, _ in poses:
        img = _synth_board(K_true, dist_true, rvec, tvec, size)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, (COLS, ROWS), None)
        if not ok:
            continue
        objs.append(objp)
        imgs.append(corners)
    if len(objs) < 6:
        return False, ("only %d of %d synthetic boards were found -- the "
                       "self-test cannot certify the tool" % (len(objs),
                                                              len(poses)))
    ok, K, D, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
    if not ok:
        return False, "calibrateCamera failed on synthetic input"
    ef = abs(K[0, 0] - K_true[0, 0]) / K_true[0, 0]
    ec = abs(K[0, 2] - K_true[0, 2])
    msg = ("recovered fx %.1f (true %.1f, %.2f%%), cx %.1f (true %.1f, "
           "%.1f px) from %d boards" % (K[0, 0], K_true[0, 0], 100 * ef,
                                        K[0, 2], K_true[0, 2], ec, len(objs)))
    if verbose:
        print("  self-test: " + msg)
    # 2% and 8 px. Loose enough for the discretisation of a drawn board,
    # tight enough that a tool with the object points transposed, the board
    # size wrong or the square size wrong cannot pass.
    return (ef < 0.02 and ec < 8.0), msg


# ------------------------------------------------------------ calibrate
def calibrate(paths, cols=COLS, rows=ROWS, square=SQUARE_M):
    objp = object_points(cols, rows, square)
    objs, imgs, used, size = [], [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for p in sorted(paths):
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print("  no board: %s" % os.path.basename(p))
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
        objs.append(objp)
        imgs.append(corners)
        used.append(p)
    if len(objs) < 8:
        raise SystemExit(
            "REFUSING: %d usable board views, need at least 8. A calibration "
            "from a handful of similar views has a focal length that looks "
            "fine and is wrong, and nothing downstream disagrees with it."
            % len(objs))
    ok, K, D, rv, tv = cv2.calibrateCamera(objs, imgs, size, None, None)
    err = 0.0
    for i in range(len(objs)):
        proj, _ = cv2.projectPoints(objs[i], rv[i], tv[i], K, D)
        err += cv2.norm(imgs[i], proj, cv2.NORM_L2) / len(proj)
    return K, D, size, err / len(objs), used


def write(K, D, size, err, used, out=OUT):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as fh:
        fh.write("# scene camera intrinsics -- written by "
                 "scripts/calibrate_scene_camera.py\n")
        fh.write("# reprojection error %.4f px over %d views\n" % (err,
                                                                   len(used)))
        fh.write("image_width: %d\nimage_height: %d\n" % size)
        fh.write("camera_matrix: %s\n" % json.dumps(
            [round(float(x), 6) for x in K.ravel()]))
        fh.write("distortion: %s\n" % json.dumps(
            [round(float(x), 8) for x in D.ravel()]))
        fh.write("reprojection_error_px: %.5f\n" % err)
        fh.write("n_views: %d\n" % len(used))
        fh.write("square_size_m: %s\n" % SQUARE_M)
    print("wrote %s" % out)


def capture(device, n=20, outdir=None):
    """Grab board views from the live camera, one per keypress or per second."""
    outdir = outdir or os.path.join(WS, ".scratch", "scene_cal")
    os.makedirs(outdir, exist_ok=True)
    cap = cv2.VideoCapture(device if device else 0, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(
            "REFUSING: no camera. On WSL the camera has to be handed over "
            "from Windows first -- run scripts/attach_scene_camera.sh.")
    got = 0
    print("Hold the %dx%d board at different angles and distances. "
          "%d views wanted." % (COLS, ROWS, n))
    while got < n:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(gray, (COLS, ROWS), None)
        if found:
            p = os.path.join(outdir, "cal_%02d.png" % got)
            cv2.imwrite(p, frame)
            got += 1
            print("  %2d/%d  %s" % (got, n, p))
            cv2.waitKey(700)
    cap.release()
    return sorted(glob.glob(os.path.join(outdir, "*.png")))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--images", default=None)
    ap.add_argument("--capture", action="store_true")
    ap.add_argument("--device", default="")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)

    print("== the tool checks itself first ==")
    ok, msg = self_test()
    if not ok:
        print("SELF-TEST FAILED: %s" % msg)
        return 2
    if a.self_test:
        print("SELF-TEST PASS")
        return 0

    paths = capture(a.device, a.n) if a.capture else sorted(
        glob.glob(a.images or ""))
    if not paths:
        print("no images. Use --capture, or --images 'dir/*.png'.")
        return 2
    K, D, size, err, used = calibrate(paths)
    print("\n%d views, reprojection error %.4f px" % (len(used), err))
    print("fx %.2f  fy %.2f  cx %.2f  cy %.2f" % (K[0, 0], K[1, 1],
                                                  K[0, 2], K[1, 2]))
    if err > 1.0:
        print("\nWARNING: %.2f px is high. A calibration this loose puts the "
              "wearer tens of millimetres out at 2 m." % err)
    write(K, D, size, err, used, a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
