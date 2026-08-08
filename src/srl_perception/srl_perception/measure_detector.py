#!/usr/bin/env python3
"""
measure_detector.py — detection rate, pose accuracy and latency, offline.

This produces the number that GATES THE USER STUDY: if detection rate at
working distance is under 95%, the study is blocked, because a detector that
misses one grasp in twenty injects a failure the participant will attribute to
the autonomy and no amount of analysis can separate the two afterwards.

It runs with NO CAMERA. Synthetic images of a known tag at a known pose are
rendered with a pinhole model and pushed through the SAME detector code path
the live node uses (cv2.aruco detectMarkers + solvePnP), so what is measured
is the real decode-and-pose pipeline. Ground truth is exact by construction,
which a live rig cannot give without a motion-capture system.

WHAT THIS CANNOT MEASURE, and therefore what remains unverified until a camera
is attached: real sensor noise, real motion blur, real lighting and exposure,
rolling shutter, the Kinova camera's actual intrinsics and distortion, and the
end-to-end ROS transport latency. The rendered figures are an UPPER BOUND on
detection rate and a LOWER BOUND on pose error.

    ros2 run srl_perception measure_detector
    ros2 run srl_perception measure_detector --sweep-distance --trials 200
"""
import argparse
import json
import math
import sys
import time

import cv2
import numpy as np

FAMILY = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE = 0.040          # m, matches the detector's default

# Kinova Gen3 wrist colour camera, 1280x720. Focal length from the published
# ~65 deg horizontal FOV: fx = (w/2) / tan(FOV/2). ASSUMPTION, flagged: the
# real intrinsics must come from the camera's own CameraInfo once attached.
IMG_W, IMG_H = 1280, 720
HFOV_DEG = 65.0
FX = (IMG_W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
K = np.array([[FX, 0, IMG_W / 2.0], [0, FX, IMG_H / 2.0], [0, 0, 1.0]])
D = np.zeros(5)


REFINE = {"none": cv2.aruco.CORNER_REFINE_NONE,
          "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
          "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG}


def _detector(refine="subpix"):
    d = cv2.aruco.Dictionary_get(FAMILY)
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = REFINE[refine]
    return d, p


def render(tag_id, rvec, tvec, dictionary, blur_px=0.0, noise_sigma=0.0,
           bg=255, seed=0):
    """Render a tag at a known pose through the pinhole model."""
    rng = np.random.default_rng(seed)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, 240)
    m = cv2.copyMakeBorder(marker, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)
    h = TAG_SIZE / 2.0
    pad = h * (m.shape[0] / 240.0)
    obj = np.array([[-pad, pad, 0], [pad, pad, 0], [pad, -pad, 0], [-pad, -pad, 0]],
                   float)
    img_pts, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    img_pts = img_pts.reshape(4, 2).astype(np.float32)
    # HANDEDNESS. `obj` is in the ArUco convention (TL, TR, BR, BL with +y UP),
    # but OpenCV's camera frame has +y DOWN, so object-+y projects to LARGER
    # image y. Pairing those with the marker image's own TL,TR,BR,BL therefore
    # reverses the winding and renders a MIRRORED tag — which AprilTag decoding
    # is not invariant to, so the detector silently finds nothing (measured:
    # 0/120 at every distance, while a vertical flip of the same crop decodes
    # immediately). Listing the marker corners BL, BR, TR, TL restores the
    # winding and leaves ground truth in the ArUco convention, directly
    # comparable with what solvePnP returns.
    h_img, w_img = m.shape[0], m.shape[1]
    src = np.array([[0, h_img], [w_img, h_img], [w_img, 0], [0, 0]], np.float32)
    H = cv2.getPerspectiveTransform(src, img_pts)
    # GROUND TRUTH the render actually realises. Reversing the corner winding
    # above fixes the mirror, but it also flips the rendered tag's own frame in
    # y. For a planar target every point is at z=0, so that flip IS a 180 deg
    # rotation about x -- applied in OBJECT space, hence R_render @ Rx(pi) and
    # not Rx(pi) @ R_render.
    #
    # The two forms COMMUTE for a tilt about x or about y, so checking those
    # axes cannot tell them apart; only a general in-plane axis can. Measured
    # over 200 random axes with no blur and no noise:
    #     Rx(pi) @ R_render   mean 21.79 deg, p95 54.07   <- wrong
    #     R_render @ Rx(pi)   mean  1.06 deg, p95  4.31   <- right
    # The wrong form is also noise-INDEPENDENT and scales with tilt, which is
    # how it was told apart from the square-marker planar-pose ambiguity (that
    # would be noise-driven and bimodal).
    canvas = np.full((IMG_H, IMG_W), bg, np.uint8)
    warped = cv2.warpPerspective(m, H, (IMG_W, IMG_H), borderValue=bg)
    canvas = np.minimum(canvas, warped)
    if blur_px > 0:
        k = int(2 * round(blur_px) + 1)
        canvas = cv2.GaussianBlur(canvas, (k, k), blur_px)
    if noise_sigma > 0:
        canvas = np.clip(canvas.astype(np.int16) +
                         rng.normal(0, noise_sigma, canvas.shape), 0, 255).astype(np.uint8)
    R_render, _ = cv2.Rodrigues(np.asarray(rvec, float))
    R_truth = R_render @ np.diag([1.0, -1.0, -1.0])          # R_render @ Rx(pi)
    return canvas, R_truth


def measure(distances, trials, blur_px, noise_sigma, tilt_deg_max=35.0, seed=0,
            refine="subpix"):
    dictionary, params = _detector(refine)
    rng = np.random.default_rng(seed)
    rows = []
    for z in distances:
        n_ok = 0
        pos_err, rot_err, lat = [], [], []
        for t in range(trials):
            tag_id = int(rng.integers(0, 20))
            # random in-plane offset and out-of-plane tilt, the two things
            # that actually vary as an arm approaches an object on a table
            tx = rng.uniform(-0.25, 0.25) * z
            ty = rng.uniform(-0.15, 0.15) * z
            tilt = math.radians(rng.uniform(0, tilt_deg_max))
            axis = rng.normal(size=3)
            axis[2] = 0.0
            axis /= max(1e-9, np.linalg.norm(axis))
            rvec = (axis * tilt).reshape(3, 1)
            tvec = np.array([[tx], [ty], [z]], float)
            img, R_truth = render(tag_id, rvec, tvec, dictionary, blur_px,
                                  noise_sigma, seed=int(rng.integers(0, 1 << 30)))
            t0 = time.monotonic()
            corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary,
                                                      parameters=params)
            hit = ids is not None and tag_id in ids.flatten()
            if hit:
                i = list(ids.flatten()).index(tag_id)
                h = TAG_SIZE / 2.0
                obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)
                ok, rv, tv = cv2.solvePnP(obj, corners[i].reshape(4, 2).astype(float),
                                          K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                lat.append((time.monotonic() - t0) * 1000.0)
                if ok:
                    n_ok += 1
                    pos_err.append(float(np.linalg.norm(tv.flatten() - tvec.flatten())))
                    R1, _ = cv2.Rodrigues(rv)
                    dR = R1.T @ R_truth
                    ang = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
                    rot_err.append(math.degrees(ang))
            else:
                lat.append((time.monotonic() - t0) * 1000.0)
        rows.append(dict(
            distance_m=z,
            detection_rate=n_ok / trials,
            pos_err_mean_mm=float(np.mean(pos_err) * 1000) if pos_err else float("nan"),
            pos_err_p95_mm=float(np.percentile(pos_err, 95) * 1000) if pos_err else float("nan"),
            rot_err_mean_deg=float(np.mean(rot_err)) if rot_err else float("nan"),
            rot_err_p95_deg=float(np.percentile(rot_err, 95)) if rot_err else float("nan"),
            detect_latency_ms=float(np.mean(lat)) if lat else float("nan"),
            trials=trials))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=120)
    ap.add_argument("--blur-px", type=float, default=0.8,
                    help="Gaussian blur sigma, standing in for focus + motion")
    ap.add_argument("--noise-sigma", type=float, default=3.0,
                    help="additive pixel noise, standing in for sensor noise")
    ap.add_argument("--working-distance", type=float, default=0.35,
                    help="the distance the STUDY GATE is evaluated at")
    ap.add_argument("--refine", default="subpix", choices=list(REFINE))
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    dists = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.80, 1.00]
    rows = measure(dists, a.trials, a.blur_px, a.noise_sigma, refine=a.refine)

    print("AprilTag 36h11, %.0f mm tag, %dx%d, fx=%.1f px (ASSUMED %.0f deg HFOV)"
          % (TAG_SIZE * 1000, IMG_W, IMG_H, FX, HFOV_DEG))
    print("blur sigma %.1f px, pixel noise sigma %.1f, %d trials per distance, "
          "corner refine = %s" % (a.blur_px, a.noise_sigma, a.trials, a.refine))
    print("SYNTHETIC IMAGES: real lighting, motion blur, rolling shutter and the")
    print("camera's true intrinsics are NOT modelled. Treat detection rate as an")
    print("UPPER bound and pose error as a LOWER bound.")
    print()
    print("%8s %10s %12s %12s %12s %12s %10s"
          % ("dist_m", "det_rate", "pos_mean_mm", "pos_p95_mm",
             "rot_mean_deg", "rot_p95_deg", "lat_ms"))
    for r in rows:
        print("%8.2f %10.3f %12.2f %12.2f %12.2f %12.2f %10.2f"
              % (r["distance_m"], r["detection_rate"], r["pos_err_mean_mm"],
                 r["pos_err_p95_mm"], r["rot_err_mean_deg"], r["rot_err_p95_deg"],
                 r["detect_latency_ms"]))

    wd = min(rows, key=lambda r: abs(r["distance_m"] - a.working_distance))
    print()
    print("STUDY GATE at the working distance of %.2f m:" % wd["distance_m"])
    print("  detection rate = %.1f%%" % (100 * wd["detection_rate"]))
    if wd["detection_rate"] < 0.95:
        print("  *** UNDER 95%: THE USER STUDY IS BLOCKED. ***")
        print("  A detector that misses one grasp in twenty injects failures the")
        print("  participant will attribute to the autonomy, and analysis cannot")
        print("  separate the two afterwards. Fix perception before running E2-E4.")
    else:
        print("  >= 95%: gate PASSED in simulation. Must be re-measured on the")
        print("  real cameras before any participant session - see the caveats above.")
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"rows": rows, "gate_distance_m": wd["distance_m"],
                       "gate_rate": wd["detection_rate"],
                       "blocked": wd["detection_rate"] < 0.95}, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
