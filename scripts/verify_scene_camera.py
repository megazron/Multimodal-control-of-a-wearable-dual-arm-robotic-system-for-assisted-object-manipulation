#!/usr/bin/env python3
"""End to end: a picture in, a wearer model out, and every claim labelled.

    .venv_pose/bin/python scripts/verify_scene_camera.py
    .venv_pose/bin/python scripts/verify_scene_camera.py --image shot.png

WHAT THIS IS FOR. The scene camera path has six stages and five of them can
fail silently. This drives all six on a REAL photograph of a REAL person --
not a render, because a learned detector on a rendered person is CLAUDE.md's
"learned model scores near zero: synthetic renderer out of distribution" --
and prints, for each stage, whether the number is MEASURED here or INFERRED
from something not present on this machine.

    1. a frame                       from a file, or the live camera
    2. the frame gate                dark / blank / two people
    3. body landmarks                the pose model
    4. metric placement              PnP against the intrinsic
    5. the segment gates             confidence, plausibility, speed
    6. the fusion                    tracked vs mannequin, per part

Stage 4 needs an intrinsic and an extrinsic. Without them this reports the
stages it CAN run and says plainly that the rest is not measured, rather than
substituting a plausible focal length -- which would make every stage after it
look like it worked.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_perception"))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))

from srl_perception import scene_geometry as SG              # noqa: E402
from srl_perception import wearer_tracking as WT             # noqa: E402

ROWS = []


def row(stage, verdict, measured, detail):
    ROWS.append((stage, verdict, measured, detail))
    print("  %-26s %-9s %-9s %s"
          % (stage[:26], verdict, "MEASURED" if measured else "inferred",
             detail[:88]))


def load_yaml(p):
    try:
        import yaml
        return yaml.safe_load(open(p))
    except Exception:                                         # noqa: BLE001
        return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=None,
                    help="a photograph of a person. Default: the bundled "
                         "reference frames if present.")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--model", default="models/mediapipe/pose_landmarker_full.task")
    a = ap.parse_args(argv)

    print("\n== SCENE CAMERA, END TO END ==")
    print("  %-26s %-9s %-9s %s" % ("stage", "verdict", "evidence", "detail"))

    # ---------------------------------------------------------- 0. device
    import glob
    vids = sorted(glob.glob("/dev/video*"))
    row("USB camera attached", "YES" if vids else "NO", True,
        ", ".join(vids) if vids else
        "no /dev/video* -- run scripts/attach_scene_camera.sh")

    # ------------------------------------------------------ 1. the frame
    import cv2
    img = None
    src = ""
    if a.live and vids:
        cap = cv2.VideoCapture(vids[0], cv2.CAP_V4L2)
        for _ in range(10):
            ok, f = cap.read()
            if ok:
                img = f
        cap.release()
        src = vids[0]
    if img is None:
        cands = ([a.image] if a.image else
                 sorted(glob.glob(os.path.join(WS, "recordings", "scene_camera",
                                               "*.jpg")))
                 + sorted(glob.glob("/tmp/pose.jpg"))
                 + sorted(glob.glob("/tmp/0000*.jpg")))
        for c in cands:
            if c and os.path.exists(c):
                img = cv2.imread(c)
                src = c
                break
    if img is None:
        row("a frame", "NONE", False,
            "no camera and no reference photograph -- nothing to run on")
        return 2
    row("a frame", "OK", bool(vids and a.live),
        "%dx%d from %s" % (img.shape[1], img.shape[0], src))

    # ------------------------------------------------- 2. calibration
    K = D = T = None
    d = load_yaml(os.path.join(WS, "config/scene_camera_intrinsics.yaml"))
    if d:
        K = np.array(d["camera_matrix"], float).reshape(3, 3)
        D = np.array(d.get("distortion", [0] * 5), float).ravel()
        row("intrinsic calibration", "OK", True,
            "fx %.1f cy %.1f, reprojection %.3f px over %d views"
            % (K[0, 0], K[1, 2], d.get("reprojection_error_px", -1),
               d.get("n_views", 0)))
    else:
        row("intrinsic calibration", "MISSING", True,
            "no config/scene_camera_intrinsics.yaml -- distances cannot be "
            "measured, only shapes")
    e = load_yaml(os.path.join(WS, "config/scene_camera_extrinsics.yaml"))
    if e:
        T = np.array(e["T_world_camera"], float).reshape(4, 4)
        row("camera pose in the robot", "OK", True,
            "at (%.2f, %.2f, %.2f) m, residual %.2f px, %s"
            % (T[0, 3], T[1, 3], T[2, 3], e.get("residual_px", -1),
               e.get("route", "?")))
    else:
        row("camera pose in the robot", "MISSING", True,
            "no config/scene_camera_extrinsics.yaml -- a body could be "
            "placed in the CAMERA frame but not in the robot's")

    # -------------------------------------------------- 3. the pose model
    model = os.path.join(WS, a.model)
    try:
        import mediapipe as mp
        from mediapipe.tasks import python as tp
        from mediapipe.tasks.python import vision
    except Exception as ex:                                   # noqa: BLE001
        row("pose model", "UNAVAILABLE", True,
            "%r -- run this with .venv_pose/bin/python" % (ex,))
        return 2
    if not os.path.exists(model):
        row("pose model", "MISSING", True, model)
        return 2
    det = vision.PoseLandmarker.create_from_options(
        vision.PoseLandmarkerOptions(
            base_options=tp.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.IMAGE, num_poses=2,
            min_pose_detection_confidence=0.5))

    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    mpimg = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    ts = []
    for _ in range(5):
        t0 = time.perf_counter()
        res = det.detect(mpimg)
        ts.append((time.perf_counter() - t0) * 1e3)
    n = len(res.pose_landmarks) if res.pose_landmarks else 0
    row("pose model", "OK" if n else "NO BODY", True,
        "%d person(s), %.1f ms median (%.0f Hz) on this CPU"
        % (n, sorted(ts)[len(ts) // 2], 1000.0 / max(1e-6, sorted(ts)[len(ts) // 2])))
    if not n:
        return 1

    # ---------------------------------------------------- 4. frame gate
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, why = WT.frame_is_usable(float(gray.mean()),
                                 float(np.percentile(gray, 97)
                                       - np.percentile(gray, 3)), n)
    row("frame gate", "PASS" if ok else "REFUSED", True, why)

    # ------------------------------------------------ 5. metric skeleton
    L, W = res.pose_landmarks[0], res.pose_world_landmarks[0]

    def dist(i, j):
        return math.sqrt(sum((getattr(W[i], c) - getattr(W[j], c)) ** 2
                             for c in "xyz"))
    limbs = dict(shoulder_width=dist(11, 12), L_upper=dist(11, 13),
                 L_fore=dist(13, 15), R_upper=dist(12, 14),
                 R_fore=dist(14, 16))
    row("metric limb lengths", "OK", True,
        "  ".join("%s %.3f" % (k, v) for k, v in limbs.items()))
    man = dict(upper=0.30, fore=0.26)
    row("vs the mannequin", "DIFFERS", True,
        "mannequin upper 0.300 fore 0.260; this person %+.0f / %+.0f mm"
        % (1000 * (limbs["L_upper"] - man["upper"]),
           1000 * (limbs["L_fore"] - man["fore"])))

    # ---------------------------------------------------- 6. placement
    if K is None:
        row("metric placement", "NOT RUN", False,
            "no intrinsic -- the body's DISTANCE cannot be solved")
        placed = None
    else:
        LMI = [0, 7, 8, 11, 12, 13, 14, 15, 16, 19, 20, 23, 24, 25, 26, 27, 28]
        obj, ipt = [], []
        for i in LMI:
            if i < len(L) and L[i].visibility >= 0.5:
                obj.append((W[i].x, W[i].y, W[i].z))
                ipt.append((L[i].x * img.shape[1], L[i].y * img.shape[0]))
        placed, resid = SG.lift_skeleton(obj, ipt, K, D)
        if placed is None:
            row("metric placement", "REFUSED", True, str(resid))
        else:
            row("metric placement", "OK", True,
                "body centre %.2f m from the camera, PnP residual %.2f px "
                "over %d landmarks" % (placed[:, 2].mean(), resid, len(obj)))

    # ---------------------------------------------------- 7. the gates
    pts, vis = {}, {}
    if placed is not None and T is not None:
        fr = SG.SceneFrames(K, D, T)
        LMN = {0: "nose", 11: "L-shoulder", 12: "R-shoulder", 13: "L-elbow",
               14: "R-elbow", 15: "L-wrist", 16: "R-wrist", 19: "L-hand-tip",
               20: "R-hand-tip", 23: "L-hip", 24: "R-hip"}
        used = [i for i in [0, 7, 8, 11, 12, 13, 14, 15, 16, 19, 20, 23, 24,
                            25, 26, 27, 28]
                if i < len(L) and L[i].visibility >= 0.5]
        cam = {i: placed[k] for k, i in enumerate(used)}
        for i, nm in LMN.items():
            if i in cam:
                pts[nm] = tuple(fr.camera_to_world(cam[i].reshape(1, 3))[0])
                vis[nm] = float(L[i].visibility)
    else:
        # The gates can still be exercised in the CAMERA frame, which is
        # honest as long as it says so: the confidence and plausibility gates
        # do not care which frame the points are in.
        LMN = {0: "nose", 11: "L-shoulder", 12: "R-shoulder", 13: "L-elbow",
               14: "R-elbow", 15: "L-wrist", 16: "R-wrist", 19: "L-hand-tip",
               20: "R-hand-tip", 23: "L-hip", 24: "R-hip"}
        for i, nm in LMN.items():
            if i < len(L):
                pts[nm] = (W[i].x, W[i].y, W[i].z)
                vis[nm] = float(L[i].visibility)
    est = WT.segments_from_joints(pts, vis, now=time.monotonic(),
                                  source="verify")
    good = [k for k, s in est.segments.items() if s.usable]
    bad = {k: s.reasons or ["conf %.2f" % s.conf]
           for k, s in est.segments.items() if not s.usable}
    row("segment gates", "OK", T is not None,
        "%d of %d usable%s" % (len(good), len(est.segments),
                               "" if not bad else
                               "; refused: " + ", ".join(
                                   "%s (%s)" % (k, v[0]) for k, v in
                                   list(bad.items())[:3])))

    # ---------------------------------------------------- 8. the fusion
    try:
        from srl_teleop import wearer_posture as wp
        man_model = wp.wearer_model("down")
    except Exception:                                         # noqa: BLE001
        man_model = []
    if man_model:
        fused, dec = WT.fuse(man_model, est, time.monotonic())
        ntr, ntot, text = WT.summarise(dec)
        row("fusion with the mannequin", "OK", T is not None, text)
    print()
    fails = [r for r in ROWS if r[1] in ("MISSING", "NO", "REFUSED", "NONE",
                                         "UNAVAILABLE")]
    print("%d stages, %d measured on this machine, %d not yet available"
          % (len(ROWS), sum(1 for r in ROWS if r[2]), len(fails)))
    for s, v, _m, d in fails:
        print("  %-26s %s  %s" % (s, v, d[:80]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
