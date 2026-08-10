#!/usr/bin/env python3
"""Open-vocabulary detection + known-dimension depth fit, on REAL RGB-D.

WHY THIS DATASET AND NOT A RENDER. The synthetic renderer is out of
YOLO-World's distribution -- 0-4% on our flat primitives against 0.89-0.91 on
a real photograph -- so nothing rendered here can measure the detector. LM-O
(LineMOD-Occluded, BOP) is REAL RGB-D of household objects with ground-truth
6D poses and real sensor depth noise, under occlusion.

WHAT IT ANSWERS
  * does an open-vocabulary detector find real objects, and how often
  * how well-localised is its box (IoU against ground truth)
  * does the known-dimension depth fit hold up on a REAL sensor rather than
    the modelled noise used in measure_depth_pose_accuracy.py

WHAT IT DOES NOT ANSWER, and this is stated rather than buried: these are not
OUR objects in OUR lighting. The detection rate on the 40 mm block, the 45 mm
part and the multimeter can only come from the lab, and the procedure for that
is in docs/NEXT_SESSION.md.

    .percep_venv/bin/python scripts/measure_detector_on_real_rgbd.py [--n 60]
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LMO = os.path.join(ROOT, "datasets/lmo")
OUT = os.path.join(ROOT, "recordings/baselines/detector_real_rgbd.json")

# LM-O object ids -> an open-vocabulary prompt. The prompt is the ONLY thing
# the detector is told; no class weights, no fine-tuning.
PROMPT = {1: "ape figurine", 5: "watering can", 6: "cat figurine",
          8: "power drill", 9: "duck figurine", 10: "egg carton",
          11: "glue bottle", 12: "hole puncher"}


def find_split():
    for cand in ("test", "test_all"):
        p = os.path.join(LMO, cand)
        if os.path.isdir(p):
            return p
    for root, dirs, _ in os.walk(LMO):
        for d in dirs:
            if d.startswith("test"):
                return os.path.join(root, d)
    return None


def model_extents():
    """Object bounding-box dimensions, mm, from the BOP models_info.json."""
    for root, _d, files in os.walk(LMO):
        if "models_info.json" in files:
            info = json.load(open(os.path.join(root, "models_info.json")))
            return {int(k): (v["size_x"], v["size_y"], v["size_z"])
                    for k, v in info.items()}
    return {}


def fit_known_box(P, size_m):
    """Same estimator as measure_depth_pose_accuracy: segment, then fit."""
    if len(P) < 20:
        return None
    z_near = np.percentile(P[:, 2], 2.0)
    Q = P[P[:, 2] <= z_near + size_m[2] + 0.010]
    if len(Q) < 20:
        Q = P
    c = np.empty(3)
    for i, s in enumerate(size_m):
        lo = np.percentile(Q[:, i], 2.0)
        hi = np.percentile(Q[:, i], 98.0)
        c[i] = 0.5 * (lo + hi) if (hi - lo) > 0.75 * s else lo + s / 2.0
    return c


def iou(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    ua = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / ua if ua > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60, help="frames to evaluate")
    a = ap.parse_args()

    split = find_split()
    if split is None:
        print("LM-O not unpacked yet at %s -- run the download first." % LMO)
        return 2
    ext = model_extents()
    print("split: %s" % split)
    print("model extents for %d objects" % len(ext))

    from ultralytics import YOLOWorld
    import cv2
    model = YOLOWorld("yolov8s-world.pt")
    # SET CLASSES BEFORE THE FIRST GPU PREDICT. ultralytics 8.4 + torch 2.13
    # raises "Expected all tensors to be on the same device" if set_classes()
    # is called after the model has moved to CUDA. Documented in CLAUDE.md.
    names = [PROMPT[k] for k in sorted(PROMPT)]
    model.set_classes(names)

    scenes = sorted(d for d in os.listdir(split)
                    if os.path.isdir(os.path.join(split, d)))
    rows, done = [], 0
    for sc in scenes:
        base = os.path.join(split, sc)
        try:
            gt = json.load(open(os.path.join(base, "scene_gt.json")))
            gti = json.load(open(os.path.join(base, "scene_gt_info.json")))
            cam = json.load(open(os.path.join(base, "scene_camera.json")))
        except Exception:                                        # noqa: BLE001
            continue
        for key in sorted(gt, key=lambda x: int(x)):
            if done >= a.n:
                break
            rgb = os.path.join(base, "rgb", "%06d.png" % int(key))
            dep = os.path.join(base, "depth", "%06d.png" % int(key))
            if not (os.path.exists(rgb) and os.path.exists(dep)):
                continue
            im = cv2.imread(rgb)
            dm = cv2.imread(dep, cv2.IMREAD_UNCHANGED).astype(np.float32)
            K = np.array(cam[key]["cam_K"], dtype=float).reshape(3, 3)
            ds = cam[key].get("depth_scale", 1.0)
            res = model.predict(im, conf=0.05, verbose=False)[0]
            dets = []
            for b, c in zip(res.boxes.xyxy.cpu().numpy(),
                            res.boxes.cls.cpu().numpy().astype(int)):
                dets.append((names[c], b))
            for inst, info in zip(gt[key], gti[key]):
                oid = inst["obj_id"]
                if oid not in PROMPT or info.get("visib_fract", 0) < 0.3:
                    continue
                gx, gy, gw, gh = info["bbox_visib"]
                gbox = (gx, gy, gx + gw, gy + gh)
                want = PROMPT[oid]
                cands = [d for d in dets if d[0] == want]
                best = max((iou(gbox, d[1]), d[1]) for d in cands) \
                    if cands else (0.0, None)
                rec = dict(scene=sc, im=int(key), obj=oid, prompt=want,
                           detected=bool(best[1] is not None and best[0] > 0.1),
                           iou=round(float(best[0]), 3))
                if rec["detected"]:
                    x0, y0, x1, y1 = [int(v) for v in best[1]]
                    x0, y0 = max(0, x0), max(0, y0)
                    sub = dm[y0:y1, x0:x1]
                    ys, xs = np.nonzero(sub)
                    if len(xs) > 40:
                        z = sub[ys, xs] * ds / 1000.0
                        X = (xs + x0 - K[0, 2]) * z / K[0, 0]
                        Y = (ys + y0 - K[1, 2]) * z / K[1, 1]
                        P = np.stack([X, Y, z], axis=1)
                        sz = tuple(v / 1000.0 for v in ext.get(oid,
                                                               (50, 50, 50)))
                        c = fit_known_box(P, sz)
                        gtt = np.array(inst["cam_t_m2c"]) / 1000.0
                        if c is not None:
                            rec["fit_err_mm"] = round(
                                float(np.linalg.norm(c - gtt) * 1000), 1)
                rows.append(rec)
            done += 1
        if done >= a.n:
            break

    n = len(rows)
    det = [r for r in rows if r["detected"]]
    errs = [r["fit_err_mm"] for r in det if "fit_err_mm" in r]
    print("\n%d object instances over %d frames" % (n, done))
    print("  detection rate      %.1f%%  (%d/%d)"
          % (100.0 * len(det) / max(1, n), len(det), n))
    if det:
        print("  median box IoU      %.2f" % np.median([r["iou"] for r in det]))
    if errs:
        e = np.array(errs)
        print("  known-fit pose error  median %.1f mm   mean %.1f   "
              "p90 %.1f   n=%d" % (np.median(e), e.mean(),
                                   np.percentile(e, 90), len(e)))
        print("  fraction within 10 mm  %.0f%%"
              % (100.0 * (e <= 10).mean()))
    else:
        print("  no pose fits produced")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(rows=rows, frames=done), open(OUT, "w"), indent=2)
    print("-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
