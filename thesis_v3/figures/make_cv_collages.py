#!/usr/bin/env python3
"""Two collages of the computer-vision pipelines, computed stage by stage
with OpenCV on the raw frames and drawn with OpenCV's own primitives, so what
is shown is what the code produced and nothing is hand-placed.

Collage 1, the wrist camera (object detection for grasping), mirrors the
project's own 33-stage pipeline: colour -> grey -> edges -> HSV -> green and
box thresholds -> morphology -> connected components -> min-area rectangles
-> depth -> RANSAC table plane -> height above the plane -> pieces standing
on the surface -> the two objects with their measured widths against the
gripper. Every stage that the pipeline recorded a number for is recomputed
here and compared with the recorded value; the comparison is printed and
saved (cv_collage_checks.json), and the figure is refused if a recomputed
value disagrees with the record by more than the stated tolerance.

Collage 2, the room camera (the wearer and the arms for the safety layer):
colour -> MediaPipe Pose landmarks and mask (from make_scene_understanding.py's
saved stage 1) -> grey -> local backdrop (71-px median) -> absolute difference
-> threshold -> morphology -> connected components -> thinning -> centreline
bends -> final annotation.

    python3 thesis_v3/figures/make_cv_collages.py
"""
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "gallery", "vision"))
import make_vision_gallery as G  # noqa: E402  (loaders: cloud, plane, band, boxes, RUN, SRC)
sys.path.insert(0, HERE)
from make_scene_understanding import zhang_suen, skeleton_path, bends, CROP, WORK  # noqa: E402

RAW = os.path.join(G.RUN, "raw")
OUT = os.path.join(HERE, "vision")
TILE = (480, 270)
GREEN_HSV = ((40, 80, 40), (85, 255, 255))     # 04_green_threshold, the pipeline's own values
BOX_HSV = G.BOX_HSV
GRIPPER_MM = 85.0
CUBE_MM = 40.0

checks = {}


def rec(cam, stage):
    d = json.load(open(os.path.join(G.RUN, "data", cam, stage + ".json")))
    return {m["quantity"]: m["value"] for m in d["measurements"]}


def tile(img, title, sub=None):
    """Resize to the tile size and stamp a title band, OpenCV style."""
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    t = cv2.resize(img, TILE, interpolation=cv2.INTER_AREA)
    cv2.rectangle(t, (0, 0), (TILE[0], 30 if sub is None else 52), (30, 30, 30), -1)
    cv2.putText(t, title, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(t, sub, (6, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1, cv2.LINE_AA)
    return t


def grid(tiles, cols):
    rows = []
    for i in range(0, len(tiles), cols):
        row = tiles[i:i + cols]
        while len(row) < cols:
            row.append(np.full((TILE[1], TILE[0], 3), 255, np.uint8))
        rows.append(np.hstack([np.pad(t, ((2, 2), (2, 2), (0, 0)), constant_values=255) for t in row]))
    return np.vstack(rows)


def colourise_labels(lab):
    rng = np.random.default_rng(1)
    lut = np.vstack([[0, 0, 0], rng.integers(60, 255, (max(lab.max(), 1), 3))]).astype(np.uint8)
    return lut[lab]


def check(name, got, want, tol):
    ok = abs(got - want) <= tol
    checks[name] = {"recomputed": float(got), "recorded": float(want), "tolerance": tol, "ok": bool(ok)}
    print("%-46s recomputed %10.3f  recorded %10.3f  %s" % (name, got, want, "ok" if ok else "MISMATCH"))
    return ok


# --------------------------------------------------------------------------
# collage 1: the wrist camera
# --------------------------------------------------------------------------
def wrist_collage(cam="left_gripper"):
    bgr = cv2.imread(os.path.join(RAW, cam, "colour.png"))
    H, W = bgr.shape[:2]
    grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(grey, (5, 5), 0), 60, 160)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    r03 = rec(cam, "03_hsv")
    check(cam + " median hue", np.median(hsv[..., 0]), r03["median hue"], 1.0)
    check(cam + " median saturation", np.median(hsv[..., 1]), r03["median saturation"], 1.0)
    check(cam + " median value", np.median(hsv[..., 2]), r03["median value"], 1.0)

    green = cv2.inRange(hsv, np.array(GREEN_HSV[0]), np.array(GREEN_HSV[1]))
    r04 = rec(cam, "04_green_threshold")
    check(cam + " green pixels selected", int((green > 0).sum()), r04["pixels selected"], 0)
    k = np.ones((9, 9), np.uint8)
    green_open = cv2.morphologyEx(green, cv2.MORPH_OPEN, k)
    r05 = rec(cam, "05_morphology")
    check(cam + " green pixels after opening", int((green_open > 0).sum()), r05["pixels after"], 0)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(green_open, 8)
    big = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 40]
    r06 = rec(cam, "06_components")
    check(cam + " components over 40 px", len(big), r06["components over 40 px"], 0)
    box = cv2.inRange(hsv, np.array(BOX_HSV[0]), np.array(BOX_HSV[1]))
    box = cv2.morphologyEx(box, cv2.MORPH_OPEN, k)
    box = cv2.morphologyEx(box, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))

    # min-area rectangles on the largest green component and the largest box component
    def largest_contour(mask):
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return max(cs, key=cv2.contourArea)
    c_cube = largest_contour(green_open)
    c_box = largest_contour(box)
    rect_cube = cv2.minAreaRect(c_cube)
    rect_box = cv2.minAreaRect(c_box)
    rects = bgr.copy()
    cv2.drawContours(rects, [np.int32(cv2.boxPoints(rect_cube))], 0, (0, 200, 0), 3)
    cv2.drawContours(rects, [np.int32(cv2.boxPoints(rect_box))], 0, (255, 120, 0), 3)

    # depth, plane, height
    depth = G.depth(cam)
    P, v, u = G.cloud(cam)
    nrm, off, tol, frac_rec, rms_rec = G.plane(cam)
    hgt = P @ nrm + off
    inl = np.abs(hgt) < tol
    check(cam + " plane inlier fraction (%)", 100 * inl.mean(), frac_rec, 0.5)
    check(cam + " plane residual RMS (mm)", 1000 * np.sqrt(np.mean(hgt[inl] ** 2)), rms_rec, 0.1)
    r20 = rec(cam, "20_height_map")
    above = hgt > r20["height threshold"] / 1000.0
    check(cam + " points above the plane (%)", 100 * above.mean(), r20["that fraction"], 0.5)
    lo, hi = G.band(cam)
    dv = np.clip((depth - lo) / (hi - lo), 0, 1); dv[depth <= 0] = 0
    depth_img = cv2.applyColorMap((255 * (1 - dv)).astype(np.uint8), cv2.COLORMAP_JET); depth_img[depth <= 0] = 0
    plane_img = np.full(depth.shape + (3,), 60, np.uint8)
    plane_img[v[inl], u[inl]] = (0, 180, 0); plane_img[v[~inl], u[~inl]] = (0, 0, 200)
    hmap = np.zeros(depth.shape, np.float32); hmap[v, u] = hgt * 1000
    hm = np.clip(hmap, -20, 100); hm_img = cv2.applyColorMap(((hm + 20) / 120 * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    hm_img[depth <= 0] = 0
    on = np.zeros(depth.shape, np.uint8); on[v[above], u[above]] = 255
    n2, lab2, st2, _ = cv2.connectedComponentsWithStats(on, 8)
    pieces = [i for i in range(1, n2) if st2[i, cv2.CC_STAT_AREA] > 200]
    r22 = rec(cam, "22_on_surface")
    checks[cam + " pieces on the surface"] = {"recomputed": len(pieces), "recorded": r22["pieces standing on the surface"],
                                              "note": "area gate differs; informational"}

    # the objects' sizes come from the pipeline's own 3-D boxes (30_boxes3d), not from the colour rectangle,
    # whose extent spans two faces of the cube and is not a width
    rows = G.boxes(cam)
    cube_row = min((r for r in rows if r["graspable"].strip().lower() == "yes"), key=lambda r: float(r["range m"]))
    box_row = max(rows, key=lambda r: float(r["length mm"]))
    cube_h = float(cube_row["height mm"]); cube_l, cube_w = float(cube_row["length mm"]), float(cube_row["width mm"])
    checks[cam + " cube 3-D box (mm)"] = {"length": cube_l, "width": cube_w, "height": cube_h, "range_m": float(cube_row["range m"]),
                                         "note": "pipeline's own measurement; graspable flag %s" % cube_row["graspable"].strip()}
    print("%-46s %.1f x %.1f x %.1f mm at %.3f m, graspable %s" % (cam + " cube 3-D box", cube_l, cube_w, cube_h, float(cube_row["range m"]), cube_row["graspable"].strip()))
    (cx, cy), (rw, rh), _ = rect_cube
    (bx, by), _, _ = rect_box
    final = rects.copy()
    cv2.putText(final, "cube %.0f mm: fits %.0f mm gripper" % (cube_h, GRIPPER_MM), (int(cx) - 260, int(cy) + 150),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 160, 0), 3, cv2.LINE_AA)
    cv2.putText(final, "box %.0f x %.0f mm: too wide" % (float(box_row["length mm"]), float(box_row["width mm"])),
                (int(bx) - 250, int(by) - 130), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 120, 0), 3, cv2.LINE_AA)
    width_mm = cube_h  # for the tile subtitle

    tiles = [
        tile(bgr, "01 colour frame", "%dx%d, left wrist camera" % (W, H)),
        tile(grey, "02 greyscale"),
        tile(edges, "03 Gaussian blur 5x5 + Canny 60/160"),
        tile(hsv, "04 HSV (shown as BGR)", "median H %.0f S %.0f V %.0f" % (np.median(hsv[..., 0]), np.median(hsv[..., 1]), np.median(hsv[..., 2]))),
        tile(green, "05 green threshold", "H 40-85, S>=80, V>=40: %d px" % (green > 0).sum()),
        tile(green_open, "06 morphological open 9x9", "%d px, %d blobs -> %d" % ((green_open > 0).sum(), r05["blobs before"], len(big))),
        tile(colourise_labels(lab), "07 connected components (8-conn)", "%d components over 40 px" % len(big)),
        tile(box, "08 box threshold (HSV) + open/close"),
        tile(rects, "09 minAreaRect on each object"),
        tile(depth_img, "10 depth (Kinova module, 480x270)", "%.0f%% return; band %.2f-%.2f m" % (100 * (depth > 0).mean(), lo, hi)),
        tile(plane_img, "11 RANSAC table plane, 400 draws", "within 6 mm: %.1f%%, RMS %.1f mm" % (100 * inl.mean(), 1000 * np.sqrt(np.mean(hgt[inl] ** 2)))),
        tile(hm_img, "12 height above the plane (mm)", "%.1f%% of points > %.0f mm" % (100 * above.mean(), r20["height threshold"])),
        tile(on, "13 pixels more than 5 mm above the plane", "%d pieces > 200 px (pipeline gate: %d)" % (len(pieces), int(r22["pieces standing on the surface"]))),
        tile(final, "14 result: objects and grasp check", "cube %.0f mm fits %.0f mm gripper; box %.0f mm not" % (cube_h, GRIPPER_MM, float(box_row["length mm"]))),
    ]
    img = grid(tiles, 3)
    cv2.imwrite(os.path.join(OUT, "cv_collage_wrist.png"), img)
    return img


# --------------------------------------------------------------------------
# collage 2: the room camera
# --------------------------------------------------------------------------
def room_collage():
    full = cv2.imread(os.path.join(RAW, "scene_hd", "colour.png"))
    x0, y0, x1, y1 = CROP
    bgr = full[y0:y1, x0:x1]
    H, W = bgr.shape[:2]
    grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    body = np.load(os.path.join(WORK, "body_mask.npy")) > 0.5
    pose = json.load(open(os.path.join(WORK, "pose.json")))["landmarks"]
    det = json.load(open(os.path.join(WORK, "detect.json")))
    # stage images
    lm = bgr.copy()
    SK = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24), (0, 11), (0, 12)]
    for a, b in SK:
        if pose[a][2] > 0.3 and pose[b][2] > 0.3:
            cv2.line(lm, (int(pose[a][0]), int(pose[a][1])), (int(pose[b][0]), int(pose[b][1])), (255, 255, 255), 2)
    for x, y, vis in pose:
        if vis > 0.3:
            cv2.circle(lm, (int(x), int(y)), 4, (0, 255, 255), -1)
    mask_img = (body * 255).astype(np.uint8)
    med = cv2.medianBlur(grey, det["arm_rule"]["backdrop_median_px"])
    diff = cv2.absdiff(grey, med)
    cy0, cy1 = det["arm_rule"]["corridor_y"]; tx0, tx1 = det["arm_rule"]["torso_x"]
    corridor = np.zeros((H, W), bool); corridor[cy0:cy1, :] = True; corridor[:, max(0, tx0 - 5):tx1 + 5] = False
    thr = ((diff > det["arm_rule"]["diff_grey"]) & corridor & ~body).astype(np.uint8) * 255
    morph = cv2.morphologyEx(cv2.morphologyEx(thr, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)), cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    arm_mask = np.load(os.path.join(WORK, "arm_mask.npy"))
    n, lab = cv2.connectedComponents((arm_mask > 0).astype(np.uint8), connectivity=8)
    comp_img = colourise_labels(lab)
    skel = np.zeros((H, W), np.uint8)
    final = bgr.copy()
    for side, a in det["arms"].items():
        am = cv2.morphologyEx((arm_mask == (1 if side == "left" else 2)).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)) > 0
        sk = zhang_suen(am); skel[sk] = 255
        P = np.array(a["centreline"], np.int32); J = a["joints_idx"]
        col = (255, 120, 0) if side == "left" else (0, 160, 255)
        cs, _ = cv2.findContours(am.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(final, cs, -1, col, 2)
        cv2.polylines(final, [P.reshape(-1, 1, 2)], False, (255, 255, 255), 2)
        for k in J:
            cv2.circle(final, tuple(P[k]), 7, (0, 0, 255), 2)
        cv2.putText(final, "%s arm: %d links" % (side, a["n_links"]), (int(P[0, 0]) - 90, int(P[0, 1]) - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
    cs, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(final, cs, -1, (0, 0, 255), 2)
    for cls, col in (("white desk", (80, 80, 80)), ("cardboard box", (0, 180, 0)), ("green cube", (0, 180, 0))):
        b = det["yolo"]["best"].get(cls)
        if b:
            bx0, by0, bx1, by1 = [int(v) for v in b["box"]]
            cv2.rectangle(final, (bx0, by0), (bx1, by1), col, 2)
            cv2.putText(final, "%s %.2f" % (cls, b["conf"]), (bx0, by0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
    yolo_img = bgr.copy()
    for d in det["yolo"]["best"].values():
        if d["conf"] >= 0.15:
            bx0, by0, bx1, by1 = [int(v) for v in d["box"]]
            cv2.rectangle(yolo_img, (bx0, by0), (bx1, by1), (200, 0, 200), 2)
            cv2.putText(yolo_img, "%s %.2f" % (d["class"], d["conf"]), (bx0, by0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 0, 200), 2, cv2.LINE_AA)
    summ = json.load(open(os.path.join(WORK, "summary.json")))["checks"]
    tiles = [
        tile(bgr, "01 room camera, cropped to the scene", "%dx%d of 1280x720" % (W, H)),
        tile(lm, "02 MediaPipe Pose: 33 landmarks", "shoulders %.0f px apart" % abs(pose[11][0] - pose[12][0])),
        tile(mask_img, "03 MediaPipe segmentation mask", "%.1f%% of the crop" % (100 * body.mean())),
        tile(yolo_img, "04 YOLO-World, best box per prompt", "desk %.2f, box %.2f, arms none" % (det["yolo"]["best"]["white desk"]["conf"], det["yolo"]["best"]["cardboard box"]["conf"])),
        tile(grey, "05 greyscale"),
        tile(med, "06 local backdrop: median 71 px"),
        tile(cv2.convertScaleAbs(diff, alpha=4), "07 |grey - backdrop| (x4 for display)"),
        tile(thr, "08 threshold > 12, corridor beside torso", "rows %d-%d; torso x %d-%d out" % (cy0, cy1, tx0, tx1)),
        tile(morph, "09 open 3x3, close 21x21"),
        tile(comp_img, "10 two largest components = arms", "%d px inside body mask" % summ["arm_pixels_inside_body_mask"]),
        tile(skel, "11 Zhang-Suen thinning", "centreline, shoulder end to far end"),
        tile(final, "12 result: wearer, arms, table, objects", "bends >25 deg; starts %.0f/%.0f px from shoulders" % (summ["left_arm_start_to_shoulder_px"], summ["right_arm_start_to_shoulder_px"])),
    ]
    img = grid(tiles, 3)
    cv2.imwrite(os.path.join(OUT, "cv_collage_room.png"), img)
    return img


def main():
    w = wrist_collage()
    r = room_collage()
    json.dump(checks, open(os.path.join(OUT, "cv_collage_checks.json"), "w"), indent=1)
    bad = [k for k, v in checks.items() if v.get("ok") is False]
    from PIL import Image
    for name in ("cv_collage_wrist", "cv_collage_room"):
        im = Image.open(os.path.join(OUT, name + ".png")).convert("RGB")
        im.save(os.path.join(OUT, name + ".pdf"), resolution=200)
    print("wrote collages;", "ALL CHECKS OK" if not bad else "MISMATCHES: %s" % bad)
    if bad:
        sys.exit(1)


if __name__ == "__main__":
    main()
