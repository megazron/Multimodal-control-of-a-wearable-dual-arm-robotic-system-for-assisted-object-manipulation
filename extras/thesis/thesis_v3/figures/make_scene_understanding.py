#!/usr/bin/env python3
"""Scene understanding on the room camera: WHO is in the scene and WHERE
the arms are, from one real frame, with a different method for each thing
because no single detector covers them.

  wearer      MediaPipe Pose landmarker (the tracker the safety layer runs):
              33 landmarks and a body segmentation mask.
  robot arms  white links on a white backdrop, which defeats every learned
              detector tried (YOLO-World, FastSAM text prompts). Engineered
              instead: the backdrop is smooth, so a 71-px median of the
              grey image is the local backdrop level, and anything that
              differs from it by more than 12 grey levels inside the arm
              corridor (beside the torso, between shoulder height and the
              table) is structure. FastSAM segments that overlap a
              candidate by half are merged in; the two largest components
              are the arms. Each arm is thinned (Zhang-Suen), the skeleton
              walked from the shoulder end to its farthest point, and joints
              placed where the centreline bends by more than 25 degrees,
              which is what separates one link from the next.
  table and   YOLO-World, open vocabulary, with the prompts that describe
  objects     these objects ("white desk", "cardboard box", "green cube",
              "tripod", "camera"), plus the project's own colour detector
              for the green cube as a cross-check.

Three stages, because MediaPipe and ultralytics live in different venvs:

  .venv_pose/bin/python   extras/thesis/thesis_v3/figures/make_scene_understanding.py pose
  .venv_vision/bin/python extras/thesis/thesis_v3/figures/make_scene_understanding.py detect
  python3                 extras/thesis/thesis_v3/figures/make_scene_understanding.py compose

Intermediate results are kept under figures/vision/scene_understanding/ so
every number in the figure can be checked, and the figure is
figures/vision/scene_understanding.pdf (+ _300.png).
"""
import collections
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 4))
FRAME = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141", "raw", "scene_hd", "colour.png")
CROP = (330, 90, 950, 720)          # x0, y0, x1, y1: the wearer, the arms and the table
WORK = os.path.join(HERE, "vision", "scene_understanding")
OUT = os.path.join(HERE, "vision", "scene_understanding")
POSE_MODEL = os.path.join(ROOT, "models", "mediapipe", "pose_landmarker_full.task")
YOLO_PROMPTS = ["mannequin", "white desk", "cardboard box", "green cube", "tripod", "camera", "robotic arm"]
YOLO_CONF = 0.10
BACKDROP_MEDIAN_PX = 71
ARM_DIFF_GREY = 12
BEND_DEG = 25.0


def load_crop():
    import cv2
    im = cv2.imread(FRAME)
    x0, y0, x1, y1 = CROP
    return im[y0:y1, x0:x1]


# --------------------------------------------------------------------------
# stage 1: the wearer (MediaPipe Pose)
# --------------------------------------------------------------------------
def stage_pose(camera=None):
    """MediaPipe Pose on the cropped room frame (default) or, with a camera
    name, on that camera's full frame (used for the room depth camera, whose
    colour stream is what the live wearer tracker consumes)."""
    import cv2
    import mediapipe as mp
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision
    os.makedirs(WORK, exist_ok=True)
    if camera:
        im = cv2.imread(os.path.join(os.path.dirname(os.path.dirname(FRAME)), camera, "colour.png"))
        prefix = camera + "_"
    else:
        im = load_crop()
        prefix = ""
    rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    opts = vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=POSE_MODEL),
        running_mode=vision.RunningMode.IMAGE, output_segmentation_masks=True,
        min_pose_detection_confidence=0.3, min_pose_presence_confidence=0.3)
    lm = vision.PoseLandmarker.create_from_options(opts)
    res = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
    if not res.pose_landmarks:
        raise SystemExit("MediaPipe found no body in the frame")
    h, w = im.shape[:2]
    pts = [[p.x * w, p.y * h, p.visibility] for p in res.pose_landmarks[0]]
    mask = res.segmentation_masks[0].numpy_view()
    mask = mask[..., 0] if mask.ndim == 3 else mask
    np.save(os.path.join(WORK, prefix + "body_mask.npy"), mask.astype(np.float32))
    json.dump({"landmarks": pts, "n_poses": len(res.pose_landmarks), "model": os.path.relpath(POSE_MODEL, ROOT),
               "frame_shape": [h, w]},
              open(os.path.join(WORK, prefix + "pose.json"), "w"), indent=1)
    print("pose%s: %d body found, mask covers %.1f%% of the frame" % (" " + camera if camera else "", len(res.pose_landmarks), 100 * (mask > 0.5).mean()))


# --------------------------------------------------------------------------
# stage 2: arms, table, objects
# --------------------------------------------------------------------------
def zhang_suen(mask, max_iter=200):
    img = mask.astype(np.uint8).copy()
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            P = np.pad(img, 1)
            p2 = P[:-2, 1:-1]; p3 = P[:-2, 2:]; p4 = P[1:-1, 2:]; p5 = P[2:, 2:]
            p6 = P[2:, 1:-1]; p7 = P[2:, :-2]; p8 = P[1:-1, :-2]; p9 = P[:-2, :-2]
            nb = [p2, p3, p4, p5, p6, p7, p8, p9]
            B = sum(n.astype(int) for n in nb)
            seq = nb + [p2]
            A = sum(((seq[k] == 0) & (seq[k + 1] == 1)).astype(int) for k in range(8))
            if step == 0:
                c1 = (p2 * p4 * p6) == 0; c2 = (p4 * p6 * p8) == 0
            else:
                c1 = (p2 * p4 * p8) == 0; c2 = (p2 * p6 * p8) == 0
            rem = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & c1 & c2
            if rem.any():
                img[rem] = 0; changed = True
        if not changed:
            break
    return img.astype(bool)


def skeleton_path(sk, start):
    """Longest geodesic from `start` over the 8-connected skeleton: the arm centreline."""
    H, W = sk.shape
    dist = {start: 0.0}; prev = {start: None}; q = collections.deque([start])
    while q:
        x, y = q.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if (dx or dy) and 0 <= nx < W and 0 <= ny < H and sk[ny, nx] and (nx, ny) not in dist:
                    dist[(nx, ny)] = dist[(x, y)] + (1.414 if dx and dy else 1.0)
                    prev[(nx, ny)] = (x, y); q.append((nx, ny))
    tip = max(dist, key=dist.get)
    path = []; c = tip
    while c is not None:
        path.append(c); c = prev[c]
    return np.array(path[::-1], float), tip


def bends(P, arc, win=5, min_deg=BEND_DEG, min_gap_px=30):
    ang = np.zeros(len(P))
    for k in range(win, len(P) - win):
        a = P[k] - P[k - win]; b = P[k + win] - P[k]
        ang[k] = np.degrees(np.arccos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9), -1, 1)))
    cand = [k for k in range(1, len(P) - 1) if ang[k] > min_deg and ang[k] >= ang[max(0, k - 8):k + 9].max()]
    joints = []
    for k in cand:
        if not joints or arc[k] - arc[joints[-1]] > min_gap_px:
            joints.append(k)
    return joints, ang


def stage_detect():
    import cv2
    from ultralytics import YOLO, FastSAM
    im = load_crop()
    H, W = im.shape[:2]
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    body = np.load(os.path.join(WORK, "body_mask.npy")) > 0.5
    pose = json.load(open(os.path.join(WORK, "pose.json")))["landmarks"]

    # -- open-vocabulary detector for the furniture and objects
    yolo = YOLO(os.path.join(ROOT, "yolov8s-worldv2.pt"))
    yolo.set_classes(YOLO_PROMPTS)
    r = yolo.predict(im, conf=YOLO_CONF, imgsz=960, device="cpu", verbose=False)[0]
    dets = [{"class": YOLO_PROMPTS[int(b.cls)], "conf": float(b.conf), "box": [float(v) for v in b.xyxy[0].tolist()]}
            for b in r.boxes]
    # keep the best box per class for the furniture; all boxes for tripods/cameras
    best = {}
    for d in dets:
        if d["class"] in ("white desk", "cardboard box", "green cube", "mannequin"):
            if d["class"] not in best or d["conf"] > best[d["class"]]["conf"]:
                best[d["class"]] = d
    table = best.get("white desk")
    table_top = int(table["box"][1]) if table else int(H * 0.57)

    # -- the project's own colour detector for the cube (cross-check)
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    try:
        from srl_object_detector import PALETTE
    except Exception:
        PALETTE = [("green", (40, 70, 40), (85, 255, 255))]
    colour_blobs = []
    for name, lo, hi in PALETTE:
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] > 150:
                x, y, w, h = stats[i, :4]
                if body[y:y + h, x:x + w].mean() > 0.2:   # the mannequin's skin is red; a blob on the body is not an object
                    continue
                colour_blobs.append({"colour": name, "box": [int(x), int(y), int(x + w), int(y + h)], "area": int(stats[i, cv2.CC_STAT_AREA])})

    # -- FastSAM everything-mode segments (used as pieces for the arms)
    fs = FastSAM(os.path.join(ROOT, "FastSAM-s.pt"))
    rr = fs(im, device="cpu", retina_masks=True, imgsz=1024, conf=0.4, iou=0.9, verbose=False)[0]
    fs_masks = rr.masks.data.numpy() > 0.5 if rr.masks is not None else np.zeros((0, H, W), bool)

    # -- the arms: local-backdrop difference inside the corridor, plus FastSAM pieces
    sh_y = (pose[11][1] + pose[12][1]) / 2
    bx = np.argwhere(body); bxmin, bxmax = int(bx[:, 1].min()), int(bx[:, 1].max()); cx = bx[:, 1].mean()
    corridor = np.zeros((H, W), bool)
    corridor[max(0, int(sh_y - 70)):table_top, :] = True
    corridor[:, max(0, bxmin - 5):bxmax + 5] = False
    med = cv2.medianBlur(g, BACKDROP_MEDIAN_PX)
    diff = np.abs(g.astype(int) - med.astype(int))
    m = (diff > ARM_DIFF_GREY) & corridor & ~body
    m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8)).astype(bool)
    merged = 0
    for mk in fs_masks:
        if mk.sum() < 0.05 * H * W and mk[m].sum() > 0.5 * mk.sum():
            m |= mk; merged += 1
    m &= ~body
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    comps = sorted([i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] > 2500], key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:2]
    arms = {}
    arm_mask = np.zeros((H, W), np.uint8)
    for i in comps:
        am = cv2.morphologyEx((lab == i).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)) > 0
        sk = zhang_suen(am)
        ys, xs = np.nonzero(sk); pts = np.stack([xs, ys], 1)
        side = "left" if pts[:, 0].mean() < cx else "right"      # image left / image right
        start = tuple(int(v) for v in pts[int(np.argmin(np.abs(pts[:, 0] - cx)))])
        path, tip = skeleton_path(sk, start)
        seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        arc = np.arange(0, seg[-1], 4.0)
        P = np.stack([np.interp(arc, seg, path[:, 0]), np.interp(arc, seg, path[:, 1])], 1)
        joints, ang = bends(P, arc)
        arms[side] = {"area_px": int(am.sum()), "centreline": P.tolist(), "arc_px": arc.tolist(),
                      "joints_idx": joints, "bend_deg": [float(ang[k]) for k in joints],
                      "length_px": float(seg[-1]), "start": list(start), "tip": [int(tip[0]), int(tip[1])],
                      "n_links": len(joints) + 1}
        arm_mask[am] = 1 if side == "left" else 2
    np.save(os.path.join(WORK, "arm_mask.npy"), arm_mask)
    json.dump({"crop_xyxy": CROP, "yolo": {"prompts": YOLO_PROMPTS, "conf": YOLO_CONF, "detections": dets, "best": best},
               "colour_blobs": colour_blobs, "fastsam_segments": int(len(fs_masks)), "fastsam_merged_into_arms": merged,
               "arm_rule": {"backdrop_median_px": BACKDROP_MEDIAN_PX, "diff_grey": ARM_DIFF_GREY, "bend_deg": BEND_DEG,
                            "corridor_y": [max(0, int(sh_y - 70)), table_top], "torso_x": [bxmin, bxmax]},
               "arms": arms}, open(os.path.join(WORK, "detect.json"), "w"), indent=1)
    print("yolo:", ", ".join("%s %.2f" % (d["class"], d["conf"]) for d in sorted(dets, key=lambda d: -d["conf"])))
    print("colour blobs:", colour_blobs)
    for s, a in arms.items():
        print("arm %s: area %d px, centreline %.0f px, %d joints (%s), %d links" % (
            s, a["area_px"], a["length_px"], len(a["joints_idx"]), ", ".join("%.0f deg" % b for b in a["bend_deg"]), a["n_links"]))


# --------------------------------------------------------------------------
# stage 3: the figure
# --------------------------------------------------------------------------
SKELETON = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24), (0, 11), (0, 12)]


def stage_compose():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Patch
    from matplotlib.lines import Line2D
    im = plt.imread(FRAME)[CROP[1]:CROP[3], CROP[0]:CROP[2]]
    if im.dtype != np.uint8:
        im = (im * 255).astype(np.uint8)
    im = im[..., :3]
    H, W = im.shape[:2]
    body = np.load(os.path.join(WORK, "body_mask.npy")) > 0.5
    pose = json.load(open(os.path.join(WORK, "pose.json")))["landmarks"]
    det = json.load(open(os.path.join(WORK, "detect.json")))
    arm_mask = np.load(os.path.join(WORK, "arm_mask.npy"))
    BODY, LEFT, RIGHT, TABLE, OBJ, OTHER = "#c1392b", "#2c6fbb", "#e67e22", "#555555", "#3f9142", "#8e44ad"

    def tint(ax, mask, colour, alpha=0.45):
        rgba = np.zeros((H, W, 4)); c = matplotlib.colors.to_rgb(colour)
        rgba[mask] = (*c, alpha); ax.imshow(rgba)

    def draw_body(ax, skeleton=True):
        tint(ax, body, BODY)
        if skeleton:
            for a, b in SKELETON:
                if pose[a][2] > 0.3 and pose[b][2] > 0.3:
                    ax.plot([pose[a][0], pose[b][0]], [pose[a][1], pose[b][1]], color="white", lw=1.8)
            vis = [p for p in pose if p[2] > 0.3]
            ax.scatter([p[0] for p in vis], [p[1] for p in vis], s=12, color="yellow", zorder=5, linewidths=0)

    def draw_arms(ax, label=True):
        tint(ax, arm_mask == 1, LEFT); tint(ax, arm_mask == 2, RIGHT)
        for side, a in det["arms"].items():
            P = np.array(a["centreline"]); J = a["joints_idx"]
            ax.plot(P[:, 0], P[:, 1], color="white", lw=1.6)
            ax.scatter(P[J, 0], P[J, 1], s=60, facecolor="none", edgecolor="red", lw=1.8, zorder=6)
            ax.scatter([a["start"][0]], [a["start"][1]], s=30, color="lime", zorder=6)
            ax.scatter([a["tip"][0]], [a["tip"][1]], s=30, color="magenta", zorder=6)
            if label:
                cuts = [0] + J + [len(P) - 1]
                for k in range(len(cuts) - 1):
                    mid = P[(cuts[k] + cuts[k + 1]) // 2]
                    ax.text(mid[0], mid[1] - 9, "L%d" % (k + 1), color="white", fontsize=7, ha="center",
                            bbox=dict(facecolor=LEFT if side == "left" else RIGHT, edgecolor="none", pad=1))

    def draw_boxes(ax, classes, colour, lw=1.6, best_only=False, min_conf=0.0):
        k = 0
        for d in det["yolo"]["detections"]:
            if d["class"] in classes and d["conf"] >= min_conf and (not best_only or
                                                                    abs(det["yolo"]["best"][d["class"]]["conf"] - d["conf"]) < 1e-9):
                x0, y0, x1, y1 = d["box"]
                ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=colour, lw=lw))
                # label below the box for every other detection, above otherwise, so neighbours do not collide
                if k % 2 == 0:
                    ax.text(x0 + 2, y0 - 3, "%s %.2f" % (d["class"], d["conf"]), color="white", fontsize=6.5, va="bottom",
                            bbox=dict(facecolor=colour, edgecolor="none", pad=1))
                else:
                    ax.text(x0 + 2, y1 + 3, "%s %.2f" % (d["class"], d["conf"]), color="white", fontsize=6.5, va="top",
                            bbox=dict(facecolor=colour, edgecolor="none", pad=1))
                k += 1

    fig, axes = plt.subplots(2, 3, figsize=(11.5, 8.2))
    titles = ["(a) room camera, cropped to the scene",
              "(b) wearer: MediaPipe Pose mask and landmarks",
              "(c) robot arms: silhouette, centreline and its bends",
              "(d) table and objects: YOLO-World",
              "(e) objects: the colour detector, as a cross-check",
              "(f) the scene the safety layer reasons about"]
    for ax, t in zip(axes.flat, titles):
        ax.imshow(im); ax.set_axis_off(); ax.set_title(t, fontsize=8.5, loc="left")
    draw_body(axes[0, 1])
    draw_arms(axes[0, 2])
    draw_boxes(axes[1, 0], ["white desk", "cardboard box", "green cube"], TABLE, best_only=True)
    draw_boxes(axes[1, 0], ["tripod", "camera"], OTHER, lw=1.0, min_conf=0.15)
    for b in det["colour_blobs"]:
        x0, y0, x1, y1 = b["box"]
        axes[1, 1].add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=OBJ, lw=1.8))
        axes[1, 1].text(x0 + 2, y0 - 3, "%s, %d px" % (b["colour"], b["area"]), color="white", fontsize=6.5, va="bottom",
                        bbox=dict(facecolor=OBJ, edgecolor="none", pad=1))
    ax = axes[1, 2]
    draw_body(ax, skeleton=True); draw_arms(ax, label=False)
    best = det["yolo"]["best"]
    for cls, col in (("white desk", TABLE), ("cardboard box", OBJ), ("green cube", OBJ)):
        if cls in best:
            x0, y0, x1, y1 = best[cls]["box"]
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=col, lw=1.8))
    handles = [Patch(facecolor=BODY, alpha=0.6, label="wearer (MediaPipe Pose)"),
               Patch(facecolor=LEFT, alpha=0.6, label="left arm, %d links" % det["arms"].get("left", {}).get("n_links", 0)),
               Patch(facecolor=RIGHT, alpha=0.6, label="right arm, %d links" % det["arms"].get("right", {}).get("n_links", 0)),
               Line2D([], [], marker="o", color="red", markerfacecolor="none", ls="", label="joint (bend > %.0f deg)" % BEND_DEG),
               Patch(fill=False, edgecolor=TABLE, label="table (YOLO-World)"),
               Patch(fill=False, edgecolor=OBJ, label="objects (YOLO-World, colour)")]
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=2, fontsize=6.5, frameon=False, columnspacing=1.0, handlelength=1.4)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.96, bottom=0.09, wspace=0.03, hspace=0.10)
    fig.savefig(OUT + ".pdf"); fig.savefig(OUT + "_300.png", dpi=300); fig.savefig(OUT + ".png", dpi=120)
    # a one-line summary for the caption, from the record
    # checks: the two object detectors agree; each arm starts at a shoulder; arms do not overlap the body
    def iou(a, b):
        ix0, iy0, ix1, iy1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
        inter = max(0, ix1 - ix0) * max(0, iy1 - iy0)
        return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)
    checks = {}
    if "green cube" in best and det["colour_blobs"]:
        checks["cube_iou_yolo_vs_colour"] = round(iou(best["green cube"]["box"], det["colour_blobs"][0]["box"]), 2)
    sh = {"left": pose[12][:2] if pose[12][0] < pose[11][0] else pose[11][:2], "right": pose[11][:2] if pose[12][0] < pose[11][0] else pose[12][:2]}
    for side, a in det["arms"].items():
        checks["%s_arm_start_to_shoulder_px" % side] = round(float(np.hypot(a["start"][0] - sh[side][0], a["start"][1] - sh[side][1])), 1)
    checks["arm_pixels_inside_body_mask"] = int(((arm_mask > 0) & body).sum())
    summ = {"checks": checks, "body_mask_frac": float(body.mean()), "yolo_best": {k: round(v["conf"], 2) for k, v in best.items()},
            "arms": {s: {"links": a["n_links"], "length_px": round(a["length_px"]), "bends_deg": [round(b) for b in a["bend_deg"]]} for s, a in det["arms"].items()},
            "colour_blobs": det["colour_blobs"]}
    json.dump(summ, open(os.path.join(WORK, "summary.json"), "w"), indent=1)
    print(json.dumps(summ))
    print("wrote", OUT + ".pdf")


if __name__ == "__main__":
    stage = sys.argv[1] if len(sys.argv) > 1 else "compose"
    if stage == "pose":
        stage_pose(sys.argv[2] if len(sys.argv) > 2 else None)
    else:
        {"detect": stage_detect, "compose": stage_compose}[stage]()
