#!/usr/bin/env python3
"""What each of the four real cameras sees and what the software extracts
from it: one column per camera, three rows -- colour, depth, and the
result of the stage that camera feeds -- from the frames of the 2026-08-30
session (recordings/vision_thesis/20260830_073141/raw/<camera>/).

  wrist cameras   the table plane fitted by RANSAC on the depth points
                  (inliers green) and the two task objects boxed from the
                  colour image by the project's own colour-and-geometry
                  detector, numbers from the pipeline's own stage records;
  room depth cam  MediaPipe Pose on its colour stream, the wearer tracker
                  the clearance floor consumes (mask and landmarks);
  room HD camera  the full scene understanding of
                  make_scene_understanding.py (wearer, arms with links,
                  table and objects).

Run make_scene_understanding.py (pose, pose scene_rs, detect) first; then

    python3 thesis_v3/figures/make_camera_roles.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Patch, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
RAW = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141", "raw")
WORK = os.path.join(HERE, "vision", "scene_understanding")
OUT = os.path.join(HERE, "vision", "camera_roles")
sys.path.insert(0, os.path.join(HERE, "gallery", "vision"))
import make_vision_gallery as G  # noqa: E402  (loaders: depth, cloud, plane, band, box_rect, cube_rect, boxes)

CAMS = [
    ("left_gripper", "Left wrist camera", "RGB-D (Kinova vision module)",
     "on the left gripper: table plane\nand objects for the grasp"),
    ("right_gripper", "Right wrist camera", "RGB-D (Kinova vision module)",
     "on the right gripper: the same\nfor the right arm"),
    ("scene_rs", "Room depth camera", "RGB-D (RealSense D435i)",
     "across the room: tracks the wearer\nfor the clearance floor"),
    ("scene_hd", "Room HD camera", "RGB only (webcam)",
     "across the room: the whole scene,\nwearer, arms, table, objects"),
]
GREEN, RED, BLUE, ORANGE, BODY = "#3f9142", "#c1392b", "#2c6fbb", "#e67e22", "#c1392b"
SKELETON = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24), (0, 11), (0, 12)]


def to169(a):
    h, w = a.shape[:2]
    th = int(round(w * 9 / 16))
    if th < h:
        o = (h - th) // 2
        return a[o:o + th], o
    return a, 0


def wrist_result(ax, cam):
    """On the colour image: depth points projected through the pipeline's own
    alignment (depth point + module baseline, then the colour intrinsics),
    green on the fitted plane and red off it, plus the two objects' rotated
    boxes from the colour-and-geometry detector."""
    bgr = G.cv2.imread(os.path.join(RAW, cam, "colour.png"))
    im = G.cv2.cvtColor(bgr, G.cv2.COLOR_BGR2RGB).astype(float)
    H, W = im.shape[:2]
    P, v, u = G.cloud(cam)
    n, off, tol, frac_stored, rms = G.plane(cam)
    inl = np.abs(P @ n + off) < tol
    al = {m["quantity"]: m["value"] for m in json.load(open(os.path.join(RAW, "..", "data", cam, "16_alignment.json")))["measurements"]}
    t = np.array([al["baseline x"], al["baseline y"], al["baseline z"]]) / 1000.0
    fx, fy, cx, cy = G.SRC[cam]["K_colour"]
    Q = P + t
    uc = np.round(fx * Q[:, 0] / Q[:, 2] + cx).astype(int); vc = np.round(fy * Q[:, 1] / Q[:, 2] + cy).astype(int)
    ok = (uc >= 0) & (uc < W) & (vc >= 0) & (vc < H)
    for sel, col in ((inl, GREEN), (~inl, RED)):
        m = np.zeros((H, W), np.uint8)
        m[vc[sel & ok], uc[sel & ok]] = 1
        m = G.cv2.dilate(m, np.ones((5, 5), np.uint8)) > 0
        im[m] = 0.45 * im[m] + 0.55 * np.array(matplotlib.colors.to_rgb(col)) * 255
    ax.imshow(im.astype(np.uint8))
    cube_pts, _ = G.cube_rect(bgr)
    box_pts, _ = G.box_rect(bgr)
    for pts, col in ((cube_pts, GREEN), (box_pts, BLUE)):
        ax.add_patch(Polygon(np.array(pts, float), closed=True, fill=False, edgecolor=col, lw=1.8))
    rows = G.boxes(cam)
    txt = "plane: %.0f%% of depth points, RMS %.1f mm\n%d objects, %d fit the gripper" % (
        100 * inl.mean(), rms, len(rows), sum(1 for r in rows if r["graspable"].strip().lower() == "yes"))
    ax.text(0.02, 0.04, txt, transform=ax.transAxes, fontsize=7.5, color="white",
            bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=2))


def draw_pose(ax, prefix, y_off=0, colour=BODY, alpha=0.45):
    mask = np.load(os.path.join(WORK, prefix + "body_mask.npy")) > 0.5
    pose = json.load(open(os.path.join(WORK, prefix + "pose.json")))["landmarks"]
    full_frac = mask.mean()
    if y_off:
        mask = mask[y_off:y_off + int(round(mask.shape[1] * 9 / 16))]
    rgba = np.zeros(mask.shape + (4,)); c = matplotlib.colors.to_rgb(colour)
    rgba[mask] = (*c, alpha)
    ax.imshow(rgba)
    for a, b in SKELETON:
        if pose[a][2] > 0.3 and pose[b][2] > 0.3:
            ax.plot([pose[a][0], pose[b][0]], [pose[a][1] - y_off, pose[b][1] - y_off], color="white", lw=1.4)
    vis = [p for p in pose if p[2] > 0.3]
    ax.scatter([p[0] for p in vis], [p[1] - y_off for p in vis], s=8, color="yellow", zorder=5, linewidths=0)
    return full_frac


def scene_result(ax):
    """The room camera: the composite of make_scene_understanding, drawn on the full 16:9 frame."""
    det = json.load(open(os.path.join(WORK, "detect.json")))
    x0, y0, x1, y1 = det["crop_xyxy"]
    full = plt.imread(os.path.join(RAW, "scene_hd", "colour.png"))
    if full.dtype != np.uint8:
        full = (full * 255).astype(np.uint8)
    ax.imshow(full[..., :3])
    body = np.load(os.path.join(WORK, "body_mask.npy")) > 0.5
    arm = np.load(os.path.join(WORK, "arm_mask.npy"))
    H, W = full.shape[:2]
    rgba = np.zeros((H, W, 4))
    for m, col in ((body, BODY), (arm == 1, BLUE), (arm == 2, ORANGE)):
        c = matplotlib.colors.to_rgb(col)
        sub = np.zeros((y1 - y0, x1 - x0), bool); sub[:m.shape[0], :m.shape[1]] = m
        rgba[y0:y1, x0:x1][sub] = (*c, 0.5)
    ax.imshow(rgba)
    for side, a in det["arms"].items():
        P = np.array(a["centreline"]) + [x0, y0]; J = a["joints_idx"]
        ax.plot(P[:, 0], P[:, 1], color="white", lw=1.2)
        ax.scatter(P[J, 0], P[J, 1], s=30, facecolor="none", edgecolor="red", lw=1.4, zorder=6)
    for cls, col in (("white desk", "0.3"), ("cardboard box", GREEN), ("green cube", GREEN)):
        b = det["yolo"]["best"].get(cls)
        if b:
            bx0, by0, bx1, by1 = b["box"]
            ax.add_patch(Rectangle((bx0 + x0, by0 + y0), bx1 - bx0, by1 - by0, fill=False, edgecolor=col, lw=1.4))
    n_links = sum(a["n_links"] for a in det["arms"].values())
    n_joints = sum(len(a["joints_idx"]) for a in det["arms"].values())
    n_obj = sum(1 for k in ("cardboard box", "green cube") if k in det["yolo"]["best"])
    ax.text(0.02, 0.04, "wearer; 2 arms, %d links, %d joints;\ntable; %d objects" % (n_links, n_joints, n_obj),
            transform=ax.transAxes, fontsize=7.5, color="white", bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=2))


def main():
    fig, axes = plt.subplots(3, 4, figsize=(12.5, 8.8))
    for j, (cam, title, kind, role) in enumerate(CAMS):
        d = os.path.join(RAW, cam)
        rgb, yoff = to169(plt.imread(os.path.join(d, "colour.png")))
        ax = axes[0, j]
        ax.imshow(rgb); ax.set_title("%s\n%s" % (title, kind), fontsize=10); ax.set_axis_off()
        ax = axes[1, j]
        dp = os.path.join(d, "depth.npy")
        if os.path.exists(dp):
            depth, _ = to169(np.load(dp).astype(float))
            valid = depth > 0
            shown = np.where(valid, depth, np.nan)
            vmax = np.percentile(depth[valid], 97) if valid.any() else 1.0
            im = ax.imshow(shown, cmap="viridis", vmin=0.0, vmax=vmax)
            ax.text(0.02, 0.04, "%.0f%% of pixels return a depth" % (100 * valid.mean()),
                    transform=ax.transAxes, fontsize=8, color="white",
                    bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=2))
            cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
            cb.set_label("distance (m)", fontsize=8); cb.ax.tick_params(labelsize=7)
            ax.set_axis_off()
        else:
            ax.text(0.5, 0.5, "no depth sensor", ha="center", va="center", fontsize=11, color="0.35")
            ax.set_facecolor("0.93"); ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
            ax.set_aspect(9 / 16)
        ax = axes[2, j]
        if cam in ("left_gripper", "right_gripper"):
            wrist_result(ax, cam)
        elif cam == "scene_rs":
            ax.imshow(rgb)
            frac = draw_pose(ax, "scene_rs_", y_off=yoff)
            ax.text(0.02, 0.04, "wearer found: body mask %.1f%% of frame,\n33 landmarks" % (100 * frac),
                    transform=ax.transAxes, fontsize=7.5, color="white",
                    bbox=dict(facecolor="black", alpha=0.55, edgecolor="none", pad=2))
        else:
            scene_result(ax)
        ax.set_axis_off()
        fig.text(0.05 + (0.94 / 4) * (j + 0.5) - 0.012, 0.095, role, ha="center", va="top", fontsize=8.5, color="0.25")
    for r, lab in enumerate(("colour", "depth", "what is\nextracted")):
        axes[r, 0].text(-0.06, 0.5, lab, transform=axes[r, 0].transAxes, rotation=90, va="center", ha="right", fontsize=11)
    fig.legend(handles=[Patch(color=GREEN, label="on the fitted table plane"), Patch(color=RED, label="in range, not on the plane"),
                        Patch(fill=False, edgecolor=GREEN, label="cube (colour detector)"), Patch(fill=False, edgecolor=BLUE, label="box (colour detector)"),
                        Patch(facecolor=BODY, alpha=0.5, label="wearer (MediaPipe Pose)"),
                        Patch(facecolor=BLUE, alpha=0.5, label="left arm"), Patch(facecolor=ORANGE, alpha=0.5, label="right arm")],
               loc="lower center", ncol=4, frameon=False, fontsize=7.5, bbox_to_anchor=(0.5, 0.005))
    fig.subplots_adjust(left=0.05, right=0.99, top=0.93, bottom=0.14, wspace=0.12, hspace=0.06)
    fig.savefig(OUT + ".pdf"); fig.savefig(OUT + "_300.png", dpi=300); fig.savefig(OUT + ".png", dpi=130)
    print("wrote", OUT + ".pdf")


if __name__ == "__main__":
    main()
