#!/usr/bin/env python3
"""Plain-language gallery from the real-camera run of 2026-08-30.

    python3 extras/thesis/thesis_v3/figures/gallery/vision/make_vision_gallery.py

Everything is drawn from recordings/vision_thesis/20260830_073141 (raw
frames, depth arrays, stage logs, the arm-calibration frames) and
recordings/baselines/arm_directional_calibration.json. Nothing is synthesised:
where a camera has no depth sensor the panel says so; the plane-fit mask is
recomputed from the plane the pipeline stored (normal, offset, 6 mm tolerance)
over the same depth array it fitted, and its inlier fraction is checked
against the stored one.
"""

import csv
import json
import os
import sys

import cv2
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle, Polygon, Patch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, *[".."] * 6))
RUN = os.path.join(ROOT, "recordings", "vision_thesis", "20260830_073141")
CAL = os.path.join(ROOT, "recordings", "calibration_frames")
CALJ = os.path.join(ROOT, "recordings", "baselines", "arm_directional_calibration.json")
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from srl_object_detector import blobs  # noqa: E402

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
AMBER = "#c98a1a"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})

CAMS = ["left_gripper", "right_gripper", "scene_rs", "scene_hd"]
NAMES = {
    "left_gripper": "left wrist camera",
    "right_gripper": "right wrist camera",
    "scene_rs": "room depth camera",
    "scene_hd": "room webcam",
}
MANIFEST = json.load(open(os.path.join(RUN, "manifest.json")))
SRC = {s["name"]: s for s in MANIFEST["sources"]}


def save(fig, name):
    fig.savefig(os.path.join(HERE, name + ".pdf"))
    fig.savefig(os.path.join(HERE, name + ".png"), dpi=150)
    plt.close(fig)
    print("wrote", name)


def rgb(cam):
    return cv2.cvtColor(cv2.imread(os.path.join(RUN, "raw", cam, "colour.png")), cv2.COLOR_BGR2RGB)


def depth(cam):
    p = os.path.join(RUN, "raw", cam, "depth.npy")
    return np.load(p) if os.path.exists(p) else None


def stage(cam, name):
    p = os.path.join(RUN, "data", cam, name + ".json")
    if not os.path.exists(p):
        return None
    d = json.load(open(p))
    d["m"] = {x["quantity"]: x["value"] for x in d.get("measurements", [])}
    return d


def band(cam):
    m = stage(cam, "15_raw_depth")["m"]
    return m["band near"], m["band far"]


# ---------------------------------------------------------------- 1. cameras
def four_cameras():
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 4.3))
    for ax, cam in zip(axes.ravel(), CAMS):
        ax.imshow(rgb(cam))
        ax.set_title(NAMES[cam])
        ax.axis("off")
    fig.subplots_adjust(left=0.01, right=0.99, top=0.93, bottom=0.01, wspace=0.03, hspace=0.15)
    save(fig, "four_cameras")


def colour_and_depth():
    fig, axes = plt.subplots(4, 2, figsize=(7.2, 8.4))
    for r, cam in enumerate(CAMS):
        axes[r, 0].imshow(rgb(cam))
        axes[r, 0].set_title(NAMES[cam] + " -- picture")
        d = depth(cam)
        if d is None:
            axes[r, 1].text(0.5, 0.5, "no depth sensor\n(this camera only takes pictures)",
                            ha="center", va="center", color=GREY, fontsize=9,
                            transform=axes[r, 1].transAxes)
            axes[r, 1].set_facecolor("0.94")
            axes[r, 1].set_title(NAMES[cam] + " -- depth")
        else:
            lo, hi = band(cam)
            shown = np.ma.masked_where(d <= 0, np.clip(d * 1000, 0, hi * 1000))
            im = axes[r, 1].imshow(shown, cmap="viridis", vmin=lo * 1000, vmax=hi * 1000)
            axes[r, 1].set_title(NAMES[cam] + " -- depth")
            cb = fig.colorbar(im, ax=axes[r, 1], fraction=0.035, pad=0.02)
            cb.set_label("distance from camera (mm)", fontsize=7)
            cb.ax.tick_params(labelsize=6)
        for ax in axes[r]:
            ax.set_xticks([]); ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
    fig.text(0.5, 0.005, "white = no depth reading at that pixel", ha="center", fontsize=7, color=GREY)
    fig.subplots_adjust(left=0.01, right=0.97, top=0.96, bottom=0.03, wspace=0.05, hspace=0.25)
    save(fig, "colour_and_depth")


# ------------------------------------------------------------ 2. plane fit
def cloud(cam):
    """Depth pixels inside the working band, lifted to metres, plus their (v,u)."""
    d = depth(cam)
    fx, fy, cx, cy = SRC[cam]["K_depth"]
    lo, hi = band(cam)
    v, u = np.nonzero((d > lo) & (d < hi))
    z = d[v, u]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], 1), v, u


def plane(cam):
    m = stage(cam, "18_ransac")["m"]
    n = np.array([m["normal x"], m["normal y"], m["normal z"]])
    return n, m["offset d"], m["inlier tolerance"] / 1000.0, m["inlier fraction"], m["residual RMS"]


def plane_fit_per_camera():
    cams = ["left_gripper", "right_gripper", "scene_rs"]
    ratios = [depth(c).shape[1] / depth(c).shape[0] for c in cams]
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.5), gridspec_kw={"width_ratios": ratios})
    check = {}
    for ax, cam in zip(axes, cams):
        d = depth(cam)
        P, v, u = cloud(cam)
        n, off, tol, frac_stored, rms = plane(cam)
        h = P @ n + off
        inl = np.abs(h) < tol
        frac = 100.0 * inl.mean()
        check[cam] = (frac, frac_stored)
        canvas = np.full(d.shape + (3,), 0.92)
        base = np.clip(d / max(band(cam)[1], 1e-6), 0, 1)
        canvas[...] = np.stack([0.55 + 0.35 * base] * 3, -1)
        canvas[d <= 0] = 1.0
        canvas[v[inl], u[inl]] = matplotlib.colors.to_rgb(GREEN)
        canvas[v[~inl], u[~inl]] = matplotlib.colors.to_rgb(RED)
        ax.imshow(canvas)
        ax.set_title("%s\n%.0f%% on one flat surface" % (NAMES[cam], frac), fontsize=8.5)
        ax.axis("off")
    fig.legend(handles=[Patch(color=GREEN, label="on the fitted table plane (within 6 mm)"),
                        Patch(color=RED, label="in range but not on the plane"),
                        Patch(color="0.75", label="outside the camera's working range"),
                        Patch(facecolor="white", edgecolor="0.6", label="no depth reading")],
               loc="lower center", ncol=2, frameon=False, fontsize=7, bbox_to_anchor=(0.5, -0.02))
    fig.subplots_adjust(left=0.01, right=0.99, top=0.84, bottom=0.2, wspace=0.04)
    save(fig, "plane_fit_per_camera")
    return check


# -------------------------------------------------------- 3. depth coverage
def depth_coverage():
    cams = ["left_gripper", "right_gripper", "scene_rs"]
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.2, 2.8), gridspec_kw={"width_ratios": [1, 1.4]})
    ret, inband = [], []
    for cam in cams:
        m = stage(cam, "15_raw_depth")["m"]
        total = m["depth width"] * m["depth height"]
        ret.append(100 * m["pixels with a return"] / total)
        inband.append(100 * m["returns inside this camera's working band"] / total)
    x = np.arange(len(cams))
    ax0.bar(x - 0.18, ret, 0.36, color=BLUE, label="has any depth reading")
    ax0.bar(x + 0.18, inband, 0.36, color=GREEN, label="reading is within working range")
    for xi, a, b in zip(x, ret, inband):
        ax0.text(xi - 0.18, a + 1, "%.0f" % a, ha="center", fontsize=7)
        ax0.text(xi + 0.18, b + 1, "%.0f" % b, ha="center", fontsize=7)
    ax0.set_xticks(x)
    ax0.set_xticklabels([NAMES[c].replace(" ", "\n", 1) for c in cams], fontsize=7.5)
    ax0.set_ylabel("share of the picture (%)")
    ax0.set_ylim(0, 105)
    ax0.legend(frameon=False, fontsize=7, loc="upper right")
    cols = {"left_gripper": BLUE, "right_gripper": RED, "scene_rs": GREY}
    for cam in cams:
        d = depth(cam)
        vals = d[d > 0]
        vals = vals[vals < 10]
        ax1.hist(vals, bins=80, range=(0, 10), histtype="step", lw=1.3, color=cols[cam],
                 density=True, label=NAMES[cam])
        lo, hi = band(cam)
        ax1.axvspan(lo, hi, color=cols[cam], alpha=0.06)
    ax1.set_xlabel("distance from camera (m)")
    ax1.set_ylabel("share of readings")
    ax1.set_yticks([])
    ax1.legend(frameon=False, fontsize=7, loc="upper right")
    ax1.text(0.98, 0.62, "shaded band =\neach camera's\nworking range", transform=ax1.transAxes,
             ha="right", va="top", fontsize=6.5, color=GREY)
    fig.subplots_adjust(left=0.09, right=0.99, top=0.95, bottom=0.2, wspace=0.25)
    save(fig, "depth_coverage")


# ------------------------------------------------------- 4. pipeline status
PLAIN_STAGE = {
    "raw_colour": "the picture", "intrinsics": "lens numbers", "hsv": "colour as hue",
    "green_threshold": "find green pixels", "morphology": "clean up speckle",
    "components": "group into blobs", "vocabulary": "which 'green' definition",
    "region_mean": "average colour per blob", "minarearect": "tilted box round blob",
    "size_at_range": "size implies distance", "fastsam": "AI region splitter",
    "area_filter": "drop tiny regions", "segment_match": "region that matches 'cube'",
    "yoloworld": "AI object finder", "raw_depth": "the depth map",
    "alignment": "line depth up with picture", "deprojection": "pixels to 3-D points",
    "ransac": "find the flat table", "pca_refine": "tidy the table fit",
    "height_map": "height above table", "mode_surface": "most common height",
    "on_surface": "what stands on the table", "cube_measurement": "measure the cube",
    "layers": "slice by height", "min_width": "narrowest width", "nested": "boxes inside boxes",
    "grasp": "can the hand grasp it", "apriltag": "printed marker", "table": "table as an object",
    "boxes3d": "3-D box per object", "distances": "distances between objects",
    "people": "is a person in view", "self_view": "is the robot in its own view",
}


def pipeline_status():
    stages = SRC["left_gripper"]["stages"]
    titles = [PLAIN_STAGE.get(s["slug"], s["slug"].replace("_", " ")) for s in stages]
    state_colour = {"ok": GREEN, "refused": AMBER, "by_design": "0.80"}
    fig, ax = plt.subplots(figsize=(7.2, 2.9))
    for r, cam in enumerate(CAMS):
        for c, st in enumerate(SRC[cam]["stages"]):
            ax.add_patch(Rectangle((c, len(CAMS) - 1 - r), 0.92, 0.85,
                                   color=state_colour.get(st["status"], "0.5")))
    ax.set_xlim(0, len(stages))
    ax.set_ylim(0, len(CAMS))
    ax.set_yticks(np.arange(len(CAMS)) + 0.45)
    ax.set_yticklabels([NAMES[c] for c in reversed(CAMS)], fontsize=8)
    ax.set_xticks(np.arange(len(stages)) + 0.46)
    ax.set_xticklabels(titles, rotation=60, ha="right", fontsize=5.8)
    ax.set_xlabel("the 33 steps of the picture-to-grasp pipeline, in order")
    for s in ax.spines.values():
        s.set_visible(False)
    ax.tick_params(length=0)
    ax.legend(handles=[Patch(color=GREEN, label="gave a result"),
                       Patch(color=AMBER, label="declined, and said why"),
                       Patch(color="0.80", label="not applicable to this camera")],
              loc="upper center", bbox_to_anchor=(0.5, 1.22), ncol=3, frameon=False, fontsize=7.5)
    fig.set_size_inches(7.2, 3.4)
    fig.subplots_adjust(left=0.19, right=0.99, top=0.88, bottom=0.46)
    save(fig, "pipeline_status")
    return {cam: sum(1 for s in SRC[cam]["stages"] if s["status"] == "ok") for cam in CAMS}


# ---------------------------------------------------------- 5. plan view
def boxes(cam):
    rows = list(csv.DictReader(open(os.path.join(
        RUN, "data", cam, "30_boxes3d__every_object__in_the_camera_frame.csv"))))
    return rows


def plan_view_left():
    cam = "left_gripper"
    P, v, u = cloud(cam)
    n, off, tol, _, _ = plane(cam)
    h = (P @ n + off) * 1000
    keep = h > -20
    fig, ax = plt.subplots(figsize=(5.2, 4.2))
    order = np.argsort(h[keep])
    sc = ax.scatter(P[keep][order, 0], P[keep][order, 2], c=np.clip(h[keep][order], 0, 90),
                    s=1.2, cmap="viridis", rasterized=True)
    cb = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
    cb.set_label("height above the table (mm)", fontsize=8)
    cb.ax.tick_params(labelsize=7)
    for b in boxes(cam):
        x, z = float(b["x m"]), float(b["z m"])
        L, W = float(b["length mm"]) / 1000, float(b["width mm"]) / 1000
        yaw = np.deg2rad(float(b["yaw deg"]))
        c, s = np.cos(yaw), np.sin(yaw)
        corners = np.array([[-L / 2, -W / 2], [L / 2, -W / 2], [L / 2, W / 2], [-L / 2, W / 2]])
        rot = corners @ np.array([[c, -s], [s, c]]).T
        colour = GREEN if b["graspable"] == "yes" else RED
        ax.add_patch(Polygon(rot + [x, z], closed=True, fill=False, edgecolor=colour, lw=1.4))
    ax.set_xlabel("left  <--  across the view (m)  -->  right")
    ax.set_ylabel("distance from camera (m)")
    ax.set_aspect("equal")
    ax.legend(handles=[Patch(fill=False, edgecolor=GREEN, label="object the 85 mm jaw could grasp"),
                       Patch(fill=False, edgecolor=RED, label="object too wide for the jaw")],
              frameon=False, fontsize=7.5, loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=1)
    fig.subplots_adjust(left=0.13, right=0.97, top=0.98, bottom=0.27)
    save(fig, "plan_view_left")


# ---------------------------------------------- 6. right gripper pipeline
BOX_HSV = ((0, 90, 50), (25, 255, 200))


def box_rect(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, *BOX_HSV)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    idx = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    pts = np.column_stack(np.nonzero(lab == idx))[:, ::-1].astype(np.float32)
    return cv2.boxPoints(cv2.minAreaRect(pts)), (lab == idx)


def cube_rect(bgr):
    # a cube is roughly square in the picture; the teal robot base in the
    # right camera's view is also 'green' to the palette but is not square
    cands = [d for d in blobs(bgr) if d["colour"] == "green" and 0.6 <= d["aspect"] <= 1.7]
    b = sorted(cands, key=lambda d: -d["area_px"])[0]
    mask = b["_lab"] == b["mask_i"]
    pts = np.column_stack(np.nonzero(mask))[:, ::-1].astype(np.float32)
    return cv2.boxPoints(cv2.minAreaRect(pts)), mask


def pipeline_row(axes, cam):
    bgr = cv2.imread(os.path.join(RUN, "raw", cam, "colour.png"))
    im = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    cube_pts, cube_m = cube_rect(bgr)
    box_pts, box_m = box_rect(bgr)
    axes[0].imshow(im)
    seg = im.copy().astype(float)
    seg[cube_m] = 0.5 * seg[cube_m] + 0.5 * np.array(matplotlib.colors.to_rgb(GREEN)) * 255
    seg[box_m] = 0.5 * seg[box_m] + 0.5 * np.array(matplotlib.colors.to_rgb(BLUE)) * 255
    axes[1].imshow(seg.astype(np.uint8))
    axes[2].imshow(im)
    axes[2].add_patch(Polygon(cube_pts, closed=True, fill=False, edgecolor=GREEN, lw=1.6))
    axes[2].add_patch(Polygon(box_pts, closed=True, fill=False, edgecolor=BLUE, lw=1.6))
    for ax in axes:
        ax.axis("off")


def left_vs_right_pipeline():
    fig, axes = plt.subplots(2, 3, figsize=(7.2, 3.1))
    pipeline_row(axes[0], "left_gripper")
    pipeline_row(axes[1], "right_gripper")
    for ax, t in zip(axes[0], ["picture", "cube and box picked out", "boxes drawn round them"]):
        ax.set_title(t)
    axes[0, 0].text(-0.02, 0.5, NAMES["left_gripper"], transform=axes[0, 0].transAxes,
                    rotation=90, va="center", ha="right", fontsize=8)
    axes[1, 0].text(-0.02, 0.5, NAMES["right_gripper"], transform=axes[1, 0].transAxes,
                    rotation=90, va="center", ha="right", fontsize=8)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.92, bottom=0.01, wspace=0.03, hspace=0.06)
    save(fig, "left_vs_right_pipeline")


# ----------------------------------------------- 7. arm calibration frames
def calibration():
    runs = json.load(open(CALJ))["runs"]
    dirs = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    plain = {"+X": "forward", "-X": "back", "+Y": "left", "-Y": "right", "+Z": "up", "-Z": "down"}
    speeds = sorted({r["vmax_rad_s"] for r in runs})
    # frames: one arm, the middle speed, six directions
    fig, axes = plt.subplots(2, 6, figsize=(7.2, 2.5))
    for r, arm in enumerate(["left", "right"]):
        for c, d in enumerate(dirs):
            run = next(x for x in runs if x["arm"] == arm and x["direction"] == d and x["vmax_rad_s"] == speeds[1])
            img = cv2.imread(os.path.join(ROOT, run["frame"]))
            axes[r, c].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            axes[r, c].axis("off")
            if r == 0:
                axes[r, c].set_title("moved " + plain[d], fontsize=8)
        axes[r, 0].text(-0.05, 0.5, arm + " arm", transform=axes[r, 0].transAxes,
                        rotation=90, va="center", ha="right", fontsize=8)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.88, bottom=0.01, wspace=0.03, hspace=0.05)
    save(fig, "calibration_frames")

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(7.2, 2.7))
    x = np.arange(len(dirs))
    for k, (arm, colour) in enumerate([("left", BLUE), ("right", RED)]):
        for j, sp in enumerate(speeds):
            vals = [next(r["ss_cart_mm"] for r in runs if r["arm"] == arm and r["direction"] == d and r["vmax_rad_s"] == sp)
                    for d in dirs]
            ax0.scatter(x + (k - 0.5) * 0.25, vals, color=colour, s=14 + 10 * j, alpha=0.85,
                        label=("%s arm" % arm) if j == 0 else None, zorder=3)
    ax0.set_xticks(x)
    ax0.set_xticklabels([plain[d] for d in dirs])
    ax0.set_ylabel("stopped short of target by (mm)")
    ax0.set_ylim(0, 10)
    ax0.legend(frameon=False, fontsize=7.5)
    ax0.text(0.02, 0.04, "dot size = commanded speed (slow, medium, fast)", transform=ax0.transAxes,
             fontsize=6.5, color=GREY)
    worst = [r["ss_joint_worst_deg"] for r in runs]
    ax1.hist(worst, bins=np.arange(0.288, 0.2985, 0.001), color=GREY)
    ax1.set_xlabel("largest joint error at rest (degrees)")
    ax1.set_ylabel("runs (of 36)")
    ax1.set_xticks([0.288, 0.290, 0.292, 0.294, 0.296, 0.298])
    ax1.set_xticklabels(["0.288", "0.290", "0.292", "0.294", "0.296", "0.298"], fontsize=7.5)
    fig.subplots_adjust(left=0.09, right=0.99, top=0.95, bottom=0.17, wspace=0.3)
    save(fig, "arm_park_error")
    return runs


if __name__ == "__main__":
    four_cameras()
    colour_and_depth()
    chk = plane_fit_per_camera()
    print("plane-fit inlier check (recomputed vs stored):", chk)
    depth_coverage()
    ok = pipeline_status()
    print("stages ok:", ok)
    plan_view_left()
    left_vs_right_pipeline()
    calibration()
