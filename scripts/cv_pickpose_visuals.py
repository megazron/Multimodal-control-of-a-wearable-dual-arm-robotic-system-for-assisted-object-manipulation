#!/usr/bin/env python3
"""EVERY VISION STAGE, AS A PICTURE, WITH THE ARMS PARKED AT THE PICK POSE.

    .venv_vision/bin/python scripts/cv_pickpose_visuals.py

One terminal, one program, nothing else.  It is deliberately NOT wired into
the GUI, the launch files or any stack: it opens the cameras, runs every
computer-vision step this repository developed, and writes one labelled figure
per step into a folder stamped with the date and time.

IT NEVER COMMANDS MOTION.  There is no publisher, no service client, no Kortex
session and no trajectory anywhere in this file -- the only ROS traffic it can
generate is a SUBSCRIPTION.  The arms are expected to be sitting at the pick
pose (the operator drives them there; this program only records where they
actually are and how far that is from `config/pick_pose_ideal_*.txt`).  That
check is the point: a figure captioned "at the pick pose" is worth nothing if
nobody measured the pose, and the arms drift when a bridge is restarted.

WHAT IT LOOKS AT
==========================================================================
    left_gripper    Kinova vision module on the left arm   colour + depth
    right_gripper   Kinova vision module on the right arm  colour + depth
    scene_rs        RealSense D435i across the room        colour + depth
    scene_hd        HD USB webcam across the room          colour only

Gripper frames are read from the ROS topics `bringup_arm.sh` already
publishes, because the driver holds the RTSP connections and Kinova allows
only two per stream; if no stack is up it falls back to RTSP directly (colour
over ffmpeg, depth over GStreamer -- ffmpeg cannot read the depth stream, see
`srl_cameras`).  A camera that is missing produces a labelled REFUSAL tile
with the command that fixes it, never a silently absent panel.

WHAT IT DRAWS, AND WHY ONE FIGURE PER STEP
==========================================================================
"The vision works" is not a claim a thesis can print.  Each stage below is one
operation with one formula, drawn on the frame it was given, with the numbers
it produced burned into the figure:

  COLOUR   raw frame and provenance; the intrinsics and the ray formula;
           the HSV decomposition; the pick path's own green threshold; what
           the 9x9 opening actually removes; connected components; the whole
           colour vocabulary swept side by side (three different definitions
           of "green" live in this repository and the figure shows all
           three); the region-MEAN ratio test and its near-black guard;
           minAreaRect yaw with the degeneracy gate; the size-at-range gate
           that stops a 210 mm pad being called a 40 mm cube; FastSAM's
           regions; the area filter; the segment-and-match backend; and
           YOLO-World's open-vocabulary boxes.

  DEPTH    the raw depth and where it has no return; depth aligned into the
           colour frame; deprojection to a cloud; the RANSAC plane and its
           PCA refinement (both halves -- refining the normal and keeping the
           old offset is the bug that produced "plane RMS nan mm"); height
           above the plane; the mode-histogram surface estimator; what stands
           on the surface; the cube measurement the pick actually uses, with
           its centre from the foot point and half its height; the layer
           split that separates a cube from the pad it rests on; rotating
           calipers against the PCA and axis-aligned controls that both read
           a 40 mm cube too wide; nested-mask suppression; and the parallel
           jaw grasp.

EVERY FIGURE CARRIES ITS OWN PROVENANCE.  Source, resolution, intrinsics and
where they came from, the constants used, the timing, and -- when a stage
could not run -- the reason and the fix.  A stage that is absent BY DESIGN
(the webcam has no depth sensor) is drawn in blue and says so; a stage that
FAILED is drawn in red.  Those are different facts and this repository has
paid for confusing them before.

THE INSTRUMENT IS CHECKED BEFORE IT IS BELIEVED
==========================================================================
    .venv_vision/bin/python scripts/cv_pickpose_visuals.py --self-test

runs every arithmetic stage against a CONSTRUCTED depth frame whose answer is
known exactly -- a plane at 0.600 m carrying a 40 mm cube -- and asserts the
numbers: plane normal, RMS, cube height, minimum width, and the two controls
that must FAIL (PCA on a cube reads ~53 mm, the axis-aligned box reads wider
still).  No camera needed.  The colour stages are exercised on a drawn frame
purely to prove the drawing code runs; that half is plumbing, not evidence,
and says so.

WHERE THE OUTPUT GOES
==========================================================================
    recordings/vision_thesis/<YYYYmmdd_HHMMSS>/
        report.md                     every stage, its formula, its numbers
        manifest.json                 the same, machine readable
        pose.json                     the arms' measured joints vs the pick pose
        raw/<source>/colour.png       what the camera delivered
        raw/<source>/depth.npy        depth in METRES, float32
        raw/<source>/intrinsics.json
        stages/<source>/NN_<slug>.png one figure per algorithm
        sheet_<source>.png            the whole pipeline on one page

The raw frames are kept so figures can be regenerated without the hardware:

    .venv_vision/bin/python scripts/cv_pickpose_visuals.py --replay <dir>

WHICH INTERPRETER
==========================================================================
`.venv_vision/bin/python`.  It is the only one on this box with cv2, torch,
ultralytics, pyrealsense2 AND rclpy at once.  Under the system python3 the
colour and depth arithmetic still runs; the FastSAM and YOLO-World stages
refuse by name rather than disappearing.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
import traceback
from datetime import datetime

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))

import cv2  # noqa: E402

# ------------------------------------------------------------------ constants

ARM_IP = {"left": "192.168.1.10", "right": "192.168.1.9"}

# The pick path's own green.  scripts/servo_pick_left.py:measure -- these exact
# numbers are what the cube measurement runs on.
PICK_GREEN_LO = (40, 80, 40)
PICK_GREEN_HI = (85, 255, 255)
PICK_OPEN_K = 9
# Height above the fitted plane at which a point stops being the table.
PICK_MIN_H_M = 0.005
# Depth gate, servo_pick_left.measure.
PICK_Z_MIN, PICK_Z_MAX = 0.08, 1.5
# RANSAC, servo_pick_left.fit_plane.
RANSAC_ITERS, RANSAC_TOL_M = 400, 0.006
# The jaws.  rgbd_grasp.GRIPPER_MAX_M / segment_lift.JAW_M.
JAW_M = 0.085
# Kinova vision module, factory intrinsics and the depth->colour baseline.
KINOVA_COLOR_K = (1297.6729, 1298.6313, 620.914, 238.28032)
KINOVA_DEPTH_K = (342.2138, 342.2138, 233.06856, 132.48465)
KINOVA_DEPTH_TO_COLOR_M = (-0.02706, -0.00997, -0.00471)

SOURCES = ("left_gripper", "right_gripper", "scene_rs", "scene_hd")

FONT = cv2.FONT_HERSHEY_SIMPLEX
INK = (28, 28, 28)
PAPER = (250, 250, 248)
RULE = (196, 196, 190)
OK_C = (60, 140, 60)
BAD_C = (40, 40, 205)
DESIGN_C = (160, 90, 30)
ACCENT = (30, 120, 200)


class Refusal(Exception):
    """This stage could not run, and the message says why and what fixes it."""


class ByDesign(Exception):
    """This stage does not apply to this source, which is not a fault."""


# -------------------------------------------------------------- drawing tools

def _wrap(text, width_px, scale, thick):
    """Break `text` into lines that fit `width_px`, greedily by word."""
    words, lines, cur = str(text).split(), [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if cv2.getTextSize(trial, FONT, scale, thick)[0][0] <= width_px or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def text_block(text, width, scale=0.46, thick=1, colour=INK, bg=PAPER,
               pad=8, lead=7):
    """Wrapped text rendered onto its own strip, height decided by the text."""
    lines = []
    for para in str(text).split("\n"):
        lines.extend(_wrap(para, width - 2 * pad, scale, thick) or [""])
    lh = cv2.getTextSize("Ag", FONT, scale, thick)[0][1] + lead
    img = np.full((max(lh * len(lines) + 2 * pad, 1), width, 3), bg, np.uint8)
    y = pad + lh - lead
    for ln in lines:
        cv2.putText(img, ln, (pad, y), FONT, scale, colour, thick, cv2.LINE_AA)
        y += lh
    return img


def fit_tile(img, w, h):
    """Letterbox any image (grey, float, colour) into a w x h tile."""
    a = img
    if a.dtype != np.uint8:
        a = a.astype(np.float32)
        lo, hi = float(np.nanmin(a)), float(np.nanmax(a))
        a = np.zeros_like(a) if hi - lo < 1e-9 else (a - lo) / (hi - lo)
        a = (a * 255).astype(np.uint8)
    if a.ndim == 2:
        a = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    ih, iw = a.shape[:2]
    s = min(w / float(iw), h / float(ih))
    r = cv2.resize(a, (max(1, int(iw * s)), max(1, int(ih * s))),
                   interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_NEAREST)
    out = np.full((h, w, 3), (235, 235, 232), np.uint8)
    y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
    out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    return out


def panel(img, caption, w=560, h=420):
    """One picture with its caption bar underneath."""
    tile = fit_tile(img, w, h)
    cv2.rectangle(tile, (0, 0), (w - 1, h - 1), RULE, 1)
    cap = text_block(caption, w, scale=0.44, bg=(242, 242, 239))
    return np.vstack([tile, cap])


def D(*rows):
    """Structured measurements: (quantity, value, unit) -> rows for export.

    Every stage returns its numbers as VALUES with units, not as a sentence.
    The sentence is a caption; these are the data, and they are written to
    JSON and CSV as well as drawn.
    """
    out = []
    for q, v, u in rows:
        if isinstance(v, np.generic):
            v = v.item()
        if isinstance(v, np.ndarray):
            v = [float(x) for x in v.ravel()]
        out.append(dict(quantity=q, value=v, unit=u))
    return out


def fmt_value(v):
    if isinstance(v, float):
        if v != v:
            return "nan"
        a = abs(v)
        if a >= 1000 or (a < 0.001 and a > 0):
            return "%.4g" % v
        return ("%.4f" % v).rstrip("0").rstrip(".") or "0"
    if isinstance(v, (list, tuple)):
        return ", ".join(fmt_value(x) for x in v)
    if isinstance(v, bool):
        return "yes" if v else "no"
    return str(v)


def table_image(headers, rows, width=1120, title="", max_rows=24):
    """A real table: header rule, aligned columns, one row per record."""
    rows = [[fmt_value(c) for c in r] for r in rows]
    shown = rows[:max_rows]
    ncol = len(headers)
    pad, lh = 12, 26
    widths = []
    for i in range(ncol):
        w = cv2.getTextSize(str(headers[i]), FONT, 0.46, 2)[0][0]
        for r in shown:
            w = max(w, cv2.getTextSize(r[i] if i < len(r) else "", FONT, 0.46,
                                       1)[0][0])
        widths.append(w + 22)
    total = sum(widths) + 2 * pad
    W = max(width, total)
    H = 40 + lh * (len(shown) + 1) + (26 if len(rows) > max_rows else 0) + pad
    img = np.full((H, W, 3), (252, 252, 250), np.uint8)
    if title:
        cv2.putText(img, title, (pad, 24), FONT, 0.5, INK, 1, cv2.LINE_AA)
    y = 44
    x = pad
    for i, hdr in enumerate(headers):
        cv2.putText(img, str(hdr), (x, y + 18), FONT, 0.46, (60, 60, 60), 2,
                    cv2.LINE_AA)
        x += widths[i]
    cv2.line(img, (pad, y + 24), (W - pad, y + 24), RULE, 1)
    for k, r in enumerate(shown):
        y2 = y + 24 + lh * (k + 1)
        if k % 2:
            cv2.rectangle(img, (pad, y2 - 18), (W - pad, y2 + 6),
                          (245, 245, 242), -1)
        x = pad
        for i in range(ncol):
            cv2.putText(img, r[i] if i < len(r) else "", (x, y2), FONT, 0.46,
                        INK, 1, cv2.LINE_AA)
            x += widths[i]
    if len(rows) > max_rows:
        cv2.putText(img, "... and %d more rows, all of them in the CSV"
                    % (len(rows) - max_rows), (pad, H - 12), FONT, 0.44,
                    (120, 120, 120), 1, cv2.LINE_AA)
    return img


#: The text of the figure most recently built, so `report.md` can carry every
#: stage's formula and measured numbers without each stage returning them
#: twice.  Cleared by the runner before each stage.
_LAST_FIGURE = {}


def figure(title, panels, formula="", provenance="", numbers="", note="",
           badge=None, cols=None, panel_w=560, panel_h=420, data=None,
           tables=None):
    """Assemble one stage figure: header, panels, and the MEASUREMENTS table.

    `data` is a list of {quantity, value, unit} from D(); `tables` is
    {name: (headers, rows)} for per-object records. Both are drawn into the
    figure AND exported to JSON and CSV, so no number exists only as pixels.
    """
    _LAST_FIGURE.update(dict(title=title, formula=formula,
                             provenance=provenance, numbers=numbers, note=note,
                             data=data or [], tables=tables or {}))
    if not panels:
        panels = [(np.full((10, 10, 3), 220, np.uint8), "no panel")]
    cols = cols or (1 if len(panels) == 1 else 2 if len(panels) <= 4 else 3)
    total_w = cols * panel_w
    grid_rows, buf = [], []

    def flush():
        """Close off a row of square-ish tiles, padding it to full width."""
        if not buf:
            return
        th = max(t.shape[0] for t in buf)
        row = [np.vstack([t, np.full((th - t.shape[0], t.shape[1], 3), PAPER,
                                     np.uint8)]) for t in buf]
        while len(row) < cols:
            row.append(np.full((th, panel_w, 3), PAPER, np.uint8))
        grid_rows.append(np.hstack(row))
        del buf[:]

    # A CHART IS NOT A PHOTOGRAPH. Letterboxing a 1120x420 histogram into a
    # 560x420 tile halves it and the axis labels stop being readable, which is
    # the whole content. Anything markedly wider than a tile gets its own
    # full-width row instead.
    for im, cap in panels:
        ar = im.shape[1] / float(max(im.shape[0], 1))
        if ar >= 1.7 and cols > 1:
            flush()
            grid_rows.append(panel(im, cap, total_w,
                                   int(round(total_w / ar))))
        else:
            buf.append(panel(im, cap, panel_w, panel_h))
    flush()
    grid = np.vstack(grid_rows)
    W = grid.shape[1]

    head = np.full((54, W, 3), (238, 238, 234), np.uint8)
    cv2.putText(head, title[:120], (12, 36), FONT, 0.82, INK, 2, cv2.LINE_AA)
    if badge:
        label, col = badge
        (tw, _), _ = cv2.getTextSize(label, FONT, 0.55, 2)
        cv2.rectangle(head, (W - tw - 30, 12), (W - 10, 44), col, -1)
        cv2.putText(head, label, (W - tw - 20, 35), FONT, 0.55,
                    (255, 255, 255), 2, cv2.LINE_AA)
    parts = [head]
    if formula:
        parts.append(text_block("FORMULA   " + formula, W, scale=0.52,
                                colour=(120, 40, 20), bg=(246, 244, 236)))
    if provenance:
        parts.append(text_block("SOURCE    " + provenance, W, scale=0.45,
                                colour=(70, 70, 70), bg=(246, 246, 243)))
    parts.append(grid)
    if data:
        parts.append(text_block("MEASUREMENTS", W, scale=0.55,
                                colour=(20, 80, 20), bg=(238, 245, 238)))
        rows = [[d["quantity"], d["value"], d["unit"]] for d in data]
        half = int(math.ceil(len(rows) / 2.0))
        if len(rows) > 8 and W >= 1000:
            a = table_image(["quantity", "value", "unit"], rows[:half],
                            W // 2 - 4, max_rows=40)
            b = table_image(["quantity", "value", "unit"], rows[half:],
                            W - W // 2 - 4, max_rows=40)
            hh = max(a.shape[0], b.shape[0])
            a = np.vstack([a, np.full((hh - a.shape[0], a.shape[1], 3),
                                      (252, 252, 250), np.uint8)])
            b = np.vstack([b, np.full((hh - b.shape[0], b.shape[1], 3),
                                      (252, 252, 250), np.uint8)])
            tbl = np.hstack([a, b])
            tbl = cv2.resize(tbl, (W, tbl.shape[0]))
        else:
            tbl = table_image(["quantity", "value", "unit"], rows, W,
                              max_rows=40)
            if tbl.shape[1] != W:
                tbl = cv2.resize(tbl, (W, tbl.shape[0]))
        parts.append(tbl)
    for tname, (thead, trows) in (tables or {}).items():
        t = table_image(thead, trows, W, title=tname)
        if t.shape[1] != W:
            t = cv2.resize(t, (W, t.shape[0]))
        parts.append(t)
    if numbers:
        parts.append(text_block("SUMMARY   " + numbers, W, scale=0.5,
                                colour=(20, 80, 20), bg=(240, 246, 240)))
    if note:
        parts.append(text_block("NOTE      " + note, W, scale=0.45,
                                colour=(90, 60, 20), bg=(246, 243, 236)))
    return np.vstack(parts)


def refusal_figure(title, reason, by_design=False, width=1160):
    """A stage that could not run, drawn LOUDLY rather than left out."""
    col = DESIGN_C if by_design else BAD_C
    label = "BY DESIGN" if by_design else "REFUSED"
    body = np.full((300, width, 3), (246, 246, 244), np.uint8)
    cv2.line(body, (0, 0), (width, 0), col, 3)
    blk = text_block(reason, width - 40, scale=0.56, colour=col,
                     bg=(246, 246, 244))
    h = min(blk.shape[0], 280)
    body[16:16 + h, 20:width - 20] = blk[:h, :width - 40]
    return figure(title, [(body, label)], badge=(label, col), cols=1,
                  panel_w=width, panel_h=300)


def colourise_depth(depth_m, lo=None, hi=None, holes=(200, 60, 200)):
    """Depth in metres -> a turbo image, with NO-RETURN pixels flagged."""
    d = np.asarray(depth_m, np.float32)
    v = np.isfinite(d) & (d > 0)
    if not v.any():
        return np.full(d.shape + (3,), holes, np.uint8)
    lo = float(np.percentile(d[v], 2)) if lo is None else lo
    hi = float(np.percentile(d[v], 98)) if hi is None else hi
    if hi - lo < 1e-6:
        hi = lo + 1e-3
    x = np.clip((d - lo) / (hi - lo), 0, 1)
    try:
        img = cv2.applyColorMap((x * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    except Exception:                                          # noqa: BLE001
        img = cv2.applyColorMap((x * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[~v] = holes
    return img


def mask_overlay(bgr, mask, colour=(0, 230, 0), alpha=0.55, outline=True):
    """A boolean mask laid over the frame, with its outline drawn."""
    out = bgr.copy()
    m = mask.astype(bool)
    if m.any():
        tint = np.zeros_like(out)
        tint[m] = colour
        out = cv2.addWeighted(out, 1.0, tint, alpha, 0)
        if outline:
            cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(out, cs, -1, (255, 255, 255), 2)
    return out


def bar_chart(counts, edges, w=1120, h=440, title="", xlabel="", marks=(),
              bands=()):
    """A histogram drawn by hand: no plotting backend to negotiate with."""
    img = np.full((h, w, 3), (252, 252, 250), np.uint8)
    l, r, t, b = 70, w - 16, 44, h - 44
    cv2.rectangle(img, (l, t), (r, b), RULE, 1)
    counts = np.asarray(counts, float)
    mx = max(1.0, float(counts.max()))
    n = len(counts)
    x0, x1 = float(edges[0]), float(edges[-1])
    span = max(x1 - x0, 1e-9)

    def X(v):
        return int(l + (float(v) - x0) / span * (r - l))
    for lo, hi, col in bands:
        cv2.rectangle(img, (X(lo), t + 1), (X(hi), b - 1), col, -1)
    bw = max(1, int((r - l) / float(n)))
    for i, c in enumerate(counts):
        x = l + int(i * (r - l) / float(n))
        y = int(b - (c / mx) * (b - t - 6))
        cv2.rectangle(img, (x, y), (x + bw - 1, b - 1), (120, 120, 130), -1)
    # Labels are STAGGERED down the plot: three marks a millimetre apart land
    # on the same x, and written at one height they overprint into a smear.
    for k, (v, lab, col) in enumerate(marks):
        x = X(v)
        cv2.line(img, (x, t), (x, b), col, 2)
        if not lab:
            continue
        tw = cv2.getTextSize(lab, FONT, 0.5, 2)[0][0]
        lx = x + 6 if x + 6 + tw < r else x - 6 - tw
        cv2.putText(img, lab, (max(l + 2, lx), t + 22 + 24 * k), FONT, 0.5,
                    col, 2, cv2.LINE_AA)
    for k in range(6):
        v = x0 + span * k / 5.0
        x = X(v)
        cv2.line(img, (x, b), (x, b + 5), INK, 1)
        cv2.putText(img, "%.3f" % v, (x - 26, b + 24), FONT, 0.44, INK, 1,
                    cv2.LINE_AA)
    lab = "count, tallest bar %d" % int(mx)
    cv2.putText(img, lab, (r - cv2.getTextSize(lab, FONT, 0.44, 1)[0][0], t - 10),
                FONT, 0.44, (110, 110, 110), 1, cv2.LINE_AA)
    xw = cv2.getTextSize(xlabel, FONT, 0.48, 1)[0][0]
    cv2.putText(img, xlabel, ((l + r - xw) // 2, h - 6), FONT, 0.48, INK, 1,
                cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (l, t - 10), FONT, 0.5, INK, 1, cv2.LINE_AA)
    return img


def line_chart(xs, ys, w=1120, h=440, title="", xlabel="", ylabel="",
               vlines=(), points=(), y_from_zero=False):
    """A curve with BOTH axes ticked. y is scaled to the data, not to zero.

    Scaling to zero is what flattened the caliper sweep into a line along the
    top of the panel: the whole content is a 15 mm variation on a 40 mm value.
    """
    img = np.full((h, w, 3), (252, 252, 250), np.uint8)
    l, r, t, b = 96, w - 20, 44, h - 46
    cv2.rectangle(img, (l, t), (r, b), RULE, 1)
    xs = np.asarray(xs, float)
    ys = np.asarray(ys, float)
    x0, x1 = float(xs.min()), float(xs.max())
    y0, y1 = float(ys.min()), float(ys.max())
    if y_from_zero:
        y0 = 0.0
    pad = max(y1 - y0, 1e-9) * 0.10
    y0, y1 = y0 - pad, y1 + pad
    sx = max(x1 - x0, 1e-9)
    sy = max(y1 - y0, 1e-9)

    def P(x, y):
        return (int(l + (x - x0) / sx * (r - l)),
                int(b - (y - y0) / sy * (b - t)))
    for k in range(6):
        gy = y0 + sy * k / 5.0
        yy = P(x0, gy)[1]
        cv2.line(img, (l, yy), (r, yy), (238, 238, 236), 1)
        cv2.putText(img, "%.4g" % gy, (8, yy + 5), FONT, 0.44, INK, 1,
                    cv2.LINE_AA)
        gx = x0 + sx * k / 5.0
        xx = P(gx, y0)[0]
        cv2.line(img, (xx, b), (xx, b + 5), INK, 1)
        cv2.putText(img, "%.4g" % gx, (xx - 22, b + 24), FONT, 0.44, INK, 1,
                    cv2.LINE_AA)
    for v, lab, col in vlines:
        if x0 <= v <= x1:
            xx = P(v, y0)[0]
            cv2.line(img, (xx, t), (xx, b), col, 2)
            if lab:
                tw = cv2.getTextSize(lab, FONT, 0.46, 2)[0][0]
                cv2.putText(img, lab,
                            (xx + 6 if xx + 6 + tw < r else xx - 6 - tw, t + 20),
                            FONT, 0.46, col, 2, cv2.LINE_AA)
    cv2.polylines(img, [np.array([P(x, y) for x, y in zip(xs, ys)], np.int32)],
                  False, (60, 60, 200), 2, cv2.LINE_AA)
    for px, py, col, lab in points:
        q = P(px, py)
        cv2.drawMarker(img, q, col, cv2.MARKER_TILTED_CROSS, 18, 3)
        if lab:
            tw = cv2.getTextSize(lab, FONT, 0.5, 2)[0][0]
            cv2.putText(img, lab,
                        (q[0] + 10 if q[0] + 10 + tw < r else q[0] - 10 - tw,
                         max(t + 18, q[1] - 12)), FONT, 0.5, col, 2, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (l, t - 12), FONT, 0.5, INK, 1, cv2.LINE_AA)
    if ylabel:
        cv2.putText(img, ylabel, (8, t - 14), FONT, 0.46, INK, 1, cv2.LINE_AA)
    xw = cv2.getTextSize(xlabel, FONT, 0.48, 1)[0][0]
    cv2.putText(img, xlabel, ((l + r - xw) // 2, h - 8), FONT, 0.48, INK, 1,
                cv2.LINE_AA)
    return img


def scatter(xs, ys, w=1120, h=560, xlabel="", ylabel="", groups=None,
            lines=(), equal=True, title=""):
    """A 2-D scatter, drawn by hand.  `groups` colours the points."""
    img = np.full((h, w, 3), (252, 252, 250), np.uint8)
    l, r, t, b = 78, w - 16, 44, h - 46
    cv2.rectangle(img, (l, t), (r, b), RULE, 1)
    xs = np.asarray(xs, float)
    ys = np.asarray(ys, float)
    if len(xs) == 0:
        cv2.putText(img, "no points", (l + 20, (t + b) // 2), FONT, 0.7, BAD_C,
                    2, cv2.LINE_AA)
        return img
    x0, x1 = float(xs.min()), float(xs.max())
    y0, y1 = float(ys.min()), float(ys.max())
    px = max(x1 - x0, 1e-6) * 0.05
    py = max(y1 - y0, 1e-6) * 0.05
    x0, x1, y0, y1 = x0 - px, x1 + px, y0 - py, y1 + py
    if equal:
        sx = (r - l) / (x1 - x0)
        sy = (b - t) / (y1 - y0)
        s = min(sx, sy)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        x0, x1 = cx - (r - l) / (2 * s), cx + (r - l) / (2 * s)
        y0, y1 = cy - (b - t) / (2 * s), cy + (b - t) / (2 * s)

    def P(x, y):
        return (int(l + (x - x0) / (x1 - x0) * (r - l)),
                int(b - (y - y0) / (y1 - y0) * (b - t)))
    cols = [(150, 150, 155)] if groups is None else None
    if groups is None:
        for x, y in zip(xs, ys):
            cv2.circle(img, P(x, y), 1, (150, 150, 155), -1)
    else:
        for g, col in groups:
            for x, y in zip(xs[g], ys[g]):
                cv2.circle(img, P(x, y), 1, col, -1)
    for (ax, ay), (bx, by), col, lab in lines:
        cv2.line(img, P(ax, ay), P(bx, by), col, 2)
        if lab:
            cv2.putText(img, lab, P((ax + bx) / 2, (ay + by) / 2), FONT, 0.46,
                        col, 2, cv2.LINE_AA)
    for k in range(6):
        v = x0 + (x1 - x0) * k / 5.0
        x = P(v, y0)[0]
        cv2.putText(img, "%.2f" % v, (x - 20, b + 22), FONT, 0.42, INK, 1,
                    cv2.LINE_AA)
        v = y0 + (y1 - y0) * k / 5.0
        y = P(x0, v)[1]
        cv2.putText(img, "%.2f" % v, (6, y + 4), FONT, 0.42, INK, 1, cv2.LINE_AA)
    xw = cv2.getTextSize(xlabel, FONT, 0.5, 1)[0][0]
    cv2.putText(img, xlabel, ((l + r - xw) // 2, h - 8), FONT, 0.5, INK, 1,
                cv2.LINE_AA)
    cv2.putText(img, ylabel, (6, t - 14), FONT, 0.5, INK, 1, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (l + 180, t - 12), FONT, 0.5, INK, 1,
                    cv2.LINE_AA)
    return img


# ------------------------------------------------------- the algorithms, once
#
# Where a function already exists in the repository it is IMPORTED, so a figure
# cannot drift away from the code it claims to illustrate.  Where the import
# drags in the whole ROS pick executor, a copy is used and the figure SAYS it
# is a copy.

def deproject(depth_m, K, z_min=PICK_Z_MIN, z_max=PICK_Z_MAX, stride=1):
    """Pixels + depth -> camera-frame points.  x=(u-cx)z/fx, y=(v-cy)z/fy, z=z.

    Returns (points (N,3), pixels (N,2) as u,v).  This is the same arithmetic
    as `rgbd_grasp.deproject` and `servo_pick_left.measure`; it is written out
    once here because every depth stage needs both halves back.
    """
    fx, fy, cx, cy = K
    d = np.asarray(depth_m, np.float32)
    if stride > 1:
        d = d[::stride, ::stride]
    v, u = np.nonzero(np.isfinite(d) & (d > z_min) & (d < z_max))
    z = d[v, u]
    if stride > 1:
        u = u * stride
        v = v * stride
    P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)
    return P.astype(np.float64), np.stack([u, v], 1)


def fit_plane_pick(P, it=RANSAC_ITERS, tol=RANSAC_TOL_M, seed=0):
    """The pick path's own plane fitter.  RANSAC, then a PCA refinement.

    Copied from `scripts/servo_pick_left.py:fit_plane` -- that module imports
    rclpy, the pick executor and TRAC-IK at module level, none of which belong
    in a camera-only program.  `_plane_provenance` records whether the ORIGINAL
    was importable and used instead.

    Both halves of the refinement matter.  Returning the refined normal with
    the RANSAC offset describes a different plane from either, the inlier mask
    can then select nothing, and `mean()` of an empty slice is the NaN that
    was reported as "plane RMS nan mm" while the run also reported FOUND.
    """
    rng = np.random.default_rng(seed)
    best, bc = None, -1
    for _ in range(it):
        p = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n = n / L
        d = -n @ p[0]
        c = int((np.abs(P @ n + d) < tol).sum())
        if c > bc:
            bc, best = c, (n, d)
    if best is None:
        raise Refusal("RANSAC drew 400 degenerate triples; the cloud is "
                      "collinear or has fewer than three distinct points")
    n, d = best
    m = np.abs(P @ n + d) < tol
    Q = P[m]
    cen = Q.mean(0)
    C = np.cov((Q - cen).T)
    w, V = np.linalg.eigh(C)
    n_ref = V[:, 0] / np.linalg.norm(V[:, 0])
    d_ref = -float(n_ref @ cen)
    return dict(n_ransac=n, d_ransac=float(d), n=n_ref, d=d_ref,
                inliers=np.abs(P @ n_ref + d_ref) < tol,
                inliers_ransac=m, n_iters=it, tol=tol)


def _plane_fitter():
    """Use the pick path's own function when it can be imported.  Say which."""
    try:
        from servo_pick_left import fit_plane as orig            # noqa: F401
    except Exception as exc:                                     # noqa: BLE001
        return fit_plane_pick, ("local copy of servo_pick_left.fit_plane "
                                "(the original is not importable here: %s)"
                                % type(exc).__name__)
    def wrapped(P, it=RANSAC_ITERS, tol=RANSAC_TOL_M, seed=0):
        n, d, inl = orig(P, it=it, tol=tol, seed=seed)
        return dict(n_ransac=n, d_ransac=float(d), n=n, d=float(d),
                    inliers=inl, inliers_ransac=inl, n_iters=it, tol=tol)
    return wrapped, "scripts/servo_pick_left.py:fit_plane, imported"


def min_width(flat2, step_deg=0.25):
    """Rotating calipers: the narrowest way an outline presents itself.

    `segment_lift._min_width`, imported when available.  This is NOT PCA: the
    principal axes of a square are degenerate, so SVD returns the diagonal and
    a 40 mm cube measures 40*sqrt(2) = 53 mm.  The jaws close across the
    MINIMUM width, which is what this searches for.
    """
    try:
        from srl_perception.segment_lift import _min_width as f
        return f(flat2, step_deg=step_deg)
    except Exception:                                            # noqa: BLE001
        th = np.radians(np.arange(0.0, 180.0, step_deg))
        dirs = np.stack([np.cos(th), np.sin(th)], axis=1)
        proj = np.asarray(flat2, float) @ dirs.T
        span = proj.max(axis=0) - proj.min(axis=0)
        k = int(np.argmin(span))
        return float(span[k]), dirs[k], float(th[k])


#: The band of range each camera actually works in.
#:
#: THIS WAS ONE CONSTANT AND IT WAS THE WRIST CAMERA'S. Measured 2026-08-30 on
#: the live rig: the RealSense sits across the room, its nearest return is
#: 0.741 m and its median is 3.83 m, and the 0.08-1.5 m gate -- which is the
#: PICK path's band, correct for a camera on the hand -- kept **12 of 85 137
#: returns**. Fifteen of that camera's stages then refused for want of depth,
#: naming a shortage the camera did not have. A gate is part of the
#: measurement, so it belongs to the thing being measured.
DEPTH_BAND = {"left_gripper": (PICK_Z_MIN, PICK_Z_MAX),
              "right_gripper": (PICK_Z_MIN, PICK_Z_MAX),
              "scene_rs": (0.15, 8.0),
              "self_test": (PICK_Z_MIN, PICK_Z_MAX)}
DEFAULT_BAND = (0.15, 8.0)


def align_depth_to_colour(depth_m, K_d, K_c, size_c, t_dc=(0, 0, 0), splat=0):
    """Depth in the depth sensor's frame -> a depth image in the COLOUR frame.

    Forward projection with a z-buffer: deproject with the DEPTH intrinsics,
    translate by the recorded depth->colour baseline, project with the COLOUR
    intrinsics, keep the nearest point per colour pixel.

    The rotation between the two sensors is taken as identity, which is what
    the recorded baseline alone can support.  The live pick path uses the URDF
    FK between `camera_depth_frame` and `camera_color_frame` instead, and that
    difference is stated on the figure rather than buried here.
    """
    P, _ = deproject(depth_m, K_d, z_min=0.02, z_max=6.0)
    if len(P) == 0:
        raise Refusal("the depth frame has no valid return to align")
    P = P + np.asarray(t_dc, float)
    fx, fy, cx, cy = K_c
    W, H = size_c
    z = P[:, 2]
    ok = z > 1e-6
    u = (P[ok, 0] * fx / z[ok] + cx).astype(np.int32)
    v = (P[ok, 1] * fy / z[ok] + cy).astype(np.int32)
    zz = z[ok]
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, zz = u[inside], v[inside], zz[inside]
    out = np.full((H, W), np.inf, np.float32)
    order = np.argsort(-zz)                       # far first, near overwrites
    # SPLAT, BECAUSE A FORWARD PROJECTION INTO A BIGGER IMAGE IS STIPPLE.
    #
    # 480x270 of depth into 1280x720 of colour is one sample per ~7 colour
    # pixels, and measured on the live rig it filled **7.2%** of the frame.
    # Every above-plane region was then a dot pattern, every connected
    # component came out under the 40 px floor, and the "what is standing on
    # the surface" stage reported nothing while the 3-D box stage -- which
    # works per mask and does not need connectivity -- posed five objects in
    # the same frame. The splat radius is derived from the resolution ratio,
    # not chosen: it covers exactly the gap one depth sample spans.
    for du in range(-splat, splat + 1):
        for dv in range(-splat, splat + 1):
            uu = np.clip(u[order] + du, 0, W - 1)
            vv2 = np.clip(v[order] + dv, 0, H - 1)
            np.minimum.at(out, (vv2, uu), zz[order])
    out[~np.isfinite(out)] = 0.0
    return out, dict(projected=int(inside.sum()), dropped=int((~inside).sum()),
                     filled_frac=float((out > 0).mean()),
                     splat_radius_px=int(splat))


# ----------------------------------------------------------------- the frames

class Frame:
    """One camera's contribution: what it delivered and where it came from."""

    def __init__(self, name, colour, depth_m=None, K_c=None, K_d=None,
                 depth_is_aligned=False, provenance="", notes=None,
                 no_depth_reason=""):
        self.name = name
        self.colour = colour
        self.depth_m = depth_m
        self.K_c = tuple(K_c) if K_c else None
        self.K_d = tuple(K_d) if K_d else (tuple(K_c) if K_c else None)
        self.depth_is_aligned = depth_is_aligned
        self.provenance = provenance
        self.notes = notes or []
        self.no_depth_reason = no_depth_reason
        self.band = DEPTH_BAND.get(name, DEFAULT_BAND)
        self.cache = {}

    @property
    def size(self):
        h, w = self.colour.shape[:2]
        return (w, h)


def _ros_env(domain):
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "SHM")
    if domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(domain)


def _decode_image(msg):
    """A sensor_msgs/Image -> a numpy array, without cv_bridge.

    cv_bridge is not importable from `.venv_vision`, and decoding four
    encodings by hand is less fragile than a second interpreter.
    """
    enc = msg.encoding
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ("bgr8", "rgb8"):
        a = buf.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == "rgb8" else a.copy()
    if enc in ("mono8",):
        return buf.reshape(msg.height, msg.width).copy()
    if enc in ("16UC1", "mono16"):
        a = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height,
                                                              msg.width)
        return a.astype(np.float32) * 0.001            # millimetres -> metres
    if enc in ("32FC1",):
        a = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height,
                                                               msg.width)
        return a.copy()
    raise Refusal("unhandled image encoding %r on %s" % (enc, msg.header.frame_id))


def grab_gripper_ros(arm, domain=7, wait_s=12.0):
    """Colour + depth + both camera_infos from the topics the driver publishes.

    Preferred over RTSP because `bringup_arm.sh` already has the streams open
    and Kinova allows only two clients per stream -- a third connection is
    refused, and the symptom is an unrelated-looking camera failure.
    """
    _ros_env(domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    started_here = not rclpy.ok()
    if started_here:
        rclpy.init(args=[])
    node = Node("cv_pickpose_visuals_%s_%d" % (arm, os.getpid() % 10000))
    got = {}
    subs = [
        (Image, "/%s_camera/color/image_raw" % arm, "c"),
        (CameraInfo, "/%s_camera/color/camera_info" % arm, "ck"),
        (Image, "/%s_camera/depth/image_raw" % arm, "d"),
        (CameraInfo, "/%s_camera/depth/camera_info" % arm, "dk"),
    ]
    for T, topic, key in subs:
        node.create_subscription(
            T, topic, lambda m, k=key: got.__setitem__(k, m),
            qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < wait_s and len(got) < 4:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    if started_here:
        rclpy.shutdown()
    if len(got) < 4:
        live = running_domains()
        hint = ""
        if live and str(domain) not in live:
            hint = (" THE ARM'S OWN PROCESSES ARE ON ROS_DOMAIN_ID=%s, not %s "
                    "-- %s. Re-run with --domain %s (or --domain auto, which "
                    "reads this from /proc)."
                    % (", ".join(sorted(live)), domain,
                       "; ".join(v[0] for v in live.values()),
                       sorted(live)[0]))
        raise Refusal(
            "only %d of 4 topics answered on /%s_camera within %.0f s "
            "(missing: %s), on ROS_DOMAIN_ID=%s. Either the camera driver is "
            "not up -- `bash scripts/bringup_arm.sh %s` -- or this terminal is "
            "on a different domain from the one it was started on.%s"
            % (len(got), arm, wait_s,
               ", ".join(k for _, _, k in subs if k not in got), domain, arm,
               hint))
    col = _decode_image(got["c"])
    dep = _decode_image(got["d"])
    ck, dk = got["ck"], got["dk"]
    K_c = (ck.k[0], ck.k[4], ck.k[2], ck.k[5])
    K_d = (dk.k[0], dk.k[4], dk.k[2], dk.k[5])
    return Frame(
        "%s_gripper" % arm, col, dep, K_c, K_d, depth_is_aligned=False,
        provenance=("Kinova vision module on the %s arm, over ROS topics "
                    "/%s_camera/{color,depth} (driver: kinova_vision_node, "
                    "started by bringup_arm.sh), ROS_DOMAIN_ID=%s"
                    % (arm, arm, os.environ.get("ROS_DOMAIN_ID"))),
        notes=["intrinsics read from the published camera_info, not assumed: "
               "colour cx=%.1f cy=%.1f on a %dx%d image -- the principal "
               "point is not the image centre and assuming it is throws every "
               "ray"
               % (K_c[2], K_c[3], col.shape[1], col.shape[0])])


def grab_gripper_rtsp(arm, depth_seconds=7):
    """The fallback: colour over ffmpeg, depth over GStreamer.  No ROS."""
    from srl_perception.srl_cameras import GripperCamera, CameraError
    ip = ARM_IP[arm]
    cam = GripperCamera(ip)
    try:
        col = cam.read_colour()
    except CameraError as exc:
        raise Refusal(str(exc))
    dep, dnote = None, ""
    try:
        dep = cam.read_depth(seconds=depth_seconds)
    except CameraError as exc:
        dnote = str(exc)
    cam.close()
    return Frame(
        "%s_gripper" % arm, col, dep, KINOVA_COLOR_K, KINOVA_DEPTH_K,
        depth_is_aligned=False,
        provenance="Kinova vision module on the %s arm, direct RTSP from %s "
                   "(no ROS stack was answering)" % (arm, ip),
        notes=(["factory intrinsics from srl_cameras, not read from this "
                "robot"] + ([dnote] if dnote else [])),
        no_depth_reason=dnote)


#: Depth+colour combinations to try on the RealSense, best first.
#:
#: NEGOTIATED, NOT HARDCODED, and that is the whole point of this list. On
#: 2026-08-30 the D435i came up over usbip enumerated as **USB 2.1**, and at
#: USB 2.1 the stereo module does not offer 424x240 AT ALL -- its smallest
#: depth mode is 480x270. `real_calibration/scene_cameras.py` asks for
#: 424x240 unconditionally, so librealsense answered "Couldn't resolve
#: requests" and the camera read as broken when it was working perfectly at a
#: mode nobody asked for. The device is asked what it HAS before anything is
#: requested of it.
RS_CANDIDATES = [
    ((424, 240, 15), (640, 480, 15)),      # USB3: measured 60/60 live frames
    ((480, 270, 15), (640, 480, 15)),      # USB2.1: the smallest depth it has
    ((480, 270, 6), (640, 480, 6)),
    ((640, 480, 15), (640, 480, 15)),
    ((640, 480, 6), (640, 480, 6)),
]


def rs_profiles():
    """What this device actually offers right now, and how it enumerated."""
    import pyrealsense2 as rs
    ctx = rs.context()
    devs = list(ctx.query_devices())
    if not devs:
        raise Refusal(
            "no RealSense is connected. Over usbip Windows can believe the "
            "device is attached while Linux has nothing -- the repair is "
            "detach THEN attach: `python3 scripts/usb_cameras.py --fix`.")
    d = devs[0]
    try:
        usb = d.get_info(rs.camera_info.usb_type_descriptor)
    except Exception:                                            # noqa: BLE001
        usb = "?"
    have = set()
    for sen in d.query_sensors():
        for pr in sen.get_stream_profiles():
            v = pr.as_video_stream_profile()
            if not v:
                continue
            st = str(pr.stream_type()).split(".")[-1]
            fmt = str(v.format()).split(".")[-1]
            if st == "depth" and fmt == "z16":
                have.add(("depth", v.width(), v.height(), pr.fps()))
            elif st == "color" and fmt == "bgr8":
                have.add(("color", v.width(), v.height(), pr.fps()))
    return d, usb, have


def grab_scene_rs(frames=12, rotate180=True):
    """RealSense D435i: depth ALIGNED to colour, averaged over valid pixels.

    The stream mode is negotiated against what the device reports, because
    what it offers depends on how it enumerated -- see RS_CANDIDATES.
    """
    import pyrealsense2 as rs
    dev, usb, have = rs_profiles()
    picks = [(dp, cp) for dp, cp in RS_CANDIDATES
             if ("depth",) + dp[:2] + (dp[2],) in have
             and ("color",) + cp[:2] + (cp[2],) in have]
    if not picks:
        raise Refusal(
            "this RealSense (USB %s) offers none of the depth+colour "
            "combinations this program knows. It has: %s"
            % (usb, ", ".join(sorted("%s %dx%d@%d" % h for h in have))[:400]))
    last = ""
    # EACH CANDIDATE GETS MORE THAN ONE GO, AND A FAILURE COSTS A SETTLE.
    #
    # Measured 2026-08-30: a failed pipe.start leaves the device busy for a
    # second or two, so the NEXT candidate fails too -- errno 16, "Device or
    # resource busy" -- and a single-pass loop walks straight down to its
    # worst mode. That is how this landed on 640x480@6 with 13% of pixels
    # valid when 480x270@15 was available and gave 25%. A transient must not
    # be allowed to demote the mode.
    attempts = [(dp, cp, k) for k in (0, 1) for dp, cp in picks]
    for dp, cp, k in attempts:
        if last:
            time.sleep(2.5)
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, dp[0], dp[1], rs.format.z16, dp[2])
        cfg.enable_stream(rs.stream.color, cp[0], cp[1], rs.format.bgr8, cp[2])
        try:
            prof = pipe.start(cfg)
        except Exception as exc:                                 # noqa: BLE001
            last = "%dx%d@%d depth + %dx%d@%d colour: %s" % (dp + cp + (exc,))
            continue
        try:
            scale = prof.get_device().first_depth_sensor().get_depth_scale()
            ci = prof.get_stream(rs.stream.color) \
                     .as_video_stream_profile().get_intrinsics()
            align = rs.align(rs.stream.color)
            for _ in range(15):
                pipe.wait_for_frames(timeout_ms=8000)
            ds, colour, hashes = [], None, []
            import hashlib
            for _ in range(max(2, frames)):
                f = align.process(pipe.wait_for_frames(timeout_ms=8000))
                dd = np.asanyarray(f.get_depth_frame().get_data()) \
                    .astype(np.float32) * scale
                colour = np.asanyarray(f.get_color_frame().get_data()).copy()
                ds.append(dd)
                hashes.append(hashlib.md5(dd.tobytes()).hexdigest()[:8])
            Dp = np.stack(ds)
            V = Dp > 0
            cnt = V.sum(0)
            depth = np.where(cnt > 0, np.where(V, Dp, 0).sum(0)
                             / np.maximum(cnt, 1), 0.0).astype(np.float32)
            uniq = len(set(hashes))
        except Exception as exc:                                 # noqa: BLE001
            last = "%dx%d@%d depth started and then: %s" % (dp + (exc,))
            try:
                pipe.stop()
            except Exception:                                    # noqa: BLE001
                pass
            continue
        finally:
            try:
                pipe.stop()
            except Exception:                                    # noqa: BLE001
                pass
        # DEPTH CAN BE STALE AND LOOK HEALTHY. librealsense re-delivers the
        # last depth frame inside each frameset when the stream stalls, so
        # wait_for_frames returns at full rate and the content never changes.
        if uniq < max(2, len(hashes) // 3):
            last = ("%dx%d@%d depth was STALE: %d frames, %d distinct images"
                    % (dp[0], dp[1], dp[2], len(hashes), uniq))
            continue
        w, h = ci.width, ci.height
        if rotate180:
            depth = cv2.rotate(depth, cv2.ROTATE_180)
            colour = cv2.rotate(colour, cv2.ROTATE_180)
            K = (ci.fx, ci.fy, w - 1 - ci.ppx, h - 1 - ci.ppy, w, h)
        else:
            K = (ci.fx, ci.fy, ci.ppx, ci.ppy, w, h)
        live = dict(frames=len(hashes), unique_content=uniq,
                    unique_frame_numbers=uniq,
                    valid_frac=float((depth > 0).mean()))
        chosen = ("depth %dx%d@%d + colour %dx%d@%d, USB %s"
                  % (dp + cp + (usb,)))
        break
    else:
        raise Refusal("every stream combination this RealSense offers failed. "
                      "Last: %s" % last)
    fx, fy, cx, cy, w, h = K
    return Frame(
        "scene_rs", colour, depth, (fx, fy, cx, cy), (fx, fy, cx, cy),
        depth_is_aligned=True,
        provenance=("RealSense D435i across the room, %s, negotiated against "
                    "what the device reports it has. Depth is aligned to "
                    "colour by librealsense and averaged over %d frames, "
                    "valid pixels only." % (chosen, frames)),
        notes=["rotated 180 deg: the camera is mounted upside down and the "
               "principal point is rotated WITH the image (cx'=W-1-cx)"
               if rotate180 else "not rotated",
               "liveness: %d frames, %d distinct depth images, %d distinct "
               "frame numbers, %.0f%% of pixels valid"
               % (live["frames"], live["unique_content"],
                  live["unique_frame_numbers"], 100 * live["valid_frac"])])


def grab_scene_hd_ros(domain=7, wait_s=10.0):
    """The HD webcam through `scene_camera_node`, when that node holds it.

    V4L2 ALLOWS EXACTLY ONE CAPTURE CLIENT. If `scene_camera_node` is running
    it owns /dev/videoN, and every other opener gets "can't open camera by
    index" -- which reads like a permission or attach problem and is neither.
    Reading its topic is the correct answer: it is the same pixels, and it
    does not fight the node for the device.
    """
    _ros_env(domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    started = not rclpy.ok()
    if started:
        rclpy.init(args=[])
    node = Node("cv_pickpose_scene_hd_%d" % (os.getpid() % 10000))
    got = {}
    node.create_subscription(Image, "/scene_camera/image_raw",
                             lambda m: got.__setitem__("c", m),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/scene_camera/camera_info",
                             lambda m: got.__setitem__("k", m),
                             qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < wait_s and "c" not in got:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    if started:
        rclpy.shutdown()
    if "c" not in got:
        raise Refusal("scene_camera_node holds the device but nothing arrived "
                      "on /scene_camera/image_raw within %.0f s on "
                      "ROS_DOMAIN_ID=%s" % (wait_s, domain))
    col = _decode_image(got["c"])
    K = None
    if "k" in got and any(got["k"].k):
        kk = got["k"]
        K = (kk.k[0], kk.k[4], kk.k[2], kk.k[5])
    return Frame(
        "scene_hd", col, None, K, K,
        provenance="HD USB webcam through scene_camera_node's own topic "
                   "/scene_camera/image_raw on ROS_DOMAIN_ID=%s -- the node "
                   "holds /dev/video*, and V4L2 allows one client" % domain,
        notes=([] if K else
               ["NO INTRINSICS: the node published a zero camera_info, which "
                "it does deliberately until it is calibrated. Stages needing "
                "fx, fy, cx, cy refuse rather than assume cx = w/2."]),
        no_depth_reason="the HD USB webcam is a colour camera; it has no "
                        "depth sensor. This is a by-design absence, not a "
                        "fault -- the RealSense is the depth scene camera.")


def grab_scene_hd(width=1280, height=720):
    """The HD USB webcam.  MJPG is mandatory over usbip; YUYV delivers nothing."""
    from srl_perception.srl_cameras import probe_scene_camera, video_identity
    from srl_perception.srl_cameras import video_indices
    have = video_indices()
    if not have:
        raise Refusal(
            "there is no /dev/video* on this host at all, so no USB camera "
            "has been handed into WSL. There is no USB in WSL: every camera "
            "comes over usbipd. Fix it with `python3 scripts/usb_cameras.py "
            "--fix`, which also repairs the stale case where Windows still "
            "believes the device is attached and `attach` refuses.")
    idx, tried = probe_scene_camera(640, 480)
    if idx is None:
        holders = []
        for i in have:
            for who in who_holds("/dev/video%d" % i):
                holders.append("/dev/video%d is open in %s" % (i, who))
        raise Refusal(
            "video nodes exist (%s) but none delivered a frame: %s.%s"
            % (", ".join("video%d" % i for i in have),
               "; ".join(tried) if tried else "no candidate was tried",
               (" V4L2 ALLOWS ONE CAPTURE CLIENT and %s -- reading that "
                "node's topic instead." % "; ".join(holders)) if holders else
               " Attach the cameras with `python3 scripts/usb_cameras.py "
               "--fix`; the busid changes every session."))
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    f = None
    for _ in range(8):
        ok, x = cap.read()
        if ok and x is not None:
            f = x
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
           int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    cap.release()
    if f is None:
        raise Refusal("/dev/video%d opened and delivered no frame. Over usbip "
                      "that is the signature of the wrong pixel format -- "
                      "MJPG is required." % idx)
    name, vidpid = video_identity(idx)
    h, w = f.shape[:2]
    # No calibration exists for this camera, so no intrinsics are invented.
    return Frame(
        "scene_hd", f, None, None, None,
        provenance="HD USB webcam /dev/video%d (%s %s), MJPG %dx%d"
                   % (idx, name or "?", vidpid or "?", got[0], got[1]),
        notes=["NO INTRINSICS: this camera has never been calibrated in this "
               "repository, so every stage that needs fx, fy, cx, cy refuses "
               "rather than assuming cx = w/2. Calibrate it with "
               "scripts/calibrate_scene_camera.py."],
        no_depth_reason="the HD USB webcam is a colour camera; it has no "
                        "depth sensor. This is a by-design absence, not a "
                        "fault -- the RealSense is the depth scene camera.")


# ------------------------------------------------------------------- the pose
#
# READ ONLY.  Nothing in this file publishes, calls a service, or opens a
# Kortex session.  The arms are put at the pick pose by the operator; all this
# does is measure where they actually are and say how far that is from the
# saved pick pose, so a figure captioned "at the pick pose" is a measurement
# rather than an assumption.

def read_pose_file(path):
    """`joint_N: degrees` -> {name: radians}, comments ignored."""
    out = {}
    with open(path) as fh:
        for line in fh:
            line = line.split("#")[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            try:
                out[k.strip()] = math.radians(float(v.strip()))
            except ValueError:
                continue
    return out


def ang_wrap(d):
    """Angular difference wrapped to [-pi, pi].

    Joints 3, 5 and 7 of a Gen3 are CONTINUOUS: -179.05 and +181.02 deg are
    the SAME pose, and plain subtraction calls that 360.08 deg of error.
    """
    return (np.asarray(d, float) + np.pi) % (2 * np.pi) - np.pi


def measure_pose(domain=7, wait_s=6.0):
    """Subscribe to /real/joint_states and compare with the saved pick pose."""
    _ros_env(domain)
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception as exc:                                     # noqa: BLE001
        return dict(measured=False,
                    why="rclpy is not importable here (%s), so the arms' "
                        "pose could not be read. Run under "
                        ".venv_vision/bin/python with ROS sourced." % exc)
    started = not rclpy.ok()
    if started:
        rclpy.init(args=[])
    node = Node("cv_pickpose_pose_reader_%d" % (os.getpid() % 10000))
    seen, stamps = {}, set()

    def on_js(m):
        for i, nm in enumerate(m.name):
            if i < len(m.position):
                seen[nm] = float(m.position[i])
        stamps.add((m.header.stamp.sec, m.header.stamp.nanosec))

    node.create_subscription(JointState, "/real/joint_states", on_js, 10)
    node.create_subscription(JointState, "/joint_states", on_js, 10)
    t0 = time.time()
    while time.time() - t0 < wait_s:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    if started:
        rclpy.shutdown()
    if not seen:
        return dict(measured=False,
                    why="no /real/joint_states or /joint_states within %.0f s "
                        "on ROS_DOMAIN_ID=%s. The arms publish through "
                        "kortex_highlevel_bridge; start it with "
                        "`bash scripts/bringup_arm.sh left` (and right). The "
                        "figures are still valid pictures of what the cameras "
                        "saw -- only the claim 'at the pick pose' is "
                        "unsupported." % (wait_s, domain))
    out = dict(measured=True, distinct_stamps=len(stamps), arms={})
    for arm in ("left", "right"):
        path = os.path.join(WS, "config", "pick_pose_ideal_%s.txt" % arm)
        if not os.path.exists(path):
            continue
        want = read_pose_file(path)
        rows, worst = [], 0.0
        for j in range(1, 8):
            key = "joint_%d" % j
            live = seen.get("%s_%s" % (arm, key), seen.get(key))
            if live is None or key not in want:
                rows.append(dict(joint=key, live_deg=None, want_deg=None,
                                 delta_deg=None))
                continue
            dlt = float(np.degrees(ang_wrap(live - want[key])))
            worst = max(worst, abs(dlt))
            rows.append(dict(joint=key, live_deg=round(math.degrees(live), 3),
                             want_deg=round(math.degrees(want[key]), 3),
                             delta_deg=round(dlt, 3)))
        have = [r for r in rows if r["delta_deg"] is not None]
        out["arms"][arm] = dict(
            reference="config/pick_pose_ideal_%s.txt" % arm,
            joints=rows, worst_deviation_deg=round(worst, 3) if have else None,
            at_pick_pose=(bool(have) and worst <= 2.0),
            n_joints_read=len(have))
    return out


# ------------------------------------------------------------ stage machinery

STAGES = []


def stage(num, slug, title, group="colour"):
    def deco(fn):
        STAGES.append(dict(num=num, slug=slug, title=title, group=group,
                           fn=fn))
        return fn
    return deco


def need_K(fr):
    if not fr.K_c:
        raise Refusal(
            "this camera has no intrinsics in this repository, and this stage "
            "is arithmetic on fx, fy, cx, cy. Assuming cx = w/2 is exactly "
            "the error that throws a ray by 122 px on the Kinova module -- "
            "94 mm at 1 m, three times the grasp tolerance. Calibrate it: "
            "scripts/calibrate_scene_camera.py")
    return fr.K_c


def need_depth(fr):
    if fr.depth_m is None:
        raise ByDesign(fr.no_depth_reason or "no depth frame from this source")
    return fr.depth_m


def hsv_of(fr):
    if "hsv" not in fr.cache:
        fr.cache["hsv"] = cv2.cvtColor(fr.colour, cv2.COLOR_BGR2HSV)
    return fr.cache["hsv"]


def green_masks(fr):
    """The pick path's green, before and after its 9x9 opening."""
    if "green" not in fr.cache:
        raw = cv2.inRange(hsv_of(fr), np.array(PICK_GREEN_LO),
                          np.array(PICK_GREEN_HI))
        opened = cv2.morphologyEx(
            raw, cv2.MORPH_OPEN,
            np.ones((PICK_OPEN_K, PICK_OPEN_K), np.uint8))
        fr.cache["green"] = (raw, opened)
    return fr.cache["green"]


def components(fr, min_area=40):
    if "cc" not in fr.cache:
        _, opened = green_masks(fr)
        n, lab, st, ce = cv2.connectedComponentsWithStats(opened, 8)
        rows = []
        for i in range(1, n):
            a = int(st[i, cv2.CC_STAT_AREA])
            if a < min_area:
                continue
            rows.append(dict(
                i=i, area=a,
                box=(int(st[i, cv2.CC_STAT_LEFT]), int(st[i, cv2.CC_STAT_TOP]),
                     int(st[i, cv2.CC_STAT_WIDTH]), int(st[i, cv2.CC_STAT_HEIGHT])),
                centroid=(float(ce[i][0]), float(ce[i][1]))))
        rows.sort(key=lambda r: -r["area"])
        fr.cache["cc"] = (lab, rows)
    return fr.cache["cc"]


def sam_masks(fr, imgsz=1024, conf=0.25, iou=0.7):
    """FastSAM's regions for this frame, computed once and reused."""
    if "sam" in fr.cache:
        if isinstance(fr.cache["sam"], Exception):
            raise Refusal(str(fr.cache["sam"]))
        return fr.cache["sam"]
    try:
        from ultralytics import FastSAM
    except Exception as exc:                                     # noqa: BLE001
        e = Refusal(
            "ultralytics is not importable (%s), so FastSAM cannot run. Use "
            ".venv_vision/bin/python. This is REPORTED rather than quietly "
            "falling back to clustering: a map built by clustering has "
            "different failure modes and must not look like one built by "
            "segmentation." % exc)
        fr.cache["sam"] = e
        raise e
    w = os.path.join(WS, "FastSAM-s.pt")
    if not os.path.exists(w):
        e = Refusal("no FastSAM-s.pt in %s. ultralytics would DOWNLOAD it, "
                    "which on a lab machine with no route out is a hang "
                    "rather than an error." % WS)
        fr.cache["sam"] = e
        raise e
    t0 = time.time()
    model = fr.cache.get("_sam_model") or FastSAM(w)
    fr.cache["_sam_model"] = model
    res = model.predict(fr.colour, device="cpu", retina_masks=True,
                        imgsz=imgsz, conf=conf, iou=iou, verbose=False)
    r = res[0]
    H, W = fr.colour.shape[:2]
    out = []
    if r.masks is not None:
        for mk in r.masks.data.cpu().numpy():
            m = mk.astype(bool)
            if m.shape != (H, W):
                m = cv2.resize(mk.astype(np.uint8), (W, H),
                               interpolation=cv2.INTER_NEAREST) > 0
            out.append(m)
    fr.cache["sam"] = out
    fr.cache["sam_s"] = time.time() - t0
    fr.cache["sam_cfg"] = dict(imgsz=imgsz, conf=conf, iou=iou, device="cpu")
    if not out:
        raise Refusal("FastSAM ran in %.1f s and returned no regions at all "
                      "for this frame" % fr.cache["sam_s"])
    return out


def colourise_labels(shape, masks, alpha=0.65, base=None):
    """Every mask in its own colour, over the frame."""
    rng = np.random.default_rng(3)
    out = np.zeros(shape[:2] + (3,), np.uint8) if base is None else base.copy()
    for m in masks:
        c = rng.integers(60, 255, 3).tolist()
        tint = np.zeros_like(out)
        tint[m] = c
        out = cv2.addWeighted(out, 1.0, tint, alpha, 0)
    return out


# ------------------------------------------------------------- COLOUR  stages

@stage(1, "raw_colour", "The frame, as the camera delivered it")
def st_raw(fr):
    img = fr.colour
    h, w = img.shape[:2]
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    focus = float(cv2.Laplacian(grey, cv2.CV_64F).var())
    clipped = float((grey >= 254).mean() * 100)
    dark = float((grey <= 1).mean() * 100)
    hist = np.zeros((260, 520, 3), np.uint8)
    hist[:] = (252, 252, 250)
    for ch, col in ((0, (200, 90, 60)), (1, (60, 170, 60)), (2, (60, 60, 210))):
        hc = cv2.calcHist([img], [ch], None, [64], [0, 256]).ravel()
        hc = hc / max(hc.max(), 1) * 220
        pts = [(int(8 + i * 500 / 63.0), int(250 - v)) for i, v in enumerate(hc)]
        cv2.polylines(hist, [np.array(pts, np.int32)], False, col, 2,
                      cv2.LINE_AA)
    cv2.putText(hist, "B G R histograms, 64 bins", (10, 20), FONT, 0.45, INK,
                1, cv2.LINE_AA)
    return figure(
        "01  RAW FRAME", [(img, "the frame, untouched"),
                          (hist, "per-channel intensity")],
        data=D(("frame width", w, "px"), ("frame height", h, "px"),
               ("focus, variance of Laplacian", round(focus, 1), "-"),
               ("pixels clipped white", round(clipped, 3), "%"),
               ("pixels crushed black", round(dark, 3), "%"),
               ("mean blue", round(float(img[:, :, 0].mean()), 2), "0-255"),
               ("mean green", round(float(img[:, :, 1].mean()), 2), "0-255"),
               ("mean red", round(float(img[:, :, 2].mean()), 2), "0-255"),
               ("median grey", float(np.median(grey)), "0-255"),
               ("grey std", round(float(grey.std()), 2), "0-255")),
        formula="none -- this is the input every later stage is a function of",
        provenance=fr.provenance,
        numbers="%d x %d px | focus (variance of Laplacian) %.0f | %.2f%% of "
                "pixels clipped white, %.2f%% crushed black"
                % (w, h, focus, clipped, dark),
        note="; ".join(fr.notes) if fr.notes else "",
        badge=("INPUT", ACCENT))


@stage(2, "intrinsics", "Intrinsics and the ray each pixel stands for")
def st_intrinsics(fr):
    fx, fy, cx, cy = need_K(fr)
    img = fr.colour.copy()
    h, w = img.shape[:2]
    cv2.drawMarker(img, (int(round(cx)), int(round(cy))), (0, 0, 255),
                   cv2.MARKER_CROSS, 40, 3)
    cv2.drawMarker(img, (w // 2, h // 2), (255, 200, 0), cv2.MARKER_TILTED_CROSS,
                   34, 2)
    cv2.arrowedLine(img, (w // 2, h // 2), (int(round(cx)), int(round(cy))),
                    (255, 255, 255), 2, tipLength=0.2)
    cv2.putText(img, "principal point (cx, cy)", (int(cx) + 14, int(cy) - 12),
                FONT, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "image centre (w/2, h/2)", (w // 2 + 14, h // 2 + 26),
                FONT, 0.6, (255, 200, 0), 2, cv2.LINE_AA)
    rays = np.full((h, w, 3), 255, np.uint8)
    for u in range(0, w, max(1, w // 16)):
        cv2.line(rays, (u, 0), (u, h), (230, 230, 230), 1)
    for v in range(0, h, max(1, h // 12)):
        cv2.line(rays, (0, v), (w, v), (230, 230, 230), 1)
    for u in range(0, w, max(1, w // 8)):
        for v in range(0, h, max(1, h // 6)):
            bx, by = (u - cx) / fx, (v - cy) / fy
            cv2.putText(rays, "%.2f,%.2f" % (bx, by), (u + 3, v + 14), FONT,
                        0.35, (90, 90, 90), 1, cv2.LINE_AA)
    cv2.drawMarker(rays, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    off = math.hypot(cx - w / 2.0, cy - h / 2.0)
    hfov = math.degrees(2 * math.atan(w / (2.0 * fx)))
    vfov = math.degrees(2 * math.atan(h / (2.0 * fy)))
    return figure(
        "02  INTRINSICS", [(img, "where the optical axis actually crosses"),
                           (rays, "the bearing (x/z, y/z) each pixel carries")],
        data=D(("fx", round(fx, 4), "px"), ("fy", round(fy, 4), "px"),
               ("cx", round(cx, 4), "px"), ("cy", round(cy, 4), "px"),
               ("image centre u", w / 2.0, "px"),
               ("image centre v", h / 2.0, "px"),
               ("principal point offset from centre", round(off, 2), "px"),
               ("that offset at 1 m range", round(1000 * off / fx, 1), "mm"),
               ("horizontal field of view", round(hfov, 3), "deg"),
               ("vertical field of view", round(vfov, 3), "deg"),
               ("ground sample distance at 1 m", round(1000.0 / fx, 4), "mm/px")),
        formula="bearing = ((u - cx)/fx, (v - cy)/fy);  X = bearing * Z",
        provenance=fr.provenance,
        numbers="fx %.2f  fy %.2f  cx %.2f  cy %.2f | field of view %.1f x "
                "%.1f deg | principal point is %.1f px from the image centre, "
                "which is %.0f mm of sideways error at 1 m if you assume the "
                "centre" % (fx, fy, cx, cy, hfov, vfov, off, 1000 * off / fx),
        badge=("GEOMETRY", ACCENT))


@stage(3, "hsv", "BGR to HSV, the space the colour tests live in")
def st_hsv(fr):
    hsv = hsv_of(fr)
    hue = cv2.cvtColor(np.stack([hsv[:, :, 0],
                                 np.full_like(hsv[:, :, 0], 255),
                                 np.full_like(hsv[:, :, 0], 255)], 2),
                       cv2.COLOR_HSV2BGR)
    return figure(
        "03  HSV DECOMPOSITION",
        [(fr.colour, "BGR in"), (hue, "H, drawn at full S and V"),
         (hsv[:, :, 1], "S -- how far from grey"),
         (hsv[:, :, 2], "V -- how bright")],
        data=D(("median hue", float(np.median(hsv[:, :, 0])), "0-179"),
               ("median saturation", float(np.median(hsv[:, :, 1])), "0-255"),
               ("median value", float(np.median(hsv[:, :, 2])), "0-255"),
               ("pixels below S=80", round(100 * float((hsv[:, :, 1] < 80).mean()), 3), "%"),
               ("pixels below V=40", round(100 * float((hsv[:, :, 2] < 40).mean()), 3), "%"),
               ("pixels above V=200", round(100 * float((hsv[:, :, 2] > 200).mean()), 3), "%")),
        formula="V = max(B,G,R);  S = (V - min(B,G,R))/V;  H = the sector of "
                "the RGB hexagon, 0..179 in OpenCV (not 0..359)",
        provenance=fr.provenance,
        numbers="median H %.0f  S %.0f  V %.0f | %.1f%% of pixels are below "
                "S=80 and cannot satisfy any colour term in this repository"
                % (np.median(hsv[:, :, 0]), np.median(hsv[:, :, 1]),
                   np.median(hsv[:, :, 2]), 100 * float((hsv[:, :, 1] < 80).mean())),
        note="Hue is why colour is done here and not in BGR: brightness moves "
             "B, G and R together and leaves H alone, so a shadow does not "
             "change what colour something is -- until V collapses, which is "
             "what the value floors in each term exist to catch.")


@stage(4, "green_threshold", "The pick path's own green threshold")
def st_threshold(fr):
    raw, _ = green_masks(fr)
    hsv = hsv_of(fr)
    inb = raw > 0
    return figure(
        "04  COLOUR THRESHOLD  (cv2.inRange)",
        [(fr.colour, "in"), (raw, "mask: 255 where every channel is in band"),
         (mask_overlay(fr.colour, inb), "what that selects, over the frame")],
        data=D(("hue low", PICK_GREEN_LO[0], "0-179"),
               ("hue high", PICK_GREEN_HI[0], "0-179"),
               ("saturation floor", PICK_GREEN_LO[1], "0-255"),
               ("value floor", PICK_GREEN_LO[2], "0-255"),
               ("pixels selected", int(inb.sum()), "px"),
               ("fraction of the frame", round(100 * float(inb.mean()), 4), "%"),
               ("median S of the selected",
                float(np.median(hsv[:, :, 1][inb])) if inb.any() else float("nan"), "0-255"),
               ("median V of the selected",
                float(np.median(hsv[:, :, 2][inb])) if inb.any() else float("nan"), "0-255")),
        formula="mask(u,v) = 1 iff  %d <= H <= %d  and  %d <= S <= %d  and  "
                "%d <= V <= %d"
                % (PICK_GREEN_LO[0], PICK_GREEN_HI[0], PICK_GREEN_LO[1],
                   PICK_GREEN_HI[1], PICK_GREEN_LO[2], PICK_GREEN_HI[2]),
        provenance="constants from scripts/servo_pick_left.py:measure -- the "
                   "thresholds the live pick actually runs on",
        numbers="%d px selected (%.3f%% of the frame) | of those, median S %.0f "
                "and median V %.0f"
                % (int(inb.sum()), 100 * float(inb.mean()),
                   float(np.median(hsv[:, :, 1][inb])) if inb.any() else -1,
                   float(np.median(hsv[:, :, 2][inb])) if inb.any() else -1),
        note="A threshold is a hard decision with no confidence attached, "
             "which is why colour is a FALLBACK in this repository and capped "
             "below any AprilTag: it fails by being confidently wrong under "
             "changed light, and nothing downstream can tell.")


@stage(5, "morphology", "What the 9x9 opening removes")
def st_morphology(fr):
    raw, opened = green_masks(fr)
    removed = cv2.bitwise_and(raw, cv2.bitwise_not(opened))
    tri = fr.colour.copy()
    tri = mask_overlay(tri, opened > 0, (0, 220, 0), 0.5, outline=False)
    tri = mask_overlay(tri, removed > 0, (0, 0, 255), 0.9, outline=False)
    return figure(
        "05  MORPHOLOGICAL OPENING",
        [(raw, "before: the raw threshold"),
         (opened, "after: open with a %dx%d box" % (PICK_OPEN_K, PICK_OPEN_K)),
         (tri, "green kept, RED removed")],
        data=D(("kernel", "%dx%d" % (PICK_OPEN_K, PICK_OPEN_K), "px"),
               ("pixels before", int((raw > 0).sum()), "px"),
               ("pixels after", int((opened > 0).sum()), "px"),
               ("pixels removed", int((removed > 0).sum()), "px"),
               ("removed",
                round(100.0 * (removed > 0).sum() / max(int((raw > 0).sum()), 1), 3), "%"),
               ("blobs before", int(cv2.connectedComponents(raw)[0] - 1), "count"),
               ("blobs after", int(cv2.connectedComponents(opened)[0] - 1), "count")),
        formula="open(M) = dilate(erode(M, K), K),  K = ones(%d, %d).  Erode "
                "deletes anything thinner than the kernel; dilate restores "
                "what survived to its original size."
                % (PICK_OPEN_K, PICK_OPEN_K),
        provenance="9x9 is the pick path's kernel (servo_pick_left); "
                   "prompt_detector and colour_shape_detector both use 5x5",
        numbers="%d px in, %d px out, %d px removed (%.1f%%) | %d separate "
                "blobs before, %d after"
                % (int((raw > 0).sum()), int((opened > 0).sum()),
                   int((removed > 0).sum()),
                   100.0 * (removed > 0).sum() / max((raw > 0).sum(), 1),
                   cv2.connectedComponents(raw)[0] - 1,
                   cv2.connectedComponents(opened)[0] - 1),
        note="The kernel size IS a decision about the smallest object that "
             "can survive: at this range a 9 px feature is deleted whatever "
             "colour it is.")


@stage(6, "components", "Connected components, and their statistics")
def st_components(fr):
    lab, rows = components(fr)
    _, opened = green_masks(fr)
    vis = fr.colour.copy()
    rng = np.random.default_rng(7)
    for r in rows[:12]:
        x, y, w, h = r["box"]
        c = tuple(int(v) for v in rng.integers(60, 255, 3))
        cv2.rectangle(vis, (x, y), (x + w, y + h), c, 2)
        cv2.putText(vis, "#%d %d px" % (r["i"], r["area"]), (x, max(14, y - 6)),
                    FONT, 0.5, c, 2, cv2.LINE_AA)
        cv2.drawMarker(vis, (int(r["centroid"][0]), int(r["centroid"][1])), c,
                       cv2.MARKER_CROSS, 12, 2)
    tbl = np.full((max(220, 26 * min(len(rows), 8) + 40), 620, 3), (252, 252, 250),
                  np.uint8)
    cv2.putText(tbl, "  #   area px    box (x,y,w,h)        centroid", (8, 24),
                FONT, 0.5, INK, 1, cv2.LINE_AA)
    for k, r in enumerate(rows[:8]):
        cv2.putText(tbl, "%3d %8d   %-22s %.1f, %.1f"
                    % (r["i"], r["area"], str(r["box"]), r["centroid"][0],
                       r["centroid"][1]),
                    (8, 52 + 26 * k), FONT, 0.46, INK, 1, cv2.LINE_AA)
    if not rows:
        cv2.putText(tbl, "no component survived the 40 px floor", (8, 60),
                    FONT, 0.5, BAD_C, 2, cv2.LINE_AA)
    return figure(
        "06  CONNECTED COMPONENTS",
        [(vis, "each component boxed, with its area"),
         (tbl, "cv2.connectedComponentsWithStats, largest first")],
        data=D(("components over 40 px", len(rows), "count"),
               ("largest area", rows[0]["area"] if rows else 0, "px"),
               ("total area kept", int(sum(r["area"] for r in rows)), "px"),
               ("median area",
                float(np.median([r["area"] for r in rows])) if rows else 0.0, "px")),
        tables={"every component": (
            ["#", "area px", "box x", "box y", "box w", "box h",
             "centroid u", "centroid v"],
            [[r["i"], r["area"], r["box"][0], r["box"][1], r["box"][2],
              r["box"][3], round(r["centroid"][0], 1), round(r["centroid"][1], 1)]
             for r in rows])},
        formula="8-connectivity flood label; per label: area, bounding box, "
                "centroid = (1/A) * sum of member pixel coordinates",
        provenance=fr.provenance,
        numbers="%d components over 40 px | largest %d px | this is where a "
                "MASK becomes a set of candidate OBJECTS"
                % (len(rows), rows[0]["area"] if rows else 0),
        note="A centroid is not an object's centre: it is the centre of the "
             "pixels that happened to pass the threshold, so a partly shadowed "
             "cube reports a centroid shifted into its lit half.")


@stage(7, "vocabulary", "The whole colour vocabulary, swept")
def st_vocabulary(fr):
    try:
        from srl_perception.prompt_detector import COLOUR_TERMS
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("srl_perception.prompt_detector will not import (%s)" % exc)
    hsv = hsv_of(fr)
    panels, counts = [], {}
    for term, bands in COLOUR_TERMS.items():
        m = None
        for lo, hi in bands:
            part = cv2.inRange(hsv, np.array(lo), np.array(hi))
            m = part if m is None else cv2.bitwise_or(m, part)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        counts[term] = int((m > 0).sum())
        panels.append((mask_overlay(fr.colour, m > 0, (0, 220, 220), 0.6,
                                    outline=False),
                       "%s: %s -> %d px"
                       % (term, " or ".join("H %d-%d" % (b[0][0], b[1][0])
                                            for b in bands), counts[term])))
    greens = {
        "servo_pick_left (the live pick)": (PICK_GREEN_LO, PICK_GREEN_HI, 9),
        "prompt_detector.COLOUR_TERMS": ((36, 80, 25), (85, 255, 255), 5),
        "colour_shape_detector.DEFAULT": ((40, 80, 60), (85, 255, 255), 5)}
    cmp_img = fr.colour.copy()
    cols = [(0, 255, 0), (255, 160, 0), (0, 160, 255)]
    lines, gdata = [], []
    for (name, (lo, hi, k)), col in zip(greens.items(), cols):
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cmp_img, cs, -1, col, 2)
        npx = int((m > 0).sum())
        gdata.append([name, lo[0], hi[0], lo[1], lo[2], "%dx%d" % (k, k), npx])
        lines.append("%s H%d-%d S>=%d V>=%d open %d: %d px"
                     % (name, lo[0], hi[0], lo[1], lo[2], k, npx))
    panels.append((cmp_img, "THREE definitions of 'green' in this repository, "
                            "outlined together"))
    return figure(
        "07  THE COLOUR VOCABULARY",
        panels,
        data=D(("colour terms tested", len(counts), "count"),
               ("terms with any pixels", sum(1 for v in counts.values() if v), "count"),
               ("largest term", max(counts, key=lambda k: counts[k]) if counts else "-", "-"),
               ("its pixel count", max(counts.values()) if counts else 0, "px")),
        tables={"pixels per colour term": (
            ["colour term", "pixels", "fraction of frame %"],
            [[k, v, round(100.0 * v / float(fr.colour[:, :, 0].size), 4)]
             for k, v in sorted(counts.items(), key=lambda kv: -kv[1])]),
                "the three definitions of green": (
            ["module", "H low", "H high", "S floor", "V floor", "open kernel",
             "pixels"], gdata)},
        formula="one inRange per band, OR-ed across bands (red needs two: it "
                "wraps the hue circle at 0/179), then a 5x5 opening",
        provenance="srl_perception/prompt_detector.py:COLOUR_TERMS -- the "
                   "colour backend's entire vocabulary, stated so its limit "
                   "is visible rather than discovered",
        numbers=" | ".join("%s %d" % (k, v) for k, v in
                           sorted(counts.items(), key=lambda kv: -kv[1])[:6]),
        note="THE THREE GREENS ARE NOT THE SAME: " + "; ".join(lines) +
             ". They differ in the value floor (25 / 40 / 60) and in the "
             "opening kernel, so the same cube in the same frame gives three "
             "different pixel counts depending on which module is asking.",
        cols=3)


@stage(8, "region_mean", "The region-MEAN ratio test, and its near-black guard")
def st_region_mean(fr):
    try:
        from srl_perception.prompt_detector import _colour_matches
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("prompt_detector will not import (%s)" % exc)
    lab, rows = components(fr)
    if not rows:
        raise Refusal("no colour region survived the threshold and the "
                      "opening, so there is nothing to take a mean over. That "
                      "is a statement about THIS frame, not a failure of the "
                      "test.")
    vis = fr.colour.copy()
    swatch = np.full((max(240, 34 * min(len(rows), 8) + 50), 760, 3),
                     (252, 252, 250), np.uint8)
    cv2.putText(swatch, "region   mean B,G,R     max<25?   green ratio test",
                (10, 24), FONT, 0.48, INK, 1, cv2.LINE_AA)
    recs = []
    for k, r in enumerate(rows[:8]):
        m = (lab == r["i"]).astype(np.uint8)
        b, g, rr = [float(v) for v in cv2.mean(fr.colour, m)[:3]]
        dark = max(b, g, rr) < 25.0
        ok = bool(_colour_matches("green", b, g, rr))
        recs.append(dict(region=r["i"], area=r["area"],
                         mean_bgr=[round(b, 1), round(g, 1), round(rr, 1)],
                         near_black=dark, passes_green=ok))
        col = OK_C if ok else BAD_C
        x, y, w, h = r["box"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
        cv2.putText(vis, "PASS" if ok else "reject", (x, max(14, y - 6)), FONT,
                    0.5, col, 2, cv2.LINE_AA)
        y0 = 44 + 34 * k
        cv2.rectangle(swatch, (10, y0), (60, y0 + 26),
                      (int(b), int(g), int(rr)), -1)
        cv2.rectangle(swatch, (10, y0), (60, y0 + 26), RULE, 1)
        cv2.putText(swatch, "#%-4d  %5.1f %5.1f %5.1f     %-6s   %s"
                    % (r["i"], b, g, rr, "YES" if dark else "no",
                       "g>b*1.25 and g>r*1.8  ->  %s" % ("PASS" if ok else "reject")),
                    (70, y0 + 19), FONT, 0.45, col if ok else INK, 1, cv2.LINE_AA)
    return figure(
        "08  REGION MEAN, NOT PER-PIXEL HUE",
        [(vis, "verdict per region"), (swatch, "the means and the test")],
        data=D(("regions tested", len(recs), "count"),
               ("regions passing", sum(1 for r in recs if r["passes_green"]), "count"),
               ("regions rejected as near-black",
                sum(1 for r in recs if r["near_black"]), "count"),
               ("blue ratio threshold", 1.25, "-"),
               ("red ratio threshold", 1.8, "-"),
               ("near-black floor", 25.0, "0-255")),
        tables={"region means and verdicts": (
            ["region", "area px", "mean B", "mean G", "mean R", "near black",
             "passes green"],
            [[r["region"], r["area"], r["mean_bgr"][0], r["mean_bgr"][1],
              r["mean_bgr"][2], r["near_black"], r["passes_green"]]
             for r in recs])},
        formula="green  <=>  mean_G > 1.25 * mean_B  and  mean_G > 1.8 * "
                "mean_R,  refused outright when max(B,G,R) < 25",
        provenance="srl_perception/prompt_detector.py:_colour_matches",
        numbers="%d of %d regions pass" % (sum(r["passes_green"] for r in recs),
                                           len(recs)),
        note="Per-pixel hue picked the room's teal robots three times: their "
             "shadowed hue is 76 against the target cube's 74, two apart and "
             "inside noise. Region MEANS separate them, because teal carries "
             "far more blue. The near-black guard exists because a region "
             "measuring (4.1, 8.8, 3.9) -- visually black -- satisfies the "
             "ratios on sensor noise alone and was reported as the cube.")


@stage(9, "minarearect", "Rotated box, in-image yaw, and the degeneracy gate")
def st_minarearect(fr):
    _, opened = green_masks(fr)
    cnts, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= 400]
    if not cnts:
        raise Refusal("no contour over the 400 px floor in this frame, so "
                      "there is no rotated box to fit")
    vis = fr.colour.copy()
    recs = []
    MIN_ASPECT = 1.15
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:8]:
        (cx, cy), (w, h), ang = cv2.minAreaRect(c)
        box = cv2.boxPoints(((cx, cy), (w, h), ang)).astype(np.int32)
        lo_ax, hi_ax = min(w, h), max(w, h)
        aspect = hi_ax / lo_ax if lo_ax > 0 else 0.0
        known = aspect >= MIN_ASPECT
        col = OK_C if known else (150, 110, 40)
        cv2.drawContours(vis, [box], 0, col, 2)
        cv2.putText(vis, "%.1f deg%s" % (ang, "" if known else "  NOT PUBLISHED"),
                    (int(cx) - 40, int(cy) - 10), FONT, 0.5, col, 2, cv2.LINE_AA)
        if known:
            th = math.radians(ang)
            L = hi_ax / 2.0
            cv2.arrowedLine(vis, (int(cx), int(cy)),
                            (int(cx + L * math.cos(th)), int(cy + L * math.sin(th))),
                            col, 2, tipLength=0.2)
        recs.append(dict(area=float(cv2.contourArea(c)), w=round(w, 1),
                         h=round(h, 1), angle_deg=round(float(ang), 1),
                         aspect=round(float(aspect), 3), yaw_published=known))
    return figure(
        "09  ROTATED BOX AND YAW",
        [(mask_overlay(fr.colour, opened > 0, (0, 200, 0), 0.35), "the mask"),
         (vis, "minAreaRect, with the angle it measured")],
        data=D(("contours measured", len(recs), "count"),
               ("aspect gate for publishing yaw", MIN_ASPECT, "-"),
               ("yaws published", sum(1 for r in recs if r["yaw_published"]), "count"),
               ("yaws withheld as degenerate",
                sum(1 for r in recs if not r["yaw_published"]), "count")),
        tables={"rotated boxes": (
            ["area px", "w px", "h px", "angle deg", "aspect", "yaw published"],
            [[round(r["area"], 1), r["w"], r["h"], r["angle_deg"], r["aspect"],
              r["yaw_published"]] for r in recs])},
        formula="minAreaRect -> ((cx,cy), (w,h), angle);  yaw is published as "
                "q = (0, 0, sin(angle/2), cos(angle/2)) ONLY when "
                "max(w,h)/min(w,h) >= %.2f" % MIN_ASPECT,
        provenance="srl_perception/colour_shape_detector.py",
        numbers=" | ".join("%.0f x %.0f px, %.1f deg, aspect %.2f, %s"
                           % (r["w"], r["h"], r["angle_deg"], r["aspect"],
                              "yaw published" if r["yaw_published"] else
                              "identity (not measured)")
                           for r in recs[:4]),
        note="Rotation about the OPTICAL AXIS is exactly what this measures; "
             "out-of-plane tilt is what it cannot see. The detector published "
             "identity for months while measuring the angle into a debug "
             "string, so an object at 30 deg got a square grasp and no check "
             "could tell that from a square object. A near-square blob still "
             "gets identity, because minAreaRect's angle is degenerate there "
             "and publishing it would be inventing a direction.")


@stage(10, "size_at_range", "The size-at-range gate")
def st_size_at_range(fr):
    fx, fy, cx, cy = need_K(fr)
    _, opened = green_masks(fr)
    cnts, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= 400]
    if not cnts:
        raise Refusal("no blob over 400 px to test")
    OBJ_M = 0.040
    WIN = (0.25, 1.50)
    vis = fr.colour.copy()
    recs = []
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:8]:
        (bx, by), (w, h), ang = cv2.minAreaRect(c)
        px = max(w, h)
        if px < 5:
            continue
        z = fx * OBJ_M / px
        inside = WIN[0] <= z <= WIN[1]
        col = OK_C if inside else BAD_C
        box = cv2.boxPoints(((bx, by), (w, h), ang)).astype(np.int32)
        cv2.drawContours(vis, [box], 0, col, 2)
        cv2.putText(vis, "%.0f px -> z=%.3f m %s"
                    % (px, z, "OK" if inside else "REJECT"),
                    (int(bx) - 90, int(by) + int(max(w, h) / 2) + 22), FONT,
                    0.5, col, 2, cv2.LINE_AA)
        recs.append(dict(px=round(float(px), 1), implied_range_m=round(z, 4),
                         accepted=bool(inside)))
    zs = np.linspace(0.15, 2.0, 400)
    pxs = fx * OBJ_M / zs
    curve = line_chart(
        zs, pxs, title="apparent size of a %.0f mm object against range"
                       % (OBJ_M * 1000),
        xlabel="range (m)", ylabel="px across", y_from_zero=True,
        vlines=[(WIN[0], "near %.2f m" % WIN[0], (120, 120, 120)),
                (WIN[1], "far %.2f m" % WIN[1], (120, 120, 120))],
        points=[(r["implied_range_m"], r["px"],
                 OK_C if r["accepted"] else BAD_C,
                 "%.0f px" % r["px"]) for r in recs[:6]
                if 0.15 <= r["implied_range_m"] <= 2.0])
    return figure(
        "10  SIZE AT RANGE",
        [(vis, "each blob, with the range its size implies"),
         (curve, "px = fx * size / range;  the working window in grey")],
        data=D(("fx used", round(fx, 3), "px"),
               ("assumed object width", OBJ_M * 1000, "mm"),
               ("near limit of the window", WIN[0], "m"),
               ("far limit of the window", WIN[1], "m"),
               ("blobs tested", len(recs), "count"),
               ("blobs accepted", sum(1 for r in recs if r["accepted"]), "count"),
               ("blobs rejected", sum(1 for r in recs if not r["accepted"]), "count")),
        tables={"implied range per blob": (
            ["px across", "implied range m", "accepted"],
            [[r["px"], r["implied_range_m"], r["accepted"]] for r in recs])},
        formula="z = fx * object_width / px_across;  accept iff %.2f <= z <= "
                "%.2f m.  With depth the honest form is used instead: compare "
                "px against fx * size / measured_depth, tolerance 0.55 to 1.80"
                % WIN,
        provenance="srl_perception/colour_shape_detector.py -- expected_px() "
                   "and size_at_range_ok()",
        numbers=" | ".join("%.0f px -> %.3f m %s"
                           % (r["px"], r["implied_range_m"],
                              "accept" if r["accepted"] else "REJECT")
                           for r in recs[:6]),
        note="This gate was added after a measured failure: T1's coloured "
             "pads are 210 x 130 mm in exactly the cube colours, and with a "
             "minimum-area floor and no upper bound the detector locked onto "
             "the pads and scored 0 of 4 cubes. A 210 mm pad read as a 40 mm "
             "cube implies z = 0.134 m -- 'a cube 134 mm from the lens' -- "
             "which is outside anything the arm works in, so the pad "
             "disappears while a real cube at 0.5 m passes untouched.")


@stage(11, "fastsam", "FastSAM: every region in the picture, no prompt")
def st_fastsam(fr):
    ms = sam_masks(fr)
    over = colourise_labels(fr.colour.shape, ms, alpha=0.55, base=fr.colour)
    flat = colourise_labels(fr.colour.shape, ms, alpha=1.0)
    edges = fr.colour.copy()
    for m in ms:
        cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(edges, cs, -1, (255, 255, 255), 1)
    areas = sorted((int(m.sum()) for m in ms), reverse=True)
    cfg = fr.cache.get("sam_cfg", {})
    return figure(
        "11  SEGMENTATION  (FastSAM-s, CPU)",
        [(fr.colour, "in"), (over, "%d regions, over the frame" % len(ms)),
         (flat, "the regions alone"), (edges, "their boundaries")],
        data=D(("regions returned", len(ms), "count"),
               ("largest region", areas[0], "px"),
               ("largest as a fraction of the frame",
                round(100.0 * areas[0] / float(fr.colour[:, :, 0].size), 2), "%"),
               ("smallest region", areas[-1], "px"),
               ("median region", float(np.median(areas)), "px"),
               ("inference time", round(fr.cache.get("sam_s", float("nan")), 2), "s"),
               ("input size", cfg.get("imgsz"), "px"),
               ("confidence threshold", cfg.get("conf"), "-"),
               ("NMS IoU", cfg.get("iou"), "-"),
               ("device", cfg.get("device"), "-")),
        tables={"the twenty largest regions": (
            ["rank", "area px", "fraction of frame %"],
            [[i + 1, a, round(100.0 * a / float(fr.colour[:, :, 0].size), 3)]
             for i, a in enumerate(areas[:20])])},
        formula="object-agnostic instance masks; no class, no prompt, no "
                "scene knowledge -- 'what are the regions', not 'where is the "
                "green cube'",
        provenance="FastSAM-s.pt (~11M parameters) via ultralytics, "
                   "imgsz=%s conf=%s iou=%s on the %s, %.1f s for this frame"
                   % (cfg.get("imgsz"), cfg.get("conf"), cfg.get("iou"),
                      cfg.get("device"), fr.cache.get("sam_s", float("nan"))),
        numbers="%d regions | largest %d px (%.1f%% of frame), smallest %d px"
                % (len(ms), areas[0], 100.0 * areas[0] / fr.colour[:, :, 0].size,
                   areas[-1]),
        note="This is what replaced clustering. Two 40 mm cubes on a 60 mm "
             "pitch are 20 mm apart, which is the cluster distance itself, so "
             "depth-clustering fused them into one object 140 mm across the "
             "jaws and refused it. In the PICTURE they are not remotely "
             "ambiguous. Masks decide WHAT is an object; depth decides WHERE "
             "it is.")


@stage(12, "area_filter", "The area filter: noise below, the table above")
def st_area_filter(fr):
    ms = sam_masks(fr)
    H, W = fr.colour.shape[:2]
    MIN_A, MAX_F = 60, 0.35
    keep = [m for m in ms if MIN_A <= m.sum() <= MAX_F * H * W]
    small = [m for m in ms if m.sum() < MIN_A]
    big = [m for m in ms if m.sum() > MAX_F * H * W]
    return figure(
        "12  AREA FILTER",
        [(colourise_labels(fr.colour.shape, keep, 0.6, fr.colour),
          "kept: %d regions" % len(keep)),
         (colourise_labels(fr.colour.shape, big, 0.6, fr.colour),
          "rejected as TOO LARGE: %d (the table, a wall, the arm itself)"
          % len(big)),
         (colourise_labels(fr.colour.shape, small, 0.9, fr.colour),
          "rejected as noise: %d" % len(small))],
        data=D(("regions in", len(ms), "count"),
               ("kept", len(keep), "count"),
               ("rejected as noise", len(small), "count"),
               ("rejected as too large", len(big), "count"),
               ("lower bound", MIN_A, "px"),
               ("upper bound", round(MAX_F * H * W), "px"),
               ("upper bound as a fraction", MAX_F, "-")),
        formula="keep iff  %d px <= area <= %.2f * W * H  (= %d px here)"
                % (MIN_A, MAX_F, int(MAX_F * H * W)),
        provenance="srl_perception/segment_lift.py: MIN_AREA_PX, MAX_AREA_FRAC",
        numbers="%d in, %d kept, %d too small, %d too large"
                % (len(ms), len(keep), len(small), len(big)),
        note="The upper bound is not tidiness. A segmenter returns the table "
             "and the room as legitimate regions, and a region covering a "
             "third of the frame is never a thing to be picked up.")


@stage(13, "segment_match", "Segment everything, then pick the one that matches")
def st_segment_match(fr, prompt="the green cube"):
    try:
        from srl_perception.prompt_detector import _colour_matches, parse_prompt
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("prompt_detector will not import (%s)" % exc)
    ms = sam_masks(fr)
    H, W = fr.colour.shape[:2]
    cols, shapes, rest = parse_prompt(prompt)
    term = cols[0] if cols else None
    if term is None:
        raise Refusal("the prompt %r carries no colour word, so the colour "
                      "gate has nothing to test. Known words: see stage 07."
                      % prompt)
    kept, rejected, recs = [], [], []
    for m in ms:
        a = int(m.sum())
        if a < 40 or a > 0.35 * H * W:
            continue
        b, g, r = [float(v) for v in cv2.mean(fr.colour, m.astype(np.uint8))[:3]]
        if _colour_matches(term, b, g, r):
            kept.append(m)
            ys, xs = np.nonzero(m)
            recs.append(dict(area=a, mean_bgr=[round(b, 1), round(g, 1),
                                               round(r, 1)],
                             centre_uv=[round(float(xs.mean()), 1),
                                        round(float(ys.mean()), 1)],
                             score=round(min(1.0, a / 2000.0), 3)))
        else:
            rejected.append(m)
    vis = fr.colour.copy()
    for m, rec in zip(kept, recs):
        ys, xs = np.nonzero(m)
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 220, 0), 2)
        cv2.putText(vis, "%s  %.2f" % (term, rec["score"]), (x0, max(14, y0 - 6)),
                    FONT, 0.55, (0, 220, 0), 2, cv2.LINE_AA)
        cv2.drawMarker(vis, (int(rec["centre_uv"][0]), int(rec["centre_uv"][1])),
                       (0, 220, 0), cv2.MARKER_CROSS, 18, 2)
    if not kept:
        raise Refusal(
            "FastSAM returned %d regions and NONE of them is %s-dominant by "
            "the region-mean test. That is a real 'not found' for this "
            "prompt in this frame -- the segmenter ran, the gate ran, and "
            "nothing matched." % (len(ms), term))
    return figure(
        "13  SEGMENT, THEN MATCH   prompt: %r" % prompt,
        [(colourise_labels(fr.colour.shape, rejected, 0.45, fr.colour),
          "%d regions rejected on colour" % len(rejected)),
         (mask_overlay(fr.colour, np.any(kept, axis=0), (0, 220, 0), 0.5),
          "%d region(s) matched %r" % (len(kept), term)),
         (vis, "what the grasp planner is handed")],
        data=D(("colour word parsed", term, "-"),
               ("regions considered", len(kept) + len(rejected), "count"),
               ("regions matched", len(kept), "count"),
               ("regions rejected on colour", len(rejected), "count"),
               ("best score", max((r["score"] for r in recs), default=0.0), "0-1")),
        tables={"matched regions": (
            ["area px", "centre u", "centre v", "mean B", "mean G", "mean R",
             "score"],
            [[r["area"], r["centre_uv"][0], r["centre_uv"][1], r["mean_bgr"][0],
              r["mean_bgr"][1], r["mean_bgr"][2], r["score"]] for r in recs])},
        formula="for each region: mean BGR over the MASK, then the ratio test "
                "of stage 08; score = min(1, area/2000)",
        provenance="srl_perception/prompt_detector.py:_segment -- the backend "
                   "that actually found the target on this rig",
        numbers=" | ".join("%d px at (%.0f, %.0f), mean BGR %s"
                           % (r["area"], r["centre_uv"][0], r["centre_uv"][1],
                              r["mean_bgr"]) for r in recs[:5]),
        note="Naming the cube failed at every confidence -- it is ~19 px and "
             "dark, and an open-vocabulary detector cannot recognise it as "
             "anything. Segmenting it is trivial: 89 regions, exactly one "
             "green-dominant, no false positives. The MASK is the other win: "
             "a colour threshold gives whatever pixels passed, a segment "
             "gives the object's actual extent, which is what a width "
             "measurement needs.")


@stage(14, "yoloworld", "YOLO-World: open-vocabulary naming")
def st_yoloworld(fr, prompt="the green cube"):
    try:
        from ultralytics import YOLOWorld
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("ultralytics is not importable (%s); run under "
                      ".venv_vision/bin/python" % exc)
    w = os.path.join(WS, "yolov8s-world.pt")
    if not os.path.exists(w):
        raise Refusal("no yolov8s-world.pt in %s" % WS)
    from srl_perception.prompt_detector import parse_prompt
    cols, shapes, rest = parse_prompt(prompt)
    classes = [" ".join(x for x in [c, s] if x)
               for c in (cols or [""]) for s in (shapes or rest or ["object"])]
    classes = [c.strip() for c in classes if c.strip()] or ["object"]
    t0 = time.time()
    model = YOLOWorld(w)
    model.set_classes(classes)
    res = model.predict(fr.colour, conf=0.05, verbose=False)
    dt = time.time() - t0
    vis = fr.colour.copy()
    recs = []
    for r in res:
        for b in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            lab = classes[int(b.cls[0])] if int(b.cls[0]) < len(classes) else "object"
            sc = float(b.conf[0])
            recs.append(dict(label=lab, score=round(sc, 3),
                             bbox=[round(v, 1) for v in (x1, y1, x2 - x1, y2 - y1)]))
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 160, 255), 2)
            cv2.putText(vis, "%s %.2f" % (lab, sc), (int(x1), max(14, int(y1) - 6)),
                        FONT, 0.55, (0, 160, 255), 2, cv2.LINE_AA)
    if not recs:
        raise Refusal(
            "YOLO-World ran in %.1f s over classes %s at conf 0.05 and found "
            "NOTHING. This is the measured behaviour on this rig, not a "
            "misconfiguration: it finds a person at 0.725 and a laptop at "
            "0.739 in the same room and cannot see the small dark cube at "
            "any confidence. It is why the backend order is segment first, "
            "naming second, colour as the floor." % (dt, classes))
    return figure(
        "14  OPEN-VOCABULARY DETECTION",
        [(vis, "boxes and scores")],
        data=D(("classes asked for", ", ".join(classes), "-"),
               ("confidence threshold", 0.05, "-"),
               ("detections", len(recs), "count"),
               ("best score", max((r["score"] for r in recs), default=0.0), "0-1"),
               ("inference time", round(dt, 2), "s")),
        tables={"detections": (
            ["label", "score", "x", "y", "w", "h"],
            [[r["label"], r["score"]] + r["bbox"] for r in recs])},
        formula="text prompt -> class embeddings -> boxes; classes asked for: "
                "%s" % ", ".join(classes),
        provenance="yolov8s-world.pt via ultralytics, ~60M parameters, CPU, "
                   "%.1f s. Chosen over GroundingDINO (218M) and OWLv2 (428M) "
                   "for one measured reason: this machine has 4 GB of VRAM "
                   "shared with everything else." % dt,
        numbers=" | ".join("%s %.2f" % (r["label"], r["score"])
                           for r in recs[:6]),
        cols=1, panel_w=900, panel_h=620)


# -------------------------------------------------------------- depth helpers

def aligned_depth(fr):
    """Depth expressed in the COLOUR frame, so a mask can index it."""
    if "aligned" in fr.cache:
        return fr.cache["aligned"]
    d = need_depth(fr)
    if fr.depth_is_aligned and d.shape[:2] == fr.colour.shape[:2]:
        fr.cache["aligned"] = (d, "aligned by librealsense (rs.align to "
                                  "colour); no re-projection done here", {})
        return fr.cache["aligned"]
    need_K(fr)
    ratio = fr.colour.shape[1] / float(max(d.shape[1], 1))
    splat = max(0, int(round(ratio / 2.0)))
    a, stats = align_depth_to_colour(d, fr.K_d, fr.K_c, fr.size,
                                     KINOVA_DEPTH_TO_COLOR_M, splat=splat)
    fr.cache["aligned"] = (
        a, "forward-projected here: deproject with the depth intrinsics, "
           "translate by the recorded %.1f, %.1f, %.1f mm baseline, project "
           "with the colour intrinsics, nearest point wins"
        % tuple(1000 * v for v in KINOVA_DEPTH_TO_COLOR_M), stats)
    return fr.cache["aligned"]


def depth_cloud(fr, max_points=80000):
    """The cloud in the DEPTH sensor's own frame, as the pick path builds it."""
    if "cloud" not in fr.cache:
        d = need_depth(fr)
        K = fr.K_d or need_K(fr)
        P, uv = deproject(d, K, z_min=fr.band[0], z_max=fr.band[1])
        if len(P) < 500:
            valid = np.isfinite(d) & (d > 0)
            raise Refusal(
                "only %d depth returns fall inside this camera's working band "
                "of %.2f to %.2f m, out of %d returns whose median range is "
                "%.2f m. Either the camera is not looking at anything in that "
                "band, or the band is wrong for this camera."
                % (len(P), fr.band[0], fr.band[1], int(valid.sum()),
                   float(np.median(d[valid])) if valid.any() else float("nan")))
        fr.cache["cloud"] = (P, uv)
    return fr.cache["cloud"]


def colour_cloud(fr):
    """The cloud in the COLOUR frame, so FastSAM masks can select from it."""
    if "ccloud" not in fr.cache:
        a, _, _ = aligned_depth(fr)
        K = need_K(fr)
        P, uv = deproject(a, K, z_min=fr.band[0], z_max=fr.band[1])
        if len(P) < 500:
            raise Refusal("only %d aligned depth points; nothing to lift the "
                          "masks through" % len(P))
        idx = np.full(fr.colour.shape[:2], -1, np.int64)
        idx[uv[:, 1], uv[:, 0]] = np.arange(len(P))
        fr.cache["ccloud"] = (P, uv, idx)
    return fr.cache["ccloud"]


def _fit(P, seed=0, max_fit=60000):
    fitter, prov = _plane_fitter()
    Q = P
    if len(P) > max_fit:
        Q = P[np.random.default_rng(seed).choice(len(P), max_fit, replace=False)]
    pl = fitter(Q, seed=seed)
    n, d = np.asarray(pl["n"], float), float(pl["d"])
    if d < 0:                       # camera on the positive side, as the pick
        n, d = -n, -d               # path forces before measuring heights
    pl["n"], pl["d"] = n, d
    pl["h_all"] = P @ n + d
    pl["inliers"] = np.abs(pl["h_all"]) < pl["tol"]
    pl["rms_mm"] = float(np.sqrt((pl["h_all"][pl["inliers"]] ** 2).mean()) * 1000) \
        if pl["inliers"].any() else float("nan")
    pl["inlier_frac"] = float(pl["inliers"].mean())
    pl["fit_points"] = len(Q)
    pl["provenance"] = prov
    return pl


def plane_depth(fr):
    if "plane_d" not in fr.cache:
        P, _ = depth_cloud(fr)
        fr.cache["plane_d"] = _fit(P)
    return fr.cache["plane_d"]


def plane_colour(fr):
    if "plane_c" not in fr.cache:
        P, _, _ = colour_cloud(fr)
        fr.cache["plane_c"] = _fit(P)
    return fr.cache["plane_c"]


def instances(fr, min_points=25, min_height_m=0.004):
    """FastSAM masks lifted through the aligned depth: one cloud per region."""
    if "inst" in fr.cache:
        return fr.cache["inst"]
    ms = sam_masks(fr)
    P, uv, idx = colour_cloud(fr)
    pl = plane_colour(fr)
    H, W = fr.colour.shape[:2]
    h_all = pl["h_all"]
    out = []
    for m in ms:
        a = int(m.sum())
        if a < 60 or a > 0.35 * H * W:
            continue
        er = cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8),
                       iterations=1).astype(bool)
        use = er if er.sum() >= min_points else m
        sel = idx[use]
        sel = sel[sel >= 0]
        if len(sel) < min_points:
            continue
        h = h_all[sel]
        above = h >= min_height_m
        if int(above.sum()) < min_points:
            continue
        sel_a = sel[above]
        pts = P[sel_a]
        b, g, r = [float(v) for v in cv2.mean(fr.colour, use.astype(np.uint8))[:3]]
        out.append(dict(mask=m, used=use, pts=pts, sel=sel_a, area_px=a,
                        mean_bgr=[b, g, r], heights=h_all[sel_a],
                        n_raw=len(sel), n_above=int(above.sum()),
                        eroded_away=int(m.sum() - use.sum())))
    if not out:
        raise Refusal(
            "no segmented region kept %d lifted points above the fitted plane. "
            "Either nothing is standing on the surface in view, or the depth "
            "has no return where the objects are." % min_points)
    out.sort(key=lambda o: -len(o["pts"]))
    fr.cache["inst"] = out
    return out


def _measure_instance(pts, up):
    """(centre, [long, short, height], width) -- segment_lift._measure's method."""
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ up)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = tmp - up * float(tmp @ up)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    c = pts.mean(axis=0)
    rel = pts - c
    flat2 = np.stack([rel @ e1, rel @ e2], axis=1)
    short_m, ndir, _ = min_width(flat2)
    ldir = np.array([-ndir[1], ndir[0]])
    long_m = float(np.ptp(flat2 @ ldir))
    height_m = float(np.ptp(rel @ up))
    a_n, a_l = flat2 @ ndir, flat2 @ ldir
    mid2 = (ndir * float((a_n.max() + a_n.min()) / 2.0)
            + ldir * float((a_l.max() + a_l.min()) / 2.0))
    mid = c + e1 * float(mid2[0]) + e2 * float(mid2[1])
    return mid, [long_m, short_m, height_m], short_m


def _proj(pts, K):
    fx, fy, cx, cy = K
    z = np.maximum(pts[:, 2], 1e-6)
    return np.stack([pts[:, 0] * fx / z + cx, pts[:, 1] * fy / z + cy], 1)


# -------------------------------------------------------------- DEPTH  stages

@stage(15, "raw_depth", "The depth frame, and where it has no answer")
def st_raw_depth(fr):
    d = need_depth(fr)
    v = np.isfinite(d) & (d > 0)
    holes = (~v).astype(np.uint8) * 255
    vals = d[v]
    hist_counts, edges = np.histogram(vals, bins=120)
    chart = bar_chart(hist_counts, edges, title="depth histogram",
                      xlabel="range (m)")
    return figure(
        "15  RAW DEPTH",
        [(colourise_depth(d), "depth, turbo; magenta = NO RETURN"),
         (holes, "the holes alone"), (chart, "how the returns are distributed")],
        data=D(("depth width", d.shape[1], "px"), ("depth height", d.shape[0], "px"),
               ("pixels with a return", int(v.sum()), "px"),
               ("that fraction", round(100 * float(v.mean()), 2), "%"),
               ("median range", round(float(np.median(vals)), 4), "m"),
               ("5th percentile", round(float(np.percentile(vals, 5)), 4), "m"),
               ("95th percentile", round(float(np.percentile(vals, 95)), 4), "m"),
               ("nearest return", round(float(vals.min()), 4), "m"),
               ("furthest return", round(float(vals.max()), 4), "m"),
               ("returns inside this camera's working band",
                int(((vals > fr.band[0]) & (vals < fr.band[1])).sum()), "px"),
               ("band near", fr.band[0], "m"), ("band far", fr.band[1], "m")),
        formula="one 16-bit value per pixel, in MILLIMETRES on the wire, "
                "divided by 1000 here. 0 means 'no return' and is NOT a "
                "distance of zero.",
        provenance=fr.provenance,
        numbers="%d x %d | %.1f%% of pixels have a return | median %.3f m, "
                "5th-95th %.3f-%.3f m | %d px between the %.2f and %.2f m "
                "gate the pick path uses"
                % (d.shape[1], d.shape[0], 100 * float(v.mean()),
                   float(np.median(vals)), float(np.percentile(vals, 5)),
                   float(np.percentile(vals, 95)),
                   int(((vals > fr.band[0]) & (vals < fr.band[1])).sum()),
                   fr.band[0], fr.band[1]),
        note="A hole is not a far surface. Averaging a hole in as 0 drags "
             "every edge pixel toward the camera, which is why the RealSense "
             "average is taken over VALID pixels only.",
        badge=("DEPTH", ACCENT))


@stage(16, "alignment", "Depth put into the colour frame")
def st_alignment(fr):
    a, how, stats = aligned_depth(fr)
    edges = cv2.Canny(cv2.cvtColor(fr.colour, cv2.COLOR_BGR2GRAY), 60, 160)
    over = colourise_depth(a)
    over[edges > 0] = (255, 255, 255)
    return figure(
        "16  DEPTH ALIGNED TO COLOUR",
        [(colourise_depth(need_depth(fr)), "depth, in the depth sensor's own "
                                           "frame and resolution"),
         (colourise_depth(a), "the same depth, in the colour frame"),
         (over, "colour EDGES drawn on the aligned depth -- they should sit "
                "on the depth discontinuities")],
        data=D(("points projected", stats.get("projected", 0), "count"),
               ("points falling outside the colour frame",
                stats.get("dropped", 0), "count"),
               ("colour pixels given a depth",
                round(100 * stats.get("filled_frac", 0.0), 2), "%"),
               ("splat radius", stats.get("splat_radius_px", 0), "px"),
               ("baseline x", KINOVA_DEPTH_TO_COLOR_M[0] * 1000, "mm"),
               ("baseline y", KINOVA_DEPTH_TO_COLOR_M[1] * 1000, "mm"),
               ("baseline z", KINOVA_DEPTH_TO_COLOR_M[2] * 1000, "mm"),
               ("rotation assumed", "identity", "-")) if stats else
             D(("re-projection needed", False, "-"),
               ("aligner", "librealsense rs.align", "-")),
        formula="P = ((u-cx_d)z/fx_d, (v-cy_d)z/fy_d, z);  P' = P + t;  "
                "u' = fx_c X'/Z' + cx_c,  v' = fy_c Y'/Z' + cy_c",
        provenance=how,
        numbers=("%d points projected, %d fell outside the colour frame, "
                 "%.1f%% of colour pixels got a depth"
                 % (stats.get("projected", 0), stats.get("dropped", 0),
                    100 * stats.get("filled_frac", 0.0))) if stats else
                "no re-projection was needed",
        note="The rotation between the two sensors is taken as IDENTITY here, "
             "because a recorded translation is all this program has. The "
             "live pick path uses the URDF FK between camera_depth_frame and "
             "camera_color_frame instead -- and that extrinsic is known to be "
             "~8.5 deg wrong and to rotate with the wrist, which is why the "
             "pick servos in the camera frame rather than planning through it.")


@stage(17, "deprojection", "Pixels and depth become a cloud")
def st_deprojection(fr):
    P, uv = depth_cloud(fr)
    K = fr.K_d
    sub = P[::max(1, len(P) // 60000)]
    top = scatter(sub[:, 0], sub[:, 2], xlabel="X right (m)",
                  ylabel="Z forward (m)", title="looking down the camera's Y")
    side = scatter(sub[:, 2], -sub[:, 1], xlabel="Z forward (m)",
                   ylabel="-Y up (m)", title="looking along the camera's X")
    return figure(
        "17  DEPROJECTION",
        [(colourise_depth(need_depth(fr)), "the depth image"),
         (top, "the cloud from above"), (side, "the cloud from the side")],
        data=D(("points lifted", len(P), "count"),
               ("fx", round(K[0], 3), "px"), ("fy", round(K[1], 3), "px"),
               ("cx", round(K[2], 3), "px"), ("cy", round(K[3], 3), "px"),
               ("X minimum", round(float(P[:, 0].min()), 4), "m"),
               ("X maximum", round(float(P[:, 0].max()), 4), "m"),
               ("Y minimum", round(float(P[:, 1].min()), 4), "m"),
               ("Y maximum", round(float(P[:, 1].max()), 4), "m"),
               ("Z minimum", round(float(P[:, 2].min()), 4), "m"),
               ("Z maximum", round(float(P[:, 2].max()), 4), "m"),
               ("centroid X", round(float(P[:, 0].mean()), 4), "m"),
               ("centroid Y", round(float(P[:, 1].mean()), 4), "m"),
               ("centroid Z", round(float(P[:, 2].mean()), 4), "m")),
        formula="X = (u - cx) Z / fx,   Y = (v - cy) Z / fy,   Z = depth",
        provenance="rgbd_grasp.deproject / servo_pick_left.measure, with "
                   "fx %.2f fy %.2f cx %.2f cy %.2f" % K,
        numbers="%d points in the %.2f-%.2f m band | extent X %.3f to %.3f, "
                "Y %.3f to %.3f, Z %.3f to %.3f m"
                % (len(P), fr.band[0], fr.band[1], P[:, 0].min(), P[:, 0].max(),
                   P[:, 1].min(), P[:, 1].max(), P[:, 2].min(), P[:, 2].max()),
        note="This is the whole of 'depth decides WHERE'. Everything after it "
             "is geometry on these points, in the CAMERA's frame -- no robot "
             "transform is involved yet, which is exactly why the pick "
             "measures its error here.")


@stage(18, "ransac", "RANSAC: the support plane")
def st_ransac(fr):
    pl = plane_depth(fr)
    P, uv = depth_cloud(fr)
    d = need_depth(fr)
    vis = np.zeros(d.shape + (3,), np.uint8)
    vis[:] = (40, 40, 40)
    vis[uv[pl["inliers"], 1], uv[pl["inliers"], 0]] = (60, 220, 60)
    vis[uv[~pl["inliers"], 1], uv[~pl["inliers"], 0]] = (60, 60, 230)
    sub = slice(None, None, max(1, len(P) // 60000))
    hh = pl["h_all"][sub]
    counts, edges = np.histogram(np.clip(hh, -0.05, 0.15), bins=160)
    chart = bar_chart(counts, edges, title="signed distance from the plane",
                      xlabel="h = n . P + d   (m)",
                      marks=[(0.0, "the plane", (40, 40, 200)),
                             (pl["tol"], "+tol %.0f mm" % (pl["tol"] * 1000),
                              (120, 120, 120)),
                             (-pl["tol"], "-tol", (120, 120, 120))])
    tilt = math.degrees(math.acos(min(1.0, abs(float(pl["n"][2])))))
    return figure(
        "18  RANSAC PLANE FIT",
        [(vis, "green = inlier (on the plane), red = off it"),
         (chart, "every point's distance from the fitted plane")],
        data=D(("normal x", round(float(pl["n"][0]), 6), "-"),
               ("normal y", round(float(pl["n"][1]), 6), "-"),
               ("normal z", round(float(pl["n"][2]), 6), "-"),
               ("offset d", round(float(pl["d"]), 6), "m"),
               ("points fitted", pl["fit_points"], "count"),
               ("points scored", len(P), "count"),
               ("inliers", int(pl["inliers"].sum()), "count"),
               ("inlier fraction", round(100 * pl["inlier_frac"], 2), "%"),
               ("residual RMS", round(pl["rms_mm"], 3), "mm"),
               ("inlier tolerance", pl["tol"] * 1000, "mm"),
               ("RANSAC iterations", pl["n_iters"], "count"),
               ("angle to the optical axis", round(tilt, 2), "deg")),
        formula="draw 3 points -> n = (b-a) x (c-a) / |...|,  d = -n.a;  score "
                "= #{ |n.P + d| < %.0f mm };  keep the best of %d draws"
                % (pl["tol"] * 1000, pl["n_iters"]),
        provenance=pl["provenance"] + "; fitted on %d of %d points"
                   % (pl["fit_points"], len(P)),
        numbers="normal (%.4f, %.4f, %.4f), offset %.4f m | %d of %d points "
                "are inliers (%.1f%%) | RMS %.2f mm | %.1f deg between the "
                "plane normal and the camera's optical axis"
                % (pl["n"][0], pl["n"][1], pl["n"][2], pl["d"],
                   int(pl["inliers"].sum()), len(P), 100 * pl["inlier_frac"],
                   pl["rms_mm"], tilt),
        note="A plane with too few inliers is not a plane. The pick refuses "
             "below 200 inliers rather than returning a geometry whose RMS is "
             "NaN -- everything downstream would treat that as a fix and the "
             "arm would move on it.")


@stage(19, "pca_refine", "The PCA refinement, and the NaN it once produced")
def st_pca_refine(fr):
    pl = plane_depth(fr)
    P, uv = depth_cloud(fr)
    n0, d0 = np.asarray(pl["n_ransac"], float), float(pl["d_ransac"])
    if float(np.dot(n0, pl["n"])) < 0:
        n0, d0 = -n0, -d0
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n0, pl["n"]))))))
    h_r = P @ n0 + d0
    h_p = pl["h_all"]
    in_r = np.abs(h_r) < pl["tol"]
    mixed = np.abs(P @ np.asarray(pl["n"], float) + d0) < pl["tol"]
    counts_r, edges = np.histogram(np.clip(h_r, -0.03, 0.03), bins=120)
    counts_p, _ = np.histogram(np.clip(h_p, -0.03, 0.03), bins=edges)
    a = bar_chart(counts_r, edges, title="RANSAC plane: 3 points decide it",
                  xlabel="h (m)", marks=[(0, "", (40, 40, 200))])
    b = bar_chart(counts_p, edges, title="after PCA on the inliers",
                  xlabel="h (m)", marks=[(0, "", (40, 40, 200))])
    rms_r = float(np.sqrt((h_r[in_r] ** 2).mean()) * 1000) if in_r.any() else float("nan")
    return figure(
        "19  PCA REFINEMENT OF THE PLANE",
        [(a, "before: normal from three sampled points"),
         (b, "after: normal = eigenvector of the SMALLEST eigenvalue of the "
             "inliers' covariance")],
        data=D(("normal moved", round(ang, 4), "deg"),
               ("RMS before refinement", round(rms_r, 3), "mm"),
               ("RMS after refinement", round(pl["rms_mm"], 3), "mm"),
               ("inliers with the matched pair", int(pl["inliers"].sum()), "count"),
               ("inliers with a mixed normal and offset", int(mixed.sum()), "count"),
               ("inliers lost by mixing",
                int(pl["inliers"].sum()) - int(mixed.sum()), "count")),
        formula="C = cov(Q - mean(Q)) for inliers Q;  n = eigvec(C)[:, argmin];"
                "  d = -n . mean(Q)   <-- BOTH halves, together",
        provenance=pl["provenance"],
        numbers="the normal moved %.3f deg | RMS %.2f mm -> %.2f mm | mixing "
                "the refined normal with the RANSAC offset selects %d inliers "
                "instead of %d"
                % (ang, rms_r, pl["rms_mm"], int(mixed.sum()),
                   int(pl["inliers"].sum())),
        note="That last number is the bug this stage exists to show. The "
             "function once returned the REFINED normal with the OLD offset; "
             "those describe different planes, the inlier mask could select "
             "ZERO points, and mean() of an empty slice is NaN. The pick "
             "printed 'plane RMS nan mm' and 'FOUND' in the same breath, then "
             "closed 540 mm blind on a centre computed against that plane.")


@stage(20, "height_map", "Height above the plane")
def st_height_map(fr):
    pl = plane_depth(fr)
    P, uv = depth_cloud(fr)
    d = need_depth(fr)
    h = pl["h_all"]
    img = np.full(d.shape + (3,), (32, 32, 32), np.uint8)
    x = np.clip((h + 0.01) / 0.12, 0, 1)
    try:
        cm = cv2.applyColorMap((x * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    except Exception:                                            # noqa: BLE001
        cm = cv2.applyColorMap((x * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[uv[:, 1], uv[:, 0]] = cm[:, 0, :]
    above = h > PICK_MIN_H_M
    onmask = np.zeros(d.shape, np.uint8)
    onmask[uv[above, 1], uv[above, 0]] = 255
    return figure(
        "20  HEIGHT ABOVE THE PLANE",
        [(img, "h from -10 mm (blue) to +110 mm (red)"),
         (onmask, "h > %.0f mm: what STANDS on the surface"
                  % (PICK_MIN_H_M * 1000))],
        data=D(("height threshold", PICK_MIN_H_M * 1000, "mm"),
               ("points above it", int(above.sum()), "count"),
               ("that fraction", round(100 * float(above.mean()), 3), "%"),
               ("tallest point", round(1000 * float(h.max()), 2), "mm"),
               ("lowest point", round(1000 * float(h.min()), 2), "mm"),
               ("median height", round(1000 * float(np.median(h)), 3), "mm")),
        formula="h(P) = n . P + d,  signed;  positive is off the surface, "
                "toward the camera",
        provenance="servo_pick_left.measure -- the same expression the live "
                   "pick uses to separate the cube from the table",
        numbers="%d of %d points stand more than %.0f mm off the plane "
                "(%.2f%%) | tallest %.1f mm"
                % (int(above.sum()), len(P), PICK_MIN_H_M * 1000,
                   100 * float(above.mean()), 1000 * float(h.max())),
        note="The floor is 5 mm because the plane RMS is under 1 mm on this "
             "sensor. Set it below the depth noise and the table itself "
             "becomes an object; set it far above and a flat item is missed.")


@stage(21, "mode_surface", "The surface height as a MODE, not a mean")
def st_mode_surface(fr):
    try:
        from srl_perception.surface_from_depth import (BIN_M, INLIER_BAND_M,
                                                       MIN_SHARE, MIN_POINTS)
    except Exception:                                            # noqa: BLE001
        BIN_M, INLIER_BAND_M, MIN_SHARE, MIN_POINTS = 0.002, 0.006, 0.20, 200
    pl = plane_depth(fr)
    P, _ = depth_cloud(fr)
    s = np.asarray(P @ np.asarray(pl["n"], float), float)   # along the normal
    if len(s) < MIN_POINTS:
        raise Refusal("only %d points; the estimator needs %d"
                      % (len(s), MIN_POINTS))
    lo = float(s.min())
    keys = np.floor((s - lo) / BIN_M).astype(int)
    cnt = np.bincount(keys)
    k = int(np.argmax(cnt))
    share = float(cnt[k]) / len(s)
    centre = lo + (k + 0.5) * BIN_M
    inl = np.abs(s - centre) <= INLIER_BAND_M
    z_hat = float(s[inl].mean())
    spread = float(np.sqrt(((s[inl] - z_hat) ** 2).mean()))
    mean_all = float(s.mean())
    counts, edges = np.histogram(s, bins=max(20, min(400, int((s.max() - lo) / BIN_M))))
    chart = bar_chart(counts, edges, title="points along the plane normal",
                      xlabel="n . P  (m)",
                      marks=[(centre, "modal bin", (40, 40, 200)),
                             (z_hat, "refined %.4f" % z_hat, OK_C),
                             (mean_all, "the MEAN %.4f" % mean_all, BAD_C)],
                      bands=[(z_hat - INLIER_BAND_M, z_hat + INLIER_BAND_M,
                              (225, 240, 225))])
    return figure(
        "21  SURFACE HEIGHT BY MODE",
        [(chart, "the mode, the refined value, and where a mean would land")],
        data=D(("bin width", BIN_M * 1000, "mm"),
               ("points binned", len(s), "count"),
               ("share in the fullest bin", round(100 * share, 2), "%"),
               ("share required", MIN_SHARE * 100, "%"),
               ("modal bin centre", round(centre, 5), "m"),
               ("refined estimate", round(z_hat, 5), "m"),
               ("spread of the inliers", round(1000 * spread, 3), "mm"),
               ("inlier band", INLIER_BAND_M * 1000, "mm"),
               ("a plain mean would say", round(mean_all, 5), "m"),
               ("that mean's error", round(1000 * abs(mean_all - z_hat), 2), "mm"),
               ("RANSAC's own offset", round(-pl["d"], 5), "m"),
               ("the two estimators differ by",
                round(1000 * abs(-pl["d"] - z_hat), 3), "mm")),
        formula="bin at %.0f mm; take the fullest bin; require it to hold "
                ">= %.0f%% of the points; refine as the mean of everything "
                "within +/-%.0f mm of it"
                % (BIN_M * 1000, MIN_SHARE * 100, INLIER_BAND_M * 1000),
        provenance="srl_perception/surface_from_depth.py, applied along the "
                   "fitted plane normal because this program has no camera-to-"
                   "world transform; the node applies it to world z",
        numbers="modal bin holds %.1f%% of %d points (floor %.0f%%) | refined "
                "%.4f m, spread %.2f mm | a plain MEAN would say %.4f m, "
                "which is %.1f mm out | RANSAC's own offset is %.4f m, "
                "%.2f mm from this"
                % (100 * share, len(s), 100 * MIN_SHARE, z_hat, 1000 * spread,
                   mean_all, 1000 * abs(mean_all - z_hat), -pl["d"],
                   1000 * abs(-pl["d"] - z_hat)),
        note="Two independent estimators of one surface, so they can be "
             "compared: RANSAC votes on inliers, this votes on the fullest "
             "histogram bin. A mean is dragged up by everything standing on "
             "the table and down by every hole, and it MOVES as the objects "
             "move -- the 'measurement' would change between trials on a "
             "motionless table.",
        cols=1, panel_w=1120, panel_h=430)


@stage(22, "on_surface", "What is standing on the surface")
def st_on_surface(fr):
    pl = plane_colour(fr)
    P, uv, idx = colour_cloud(fr)
    H, W = fr.colour.shape[:2]
    above = pl["h_all"] > PICK_MIN_H_M
    m = np.zeros((H, W), np.uint8)
    m[uv[above, 1], uv[above, 0]] = 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    vis = fr.colour.copy()
    rows = []
    for i in range(1, n):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 40:
            continue
        sel = idx[lab == i]
        sel = sel[sel >= 0]
        if len(sel) < 25:
            continue
        hh = pl["h_all"][sel]
        x, y, w, h = (int(st[i, cv2.CC_STAT_LEFT]), int(st[i, cv2.CC_STAT_TOP]),
                      int(st[i, cv2.CC_STAT_WIDTH]), int(st[i, cv2.CC_STAT_HEIGHT]))
        rows.append(dict(area_px=a, n_points=len(sel),
                         top_mm=round(1000 * float(hh.max()), 1),
                         median_mm=round(1000 * float(np.median(hh)), 1),
                         box=[x, y, w, h]))
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 200, 255), 2)
        cv2.putText(vis, "%.0f mm tall" % (1000 * float(hh.max())),
                    (x, max(14, y - 6)), FONT, 0.5, (0, 200, 255), 2, cv2.LINE_AA)
    rows.sort(key=lambda r: -r["area_px"])
    if not rows:
        raise Refusal("nothing stands more than %.0f mm above the fitted "
                      "plane in this view" % (PICK_MIN_H_M * 1000))
    return figure(
        "22  ON THE SURFACE",
        [(m, "the above-plane mask, in the colour frame"),
         (vis, "its connected pieces, each with its top height")],
        data=D(("pieces standing on the surface", len(rows), "count"),
               ("height threshold", PICK_MIN_H_M * 1000, "mm"),
               ("largest piece", rows[0]["area_px"], "px"),
               ("tallest piece", max(r["top_mm"] for r in rows), "mm")),
        tables={"pieces above the plane": (
            ["area px", "points", "top mm", "median height mm", "box x",
             "box y", "box w", "box h"],
            [[r["area_px"], r["n_points"], r["top_mm"], r["median_mm"]]
             + r["box"] for r in rows])},
        formula="{ pixels whose lifted point has h > %.0f mm }, 8-connected"
                % (PICK_MIN_H_M * 1000),
        provenance="the plane of stage 18 fitted in the colour frame; heights "
                   "from the aligned depth of stage 16",
        numbers=" | ".join("%d px, %d pts, top %.0f mm"
                           % (r["area_px"], r["n_points"], r["top_mm"])
                           for r in rows[:6]),
        note="This is the pre-2020 method on its own -- threshold above a "
             "plane, then cluster what is left. It is exactly what fails when "
             "two 40 mm cubes sit 20 mm apart: connectivity fuses them into "
             "one object 140 mm across. Compare this figure with stage 11.")


@stage(23, "cube_measurement", "The measurement the pick actually acts on")
def st_cube(fr):
    fx, fy, cx, cy = need_K(fr)
    P, uv = depth_cloud(fr)
    pl = plane_depth(fr)
    n, d = np.asarray(pl["n"], float), float(pl["d"])
    h = pl["h_all"]
    _, green = green_masks(fr)
    Pc = P + np.asarray(KINOVA_DEPTH_TO_COLOR_M if fr.name.endswith("gripper")
                        else (0, 0, 0), float)
    uu = Pc[:, 0] * fx / np.maximum(Pc[:, 2], 1e-6) + cx
    vv = Pc[:, 1] * fy / np.maximum(Pc[:, 2], 1e-6) + cy
    H, W = fr.colour.shape[:2]
    ok = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
    is_green = np.zeros(len(P), bool)
    is_green[ok] = green[vv[ok].astype(int), uu[ok].astype(int)] > 0
    cube = is_green & (h > PICK_MIN_H_M)
    if int(cube.sum()) < 60:
        raise Refusal(
            "only %d points are both %s-coloured AND more than %.0f mm above "
            "the plane (the pick needs 60 and returns None below that). "
            "%d points are coloured but ON the plane; %d stand off the plane "
            "but are not the colour."
            % (int(cube.sum()), "green", PICK_MIN_H_M * 1000,
               int((is_green & ~(h > PICK_MIN_H_M)).sum()),
               int(((~is_green) & (h > PICK_MIN_H_M)).sum())))
    Pk = P[cube]
    hk = Pk @ n + d
    mean_pt = Pk.mean(0)
    foot = mean_pt - n * (mean_pt @ n + d)
    centre = foot + n * (hk.max() / 2.0)
    rms = float(np.sqrt((h[pl["inliers"]] ** 2).mean()) * 1000)
    vis = fr.colour.copy()
    pts_uv = np.stack([uu[cube], vv[cube]], 1).astype(int)
    for u_, v_ in pts_uv[::max(1, len(pts_uv) // 4000)]:
        cv2.circle(vis, (u_, v_), 1, (0, 255, 255), -1)
    cimg = _proj((centre + np.asarray(
        KINOVA_DEPTH_TO_COLOR_M if fr.name.endswith("gripper") else (0, 0, 0),
        float))[None, :], (fx, fy, cx, cy))[0]
    cv2.drawMarker(vis, (int(cimg[0]), int(cimg[1])), (0, 0, 255),
                   cv2.MARKER_CROSS, 30, 3)
    cv2.putText(vis, "centre  %.3f, %.3f, %.3f m" % tuple(centre),
                (max(6, int(cimg[0]) - 150), max(20, int(cimg[1]) - 20)),
                FONT, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    side = scatter(Pk[:, 0], hk, xlabel="X in the camera frame (m)",
                   ylabel="height above the plane (m)", equal=False,
                   title="the object's own points, against the surface",
                   lines=[((float(Pk[:, 0].min()), 0.0),
                           (float(Pk[:, 0].max()), 0.0), (40, 40, 200),
                           "the plane"),
                          ((float(centre[0]), 0.0),
                           (float(centre[0]), float(hk.max())), (0, 160, 0),
                           "half height")])
    return figure(
        "23  THE CUBE, MEASURED",
        [(mask_overlay(fr.colour, green > 0, (0, 220, 0), 0.4), "colour gate"),
         (vis, "points that pass BOTH gates, and the centre"),
         (side, "why the centre is the foot plus half the height")],
        data=D(("object points", int(cube.sum()), "count"),
               ("points that are the colour but ON the plane",
                int((is_green & ~(h > PICK_MIN_H_M)).sum()), "count"),
               ("points off the plane but not the colour",
                int(((~is_green) & (h > PICK_MIN_H_M)).sum()), "count"),
               ("top above the plane", round(1000 * float(hk.max()), 2), "mm"),
               ("median height", round(1000 * float(np.median(hk)), 2), "mm"),
               ("centre x", round(float(centre[0]), 5), "m"),
               ("centre y", round(float(centre[1]), 5), "m"),
               ("centre z", round(float(centre[2]), 5), "m"),
               ("range to the centre",
                round(float(np.linalg.norm(centre)), 5), "m"),
               ("plane RMS", round(rms, 3), "mm"),
               ("plane inlier fraction", round(pl["inlier_frac"], 4), "-"),
               ("minimum points the pick requires", 60, "count")),
        formula="cube = colour(u,v) AND h > %.0f mm;  foot = mean(Pk) - n "
                "(n.mean(Pk) + d);  centre = foot + n * max(h)/2"
                % (PICK_MIN_H_M * 1000),
        provenance="scripts/servo_pick_left.py:measure -- this is the "
                   "measurement the servo loop nulls against, in the CAMERA's "
                   "frame, where the data is good",
        numbers="%d object points | top %.1f mm above the plane | centre "
                "(%.4f, %.4f, %.4f) m in the camera frame | plane RMS %.2f mm, "
                "inlier fraction %.2f"
                % (int(cube.sum()), 1000 * float(hk.max()), centre[0],
                   centre[1], centre[2], rms, pl["inlier_frac"]),
        note="Two independent gates, and that is the point: colour alone "
             "picks up the green pad the cube stands on and anything green in "
             "the room; height alone picks up everything on the table. The "
             "centre uses the foot point rather than the cloud's centroid "
             "because a depth camera sees a SHELL -- the front surface only "
             "-- and a shell's centroid is biased toward the camera, measured "
             "at 10.5 mm in the error budget against a 30 mm capture gate.")


@stage(24, "layers", "Splitting a cube from the pad it stands on")
def st_layers(fr):
    try:
        from srl_perception.segment_lift import _layers
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("segment_lift will not import (%s)" % exc)
    inst = instances(fr)
    o = inst[0]
    h = o["heights"]
    lays = _layers(h) or [(float(h.min()), float(h.max()) + 1e-6)]
    counts, edges = np.histogram(h, bins=80)
    marks = []
    for k, (h0, h1) in enumerate(lays):
        marks.append((h1, "cut %d at %.0f mm" % (k + 1, 1000 * h1),
                      (40, 40, 200)))
    chart = bar_chart(counts, edges, title="heights inside ONE segmented region",
                      xlabel="h above the plane (m)", marks=marks)
    H, W = fr.colour.shape[:2]
    P, uv, idx = colour_cloud(fr)
    vis = fr.colour.copy()
    cols = [(80, 200, 255), (0, 220, 0), (255, 120, 0), (200, 0, 200)]
    rows = []
    for k, (h0, h1) in enumerate(lays):
        keep = (h >= h0) & (h < h1)
        if keep.sum() < 25:
            continue
        pix = uv[o["sel"][keep]]
        for u_, v_ in pix[::max(1, len(pix) // 3000)]:
            cv2.circle(vis, (int(u_), int(v_)), 1, cols[k % len(cols)], -1)
        rows.append("layer %d: %.0f-%.0f mm, %d points"
                    % (k + 1, 1000 * h0, 1000 * h1, int(keep.sum())))
    return figure(
        "24  HEIGHT LAYERS INSIDE ONE REGION",
        [(mask_overlay(fr.colour, o["mask"], (0, 200, 200), 0.45),
          "the region, as the segmenter returned it"),
         (chart, "its points' heights"),
         (vis, "the layers, coloured separately")],
        data=D(("region area", o["area_px"], "px"),
               ("points lifted from it", o["n_raw"], "count"),
               ("points above the plane", o["n_above"], "count"),
               ("pixels removed by the 1 px erosion", o["eroded_away"], "px"),
               ("layers found", len(lays), "count"),
               ("lowest point", round(1000 * float(h.min()), 2), "mm"),
               ("highest point", round(1000 * float(h.max()), 2), "mm")),
        tables={"height layers": (
            ["layer", "from mm", "to mm", "points"],
            [[k + 1, round(1000 * h0, 2), round(1000 * h1, 2),
              int(((h >= h0) & (h < h1)).sum())]
             for k, (h0, h1) in enumerate(lays)])},
        formula="find the MODAL height bin (%0.0f mm bins); cut %0.0f mm "
                "above it; recurse on what is left, twice" % (5.0, 12.0),
        provenance="srl_perception/segment_lift.py:_layers",
        numbers=" | ".join(rows) if rows else "one layer only",
        note="Appearance decides what is SIDE BY SIDE; height decides what is "
             "STACKED. A blue cube on a blue pad is correctly ONE region -- "
             "same colour, touching, no edge between them. And the split "
             "cannot look for an empty band, because there is none: the cube "
             "RESTS on the pad, so its sides fill every bin. What is there is "
             "a MODE -- the support's top face is a large flat area and "
             "dominates the histogram.")


@stage(25, "min_width", "Rotating calipers, against the two controls")
def st_min_width(fr):
    inst = instances(fr)
    pl = plane_colour(fr)
    up = np.asarray(pl["n"], float)
    o = inst[0]
    pts = o["pts"]
    centre, ext, width = _measure_instance(pts, up)
    # the same points, in the plane's basis, so the three answers compare
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ up)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = tmp - up * float(tmp @ up)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    rel = pts - pts.mean(0)
    flat = np.stack([rel @ e1, rel @ e2], 1)
    w_min, ndir, th = min_width(flat)
    aabb = float(np.ptp(flat, axis=0).min())
    C = np.cov(flat.T)
    ev, EV = np.linalg.eigh(C)
    pca_dir = EV[:, 0]
    w_pca = float(np.ptp(flat @ pca_dir))
    ang = np.radians(np.arange(0, 180, 0.5))
    spans = [float(np.ptp(flat @ np.array([math.cos(a), math.sin(a)])))
             for a in ang]
    km = int(np.argmin(spans))
    kx = int(np.argmax(spans))
    curve = line_chart(
        np.degrees(ang), 1000 * np.asarray(spans),
        title="span across a direction, swept through 180 deg",
        xlabel="direction in the support plane (deg)", ylabel="span (mm)",
        points=[(math.degrees(ang[km]), 1000 * spans[km], OK_C,
                 "minimum %.1f mm at %.1f deg"
                 % (1000 * spans[km], math.degrees(ang[km]))),
                (math.degrees(ang[kx]), 1000 * spans[kx], (150, 110, 40),
                 "widest %.1f mm" % (1000 * spans[kx]))])
    # draw the jaw direction back in the image
    vis = fr.colour.copy()
    c3 = pts.mean(0)
    d3 = e1 * ndir[0] + e2 * ndir[1]
    a3 = c3 - d3 * w_min / 2.0
    b3 = c3 + d3 * w_min / 2.0
    pr = _proj(np.stack([a3, b3]), fr.K_c)
    cv2.line(vis, tuple(pr[0].astype(int)), tuple(pr[1].astype(int)),
             (0, 0, 255), 3)
    cv2.putText(vis, "jaws close here: %.1f mm" % (1000 * w_min),
                (int(pr[0][0]), max(20, int(pr[0][1]) - 12)), FONT, 0.6,
                (0, 0, 255), 2, cv2.LINE_AA)
    return figure(
        "25  MINIMUM WIDTH  (rotating calipers)",
        [(vis, "the direction the jaws would close"),
         (curve, "the swept span, and its minimum")],
        data=D(("instance points", len(pts), "count"),
               ("minimum width, rotating calipers", round(1000 * w_min, 2), "mm"),
               ("at this angle in the plane",
                round(math.degrees(ang[km]), 2), "deg"),
               ("PCA short axis, THE CONTROL", round(1000 * w_pca, 2), "mm"),
               ("axis-aligned box, THE CONTROL", round(1000 * aabb, 2), "mm"),
               ("PCA overstates by", round(1000 * (w_pca - w_min), 2), "mm"),
               ("long axis", round(1000 * ext[0], 2), "mm"),
               ("height", round(1000 * ext[2], 2), "mm"),
               ("jaw span", JAW_M * 1000, "mm"),
               ("graspable as it lies", bool(w_min <= JAW_M), "-"),
               ("sweep step", 0.5, "deg")),
        formula="width = min over theta of [ max(p.u(theta)) - min(p.u(theta)) ]"
                ",  u(theta) = (cos, sin) in the support plane, swept at 0.25 deg",
        provenance="srl_perception/segment_lift.py:_min_width and "
                   "rgbd_grasp.min_width_frame",
        numbers="calipers %.1f mm | PCA's short axis %.1f mm | axis-aligned "
                "box %.1f mm | long axis %.1f mm, height %.1f mm | jaws close "
                "on %.0f mm, so this is %s"
                % (1000 * w_min, 1000 * w_pca, 1000 * aabb, 1000 * ext[0],
                   1000 * ext[2], 1000 * JAW_M,
                   "graspable" if w_min <= JAW_M else "TOO WIDE"),
        note="The two controls are here because both were used and both were "
             "wrong. The principal axes of a SQUARE are degenerate -- equal "
             "variance in both directions -- so SVD returns the diagonal and "
             "a 40 mm cube measures 40*sqrt(2) = 53 mm; against an 85 mm jaw "
             "that refuses a 60 mm object as ungraspable. The error that "
             "matters is not the 13 mm, it is the DIRECTION: a jaw told to "
             "close across a diagonal approaches the corner and the object "
             "rolls out.")


@stage(26, "nested", "Dropping masks that are unions of finer ones")
def st_nested(fr):
    inst = instances(fr)
    voxel, frac = 0.005, 0.55
    order = sorted(range(len(inst)), key=lambda i: len(inst[i]["pts"]))
    claimed, kept, dropped = set(), [], []
    for i in order:
        keys = {(int(math.floor(x / voxel)), int(math.floor(y / voxel)),
                 int(math.floor(z / voxel))) for x, y, z in inst[i]["pts"]}
        if not keys:
            continue
        ov = len(keys & claimed) / float(len(keys))
        if ov > frac:
            dropped.append((i, ov))
            continue
        claimed |= keys
        kept.append((i, ov))
    return figure(
        "26  NESTED-MASK SUPPRESSION",
        [(colourise_labels(fr.colour.shape, [inst[i]["mask"] for i, _ in kept],
                           0.55, fr.colour),
          "kept: %d instances" % len(kept)),
         (colourise_labels(fr.colour.shape,
                           [inst[i]["mask"] for i, _ in dropped], 0.55,
                           fr.colour),
          "dropped: %d whose volume was already explained" % len(dropped))],
        data=D(("instances in", len(inst), "count"),
               ("kept", len(kept), "count"),
               ("dropped as already explained", len(dropped), "count"),
               ("voxel size", voxel * 1000, "mm"),
               ("overlap fraction that rejects", frac, "-")),
        tables={"per instance": (
            ["instance", "points", "area px", "claimed overlap", "verdict"],
            [[i, len(inst[i]["pts"]), inst[i]["area_px"], round(ov, 3), "kept"]
             for i, ov in kept]
            + [[i, len(inst[i]["pts"]), inst[i]["area_px"], round(ov, 3),
                "dropped"] for i, ov in dropped])},
        formula="voxelise each instance at %.0f mm; accept smallest first; "
                "reject one whose claimed-voxel overlap exceeds %.2f"
                % (voxel * 1000, frac),
        provenance="srl_perception/segment_lift.py:_suppress_nested",
        numbers="%d instances in, %d kept, %d dropped | overlaps: %s"
                % (len(inst), len(kept), len(dropped),
                   ", ".join("%.2f" % o for _, o in dropped[:6]) or "none"),
        note="FastSAM returns a HIERARCHY, not a partition: for one picture it "
             "hands back the cube, the pad, and a region covering both. All "
             "three are legitimate. Lifted, they became three objects in the "
             "same space and the merge absorbed a 40 mm cube -- measured "
             "EXACT at 40.3 mm against a 40 mm truth -- into a 148 mm blob "
             "that was then refused as ungraspable. Volume, not pixels: two "
             "masks that overlap in the image can be a metre apart in depth.")


@stage(27, "grasp", "The parallel-jaw grasp")
def st_grasp(fr):
    try:
        from srl_perception.rgbd_grasp import grasp_from_cloud, GraspRefusal
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("rgbd_grasp will not import (%s)" % exc)
    inst = instances(fr)
    o = inst[0]
    pts = o["pts"]
    try:
        g = grasp_from_cloud(pts, view_axis=(0.0, 0.0, 1.0))
    except GraspRefusal as exc:
        raise Refusal("the grasp planner refused, which is an ANSWER: %s"
                      % exc)
    vis = fr.colour.copy()
    c, ax, ap, w = (np.asarray(g["centre"]), np.asarray(g["close_axis"]),
                    np.asarray(g["approach"]), float(g["width_m"]))
    jaw = np.stack([c - ax * w / 2, c + ax * w / 2])
    appr = np.stack([c, c - ap * 0.12])
    pj, pa = _proj(jaw, fr.K_c), _proj(appr, fr.K_c)
    cv2.line(vis, tuple(pj[0].astype(int)), tuple(pj[1].astype(int)),
             (0, 0, 255), 4)
    for p in pj:
        cv2.circle(vis, tuple(p.astype(int)), 7, (0, 0, 255), -1)
    cv2.arrowedLine(vis, tuple(pa[1].astype(int)), tuple(pa[0].astype(int)),
                    (255, 140, 0), 3, tipLength=0.15)
    cv2.putText(vis, "close %.1f mm" % (1000 * w),
                (int(pj[0][0]) - 40, max(20, int(pj[0][1]) - 14)), FONT, 0.6,
                (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(vis, "approach (120 mm stand-off)",
                (int(pa[1][0]) - 60, int(pa[1][1]) - 10), FONT, 0.55,
                (255, 140, 0), 2, cv2.LINE_AA)
    top = scatter(pts[:, 0], pts[:, 2], xlabel="X (m)", ylabel="Z (m)",
                  title="the instance's own points, from above",
                  lines=[((float(jaw[0][0]), float(jaw[0][2])),
                          (float(jaw[1][0]), float(jaw[1][2])), (40, 40, 200),
                          "jaw axis")])
    return figure(
        "27  GRASP SYNTHESIS",
        [(vis, "where the jaws close and how the hand comes in"),
         (top, "the same, from above")],
        data=D(("object points", len(pts), "count"),
               ("jaw closing width", round(1000 * w, 2), "mm"),
               ("usable jaw span", (JAW_M - 0.010) * 1000, "mm"),
               ("margin left", round(1000 * (JAW_M - 0.010 - w), 2), "mm"),
               ("centre x", round(float(c[0]), 5), "m"),
               ("centre y", round(float(c[1]), 5), "m"),
               ("centre z", round(float(c[2]), 5), "m"),
               ("close axis x", round(float(ax[0]), 4), "-"),
               ("close axis y", round(float(ax[1]), 4), "-"),
               ("close axis z", round(float(ax[2]), 4), "-"),
               ("approach x", round(float(ap[0]), 4), "-"),
               ("approach y", round(float(ap[1]), 4), "-"),
               ("approach z", round(float(ap[2]), 4), "-"),
               ("object length", round(1000 * float(g["length_m"]), 2), "mm"),
               ("stand-off drawn", 120.0, "mm")),
        formula="close across the MINIMUM width, perpendicular to the view "
                "axis; approach along the remaining axis; pregrasp = centre - "
                "approach * standoff",
        provenance="srl_perception/rgbd_grasp.py:grasp_from_cloud, geometric "
                   "and deliberately not learned -- a learned grasp model "
                   "needs a GPU budget this machine does not have and cannot "
                   "explain a refusal",
        numbers="width %.1f mm (usable jaw %.0f mm) | centre (%.3f, %.3f, "
                "%.3f) m | close axis (%.3f, %.3f, %.3f) | approach (%.3f, "
                "%.3f, %.3f)"
                % (1000 * w, 1000 * (JAW_M - 0.010), c[0], c[1], c[2],
                   ax[0], ax[1], ax[2], ap[0], ap[1], ap[2]),
        note="The view axis is passed in deliberately. One depth view sees a "
             "SHELL with almost no thickness along the line of sight, so an "
             "unconstrained minimum-width search picks the view direction "
             "every time and reports ~0 mm -- measured 0.0 mm on a "
             "constructed 50 mm cube. Acting on that closes the jaw THROUGH a "
             "surface whose far side was never observed.")


def object_pose(pts, up, d_plane):
    """One instance as a POSE and a BOX, using the best half of each estimator.

    Validated against a constructed cube in --self-test: mid-extent is
    unbiased ACROSS the surface (0.22 mm) and 16 mm out ALONG the normal,
    because that axis is still the mean of a one-sided cloud; the shell
    correction is the other way round. Taking mid-extent horizontally and
    foot-plus-half-height vertically lands within 2 mm in full 3-D, and that
    is what this returns.
    """
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    mid, ext, w = _measure_instance(pts, up)
    h = pts @ up + d_plane
    top = float(h.max())
    mid_h = float(np.asarray(mid) @ up + d_plane)
    centre = np.asarray(mid, float) + up * (top / 2.0 - mid_h)
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ up)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = tmp - up * float(tmp @ up)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    flat = np.stack([(pts - pts.mean(0)) @ e1, (pts - pts.mean(0)) @ e2], 1)
    _, ndir, th = min_width(flat)
    short_ax = e1 * ndir[0] + e2 * ndir[1]
    long_ax = np.cross(up, short_ax)
    return dict(centre=centre, up=up, short_axis=short_ax, long_axis=long_ax,
                width_m=float(ext[1]), length_m=float(ext[0]), height_m=top,
                yaw_deg=float(math.degrees(th)),
                range_m=float(np.linalg.norm(centre)),
                plane_coords=(float(centre @ e1), float(centre @ e2)),
                basis=(e1, e2))


def box_corners(o):
    """The eight corners of an instance's oriented box, plane to top."""
    c, u = np.asarray(o["centre"]), np.asarray(o["up"])
    a, b = np.asarray(o["long_axis"]), np.asarray(o["short_axis"])
    L, S, H = o["length_m"] / 2, o["width_m"] / 2, o["height_m"] / 2
    return np.array([c + sa * a * L + sb * b * S + sc * u * H
                     for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)])


BOX_EDGES = [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6), (5, 7), (6, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]


def draw_box(img, o, K, colour=(0, 220, 255), label=""):
    pr = _proj(box_corners(o), K).astype(int)
    for i, j in BOX_EDGES:
        cv2.line(img, tuple(pr[i]), tuple(pr[j]), colour, 2, cv2.LINE_AA)
    c = _proj(np.asarray(o["centre"])[None, :], K)[0].astype(int)
    cv2.drawMarker(img, tuple(c), colour, cv2.MARKER_CROSS, 16, 2)
    if label:
        cv2.putText(img, label, (int(pr[:, 0].min()), max(16, int(pr[:, 1].min()) - 8)),
                    FONT, 0.52, colour, 2, cv2.LINE_AA)
    return img


@stage(28, "apriltag", "Fiducials: the primary detector")
def st_apriltag(fr, tag_size_m=0.040):
    grey = cv2.cvtColor(fr.colour, cv2.COLOR_BGR2GRAY)
    fams = [("36h11", cv2.aruco.DICT_APRILTAG_36h11),
            ("25h9", cv2.aruco.DICT_APRILTAG_25h9),
            ("16h5", cv2.aruco.DICT_APRILTAG_16h5)]
    hits, api = [], ""
    for fname, fid in fams:
        try:                                    # OpenCV 4.7+
            dic = cv2.aruco.getPredefinedDictionary(fid)
            det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
            corners, ids, _ = det.detectMarkers(grey)
            api = "cv2.aruco.ArucoDetector (OpenCV %s)" % cv2.__version__
        except AttributeError:                  # OpenCV 4.6 and earlier
            dic = cv2.aruco.Dictionary_get(fid)
            corners, ids, _ = cv2.aruco.detectMarkers(
                grey, dic, parameters=cv2.aruco.DetectorParameters_create())
            api = "cv2.aruco.detectMarkers (OpenCV %s)" % cv2.__version__
        if ids is not None and len(ids):
            for c, i in zip(corners, ids.ravel()):
                hits.append((fname, int(i), c.reshape(4, 2)))
    vis = fr.colour.copy()
    rows = []
    for fname, tid, quad in hits:
        cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 220, 255), 2)
        ctr = quad.mean(0)
        cv2.putText(vis, "%s id %d" % (fname, tid),
                    (int(ctr[0]) - 40, int(ctr[1]) - 10), FONT, 0.55,
                    (0, 220, 255), 2, cv2.LINE_AA)
        row = dict(family=fname, id=tid, centre_u=round(float(ctr[0]), 1),
                   centre_v=round(float(ctr[1]), 1),
                   side_px=round(float(np.linalg.norm(quad[0] - quad[1])), 1))
        if fr.K_c:
            fx, fy, cx, cy = fr.K_c
            h = tag_size_m / 2.0
            obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
            Kc = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
            ok, rv, tv = cv2.solvePnP(obj, quad.astype(float), Kc,
                                      np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                row.update(x_m=round(float(tv[0]), 4), y_m=round(float(tv[1]), 4),
                           z_m=round(float(tv[2]), 4),
                           range_m=round(float(np.linalg.norm(tv)), 4))
                cv2.putText(vis, "%.3f m" % float(np.linalg.norm(tv)),
                            (int(ctr[0]) - 40, int(ctr[1]) + 18), FONT, 0.55,
                            (0, 220, 255), 2, cv2.LINE_AA)
        rows.append(row)
    if not hits:
        raise Refusal(
            "no AprilTag of family 36h11, 25h9 or 16h5 is in this frame. That "
            "is an answer, not a failure: a tag either decodes or it does "
            "not, which is exactly why this is the PRIMARY detector and "
            "colour is the capped fallback. Detector used: %s." % api)
    return figure(
        "28  FIDUCIAL DETECTION",
        [(vis, "%d tag(s), with the range solvePnP gives" % len(hits))],
        formula="decode the payload, then solvePnP on the four corners of a "
                "square of known side (%.0f mm) -> full 6-DOF pose"
                % (tag_size_m * 1000),
        provenance="srl_perception/apriltag_detector.py's method, via %s. No "
                   "extra dependency: the AprilTag families ship inside "
                   "OpenCV's aruco module." % api,
        data=D(("tags detected", len(hits), "count"),
               ("assumed tag side", tag_size_m * 1000, "mm"),
               ("nearest tag",
                min((r.get("range_m", float("nan")) for r in rows),
                    default=float("nan")), "m")),
        tables={"tags": (["family", "id", "centre u", "centre v", "side px",
                          "x m", "y m", "z m", "range m"],
                         [[r.get(k, "-") for k in ("family", "id", "centre_u",
                                                   "centre_v", "side_px", "x_m",
                                                   "y_m", "z_m", "range_m")]
                          for r in rows])},
        numbers="%d tag(s)" % len(hits),
        note="A tag is unambiguous where colour is confidently wrong, and it "
             "carries an ID, so it says WHICH object it is. This program "
             "reports it whether or not tags are in use, because a scene with "
             "no tags and a scene where the detector is broken must not look "
             "the same.",
        cols=1, panel_w=900, panel_h=620)


@stage(29, "table", "The support surface, as an object")
def st_table(fr):
    K = need_K(fr)
    pl = plane_colour(fr)
    P, uv, idx = colour_cloud(fr)
    n = np.asarray(pl["n"], float)
    inl = pl["inliers"]
    if int(inl.sum()) < 200:
        raise Refusal("only %d points lie on the fitted plane; that is not a "
                      "surface" % int(inl.sum()))
    Q = P[inl]
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ n)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = tmp - n * float(tmp @ n)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    a, b = Q @ e1, Q @ e2
    # 2nd/98th percentile, not min/max: one stray return stretches the extent
    a0, a1 = float(np.percentile(a, 2)), float(np.percentile(a, 98))
    b0, b1 = float(np.percentile(b, 2)), float(np.percentile(b, 98))
    corners = np.array([e1 * x + e2 * y - n * pl["d"]
                        for x, y in ((a0, b0), (a1, b0), (a1, b1), (a0, b1))])
    vis = fr.colour.copy()
    vis = mask_overlay(vis, _mask_from_uv(uv[inl], fr.colour.shape),
                       (90, 200, 90), 0.35, outline=False)
    pr = _proj(corners, K).astype(int)
    cv2.polylines(vis, [pr], True, (0, 140, 255), 3, cv2.LINE_AA)
    for k, q in enumerate(pr):
        cv2.circle(vis, tuple(q), 6, (0, 140, 255), -1)
    ctr = corners.mean(0)
    cpix = _proj(ctr[None, :], K)[0].astype(int)
    cv2.drawMarker(vis, tuple(cpix), (0, 140, 255), cv2.MARKER_CROSS, 26, 3)
    cv2.putText(vis, "surface %.3f m away, %.1f deg to the optical axis"
                % (float(np.linalg.norm(ctr)),
                   math.degrees(math.acos(min(1.0, abs(float(n[2])))))),
                (10, 30), FONT, 0.62, (0, 140, 255), 2, cv2.LINE_AA)
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    return figure(
        "29  THE SUPPORT SURFACE",
        [(colourise_depth(aligned_depth(fr)[0]), "the depth it was fitted to"),
         (vis, "the plane's inliers and the extent they span")],
        formula="fit the plane, keep its inliers, span them in the plane's own "
                "basis; extent taken at the 2nd and 98th percentile so one "
                "stray return cannot stretch it",
        provenance="the plane of stage 18, expressed as a rectangle in the "
                   "camera frame",
        data=D(("points on the surface", int(inl.sum()), "count"),
               ("surface extent along its first axis", round(a1 - a0, 4), "m"),
               ("surface extent along its second axis", round(b1 - b0, 4), "m"),
               ("area spanned", round((a1 - a0) * (b1 - b0), 4), "m2"),
               ("perpendicular distance from the camera", round(pl["d"], 4), "m"),
               ("distance to the middle of that extent",
                round(float(np.linalg.norm(ctr)), 4), "m"),
               ("nearest point on the surface",
                round(float(np.linalg.norm(Q, axis=1).min()), 4), "m"),
               ("furthest point on the surface",
                round(float(np.linalg.norm(Q, axis=1).max()), 4), "m"),
               ("tilt from the optical axis", round(tilt, 2), "deg"),
               ("flatness, RMS of the inliers", round(pl["rms_mm"], 3), "mm")),
        numbers="%.3f x %.3f m of surface, %.3f m away, flat to %.2f mm"
                % (a1 - a0, b1 - b0, pl["d"], pl["rms_mm"]),
        note="This is a MEASURED surface, not the declared one. "
             "srl_experiments/work_surface.py has had set_measured() since it "
             "was written and nothing ever called it, so a table 20 mm out "
             "would have been absorbed silently into every grasp.")


def _mask_from_uv(uv, shape):
    m = np.zeros(shape[:2], bool)
    m[uv[:, 1], uv[:, 0]] = True
    return m


@stage(30, "boxes3d", "Every object, as an oriented 3-D box")
def st_boxes3d(fr):
    K = need_K(fr)
    inst = instances(fr)
    pl = plane_colour(fr)
    up = np.asarray(pl["n"], float)
    objs = []
    for k, o in enumerate(inst[:12]):
        try:
            op = object_pose(o["pts"], up, pl["d"])
        except Exception:                                        # noqa: BLE001
            continue
        op["idx"] = k
        op["points"] = len(o["pts"])
        op["area_px"] = o["area_px"]
        op["mean_bgr"] = [round(v, 1) for v in o["mean_bgr"]]
        objs.append(op)
    if not objs:
        raise Refusal("no instance could be posed")
    vis = fr.colour.copy()
    rng = np.random.default_rng(11)
    for o in objs:
        col = tuple(int(v) for v in rng.integers(70, 255, 3))
        draw_box(vis, o, K, col,
                 "#%d  %.0fx%.0fx%.0f mm  %.3f m"
                 % (o["idx"], 1000 * o["length_m"], 1000 * o["width_m"],
                    1000 * o["height_m"], o["range_m"]))
    e1, e2 = objs[0]["basis"]
    xs = np.concatenate([o["points"] * 0 + 0 for o in objs]) if False else None
    allp = np.vstack([inst[o["idx"]]["pts"] for o in objs])
    lines = []
    for o in objs:
        c = np.asarray(o["centre"])
        a, b = np.asarray(o["long_axis"]), np.asarray(o["short_axis"])
        cor = [c + sa * a * o["length_m"] / 2 + sb * b * o["width_m"] / 2
               for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        cor2 = [(float(q @ e1), float(q @ e2)) for q in cor]
        for i in range(4):
            lines.append((cor2[i], cor2[(i + 1) % 4], (40, 40, 200),
                          "#%d" % o["idx"] if i == 0 else ""))
    plan = scatter(allp @ e1, allp @ e2, xlabel="along the surface, axis 1 (m)",
                   ylabel="along the surface, axis 2 (m)", lines=lines,
                   title="plan view: every object on the surface, to scale")
    return figure(
        "30  ORIENTED 3-D BOXES",
        [(vis, "each object's box, projected back into the picture"),
         (plan, "the same objects, looking down on the surface")],
        formula="centre = mid-extent ACROSS the surface + (foot + top/2) ALONG "
                "the normal; axes = (minimum-width direction, its "
                "perpendicular, the plane normal); size = the spans along them",
        provenance="segment_lift._measure for the horizontal centre and "
                   "rgbd_grasp.shell_corrected_centre's reasoning for the "
                   "vertical -- the combination is checked to 2 mm against a "
                   "constructed cube in --self-test",
        data=D(("objects posed", len(objs), "count"),
               ("nearest object", round(min(o["range_m"] for o in objs), 4), "m"),
               ("furthest object", round(max(o["range_m"] for o in objs), 4), "m"),
               ("largest footprint",
                round(1000 * max(o["length_m"] for o in objs), 1), "mm"),
               ("tallest object",
                round(1000 * max(o["height_m"] for o in objs), 1), "mm"),
               ("graspable within the %.0f mm jaw" % (JAW_M * 1000),
                sum(1 for o in objs if o["width_m"] <= JAW_M), "count")),
        tables={"every object, in the camera frame": (
            ["#", "points", "area px", "x m", "y m", "z m", "range m",
             "length mm", "width mm", "height mm", "yaw deg", "graspable"],
            [[o["idx"], o["points"], o["area_px"], round(float(o["centre"][0]), 4),
              round(float(o["centre"][1]), 4), round(float(o["centre"][2]), 4),
              round(o["range_m"], 4), round(1000 * o["length_m"], 1),
              round(1000 * o["width_m"], 1), round(1000 * o["height_m"], 1),
              round(o["yaw_deg"], 1), o["width_m"] <= JAW_M] for o in objs])},
        numbers="%d objects, %.3f to %.3f m away"
                % (len(objs), min(o["range_m"] for o in objs),
                   max(o["range_m"] for o in objs)),
        note="These coordinates are in the CAMERA's frame. Putting them in the "
             "robot's frame needs the camera extrinsic, which on the wrist is "
             "measured at ~8.5 deg wrong and rotates with the joint -- 34 mm "
             "of error at 0.23 m. That is why the pick servos on an error "
             "measured HERE rather than planning through that transform.")


@stage(31, "distances", "How far apart everything is")
def st_distances(fr):
    K = need_K(fr)
    inst = instances(fr)
    pl = plane_colour(fr)
    up = np.asarray(pl["n"], float)
    objs = []
    for k, o in enumerate(inst[:10]):
        try:
            op = object_pose(o["pts"], up, pl["d"])
        except Exception:                                        # noqa: BLE001
            continue
        op["idx"] = k
        objs.append(op)
    if len(objs) < 1:
        raise Refusal("nothing to measure between")
    vis = fr.colour.copy()
    for o in objs:
        c = _proj(np.asarray(o["centre"])[None, :], K)[0].astype(int)
        cv2.drawMarker(vis, tuple(c), (0, 220, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(vis, "#%d %.3f m" % (o["idx"], o["range_m"]),
                    (c[0] + 8, c[1] - 8), FONT, 0.5, (0, 220, 255), 2, cv2.LINE_AA)
    pairs = []
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            a, b = np.asarray(objs[i]["centre"]), np.asarray(objs[j]["centre"])
            dist = float(np.linalg.norm(a - b))
            u = (b - a) / max(dist, 1e-9)
            gap = dist - (objs[i]["width_m"] / 2 + objs[j]["width_m"] / 2)
            pairs.append([objs[i]["idx"], objs[j]["idx"], round(dist, 4),
                          round(max(gap, 0.0), 4)])
            pa = _proj(a[None, :], K)[0].astype(int)
            pb = _proj(b[None, :], K)[0].astype(int)
            cv2.line(vis, tuple(pa), tuple(pb), (255, 160, 0), 1, cv2.LINE_AA)
            mid = ((pa + pb) / 2).astype(int)
            cv2.putText(vis, "%.0f mm" % (1000 * dist), tuple(mid), FONT, 0.45,
                        (255, 160, 0), 2, cv2.LINE_AA)
    wrist = fr.name.endswith("gripper")
    return figure(
        "31  DISTANCES",
        [(vis, "range to each object, and the span between them")],
        formula="range = |centre| (the camera is the origin); height = n.c + "
                "d; separation = |c_i - c_j|; free gap subtracts each "
                "object's half-width",
        provenance="the poses of stage 30, all in the camera's own frame",
        data=D(("objects measured", len(objs), "count"),
               ("nearest object", round(min(o["range_m"] for o in objs), 4), "m"),
               ("furthest object", round(max(o["range_m"] for o in objs), 4), "m"),
               ("closest pair",
                round(min((p[2] for p in pairs), default=float("nan")), 4), "m"),
               ("tightest free gap",
                round(min((p[3] for p in pairs), default=float("nan")), 4), "m"),
               ("what the origin IS",
                "the wrist camera, so range is distance from the HAND"
                if wrist else "the scene camera across the room", "-")),
        tables={"each object": (
            ["#", "range from camera m", "height above the surface mm",
             "x m", "y m", "z m"],
            [[o["idx"], round(o["range_m"], 4),
              round(1000 * o["height_m"] / 2 + 0.0, 1),
              round(float(o["centre"][0]), 4), round(float(o["centre"][1]), 4),
              round(float(o["centre"][2]), 4)] for o in objs]),
                "between objects": (
            ["object A", "object B", "centre to centre m", "free gap m"], pairs)},
        numbers="%d objects, nearest %.3f m, closest pair %s"
                % (len(objs), min(o["range_m"] for o in objs),
                   ("%.3f m" % min(p[2] for p in pairs)) if pairs else "n/a"),
        note="On a WRIST camera the origin is the hand, so 'range' is already "
             "the distance the arm has to close -- that is the quantity the "
             "servo loop nulls. On the scene camera it is the distance from "
             "the camera, and turning that into a distance from the ARM needs "
             "the scene-camera-to-robot calibration, which this repository "
             "does not have measured.",
        cols=1, panel_w=980, panel_h=680)


@stage(32, "people", "People in the picture")
def st_people(fr):
    """MediaPipe Pose, run in .venv_pose because mediapipe is only there.

    A SEPARATE INTERPRETER ON PURPOSE. CLAUDE.md records that installing a
    package into .venv_vision once took real_calibration/check_all.py from
    4/4 to 2/4, so the environments here are left alone and this shells out.
    """
    import subprocess
    import tempfile
    venv = os.path.join(WS, ".venv_pose", "bin", "python")
    model = os.path.join(WS, "models", "mediapipe", "pose_landmarker_full.task")
    if not os.path.exists(venv):
        raise Refusal("no .venv_pose on this machine, and mediapipe is not "
                      "importable from the vision environment")
    if not os.path.exists(model):
        raise Refusal("no pose_landmarker_full.task under models/mediapipe")
    tmp = tempfile.mkdtemp(prefix="cvvis_pose_")
    png = os.path.join(tmp, "frame.png")
    cv2.imwrite(png, fr.colour)
    code = (
        "import json,sys,cv2,numpy as np;"
        "import mediapipe as mp;"
        "from mediapipe.tasks import python as mpp;"
        "from mediapipe.tasks.python import vision;"
        "b=mpp.BaseOptions(model_asset_path=%r);"
        "o=vision.PoseLandmarkerOptions(base_options=b,num_poses=4,"
        "min_pose_detection_confidence=0.5);"
        "d=vision.PoseLandmarker.create_from_options(o);"
        "img=cv2.imread(%r);"
        "m=mp.Image(image_format=mp.ImageFormat.SRGB,"
        "data=cv2.cvtColor(img,cv2.COLOR_BGR2RGB));"
        "r=d.detect(m);"
        "out=[[[p.x,p.y,p.z,p.visibility] for p in L] for L in (r.pose_landmarks or [])];"
        "print(json.dumps(out))" % (model, png))
    t0 = time.time()
    try:
        res = subprocess.run([venv, "-c", code], capture_output=True, text=True,
                             timeout=180)
    except Exception as exc:                                     # noqa: BLE001
        raise Refusal("the pose interpreter would not run: %s" % exc)
    dt = time.time() - t0
    line = [l for l in (res.stdout or "").strip().split("\n") if l.startswith("[")]
    if not line:
        raise Refusal("mediapipe returned nothing usable in %.1f s: %s"
                      % (dt, (res.stderr or "")[-300:]))
    people = json.loads(line[-1])
    if not people:
        raise Refusal("no person is in this frame. MediaPipe ran in %.1f s and "
                      "found nobody, which is an answer -- the wearer-tracking "
                      "safety case only ever makes the modelled body BIGGER, "
                      "so 'nobody seen' falls back to the mannequin rather "
                      "than to an empty scene." % dt)
    H, W = fr.colour.shape[:2]
    LINKS = [(11, 13), (13, 15), (12, 14), (14, 16), (11, 12), (11, 23),
             (12, 24), (23, 24), (23, 25), (25, 27), (24, 26), (26, 28)]
    NAMES = {11: "left shoulder", 12: "right shoulder", 13: "left elbow",
             14: "right elbow", 15: "left wrist", 16: "right wrist",
             23: "left hip", 24: "right hip"}
    vis = fr.colour.copy()
    rows = []
    dep = None
    if fr.depth_m is not None:
        try:
            dep = aligned_depth(fr)[0]
        except Exception:                                        # noqa: BLE001
            dep = None
    for pi, L in enumerate(people):
        for a, b in LINKS:
            if a < len(L) and b < len(L):
                pa = (int(L[a][0] * W), int(L[a][1] * H))
                pb = (int(L[b][0] * W), int(L[b][1] * H))
                cv2.line(vis, pa, pb, (0, 255, 180), 2, cv2.LINE_AA)
        for i, nm in NAMES.items():
            if i >= len(L):
                continue
            u, v = int(L[i][0] * W), int(L[i][1] * H)
            cv2.circle(vis, (u, v), 5, (0, 120, 255), -1)
            z = None
            if dep is not None and 0 <= v < dep.shape[0] and 0 <= u < dep.shape[1]:
                win = dep[max(0, v - 4):v + 5, max(0, u - 4):u + 5]
                val = win[np.isfinite(win) & (win > 0)]
                z = float(np.median(val)) if val.size else None
            rows.append([pi, nm, u, v, round(float(L[i][3]), 3),
                         round(z, 4) if z else "no depth"])
        cv2.putText(vis, "person %d" % pi,
                    (int(L[0][0] * W) - 30, max(20, int(L[0][1] * H) - 20)),
                    FONT, 0.6, (0, 255, 180), 2, cv2.LINE_AA)
    return figure(
        "32  PEOPLE IN THE PICTURE",
        [(vis, "%d person(s): shoulders, elbows, wrists, hips" % len(people))],
        formula="markerless pose landmarks; each landmark's range read from "
                "the depth at its own pixel (median of a 9x9 window)",
        provenance="MediaPipe PoseLandmarker (full), num_poses=4, run in "
                   ".venv_pose as a separate interpreter -- the same model "
                   "srl_perception/wearer_tracker_node.py uses, %.1f s" % dt,
        data=D(("people detected", len(people), "count"),
               ("landmarks per person", len(people[0]), "count"),
               ("inference time", round(dt, 2), "s"),
               ("depth attached to landmarks", dep is not None, "-")),
        tables={"body landmarks": (
            ["person", "landmark", "u px", "v px", "visibility", "range m"],
            rows)},
        numbers="%d person(s)" % len(people),
        note="This is the wearer-tracking path. The rule the whole safety "
             "case rests on: a camera may only ever make the modelled body "
             "BIGGER -- fuse() keeps whichever of the tracked and mannequin "
             "primitive is CLOSER to the robot, per part, so a body measured "
             "further away changes nothing and a lost track falls back rather "
             "than shrinking the person.",
        cols=1, panel_w=980, panel_h=680)


@stage(33, "self_view", "The robot in the picture, and what is NOT detected")
def st_self_view(fr):
    d = need_depth(fr)
    near = np.isfinite(d) & (d > 0) & (d < fr.band[0])
    vis = colourise_depth(d)
    vis[near] = (0, 0, 255)
    wrist = fr.name.endswith("gripper")
    rows = []
    n, lab, st_, ce = cv2.connectedComponentsWithStats(near.astype(np.uint8), 8)
    for i in range(1, n):
        a = int(st_[i, cv2.CC_STAT_AREA])
        if a < 50:
            continue
        sel = d[lab == i]
        sel = sel[np.isfinite(sel) & (sel > 0)]
        rows.append([a, round(float(sel.min()), 4), round(float(np.median(sel)), 4),
                     int(st_[i, cv2.CC_STAT_LEFT]), int(st_[i, cv2.CC_STAT_TOP])])
    return figure(
        "33  WHAT IS IN FRONT OF THE SENSOR, AND WHAT IS NOT DETECTED",
        [(vis, "returns nearer than the working gate, in red")],
        formula="{ pixels with 0 < depth < %.2f m } -- nearer than anything "
                "this camera works on" % fr.band[0],
        provenance=fr.provenance,
        data=D(("pixels nearer than the gate", int(near.sum()), "px"),
               ("that fraction", round(100 * float(near.mean()), 3), "%"),
               ("nearest return of all",
                round(float(d[np.isfinite(d) & (d > 0)].min()), 4)
                if (np.isfinite(d) & (d > 0)).any() else float("nan"), "m"),
               ("blobs of near return", len(rows), "count"),
               ("this camera is on the hand", wrist, "-"),
               ("near edge of this camera's band", fr.band[0], "m")),
        tables={"near-field blobs": (["area px", "nearest m", "median m",
                                      "box x", "box y"], rows)},
        numbers="%d px nearer than %.2f m" % (int(near.sum()), fr.band[0]),
        note="THE ROBOT ITSELF IS NOT DETECTED IN THE IMAGE, and no stage in "
             "this program claims to. Nothing in this repository finds an arm "
             "in a picture: the robot's pose comes from its own encoders "
             "through the URDF, and drawing it into a camera image needs the "
             "camera-to-robot extrinsic -- measured at ~8.5 deg wrong on the "
             "wrist, and never measured at all for the scene cameras. On a "
             "wrist camera the near-field blobs above are usually the "
             "gripper's own fingers entering the bottom of the frame, which "
             "is also why the last ~80 mm of a grasp is completed from the "
             "last good fix rather than from live vision.",
        cols=1, panel_w=980, panel_h=680)


HEAVY = {"fastsam", "area_filter", "segment_match", "yoloworld", "layers",
         "min_width", "nested", "grasp", "boxes3d", "distances", "people"}
PROMPTED = {"segment_match", "yoloworld"}


# --------------------------------------------------------------- the run loop

# ------------------------------------------------------- diagnosis and repair
#
# WHAT --fix IS ALLOWED TO DO, AND WHAT IT IS NOT.
#
# It may re-attach a USB camera over usbipd and it may start the arm's CAMERA
# driver, because both are reversible and neither touches the arm's control.
# It stops any camera driver IT started before the program exits, so the
# machine is left as it was found.
#
# It will NEVER start a Kortex bridge: the arm permits exactly ONE session and
# a program that opens one behind your back is how the next run finds the
# session leaked (HARD CONSTRAINT 2). It will never reboot the vision module
# either -- that needs the bridge stopped first and is a deliberate act, not a
# retry. Both are named in the refusal instead.

#: camera drivers this program started, so it can stop them again
_STARTED = []


def running_domains(only=None):
    """Which ROS_DOMAIN_ID the arm's own processes are on, read from /proc.

    ASKED, NOT ASSUMED. Measured on this rig 2026-08-30: both camera drivers
    and both bridges were live on domain 42 while this program defaulted to 7,
    so every topic was missing and the arms were plainly running. A domain
    mismatch publishes into the void in silence, and it is the first thing to
    suspect when a healthy stack has no topics.
    """
    found = {}
    for pat in (only or ("kinova_vision_node", "kortex_highlevel_bridge")):
        rc, out = _sh("pgrep -f %s || true" % pat)
        for pid in out.split():
            try:
                with open("/proc/%s/environ" % pid, "rb") as fh:
                    env = fh.read().decode("utf-8", "replace")
            except OSError:
                continue
            for kv in env.split("\0"):
                if kv.startswith("ROS_DOMAIN_ID="):
                    d = kv.split("=", 1)[1].strip()
                    if d:
                        found.setdefault(d, []).append("%s (pid %s)" % (pat, pid))
    return found


def _sh(cmd, timeout=180):
    import subprocess
    try:
        r = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True,
                           text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except Exception as exc:                                     # noqa: BLE001
        return 127, str(exc)


def port_open(ip, port, timeout=3.0):
    import socket
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except Exception:                                            # noqa: BLE001
        return False


def who_holds(path):
    """Which processes have this device open. V4L2 allows exactly one."""
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            for fd in os.listdir("/proc/%s/fd" % pid):
                if os.path.realpath("/proc/%s/fd/%s" % (pid, fd)) == path:
                    with open("/proc/%s/comm" % pid) as fh:
                        out.append("%s (pid %s)" % (fh.read().strip(), pid))
                    break
        except OSError:
            continue
    return sorted(set(out))


def fix_usb(log=print, settle_s=20):
    """Hand the USB cameras back into WSL, and wait for the nodes to appear.

    This is `scripts/usb_cameras.py --fix`, which also repairs the STALE case:
    after the usbipd service restarts Windows still believes the device is
    attached, `attach` refuses, and the obvious command does nothing.
    """
    before = sorted(glob.glob("/dev/video*"))
    log("    running scripts/usb_cameras.py --fix ...")
    rc, out = _sh([sys.executable, os.path.join(HERE, "usb_cameras.py"), "--fix"],
                  timeout=240)
    tail = [ln for ln in out.strip().split("\n") if ln.strip()][-4:]
    for ln in tail:
        log("      %s" % ln[:160])
    t0 = time.time()
    while time.time() - t0 < settle_s:
        now = sorted(glob.glob("/dev/video*"))
        if now and now != before:
            log("    /dev/video* went from %d node(s) to %d after %.0f s"
                % (len(before), len(now), time.time() - t0))
            time.sleep(1.5)         # let the driver finish enumerating
            return True, "usbipd attach: %s" % (tail[-1] if tail else "rc %d" % rc)
        time.sleep(0.5)
    return (bool(glob.glob("/dev/video*")),
            "usbipd attach ran (rc %d) and /dev/video* did not change" % rc)


def diagnose_gripper(arm, domain):
    """Which of the four gripper-camera faults this is, by measurement."""
    ip = ARM_IP[arm]
    rc, _ = _sh(["ping", "-c", "2", "-W", "2", ip], timeout=12)
    pings = rc == 0
    rtsp = port_open(ip, 554)
    api = port_open(ip, 10000)
    _, cam = _sh("pgrep -af 'kinova_vision_node.*%s_camera' || true" % arm)
    _, br = _sh("pgrep -af 'kortex_highlevel_bridge_%s' || true" % arm)
    return dict(ip=ip, pings=pings, rtsp_554=rtsp, api_10000=api,
                camera_node=cam.strip(), bridge=br.strip(), domain=domain)


def gripper_verdict(d):
    if not d["pings"]:
        return ("the arm does not answer a ping at %s. It is off, unplugged, "
                "or this box is not on its network -- no camera fix applies."
                % d["ip"])
    if not d["rtsp_554"]:
        return ("the arm pings and port 554 is NOT LISTENING, so the vision "
                "MODULE is wedged -- not the driver, not the network. The "
                "camera node would retry for ever and never connect. Recover "
                "it deliberately, with the bridge stopped: "
                ".kortex_venv/bin/python scripts/reboot_vision_module.py %s"
                % ("left" if d["ip"].endswith(".10") else "right"))
    if not d["camera_node"]:
        return ("the arm pings and its RTSP server is listening, but no "
                "kinova_vision_node is running for this arm.")
    return ("the driver is running and the arm is serving RTSP, so this is a "
            "graph problem: the node is probably on a different "
            "ROS_DOMAIN_ID from this terminal (%s)." % d["domain"])


def start_gripper_camera(arm, log=print, wait_s=45):
    """Start the arm's CAMERA driver only -- never a bridge. Stopped on exit.

    Exactly the launch `bringup_arm.sh` uses, frame ids included: the driver's
    own static TF disagrees with srl_description, and both arms' drivers
    publish the SAME unprefixed camera_link, so left and right collide in one
    TF tree unless they are renamed.
    """
    import subprocess
    ip = ARM_IP[arm]
    logdir = os.path.join(os.environ.get("TMPDIR", "/tmp"), "srl_bringup")
    os.makedirs(logdir, exist_ok=True)
    logf = os.path.join(logdir, "cam_%s_cvvis.log" % arm)
    cmd = ("source /opt/ros/jazzy/setup.bash && "
           "source $HOME/kortex_ws/install/setup.bash && "
           "exec ros2 launch kinova_vision kinova_vision.launch.py "
           "device:=%s camera:=%s_camera launch_tf:=false "
           "camera_link_frame_id:=%s_camera_link "
           "color_frame_id:=%s_camera_color_frame "
           "depth_frame_id:=%s_camera_depth_frame" % (ip, arm, arm, arm, arm))
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(env.get("ROS_DOMAIN_ID", "7"))
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "SHM"
    log("    starting the %s camera driver (NOT a bridge) -> %s" % (arm, logf))
    fh = open(logf, "w")
    proc = subprocess.Popen(["bash", "-lc", "set +u; " + cmd], stdout=fh,
                            stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)
    _STARTED.append((arm, proc, logf))
    t0 = time.time()
    while time.time() - t0 < wait_s:
        if proc.poll() is not None:
            return False, "the launch exited immediately; see %s" % logf
        try:
            with open(logf) as r:
                txt = r.read()
        except OSError:
            txt = ""
        if txt.count("Stream started") >= 2:
            log("    colour and depth streams started after %.0f s"
                % (time.time() - t0))
            time.sleep(2.0)
            return True, "started kinova_vision for %s" % arm
        time.sleep(1.0)
    return False, ("the camera driver did not report two started streams in "
                   "%.0f s; see %s" % (wait_s, logf))


def stop_started(log=print):
    """SIGINT anything this program started, and say so."""
    import signal
    for arm, proc, logf in _STARTED:
        if proc.poll() is not None:
            continue
        log("stopping the %s camera driver this program started (pid %d)"
            % (arm, proc.pid))
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:                                        # noqa: BLE001
            proc.send_signal(signal.SIGINT)
        for _ in range(20):
            if proc.poll() is not None:
                break
            time.sleep(0.5)
    del _STARTED[:]


def repair(name, args, log=print):
    """Try to make `name` work. Returns a list of what was actually done."""
    done = []
    if name in ("scene_rs", "scene_hd"):
        okd, why = fix_usb(log)
        done.append(why)
        if name == "scene_hd":
            for dev in sorted(glob.glob("/dev/video*")):
                holders = who_holds(dev)
                if holders:
                    done.append("%s is held by %s -- V4L2 allows ONE capture "
                                "client" % (dev, ", ".join(holders)))
        return done
    if name in ("left_gripper", "right_gripper"):
        arm = name.split("_")[0]
        d = diagnose_gripper(arm, args.domain)
        verdict = gripper_verdict(d)
        log("    %s" % verdict)
        done.append(verdict)
        live = running_domains()
        if live and str(args.domain) not in live:
            newd = sorted(live)[0]
            log("    the arm's processes are on ROS_DOMAIN_ID=%s, not %s -- "
                "switching to it" % (newd, args.domain))
            done.append("switched from ROS_DOMAIN_ID=%s to %s, where the "
                        "arm's own processes are" % (args.domain, newd))
            args.domain = int(newd)
            _ros_env(args.domain)
        done.append("ping %s, port 554 %s, port 10000 %s, driver %s"
                    % ("ok" if d["pings"] else "FAILS",
                       "open" if d["rtsp_554"] else "SHUT",
                       "open" if d["api_10000"] else "shut",
                       "running" if d["camera_node"] else "not running"))
        if d["pings"] and d["rtsp_554"] and not d["camera_node"]:
            ok, why = start_gripper_camera(arm, log)
            done.append(why)
        return done
    return done


def _acquire_once(name, args):
    if name in ("left_gripper", "right_gripper"):
        arm = name.split("_")[0]
        if args.gripper in ("auto", "ros"):
            try:
                return grab_gripper_ros(arm, args.domain, args.wait)
            except Refusal as first:
                if args.gripper == "ros":
                    raise
                try:
                    return grab_gripper_rtsp(arm)
                except Refusal as second:
                    raise Refusal("%s  --- then the direct RTSP fallback also "
                                  "failed: %s" % (first, second))
        return grab_gripper_rtsp(arm)
    if name == "scene_rs":
        return grab_scene_rs(args.rs_frames, not args.no_rs_rotate)
    if name == "scene_hd":
        try:
            return grab_scene_hd()
        except Refusal as direct:
            # The usual cause is not a missing camera: it is that
            # scene_camera_node already has the device open.
            try:
                return grab_scene_hd_ros(args.domain, args.wait)
            except Refusal as viaros:
                raise Refusal("%s  --- and through scene_camera_node's topic: "
                              "%s" % (direct, viaros))
    raise Refusal("unknown source %r" % name)


def acquire(name, args, log=print):
    """One frame from `name`, repairing what is safely repairable if asked."""
    try:
        return _acquire_once(name, args), []
    except Refusal as first:
        if not args.fix:
            raise Refusal(
                "%s\n\nRun again with --fix and this program will attempt "
                "the repair itself (re-attach the USB cameras over usbipd, "
                "start the arm's camera driver if its RTSP server is "
                "listening). It will not start a Kortex bridge or reboot a "
                "vision module -- those are deliberate acts." % first)
        log("  --fix: attempting a repair ...")
        done = repair(name, args, log)
        try:
            fr = _acquire_once(name, args)
            log("  --fix: recovered.")
            return fr, done
        except Refusal as second:
            raise Refusal("after attempting %s -- it still refuses: %s"
                          % ("; ".join(done) or "nothing", second))


def save_raw(fr, outdir):
    d = os.path.join(outdir, "raw", fr.name)
    os.makedirs(d, exist_ok=True)
    cv2.imwrite(os.path.join(d, "colour.png"), fr.colour)
    meta = dict(name=fr.name, provenance=fr.provenance, notes=fr.notes,
                K_colour=fr.K_c, K_depth=fr.K_d,
                depth_is_aligned=fr.depth_is_aligned,
                no_depth_reason=fr.no_depth_reason,
                colour_shape=list(fr.colour.shape))
    if fr.depth_m is not None:
        np.save(os.path.join(d, "depth.npy"), fr.depth_m.astype(np.float32))
        cv2.imwrite(os.path.join(d, "depth_view.png"), colourise_depth(fr.depth_m))
        meta["depth_shape"] = list(fr.depth_m.shape)
    with open(os.path.join(d, "intrinsics.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    return d


def load_raw(d):
    with open(os.path.join(d, "intrinsics.json")) as fh:
        meta = json.load(fh)
    colour = cv2.imread(os.path.join(d, "colour.png"))
    if colour is None:
        raise Refusal("no colour.png in %s" % d)
    dp = os.path.join(d, "depth.npy")
    depth = np.load(dp) if os.path.exists(dp) else None
    return Frame(meta["name"], colour, depth, meta.get("K_colour"),
                 meta.get("K_depth"), meta.get("depth_is_aligned", False),
                 provenance="REPLAY of " + meta.get("provenance", d),
                 notes=meta.get("notes", []),
                 no_depth_reason=meta.get("no_depth_reason", ""))


def _csv(path, headers, rows):
    """A CSV with no dependencies, quoting anything containing a comma."""
    def cell(v):
        t = fmt_value(v) if not isinstance(v, str) else v
        return '"%s"' % t.replace('"', '""') if ("," in t or '"' in t) else t
    with open(path, "w") as fh:
        fh.write(",".join(str(h) for h in headers) + "\n")
        for r in rows:
            fh.write(",".join(cell(c) for c in r) + "\n")


def run_stages(fr, outdir, prompt, fast=False, only=None):
    sdir = os.path.join(outdir, "stages", fr.name)
    ddir = os.path.join(outdir, "data", fr.name)
    os.makedirs(sdir, exist_ok=True)
    os.makedirs(ddir, exist_ok=True)
    out, figs, flat = [], [], []
    for s in sorted(STAGES, key=lambda x: x["num"]):
        if only and s["slug"] not in only:
            continue
        t0 = time.time()
        status, reason, img = "ok", "", None
        _LAST_FIGURE.clear()
        try:
            if fast and s["slug"] in HEAVY:
                raise Refusal("--fast was given, so the segmentation, "
                              "open-vocabulary and per-instance stages were "
                              "skipped. They are the slow ones; drop --fast "
                              "for the full set.")
            img = (s["fn"](fr, prompt) if s["slug"] in PROMPTED
                   else s["fn"](fr))
        except ByDesign as exc:
            status, reason = "by_design", str(exc)
            img = refusal_figure("%02d  %s" % (s["num"], s["title"].upper()),
                                 reason, by_design=True)
        except Refusal as exc:
            status, reason = "refused", str(exc)
            img = refusal_figure("%02d  %s" % (s["num"], s["title"].upper()),
                                 reason)
        except Exception as exc:                                 # noqa: BLE001
            status = "error"
            reason = "%s: %s\n%s" % (type(exc).__name__, exc,
                                     traceback.format_exc().strip().split("\n")[-3])
            img = refusal_figure("%02d  %s" % (s["num"], s["title"].upper()),
                                 "THIS STAGE CRASHED, which is a defect in "
                                 "the program and not a result: " + reason)
        dt = time.time() - t0
        path = os.path.join(sdir, "%02d_%s.png" % (s["num"], s["slug"]))
        cv2.imwrite(path, img)
        figs.append(img)
        rec = dict(num=s["num"], slug=s["slug"], title=s["title"],
                   status=status, seconds=round(dt, 2),
                   figure=os.path.relpath(path, outdir))
        if reason:
            rec["reason"] = reason
        for k in ("formula", "provenance", "numbers", "note"):
            if _LAST_FIGURE.get(k):
                rec[k] = _LAST_FIGURE[k]

        # THE NUMBERS LEAVE THE PICTURE. Every measurement is written as a
        # value with a unit, per stage as JSON, per run as one flat CSV, and
        # every per-object record as its own CSV.
        dat = _LAST_FIGURE.get("data") or []
        tabs = _LAST_FIGURE.get("tables") or {}
        rec["data"] = dat
        rec["tables"] = {}
        base = "%02d_%s" % (s["num"], s["slug"])
        if dat or tabs:
            with open(os.path.join(ddir, base + ".json"), "w") as fh:
                json.dump(dict(source=fr.name, stage=s["slug"], num=s["num"],
                               title=s["title"], status=status,
                               formula=rec.get("formula", ""),
                               measurements=dat,
                               tables={k: dict(headers=v[0], rows=v[1])
                                       for k, v in tabs.items()}),
                          fh, indent=2, default=str)
        for d in dat:
            flat.append([fr.name, s["num"], s["slug"], d["quantity"],
                         d["value"], d["unit"]])
        for tname, (thead, trows) in tabs.items():
            slug = "".join(c if c.isalnum() else "_" for c in tname).strip("_")
            cp = os.path.join(ddir, "%s__%s.csv" % (base, slug.lower()))
            _csv(cp, thead, trows)
            rec["tables"][tname] = dict(rows=len(trows),
                                        csv=os.path.relpath(cp, outdir),
                                        headers=list(thead))
        out.append(rec)
        mark = {"ok": "ok", "refused": "REFUSED", "by_design": "by design",
                "error": "ERROR"}[status]
        print("  %-14s %02d/%02d %-34s %-9s %5.1f s"
              % (fr.name, s["num"], len(STAGES), s["slug"], mark, dt),
              flush=True)
        if status in ("refused", "error"):
            print("      -> %s" % reason.split("\n")[0][:150], flush=True)
    _csv(os.path.join(ddir, "measurements.csv"),
         ["source", "stage_num", "stage", "quantity", "value", "unit"], flat)
    sheet = contact_sheet(figs, fr)
    spath = os.path.join(outdir, "sheet_%s.png" % fr.name)
    cv2.imwrite(spath, sheet)
    return out, os.path.relpath(spath, outdir)


def contact_sheet(figs, fr, cols=3, tile_w=760):
    tiles = []
    for f in figs:
        s = tile_w / float(f.shape[1])
        tiles.append(cv2.resize(f, (tile_w, max(1, int(f.shape[0] * s)))))
    rows, i = [], 0
    while i < len(tiles):
        row = tiles[i:i + cols]
        hh = max(t.shape[0] for t in row)
        row = [np.vstack([t, np.full((hh - t.shape[0], t.shape[1], 3), PAPER,
                                     np.uint8)]) for t in row]
        while len(row) < cols:
            row.append(np.full((hh, tile_w, 3), PAPER, np.uint8))
        rows.append(np.hstack(row))
        i += cols
    body = np.vstack(rows) if rows else np.full((100, tile_w, 3), PAPER, np.uint8)
    head = np.full((70, body.shape[1], 3), (238, 238, 234), np.uint8)
    cv2.putText(head, "%s -- every vision stage, in order" % fr.name, (16, 44),
                FONT, 1.0, INK, 2, cv2.LINE_AA)
    return np.vstack([head, body])


def write_report(outdir, run, path="report.md"):
    L = []
    A = L.append
    A("# Computer vision stages, %s" % run["started"])
    A("")
    A("Written by `scripts/cv_pickpose_visuals.py`. The arms were NOT "
      "commanded; this program has no publisher, no service client and no "
      "Kortex session.")
    A("")
    p = run.get("pose", {})
    A("## The arms")
    A("")
    if not p.get("measured"):
        A("**The pose was not measured.** %s" % p.get("why", ""))
    else:
        A("Read from `/real/joint_states` (%d distinct source stamps), "
          "compared with the saved ideal pick pose." % p.get("distinct_stamps", 0))
        A("")
        A("| arm | reference | joints read | worst deviation | at the pick pose |")
        A("| --- | --- | --- | --- | --- |")
        for arm, a in p.get("arms", {}).items():
            A("| %s | `%s` | %d of 7 | %s | %s |"
              % (arm, a["reference"], a["n_joints_read"],
                 ("%.2f deg" % a["worst_deviation_deg"])
                 if a["worst_deviation_deg"] is not None else "-",
                 "YES" if a["at_pick_pose"] else "**NO**"))
        A("")
        for arm, a in p.get("arms", {}).items():
            rows = [r for r in a["joints"] if r["delta_deg"] is not None]
            if not rows:
                continue
            A("<details><summary>%s, joint by joint</summary>" % arm)
            A("")
            A("| joint | measured (deg) | pick pose (deg) | delta (deg) |")
            A("| --- | --- | --- | --- |")
            for r in rows:
                A("| %s | %.3f | %.3f | %+.3f |"
                  % (r["joint"], r["live_deg"], r["want_deg"], r["delta_deg"]))
            A("")
            A("</details>")
            A("")
    for src in run["sources"]:
        A("## %s" % src["name"])
        A("")
        if src.get("error"):
            A("**No frame.** %s" % src["error"])
            A("")
            continue
        A("%s" % src["provenance"])
        A("")
        for n in src.get("notes", []):
            A("- %s" % n)
        A("")
        A("Contact sheet: `%s`" % src.get("sheet", ""))
        A("")
        for st in src["stages"]:
            A("### %02d  %s  [%s]" % (st["num"], st["title"], st["status"]))
            A("")
            A("![%s](%s)" % (st["slug"], st["figure"]))
            A("")
            if st.get("formula"):
                A("- **formula** `%s`" % st["formula"])
            if st.get("provenance"):
                A("- **source** %s" % st["provenance"])
            if st.get("numbers"):
                A("- **summary** %s" % st["numbers"])
            if st.get("data"):
                A("")
                A("| quantity | value | unit |")
                A("| --- | ---: | --- |")
                for d in st["data"]:
                    A("| %s | %s | %s |" % (d["quantity"], fmt_value(d["value"]),
                                            d["unit"]))
                A("")
            for tname, t in (st.get("tables") or {}).items():
                A("")
                A("*%s* -- %d rows, full data in `%s`"
                  % (tname, t["rows"], t["csv"]))
                A("")
            if st.get("note"):
                A("- **note** %s" % st["note"])
            if st.get("reason"):
                A("- **%s** %s" % (st["status"], st["reason"]))
            A("")
    with open(os.path.join(outdir, path), "w") as fh:
        fh.write("\n".join(L) + "\n")


# ------------------------------------------------------------- the self-test

def _synthetic(seed=0, noise_m=0.0003):
    """A CONSTRUCTED depth frame: a plane at a known pose carrying a 40 mm cube.

    Constructed, not rendered: every number below is chosen, so the right
    answer is known to the micrometre before the estimators are run. The
    colour image IS drawn, and the colour stages exercised on it are plumbing
    checks -- they prove the code runs, they are not evidence about anything.
    """
    W, H = 640, 480
    K = (600.0, 600.0, 320.0, 240.0)
    fx, fy, cx, cy = K
    n = np.array([0.0, -0.9, -0.4])
    n = n / np.linalg.norm(n)
    p0 = np.array([0.0, 0.15, 0.35])
    d = -float(n @ p0)
    if d < 0:
        n, d = -n, -d
    # cube: 40 mm, resting on the plane, turned 45 deg in it
    e1 = np.array([1.0, 0.0, 0.0])
    e1 = e1 - n * float(e1 @ n)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    th = math.radians(45.0)
    a1 = e1 * math.cos(th) + e2 * math.sin(th)
    a2 = -e1 * math.sin(th) + e2 * math.cos(th)
    half = 0.020
    foot = p0 + e1 * 0.010 + e2 * 0.005
    centre = foot + n * half
    R = np.stack([a1, a2, n], 1)                       # columns
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    b = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, float)], -1)
    nb = b @ n
    t_plane = np.where(np.abs(nb) > 1e-9, -d / np.where(np.abs(nb) > 1e-9, nb, 1),
                       -1.0)
    A = b @ R                                           # (H, W, 3) local dirs
    Bc = R.T @ centre
    lo = np.full(A.shape[:2], -np.inf)
    hi = np.full(A.shape[:2], np.inf)
    for i in range(3):
        ai = A[:, :, i]
        safe = np.abs(ai) > 1e-9
        t1 = np.where(safe, (Bc[i] - half) / np.where(safe, ai, 1), -np.inf)
        t2 = np.where(safe, (Bc[i] + half) / np.where(safe, ai, 1), np.inf)
        lo = np.maximum(lo, np.minimum(t1, t2))
        hi = np.minimum(hi, np.maximum(t1, t2))
    hit = (hi >= lo) & (lo > 0)
    depth = np.where(t_plane > 0, t_plane, 0.0)
    depth = np.where(hit, np.minimum(np.where(depth > 0, depth, np.inf), lo),
                     depth)
    depth[~np.isfinite(depth)] = 0.0
    rng = np.random.default_rng(seed)
    depth = np.where(depth > 0, depth + rng.normal(0, noise_m, depth.shape), 0.0)
    colour = np.full((H, W, 3), (185, 185, 180), np.uint8)
    colour[depth > 0] = (170, 170, 165)
    colour[hit] = (40, 165, 55)
    truth = dict(n=n, d=d, cube_centre=centre, cube_size=2 * half,
                 cube_pixels=int(hit.sum()), K=K)
    fr = Frame("self_test", colour, depth.astype(np.float32), K, K,
               depth_is_aligned=True,
               provenance="CONSTRUCTED, not measured: a plane at n=(%.4f, "
                          "%.4f, %.4f) d=%.4f carrying a 40 mm cube turned 45 "
                          "deg, ray-traced analytically with %.1f mm of "
                          "Gaussian range noise" % (n[0], n[1], n[2], d,
                                                    noise_m * 1000),
               notes=["the COLOUR image is drawn, so the colour stages here "
                      "are plumbing checks and not evidence"])
    return fr, truth


def self_test(outdir=None, verbose=True, fast=False):
    ok = True
    results = []

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        results.append(dict(check=name, passed=bool(cond), detail=str(detail)))
        if verbose:
            print("  %-64s %s%s" % (name, "ok" if cond else "FAIL",
                                    "" if cond else "   " + str(detail)))

    fr, truth = _synthetic()
    P, uv = depth_cloud(fr)
    check("the constructed frame lifts to a cloud", len(P) > 50000, len(P))

    # the deprojection is its own inverse
    q = np.array([[0.031, -0.017, 0.402]])
    pr = _proj(q, fr.K_c)[0]
    back = np.array([(pr[0] - fr.K_c[2]) * q[0, 2] / fr.K_c[0],
                     (pr[1] - fr.K_c[3]) * q[0, 2] / fr.K_c[1], q[0, 2]])
    check("project then deproject returns the same point (< 1 um)",
          float(np.linalg.norm(back - q[0])) < 1e-6,
          float(np.linalg.norm(back - q[0])))

    pl = plane_depth(fr)
    ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(pl["n"], truth["n"]))))))
    check("the fitted plane normal is within 0.5 deg of the constructed one",
          ang < 0.5, "%.4f deg" % ang)
    check("the fitted offset is within 2 mm of the constructed one",
          abs(pl["d"] - truth["d"]) < 0.002,
          "%.2f mm" % (1000 * abs(pl["d"] - truth["d"])))
    check("the plane RMS is under 1 mm on 0.3 mm of noise",
          pl["rms_mm"] < 1.0, "%.3f mm" % pl["rms_mm"])

    # the cube: everything standing off the plane
    h = pl["h_all"]
    cube = h > 0.005
    check("the cube's points are found above the plane", int(cube.sum()) > 400,
          int(cube.sum()))
    top_mm = 1000 * float(h[cube].max())
    check("the cube measures 40 mm tall, within 3 mm",
          abs(top_mm - 40.0) < 3.0, "%.2f mm" % top_mm)

    pts = P[cube]
    up = np.asarray(pl["n"], float)
    c_meas, ext, w_min = _measure_instance(pts, up)
    check("rotating calipers measure the 40 mm cube within 5 mm",
          abs(1000 * w_min - 40.0) < 5.0, "%.2f mm" % (1000 * w_min))
    tmp = np.array([1.0, 0.0, 0.0])
    e1 = tmp - up * float(tmp @ up)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    rel = pts - pts.mean(0)
    flat = np.stack([rel @ e1, rel @ e2], 1)
    aabb = 1000 * float(np.ptp(flat, axis=0).min())
    check("THE CONTROL: the axis-aligned box reads the turned cube over 48 mm",
          aabb > 48.0, "%.2f mm" % aabb)
    # THE TWO CENTRE ESTIMATORS FIX DIFFERENT AXES, AND NEITHER FIXES BOTH.
    # Decomposed against the constructed truth rather than reported as one
    # number, because one number hides which half is wrong.
    err = np.asarray(c_meas) - truth["cube_centre"]
    along = float(err @ up)
    inplane = float(np.linalg.norm(err - up * along))
    check("mid-extent puts the centre within 2 mm ACROSS the surface",
          inplane < 0.002, "%.2f mm" % (1000 * inplane))
    check("THE CONTROL: along the normal it is out by more than 10 mm, "
          "because that axis is still the MEAN of a one-sided cloud",
          abs(along) > 0.010, "%.2f mm" % (1000 * along))
    from srl_perception.rgbd_grasp import shell_corrected_centre
    c_shell, top_m = shell_corrected_centre(pts, -up * pl["d"], up)
    es = np.asarray(c_shell) - truth["cube_centre"]
    along_s = float(es @ up)
    inplane_s = float(np.linalg.norm(es - up * along_s))
    check("the shell correction fixes the normal axis to within 2 mm",
          abs(along_s) < 0.002, "%.2f mm" % (1000 * along_s))
    check("THE CONTROL: and it is worse ACROSS the surface, because its "
          "footprint centroid is a shell's", inplane_s > 0.002,
          "%.2f mm" % (1000 * inplane_s))
    combined = (np.asarray(c_meas) - up * along) + up * along_s
    ec = float(np.linalg.norm(combined - truth["cube_centre"]))
    check("mid-extent across, shell correction along: within 2 mm in FULL 3-D",
          ec < 0.002, "%.2f mm" % (1000 * ec))

    # the box the 3-D stages actually publish, against the same truth
    op = object_pose(pts, up, pl["d"])
    check("the oriented box measures the 40 mm cube to within 3 mm on all "
          "three sides",
          max(abs(1000 * op["length_m"] - 40), abs(1000 * op["width_m"] - 40),
              abs(1000 * op["height_m"] - 40)) < 3.0,
          "%.1f x %.1f x %.1f mm" % (1000 * op["length_m"], 1000 * op["width_m"],
                                     1000 * op["height_m"]))
    berr = 1000 * float(np.linalg.norm(np.asarray(op["centre"])
                                       - truth["cube_centre"]))
    check("and its centre is within 2 mm of the constructed one", berr < 2.0,
          "%.2f mm" % berr)
    check("its range matches the constructed centre's",
          abs(op["range_m"] - float(np.linalg.norm(truth["cube_centre"]))) < 0.002,
          "%.4f m" % op["range_m"])

    # the mode estimator, against the same truth
    s = P @ np.asarray(pl["n"], float)
    lo = float(s.min())
    cnt = np.bincount(np.floor((s - lo) / 0.002).astype(int))
    k = int(np.argmax(cnt))
    band = np.abs(s - (lo + (k + 0.5) * 0.002)) <= 0.006
    z_hat = float(s[band].mean())
    check("the mode estimator lands within 3 mm of the plane offset",
          abs(z_hat + pl["d"]) < 0.003, "%.2f mm" % (1000 * abs(z_hat + pl["d"])))

    # A CHECK THAT CANNOT FAIL IS NOT A CHECK: break the input on purpose.
    bad = Frame("self_test_broken", fr.colour, fr.depth_m * 1.5, fr.K_c,
                fr.K_c, depth_is_aligned=True, provenance="deliberately broken")
    bpl = plane_depth(bad)
    btop = 1000 * float(bpl["h_all"][bpl["h_all"] > 0.005].max())
    check("THE CONTROL: with the depth scaled 1.5x the height check FAILS",
          abs(btop - 40.0) > 3.0, "%.2f mm" % btop)

    # the colour half is plumbing: it must run, and it must select the cube
    raw, opened = green_masks(fr)
    n_cc = cv2.connectedComponents(opened)[0] - 1
    check("PLUMBING: the drawn cube is one green component", n_cc == 1, n_cc)

    if outdir:
        os.makedirs(outdir, exist_ok=True)
        recs, sheet = run_stages(fr, outdir, "the green cube", fast=fast)
        bad_stages = [r for r in recs if r["status"] == "error"]
        check("every stage produced a figure without crashing",
              not bad_stages, ", ".join(r["slug"] for r in bad_stages))
        with open(os.path.join(outdir, "self_test.json"), "w") as fh:
            json.dump(dict(checks=results, stages=recs), fh, indent=2)
    if verbose:
        print("\nself-test %s  (%d checks, %d failed)"
              % ("PASSED" if ok else "FAILED", len(results),
                 sum(1 for r in results if not r["passed"])))
    return ok


# -------------------------------------------------------------------- the CLI

BANNER = """
cv_pickpose_visuals -- every vision stage, as a picture

  THIS PROGRAM NEVER MOVES THE ARMS.  It has no publisher, no service client
  and no Kortex session.  Park the arms at the pick pose yourself; all this
  does is read where they are and say how far that is from
  config/pick_pose_ideal_*.txt.
"""


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Run every computer-vision stage on the gripper and scene "
                    "cameras and write one labelled figure per stage.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="The arms are never commanded. Park them at the pick pose "
               "first: bash scripts/bringup_arm.sh left (and right), then "
               "hand-guide or drive them there.")
    ap.add_argument("--sources", default="auto",
                    help="comma separated, from %s, or 'auto' for all four"
                         % ",".join(SOURCES))
    ap.add_argument("--out", default=None,
                    help="output directory (default recordings/vision_thesis/"
                         "<date>_<time>)")
    ap.add_argument("--prompt", default="the green cube",
                    help="the sentence the open-vocabulary stages are asked")
    ap.add_argument("--domain", default="auto",
                    help="ROS_DOMAIN_ID for the gripper cameras and the joint "
                         "states. 'auto' (the default) reads it from the arm's "
                         "own running processes, because a domain mismatch is "
                         "silent and is the usual cause of 'the stack is up "
                         "and there are no topics'.")
    ap.add_argument("--wait", type=float, default=12.0,
                    help="seconds to wait for the camera topics")
    ap.add_argument("--gripper", choices=("auto", "ros", "rtsp"), default="auto",
                    help="where gripper frames come from. 'auto' tries the ROS "
                         "topics first because the driver holds the RTSP "
                         "streams and Kinova allows only two clients.")
    ap.add_argument("--rs-frames", type=int, default=12,
                    help="RealSense frames to average over valid pixels")
    ap.add_argument("--no-rs-rotate", action="store_true",
                    help="the RealSense is mounted upside down and is rotated "
                         "180 deg by default; this turns that off")
    ap.add_argument("--fix", action="store_true",
                    help="attempt the repair when a camera does not answer: "
                         "re-attach the USB cameras over usbipd, and start the "
                         "arm's CAMERA driver when its RTSP server is "
                         "listening. Never starts a Kortex bridge and never "
                         "reboots a vision module. Anything it starts, it "
                         "stops again on the way out.")
    ap.add_argument("--keep-started", action="store_true",
                    help="with --fix, leave a camera driver this program "
                         "started running after it exits")
    ap.add_argument("--no-pose", action="store_true",
                    help="skip reading the arms' joint states")
    ap.add_argument("--fast", action="store_true",
                    help="skip FastSAM, YOLO-World and the per-instance stages")
    ap.add_argument("--only", default="",
                    help="comma separated stage slugs, for iterating on one "
                         "figure")
    ap.add_argument("--replay", default="",
                    help="re-run every stage on the raw frames saved by an "
                         "earlier run, with no camera")
    ap.add_argument("--self-test", action="store_true",
                    help="check the arithmetic against a constructed frame "
                         "whose answer is known. No camera needed.")
    args = ap.parse_args(argv)

    print(BANNER)
    # TWO DIFFERENT QUESTIONS, AND THEY CAN HAVE DIFFERENT ANSWERS.
    #
    # The camera topics come from kinova_vision_node; the joint states come
    # from kortex_highlevel_bridge. Measured on this rig 2026-08-30: the
    # cameras were on domain 42 while a freshly started pair of bridges was on
    # domain 0. Picking ONE domain for both loses whichever half it does not
    # pick -- and the first version of this picked by sorting the domains as
    # STRINGS, so "0" beat "42" and it silently chose the half with no
    # cameras. Each half is now asked of the processes that actually serve it,
    # and a split is reported rather than resolved by luck.
    pose_domain = None
    if str(args.domain).lower() == "auto":
        cams = running_domains(only=("kinova_vision_node",))
        brs = running_domains(only=("kortex_highlevel_bridge",))
        if cams:
            args.domain = int(sorted(cams, key=lambda d: -len(cams[d]))[0])
            print("ROS_DOMAIN_ID=%d for the cameras, read from the camera "
                  "drivers themselves (%s)"
                  % (args.domain, "; ".join(cams[str(args.domain)][:2])))
        elif brs:
            args.domain = int(sorted(brs, key=lambda d: -len(brs[d]))[0])
            print("no camera driver is running; using ROS_DOMAIN_ID=%d from "
                  "the bridges" % args.domain)
        else:
            args.domain = int(os.environ.get("ROS_DOMAIN_ID", "7"))
            print("no arm process is running, so ROS_DOMAIN_ID=%d is a guess "
                  "from the environment" % args.domain)
        if brs:
            pose_domain = int(sorted(brs, key=lambda d: -len(brs[d]))[0])
        if cams and brs and set(cams) != set(brs):
            print("\n  *** THE STACK IS SPLIT ACROSS DOMAINS ***")
            print("      camera drivers on %s; bridges on %s."
                  % (", ".join(sorted(cams)), ", ".join(sorted(brs))))
            print("      Nothing on one domain can see the other, and neither")
            print("      half reports the other as missing. Reading the "
                  "cameras on %d" % args.domain)
            print("      and the joint states on %d for this run, but the "
                  "stack should be" % (pose_domain if pose_domain is not None
                                       else args.domain))
            print("      brought up on ONE domain.\n")
    else:
        args.domain = int(args.domain)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = args.out or os.path.join(WS, "recordings", "vision_thesis", stamp)
    os.makedirs(outdir, exist_ok=True)
    only = set(s.strip() for s in args.only.split(",") if s.strip())

    if args.self_test:
        good = self_test(outdir=os.path.join(outdir, "self_test"),
                         fast=args.fast)
        print("\nwritten to %s" % os.path.join(outdir, "self_test"))
        return 0 if good else 1

    run = dict(started=datetime.now().isoformat(timespec="seconds"),
               stamp=stamp, argv=sys.argv, interpreter=sys.executable,
               out=outdir, prompt=args.prompt, sources=[],
               commanded_motion=False,
               motion_note="This program contains no publisher, service "
                           "client or Kortex session. The arms were not moved.")

    # THE POSE FIRST, so it describes the arms as they were when the cameras
    # were read rather than after somebody nudged something.
    if args.no_pose or args.replay:
        run["pose"] = dict(measured=False, why="--no-pose was given"
                           if args.no_pose else "replay of saved frames")
    else:
        pd = pose_domain if pose_domain is not None else args.domain
        print("reading the arms' joint states on ROS_DOMAIN_ID=%d "
              "(subscribe only) ..." % pd)
        run["pose"] = measure_pose(pd)
        run["pose"]["domain"] = pd
        p = run["pose"]
        if p.get("measured"):
            for arm, a in p["arms"].items():
                w = a["worst_deviation_deg"]
                print("  %-5s worst deviation from the ideal pick pose: %s  %s"
                      % (arm, "%.2f deg" % w if w is not None else "unknown",
                         "AT THE PICK POSE" if a["at_pick_pose"]
                         else "*** NOT at the pick pose ***"))
        else:
            print("  not measured: %s" % p.get("why", "")[:160])

    if args.replay:
        dirs = sorted(glob.glob(os.path.join(args.replay, "raw", "*")))
        if not dirs:
            print("no raw frames under %s" % args.replay)
            return 2
        names = [os.path.basename(d) for d in dirs]
    else:
        names = (list(SOURCES) if args.sources == "auto"
                 else [s.strip() for s in args.sources.split(",") if s.strip()])

    for i, name in enumerate(names):
        print("\n[%d/%d] %s" % (i + 1, len(names), name))
        rec = dict(name=name)
        try:
            repairs = []
            if args.replay:
                fr = load_raw(os.path.join(args.replay, "raw", name))
            else:
                fr, repairs = acquire(name, args)
            if repairs:
                rec["repairs"] = repairs
        except Refusal as exc:
            rec["error"] = str(exc)
            print("  no frame: %s" % str(exc)[:400])
            img = refusal_figure("%s -- NO FRAME" % name, str(exc))
            cv2.imwrite(os.path.join(outdir, "sheet_%s.png" % name), img)
            run["sources"].append(rec)
            continue
        except Exception as exc:                                 # noqa: BLE001
            rec["error"] = "%s: %s" % (type(exc).__name__, exc)
            print("  no frame: %s" % rec["error"])
            run["sources"].append(rec)
            continue
        rec.update(provenance=fr.provenance, notes=fr.notes,
                   colour_shape=list(fr.colour.shape),
                   has_depth=fr.depth_m is not None,
                   K_colour=fr.K_c, K_depth=fr.K_d)
        rec["raw"] = os.path.relpath(save_raw(fr, outdir), outdir)
        stages, sheet = run_stages(fr, outdir, args.prompt, fast=args.fast,
                                   only=only)
        rec["stages"], rec["sheet"] = stages, sheet
        run["sources"].append(rec)

    if _STARTED and not args.keep_started:
        stop_started()
    elif _STARTED:
        print("\nleaving %d camera driver(s) running, as asked (--keep-started)"
              % len(_STARTED))
    run["finished"] = datetime.now().isoformat(timespec="seconds")
    with open(os.path.join(outdir, "manifest.json"), "w") as fh:
        json.dump(run, fh, indent=2, default=str)
    allrows = []
    for src in run["sources"]:
        for st in src.get("stages", []):
            for d in st.get("data", []):
                allrows.append([src["name"], st["num"], st["slug"],
                                d["quantity"], d["value"], d["unit"]])
    _csv(os.path.join(outdir, "measurements.csv"),
         ["source", "stage_num", "stage", "quantity", "value", "unit"], allrows)
    write_report(outdir, run)

    good = sum(1 for s in run["sources"] for st in s.get("stages", [])
               if st["status"] == "ok")
    total = sum(len(s.get("stages", [])) for s in run["sources"])
    err = [(s["name"], st["slug"]) for s in run["sources"]
           for st in s.get("stages", []) if st["status"] == "error"]
    print("\n%d of %d stages produced a result; %d crashed%s"
          % (good, total, len(err),
             (" (%s)" % ", ".join("%s/%s" % e for e in err)) if err else ""))
    for s in run["sources"]:
        if s.get("error"):
            print("  %-14s NO FRAME" % s["name"])
    print("\nwritten to %s" % outdir)
    print("  measurements.csv  %d measurements, every one with a unit"
          % len(allrows))
    print("  data/          per stage: the same numbers as JSON, plus a CSV "
          "per object table")
    print("  report.md      every stage with its formula and its numbers")
    print("  sheet_*.png    one page per camera")
    print("  raw/           the frames, so figures can be redrawn with "
          "--replay %s" % outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
