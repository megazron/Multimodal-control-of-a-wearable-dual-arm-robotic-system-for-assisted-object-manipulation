"""SEGMENT THE PICTURE, THEN LIFT EACH MASK THROUGH THE DEPTH.

This replaces the way this repository has always found objects on a surface:
fit a plane to the fused point cloud, then cluster the points above it by
Euclidean distance. That is the pre-2020 method and it has one well-known
failure, which is the one that stopped the calibration stage working.

WHAT WENT WRONG WITH CLUSTERING
-------------------------------
T1's cubes are 40 mm wide on a 60 mm pitch, so the GAP between two adjacent
faces is 20 mm. `table_scene.objects_on_plane` clusters at `cluster_m=0.020`.
Two cubes 20 mm apart are therefore ONE cluster, and the map reported a single
object **140 mm across the jaws** where two 40 mm cubes are, refused it as
wider than the gripper, and gave a confident reason. Tightening the cluster
distance moves the failure rather than removing it: below the depth noise the
same cube splits into several objects instead, and there is no setting that is
right for both a 20 mm gap and a noisy surface.

The two cubes are not remotely ambiguous in the PICTURE. They have different
colours, a visible edge between them, and a segmenter cuts them apart without
being told anything about the scene.

WHAT THE FIELD ACTUALLY DOES
----------------------------
Unseen-object instance segmentation: get object-agnostic mask proposals from
the image, then lift each mask through the depth to get that instance's own
points, and run grasp synthesis per instance. ZISVFM and the UOIS line of work
do exactly this with SAM-family models; CLASP's geometric-grounding pathway is
the same shape. The instance boundary comes from appearance, where it is easy,
and depth is used only to place the instance in metres, where it is reliable.

So: masks decide WHAT is an object, depth decides WHERE it is. Clustering asked
depth to decide both, and depth cannot separate a 20 mm gap.

WHAT THIS COSTS AND WHAT IT DOES NOT NEED
-----------------------------------------
FastSAM-s is ~11M parameters and the weights are already in this repository --
`prompt_detector` has used it since 2026-08-21, where on the real arms it cut
the scene into 89 regions and found the target cube every time. It runs on the
CPU, which is what this host has: torch here is 2.13.0+cpu and there is no GPU
at all. A calibration sweep is one frame per cell, a dozen cells, so a
segmenter at one or two frames a second is not the bottleneck -- the arm is.

THE FALLBACK IS NAMED, NEVER SILENT
-----------------------------------
If ultralytics or the weights are missing, `objects_in_view` RAISES. It does
not quietly cluster instead. A map built by clustering and a map built by
segmentation have different failure modes, and one that could be either is
evidence for nothing. `world_model` records which finder produced the objects.
"""
from __future__ import annotations

import math
import os

import numpy as np

# A mask smaller than this is noise; larger than this fraction of the frame is
# the table, the wall, or the robot's own arm.
MIN_AREA_PX = 60
MAX_AREA_FRAC = 0.35
# How far above the support plane a mask's points must sit, in the MEDIAN, to
# be a thing ON the surface rather than the surface itself. Below the depth
# noise and this admits the table; well above it and a flat mat is missed.
MIN_HEIGHT_M = 0.004
# A mask must keep this many lifted points to be measured at all. A mask with
# three depth returns has a centre with no meaning.
MIN_POINTS = 25
# The jaws. `rgbd_grasp` uses the same number and it is the gripper's, not a
# tuning value.
JAW_M = 0.085


class SegRefusal(Exception):
    """The segmenter could not run, or ran and found nothing usable."""


def _layers(h, bin_m=0.005, step_m=0.012, min_points=MIN_POINTS, depth=2):
    """Split a mask's heights into the SUPPORT it rests on and what stands on it.

    WHY THIS IS NEEDED AND SEGMENTATION ALONE IS NOT. In this scene a blue
    cube stands on a blue pad and a green cube on a green pad. They are the
    same colour, they touch, and there is no edge between them -- the
    segmenter correctly returns ONE region, because in the picture it IS one
    region. `mock_rgbd_camera` says so in its own source: the depth step is
    the only thing that separates a cube from the same-coloured pad it stands
    on. Appearance decides what is SIDE BY SIDE, which is what clustering
    could not do across a 20 mm gap; height decides what is STACKED, which
    appearance cannot do at all.

    AND IT IS NOT A GAP. My first version looked for empty height bins between
    layers, and the test written against it failed immediately -- correctly. A
    cube RESTS on the pad: the pad's top face and the cube's bottom face are
    at the SAME height, and the cube's own sides fill every bin between there
    and its top. There is no empty band to find. A splitter looking for one
    finds a single layer and hands back the blob it was given.

    What is actually there is a MODE. The support's top face is a large flat
    area, so its height is the most populated bin by a wide margin; whatever
    stands on it is a minority of the points spread above. So: find the modal
    height, cut `step_m` above it, and recurse on what is left -- table, then
    pad, then cube.
    """
    h = np.asarray(h, float)
    if len(h) < min_points:
        return []
    lo, hi = float(h.min()), float(h.max())
    if hi - lo < step_m or depth <= 0:
        return [(lo, hi + 1e-6)]
    nb = max(1, int(math.ceil((hi - lo) / bin_m)) + 1)
    counts = np.bincount(np.clip(((h - lo) / bin_m).astype(int), 0, nb - 1),
                         minlength=nb)
    mode_h = lo + float(np.argmax(counts)) * bin_m
    cut = mode_h + step_m
    below = h < cut
    n_above = int((~below).sum())
    if n_above < min_points or int(below.sum()) < min_points:
        return [(lo, hi + 1e-6)]
    out = [(lo, cut)]
    out.extend(_layers(h[~below], bin_m=bin_m, step_m=step_m,
                       min_points=min_points, depth=depth - 1))
    return out


class Seg:
    """One mask, lifted into metres."""

    def __init__(self, centre, extents, width_m, points, area_px, mean_bgr,
                 height_m, source=""):
        self.centre = [float(v) for v in centre]
        self.extents = [float(v) for v in extents]
        self.width_m = float(width_m)
        self.points = np.asarray(points, float).reshape(-1, 3)
        self.area_px = int(area_px)
        self.mean_bgr = [float(v) for v in mean_bgr]
        self.height_m = float(height_m)
        self.source = str(source)
        p = self.points
        self.lo = p.min(axis=0) if len(p) else np.zeros(3)
        self.hi = p.max(axis=0) if len(p) else np.zeros(3)

    @property
    def n_points(self):
        return int(len(self.points))

    @property
    def graspable(self):
        return self.width_m <= JAW_M

    @property
    def why(self):
        if self.graspable:
            return ""
        return ("%.0f mm across its short axis and the jaws close on %.0f mm"
                % (self.width_m * 1000, JAW_M * 1000))

    def __repr__(self):
        return ("Seg(%s, %.0f mm, %d px, %d pts)"
                % (["%.3f" % v for v in self.centre], self.width_m * 1000,
                   self.area_px, self.n_points))


def _weights_path(name="FastSAM-s.pt"):
    """Where the weights are, or a refusal naming every place looked.

    `ultralytics` will DOWNLOAD a missing weight file, which on a lab machine
    with no route to the internet is a hang rather than an error, and on this
    one would silently fetch a different revision than the measurements were
    made with.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    tried = [os.path.join(root, name), os.path.join(os.getcwd(), name),
             os.path.expanduser("~/kortex_ws/" + name)]
    for p in tried:
        if os.path.exists(p):
            return p
    raise SegRefusal("no %s in any of: %s" % (name, ", ".join(tried)))


_MODEL = {}


def masks(bgr, weights="FastSAM-s.pt", imgsz=640, conf=0.25, iou=0.7,
          device="cpu", min_area=MIN_AREA_PX, max_area_frac=MAX_AREA_FRAC):
    """EVERY region in the picture, as boolean masks. No prompt, no class.

    Deliberately unconditional: the calibration stage asks "what is on the
    table", not "where is the green cube". A finder that needs to be told what
    to look for cannot answer the first question, and answering the first
    question is the whole point of mapping before planning.
    """
    import cv2
    try:
        from ultralytics import FastSAM
    except ImportError as e:
        raise SegRefusal(
            "ultralytics is not importable (%s), so the segmenter cannot run. "
            "This is reported rather than falling back to clustering: a map "
            "built by clustering has different failure modes and must not "
            "look like one built by segmentation." % e)
    path = _weights_path(weights)
    if path not in _MODEL:
        _MODEL[path] = FastSAM(path)
    res = _MODEL[path].predict(bgr, device=device, retina_masks=True,
                               imgsz=imgsz, conf=conf, iou=iou, verbose=False)
    r = res[0]
    if r.masks is None:
        return []
    H, W = bgr.shape[:2]
    out = []
    for mk in r.masks.data.cpu().numpy():
        m = mk.astype(bool)
        if m.shape != (H, W):
            m = cv2.resize(mk.astype(np.uint8), (W, H),
                           interpolation=cv2.INTER_NEAREST) > 0
        a = int(m.sum())
        if a < min_area or a > max_area_frac * H * W:
            continue
        out.append(m)
    return out


def _min_width(flat2, step_deg=0.25):
    """The narrowest direction across a 2-D outline, and the span across it.

    THIS IS NOT PCA, AND THE SELF-TEST CAUGHT ME USING PCA. The principal axes
    of a SQUARE are degenerate -- both horizontal directions have identical
    variance -- so SVD returns the diagonal, and a 40 mm cube was measured at
    **53 mm**, which is 40*sqrt(2). Every cube in the scene would have been
    reported half again as wide as it is, and against an 85 mm jaw a 60 mm
    object would have been refused as ungraspable.

    The quantity a parallel jaw actually cares about is the MINIMUM WIDTH: the
    smallest distance between two parallel lines enclosing the outline. That is
    rotating calipers, and it is what the jaws do physically -- they close
    across the narrowest way the object presents itself.

    Swept rather than hull-based so there is no scipy dependency in the middle
    of the perception path. At 0.25 deg the worst error on a square is
    40*(cos+sin-1) = 0.17 mm, which is under the depth noise it is measuring.
    """
    th = np.radians(np.arange(0.0, 180.0, step_deg))
    dirs = np.stack([np.cos(th), np.sin(th)], axis=1)          # (A, 2)
    proj = flat2 @ dirs.T                                      # (N, A)
    span = proj.max(axis=0) - proj.min(axis=0)
    k = int(np.argmin(span))
    return float(span[k]), dirs[k], float(th[k])


def _measure(pts, up=(0.0, 0.0, 1.0)):
    """(centre, extents, width) for one instance's points.

    `width` is the MINIMUM width in the horizontal plane -- the narrowest way
    the object presents itself to a parallel jaw. `extents` are
    [long, short, height], the long axis being measured perpendicular to the
    narrow one.
    """
    up = np.asarray(up, float)
    up = up / np.linalg.norm(up)
    # An orthonormal horizontal basis. Any one will do; the min-width search
    # is rotation-invariant, which is the point of using it.
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(tmp @ up)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    e1 = tmp - up * float(tmp @ up)
    e1 = e1 / np.linalg.norm(e1)
    e2 = np.cross(up, e1)

    c = pts.mean(axis=0)
    rel = pts - c
    flat2 = np.stack([rel @ e1, rel @ e2], axis=1)
    short_m, ndir, _ = _min_width(flat2)
    ldir = np.array([-ndir[1], ndir[0]])
    long_m = float(np.ptp(flat2 @ ldir))
    height_m = float(np.ptp(rel @ up))

    # THE CENTRE IS THE MIDDLE OF THE EXTENT, NOT THE MEAN OF THE POINTS.
    #
    # A wrist camera sees the near face and the top of a cube and NOT the far
    # face, so the returns are lopsided and their mean sits toward the camera.
    # For a 40 mm cube that is up to 10 mm, which is a third of the 30 mm
    # capture gate spent before the arm has moved. The mid-extent is unbiased
    # in the two axes whose full width was seen.
    a_n = flat2 @ ndir
    a_l = flat2 @ ldir
    mid2 = (ndir * float((a_n.max() + a_n.min()) / 2.0)
            + ldir * float((a_l.max() + a_l.min()) / 2.0))
    mid = c + e1 * float(mid2[0]) + e2 * float(mid2[1])
    return mid, [long_m, short_m, height_m], short_m


def _suppress_nested(segs, voxel_m=0.005, frac=0.55):
    """Drop masks that are just unions of finer ones. PREFER THE FINE CUT.

    FastSAM returns a HIERARCHY, not a partition: for the same picture it
    hands back the cube, the pad, and a single region covering the pad AND the
    cube together. All three are legitimate segmentations. Lifted, they became
    three objects occupying the same space, and the fusion -- which merges
    things that overlap -- absorbed the 40 mm cube into the 148 mm blob and
    reported one ungraspable object where two graspable ones are.

    MEASURED, one view at the corrected viewing angle: the cube came back at
    (0.480, 0.450) with a width of 40.3 mm against a truth of (0.480, 0.450)
    and 40 mm -- exact. It was then thrown away by the merge.

    The rule is the one instance segmentation always uses: accept the FINEST
    masks first, and reject a coarser one whose volume is already explained.
    Sorted small-first, a mask is kept only if more than `1 - frac` of its
    voxels are still unclaimed. So the cube is accepted, its duplicate is
    rejected (fully claimed), the pad is accepted (its own volume is free),
    and the pad-plus-cube blob is rejected (claimed twice over).

    Volume rather than pixels, because two masks that overlap in the image may
    be a metre apart in depth.
    """
    order = sorted(segs, key=lambda s: len(s.points))
    claimed = set()
    kept = []
    for s in order:
        keys = {(int(math.floor(x / voxel_m)), int(math.floor(y / voxel_m)),
                 int(math.floor(z / voxel_m))) for x, y, z in s.points}
        if not keys:
            continue
        overlap = len(keys & claimed) / float(len(keys))
        if overlap > frac:
            continue
        claimed |= keys
        kept.append(s)
    return kept


def objects_in_view(bgr, depth_m, K, cam_pose, plane, stride=1, source="",
                    up=(0.0, 0.0, 1.0), min_height_m=MIN_HEIGHT_M,
                    min_points=MIN_POINTS, **kw):
    """One RGB-D frame -> the instances standing on `plane`, in the robot frame.

    `plane` is a `table_scene.Plane` (anything with `.height(points)`), fitted
    to the SAME cloud or to the fused map. The plane is used only to decide
    which masks are things ON the surface -- the instance boundaries come from
    the picture, which is the whole point.
    """
    from pick_from_table import cloud_in_robot_frame
    ms = masks(bgr, **kw)
    if not ms:
        raise SegRefusal("the segmenter returned no regions for this frame")
    pts, uv = cloud_in_robot_frame(depth_m, K, cam_pose, stride=stride,
                                   with_pixels=True)
    H, W = bgr.shape[:2]
    # Pixel -> index into the lifted cloud. One pass, so a frame with a dozen
    # masks does not walk the whole cloud a dozen times.
    idx = np.full((H, W), -1, np.int64)
    idx[uv[:, 1], uv[:, 0]] = np.arange(len(pts))

    import cv2
    out = []
    for m in ms:
        # ERODE BY ONE PIXEL BEFORE LIFTING. A mask boundary sits on the
        # depth discontinuity, where a pixel's depth is a blend of the object
        # and whatever is behind it -- the classic flying pixel. Those points
        # land in mid-air between the two surfaces and they are exactly the
        # ones that stretch a measured width. One pixel at 1.3 m on this
        # intrinsic is 2.1 mm, so the erosion costs about that and removes a
        # tail that was running to tens of millimetres.
        me = cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8),
                       iterations=1).astype(bool)
        if me.sum() < min_points:
            me = m
        sel_img = idx * 0 - 1
        sel_img[me] = idx[me]
        sel = sel_img[me]
        sel = sel[sel >= 0]
        if len(sel) < min_points:
            continue
        # DROP THE SURFACE POINTS FIRST, BEFORE ANY LAYERING.
        #
        # A mask does not stop at the object: it carries a ring of the table
        # around it. Those points sit at h = 0, so the MODE of the height
        # histogram is the table -- and the cut at mode + 12 mm then lands
        # 12 mm up the SIDE OF THE CUBE, slicing its bottom band off as a
        # separate "object". The live map had five of them: 7 mm, 8 mm, 21 mm
        # and 28 mm pieces at the feet of real cubes.
        #
        # With the surface removed the mode is the object's own, and the only
        # thing the layer split still has to do is the job it was written for:
        # separating a cube from the pad it stands on.
        h_raw = plane.height(pts[sel])
        above = h_raw >= min_height_m
        if int(above.sum()) < min_points:
            continue
        sel = sel[above]
        h_all = h_raw[above]
        # SPLIT THE MASK INTO ITS HEIGHT LAYERS, then split each layer into
        # its connected pieces IN THE IMAGE. A blue pad carrying two blue
        # cubes is one region, two layers, and two pieces in the upper layer.
        for (h0, h1) in _layers(h_all) or [(float(h_all.min()),
                                            float(h_all.max()) + 1e-6)]:
            keep = (h_all >= h0) & (h_all < h1)
            if int(keep.sum()) < min_points:
                continue
            if float(np.median(h_all[keep])) < min_height_m:
                continue                      # this layer IS the surface
            layer_img = np.zeros((H, W), np.uint8)
            uvk = uv[sel[keep]]
            layer_img[uvk[:, 1], uvk[:, 0]] = 1
            # Close one pixel of sampling stipple so a solid face is one
            # component, without bridging a real 20 mm gap (which is ~10 px
            # here).
            layer_img = cv2.morphologyEx(layer_img, cv2.MORPH_CLOSE,
                                         np.ones((3, 3), np.uint8))
            ncc, lab = cv2.connectedComponents(layer_img, connectivity=8)
            for ci in range(1, ncc):
                pix = lab[uvk[:, 1], uvk[:, 0]] == ci
                if int(pix.sum()) < min_points:
                    continue
                p = pts[sel[keep][pix]]
                # THE HEIGHT TEST AGAIN, PER COMPONENT. It was applied to the
                # whole LAYER, so a layer whose median sits above the surface
                # could still contain a component lying flat on it -- and the
                # 3 mm, 0.2 mm-high white slivers in the object list were
                # exactly that: bare table, promoted by the company it kept.
                hp = plane.height(p)
                if float(np.median(hp)) < min_height_m:
                    continue
                centre, extents, width = _measure(p, up=up)
                cm = np.zeros((H, W), np.uint8)
                cm[uvk[pix][:, 1], uvk[pix][:, 0]] = 1
                b, g, r = [float(v) for v in cv2.mean(bgr, cm)[:3]]
                out.append(Seg(centre, extents, width, p, int(pix.sum()),
                               [b, g, r],
                               float(np.median(plane.height(p))),
                               source=source))
    out = _suppress_nested(out)
    out.sort(key=lambda s: (-s.n_points))
    return out


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):
    """No camera, no model: the parts that decide the NUMBERS.

    `masks` needs FastSAM and is exercised live. `_measure` is arithmetic on
    constructed geometry, which is where the errors that matter live -- the
    mid-extent centre exists because the mean of a one-sided cloud is biased,
    and that is checkable exactly.
    """
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-58s %s%s" % (name, "ok" if cond else "FAIL",
                                    "" if cond else "   " + str(detail)))

    rng = np.random.default_rng(0)
    # A 40 mm cube, fully seen, centred at a known place.
    c = np.array([0.42, 0.45, 1.27])
    full = c + rng.uniform(-0.02, 0.02, (2000, 3))
    mid, ext, w = _measure(full)
    check("a fully seen 40 mm cube reads 40 mm across the short axis",
          abs(w - 0.04) < 0.004, w)
    check("and its centre is within 2 mm", float(np.linalg.norm(mid - c)) < 0.002,
          mid)

    # THE ONE-SIDED CASE, which is what a wrist camera actually returns: only
    # the -y half of the cube plus the top.
    half = full[(full[:, 1] < c[1]) | (full[:, 2] > c[2] + 0.015)]
    mid2, _, w2 = _measure(half)
    mean_err = float(abs(half.mean(axis=0)[1] - c[1]))
    mid_err = float(abs(mid2[1] - c[1]))
    check("the MEAN of a one-sided cloud is biased by more than 5 mm",
          mean_err > 0.005, mean_err)
    check("and the mid-extent centre is not (under 2 mm)", mid_err < 0.002,
          mid_err)
    check("the one-sided cube still measures 40 mm wide",
          abs(w2 - 0.04) < 0.005, w2)

    # A cube turned 45 degrees must not read wider. This is what the axis-
    # aligned bounding box gets wrong.
    t = np.pi / 4
    R = np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0],
                  [0, 0, 1.0]])
    turned = (full - c) @ R.T + c
    _, _, w3 = _measure(turned)
    aabb = float(np.ptp(turned[:, :2], axis=0).min())
    check("a cube at 45 deg still reads 40 mm on the principal axis",
          abs(w3 - 0.04) < 0.005, w3)
    check("THE CONTROL: the axis-aligned box gets it wrong by over 10 mm",
          abs(aabb - 0.04) > 0.010, aabb)

    # THE CASE THAT BROKE CLUSTERING. Two cubes 60 mm apart are two separate
    # inputs here because the masks came from the picture, and each measures
    # 40 mm. Measured as one cluster -- which is what the old finder did -- the
    # pair is 100 mm LONG, and that is the number that says two objects were
    # fused. Deliberately NOT the width: a 100 x 40 pair still presents 40 mm
    # to the jaws, so a width test would have called the fused blob graspable
    # and reported nothing wrong. My first version of this control asserted
    # the width and was simply false.
    c2 = c + np.array([0.06, 0, 0])
    other = c2 + rng.uniform(-0.02, 0.02, (2000, 3))
    a_mid, a_ext, a_w = _measure(full)
    b_mid, b_ext, b_w = _measure(other)
    check("two cubes measured per-mask read 40 mm wide each",
          abs(a_w - 0.04) < 0.004 and abs(b_w - 0.04) < 0.004, (a_w, b_w))
    check("and 40 mm LONG each -- one object, not two fused",
          abs(a_ext[0] - 0.04) < 0.004 and abs(b_ext[0] - 0.04) < 0.004,
          (a_ext[0], b_ext[0]))
    check("their centres are 60 mm apart, as placed",
          abs(float(np.linalg.norm(np.array(a_mid) - np.array(b_mid))) - 0.06)
          < 0.003, float(np.linalg.norm(np.array(a_mid) - np.array(b_mid))))
    both_ext = _measure(np.vstack([full, other]))[1]
    check("THE CONTROL: fused into ONE cluster they read 100 mm long",
          both_ext[0] > 0.090, both_ext[0])
    check("and the fused blob is still 40 mm WIDE, so a width test would "
          "have missed it", abs(both_ext[1] - 0.04) < 0.006, both_ext[1])

    check("a width over the jaw span is refused and says so",
          not Seg([0, 0, 0], [0.2, 0.14, 0.04], 0.14, np.zeros((1, 3)), 1,
                  [0, 0, 0], 0.04).graspable, "")
    if verbose:
        print("segment_lift self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
