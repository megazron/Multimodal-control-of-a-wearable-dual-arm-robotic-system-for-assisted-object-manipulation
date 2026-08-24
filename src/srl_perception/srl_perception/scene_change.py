"""WHAT MOVED ON THE TABLE, FROM ONE FIXED CAMERA, WITHOUT MOVING THE ARM.

WHY THIS IS THE SCENE CAMERA'S JOB AND NOT THE WRIST CAMERAS'
-------------------------------------------------------------
Re-measuring the table with the wrist cameras takes minutes: the arm has to
drive to every viewpoint. Almost all of that re-measures surface that has not
changed, because what changes on a table is the OBJECTS, and they occupy a
small fraction of it.

A fixed camera watching the table answers "did anything change, and roughly
where" **instantly and with no arm motion at all**. That is exactly the
question that turns a full sweep into a visit to three viewpoints.

AND IT CANNOT DO THE OTHER HALF. `scene_camera_node` publishes
`/scene_camera/image_raw` and `/scene_camera/camera_info` and NO DEPTH -- it
is a monocular colour camera. One colour image gives a DIRECTION to a thing
and never a DISTANCE, so it cannot place a point in metres and cannot measure
an object. This module therefore never tries. It returns REGIONS OF THE WORK
SURFACE to go and look at properly, and the wrist cameras do the measuring.

The one geometric thing a monocular camera CAN do is intersect its own rays
with a plane it has been told about. That is what turns a changed patch of
image into a patch of table, and it is exact given the camera pose and the
plane -- both of which this pipeline already measures.

WHAT MAKES A CHANGE A CHANGE
----------------------------
Absolute difference, blurred, thresholded, opened. Deliberately dull: the
question is "is something different here", not "what is it", and a detector
with opinions would need its own validation. The thresholds are named and the
refusals say which one bit.

A CAMERA THAT HAS NOT BEEN CALIBRATED SAYS SO
---------------------------------------------
`scene_camera_node` publishes a ZERO intrinsic matrix until it is calibrated.
Deprojecting through fx = 0 is a division by zero that numpy will happily turn
into infinities, and the regions would come back somewhere near the horizon.
`REFUSES` on it by name.
"""
from __future__ import annotations

import math

import numpy as np

# A pixel must differ by this much (0-255, summed over channels / 3) to count.
DIFF_THRESHOLD = 28
# Blur radius before differencing, in pixels. Kills sensor noise and one-pixel
# registration jitter without moving a real edge.
BLUR_PX = 5
# A changed blob smaller than this is noise, not an object.
MIN_BLOB_PX = 120
# How far outside a changed blob's footprint to look, in metres. An object
# that moved reveals the table it came from AND covers the table it went to,
# and the region worth re-measuring is a little larger than either.
REGION_PAD_M = 0.06


class ChangeRefusal(Exception):
    """Named, with the reason. Never an empty answer standing in for one."""


def _valid_K(K):
    K = [float(v) for v in K]
    if len(K) < 4:
        raise ChangeRefusal("camera_info carries %d intrinsics, expected "
                            "fx, fy, cx, cy" % len(K))
    if K[0] <= 1e-6 or K[1] <= 1e-6:
        raise ChangeRefusal(
            "the camera's intrinsic matrix is ZERO (fx=%.3f, fy=%.3f). "
            "`scene_camera_node` publishes that until it is calibrated, and "
            "deprojecting through it would put every region near the horizon "
            "rather than on the table." % (K[0], K[1]))
    return K


def changed_mask(ref_bgr, cur_bgr, threshold=DIFF_THRESHOLD, blur_px=BLUR_PX,
                 min_blob_px=MIN_BLOB_PX):
    """(mask, blobs) -- where the picture differs, as boolean image + labels."""
    import cv2
    a = np.asarray(ref_bgr)
    b = np.asarray(cur_bgr)
    if a.shape != b.shape:
        raise ChangeRefusal(
            "the reference is %s and the current frame is %s. A difference "
            "between two different pictures is not a change in the scene."
            % (a.shape, b.shape))
    k = int(blur_px) | 1
    fa = cv2.GaussianBlur(a.astype(np.uint8), (k, k), 0).astype(np.int16)
    fb = cv2.GaussianBlur(b.astype(np.uint8), (k, k), 0).astype(np.int16)
    d = np.abs(fa - fb).mean(axis=2)
    m = (d >= float(threshold)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(m, connectivity=8)
    keep = np.zeros_like(m, bool)
    blobs = []
    for i in range(1, n):
        sel = lab == i
        area = int(sel.sum())
        if area < min_blob_px:
            continue
        keep |= sel
        ys, xs = np.nonzero(sel)
        blobs.append(dict(area_px=area,
                          bbox=[int(xs.min()), int(ys.min()),
                                int(xs.max()), int(ys.max())],
                          pixels=np.stack([xs, ys], axis=1)))
    return keep, blobs


def rays_to_plane(uv, K, cam_pose, plane):
    """Where each pixel's ray meets the support plane, in the robot frame.

    The one piece of geometry a monocular camera really can do. `cam_pose` is
    the 4x4 of the camera's OPTICAL frame in `world`; the optical convention
    is z forward, x right, y down, which is what the rest of this package
    deprojects with.
    """
    fx, fy, cx, cy = _valid_K(K)
    uv = np.asarray(uv, float).reshape(-1, 2)
    d_cam = np.stack([(uv[:, 0] - cx) / fx,
                      (uv[:, 1] - cy) / fy,
                      np.ones(len(uv))], axis=1)
    M = np.asarray(cam_pose, float)
    R, o = M[:3, :3], M[:3, 3]
    d = d_cam @ R.T
    n = np.asarray(plane.normal, float)
    denom = d @ n
    t = (float(plane.offset) - float(o @ n)) / np.where(
        np.abs(denom) < 1e-9, np.nan, denom)
    # A ray that runs away from the plane, or parallel to it, meets nothing in
    # front of the camera. Dropped rather than wrapped around behind it.
    good = np.isfinite(t) & (t > 0)
    return o + d[good] * t[good][:, None], good


def changed_regions(ref_bgr, cur_bgr, K, cam_pose, plane, pad_m=REGION_PAD_M,
                    **kw):
    """Patches of the WORK SURFACE worth re-measuring, in the robot frame.

    Returns a list of dicts with the region's centre, its (x, y) bounds padded
    by `pad_m`, and the pixel area that produced it -- never an object, never
    a height. What is standing there is for a camera with depth to say.
    """
    _valid_K(K)
    _mask, blobs = changed_mask(ref_bgr, cur_bgr, **kw)
    out = []
    for b in blobs:
        pts, good = rays_to_plane(b["pixels"], K, cam_pose, plane)
        if len(pts) < 10:
            continue
        lo = pts[:, :2].min(axis=0) - pad_m
        hi = pts[:, :2].max(axis=0) + pad_m
        out.append(dict(
            centre=[float(v) for v in pts[:, :2].mean(axis=0)],
            x=[float(lo[0]), float(hi[0])], y=[float(lo[1]), float(hi[1])],
            area_px=b["area_px"], bbox_px=b["bbox"],
            n_rays=int(good.sum())))
    out.sort(key=lambda r: -r["area_px"])
    return out


def covers(region, cells, cell_m=0.05):
    """Do a view's footprint cells overlap this region? Which ones."""
    hit = set()
    for (i, j) in cells:
        x, y = (i + 0.5) * cell_m, (j + 0.5) * cell_m
        if region["x"][0] <= x <= region["x"][1] and \
           region["y"][0] <= y <= region["y"][1]:
            hit.add((i, j))
    return hit


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):
    """Constructed images and constructed geometry: the ground truth is
    arithmetic, which is what the standing rule allows synthetic data for."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-62s %s%s" % (name, "PASS" if cond else "FAIL",
                                    "" if cond else "  -- " + str(detail)))

    def refuses(name, fn, needle=""):
        nonlocal ok
        try:
            fn()
        except ChangeRefusal as e:
            hit = needle in str(e)
            ok = ok and hit
            if verbose:
                print("  %-62s %s" % (name, "PASS" if hit else "FAIL -- %s" % e))
            return
        ok = False
        if verbose:
            print("  %-62s FAIL -- it did not refuse" % name)

    from srl_perception import table_scene as TS

    H, W = 480, 640
    K = [600.0, 600.0, W / 2 - 0.5, H / 2 - 0.5]
    # A camera 1.2 m above a plane at z = 0, looking straight down.
    plane = TS.Plane(np.array([0.0, 0.0, 1.0]), 0.0, 1000, 1000)
    M = np.eye(4)
    M[:3, :3] = np.array([[1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0]])   # optical z -> world -z
    M[:3, 3] = [0.0, 0.0, 1.2]

    ref = np.full((H, W, 3), 200, np.uint8)
    cur = ref.copy()
    # A dark 40 px square appears, centred 100 px right and 60 px down.
    u0, v0 = W // 2 + 100, H // 2 + 60
    cur[v0 - 20:v0 + 20, u0 - 20:u0 + 20] = 30

    regs = changed_regions(ref, cur, K, M, plane, blur_px=3)
    check("one appearance gives one region", len(regs) == 1, regs)
    if regs:
        # Ground truth by similar triangles: 1.2 m up, 600 px focal.
        tx = (u0 - K[2]) / K[0] * 1.2
        ty = -(v0 - K[3]) / K[1] * 1.2
        c = regs[0]["centre"]
        check("the region lands where the square is, to 5 mm",
              abs(c[0] - tx) < 0.005 and abs(c[1] - ty) < 0.005,
              (c, [tx, ty]))
        check("the region is padded, so it is bigger than the blob itself",
              (regs[0]["x"][1] - regs[0]["x"][0]) > 2 * REGION_PAD_M)

    check("an unchanged pair reports NOTHING",
          changed_regions(ref, ref.copy(), K, M, plane, blur_px=3) == [])
    # THE CONTROL for the threshold: a detector that fired on anything would
    # pass the test above only by luck, and one that never fired would pass
    # this one always.
    noisy = ref.copy().astype(np.int16) + np.random.default_rng(0).integers(
        -6, 7, ref.shape)
    check("sensor-scale noise is not a change",
          changed_regions(ref, np.clip(noisy, 0, 255).astype(np.uint8),
                          K, M, plane, blur_px=3) == [])

    two = cur.copy()
    two[80:140, 60:120] = 10
    check("two changes give two regions",
          len(changed_regions(ref, two, K, M, plane, blur_px=3)) == 2)

    refuses("a ZERO intrinsic matrix is refused BY NAME",
            lambda: changed_regions(ref, cur, [0.0, 0.0, 0.0, 0.0], M, plane),
            "ZERO")
    refuses("two differently sized pictures are refused",
            lambda: changed_regions(ref, cur[:100], K, M, plane),
            "not a change in the scene")

    # A ray pointing away from the plane meets nothing in FRONT of the camera.
    up = np.eye(4)
    up[:3, 3] = [0.0, 0.0, 1.2]                       # optical z -> world +z
    pts, good = rays_to_plane([[K[2], K[3]]], K, up, plane)
    check("a camera looking AWAY from the plane returns no intersection",
          len(pts) == 0, pts)

    if verbose:
        print("scene_change self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
