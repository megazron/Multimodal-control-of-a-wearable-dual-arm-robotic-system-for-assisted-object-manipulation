#!/usr/bin/env python3
"""DETECTION THAT DRIVES THE GRASP. The one source, used by task and harness.

    from vision_grasp import observe_and_detect
    cubes = observe_and_detect("left")     # [(x, y, pad_index), ...]

WHY THIS MODULE EXISTS. `run_abc` built T1's path straight from `T1_CUBES` and
`T1_PAIR` -- declared coordinates and a declared colour -- so every recorded
clip of a COLOUR-MATCHED task was produced without a camera being consulted.
The detector, the depth fit and `grasp_generator` all existed; nothing joined
them to the recorded path. This is the join.

IT IS ALSO THE ONLY COPY. `scripts/verify_colour_vision.py` imports `classify`
and `deproject` from here rather than carrying its own, because a harness that
verifies a different implementation from the one that runs verifies nothing.

WHAT DETECTION HAS TO SURVIVE HERE, all three measured on 2026-08-16:

  * THE PADS ARE THE CUBES' OWN COLOURS. They are 210 x 130 mm against a 40 mm
    cube, and colour segmentation alone locks onto them -- 0 of 4. Rejected by
    a SIZE-AT-RANGE test: a blob must be the size that object would be at the
    range it appears to be at.
  * THE CUBES TOUCH THE PADS IN PROJECTION, so 8-connectivity merges them into
    one blob. Only a DEPTH STEP separates them; colour cannot. A single depth
    threshold recovers one cube of two (the pad is tilted and spans ~40 mm of
    depth across its own width) and a morphological closing is worse, because
    the cubes protrude past the pad's EDGE rather than sitting inside it.
  * THE GRASP MUST BE PLANNED AFTER RETURNING HOME. `/compute_ik` seeds from
    the live joint state; planned from the observe pose the same 24 waypoints
    solve with 0 failures at 0.0017 m of wearer clearance, and from home at
    0.1610 m. The unsafe branch is the one the obvious sequencing produces.
"""
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = HERE
for _ in range(6):
    if os.path.isdir(os.path.join(_ROOT, "recordings")):
        break
    _ROOT = os.path.dirname(_ROOT)
BASE = os.path.join(_ROOT, "recordings", "baselines")
for _p in (os.path.join(_ROOT, "config"),
           os.path.join(_ROOT, "src", "srl_perception")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def bands():
    from srl_perception.colour_shape_detector import DEFAULT_COLOURS
    return [(n, np.array(lo), np.array(hi)) for n, lo, hi in DEFAULT_COLOURS
            if n in ("blue", "green")]


def classify(img, depth, info, min_px=25, object_size_m=0.040,
             pad_sizes=((0.210, 0.130), (0.100, 0.160))):
    """Blobs of cube colour, with their pixel centroid and median depth.

    Deliberately simple and deliberately the DETECTOR'S OWN thresholds: the
    question here is whether the pipeline can carry a colour decision from
    pixels to a grasp, not whether a better segmenter exists.
    """
    import cv2
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    out, rejected = [], []
    fx = info.k[0]

    def components(mask):
        n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
        return [(lab == i, stats[i], cent[i]) for i in range(1, n)
                if stats[i, cv2.CC_STAT_AREA] >= min_px]

    for name, lo, hi in bands():
        mask = cv2.inRange(hsv, lo, hi)
        cands = []
        for sel, st, ct in components(mask):
            dz = depth[sel]
            dz = dz[(dz > 0.05) & (dz < 5.0)]
            if dz.size == 0:
                continue
            # DEPTH SPLIT -- COLOUR ALONE CANNOT SEPARATE A CUBE FROM THE PAD
            # IT SITS ON.
            #
            # The pads are the cubes' own colours by design (that is the task),
            # and from the observe pose a cube projects onto the pad behind it,
            # so 8-connectivity merges them into one component. Measured: three
            # components for four cubes and two pads, and the size gate then
            # correctly threw away the merged blobs -- 0 of 4.
            #
            # The cubes stand ~36 mm proud of the pad surface, which is a real
            # depth step. Where a component spans more than 20 mm in depth it
            # holds more than one object, and the NEARER part is the cube.
            p25, p50, p75 = np.percentile(dz, [25, 50, 75])
            if (p75 - p25) > 0.020:
                # CUT THE MASK AT THE DEPTH STEP.
                #
                # Two earlier attempts failed and both are worth recording.
                # A single depth threshold over the whole component recovered
                # one of the two blue cubes and lost the other, because the pad
                # is tilted in view and spans ~40 mm of depth across its own
                # width -- the same order as the step being looked for. A
                # morphological CLOSING to estimate the pad surface was worse
                # still (1 of 4): the cubes protrude past the pad's EDGE rather
                # than sitting inside its silhouette, so closing never fills
                # them and the residual stays flat.
                #
                # What actually separates them is the discontinuity itself. A
                # cube stands ~36 mm proud of the pad, so at the boundary
                # between the two the depth image has a step. Break the mask
                # wherever the local depth range exceeds that step and the
                # merged blob falls apart into pad and cubes, wherever on the
                # pad they happen to sit.
                d32 = np.where((depth > 0.05) & (depth < 5.0),
                               depth, 0.0).astype(np.float32)
                k3 = np.ones((3, 3), np.uint8)
                grad = (cv2.dilate(d32, k3) - cv2.erode(d32, k3))
                cut = (sel & (grad < 0.012)).astype(np.uint8)
                found = [c for c in components(cut)]
                if found:
                    cands += found
                else:
                    cands.append((sel, st, ct))
            else:
                cands.append((sel, st, ct))

        for sel, st, ct in cands:
            u, v = float(ct[0]), float(ct[1])
            d = depth[sel]
            d = d[(d > 0.05) & (d < 5.0)]
            if d.size == 0:
                continue
            zmed = float(np.median(d))
            blob_px = float(max(st[cv2.CC_STAT_WIDTH], st[cv2.CC_STAT_HEIGHT]))
            from srl_perception.colour_shape_detector import (
                size_at_range_ok, expected_px)
            if not size_at_range_ok(blob_px, fx, object_size_m, zmed):
                rejected.append(dict(colour=name, u=round(u, 1),
                                     v=round(v, 1),
                                     blob_px=round(blob_px, 1),
                                     depth_m=round(zmed, 4),
                                     expected_px=round(
                                         expected_px(fx, object_size_m,
                                                     zmed), 1),
                                     why="not cube-sized at its own range"))
                continue
            px = hsv[sel]
            lo_m = (px.astype(int) - lo.astype(int)).min(axis=0)
            hi_m = (hi.astype(int) - px.astype(int)).min(axis=0)
            out.append(dict(colour=name, u=u, v=v,
                            area=int(st[cv2.CC_STAT_AREA]),
                            depth_m=zmed,
                            hsv_median=[int(t) for t in np.median(px, axis=0)],
                            hsv_margin_low=[int(t) for t in lo_m],
                            hsv_margin_high=[int(t) for t in hi_m],
                            blob_px=round(blob_px, 1),
                            expected_px=round(
                                expected_px(fx, object_size_m, zmed), 1),
                            hsv_margin_min=[int(min(a_, b_)) for a_, b_
                                            in zip(lo_m, hi_m)]))
    return out, rejected


def deproject(det, info, p_cam, R_wc, object_size_m=0.040):
    """Pixel + depth -> world, through the real intrinsics and TF.

    THE DEPTH IS THE VISIBLE SURFACE, NOT THE CENTRE, and for a 40 mm cube
    that difference is half the object. Measured before this correction: 60 mm
    of localisation error against a 60 mm cube pitch, so every detection
    matched its NEIGHBOUR -- which carries the other colour, and the result
    read as a colour failure when it was a geometry one.

    The centre lies half an object further along the viewing ray than the face
    the camera sees, so that is added back. It is exact for a face viewed
    square on and slightly over-corrects on an oblique face; either way it is a
    great deal closer than not correcting at all.
    """
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    z = det["depth_m"]
    c = np.array([(det["u"] - cx) / fx * z, (det["v"] - cy) / fy * z, z])
    ray = c / max(1e-9, float(np.linalg.norm(c)))
    c = c + ray * (object_size_m / 2.0)
    return p_cam + R_wc @ c

# ------------------------------------------------------------------ the join
class DetectionUnavailable(RuntimeError):
    """Raised LOUDLY. A blind pick that silently falls back to the declared
    coordinate is indistinguishable from a working perception path, which is
    the failure mode TASK_SPEC P-4 exists to forbid."""


def observe_and_detect(arm, node=None, settle_s=6.0, expect=None,
                       return_home=True):
    """Move to the observe pose, take one frame, come back, report cubes.

    Returns (cubes, info) where cubes is [(x, y, pad_index)] ordered by x and
    pad_index is chosen by the DETECTED colour through PLANE_COLOURS.
    """
    import rclpy
    sys.path.insert(0, os.path.join(_ROOT, "scripts"))
    from verify_colour_vision import Vision
    import msc_clip_tasks as M
    import home_positions as hp

    obs_f = os.path.join(BASE, "observe_pose_%s.json" % arm)
    via_f = os.path.join(BASE, "observe_transit_%s.json" % arm)
    for f in (obs_f, via_f):
        if not os.path.exists(f):
            raise DetectionUnavailable(
                "%s is missing -- run scripts/solve_observe_pose.py and "
                "scripts/solve_observe_transit.py" % f)
    q_obs = np.array(json.load(open(obs_f))["solved"]["q"], float)
    vd = json.load(open(via_f))
    if not vd.get("via"):
        raise DetectionUnavailable(
            "no via recorded for the %s arm; the straight move from home "
            "breaches the wearer floor" % arm)
    q_via = np.array(vd["via"], float)
    q_home = np.array(hp.load_home_radians(arm), float)

    own = node is None
    n = Vision(arm) if own else node
    t = {}
    try:
        n.spin(4.0)
        t0 = time.time()
        ok1, _e1 = n.stage(q_via, secs=3.0)
        ok2, _e2 = n.stage(q_obs, secs=3.0)
        t["move_to_observe_s"] = round(time.time() - t0, 2)
        if not (ok1 and ok2):
            raise DetectionUnavailable(
                "the %s arm did not reach the observe pose" % arm)
        n.spin(settle_s)
        for _ in range(40):
            if n.rgb is not None and n.depth is not None and n.info is not None:
                break
            n.spin(1.0)
        if n.rgb is None or n.depth is None or n.info is None:
            raise DetectionUnavailable(
                "no camera stream on /%s_camera -- is mock_rgbd_camera or the "
                "real driver running?" % arm)
        t0 = time.time()
        img, dep = n.image(), n.depth_m()
        p_cam, R_wc = n.cam_pose()
        dets, rejected = classify(img, dep, n.info)
        for d in dets:
            d["world"] = [float(v) for v in deproject(d, n.info, p_cam, R_wc)]
        t["detect_s"] = round(time.time() - t0, 3)
    finally:
        if return_home:
            t0 = time.time()
            try:
                n.stage(q_via, secs=3.0)
                n.stage(q_home, secs=3.0)
            except Exception:                                    # noqa: BLE001
                pass
            t["return_home_s"] = round(time.time() - t0, 2)
        if own:
            n.destroy_node()

    if expect is not None and len(dets) != expect:
        raise DetectionUnavailable(
            "saw %d cubes, the task expects %d. Refusing to pick blind; a "
            "fallback to declared coordinates here would look exactly like a "
            "working camera." % (len(dets), expect))

    colours = M.PLANE_COLOURS
    cubes = []
    for d in sorted(dets, key=lambda z: z["world"][0]):
        if d["colour"] not in colours:
            continue
        cubes.append((round(float(d["world"][0]), 4),
                      round(float(d["world"][1]), 4),
                      colours.index(d["colour"])))
    t["cubes"] = cubes
    t["rejected_by_size"] = len(rejected)
    t["added_total_s"] = round(sum(v for k, v in t.items()
                                   if k.endswith("_s")), 2)
    t["added_per_pick_s"] = (round(t["added_total_s"] / len(cubes), 2)
                             if cubes else None)
    return cubes, t
