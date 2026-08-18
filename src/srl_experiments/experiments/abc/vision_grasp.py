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

# HOW FAR OFF THE WORK PLANE A DETECTION MAY LAND AND STILL BE A CUBE.
#
# BACK TO ONE CUBE, AND THE MAT IS EXCLUDED BY WHERE IT IS INSTEAD.
#
# It was tightened to 10 mm on 2026-08-18 to reject pad fragments, which on
# the rebuilt layout sit only `PAD_T` under a cube's centre: measured, cubes
# landed 0.3 to 1.8 mm off the plane and fragments 16 to 19, so 10 mm looked
# like a clean cut with six times the margin either way.
#
# IT IS NOT, AND THE SWEEP SAID SO IMMEDIATELY: the left arm came back with
# ZERO cubes on its own side. Standing alone the arm settles to within 2 mm of
# the observe pose and the cubes deproject within 2 mm of the plane; inside a
# recording, with a scene node, two cameras and an RViz capture on the same
# machine, it does not, and the cube's own z error eats the 6 mm that
# separated the two groups. A discriminator with 6 mm of headroom is a
# coin-toss dressed as a threshold.
PLANE_WINDOW_M = 0.040
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


def check_frame_still(n, p_cam, tol_m=0.004):
    """Was the arm still when this frame was taken? Returns the drift, or None.

    Raises DetectionUnavailable if the camera has moved more than `tol_m`
    between the pose the FRAME was rendered from and the pose the camera is at
    NOW.

    WHY THIS IS A FUNCTION AND NOT FOUR LINES INSIDE THE LOOK. It is the check
    that makes the 2026-08-17 fault impossible to record silently, and a check
    that has never been seen to fire is not a check. Inline, it could only be
    exercised by getting a real arm to move at the right instant;
    `scripts/verify_frame_pose_gate.py` calls it directly with a deliberately
    faked render pose and asserts it raises.

    Using the frame's own pose (see `Vision.cam_pose`) already makes a
    SETTLED-but-slightly-wrong arm harmless -- the deprojection is then
    self-consistent and correct. What it cannot fix is an arm still MOVING:
    the render pose is stale by the time the frame is processed and the image
    is smeared across poses anyway. This is the check for that, and it is
    separate on purpose.

    Returns None when there is nothing to compare -- a real camera that does
    not stamp a render pose, or no TF. UNKNOWN, never a quiet zero.
    """
    if getattr(n, "render_pose", None) is None:
        return None
    try:
        live, _R = _live_cam_pose(n)
    except Exception:                                            # noqa: BLE001
        return None
    drift = float(np.linalg.norm(np.asarray(p_cam, float)
                                 - np.asarray(live, float)))
    if drift > tol_m:
        # THE TWO POSITIONS, NOT JUST THE DISTANCE. A refusal that reports one
        # number cannot be told apart from a refusal that reports a CONSTANT,
        # and "the same value twice" is the difference between an arm that is
        # moving and a frame mismatch. It read exactly 0.0170 m on two
        # separate sweep runs, which is not what drift looks like.
        raise DetectionUnavailable(
            "the camera moved %.4f m between rendering this frame and now "
            "(tolerance %.4f m), so the arm was not still when it was taken. "
            "render (%.4f, %.4f, %.4f) live (%.4f, %.4f, %.4f) delta "
            "(%+.4f, %+.4f, %+.4f). Deprojecting it would return a confident "
            "wrong answer -- the 2026-08-17 fault, refused rather than "
            "recorded."
            % (drift, tol_m,
               p_cam[0], p_cam[1], p_cam[2], live[0], live[1], live[2],
               p_cam[0] - live[0], p_cam[1] - live[1], p_cam[2] - live[2]))
    return round(drift, 6)


def _live_cam_pose(n):
    """The camera pose from LIVE TF, whatever `cam_pose()` decides to use.

    `Vision.cam_pose()` now prefers the pose stamped onto the frame, which is
    the right answer for deprojection and the wrong one for "has the arm
    moved". Both are needed and they must come from different places, so this
    reaches past it deliberately.
    """
    import rclpy.time
    t = n.buf.lookup_transform("world", "%s_camera_color_frame" % n.arm,
                               rclpy.time.Time())
    tr = t.transform.translation
    return np.array([tr.x, tr.y, tr.z]), t.transform.rotation


def observe_and_detect(arm, node=None, settle_s=6.0, expect=None,
                       return_home=True, frame_pose_tol_m=0.004,
                       frame_pose_wait_s=8.0):
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
    # ---- PAUSE THE FOLLOWER FOR THE WHOLE LOOK -------------------------
    #
    # `Vision.stage()` publishes a joint trajectory to the SAME controller
    # `ik_follower_node` streams position commands to, so once the follower
    # has a target it fights the observe move and the arm drifts inside
    # `stage()`'s 0.02 rad-per-joint arrival tolerance while the frame is
    # being taken.
    #
    # MEASURED, 2026-08-17, inside the recording sweep: the four T1 cubes came
    # back 31.7 / 32.7 / 34.0 / 35.7 mm from truth against a 30 mm capture
    # gate, so NONE of the four was grasped, and the pad miss at closure was
    # 31.6 / 32.6 / 33.9 / 35.6 mm -- the same numbers to 0.1 mm. Standalone
    # on the same stack, with nothing else loading the machine, the identical
    # code returned 1.3-3.0 mm, which is why it read as flaky.
    #
    # Two things were wrong and BOTH are fixed, because either alone leaves a
    # way for the failure to come back:
    #   * the arm was not holding the pose -- fixed here, by pausing;
    #   * the deprojection read the camera pose LATER off live TF instead of
    #     the pose the frame came from -- fixed in `Vision.cam_pose()`, which
    #     now uses the pose `mock_rgbd_camera` stamps onto each frame.
    #
    # It is the same treatment `stage_presentation_pose.py` already gives the
    # staging move, through the same module, so there is one implementation.
    sys.path.insert(0, os.path.join(_ROOT, "scripts"))
    import follower_pause as FP
    paused = FP.pause(n)
    t["followers"] = dict(paused)
    try:
        n.spin(4.0)
        t0 = time.time()
        ok1, _e1 = n.stage(q_via, secs=3.0)
        ok2, e_obs = n.stage(q_obs, secs=3.0)
        t["move_to_observe_s"] = round(time.time() - t0, 2)
        t["observe_arrival_rad"] = None if e_obs is None else round(e_obs, 5)
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
        # THE ARM MUST HAVE BEEN STILL WHEN THE FRAME WAS TAKEN.
        #
        # Using the frame's own render pose fixes the deprojection for an arm
        # that has settled somewhere slightly wrong -- the answer is then
        # self-consistent and correct. What it cannot fix is an arm that is
        # still MOVING: the render pose is then already stale by the time the
        # frame is processed, and on a real camera the picture is smeared
        # across poses as well. So compare the pose the FRAME came from against
        # the pose the camera is at NOW.
        #
        # WAIT FOR A STILL FRAME; DO NOT REFUSE THE FIRST BAD ONE. A drift over
        # tolerance means THIS frame cannot be trusted, not that the arm will
        # never be still. Two things produce a transient here and both clear
        # within a second or two: the arm finishing its approach, and --
        # measured on this host -- the camera's own TF listener being starved
        # while the DDS graph is busy, so it renders from a transform several
        # hundred ms old.
        #
        # Refusing the first bad frame turned a transient into a lost clip. It
        # read exactly 0.0170 m on two separate sweep runs, and a value that
        # repeats to 0.1 mm across differently loaded runs is a fixed lag, not
        # an arm still travelling.
        #
        # It still REFUSES if it never settles, reporting how long it waited
        # and the worst it saw, so a permanently stale camera is still caught.
        # That is the property that makes the 2026-08-17 fault impossible to
        # record silently.
        img = dep = p_cam = R_wc = None
        best, waited = None, 0.0
        while True:
            img, dep = n.image(), n.depth_m()
            p_cam, R_wc = n.cam_pose()
            try:
                t["frame_pose_drift_m"] = check_frame_still(
                    n, p_cam, frame_pose_tol_m)
                t["frame_pose_waited_s"] = round(waited, 2)
                break
            except DetectionUnavailable as e:
                best = str(e)
                if waited > frame_pose_wait_s:
                    raise DetectionUnavailable(
                        "no still frame in %.1f s. %s"
                        % (frame_pose_wait_s, best))
                n.spin(0.4)
                waited += 0.4
        t["frame_pose_is_stamped"] = n.render_pose is not None
        dets, rejected = classify(img, dep, n.info)
        for d in dets:
            d["world"] = [float(v) for v in deproject(d, n.info, p_cam, R_wc)]
        # ==============================================================
        # A CUBE IS ON THE WORK PLANE. A FRAGMENT OF A PAD IS NOT.
        # ==============================================================
        # THE PADS ARE THE CUBES' OWN COLOURS, which is what makes the task
        # judgeable from a frame and also means the only thing separating a
        # cube from a piece of pad is blob SIZE at range. That is not enough:
        # two cubes standing in front of a 210 x 130 mm pad split it into
        # several connected fragments, the big one is rejected as too large and
        # the leftovers are cube-sized. Measured inside the sweep, this refused
        # T1 twice with "saw 8 cubes" and "saw 9 cubes" while the same code
        # standalone on the same stack saw 4 -- an intermittency that is really
        # a sensitivity to exactly where the arm settled.
        #
        # The nine split cleanly by DEPTH, and that is the discriminator the
        # size test was missing:
        #
        #     4 detections   z 1.1118 .. 1.1191    <-  T1_Z is 1.1200
        #     5 detections   z 0.9576 .. 0.9601    <-  160 mm low
        #
        # so the real cubes land within 9 mm of the plane they rest on and the
        # spurious ones are 160 mm off. A 40 mm window -- one cube -- sits an
        # order of magnitude clear of both.
        #
        # THIS IS A PHYSICAL FACT, NOT A FUDGE. T1's cubes rest on the work
        # plane; `work_surface.WORK_PLANE_M` owns where that is, and a cube
        # 160 mm below it is not a cube the arm can pick. It is also a check
        # that CAN fail on a good input -- move a cube off the plane and it is
        # rejected and named -- which is the property the size test alone had
        # for size but not for height. See test_cube_must_be_on_work_plane.py.
        on_plane, off_plane = [], []
        for d in dets:
            dz = float(d["world"][2]) - M.T1_Z
            d["dz_from_work_plane_m"] = round(dz, 4)
            (on_plane if abs(dz) <= PLANE_WINDOW_M
             else off_plane).append(d)
        for d in off_plane:
            rejected.append(dict(
                d, why="not on the work plane: %+.1f mm from z = %.4f "
                       "(window +/-%.0f mm) -- a fragment of a same-coloured "
                       "pad, not a cube"
                       % (d["dz_from_work_plane_m"] * 1000.0, M.T1_Z,
                          PLANE_WINDOW_M * 1000.0)))
        dets = on_plane
        t["rejected_off_work_plane"] = len(off_plane)
        # ==============================================================
        # A DETECTION STANDING ON A MAT, AT THE MAT'S OWN HEIGHT, IS THE MAT
        # ==============================================================
        # The pads are the cubes' own colours -- deliberately, so that "each
        # cube ended on the pad of its own colour" is judgeable from a frame --
        # and on the rebuilt layout they are mats lying ON the table the cubes
        # stand on. So a fragment of pad is the right colour, at very nearly
        # the right height, and at 0.5 m it is the right SIZE too: measured,
        # the left camera returned five blue blobs where there are two cubes,
        # the extra three being pad fragments 37 to 51 px against an expected
        # cube of 38 to 45.
        #
        # NEITHER SIZE NOR HEIGHT SEPARATES THEM. The heights differ by only
        # `PAD_T`, and tightening the plane window to 10 mm to exploit that
        # left 6 mm of headroom -- which the arm's own settling ate the first
        # time it ran inside a sweep, returning ZERO cubes on the left side.
        #
        # SO THE PAD IS EXCLUDED BY WHERE IT IS, which is the one thing about
        # it that is not in doubt: it is a FIXTURE, at a declared position,
        # 160 x 140 mm. A blob whose deprojected centre lies inside a pad's
        # footprint AND at or below the pad's own surface is a piece of that
        # pad. A CUBE STANDING ON THE PAD IS NOT EXCLUDED -- its centre is
        # half a cube above the mat, well clear of the test -- which matters
        # because that is exactly where the cubes end up.
        #
        # THIS DOES NOT WEAKEN THE VISION CLAIM AND IT IS WORTH SAYING WHY.
        # What is being claimed is that the CUBE'S COLOUR comes from the
        # pixels: `verify_vision_drives_grasp` flips every declared colour and
        # requires the plan not to move. Using a fixture's declared FOOTPRINT
        # to avoid measuring the fixture says nothing about any cube's colour.
        # The alternative -- inferring the mat's position from the image -- is
        # solving a harder problem than the task has.
        import t1_task as _T1
        pad_h = (_T1.PAD_W / 2.0, _T1.PAD_D / 2.0)
        pad_top = _T1.TABLE_TOP + _T1.PLANE_T
        keep_p, on_pad = [], []
        for d in dets:
            w = d["world"]
            inside = any(abs(w[0] - px) <= pad_h[0] + 0.005
                         and abs(w[1] - py) <= pad_h[1] + 0.005
                         for px, py in _T1.T1_PLANES)
            if inside and float(w[2]) <= pad_top + _T1.CUBE_M / 4.0:
                on_pad.append(dict(
                    d, why="inside the %s pad's footprint at z = %.4f, which "
                           "is the MAT's own surface (%.4f) and not a cube "
                           "standing on it (%.4f)"
                           % ("blue" if w[0] > 0 else "green", w[2], pad_top,
                              pad_top + _T1.CUBE_M / 2.0)))
            else:
                keep_p.append(d)
        rejected.extend(on_pad)
        dets = keep_p
        t["rejected_as_pad"] = len(on_pad)
        # ==============================================================
        # TWO 40 mm CUBES CANNOT BE 25 mm APART. ONE OF THEM IS THE PAD.
        # ==============================================================
        # The work-plane window above removes pad fragments that are far BELOW
        # the plane. It cannot remove the ones that are level with it, and they
        # exist: the blue pad sits 95 mm behind the cube row and 20 mm below it,
        # and from a camera looking down at the observe pose those two offsets
        # very nearly cancel in RANGE. Measured -- the sliver of blue pad
        # showing just above the green cube at x = 0.620:
        #
        #     green cube    world [0.6198, 0.1179, 1.1178]   39 px   depth 0.7160
        #     blue sliver   world [0.6222, 0.1431, 1.1220]   33 px   depth 0.7240
        #
        # 2.4 mm apart in x, both within 5 mm of the work plane. No height or
        # size test can separate those, and the pair is what made the detector
        # report five cubes.
        #
        # But they cannot both be cubes. The cubes are 40 mm wide on a 60 mm
        # pitch, so ANY two detections closer together than one cube width are
        # the same physical object seen twice -- once as itself and once as a
        # piece of whatever is behind it. The one with the larger blob is the
        # object; a sliver is by definition the smaller.
        #
        # PURELY PERCEPTUAL, ON PURPOSE. This uses the cubes' own SIZE and
        # nothing about where they are declared to be, so it does not weaken
        # the claim that vision drives the grasp: the mislabel control still
        # flips every declared colour and the plan is unchanged. Applying the
        # pads' declared footprint as an image mask would have worked too, and
        # was not done for exactly that reason.
        dets.sort(key=lambda z: -float(z.get("blob_px", 0)))
        kept, merged = [], []
        for d in dets:
            near = next((k for k in kept
                         if math.dist(d["world"][:2], k["world"][:2])
                         < M.CUBE_M), None)
            if near is None:
                kept.append(d)
            else:
                merged.append(dict(
                    d, why="a duplicate of the %s detection %.1f mm away "
                           "(%.0f px against %.0f px) -- two %.0f mm cubes "
                           "cannot be that close, so the smaller blob is a "
                           "sliver of the object behind it"
                           % (near.get("colour"),
                              math.dist(d["world"][:2],
                                        near["world"][:2]) * 1000.0,
                              d.get("blob_px", 0), near.get("blob_px", 0),
                              M.CUBE_M * 1000.0)))
        rejected.extend(merged)
        dets = sorted(kept, key=lambda z: z["world"][0])
        t["rejected_as_duplicates"] = len(merged)
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
        # ALWAYS, ON EVERY EXIT PATH. A failed restore disarms the arm, which
        # is the safe direction and also the one that costs every run after it.
        FP.resume(n, paused)
        t["followers"] = dict(paused)
        if own:
            n.destroy_node()

    if expect is not None and len(dets) != expect:
        # SAY WHAT IT SAW, not just how many. The count alone sent a session
        # hunting: "saw 8 cubes, expects 4" inside the sweep while the SAME
        # code standalone on the same stack saw 4, and the message carried
        # nothing to tell an extra blob from a duplicated one. A refusal that
        # cannot be diagnosed costs more than the run it correctly refused.
        _lines = []
        for d in sorted(dets, key=lambda z: (z.get("colour", ""),
                                             z.get("u", 0))):
            _w = d.get("world")
            _lines.append(
                "      %-6s u,v %6.1f,%6.1f  blob %5.1f px (cube would be "
                "%5.1f)  depth %.4f  world %s"
                % (d.get("colour", "?"), d.get("u", -1), d.get("v", -1),
                   d.get("blob_px", -1), d.get("expected_px", -1),
                   d.get("depth_m", -1),
                   "-" if _w is None
                   else "[%.4f, %.4f, %.4f]" % tuple(_w)))
        for r in rejected:
            _lines.append(
                "      %-6s u,v %6.1f,%6.1f  blob %5.1f px (cube would be "
                "%5.1f)  depth %.4f  REJECTED: %s"
                % (r.get("colour", "?"), r.get("u", -1), r.get("v", -1),
                   r.get("blob_px", -1), r.get("expected_px", -1),
                   r.get("depth_m", -1), r.get("why", "")))
        raise DetectionUnavailable(
            "saw %d cubes, the task expects %d. Refusing to pick blind; a "
            "fallback to declared coordinates here would look exactly like a "
            "working camera.\n    ACCEPTED %d, REJECTED BY SIZE %d:\n%s"
            % (len(dets), expect, len(dets), len(rejected),
               "\n".join(_lines)))

    colours = M.PLANE_COLOURS
    cubes = []
    # WHAT THE CLASSIFIER HAD TO GO ON, PER CUBE, kept beside the answer.
    #
    # `cubes` is deliberately a bare [(x, y, pad_index)] -- it is what the
    # builder consumes and nothing more -- so the evidence for each entry used
    # to be discarded at this line. The GUI's prompt panel shows the operator
    # what the camera decided and how sure it was BEFORE the arm moves, and
    # "how sure" needs a number.
    #
    # THE NUMBER IS `hsv_margin_min`: the smallest distance, in 8-bit counts,
    # between this blob's median pixel and the edge of the colour band it was
    # matched to, over H, S and V. It is a real margin rather than a
    # probability -- 0 means the pixel is exactly on the boundary and a
    # neighbouring shade would have classified differently -- and it is what
    # the detector actually decided on, which is why it is reported instead of
    # a softmax nothing computes.
    seen = []
    for d in sorted(dets, key=lambda z: z["world"][0]):
        if d["colour"] not in colours:
            continue
        cubes.append((round(float(d["world"][0]), 4),
                      round(float(d["world"][1]), 4),
                      colours.index(d["colour"])))
        m = d.get("hsv_margin_min") or []
        seen.append(dict(colour=d["colour"],
                         x=round(float(d["world"][0]), 4),
                         y=round(float(d["world"][1]), 4),
                         z=round(float(d["world"][2]), 4),
                         blob_px=d.get("blob_px"),
                         expected_px=d.get("expected_px"),
                         depth_m=d.get("depth_m"),
                         hsv_median=d.get("hsv_median"),
                         hsv_margin_min=list(m),
                         confidence_counts=(min(m) if m else None)))
    t["cubes"] = cubes
    t["seen"] = seen
    t["rejected"] = [dict(colour=r.get("colour"), why=r.get("why"),
                          blob_px=r.get("blob_px"),
                          expected_px=r.get("expected_px"))
                     for r in rejected]
    t["rejected_by_size"] = len(rejected)
    t["added_total_s"] = round(sum(v for k, v in t.items()
                                   if k.endswith("_s")), 2)
    t["added_per_pick_s"] = (round(t["added_total_s"] / len(cubes), 2)
                             if cubes else None)
    return cubes, t
