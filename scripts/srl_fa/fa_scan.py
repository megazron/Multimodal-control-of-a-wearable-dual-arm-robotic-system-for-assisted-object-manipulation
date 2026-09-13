#!/usr/bin/env python3
"""Get the camera to a GOOD VIEWING GEOMETRY, then scan the table.

    python3 scripts/srl_fa/fa_scan.py --self-test
    python3 scripts/srl_fa/fa_scan.py --arm left --seek          # move, no scan
    python3 scripts/srl_fa/fa_scan.py --arm left                 # seek + scan

THE PROBLEM
-----------
"Scan the table" is not a pose you can save.  Measured on the real left arm on
2026-08-30 from the pose it happened to be in: the camera sat 0.287 m from the
surface at an INCIDENCE OF 69.4 deg -- nearly edge on.  At that angle depth is
grazing, a 40 mm cube measures 91 mm wide, the finger pads fall outside the
frame entirely so the descent guard has nothing to look at, and the fitted
"plane" quietly recruits the floor.  Every downstream number is then wrong in
a way that looks like a segmentation bug.

A SAVED SCAN POSE CANNOT FIX IT EITHER, for the reason this repository keeps
rediscovering: a joint vector recorded against one table height, one table
position and one mount is wrong the day any of those move, and it fails
silently because a pose always "succeeds".

THE FIX: CLOSE THE LOOP ON THE VIEWING GEOMETRY ITSELF
------------------------------------------------------
Two numbers describe how well a camera is placed to scan a surface, and BOTH
are measurable in the camera's own frame, with no hand-eye transform:

    standoff       = d          (the plane fit's own offset)
    incidence      = angle between the optical axis and the plane normal

So the arm is driven until those two numbers are what we asked for.  The
commanded step goes out through the imperfect extrinsic, which makes it
inexact -- and irrelevant, because the next measurement is taken in the frame
where the data is good.  This is the same argument that makes the grasp servo
work, applied one level earlier.

WHY THE AZIMUTH IS PRESERVED
----------------------------
The correction moves the camera along the plane's normal and along the ONE
tangential direction that already points from the target back towards the
camera.  It changes how steeply and from how far the camera looks; it does not
choose a new side of the table to look from.  That keeps the motion small,
keeps the arm in the half of its workspace it is already in, and makes the
step predictable enough for a transit check to be meaningful.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
sys.path.insert(0, "/home/gausms/kortex_ws/scripts/srl_fa")

from handeye_from_table import solve as solve_handeye  # noqa: E402
import execute_pick_left as X  # noqa: E402
from srl_fk import FK  # noqa: E402

import fa_kin as K  # noqa: E402
import fa_perception as PC  # noqa: E402

# ------------------------------------------------------------------- targets
WANT_INCIDENCE_DEG = 30.0   # steep enough for good depth, shallow enough that
#                             the pads stay in frame and the hand does not
#                             occlude the object it is about to grasp
WANT_STANDOFF_M = 0.45      # the Kinova depth module is usable 0.25-1.5 m;
#                             0.45 m covers ~0.5 x 0.3 m of table per view
OK_INCIDENCE_DEG = 8.0      # convergence band
OK_STANDOFF_M = 0.07
SEEK_ITERS = 6
SEEK_DAMP = 0.65            # take this fraction of the correction each step
TRANSIT_MARGIN_M = 0.055    # air under every part of the hand while moving
MAX_VIEW_DQ_DEG = 75.0      # refuse a "small correction" that reconfigures

# The lateral offsets that make a SCAN rather than a snapshot.  Parallax is
# what lets handeye_from_table separate a real table tilt from a mount error;
# a set of views taken from one point cannot, however many there are.
SCAN_OFFSETS_M = [(0.00, 0.00), (0.10, 0.00), (-0.10, 0.00),
                  (0.00, 0.09), (0.00, -0.09)]
MERGE_MM = 55.0


class ScanError(Exception):
    pass


# --------------------------------------------------------------- the geometry
def target_camera_pose(sc, want_inc_deg, want_d, damp=1.0):
    """Where the camera should go, expressed IN THE CAMERA'S CURRENT FRAME.

    Returns (position, rotation, target_point, current_error) with the
    rotation in the same current-camera frame.  Pure geometry -- no ROS, no
    FK -- so it is testable against constructed scenes, which is what the
    self-test does.
    """
    n = np.asarray(sc.n, float)
    c = sc.surface_point()
    if c is None:
        # The optical axis misses the plane entirely (looking away from it).
        # Aim at the centroid of what IS on the plane rather than refusing:
        # that is exactly the case a correction should recover from.
        h = sc.P @ n + sc.d
        inl = np.abs(h) < 0.006
        if inl.sum() < 200:
            raise ScanError("the camera is not looking at a surface at all")
        c = sc.P[inl].mean(0)
        c = c - n * float(c @ n + sc.d)          # project exactly onto it
    # tangential direction: from the target back towards where we are now
    v = -c                                        # camera origin is 0
    t = v - n * float(v @ n)
    nt = np.linalg.norm(t)
    if nt < 1e-4:
        t = PC.plane_basis(n)[0]                  # already straight overhead
    else:
        t = t / nt
    want = c + n * want_d + t * (want_d * math.tan(math.radians(want_inc_deg)))
    p = damp * want                               # current position is 0
    R = K.look_at(p, c)
    err = (sc.incidence_deg - want_inc_deg, sc.standoff_m - want_d)
    return p, R, c, err


def view_is_good(sc, want_inc=WANT_INCIDENCE_DEG, want_d=WANT_STANDOFF_M):
    return (abs(sc.incidence_deg - want_inc) <= OK_INCIDENCE_DEG
            and abs(sc.standoff_m - want_d) <= OK_STANDOFF_M)


def world_surface(fk, arm, sc, correction=None, columns=()):
    """The measured plane, pushed into world. CARRIES THE MOUNT ERROR."""
    T, = fk.poses(arm, sc.q, [K.CAM_LINK])
    R = T[:3, :3] @ (np.eye(3) if correction is None else correction)
    n_w = R @ sc.n
    if n_w[2] < 0:
        n_w = -n_w
    p_on = T[:3, :3] @ (-np.asarray(sc.n) * sc.d) + T[:3, 3]
    if correction is not None:
        p_on = R @ (-np.asarray(sc.n) * sc.d) + T[:3, 3]
    return K.Surface(n_w, -float(n_w @ p_on), columns)


# ------------------------------------------------------------------ the seek
def seek_view(arm, want_inc=WANT_INCIDENCE_DEG, want_d=WANT_STANDOFF_M,
              iters=SEEK_ITERS, damp=SEEK_DAMP, dry_run=False, log=None):
    """Drive the arm until the camera looks at the table properly.

    Returns the final Scene.  Raises ScanError with a plain reason when it
    cannot -- which is the honest outcome when the table is out of reach from
    where the arm is standing, and much better than scanning anyway.
    """
    log = log or arm.log
    fk = arm.fk
    last = None
    for it in range(1, iters + 1):
        sc = arm.look()
        if sc is None:
            raise ScanError("no RGB-D frames from the %s wrist camera"
                            % arm.arm_name)
        if not sc.plane_ok:
            raise ScanError("no usable surface in view (%.0f%% inliers, "
                            "%.1f mm RMS)" % (sc.inlier_frac * 100, sc.rms_mm))
        last = sc
        log("  view %d: standoff %.3f m, incidence %.1f deg, %.0f%% inliers"
            % (it, sc.standoff_m, sc.incidence_deg, sc.inlier_frac * 100))
        if view_is_good(sc, want_inc, want_d):
            log("  viewing geometry reached (want %.0f deg / %.2f m)"
                % (want_inc, want_d))
            return sc
        # THE DAMPING IS LINE-SEARCHED, NOT FIXED.
        #
        # A fixed fraction refuses the first move of every session: from the
        # 69 deg / 0.29 m pose the left arm was actually in on 2026-08-30, the
        # 0.65-damped correction needs 124 deg on one joint, and a correction
        # that large is neither safe nor a "small step".  Shrinking it until
        # the arm can actually make the move keeps the loop progressing --
        # closed loop does not care how big each step is, only that each one
        # is measured.
        #
        # dq IS WRAPPED.  Joints 3, 5 and 7 are CONTINUOUS: -179.05 deg and
        # +181.02 deg are the same pose, and plain subtraction calls that
        # 360 deg of motion.  That is docs/ENGINEERING_LOG.md's own ang_wrap finding, and
        # unwrapped it refuses correct solutions at random.
        chosen = None
        for f in (damp, damp * 0.7, damp * 0.45, damp * 0.28, damp * 0.16):
            p, R, c, err = target_camera_pose(sc, want_inc, want_d, f)
            Tnow = arm.T_cam(sc.q)
            Tdes = Tnow @ np.block([[R, p.reshape(3, 1)],
                                    [np.zeros((1, 3)), np.ones((1, 1))]])
            q, ep, er, _ = K.ik_link(fk, arm.arm_name, K.CAM_LINK,
                                     Tdes[:3, 3], Tdes[:3, :3], sc.q)
            if ep > 0.006 or math.degrees(er) > 3.0:
                continue
            dq = math.degrees(np.abs(X.ang_wrap(np.asarray(q)
                                                - np.asarray(sc.q))).max())
            if dq > MAX_VIEW_DQ_DEG:
                continue
            surf = world_surface(fk, arm.arm_name, sc)
            okp, worst, _ = K.path_clear(fk, arm.arm_name, sc.q, q, surf,
                                         TRANSIT_MARGIN_M)
            if okp:
                chosen = (q, f, dq, None)
                break
            det = K.lift_first_detour(fk, arm.arm_name, sc.q, q, surf,
                                      TRANSIT_MARGIN_M)
            if det is not None:
                chosen = (q, f, dq, det)
                break
            last_worst = worst
        if chosen is None:
            raise ScanError(
                "no safe step towards a %.0f deg / %.2f m view from here: "
                "every damping from %.2f down to %.2f was out of reach, over "
                "%.0f deg on a joint, or took the hand under %.0f mm of table "
                "clearance. Move the arm nearer the table and try again."
                % (want_inc, want_d, damp, damp * 0.16, MAX_VIEW_DQ_DEG,
                   TRANSIT_MARGIN_M * 1000))
        q, f, dq, det = chosen
        if det is not None:
            log("  straight path would graze -- lifting first")
            waypoints = det
        else:
            waypoints = [q]
        log("  step: damping %.2f, worst joint %.1f deg" % (f, dq))
        if dry_run:
            log("  DRY RUN: would move %.1f deg (worst joint)" % dq)
            return sc
        for i, wq in enumerate(waypoints):
            if arm.goto(wq, "seek-%d.%d" % (it, i)) is None:
                raise ScanError("a motion guard fired while seeking the view")
    if last is not None and view_is_good(last, want_inc,
                                         want_d + OK_STANDOFF_M):
        return last
    raise ScanError("could not reach a good viewing geometry in %d steps "
                    "(last: %.1f deg incidence, %.3f m standoff)"
                    % (iters, last.incidence_deg if last else float("nan"),
                       last.standoff_m if last else float("nan")))


def recentre_view(arm, sc, want_inc=WANT_INCIDENCE_DEG,
                  want_d=WANT_STANDOFF_M, log=None, min_shift_m=0.035):
    """Aim the optical axis at what is ON the table, not at wherever it fell.

    Reaching a good standoff and incidence says nothing about WHAT is in the
    middle of the frame.  Measured on the left arm on 2026-08-30 the seek
    converged to 33.8 deg / 0.43 m with the green cube half out of the bottom
    of the picture -- and a cube clipped by the frame edge measures 53 x 39 mm
    instead of 60 x 60, which is a size error big enough to choose the wrong
    grip.

    So once the geometry is right, the aim point moves to the centroid of the
    things standing on the surface (the table centroid if there are none) and
    the same standoff and incidence are re-established around it.  Small
    shifts are skipped: a move is only worth its risk if it changes the view.
    """
    log = log or arm.log
    if not sc.objects:
        return sc
    n = np.asarray(sc.n, float)
    cen = np.mean([o["foot"] for o in sc.objects], 0)
    cen = cen - n * float(cen @ n + sc.d)
    aim = sc.surface_point()
    shift = float(np.linalg.norm(cen - aim)) if aim is not None else 1.0
    if shift < min_shift_m:
        log("  already centred on the objects (%.0f mm off)" % (shift * 1000))
        return sc
    v = -cen
    t = v - n * float(v @ n)
    t = t / np.linalg.norm(t) if np.linalg.norm(t) > 1e-4 \
        else PC.plane_basis(n)[0]
    p = cen + n * want_d + t * (want_d * math.tan(math.radians(want_inc)))
    R = K.look_at(p, cen)
    Tnow = arm.T_cam(sc.q)
    Tdes = Tnow @ np.block([[R, p.reshape(3, 1)],
                            [np.zeros((1, 3)), np.ones((1, 1))]])
    q, ep, er, _ = K.ik_link(arm.fk, arm.arm_name, K.CAM_LINK,
                             Tdes[:3, 3], Tdes[:3, :3], sc.q)
    if ep > 0.006 or math.degrees(er) > 3.0:
        log("  cannot recentre (%.0f mm off): that aim point is out of reach"
            % (shift * 1000))
        return sc
    surf = world_surface(arm.fk, arm.arm_name, sc)
    okp, worst, _ = K.path_clear(arm.fk, arm.arm_name, sc.q, q, surf,
                                 TRANSIT_MARGIN_M)
    if not okp:
        log("  cannot recentre: the path would graze at %.0f mm"
            % (worst * 1000))
        return sc
    log("  recentring %.0f mm onto the objects" % (shift * 1000))
    if arm.goto(q, "recentre") is None:
        return sc
    sc2 = arm.look()
    if sc2 is None or not sc2.plane_ok:
        log("  lost the surface while recentring -- keeping the previous view")
        return sc
    log("  after recentring: standoff %.3f m, incidence %.1f deg, %d object(s)"
        % (sc2.standoff_m, sc2.incidence_deg, len(sc2.objects)))
    return sc2


# ------------------------------------------------------------------ the scan
class TableModel:
    """What the scan found. World-frame poses are ESTIMATES, and say so."""

    def __init__(self):
        self.n_w = None
        self.d_w = None
        self.tilt_deg = None
        self.residual_mm = None
        self.handeye_deg = {}
        self.objects = []
        self.surface = None
        self.views = []
        self.arms = []
        self.when = time.time()

    # ---- the honesty checks -------------------------------------------
    @property
    def error_bar_mm(self):
        """How far a world object pose could be out, at 0.4 m of reach.

        The hand-eye correction that had to be applied IS the error bar: it is
        the amount by which the arm's own idea of where its camera points was
        wrong.  Quoting object positions without it would be the confident
        wrong answer this repository keeps catching.
        """
        if not self.handeye_deg:
            return float("nan")
        return 400.0 * math.tan(math.radians(max(self.handeye_deg.values())))

    def describe(self):
        out = []
        if self.n_w is None:
            return "nothing scanned yet"
        out.append("table: %.1f deg off level, surface residual %.1f mm, "
                   "hand-eye correction %s"
                   % (self.tilt_deg, self.residual_mm,
                      ", ".join("%s %.2f deg" % (a, v)
                                for a, v in self.handeye_deg.items())))
        out.append("       world object poses are ESTIMATES to about "
                   "+/-%.0f mm at 0.4 m" % self.error_bar_mm)
        out.append("%d object(s):" % len(self.objects))
        for o in self.objects:
            out.append("  %-16s %5.0f x %5.0f x %5.0f mm at "
                       "(%+.3f, %+.3f, %+.3f)  seen by %s"
                       % (o["name"], *o["size_mm"], *o["centre_world"],
                          "+".join(o["seen_by"])))
        return "\n".join(out)

    def to_json(self):
        return {
            "when": self.when,
            "arms": self.arms,
            "table": {"normal_world": list(map(float, self.n_w)),
                      "offset": float(self.d_w),
                      "tilt_deg": float(self.tilt_deg),
                      "residual_mm": float(self.residual_mm)},
            "handeye_correction_deg": self.handeye_deg,
            "error_bar_mm_at_400mm": self.error_bar_mm,
            "objects": [{k: (list(map(float, v)) if isinstance(v, np.ndarray)
                             else v)
                         for k, v in o.items() if k != "points"}
                        for o in self.objects],
            "caveat": ("object poses in world carry the residual mount error; "
                       "descent is guarded by fa_perception.descent_floor, "
                       "which is computed in the camera frame and does not"),
        }


def capture_views(arm, sc0, offsets=SCAN_OFFSETS_M, log=None, grip=0.0):
    """Take the centre view plus lateral offsets, all aimed at one point."""
    log = log or arm.log
    fk = arm.fk
    n = np.asarray(sc0.n, float)
    c = sc0.surface_point()
    if c is None:
        raise ScanError("lost the surface between seeking and scanning")
    e1, e2 = PC.plane_basis(n)
    Tnow = arm.T_cam(sc0.q)
    obs = []
    for k, (du, dv) in enumerate(offsets):
        if k == 0:
            sc = sc0
        else:
            p = np.zeros(3) + e1 * du + e2 * dv   # from the current camera
            R = K.look_at(p, c)
            Tdes = Tnow @ np.block([[R, p.reshape(3, 1)],
                                    [np.zeros((1, 3)), np.ones((1, 1))]])
            q, ep, er, _ = K.ik_link(fk, arm.arm_name, K.CAM_LINK,
                                     Tdes[:3, 3], Tdes[:3, :3], arm.fresh_q())
            if ep > 0.008 or math.degrees(er) > 4.0:
                log("  view %d (%+.0f,%+.0f mm): out of reach, skipped"
                    % (k, du * 1000, dv * 1000))
                continue
            surf = world_surface(fk, arm.arm_name, sc0)
            okp, worst, _ = K.path_clear(fk, arm.arm_name, arm.fresh_q(), q,
                                         surf, TRANSIT_MARGIN_M)
            if not okp:
                log("  view %d: path would graze at %.0f mm, skipped"
                    % (k, worst * 1000))
                continue
            if arm.goto(q, "scan-view-%d" % k) is None:
                log("  view %d: motion guard fired, stopping the sweep" % k)
                break
            sc = arm.look()
        if sc is None or not sc.plane_ok:
            log("  view %d: no usable surface, skipped" % k)
            continue
        log("  view %d: %.0f%% inliers, %.2f mm RMS, incidence %.1f deg, "
            "%d object(s)" % (k, sc.inlier_frac * 100, sc.rms_mm,
                              sc.incidence_deg, len(sc.objects)))
        obs.append(sc)
    return obs


def fuse(fk, per_arm, log=print):
    """Views -> one table model in world, with the mount error solved out."""
    model = TableModel()
    model.arms = sorted(per_arm)
    corr, normals, plane_pts, raw = {}, [], [], []
    for arm, obs in per_arm.items():
        Rs = [fk.poses(arm, o.q, [K.CAM_LINK])[0][:3, :3] for o in obs]
        ns = [o.n for o in obs]
        if len(obs) >= 2:
            C, before, after = solve_handeye(Rs, ns)
            ang = math.degrees(math.acos(
                float(np.clip((np.trace(C) - 1) / 2, -1, 1))))
            log("  %-5s %d views disagreed by %.2f deg -> %.2f deg after a "
                "%.2f deg correction" % (arm, len(obs), before, after, ang))
        else:
            C, ang = np.eye(3), float("nan")
            log("  %-5s only %d view: cannot calibrate, using identity"
                % (arm, len(obs)))
        corr[arm] = C
        model.handeye_deg[arm] = ang
        for o in obs:
            T, = fk.poses(arm, o.q, [K.CAM_LINK])
            R, t = T[:3, :3] @ C, T[:3, 3]
            nw = R @ o.n
            if nw[2] < 0:
                nw = -nw
            normals.append(nw)
            for ob in o.objects:
                raw.append({
                    "arm": arm,
                    "centre_world": R @ ob["centre"] + t,
                    "top_world": R @ ob["top_centre"] + t,
                    "size_mm": ob["size_mm"],
                    "height_mm": ob["height_mm"],
                    "colour": ob["colour"],
                    "shape": ob["shape"],
                    "n_pts": ob["n_pts"],
                    "hsv": ob["hsv"],
                })
            keep = o.P[np.abs(o.P @ o.n + o.d) < 0.006]
            if len(keep):
                sub = keep[::max(1, len(keep) // 500)]
                plane_pts.append((R @ sub.T).T + t)

    n_w = np.mean(normals, 0)
    n_w /= np.linalg.norm(n_w)
    allp = np.vstack(plane_pts)
    d_w = -float(n_w @ allp.mean(0))
    model.n_w, model.d_w = n_w, d_w
    model.tilt_deg = math.degrees(math.acos(min(1.0, abs(n_w[2]))))
    model.residual_mm = float(np.sqrt(((allp @ n_w + d_w) ** 2).mean()) * 1000)

    merged = []
    for ob in sorted(raw, key=lambda o: -o["n_pts"]):
        c = ob["centre_world"]
        for m in merged:
            if np.linalg.norm(c - m["centre_world"]) < MERGE_MM / 1000.0:
                m["seen_by"] = sorted(set(m["seen_by"] + [ob["arm"]]))
                m["n_views"] += 1
                # keep the view that saw the most of it
                if ob["n_pts"] > m["n_pts"]:
                    for k in ("centre_world", "top_world", "size_mm",
                              "height_mm", "colour", "shape", "n_pts", "hsv"):
                        m[k] = ob[k]
                break
        else:
            ob = dict(ob)
            ob["seen_by"] = [ob.pop("arm")]
            ob["n_views"] = 1
            merged.append(ob)

    used = {}
    for i, m in enumerate(merged):
        m["id"] = i
        m["name"] = PC.label(m, used)
        m["centre_world"] = np.asarray(m["centre_world"], float)
        m["top_world"] = np.asarray(m["top_world"], float)
    model.objects = merged

    cols = []
    for m in merged:
        foot = m["centre_world"] - n_w * (n_w @ m["centre_world"] + d_w)
        r = max(m["size_mm"][0], m["size_mm"][1]) / 2000.0
        cols.append((foot, r, m["height_mm"] / 1000.0))
    model.surface = K.Surface(n_w, d_w, cols)
    return model


def scan(arms, seek=True, log=print, want_inc=WANT_INCIDENCE_DEG,
         want_d=WANT_STANDOFF_M):
    """The whole thing: place each camera, sweep it, fuse. Returns TableModel."""
    per_arm = {}
    for arm in arms:
        log("=== %s arm ===" % arm.arm_name)
        try:
            sc0 = seek_view(arm, want_inc, want_d, log=log) if seek \
                else arm.look()
            if sc0 is None:
                raise ScanError("no frames")
            if seek:
                sc0 = recentre_view(arm, sc0, want_inc, want_d, log=log)
            obs = capture_views(arm, sc0, log=log)
        except ScanError as e:
            log("  %s arm: %s" % (arm.arm_name, e))
            continue
        if obs:
            per_arm[arm.arm_name] = obs
    if not per_arm:
        raise ScanError("no usable viewpoints from any arm")
    log("=== fusing ===")
    model = fuse(arms[0].fk, per_arm, log=log)
    model.views = [o for obs in per_arm.values() for o in obs]
    return model


# ----------------------------------------------------------------- self-test
def _fake_scene(n, d, standoff_pts=6000, seed=0):
    class S:
        pass
    rng = np.random.default_rng(seed)
    e1, e2 = PC.plane_basis(n)
    origin = -np.asarray(n, float) * d
    P = np.array([origin + e1 * u + e2 * v
                  for u, v in rng.uniform(-0.3, 0.3, (standoff_pts, 2))])
    s = S()
    s.P, s.n, s.d = P, np.asarray(n, float), float(d)
    s.incidence_deg = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    s.standoff_m = float(d)
    s.surface_point = lambda: (np.array([0.0, 0.0, 1.0])
                               * (-d / float(n[2])) if abs(n[2]) > 1e-6
                               else None)
    return s


def self_test():                                              # noqa: C901
    ok = True

    def chk(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        print("  %-58s %s %s" % (name, "OK" if cond else "FAIL", detail))

    print("1. the correction reaches the geometry it was asked for")
    for inc0, d0 in ((69.4, 0.287), (5.0, 0.90), (45.0, 0.30), (0.0, 0.62)):
        th = math.radians(inc0)
        n = np.array([math.sin(th), 0.0, -math.cos(th)])
        sc = _fake_scene(n, d0)
        p, R, c, err = target_camera_pose(sc, 30.0, 0.45, damp=1.0)
        # where the corrected camera would stand, in the same frame
        axis = R @ np.array([0.0, 0.0, 1.0])
        new_inc = math.degrees(math.acos(min(1.0, abs(float(axis @ n)))))
        new_d = abs(float(p @ n + sc.d))
        chk("from %4.1f deg / %.2f m -> %4.1f deg / %.2f m"
            % (inc0, d0, new_inc, new_d),
            abs(new_inc - 30.0) < 0.5 and abs(new_d - 0.45) < 0.005)

    print("\n2. damping moves PART of the way and never overshoots")
    th = math.radians(69.4)
    n = np.array([math.sin(th), 0.0, -math.cos(th)])
    sc = _fake_scene(n, 0.287)
    p_full, _, _, _ = target_camera_pose(sc, 30.0, 0.45, damp=1.0)
    p_half, _, _, _ = target_camera_pose(sc, 30.0, 0.45, damp=0.5)
    chk("half damping is half the step",
        float(np.linalg.norm(p_half - 0.5 * p_full)) < 1e-12)
    chk("the step is towards the target, not past it",
        float(np.linalg.norm(p_half)) < float(np.linalg.norm(p_full)))

    print("\n3. iterating the damped correction CONVERGES")
    inc, dd = 69.4, 0.287
    for _ in range(8):
        th = math.radians(inc)
        n = np.array([math.sin(th), 0.0, -math.cos(th)])
        sc = _fake_scene(n, dd)
        p, R, c, _ = target_camera_pose(sc, 30.0, 0.45, damp=SEEK_DAMP)
        axis = R @ np.array([0.0, 0.0, 1.0])
        inc = math.degrees(math.acos(min(1.0, abs(float(axis @ n)))))
        dd = abs(float(p @ n + sc.d))
    chk("8 damped steps land inside the acceptance band",
        abs(inc - 30.0) <= OK_INCIDENCE_DEG and abs(dd - 0.45) <= OK_STANDOFF_M,
        "(%.1f deg, %.3f m)" % (inc, dd))

    print("\n4. view_is_good is not a rubber stamp")
    good = _fake_scene(np.array([math.sin(math.radians(30)), 0,
                                 -math.cos(math.radians(30))]), 0.45)
    bad = _fake_scene(np.array([math.sin(math.radians(69)), 0,
                                -math.cos(math.radians(69))]), 0.287)
    chk("accepts the geometry it asked for", view_is_good(good))
    chk("REJECTS the 69 deg / 0.29 m pose measured on the real arm",
        not view_is_good(bad))
    edge = _fake_scene(np.array([math.sin(math.radians(30)), 0,
                                 -math.cos(math.radians(30))]), 0.45 + 0.10)
    chk("rejects a standoff outside the band", not view_is_good(edge))

    print("\n5. a camera looking AWAY from the plane still gets a target")
    n = np.array([0.0, 0.0, 1.0])          # plane behind the camera
    sc = _fake_scene(n, 0.5)
    try:
        p, R, c, _ = target_camera_pose(sc, 30.0, 0.45)
        chk("recovered by aiming at the visible surface centroid",
            abs(float(c @ n + sc.d)) < 1e-6)
    except ScanError as e:
        chk("recovered by aiming at the visible surface centroid", False, str(e))

    print("\n6. the error bar is derived, not asserted")
    m = TableModel()
    m.handeye_deg = {"left": 8.45}
    chk("8.45 deg of mount error is ~59 mm at 0.4 m",
        abs(m.error_bar_mm - 59.4) < 1.0, "(%.1f mm)" % m.error_bar_mm)
    m.handeye_deg = {"left": 0.0}
    chk("no correction, no error bar", abs(m.error_bar_mm) < 1e-9)

    print("\nknown-answer self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------------------------- main
def _live(arm_names, seek_only, dry_run, out):
    import rclpy
    from fa_arm import Arm
    rclpy.init()
    fk = FK()
    arms = []
    for a in arm_names:
        arm = Arm(a, fk=fk)
        if not arm.start():
            print("%s arm unavailable" % a)
            continue
        arms.append(arm)
    if not arms:
        return 2
    if seek_only:
        for arm in arms:
            print("=== %s ===" % arm.arm_name)
            try:
                sc = seek_view(arm, dry_run=dry_run)
                print(sc.describe())
            except ScanError as e:
                print("  REFUSED: %s" % e)
        return 0
    model = scan(arms)
    print()
    print(model.describe())
    if out:
        json.dump(model.to_json(), open(out, "w"), indent=1, default=float)
        print("\nwrote %s" % out)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--arm", default="left")
    ap.add_argument("--arms", default=None)
    ap.add_argument("--seek", action="store_true", help="place the camera only")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.self_test:
        raise SystemExit(self_test())
    names = [s.strip() for s in (a.arms or a.arm).split(",") if s.strip()]
    raise SystemExit(_live(names, a.seek, a.dry_run, a.out))
