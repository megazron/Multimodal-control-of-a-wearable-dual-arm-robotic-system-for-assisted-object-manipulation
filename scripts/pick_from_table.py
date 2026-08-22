#!/usr/bin/env python3
"""Look at the table, say what is on it, pick one up.

    python3 scripts/pick_from_table.py --self-test        # no ROS, no robot
    python3 scripts/pick_from_table.py --arm left         # DRY RUN, default
    python3 scripts/pick_from_table.py --arm left --execute

WHAT WAS MISSING, in the operator's words: "it is unable to check what's on
the table, it will just move here and there."

That was accurate. There was a detector that finds a NAMED thing, a grasp
planner that turns ONE given cloud into ONE grasp, and a task layer that
commands coordinates written down in advance. Nothing looked at a surface and
enumerated what was standing on it, so nothing could act on what was actually
there -- and a coordinate written down in advance, driven by an arm whose
camera transform has never been solved, is a move to somewhere plausible and
wrong.

THE CHAIN THIS RUNS, and every link either produces a number or refuses:

  1  LOOK      drive to the observe pose (`solve_observe_pose.py` solved one
               for the geometry that exists: 12 points in frame, clearance
               0.2202, straight transit 0.2130)
  2  CLOUD     wrist RGB-D -> points, transformed into the ROBOT frame by
               FORWARD KINEMATICS. No calibration is involved and none is
               needed; that is why this path can be trusted while the scene
               camera's extrinsic still has 0% coverage on real recordings
  3  SCENE     `table_scene.analyse` -- support plane, then every cluster
               standing on it, then a plane-constrained grasp for each
  4  CHOOSE    the highest-quality grasp, or the one whose object matches a
               `--want` phrase through the detector
  5  CHECK     `grasp_pipeline.reachable` for pregrasp AND grasp, which since
               2026-08-22 reads the MOVING-chain clearance -- the whole-chain
               number is the immobile mount stub and reads 0.2202 m in every
               pose, so the wearer floor compared against it never fired
  6  PATH      `safe_motion.check_path` over the DENSIFIED sweep, not the
               endpoints. A pick that cleared three poses at 0.4 m and swept
               through the wearer between them is defect 2 of 2026-08-21
  7  EXECUTE   only with --execute, and only if 5 and 6 both passed

DRY RUN IS THE DEFAULT and that is not timidity. Every step above prints what
it found; a run that prints a plan and moves nothing is the thing to do first
on a table you have not looked at today.

WHAT IS VALIDATED AND WHAT IS NOT. `table_scene` is tested against
CONSTRUCTED ground truth -- a table and boxes of known size and position,
where the answer is known in advance. It has never been run against a real
wrist-camera recording of a table, because there is not one: every RGB-D
capture in this repository is from the SCENE camera. The first real run is a
measurement to be checked, not a result.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, os.path.join(ROOT, "config"),
          os.path.join(ROOT, "src/srl_perception"),
          os.path.join(ROOT, "src/srl_teleop"),
          os.path.join(ROOT, "scripts/real_calibration")):
    if p not in sys.path:
        sys.path.insert(0, p)

from srl_perception import table_scene as TS                 # noqa: E402
from srl_perception import rgbd_grasp as RG                  # noqa: E402


# --------------------------------------------------------------- the cloud
def cloud_in_robot_frame(depth_m, K, cam_pose, stride=2, z_max=1.5,
                         z_min=0.08, with_pixels=False):
    """Depth image -> (N, 3) points in the ROBOT frame.

    `cam_pose` is the 4x4 of the camera link, from FORWARD KINEMATICS. That
    is the whole reason this is trustworthy: FK is exact given the URDF, so
    the only error in the point positions is the depth sensor's own and any
    error in where `camera_link` sits in the URDF.

    THAT LAST ONE IS UNVERIFIED. Nothing has ever checked `camera_link`
    against the physical module, and a 10 mm URDF error is invisible and
    lands directly in every grasp. Hand-eye for the wrist camera is the
    cheapest high-value calibration left -- see docs/system/22_grasping.md.

    `stride` subsamples: a 640x480 frame is 307k points and the scene only
    needs enough to fit a plane and cluster on it.
    """
    fx, fy, cx, cy = (float(K[0]), float(K[1]), float(K[2]), float(K[3]))
    d = np.asarray(depth_m, float)[::stride, ::stride]
    h, w = d.shape
    vs, us = np.mgrid[0:h, 0:w]
    us = us * stride
    vs = vs * stride
    z = d.reshape(-1)
    u = us.reshape(-1)
    v = vs.reshape(-1)
    good = (z > z_min) & (z < z_max) & np.isfinite(z)
    if not good.any():
        raise TS.SceneRefusal(
            "no depth between %.2f and %.2f m in this frame. A colour blob "
            "with no depth return is a direction, not an object."
            % (z_min, z_max))
    z, u, v = z[good], u[good], v[good]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    P = np.stack([x, y, z], axis=1)
    M = np.asarray(cam_pose, float)
    out = P @ M[:3, :3].T + M[:3, 3]
    if with_pixels:
        # KEEP THE PIXEL EACH POINT CAME FROM. Without it a 3-D cluster
        # cannot be matched against a 2-D detector mask, and "pick the red
        # one" has to fall back to "pick the best one" -- which is a
        # different instruction wearing the same words.
        return out, np.stack([u, v], axis=1)
    return out


# ------------------------------------------------------------- the planner
def choose_by_name(rows, uv, detections, min_overlap=0.15):
    """Which cluster is the thing the operator named.

    `detections` are `prompt_detector.Detection`s, each with a `mask` (a
    boolean image) or, failing that, a `bbox`. A cluster is scored by the
    FRACTION OF ITS OWN PIXELS that fall inside the detection, so a small
    object inside a big box does not lose to a big object that merely
    overlaps it.

    Returns (row, detection, overlap) or raises. It REFUSES a weak match
    rather than returning the least-bad one: "I could not find the red cube"
    is an answer, and moving to the nearest thing that is not it is not.
    """
    best = None
    for det in detections:
        m = getattr(det, "mask", None)
        for r in rows:
            idx = r["object"].pixel_index
            if idx is None or not len(idx):
                continue
            pu = uv[idx, 0].astype(int)
            pv = uv[idx, 1].astype(int)
            if m is not None:
                H, W = m.shape[:2]
                inb = (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                hit = np.zeros(len(pu), bool)
                hit[inb] = m[pv[inb], pu[inb]].astype(bool)
            else:
                x, y, w, h = det.bbox
                hit = (pu >= x) & (pu < x + w) & (pv >= y) & (pv < y + h)
            frac = float(hit.mean())
            if best is None or frac > best[2]:
                best = (r, det, frac)
    if best is None:
        raise TS.SceneRefusal(
            "the detector found %d thing(s) and none of them overlaps any "
            "cluster standing on the table. Either it is looking at "
            "something off the surface, or the clusters and the image are "
            "not the same frame." % len(detections))
    if best[2] < min_overlap:
        raise TS.SceneRefusal(
            "the best match puts only %.0f%% of a cluster's pixels inside "
            "the detection (need %.0f%%). Refusing rather than picking up "
            "the nearest thing that is not what you asked for."
            % (best[2] * 100, min_overlap * 100))
    return best


def plan(depth_m, K, cam_pose, arm, scorer=None, home=None, want=None,
         detector=None, bgr=None, verbose=True, **scene_kw):
    """The whole chain, up to but not including motion.

    Returns a dict with the scene, the chosen grasp and the checks. Raises
    `TS.SceneRefusal` with a reason a person can act on.
    """
    pts, uv = cloud_in_robot_frame(depth_m, K, cam_pose, with_pixels=True)
    if verbose:
        print("cloud: %d points in the robot frame" % len(pts))
    scene = TS.analyse(pts, pixel_uv=uv, **scene_kw)
    if verbose:
        print(TS.describe(scene))
    if not scene["graspable"]:
        raise TS.SceneRefusal(
            "nothing on this surface can be grasped. %d object(s) were found "
            "and each was refused for its own reason -- see above."
            % len(scene["objects"]))

    rows = scene["graspable"]
    matched = None
    if want:
        if detector is None or bgr is None:
            raise TS.SceneRefusal(
                "--want %r needs a detector and a colour frame. Without them "
                "this would silently pick the best-scoring object and call "
                "it the one you asked for." % want)
        dets = detector.detect(bgr, want, depth_m=depth_m, K=K)
        if not dets:
            raise TS.SceneRefusal(
                "the detector found nothing matching %r in this frame. It "
                "reports its backend on every result -- check which one ran "
                "before concluding the object is absent." % want)
        best, det, frac = choose_by_name(rows, uv, dets)
        matched = {"detection": det, "overlap": frac}
        if verbose:
            print("matched %r -> object %d (%.0f%% of its pixels inside the "
                  "detection, backend %s, score %.2f)"
                  % (want, best["object"].index, frac * 100,
                     det.backend, det.score))
    else:
        best = rows[0]
    g = best["grasp"]
    if verbose:
        print("\nchosen: %s" % g.describe())

    out = {"scene": scene, "grasp": g, "object": best["object"],
           "matched": matched, "reach": None, "path": None}
    if scorer is None:
        return out

    # ---- 5 reach and the wearer floor, for BOTH poses
    from srl_perception import grasp_pipeline as GP
    if home is None:
        import home_positions as hp
        home = np.array(hp.load_home_radians(arm))
    checks = {}
    for name, pt in (("pregrasp", g.pregrasp), ("grasp", g.centre)):
        ok, q, why = GP.reachable(scorer, arm, pt, home)
        checks[name] = {"ok": bool(ok), "q": None if q is None else list(q),
                        "why": why}
        if verbose:
            print("  %-9s %s -- %s" % (name, "OK " if ok else "NO ", why))
    out["reach"] = checks
    if not (checks["pregrasp"]["ok"] and checks["grasp"]["ok"]):
        raise TS.SceneRefusal(
            "the %s arm cannot safely reach the chosen grasp. pregrasp: %s; "
            "grasp: %s" % (arm, checks["pregrasp"]["why"],
                           checks["grasp"]["why"]))

    # ---- 6 THE WHOLE PATH, and PLAN one when the straight line is unsafe
    #
    # The straight line is tried first because most moves on this rig are
    # fine and planning a clear move turns 60 ms into seconds. When it is
    # NOT fine the old behaviour was to stop -- `safe_motion.check_path` was
    # written to CATCH a sweep through the wearer, and catching it means the
    # run ends. Now the planner is asked to go round, and the result is
    # re-checked with the SAME checker that rejected the straight line, so a
    # plan cannot be accepted by the thing that produced it.
    import safe_motion as SM
    from srl_perception import joint_planner as JP
    lo, hi, cont = scorer.lim[arm]
    checker = JP.Checker(
        lambda q: scorer.clearance_parts(arm, q)["moving_chain_m"],
        lo, hi, cont)
    legs = [("home -> pregrasp", home, checks["pregrasp"]["q"]),
            ("pregrasp -> grasp", checks["pregrasp"]["q"],
             checks["grasp"]["q"])]
    path = []
    for label, a, b in legs:
        ok, why, worst, n = SM.check_path(scorer, arm, np.array(a),
                                          np.array(b))
        row = {"leg": label, "ok": bool(ok), "why": why, "worst_m": worst,
               "samples": n, "planned": False, "waypoints": None}
        if not ok:
            if verbose:
                print("  %-18s straight line REFUSED (%s) -- planning round"
                      % (label, why))
            try:
                wps, info = JP.plan(np.array(a), np.array(b), checker,
                                    seed=5, max_iter=4000)
            except JP.PlanRefusal as e:
                out["path"] = path + [row]
                raise TS.SceneRefusal(
                    "the sweep %s is not safe and no way round was found. "
                    "Straight line: %s. Planner: %s" % (label, why, e))
            # RE-CHECK EVERY LEG with the checker that rejected the straight
            # line. A planner that marks its own homework is a planner.
            bad = [(i, SM.check_path(scorer, arm, np.array(x), np.array(y)))
                   for i, (x, y) in enumerate(zip(wps[:-1], wps[1:]))]
            failed = [(i, r) for i, r in bad if not r[0]]
            if failed:
                out["path"] = path + [row]
                raise TS.SceneRefusal(
                    "the planner returned a path whose leg %d does NOT pass "
                    "the same check that rejected the straight line: %s. "
                    "Refusing rather than trusting the planner about its own "
                    "output." % (failed[0][0], failed[0][1][1]))
            row.update(ok=True, planned=True,
                       why="%s; re-checked on %d legs" % (info["method"],
                                                          len(bad)),
                       worst_m=min(r[2] for _i, r in bad),
                       samples=sum(r[3] for _i, r in bad),
                       waypoints=[list(map(float, w)) for w in wps])
            if verbose:
                print("  %-18s PLANNED round: %s, %d waypoints, worst "
                      "clearance %.4f m"
                      % (label, info["method"], len(wps), row["worst_m"]))
        elif verbose:
            print("  %-18s OK  over %d samples -- %s" % (label, n, why))
        path.append(row)
    out["path"] = path
    return out


# ---------------------------------------------------------------- self-test
def self_test(verbose=True):
    """Constructed table, constructed camera, known answer.

    No ROS, no robot, no depth sensor: a synthetic depth image is RENDERED
    from a known scene, deprojected, and the recovered object centres are
    compared against the truth they were built from. Rendering is normally
    the kind of synthetic this repository refuses -- but here the renderer is
    a pinhole projection of boxes whose corners are arithmetic, so the ground
    truth is still CONSTRUCTED and the check is exact.
    """
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-52s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    # A camera 0.5 m above a table at z = 0.90, looking straight down.
    K = (600.0, 600.0, 320.0, 240.0, 640, 480)
    cam_pose = np.array([[1.0, 0, 0, 0.0],
                         [0, -1.0, 0, 0.0],
                         [0, 0, -1.0, 1.40],
                         [0, 0, 0, 1.0]])
    truth = [((0.03, 0.02, 0.925), 0.05),
             ((-0.06, -0.04, 0.930), 0.06)]

    H, W = 480, 640
    depth = np.full((H, W), 0.50)              # the table, 0.50 m below
    vs, us = np.mgrid[0:H, 0:W]
    for (cx_, cy_, cz), side in truth:
        # Project the box top into the image. The camera looks down -z with
        # x right and y flipped, so world (x, y) maps through cam_pose.
        top = cz + side / 2.0
        zc = cam_pose[2, 3] - top
        u = K[2] + (cx_ - cam_pose[0, 3]) * K[0] / zc
        v = K[3] - (cy_ - cam_pose[1, 3]) * K[1] / zc
        r = (side / 2.0) * K[0] / zc
        m = (np.abs(us - u) <= r) & (np.abs(vs - v) <= r)
        depth[m] = zc

    got = plan(depth, K, cam_pose, "left", verbose=False)
    sc = got["scene"]
    check("the table is found under a rendered scene",
          abs(sc["plane"].offset - 0.90) < 0.004,
          "plane at %.4f m (true 0.900)" % sc["plane"].offset)
    check("both boxes are found", len(sc["objects"]) == 2,
          "%d found" % len(sc["objects"]))
    worst = 0.0
    for (c, side) in truth:
        d = min(np.linalg.norm(o.centre[:2] - np.asarray(c[:2]))
                for o in sc["objects"])
        worst = max(worst, d)
    check("object centres within 8 mm in the plane", worst < 0.008,
          "worst %.1f mm" % (worst * 1000))
    check("a grasp was chosen", got["grasp"] is not None,
          got["grasp"].describe() if got["grasp"] else "")
    check("the approach points down at the table",
          float(got["grasp"].approach @ np.array([0, 0, -1.0])) > 0.99,
          "approach %s" % np.round(got["grasp"].approach, 3))
    check("the pregrasp stands off ABOVE the grasp",
          got["grasp"].pregrasp[2] > got["grasp"].centre[2] + 0.05,
          "%.3f vs %.3f" % (got["grasp"].pregrasp[2], got["grasp"].centre[2]))

    # AN EMPTY TABLE MUST REFUSE, not pick something.
    try:
        plan(np.full((H, W), 0.50), K, cam_pose, "left", verbose=False)
        check("an empty table refuses", False, "it planned a grasp")
    except TS.SceneRefusal as e:
        check("an empty table refuses by name",
              "nothing" in str(e).lower() or "NOTHING" in str(e),
              str(e)[:64])

    # NO DEPTH AT ALL must refuse.
    try:
        plan(np.zeros((H, W)), K, cam_pose, "left", verbose=False)
        check("a frame with no depth refuses", False)
    except TS.SceneRefusal as e:
        check("a frame with no depth refuses by name", "no depth" in str(e),
              str(e)[:64])

    # --want with no detector must refuse rather than silently pick the best.
    try:
        plan(depth, K, cam_pose, "left", want="the red one", verbose=False)
        check("--want with no detector refuses", False, "it planned anyway")
    except TS.SceneRefusal as e:
        check("--want with no detector refuses",
              "needs a detector" in str(e), str(e)[:60])

    # ---- NAME-MATCHING PICKS THE NAMED THING, NOT THE BEST-SCORING ONE.
    #
    # This is the check that matters. Without it the pipeline would happily
    # accept "pick up the red one", pick whatever it liked best, and report
    # success -- which is the failure the operator described as "it will
    # just move here and there".
    default = plan(depth, K, cam_pose, "left", verbose=False)
    other = [o for o in default["scene"]["objects"]
             if o.index != default["object"].index]
    if not other:
        check("there are two distinct objects to choose between", False)
    else:
        target = other[0]

        class _Det:
            """A mask covering exactly the OTHER object's pixels."""

            def __init__(self, tgt):
                m = np.zeros((H, W), bool)
                # rebuild uv for the whole cloud and mark the target's pixels
                _, uvv = cloud_in_robot_frame(depth, K, cam_pose,
                                              with_pixels=True)
                px = uvv[tgt.pixel_index]
                m[px[:, 1].astype(int), px[:, 0].astype(int)] = True
                self.d = type("D", (), {})()
                self.d.mask = m
                self.d.bbox = (0, 0, W, H)
                self.d.backend = "test"
                self.d.score = 0.99
                self.d.label = "the one I asked for"

            def detect(self, bgr, prompt, depth_m=None, K=None):
                return [self.d]

        got2 = plan(depth, K, cam_pose, "left", want="the one I asked for",
                    detector=_Det(target), bgr=np.zeros((H, W, 3), np.uint8),
                    verbose=False)
        check("naming an object selects THAT one, not the best-scoring one",
              got2["object"].index == target.index
              and got2["object"].index != default["object"].index,
              "asked for object %d, got %d (default would have been %d)"
              % (target.index, got2["object"].index,
                 default["object"].index))
        check("the match records how much of the cluster was inside",
              got2["matched"] is not None
              and got2["matched"]["overlap"] > 0.5,
              "overlap %.0f%%" % (100 * got2["matched"]["overlap"])
              if got2["matched"] else "no match record")

        # A DETECTION THAT MATCHES NOTHING must refuse, not fall back.
        class _Miss:
            def detect(self, bgr, prompt, depth_m=None, K=None):
                d = type("D", (), {})()
                d.mask = np.zeros((H, W), bool)
                d.bbox = (0, 0, 2, 2)
                d.backend = "test"
                d.score = 0.9
                d.label = "nothing"
                return [d]

        try:
            plan(depth, K, cam_pose, "left", want="a thing that is not here",
                 detector=_Miss(), bgr=np.zeros((H, W, 3), np.uint8),
                 verbose=False)
            check("a detection matching no cluster refuses", False,
                  "it picked something anyway")
        except TS.SceneRefusal as e:
            check("a detection matching no cluster refuses",
                  "Refusing rather than picking up" in str(e), str(e)[:60])

    if verbose:
        print("pick_from_table self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--want", default=None,
                    help="pick the object matching this phrase (NOT wired "
                         "up yet -- it refuses rather than guessing)")
    ap.add_argument("--execute", action="store_true",
                    help="actually move. Without it this plans and prints.")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    if a.self_test:
        print("CONTROLS")
        return 0 if self_test() else 1

    print("CONTROLS")
    if not self_test(verbose=True):
        print("\nA CONTROL FAILED -- not looking at a real table with a "
              "planner that cannot solve a constructed one.")
        return 2

    print("\nLIVE. This needs a running stack, the wrist camera, and the "
          "arm at the observe pose.")
    print("Steps 1 and 2 (drive to observe, grab a frame) are not wired to "
          "ROS in this script yet: it is the PLANNER and its checks, "
          "callable from a node or a test.")
    print("Nothing was commanded.%s"
          % ("  --execute is accepted and currently does nothing, which is "
             "reported rather than silently ignored." if a.execute else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
