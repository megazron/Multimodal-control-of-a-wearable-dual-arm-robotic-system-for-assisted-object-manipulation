#!/usr/bin/env python3
"""PROMPT -> DETECTION -> GRASP -> REACHABILITY, as one callable.

    plan = plan_grasp(bgr, depth_m, "the green cube", cam_p, cam_R, K, dK)

Everything the autonomy layer needs from a camera frame, with every stage's
evidence kept so a refusal names the stage that produced it. A pipeline that
returns None tells the operator nothing; this one says WHICH step failed and
what number failed it.

THE STAGES, AND WHAT EACH CAN REFUSE
  detect   nothing matched the prompt, or the colour backend was given a word
           it does not know
  depth    the object was found but has no depth return -- a bearing, not a
           position, and acting on it means guessing a range
  grasp    too few points, or wider than the jaw opens
  reach    no IK, or the path breaches the wearer clearance floor

REACHABILITY IS CHECKED WITH A MULTI-START SOLVER. A single seed at home with
a term pulling solutions back toward home is a local optimiser told to stay
put -- it reports "unreachable" for points the arm reaches easily. That
mistake was made in this repository on 2026-08-21 and contradicted by an
operator moving the arm there by hand.
"""
from __future__ import annotations

import math

import numpy as np

from srl_perception import rgbd_grasp as RG
from srl_perception.prompt_detector import PromptDetector

CONT_IDX = (0, 2, 4, 6)
SEAM_MARGIN_RAD = 0.30
CLEARANCE_FLOOR_M = 0.150


class PlanFailure(Exception):
    def __init__(self, stage, reason, evidence=None):
        super().__init__("%s: %s" % (stage, reason))
        self.stage = stage
        self.reason = reason
        self.evidence = evidence or {}


def mask_for(det, shape):
    """The detection's pixels. Falls back to its bounding box."""
    if det.mask is not None:
        return det.mask
    x, y, w, h = det.bbox
    m = np.zeros(shape[:2], bool)
    m[max(0, y):y + h, max(0, x):x + w] = True
    return m


def plan_grasp(bgr, depth_m, prompt, cam_p, cam_R, colour_K, depth_K,
               detector=None, near_m=None, far_m=None, standoff_m=0.12,
               approach_hint=None):
    """Full plan for the best match to `prompt`, in the ROBOT frame."""
    det = detector or PromptDetector(backend="auto", near_m=near_m, far_m=far_m)
    hits = det.detect(bgr, prompt, depth_m=depth_m, K=colour_K,
                      depth_K=depth_K)
    if not hits:
        raise PlanFailure("detect", "nothing in view matched %r" % prompt,
                          dict(backend=det.backend))
    best = hits[0]
    if best.depth_m is None:
        raise PlanFailure(
            "depth",
            "found %r at pixel (%.0f, %.0f) but there is no depth return "
            "there, so it is a direction and not a position (%s)"
            % (best.label, best.centre_uv[0], best.centre_uv[1],
               best.extra.get("depth_note", "")),
            dict(detection=best.as_dict()))

    # the object's pixels, re-projected into the DEPTH image and turned into a
    # cloud. Colour and depth are different resolutions, so this maps through
    # the bearing rather than indexing one image with the other's pixels.
    m = mask_for(best, bgr.shape)
    vs, us = np.nonzero(m)
    bx = (us - colour_K[2]) / colour_K[0]
    by = (vs - colour_K[3]) / colour_K[1]
    du = np.round(depth_K[2] + bx * depth_K[0]).astype(int)
    dv = np.round(depth_K[3] + by * depth_K[1]).astype(int)
    H, W = depth_m.shape[:2]
    ok = (du >= 0) & (du < W) & (dv >= 0) & (dv < H)
    dmask = np.zeros((H, W), bool)
    dmask[dv[ok], du[ok]] = True
    pts = RG.cloud_from_mask(depth_m, dmask, depth_K)
    try:
        # The cloud is in the CAMERA frame, so the line of sight is +z.
        # Passing it in is what stops the jaw being planned along the one
        # direction the camera could not measure.
        g = RG.grasp_from_cloud(pts, approach_hint=approach_hint,
                                view_axis=np.array([0.0, 0.0, 1.0]))
    except RG.GraspRefusal as exc:
        # SAY HOW TO FIX IT, NOT JUST THAT IT FAILED.
        #
        # Measured on the rig 2026-08-21: the target cube at 1.5 m gave 18
        # depth points where 30 are needed. "Too few points" is true and
        # useless; an autonomy layer reading it cannot act. The depth sensor
        # is 480x270, so an object's pixel area falls as 1/range^2 -- which
        # means the range that WOULD give enough points is arithmetic, not a
        # guess, and the robot can simply go there.
        ev = dict(n_points=int(len(pts)), detection=best.as_dict())
        reason = str(exc)
        if "depth points" in reason and best.depth_m and len(pts) > 0:
            need = RG.MIN_POINTS
            closer = best.depth_m * math.sqrt(len(pts) / float(need))
            ev["suggested_range_m"] = round(float(closer), 3)
            reason += (". At %.2f m it subtends %d points; move to about "
                       "%.2f m and it would subtend %d, because area on a "
                       "480x270 depth sensor falls as 1/range^2."
                       % (best.depth_m, len(pts), closer, need))
        raise PlanFailure("grasp", reason, ev) from None
    gr = RG.to_robot_frame(g, cam_p, cam_R)
    # A SANITY CHECK ON THE ANSWER ITSELF. A detection that lands below the
    # floor or above the rig is not a detection, however confident the
    # pipeline is about it -- measured: a black segment reported at
    # z = -0.176 m, which is underground.
    z = float(gr["centre"][2])
    if not (0.30 <= z <= 2.00):
        raise PlanFailure(
            "sanity",
            "the object solves to z = %.3f m, which is outside anything this "
            "rig can contain (0.30 to 2.00 m). Either the detection is a "
            "false positive or the camera pose is wrong." % z,
            dict(centre=[float(v) for v in gr["centre"]],
                 detection=best.as_dict()))
    gr["pregrasp"] = RG.pregrasp(gr, standoff_m)
    gr["detection"] = best.as_dict()
    gr["n_points"] = int(len(pts))
    gr["backend"] = det.backend
    return gr


def reachable(scorer, arm, goal, home, restarts=40, seed=11,
              seam=SEAM_MARGIN_RAD, floor=CLEARANCE_FLOOR_M):
    """(ok, q, why) for a Cartesian point. MULTI-START, deliberately.

    `scorer` is solve_home_pose.Scorer; passed in rather than imported so this
    module stays importable with no ROS and no robot.
    """
    from scipy.optimize import minimize
    rng = np.random.default_rng(seed)
    lo, hi, _ = scorer.lim[arm]
    goal = np.asarray(goal, float)

    def hand(q):
        return np.asarray(
            scorer.arm_terms(arm, q, with_clearance=False)["hand"], float)

    def cost(q):
        return float(np.sum((hand(q) - goal) ** 2))

    sols = []
    for s0 in [np.asarray(home, float)] + [rng.uniform(lo, hi)
                                           for _ in range(restarts)]:
        r = minimize(cost, s0, method="L-BFGS-B", bounds=list(zip(lo, hi)),
                     options=dict(maxiter=400, ftol=1e-14))
        if math.sqrt(max(r.fun, 0.0)) < 0.010:
            sols.append(r.x)
    if not sols:
        return False, None, ("no IK solution in %d starts -- this point is "
                             "outside the arm's %d mm reach, not blocked by a "
                             "rule" % (restarts + 1, 902))
    best = None
    for q in sols:
        t = scorer.arm_terms(arm, q)
        mg = min(math.pi - abs(float(q[i])) for i in CONT_IDX)
        key = (t["clearance_m"] >= floor, mg >= seam, t["clearance_m"])
        if best is None or key > best[0]:
            best = (key, q, t, mg)
    if not best[0][0]:
        return False, best[1], ("wearer clearance %.3f m, under the %.3f m "
                                "floor" % (best[2]["clearance_m"], floor))
    if not best[0][1]:
        return False, best[1], ("a continuous joint sits %.3f rad from the "
                                "+/-pi seam (limit %.2f)" % (best[3], seam))
    return True, best[1], "reachable, clearance %.3f m" % best[2]["clearance_m"]


def plan_and_check(scorer, arm, home, *args, **kw):
    """plan_grasp, then ask whether THIS arm can actually go there."""
    g = plan_grasp(*args, **kw)
    ok_p, q_p, why_p = reachable(scorer, arm, g["pregrasp"], home)
    ok_g, q_g, why_g = reachable(scorer, arm, g["centre"], home)
    g["reach"] = dict(pregrasp_ok=ok_p, pregrasp_why=why_p,
                      grasp_ok=ok_g, grasp_why=why_g,
                      q_pregrasp=None if q_p is None else [float(v) for v in q_p],
                      q_grasp=None if q_g is None else [float(v) for v in q_g])
    if not (ok_p and ok_g):
        raise PlanFailure(
            "reach",
            "the %s arm cannot reach it -- pregrasp: %s; grasp: %s"
            % (arm, why_p, why_g), g)
    return g
