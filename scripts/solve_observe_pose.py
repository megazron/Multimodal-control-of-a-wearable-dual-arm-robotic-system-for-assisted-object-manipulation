#!/usr/bin/env python3
"""WHERE MUST THE ARM STAND SO THE WRIST CAMERA CAN SEE THE WORK?

    python3 scripts/solve_observe_pose.py --arm left
    python3 scripts/solve_observe_pose.py --self-test

WHY THIS EXISTS. Every grasp in this repository is computed from a coordinate
written into the task file. To compute it from what the camera SAW instead,
there has to be a pose the camera can see the work from -- and there isn't one
today, because nothing has ever asked for it.

THE REACH IS NOT IT, AND THAT IS MEASURED, NOT ASSUMED. The wrist camera's
optical axis IS the tool axis: `camera_color_frame` sits at rpy (pi, pi, 0)
from `end_effector_link`, which is diag(-1, -1, 1), so camera +z = EE +z to
0.00 deg. The camera looks exactly where the gripper points. But T1's path
descends VERTICALLY from a 0.10 m standoff while the pinned anchor points
30.8 deg (left) / 22.1 (right) ABOVE horizontal, so during the approach the
object sits 65.8 / 62.5 deg off the optical axis -- outside a 55 deg
horizontal field of view -- and 0.128 to 0.167 m away, inside the 0.25 m
minimum range of the depth module the Gen3 vision head carries. **The object
is never simultaneously in frame and in depth range during a reach.**

So detection has to happen from somewhere else, and this file solves for that
somewhere: a pose with every object inside the frustum, far enough out for
depth to be valid, and clear of the wearer.

WHAT IS SOLVED. Seven joints per arm, against:

    (i)   every target point inside the image, with a margin, using the
          mock camera's OWN intrinsics rather than a nominal field of view
    (ii)  every target at least `--min-range` from the camera
    (iii) the wearer clearance floor, geometric, the mount guard's model
    (iv)  joint limits, and continuous joints clear of the +/-pi seam

It does NOT constrain the wrist to be level or the camera to be on top: this
is a LOOKING pose, not a resting one, and the two want different things. The
grasp orientation stays the pinned anchor, which is what `run_abc` commands.

CONTROLS, and there is no report without them:

    the FK reproduces live TF          srl_fk.self_test
    a target BEHIND the camera fails   a point at the camera's back must never
                                       project into the image
    an impossible field of view fails  demanding a 1 deg half-angle must find
                                       nothing, so "solved" cannot be vacuous
    the projection matches the mock    same intrinsics, same optical
                                       convention, checked against a point
                                       whose pixel is computed by hand
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from srl_fk import FK, CompiledFK, self_test as fk_self_test        # noqa: E402
from srl_teleop import mount_guard_node as MG                       # noqa: E402
import solve_home_pose as SH                                        # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/observe_pose.json")

# THE MOCK CAMERA'S OWN INTRINSICS, imported in spirit rather than guessed.
# mock_rgbd_camera.py: W, H = 640, 480 and FX = FY = 615.0, which is a 55.0 deg
# horizontal and 42.6 deg vertical field of view. A solver that used a nominal
# 60 deg would place poses the renderer then crops.
W, H = 640, 480
FX = FY = 615.0
CX, CY = W / 2.0 - 0.5, H / 2.0 - 0.5
CAM_FRAME = "camera_color_frame"
FLOOR = 0.15
MIN_RANGE = 0.30          # the depth module is unusable below 0.25; 50 mm spare


def fov_deg():
    return (2 * math.degrees(math.atan(CX / FX)),
            2 * math.degrees(math.atan(CY / FY)))


class Looker:
    """Projection of world points through the wrist camera, for one arm."""

    def __init__(self, sc=None):
        self.sc = sc or SH.Scorer()
        self.fk = self.sc.fk
        self.cam = {a: CompiledFK(self.fk, a, [CAM_FRAME])
                    for a in ("left", "right")}

    def camera(self, arm, q):
        """(position, R_world_from_camera) of the colour optical frame."""
        M = self.cam[arm](q)[0]
        return M[:3, 3], M[:3, :3]

    def project(self, arm, q, pts):
        """Pixel (u, v) and range for each world point. z<=0 means behind.

        Optical convention, which is what the real driver and the mock both
        use: camera +z forward, +x right, +y down.
        """
        p, R = self.camera(arm, q)
        out = []
        for w in pts:
            c = R.T @ (np.asarray(w, float) - p)
            if c[2] <= 1e-6:
                out.append((None, None, float(c[2])))
                continue
            out.append((FX * c[0] / c[2] + CX, FY * c[1] / c[2] + CY,
                        float(np.linalg.norm(c))))
        return out

    def terms(self, arm, q, pts, margin=0.90, min_range=MIN_RANGE):
        """How badly this pose fails to see `pts`. Zero when it sees them."""
        pr = self.project(arm, q, pts)
        worst_u = worst_v = 0.0
        behind = 0
        near = 0.0
        rngs = []
        for u, v, r in pr:
            if u is None:
                behind += 1
                continue
            rngs.append(r)
            worst_u = max(worst_u, abs(u - CX) / (CX * margin))
            worst_v = max(worst_v, abs(v - CY) / (CY * margin))
            near = max(near, min_range - r)
        c, who = self.sc.clearance(arm, q)
        return dict(behind=behind, worst_u=worst_u, worst_v=worst_v,
                    min_range_m=min(rngs) if rngs else None,
                    max_range_m=max(rngs) if rngs else None,
                    range_shortfall_m=max(0.0, near),
                    clearance_m=c, clearance_to=who,
                    all_in_frame=(behind == 0 and worst_u <= 1.0
                                  and worst_v <= 1.0),
                    pixels=[(None if u is None else round(u, 1),
                             None if v is None else round(v, 1),
                             round(r, 4)) for u, v, r in pr])


def transit_samples(q_home, q, n=12):
    """Joint-space straight line home -> q, as n configurations."""
    a = np.asarray(q_home, float)
    d = np.asarray(q, float) - a
    return [a + d * (i / float(n - 1)) for i in range(n)]


def transit_worst(lk, arm, q_home, q, n=12):
    """Worst wearer clearance anywhere on the move from home."""
    return min(lk.sc.clearance(arm, s)[0]
               for s in transit_samples(q_home, q, n))


def cost(q, lk, arm, pts, margin, min_range, floor, q_home=None):
    hinge = lambda v: v * v if v > 0 else 0.0                     # noqa: E731
    t = lk.terms(arm, q, pts, margin, min_range)
    c = 40.0 * t["behind"]
    c += 8.0 * hinge(t["worst_u"] - 1.0) + 8.0 * hinge(t["worst_v"] - 1.0)
    c += 60.0 * hinge(t["range_shortfall_m"])
    c += 400.0 * hinge(floor - t["clearance_m"])
    # THE MOVE FROM HOME IS PART OF THE POSE, and leaving it out is what made
    # the first two observe poses unusable. Solved on frustum geometry alone
    # the search put the camera at z = 1.78 -- above the wearer's head -- and
    # the arm swung THROUGH the person to get there: 0.1199 m (left) and
    # 0.1264 (right) against a 0.150 floor, 170-184 deg on joint_5, and 1.3-1.5
    # m of end-effector path. The pose was clean; the journey was not.
    if q_home is not None:
        c += 600.0 * hinge(floor + 0.012 - transit_worst(lk, arm, q_home, q))
        # and prefer a SHORT move, an order of magnitude below the constraints
        c += 0.05 * float(np.sum((np.asarray(q) - np.asarray(q_home)) ** 2))
    for i in SH.CONTINUOUS_IDX:
        c += 20.0 * hinge(SH.SEAM_MARGIN_RAD - (math.pi - abs(float(q[i]))))
    # preferences, well below the constraints: look from as close to
    # straight-on as possible, and do not wander further away than needed
    c += 0.02 * (t["worst_u"] ** 2 + t["worst_v"] ** 2)
    if t["max_range_m"]:
        c += 0.05 * max(0.0, t["max_range_m"] - min_range - 0.25) ** 2
    return c


def solve(lk, arm, pts, margin=0.90, min_range=MIN_RANGE, floor=FLOOR,
          restarts=160, seed=3, q_home=None):
    from scipy.optimize import minimize
    rng = np.random.default_rng(seed)
    lo, hi, _ = lk.sc.lim[arm]
    bounds = list(zip(lo, hi))
    seeds = SH.seed_set(lk.sc, arm, restarts, rng)
    best = None
    for s in seeds:
        s = np.clip(np.asarray(s, float), lo, hi)
        try:
            r = minimize(cost, s, args=(lk, arm, pts, margin, min_range,
                                        floor, q_home),
                         method="L-BFGS-B", bounds=bounds,
                         options=dict(maxiter=300, ftol=1e-12, eps=1e-6))
        except Exception:                                         # noqa: BLE001
            continue
        q = np.array(r.x)
        v = float(cost(q, lk, arm, pts, margin, min_range, floor, q_home))
        t = lk.terms(arm, q, pts, margin, min_range)
        ok = (t["all_in_frame"] and t["range_shortfall_m"] <= 0
              and t["clearance_m"] >= floor)
        seam = min(math.pi - abs(float(q[i])) for i in SH.CONTINUOUS_IDX)
        if not ok or seam < SH.SEAM_MARGIN_RAD:
            continue
        if q_home is not None:
            # checked FINELY here, coarsely inside the cost
            if transit_worst(lk, arm, q_home, q, 40) < floor:
                continue
        if best is None or v < best[0]:
            best = (v, q, t, seam)
    return best


def self_test(lk, verbose=True):
    """Can this file fail? Three ways it must."""
    ok = True
    arm = "left"
    q = np.array(SH.Scorer().fk.limits(arm)[0]) * 0.0
    p, R = lk.camera(arm, q)
    # 1. a point behind the camera must not project
    back = p - R[:, 2] * 0.5
    u, v, z = lk.project(arm, q, [back])[0]
    good = u is None
    ok = ok and good
    if verbose:
        print("   a point BEHIND the camera does not project        %s"
              % ("PASS" if good else "FAIL"))
    # 2. the projection matches a hand-computed pixel
    fwd = p + R[:, 2] * 1.0 + R[:, 0] * 0.10          # 1 m out, 0.1 m right
    u, v, r = lk.project(arm, q, [fwd])[0]
    want_u = FX * 0.10 / 1.0 + CX
    good = abs(u - want_u) < 1e-6 and abs(v - CY) < 1e-6
    ok = ok and good
    if verbose:
        print("   projection matches a hand-computed pixel          %s "
              "(u %.2f want %.2f)" % ("PASS" if good else "FAIL", u, want_u))
    # 3. an impossible field of view finds nothing
    pts = [[0.30, 0.30, 1.00], [-0.30, 0.30, 1.00]]
    got = solve(lk, arm, pts, margin=0.02, restarts=25)
    good = got is None
    ok = ok and good
    if verbose:
        print("   an impossible 1 deg field of view finds nothing   %s"
              % ("PASS" if good else "FAIL"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=["left", "right"])
    ap.add_argument("--points", default=None,
                    help="JSON list of [x,y,z] to see; default = T1's cubes "
                         "and pads for this arm")
    ap.add_argument("--margin", type=float, default=0.90)
    ap.add_argument("--min-range", type=float, default=MIN_RANGE)
    ap.add_argument("--floor", type=float, default=FLOOR)
    ap.add_argument("--restarts", type=int, default=160)
    ap.add_argument("--ignore-transit", action="store_true",
                    help="solve the POSE only, without requiring the straight "
                         "joint-space move from home to clear the wearer. Use "
                         "with scripts/solve_observe_transit.py, which then "
                         "finds a via point for the move.")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    print("INSTRUMENT CHECKS")
    ok1, worst1, _ = fk_self_test(verbose=False)
    print("   FK vs live TF                                     %.6f m -> %s"
          % (worst1, "PASS" if ok1 else "FAIL"))
    lk = Looker()
    if not self_test(lk) or not ok1:
        print("REFUSING: a control failed.")
        return 6
    hf, vf = fov_deg()
    print("   camera %d x %d, f = %.0f -> %.1f deg H x %.1f deg V"
          % (W, H, FX, hf, vf))

    if a.points:
        pts = json.loads(a.points)
    else:
        sys.path.insert(0, os.path.join(
            ROOT, "src/srl_experiments/experiments/abc"))
        # T1'S OWN HEIGHTS, from the module that owns them. They used to come
        # from `clip_tasks.BENCH_TOP`, the plane every OTHER task works on;
        # after the 2026-08-17 rebuild T1's objects rest on the table 120 mm
        # below it, and an observe pose solved to see six points that are not
        # there is a pose that verifies perfectly and sees nothing.
        import t1_task as T1M
        pts = [[px, py, T1M.T1_Z] for px, py in T1M.T1_CUBES]
        pts += [[px, py, T1M.TABLE_TOP + T1M.PLANE_T]
                for px, py in T1M.T1_PLANES]
    print("\n%d points to see:" % len(pts))
    for p in pts:
        print("   %s" % [round(v, 3) for v in p])

    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as _hp
    q_home = np.array(_hp.load_home_radians(a.arm), float)
    got = solve(lk, a.arm, pts, a.margin, a.min_range, a.floor, a.restarts,
                q_home=None if a.ignore_transit else q_home)
    if got is None:
        print("\nNO OBSERVE POSE satisfies every requirement for the %s arm."
              % a.arm)
        json.dump(dict(arm=a.arm, points=pts, solved=None),
                  open(a.out, "w"), indent=2)
        return 5
    v, q, t, seam = got
    print("\nOBSERVE POSE, %s arm" % a.arm)
    print("   all targets in frame      %s" % t["all_in_frame"])
    print("   worst |u| / half-width    %.3f  (1.0 = the frame edge x %.2f)"
          % (t["worst_u"], a.margin))
    print("   worst |v| / half-height   %.3f" % t["worst_v"])
    print("   range to targets          %.3f .. %.3f m (min allowed %.2f)"
          % (t["min_range_m"], t["max_range_m"], a.min_range))
    print("   wearer clearance          %.4f m to %s (floor %.3f)"
          % (t["clearance_m"], t["clearance_to"], a.floor))
    print("   seam margin               %.3f rad" % seam)
    _tw = transit_worst(lk, a.arm, q_home, q, 40)
    _dj = np.abs(np.asarray(q) - q_home)
    print("   transit from home         worst clearance %.4f m, largest joint "
          "%.1f deg, total %.1f deg"
          % (_tw, math.degrees(_dj.max()), math.degrees(_dj.sum())))
    print("   joints, ROS deg           %s"
          % " ".join("%+.2f" % math.degrees(x) for x in q))
    print("   pixels (u, v, range)")
    for p, px in zip(pts, t["pixels"]):
        print("      %-28s -> %s" % ([round(c, 3) for c in p], px))
    json.dump(dict(arm=a.arm, points=pts, margin=a.margin,
                   min_range_m=a.min_range, floor_m=a.floor,
                   fov_deg=[hf, vf], intrinsics=dict(W=W, H=H, FX=FX, FY=FY,
                                                     CX=CX, CY=CY),
                   solved=dict(q=[float(x) for x in q],
                               q_deg=[math.degrees(float(x)) for x in q],
                               seam_margin_rad=seam, **{
                                   k: t[k] for k in
                                   ("all_in_frame", "worst_u", "worst_v",
                                    "min_range_m", "max_range_m",
                                    "clearance_m", "clearance_to",
                                    "pixels")})),
              open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
