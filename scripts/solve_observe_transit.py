#!/usr/bin/env python3
"""A CLEAN WAY TO GET TO THE OBSERVE POSE, since a straight line is not one.

    python3 scripts/solve_observe_transit.py --arm left --pose t1_left

WHY. `solve_observe_pose.py` finds a pose whose camera sees the work and which
is itself clear of the wearer. Verified on 2026-08-16, the POSE is fine on both
arms -- commandable by `/compute_ik` 10 of 10, clearance 0.1610 m -- and the
STRAIGHT JOINT-SPACE MOVE FROM HOME IS NOT: it dips to 0.1199 m (left) and
0.1264 (right) against a 0.150 m floor, roughly halfway along.

Both endpoints being clear says nothing about the line between them. A joint
interpolation is a straight line in JOINT space, not in the world, and the
elbow sweeps a long arc through the volume the wearer occupies.

Re-solving the POSE with the straight-line transit as a constraint returns
NOTHING, for any of the three target sets. That is not a proof that the arm
cannot get there; it is a proof that it cannot get there IN ONE STRAIGHT LINE.
So this searches for a single VIA configuration such that home -> via and
via -> observe are each clean, which is the smallest thing that can work and
is directly executable: two trajectory segments, no planner required.

WHY NOT JUST ASK MOVEIT TO PLAN IT. Because MoveIt cannot see the wearer where
it matters. The SRDF permanently excludes torso/harness/backpack against each
arm's base, shoulder and half_arm_1 -- exactly the pairs a shoulder mount
threatens -- so a planned path can be returned `valid` with the tube inside the
person. docs/ENGINEERING_LOG.md hard constraint 11. The wearer check here is geometric, with
the mount guard's own capsule model.

CONTROLS, and there is no report without them:

    the straight line still fails    if it passes, the premise is gone and the
                                     via point is unnecessary
    a via INSIDE the wearer is       so the segment check can fail
    rejected
    both segments are checked at a   the reported clearance is the worst over
    fine step                        both, not of the endpoints
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

import solve_home_pose as SH                                     # noqa: E402
import home_positions as hp                                      # noqa: E402

FLOOR = 0.15
OUT = os.path.join(ROOT, "recordings/baselines/observe_transit.json")


def seg_worst(sc, arm, a, b, n):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    return min(sc.clearance(arm, a + d * (i / float(n - 1)))[0]
               for i in range(n))


def path_worst(sc, arm, qs, step=0.05):
    """Worst clearance over a multi-segment joint path, at a fine step."""
    worst = 1e9
    for a, b in zip(qs, qs[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(math.ceil(float(np.max(np.abs(b - a))) / step)) + 1)
        worst = min(worst, seg_worst(sc, arm, a, b, n))
    return worst


def cost(v, sc, arm, q_home, q_obs, floor, n=10):
    hinge = lambda x: x * x if x > 0 else 0.0                     # noqa: E731
    m = floor + 0.012
    c = 0.0
    c += 400.0 * hinge(m - seg_worst(sc, arm, q_home, v, n))
    c += 400.0 * hinge(m - seg_worst(sc, arm, v, q_obs, n))
    for i in SH.CONTINUOUS_IDX:
        c += 20.0 * hinge(SH.SEAM_MARGIN_RAD - (math.pi - abs(float(v[i]))))
    # prefer a SHORT detour: the via should be near the straight line
    mid = (np.asarray(q_home, float) + np.asarray(q_obs, float)) * 0.5
    c += 0.02 * float(np.sum((np.asarray(v) - mid) ** 2))
    return c


def solve_via(sc, arm, q_home, q_obs, floor=FLOOR, restarts=120, seed=5):
    from scipy.optimize import minimize
    rng = np.random.default_rng(seed)
    lo, hi, _ = sc.lim[arm]
    bounds = list(zip(lo, hi))
    mid = (q_home + q_obs) * 0.5
    seeds = [mid, q_home.copy(), q_obs.copy()]
    seeds += [np.clip(mid + rng.normal(0, 0.6, 7), lo, hi)
              for _ in range(restarts)]
    best = None
    for s in seeds:
        s = np.clip(np.asarray(s, float), lo, hi)
        try:
            r = minimize(cost, s, args=(sc, arm, q_home, q_obs, floor),
                         method="L-BFGS-B", bounds=bounds,
                         options=dict(maxiter=220, ftol=1e-12, eps=1e-6))
        except Exception:                                         # noqa: BLE001
            continue
        v = np.array(r.x)
        w = path_worst(sc, arm, [q_home, v, q_obs], 0.05)
        seam = min(math.pi - abs(float(v[i])) for i in SH.CONTINUOUS_IDX)
        if w < floor or seam < SH.SEAM_MARGIN_RAD:
            continue
        travel = float(np.sum(np.abs(v - q_home)) + np.sum(np.abs(q_obs - v)))
        if best is None or travel < best[0]:
            best = (travel, v, w)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", required=True,
                    help="observe_<pose>.json in recordings/baselines")
    ap.add_argument("--arm", required=True, choices=["left", "right"])
    ap.add_argument("--floor", type=float, default=FLOOR)
    ap.add_argument("--restarts", type=int, default=120)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    f = os.path.join(ROOT, "recordings/baselines/observe_%s.json" % a.pose)
    if not os.path.exists(f):
        print("no %s" % f)
        return 2
    d = json.load(open(f))
    if not d.get("solved"):
        print("%s records NO SOLUTION" % a.pose)
        return 3
    q_obs = np.array(d["solved"]["q"], float)
    q_home = np.array(hp.load_home_radians(a.arm), float)
    sc = SH.Scorer()

    straight = path_worst(sc, a.arm, [q_home, q_obs], 0.05)
    print("CONTROLS")
    print("   straight-line transit worst clearance   %.4f m -> %s"
          % (straight, "still fails, as expected"
             if straight < a.floor else "PASSES, via point unnecessary"))
    inside = q_home.copy()
    inside[1] += 1.2
    bad = path_worst(sc, a.arm, [q_home, inside], 0.05)
    print("   a via swung into the wearer scores      %.4f m -> %s"
          % (bad, "rejected" if bad < a.floor else "NOT REJECTED"))
    if bad >= a.floor:
        print("REFUSING: the segment check cannot fail.")
        return 6

    got = solve_via(sc, a.arm, q_home, q_obs, a.floor, a.restarts)
    if got is None:
        print("\nNO SINGLE VIA CONFIGURATION makes both segments clean for "
              "the %s arm." % a.arm)
        json.dump(dict(pose=a.pose, arm=a.arm, straight_worst_m=straight,
                       via=None), open(a.out, "w"), indent=2)
        return 5
    travel, v, worst = got
    dj1 = np.abs(v - q_home)
    dj2 = np.abs(q_obs - v)
    print("\nVIA FOUND for the %s arm" % a.arm)
    print("   worst clearance over BOTH segments      %.4f m (floor %.3f)"
          % (worst, a.floor))
    print("   home -> via   largest joint %.1f deg, total %.1f deg"
          % (math.degrees(dj1.max()), math.degrees(dj1.sum())))
    print("   via -> observe largest joint %.1f deg, total %.1f deg"
          % (math.degrees(dj2.max()), math.degrees(dj2.sum())))
    print("   via, ROS deg  %s"
          % " ".join("%+.2f" % math.degrees(x) for x in v))
    json.dump(dict(pose=a.pose, arm=a.arm, straight_worst_m=round(straight, 4),
                   via=[float(x) for x in v],
                   via_deg=[math.degrees(float(x)) for x in v],
                   worst_clearance_m=round(float(worst), 4),
                   total_travel_deg=round(math.degrees(travel), 2)),
              open(a.out, "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
