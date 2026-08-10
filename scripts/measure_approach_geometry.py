#!/usr/bin/env python3
"""What approach direction does the PINNED wrist actually give, and where can it go?

TWO QUESTIONS, ONE HARNESS, because the answers are coupled.

1. THE APPROACH DIRECTION IS NOT CHOSEN -- IT IS INHERITED. `orientation_mode`
   is `fixed`, so the commanded wrist quaternion is the anchor, which is the
   arm's own orientation at home. Nothing about the object's shape or pose
   enters into it. This measures the tool axis that convention actually
   produces at each task pose, in world, and reports the angle from
   straight-down. A top-down grasp needs that angle near 180 deg; a side
   grasp needs it near 90 deg.

2. WHERE CAN THE HAND GO? Objects are to move to the bench EDGE so the hand
   can come in from the side, and the reachable fore/aft band on this rig is
   narrow and has surprised the project before (y = 0.45 measured 1 of 6
   reachable while y = 0.35 was fine). So the y range is measured at N=10
   over the full path rather than assumed, and the edge is placed inside
   whatever comes back.

    python3 scripts/measure_approach_geometry.py [--repeats 10]
"""
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
import clip_tasks as CT                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/approach_geometry.json")


def axis_from_quat(q, v=(0.0, 0.0, 1.0)):
    """Rotate a body-frame vector by quaternion q -> world."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    # standard quaternion rotation
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    a = ap.parse_args()
    N = a.repeats

    import rclpy
    if not rclpy.ok():
        rclpy.init()
    n = Solver()
    n.spin(4.0)
    bad = False
    for arm in ("left", "right"):
        w, _j = n.home_ok(arm)
        print("  %-5s home offset %.4f rad" % (arm, w))
        bad |= w > HOME_TOL_RAD
    if bad:
        print("\n  REFUSING: arms not at home.")
        return 3

    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("\n  REFUSING: no tf2 for an end effector.")
        return 4

    res = {}

    # ---------------------------------------------------- 1. the tool axis
    print("\nTHE APPROACH DIRECTION THE PINNED WRIST GIVES")
    print("  orientation_mode is `fixed`, so this is the anchor quaternion --")
    print("  the arm's own orientation at home. The object plays no part.")
    print()
    print("  %-6s %-34s %8s %10s" % ("arm", "tool axis (world)", "from -z",
                                     "verdict"))
    down = (0.0, 0.0, -1.0)
    for arm in ("left", "right"):
        ax = axis_from_quat(quat[arm])
        nrm = math.sqrt(sum(c * c for c in ax))
        ax = tuple(c / nrm for c in ax)
        dot = sum(c * d for c, d in zip(ax, down))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        # 0 deg = straight down (top-down grasp), 90 = horizontal (side)
        verdict = ("TOP-DOWN" if ang < 35 else
                   "SIDE" if ang < 125 else "BOTTOM-UP")
        res.setdefault("tool_axis", {})[arm] = dict(
            axis=[round(c, 4) for c in ax], deg_from_down=round(ang, 1),
            verdict=verdict)
        print("  %-6s (%+.3f, %+.3f, %+.3f)%14s %7.1f deg  %s"
              % (arm, ax[0], ax[1], ax[2], "", ang, verdict))
    print()
    print("  A TOP-DOWN GRASP WOULD NEED THE WRIST ROTATED BY THE ANGLE ABOVE.")
    print("  With orientation_mode `fixed` no mode can command that, and the")
    print("  master arm cannot express it at all: j5 and j7 are dead on the")
    print("  right arm and j6 is clamped on the left, so wrist roll is")
    print("  unobservable. This is why the objects go to the bench EDGE.")

    calls = {"n": 0}

    def ok(arm, p, k=None):
        k = N if k is None else k
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    # controls
    ctl_far = ok("left", [1.60, 0.35, 1.15], k=1)
    ctl_good = ok("left", CT.A_PICK, k=1)
    print("\n  control unreachable -> %s (want False);  A_PICK -> %s (want True)"
          % (ctl_far, ctl_good))
    if ctl_far or not ctl_good:
        print("  REFUSING: controls misbehaved.")
        return 5

    # ------------------------------------------------ 2. the reachable band
    print("\nREACHABLE FORE/AFT BAND at the working height, N=%d per point" % N)
    print("  %-6s %6s  %s" % ("arm", "y", "verdict"))
    band = {}
    for arm, x in (("left", 0.32), ("right", -0.32)):
        good = []
        for i in range(19):
            y = round(0.10 + i * 0.025, 3)
            if ok(arm, [x, y, 1.15]):
                good.append(y)
        band[arm] = good
        if good:
            print("  %-6s %.3f .. %.3f   (%d of 19 sampled)"
                  % (arm, min(good), max(good), len(good)))
        else:
            print("  %-6s NONE reachable" % arm)
    res["y_band"] = band

    both = sorted(set(band["left"]) & set(band["right"]))
    res["y_band_both"] = both
    if both:
        print("\n  BOTH arms: y = %.3f .. %.3f" % (min(both), max(both)))
        print("  The near edge of the bench must sit at or inside y = %.3f"
              % min(both))
    res["ik_calls"] = calls["n"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n  %d IK calls  -> %s" % (calls["n"], OUT))
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                        # noqa: BLE001
            pass
    sys.exit(rc)
