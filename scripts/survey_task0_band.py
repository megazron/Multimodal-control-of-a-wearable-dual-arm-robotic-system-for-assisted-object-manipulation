#!/usr/bin/env python3
"""Measure the band Task 0's spheres can live in, WITH THE BENCH IN SCENE.

Written because the first attempt at Task 0 placed spheres from z = 1.03, and
the bench slab occupies z = 1.06..1.10 (BENCH_TOP 1.10, BENCH_THICK 0.04).
Sphere A was inside the furniture and the verifier correctly refused.

That the existing Task A declares targets at z = 1.05 -- also under the bench
top -- and passes is not a contradiction: verify_abc_scenarios checks the
study tasks in FREE SPACE and only applies furniture for the clip tasks. So
the band has never actually been measured against the bench, and guessing it a
second time would repeat the mistake with different numbers.

    python3 scripts/survey_task0_band.py [--repeats 3]
"""
import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/task0_band_survey.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--y", type=float, default=0.35)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home" % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    import clip_scene as CS
    tmp = CS.Scene.__new__(CS.Scene)
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(tmp)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("REFUSING: no /apply_planning_scene")
        return 5
    import time as _t
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + 15.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    print("bench applied: %s   (BENCH_TOP=%.2f, slab %.2f..%.2f)"
          % (fut.done(), CT.BENCH_TOP, CT.BENCH_TOP - CT.BENCH_THICK,
             CT.BENCH_TOP))

    def ok(arm, p, k):
        for _ in range(k):
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    xs = [round(0.26 + 0.02 * i, 3) for i in range(14)]      # 0.26 .. 0.52
    zs = [round(1.04 + 0.03 * i, 3) for i in range(15)]      # 1.04 .. 1.46

    # BOTH ARMS, and the answer is the INTERSECTION at mirrored x.  The two
    # arms are parked asymmetrically -- docs/ENGINEERING_LOG.md records the residual as
    # 1.3837 m and proves it independent of the mount -- so the right arm's
    # band is NOT the mirror of the left's, and a set derived from the left
    # alone puts a sphere the right arm cannot reach.  Measured: it did
    # exactly that, right sphere A at (-0.31, 0.35, 1.24), 9 waypoint
    # failures.  The sets must stay mirrored to compare the arms fairly, so
    # the band has to be common ground, not one arm's.
    grid = {}
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        print("\n%s ARM, y = %.2f, k = %d, bench IN scene"
              % (arm.upper(), a.y, a.repeats))
        print("      " + "".join("%6.2f" % x for x in xs))
        for z in zs:
            row = ""
            for x in xs:
                good = ok(arm, [sgn * x, a.y, z], a.repeats)
                grid.setdefault("%.3f,%.3f" % (x, z), {})[arm] = good
                row += "     %s" % ("#" if good else ".")
            print("%5.2f %s" % (z, row))

    both = [[x, a.y, z] for z in zs for x in xs
            if grid["%.3f,%.3f" % (x, z)]["left"]
            and grid["%.3f,%.3f" % (x, z)]["right"]]
    print("\nBOTH ARMS (at mirrored x)")
    print("      " + "".join("%6.2f" % x for x in xs))
    for z in zs:
        print("%5.2f %s" % (z, "".join(
            "     %s" % ("#" if grid["%.3f,%.3f" % (x, z)]["left"]
                         and grid["%.3f,%.3f" % (x, z)]["right"] else ".")
            for x in xs)))
    if both:
        zf = [p[2] for p in both]
        xf = [p[0] for p in both]
        print("\n  %d of %d cells feasible for BOTH  |  z %.2f..%.2f  "
              "x %.2f..%.2f" % (len(both), len(xs) * len(zs),
                                min(zf), max(zf), min(xf), max(xf)))
    else:
        print("\n  NO cell is feasible for both arms at y = %.2f." % a.y)
    feasible = both

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(dict(y=a.y, repeats=a.repeats, xs=xs, zs=zs, grid=grid,
                   feasible_both=feasible, bench_top=CT.BENCH_TOP,
                   bench_thick=CT.BENCH_THICK), open(OUT, "w"), indent=2)
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
