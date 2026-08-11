#!/usr/bin/env python3
"""T1 cube grasping, and T2 container transfer -- measured before either is built.

T1.  The object is a CUBE.  A cube admits only FOUR grasp yaws (its 4-fold
     symmetry about the vertical); a cylinder admits all of them.  That is the
     whole of Gate 2's cube-versus-cylinder distinction, so it is measured as
     three orientation policies on the SAME poses:

       fixed      the pinned anchor, exactly.  MASTER_TELEOP has nothing else:
                  orientation_mode is `fixed` and nothing in the master
                  measures the wrist.
       cube_4yaw  best of the four yaws a cube's faces allow.  What the
                  autonomy modes can command, and what VR can be aimed at.
       free_yaw   best of eight.  A CYLINDER.  Reported only as the reference
                  the cube number is read against -- the object is NOT being
                  changed to improve the result.

T1 REGION.  Also the extent of the randomisation region, and whether four
     cubes and two coloured planes fit in it.  The graspable strip and the
     SUPPORTED strip are not the same set, and the bench edge is what
     separates them, so the bench edge is swept.

T2.  Gate 3 tested a VERTICAL STACK and found none.  A transfer may not need
     one: two containers brought rim-to-rim, or tilted toward each other, is a
     different relative pose.  So the question is generalised to the thing
     that actually matters --

         what is the SMALLEST distance between an end-effector pose the LEFT
         arm can reach and one the RIGHT arm can reach?

     If that minimum exceeds the reach of a cube crossing between two rims,
     then rim-to-rim, tilted, stacked and every other arrangement are all
     impossible at once, and it is one number rather than a list of failed
     special cases.

    python3 scripts/measure_t1_cubes_t2_transfer.py [--repeats 10]
"""
import argparse
import json
import math
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from srl_autonomy.grasp_library import _quat_from_z_and_x    # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_cubes_t2_transfer.json")

CUBE_M = 0.040
# Container: a cylinder that holds four 40 mm cubes.  75 mm outside diameter
# is the largest an 85 mm gripper can close on with any margin at all, and it
# is the figure the original cup spec used.
CONTAINER_D = 0.075
CONTAINER_H = 0.090


def yaw_k(quat, k, n):
    """The repo's own yaw-free rotation, taken one step at a time.

    Same composition as Solver.solve_yaw_free, so "yaw" means the same thing
    here as it does everywhere else -- a second definition would make the
    cube and cylinder numbers incomparable, which is the only thing this
    script is for.
    """
    yaw = -math.pi + 2.0 * math.pi * k / n
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.x = quat.x * cy - quat.y * sy
    q.y = quat.x * sy + quat.y * cy
    q.z = quat.z * cy + quat.w * sy
    q.w = quat.w * cy - quat.z * sy
    return q


def apply_furniture(n, near_y=None):
    """Bench in the planning scene, optionally with the edge moved.

    Moving BENCH_NEAR_Y is a HYPOTHETICAL: it is restored before returning,
    and nothing downstream is allowed to inherit it.  The planning scene
    lives in move_group and outlives this process, which has already bitten
    once in this project.
    """
    import clip_scene as CS
    old = CT.BENCH_NEAR_Y
    if near_y is not None:
        CT.BENCH_NEAR_Y = near_y
    try:
        tmp = CS.Scene.__new__(CS.Scene)
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = CS.Scene._collision_furniture(tmp)
    finally:
        CT.BENCH_NEAR_Y = old
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        return False
    import time as _t
    for _ in range(3):
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if fut.done():
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--quick", type=int, default=3,
                    help="repeats for the sweeps, where N=10 is unaffordable")
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s %.4f rad from home" % (arm, w))
            return 3
    anchor = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in anchor.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4
    qt = _quat_from_z_and_x([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    topdown = Quaternion(x=float(qt[0]), y=float(qt[1]), z=float(qt[2]),
                         w=float(qt[3]))

    if not apply_furniture(n):
        print("REFUSING: the bench did not apply -- a bench task verified in "
              "free space answers a different question")
        return 5
    print("bench in scene: True  (top %.2f, near edge %.3f)"
          % (CT.BENCH_TOP, CT.BENCH_NEAR_Y))

    calls = {"n": 0}

    def ok(arm, ee, q, k):
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(ee), q, tries=6):
                return False
        return True

    def any_yaw(arm, ee, base, nyaw, k):
        return any(ok(arm, ee, yaw_k(base, i, nyaw), k) for i in range(nyaw))

    res = {"cube_m": CUBE_M, "container_d": CONTAINER_D,
           "bench_top": CT.BENCH_TOP, "bench_near_y": CT.BENCH_NEAR_Y,
           "repeats": a.repeats}

    # ---------------- instrument controls -----------------------------
    ctl_good = ok("left", CT.A_PICK, anchor["left"], 1)
    ctl_far = ok("left", [1.6, 0.35, 1.15], anchor["left"], 1)
    print("CONTROLS  known-good pick -> %s;  1.6 m out -> %s"
          % ("ok" if ctl_good else "UNREACHABLE",
             "SOLVED" if ctl_far else "none"))
    if not ctl_good or ctl_far:
        print("REFUSING TO REPORT: a control failed.")
        return 6
    res["controls"] = dict(known_good=bool(ctl_good), far=not bool(ctl_far))

    # =================================================== T1: cube grasping
    print("\n" + "=" * 72)
    print("T1  grasping a %d mm CUBE, per arm, per orientation policy" %
          (CUBE_M * 1000))
    print("=" * 72)
    # Poses: the measured graspable strip for each arm, from
    # docs/system/12_randomisation_region.md -- asked where the objects can
    # actually be, not over an abstract box.
    strip = {"left": [(x, y) for x in (0.30, 0.33, 0.36, 0.39)
                      for y in (0.15, 0.19, 0.23)],
             "right": [(x, y) for x in (-0.48, -0.51, -0.54, -0.57)
                       for y in (0.15, 0.19, 0.23)]}
    zc = CT.BENCH_TOP + CUBE_M / 2.0
    t1 = {}
    for arm in ("left", "right"):
        pol = {"fixed": 0, "cube_4yaw": 0, "free_yaw": 0, "topdown_cube": 0}
        for (x, y) in strip[arm]:
            ee = CT.ee_for([x, y, zc])
            if ok(arm, ee, anchor[arm], a.repeats):
                pol["fixed"] += 1
            if any_yaw(arm, ee, anchor[arm], 4, a.quick):
                pol["cube_4yaw"] += 1
            if any_yaw(arm, ee, anchor[arm], 8, a.quick):
                pol["free_yaw"] += 1
            if any_yaw(arm, ee, topdown, 4, a.quick):
                pol["topdown_cube"] += 1
        of = len(strip[arm])
        t1[arm] = {k: dict(ok=v, of=of, pct=round(100.0 * v / of, 1))
                   for k, v in pol.items()}
        print("  %-5s " % arm + "  ".join(
            "%s %d/%d (%.0f%%)" % (k, v["ok"], v["of"], v["pct"])
            for k, v in t1[arm].items()))
    res["t1_cube_grasp"] = t1

    # ------------- T1 region extent, and the bench edge ----------------
    print("\nT1  randomisation region: graspable AND SUPPORTED, "
          "as the bench edge moves")
    print("  (supported means the cube's near face is at or behind the edge)")
    xs = {"left": [round(0.24 + 0.03 * i, 3) for i in range(9)],
          "right": [round(-0.24 - 0.03 * i, 3) for i in range(12)]}
    ys = [round(0.13 + 0.02 * i, 3) for i in range(11)]
    reach = {}
    for arm in ("left", "right"):
        for pol, q in (("fixed", anchor[arm]), ("topdown", topdown)):
            reach[(arm, pol)] = {
                (x, y): ok(arm, CT.ee_for([x, y, zc]), q, a.quick)
                for x in xs[arm] for y in ys}
    edges = [CT.BENCH_NEAR_Y, 0.21, 0.19, 0.17, 0.15]
    region = {}
    for e in edges:
        row = {}
        for arm in ("left", "right"):
            for pol in ("fixed", "topdown"):
                cells = [(x, y) for (x, y), v in reach[(arm, pol)].items()
                         if v and (y - CUBE_M / 2.0) >= e]
                if cells:
                    xr = (min(abs(c[0]) for c in cells),
                          max(abs(c[0]) for c in cells))
                    yr = (min(c[1] for c in cells), max(c[1] for c in cells))
                    row["%s_%s" % (arm, pol)] = dict(
                        cells=len(cells),
                        x_mm=[round(1000 * v) for v in xr],
                        y_mm=[round(1000 * v) for v in yr],
                        width_mm=round(1000 * (xr[1] - xr[0])),
                        depth_mm=round(1000 * (yr[1] - yr[0])))
                else:
                    row["%s_%s" % (arm, pol)] = dict(cells=0)
        region["edge_%.3f" % e] = row
        print("  edge y=%.3f  " % e + "  ".join(
            "%s:%s" % (k.replace("_", "/"),
                       ("%dc %dx%d mm" % (v["cells"], v["width_mm"],
                                          v["depth_mm"]))
                       if v["cells"] else "NONE")
            for k, v in row.items()))
    res["t1_region_vs_bench_edge"] = region

    # =================================================== T2: transfer
    print("\n" + "=" * 72)
    print("T2  container transfer -- the SMALLEST achievable inter-arm "
          "EE distance")
    print("=" * 72)
    # Generalises Gate 3.  If the closest the two end effectors can get
    # exceeds what a cube crossing two rims needs, then rim-to-rim, tilted
    # and stacked are all impossible together, in one number.
    gx = [round(-0.60 + 0.04 * i, 3) for i in range(31)]
    gy = [0.13, 0.17, 0.21, 0.25, 0.29, 0.33]
    gz = [round(1.06 + 0.04 * i, 3) for i in range(8)]
    tbl = {}
    for arm in ("left", "right"):
        for pol, q in (("fixed", anchor[arm]), ("topdown", topdown)):
            pts = [(x, y, z) for x in gx for y in gy for z in gz
                   if ok(arm, [x, y, z], q, a.quick)]
            tbl[(arm, pol)] = pts
            print("  %-5s %-8s reachable %4d of %d"
                  % (arm, pol, len(pts), len(gx) * len(gy) * len(gz)))
    if not all(tbl.values()):
        empty = [k for k, v in tbl.items() if not v]
        print("  REFUSING: %s reachable nowhere in the grid -- a minimum "
              "distance from an empty set is not a measurement." % (empty,))
        res["t2"] = {"verdict": "INVALID -- empty reachable set: %s"
                                % [list(k) for k in empty]}
    else:
        best = {}
        for lp in ("fixed", "topdown"):
            for rp in ("fixed", "topdown"):
                pairs = 0
                dmin, arg = float("inf"), None
                for p in tbl[("left", lp)]:
                    for q2 in tbl[("right", rp)]:
                        pairs += 1
                        d = math.dist(p, q2)
                        if d < dmin:
                            dmin, arg = d, (p, q2)
                best["left_%s__right_%s" % (lp, rp)] = dict(
                    pairs_tested=pairs, min_distance_m=round(dmin, 4),
                    closest=[list(arg[0]), list(arg[1])])
                print("  left %-8s / right %-8s : %d pairs, MINIMUM "
                      "separation %.3f m" % (lp, rp, pairs, dmin))
        overall = min(v["min_distance_m"] for v in best.values())
        # What a transfer actually needs: the two rims adjacent, i.e. the two
        # EE poses within about one container diameter plus a cube.
        need = CONTAINER_D + CUBE_M
        res["t2"] = dict(policies=best, min_separation_m=overall,
                         transfer_needs_m=round(need, 4),
                         verdict=("POSSIBLE" if overall <= need
                                  else "IMPOSSIBLE"))
        print("\n  smallest inter-arm EE separation anywhere: %.3f m" % overall)
        print("  a rim-to-rim transfer needs about %.3f m "
              "(container %.0f mm + cube %.0f mm)"
              % (need, CONTAINER_D * 1000, CUBE_M * 1000))
        print("  -> %s, by a factor of %.1f"
              % (res["t2"]["verdict"], overall / need))

    import clip_scene as CS
    CS.remove_furniture(n)
    res["ik_calls"] = calls["n"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n%d IK calls -> %s" % (calls["n"], OUT))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
