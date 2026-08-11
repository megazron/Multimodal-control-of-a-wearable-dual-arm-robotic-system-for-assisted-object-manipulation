#!/usr/bin/env python3
"""GATE 3, redone -- and GATE 2's right-arm question, which the first run
asked in the wrong place.

WHY THIS EXISTS.  The first Gate 3 sweep put its candidate stacks at y = 0.30
to 0.40 and z = 1.10 to 1.25, and reported that no cup-over-cup pair exists
anywhere.  Its own control then failed: neither arm could reach ANYTHING in
that band on its own, so the zero was the sweep's, not the robot's.  The
anchor-orientation graspable strip is at y = 0.13-0.23 (measured, see
docs/system/12_randomisation_region.md) -- the sweep was looking where nothing
is reachable by anybody.

METHOD.  Build a reachability TABLE over (x, y, z) once, per arm and per
orientation policy, then read stacks out of it by lookup.  That is cheaper
than testing pairs directly and it makes the control free: the same table
says how many poses each arm can reach ALONE, so a zero for pairs is only
reported when the solo counts are non-zero.

    python3 scripts/gate3_cup_stack.py [--repeats 3]
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from srl_autonomy.grasp_library import _quat_from_z_and_x    # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/gate3_cup_stack.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(4.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s %.4f rad from home" % (arm, w))
            return 3
    anchor = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    qt = _quat_from_z_and_x([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    topdown = Quaternion(x=float(qt[0]), y=float(qt[1]), z=float(qt[2]),
                         w=float(qt[3]))

    import clip_scene as CS
    tmp = CS.Scene.__new__(CS.Scene)
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(tmp)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)
    import time as _t
    bench_in = False
    for _ in range(3):                       # retry: one timeout is not proof
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if fut.done():
            bench_in = True
            break
    print("bench in scene: %s" % bench_in)
    if not bench_in:
        print("  NOTE: running WITHOUT the bench.  That is MORE permissive, "
              "so a zero here is conservative -- but a non-zero would have "
              "to be re-run with the bench before it could be believed.")

    calls = {"n": 0}

    def ok(arm, p, q, k):
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(p), q, tries=6):
                return False
        return True

    # The grid.  x spans the whole frontal width because a stack needs BOTH
    # arms at the same x; y covers the measured anchor strip AND back onto the
    # bench; z covers the working band plus room for a cup above a cup.
    xs = [round(-0.60 + 0.04 * i, 3) for i in range(31)]
    ys = [0.13, 0.17, 0.21, 0.25, 0.29]
    zs = [round(1.06 + 0.04 * i, 3) for i in range(8)]      # 1.06 .. 1.34

    table = {}
    for pol in ("anchor", "topdown"):
        for arm in ("left", "right"):
            q = anchor[arm] if pol == "anchor" else topdown
            t = {}
            for x in xs:
                for y in ys:
                    for z in zs:
                        t[(x, y, z)] = ok(arm, [x, y, z], q, a.repeats)
            table[(pol, arm)] = t
            print("  %-8s %-5s solo-reachable %4d of %d"
                  % (pol, arm, sum(t.values()), len(t)))

    # ---- CONTROL.  A zero for stacks is only meaningful if each arm can
    # reach something on its own in this very grid.
    solo = {"%s_%s" % (p, arm): sum(table[(p, arm)].values())
            for p in ("anchor", "topdown") for arm in ("left", "right")}
    res = {"solo_reachable": solo, "xs": xs, "ys": ys, "zs": zs,
           "repeats": a.repeats, "bench_in_scene": bench_in}
    if not all(solo.values()):
        zero = [k for k, v in solo.items() if not v]
        print("\n  CONTROL FAILED for %s -- refusing to report a stack count "
              "from a grid one arm cannot reach at all." % zero)
        res["verdict"] = "INVALID -- control failed: %s" % zero
        json.dump(res, open(OUT, "w"), indent=2)
        return 7

    # ---- the stacks
    print("\nSTACKS  (source cup above destination cup: same x and y, "
          "z apart)")
    # dz MUST be a multiple of the z step or `z + dz` never lands on a grid
    # value and the loop body never runs.  The first version used 0.10/0.14/
    # 0.18 against a 0.04 grid and reported "0 stacks tested, 0 admitted a
    # pair" -- a check that passed on no data, which is the failure class this
    # project keeps a list of.  Pairs are now taken from the grid itself.
    zstep = round(zs[1] - zs[0], 3)
    hits, tested = [], 0
    for dz in (round(2 * zstep, 3), round(3 * zstep, 3), round(4 * zstep, 3)):
        for pol in ("anchor", "topdown", "mixed"):
            got = 0
            for x in xs:
                for y in ys:
                    for z in zs:
                        z2 = round(z + dz, 3)
                        if z2 > zs[-1] + 1e-9:
                            continue
                        tested += 1
                        for hi, lo in (("left", "right"), ("right", "left")):
                            ph = ("anchor" if pol in ("anchor", "mixed")
                                  else "topdown")
                            pl = "topdown" if pol == "mixed" else ph
                            if (table[(ph, hi)][(x, y, z2)] and
                                    table[(pl, lo)][(x, y, z)]):
                                got += 1
                                hits.append(dict(dz=dz, policy=pol, x=x, y=y,
                                                 z_low=z, above=hi, below=lo))
                                break
            print("   dz=%.2f  %-8s  %d stacks" % (dz, pol, got))
    if tested == 0:
        print("\n  REFUSING: 0 stacks were actually tested.  A zero from a "
              "loop that never ran is not a measurement.")
        res["verdict"] = "INVALID -- 0 stacks tested"
        json.dump(res, open(OUT, "w"), indent=2)
        return 8
    res["stacks_tested"] = tested
    res["stack_hits"] = len(hits)
    res["examples"] = hits[:8]
    res["verdict"] = ("POSSIBLE" if hits else
                      "IMPOSSIBLE -- no cup-over-cup pair anywhere in the "
                      "swept volume, at any tested dz or orientation policy")
    print("\n  %d stacks tested, %d admitted a pair -> %s"
          % (tested, len(hits), res["verdict"]))

    # ---- GATE 2, right arm, IN ITS OWN BAND rather than the left's mirror.
    print("\nGATE 2 addendum: the right arm in ITS OWN measured band")
    band = [[x, y, CT.BENCH_TOP + 0.02]
            for x in (-0.48, -0.51, -0.54, -0.57)
            for y in (0.15, 0.19, 0.23)]
    g2r = {}
    for pol, q in (("fixed", anchor["right"]), ("topdown", topdown)):
        g2r[pol] = sum(1 for p in band if ok("right", CT.ee_for(p), q, 3))
    g2r["yaw_free"] = sum(1 for p in band
                          if n.solve_yaw_free("right", CT.ee_for(p),
                                              anchor["right"]))
    g2r["of"] = len(band)
    print("   " + "   ".join("%s %d/%d" % (k, v, g2r["of"])
                             for k, v in g2r.items() if k != "of"))
    res["gate2_right_own_band"] = g2r

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
