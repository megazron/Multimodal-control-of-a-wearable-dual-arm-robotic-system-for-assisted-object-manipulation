#!/usr/bin/env python3
"""Does a T1 layout FIT in the left arm's region, at the CURRENT bench edge?

Four cubes at >= 60 mm pitch plus two coloured placement targets, all of them
SUPPORTED (resting on the bench, not floating in front of its edge) and all of
them reachable N=10 over the full pick and place path.

WHY IT SEARCHES RATHER THAN TESTING ONE LAYOUT.  A single hand-placed layout
that fails proves nothing -- it might just be a bad layout.  So the feasible
cells are enumerated first and every combination of six is searched.  Only
then does "does not fit" mean the region cannot hold it.

THE BINDING MODE IS MASTER_TELEOP.  It is the one condition whose wrist is
pinned, so a layout it cannot reach is a layout T1 cannot run in all five
conditions.  Top-down is reported alongside, because the difference between
the two is the whole story if the answer is no.

    python3 scripts/verify_t1_layout.py [--repeats 10]
"""
import argparse
import itertools
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
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from srl_autonomy.grasp_library import _quat_from_z_and_x    # noqa: E402
from geometry_msgs.msg import Quaternion                     # noqa: E402
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_layout.json")

CUBE_M = 0.040
PITCH_M = 0.060              # T0's measured minimum separation
PLANE = (0.110, 0.080)       # a coloured placement target
# A plane must clear the cubes and the other plane by at least its own
# half-diagonal, or a cube released over one lands on the other.
PLANE_CLEAR_M = 0.5 * math.hypot(*PLANE) + CUBE_M / 2.0
STANDOFF_M = 0.10            # task_actions.STANDOFF_M
LIFT_M = 0.08                # task_actions.LIFT_M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--quick", type=int, default=3)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    w, _ = n.home_ok("left")
    if w > HOME_TOL_RAD:
        print("REFUSING: left arm %.4f rad from home" % w)
        return 3
    anchor = n.ee_quat("left")
    if anchor is None:
        print("REFUSING: no tf2 EE orientation")
        return 4
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
    applied = False
    for _ in range(3):
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if fut.done():
            applied = True
            break
    if not applied:
        print("REFUSING: bench did not apply")
        return 5
    print("bench in scene: True   edge y=%.3f, top z=%.3f"
          % (CT.BENCH_NEAR_Y, CT.BENCH_TOP))

    calls = {"n": 0}

    def ok(arm, ee, q, k):
        for _ in range(k):
            calls["n"] += 1
            if not n.solve(arm, list(ee), q, tries=6):
                return False
        return True

    def full_path_ok(obj_xyz, q, k):
        """The WHOLE pick: standoff, descend, lift -- densified.  Endpoints
        being reachable says nothing about the segment between them, and the
        arm flies it."""
        ee = CT.ee_for(obj_xyz)
        pre = [ee[0], ee[1], ee[2] + STANDOFF_M]
        lift = [ee[0], ee[1], ee[2] + LIFT_M]
        path = densify([pre, ee], 0.02) + densify([ee, lift], 0.02)
        if not path:
            raise RuntimeError("densify returned nothing -- refusing to "
                               "report a pass on an empty path")
        return all(ok("left", w, q, k) for w in path)

    zc = CT.BENCH_TOP + CUBE_M / 2.0

    # ---- controls ----------------------------------------------------
    ctl_good = ok("left", CT.A_PICK, anchor, 1)
    ctl_far = ok("left", [1.6, 0.35, zc], anchor, 1)
    print("CONTROLS  known-good pick -> %s;  1.6 m out -> %s"
          % ("ok" if ctl_good else "UNREACHABLE",
             "SOLVED" if ctl_far else "none"))
    if not ctl_good or ctl_far:
        print("REFUSING TO REPORT: a control failed.")
        return 6

    # ---- the feasible cells ------------------------------------------
    xs = [round(0.24 + 0.02 * i, 3) for i in range(14)]      # 0.24 .. 0.50
    ys = [round(0.13 + 0.02 * i, 3) for i in range(12)]      # 0.13 .. 0.35
    res = {"bench_near_y": CT.BENCH_NEAR_Y, "cube_m": CUBE_M,
           "pitch_m": PITCH_M, "plane": list(PLANE),
           "plane_clear_m": round(PLANE_CLEAR_M, 4), "repeats": a.repeats}

    def supported(y):
        return (y - CUBE_M / 2.0) >= CT.BENCH_NEAR_Y

    for pol, q in (("fixed", anchor), ("topdown", topdown)):
        cells = []
        for x in xs:
            for y in ys:
                if not supported(y):
                    continue
                if full_path_ok([x, y, zc], q, a.quick):
                    cells.append((x, y))
        print("\n%s: %d cells both SUPPORTED and reachable over the full path"
              % (pol.upper(), len(cells)))
        entry = {"cells": len(cells),
                 "extent_mm": (None if not cells else
                               [round(1000 * (max(c[0] for c in cells)
                                              - min(c[0] for c in cells))),
                                round(1000 * (max(c[1] for c in cells)
                                              - min(c[1] for c in cells)))])}
        if len(cells) < 6:
            # THE GUARD: a search over fewer than six cells cannot place six
            # items, and reporting "no layout found" from it would be true
            # but uninformative.  Say which it is.
            entry["verdict"] = ("IMPOSSIBLE -- only %d usable cells, and six "
                                "items are required" % len(cells))
            print("   -> %s" % entry["verdict"])
            res[pol] = entry
            continue

        # ---- search every combination of six -------------------------
        found, tried = None, 0
        for combo in itertools.combinations(cells, 6):
            tried += 1
            # 4 cubes + 2 planes; try each choice of which two are planes
            for planes in itertools.combinations(range(6), 2):
                cubes = [combo[i] for i in range(6) if i not in planes]
                pl = [combo[i] for i in planes]
                if any(math.dist(p, r) < PITCH_M
                       for p, r in itertools.combinations(cubes, 2)):
                    continue
                if math.dist(pl[0], pl[1]) < PLANE_CLEAR_M:
                    continue
                if any(math.dist(c, p) < PLANE_CLEAR_M
                       for c in cubes for p in pl):
                    continue
                found = {"cubes": [list(c) for c in cubes],
                         "planes": [list(p) for p in pl]}
                break
            if found:
                break
        entry["combinations_tried"] = tried
        if tried == 0:
            entry["verdict"] = "INVALID -- 0 combinations tested"
            print("   REFUSING: 0 combinations tested.")
        elif not found:
            entry["verdict"] = ("DOES NOT FIT -- %d combinations of 6 cells "
                                "searched, none satisfies the spacing" % tried)
            print("   -> %s" % entry["verdict"])
        else:
            # Re-verify the winning layout at the FULL N over the full path.
            allok = all(full_path_ok([c[0], c[1], zc], q, a.repeats)
                        for c in found["cubes"] + found["planes"])
            entry["layout"] = found
            entry["verified_at_N"] = bool(allok)
            entry["verdict"] = "FITS" if allok else "FAILED at N=%d" % a.repeats
            print("   -> %s" % entry["verdict"])
            for c in found["cubes"]:
                print("      cube  x=%.3f y=%.3f" % tuple(c))
            for p in found["planes"]:
                print("      plane x=%.3f y=%.3f" % tuple(p))
        res[pol] = entry

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
