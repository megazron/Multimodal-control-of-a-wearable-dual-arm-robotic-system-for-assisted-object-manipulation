#!/usr/bin/env python3
"""How far FORWARD can the table's near edge come before the grasp dies?

    python3 scripts/sweep_table_edge.py [--repeats 5]

WHY. "Objects fall outside the table -- change its dimensions so everything
sits on it properly." The work-surface survey says why they are outside it:
EVERY reachable cell on the work plane lies at y = 0.05..0.20, and the table's
near edge is at y = 0.245. The whole usable region is in front of the table,
so the objects are not misplaced -- the table is.

The obvious fix is to bring the edge forward under them. The obstacle is the
approach cone: the pinned tool axis is 30.7 deg above horizontal, so the hand
arrives from the near side and from BELOW, through exactly the volume a
forward edge would fill. This sweeps the edge and scores T1's own verified
pick paths, so the answer is a measured limit rather than an argument.

CONTROLS: the shipped edge (0.245) must score 0, and an edge pushed under the
objects must score more than 0. Without both, a table of zeros is
indistinguishable from a loop that never ran.
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
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/table_edge_sweep.json")
STANDOFF, LIFT = 0.10, 0.08


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s %.4f rad from home" % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}

    import clip_scene as CS
    import time as _t
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)

    def apply_bench(near_y, top_z=None, thick=None):
        CS.remove_furniture(n)
        top = CT.BENCH_TOP if top_z is None else top_z
        th = CT.BENCH_THICK if thick is None else thick
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "bench"
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        yd = CT.BENCH_FAR_Y - near_y
        pr.dimensions = [2 * CT.BENCH_HALF_X, yd, th]
        co.primitives.append(pr)
        p = Pose()
        p.position.x = 0.0
        p.position.y = (near_y + CT.BENCH_FAR_Y) / 2.0
        p.position.z = top - th / 2.0
        p.orientation.w = 1.0
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = [co]
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)

    calls = {"n": 0}

    def ok(p):
        for _ in range(a.repeats):
            calls["n"] += 1
            if not n.solve(MCT.T1_ARM, list(p), quat[MCT.T1_ARM], tries=6):
                return False
        return True

    def score():
        bad = 0
        for x, y in MCT.T1_CUBES + MCT.T1_PLANES:
            ee = CT.ee_for([x, y, MCT.T1_Z])
            pre = [ee[0], ee[1], ee[2] + STANDOFF]
            up = [ee[0], ee[1], ee[2] + LIFT]
            for w in densify([pre, ee], 0.03) + densify([ee, up], 0.03):
                bad += not ok(w)
        return bad

    print("TABLE EDGE SWEEP -- waypoint failures over T1's six pick paths, "
          "N=%d" % a.repeats)
    print("  the shipped edge is y = %.3f; every reachable work cell is at "
          "y = 0.05..0.20" % CT.BENCH_NEAR_Y)
    rows = {}
    for near in (0.245, 0.220, 0.200, 0.180, 0.160, 0.140, 0.100):
        apply_bench(near)
        s = score()
        rows["edge_%.3f" % near] = s
        print("   near edge y=%.3f   %3d failures%s"
              % (near, s, "   <- shipped" if abs(near - 0.245) < 1e-6 else ""))

    # A LOWER TABLE, which is the other way to get the top out of the cone.
    # The objects would then need a stand; this measures whether the ARM is
    # happy with a table top well below the work plane.
    for top in (1.00, 0.95, 0.90):
        apply_bench(0.100, top_z=top)
        s = score()
        rows["low_%.2f_edge_0.100" % top] = s
        print("   top z=%.2f, edge y=0.100   %3d failures" % (top, s))

    CS.remove_furniture(n)
    shipped = rows["edge_0.245"]
    blocked = rows["edge_0.100"]
    good = shipped == 0 and blocked > 0
    print("  controls: shipped %d (want 0), edge under the objects %d "
          "(want > 0) -> %s" % (shipped, blocked, "OK" if good else "FAILED"))
    out = dict(rows=rows, repeats=a.repeats, ik_calls=calls["n"],
               controls_ok=bool(good), shipped_edge=CT.BENCH_NEAR_Y)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
