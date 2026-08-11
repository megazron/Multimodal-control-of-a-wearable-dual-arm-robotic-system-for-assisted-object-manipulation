#!/usr/bin/env python3
"""Which support geometry lets T1's objects REST on something and still be
picked by the pinned wrist?

    python3 scripts/sweep_t1_supports.py [--repeats 5]

WHY. Making every object rest on a surface is a stated requirement, and the
first attempt at it -- a cantilevered lip under the rear of each cube and each
plane -- took T1 from 0 waypoint failures to 52 on the picks and 16 on the
transports. That is the same wall the 2026-08-11 rail sweep hit: the pinned
tool axis is 30.7 deg above horizontal, so the hand enters from the near side
and from BELOW, and anything in that cone converts an unsupported object into
an unreachable one.

This asks the question properly instead of guessing: which pieces of support,
at which overhang, cost how many waypoints. It CHANGES NOTHING -- it prints a
table and writes a baseline.

CONTROLS. A bench-only scene must give 0 failures (that is the shipped,
verified layout) and a deliberately absurd support -- a slab filling the whole
approach cone -- must give many. Without both, a table of zeros would be
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
from moveit_msgs.msg import PlanningScene                    # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene               # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_support_sweep.json")
STEP, STANDOFF, LIFT = 0.02, 0.10, 0.08


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=5)
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
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    import clip_scene as CS
    import time as _t
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)

    def apply(boxes):
        CS.remove_furniture(n)
        objs = []
        for name, xyz, size, _c in boxes:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            pr = SolidPrimitive()
            pr.type = SolidPrimitive.BOX
            pr.dimensions = [float(v) for v in size]
            co.primitives.append(pr)
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = [float(v)
                                                           for v in xyz]
            ps.orientation.w = 1.0
            co.primitive_poses.append(ps)
            co.operation = CollisionObject.ADD
            objs.append(co)
        if not objs:
            return
        ps2 = PlanningScene()
        ps2.is_diff = True
        ps2.world.collision_objects = objs
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps2))
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
        """Waypoint failures over T1's six pick paths and eight transports."""
        zc = MCT.T1_Z
        bad = 0
        for x, y in MCT.T1_CUBES + MCT.T1_PLANES:
            ee = CT.ee_for([x, y, zc])
            pre = [ee[0], ee[1], ee[2] + STANDOFF]
            up = [ee[0], ee[1], ee[2] + LIFT]
            for w in densify([pre, ee], STEP) + densify([ee, up], STEP):
                bad += not ok(w)
        return bad

    bench = [f for f in CS.furniture_boxes("t1") if f[0] == "bench"]

    def lips(which, frac):
        old = CS.SUPPORT_UNDER_FRAC
        CS.SUPPORT_UNDER_FRAC = frac
        try:
            out = []
            if "cube" in which:
                for i, (cx, cy) in enumerate(MCT.T1_CUBES):
                    out.append(CS._lip("lip_cube_%d" % i, [cx, cy, MCT.T1_Z],
                                       (0.04, 0.04, 0.04), CT.BENCH_NEAR_Y))
            if "plane" in which:
                for i, (px, py) in enumerate(MCT.T1_PLANES):
                    out.append(CS._lip(
                        "lip_plane_%d" % i,
                        [px, py, CT.BENCH_TOP + CS.PLANE_T / 2.0],
                        (CS.PLANE_W, CS.PLANE_D, CS.PLANE_T),
                        CT.BENCH_NEAR_Y))
            return out
        finally:
            CS.SUPPORT_UNDER_FRAC = old

    def footprint_only(items, size):
        """Support confined to the object's OWN footprint -- it does not reach
        back to the bench. Not buildable on its own; it ISOLATES which half of
        a lip obstructs, the part under the object or the part between the
        object and the bench edge."""
        out = []
        for i, (x, y) in enumerate(items):
            out.append(("fp_%d" % i,
                        [x, y, CT.BENCH_TOP - 0.006],
                        [size[0] + 0.02, size[1], 0.012],
                        (0.6, 0.5, 0.3, 1.0)))
        return out

    def side_ledges(items, size, dx=0.06):
        """Support from the SIDE: a post outboard of the gripper corridor,
        and a thin ledge reaching in under the object at its own y."""
        out = []
        for i, (x, y) in enumerate(items):
            for s2, sgn in (("p", 1), ("m", -1)):
                out.append(("post_%d%s" % (i, s2),
                            [x + sgn * (dx + 0.015), y,
                             CT.BENCH_TOP - 0.06],
                            [0.03, size[1], 0.12], (0.5, 0.5, 0.5, 1.0)))
                out.append(("ledge_%d%s" % (i, s2),
                            [x + sgn * (dx / 2.0 + size[0] / 4.0), y,
                             CT.BENCH_TOP - 0.004],
                            [dx - size[0] / 2.0 + 0.02, size[1], 0.008],
                            (0.6, 0.5, 0.3, 1.0)))
        return out

    cases = [("bench only (shipped, CONTROL: must be 0)", bench)]
    cases.append(("cube footprint-only pads",
                  bench + footprint_only(MCT.T1_CUBES, (0.04, 0.04))))
    cases.append(("cube+plane footprint-only pads",
                  bench + footprint_only(MCT.T1_CUBES, (0.04, 0.04))
                  + footprint_only(MCT.T1_PLANES, (CS.PLANE_W, CS.PLANE_D))))
    cases.append(("cube SIDE ledges dx=0.06",
                  bench + side_ledges(MCT.T1_CUBES, (0.04, 0.04), 0.06)))
    cases.append(("cube SIDE ledges dx=0.09",
                  bench + side_ledges(MCT.T1_CUBES, (0.04, 0.04), 0.09)))
    for frac in (0.75, 0.5, 0.25):
        cases.append(("cube lips, %d%% under" % (frac * 100),
                      bench + lips(("cube",), frac)))
    for frac in (0.75, 0.5, 0.25):
        cases.append(("plane lips, %d%% under" % (frac * 100),
                      bench + lips(("plane",), frac)))
    for frac in (0.75, 0.5, 0.25):
        cases.append(("cube + plane lips, %d%% under" % (frac * 100),
                      bench + lips(("cube", "plane"), frac)))
    # CONTROL that must FAIL: a slab filling the approach cone in front of
    # the cubes. If this scores 0 the sweep is not measuring supports at all.
    cases.append(("CONTROL slab across the approach (must be > 0)",
                  bench + [("blocker", [0.34, 0.16, CT.BENCH_TOP - 0.03],
                            [0.40, 0.14, 0.06], (0.5, 0.5, 0.5, 1.0))]))

    print("T1 SUPPORT SWEEP -- waypoint failures over 6 pick paths, N=%d"
          % a.repeats)
    rows = {}
    for label, boxes in cases:
        apply(boxes)
        s = score()
        rows[label] = s
        print("   %-46s %3d failures" % (label, s))
    CS.remove_furniture(n)
    good = rows[cases[0][0]] == 0 and rows[cases[-1][0]] > 0
    print("   controls %s" % ("OK" if good else "FAILED -- report nothing"))
    out = dict(rows=rows, repeats=a.repeats, ik_calls=calls["n"],
               controls_ok=bool(good))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("   -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
