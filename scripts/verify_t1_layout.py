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


def _drop(node, ids):
    """Delete collision objects by id, whoever added them."""
    if not ids:
        return
    from moveit_msgs.msg import CollisionObject
    import time as _t
    ps = PlanningScene()
    ps.is_diff = True
    for i in ids:
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = i
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10.0):
        return
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + 10.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--quick", type=int, default=3)
    ap.add_argument("--rail-y", type=float, default=None,
                    help="front edge of a cantilevered support lip, in y. "
                         "None = no rail, the bench edge does the work.")
    ap.add_argument("--no-support-required", action="store_true",
                    help="OPTION 4: the object is held by a fixture rather "
                         "than resting on the bench.  A cube that cannot "
                         "fall cannot be DROPPED, so `drops` stops being a "
                         "measurable outcome -- that is the cost, and it is "
                         "recorded in the result rather than assumed away.")
    ap.add_argument("--sweep-standoff", action="store_true",
                    help="find the LARGEST standoff/lift that leaves a "
                         "workable layout, instead of testing one value")
    ap.add_argument("--rail-t", type=float, default=0.015,
                    help="rail thickness in z; its TOP is the bench top, so "
                         "the space beneath it stays free for the fingers")
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
    objs = CS.Scene._collision_furniture(tmp)
    rail_ids = []
    if a.rail_y is not None:
        # A LIP CANTILEVERED FORWARD FROM THE BENCH EDGE.  Its TOP is flush
        # with the bench top so an object resting on it is at the same height
        # as one on the bench, and it is thin in z so the space BENEATH it
        # stays free -- which is the whole point, because the pinned approach
        # enters from the near side 30.7 deg above horizontal and the fingers
        # pass UNDER the object.  A shelf standing on the floor would fill
        # exactly that space.
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        depth = CT.BENCH_NEAR_Y - a.rail_y
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "support_rail"
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [1.4, float(depth), float(a.rail_t)]
        co.primitives.append(pr)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = float(a.rail_y + depth / 2.0)
        pose.position.z = float(CT.BENCH_TOP - a.rail_t / 2.0)
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        objs.append(co)
        # REMOVE WHAT YOU ADD.  clip_scene.remove_furniture() only knows its
        # own ids, so "support_rail" survives it -- and the planning scene
        # outlives the process.  Two consecutive runs failed their own control
        # on a rail left behind by a third.  Registered here so every exit
        # path can take it out.
        rail_ids.append("support_rail")
        print("RAIL  edge y=%.3f, depth %.3f m, thickness %.3f m, "
              "top flush with the bench at z=%.3f"
              % (a.rail_y, depth, a.rail_t, CT.BENCH_TOP))
    ps.world.collision_objects = objs
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

    def full_path_ok(obj_xyz, q, k, standoff=STANDOFF_M, lift=LIFT_M):
        """The WHOLE pick: standoff, descend, lift -- densified.  Endpoints
        being reachable says nothing about the segment between them, and the
        arm flies it."""
        ee = CT.ee_for(obj_xyz)
        pre = [ee[0], ee[1], ee[2] + standoff]
        up = [ee[0], ee[1], ee[2] + lift]
        path = densify([pre, ee], 0.02) + densify([ee, up], 0.02)
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
        # CLEAN UP BEFORE RETURNING.  The planning scene lives in move_group
        # and outlives this process: an early return that skips
        # remove_furniture() leaves the rail behind, and the NEXT run inherits
        # it and fails its own control for a reason that has nothing to do
        # with what it is testing.  That happened -- twice.
        _drop(n, rail_ids)
        CS.remove_furniture(n)
        print("REFUSING TO REPORT: a control failed.")
        return 6

    # ---- the feasible cells ------------------------------------------
    xs = [round(0.24 + 0.02 * i, 3) for i in range(14)]      # 0.24 .. 0.50
    ys = [round(0.13 + 0.02 * i, 3) for i in range(12)]      # 0.13 .. 0.35
    support_edge = CT.BENCH_NEAR_Y if a.rail_y is None else a.rail_y
    res = {"bench_near_y": CT.BENCH_NEAR_Y, "rail_y": a.rail_y,
           "rail_t": a.rail_t, "support_edge": None, "cube_m": CUBE_M,
           "pitch_m": PITCH_M, "plane": list(PLANE),
           "plane_clear_m": round(PLANE_CLEAR_M, 4), "repeats": a.repeats}
    res["support_edge"] = support_edge
    res["support_required"] = not a.no_support_required
    if a.no_support_required:
        res["cost_of_option_4"] = ("objects are fixtured, not resting; a cube "
                                   "that cannot fall cannot be dropped, so "
                                   "`drops` is not a measurable outcome")

    def supported(y):
        if a.no_support_required:
            return True
        """Resting on something.  With a rail the support edge moves forward;
        the object still has to OVERHANG it, because support has to come from
        BEHIND -- there is nowhere else it can come from when the fingers
        occupy the space underneath."""
        return (y - CUBE_M / 2.0) >= support_edge

    if a.sweep_standoff:
        # THE STANDOFF LADDER.  The grasp pose is cheap, the path is not, so
        # the supported cells whose GRASP POSE works are found first and only
        # those climb the ladder.  A cell that cannot be grasped at all is
        # not made reachable by a smaller standoff.
        print("\nSTANDOFF SWEEP  (support edge y=%.3f)" % support_edge)
        base = [(x, y) for x in xs for y in ys
                if supported(y) and ok("left", CT.ee_for([x, y, zc]),
                                       anchor, a.quick)]
        print("  %d cells supported with a feasible GRASP POSE (fixed wrist)"
              % len(base))
        if not base:
            print("  REFUSING: 0 cells to sweep.  A ladder with no rungs "
                  "reports nothing, and 0 cells at every standoff would read "
                  "as 'the standoff does not help'.")
            res["standoff_sweep"] = {"verdict": "INVALID -- 0 base cells"}
        else:
            ladder = {}
            for so in (0.10, 0.08, 0.06, 0.04, 0.02):
                for lf in (0.08, 0.04, 0.02):
                    if lf > so:
                        continue
                    good = [c for c in base
                            if full_path_ok([c[0], c[1], zc], anchor,
                                            a.quick, standoff=so, lift=lf)]
                    ladder["so%.2f_lift%.2f" % (so, lf)] = len(good)
                    print("    standoff %.2f  lift %.2f  ->  %2d cells"
                          % (so, lf, len(good)))
            res["standoff_sweep"] = dict(base_cells=len(base), ladder=ladder,
                                         support_edge=support_edge)
        res["ik_calls"] = calls["n"]
        json.dump(res, open(OUT, "w"), indent=2)
        print("\n%d IK calls -> %s" % (calls["n"], OUT))
        _drop(n, rail_ids)
        CS.remove_furniture(n)
        n.destroy_node()
        rclpy.shutdown()
        return 0

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

    # ---- DOES THE RAIL DISTURB ANYTHING ELSE? ------------------------
    # Asserting "it is out of the way" is not evidence.  T0's spheres, T2's
    # carry band and T3's own objects are re-checked WITH the rail in the
    # scene, because the rail sits exactly where a near-side approach passes.
    print("\nREGRESSION with the rail in scene")
    reg = {}
    sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))
    import task0 as T0M
    bad = 0
    for label, p in sorted({**T0M.TARGETS_LEFT}.items()):
        if not ok("left", p, anchor, a.quick):
            bad += 1
    for label, p in sorted({**T0M.TARGETS_RIGHT}.items()):
        if not ok("right", p, n.ee_quat("right"), a.quick):
            bad += 1
    reg["T0_spheres_unreachable"] = bad
    print("   T0 spheres (8)                    %d unreachable" % bad)

    # BOTH ARM ASSIGNMENTS, at N.  The first version forced left=+x and
    # right=-x at k=1 and reported 37 T2 failures -- against a no-rail
    # baseline of exactly 37, i.e. the number was the CHECK, not the scene.
    # verify_abc_scenarios tries both ways for a reason: in this model the
    # links named left_* sit at POSITIVE x, the naming is viewer perspective
    # rather than anatomical, and assuming otherwise once made an audit report
    # every waypoint unreachable on perfectly good geometry.
    import tasks as TSK
    qr = n.ee_quat("right")

    def pair_ok(p, sep, k):
        lo = [p[0] - sep / 2.0, p[1], p[2]]
        hi = [p[0] + sep / 2.0, p[1], p[2]]
        return ((ok("left", hi, anchor, k) and ok("right", lo, qr, k)) or
                (ok("left", lo, anchor, k) and ok("right", hi, qr, k)))

    tb = 0
    for name, path in TSK.TASK_B["paths"].items():
        pts = densify(path, 0.02)
        if not pts:
            raise RuntimeError("densify returned nothing for T2 %s" % name)
        tb += sum(1 for p in pts if not pair_ok(p, TSK.TRAY_SEP, a.repeats))
    reg["T2_carry_waypoint_failures"] = tb
    print("   T2 carry waypoints, both assignments, N=%d   %d failures"
          % (a.repeats, tb))

    t3 = {}
    for nm, obj in (("circuit_box", CT.BOX_OBJ), ("multimeter", CT.C_MM_OBJ)):
        reach = [arm for arm, q2 in (("left", anchor), ("right", qr))
                 if ok(arm, CT.ee_for(obj), q2, a.repeats)]
        t3[nm] = reach
        print("   T3 %-12s reachable by: %s"
              % (nm, ", ".join(reach) if reach else "NEITHER ARM"))
    reg["T3_objects"] = t3
    reg["T3_unreachable_by_either"] = sum(1 for v in t3.values() if not v)
    res["regression"] = reg

    _drop(n, rail_ids)
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
