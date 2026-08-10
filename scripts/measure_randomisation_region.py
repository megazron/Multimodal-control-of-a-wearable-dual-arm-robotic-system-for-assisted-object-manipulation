#!/usr/bin/env python3
"""MEASURE THE RANDOMISATION REGION, before anything is designed around it.

An ALOHA-style protocol randomises an object's start position inside a marked
region on the bench.  The region is not a design choice here -- it is whatever
the arms can actually pick from -- so it gets MEASURED first, and the
colour-matched placement task is designed inside the answer.  Designing the
task first and discovering the region afterwards is how the previous task set
acquired five waypoints that no arm could reach.

TWO ORIENTATIONS, AND THEY ARE NOT THE SAME REGION.  That is the point of
running both:

  ANCHOR    the pinned wrist that `orientation_mode: fixed` gives every
            teleoperated mode.  Its tool axis is measured at
            (-0.153, +0.846, +0.511) -- 120.8 deg from straight down and
            30.7 deg ABOVE horizontal -- so the hand comes in from the near
            side, BELOW the object, and reaches up into it.  This is what
            modes 1-4 can command, and it is the ONLY thing they can command:
            nothing in the master measures the wrist.
  TOP-DOWN  approach along -z, built with grasp_library's own
            _quat_from_z_and_x so there is one definition of "top-down" in
            the repository.  This is what modes 5 and 6 command, because they
            emit a full 6-DOF pose.

A colour-matched placement task that is to be COMPARED across modes can only
use the INTERSECTION.  Anywhere else, one family of modes cannot do the task
at all, and the comparison is not a comparison.

    python3 scripts/measure_randomisation_region.py [--repeats 3]

Object positions are swept, not EE poses: `ee_for()` derives the wrist from
the object, because doing it the other way round is what once put the wrist
95 mm inside the bench.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
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

OUT = os.path.join(ROOT, "recordings/baselines/randomisation_region.json")
OBJ_H = 0.040          # the 40 mm block: the canonical graspable object


def largest_rect(cells, xs, ys):
    """Largest axis-aligned rectangle of all-feasible cells, in CELLS.

    A randomisation region has to be a shape an experimenter can mark on a
    bench with tape and an object can be dropped anywhere inside.  A ragged
    feasible set is not that, so the deliverable is the biggest rectangle
    inside it, not the raw count -- which would otherwise flatter a region
    that is a scatter of isolated islands.
    """
    ok = [[cells.get((x, y), False) for y in ys] for x in xs]
    nx, ny = len(xs), len(ys)
    best = (0, None)
    for i0 in range(nx):
        for i1 in range(i0, nx):
            for j0 in range(ny):
                for j1 in range(j0, ny):
                    if all(ok[i][j] for i in range(i0, i1 + 1)
                           for j in range(j0, j1 + 1)):
                        area = (i1 - i0 + 1) * (j1 - j0 + 1)
                        if area > best[0]:
                            best = (area, (xs[i0], xs[i1], ys[j0], ys[j1]))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
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
    anchor = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in anchor.values()):
        print("REFUSING: no tf2 EE orientation.  An identity quaternion is "
              "not a neutral substitute -- it is a specific unreachable pose.")
        return 4

    q = _quat_from_z_and_x([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    topdown = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]),
                         w=float(q[3]))

    import clip_scene as CS
    tmp = CS.Scene.__new__(CS.Scene)
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = CS.Scene._collision_furniture(tmp)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("REFUSING: no /apply_planning_scene -- the bench would be "
              "decoration and every pose would look reachable")
        return 5
    import time as _t
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = _t.time() + 15.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    print("bench applied: %s  (top %.2f m)" % (fut.done(), CT.BENCH_TOP))

    def ok(arm, obj_xyz, quat, k):
        ee = CT.ee_for(obj_xyz)
        for _ in range(k):
            if not n.solve(arm, ee, quat, tries=6):
                return False
        return True

    # OBJECT positions.  The y range MUST straddle the bench edge, not start
    # at it: on_bench() applies OVERHANG = 0.08, so Task A's own block sits at
    # y = 0.185 while the bench begins at 0.245.  A first version of this
    # sweep started at the edge and ran backwards, missed the whole graspable
    # strip, and reported an empty region for both arms -- the instrument, not
    # the robot.
    xs = [round(0.24 + 0.03 * i, 3) for i in range(11)]      # 0.24 .. 0.54
    ys = [round(0.13 + 0.02 * i, 3) for i in range(11)]      # 0.13 .. 0.33
    zc = CT.BENCH_TOP + OBJ_H / 2.0

    def supported(y):
        """Is the object actually RESTING on the bench, or in mid-air?

        Checking this at all is a consequence of the physics gate.  Markers do
        not fall, so a task coordinate can float and nothing downstream
        objects.  Measured: Task A's 40 mm block spans y 0.165..0.205 against
        a bench that starts at 0.245 -- ZERO overlap, 40 mm clear of the edge,
        entirely unsupported.  Under the recommended architecture, where
        objects have dynamics, it hits the floor before the trial starts.
        """
        return (y - OBJ_H / 2.0) >= CT.BENCH_NEAR_Y

    # INSTRUMENT CONTROL.  The headline of this script is a REGION, and an
    # empty region and a broken solver look identical.  The known-good pose
    # is Task A's own pick, which is verified elsewhere.
    ctl = ok("left", CT.A_BLOCK_OBJ, anchor["left"], 1)
    ctl_far = ok("left", [1.60, 0.35, zc], anchor["left"], 1)
    print("CONTROL  Task A's own pick -> %s (want reachable);  1.6 m out -> "
          "%s (want unreachable)"
          % ("reachable" if ctl else "UNREACHABLE",
             "REACHABLE" if ctl_far else "unreachable"))
    if not ctl or ctl_far:
        print("REFUSING TO REPORT: a control failed, so any region printed "
              "here -- especially an empty one -- means nothing.")
        return 6

    res = {"xs": xs, "ys": ys, "object_height_m": OBJ_H,
           "supported_from_y_m": round(CT.BENCH_NEAR_Y + OBJ_H / 2.0, 4),
           "object_centre_z_m": zc, "bench_top_m": CT.BENCH_TOP,
           "bench_near_y_m": CT.BENCH_NEAR_Y, "repeats": a.repeats,
           "controls": dict(known_good_reachable=bool(ctl),
                            far_pose_unreachable=not ctl_far)}
    grids = {}
    for label, quats in (("anchor", anchor),
                         ("topdown", {"left": topdown, "right": topdown})):
        for arm in ("left", "right"):
            sgn = 1.0 if arm == "left" else -1.0
            cells = {}
            print("\n%s / %s  (object x mirrored for the right arm)"
                  % (label.upper(), arm))
            print("       " + "".join("%6.2f" % y for y in ys))
            for x in xs:
                row = ""
                for y in ys:
                    good = ok(arm, [sgn * x, y, zc], quats[arm], a.repeats)
                    cells[(x, y)] = good
                    row += "     %s" % ("#" if good else ".")
                print("%6.2f %s" % (x, row))
            grids[(label, arm)] = cells

    # PER ARM as well as shared.  A shared region is what a task needing both
    # arms at once requires; a PER-ARM region is what a task with its own
    # target set per arm requires -- which is how Task 0 is already built,
    # precisely because 0 of 63 frontal cells are shared.  Reporting only the
    # shared region would report an empty set and imply, wrongly, that nothing
    # can be randomised at all.
    print("\n" + "=" * 70)
    res["per_arm"] = {}
    for label in ("anchor", "topdown"):
        for arm in ("left", "right"):
            cells = grids[(label, arm)]
            sup = {k: (v and supported(k[1])) for k, v in cells.items()}
            for tag, cc in (("reachable", cells), ("supported", sup)):
                area, rect = largest_rect(cc, xs, ys)
                key = "%s_%s_%s" % (label, arm, tag)
                res["per_arm"][key] = dict(
                    cells=sum(1 for v in cc.values() if v),
                    rect=None if not rect else dict(
                        x_min=rect[0], x_max=rect[1],
                        y_min=rect[2], y_max=rect[3],
                        width_mm=round(1000 * (rect[1] - rect[0]), 1),
                        depth_mm=round(1000 * (rect[3] - rect[2]), 1)))
                print("  %-28s %3d cells  %s"
                      % (key, res["per_arm"][key]["cells"],
                         "no rectangle" if not rect else
                         "|x| %.2f..%.2f  y %.2f..%.2f  = %.0f x %.0f mm"
                         % (rect[0], rect[1], rect[2], rect[3],
                            1000 * (rect[1] - rect[0]),
                            1000 * (rect[3] - rect[2]))))
            res["per_arm"]["%s_%s_grid" % (label, arm)] = {
                "%.3f,%.3f" % k: bool(v) for k, v in cells.items()}
    print("\n" + "=" * 70)
    for label in ("anchor", "topdown", "intersection", "supported_anchor",
                  "supported_intersection"):
        if label.startswith("supported_"):
            base = label.split("_", 1)[1]
            src = ("anchor",) if base == "anchor" else ("anchor", "topdown")
            cells = {k: (supported(k[1]) and
                         all(grids[(m, arm)][k] for m in src
                             for arm in ("left", "right")))
                     for k in grids[("anchor", "left")]}
            title = ("SUPPORTED and reachable -- %s.  This is the region a "
                     "task with real object dynamics may use." % base)
        elif label == "intersection":
            cells = {k: all(grids[(m, arm)][k] for m in ("anchor", "topdown")
                            for arm in ("left", "right")) for k in
                     grids[("anchor", "left")]}
            title = "BOTH ARMS, BOTH ORIENTATIONS -- the comparable region"
        else:
            cells = {k: grids[(label, "left")][k] and grids[(label, "right")][k]
                     for k in grids[(label, "left")]}
            title = "BOTH ARMS, %s" % label.upper()
        area, rect = largest_rect(cells, xs, ys)
        nfeas = sum(1 for v in cells.values() if v)
        print("\n%s" % title)
        print("  %d of %d cells feasible for both arms" % (nfeas, len(cells)))
        if rect:
            x0, x1, y0, y1 = rect
            print("  largest rectangle  |x| %.2f..%.2f  y %.2f..%.2f"
                  "   = %.0f x %.0f mm"
                  % (x0, x1, y0, y1, 1000 * (x1 - x0), 1000 * (y1 - y0)))
        else:
            print("  NO rectangle -- there is no region to randomise in.")
        res[label] = dict(
            cells_feasible=nfeas, cells_total=len(cells),
            rect=None if not rect else dict(
                x_min=rect[0], x_max=rect[1], y_min=rect[2], y_max=rect[3],
                width_mm=round(1000 * (rect[1] - rect[0]), 1),
                depth_mm=round(1000 * (rect[3] - rect[2]), 1)),
            grid={"%.3f,%.3f" % k: bool(v) for k, v in cells.items()})

    # REMOVE THE FURNITURE.  The planning scene lives in move_group and
    # OUTLIVES this process, so leaving the bench behind silently changes the
    # answer every later script gets from the same stack.  It did exactly
    # that: the next run of verify_abc_scenarios refused its own control,
    # because Task A's z = 1.05 target sits UNDER the bench slab (1.06..1.10)
    # and that task is verified in free space.  State left from a previous
    # run, which is one of the four instrument-failure mechanisms this
    # project keeps a list of.
    CS.remove_furniture(n)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
