#!/usr/bin/env python3
"""Where can each arm actually WORK on the table top?

    python3 scripts/survey_work_surface.py [--repeats 3] [--full-path]

WHY. Four things in the current brief all need the same number and none of
them can be answered without it:

  * the coloured planes are to go in the FRONT CENTRE -- but this platform's
    central recorded finding is that 0 of 63 frontal cells are reachable by
    BOTH arms and the whole centreline x in [-0.2, +0.2] by NEITHER, so
    "centred and reachable" needs measuring before it is designed around;
  * the table is to be resized so everything sits on it, which means knowing
    what "everything" spans;
  * WORKSPACE MARKINGS are to be drawn on the surface -- they have to be the
    measured boundary, not a drawn guess, or they are decoration that lies;
  * T1 stage 2 samples random positions "anywhere in the marked reachable
    region", so the region must exist as a number first.

WHAT IS MEASURED. A grid on the work plane. For each cell, for each arm, IK at
the arm's own anchor orientation -- the PINNED wrist, which is what every mode
actually commands -- with the scene loaded. `--full-path` additionally
requires the whole pick path (standoff, descend, lift) rather than the grasp
pose alone, which is the difference that has previously turned 10 feasible
cells into 0.

CONTROLS, because a map of zeros and a loop that never ran look identical:
a pose 1.6 m out must be unreachable, a pose inside the wearer must be
unreachable, and a known-good pick must be reachable. A failed control means
no report.
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
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/work_surface_region.json")
STANDOFF, LIFT = 0.10, 0.08


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--x", type=float, nargs=2, default=(-0.70, 0.70))
    ap.add_argument("--y", type=float, nargs=2, default=(0.05, 0.55))
    ap.add_argument("--z", type=float, default=None,
                    help="object-centre height; default bench top + 20 mm")
    ap.add_argument("--full-path", action="store_true",
                    help="require standoff+descend+lift, not just the grasp")
    ap.add_argument("--scene", default="t1",
                    help="which task's furniture to load")
    a = ap.parse_args()
    z = CT.BENCH_TOP + 0.02 if a.z is None else a.z

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, _ = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s is %.4f rad from home." % (arm, w))
            return 3
    quat = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quat.values()):
        print("REFUSING: no tf2 EE orientation")
        return 4

    import clip_scene as CS
    import time as _t
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    tmp = CS.Scene.__new__(CS.Scene)
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)
    CS.remove_furniture(n)
    objs = CS.Scene._collision_furniture(tmp, a.scene)
    if objs:
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = objs
        fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
        end = _t.time() + 20.0
        while _t.time() < end and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)

    calls = {"n": 0}

    def ok(arm, p, k=None):
        for _ in range(a.repeats if k is None else k):
            calls["n"] += 1
            if not n.solve(arm, list(p), quat[arm], tries=6):
                return False
        return True

    def cell_ok(arm, obj_xyz):
        """Grasp pose alone, or the whole pick path."""
        ee = CT.ee_for(obj_xyz)
        if not a.full_path:
            return ok(arm, ee)
        pre = [ee[0], ee[1], ee[2] + STANDOFF]
        up = [ee[0], ee[1], ee[2] + LIFT]
        for w in densify([pre, ee], 0.03) + densify([ee, up], 0.03):
            if not ok(arm, w):
                return False
        return True

    ctl_far = ok("left", [1.60, 0.35, 1.15], k=1)
    ctl_wearer = ok("left", [0.0, -0.10, 1.25], k=1)
    ctl_good = ok("left", CT.A_PICK, k=1)
    print("CONTROLS  1.6 m out -> %s | inside the wearer -> %s | known pick -> %s"
          % ("REACHABLE" if ctl_far else "unreachable",
             "REACHABLE" if ctl_wearer else "unreachable",
             "reachable" if ctl_good else "UNREACHABLE"))
    if ctl_far or ctl_wearer or not ctl_good:
        print("REFUSING TO REPORT: a control failed.")
        n.destroy_node()
        rclpy.shutdown()
        return 6

    def frange(lo, hi, st):
        out, v = [], lo
        while v <= hi + 1e-9:
            out.append(round(v, 4))
            v += st
        return out

    xs, ys = frange(*a.x, a.step), frange(*a.y, a.step)
    grid = {}
    print("\nsurveying %d x %d = %d cells at z=%.3f, %s, N=%d, scene %r"
          % (len(xs), len(ys), len(xs) * len(ys), z,
             "FULL PATH" if a.full_path else "grasp pose only",
             a.repeats, a.scene))
    for arm in ("left", "right"):
        cells = []
        for x in xs:
            for y in ys:
                if cell_ok(arm, [x, y, z]):
                    cells.append([x, y])
        grid[arm] = cells
        if cells:
            X = [c[0] for c in cells]
            Y = [c[1] for c in cells]
            print("   %-5s %3d of %d cells   x %.2f..%.2f   y %.2f..%.2f"
                  % (arm, len(cells), len(xs) * len(ys),
                     min(X), max(X), min(Y), max(Y)))
        else:
            print("   %-5s NO reachable cell in this box" % arm)

    both = [c for c in grid["left"] if c in grid["right"]]
    either = [[x, y] for x in xs for y in ys
              if [x, y] in grid["left"] or [x, y] in grid["right"]]
    centre = [c for c in either if abs(c[0]) <= 0.10]
    print("\n   BOTH arms      %d cells" % len(both))
    print("   EITHER arm     %d cells" % len(either))
    print("   FRONT CENTRE   %d cells with |x| <= 0.10  %s"
          % (len(centre), sorted(centre)[:6]))

    out = dict(z=z, step=a.step, repeats=a.repeats, full_path=a.full_path,
               scene=a.scene, x_range=list(a.x), y_range=list(a.y),
               cells=grid, both=both, either=either, front_centre=centre,
               ik_calls=calls["n"],
               controls=dict(far=not ctl_far, wearer=not ctl_wearer,
                             known_good=bool(ctl_good)))
    CS.remove_furniture(n)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("\n   %d IK calls -> %s" % (calls["n"], OUT))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
