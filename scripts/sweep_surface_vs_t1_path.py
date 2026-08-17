#!/usr/bin/env python3
"""THE SURFACE AGAINST T1'S **WHOLE** PATH, height and near edge together.

    python3 scripts/sweep_surface_vs_t1_path.py --repeats 3

WHY THIS EXISTS AND WHY THE EXISTING SWEEP WAS NOT ENOUGH.
`measure_objects_on_the_table.py` walks T1's SIX PICK PATHS and reported a slab
at 1.020 as costing 0 of 54 waypoints. Acting on that -- raising the drawn
surface from 0.950 to 1.020 and bringing its near edge forward from 0.100 to
0.050 so the workspace marking would be painted on something -- put
`verify_t1_paths.py` at **36 IK failures of 171** on the full path, from 0.

Picks are not the path. The path also contains the two PAD PLACEMENTS, the
transits between pick and place, and the standoff and lift at each end, and a
sweep that scores only the picks cannot see a placement being deleted. That is
the same shape as the earlier fault where a verifier carried its own stale copy
of the layout: the instrument measured something adjacent to the question.

So this walks `msc_clip_tasks.t1()` -- the list the recorder actually sends,
imported and not reconstructed -- with a slab really applied, over a grid of
(top, near edge), and reports failures BROKEN DOWN BY PHASE so the answer says
WHICH waypoints a given surface costs.

CONTROLS, and no report without them:
    no slab                        must be 0 failures
    a slab THROUGH the objects     must be > 0
    the shipped surface 0.950/0.100 must be 0 -- the committed known answer
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_scene as CS                                      # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR, _remove     # noqa: E402
from search_t1_layout_on_surface import put_table, TABLE_ID   # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/surface_vs_t1_path.json")


def phase_of(i, n_per_cube):
    """Which part of one cube's chain a waypoint index belongs to."""
    k = i % n_per_cube if n_per_cube else 0
    return k


def t1_waypoints():
    """(arm, [waypoints]) for T1's working arm, from the task's own builder."""
    seq = MCT.t1()
    arm = MCT.T1_ARM
    return arm, [list(w) for w in seq[arm]]


def walk(rig, arm, pts, repeats, floor):
    """(failures, worst_clearance, indices_of_failures)."""
    bad, worst, idx = 0, 1e9, []
    for i, w in enumerate(pts):
        j, ok = None, True
        for _ in range(repeats):
            j = rig.solve_joints(arm, w, avoid=True)
            rig.calls += 1
            if j is None:
                ok = False
                break
        if not ok:
            bad += 1
            idx.append(i)
            continue
        c, _k = rig.clearance(arm, j)
        if c is not None:
            worst = min(worst, c)
    return bad, (None if worst > 1e8 else worst), idx


def frange(lo, hi, st):
    out, v = [], lo
    while v <= hi + 1e-9:
        out.append(round(v, 4))
        v += st
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--tops", type=float, nargs="*",
                    default=[0.950, 0.980, 1.000, 1.020])
    ap.add_argument("--edges", type=float, nargs="*",
                    default=[0.050, 0.062, 0.080, 0.100])
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for aa in ("left", "right"):
        w, j = n.home_ok(aa)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (aa, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    arm, pts = t1_waypoints()
    print("T1 FULL PATH vs the surface -- arm %s, %d waypoints, N=%d, "
          "anchor %s" % (arm.upper(), len(pts), a.repeats, rig.anchor))

    res = dict(arm=arm, waypoints=len(pts), repeats=a.repeats,
               anchor=rig.anchor, floor=a.floor)

    # ------------------------------------------------------------ controls
    rig.set_furniture(False)
    _remove(rig.n, rig.cli, [TABLE_ID])
    b_none, c_none, _ = walk(rig, arm, pts, 1, a.floor)
    put_table(rig, MCT.T1_Z + 0.02, 0.050)          # slab THROUGH the objects
    b_thru, _, _ = walk(rig, arm, pts, 1, a.floor)
    _remove(rig.n, rig.cli, [TABLE_ID])
    put_table(rig, 0.950, 0.100)                    # the shipped surface
    b_ship, c_ship, _ = walk(rig, arm, pts, 1, a.floor)
    ctl = dict(no_slab=b_none, slab_through_objects=b_thru,
               shipped_0p950_edge_0p100=b_ship)
    good = (b_none == 0 and b_thru > 0 and b_ship == 0)
    print("\nCONTROLS")
    print("   no slab                       %3d failures  (must be 0)" % b_none)
    print("   slab through the objects      %3d failures  (must be > 0)" % b_thru)
    print("   shipped 0.950 / edge 0.100    %3d failures  (must be 0)" % b_ship)
    res["controls"] = ctl
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, **res), open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 6

    # --------------------------------------------------------------- sweep
    print("\nFAILURES OF %d, by surface top (rows) and near edge (columns)"
          % len(pts))
    print("   %-7s %s" % ("top", "".join("%9.3f" % e for e in a.edges)))
    grid, best = {}, None
    for t in a.tops:
        row, line = {}, ""
        for e in a.edges:
            _remove(rig.n, rig.cli, [TABLE_ID])
            put_table(rig, t, e)
            b, c, idx = walk(rig, arm, pts, a.repeats, a.floor)
            row[e] = dict(failures=b, worst_clearance=None if c is None
                          else round(c, 4), first_failures=idx[:12])
            line += "%9d" % b
            if b == 0 and (best is None or (t, e) > (best[0], best[1])):
                best = (t, e)
        grid[t] = row
        print("   %-7.3f %s" % (t, line))
    res["failures_by_top_and_edge"] = {
        str(t): {str(e): v for e, v in row.items()} for t, row in grid.items()}

    print("\n" + "=" * 66)
    if best is None:
        print("NO (top, edge) IN THIS GRID COSTS THE FULL PATH NOTHING.")
        print("The shipped 0.950 / 0.100 is the control and it passes, so the "
              "grid simply does not contain it -- widen --tops/--edges.")
        res["best"] = None
    else:
        t, e = best
        print("HIGHEST SURFACE THAT COSTS THE FULL PATH NOTHING: top %.3f, "
              "near edge %.3f" % (t, e))
        print("   work plane %.3f, so the residual gap is %.1f mm."
              % (MCT.T1_Z - MCT.CUBE_M / 2.0,
                 (MCT.T1_Z - MCT.CUBE_M / 2.0 - t) * 1000.0))
        print("   NOTE the edge: a surface that is high AND far forward is not "
              "the same as one that is only high. The pick sweep held the edge "
              "still and could not see this.")
        res["best"] = dict(top=t, near_y=e,
                           gap_m=round(MCT.T1_Z - MCT.CUBE_M / 2.0 - t, 4))
    print("=" * 66)
    print("%d IK calls -> %s" % (rig.calls, a.out))
    json.dump(res, open(a.out, "w"), indent=2)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
