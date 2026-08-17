#!/usr/bin/env python3
"""A WHOLE T1 LAYOUT THAT RESTS ON THE SURFACE. Arithmetic first, then IK.

    python3 scripts/search_t1_layout_on_surface.py --repeats 10

THE QUESTION. TASK_SPEC T1-1 asks for the cubes to rest on the table. Every
previous attempt swept the SLAB -- raise it under objects fixed in the air --
and priced it at 46 of 54 waypoints, and every one of those was solved through
`measure_what_binds.Rig`, which asked IK for the HOME wrist instead of
`WORKSPACE_ORIENT`. On the left arm, at the current home, those two tool axes
are 32.26 deg apart. So the price was measured at an orientation the task
never sends.

STAGE 0 PREDICTS, IT DOES NOT DECIDE, AND THE FIRST VERSION OF IT WAS WRONG IN
A WAY WORTH KEEPING ON THE PAGE. `ee_for()` puts the WRIST at `obj -
PAD_OFFSET`; for the left arm that is `y - 0.0946`, `z - 0.0572`. Reasoning
about the wrist POINT gives

    y_obj - 0.0946  <  E  <=  y_obj - depth/2

a window 74.6 mm wide, and it is the wrong window. The wrist is not what hits
the table: the GRIPPER BODY is. The pads are 0.0946 m forward and 0.0572 m up
of the wrist along the anchor's tool axis, so the hand is BELOW the surface
plane everywhere in front of

    y_cross  =  y_obj - (z_obj - z_top) * 0.0946 / 0.0572

and the table must therefore start BEHIND that, not in front of it:

    y_obj - 0.0331  <=  E  <=  y_obj - depth/2        (a 40 mm cube: 13.1 mm)

Moving the edge FORWARD -- which is what the first attempt did, to get the
table under the workspace marking -- makes it worse, not better, and the
shipped-cube control caught it: table raised to 1.100 with the edge at 0.062,
the pick T1 has been performing all along went to NO IK SOLUTION.

So the window is about 13 mm wide instead of 75, it is a bound on the OBJECT'S
DISTANCE FROM THE EDGE rather than on the edge itself, and the 0.0331 is a
line estimate that ignores finger width and the rest of the arm. STAGE 1
MEASURES IT: a slab really applied at 1.100, the edge swept, the pick that T1
runs walked in full at N repeats. Stage 0's number is printed next to stage
1's so a disagreement is visible rather than absorbed.

WHAT THAT DOES TO THE PADS, and it is the reason SLOT_DY had to go. The same
bound on a pad of depth D whose far slot is +SLOT_DY needs
`D/2 + SLOT_DY <= 0.0331`-ish -- a pad about 26 mm deep, which cannot hold a
40 mm cube. But SLOT_DY separates the two cubes in Y, which is the one axis
this geometry has none of. Separated in X instead -- two cubes side by side on
a wide shallow mat, which is how a person would lay it out -- the pad only has
to satisfy `D/2 <= 0.0331`-ish and can be as wide as the row allows.
`msc_clip_tasks` chose y because "+/-0.035 in x would put the inboard slot at
0.265, outside" the region as surveyed THEN, x 0.30..0.70. The region now runs
to 1.000, so x is affordable and y is not.

STAGE 2 measures the y window, STAGE 3 the pad depth, STAGE 4 how far outboard
the row stays workable -- the surveyed region stops at |x| = 1.000 because the
survey BOX did, which CLAUDE.md says in as many words -- and STAGE 5 packs the
row and REFUSES to emit a layout with any margin closed.

CONTROLS, and no report without them:
    a pose inside the torso              clearance must be NEGATIVE
    an object BURIED in the raised slab  must FAIL, or the table is not in the
                                         scene and every sweep is about air
    the shipped cube, table at 0.950     must PASS -- the known answer, and it
                                         is the pick T1 has been grasping
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import clip_scene as CS                                      # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_layout_on_surface.json")

# THE TASK'S OWN, imported rather than restated -- msc_clip_tasks.t1() builds
# every pick from these two and a copy here would go stale the way
# verify_msc_tasks' copy of the layout did.
STANDOFF, LIFT = MCT.STANDOFF, MCT.LIFT


# --------------------------------------------------------------- stage 0
def cross_back(arm, z_obj, z_top):
    """How far IN FRONT of an object the gripper breaks the surface plane.

    The pads sit `PAD_OFFSET` forward and up of the wrist along the anchor's
    tool axis, so the hand is below `z_top` everywhere in front of
    `y_obj - this`. A prediction, checked against stage 1.
    """
    off = CT.PAD_OFFSET_BY_ARM.get(arm, CT.PAD_OFFSET)
    if off[2] <= 0:
        return float("inf")
    return (z_obj - z_top) * off[1] / off[2]


def check_layout(arm, edge, back_max, cubes, planes, plane_wd, cube_m):
    """Every margin for a concrete layout. [] means every margin holds.

    `back_max` is the MEASURED greatest distance an object may sit behind the
    near edge -- stage 1's answer, not stage 0's estimate.
    """
    pw, pd = plane_wd
    bad = []
    for i, (cx, cy) in enumerate(cubes):
        if cy - cube_m / 2.0 < edge - 1e-9:
            bad.append("cube_%d near face %.4f is in FRONT of the edge %.4f"
                       % (i, cy - cube_m / 2.0, edge))
        if cy - edge > back_max + 1e-9:
            bad.append("cube_%d sits %.4f behind the edge, past the measured "
                       "%.4f" % (i, cy - edge, back_max))
    for i, (px, py) in enumerate(planes):
        if py - pd / 2.0 < edge - 1e-9:
            bad.append("plane_%d near face %.4f is in FRONT of the edge %.4f"
                       % (i, py - pd / 2.0, edge))
        if py - edge > back_max + 1e-9:
            bad.append("plane_%d sits %.4f behind the edge, past the measured "
                       "%.4f" % (i, py - edge, back_max))
    # x overlap, in the shared row. Pads and cubes are all in it.
    boxes = ([("cube_%d" % i, cx - cube_m / 2, cx + cube_m / 2)
              for i, (cx, _) in enumerate(cubes)]
             + [("plane_%d" % i, px - pw / 2, px + pw / 2)
                for i, (px, _) in enumerate(planes)])
    boxes.sort(key=lambda b: b[1])
    for a, b in zip(boxes, boxes[1:]):
        if b[1] < a[2] - 1e-9:
            bad.append("%s and %s overlap in x by %.1f mm"
                       % (a[0], b[0], (a[2] - b[1]) * 1000.0))
    return bad


TABLE_ID = "search_table"


def put_table(rig, top_z, near_y, far_y=0.72, half_x=1.05, thick=0.035):
    """The slab, as a REAL collision object at this cell's height and edge."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from measure_what_binds import _apply
    o = CollisionObject()
    o.id = TABLE_ID
    o.header.frame_id = "world"
    o.operation = CollisionObject.ADD
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [2 * half_x, far_y - near_y, thick]
    p = Pose()
    p.position.x = 0.0
    p.position.y = float((near_y + far_y) / 2.0)
    p.position.z = float(top_z - thick / 2.0)
    p.orientation.w = 1.0
    o.primitives = [sp]
    o.primitive_poses = [p]
    _apply(rig.n, rig.cli, [o])


# --------------------------------------------------------------- stage 1
def pick_path(obj, arm):
    """The waypoints T1 actually sends around one pick, from the task's own
    builder shape: standoff above, descend, lift. `ee_for` is the task's."""
    ee = CT.ee_for(list(obj), arm)
    above = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([above, ee], 0.025) + densify([ee, up], 0.025)


def workable(rig, arm, obj, repeats, floor):
    """(ok, worst_clearance). Full pick path, N repeats, geometric clearance."""
    worst = 1e9
    for w in pick_path(obj, arm):
        for _ in range(repeats):
            j = rig.solve_joints(arm, w, avoid=True)
            rig.calls += 1
            if j is None:
                return False, None
        c, _k = rig.clearance(arm, j)
        if c is None:
            return False, None
        worst = min(worst, c)
    return worst >= floor, (None if worst > 1e8 else worst)


def frange(lo, hi, st):
    out, v = [], lo
    while v <= hi + 1e-9:
        out.append(round(v, 4))
        v += st
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--scout-repeats", type=int, default=2)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--arm", default=MCT.T1_ARM)
    ap.add_argument("--top", type=float, default=None,
                    help="surface height; default work_surface.table_top()")
    ap.add_argument("--plane-w", type=float, default=0.240)
    ap.add_argument("--gap", type=float, default=0.020)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    arm = a.arm
    sgn = 1.0 if arm == "left" else -1.0
    top = CS.TABLE_TOP if a.top is None else a.top
    cube_m = MCT.CUBE_M
    z_cube = round(top + cube_m / 2.0, 4)      # a cube RESTING on the surface
    res = dict(arm=arm, table_top=top, cube_m=cube_m, z_cube=z_cube,
               plane_w=a.plane_w, gap=a.gap, repeats=a.repeats,
               scout_repeats=a.scout_repeats, floor=a.floor)

    # ------------------------------------------------------------- stage 0
    pred = cross_back(arm, z_cube, top)
    res["stage0_predicted_back_max_m"] = round(pred, 4)
    off = CT.PAD_OFFSET_BY_ARM.get(arm, CT.PAD_OFFSET)
    print("STAGE 0 -- prediction. surface top %.3f, cube centre %.4f, "
          "%s PAD_OFFSET %s" % (top, z_cube, arm, [round(v, 4) for v in off]))
    print("   the gripper breaks the surface plane %.4f m in FRONT of the "
          "object, so an object may sit AT MOST that far behind the near "
          "edge." % pred)
    print("   a %.0f mm cube resting flush leaves %.4f m of that spent on its "
          "own half-depth, so the usable window is %.4f m wide."
          % (cube_m * 1000.0, cube_m / 2.0, pred - cube_m / 2.0))
    print("   THIS IS AN ESTIMATE. It is a line through the tool axis and "
          "ignores finger width and the rest of the arm. Stage 1 measures it.")

    # --------------------------------------------------------------- ROS up
    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for aa in ("left", "right"):
        wr, jj = n.home_ok(aa)
        if wr > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)."
                  % (aa, wr, jj))
            return 3
    rig = Rig(n, "t1", 1)
    print("\n   Rig anchor = %s" % rig.anchor)

    # ------------------------------------------------------------ controls
    # THE SLAB IS OURS FROM HERE ON. clip_scene's own furniture is removed so
    # the only surface in the scene is the one each cell asks for; otherwise a
    # cell would be measured against two tables.
    rig.set_furniture(False)
    ctl, good = {}, True

    jt = rig.solve_joints(arm, [0.05, -0.02, 1.22], avoid=False)
    c_t = rig.clearance(arm, jt)[0] if jt is not None else None
    ctl["inside_torso_clearance_negative"] = dict(
        want="< 0", got=None if c_t is None else round(c_t, 4))
    good = good and c_t is not None and c_t < 0.0

    # THE KNOWN ANSWER: the pick T1 has been performing, with the table where
    # it shipped (0.950). This must pass, and it is what says the rig and the
    # path builder are right before any of the new geometry is believed.
    put_table(rig, 0.950, 0.100)
    ok_s, c_s = workable(rig, arm, [round(sgn * 0.560, 4), 0.120, 1.120],
                         a.scout_repeats, a.floor)
    ctl["shipped_pick_table_at_0.950"] = dict(
        want="reachable and clear", ok=bool(ok_s),
        clearance=None if c_s is None else round(c_s, 4))
    good = good and ok_s

    # BURIED: centre inside the slab. Must FAIL, or the table is not applied
    # and every sweep below is about air.
    put_table(rig, top, 0.100)
    ok_b, _ = workable(rig, arm, [round(sgn * 0.560, 4), 0.400, top - 0.010],
                       1, a.floor)
    ctl["buried_in_the_slab_fails"] = dict(want="not reachable",
                                           reachable=bool(ok_b))
    good = good and not ok_b

    print("\nCONTROLS")
    for k, v in ctl.items():
        print("   %-34s %s" % (k, v))
    res["controls"] = ctl
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused="control", **res), open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 6

    # ------------------------------------------------------------- stage 1
    # HOW FAR BEHIND THE EDGE MAY AN OBJECT SIT? Hold the object still and
    # move the EDGE, which is the same degree of freedom and keeps the arm's
    # pose identical across the sweep, so the only thing changing is the slab.
    # TWO DIMENSIONS, BECAUSE ONE OF THEM TURNED OUT NOT TO BE THE ONE THAT
    # BINDS. Swept at a fixed top of 1.100 the answer was "no edge position
    # works, including an edge at y = 0" -- so the near edge is not what
    # blocks the pick. What blocks it is the slab occupying the volume the
    # ARM needs, which is the bench finding again and is about HEIGHT. So the
    # surface height is swept too, with the object RESTING on each one.
    x_probe = round(sgn * 0.560, 4)
    y_probe = 0.120
    tops = frange(0.900, 1.100, 0.020)
    backs = frange(0.020, 0.060, 0.010)
    print("\nSTAGE 1 -- surface HEIGHT and near edge together. The object "
          "RESTS on each surface (z = top + %.3f), so this is the question "
          "TASK_SPEC T1-1 actually asks." % (cube_m / 2.0))
    print("   object at x %.3f y %.3f, N=%d, full pick path, slab really "
          "applied.  rows = top, columns = how far the object sits behind "
          "the edge" % (x_probe, y_probe, a.scout_repeats))
    print("   %-7s %s" % ("top", "".join("%9.3f" % b for b in backs)))
    s1, best = {}, None
    for t in tops:
        z_here = round(t + cube_m / 2.0, 4)
        cells, line = {}, ""
        for b in backs:
            e = round(y_probe - b, 4)
            put_table(rig, t, e)
            ok, c = workable(rig, arm, [x_probe, y_probe, z_here],
                            a.scout_repeats, a.floor)
            cells[b] = dict(edge=e, ok=bool(ok),
                            clearance=None if c is None else round(c, 4))
            line += "%9s" % ("%.4f" % c if ok and c is not None else "   -")
            if ok and (best is None or t > best[0]):
                best = (t, b, e, c)
        s1[t] = cells
        print("   %-7.3f %s" % (t, line))
    res["stage1_top_by_back"] = {str(k): {str(kk): vv for kk, vv in v.items()}
                                 for k, v in s1.items()}
    if best is None:
        print("\nREFUSING: no (height, edge) pair lets the shipped object be "
              "picked off a surface at all. T1-1 stays BLOCKED and the "
              "layout must not be changed.")
        res["ok"] = False
        json.dump(res, open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 1
    top, b_best, e_best, c_best = best
    z_cube = round(top + cube_m / 2.0, 4)
    res["table_top"] = top
    res["z_cube"] = z_cube
    print("\n   HIGHEST WORKABLE SURFACE: top %.3f, object %.3f behind the "
          "edge (edge %.4f), clearance %.4f" % (top, b_best, e_best, c_best))
    print("   the objects currently float at z = 1.1200 over a top of 0.9500. "
          "Resting them on THIS surface puts them at %.4f -- a move of "
          "%+.1f mm in z, and it is a real change to every verified T1 "
          "coordinate." % (z_cube, (z_cube - 1.120) * 1000.0))
    s1t = s1[top]
    okb = [b for b in sorted(s1t) if s1t[b]["ok"]]
    # The largest back-distance in the run that starts at the cube's own
    # half-depth -- contiguous, because a hole would make the span a lie.
    run = []
    for b in sorted(s1t):
        if s1t[b]["ok"]:
            run.append(b)
        elif run:
            break
    back_meas = run[-1] if run else None
    print("\n   MEASURED: workable while the object is 0 .. %.4f m behind the "
          "edge (%d contiguous steps)" % (back_meas, len(run)))
    print("   stage 0 predicted %.4f m.  %s"
          % (pred, "agree within a step" if abs(back_meas - pred) <= 0.0051
             else "DISAGREE -- the line estimate is not what binds"))
    # STAY ONE STEP INSIDE THE LAST CELL THAT PASSED, then confirm at full N.
    back_use = round(max(cube_m / 2.0, back_meas - 0.005), 4)
    e_use = round(y_probe - back_use, 4)
    put_table(rig, top, e_use)
    okf, cf = workable(rig, arm, [x_probe, y_probe, z_cube], a.repeats, a.floor)
    print("   N=%d at back %.4f (edge %.4f) -> %s clearance %s"
          % (a.repeats, back_use, e_use, "ok" if okf else "NO",
             "----" if cf is None else "%.4f" % cf))
    res["back_max_m"] = back_meas
    res["back_used_m"] = back_use
    res["back_used_full_n_ok"] = bool(okf)
    if not okf:
        print("\nREFUSING: the chosen back-distance does not survive N=%d."
              % a.repeats)
        res["ok"] = False
        json.dump(res, open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 1

    # ------------------------------------------------------------- stage 2
    # THE PAD. It may be at most `back_use` behind the edge measured to its
    # CENTRE, and its near face must be on the table, so its depth is bounded
    # by 2*back_use. Its two slots go side by side in X, not in y.
    plane_d = round(min(2.0 * back_use, 2.0 * back_meas), 4)
    y_row = round(e_use + back_use, 4)
    print("\nSTAGE 2 -- the pad. depth <= 2 x %.4f = %.4f, slots separated in "
          "X. Row y = %.4f for cube and pad centres alike."
          % (back_use, plane_d, y_row))
    print("   the shipped pad was 0.210 x 0.130 with slots at +/-0.030 in Y; "
          "its far slot sat %.4f m behind the edge, %.4f past the measured "
          "limit." % (0.130 / 2.0 + 0.030, 0.130 / 2.0 + 0.030 - back_meas))

    # ------------------------------------------------------------- stage 3
    xs = frange(0.375, 1.300, 0.025)
    print("\nSTAGE 3 -- how wide is the row? x %.3f..%.3f step 0.025 at "
          "y %.4f, N=%d. The survey stopped at 1.000 because its BOX did."
          % (xs[0], xs[-1], y_row, a.scout_repeats))
    put_table(rig, top, e_use)
    scout = {}
    for x in xs:
        ok, c = workable(rig, arm, [round(sgn * x, 4), y_row, z_cube],
                         a.scout_repeats, a.floor)
        scout[x] = dict(ok=bool(ok), clearance=None if c is None else round(c, 4))
        print("   x %.3f  %-4s  clearance %s"
              % (x, "ok" if ok else "no",
                 "  ----" if c is None else "%.4f" % c))
    res["stage3_scout"] = {str(k): v for k, v in scout.items()}
    hits = [x for x in xs if scout[x]["ok"]]
    if not hits:
        print("\nREFUSING: no x on the row is workable, which contradicts "
              "stage 1. The instrument is wrong.")
        res["ok"] = False
        json.dump(res, open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 7
    runs, cur = [], [hits[0]]
    for p, q in zip(hits, hits[1:]):
        if abs(q - p - 0.025) < 1e-9:
            cur.append(q)
        else:
            runs.append(cur)
            cur = [q]
    runs.append(cur)
    xrun = max(runs, key=len)
    print("\n   %d workable x, longest contiguous run %.3f..%.3f (%d cells)"
          % (len(hits), xrun[0], xrun[-1], len(xrun)))
    if len(runs) > 1:
        print("   NOT ONE RUN: %d runs %s -- the span is the longest, not the "
              "envelope." % (len(runs), [(r[0], r[-1]) for r in runs]))
    ends = {}
    for x in (xrun[0], xrun[-1]):
        ok, c = workable(rig, arm, [round(sgn * x, 4), y_row, z_cube],
                         a.repeats, a.floor)
        ends[x] = dict(ok=bool(ok), clearance=None if c is None else round(c, 4))
        print("   N=%d at x %.3f -> %s clearance %s"
              % (a.repeats, x, "ok" if ok else "NO",
                 "----" if c is None else "%.4f" % c))
    res["stage3_ends_full_n"] = {str(k): v for k, v in ends.items()}
    lo = round(xrun[0] + (0.020 if ends[xrun[0]]["ok"] else 0.045), 4)
    hi = round(xrun[-1] - (0.020 if ends[xrun[-1]]["ok"] else 0.045), 4)
    res["usable_row_x"] = [lo, hi]
    print("   USABLE ROW, 20 mm inside the last N/N cell: x %.4f .. %.4f "
          "(%.3f m)" % (lo, hi, hi - lo))

    # ------------------------------------------------------------- stage 4
    need = 2 * a.plane_w + 4 * cube_m + 5 * a.gap
    w_use = a.plane_w
    print("\nSTAGE 4 -- pack. need %.4f m for 2 pads (%.3f) + 4 cubes + 5 "
          "gaps (%.3f); have %.4f m" % (need, a.plane_w, a.gap, hi - lo))
    if need > hi - lo:
        w_use = round((hi - lo - 4 * cube_m - 5 * a.gap) / 2.0, 4)
        print("   DOES NOT FIT, short by %.1f mm. Widest pad that does: %.4f m"
              % ((need - (hi - lo)) * 1000.0, w_use))
    res["plane_w_used"] = w_use
    res["plane_d_used"] = plane_d
    if w_use <= 2 * cube_m:
        print("   REFUSING: a pad %.4f m wide cannot hold two cubes side by "
              "side (%.3f m needed)." % (w_use, 2 * cube_m + 0.02))
        res["ok"] = False
        json.dump(res, open(a.out, "w"), indent=2)
        rclpy.shutdown()
        return 8
    # Pads INBOARD -- the brief asks for them as near centre as the geometry
    # allows -- then the four cubes outboard of them.
    planes, cubes = [], []
    cur_x = lo + w_use / 2.0
    for _ in range(2):
        planes.append([round(sgn * round(cur_x, 4), 4), y_row])
        cur_x += w_use + a.gap
    cx = cur_x - w_use / 2.0 + cube_m / 2.0
    for _ in range(4):
        cubes.append([round(sgn * round(cx, 4), 4), y_row])
        cx += cube_m + a.gap
    slot_dx = round(min((w_use - cube_m) / 2.0 - 0.010,
                        cube_m / 2.0 + 0.015), 4)
    res["layout"] = dict(cubes=cubes, planes=planes, plane_w=w_use,
                         plane_d=plane_d, slot_dx=slot_dx, y_row=y_row,
                         table_near_y=e_use, table_top=top)
    print("   pads   %s  size %.3f x %.3f, slots at +/-%.4f in X"
          % (planes, w_use, plane_d, slot_dx))
    print("   cubes  %s" % cubes)
    bad = check_layout(arm, e_use, back_use,
                       [[abs(c[0]), c[1]] for c in cubes],
                       [[abs(p[0]), p[1]] for p in planes],
                       (w_use, plane_d), cube_m)
    res["violations"] = bad
    for b in bad:
        print("   VIOLATION: %s" % b)

    # ------------------------------------------------------------- stage 5
    print("\nSTAGE 5 -- every proposed point, N=%d, full pick path, slab at "
          "%.3f / edge %.4f" % (a.repeats, top, e_use))
    pts = ([("cube_%d" % i, [c[0], c[1], z_cube]) for i, c in enumerate(cubes)]
           + [("plane_%d_%s" % (i, nm),
               [round(p[0] + sgn * s, 4), p[1], z_cube])
              for i, p in enumerate(planes)
              for s, nm in ((-slot_dx, "in"), (+slot_dx, "out"))])
    allok = True
    res["stage5"] = {}
    for nm, p in pts:
        ok, c = workable(rig, arm, p, a.repeats, a.floor)
        res["stage5"][nm] = dict(point=p, ok=bool(ok),
                                 clearance=None if c is None else round(c, 4))
        allok = allok and ok
        print("   %-16s %s  %-4s clearance %s"
              % (nm, [round(v, 4) for v in p], "ok" if ok else "FAIL",
                 "  ----" if c is None else "%.4f" % c))
    res["ok"] = bool(allok and not bad)
    print("\n   %s   %d IK calls -> %s"
          % ("LAYOUT VERIFIED" if res["ok"] else "LAYOUT NOT USABLE",
             rig.calls, a.out))
    json.dump(res, open(a.out, "w"), indent=2)
    rclpy.shutdown()
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
