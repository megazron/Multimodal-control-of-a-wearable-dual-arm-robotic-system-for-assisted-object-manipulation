#!/usr/bin/env python3
"""IS THERE **ANY** TABLE THAT PUTS THE WORK IN THE CENTRE, ON THE SURFACE?

    python3 scripts/search_centre_on_surface.py [--full-repeats 10]

THE QUESTION, AND WHY IT IS NOT THE ONE ALREADY ANSWERED. `centre_vs_height`
swept the WORK PLANE above a table fixed at 0.95 and found no height at which
|x| <= 0.10 both solves and clears the wearer. That holds the table still and
lifts the work into the air, which is exactly what the brief does not want.

This sweeps the TABLE instead, and asks for the objects to rest ON it:

    table top height        z_t        how high the surface is
    object distance out     y_obj      how far in front of the wearer
    overhang                o          how far back from the near edge the
                                       object sits -- 0.00 puts it at the very
                                       edge, which is what makes a near-side
                                       approach possible at all
    lateral position        x          0.00 is the centre
    approach                pinned or top-down

"The wearer stands further back" and "the table moves forward" are the same
degree of freedom, so y_obj covers both. A NARROWER table changes only the
half-width, which is scenery: nothing about the approach depends on how far
the table extends sideways past the object, so it is not swept -- it is
reported as free.

TWO STAGES. Stage 1 is the GRASP POSE ALONE at N=1, which this project has
measured to be OPTIMISTIC -- cells that pass it fail the full path, never the
reverse -- so a cell failing here cannot be rescued. Stage 2 walks the whole
pick path (standoff, descend, grasp, lift) at N=--full-repeats with the wearer
measured GEOMETRICALLY, because `avoid_collisions` cannot see the pairs a
shoulder mount threatens.

THE TABLE IS A REAL COLLISION OBJECT AT EVERY CELL. clip_scene's own furniture
is removed first, and a slab is applied at that cell's height and edge, so the
approach cone has to get past the surface the object is resting on. That is
the whole difficulty: the pinned wrist arrives from the near side and BELOW,
through the volume a table top occupies.

CONTROLS, and no report without them:
    a pose inside the torso              clearance must be NEGATIVE
    the SHIPPED layout (x=0.56, objects   must come back reachable and clear
      170 mm above a 0.95 table)          -- a known answer, already committed
    an object BURIED in the slab          must FAIL, or the table is not
                                          actually in the scene
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
import clip_tasks as CT                                      # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR, _apply, _remove  # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee, q_matrix,   # noqa: E402
                                    as_msg, as_tuple, top_down)

OUT = os.path.join(ROOT, "recordings/baselines/centre_on_surface.json")
CUBE = 0.040
THICK = 0.035
HALF_X = 1.05
DEPTH = 0.62
STANDOFF, LIFT = 0.10, 0.08
TABLE_ID = "search_table"


def put_table(rig, top_z, near_y):
    o = CollisionObject()
    o.id = TABLE_ID
    o.header.frame_id = "world"
    o.operation = CollisionObject.ADD
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [2 * HALF_X, DEPTH, THICK]
    p = Pose()
    p.position.x = 0.0
    p.position.y = float(near_y + DEPTH / 2.0)
    p.position.z = float(top_z - THICK / 2.0)
    p.orientation.w = 1.0
    o.primitives = [sp]
    o.primitive_poses = [p]
    _apply(rig.n, rig.cli, [o])


def clear_table(rig):
    _remove(rig.n, rig.cli, [TABLE_ID])


def wrist(obj, q, pad_ee):
    return list(np.asarray(obj, float) - q_matrix(q) @ np.asarray(pad_ee))


def path_for(ee, q, along_axis):
    if along_axis:
        a = q_matrix(q)[:, 2]
        pre = list(np.asarray(ee, float) - STANDOFF * a)
    else:
        pre = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def solve_cell(rig, arm, obj, q, pad_ee, repeats, full, along_axis):
    """(ok, worst_clearance, who). `full` walks the path; else the grasp only."""
    ee = wrist(obj, q, pad_ee)
    wps = path_for(ee, q, along_axis) if full else [ee]
    qm = as_msg(q)
    worst, who = 1e9, None
    for w in wps:
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                return False, None, None
            c, k = rig.clearance(arm, j)
            if c is None:
                return False, None, None
            if c < worst:
                worst, who = c, k
    return True, worst, who


def frange(lo, hi, st):
    out, v = [], lo
    while v <= hi + 1e-9:
        out.append(round(v, 4))
        v += st
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full-repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    for arm in ("left", "right"):
        w, j = n.home_ok(arm)
        if w > HOME_TOL_RAD:
            print("REFUSING: %s arm %.4f rad from home (joint_%d)." % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    fk = FK(n)
    pad_ee = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_ee[arm], _ = pad_mid_in_ee(fk, arm, home)
    anchors = {"pinned": {arm: as_tuple(rig.quat[arm]) for arm in
                          ("left", "right")},
               "top_down": {arm: top_down(0.0) for arm in ("left", "right")}}

    # ------------------------------------------------------------ controls
    ctl, good = {}, True
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22], as_msg(
        anchors["pinned"]["left"]), avoid=False, tries=8)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = None if c_t is None else round(c_t, 4)
    good = good and c_t is not None and c_t < 0.0

    # the shipped layout: cube at x 0.56, 170 mm above a 0.95 table
    rig.set_furniture(False)
    put_table(rig, 0.95, 0.10)
    ok_s, c_s, _ = solve_cell(rig, "left", [0.56, 0.12, 1.12],
                              anchors["pinned"]["left"], pad_ee["left"],
                              1, True, False)
    ctl["shipped_layout_reachable_and_clear"] = "%s / %s" % (
        ok_s, None if c_s is None else round(c_s, 4))
    good = good and ok_s and c_s is not None and c_s >= a.floor

    # an object BURIED in the slab must fail -- proves the table is applied
    ok_b, _, _ = solve_cell(rig, "left", [0.56, 0.30, 0.94],
                            anchors["pinned"]["left"], pad_ee["left"],
                            1, True, False)
    ctl["buried_in_the_slab_fails"] = ok_b
    good = good and not ok_b

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-38s %s" % (k, v))
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    # -------------------------------------------------------------- sweep
    zs = [0.70, 0.80, 0.90, 0.95, 1.00, 1.10]
    ys = frange(0.10, 0.55, 0.075)
    xs = frange(0.0, 0.45, 0.05)
    ohs = [0.00, 0.05]
    print("\nSTAGE 1 -- grasp pose only, N=1, OPTIMISTIC. %d cells per arm "
          "per approach." % (len(zs) * len(ys) * len(xs) * len(ohs)))
    print("   objects REST ON the table: cube centre = table top + %.3f\n"
          % (CUBE / 2.0))

    hits = []
    for approach in ("pinned", "top_down"):
        for z_t in zs:
            for oh in ohs:
                for y_obj in ys:
                    near_y = round(max(0.02, y_obj - CUBE / 2.0 - oh), 4)
                    rig.set_furniture(False)
                    put_table(rig, z_t, near_y)
                    for arm in ("left", "right"):
                        sgn = 1.0 if arm == "left" else -1.0
                        for x in xs:
                            obj = [round(sgn * x, 4), y_obj,
                                   round(z_t + CUBE / 2.0, 4)]
                            ok, c, who = solve_cell(
                                rig, arm, obj, anchors[approach][arm],
                                pad_ee[arm], 1, False,
                                approach == "top_down")
                            if ok and c is not None and c >= a.floor:
                                hits.append(dict(approach=approach, arm=arm,
                                                 table_top=z_t, near_y=near_y,
                                                 overhang=oh, x=round(x, 4),
                                                 y=y_obj, obj=obj,
                                                 clearance=round(c, 4)))
    clear_table(rig)
    print("   %d (approach, arm, table, cell) combinations survive stage 1"
          % len(hits))
    if hits:
        best = min(hits, key=lambda h: h["x"])
        print("   nearest the centreline at stage 1: |x| = %.3f  (%s, %s, "
              "table top %.2f, near edge %.3f)"
              % (best["x"], best["approach"], best["arm"], best["table_top"],
                 best["near_y"]))

    # ------------------------------------------------- stage 2, full path
    # Only the cells at or inside |x| = 0.15, plus the innermost per
    # (approach, arm), because those decide the question.
    cand, seen = [], set()
    for h in sorted(hits, key=lambda d: d["x"]):
        k = (h["approach"], h["arm"])
        if h["x"] <= 0.15 + 1e-9 or k not in seen:
            seen.add(k)
            cand.append(h)
    print("\nSTAGE 2 -- FULL pick path, N=%d, wearer geometric. %d cells."
          % (a.full_repeats, len(cand)))
    confirmed = []
    for h in cand:
        rig.set_furniture(False)
        put_table(rig, h["table_top"], h["near_y"])
        ok, c, who = solve_cell(rig, h["arm"], h["obj"],
                                anchors[h["approach"]][h["arm"]],
                                pad_ee[h["arm"]], a.full_repeats, True,
                                h["approach"] == "top_down")
        h2 = dict(h, full_path_ok=ok,
                  full_clearance=None if c is None else round(c, 4),
                  safe=bool(ok and c is not None and c >= a.floor), to=who)
        confirmed.append(h2)
        print("   %-8s %-5s top %.2f edge %.3f oh %.2f  x=%+.3f y=%.3f  "
              "%-11s clr %s"
              % (h["approach"], h["arm"], h["table_top"], h["near_y"],
                 h["overhang"], h["x"], h["y"],
                 "CONFIRMED" if h2["safe"] else
                 ("reach only" if ok else "unreachable"),
                 "----" if c is None else "%.4f" % c))
    clear_table(rig)

    print("\n" + "=" * 72)
    print("ANSWER")
    print("=" * 72)
    safe = [h for h in confirmed if h["safe"]]
    centre = [h for h in safe if h["x"] <= 0.10 + 1e-9]
    if centre:
        b = min(centre, key=lambda h: h["x"])
        print("   THE CENTRE WORKS ON THE SURFACE: |x| = %.3f, table top "
              "%.2f, near edge %.3f, %s approach, %s arm, clearance %.4f"
              % (b["x"], b["table_top"], b["near_y"], b["approach"],
                 b["arm"], b["full_clearance"]))
    elif safe:
        b = min(safe, key=lambda h: h["x"])
        print("   NO configuration puts |x| <= 0.10 on the surface.")
        print("   CLOSEST ACHIEVABLE TO CENTRE: |x| = %.3f m  (%.0f mm off "
              "centre)" % (b["x"], b["x"] * 1000))
        print("      table top %.2f, near edge %.3f, overhang %.2f, %s "
              "approach, %s arm, clearance %.4f m"
              % (b["table_top"], b["near_y"], b["overhang"], b["approach"],
                 b["arm"], b["full_clearance"]))
        td = [h for h in safe if h["approach"] == "top_down"]
        if td:
            t = min(td, key=lambda h: h["x"])
            print("      nearest with a TOP-DOWN grasp: |x| = %.3f, table top "
                  "%.2f, clearance %.4f" % (t["x"], t["table_top"],
                                            t["full_clearance"]))
        else:
            print("      NO top-down cell survives the full path at any "
                  "table height in %.2f..%.2f." % (min(zs), max(zs)))
    else:
        print("   NOTHING survives the full path anywhere in this sweep.")

    out = dict(zs=zs, ys=ys, xs=xs, overhangs=ohs, floor=a.floor,
               cube_m=CUBE, table_thickness=THICK, table_depth=DEPTH,
               half_width_note="half-width is scenery: nothing about the "
                               "approach depends on how far the table extends "
                               "sideways past the object, so a narrower table "
                               "is free and is not swept",
               full_repeats=a.full_repeats, controls=ctl,
               stage1_hits=hits, stage2=confirmed, ik_calls=rig.calls)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
