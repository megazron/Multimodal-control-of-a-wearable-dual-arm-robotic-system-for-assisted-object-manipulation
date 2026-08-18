#!/usr/bin/env python3
"""CAN THE CENTRE OF THE TABLE BE MADE REACHABLE BY BOTH ARMS? PER LEVER.

    python3 scripts/search_centre_geometry.py --self-test
    python3 scripts/search_centre_geometry.py --stand      # (a) wearer back
    python3 scripts/search_centre_geometry.py --width      # (b) narrower table
    python3 scripts/search_centre_geometry.py --rotate     # (c) short end
    python3 scripts/search_centre_geometry.py --cells "0.95,0.38,1.05,0.62"

THE QUESTION, STATED ONCE SO EVERY LEVER IS SCORED THE SAME WAY

    Can BOTH arms work at |x| <= 0.10 -- the table's own centreline, directly
    in front of the person -- with the whole pick path walked at N repeats,
    the wearer and the table in the scene, and the 150 mm clearance floor
    held GEOMETRICALLY rather than by /compute_ik?

`measure_centre_gap.classify_cell` is the scorer and it is not reimplemented
here: it walks pre-grasp -> grasp -> lift, solves each waypoint N times, reads
the mount guard's own capsule model on every solution, and names the blocking
link and wearer part. This file only decides WHICH SCENES it is pointed at.

WHAT IS SWEPT, AND WHY EACH ONE IS A DIFFERENT QUANTITY

    (a) standing distance   the table's NEAR EDGE moves away from the wearer.
                            The row the objects stand on moves with it, so
                            this trades torso clearance against forward reach
                            and the two bound it from opposite sides.
    (b) table width         the slab's HALF-WIDTH. The centre column is over
                            slab at every width, so this can only act on the
                            arm's outboard routing, which is the point of
                            measuring it rather than asserting it.
    (c) rotation            the wearer at the short end: half-width 0.31,
                            depth 2.10. Geometrically this is (b) at a narrow
                            width plus depth that no arm ever reaches, and the
                            sweep says so rather than the argument.

THE TABLE IS THE ONLY FURNITURE, AND THAT IS DELIBERATE. `Rig.set_furniture`
re-applies clip_scene's SHIPPED table, which is a different slab at a fixed
width; leaving it in while a candidate table is under test puts two tables in
the scene. Every cell here runs with the shipped furniture OUT and exactly one
parameterised slab in.

A DEFECT IN THE SCORER THIS FILE WORKS AROUND, AND NAMES.  `measure_centre_
gap._why` ends with `rig.set_furniture(True)`, which puts the SHIPPED
furniture back and leaves it there for every later cell in the sweep. It is
invisible in a run that never fails a cell and it silently changes the scene
in a run that does. This file restores the scene after every cell and asserts
the restore with a control.

CONTROLS, and no report is printed if one fails:

    inside the torso reads NEGATIVE          the capsule model is live
    1.6 m out is unreachable                 IK is not answering yes to
                                             everything
    the shipped cube is reachable and clear   the rig agrees with the
                                             layout that is known to work
    a slab through a good pose blocks it      the table is really in the scene
    the link namer names a link               a blocked column will not read
                                             "unknown"
    the scene is restored after a failed cell  the _why defect above
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from geometry_msgs.msg import Pose                            # noqa: E402
from moveit_msgs.msg import CollisionObject                   # noqa: E402
from shape_msgs.msg import SolidPrimitive                     # noqa: E402

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR, _apply, _remove  # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee,        # noqa: E402
                                    as_msg, as_tuple)
import measure_centre_gap as MCG                              # noqa: E402
from measure_centre_gap import classify_cell, clearance_named, OK  # noqa: E402
import search_t1_centre as STC                                # noqa: E402
from search_t1_centre import controls, frange, CUBE, TABLE_ID  # noqa: E402
from srl_teleop import wearer_posture as WP                   # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_geometry.json")

# The table under test. ONE dict, read by put_table and by nothing else, so a
# lever cannot move a dimension the slab does not follow.
TABLE = dict(top=0.950, near_y=0.280, half_x=1.05, depth=0.62, thick=0.035)

# What "the centre" means, as a number. Two 100 mm pads side by side straddling
# the centreline put each pad's own centre at |x| = 0.055 and its inner slot at
# |x| = 0.025, so an arm that reaches |x| <= 0.10 can work the centre pair and
# an arm that cannot, cannot.
CENTRE_X = 0.10


def put_table(rig, top_z, near_y):
    """The slab, at whatever TABLE currently says. Signature matches the one
    `measure_centre_gap._why` calls, because this replaces it in that module."""
    o = CollisionObject()
    o.id = TABLE_ID
    o.header.frame_id = "world"
    o.operation = CollisionObject.ADD
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [2 * TABLE["half_x"], TABLE["depth"], TABLE["thick"]]
    p = Pose()
    p.position.x = 0.0
    p.position.y = float(near_y + TABLE["depth"] / 2.0)
    p.position.z = float(top_z - TABLE["thick"] / 2.0)
    p.orientation.w = 1.0
    o.primitives = [sp]
    o.primitive_poses = [p]
    _apply(rig.n, rig.cli, [o])


def clear_table(rig):
    _remove(rig.n, rig.cli, [TABLE_ID])


# THE SCORER CALLS THESE BY NAME. Patched in both modules so a slab restored
# after a failed cell is the CANDIDATE slab and not the 1.05 m default.
MCG.put_table = put_table
MCG.clear_table = clear_table
STC.put_table = put_table
STC.clear_table = clear_table


def scene(rig, top, near_y):
    """Exactly one slab, the shipped furniture out. Idempotent."""
    rig.set_furniture(False)
    put_table(rig, top, near_y)
    rig._slab_on = {TABLE_ID}
    MCG._why.top, MCG._why.near_y = top, near_y


def innermost(rig, fkc, arm, q, pad_mid, top, near_y, repeats, floor,
              x_max, x_step, log):
    """Walk outward from the centreline; stop at the first column that passes.

    Returns (x or None, the row that decided it, every row walked).
    """
    sgn = 1.0 if arm == "left" else -1.0
    y = round(near_y + CUBE / 2.0, 4)
    z = round(top + CUBE / 2.0, 4)
    rows = []
    for x in frange(0.0, x_max, x_step):
        obj = [round(sgn * x, 4), y, z]
        r = classify_cell(rig, fkc, arm, obj, q, pad_mid, repeats, floor, True)
        r.update(arm=arm, x=x, obj=obj)
        rows.append(r)
        scene(rig, top, near_y)          # the _why defect, undone every cell
        if log:
            log("      |x| %.3f  %-18s %s"
                % (x, r["verdict"], (r.get("detail") or "clearance %.4f m, %s"
                                     % (r.get("clearance", float("nan")),
                                        r.get("link")))[:88]))
        if r["verdict"] == OK:
            return x, r, rows
    return None, None, rows


def config(rig, fkc, pad_mid, name, top, near_y, half_x, depth, q_of,
           repeats, floor, x_max, x_step, log, verbose=False):
    """One geometry. Both arms. Returns the row that goes in the table."""
    TABLE["half_x"], TABLE["depth"] = half_x, depth
    TABLE["top"], TABLE["near_y"] = top, near_y
    scene(rig, top, near_y)
    out = dict(name=name, top=top, near_y=near_y, half_x=half_x, depth=depth)
    for arm in ("left", "right"):
        x, r, rows = innermost(rig, fkc, arm, q_of(arm), pad_mid[arm], top,
                               near_y, repeats, floor, x_max, x_step,
                               log if verbose else None)
        out[arm] = dict(
            innermost=x,
            clearance=None if r is None else r.get("clearance"),
            wearer=None if r is None else r.get("wearer"),
            link=None if r is None else r.get("link"),
            # what stopped the CENTRE column itself, always reported
            centre=dict(verdict=rows[0]["verdict"],
                        detail=(rows[0].get("detail") or "")[:140],
                        wearer=rows[0].get("wearer"),
                        link=rows[0].get("link"),
                        clearance=rows[0].get("clearance")))
        out["%s_rows" % arm] = [
            dict(x=w["x"], verdict=w["verdict"], clearance=w.get("clearance"),
                 wearer=w.get("wearer"), link=w.get("link"))
            for w in rows]
    both = [out[a]["innermost"] for a in ("left", "right")]
    out["both_reach_centre"] = all(v is not None and v <= CENTRE_X + 1e-9
                                   for v in both)
    out["worst_innermost"] = None if any(v is None for v in both) \
        else max(both)
    log("  %-34s  left %-6s  right %-6s   %s"
        % (name,
           "----" if both[0] is None else "%.3f" % both[0],
           "----" if both[1] is None else "%.3f" % both[1],
           "CENTRE REACHABLE" if out["both_reach_centre"] else
           "centre blocked: L %s / R %s"
           % (out["left"]["centre"]["verdict"],
              out["right"]["centre"]["verdict"])))
    return out


def approach_of(name, elev, head):
    """The wrist orientation each column is solved at, mirrored per arm."""
    if name == "t1":
        return lambda arm: T1.APPROACH[arm]
    def q(arm):
        sgn = -1.0 if arm == "left" else 1.0
        return GF.q_from_axis(GF.axis_for(elev, sgn * head), 0.0)
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-step", type=float, default=0.025)
    ap.add_argument("--top", type=float, default=0.950)
    ap.add_argument("--near-y", type=float, default=0.280)
    ap.add_argument("--half-x", type=float, default=1.05)
    ap.add_argument("--depth", type=float, default=0.62)
    ap.add_argument("--approach", default="t1",
                    help="t1, or head:<deg> for a swept heading")
    ap.add_argument("--elev", type=float, default=T1.APPROACH_ELEV_DEG)
    ap.add_argument("--stand", action="store_true")
    ap.add_argument("--width", action="store_true")
    ap.add_argument("--rotate", action="store_true")
    ap.add_argument("--heads", default="")
    ap.add_argument("--cells", default="",
                    help="explicit 'top,near_y,half_x,depth' configs, ';'-sep")
    ap.add_argument("--tag", default="")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

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
    fkc = fk.cli
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    TABLE["half_x"], TABLE["depth"] = a.half_x, a.depth
    ctl, good = controls(rig, pad_mid, a.floor)
    # THE TORSO CONTROL HAS TO FIND A POINT THIS ARM CAN REACH, and the
    # imported one does not. `search_t1_centre.controls` drives the left arm to
    # the single point (0.05, -0.02, 1.22) and asserts the capsule model reads
    # negative there. That is a fine control for the as-built mount and it
    # FAILS THE MOMENT THE MOUNT MOVES: with the bases 150 mm further outboard
    # the arm cannot reach that far inboard at all, `solve_arm_joints` returns
    # None, and the control reports None -- which is indistinguishable from a
    # dead capsule model.
    #
    # So the control keeps its ground truth and drops its accidental
    # dependency on one mount: every candidate point below is INSIDE the torso
    # box by construction (|x| < 0.18, |y| < 0.11, 0.98 < z < 1.46, from
    # wearer_posture.TORSO_PARTS), and the first one the arm can reach is the
    # one used. A run where none of them is reachable still refuses.
    TORSO_INSIDE = [(0.05, -0.02, 1.22), (0.10, 0.0, 1.25), (0.15, 0.0, 1.25),
                    (0.15, 0.05, 1.35), (0.15, -0.05, 1.10),
                    (0.10, 0.05, 1.40), (0.16, 0.08, 1.30)]
    ql = as_msg(as_tuple(rig.quat["left"]))
    c = who = seg = None
    for pt in TORSO_INSIDE:
        jt = n.solve_arm_joints("left", list(pt), ql, avoid=False, tries=8)
        if jt is None:
            continue
        c, who, seg, _p = clearance_named(fkc, n, "left", jt)
        if c is not None and c < 0.0:
            ctl["inside_torso_is_negative"] = "%.4f at %s" % (c, pt)
            break
    log("CONTROLS   (wearer posture in the URDF: %s; tag %s)"
        % (WP.posture_from_env(), a.tag or "-"))
    for k, v in ctl.items():
        log("   %-38s %s" % (k, v))
    log("   %-38s %s / %s / %s"
        % ("link namer inside the torso", None if c is None else round(c, 4),
           who, seg))
    # `good` is recomputed rather than inherited: the imported helper folded
    # its own torso reading into it, and that reading is the one just replaced.
    good = (ctl["far_1.6m_unreachable"]
            and "True" in str(ctl["shipped_cube_reachable_and_clear"])
            and "True -> False" in str(ctl["slab_control_free_then_buried"])
            and c is not None and c < 0 and who and seg)

    # THE RESTORE CONTROL. Fail a cell on purpose (the centre column, which is
    # known blocked as built), then assert that the slab under test is still
    # the only table in the scene and that a pose which was reachable before
    # is reachable after. Without this the _why defect is invisible.
    scene(rig, a.top, a.near_y)
    before = n.solve_arm_joints("left", [0.560, 0.300, 1.000],
                                as_msg(T1.APPROACH["left"]), avoid=True,
                                tries=6) is not None
    classify_cell(rig, fkc, "left", [0.0, round(a.near_y + CUBE / 2.0, 4),
                                     round(a.top + CUBE / 2.0, 4)],
                  T1.APPROACH["left"], pad_mid["left"], 1, a.floor, True)
    scene(rig, a.top, a.near_y)
    after = n.solve_arm_joints("left", [0.560, 0.300, 1.000],
                               as_msg(T1.APPROACH["left"]), avoid=True,
                               tries=6) is not None
    ctl["scene_restored_after_a_failed_cell"] = "%s -> %s" % (before, after)
    log("   %-38s %s -> %s" % ("scene restored after a failed cell",
                               before, after))
    good = good and before and after
    if not good:
        log("\nREFUSING: a control failed.")
        return 6
    if a.self_test:
        log("\nself-test only.")
        return 0

    res = dict(tag=a.tag, posture=WP.posture_from_env(), floor=a.floor,
               repeats=a.repeats, centre_x=CENTRE_X, approach=a.approach,
               elev=a.elev, configs=[])

    heads = [float(v) for v in a.heads.split(",") if v.strip()]
    approaches = [("t1 approach (%+.0f elev, %+.0f inboard)"
                   % (T1.APPROACH_ELEV_DEG, -T1.APPROACH_HEAD_DEG),
                   approach_of("t1", 0, 0))] if a.approach == "t1" else []
    for h in heads:
        approaches.append(("heading %+.0f inboard, elev %+.0f" % (h, a.elev),
                           approach_of("head", a.elev, h)))
    if not approaches:
        approaches = [("t1 approach", approach_of("t1", 0, 0))]

    todo = []
    if a.stand:
        for e in frange(0.180, 0.680, 0.050):
            todo.append(("(a) near edge %.3f" % e, a.top, e, a.half_x,
                         a.depth))
    if a.width:
        for hx in (0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.70, 1.05):
            todo.append(("(b) half width %.2f" % hx, a.top, a.near_y, hx,
                         a.depth))
    if a.rotate:
        # the wearer at the SHORT end: the shipped 2.10 x 0.62 table turned
        # 90 deg, plus the same turn on a smaller table so "rotated" is not
        # confounded with "enormous".
        todo.append(("(c) rotated 2.10 deep x 0.62 wide", a.top, a.near_y,
                     0.31, 2.10))
        todo.append(("(c) rotated 1.20 deep x 0.62 wide", a.top, a.near_y,
                     0.31, 1.20))
        todo.append(("(c) rotated 1.20 deep x 0.80 wide", a.top, a.near_y,
                     0.40, 1.20))
    for spec in [s for s in a.cells.split(";") if s.strip()]:
        t, e, hx, d = [float(v) for v in spec.split(",")]
        todo.append(("cell %.3f/%.3f/%.2f/%.2f" % (t, e, hx, d), t, e, hx, d))
    if not todo:
        todo = [("as built", a.top, a.near_y, a.half_x, a.depth)]

    t0 = time.time()
    for aname, qof in approaches:
        log("\n" + "=" * 74)
        log("%s   -- innermost REACHABLE |x|, full pick path, N=%d, floor "
            "%.3f m" % (aname, a.repeats, a.floor))
        log("=" * 74)
        for name, top, e, hx, d in todo:
            row = config(rig, fkc, pad_mid, name, top, e, hx, d, qof,
                         a.repeats, a.floor, a.x_max, a.x_step, log)
            row["approach"] = aname
            res["configs"].append(row)
    clear_table(rig)

    res["log"] = lines
    res["ik_calls"] = rig.calls
    res["seconds"] = round(time.time() - t0, 1)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    log("\n%d IK calls in %.0f s -> %s" % (rig.calls, time.time() - t0, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
