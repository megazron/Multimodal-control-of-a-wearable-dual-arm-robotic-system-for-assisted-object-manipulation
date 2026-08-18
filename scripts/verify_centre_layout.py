#!/usr/bin/env python3
"""SCORE ONE PROPOSED CENTRE LAYOUT, POINT BY POINT, ON THE FULL PICK PATH.

    python3 scripts/verify_centre_layout.py --self-test
    python3 scripts/verify_centre_layout.py --layout t1

WHAT THIS IS FOR. The column sweeps answer "how far in can this arm work",
which is a boundary. A LAYOUT is a specific list of points -- two pad slots per
pad and one pose per cube -- and a boundary does not certify a list: the slots
sit at different x AND the cubes sit further out, and every one of them has to
walk its own pre-grasp, grasp and lift with the wearer and the table in the
scene.

So this takes the points, not the boundary, and scores each with
`measure_centre_gap.classify_cell` at N repeats -- the same scorer, the same
capsule wearer model, the same 150 mm floor measured geometrically because
`avoid_collisions` cannot see the pairs a shoulder mount threatens.

`--layout t1` reads the layout out of `t1_task`, so what is certified is what
the task file says and not a copy of it that can drift.

CONTROLS, and no report without them: the same four the column sweeps run,
plus the link namer, imported rather than reimplemented.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee,        # noqa: E402
                                    as_msg, as_tuple)
from measure_centre_gap import classify_cell, clearance_named, OK  # noqa: E402
import search_centre_geometry as SCG                          # noqa: E402
from search_t1_centre import controls                         # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_layout.json")


def points_from_t1():
    """(label, arm, [x, y, z]) for every pose the layout asks an arm to reach.

    THE SLOTS, NOT THE PAD CENTRE. A pad is 100 mm wide and two cubes land on
    it at +/- SLOT_DX, so the pad centre is a poor proxy for the pose that has
    to work -- it is the INNER slot that decides whether the layout stands.
    """
    z = round(T1.TABLE_TOP + T1.CUBE_M / 2.0, 4)
    row = round(T1.TABLE_NEAR_Y + T1.CUBE_M / 2.0, 4)
    pts = []
    for i, (px, _py) in enumerate(T1.T1_PLANES):
        arm = "left" if i == 0 else "right"
        for s, tag in ((-T1.SLOT_DX, "inner" if i == 0 else "outer"),
                       (+T1.SLOT_DX, "outer" if i == 0 else "inner")):
            pts.append(("pad %d (%s) %s slot" % (i, arm, tag), arm,
                        [round(px + s, 4), row, z]))
    for i, (cx, cy) in enumerate(T1.T1_CUBES):
        arm = "left" if cx > 0 else "right"
        pts.append(("cube %d" % i, arm, [round(cx, 4), round(cy, 4), z]))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--layout", default="t1")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--tag", default="")
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
            print("REFUSING: %s arm %.4f rad from home (joint_%d)."
                  % (arm, w, j))
            return 3
    rig = Rig(n, "t1", 1)
    fk = FK(n)
    fkc = fk.cli
    pad_mid = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_mid[arm], _ = pad_mid_in_ee(fk, arm, home)

    SCG.TABLE["half_x"], SCG.TABLE["depth"] = 1.05, 0.62
    ctl, _ = controls(rig, pad_mid, a.floor)
    ql = as_msg(as_tuple(rig.quat["left"]))
    c = who = seg = None
    for pt in SCG_TORSO:
        jt = n.solve_arm_joints("left", list(pt), ql, avoid=False, tries=8)
        if jt is None:
            continue
        c, who, seg, _p = clearance_named(fkc, n, "left", jt)
        if c is not None and c < 0.0:
            ctl["inside_torso_is_negative"] = "%.4f at %s" % (c, pt)
            break
    log("CONTROLS (tag %s)" % (a.tag or "-"))
    for k, v in ctl.items():
        log("   %-38s %s" % (k, v))
    log("   %-38s %s / %s / %s" % ("link namer inside the torso",
                                   None if c is None else round(c, 4),
                                   who, seg))
    good = (ctl["far_1.6m_unreachable"]
            and "True -> False" in str(ctl["slab_control_free_then_buried"])
            and c is not None and c < 0 and who and seg)
    if not good:
        log("\nREFUSING: a control failed.")
        return 6
    if a.self_test:
        log("\nself-test only.")
        return 0

    SCG.scene(rig, T1.TABLE_TOP, T1.TABLE_NEAR_Y)
    log("\n" + "=" * 96)
    log("LAYOUT: table top %.3f, near edge %.3f, approach elev %+.0f / "
        "heading %+.0f inboard, N=%d, floor %.3f"
        % (T1.TABLE_TOP, T1.TABLE_NEAR_Y, T1.APPROACH_ELEV_DEG,
           -T1.APPROACH_HEAD_DEG, a.repeats, a.floor))
    log("=" * 96)
    rows, allok = [], True
    for label, arm, p in points_from_t1():
        r = classify_cell(rig, fkc, arm, p, T1.APPROACH[arm], pad_mid[arm],
                          a.repeats, a.floor, True)
        SCG.scene(rig, T1.TABLE_TOP, T1.TABLE_NEAR_Y)
        r.update(label=label, arm=arm, point=p)
        rows.append(r)
        allok = allok and r["verdict"] == OK
        log("  %-28s %-5s x %+7.3f  %-18s %s"
            % (label, arm, p[0], r["verdict"],
               ("clearance %.4f m, closest %s to %s"
                % (r.get("clearance", float("nan")), r.get("link"),
                   r.get("wearer"))) if r["verdict"] == OK
               else (r.get("detail") or "")[:70]))
    log("\n%s" % ("EVERY POSE IN THE LAYOUT PASSES" if allok
                  else "THE LAYOUT DOES NOT STAND -- see the rows above"))
    res = dict(tag=a.tag, ok=allok, repeats=a.repeats, floor=a.floor,
               table=dict(top=T1.TABLE_TOP, near_y=T1.TABLE_NEAR_Y),
               approach=dict(elev=T1.APPROACH_ELEV_DEG,
                             head_inboard=-T1.APPROACH_HEAD_DEG),
               planes=T1.T1_PLANES, cubes=T1.T1_CUBES, rows=rows, log=lines)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    log("-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if allok else 1


# the same constructed torso-interior candidates the column sweep uses; a
# control that depends on one mount is not a control (see search_centre_geometry)
SCG_TORSO = [(0.05, -0.02, 1.22), (0.10, 0.0, 1.25), (0.15, 0.0, 1.25),
             (0.15, 0.05, 1.35), (0.15, -0.05, 1.10), (0.10, 0.05, 1.40),
             (0.16, 0.08, 1.30)]

if __name__ == "__main__":
    sys.exit(main())
