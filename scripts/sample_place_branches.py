#!/usr/bin/env python3
"""HOW BADLY CAN ONE POSE GO? THE CLEARANCE DISTRIBUTION AT A SINGLE WAYPOINT.

    python3 scripts/sample_place_branches.py

WHY A DISTRIBUTION AND NOT A NUMBER. `verify_t1.py` walks T1's whole path at
N=10 and reports the worst clearance it saw. Run three times against the same
layout it returned CLEAN, then 1 breach at 0.1348, then 3 at 0.0801 -- all on
the LEFT arm, all at the two PLACE poses. The pose is fixed; what moves is
which elbow configuration TRAC-IK comes back with, and a 7-DOF arm has a
continuum of them for a given hand pose.

So the question is not "is this pose clear" but "what fraction of the branches
this solver returns are clear, and how bad is the worst one". A layout whose
place pose is clear nine times in ten is a layout that puts the metal 25 mm
inside a person's forearm on the tenth run, and no IK-based check will ever
say so: `/compute_ik` returns `valid` for every one of them, because the SRDF
excludes exactly the pairs a shoulder mount threatens.

`ik_follower_node` does NOT choose the safest branch -- it takes what the
solver gives and REFUSES to publish below `min_clearance_m`, which is 0.05 m
in sim. So a 0.13 m branch is published, and only the study's own 0.150 m
floor calls it a breach.

This samples one pose many times, per candidate (approach heading, pad column),
and reports the whole distribution: worst, median, and the fraction under the
floor. That is what choosing between candidates needs.
"""
import argparse
import json
import os
import sys

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import FK, pad_mid_in_ee, as_msg  # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_place_branches.json")


def sample(rig, arm, xyz, q, n):
    qm = as_msg(q)
    cs, whos = [], {}
    for _ in range(n):
        j = rig.n.solve_arm_joints(arm, list(xyz), qm, avoid=True, tries=6)
        rig.calls += 1
        if j is None:
            cs.append(None)
            continue
        c, who = rig.clearance(arm, j)
        cs.append(None if c is None else float(c))
        if who:
            whos[who] = whos.get(who, 0) + 1
    good = [c for c in cs if c is not None]
    return dict(n=n, solved=len(good), fail=len(cs) - len(good),
                worst=None if not good else round(min(good), 4),
                median=None if not good else round(float(np.median(good)), 4),
                best=None if not good else round(max(good), 4),
                under_floor=sum(1 for c in good if c < CLEAR_FLOOR),
                closest_to=max(whos, key=whos.get) if whos else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=40)
    ap.add_argument("--arm", default="left")
    ap.add_argument("--heads", default="-50,-35,-25")
    ap.add_argument("--columns", default="0.425,0.450,0.500,0.550")
    ap.add_argument("--elev", type=float, default=T1.APPROACH_ELEV_DEG)
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
    home = [n.js.get(k, 0.0) for k in n.names(a.arm)]
    pad_mid, _ = pad_mid_in_ee(fk, a.arm, home)
    rig.set_furniture(True)

    sgn = 1.0 if a.arm == "left" else -1.0
    z_place = round(T1.T1_Z + T1.PLANE_T, 4)
    print("SAMPLING THE PLACE POSE, %s arm, %d draws each, floor %.3f m"
          % (a.arm, a.samples, CLEAR_FLOOR))
    print("   the object sits at z = %.4f (cube on a %.0f mm mat on the "
          "table)\n" % (z_place, T1.PLANE_T * 1000))
    rows = []
    for head in [float(v) for v in a.heads.split(",")]:
        q = GF.q_from_axis(GF.axis_for(a.elev, head if a.arm == "left"
                                       else -head))
        for col in [float(v) for v in a.columns.split(",")]:
            for slot, tag in ((col - T1.SLOT_DX, "inner"),
                              (col + T1.SLOT_DX, "outer")):
                xyz = GF.wrist_for([round(sgn * slot, 4), T1.ROW_Y, z_place],
                                   q, pad_mid)
                r = sample(rig, a.arm, xyz, q, a.samples)
                r.update(head=head, column=col, slot=round(slot, 4), tag=tag)
                rows.append(r)
                print("   head %+05.1f  pad |x| %.3f  %-5s slot %.3f   "
                      "worst %s  median %s  under floor %2d/%d  (%s)"
                      % (head, col, tag, slot,
                         "----" if r["worst"] is None else "%.4f" % r["worst"],
                         "----" if r["median"] is None else "%.4f" % r["median"],
                         r["under_floor"], r["solved"], r["closest_to"]))

    clean = [r for r in rows if r["under_floor"] == 0 and r["fail"] == 0]
    print("\n" + "=" * 72)
    if clean:
        by = {}
        for r in clean:
            by.setdefault((r["head"], r["column"]), []).append(r)
        both = {k: v for k, v in by.items() if len(v) == 2}
        if both:
            k = min(both, key=lambda t: t[1])
            print("CLEAN ON EVERY DRAW, BOTH SLOTS, innermost column: "
                  "heading %+.1f, pad |x| = %.3f" % k)
            for r in both[k]:
                print("   %-5s slot %.3f  worst %.4f  median %.4f"
                      % (r["tag"], r["slot"], r["worst"], r["median"]))
        else:
            print("NO (heading, column) has BOTH slots clean on every draw.")
    else:
        print("NOTHING is clean on every draw. The place pose admits a branch "
              "inside the floor at every heading and column tried.")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(dict(arm=a.arm, samples=a.samples, elev=a.elev,
                   floor=CLEAR_FLOOR, rows=rows, ik_calls=rig.calls),
              open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
