#!/usr/bin/env python3
"""HOW FAR INBOARD CAN EACH ARM WORK, at every forward distance?

    python3 scripts/measure_centre_reach.py [--repeats 3]

THE QUESTION THIS ANSWERS. "Put the coloured planes in the CENTRE of the
table, directly in front of the person." This repository has twice recorded
that there are zero reachable cells with |x| <= 0.10, and both times the
number came from a survey taken at ONE forward distance or with the bench in
the scene. Neither settles it, because the thing that closes the centre is
the WEARER, and the wearer's own arms hang at y = 0 -- so reaching inboard
300 mm in FRONT of them is a different question from reaching inboard beside
them, and nobody had asked it.

SO IT SCANS INBOARD AT EVERY y, not at one. For each forward distance, walk x
from outboard toward the centreline until the arm can no longer be commanded
there, and report the innermost x that worked.

IT IS A ONE-SIDED BOUND ON PURPOSE, and that is what makes it cheap enough to
run. The scan tests the GRASP POSE ALONE, which this project has measured to
be OPTIMISTIC -- cells that pass it fail the full path, never the reverse
(`_region()` refuses a grasp-pose-only survey for exactly that reason). So an
innermost x from this scan is a LOWER BOUND ON |x|: the full path cannot do
better. If the scan says the nearest workable column is 250 mm off the
centreline, then the centre is out, and no full-path run can rescue it.
Candidates that DO look feasible are then re-tested over the whole place path
at the caller's N before anything is built on them.

CONTROLS. The same four the binding ladder uses, via measure_what_binds.Rig,
plus one specific to this question: the OUTERMOST column of the scan must
pass for both arms, because a scan that fails everywhere and a scan that
never ran produce the same map of zeros.
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
from measure_what_binds import Rig, controls                 # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_reach.json")
Z_WORK = CT.BENCH_TOP + 0.02
STANDOFF, LIFT = 0.10, 0.08


def place_path(ee):
    """The place leg of T1: arrive high, descend, release, withdraw."""
    hi = [ee[0], ee[1], ee[2] + LIFT]
    up = [ee[0], ee[1], ee[2] + STANDOFF]
    return densify([hi, ee], 0.03) + densify([ee, up], 0.03)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--full-repeats", type=int, default=10)
    ap.add_argument("--z", type=float, default=Z_WORK)
    ap.add_argument("--step", type=float, default=0.025)
    ap.add_argument("--x-out", type=float, default=0.55)
    ap.add_argument("--y", type=float, nargs=2, default=(0.10, 0.55))
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
    rig = Rig(n, "t1", a.repeats)
    seed = {"left": [0.400, 0.175, a.z], "right": [-0.400, 0.175, a.z]}
    ctl, ok = controls(rig, seed)
    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-20s want %-10s got %-11s" % (k, v["want"], v["got"]))
    if not ok:
        print("\nREFUSING TO REPORT: a control failed.")
        return 6

    ys = []
    y = a.y[0]
    while y <= a.y[1] + 1e-9:
        ys.append(round(y, 4))
        y += a.step

    print("\nINNERMOST WORKABLE COLUMN, per forward distance, z=%.3f, N=%d"
          % (a.z, a.repeats))
    print("   (grasp pose only -- an OPTIMISTIC bound; the full path is "
          "never better)")
    rows = {}
    outer_ok = {"left": False, "right": False}
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        rows[arm] = {}
        for yy in ys:
            # SCAN THE WHOLE ROW. The first version walked inward and STOPPED
            # at the first column that failed, which quietly assumed both
            # that the outermost column always works and that the reachable
            # set is an interval in x. Neither holds: the left arm's row at
            # y = 0.30 fails at x = 0.55 and succeeds at 0.25..0.50, so the
            # scan reported "nothing at this y" for a row with eleven working
            # columns in it. A reachable set this rig has already measured to
            # fill 62% of its own bounding box is not something to walk into
            # and stop at the first gap.
            hits = []
            x = a.x_out
            while x >= -0.05:
                p = CT.ee_for([sgn * x, yy, a.z], arm)
                if rig.solve(arm, p):
                    hits.append(round(sgn * x, 4))
                x -= a.step
            if hits:
                outer_ok[arm] = True
            inner = min(hits, key=abs) if hits else None
            rows[arm]["%.3f" % yy] = inner
        print("   %-5s %s" % (arm.upper(), " ".join(
            "y%.2f:%s" % (float(k), "----" if v is None else "%+.3f" % v)
            for k, v in rows[arm].items())))

    if not (outer_ok["left"] and outer_ok["right"]):
        print("\nREFUSING TO REPORT: the outermost column failed for an arm "
              "-- a map of zeros and a loop that never ran look identical.")
        json.dump(dict(refused="outer column control failed"),
                  open(a.out, "w"), indent=2)
        return 7

    # THE BEST CASE, AND WHAT STOPS IT. Take the innermost column any arm
    # reached at any y, and classify the next 25 mm inboard of it.
    best = {}
    for arm in ("left", "right"):
        vals = [v for v in rows[arm].values() if v is not None]
        best[arm] = min(abs(v) for v in vals) if vals else None
    print("\n   nearest the centreline either arm gets:  left %s   right %s"
          % ("----" if best["left"] is None else "%.3f m" % best["left"],
             "----" if best["right"] is None else "%.3f m" % best["right"]))

    verdicts = {}
    for arm in ("left", "right"):
        if best[arm] is None:
            continue
        sgn = 1.0 if arm == "left" else -1.0
        yy = None
        for k, v in rows[arm].items():
            if v is not None and abs(v) == best[arm]:
                yy = float(k)
                break
        p = CT.ee_for([sgn * (best[arm] - a.step), yy, a.z], arm)
        v, why, c, who = rig.classify(arm, p)
        verdicts[arm] = dict(y=yy, x=round(sgn * (best[arm] - a.step), 4),
                             binds=v, detail=why,
                             clearance_m=None if c is None else round(c, 4),
                             clearance_to=who)
        print("   one step further in (%s x=%+.3f y=%.3f): BINDS %s -- %s"
              % (arm, sgn * (best[arm] - a.step), yy, v, why))

    # AND THE CENTRELINE ITSELF, ASKED DIRECTLY. A bound is an argument; this
    # is the measurement the brief actually asked for.
    centre = {}
    print("\n   THE CENTRELINE ITSELF, asked directly (x = 0.00 and +/-0.10):")
    for arm in ("left", "right"):
        centre[arm] = {}
        for cx in (0.0, 0.10, -0.10):
            hits = []
            for yy in ys:
                p = CT.ee_for([cx, yy, a.z], arm)
                if rig.solve(arm, p):
                    hits.append(yy)
            centre[arm]["%+.2f" % cx] = hits
            print("      %-5s x=%+.2f  %s" % (
                arm, cx, ("reachable at y " + str(hits)) if hits
                else "NO y in %.2f..%.2f" % (a.y[0], a.y[1])))

    res = dict(z=a.z, step=a.step, repeats=a.repeats, controls=ctl,
               innermost=rows, best=best, one_step_further=verdicts,
               centreline=centre, ik_calls=rig.calls,
               method="grasp pose only -- an optimistic bound on |x|")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
