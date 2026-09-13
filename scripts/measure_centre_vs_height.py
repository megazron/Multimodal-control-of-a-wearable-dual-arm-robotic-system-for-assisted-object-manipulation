#!/usr/bin/env python3
"""IS THE FRONT CENTRE REACHABLE AT ANY WORK HEIGHT? Asked properly.

    python3 scripts/measure_centre_vs_height.py [--full-repeats 10]

THE QUESTION. The brief asks for the two coloured pads in the CENTRE of the
table, directly in front of the person. `centre_reach.json` says no -- no y
from 0.10 to 0.55 at x = 0.00 for either arm -- but it was measured at ONE
work height, z = 1.120, and `band_vs_work_height.json` shows forward reach
climbing from 0.025 m to 0.500 m as the work plane rises above the table. A
result taken at one height cannot answer a question about every height, and
the two measurements have never been crossed.

So this crosses them: for each work-plane height, how near the centreline can
each arm work, and does the centreline itself ever open up?

TWO STAGES, AND THE FIRST ONE IS DELIBERATELY OPTIMISTIC.

  Stage 1  GRASP POSE ONLY, N=1, no furniture removed, geometric clearance
           computed on the returned solution. This project has measured that
           grasp-pose-only is optimistic -- cells that pass it fail the full
           path, never the reverse -- so a cell that FAILS here cannot be
           rescued by anything downstream. It is cheap enough to sweep nine
           columns by nineteen rows by eleven heights.

  Stage 2  Only for cells that survive stage 1: the WHOLE pick-and-place path
           (standoff, descend, grasp, lift) at N=`--full-repeats`, furniture
           in the scene, and the wearer clearance measured GEOMETRICALLY on
           every solution -- `avoid_collisions` cannot see the wearer, because
           the SRDF excludes the pairs a shoulder mount threatens.

WHY CLEARANCE IS COMPUTED IN STAGE 1 TOO. "Reachable" and "safe" are different
boundaries and this repository has quoted the first while meaning the second.
A column that solves IK at 40 mm from the wearer is not a column the pads can
go in, and saying so at stage 1 costs one FK call.

CONTROLS, and there is no report without them:

    a pose inside the torso              clearance must be NEGATIVE
    the home pose                        must clear the floor
    a cell 0.60 m outboard, work height  must be reachable AND clear
    the outermost column of the scan     must be reachable for both arms
                                         (a map of zeros and a loop that never
                                          ran are the same picture)
    clearance must FALL as x -> 0        a constant is a failed control, not
                                         a finding
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
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_vs_height.json")
STANDOFF, LIFT = 0.10, 0.08
# The work plane T1 runs on today. Everything is reported relative to it.
Z_NOW = CT.BENCH_TOP + 0.02
TABLE_TOP = 0.95


def path_for(ee):
    """The pick leg T1 commands: arrive high, descend to the grasp, lift."""
    pre = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def frange(lo, hi, st):
    out, v = [], lo
    while v <= hi + 1e-9:
        out.append(round(v, 4))
        v += st
    return out


def _wrist(rig, arm, obj):
    """WRIST pose putting the pads on `obj` at whatever orientation is set.

    At the anchor this is `ee_for()`. Under any other orientation the pad
    offset must be ROTATED, because `PAD_OFFSET_BY_ARM` is a world vector
    measured AT THE ANCHOR and is only that vector there -- a top-down grasp
    built with ee_for() puts the wrist where the near-side pads would have
    been and the fingers nowhere near the object.
    """
    if rig.pad_ee is None:
        return CT.ee_for(obj, arm)
    import numpy as _np
    return list(_np.asarray(obj, float)
                - rig.pad_rot[arm] @ _np.asarray(rig.pad_ee[arm]))


def grasp_probe(rig, arm, obj):
    """(solved, clearance_m, who) for the GRASP POSE ALONE. Optimistic."""
    j = rig.solve_joints(arm, _wrist(rig, arm, obj), avoid=True)
    if j is None:
        return False, None, None
    c, who = rig.clearance(arm, j)
    return True, c, who


def full_path(rig, arm, obj, repeats):
    """(all waypoints solved, worst clearance over every solution, who)."""
    worst, who = 1e9, None
    for w in path_for(_wrist(rig, arm, obj)):
        for _ in range(repeats):
            j = rig.solve_joints(arm, w, avoid=True)
            if j is None:
                return False, None, None
            c, k = rig.clearance(arm, j)
            if c is None:
                return False, None, None
            if c < worst:
                worst, who = c, k
    return True, worst, who


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full-repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=0.025)
    ap.add_argument("--x-max", type=float, default=0.30,
                    help="scan |x| from 0 out to this, plus the outer control")
    ap.add_argument("--x-control", type=float, default=0.60)
    ap.add_argument("--y", type=float, nargs=2, default=(0.10, 0.55))
    ap.add_argument("--z", type=float, nargs="*", default=None)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--orientation", default="pinned",
                    choices=["pinned", "top_down"],
                    help="pinned is what every mode commands; top_down asks "
                         "whether unpinning the wrist opens the centre, since "
                         "what binds inboard is the WEARER and the whole arm "
                         "chain moves when the wrist does")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    # From the current work plane up past the top of the wearer's torso
    # (the box ends at z = 1.46) and the upper arms (1.43). Above that the
    # neck and head take over and they are much narrower, so if the centre
    # ever opens it opens there -- and the sweep has to go high enough to say
    # so rather than stopping at the table and inferring.
    zs = a.z if a.z else [1.12, 1.15, 1.20, 1.25, 1.30, 1.35,
                          1.40, 1.45, 1.50, 1.55]

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
    rig.pad_ee = None
    rig.pad_rot = None
    if a.orientation == "top_down":
        # Read the pad offset in the END EFFECTOR frame, where it is a
        # constant of the hardware, and rotate it by the approach being asked
        # about. Reusing the anchor's world-frame PAD_OFFSET here would put
        # the fingers somewhere else entirely.
        import numpy as _np
        from measure_grasp_approach import (FK as _FK, top_down as _td,
                                            pad_mid_in_ee as _pad,
                                            q_matrix as _qm, as_msg as _asm)
        _fk = _FK(n)
        rig.pad_ee, rig.pad_rot = {}, {}
        for arm in ("left", "right"):
            home_q = [n.js.get(k, 0.0) for k in n.names(arm)]
            v, _ = _pad(_fk, arm, home_q)
            rig.pad_ee[arm] = v
            rig.pad_rot[arm] = _qm(_td(0.0))
            rig.quat[arm] = _asm(_td(0.0))
        _ = _np

    # ------------------------------------------------------------- controls
    # The torso control asks the ANCHOR, whatever orientation is under test:
    # its job is to prove the clearance model reports a negative number for an
    # arm inside a person, and a control that cannot be posed proves nothing.
    ctl = {}
    _anchor = n.ee_quat("left")
    _save, rig.quat["left"] = rig.quat["left"], _anchor
    jt = rig.solve_joints("left", [0.05, -0.02, 1.22], avoid=False)
    rig.quat["left"] = _save
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = dict(
        want="< 0", got=None if c_t is None else round(c_t, 4))

    home = [n.js.get(k, 0.0) for k in n.names("left")]
    c_h = rig.clearance("left", home)[0]
    ctl["home_clears_floor"] = dict(want=">= %.2f" % a.floor,
                                    got=None if c_h is None else round(c_h, 4))

    ok_o, c_o, _ = full_path(rig, "left", [a.x_control, 0.175, Z_NOW], 1)
    ctl["outboard_cell_reachable_and_clear"] = dict(
        want="True and >= %.2f" % a.floor,
        got="%s / %s" % (ok_o, None if c_o is None else round(c_o, 4)))

    # Clearance must FALL as the target comes inboard, per arm. A map with one
    # value in it is the "zero variance" row of docs/ENGINEERING_LOG.md's instrument table.
    varies = True
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        _, c_in, _ = grasp_probe(rig, arm, [sgn * 0.30, 0.175, Z_NOW])
        _, c_out, _ = grasp_probe(rig, arm, [sgn * 0.65, 0.175, Z_NOW])
        good = (c_in is not None and c_out is not None and c_in < c_out)
        varies = varies and good
        ctl["%s_clearance_varies" % arm] = dict(
            want="inboard < outboard",
            got="%s vs %s" % (None if c_in is None else round(c_in, 4),
                              None if c_out is None else round(c_out, 4)))

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-34s want %-22s got %s" % (k, v["want"], v["got"]))
    ok_ctl = (c_t is not None and c_t < 0.0
              and c_h is not None and c_h >= a.floor
              and ok_o and c_o is not None and c_o >= a.floor
              and varies)
    if not ok_ctl:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    xs = frange(0.0, a.x_max, a.step)
    ys = frange(a.y[0], a.y[1], a.step)

    # ------------------------------------------------- stage 1, per height
    print("\nSTAGE 1 -- GRASP POSE ONLY, N=1, an OPTIMISTIC bound on |x|.")
    print("   The full path is never better, so a height with nothing here "
          "has nothing anywhere.")
    print("   floor %.2f m; work plane now %.3f; table top %.2f\n"
          % (a.floor, Z_NOW, TABLE_TOP))
    print("   %-6s %-6s %9s %9s %9s   %s"
          % ("z", "arm", "innerIK", "innerSAFE", "clr@x=0", "centreline "
             "x=0.00 safe rows"))

    stage1 = {}
    outer_ok = {"left": False, "right": False}
    survivors = []
    for z in zs:
        stage1["%.3f" % z] = {}
        for arm in ("left", "right"):
            sgn = 1.0 if arm == "left" else -1.0
            ik_hits, safe_hits, centre_clr, centre_safe = [], [], [], []
            grid = {}
            # the outer control column, so a row of zeros is distinguishable
            # from a loop that never ran
            ok_c, _, _ = grasp_probe(rig, arm, [sgn * a.x_control, 0.175, z])
            if ok_c:
                outer_ok[arm] = True
            for x in xs:
                for y in ys:
                    obj = [round(sgn * x, 4), y, z]
                    ok, c, who = grasp_probe(rig, arm, obj)
                    if not ok:
                        continue
                    ik_hits.append((x, y))
                    grid["%+.3f,%.3f" % (sgn * x, y)] = dict(
                        clearance_m=None if c is None else round(c, 4), to=who)
                    if c is not None and c >= a.floor:
                        safe_hits.append((x, y))
                        if x <= 1e-9:
                            centre_safe.append(y)
                        survivors.append((z, arm, obj))
                    if x <= 1e-9 and c is not None:
                        centre_clr.append(c)
            inner_ik = min((h[0] for h in ik_hits), default=None)
            inner_safe = min((h[0] for h in safe_hits), default=None)
            stage1["%.3f" % z][arm] = dict(
                outer_control_reachable=ok_c,
                innermost_ik_x=inner_ik, innermost_safe_x=inner_safe,
                centreline_safe_rows=[round(v, 3) for v in centre_safe],
                centreline_best_clearance_m=(
                    None if not centre_clr else round(max(centre_clr), 4)),
                n_ik=len(ik_hits), n_safe=len(safe_hits), clearance=grid)
            print("   %-6.3f %-6s %9s %9s %9s   %s"
                  % (z, arm,
                     "----" if inner_ik is None else "%.3f" % inner_ik,
                     "----" if inner_safe is None else "%.3f" % inner_safe,
                     "----" if not centre_clr else "%.4f" % max(centre_clr),
                     "none" if not centre_safe
                     else "y " + " ".join("%.3f" % v for v in centre_safe)))

    if not (outer_ok["left"] and outer_ok["right"]):
        print("\nREFUSING TO REPORT: the outer control column failed for an "
              "arm -- a map of zeros and a loop that never ran look the same.")
        json.dump(dict(refused="outer control column failed", controls=ctl,
                       stage1=stage1), open(a.out, "w"), indent=2)
        return 7

    # ------------------------------------------------- stage 2, full path
    # Two things get re-tested over the whole path: everything inside the
    # centre band the question is about, and the INNERMOST safe cell at each
    # height, because "how far off centre must the pads sit" is the number the
    # brief actually turns on and a grasp-pose bound is not allowed to answer
    # it.
    cand = [s for s in survivors if abs(s[2][0]) <= 0.10 + 1e-9]
    seen = {(round(z, 3), arm, round(o[0], 4), round(o[1], 4))
            for z, arm, o in cand}
    for zk, per in stage1.items():
        for arm, d in per.items():
            if d["innermost_safe_x"] is None:
                continue
            sgn = 1.0 if arm == "left" else -1.0
            best = None
            for key, v in d["clearance"].items():
                xs_, ys_ = (float(t) for t in key.split(","))
                if abs(abs(xs_) - d["innermost_safe_x"]) > 1e-9:
                    continue
                if v["clearance_m"] is None or v["clearance_m"] < a.floor:
                    continue
                if best is None or v["clearance_m"] > best[1]:
                    best = ([xs_, ys_, float(zk)], v["clearance_m"])
            if best is None:
                continue
            k = (round(float(zk), 3), arm, round(best[0][0], 4),
                 round(best[0][1], 4))
            if k in seen:
                continue
            seen.add(k)
            cand.append((float(zk), arm, best[0]))
            _ = sgn
    print("\nSTAGE 2 -- FULL PICK PATH, N=%d, furniture in, geometric "
          "clearance." % a.full_repeats)
    print("   %d cells to re-test: the centre band, plus the innermost safe "
          "column at each height." % len(cand))
    stage2 = []
    for z, arm, obj in cand:
        ok, c, who = full_path(rig, arm, obj, a.full_repeats)
        stage2.append(dict(z=z, arm=arm, obj=obj, reachable=ok,
                           worst_clearance_m=None if c is None else round(c, 4),
                           to=who, safe=bool(ok and c is not None
                                             and c >= a.floor)))
        print("   z=%.3f %-5s (%+.3f, %.3f)  %-12s worst clearance %s -> %s"
              % (z, arm, obj[0], obj[1],
                 "REACHABLE" if ok else "unreachable",
                 "----" if c is None else "%.4f m" % c, who))
    if not cand:
        print("   nothing survived stage 1 inside the centre band, so there "
              "is nothing for the full path to confirm or deny.")

    safe2 = [r for r in stage2 if r["safe"] and abs(r["obj"][0]) <= 0.10 + 1e-9]
    print("\nINNERMOST SAFE COLUMN, CONFIRMED OVER THE FULL PATH")
    for r in sorted(stage2, key=lambda d: (d["z"], d["arm"])):
        if abs(r["obj"][0]) <= 0.10 + 1e-9:
            continue
        print("   z=%.3f %-5s x=%+.3f y=%.3f  %-12s clearance %s  "
              "(%.0f mm off centre, %.0f mm above the table)"
              % (r["z"], r["arm"], r["obj"][0], r["obj"][1],
                 "SAFE" if r["safe"] else
                 ("reach only" if r["reachable"] else "unreachable"),
                 "----" if r["worst_clearance_m"] is None
                 else "%.4f m" % r["worst_clearance_m"],
                 abs(r["obj"][0]) * 1000.0, (r["z"] - TABLE_TOP) * 1000.0))
    print("\nANSWER")
    if safe2:
        print("   The centre IS workable at %d (height, arm, cell) "
              "combinations:" % len(safe2))
        for r in safe2:
            print("      z=%.3f %-5s (%+.3f, %.3f) clearance %.4f m  "
                  "-- %.0f mm above the table top"
                  % (r["z"], r["arm"], r["obj"][0], r["obj"][1],
                     r["worst_clearance_m"], (r["z"] - TABLE_TOP) * 1000.0))
    else:
        print("   NO height in %.2f..%.2f puts |x| <= 0.10 both inside the "
              "arm's reach and outside the %.0f mm wearer floor, over the "
              "full pick path." % (min(zs), max(zs), a.floor * 1000.0))

    out = dict(orientation=a.orientation,
               zs=zs, xs=xs, ys=ys, step=a.step, floor=a.floor,
               work_plane_now=Z_NOW, table_top=TABLE_TOP,
               full_repeats=a.full_repeats, controls=ctl,
               stage1=stage1, stage2=stage2, ik_calls=rig.calls,
               method="stage 1 grasp pose only N=1 (optimistic); stage 2 "
                      "full pick path at N=full_repeats with furniture in "
                      "and the wearer measured geometrically",
               caveat="clearance is the clearance of the solution "
                      "/compute_ik returned -- the pose the follower would be "
                      "commanded to, but a sample of the null space and not "
                      "an infimum")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
