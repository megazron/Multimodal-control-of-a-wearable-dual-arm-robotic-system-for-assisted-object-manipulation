#!/usr/bin/env python3
"""CAN THE ARMS WORK IN THE CENTRE IF THE WEARER MOVES THEIR ARMS?

    SRL_WEARER_ARMS=behind python3 scripts/measure_centre_vs_wearer_posture.py
    python3 scripts/sweep_wearer_postures.py          # all five, one stack each

THE QUESTION, AND WHY IT IS FAIR TO ASK IT AGAIN. This project has measured the
front centre unreachable at 780 configurations and named the binding constraint
the WEARER'S OWN FOREARM, with min |x| stuck at 0.325 across every table
distance and every work height. That measurement is sound and it is also
measured against a mannequin whose arms hang rigidly at its sides, because
until now that was the only posture the model could hold. A person standing in
a rig with two robot arms working in front of their chest does not stand like
that, and asking them to hold their arms clear is an ordinary operating
instruction rather than a change to the platform.

So: same sweep, same controls, same floor, with the wearer's arms in postures a
person can actually adopt. `wearer_posture.py` holds them, the URDF's arm block
is generated from it, and the geometric clearance model is built from it, so a
posture reaches BOTH the collision model MoveIt plans against and the floor the
clearance is measured against. That double reach is checked here as a control,
because a posture that reaches one and not the other is a sweep that runs five
times and reports the shipped answer five times.

WHAT IT REPORTS, per posture and per arm:

    innermost IK column        how near the centreline a grasp pose solves
    innermost SAFE column      ... and also keeps the 150 mm wearer floor
    the centreline itself      whether |x| <= 0.10 opens at all
    what binds at the limit    which wearer part the arm is closest to

and the last of those is the one that decides the project's shape: if the arms
come out of the way and the answer does not move, the constraint was never the
arms.

CONTROLS, and there is no report without them:

    the posture is IN THE URDF      the loaded robot_description's human arm
                                    origins must match the posture table, and
                                    `none` must have no human arm links at all
    the posture is IN THE FLOOR     mount_guard's model must name the same
                                    posture in the same process
    a pose inside the torso         clearance must be NEGATIVE
    an outboard cell SOLVES         so a map of nothing is distinguishable
                                    from a loop that never ran
    the clearance MAP is not flat   at least three distinct values per
                                    arm, asked of the whole map after stage 1

AND A NOTE ON WHAT IS NOT A CONTROL HERE, because getting this wrong cost two
postures a report. The fixed-wearer sweep requires the outboard cell to clear
the floor, and requires clearance to FALL as x goes to zero. Both are true of a
wearer whose arms hang at their sides and neither is true by construction:
under `behind` and `out` the wearer's own arm moves toward the outboard cell,
so the requirement failed and the run refused, on a posture the sweep exists to
measure. A control whose ground truth moves with the thing under test is an
outcome in disguise. Those two are now REPORTED, beside `home_clears_the_floor`
-- if a wearer folding their arms puts the robot's own resting pose inside
them, that is the finding, and refusing to print it would hide it.
"""
import argparse
import json
import os
import sys

import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR                  # noqa: E402
from measure_centre_vs_height import (grasp_probe, full_path,    # noqa: E402
                                      frange, Z_NOW, TABLE_TOP)
from srl_teleop import wearer_posture as WP                      # noqa: E402
from srl_teleop import mount_guard_node as MG                    # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_vs_wearer_posture.json")


def grasp_probe_n(rig, arm, obj, repeats):
    """(solved every time, WORST clearance over the repeats, who).

    `grasp_probe` asks once. This asks `repeats` times and keeps the worst,
    because the solver seeds randomly and a cell that clears the floor on one
    flip and breaches it on the next is not a cell the pads can go in.
    """
    worst, who, ok = 1e9, None, True
    for _ in range(max(1, repeats)):
        got, c, k = grasp_probe(rig, arm, obj)
        if not got or c is None:
            return False, None, None
        if c < worst:
            worst, who = c, k
    return ok, worst, who


def robot_description(node, timeout=20.0):
    """The URDF the STACK is actually running, not the one on disk.

    Read from the latched /robot_description topic rather than re-running
    xacro, because the whole point of the check is that what move_group loaded
    might not be what this process would generate.
    """
    import time as _t
    from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                           QoSReliabilityPolicy)
    from std_msgs.msg import String
    got = {}
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    sub = node.create_subscription(
        String, "/robot_description", lambda m: got.setdefault("x", m.data), qos)
    end = _t.time() + timeout
    while _t.time() < end and "x" not in got:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return got.get("x", "")


def urdf_says(node, posture):
    """Does the LOADED robot description actually hold this posture?

    THIS IS THE CONTROL THAT MATTERS. The posture travels to move_group through
    an environment variable read at xacro time. If the stack was already up, or
    the variable did not survive the launch shell, every posture measures the
    same wearer and the sweep prints five confident identical answers.

    Returns (ok, detail).
    """
    import xml.etree.ElementTree as ET
    xml = robot_description(node)
    if not xml:
        return False, "no robot_description to check"
    root = ET.fromstring(xml)
    joints = {j.get("name"): j for j in root.iter("joint")}
    want = {}
    for side in ("left", "right"):
        for name, _k, _d, ctr, rpy in WP.arm_links(posture, side, "local"):
            want["torso_to_human_%s_%s" % (side, name)] = (ctr, rpy)
    if not want:
        stray = [n for n in joints if n.startswith("torso_to_human_")]
        if stray:
            return False, "posture 'none' but the URDF still has %s" % stray[0]
        return True, "no human arm joints, as 'none' requires"
    for jname, (ctr, rpy) in want.items():
        j = joints.get(jname)
        if j is None:
            return False, "the URDF has no %s" % jname
        o = j.find("origin")
        got = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        for k in range(3):
            if abs(got[k] - ctr[k]) > 1e-4:
                return False, ("%s origin is %s, the %s table says %s"
                               % (jname, got, posture,
                                  [round(v, 6) for v in ctr]))
    return True, "%d human arm joints match the %s table" % (len(want), posture)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--posture", default=None,
                    help="default: whatever SRL_WEARER_ARMS says, which is "
                         "what the stack was launched with")
    ap.add_argument("--full-repeats", type=int, default=10)
    ap.add_argument("--stage1-repeats", type=int, default=2,
                    help="a grasp pose must solve AND clear the floor on "
                         "EVERY repeat. TRAC-IK restarts randomly, so one "
                         "call is one flip and a single-call map of the "
                         "innermost column is not reproducible -- measured, "
                         "it moved 75 mm between two runs of the same code")
    ap.add_argument("--confirm-columns", type=int, default=3,
                    help="how many of the innermost safe columns per arm to "
                         "re-test over the whole path")
    ap.add_argument("--scatter-probe", type=int, default=10,
                    help="re-ask the innermost safe cell this many times and "
                         "report the solver's own spread beside the answer")
    ap.add_argument("--step", type=float, default=0.025)
    ap.add_argument("--y-step", type=float, default=0.05,
                    help="the answer is a column, so y can be coarser than x")
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-control", type=float, default=0.75)
    ap.add_argument("--y", type=float, nargs=2, default=(0.10, 0.55))
    ap.add_argument("--z", type=float, nargs="*", default=None)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    posture = a.posture or WP.posture_from_env()
    if posture != MG.POSTURE:
        print("REFUSING: this process built its clearance model for %r but "
              "was asked about %r. Set %s before starting the stack, not "
              "after." % (MG.POSTURE, posture, WP.ENV_VAR))
        return 4
    zs = a.z if a.z else [Z_NOW]
    out_path = a.out or os.path.join(
        ROOT, "recordings/baselines/centre_posture_%s.json" % posture)

    print("WEARER ARM POSTURE: %s -- %s" % (posture, WP.POSTURES[posture]["doc"]))

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

    ctl = {}
    ok_urdf, why_urdf = urdf_says(n, posture)
    ctl["posture_is_in_the_urdf"] = dict(want="True", got="%s (%s)"
                                         % (ok_urdf, why_urdf))
    ctl["posture_is_in_the_clearance_model"] = dict(
        want=posture, got=MG.POSTURE)

    _anchor = n.ee_quat("left")
    _save, rig.quat["left"] = rig.quat["left"], _anchor
    jt = rig.solve_joints("left", [0.05, -0.02, 1.22], avoid=False)
    rig.quat["left"] = _save
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    ctl["inside_torso_negative"] = dict(want="< 0",
                                        got=None if c_t is None else round(c_t, 4))

    # A CONTROL WHOSE GROUND TRUTH MOVES WITH THE THING UNDER TEST IS NOT A
    # CONTROL. The first version of this file required the outboard cell to be
    # both reachable AND clear of the floor, copied from the fixed-wearer
    # sweep. Under `behind` and `out` the wearer's own arm swings toward that
    # cell, so the requirement failed and two postures refused to report --
    # not because the instrument was broken but because the CONTROL was an
    # outcome in disguise. What survives posture is: the loop ran (the cell
    # solves), and the clearance map is not a constant. The clearance AT that
    # cell, and whether it beats an inboard one, are results and are reported
    # as such.
    ok_o, c_o, who_o = full_path(rig, "left", [a.x_control, 0.175, zs[0]], 1)
    ctl["outboard_cell_solves"] = dict(
        want="True", got="%s (clearance %s to %s)"
        % (ok_o, None if c_o is None else round(c_o, 4), who_o))

    # `varies` is settled AFTER stage 1, from the map itself -- see below.
    direction = {}
    for arm in ("left", "right"):
        sgn = 1.0 if arm == "left" else -1.0
        _, c_in, w_in = grasp_probe(rig, arm, [sgn * 0.30, 0.175, zs[0]])
        _, c_out, w_out = grasp_probe(rig, arm, [sgn * 0.65, 0.175, zs[0]])
        direction[arm] = dict(
            inboard=None if c_in is None else round(c_in, 4), inboard_to=w_in,
            outboard=None if c_out is None else round(c_out, 4),
            outboard_to=w_out)

    home = [n.js.get(k, 0.0) for k in n.names("left")]
    c_h = rig.clearance("left", home)[0]
    home_r = [n.js.get(k, 0.0) for k in n.names("right")]
    c_hr = rig.clearance("right", home_r)[0]

    print("\nCONTROLS")
    for k, v in ctl.items():
        print("   %-38s want %-22s got %s" % (k, v["want"], v["got"]))
    ok_ctl = (ok_urdf and MG.POSTURE == posture
              and c_t is not None and c_t < 0.0
              and ok_o)
    if not ok_ctl:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(posture=posture, refused=True, controls=ctl),
                  open(out_path, "w"), indent=2)
        return 6

    print("\nOUTCOMES, NOT CONTROLS -- these move WITH the posture")
    for arm, d in direction.items():
        print("   %-5s clearance at |x|=0.30 %s (%s) ; at |x|=0.65 %s (%s)"
              % (arm, d["inboard"], d["inboard_to"], d["outboard"],
                 d["outboard_to"]))
    print("   the outboard control cell x=%.2f: clearance %s to %s"
          % (a.x_control, None if c_o is None else round(c_o, 4), who_o))
    print("\nHOME POSE AGAINST THIS POSTURE (an outcome, not a control)")
    for arm, c in (("left", c_h), ("right", c_hr)):
        print("   %-5s home clearance %s  %s"
              % (arm, "----" if c is None else "%.4f m" % c,
                 "" if c is None or c >= a.floor
                 else "<-- INSIDE the %.0f mm floor" % (a.floor * 1000.0)))

    xs = frange(0.0, a.x_max, a.step)
    ys = frange(a.y[0], a.y[1], a.y_step or a.step)

    print("\nSTAGE 1 -- GRASP POSE ONLY, N=%d, an OPTIMISTIC bound on |x|."
          % a.stage1_repeats)
    print("   %-6s %-6s %9s %9s %9s   %s"
          % ("z", "arm", "innerIK", "innerSAFE", "clr@x=0", "what binds at "
             "the innermost safe column"))
    stage1, survivors = {}, []
    outer_ok = {"left": False, "right": False}
    for z in zs:
        stage1["%.3f" % z] = {}
        for arm in ("left", "right"):
            sgn = 1.0 if arm == "left" else -1.0
            ik_hits, safe_hits, centre_clr, centre_safe = [], [], [], []
            grid, binds = {}, {}
            ok_c, _, _ = grasp_probe(rig, arm, [sgn * a.x_control, 0.175, z])
            if ok_c:
                outer_ok[arm] = True
            for x in xs:
                for y in ys:
                    obj = [round(sgn * x, 4), y, z]
                    ok, c, who = grasp_probe_n(rig, arm, obj,
                                               a.stage1_repeats)
                    if not ok:
                        continue
                    ik_hits.append((x, y))
                    grid["%+.3f,%.3f" % (sgn * x, y)] = dict(
                        clearance_m=None if c is None else round(c, 4), to=who)
                    if c is not None and c >= a.floor:
                        safe_hits.append((x, y))
                        binds.setdefault(round(x, 4), who)
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
                binds_at_innermost_safe=binds.get(inner_safe),
                centreline_safe_rows=[round(v, 3) for v in centre_safe],
                centreline_best_clearance_m=(
                    None if not centre_clr else round(max(centre_clr), 4)),
                n_ik=len(ik_hits), n_safe=len(safe_hits), clearance=grid)
            print("   %-6.3f %-6s %9s %9s %9s   %s"
                  % (z, arm,
                     "----" if inner_ik is None else "%.3f" % inner_ik,
                     "----" if inner_safe is None else "%.3f" % inner_safe,
                     "----" if not centre_clr else "%.4f" % max(centre_clr),
                     binds.get(inner_safe) or "-"))

    # THE MAP MUST NOT BE A CONSTANT, and this is the place to ask.
    # docs/ENGINEERING_LOG.md's instrument table lists "zero variance" as a dead channel or
    # a cached value republished, and a clearance map with one number in it is
    # exactly that picture. Asked of the MAP rather than of two hand-picked
    # cells, because under `out` the wearer's hand sits at |x| = 0.655 and one
    # of those cells stops solving -- which refused a posture on the strength
    # of a control that was really a result.
    distinct = {}
    for zk, per in stage1.items():
        for arm, d in per.items():
            vals = {v["clearance_m"] for v in d["clearance"].values()
                    if v["clearance_m"] is not None}
            distinct["%s@%s" % (arm, zk)] = len(vals)
    ctl["clearance_map_is_not_a_constant"] = dict(
        want=">= 3 distinct values per arm", got=str(distinct))
    print("   %-38s want %-22s got %s"
          % ("clearance_map_is_not_a_constant", ">= 3 distinct", distinct))
    if any(v < 3 for v in distinct.values()):
        print("\nREFUSING TO REPORT: the clearance map is flat, which is the "
              "picture a dead measurement makes.")
        json.dump(dict(posture=posture, refused="flat clearance map",
                       controls=ctl, stage1=stage1), open(out_path, "w"),
                  indent=2)
        return 8

    if not (outer_ok["left"] and outer_ok["right"]):
        print("\nREFUSING TO REPORT: the outer control column failed for an "
              "arm -- a map of zeros and a loop that never ran look the same.")
        json.dump(dict(posture=posture, refused="outer control column failed",
                       controls=ctl, stage1=stage1), open(out_path, "w"),
                  indent=2)
        return 7

    # ------------------------------------------------ what stage 2 re-tests
    #
    # The centre band, always, because that is the question. Plus the
    # INNERMOST FEW SAFE COLUMNS per arm, walked from the centreline outward,
    # because "how far off centre must the work sit" is the number the brief
    # turns on and a single optimistic probe is not allowed to answer it. Each
    # column contributes its best cell -- the one with most clearance to spare
    # -- since a column is usable if ANY cell in it is.
    cand = [s for s in survivors if abs(s[2][0]) <= 0.10 + 1e-9]
    seen = {(round(z, 3), arm, round(o[0], 4), round(o[1], 4))
            for z, arm, o in cand}
    for zk, per in stage1.items():
        for arm, d in per.items():
            best_by_col = {}
            for key, v in d["clearance"].items():
                xs_, ys_ = (float(t) for t in key.split(","))
                if v["clearance_m"] is None or v["clearance_m"] < a.floor:
                    continue
                col = round(abs(xs_), 4)
                cur = best_by_col.get(col)
                if cur is None or v["clearance_m"] > cur[1]:
                    best_by_col[col] = ([xs_, ys_, float(zk)], v["clearance_m"])
            for col in sorted(best_by_col)[:max(1, a.confirm_columns)]:
                obj = best_by_col[col][0]
                k = (round(float(zk), 3), arm, round(obj[0], 4),
                     round(obj[1], 4))
                if k in seen:
                    continue
                seen.add(k)
                cand.append((float(zk), arm, obj))

    # ------------------------------------------------------- solver scatter
    # THE PUBLISHED NUMBER AND THIS ONE ARE BOTH SAMPLES. Measured here at
    # N=1: two runs of identical code put the left arm's innermost safe column
    # at 0.350 and at 0.425. So report the spread rather than pretending the
    # answer is a constant, and let the full path at N=`full_repeats` be the
    # number that gets quoted.
    scatter = {}
    for zk, per in stage1.items():
        for arm, d in per.items():
            if d["innermost_safe_x"] is None or a.scatter_probe < 2:
                continue
            sgn = 1.0 if arm == "left" else -1.0
            row = None
            for key, v in d["clearance"].items():
                xs_, ys_ = (float(t) for t in key.split(","))
                if abs(abs(xs_) - d["innermost_safe_x"]) > 1e-9:
                    continue
                if v["clearance_m"] is not None and v["clearance_m"] >= a.floor:
                    row = ys_
                    break
            if row is None:
                continue
            obj = [sgn * d["innermost_safe_x"], row, float(zk)]
            vals = []
            for _ in range(a.scatter_probe):
                ok, c, _w = grasp_probe(rig, arm, obj)
                vals.append(None if not ok or c is None else round(c, 4))
            good = [v for v in vals if v is not None]
            scatter["%s@%s" % (arm, zk)] = dict(
                cell=obj, n=a.scatter_probe, solved=len(good),
                min=min(good) if good else None,
                max=max(good) if good else None,
                clears_every_time=bool(good and len(good) == len(vals)
                                       and min(good) >= a.floor),
                values=vals)
    if scatter:
        print("\nSOLVER SCATTER at the innermost safe cell, %d repeats each"
              % a.scatter_probe)
        for k, v in scatter.items():
            print("   %-14s solved %d/%d  clearance %s..%s  clears the floor "
                  "every time: %s"
                  % (k, v["solved"], v["n"],
                     "----" if v["min"] is None else "%.4f" % v["min"],
                     "----" if v["max"] is None else "%.4f" % v["max"],
                     v["clears_every_time"]))

    print("\nSTAGE 2 -- FULL PICK PATH, N=%d, furniture in, geometric "
          "clearance.  %d cells." % (a.full_repeats, len(cand)))
    stage2 = []
    for z, arm, obj in cand:
        ok, c, who = full_path(rig, arm, obj, a.full_repeats)
        stage2.append(dict(z=z, arm=arm, obj=obj, reachable=ok,
                           worst_clearance_m=None if c is None else round(c, 4),
                           to=who,
                           safe=bool(ok and c is not None and c >= a.floor)))
        print("   z=%.3f %-5s (%+.3f, %.3f)  %-12s worst clearance %s -> %s"
              % (z, arm, obj[0], obj[1],
                 "REACHABLE" if ok else "unreachable",
                 "----" if c is None else "%.4f m" % c, who))

    centre_ok = [r for r in stage2
                 if r["safe"] and abs(r["obj"][0]) <= 0.10 + 1e-9]
    confirmed = {}
    for arm in ("left", "right"):
        rows = [r for r in stage2 if r["arm"] == arm and r["safe"]]
        confirmed[arm] = (min(abs(r["obj"][0]) for r in rows) if rows else None)

    print("\nANSWER FOR POSTURE %r" % posture)
    for arm in ("left", "right"):
        s1 = stage1["%.3f" % zs[0]][arm]
        print("   %-5s innermost safe column, grasp pose %s ; confirmed over "
              "the full path %s ; binds on %s"
              % (arm,
                 "----" if s1["innermost_safe_x"] is None
                 else "%.3f" % s1["innermost_safe_x"],
                 "----" if confirmed[arm] is None else "%.3f" % confirmed[arm],
                 s1["binds_at_innermost_safe"] or "-"))
    if centre_ok:
        print("   THE CENTRE OPENS: %d cells at |x| <= 0.10 are reachable and "
              "clear over the full path." % len(centre_ok))
    else:
        print("   The centre stays shut: no cell at |x| <= 0.10 is both "
              "reachable and clear of the %.0f mm floor."
              % (a.floor * 1000.0))

    out = dict(posture=posture, posture_doc=WP.POSTURES[posture]["doc"],
               zs=zs, xs=xs, ys=ys, step=a.step, floor=a.floor,
               work_plane=Z_NOW, table_top=TABLE_TOP,
               full_repeats=a.full_repeats, controls=ctl,
               outboard_probe=dict(
                   x=a.x_control,
                   clearance_m=None if c_o is None else round(c_o, 4),
                   to=who_o),
               inboard_vs_outboard=direction,
               home_clearance_m=dict(
                   left=None if c_h is None else round(c_h, 4),
                   right=None if c_hr is None else round(c_hr, 4)),
               innermost_safe_confirmed=confirmed,
               solver_scatter=scatter,
               centre_cells=centre_ok,
               stage1=stage1, stage2=stage2, ik_calls=rig.calls,
               method="stage 1 grasp pose only N=1 (optimistic); stage 2 full "
                      "pick path at N=full_repeats with furniture in and the "
                      "wearer measured geometrically against the posture's "
                      "own primitives")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    json.dump(out, open(out_path, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, out_path))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
