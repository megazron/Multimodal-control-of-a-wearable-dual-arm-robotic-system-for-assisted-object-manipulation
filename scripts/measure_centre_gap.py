#!/usr/bin/env python3
"""WHAT IS IN THE GAP IN FRONT OF THE WEARER? PER ARM, PER COLUMN, BY LINK.

    python3 scripts/measure_centre_gap.py                     # the gap, as built
    python3 scripts/measure_centre_gap.py --reconcile         # + the 0.075 protocol
    python3 scripts/measure_centre_gap.py --tables            # table height/distance
    python3 scripts/measure_centre_gap.py --self-test         # controls only

THE QUESTION, AND WHY IT HAS BEEN ANSWERED FOUR WAYS

The empty patch directly in front of the wearer, between the two workspace
markings, is where the pads belong. Four different numbers have been quoted for
whether an arm can reach into it -- 0.075, 0.250, 0.400, 0.425 -- and every one
of them is the answer to a DIFFERENT question. That is the whole problem: none
of them is wrong, and none of them alone is usable.

So this reports the ladder, per arm, per 25 mm column, and never collapses it:

    IK-only            /compute_ik with avoid_collisions, GRASP POSE alone,
                       objects where that measurement put them
    IK, full path      the same with the whole pick path walked at N repeats
    + clearance floor  the 150 mm wearer floor, measured GEOMETRICALLY on
                       every solution, because `avoid_collisions` cannot see
                       the pairs a shoulder mount threatens

and for every column that fails it names WHAT stopped it and WHICH LINK:

    IK_FAIL       no configuration reaches the pose at all, collisions off
    COLLISION     a collision-free solution exists only with collisions off;
                  split into WEARER (the capsule model reads negative, with
                  the wearer part and the ROBOT LINK SEGMENT named), TABLE
                  (free once the furniture is out) and SELF/OTHER
    FLOOR         it solves cleanly and sits inside the 150 mm floor, with the
                  wearer part, the robot link segment and the distance named

NAMING THE LINK IS THE POINT. "The wearer blocks it" is what has been reported
before, and it is not actionable: a torso cannot be moved and a forearm can.
`mount_guard_node.CHAIN` is walked segment by segment, so the answer is
`half_arm_1_link -> half_arm_2_link, 0.061 m from L-forearm` rather than
`WEARER`.

CONTROLS, and no report without them, imported from `search_t1_centre` so
there is one implementation of each.
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
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from geometry_msgs.msg import Quaternion                      # noqa: E402
from moveit_msgs.msg import RobotState                        # noqa: E402
from moveit_msgs.srv import GetPositionFK                     # noqa: E402
from moveit_msgs.srv import GetStateValidity                  # noqa: E402

from verify_task_scenes import Solver, HOME_TOL_RAD           # noqa: E402
from audit_scenario_reachability import densify               # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR               # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee,        # noqa: E402
                                    as_msg, as_tuple)
from search_t1_centre import (put_table, clear_table, controls,  # noqa: E402
                              frange, CUBE, STANDOFF, LIFT, TABLE_ID)
from srl_teleop import mount_guard_node as MG                 # noqa: E402
from srl_teleop import wearer_posture as WP                   # noqa: E402
import grasp_frames as GF                                     # noqa: E402
import t1_task as T1                                          # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/centre_gap.json")

IK_FAIL, COLL_WEARER, COLL_TABLE, COLL_OTHER, FLOOR, OK = (
    "IK_FAIL", "COLLISION:WEARER", "COLLISION:TABLE", "COLLISION:SELF/OTHER",
    "FLOOR", "REACHABLE")


# ---------------------------------------------------------------- clearance
def clearance_named(fkc, node, arm, joints, model=None):
    """(distance, wearer part, robot link segment, point) -- the WORST pair.

    `Rig.clearance` returns the distance and the wearer part. This also returns
    the ROBOT LINK SEGMENT the worst point lies on, which is the half of the
    answer that says what could be changed.
    """
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.fk_link_names = ["%s_%s" % (arm, ln) for ln in MG.CHAIN]
    rs = RobotState()
    rs.joint_state.name = node.names(arm)
    rs.joint_state.position = [float(v) for v in joints[:7]]
    req.robot_state = rs
    fut = fkc.call_async(req)
    import time as _t
    end = _t.time() + 5.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.002)
    res = fut.result()
    if res is None or res.error_code.val != 1 or not res.pose_stamped:
        return None, None, None, None
    pts = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
           for ps in res.pose_stamped]
    wearer = MG.WEARER if model is None else model
    worst, who, seg, at = 1e9, None, None, None
    for si, (a, b) in enumerate(zip(pts, pts[1:])):
        for k in range(MG.SAMPLES + 1):
            t = k / float(MG.SAMPLES)
            p = [a[i] + (b[i] - a[i]) * t for i in range(3)]
            for name, kind, prm, ctr, rpy in wearer:
                d = MG.dist_point(p, kind, prm, ctr, rpy) - MG.TUBE_R
                if d < worst:
                    worst, who = d, name
                    seg = "%s -> %s" % (MG.CHAIN[si], MG.CHAIN[si + 1])
                    at = [round(v, 4) for v in p]
    return worst, who, seg, at


# -------------------------------------------------------------------- paths
def pick_path(obj, q, pad_mid):
    pre, ee = GF.approach_path(obj, q, STANDOFF, pad_mid)
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def classify_cell(rig, fkc, arm, obj, q, pad_mid, repeats, floor, full=True):
    """The ladder, on the whole path. Returns a dict naming the blocker."""
    qm = as_msg(q)
    wps = pick_path(obj, q, pad_mid) if full else \
        [GF.wrist_for(obj, q, pad_mid)]
    worst, who, seg, at, worst_i = 1e9, None, None, None, None
    for i, w in enumerate(wps):
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                # WHY did it fail? Ask the ladder at THIS waypoint.
                return _why(rig, fkc, arm, w, qm, i, len(wps))
            c, k, s, p = clearance_named(fkc, rig.n, arm, j)
            if c is None:
                return dict(verdict=IK_FAIL, detail="FK failed", at_wp=i)
            if c < worst:
                worst, who, seg, at, worst_i = c, k, s, p, i
    if worst < floor:
        return dict(verdict=FLOOR, clearance=round(worst, 4), wearer=who,
                    link=seg, point=at, at_wp=worst_i,
                    detail="solves cleanly; %s is %.4f m from %s against a "
                           "%.3f m floor" % (seg, worst, who, floor))
    return dict(verdict=OK, clearance=round(worst, 4), wearer=who, link=seg,
                point=at, at_wp=worst_i)


def contacts_for(node, arm, joints, max_pairs=4):
    """WHICH LINK PAIR IS TOUCHING, from MoveIt itself.

    `COLLISION:SELF/OTHER` means MoveIt refused a state that the capsule wearer
    model calls clear -- so it is a self-collision, or a body pair the capsules
    do not carry. Either way "SELF/OTHER" is not an answer anyone can act on.
    `/check_state_validity` will return the CONTACTS if asked, and that names
    the two links.

    Cached on the node, because the client has to be created once and this is
    called from inside a sweep.
    """
    cli = getattr(node, "_valid_cli", None)
    if cli is None:
        cli = node.create_client(GetStateValidity, "/check_state_validity")
        cli.wait_for_service(timeout_sec=20.0)
        node._valid_cli = cli
    req = GetStateValidity.Request()
    req.group_name = "%s_arm" % arm
    rs = RobotState()
    rs.joint_state.name = node.names(arm)
    rs.joint_state.position = [float(v) for v in joints[:7]]
    rs.is_diff = True
    req.robot_state = rs
    fut = cli.call_async(req)
    import time as _t
    end = _t.time() + 5.0
    while _t.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.002)
    res = fut.result()
    if res is None:
        return None
    out = []
    for c in list(res.contacts)[:max_pairs]:
        out.append("%s / %s" % (c.contact_body_1, c.contact_body_2))
    return out or (["valid"] if res.valid else ["no contacts reported"])


def _why(rig, fkc, arm, w, qm, i, n):
    """A waypoint that will not solve collision-free. Name what blocks it."""
    # 1. Is it the FURNITURE? Free with the scene out and nothing else changed.
    #
    # BOTH TABLES. `Rig.set_furniture(False)` removes clip_scene's OWN ids and
    # nothing else, and every sweep in this file applies its table separately
    # as `search_table` -- so this branch used to leave the very slab under
    # test in the scene and never fire. Measured: a whole heading swept as
    # COLLISION:SELF/OTHER when `/check_state_validity` named the contact as
    # `robotiq_85_base_link / table`. The ladder was hiding the answer behind
    # its most useless verdict.
    rig.set_furniture(False)
    _had_slab = TABLE_ID in getattr(rig, "_slab_on", set())
    clear_table(rig)
    free = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
    rig.calls += 1
    rig.set_furniture(True)
    if _had_slab:
        put_table(rig, _why.top, _why.near_y)
    if free is not None:
        return dict(verdict=COLL_TABLE, at_wp=i, of=n,
                    detail="free once the furniture is out, so the TABLE "
                           "blocks it")
    # 2. Does a solution exist at all with collisions off?
    j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=False, tries=8)
    rig.calls += 1
    if j is None:
        return dict(verdict=IK_FAIL, at_wp=i, of=n,
                    detail="no configuration reaches this pose with "
                           "collisions OFF -- kinematics or the wrist")
    c, k, s, p = clearance_named(fkc, rig.n, arm, j)
    if c is not None and c < 0.0:
        return dict(verdict=COLL_WEARER, at_wp=i, of=n,
                    clearance=round(c, 4), wearer=k, link=s, point=p,
                    detail="%s is %.0f mm INSIDE %s" % (s, -c * 1000.0, k))
    pairs = contacts_for(rig.n, arm, j)
    return dict(verdict=COLL_OTHER, at_wp=i, of=n,
                clearance=None if c is None else round(c, 4), wearer=k,
                link=s, contacts=pairs,
                detail="MoveIt refuses it and the capsule wearer model reads "
                       "%s; contacts: %s"
                       % ("None" if c is None else "%.4f m" % c,
                          "; ".join(pairs) if pairs else "none reported"))


# ------------------------------------------------------------- the x sweep
def sweep_columns(rig, fkc, arm, q, pad_mid, xs, y, z_obj, repeats, floor,
                  log, full=True):
    rows = []
    sgn = 1.0 if arm == "left" else -1.0
    for x in xs:
        obj = [round(sgn * x, 4), y, z_obj]
        r = classify_cell(rig, fkc, arm, obj, q, pad_mid, repeats, floor, full)
        r.update(arm=arm, x=x, obj=obj)
        rows.append(r)
        log("   |x| %.3f  %-18s %s"
            % (x, r["verdict"],
               (r.get("detail") or
                ("clearance %.4f m, closest %s to %s"
                 % (r.get("clearance", float("nan")), r.get("link"),
                    r.get("wearer"))))[:96]))
    return rows


def innermost(rows, verdict=OK):
    hit = [r for r in rows if r["verdict"] == verdict]
    return min((r["x"] for r in hit), default=None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-step", type=float, default=0.025)
    ap.add_argument("--table-top", type=float, default=T1.TABLE_TOP)
    ap.add_argument("--near-y", type=float, default=T1.TABLE_NEAR_Y)
    ap.add_argument("--reconcile", action="store_true",
                    help="also reproduce the protocol that returned 0.075")
    ap.add_argument("--tables", action="store_true",
                    help="sweep table height and distance for the gap")
    ap.add_argument("--tag", default="down",
                    help="a label for this run, e.g. the wearer posture or "
                         "mount candidate the stack was launched with")
    ap.add_argument("--approaches", default="t1,anchor",
                    help="which wrist orientations to sweep the columns at")
    # WHY HEADING IS ITS OWN SWEEP. The approach axis decides where the WRIST
    # sits relative to the object, and it is the wrist -- not the object --
    # that has to clear the torso. At T1's heading of 50 deg inboard the wrist
    # sits 70.8 mm NEARER the wearer than the cube it is grasping; at 90 deg,
    # a purely lateral hand, it sits at the cube's own y. That is 70 mm of row
    # distance, for free, and no fan in this repository has ever tested beyond
    # 50 deg.
    ap.add_argument("--heads", default="",
                    help="extra headings in degrees INBOARD, mirrored per arm")
    ap.add_argument("--elev", type=float, default=T1.APPROACH_ELEV_DEG)
    # ROLL MATTERS AND HAS NEVER BEEN SWEPT AT THE GAP. At elevation -90 the
    # heading is degenerate -- the tool axis is straight down whatever it is --
    # so roll is the only remaining freedom, and it decides which way the
    # gripper BODY lies over the table.
    ap.add_argument("--rolls", default="0")
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

    ctl, good = controls(rig, pad_mid, a.floor)
    log("CONTROLS   (run tag: %s; wearer posture in the URDF: %s)"
        % (a.tag, WP.posture_from_env()))
    for k, v in ctl.items():
        log("   %-38s %s" % (k, v))
    if not good:
        log("\nREFUSING: a control failed.")
        return 6
    # A CONTROL ON THE LINK NAMER ITSELF. A pose driven into the torso must
    # come back with a NEGATIVE distance, the torso named, and a link segment
    # -- not None. A namer that returns None under stress would make every
    # blocked column read as "unknown".
    ql = as_msg(as_tuple(rig.quat["left"]))
    jt = n.solve_arm_joints("left", [0.05, -0.02, 1.22], ql, avoid=False,
                            tries=8)
    c, who, seg, _p = clearance_named(fkc, n, "left", jt) if jt is not None \
        else (None, None, None, None)
    log("   %-38s %s / %s / %s"
        % ("link namer inside the torso", None if c is None else round(c, 4),
           who, seg))
    if not (c is not None and c < 0 and who and seg):
        log("\nREFUSING: the link namer cannot name a link on a pose that is "
            "definitely inside the wearer.")
        return 6
    if a.self_test:
        log("\nself-test only.")
        return 0

    res = dict(tag=a.tag, posture=WP.posture_from_env(), floor=a.floor,
               repeats=a.repeats, controls=ctl,
               table=dict(top=a.table_top, near_y=a.near_y))

    # ==================================================================
    # 1. THE GAP, ON THE TABLE, AS BUILT
    # ==================================================================
    z_obj = round(a.table_top + CUBE / 2.0, 4)
    y = round(a.near_y + CUBE / 2.0, 4)
    xs = frange(0.0, a.x_max, a.x_step)
    rig.set_furniture(False)
    put_table(rig, a.table_top, a.near_y)
    rig._slab_on = {TABLE_ID}
    _why.top, _why.near_y = a.table_top, a.near_y
    log("\n" + "=" * 74)
    log("1. THE GAP -- objects RESTING on the table, full pick path, N=%d"
        % a.repeats)
    log("   table top %.3f, near edge %.3f, row y = %.3f, object z = %.3f"
        % (a.table_top, a.near_y, y, z_obj))
    log("=" * 74)
    res["gap"] = {}
    for arm in ("left", "right"):
        _want = [v.strip() for v in a.approaches.split(",")]
        _all = [("t1", "T1 approach", T1.APPROACH[arm]),
                ("anchor", "pinned anchor", as_tuple(rig.quat[arm]))]
        for _hd in [float(v) for v in a.heads.split(",") if v.strip()]:
            _sgn = -1.0 if arm == "left" else 1.0
            for _rl in [float(v) for v in a.rolls.split(",") if v.strip()]:
                _k = "h%+.0f/r%+.0f" % (_hd, _rl)
                _all.append((_k, "heading %+.0f deg inboard, roll %+.0f"
                             % (_hd, _rl),
                             GF.q_from_axis(GF.axis_for(a.elev, _sgn * _hd),
                                            math.radians(_rl))))
                _want.append(_k)
        for _key, name, q in [t for t in _all if t[0] in _want]:
            e, h = GF.elev_head_of(q)
            log("\n  %s arm, %s (elevation %+.1f deg, heading %+.1f deg)"
                % (arm.upper(), name, e, h))
            rows = sweep_columns(rig, fkc, arm, q, pad_mid[arm], xs, y, z_obj,
                                 a.repeats, a.floor, log)
            res["gap"]["%s/%s" % (arm, name)] = rows
            log("      innermost REACHABLE |x| = %s"
                % ("none" % () if innermost(rows) is None
                   else "%.3f" % innermost(rows)))
    clear_table(rig)

    # ==================================================================
    # 2. RECONCILING THE FOUR ANSWERS
    # ==================================================================
    if a.reconcile:
        log("\n" + "=" * 74)
        log("2. THE SAME COLUMNS, ASKED THE WAY EACH EARLIER NUMBER ASKED")
        log("=" * 74)
        rig.set_furniture(True)          # the shipped scene, table at 0.980
        res["reconcile"] = {}
        # (label, z, y range, repeats, full path, floor applied)
        protocols = [
            ("centre_reach 0.075: IK ONLY, grasp pose, floating plane, "
             "y 0.10..0.55", 1.120, frange(0.10, 0.55, 0.025), 3, False, None),
            ("the same cells WITH the 150 mm floor", 1.120,
             frange(0.10, 0.55, 0.025), 3, False, a.floor),
            ("the same cells over the FULL PATH at N=10, with the floor",
             1.120, frange(0.10, 0.55, 0.025), a.repeats, True, a.floor),
        ]
        for label, z, ys, reps, full, floor in protocols:
            log("\n  %s" % label)
            for arm in ("left", "right"):
                q = as_tuple(rig.quat[arm])
                sgn = 1.0 if arm == "left" else -1.0
                best = None
                for x in xs:
                    for yy in ys:
                        obj = [round(sgn * x, 4), yy, z]
                        r = classify_cell(rig, fkc, arm, obj, q, pad_mid[arm],
                                          reps, floor if floor else -1e9, full)
                        if r["verdict"] == OK:
                            best = dict(x=x, y=yy,
                                        clearance=r.get("clearance"),
                                        wearer=r.get("wearer"),
                                        link=r.get("link"))
                            break
                    if best:
                        break
                res["reconcile"].setdefault(label, {})[arm] = best
                log("     %-5s innermost |x| = %s%s"
                    % (arm, "none" if best is None else "%.3f" % best["x"],
                       "" if best is None else
                       "  at y = %.3f, clearance %s to %s (%s)"
                       % (best["y"], best["clearance"], best["wearer"],
                          best["link"])))

    # ==================================================================
    # 3. WOULD A DIFFERENT TABLE OPEN IT?
    # ==================================================================
    if a.tables:
        log("\n" + "=" * 74)
        log("3. TABLE HEIGHT AND DISTANCE -- innermost REACHABLE |x| per cell")
        log("=" * 74)
        res["tables"] = []
        for top in (0.80, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10):
            for edge in (0.180, 0.230, 0.280, 0.330, 0.380, 0.430):
                zz = round(top + CUBE / 2.0, 4)
                yy = round(edge + CUBE / 2.0, 4)
                rig.set_furniture(False)
                put_table(rig, top, edge)
                row = dict(top=top, edge=edge)
                for arm in ("left", "right"):
                    rows = sweep_columns(rig, fkc, arm, T1.APPROACH[arm],
                                         pad_mid[arm], xs, yy, zz, 2, a.floor,
                                         lambda _s: None)
                    row[arm] = innermost(rows)
                res["tables"].append(row)
                log("   top %.2f edge %.3f   left %s   right %s"
                    % (top, edge,
                       "----" if row["left"] is None else "%.3f" % row["left"],
                       "----" if row["right"] is None
                       else "%.3f" % row["right"]))
        clear_table(rig)

    res["log"] = lines
    res["ik_calls"] = rig.calls
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    log("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
