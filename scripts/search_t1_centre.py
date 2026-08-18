#!/usr/bin/env python3
"""HOW CLOSE TO THE CENTRELINE CAN A PAD GO, WITH THE OBJECTS ON THE SURFACE?

    python3 scripts/search_t1_centre.py                 # the whole search
    python3 scripts/search_t1_centre.py --self-test     # controls only
    python3 scripts/search_t1_centre.py --stage a       # the literal question

WHY THIS EXISTS, AND WHY IT IS NOT search_centre_on_surface.py

The recorded T1 layout puts its pads at |x| = 0.595 and 0.825 -- the table's
side edge -- while `recordings/baselines/centre_reach.json` records an
innermost workable column of x = 0.075 for the right arm. Both numbers are in
the repository and they are about DIFFERENT QUESTIONS, which is exactly the
condition CLAUDE.md's standing rule says to resolve by measurement rather than
by argument:

    centre_reach 0.075   IK ONLY, no clearance floor, work plane 1.120,
                         objects FLOATING 170 mm above the table, grasp pose
                         alone at N=3
    the layout 0.595     full pick path at N=10, wearer measured
                         geometrically, whole pad FOOTPRINT on surveyed cells

So this script asks ONE question in one instrument, and reports every rung of
it separately so the next reader cannot mistake an IK number for a safe one:

    per arm, objects RESTING on the surface, from the CURRENT home,
    N=10 over the FULL pick path, wearer and furniture in the scene:

        1. the innermost |x| that IK solves at all
        2. the innermost |x| that ALSO keeps the 150 mm wearer floor
        3. if (2) is outside the centre, WHAT WOULD MOVE IT -- table height,
           table distance, wearer arm posture, and the approach ORIENTATION

THE ORIENTATION IS A REAL VARIABLE HERE AND IT WAS NOT BEFORE. Every previous
sweep pinned the wrist at `master_calibration.WORKSPACE_ORIENT`, because every
teleop mode commands it and the pinned wrist is the thing the mode comparison
holds constant. A task built for **06_full_autonomy alone** commands no such
thing: under 06 the system supplies the whole pose. So the approach is swept
as a fan of (elevation, heading, roll) and the pinned anchor is carried in the
fan as one member, which keeps the comparison honest -- if the anchor wins,
the fan says so.

This does NOT move `WORKSPACE_ORIENT`. HARD CONSTRAINT 1 forbids re-deriving
the global anchor, and re-deriving it costs T2 its right arm. A per-grasp
approach used by one 06-only task is a different object from the global
constant, and the two are kept apart on purpose.

CONTROLS, and nothing is reported if one fails:

    a pose inside the wearer's torso        clearance must be NEGATIVE
    1.6 m out in front                      must be unreachable
    the SHIPPED T1 cube, floating           must be reachable AND >= the floor
    an object BURIED in the slab            must FAIL -- otherwise the table
                                            is not in the scene and every
                                            "on the surface" answer is a lie
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from audit_scenario_reachability import densify              # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR, _apply, _remove  # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee, q_matrix,   # noqa: E402
                                    as_msg, as_tuple)
from srl_teleop import wearer_posture as WP                  # noqa: E402
from srl_teleop import mount_guard_node as MG                # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_centre.json")
CUBE = 0.040
STANDOFF, LIFT = 0.10, 0.08
THICK = 0.035
HALF_X = 1.05
DEPTH = 0.62
TABLE_ID = "search_table"


# ---------------------------------------------------------------- the table
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


# ------------------------------------------------------- the approach fan
def _norm(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def q_from_matrix(R):
    """Rotation matrix -> (x, y, z, w), branch-safe at every trace."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def q_from_axis(axis, roll):
    """A wrist orientation whose TOOL AXIS (+z of the EE frame) is `axis`.

    `roll` spins the hand about that axis, which is what decides which pair of
    faces the pads close on. It is a free parameter of the grasp and this
    project has never swept it -- TASK_SPEC section 2 records T3's box failing
    under both approaches for want of exactly this roll.
    """
    a = _norm(axis)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.95 else \
        np.array([0.0, 1.0, 0.0])
    x = _norm(np.cross(ref, a))
    y = np.cross(a, x)
    R = np.column_stack([x, y, a])
    c, s = math.cos(roll), math.sin(roll)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return q_from_matrix(R @ Rz)


def axis_for(elev_deg, head_deg):
    """Unit approach direction: heading 0 is +y (away from the wearer)."""
    e, h = math.radians(elev_deg), math.radians(head_deg)
    return (math.sin(h) * math.cos(e), math.cos(h) * math.cos(e),
            math.sin(e))


def tool_elev_head(qt):
    a = q_matrix(qt)[:, 2]
    return (math.degrees(math.asin(max(-1.0, min(1.0, a[2])))),
            math.degrees(math.atan2(a[0], a[1])))


def build_fan(rig, elevs, heads, rolls):
    """[(name, {arm: quat_tuple})], the pinned anchor first so it is compared
    rather than replaced."""
    fan = [("anchor(pinned)",
            {a: as_tuple(rig.quat[a]) for a in ("left", "right")})]
    for e in elevs:
        for h in heads:
            for r in rolls:
                # heading is MIRRORED for the right arm, so "30 deg outboard"
                # means the same thing on both. A single signed heading would
                # have measured one arm reaching across the body and the other
                # reaching away from it, and called them the same cell.
                # ONE DECIMAL IN THE NAME. With %d a refinement pass at 12.5
                # deg and one at 12.0 deg would carry the SAME name, and the
                # name is the key `stage_b` looks the quaternion up by.
                fan.append(("e%+05.1f/h%+05.1f/r%05.1f" % (e, h, r),
                            {"left": q_from_axis(axis_for(e, h), math.radians(r)),
                             "right": q_from_axis(axis_for(e, -h),
                                                  math.radians(r))}))
    return fan


# ------------------------------------------------------------ one cell
def path_for(ee, q, along_axis):
    if along_axis:
        a = q_matrix(q)[:, 2]
        pre = list(np.asarray(ee, float) - STANDOFF * a)
    else:
        pre = [ee[0], ee[1], ee[2] + STANDOFF]
    up = [ee[0], ee[1], ee[2] + LIFT]
    return densify([pre, ee], 0.025) + densify([ee, up], 0.025)


def wrist_for(obj, q, pad_ee):
    return list(np.asarray(obj, float) - q_matrix(q) @ np.asarray(pad_ee))


def solve_cell(rig, arm, obj, q, pad_ee, repeats, full, postures=()):
    """(ok, worst clearance, who, {posture: worst clearance}).

    The pads land ON the object: the wrist target is the object minus the pad
    midpoint rotated into world, so this measures where the FINGERS go and not
    where the wrist goes. Measuring the wrist is how a grasp ends up 98 mm
    short of the thing it is grasping.
    """
    ee = wrist_for(obj, q, pad_ee)
    wps = path_for(ee, q, True) if full else [ee]
    qm = as_msg(q)
    worst, who = 1e9, None
    alt = {p: 1e9 for p in postures}
    for w in wps:
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, list(w), qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                return False, None, None, {}
            c, k = rig.clearance(arm, j)
            if c is None:
                return False, None, None, {}
            if c < worst:
                worst, who = c, k
            for p in postures:
                cp = clearance_vs(rig, arm, j, p)
                if cp is not None and cp < alt[p]:
                    alt[p] = cp
    return True, worst, who, {p: round(v, 4) for p, v in alt.items()}


_WEARER_CACHE = {}


def clearance_vs(rig, arm, joints, posture):
    """The same solution, scored against a DIFFERENT wearer arm posture.

    IK is not re-run: the URDF in the planning scene still carries the shipped
    posture, so this answers "would the metal be inside a wearer standing like
    that", not "would MoveIt have solved it". Said plainly because the two are
    not the same question and the difference is where the SRDF gap lives.
    """
    if posture not in _WEARER_CACHE:
        # THE NAMES ARE `wearer_posture.POSTURES`, NOT NAMES INVENTED HERE.
        # This asked for a posture called "clear", which CLAUDE.md's prose
        # uses for "arms held clear" and the module has never carried, and
        # `wearer_model` correctly raised on it -- 9 minutes into the sweep,
        # after stage 1 had already done its work. `none` IS the arms-removed
        # limit and it returns the torso parts on its own, so the special case
        # that used to be here was covering for the wrong name rather than for
        # a missing model.
        _WEARER_CACHE[posture] = WP.wearer_model(posture)
    model = _WEARER_CACHE[posture]
    saved = MG.WEARER
    try:
        MG.WEARER = model
        c, _ = rig.clearance(arm, joints)
    finally:
        MG.WEARER = saved
    return c


def nums_str(s):
    return [v.strip() for v in s.split(",") if v.strip()]


def frange(lo, hi, st):
    out, v = [], lo
    while v <= hi + 1e-9:
        out.append(round(v, 4))
        v += st
    return out


# ------------------------------------------------------------- controls
def controls(rig, pad_ee, floor):
    ctl, good = {}, True
    ql = as_msg(as_tuple(rig.quat["left"]))

    # THE CLEARANCE INSTRUMENT MUST READ NEGATIVE WITH THE HAND INSIDE THE
    # PERSON, and the point it is asked at must be one the arm can actually
    # reach or the control is measuring REACH instead.
    #
    # It was a single pose, [0.05, -0.02, 1.22] -- the middle of the torso box
    # (0.36 x 0.22 x 0.48 centred on (0, 0, 1.22)). After the 2026-08-18 mount
    # change moved both mounts 150 mm outboard, /compute_ik stopped returning
    # ANY solution for it even with avoid=False, so the control read None and
    # refused every run of every script that imports it. A control that fails
    # because the geometry moved tells you nothing about the instrument.
    #
    # So it walks a short list of points that are all unambiguously INSIDE the
    # torso box and uses the first one the arm can reach, naming it in the
    # result. Every candidate is >= 60 mm inside the box on x and inside it on
    # y and z, so a negative reading is the only correct answer for any of
    # them; which one gets used cannot change the verdict, only whether there
    # is one.
    inside = None
    for p_in in ([0.05, -0.02, 1.22], [0.05, 0.02, 1.22], [0.10, 0.02, 1.22],
                 [0.10, -0.02, 1.22], [0.14, 0.02, 1.22], [0.05, 0.06, 1.22]):
        j = rig.n.solve_arm_joints("left", list(p_in), ql, avoid=False, tries=8)
        if j is None:
            continue
        c = rig.clearance("left", j)[0]
        if c is not None:
            inside = (p_in, c)
            break
    ctl["inside_torso_is_negative"] = (
        "unreachable at every candidate" if inside is None
        else "%.4f at %s" % (inside[1], inside[0]))
    good = good and inside is not None and inside[1] < 0.0

    ctl["far_1.6m_unreachable"] = rig.n.solve_arm_joints(
        "left", [1.60, 0.35, 1.15], ql, avoid=True, tries=6) is None
    good = good and ctl["far_1.6m_unreachable"]

    # the SHIPPED cube, floating 120 mm over the table, on the real furniture
    rig.set_furniture(True)
    clear_table(rig)
    ok, cs, _, _ = solve_cell(rig, "left", [0.560, 0.120, 1.120],
                              as_tuple(rig.quat["left"]), pad_ee["left"],
                              1, True)
    ctl["shipped_cube_reachable_and_clear"] = "%s / %s" % (
        ok, None if cs is None else round(cs, 4))
    good = good and ok and cs is not None and cs >= floor

    # THE SLAB CONTROL IS CONSTRUCTED, not argued. Take one object pose,
    # solve it with the slab absent and again with the slab straight through
    # it. The ONLY thing that changed is the slab, so "reachable then, not
    # reachable now" is the slab being in the planning scene and nothing else.
    # A control that only asserts the second half cannot tell a slab that is
    # working from a pose that never solved.
    rig.set_furniture(False)
    buried = [0.560, 0.300, 0.9825]
    ql = as_tuple(rig.quat["left"])
    clear_table(rig)
    ok_free, _, _, _ = solve_cell(rig, "left", buried, ql, pad_ee["left"],
                                  1, True)
    put_table(rig, 1.00, 0.10)
    ok_b, _, _, _ = solve_cell(rig, "left", buried, ql, pad_ee["left"],
                               1, True)
    clear_table(rig)
    ctl["slab_control_free_then_buried"] = "%s -> %s (want True -> False)" % (
        ok_free, ok_b)
    good = good and ok_free and not ok_b
    return ctl, good


# ------------------------------------------------------------ stage A
def stage_a(rig, pad_ee, floor, xs, ys, repeats, table_top, near_y, log):
    """The literal question, on the CURRENT scene, at the pinned anchor.

    Two objects heights, because the repository's two answers were taken at
    two different ones and never compared: RESTING on the surface, which is
    what T1-1 asks for, and the shipped work plane 120 mm above it.
    """
    rows = []
    for label, z_obj, use_slab in (
            ("resting on the surface", round(table_top + CUBE / 2.0, 4), True),
            ("shipped work plane (floating)", 1.120, False)):
        if use_slab:
            rig.set_furniture(False)
            put_table(rig, table_top, near_y)
        else:
            clear_table(rig)
            rig.set_furniture(True)
        for arm in ("left", "right"):
            sgn = 1.0 if arm == "left" else -1.0
            hit_ik, hit_safe = None, None
            per_x = []
            for x in xs:
                best_ik, best_safe = None, None
                for y in ys:
                    obj = [round(sgn * x, 4), y, z_obj]
                    ok, c, who, _ = solve_cell(
                        rig, arm, obj, as_tuple(rig.quat[arm]), pad_ee[arm],
                        1, False)
                    if not ok:
                        continue
                    if best_ik is None:
                        best_ik = dict(y=y, clearance=round(c, 4), to=who)
                    if c >= floor:
                        ok2, c2, who2, _ = solve_cell(
                            rig, arm, obj, as_tuple(rig.quat[arm]),
                            pad_ee[arm], repeats, True)
                        if ok2 and c2 is not None and c2 >= floor:
                            best_safe = dict(y=y, clearance=round(c2, 4),
                                             to=who2)
                            break
                per_x.append(dict(x=x, ik=best_ik, safe=best_safe))
                if best_ik and hit_ik is None:
                    hit_ik = dict(x=x, **best_ik)
                if best_safe and hit_safe is None:
                    hit_safe = dict(x=x, **best_safe)
            rows.append(dict(objects=label, z_obj=z_obj, arm=arm,
                             min_x_ik=hit_ik, min_x_safe=hit_safe,
                             per_x=per_x))
            log("  %-30s %-5s  IK from |x|=%s   SAFE from |x|=%s"
                % (label, arm,
                   "----" if hit_ik is None else "%.3f" % hit_ik["x"],
                   "----" if hit_safe is None else "%.3f" % hit_safe["x"]))
    clear_table(rig)
    return rows


# ------------------------------------------------------------ stage B
def prune_fan(rig, pad_ee, floor, fan, probes, log, cap=14):
    """WHICH APPROACHES CAN TOUCH AN OBJECT ON A SURFACE AT ALL.

    The full grid is 55 approaches x 5 table heights x 2 overhangs x 7 forward
    distances x 25 columns x 2 arms, which is 192500 solves and two and a half
    hours. Most of that is spent proving over and over that an approach which
    cannot reach ONE object on ONE table cannot reach any of them.

    So the fan is pruned first against a handful of probe cells, and the prune
    is deliberately GENEROUS -- an approach survives if it reaches any probe
    with any clearance, floor or no floor. Pruning on the floor as well would
    let a single unlucky probe delete an approach that works 50 mm away.
    """
    # PROBE OUTER, FAN INNER. Applying the planning scene costs about half a
    # second and solving costs 45 ms, so a fan-outer loop spends nine tenths
    # of its time re-applying a table it just removed. With the probes outside
    # the slab is applied once per probe instead of once per (fan, probe).
    hits = {name: 0 for name, _q in fan}
    for (top, y, x, arm) in probes:
        rig.set_furniture(False)
        put_table(rig, top, round(max(0.02, y - CUBE / 2.0), 4))
        sgn = 1.0 if arm == "left" else -1.0
        obj = [round(sgn * x, 4), y, round(top + CUBE / 2.0, 4)]
        for name, quats in fan:
            ok, _c, _w, _ = solve_cell(rig, arm, obj, quats[arm], pad_ee[arm],
                                       1, False)
            hits[name] += 1 if ok else 0
    kept = [(name, quats, hits[name]) for name, quats in fan if hits[name]]
    clear_table(rig)
    kept.sort(key=lambda t: -t[2])
    log("   %d of %d approaches touch an object resting on a surface "
        "(%d probes each)" % (len(kept), len(fan), len(probes)))
    for name, _q, hit in kept[:cap]:
        log("      %-18s %d/%d probes" % (name, hit, len(probes)))
    if len(kept) > cap:
        # SAY WHAT WAS DROPPED. A cap that is not printed reads as "the fan was
        # searched" when it was truncated, which is the silent-truncation row
        # of CLAUDE.md's table.
        log("      ... CAPPED at %d of %d survivors, best-probe-count first. "
            "DROPPED: %s" % (cap, len(kept),
                             ", ".join(n for n, _q, _h in kept[cap:])))
    return [(n, q) for n, q, _h in kept[:cap]]


def stage_b(rig, pad_ee, floor, repeats, fan, tops, ohs, ys, xs, postures,
            log):
    """What would move it: table height, table distance, approach, posture."""
    log("\nSTAGE B1 -- grasp pose only, N=1, %d approaches x %d tops x %d "
        "overhangs x %d y x %d x x 2 arms = %d cells"
        % (len(fan), len(tops), len(ohs), len(ys), len(xs),
           len(fan) * len(tops) * len(ohs) * len(ys) * len(xs) * 2))
    hits = []
    for top in tops:
        for oh in ohs:
            for y in ys:
                near = round(max(0.02, y - CUBE / 2.0 - oh), 4)
                rig.set_furniture(False)
                put_table(rig, top, near)
                z_obj = round(top + CUBE / 2.0, 4)
                for name, quats in fan:
                    for arm in ("left", "right"):
                        sgn = 1.0 if arm == "left" else -1.0
                        for x in xs:
                            obj = [round(sgn * x, 4), y, z_obj]
                            ok, c, who, _ = solve_cell(
                                rig, arm, obj, quats[arm], pad_ee[arm],
                                1, False)
                            if ok and c is not None and c >= floor:
                                hits.append(dict(
                                    approach=name, arm=arm, table_top=top,
                                    near_y=near, overhang=oh, x=x, y=y,
                                    obj=obj, clearance=round(c, 4), to=who))
                                break        # innermost x per (cell, approach)
    clear_table(rig)
    log("   %d survivors at stage 1; %d IK calls so far" % (len(hits),
                                                            rig.calls))
    if not hits:
        return hits, []

    # Stage 2: the full path at N=repeats, on the innermost survivors. Take
    # the innermost per (arm) overall plus every survivor within 100 mm of it,
    # because "the innermost cell" alone has no margin and this project's own
    # rule is to stay 20 mm inside the last pose that passed N/N.
    cand = []
    for arm in ("left", "right"):
        mine = sorted([h for h in hits if h["arm"] == arm],
                      key=lambda d: (d["x"], -d["clearance"]))
        if not mine:
            continue
        lo = mine[0]["x"]
        cand += [h for h in mine if h["x"] <= lo + 0.100 + 1e-9][:120]
    log("\nSTAGE B2 -- FULL pick path, N=%d, wearer geometric, %d cells"
        % (repeats, len(cand)))
    confirmed = []
    for h in cand:
        rig.set_furniture(False)
        put_table(rig, h["table_top"], h["near_y"])
        q = dict(fan)[h["approach"]][h["arm"]]
        ok, c, who, alt = solve_cell(rig, h["arm"], h["obj"], q,
                                     pad_ee[h["arm"]], repeats, True,
                                     postures)
        h2 = dict(h, full_path_ok=ok,
                  full_clearance=None if c is None else round(c, 4),
                  full_to=who, by_posture=alt,
                  safe=bool(ok and c is not None and c >= floor))
        confirmed.append(h2)
        if h2["safe"]:
            log("   CONFIRMED %-5s |x|=%.3f y=%.3f top %.2f oh %.2f  %-18s "
                "clr %.4f" % (h["arm"], h["x"], h["y"], h["table_top"],
                              h["overhang"], h["approach"],
                              h2["full_clearance"]))
    clear_table(rig)
    return hits, confirmed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--floor", type=float, default=CLEAR_FLOOR)
    ap.add_argument("--stage", default="ab", choices=["a", "b", "ab"])
    # THE FAN IS A COMMAND-LINE OBJECT, because the first pass is a coarse
    # net and the second has to be a fine one AROUND WHAT IT CAUGHT. Hard
    # coding the grid would mean either a coarse answer or a sweep nobody
    # waits for.
    ap.add_argument("--elevs", default="-90,-75,-60,-45,-30,-15,0,15,30")
    ap.add_argument("--heads", default="0,25,50")
    ap.add_argument("--rolls", default="0,90")
    ap.add_argument("--tops", default="0.80,0.90,0.95,1.00,1.05")
    ap.add_argument("--ohs", default="0.00,0.06")
    ap.add_argument("--ys", default="0.15,0.20,0.25,0.30,0.35,0.40,0.45")
    ap.add_argument("--x-max", type=float, default=0.60)
    ap.add_argument("--x-step", type=float, default=0.025)
    ap.add_argument("--fan-cap", type=int, default=14)
    # VALIDATED AGAINST `wearer_posture.POSTURES` AT PARSE TIME, not 9 minutes
    # into the sweep. The first run of this script died in stage 2 on a
    # posture name that does not exist, having already spent 6926 IK calls.
    ap.add_argument("--postures", default="down,folded,none",
                    help="wearer arm postures to re-score the SAME solutions "
                         "against: %s" % ", ".join(sorted(WP.POSTURES)))
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    lines = []
    bad = [p for p in nums_str(a.postures) if p not in WP.POSTURES]
    if bad:
        print("REFUSING: %s is not a wearer posture. Known: %s"
              % (", ".join(bad), ", ".join(sorted(WP.POSTURES))))
        return 7

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
    pad_ee = {}
    for arm in ("left", "right"):
        home = [n.js.get(k, 0.0) for k in n.names(arm)]
        pad_ee[arm], _ = pad_mid_in_ee(fk, arm, home)

    t0 = time.time()
    ctl, good = controls(rig, pad_ee, a.floor)
    log("CONTROLS  (wearer posture in the URDF: %s)" % WP.posture_from_env())
    for k, v in ctl.items():
        log("   %-38s %s" % (k, v))
    if not good:
        log("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6
    for arm in ("left", "right"):
        e, h = tool_elev_head(as_tuple(rig.quat[arm]))
        log("   %s pinned anchor: tool axis elevation %+.1f deg, heading "
            "%+.1f deg" % (arm, e, h))
    if a.self_test:
        log("\nself-test only: the controls separate all four.")
        n.destroy_node()
        rclpy.shutdown()
        return 0

    res = dict(floor=a.floor, repeats=a.repeats, cube_m=CUBE,
               wearer_posture=WP.posture_from_env(), controls=ctl)

    if "a" in a.stage:
        log("\n" + "=" * 72)
        log("STAGE A -- THE LITERAL QUESTION, pinned anchor, current table")
        log("=" * 72)
        res["stage_a"] = stage_a(
            rig, pad_ee, a.floor,
            xs=frange(0.0, 0.70, 0.025),
            # OUT TO 0.55, because `centre_reach.json`'s x = 0.075 lives at
            # y >= 0.425 -- TASK_SPEC records the right arm's only near-centre
            # band as "y 0.425..0.475 only". A sweep that stopped at 0.40 would
            # have reported "no such cell" about a cell it never asked for.
            ys=frange(0.100, 0.550, 0.025),
            repeats=a.repeats, table_top=0.980, near_y=0.100, log=log)

    if "b" in a.stage:
        log("\n" + "=" * 72)
        log("STAGE B -- WHAT WOULD MOVE IT")
        log("=" * 72)
        nums = lambda s: [float(v) for v in s.split(",") if v.strip()]  # noqa
        tops = nums(a.tops)
        fan = build_fan(rig, elevs=nums(a.elevs), heads=nums(a.heads),
                        rolls=nums(a.rolls))
        log("\nSTAGE B0 -- prune the %d-member approach fan" % len(fan))
        _mid = tops[len(tops) // 2]
        fan = prune_fan(rig, pad_ee, a.floor, fan,
                        probes=[(_mid, 0.30, 0.35, "left"),
                                (_mid, 0.30, 0.35, "right"),
                                (min(tops), 0.35, 0.20, "left"),
                                (max(tops), 0.25, 0.45, "left"),
                                (_mid, 0.40, 0.10, "right")], log=log,
                        cap=a.fan_cap)
        hits, confirmed = stage_b(
            rig, pad_ee, a.floor, a.repeats, fan,
            tops=tops, ohs=nums(a.ohs), ys=nums(a.ys),
            xs=frange(0.0, a.x_max, a.x_step),
            postures=nums_str(a.postures), log=log)
        res["stage_b_hits"] = hits
        res["stage_b_confirmed"] = confirmed

    log("\n" + "=" * 72)
    log("ANSWER")
    log("=" * 72)
    conf = [h for h in res.get("stage_b_confirmed", []) if h["safe"]]
    for arm in ("left", "right"):
        mine = [h for h in conf if h["arm"] == arm]
        if mine:
            b = min(mine, key=lambda d: d["x"])
            log("   %-5s closest to centre, ON THE SURFACE, full path N=%d, "
                "clear of the 150 mm floor:" % (arm, a.repeats))
            log("        |x| = %.3f m  (%.0f mm off centre)   y = %.3f  "
                "table top %.2f  near edge %.3f" % (b["x"], b["x"] * 1000,
                                                    b["y"], b["table_top"],
                                                    b["near_y"]))
            log("        approach %s   clearance %.4f m to %s"
                % (b["approach"], b["full_clearance"], b["full_to"]))
        else:
            log("   %-5s NOTHING survives the full path anywhere in this "
                "sweep." % arm)
    res["elapsed_s"] = round(time.time() - t0, 1)
    res["ik_calls"] = rig.calls
    res["log"] = lines
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2)
    log("\n%d IK calls, %.0f s -> %s" % (rig.calls, res["elapsed_s"], a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
