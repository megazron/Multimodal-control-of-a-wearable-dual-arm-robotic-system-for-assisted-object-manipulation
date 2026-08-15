#!/usr/bin/env python3
"""DRIVE THE COMMANDED EE STRAIGHT AT THE WEARER AND SEE WHAT THE ARM DOES.

    python3 scripts/verify_predictive_avoidance.py [--repeats 3]

WHAT THIS PROVES, AND WHAT IT DOES NOT. It exercises
`srl_teleop.predictive_avoidance` against the REAL solver and the REAL wearer
model, marching a commanded end-effector pose from a safe start directly into
the head and then into the torso. At every step it computes:

    the BASELINE solution      -- what ik_follower_node would command today
    the NULL-SPACE FAMILY      -- the same EE pose from a fan of seeds
    the CHOICE                 -- what the avoider picks
    the EE RESIDUAL            -- how far the chosen solution's end effector
                                  actually sits from the one asked for

It does NOT run inside `ik_follower_node`. The avoider is not wired into the
follower yet, deliberately: that node gates motion near a person's chest, and
wiring an unvalidated strategy into it is the wrong order. This is the
validation that has to come first.

THE FOUR QUESTIONS THE BRIEF ASKS, ANSWERED FROM THE NUMBERS:

  minimum clearance achieved      per target, with and without avoidance
  did the EE still reach target   fraction of commanded poses that produced a
                                  usable solution
  does it ever lock up            a step is a LOCKUP only if nothing at all is
                                  publishable; degrading to a worse-but-legal
                                  pose is not a lockup and is counted apart
  can the operator feel it        the EE residual, which should be ~0 because
                                  every candidate solves the SAME pose

CONTROLS, and no report without them:
    a pose deep inside the torso     baseline clearance must be NEGATIVE, or
                                     the wearer model is not being consulted
    the fan must actually differ     the candidate solutions must not all be
                                     the same joint vector, or there is no
                                     null space being searched and any
                                     "improvement" is noise
    avoidance must never REDUCE      the chosen clearance must be >= the
      clearance                      baseline's at every step
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

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import (FK, link_names, as_msg,  # noqa: E402
                                    as_tuple)
from srl_teleop.predictive_avoidance import (                # noqa: E402
    Lookahead, choose, ee_residual, seed_fan)

OUT = os.path.join(ROOT, "recordings/baselines/predictive_avoidance.json")
# From mount_guard_node.WEARER, in world: the two things the brief names.
TARGETS = {"head": (0.0, 0.0, 1.645), "torso": (0.0, 0.0, 1.22)}
TRIGGER_M = 0.25          # start avoiding here; well above the floor
FAN_N = 7


def solve_seeded(node, arm, xyz, quat, seed):
    """One IK request from an EXPLICIT seed, and no fallback that ignores it.

    THIS HAD TO BE WRITTEN OUT. `Solver.solve_joints` seeds from the live
    `/joint_states` and perturbs index 2 itself, so it cannot be asked for a
    particular seed -- and the first version of this file fell back to it when
    a seeded call was unavailable. That fallback would have returned the SAME
    solution for every member of the fan, the null-space search would have
    been searching one point, and the "distinct solutions" control would have
    been the only thing standing between that and a confident wrong result.
    A seeded solve is the whole mechanism; it does not get a fallback.
    """
    from moveit_msgs.msg import PositionIKRequest, RobotState
    from moveit_msgs.srv import GetPositionIK
    from geometry_msgs.msg import Pose, PoseStamped
    import time as _t
    req = GetPositionIK.Request()
    r = PositionIKRequest()
    r.group_name = "%s_arm" % arm
    rs = RobotState()
    rs.joint_state.name = node.names(arm)
    rs.joint_state.position = [float(v) for v in seed]
    r.robot_state = rs
    r.avoid_collisions = False        # the WEARER is scored geometrically
    ps = PoseStamped()
    ps.header.frame_id = "world"
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation = quat
    ps.pose = p
    r.pose_stamped = ps
    r.timeout.nanosec = 100_000_000
    req.ik_request = r
    fut = node.ik.call_async(req)
    t0 = _t.monotonic()
    while not fut.done() and _t.monotonic() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.002)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        return None
    names = list(res.solution.joint_state.name)
    pos = list(res.solution.joint_state.position)
    idx = {nm: i for i, nm in enumerate(names)}
    want = node.names(arm)
    if not all(nm in idx for nm in want):
        return None
    return [float(pos[idx[nm]]) for nm in want]


def solve_with_clearance(rig, fk, arm, xyz, quat, seed):
    """(joints, clearance, part, ee_residual) for one seeded request."""
    j = solve_seeded(rig.n, arm, xyz, quat, seed)
    rig.calls += 1
    if j is None:
        return None, None, None, None
    c, part = rig.clearance(arm, j)
    p = fk.poses(arm, j, link_names(arm))
    resid = None
    if p is not None:
        ee = p["%s_end_effector_link" % arm][0]
        resid = ee_residual(tuple(ee), tuple(xyz))
    return j, c, part, resid


def march(rig, fk, arm, start, target, quat, steps, avoid_on):
    """Walk the commanded EE from `start` to `target`. Returns per-step rows."""
    rows = []
    la = Lookahead(horizon_s=0.30)
    t = 0.0
    for k in range(steps + 1):
        f = k / float(steps)
        xyz = tuple(start[i] + f * (target[i] - start[i]) for i in range(3))
        t += 0.1
        pred = la.update(t, xyz)
        # BASELINE: one solve from the live seed, which is what the follower
        # does today.
        seed = [rig.n.js.get(nm, 0.0) for nm in rig.n.names(arm)]
        base = solve_with_clearance(rig, fk, arm, xyz, quat, seed)
        if not avoid_on:
            rows.append(dict(step=k, xyz=[round(v, 4) for v in xyz],
                             clearance=None if base[1] is None
                             else round(base[1], 4),
                             part=base[2], action="baseline",
                             ee_resid=None if base[3] is None
                             else round(base[3], 5),
                             solved=base[0] is not None, candidates=1))
            continue
        # LOOKAHEAD: if the PREDICTED pose is the one that gets close, the
        # avoider should already be working by the time the arm is there.
        pred_clear = None
        if pred is not None:
            pj = solve_with_clearance(rig, fk, arm, pred[0], quat, seed)
            pred_clear = pj[1]
        cands = [(base[0], base[1], base[3])]
        distinct = 0
        if base[0] is not None:
            for s in seed_fan(base[0], FAN_N)[1:]:
                cj = solve_with_clearance(rig, fk, arm, xyz, quat, s)
                if cj[0] is not None:
                    cands.append((cj[0], cj[1], cj[3]))
                    if max(abs(cj[0][i] - base[0][i]) for i in range(7)) > 1e-3:
                        distinct += 1
        d = choose(cands, CLEAR_FLOOR, TRIGGER_M)
        rows.append(dict(step=k, xyz=[round(v, 4) for v in xyz],
                         clearance=None if d.clearance is None
                         else round(d.clearance, 4),
                         baseline_clearance=None if base[1] is None
                         else round(base[1], 4),
                         predicted_clearance=None if pred_clear is None
                         else round(pred_clear, 4),
                         part=base[2], action=d.action,
                         ee_resid=None if d.ee_residual_m is None
                         else round(d.ee_residual_m, 5),
                         solved=d.action != "refuse",
                         candidates=len(cands), distinct=distinct))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=14)
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
    rig.set_furniture(False)          # free space: this is about the WEARER
    fk = FK(n)
    arm = "left"
    quat = as_msg(as_tuple(rig.quat[arm]))

    # ------------------------------------------------------------ controls
    ctl, good = {}, True
    deep = solve_with_clearance(rig, fk, arm, (0.05, -0.02, 1.22), quat,
                                [n.js.get(k, 0.0) for k in n.names(arm)])
    ctl["deep_inside_torso_negative"] = (None if deep[1] is None
                                         else round(deep[1], 4))
    good = good and deep[1] is not None and deep[1] < 0.0

    print("CONTROLS")
    for k, v in ctl.items():
        print("   %-32s %s" % (k, v))
    if not good:
        print("\nREFUSING TO REPORT: a control failed.")
        json.dump(dict(refused=True, controls=ctl), open(a.out, "w"), indent=2)
        return 6

    start = (0.55, 0.36, 1.18)        # the home hand position
    res = {}
    print("\nDRIVING THE COMMANDED EE STRAIGHT AT THE WEARER, %d steps"
          % a.steps)
    for name, tgt in TARGETS.items():
        res[name] = {}
        for mode in ("off", "on"):
            rows = march(rig, fk, arm, start, tgt, quat, a.steps, mode == "on")
            solved = [r for r in rows if r["solved"]]
            clears = [r["clearance"] for r in rows if r["clearance"] is not None]
            resids = [r["ee_resid"] for r in rows if r.get("ee_resid")]
            acted = [r for r in rows if r.get("action") == "avoided"]
            refused = [r for r in rows if r.get("action") == "refuse"]
            res[name][mode] = dict(
                rows=rows, n=len(rows), solved=len(solved),
                min_clearance=None if not clears else round(min(clears), 4),
                avoided_steps=len(acted), refused_steps=len(refused),
                max_ee_residual_m=None if not resids else round(max(resids), 5),
                distinct_candidates=sum(r.get("distinct", 0) for r in rows))
            r = res[name][mode]
            print("   %-6s avoidance %-3s  min clearance %-8s  solved %2d/%2d "
                  " avoided %2d  refused %2d  max EE residual %s"
                  % (name, mode,
                     "----" if r["min_clearance"] is None
                     else "%.4f" % r["min_clearance"],
                     r["solved"], r["n"], r["avoided_steps"],
                     r["refused_steps"],
                     "----" if r["max_ee_residual_m"] is None
                     else "%.5f m" % r["max_ee_residual_m"]))

    # ---- the two controls that need the sweep to have happened -----------
    distinct = sum(res[t]["on"]["distinct_candidates"] for t in TARGETS)
    ctl["fan_produces_distinct_solutions"] = distinct
    never_worse = True
    for t in TARGETS:
        for r in res[t]["on"]["rows"]:
            b, c = r.get("baseline_clearance"), r.get("clearance")
            if b is not None and c is not None and c < b - 1e-6:
                never_worse = False
    ctl["avoidance_never_reduces_clearance"] = never_worse
    print("\n   fan produced %d distinct solutions (must be > 0, or there is "
          "no null space being searched)" % distinct)
    print("   avoidance never reduced clearance: %s" % never_worse)

    print("\n" + "=" * 72)
    print("ANSWER")
    print("=" * 72)
    for t in TARGETS:
        off, on = res[t]["off"], res[t]["on"]
        print("   %-6s min clearance %s -> %s   solved %d/%d -> %d/%d   "
              "lockups %d"
              % (t,
                 "----" if off["min_clearance"] is None
                 else "%.4f" % off["min_clearance"],
                 "----" if on["min_clearance"] is None
                 else "%.4f" % on["min_clearance"],
                 off["solved"], off["n"], on["solved"], on["n"],
                 on["refused_steps"]))
    print("\n   The EE residual is the number that says the operator cannot "
          "feel it.")

    out = dict(trigger_m=TRIGGER_M, floor_m=CLEAR_FLOOR, fan=FAN_N,
               steps=a.steps, controls=ctl, results=res, ik_calls=rig.calls,
               caveat="the avoider is exercised against the real solver and "
                      "the real wearer model, but is NOT wired into "
                      "ik_follower_node -- that node gates motion near a "
                      "person and this validation comes first")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
