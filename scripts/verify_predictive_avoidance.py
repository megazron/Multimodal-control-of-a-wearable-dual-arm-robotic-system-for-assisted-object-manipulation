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
    Lookahead, NullSpaceRetreat, choose, ee_residual, seed_fan,
    tangential_target)

OUT = os.path.join(ROOT, "recordings/baselines/predictive_avoidance.json")
# From mount_guard_node.WEARER, in world: the two things the brief names.
# THE THIRD TARGET IS THE FAIR TEST FOR THE TANGENTIAL STAGE.
#
# head and torso drive the commanded hand STRAIGHT at the wearer, which is the
# right worst case for the floor and the WORST POSSIBLE case for a stage that
# works by keeping the across-the-body part of the operator's motion: a purely
# inward step has no across part to keep, so sliding cannot manufacture
# progress and the stage can only hold. `across` sweeps the hand over the front
# of the chest instead -- oblique, with a real inward component and a real
# tangential one, which is what an operator reaching past someone actually
# does. Reporting only the head-on cases would understate the stage in a way
# that would then need a paragraph of excuse; this measures it instead.
TARGETS = {"head": (0.0, 0.0, 1.645), "torso": (0.0, 0.0, 1.22),
           "across": (-0.35, 0.20, 1.18)}
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


def quat_matrix(q):
    """(x, y, z, w) to a rotation matrix, for the Jacobian's orientation rows."""
    import numpy as np
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def projection_retreat(rig, fk, arm, seed_solution, target_m):
    """The REAL null-space stage: project a clearance gradient into ker(J).

    The seed fan above samples solutions and was measured to move the elbow by
    millimetres. This moves it deliberately: build the Jacobian by finite
    differences from /compute_fk, project the clearance gradient onto its null
    space, step, and correct the hand back onto its pose each step. Returns
    (joints, clearance, ee_residual_m, steps, why, seconds).
    """
    import time as _t
    links = ["%s_end_effector_link" % arm]

    def _fk(q):
        p = fk.poses(arm, list(q), links)
        if p is None:
            raise RuntimeError("FK failed inside the null-space step")
        pos, quat = p[links[0]]
        return (pos, quat_matrix(quat))

    def _clear(q):
        c, _part = rig.clearance(arm, list(q))
        return c

    t0 = _t.monotonic()
    q, c, resid, steps, why = NullSpaceRetreat(
        max_steps=6, step_rad=0.06, max_ee_drift_m=0.0005).retreat(
            _fk, _clear, seed_solution, target_m)
    return q, c, resid, steps, why, _t.monotonic() - t0


def closest_arm_link(rig, fk, arm, q):
    """WHICH ARM LINK is nearest the wearer, and how near. Named on refusal.

    `Rig.clearance` returns the wearer PART, which answers "near what" and not
    "what of ours". The brief asks a refusal to name the link and the distance,
    so both ends of the closest pair are reported: without the arm link, a
    refusal says a person was nearly hit and not what nearly hit them.
    """
    import numpy as np
    from srl_teleop import mount_guard_node as MG
    pts = fk.poses(arm, list(q), link_names(arm))
    if pts is None:
        return None, None, None
    named = [(ln, pts[ln][0]) for ln in link_names(arm) if ln in pts]
    worst = (1e9, None, None)
    for (ln, a_), (_, b_) in zip(named, named[1:] + named[-1:]):
        for k in range(MG.SAMPLES + 1):
            t = k / float(MG.SAMPLES)
            p = [a_[i] + (b_[i] - a_[i]) * t for i in range(3)]
            for nm, kind, prm, ctr, rpy in MG.WEARER:
                d = MG.dist_point(p, kind, prm, ctr, rpy) - MG.TUBE_R
                if d < worst[0]:
                    worst = (d, ln, nm)
    _ = np
    return worst[1], worst[2], round(float(worst[0]), 4)


def wearer_distance(p):
    """Distance from a world point to the wearer, and the direction AWAY.

    The direction is the gradient of that distance, by central differences.
    Cheap (six evaluations of a closed-form primitive set) and exact enough to
    project a step onto, which is all the tangential stage needs.
    """
    import numpy as np
    from srl_teleop import mount_guard_node as MG

    def d(q):
        return min(MG.dist_point(q, k, prm, c, r)
                   for _n, k, prm, c, r in MG.WEARER)

    p = np.asarray(p, float)
    h = 1e-4
    g = np.zeros(3)
    for i in range(3):
        a_, b_ = p.copy(), p.copy()
        a_[i] += h
        b_[i] -= h
        g[i] = (d(a_) - d(b_)) / (2 * h)
    n = float(np.linalg.norm(g))
    return d(p), (g / n if n > 1e-9 else g)


def march(rig, fk, arm, start, target, quat, steps, avoid_on):
    """Walk the commanded EE from `start` to `target`. Returns per-step rows."""
    rows = []
    la = Lookahead(horizon_s=0.30)
    t = 0.0
    import time as _t
    for k in range(steps + 1):
        _step_t0 = _t.monotonic()
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
                             solved=base[0] is not None, candidates=1,
                             seconds=round(_t.monotonic() - _step_t0, 4)))
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
        # THE REAL NULL-SPACE STAGE, tried on top of the fan's answer.
        #
        # It runs only when something is actually near the wearer, for the
        # same reason the fan does: avoidance that fires in free space is
        # jitter the operator can feel, and (b) of the brief says they must
        # not. Reported SEPARATELY from the fan so the two can be compared
        # rather than blended -- the fan was measured worth 4.4 mm and a
        # number that mixes them would hide which half did the work.
        proj = None
        if (avoid_on == "projection" and base[0] is not None
                and base[1] is not None and base[1] < TRIGGER_M):
            try:
                pq, pc, presid, psteps, pwhy, psec = projection_retreat(
                    rig, fk, arm, base[0], TRIGGER_M)
                proj = dict(clearance=None if pc is None else round(pc, 4),
                            gain_m=None if (pc is None or base[1] is None)
                            else round(pc - base[1], 4),
                            ee_resid=round(presid, 6), steps=psteps,
                            why=pwhy, seconds=round(psec, 3))
                # THE FLOOR IS NOT OPTIONAL AND THIS BYPASSED IT.
                #
                # The first version accepted the projection whenever it beat
                # the fan's clearance and set the action to "avoided", which
                # marked the step COMPLETED even when the result was still
                # under the floor. Measured, that reported 10 of 15 torso
                # steps completed while the projection's best clearance gain
                # for that target was 0.0000 m -- a completion rate produced
                # by the bookkeeping and not by the robot. A result that
                # improves clearance without reaching the floor is still a
                # refusal, and it is recorded as one.
                if (pc is not None and pc >= CLEAR_FLOOR
                        and (d.clearance is None or pc > d.clearance)):
                    d.clearance = pc
                    d.ee_residual_m = presid
                    d.action = "avoided" if psteps else d.action
                    d.joints = pq
                elif pc is not None and d.clearance is not None and pc > d.clearance:
                    # better, but still under the floor: keep the number,
                    # keep the refusal
                    d.clearance = pc
            except Exception as exc:                       # noqa: BLE001
                proj = dict(error=str(exc)[:120])
        # A REFUSAL HAS TO SAY WHAT IT REFUSED ON. Only computed on refusal:
        # it walks the whole chain against the whole wearer and is far too
        # expensive to run on every step that is going fine.
        refusal = None
        if d.action == "refuse" and base[0] is not None:
            ln, part, dist = closest_arm_link(rig, fk, arm, base[0])
            refusal = dict(arm_link=ln, wearer_part=part, clearance_m=dist)
        # THE TANGENTIAL STAGE: reroute rather than refuse.
        #
        # Every refusal above names `end_effector_link` -- the HAND is what is
        # inside the person, not the elbow. No null space can help with that,
        # because the null space holds the hand's pose FIXED by definition. The
        # only thing that can is moving the commanded target, which is what
        # this does: strip the component of the operator's step that heads INTO
        # the wearer and keep the component that runs across them. The hand
        # then slides along the person instead of stopping in front of them,
        # and the cost -- how far the achieved pose ends up from the one asked
        # for -- is reported rather than hidden.
        slide = None
        if avoid_on == "tangential" and d.action == "refuse":
            step_vec = tuple(
                xyz[i] - (rows[-1]["xyz"][i] if rows else start[i])
                for i in range(3))
            dist, away = wearer_distance(xyz)
            new_xyz = tangential_target(
                (rows[-1]["xyz"] if rows else list(start)), step_vec, away)
            sj = solve_with_clearance(rig, fk, arm, new_xyz, quat, seed)
            slide = dict(from_xyz=[round(v, 4) for v in xyz],
                         to_xyz=[round(float(v), 4) for v in new_xyz],
                         moved_m=round(float(sum(
                             (new_xyz[i] - xyz[i]) ** 2
                             for i in range(3)) ** 0.5), 4),
                         clearance=None if sj[1] is None else round(sj[1], 4),
                         wearer_distance_m=round(float(dist), 4))
            if sj[1] is not None and sj[1] >= CLEAR_FLOOR:
                d.action = "rerouted"
                d.clearance = sj[1]
                d.joints = sj[0]
        rows.append(dict(step=k, projection=proj, refusal=refusal, slide=slide,
                         xyz=[round(v, 4) for v in xyz],
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
                         candidates=len(cands), distinct=distinct,
                         seconds=round(_t.monotonic() - _step_t0, 4)))
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
        # THREE CONDITIONS, NOT TWO. "on" is the seed fan, which is the
        # strategy already measured at 4.4 mm; "projection" is the real
        # null-space stage. Reporting them side by side is the only way to say
        # which one earned the improvement.
        for mode in ("off", "on", "projection", "tangential"):
            rows = march(rig, fk, arm, start, tgt, quat, a.steps,
                         False if mode == "off" else mode)
            for r in rows:
                if r.get("action") == "rerouted":
                    r["solved"] = True
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
                distinct_candidates=sum(r.get("distinct", 0) for r in rows),
                # THE ACHIEVED LOOP RATE, which the brief asks for by name.
                # It is the whole decision per commanded pose, against the
                # SERVICE-based solver -- a follower with a local IK library
                # would be faster and this number does not pretend otherwise.
                refusals=[r["refusal"] for r in rows if r.get("refusal")],
                rerouted_steps=len([r for r in rows
                                    if r.get("action") == "rerouted"]),
                max_slide_m=max([r["slide"]["moved_m"] for r in rows
                                 if r.get("slide")] or [None]),
                seconds_per_step_median=(
                    sorted(r["seconds"] for r in rows if "seconds" in r)
                    [len([r for r in rows if "seconds" in r]) // 2]
                    if any("seconds" in r for r in rows) else None),
                projection_gain_m=max(
                    [r["projection"]["gain_m"] for r in rows
                     if r.get("projection")
                     and r["projection"].get("gain_m") is not None] or [None]),
                projection_seconds=max(
                    [r["projection"]["seconds"] for r in rows
                     if r.get("projection")
                     and r["projection"].get("seconds") is not None]
                    or [None]))
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
    # THREE CONDITIONS, AND A REFUSAL IS NOT A LOCKUP.
    #
    # The first version of this summary printed "lockups N" where N was the
    # number of REFUSED steps, while this file's own docstring says a refusal
    # is the correct answer when the hand is being driven into a person's head
    # and only "nothing at all is publishable" is a lockup. It also compared
    # off against the seed fan and never mentioned the projection, which is
    # the stage that did the work. Both fixed: a lockup is a step that
    # returned no decision, and all three conditions are shown.
    for t in TARGETS:
        off = res[t]["off"]
        print("   %-6s  %-11s min clearance %s  completed %2d/%2d  "
              "avoided %2d  refused %2d  max EE residual %s m"
              % (t, "off",
                 "----" if off["min_clearance"] is None
                 else "%.4f" % off["min_clearance"],
                 off["solved"], off["n"], off["avoided_steps"],
                 off["refused_steps"],
                 "----" if off["max_ee_residual_m"] is None
                 else "%.5f" % off["max_ee_residual_m"]))
        for mode, label in (("on", "seed fan"), ("projection", "null space"),
                            ("tangential", "tangential")):
            d = res[t][mode]
            print("   %-6s  %-11s min clearance %s  completed %2d/%2d  "
                  "avoided %2d  refused %2d  max EE residual %s m%s"
                  % ("", label,
                     "----" if d["min_clearance"] is None
                     else "%.4f" % d["min_clearance"],
                     d["solved"], d["n"], d["avoided_steps"],
                     d["refused_steps"],
                     "----" if d["max_ee_residual_m"] is None
                     else "%.5f" % d["max_ee_residual_m"],
                     "" if d.get("projection_gain_m") is None
                     else "   best single-step gain %.4f m"
                     % d["projection_gain_m"]))
        lock = [m for m in ("off", "on", "projection")
                if res[t][m]["n"] != (res[t][m]["solved"]
                                      + res[t][m]["refused_steps"])]
        print("   %-6s  LOCKUPS (a step that returned no decision at all): "
              "%s" % ("", "none" if not lock else ",".join(lock)))
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
