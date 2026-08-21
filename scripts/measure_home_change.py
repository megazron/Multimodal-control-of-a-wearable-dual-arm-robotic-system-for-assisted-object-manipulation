#!/usr/bin/env python3
"""IF THE PRESENTATION POSE BECOMES HOME, CAN THE TASKS STILL BE PERFORMED?

    python3 scripts/measure_home_change.py --repeats 10

THE QUESTION, AND WHY IT IS TWO QUESTIONS. "Make the presentation pose the
home pose" sounds like one change and is two, because this repository derives
two different things from home:

  * WHERE THE ARM RESTS -- the joint angles in config/home_positions_*.txt and
    the URDF's initial_positions. This is the picture, and it is what the
    request is about.
  * THE PINNED ANCHOR -- `master_calibration.WORKSPACE_ORIENT`, the quaternion
    `run_abc.send()` writes into EVERY waypoint of EVERY task under EVERY
    mode. It was MEASURED at home, off TF, and it is the approach direction:
    30.7 deg (left) / 22.1 (right) above horizontal, the hand arriving from
    the near side and below.

**THEY ARE SEPARABLE IN CODE, AND THAT IS THE WHOLE ANSWER TO "IS IT THE POSE
OR THE APPROACH".** `WORKSPACE_ORIENT` and `WORKSPACE_CENTRE` are read ONCE,
in `master_pose_node.__init__`, to seed ROS parameters. Nothing recomputes
them from the live home. So moving home does not move the anchor unless
someone edits master_calibration.py, and a level home does not force a level
grasp -- exactly as `home_wrist_is_real.md` already says the two numbers
measure different things.

So the 2 x 2 is measured, not argued:

                        anchor STORED (30.7 deg)   anchor LEVEL (re-derived)
    home STORED              the platform today         --
    home PRESENTATION        the change as asked        the change if the
                                                        anchor follows home

HOW THE LEVEL-ANCHOR PATHS ARE BUILT, and this matters. Every task coordinate
is an EE pose derived through `ee_for()` from an OBJECT pose, using a pad
offset that is only valid AT THE ANCHOR ORIENTATION. Re-orienting the wrist
without re-deriving the wrist position would put the fingers somewhere else
and measure a task nobody would command. Because the orientation is constant
along a path and the pad offset is a constant in the EE frame, the correction
is exact and is a single per-arm translation:

    new_wrist = old_wrist + (R_stored - R_level) @ pad_offset_in_ee_frame

which holds the finger PADS on precisely the world trajectory the task
declares and moves the wrist to wherever the new orientation puts it.

CONTROLS, and no report without them:

    the arms are WHERE THE RUN CLAIMS       verified off /joint_states, not
                                            off the staging script's own
                                            report -- `stage_presentation_pose
                                            --home` has once reported success
                                            and moved nothing
    a waypoint driven into the torso        must measure NEGATIVE clearance
    home / anchor both STORED               must REPRODUCE the numbers
                                            measured on 2026-08-15: T1 clean
                                            at 0.1610 both arms, T0 four IK
                                            failures per arm, T2 0.0046 /
                                            0.0904, T3 0.0618 / 0.1610. A
                                            known answer, on the same
                                            instrument, from a run that is
                                            already committed
"""
import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
sys.path.insert(0, os.path.join(ROOT, "config"))

from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as M                                   # noqa: E402
from measure_what_binds import Rig, CLEAR_FLOOR              # noqa: E402
from measure_grasp_approach import (FK, pad_mid_in_ee, q_matrix,  # noqa: E402
                                    as_msg, as_tuple, link_names,
                                    deg_from_down)
from srl_teleop import master_calibration as mc              # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/home_change.json")

# The numbers the loaded-home cell has to come back with.
#
# RE-BASELINED 2026-08-15 WHEN THE HOME POSE CHANGED, and the old values are
# kept here rather than deleted because the whole point of a known answer is
# that it is checkable against history:
#
#     at the LEGACY home (committed in grasp_approach.json)
#         t0  (4, 0.0086) / (4, 0.0176)      t1  (0, 0.1610) / (0, 0.1610)
#         t2  (0, 0.0046) / (0, 0.0904)      t3  (0, 0.0618) / (0, 0.1610)
#
# The platform moved, so those are the answer to a question nobody can ask any
# more: `stored` now names the presentation pose. Re-baselined against
# recordings/baselines/home_change_applied.json.
#
# T2 IS DELIBERATELY NOT IN THE CONTROL. Its left arm is marginal at the new
# home -- 1 of 11 waypoints failed 1 of 10 draws -- and a cell that flips
# between runs makes a control that cries wolf, which is worse than no control
# because it teaches the reader to ignore the banner. T2's state is REPORTED in
# the table this script prints; it is just not used to certify the instrument.
KNOWN = {
    "t0": {"left": (4, 0.0078), "right": (4, 0.0126)},
    "t1": {"left": (0, 0.1609), "right": (0, 0.1610)},
    "t1s2": {"left": (0, 0.1610), "right": (0, 0.1016)},
    "t3": {"left": (0, 0.0618), "right": (0, 0.1610)},
}
# THE COMPARISON IS ONE-SIDED, AND ON PURPOSE. The committed 2026-08-15 run
# took the clearance of the LAST solution per waypoint; this one takes the
# WORST over all N. Worst-over-N can only be lower than one sample of the same
# distribution, so "a bit lower" is the expected result of the fix and "higher"
# or "far lower" are the failures. A symmetric tolerance flagged t2's right arm
# at 0.0799 against 0.0904 -- 10.5 mm lower, exactly the direction the change
# predicts -- and calling that a disagreement would have been the instrument
# marking its own correction as a fault.
KNOWN_ABOVE_TOL_M = 0.002      # how far ABOVE the committed value is allowed
KNOWN_BELOW_TOL_M = 0.030      # how far BELOW before it stops being sampling


def stage(pose, timeout_s, tries=3):
    """Command the arms to `pose` and RETURN NOTHING but the script's word.

    Deliberately not trusted: the caller verifies arrival off /joint_states.

    THE TIMEOUT IS GENEROUS AND THE MOVE IS RETRIED, because the transition
    that matters here is the longest one this rig ever makes -- 1.94 rad from
    the stored home to the presentation pose -- and it is INTERMITTENT at the
    default deadline. `stage_presentation_pose` caps the rate at 0.30 rad/s
    and its own docstring records that the sustainable rate has never been
    measured from below, so the deadline is a guess with a known-loose bound.
    Measured here: the same transition arrived on one run and missed by
    1.9445 rad on the next. A cell that cannot be staged must be REFUSED, not
    reported -- but it should not be refused for want of a few seconds.
    """
    last = (1, ["never ran"])
    for _ in range(tries):
        cmd = [sys.executable, os.path.join(HERE, "stage_presentation_pose.py"),
               "--timeout-s", "%.1f" % timeout_s]
        if pose == "stored":
            cmd.append("--home")
        r = subprocess.run(cmd, capture_output=True, text=True)
        txt = (r.stdout or r.stderr).strip().splitlines()[-3:]
        last = (r.returncode, txt)
        if r.returncode == 0:
            return last
    return last


def presentation_q():
    """The saved presentation pose, per arm, as absolute joint radians."""
    import home_positions as hp
    d = json.load(open(os.path.join(
        ROOT, "recordings/baselines/presentation_pose.json")))
    return {a: [float(v) for v in d["poses"][a]["q"]] for a in
            ("left", "right")}, hp


def task_paths(seed):
    """{task: {arm: [waypoints]}} for all five, from the task's OWN builder."""
    return {
        "t0": M.t0(),
        "t1": M.t1(),
        "t1s2": M.t1_stage2(seed),
        "t2": M.t2(),
        "t3": M.t3(),
    }


def distinct(wps):
    """Consecutive duplicates are HOLDS. Measuring each again buys nothing."""
    out, seen = [], set()
    for w in wps:
        k = tuple(round(float(v), 4) for v in w)
        if k not in seen:
            seen.add(k)
            out.append([float(v) for v in w])
    return out


def walk(rig, arm, wps, quat, repeats, shift):
    """(IK failures, worst clearance, which body, where).

    CLEARANCE ON EVERY SOLUTION, NOT ON THE LAST ONE. The first version of
    this took the clearance of whichever solve happened to run last, which is
    ONE sample of a 7-DOF null space that TRAC-IK re-seeds randomly -- so a
    cell could differ from another cell by 60 mm for no reason but the draw,
    and the seed is exactly what changes when home moves. The IK failure count
    was always solid (all N must succeed); the clearance was not, and the
    clearance is the number HARD CONSTRAINT 11 turns on.
    """
    worst, who, where, fails = 1e9, None, None, 0
    qm = as_msg(quat)
    for w in wps:
        p = [w[i] + shift[i] for i in range(3)]
        ok = True
        for _ in range(repeats):
            j = rig.n.solve_arm_joints(arm, p, qm, avoid=True, tries=6)
            rig.calls += 1
            if j is None:
                ok = False
                break
            c, k = rig.clearance(arm, j)
            if c is not None and c < worst:
                worst, who, where = c, k, list(p)
        if not ok:
            fails += 1
    return fails, (None if worst > 1e8 else worst), who, where


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--stage-timeout-s", type=float, default=40.0)
    ap.add_argument("--tasks", nargs="*", default=None,
                    help="subset of t0 t1 t1s2 t2 t3; the known-answer "
                         "control only covers whichever of them it can")
    ap.add_argument("--cells", nargs="*", default=None,
                    help="subset of home/anchor cells, e.g. "
                         "presentation/stored")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(8.0)
    if not n.ik.wait_for_service(timeout_sec=25.0):
        print("no /compute_ik -- is the sim up?")
        return 2
    rig = Rig(n, "t1", 1)
    fk = FK(n)

    pres_q, hp = presentation_q()
    stored_q = {a2: list(hp.load_home_radians(a2)) for a2 in ("left", "right")}
    want_q = {"stored": stored_q, "presentation": pres_q}

    # ---- the two anchors ------------------------------------------------
    anchors = {"stored": {a2: as_tuple(_q_from(mc.WORKSPACE_ORIENT[a2]))
                          for a2 in ("left", "right")}}
    pad_ee, level = {}, {}
    for arm in ("left", "right"):
        v, _ = pad_mid_in_ee(fk, arm, stored_q[arm])
        pad_ee[arm] = v
        p = fk.poses(arm, pres_q[arm], link_names(arm))
        level[arm] = p["%s_end_effector_link" % arm][1]
    anchors["level"] = level

    print("THE TWO ANCHORS, as tool elevation above horizontal")
    for name, q in anchors.items():
        print("   %-6s left %+6.2f deg   right %+6.2f deg"
              % (name, 90.0 - deg_from_down(q["left"]),
                 90.0 - deg_from_down(q["right"])))

    # The exact per-arm wrist shift that holds the PADS still when the
    # anchor changes. Zero for the stored anchor, by construction.
    shift = {"stored": {a2: (0.0, 0.0, 0.0) for a2 in ("left", "right")}}
    shift["level"] = {}
    for arm in ("left", "right"):
        d = ((q_matrix(anchors["stored"][arm]) - q_matrix(anchors["level"][arm]))
             @ np.asarray(pad_ee[arm]))
        shift["level"][arm] = tuple(float(v) for v in d)
        print("   level-anchor wrist shift, %-5s (%+.4f, %+.4f, %+.4f) m"
              % (arm, d[0], d[1], d[2]))

    # ---- control: the clearance model reports negative inside a person --
    # THE PROBE POINT MOVED BECAUSE THE MOUNT DID, and this control has been
    # REFUSING since 2026-08-18 as a result -- so this whole script, the one
    # that certifies "the tasks still work after a home change", could not
    # run through the mount change, the T1 re-layout or anything after them.
    #
    # It asked for [0.05, -0.02, 1.22]: deep inside the torso box
    # (x +/-0.18, y +/-0.11, z 0.98-1.46) and, once the mounts went 150 mm
    # outboard and 15 deg of yaw, out of the left arm's reach. IK returned
    # None, `c_t` was None, and the banner said the CLEARANCE MODEL cannot go
    # negative -- which is false and is about the IK.
    #
    # The control's purpose is that the model reads NEGATIVE inside a person.
    # Which interior point is incidental; being reachable is not. Measured on
    # the geometry that exists: [0.10, 0.00, 1.25] solves and reads
    # -0.1406 m. Still deep inside the chest, and now it can be asked.
    jt = n.solve_arm_joints("left", [0.10, 0.00, 1.25],
                            as_msg(anchors["stored"]["left"]),
                            avoid=False, tries=8)
    c_t = rig.clearance("left", jt)[0] if jt is not None else None
    print("\nCONTROL  a pose inside the torso: %s (must be < 0)"
          % ("None" if c_t is None else "%.4f m" % c_t))
    if c_t is None:
        print("REFUSING TO REPORT: the control point is UNREACHABLE, which "
              "is a fact about IK and the mount, not about clearance. Pick "
              "an interior point this arm can still solve.")
        return 6
    if c_t >= 0.0:
        print("REFUSING TO REPORT: the clearance model cannot go negative.")
        return 6

    paths = task_paths(a.seed)
    order = tuple(a.tasks) if a.tasks else ("t0", "t1", "t1s2", "t2", "t3")
    cells = [("stored", "stored"), ("presentation", "stored"),
             ("presentation", "level")]
    if a.cells:
        want = set(a.cells)
        cells = [c for c in cells if "%s/%s" % c in want]
    res, staged_ok = {}, {}

    for home_name, anchor_name in cells:
        key = "%s_home/%s_anchor" % (home_name, anchor_name)
        print("\n" + "=" * 72)
        print("HOME = %-12s   ANCHOR = %-6s" % (home_name, anchor_name))
        print("=" * 72)
        rc, tail = stage(home_name, a.stage_timeout_s)
        n.spin(3.0)
        # ---- control: ARE THE ARMS WHERE THIS RUN CLAIMS? -------------
        arrived = {}
        for arm in ("left", "right"):
            cur = [n.js.get(k, 0.0) for k in n.names(arm)]
            tgt = want_q[home_name][arm]
            off = max(abs((cur[i] - tgt[i] + math.pi) % (2 * math.pi) - math.pi)
                      for i in range(7))
            arrived[arm] = off
            print("   staged %-5s worst joint error %.4f rad %s"
                  % (arm, off, "" if off <= HOME_TOL_RAD else "  <-- NOT THERE"))
        staged_ok[key] = all(v <= HOME_TOL_RAD for v in arrived.values())
        if not staged_ok[key]:
            print("   staging said rc=%d %s" % (rc, tail))
            print("   REFUSING to measure this cell: the arms are not in the "
                  "pose it is named after.")
            res[key] = dict(refused="arms not staged", staged=arrived)
            continue

        res[key] = dict(staged={k: round(v, 4) for k, v in arrived.items()},
                        tasks={})
        for t in order:
            res[key]["tasks"][t] = {}
            for arm in ("left", "right"):
                wps = distinct(paths[t][arm])
                f, c, who, where = walk(rig, arm, wps,
                                        anchors[anchor_name][arm],
                                        a.repeats, shift[anchor_name][arm])
                res[key]["tasks"][t][arm] = dict(
                    waypoints=len(wps), ik_failures=f,
                    worst_clearance_m=None if c is None else round(c, 4),
                    to=who, at=where,
                    performable=bool(f == 0),
                    clears_floor=bool(c is not None and c >= CLEAR_FLOOR))
                print("   %-5s %-5s %3d wp  %2d IK failures  worst clearance "
                      "%s to %-12s %s"
                      % (t, arm, len(wps), f,
                         "----" if c is None else "%.4f" % c, who,
                         "" if f == 0 else "<-- NOT PERFORMABLE"))

    # ---- control: the stored/stored cell must reproduce a known answer --
    base = res.get("stored_home/stored_anchor", {})
    print("\n" + "=" * 72)
    print("KNOWN-ANSWER CONTROL -- stored/stored against 2026-08-15")
    print("=" * 72)
    agree = True
    for t, per in KNOWN.items():
        for arm, (kf, kc) in per.items():
            got = base.get("tasks", {}).get(t, {}).get(arm)
            if got is None:
                agree = False
                print("   %-5s %-5s MISSING" % (t, arm))
                continue
            gc = got["worst_clearance_m"]
            ok = (got["ik_failures"] == kf and gc is not None
                  and gc <= kc + KNOWN_ABOVE_TOL_M
                  and gc >= kc - KNOWN_BELOW_TOL_M)
            agree = agree and ok
            print("   %-5s %-5s want %d fail / <=%.4f   got %d / %s   %s"
                  % (t, arm, kf, kc + KNOWN_ABOVE_TOL_M, got["ik_failures"],
                     "----" if gc is None else "%.4f" % gc,
                     "ok" if ok else "DISAGREES"))
    if not agree:
        print("\n   The instrument does not reproduce a committed measurement."
              "\n   Everything below is an instrument check, not a finding.")

    # ---- the answer ------------------------------------------------------
    print("\n" + "=" * 72)
    print("CAN EACH TASK STILL BE PERFORMED?")
    print("=" * 72)
    head = {("stored", "stored"): "home STORED/anchor STORED",
            ("presentation", "stored"): "home PRES/anchor STORED",
            ("presentation", "level"): "home PRES/anchor LEVEL"}
    print("   %-6s %s" % ("task", "  ".join("%-26s" % head[c] for c in cells)))
    verdict = {}
    for t in order:
        row = []
        for home_name, anchor_name in cells:
            k = "%s_home/%s_anchor" % (home_name, anchor_name)
            d = res.get(k, {}).get("tasks", {}).get(t)
            if d is None:
                row.append("--")
                continue
            f = sum(d[arm]["ik_failures"] for arm in ("left", "right"))
            cs = [d[arm]["worst_clearance_m"] for arm in ("left", "right")
                  if d[arm]["worst_clearance_m"] is not None]
            row.append("%s  %d fail, clr %s"
                       % ("OK  " if f == 0 else "FAIL",
                          f, "----" if not cs else "%.4f" % min(cs)))
        verdict[t] = row
        print("   %-6s %s" % (t, "  ".join("%-26s" % v for v in row)))

    out = dict(repeats=a.repeats, seed=a.seed, floor=CLEAR_FLOOR,
               anchors={k: {a2: [round(float(x), 6) for x in v[a2]]
                            for a2 in ("left", "right")}
                        for k, v in anchors.items()},
               level_anchor_wrist_shift_m={
                   a2: [round(v, 4) for v in shift["level"][a2]]
                   for a2 in ("left", "right")},
               known_answer_control_agrees=bool(agree),
               staged_ok=staged_ok, cells=res, ik_calls=rig.calls,
               method="every distinct waypoint of the task's own builder, "
                      "N=repeats, avoid_collisions with the task furniture "
                      "applied, wearer clearance from the mount guard's "
                      "capsule model on the returned solution; the "
                      "level-anchor paths are shifted per arm so the finger "
                      "PADS stay on the trajectory the task declares")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=2)
    print("\n%d IK calls -> %s" % (rig.calls, a.out))
    n.destroy_node()
    rclpy.shutdown()
    return 0


def _q_from(t):
    q = Quaternion()
    q.x, q.y, q.z, q.w = (float(v) for v in t)
    return q


if __name__ == "__main__":
    sys.exit(main())
