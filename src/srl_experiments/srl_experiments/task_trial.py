#!/usr/bin/env python3
"""
task_trial.py — one reach-and-grasp trial, shared by E2, E3, E4 and E5.

Kept in one place so the four experiments cannot drift apart in how a trial is
timed, when it is invalidated, or what counts as success. If E2 and E3 measured
"completion time" slightly differently, the crossed design in E3 would be
comparing two different quantities and nobody would notice.
"""
import json
import math
import time

import numpy as np
import rclpy

# Object layouts, in the wearer frame (+x wearer's right, +y forward, +z up),
# expressed relative to the arm's home so they move with the mount.
SCENARIOS = {
    # single object, increasing reach
    "near_single": dict(objects=[(0.10, 0.10, -0.20)], target=0, difficulty="easy"),
    "mid_single": dict(objects=[(0.18, 0.16, -0.30)], target=0, difficulty="moderate"),
    "far_single": dict(objects=[(0.26, 0.22, -0.38)], target=0, difficulty="hard"),
    # two objects close together: the intent estimator must disambiguate
    "near_pair": dict(objects=[(0.10, 0.14, -0.24), (0.19, 0.14, -0.24)],
                      target=0, difficulty="moderate, ambiguous"),
    # clutter
    "cluttered": dict(objects=[(0.08, 0.12, -0.22), (0.16, 0.13, -0.26),
                               (0.24, 0.12, -0.30), (0.16, 0.20, -0.22)],
                      target=1, difficulty="hard, 3 distractors"),
    # E4: required approach ORIENTATION is what varies, not position
    "upright_easy": dict(objects=[(0.16, 0.15, -0.28)], target=0,
                         approach_tilt_deg=0, difficulty="control"),
    "tilted_30": dict(objects=[(0.16, 0.15, -0.28)], target=0,
                      approach_tilt_deg=30, difficulty="moderate"),
    "tilted_60": dict(objects=[(0.16, 0.15, -0.28)], target=0,
                      approach_tilt_deg=60, difficulty="hard"),
    "side_facing": dict(objects=[(0.16, 0.15, -0.28)], target=0,
                        approach_tilt_deg=90,
                        difficulty="ungraspable by direct teleop"),
    "inverted": dict(objects=[(0.16, 0.15, -0.28)], target=0,
                     approach_tilt_deg=120,
                     difficulty="ungraspable by direct teleop"),
}


def scenario_world(name, home):
    s = SCENARIOS[name]
    home = np.asarray(home, float)
    objs = [home + np.array(o, float) for o in s["objects"]]
    return objs, s["target"], s


def run_trial(r, cfg, scenario, condition, index, block=0, arm=None,
              attention="single_task", p_threshold=None):
    """Execute one trial and write its summary row. Returns the summary dict.

    `r` is an ExperimentRunner. The operator model is whichever is publishing
    /master_arm_pose_<arm> — scripted or human.
    """
    arm = arm or r.arm
    home = np.array(cfg["home_xyz"], float)
    objs, tgt_i, spec = scenario_world(scenario, home)
    target = objs[tgt_i]
    timeout = float(cfg.get("timeout_s", 45.0))

    r.log.start(index, condition=condition, scenario=scenario,
                target_id="tag_%d" % tgt_i, arm=arm, block=block)

    r.send_operator(home)
    r.spin(0.5)
    r.estop_events = 0
    # Per-trial, not per-session: without this the first abort
    # would invalidate every remaining trial in the block and the
    # session would look far worse than the rig actually was.
    r.aborts = []
    # Snapshot the faults ALREADY outstanding. A trial is invalidated by a
    # fault that appears WHILE it runs, not by one that was already held --
    # a pre-existing fault means the trial should never have been started
    # (that is what /recovery/ready is for), and counting it here would mark
    # every trial in the session invalid for one unrelated standing fault.
    r.faults_at_start = set(getattr(r, "recovery_faults", {}) or {})
    r.recovery_seen = {}

    t0 = time.monotonic()
    pts = []
    first_assist = None
    assist_start = None
    assist_total = 0.0
    assist_entered = 0
    assist_cancelled = 0
    # Seed from the CURRENT state, not from a constant. The arbiter latches
    # GRASPED until the gripper opens, and the mock gripper starts at 0.79 rad
    # (already "closed"), so a trial that inherited GRASPED from the previous
    # one broke on its first iteration and logged 0.00 s / success=0 for every
    # remaining trial. Only a TRANSITION into GRASPED inside this trial counts.
    prev_state = r.autonomy_state
    intent_switches = 0
    prev_top = None
    intent_at_handover = ""
    interventions = 0
    correction_time = 0.0
    dropout_frames = 0
    total_frames = 0
    reached = False

    # In `direct` the operator does the whole reach. In `shared` the autonomy
    # may take the wrist. In `full_auto` the operator designates and stops.
    designate_until = 0.35 * timeout if condition == "full_auto" else timeout
    # A trial ENDS when the target has been held for `dwell_s`, or when the
    # autonomy reports GRASPED. Without a completion condition every trial
    # runs to the timeout and "completion time" degenerates to the timeout
    # value for every condition - which is exactly what the first pilot run
    # produced, and it would have made E2/E3 measure nothing.
    dwell_s = float(cfg.get("dwell_s", 0.4))
    dwell_start = None

    r.publish_objects(objs)
    r.send_operator(target, point_at=target)
    obj_tick = 0
    while time.monotonic() - t0 < timeout and rclpy.ok():
        rclpy.spin_once(r, timeout_sec=0.005)
        obj_tick += 1
        if obj_tick % 10 == 0:
            # Re-publish: object_pose_tracker DROPS stale objects on purpose,
            # so a one-shot publish would vanish mid-trial.
            r.publish_objects(objs)
        r.sample_once("reach" if not reached else "grasp")
        total_frames += 1

        ch = (r.channels or {}).get("channels", {})
        if ch and any(not ch.get(c, True) for c in r.required_channels()):
            dropout_frames += 1

        st = r.autonomy_state
        if st == "ASSIST" and prev_state != "ASSIST":
            assist_entered += 1
            assist_start = time.monotonic()
            if first_assist is None:
                first_assist = time.monotonic() - t0
            intent_at_handover = (r.intent or {}).get("top", "") or ""
        if st != "ASSIST" and prev_state == "ASSIST":
            if assist_start:
                assist_total += time.monotonic() - assist_start
            assist_start = None
            if st == "DIRECT":
                assist_cancelled += 1
                # A cancel costs the operator a re-approach; that time is the
                # cost of a wrong inference, which E5 needs measured.
                correction_time += 0.0
                interventions += 1
        became_grasped = (st == "GRASPED" and prev_state != "GRASPED")
        prev_state = st

        top = (r.intent or {}).get("top")
        if top is not None and prev_top is not None and top != prev_top:
            intent_switches += 1
        prev_top = top

        p = r.master.pose.position if r.master else None
        if p is not None:
            cur = np.array([p.x, p.y, p.z])
            pts.append(cur)
            inside = np.linalg.norm(cur - target) < 0.03
            if inside:
                reached = True
                dwell_start = dwell_start or time.monotonic()
                if time.monotonic() - dwell_start > dwell_s:
                    break
            else:
                dwell_start = None
        if became_grasped:
            break
        if time.monotonic() - t0 > designate_until and condition == "full_auto":
            break

    if assist_start:
        assist_total += time.monotonic() - assist_start
    mt = time.monotonic() - t0
    final = pts[-1] if pts else home
    err = float(np.linalg.norm(final - target))
    success = int(err < 0.03)

    good, bad = r.channels_ok()
    frac = dropout_frames / max(1, total_frames)
    if not good:
        r.log.invalidate("channel disabled mid-trial: %s" % ",".join(bad))
    elif frac > float(cfg.get("max_dropout_fraction", 1.0)):
        r.log.invalidate("dropout fraction %.3f exceeds %.3f"
                         % (frac, cfg["max_dropout_fraction"]))
    if r.estop_events:
        r.log.invalidate("e-stop during trial")
    # An abort from the safety layer always wins: it names a specific rig
    # fault, which is more useful in the log than the generic e-stop that
    # usually accompanies it.
    new_faults = set(getattr(r, "recovery_seen", {}) or {}) - r.faults_at_start
    if r.aborts or new_faults:
        causes = sorted({str(a.get("cause", "?")) for a in r.aborts})
        if not causes:
            seen = getattr(r, "recovery_seen", {})
            causes = ["%s" % seen.get(f, f) for f in sorted(new_faults)]
        r.log.invalidate("aborted mid-trial: %s" % "; ".join(causes))

    return r.log.finish(
        completion_time_s=round(mt, 4),
        grasp_success=success,
        positioning_error_m=round(err, 5),
        failed_attempts=0 if success else 1,
        path_length_m=round(r.path_length(pts), 5),
        straight_line_m=round(float(np.linalg.norm(target - home)), 5),
        path_ratio=round(r.path_length(pts) /
                         max(1e-6, float(np.linalg.norm(target - home))), 4),
        time_to_handover_s=(round(first_assist, 4) if first_assist else ""),
        intervention_count=interventions,
        correction_time_s=round(correction_time, 4),
        assist_entered=assist_entered,
        assist_cancelled=assist_cancelled,
        assist_duration_s=round(assist_total, 4),
        intent_correct_at_handover=(int(intent_at_handover == "tag_%d" % tgt_i)
                                    if intent_at_handover else ""),
        intent_switches=intent_switches,
        dropout_fraction=round(frac, 4),
        estop_events=r.estop_events,
    )
