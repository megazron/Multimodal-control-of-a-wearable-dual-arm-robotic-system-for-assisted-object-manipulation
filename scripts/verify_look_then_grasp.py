#!/usr/bin/env python3
"""LOOK, THEN GRASP: the whole sequence, driven by the camera. With timings.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    ros2 run srl_perception mock_rgbd_camera --ros-args -p arm:=left -p task:=t1 &
    python3 scripts/verify_look_then_grasp.py --arm left

WHAT IT DOES, in the order the robot does it:

    1. HOME -> VIA -> OBSERVE. Two segments, because a straight joint-space
       line from home dips to 0.1199 m against a 0.150 m wearer floor even
       though both endpoints are clear. `solve_observe_transit.py` supplies
       the via.
    2. DETECT. One frame from the wrist camera. Colour segmentation, cut at
       the depth step, size-gated at range, deprojected through the real
       intrinsics and TF.
    3. DECIDE FROM WHAT WAS SEEN. Each detection's own classified colour picks
       its destination pad, and its own deprojected position is the grasp
       point. The scene file is not consulted.
    4. GRASP. The pick and place waypoints are built from those numbers and
       every one is put through `/compute_ik`, collision-aware, N times.

THE CONTROL THAT MATTERS IS P-B: MOVE THE FILE, NOT THE OBJECT.
A pipeline that reads a detection and a pipeline that reads a coordinate look
identical from the outside as long as the two agree. So the scene's declared
cube coordinates are perturbed IN MEMORY after detection, and the plan is
rebuilt. If the plan moves, it was reading the file. It must not move.

TIMING is reported as the ADDED cost of looking: the observe move, the frame,
the classification, and the return, divided by the picks that one look serves.
One look serves every cube in frame, so the per-pick cost falls with the
number of cubes and that is stated rather than hidden.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

import verify_colour_vision as VCV                               # noqa: E402
from verify_task_scenes import Solver                            # noqa: E402
from measure_what_binds import Rig                               # noqa: E402
from measure_grasp_approach import as_msg                        # noqa: E402
import msc_clip_tasks as M                                       # noqa: E402
import clip_tasks as CT                                          # noqa: E402
import home_positions as hp                                      # noqa: E402
from srl_teleop import master_calibration as mc                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/look_then_grasp.json")


def plan_from_detections(dets, arm):
    """Pick and place waypoints derived ONLY from what the camera reported.

    The destination is chosen by the DETECTED colour, through the same
    `T1_PLANES_BY_ARM` table the task uses, so a mis-classified cube goes to
    the wrong pad -- which is the point. A perception error must be able to
    produce a wrong-colour PLACEMENT, or the colour is decorative.
    """
    pads = M.T1_PLANES_BY_ARM[arm]
    colours = M.PLANE_COLOURS            # ("blue", "green") -> pad 0, pad 1
    out = []
    for d in sorted(dets, key=lambda z: z["world"][0]):
        if d["colour"] not in colours:
            continue
        pad_i = colours.index(d["colour"])
        px, py = pads[pad_i]
        obj = [float(d["world"][0]), float(d["world"][1]), M.T1_Z]
        pick = CT.ee_for(obj, arm)
        place = CT.ee_for([px, py, M.T1_Z], arm)
        out.append(dict(colour=d["colour"], pad=pad_i,
                        object_seen=[round(v, 4) for v in obj],
                        pick=[round(float(v), 4) for v in pick],
                        place=[round(float(v), 4) for v in place],
                        waypoints=[
                            [pick[0], pick[1], pick[2] + M.STANDOFF],
                            list(pick),
                            [pick[0], pick[1], pick[2] + M.LIFT],
                            [place[0], place[1], place[2] + M.STANDOFF],
                            list(place),
                            [place[0], place[1], place[2] + M.LIFT]]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--settle-s", type=float, default=6.0)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    obs_f = os.path.join(ROOT,
                         "recordings/baselines/observe_pose_%s.json" % a.arm)
    via_f = os.path.join(ROOT,
                         "recordings/baselines/observe_transit_%s.json" % a.arm)
    for f in (obs_f, via_f):
        if not os.path.exists(f):
            print("missing %s" % f)
            return 2
    q_obs = np.array(json.load(open(obs_f))["solved"]["q"], float)
    vd = json.load(open(via_f))
    if not vd.get("via"):
        print("no via recorded for %s" % a.arm)
        return 3
    q_via = np.array(vd["via"], float)
    q_home = np.array(hp.load_home_radians(a.arm), float)

    rclpy.init()
    n = VCV.Vision(a.arm)
    n.spin(6.0)

    timing = {}
    t0 = time.time()
    ok1, e1 = n.stage(q_via, secs=3.0)
    ok2, e2 = n.stage(q_obs, secs=3.0)
    timing["move_home_to_observe_s"] = round(time.time() - t0, 2)
    print("1. HOME -> VIA -> OBSERVE   arrived=%s/%s (worst %.4f rad)  %.2f s"
          % (ok1, ok2, max(e1 or 0, e2 or 0),
             timing["move_home_to_observe_s"]))
    if not (ok1 and ok2):
        print("REFUSING: the arm is not at the observe pose.")
        return 4

    n.spin(a.settle_s)
    for _ in range(40):
        if n.rgb is not None and n.depth is not None and n.info is not None:
            break
        n.spin(1.0)
    if n.rgb is None or n.depth is None:
        print("no camera stream -- is mock_rgbd_camera running?")
        return 5

    t0 = time.time()
    img, dep = n.image(), n.depth_m()
    p_cam, R_wc = n.cam_pose()
    dets, rejected = VCV.classify(img, dep, n.info)
    for d in dets:
        d["world"] = [float(v) for v in
                      VCV.deproject(d, n.info, p_cam, R_wc)]
    timing["detect_and_classify_s"] = round(time.time() - t0, 3)
    print("2. DETECT                   %d cubes, %d blobs rejected by size, "
          "%.3f s" % (len(dets), len(rejected),
                      timing["detect_and_classify_s"]))

    gt = VCV.truth()
    sc = VCV.score(dets, gt)
    print("   classification            %d of %d correct, %d wrong colour, "
          "%d missed, worst %.4f m"
          % (sc["correct"], sc["n"], sc["wrong_colour"], sc["missed"],
             sc["worst_localisation_m"] or 0.0))

    plan = plan_from_detections(dets, a.arm)
    print("3. DECIDE FROM WHAT WAS SEEN  %d picks planned" % len(plan))
    for p in plan:
        print("      %-6s seen at %-24s -> pad %d at %s"
              % (p["colour"], p["object_seen"], p["pad"],
                 [round(v, 3) for v in p["place"][:2]]))

    # ---- CONTROL P-B: move the FILE, not the object -------------------
    saved = [list(c) for c in M.T1_CUBES]
    try:
        for c in M.T1_CUBES:
            c[0] += 0.25          # a quarter metre, far beyond any tolerance
        plan2 = plan_from_detections(dets, a.arm)
    finally:
        for c, s_ in zip(M.T1_CUBES, saved):
            c[0], c[1] = s_
    same = all(np.allclose(p1["pick"], p2["pick"], atol=1e-9)
               for p1, p2 in zip(plan, plan2)) and len(plan) == len(plan2)
    print("   CONTROL P-B: declared coordinates shifted 0.25 m -> plan %s"
          % ("UNCHANGED (reads the detection)" if same
             else "MOVED -- it is reading the file"))

    # ---- 4. are the detection-derived grasps executable? --------------
    #
    # MEASURED FROM BOTH SEEDS, BECAUSE THE SEED IS THE WHOLE QUESTION.
    # `/compute_ik` seeds from the LIVE joint state, so where the arm is
    # standing when the grasp is planned decides which null-space branch comes
    # back -- and the branches differ in how close the elbow passes to the
    # wearer. Planning straight off the observe pose, which is where the arm
    # is the instant the frame is taken, is the obvious thing to do and it is
    # the wrong thing to do.
    sol = Solver()
    sol.spin(5.0)
    rig = Rig(sol, "t1", 1)
    anchor = as_msg(tuple(mc.WORKSPACE_ORIENT[a.arm]))

    def check_plan():
        fails, total, worst = 0, 0, 1e9
        for p_ in plan:
            for w in p_["waypoints"]:
                total += 1
                for _ in range(a.repeats):
                    j = sol.solve_arm_joints(a.arm, [float(v) for v in w],
                                             anchor, avoid=True, tries=6)
                    if j is None:
                        fails += 1
                        break
                    c, _w = rig.clearance(a.arm, j)
                    if c is not None and c < worst:
                        worst = c
        return fails, total, worst

    t0 = time.time()
    f_obs, total, c_obs = check_plan()
    timing["verify_plan_s"] = round(time.time() - t0, 2)
    print("4. GRASP, planned FROM THE OBSERVE POSE")
    print("      %d of %d waypoints unsolvable at N=%d, worst clearance "
          "%.4f m  %s" % (f_obs, total, a.repeats, c_obs,
                          "" if c_obs >= 0.15 else "<-- WEARER BREACH"))

    t0 = time.time()
    n.stage(q_via, secs=3.0)
    n.stage(q_home, secs=3.0)
    timing["move_observe_to_home_s"] = round(time.time() - t0, 2)
    sol.spin(2.0)
    f_home, _t, c_home = check_plan()
    print("4b. GRASP, planned AFTER RETURNING HOME")
    print("      %d of %d waypoints unsolvable at N=%d, worst clearance "
          "%.4f m  %s" % (f_home, total, a.repeats, c_home,
                          "" if c_home >= 0.15 else "<-- WEARER BREACH"))
    fails, worst_c = f_home, c_home

    look = (timing["move_home_to_observe_s"]
            + timing["detect_and_classify_s"]
            + timing["move_observe_to_home_s"])
    per_pick = look / max(1, len(plan))
    print("\nADDED TIME FOR LOOKING")
    print("   home -> via -> observe      %6.2f s" %
          timing["move_home_to_observe_s"])
    print("   frame + classify            %6.3f s" %
          timing["detect_and_classify_s"])
    print("   observe -> via -> home      %6.2f s" %
          timing["move_observe_to_home_s"])
    print("   TOTAL per LOOK              %6.2f s" % look)
    print("   per PICK (%d cubes, one look serves all) %6.2f s"
          % (len(plan), per_pick))

    res = dict(arm=a.arm, timing=timing, added_per_look_s=round(look, 2),
               added_per_pick_s=round(per_pick, 2), picks=len(plan),
               classification=sc, plan=plan,
               control_pb_plan_unchanged=bool(same),
               plan_ik_failures=fails, plan_waypoints=total,
               plan_worst_clearance_m=round(float(worst_c), 4),
               planned_from_observe=dict(
                   ik_failures=f_obs, worst_clearance_m=round(float(c_obs), 4)),
               planned_from_home=dict(
                   ik_failures=f_home,
                   worst_clearance_m=round(float(c_home), 4)))
    json.dump(res, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    return 0 if (same and fails == 0 and worst_c >= 0.15) else 6


if __name__ == "__main__":
    sys.exit(main())
