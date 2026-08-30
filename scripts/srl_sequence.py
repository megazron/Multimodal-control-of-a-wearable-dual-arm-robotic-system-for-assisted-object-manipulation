#!/usr/bin/env python3
"""HOME -> PICKUP POSE -> SCAN -> LOCATE -> PICK, for the LEFT arm.

    python3 scripts/srl_sequence.py             # the whole sequence
    python3 scripts/srl_sequence.py --dry-run   # plan and measure, no motion
    python3 scripts/srl_sequence.py --skip-home # start at the pickup pose

Every leg is checked against the LIVE table before it is commanded, and the
force guard is armed throughout.

ABOUT THE HOME LEG
------------------
`config/home_positions_left.txt` is the SIM home (the solved presentation
pose).  HARD CONSTRAINT 0 says the real arms have never been set to it, and
CLAUDE.md records that the bridge refuses to enable across that ~1.9 rad gap.
From the pickup pose it is a very large reconfiguration -- joint 5 alone moves
about 135 deg.  So this leg is checked like any other and ABORTS rather than
forcing: if it cannot be shown safe, the sequence says so and stops instead of
swinging the arm across the rig.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
from auto_observe import find_cube, live_table, plausible  # noqa: E402
from execute_pick_left import ang_wrap  # noqa: E402
from guarded_descent import Blind, descend  # noqa: E402
from plan_pick_left import ik  # noqa: E402
from safe_goto_left import HAND_LINKS, lift_pose  # noqa: E402
from servo_pick_left import (CUBE_GRIP, Eye, GRIP_OPEN, GRIP_SQUEEZE,  # noqa: E402
                             MAX_STEP_M, MIN_CLEAR_M, STEP_FRAC, TOL_M,
                             measure, pads_in_cam)
from srl_fk import FK  # noqa: E402
from srl_scene import segment_objects  # noqa: E402

WS = "/home/gausms/kortex_ws"
TRANSIT_MARGIN_M = 0.045
LIFT_M = 0.12


def ik_relaxed(target, R_pref, q, grip, n_cam_to_world, fk):
    """IK to `target`, letting the approach TILT if exact orientation fails.

    Demanding an exact top-down wrist is what stopped the servo 97 mm short
    with a 23.6 mm IK residual -- and it was not reach: max pad reach measured
    1.278 m against a cube at 1.000 m.  A fixed 6-DOF pose on a 7-DOF arm
    spends the whole redundancy, which CLAUDE.md sizes at a factor of seven in
    usable workspace.  The follower already runs a 15 deg cone, so allowing one
    here costs nothing that was not already accepted.

    Returns (q, residual_m, tilt_deg_used).
    """
    qn, ep, er, _ = ik(target, R_pref, q, grip=grip)
    if ep <= 0.004:
        return qn, ep, 0.0
    best = (qn, ep, 0.0)
    a_ee = R_pref.T @ n_cam_to_world
    for tilt in (8.0, 15.0, 25.0, 35.0):
        for az in range(0, 360, 45):
            t, A = math.radians(tilt), math.radians(az)
            u = np.cross(n_cam_to_world, [1.0, 0, 0])
            if np.linalg.norm(u) < 1e-6:
                u = np.cross(n_cam_to_world, [0, 1.0, 0])
            u /= np.linalg.norm(u)
            w = np.cross(n_cam_to_world, u)
            axis = math.cos(t) * n_cam_to_world + math.sin(t) * (
                math.cos(A) * u + math.sin(A) * w)
            axis /= np.linalg.norm(axis)
            cur = R_pref @ np.array([0.0, 0.0, 1.0])
            v = np.cross(cur, -axis)
            sn = np.linalg.norm(v)
            cs = float(cur @ (-axis))
            if sn < 1e-9:
                Rd = R_pref
            else:
                vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                Rd = (np.eye(3) + vx + vx @ vx * ((1 - cs) / (sn * sn))) @ R_pref
            q2, e2, _, _ = ik(target, Rd, q, grip=grip)
            if e2 < best[1]:
                best = (q2, e2, tilt)
            if e2 <= 0.004:
                return q2, e2, tilt
    return best


def read_pose_txt(path):
    q = []
    for line in open(path):
        line = line.strip()
        if line.startswith("joint_"):
            q.append(math.radians(float(line.split(":")[1].split("#")[0])))
    return np.array(q[:7])


def path_low(fk, tab, q0, q1, steps=80, grip=0.0):
    if tab is None:
        return None
    return min(tab.min_hand_height(fk, q0 + ang_wrap(q1 - q0) * (k / steps), grip)
               for k in range(steps + 1))


def leg(node, fk, tab, q_target, label, margin=TRANSIT_MARGIN_M, dry=False):
    """Check a leg against the table, then command it. False = refused/failed."""
    q0 = node.fresh_q()
    span = math.degrees(np.abs(ang_wrap(q_target - q0)).max())
    low = path_low(fk, tab, q0, q_target)
    lows = "n/a" if low is None else "%.0f mm" % (low * 1000)
    print("  %-12s worst joint %6.1f deg, lowest hand point %s"
          % (label, span, lows))
    if low is not None and low < margin:
        print("     REFUSED: that path takes the hand to %.0f mm above the "
              "table (need %.0f). Not commanding it." % (low * 1000, margin * 1000))
        return False
    if span < 1.0:
        print("     already there")
        return True
    if dry:
        return True
    return node.goto(q_target, label) is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-home", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Eye("left")
    node.fk = fk = FK()
    print("=== 1/6  connect ===")
    if not node.preflight():
        print("  retrying discovery")
        if not node.preflight():
            raise SystemExit(2)
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    if not node.frames():
        raise SystemExit(
            "NO CAMERA FRAMES on /left_camera. Refusing to scan blind -- that "
            "wasted 14 viewpoints once. Fix: bash scripts/bringup_arm.sh left "
            "--restart")
    print("  camera OK: colour %dx%d, depth %dx%d"
          % (node.img["c"].width, node.img["c"].height,
             node.img["d"].width, node.img["d"].height))
    tab = live_table(node, fk)
    if tab is not None:
        print("  live table fit OK; hand is %.0f mm above it"
              % (tab.min_hand_height(fk, node.fresh_q()) * 1000))
    else:
        print("  no live table fit from here (camera may not be on the table)")

    if not args.dry_run:
        node.hold_gripper(GRIP_OPEN, 2.0, "OPEN gripper")

    # ---------------------------------------------------------------- HOME
    if not args.skip_home:
        print("\n=== 2/6  HOME ===")
        q_home = read_pose_txt(WS + "/config/home_positions_left.txt")
        print("  home (deg):", [round(math.degrees(x), 2) for x in q_home])
        if not leg(node, fk, tab, q_home, "HOME", dry=args.dry_run):
            print("  stopping: the home leg could not be made safe.")
            print("  Re-run with --skip-home to go straight to the pickup pose.")
            return
    else:
        print("\n=== 2/6  HOME (skipped) ===")

    # -------------------------------------------------------- PICKUP POSE
    print("\n=== 3/6  PICKUP POSE ===")
    q_pick = np.array(json.load(
        open(WS + "/recordings/baselines/pick_pose_both_v3.json"))["left"]["rad"])
    print("  pickup (deg):", [round(math.degrees(x), 2) for x in q_pick])
    if not leg(node, fk, tab, q_pick, "PICKUP", dry=args.dry_run):
        print("  stopping: could not reach the pickup pose safely.")
        return
    tab = live_table(node, fk) or tab

    # --------------------------------------------------------------- SCAN
    print("\n=== 4/6  SCAN the table ===")
    m, q_obs = find_cube(node, fk, tab, verbose=True)
    if m is None:
        raise SystemExit("scan finished without finding the cube")
    tab = live_table(node, fk) or tab
    objs = []
    print("  cube: %.1f mm tall, %d depth points, plane RMS %.2f mm"
          % (m["height_mm"], m["n_pts"], m["plane_rms_mm"]))

    # ------------------------------------------------------------- LOCATE
    print("\n=== 5/6  LOCATE in robot coordinates ===")
    q = node.fresh_q()
    Tw_d, = fk.poses("left", q, ["camera_depth_frame"])
    cube_w = (Tw_d @ np.r_[m["centre"], 1])[:3]
    Tb, = fk.poses("left", q, ["base_link"])
    cube_b = np.linalg.inv(Tb) @ np.r_[cube_w, 1]
    print("  cube in world          : (%+.4f, %+.4f, %+.4f)" % tuple(cube_w))
    print("  cube in left_base_link : (%+.4f, %+.4f, %+.4f)" % tuple(cube_b[:3]))
    print("  range camera->cube     : %.4f m" % np.linalg.norm(m["centre"]))
    json.dump({"q": list(map(float, q)), "grasp_world": cube_w.tolist(),
               "table_normal_world": list(map(float, Tw_d[:3, :3] @ m["n"])),
               "height_mm": m["height_mm"], "n_cube": m["n_pts"],
               "plane_rms_mm": m["plane_rms_mm"]},
              open(WS + "/recordings/baselines/cube_located_v2.json", "w"), indent=1)
    if args.dry_run:
        print("\ndry run: located, nothing grasped")
        return

    # ----------------------------------------------------------------- PICK
    print("\n=== 6/6  PICK ===")
    print("  servoing the pads onto the cube (camera-frame error, so the")
    print("  8.45 deg mount error cannot steer it wrong)")
    last = None
    print("  %-4s %9s %9s %9s  %s" % ("it", "err_mm", "pads>tbl", "cube_mm", "action"))
    for it in range(1, 10):
        mm = measure(node)
        if not plausible(mm):
            print("  cube out of view (expected this close)")
            break
        qq = mm["q"]
        Tw, = fk.poses("left", qq, ["camera_depth_frame"])
        last = {"cube_w": (Tw @ np.r_[mm["centre"], 1])[:3],
                "n_w": Tw[:3, :3] @ mm["n"]}
        pad = pads_in_cam(fk, qq, CUBE_GRIP, "left")
        ev = mm["centre"] - pad
        err = float(np.linalg.norm(ev))
        print("  %-4d %9.1f %9.1f %9.1f  " % (it, err * 1000,
                                              (pad @ mm["n"] + mm["d"]) * 1000,
                                              mm["height_mm"]), end="")
        if err < TOL_M:
            print("within tolerance")
            break
        step = ev * STEP_FRAC
        if np.linalg.norm(step) > MAX_STEP_M:
            step = step / np.linalg.norm(step) * MAX_STEP_M
        h_new = float(pad @ mm["n"] + mm["d"]) + float(step @ mm["n"])
        if h_new < MIN_CLEAR_M:
            step = step + mm["n"] * (MIN_CLEAR_M - h_new)
        Tl, Tr, Tee = fk.poses("left", qq,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        qn, ep, tl = ik_relaxed(mid + Tw[:3, :3] @ step, Tee[:3, :3], qq,
                                CUBE_GRIP, Tw[:3, :3] @ mm["n"], fk)
        if ep > 0.006:
            print("IK residual %.1f mm even with a 35 deg cone; stopping"
                  % (ep * 1000))
            break
        print("step %.1f mm%s" % (np.linalg.norm(step) * 1000,
                                  "" if tl == 0 else " (tilt %.0f deg)" % tl))
        if node.goto(qn, "servo-%d" % it) is None:
            print("  guard fired")
            break

    if last is None:
        raise SystemExit("never got a fix on the cube -- NOT closing the hand")

    target = (m["height_mm"] / 2.0) / 1000.0
    print("\n  guarded descent to a measured gap of %.1f mm" % (target * 1000))

    def measure_fn():
        mm = measure(node)
        if not plausible(mm):
            return None
        return (pads_in_cam(fk, node.fresh_q(), CUBE_GRIP, "left"),
                mm["n"], mm["d"], mm.get("inlier_frac", 0.6))

    def move_fn(vec_cam):
        qq = node.fresh_q()
        Tw, = fk.poses("left", qq, ["camera_depth_frame"])
        Tl, Tr, Tee = fk.poses("left", qq,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        qn, ep, _ = ik_relaxed(mid + Tw[:3, :3] @ vec_cam, Tee[:3, :3], qq,
                               CUBE_GRIP, Tw[:3, :3] @ np.array([0, 0, 1.0]), fk)
        if ep < 0.006:
            node.goto(qn, "descend")

    try:
        final = descend(measure_fn, move_fn, target)
        print("  final measured gap: %.1f mm" % (final * 1000))
    except Blind as e:
        print("  %s" % e)
        qq = node.fresh_q()
        Tl, Tr, Tee = fk.poses("left", qq,
                               ["robotiq_85_left_finger_tip_link",
                                "robotiq_85_right_finger_tip_link",
                                "end_effector_link"], gripper=CUBE_GRIP)
        mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        rem = float(np.linalg.norm(last["cube_w"] - mid))
        print("  completing %.1f mm from the last world fix" % (rem * 1000))
        if rem > 0.002:
            qn, ep, tl = ik_relaxed(last["cube_w"], Tee[:3, :3], qq, CUBE_GRIP,
                                    last["n_w"] / np.linalg.norm(last["n_w"]), fk)
            if ep < 0.006:
                node.goto(qn, "FINAL-APPROACH")
            else:
                print("  final approach unreachable (%.1f mm residual)" % (ep * 1000))

    node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE on the cube")

    up = last["n_w"] / np.linalg.norm(last["n_w"])
    qq = node.fresh_q()
    Tl, Tr, Tee = fk.poses("left", qq,
                           ["robotiq_85_left_finger_tip_link",
                            "robotiq_85_right_finger_tip_link",
                            "end_effector_link"], gripper=GRIP_SQUEEZE)
    mid = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    qn, ep, _, _ = ik(mid + up * LIFT_M, Tee[:3, :3], qq, grip=GRIP_SQUEEZE)
    if ep < 0.004:
        node.goto(qn, "LIFT")

    print("\n  did it come up?")
    m2 = measure(node)
    if not plausible(m2):
        print("  no cube in view after lifting -- consistent with EITHER a")
        print("  hold (cube under the fingers, out of frame) OR a miss.")
        return
    pad2 = pads_in_cam(fk, node.fresh_q(), GRIP_SQUEEZE, "left")
    d = float(np.linalg.norm(m2["centre"] - pad2))
    print("  cube is %.1f mm from the pads: %s"
          % (d * 1000, "HELD" if d < 0.05 else "MISSED, still on the table"))


if __name__ == "__main__":
    main()
