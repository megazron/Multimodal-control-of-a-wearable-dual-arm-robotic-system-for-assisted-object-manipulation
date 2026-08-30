#!/usr/bin/env python3
"""Pick a cube AT THE RIM OF REACH, approaching along the tool axis.

    python3 scripts/srl_pick_reach.py --arm left              # plan only
    python3 scripts/srl_pick_reach.py --arm left --execute    # move the arm

WHY THIS EXISTS: THE CUBE IS 0.95 m FROM THE BASE AND TOP-DOWN IS GEOMETRY,
NOT TUNING
---------------------------------------------------------------------------
Measured 2026-08-26: the left base sits at (0.35, -0.32, 1.25) and the cube
at (0.40, +0.51, 0.79) -- 0.83 m forward and 0.46 m below the shoulder,
0.95 m away.  A vertical wrist above the cube needs the WRIST at that same
horizontal radius, and position-only IK says the pads reach the cube with
0.1 mm to spare ONLY when the tool is tilted 48-66 deg from vertical.  So
`srl_pick_topdown`'s "no straight-down pose reaches 160 mm above the cube"
was not a solver failure; the pose it asked for does not exist on this arm.

THE APPROACH IS A LINE ALONG THE TOOL AXIS, NOT A DESCENT
---------------------------------------------------------
The tool axis is tilted TILT deg from vertical toward the cube (radially away
from the base -- the only azimuth that solves at the rim), and the hand
advances along that axis from a retracted pre-pose.  The wrist camera looks
along the same axis, so the cube stays in frame for most of the approach.

WHAT KEEPS IT HONEST, ALL MEASURED PER STEP
-------------------------------------------
* AIM  is closed-loop: the cube is re-measured in the wrist camera's own
  frame every step and the target re-solved, which is what absorbs the
  ~8.5 deg mount extrinsic (`docs/PICK_THE_CUBE.md`, lesson 1).
* STOP is extrinsic-free: hand mesh points and the measured table plane are
  expressed in the SAME camera frame through one rigid FK chain, so the
  tip-to-table gap never touches the mount error (`srl_scene`'s proof).
* STEP is guarded: no step is larger than half the measured tip-to-table
  gap, so a wrong prediction cannot reach the table before the next
  measurement corrects it (`guarded_descent`'s rule, applied on a slope).
* The WEARER is checked on the arm's mesh surface for every commanded pose
  (`srl_body_geometry.wearer_clearance` -- NOT `ClearanceModel`, which
  samples link origins and returned 367 mm for a pose touching the person).
* BLIND ENDGAME: the camera loses the cube in the last ~80 mm (mount
  geometry).  The last fix is carried in world, the pinch point is raised
  8 mm above the cube centre so the residual extrinsic error (<=9 mm over
  the blind travel) spends itself on cube, not table, and the blind advance
  is refused outright if it would exceed BLIND_MAX_M.

Poses are solved at CUBE_GRIP -- the opening the hand will actually hold the
cube at -- because the pad midpoint moves 11.43 mm when the hand closes
(lesson 2).  Joint arithmetic goes through ang_wrap (lesson 4); arrival is
waited for, not timed (lesson 6).
"""
import argparse
import json
import math
import sys
import time

import numpy as np

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import plan_pick_left as planner  # noqa: E402
from execute_pick_left import ang_wrap  # noqa: E402
from servo_pick_left import (CUBE_GRIP, Eye, GRIP_OPEN,  # noqa: E402
                             GRIP_SQUEEZE, NoFrames, measure)
from srl_body_geometry import (HAND_LINKS, body_points, tool_extent,  # noqa: E402
                               wearer_clearance)
from srl_fk import FK  # noqa: E402

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_teleop")
from srl_teleop.clearance import ClearanceModel  # noqa: E402

BASE_W = {"left": np.array([0.35, -0.32, 1.25]),
          "right": np.array([-0.35, -0.32, 1.25])}
FLOOR_M = 0.150            # HARD CONSTRAINT 11
TABLE_MIN_M = 0.004        # smallest allowed hand-mesh-to-table gap
#: Pads grip this far ABOVE the cube centre.  8 mm cleared the finger TIPS
#: but the plan gate then measured the hand MESH (knuckle and camera
#: bracket) 4-20 mm BELOW the table at every solvable tilt; 14 mm is still
#: 8 mm of pad on a 44 mm cube and buys the mesh its clearance.
PINCH_UP_M = 0.014
#: Pre-pose retreat along -tool axis.  0.13 breaches the wearer floor: at
#: -120 mm the wrist link is 0.142 m from the mannequin's lower arm.  At
#: -100 mm the whole ladder clears (wearer 0.155, hand-table 46 mm).
PRE_BACK_M = 0.10
STEP_MAX_M = 0.020
ALONG_TOL_M = 0.006        # "arrived" when this close along the axis
BLIND_MAX_M = 0.095        # refuse to finish more than this without vision
MAX_ITERS = 18
CUBE_MM_RANGE = (20.0, 75.0)
#: Tilts from vertical, tried in order.  55 deg is the shallowest at which
#: 51 mm finger tips can clear a table while the pads hold a 44 mm cube
#: above its centre; 72 is nearly horizontal and always clears.
TILTS_DEG = (55.0, 60.0, 66.0, 72.0)
YAWS_DEG = tuple(range(0, 360, 45))
DEFAULT_CUBE = (0.4036, 0.5124, 0.7873, 44.0)   # the 19:34 wrist-camera fix


def R_approach(tilt_deg, azim_rad, yaw_deg):
    """Tool +z tilted `tilt` from straight-down, toward azimuth; hand yawed."""
    t = math.radians(tilt_deg)
    z = np.array([math.sin(t) * math.cos(azim_rad),
                  math.sin(t) * math.sin(azim_rad), -math.cos(t)])
    y = math.radians(yaw_deg)
    x0 = np.array([math.cos(y), math.sin(y), 0.0])
    x = x0 - z * (x0 @ z)
    n = np.linalg.norm(x)
    if n < 1e-6:
        x0 = np.array([0.0, 1.0, 0.0])
        x = x0 - z * (x0 @ z)
        n = np.linalg.norm(x)
    x /= n
    return np.column_stack([x, np.cross(z, x), z])


class Gate:
    """Every commanded pose passes the same three checks, and says which
    one it failed."""

    def __init__(self, fk, arm, model, table_z_w):
        self.fk, self.arm, self.model = fk, arm, model
        self.table_z = table_z_w

    def check(self, q, grip, table_margin=TABLE_MIN_M):
        d, link, part = wearer_clearance(self.fk, self.arm, q, grip,
                                         self.model)
        if d < FLOOR_M:
            return False, "wearer %.3f m < %.3f (%s/%s)" % (
                d, FLOOR_M, (link or "?"), part)
        pts = np.vstack(list(body_points(self.fk, self.arm, q, grip,
                                         links=HAND_LINKS).values()))
        gap = float(pts[:, 2].min() - self.table_z)
        if gap < table_margin:
            return False, "hand mesh %.1f mm above table < %.1f" % (
                gap * 1000, table_margin * 1000)
        return True, "wearer %.3f table %.0f mm" % (d, gap * 1000)


def solve_at(p_des, R, q_seed, grip=CUBE_GRIP, iters=110):
    q, ep, er, _ = planner.ik(np.asarray(p_des, float), R, q_seed,
                              grip=grip, iters=iters)
    ok = ep < 3e-3 and er < math.radians(3.0)
    return ok, q, ep


def plan(fk, arm, gate, cube_w, cube_h_mm, q_now, tips, verbose=True):
    """The whole approach ladder, solved and gated before anything moves.

    Returns (tilt, yaw, rungs) where rungs is the ordered list of
    (label, p_pad, q) from pre-pose to grasp, or None with the reason
    printed.
    """
    azim = math.atan2(cube_w[1] - BASE_W[arm][1], cube_w[0] - BASE_W[arm][0])
    grasp_p = cube_w + np.array([0.0, 0.0, PINCH_UP_M])
    table_z = cube_w[2] - cube_h_mm / 2000.0
    for tilt in TILTS_DEG:
        # analytic tip check first: pads at grasp_p put the tips
        # tips*cos(tilt) lower, and they must clear the table.
        tip_drop = tips * math.cos(math.radians(tilt))
        tip_gap = (grasp_p[2] - table_z) - tip_drop
        if tip_gap < TABLE_MIN_M:
            if verbose:
                print("  tilt %2.0f: tips would sit %.1f mm above the table "
                      "-- too shallow" % (tilt, tip_gap * 1000))
            continue
        for yaw in YAWS_DEG:
            R = R_approach(tilt, azim, yaw)
            ok, q_g, ep = solve_at(grasp_p, R, q_now)
            if not ok:
                continue
            good, why = gate.check(q_g, CUBE_GRIP)
            if not good:
                if verbose:
                    print("  tilt %2.0f yaw %3d: grasp pose %s"
                          % (tilt, yaw, why))
                continue
            # the ladder back to the pre-pose, every rung solved and gated
            rungs = [("grasp", grasp_p, q_g)]
            q_seed = q_g
            back = STEP_MAX_M
            fail = None
            while back <= PRE_BACK_M + 1e-9:
                p = grasp_p - R[:, 2] * back
                ok, q_i, ep = solve_at(p, R, q_seed)
                good = ok and gate.check(q_i, CUBE_GRIP)[0]
                if not good:
                    fail = "rung at -%.0f mm %s" % (
                        back * 1000, "unsolved" if not ok else "gated")
                    break
                rungs.append(("-%.0fmm" % (back * 1000), p, q_i))
                q_seed = q_i
                back += STEP_MAX_M
            if fail:
                if verbose:
                    print("  tilt %2.0f yaw %3d: %s" % (tilt, yaw, fail))
                continue
            rungs.reverse()
            if verbose:
                print("  tilt %2.0f yaw %3d: FULL LADDER SOLVES, %d rungs, "
                      "tip-table %.1f mm at grasp"
                      % (tilt, yaw, len(rungs), tip_gap * 1000))
            return tilt, yaw, azim, rungs
    return None


def reverify(fk, arm, gate, tilt, yaw, azim, rungs, n=10):
    """The standing rule: N repeats over the whole path, jittered seeds.
    A pose passing one IK call is one coin flip."""
    R = R_approach(tilt, azim, yaw)
    rng = np.random.default_rng(0)
    worst = 0
    for label, p, q_ref in rungs:
        okc = 0
        for k in range(n):
            seed = q_ref + rng.uniform(-0.35, 0.35, 7)
            ok, q, ep = solve_at(p, R, seed)
            if ok and gate.check(q, CUBE_GRIP)[0]:
                okc += 1
        print("    %-8s %2d/%2d" % (label, okc, n))
        worst = max(worst, n - okc)
    return worst == 0


def cam_geometry(fk, arm, q, m, grip):
    """Everything the guard needs, in the camera's own frame: cube centre,
    pad midpoint, tip-to-table gap over the whole hand mesh.  One rigid
    chain -- no mount extrinsic anywhere in this function."""
    Tw_d, Tl, Tr = fk.poses(arm, q, ["camera_depth_frame",
                                     "robotiq_85_left_finger_tip_link",
                                     "robotiq_85_right_finger_tip_link"],
                            gripper=grip)
    inv = np.linalg.inv(Tw_d)
    pts_w = np.vstack(list(body_points(fk, arm, q, grip,
                                       links=HAND_LINKS).values()))
    pts_c = (inv[:3, :3] @ pts_w.T).T + inv[:3, 3]
    hand_gap = float((pts_c @ m["n"] + m["d"]).min())
    mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
    mid_c = inv[:3, :3] @ mid_w + inv[:3, 3]
    err_c = m["centre"] - mid_c                  # pad -> cube, camera frame
    err_w = Tw_d[:3, :3] @ err_c                 # rotated into world to steer
    cube_w = (Tw_d @ np.r_[m["centre"], 1.0])[:3]
    return dict(hand_gap=hand_gap, err_w=err_w, cube_w=cube_w, mid_w=mid_w)


def look(node, tries=6):
    for i in range(tries):
        try:
            m = measure(node)
        except NoFrames as e:
            print("    [%d] %s" % (i, e))
            node.spin(0.4)
            continue
        if m is None:
            print("    [%d] no usable cube+plane in this frame" % i)
        elif not (CUBE_MM_RANGE[0] <= m["height_mm"] <= CUBE_MM_RANGE[1]):
            print("    [%d] rejected: %.1f mm tall is not a cube"
                  % (i, m["height_mm"]))
        else:
            return m
        node.spin(0.4)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--execute", action="store_true",
                    help="actually move; without it, plan and stop")
    ap.add_argument("--cube", nargs=4, type=float, metavar=("X", "Y", "Z", "H_MM"),
                    default=list(DEFAULT_CUBE),
                    help="world cube fix to plan against (live measure "
                         "replaces it before any motion)")
    a = ap.parse_args()
    arm = a.arm

    import rclpy
    from std_msgs.msg import Float64MultiArray
    rclpy.init()
    node = Eye(arm)
    node.fk = FK()
    node.grip_fb = None
    node.create_subscription(
        Float64MultiArray, "/real/gripper_%s" % arm,
        lambda msg: setattr(node, "grip_fb",
                            (float(msg.data[0]), float(msg.data[1]))
                            if len(msg.data) >= 2 else None), 10)
    fk = node.fk
    planner.set_arm(arm)
    model = ClearanceModel()

    if not node.preflight():
        return 2
    node.set_deadband(0.10)
    q_now = node.fresh_q()
    node._hold = q_now.copy()

    tips = tool_extent(fk, arm, q_now, CUBE_GRIP)["tips_past_pads"]
    cube_w = np.array(a.cube[:3])
    cube_h = a.cube[3]

    if a.execute:
        print("\n=== live look before planning ===")
        m = look(node)
        if m is not None:
            g = cam_geometry(fk, arm, node.fresh_q(), m, CUBE_GRIP)
            cube_w = g["cube_w"]
            cube_h = m["height_mm"]
            print("  cube measured NOW at (%+.4f, %+.4f, %+.4f), %.0f mm, "
                  "plane rms %.1f mm"
                  % (*cube_w, cube_h, m["plane_rms_mm"]))
        else:
            print("  wrist camera cannot see the cube from here; planning "
                  "on the given fix (%.3f, %.3f, %.3f)" % tuple(cube_w))

    table_z = cube_w[2] - cube_h / 2000.0
    gate = Gate(fk, arm, model, table_z)
    print("\n=== plan: approach ladder to the cube at (%.3f, %.3f, %.3f) ==="
          % tuple(cube_w))
    print("  finger tips %.1f mm past the pads; table plane z %.3f"
          % (tips * 1000, table_z))
    got = plan(fk, arm, gate, cube_w, cube_h, q_now, tips)
    if got is None:
        print("NO approach solves.  The cube is outside what this arm can "
              "grasp -- move it closer to the robot (smaller y) and retry.")
        return 1
    tilt, yaw, azim, rungs = got

    print("\n=== re-verify, jittered seeds (the standing rule) ===")
    if not reverify(fk, arm, gate, tilt, yaw, azim, rungs):
        print("  MARGINAL -- a rung does not solve from every seed.  "
              "Not usable; move the cube closer.")
        return 1
    print("  every rung solves from every seed")

    print("\n=== transit check: joint ramp from HERE to the pre-pose ===")
    q_pre = rungs[0][2]
    transit_ok = True
    for s in np.linspace(0.0, 1.0, 21):
        qi = q_now + ang_wrap(q_pre - q_now) * s
        good, why = gate.check(qi, CUBE_GRIP, table_margin=0.03)
        if not good:
            print("  transit sample %.2f FAILS: %s" % (s, why))
            transit_ok = False
            break
    if transit_ok:
        print("  all 21 samples clear (wearer floor + 30 mm above table)")

    if not a.execute:
        print("\nplan only: nothing commanded.  Re-run with --execute.")
        return 0 if transit_ok else 1
    if not transit_ok:
        print("refusing to move: the transit path is not clear.")
        return 1

    # ---------------- moving from here on ----------------
    R = R_approach(tilt, azim, yaw)
    zt = R[:, 2]
    node.hold_gripper(GRIP_OPEN, 2.0, "OPEN")

    print("\n=== 1/3 transit to the pre-pose ===")
    if node.goto(q_pre, "PRE [tilt %.0f yaw %d]" % (tilt, yaw)) is None:
        return 1

    print("\n=== 2/3 advance along the approach axis, re-measuring ===")
    print("  %-4s %9s %9s %9s  %s"
          % ("it", "along_mm", "hand_mm", "lat_mm", "action"))
    last_fix = np.array(cube_w)
    grasp_p = None
    for it in range(1, MAX_ITERS + 1):
        q = node.fresh_q()
        m = look(node, tries=3)
        if m is not None:
            g = cam_geometry(fk, arm, q, m, CUBE_GRIP)
            last_fix = g["cube_w"]
            hand_gap = g["hand_gap"]
            mid_w = g["mid_w"]
        else:
            _, Tl, Tr = fk.poses(arm, q, ["end_effector_link",
                                          "robotiq_85_left_finger_tip_link",
                                          "robotiq_85_right_finger_tip_link"],
                                 gripper=CUBE_GRIP)
            mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
            hand_gap = None
        grasp_p = last_fix + np.array([0.0, 0.0, PINCH_UP_M])
        to_go = grasp_p - mid_w
        along = float(to_go @ zt)
        lat = to_go - zt * along
        if along <= ALONG_TOL_M and np.linalg.norm(lat) <= 0.008:
            print("  %-4d %9.1f %9s %9.1f  AT GRASP"
                  % (it, along * 1000,
                     "-" if hand_gap is None else "%.1f" % (hand_gap * 1000),
                     np.linalg.norm(lat) * 1000))
            break
        if m is None and along > BLIND_MAX_M:
            print("  %-4d %9.1f %9s %9.1f  cube not visible and %.0f mm to "
                  "go > %.0f blind limit -- stopping"
                  % (it, along * 1000, "-", np.linalg.norm(lat) * 1000,
                     along * 1000, BLIND_MAX_M * 1000))
            return 1
        step = min(STEP_MAX_M, max(0.0, along - ALONG_TOL_M / 2))
        if hand_gap is not None:
            # the guarded rule, on a slope: never step more than half the
            # measured hand-to-table gap less the floor
            step = min(step, max(0.0, (hand_gap - TABLE_MIN_M) * 0.5))
            if step < 0.001 and along > ALONG_TOL_M:
                print("  %-4d %9.1f %9.1f %9.1f  gap will not admit another "
                      "step -- stopping here"
                      % (it, along * 1000, hand_gap * 1000,
                         np.linalg.norm(lat) * 1000))
                break
        p_next = mid_w + zt * step + lat * min(1.0, step / max(along, 1e-6))
        ok, q_n, ep = solve_at(p_next, R, q)
        if not ok:
            print("  %-4d %9.1f %9s %9.1f  IK residual %.1f mm -- stopping"
                  % (it, along * 1000,
                     "-" if hand_gap is None else "%.1f" % (hand_gap * 1000),
                     np.linalg.norm(lat) * 1000, ep * 1000))
            break
        good, why = gate.check(q_n, CUBE_GRIP)
        if not good:
            print("  %-4d %9.1f %9s %9.1f  %s -- stopping"
                  % (it, along * 1000,
                     "-" if hand_gap is None else "%.1f" % (hand_gap * 1000),
                     np.linalg.norm(lat) * 1000, why))
            break
        print("  %-4d %9.1f %9s %9.1f  advance %.1f mm"
              % (it, along * 1000,
                 "-" if hand_gap is None else "%.1f" % (hand_gap * 1000),
                 np.linalg.norm(lat) * 1000, step * 1000))
        if node.goto(q_n, "advance-%d" % it) is None:
            return 1

    print("\n=== 3/3 close, lift, verify ===")
    node.hold_gripper(GRIP_SQUEEZE, 4.0, "CLOSE")
    node.spin(1.0)
    held = None
    if node.grip_fb:
        cmd, meas = node.grip_fb
        held = meas < cmd - 0.06
        print("  gripper: commanded %.3f, measured %.3f -> %s"
              % (cmd, meas, "STOPPED SHORT (holding something)" if held
                 else "closed fully (EMPTY)"))
    q = node.fresh_q()
    # retreat along -tool, then straight up, both gated
    for lbl, dp in (("RETREAT", -zt * 0.10), ("LIFT", np.array([0, 0, 0.10]))):
        _, Tl, Tr = fk.poses(arm, q, ["end_effector_link",
                                      "robotiq_85_left_finger_tip_link",
                                      "robotiq_85_right_finger_tip_link"],
                             gripper=GRIP_SQUEEZE)
        mid_w = (Tl[:3, 3] + Tr[:3, 3]) / 2.0
        ok, q_n, ep = solve_at(mid_w + dp, R, q, grip=GRIP_SQUEEZE)
        if ok and gate.check(q_n, GRIP_SQUEEZE)[0]:
            if node.goto(q_n, lbl) is None:
                break
            q = node.fresh_q()
    node.spin(1.0)
    m = look(node, tries=3)
    if m is None:
        print("  wrist camera: no cube on the table below -- consistent "
              "with HOLDING it (or with looking elsewhere)")
    else:
        print("  wrist camera: a %.0f mm object still on the table -- "
              "the grasp probably MISSED" % m["height_mm"])
    print("\n*** PICKED ***" if held else
          "\nnot holding the cube -- re-run; the approach starts from "
          "wherever the arm is.")
    return 0 if held else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:                                     # noqa: BLE001
            pass
