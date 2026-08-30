#!/usr/bin/env python3
"""Find a cube with every camera on the rig and pick it up, tool pointing DOWN.

    bash scripts/bringup_arm.sh right          # bridge + wrist camera
    bash scripts/start_scene_cameras.sh        # the two room cameras
    python3 scripts/srl_pick_live.py --arm right

    python3 scripts/srl_pick_live.py --arm right --dry-run   # no motion

WHAT "POINTING FULL DOWN" MEANS HERE, EXACTLY
---------------------------------------------
The tool axis is the END EFFECTOR'S +Z.  That is not an assumption: FK says
the pad midpoint sits 0.11012 m from the EE origin and the dot product of that
offset with the EE z-axis is 0.11012 -- the whole of it -- while x and y get
0.00000.  So "full down" is the pose whose EE z-axis is world -z, and the one
remaining freedom is the hand's yaw about that vertical, which is searched.

THE SAME POSE LOOKS AND GRASPS.  The wrist camera points along the tool axis,
so an arm pointing straight down is an arm looking straight down at the table.
The observation pose and the grasp pose are therefore the same orientation at
two heights, which is why nothing has to be re-solved between seeing the cube
and going for it, and why the cube cannot leave the frame sideways on the way
in.

WHY THE GRASP IS CLOSED LOOP AND NOT PLANNED
--------------------------------------------
The wrist camera's mount rotation is wrong by ~8.45 deg and it turns WITH the
wrist, so the same table measured from four arm poses gave normals spanning
8.45 deg -- 34 mm at 0.23 m.  A grasp planned through that transform and
executed blind cannot land inside a 30 mm capture gate, and did not.  The
error between the PADS and the CUBE is instead measured in the camera's own
frame, where the data is good (0.54 mm plane RMS), and nulled by iteration.
Closed loop converges through a wrong extrinsic; open loop cannot.

WHY THE PADS ARE PLACED AT THE OPENING THEY WILL HOLD AT
-------------------------------------------------------
The Robotiq 85's fingers swing on a four-bar, so wrist-to-pad is 0.09833 m
wide open and 0.10976 m closed on a 40 mm cube.  Servoing the OPEN pad
midpoint onto the cube and then closing drives the pads 11.43 mm further down
-- into the table.  Every pose here is solved at CUBE_GRIP.

WHAT THE ROOM CAMERAS ARE FOR
-----------------------------
They are not calibrated to the robot and this program never treats them as if
they were.  They do two jobs the wrist camera cannot:

  * They see the cube whether or not the arm is looking at it.  To the wrist
    camera "there is no cube" and "I am not pointed at the table" are the same
    observation, and that ambiguity is what once produced a confident "no cube
    here" at fourteen viewpoints in a row.
  * They are an INDEPENDENT WITNESS to the lift.  The gripper's own feedback
    says the fingers stopped early, which is good evidence but is evidence
    from the same subsystem that did the grasping.  A camera bolted to a
    tripod across the room, watching the cube rise, is not.  Both must agree
    before this program claims a pick.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import plan_pick_left as planner  # noqa: E402
from execute_pick_left import ang_wrap  # noqa: E402
from servo_pick_left import (CUBE_GRIP, Eye, GRIP_OPEN,  # noqa: E402
                             GRIP_SQUEEZE, MIN_CLEAR_M, NoFrames, measure,
                             pads_in_cam)
from srl_fk import FK  # noqa: E402

GRIP_CLOSED_RAD = 0.8       # the bridge's own normalisation constant
OBS_STANDOFF_M = 0.32       # pad height above the table when looking
PRE_STANDOFF_M = 0.12       # pad height above the cube centre before descent
LIFT_M = 0.12
STEP_FRAC = 0.6
MAX_STEP_M = 0.045
TOL_M = 0.006
MAX_SERVO = 10
YAWS = [math.radians(a) for a in range(0, 360, 15)]

# Where to look first, in world metres. From the last both-arm table scan:
# objects sat around (-0.39, +0.19, 0.71) and the surface fit at z ~ 0.71.
# It is a STARTING GUESS for the search, not a measurement anything trusts --
# the scan's own caveat is that its world coordinates carry the mount error.
TABLE_GUESS_XY = (-0.39, 0.19)
TABLE_GUESS_Z = 0.71
SEARCH_DXY = [(0.0, 0.0), (0.06, 0.0), (-0.06, 0.0), (0.0, 0.06), (0.0, -0.06),
              (0.10, 0.06), (-0.10, 0.06), (0.10, -0.06), (-0.10, -0.06),
              (0.0, 0.12), (0.0, -0.12), (0.16, 0.0), (-0.16, 0.0)]


def down_R(yaw):
    """Tool axis (EE +z) along world -z, hand yawed by `yaw` about vertical."""
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def tool_down_deg(R):
    """How far the tool axis is from straight down, in degrees."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[:, 2] @ [0, 0, -1.0])))))


def solve_down(fk, arm, p_des, q_seed, grip=CUBE_GRIP, yaws=YAWS):
    """Best straight-down IK at `p_des`, over the free yaw.

    Scored by joint travel from the seed, so the arm takes the nearest way of
    pointing down rather than the first one the search happens to find.
    Travel is measured through ang_wrap because joints 3, 5 and 7 are
    CONTINUOUS -- plain subtraction once called -179.05 and +181.02 deg a
    360.08 deg move and aborted a run that had arrived.
    """
    prev = planner.set_arm(arm)
    try:
        best = None
        for yaw in yaws:
            q, ep, er, _ = planner.ik(np.asarray(p_des, float), down_R(yaw),
                                      q_seed, grip=grip)
            if ep > 2e-3 or er > math.radians(2.0):
                continue
            travel = float(np.abs(ang_wrap(q - q_seed)).max())
            if best is None or travel < best[0]:
                best = (travel, q, ep, er, yaw)
        return best
    finally:
        planner.set_arm(prev)


class Picker(Eye):
    """The wrist camera and the arm, plus the room cameras as a witness."""

    def __init__(self, arm):
        super().__init__(arm)
        self.fk = FK()
        self.det = None
        self.det_t = 0.0
        self.grip_fb = None
        self.create_subscription(String, "/srl/detections", self._on_det, 10)
        self.create_subscription(Float64MultiArray, "/real/gripper_%s" % arm,
                                 self._on_grip, 10)
        # keep the scene status subscriptions alive so a dead room camera is
        # visible as silence rather than as a stale detection
        self.scene_status = {}
        for k in ("usb", "rs"):
            self.create_subscription(
                String, "/scene/%s/status" % k,
                lambda m, k=k: self.scene_status.__setitem__(k, m.data),
                qos_profile_sensor_data)

    def _on_det(self, m):
        try:
            self.det = json.loads(m.data)
            self.det_t = time.time()
        except Exception:                            # noqa: BLE001
            pass

    def _on_grip(self, m):
        if len(m.data) >= 2:
            self.grip_fb = (float(m.data[0]), float(m.data[1]))

    # ------------------------------------------------------------- witnesses
    def scene_cube(self, colour="green", max_age=3.0):
        """The room cameras' current view of the cube, or None.

        Returns pixel rows per camera. Deliberately NOT converted to metres:
        there is no measured extrinsic from either room camera to the robot,
        and inventing one is exactly the class of mistake this repo keeps
        finding.
        """
        for _ in range(40):
            if self.det is not None and time.time() - self.det_t < max_age:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.det is None or time.time() - self.det_t > max_age:
            return None
        out = {}
        for cam, c in self.det.get("cameras", {}).items():
            if not cam.startswith("scene/"):
                continue
            hits = [d for d in c["detections"] if d["colour"] == colour]
            if hits:
                out[cam] = max(hits, key=lambda d: d["area_px"])
        return out or None

    def grip_state(self, wait=2.0):
        """(commanded, measured) gripper position, normalised 0 open 1 shut."""
        t0 = time.time()
        while time.time() - t0 < wait:
            if self.grip_fb is not None:
                return self.grip_fb
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.grip_fb


def cube_in_world(fk, arm, m):
    """A wrist-camera measurement lifted into world coordinates."""
    Tw_d, = fk.poses(arm, m["q"], ["camera_depth_frame"])
    return ((Tw_d @ np.r_[m["centre"], 1.0])[:3],
            Tw_d[:3, :3] @ m["n"])


def pad_mid(fk, arm, q, grip):
    Tl, Tr = fk.poses(arm, q, ["robotiq_85_left_finger_tip_link",
                               "robotiq_85_right_finger_tip_link"], gripper=grip)
    return (Tl[:3, 3] + Tr[:3, 3]) / 2.0


# --------------------------------------------------------------------- stages
def observe(node, args):
    """Point the arm straight down over the table until the cube is in frame.

    Each candidate is a straight-down pose at a different patch of table. The
    search is over PLACES TO LOOK, not over orientations: the orientation is
    the one the user asked for and is held fixed the whole way through.
    """
    fk, arm = node.fk, node.arm_name
    seen = node.scene_cube()
    if seen:
        print("  room cameras see a green cube: %s"
              % ", ".join("%s at (%.0f,%.0f) %dpx"
                          % (c, d["u"], d["v"], d["area_px"])
                          for c, d in seen.items()))
    else:
        print("  room cameras do NOT see a green cube right now.")
        print("  (continuing anyway -- they are uncalibrated witnesses, not a "
              "gate; but if the wrist search also fails, believe them)")

    q0 = node.fresh_q().copy()
    z = TABLE_GUESS_Z + OBS_STANDOFF_M
    tried = 0
    for dx, dy in SEARCH_DXY:
        p = np.array([TABLE_GUESS_XY[0] + dx, TABLE_GUESS_XY[1] + dy, z])
        best = solve_down(fk, arm, p, q0)
        if best is None:
            print("  look at (%+.2f,%+.2f,%.2f): no straight-down IK" % tuple(p))
            continue
        travel, q, ep, er, yaw = best
        tried += 1
        print("  look at (%+.2f,%+.2f,%.2f): yaw %3.0f deg, travel %.1f deg, "
              "residual %.2f mm / %.2f deg"
              % (p[0], p[1], p[2], math.degrees(yaw), math.degrees(travel),
                 ep * 1000, math.degrees(er)))
        if args.dry_run:
            return None
        if node.goto(q, "LOOK-%d" % tried) is None:
            print("     guard fired on the way to the viewpoint")
            continue
        node.spin(1.0)
        try:
            m = measure(node)
        except NoFrames as e:
            print("     %s" % e)
            continue
        if m is None:
            print("     no cube in this view")
            continue
        Rk = node.fk.poses(arm, node.fresh_q(), ["end_effector_link"])[0][:3, :3]
        print("     CUBE: %d pts, %.1f mm tall, plane rms %.2f mm, tool %.2f "
              "deg off vertical"
              % (m["n_pts"], m["height_mm"], m["plane_rms_mm"], tool_down_deg(Rk)))
        return m
    return None


def servo_down(node, args, m0):
    """Null the pad-to-cube error in the camera frame, staying straight down."""
    fk, arm = node.fk, node.arm_name
    last = None
    print("\n%-4s %9s %9s %9s %8s %s"
          % ("it", "err_mm", "pad>tbl", "cube_mm", "tilt", "action"))
    for it in range(1, MAX_SERVO + 1):
        try:
            m = measure(node)
        except NoFrames as e:
            print("  %s" % e)
            break
        if m is None:
            print("  cube out of view (expected this close) -- finishing from "
                  "the last fix")
            break
        q = m["q"]
        cw, nw = cube_in_world(fk, arm, m)
        last = {"cube_w": cw, "n_w": nw}
        pad = pads_in_cam(fk, q, CUBE_GRIP, arm)
        err_v = m["centre"] - pad
        err = float(np.linalg.norm(err_v))
        pad_h = float(pad @ m["n"] + m["d"])
        Tee, = fk.poses(arm, q, ["end_effector_link"])
        print("%-4d %9.1f %9.1f %9.1f %7.2f  "
              % (it, err * 1000, pad_h * 1000, m["height_mm"],
                 tool_down_deg(Tee[:3, :3])), end="")
        if err < TOL_M:
            print("WITHIN TOLERANCE")
            break
        step_v = err_v * STEP_FRAC
        if np.linalg.norm(step_v) > MAX_STEP_M:
            step_v = step_v / np.linalg.norm(step_v) * MAX_STEP_M
        new_h = pad_h + float(step_v @ m["n"])
        if new_h < MIN_CLEAR_M:
            step_v = step_v + m["n"] * (MIN_CLEAR_M - new_h)
            print("(floor-limited) ", end="")
        Tw_d, = fk.poses(arm, q, ["camera_depth_frame"])
        step_w = Tw_d[:3, :3] @ step_v
        tgt = pad_mid(fk, arm, q, CUBE_GRIP) + step_w
        # RE-SOLVED STRAIGHT DOWN EVERY STEP, not nudged from the last pose.
        # Carrying the previous solution forward lets the tool axis drift a
        # fraction of a degree per iteration, and ten iterations of that is a
        # grasp that is no longer the pose that was asked for.
        best = solve_down(fk, arm, tgt, q)
        if best is None:
            print("no straight-down IK at the target -- stopping")
            break
        travel, qn, ep, er, yaw = best
        print("step %.1f mm (%.1f deg)"
              % (np.linalg.norm(step_v) * 1000, math.degrees(travel)))
        if args.dry_run:
            break
        if node.goto(qn, "servo-%d" % it) is None:
            print("  motion guard fired; stopping")
            break
    else:
        print("  reached the iteration limit")
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="right", choices=["left", "right"])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--attempts", type=int, default=3)
    a = ap.parse_args()

    rclpy.init()
    node = Picker(a.arm)
    if not node.preflight():
        return 2
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    fk, arm = node.fk, a.arm

    st = node.scene_status
    node.spin(2.5)
    print("  room cameras: %s"
          % (", ".join("%s %s" % (k, v) for k, v in st.items()) or "SILENT"))

    for attempt in range(1, a.attempts + 1):
        print("\n" + "=" * 72)
        print("ATTEMPT %d of %d" % (attempt, a.attempts))
        print("=" * 72)

        if not a.dry_run:
            print("\nopening the hand")
            node.hold_gripper(GRIP_OPEN, 2.5, "OPEN gripper")

        print("\n--- 1. LOOK STRAIGHT DOWN AND FIND THE CUBE ---")
        m = observe(node, a)
        if m is None:
            print("no cube found from any straight-down viewpoint")
            if a.dry_run:
                return 0
            continue

        cw, nw = cube_in_world(fk, arm, m)
        print("\n  cube in world: (%+.4f, %+.4f, %+.4f)" % tuple(cw))
        print("  table normal : (%+.3f, %+.3f, %+.3f), %.2f deg off vertical"
              % (*nw, math.degrees(math.acos(
                  max(-1.0, min(1.0, float(nw @ [0, 0, 1.0])))))))

        print("\n--- 2. PRE-GRASP, TOOL STRAIGHT DOWN, ABOVE THE CUBE ---")
        # Straight up in WORLD, not along the measured normal: the pose asked
        # for is vertical, and the measured normal carries the mount error.
        p_pre = cw + np.array([0.0, 0.0, PRE_STANDOFF_M])
        best = solve_down(fk, arm, p_pre, node.fresh_q())
        if best is None:
            print("  no straight-down IK above the cube at %.0f mm standoff"
                  % (PRE_STANDOFF_M * 1000))
            continue
        travel, q_pre, ep, er, yaw = best
        Rp = fk.poses(arm, q_pre, ["end_effector_link"])[0][:3, :3]
        print("  yaw %.0f deg, travel %.1f deg, residual %.2f mm / %.2f deg, "
              "tool %.3f deg off vertical"
              % (math.degrees(yaw), math.degrees(travel), ep * 1000,
                 math.degrees(er), tool_down_deg(Rp)))
        print("  q (deg): %s" % [round(math.degrees(x), 2) for x in q_pre])
        if a.dry_run:
            print("\ndry run: nothing commanded")
            return 0
        if node.goto(q_pre, "PRE-GRASP") is None:
            print("  guard fired going to pre-grasp")
            continue

        print("\n--- 3. SERVO THE PADS ONTO THE CUBE ---")
        last = servo_down(node, a, m)
        if last is None:
            print("  never got a fix -- NOT closing the hand")
            continue

        # ---- complete the approach from the last world fix ----
        q = node.fresh_q()
        mid = pad_mid(fk, arm, q, CUBE_GRIP)
        rem = last["cube_w"] - mid
        print("\n  remaining pad-to-cube from the last fix: %.1f mm"
              % (np.linalg.norm(rem) * 1000))
        if np.linalg.norm(rem) > 0.002:
            best = solve_down(fk, arm, last["cube_w"], q)
            if best is None:
                print("  no straight-down IK at the cube -- not moving")
            elif node.goto(best[1], "FINAL-APPROACH") is None:
                print("  guard fired on the final approach")

        print("\n--- 4. CLOSE ---")
        before = node.scene_cube()
        node.hold_gripper(GRIP_SQUEEZE, 5.0, "CLOSE gripper")
        node.spin(1.0)
        fb = node.grip_state()
        target_norm = GRIP_SQUEEZE / GRIP_CLOSED_RAD
        if fb is None:
            print("  no gripper feedback -- cannot say what the fingers did")
            held_by_grip = None
        else:
            cmd, meas = fb
            # THE FINGERS STOPPING SHORT IS THE EVIDENCE. An empty hand runs
            # to the commanded position; a hand with a 40 mm cube in it stalls
            # at the cube's width. 0.447 rad / 0.8 = 0.559 normalised is where
            # a 40 mm cube stops it, against 0.750 commanded.
            held_by_grip = meas < target_norm - 0.06
            print("  gripper: commanded %.3f, measured %.3f  -> %s"
                  % (cmd, meas,
                     "STOPPED SHORT (something is between the pads)"
                     if held_by_grip else
                     "closed to the commanded position (hand is EMPTY)"))

        print("\n--- 5. LIFT ---")
        q = node.fresh_q()
        mid = pad_mid(fk, arm, q, GRIP_SQUEEZE)
        lifted = False
        for h in (0.06, LIFT_M):
            best = solve_down(fk, arm, mid + np.array([0.0, 0.0, h]), q,
                              grip=GRIP_SQUEEZE)
            if best is None:
                print("  no straight-down IK %d mm up" % int(h * 1000))
                break
            if node.goto(best[1], "LIFT-%dmm" % int(h * 1000)) is None:
                break
            lifted = True
            node.spin(0.5)

        print("\n--- 6. DID IT ACTUALLY COME UP? ---")
        node.spin(2.0)
        after = node.scene_cube()
        witness = None
        if before and after:
            for cam in set(before) & set(after):
                dv = before[cam]["v"] - after[cam]["v"]
                print("  %s: cube was at v=%.0f, now v=%.0f  (%+.0f px, up is "
                      "positive)" % (cam, before[cam]["v"], after[cam]["v"], dv))
                if dv > 8:
                    witness = True
        elif before and not after:
            print("  room cameras no longer see the cube on the table "
                  "(consistent with it being in the hand and occluded)")
        fb2 = node.grip_state()
        if fb2:
            print("  gripper still reads measured %.3f" % fb2[1])

        got = bool(held_by_grip) and lifted
        if got:
            print("\n*** PICKED. The fingers stalled on the cube and the arm "
                  "lifted%s. ***" % (" and the room camera saw it rise"
                                     if witness else ""))
            return 0
        print("\n  not picked on this attempt.")
        if held_by_grip is False:
            print("  the hand closed on nothing -- the pads were not on the "
                  "cube. Re-measuring and trying again.")
    print("\nout of attempts.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        try:
            rclpy.shutdown()
        except Exception:                            # noqa: BLE001
            pass
