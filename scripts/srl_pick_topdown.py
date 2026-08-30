#!/usr/bin/env python3
"""Pick a cube FROM DIRECTLY ABOVE, positioned by continuous vision.

    python3 scripts/srl_pick_topdown.py --arm left --dry-run
    python3 scripts/srl_pick_topdown.py --arm left

WHY THIS EXISTS RATHER THAN ANOTHER PATCH TO `srl_pick_cube`
------------------------------------------------------------
That pipeline grasps at the PINNED WRIST ANCHOR -- the approach direction
`run_abc.send()` writes into every waypoint -- and on this table it cannot
reach the pose it asks for. Measured 2026-08-26, twenty-five descent steps in
a row:

    IK residual 17.3 mm at this step -- NOT commanded
    gap 37.2 mm, target 15.3 mm, gain at its 8.00 ceiling

The gap never closed because no step was ever commanded: the orientation was
unreachable and the move was silently skipped. Loosening the residual gate
would only have commanded a pose 17 mm from the one being solved for.

So the approach changes rather than the tolerance. THE TOOL POINTS STRAIGHT
DOWN -- the same attitude the scan pose uses, which is known to solve over
this table with 0.28 m of wearer clearance -- and the descent is along world
-z, which is the direction the object is actually approached from.

WHAT "CONTINUOUS" MEANS HERE, CONCRETELY
----------------------------------------
Every iteration re-measures. Nothing is planned once and executed open loop:

  the CUBE      re-measured in the wrist depth camera's own frame each step,
                where the data is good (plane RMS ~1.6 mm on this rig) and
                the mount extrinsic has not been applied yet.
  the TABLE     re-fitted from the same frame each step, so the stopping
                height is a MEASURED gap and not a commanded coordinate.
  the WEARER    re-checked from the arm's collision meshes against the body
                model, per body-part frame, before every commanded pose.
  the SCENE     the room camera is watched as an independent witness: it sees
                the cube whether or not the arm is looking, and it is what
                settles "did it come up" without asking the gripper about its
                own grasp.

THE FINGERS REACH PAST THE PADS AND THAT IS MEASURED, NOT ASSUMED
-----------------------------------------------------------------
`srl_body_geometry` reads the collision meshes: the finger tips sit 51.02 mm
beyond the pad midpoint along the tool axis, constant at every opening because
tips and pads swing together on the four-bar. Stopping with the pads at the
cube's centre therefore puts the TIPS 51 mm lower -- into the table. The stop
height accounts for it explicitly.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray, String

sys.path.insert(0, "/home/gausms/kortex_ws/scripts")
import plan_pick_left as planner  # noqa: E402
from execute_pick_left import ang_wrap  # noqa: E402
from servo_pick_left import (CUBE_GRIP, Eye, GRIP_OPEN,  # noqa: E402
                             GRIP_SQUEEZE, NoFrames, measure)
from srl_body_geometry import tool_extent, wearer_clearance  # noqa: E402
from srl_fk import FK  # noqa: E402

WS = "/home/gausms/kortex_ws"
sys.path.insert(0, "%s/src/srl_teleop" % WS)
from srl_teleop.clearance import ClearanceModel  # noqa: E402

FLOOR_M = 0.150          # HARD CONSTRAINT 11
HOVER_M = 0.16           # pad height above the cube top before descending
TOL_M = 0.006
MAX_ITERS = 14
#: A CUBE IS THIS TALL. The green gate also catches the mannequin's shirt and
#: any other green thing in frame; without a size gate the first run locked
#: onto a 268.5 mm "cube" and planned a grasp on it. Same range
#: `auto_observe.CUBE_MM_RANGE` uses.
CUBE_MM_RANGE = (20.0, 75.0)
#: 45 deg steps, not 15. The sweep is 24 IK solves in Python, each
#: with a 7-column finite-difference Jacobian per iteration -- it took
#: minutes per waypoint and the arm just sat there. Eight yaws cover
#: the same circle and the FIRST good one is taken, so a typical solve
#: is one or two.
YAWS = [math.radians(a) for a in range(0, 360, 45)]


def down_R(yaw):
    """Tool axis (EE +z) along world -z; hand yawed about the vertical."""
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def solve_down(fk, arm, p_des, q_seed, grip=CUBE_GRIP):
    """Nearest straight-down pose putting the PAD MIDPOINT at p_des."""
    prev = planner.set_arm(arm)
    try:
        # FIRST GOOD ONE WINS. Scoring all 8 to pick the shortest travel cost
        # more time than the travel it saved.
        for yaw in YAWS:
            q, ep, er, _ = planner.ik(np.asarray(p_des, float), down_R(yaw),
                                      q_seed, grip=grip, iters=110)
            if ep > 3e-3 or er > math.radians(3.0):
                continue
            travel = float(np.abs(ang_wrap(q - q_seed)).max())
            return (travel, q, ep, math.degrees(yaw))
        return None
    finally:
        planner.set_arm(prev)


class TopDownPicker(Eye):
    def __init__(self, arm):
        super().__init__(arm)
        self.fk = FK()
        self.model = ClearanceModel()
        self.det = None
        self.grip_fb = None
        self.create_subscription(String, "/srl/detections",
                                 self._on_det, 10)
        # THE SCENE UNDERSTANDING, which finds the TABLE first and then what
        # stands on it. Colour alone cannot tell a cube from the mannequin's
        # shirt -- a 268 mm "cube" was locked onto exactly that way -- and
        # this segments by GEOMETRY: the dominant plane, then clusters above
        # it, each measured and named from its own size.
        self.scene = None
        self.create_subscription(String, "/srl/scene",
                                 self._on_scene, 10)
        self.create_subscription(Float64MultiArray, "/real/gripper_%s" % arm,
                                 self._on_grip, 10)

    def _on_det(self, m):
        try:
            self.det = json.loads(m.data)
        except Exception:                                     # noqa: BLE001
            pass

    def _on_scene(self, m):
        try:
            self.scene = json.loads(m.data)
        except Exception:                                     # noqa: BLE001
            pass

    def scene_pick_target(self, max_age=3.0):
        """The nearest PICKABLE cube this arm's wrist camera can see.

        Returned in WORLD, converted through the wrist depth frame's own FK
        pose -- the one transform on this rig that comes from a single
        kinematic chain rather than a guessed mount.
        """
        sc = self.scene
        if not sc or time.time() - sc.get("t", 0) > max_age:
            return None
        cam = (sc.get("cameras") or {}).get("gripper/%s" % self.arm_name)
        if not cam or not cam.get("objects"):
            return None
        cubes = [o for o in cam["objects"] if o.get("pickable")]
        if not cubes:
            return None
        o = min(cubes, key=lambda x: x["centre_cam"][2])   # nearest first
        q = self.fresh_q()
        Tw_d, = self.fk.poses(self.arm_name, q, ["camera_depth_frame"])
        cw = (Tw_d @ np.r_[np.array(o["centre_cam"], float), 1.0])[:3]
        return dict(obj=o, world=cw, q=q)

    def _on_grip(self, m):
        if len(m.data) >= 2:
            self.grip_fb = (float(m.data[0]), float(m.data[1]))

    def scene_cube(self, colour="green"):
        """The room camera's view of the cube. An INDEPENDENT witness."""
        if not self.det:
            return None
        for cam, c in (self.det.get("cameras") or {}).items():
            if not cam.startswith("scene/"):
                continue
            hits = [d for d in c["detections"] if d["colour"] == colour]
            if hits:
                return max(hits, key=lambda d: d["area_px"])
        return None

    def safe(self, q, grip=CUBE_GRIP):
        """Wearer clearance for a pose, on the arm's SURFACE."""
        d, link, part = wearer_clearance(self.fk, self.arm_name, q, grip,
                                         self.model)
        return d, "%s vs %s" % ((link or "?").replace(self.arm_name + "_", ""),
                                part)

    def go(self, q, label):
        """Command a pose only if it clears the wearer."""
        d, where = self.safe(q)
        if d < FLOOR_M:
            print("  %s REFUSED: wearer clearance %.4f m < %.3f floor (%s)"
                  % (label, d, FLOOR_M, where))
            return False
        return self.goto(q, "%s [clear %.3f m]" % (label, d)) is not None


def cube_world(fk, arm, m):
    Tw_d, = fk.poses(arm, m["q"], ["camera_depth_frame"])
    return ((Tw_d @ np.r_[m["centre"], 1.0])[:3],
            Tw_d[:3, :3] @ m["n"])


def pad_mid(fk, arm, q, grip):
    Tl, Tr = fk.poses(arm, q, ["robotiq_85_left_finger_tip_link",
                               "robotiq_85_right_finger_tip_link"],
                      gripper=grip)
    return (Tl[:3, 3] + Tr[:3, 3]) / 2.0


def look(node, tries=8):
    """Re-measure until the cube is seen. Says WHY each time it is not."""
    for i in range(tries):
        try:
            m = measure(node)
        except NoFrames as e:
            print("    [%d] %s" % (i, e))
            node.spin(0.3)
            continue
        if m is None:
            print("    [%d] no usable cube+plane in this frame" % i)
        elif m["n_pts"] < 80:
            print("    [%d] only %d points" % (i, m["n_pts"]))
        elif not (CUBE_MM_RANGE[0] <= m["height_mm"] <= CUBE_MM_RANGE[1]):
            # NOT A CUBE. A green blob 268 mm tall is the mannequin, a chair
            # or the backdrop -- and a grasp planned on it is a grasp planned
            # on a person. The size gate separates "green" from "the cube".
            print("    [%d] rejected: %.1f mm tall, outside %.0f-%.0f mm"
                  % (i, m["height_mm"], CUBE_MM_RANGE[0], CUBE_MM_RANGE[1]))
        else:
            return m
        node.spin(0.3)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    node = TopDownPicker(a.arm)
    if not node.preflight():
        return 2
    node.set_deadband(0.10)
    node._hold = node.fresh_q().copy()
    fk, arm = node.fk, a.arm

    tips = tool_extent(fk, arm, node.fresh_q(), CUBE_GRIP)["tips_past_pads"]
    print("\nfinger tips reach %.1f mm past the pad midpoint (from the "
          "collision meshes)" % (tips * 1000))
    w = node.scene_cube()
    print("room camera: %s" % ("sees a green cube at (%.0f, %.0f)"
                               % (w["u"], w["v"]) if w else "no green cube"))

    if not a.dry_run:
        node.hold_gripper(GRIP_OPEN, 2.0, "OPEN")

    print("\n=== 1/4  what is on the table ===")
    tgt = None
    for i in range(12):
        tgt = node.scene_pick_target()
        if tgt:
            break
        sc = node.scene
        if sc is None:
            print("    [%d] no /srl/scene yet -- is "
                  "scripts/srl_scene_understanding.py running?" % i)
        else:
            cam = (sc.get("cameras") or {}).get("gripper/%s" % arm) or {}
            objs = cam.get("objects") or []
            print("    [%d] %d object(s) on the table, none pickable: %s"
                  % (i, len(objs),
                     ", ".join("%s %.0fmm" % (o["name"], o["width_mm"])
                               for o in objs) or "-"))
        node.spin(0.5)
    if tgt is None:
        print("no pickable cube on the table from this view.")
        return 1
    o = tgt["obj"]
    cw = tgt["world"]
    m = {"height_mm": o["height_mm"]}
    print("  TABLE-SEGMENTED: %s %s, %.0f x %.0f mm, %.2f m away, %d points"
          % (o["colour"], o["name"], o["width_mm"], o["height_mm"],
             o["centre_cam"][2], o["n_points"]))
    print("  cube in world: (%+.4f, %+.4f, %+.4f)" % tuple(cw))

    print("\n=== 2/4  hover DIRECTLY ABOVE it, tool straight down ===")
    top_z = cw[2] + m["height_mm"] / 2000.0
    hover = np.array([cw[0], cw[1], top_z + HOVER_M])
    best = solve_down(fk, arm, hover, node.fresh_q())
    if best is None:
        print("  no straight-down pose reaches %.0f mm above the cube."
              % (HOVER_M * 1000))
        return 1
    travel, q_h, ep, yaw = best
    d, where = node.safe(q_h)
    print("  yaw %3.0f deg, travel %.1f deg, residual %.2f mm, wearer %.4f m "
          "(%s)" % (yaw, math.degrees(travel), ep * 1000, d, where))
    if a.dry_run:
        print("\ndry run: nothing commanded")
        return 0
    if not node.go(q_h, "HOVER"):
        return 1

    print("\n=== 3/4  descend, re-measuring every step ===")
    print("  %-4s %9s %9s %9s  %s"
          % ("it", "gap_mm", "cube_mm", "clear_m", "action"))
    for it in range(1, MAX_ITERS + 1):
        t2 = node.scene_pick_target()
        if t2 is None:
            print("  cube no longer segmented -- the wrist camera cannot see "
                  "its own grasp this close; holding the last fix")
            break
        cw = t2["world"]
        m = {"height_mm": t2["obj"]["height_mm"]}
        q = node.fresh_q()
        mid = pad_mid(fk, arm, q, CUBE_GRIP)
        # STOP SO THE FINGER TIPS CLEAR THE TABLE. The pads sit at the cube's
        # mid-height; the tips are `tips` lower, and they must stay above the
        # surface the cube is standing on.
        want_z = cw[2] + max(0.0, tips - m["height_mm"] / 2000.0) + 0.002
        gap = mid[2] - want_z
        clear, _ = node.safe(q)
        if gap <= TOL_M:
            print("  %-4d %9.1f %9.1f %9.4f  AT GRASP HEIGHT"
                  % (it, gap * 1000, m["height_mm"], clear))
            break
        step = min(gap, 0.020)
        tgt = np.array([cw[0], cw[1], mid[2] - step])
        best = solve_down(fk, arm, tgt, q)
        if best is None:
            print("  %-4d %9.1f %9.1f %9.4f  no straight-down IK here -- "
                  "stopping" % (it, gap * 1000, m["height_mm"], clear))
            break
        print("  %-4d %9.1f %9.1f %9.4f  down %.1f mm"
              % (it, gap * 1000, m["height_mm"], clear, step * 1000))
        if not node.go(best[1], "descend-%d" % it):
            break

    print("\n=== 4/4  close and lift ===")
    before = node.scene_cube()
    node.hold_gripper(GRIP_SQUEEZE, 4.0, "CLOSE")
    node.spin(1.0)
    fb = node.grip_fb
    held = None
    if fb:
        cmd, meas = fb
        held = meas < cmd - 0.06
        print("  gripper: commanded %.3f, measured %.3f -> %s"
              % (cmd, meas, "STOPPED SHORT (holding something)" if held
                 else "closed fully (EMPTY)"))
    q = node.fresh_q()
    mid = pad_mid(fk, arm, q, GRIP_SQUEEZE)
    for h in (0.05, 0.12):
        best = solve_down(fk, arm, mid + np.array([0, 0, h]), q,
                          grip=GRIP_SQUEEZE)
        if best and node.go(best[1], "LIFT-%dmm" % int(h * 1000)):
            q = node.fresh_q()
    node.spin(1.5)
    after = node.scene_cube()
    if before and after:
        dv = before["v"] - after["v"]
        print("  room camera: cube v %.0f -> %.0f (%+.0f px, up is positive)"
              % (before["v"], after["v"], dv))
    print("\n*** PICKED ***" if held else "\n  not holding the cube.")
    return 0 if held else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        try:
            rclpy.shutdown()
        except Exception:                                     # noqa: BLE001
            pass
