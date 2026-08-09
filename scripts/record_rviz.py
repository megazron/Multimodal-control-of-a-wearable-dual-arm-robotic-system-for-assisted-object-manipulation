#!/usr/bin/env python3
"""REAL SCREEN RECORDINGS OF RVIZ, with the task objects visible and moving.

    python3 scripts/record_rviz.py --all
    python3 scripts/record_rviz.py --task t3 --scenario S2

Writes recordings/verification/<task>/<scenario>/<condition>/rviz.mp4 alongside
the TF-rendered clip.mp4, the plots, the rosbag and summary.json.

WHY A VIRTUAL DISPLAY, MEASURED NOT ASSUMED
-------------------------------------------
x11grab on WSLg's own :0 records BLACK. Measured: full-screen grab of the live
desktop with RViz visible gives mean pixel value 0.0 -- XWayland windows are
composited by the Wayland compositor and their pixels never reach the X root
window that x11grab reads.

Xvfb has no compositor, so its root window really does hold the rendered
pixels. RViz on :99 with LIBGL_ALWAYS_SOFTWARE grabs at mean 126.8, std 110.4,
40742 distinct colours. That is the whole reason for the virtual display: not
convenience, but that the obvious approach produces a black rectangle.

WHAT IS IN FRAME
----------------
`verification_capture.rviz` strips every panel so the 3-D view fills the
window, and pins the camera in front of the wearer at a FIXED pose so all
clips are comparable. T5 also gets a second, closer angle from the wearer's
side, because T5 is the handover and the interesting thing happens at their
waist.

THE OBJECTS ARE THE POINT
-------------------------
A clip of a gripper closing on nothing verifies nothing. Every object is a
visible RViz marker at its real dimensions, and its pose is computed from the
LIVE gripper pose each frame, so it moves with the arm:

  T3  rigid tray spanning the two grippers, with a ball on top that ROLLS to
      the low side and FALLS OFF past 11.3 deg of tilt
  T6  compliant sling drawn as its actual catenary, with the ball in the
      bottom of the V; it drops out when the sag falls under one ball
      diameter, which is the documented failure mode
  T2  container carried by the holding gripper, blocks carried by the filling
      gripper and LEFT INSIDE the container on release
  T5  tool carried from the cradle and left at the receive point
  T7/T9  the moving target the operator is tracking
  T8  the goal, plus the wearer stance the task requires

Overlay text (task, scenario, condition, elapsed, live metric) is drawn as
TEXT_VIEW_FACING markers rather than burned in afterwards, so the number on
screen is the number from that frame and cannot drift out of sync with it.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy
import rclpy.time
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
MODE_BUCKET = "00_unclassified_scripted_playback"
from record_verification import (Driver, densify, clearance_all,      # noqa: E402
                                 wearer_prims, BIM, OUT, FFMPEG)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRATCH = "/tmp/claude-1000/-home-gausms-kortex-ws/3732aa29-5a7e-4c8e-b77e-379233bdc9c9/scratchpad"
RVIZ_CFG = os.path.join(ROOT, "src/srl_experiments/config/verification_capture.rviz")
# FOUR SIMULTANEOUS VIEWS, one RViz per virtual display, all watching the same
# ROS graph -- so the four recordings are frame-for-frame the same trial, not
# four passes that might diverge. One angle hides things: the front view
# flattens reach depth, the top view is where a coordination error is obvious,
# and only a tight gripper view answers "did it actually grab it".
VIEWS = {
    #  name       display  yaw      pitch  dist  focal(x,y,z)         hud
    "front":   (":91", 1.5708, 0.32, 2.25, (0.0, 0.26, 1.16), True),
    "back":    (":95", -1.5708, 0.30, 2.25, (0.0, 0.10, 1.20), False),
    "left":    (":92", 0.0000, 0.20, 2.10, (0.0, 0.30, 1.15), False),
    "right":   (":96", 3.1416, 0.20, 2.10, (0.0, 0.30, 1.15), False),
    "iso":     (":97", 0.9000, 0.45, 2.40, (0.0, 0.26, 1.16), False),
    "top":     (":93", 1.5708, 1.40, 2.30, (0.0, 0.30, 1.15), False),
    "gripper": (":94", 1.5708, 0.25, 0.40, (0.0, 0.0, 0.0), False),
}
# The quad is a quick-review tile, not all seven: front | left / top | gripper
QUAD = ("front", "left", "top", "gripper")
VW, VH = 800, 500          # per view; the 2x2 tile is 1600x1000
W, H = VW, VH
CONDITIONS = ("direct", "assisted", "shared")

# DEFAULTS ONLY. The live value comes from the scenario, because this module
# constant was the stale nine-task 350 mm and the five-task spec is 540 mm.
# At the respec'd 500 mm separation a 350 mm sling is GEOMETRICALLY
# IMPOSSIBLE: (L/2)^2 - (s/2)^2 = -0.032, so sag() takes the root of a
# negative and the sling rendered as a flat line with the ball instantly
# fallen. All nine f4 clips failed the verifier on it, correctly. Stale
# geometry surviving in the renderer is exactly what the re-record existed to
# remove.
SLING_L, BALL_R = 0.540, 0.020
TILT_FAIL_DEG = 11.3

# ---------------------------------------------------------------- grippers
# THE GRIPPER WAS NEVER COMMANDED. mock_components boots the Robotiq driven
# knuckle at 0.793 rad -- fully closed on nothing -- so every clip showed a
# shut hand throughout, and the "grasp" existed only in the marker layer. A
# pick rendered with a closed gripper is not a pick.
GRIP_OPEN = 0.05
# Robotiq 2F-85: 85 mm stroke across roughly 0 -> 0.8 rad of driven knuckle,
# so closing on a w-mm object leaves the knuckle short of full closure.
# bimanual_metrics calls 0.10..0.74 "holding" and >= 0.74 "free air" -- closed
# on NOTHING -- so an object grasp must land inside that band, which is what
# makes the marker gate meaningful rather than decorative.
def grip_for(width_mm):
    return max(0.12, min(0.70, 0.8 * (1.0 - float(width_mm) / 85.0)))


GRIP_HOLD_MIN, GRIP_FREE_AIR = 0.10, 0.74

# Fraction of the transit at which the plan hands the gripper back to the
# schedule so it can open and PLACE. These match the thresholds already inside
# grip_schedule(), so the release stays defined in exactly one place; the hold
# above only stops the object being dropped BEFORE it.
RELEASE_FRAC = {"t2": 0.80, "t5": 0.86}


def holding(knuckle, width_mm=None):
    """True only when the fingers are closed ON SOMETHING.

    With `width_mm` given, the test is that the fingers have actually reached
    the object's own width rather than merely left the open position. Without
    it the band alone attached the marker the instant the knuckle passed 0.10,
    so a 40 mm block (which needs 0.42) jumped to the gripper while the
    fingers were still visibly open -- the object appeared to be grasped
    before it was touched. 0.90 of the target absorbs the mock's first-order
    tracking lag without accepting a gripper that has barely moved.
    """
    if knuckle is None or knuckle != knuckle:
        return False
    if width_mm is not None:
        return knuckle >= 0.90 * grip_for(width_mm)
    return GRIP_HOLD_MIN <= knuckle < GRIP_FREE_AIR


KNUCKLE = "%s_robotiq_85_left_knuckle_joint"

# ---------------------------------------------------------------- grasping
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from srl_autonomy import grasp_library as gl                    # noqa: E402

# Which library object each task's grasped item is. Sizes drive both the
# finger width and the minor-axis alignment.
TASK_OBJECT = {"t2": "tag_0",      # 40 mm cube
               "t5": "tag_5",      # narrow rod, the tool handle
               "t3": "tag_2", "t6": "tag_2"}


def _q(t):
    """Pass orientations as plain 4-tuples -- see the note in solve_joints."""
    return tuple(float(v) for v in t)


def plan_grasp(dr, arm, position, object_id, yaw=0.0, step=0.02):
    """A REAL grasp: wrist aligned to the object, standoff, validated approach.

    Uses the existing generator (srl_autonomy/grasp_library) rather than a
    second implementation. For each candidate yaw it requires the pre-grasp
    standoff, the grasp itself AND every point on the straight-line approach
    between them to solve -- a grasp that passes IK at the endpoint but strikes
    something on the way in is the documented failure mode.

    Returns a plan, or a dict with `refused` naming why. A refusal is
    information and is reported rather than silently worked around.
    """
    cands = gl.candidates(object_id, position, yaw, 8)
    first_why = None
    for c in cands:
        pre = gl.pregrasp(c)
        q = _q(c["quat"])
        if dr.solve_joints(arm, list(pre["position"]), q) is None:
            first_why = first_why or "pre-grasp standoff not solvable"
            continue
        if dr.solve_joints(arm, list(c["position"]), q) is None:
            first_why = first_why or "grasp pose not solvable"
            continue
        path = densify([list(pre["position"]), list(c["position"])], step)
        if any(dr.solve_joints(arm, w, q) is None for w in path):
            first_why = first_why or "approach path blocked"
            continue
        return dict(ok=True, arm=arm, quat=c["quat"],
                        grasp=list(c["position"]),
                    pregrasp=list(pre["position"]), path=path,
                    width_mm=1000.0 * gl.graspable_width(object_id),
                    yaw_deg=math.degrees(c["yaw_offset"]),
                    approach_m=c["approach"])
    return dict(ok=False,
                refused=first_why or "no candidate solvable",
                width_mm=1000.0 * gl.graspable_width(object_id))


# ------------------------------------------------------------------ markers
def mk(ns, i, typ, frame="world"):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = i
    m.type = typ
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.color.a = 1.0
    return m


def rgba(m, r, g, b, a=1.0):
    m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), float(a)
    return m


def at(m, p):
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
    return m


def scale(m, x, y, z):
    m.scale.x, m.scale.y, m.scale.z = float(x), float(y), float(z)
    return m


def quat_from_axis(v):
    """Orientation whose +x axis points along v (for the tray)."""
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    x = v / n
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(x, up))) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    R = np.column_stack([x, y, z])
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


class Scene:
    """Task objects, posed from the LIVE gripper poses every frame.

    State that must persist between frames -- has the ball fallen, how many
    blocks are in the container -- lives here, because an object that
    un-drops itself on the next frame is not a simulation of anything.
    """

    def __init__(self, task, run):
        self.task = task
        self.run = run
        self.ball_fallen = False
        self.ball_fall_z = None
        self.placed = []          # world positions of blocks left in the box
        self.carrying_block = False
        self.tool_released_at = None
        self.knuck = {}
        self.grip_log = []        # (t, arm, knuckle) for the pixel check

    # -------------------------------------------------------------- build
    def markers(self, t, el, er, phase, frac, metric_text, knuck=None):
        """knuck: {"left": rad, "right": rad} MEASURED from /joint_states.

        The carried object is attached only while `holding()` is true for the
        arm that should have it, so the marker follows the gripper instead of
        substituting for it.
        """
        self.knuck = knuck or {}
        A = MarkerArray()
        # DELETEALL FIRST, EVERY FRAME. The array is TRANSIENT_LOCAL and ids
        # are reassigned sequentially, so without this the previous run's
        # container and blocks stayed on screen underneath the next task's
        # objects -- two overlays superimposed, two scenes in one frame. It
        # also handles a frame with fewer markers than the last one, which
        # would otherwise leave the surplus ids orphaned.
        clr = Marker()
        clr.header.frame_id = "world"
        clr.action = Marker.DELETEALL
        A.markers.append(clr)
        i = [0]

        def add(m):
            m.id = i[0]
            i[0] += 1
            A.markers.append(m)

        # ---- context. The first version put a 0.9 x 0.5 x 1.0 m box at
        # y=0.62, which sat directly between the camera and the arms and hid
        # the entire task. A support is only drawn where an object actually
        # rests, as a thin slab, and never in front of the work.
        if self.task == "t2":
            pk = np.asarray(self.run["sc"]["pick"], float)
            sl = mk("scene", 0, Marker.CUBE)
            add(rgba(scale(at(sl, (pk[0], pk[1], pk[2] - 0.075)),
                           0.30, 0.24, 0.02), .35, .30, .26))
        elif self.task == "t5":
            ob = np.asarray(self.run["sc"]["object"], float)
            sl = mk("scene", 0, Marker.CUBE)
            add(rgba(scale(at(sl, (ob[0], ob[1], ob[2] - 0.045)),
                           0.26, 0.20, 0.02), .35, .30, .26))

        task = self.task
        if task in ("t3", "t6") and el is not None and er is not None:
            # ONLY once the arms are actually holding it. During the approach
            # the grippers are still ~1.46 m apart (they start at home), so a
            # 350 mm sling reads as taut and the ball "fell" before the task
            # began -- which is how S4 reported a drop on a scenario whose sag
            # (58 mm) comfortably clears the 40 mm ball.
            self._carry(add, el, er, live=(phase != "approach"))
        elif task == "t2":
            self._fill(add, el, er, frac)
        elif task == "t5":
            self._tool(add, el, er, frac)
        elif task in ("t7", "t9"):
            for arm, p in (self.run["paths"].items()):
                cur = p[min(int(frac * (len(p) - 1)), len(p) - 1)]
                # 100 mm and translucent, not 70 mm and solid: at 70 mm the
                # gripper ARRIVES ON the target and hides it completely --
                # green vanished from frames 4..8 of a clip where nothing was
                # wrong, and for T9 an invisible target makes the whole point
                # (the arm holding a world-fixed spot while the base sways)
                # impossible to read. A translucent halo stays visible around
                # the gripper.
                m = mk("target", 0, Marker.SPHERE)
                add(rgba(scale(at(m, cur), .10, .10, .10), .1, .95, .25, .45))
        elif task == "t8":
            tgt = self.run.get("target")
            if tgt:
                m = mk("target", 0, Marker.SPHERE)
                add(rgba(scale(at(m, tgt), .09, .09, .09), .1, .9, .2, .9))
                tx = mk("target", 1, Marker.TEXT_VIEW_FACING)
                tx.text = "TARGET (needs wearer to %s)" % \
                    self.run.get("stance", "reposition").replace("_", " ")
                add(rgba(scale(at(tx, (tgt[0], tgt[1], tgt[2] + 0.16)),
                               .001, .001, .07), .6, 1., .6))

        # ---- overlay text, in the scene so it cannot drift out of sync
        hdr = mk("hud", 0, Marker.TEXT_VIEW_FACING)
        # RViz renders TEXT_VIEW_FACING with very narrow spaces, so
        # "T3  S2  [direct]" came out as "T3S2[direct]". Explicit separators.
        # THE RUN'S OWN NAME, not the scene code. `task` here is the RENDERER
        # code (t3 tray, t6 sling, ...), so a five-task clip labelled itself
        # "T3" when it is f3. A clip that misnames itself is worse than an
        # unlabelled one: it looks authoritative.
        hdr.text = "%s | %s | %s" % (self.run["task"].upper(),
                                     self.run["scenario"],
                                     self.run["condition"])
        add(rgba(scale(at(hdr, (0.0, 0.0, 1.95)), .001, .001, .085), 1, 1, 1))
        sub = mk("hud", 1, Marker.TEXT_VIEW_FACING)
        note = self.run.get("grasp_note", "")
        sub.text = "t=%.1f s | %s | %s%s" % (
            t, metric_text, phase, (" | " + note) if note else "")
        add(rgba(scale(at(sub, (0.0, 0.0, 1.85)), .001, .001, .062),
                 1.0, .85, .3))
        # THE CAVEAT IS BURNED INTO THE PICTURE, not left to the index. A clip
        # whose motion comes from a direct /compute_ik call is not something an
        # operator could command, and a viewer who never opens INDEX.md must
        # still not conclude that teleoperated grasping works.
        cav = self.run.get("caveat", "")
        if cav:
            cv = mk("hud", 2, Marker.TEXT_VIEW_FACING)
            cv.text = cav
            add(rgba(scale(at(cv, (0.0, 0.0, 1.76)), .001, .001, .052),
                     1.0, .30, .25))
        return A

    # -------------------------------------------------------------- T3/T6
    def _carry(self, add, el, er, live=True):
        L = float(self.run.get("sc", {}).get("length") or SLING_L)
        a, b = np.asarray(el, float), np.asarray(er, float)
        mid = (a + b) / 2.0
        sep = float(np.linalg.norm(a - b))
        if not live or sep > 0.60:
            return          # not holding it yet -- draw nothing, fail nothing
        if not (holding(self.knuck.get("left"), 20)
                and holding(self.knuck.get("right"), 20)):
            return          # fingers are not closed on it, so it is not held
        dz = float(a[2] - b[2])
        tilt = math.degrees(math.atan2(abs(dz), max(sep, 1e-6)))
        if self.task == "t3":
            m = mk("object", 0, Marker.CUBE)
            q = quat_from_axis(b - a)
            at(m, mid)
            m.pose.orientation.x, m.pose.orientation.y, \
                m.pose.orientation.z, m.pose.orientation.w = q
            add(rgba(scale(m, sep, 0.26, 0.022), .80, .58, .30))
            # the ball rolls to the LOW side and leaves past the threshold
            if not self.ball_fallen and tilt > TILT_FAIL_DEG:
                self.ball_fallen = True
                self.ball_fall_z = mid[2]
            low = a if a[2] < b[2] else b
            roll = min(1.0, tilt / TILT_FAIL_DEG)
            bp = mid + (low - mid) * roll
            if self.ball_fallen:
                self.ball_fall_z = max(0.05, (self.ball_fall_z or bp[2]) - 0.05)
                bp = np.array([bp[0], bp[1], self.ball_fall_z])
            ball = mk("object", 1, Marker.SPHERE)
            add(rgba(scale(at(ball, (bp[0], bp[1], bp[2] + 0.03)),
                           2 * BALL_R, 2 * BALL_R, 2 * BALL_R),
                     .9, .15, .15) if self.ball_fallen else
                rgba(scale(at(ball, (bp[0], bp[1], bp[2] + 0.03)),
                           2 * BALL_R, 2 * BALL_R, 2 * BALL_R), .95, .75, .1))
        else:
            v = max(0.0, (L / 2) ** 2 - (sep / 2) ** 2)
            sag = math.sqrt(v)
            ts = np.linspace(0, 1, 21)
            pts = np.array([a + (b - a) * s for s in ts])
            pts[:, 2] -= sag * np.sin(np.pi * ts)
            ln = mk("object", 0, Marker.LINE_STRIP)
            ln.scale.x = 0.012
            from geometry_msgs.msg import Point
            for p in pts:
                pt = Point()
                pt.x, pt.y, pt.z = (float(q) for q in p)
                ln.points.append(pt)
            add(rgba(ln, .55, .35, .18))
            held = sag >= 2 * BALL_R
            if not held:
                self.ball_fallen = True
            low = pts[int(np.argmin(pts[:, 2]))]
            if self.ball_fallen:
                self.ball_fall_z = max(0.05, (self.ball_fall_z
                                              if self.ball_fall_z is not None
                                              else low[2]) - 0.05)
                low = np.array([low[0], low[1], self.ball_fall_z])
            ball = mk("object", 1, Marker.SPHERE)
            c = (.9, .15, .15) if self.ball_fallen else (.95, .75, .1)
            add(rgba(scale(at(ball, (low[0], low[1], low[2] + BALL_R)),
                           2 * BALL_R, 2 * BALL_R, 2 * BALL_R), *c))

    # ----------------------------------------------------------------- T2
    def _fill(self, add, el, er, frac):
        sc = self.run["sc"]
        ha, fa = sc["hold_arm"], sc["fill_arm"]
        hold_ee = el if ha == "left" else er
        fill_ee = el if fa == "left" else er
        if hold_ee is None or fill_ee is None:
            return
        hold_ee = np.asarray(hold_ee, float)
        fill_ee = np.asarray(fill_ee, float)
        off = np.asarray(sc["release"], float) - np.asarray(sc["hold"], float)
        opening = hold_ee + off                      # the box travels with it
        box = mk("object", 0, Marker.CUBE)
        # TEAL, not blue: the wearer's shirt is blue and at 55% alpha the
        # container disappeared into it.
        add(rgba(scale(at(box, (opening[0], opening[1], opening[2] - 0.06)),
                       0.20, 0.20, 0.13), .05, .75, .70, .45))
        # 45% alpha, not 92%: an opaque container HID the blocks that had been
        # dropped into it, which is the one thing this clip has to show.
        handle = mk("object", 1, Marker.CUBE)
        mid = (hold_ee + opening) / 2.0
        add(rgba(scale(at(handle, (mid[0], mid[1], mid[2] - 0.02)),
                       float(abs(off[0])), 0.035, 0.02), .05, .60, .58))
        # blocks waiting at the pick point
        pick = np.asarray(sc["pick"], float)
        n_left = max(0, 3 - len(self.placed) - (1 if self.carrying_block else 0))
        for k in range(n_left):
            b = mk("blocks", k, Marker.CUBE)
            side = sc.get("block_mm", 40) / 1000.0
            add(rgba(scale(at(b, (pick[0] + 0.05 * k, pick[1],
                                  pick[2] - 0.02)), side, side, side),
                     .9, .55, .1))
        # the carried block, and the ones already dropped in
        side = sc.get("block_mm", 40) / 1000.0
        # ATTACH ON THE MEASURED FINGERS, not on a path fraction. The block
        # rides the gripper only while it is actually closed on it, and is
        # released the moment the fingers open.
        k = self.knuck.get(fa)
        near_pick = float(np.linalg.norm(fill_ee - pick)) < 0.12
        if holding(k, side * 1000.0) and (self.carrying_block or near_pick):
            self.carrying_block = True
            b = mk("carry", 0, Marker.CUBE)
            add(rgba(scale(at(b, (fill_ee[0], fill_ee[1], fill_ee[2] - 0.055)),
                           side, side, side), 1.0, .45, .0))
        elif self.carrying_block and not holding(k, side * 1000.0):
            self.carrying_block = False
            self.placed.append(np.array([opening[0], opening[1],
                                         opening[2] - 0.06]))
        for k, p in enumerate(self.placed):
            p = hold_ee + off + np.array([0.0, 0.0, -0.035])
            b = mk("placed", k, Marker.CUBE)
            add(rgba(scale(at(b, (p[0] + 0.045 * k - 0.045, p[1],
                                  p[2] + 0.005)),
                           side, side, side), .15, 1.0, .25))

    # ----------------------------------------------------------------- T5
    def _tool(self, add, el, er, frac):
        sc = self.run["sc"]
        arm = sc.get("arm", "right")
        ee = el if arm == "left" else er
        recv = np.asarray(sc["receive"], float)
        obj = np.asarray(sc["object"], float)
        k = self.knuck.get(arm)
        if holding(k, 32) and ee is not None and self.tool_released_at is None:
            # in the hand
            p = np.asarray(ee, float) + np.array([0.0, 0.0, -0.03])
            self.carrying_block = True
        elif self.carrying_block and not holding(k, 32):
            p = recv
            self.tool_released_at = recv
        elif self.tool_released_at is not None:
            p = recv
        else:
            p = obj
        h = mk("object", 0, Marker.CUBE)
        add(rgba(scale(at(h, p), 0.13, 0.032, 0.024), .2, .2, .22))
        head = mk("object", 1, Marker.CUBE)
        add(rgba(scale(at(head, (p[0] - 0.08, p[1], p[2])),
                       0.055, 0.04, 0.024), .8, .75, .1))
        r = mk("recv", 0, Marker.SPHERE)
        add(rgba(scale(at(r, recv), .05, .05, .05), .1, .9, .2, .55))


# ------------------------------------------------------------------ capture
CFG_DIR = os.path.join(SCRATCH, "rvizcfg")


def write_cfg(name, arm="left"):
    """One RViz config per view. The HUD lives on its own topic so only the
    FRONT view shows it -- four copies of the same text is clutter."""
    yaw, pitch, dist, focal, hud = VIEWS[name][1:]
    tgt = "%s_end_effector_link" % arm if name == "gripper" else "world"
    disp = ["""    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 14
      Color: 90; 90; 90
      Alpha: 0.4
      Reference Frame: world
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Name: TaskObjects
      Enabled: true
      Namespaces: {}
      Topic:
        Depth: 20
        Durability Policy: Transient Local
        Value: /task_objects
      Value: true"""]
    if hud:
        disp.append("""    - Class: rviz_default_plugins/MarkerArray
      Name: HUD
      Enabled: true
      Namespaces: {}
      Topic:
        Depth: 20
        Durability Policy: Transient Local
        Value: /task_hud
      Value: true""")
    txt = """Panels: []
Visualization Manager:
  Class: ""
  Name: root
  Displays:
%s
  Global Options:
    Background Color: 45; 45; 48
    Fixed Frame: world
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: %s
      Distance: %.3f
      Focal Point:
        X: %.3f
        Y: %.3f
        Z: %.3f
      Pitch: %.4f
      Yaw: %.4f
      Target Frame: %s
      Near Clip Distance: 0.01
      Invert Z Axis: false
      Value: Orbit (rviz_default_plugins)
Window Geometry:
  Height: %d
  Width: %d
  Hide Left Dock: true
  Hide Right Dock: true
""" % ("\n".join(disp), name, dist, focal[0], focal[1], focal[2],
       pitch, yaw, tgt, VH, VW)
    os.makedirs(CFG_DIR, exist_ok=True)
    # PER-ARM FILENAME for the gripper view. With a single gripper.rviz path
    # the "is it already running?" check matched the instance started for the
    # OTHER arm, so the restart never fired and a right-arm task was filmed
    # with the camera still bolted to the left hand. Caught because the
    # pixel check saw no finger motion on T5 while the joint trace showed a
    # clean open-close-open.
    fn = ("gripper_%s" % arm) if name == "gripper" else name
    p = os.path.join(CFG_DIR, "%s.rviz" % fn)
    open(p, "w").write(txt)
    return p


def _running(pat):
    out = subprocess.run(["ps", "-eo", "args", "--no-headers"],
                         capture_output=True, text=True).stdout
    return any(pat in l for l in out.splitlines())


_GRIP_ARM = {"cur": None}


def ensure_display(log, gripper_arm="left"):
    """Four Xvfb displays, each with its own RViz. Idempotent.

    The gripper camera follows a LINK (Target Frame = <arm>_end_effector_link)
    so it tracks the hand rather than a fixed point. That frame is baked into
    the config, so this restarts only that one view when the active arm
    changes -- which is rare, because the sweep is grouped by task.
    """
    for name, (disp, *_rest) in VIEWS.items():
        if not _running("Xvfb %s" % disp):
            subprocess.Popen(["setsid", "/usr/bin/Xvfb", disp, "-screen", "0",
                              "%dx%dx24" % (VW, VH)],
                             stdout=open(log + ".xvfb", "a"),
                             stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3)
    started = False
    for name, (disp, *_rest) in VIEWS.items():
        cfg = write_cfg(name, gripper_arm)
        if name == "gripper":
            # stop any gripper RViz bound to the OTHER arm
            other = write_cfg("gripper", "right" if gripper_arm == "left"
                              else "left")
            if _running("rviz2 -d %s" % other):
                subprocess.run(["pkill", "-f", "rviz2 -d %s" % other],
                               capture_output=True)
                time.sleep(1.5)
        if not _running("rviz2 -d %s" % cfg):
            env = dict(os.environ, DISPLAY=disp, LIBGL_ALWAYS_SOFTWARE="1",
                       GALLIUM_DRIVER="llvmpipe", QT_QPA_PLATFORM="xcb")
            subprocess.Popen(["rviz2", "-d", cfg], env=env,
                             stdout=open(log + ".rviz." + name, "w"),
                             stderr=subprocess.STDOUT, start_new_session=True)
            started = True
    _GRIP_ARM["cur"] = gripper_arm
    if started:
        time.sleep(20)


def start_grabs(out_dir):
    """One ffmpeg per view, started together so the four are synchronised."""
    procs = {}
    for name, (disp, *_rest) in VIEWS.items():
        path = os.path.join(out_dir, "rviz_%s.mp4" % name)
        procs[name] = subprocess.Popen(
            [FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", "%dx%d" % (VW, VH), "-framerate", "12",
             "-i", "%s.0" % disp, "-c:v", "libx264", "-preset", "ultrafast",
             "-pix_fmt", "yuv420p", path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
    return procs


def stop_grabs(procs):
    for p in procs.values():
        try:
            p.communicate(input=b"q", timeout=15)
        except Exception:                                      # noqa: BLE001
            p.terminate()
            try:
                p.wait(timeout=8)
            except Exception:                                  # noqa: BLE001
                p.kill()


def make_quad(out_dir):
    """2x2 tile: the one file worth watching. front | side / top | gripper."""
    src = [os.path.join(out_dir, "rviz_%s.mp4" % n) for n in QUAD]
    if not all(os.path.exists(p) and os.path.getsize(p) > 20000 for p in src):
        return False
    dst = os.path.join(out_dir, "rviz_quad.mp4")
    r = subprocess.run(
        [FFMPEG, "-y", "-loglevel", "error",
         "-i", src[0], "-i", src[1], "-i", src[2], "-i", src[3],
         "-filter_complex",
         "[0:v][1:v]hstack=inputs=2[t];[2:v][3:v]hstack=inputs=2[b];"
         "[t][b]vstack=inputs=2[v]",
         "-map", "[v]", "-c:v", "libx264", "-pix_fmt", "yuv420p", dst],
        capture_output=True)
    return r.returncode == 0 and os.path.exists(dst)


# ------------------------------------------------------------------- runs
def build_runs(spec, only_task=None, only_scen=None):
    """Same scenarios as the TF recorder, plus the scenario dict itself so the
    scene can pose objects (hold/release/pick, the tool cradle, the stance)."""
    T = spec["tasks"]
    runs = []

    def add(task, scen, paths, sc, target=None, note="", stance=""):
        if only_task and task != only_task:
            return
        if only_scen and not scen.startswith(only_scen):
            return
        runs.append(dict(task=task, scenario=scen, paths=paths, sc=sc,
                         target=target, note=note, stance=stance,
                         condition=""))

    for key, task in (("T3_rigid", "t3"), ("T6_compliant", "t6")):
        for name, s in T.get(key, {}).items():
            sep = float(s.get("sep", 0.310))
            p = s["path"]
            add(task, name,
                {"left": [[w[0] + sep / 2, w[1], w[2]] for w in p],
                 "right": [[w[0] - sep / 2, w[1], w[2]] for w in p]}, s,
                note="%s, %.0f mm span" % (
                    "rigid tray" if task == "t3" else "compliant sling",
                    1000 * sep))
    for name, s in T.get("T2_hold_fill", {}).items():
        add("t2", name, {s["fill_arm"]: s["fill_path"],
                         s["hold_arm"]: [s["hold"], s["hold"]]}, s,
            target=s["release"],
            note="%s holds, %s fills" % (s["hold_arm"], s["fill_arm"]))
    for name, s in T.get("T5_handover", {}).items():
        arm = s.get("arm", "right")
        o, r = s["object"], s["receive"]
        add("t5", name, {arm: [[o[0], o[1], o[2] + 0.08], o,
                               [o[0], o[1], o[2] + 0.08],
                               [(o[0] + r[0]) / 2, (o[1] + r[1]) / 2,
                                max(o[2], r[2]) + 0.08],
                               [r[0], r[1], r[2] + 0.06], r,
                               [r[0], r[1], r[2] + 0.08]]}, s,
            target=r, note="%s arm delivers to the wearer" % arm)
    for name, s in T.get("T7_pursuit", {}).items():
        sys.path.insert(0, os.path.join(BIM, "t7_pursuit"))
        import targets as tg
        _, L, R = tg.trial_targets(s, 8.0, dt=0.25)
        paths = {}
        if s.get("unimanual") != "right":
            paths["left"] = [list(map(float, q)) for q in L]
        if s.get("unimanual") != "left":
            paths["right"] = [list(map(float, q)) for q in R]
        add("t7", name, paths, s,
            note="pursuit L %.2f / R %.2f m/s" % (s.get("speed_left", 0),
                                                  s.get("speed_right", 0)))
    for name, s in T.get("T8_wearer_assisted_reach", {}).items():
        add("t8", name, {s["arm"]: s["approach_path"]}, s,
            target=s["approach_path"][-1], stance=s["wearer_stance"],
            note="%s arm; wearer must %s" % (s["arm"],
                                             s["wearer_stance"].replace("_", " ")))
    for name, s in T.get("T9_wearer_motion", {}).items():
        amp = float(s["sway_amplitude_m"])
        paths = {}
        for arm, c in (("left", s["centre_left"]), ("right", s["centre_right"])):
            ring = []
            for k in range(17):
                a_ = 2 * math.pi * k / 16
                ring.append([c[0] - amp * math.cos(a_),
                             c[1] - amp * math.sin(a_), c[2]])
            paths[arm] = ring if amp > 0 else [c, c]
        add("t9", name, paths, s,
            note="wearer sway %.0f mm" % (1000 * amp))
    return runs


def live_metric(task, el, er, cmd, sling_l=SLING_L):
    """The number burned into the overlay, chosen per task.

    `el`/`er` arrive as numpy arrays here and as lists elsewhere, so they are
    tested against None explicitly -- `if el` on an array raises.
    """
    if task in ("t3", "t6") and el is not None and er is not None:
        a, b = np.asarray(el), np.asarray(er)
        sep = float(np.linalg.norm(a - b))
        dz = abs(float(a[2] - b[2]))
        if task == "t3":
            # RViz collapses runs of spaces, so units are joined deliberately
            return "tilt=%.1fdeg" % math.degrees(math.atan2(dz, max(sep, 1e-6)))
        v = max(0.0, (sling_l / 2) ** 2 - (sep / 2) ** 2)
        return "sep=%.0fmm | sag=%.0fmm" % (1000 * sep, 1000 * math.sqrt(v))
    errs = []
    for arm, c in cmd.items():
        p = el if arm == "left" else er
        if p is not None:
            errs.append(np.linalg.norm(np.asarray(p) - np.asarray(c)))
    if errs:
        return "tracking=%.1fmm" % (1000 * float(np.mean(errs)))
    return ""


def grip_schedule(task, sc, arm, frac, phase):
    """Commanded knuckle angle for one arm at one instant.

    open before approach -> close on the object at its own width -> hold
    through transit -> open on release. Widths come from the scenario, so the
    fingers stop ON the object rather than slamming to free air, which is what
    makes `holding()` true and lets the marker attach.
    """
    if task == "t2":
        if arm == sc["hold_arm"]:
            return grip_for(35)                      # the container handle
        if phase == "approach" or frac < 0.18:
            return GRIP_OPEN
        if frac < 0.80:
            return grip_for(sc.get("block_mm", 40))
        return GRIP_OPEN
    if task in ("t3", "t6"):
        return GRIP_OPEN if phase == "approach" else grip_for(20)
    if task == "t5":
        if arm != sc.get("arm", "right"):
            return GRIP_OPEN
        if phase == "approach" or frac < 0.22:
            return GRIP_OPEN
        if frac < 0.86:
            return grip_for(32)                      # the tool handle
        return GRIP_OPEN
    return GRIP_OPEN                                  # t7/t8/t9: nothing held


def active_arm(task, sc):
    if task == "t2":
        return sc["fill_arm"]
    if task == "t5":
        return sc.get("arm", "right")
    if task == "t8":
        return sc.get("arm", "left")
    return "left"


def run_one(dr, pub, hud_pub, run, cond, out_dir, fps=10.0):
    """Drive the arms AND the grippers, publish the scene, record four views."""
    run = dict(run, condition=cond)
    # SCENE CODE, not the run's own name. The five-task runs are named f1..f5
    # so their output cannot collide with the stale nine-task clips, but the
    # Scene renders by the proven codes (t2 container, t3 tray, t6 sling, t7
    # targets). Everything downstream that switches on `task` is choosing a
    # RENDERER, so it must see the code.
    task = run.get("scene_code") or run["task"]
    sc = run["sc"]
    scene = Scene(task, run)
    q = {a: dr.ee_quat(a) for a in ("left", "right")}
    smooth = {"direct": 0.0, "assisted": 0.30, "shared": 0.55}[cond]
    dense = {a: (densify(p, 0.015) if len(p) > 1 else list(p))
             for a, p in run["paths"].items()}
    nmax = max(len(p) for p in dense.values())
    stride = max(1, int(math.ceil(nmax / 22.0)))
    os.makedirs(out_dir, exist_ok=True)

    def knuckles():
        return {a: dr.js.get(KNUCKLE % a) for a in ("left", "right")}

    # The gripper needs exactly ONE owner at a time. The scripted grasp issues
    # send_gripper() and then tick() re-issued the frac schedule on the same
    # cycle, so the two alternated every frame: the recorded trace showed the
    # knuckle sawing 0.42 -> 0.18 -> 0.24 through the lift and reaching 0.05
    # (fully open) at the start of transit, i.e. the object was grasped and
    # then immediately dropped, then re-grasped in mid-air when the schedule's
    # frac crossed its closing threshold. `grip_hold` is that owner: while it
    # holds a value for an arm, the schedule does not touch that arm.
    grip_hold = {}

    def set_grips(frac, phase):
        for a in ("left", "right"):
            g = grip_hold.get(a)
            if g is None:
                g = grip_schedule(task, sc, a, frac, phase)
            dr.send_gripper(a, g)

    # ---- plan a REAL grasp where the generator allows one
    plan = None
    grasp_note = ""
    if task in TASK_OBJECT:
        # Attempt a real grasp for EVERY grasping task, not just the one known
        # to work. Where the generator refuses, the refusal is recorded and
        # shown in the overlay, and the clip shows the arm doing what it can.
        if task == "t2":
            arm_g, pick = sc["fill_arm"], list(sc["pick"])
        elif task == "t5":
            arm_g, pick = sc.get("arm", "right"), list(sc["object"])
        else:                                    # t3 / t6: the tray edge
            sep = float(sc.get("sep", 0.310))
            w0 = sc["path"][0]
            arm_g = "left"
            pick = [w0[0] + sep / 2, w0[1], w0[2]]
        plan = plan_grasp(dr, arm_g, pick, TASK_OBJECT[task])
        if plan.get("ok"):
            grasp_note = ("top-down grasp, wrist yaw %+.0f deg, standoff %.0f mm"
                          % (plan["yaw_deg"], 1000 * plan["approach_m"]))
        else:
            grasp_note = "GRASP REFUSED: %s" % plan["refused"]
        print("      grasp plan: %s" % grasp_note)
    run["grasp_note"] = grasp_note
    run["plan"] = plan

    go_home(dr)
    set_grips(0.0, "approach")          # OPEN before the approach
    dr.spin(1.0)
    procs = start_grabs(out_dir)
    time.sleep(1.2)
    t0 = time.monotonic()
    frames = 0
    prev = {}
    grip_trace = []

    def tick(phase, frac, cmd):
        nonlocal frames
        set_grips(frac, phase)
        el = dr.link_pose("left_end_effector_link")
        er = dr.link_pose("right_end_effector_link")
        k = knuckles()
        grip_trace.append(dict(t=round(time.monotonic() - t0, 2), phase=phase,
                               left=k.get("left"), right=k.get("right")))
        A = scene.markers(time.monotonic() - t0,
                          el.tolist() if el is not None else None,
                          er.tolist() if er is not None else None,
                          phase, frac,
                          live_metric(task, el, er, cmd,
                                      float(sc.get("length") or SLING_L)),
                          knuck=k)
        # HUD on its own topic: only the FRONT view subscribes to it
        # The HUD needs its OWN DELETEALL. Splitting the array sent the
        # single leading DELETEALL to the objects topic only, so the previous
        # run's title text stayed on screen under the new one -- two headings
        # superimposed and neither readable.
        hud = MarkerArray()
        clr = Marker()
        clr.header.frame_id = "world"
        clr.action = Marker.DELETEALL
        hud.markers.append(clr)
        keep = []
        for m in A.markers:
            (hud.markers if m.ns == "hud" else keep).append(m)
        A.markers = keep
        pub.publish(A)
        hud_pub.publish(hud)
        frames += 1

    first = {a: np.asarray(p[0], float) for a, p in dense.items()}

    if plan is not None and plan.get("ok"):
        # ---------------- REAL GRASP: pregrasp -> approach -> close -> lift
        ga = plan["arm"]
        gq = _q(plan["quat"])
        hold_arm = sc.get("hold_arm")
        hp_ = np.asarray(sc["hold"], float) if hold_arm else None
        if hold_arm:
            sol = dr.solve_joints(hold_arm, list(hp_), q[hold_arm])
            if sol is not None:
                dr.send(hold_arm, sol, 1.6)

        def go(pose, quat, secs, phase, frac, grip):
            sol = dr.solve_joints(ga, list(pose), quat)
            if sol is not None:
                dr.send(ga, sol, secs)
            grip_hold[ga] = grip          # the plan owns this arm's gripper
            for _ in range(max(2, int(secs * fps))):
                dr.spin(1.0 / fps)
                tick(phase, frac, {ga: np.asarray(pose, float)})

        # 1 PRE-GRASP: standoff, gripper OPEN, wrist already aligned
        go(plan["pregrasp"], gq, 2.0, "pregrasp", 0.0, GRIP_OPEN)
        # 2 APPROACH: straight line down the approach vector, still open
        for k, w in enumerate(plan["path"][1:], 1):
            go(w, gq, 0.30, "approach_vec",
               0.05 * k / max(1, len(plan["path"])), GRIP_OPEN)
        # 3 CLOSE to the object's width -- not fully shut
        gw = grip_for(plan["width_mm"])
        grip_hold[ga] = gw
        for _ in range(int(1.4 * fps)):
            dr.spin(1.0 / fps)
            tick("close", 0.12, {ga: np.asarray(plan["grasp"], float)})
        # 4 LIFT back along the approach vector before transiting
        for w in reversed(plan["path"][:-1]):
            go(w, gq, 0.30, "lift", 0.16, gw)
        prev[ga] = np.asarray(plan["pregrasp"], float)
        if hold_arm:
            prev[hold_arm] = hp_
    else:
        for arm, tgt in first.items():
            sol = dr.solve_joints(arm, list(tgt), q[arm])
            if sol is not None:
                dr.send(arm, sol, 1.6)
        for _ in range(int(1.6 * fps)):
            dr.spin(1.0 / fps)
            tick("approach", 0.0, first)
        prev.update(first)

    for i in range(0, nmax, stride):
        cmd = {}
        for arm, path in dense.items():
            tgt = np.asarray(path[min(i, len(path) - 1)], float)
            if smooth and arm in prev:
                tgt = prev[arm] + (tgt - prev[arm]) * (1.0 - smooth)
            prev[arm] = tgt
            cmd[arm] = tgt
            sol = dr.solve_joints(arm, list(tgt), q[arm])
            if sol is not None:
                dr.send(arm, sol, 0.28)
        dr.spin(0.28)
        frac = i / max(1, nmax - 1)
        # PLACE: hand the gripper back so the schedule opens it at the target.
        # Releasing is what makes the block count as placed, so a plan that
        # never let go would also never register an outcome.
        if frac >= RELEASE_FRAC.get(task, 2.0):
            grip_hold.clear()
        tick("task", frac, cmd)

    # release, and let the fingers be SEEN to open before the clip ends
    for _ in range(int(1.8 * fps)):
        dr.spin(1.0 / fps)
        tick("hold", 1.0, prev)

    dur = time.monotonic() - t0
    stop_grabs(procs)
    quad = make_quad(out_dir)
    json.dump(grip_trace, open(os.path.join(out_dir, "grip_trace.json"), "w"))
    ks = [g for g in grip_trace if g.get(active_arm(task, sc)) is not None]
    vals = [g[active_arm(task, sc)] for g in ks]
    pl = run.get("plan") or {}
    return dict(frames=frames, duration_s=dur, quad=quad,
                grasp_planned=bool(pl.get("ok")),
                grasp_refused=pl.get("refused", ""),
                grasp_yaw_deg=pl.get("yaw_deg"),
                grasp_width_mm=pl.get("width_mm"),
                grasp_standoff_m=pl.get("approach_m"),
                blocks_placed=len(scene.placed),
                ball_fallen=bool(scene.ball_fallen),
                tool_delivered=scene.tool_released_at is not None,
                grip_min=min(vals) if vals else None,
                grip_max=max(vals) if vals else None,
                grip_opened=bool(vals and min(vals) < 0.10),
                grip_closed_on_object=bool(vals and any(
                    GRIP_HOLD_MIN <= v < GRIP_FREE_AIR for v in vals)))


def go_home(dr):
    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as hp
    for arm in ("left", "right"):
        dr.send(arm, hp.load_home_radians(arm), 1.2)
    dr.spin(1.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default=None)
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--condition", default=None, choices=CONDITIONS)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--final5", action="store_true",
                    help="record the CURRENT five-task spec, not the stale nine")
    ap.add_argument("--resume", action="store_true",
                    help="skip clips that already have a rviz_capture.json")
    a = ap.parse_args()

    rclpy.init()
    dr = Driver()
    dr.spin(3.0)
    if not dr.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim")
        return 2
    qos = QoSProfile(depth=20)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    pub = dr.create_publisher(MarkerArray, "/task_objects", qos)
    hud_pub = dr.create_publisher(MarkerArray, "/task_hud", qos)
    from trajectory_msgs.msg import JointTrajectory as _JT
    gpub = {a: dr.create_publisher(
        _JT, "/%s_gripper_controller/joint_trajectory" % a, 5)
        for a in ("left", "right")}

    def send_gripper(arm, angle):
        from trajectory_msgs.msg import JointTrajectoryPoint as _JTP
        m = _JT()
        m.joint_names = [KNUCKLE % arm]
        pt = _JTP()
        pt.positions = [float(angle)]
        pt.time_from_start.nanosec = 250_000_000
        m.points = [pt]
        gpub[arm].publish(m)
    dr.send_gripper = send_gripper

    if a.final5:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import final5_runs
        runs = [r for r in final5_runs.build()
                if (not a.task or r["task"] == a.task)
                and (not a.scenario or r["scenario"].startswith(a.scenario))]
    else:
        spec = yaml.safe_load(open(os.path.join(BIM,
                                                "scenarios_verified.yaml")))
        runs = build_runs(spec, a.task, a.scenario)
    conds = [a.condition] if a.condition else list(CONDITIONS)
    print("RVIZ SCREEN CAPTURE -- %d run(s) x %d condition(s)"
          % (len(runs), len(conds)))
    idx = []
    prog_path = os.path.join(OUT, "sweep_progress.json")
    done = set()
    if a.resume and os.path.exists(prog_path):
        try:
            for row in json.load(open(prog_path)).get("done", []):
                done.add(tuple(row))
        except Exception:                                    # noqa: BLE001
            pass
        print("  resuming: %d clip(s) already recorded" % len(done))
    total = sum(len(run.get("conditions", conds)) for run in runs)
    n_done = 0
    for run in runs:
        # A run may restrict its own conditions. f2 is modes 4 and above only,
        # because teleoperated grasping is not achievable on this master, so
        # there is no DIRECT or VR clip to record.
        for cond in run.get("conditions", conds):
            key = (run["task"], run["scenario"], cond)
            if key in done:
                n_done += 1
                print("  [%d/%d] skip (done) %s" % (n_done, total, "/".join(key)))
                continue
            d = os.path.join(OUT, MODE_BUCKET, run["task"],
                             run["scenario"], cond)
            ensure_display(os.path.join(SCRATCH, "capture"),
                           active_arm(run["task"], run["sc"]))
            r = run_one(dr, pub, hud_pub, run, cond, d)
            sz = sum(os.path.getsize(os.path.join(d, "rviz_%s.mp4" % n))
                     for n in VIEWS
                     if os.path.exists(os.path.join(d, "rviz_%s.mp4" % n)))
            r.update(task=run["task"], scenario=run["scenario"],
                     condition=cond, note=run["note"], bytes=sz)
            json.dump(r, open(os.path.join(d, "rviz_capture.json"), "w"),
                      indent=2)
            idx.append(r)
            # PROGRESS AFTER EVERY CLIP, not at the end. A sweep that dies at
            # clip 60 of 87 must not lose 60 clips' worth of work, and the
            # previous sweep did exactly that.
            done.add(key)
            n_done += 1
            json.dump(dict(done=sorted(done), total=total),
                      open(prog_path, "w"), indent=2)
            print("  [%d/%d] %-4s %-22s %-9s %5.1f s  grip %.2f..%.2f %s  quad=%s  %s"
                  % (n_done, total, run["task"], run["scenario"], cond,
                     r["duration_s"],
                     r["grip_min"] if r["grip_min"] is not None else -1,
                     r["grip_max"] if r["grip_max"] is not None else -1,
                     "OPEN+CLOSE" if (r["grip_opened"]
                                      and r["grip_closed_on_object"])
                     else "static",
                     r["quad"],
                     "blocks=%d" % r["blocks_placed"] if run["task"] == "t2"
                     else ("ball_fell" if r["ball_fallen"] else "")))
    json.dump(idx, open(os.path.join(OUT, "rviz_index.json"), "w"), indent=2)
    dr.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
