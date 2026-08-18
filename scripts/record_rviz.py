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
import signal
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src/srl_teleop"))
from srl_teleop import procscan                              # noqa: E402
from srl_teleop import gripper_state as _gs                  # noqa: E402
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
# SCRATCH WAS A HARDCODED PATH FROM ONE SESSION and that session's directory is
# gone, so every render check, config cache and frame grab wrote into nothing.
# It is derived now, and created if absent. Override with SRL_SCRATCH.
SCRATCH = os.environ.get(
    "SRL_SCRATCH",
    os.path.join(os.environ.get("TMPDIR", "/tmp"), "srl_rviz_scratch"))
os.makedirs(SCRATCH, exist_ok=True)
RVIZ_CFG = os.path.join(ROOT, "src/srl_experiments/config/verification_capture.rviz")
# FOUR SIMULTANEOUS VIEWS, one RViz per virtual display, all watching the same
# ROS graph -- so the four recordings are frame-for-frame the same trial, not
# four passes that might diverge. One angle hides things: the front view
# flattens reach depth, the top view is where a coordination error is obvious,
# and only a tight gripper view answers "did it actually grab it".
# FRAMED ON THE WORKSPACE, NOT ON THE WEARER.
#
# Every distance here was 2.10-2.40 m, which fits the whole 1.8 m mannequin in
# frame and leaves the actual task -- a 0.87 x 0.28 m volume around
# (0.15, 0.35, 1.16) -- occupying roughly 5% of the picture. On a 800x500 clip
# that is a few dozen pixels of gripper, and no viewer can tell a grasp from a
# near miss. The distances below frame the workspace with margin for the arms
# above it; the wearer's torso and head stay visible for context but no longer
# set the scale.
#
# The focal point follows the task volume, whose x centre moved when task A's
# bin went outboard to 0.62 (see clip_tasks.A_BIN).
# x -0.10, not +0.15. THIS is what aims the eight capture angles; the
# verification_capture.rviz file aims only the standalone RViz, so editing
# that one changes nothing about a clip. +0.15 was on the LEFT arm's side,
# and every MSc task now spans x -0.54..0.34 with T1 out at -0.32..-0.50, so
# the work sat at the right edge of frame and partly outside it while the
# empty left-arm workspace marking held the middle of the picture.
_FOCUS = (0.15, 0.35, 1.16)

# ONE FRAMING DOES NOT FIT FOUR TASKS, and the clips proved it.
#
# Everything below was read off the contact sheets of the 2026-08-11 mode-06
# recording, not reasoned about:
#
#   T0  the two FRONT_UP targets sit at z = 1.62 and were ABOVE THE FRAME. The
#       clip showed both arms reaching up out of shot toward spheres a viewer
#       could not see, which is the same "arms reaching for nothing" the
#       subject check was added to catch, arriving through the camera instead.
#   T1  the whole workspace -- four cubes and two coloured planes inside
#       0.18 x 0.10 m -- rendered about sixty pixels wide. Four cubes could
#       not be told from two, and which plane a cube landed on was
#       unreadable, so the colour-matching outcome the task exists to score
#       could not be scored from the picture.
#   T2  the carry now runs to z = 1.60 and THE TRAY LEFT THE TOP OF THE FRAME
#       partway through the lift. The last third of the clip is two arms
#       holding something out of shot.
#   T3  the four measurement points are 14 mm spheres and came out sub-pixel.
#
# Every one of those is a consequence of raising the geometry -- T0's
# directions and T2's band -- without re-aiming the camera at it. The FRONT
# view is the one that carries the caption and the one a reader looks at, so
# it follows the task. Every other angle stays global, because the
# wearer-clearance question they answer is the same in every task.
TASK_FOCUS = {
    "t0": ((0.00, 0.35, 1.28), 1.95),
    # x -0.41, not +0.34. THIS is what aims the front view; _FOCUS aims the
    # other seven and verification_capture.rviz aims only the standalone
    # RViz. +0.34 was the LEFT-arm cube row. T1 moved to the right arm and
    # its cubes now sit at x -0.32..-0.50 with the mats at -0.34 and -0.47,
    # so the whole task sat at the right edge of frame while the empty
    # left-arm workspace marking held the middle of the picture.
    # CENTRED ON THE WORK, AND LOOKING DOWN AT IT. Focal moved from -0.30 to
    # -0.38, which is the middle of the cube row (-0.32..-0.50) rather than a
    # compromise between the wearer and the work, and pitch raised from the
    # shared 0.28 to 0.52 rad. Looked at: at 0.28 the camera is nearly
    # edge-on to the surface, the mats read as lines and the cubes sit on top
    # of them, and the work sat right of centre. Distance out to 1.55 so the
    # wearer stays fully in shot from the higher angle.
    # RE-AIMED 2026-08-15, because T1 moved back to the LEFT arm and the work
    # with it: cubes x 0.560..0.740 y 0.120, mats x 0.450 and 0.610 y 0.240.
    # Left at -0.38 the camera pointed at the empty right-hand side and the
    # whole task sat cut off against the left edge of frame -- looked at, on
    # the first clip recorded after the move. Focal moved to +0.45, between
    # the wearer at x = 0 and the work centre at 0.57 and biased toward the
    # work, and the distance out to 1.85 so the wearer is still in shot: the
    # subject of a clip of a WORN arm is the arm AND the person wearing it.
    # Looked at twice. At focal +0.45 / dist 1.85 the work sat about 60 px
    # left of frame centre and the scene filled a little over half the width,
    # with dead sky above. +0.54 / 1.65 centres the cube row and the mats and
    # brings the whole thing up to size without losing the wearer, who is the
    # other half of the subject.
    # RE-AIMED 2026-08-17 FOR THE REBUILT T1, and this time it centres on the
    # WEARER rather than on one arm's work. The task is two-armed now: the
    # blue pad and its cubes sit at x +0.375..+0.685 and the green pad and
    # its cubes at x -0.275..-0.575, straddling the centreline. Focal +0.54
    # was the middle of the OLD single-sided cube row and would have put the
    # whole right-hand half of the task off the left edge of frame.
    #
    # Distance 2.05, not 1.65: the work now spans 1.26 m of x against the old
    # 0.29 m, and the near edge sits at y = 0.280 so the table itself is
    # 180 mm further forward. Focal z drops to 1.02 -- the work is on a 0.95
    # table rather than 120 mm above a 0.98 one -- and the pitch stays at
    # 0.52 rad, which is what stopped the mats reading as lines.
    "t1": ((0.00, 0.30, 1.02), 2.05, 0.52),
    # t1s2 works BOTH sides, so it centres on the wearer.
    "t1s2": ((0.00, 0.24, 1.16), 1.70, 0.52),
    "t2": ((0.00, 0.35, 1.46), 1.55),
    # T3's subject is the BOX's faces and the four pads on them, which are
    # vertical surfaces, so it wants less downward pitch than T1 -- but more
    # than the shared 0.28, because the meter is presented above the box.
    "t3": ((-0.20, 0.19, 1.20), 1.70, 0.40),
    # THE DANCE. Centred on the wearer because both arms work both sides, and
    # pulled BACK to 2.10 because the routines span z 1.02..1.58 -- a 560 mm
    # vertical range against T1's 120 mm. Framed at T1's distance the apex of
    # every reach would leave the top of the picture, which is exactly what
    # happened to T0's two FRONT_UP targets and T2's tray before TASK_FOCUS
    # existed.
    # 1.75, NOT 2.10. Framed at 2.10 the wearer filled about a third of the
    # height and the empty ground plane took the rest -- and the canon, which
    # separates the two arms by 68 mm on average, was a few pixels and read as
    # unison. The focus rises to 1.32 so the apex at z = 1.54 still clears the
    # top of the picture at the closer distance.
    "d1": ((0.00, 0.33, 1.32), 1.75),
    "d2": ((0.00, 0.33, 1.32), 1.75),
    "d3": ((0.00, 0.33, 1.32), 1.75),
}
VIEWS = {
    #  name       display  yaw      pitch  dist  focal(x,y,z)   hud
    "front":   (":91", 1.5708, 0.28, 1.35, _FOCUS, True),
    "back":    (":95", -1.5708, 0.26, 1.35, _FOCUS, False),
    "left":    (":92", 0.0000, 0.18, 1.30, _FOCUS, False),
    "right":   (":96", 3.1416, 0.18, 1.30, _FOCUS, False),
    "iso":     (":97", 0.9000, 0.40, 1.45, _FOCUS, False),
    "top":     (":93", 1.5708, 1.35, 1.40, _FOCUS, False),
    # ONE DISPLAY PER ARM, and neither is ever killed. See ensure_display().
    # 0.60 m, not 0.40. At 0.40 the camera sits INSIDE task A's bin once the
    # hand descends into it, so the place -- half the point of a pick and
    # place -- was a teal wall. 0.60 still renders the fingers large enough to
    # read a grasp while keeping the bin in frame rather than around the lens.
    "gripper": (":94", 1.5708, 0.25, 0.60, (0.0, 0.0, 0.0), False),
    "gripper_right": (":98", 1.5708, 0.25, 0.60, (0.0, 0.0, 0.0), False),
}
# The gripper camera is bolted to a LINK, so it needs one RViz per arm. Which
# display the clip is grabbed from depends on the task's active arm.
GRIP_DISPLAY = {"left": ":94", "right": ":98"}
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
# THE GRIPPER WAS NEVER COMMANDED. mock_components boots the Robotiq driven
# knuckle at 0.793 rad -- fully closed on nothing -- so every clip showed a
# shut hand throughout, and the "grasp" existed only in the marker layer. A
# pick rendered with a closed gripper is not a pick.
#
# EVERY CONSTANT AND BOTH FUNCTIONS BELOW COME FROM srl_teleop.gripper_state.
# They used to be restated here and again in clip_tasks.py -- three copies of
# the same thresholds, in the three places that decide whether a grasp
# happened. A drifted grasp threshold silently reclassifies every trial in a
# study, and the copies were already disagreeing: this file called 0.05
# "GRIP_OPEN" while gripper_state calls 0.10 "OPEN_RAD", one a command and
# one a classification bound.
GRIP_OPEN = _gs.CMD_OPEN_RAD
GRIP_HOLD_MIN, GRIP_FREE_AIR = _gs.OPEN_RAD, _gs.FREE_AIR_RAD
grip_for = _gs.grip_for
holding = _gs.holding

# Fraction of the transit at which the plan hands the gripper back to the
# schedule so it can open and PLACE. These match the thresholds already inside
# grip_schedule(), so the release stays defined in exactly one place; the hold
# above only stops the object being dropped BEFORE it.
RELEASE_FRAC = {"t2": 0.80, "t5": 0.86}


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


def write_cfg(name, arm="left", task=None):
    """One RViz config per view. The HUD lives on its own topic so only the
    FRONT view shows it -- four copies of the same text is clutter."""
    yaw, pitch, dist, focal, hud = VIEWS[name][1:]
    # THE FRONT VIEW FOLLOWS THE TASK. See TASK_FOCUS.
    if name == "front" and task in TASK_FOCUS:
        # PITCH IS OPTIONAL AND PER TASK. It used to be shared by every task
        # at 0.28 rad, and for the pick-and-place tasks that is nearly
        # edge-on to the work surface: LOOKED AT, T1's cubes and mats
        # foreshortened into a thin band in which a 40 mm cube and a 140 mm
        # mat are hard to tell apart. A task whose subject lies FLAT on a
        # surface needs to be looked down at; one whose subject is in free
        # space does not.
        _f = TASK_FOCUS[task]
        focal, dist = _f[0], _f[1]
        if len(_f) > 2:
            pitch = _f[2]
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
    # PER-TASK FILENAME for the front view, for the same reason the gripper
    # view is per-arm: the running-instance check keys on the config PATH, so
    # one shared front.rviz would match an RViz already framed for a different
    # task and the re-aim would silently never happen.
    fn = (("gripper_%s" % arm) if name == "gripper" else
          ("front_%s" % task) if (name == "front" and task in TASK_FOCUS)
          else name)
    p = os.path.join(CFG_DIR, "%s.rviz" % fn)
    open(p, "w").write(txt)
    return p


def _display_renders(disp, thresh=8.0):
    """Does this X display actually contain a picture?

    Mean grey of one grabbed frame. The threshold is far below any real RViz
    frame (measured 62-101) and far above an empty root window (0.04).
    """
    f = os.path.join(SCRATCH, "renderchk_%s.png" % disp.lstrip(":"))
    r = subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
                        "-video_size", "%dx%d" % (VW, VH),
                        "-i", "%s.0" % disp, "-frames:v", "1", f],
                       capture_output=True)
    if r.returncode != 0 or not os.path.exists(f):
        return False
    try:
        import numpy as _np
        from PIL import Image as _Image
        return float(_np.asarray(_Image.open(f).convert("L")).mean()) > thresh
    except Exception:                                          # noqa: BLE001
        return False


def _re_escape(pat):
    import re as _re
    return _re.escape(pat)


def _running(pat):
    """Is a process matching `pat` alive, EXCLUDING this one and its shell?

    Delegates to srl_teleop.procscan. The previous `ps | any(pat in line)`
    had the same defect as `pgrep -f`: this process's own command line is in
    that listing, so a pattern the caller mentions matches the caller. That
    trap has cost this project four separate failures, most recently Xvfb
    never being started because the check believed it already was.
    """
    import re as _re
    return procscan.count(_re.escape(pat)) > 0


_GRIP_ARM = {"cur": None}


def ensure_display(log, gripper_arm="left", task=None):
    """One Xvfb per view, each with its own RViz. Idempotent, and NOTHING IS
    EVER KILLED.

    THE GRIPPER CAMERA IS BOLTED TO A LINK (Target Frame =
    <arm>_end_effector_link), so it needs a different config per arm. The
    previous version kept ONE display for it and, when the active arm changed,
    pkill'd the instance bound to the other arm and started a replacement.

    That produced SEVEN ENTIRELY BLACK CLIPS -- 34 s each, mean brightness 1.0
    of 255 -- and the black ones were exactly the clips where no replacement
    RViz was started. Reproduced directly by alternating the arm four times
    and grabbing the display each time:

        step 0  arm=left    brightness 99.00   left_rviz=True  right_rviz=False
        step 1  arm=right   brightness 98.92   left_rviz=True  right_rviz=True
        step 2  arm=left    brightness  0.04   left_rviz=True  right_rviz=True
        step 3  arm=right   brightness  0.04   left_rviz=True  right_rviz=True

    The pkill does not take: at step 1 BOTH instances are alive on the same
    display, so two RViz windows fight over it, and from step 2 the display
    renders nothing at all. The sweep runs a(left), b(right), c(left) per
    mode, so it crossed that transition twice in every mode.

    A kill-and-restart on a shared display is the whole bug. Each arm now owns
    its own display and its own RViz, both started once and left alone, so
    there is no transition to get wrong.
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
        # Each gripper view is permanently bound to its own arm and display.
        arm = ("right" if name == "gripper_right" else
               "left" if name == "gripper" else gripper_arm)
        cfg = write_cfg("gripper" if name.startswith("gripper") else name,
                        arm, task)
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

    # A PROCESS BEING ALIVE IS NOT EVIDENCE IT IS RENDERING.
    #
    # This is what actually produced the black clips, and it survived the
    # per-arm-display fix until it was measured. `pkill -f` sends SIGTERM;
    # RViz catches it, tears its render window down, and LINGERS. The old
    # instance then still matched `_running("rviz2 -d <cfg>")`, so the check
    # said "already running", no replacement was started, and the display
    # stayed black for the rest of the session. Found by listing processes
    # against display brightness: gripper_left alive, Xvfb :94 alive, frame
    # mean 0.04 of 255.
    #
    # The only trustworthy liveness test for a renderer is a PIXEL. Grab one;
    # if the display is dark, kill that instance BY PID with SIGKILL -- not by
    # name, and not politely -- and start a fresh one. Refuse rather than
    # record a black view: a black clip that reaches the tree is indis-
    # tinguishable from a clip of a stationary arm.
    for name, (disp, *_rest) in VIEWS.items():
        arm = ("right" if name == "gripper_right" else
               "left" if name == "gripper" else gripper_arm)
        cfg = write_cfg("gripper" if name.startswith("gripper") else name,
                        arm, task)
        for attempt in range(3):
            if _display_renders(disp):
                break
            for pid, _c in procscan.find(_re_escape("rviz2 -d %s" % cfg)):
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
            time.sleep(2)
            env = dict(os.environ, DISPLAY=disp, LIBGL_ALWAYS_SOFTWARE="1",
                       GALLIUM_DRIVER="llvmpipe", QT_QPA_PLATFORM="xcb")
            subprocess.Popen(["rviz2", "-d", cfg], env=env,
                             stdout=open(log + ".rviz." + name, "a"),
                             stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(22)
        else:
            raise RuntimeError(
                "%s (%s) will not render after 3 restarts -- refusing to "
                "record a black view" % (name, disp))


def start_grabs(out_dir, gripper_arm="left"):
    """One ffmpeg per view, started together so the four are synchronised.

    `gripper_arm` selects WHICH gripper display is grabbed. Both exist and
    both render continuously; only the active arm's is written, and it is
    written as rviz_gripper.mp4 so the output tree is unchanged.
    """
    procs = {}
    want_grip = "gripper_right" if gripper_arm == "right" else "gripper"
    for name, (disp, *_rest) in VIEWS.items():
        if name.startswith("gripper") and name != want_grip:
            continue
        out_name = "gripper" if name.startswith("gripper") else name
        path = os.path.join(out_dir, "rviz_%s.mp4" % out_name)
        procs[out_name] = subprocess.Popen(
            [FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
             # 15 fps, SET FROM A MEASUREMENT, not chosen.
             # `scripts/measure_render_rate.py` drives a real robot through
             # RViz on this display and counts frames that DIFFER from their
             # predecessor: llvmpipe delivers 16.0 fps at 800x500 and the WSL
             # d3d12 GPU path delivers 16.3, i.e. the GL driver is not the
             # limit and does not need changing. Asking for more than the
             # source can render does not make a smoother video -- x11grab
             # simply grabs the same pixels again, which is why every clip on
             # disk reported its requested 12 fps while delivering 0.6-1.3
             # distinct fps. 15 sits just under the measured ceiling.
             "-video_size", "%dx%d" % (VW, VH), "-framerate", "15",
             "-i", "%s.0" % disp, "-c:v", "libx264", "-preset", "ultrafast",
             "-pix_fmt", "yuv420p", path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
    return procs


def stop_grabs(procs):
    # TOLERATE None. run_one returns grabs=None on every FAILURE path, and
    # this then raised AttributeError inside the sweep's teardown -- which
    # MASKED the actual reason the clip failed, because the reason is logged
    # after the teardown. A cleanup that crashes on the failure path destroys
    # the evidence for the failure it is cleaning up after.
    if not procs:
        return {}
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


# ---------------------------------------------------------------- teardown
def teardown_displays(verbose=True):
    """Kill every Xvfb and RViz this module started, and clear their locks.

    WHY THIS IS SEPARATE FROM ensure_display's "NOTHING IS EVER KILLED".
    That rule is about the middle of a sweep: killing and restarting an Xvfb
    that two RViz instances share is what produced seven entirely black clips.
    It says nothing about the END of a sweep, and until now nothing cleaned up
    there -- so a run that failed left one Xvfb per view alive, holding its
    display number.

    THE LEAK IS SELF-WORSENING, which is why it gets a real teardown rather
    than a note. Xvfb refuses a display whose /tmp/.X<n>-lock exists, so eight
    orphans per mode across five modes is forty held display numbers, and the
    failure that produces is a recording that renders BLACK on a display that
    was never created. Measured after one failed mode: 8 orphaned servers and
    8 stale locks.

    Kills BY PID. `pkill -f` has killed the shell running it three times in
    this project, because the pattern matches that shell's own command line.
    """
    killed, locks = [], []
    for name, (disp, *_rest) in VIEWS.items():
        for pat in ("Xvfb %s" % disp, "rviz2.*%s" % disp.lstrip(":")):
            try:
                out = subprocess.run(["pgrep", "-f", pat], capture_output=True,
                                     text=True).stdout.split()
            except OSError:
                out = []
            for pid in out:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                    killed.append((name, disp, int(pid)))
                except (OSError, ValueError):
                    pass
    time.sleep(1.0)
    for name, (disp, *_rest) in VIEWS.items():
        n = disp.lstrip(":")
        for path in ("/tmp/.X%s-lock" % n, "/tmp/.X11-unix/X%s" % n):
            if os.path.exists(path):
                try:
                    os.unlink(path)
                    locks.append(path)
                except OSError:
                    pass
    if verbose:
        print("[rviz] teardown: killed %d process(es), removed %d lock(s)"
              % (len(killed), len(locks)))
    return killed, locks


def stale_displays():
    """Anything of ours still alive or still holding a lock, as (what, detail).

    A PRECONDITION, not a cleanup: a sweep that starts on top of forty dead
    servers renders black and passes, so it must refuse and NAME them rather
    than quietly reuse whatever is there.
    """
    bad = []
    for name, (disp, *_rest) in VIEWS.items():
        n = disp.lstrip(":")
        if _running("Xvfb %s" % disp):
            bad.append(("live Xvfb", "%s (%s)" % (disp, name)))
        if os.path.exists("/tmp/.X%s-lock" % n):
            bad.append(("stale lock", "/tmp/.X%s-lock" % n))
    return bad
