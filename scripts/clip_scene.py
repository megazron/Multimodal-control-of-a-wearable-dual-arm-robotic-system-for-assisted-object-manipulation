#!/usr/bin/env python3
"""The task scene for the clip sweep: real objects, attached on a real grasp.

WHY THIS EXISTS. The first sweep drove each mode's command path and recorded
the arms moving, but published no objects at all -- so fifteen clips evidenced
that a mode commands the robot and nothing about anything being picked up, and
task C had no multimeter in it. The pixel verification could only ever check
"did the arm move".

WHAT MAKES AN ATTACHMENT HONEST. The object follows the gripper only while the
fingers are actually closed ON IT:

    holding(knuckle, width_mm)  ==  knuckle >= 0.90 * grip_for(width_mm)

Not "the gripper is somewhere in the holding band". That band alone attaches a
40 mm block the instant the knuckle passes 0.10 rad, while the fingers are
still visibly open -- the object jumps to the hand before it is touched, which
is a picture of a grasp rather than a grasp. The 0.90 absorbs the mock's
first-order tracking lag without accepting a gripper that has barely moved.
The same rule and the same constants as `record_rviz.holding()`, imported so
the two cannot drift.

DETACH IS A RELEASE, NOT A TIMEOUT. The object is dropped the moment the
fingers open past the object's width, and it stays where it was left rather
than snapping back to its start -- otherwise a clip of a successful place
looks identical to a clip of a failed one.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
import record_rviz as rr                                     # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import tasks as TSK                                          # noqa: E402

CT_BALL_R = TSK.BALL_R               # 0.020, the T2 spec's own ball radius
Y = CT.Y
SEP = CT.SEP
KN = "%s_robotiq_85_left_knuckle_joint"

# How close the finger PADS must be to an object for a closure to count as
# grasping THAT object. 0.03 m: the binding capture half-window in the set is
# 17.5 mm (the 50 mm multimeter) and the block's is 22.5 mm, so 30 mm is
# outside every real capture window while still tolerating the few millimetres
# of tracking error between the commanded pose and tf2.
GRASP_NEAR_M = 0.03

# ORIENTATION MUST MATCH, NOT JUST POSITION.
#
# Attachment tested DISTANCE alone, so an object sitting at an angle was
# grasped as though it were square and the sim reported a clean grasp. On a
# 40 mm cube a 30 deg error is about 20 mm of misalignment across the face --
# in reality the fingers catch a corner or push the cube away. The sim could
# not show it because nothing anywhere compared the gripper's closing axis to
# the object's.
#
# A CUBE IS SYMMETRIC UNDER 90 DEG, so the error is taken modulo 90: a gripper
# at 90 deg to a square cube grasps it perfectly well, and a check that called
# that a failure would be wrong in the other direction.
GRASP_YAW_TOL_DEG = 20.0


def yaw_error_deg(axis, obj_yaw_deg, symmetry_deg=90.0):
    """Misalignment between the gripper's closing axis and the object, degrees.

    `axis` is any vector along the line joining the two finger pads; its SIGN
    does not matter, which is why the result is folded into [0, symmetry/2].
    Returns None if the axis is degenerate (pads coincident), because a
    missing measurement must not read as a good one.
    """
    import math as _m
    if axis is None:
        return None
    ax, ay = float(axis[0]), float(axis[1])
    if _m.hypot(ax, ay) < 1e-6:
        return None                      # axis is vertical: no yaw to compare
    a = _m.degrees(_m.atan2(ay, ax))
    d = (a - float(obj_yaw_deg)) % symmetry_deg
    return min(d, symmetry_deg - d)


# rgba, measured RENDERED colours are what the verifier keys on, so these are
# chosen to sit inside its existing detector bands rather than near them.
TAN = (0.78, 0.66, 0.42, 1.0)        # tray  -> _tan
ORANGE = (1.00, 0.45, 0.02, 1.0)     # block -> _orange
TEAL = (0.05, 0.75, 0.70, 1.0)       # container -> _teal
GREEN = (0.10, 0.90, 0.20, 1.0)      # circuit box -> _green
YELLOW = (0.95, 0.75, 0.10, 1.0)     # multimeter body -> _yellow
# T1's colour-matched pair needs a BLUE. The verifier's detectors key on
# channel RELATIONS, not absolute RGB, because RViz shades every surface --
# a requested colour renders nothing like itself. This one is chosen to sit
# inside the existing blue band rather than near its edge.
BLUE = (0.10, 0.30, 0.90, 1.0)       # T1 cube/plane -> _blue
GREY = (0.32, 0.34, 0.36, 1.0)
DARK = (0.18, 0.19, 0.21, 1.0)

# T1's TWO COLOURED PLANES -- the thing a cube is matched TO, and the reason
# the task has a scoreable wrong-colour outcome at all. They were specified
# and never drawn: every T1 clip so far shows four cubes and nowhere to put
# them, so "placed on the plane of its own colour" had no referent on screen
# and a wrong-colour placement was not distinguishable from a right one.
#
# A MAT, NOT A BLOCK. A colour-matched target is a marked area on the work
# surface, so it is drawn 4 mm thick with its TOP at the bench-top plane --
# the same plane a cube's base sits on. It is deliberately NOT a collision
# object: a printed target is not something the gripper must avoid, and
# making it one would refuse the very placement the task is about. What IS a
# collision object is the LIP that holds it out over the bench edge.
PLANE_W, PLANE_D, PLANE_T = 0.14, 0.10, 0.004

# THE TABLE UNDER THE WORK SURFACE. Top at 0.95 and reaching forward to
# y = 0.10, which is under the whole measured reachable region (y 0.05..0.20)
# and 150 mm below the work plane -- clear of the approach cone, measured at
# 0 waypoint failures. See furniture_boxes() for the sweep.
# THE HEIGHT NOW HAS ONE OWNER. srl_experiments.work_surface holds the
# declared value and, once something measures the real surface from depth,
# the measured one. This module reads it rather than defining it, so a
# measured surface reaches the scene instead of sitting beside it.
try:
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.dirname(
        _os.path.abspath(__file__))), "src", "srl_experiments"))
    from srl_experiments.work_surface import table_top as _table_top
    TABLE_TOP = _table_top()
except Exception as _e:                             # pragma: no cover
    # LOUD, not silent. Falling back to a hardcoded height without saying so
    # is the bug this module was created to remove -- it is how a measured
    # surface can exist and reach nothing.
    import warnings as _w
    _w.warn("work_surface unavailable (%s); falling back to a HARDCODED "
            "table height. A measured surface will NOT reach the scene."
            % _e, RuntimeWarning)
    TABLE_TOP = 0.95
TABLE_THICK = 0.035
# 0.10, NOT 0.02, AND THE 80 mm MATTERS. The edge sweep measured z = 0.95 with
# the edge at y = 0.100 at ZERO failures; built at 0.02 -- further forward than
# anything measured -- T1 went to 4 waypoint failures on the items and 7 on the
# clip path while T0, T2 and T3 stayed at 0. Extrapolating past the last
# measured point is how a verified layout stops being verified.
TABLE_NEAR_Y = 0.10
TABLE_FAR_Y = 0.72
# 1.05, NOT 0.90. The measured clearance-safe region runs to |x| = 1.000 --
# the old |x| <= 0.70 was where the SURVEY BOX stopped, not where the arm
# does -- and a marking drawn out to 1.000 over a table that ends at 0.90
# would be tiles floating in air. The table carries no verified coordinate
# (it is 170 mm below the work plane and nothing rests on it), so widening it
# is a scenery change; it is re-verified with the layout regardless.
TABLE_HALF_X = 1.05
TABLE_LEG = 0.055                # square section, at the four corners
TABLE_LEG_INSET = 0.10           # how far in from the corner each leg sits.
                                 # The top then OVERHANGS, which is most of
                                 # what reads as a table top at a glance; at
                                 # the old inset of one leg-width there was no
                                 # overhang at all and it read as a box.
TABLE_APRON = 0.06               # skirt depth under the top, so it reads as
                                 # a table rather than a floating slab
# THE TABLE IS WHITE. Specified in TASK_SPEC section 2, T1-8.
#
# WHITE IS SAFER FOR THE VERIFIER THAN OAK WAS, and that is worth stating
# because a colour change near a colour-keyed check is normally a risk. Every
# detector in verify_rviz_clips keys on channel DIFFERENCES -- _tan needs
# R-B > 45, _yellow R-B > 65, _green G-R > 45, _teal G-R > 40 -- and a
# neutral has R = G = B, so it cannot fire any of them. Oak (0.68, 0.52,
# 0.33) sat inside the _tan band and could leak into it; white cannot.
#
# ==========================================================================
# THE NUMBERS BELOW ARE SET FROM THE RENDERER'S MEASURED RESPONSE, NOT FROM
# WHAT "WHITE" MEANS ON PAPER, and the two are a long way apart.
# ==========================================================================
# The shipped table asked for (0.94, 0.94, 0.95) and TASK_SPEC calls it
# white. Sampled out of the 2026-08-15 T1 clip, the top renders at
# RGB(118,118,118) -- mid grey -- while the arm's own meshes in the same
# frame reach (208,207,210). Nothing here noticed, because the only check on
# T1-8 (audit_task_spec) reads CS.OAK and asks whether the REQUESTED colour
# is neutral and >= 0.80. It never looks at a pixel. That is CLAUDE.md's own
# "matched requested RGB, not RENDERED colour" row, arrived at from the
# authoring side instead of the verifying one.
#
# Measured directly (scripts/probe_marker_shading.py, one slab where the
# table is, one frame per colour off the capture display, all three controls
# correct):
#
#     requested        UP-FACING face      CAMERA-FACING face
#     0.00                  0                    0
#     0.94                119                  208
#     1.00                127                  221
#     1.50                190                  255
#     2.00                253                  255
#
# TWO THINGS COME OUT OF THAT AND BOTH ARE NEEDED.
#
# 1. A MARKER'S MATERIAL TAKES AMBIENT = 0.5 x COLOUR, and the only light in
#    the scene is a headlight on the camera. A face pointing AT the camera
#    gets ambient plus diffuse and renders 0.87 x colour; a face pointing UP
#    is grazed by that light and gets the ambient term alone, 0.5 x colour.
#    The table top is an up-facing face. That is why a table asking for 0.94
#    rendered 118 in the shipped clips: not the colour, the renderer.
#
# 2. RVIZ DOES NOT CLAMP THE COLOUR BEFORE IT REACHES THE MATERIAL.
#    std_msgs/ColorRGBA is float32 with no stated upper bound, and the last
#    two rows are the test: 1.5 and 2.0 render 190 and 253, exactly the
#    0.5 x colour line continued past 1.0. So the ambient term CAN be driven
#    to full and a white table top IS available -- it just cannot be written
#    down as "white".
#
# The values below are therefore chosen as RENDERED targets and divided back
# through the measured coefficients:
#
#     part    face seen     want   coefficient   requested
#     top     up-facing      240      127.5         1.88
#     apron   camera-facing  199      221           0.90
#     riser   camera-facing  199      221           0.90
#     legs    camera-facing  170      221           0.77
#
# IF ANOTHER RENDERER DOES CLAMP AT 1.0 this degrades to exactly the old
# behaviour -- a 127 top -- rather than to anything broken, so the trick is
# safe to depend on and its failure mode is the status quo.
#
# The top's OWN front face is camera-facing and saturates at 255. That is
# wanted: it is a 35 mm bright line along the near edge, which is the edge
# every front view looks straight at, and it comes free instead of needing a
# separate rail coplanar with it.
OAK = (1.88, 1.88, 1.89, 1.0)        # top, up-facing -> ~240 (near white);
                                     # its own front face -> 255
RISER = (0.90, 0.90, 0.91, 1.0)      # posts under the raised surface -> ~199
APRON = (0.90, 0.90, 0.91, 1.0)      # skirt under the top -> ~199
LEG = (0.77, 0.77, 0.78, 1.0)        # the four legs -> ~170

# ==========================================================================
# WORKSPACE MARKINGS -- the boundary each arm can actually reach, MEASURED
# ==========================================================================
# scripts/survey_work_surface.py, a 29 x 11 grid on the work plane at
# z = 1.12, pinned wrist, T1's scene, N=3, controls correct:
#
#     left    35 of 319 cells    x  0.30..0.70   y 0.05..0.20
#     right   32 of 319 cells    x -0.70..-0.30  y 0.05..0.20
#     BOTH arms                  0 cells
#     |x| <= 0.10 (front centre) 0 cells
#
# Drawn so a viewer and a participant can see where the robot can and cannot
# go. It is the measurement, not a drawn guess -- a marking that is not the
# measured boundary is decoration that lies, and this rig's whole difficulty
# is that the reachable region is nothing like the region people expect.
#
# TWO HONEST LIMITS ON THESE NUMBERS. The survey box was x -0.70..0.70 and
# y 0.05..0.55, and the region touches x = 0.70 and y = 0.05 on both arms, so
# it is TRUNCATED BY THE BOX and the true extent is at least this and may be
# larger outboard and nearer. And every cell was tested at the object-centre
# height with the grasp pose alone unless --full-path was given.
# ==========================================================================
# READ FROM THE SURVEY, NOT WRITTEN DOWN HERE. This was a hardcoded box and
# the box went stale: it was surveyed WITH THE BENCH IN THE SCENE, giving
# y 0.05..0.20, and the bench has since been deleted. T1's coloured planes
# sit at y 0.27..0.33, so the marking a participant would be told to work
# inside EXCLUDED BOTH OF THE TASK'S TARGETS by up to 130 mm.
#
# Re-surveyed 2026-08-13 against the current one-table scene, step 0.025,
# N=3, full path, both sides, all three controls correct -- 14751 IK calls:
#
#     left    181 cells   x  0.25..0.70   y 0.05..0.30
#     right   228 cells   x -0.70..-0.15  y 0.05..0.45
#
# THE REGION IS NOT A RECTANGLE. At 50 mm resolution the right arm's cells
# filled only 62% of their own bounding box, so a drawn box would claim a
# third of it falsely -- decoration that lies, which is exactly what the
# marking exists not to be. The cells are therefore drawn as CELLS.
# ==========================================================================
REGION_FILE = os.path.join(WS, "recordings", "baselines",
                           "work_surface_region.json")


def _load_region():
    """(cells_by_arm, step, bbox_by_arm). RAISES if the survey is missing.

    No fallback box. A fallback would be a guess wearing the survey's name,
    and the whole reason this is a file rather than a constant is that the
    constant drifted from the thing it claimed to describe and nobody could
    see it happen.
    """
    import json as _json
    if not os.path.exists(REGION_FILE):
        raise RuntimeError(
            "no surveyed region at %s -- run "
            "scripts/survey_work_surface.py --full-path. The workspace "
            "marking is the MEASURED boundary or it is not drawn."
            % REGION_FILE)
    d = _json.load(open(REGION_FILE))
    if not d.get("full_path"):
        raise RuntimeError(
            "%s was surveyed at the GRASP POSE ONLY. A cell that can be "
            "reached is not a cell that can be WORKED." % REGION_FILE)
    # THE MARKING IS THE CLEARANCE-SAFE REGION, NOT THE IK REGION.
    #
    # A participant is told to work inside this. The IK region includes cells
    # where the arm is measurably inside the 150 mm wearer clearance floor --
    # 72 of the left arm's 265, the worst of them with the tube 2.7 mm INSIDE
    # the person -- because collision-aware IK cannot see the wearer pairs
    # the SRDF excludes. A boundary drawn on the floor of a room, that a
    # person is instructed to work inside, must not include the places where
    # the machine hits them.
    if not d.get("clearance_floor_m"):
        raise RuntimeError(
            "%s carries no clearance_floor_m -- its cells were chosen by IK "
            "alone, and a marking drawn from them tells a participant to "
            "work in cells that breach HARD CONSTRAINT 11. Re-run "
            "scripts/measure_clearance_region.py, then "
            "scripts/merge_work_surface_region.py." % REGION_FILE)
    cells = {a: [tuple(c) for c in d["clear_cells"].get(a, [])]
             for a in ("left", "right")}
    box = {}
    for a, c in cells.items():
        if not c:
            raise RuntimeError("the survey has no cell for the %s arm" % a)
        box[a] = dict(x=(min(p[0] for p in c), max(p[0] for p in c)),
                      y=(min(p[1] for p in c), max(p[1] for p in c)))
    return cells, float(d["step"]), box


REGION_CELLS, REGION_STEP, WORKSPACE = _load_region()


def marked_arms(task):
    """Which arms' reachable regions this task should draw, and why.

    T0 draws none: it is reaching in free space with no work surface at all,
    so a surface marking would describe a table that is not in the scene.

    A ONE-ARM TASK DRAWS ONE MARKING. T1 runs on msc_clip_tasks.T1_ARM and
    used to draw both, which put a large empty rectangle on the idle side --
    in the middle of the front view, while the actual work sat at the edge of
    the frame. Two arms, two markings, only where two arms work.
    """
    import msc_clip_tasks as _M
    if task == "t1":
        return (_M.T1_ARM,)
    if task in ("t1s2", "t2", "t3"):
        return ("left", "right")
    return ()


def in_region(arm, x, y):
    """Is (x, y) inside a cell that was MEASURED reachable for this arm?

    Cell membership, not the bounding box. The two differ by a third of the
    right arm's box.
    """
    h = REGION_STEP / 2.0 + 1e-9
    return any(abs(x - cx) <= h and abs(y - cy) <= h
               for cx, cy in REGION_CELLS[arm])
MARK_T = 0.003                   # a painted line, not a kerb
MARK_W = 0.012
MARK_RGBA = {"left": (0.95, 0.75, 0.10, 0.85),
             "right": (0.20, 0.75, 0.95, 0.85)}

RED = (0.90, 0.15, 0.12, 1.0)
# task0 names its spheres by COLOUR ("go to the green one"), so the picture
# has to use those colours and not one colour per arm.
SPHERE_RGBA = {"red": RED, "green": GREEN, "blue": BLUE, "yellow": YELLOW}


def _m(ns, i, typ, xyz, scale, col, frame="world", quat=None):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    if quat is None:
        m.pose.orientation.w = 1.0
    else:
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = col
    return m


# ---------------------------------------------------------------- SUPPORTS
# EVERY OBJECT RESTS ON SOMETHING, AND THE SOMETHING COMES FROM BEHIND.
#
# The objects used to hang in mid air. `on_bench()` is a misnomer: it places
# an object at BENCH_NEAR_Y + depth/2 - OVERHANG with OVERHANG = 0.08, which
# for anything shallower than 160 mm puts the WHOLE object in front of the
# bench edge. Measured on the shipped layout: T1's cubes sit at y = 0.170 and
# 0.230 against a bench edge at 0.245, so three of the four have nothing
# whatever beneath them.
#
# THE SUPPORT CANNOT SIMPLY GO UNDERNEATH, and that is measured, not a
# preference. The pinned tool axis is (-0.153, +0.846, +0.511) -- 30.7 deg
# ABOVE horizontal -- so the hand enters from the NEAR side and from BELOW and
# closes underneath the object. Anything filling that cone converts an
# unsupported object into an unreachable one, which is what the rail sweep
# found (FIXED cells 0 at every continuous-rail position, 2026-08-11).
#
# So each object gets a CANTILEVERED LIP of its own: a thin slab whose TOP is
# exactly the object's base plane, as wide as the object plus a margin,
# running from `SUPPORT_UNDER_FRAC` of the way through the object's own
# footprint BACKWARD to the bench. Local in x, so it is not the continuous
# rail that failed; and its front edge is behind the point where the approach
# axis crosses the object's base plane, which for a 40 mm cube is 33 mm in
# front of the object centre.
SUPPORT_T = 0.012
# Fraction of the object's depth that has lip beneath it. 0.75 leaves the
# centre of mass 25% of the depth behind the lip edge -- resting, not
# balancing -- while keeping the lip edge out of the approach cone.
SUPPORT_UNDER_FRAC = 0.75
SUPPORT_MARGIN_X = 0.02

# ==========================================================================
# SUPPORTS ARE OFF, AND THE REASON IS A MEASUREMENT, NOT A PREFERENCE
# ==========================================================================
# The lips above were built, applied and measured. They break the task.
#
# `scripts/sweep_t1_supports.py`, T1's six pick paths densified at 20 mm,
# N=5, both controls correct (bench-only scores 0, a slab across the approach
# scores 61):
#
#     bench only, no support (the shipped, verified layout)      0 failures
#     cube lips, 75% / 50% / 25% of the depth supported     22 / 20 / 16
#     plane lips, 75% / 50% / 25%                           52 / 46 / 32
#     cube + plane lips                                     52 / 46 / 32
#     pads under the object's OWN FOOTPRINT only                26
#     side ledges on posts, dx = 0.06 / 0.09                65 / 65
#
# Read the last two rows together and they settle it. The footprint-only pad
# does not reach back to the bench at all, so it can only be obstructing
# DIRECTLY BENEATH THE OBJECT -- which is where the fingers close. And moving
# the support out to the sides is worse, because the posts then stand in the
# gripper's own corridor. There is no direction left: the approach cone
# occupies the front, the underside, and now measurably the sides too.
#
# This is the same wall the 2026-08-11 rail sweep hit from the other side
# (FIXED cells 0 at every continuous-rail position) and the same geometry
# d44dbb8 states in one line: the pinned tool axis is 30.7 deg above
# horizontal, the hand enters from the near side and from below, and the
# instinctive fix -- put something under it -- fills exactly the volume the
# fingers need.
#
# So the objects stay FIXTURED, which is what "option 4" already decided and
# what every T1 clip caption already says. What has changed is that it is now
# a measured impossibility with a table behind it rather than a choice nobody
# had re-examined.
#
# WHAT WOULD ACTUALLY FIX IT, named so it is not lost: raise the objects onto
# stands well clear of the bench top, so the approach cone lies in free air
# ABOVE the bench instead of in the 60 mm strip in front of its edge. That
# moves T1_Z, so it is a re-derivation of the whole layout against
# verify_t1_layout.py -- a task-position change with its own verification
# pass, not a scene edit. Flip SUPPORTS_ENABLED and re-run the sweep to
# re-measure any candidate geometry.
SUPPORTS_ENABLED = False


def _lip(name, obj_xyz, obj_size, bench_y, top_z=None):
    """A cantilevered lip under the REAR of one object, back to the bench."""
    d = obj_size[1]
    front = obj_xyz[1] + d / 2.0 - SUPPORT_UNDER_FRAC * d
    back = max(bench_y + 0.03, front + 0.02)
    top = (obj_xyz[2] - obj_size[2] / 2.0) if top_z is None else top_z
    return (name,
            [obj_xyz[0], round((front + back) / 2.0, 4),
             round(top - SUPPORT_T / 2.0, 4)],
            [round(obj_size[0] + SUPPORT_MARGIN_X, 4),
             round(back - front, 4), SUPPORT_T],
            TAN)


def furniture_boxes(task):
    """(name, xyz, size, colour) for every SOLID in THIS task's scene.

    PER TASK, and that is the whole point of the argument.

    T0 IS REACHING ONLY -- no objects, no grasp, nothing to rest on a surface
    -- so it gets NO FURNITURE AT ALL. It used to be handed the bench, the bin
    and a circuit box, none of which it uses, and with them the bench's own
    limits: T0's target band was derived as the largest rectangle both arms
    can work WITH THE BENCH IN THE SCENE, 200 x 80 mm, which is why its three
    targets were a hand's breadth apart. Free space is a much larger volume
    and it is the volume this task actually runs in.

    The legacy A/B/C set keeps the furniture it was verified against, exactly:
    bench, bin, and the circuit box at x = -0.35. The MSc four do not get the
    bin (nothing places into it) and T3 does not get the legacy circuit box --
    it carries its OWN, 190 mm further outboard, as a graspable item, and
    drawing both put two circuit boxes in one picture.
    """
    import clip_tasks as _ct
    import msc_clip_tasks as _mct
    import task3 as _t3
    out = []
    # T0 AND THE DANCE ARE FREE-SPACE. No work surface, so no furniture: a
    # table drawn under a routine that never touches one is scenery that
    # invites a viewer to read the motion as reaching for something.
    #
    # The dance was REFUSED by this module's own argument list until now
    # ("d1" was not in `choices`), so every dance clip was filmed with no
    # scene node running at all -- no furniture, no markers, and no
    # scene_events.json to say how far the arms travelled.
    if task == "t0" or task in ("d1", "d2", "d3"):
        return out
    # A REAL TABLE, IN TWO LEVELS, AND THE SPLIT IS MEASURED.
    #
    # "Objects fall outside the table." They do, and the table is what is
    # wrong: scripts/survey_work_surface.py finds EVERY reachable cell on the
    # work plane at y = 0.05..0.20 while the slab's near edge sat at 0.245, so
    # the entire usable region was in front of the furniture.
    #
    # The edge cannot simply come forward. At the objects' own height the
    # approach cone is in the way, and scripts/sweep_table_edge.py prices it
    # exactly -- T1's six pick paths, N=5, both controls correct:
    #
    #     near edge y = 0.245 (shipped)   0 failures
    #                   0.220 / 0.200     6 / 6
    #                   0.180 / 0.160    18 / 20
    #                   0.140 / 0.100    23 / 34
    #
    # But a LOWER top reaching right forward costs nothing:
    #
    #     top z = 1.00 / 0.95 / 0.90, edge y = 0.100    0 / 0 / 0 failures
    #
    # So: a TABLE at 0.95 spanning the whole reachable region, and the objects
    # on a RAISED WORK SURFACE at the verified 1.10 whose own edge stays at
    # 0.245 where the grasp needs it. Seen from the front the objects are
    # within the table's footprint and nothing hangs over nothing; the pick
    # geometry is untouched, because the surface they overhang is unchanged.
    yc = (_ct.BENCH_NEAR_Y + _ct.BENCH_FAR_Y) / 2.0
    yd = _ct.BENCH_FAR_Y - _ct.BENCH_NEAR_Y
    # THE BENCH IS GONE. ONE TABLE.
    #
    # It cost 0.250 m of forward reach and supported nothing. Measured
    # (scripts/measure_forward_reach.py), forward reach at working height:
    #
    #     no furniture 0.425   table only 0.425   bench only 0.175
    #
    # and its top surface (y 0.245..0.630) overlapped the reachable band
    # (y 0.05..0.20) by EXACTLY ZERO, so no object could ever be placed on
    # it. Every object was in fact floating in front of its near edge, over
    # nothing -- which is what "the objects are on neither surface" meant.
    #
    # Raising it does not help: with the surface at 1.20/1.25/1.30/1.35 the
    # band reads 0.200 every time. The band rises WITH the surface and never
    # moves forward, because the arm's approach cone points 30.7 deg ABOVE
    # horizontal, so the forearm trails below and behind the fingertip -- and
    # over a slab, that trailing volume is inside the slab.
    #
    # That is also why the objects cannot simply lie on the table: at 20 mm
    # above the table top the band is 0.025 m, i.e. nothing on the table is
    # reachable at all. They need to be ~150 mm clear of it, which is what
    # the PEDESTALS below provide. One table, and small stands on it.
    if task in ("a", "b", "c"):
        out.append(("bench", [0.0, yc, _ct.BENCH_TOP - _ct.BENCH_THICK / 2.0],
                    [2 * _ct.BENCH_HALF_X, yd, _ct.BENCH_THICK], TAN))
    tyc = (TABLE_NEAR_Y + TABLE_FAR_Y) / 2.0
    tyd = TABLE_FAR_Y - TABLE_NEAR_Y
    out.append(("table", [0.0, tyc, TABLE_TOP - TABLE_THICK / 2.0],
                [2 * TABLE_HALF_X, tyd, TABLE_THICK], OAK))
    if task in ("a", "b", "c"):
        out.append(("circuit_box", list(_ct.BOX_OBJ),
                    [0.17, _ct.BOX_D, _ct.BOX_H], GREEN))
        bx, by = _ct.A_BIN_OBJ[0], _ct.A_BIN_OBJ[1]
        w = _ct.A_BIN_D / 2.0
        out.append(("bin_floor", [bx, by, _ct.BENCH_TOP + 0.01],
                    [_ct.A_BIN_D, _ct.A_BIN_D, 0.02], TEAL))
        for dx, dy in ((w, 0.0), (-w, 0.0), (0.0, w), (0.0, -w)):
            out.append(("bin_wall_%+.0f_%+.0f" % (dx * 100, dy * 100),
                        [bx + dx, by + dy,
                         _ct.BENCH_TOP + _ct.A_BIN_H / 2.0],
                        [0.02 if dx else _ct.A_BIN_D,
                         _ct.A_BIN_D if dx else 0.02, _ct.A_BIN_H], TEAL))
        return out
    # PEDESTALS: TRIED, MEASURED, REMOVED. They were added so the objects
    # would rest on something once the bench went, and they are FATAL --
    # 0 of 4 cubes and 0 of 2 planes reachable in EVERY layout tried:
    #
    #     option                       cubes  planes
    #     left arm,  table only         2/4    1/2
    #     left arm,  stands ON          0/4    0/2
    #     right arm, table only         4/4    2/2
    #     right arm, stands ON          0/4    0/2
    #     split,     table only         3/4    2/2
    #
    # This is the bench finding again at 1/30 the size: a 60 mm post under the
    # approach path blocks as completely as a 1.7 m slab, because the arm's
    # trailing forearm needs that volume and does not care how wide the
    # obstacle is. It is not about the support's footprint; it is about there
    # being a support there AT ALL.
    #
    # So the objects are unsupported, and that is recorded as a KNOWN COST
    # rather than hidden: with the pinned wrist there is no support geometry
    # yet found that an object can rest on and still be reached. The table is
    # 170 mm below them and is what a viewer reads as the work surface.
    if not SUPPORTS_ENABLED:
        return out
    if task in ("t1", "t1s2"):
        for i, (cx, cy) in enumerate(_mct.T1_CUBES):
            out.append(_lip("lip_cube_%d" % i, [cx, cy, _mct.T1_Z],
                            (0.04, 0.04, 0.04), _ct.BENCH_NEAR_Y))
        for i, (px, py) in enumerate(_mct.T1_PLANES):
            out.append(_lip("lip_plane_%d" % i,
                            [px, py, _ct.BENCH_TOP + PLANE_T / 2.0],
                            (PLANE_W, PLANE_D, PLANE_T), _ct.BENCH_NEAR_Y))
    elif task == "t3":
        out.append(_lip("lip_circuit_box", _t3.BOX_OBJ, _t3.BOX_SIZE,
                        _ct.BENCH_NEAR_Y))
        out.append(_lip("lip_multimeter", _t3.METER_OBJ, _t3.METER_SIZE,
                        _ct.BENCH_NEAR_Y))
    return out


def fixtures_for(task):
    """Names of the NON-GRASPABLE subjects tick() draws for this task.

    Declared here rather than discovered from a recording, so an audit can ask
    "does the scene publish what the spec requires" without running a clip,
    and so tick() and the audit cannot drift apart.
    """
    if task == "t0":
        return ["L1", "L2", "L3", "R1", "R2", "R3"]
    if task in ("d1", "d2", "d3"):
        return []
    if task == "t1":
        return ["plane_blue", "plane_green"]
    if task == "t2":
        return ["tray", "ball"]
    if task == "t3":
        return ["P1", "P2", "P3", "P4"]
    return []


def furniture_ids():
    """Every collision-object name this scene can own, over ALL tasks.

    A SUPERSET ON PURPOSE. This is what removal is driven from, and removing
    an object that was never added is a no-op in MoveIt, whereas leaving one
    task's lip loaded while the next task runs is exactly the leak the bench
    comment below is about.
    """
    names = set()
    for t in ("a", "b", "c", "t0", "t1", "t2", "t3"):
        for n in furniture_boxes(t):
            names.add(n[0])
    return sorted(names)


def remove_furniture(node, timeout_s=10.0):
    """Take the clip furniture back out of the planning scene.

    THE BENCH MUST NOT LEAK. It is CLIP-scene furniture: the study tasks --
    positioning, coordinated carry, dual pursuit -- are free-space motions
    that were verified without it, and leaving it loaded blocks them. The
    first run after adding the bench failed verify_abc_scenarios' own
    instrument control ("a declared Task A target -> UNREACHABLE"), which is
    exactly what should happen and exactly why the control exists.
    """
    from moveit_msgs.srv import ApplyPlanningScene
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return False
    ps = PlanningScene()
    ps.is_diff = True
    for name in furniture_ids():
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = name
        co.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(co)
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    end = time.time() + timeout_s
    while time.time() < end and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    return fut.done()


class Scene(Node):
    """Static furniture plus one graspable object per arm, per task."""

    def __init__(self, task, out=None, seed=0):
        super().__init__("clip_scene")
        self.task = task
        self.out = out
        self.seed = seed
        # THE EVIDENCE LOG. "Did the task complete" must be a measurement, not
        # a judgement made by squinting at a frame. This records when the
        # fingers actually reached the object's width, how far the object
        # travelled WHILE HELD, and where it was let go -- so a clip can be
        # scored as grasped-and-carried-and-delivered rather than as the arm
        # having moved somewhere near an object.
        self.events = []
        # T2's carry trace. One row per tick for the whole clip, so tilt and
        # separation are a SERIES and not a verdict. Empty for every other
        # task, and empty is meaningful: it says the task has no coupling.
        self.carry_series = []
        self.t0 = None
        qos = QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/task_objects", qos)
        # THE SCENE MUST EXIST TO THE PLANNER, NOT JUST TO THE EYE.
        #
        # A Marker is decoration: `avoid_collisions=True` cannot see one, so
        # the arm swept straight through a bench that looked solid on screen.
        # This project already paid for that lesson once, in task_scene.py --
        # "rehearsing against decoration teaches a motion that will collide on
        # the real rig" -- and the clip scene had regressed it, publishing a
        # MarkerArray and nothing else. The furniture now goes to
        # /planning_scene as CollisionObjects as well.
        # THE SERVICE, NOT THE TOPIC.
        #
        # Publishing a PlanningScene to /planning_scene once at startup put
        # NOTHING in move_group: asked for its world object names it returned
        # NONE, while the bench looked correct on screen. A single publish
        # loses the race with subscription matching, and the topic route gives
        # no acknowledgement that it landed. /apply_planning_scene is
        # synchronous and returns success, and _verify_scene() then reads the
        # names BACK out of move_group -- because "I sent it" and "it is
        # there" have already differed once here.
        self.scene = self.create_publisher(PlanningScene, "/planning_scene", 4)
        self.apply = self.create_client(ApplyPlanningScene,
                                        "/apply_planning_scene")
        self.getscene = self.create_client(GetPlanningScene,
                                           "/get_planning_scene")
        self._scene_sent = False
        self.knuck = {}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

        # graspable: name -> dict(arm, width_mm, pos, size, colour, held,
        #                          placed)
        self.items = self._items(task)
        # OFFSET FROM THE WRIST TO THE FINGER PADS, resolved once from live TF.
        #
        # Every declared coordinate in clip_tasks is an END-EFFECTOR pose --
        # that is what the follower is commanded to and what was verified at
        # N=10. The fingers are 0.1118 m further along the tool axis, so an
        # object drawn AT the declared pick sits 0.1118 m from where the hand
        # actually closes, and a bin drawn at the declared place receives the
        # block 0.1118 m away from itself. Measured on the first attempt:
        # released at (0.604, 0.447, 1.142) against a bin at (0.62, 0.35,
        # 1.02) -- the block was left hanging in mid-air beside the bin.
        #
        # Shifting the OBJECTS by that offset keeps every verified coordinate
        # untouched and makes the picture agree with the physics. One offset
        # for the whole run is right because orientation_mode is `fixed`: the
        # wrist holds the anchor orientation throughout, so the tool axis does
        # not change during a clip.
        self.pad_off = None
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ layout
    def _items(self, task):
        if task == "a":
            return {"block": dict(arm="left", width_mm=40,
                                  pos=list(CT.A_PICK), size=(0.04,) * 3,
                                  col=ORANGE, held=False)}
        if task == "b":
            return {"part": dict(arm="right", width_mm=45,
                                 pos=list(CT.B_START),
                                 size=(0.045, 0.045, 0.05), col=ORANGE,
                                 held=False)}
        if task == "c":
            return {"multimeter": dict(arm="left", width_mm=CT.C_MM_MM,
                                       pos=list(CT.C_PRESENT),
                                       size=CT.C_MM_SIZE, col=YELLOW,
                                       held=False)}

        # ---------------------------------------------------------- MSc set
        import msc_clip_tasks as MCT
        import task3 as T3M
        if task == "t0" or task in ("d1", "d2", "d3"):
            # NO OBJECTS. T0 is reaching only, and the dance is a routine --
            # neither has anything to grasp, which is exactly why they run in
            # every mode including any that cannot.
            return {}
        if task == "t1":
            # Four cubes, two blue and two green, colour matched onto two
            # planes. The pairing is 0,2 -> blue and 1,3 -> green, declared in
            # msc_clip_tasks so a WRONG-COLOUR placement is scoreable rather
            # than ambiguous. Only the first is graspable in the clip; the
            # rest are shown so the colour-matching task is legible.
            out = {}
            for i, (cx, cy) in enumerate(MCT.T1_CUBES):
                # ee_for(), NOT the object position. _items() positions are
                # WRIST poses: tick() adds pad_off to recover where the object
                # actually sits, exactly as the a/b/c tables do (their pos is
                # A_PICK = ee_for(A_BLOCK_OBJ)). Passing the OBJECT position
                # here displaced every MSc item by |PAD_OFFSET| -- measured as
                # T1's min_pad_obj_m of 0.0957 m against a 0.03 m gate, so the
                # fingers closed at the right MOMENT 96 mm from the cube.
                out["cube_%d" % i] = dict(
                    # FOLLOW THE TASK'S ARM. This was hardcoded "left" while
                    # the task moved to the RIGHT arm, so the right gripper
                    # closed on the cube and the scene was watching the left
                    # one. Every T1 grasp went unrecorded: "NO GRASP RECORDED
                    # at all" on a run where the arm plainly picked things up.
                    arm=MCT.T1_ARM, width_mm=40,
                    pos=CT.ee_for([cx, cy, MCT.T1_Z], MCT.T1_ARM),
                    size=(0.04,) * 3,
                    col=(BLUE if i in (0, 2) else GREEN), held=False,
                    # ALL FOUR are picked, one after another: T1's schedule is
                    # four closes and four opens. The flag stays because
                    # tick() now honours it, and a future fixtured item will
                    # need it.
                    graspable=True)
            return out
        if task == "t1s2":
            # BOTH ARMS, RANDOM POSITIONS, drawn from the surveyed cells at
            # the clip's fixed seed so the picture is reproducible. Two cubes
            # per arm, coloured by ARM here rather than by pair: stage 2 is
            # about simultaneity, not colour matching, and reusing the
            # blue/green pairing would invite a reader to look for a colour
            # rule that this stage does not have.
            import msc_clip_tasks as _MCT
            out = {}
            # THE RUN'S SEED, not a hardcoded one. This read T0_CLIP_SEED
            # while run_abc drove --seed, so the two could draw different
            # layouts and the scene would then be measuring closures against
            # cubes the arm was never sent to.
            tgt = _MCT.stage2_targets(self.seed)["cubes"]
            for arm in ("left", "right"):
                for i, cube in enumerate(tgt[arm]):
                    out["cube_%s_%d" % (arm, i)] = dict(
                        arm=arm, width_mm=40,
                        pos=CT.ee_for(cube, arm),
                        size=(0.04,) * 3,
                        col=(YELLOW if arm == "left" else TEAL),
                        held=False, graspable=True)
            return out
        if task == "t2":
            # NO `items` ENTRY. The tray is ONE body held at TWO points, and
            # an `items` entry is pinned to ONE arm's pads by construction --
            # which is what put a 560 mm tray centred on the left hand while
            # the right one held air. It is drawn as a FIXTURE spanning
            # between the two grippers instead; see tick().
            #
            # Nothing is lost: T2 declares width_mm = 0, grip_obj = None and
            # place_target = None, because the task is the CARRY and both
            # grippers are already closed on the tray when it starts. There is
            # no grasp event to record.
            return {}
        if task == "t3":
            # WIDTH 50, NOT 110. The box is 170 x 110 x 50 mm and the 2F-85
            # spans 85 mm, so 110 is not a grip the hand can make at all --
            # grip_for(110) never produced a closable target and the knuckle
            # stayed at 0.0 for the whole run (min_pad_obj_closed_m: None).
            # 50 mm is the box's HEIGHT: the gripper takes it across the
            # thickness, which is the only dimension that fits.
            return {"circuit_box": dict(arm="right", width_mm=50,
                                        pos=CT.ee_for(T3M.BOX_OBJ,
                                                       T3M.BOX_ARM),
                                        size=T3M.BOX_SIZE, col=GREEN,
                                        held=False),
                    "multimeter": dict(arm="left", width_mm=30,
                                       pos=CT.ee_for(T3M.METER_OBJ,
                                                      T3M.METER_ARM),
                                       size=T3M.METER_SIZE, col=YELLOW,
                                       held=False)}
        raise KeyError("clip_scene has no item table for task %r -- refusing "
                       "to draw an empty scene under a task's name" % task)

    def _js(self, m):
        for a in ("left", "right"):
            n = KN % a
            if n in m.name:
                self.knuck[a] = float(m.position[m.name.index(n)])

    # Distance from end_effector_link to the centre of the finger PADS, along
    # the tool approach axis (+z in the EE frame). A Robotiq 2F-85's fingers
    # close about here; the wrist is 111.8 mm behind it.
    PAD_DEPTH_M = 0.1118

    def _grip_axis(self, arm):
        """The gripper's CLOSING axis in world, from the two finger pads.

        Read from TF rather than derived from the wrist quaternion, so it
        cannot disagree with the model actually running. Returns None if
        either pad frame is missing -- and None is treated by the caller as
        "no measurement", never as "aligned", because a missing check that
        reads as a pass is the failure this gate was added to stop.
        """
        try:
            a = self.buf.lookup_transform(
                "world", "%s_robotiq_85_left_finger_tip_link" % arm,
                rclpy.time.Time()).transform.translation
            b = self.buf.lookup_transform(
                "world", "%s_robotiq_85_right_finger_tip_link" % arm,
                rclpy.time.Time()).transform.translation
        except Exception:                                     # noqa: BLE001
            return None
        return (b.x - a.x, b.y - a.y, b.z - a.z)

    def _grip(self, arm):
        """Where the finger PADS are, in world -- not where the wrist is.

        The object used to be pinned 45 mm below the end effector in WORLD z,
        which put it under the wrist and above the open fingers: on the
        close-up view the block hovered at the gripper's base rather than
        between the pads, and it stayed 45 mm below the wrist however the
        wrist was rotated. The offset has to ride the TOOL AXIS, or a grasp
        with the hand tilted renders the object off to one side of it.
        """
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = t.transform.translation
        q = t.transform.rotation
        # Rotate (0, 0, PAD_DEPTH) by q -- the approach axis in world.
        x, y, z, w = q.x, q.y, q.z, q.w
        d = self.PAD_DEPTH_M
        ax = 2.0 * (x * z + w * y) * d
        ay = 2.0 * (y * z - w * x) * d
        az = (1.0 - 2.0 * (x * x + y * y)) * d
        return [v.x + ax, v.y + ay, v.z + az]

    # ------------------------------------------------------------- frame
    def _collision_furniture(self, task=None):
        """THIS TASK's solids as real CollisionObjects.

        ONE TABLE, `furniture_boxes()`, feeds both this and the markers drawn
        in tick(). Two descriptions of one bench is how a scene comes to look
        solid and behave hollow, and this file has already paid for that.

        The graspable object is deliberately NOT included: it is attached to
        the gripper as the hand closes, and a collision object sitting where
        the fingers must go would make every grasp pose infeasible. Its
        SUPPORT is what has to be solid, not the thing being picked up.
        """
        from geometry_msgs.msg import Pose
        out = []
        for name, xyz, size, _col in furniture_boxes(
                task or getattr(self, "task", None)):
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            pr = SolidPrimitive()
            pr.type = SolidPrimitive.BOX
            pr.dimensions = [float(v) for v in size]
            co.primitives.append(pr)
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = [float(v)
                                                           for v in xyz]
            ps.orientation.w = 1.0
            co.primitive_poses.append(ps)
            co.operation = CollisionObject.ADD
            out.append(co)
        return out

    def _publish_scene(self):
        # CLEAR THE PREVIOUS TASK'S FURNITURE FIRST. Now that each task loads
        # only its own, a leak is no longer merely untidy: T0 declares NO
        # furniture, so a bench left behind by the T3 clip would put T0 back
        # inside the very limits removing the bench was meant to lift, and
        # nothing downstream would disagree. clip_scene removes its own on
        # exit, but only on the paths where it is allowed to run its handler.
        try:
            remove_furniture(self, timeout_s=8.0)
        except Exception:                                     # noqa: BLE001
            pass
        objs = self._collision_furniture()
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = objs
        # Topic as well, for any consumer that only watches it (RViz).
        self.scene.publish(ps)
        if not self.apply.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "[SCENE] /apply_planning_scene is not available -- the bench "
                "will be DECORATION and the arm will sweep through it")
            return False
        fut = self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        end = time.time() + 10.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        got = self._verify_scene([o.id for o in objs])
        self.get_logger().info(
            "[SCENE] %d collision objects applied, %d confirmed in move_group"
            % (len(objs), len(got)))
        return len(got) == len(objs)

    def _verify_scene(self, want):
        """Read the world object names BACK from move_group."""
        from moveit_msgs.msg import PlanningSceneComponents
        if not self.getscene.wait_for_service(timeout_sec=8.0):
            return []
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = self.getscene.call_async(req)
        end = time.time() + 10.0
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done() or fut.result() is None:
            return []
        have = {o.id for o in fut.result().scene.world.collision_objects}
        missing = [w for w in want if w not in have]
        if missing:
            self.get_logger().error(
                "[SCENE] NOT IN move_group: %s -- these are decoration"
                % ", ".join(missing))
        return [w for w in want if w in have]

    def _ee(self, arm):
        """The wrist in world, or None. Used for the travel measurement."""
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = t.transform.translation
        return [v.x, v.y, v.z]

    def _pad_offset(self, arm):
        """Wrist -> finger-pad vector in world, AT THE PINNED ANCHOR.

        A CONSTANT, NOT A LIVE READING, and that is the fix. This used to
        sample the arm's CURRENT orientation once at clip start, which is
        only the anchor if nothing has moved the arm first -- and something
        now does: the presentation pose is staged before capture, which
        rotates the wrist. Measured, the right arm's tool-axis offset moves
        80.1 mm between home and the staged pose, so the whole scene was
        drawn 80 mm out and every grasp missed by ~31 mm against a 30 mm
        gate. "NO GRASP RECORDED at all" on a run where the arm plainly
        picked things up, for the second time in this file's history.
        SHARED WITH THE TASK LAYER. clip_tasks.PAD_OFFSET_BY_ARM is the same
        table `ee_for()` derives every commanded wrist pose from, so the
        picture and the path cannot disagree about where an object is. Two
        descriptions of one offset is what produced the 49 mm error this
        module already carries a comment about.
        """
        off = CT.PAD_OFFSET_BY_ARM.get(arm)
        if off is not None:
            return list(off)
        # Only reached for an arm name the table does not know, which is a
        # programming error rather than a missing measurement -- fall through
        # to the live reading and let the caller see something rather than
        # silently drawing the scene at the origin.
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        d = self.PAD_DEPTH_M
        return [2.0 * (x * z + w * y) * d,
                2.0 * (y * z - w * x) * d,
                (1.0 - 2.0 * (x * x + y * y)) * d]

    def tick(self):
        # THE FURNITURE IS PUBLISHED UNCONDITIONALLY. Only the GRASPABLE items
        # are gated on self.items.
        #
        # The first version returned here when items was empty, to avoid
        # `next(iter(...))` raising StopIteration on a task with no objects.
        # It suppressed the WHOLE SCENE: the T0 clip rendered the arms waving
        # in an empty void -- no spheres, no bench, no bin -- and passed every
        # automatic check, because "the arm moved" was all anything measured.
        # A task with nothing to GRASP still has a bench to stand on and
        # targets to reach.
        # ONE OFFSET PER ARM, NOT ONE PER SCENE.
        #
        # The wrist->pad vector is the anchor orientation rotated into world,
        # and the two arms are parked asymmetrically, so their offsets differ:
        # measured, left (-0.0171, +0.0945, +0.0572) against right (+0.0289,
        # +0.0995, +0.0421) -- 49 mm apart. The whole scene used to be shifted
        # by whichever arm owned the FIRST item, so in T3 the multimeter, a
        # LEFT-arm object, was drawn 49 mm from where the left hand closes --
        # outside the 30 mm capture window. The gripper closed exactly on
        # schedule (`[GRIP] deferred 1/5 ticks`), the grasp test refused it,
        # and the clip showed an open hand passing an untouched meter with
        # carried_m 0.000 and no GRASPED event at all.
        if self.items and self.pad_off is None:
            arms = {it["arm"] for it in self.items.values()}
            offs = {a: self._pad_offset(a) for a in arms}
            if any(v is None for v in offs.values()):
                return          # no TF yet -- draw nothing rather than draw
                                # the scene in the wrong place
            self.pad_off_by_arm = offs
            # The scalar `pad_off` is kept for consumers that compare ONE
            # declared EE-frame target against the scene: record_abc_sweep
            # shifts `place_target` by it. It is the offset of the arm that
            # owns the FIRST item, which is the arm place_target belongs to in
            # every task that declares one.
            self.pad_off = offs[next(iter(self.items.values()))["arm"]]
            for it in self.items.values():
                o = offs[it["arm"]]
                it["pos"] = [it["pos"][i] + o[i] for i in range(3)]

        # ---- HOW FAR DID THE ARM ACTUALLY GO -----------------------------
        # The path integral of each end effector over the life of this node,
        # which is the life of the clip. run_abc prints the same quantity, but
        # its stdout goes to /dev/null through the GUI's job launcher, so no
        # clip on disk has ever carried the one number that says whether the
        # motion is legible. Written into scene_events.json beside everything
        # else the clip is judged on.
        self.ee_track = getattr(self, "ee_track", {"left": None, "right": None})
        self.ee_travel = getattr(self, "ee_travel", {"left": 0.0, "right": 0.0})
        self.ee_first = getattr(self, "ee_first", {})
        self.ee_samples = getattr(self, "ee_samples", {"left": 0, "right": 0})
        # THE SAMPLE COUNT TRAVELS WITH THE PATH LENGTH.
        #
        # `ee_travel_m` is a sum of |dp| over TICKS, so it is a property of how
        # long and how often the recorder watched as much as of how the arm
        # moved: more ticks over a stationary arm still accumulates sensor
        # noise, and a run observed for longer reads as a longer path. The
        # count is written out now so any comparison between runs can be
        # normalised and audited instead of assumed comparable.
        #
        # `ee_first` is likewise the FIRST POSE THIS NODE HAPPENED TO SEE, not
        # the pose the task started from, so `ee_net_m` is anchored on the
        # recorder's start-up rather than on the motion. Both are reported
        # with that caveat attached in scene_events.json rather than left for
        # a reader to discover. See findings.md, 2026-08-15, finding 3.
        for _arm in ("left", "right"):
            _p = self._ee(_arm)
            if _p is None:
                continue
            self.ee_first.setdefault(_arm, list(_p))
            _prev = self.ee_track[_arm]
            if _prev is not None:
                self.ee_travel[_arm] += math.dist(_prev, _p)
                self.ee_samples[_arm] += 1
            self.ee_track[_arm] = list(_p)

        A = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        A.markers.append(d)
        i = 0

        def add(typ, xyz, scale, col, ns="scene", quat=None):
            nonlocal i
            A.markers.append(_m(ns, i, typ, xyz, scale, col, quat=quat))
            i += 1

        # ---- furniture, PER TASK ---------------------------------------
        # DRAWN FROM THE SAME TABLE AS THE COLLISION OBJECTS, so the picture
        # and the planner cannot disagree. Two descriptions of one bench is
        # how a scene comes to look solid and behave hollow.
        solids = furniture_boxes(self.task)
        for _name, _xyz, _size, _col in solids:
            add(Marker.CUBE, list(_xyz), tuple(_size), _col)
        if any(s[0] == "table" for s in solids):
            # LEGS AND APRON -- drawn only, because none of them is anywhere
            # the arm goes and a collision object that is never approached is
            # cost without cover.
            #
            # WHAT A TABLE NEEDS TO READ AS ONE, and the shipped one had two
            # of the three. It had a top and four legs; its skirt ran along
            # the near and far edges ONLY, so from the ends the top floated on
            # two rails, and the legs sat at the extreme corners with no
            # overhang, which reads as a box rather than a table.
            #
            #   * the legs are INSET, so the top overhangs them on all four
            #     sides. An overhang is most of what says "table top" at a
            #     glance;
            #   * the apron runs all the way round, four rails not two;
            #   * a thin RAIL along the near edge, which is the edge every
            #     front view looks straight down at and the one that carries
            #     the silhouette.
            #
            # Nothing here moves the top, the near edge or the half-width:
            # y = 0.100 is the measured edge (0 waypoint failures; 0.02 cost
            # T1 four), |x| = 1.05 covers the marking out to 1.000, and the
            # top at 0.95 is the highest slab that costs nothing.
            tyc = (TABLE_NEAR_Y + TABLE_FAR_Y) / 2.0
            tyd = TABLE_FAR_Y - TABLE_NEAR_Y
            for sx in (-1, 1):
                for sy in (TABLE_NEAR_Y + TABLE_LEG_INSET,
                           TABLE_FAR_Y - TABLE_LEG_INSET):
                    add(Marker.CUBE,
                        [sx * (TABLE_HALF_X - TABLE_LEG_INSET), sy,
                         (TABLE_TOP - TABLE_THICK) / 2.0],
                        (TABLE_LEG, TABLE_LEG, TABLE_TOP - TABLE_THICK),
                        LEG)
            z_apron = TABLE_TOP - TABLE_THICK - TABLE_APRON / 2.0
            inset = TABLE_LEG_INSET - TABLE_LEG / 2.0
            for sy in (TABLE_NEAR_Y + inset, TABLE_FAR_Y - inset):
                add(Marker.CUBE, [0.0, sy, z_apron],
                    (2 * (TABLE_HALF_X - inset), 0.022, TABLE_APRON), APRON)
            for sx in (-1, 1):
                add(Marker.CUBE,
                    [sx * (TABLE_HALF_X - inset), tyc, z_apron],
                    (0.022, tyd - 2 * inset, TABLE_APRON), APRON)
            # NO SEPARATE NEAR-EDGE RAIL, AND THE RENDER IS WHY. One was
            # drawn here to give the front edge its own tone, 12 mm deep at
            # y = 0.100. Its front face is then EXACTLY COPLANAR with the top
            # slab's, which is z-fighting: measured, the slab won at the front
            # camera angle, but which of two coincident faces wins is not
            # something to leave to the depth buffer in a video.
            #
            # It bought nothing anyway. The top slab's OWN front face points
            # at the camera and so takes the diffuse term the top face cannot:
            # measured 221 of 255 at full white, against 127 for the top. The
            # bright edge line is already there and it is free.
            # THE RISERS CARRY THE BENCH, AND ONLY a/b/c HAS A BENCH.
            #
            # They were drawn whenever a table was, so T1, T1S2, T2 and T3 --
            # every MSc clip -- showed two 110 mm posts standing on the table
            # holding nothing, 60 mm short of the objects they appear to be
            # under. One of them is visible in the front view of every T1
            # clip in the 2026-08-15 set. A support under nothing invites a
            # viewer to read the objects as resting on it, which is the one
            # thing this scene must not imply: with the pinned wrist there is
            # no support geometry an object can rest on and still be reached,
            # and that absence is a recorded cost, not something to dress.
            if any(s[0] == "bench" for s in solids):
                gap = CT.BENCH_TOP - CT.BENCH_THICK - TABLE_TOP
                for sx in (-0.60, 0.60):
                    add(Marker.CUBE,
                        [sx, (CT.BENCH_NEAR_Y + CT.BENCH_FAR_Y) / 2.0,
                         TABLE_TOP + gap / 2.0],
                        (0.06, CT.BENCH_FAR_Y - CT.BENCH_NEAR_Y - 0.04, gap),
                        RISER)

        # ---- PER-TASK SCENE FIXTURES: the task's SUBJECT ----------------
        # Things the task is ABOUT that are not grasped. They belong here, not
        # in _items(), because _items() is the attach/detach machinery -- an
        # entry there follows the gripper. A T0 sphere must be visible and
        # must NOT move with the hand.
        #
        # This section exists because two clips passed every automatic check
        # with their subject missing: T0 showed arms reaching for nothing, and
        # T2 showed a tray with no ball to fall off it.
        self.fixtures = getattr(self, "fixtures", [])
        if self.task == "t0":
            # The SAMPLED study set at the clip's fixed seed -- not the
            # A/B/C/D calibration constants, which is a different set at
            # different coordinates.
            import msc_clip_tasks as _MCT
            import task0 as _T0
            tgt, _meta = _T0.sample_trial(_MCT.T0_CLIP_SEED)
            for label in ("L1", "L2", "L3", "R1", "R2", "R3"):
                p3 = tgt[label]
                # COLOURED AS THE SPEC COLOURS THEM. task0.SPHERE_COLOURS is
                # red / green / blue for 1 / 2 / 3 on BOTH arms, and the clip
                # was colouring by ARM instead -- so "go to the green one" had
                # three blue spheres on the left and three green on the right,
                # and the FULL_AUTONOMY condition, whose whole point is naming
                # one of several visible targets by colour, could not be shown.
                add(Marker.SPHERE, list(p3),
                    (2 * _T0.TARGET_R_M,) * 3,
                    SPHERE_RGBA[_T0.SPHERE_COLOURS[label]], ns="targets")
                # The LABEL, so a viewer can tell L2 from L3 rather than
                # trusting that the arm went to the right one.
                lab = Marker()
                lab.header.frame_id = "world"
                lab.ns, lab.id = "target_labels", i
                lab.type = Marker.TEXT_VIEW_FACING
                lab.action = Marker.ADD
                lab.text = label
                lab.pose.position.x = float(p3[0])
                lab.pose.position.y = float(p3[1])
                lab.pose.position.z = float(p3[2]) + 0.045
                lab.pose.orientation.w = 1.0
                lab.scale.z = 0.045
                (lab.color.r, lab.color.g,
                 lab.color.b, lab.color.a) = (1.0, 1.0, 1.0, 1.0)
                A.markers.append(lab)
                i += 1
                if label not in self.fixtures:
                    self.fixtures.append(label)
        if marked_arms(self.task):
            # THE MARKED REACHABLE REGION, on the surface, per arm. Drawn as
            # four thin bars rather than a filled patch so it reads as a
            # boundary and does not hide what is standing inside it.
            # ONLY THE ARMS THIS TASK USES. T1 runs on one arm, and drawing
            # the other one's marking put a large empty rectangle in the
            # middle of every T1 frame -- which is what the front view was
            # centring on while the actual work sat at the edge of the shot.
            for arm in marked_arms(self.task):
                col = MARK_RGBA[arm]
                zt = CT.BENCH_TOP + MARK_T / 2.0
                # THE MEASURED CELLS, drawn as cells. The bounding box was
                # what used to be drawn and the right arm's cells fill only
                # 62% of it, so the box asserted a third of a region that
                # was never measured reachable.
                cell = REGION_STEP - 0.004        # a hairline between tiles
                for cx, cy in REGION_CELLS[arm]:
                    add(Marker.CUBE, [cx, cy, zt], (cell, cell, MARK_T),
                        (col[0], col[1], col[2], 0.30), ns="workspace")
                # A heavier outline on the bounding box, so the region reads
                # as one shape from across the room and the cells give it
                # its true edge close up.
                x0, x1 = WORKSPACE[arm]["x"]
                y0, y1 = WORKSPACE[arm]["y"]
                x0, x1 = x0 - REGION_STEP / 2.0, x1 + REGION_STEP / 2.0
                y0, y1 = y0 - REGION_STEP / 2.0, y1 + REGION_STEP / 2.0
                for yy in (y0, y1):
                    add(Marker.CUBE, [(x0 + x1) / 2.0, yy, zt],
                        (x1 - x0, MARK_W, MARK_T), col, ns="workspace")
                for xx in (x0, x1):
                    add(Marker.CUBE, [xx, (y0 + y1) / 2.0, zt],
                        (MARK_W, y1 - y0, MARK_T), col, ns="workspace")
                lab = Marker()
                lab.header.frame_id = "world"
                lab.ns, lab.id = "workspace_labels", i
                lab.type = Marker.TEXT_VIEW_FACING
                lab.action = Marker.ADD
                lab.text = "%s arm reach (measured)" % arm
                lab.pose.position.x = float((x0 + x1) / 2.0)
                lab.pose.position.y = float(y0 - 0.03)
                lab.pose.position.z = float(CT.BENCH_TOP + 0.02)
                lab.pose.orientation.w = 1.0
                lab.scale.z = 0.030
                (lab.color.r, lab.color.g,
                 lab.color.b, lab.color.a) = col
                A.markers.append(lab)
                i += 1
        if self.task in ("t1", "t1s2"):
            # THE TWO COLOURED PLANES. T1 is "blue cube to blue plane, green
            # cube to green plane" and the planes had never been drawn, so the
            # task had no target on screen and a wrong-colour placement could
            # not be scored from a frame.
            #
            # Positions are T1_PLANES verbatim -- the option-4 layout,
            # verified N=10 over the full path -- and the pairing is the one
            # declared in msc_clip_tasks: cubes 0,2 -> plane 0 (BLUE),
            # cubes 1,3 -> plane 1 (GREEN). Drawn as MATS resting on their
            # lips, top flush with the bench-top plane, which is the plane a
            # placed cube's base sits on.
            # STAGE 2 GETS THE PADS TOO, AND ONE PAIR PER SIDE.
            #
            # This block ran for `t1` only, so stage 2 -- the both-arms half of
            # the same task -- had no pads drawn at all and placed onto bare
            # coordinates. A colour-matching task with no colours on screen
            # cannot be scored from a frame, which is the whole reason the pads
            # were added to stage 1 in the first place.
            import msc_clip_tasks as _MCT
            _pads = []
            if self.task == "t1":
                _pads = [(_MCT.T1_ARM, p) for p in _MCT.T1_PLANES]
            else:
                for _a in ("left", "right"):
                    _pads += [(_a, p) for p in _MCT.T1_PLANES_BY_ARM[_a]]
            for pi, (_arm_of_pad, (px, py)) in enumerate(_pads):
                # the colour alternates within each side's pair, so both sides
                # show one blue and one green
                pi = pi % len(_MCT.T1_PLANES)
                name = "plane_%s_%s" % ("blue" if pi == 0 else "green",
                                        _arm_of_pad)
                add(Marker.CUBE,
                    [px, py, CT.BENCH_TOP - PLANE_T / 2.0],
                    (PLANE_W, PLANE_D, PLANE_T),
                    BLUE if pi == 0 else GREEN, ns="planes")
                # an outline, so the mat reads as a target and not as a
                # shadow on the bench
                add(Marker.CUBE,
                    [px, py, CT.BENCH_TOP - PLANE_T / 2.0],
                    (PLANE_W * 1.10, PLANE_D * 1.14, PLANE_T * 0.5),
                    ((BLUE if pi == 0 else GREEN)[0],
                     (BLUE if pi == 0 else GREEN)[1],
                     (BLUE if pi == 0 else GREEN)[2], 0.35), ns="planes")
                if name not in self.fixtures:
                    self.fixtures.append(name)
        elif self.task == "t3":
            # THE FOUR MEASUREMENT POINTS. task3 declares them and the clip
            # never drew them, so the clip could not show the one thing T3's
            # design turns on: P1 and P2 are served by the initial
            # presentation, P3 is on the FAR face and P4 needs the meter
            # turned, so at least two REPOSITIONING REQUESTS are structurally
            # required. Without the points on screen a viewer sees a hold,
            # not a task with a coordination demand in it.
            # ALL FOUR ARE ON THE BOX. P4 used to be drawn on the MULTIMETER,
            # which contradicts the spec -- "one circuit box with four
            # measurement points, one multimeter" -- and it misread what P4's
            # `needs` says: the meter's DISPLAY has to be turned toward the
            # wearer to READ that point, which is a fact about the meter and
            # not about where the point is. A test point lives on the board
            # being probed.
            #
            # THEY MUST READ AS MEASUREMENT POINTS, NOT AS DECORATION, and a
            # 14 mm sphere floating at the face did not: at T3's framing it
            # is sub-pixel, and the earlier pass recorded that as an honest
            # residual. Three things fix it, and none of them is "make the
            # dot bigger until it shows up":
            #
            #   * a FLAT PAD ON THE FACE, not a ball beside it. A disc lying
            #     in the surface is what a test pad looks like; a sphere is a
            #     marker hovering near one.
            #   * a RING around it, so the pad has an edge and survives being
            #     shaded down to a few pixels.
            #   * a LABEL. P1..P4 named on screen is what makes a viewer read
            #     them as points being measured IN TURN rather than as spots
            #     on a box, and it is the same thing that made T0's spheres
            #     legible.
            #
            # Colour still carries the coordination demand: yellow for the
            # two the initial presentation serves, red for the two that
            # structurally require a repositioning request.
            import task3 as _T3
            it = self.items.get("circuit_box")
            for mp in (_T3.MEASUREMENT_POINTS if it is not None else []):
                size = it["size"]
                far = mp["face"] == "far"
                dy = (size[1] / 2.0 if far else -size[1] / 2.0)
                col = (YELLOW if mp["served_by_initial_presentation"]
                       else RED)
                p3 = [it["pos"][0] + mp["offset_mm"][0] / 1000.0,
                      it["pos"][1] + dy,
                      it["pos"][2] + mp["offset_mm"][1] / 1000.0]
                # PAD_T sticks out of the face by half its depth so the pad
                # is never coplanar with the box: two coincident surfaces
                # z-fight and the pad flickers in and out between frames,
                # which on an 8-13 fps capture looks like a fault.
                PAD_W, PAD_T = 0.020, 0.006
                add(Marker.CUBE, p3, (PAD_W, PAD_T, PAD_W), col, ns="measure")
                add(Marker.CUBE, p3, (PAD_W * 1.7, PAD_T * 0.6, PAD_W * 1.7),
                    (col[0], col[1], col[2], 0.45), ns="measure")
                lab = Marker()
                lab.header.frame_id = "world"
                lab.ns, lab.id = "measure_labels", i
                lab.type = Marker.TEXT_VIEW_FACING
                lab.action = Marker.ADD
                lab.text = mp["id"]
                lab.pose.position.x = float(p3[0])
                # The label stands OFF the face on the same side the pad
                # does, so it never sinks into the box on the far side.
                lab.pose.position.y = float(p3[1] + (0.03 if far else -0.03))
                lab.pose.position.z = float(p3[2] + 0.028)
                lab.pose.orientation.w = 1.0
                lab.scale.z = 0.030
                (lab.color.r, lab.color.g,
                 lab.color.b, lab.color.a) = (1.0, 1.0, 1.0, 1.0)
                A.markers.append(lab)
                i += 1
                if mp["id"] not in self.fixtures:
                    self.fixtures.append(mp["id"])
        elif self.task == "t2":
            # THE TRAY IS ONE BODY HELD AT TWO POINTS, SO IT IS DRAWN BETWEEN
            # THE TWO GRIPPERS -- not at one of them.
            #
            # It used to be an ordinary `items` entry owned by the LEFT arm,
            # and the attach rule pins an item to its owner's pads. With the
            # grips 500 mm apart that put a 560 mm tray centred on the left
            # hand, spanning from 30 mm PAST the right gripper to 280 mm
            # beyond the left one: the right arm was holding air, in the one
            # task whose entire claim is that neither arm's pose is free given
            # the other's.
            gl, gr = self._grip("left"), self._grip("right")
            if gl is not None and gr is not None:
                mid = [(gl[k] + gr[k]) / 2.0 for k in range(3)]
                span = math.dist(gl, gr)
                th = TSK.TASK_B["objects"]["tray"]["size"][2]
                dep = TSK.TASK_B["objects"]["tray"]["size"][1]
                # THE TRAY IS RIGID, AND IT WAS DRAWN ELASTIC.
                #
                # Its length was `span + 0.06`, the LIVE distance between the
                # grippers plus a constant, so the tray grew and shrank to fit
                # whatever the arms were doing. Measured over the shipped
                # clips it ran 0.487 to 1.211 m against a 0.560 m spec: it
                # stretched to nearly a quarter more than double its length
                # and still looked held at both ends. That is why T2-1 ("held
                # by BOTH grippers") and T2-2 ("ball visible ON the tray")
                # could not fail -- not because the arms were coordinated, but
                # because the prop deformed to match them, and the ball rode a
                # surface that was redefined every frame to stay under it.
                #
                # It is drawn at the SPEC length now, from tasks.TASK_B, and
                # oriented along the line between the grippers. A separation
                # error is then visible as exactly what it is: a rigid board
                # that does not reach one of the hands, or one whose end
                # sticks out past it.
                spec_len = float(TSK.TASK_B["objects"]["tray"]["size"][0])
                dxv = [gr[k] - gl[k] for k in range(3)]
                nrm = math.sqrt(sum(v * v for v in dxv)) or 1.0
                ux = [v / nrm for v in dxv]
                # yaw and pitch that take +x onto the gripper line; roll is
                # left at zero so the tray stays level about its own long axis
                yaw = math.atan2(ux[1], ux[0])
                pitch = -math.asin(max(-1.0, min(1.0, ux[2])))
                cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
                cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                tray_q = (-sy * sp, cy * sp, sy * cp, cy * cp)
                add(Marker.CUBE, mid, (spec_len, dep, th),
                    TAN, ns="tray", quat=tray_q)
                self.tray_span_m = round(span, 4)
                self.tray_spec_m = round(spec_len, 4)
                self.tray_fit_err_m = round(span - spec_len, 4)
                # THE BALL, AND IT HAS TO BE ABLE TO FALL OFF.
                #
                # It was drawn ON the tray and carried WITH it, unconditionally
                # and before the tilt was even computed, so it rode the tray
                # through any angle at all. Measured over the shipped clips the
                # carry reached 23 deg with the ball still sitting on the
                # surface, against a declared drop angle of 6.8 -- which is
                # 60 mm of height difference over 500 mm and is the whole
                # failure criterion of the task. A failure indicator that
                # cannot indicate failure is set dressing.
                #
                # It rolls off past `fail_tilt_deg` now, and the drop LATCHES:
                # a ball that climbs back on when the tray levels again turns
                # a carry that failed in the middle into a carry that passed,
                # which is the same defect wearing the opposite sign.
                dz = gl[2] - gr[2]
                tilt = math.degrees(math.asin(
                    max(-1.0, min(1.0, dz / span)))) if span > 1e-6 else 0.0
                fail_tilt = float(TSK.TASK_B.get("fail_tilt_deg", 6.8))
                if abs(tilt) > fail_tilt and not getattr(self, "ball_off", False):
                    self.ball_off = True
                    self.ball_off_tilt_deg = round(tilt, 2)
                if getattr(self, "ball_off", False):
                    # on the floor under the LOW end of the tray, which is
                    # where it went
                    low = gl if gl[2] <= gr[2] else gr
                    bp = [low[0], low[1], TABLE_TOP + CT_BALL_R]
                else:
                    bp = list(mid)
                    bp[2] += th / 2.0 + CT_BALL_R
                add(Marker.SPHERE, bp, (2 * CT_BALL_R,) * 3, YELLOW, ns="ball")
                for nm in ("tray", "ball"):
                    if nm not in self.fixtures:
                        self.fixtures.append(nm)
                # ---- TILT AND SEPARATION, EVERY TICK ---------------------
                #
                # tasks.TASK_B has declared `log_continuously = ("tilt_deg",
                # "sep_err_mm", "height_diff_mm")` since the task was written
                # and NOTHING READ IT. The clip could only ever answer "did
                # the ball stay on", which is the end state of a carry whose
                # whole measurement is the carry: a tray that swings to 9 deg
                # in the middle and comes back level scores identically to
                # one that never moves. That is the difference the coupling
                # task exists to show.
                #
                # Measured here rather than in the trial logger because the
                # clip is the only thing that knows where the two grippers
                # actually are, and because a series in scene_events.json is
                # what an inspection pass can plot against the footage.
                #
                # TILT IS OVER THE SPAN, not over a nominal. 6.8 deg is 60 mm
                # of height difference over 500 mm, and taking the baseline
                # from a constant while the arms drift apart reports an angle
                # the tray is not at.
                if self.t0 is None:
                    self.t0 = self.get_clock().now().nanoseconds * 1e-9
                    self.t0_wall = time.time()
                tnow = self.get_clock().now().nanoseconds * 1e-9 - self.t0
                self.carry_series.append(dict(
                    t=round(tnow, 2),
                    tilt_deg=round(tilt, 3),
                    sep_m=round(span, 4),
                    sep_err_mm=round((span - TSK.TRAY_SEP) * 1000.0, 1),
                    height_diff_mm=round(dz * 1000.0, 1),
                    tray_fit_err_m=self.tray_fit_err_m,
                    ball_off=bool(getattr(self, "ball_off", False))))

        # ---- the graspable object --------------------------------------
        for name, it in self.items.items():
            arm = it["arm"]
            k = self.knuck.get(arm)
            g = self._grip(arm)
            # ABSENCE OF DATA IS NOT A RELEASE.
            #
            # /joint_states takes a moment to arrive, so on the first ticks
            # the knuckle is unknown -- and an unknown knuckle used to
            # evaluate as "not closed", which for an item declared held=True
            # fired a RELEASED event at t=0.00 before any measurement existed.
            # T2's tray was dropped on the first frame of every clip and never
            # picked up again. Draw the item where it was declared and wait.
            if k is None:
                add(Marker.CUBE, it["pos"], it["size"], it["col"], ns="item")
                continue
            # PROXIMITY IS PART OF "GRASPED", AND IT WAS MISSING.
            #
            # The test was the knuckle alone, so ANY closure anywhere counted
            # as grasping this object. Measured on the 2026-08-10 re-record:
            # task A logged GRASPED at t=1.21 s with knuckle 0.5176 while the
            # arm was still at home and the block was 0.5 m away, then a
            # RELEASED that carried nothing, and the REAL grasp 29 s later was
            # reported as "23.8 s outside the clip". The event was true about
            # the fingers and false about the object.
            #
            # record_rviz.py already gates its own attachment on `near_pick`
            # for exactly this reason; clip_scene, which writes the file the
            # sweep judges completion from, did not. Two descriptions of one
            # grasp, and the authoritative one was the weaker.
            #
            # GRASP_NEAR_M is the capture half-window rounded up: 22.5 mm for
            # the 40 mm block is the distance at which the fingers close BESIDE
            # the object rather than on it, so beyond that a closure cannot be
            # a grasp of this item whatever the knuckle says.
            # A FIXTURE IS DRAWN AND NEVER FOLLOWS THE HAND.
            #
            # `graspable: False` was declared on three of T1's four cubes and
            # never read, so every cube the hand closed near came with it.
            # Measured on the first mode-06 re-record: four GRASPED events and
            # all four cubes ending at the same point.
            #
            # AND A PLACED OBJECT STAYS PLACED. Two blue cubes go to the same
            # blue plane, so the hand arrives at plane 0 a second time with
            # cube_2 while cube_0 is already sitting there -- closed, and well
            # inside GRASP_NEAR_M of it. Without this the placed cube is
            # silently picked up again by the delivery of the next one.
            if it.get("graspable") is False or it.get("placed"):
                add(Marker.CUBE, it["pos"], it["size"], it["col"], ns="item")
                continue
            closed = rr.holding(k, it["width_mm"])
            d = math.dist(g, it["pos"]) if g is not None else None
            # CLOSEST APPROACH, LOGGED WHETHER OR NOT IT BECAME A GRASP.
            #
            # Without this a refused grasp is unattributable: "no grasp" is
            # equally consistent with the path landing short and with the gate
            # being too tight, and the only way to tell is the distance the
            # gate was applied to. VR task B recorded NO GRASP AT ALL while its
            # arm visibly travelled to the part and stopped beside it, and
            # nothing in the dump could say by how much it missed.
            #
            # TWO numbers, because one does not separate the cases:
            #   min_pad_obj_m         did the pads EVER get near the object
            #   min_pad_obj_closed_m  where were they WHEN THE FINGERS CLOSED
            # A path that lands short has both large. A gate that is too tight
            # has the closed distance just over GRASP_NEAR_M. A gripper that
            # never closes has the second one absent entirely.
            if d is not None:
                if d < it.get("min_d", 1e9):
                    it["min_d"] = d
                    it["min_d_knuckle"] = k
                if closed and d < it.get("min_d_closed", 1e9):
                    it["min_d_closed"] = d
            near = d is not None and d <= GRASP_NEAR_M
            # ORIENTATION GATE. An object at an angle needs a gripper turned
            # to match; distance alone called that a grasp.
            # THE ITEM'S OWN ARM, not `_arm`. `_arm` is the travel loop's
            # variable and Python leaks it, so it is always "right" by the
            # time this runs: every left-arm item was having its yaw error
            # measured against the RIGHT gripper's closing axis. Silent,
            # because the check only reports.
            axis = self._grip_axis(arm) if hasattr(self, "_grip_axis") else None
            yerr = yaw_error_deg(axis, it.get("yaw_deg", 0.0),
                                 it.get("symmetry_deg", 90.0))
            aligned = (yerr is None) or (yerr <= GRASP_YAW_TOL_DEG)
            if yerr is not None:
                it["max_yaw_err_deg"] = max(it.get("max_yaw_err_deg", 0.0),
                                            yerr)
                if near and closed and not aligned:
                    # LOUD, and only when it actually cost a grasp: the pads
                    # were on the object, the fingers closed, and the grasp
                    # was refused for ORIENTATION. Without this the operator
                    # sees "no grasp" and looks at the path.
                    self._yaw_refusals = getattr(self, "_yaw_refusals", 0) + 1
                    if self._yaw_refusals in (1, 10) or \
                            self._yaw_refusals % 100 == 0:
                        self.get_logger().warn(
                            "GRASP REFUSED ON ORIENTATION: %s is at %.0f deg "
                            "and the gripper is %.0f deg off it (limit %.0f). "
                            "The pads are on the object; the hand is turned "
                            "the wrong way."
                            % (k, it.get("yaw_deg", 0.0), yerr,
                               GRASP_YAW_TOL_DEG))
            # REPORTS, DOES NOT GATE. Making attachment conditional on
            # alignment refused EVERY grasp in a live sweep -- "NO GRASP
            # RECORDED at all" across four clips -- because the pinned wrist
            # sits well off square and a 20 deg tolerance rejects it. The
            # point of the check was to make a rotated object VISIBLE, and
            # max_yaw_err_deg in scene_events.json plus the warning do that
            # without deciding whether the object attaches. A check that
            # silently stops the task it measures is worse than the silence
            # it replaced.
            on = closed and (it["held"] or near)
            if self.t0 is None:
                self.t0 = self.get_clock().now().nanoseconds * 1e-9
                self.t0_wall = time.time()
            now = self.get_clock().now().nanoseconds * 1e-9 - self.t0
            if on and not it["held"]:
                # WALL TIME ON EVERY EVENT. Without it there is no common
                # clock between this node and the ffmpeg grab, and B/S1
                # recorded a GRASPED at t=44.1 s inside a 29.1 s clip -- the
                # grasp happened after the video ended and nothing noticed,
                # because the two timestamps were never comparable.
                self.events.append(dict(t=round(now, 2), wall=time.time(),
                                        ev="GRASPED",
                                        item=name, arm=arm,
                                        knuckle=round(k or -1, 4),
                                        needed=round(0.90 * rr.grip_for(
                                            it["width_mm"]), 4),
                                        at=[round(v, 4) for v in it["pos"]]))
                it["carried"] = 0.0
            if on and g is not None:
                prev = it.get("last_held")
                if prev is not None:
                    it["carried"] = it.get("carried", 0.0) + math.dist(prev, g)
                it["last_held"] = list(g)
                # ATTACHED at the finger PADS -- _grip() already returns that
                # point, offset along the tool axis rather than in world z.
                it["pos"] = list(g)
                it["held"] = True
            elif it["held"] and not on:
                # RELEASED: left where it was put, never snapped back.
                it["held"] = False
                it["placed"] = True
                it["last_held"] = None
                self.events.append(dict(t=round(now, 2), wall=time.time(),
                                        ev="RELEASED",
                                        item=name, arm=arm,
                                        carried_m=round(it.get("carried", 0.0), 4),
                                        at=[round(v, 4) for v in it["pos"]]))
            add(Marker.CUBE, it["pos"], it["size"], it["col"], ns="item")
            # a thin outline so the object is legible against the table
            add(Marker.CUBE, it["pos"],
                tuple(s * 1.06 for s in it["size"]),
                (it["col"][0], it["col"][1], it["col"][2], 0.25), ns="item")
        self.pub.publish(A)


def _carry_summary(series):
    """T2's coupling metrics, derived from the series and nothing else.

    Returns None for a task with no carry, which is NOT the same as a carry
    that measured zero -- `by_design.py` exists because those two have been
    rendered identically before.

    `time_above_fail_tilt_s` is integrated from the sample spacing rather
    than counted in samples, so a dropped frame does not read as a shorter
    excursion.
    """
    if not series:
        return None
    # SEPARATION IS MEASURED FROM THE FIRST TICK, WHICH INCLUDES THE APPROACH.
    #
    # Read the sep_err figures with that in mind: the arms start at home,
    # about 1.46 m apart, so the series opens with a ~700 mm "error" that is
    # the arms not yet being on the tray rather than anything about the
    # carry. Measured across the first full sweep: sep_err_max_mm is 700.3 in
    # four of five modes, which is the same number every time because it is
    # the START pose, not a carry event.
    #
    # TILT DOES NOT HAVE THIS PROBLEM -- it is an angle between two grippers
    # and is meaningful whenever both exist -- so the tilt figures below are
    # usable as they stand and the separation figures are not, until the
    # series is gated on the carry phase. Naming it here rather than quoting
    # 700 mm as if it described the carry.
    tilts = [abs(r["tilt_deg"]) for r in series]
    seps = [r["sep_err_mm"] for r in series]
    thr = TSK.TASK_B["fail_tilt_deg"]
    above = 0.0
    for i, r in enumerate(series):
        if abs(r["tilt_deg"]) <= thr:
            continue
        dt = (series[i]["t"] - series[i - 1]["t"]) if i else 0.0
        above += max(0.0, dt)
    return dict(
        samples=len(series),
        span_s=round(series[-1]["t"] - series[0]["t"], 2),
        tilt_rms_deg=round(
            math.sqrt(sum(t * t for t in tilts) / len(tilts)), 3),
        tilt_max_deg=round(max(tilts), 3),
        fail_tilt_deg=thr,
        time_above_fail_tilt_s=round(above, 2),
        sep_err_max_mm=round(max(seps, key=abs), 1),
        sep_err_rms_mm=round(
            math.sqrt(sum(s * s for s in seps) / len(seps)), 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True,
                    choices=["a", "b", "c", "t0", "t1", "t1s2", "t2", "t3",
                             "d1", "d2", "d3"])
    ap.add_argument("--out", default=None)
    # THE SEED THE SCENE DRAWS, and it has to be the seed the task RUNS.
    # t1s2's cubes are a random draw; the scene built one at a hardcoded
    # T0_CLIP_SEED while run_abc built another from --seed, so the picture
    # and the waypoints could describe different layouts and every distance
    # measured between them would be nonsense. One seed, passed to both.
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rclpy.init()
    n = Scene(a.task, a.out, seed=a.seed)
    # APPLY THE COLLISION SCENE BEFORE THE EXECUTOR STARTS.
    #
    # It was applied from inside the 10 Hz tick, which spins to wait on the
    # service response -- inside a callback the executor is already running,
    # so it raised "Executor is already spinning" and killed the node. The
    # furniture is static and needs no TF, so it belongs here, where a manual
    # spin loop is the only one running.
    n._publish_scene()
    import json
    import signal

    def dump(*_):
        if n.out:
            summary = []
            for name, it in n.items.items():
                summary.append(dict(item=name, arm=it["arm"],
                                    width_mm=it["width_mm"],
                                    carried_m=round(it.get("carried", 0.0), 4),
                                    final=[round(v, 4) for v in it["pos"]],
                                    still_held=bool(it["held"]),
                                    # THE DIAGNOSTIC PAIR. See the comment at
                                    # the grasp test: these are what make a
                                    # refused grasp attributable instead of
                                    # merely absent.
                                    min_pad_obj_m=(
                                        round(it["min_d"], 4)
                                        if "min_d" in it else None),
                                    min_pad_obj_knuckle=(
                                        round(it["min_d_knuckle"], 4)
                                        if it.get("min_d_knuckle") is not None
                                        else None),
                                    min_pad_obj_closed_m=(
                                        round(it["min_d_closed"], 4)
                                        if "min_d_closed" in it else None),
                                    grasp_near_m=GRASP_NEAR_M))
            json.dump(dict(task=n.task, events=n.events, items=summary,
                           # WHAT THE SCENE DREW that is not a graspable item.
                           # The task's SUBJECT can be a fixture -- T0's
                           # spheres, T2's ball -- and a check that reads only
                           # `items` cannot tell a missing subject from a task
                           # that never had one.
                           fixtures=sorted(getattr(n, "fixtures", [])),
                           # WHAT THE SCENE PUT IN THE PLANNING SCENE, by
                           # name, so "T0 has no bench" is a fact in the file
                           # and not a claim in a commit message.
                           furniture=[f[0] for f in furniture_boxes(n.task)],
                           # HOW FAR EACH ARM ACTUALLY TRAVELLED, tf2 path
                           # integral over the clip, and the straight-line
                           # net. A few centimetres of travel means the task
                           # geometry is too tight to see.
                           ee_samples=getattr(n, "ee_samples", {}),
                           ee_metric_caveat=(
                               "ee_travel_m is a sum of |dp| over TICKS and "
                               "ee_net_m is anchored on the first pose this "
                               "node saw, so both depend on the observation "
                               "window. Compare only against runs with a "
                               "comparable ee_samples, and see findings.md "
                               "2026-08-15 finding 3."),
                           ee_travel_m={a2: round(v, 4) for a2, v
                                        in getattr(n, "ee_travel",
                                                   {}).items()},
                           ee_net_m={a2: (round(math.dist(
                               n.ee_first[a2], n.ee_track[a2]), 4)
                               if getattr(n, "ee_track", {}).get(a2) else None)
                               for a2 in getattr(n, "ee_first", {})},
                           t0_wall=getattr(n, "t0_wall", None),
                           # THE CARRY, SAMPLE BY SAMPLE. tasks.TASK_B asked
                           # for this from the day it was written; until now
                           # nothing wrote it. The summary is derived from
                           # the series here rather than accumulated during
                           # the run, so the two cannot disagree.
                           carry_series=getattr(n, "carry_series", []),
                           carry_summary=_carry_summary(
                               getattr(n, "carry_series", [])),
                           # The wrist->pad offset the whole scene was shifted
                           # by, so a consumer comparing against a declared
                           # EE-frame target can apply the same shift instead
                           # of re-deriving it.
                           pad_off=getattr(n, "pad_off", None),
                           pad_off_by_arm=getattr(n, "pad_off_by_arm", None)),
                      open(n.out, "w"), indent=2)
        try:
            remove_furniture(n)
        except Exception:                                      # noqa: BLE001
            pass
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, dump)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dump()
        except Exception:                                     # noqa: BLE001
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
