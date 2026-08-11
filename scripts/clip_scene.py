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

RED = (0.90, 0.15, 0.12, 1.0)
# task0 names its spheres by COLOUR ("go to the green one"), so the picture
# has to use those colours and not one colour per arm.
SPHERE_RGBA = {"red": RED, "green": GREEN, "blue": BLUE, "yellow": YELLOW}


def _m(ns, i, typ, xyz, scale, col, frame="world"):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation.w = 1.0
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
    if task == "t0":
        return out
    yc = (_ct.BENCH_NEAR_Y + _ct.BENCH_FAR_Y) / 2.0
    yd = _ct.BENCH_FAR_Y - _ct.BENCH_NEAR_Y
    out.append(("bench", [0.0, yc, _ct.BENCH_TOP - _ct.BENCH_THICK / 2.0],
                [2 * _ct.BENCH_HALF_X, yd, _ct.BENCH_THICK], TAN))
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
    if not SUPPORTS_ENABLED:
        return out
    if task == "t1":
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

    def __init__(self, task, out=None):
        super().__init__("clip_scene")
        self.task = task
        self.out = out
        # THE EVIDENCE LOG. "Did the task complete" must be a measurement, not
        # a judgement made by squinting at a frame. This records when the
        # fingers actually reached the object's width, how far the object
        # travelled WHILE HELD, and where it was let go -- so a clip can be
        # scored as grasped-and-carried-and-delivered rather than as the arm
        # having moved somewhere near an object.
        self.events = []
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
        if task == "t0":
            # NO OBJECTS. T0 is reaching only, which is exactly why it runs in
            # every mode including any that cannot grasp.
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
                    arm="left", width_mm=40,
                    pos=CT.ee_for([cx, cy, MCT.T1_Z]), size=(0.04,) * 3,
                    col=(BLUE if i in (0, 2) else GREEN), held=False,
                    # ALL FOUR are picked, one after another: T1's schedule is
                    # four closes and four opens. The flag stays because
                    # tick() now honours it, and a future fixtured item will
                    # need it.
                    graspable=True)
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
                                        pos=CT.ee_for(T3M.BOX_OBJ),
                                        size=T3M.BOX_SIZE, col=GREEN,
                                        held=False),
                    "multimeter": dict(arm="left", width_mm=30,
                                       pos=CT.ee_for(T3M.METER_OBJ),
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
        """Wrist -> finger-pad vector in world, from the live anchor pose."""
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
        for _arm in ("left", "right"):
            _p = self._ee(_arm)
            if _p is None:
                continue
            self.ee_first.setdefault(_arm, list(_p))
            _prev = self.ee_track[_arm]
            if _prev is not None:
                self.ee_travel[_arm] += math.dist(_prev, _p)
            self.ee_track[_arm] = list(_p)

        A = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        A.markers.append(d)
        i = 0

        def add(typ, xyz, scale, col, ns="scene"):
            nonlocal i
            A.markers.append(_m(ns, i, typ, xyz, scale, col))
            i += 1

        # ---- furniture, PER TASK ---------------------------------------
        # DRAWN FROM THE SAME TABLE AS THE COLLISION OBJECTS, so the picture
        # and the planner cannot disagree. Two descriptions of one bench is
        # how a scene comes to look solid and behave hollow.
        solids = furniture_boxes(self.task)
        for _name, _xyz, _size, _col in solids:
            add(Marker.CUBE, list(_xyz), tuple(_size), _col)
        if any(s[0] == "bench" for s in solids):
            yc = (CT.BENCH_NEAR_Y + CT.BENCH_FAR_Y) / 2.0
            for sx in (-0.75, 0.75):
                add(Marker.CUBE,
                    [sx, yc, (CT.BENCH_TOP - CT.BENCH_THICK) / 2.0],
                    (0.05, 0.05, CT.BENCH_TOP - CT.BENCH_THICK), DARK)

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
        elif self.task == "t1":
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
            import msc_clip_tasks as _MCT
            for pi, (px, py) in enumerate(_MCT.T1_PLANES):
                name = "plane_%s" % ("blue" if pi == 0 else "green")
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
            import task3 as _T3
            for mp in _T3.MEASUREMENT_POINTS:
                host = "multimeter" if mp["id"] == "P4" else "circuit_box"
                it = self.items.get(host)
                if it is None:
                    continue
                size = it["size"]
                dy = (size[1] / 2.0 if mp["face"] == "far" else -size[1] / 2.0)
                p3 = [it["pos"][0] + mp["offset_mm"][0] / 1000.0,
                      it["pos"][1] + dy,
                      it["pos"][2] + mp["offset_mm"][1] / 1000.0]
                add(Marker.SPHERE, p3, (0.014,) * 3,
                    (YELLOW if mp["served_by_initial_presentation"]
                     else RED), ns="measure")
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
                add(Marker.CUBE, mid, (round(span + 0.06, 4), dep, th),
                    TAN, ns="tray")
                # THE BALL. It is the failure indicator: the tray tilting past
                # 6.8 deg rolls it off, and without it a tilt has no visible
                # consequence at all. Drawn ON the tray and carried WITH it.
                bp = list(mid)
                bp[2] += th / 2.0 + CT_BALL_R
                add(Marker.SPHERE, bp, (2 * CT_BALL_R,) * 3, YELLOW, ns="ball")
                for nm in ("tray", "ball"):
                    if nm not in self.fixtures:
                        self.fixtures.append(nm)

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True, choices=["a", "b", "c", "t0", "t1", "t2", "t3"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    rclpy.init()
    n = Scene(a.task, a.out)
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
                           ee_travel_m={a2: round(v, 4) for a2, v
                                        in getattr(n, "ee_travel",
                                                   {}).items()},
                           ee_net_m={a2: (round(math.dist(
                               n.ee_first[a2], n.ee_track[a2]), 4)
                               if getattr(n, "ee_track", {}).get(a2) else None)
                               for a2 in getattr(n, "ee_first", {})},
                           t0_wall=getattr(n, "t0_wall", None),
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
