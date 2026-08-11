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


def _m(ns, i, typ, xyz, scale, col, frame="world"):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = col
    return m


def furniture_ids():
    """Names of every collision object this scene owns."""
    import clip_tasks as _ct
    w = _ct.A_BIN_D / 2.0
    return ["bench", "circuit_box", "bin_floor"] + [
        "bin_wall_%+.0f_%+.0f" % (dx * 100, dy * 100)
        for dx, dy in ((w, 0.0), (-w, 0.0), (0.0, w), (0.0, -w))]


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
                out["cube_%d" % i] = dict(
                    arm="left", width_mm=40,
                    pos=[cx, cy, MCT.T1_Z], size=(0.04,) * 3,
                    col=(BLUE if i in (0, 2) else GREEN), held=False,
                    graspable=(i == 0))
            return out
        if task == "t2":
            # ONE body held at two points 500 mm apart. It is drawn as a
            # single wide object because that is what it is -- drawing two
            # would show the coupling task as two independent objects.
            return {"tray": dict(arm="left", width_mm=30,
                                 pos=[0.0, CT.Y, 1.32],
                                 size=(0.56, 0.26, 0.02), col=ORANGE,
                                 held=True)}
        if task == "t3":
            return {"circuit_box": dict(arm="right", width_mm=110,
                                        pos=list(T3M.BOX_OBJ),
                                        size=T3M.BOX_SIZE, col=GREEN,
                                        held=False),
                    "multimeter": dict(arm="left", width_mm=30,
                                       pos=list(T3M.METER_OBJ),
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
    def _collision_furniture(self):
        """Bench, bin walls and circuit box as real CollisionObjects.

        The graspable object is deliberately NOT included: it is attached to
        the gripper as the hand closes, and a collision object sitting where
        the fingers must go would make every grasp pose infeasible. Its
        support is what has to be solid, not the thing being picked up.
        """
        out = []

        def box(name, xyz, size):
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            pr = SolidPrimitive()
            pr.type = SolidPrimitive.BOX
            pr.dimensions = [float(v) for v in size]
            co.primitives.append(pr)
            from geometry_msgs.msg import Pose
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = [float(v)
                                                           for v in xyz]
            ps.orientation.w = 1.0
            co.primitive_poses.append(ps)
            co.operation = CollisionObject.ADD
            out.append(co)

        yc = (CT.BENCH_NEAR_Y + CT.BENCH_FAR_Y) / 2.0
        yd = CT.BENCH_FAR_Y - CT.BENCH_NEAR_Y
        box("bench", [0.0, yc, CT.BENCH_TOP - CT.BENCH_THICK / 2.0],
            [2 * CT.BENCH_HALF_X, yd, CT.BENCH_THICK])
        box("circuit_box", CT.BOX_OBJ, [0.17, CT.BOX_D, CT.BOX_H])
        # The bin as four walls, so the block can be released INTO it rather
        # than onto a solid block of the same size.
        bx, by = CT.A_BIN_OBJ[0], CT.A_BIN_OBJ[1]
        bz = CT.BENCH_TOP + CT.A_BIN_H / 2.0
        w = CT.A_BIN_D / 2.0
        box("bin_floor", [bx, by, CT.BENCH_TOP + 0.01],
            [CT.A_BIN_D, CT.A_BIN_D, 0.02])
        for dx, dy in ((w, 0.0), (-w, 0.0), (0.0, w), (0.0, -w)):
            box("bin_wall_%+.0f_%+.0f" % (dx * 100, dy * 100),
                [bx + dx, by + dy, bz],
                [0.02 if dx else CT.A_BIN_D,
                 CT.A_BIN_D if dx else 0.02, CT.A_BIN_H])
        return out

    def _publish_scene(self):
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
        if not self.items:
            # T0 HAS NO OBJECTS. A task with nothing to grasp must draw
            # nothing -- not a placeholder, which would show an object in the
            # one task whose whole point is that there isn't one. Without this
            # guard the line below raises StopIteration on an empty dict.
            return
        if self.pad_off is None:
            arm = next(iter(self.items.values()))["arm"]
            off = self._pad_offset(arm)
            if off is None:
                return          # no TF yet -- draw nothing rather than draw
                                # the scene in the wrong place
            self.pad_off = off
            for it in self.items.values():
                it["pos"] = [it["pos"][i] + off[i] for i in range(3)]

        A = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        A.markers.append(d)
        i = 0

        def add(typ, xyz, scale, col, ns="scene"):
            nonlocal i
            A.markers.append(_m(ns, i, typ, xyz, scale, col))
            i += 1

        # ---- furniture, common to every task ---------------------------
        # DRAWN FROM THE SAME CONSTANTS AS THE COLLISION OBJECTS, so the
        # picture and the planner cannot disagree. Two descriptions of one
        # bench is how a scene comes to look solid and behave hollow.
        yc = (CT.BENCH_NEAR_Y + CT.BENCH_FAR_Y) / 2.0
        yd = CT.BENCH_FAR_Y - CT.BENCH_NEAR_Y
        add(Marker.CUBE, [0.0, yc, CT.BENCH_TOP - CT.BENCH_THICK / 2.0],
            (2 * CT.BENCH_HALF_X, yd, CT.BENCH_THICK), TAN)
        for sx in (-0.75, 0.75):
            add(Marker.CUBE, [sx, yc, (CT.BENCH_TOP - CT.BENCH_THICK) / 2.0],
                (0.05, 0.05, CT.BENCH_TOP - CT.BENCH_THICK), DARK)
        # the bin task A places into: floor plus four walls, on the bench
        bx, by = CT.A_BIN_OBJ[0], CT.A_BIN_OBJ[1]
        w = CT.A_BIN_D / 2.0
        add(Marker.CUBE, [bx, by, CT.BENCH_TOP + 0.01],
            (CT.A_BIN_D, CT.A_BIN_D, 0.02), TEAL)
        for dx, dy in ((w, 0), (-w, 0), (0, w), (0, -w)):
            add(Marker.CUBE,
                [bx + dx, by + dy, CT.BENCH_TOP + CT.A_BIN_H / 2.0],
                (0.02 if dx else CT.A_BIN_D,
                 CT.A_BIN_D if dx else 0.02, CT.A_BIN_H), TEAL)
        # the circuit box task C probes and task B places onto
        add(Marker.CUBE, CT.BOX_OBJ, (0.17, CT.BOX_D, CT.BOX_H), GREEN)

        # ---- the graspable object --------------------------------------
        for name, it in self.items.items():
            arm = it["arm"]
            k = self.knuck.get(arm)
            g = self._grip(arm)
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
    ap.add_argument("--task", required=True, choices=["a", "b", "c"])
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
                           t0_wall=getattr(n, "t0_wall", None),
                           # The wrist->pad offset the whole scene was shifted
                           # by, so a consumer comparing against a declared
                           # EE-frame target can apply the same shift instead
                           # of re-deriving it.
                           pad_off=getattr(n, "pad_off", None)),
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
