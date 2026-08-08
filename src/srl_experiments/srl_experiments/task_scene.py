#!/usr/bin/env python3
"""The experiment scene as MoveIt COLLISION OBJECTS, with grasp attachment.

    ros2 run srl_experiments task_scene --ros-args -p task:=t2

Every bimanual protocol specifies its objects in the `world` frame, measured
once with a rule and stored -- NO APRILTAGS. This node turns that table of
numbers into a planning scene so the arm plans AROUND the table and the
objects instead of through them, and so a grasped object becomes part of the
arm for collision purposes.

WHY COLLISION OBJECTS AND NOT MARKERS
-------------------------------------
An RViz marker is decoration: `avoid_collisions=True` cannot see it, so the
arm sweeps straight through a table that looks solid on screen. Rehearsing a
task against decoration teaches the operator a motion that will collide on
the real rig, which is worse than not rehearsing.

ATTACH ON GRASP, DETACH ON RELEASE
----------------------------------
A held object is attached to the gripper link, so it moves with the arm and
is checked against the wearer and the scene. Without this a carried block
passes through the operator's chest and nothing objects.

The attach trigger is a gripper CLOSING TRANSITION into the "holding" band,
not merely a closed gripper -- `bimanual_metrics.gripper_state()` calls a
fully-closed gripper `free_air`, because closing on nothing is not a grasp.
That distinction already cost this project a pilot: the mock gripper boots at
0.79 rad, past any naive "closed" threshold, so every trial after the first
logged an instant grasp.
"""
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

# Robotiq 2F-85 driven knuckle. Same constants as bimanual_metrics, restated
# here rather than imported so this node has no dependency on the analysis
# code path -- they are asserted equal by test_task_scene.
GRIPPER_OPEN_RAD = 0.10
GRIPPER_FREE_AIR_RAD = 0.74

KNUCKLE = "%s_robotiq_85_left_knuckle_joint"
GRIP_LINK = "%s_robotiq_85_left_finger_link"
# Everything on the hand must be allowed to touch the held object.
TOUCH_LINKS = [
    "%s_robotiq_85_%s" % (a, p)
    for a in ("left", "right")
    for p in ("base_link", "left_knuckle_link", "right_knuckle_link",
              "left_finger_link", "right_finger_link",
              "left_inner_knuckle_link", "right_inner_knuckle_link",
              "left_finger_tip_link", "right_finger_tip_link")
]


def layout_path(task):
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "experiments", "bimanual"))
    for d in sorted(os.listdir(root)):
        if d.startswith(task + "_") or d == task:
            return os.path.join(root, d, "layout.yaml")
    raise FileNotFoundError("no layout for task %r under %s" % (task, root))


def box(name, xyz, size, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.BOX
    sp.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


def cyl(name, xyz, radius, height, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.CYLINDER
    sp.dimensions = [float(height), float(radius)]
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


def sphere(name, xyz, radius, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    sp = SolidPrimitive()
    sp.type = SolidPrimitive.SPHERE
    sp.dimensions = [float(radius)]
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


def build(spec):
    """layout dict -> ([CollisionObject], {name: grasp xyz})."""
    objs, grasps = [], {}
    t = spec["table"]
    objs.append(box("table", [t["x"], t["y"], t["z"] - t["thickness"] / 2.0],
                    [t["size_x"], t["size_y"], t["thickness"]]))
    for name, o in (spec.get("objects") or {}).items():
        shape = o.get("shape", "box")
        xyz = [o["x"], o["y"], o["z"]]
        if shape == "box":
            objs.append(box(name, xyz, o["size"]))
        elif shape == "cylinder":
            objs.append(cyl(name, xyz, o["radius"], o["height"]))
        elif shape == "sphere":
            objs.append(sphere(name, xyz, o["radius"]))
        else:
            raise ValueError("unknown shape %r for %r" % (shape, name))
        if o.get("graspable"):
            grasps[name] = o.get("grasp", xyz)
    return objs, grasps


class TaskScene(Node):
    def __init__(self):
        super().__init__("task_scene")
        self.declare_parameter("task", "t2")
        self.declare_parameter("publish_hz", 1.0)
        self.task = str(self.get_parameter("task").value)
        self.spec = yaml.safe_load(open(layout_path(self.task)))
        self.objs, self.grasps = build(self.spec)

        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.state_pub = self.create_publisher(String, "/task_scene_state", 10)
        self.create_subscription(JointState, "/joint_states", self._js, 20)

        self.knuckle = {a: None for a in ("left", "right")}
        self.prev = {a: None for a in ("left", "right")}
        self.attached = {a: None for a in ("left", "right")}
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value),
                          self._tick)
        self.get_logger().info(
            "task_scene: %s -- table top z=%.3f, %d object(s): %s"
            % (self.task, self.spec["table"]["z"], len(self.objs) - 1,
               ", ".join(sorted((self.spec.get("objects") or {}))) or "none"))
        self._publish_all()

    # -------------------------------------------------------------- scene
    def _publish_all(self):
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = list(self.objs)
        self.pub.publish(ps)

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        for a in ("left", "right"):
            v = d.get(KNUCKLE % a)
            if v is not None:
                self.knuckle[a] = float(v)

    def _holding(self, v):
        """Mirrors bimanual_metrics.gripper_state: a gripper closed all the
        way is `free_air` and holds NOTHING."""
        return v is not None and GRIPPER_OPEN_RAD <= v < GRIPPER_FREE_AIR_RAD

    def _nearest_graspable(self, arm):
        # Placeholder for a TF lookup; the scene knows only nominal poses, so
        # attach the graspable whose nominal point is nearest this arm's side.
        if not self.grasps:
            return None
        side = 1.0 if arm == "left" else -1.0
        best, bd = None, 1e9
        for name, g in self.grasps.items():
            d = abs(g[0] - side * abs(g[0]))
            if d < bd:
                best, bd = name, d
        return best

    def attach(self, arm, name):
        aco = AttachedCollisionObject()
        aco.link_name = GRIP_LINK % arm
        aco.object.id = name
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = list(TOUCH_LINKS)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        # Remove from the world in the same diff, or the object exists twice
        # and instantly self-collides.
        rm = CollisionObject()
        rm.id = name
        rm.header.frame_id = "world"
        rm.operation = CollisionObject.REMOVE
        ps.world.collision_objects = [rm]
        self.pub.publish(ps)
        self.attached[arm] = name
        self.get_logger().info(
            "[SCENE] %s arm GRASPED %r -- attached to %s, now moves with the "
            "arm and is collision-checked against the wearer"
            % (arm, name, GRIP_LINK % arm))

    def detach(self, arm):
        name = self.attached.get(arm)
        if not name:
            return
        aco = AttachedCollisionObject()
        aco.link_name = GRIP_LINK % arm
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        self.pub.publish(ps)
        self.attached[arm] = None
        # Put it back in the world where it nominally lives.
        self._publish_all()
        self.get_logger().info("[SCENE] %s arm RELEASED %r" % (arm, name))

    def _tick(self):
        for a in ("left", "right"):
            v = self.knuckle[a]
            now = self._holding(v)
            was = self.prev[a]
            # CLOSING TRANSITION into the holding band, not a closed gripper.
            if now and was is False and self.attached[a] is None:
                obj = self._nearest_graspable(a)
                if obj:
                    self.attach(a, obj)
            elif not now and self.attached[a] is not None:
                self.detach(a)
            self.prev[a] = now
        self._publish_all()
        m = String()
        m.data = "task=%s attached=%s" % (
            self.task, {k: v for k, v in self.attached.items() if v})
        self.state_pub.publish(m)


def main(argv=None):
    rclpy.init(args=argv)
    n = TaskScene()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
