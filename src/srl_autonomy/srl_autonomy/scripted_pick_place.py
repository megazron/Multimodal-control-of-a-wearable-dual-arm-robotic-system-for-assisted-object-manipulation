#!/usr/bin/env python3
"""
pick_place_node.py — full pick-and-place using MoveIt (moveit_py)
====================================================================
Interactive: type a cube color, it picks that cube off the table and
places it at a fixed drop zone. Uses the SAME cube positions spawn_cubes.py
used (known ground truth, since there's no live camera yet). When vision
goes live, swap get_cube_pose() for a /located_object_pose subscription --
everything downstream (grasp/lift/place) stays identical.

Sequence per pick: pre-grasp -> approach -> close gripper -> attach cube
-> lift -> pre-place -> place -> open gripper -> detach cube -> retreat.

Run:
  ros2 launch srl_teleop pick_place.launch.py
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from moveit.planning import MoveItPy

# ══════════════════════════════════════════════════════════════════════════════
#  Must match spawn_cubes.py exactly (ground-truth cube positions, no camera yet)
# ══════════════════════════════════════════════════════════════════════════════
CUBE_SIZE = 0.04
CUBE_POSITIONS = {
    "red":   (-0.15, 0.40, 0.995),
    "green": ( 0.00, 0.40, 0.995),
    "blue":  ( 0.15, 0.40, 0.995),
}
PLACE_POSITION = (0.0, 0.55, 0.995)   # a clear drop zone on the same table

PRE_OFFSET_Z = 0.10   # how far above the cube/place point to pre-position

ARM_GROUP     = "left_arm"
GRIPPER_GROUP = "left_gripper"
EE_LINK       = "left_end_effector_link"
GRIPPER_OPEN  = "left_gripper_open"
GRIPPER_CLOSED = "left_gripper_closed"
TOUCH_LINKS = [
    "left_robotiq_85_left_finger_tip_link",
    "left_robotiq_85_right_finger_tip_link",
    "left_robotiq_85_left_finger_link",
    "left_robotiq_85_right_finger_link",
]


def make_pose(x, y, z):
    p = PoseStamped()
    p.header.frame_id = "world"
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.w = 1.0   # identity -- same orientation proven reachable earlier
    return p


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        self.get_logger().info("Starting MoveItPy...")
        self.moveit = MoveItPy(node_name="pick_place_moveit_py")
        self.arm = self.moveit.get_planning_component(ARM_GROUP)
        self.gripper = self.moveit.get_planning_component(GRIPPER_GROUP)
        self.get_logger().info("MoveItPy ready.")

        # Wait for a real planning_scene subscriber before we ever publish
        while self.scene_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for /planning_scene subscriber...")
            time.sleep(0.5)

        import threading
        threading.Thread(target=self.prompt_loop, daemon=True).start()

    # ── low-level helpers ─────────────────────────────────────────────────
    def move_arm_to(self, pose_stamped, label=""):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=EE_LINK)
        result = self.arm.plan()
        if not result:
            self.get_logger().error(f"Planning FAILED for '{label}'.")
            return False
        self.moveit.execute(result.trajectory, controllers=[])
        self.get_logger().info(f"Moved to '{label}'.")
        return True

    def set_gripper(self, state_name):
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name=state_name)
        result = self.gripper.plan()
        if not result:
            self.get_logger().error(f"Gripper planning FAILED for '{state_name}'.")
            return False
        self.moveit.execute(result.trajectory, controllers=[])
        self.get_logger().info(f"Gripper -> '{state_name}'.")
        return True

    def attach_cube(self, cube_id, x, y, z):
        scene = PlanningScene()
        scene.is_diff = True

        # Remove from the world (it's now rigidly part of the robot)
        remove_obj = CollisionObject()
        remove_obj.id = cube_id
        remove_obj.header.frame_id = "world"
        remove_obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove_obj)

        # Add as an attached object on the end effector
        attached = AttachedCollisionObject()
        attached.link_name = EE_LINK
        attached.touch_links = TOUCH_LINKS
        attached.object.id = cube_id
        attached.object.header.frame_id = EE_LINK
        attached.object.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [CUBE_SIZE] * 3
        pose = Pose()
        pose.orientation.w = 1.0   # cube is now defined relative to EE_LINK
        attached.object.primitives.append(box)
        attached.object.primitive_poses.append(pose)

        scene.robot_state.attached_collision_objects.append(attached)
        scene.robot_state.is_diff = True

        self.scene_pub.publish(scene)
        self.get_logger().info(f"Attached '{cube_id}' to {EE_LINK}.")

    def detach_cube(self, cube_id, x, y, z):
        scene = PlanningScene()
        scene.is_diff = True

        # Detach from the robot
        detach = AttachedCollisionObject()
        detach.link_name = EE_LINK
        detach.object.id = cube_id
        detach.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(detach)
        scene.robot_state.is_diff = True

        # Add back into the world at the new (place) position
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = cube_id
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [CUBE_SIZE] * 3
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj)

        self.scene_pub.publish(scene)
        self.get_logger().info(f"Detached '{cube_id}' at ({x}, {y}, {z}).")

    # ── the full pick+place sequence ────────────────────────────────────────
    def pick_and_place(self, color):
        if color not in CUBE_POSITIONS:
            print(f"Unknown color '{color}'. Options: {list(CUBE_POSITIONS)}")
            return

        cube_id = f"cube_{color}"
        cx, cy, cz = CUBE_POSITIONS[color]
        px, py, pz = PLACE_POSITION

        steps = [
            ("pre-grasp", lambda: self.move_arm_to(
                make_pose(cx, cy, cz + PRE_OFFSET_Z), "pre-grasp")),
            ("approach", lambda: self.move_arm_to(
                make_pose(cx, cy, cz), "grasp point")),
            ("close gripper", lambda: self.set_gripper(GRIPPER_CLOSED)),
            ("attach", lambda: self.attach_cube(cube_id, cx, cy, cz) or True),
            ("lift", lambda: self.move_arm_to(
                make_pose(cx, cy, cz + PRE_OFFSET_Z), "lift")),
            ("pre-place", lambda: self.move_arm_to(
                make_pose(px, py, pz + PRE_OFFSET_Z), "pre-place")),
            ("place", lambda: self.move_arm_to(
                make_pose(px, py, pz), "place point")),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN)),
            ("detach", lambda: self.detach_cube(cube_id, px, py, pz) or True),
            ("retreat", lambda: self.move_arm_to(
                make_pose(px, py, pz + PRE_OFFSET_Z), "retreat")),
        ]

        print(f"\n=== Picking '{color}' cube, placing at drop zone ===")
        for name, fn in steps:
            print(f"  -> {name} ...")
            ok = fn()
            if ok is False:
                print(f"  FAILED at step '{name}'. Aborting this pick.")
                return
        print(f"=== Done: '{color}' cube placed. ===\n")

    def prompt_loop(self):
        while rclpy.ok():
            color = input(
                "\nPick which cube? (red/green/blue, or 'quit'): "
            ).strip().lower()
            if color in ("quit", "exit"):
                return
            self.pick_and_place(color)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
