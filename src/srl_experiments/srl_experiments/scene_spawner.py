#!/usr/bin/env python3
"""
spawn_cubes.py — add a table + 3 colored cubes to the MoveIt planning scene
=============================================================================
Publishes a moveit_msgs/PlanningScene diff to /planning_scene: a table
(grey box) positioned in front of the wearer, with three colored cubes
resting on its surface, spread left-to-right for a natural pick-and-place
layout reachable by either arm.

COORDINATE FRAME (world/backpack): X = left/right, Y = forward/back
(+Y = front), Z = up/down. Table sits at +Y (in front of the torso).

Run:
  ros2 run srl_teleop spawn_cubes
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


CUBE_SIZE = 0.04   # 4 cm cubes

# Table: sits in front of the torso, waist/chest height.
TABLE_SIZE = (1.0, 0.6, 0.05)      # 100cm x 60cm x 5cm thick (real table size)
TABLE_CENTER = (0.0, 0.45, 0.95)   # in front of body, table-height
TABLE_TOP_Z = TABLE_CENTER[2] + TABLE_SIZE[2] / 2.0

# Cubes sit ON the table surface, spread across X (left/right),
# same Y as the table, centered in front of the body.
CUBE_Y = TABLE_CENTER[1]
CUBE_Z = TABLE_TOP_Z + CUBE_SIZE / 2.0
CUBES = [
    ("cube_red",   (1.0, 0.0, 0.0), (-0.15, CUBE_Y, CUBE_Z)),
    ("cube_green", (0.0, 1.0, 0.0), ( 0.00, CUBE_Y, CUBE_Z)),
    ("cube_blue",  (0.0, 0.0, 1.0), ( 0.15, CUBE_Y, CUBE_Z)),
]


def make_box(obj_id, frame_id, size_xyz, pos_xyz):
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = obj_id
    obj.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size_xyz)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pos_xyz
    pose.orientation.w = 1.0

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)
    return obj


class SpawnCubes(Node):
    def __init__(self):
        super().__init__("spawn_cubes")
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.publish_count = 0
        self.create_timer(0.5, self.try_spawn)

    def try_spawn(self):
        if self.publish_count >= 5:
            return

        n_subs = self.pub.get_subscription_count()
        if n_subs == 0:
            self.get_logger().info(
                "Waiting for a subscriber on /planning_scene "
                "(is demo.launch.py / move_group running?)...")
            return

        scene = PlanningScene()
        scene.is_diff = True

        # Table
        table = make_box("table", "world", TABLE_SIZE, TABLE_CENTER)
        scene.world.collision_objects.append(table)
        table_color = ObjectColor()
        table_color.id = "table"
        table_color.color = ColorRGBA(r=0.55, g=0.45, b=0.35, a=1.0)
        scene.object_colors.append(table_color)
        self.get_logger().info(f"Spawning table at {TABLE_CENTER}")

        # Cubes
        for cube_id, rgb, pos in CUBES:
            cube = make_box(cube_id, "world", (CUBE_SIZE,) * 3, pos)
            scene.world.collision_objects.append(cube)

            color = ObjectColor()
            color.id = cube_id
            color.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            scene.object_colors.append(color)
            self.get_logger().info(f"Spawning {cube_id} at {pos}")

        self.pub.publish(scene)
        self.publish_count += 1
        self.get_logger().info(
            f"Table + cubes published ({n_subs} subscriber(s), attempt "
            f"{self.publish_count}/5). Check RViz.")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnCubes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
