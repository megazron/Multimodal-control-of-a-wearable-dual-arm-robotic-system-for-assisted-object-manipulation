#!/usr/bin/env python3
"""
perception.launch.py — both wrist cameras through the AprilTag detector,
fused into one world-frame object list.

    ros2 launch srl_perception perception.launch.py
    ros2 launch srl_perception perception.launch.py fallback:=true

`fallback:=true` additionally starts the colour/shape detector. Do that
deliberately: colour segmentation fails by being confidently wrong, which is
the one failure mode AprilTag does not have. See the package README.

`mock_camera:=true` stands the sim RGB-D publisher at the camera boundary so
the WHOLE pipeline runs: image -> detector -> tracker -> fingerprint, and
depth -> work surface. Until this existed there was no camera publisher in
sim at all, so nothing downstream of the camera had ever run end to end --
the nodes were tested one at a time against fixtures and the joins between
them never were.

WHAT THE MOCK DOES AND DOES NOT LICENCE, because this is exactly where a
number gets quoted that should not be. The geometry is CONSTRUCTED: frames,
intrinsics, deprojection, the world<-camera composition and the surface
height are all checkable against answers known in advance, and they are
checked. The APPEARANCE is rendered, so no detection rate, no accuracy and no
confidence figure from this path means anything. Detection at working
distance remains UNMEASURED and needs the real cameras.

`work_surface:=true` runs the depth-to-height estimator. Off by default
because it only says anything useful once a camera is looking at the work
region, which means the SCAN POSE -- from the home pose the wrist cameras do
not see the surface at all.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("tag_size_m", default_value="0.040"),
        DeclareLaunchArgument("family", default_value="36h11"),
        DeclareLaunchArgument("corner_refine", default_value="subpix",
                              choices=["none", "subpix", "apriltag"],
                              description="apriltag is 43x slower for 0.2 mm"),
        DeclareLaunchArgument("fallback", default_value="false",
                              description="also run the colour/shape fallback"),
        DeclareLaunchArgument("mock_camera", default_value="false",
                              description="stand the sim RGB-D publisher at "
                                          "the camera boundary; geometry is "
                                          "real, APPEARANCE IS NOT"),
        DeclareLaunchArgument("task", default_value="t1",
                              description="which scene the mock camera renders"),
        DeclareLaunchArgument("work_surface", default_value="false",
                              description="measure the table height from "
                                          "depth; needs the SCAN POSE"),
    ]
    nodes = []
    for arm in ("left", "right"):
        nodes.append(Node(
            package="srl_perception", executable="mock_rgbd_camera",
            name=f"mock_rgbd_camera_{arm}", output="screen",
            condition=IfCondition(LaunchConfiguration("mock_camera")),
            parameters=[{"arm": arm,
                         "task": LaunchConfiguration("task")}]))
        nodes.append(Node(
            package="srl_perception", executable="work_surface_node",
            name=f"work_surface_node_{arm}", output="screen",
            condition=IfCondition(LaunchConfiguration("work_surface")),
            parameters=[{"arm": arm}]))
        nodes.append(Node(
            package="srl_perception", executable="apriltag_detector",
            name=f"apriltag_detector_{arm}", output="screen",
            parameters=[{"arm": arm,
                         "family": LaunchConfiguration("family"),
                         "tag_size_m": LaunchConfiguration("tag_size_m"),
                         "corner_refine": LaunchConfiguration("corner_refine")}]))
        nodes.append(Node(
            package="srl_perception", executable="colour_shape_detector",
            name=f"colour_shape_detector_{arm}", output="screen",
            condition=IfCondition(LaunchConfiguration("fallback")),
            parameters=[{"arm": arm, "enabled": True}]))
    nodes.append(Node(package="srl_perception", executable="object_pose_tracker",
                      name="object_pose_tracker", output="screen"))
    return LaunchDescription(args + nodes)
