#!/usr/bin/env python3
"""FIRST-PERSON VIEW: a camera frame at the WEARER'S EYES, not a free camera.

    ros2 run srl_vr_teleop wearer_view

Publishes TF `head -> wearer_eyes` plus a CameraInfo, so RViz (or the headset
client) can render the sim from where the wearer's eyes actually are.

WHY THIS MATTERS AND IS NOT COSMETIC
------------------------------------
These are SUPERNUMERARY limbs bolted to a person's back. Whether they feel
like part of the operator or like a machine they are driving is the thing the
experiments measure. A third-person orbit camera silently answers that
question the wrong way: it shows the arms as external objects, and an
operator who has only ever seen them that way is not the operator the study
is about.

The offsets are anatomical, not tuned: eyes sit ~90 mm forward of the head
link's centre and ~60 mm above it, and the view direction is the wearer's
+y (this repo's forward), NOT ROS +x. Getting that wrong points the camera
out of the side of the wearer's head, which looks plausible in RViz until you
notice the arms are on the wrong side.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster


class WearerView(Node):
    def __init__(self):
        super().__init__("wearer_view")
        self.declare_parameter("eye_forward_m", 0.09)
        self.declare_parameter("eye_up_m", 0.06)
        self.declare_parameter("hfov_deg", 90.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.br = TransformBroadcaster(self)
        self.info = self.create_publisher(CameraInfo, "/wearer_view/camera_info", 5)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "wearer_view: TF head -> wearer_eyes (+%.0f mm forward, "
            "+%.0f mm up), looking along the wearer's +y."
            % (100 * float(self.get_parameter("eye_forward_m").value),
               100 * float(self.get_parameter("eye_up_m").value)))

    def _tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "head"
        t.child_frame_id = "wearer_eyes"
        t.transform.translation.x = 0.0
        t.transform.translation.y = float(
            self.get_parameter("eye_forward_m").value)
        t.transform.translation.z = float(self.get_parameter("eye_up_m").value)
        # Optical convention (z forward, x right, y down) aligned to the
        # wearer's forward (+y world). Rotation about x by -90 deg takes
        # world-forward +y onto optical +z.
        h = math.radians(-90.0) / 2.0
        t.transform.rotation.x = math.sin(h)
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(h)
        self.br.sendTransform(t)

        w = int(self.get_parameter("width").value)
        hgt = int(self.get_parameter("height").value)
        f = (w / 2.0) / math.tan(
            math.radians(float(self.get_parameter("hfov_deg").value)) / 2.0)
        ci = CameraInfo()
        ci.header.stamp = t.header.stamp
        ci.header.frame_id = "wearer_eyes"
        ci.width, ci.height = w, hgt
        ci.k = [f, 0.0, w / 2.0, 0.0, f, hgt / 2.0, 0.0, 0.0, 1.0]
        ci.p = [f, 0.0, w / 2.0, 0.0, 0.0, f, hgt / 2.0, 0.0,
                0.0, 0.0, 1.0, 0.0]
        self.info.publish(ci)


def main(argv=None):
    rclpy.init(args=argv)
    n = WearerView()
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
