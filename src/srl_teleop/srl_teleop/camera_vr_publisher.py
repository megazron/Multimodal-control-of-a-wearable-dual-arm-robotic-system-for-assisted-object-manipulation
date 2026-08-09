#!/usr/bin/env python3
"""Relay the wrist cameras into the VR overlay, with the same staleness rule.

    ros2 run srl_teleop camera_vr_publisher

Publishes /vr_camera_state as JSON, and JPEG frames on
/vr_camera_<arm>/compressed for the headset overlay to draw.

THE STALENESS RULE IS THE SAME ONE THE GUI USES, and it lives in
camera_relay.ChannelState so the two cannot drift apart. A relay that paints
the last frame it received cannot be told from a live view of a still scene,
and in a headset the operator has no other window onto the workspace at all,
so the consequence is worse here than on the desktop.

When a channel is not live this publishes the STATE and no frame. It never
republishes an old frame, because a consumer receiving a frame reasonably
concludes there was something to send.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from srl_teleop import camera_relay as cr

ARMS = ("left", "right")


class CameraVRPublisher(Node):

    def __init__(self):
        super().__init__("camera_vr_publisher")
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("max_hz", 15.0)
        self.st = {a: cr.ChannelState() for a in ARMS}
        self.last_pub = {a: 0.0 for a in ARMS}
        self.pub = {a: self.create_publisher(
            CompressedImage, "/vr_camera_%s/compressed" % a, 2) for a in ARMS}
        self.state_pub = self.create_publisher(String, "/vr_camera_state", 10)
        for a in ARMS:
            for topic in ("/%s_wrist_camera/image_raw" % a,
                          "/wrist_mounted_camera/%s/image" % a):
                self.create_subscription(
                    Image, topic, (lambda m, arm=a: self.on_img(arm, m)),
                    qos_profile_sensor_data)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            "wrist camera relay to VR up. Subscribes only; the vision driver "
            "owns the device.")

    def on_img(self, arm, msg):
        self.st[arm].on_frame(msg.width, msg.height, msg.encoding)
        now = self.get_clock().now().nanoseconds * 1e-9
        cap = 1.0 / max(1e-3, float(self.get_parameter("max_hz").value))
        if now - self.last_pub[arm] < cap:
            return                       # rate cap: the link is the scarce part
        self.last_pub[arm] = now
        try:
            import cv2
            import numpy as np
            ch = 1 if msg.encoding == "mono8" else 3
            a = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, ch)
            if msg.encoding == "rgb8":
                a = a[:, :, ::-1]
            ok, enc = cv2.imencode(
                ".jpg", a, [int(cv2.IMWRITE_JPEG_QUALITY),
                            int(self.get_parameter("jpeg_quality").value)])
            if not ok:
                return
            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = enc.tobytes()
            self.pub[arm].publish(out)
        except Exception as e:                                # noqa: BLE001
            self.get_logger().warn("encode failed on %s: %r" % (arm, e),
                                   throttle_duration_sec=5.0)

    def tick(self):
        m = String()
        m.data = json.dumps({a: dict(state=self.st[a].state(),
                                     caption=self.st[a].caption(),
                                     hz=round(self.st[a].hz(), 2),
                                     show=self.st[a].show_image())
                             for a in ARMS})
        self.state_pub.publish(m)


def main(argv=None):
    rclpy.init(args=argv)
    n = CameraVRPublisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
