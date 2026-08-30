#!/usr/bin/env python3
"""The USB webcam across the room, as a ROS node.

    python3 scripts/srl_scene_camera_node.py [--index N]

MJPG IS MANDATORY, NOT A PREFERENCE.  Over usbip the default uncompressed
YUYV negotiates happily and then delivers NOTHING: the device opens, every
call succeeds, and read() returns False for ever.  srl_cameras.py records the
measurement -- MJPG 640x480 -> 24.9 fps, YUYV -> no frames at all -- and this
node goes through SceneCamera so it cannot get that wrong independently.

THE DEVICE INDEX IS PROBED, NOT ASSUMED.  A RealSense on the same bus presents
six /dev/video* nodes, several of which open cleanly and return nothing for
ever.  probe_scene_camera() reads a frame rather than opening a handle, so the
node picks the device that actually delivers pixels.  Measured here: the USB
webcam is /dev/video0 and the RealSense holds 2 through 7.

THERE IS NO CALIBRATION FOR THIS CAMERA, and this node does not invent one.
It publishes colour only.  Anything needing a metric answer from the scene
uses the RealSense, which carries intrinsics read from the device.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, "/home/gausms/kortex_ws/src/srl_perception")
from srl_perception.srl_cameras import CameraError, SceneCamera  # noqa: E402

FRAME = "scene_usb_frame"


class SceneCameraNode(Node):
    def __init__(self, index=None, width=640, height=480, hz=15.0):
        super().__init__("srl_scene_camera")
        self.width, self.height = width, height
        self.cam = SceneCamera(index=index, width=width, height=height).open()
        self.get_logger().info("scene camera on /dev/video%d at %dx%d"
                               % (self.cam.index, width, height))
        self.pub = self.create_publisher(Image, "/scene/usb/image_raw",
                                         qos_profile_sensor_data)
        self.pub_s = self.create_publisher(String, "/scene/usb/status", 10)
        self.n = 0
        self.t0 = time.time()
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(2.0, self.report)

    def tick(self):
        try:
            # warm=1: this is a timer, so the next call takes the next frame.
            # Draining the buffer here would throw away the rate.
            f = self.cam.read(warm=1)
            self._misses = 0
        except CameraError as e:
            # THE usbip LINK STALLS WITH THE DEVICE STILL PRESENT.
            #
            # Measured repeatedly on 2026-08-26: lsusb still lists it,
            # /dev/video* still exist, usbipd still says "Attached", and
            # read() returns nothing for ever. This node used to warn once
            # per frame and carry on returning nothing, so the window showed
            # an empty panel -- indistinguishable from a camera that was
            # never started.
            #
            # Reopening also RE-PROBES the device index, which matters
            # independently: a detach/attach renumbers the nodes, so the
            # webcam that was /dev/video0 comes back as /dev/video1 and an
            # index remembered from start-up is then simply wrong.
            self._misses = getattr(self, "_misses", 0) + 1
            self.get_logger().warn("%s -- miss %d" % (e, self._misses))
            if self._misses >= 3:
                self._reopen()
            return
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = FRAME
        m.height, m.width = f.shape[0], f.shape[1]
        m.encoding, m.is_bigendian, m.step = "bgr8", 0, f.shape[1] * 3
        m.data = f.tobytes()
        self.pub.publish(m)
        self.n += 1

    def _reopen(self):
        """Close and re-open the camera, re-probing which node it is on."""
        self._misses = 0
        self._reopens = getattr(self, "_reopens", 0) + 1
        self.get_logger().warn("reopening the scene camera (reopen %d)"
                               % self._reopens)
        try:
            self.cam.close()
        except Exception:                                     # noqa: BLE001
            pass
        try:
            # index=None re-probes by USB identity, so a renumbered device is
            # found again and the RealSense is still excluded.
            self.cam = SceneCamera(index=None, width=self.width,
                                   height=self.height).open()
            self.get_logger().info("scene camera back on /dev/video%d"
                                   % self.cam.index)
        except CameraError as e:
            self.get_logger().error(
                "could not reopen the scene camera (%s). The usbip link is "
                "wedged: on the WINDOWS side run\n"
                "    usbipd detach --busid 2-9\n"
                "    usbipd attach --wsl --busid 2-9" % e)

    def report(self):
        dt = time.time() - self.t0
        self.pub_s.publish(String(
            data='{"frames": %d, "hz": %.2f, "dev": %d, "reopens": %d}'
                 % (self.n, self.n / max(dt, 1e-6), self.cam.index,
                    getattr(self, "_reopens", 0))))

    def destroy_node(self):
        try:
            self.cam.close()
        except Exception:                            # noqa: BLE001
            pass
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--hz", type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    try:
        node = SceneCameraNode(index=a.index, hz=a.hz)
    except CameraError as e:
        print("scene camera would not start: %s" % e, file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
