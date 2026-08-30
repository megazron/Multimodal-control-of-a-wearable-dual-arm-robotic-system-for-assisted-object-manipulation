#!/usr/bin/env python3
"""The RealSense D435i, across the room, as a ROS node.

    .venv_vision/bin/python scripts/srl_realsense_node.py

WHY THIS INTERPRETER.  pyrealsense2 lives in .venv_vision and rclpy comes from
the sourced ROS install; .venv_vision is the only python on this box that has
BOTH.  The system python3 has rclpy and no pyrealsense2, and installing
pyrealsense2 into it is not worth the risk -- CLAUDE.md records that adding a
package to .venv_vision once took real_calibration/check_all.py from 4/4 to
2/4, so environments here are left alone.

WHY THE FRAME RATE IS LOW AND THAT IS NOT A FAULT.  The camera reaches this
box over usbip, and usbip is the bottleneck: measured 6.7 fps for 640x480
depth + colour where the device itself does 30.  It is a SCENE camera -- it
answers "what is on the table and roughly where", which does not move -- so
6.7 Hz is ample.  The number is published in the status string rather than
left to be discovered, because a rate that silently halves is the signature
of a camera about to drop off the bus.

INTRINSICS ARE READ FROM THE DEVICE, never assumed.  Measured on this unit:
depth fx=382.72 cx=321.58 cy=240.60, colour fx=603.02 cx=318.87 cy=231.22.
The colour principal point is 8.8 px above centre, which is small but real.

DEPTH IS ALIGNED TO COLOUR before publishing, so a pixel in the colour image
and the same pixel in the depth image are the same ray.  Without that the two
sensors are 15 mm apart and every detection is thrown by the disparity, which
at 3 m is small but at the 0.5 m end of the workspace is not.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

COLOR_FRAME = "scene_rs_color_frame"
DEPTH_FRAME = "scene_rs_color_frame"   # depth is ALIGNED to colour, so same frame


class RealSenseNode(Node):
    def __init__(self, width=640, height=480, fps=30,
                 depth_only=False):
        super().__init__("srl_realsense")
        import pyrealsense2 as rs
        self.rs = rs
        self.pub_c = self.create_publisher(Image, "/scene/rs/color/image_raw",
                                           qos_profile_sensor_data)
        self.pub_d = self.create_publisher(Image, "/scene/rs/depth/image_raw",
                                           qos_profile_sensor_data)
        self.pub_ck = self.create_publisher(CameraInfo, "/scene/rs/color/camera_info",
                                            qos_profile_sensor_data)
        self.pub_dk = self.create_publisher(CameraInfo, "/scene/rs/depth/camera_info",
                                            qos_profile_sensor_data)
        self.pub_s = self.create_publisher(String, "/scene/rs/status", 10)

        self.pipe = rs.pipeline()
        cfg = rs.config()
        # BANDWIDTH IS THE BINDING CONSTRAINT OVER usbip, not the sensor.
        #
        # Measured 2026-08-26: 640x480 depth+colour at 30 fps enumerated fine
        # and then delivered NOTHING -- 24 pipeline restarts in a row, "Frame
        # didn't arrive within 2000" every time. A camera needs isochronous
        # transfers and usbip does not carry them well. Depth alone at a lower
        # rate is roughly a quarter of the traffic, and depth is what the
        # geometry actually needs; colour is only used to name a colour.
        self.depth_only = depth_only
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        if not depth_only:
            cfg.enable_stream(rs.stream.color, width, height,
                              rs.format.bgr8, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.get_logger().info(
            "RealSense %s serial %s fw %s"
            % (dev.get_info(rs.camera_info.name),
               dev.get_info(rs.camera_info.serial_number),
               dev.get_info(rs.camera_info.firmware_version)))
        # metres per depth unit, read from the device rather than assumed 0.001
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.align = None if depth_only else rs.align(rs.stream.color)
        strm = (rs.stream.depth if depth_only else rs.stream.color)
        self.k_color = prof.get_stream(strm) \
            .as_video_stream_profile().get_intrinsics()
        self.get_logger().info(
            "colour K fx=%.2f fy=%.2f cx=%.2f cy=%.2f  depth_scale=%.6f m/unit"
            % (self.k_color.fx, self.k_color.fy, self.k_color.ppx,
               self.k_color.ppy, self.depth_scale))
        self._w, self._h, self._fps = width, height, fps
        self.n = 0
        self.t0 = time.time()
        self.create_timer(1.0 / max(fps, 1), self.tick)
        self.create_timer(2.0, self.report)

    def _info(self, k, stamp, frame):
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.width, m.height = k.width, k.height
        m.distortion_model = "plumb_bob"
        m.d = [float(x) for x in k.coeffs]
        m.k = [k.fx, 0.0, k.ppx, 0.0, k.fy, k.ppy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [k.fx, 0.0, k.ppx, 0.0, 0.0, k.fy, k.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def tick(self):
        try:
            fs = self.pipe.wait_for_frames(2000)
        except Exception as e:                       # noqa: BLE001
            # THE usbip LINK STALLS, AND IT STALLS SILENTLY.
            #
            # Measured three times on 2026-08-26: the device stays enumerated
            # (lsusb shows it, /dev/video* exist, usbipd says "Attached") and
            # frames simply stop. Without recovery this node then sat there
            # warning once every two seconds for ever, and the operator's
            # first sign was an empty picture in the window -- which looks
            # exactly like a camera that was never started.
            #
            # A restart of the pipeline recovers it without touching usbipd,
            # so it is tried first and the count is reported rather than
            # hidden: a camera that recovers ten times an hour is a camera
            # about to need re-attaching, and that is worth seeing coming.
            self._misses = getattr(self, "_misses", 0) + 1
            self.get_logger().warn(
                "no frames (%s) -- miss %d" % (e, self._misses))
            if self._misses >= 3:
                self._restart_pipeline()
            return
        if self.align is not None:
            fs = self.align.process(fs)
        df = fs.get_depth_frame()
        cf = None if self.depth_only else fs.get_color_frame()
        if df is None or (not self.depth_only and not cf):
            # A PARTIAL FRAME IS NOT A SUCCESS. The miss counter used to be
            # cleared as soon as wait_for_frames RETURNED, before checking
            # that the frames were usable -- so a stall that returns empty
            # frame sets reported "miss 1" for ever and the recovery at three
            # consecutive misses could never fire. Measured 2026-08-26: the
            # log read "miss 1" over and over while the camera published
            # nothing at all.
            self._misses = getattr(self, "_misses", 0) + 1
            if self._misses >= 3:
                self._restart_pipeline()
            return
        stamp = self.get_clock().now().to_msg()
        dep = np.asanyarray(df.get_data())
        col = (np.zeros((dep.shape[0], dep.shape[1], 3), np.uint8)
               if self.depth_only else np.asanyarray(cf.get_data()))
        # Publish depth in MILLIMETRES as 16UC1 -- the same units and encoding
        # the Kinova wrist camera uses, so one reader handles both.
        if abs(self.depth_scale - 0.001) > 1e-6:
            dep = (dep.astype(np.float32) * self.depth_scale * 1000.0
                   ).astype(np.uint16)

        mi = Image()
        mi.header.stamp, mi.header.frame_id = stamp, COLOR_FRAME
        mi.height, mi.width = col.shape[0], col.shape[1]
        mi.encoding, mi.is_bigendian, mi.step = "bgr8", 0, col.shape[1] * 3
        mi.data = col.tobytes()
        self.pub_c.publish(mi)

        md = Image()
        md.header.stamp, md.header.frame_id = stamp, DEPTH_FRAME
        md.height, md.width = dep.shape[0], dep.shape[1]
        md.encoding, md.is_bigendian, md.step = "16UC1", 0, dep.shape[1] * 2
        md.data = dep.astype(np.uint16).tobytes()
        self.pub_d.publish(md)

        ci = self._info(self.k_color, stamp, COLOR_FRAME)
        self.pub_ck.publish(ci)
        self.pub_dk.publish(ci)          # aligned, so the colour K is the depth K
        self.n += 1
        self._misses = 0                 # cleared only on a PUBLISHED frame

    def _restart_pipeline(self):
        """Stop and restart the RealSense pipeline in place.

        Recovers the common usbip stall. If it CANNOT recover, it says so and
        names the fix rather than retrying for ever -- the device has to be
        detached and re-attached, and no amount of librealsense will do that.
        """
        rs = self.rs
        self._misses = 0
        self._restarts = getattr(self, "_restarts", 0) + 1
        self.get_logger().warn("restarting the RealSense pipeline "
                               "(restart %d)" % self._restarts)
        try:
            self.pipe.stop()
        except Exception:                            # noqa: BLE001
            pass
        try:
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, self._w, self._h,
                              rs.format.z16, self._fps)
            if not self.depth_only:
                cfg.enable_stream(rs.stream.color, self._w, self._h,
                                  rs.format.bgr8, self._fps)
            self.pipe = rs.pipeline()
            self.pipe.start(cfg)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("RealSense pipeline restarted")
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(
                "could not restart the RealSense (%s). The usbip link is "
                "wedged: on the WINDOWS side run\n"
                "    usbipd detach --busid 3-3\n"
                "    usbipd attach --wsl --busid 3-3\n"
                "No amount of retrying here will fix that." % e)

    def report(self):
        dt = time.time() - self.t0
        self.pub_s.publish(String(
            data='{"frames": %d, "hz": %.2f, "restarts": %d, "misses": %d}'
                 % (self.n, self.n / max(dt, 1e-6),
                    getattr(self, "_restarts", 0), getattr(self, "_misses", 0))))

    def destroy_node(self):
        try:
            self.pipe.stop()
        except Exception:                            # noqa: BLE001
            pass
        super().destroy_node()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--depth-only", action="store_true")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()
    rclpy.init()
    try:
        node = RealSenseNode(a.width, a.height, a.fps, a.depth_only)
    except Exception as e:                           # noqa: BLE001
        print("RealSense would not start: %s\n"
              "  - is it attached?  usbipd attach --wsl --busid 3-3\n"
              "  - is another process holding it?  only one client is allowed"
              % e, file=sys.stderr)
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
