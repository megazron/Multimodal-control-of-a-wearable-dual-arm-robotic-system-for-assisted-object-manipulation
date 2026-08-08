#!/usr/bin/env python3
"""
vlm_locate_node.py — LocateAnything VLM object localization (ROS2)
====================================================================
On-demand object localization: type what you want ("the red cup"),
the node runs LocateAnything-3B on the current camera frame, finds
the object's pixel box, looks up its depth, converts to a 3D point
in the camera frame, transforms it to the robot base frame via tf2,
and publishes it as a PoseStamped -- a pick target for MoveIt.

WHY ON-REQUEST, NOT CONTINUOUS:
  Detection takes ~1-3 minutes on CPU (measured on this hardware).
  This node only runs it when you ask, not per camera frame.

TWO INPUT MODES (--ros-args -p use_test_image:=true/false):
  TEST IMAGE : reads a static file each time (no camera needed --
               use this to validate the VLM + node logic right now)
  LIVE CAMERA: subscribes to color + depth + camera_info topics from
               kinova_vision (use once the real arm camera is up)

TOPIC NAMES ARE PARAMETERS -- the kinova_vision topic names haven't
been confirmed on a live camera yet in this project. Check with
`ros2 topic list` once the camera is running and adjust if needed.

Run (test image, no camera):
  ros2 run srl_teleop vlm_locate_node --ros-args \
    -p use_test_image:=true \
    -p test_image_path:=/path/to/image.jpg

Run (live camera, once connected):
  ros2 run srl_teleop vlm_locate_node --ros-args \
    -p use_test_image:=false \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/depth/image_raw \
    -p camera_info_topic:=/camera/color/camera_info \
    -p camera_frame:=left_camera_color_frame \
    -p target_frame:=world
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

import subprocess
import json
import tempfile
import os
import threading

try:
    from cv_bridge import CvBridge
    import cv2
    CV_OK = True
except ImportError:
    CV_OK = False


# ══════════════════════════════════════════════════════════════════════════════
LOCATE_CLI = os.path.expanduser(
    "~/locate-anything.cpp/build/examples/cli/locate-anything-cli")
LOCATE_MODEL = os.path.expanduser(
    "~/locate-anything.cpp/models/locate-anything-q8_0.gguf")
# ══════════════════════════════════════════════════════════════════════════════


class VlmLocateNode(Node):
    def __init__(self):
        super().__init__("vlm_locate_node")

        self.declare_parameter("use_test_image", True)
        self.declare_parameter("test_image_path", "")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "left_camera_color_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("mode", "slow")  # slow was fastest in benchmarks

        self.use_test_image = self.get_parameter("use_test_image").value
        self.test_image_path = self.get_parameter("test_image_path").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.decode_mode = self.get_parameter("mode").value

        if not CV_OK:
            self.get_logger().error(
                "cv_bridge/opencv not available -- "
                "sudo apt install ros-jazzy-cv-bridge python3-opencv")
            return

        self.bridge = CvBridge()
        self.latest_color = None   # numpy BGR image
        self.latest_depth = None   # numpy depth image (meters or mm, driver-dependent)
        self.latest_intrinsics = None  # (fx, fy, cx, cy)
        self.lock = threading.Lock()

        self.pub = self.create_publisher(PoseStamped, "/located_object_pose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if not self.use_test_image:
            self.create_subscription(Image, self.get_parameter("color_topic").value,
                                     self.on_color, 5)
            self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                     self.on_depth, 5)
            self.create_subscription(CameraInfo,
                                     self.get_parameter("camera_info_topic").value,
                                     self.on_camera_info, 5)
            self.get_logger().info(
                f"LIVE CAMERA mode. Waiting on "
                f"{self.get_parameter('color_topic').value}")
        else:
            self.get_logger().info(
                f"TEST IMAGE mode. Using: {self.test_image_path}")

        self.get_logger().info(
            "vlm_locate_node ready. Detection takes ~1-3 min on this CPU.")

        # Run the interactive prompt loop in a background thread so ROS
        # callbacks (camera subscriptions, tf) keep spinning normally.
        threading.Thread(target=self.prompt_loop, daemon=True).start()

    # ── Camera callbacks (live mode) ─────────────────────────────────────────
    def on_color(self, msg):
        with self.lock:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_depth(self, msg):
        with self.lock:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_camera_info(self, msg):
        with self.lock:
            k = msg.k  # row-major 3x3 intrinsic matrix
            self.latest_intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    # ── Interactive loop ──────────────────────────────────────────────────────
    def prompt_loop(self):
        while rclpy.ok():
            query = input(
                "\nWhat should the arm locate? "
                "(e.g. 'the red cup', or 'quit'): ").strip()
            if query.lower() in ("quit", "exit"):
                self.get_logger().info("Exiting locate prompt loop.")
                return
            if not query:
                continue
            self.run_locate(query)

    def run_locate(self, prompt_text):
        # ── get an image to run on ──
        if self.use_test_image:
            image_path = self.test_image_path
            if not image_path or not os.path.exists(image_path):
                print(f"  [ERROR] test_image_path not found: {image_path}")
                return
            depth_img = None
            intrinsics = None
        else:
            with self.lock:
                color = self.latest_color
                depth_img = self.latest_depth
                intrinsics = self.latest_intrinsics
            if color is None:
                print("  [ERROR] No color frame received yet from the camera topic.")
                return
            tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
            cv2.imwrite(tmp.name, color)
            image_path = tmp.name

        print(f"  Running LocateAnything on '{prompt_text}' "
              f"(this takes a while on CPU)...")

        try:
            result = subprocess.run(
                [LOCATE_CLI, "detect",
                 "--model", LOCATE_MODEL,
                 "--input", image_path,
                 "--prompt", prompt_text,
                 "--mode", self.decode_mode],
                capture_output=True, text=True, timeout=600,
            )
        except subprocess.TimeoutExpired:
            print("  [ERROR] Detection timed out (>10 min).")
            return
        except FileNotFoundError:
            print(f"  [ERROR] CLI not found at {LOCATE_CLI}")
            return

        if result.returncode != 0:
            print(f"  [ERROR] locate-anything-cli failed:\n{result.stderr}")
            return

        try:
            data = json.loads(result.stdout)
        except json.JSONDecodeError:
            print(f"  [ERROR] Could not parse output:\n{result.stdout}")
            return

        detections = data.get("detections", [])
        if not detections:
            print("  No object found matching that description.")
            return

        det = detections[0]
        box = det["box"]  # [x1, y1, x2, y2] in pixels
        cx_px = (box[0] + box[2]) / 2.0
        cy_px = (box[1] + box[3]) / 2.0
        print(f"  Found '{det.get('label', prompt_text)}' "
              f"at pixel ({cx_px:.0f}, {cy_px:.0f}), box={box}")

        # ── 3D point, only possible with live depth + intrinsics ──
        if depth_img is None or intrinsics is None:
            print("  (Test-image mode: no depth available, "
                  "pixel location only -- can't compute 3D pose.)")
            return

        fx, fy, ppx, ppy = intrinsics
        u, v = int(cx_px), int(cy_px)
        if not (0 <= v < depth_img.shape[0] and 0 <= u < depth_img.shape[1]):
            print("  [ERROR] Detected point outside depth image bounds.")
            return

        z = float(depth_img[v, u])
        # Kinova depth is typically mm as uint16, or meters as float32 --
        # confirm units once the real camera is live and adjust if needed.
        if z > 100:   # heuristic: looks like millimeters
            z = z / 1000.0
        if z <= 0.0:
            print("  [ERROR] No valid depth at that pixel (occluded/out of range).")
            return

        x = (u - ppx) * z / fx
        y = (v - ppy) * z / fy

        cam_point = PointStamped()
        cam_point.header.frame_id = self.camera_frame
        cam_point.header.stamp = self.get_clock().now().to_msg()
        cam_point.point.x = x
        cam_point.point.y = y
        cam_point.point.z = z

        try:
            world_point = self.tf_buffer.transform(
                cam_point, self.target_frame, timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            print(f"  [ERROR] tf2 transform failed: {e}")
            print(f"  (Check that '{self.camera_frame}' and "
                  f"'{self.target_frame}' are valid, connected frames.)")
            return

        pose = PoseStamped()
        pose.header = world_point.header
        pose.pose.position = world_point.point
        pose.pose.orientation.w = 1.0  # identity orientation for now

        self.pub.publish(pose)
        print(f"  Published pose in '{self.target_frame}': "
              f"({world_point.point.x:.3f}, {world_point.point.y:.3f}, "
              f"{world_point.point.z:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = VlmLocateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
