#!/usr/bin/env python3
"""scene_camera_node.py -- the front-of-the-table USB camera, into ROS.

    ros2 run srl_perception scene_camera_node
    ros2 run srl_perception scene_camera_node --ros-args -p device:=/dev/video2
    ros2 run srl_perception scene_camera_node \
        --ros-args -p source:=file:/tmp/x.png

Publishes:
    /scene_camera/image_raw      sensor_msgs/Image, bgr8
    /scene_camera/camera_info    sensor_msgs/CameraInfo, from the calibration
    /scene_camera/state          std_msgs/String, JSON -- the honest one

WHY NOT `usb_cam` OR `v4l2_camera`. Both are packaged and neither is
installed, and installing either would not give the two behaviours this
repository requires of anything that produces frames:

  * IT REFUSES RATHER THAN REPUBLISHING. If the device stops delivering, this
    node publishes NOTHING and says so on /scene_camera/state. It never
    re-sends the last good frame. A frozen picture of a workspace cannot be
    told from a live picture of a workspace that is not moving, which is
    CLAUDE.md's "data fresh but never changes" row -- and here the consumer
    is a body tracker feeding a collision model.
  * IT PUBLISHES CameraInfo THAT IS EITHER REAL OR EMPTY. An uncalibrated
    camera publishes a ZERO K and says `calibrated: false`, rather than a
    plausible guessed focal length. A guessed intrinsic silently scales every
    body position; nothing downstream disagrees with it.

USB PASSTHROUGH. There is no USB on WSL without usbipd, exactly as for the
Teensy. `scripts/attach_scene_camera.sh` checks and prints the two commands.
The kernel here has CONFIG_USBIP_VHCI_HCD=m and CONFIG_USB_VIDEO_CLASS=m, so
the modules exist and load on attach; what it does NOT have is a camera
already attached, and this node says which of those is wrong.
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

DEFAULT_INTRINSICS = "config/scene_camera_intrinsics.yaml"


def load_intrinsics(path):
    """(K, D, w, h, source) or (None, ...) -- never a guess.

    Deliberately parsed with the stdlib rather than yaml so that a missing
    PyYAML cannot turn "uncalibrated" into "crashed", which are different
    problems with different fixes.
    """
    if not path or not os.path.exists(path):
        return None, None, 0, 0, "no calibration file at %s" % path
    try:
        import yaml
        with open(path) as fh:
            d = yaml.safe_load(fh)
        K = np.array(d["camera_matrix"], dtype=float).reshape(3, 3)
        D = np.array(d.get("distortion", [0] * 5), dtype=float).ravel()
        return K, D, int(d["image_width"]), int(d["image_height"]), path
    except Exception as e:                                    # noqa: BLE001
        return None, None, 0, 0, "calibration file unreadable: %r" % (e,)


class SceneCamera(Node):

    def __init__(self):
        super().__init__("scene_camera")
        self.declare_parameter("device", "")
        # WHICH CAMERA, NOT WHICH NUMBER. `/dev/videoN` is a position in a
        # list that moves whenever a device is re-attached over usbip. This
        # is matched against the driver's card name and the USB VID:PID --
        # "realsense", "hd usb", "8086:0b3a" all work -- and it is a
        # PREFERENCE, not a filter: everything is tried, in order, so a
        # busy or absent favourite degrades to the next camera rather than
        # to "no camera".
        # PREFER THE HD USB CAMERA, NOT THE REALSENSE. Changed 2026-08-30.
        #
        # Two files disagreed about which device IS the scene camera:
        # `usb_cameras.py` has always labelled 32e4:0317 "HD USB CAMERA
        # (scene camera)", and this node preferred "realsense". So the node
        # opened a RealSense node, held it, and the camera the rest of the
        # system calls the scene camera sat free.
        #
        # And the RealSense is the WORSE choice over usbip, which is the only
        # way it reaches WSL. Measured across this session: it runs about
        # 5 Hz at 640x480, detaches on its own, and when it goes it goes like
        # this --
        #
        #     VIDEOIO(V4L2:/dev/video6): select() timeout
        #     scene camera delivered no frame. NOTHING is being published
        #
        # -- while /dev/video0 was sitting there reporting "YES delivers".
        # The HD camera has been the reliable one all along.
        #
        # MATCHED ON VID:PID, NOT A NAME OR A NUMBER. `/dev/videoN` is a
        # position in a list that moves on every usbip re-attach, and this
        # camera presents TWO nodes (video0 and video1) under one identity.
        # `Dev.matches` does a substring over name and vid_pid, so the USB ID
        # is the one thing that survives a replug.
        self.declare_parameter("prefer", "32e4:0317")
        # FOUR OF THE REALSENSE'S SIX NODES ARE DEPTH OR INFRARED. They
        # open, they deliver, and a grey picture of the right room passes
        # any check that asks only whether a frame arrived.
        self.declare_parameter("require_colour", True)
        # SELF-REPAIR THE USB LINK, not just the V4L2 handle. Measured
        # 2026-08-29: the RealSense dropped off the usbip bus mid-session
        # and came back with `usbipd list` saying Attached, all six device
        # nodes present, and nothing delivering. Re-opening the handle can
        # never fix that; detach-then-attach does, and did on the first read.
        self.declare_parameter("usbip_repair", True)
        self.declare_parameter("source", "v4l2")
        # 640x480 IS THE DEFAULT BECAUSE IT IS WHAT THE LINK CARRIES.
        # Measured 2026-08-29: the RealSense over usbip delivers nothing at
        # 1280x720 -- every read blocks until V4L2's select() times out, at
        # about 11 s PER DEVICE, so asking for it cost the better part of a
        # minute of probing before settling on the size that works anyway.
        # Ask for more and it is still tried first; see `_sizes`.
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("intrinsics", DEFAULT_INTRINSICS)
        # 0.5 s WAS TIGHTER THAN THE LINK. Measured 2026-08-29 on the
        # RealSense over usbip: ~5 Hz with gaps up to 0.93 s, so the panel
        # flickered STALE on a camera the node itself called healthy. The
        # threshold has to be looser than the transport's worst gap or it
        # reports the transport as a fault.
        self.declare_parameter("stale_after_s", 1.5)

        ws = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")
        ip = self.get_parameter("intrinsics").value
        if ip and not os.path.isabs(ip):
            ip = os.path.join(ws, ip)
        self.K, self.D, self.cw, self.ch, self.calib_src = load_intrinsics(ip)

        self.pub = self.create_publisher(Image, "/scene_camera/image_raw",
                                         qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, "/scene_camera/camera_info", 10)
        self.state_pub = self.create_publisher(String, "/scene_camera/state", 10)

        self.cap = None
        self.dev = None
        self.last_ok = 0.0
        self.n_frames = 0
        self.n_fail = 0
        self.reason = "starting"
        self._still = None

        self._open()
        hz = max(1.0, float(self.get_parameter("fps").value))
        self.create_timer(1.0 / hz, self.tick)
        self.create_timer(1.0, self.say)

    # -------------------------------------------------------------- open
    def _candidates(self):
        """Devices to try, in order. An explicit `device` wins outright."""
        from srl_perception import video_devices as vd
        d = self.get_parameter("device").value
        if d:
            for dev in vd.devices():
                if dev.path == d:
                    return [dev]
            return [vd.Dev(d)]
        return vd.order(str(self.get_parameter("prefer").value or ""))

    def _sizes(self):
        """The size asked for, then the one the usbip link can carry.

        NOT A SILENT DOWNGRADE -- `reason` names the size that worked, and
        the state message carries it, so a 640x480 picture is never reported
        as the 1280x720 that was asked for.
        """
        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        sizes = [(w, h)]
        if (w, h) != (640, 480):
            sizes.append((640, 480))
        return sizes

    def _open(self):
        src = str(self.get_parameter("source").value)
        if src.startswith("file:"):
            # A STILL, for testing the whole path with no device. It is
            # published with a MOVING timestamp and identical pixels, which is
            # exactly the input the staleness rule must NOT be fooled by --
            # so the state message says `source: file` and every consumer can
            # see that this is not a camera.
            import cv2
            path = src.split(":", 1)[1]
            img = cv2.imread(path)
            if img is None:
                self.reason = "cannot read %s" % path
                return
            self._still = img
            self.reason = "file source %s (NOT A CAMERA)" % path
            return
        import cv2
        from srl_perception import video_devices as vd
        cands = self._candidates()
        if not cands:
            self.reason = (
                "no camera is attached to this machine. On WSL a USB camera "
                "must be handed over from Windows first -- run "
                "python3 scripts/usb_cameras.py --fix, which attaches them.")
            return
        want_colour = bool(self.get_parameter("require_colour").value)
        rejected = []
        # ONE WALK OF /proc for the whole probe, not one per candidate.
        open_now = vd.holders()
        for dev in cands:
            path = dev.path if hasattr(dev, "path") else dev
            held = vd.users(path, open_now)
            label = getattr(dev, "name", "") or path
            if held:
                # SAY WHOSE IT IS. `quest_bridge_node` claims the wide-view
                # camera the moment the VR chain starts; a second reader
                # gets an open that succeeds and frames that never come,
                # which is indistinguishable from a broken camera.
                rejected.append("%s in use by pid %s"
                                % (path, ", ".join(str(p) for p in held)))
                continue
            cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                rejected.append("%s will not open" % path)
                continue
            # MJPG IS MANDATORY OVER USBIP, not a preference, and this
            # line was missing. The default uncompressed YUYV negotiates
            # happily and then delivers NOTHING: the device opens, every
            # call succeeds, and read() returns False for ever. Measured on
            # this rig (srl_cameras.py): MJPG 640x480 -> 24.9 fps, YUYV ->
            # no frames at all. Without it the loop below rejected a WORKING
            # camera and blamed "an IR or depth node", which sent the reader
            # to the wrong device entirely.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))
            frame = None
            for w, h in self._sizes():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                # A DEVICE THAT OPENS IS NOT A DEVICE THAT DELIVERS.
                # Several things on this machine present as /dev/video* and
                # return nothing -- the IR sensor of this very webcam is
                # one. So the candidate has to produce a frame before it is
                # accepted, AT A SIZE IT WILL ACTUALLY DELIVER: measured
                # 2026-08-29, the RealSense over usbip delivers nothing at
                # 1280x720 and 24.9 fps at 640x480, and the node reported
                # that as "none of them delivered a frame" -- a working
                # camera, refused for asking too much of the link.
                ok, f = cap.read()
                if ok and f is not None:
                    frame = f
                    break
            if frame is None:
                cap.release()
                rejected.append("%s delivered no frame at %s"
                                % (path, " or ".join("%dx%d" % s
                                                     for s in self._sizes())))
                continue
            if want_colour and not vd.is_colour(frame):
                cap.release()
                rejected.append("%s is monochrome (depth or infrared)" % path)
                continue
            self.cap = cap
            self.dev = path
            # WRITE DOWN WHAT WORKED, so the next start does not pay for
            # the probe again. Keyed on the device's own identity, never on
            # the path -- see video_devices.remember.
            if hasattr(dev, "key"):
                vd.remember(dev)
            self.reason = "open on %s (%s), %dx%d" % (
                path, label, frame.shape[1], frame.shape[0])
            self.get_logger().info("scene camera %s" % self.reason)
            return
        self.reason = ("no usable camera. Tried %d: %s"
                       % (len(cands), "; ".join(rejected) or "nothing"))

    # -------------------------------------------------------------- tick
    def tick(self):
        frame = None
        if self._still is not None:
            frame = self._still
        elif self.cap is not None:
            ok, f = self.cap.read()
            if ok and f is not None:
                frame = f
            else:
                self.n_fail += 1
                if self.n_fail in (1, 30) or self.n_fail % 300 == 0:
                    self.get_logger().error(
                        "scene camera delivered no frame (%d in a row). "
                        "NOTHING is being published -- the last picture is "
                        "not being re-sent." % self.n_fail)
                # RE-PROBE, DO NOT SIT THERE FAILING FOR EVER.
                #
                # This used to `return` on every failed read and never try
                # again, so a camera that went away was gone until somebody
                # noticed and restarted the node. Under WSL that is the
                # NORMAL case, not an edge case: the usbipd service drops
                # devices on its own, and a re-attach can hand the SAME
                # camera back as a different /dev/videoN. The node was still
                # holding the old handle, reading nothing, and reporting
                # "open on /dev/video0" while /dev/video0 no longer existed.
                #
                # `_open()` re-runs the whole probe -- candidates, MJPG, and
                # the read test -- so recovery does not assume the number is
                # unchanged. Attempted on a back-off rather than every tick,
                # because re-opening a V4L2 device at 30 Hz is its own way of
                # keeping a camera unusable.
                self._maybe_reopen()
                return
        if frame is None:
            return
        self.n_fail = 0
        self.n_frames += 1
        self.last_ok = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = "scene_camera"
        m.height, m.width = frame.shape[0], frame.shape[1]
        m.encoding = "bgr8"
        m.is_bigendian = 0
        m.step = frame.shape[1] * 3
        m.data = frame.tobytes()
        self.pub.publish(m)
        self.info_pub.publish(self._info(stamp, frame.shape))

    #: Consecutive failed reads before the first re-probe. At the 30 Hz
    #: tick this is about a second -- long enough not to fire on a single
    #: dropped frame, short enough that a re-attach is picked up quickly.
    REOPEN_AFTER_FAILURES = 30
    #: Seconds between re-probe attempts once it is failing.
    REOPEN_EVERY_S = 2.0

    def _maybe_reopen(self):
        """Re-run the probe, on a back-off, while the camera is not reading."""
        if self.n_fail < self.REOPEN_AFTER_FAILURES:
            return
        now = time.monotonic()
        if now - getattr(self, "_last_reopen", 0.0) < self.REOPEN_EVERY_S:
            return
        self._last_reopen = now
        old_dev = getattr(self, "dev", None)
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:                                     # noqa: BLE001
            pass
        self.cap = None
        self.dev = None
        try:
            self._open()
        except Exception as e:                                # noqa: BLE001
            self.reason = "re-probe failed: %s" % e
            return
        if self.cap is None:
            self._repair_usb()
        if self.cap is not None:
            self.n_fail = 0
            self.get_logger().info(
                "scene camera RECOVERED: %s (was %s)"
                % (self.reason, old_dev or "not open"))

    REPAIR_EVERY_S = 60.0

    def _repair_usb(self):
        """Re-attach the cameras over usbip, then probe once more.

        Rate-limited hard: a detach-attach is disruptive to anything else
        holding a camera, so it happens at most once a minute and only when
        no device delivered anything at all.
        """
        if not bool(self.get_parameter("usbip_repair").value):
            return
        now = time.monotonic()
        if now - getattr(self, "_last_repair", 0.0) < self.REPAIR_EVERY_S:
            return
        self._last_repair = now
        ws = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")
        script = os.path.join(ws, "scripts", "usb_cameras.py")
        if not os.path.exists(script):
            return
        import subprocess
        import sys as _sys
        self.get_logger().warn(
            "no camera delivered anything -- re-attaching over usbip")
        try:
            subprocess.run([_sys.executable, script, "--fix", "--quiet"],
                           capture_output=True, timeout=180)
        except Exception as e:                                # noqa: BLE001
            self.reason = "%s; usbip repair failed: %r" % (self.reason, e)
            return
        try:
            self._open()
        except Exception as e:                                # noqa: BLE001
            self.reason = "re-probe after usbip repair failed: %s" % e

    def _info(self, stamp, shape):
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = "scene_camera"
        ci.height, ci.width = shape[0], shape[1]
        if self.K is None:
            # ZERO, NOT A GUESS. A plausible-looking focal length here would
            # scale every body position by an unknown factor and nothing
            # downstream would disagree with it.
            ci.k = [0.0] * 9
            ci.d = []
            ci.distortion_model = ""
            return ci
        K = self.K
        if (self.cw, self.ch) != (shape[1], shape[0]) and self.cw and self.ch:
            # THE CALIBRATION WAS TAKEN AT A DIFFERENT SIZE. Scaling the
            # intrinsic is correct and is done here rather than left to the
            # consumer, because a consumer that forgets is a consumer whose
            # body positions are wrong by the ratio.
            sx, sy = shape[1] / float(self.cw), shape[0] / float(self.ch)
            K = K.copy()
            K[0, 0] *= sx
            K[0, 2] *= sx
            K[1, 1] *= sy
            K[1, 2] *= sy
        ci.k = [float(x) for x in K.ravel()]
        ci.d = [float(x) for x in self.D]
        ci.distortion_model = "plumb_bob"
        ci.p = [float(x) for x in
                np.hstack([K, np.zeros((3, 1))]).ravel()]
        return ci

    # ------------------------------------------------------------- state
    def say(self):
        age = None if self.last_ok == 0.0 else time.monotonic() - self.last_ok
        stale = float(self.get_parameter("stale_after_s").value)
        live = age is not None and age <= stale
        st = dict(
            live=bool(live),
            source=("file" if self._still is not None
                    else ("v4l2" if self.cap is not None else "none")),
            # WHICH CAMERA, BY NAME. The path alone is not an answer: it
            # moves between sessions, so a recording that says /dev/video6
            # does not say which camera took it.
            device=self.dev,
            reason=self.reason,
            frames=self.n_frames,
            consecutive_failures=self.n_fail,
            # WHETHER IT IS TRYING TO COME BACK. A consumer
            # must be able to tell 'gone for ever' from
            # 'reconnecting', because they need different
            # reactions from the operator.
            reprobing=bool(self.n_fail >= self.REOPEN_AFTER_FAILURES),
            age_s=None if age is None else round(age, 3),
            calibrated=self.K is not None,
            calibration=self.calib_src,
        )
        self.state_pub.publish(String(data=json.dumps(st)))
        if not live and self.n_frames == 0:
            self.get_logger().warn("scene camera: %s" % self.reason,
                                   throttle_duration_sec=10.0)


def main(argv=None):
    rclpy.init(args=argv)
    n = SceneCamera()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.cap is not None:
            n.cap.release()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
