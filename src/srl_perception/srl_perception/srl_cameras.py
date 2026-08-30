#!/usr/bin/env python3
"""The two cameras this rig actually has, and how to get frames out of them.

SCENE CAMERA -- a USB webcam handed into WSL with usbipd.
  MJPG IS MANDATORY, not a preference. Over usbip the default uncompressed
  YUYV negotiates happily and then delivers NOTHING: the device opens, every
  call succeeds, and read() returns False forever. Measured 2026-08-21 on
  this rig: MJPG 640x480 -> 24.9 fps, YUYV -> no frames at all.
  1920x1080 also returns nothing; it is over the usbip bandwidth.

GRIPPER CAMERA -- the Kinova vision module on the arm itself.
  Colour is H.264 over RTSP and ffmpeg reads it. DEPTH IS NOT: it is a
  GStreamer-payloaded RAW stream (rtpgstdepay), 480x270 GRAY16_LE in
  millimetres, and ffmpeg/OpenCV simply will not open it. That is why
  rtsp://<ip>/depth "does not work" until you use GStreamer.
  Kinova documents max TWO simultaneous connections per stream and a 30 s
  inactivity timeout.

INTRINSICS ARE READ FROM THE ROBOT, NOT ASSUMED. The factory values on this
arm are colour cx=620.91 cy=238.28 -- the principal point is 122 px above the
image centre. Anyone who assumes cy = h/2 throws every ray by 122 px, which
at 1 m is 94 mm, three times the grasp tolerance.
"""
from __future__ import annotations

import os
import subprocess
import tempfile
import time

import numpy as np

# Factory intrinsics read from this arm over the Kortex API, 2026-08-21.
# (fx, fy, cx, cy)
KINOVA_COLOR_K = (1297.6729, 1298.6313, 620.914, 238.28032)
KINOVA_DEPTH_K = (342.2138, 342.2138, 233.06856, 132.48465)
KINOVA_DEPTH_WH = (480, 270)
# depth sensor -> colour sensor, metres. 27 mm: negligible past ~0.5 m, and
# recorded so nobody has to wonder whether it was neglected or forgotten.
KINOVA_DEPTH_TO_COLOR_M = (-0.02706, -0.00997, -0.00471)


class CameraError(RuntimeError):
    """Raised with a reason and, where possible, the fix."""


SCENE_CAMERA_ENV = "SRL_SCENE_CAMERA_INDEX"


def video_indices():
    """Every /dev/videoN on this host, ascending."""
    import glob
    import re
    out = []
    for p in glob.glob("/dev/video*"):
        m = re.search(r"(\d+)$", p)
        if m:
            out.append(int(m.group(1)))
    return sorted(out)


# USB identity of the scene webcam on this rig, read from sysfs.
# 32e4:0317 is the "HD USB CAMERA". The RealSense is 8086:0b3a and must NEVER
# be chosen as the scene camera even though it delivers frames perfectly.
SCENE_VIDPID = ("32e4:0317",)
EXCLUDE_VIDPID = ("8086:0b3a",)        # RealSense: driven by pyrealsense2


def video_identity(index):
    """(card name, 'vid:pid') for /dev/videoN, from sysfs. ('', '') if absent.

    THE DEVICE NUMBER IS NOT AN IDENTITY. Detaching and re-attaching over
    usbipd renumbers every node: measured on this rig, the same two cameras
    came back as video1..video8 having previously been video0..video7, so the
    webcam stopped being index 0 without anything changing physically.
    Anything that remembers an index is wrong after the next reboot, unplug,
    or usbipd attach -- and it fails by silently opening the WRONG CAMERA,
    which is far worse than failing to open one.
    """
    base = "/sys/class/video4linux/video%d" % index
    try:
        with open(base + "/name") as fh:
            name = fh.read().strip()
    except OSError:
        return "", ""
    vid = pid = ""
    d = os.path.realpath(base)
    for _ in range(6):                 # walk up to the USB device node
        d = os.path.dirname(d)
        try:
            with open(os.path.join(d, "idVendor")) as fh:
                vid = fh.read().strip()
            with open(os.path.join(d, "idProduct")) as fh:
                pid = fh.read().strip()
            break
        except OSError:
            continue
    return name, ("%s:%s" % (vid, pid) if vid and pid else "")


def scene_camera_candidates():
    """Every /dev/videoN that could be the scene webcam, best first.

    Ordered by how sure we are of the identification:
      1. USB VID:PID matches the known scene camera
      2. the card name looks like a webcam and is NOT an excluded device
      3. anything left that is not excluded
    The RealSense is excluded by VID:PID at every level -- it opens cleanly
    and returns frames, so a probe that only asks "does this deliver pixels?"
    will happily choose its colour stream and then the scene view is a
    close-up of whatever the depth camera is pointed at.
    """
    tier1, tier2, tier3 = [], [], []
    for i in video_indices():
        name, vidpid = video_identity(i)
        if vidpid in EXCLUDE_VIDPID:
            continue
        if vidpid in SCENE_VIDPID:
            tier1.append(i)
        elif name and "realsense" not in name.lower():
            tier2.append(i)
        else:
            tier3.append(i)
    return tier1 + tier2 + tier3


def probe_scene_camera(width=640, height=480):
    """The first /dev/videoN that actually DELIVERS an MJPG frame.

    NOT the first that OPENS. This defaulted to index 0 for the life of the
    file, which was right on the rig it was written for and is wrong the
    moment anything else claims a lower node: a RealSense presents SIX
    /dev/video* nodes, of which the depth and metadata ones open cleanly and
    return nothing for ever. That is this module's own documented failure
    mode -- "the device opens, every call succeeds, and read() returns False
    forever" -- and it was reachable through the default argument.

    So the probe is a READ, not an open, and it returns (index, tried) so a
    failure can name every node it rejected rather than blaming the one it
    happened to try first.
    """
    import cv2
    tried = []
    cands = scene_camera_candidates()
    if not cands:
        return None, ["no non-RealSense video node exists; only the depth "
                      "camera is attached"]
    for i in cands:
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                tried.append("%d: will not open" % i)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ok, frame = cap.read()
            if ok and frame is not None:
                return i, tried
            nm, vp = video_identity(i)
            tried.append("%d (%s %s): opens, delivers no frame"
                         % (i, nm or "?", vp or "?"))
        finally:
            cap.release()
    return None, tried


class SceneCamera:
    """The USB webcam looking at the rig from across the room.

    `index=None` (the default) resolves at open() time: the environment
    variable SRL_SCENE_CAMERA_INDEX if set, otherwise the first node that
    delivers a frame. An explicit index is always honoured as given.
    """

    def __init__(self, index=None, width=640, height=480):
        self.index, self.width, self.height = index, width, height
        self._cap = None

    def _resolve(self):
        if self.index is not None:
            return
        env = os.environ.get(SCENE_CAMERA_ENV, "").strip()
        if env:
            try:
                self.index = int(env)
                return
            except ValueError:
                raise CameraError(
                    "%s is %r, which is not a device index. Set it to the N "
                    "of the /dev/videoN you want, or unset it to let the "
                    "camera be found." % (SCENE_CAMERA_ENV, env))
        found, tried = probe_scene_camera(self.width, self.height)
        if found is None:
            raise CameraError(
                "no /dev/video* delivered a frame. Tried: %s. Either nothing "
                "is attached (usbipd attach --wsl --busid <id>), this user is "
                "not in the 'video' group, or another process holds the "
                "device -- V4L2 allows one capture client, and the VR bridge "
                "takes it when the headset page is open."
                % ("; ".join(tried) if tried else "no /dev/video* at all"))
        self.index = found

    def open(self):
        import cv2
        self._resolve()
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise CameraError(
                "/dev/video%d will not open. Either it is not attached "
                "(usbipd attach --wsl --busid <id>), this user is not in the "
                "'video' group, or another process holds it -- V4L2 allows "
                "one capture client, and the VR bridge takes it when the "
                "headset page is open." % self.index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap = cap
        return self

    def read(self, warm=6):
        if self._cap is None:
            self.open()
        f = None
        for _ in range(max(warm, 1)):
            ok, x = self._cap.read()
            if ok and x is not None:
                f = x
        if f is None:
            raise CameraError(
                "scene camera opened but returned no frame. Over usbip that "
                "is the signature of the wrong pixel format -- MJPG is "
                "required; the default YUYV yields nothing.")
        return f

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class GripperCamera:
    """The Kinova vision module: colour over ffmpeg, depth over GStreamer."""

    def __init__(self, ip, colour_K=KINOVA_COLOR_K, depth_K=KINOVA_DEPTH_K):
        self.ip = ip
        self.colour_K = colour_K
        self.depth_K = depth_K
        self._cap = None

    def read_colour(self, timeout_s=8.0):
        import cv2
        if self._cap is None or not self._cap.isOpened():
            self._cap = cv2.VideoCapture("rtsp://%s/color" % self.ip,
                                         cv2.CAP_FFMPEG)
        f, t0 = None, time.time()
        while time.time() - t0 < timeout_s:
            ok, x = self._cap.read()
            if ok and x is not None:
                f = x
            if f is not None and time.time() - t0 > 2.0:
                break
        if f is None:
            raise CameraError(
                "no colour frame from rtsp://%s/color. The arm must be "
                "powered and on the network, and Kinova allows only two "
                "simultaneous connections per stream." % self.ip)
        return f

    def read_depth(self, seconds=7, keep=None):
        """Depth in METRES, 480x270. GStreamer, because ffmpeg cannot.

        The stream carries raw GRAY16_LE frames back to back, so the last
        complete frame is taken from the tail of the capture.
        """
        raw = keep or os.path.join(tempfile.gettempdir(), "srl_depth.raw")
        try:
            os.remove(raw)
        except OSError:
            pass
        cmd = ("timeout %d gst-launch-1.0 rtspsrc location=rtsp://%s/depth "
               "latency=30 ! rtpgstdepay ! filesink location=%s"
               % (int(seconds), self.ip, raw))
        subprocess.run(["bash", "-c", cmd + " >/dev/null 2>&1"], check=False)
        W, H = KINOVA_DEPTH_WH
        need = W * H * 2
        try:
            data = open(raw, "rb").read()
        except OSError:
            raise CameraError(
                "gst-launch-1.0 produced nothing for rtsp://%s/depth. Is "
                "gstreamer installed with the rtsp plugins (rtspsrc, "
                "rtpgstdepay)?" % self.ip)
        if len(data) < need:
            raise CameraError(
                "depth stream gave %d bytes, less than one %dx%d frame (%d). "
                "The stream is GRAY16_LE and payloaded with rtpgstdepay -- "
                "ffmpeg and OpenCV cannot read it, which is why "
                "rtsp://<ip>/depth appears to 'not work'."
                % (len(data), W, H, need))
        f = np.frombuffer(data[-need:], dtype="<u2").reshape(H, W)
        return f.astype(np.float32) / 1000.0        # mm -> metres

    def read_rgbd(self, **kw):
        return self.read_colour(), self.read_depth(**kw)

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


def fetch_intrinsics(ip, user="admin", password="admin"):
    """Ask the ROBOT for its own intrinsics. Falls back to the recorded ones.

    Preferred over the constants above because a different arm has different
    values, and a calibration silently using another robot's numbers is the
    kind of error that looks like a bad grasp.
    """
    try:
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.RouterClient import RouterClient
        from kortex_api.SessionManager import SessionManager
        from kortex_api.autogen.client_stubs.VisionConfigClientRpc import (
            VisionConfigClient)
        from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import (
            DeviceManagerClient)
        from kortex_api.autogen.messages import (Session_pb2, VisionConfig_pb2,
                                                 DeviceConfig_pb2)
    except Exception as exc:                                  # noqa: BLE001
        return dict(colour=KINOVA_COLOR_K, depth=KINOVA_DEPTH_K,
                    source="recorded constants (kortex_api unavailable: %s)" % exc)
    t = TCPTransport()
    r = RouterClient(t, lambda ex: None)
    try:
        t.connect(ip, 10000)
        s = Session_pb2.CreateSessionInfo()
        s.username, s.password = user, password
        s.session_inactivity_timeout = 60000
        s.connection_inactivity_timeout = 20000
        sm = SessionManager(r)
        sm.CreateSession(s)
        dm, vc = DeviceManagerClient(r), VisionConfigClient(r)
        vid = [d.device_identifier for d in dm.ReadAllDevices().device_handle
               if d.device_type == DeviceConfig_pb2.VISION][0]
        out = {}
        for key, sensor in (("colour", VisionConfig_pb2.SENSOR_COLOR),
                            ("depth", VisionConfig_pb2.SENSOR_DEPTH)):
            ii = vc.GetIntrinsicParameters(
                VisionConfig_pb2.SensorIdentifier(sensor=sensor), vid)
            out[key] = (ii.focal_length_x, ii.focal_length_y,
                        ii.principal_point_x, ii.principal_point_y)
        sm.CloseSession()
        out["source"] = "read from the robot at %s" % ip
        return out
    except Exception as exc:                                  # noqa: BLE001
        return dict(colour=KINOVA_COLOR_K, depth=KINOVA_DEPTH_K,
                    source="recorded constants (robot unreachable: %s)" % exc)
    finally:
        try:
            t.disconnect()
        except Exception:                                     # noqa: BLE001
            pass
