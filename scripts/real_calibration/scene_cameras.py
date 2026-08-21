#!/usr/bin/env python3
"""The two room cameras, at settings that were MEASURED to work over usbip.

REALSENSE D435i -- depth 424x240 @15 and colour 640x480 @15, and those
numbers are not a preference.

    Measured 2026-08-21 over usbip on this rig, unique DEPTH frames out of 60:

        depth 640x480 @30, alone          33/60, 4 timeouts
        depth 640x480 @15, alone           4/60, 4 timeouts
        depth 848x480 @15, alone           0/60   -- nothing at all
        depth 640x480 @15 + colour         6/60   -- looks like 25 fps, is not
        depth 424x240 @15 + colour        60/60, 0 timeouts   <-- this one

    The dangerous row is 640x480 with colour. librealsense re-delivers the
    LAST depth frame inside each frameset, so wait_for_frames() returns at
    full rate, every call succeeds, and the depth content never changes. It
    reads as a healthy 25 fps stream while the depth is frozen -- caught only
    by hashing frame CONTENT and by temporal std reading exactly 0.0000 over
    239k pixels. Any capture at 640x480 depth here is silently stale.

MOUNTED UPSIDE DOWN. The rotation is handled ONCE, here, and both halves are
done together: rotating the image without rotating the principal point is a
bug that produces PLAUSIBLE coordinates, because cx is near the centre so the
error is only a few pixels. In the rotated frame cx' = W-1-cx and
cy' = H-1-cy -- 320.13 / 247.78 against a native 318.87 / 231.22.

HD USB CAMERA -- MJPG IS MANDATORY. Default YUYV negotiates, opens, and then
read() returns False forever: measured MJPG 640x480 -> 25.0 fps with 60/60
distinct frames, YUYV -> no frames at all.
"""
from __future__ import annotations

import hashlib

import numpy as np

RS_DEPTH_WH = (424, 240)
RS_COLOR_WH = (640, 480)
RS_FPS = 15
HD_INDEX = 0
HD_WH = (640, 480)


class CameraError(RuntimeError):
    pass


class SceneRealSense:
    """Depth + colour, aligned to colour, with a liveness check."""

    def __init__(self, rotate180=True):
        self.rotate180 = rotate180
        self.pipe = None
        self.scale = None
        self.K = None          # (fx, fy, cx, cy, w, h) AFTER rotation
        self._align = None

    def open(self):
        import pyrealsense2 as rs
        import cv2  # noqa: F401  (imported so a missing cv2 fails here)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, RS_DEPTH_WH[0], RS_DEPTH_WH[1],
                          rs.format.z16, RS_FPS)
        cfg.enable_stream(rs.stream.color, RS_COLOR_WH[0], RS_COLOR_WH[1],
                          rs.format.bgr8, RS_FPS)
        try:
            prof = self.pipe.start(cfg)
        except Exception as exc:
            raise CameraError(
                "RealSense would not start at %dx%d depth + %dx%d colour @%d. "
                "If it enumerates but will not stream, check it negotiated "
                "USB3 (`cat /sys/bus/usb/devices/*/speed` should read 5000, "
                "not 480) and that this user can open /dev/video* and "
                "/dev/bus/usb/*/* : %s"
                % (RS_DEPTH_WH[0], RS_DEPTH_WH[1], RS_COLOR_WH[0],
                   RS_COLOR_WH[1], RS_FPS, exc))
        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        ci = prof.get_stream(rs.stream.color) \
                 .as_video_stream_profile().get_intrinsics()
        w, h = ci.width, ci.height
        if self.rotate180:
            # BOTH halves of the rotation, together. See the module docstring.
            self.K = (ci.fx, ci.fy, w - 1 - ci.ppx, h - 1 - ci.ppy, w, h)
        else:
            self.K = (ci.fx, ci.fy, ci.ppx, ci.ppy, w, h)
        self._align = rs.align(rs.stream.color)
        for _ in range(20):
            self.pipe.wait_for_frames()
        return self

    def _raw(self):
        import cv2
        f = self._align.process(self.pipe.wait_for_frames(timeout_ms=4000))
        d = np.asanyarray(f.get_depth_frame().get_data()).astype(np.float32) \
            * self.scale
        c = np.asanyarray(f.get_color_frame().get_data())
        n = f.get_depth_frame().get_frame_number()
        if self.rotate180:
            d = cv2.rotate(d, cv2.ROTATE_180)
            c = cv2.rotate(c, cv2.ROTATE_180)
        return d, c, n

    def grab(self, n=20, require_live=True):
        """Averaged depth over VALID pixels, latest colour, and liveness.

        Averaging only over valid pixels matters: a hole is 0, and including
        zeros in the mean drags every edge pixel toward the camera.
        """
        ds, cs, ns, hs = [], None, [], []
        for _ in range(n):
            d, c, fn = self._raw()
            ds.append(d)
            cs = c
            ns.append(fn)
            hs.append(hashlib.md5(d.tobytes()).hexdigest()[:8])
        D = np.stack(ds)
        V = D > 0
        cnt = V.sum(0)
        mean = np.where(cnt > 0, np.where(V, D, 0).sum(0)
                        / np.maximum(cnt, 1), 0.0)
        uniq_n = len(set(ns))
        uniq_h = len(set(hs))
        live = dict(frames=n, unique_frame_numbers=uniq_n,
                    unique_content=uniq_h,
                    valid_frac=float((mean > 0).mean()))
        if require_live and (uniq_h < max(2, n // 3)):
            raise CameraError(
                "DEPTH IS STALE: %d frames gave only %d distinct depth "
                "images (frame numbers %d distinct). librealsense re-delivers "
                "the last depth frame when the stream stalls, so this looks "
                "like a healthy rate and is not. Drop the depth resolution."
                % (n, uniq_h, uniq_n))
        return mean, cs, live

    def close(self):
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
            self.pipe = None


class SceneWebcam:
    """The HD USB camera. MJPG only."""

    def __init__(self, index=HD_INDEX, wh=HD_WH):
        self.index, self.wh = index, wh
        self.cap = None

    def open(self):
        import cv2
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise CameraError(
                "/dev/video%d will not open. Either it is not attached "
                "(usbipd attach --wsl --busid <id>) or this user cannot read "
                "it -- the nodes are root:video and the video group may be "
                "empty." % self.index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.wh[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.wh[1])
        for _ in range(5):
            cap.read()
        self.cap = cap
        return self

    def grab(self):
        ok, fr = self.cap.read()
        if not ok:
            raise CameraError(
                "HD camera returned no frame. Over usbip the default YUYV "
                "negotiates and then delivers nothing; MJPG is mandatory.")
        return fr

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def deproject(depth, K):
    """(N,3) camera-frame points and the (N,2) pixels they came from."""
    fx, fy, cx, cy, w, h = K
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    m = depth > 0
    z = depth[m]
    P = np.stack([(uu[m] - cx) * z / fx, (vv[m] - cy) * z / fy, z], 1)
    return P, np.stack([uu[m], vv[m]], 1)


def self_test(verbose=True):
    """Geometry only -- runs with no camera attached.

    The rotation check is the one that matters: it is the bug that produces
    believable wrong numbers.
    """
    w, h = 640, 480
    native_cx, native_cy = 318.87, 231.22
    rot_cx, rot_cy = w - 1 - native_cx, h - 1 - native_cy
    assert abs(rot_cx - 320.13) < 0.01 and abs(rot_cy - 247.78) < 0.01
    if verbose:
        print("rotated principal point %.2f, %.2f (native %.2f, %.2f)  OK"
              % (rot_cx, rot_cy, native_cx, native_cy))

    # A point on the optical axis must land on the principal point, and the
    # round trip must be exact.
    K = (600.0, 600.0, rot_cx, rot_cy, w, h)
    depth = np.zeros((h, w), np.float32)
    u0, v0 = 401, 122
    depth[v0, u0] = 2.5
    P, uv = deproject(depth, K)
    assert len(P) == 1
    fx, fy, cx, cy, _, _ = K
    ub = P[0, 0] * fx / P[0, 2] + cx
    vb = P[0, 1] * fy / P[0, 2] + cy
    assert abs(ub - u0) < 1e-6 and abs(vb - v0) < 1e-6, \
        "deproject/reproject round trip is not exact"
    if verbose:
        print("deproject round trip exact at (%d,%d) -> %s  OK"
              % (u0, v0, np.round(P[0], 4).tolist()))

    # Using the NATIVE principal point on a rotated image is the silent bug.
    Kbad = (600.0, 600.0, native_cx, native_cy, w, h)
    Pb, _ = deproject(depth, Kbad)
    err = float(np.linalg.norm(Pb[0] - P[0]))
    if verbose:
        print("native cx on a rotated image displaces this ray by %.1f mm "
              "at 2.5 m -- small enough to look right  OK" % (err * 1000))
    assert err > 0.005
    if verbose:
        print("scene_cameras self-test PASSED")
    return True


if __name__ == "__main__":
    self_test(verbose=True)
