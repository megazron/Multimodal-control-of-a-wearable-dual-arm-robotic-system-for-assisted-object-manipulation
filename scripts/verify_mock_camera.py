#!/usr/bin/env python3
"""Verify the MOCK camera's PLUMBING against constructed ground truth.

    python3 scripts/verify_mock_camera.py

WHAT IT CHECKS, and why each one is checkable:

  1. the three topics publish, with the right encodings (rgb8, 16UC1) and
     dimensions -- plumbing, directly observable;
  2. CameraInfo carries a usable K -- plumbing;
  3. DEPROJECTION lands on the object's TRUE world position. This is the
     load-bearing one. I know where every object is because the scene is
     CONSTRUCTED, so pixel + depth + TF -> world can be checked against a
     number known in advance. Arithmetic against a known answer is exactly
     the case the standing rule permits synthetic data for.
  4. the frame composition is the right way round, tested by a CONTROL: the
     deliberately transposed rotation must FAIL. A check that only ever
     passes cannot tell a correct pipeline from a broken one, and the
     world-space-vs-object-space rotation bug already cost this project a
     day once.

WHAT IT DOES NOT CHECK: detection rate, accuracy, or anything about how the
world LOOKS. See the mock's module docstring. Passing this script says the
pipe is connected and the arithmetic is right; it says nothing whatever
about whether a detector will find a real object in a real image.
"""
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception", "srl_perception"))

# 5 mm. The metric is the distance from the deprojected point to the object's
# SURFACE, not to its centre -- a camera ray hits the front face, so measuring
# to the centroid charges the round trip for half the object's depth and reads
# as a 29 mm error on a 50 mm cube when the arithmetic is in fact exact. That
# was the first version, and the number looked like a real defect.
TOL_M = 0.005


def _q2R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Sink(Node):
    def __init__(self, arm):
        super().__init__("mock_camera_verifier")
        self.color = self.depth = self.info = None
        b = "/%s_camera" % arm
        self.create_subscription(Image, b + "/color/image_raw",
                                 lambda m: setattr(self, "color", m), 1)
        self.create_subscription(Image, b + "/depth/image_raw",
                                 lambda m: setattr(self, "depth", m), 1)
        self.create_subscription(CameraInfo, b + "/color/camera_info",
                                 lambda m: setattr(self, "info", m), 1)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def spin(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    # RIGHT, not left: docs/ENGINEERING_LOG.md records that the LEFT wrist camera is parked
    # looking UP and back at home (elevation +38.3 deg) and "sees nothing at
    # table height from home", while the right points down and reaches z=0.80
    # in front of the wearer. Verifying against the left camera measures an
    # empty frame and calls it a failure of the pipeline.
    arm, task = "right", "t1"
    rows = []

    env = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS="SHM")
    cam = subprocess.Popen(
        ["python3", "-u",
         os.path.join(WS, "src", "srl_perception", "srl_perception",
                      "mock_rgbd_camera.py"),
         "--ros-args", "-p", "arm:=" + arm, "-p", "task:=" + task,
         "-p", "probe:=true"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    rclpy.init()
    n = Sink(arm)
    try:
        n.spin(12.0)
        ok = n.color is not None and n.depth is not None and n.info is not None
        rows.append(("all three topics publishing", ok,
                     "color=%s depth=%s info=%s"
                     % (n.color is not None, n.depth is not None,
                        n.info is not None)))
        if not ok:
            raise SystemExit(_report(rows))

        rows.append(("encodings are the DRIVER's, not the convenient ones",
                     n.color.encoding == "rgb8"
                     and n.depth.encoding == "16UC1",
                     "%s / %s (16UC1 mm exercises the branch that will run)"
                     % (n.color.encoding, n.depth.encoding)))
        K = np.array(n.info.k).reshape(3, 3)
        rows.append(("CameraInfo carries a usable K", K[0, 0] > 1.0,
                     "fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
                     % (K[0, 0], K[1, 1], K[0, 2], K[1, 2])))

        # SNAPSHOT THE FRAME, THEN LET TF CATCH UP. lookup_transform's own
        # timeout cannot help here: it blocks the calling thread, so nothing
        # is spinning to FILL the buffer while it waits, and it always times
        # out. Spinning past the image stamp first is what makes a stamped
        # lookup possible at all.
        snap_c, snap_d, snap_i = n.color, n.depth, n.info
        n.spin(1.0)
        n.color, n.depth, n.info = snap_c, snap_d, snap_i
        try:
            # LOOK UP TF AT THE IMAGE'S OWN STAMP, not at "latest". A depth
            # image is a measurement taken at an instant; deprojecting it
            # through the pose the camera has NOW is only correct if nothing
            # moved in between, and the wrist camera is on the end of a
            # moving arm. Measured: with the latest-TF lookup the round trip
            # missed by 0.1332 m; the geometry was right and the CLOCK was
            # wrong. The real pipeline has exactly this hazard, so testing it
            # the sloppy way would have hidden a defect rather than found one.
            stamp = rclpy.time.Time.from_msg(n.depth.header.stamp)
            try:
                from rclpy.duration import Duration
                t = n.tfb.lookup_transform("world", n.depth.header.frame_id,
                                           stamp, Duration(seconds=0.5))
            except Exception:
                t = n.tfb.lookup_transform("world", n.depth.header.frame_id,
                                           rclpy.time.Time())
                rows.append(("TF at the image stamp", False,
                             "fell back to latest -- buffer has no history "
                             "at the image time"))
        except Exception as e:
            rows.append(("TF world <- camera", False, str(e)[:60]))
            raise SystemExit(_report(rows))
        tr, ro = t.transform.translation, t.transform.rotation
        p_cam = np.array([tr.x, tr.y, tr.z])
        R_wc = _q2R(ro.x, ro.y, ro.z, ro.w)

        import clip_scene as CS
        # Same unpacking as the mock: TUPLES, and fixtures_for is names only.
        # THE PROBES, not the furniture: at home neither wrist camera frames
        # the work surface (measured -- bench at pixel (954, 539) for the
        # right camera, outside a 640-wide frame). Deprojection needs an
        # object IN VIEW, and the probes are fixed world constants placed
        # there for exactly that. The furniture is still rendered; it is
        # simply not what this check can use from the home pose.
        import mock_rgbd_camera as MC
        objs = [dict(name="probe_%d" % i, xyz=list(x),
                     size=[MC.PROBE_SIZE] * 3)
                for i, x in enumerate(MC.PROBE_WORLD)]
        objs += [dict(name=nm, xyz=list(xyz), size=list(sz))
                 for nm, xyz, sz, _rgba in CS.furniture_boxes(task)]
        dep = np.frombuffer(n.depth.data, np.uint16).reshape(
            n.depth.height, n.depth.width).astype(float) / 1000.0

        # DEPROJECT every object that is genuinely in view, and compare with
        # the position the scene definition GAVE it.
        errs, seen = [], 0
        for o in objs:
            pw = np.asarray(o["xyz"], float)
            pc = R_wc.T @ (pw - p_cam)
            if pc[2] <= 0.05:
                continue
            u = int(round(K[0, 0] * pc[0] / pc[2] + K[0, 2]))
            v = int(round(K[1, 1] * pc[1] / pc[2] + K[1, 2]))
            if not (0 <= u < n.depth.width and 0 <= v < n.depth.height):
                continue
            z = dep[v, u]
            if z <= 0.0:
                continue
            seen += 1
            # pixel + depth -> camera -> world, the consumer's own path
            back = p_cam + R_wc @ np.array([(u - K[0, 2]) * z / K[0, 0],
                                            (v - K[1, 2]) * z / K[1, 1], z])
            # DISTANCE TO THE BOX SURFACE, which is what a camera ray hits.
            h = 0.5 * np.asarray(o["size"], float)
            d = np.maximum(np.abs(back - pw) - h, 0.0)
            errs.append((o.get("name", "?"), float(np.linalg.norm(d))))
        worst = max((e for _, e in errs), default=9.9)
        # THREE STATES, NOT TWO, AND THAT IS THE POINT.
        #
        # This read `seen >= 2 and worst <= TOL_M` and printed FAIL for
        # anything else -- so a run with ONE object in view and an error of
        # 0.2 mm, which is the arithmetic being exactly right, was reported
        # identically to a broken deprojection. Measured on this machine:
        # "FAIL 1 objects in view, worst error 0.0002 m (tol 0.005)".
        #
        # From the HOME pose the wrist cameras frame almost nothing at table
        # height, which is documented and expected, so "too few objects to
        # judge" is the NORMAL outcome there and it is not a defect. It is
        # also not a pass. INSUFFICIENT says so, and names the fix.
        if seen < 2:
            rows.append(("deprojection lands on TRUE world position",
                         None,
                         "INSUFFICIENT: only %d object%s in view (need 2). "
                         "Worst error so far %.4f m against a %.3f m "
                         "tolerance, which is not a verdict. From the home "
                         "pose the cameras frame almost nothing at table "
                         "height -- drive to the SCAN POSE "
                         "(recordings/baselines/scan_pose.json) and re-run."
                         % (seen, "" if seen == 1 else "s", worst, TOL_M)))
        else:
            rows.append(("deprojection lands on TRUE world position",
                         worst <= TOL_M,
                         "%d objects in view, worst error %.4f m (tol %.3f)"
                         % (seen, worst, TOL_M)))

        # CONTROL: compose the rotation the WRONG way round. It must fail.
        bad = []
        for o in objs:
            pw = np.asarray(o["xyz"], float)
            pc = R_wc.T @ (pw - p_cam)
            if pc[2] <= 0.05:
                continue
            u = int(round(K[0, 0] * pc[0] / pc[2] + K[0, 2]))
            v = int(round(K[1, 1] * pc[1] / pc[2] + K[1, 2]))
            if not (0 <= u < n.depth.width and 0 <= v < n.depth.height):
                continue
            z = dep[v, u]
            if z <= 0.0:
                continue
            back = p_cam + R_wc.T @ np.array([(u - K[0, 2]) * z / K[0, 0],
                                              (v - K[1, 2]) * z / K[1, 1], z])
            h = 0.5 * np.asarray(o["size"], float)
            bad.append(float(np.linalg.norm(
                np.maximum(np.abs(back - pw) - h, 0.0))))
        cworst = max(bad, default=0.0)
        rows.append(("CONTROL: transposed rotation must FAIL",
                     cworst > TOL_M,
                     "worst %.4f m -- if this passed, the check above "
                     "proves nothing" % cworst))
    finally:
        try:
            n.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(cam.pid), 15)
        except Exception:
            pass
    return _report(rows)


def _report(rows):
    print("=" * 70)
    print("MOCK CAMERA -- PLUMBING ONLY. Detection accuracy is NOT tested")
    print("=" * 70)
    bad = unk = 0
    for label, ok, detail in rows:
        # None is INSUFFICIENT: the check could not be run, which is neither
        # a pass nor a failure. Collapsing it into either one is how a
        # correct pipeline gets reported as broken and, worse, how a broken
        # one gets reported as untested and ignored.
        word = "??" if ok is None else ("OK" if ok else "FAIL")
        print("   %-46s %-4s %s" % (label, word, detail))
        if ok is None:
            unk += 1
        elif not ok:
            bad += 1
    print("\n%d of %d checks fail%s"
          % (bad, len(rows),
             ", %d could not be run (??)" % unk if unk else ""))
    print("\nUNVERIFIED WITHOUT THE REAL KINOVA DRIVER: detection rate and "
          "accuracy,\nthe real intrinsics/distortion/extrinsic, exposure, "
          "motion blur, rolling\nshutter, depth holes, and whether the "
          "vision module streams at all over\nthis machine's network path.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
