#!/usr/bin/env python3
"""U-1: RUN THE WHOLE DETECTION PATH, ONCE, END TO END.

    python3 scripts/verify_detection_path.py       # needs the stack up

    camera -> detector -> tracker -> /perception/objects -> yaw

Until the mock RGB-D publisher was wired into a launch path there was NO
camera publisher in sim, so this chain had never run at all. Every node in it
had been tested on its own against a fixture; not one of the JOINS had. This
script runs the chain and reports what came out of each one, so a break has a
place rather than being "perception does not work".

It also checks the U-2 fix where it actually bites: the fallback detector used
to publish an identity quaternion for every object, `grasp_generator` read
that as yaw = 0, and an object at an angle got a square grasp with nothing
able to tell that from a square object. So the last check asks whether the
yaw arriving at the consumer is MEASURED or explicitly UNKNOWN. Both are
acceptable answers; a fabricated zero is not.

WHAT IT DOES NOT AND CANNOT ESTABLISH, and this is the whole reason the mock
has a warning banner: DETECTION RATE. Flat-shaded primitives on a flat
background score 0-4% for a learned detector that scores 0.89-0.91 on a real
photograph. The colour/shape fallback used here is a threshold on hue, so it
finds the mock's flat colours essentially perfectly and that number means
NOTHING about a real table. Detection at working distance stays UNMEASURED
until the Kinova cameras exist.
"""

import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_perception"))

import find_scan_pose as FSP                                   # noqa: E402
from srl_perception import scene_fingerprint as sf             # noqa: E402

POSES = os.path.join(WS, "recordings", "baselines", "scan_pose.json")
TASK = "t1"
# THE ARM COMES FROM THE TASK, NEVER FROM A LITERAL. This was "left" for one
# run and found nothing: T1's cubes moved to the RIGHT arm and sit at negative
# x, so the left scan camera correctly frames an empty region. An empty region
# and a broken detector look identical from here, which is the whole reason
# this file exists, so the side is derived.
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments",
                                "experiments", "abc"))
import msc_clip_tasks as MCT                                   # noqa: E402
ARM = MCT.T1_ARM


class Sink(Node):
    def __init__(self, arm):
        super().__init__("detection_path_probe")
        self.raw = None
        self.tracked = None
        self.diag = None
        self.create_subscription(
            Detection3DArray, "/perception/detections/%s" % arm,
            lambda m: setattr(self, "raw", m), 10)
        self.create_subscription(
            Detection3DArray, "/perception/objects",
            lambda m: setattr(self, "tracked", m), 10)
        self.create_subscription(
            String, "/perception/fallback_diag/%s" % arm,
            lambda m: setattr(self, "diag", m.data), 10)

    def spin(self, secs):
        t = time.time()
        while time.time() - t < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    if not os.path.exists(POSES):
        print("no %s -- run scripts/find_scan_pose.py --save first" % POSES)
        return 2
    d = json.load(open(POSES))["poses"][ARM]
    eye = np.asarray(d["cam_xyz"], float)
    tgt = np.asarray(d["target"], float)
    q = FSP._R_to_q(FSP._look_at(eye, tgt))
    frame = "scan_cam_%s" % ARM
    env = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS="SHM")
    P = os.path.join(WS, "src", "srl_perception", "srl_perception")

    procs = [
        subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "--x", str(eye[0]), "--y", str(eye[1]), "--z", str(eye[2]),
             "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]),
             "--qw", str(q[3]), "--frame-id", "world",
             "--child-frame-id", frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
        subprocess.Popen(
            ["python3", "-u", os.path.join(P, "mock_rgbd_camera.py"),
             "--ros-args", "-p", "arm:=" + ARM, "-p", "task:=" + TASK,
             "-p", "frame_id:=" + frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
        subprocess.Popen(
            ["python3", "-u", os.path.join(P, "colour_shape_detector.py"),
             "--ros-args", "-p", "arm:=" + ARM, "-p", "enabled:=true",
             "-p", "object_width_m:=0.04"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
        subprocess.Popen(
            ["python3", "-u", os.path.join(P, "object_pose_tracker.py")],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
    ]
    rows = []
    try:
        rclpy.init()
        s = Sink(ARM)
        s.spin(22.0)

        # DETECTIONS, NOT A MESSAGE. The first version asked only whether a
        # message arrived, and an EMPTY Detection3DArray satisfied it -- so
        # a run that found nothing at all was reported OK on the strength of
        # the topic existing.
        n_raw = len(s.raw.detections) if s.raw else 0
        rows.append(("the camera reaches the DETECTOR", n_raw > 0,
                     "%d raw detections on the %s camera (task %s puts its "
                     "objects on the %s arm)"
                     % (n_raw, ARM, TASK, MCT.T1_ARM)))
        rows.append(("the detector reaches the TRACKER",
                     s.tracked is not None and bool(s.tracked.detections),
                     "%d tracked objects on /perception/objects"
                     % (len(s.tracked.detections) if s.tracked else 0)))

        # ---- U-2 AT THE CONSUMER'S END ------------------------------
        # This is the read grasp_generator does. An identity quaternion here
        # is the fault: it means the consumer will assume square without
        # anything saying so.
        states = []
        for det in (s.tracked.detections if s.tracked else []):
            if not det.results:
                continue
            o = det.results[0].pose.pose.orientation
            yaw = sf.yaw_from_quat((o.x, o.y, o.z, o.w))
            states.append((det.id or det.results[0].hypothesis.class_id,
                           None if yaw is None else round(yaw, 1)))
        measured = [n for n, y in states if y is not None]
        unknown = [n for n, y in states if y is None]
        rows.append(("yaw arrives as MEASURED or as explicitly UNKNOWN",
                     bool(states),
                     "%d measured %s, %d unknown %s"
                     % (len(measured), measured[:3],
                        len(unknown), unknown[:3])))

        # The detector's own diagnostic has to agree with what it published;
        # two descriptions of one measurement is how the last three faults
        # here stayed invisible.
        agree = None
        if s.diag:
            try:
                dd = json.loads(s.diag)
                n_known = sum(1 for x in dd.get("detections", [])
                              if x.get("yaw_known"))
                agree = (n_known > 0) == (len(measured) > 0)
            except ValueError:
                agree = False
        rows.append(("the detector's diagnostic agrees with its wire output",
                     bool(agree),
                     (s.diag or "no diagnostic published")[:100]))
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:                                  # noqa: BLE001
                pass
        try:
            rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass

    print("=" * 72)
    print("DETECTION PATH, END TO END -- PLUMBING ONLY")
    print("=" * 72)
    bad = 0
    for label, ok, detail in rows:
        print("   %-50s %-4s %s" % (label, "OK" if ok else "FAIL", detail))
        bad += not ok
    print("\n%d of %d checks fail" % (bad, len(rows)))
    print("\nNO DETECTION RATE MAY BE READ OFF THIS RUN. The mock renders "
          "flat colour\nand the fallback detector thresholds hue, so it finds "
          "everything by\nconstruction. Detection at working distance is "
          "UNMEASURED.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
