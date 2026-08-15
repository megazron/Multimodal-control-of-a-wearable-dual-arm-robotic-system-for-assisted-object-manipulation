#!/usr/bin/env python3
"""From the SCAN POSE, does the camera actually see the work surface?

    python3 scripts/verify_scan_view.py

The scan pose is chosen geometrically (find_scan_pose.py: the work region's
corners must project inside the image, and the pose must solve IK 10/10). This
script asks the different and more useful question -- what does the CAMERA
produce from there -- by pointing the mock RGB-D node at the scan pose through
a static transform and reading the images off the topics.

WHY A STATIC TRANSFORM RATHER THAN MOVING THE ARM. The mock takes its pose
from TF for whatever frame it is told to use, so publishing `world -> scan_cam
_<arm>` and running the mock on that frame renders exactly the view the wrist
camera will have when the arm is at the scan pose. No follower, no trajectory,
no risk of measuring a half-finished motion -- and the arm being verifiably at
home while it runs is a stronger guarantee than trusting it arrived.

WHAT IT CANNOT TELL YOU is the same list as always: this is the mock, so the
APPEARANCE is not the real camera's and no detection number may be read off
it. What it does establish is that the work surface, the coloured planes and
the objects fall inside the frame from the scan pose -- which is the thing
that is NOT true at the home pose, and the thing somebody would otherwise
discover with real hardware in front of them.
"""
import json
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
for p in ("find_scan_pose",):
    pass
import find_scan_pose as FSP                                  # noqa: E402

POSES = os.path.join(WS, "recordings", "baselines", "scan_pose.json")


class Sink(Node):
    def __init__(self, arm):
        super().__init__("scan_view_sink")
        self.color = None
        self.create_subscription(Image, "/%s_camera/color/image_raw" % arm,
                                 lambda m: setattr(self, "color", m), 1)

    def spin(self, s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    if not os.path.exists(POSES):
        print("no %s -- run find_scan_pose.py --save first" % POSES)
        return 2
    d = json.load(open(POSES))["poses"]

    rows = []
    for arm in ("left", "right"):
        if arm not in d:
            rows.append((arm, False, "no scan pose saved"))
            continue
        eye = np.asarray(d[arm]["cam_xyz"], float)
        tgt = np.asarray(d[arm]["target"], float)
        R = FSP._look_at(eye, tgt)
        q = FSP._R_to_q(R)
        frame = "scan_cam_%s" % arm

        env = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS="SHM")
        stf = subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "--x", str(eye[0]), "--y", str(eye[1]), "--z", str(eye[2]),
             "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]),
             "--qw", str(q[3]), "--frame-id", "world",
             "--child-frame-id", frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        cam = subprocess.Popen(
            ["python3", "-u",
             os.path.join(WS, "src", "srl_perception", "srl_perception",
                          "mock_rgbd_camera.py"),
             "--ros-args", "-p", "arm:=" + arm, "-p", "task:=t1",
             "-p", "frame_id:=" + frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        rclpy.init()
        n = Sink(arm)
        try:
            n.spin(14.0)
            if n.color is None:
                rows.append((arm, False, "no image published"))
                continue
            a = np.frombuffer(n.color.data, np.uint8).reshape(
                n.color.height, n.color.width, 3).astype(int)
            r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
            # DETECT BY CHANNEL RELATIONS, not by exact RGB. The clip verifier
            # learned this the hard way: RViz shades every surface, so an
            # exact match against the requested colour fails on frames whose
            # object is plainly visible. Here the mock paints flat colour, but
            # the same rule is used so the two verifiers cannot disagree.
            # THE TABLE IS WHITE NOW, AND THIS DETECTOR WAS LOOKING FOR OAK.
            #
            # It required b < 130 and r > b + 40. The table top renders at
            # (240, 240, 242) -- clip_scene.OAK, which kept its name and lost
            # its colour on 2026-08-15 -- so BOTH conditions are unsatisfiable
            # and `surface_ok` could only ever be False. This script would
            # have reported "the camera cannot see the work surface" for a
            # frame filled with it.
            #
            # The same colour change is SAFE for the object detectors and
            # BREAKS this one, for one reason: a neutral has R = G = B, so it
            # cannot fire a channel-difference test. The objects are found by
            # channel difference and must not match the table; the table is
            # the thing that must match. So it is keyed on what a neutral
            # actually is -- bright, and nearly equal in all three -- rather
            # than on a hue it no longer has.
            lo = np.minimum(np.minimum(r, g), b)
            hi = np.maximum(np.maximum(r, g), b)
            surface = int(((lo > 150) & (hi - lo < 25)).sum())
            blue = int(((b > 120) & (b > r + 60) & (b > g + 60)).sum())
            green = int(((g > 120) & (g > r + 60) & (g > b + 40)).sum())
            tot = a.shape[0] * a.shape[1]
            # HOW MANY T1 OBJECTS ARE ON THIS ARM'S SIDE AT ALL? T1 is a
            # ONE-ARM task, so the camera on the OTHER side correctly frames a
            # work surface with nothing on it. THE SIDE IS NOT NAMED HERE --
            # the code below reads MCT.T1_CUBES, which is why it kept working
            # through two layout moves while this comment named the wrong arm
            # each time. It said RIGHT until 2026-08-15, by which point stage
            # 1 had been back on the LEFT for three days. Demanding objects in
            # both views would fail a working scan pose for the layout's
            # reason, which is the by-design-versus-real-gap confusion the
            # status table just had to be fixed for.
            sys.path.insert(0, os.path.join(
                WS, "src", "srl_experiments", "experiments", "abc"))
            import msc_clip_tasks as MCT
            sys.path.insert(0, os.path.join(WS, "src", "srl_experiments"))
            from srl_experiments.by_design import Expectation
            side = 1.0 if arm == "left" else -1.0
            n_obj = sum(1 for x, _y in MCT.T1_CUBES if x * side > 0)
            # Same helper the status table uses: "no objects visible" is a
            # GAP only where the layout puts objects.
            obj_exp = Expectation(
                {a for a in ("left", "right")
                 if any(x * (1.0 if a == "left" else -1.0) > 0
                        for x, _y in MCT.T1_CUBES)},
                label="the T1 layout",
                reason=("T1 is a one-arm task -- all %d cubes on one side, "
                        "read from MCT.T1_CUBES" % len(MCT.T1_CUBES)))
            verdict, why = obj_exp.classify(arm, present=bool(blue or green))
            surface_ok = surface > 0.02 * tot
            if n_obj:
                ok = surface_ok and blue > 0 and green > 0
                det = ("white table %.1f%% of frame, blue %d px, green %d px "
                       "(%d T1 objects this side)"
                       % (100.0 * surface / tot, blue, green, n_obj))
            else:
                ok = surface_ok
                det = ("white table %.1f%% of frame; NO T1 objects on this "
                       "side by design (T1 is a one-arm task and all %d "
                       "cubes are on the other side) -- surface framing "
                       "verified, object visibility n/a"
                       % (100.0 * surface / tot, len(MCT.T1_CUBES)))
            rows.append((arm, ok, det))
        finally:
            try:
                n.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
            for p in (cam, stf):
                try:
                    os.killpg(os.getpgid(p.pid), 15)
                except Exception:
                    pass
            time.sleep(1.0)

    print("=" * 70)
    print("SCAN-POSE VIEW -- what the mock camera renders from the scan pose")
    print("=" * 70)
    bad = 0
    for arm, ok, detail in rows:
        print("   %-6s %-4s %s" % (arm, "OK" if ok else "FAIL", detail))
        bad += not ok
    print("\n%d of %d fail" % (bad, len(rows)))
    print("\nCompare with the HOME pose, where the bench projects to pixel "
          "(954, 539)\nand is outside a 640-wide frame entirely. Appearance "
          "is the mock's, not the\nreal camera's -- no detection number may "
          "be read off this.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
