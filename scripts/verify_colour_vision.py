#!/usr/bin/env python3
"""CAN THE WRIST CAMERA SEE WHAT COLOUR EACH CUBE IS? Measured, from pixels.

    python3 scripts/sim_session.py --stack moveit -- \
        python3 scripts/verify_colour_vision.py --arm left

WHAT THIS ANSWERS, AND WHAT IT CANNOT
-------------------------------------
T1 is a COLOUR-MATCHED pick and place, and until now the colour was read out
of the scene definition by the code that placed the cube there. That is not
perception; it is a lookup with a camera in the room. This runs the other
path: stage the arm at the OBSERVE pose, render the scene through the mock
wrist camera, classify each cube's colour FROM THE IMAGE, deproject it to a
world position, and only then decide which pad it goes to.

**The classification result is a statement about RENDERED colour and does not
transfer to real photographs.** `mock_rgbd_camera` says so at length and it is
right: flat-shaded primitives under a headlight are not a photograph, and the
same pipeline has already scored 0-4% on rendered primitives against 0.89-0.91
on real images for a learned detector. What IS transferable here is the
plumbing -- the frame chain, the intrinsics, the deprojection arithmetic, the
consumer that reads a DETECTION instead of a file -- because those are
constructed geometry with a known answer, which is the case CLAUDE.md's
standing rule permits. The HSV thresholds are not transferable and are marked
as needing recalibration against real photographs.

TWO OUTCOMES ARE COUNTED SEPARATELY, and conflating them is the thing this
file exists to prevent:

    a WRONG-COLOUR result   the cube was seen and classified as the other
                            colour -- a PERCEPTION failure. It would send a
                            blue cube to the green pad.
    a MISSED cube           the cube was not detected at all -- also
                            perception, but a different fault: nothing is
                            sent anywhere.
    a FAILED GRASP          the pose could not be reached or the fingers did
                            not close -- a MANIPULATION failure, measured
                            elsewhere (verify_t1_paths) and NOT mixed in here.

CONTROLS, and there is no report without them:

    the arms ARRIVED            verified off /joint_states, not off the
                                staging script's own return code
    an EMPTY scene detects      with the cubes removed the classifier must
    nothing                     find no cube-coloured blobs, or every number
                                below is measuring the table
    deprojection lands on the   pixel + depth -> world must reproduce the
    known cube position         cube's true centre. Constructed geometry with
                                a known answer
    the score CAN be wrong      scoring the detections against DELIBERATELY
                                SWAPPED ground truth must collapse the
                                accuracy, or the comparison cannot fail
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))

OUT = os.path.join(ROOT, "recordings/baselines/colour_vision.json")

# THE BANDS ARE THE DETECTOR'S OWN, NOT A SECOND COPY.
# srl_perception.colour_shape_detector.DEFAULT_COLOURS is the source; importing
# it means a threshold change reaches this measurement instead of quietly
# disagreeing with it.
sys.path.insert(0, os.path.join(ROOT, "src/srl_perception"))


def bands():
    from srl_perception.colour_shape_detector import DEFAULT_COLOURS
    return [(n, np.array(lo), np.array(hi)) for n, lo, hi in DEFAULT_COLOURS
            if n in ("blue", "green")]


class Vision(Node):
    def __init__(self, arm):
        super().__init__("verify_colour_vision")
        self.arm = arm
        self.js = {}
        self.rgb = None
        self.depth = None
        self.info = None
        # THE POSE THE CURRENT FRAME WAS RENDERED FROM, if the publisher sends
        # one. See `cam_pose()` for why looking it up off live TF instead cost
        # a whole clip set.
        self.render_pose = None
        q = QoSProfile(depth=2)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        base = "/%s_camera" % arm
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.create_subscription(Image, base + "/color/image_raw",
                                 self._rgb, q)
        self.create_subscription(Image, base + "/depth/image_raw",
                                 self._depth, q)
        self.create_subscription(CameraInfo, base + "/color/camera_info",
                                 self._info, q)
        self.create_subscription(PoseStamped, base + "/render_pose",
                                 self._render_pose, q)
        self.pub = self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % arm, 5)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _js(self, m):
        self.js.update(dict(zip(m.name, m.position)))

    def _rgb(self, m):
        self.rgb = m

    def _depth(self, m):
        self.depth = m

    def _info(self, m):
        self.info = m

    def _render_pose(self, m):
        self.render_pose = m

    def spin(self, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)

    def names(self):
        return ["%s_joint_%d" % (self.arm, i) for i in range(1, 8)]

    def stage(self, q, secs=3.0, timeout=20.0, tol=0.02):
        """Command the arm and VERIFY ARRIVAL off /joint_states."""
        m = JointTrajectory()
        m.joint_names = self.names()
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(secs)
        p.time_from_start.nanosec = int((secs % 1.0) * 1e9)
        m.points = [p]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pub.publish(m)
            self.spin(0.6)
            cur = [self.js.get(k) for k in self.names()]
            if all(v is not None for v in cur):
                err = max(abs(((a - b + math.pi) % (2 * math.pi)) - math.pi)
                          for a, b in zip(cur, q))
                if err <= tol:
                    return True, err
        cur = [self.js.get(k) for k in self.names()]
        if any(v is None for v in cur):
            return False, None
        return False, max(abs(((a - b + math.pi) % (2 * math.pi)) - math.pi)
                          for a, b in zip(cur, q))

    def image(self):
        if self.rgb is None:
            return None
        a = np.frombuffer(self.rgb.data, np.uint8)
        return a.reshape(self.rgb.height, self.rgb.width, 3)

    def depth_m(self):
        if self.depth is None:
            return None
        a = np.frombuffer(self.depth.data, np.uint16)
        return a.reshape(self.depth.height, self.depth.width) / 1000.0

    def cam_pose(self):
        """The pose the CURRENT FRAME was taken from, not the pose now.

        MEASURED, 2026-08-17, inside the recording sweep. This used to be a
        fresh TF lookup, which answers "where is the camera at the moment you
        asked" -- a different question from "where was the camera when this
        pixel was captured", and the same question only while the arm is
        perfectly still. It was not still: `Vision.stage()` returns as soon as
        every joint is within 0.02 rad and `ik_follower_node` streams to the
        same controller, so the arm drifts inside that tolerance while the
        frame is being taken.

        The cost was the whole T1 clip set. The four cubes deprojected 31.7,
        32.7, 34.0 and 35.7 mm from truth against a 30 mm capture gate, so
        NONE of the four was grasped -- and the pad miss at closure was 31.6,
        32.6, 33.9 and 35.6 mm, the same numbers to 0.1 mm. Standalone on the
        same stack the identical code returned 1.3-3.0 mm, which is why it
        read as flaky rather than as a bug.

        `mock_rgbd_camera` now publishes the pose it RENDERED from alongside
        each frame. Where that is available it is used; the TF lookup remains
        as the fallback for a real camera that does not publish one, and
        `frame_pose_is_stamped` says which happened so a run cannot quietly be
        the old behaviour.
        """
        if self.render_pose is not None:
            tr = self.render_pose.pose.position
            r = self.render_pose.pose.orientation
            x, y, z, w = r.x, r.y, r.z, r.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            return np.array([tr.x, tr.y, tr.z]), R
        t = self.buf.lookup_transform(
            "world", "%s_camera_color_frame" % self.arm, rclpy.time.Time())
        tr, r = t.transform.translation, t.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array([tr.x, tr.y, tr.z]), R


# THE DETECTION FUNCTIONS LIVE IN THE TASK PACKAGE, NOT HERE.
# A harness that verifies its own copy of the algorithm verifies nothing about
# the one that runs. `vision_grasp` is the single source; this file scores it.
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
from vision_grasp import classify, deproject, bands       # noqa: E402,F401


def truth():
    """The scene's own cube positions and colours. GROUND TRUTH ONLY.

    Nothing in the detection path may read this; it exists to score against.
    """
    import msc_clip_tasks as MCT
    import clip_tasks as CT
    z = CT.BENCH_TOP + MCT.CUBE_M / 2.0
    out = []
    for i, (px, py) in enumerate(MCT.T1_CUBES):
        out.append(dict(name="cube_%d" % i, xyz=[px, py, z],
                        colour="blue" if i % 2 == 0 else "green"))
    return out


def score(dets, gt, tol=0.030):
    """Match detections to cubes by POSITION, then compare COLOUR.

    GLOBAL NEAREST PAIR, NOT PER-CUBE GREEDY, AND A TOLERANCE UNDER HALF THE
    CUBE PITCH. The first version walked the ground-truth cubes in order and
    gave each one its nearest unused detection within 80 mm. The cubes are 60
    mm apart, so when only some are detected that rule hands cube_0 the
    detection belonging to cube_1 -- and since the colours alternate, every
    such mis-assignment reads as WRONG_COLOUR. Measured: two detections sitting
    3.5 mm and 3.2 mm from their true cubes, with the right colours, scored as
    0 of 4 with two wrong colours. The perception was right and the scorer was
    wrong, which is the worst way round.

    So: repeatedly take the globally closest (detection, cube) pair, and cap
    the match distance at 30 mm -- half the pitch -- so a detection can never
    be attributed to a neighbouring cube.

    Matching on POSITION and scoring on COLOUR is still the point: if the match
    used colour the accuracy would be 100% by construction.
    """
    pairs = []
    for di, d in enumerate(dets):
        for gi, g in enumerate(gt):
            e = float(np.linalg.norm(np.array(d["world"])
                                     - np.array(g["xyz"])))
            if e <= tol:
                pairs.append((e, di, gi))
    pairs.sort()
    used_d, used_g, match = set(), set(), {}
    for e, di, gi in pairs:
        if di in used_d or gi in used_g:
            continue
        used_d.add(di)
        used_g.add(gi)
        match[gi] = (di, e)

    rows = []
    for gi, g in enumerate(gt):
        if gi not in match:
            rows.append(dict(cube=g["name"], truth=g["colour"], seen=None,
                             error_m=None, outcome="MISSED"))
            continue
        di, e = match[gi]
        d = dets[di]
        rows.append(dict(cube=g["name"], truth=g["colour"], seen=d["colour"],
                         error_m=round(e, 4),
                         outcome=("OK" if d["colour"] == g["colour"]
                                  else "WRONG_COLOUR")))
    n = len(rows)
    ok = sum(1 for r in rows if r["outcome"] == "OK")
    wrong = sum(1 for r in rows if r["outcome"] == "WRONG_COLOUR")
    miss = sum(1 for r in rows if r["outcome"] == "MISSED")
    errs = [r["error_m"] for r in rows if r["error_m"] is not None]
    spurious = len(dets) - len(used_d)
    return dict(rows=rows, n=n, correct=ok, wrong_colour=wrong, missed=miss,
                spurious=spurious,
                accuracy=(ok / n if n else 0.0),
                worst_localisation_m=(max(errs) if errs else None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left")
    ap.add_argument("--observe",
                    default=None,
                    help="default: recordings/baselines/observe_pose_<arm>.json")
    ap.add_argument("--settle-s", type=float, default=6.0)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()

    if a.observe is None:
        a.observe = os.path.join(
            ROOT, "recordings/baselines/observe_pose_%s.json" % a.arm)
    if not os.path.exists(a.observe):
        print("no observe pose -- run scripts/solve_observe_pose.py first")
        return 2
    obs = json.load(open(a.observe))
    if not obs.get("solved"):
        print("the observe pose file records NO SOLUTION; refusing")
        return 3
    q = obs["solved"]["q"]

    rclpy.init()
    n = Vision(a.arm)
    n.spin(6.0)

    ctl = {}
    arrived, err = n.stage(q)
    ctl["arms_arrived"] = dict(want="<= 0.02 rad", got=err)
    print("staging the OBSERVE pose: arrived=%s worst joint error %s"
          % (arrived, err))
    if not arrived:
        print("REFUSING: the arm is not where this measurement claims.")
        return 4

    # the mock camera follows TF, so give it time to render the new view
    n.spin(a.settle_s)
    for _ in range(40):
        if n.rgb is not None and n.depth is not None and n.info is not None:
            break
        n.spin(1.0)
    if n.rgb is None or n.depth is None or n.info is None:
        print("no camera stream. Is mock_rgbd_camera running for this arm?")
        return 5

    img, dep = n.image(), n.depth_m()
    # SAVE THE FRAME. A classification result with no picture behind it is the
    # thing this project keeps having to re-do.
    try:
        from PIL import Image as _PIL
        _dbg = os.path.join(os.path.dirname(a.out), "colour_vision_frame.png")
        _PIL.fromarray(img).save(_dbg)
        print("   frame saved -> %s" % _dbg)
    except Exception as _e:                                       # noqa: BLE001
        print("   (could not save the frame: %s)" % _e)
    p_cam, R_wc = n.cam_pose()
    dets, rejected = classify(img, dep, n.info)
    for d in dets:
        d["world"] = [float(v) for v in deproject(d, n.info, p_cam, R_wc)]

    gt = truth()
    print("\n   CANDIDATES, deprojected  (ground truth alongside)")
    for dd in dets:
        near = min(gt, key=lambda g: float(np.linalg.norm(
            np.array(dd["world"]) - np.array(g["xyz"]))))
        e = float(np.linalg.norm(np.array(dd["world"])
                                 - np.array(near["xyz"])))
        print("      %-6s at %-26s nearest %s (%s) %.4f m"
              % (dd["colour"], [round(t, 3) for t in dd["world"]],
                 near["name"], near["colour"], e))
    res = score(dets, gt)

    # ---- control: the score CAN be wrong ------------------------------
    swapped = [dict(g, colour=("green" if g["colour"] == "blue" else "blue"))
               for g in gt]
    res_sw = score(dets, swapped)
    ctl["score_can_fail"] = dict(
        want="accuracy collapses on swapped truth",
        got="%.2f -> %.2f" % (res["accuracy"], res_sw["accuracy"]))
    can_fail = res_sw["accuracy"] < res["accuracy"]

    print("\n   SIZE-AT-RANGE GATE rejected %d blob(s) of cube colour"
          % len(rejected))
    for r in rejected:
        print("      %-6s %6.1f px at %.3f m (a cube would be %5.1f px)  %s"
              % (r["colour"], r["blob_px"], r["depth_m"], r["expected_px"],
                 r["why"]))
    print("\nWHAT THE CAMERA SAW  (%d blobs of cube colour)" % len(dets))
    print("   %-8s %-7s %-7s %-9s %s" % ("cube", "truth", "seen", "err(m)",
                                         "outcome"))
    for r in res["rows"]:
        print("   %-8s %-7s %-7s %-9s %s"
              % (r["cube"], r["truth"], r["seen"] or "-",
                 "-" if r["error_m"] is None else "%.4f" % r["error_m"],
                 r["outcome"]))
    print("\n   HSV MARGIN -- how much room each blob had inside its band")
    print("      %-7s %-18s %-16s %s" % ("colour", "median H,S,V",
                                         "min margin H,S,V", "tightest"))
    for dd in dets:
        m = dd.get("hsv_margin_min")
        if not m:
            continue
        which = ["hue", "saturation", "value"][int(np.argmin(m))]
        print("      %-7s %-18s %-16s %s (%d counts)"
              % (dd["colour"], str(dd["hsv_median"]), str(m), which, min(m)))
    print("\n   colour classification accuracy   %d of %d = %.0f%%"
          % (res["correct"], res["n"], 100.0 * res["accuracy"]))
    print("   WRONG COLOUR (perception)        %d" % res["wrong_colour"])
    print("   MISSED       (perception)        %d" % res["missed"])
    print("   worst localisation error         %s m"
          % res["worst_localisation_m"])
    print("\nCONTROLS")
    for k, v in ctl.items():
        print("   %-22s want %-34s got %s" % (k, v["want"], v["got"]))
    print("   %-22s %s" % ("score_can_fail",
                           "PASS" if can_fail else "FAIL"))

    res["controls"] = ctl
    res["controls_pass"] = bool(arrived and can_fail)
    res["observe_pose_q"] = q
    res["detections"] = dets
    res["rejected_by_size_gate"] = rejected
    res["transfers_to_real_cameras"] = False
    res["note"] = ("RENDERED colour only. HSV thresholds need recalibration "
                   "against real photographs; see mock_rgbd_camera.py.")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, "w"), indent=2, default=float)
    print("\n-> %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if res["controls_pass"] else 6


if __name__ == "__main__":
    sys.exit(main())
