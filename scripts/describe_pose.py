#!/usr/bin/env python3
"""WHICH POSE AM I LOOKING AT? Joint angles and the geometry, side by side.

    python3 scripts/describe_pose.py            # needs the stack

Answers one question and answers it with numbers: for HOME and for the stored
PRESENTATION pose, what are the joint angles, where do the hands and elbows
actually end up, and how wide does the robot stand.

WHY IT EXISTS. A pose in a screenshot is not self-identifying, and the two
that matter here differ in ways a description can straddle -- both put the
hands forward of the body, and "splayed" is a judgement until it is a number.
Getting it wrong in either direction is expensive:

  * calling HOME the presentation pose would invite a change to the home
    angles, which are ground truth read off the physical arms. HARD
    CONSTRAINT 1: the sim to real bridge replays sim angles onto the real
    arm, so any edit is commanded as a jump. See home_wrist_is_real.md.
  * calling the PRESENTATION pose home would leave a redesignable pose
    untouched and send the search somewhere it cannot help.

THE DISCRIMINATOR IS THE WRIST. At home the tool axis points UP, ~+31 deg
(left) and ~+22 (right) above horizontal; the presentation pose brings it to
about +7. Nothing else in the two poses differs, because the staging move
turns one wrist joint and nothing else.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "config"))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_teleop"))
from srl_teleop import mount_guard_node as MG                  # noqa: E402

POSE_FILE = os.path.join(ROOT, "recordings", "baselines",
                         "presentation_pose.json")
# The wearer's torso, from human_backpack.xacro via mount_guard_node: a
# 0.36 x 0.22 x 0.48 box centred at z = 1.22. Half-width 0.18, which is the
# number "roughly the width of the torso" refers to.
TORSO_HALF_W = 0.18
SHOULDER = "shoulder_link"
ELBOW = "forearm_link"
HAND = "end_effector_link"


class FK(Node):
    def __init__(self):
        super().__init__("describe_pose")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.js = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)

    def spin(self, s):
        t = time.time()
        while time.time() - t < s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def links(self, arm, q, names):
        rs = RobotState()
        nm = ["%s_joint_%d" % (arm, i + 1) for i in range(7)]
        pos = [float(v) for v in q]
        if self.js is not None:
            for n, p in zip(self.js.name, self.js.position):
                if n not in nm:
                    nm.append(n)
                    pos.append(float(p))
        rs.joint_state.name, rs.joint_state.position = nm, pos
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, n) for n in names]
        req.robot_state = rs
        fut = self.fk.call_async(req)
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.005)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        out = {}
        for n, ps in zip(names, r.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[n] = dict(xyz=(p.x, p.y, p.z),
                          quat=(o.x, o.y, o.z, o.w))
        return out

    def clearance(self, arm, q):
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_%s" % (arm, ln) for ln in MG.CHAIN]
        rs = RobotState()
        nm = ["%s_joint_%d" % (arm, i + 1) for i in range(7)]
        rs.joint_state.name = nm
        rs.joint_state.position = [float(v) for v in q]
        req.robot_state = rs
        fut = self.fk.call_async(req)
        t = time.monotonic()
        while not fut.done() and time.monotonic() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.005)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None, None
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
               for p in r.pose_stamped]
        worst, who = 1e9, None
        for a, b in zip(pts, pts[1:]):
            for k in range(MG.SAMPLES + 1):
                t2 = k / float(MG.SAMPLES)
                pp = [a[j] + (b[j] - a[j]) * t2 for j in range(3)]
                for name, kind, prm, ctr in MG.WEARER:
                    d = MG.dist_point(pp, kind, prm, ctr) - MG.TUBE_R
                    if d < worst:
                        worst, who = d, name
        return worst, who


def elevation(q):
    x, y, z, w = q
    ax = (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
          1.0 - 2.0 * (x * x + y * y))
    return math.degrees(math.atan2(ax[2], math.hypot(ax[0], ax[1])))


def describe(n, label, poses):
    print("\n%s" % label)
    hands = {}
    for arm in ("left", "right"):
        q = poses[arm]
        L = n.links(arm, q, [SHOULDER, ELBOW, HAND])
        if L is None:
            print("   %-5s FK failed" % arm)
            continue
        sh, el, hd = L[SHOULDER]["xyz"], L[ELBOW]["xyz"], L[HAND]["xyz"]
        hands[arm] = hd
        clr, who = n.clearance(arm, q)
        print("   %-5s joints  %s"
              % (arm, " ".join("%+7.1f" % math.degrees(v) for v in q)))
        print("         hand    (%+.3f, %+.3f, %.3f)   wrist axis %+.1f deg"
              % (hd[0], hd[1], hd[2], elevation(L[HAND]["quat"])))
        print("         elbow   (%+.3f, %+.3f, %.3f)   %+.0f mm vs shoulder"
              % (el[0], el[1], el[2], (el[2] - sh[2]) * 1000))
        print("         wearer clearance %.4f m (%s)"
              % (clr if clr is not None else float("nan"), who))
    if len(hands) == 2:
        span = abs(hands["left"][0] - hands["right"][0])
        print("   HAND SPAN %.3f m against a %.3f m torso width  -> %s"
              % (span, 2 * TORSO_HALF_W,
                 "WIDER THAN THE TORSO" if span > 2 * TORSO_HALF_W
                 else "within the torso width"))
        print("   hands forward of the shoulder line by %.3f / %.3f m"
              % (hands["left"][1], hands["right"][1]))
    return hands


def main():
    ap = argparse.ArgumentParser()
    ap.parse_args()
    import home_positions as hp
    rclpy.init()
    n = FK()
    n.spin(2.5)
    if not n.fk.wait_for_service(timeout_sec=20.0):
        print("no /compute_fk -- start the sim")
        return 2

    home = {a: list(hp.load_home_radians(a)) for a in ("left", "right")}
    print("=" * 72)
    print("WHICH POSE IS WHICH -- joint angles and where the arms end up")
    print("=" * 72)
    describe(n, "HOME  (ground truth, read from the physical arms; "
                "MUST NOT CHANGE)", home)

    if os.path.exists(POSE_FILE):
        d = json.load(open(POSE_FILE))
        pres = {a: d["poses"][a]["q"] for a in ("left", "right")}
        describe(n, "PRESENTATION  (a staging pose; free to change)", pres)
        print("\n   difference from home, per joint, in degrees:")
        for a in ("left", "right"):
            print("     %-5s %s" % (a, " ".join(
                "%+6.1f" % math.degrees(pres[a][i] - home[a][i])
                for i in range(7))))
    else:
        print("\nNO STORED PRESENTATION POSE at %s" % POSE_FILE)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
