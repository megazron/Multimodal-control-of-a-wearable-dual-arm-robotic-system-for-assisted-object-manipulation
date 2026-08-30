#!/usr/bin/env python3
"""Prompted, segment-by-segment VR teleoperation measurement.

    python3 scripts/vr_teleop_protocol.py

One motion at a time. The script says what to do, the operator does it and
presses A on the right controller, and the segment is scored before the next
instruction is given. Nothing is recorded between segments, so repositioning
the hand is free -- the same reason `teleop_recorder` gates on a button rather
than a clock.

WHAT EACH SEGMENT MEASURES, AND WHY THESE AND NOT OTHERS
--------------------------------------------------------
The six translation segments are the axes, one at a time, because the open
question this rig has is a FRAME question and it cannot be answered by moving
diagonally. CLAUDE.md records that the repository disagrees with itself about
whether world +x is the wearer's right, and that at align_yaw_deg = 0 the lab
measured left/right inverted, forward/back inverted, and ONLY UP AND DOWN
WORKING -- because a yaw does not touch z, so z is the one axis a wrong yaw
cannot get wrong. That signature is only visible axis by axis.

Each segment reports three different things, which fail independently:

  DIRECTION   the angle between the hand's motion and the arm's. This is the
              frame check. A yaw error shows here as ~180 deg on x and y with
              z clean; a genuine mirror would show as one axis inverted and
              the others not, which no value of align_yaw_deg can produce.
  GAIN        arm distance / hand distance. Should be `scale`. A gain that is
              not `scale` is a scaling bug; a gain that FALLS with speed is
              the rate limiter clipping, which is the accumulating miss.
  TRACKING    how far the commanded pose ends up from where the hand asked,
              and how far the SIM arm ends up from the commanded pose. These
              are different failures: the first is this node's filter and
              limiter, the second is the follower and IK.

COMMANDED AND ACTUAL ARE BOTH READ, because they are not the same claim. The
mapper publishing a pose says nothing about the arm reaching it -- that is the
whole reason this repository draws COMMANDED and ACTUAL side by side.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WS = "/home/gausms/kortex_ws"


def home_radians(arm):
    """The home pose, from THE source every node loads.

    config/home_positions_<arm>.txt, not the URDF's spawn block. Those two
    have drifted apart in the working tree -- left joint_6 by 0.976 rad --
    and homing to the wrong one is exactly the jump HARD CONSTRAINT 1 is
    about. One source, named here so it is obvious which was used.
    """
    d = {}
    for ln in open("%s/config/home_positions_%s.txt" % (WS, arm)):
        ln = ln.strip()
        if ln.startswith("joint_"):
            k, v = ln.split(":")
            d[k.strip()] = math.radians(float(v))
    return [d["joint_%d" % i] for i in range(1, 8)]

# (key, spoken instruction, expected world direction for the RIGHT arm)
# The expected vector is what the operator's motion SHOULD produce in world
# coordinates given the wearer's frame: +y is the direction the wearer faces,
# +z is up, +x is the wearer's right.
SEGMENTS = [
    ("front", "Move BOTH controllers straight AWAY from you (forward), "
              "about 30 cm, slowly.", [0.0, 1.0, 0.0]),
    ("back", "Move them straight BACK toward your chest, about 30 cm.",
     [0.0, -1.0, 0.0]),
    ("right", "Move them to YOUR RIGHT, about 30 cm.", [-1.0, 0.0, 0.0]),
    ("left", "Move them to YOUR LEFT, about 30 cm.", [1.0, 0.0, 0.0]),
    ("up", "Move them straight UP, about 30 cm.", [0.0, 0.0, 1.0]),
    ("down", "Move them straight DOWN, about 30 cm.", [0.0, 0.0, -1.0]),
    ("fast_front", "Move FORWARD again, but FAST this time (a brisk reach).",
     [0.0, 1.0, 0.0]),
    ("yaw", "Keep your hand still and ROTATE the controller (twist your "
            "wrist) about 90 degrees.", None),
]
HANDS = ("left", "right")


class Protocol(Node):
    def __init__(self, arms):
        super().__init__("vr_teleop_protocol")
        self.arms = arms
        self.joy = None
        self.ctrl = {h: None for h in HANDS}
        self.cmd = {h: None for h in HANDS}
        self.mapper = {h: None for h in HANDS}
        self.create_subscription(Joy, "/vr/controller_joy_right",
                                 lambda m: setattr(self, "joy", m), 20)
        for h in HANDS:
            self.create_subscription(
                PoseStamped, "/vr/controller_pose_%s" % h,
                lambda m, h=h: self.ctrl.__setitem__(h, m), 20)
            self.create_subscription(
                PoseStamped, "/master_arm_pose_%s" % h,
                lambda m, h=h: self.cmd.__setitem__(h, m), 20)
            self.create_subscription(
                String, "/vr/mapper_%s" % h,
                lambda m, h=h: self.mapper.__setitem__(h, json.loads(m.data)), 10)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.jpub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 10)
            for a in arms}
        self.q = {}
        self.create_subscription(
            JointState, "/joint_states",
            lambda m: self.q.update(dict(zip(m.name, m.position))), 10)

    # ----------------------------------------------------------------- home
    def home(self, secs=5.0, tol_deg=1.0, label=""):
        """Put every arm back at home and WAIT for it, before measuring.

        EVERY SEGMENT STARTS FROM THE SAME POSE. Without this each segment
        begins wherever the last one left the arm, so the arm is in a
        different part of its workspace each time -- and reach, IK
        conditioning and clearance headroom all vary across it. Two segments
        that differ because the arm started somewhere else are not a
        comparison, and the difference would read as a property of the
        DIRECTION being tested.

        Commanded with the clutch out. The follower only publishes while the
        mapper is engaged, so nothing fights this as long as the operator has
        released the grip -- which the prompt asks for.
        """
        self.spin(0.5)
        before = {}
        for a in self.arms:
            names = ["%s_joint_%d" % (a, i) for i in range(1, 8)]
            q = home_radians(a)
            now = [self.q.get(nm) for nm in names]
            if all(v is not None for v in now):
                before[a] = max(abs(math.degrees(q[i] - now[i]))
                                for i in range(7))
            t = JointTrajectory()
            t.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = Duration(sec=int(secs), nanosec=0)
            t.points = [pt]
            for _ in range(3):
                self.jpub[a].publish(t)
                self.spin(0.05)
        self.spin(secs + 2.0)
        worst = {}
        for a in self.arms:
            names = ["%s_joint_%d" % (a, i) for i in range(1, 8)]
            q = home_radians(a)
            now = [self.q.get(nm) for nm in names]
            worst[a] = (max(abs(math.degrees(q[i] - now[i])) for i in range(7))
                        if all(v is not None for v in now) else float("nan"))
        bits = []
        for a in self.arms:
            bits.append("%s %.2f->%.2f deg" % (a, before.get(a, float("nan")),
                                               worst[a]))
        ok = all((not math.isnan(w)) and w < tol_deg for w in worst.values())
        print("        HOME%s: %s   %s"
              % (label, "  ".join(bits),
                 "at home" if ok else "NOT AT HOME -- segment may be invalid"))
        return ok

    # ------------------------------------------------------------- plumbing
    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.01)

    @staticmethod
    def _p(m):
        return None if m is None else np.array(
            [m.pose.position.x, m.pose.position.y, m.pose.position.z])

    @staticmethod
    def _q(m):
        return None if m is None else np.array(
            [m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w])

    def ee(self, arm):
        """ACTUAL end-effector pose from TF -- where the sim arm really is."""
        try:
            t = self.tf_buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = t.transform.translation
        return np.array([v.x, v.y, v.z])

    def a_pressed(self):
        j = self.joy
        return bool(j is not None and len(j.buttons) > 0 and j.buttons[0])

    def wait_for_a(self, timeout=600.0):
        """Rising edge, then release, so one press cannot end two segments."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin(0.02)
            if self.a_pressed():
                while self.a_pressed() and time.time() - t0 < timeout:
                    self.spin(0.02)
                return True
        return False

    def snapshot(self):
        return dict(
            t=time.time(),
            ctrl={h: self._p(self.ctrl[h]) for h in HANDS},
            ctrl_q={h: self._q(self.ctrl[h]) for h in HANDS},
            cmd={h: self._p(self.cmd[h]) for h in HANDS},
            ee={a: self.ee(a) for a in self.arms})


def ang_between(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return math.degrees(math.acos(
        float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


def score(key, want, s0, s1, peak, node):
    """One segment, scored. Returns a dict; prints the human version."""
    out = {"segment": key, "dt_s": round(s1["t"] - s0["t"], 2)}
    print("\n  %-9s %-11s %-11s %-9s %s"
          % ("", "hand_mm", "cmd_mm", "arm_mm", "notes"))
    for h in HANDS:
        arm = h                                   # one controller per arm
        if arm not in node.arms:
            continue
        c0, c1 = s0["ctrl"][h], s1["ctrl"][h]
        m0, m1 = s0["cmd"][h], s1["cmd"][h]
        e0, e1 = s0["ee"].get(arm), s1["ee"].get(arm)
        if c0 is None or c1 is None:
            print("  %-9s no controller pose" % h)
            continue
        dh = c1 - c0
        dc = None if (m0 is None or m1 is None) else m1 - m0
        de = None if (e0 is None or e1 is None) else e1 - e0
        row = {"hand_mm": round(float(np.linalg.norm(dh)) * 1000, 1),
               "hand_vec": [round(float(v), 4) for v in dh]}
        note = []
        if dc is None:
            note.append("NO COMMAND PUBLISHED (clutch never engaged?)")
        else:
            row["cmd_mm"] = round(float(np.linalg.norm(dc)) * 1000, 1)
            row["cmd_vec"] = [round(float(v), 4) for v in dc]
            if np.linalg.norm(dh) > 0.02:
                row["gain"] = round(float(np.linalg.norm(dc) /
                                          np.linalg.norm(dh)), 3)
        if de is not None:
            row["arm_mm"] = round(float(np.linalg.norm(de)) * 1000, 1)
            row["arm_vec"] = [round(float(v), 4) for v in de]
            if dc is not None and np.linalg.norm(dc) > 0.02:
                row["follow_err_mm"] = round(
                    float(np.linalg.norm(de - dc)) * 1000, 1)
                row["arm_vs_cmd_deg"] = round(ang_between(de, dc), 1)
        if want is not None and de is not None and np.linalg.norm(de) > 0.02:
            row["dir_err_deg"] = round(ang_between(de, np.array(want)), 1)
            if row["dir_err_deg"] > 120:
                note.append("INVERTED vs expected")
            elif row["dir_err_deg"] > 45:
                note.append("off-axis")
        mp = node.mapper.get(h) or {}
        for k in ("cutoff_hz", "hand_speed_mps", "lag_m", "scale"):
            if k in mp:
                row[k] = mp[k]
        pk = peak.get(h, {})
        row["peak_hand_speed_mps"] = round(pk.get("v", 0.0), 3)
        row["max_lag_mm"] = round(pk.get("lag", 0.0) * 1000, 1)
        print("  %-9s %-11s %-11s %-9s %s"
              % (h,
                 row.get("hand_mm", "--"),
                 row.get("cmd_mm", "--"),
                 row.get("arm_mm", "--"),
                 "; ".join(note) or ""))
        if "gain" in row:
            print("            gain %.3f (scale %s)   peak hand %.2f m/s   "
                  "max lag %.1f mm"
                  % (row["gain"], row.get("scale", "?"),
                     row["peak_hand_speed_mps"], row["max_lag_mm"]))
        if "dir_err_deg" in row:
            print("            arm direction %.1f deg from EXPECTED  "
                  "(arm vs commanded %.1f deg, follow error %.1f mm)"
                  % (row["dir_err_deg"], row.get("arm_vs_cmd_deg", float("nan")),
                     row.get("follow_err_mm", float("nan"))))
        out[h] = row
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", nargs="+", default=["left", "right"])
    ap.add_argument("--out", default="")
    ap.add_argument("--only", nargs="*", default=None,
                    help="run only these segment keys")
    a = ap.parse_args()

    rclpy.init()
    n = Protocol(a.arms)
    print("waiting for the VR graph ...")
    n.spin(3.0)
    if all(v is None for v in n.ctrl.values()):
        print("No controller poses on /vr/controller_pose_*. Is the bridge up "
              "and the headset page open?")
        return 2

    segs = [s for s in SEGMENTS
            if a.only is None or s[0] in a.only]
    results = []
    print("\n" + "=" * 74)
    print("VR TELEOPERATION PROTOCOL -- %d segments" % len(segs))
    print("For each one: CLUTCH IN (hold the grip), do the motion, release,")
    print("then press A on the RIGHT controller to close the segment.")
    print("=" * 74)
    print("\nReturning both arms to home before we start ...")
    n.home(label=" (start)")

    for i, (key, text, want) in enumerate(segs, 1):
        print("\n" + "-" * 74)
        print("[%d/%d]  %s" % (i, len(segs), key.upper()))
        print("        %s" % text)
        print("        ... then press A.")
        print("-" * 74, flush=True)
        # BACK TO HOME FIRST, with the clutch out, so every segment starts
        # from the same arm pose and the segments are comparable.
        n.home(label=" (segment %d)" % i)
        print("        ready -- clutch in and go.", flush=True)
        n.spin(0.6)
        s0 = n.snapshot()
        peak = {h: {"v": 0.0, "lag": 0.0} for h in HANDS}
        t0 = time.time()
        while True:
            n.spin(0.02)
            for h in HANDS:
                mp = n.mapper.get(h) or {}
                v = mp.get("hand_speed_mps")
                lg = mp.get("lag_m")
                if isinstance(v, (int, float)):
                    peak[h]["v"] = max(peak[h]["v"], float(v))
                if isinstance(lg, (int, float)):
                    peak[h]["lag"] = max(peak[h]["lag"], float(lg))
            if n.a_pressed():
                while n.a_pressed():
                    n.spin(0.02)
                break
            if time.time() - t0 > 600:
                print("        (timed out waiting for A)")
                break
        s1 = n.snapshot()
        results.append(score(key, want, s0, s1, peak, n))

    print("\n" + "=" * 74)
    print("SUMMARY")
    print("=" * 74)
    print("%-12s %-8s %-8s %-8s %-9s %-9s"
          % ("segment", "hand_mm", "arm_mm", "gain", "dir_err", "max_lag_mm"))
    for r in results:
        for h in HANDS:
            if h not in r:
                continue
            d = r[h]
            print("%-12s %-8s %-8s %-8s %-9s %-9s"
                  % ("%s/%s" % (r["segment"], h[0]),
                     d.get("hand_mm", "--"), d.get("arm_mm", "--"),
                     d.get("gain", "--"), d.get("dir_err_deg", "--"),
                     d.get("max_lag_mm", "--")))
    if a.out:
        with open(a.out, "w") as fh:
            json.dump(results, fh, indent=1, default=float)
        print("\nwritten to %s" % a.out)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
