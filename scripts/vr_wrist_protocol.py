#!/usr/bin/env python3
"""Prompted wrist-rotation measurement: every axis, both directions.

    bash scripts/run_vr_wrist.sh

The translation protocol answers "does the arm go where my hand goes".  This
answers the harder one -- "does the arm TURN the way my wrist turns" -- and it
is harder because orientation can be wrong in a way position cannot.

WHY ORIENTATION NEEDS ITS OWN PROTOCOL
--------------------------------------
THE MIRROR.  The operator sits across the room FACING the wearer, and facing
someone and copying them is a REFLECTION: determinant -1.  A reflection leaves
positions looking perfectly correct while inverting every rotation, so a rig
that is mirrored passes the whole translation protocol and fails only here.
`align_yaw_deg` is an angle precisely so that no value of it can produce one --
but that is an argument about the code, and this measures the metal.  A mirror
shows up as one axis inverted while the other two are clean.

THE UNSCALED CHANNEL.  Position is multiplied by `scale`; orientation is not,
and cannot be -- there is no meaningful "half a rotation" mapping.  So the
right answer for gain here is 1.000 on every axis, which makes any departure a
real defect rather than a setting.

WHAT WAS UNFILTERED UNTIL 2026-08-26.  Orientation reached IK raw while
position went through a low pass, so wrist tremor arrived at the arm
unattenuated -- and the pads hang off the wrist.  The wrist smoother is new,
so its lag is new too, and `--no-smooth` reruns any segment with it off to
show what it costs.

AXES, IN THE OPERATOR'S OWN TERMS.  Named for what the hand does, not for a
frame the operator cannot see: ROLL is twisting like a doorknob, PITCH is
tipping the fingers up and down, YAW is sweeping the fingers left and right.
Each is done BOTH WAYS, because a sign error is invisible in one direction.
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
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WS = "/home/gausms/kortex_ws"
HANDS = ("left", "right")

# (key, instruction, expected sign about the hand's own axis)
SEGMENTS = [
    ("roll_cw", "ROLL: twist the controller CLOCKWISE like a doorknob, "
                "about 90 degrees. Keep your hand in one place.", +1),
    ("roll_ccw", "ROLL the other way -- ANTICLOCKWISE, about 90 degrees.", -1),
    ("pitch_up", "PITCH: tip the controller's nose UP about 60 degrees, "
                 "hand still.", +1),
    ("pitch_down", "PITCH the nose DOWN about 60 degrees.", -1),
    ("yaw_left", "YAW: sweep the controller's nose to YOUR LEFT about "
                 "60 degrees, hand still.", +1),
    ("yaw_right", "YAW the nose to YOUR RIGHT about 60 degrees.", -1),
]


def home_radians(arm):
    """Home from THE source every node loads: config/home_positions_<arm>.txt.

    Not the URDF spawn block -- the two have drifted in the working tree and
    homing to the wrong one is the jump HARD CONSTRAINT 1 is about.
    """
    d = {}
    for ln in open("%s/config/home_positions_%s.txt" % (WS, arm)):
        ln = ln.strip()
        if ln.startswith("joint_"):
            k, v = ln.split(":")
            d[k.strip()] = math.radians(float(v))
    return [d["joint_%d" % i] for i in range(1, 8)]


# ------------------------------------------------------------------ rotations
def q_norm(q):
    q = np.asarray(q, float)
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def axis_angle(q):
    """(unit axis, angle in degrees) of a rotation, shortest arc.

    The sign is canonicalised so the angle is in [0, 180]: q and -q are the
    same rotation, and reporting 350 deg about -n instead of 10 deg about +n
    would make every comparison below nonsense.
    """
    q = q_norm(q)
    if q[3] < 0:
        q = -q
    ang = 2.0 * math.acos(float(np.clip(q[3], -1.0, 1.0)))
    s = math.sqrt(max(0.0, 1.0 - q[3] * q[3]))
    if s < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    return q[:3] / s, math.degrees(ang)


def ang_between(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return math.degrees(math.acos(
        float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


class Wrist(Node):
    def __init__(self, arms):
        super().__init__("vr_wrist_protocol")
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
                lambda m, h=h: self.mapper.__setitem__(h, json.loads(m.data)),
                10)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.jpub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 10)
            for a in arms}
        self.q = {}
        self.create_subscription(
            JointState, "/joint_states",
            lambda m: self.q.update(dict(zip(m.name, m.position))), 10)

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.01)

    def home(self, secs=5.0, tol_deg=1.0, label=""):
        """Same pose before every segment, so segments are comparable.

        It matters more here than for translation: the wrist's available
        rotation depends on where joints 5-7 currently sit, and a segment
        started near a joint limit measures the LIMIT rather than the mapping.
        """
        self.spin(0.5)
        before, worst = {}, {}
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
        for a in self.arms:
            names = ["%s_joint_%d" % (a, i) for i in range(1, 8)]
            q = home_radians(a)
            now = [self.q.get(nm) for nm in names]
            worst[a] = (max(abs(math.degrees(q[i] - now[i])) for i in range(7))
                        if all(v is not None for v in now) else float("nan"))
        ok = all((not math.isnan(w)) and w < tol_deg for w in worst.values())
        print("        HOME%s: %s   %s"
              % (label,
                 "  ".join("%s %.2f->%.2f deg"
                           % (a, before.get(a, float("nan")), worst[a])
                           for a in self.arms),
                 "at home" if ok else "NOT AT HOME -- segment may be invalid"))
        return ok

    # ------------------------------------------------------------- readings
    @staticmethod
    def _q(m):
        return None if m is None else q_norm(
            [m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w])

    @staticmethod
    def _p(m):
        return None if m is None else np.array(
            [m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def ee_q(self, arm):
        """ACTUAL end-effector rotation from TF -- where the arm really is."""
        try:
            t = self.tf_buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        r = t.transform.rotation
        return q_norm([r.x, r.y, r.z, r.w])

    def a_pressed(self):
        j = self.joy
        return bool(j is not None and len(j.buttons) > 0 and j.buttons[0])

    def snapshot(self):
        return dict(t=time.time(),
                    cq={h: self._q(self.ctrl[h]) for h in HANDS},
                    cp={h: self._p(self.ctrl[h]) for h in HANDS},
                    mq={h: self._q(self.cmd[h]) for h in HANDS},
                    eq={a: self.ee_q(a) for a in self.arms})


def score(key, sign, s0, s1, peak, node):
    out = {"segment": key, "dt_s": round(s1["t"] - s0["t"], 2)}
    print("\n  %-7s %-10s %-10s %-10s %s"
          % ("", "hand_deg", "cmd_deg", "arm_deg", "notes"))
    for h in HANDS:
        arm = h
        if arm not in node.arms:
            continue
        c0, c1 = s0["cq"][h], s1["cq"][h]
        m0, m1 = s0["mq"][h], s1["mq"][h]
        e0, e1 = s0["eq"].get(arm), s1["eq"].get(arm)
        if c0 is None or c1 is None:
            print("  %-7s no controller pose" % h)
            continue
        # relative rotations, each in its own frame
        dh_ax, dh_deg = axis_angle(q_mul(c1, q_conj(c0)))
        row = {"hand_deg": round(dh_deg, 1),
               "hand_axis": [round(float(v), 3) for v in dh_ax]}
        note = []

        # HAND MUST HAVE STAYED PUT. A "rotation" that also translated is not
        # a rotation segment, and its axis error would be meaningless.
        p0, p1 = s0["cp"][h], s1["cp"][h]
        if p0 is not None and p1 is not None:
            row["hand_moved_mm"] = round(float(np.linalg.norm(p1 - p0)) * 1000, 1)
            if row["hand_moved_mm"] > 120:
                note.append("HAND ALSO TRANSLATED %.0f mm -- redo"
                            % row["hand_moved_mm"])

        if m0 is not None and m1 is not None:
            dc_ax, dc_deg = axis_angle(q_mul(m1, q_conj(m0)))
            row["cmd_deg"] = round(dc_deg, 1)
            if dh_deg > 8:
                row["cmd_gain"] = round(dc_deg / dh_deg, 3)
        else:
            note.append("NO COMMAND (clutch never engaged?)")

        if e0 is not None and e1 is not None:
            de_ax, de_deg = axis_angle(q_mul(e1, q_conj(e0)))
            row["arm_deg"] = round(de_deg, 1)
            row["arm_axis"] = [round(float(v), 3) for v in de_ax]
            if dh_deg > 8:
                row["gain"] = round(de_deg / dh_deg, 3)
                # THE MIRROR TEST. Compare the arm's rotation axis with the
                # hand's, both expressed in world. Same axis -> the rotation
                # was copied; OPPOSITE axis at the same angle -> it was
                # INVERTED, which is the reflection signature and cannot be
                # produced by any yaw.
                row["axis_err_deg"] = round(ang_between(de_ax, dh_ax), 1)
                if row["axis_err_deg"] > 150:
                    note.append("ROTATION INVERTED (mirror signature)")
                elif row["axis_err_deg"] > 60:
                    note.append("axis off by %.0f deg" % row["axis_err_deg"])
                if de_deg < 0.25 * dh_deg:
                    note.append("arm barely rotated -- joint limit or IK?")
        mp = node.mapper.get(h) or {}
        for k in ("rot_smoothed", "rot_cutoff_hz", "scale"):
            if k in mp:
                row[k] = mp[k]
        row["peak_lag_mm"] = round(peak.get(h, 0.0) * 1000, 1)

        print("  %-7s %-10s %-10s %-10s %s"
              % (h, row.get("hand_deg", "--"), row.get("cmd_deg", "--"),
                 row.get("arm_deg", "--"), "; ".join(note)))
        if "gain" in row:
            print("          gain %.3f (want 1.000 -- orientation is NOT "
                  "scaled)   axis error %.1f deg   wrist smoothing %s"
                  % (row["gain"], row.get("axis_err_deg", float("nan")),
                     row.get("rot_smoothed", "?")))
        out[h] = row
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", nargs="+", default=["left", "right"])
    ap.add_argument("--out", default="")
    ap.add_argument("--only", nargs="*", default=None)
    a = ap.parse_args()

    rclpy.init()
    n = Wrist(a.arms)
    print("waiting for the VR graph ...")
    n.spin(3.0)
    if all(v is None for v in n.ctrl.values()):
        print("No controller poses on /vr/controller_pose_*. Is the bridge up "
              "and the headset page open?")
        return 2

    segs = [s for s in SEGMENTS if a.only is None or s[0] in a.only]
    print("\n" + "=" * 74)
    print("VR WRIST PROTOCOL -- %d segments" % len(segs))
    print("Each one: CLUTCH IN, rotate the wrist WITHOUT moving your hand,")
    print("release, then press A on the RIGHT controller.")
    print("Orientation is NOT scaled, so the arm should turn 1:1 with you.")
    print("=" * 74)
    n.home(label=" (start)")

    results = []
    for i, (key, text, sign) in enumerate(segs, 1):
        print("\n" + "-" * 74)
        print("[%d/%d]  %s" % (i, len(segs), key.upper()))
        print("        %s" % text)
        print("        ... then press A.")
        print("-" * 74, flush=True)
        n.home(label=" (segment %d)" % i)
        print("        ready -- clutch in and rotate.", flush=True)
        n.spin(0.6)
        s0 = n.snapshot()
        peak = {h: 0.0 for h in HANDS}
        t0 = time.time()
        while True:
            n.spin(0.02)
            for h in HANDS:
                lg = (n.mapper.get(h) or {}).get("lag_m")
                if isinstance(lg, (int, float)):
                    peak[h] = max(peak[h], float(lg))
            if n.a_pressed():
                while n.a_pressed():
                    n.spin(0.02)
                break
            if time.time() - t0 > 600:
                print("        (timed out waiting for A)")
                break
        results.append(score(key, sign, s0, n.snapshot(), peak, n))

    print("\n" + "=" * 74)
    print("SUMMARY   (gain should be 1.000; axis error should be ~0)")
    print("=" * 74)
    print("%-16s %-10s %-10s %-8s %-11s"
          % ("segment", "hand_deg", "arm_deg", "gain", "axis_err"))
    for r in results:
        for h in HANDS:
            if h not in r:
                continue
            d = r[h]
            print("%-16s %-10s %-10s %-8s %-11s"
                  % ("%s/%s" % (r["segment"], h[0]),
                     d.get("hand_deg", "--"), d.get("arm_deg", "--"),
                     d.get("gain", "--"), d.get("axis_err_deg", "--")))
    if a.out:
        with open(a.out, "w") as fh:
            json.dump(results, fh, indent=1, default=float)
        print("\nwritten to %s" % a.out)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
