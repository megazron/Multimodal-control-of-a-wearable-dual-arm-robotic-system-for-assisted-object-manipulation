#!/usr/bin/env python3
"""The task scene for the clip sweep: real objects, attached on a real grasp.

WHY THIS EXISTS. The first sweep drove each mode's command path and recorded
the arms moving, but published no objects at all -- so fifteen clips evidenced
that a mode commands the robot and nothing about anything being picked up, and
task C had no multimeter in it. The pixel verification could only ever check
"did the arm move".

WHAT MAKES AN ATTACHMENT HONEST. The object follows the gripper only while the
fingers are actually closed ON IT:

    holding(knuckle, width_mm)  ==  knuckle >= 0.90 * grip_for(width_mm)

Not "the gripper is somewhere in the holding band". That band alone attaches a
40 mm block the instant the knuckle passes 0.10 rad, while the fingers are
still visibly open -- the object jumps to the hand before it is touched, which
is a picture of a grasp rather than a grasp. The 0.90 absorbs the mock's
first-order tracking lag without accepting a gripper that has barely moved.
The same rule and the same constants as `record_rviz.holding()`, imported so
the two cannot drift.

DETACH IS A RELEASE, NOT A TIMEOUT. The object is dropped the moment the
fingers open past the object's width, and it stays where it was left rather
than snapping back to its start -- otherwise a clip of a successful place
looks identical to a clip of a failed one.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
import record_rviz as rr                                     # noqa: E402
import clip_tasks as CT                                      # noqa: E402

Y = CT.Y
SEP = CT.SEP
KN = "%s_robotiq_85_left_knuckle_joint"

# rgba, measured RENDERED colours are what the verifier keys on, so these are
# chosen to sit inside its existing detector bands rather than near them.
TAN = (0.78, 0.66, 0.42, 1.0)        # tray  -> _tan
ORANGE = (1.00, 0.45, 0.02, 1.0)     # block -> _orange
TEAL = (0.05, 0.75, 0.70, 1.0)       # container -> _teal
GREEN = (0.10, 0.90, 0.20, 1.0)      # circuit box -> _green
YELLOW = (0.95, 0.75, 0.10, 1.0)     # multimeter body -> _yellow
GREY = (0.32, 0.34, 0.36, 1.0)
DARK = (0.18, 0.19, 0.21, 1.0)


def _m(ns, i, typ, xyz, scale, col, frame="world"):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = col
    return m


class Scene(Node):
    """Static furniture plus one graspable object per arm, per task."""

    def __init__(self, task, out=None):
        super().__init__("clip_scene")
        self.task = task
        self.out = out
        # THE EVIDENCE LOG. "Did the task complete" must be a measurement, not
        # a judgement made by squinting at a frame. This records when the
        # fingers actually reached the object's width, how far the object
        # travelled WHILE HELD, and where it was let go -- so a clip can be
        # scored as grasped-and-carried-and-delivered rather than as the arm
        # having moved somewhere near an object.
        self.events = []
        self.t0 = None
        qos = QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/task_objects", qos)
        self.knuck = {}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

        # graspable: name -> dict(arm, width_mm, pos, size, colour, held,
        #                          placed)
        self.items = self._items(task)
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ layout
    def _items(self, task):
        if task == "a":
            return {"block": dict(arm="left", width_mm=40,
                                  pos=list(CT.A_PICK), size=(0.04,) * 3,
                                  col=ORANGE, held=False)}
        if task == "b":
            return {"part": dict(arm="right", width_mm=45,
                                 pos=[CT.B_START[0], CT.B_START[1],
                                      CT.B_START[2] - 0.02],
                                 size=(0.045, 0.045, 0.05), col=ORANGE,
                                 held=False)}
        return {"multimeter": dict(arm="left", width_mm=50,
                                   pos=[CT.C_PRESENT[0], CT.C_PRESENT[1],
                                        CT.C_PRESENT[2] - 0.02],
                                   size=(0.05, 0.09, 0.13), col=YELLOW,
                                   held=False)}

    def _js(self, m):
        for a in ("left", "right"):
            n = KN % a
            if n in m.name:
                self.knuck[a] = float(m.position[m.name.index(n)])

    def _grip(self, arm):
        try:
            t = self.buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = t.transform.translation
        return [v.x, v.y, v.z]

    # ------------------------------------------------------------- frame
    def tick(self):
        A = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        A.markers.append(d)
        i = 0

        def add(typ, xyz, scale, col, ns="scene"):
            nonlocal i
            A.markers.append(_m(ns, i, typ, xyz, scale, col))
            i += 1

        # ---- furniture, common to every task ----------------------------
        # 1.70 m wide, widened from 1.30 so task A's bin sits ON the bench
        # at x = 0.62. Scenery only -- no verified coordinate depends on it.
        add(Marker.CUBE, [0.0, Y + 0.02, 0.92], (1.70, 0.46, 0.03), TAN)
        for sx in (-0.75, 0.75):
            add(Marker.CUBE, [sx, Y + 0.02, 0.78], (0.05, 0.05, 0.26), DARK)
        # the bin / container that task A places into
        add(Marker.CUBE, [CT.A_BIN[0], CT.A_BIN[1], CT.A_BIN[2] - 0.03],
            (0.16, 0.16, 0.02), TEAL)
        for dx, dy in ((0.08, 0), (-0.08, 0), (0, 0.08), (0, -0.08)):
            add(Marker.CUBE,
                [CT.A_BIN[0] + dx, CT.A_BIN[1] + dy, CT.A_BIN[2] + 0.01],
                (0.02 if dx else 0.16, 0.16 if dx else 0.02, 0.07), TEAL)
        # the circuit box task C probes
        add(Marker.CUBE, [-SEP / 2.0, Y, 1.09], (0.17, 0.11, 0.05), GREEN)
        # a spare tray on the bench, so the scene reads as a workspace
        add(Marker.CUBE, [0.0, Y - 0.13, 0.95], (0.34, 0.16, 0.012), TAN)

        # ---- the graspable object --------------------------------------
        for name, it in self.items.items():
            arm = it["arm"]
            k = self.knuck.get(arm)
            on = rr.holding(k, it["width_mm"])
            g = self._grip(arm)
            if self.t0 is None:
                self.t0 = self.get_clock().now().nanoseconds * 1e-9
                self.t0_wall = time.time()
            now = self.get_clock().now().nanoseconds * 1e-9 - self.t0
            if on and not it["held"]:
                # WALL TIME ON EVERY EVENT. Without it there is no common
                # clock between this node and the ffmpeg grab, and B/S1
                # recorded a GRASPED at t=44.1 s inside a 29.1 s clip -- the
                # grasp happened after the video ended and nothing noticed,
                # because the two timestamps were never comparable.
                self.events.append(dict(t=round(now, 2), wall=time.time(),
                                        ev="GRASPED",
                                        item=name, arm=arm,
                                        knuckle=round(k or -1, 4),
                                        needed=round(0.90 * rr.grip_for(
                                            it["width_mm"]), 4),
                                        at=[round(v, 4) for v in it["pos"]]))
                it["carried"] = 0.0
            if on and g is not None:
                prev = it.get("last_held")
                if prev is not None:
                    it["carried"] = it.get("carried", 0.0) + math.dist(
                        prev, [g[0], g[1], g[2] - 0.045])
                it["last_held"] = [g[0], g[1], g[2] - 0.045]
                # ATTACHED: the object rides the gripper, offset just below
                # the finger tips so it does not float inside the hand.
                it["pos"] = [g[0], g[1], g[2] - 0.045]
                it["held"] = True
            elif it["held"] and not on:
                # RELEASED: left where it was put, never snapped back.
                it["held"] = False
                it["last_held"] = None
                self.events.append(dict(t=round(now, 2), wall=time.time(),
                                        ev="RELEASED",
                                        item=name, arm=arm,
                                        carried_m=round(it.get("carried", 0.0), 4),
                                        at=[round(v, 4) for v in it["pos"]]))
            add(Marker.CUBE, it["pos"], it["size"], it["col"], ns="item")
            # a thin outline so the object is legible against the table
            add(Marker.CUBE, it["pos"],
                tuple(s * 1.06 for s in it["size"]),
                (it["col"][0], it["col"][1], it["col"][2], 0.25), ns="item")
        self.pub.publish(A)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True, choices=["a", "b", "c"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    rclpy.init()
    n = Scene(a.task, a.out)
    import json
    import signal

    def dump(*_):
        if n.out:
            summary = []
            for name, it in n.items.items():
                summary.append(dict(item=name, arm=it["arm"],
                                    width_mm=it["width_mm"],
                                    carried_m=round(it.get("carried", 0.0), 4),
                                    final=[round(v, 4) for v in it["pos"]],
                                    still_held=bool(it["held"])))
            json.dump(dict(task=n.task, events=n.events, items=summary,
                           t0_wall=getattr(n, "t0_wall", None)),
                      open(n.out, "w"), indent=2)
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, dump)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dump()
        except Exception:                                     # noqa: BLE001
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
