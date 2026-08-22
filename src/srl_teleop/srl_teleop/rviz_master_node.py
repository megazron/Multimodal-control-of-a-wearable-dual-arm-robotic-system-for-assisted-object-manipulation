#!/usr/bin/env python3
"""The RViz master: three states, the floor as geometry, the whole path.

    ros2 run srl_teleop rviz_master
    ros2 run srl_teleop rviz_master --ros-args -p floor_m:=0.15

A thin ROS shell over `srl_teleop.rviz_master`, which holds every decision
about WHAT is drawn and is tested with no graph, no move_group and no
display. This file holds only the plumbing: what to subscribe to, how often
to redraw, and how to turn a marker dict into a `visualization_msgs/Marker`.

TOPICS OUT. One namespace per concern, so an operator can switch a layer off
without losing the rest, and so a layer that stops publishing is visibly
absent rather than merged into another:

    /viz/states       COMMANDED (sim) and ACTUAL (real) chains, and the last
                      PLANNED-BUT-REFUSED pose with its reason string
    /viz/wearer       the body inflated by the clearance floor, drawn from
                      the body the guard is ENFORCING -- not the mannequin
    /viz/path         the densified sweep, one point per sample, coloured by
                      that sample's own clearance
    /viz/liveness     joint-state age per arm, the bridge's error count, the
                      Kortex session, and whether the wrist has been unpinned
    /viz/envelope     the measured workspace as three volumes per arm

TOPICS IN, and every one of them can be absent. An absent input renders as
UNKNOWN and never as OK -- `divergence.py` already established that fresh,
frozen and absent are three states rather than two, and for the same reason:
subtracting a missing thing gives 0.000, which looks like flawless tracking.

WHAT THIS NODE MAY NOT DO. It publishes markers and nothing else. It has no
publisher on any command topic, no service client that moves anything, and
no parameter that can loosen the floor -- `floor_m` is what it DRAWS, and
`participant_safety_node` is what the arm is stopped by. A drawing that could
change the guard would be a drawing that could be wrong in the one direction
that matters.
"""
from __future__ import annotations

import ast
import json

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from srl_teleop import rviz_master as RM

ARMS = ("left", "right")

_TYPE = {
    "line_strip": Marker.LINE_STRIP,
    "sphere": Marker.SPHERE,
    "cube": Marker.CUBE,
    "cylinder": Marker.CYLINDER,
    "text": Marker.TEXT_VIEW_FACING,
}


def to_marker(d, stamp):
    """One marker dict -> one `visualization_msgs/Marker`."""
    m = Marker()
    m.header.frame_id = d["frame_id"]
    m.header.stamp = stamp
    m.ns = d["ns"]
    m.id = int(d["id"])
    if d["action"] == "deleteall":
        m.action = Marker.DELETEALL
        return m
    m.action = Marker.ADD
    m.type = _TYPE[d["type"]]
    (px, py, pz), (qx, qy, qz, qw) = d["pose"]
    m.pose.position = Point(x=float(px), y=float(py), z=float(pz))
    m.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz),
                                    w=float(qw))
    sx, sy, sz = d["scale"]
    m.scale = Vector3(x=float(sx), y=float(sy), z=float(sz))
    r, g, b, a = d["colour"]
    m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
    m.text = d["text"]
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in d["points"]]
    return m


class RvizMaster(Node):

    def __init__(self):
        super().__init__("rviz_master")
        self.declare_parameter("floor_m", RM.DEFAULT_FLOOR_M)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("stale_s", 1.0)
        self.floor = float(self.get_parameter("floor_m").value)
        self.stale_s = float(self.get_parameter("stale_s").value)

        self.pub = {k: self.create_publisher(MarkerArray, "/viz/" + k, 1)
                    for k in ("states", "wearer", "path", "liveness",
                              "envelope")}

        # ---- inputs, all optional
        self.js = {"sim": (None, 0.0), "real": (None, 0.0)}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._js("sim", m), 10)
        self.create_subscription(JointState, "/real/joint_states",
                                 lambda m: self._js("real", m), 10)
        self.guard = None
        self.create_subscription(String, "/mount_guard", self._guard, 10)
        self.body = None
        self.create_subscription(String, "/wearer/enforced", self._body, 10)
        self.ik = {a: None for a in ARMS}
        for a in ARMS:
            self.create_subscription(
                Float64MultiArray, "/ik_status_%s" % a,
                (lambda m, arm=a: self.ik.__setitem__(arm, list(m.data))), 10)
        # The refused pose. Nothing publishes this yet -- `safe_motion` and
        # `real_homing_node` refuse in-process -- so it renders as absent and
        # SAYS it is absent, rather than as "no refusals have happened".
        self.refused = None
        self.create_subscription(String, "/viz/refused_in", self._refused, 10)
        # THE DENSIFIED PATH. `safe_motion.check_path(..., trace=[])` fills a
        # list of {"ee": [x,y,z], "clearance_m": c, "part": name} covering
        # EVERY sample -- including the ones after a breach, which is the
        # half that matters. Publish that list here as JSON and the middle of
        # a motion becomes visible for the first time.
        self.path = None
        self.create_subscription(String, "/viz/path_in", self._path, 10)

        self._tf = None
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf = Buffer()
            TransformListener(self._tf, self)
        except Exception as e:                              # noqa: BLE001
            self.get_logger().warn("no TF listener (%s); the arm chains will "
                                   "render as UNKNOWN rather than as absent"
                                   % e)

        self.create_timer(1.0 / max(0.5, float(
            self.get_parameter("rate_hz").value)), self.tick)
        self.get_logger().info(
            "rviz_master up. Drawing the %0.0f mm floor; the arm is stopped "
            "by participant_safety_node, not by this node."
            % (self.floor * 1000))

    # ------------------------------------------------------------- inputs
    def _js(self, which, m):
        self.js[which] = (m, self._now())

    def _guard(self, m):
        # mount_guard publishes str(dict), not JSON. Parsed with
        # literal_eval rather than eval, and a parse failure is recorded as
        # a parse failure instead of silently leaving the last good value on
        # screen looking current.
        try:
            self.guard = (ast.literal_eval(m.data), self._now())
        except (ValueError, SyntaxError) as e:
            self.get_logger().warn("cannot parse /mount_guard: %s" % e,
                                   throttle_duration_sec=10.0)
            self.guard = None

    def _body(self, m):
        try:
            self.body = (json.loads(m.data), self._now())
        except json.JSONDecodeError:
            self.body = None

    def _refused(self, m):
        try:
            self.refused = (json.loads(m.data), self._now())
        except json.JSONDecodeError:
            self.get_logger().warn(
                "/viz/refused_in is not JSON. A refusal that cannot be "
                "parsed is a refusal the operator does not see, which is "
                "the state this topic exists to end.",
                throttle_duration_sec=10.0)

    def _path(self, m):
        try:
            d = json.loads(m.data)
        except json.JSONDecodeError:
            self.get_logger().warn("/viz/path_in is not JSON",
                                   throttle_duration_sec=10.0)
            return
        pts = d.get("trace") if isinstance(d, dict) else d
        if not isinstance(pts, list) or len(pts) < 3:
            # `path_markers` refuses a short list by design -- waypoints
            # pretending to be a path is the exact picture that permitted
            # defect 2 -- so say so here rather than let it raise per tick.
            self.get_logger().warn(
                "/viz/path_in carried %s sample(s). A path of fewer than "
                "three points is a waypoint list; densify before publishing."
                % (len(pts) if isinstance(pts, list) else "no"),
                throttle_duration_sec=10.0)
            self.path = None
            return
        self.path = (pts, self._now(),
                     (d.get("why") if isinstance(d, dict) else "") or "")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, stamped):
        if stamped is None:
            return None
        return self._now() - stamped[1]

    # -------------------------------------------------------------- chains
    def chain(self, arm, prefix=""):
        """Link origins in world, or None if TF cannot answer.

        None is a THIRD state and is drawn as such. Returning [] here would
        render as an arm with no links, which is indistinguishable from an
        arm at the origin.
        """
        if self._tf is None:
            return None
        pts = []
        for link in RM_CHAIN:
            frame = "%s%s_%s" % (prefix, arm, link)
            try:
                t = self._tf.lookup_transform(
                    "world", frame, rclpy.time.Time()).transform.translation
            except Exception:                                # noqa: BLE001
                return None
            pts.append((t.x, t.y, t.z))
        return pts

    # ---------------------------------------------------------------- draw
    def tick(self):
        stamp = self.get_clock().now().to_msg()

        # ---- 1 the three states
        ms = [RM.delete_all("srl_" + s) for s in
              (RM.COMMANDED, RM.ACTUAL, RM.REFUSED)]
        sim = self.chain("left") or self.chain("right")
        real = self.chain("left", "real_") or self.chain("right", "real_")
        if sim:
            ms += RM.state_markers(RM.COMMANDED, sim, "COMMANDED (sim)")
        if real:
            ms += RM.state_markers(RM.ACTUAL, real, "ACTUAL (real arm)")
        if self.refused is not None:
            r = self.refused[0]
            pts = r.get("chain") or []
            why = r.get("why") or ""
            if len(pts) >= 2 and why:
                ms += RM.state_markers(RM.REFUSED, pts,
                                       "REFUSED: " + str(why))
        self._emit("states", ms, stamp)

        # ---- 2 the wearer, inflated by the floor
        prims = self._wearer_prims()
        note = (self.body[0].get("enforcing", "") if self.body else
                "SOURCE UNKNOWN -- /wearer/enforced is silent")
        ms = [RM.delete_all("srl_wearer")]
        ms += RM.wearer_shell(prims, self.floor, enforced_note=note)
        g = self.guard[0] if self.guard else None
        if g and g.get("worst") and sim:
            # "left forearm_link <-> L-upperarm 0.0962 m"
            try:
                parts = str(g["worst"]).split()
                margin = float(parts[-2])
                part = parts[-3]
                arm = parts[0]
                a, b = sim[-3], sim[-2]
                ms += RM.nearest_segment(a, b, margin, part, arm=arm)
            except (ValueError, IndexError):
                pass
        self._emit("wearer", ms, stamp)

        # ---- 3 the whole path, one point per SAMPLE
        ms = [RM.delete_all("srl_path"), RM.delete_all("srl_path_worst")]
        if self.path is not None:
            pts, _t, why = self.path
            samples = [p["ee"] for p in pts]
            clear = [p.get("clearance_m") for p in pts]
            try:
                ms += RM.path_markers(samples, clear, floor_m=self.floor,
                                      why=why)
            except RM.VizError as e:
                self.get_logger().warn("cannot draw the path: %s" % e,
                                       throttle_duration_sec=10.0)
        self._emit("path", ms, stamp)

        # ---- 4 liveness
        rows = []
        for which, label in (("sim", "sim joint states"),
                             ("real", "real joint states")):
            msg, t = self.js[which]
            if msg is None:
                rows.append((label, "never arrived", None))
            else:
                age = self._now() - t
                rows.append((label, "%.2f s ago" % age, age <= self.stale_s))
        for a in ARMS:
            d = self.ik[a]
            if d is None:
                rows.append(("%s IK" % a, "no /ik_status_%s" % a, None))
                continue
            fail = d[1] if len(d) > 1 else 0.0
            rows.append(("%s IK failures" % a, "%d" % int(fail), fail == 0))
            # THE WRIST RELAXATION, on the face of it. Under the default
            # `exact` policy this reads 0 forever and says so; the moment
            # somebody sets a cone it is the operator's only warning that the
            # gripper is arriving at an angle the task did not ask for.
            if len(d) >= 16:
                n, last, worst = int(d[13]), d[14], d[15]
                rows.append(("%s wrist unpinned" % a,
                             ("never" if n == 0 else
                              "%d solves, last %.1f deg, worst %.1f deg"
                              % (n, last, worst)),
                             True if n == 0 else None))
        rows.append(("clearance floor DRAWN", "%.0f mm" % (self.floor * 1000),
                     True))
        if self.path is None:
            rows.append(("path drawn", "nothing on /viz/path_in", None))
        else:
            pts = self.path[0]
            cl = [p.get("clearance_m") for p in pts
                  if p.get("clearance_m") is not None]
            rows.append(("path drawn", "%d samples, worst %.3f m"
                         % (len(pts), min(cl) if cl else float("nan")),
                         bool(cl) and min(cl) >= self.floor))
        if g:
            rows.append(("guard pairs checked", str(g.get("checked_pairs")),
                         (g.get("checked_pairs") or 0) > 0))
            rows.append(("below floor", str(g.get("below_floor")),
                         g.get("below_floor") == 0))
        else:
            rows.append(("mount guard", "silent", None))
        self._emit("liveness", [RM.delete_all("srl_liveness")]
                   + RM.liveness_text(rows), stamp)

    def _wearer_prims(self):
        try:
            from srl_teleop import mount_guard_node as MG
            return list(MG.WEARER)
        except Exception:                                    # noqa: BLE001
            return []

    def _emit(self, key, dicts, stamp):
        arr = MarkerArray()
        arr.markers = [to_marker(d, stamp) for d in dicts]
        self.pub[key].publish(arr)


# The arm chain, from the guard's own list so the drawing and the check are
# looking at the same links. Two copies is how the wearer guard came to
# resolve 0 of 9 parts.
try:
    from srl_teleop.mount_guard_node import CHAIN as RM_CHAIN
except Exception:                                            # noqa: BLE001
    RM_CHAIN = ["base_link", "shoulder_link", "half_arm_1_link",
                "half_arm_2_link", "forearm_link", "spherical_wrist_1_link",
                "spherical_wrist_2_link", "bracelet_link",
                "end_effector_link"]


def main(args=None):
    rclpy.init(args=args)
    n = RvizMaster()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
