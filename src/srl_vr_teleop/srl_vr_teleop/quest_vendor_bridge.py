#!/usr/bin/env python3
"""Quest bridge speaking the VENDOR wire protocol, over `adb reverse`.

    ros2 run srl_vr_teleop quest_vendor_bridge

TRANSPORT, AND WHY THIS REPLACED THE WebXR PATH
-----------------------------------------------
The user's Unity client already streams to `ws://127.0.0.1:8766` and reaches
the host through `adb reverse tcp:8766 tcp:8766` over USB. That removes the
whole HTTPS problem: WebXR needs a secure context, which on a LAN means a
self-signed certificate with the IP in subjectAltName, a Windows firewall
rule, and the operator accepting a warning inside the headset. `127.0.0.1` is
a secure origin by definition. It is also USB, so it does not share wifi with
the arms.

MEASURED, and it is what makes a ROS node the right place for this: under
`networkingMode=mirrored`, **Windows can connect to a socket bound on
127.0.0.1 INSIDE WSL** (verified by binding here and connecting from Windows
PowerShell). So `adb reverse` run on Windows lands directly on this node. No
relay process, no second hop.

WHAT THIS ADDS OVER THE VENDOR SERVER
-------------------------------------
The vendor server is **receive-only** -- there is no `send()` anywhere in it.
That is fine for a visualiser and fatal for teleoperation, because the
operator inside a headset cannot see the real arm and needs state pushed back
to them. This bridge is bidirectional on the same socket: the client gets a
status frame at `status_hz` carrying everything the in-headset overlay shows,
and an `echo` of the client's own timestamp so round-trip latency is measured
end to end rather than estimated.

FRAMES -- stated explicitly, because getting this wrong is silent
-----------------------------------------------------------------
Unity/Quest is LEFT-handed: **x right, y UP, z forward**.
This repo's world frame is **x = wearer's RIGHT, y = FORWARD, z = UP**
(NOT the ROS x-forward convention; see docs/ENGINEERING_LOG.md).

    p_world = (x_q, z_q, y_q)

i.e. swap y and z. Swapping two axes flips handedness, which is exactly the
left-to-right-handed conversion required; for the rotation that means the
same component swap plus a negated scalar part:

    q_world = (qx, qz, qy, -qw)

`test_quest_frames.py` pins this against known rotations. A handedness error
does not look like an error -- it looks like an arm that turns the wrong way
only for some motions.

SAFETY
------
Three independent freezes, all of which stop commands rather than merely
warning:

  * TRACKING LOSS  `positionTracked`/`rotationTracked` false, or a controller
    absent, for `tracking_timeout_s` (0.2 s).
  * LINK LOSS      no packet for `packet_timeout_s`.
  * OBSERVER E-STOP  `require_observer_estop` (default TRUE) refuses to arm
    real-arm control until `/observer_estop_present` is seen. The operator is
    blind inside the headset, so a second person holding a stop is not
    optional, and the system must refuse rather than trust that one exists.

Freezing means the commanded pose stops updating and `/vr_frozen` goes true.
It does not re-home and it does not release the gripper: dropping whatever is
held because a tracker blinked would be a worse outcome than holding still.
"""
import asyncio
import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray, String

DEFAULT_PORT = 8766
SIDES = ("left", "right")


# ----------------------------------------------------------------- frames
def quest_to_world_position(p):
    """Unity (x right, y up, z forward) -> world (x right, y forward, z up)."""
    return np.array([float(p.get("x", 0.0)),
                     float(p.get("z", 0.0)),
                     float(p.get("y", 0.0))], dtype=float)


def quest_to_world_quat(q):
    """Same axis swap, with the handedness flip carried by -w.

    Returned as (x, y, z, w) to match geometry_msgs ordering.
    """
    return (float(q.get("x", 0.0)),
            float(q.get("z", 0.0)),
            float(q.get("y", 0.0)),
            -float(q.get("w", 1.0)))


class Ctl:
    """Per-controller state, kept out of the ROS node so it is testable."""

    def __init__(self, side):
        self.side = side
        self.pos = None
        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.trigger = 0.0
        self.grip = 0.0
        self.tracked = False
        self.last_tracked_t = 0.0
        self.clutch = True            # engaged at start, like the mannequin
        self.prev_grip_down = False
        self.ref = None               # master pose at engage
        self.anchor = None            # commanded pose at engage
        self.cmd = None


class QuestVendorBridge(Node):
    def __init__(self):
        super().__init__("quest_vendor_bridge")
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("bind", "127.0.0.1")
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("tracking_timeout_s", 0.2)
        self.declare_parameter("packet_timeout_s", 0.3)
        self.declare_parameter("grip_threshold", 0.6)
        self.declare_parameter("status_hz", 20.0)
        self.declare_parameter("require_observer_estop", True)
        self.declare_parameter("observer_estop_timeout_s", 2.0)

        self.port = int(self.get_parameter("port").value)
        self.bind = str(self.get_parameter("bind").value)
        self.track_to = float(self.get_parameter("tracking_timeout_s").value)
        self.pkt_to = float(self.get_parameter("packet_timeout_s").value)
        self.grip_th = float(self.get_parameter("grip_threshold").value)
        self.need_obs = bool(self.get_parameter("require_observer_estop").value)
        self.obs_to = float(
            self.get_parameter("observer_estop_timeout_s").value)

        self.ctl = {s: Ctl(s) for s in SIDES}
        self.lock = threading.Lock()
        self.last_packet = 0.0
        self.seq = 0
        self.rx_hz = 0.0
        self._rx_n = 0
        self._rx_t = time.monotonic()
        self.rtt_ms = float("nan")
        self.frozen = True
        self.freeze_reason = "no packets yet"
        self._freeze_cat = None
        self._freeze_logged = 0.0
        self.observer_ok = 0.0
        # NOT `self.clients`: rclpy.Node already defines a `clients`
        # property (its service clients) with no setter, and the
        # collision kills the node at construction.
        self._ws_clients = set()

        # THE COMMAND PATH ENDED HERE AND REACHED NOTHING. This bridge used to
        # publish only /vr_pose_<side> and /vr_gripper_<side>, which no node
        # in the workspace subscribes to: vr_pose_mapper, the stage that turns
        # a controller pose into /master_arm_pose_<arm>, listens on
        # /vr/controller_pose_<side> and /vr/controller_joy_<side>. Measured on
        # a live graph: /vr_pose_left had 1 publisher and 0 subscribers, and
        # /vr/controller_pose_left had neither. VR commands could not reach an
        # arm through the vendor transport at all.
        #
        # The bridge now publishes the topics the mapper consumes. The legacy
        # names are kept as well, because the console and the mock read them,
        # and they cost one publish each.
        self.pose_pub = {
            s: self.create_publisher(PoseStamped, "/vr_pose_%s" % s, 10)
            for s in SIDES}
        self.mapper_pose_pub = {
            s: self.create_publisher(PoseStamped,
                                     "/vr/controller_pose_%s" % s, 10)
            for s in SIDES}
        self.mapper_joy_pub = {
            s: self.create_publisher(Joy, "/vr/controller_joy_%s" % s, 10)
            for s in SIDES}
        self.tracking_pub = self.create_publisher(Bool, "/vr/tracking_ok", 10)
        self.grip_pub = {
            s: self.create_publisher(Float64MultiArray, "/vr_gripper_%s" % s,
                                     10) for s in SIDES}
        self.frozen_pub = self.create_publisher(Bool, "/vr_frozen", 10)
        self.state_pub = self.create_publisher(String, "/vr_state", 10)
        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self.create_subscription(Bool, "/observer_estop_present",
                                 self._observer, 10)
        self.create_subscription(Bool, "/estop_state", self._estop_state, 10)
        self.estopped = False

        self.create_timer(1.0 / float(self.get_parameter("status_hz").value),
                          self._tick)
        threading.Thread(target=self._serve, daemon=True).start()
        self.get_logger().warn(
            "quest_vendor_bridge on ws://%s:%d -- run on the host:\n"
            "    adb reverse tcp:%d tcp:%d\n"
            "Frames: Quest (x right, y UP, z fwd) -> world (x right, y fwd, "
            "z up) as (x, z, y); quaternion (qx, qz, qy, -qw)."
            % (self.bind, self.port, self.port, self.port))
        if self.need_obs:
            self.get_logger().warn(
                "OBSERVER E-STOP REQUIRED. Real-arm control stays refused "
                "until /observer_estop_present is true: the operator cannot "
                "see the real arm from inside the headset.")

    # ------------------------------------------------------------- ROS in
    def _observer(self, m):
        if m.data:
            self.observer_ok = time.monotonic()

    def _estop_state(self, m):
        self.estopped = bool(m.data)

    def observer_present(self):
        return (time.monotonic() - self.observer_ok) < self.obs_to

    # ---------------------------------------------------------- websocket
    def _serve(self):
        try:
            import websockets
        except ImportError:
            self.get_logger().error(
                "python websockets is not installed, so the Quest cannot "
                "connect and nothing downstream will move. "
                "pip install --user websockets")
            return

        async def handler(ws):
            self._ws_clients.add(ws)
            self.get_logger().info("Quest client connected (%s)"
                                   % (ws.remote_address,))
            try:
                async for raw in ws:
                    if isinstance(raw, bytes):
                        raw = raw.decode("utf-8", "replace")
                    try:
                        pkt = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(pkt, dict):
                        continue
                    if "left" not in pkt or "right" not in pkt:
                        continue
                    self._on_packet(pkt)
            except Exception:                                # noqa: BLE001
                pass
            finally:
                self._ws_clients.discard(ws)
                self.get_logger().warn(
                    "Quest client disconnected -- arm FREEZES")

        async def pusher():
            """Status back to the headset. The vendor server cannot do this;
            it is what the in-headset overlay renders."""
            while rclpy.ok():
                await asyncio.sleep(
                    1.0 / max(float(self.get_parameter("status_hz").value), 1))
                if not self._ws_clients:
                    continue
                msg = json.dumps(self.overlay_state())
                for ws in list(self._ws_clients):
                    try:
                        await ws.send(msg)
                    except Exception:                        # noqa: BLE001
                        self._ws_clients.discard(ws)

        async def run():
            async with websockets.serve(handler, self.bind, self.port,
                                        ping_interval=None,
                                        max_queue=8):
                await pusher()

        asyncio.run(run())

    # ------------------------------------------------------------ packets
    def _on_packet(self, pkt):
        now = time.monotonic()
        with self.lock:
            self.last_packet = now
            self.seq = int(pkt.get("sequence", self.seq + 1))
            self._rx_n += 1
            if now - self._rx_t >= 1.0:
                self.rx_hz = self._rx_n / (now - self._rx_t)
                self._rx_n = 0
                self._rx_t = now
            ts = pkt.get("timestamp")
            if isinstance(ts, (int, float)) and ts > 0:
                # The client stamps its own clock; we echo it back and the
                # CLIENT computes round trip. What we can measure here is the
                # one-way age only if the clocks agree, which they do not, so
                # we do not pretend to.
                self._client_ts = ts
            for s in SIDES:
                c = self.ctl[s]
                d = pkt.get(s) or {}
                tracked = (bool(d.get("connected", True))
                           and bool(d.get("positionTracked", False))
                           and bool(d.get("rotationTracked", False)))
                c.tracked = tracked
                if tracked:
                    c.last_tracked_t = now
                    c.pos = quest_to_world_position(d.get("position") or {})
                    c.quat = quest_to_world_quat(d.get("orientation") or {})
                c.trigger = float(d.get("trigger", 0.0))
                grip = float(d.get("grip", 0.0))
                c.grip = grip
                down = grip > self.grip_th
                # CLUTCH ON GRIP, on the falling->rising EDGE only.
                if down and not c.prev_grip_down:
                    self._toggle_clutch(c)
                c.prev_grip_down = down

    def _toggle_clutch(self, c):
        if c.clutch:
            c.clutch = False
            c.anchor = None if c.cmd is None else np.array(c.cmd)
            self.get_logger().info(
                "[VR:%s] clutch DISENGAGED -- arm frozen where it is"
                % c.side)
        else:
            if c.pos is None:
                self.get_logger().warn(
                    "[VR:%s] engage refused -- no tracked pose to reference"
                    % c.side)
                return
            c.ref = np.array(c.pos)
            if c.anchor is None:
                c.anchor = np.zeros(3)
            c.clutch = True
            self.get_logger().info(
                "[VR:%s] clutch ENGAGED -- reference latched at the "
                "controller's CURRENT pose (%.3f, %.3f, %.3f); zero jump by "
                "construction" % (c.side, *c.ref))

    # --------------------------------------------------------------- tick
    def freeze_check(self):
        now = time.monotonic()
        if self.last_packet == 0.0:
            return True, "no_packets", "no packets yet"
        if now - self.last_packet > self.pkt_to:
            return True, "link_lost", (
                "link lost -- no packet for %.2f s (limit %.2f)"
                % (now - self.last_packet, self.pkt_to))
        for s in SIDES:
            c = self.ctl[s]
            if not c.tracked and (now - c.last_tracked_t) > self.track_to:
                return True, "tracking_lost_%s" % s, (
                    "%s controller tracking lost for %.2f s (limit %.2f)"
                    % (s, now - c.last_tracked_t, self.track_to))
        if self.estopped:
            return True, "estop", "e-stop latched"
        if self.need_obs and not self.observer_present():
            return True, "no_observer", (
                "observer e-stop NOT confirmed -- refusing to drive the arm "
                "while the operator cannot see it")
        return False, "", ""

    def _tick(self):
        with self.lock:
            frozen, cat, why = self.freeze_check()
            # Log on the CATEGORY changing, not the message. The message
            # carries the elapsed time, so comparing it made every tick look
            # like a new event and produced 839 identical ERROR lines in one
            # 11 s run -- a flood that buries the transition it is reporting.
            if cat != self._freeze_cat:
                if frozen:
                    self.get_logger().error("[VR] FROZEN: %s" % why)
                else:
                    self.get_logger().info("[VR] running -- unfrozen")
                self._freeze_cat = cat
                self._freeze_logged = time.monotonic()
            elif frozen and time.monotonic() - self._freeze_logged > 5.0:
                self.get_logger().error("[VR] still FROZEN: %s" % why)
                self._freeze_logged = time.monotonic()
            self.frozen, self.freeze_reason = frozen, why
            scale = float(self.get_parameter("scale").value)
            for s in SIDES:
                c = self.ctl[s]
                if not frozen and c.clutch and c.pos is not None \
                        and c.ref is not None:
                    c.cmd = c.anchor + scale * (c.pos - c.ref)
                if c.cmd is None:
                    continue
                m = PoseStamped()
                m.header.frame_id = "world"
                m.header.stamp = self.get_clock().now().to_msg()
                m.pose.position.x = float(c.cmd[0])
                m.pose.position.y = float(c.cmd[1])
                m.pose.position.z = float(c.cmd[2])
                (m.pose.orientation.x, m.pose.orientation.y,
                 m.pose.orientation.z, m.pose.orientation.w) = c.quat
                self.pose_pub[s].publish(m)
                # The same pose on the topic vr_pose_mapper listens on. The
                # mapper owns the anchor and the clutch, so what it needs is
                # the RAW controller pose, not this bridge's already-anchored
                # command; c.raw_pose carries it.
                mm = PoseStamped()
                mm.header = m.header
                rp = getattr(c, "raw_pose", None)
                if rp is None:
                    rp = c.pos
                mm.pose.position.x = float(rp[0])
                mm.pose.position.y = float(rp[1])
                mm.pose.position.z = float(rp[2])
                (mm.pose.orientation.x, mm.pose.orientation.y,
                 mm.pose.orientation.z, mm.pose.orientation.w) = c.quat
                self.mapper_pose_pub[s].publish(mm)
                j = Joy()
                j.header = m.header
                # Layout the mapper expects: axes [trigger, grip, stick x/y],
                # buttons [primary, secondary, stick click].
                j.axes = [float(c.trigger), float(getattr(c, "grip", 0.0)),
                          float(getattr(c, "stick_x", 0.0)),
                          float(getattr(c, "stick_y", 0.0))]
                j.buttons = [int(getattr(c, "btn_a", 0)),
                             int(getattr(c, "btn_b", 0)),
                             int(getattr(c, "stick_click", 0))]
                self.mapper_joy_pub[s].publish(j)
                g = Float64MultiArray()
                g.data = [float(c.trigger)]
                self.grip_pub[s].publish(g)
        b = Bool()
        b.data = bool(self.frozen)
        self.frozen_pub.publish(b)
        # vr_pose_mapper refuses to drive while tracking is not OK, so the
        # freeze state has to reach it as well as the console. Publishing the
        # freeze without publishing this leaves the mapper driving on a frozen
        # pose, which is worse than either alone.
        tb = Bool()
        tb.data = not bool(b.data)
        self.tracking_pub.publish(tb)
        st = String()
        st.data = json.dumps(self.overlay_state())
        self.state_pub.publish(st)

    def overlay_state(self):
        """Exactly what the in-headset overlay renders. The operator cannot
        see the real arm, so this IS their situational awareness."""
        return {
            "type": "status",
            "seq": self.seq,
            "rx_hz": round(self.rx_hz, 1),
            "echo": getattr(self, "_client_ts", 0),
            "frozen": bool(self.frozen),
            "freeze_reason": self.freeze_reason,
            "estop": bool(self.estopped),
            "observer_estop": bool(self.observer_present()),
            "scale": float(self.get_parameter("scale").value),
            "arms": {
                s: {"clutch": bool(self.ctl[s].clutch),
                    "tracked": bool(self.ctl[s].tracked),
                    "gripper": round(float(self.ctl[s].trigger), 3),
                    "cmd": (None if self.ctl[s].cmd is None
                            else [round(float(v), 4)
                                  for v in self.ctl[s].cmd])}
                for s in SIDES},
        }


def main(argv=None):
    rclpy.init(args=argv)
    n = QuestVendorBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
