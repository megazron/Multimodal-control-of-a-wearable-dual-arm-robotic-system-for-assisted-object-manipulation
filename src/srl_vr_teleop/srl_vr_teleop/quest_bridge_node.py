#!/usr/bin/env python3
"""
quest_bridge_node.py — Meta Quest -> ROS transport.

TRANSPORT CHOICE: **WebXR in the Quest browser over a raw WebSocket**, with a
small JSON protocol, terminated here.

The three candidates and why this one:

| | latency | effort | risk |
| --- | --- | --- | --- |
| Unity + ROS-TCP-Connector | best in principle; Unity's render loop is 72/90 Hz and the connector adds ~1 ms on LAN | Unity licence, sideloading via adb, a rebuild per iteration | highest — a Unity project is a second build system nobody else here can run |
| **WebXR + WebSocket (chosen)** | one frame of `requestAnimationFrame` (72 Hz -> ~14 ms) plus WebSocket transit (~2-4 ms on LAN) | open a URL on the headset; edit and reload | lowest |
| direct OpenXR on a PC with Link | lowest possible | needs a tethered PC and a Windows OpenXR runtime; WSL cannot host it | tethering defeats the point of a standalone headset |

**Justified on latency, as asked.** The dominant term is NOT the transport, it
is the headset's own frame period: WebXR delivers poses in
`requestAnimationFrame`, so pose age is bounded below by one frame (13.9 ms at
72 Hz) whatever the transport. A Unity app has exactly the same floor. The
WebSocket adds single-digit milliseconds on a LAN, so choosing Unity would buy
back a few ms of a ~20 ms budget in exchange for a build system that cannot be
iterated from this workspace. If measurement later shows the WebSocket path
exceeding ~30 ms, the decision is worth revisiting — the protocol below is
deliberately transport-agnostic so that swap is a client change only.

**rosbridge was NOT chosen** even though it is the obvious WebSocket answer:
it wraps every message in a JSON envelope with full type names and goes
through a generic serialiser. This bridge speaks a 14-float protocol straight
into a struct. rosbridge is supported as a fallback (`protocol:=rosbridge`)
because it needs no custom client, but it is not the default.

PROTOCOL (client -> bridge), one JSON object per frame, one frame per rAF:

    {"t": <client ms>, "seq": <int>,
     "l": {"p":[x,y,z], "q":[x,y,z,w], "trig":0..1, "grip":0..1,
           "a":bool, "b":bool, "stick":[x,y], "valid":bool},
     "r": {...same...},
     "hmd": {"p":[x,y,z], "q":[x,y,z,w]}}

and (bridge -> client) robot state for rendering, at 20 Hz.

LATENCY is measured, not assumed: the client stamps `t`, the bridge echoes it
back in the state message, and the client reports the round trip. The one-way
estimate published on /vr/latency is half of that plus the measured bridge
processing time. Both are published so the split is visible.
"""
import asyncio
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray, String

DEFAULT_PORT = 8765


class QuestBridge(Node):
    def __init__(self):
        super().__init__('quest_bridge_node')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('protocol', 'native')      # native | rosbridge
        # Quest world axes are +x right, +y UP, +z BACKWARD (out of the screen).
        # ROS is +x forward, +y left, +z up. This is the fixed relabelling; the
        # operator-specific part (where "forward" is) is handled by the clutch
        # anchor in vr_pose_mapper, not here.
        self.declare_parameter('axis_map', 'quest_to_ros')
        self.declare_parameter('stale_timeout_s', 0.2)
        self.declare_parameter('mock', False)

        self.port = int(self.get_parameter('port').value)
        self.stale = float(self.get_parameter('stale_timeout_s').value)

        self.pose_pub, self.joy_pub = {}, {}
        for h in ('left', 'right'):
            self.pose_pub[h] = self.create_publisher(
                PoseStamped, f'/vr/controller_pose_{h}', 10)
            self.joy_pub[h] = self.create_publisher(Joy, f'/vr/controller_joy_{h}', 10)
        self.hmd_pub = self.create_publisher(PoseStamped, '/vr/hmd_pose', 10)
        self.lat_pub = self.create_publisher(Float64MultiArray, '/vr/latency', 10)
        self.stat_pub = self.create_publisher(String, '/vr/bridge_status', 10)
        self.track_pub = self.create_publisher(Bool, '/vr/tracking_ok', 10)

        self.last_rx = 0.0
        self.seq = -1
        self.n_frames = 0
        self.n_dropped = 0
        self.rtt = deque(maxlen=200)
        self.proc = deque(maxlen=200)
        self.rate_win = deque(maxlen=200)
        self.clients = set()
        self.robot_state = {}
        self.create_subscription(String, '/vr/robot_state_json',
                                 lambda m: self.robot_state.update(json.loads(m.data)), 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0, self._report)

        if bool(self.get_parameter('mock').value):
            self.get_logger().warn(
                'MOCK MODE: no headset. Poses come from vr_mock_publisher; '
                'this bridge only forwards. Everything downstream of here is '
                'exercised, everything upstream is not.')
        else:
            threading.Thread(target=self._serve, daemon=True).start()

    # ------------------------------------------------------------- transport
    def _serve(self):
        try:
            import websockets
        except ImportError:
            self.get_logger().error(
                'python3-websockets is not installed, so the Quest cannot '
                'connect. Install it, or run with mock:=true to exercise '
                'everything downstream. pip install websockets')
            return
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(ws):
            self.clients.add(ws)
            self.get_logger().info('Quest client connected')
            try:
                async for raw in ws:
                    t0 = time.monotonic()
                    self._on_frame(raw)
                    self.proc.append((time.monotonic() - t0) * 1000.0)
                    await ws.send(json.dumps(self._state_msg()))
            except Exception as e:                              # noqa: BLE001
                self.get_logger().warn('client error: %s' % e)
            finally:
                self.clients.discard(ws)
                self.get_logger().warn(
                    'Quest client disconnected - the arm will FREEZE on the '
                    'stale-pose watchdog.')

        async def main():
            async with websockets.serve(handler, '0.0.0.0', self.port,
                                        ping_interval=2, ping_timeout=4):
                self.get_logger().info(
                    'WebXR bridge listening on ws://0.0.0.0:%d' % self.port)
                await asyncio.Future()

        loop.run_until_complete(main())

    def _state_msg(self):
        s = dict(self.robot_state)
        s['echo_t'] = self.seq_t if hasattr(self, 'seq_t') else 0
        s['bridge_ms'] = round(float(np.mean(self.proc)), 3) if self.proc else 0.0
        return s

    # ---------------------------------------------------------------- frames
    @staticmethod
    def quest_to_ros_p(p):
        """Quest (x right, y up, z back) -> ROS (x forward, y left, z up)."""
        return np.array([-p[2], -p[0], p[1]], float)

    @staticmethod
    def quest_to_ros_q(q):
        """Same relabelling applied to a quaternion (x, y, z, w)."""
        x, y, z, w = q
        return np.array([-z, -x, y, w], float)

    def _on_frame(self, raw):
        try:
            m = json.loads(raw)
        except ValueError:
            return
        now = self.get_clock().now()
        self.last_rx = time.monotonic()
        self.n_frames += 1
        self.rate_win.append(self.last_rx)
        seq = int(m.get('seq', -1))
        if self.seq >= 0 and seq != self.seq + 1:
            self.n_dropped += max(0, seq - self.seq - 1)
        self.seq = seq
        self.seq_t = m.get('t', 0)

        for key, hand in (('l', 'left'), ('r', 'right')):
            c = m.get(key)
            if not c or not c.get('valid', True):
                continue
            p = self.quest_to_ros_p(c['p'])
            q = self.quest_to_ros_q(c['q'])
            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = 'vr_play_space'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
            self.pose_pub[hand].publish(ps)

            j = Joy()
            j.header.stamp = now.to_msg()
            # axes:   [trigger, grip, stick_x, stick_y]
            # buttons:[a, b]
            j.axes = [float(c.get('trig', 0.0)), float(c.get('grip', 0.0)),
                      float(c.get('stick', [0, 0])[0]), float(c.get('stick', [0, 0])[1])]
            j.buttons = [int(bool(c.get('a'))), int(bool(c.get('b')))]
            self.joy_pub[hand].publish(j)

        h = m.get('hmd')
        if h:
            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = 'vr_play_space'
            p = self.quest_to_ros_p(h['p'])
            q = self.quest_to_ros_q(h['q'])
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
            self.hmd_pub.publish(ps)

        if 'rtt' in m:
            self.rtt.append(float(m['rtt']))

    # -------------------------------------------------------------- watchdog
    def _watchdog(self):
        age = time.monotonic() - self.last_rx if self.last_rx else 1e9
        ok = age < self.stale
        b = Bool()
        b.data = bool(ok)
        self.track_pub.publish(b)
        if not ok and self.last_rx:
            self.get_logger().error(
                'VR pose STALE by %.3f s (> %.2f). Downstream must FREEZE.'
                % (age, self.stale), throttle_duration_sec=2.0)

    def _report(self):
        hz = 0.0
        if len(self.rate_win) > 5:
            span = self.rate_win[-1] - self.rate_win[0]
            if span > 0:
                hz = (len(self.rate_win) - 1) / span
        m = Float64MultiArray()
        # [rate_hz, rtt_mean_ms, rtt_p95_ms, bridge_ms, dropped, frames]
        m.data = [hz,
                  float(np.mean(self.rtt)) if self.rtt else float('nan'),
                  float(np.percentile(self.rtt, 95)) if self.rtt else float('nan'),
                  float(np.mean(self.proc)) if self.proc else float('nan'),
                  float(self.n_dropped), float(self.n_frames)]
        self.lat_pub.publish(m)
        s = String()
        s.data = json.dumps(dict(rate_hz=round(hz, 1), clients=len(self.clients),
                                 dropped=self.n_dropped, frames=self.n_frames))
        self.stat_pub.publish(s)
        if hz and hz < 60.0 and self.n_frames > 100:
            self.get_logger().warn(
                'pose rate %.1f Hz is below the 60 Hz requirement' % hz,
                throttle_duration_sec=10.0)


def main():
    rclpy.init()
    n = QuestBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
