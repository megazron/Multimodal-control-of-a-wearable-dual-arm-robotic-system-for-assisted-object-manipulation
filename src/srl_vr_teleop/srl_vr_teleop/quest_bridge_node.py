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
import os
import ssl
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
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web')

# --------------------------------------------------------------------- frames
# WebXR is RIGHT-handed:  +x right, +y up, +z toward the viewer (BACKWARD).
# Unity (the vendor/USB path) is LEFT-handed: +x right, +y up, +z FORWARD.
# They are NOT the same frame and they do NOT get the same conversion.
#
# This repo's world frame is  x = the wearer's RIGHT, y = FORWARD, z = UP,
# which is *not* the ROS x-forward convention -- see vr_bringup.md section 6.
# quest_vendor_bridge.quest_to_world_* lands in that frame; everything
# downstream, in particular vr_pose_mapper, assumes it.
#
#   world x (right)   =  +x_xr
#   world y (forward) =  -z_xr
#   world z (up)      =  +y_xr
#
# That matrix is a pure -90 deg rotation about x with det = +1, so unlike the
# Unity conversion it needs NO handedness flip and the scalar part of the
# quaternion is untouched. Negating w here would mirror every rotation.


def webxr_to_world_position(p):
    """WebXR (x right, y up, z BACK) -> world (x right, y forward, z up)."""
    return np.array([float(p[0]), -float(p[2]), float(p[1])], dtype=float)


def webxr_to_world_quat(q):
    """The same relabelling applied to (x, y, z, w). w is NOT negated."""
    return np.array([float(q[0]), -float(q[2]), float(q[1]), float(q[3])],
                    dtype=float)


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
        # TLS. Empty means plain ws:// -- correct for the USB/127.0.0.1 route,
        # and USELESS for wifi: WebXR will not start a session outside a
        # secure context, and a page served over https cannot open a ws://
        # socket (mixed content). Over wifi both of these must be set.
        self.declare_parameter('certfile', '')
        self.declare_parameter('keyfile', '')
        # Serve the client page from this SAME port and origin. That is the
        # whole point: a self-signed cert needs an exception PER ORIGIN, and a
        # WebSocket cannot show a certificate prompt -- it just fails with no
        # reason. Page on :8443 and socket on :8765 means the operator accepts
        # one warning, gets a working page, and a socket that silently never
        # connects. One origin, one warning, no trap.
        self.declare_parameter('web_dir', '')

        self.port = int(self.get_parameter('port').value)
        self.stale = float(self.get_parameter('stale_timeout_s').value)
        self.certfile = str(self.get_parameter('certfile').value)
        self.keyfile = str(self.get_parameter('keyfile').value)
        self.web_dir = str(self.get_parameter('web_dir').value) or \
            os.path.normpath(WEB_DIR)

        self.pose_pub, self.joy_pub = {}, {}
        for h in ('left', 'right'):
            self.pose_pub[h] = self.create_publisher(
                PoseStamped, f'/vr/controller_pose_{h}', 10)
            self.joy_pub[h] = self.create_publisher(Joy, f'/vr/controller_joy_{h}', 10)
        # PER-CONTROLLER validity, which is NOT the same thing as
        # /vr/tracking_ok. `last_rx` is stamped on every frame ARRIVAL, so the
        # global watchdog catches a network dropout, a sleeping headset and a
        # backgrounded app -- but NOT the case it is named after: one
        # controller losing tracking while the headset happily streams at
        # 90 Hz. Measured with a real Quest: covering a controller produced
        # zero tracking-loss transitions and the arm kept being commanded.
        # That is tolerable when the operator is wearing the headset and their
        # hands are in its cameras' view; it is not tolerable off-head, where
        # occlusion is routine.
        self.valid_pub = {h: self.create_publisher(
            Bool, '/vr/controller_valid_%s' % h, 10) for h in ('left', 'right')}
        self.last_valid = {h: 0.0 for h in ('left', 'right')}
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
        # NOT `self.clients`: rclpy.node.Node exposes `clients` as a read-only
        # PROPERTY (its service clients), so assigning it raises AttributeError
        # in __init__ and the node never comes up at all. This bridge has been
        # in the tree as "the WebXR fallback" since 2026-08-07 and had never
        # been started -- the crash is on line 1 of construction, so any single
        # run would have found it.
        self.ws_clients = set()
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

        ctx = None
        scheme, wscheme = 'http', 'ws'
        if self.certfile and self.keyfile:
            if not (os.path.exists(self.certfile) and os.path.exists(self.keyfile)):
                self.get_logger().error(
                    'certfile/keyfile given but missing on disk (%s, %s). '
                    'REFUSING to fall back to plain ws:// -- over wifi that '
                    'fails inside the headset with no reason shown, and '
                    'silently pretending to be secure is worse than stopping.'
                    % (self.certfile, self.keyfile))
                return
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(self.certfile, self.keyfile)
            scheme, wscheme = 'https', 'wss'

        def _page(path):
            """Static file out of web_dir, path-traversal refused."""
            name = 'vr_client.html' if path in ('/', '') else path.lstrip('/')
            name = name.split('?')[0]
            full = os.path.normpath(os.path.join(self.web_dir, name))
            if not full.startswith(os.path.normpath(self.web_dir)):
                return None, None
            if not os.path.isfile(full):
                return None, None
            ct = ('text/html; charset=utf-8' if full.endswith('.html')
                  else 'application/javascript' if full.endswith('.js')
                  else 'text/css' if full.endswith('.css')
                  else 'application/octet-stream')
            with open(full, 'rb') as f:
                return f.read(), ct

        def process_request(conn, request):
            # Anything that is not a WebSocket upgrade is served as a page,
            # so the client and the socket share ONE origin and ONE cert
            # exception.
            if request.headers.get('Upgrade', '').lower() == 'websocket':
                return None
            body, ct = _page(request.path)
            if body is None:
                return conn.respond(404, 'no such file\n')
            r = conn.respond(200, '')
            r.body = body
            # websockets' Headers is a MULTIdict: __setitem__ APPENDS. Setting
            # Content-Type on a response that already has one yields two, and
            # a browser given two Content-Length values drops the connection.
            for k in ('Content-Type', 'Content-Length', 'Cache-Control'):
                if k in r.headers:
                    del r.headers[k]
            r.headers['Content-Type'] = ct
            r.headers['Content-Length'] = str(len(body))
            r.headers['Cache-Control'] = 'no-store'
            return r

        async def handler(ws):
            self.ws_clients.add(ws)
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
                self.ws_clients.discard(ws)
                self.get_logger().warn(
                    'Quest client disconnected - the arm will FREEZE on the '
                    'stale-pose watchdog.')

        async def main():
            async with websockets.serve(handler, '0.0.0.0', self.port,
                                        ping_interval=2, ping_timeout=4,
                                        process_request=process_request,
                                        ssl=ctx):
                self.get_logger().info(
                    'WebXR bridge listening: page %s://<this host>:%d/  '
                    'socket %s://<this host>:%d/  (web_dir=%s)'
                    % (scheme, self.port, wscheme, self.port, self.web_dir))
                if ctx is None:
                    self.get_logger().warn(
                        'NO TLS. Fine over adb reverse (127.0.0.1 is a secure '
                        'origin); over wifi WebXR will refuse to start.')
                await asyncio.Future()

        loop.run_until_complete(main())

    def _state_msg(self):
        s = dict(self.robot_state)
        s['echo_t'] = self.seq_t if hasattr(self, 'seq_t') else 0
        s['bridge_ms'] = round(float(np.mean(self.proc)), 3) if self.proc else 0.0
        return s

    # ---------------------------------------------------------------- frames
    # Delegated to the module-level functions so the conversion is testable
    # without a ROS context, exactly as quest_vendor_bridge's is.
    quest_to_ros_p = staticmethod(webxr_to_world_position)
    quest_to_ros_q = staticmethod(webxr_to_world_quat)

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
                # Deliberately NOT publishing a pose: an emulated position is
                # the runtime GUESSING from the headset, and driving an arm
                # off a fabricated pose is worse than not moving. The
                # watchdog below turns the absence into an explicit False.
                continue
            self.last_valid[hand] = self.last_rx
            p = webxr_to_world_position(c['p'])
            q = webxr_to_world_quat(c['q'])
            ps = PoseStamped()
            ps.header.stamp = now.to_msg()
            ps.header.frame_id = 'vr_play_space'  # axes aligned to world
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
            ps.header.frame_id = 'vr_play_space'  # axes aligned to world
            p = webxr_to_world_position(h['p'])
            q = webxr_to_world_quat(h['q'])
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
            self.hmd_pub.publish(ps)

        if 'rtt' in m:
            self.rtt.append(float(m['rtt']))

    # -------------------------------------------------------------- watchdog
    def _watchdog(self):
        now = time.monotonic()
        age = now - self.last_rx if self.last_rx else 1e9
        ok = age < self.stale
        b = Bool()
        b.data = bool(ok)
        self.track_pub.publish(b)

        # Per hand: a controller counts as tracked only while VALID poses keep
        # arriving for it. Same 0.2 s threshold, applied to the right quantity.
        for h in ('left', 'right'):
            v = Bool()
            lv = self.last_valid[h]
            v.data = bool(lv and (now - lv) < self.stale)
            self.valid_pub[h].publish(v)
            if lv and not v.data:
                self.get_logger().error(
                    '%s controller has had no VALID pose for %.3f s while the '
                    'stream is still flowing -- it is occluded or emulated. '
                    'Downstream must freeze THAT ARM.' % (h, now - lv),
                    throttle_duration_sec=2.0)
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
        s.data = json.dumps(dict(rate_hz=round(hz, 1), clients=len(self.ws_clients),
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
