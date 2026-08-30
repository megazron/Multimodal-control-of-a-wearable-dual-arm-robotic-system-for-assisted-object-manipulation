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
from sensor_msgs.msg import Image, Joy
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
        # SCENE CAMERA IN THE HEADSET. Off by nothing -- the endpoint only
        # opens the device when a client actually asks for a frame, so a rig
        # with no camera attached costs nothing and logs nothing.
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_quality', 70)
        self.declare_parameter('camera_fps', 10.0)
        self.declare_parameter('camera_enabled', True)

        self.port = int(self.get_parameter('port').value)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.camera_w = int(self.get_parameter('camera_width').value)
        self.camera_h = int(self.get_parameter('camera_height').value)
        self.camera_q = int(self.get_parameter('camera_quality').value)
        self.camera_fps = float(self.get_parameter('camera_fps').value)
        self.camera_enabled = bool(self.get_parameter('camera_enabled').value)
        # THE SCENE CAMERA TOPIC, so this node can serve the headset WITHOUT
        # owning /dev/video*. See `_camera_loop`: one V4L2 device, one
        # opener. `scene_camera_node` publishes bgr8 on /scene_camera/image_raw.
        self._scene_jpg = None
        self._scene_t = 0.0
        self.create_subscription(Image, '/scene_camera/image_raw',
                                 self._on_scene_image, 5)
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
        # THE ADDRESSES, so `/vr/bridge_status` can name the headset instead of
        # counting anonymous sockets. Keyed by the socket so a disconnect
        # removes the right one when two devices are connected.
        self.client_addrs = {}
        self.last_client_addr = None
        self.last_client_t = 0.0
        # WHERE THE HEADSET SHOULD BE POINTED. Computed once, here, because
        # this node is the only thing that knows the scheme it ended up
        # serving: whether a certificate was loaded decides https/wss, and an
        # operator handed an http:// URL for a wss:// bridge gets a failure
        # inside the headset with nothing shown on the machine.
        self.serve_urls = []
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

    # --------------------------------------------------------------- who/where
    @staticmethod
    def _peer_of(ws):
        """The client's IP, as a string, from whatever the library exposes.

        `remote_address` is a (host, port) tuple on IPv4 and a 4-tuple on IPv6,
        and on some versions it is None once the socket is closing. Every one
        of those has been seen; returning a string in all cases keeps the
        status message one shape.
        """
        try:
            a = getattr(ws, 'remote_address', None)
            if not a:
                return 'unknown'
            host = a[0]
            # A v4 address arriving over a dual-stack socket comes through
            # mapped, and "::ffff:192.168.1.30" is not what anybody wants to
            # read off a screen.
            if isinstance(host, str) and host.startswith('::ffff:'):
                host = host[7:]
            return str(host)
        except Exception:                                     # noqa: BLE001
            return 'unknown'

    def _lan_ips(self):
        """This machine's IPv4 addresses, the robot's network FIRST.

        Same ordering rule as vr_bringup.lan_ips, and for the same reason: the
        first address `hostname -I` returns on this box is 172.26.244.255, a
        NAT interface, while the lab switch is 192.168.1.25. Handing the
        operator the wrong one sends them to a page the headset cannot load,
        and inside a headset that is indistinguishable from a bad certificate.
        """
        import subprocess
        try:
            out = subprocess.run(['hostname', '-I'], capture_output=True,
                                 text=True, timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return []
        ips = [t for t in out.split()
               if t and '.' in t and ':' not in t and t[0].isdigit()
               and not t.startswith('127.')]
        return sorted(ips, key=lambda t: (not t.startswith('192.168.'), t))

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

        # THE ADDRESS TO TYPE INTO THE HEADSET, decided here where the scheme
        # is known and published so the window can show it. Every address, not
        # just the preferred one: the headset may be on either network, and
        # one that does not work fails inside the headset where the operator
        # cannot see why.
        self.serve_urls = ['%s://%s:%d/' % (scheme, ip, self.port)
                           for ip in self._lan_ips()]
        self.get_logger().info(
            'open this in the headset browser: %s'
            % (', '.join(self.serve_urls) or 'no LAN address on this machine'))

        # ------------------------------------------------ scene camera
        # ONE JPEG ENDPOINT ON THE EXISTING ORIGIN, not a second server.
        # The page and the socket already share one port so the operator
        # accepts ONE certificate exception; a camera on its own port would
        # need a second one, and an <img> cannot prompt for it -- it would
        # just show a broken image with nothing in the log.
        #
        # THE GRAB RUNS IN ITS OWN THREAD AND THE REQUEST NEVER TOUCHES THE
        # DEVICE. This is not tidiness. `process_request` runs ON THE ASYNCIO
        # EVENT LOOP, and cv2's open/read are blocking calls that take seconds
        # over usbip -- the first version of this endpoint called them inline
        # and the very first GET wedged the whole server: the page stopped
        # serving AND the WebSocket stopped answering, which on a lab day is
        # indistinguishable from the bridge having died. The handler now only
        # reads a bytes object another thread has already filled.
        #
        # MJPG IS NOT OPTIONAL. Over usbip the default uncompressed YUYV
        # negotiates and then delivers no frames at all -- the device opens,
        # every call succeeds, and read() returns False forever. Measured on
        # this rig 2026-08-21: MJPG 640x480 24.9 fps, YUYV nothing.
        cam_lock = threading.Lock()
        cam_latest = {'jpg': None, 'n': 0, 'err': None}

        def _camera_loop():
            try:
                import cv2
            except Exception as exc:                          # noqa: BLE001
                with cam_lock:
                    cam_latest['err'] = 'opencv not installed: %s' % exc
                return
            cap = None
            while rclpy.ok():
                # YIELD THE DEVICE TO THE SCENE CAMERA NODE.
                #
                # A V4L2 device cannot be opened twice for capture. This
                # thread and `scene_camera_node` both want /dev/video0, so
                # whichever started first won and the other silently got
                # nothing -- which is why the scene camera "worked
                # sometimes". Measured 2026-08-30: `fuser /dev/video0` named
                # quest_bridge_node, and the scene camera reported BUSY.
                #
                # There is a correct owner and it is not this thread. The
                # scene camera node PUBLISHES the frames, so everything --
                # detection, the window, this headset endpoint -- can have
                # them at once. This thread only ever needed the device
                # because nothing else was providing them.
                #
                # So: if a publisher appears on the scene camera topic, RELEASE
                # the device and serve from the topic. If it goes away, take
                # the device back. The headset keeps its picture either way,
                # and the two nodes stop fighting over a file handle.
                if self._scene_topic_alive():
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:                     # noqa: BLE001
                            pass
                        cap = None
                        self.get_logger().warn(
                            'scene_camera_node is publishing -- released '
                            '/dev/video%d to it and serving the headset from '
                            'the topic instead. Two openers of one V4L2 '
                            'device is one too many.' % self.camera_index)
                    with cam_lock:
                        jpg = self._scene_jpg
                        if jpg is not None:
                            cam_latest['jpg'] = jpg
                            cam_latest['n'] += 1
                            cam_latest['err'] = None
                        else:
                            cam_latest['err'] = (
                                'scene_camera_node owns the device but has '
                                'not published a frame yet')
                    time.sleep(max(0.0, 1.0 / max(self.camera_fps, 1.0)))
                    continue
                if cap is None or not cap.isOpened():
                    cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_h)
                    if not cap.isOpened():
                        with cam_lock:
                            cam_latest['err'] = (
                                '/dev/video%d will not open -- attached with '
                                'usbipd, and readable by this user?'
                                % self.camera_index)
                        time.sleep(2.0)
                        continue
                ok, frame = cap.read()
                if not ok or frame is None:
                    try:
                        cap.release()
                    except Exception:                         # noqa: BLE001
                        pass
                    cap = None
                    with cam_lock:
                        cam_latest['err'] = (
                            'opened but returned no frame -- over usbip that '
                            'means the pixel format is wrong; MJPG is required')
                    time.sleep(1.0)
                    continue
                ok, buf = cv2.imencode(
                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.camera_q])
                if ok:
                    with cam_lock:
                        cam_latest['jpg'] = buf.tobytes()
                        cam_latest['n'] += 1
                        cam_latest['err'] = None
                time.sleep(max(0.0, 1.0 / max(self.camera_fps, 1.0)))

        if self.camera_enabled:
            threading.Thread(target=_camera_loop, daemon=True).start()
            self.get_logger().info(
                'scene camera thread started on /dev/video%d (%dx%d) -- '
                'served at /camera.jpg on this same origin'
                % (self.camera_index, self.camera_w, self.camera_h))

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
            if request.path.split('?')[0] == '/camera.jpg':
                with cam_lock:
                    jpg = cam_latest['jpg']
                    err = cam_latest['err']
                if jpg is None:
                    return conn.respond(503, (err or 'no frame yet') + '\n')
                r = conn.respond(200, '')
                r.body = jpg
                for k in ('Content-Type', 'Content-Length', 'Cache-Control'):
                    if k in r.headers:
                        del r.headers[k]
                r.headers['Content-Type'] = 'image/jpeg'
                r.headers['Content-Length'] = str(len(jpg))
                r.headers['Cache-Control'] = 'no-store'
                return r
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
            # WHO IS ON THE OTHER END, published rather than only logged.
            #
            # The bridge knew the peer address all along -- websockets carries
            # it -- and threw it away, so `/vr/bridge_status` could say
            # "clients: 1" and nothing else. In desk operation the operator is
            # across the room and the headset is on a shelf; when two headsets
            # are on the bench, or a phone has been left on the page from an
            # earlier test, "1 client" is the same message whether or not it is
            # the right device. The address is the only thing that
            # distinguishes them, and it costs nothing to report.
            peer = self._peer_of(ws)
            self.ws_clients.add(ws)
            self.client_addrs[ws] = peer
            self.get_logger().info('Quest client connected from %s' % peer)
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
                self.client_addrs.pop(ws, None)
                # REMEMBER THE LAST ONE. A disconnect is exactly when the
                # operator wants to know which device dropped, and clearing the
                # address on the way out is what would take it off the screen.
                self.last_client_addr = peer
                self.last_client_t = time.time()
                self.get_logger().warn(
                    'Quest client %s disconnected - the arm will FREEZE on '
                    'the stale-pose watchdog.' % peer)

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
        # RATE MUST DECAY TO ZERO WHEN FRAMES STOP.
        #
        # This was computed as (n-1)/(last-first) over a window of ARRIVAL
        # times. When the stream stops the window stops changing, so the
        # expression keeps returning the last healthy value FOREVER -- and
        # `clients` keeps reading 1, because the browser's networking answers
        # WebSocket pings while the page is suspended. Measured 2026-08-19:
        # the headset was taken off, frames froze at 21343, and the bridge
        # went on reporting "89.8 Hz, 1 client" indefinitely. The desk
        # monitor is the operator's only status display in off-head
        # operation, and it would have shown a healthy green link with
        # nothing arriving at all.
        #
        # Only the frame COUNTER told the truth, so the rate is now computed
        # against a window ending NOW.
        now = time.monotonic()
        age = now - self.last_rx if self.last_rx else float('inf')
        live = age < 1.0
        hz = 0.0
        if live and len(self.rate_win) > 5:
            # the ws thread appends while this timer iterates; snapshot first
            recent = [t for t in list(self.rate_win) if now - t <= 2.0]
            if len(recent) > 5:
                span = recent[-1] - recent[0]
                if span > 0:
                    hz = (len(recent) - 1) / span
        m = Float64MultiArray()
        # [rate_hz, rtt_mean_ms, rtt_p95_ms, bridge_ms, dropped, frames]
        m.data = [hz,
                  float(np.mean(self.rtt)) if self.rtt else float('nan'),
                  float(np.percentile(self.rtt, 95)) if self.rtt else float('nan'),
                  float(np.mean(self.proc)) if self.proc else float('nan'),
                  float(self.n_dropped), float(self.n_frames)]
        self.lat_pub.publish(m)
        s = String()
        addrs = sorted(set(self.client_addrs.values()))
        s.data = json.dumps(dict(rate_hz=round(hz, 1),
                                 # `clients` counts OPEN SOCKETS, which is not
                                 # the same as a client that is sending. Read
                                 # `live` instead.
                                 clients=len(self.ws_clients),
                                 # WHO, not just how many. Empty while nobody
                                 # is connected; `last_client` then says which
                                 # device it was and how long ago, because a
                                 # disconnect is exactly when that matters.
                                 client_addrs=addrs,
                                 headset_ip=(addrs[0] if addrs else None),
                                 last_client=self.last_client_addr,
                                 last_client_age_s=(
                                     None if not self.last_client_t
                                     else round(time.time() - self.last_client_t, 1)),
                                 urls=self.serve_urls,
                                 port=self.port,
                                 live=bool(live),
                                 age_s=(round(age, 2) if age != float('inf')
                                        else None),
                                 dropped=self.n_dropped, frames=self.n_frames))
        self.stat_pub.publish(s)
        if not live and self.last_rx:
            self.get_logger().warn(
                'NO FRAMES for %.1f s. The socket may still be open -- a '
                'suspended page still answers pings -- so treat `clients` as '
                'meaningless here. On a Quest this is what taking the headset '
                'off looks like: the session suspends within about a second.'
                % age, throttle_duration_sec=5.0)
        if hz and hz < 60.0 and self.n_frames > 100:
            self.get_logger().warn(
                'pose rate %.1f Hz is below the 60 Hz requirement' % hz,
                throttle_duration_sec=10.0)


    #: How stale a scene-camera frame may be and still count as "that node is
    #: alive". Two seconds: long enough that a slow publisher is not mistaken
    #: for a dead one, short enough that this node takes the device back
    #: promptly when scene_camera_node stops.
    SCENE_TOPIC_STALE_S = 2.0

    def _on_scene_image(self, msg):
        """Keep the latest scene frame, already JPEG-encoded.

        Encoded HERE rather than in the serving thread because the HTTP
        handler must never do work that can block -- the same reason the
        device grab was moved off the asyncio loop in the first place.
        """
        try:
            import cv2
            import numpy as _np
            buf = _np.frombuffer(bytes(msg.data), dtype=_np.uint8)
            img = buf.reshape(msg.height, msg.width, -1)
            ok, jpg = cv2.imencode(
                '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.camera_q])
            if ok:
                self._scene_jpg = jpg.tobytes()
                self._scene_t = time.monotonic()
        except Exception as e:                                # noqa: BLE001
            self.get_logger().warn(
                'scene frame from the topic could not be encoded (%r)' % (e,),
                throttle_duration_sec=10.0)

    def _scene_topic_alive(self):
        """Is scene_camera_node THERE? Asked of the GRAPH, not of the frames.

        THE FIRST VERSION OF THIS ASKED FOR FRAMES, AND DEADLOCKED. A V4L2
        device cannot be opened twice, so:

            this node holds /dev/video0
              -> scene_camera_node cannot open it
                -> it never publishes a frame
                  -> "is it publishing?" is false
                    -> this node keeps the device, for ever

        The condition for yielding can never become true by yielding's own
        absence. Measured 2026-08-30: `fuser /dev/video0` named this process,
        scene_camera_node was not running, and the device read BUSY.

        A PUBLISHER EXISTS AS SOON AS THE NODE CONSTRUCTS, before it opens
        any device, so asking the graph breaks the cycle: the node starts,
        this sees the publisher, releases the device, and the node then opens
        it successfully and starts publishing.

        The frame timestamp is still used -- for what it is actually good
        for, which is telling the operator that scene_camera_node holds the
        device and is producing nothing. That is a fault worth naming, but it
        is NOT a reason to take the device back: doing so would restart the
        deadlock from the other side.
        """
        try:
            if self.count_publishers('/scene_camera/image_raw') > 0:
                if (self._scene_jpg is None
                        or (time.monotonic() - self._scene_t)
                        > self.SCENE_TOPIC_STALE_S):
                    self.get_logger().warn(
                        'scene_camera_node owns /dev/video%d and is not '
                        'publishing frames. The headset picture will stay '
                        'blank until it does; this node will NOT take the '
                        'device back, because that is how the two of us '
                        'deadlocked in the first place.'
                        % self.camera_index,
                        throttle_duration_sec=15.0)
                return True
        except Exception:                                     # noqa: BLE001
            pass
        return False


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
