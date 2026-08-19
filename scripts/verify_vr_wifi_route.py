#!/usr/bin/env python3
"""
verify_vr_wifi_route.py -- does the wifi/WebXR route actually work, end to end?

Starts the real `quest_bridge_node` with the real certificate, fetches the
real client page over HTTPS, opens a real `wss://` socket, speaks the real
protocol, and checks the poses arrive on the real ROS topics in the right
frame. No mock anywhere except the operator.

WHY THIS IS A SCRIPT AND NOT A UNIT TEST: it spawns a node and binds a port,
so it does not belong in the pytest gate. It is what you run before carrying a
headset to the lab.

    python3 scripts/verify_vr_wifi_route.py
    python3 scripts/verify_vr_wifi_route.py --break-cert   # prove it can fail
    python3 scripts/verify_vr_wifi_route.py --break-frame  # prove it can fail

THE TWO --break FLAGS ARE THE POINT. A route verifier that passes whatever it
is given tells you nothing. `--break-cert` serves a certificate that does not
carry this machine's address -- the exact failure a moved DHCP lease produces,
which inside a headset shows as ERR_CERT_COMMON_NAME_INVALID and reads like a
broken certificate rather than a stale one. `--break-frame` puts the shipped
axis mapping back. Both must FAIL, and the script exits non-zero if either
passes.
"""
import argparse
import asyncio
import json
import os
import shutil
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CERT_DIR = os.environ.get('VR_CERT_DIR', os.path.expanduser('~/.srl_vr_cert'))
WEB = os.path.join(WS, 'src/srl_vr_teleop/web')


def lan_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('192.0.2.1', 9))
        return s.getsockname()[0]
    finally:
        s.close()


def make_bad_cert(tmp):
    """A cert for an address this host does NOT hold. Everything about it is
    valid except the one field the browser checks."""
    crt, key = os.path.join(tmp, 'bad.crt'), os.path.join(tmp, 'bad.key')
    subprocess.run(
        ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-sha256', '-nodes',
         '-days', '30', '-keyout', key, '-out', crt,
         '-subj', '/CN=10.99.99.99',
         '-addext', 'subjectAltName = IP:10.99.99.99'],
        check=True, capture_output=True)
    return crt, key


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--break-cert', action='store_true')
    ap.add_argument('--break-frame', action='store_true')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy

    ip = lan_ip()
    tmp = tempfile.mkdtemp(prefix='vrverify')
    if a.break_cert:
        crt, key = make_bad_cert(tmp)
        print('BROKEN-CERT MODE: serving a cert for 10.99.99.99, not %s' % ip)
    else:
        crt = os.path.join(CERT_DIR, 'vr.crt')
        key = os.path.join(CERT_DIR, 'vr.key')
        if not os.path.exists(crt):
            subprocess.run(['bash', os.path.join(WS, 'scripts/make_vr_cert.sh')],
                           check=True, capture_output=True)

    env = dict(os.environ, PYTHONUNBUFFERED='1')
    argv = ['ros2', 'run', 'srl_vr_teleop', 'quest_bridge_node', '--ros-args']
    if a.break_frame:
        # Monkeypatched from OUTSIDE, in a throwaway launcher. The production
        # node gets no 'break me' switch: a negative control that lives in the
        # shipped code is one more thing that can be left on.
        print('BROKEN-FRAME MODE: the shipped [-z,-x,y] mapping, restored by '
              'an external launcher.')
        shim = os.path.join(tmp, 'shim.py')
        with open(shim, 'w') as f:
            f.write(
                'import sys, numpy as np\n'
                'from srl_vr_teleop import quest_bridge_node as q\n'
                'q.webxr_to_world_position = lambda p: np.array('
                '[-float(p[2]), -float(p[0]), float(p[1])])\n'
                'q.webxr_to_world_quat = lambda v: np.array('
                '[-float(v[2]), -float(v[0]), float(v[1]), float(v[3])])\n'
                'q.main()\n')
        argv = [sys.executable, shim, '--ros-args']

    got, fails = {}, []

    def check(name, cond, detail=''):
        print('  %-58s %s %s' % (name, 'PASS' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    class Sub(Node):
        def __init__(self):
            super().__init__('vr_route_verifier')
            self.create_subscription(PoseStamped, '/vr/controller_pose_left',
                                     lambda m: got.setdefault('pose', m), 10)
            self.create_subscription(Joy, '/vr/controller_joy_left',
                                     lambda m: got.setdefault('joy', m), 10)
            self.create_subscription(PoseStamped, '/vr/hmd_pose',
                                     lambda m: got.setdefault('hmd', m), 10)

    proc = subprocess.Popen(
        argv + ['-p', 'port:=%d' % a.port, '-p', 'certfile:=%s' % crt,
                '-p', 'keyfile:=%s' % key, '-p', 'web_dir:=%s' % WEB],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        start_new_session=True)
    time.sleep(6)
    if proc.poll() is not None:
        print('  the bridge DIED on startup (exit %s). Run it by hand to see '
              'why.' % proc.returncode)
        shutil.rmtree(tmp, ignore_errors=True)
        return 2

    print('\n1. the page, over HTTPS, at the LAN address')
    # hostname checking ON. This is the check the headset performs and the one
    # a stale cert fails.
    strict = ssl.create_default_context(cafile=crt)
    try:
        with urllib.request.urlopen('https://%s:%d/' % (ip, a.port),
                                    context=strict, timeout=6) as r:
            body = r.read().decode()
        check('served, and the cert validates for %s' % ip, r.status == 200)
        check('it is the WebXR client', 'immersive-vr' in body)
        check('one Content-Type header, not two',
              len(r.headers.get_all('Content-Type') or []) == 1)
    except Exception as e:                                   # noqa: BLE001
        check('served, and the cert validates for %s' % ip, False, repr(e)[:110])

    print('\n2. the socket, on the SAME origin')
    lax = ssl.create_default_context(cafile=crt)
    lax.check_hostname = False
    lax.verify_mode = ssl.CERT_NONE

    async def talk():
        import websockets
        async with websockets.connect('wss://%s:%d/' % (ip, a.port),
                                      ssl=lax) as ws:
            f = {'t': int(time.time() * 1000), 'seq': 1,
                 'l': {'p': [1.0, 2.0, -3.0], 'q': [0, 0, 0, 1],
                       'trig': 0.42, 'grip': 0.9, 'a': True, 'b': False,
                       'stick': [0.1, -0.7], 'valid': True},
                 'hmd': {'p': [0.0, 1.6, 0.0], 'q': [0, 0, 0, 1]}}
            rep = None
            for i in range(5):
                f['seq'] = i + 1
                await ws.send(json.dumps(f))
                rep = await asyncio.wait_for(ws.recv(), timeout=4)
            return json.loads(rep)

    rclpy.init()
    sub = Sub()
    threading.Thread(target=lambda: rclpy.spin(sub), daemon=True).start()
    try:
        rep = asyncio.run(talk())
        check('wss connects on the page port', isinstance(rep, dict))
        check('the reply echoes the client stamp (RTT is measurable)',
              'echo_t' in rep)
    except Exception as e:                                   # noqa: BLE001
        check('wss connects on the page port', False, repr(e)[:110])

    print('\n3. the frame, all the way to a ROS topic')
    time.sleep(1.5)
    p = got.get('pose')
    if p is None:
        check('the controller pose reached ROS', False)
    else:
        v = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        # WebXR (x=1 right, y=2 up, z=-3 i.e. 3 m FORWARD)
        #   -> world (x=1 right, y=3 forward, z=2 up)
        check('WebXR (1, 2, -3) becomes world (1, 3, 2)',
              abs(v[0] - 1) < 1e-6 and abs(v[1] - 3) < 1e-6 and abs(v[2] - 2) < 1e-6,
              str(v))
    j = got.get('joy')
    if j is None:
        check('the buttons reached ROS', False)
    else:
        check('axes are [trigger, grip, stick_x, stick_y]',
              abs(j.axes[0] - 0.42) < 1e-5 and abs(j.axes[1] - 0.9) < 1e-5
              and abs(j.axes[3] + 0.7) < 1e-5, str([round(x, 3) for x in j.axes]))

    # Kill the process GROUP. `ros2 run` is a wrapper; terminating it leaves
    # the node alive holding the port, and the NEXT run of this script then
    # fails on EADDRINUSE for a reason that has nothing to do with the route.
    import signal
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)
    rclpy.shutdown()
    shutil.rmtree(tmp, ignore_errors=True)

    print()
    broke = a.break_cert or a.break_frame
    if broke:
        if fails:
            print('EXPECTED FAILURE, and it happened: %s' % ', '.join(fails))
            print('The verifier can fail, so a PASS from it means something.')
            return 0
        print('THE VERIFIER PASSED A DELIBERATELY BROKEN ROUTE. It is not a '
              'check. Do not trust it.')
        return 1
    if fails:
        print('ROUTE NOT READY: %s' % ', '.join(fails))
        return 1
    print('WIFI/WebXR ROUTE OK: https page, wss socket on the same origin, '
          'poses on ROS in the world frame. No adb involved.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
