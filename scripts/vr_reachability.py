#!/usr/bin/env python3
"""
vr_reachability.py -- STEP 1 of the wifi VR route: can the headset reach this
machine AT ALL?

WHY THIS EXISTS AND WHY IT IS DELIBERATELY PLAIN HTTP.
Lab wifi very often has AP client isolation: two clients on the same SSID can
each reach the internet and neither can reach the other. From inside a headset
that failure is INDISTINGUISHABLE from a certificate problem -- the browser
says "can't reach this site" / "connection is not private" and the natural
reaction is to go and fight the cert. So this step uses NO TLS. If this page
loads, the network policy allows the headset to reach WSL and every later
failure is a cert or a firewall problem. If it does not load, no certificate
anywhere will fix it.

It also runs a WebSocket echo on a second port, because some networks pass
HTTP through a transparent proxy that drops the Upgrade handshake. The VR path
is a WebSocket, so HTTP alone is not proof.

USAGE

    python3 scripts/vr_reachability.py                 # serve, print every hit
    python3 scripts/vr_reachability.py --selftest      # prove the tool works
    python3 scripts/vr_reachability.py --http-port 8080 --ws-port 8081

THE SELF-TEST IS NOT DECORATION. A reachability prober that answers "yes" no
matter what is worse than no prober: it sends you to debug the wrong layer.
--selftest fetches its own page over the LAN address (ground truth: reachable)
AND probes a port nothing is listening on (ground truth: unreachable) and
requires the tool to answer differently. If both come back the same the tool
is broken and it says so and exits non-zero.
"""
import argparse
import http.server
import json
import socket
import socketserver
import sys
import threading
import time
import urllib.request

HITS = []
T0 = time.time()


def lan_addrs():
    """Every non-loopback IPv4 this host holds, best guess first."""
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('192.0.2.1', 9))          # TEST-NET-1, never routed anywhere
        out.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for fam, _, _, _, sa in socket.getaddrinfo(socket.gethostname(), None,
                                                   socket.AF_INET):
            if not sa[0].startswith('127.') and sa[0] not in out:
                out.append(sa[0])
    except socket.gaierror:
        pass
    return out


PAGE = """<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>SRL reachability</title>
<style>
 body{background:#08131b;color:#dfe;font:600 24px/1.5 system-ui,sans-serif;
      margin:0;padding:5vh 6vw}
 h1{font-size:52px;margin:0 0 .3em;color:#4fe08a}
 .k{color:#7fa}
 .bad{color:#ff6a6a}
 .ok{color:#4fe08a}
 table{font-size:22px;border-collapse:collapse}
 td{padding:.25em .9em .25em 0;vertical-align:top}
 #ws{font-size:30px}
</style>
<h1>REACHED IT.</h1>
<p>This page was served by the WSL box. The lab network is NOT isolating you.</p>
<table>
<tr><td class=k>you are</td><td id=peer>&mdash;</td></tr>
<tr><td class=k>server</td><td id=host>&mdash;</td></tr>
<tr><td class=k>page loads</td><td id=n>1</td></tr>
<tr><td class=k>HTTP poll</td><td id=poll>&mdash;</td></tr>
<tr><td class=k>WebSocket</td><td id=ws>testing&hellip;</td></tr>
<tr><td class=k>WebXR</td><td id=xr>&mdash;</td></tr>
<tr><td class=k>secure ctx</td><td id=sec>&mdash;</td></tr>
</table>
<p style="font-size:18px;color:#7a9">Leave this open. The counters must keep
moving -- a page that loads once and then stalls is a network that drops long
lived connections, which would freeze the arm.</p>
<script>
const WSPORT = __WSPORT__;
let n = 0;
async function poll(){
  try{
    const r = await fetch('/ping?n=' + (++n), {cache:'no-store'});
    const j = await r.json();
    document.getElementById('peer').textContent = j.peer;
    document.getElementById('host').textContent = j.host + '  (up ' + j.uptime_s + ' s)';
    document.getElementById('poll').innerHTML =
      '<span class=ok>' + n + ' ok</span>';
  }catch(e){
    document.getElementById('poll').innerHTML =
      '<span class=bad>FAILED after ' + n + ': ' + e + '</span>';
  }
}
setInterval(poll, 1000); poll();

// WebXR is the thing we actually need; report it plainly.
document.getElementById('sec').innerHTML = window.isSecureContext
  ? '<span class=ok>yes</span>'
  : '<span class=bad>NO -- plain HTTP. Expected at this step; WebXR needs HTTPS.</span>';
if(navigator.xr){
  navigator.xr.isSessionSupported('immersive-vr').then(function(s){
    document.getElementById('xr').innerHTML = s
      ? '<span class=ok>navigator.xr present, immersive-vr supported</span>'
      : '<span class=bad>navigator.xr present, immersive-vr NOT supported</span>';
  });
}else{
  document.getElementById('xr').innerHTML =
    '<span class=bad>navigator.xr absent (expected on plain HTTP)</span>';
}

// WebSocket, on its own port: proves the Upgrade handshake survives.
let sent = 0, got = 0;
function dial(){
  const el = document.getElementById('ws');
  const u = 'ws://' + location.hostname + ':' + WSPORT + '/';
  const s = new WebSocket(u);
  s.onopen = function(){
    el.innerHTML = '<span class=ok>OPEN</span> ' + u;
    setInterval(function(){
      if(s.readyState === 1){ s.send(JSON.stringify({t: Date.now()})); sent++; }
    }, 200);
  };
  s.onmessage = function(ev){
    got++;
    const j = JSON.parse(ev.data);
    el.innerHTML = '<span class=ok>OPEN</span> &mdash; ' + got + ' echoes, rtt ' +
                   (Date.now() - j.t) + ' ms';
  };
  s.onerror = function(){
    el.innerHTML = '<span class=bad>WebSocket FAILED</span> ' + u +
      ' &mdash; HTTP works but the Upgrade does not. A proxy or the firewall ' +
      'is eating it.';
  };
  s.onclose = function(){ setTimeout(dial, 2000); };
}
dial();
</script>
"""


class H(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):                                   # noqa: A003
        pass

    def _send(self, body, ctype):
        b = body.encode()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        peer = self.client_address[0]
        if self.path.startswith('/ping'):
            self._send(json.dumps(dict(peer=peer, host=socket.gethostname(),
                                       uptime_s=round(time.time() - T0, 1))),
                       'application/json')
            return
        if peer not in HITS:
            HITS.append(peer)
            print('\n  *** HIT from %s  (%s)  ***' % (peer, self.headers.get(
                'User-Agent', '?')[:90]), flush=True)
            print('      the headset can reach this machine. Step 1 PASSES.',
                  flush=True)
        self._send(PAGE.replace('__WSPORT__', str(self.server.ws_port)),
                   'text/html; charset=utf-8')


class Srv(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def ws_echo(port):
    """Minimal RFC6455 echo. Hand-rolled so this step needs nothing installed
    -- the point is to test the NETWORK, not our dependencies."""
    import base64
    import hashlib
    import struct
    GUID = b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

    def frame(payload):
        b = payload.encode()
        n = len(b)
        if n < 126:
            return b'\x81' + bytes([n]) + b
        if n < 65536:
            return b'\x81\x7e' + struct.pack('>H', n) + b
        return b'\x81\x7f' + struct.pack('>Q', n) + b

    def read_frame(c):
        h = c.recv(2)
        if len(h) < 2:
            return None
        op = h[0] & 0x0f
        masked = h[1] & 0x80
        n = h[1] & 0x7f
        if n == 126:
            n = struct.unpack('>H', c.recv(2))[0]
        elif n == 127:
            n = struct.unpack('>Q', c.recv(8))[0]
        key = c.recv(4) if masked else b'\0\0\0\0'
        data = b''
        while len(data) < n:
            chunk = c.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        if op == 0x8:
            return None
        return bytes(b ^ key[i % 4] for i, b in enumerate(data)).decode(
            'utf-8', 'replace')

    def serve_one(c):
        try:
            req = b''
            while b'\r\n\r\n' not in req:
                d = c.recv(4096)
                if not d:
                    return
                req += d
            key = ''
            for line in req.decode('latin1').split('\r\n'):
                if line.lower().startswith('sec-websocket-key:'):
                    key = line.split(':', 1)[1].strip()
            acc = base64.b64encode(
                hashlib.sha1(key.encode() + GUID).digest()).decode()
            c.sendall(('HTTP/1.1 101 Switching Protocols\r\n'
                       'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                       'Sec-WebSocket-Accept: %s\r\n\r\n' % acc).encode())
            print('  *** WebSocket UPGRADE from %s -- the Upgrade handshake '
                  'survives this network ***' % c.getpeername()[0], flush=True)
            while True:
                m = read_frame(c)
                if m is None:
                    return
                c.sendall(frame(m))
        except OSError:
            return
        finally:
            c.close()

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port))
    s.listen(8)
    while True:
        c, _ = s.accept()
        threading.Thread(target=serve_one, args=(c,), daemon=True).start()


# ------------------------------------------------------------------ selftest
def _probe(url, timeout=2.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status == 200
    except Exception:                                            # noqa: BLE001
        return False


def selftest(http_port, ws_port):
    addrs = lan_addrs()
    if not addrs:
        print('SELFTEST INCONCLUSIVE: no non-loopback address on this host.')
        return 2
    ip = addrs[0]
    srv = Srv(('0.0.0.0', http_port), H)
    srv.ws_port = ws_port
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=ws_echo, args=(ws_port,), daemon=True).start()
    time.sleep(0.4)

    live = _probe('http://%s:%d/ping' % (ip, http_port))
    dead_port = 1                                     # nothing may listen here
    dead = _probe('http://%s:%d/ping' % (ip, dead_port))
    srv.shutdown()

    print('  reachable server on %s:%d  -> %s' % (ip, http_port, live))
    print('  unreachable port  on %s:%d  -> %s' % (ip, dead_port, dead))
    if live and not dead:
        print('\nSELFTEST PASS: the prober says YES to a live port and NO to a '
              'dead one, so a NO from it means something.')
        return 0
    print('\nSELFTEST FAIL: the prober cannot tell reachable from unreachable. '
          'Do not trust its answer.')
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--http-port', type=int, default=8080)
    ap.add_argument('--ws-port', type=int, default=8081)
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest(a.http_port, a.ws_port))

    addrs = lan_addrs()
    srv = Srv(('0.0.0.0', a.http_port), H)
    srv.ws_port = a.ws_port
    threading.Thread(target=ws_echo, args=(a.ws_port,), daemon=True).start()

    print('=' * 72)
    print('STEP 1 -- REACHABILITY.  Plain HTTP on purpose: no TLS, so a')
    print('failure here is NETWORK POLICY and not a certificate.')
    print('=' * 72)
    for ip in addrs:
        print('\n  In the headset browser, open:\n')
        print('      http://%s:%d/\n' % (ip, a.http_port))
    print('  Waiting. Every hit is printed here, so you do not have to read')
    print('  anything inside the headset to know whether it worked.')
    print('  Ctrl-C to stop.\n')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n  %d distinct client(s) reached this machine: %s'
              % (len(HITS), ', '.join(HITS) if HITS else 'NONE'))
        if not HITS:
            print('\n  NOBODY REACHED IT. Before touching certificates, check:')
            print('    * headset and this PC on the SAME SSID (not guest/5G split)')
            print('    * AP client isolation -- ask for it off, or use a phone')
            print('      hotspot, which never isolates')
            print('    * Windows Firewall inbound (see scripts/vr_firewall.md)')


if __name__ == '__main__':
    main()
