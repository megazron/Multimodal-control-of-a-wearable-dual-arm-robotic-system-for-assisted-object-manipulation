#!/usr/bin/env python3
"""rtsp_probe.py -- does this arm's camera actually SERVE?

    python3 scripts/rtsp_probe.py 192.168.1.10        # exit 0 = serving

WHY NOT GStreamer. The first version of this check ran

    gst-launch-1.0 rtspsrc location=rtsp://IP/color protocols=tcp ! fakesink

and it FAILED ON A HEALTHY CAMERA, every time. `rtspsrc` exposes its pads
DYNAMICALLY once the stream is described, so that pipeline cannot link
statically and never reaches PLAYING no matter what the server does. A probe
that always says "broken" is worse than no probe: on 2026-08-30 it refused to
start a right wrist camera that was answering perfectly, which is a gate
inventing the fault it was written to detect.

WHY NOT ping. `ping` proves the arm's network stack is alive and says NOTHING
about the vision module. Measured the same day: both arms answered ping AND
served HTTP 200 in 86 ms while one of their RTSP servers was wedged.

SO ASK RTSP ITSELF, in the protocol's own words. OPTIONS then DESCRIBE, over
TCP, on port 554. A camera that answers DESCRIBE with an SDP containing a
video track is serving; one that accepts the connection and never replies is
the wedged vision module, which does not recover on its own and needs the arm
power-cycled. Those two are indistinguishable from `kinova_vision`'s output,
which retries for ever and publishes a topic with no frames on it either way.
"""
import socket
import sys


def probe(ip, port=554, timeout=6.0):
    """(True, detail) if the camera serves an SDP with a video track."""
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
    except Exception as e:                                    # noqa: BLE001
        return False, "no TCP connection to %s:%d (%s)" % (ip, port, e)
    s.settimeout(timeout)
    try:
        s.sendall(("OPTIONS rtsp://%s/color RTSP/1.0\r\nCSeq: 1\r\n"
                   "User-Agent: srl-probe\r\n\r\n" % ip).encode())
        try:
            head = s.recv(2048).decode("utf-8", "replace")
        except socket.timeout:
            return False, ("connected to %s:%d and got NO REPLY to OPTIONS -- "
                           "the vision module is wedged" % (ip, port))
        if "200 OK" not in head.splitlines()[0]:
            return False, "OPTIONS refused: %s" % head.splitlines()[0]
        s.sendall(("DESCRIBE rtsp://%s/color RTSP/1.0\r\nCSeq: 2\r\n"
                   "Accept: application/sdp\r\nUser-Agent: srl-probe\r\n\r\n"
                   % ip).encode())
        try:
            sdp = s.recv(8192).decode("utf-8", "replace")
        except socket.timeout:
            return False, ("answered OPTIONS but NOT DESCRIBE -- the server is "
                           "up and the camera behind it is not")
        if "m=video" not in sdp:
            return False, "DESCRIBE returned no video track"
        rate = next((l.split(":", 1)[1] for l in sdp.splitlines()
                     if l.startswith("a=framerate:")), "?")
        return True, "serving, %s fps" % rate.strip()
    finally:
        try:
            s.close()
        except Exception:                                     # noqa: BLE001
            pass


def main():
    if len(sys.argv) < 2:
        print("usage: rtsp_probe.py <arm-ip>", file=sys.stderr)
        return 2
    ok, detail = probe(sys.argv[1])
    print(detail)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
