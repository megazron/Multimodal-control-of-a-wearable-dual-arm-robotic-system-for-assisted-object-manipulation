#!/usr/bin/env python3
"""
reboot_vision_module.py -- recover a wrist camera whose RTSP server has died.

    .kortex_venv/bin/python scripts/reboot_vision_module.py left
    .kortex_venv/bin/python scripts/reboot_vision_module.py both --wait 90

THE FAILURE THIS FIXES, AND HOW TO RECOGNISE IT
==========================================================================
`kinova_vision_node` retries for ever, twice a second, saying:

    [color]: Stream is PAUSED
    [color]: Failed to start stream
    [color]: Trying to connect... (attempt #16)

and the arm looks completely healthy the whole time -- it pings, its web
interface answers on port 80, and the Kortex API answers on 10000. The one
thing that is down is the vision module's RTSP server on **port 554**. That
is the check that distinguishes this from every other camera fault, and it
is the check `bringup_arm.sh` now runs before launching a camera.

Observed 2026-08-29 on both arms at once, after camera nodes were terminated
uncleanly while streaming. A stale RTSP client can wedge the module: the
server stops listening and does NOT recover on its own -- measured, it was
still shut 45 s after every client had been stopped.

WHAT THIS DOES
==========================================================================
`DeviceConfig.RebootRequest` on the VISION device (id 9, resolved from the
device manager rather than hardcoded). No motion, no configuration loss --
it restarts the camera module only. Measured recovery: left 30 s, right 40 s.

HARD CONSTRAINT 2: THE ARM PERMITS EXACTLY ONE KORTEX SESSION. The bridge
holds it. This REFUSES while a bridge is running rather than fighting for the
session, because a failed connect that leaves the arm pinging and its API
port answering is indistinguishable from a network fault -- which is the
exact confusion the constraint exists to prevent. Stop the bridge with
DISCONNECT (SIGINT) first, run this, then CONNECT.
"""
import argparse
import socket
import subprocess
import sys
import time

IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}
RTSP_PORT = 554


def rtsp_up(ip, timeout=2.0):
    """Is the vision module's RTSP server listening? The whole diagnosis."""
    try:
        with socket.create_connection((ip, RTSP_PORT), timeout=timeout):
            return True
    except OSError:
        return False


def bridge_running(arm):
    try:
        out = subprocess.run(["pgrep", "-f", "kortex_highlevel_bridge_%s" % arm],
                             capture_output=True, text=True, timeout=5)
        return bool(out.stdout.strip())
    except Exception:                                         # noqa: BLE001
        return False


def reboot(arm, ip):
    import kortex_api.autogen.client_stubs.DeviceConfigClientRpc as DCR
    import kortex_api.autogen.client_stubs.DeviceManagerClientRpc as DMR
    import kortex_api.autogen.messages.DeviceConfig_pb2 as DC
    import kortex_api.autogen.messages.Session_pb2 as S
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager

    tr = sm = None
    try:
        tr = TCPTransport()
        rt = RouterClient(tr, lambda k: None)
        tr.connect(ip, 10000)
        si = S.CreateSessionInfo()
        si.username = "admin"; si.password = "admin"
        si.session_inactivity_timeout = 60000
        si.connection_inactivity_timeout = 2000
        sm = SessionManager(rt); sm.CreateSession(si)
        dm = DMR.DeviceManagerClient(rt)
        # RESOLVED, NOT HARDCODED. It is 9 on both of these arms, but a
        # hardcoded id that is right on the rig it was written for is exactly
        # how the scene-camera probe went wrong.
        vision = [d.device_identifier for d in dm.ReadAllDevices().device_handle
                  if DC.DeviceTypes.Name(d.device_type) == "VISION"]
        if not vision:
            print("  %s: NO VISION DEVICE on this arm -- nothing to reboot."
                  % arm)
            return False
        dc = DCR.DeviceConfigClient(rt)
        o = RouterClientSendOptions(); o.timeout_ms = 8000
        for vid in vision:
            req = DC.RebootRqst()
            try:
                req.delay = 0
            except Exception:                                 # noqa: BLE001
                pass
            dc.RebootRequest(req, deviceId=vid, options=o)
            print("  %s: REBOOT sent to VISION device %d" % (arm, vid))
        return True
    except Exception as e:                                    # noqa: BLE001
        print("  %s: FAILED %s: %s" % (arm, type(e).__name__, e))
        return False
    finally:
        try:
            if sm: sm.CloseSession()
        except Exception:                                     # noqa: BLE001
            pass
        try:
            if tr: tr.disconnect()
        except Exception:                                     # noqa: BLE001
            pass
        print("  kortex session closed cleanly (%s)" % arm)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arm", choices=("left", "right", "both"))
    ap.add_argument("--wait", type=float, default=120.0,
                    help="seconds to wait for RTSP to come back")
    ap.add_argument("--check-only", action="store_true",
                    help="report RTSP state and exit; reboot nothing")
    ap.add_argument("--force", action="store_true",
                    help="reboot even if RTSP is already up")
    a = ap.parse_args()

    arms = ["left", "right"] if a.arm == "both" else [a.arm]

    print("== WRIST CAMERA (VISION MODULE) ==")
    need = []
    for arm in arms:
        up = rtsp_up(IPS[arm])
        print("  %-6s %-14s RTSP %s" % (arm, IPS[arm], "UP" if up else "DOWN"))
        if not up or a.force:
            need.append(arm)
    if a.check_only:
        return 0 if not need else 1
    if not need:
        print("Both vision modules are serving. Nothing to do "
              "(--force to reboot anyway).")
        return 0

    blocked = [arm for arm in need if bridge_running(arm)]
    if blocked:
        print("\nREFUSED for %s: a bridge is running and holds the one Kortex "
              "session the arm allows.\nPress DISCONNECT for that arm (SIGINT, "
              "never a force-kill -- that leaks the session), run this again, "
              "then CONNECT." % ", ".join(blocked))
        return 2

    print()
    rebooted = [arm for arm in need if reboot(arm, IPS[arm])]
    if not rebooted:
        return 1

    print("\nwaiting up to %.0fs for RTSP to return..." % a.wait)
    t0 = time.time()
    pending = list(rebooted)
    while pending and time.time() - t0 < a.wait:
        time.sleep(5)
        for arm in list(pending):
            if rtsp_up(IPS[arm]):
                print("  %-6s RTSP back after %.0fs" % (arm, time.time() - t0))
                pending.remove(arm)
    if pending:
        print("  STILL DOWN after %.0fs: %s. The module may need an arm power "
              "cycle." % (a.wait, ", ".join(pending)))
        return 1
    print("\nAll vision modules serving. Press CONNECT to start the cameras.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
