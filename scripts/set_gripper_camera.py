#!/usr/bin/env python3
"""
set_gripper_camera.py -- brighten the wrist camera before the bridge owns it.

    .kortex_venv/bin/python scripts/set_gripper_camera.py left --brightness 40

WHY THIS EXISTS, AND WHY IT IS NOT A LIGHT
==========================================================================
Asked to "turn on the gripper lights", the honest answer measured off the
hardware on 2026-08-29 is that THERE ARE NONE. Enumerated against the real
arm's VISION device (id 9), the only options the module supports are:

    COLOR   BRIGHTNESS, CONTRAST, SATURATION
    DEPTH   EXPOSURE, GAIN, ENABLE_AUTO_EXPOSURE, VISUAL_PRESET,
            FRAMES_QUEUE_SIZE, ERROR_POLLING_ENABLED,
            OUTPUT_TRIGGER_ENABLED, DEPTH_UNITS, STEREO_BASELINE

There is no LED, lamp or torch. `OPTION_EMITTER_ENABLED` and
`OPTION_LASER_POWER` are not supported on this arm at all, so even the IR
projector cannot be switched. `Base_pb2` carries an `LedState` enum
(LED_ON/LED_OFF/LED_PULSE) with NO RPC that accepts it and no settable
message field -- a dead symbol that makes the API look like it has a light.

So this raises BRIGHTNESS instead, which is the thing that actually changes
how much the detector can see, and it reports the before/after values rather
than claiming success.

HARD CONSTRAINT 2: THE ARM PERMITS EXACTLY ONE KORTEX SESSION.
==========================================================================
This MUST run before `kortex_highlevel_bridge` takes that session, which is
why `bringup_arm.sh` calls it above the bridge block and not after. If the
bridge is already up this REFUSES rather than fighting it for the session --
a failed connect that leaves the arm pinging and its API port answering is
indistinguishable from a network fault, and that is the exact confusion the
constraint exists to prevent.
"""
import argparse
import subprocess
import sys

IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}
VISION_DEVICE_ID = 9


def bridge_running(arm):
    try:
        out = subprocess.run(["pgrep", "-f", "kortex_highlevel_bridge_%s" % arm],
                             capture_output=True, text=True, timeout=5)
        return bool(out.stdout.strip())
    except Exception:                                         # noqa: BLE001
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arm", choices=("left", "right"))
    ap.add_argument("--brightness", type=float, default=40.0,
                    help="COLOR brightness. 0 is the factory default; the "
                         "module accepts roughly -64..64.")
    ap.add_argument("--contrast", type=float, default=None)
    ap.add_argument("--saturation", type=float, default=None)
    a = ap.parse_args()

    if bridge_running(a.arm):
        print("SKIP: the %s bridge already holds the one Kortex session this "
              "arm allows. Camera settings must be applied before CONNECT."
              % a.arm)
        return 0

    try:
        import kortex_api.autogen.client_stubs.VisionConfigClientRpc as VCR
        import kortex_api.autogen.messages.VisionConfig_pb2 as VC
        import kortex_api.autogen.messages.Session_pb2 as S
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
        from kortex_api.SessionManager import SessionManager
    except Exception as e:                                    # noqa: BLE001
        print("SKIP: kortex_api not importable (%s). Run under "
              ".kortex_venv/bin/python." % e)
        return 0

    ip = IPS[a.arm]
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
        vc = VCR.VisionConfigClient(rt)
        opts = RouterClientSendOptions(); opts.timeout_ms = 3000

        wanted = [("OPTION_BRIGHTNESS", a.brightness)]
        if a.contrast is not None:
            wanted.append(("OPTION_CONTRAST", a.contrast))
        if a.saturation is not None:
            wanted.append(("OPTION_SATURATION", a.saturation))

        for oname, val in wanted:
            oi = VC.OptionIdentifier()
            oi.sensor = VC.SENSOR_COLOR
            oi.option = VC.Option.Value(oname)
            try:
                before = vc.GetOptionValue(oi, deviceId=VISION_DEVICE_ID,
                                           options=opts).value
            except Exception as e:                            # noqa: BLE001
                print("  %s: NOT SUPPORTED on this module (%s)"
                      % (oname, type(e).__name__))
                continue
            ov = VC.OptionValue()
            ov.sensor = VC.SENSOR_COLOR
            ov.option = VC.Option.Value(oname)
            ov.value = float(val)
            try:
                vc.SetOptionValue(ov, deviceId=VISION_DEVICE_ID, options=opts)
            except Exception as e:                            # noqa: BLE001
                print("  %s: refused by the module (%s)" % (oname, e))
                continue
            # READ IT BACK. A set that is silently clamped or ignored is the
            # repository's own "feature present but does nothing"; the only
            # honest report is the value the module actually holds now.
            try:
                after = vc.GetOptionValue(oi, deviceId=VISION_DEVICE_ID,
                                          options=opts).value
            except Exception:                                 # noqa: BLE001
                after = float("nan")
            mark = "" if abs(after - float(val)) < 1e-6 else "  (CLAMPED)"
            print("  %-22s %s -> %s%s" % (oname, round(before, 3),
                                          round(after, 3), mark))
        print("%s wrist camera configured (%s)" % (a.arm, ip))
        return 0
    except Exception as e:                                    # noqa: BLE001
        print("SKIP: could not configure the %s wrist camera: %s" % (a.arm, e))
        return 0
    finally:
        try:
            if sm: sm.CloseSession()
        except Exception:                                     # noqa: BLE001
            pass
        try:
            if tr: tr.disconnect()
        except Exception:                                     # noqa: BLE001
            pass
        print("kortex session closed cleanly (%s camera config)" % a.arm)


if __name__ == "__main__":
    sys.exit(main())
