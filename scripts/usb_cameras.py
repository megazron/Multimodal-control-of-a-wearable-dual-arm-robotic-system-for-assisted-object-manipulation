#!/usr/bin/env python3
"""
usb_cameras.py -- get the USB devices into WSL, and REPAIR the stale case.

  Despite the name this covers EVERY device usbipd is responsible for on this
  rig: both cameras and the TEENSY master arm. They are together because
  their failure is together -- when the usbipd service stops it takes all
  three at once, and on 2026-08-30 that presented as three unrelated faults
  (no scene camera, no RealSense, no /dev/ttyACM*). One command should bring
  back everything one service owns.

    python3 scripts/usb_cameras.py            # check and report
    python3 scripts/usb_cameras.py --fix      # attach / re-attach what is missing

WHY THIS EXISTS
==========================================================================
There is no USB in WSL. Every USB camera is handed over from Windows by
usbipd, and when it has not been the failure is silent in the worst way:
/dev/video* simply does not exist, the camera node finds nothing, and the
scene looks like a room with nobody in it.

THE STALE CASE IS THE ONE THAT WASTES THE AFTERNOON, and no existing script
handled it. After the usbipd service restarts, Windows still believes the
device is attached while the Linux side has nothing:

    usbipd: error: Device with busid '2-9' is already attached to a client.

`attach` REFUSES in that state, so the obvious fix does nothing and the
operator is left with an error that reads like everything is fine. The repair
is detach-then-attach, and that is what --fix does.

WHAT NEEDS ADMIN, MEASURED ON THIS MACHINE 2026-08-29
==========================================================================
  attach / detach   NO admin. Runs from WSL through usbipd.exe directly.
  bind              admin, ONCE per device, survives reboots.
  Start-Service     admin. The service stops on its own and takes BOTH
                    cameras AND the Teensy master arm with it -- one fault
                    that presents as three unrelated ones.

So everything this script does routinely needs no elevation; it names the two
things that do, rather than failing at them.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

USBIPD = "/mnt/c/Program Files/usbipd-win/usbipd.exe"

#: The devices this rig wants in WSL, by USB VID:PID. Keyed by VID:PID
#: because a busid changes when a cable moves to a different port, and a
#: script that hardcodes "2-9" breaks the first time somebody replugs.
#: Devices this rig wants in WSL, by USB VID:PID, with the KIND of device
#: each one is -- because "is it working" is a different question for a
#: camera and a serial board, and asking the wrong one is how a healthy
#: Teensy gets reported as absent.
#:
#: THE TEENSY IS IN HERE, and it is not a camera. It is added because
#: usbipd's failure mode is shared: when the service stops it takes the
#: cameras AND the master arm with it, and on 2026-08-30 that presented as
#: three unrelated faults. One command should bring back everything usbipd
#: is responsible for, which is what this list now is.
WANTED = {
    "32e4:0317": ("HD USB CAMERA (scene camera)", "video"),
    "8086:0b3a": ("Intel RealSense D435i", "video"),
    "16c0:0483": ("Teensy 4.1 (master arm)", "serial"),
}


def usbipd(*args, timeout=60):
    try:
        r = subprocess.run([USBIPD] + list(args), capture_output=True,
                           text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except FileNotFoundError:
        return 127, "usbipd.exe not found at %s" % USBIPD
    except Exception as e:                                    # noqa: BLE001
        return 1, str(e)


def state():
    """Parse `usbipd list` into {vidpid: (busid, state, name)}."""
    rc, out = usbipd("list")
    if rc != 0 and "not found" in out:
        return None, out
    devs = {}
    for line in out.splitlines():
        m = re.match(r"^\s*(\d+-\d+)\s+([0-9a-f]{4}:[0-9a-f]{4})\s+(.*?)\s{2,}(\S.*?)\s*$",
                     line)
        if m:
            busid, vidpid, name, st = m.groups()
            devs[vidpid.lower()] = (busid, st.strip(), name.strip())
    return devs, out


def service_down(text):
    return "service is currently not running" in text.lower()


def video_nodes():
    return sorted(glob.glob("/dev/video*"))


# ==========================================================================
#  HEALTH IS "IT DELIVERS A FRAME", NOT "THE DEVICE NODE EXISTS"
# ==========================================================================
# THE CASE THIS EXISTS FOR, measured 2026-08-29. The RealSense dropped off
# the usbip bus mid-session and came back WRONG: `usbipd list` said
# `Attached`, all six /dev/video* nodes were present, and not one of them
# delivered a frame. This script said "Nothing to do -- the cameras are in
# WSL" while the scene camera could not see the room.
#
# That is docs/ENGINEERING_LOG.md's own instrument row -- "feature present but does
# nothing" -- and its own standing rule, PREFER A PROBE OVER A FIND: asking
# whether a process or a device node exists is a proxy for asking whether
# the thing works, and the proxy and the answer came apart here.
#
# The repair is detach-then-attach, and it was measured to work: after it,
# /dev/video6 delivered 640x480 again on the first read.

def nodes_of(vidpid):
    """The /dev/video* belonging to one USB device, by VID:PID."""
    try:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "src", "srl_perception"))
        from srl_perception import video_devices as vd
    except Exception:                                         # noqa: BLE001
        return []
    return [d for d in vd.devices() if d.vid_pid.lower() == vidpid.lower()]


def delivers(path, width=640, height=480):
    """Does this node hand over one frame? The only question that counts.

    A device somebody else has open is reported as BUSY rather than dead --
    `quest_bridge_node` holds the wide-view camera whenever VR is running,
    and detaching a camera that is in use to "repair" it would break the
    thing that is working.
    """
    try:
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "src", "srl_perception"))
        from srl_perception import video_devices as vd
        if vd.users(path):
            return "busy"
        import cv2
    except Exception:                                         # noqa: BLE001
        return "unknown"
    cap = None
    try:
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            return "no"
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        return "yes" if (ok and frame is not None) else "no"
    except Exception:                                         # noqa: BLE001
        return "no"
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:                                 # noqa: BLE001
                pass


def serial_nodes(vidpid):
    """/dev/ttyACM* or ttyUSB* belonging to this VID:PID, from sysfs.

    Matched on the USB ID rather than the path: the Teensy moves between
    ACM0 and ACM1 on every attach, which is recorded in serial_port.py as
    the reason nothing here may hardcode a number.
    """
    out = []
    for path in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        node = "/sys/class/tty/%s/device" % os.path.basename(path)
        d = os.path.realpath(node)
        for _ in range(6):
            vid = os.path.join(d, "idVendor")
            pid = os.path.join(d, "idProduct")
            if os.path.exists(vid) and os.path.exists(pid):
                try:
                    got = "%s:%s" % (open(vid).read().strip(),
                                     open(pid).read().strip())
                except OSError:
                    break
                if got.lower() == vidpid.lower():
                    out.append(path)
                break
            d = os.path.dirname(d)
    return out


def serial_health(vidpid):
    """('yes'|'absent', detail) for a serial board.

    PRESENCE ONLY, deliberately. Opening the port to prove it talks would
    take it away from `master_pose_node`, which claims it exclusively --
    serial_port.claim_exclusive exists precisely because two readers on one
    Teensy is a silent corruption. A probe that breaks the thing it is
    probing is not a probe.
    """
    devs = serial_nodes(vidpid)
    if not devs:
        return "absent", "no /dev/ttyACM* or ttyUSB* belongs to %s" % vidpid
    return "yes", "present at %s (not opened -- the master node claims it)" % (
        ", ".join(devs))


def health(vidpid):
    """('yes'|'busy'|'no'|'absent', detail) for one wanted camera.

    'yes' as soon as ANY of its nodes delivers -- a multi-function camera
    has depth and infrared nodes that never will, and requiring all of them
    would condemn every working RealSense.
    """
    devs = nodes_of(vidpid)
    if not devs:
        return "absent", "no /dev/video* belongs to %s" % vidpid
    seen = []
    busy = False
    for d in devs:
        v = delivers(d.path)
        seen.append("%s=%s" % (d.path, v))
        if v == "yes":
            return "yes", "%s delivers (%s)" % (d.path, ", ".join(seen))
        if v == "busy":
            busy = True
    if busy:
        return "busy", "in use by something else (%s)" % ", ".join(seen)
    return "no", "%d node(s), none delivered: %s" % (len(devs),
                                                     ", ".join(seen))


def _run_fix():
    """Run the one-shot repair exactly as `--fix` does."""
    import sys as _sys
    argv, _sys.argv = _sys.argv, [_sys.argv[0], "--fix"]
    try:
        return main()
    finally:
        _sys.argv = argv


def watch(interval):
    """Keep the cameras attached, for as long as this runs.

    WHY A ONE-SHOT REPAIR WAS NEVER GOING TO BE ENOUGH. usbipd hands each
    camera across the WSL boundary, and it drops them on its own -- the
    service stops, a device re-enumerates, Windows sleeps a hub. `--fix`
    repairs that beautifully, ONCE, at the moment somebody notices and presses
    something. The operator's report on 2026-08-30 was "the cameras are
    constantly disconnected", which is the same fault happening on a timer
    while nobody is watching for it.

    So this is the same repair on a loop. It re-uses `health()` rather than
    inventing a second opinion about what "connected" means, and the answer
    it acts on is whether a camera DELIVERS A FRAME -- not whether a device
    node exists, because a stale attach leaves the node there and the camera
    dead, which is exactly the case that wastes an afternoon.

    IT SAYS WHEN IT REPAIRS, AND HOW OFTEN. A watchdog that silently patches
    a fault forever hides a degrading cable or a failing hub: the count is the
    measurement that tells you whether this is a nuisance or a hardware
    problem. It is deliberately quiet when nothing is wrong.

    NEVER TOUCHES A HEALTHY CAMERA. A detach-attach cycle on a camera that is
    working would drop the frame a node is mid-read on, so `busy` -- in use by
    something else -- counts as healthy here. The repair only ever runs for a
    camera that is absent or not delivering.
    """
    fixes = 0
    started = time.time()
    print("== USB CAMERA WATCHDOG == checking every %.0f s. Repairs are "
          "reported; silence means the cameras are up." % interval,
          flush=True)
    while True:
        try:
            bad = []
            for vidpid, (label, kind) in WANTED.items():
                st, detail = (serial_health(vidpid) if kind == "serial"
                              else health(vidpid))
                if st in ("absent", "no"):
                    bad.append((label, detail))
            if bad:
                fixes += 1
                mins = (time.time() - started) / 60.0
                print("[%s] CAMERAS DOWN after %.0f min: %s -- repairing "
                      "(repair #%d)"
                      % (time.strftime("%H:%M:%S"), mins,
                         "; ".join("%s: %s" % b for b in bad), fixes),
                      flush=True)
                # THE SAME REPAIR THE OPERATOR WOULD RUN, not a second
                # implementation of it. `--fix` is one function call away and
                # already handles the stale case, the unshared case and the
                # stopped service, each with its own message.
                rc = _run_fix()
                print("[%s] repair returned %s"
                      % (time.strftime("%H:%M:%S"), rc), flush=True)
            time.sleep(interval)
        except KeyboardInterrupt:
            print("\n  watchdog stopped after %d repair(s)." % fixes,
                  flush=True)
            return 0
        except Exception as e:                                # noqa: BLE001
            # A watchdog that dies on one bad poll is a watchdog that was
            # only ever going to help until the first surprise.
            print("[%s] watchdog poll failed (%r) -- continuing"
                  % (time.strftime("%H:%M:%S"), e), flush=True)
            time.sleep(interval)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true",
                    help="attach what is missing, and detach-then-attach "
                         "anything Windows thinks is already attached")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--watch", action="store_true",
                    help="stay running and repair the cameras whenever they "
                         "drop, instead of once")
    ap.add_argument("--interval", type=float, default=20.0,
                    help="seconds between checks in --watch mode")
    a = ap.parse_args()

    if a.watch:
        return watch(a.interval)

    before = video_nodes()
    say = (lambda *x: None) if a.quiet else print

    say("== USB CAMERAS ==")
    say("  /dev/video* now: %s" % (", ".join(before) if before else "NONE"))

    devs, raw = state()
    if devs is None:
        say("  cannot reach usbipd: %s" % raw.strip()[:200])
        return 1
    if service_down(raw):
        say("\n  THE USBIPD SERVICE IS STOPPED. Nothing can be attached, and"
            "\n  this takes the cameras AND the Teensy master arm with it."
            "\n  In an Administrator PowerShell:\n      Start-Service usbipd")
        return 2

    missing, stale, unshared = [], [], []
    for vidpid, (label, kind) in WANTED.items():
        info = devs.get(vidpid)
        if info is None:
            say("  %-34s NOT PRESENT on the host (unplugged?)" % label)
            continue
        busid, st, _name = info
        say("  %-34s busid %-6s %s" % (label, busid, st))
        low = st.lower()
        if "not shared" in low:
            unshared.append((busid, label))
        elif "attached" in low:
            # WINDOWS SAYS ATTACHED. That is a claim about Windows, not
            # about whether a camera works. Ask the camera.
            hs, detail = (serial_health(vidpid) if kind == "serial"
                          else health(vidpid))
            say("      %s -- %s" % (hs.upper(), detail))
            if hs in ("no", "absent"):
                stale.append((busid, label))
        else:
            missing.append((busid, label))

    if unshared:
        say("\n  THESE NEED A ONE-TIME ADMIN BIND (survives reboots):")
        for busid, label in unshared:
            say("      usbipd bind --busid %s      # %s" % (busid, label))

    todo = missing + stale
    if not todo:
        if before:
            say("\n  Nothing to do -- the cameras are in WSL and delivering.")
            return 0
        say("\n  No camera is attached and none is shared. See the bind step "
            "above.")
        return 1

    if not a.fix:
        say("\n  Re-run with --fix to repair %d device(s)." % len(todo))
        return 1

    say("")
    for busid, label in stale:
        say("  REPAIRING %s (%s): Windows says attached and the camera is "
            "not delivering." % (busid, label))
        rc, out = usbipd("detach", "--busid", busid)
        say("    detach: %s" % (out.strip().splitlines()[-1]
                                if out.strip() else "ok"))
        time.sleep(2.0)
    for busid, label in todo:
        rc, out = usbipd("attach", "--wsl", "--busid", busid)
        last = out.strip().splitlines()[-1] if out.strip() else "ok"
        say("  attach %s (%s): %s" % (busid, label, last))

    # THE ONLY ANSWER THAT COUNTS is whether a camera delivers a frame now.
    # Not whether a device node reappeared: that is what was already true
    # while nothing worked.
    for _ in range(10):
        time.sleep(1.0)
        if video_nodes():
            break
    after = video_nodes()
    say("\n  /dev/video* now: %s"
        % (", ".join(after) if after else "STILL NONE"))
    if not after:
        say("  The attach reported success and Linux still has no device. "
            "That is usually the service having been restarted under a live "
            "attach; unplug the camera and plug it back in.")
        return 1
    bad = []
    for vidpid, (label, kind) in WANTED.items():
        if vidpid not in devs:
            continue
        hs, detail = (serial_health(vidpid) if kind == "serial"
                      else health(vidpid))
        say("  %-34s %s -- %s" % (label, hs.upper(), detail))
        if hs in ("no", "absent"):
            bad.append(label)
    if bad:
        say("\n  STILL NOT DELIVERING: %s. Unplug it and plug it back in."
            % ", ".join(bad))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
