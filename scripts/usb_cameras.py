#!/usr/bin/env python3
"""
usb_cameras.py -- get the USB cameras into WSL, and REPAIR the stale case.

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
WANTED = {
    "32e4:0317": "HD USB CAMERA (scene camera)",
    "8086:0b3a": "Intel RealSense D435i",
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


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true",
                    help="attach what is missing, and detach-then-attach "
                         "anything Windows thinks is already attached")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

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
    for vidpid, label in WANTED.items():
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
            # Windows says attached. If Linux has no video node at all, that
            # belief is stale and `attach` will refuse rather than repair.
            if not before:
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
            say("\n  Nothing to do -- the cameras are in WSL.")
            return 0
        say("\n  No camera is attached and none is shared. See the bind step "
            "above.")
        return 1

    if not a.fix:
        say("\n  Re-run with --fix to repair %d device(s)." % len(todo))
        return 1

    say("")
    for busid, label in stale:
        say("  REPAIRING STALE %s (%s): Windows says attached, WSL has "
            "nothing." % (busid, label))
        rc, out = usbipd("detach", "--busid", busid)
        say("    detach: %s" % (out.strip().splitlines()[-1]
                                if out.strip() else "ok"))
        time.sleep(2.0)
    for busid, label in todo:
        rc, out = usbipd("attach", "--wsl", "--busid", busid)
        last = out.strip().splitlines()[-1] if out.strip() else "ok"
        say("  attach %s (%s): %s" % (busid, label, last))

    # THE ONLY ANSWER THAT COUNTS is whether Linux can see them now.
    for _ in range(10):
        time.sleep(1.0)
        after = video_nodes()
        if after and after != before:
            break
    after = video_nodes()
    say("\n  /dev/video* now: %s" % (", ".join(after) if after else "STILL NONE"))
    if not after:
        say("  The attach reported success and Linux still has no device. "
            "That is usually the service having been restarted under a live "
            "attach; unplug the camera and plug it back in.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
