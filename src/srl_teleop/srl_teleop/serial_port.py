#!/usr/bin/env python3
"""
serial_port.py — find the Teensy, wherever usbipd put it this time
=============================================================================
The board moves between /dev/ttyACM0 and /dev/ttyACM1 on every usbipd
re-attach, which has caused several failed runs. Worse, the old failure mode
was silent: a node pointed at the wrong path set self.ser = None and then
spun forever with no data, which looks exactly like a dead board. This module
exists so that a wrong port fails LOUDLY and immediately, and so the
detection logic lives in one place instead of being copy-pasted into every
node that opens the serial link.

Detection: prefer a stable /dev/teensy symlink if a udev rule provides one,
then glob /dev/ttyACM* and /dev/ttyUSB*, open each candidate and sniff for a
line containing "k1j1". The first that responds wins.
"""
import glob
import os
import time

BAUD = 115200

# A udev rule can pin the board to a stable name; if it exists, trust it
# first. Suggested rule (not installed by this repo):
#   SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", \
#     SYMLINK+="teensy"
PREFERRED = "/dev/teensy"

# Every master frame carries this token, on both arms' lines.
SNIFF_TOKEN = b"k1j1"

REATTACH_HINT = (
    "The Teensy moves between /dev/ttyACM0 and /dev/ttyACM1 on every "
    "re-attach. From Windows PowerShell (Admin):\n"
    "    usbipd attach --wsl --hardware-id 16c0:0483\n"
    "Then retry. To bypass autodetection, pass an explicit path, e.g. "
    "-p serial_port:=/dev/ttyACM1")


class PortNotFound(RuntimeError):
    """No candidate device produced master frames."""


def candidates():
    """Plausible device paths, best first, de-duplicated by real path."""
    c = []
    if os.path.exists(PREFERRED):
        c.append(PREFERRED)
    c += sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    seen, out = set(), []
    for p in c:
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        out.append(p)
    return out


def sniff(port, baud=BAUD, settle=1.5, tries=20, timeout=0.5):
    """Open `port` and look for a master frame. Returns (ok, reason).

    The reason is kept and reported on failure: "busy" and "silent" are very
    different problems, and collapsing them into one message is what made
    the old behaviour so hard to diagnose.
    """
    import fcntl
    import serial
    try:
        with serial.Serial(port, baud, timeout=timeout) as s:
            # NEVER SNIFF A PORT SOMEONE ELSE IS READING. The sniffer READS,
            # so probing a port a live node owns steals bytes out of its
            # frame stream -- the same corruption the exclusive lock exists
            # to prevent, introduced by the code that looks for the port.
            try:
                fcntl.flock(s.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return False, ("IN USE%s -- not probed, because probing "
                               "would steal bytes from the running reader"
                               % _lock_holder(port))
            time.sleep(settle)          # USB CDC needs a moment after open
            for _ in range(tries):
                if SNIFF_TOKEN in s.readline():
                    return True, "sent master frames"
        return False, "opened, but sent no master frames (board idle or wrong device)"
    except Exception as e:                        # serial.SerialException, OSError
        return False, "could not open: %s" % e


def claim_exclusive(ser, logger=None):
    """Take an exclusive advisory lock on the OPEN serial fd.

    TWO READERS ON ONE TEENSY IS THE WORST FAILURE THIS RIG HAS HAD, because
    it is invisible. On 2026-08-06 two teleop stacks were running and both
    master_pose_node instances held /dev/ttyACM0 open. Neither errored --
    POSIX lets any number of processes read the same tty -- but the kernel
    hands each byte to exactly ONE reader, so each process parsed fragments
    of the frame stream. Measured in that state: the raw topic ran at 68 Hz
    with 100% of rows distinct, where one healthy stream gives ~15 Hz of
    distinct updates inside 50 Hz rows. Every channel-health number taken
    that day had to be discarded.

    Detecting it afterwards is not good enough, so this makes it IMPOSSIBLE
    rather than merely visible: LOCK_EX|LOCK_NB on the fd. The second opener
    fails at startup, by name, with the holder's PID.

    The lock is advisory, but it binds everything that goes through this
    module -- which is every node in this package that reads the master.
    """
    import fcntl
    try:
        fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        holder = _lock_holder(ser.port)
        msg = ("REFUSING TO START: %s is already held by another "
               "master_pose_node%s. "
               "Two readers SPLIT the serial stream between them -- each gets "
               "fragments of each frame, every channel looks incoherent, and "
               "nothing errors. Stop the other stack first:\n"
               "    ps -eo pid,cmd | grep '[m]aster_pose_node'\n"
               "    kill -INT <pid>" % (ser.port, holder))
        if logger:
            logger.error(msg)
        raise PortNotFound(msg)
    return ser


def _lock_holder(port):
    """Best-effort ' (held by PID n, <cmd>)'. Never raises."""
    try:
        import glob
        import os
        for fd in glob.glob("/proc/[0-9]*/fd/*"):
            try:
                if os.readlink(fd) != port:
                    continue
            except OSError:
                continue
            pid = fd.split("/")[2]
            if pid == str(os.getpid()):
                continue
            try:
                cmd = open("/proc/%s/cmdline" % pid).read().replace(
                    "\0", " ").strip()
            except OSError:
                cmd = "?"
            return " (held by PID %s: %s)" % (pid, cmd[:90])
    except Exception:                                        # noqa: BLE001
        pass
    return ""


def find_port(explicit=None, baud=BAUD, logger=None):
    """Resolve the port to use.

    `explicit` of None or "auto" triggers autodetection; any other value is
    returned unchanged so an operator can always override. Raises
    PortNotFound rather than returning None -- callers must not be able to
    proceed into a silent no-data spin.
    """
    def say(msg):
        if logger is not None:
            logger.info(msg)

    if explicit and explicit != "auto":
        say("Using explicitly configured serial port %s (autodetect bypassed)"
            % explicit)
        return explicit

    cands = candidates()
    if not cands:
        raise PortNotFound("No /dev/ttyACM* or /dev/ttyUSB* device present.\n"
                           + REATTACH_HINT)

    say("Autodetecting Teensy among: %s" % ", ".join(cands))
    reasons = []
    for p in cands:
        ok, why = sniff(p, baud)
        if ok:
            say("Teensy found on %s (%s)" % (p, why))
            return p
        reasons.append("  %-16s %s" % (p, why))

    raise PortNotFound(
        "Checked %d device(s); none sent master frames:\n%s\n%s"
        % (len(cands), "\n".join(reasons), REATTACH_HINT))
