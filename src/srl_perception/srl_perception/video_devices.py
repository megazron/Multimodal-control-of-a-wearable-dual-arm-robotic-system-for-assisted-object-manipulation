#!/usr/bin/env python3
"""Which /dev/videoN is which camera, and which of them is free.

WHY THIS EXISTS. `/dev/video0` is not a camera; it is a POSITION IN A LIST
that changes. On this rig, measured 2026-08-29:

    video0, video1   HD USB CAMERA         32e4:0317   the wide view of the rig
    video2 .. video7 Intel RealSense D435i 8086:0b3a   colour, depth, two IR

The numbers move. They move when a device is re-attached over usbip (the
service drops devices on its own and hands the same camera back under a
different number), and they move when one camera is attached before the
other. Anything that names a number is wrong the first time that happens,
and its failure is silent: the node opens SOMETHING, publishes SOMETHING,
and the picture is of the wrong camera -- or of an infrared sensor, which is
a grey image of the same room and reads as a working camera.

Three questions this answers, none of which a path can:

  * WHICH DEVICE IS WHICH. From sysfs: the driver's card name, and the USB
    VID:PID of the device the interface hangs off. No v4l2-ctl -- it is not
    installed here, and a missing tool must not turn "which camera" into
    "crashed".
  * IS SOMEBODY ALREADY USING IT. `quest_bridge_node` claims the HD camera
    the moment the VR chain starts, and a second reader gets an open that
    succeeds and frames that never come. That is indistinguishable from a
    broken camera unless somebody asks.
  * IS IT COLOUR. Four of the RealSense's six nodes are depth or infrared.
    They open, they deliver, and they look like a working camera to any
    check that only asks whether a frame arrived.
"""
import glob
import os
import re


class Dev:
    """One /dev/videoN, with the identity the path does not carry."""

    def __init__(self, path, name="", vid_pid="", usb_path="", intf="",
                 index=0):
        self.path = path
        self.name = name
        self.vid_pid = vid_pid
        self.usb_path = usb_path
        # THE STABLE HALF OF THE IDENTITY. `1-2:1.3` is bus-and-port then
        # INTERFACE; the bus-and-port moves when the cable does and the
        # interface number does not, because it comes from the device's own
        # descriptors. A camera with several functions -- the RealSense has
        # depth and infrared on interface 1.0 and colour on 1.3 -- is told
        # apart by this and by `index`, and by nothing else available
        # without opening the device.
        self.intf = intf
        # v4l2's own node index within the interface. 0 is the capture node;
        # 1 upwards are metadata nodes, which never deliver a picture.
        self.index = index

    @property
    def key(self):
        """What is written down as 'the camera that worked'. Never a path."""
        return "%s/%s/%d" % (self.vid_pid, self.intf, self.index)

    @property
    def node(self):
        """The number in /dev/videoN. For ordering only -- never identity."""
        m = re.search(r"(\d+)$", self.path)
        return int(m.group(1)) if m else -1

    def matches(self, want):
        """Case-insensitive substring over name and VID:PID. `want` is what
        an operator would type: 'realsense', 'hd usb', '8086:0b3a'."""
        if not want:
            return True
        w = want.strip().lower()
        return w in self.name.lower() or w in self.vid_pid.lower()

    def __repr__(self):
        return "Dev(%s, %r, %s)" % (self.path, self.name, self.key)


def _read(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return ""


def _usb_identity(sysdir):
    """Walk up from the v4l2 interface to the USB device that owns it.

    The interface directory (`1-2:1.3`) carries no idVendor; the device
    directory one level up (`1-2`) does. Walking is what makes this work for
    a hub, a vhci_hcd root and a real controller alike.
    """
    d = sysdir
    for _ in range(8):
        if not d or d == "/":
            break
        vid, pid = _read(os.path.join(d, "idVendor")), \
            _read(os.path.join(d, "idProduct"))
        if vid and pid:
            return "%s:%s" % (vid, pid), d
        d = os.path.dirname(d)
    return "", ""


def devices():
    """Every /dev/videoN with what is known about it, in path order."""
    out = []
    for path in sorted(glob.glob("/dev/video*"),
                       key=lambda p: (len(p), p)):
        node = "/sys/class/video4linux/%s" % os.path.basename(path)
        name = _read(os.path.join(node, "name"))
        ifdir = os.path.realpath(os.path.join(node, "device"))
        vid_pid, usb = _usb_identity(ifdir)
        intf = os.path.basename(ifdir)
        intf = intf.split(":", 1)[1] if ":" in intf else intf
        try:
            index = int(_read(os.path.join(node, "index")) or 0)
        except ValueError:
            index = 0
        out.append(Dev(path, name, vid_pid, usb, intf, index))
    return out


def holders():
    """{/dev/videoN: [pid, ...]} for every video device something has open.

    ONE WALK OF /proc, NOT ONE PER DEVICE. The first version asked per
    device, and `sorted()` calls its key function once per element, so
    ordering eight nodes walked every process's file descriptors eight
    times -- which on a machine with a full stack up took longer than the
    camera probe it was supposed to make faster, and the node was killed
    before it opened anything.

    Read from /proc rather than by trying to open the device: opening a
    V4L2 device to find out whether it is busy can itself disturb whatever
    has it, and on this stack that is the VR bridge.
    """
    out = {}
    me = os.getpid()
    for pid_dir in glob.glob("/proc/[0-9]*"):
        try:
            pid = int(os.path.basename(pid_dir))
        except ValueError:
            continue
        if pid == me:
            continue
        try:
            fds = os.listdir(os.path.join(pid_dir, "fd"))
        except OSError:
            continue                      # gone, or not ours to read
        for fd in fds:
            try:
                tgt = os.readlink(os.path.join(pid_dir, "fd", fd))
            except OSError:
                continue
            if tgt.startswith("/dev/video"):
                out.setdefault(tgt, [])
                if pid not in out[tgt]:
                    out[tgt].append(pid)
    return out


def users(path, held=None):
    """PIDs holding this device open, excluding this process."""
    held = holders() if held is None else held
    return list(held.get(os.path.realpath(path), []))


#: Where the last camera that actually worked is written down. A probe of
#: this rig costs ~11 s PER DEVICE that opens and never delivers -- the
#: RealSense's infrared node is one -- so a cold probe is 20-40 s of an
#: operator watching nothing happen, EVERY session, for an answer that was
#: already known last session.
MEMORY = os.path.expanduser("~/.srl_scene_camera.json")


def remember(dev, path=MEMORY):
    """Write down the camera that delivered. Best effort, never fatal."""
    try:
        import json
        with open(path, "w") as fh:
            json.dump({"key": dev.key, "name": dev.name,
                       "path_then": dev.path}, fh)
        return True
    except Exception:                                         # noqa: BLE001
        return False


def remembered(path=MEMORY):
    """The key of the camera that worked last time, or ''.

    NOT AUTHORITATIVE. It sorts a candidate first and nothing more, so a
    camera that has been unplugged, or that stops delivering, costs one
    failed probe rather than a wrong answer -- which is the difference
    between a cache and a hardcoded path.
    """
    try:
        import json
        with open(path) as fh:
            return str(json.load(fh).get("key") or "")
    except Exception:                                         # noqa: BLE001
        return ""


def order(prefer="", avoid_busy=True, devs=None, memory=MEMORY):
    """The order to TRY devices in. Never a filter -- always everything.

    A preference that EXCLUDED would turn "the camera I wanted is busy" into
    "there is no camera", which is the same class of silent substitution
    this module exists to stop. So a wanted device sorts first, a busy one
    sorts last, and the caller still gets the whole list to report from.

    The order, most significant first:
      1. the camera that worked last time,
      2. what the caller asked for,
      3. a capture node before a metadata node (index 0 is the picture;
         1 upwards never deliver one),
      4. a free device before a busy one,
      5. the /dev/videoN number, so the order is stable run to run.
    """
    devs = devices() if devs is None else list(devs)
    last = remembered(memory) if memory else ""
    held = holders() if avoid_busy else {}

    # Only let a preference lead if it names something plugged in.
    # A typo must not demote the remembered camera to nothing.
    asked = bool(prefer) and any(d.matches(prefer) for d in devs)

    def key(d):
        want = 0 if (asked and d.matches(prefer)) else 1
        seen = 0 if (last and d.key == last) else 1
        rank = (want, seen) if asked else (seen, want)
        return rank + (0 if d.index == 0 else 1,
                       1 if users(d.path, held) else 0,
                       d.node)
    return sorted(devs, key=key)


def is_colour(frame, tol=2.0):
    """True if the three planes actually differ -- i.e. this is not IR.

    The RealSense's infrared nodes deliver a three-channel BGR image whose
    planes are IDENTICAL. It opens, it delivers, and every check that asks
    only "did a frame arrive" accepts a grey picture of the room as the
    scene camera.

    `tol` is in grey levels of mean absolute difference. Zero would be
    correct for a true mono source, but MJPEG chroma subsampling puts a
    fraction of a level of noise on a grey image, so the threshold is small
    rather than exact.
    """
    if frame is None or getattr(frame, "ndim", 0) != 3 or frame.shape[2] < 3:
        return False
    import numpy as np
    f = frame.astype("float32")
    bg = float(np.mean(np.abs(f[:, :, 0] - f[:, :, 1])))
    gr = float(np.mean(np.abs(f[:, :, 1] - f[:, :, 2])))
    return max(bg, gr) > tol
