#!/usr/bin/env python3
"""camera_doctor.py -- WHICH LAYER IS BROKEN, and fix the ones that can be.

    python3 scripts/camera_doctor.py            # check and name the fault
    python3 scripts/camera_doctor.py --fix      # repair what is repairable
    python3 scripts/camera_doctor.py --watch    # keep them working

WHY THIS EXISTS
===============================================================================
Every camera fault on this rig presents IDENTICALLY: a node at 20-30% CPU, a
topic that exists with `Publisher count: 1`, and a blank panel. On
2026-08-29/30 that one symptom was produced by five different causes, and
telling them apart by hand took hours each time:

  1. STALE /dev/shm SEGMENTS. After any kill -9, FastDDS segments leak.
     Publishers publish, subscribers receive nothing. Seen at 214 stale
     entries; the cameras came straight up after clearing them.

  2. QoS MISMATCH. Camera topics are BEST_EFFORT (`qos_profile_sensor_data`).
     A default RELIABLE subscriber -- including a plain `ros2 topic hz` --
     receives ZERO and says "does not appear to be published yet". I
     diagnosed a healthy scene camera as dead this way.

  3. TWO OPENERS OF ONE V4L2 DEVICE. A device cannot be opened twice for
     capture; the loser gets nothing, silently, and which one loses depends
     on start order.

  4. A WEDGED VISION MODULE ON THE ARM. ping and HTTP 200 prove the arm's
     network stack and say nothing about its camera. `kinova_vision` then
     retries for ever, publishing a topic with no frames on it.

  5. A PROBE THAT LIES. The first RTSP check here used a GStreamer pipeline
     that could never link, so it reported "not serving" for healthy
     cameras and refused to start them.

So this walks the layers IN ORDER and stops at the first one that is wrong,
because a fault at layer 2 makes every answer above it meaningless. It names
the layer in the operator's words, and `--fix` repairs the three that can be
repaired without a human: stale shared memory, an unattached USB camera, and
a node that is simply not running.

WHAT IT WILL NOT DO. It will not power-cycle an arm, and it will not kill a
node that is holding a device for a reason. Those need a person, and it says
which and why rather than guessing.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARM_IP = {"left": "192.168.1.10", "right": "192.168.1.9"}

OK, BAD, FIXED, UNKNOWN = "ok", "BROKEN", "repaired", "CANNOT TELL"


def _run(argv, timeout=90, env=None):
    try:
        p = subprocess.run(argv, capture_output=True, text=True,
                           timeout=timeout, env=env)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except Exception as e:                                    # noqa: BLE001
        return 1, "%r" % (e,)


def stale_shm():
    """Leaked FastDDS segments, and whether anything is running to own them."""
    segs = [f for f in os.listdir("/dev/shm") if f.startswith("fastrtps_")
            or f.startswith("sem.fastrtps_")]
    rc, out = _run(["pgrep", "-fc", "ros2|_node|srl_"], timeout=10)
    live = out.strip().isdigit() and int(out.strip()) > 0
    return segs, live


def clear_shm():
    """ONLY with nothing running. Clearing under a live stack breaks it --
    that is recorded in findings.md as a repair that damaged a starting
    stack, and it is why this refuses rather than asking."""
    segs, live = stale_shm()
    if live:
        return False, ("%d segment(s), but ROS processes are running. "
                       "Clearing under a live stack breaks it. Stop "
                       "everything first." % len(segs))
    n = 0
    for f in segs:
        try:
            os.remove(os.path.join("/dev/shm", f))
            n += 1
        except OSError:
            pass
    return True, "cleared %d stale segment(s)" % n


def frames(topic, secs=6.0):
    """Frames seen on `topic` at the CAMERA's QoS, not the default.

    BEST_EFFORT, because that is what every camera here publishes. A default
    RELIABLE subscriber silently receives nothing and reports the camera
    dead -- which is fault 2 above, and it is the one that fooled me.
    """
    code = (
        "import rclpy,time\n"
        "from rclpy.node import Node\n"
        "from rclpy.qos import qos_profile_sensor_data\n"
        "from sensor_msgs.msg import Image\n"
        "rclpy.init()\n"
        "n=Node('camera_doctor_probe')\n"
        "g=[]\n"
        "n.create_subscription(Image,%r,lambda m:g.append(m.width),"
        "qos_profile_sensor_data)\n"
        "t=time.time()\n"
        "while time.time()-t<%f: rclpy.spin_once(n,timeout_sec=0.2)\n"
        "print(len(g))\n" % (topic, secs))
    # PROBE ON THE RIG'S OWN TRANSPORT, or the probe is measuring itself.
    #
    # HARD CONSTRAINT 5: everything here runs FASTDDS_BUILTIN_TRANSPORTS=SHM.
    # A probe that inherits a shell without it cannot SEE the publishers and
    # reports zero frames from three healthy cameras -- which is the same
    # "publisher fine, nobody receives" fault this file exists to diagnose,
    # committed by the diagnosis. Set it explicitly rather than hoping the
    # caller sourced env.sh.
    env = dict(os.environ)
    env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "SHM")
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    rc, out = _run([sys.executable, "-c", code], timeout=secs + 25, env=env)
    for line in reversed(out.strip().splitlines()):
        if line.strip().isdigit():
            return int(line.strip()), None
    # A PROBE THAT CANNOT RUN MUST NOT REPORT ZERO.
    #
    # This returned 0 when the subprocess failed, so a shell without ROS
    # sourced made every camera read "NO FRAMES" -- three healthy cameras
    # declared broken by a probe that never asked them anything. That is
    # fault 5 in this file's own header, committed by this file. The caller
    # gets CANNOT TELL now, which is a different answer from BROKEN and
    # points at the probe rather than the rig.
    why = "rclpy unavailable -- source /opt/ros/jazzy/setup.bash and the " \
          "workspace" if "rclpy" in out else out.strip().splitlines()[-1][:90] \
          if out.strip() else "probe produced no output"
    return None, why


def check_wrist(arm, fix):
    """Layers for a wrist camera: arm reachable, RTSP serving, node up, frames."""
    ip = ARM_IP[arm]
    name = "%s wrist camera" % arm
    if _run(["ping", "-c1", "-W2", ip], timeout=8)[0] != 0:
        return name, BAD, "the arm is off the network at %s" % ip
    rc, detail = _run([sys.executable,
                       os.path.join(WS, "scripts", "rtsp_probe.py"), ip],
                      timeout=30)
    if rc != 0:
        return name, BAD, (
            "the arm is up and its VISION MODULE is not serving (%s). This "
            "does not recover on its own: POWER-CYCLE THE ARM."
            % detail.strip())
    running = _run(["pgrep", "-f", "camera:=%s_camera" % arm], timeout=10)[0] == 0
    if not running:
        if not fix:
            return name, BAD, "camera serving, node not started"
        subprocess.Popen(
            ["setsid", "bash", os.path.join(WS, "scripts", "wrist_cameras.sh"),
             arm], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        time.sleep(20)
        return name, FIXED, "started the node (%s)" % detail.strip()
    n, why = frames("/%s_camera/color/image_raw" % arm)
    if n is None:
        return name, UNKNOWN, "could not probe the topic: %s" % why
    if n == 0:
        return name, BAD, (
            "node running, camera serving, NO FRAMES on the topic. That is "
            "stale shared memory or a QoS mismatch, not the camera.")
    return name, OK, "%d frames in 6 s" % n


def check_scene(fix):
    """Layers for the scene camera: device present, free, node up, frames."""
    name = "scene camera"
    if not glob.glob("/dev/video*"):
        if not fix:
            return name, BAD, "no /dev/video* -- not attached from Windows"
        rc, out = _run([sys.executable,
                        os.path.join(WS, "scripts", "usb_cameras.py"), "--fix"],
                       timeout=180)
        if not glob.glob("/dev/video*"):
            return name, BAD, "usbipd repair did not produce a device"
        return name, FIXED, "attached the camera from Windows"
    running = _run(["pgrep", "-f", "scene_camera_nod[e]"], timeout=10)[0] == 0
    if not running:
        holder = _run(["fuser", "/dev/video0"], timeout=10)[1].strip()
        if holder:
            return name, BAD, (
                "the node is not running and /dev/video0 is held by pid %s. "
                "A V4L2 device cannot be opened twice; that process must "
                "release it first." % holder)
        if not fix:
            return name, BAD, "device free, node not started"
        subprocess.Popen(
            ["setsid", "ros2", "run", "srl_perception", "scene_camera_node"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        time.sleep(15)
        return name, FIXED, "started scene_camera_node"
    n, why = frames("/scene_camera/image_raw")
    if n is None:
        return name, UNKNOWN, "could not probe the topic: %s" % why
    if n == 0:
        return name, BAD, (
            "node running, NO FRAMES. Stale shared memory or the device "
            "stopped delivering.")
    return name, OK, "%d frames in 6 s" % n


def report(fix=False):
    rows = []
    segs, live = stale_shm()
    if len(segs) > 120 and not live:
        if fix:
            ok, detail = clear_shm()
            rows.append(("shared memory", FIXED if ok else BAD, detail))
        else:
            rows.append(("shared memory", BAD,
                         "%d stale FastDDS segments and nothing running -- "
                         "this alone stops every topic delivering" % len(segs)))
    rows.append(check_scene(fix))
    for arm in ("left", "right"):
        rows.append(check_wrist(arm, fix))
    width = max(len(r[0]) for r in rows)
    print("== CAMERA DOCTOR ==")
    for nm, st, detail in rows:
        print("  %-*s  %-9s %s" % (width, nm, st, detail))
    return 0 if all(r[1] != BAD for r in rows) else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true",
                    help="repair what can be repaired without a human")
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--interval", type=float, default=60.0)
    a = ap.parse_args()
    if not a.watch:
        return report(a.fix)
    print("== CAMERA DOCTOR WATCH == every %.0f s; silence means all well."
          % a.interval, flush=True)
    while True:
        try:
            if report(fix=True) == 0:
                pass
            time.sleep(a.interval)
        except KeyboardInterrupt:
            return 0
        except Exception as e:                                # noqa: BLE001
            print("poll failed (%r) -- continuing" % (e,), flush=True)
            time.sleep(a.interval)


if __name__ == "__main__":
    sys.exit(main())
