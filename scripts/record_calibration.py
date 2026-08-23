#!/usr/bin/env python3
"""FILM THE CALIBRATION SWEEP AND THE PICK IT PLANS FROM THE MAP.

    ./.venv_vision/bin/python scripts/record_calibration.py --arm left

Writes `recordings/calibration/<stamp>/` with

    calibrate.mp4   the arm sweeping the workspace in straight rows
    pick.mp4        the pick and place, planned from the map that sweep made
    narration.txt   every sentence the robot published on /robot_say, timed
    world_map.json  the map the pick was planned from
    summary.txt     what was measured, so the clip is not the only record

WHY Xvfb AND NOT THE REAL SCREEN
--------------------------------
x11grab on WSLg's own :0 records BLACK, measured: XWayland windows are
composited by Wayland and their pixels never reach the X root window x11grab
reads. Xvfb has no compositor, so its root window really does hold the pixels.
`record_rviz` found this the hard way and this file reuses the finding rather
than rediscovering it.

WHAT IS BEING FILMED AND WHAT IS NOT
------------------------------------
RViz, driven by the SIMULATED arm. There is no camera on this host and no real
arm has ever been driven from this repository, so the clip shows the commanded
motion against the modelled scene. The depth the map was built from was
RENDERED by `mock_rgbd_camera`. Both facts are written into summary.txt, which
is the point of writing one.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RVIZ_CFG = os.path.join(ROOT, "src/srl_experiments/config/verification_capture.rviz")
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")
DISP = ":97"
W, H = 1280, 800


def _running(pat):
    out = subprocess.run(["ps", "-eo", "args="], capture_output=True,
                         text=True).stdout
    return any(pat in ln for ln in out.splitlines())


def start_display():
    """Xvfb plus one RViz on it. Idempotent."""
    if not _running("Xvfb %s" % DISP):
        subprocess.Popen(["setsid", "/usr/bin/Xvfb", DISP, "-screen", "0",
                          "%dx%dx24" % (W, H)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
    if not _running("rviz2 -d %s" % RVIZ_CFG):
        env = dict(os.environ, DISPLAY=DISP, LIBGL_ALWAYS_SOFTWARE="1",
                   QT_X11_NO_MITSHM="1")
        subprocess.Popen(["rviz2", "-d", RVIZ_CFG], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(12)


def grab(path, seconds=None):
    """Start ffmpeg on the virtual display. Returns the Popen to stop later."""
    argv = [FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
            "-framerate", "10", "-video_size", "%dx%d" % (W, H),
            "-i", DISP]
    if seconds:
        argv += ["-t", str(seconds)]
    argv += ["-pix_fmt", "yuv420p", "-c:v", "libx264", "-preset", "veryfast",
             path]
    return subprocess.Popen(argv, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def brightness(path):
    """MEAN PIXEL VALUE OF THE RESULT, because a black clip has a file size.

    `record_rviz` records that x11grab on the wrong display produces a
    perfectly valid mp4 of nothing. A recorder that does not look at its own
    output cannot tell that apart from a successful capture.
    """
    out = os.path.join(os.path.dirname(path), "_probe.png")
    r = subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-i", path,
                        "-frames:v", "1", "-ss", "3", out],
                       capture_output=True)
    if r.returncode or not os.path.exists(out):
        return -1.0
    try:
        import numpy as np
        from PIL import Image
        a = np.asarray(Image.open(out).convert("L"), float)
        return float(a.mean())
    except Exception:                                          # noqa: BLE001
        try:
            import cv2
            import numpy as np
            return float(np.asarray(cv2.imread(out, 0), float).mean())
        except Exception:                                      # noqa: BLE001
            return -1.0
    finally:
        if os.path.exists(out):
            os.remove(out)


class Narration:
    """Every sentence the robot published while filming, with its time."""

    def __init__(self):
        self.lines = []

    def start(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("narration_recorder")
        self.t0 = time.time()
        self.node.create_subscription(String, "/robot_say", self._on, 50)

    def _on(self, m):
        try:
            d = json.loads(m.data)
        except Exception:                                      # noqa: BLE001
            return
        self.lines.append((time.time() - self.t0, d.get("phase", ""),
                           d.get("text", "")))

    def spin(self, secs):
        import rclpy
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def write(self, path):
        with open(path, "w") as f:
            for t, phase, txt in self.lines:
                f.write("%7.1f s  %-12s %s\n" % (t, phase, txt))
        return len(self.lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both",
                    choices=("left", "right", "both"))
    ap.add_argument("--object", type=int, default=None,
                    help="which mapped object to pick. Default: the "
                         "graspable one nearest the middle of the sweep.")
    ap.add_argument("--to", nargs=2, type=float, default=[0.290, 0.500])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(ROOT, "recordings/calibration", stamp)
    os.makedirs(out, exist_ok=True)
    py = os.path.join(ROOT, ".venv_vision/bin/python")

    print("filming into %s" % out, flush=True)
    start_display()
    nar = Narration()
    nar.start()

    # ---------------------------------------------------------- 1 calibrate
    print("### calibrating -- the arm sweeps the workspace", flush=True)
    cap = grab(os.path.join(out, "calibrate.mp4"))
    p = subprocess.Popen(
        [py, "-u", os.path.join(HERE, "calibrate_environment.py"),
         "--arm", a.arm, "--step-m", "0.11",
         "--out", os.path.join(out, "world_map.json")],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log = []
    while p.poll() is None:
        nar.spin(0.3)
    log = (p.stdout.read() or "").splitlines()
    cap.terminate()
    cap.wait(timeout=20)
    rc_cal = p.returncode

    # ------------------------------------------------------------- 2 pick
    rc_pick = None
    if rc_cal == 0:
        print("### picking -- planned from the map that sweep just made",
              flush=True)
        argv = [py, "-u", os.path.join(HERE, "pick_from_map.py"),
                "--arm", ("left" if a.arm == "both" else a.arm),
                "--map", os.path.join(out, "world_map.json"),
                "--to", str(a.to[0]), str(a.to[1]), "--execute"]
        if a.object is None:
            argv += ["--nearest", "0.45", "0.45"]
        else:
            argv += ["--object", str(a.object)]
        cap2 = grab(os.path.join(out, "pick.mp4"))
        p2 = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
        while p2.poll() is None:
            nar.spin(0.3)
        log += (p2.stdout.read() or "").splitlines()
        cap2.terminate()
        cap2.wait(timeout=20)
        rc_pick = p2.returncode

    n = nar.write(os.path.join(out, "narration.txt"))
    with open(os.path.join(out, "run.log"), "w") as f:
        f.write("\n".join(log))

    bright = {}
    for name in ("calibrate.mp4", "pick.mp4"):
        pth = os.path.join(out, name)
        if os.path.exists(pth):
            bright[name] = brightness(pth)

    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write("calibration and map-driven pick, %s\n\n" % stamp)
        f.write("calibration exit %s, pick exit %s\n" % (rc_cal, rc_pick))
        f.write("%d narration sentence(s) captured\n\n" % n)
        mp = os.path.join(out, "world_map.json")
        if os.path.exists(mp):
            import json as _j
            d = _j.load(open(mp))
            sw = d.get("sweep", {})
            f.write("arms swept        %s\n" % sw.get("arms"))
            f.write("cells planned     %s   reachable %s   outside envelope %s\n"
                    % (sw.get("cells"), sw.get("cells_reachable"),
                       sw.get("cells_outside_envelope")))
            f.write("views captured    %s  (%.0f%% of reachable)\n"
                    % (sw.get("views_used"),
                       100 * float(sw.get("coverage_of_reachable", 0))))
            f.write("stillness refusals %s\n" % sw.get("stillness_refusals"))
            f.write("surface measured  %.4f m\n\n" % d["surface"]["z_m"])
        for k, v in bright.items():
            f.write("%-14s mean pixel %.1f%s\n"
                    % (k, v, "   BLACK -- the capture failed" if v < 5
                       else ""))
        f.write("""
WHAT THIS IS AND IS NOT
  The arm is SIMULATED. Nothing in this repository has ever driven a real
  Kinova, and the depth the map was built from was RENDERED by
  mock_rgbd_camera -- so this films the pipeline, not the room.
  What IS real here: the sweep order, the deprojection, the segmentation, the
  fusion, the plane fit, every refusal, and the IK the pick was solved with.
""")
    print(open(os.path.join(out, "summary.txt")).read())
    for name, v in bright.items():
        if v < 5:
            print("THE CAPTURE FAILED for %s (mean pixel %.1f). A black clip "
                  "has a perfectly good file size." % (name, v))
            return 5
    return 0 if rc_cal == 0 and rc_pick == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
