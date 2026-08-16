#!/usr/bin/env python3
"""SHOOT THE T1 SCENE, from the config the clips are actually filmed with.

    python3 scripts/sim_session.py --stack moveit --keep-up -- true
    python3 scripts/render_t1_scene.py --task t1

Brings up `clip_scene` for the task, waits for the markers to exist, and grabs
a still per view on a virtual display. It reuses `record_rviz.write_cfg` so the
picture is taken through the same camera the recorded set uses -- a still shot
any other way is not evidence about the clips.

A BLANK FRAME IS NOT A SHOT. Every grab is measured for ink against the modal
background colour and refused below 2%, which is the check that caught a white
rectangle being filed as a render once already.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import record_rviz as RR                                          # noqa: E402
from measure_home_render import _ink, FFMPEG                      # noqa: E402


def shoot(name, task, out_dir, settle=22):
    cfg = RR.write_cfg(name, task=task)
    disp = RR.VIEWS[name][0]
    size = "%dx%d" % (RR.VW, RR.VH)
    if subprocess.run(["pgrep", "-f", "Xvfb %s" % disp],
                      capture_output=True).returncode != 0:
        subprocess.Popen(["setsid", "/usr/bin/Xvfb", disp, "-screen", "0",
                          size + "x24", "-nolisten", "tcp"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
    env = dict(os.environ, DISPLAY=disp, LIBGL_ALWAYS_SOFTWARE="1")
    rv = subprocess.Popen(["rviz2", "-d", cfg], env=env,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(settle)
    os.makedirs(out_dir, exist_ok=True)
    png = os.path.join(out_dir, "t1_%s.png" % name)
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
                    "-video_size", size, "-i", disp, "-frames:v", "1", png],
                   check=False)
    rv.terminate()
    try:
        rv.wait(timeout=10)
    except Exception:                                             # noqa: BLE001
        rv.kill()
    if not os.path.exists(png):
        return None, 0.0
    frac = _ink(png)
    return (png if frac >= 0.02 else None), frac


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="t1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--views", nargs="*",
                    default=["front", "top", "iso"])
    ap.add_argument("--out", default=os.path.join(
        os.environ.get("SRL_SCRATCH", "/tmp"), "t1_render"))
    a = ap.parse_args()

    src = ("set +u; source /opt/ros/jazzy/setup.bash; "
           "source %s/install/setup.bash; set -u; "
           "export FASTDDS_BUILTIN_TRANSPORTS=SHM" % ROOT)
    scene = subprocess.Popen(
        ["bash", "-lc", "%s && exec python3 %s/scripts/clip_scene.py "
         "--task %s --seed %d" % (src, ROOT, a.task, a.seed)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    print("clip_scene up for task %s; settling" % a.task)
    time.sleep(12)
    try:
        for v in a.views:
            png, frac = shoot(v, a.task, a.out)
            print("   %-8s %-6s ink %.3f%%  %s"
                  % (v, "OK" if png else "BLANK", frac * 100.0, png or ""))
    finally:
        import signal
        try:
            os.killpg(os.getpgid(scene.pid), signal.SIGINT)
            scene.wait(timeout=8)
        except Exception:                                         # noqa: BLE001
            try:
                os.killpg(os.getpgid(scene.pid), signal.SIGKILL)
            except Exception:                                     # noqa: BLE001
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
