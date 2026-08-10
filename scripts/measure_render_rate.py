#!/usr/bin/env python3
"""What frame rate can RViz RENDER on the capture display, and under which driver?

THE CAPTURE RATE WAS NEVER THE LIMIT. `record_rviz.py` asks x11grab for 12 fps
and x11grab always delivers 12 -- when the source has not repainted it grabs
the same pixels again, so every clip's header reads 12 fps whatever RViz
managed. Counting frames that DIFFER from their predecessor across the clips
on disk gives 0.6-1.3 fps, and RViz's own status bar, legible in the corner of
every recorded frame, reads 4-6 fps. Raising `-framerate` cannot help: it would
write more duplicates of the same picture.

So the lever is the RENDER rate, and the question is the GL driver. The
recorder forces `LIBGL_ALWAYS_SOFTWARE=1` because WSLg's :0 records black and
Xvfb has no GPU -- but this machine exposes /dev/dxg and ships the WSL d3d12
user-mode driver, so Mesa's d3d12 gallium backend may reach the GPU from a
plain X server after all.

WHAT IS MEASURED, AND WHY THE SOURCE IS DRIVEN. A stationary RViz repaints
almost never, so a static scene would score every driver at zero and rank them
identically. This runs its own robot_state_publisher over the real URDF and
sweeps all fourteen joints continuously, then counts distinct frames. The
negative control is the same capture with the sweep stopped: it must collapse.

    python3 scripts/measure_render_rate.py
"""
import argparse
import json
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
from srl_teleop import procscan                                # noqa: E402
from measure_capture_rate import (distinct_frames, ensure_xvfb,  # noqa: E402
                                  capture, DISPLAY)

SCRATCH = os.environ.get("SRL_SCRATCH", "/tmp")
XACRO = os.path.join(WS, "src/srl_description/urdf/srl_dual.urdf.xacro")
RVIZ_CFG = os.path.join(WS,
                        "src/srl_experiments/config/verification_capture.rviz")

# The drivers worth trying, in the order they would be preferred.
DRIVERS = [
    ("llvmpipe (shipped)", dict(LIBGL_ALWAYS_SOFTWARE="1",
                                GALLIUM_DRIVER="llvmpipe")),
    ("d3d12 (WSL GPU)", dict(LIBGL_ALWAYS_SOFTWARE="0",
                             GALLIUM_DRIVER="d3d12",
                             LD_LIBRARY_PATH="/usr/lib/wsl/lib:" +
                             os.environ.get("LD_LIBRARY_PATH", ""))),
    ("zink (GL on Vulkan)", dict(LIBGL_ALWAYS_SOFTWARE="0",
                                 MESA_LOADER_DRIVER_OVERRIDE="zink",
                                 LD_LIBRARY_PATH="/usr/lib/wsl/lib:" +
                                 os.environ.get("LD_LIBRARY_PATH", ""))),
]


def urdf_text():
    r = subprocess.run(["xacro", XACRO], capture_output=True, text=True,
                       timeout=180)
    if r.returncode != 0:
        raise RuntimeError("xacro failed: %s" % (r.stderr or "")[-300:])
    return r.stdout


class Source:
    """A real robot_state_publisher plus a continuous joint sweep.

    Both are needed: /joint_states alone moves nothing, because without
    robot_state_publisher there is no TF and RViz draws no robot at all. That
    mistake produced a 1 fps reading against an empty scene on the first
    attempt here.
    """

    def __init__(self):
        self.rsp = None
        self.mv = None

    def start(self):
        u = urdf_text()
        f = os.path.join(SCRATCH, "rate_robot.urdf")
        open(f, "w").write(u)
        self.rsp = subprocess.Popen(
            ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
             f], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        code = (
            "import math,time,rclpy\n"
            "from rclpy.node import Node\n"
            "from sensor_msgs.msg import JointState\n"
            "rclpy.init(); n=Node('render_mover')\n"
            "pub=n.create_publisher(JointState,'/joint_states',10)\n"
            "N=['%s_joint_%d'%(a,j) for a in ('left','right') "
            "for j in range(1,8)]\n"
            "t0=time.monotonic()\n"
            "while rclpy.ok():\n"
            "    t=time.monotonic()-t0\n"
            "    m=JointState(); m.header.stamp=n.get_clock().now().to_msg()\n"
            "    m.name=N\n"
            "    m.position=[1.1*math.sin(1.6*t+0.45*k) for k in range(14)]\n"
            "    pub.publish(m); time.sleep(0.02)\n")
        self.mv = subprocess.Popen([sys.executable, "-c", code],
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        time.sleep(6)

    def pause(self):
        if self.mv:
            self.mv.terminate()
            try:
                self.mv.wait(5)
            except Exception:                                  # noqa: BLE001
                self.mv.kill()
            self.mv = None

    def stop(self):
        self.pause()
        if self.rsp:
            self.rsp.terminate()
            try:
                self.rsp.wait(5)
            except Exception:                                  # noqa: BLE001
                self.rsp.kill()
            self.rsp = None


def run_rviz(env_extra, w, h):
    env = dict(os.environ, DISPLAY=DISPLAY, QT_QPA_PLATFORM="xcb")
    env.pop("MESA_LOADER_DRIVER_OVERRIDE", None)
    env.update(env_extra)
    cmd = ["rviz2"] + (["-d", RVIZ_CFG] if os.path.exists(RVIZ_CFG) else [])
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    time.sleep(16)
    return p


def kill_rviz(p):
    """Kill by HANDLE -- see the note in main() on procscan's blind spot."""
    if p is None:
        return
    try:
        p.terminate()
        p.wait(6)
    except Exception:                                          # noqa: BLE001
        try:
            p.kill()
            p.wait(4)
        except Exception:                                      # noqa: BLE001
            pass
    time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--size", default="800x500")
    ap.add_argument("--request", type=int, default=30)
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))

    ensure_xvfb(1920, 1080)
    src = Source()
    src.start()
    rows = []
    try:
        for name, env in DRIVERS:
            p = run_rviz(env, w, h)
            # LIVENESS FROM THE HANDLE, NOT FROM procscan. procscan excludes
            # the caller's own process group -- that exclusion is what makes
            # it immune to the pgrep self-match trap -- and a Popen child
            # INHERITS that group, so procscan cannot see a process this
            # script started. The first version checked procscan.count and
            # concluded every driver "DID NOT START" while seven live rviz2
            # processes piled up behind it, because the same blind spot made
            # the killer a no-op too. Use procscan for OTHER people's
            # processes; use the handle for your own.
            if p.poll() is not None:
                rows.append(dict(driver=name, ok=False,
                                 why="rviz2 did not start under this driver"))
                print("  %-22s DID NOT START" % name)
                continue
            out = os.path.join(SCRATCH, "render_%s.mp4"
                               % name.split()[0].strip("()"))
            capture(a.request, a.seconds, w, h, out)
            tot, dist = distinct_frames(out, w, h)
            fps = dist / a.seconds
            rows.append(dict(driver=name, ok=True, stored=tot, distinct=dist,
                             render_fps=round(fps, 2)))
            print("  %-22s stored %4d, distinct %4d -> %5.2f render fps"
                  % (name, tot, dist, fps))
            kill_rviz(p)

        # NEGATIVE CONTROL on the best driver: stop the sweep, keep everything
        # else identical. A detector that reports motion here is measuring
        # encoder noise and every row above is void.
        best = max([r for r in rows if r.get("ok")],
                   key=lambda r: r["render_fps"], default=None)
        static_fps = None
        if best:
            env = dict(DRIVERS[[r["driver"] for r in rows].index(
                best["driver"])][1])
            p = run_rviz(env, w, h)
            src.pause()
            time.sleep(3)
            out = os.path.join(SCRATCH, "render_static.mp4")
            capture(a.request, a.seconds, w, h, out)
            tot_s, dist_s = distinct_frames(out, w, h)
            static_fps = dist_s / a.seconds
            kill_rviz(p)
            print("\n  NEGATIVE CONTROL (sweep stopped, %s): distinct %d, "
                  "%.2f fps -> %s"
                  % (best["driver"], dist_s, static_fps,
                     "valid" if static_fps < 1.0 else "INVALID"))
    finally:
        src.stop()
        subprocess.run(["pkill", "-9", "-x", "rviz2"], check=False)

    res = dict(rows=rows, size=a.size, requested=a.request,
               seconds=a.seconds, static_fps=static_fps,
               control_valid=bool(static_fps is not None and static_fps < 1.0))
    p = os.path.join(WS, "recordings/baselines/render_rate.json")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    json.dump(res, open(p, "w"), indent=2)
    print("-> %s" % p)
    return 0 if res["control_valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
