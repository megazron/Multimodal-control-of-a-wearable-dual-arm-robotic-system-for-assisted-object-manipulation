#!/usr/bin/env python3
"""What frame rate can the RViz capture path ACTUALLY deliver?

WHY THE CONTAINER HEADER IS NOT THE ANSWER. `ffmpeg -f x11grab -framerate N`
always writes N fps: when the source has not repainted, x11grab grabs the same
pixels again and the encoder stores a duplicate. So every clip in this
repository reports its requested rate whatever RViz managed, and a clip that
looks choppy reports 12 fps exactly like a smooth one. Raising `-framerate`
without measuring would produce a 30 fps header over the same 6 real frames a
second -- a bigger file, an identical video, and a number in the docs that is
now wrong in a new way.

WHAT IS MEASURED. Capture a moving RViz at a requested rate, then decode the
result and count frames that DIFFER from their predecessor. That count over
the wall-clock duration is the delivered rate. The gap between requested and
delivered is duplicate frames, i.e. the source could not keep up.

THE SOURCE IS DRIVEN, NOT ASSUMED. A stationary RViz delivers ~0 distinct
frames and would read as a broken capture path, so this publishes a continuous
joint sweep and asserts the robot really moved -- the negative control is that
with the publisher off, the measured rate must collapse.

    python3 scripts/measure_capture_rate.py [--rates 12,24,30,60]
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

FFMPEG = os.environ.get("FFMPEG") or os.path.expanduser("~/.local/bin/ffmpeg")
DISPLAY = ":99"
SCRATCH = os.environ.get("SRL_SCRATCH", "/tmp")
RVIZ_CFG = os.path.join(WS, "src/srl_experiments/config/verification_capture.rviz")


def display_works(disp=DISPLAY):
    try:
        r = subprocess.run([FFMPEG, "-loglevel", "error", "-f", "x11grab",
                            "-video_size", "64x64", "-i", "%s.0" % disp,
                            "-frames:v", "1", "-f", "null", "-"],
                           capture_output=True, timeout=25)
        return r.returncode == 0
    except Exception:                                          # noqa: BLE001
        return False


def ensure_xvfb(w, h):
    if display_works():
        return
    lock = "/tmp/.X%s-lock" % DISPLAY.lstrip(":")
    if os.path.exists(lock) and procscan.count(r"Xvfb\s+" + DISPLAY) == 0:
        try:
            os.remove(lock)
        except OSError:
            pass
    subprocess.Popen(["Xvfb", DISPLAY, "-screen", "0", "%dx%dx24" % (w, h)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(20):
        time.sleep(1.0)
        if display_works():
            return
    raise RuntimeError("Xvfb %s did not come up" % DISPLAY)


def distinct_frames(mp4, w, h):
    """Decode to raw grey and count frames differing from their predecessor.

    Done on the DECODED stream, not on encoded frame types: a duplicate can be
    encoded as a P-frame with residual noise, so counting I/P frames would
    overcount. A byte-level difference with a small tolerance is the thing the
    eye actually sees.
    """
    sw, sh = 160, 100
    p = subprocess.run(
        [FFMPEG, "-loglevel", "error", "-i", mp4, "-vf",
         "scale=%d:%d,format=gray" % (sw, sh), "-f", "rawvideo", "-"],
        capture_output=True, timeout=300)
    buf = p.stdout
    n = sw * sh
    total = len(buf) // n
    prev, distinct = None, 0
    for i in range(total):
        f = buf[i * n:(i + 1) * n]
        if prev is not None:
            diff = sum(1 for a, b in zip(f[::7], prev[::7]) if abs(a - b) > 3)
            if diff > 4:
                distinct += 1
        prev = f
    return total, distinct


class Mover:
    """Publishes a continuous joint sweep so the source genuinely repaints."""

    def __init__(self):
        self.p = None

    def start(self):
        code = (
            "import math,time,rclpy\n"
            "from rclpy.node import Node\n"
            "from sensor_msgs.msg import JointState\n"
            "rclpy.init()\n"
            "n=Node('rate_mover')\n"
            "pub=n.create_publisher(JointState,'/joint_states',10)\n"
            "N=['%s_joint_%d'%(a,j) for a in ('left','right') "
            "for j in range(1,8)]\n"
            "t0=time.monotonic()\n"
            "while rclpy.ok():\n"
            "    t=time.monotonic()-t0\n"
            "    m=JointState()\n"
            "    m.header.stamp=n.get_clock().now().to_msg()\n"
            "    m.name=N\n"
            "    m.position=[0.9*math.sin(1.4*t+0.4*k) for k in range(14)]\n"
            "    pub.publish(m)\n"
            "    time.sleep(0.01)\n")
        self.p = subprocess.Popen([sys.executable, "-c", code],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)

    def stop(self):
        if self.p:
            self.p.terminate()
            try:
                self.p.wait(5)
            except Exception:                                  # noqa: BLE001
                self.p.kill()
            self.p = None


def start_rviz(w, h):
    env = dict(os.environ, DISPLAY=DISPLAY, QT_QPA_PLATFORM="xcb",
               LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe")
    cmd = ["rviz2"] + (["-d", RVIZ_CFG] if os.path.exists(RVIZ_CFG) else [])
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    time.sleep(14)
    return p


def capture(rate, seconds, w, h, out):
    t0 = time.monotonic()
    p = subprocess.Popen(
        [FFMPEG, "-y", "-loglevel", "error", "-f", "x11grab",
         "-video_size", "%dx%d" % (w, h), "-framerate", str(rate),
         "-i", "%s.0" % DISPLAY, "-t", str(seconds),
         "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
         out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    p.wait()
    return time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rates", default="12,24,30,60")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--size", default="800x500")
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    rates = [int(v) for v in a.rates.split(",")]

    ensure_xvfb(1920, 1080)
    rviz = start_rviz(w, h)
    mover = Mover()
    mover.start()
    time.sleep(3)

    rows = []
    try:
        for r in rates:
            out = os.path.join(SCRATCH, "rate_%d.mp4" % r)
            wall = capture(r, a.seconds, w, h, out)
            total, dist = distinct_frames(out, w, h)
            rows.append(dict(requested=r, wall_s=round(wall, 2),
                             stored=total, distinct=dist,
                             delivered_fps=round(dist / a.seconds, 2),
                             dup_pct=round(100.0 * (1 - dist / max(1, total)),
                                           1),
                             size_kb=round(os.path.getsize(out) / 1024.0)))
            print("  requested %2d fps -> stored %4d, distinct %4d, "
                  "delivered %5.2f fps, %4.1f%% duplicate, %5d kB"
                  % (r, total, dist, rows[-1]["delivered_fps"],
                     rows[-1]["dup_pct"], rows[-1]["size_kb"]))

        # NEGATIVE CONTROL: with the source stationary the delivered rate must
        # collapse. If it does not, the difference detector is reporting
        # encoder noise and every number above is meaningless.
        mover.stop()
        time.sleep(2.5)
        out = os.path.join(SCRATCH, "rate_static.mp4")
        capture(max(rates), a.seconds, w, h, out)
        tot_s, dist_s = distinct_frames(out, w, h)
        static_fps = dist_s / a.seconds
        print("\n  NEGATIVE CONTROL (publisher stopped): distinct %d, "
              "%.2f fps  -> %s" % (dist_s, static_fps,
                                   "valid" if static_fps < 1.0
                                   else "INVALID -- detector sees noise"))
    finally:
        mover.stop()
        rviz.terminate()

    best = max(rows, key=lambda r: r["delivered_fps"]) if rows else None
    res = dict(rows=rows, static_distinct=dist_s,
               static_fps=round(static_fps, 3),
               control_valid=bool(static_fps < 1.0),
               size=a.size, seconds=a.seconds)
    p = os.path.join(WS, "recordings/baselines/capture_rate.json")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    json.dump(res, open(p, "w"), indent=2)
    if best:
        print("\nCEILING: %.2f distinct fps at %s (requested %d)"
              % (best["delivered_fps"], a.size, best["requested"]))
    print("-> %s" % p)
    return 0 if (rows and static_fps < 1.0) else 1


if __name__ == "__main__":
    sys.exit(main())
