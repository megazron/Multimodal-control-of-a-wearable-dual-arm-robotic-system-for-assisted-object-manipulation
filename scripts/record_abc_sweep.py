#!/usr/bin/env python3
"""PART 5 -- the 7-angle sweep: three tasks x four modes, launched THROUGH THE GUI.

    python3 scripts/record_abc_sweep.py [--resume] [--only 04_shared_autonomy]

WHAT MAKES THIS DIFFERENT FROM EVERY EARLIER RECORDING. Job A found that no
clip in this repository had ever been recorded through a control mode: they
all came from a recorder that calls /compute_ik directly, so no follower, no
clutch, no anchor and no safety guard was in the path, and the mode axis of
the tree was empty because nothing could fill it. Here the motion is started
by pressing the GUI's own button -- `Gui.on_launch(spec)` on the real manifest
-- so a clip filed under a mode really did travel that mode's command path,
and the recording doubles as proof the GUI drives the system.

THE ISOLATION RULE IS ENFORCED HERE, NOT DOCUMENTED.
======================================================================
Measured: after a VR run, modes 01, 04 and 06 every one reported 0.0000 m of
travel, and killing `vr_pose_mapper` restored them to 0.2000 m immediately.
The mapper stays ENGAGED when its run ends and keeps publishing on
/master_arm_pose_<arm> -- the same topic mode 01 uses -- so it holds the arm
at its last command. That is the project's one-source-at-a-time rule at the
PROCESS level.

Left as a documented step, this would fail silently and expensively: every
mode after the first records a stationary arm, all 84 clips render, the
verifier passes them on colour, and the sweep looks complete. So before each
mode records anything:

  1. TEAR DOWN every upstream this mode does not need (procscan.kill_all,
     which cannot kill its own shell).
  2. START only the upstreams this mode does need, and wait for them.
  3. COUNT PUBLISHERS on the follower's input topic and REFUSE TO RECORD
     unless the count is exactly what this mode predicts. Not "check it is
     up" -- count it, and compare against a number written down in advance.

A wrong count aborts the mode with the number in the message. It is not a
warning: a warning here produces a directory full of plausible, worthless
clips.

Xvfb, NEVER WSLg's :0 -- x11grab there records BLACK, measured, because the
compositor never puts window contents in the X root window. Seven displays,
one per view, so the motion runs ONCE and all seven angles see the same run.
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
from srl_teleop import procscan                              # noqa: E402
from srl_teleop import gui_launch_specs as gls               # noqa: E402
import record_rviz as rr                                     # noqa: E402

OUT = os.path.join(WS, "recordings/verification")
PROGRESS = os.path.join(OUT, "abc_sweep_progress.json")

TASKS = ("a", "b", "c")
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
import clip_tasks as CT                                      # noqa: E402
SCENARIO = {k: v["scenario"] for k, v in CT.TASKS.items()}

# Per mode: which upstream nodes it needs, and how many publishers the
# FOLLOWER's input topic must have while it runs. The expected counts are the
# whole point -- they are predictions, written before the run, that the sweep
# refuses to proceed without.
# ORDER IS DELIBERATE: most self-sufficient first, so a session that dies
# part-way still leaves the modes that needed no operator on the remote.
MODE_ORDER = ["06_full_autonomy", "01_master_teleop", "03_shared_autonomy",
              "02_vr_teleop", "04_vr_shared"]

MODES = {
    "01_master_teleop": dict(
        needs=[], follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the runner itself is the only publisher"),
    "02_vr_teleop": dict(
        needs=[("vr_pose_mapper",
                ["ros2", "run", "srl_vr_teleop", "vr_pose_mapper"])],
        follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the MAPPER is the only publisher; the runner drives it "
             "upstream on /vr/controller_pose_*"),
    "04_shared_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself"),
    "06_full_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself, commanded by a spoken instruction"),
    "03_shared_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the arbiter's topic; the master is present but not commanding"),
    "04_vr_shared": dict(
        needs=[("vr_pose_mapper",
                ["ros2", "run", "srl_vr_teleop", "vr_pose_mapper"])],
        follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        vr_present=True,
        note="the VR transport is UP and autonomy owns the pose"),
}

# Every upstream any mode can start. Anything not in a mode's `needs` is torn
# down before that mode runs -- listed explicitly so a new upstream cannot be
# forgotten by omission.
ALL_UPSTREAMS = ["lib/srl_vr_teleop/vr_pose_mapper",
                 "lib/srl_autonomy/autonomy_executive",
                 "lib/srl_vr_teleop/quest_vendor_bridge"]


def burn_caption(front_mp4, lines, width=800):
    """Composite the caption ONTO the front clip, as pixels.

    RViz TEXT_VIEW_FACING markers were tried first and are the wrong tool: a
    marker is one 3-D object in the scene, so a caption line is a metres-wide
    billboard that mostly falls outside the frame. Two attempts produced a
    caption that was present, correct and unreadable -- words scattered across
    the picture. `drawtext` is not compiled into this ffmpeg build, so the
    caption is rendered to a PNG with exact control and composited with
    `overlay`, which is.

    Overlaid for the WHOLE clip rather than appearing at the end, so the
    viewer knows what to expect before the motion starts and can see the
    outcome without waiting for it.
    """
    from PIL import Image, ImageDraw, ImageFont
    if not os.path.exists(front_mp4):
        return False
    pad, lh = 9, 15
    h = pad * 2 + lh * len(lines)
    im = Image.new("RGBA", (width, h), (7, 11, 15, 232))
    d = ImageDraw.Draw(im)
    d.line([(0, h - 1), (width, h - 1)], fill=(63, 182, 201, 210))
    for i, (txt, col) in enumerate(lines):
        try:
            f = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf"
                % ("-Bold" if i == 0 else ""), 11 if i == 0 else 10)
        except OSError:
            f = ImageFont.load_default()
        d.text((10, pad + i * lh), txt, fill=col, font=f)
    png = front_mp4 + ".caption.png"
    im.save(png)
    tmp = front_mp4 + ".cap.mp4"
    r = subprocess.run(
        [rr.FFMPEG, "-y", "-loglevel", "error", "-i", front_mp4, "-i", png,
         "-filter_complex", "overlay=0:0", "-c:v", "libx264", "-preset",
         "ultrafast", "-pix_fmt", "yuv420p", tmp],
        capture_output=True, text=True)
    if r.returncode == 0 and os.path.exists(tmp) and os.path.getsize(tmp) > 10000:
        os.replace(tmp, front_mp4)
        os.remove(png)
        return True
    return False


W_TXT = (238, 244, 248, 255)
C_TXT = (99, 200, 216, 255)
A_TXT = (232, 163, 61, 255)
R_TXT = (255, 90, 105, 255)


class Caption:
    """The clip's own caption, published into the scene as HUD markers.

    BURNT IN, not written in a sidecar. A viewer must be able to tell from the
    picture alone whether the run did what it was supposed to; a caption that
    lives in a JSON file beside the clip is a caption nobody reads while
    watching. Only the FRONT view renders /task_hud, so the other six angles
    stay clean.

    RESULT is filled in AFTER the run and the last frames carry it, so the
    clip ends by stating its own outcome -- including a failure. A recording
    that cannot say it failed is a recording that always looks like a pass.
    """

    def __init__(self, node):
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self._MA = MarkerArray
        self.pub = node.create_publisher(
            MarkerArray, "/task_hud",
            QoSProfile(depth=4, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node = node
        self.lines = []

    WRAP = 58

    def set(self, lines):
        """Wrap to the frame BEFORE publishing.

        An RViz TEXT_VIEW_FACING marker is one 3-D object: a 150-character
        string at this camera distance spans several metres, so most of it
        falls outside the view and what lands looks like words scattered
        across the picture. Measured on the first attempt -- the caption was
        present, correct and completely unreadable. Wrapped to 58 characters
        it sits inside the frame.
        """
        import textwrap
        out = []
        for txt, col in lines:
            for chunk in textwrap.wrap(txt, self.WRAP) or [""]:
                out.append((chunk, col))
        self.lines = out
        self.publish()

    def publish(self):
        from visualization_msgs.msg import Marker
        ma = self._MA()
        d = Marker()
        d.action = Marker.DELETEALL
        ma.markers.append(d)
        for i, (txt, col) in enumerate(self.lines):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.ns = "hud"
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = 0.0
            m.pose.position.y = 0.30
            m.pose.position.z = 1.98 - 0.058 * i
            m.pose.orientation.w = 1.0
            m.scale.z = 0.050 if i == 0 else 0.042
            m.color.r, m.color.g, m.color.b, m.color.a = col
            m.text = txt
            ma.markers.append(m)
        self.pub.publish(ma)


WHITE = (0.93, 0.96, 0.98, 1.0)
CYAN = (0.25, 0.71, 0.79, 1.0)
AMBER = (0.91, 0.64, 0.24, 1.0)
RED = (1.0, 0.30, 0.37, 1.0)


def log(msg):
    print(msg, flush=True)


def load_progress():
    try:
        return json.load(open(PROGRESS))
    except Exception:                                         # noqa: BLE001
        return {}


def save_progress(p):
    os.makedirs(os.path.dirname(PROGRESS), exist_ok=True)
    tmp = PROGRESS + ".tmp"
    json.dump(p, open(tmp, "w"), indent=2)
    os.replace(tmp, PROGRESS)      # atomic: a killed sweep never truncates it


# ===========================================================================
#  ISOLATION
# ===========================================================================
class Graph:
    """A tiny ROS node used only to COUNT publishers. Nothing else."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.n = Node("abc_sweep_graph")
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        self._js = None
        # A PRIVATE EXECUTOR, not the global one. `rclpy.spin_once(node)` does
        # NOT build a throwaway executor -- it uses rclpy's GLOBAL executor,
        # which the GUI's Bus is already spinning on another thread, so the
        # first call raised "Executor is already spinning" and took the sweep
        # down with a core dump. Same collision that killed grasp_generator at
        # startup. Own the executor, add the node to it, spin only it.
        self._ex = SingleThreadedExecutor()
        self.n.create_subscription(JointState, "/joint_states",
                                   self._on_js, 10)
        self._ex.add_node(self.n)

    def _on_js(self, m):
        self._js = {n: p for n, p in zip(m.name, m.position)}

    def joints(self, timeout_s=2.0):
        """Latest arm joint vector, or None. Spun on a private executor.

        Spun on this class's PRIVATE executor -- see __init__.
        """
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self._ex.spin_once(timeout_sec=0.05)
            if self._js:
                return dict(self._js)
        return None

    def moved(self, ref, tol_rad=0.01):
        """Has any arm joint left `ref` by more than tol?"""
        cur = self.joints(0.4)
        if not cur or not ref:
            return False
        return any(abs(cur[k] - v) > tol_rad
                   for k, v in ref.items() if k in cur and "joint_" in k)

    def pubs(self, topic):
        """Publisher count, WITHOUT spinning.

        `count_publishers` reads the middleware's graph cache, which the RMW
        keeps current on its own thread -- it needs time, not a spin. Spinning
        here raised "Executor is already spinning" because the GUI's Bus owns
        the default executor on another thread, which is the same collision
        that killed grasp_generator at startup once.

        The maximum over several samples is deliberate: discovery is
        asynchronous, and a count taken before matching completes is a count
        of the question rather than the answer.
        """
        best = 0
        for _ in range(8):
            time.sleep(0.25)
            best = max(best, self.n.count_publishers(topic))
        return best

    def close(self):
        try:
            self._ex.remove_node(self.n)
        except Exception:                                     # noqa: BLE001
            pass
        try:
            self.n.destroy_node()
        except Exception:                                     # noqa: BLE001
            pass


def isolate(mode, graph, started):
    """Make `mode` the only source. Returns (ok, message).

    Tears down every upstream this mode does not need, starts the ones it
    does, then COUNTS publishers on the follower's input topic and compares
    against the number this mode predicted.
    """
    spec = MODES[mode]
    need_pats = {n[0] for n in spec["needs"]}

    killed = []
    for pat in ALL_UPSTREAMS:
        if any(p in pat for p in need_pats):
            continue
        t, _ = procscan.kill_all(pat)
        if t:
            killed.append("%s x%d" % (pat.rsplit("/", 1)[-1], len(t)))
            started.pop(pat, None)
    if killed:
        log("   torn down: %s" % ", ".join(killed))
        time.sleep(2.0)

    for name, argv in spec["needs"]:
        pat = "lib/srl_vr_teleop/%s" % name if "vr" in name else name
        if procscan.count(pat) == 0:
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            p = subprocess.Popen(argv, env=env, start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            started[pat] = p
            log("   started %s (pid %d)" % (name, p.pid))
            time.sleep(9.0)

    # THE COUNT. Before the runner starts, the follower's input topic should
    # carry ONLY this mode's upstream -- 0 for the modes whose upstream IS the
    # runner, 1 for VR where the mapper publishes it.
    expect_idle = 1 if mode == "02_vr_teleop" else 0
    bad = []
    for arm in ("left", "right"):
        topic = spec["follower_topic"] % arm
        n = graph.pubs(topic)
        if n != expect_idle:
            bad.append("%s has %d publisher(s), expected %d before the run"
                       % (topic, n, expect_idle))
    if bad:
        return False, "; ".join(bad)
    return True, ("isolated: %s idle publishers as predicted (%s)"
                  % (expect_idle, spec["note"]))


# ===========================================================================
#  THE SWEEP
# ===========================================================================
def build_gui():
    """The real GUI, offscreen. Clips are started by pressing ITS buttons."""
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    import rclpy
    import threading
    import srl_gui
    from PyQt5.QtWidgets import QApplication
    if not rclpy.ok():
        rclpy.init()
    bus = srl_gui.Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication.instance() or QApplication([sys.argv[0]])

    class A:
        no_rviz, single_rviz, self_test_exit = True, False, False
        embed_rviz, dual_rviz = False, False
    g = srl_gui.Gui(bus, A())
    return app, g


def run_one(app, gui, task, mode, out_dir, graph=None,
            start_grabs=None, motion_wait_s=45.0):
    """Press the GUI button for (task, mode) and wait for it to finish."""
    # THE WHOLE MODE NAME, not its first two characters. Cutting the key to
    # two characters is what let 03_shared_autonomy and the legacy alias
    # 04_shared_autonomy collide on one key; the specs are now keyed by the
    # full mode and this must match or every lookup returns None.
    key = "abc_%s_%s" % (task, mode)
    spec = next((s for s in gui.specs if s.key == key), None)
    if spec is None:
        return False, "no GUI spec %r" % key
    if not spec.enabled:
        return False, "GUI button disabled: %s" % spec.disabled_reason
    before = len(gui.jobs)
    ref = graph.joints(3.0) if graph is not None else None
    gui.on_launch(spec)
    app.processEvents()
    if len(gui.jobs) == before:
        return False, "the GUI refused to launch it (see its log)"
    label, proc = gui.jobs[-1]
    t0 = time.monotonic()

    # START THE CAPTURE WHEN THE ARM STARTS MOVING, not when the run starts.
    #
    # Measured on the clips this replaces: a 28.4 s clip in which the arm moved
    # only during seconds 16-26. The first sixteen seconds -- 56% of the
    # recording -- were node startup, DDS discovery, the EE lookup and the
    # approach to the task start, during which nothing on screen changed. That
    # dead lead-in is what made the clips look like slideshows: across the
    # whole file 82% of frame pairs were bit-identical and the delivered rate
    # averaged 1.3 fps, while DURING THE MOTION it was 5-9 fps. Neither a
    # higher capture rate nor a faster waypoint rate could fix that, because
    # the problem was recording a stationary picture for half the clip.
    #
    # /trial_state is NOT the gate: run_abc publishes state="running" before it
    # drives the arm anywhere, so it fires in the dead zone. The arm's own
    # joints are the only signal that means what is wanted here.
    #
    # The timeout fallback is deliberate: if motion is never detected the grab
    # starts anyway, so a mode that fails to move is still RECORDED and can be
    # seen to have failed. A gate that can discard evidence of a failure is
    # worse than a slow clip.
    grabs, gate = None, None
    if start_grabs is not None:
        while proc.poll() is None and time.monotonic() - t0 < motion_wait_s:
            app.processEvents()
            if graph is not None and graph.moved(ref):
                gate = "motion"
                break
            time.sleep(0.1)
        gate = gate or "timeout"
        grabs = start_grabs()

    while proc.poll() is None and time.monotonic() - t0 < 300:
        app.processEvents()
        time.sleep(0.25)
    rc = proc.poll()
    if rc is None:
        proc.kill()
        return False, "timed out after 300 s"
    # THE RUNNER EXITS NON-ZERO IF NO ARM MOVED. That is the whole reason the
    # clip can be trusted: a stationary arm is not recorded as a success.
    return rc == 0, ("run exited %s" % rc), grabs, gate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--only", default=None, help="one mode key")
    ap.add_argument("--tasks", default="abc")
    ap.add_argument("--settle-s", type=float, default=3.0)
    a = ap.parse_args()

    modes = [a.only] if a.only else list(MODE_ORDER)
    tasks = [t for t in TASKS if t in a.tasks]
    prog = load_progress() if a.resume else {}

    log("=" * 74)
    log("ABC SWEEP -- %d modes x %d tasks x %d angles"
        % (len(modes), len(tasks), len(rr.VIEWS) + 1))
    log("=" * 74)

    graph = Graph()
    # CLEAR ANY LATCHED HUD MARKERS FIRST. /task_hud is TRANSIENT_LOCAL, so a
    # previous run's caption markers survive that process and RViz picks them
    # up on connect -- the earlier marker-based caption attempt was still in
    # the scene, scattered across the picture, under the new overlay. A
    # MarkerArray must begin with DELETEALL, and so must a session.
    cap = Caption(graph.n)
    cap.set([])
    time.sleep(1.0)
    started = {}
    app, gui = build_gui()
    done = failed = skipped = 0
    try:
        for mode in modes:
            log("\n== MODE %s" % mode)
            ok, why = isolate(mode, graph, started)
            log("   %s" % why)
            if not ok:
                # NOT A WARNING. Recording here would fill a directory with
                # plausible clips of a stationary arm.
                log("   ABORTING THIS MODE -- isolation not satisfied.")
                failed += len(tasks)
                continue
            for task in tasks:
                scen = SCENARIO[task]
                out_dir = os.path.join(OUT, mode, task.upper(), scen)
                pkey = "%s/%s/%s" % (mode, task, scen)
                if a.resume and prog.get(pkey, {}).get("ok"):
                    log("   %-28s SKIP (already recorded)" % pkey)
                    skipped += 1
                    continue
                os.makedirs(out_dir, exist_ok=True)
                log("   %-28s recording..." % pkey)
                spec = CT.TASKS[task]
                # THE SCENE PUBLISHER, one per task, torn down after. Without
                # it the clips show arms moving past nothing at all.
                procscan.kill_all("clip_scene.py")
                scene_p = subprocess.Popen(
                    [sys.executable, os.path.join(WS, "scripts",
                                                  "clip_scene.py"),
                     "--task", task,
                     "--out", os.path.join(out_dir, "scene_events.json")],
                    start_new_session=True, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
                time.sleep(3.0)
                grip_arm = "left" if task in ("a", "c") else "right"
                rr.ensure_display(os.path.join(out_dir, "rviz"),
                                  gripper_arm=grip_arm)
                time.sleep(a.settle_s)
                good, msg, grabs, gate = run_one(
                    app, gui, task, mode, out_dir, graph=graph,
                    start_grabs=lambda: rr.start_grabs(out_dir))
                log("      capture gated on %s" % gate)
                time.sleep(1.0)
                rr.stop_grabs(grabs)
                try:
                    os.killpg(os.getpgid(scene_p.pid), 15)
                except Exception:                             # noqa: BLE001
                    pass
                # CAPTION ON THE FRONT VIEW ONLY -- the other six stay clean,
                # and the quad is built AFTER so the tile carries it too.
                import textwrap as _tw
                cap_lines = [("%s  |  TASK %s: %s  |  %s"
                              % (mode.replace("_", " ").upper(), task.upper(),
                                 spec["name"].upper(), scen), W_TXT)]
                for ln in _tw.wrap("EXPECTED: " + spec["expect"], 104):
                    cap_lines.append((ln, C_TXT))
                if spec["caveat"]:
                    for ln in _tw.wrap("CAVEAT: " + spec["caveat"], 104):
                        cap_lines.append((ln, A_TXT))
                for ln in _tw.wrap("RESULT: %s -- %s"
                                   % ("AS EXPECTED" if good
                                      else "DID NOT COMPLETE", msg), 104):
                    cap_lines.append((ln, C_TXT if good else R_TXT))
                burn_caption(os.path.join(out_dir, "rviz_front.mp4"),
                             cap_lines)
                quad = rr.make_quad(out_dir)
                files = sorted(f for f in os.listdir(out_dir)
                               if f.endswith(".mp4"))
                prog[pkey] = dict(ok=bool(good), msg=msg, mode=mode,
                                  task=task, scenario=scen, quad=bool(quad),
                                  files=files, dir=os.path.relpath(out_dir, WS))
                save_progress(prog)          # AFTER EVERY CLIP
                log("      %s  %s  (%d files%s)"
                    % ("OK  " if good else "FAIL", msg, len(files),
                       ", quad" if quad else ""))
                done += bool(good)
                failed += (not good)
    finally:
        for pat, p in started.items():
            try:
                procscan.kill_all(pat)
            except Exception:                                 # noqa: BLE001
                pass
        graph.close()

    log("\n" + "=" * 74)
    log("%d recorded, %d failed, %d skipped   -> %s"
        % (done, failed, skipped, PROGRESS))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
