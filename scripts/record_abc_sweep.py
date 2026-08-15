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
import math
import os
import signal
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
from srl_teleop import procscan                              # noqa: E402
from srl_teleop import gui_launch_specs as gls               # noqa: E402
import record_rviz as rr                                     # noqa: E402

# WHERE CLIPS LAND. Overridable with SRL_CLIP_OUT so a re-record can go to a
# NEW directory instead of overwriting the set already on disk. The frozen
# archive under archive/recordings/ is never a target either way -- it is a
# snapshot, and a snapshot you can write to is not one.
OUT = os.environ.get("SRL_CLIP_OUT") or os.path.join(WS,
                                                     "recordings/verification")
PROGRESS = os.path.join(OUT, "abc_sweep_progress.json")

TASKS = ("a", "b", "c")

# THE SECOND TRAVEL GATE, in metres, measured by the SCENE node rather than by
# the runner. 0.05 is chosen from a good clip and a bad one, not from taste:
# a healthy 01/T1 clip measured 3.0205 m (left) and 0.6010 m (right) on
# 2026-08-15, and the failure this catches reports 0.0000 on both. There is
# nothing between them to be careful about, so the number is set two orders of
# magnitude below the good case and well above tf2 jitter.
MIN_SCENE_TRAVEL_M = 0.05
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as MCT
import choreography as CH                                  # noqa: E402

# TWO CURRENT TASK SETS, selected by --taskset. Not two sweeps: everything
# below -- isolation, preconditions, the foreign-description check, the
# motion gate, the caption burner, the progress file -- is identical, and a
# fork would have to be fixed twice.
#
# The GUI key prefix differs because the dispatcher keys do: the MSc four are
# m0-m3, NOT t0-t3, because run_experiment.sh refuses t1..t9 by name as the
# archived 300/310 mm set.
TASKSETS = {
    "abc": dict(mod=CT, keys=("a", "b", "c"), prefix="abc",
                arg=lambda k: k),
    # EXPLICIT MAP, not `"m" + k[1]`. That expression sent t1s2 -> "m1",
    # which is stage ONE: a stage 2 sweep would have recorded stage 1 under
    # stage 2's name. The same slip was already fixed once in run_abc.py.
    # THE DEMONSTRATION ROUTINES. Same sweep, same eight angles, same
    # verifier -- deliberately, so a demo clip cannot be produced by a path
    # that nobody checks. The key IS the dispatcher key here, so there is no
    # map to get wrong.
    "demo": dict(mod=CH, keys=("d1", "d2", "d3"), prefix="demo",
                 arg=lambda k: k),
    "msc": dict(mod=MCT, keys=("t0", "t1", "t1s2", "t2", "t3"), prefix="msc",
                arg=lambda k: {"t0": "m0", "t1": "m1", "t1s2": "m1s2",
                               "t2": "m2", "t3": "m3"}[k]),
}
SCENARIO = {k: v["scenario"] for k, v in CT.TASKS.items()}
SCENARIO.update({k: v["scenario"] for k, v in MCT.TASKS.items()})
SCENARIO.update({k: v["scenario"] for k, v in CH.TASKS.items()})

# Per mode: which upstream nodes it needs, and how many publishers the
# FOLLOWER's input topic must have while it runs. The expected counts are the
# whole point -- they are predictions, written before the run, that the sweep
# refuses to proceed without.
# ORDER IS DELIBERATE: most self-sufficient first, so a session that dies
# part-way still leaves the modes that needed no operator on the remote.
# MOVED to scripts/mode_upstreams.py so the CLIP path and the DATA path share
# ONE definition of what each mode needs. They did not, and 02_vr_teleop's
# data run recorded 0.0000 m of travel because run_abc had no equivalent of
# isolate() and nobody started vr_pose_mapper.
from mode_upstreams import (MODE_ORDER, MODES, ALL_UPSTREAMS,   # noqa: E402
                            isolate)

# The plain-words card text. Written per task rather than generated from the
# task spec's prose, because that prose is long, uses dashes and reads like
# documentation. A card is read in about six seconds by someone who has not
# seen the clip before, so it gets short sentences and nothing decorative.
CARD_TEXT = {
    "t0": ("Reach three targets with one arm while the other stays still.",
           "Watch the arm settle on each target before it moves to the next."),
    "t1": ("Pick up four cubes and place each one on the mat of its colour.",
           "Watch the fingers close on the cube, not above it. Blue goes to "
           "the blue mat and green to the green mat."),
    "t1s2": ("Both arms pick and place at the same time.",
             "Watch both arms move together. Neither reaches into the "
             "other's half of the table."),
    "t2": ("Both arms lift one tray together.",
           "Watch the tray stay level. If one hand leads, the ball rolls."),
    # WHAT THE PATH ACTUALLY DOES. The old card said "touch it to each test
    # point in turn", which the clip never shows: the right arm holds the box
    # up and the left arm presents the meter beside it, and the hold is where
    # the measurement lives. A card describing a different task is worse than
    # no card, because a viewer trusts it over the picture.
    "t3": ("One arm holds the circuit box up. The other brings the meter to "
           "it and both hold still while a reading is taken.",
           "Watch the four test points on the box. Two of them face away "
           "from the meter, so the box has to be turned to reach them."),
    "d1": ("A slow routine. The arms take turns, one holding while the other "
           "moves.",
           "Watch the pause at the top of each reach."),
    "d2": ("A routine on a beat. The arms move in opposite directions.",
           "Watch one arm rise as the other drops, then both stop together."),
    "d3": ("Call and answer. One arm gestures and the other replies.",
           "Watch the arm wind up slightly before it moves, and settle after "
           "it arrives."),
}
MODE_TEXT = {
    "01_master_teleop": "Driven by hand from the master arm.",
    "02_vr_teleop": "Driven by hand from the VR controllers.",
    "03_shared_autonomy": "Driven by hand. The robot sets the wrist angle.",
    "04_vr_shared": "Driven from VR. The robot sets the wrist angle.",
    "06_full_autonomy": "The robot runs the task on a spoken instruction.",
}


def scene_travel_verdict(ev, floor_m=MIN_SCENE_TRAVEL_M):
    """Did the SCENE NODE see an arm move? -> (True | False | None, why).

    Pulled out of the sweep body so it can be given a deliberately broken
    input by a test. The project's rule is that a check which cannot fail on
    a broken input is not a check, and a gate buried inside a 300-line loop
    that needs a stack, an Xvfb and four minutes to reach cannot be given one.

    None means NOT ANSWERED, and it is deliberately not False. A clip
    recorded before `ee_travel_m` existed says nothing about motion either
    way, and failing it would be scoring the recorder's age.
    """
    tr = ev.get("ee_travel_m")
    if not isinstance(tr, dict) or not tr:
        return None, ("scene travel NOT REPORTED by this clip -- the runner's "
                      "own travel gate is the only witness")
    vals = {k: v for k, v in tr.items() if isinstance(v, (int, float))}
    if not vals:
        return None, ("scene travel reported %r, which carries no number"
                      % (tr,))
    shown = ", ".join("%s %.4f" % (k, v) for k, v in sorted(vals.items()))
    # MAX OVER ARMS, not all of them: T1 and T3 work one arm at a time and
    # the idle arm is SUPPOSED to be still.
    if max(vals.values()) < floor_m:
        return False, ("THE SCENE SAW A STATIONARY ARM: ee_travel %s, all "
                       "under %.2f m" % (shown, floor_m))
    return True, "scene travel %s" % shown


def prepend_card(mp4, mode, task, hold_s=6.0):
    """Put a full-frame information card in FRONT of the footage.

    It used to be an overlay across the whole clip, which competes with the
    thing it describes: the words sit on top of the arms for the entire run
    and a viewer reads them while trying to watch the motion. A card that
    plays first is read once and then gets out of the way.

    Six seconds because that is roughly how long four short lines take to
    read for someone who has not seen the clip before.
    """
    from PIL import Image, ImageDraw, ImageFont
    if not os.path.exists(mp4):
        return False
    W, H = 800, 500
    what, watch = CARD_TEXT.get(task, ("", ""))
    im = Image.new("RGB", (W, H), (9, 13, 18))
    d = ImageDraw.Draw(im)

    def font(sz, bold=False):
        try:
            return ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf"
                % ("-Bold" if bold else ""), sz)
        except OSError:
            return ImageFont.load_default()

    import textwrap as _t
    y = 70
    d.text((56, y), task.upper(), fill=(238, 244, 248), font=font(34, True))
    y += 52
    d.text((56, y), MODE_TEXT.get(mode, mode), fill=(99, 200, 216),
           font=font(17))
    y += 46
    d.line([(56, y), (W - 56, y)], fill=(60, 80, 96))
    y += 30
    for ln in _t.wrap(what, 58):
        d.text((56, y), ln, fill=(238, 244, 248), font=font(18))
        y += 27
    y += 18
    d.text((56, y), "What to watch for", fill=(232, 163, 61), font=font(15,
                                                                       True))
    y += 26
    for ln in _t.wrap(watch, 62):
        d.text((56, y), ln, fill=(206, 216, 224), font=font(16))
        y += 24
    png = mp4 + ".card.png"
    im.save(png)
    tmp = mp4 + ".carded.mp4"
    r = subprocess.run(
        [rr.FFMPEG, "-y", "-loglevel", "error",
         "-loop", "1", "-t", "%.1f" % hold_s, "-i", png, "-i", mp4,
         "-filter_complex",
         "[0:v]scale=%d:%d,fps=12,format=yuv420p[c];"
         "[1:v]scale=%d:%d,fps=12,format=yuv420p[v];[c][v]concat=n=2:v=1[o]"
         % (W, H, W, H),
         "-map", "[o]", "-c:v", "libx264", "-preset", "ultrafast",
         "-pix_fmt", "yuv420p", tmp],
        capture_output=True, text=True)
    if r.returncode == 0 and os.path.exists(tmp) \
            and os.path.getsize(tmp) > 10000:
        os.replace(tmp, mp4)
        os.remove(png)
        return True
    return False


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

    def pub_nodes(self, topic):
        """WHICH nodes publish `topic`, not how many.

        A COUNT RACES AND A NAME DOES NOT. `master_pose_node` carries
        respawn=True and, with no Teensy attached, dies on PortNotFound and
        comes back every ~5 s -- so a raw count of /master_arm_pose_left is 0
        or 1 depending on where in that cycle the check lands. Mode
        01_master_teleop passed its "expect 0 idle publishers" check only
        because the node happened to be down; mode 02_vr_teleop failed with
        "2 publishers, expected 1" for no reason but timing. The predicate was
        measuring the respawn phase, not the isolation.

        The question the sweep actually needs answered is "is any OTHER source
        driving this topic", which is about identity, so ask by identity.
        """
        out = set()
        for _ in range(6):
            for info in self.n.get_publishers_info_by_topic(topic):
                out.add(info.node_name)
            time.sleep(0.35)
        return out

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


def foreign_description():
    """PIDs of robot_state_publishers that are NOT part of a launched stack.

    A STRAY ONE POISONS EVERY CLIP IN THE SWEEP, SILENTLY. Measured 2026-08-10:
    a robot_state_publisher left over from CAD-figure work owned
    /robot_description; that URDF carries no ros2_control tag, so the stack's
    ros2_control_node threw "no 'ros2_control' tag found in the URDF" and died
    at startup. With no controller manager, /joint_states fell back to a
    publisher emitting all zeros -- and EVERY ROBOT JOINT READ 0.000, which is
    a plausible number in a plausible place. Nothing downstream disagreed.

    Run the sweep in that state and it produces fifteen clips of an arm at a
    posture it was never commanded to, all of which render, all of which pass
    a colour check. This is a PRECONDITION, checked once before the sweep and
    again before every mode, not a step to remember.

    A launched publisher is handed the description through --params-file; a
    hand-started one is handed a URDF path. That difference needs no
    bookkeeping to stay true.
    """
    out = []
    for pid, cmd in procscan.find("robot_state_publisher"):
        if "--params-file" in cmd:
            continue
        if ".urdf" in cmd or ".xacro" in cmd:
            out.append(pid)
    return out


def preconditions():
    """Everything that must be true before a single frame is worth recording.

    Returns (ok, [messages]). Each check is one this project has been bitten
    by, and each names its own recovery.
    """
    msgs = []
    stray = foreign_description()
    if stray:
        msgs.append(
            "a robot_state_publisher outside the stack owns "
            "/robot_description (pid %s) -- ros2_control_node will die with "
            "\"no 'ros2_control' tag\" and every joint will read 0.000. "
            "kill %s"
            % (", ".join(str(x) for x in stray),
               " ".join(str(x) for x in stray)))
    n_master = procscan.count("lib/srl_teleop/master_pose_node")
    if n_master > 1:
        msgs.append("%d master_pose_node instances -- two readers split the "
                    "serial stream. Kill all but one." % n_master)
    n_mg = procscan.count("lib/moveit_ros_move_group/move_group")
    if n_mg == 0:
        msgs.append("no move_group -- nothing will answer /compute_ik")
    elif n_mg > 1:
        msgs.append("%d move_group instances -- two planning scenes" % n_mg)

    # STALE DISPLAY STATE. Xvfb refuses a display whose /tmp/.X<n>-lock
    # exists, so a sweep started on top of dead servers renders BLACK on
    # displays that were never created -- and the verifier's own blind spot
    # then lets those clips through. Measured after one failed mode: 8
    # orphaned servers and 8 stale locks, and the leak compounds at 8 per
    # mode. Refuse, and NAME them, rather than quietly reusing what is there.
    for what, detail in rr.stale_displays():
        msgs.append("%s: %s -- kill by PID and remove the lock" % (what, detail))

    return (not msgs), msgs


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


def _fail(why):
    """Every exit from run_one must have the SAME ARITY as the success path.

    The error paths returned a 2-tuple while the caller unpacks 5, so the
    FIRST real failure -- a missing GUI spec -- surfaced as
    `ValueError: not enough values to unpack (expected 5, got 2)` with the
    actual reason nowhere on screen. A failure path that cannot report is
    worse than no failure path.
    """
    return False, why, None, None, None


def run_one(app, gui, task, mode, out_dir, graph=None,
            start_grabs=None, motion_wait_s=45.0, prefix="abc", gui_key=None):
    """Press the GUI button for (task, mode) and wait for it to finish."""
    # THE WHOLE MODE NAME, not its first two characters. Cutting the key to
    # two characters is what let 03_shared_autonomy and the legacy alias
    # 04_shared_autonomy collide on one key; the specs are now keyed by the
    # full mode and this must match or every lookup returns None.
    # THE GUI KEY USES THE DISPATCHER KEY, NOT THE TASKSET KEY. For the MSc
    # set those differ -- msc_clip_tasks calls them t0-t3, the dispatcher and
    # therefore the buttons call them m0-m3, because t1..t9 are refused by
    # name as the archived set. Building the key from the taskset key gave
    # `msc_t0_...`, which matches no button.
    key = "%s_%s_%s" % (prefix, gui_key or task, mode)
    spec = next((s for s in gui.specs if s.key == key), None)
    if spec is None:
        return _fail("no GUI spec %r" % key)
    if not spec.enabled:
        return _fail("GUI button disabled: %s" % spec.disabled_reason)
    before = len(gui.jobs)
    ref = graph.joints(3.0) if graph is not None else None
    gui.on_launch(spec)
    app.processEvents()
    if len(gui.jobs) == before:
        return _fail("the GUI refused to launch it (see its log)")
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
        grab_t0 = time.time()
        grabs = start_grabs()

    while proc.poll() is None and time.monotonic() - t0 < 300:
        app.processEvents()
        time.sleep(0.25)
    rc = proc.poll()
    if rc is None:
        proc.kill()
        return _fail("timed out after 300 s")
    # THE RUNNER EXITS NON-ZERO IF NO ARM MOVED. That is the whole reason the
    # clip can be trusted: a stationary arm is not recorded as a success.
    return rc == 0, ("run exited %s" % rc), grabs, gate, grab_t0


def _install_teardown():
    """Teardown on EVERY exit, including a signal.

    A `finally:` covers exceptions and returns but not SIGTERM/SIGINT, and the
    way this sweep actually ends when a session runs out of time is a signal.
    A teardown only ever exercised on the happy path is not a teardown.
    """
    def _bye(signum, _frame):
        print("\n[sweep] signal %d -- tearing displays down before exit"
              % signum)
        try:
            rr.teardown_displays()
        finally:
            os._exit(130)
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, _bye)
        except (ValueError, OSError):
            pass


def main():
    """Wrapper whose ONLY job is that teardown cannot be skipped.

    `finally` covers the exception and early-return paths; _install_teardown()
    covers signals. Between them there is no exit from a sweep that leaves an
    Xvfb alive -- which is what the previous version did on every failure, and
    every run so far HAS failed.
    """
    _install_teardown()
    try:
        return _main_body()
    finally:
        rr.teardown_displays()


def _main_body():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--only", default=None, help="one mode key")
    ap.add_argument("--taskset", default="msc", choices=sorted(TASKSETS))
    # THE LAYOUT SEED FOR RANDOMISED TASKS (t1s2). It goes to the scene node
    # on the command line and to the task through SRL_TASK_SEED, because the
    # task is launched by a fixed GUI button that cannot carry an argument.
    # Both sides read the same number or the picture and the path disagree.
    ap.add_argument("--seed", type=int, default=0,
                    help="layout seed for randomised tasks; recorded per clip")
    ap.add_argument("--tasks", default=None,
                    help="subset of the taskset's keys; default all of them")
    ap.add_argument("--settle-s", type=float, default=3.0)
    ap.add_argument("--no-verify", action="store_true",
                    help="record only; skip the verifiers")
    a = ap.parse_args()

    modes = [a.only] if a.only else list(MODE_ORDER)
    # THE SEED, INTO THE ENVIRONMENT, BEFORE ANY CHILD IS LAUNCHED. The task
    # runs behind a GUI button whose command line is fixed, so the only way
    # to give it a per-run seed is to put it where the child inherits it.
    seed = a.seed
    os.environ["SRL_TASK_SEED"] = str(seed)
    ts = TASKSETS[a.taskset]
    tasks = [k for k in ts["keys"]
             if a.tasks is None or k in a.tasks.split(",")]
    if not tasks:
        log("REFUSING: --tasks %r selected none of %s. A sweep over zero "
            "tasks would report a clean run having recorded nothing."
            % (a.tasks, list(ts["keys"])))
        return 2

    # SAY WHAT IS ABOUT TO BE FILMED, AND REFUSE IF IT IS NOT THIS TASKSET'S.
    #
    # SCENARIO is a FLAT dict merged from every task table, so a key present
    # in two of them would resolve to whichever was merged last and the sweep
    # would film one taskset's scenario under another's name. Nothing
    # downstream could tell: the directories, the captions and the counts all
    # look exactly right, and it surfaces only when a person watches the
    # footage and sees the wrong task.
    #
    # There is no collision today. This refuses the day somebody adds one.
    own = ts["mod"].TASKS
    wrong = [k for k in tasks
             if k not in own or SCENARIO.get(k) != own[k]["scenario"]]
    log("   taskset %r -> %d task(s): %s"
        % (a.taskset, len(tasks),
           ", ".join("%s/%s" % (k, SCENARIO.get(k)) for k in tasks)))
    if wrong:
        log("REFUSING: %s did not resolve to the %r taskset's own scenarios. "
            "A sweep that films the wrong tasks looks entirely normal until "
            "somebody watches it." % (", ".join(wrong), a.taskset))
        return 2
    # THE LEDGER IS ALWAYS LOADED. --resume DECIDES WHAT TO SKIP, NOT WHAT TO
    # REMEMBER, AND CONFLATING THE TWO DESTROYS EVIDENCE.
    #
    # This was `load_progress() if a.resume else {}`, so a run over a SUBSET
    # -- `--only <mode> --tasks t0`, which is what a re-record of one bad clip
    # is -- started from an empty dict and `save_progress()` then wrote that
    # empty dict plus the one new cell over the whole file. MEASURED on
    # 2026-08-15: mode 01 recorded 5 of 5, one clip was re-recorded to fix its
    # opening pose, and the ledger afterwards held ONE entry. The other four
    # clips were still on disk and had been erased from the only record of
    # what they are -- including `opened_on`, which cannot be recovered by
    # looking at the directory.
    #
    # Nothing downstream could have caught it: status_table.py reads this
    # file, so the set would simply have reported itself as one clip.
    prog = load_progress()

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
    ok, why = preconditions()
    for m in why:
        log("   PRECONDITION FAILED: %s" % m)
    if not ok:
        log("REFUSING TO RECORD. A sweep run in this state produces clips "
            "that render, pass the verifier and show the wrong thing.")
        return 2
    log("   preconditions: one move_group, no stray /robot_description owner")

    started = {}
    app, gui = build_gui()
    done = failed = skipped = 0
    try:
        for mode in modes:
            log("\n== MODE %s" % mode)
            # RE-CHECKED PER MODE. A stray can appear mid-sweep -- the one
            # that caused this check was started by a sibling session while
            # other work was running.
            pok, pwhy = preconditions()
            if not pok:
                for m in pwhy:
                    log("   PRECONDITION FAILED: %s" % m)
                log("   ABORTING THIS MODE.")
                failed += len(tasks)
                continue
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
                spec = ts["mod"].TASKS[task]
                # THE SCENE PUBLISHER, one per task, torn down after. Without
                # it the clips show arms moving past nothing at all.
                procscan.kill_all("clip_scene.py")
                scene_p = subprocess.Popen(
                    [sys.executable, os.path.join(WS, "scripts",
                                                  "clip_scene.py"),
                     "--task", task,
                     # ONE SEED FOR THE PICTURE AND THE PATH. t1s2's layout
                     # is a random draw; if the scene node draws its own the
                     # two disagree and every measured distance is between
                     # objects from different layouts.
                     "--seed", str(seed),
                     "--out", os.path.join(out_dir, "scene_events.json")],
                    start_new_session=True, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
                time.sleep(3.0)
                # WHICH ARM THE GRIPPER VIEW FOLLOWS. Taken from the task's
                # own grip schedule rather than a hardcoded list, so a new
                # task cannot silently get the wrong camera.
                _g = ts["mod"].TASKS[task]["grip"](8)
                grip_arm = ("right"
                            if len(set(_g["right"])) > len(set(_g["left"]))
                            else "left")
                # THE FRONT CAMERA FOLLOWS THE TASK. Four tasks that occupy
                # four different volumes cannot share one framing: measured on
                # the 2026-08-11 clips, T0's two highest targets and T2's tray
                # were both ABOVE the frame and T1's whole workspace was about
                # sixty pixels wide. See record_rviz.TASK_FOCUS.
                rr.ensure_display(os.path.join(out_dir, "rviz"),
                                  gripper_arm=grip_arm, task=task)
                # EVERY CLIP OPENS ON THE PRESENTATION POSE. A joint-space
                # staging move, commanded before capture and waited on --
                # not teleoperation, not part of the task, and it produces no
                # trial data. Without it every clip opens on the home wrist
                # pointing up by +85 / +79 degrees, which is real and correct
                # and reads, to anyone who has not been told, as a fault.
                #
                # IT CANNOT COME FROM THE TASK PATH: the clip runner
                # publishes EE positions and the follower pins orientation,
                # so no commanded position changes where the hand points.
                #
                # A FAILURE HERE DOES NOT STOP THE RECORDING. The pose is a
                # framing improvement; the home pose is a known, documented
                # picture, and losing the whole clip over the opening frame
                # would be the worse trade. It is logged either way, so a
                # clip that opened on home is identifiable afterwards.
                # THE VR MAPPER HOLDS THE ARMS, SO IT IS STOPPED FOR THE
                # STAGING MOVE AND STARTED AGAIN BEFORE THE RUN.
                #
                # ISOLATED BY EXPERIMENT on 2026-08-15, one variable, on one
                # stack, with a control either side:
                #
                #   vr_pose_mapper absent   --home ARRIVED, 0.0000 / 0.0000 rad
                #   vr_pose_mapper RUNNING  DID NOT ARRIVE, 2.076 / 1.924 rad
                #   vr_pose_mapper killed   ARRIVED, 0.0000 / 0.0000 rad
                #
                # and in the sweep itself it cost SEVEN clips their opening
                # pose: all five of 04_vr_shared and two of 02_vr_teleop, the
                # only two modes that run the mapper. Modes 01, 03 and 06
                # staged first time on the same stack.
                #
                # The arm does not move AT ALL while the mapper is up -- the
                # reported joint error is identical before and after the whole
                # deadline, and identical again on the next clip -- so this is
                # not a move that needs longer. It is the project's own
                # one-source-at-a-time rule at the CONTROLLER level, with a
                # publisher the follower's own pause does not silence.
                #
                # isolate() is reused rather than reimplemented: it tears down
                # what this mode does not need, starts what it does, and
                # RE-COUNTS the publishers on the follower input, so the run
                # still begins from a verified graph.
                _vr = [n[0] for n in MODES[mode]["needs"] if "vr" in n[0]]
                for _n in _vr:
                    _pat = "lib/srl_vr_teleop/%s" % _n
                    procscan.kill_all(_pat)
                    started.pop(_pat, None)
                if _vr:
                    log("      stopped %s for the staging move"
                        % ", ".join(_vr))
                    # TEN SECONDS, NOT FOUR, AND THE NUMBER IS NOT A GUESS.
                    #
                    # isolate() already sleeps 8.0 s after tearing an upstream
                    # down, for a measured reason: the RMW's graph cache
                    # outlives the process, and a participant created into the
                    # gap sees a domain that is still being reaped. Four
                    # seconds was tried here first and the staging subprocess
                    # came back rc=2 -- "no /joint_states after 15.0 s" -- on
                    # a stack that was plainly up, which is the poisoned-
                    # participant failure sim_session.wait_ready() documents
                    # at length. It is a DIFFERENT failure from the one this
                    # block exists to fix, introduced by the fix.
                    time.sleep(10.0)

                # TWO ATTEMPTS. The retry is cheap and the operation is
                # idempotent -- it commands an absolute joint-space pose and
                # waits -- but it is NOT a blanket retry: both attempts are
                # logged and a second failure is still a failure. It was added
                # when the cause was thought to be a cold stack; that was
                # wrong, and it is kept only because a retry costs seconds.
                for _try in (1, 2):
                    _st = subprocess.run(
                        [sys.executable,
                         os.path.join(WS, "scripts",
                                      "stage_presentation_pose.py"),
                         # LONGER DISCOVERY WHEN AN UPSTREAM WAS JUST KILLED.
                         # The default 15 s is for a settled graph; this
                         # subprocess is deliberately created moments after a
                         # participant left, which is the slowest case there
                         # is on this host.
                         "--discover-s", "30" if _vr else "15"],
                        capture_output=True, text=True)
                    if _st.returncode == 0:
                        if _try == 2:
                            log("      presentation pose: OK on attempt 2 "
                                "(the first did not arrive in time)")
                        break
                    if _try == 1:
                        log("      presentation pose: attempt 1 rc=%d, "
                            "retrying once" % _st.returncode)
                staged = _st.returncode == 0
                if _vr:
                    _iok, _iwhy = isolate(mode, graph, started)
                    log("      restarted %s: %s" % (", ".join(_vr), _iwhy))
                    if not _iok:
                        # NOT A WARNING. Recording now would film this mode
                        # through a graph that failed its own publisher count,
                        # which is the state isolate() exists to refuse.
                        log("      SKIPPING THIS CLIP -- the graph did not "
                            "come back to a verified state after staging.")
                        # THE SCENE NODE WAS ALREADY STARTED. Skipping without
                        # it would leak one clip_scene per skipped cell, and
                        # two scene publishers is the same one-source-at-a-time
                        # fault this branch exists to respect.
                        try:
                            os.killpg(os.getpgid(scene_p.pid), 15)
                            scene_p.wait(timeout=15)
                        except Exception:                     # noqa: BLE001
                            try:
                                os.killpg(os.getpgid(scene_p.pid), 9)
                            except Exception:                 # noqa: BLE001
                                pass
                        prog[pkey] = dict(
                            ok=False,
                            msg="upstream restart after staging failed: %s"
                                % _iwhy,
                            mode=mode, task=task, scenario=scen, quad=False,
                            opened_on=("presentation" if staged else "home"),
                            files=[], dir=os.path.relpath(out_dir, WS))
                        save_progress(prog)
                        failed += 1
                        continue
                if staged and _try == 1:
                    log("      presentation pose: OK")
                elif not staged:
                    # THE WHOLE OUTPUT, NOT 120 CHARACTERS OF IT.
                    #
                    # The staging script prints the per-arm joint error and
                    # then the reason, in that order, and the truncation cut
                    # the message off inside the FOLLOWER-PAUSE line -- so a
                    # failure whose diagnosis was three lines further down
                    # reached the log as "paused (motion_enabled true ->" and
                    # nothing else. A failure path that cannot report is worse
                    # than no failure path, which is written eight functions
                    # up this file about a different truncation.
                    log("      presentation pose: NOT STAGED (rc=%d) -- this "
                        "clip opens on HOME" % _st.returncode)
                    for _ln in ((_st.stdout or "") + (_st.stderr or "")
                                ).strip().splitlines():
                        log("        | %s" % _ln)
                time.sleep(a.settle_s)
                good, msg, grabs, gate, grab_t0 = run_one(
                    app, gui, task, mode, out_dir, graph=graph,
                    start_grabs=lambda: rr.start_grabs(out_dir,
                                                      grip_arm),
                    prefix=ts["prefix"], gui_key=ts["arg"](task))
                log("      capture gated on %s" % gate)
                # SAY WHY, IMMEDIATELY. The reason used to be folded into the
                # message printed after teardown, so a teardown that raised
                # took the diagnosis with it.
                if not good:
                    log("      RUN FAILED: %s" % msg)
                time.sleep(1.0)
                rr.stop_grabs(grabs)
                grab_t1 = time.time()
                # SIGTERM, THEN WAIT FOR IT TO ACTUALLY WRITE.
                # clip_scene dumps scene_events.json from its SIGTERM handler,
                # and the first version read the file immediately after
                # sending the signal -- so every clip reported "the scene
                # never ran" while the scene had run perfectly and was still
                # writing. Signalling a process is not the same as it having
                # finished, and the failure looked exactly like the real
                # fault it was added to detect.
                try:
                    os.killpg(os.getpgid(scene_p.pid), 15)
                except Exception:                             # noqa: BLE001
                    pass
                try:
                    scene_p.wait(timeout=15)
                except Exception:                             # noqa: BLE001
                    try:
                        os.killpg(os.getpgid(scene_p.pid), 9)
                    except Exception:                         # noqa: BLE001
                        pass

                # DID THE GRASP HAPPEN INSIDE THE VIDEO?
                #
                # PER TASK, because the tasks differ: A closes at waypoint 5
                # of a short path, B is already holding at waypoint 1 after a
                # long approach, C grips from waypoint 0 and never lets go.
                # One global window cannot suit all three, and the failure is
                # silent -- B/S1 logged GRASPED at t=44.1 s inside a 29.1 s
                # clip, so the single most important instant in the clip
                # happened after the recording stopped, and every automatic
                # check still passed it.
                #
                # This compares WALL CLOCKS, which is why clip_scene now
                # stamps one on every event: the scene node's own t=0 and the
                # ffmpeg start had no common time base before.
                try:
                    ev = json.load(open(os.path.join(out_dir,
                                                     "scene_events.json")))

                    # DID THE ARM MOVE, ACCORDING TO THE SCENE NODE?
                    #
                    # THIS IS A SECOND, INDEPENDENT WITNESS, and that is the
                    # entire point of it. run_abc already refuses to exit 0
                    # unless one arm travelled >= --min-travel-m, and this
                    # sweep trusted that exit code alone. The two measure the
                    # same quantity in DIFFERENT PROCESSES off different tf2
                    # listeners, so the runner can see motion the scene node
                    # never received -- and on 2026-08-15 that is exactly what
                    # was filed: clips reporting TRAVEL L 0.00 R 0.00 with
                    # four cubes carried 0.000 m while the sweep recorded OK.
                    # A stationary arm is this project's oldest failure mode
                    # and one witness for it is not enough.
                    #
                    # MAX OVER ARMS, not all of them: T1 and T3 work one arm
                    # at a time and the idle arm is SUPPOSED to be still.
                    #
                    # ABSENT IS NOT ZERO. A file written before this field
                    # existed says nothing about motion, and scoring it 0.00
                    # would fail good clips for the recorder's age.
                    verdict, why = scene_travel_verdict(ev)
                    if verdict is False:
                        good = False
                        msg += "; " + why
                    else:
                        log("      %s" % why)

                    gr = [e for e in ev.get("events", [])
                          if e.get("ev") == "GRASPED" and e.get("wall")]
                    if gr:
                        off = gr[0]["wall"] - grab_t0
                        span = grab_t1 - grab_t0
                        if not (0.0 <= off <= span):
                            good = False
                            msg += ("; GRASP OUTSIDE THE CLIP: at %+.1f s of "
                                    "a %.1f s window" % (off, span))
                        else:
                            log("      grasp at %+.1f s of %.1f s"
                                % (off, span))
                    elif ts["mod"].TASKS[task].get("width_mm"):
                        good = False
                        msg += "; NO GRASP RECORDED at all"

                    # AND WAS IT PUT WHERE IT WAS MEANT TO GO?
                    #
                    # A grasp inside the window is not a completed task. Under
                    # VR teleop the arm lagged the waypoint stream, the
                    # gripper opened on schedule anyway, and the block was
                    # dropped 0.15 m short and 0.15 m above the bin -- a clip
                    # of a failed place that passed every check, because
                    # nothing compared the release point to the target.
                    tgt = ts["mod"].TASKS[task].get("place_target")
                    itm = (ev.get("items") or [None])[0]
                    po = ev.get("pad_off") or [0.0, 0.0, 0.0]
                    if tgt and itm:
                        # Same wrist->pad shift the scene applied, so the
                        # comparison is like for like.
                        tgt = [tgt[i] + po[i] for i in range(3)]
                        d = math.dist(itm["final"], tgt)
                        if d > 0.12:
                            good = False
                            msg += ("; PLACED %.3f m FROM TARGET" % d)
                        else:
                            log("      placed %.0f mm from target" % (d * 1000))
                except FileNotFoundError:
                    # A TASK WITH NO OBJECTS HAS NO SCENE EVENTS, and that is
                    # correct rather than a failure. T0 is reaching only --
                    # its _items() is {} by design, so clip_scene writes no
                    # events file and never can. Requiring one would make T0
                    # unpassable for the very property that lets it run in
                    # every mode. Every task that DOES declare objects must
                    # still produce the file.
                    if ts["mod"].TASKS[task].get("grip_obj") is None:
                        log("      no objects in this task -- no scene events "
                            "expected")
                    else:
                        good = False
                        msg += "; no scene_events.json -- the scene never ran"

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
                # A CARD IN FRONT OF EVERY ANGLE, not an overlay across one.
                for _f in sorted(os.listdir(out_dir)):
                    if _f.startswith("rviz_") and _f.endswith(".mp4"):
                        prepend_card(os.path.join(out_dir, _f), mode, task)
                quad = rr.make_quad(out_dir)
                files = sorted(f for f in os.listdir(out_dir)
                               if f.endswith(".mp4"))
                prog[pkey] = dict(ok=bool(good), msg=msg, mode=mode,
                                  task=task, scenario=scen, quad=bool(quad),
                                  # WHICH POSE THE CLIP OPENED ON, recorded
                                  # per clip. A set where some clips were
                                  # staged and some were not is otherwise
                                  # indistinguishable from one where the
                                  # staging never ran.
                                  opened_on=("presentation" if staged
                                             else "home"),
                                  files=files, dir=os.path.relpath(out_dir, WS))
                save_progress(prog)          # AFTER EVERY CLIP
                log("      %s  %s  (%d files%s)"
                    % ("OK  " if good else "FAIL", msg, len(files),
                       ", quad" if quad else ""))
                done += bool(good)
                failed += (not good)
            # TEAR THE DISPLAYS DOWN AT THE END OF EACH MODE.
            #
            # WITHOUT THIS, A SWEEP CAN ONLY EVER RECORD ONE MODE. The eight
            # Xvfb servers are created per clip and reused, and until now they
            # were only torn down when the PROCESS exited -- while
            # preconditions() is re-checked before every mode and refuses on a
            # live Xvfb or a stale lock. MEASURED on 2026-08-15: a run over
            # five modes recorded the first one, then aborted the other four
            # with sixteen PRECONDITION FAILED lines each, naming displays its
            # own previous mode had left running. It reported "5 recorded, 20
            # failed" for a machine that was working perfectly.
            #
            # The refusal is RIGHT -- a sweep that starts on top of dead
            # servers renders black and passes -- so the fix belongs here, at
            # the point where the displays are genuinely finished with, not in
            # the check.
            #
            # THIS DOES NOT BREAK ensure_display's "nothing is ever killed"
            # rule. That rule is about the MIDDLE of a sweep, where killing an
            # Xvfb two RViz instances share produced seven black clips. Here
            # every capture for this mode has stopped, which is the same state
            # the end-of-process teardown runs in.
            try:
                rr.teardown_displays(verbose=False)
            except Exception as _e:                           # noqa: BLE001
                log("   display teardown after %s raised %s -- the next "
                    "mode's precondition will name what is left" % (mode, _e))
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

    # VERIFY WHAT WAS JUST RECORDED, HERE, NOT AS A DOCUMENTED NEXT STEP.
    #
    # verify_grasp_quality, verify_object_attachment and verify_gripper_motion
    # were each written to answer a question the pixel check cannot -- do the
    # fingers open and close at the right moments, does the OBJECT travel with
    # the gripper rather than the gripper waving at a stationary prop, and is
    # the wrist doing anything -- and NOTHING CALLED ANY OF THEM. Three tools
    # built for exactly this sweep, none of them wired to it, so every clip
    # shipped judged only on brightness and motion. A verifier nobody runs is
    # documentation.
    #
    # Failures do NOT unrecord the clips: the recording is the expensive part
    # and a clip that fails verification is evidence of the failure. The exit
    # code carries it instead, so a caller cannot treat a bad sweep as a good
    # one.
    if not a.no_verify:
        log("\n" + "=" * 74)
        log("VERIFYING")
        vfail = []
        for name in ("verify_rviz_clips", "verify_gripper_motion",
                     "verify_object_attachment", "verify_grasp_quality"):
            r = subprocess.run([sys.executable,
                                os.path.join(WS, "scripts", "%s.py" % name)],
                               capture_output=True, text=True)
            tail = [ln for ln in (r.stdout or "").strip().splitlines()
                    if ln.strip()][-1:] or ["(no output)"]
            log("   %-26s exit %d   %s" % (name, r.returncode, tail[0][:90]))
            if r.returncode != 0:
                vfail.append(name)
        if vfail:
            log("   VERIFICATION FAILED: %s" % ", ".join(vfail))
            failed += len(vfail)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
