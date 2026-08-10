#!/usr/bin/env python3
"""SRL CONSOLE — flight-software-grade operations GUI.

    python3 scripts/srl_console.py        (or: ros2 run srl_teleop console)

FRAMEWORK: Dear PyGui. Chosen after measuring, not from preference.

    MEASURED under WSLg on this machine, 110 frames with a live plot:
        median 1.85 ms   p95 2.29 ms   max 2.89 ms

    The 33 ms budget is met with 30x headroom. WSLg's D3D12 layer caps
    hardware OpenGL below 4.0, which matters for GL4-only toolkits; Dear
    PyGui targets 3.3 and renders fine.

    Rejected: tkinter+matplotlib — a redraw of a matplotlib canvas costs tens
    of ms and the existing tkinter launcher already showed Tk's threading
    model biting twice. Rejected for now: a FastAPI web UI — genuinely better
    for phone/Quest/SSH access and worth revisiting, but it is a second
    process, a second failure mode, and a websocket to debug when the point
    of this tool is to debug everything else.

ARCHITECTURE — the part that decides whether it feels smooth
------------------------------------------------------------
  * ROS runs on its own thread and owns every subscription, service call and
    parameter write.
  * The draw loop NEVER calls a service, never waits, never touches the
    network. It reads ONE immutable snapshot dict published by the ROS
    thread. `teleop_gui` had exactly this bug: control keys blocked its 5 Hz
    draw loop for seconds, so the console hung precisely when it was being
    used to find out why something was hung.
  * TIERED REFRESH — strip charts 30 Hz, numerics 10 Hz, health verdicts
    1 Hz, process/disk 0.2 Hz. Redrawing everything at 60 Hz is how a
    monitoring tool becomes the load it is monitoring.
  * The GUI's own frame time is displayed. A monitoring tool that stutters is
    untrustworthy, so it reports on itself.

`teleop_gui` remains the TERMINAL fallback for SSH with no display.
"""
import collections
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))

try:
    import dearpygui.dearpygui as dpg
except ImportError:
    sys.stderr.write(
        "Dear PyGui is not installed:  pip install --user dearpygui\n"
        "Terminal fallback:  ros2 run srl_teleop teleop_gui\n")
    raise SystemExit(2)

# ---- palette. Colour encodes STATE only, never decoration. ----
# ISA-101: a LOW-SATURATION grey base, saturated colour reserved for the
# abnormal. A normal state is therefore mostly UNCOLOURED -- not green. The
# first version painted five safety items red at once, and when everything is
# red nothing reads as urgent.
BG      = (18, 19, 22)
PANEL   = (26, 28, 32)
ROW     = (32, 34, 39)
INK     = (228, 230, 234)          # primary text
MUTED   = (150, 154, 162)          # labels, units
DIM     = (96, 100, 108)           # no-data, disabled
ACCENT  = (86, 156, 214)           # the ONE accent: headings only
GOOD    = (76, 175, 96)
WARN    = (214, 158, 46)
BAD     = (226, 78, 68)
# row tints: the status colour applied to the ROW, at low alpha, so the
# status and its reason are one visual unit instead of floating text.
TINT = {"OK": (30, 46, 34), "NO FRESH DATA": (52, 43, 21),
        "STOPPED": (66, 30, 30), "WAITING": (30, 32, 36),
        "NO DATA": (28, 30, 34)}
FG_OF = {"OK": GOOD, "NO FRESH DATA": WARN, "STOPPED": BAD,
         "WAITING": DIM, "NO DATA": DIM}
DEJA = "/usr/share/fonts/truetype/dejavu/"


def fmt(v, spec="%.3f", unit="", sentinel=True):
    """Render a number, or '--' when it is absent.

    A sentinel printed as -1.000 looks MEASURED. This is the same
    no-data/bad-data distinction the pipeline view turns on, applied to every
    numeric on the screen.
    """
    if v is None:
        return "--"
    try:
        f = float(v)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(f):
        return "--"
    if sentinel and f <= -0.999 and f >= -1.001:
        return "--"
    return (spec % f) + (" " + unit if unit else "")

ARMS = ("left", "right")
JOINTS = 7
CONTINUOUS = (0, 2, 4, 6)          # joints 1/3/5/7 wrap
STALE_S = 2.0                      # a value unchanged this long is flagged
CHART_S = 60.0

STACK_EXE = ("/move_group", "/ros2_control_node", "/master_pose_node",
             "/ik_follower_node", "/robot_state_publisher")

# Stage names say what the stage IS, in words a person would use. The old
# set ("FK / spherical", "workspace map", "guard") were internal module names
# shown to a human.
PIPELINE = [
    ("teensy",     "Master arm board"),
    ("master",     "Reading the sensors"),
    ("validation", "Checking the readings"),
    ("fk",         "Working out arm position"),
    ("workspace",  "Mapping to the robot"),
    ("ik",         "Solving joint angles"),
    ("guard",      "Safety checks"),
    ("controller", "Sending to the arm"),
    ("sim",        "Simulated arm"),
    ("bridge",     "Sim to real bridge"),
    ("real",       "Real arm"),
]


# ---------------------------------------------------------------- helpers
def real_stack_procs():
    """argv[0]-based, so a shell command line that merely CONTAINS these
    words cannot match. Two earlier versions of this check matched their own
    command line -- once grepping bare names, once because the documentation
    being written quoted the full paths."""
    out = []
    try:
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                with open("/proc/%s/cmdline" % pid, "rb") as f:
                    argv0 = f.read().split(b"\0")[0].decode("utf8", "replace")
            except OSError:
                continue
            if any(argv0.endswith(x) for x in STACK_EXE):
                out.append((int(pid), argv0))
    except OSError:
        pass
    return out


def proc_rss_mb(pid):
    try:
        with open("/proc/%d/statm" % pid) as f:
            return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2**20
    except (OSError, IndexError, ValueError):
        return 0.0


def teensy_ports():
    import glob
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def disk_free_gb(path=WS):
    try:
        s = os.statvfs(path)
        return s.f_bavail * s.f_frsize / 2**30
    except OSError:
        return float("nan")


def have(p):
    return os.path.exists(os.path.join(WS, p))


def models_ok():
    v = os.path.join(WS, ".percep_venv", "bin", "python")
    if not os.path.exists(v):
        return False, "perception venv not created"
    return True, ""


def quest_up(port=8766):
    import socket
    s = socket.socket()
    s.settimeout(0.3)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


class Ring:
    """Fixed-length time series for the strip charts. Pre-allocated so the
    draw loop never allocates."""

    def __init__(self, seconds=CHART_S, hz=30):
        self.n = int(seconds * hz)
        self.t = collections.deque(maxlen=self.n)
        self.v = collections.deque(maxlen=self.n)

    def add(self, t, v):
        if v is None or (isinstance(v, float) and not math.isfinite(v)):
            return
        self.t.append(t)
        self.v.append(float(v))

    def series(self):
        return list(self.t), list(self.v)


class Job:
    """Managed subprocess in its own process GROUP, output pumped by a reader
    thread. The draw loop only ever drains a queue."""

    def __init__(self, name, argv):
        self.name = name
        self.q = queue.Queue()
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        self.p = subprocess.Popen(argv, cwd=WS, env=env,
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True,
                                  bufsize=1, start_new_session=True)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        try:
            for line in self.p.stdout:
                self.q.put(line.rstrip("\n"))
        except Exception:                                    # noqa: BLE001
            pass
        self.q.put("[%s exited %s]" % (self.name, self.p.poll()))

    @property
    def alive(self):
        return self.p.poll() is None

    def stop(self, grace=6.0):
        """SIGINT the GROUP first: the Kortex bridge closes its session on
        SIGINT and LEAKS it on SIGKILL, and the arm permits exactly one."""
        if not self.alive:
            return
        try:
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
        except OSError:
            return
        t0 = time.monotonic()
        while self.alive and time.monotonic() - t0 < grace:
            time.sleep(0.1)
        if self.alive:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except OSError:
                pass


# ------------------------------------------------------------ ROS thread
class Ros:
    """Owns every ROS interaction. Publishes an immutable snapshot dict.

    The GUI reads `self.snap` — rebound atomically, never mutated in place —
    so the draw loop needs no lock and can never block on one.
    """

    def __init__(self, evlog):
        self.ok = False
        self.snap = {}
        self.evlog = evlog
        self.rings = {}
        self._raw_hist = {a: [None] * JOINTS for a in ARMS}
        self._raw_last_change = {a: [0.0] * JOINTS for a in ARMS}
        self._raw_updates = {a: [0] * JOINTS for a in ARMS}
        self._raw_jumps = {a: [0] * JOINTS for a in ARMS}
        self._raw_frames = {a: 0 for a in ARMS}
        self._t0 = time.monotonic()
        self._stop = threading.Event()
        self._done = threading.Event()
        self._cmd_q = queue.Queue()
        self._state = {}
        threading.Thread(target=self._run, daemon=True).start()

    def ring(self, key):
        if key not in self.rings:
            self.rings[key] = Ring()
        return self.rings[key]

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (DurabilityPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from std_msgs.msg import Bool, Float64MultiArray, String
            from sensor_msgs.msg import JointState
            from rcl_interfaces.srv import GetParameters, SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue
        except Exception as e:                               # noqa: BLE001
            self.evlog(("error", "gui", "rclpy unavailable: %s" % e))
            self._done.set()
            return
        rclpy.init(args=None)
        n = Node("srl_console")
        self.node = n
        self.ok = True
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
        S = self._state

        def stamp(k, v):
            S[k] = v
            S["_t_" + k] = time.monotonic()

        def on_raw(a, m):
            now = time.monotonic()
            d = list(m.data)
            self._raw_frames[a] += 1
            for i in range(min(JOINTS, len(d))):
                prev = self._raw_hist[a][i]
                if prev is None or d[i] != prev:
                    if prev is not None:
                        j = abs(d[i] - prev)
                        j = min(j, 360.0 - j)
                        if j > 60.0:
                            self._raw_jumps[a][i] += 1
                    self._raw_hist[a][i] = d[i]
                    self._raw_last_change[a][i] = now
                    self._raw_updates[a][i] += 1
            stamp("raw_%s" % a, d)

        for a in ARMS:
            n.create_subscription(Float64MultiArray, "/master_arm_raw_%s" % a,
                                  (lambda m, x=a: on_raw(x, m)), 50)
            n.create_subscription(Float64MultiArray, "/master_status_%s" % a,
                                  (lambda m, x=a: stamp("ms_%s" % x,
                                                        list(m.data))), 20)
            n.create_subscription(Float64MultiArray, "/ik_status_%s" % a,
                                  (lambda m, x=a: stamp("ik_%s" % x,
                                                        list(m.data))), 20)
            n.create_subscription(Float64MultiArray, "/bridge_status_%s" % a,
                                  (lambda m, x=a: stamp("br_%s" % x,
                                                        list(m.data))), 20)
        n.create_subscription(Float64MultiArray, "/master_fsr_buttons",
                              lambda m: stamp("fsr", list(m.data)), 20)
        n.create_subscription(Bool, "/estop_state",
                              lambda m: stamp("estop", bool(m.data)), 10)
        n.create_subscription(Float64MultiArray, "/estop_deadman",
                              lambda m: stamp("deadman", list(m.data)), 10)
        n.create_subscription(String, "/master_channel_state",
                              lambda m: stamp("channels", m.data), tl)
        n.create_subscription(String, "/blocking",
                              lambda m: stamp("blocking", m.data), 20)
        n.create_subscription(String, "/blocking_summary",
                              lambda m: stamp("blocking_summary", m.data), 10)
        n.create_subscription(String, "/vr_state",
                              lambda m: stamp("vr", m.data), 10)
        n.create_subscription(String, "/autonomy_stage",
                              lambda m: stamp("autonomy", m.data), 10)
        n.create_subscription(String, "/trial_state",
                              lambda m: stamp("trial", m.data), 10)
        n.create_subscription(Bool, "/observer_estop_present",
                              lambda m: stamp("observer", bool(m.data)), 10)
        n.create_subscription(JointState, "/joint_states",
                              lambda m: stamp("js", (list(m.name),
                                                     list(m.position))), 30)
        n.create_subscription(JointState, "/real/joint_states",
                              lambda m: stamp("rjs", (list(m.name),
                                                      list(m.position))), 30)
        self.estop_pub = n.create_publisher(Bool, "/estop", 10)

        import tf2_ros
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, n)

        def ee(arm):
            try:
                import rclpy.time
                t = buf.lookup_transform("world", "%s_end_effector_link" % arm,
                                         rclpy.time.Time())
                v = t.transform.translation
                return (v.x, v.y, v.z)
            except Exception:                                # noqa: BLE001
                return None

        last_pub = 0.0
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            # ---- queued commands (params, estop) run HERE, on this thread
            try:
                while True:
                    kind, args = self._cmd_q.get_nowait()
                    if kind == "param":
                        node_name, param, value, cb = args
                        ok, err, old = self._set_param(
                            n, SetParameters, GetParameters, Parameter,
                            ParameterValue, node_name, param, value)
                        cb(ok, err, old)
                    elif kind == "estop":
                        self.estop_pub.publish(Bool(data=True))
                        self.evlog(("error", "console",
                                    "E-STOP published on /estop"))
            except queue.Empty:
                pass
            now = time.monotonic()
            if now - last_pub < 0.03:
                continue
            last_pub = now
            for a in ARMS:
                p = ee(a)
                if p:
                    stamp("ee_%s" % a, p)
                    self.ring("ee_%s_x" % a).add(now, p[0])
                    self.ring("ee_%s_z" % a).add(now, p[2])
                d = S.get("ik_%s" % a)
                if d and len(d) > 5:
                    self.ring("clear_%s" % a).add(now, d[5])
                b = S.get("br_%s" % a)
                if b and len(b) > 2:
                    self.ring("lag_%s" % a).add(now, b[2])
            snap = dict(S)
            snap["_now"] = now
            snap["raw_meta"] = {
                a: dict(last_change=list(self._raw_last_change[a]),
                        updates=list(self._raw_updates[a]),
                        jumps=list(self._raw_jumps[a]),
                        frames=self._raw_frames[a])
                for a in ARMS}
            snap["_uptime"] = now - self._t0
            self.snap = snap                       # atomic rebind
        try:
            n.destroy_node()
            rclpy.shutdown()
        except Exception:                                    # noqa: BLE001
            pass
        self._done.set()

    def _set_param(self, n, SetParameters, GetParameters, Parameter,
                   ParameterValue, node_name, param, value):
        old = None
        try:
            getc = n.create_client(GetParameters,
                                   "%s/get_parameters" % node_name)
            if getc.wait_for_service(timeout_sec=0.8):
                f = getc.call_async(GetParameters.Request(names=[param]))
                t0 = time.monotonic()
                while not f.done() and time.monotonic() - t0 < 1.5:
                    import rclpy
                    rclpy.spin_once(n, timeout_sec=0.02)
                if f.done() and f.result() and f.result().values:
                    v = f.result().values[0]
                    old = (v.double_value if v.type == 3 else
                           v.integer_value if v.type == 2 else
                           v.bool_value if v.type == 1 else v.string_value)
            cli = n.create_client(SetParameters,
                                  "%s/set_parameters" % node_name)
            if not cli.wait_for_service(timeout_sec=1.5):
                return False, "%s has no set_parameters service" % node_name, old
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type, pv.bool_value = 1, value
            elif isinstance(value, int):
                pv.type, pv.integer_value = 2, value
            else:
                pv.type, pv.double_value = 3, float(value)
            fut = cli.call_async(SetParameters.Request(
                parameters=[Parameter(name=param, value=pv)]))
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 2.0:
                import rclpy
                rclpy.spin_once(n, timeout_sec=0.02)
            if not fut.done() or not fut.result():
                return False, "set_parameters did not return", old
            r = fut.result().results[0]
            return bool(r.successful), (r.reason or ""), old
        except Exception as e:                               # noqa: BLE001
            return False, str(e)[:90], old

    # ---- called FROM the GUI thread; never blocks it
    def set_param(self, node_name, param, value, cb):
        self._cmd_q.put(("param", (node_name, param, value, cb)))

    def estop(self):
        self._cmd_q.put(("estop", None))

    def shutdown(self, timeout=3.0):
        self._stop.set()
        t0 = time.monotonic()
        while not self._done.is_set() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)


# ------------------------------------------------------------------- GUI
class Console:
    def __init__(self):
        self.events = collections.deque(maxlen=800)
        # Created HERE, not in the last tab built. launch() writes to it and
        # was one tab reordering away from an AttributeError on first click.
        self.outbuf = collections.deque(maxlen=900)
        self.ros = Ros(self.log_event)
        self.jobs = {}
        self.frame_ms = collections.deque(maxlen=90)
        self.sel_stage = None
        self._last = {"30": 0.0, "10": 0.0, "1": 0.0, "02": 0.0}
        self._proc = []
        self._disk = 0.0

    # ------------------------------------------------------------ events
    def log_event(self, ev):
        sev, node, msg = ev
        self.events.appendleft((time.strftime("%H:%M:%S"), sev, node, msg))

    # ------------------------------------------------------------ layout
    def build(self):
        dpg.create_context()
        with dpg.theme() as th:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, BG)
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, PANEL)
                dpg.add_theme_color(dpg.mvThemeCol_Text, INK)
                dpg.add_theme_color(dpg.mvThemeCol_Button, (40, 42, 48))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, ACCENT)
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (32, 34, 39))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 10, 10)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 5)
                dpg.add_theme_color(dpg.mvThemeCol_Border, (0, 0, 0, 0))
        dpg.bind_theme(th)
        # THREE TYPE LEVELS ONLY. Proportional sans for labels and prose,
        # monospace for every numeric so digits are fixed-width and the
        # layout cannot jitter as values change.
        # Base sizes chosen to be readable at arm's length, not at 40 cm.
        # dpg.set_global_font_scale() then scales them all live.
        with dpg.font_registry():
            self.f_big = dpg.add_font(DEJA + "DejaVuSansMono-Bold.ttf", 38)
            self.f_mid = dpg.add_font(DEJA + "DejaVuSansMono.ttf", 22)
            self.f_lbl = dpg.add_font(DEJA + "DejaVuSans.ttf", 19)
            self.f_sml = dpg.add_font(DEJA + "DejaVuSans.ttf", 16)
        dpg.bind_font(self.f_lbl)
        dpg.create_viewport(title="SRL console", width=1720, height=1010)

        with dpg.window(tag="root"):
            self._safety_bar()
            with dpg.tab_bar():
                self._tab_overview()
                self._tab_master()
                self._tab_robot()
                self._tab_charts()
                self._tab_launch()
                self._tab_experiments()
                self._tab_diag()
                self._tab_log()
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("root", True)
        # PANELS FOLLOW THE WINDOW. The columns were fixed at 1010/660 px, so
        # a narrower window cut the last stage mid-word and a wider one left
        # two thirds empty.
        dpg.set_viewport_resize_callback(self._on_resize)
        self._on_resize()

    def _on_resize(self, *_):
        try:
            w = max(dpg.get_viewport_client_width(), 900)
            h = max(dpg.get_viewport_client_height(), 600)
        except Exception:                                    # noqa: BLE001
            return
        body = h - 210                       # minus the safety bar and tabs
        left = int(w * 0.60)
        right = w - left - 40
        for tag, ww, hh in (("col_pipe", left, body),
                            ("col_side", right, body)):
            if dpg.does_item_exist(tag):
                dpg.configure_item(tag, width=ww, height=hh)
        # wrap widths follow the column, so a reason is never truncated
        for key, _l in PIPELINE:
            if dpg.does_item_exist("prs_%s" % key):
                dpg.configure_item("prs_%s" % key, wrap=max(left - 260, 220))
        for tag in ("blk_src", "blk_txt", "stage_detail", "proc_warn",
                    "ev_tail"):
            if dpg.does_item_exist(tag):
                dpg.configure_item(tag, wrap=max(right - 30, 220))

    def _safety_bar(self):
        """ALWAYS VISIBLE, and with ONE place for the eye to land.

        E-stop is the only large red element. Observer-not-confirmed is
        amber. Velocity caps and frame time are plain grey. The first version
        painted all five red simultaneously, which is the same as painting
        none of them.
        """
        with dpg.group(horizontal=True):
            dpg.add_button(label="E-STOP", width=150, height=72,
                           callback=self.on_estop, tag="btn_estop")
            with dpg.theme() as t:
                with dpg.theme_component(dpg.mvButton):
                    dpg.add_theme_color(dpg.mvThemeCol_Button, BAD)
                    dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,
                                        (245, 110, 100))
                    dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.bind_item_theme("btn_estop", t)
            dpg.add_spacer(width=16)

            # THE focal readout: large, and coloured only when abnormal.
            with dpg.group():
                dpg.add_text("SYSTEM", color=MUTED, tag="sf_lbl")
                dpg.bind_item_font("sf_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_state")
                dpg.bind_item_font("sf_state", self.f_big)
            dpg.add_spacer(width=18)

            with dpg.group():
                dpg.add_text("DEAD-MAN", color=MUTED, tag="dm_lbl")
                dpg.bind_item_font("dm_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_deadman")
                dpg.bind_item_font("sf_deadman", self.f_mid)
            dpg.add_spacer(width=16)

            with dpg.group():
                dpg.add_text("OBSERVER E-STOP", color=MUTED, tag="ob_lbl")
                dpg.bind_item_font("ob_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_observer")
                dpg.bind_item_font("sf_observer", self.f_mid)
            dpg.add_spacer(width=16)

            with dpg.group():
                dpg.add_text("MIN CLEARANCE", color=MUTED, tag="cl_lbl")
                dpg.bind_item_font("cl_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_clear")
                dpg.bind_item_font("sf_clear", self.f_mid)
            dpg.add_spacer(width=16)

            with dpg.group():
                dpg.add_text("VEL CAP  sim / real", color=MUTED,
                             tag="vc_lbl")
                dpg.bind_item_font("vc_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_caps", color=MUTED)
                dpg.bind_item_font("sf_caps", self.f_mid)
            dpg.add_spacer(width=16)

            with dpg.group():
                dpg.add_text("GUI FRAME", color=MUTED, tag="fr_lbl")
                dpg.bind_item_font("fr_lbl", self.f_sml)
                dpg.add_text("--", tag="sf_frame", color=DIM)
                dpg.bind_item_font("sf_frame", self.f_mid)

            dpg.add_spacer(width=14)
            with dpg.group():
                dpg.add_text("TEXT SIZE", color=MUTED, tag="fs_lbl")
                dpg.bind_item_font("fs_lbl", self.f_sml)
                dpg.add_slider_float(tag="font_scale", default_value=1.0,
                                     min_value=0.7, max_value=1.8, width=110,
                                     format="%.2fx",
                                     callback=lambda s_, a_:
                                     dpg.set_global_font_scale(a_))
            dpg.add_spacer(width=12)
            dpg.add_button(label="Diagnose\nstopped motion", width=170,
                           height=72, callback=self.on_why)
            dpg.add_button(label="Stop\neverything", width=132, height=72,
                           callback=self.on_stop_all)
        dpg.add_spacer(height=6)

    # ---------- overview: pipeline + blockers, the two that matter most
    def _tab_overview(self):
        with dpg.tab(label="Overview"):
            with dpg.group(horizontal=True):
                # ---- PIPELINE: VERTICAL. Eleven stages never fit across a
                # window; vertical gives each a full-width row for its
                # untruncated reason, and fills the space the horizontal
                # chain left empty.
                with dpg.child_window(width=1010, height=880, border=False,
                                      tag="col_pipe"):
                    dpg.add_text("SIGNAL PATH", color=ACCENT, tag="h_pipe")
                    dpg.bind_item_font("h_pipe", self.f_lbl)
                    dpg.add_spacer(height=4)
                    for key, label in PIPELINE:
                        with dpg.child_window(height=62, border=False,
                                              no_scrollbar=True,
                                              tag="row_%s" % key):
                            with dpg.group(horizontal=True):
                                dpg.add_button(label=label, width=268,
                                               height=40, tag="pl_%s" % key,
                                               callback=(lambda s_, a_, u=key:
                                                         self.on_stage(u)))
                                dpg.add_spacer(width=10)
                                with dpg.group():
                                    with dpg.group(horizontal=True):
                                        dpg.add_text("--",
                                                     tag="pst_%s" % key)
                                        dpg.bind_item_font("pst_%s" % key,
                                                           self.f_mid)
                                        dpg.add_spacer(width=10)
                                        dpg.add_text("", tag="prt_%s" % key,
                                                     color=MUTED)
                                        dpg.bind_item_font("prt_%s" % key,
                                                           self.f_sml)
                                    # THE REASON IS THE PAYLOAD. Wrapped,
                                    # never truncated -- shorten the label
                                    # instead if space is short.
                                    dpg.add_text("", tag="prs_%s" % key,
                                                 color=MUTED, wrap=700)
                                    dpg.bind_item_font("prs_%s" % key,
                                                       self.f_sml)
                    dpg.add_spacer(height=12)
                    dpg.add_text("RECENT EVENTS", color=ACCENT, tag="h_ev2")
                    dpg.bind_item_font("h_ev2", self.f_lbl)
                    dpg.add_text("", tag="ev_tail", color=MUTED, wrap=980)
                    dpg.bind_item_font("ev_tail", self.f_sml)

                with dpg.child_window(width=660, height=880, border=False,
                                      tag="col_side"):
                    dpg.add_text("WHAT IS STOPPING MOTION", color=ACCENT,
                                 tag="h_blk")
                    dpg.bind_item_font("h_blk", self.f_lbl)
                    dpg.add_spacer(height=4)
                    dpg.add_text("--", tag="blk_src", color=MUTED, wrap=620)
                    dpg.bind_item_font("blk_src", self.f_sml)
                    dpg.add_spacer(height=6)
                    with dpg.child_window(height=250, border=False):
                        dpg.add_text("", tag="blk_txt", wrap=610)
                        dpg.bind_item_font("blk_txt", self.f_sml)
                    dpg.add_spacer(height=14)
                    dpg.add_text("STAGE DETAIL", color=ACCENT, tag="h_sd")
                    dpg.bind_item_font("h_sd", self.f_lbl)
                    dpg.add_text("click a stage", tag="stage_detail",
                                 color=MUTED, wrap=620)
                    dpg.bind_item_font("stage_detail", self.f_sml)
                    dpg.add_spacer(height=14)
                    dpg.add_text("PROCESSES", color=ACCENT, tag="h_pr")
                    dpg.bind_item_font("h_pr", self.f_lbl)
                    dpg.add_text("", tag="proc_warn", color=BAD, wrap=620)
                    dpg.bind_item_font("proc_warn", self.f_sml)
                    with dpg.child_window(height=250, border=False):
                        dpg.add_text("", tag="proc_txt")
                        dpg.bind_item_font("proc_txt", self.f_sml)

    # ---------- master arm
    def _tab_master(self):
        with dpg.tab(label="Master arm"):
            with dpg.group(horizontal=True):
                for a in ARMS:
                    with dpg.child_window(width=830, height=740):
                        dpg.add_text(a.upper(), color=ACCENT)
                        dpg.add_text("", tag="m_hdr_%s" % a, color=DIM)
                        dpg.add_separator()
                        for i in range(JOINTS):
                            with dpg.group(horizontal=True):
                                dpg.add_text("j%d" % (i + 1), color=DIM)
                                dpg.add_progress_bar(
                                    default_value=0.0, width=250,
                                    tag="m_bar_%s_%d" % (a, i))
                                dpg.add_text("", tag="m_val_%s_%d" % (a, i))
                        dpg.add_separator()
                        dpg.add_text("IMU", color=ACCENT)
                        dpg.add_text("", tag="m_imu_%s" % a)
                        dpg.add_separator()
                        dpg.add_text("FSR / buttons / clutch", color=ACCENT)
                        dpg.add_text("", tag="m_fsr_%s" % a)
                        dpg.add_separator()
                        dpg.add_text("SERIAL", color=ACCENT)
                        dpg.add_text("", tag="m_ser_%s" % a)

    # ---------- robot
    def _tab_robot(self):
        with dpg.tab(label="Robot"):
            with dpg.group(horizontal=True):
                for a in ARMS:
                    with dpg.child_window(width=830, height=740):
                        dpg.add_text("%s ARM" % a.upper(), color=ACCENT)
                        for i in range(JOINTS):
                            with dpg.group(horizontal=True):
                                dpg.add_text("q%d" % (i + 1), color=DIM)
                                dpg.add_progress_bar(
                                    default_value=0.5, width=230,
                                    tag="r_bar_%s_%d" % (a, i))
                                dpg.add_text("", tag="r_val_%s_%d" % (a, i))
                        dpg.add_separator()
                        dpg.add_text("EE / IK", color=ACCENT)
                        dpg.add_text("", tag="r_ee_%s" % a)
                        dpg.add_separator()
                        dpg.add_text("CLEARANCE", color=ACCENT)
                        dpg.add_progress_bar(default_value=0.0, width=560,
                                             tag="r_clr_%s" % a)
                        dpg.add_text("", tag="r_clrtxt_%s" % a)
                        dpg.add_separator()
                        dpg.add_text("REAL ARM", color=ACCENT)
                        dpg.add_text("", tag="r_real_%s" % a)

    # ---------- charts
    def _tab_charts(self):
        with dpg.tab(label="Charts"):
            specs = [("EE x (m)", "ee_left_x", "ee_right_x"),
                     ("EE z (m)", "ee_left_z", "ee_right_z"),
                     ("clearance (m)", "clear_left", "clear_right"),
                     ("sim->real lag (rad)", "lag_left", "lag_right")]
            for i, (title, kl, kr) in enumerate(specs):
                with dpg.plot(label=title, height=200, width=-1,
                              tag="plot_%d" % i):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="s",
                                      tag="px_%d" % i)
                    with dpg.plot_axis(dpg.mvYAxis, tag="py_%d" % i):
                        dpg.add_line_series([], [], label="left",
                                            tag="ser_%d_l" % i)
                        dpg.add_line_series([], [], label="right",
                                            tag="ser_%d_r" % i)
            self._chart_keys = specs

    # ---------- launch
    def _tab_launch(self):
        with dpg.tab(label="Modes"):
            with dpg.group(horizontal=True):
                dpg.add_combo(("both", "left", "right"), default_value="both",
                              width=110, tag="opt_arm", label="arm")
                dpg.add_combo(("spherical", "fk"), default_value="spherical",
                              width=120, tag="opt_pos", label="position")
                dpg.add_combo(("fixed", "tilt"), default_value="fixed",
                              width=100, tag="opt_ori", label="orientation")
                dpg.add_combo(("auto", "on", "off"), default_value="auto",
                              width=90, tag="opt_deg", label="degraded")
            dpg.add_separator()
            mok, mwhy = models_ok()
            # quest_up() was defined and never called, so this never greyed
            # out when the headset server was down.
            qok = quest_up()
            rows = [
                ("Sim only", self.run_sim, True, ""),
                ("Mannequin teleop", self.run_mannequin, True, ""),
                ("Mannequin -> real (cascade)", self.run_cascade, True, ""),
                ("VR teleop", self.run_vr,
                 have("scripts/start_vr.sh") and qok,
                 ("scripts/start_vr.sh missing" if not have("scripts/start_vr.sh")
                  else "No Quest client on ws://127.0.0.1:8766. Start the "
                       "headset app and run: adb reverse tcp:8766 tcp:8766")),
                ("Mode 3 orientation assist", lambda: self.run_mode(3), False,
                 "STUB: this /compute_ik plugin ignores OrientationConstraint "
                 "(measured: identical IK success with and without)."),
                ("Mode 4 shared autonomy", lambda: self.run_mode(4), True, ""),
                ("Mode 5 supervised", lambda: self.run_mode(5), True, ""),
                ("Mode 6 full autonomy (voice)", lambda: self.run_mode(6),
                 mok, mwhy or "perception models not installed"),
            ]
            for label, fn, en, why in rows:
                b = dpg.add_button(label=label, width=330, height=34,
                                   callback=(lambda s, a, f=fn: f()),
                                   enabled=en)
                if not en:
                    with dpg.tooltip(b):
                        dpg.add_text(why, wrap=420)
            dpg.add_separator()
            dpg.add_text("LIVE CONTROLS", color=ACCENT)
            self.controls = [
                ("sim max_vel (rad/s)", "/ik_follower_left", "max_vel_rad_s",
                 0.05, 1.0, 0.6),
                ("scale left", "/master_pose_node", "left_scale", 0.1, 2.0, 1.0),
                ("scale right", "/master_pose_node", "right_scale", 0.1, 2.0, 1.0),
                ("ema alpha", "/master_pose_node", "ema_alpha", 0.05, 1.0, 0.3),
                ("accel gate (g)", "/master_pose_node", "accel_gate_g",
                 0.05, 0.5, 0.15),
                ("max_step (rad)", "/ik_follower_left", "max_step_rad",
                 0.02, 0.6, 0.35),
                ("lag trip (rad)", "/sim_to_real_bridge_left", "lag_trip_rad",
                 0.1, 1.0, 0.5),
                ("preview delay (s)", "/sim_to_real_bridge_left",
                 "preview_delay_s", 0.2, 3.0, 1.0),
            ]
            for label, node, param, lo, hi, init in self.controls:
                with dpg.group(horizontal=True):
                    dpg.add_slider_float(label="", default_value=init,
                                         min_value=lo, max_value=hi,
                                         width=280, format="%.3f",
                                         tag="ctl_%s" % param)
                    dpg.add_button(label="apply " + label, width=210,
                                   callback=(lambda s, a, n=node, p=param:
                                             self.apply_param(n, p)))

    def _tab_experiments(self):
        with dpg.tab(label="Experiments"):
            with dpg.group(horizontal=True):
                dpg.add_combo(("direct", "assisted", "shared"),
                              default_value="direct", width=120,
                              tag="ex_cond", label="condition")
                dpg.add_combo(("S1", "S2", "S3", "S4"), default_value="S1",
                              width=80, tag="ex_scen", label="scenario")
                dpg.add_input_text(default_value="P00", width=80,
                                   tag="ex_pid", label="participant")
            dpg.add_separator()
            for t, key, en, why in (
                    ("Rigid carry (T3)", "t3", True, ""),
                    ("Compliant carry (T6)", "t6", True, ""),
                    ("Pursuit tracking (T7)", "t7", True, ""),
                    ("Handover to wearer (T5)", "t5", True, ""),
                    ("Hold and fill (T2)", "t2", False,
                     "No verified scenarios exist for T2 yet -- "
                     "scripts/verify_scenarios.py does not generate them. "
                     "The task is feasible with the side-handle container, "
                     "but it is not in the 75 min session (T5 keeps P6).")):
                b = dpg.add_button(label=t, width=300, height=32,
                                   enabled=en,
                                   callback=(lambda s, a, k=key:
                                             self.run_task(k)))
                if not en:
                    with dpg.tooltip(b):
                        dpg.add_text(why, wrap=400)
            dpg.add_separator()
            # There is no "session" task in the dispatcher; the P0-P7 order
            # is a sequence of the individual tasks. Claiming otherwise gave
            # a button that always exited 2.
            dpg.add_button(label="Run session order (T7, T3, T6, T5)",
                           width=300, height=34,
                           callback=lambda s=None, a=None: self.run_session())
            dpg.add_button(label="Pilot (offline)", width=300, height=30,
                           callback=lambda: self.launch(
                               "pilot", ["python3", "scripts/pilot_bimanual.py"],
                               starts_stack=False))
            dpg.add_separator()
            dpg.add_text("TRIAL STATE", color=ACCENT)
            dpg.add_text("", tag="trial_txt")

    def _tab_diag(self):
        with dpg.tab(label="Calibration"):
            for label, key, argv, need in (
                    ("Capture zeros — left", "zl",
                     ["ros2", "run", "srl_teleop", "capture_zero",
                      "--ros-args", "-p", "arm:=left"], None),
                    ("Capture zeros — right", "zr",
                     ["ros2", "run", "srl_teleop", "capture_zero",
                      "--ros-args", "-p", "arm:=right"], None),
                    ("Check channels (3 min, diffs the baseline)", "ch",
                     ["bash", "scripts/check_channels.sh"],
                     "scripts/check_channels.sh"),
                    ("Trajectory capture", "tc",
                     ["ros2", "run", "srl_experiments",
                      "record_trajectories"], None),
                    ("Measure workspace", "mw",
                     ["python3", "scripts/measure_workspace.py"],
                     "scripts/measure_workspace.py"),
                    ("Network check", "nc",
                     ["bash", "scripts/check_arm_network.sh"],
                     "scripts/check_arm_network.sh"),
                    ("Recover", "rc", ["bash", "scripts/recover.sh"],
                     "scripts/recover.sh"),
                    ("Button mapping check", "bm",
                     ["ros2", "run", "srl_teleop", "check_buttons"], None)):
                en = need is None or have(need)
                b = dpg.add_button(label=label, width=420, height=32,
                                   enabled=en,
                                   callback=(lambda s, a, k=key, v=argv:
                                             self.launch(k, v,
                                                         starts_stack=False)))
                if not en:
                    with dpg.tooltip(b):
                        dpg.add_text("%s not found" % need)

    def _tab_log(self):
        with dpg.tab(label="Event log / output"):
            dpg.add_text("EVENTS", color=ACCENT)
            with dpg.child_window(height=280):
                dpg.add_text("", tag="ev_txt")
            dpg.add_text("SUBPROCESS OUTPUT", color=ACCENT)
            with dpg.child_window(height=440, tag="out_win"):
                dpg.add_text("", tag="out_txt")

    # ------------------------------------------------------------ actions
    def preflight(self, teensy=False, real=False, starts_stack=True,
                  needs_stack=False):
        """`starts_stack` scopes the second-stack refusal.

        An autonomy node or a diagnostic is an ADD-ON to a running stack, not
        a second one -- refusing those whenever a stack exists makes the
        stack-refusal useless in the only situation where you would use them.
        Only launches that bring up their own move_group / master_pose_node
        are refused.
        """
        f = []
        running = real_stack_procs()
        # An experiment CONSUMES a stack, it does not start one. Refusing it
        # because a stack is running is backwards -- that is the only state
        # in which it can work.
        if needs_stack and not running:
            f.append("No stack is running. An experiment needs the sim up "
                     "first: use Sim only, or Mannequin teleop.")
        procs = running if starts_stack else []
        if procs:
            f.append("A stack is ALREADY RUNNING (%d processes, e.g. pid %d). "
                     "Two master_pose_node instances split the serial stream "
                     "and invalidated a full day of measurements. STOP ALL "
                     "first." % (len(procs), procs[0][0]))
        if teensy and not teensy_ports():
            f.append("No /dev/ttyACM* present. usbipd attach --wsl "
                     "--hardware-id 16c0:0483")
        if self.ros.snap.get("estop") is True:
            f.append("E-STOP is LATCHED. ros2 service call /estop_reset "
                     "std_srvs/srv/Trigger {}")
        if real:
            r = subprocess.run(["ping", "-c1", "-W2", "192.168.1.10"],
                               capture_output=True)
            if r.returncode != 0:
                f.append("192.168.1.10 not answering — off the lab network?")
        return f

    def launch(self, key, argv, teensy=False, real=False, starts_stack=True,
               needs_stack=False):
        fails = self.preflight(teensy, real, starts_stack, needs_stack)
        if fails:
            for x in fails:
                self.log_event(("error", "preflight", x))
            self.outbuf.append("=== REFUSED: %s ===" % key)
            for x in fails:
                self.outbuf.append("  • " + x)
            return
        if key in self.jobs and self.jobs[key].alive:
            self.log_event(("warn", "launch", "%s already running" % key))
            return
        self.log_event(("info", "launch", "%s: %s" % (key, " ".join(argv))))
        self.outbuf.append("=== START %s ===" % key)
        try:
            self.jobs[key] = Job(key, argv)
        except Exception as e:                               # noqa: BLE001
            self.log_event(("error", "launch", "%s failed: %s" % (key, e)))

    def run_sim(self):
        self.launch("sim", ["ros2", "launch", "srl_teleop",
                            "teleop.launch.py", "gate:=false",
                            "use_rviz:=false", "serial_port:=/dev/null"])

    def run_mannequin(self):
        self.launch("mannequin",
                    ["ros2", "launch", "srl_teleop", "teleop.launch.py",
                     "gate:=false",
                     "position_mode:=%s" % dpg.get_value("opt_pos"),
                     "orientation_mode:=%s" % dpg.get_value("opt_ori"),
                     "degraded_mode:=%s" % dpg.get_value("opt_deg")],
                    teensy=True)

    def run_cascade(self):
        self.launch("cascade", ["bash", "scripts/start_real.sh",
                                "arm:=%s" % dpg.get_value("opt_arm")],
                    teensy=True, real=True)

    def run_vr(self):
        self.launch("vr", ["bash", "scripts/start_vr.sh"])

    def run_mode(self, m):
        self.launch("mode%d" % m,
                    ["ros2", "run", "srl_autonomy", "autonomy_executive",
                     "--ros-args", "-p", "mode:=%d" % m],
                    starts_stack=False)

    def run_session(self):
        """The P0-P7 order is T7, T3, T6, T5 -- four separate runs, not one
        'session' task. Launched in sequence with the participant ID from
        the field; each is a normal managed job."""
        pid = dpg.get_value("ex_pid")
        order = [("t7", "P2 pursuit"), ("t3", "P3 rigid carry PRIMARY"),
                 ("t6", "P5 compliant carry"), ("t5", "P6 handover")]
        self.log_event(("info", "session",
                        "P0-P7 order for %s: %s" % (pid, ", ".join(
                            "%s (%s)" % (k, l) for k, l in order))))
        self.outbuf.append("=== SESSION ORDER for %s ===" % pid)
        for k, lbl in order:
            self.outbuf.append("  %-4s %s" % (k, lbl))
        self.outbuf.append("  starting the first block; run the rest in turn "
                           "so the breaks and questionnaires happen between "
                           "them.")
        self.run_task(order[0][0])

    def run_task(self, name):
        """`name` is the task KEY (t3, t6, t7, t5, t2), never a display
        label. The old version split a label on whitespace and produced
        "session" and "t3 rigid carry" -> "t3", which run_experiment.sh
        rejected with exit 2 while the GUI reported a successful launch."""
        self.launch("task_%s" % name,
                    ["bash", "scripts/run_experiment.sh", name,
                     "--participant", dpg.get_value("ex_pid"),
                     "--condition", dpg.get_value("ex_cond"),
                     "--scenario", dpg.get_value("ex_scen")],
                    starts_stack=False, needs_stack=True)

    def apply_param(self, node, param):
        val = dpg.get_value("ctl_%s" % param)

        def cb(ok, err, old):
            if ok:
                self.log_event(("info", node, "%s: %s -> %.4f"
                                % (param,
                                   ("%.4f" % old) if isinstance(old, float)
                                   else old, val)))
            else:
                self.log_event(("error", node,
                                "%s -> %.4f FAILED: %s  (value on the robot "
                                "is UNCHANGED)" % (param, val, err)))
        self.ros.set_param(node, param, float(val), cb)

    def on_estop(self, *_):
        self.ros.estop()

    def on_stop_all(self, *_):
        for k, j in list(self.jobs.items()):
            if j.alive:
                self.log_event(("warn", "console", "stopping %s" % k))
                j.stop()
        for pid, _e in real_stack_procs():
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        time.sleep(1.5)
        for pid, _e in real_stack_procs():
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass

    def on_stage(self, key):
        self.sel_stage = key

    def on_why(self, *_):
        """Walk the chain and name the FIRST stage not passing data. This is
        the control the whole application exists for: 'nothing is moving and
        I do not know why' has cost this project days."""
        states = self.pipeline_states()
        first = None
        for key, label in PIPELINE:
            st, detail, _rate = states[key]
            if st != "OK":
                first = (label, st, detail)
                break
        if first is None:
            msg = ("Everything in the chain is passing data. If the arm is "
                   "still not moving, look at the guard counters and the "
                   "clearance margin.")
        else:
            msg = ("FIRST STAGE NOT PASSING DATA:  %s  [%s]\n%s"
                   % (first[0], first[1], first[2]))
        self.log_event(("warn", "why", msg.replace("\n", "  ")))
        dpg.set_value("stage_detail", msg)

    # ------------------------------------------------------- pipeline
    def pipeline_states(self):
        s = self.ros.snap
        now = s.get("_now", time.monotonic())

        def age(k):
            t = s.get("_t_" + k)
            return None if t is None else now - t
        out = {}

        def put(k, st, detail, rate=None):
            out[k] = (st, detail, rate)

        # Teensy: a port present AND raw frames arriving
        ports = teensy_ports()
        a_raw = min([x for x in (age("raw_left"), age("raw_right"))
                     if x is not None], default=None)
        meta = s.get("raw_meta", {})
        if not ports and a_raw is None:
            put("teensy", "STOPPED", "No master arm board connected")
        elif a_raw is None:
            put("teensy", "STOPPED", "Board is plugged in but sending nothing")
        elif a_raw > STALE_S:
            put("teensy", "NO FRESH DATA", "No data for %.1f s" % a_raw)
        else:
            put("teensy", "OK", "", 1.0 / max(a_raw, 1e-3))

        # master_pose_node: raw + status
        a_ms = min([x for x in (age("ms_left"), age("ms_right"))
                    if x is not None], default=None)
        if a_ms is None:
            put("master", "STOPPED", "The sensor-reading node is not running")
        elif a_ms > STALE_S:
            put("master", "NO FRESH DATA", "%.1f s" % a_ms)
        else:
            put("master", "OK", "")

        # validation: distinct updates actually advancing
        stale_ch = []
        for a in ARMS:
            m = meta.get(a)
            if not m:
                continue
            for i in range(JOINTS):
                if now - m["last_change"][i] > STALE_S:
                    stale_ch.append("%s j%d" % (a, i + 1))
        if a_ms is None:
            put("validation", "STOPPED", "Waiting for sensor readings")
        elif len(stale_ch) >= 2 * JOINTS:
            put("validation", "STOPPED",
                "EVERY channel unchanged >%.0f s — data is being republished "
                "but not updating" % STALE_S)
        elif stale_ch:
            put("validation", "OK",
                "%d channel(s) stale: %s" % (len(stale_ch),
                                             ", ".join(stale_ch[:6])))
        else:
            put("validation", "OK", "")

        for key, k in (("fk", "ms_left"), ("workspace", "ms_left")):
            a = age(k)
            if a is None:
                put(key, "STOPPED", "Waiting for an arm position")
            elif a > STALE_S:
                put(key, "NO FRESH DATA", "%.1f s" % a)
            else:
                put(key, "OK", "")

        # IK / guard
        ik = None
        for a in ARMS:
            d = s.get("ik_%s" % a)
            if d:
                ik = d
                break
        a_ik = min([x for x in (age("ik_left"), age("ik_right"))
                    if x is not None], default=None)
        if a_ik is None:
            put("ik", "STOPPED", "The joint solver is not running")
            put("guard", "STOPPED", "Joint solver is not running")
        else:
            succ, fail = (ik[0], ik[1]) if ik and len(ik) > 1 else (0, 0)
            attempts = succ + fail
            rate = 100.0 * succ / max(attempts, 1)
            if a_ik > STALE_S:
                put("ik", "NO FRESH DATA", "Solver silent for %.1f s" % a_ik)
            elif attempts == 0:
                # NO ATTEMPTS IS NOT FAILURE. Reporting "IK success 0%" when
                # nothing has been asked of the solver points the operator at
                # the solver instead of at the missing input -- the same
                # no-data/bad-data conflation this GUI exists to prevent.
                put("ik", "NO FRESH DATA", "Solver is idle - no target positions arriving")
            elif rate < 50:
                put("ik", "STOPPED", "Only %.0f%% of %d solve attempts succeeded"
                    % (rate, attempts))
            else:
                put("ik", "OK", "%.0f%% of solves succeed" % rate, rate)
            # The guard cannot be fine while the solver has no input --
            # reporting OK there put a green row under nine stopped ones.
            if attempts == 0:
                put("guard", "WAITING",
                    "Nothing to check yet - no solutions being produced")
            else:
                put("guard", "OK", "")

        a_js = age("js")
        if a_js is None:
            put("controller", "STOPPED", "The robot is not reporting its joint angles")
            put("sim", "STOPPED", "no /joint_states")
        elif a_js > STALE_S:
            put("controller", "NO FRESH DATA", "%.1f s" % a_js)
            put("sim", "NO FRESH DATA", "%.1f s" % a_js)
        else:
            put("controller", "OK", "")
            put("sim", "OK", "")

        br = None
        for a in ARMS:
            d = s.get("br_%s" % a)
            if d:
                br = d
                break
        a_br = min([x for x in (age("br_left"), age("br_right"))
                    if x is not None], default=None)
        if a_br is None:
            put("bridge", "STOPPED", "The sim-to-real bridge is not running")
        elif br and br[0] < 0.5:
            put("bridge", "STOPPED", "Bridge is running but switched off")
        else:
            put("bridge", "OK", "%.3f rad behind the sim" % (br[2] if br else 0))
        a_rjs = age("rjs")
        if a_rjs is None:
            put("real", "STOPPED", "The real arm is not reporting joint angles")
        elif a_rjs > STALE_S:
            put("real", "NO FRESH DATA", "%.1f s" % a_rjs)
        else:
            put("real", "OK", "")

        if s.get("estop"):
            for k in ("guard", "controller", "sim", "bridge", "real"):
                out[k] = ("STOPPED", "Emergency stop is latched", None)
        return out

    # ---------------------------------------------------------- refresh
    def tick(self):
        t0 = time.perf_counter()
        now = time.monotonic()
        s = self.ros.snap
        if now - self._last["30"] >= 1 / 30.0:
            self._last["30"] = now
            self._charts()
        if now - self._last["10"] >= 0.1:
            self._last["10"] = now
            self._numerics(s, now)
            self._drain_output()
        if now - self._last["1"] >= 1.0:
            self._last["1"] = now
            self._verdicts(s, now)
        if now - self._last["02"] >= 5.0:
            self._last["02"] = now
            self._proc = real_stack_procs()
            self._disk = disk_free_gb()
        self.frame_ms.append((time.perf_counter() - t0) * 1000.0)

    def _drain_output(self):
        for j in list(self.jobs.values()):
            for _ in range(150):
                try:
                    self.outbuf.append(j.q.get_nowait())
                except queue.Empty:
                    break
        dpg.set_value("out_txt", "\n".join(list(self.outbuf)[-260:]))

    def _charts(self):
        for i, (_t, kl, kr) in enumerate(self._chart_keys):
            for suffix, key in (("l", kl), ("r", kr)):
                r = self.ros.rings.get(key)
                if not r or not r.t:
                    continue
                t, v = r.series()
                t0 = t[-1]
                dpg.set_value("ser_%d_%s" % (i, suffix),
                              [[x - t0 for x in t], v])
            dpg.set_axis_limits("px_%d" % i, -CHART_S, 2)
            # The y axis was never scaled, so every series rendered outside
            # the visible range -- the charts looked empty even with data.
            dpg.fit_axis_data("py_%d" % i)

    def _numerics(self, s, now):
        def age(k):
            t = s.get("_t_" + k)
            return None if t is None else now - t

        est = s.get("estop")
        procs_n = len(self._proc)
        # ONE focal readout. Red only for a latched e-stop; amber when the
        # stack is down; otherwise plain, because normal needs no colour.
        if est:
            dpg.set_value("sf_state", "E-STOP LATCHED")
            dpg.configure_item("sf_state", color=BAD)
        elif procs_n == 0:
            dpg.set_value("sf_state", "STACK DOWN")
            dpg.configure_item("sf_state", color=WARN)
        elif est is False:
            dpg.set_value("sf_state", "RUNNING")
            dpg.configure_item("sf_state", color=INK)
        else:
            dpg.set_value("sf_state", "NO DATA")
            dpg.configure_item("sf_state", color=DIM)

        dm = s.get("deadman")
        if dm and len(dm) > 6:
            dpg.set_value("sf_deadman", "%s  %s / %s s"
                          % ("armed" if dm[0] else "off",
                             fmt(dm[2], "%.2f"), fmt(dm[6], "%.2f")))
            dpg.configure_item("sf_deadman",
                               color=INK if dm[0] else MUTED)
        else:
            dpg.set_value("sf_deadman", "--")
            dpg.configure_item("sf_deadman", color=DIM)

        obs = s.get("observer")
        dpg.set_value("sf_observer",
                      "present" if obs else
                      ("not confirmed" if obs is False else "--"))
        dpg.configure_item("sf_observer",
                           color=INK if obs else (WARN if obs is False
                                                  else DIM))
        cl = [s.get("ik_%s" % a_) for a_ in ARMS]
        vals = [d[5] for d in cl if d and len(d) > 5 and d[5] > -0.5]
        if vals:
            m = min(vals)
            dpg.set_value("sf_clear", fmt(m, "%.3f", "m"))
            dpg.configure_item("sf_clear",
                               color=BAD if m < 0.05 else
                               (WARN if m < 0.15 else INK))
        else:
            dpg.set_value("sf_clear", "--")
            dpg.configure_item("sf_clear", color=DIM)

        caps = []
        for a_ in ARMS:
            d = s.get("ik_%s" % a_)
            caps.append(fmt(d[8], "%.2f") if d and len(d) > 8 else "--")
        br0 = s.get("br_left") or s.get("br_right")
        dpg.set_value("sf_caps", "%s / %s"
                      % (caps[0], fmt(br0[1], "%.2f") if br0 else "--"))
        if self.frame_ms:
            f = sorted(self.frame_ms)
            dpg.set_value("sf_frame", "%s / %s ms"
                          % (fmt(f[len(f) // 2], "%.2f"),
                             fmt(f[int(0.95 * len(f))], "%.2f")))

        # ---- master arm
        meta = s.get("raw_meta", {})
        for a in ARMS:
            raw = s.get("raw_%s" % a)
            m = meta.get(a, {})
            dpg.set_value("m_hdr_%s" % a,
                          "frames %6d   %s" % (m.get("frames", 0),
                                               "no data" if not raw else ""))
            for i in range(JOINTS):
                if not raw or i >= len(raw):
                    dpg.set_value("m_val_%s_%d" % (a, i), "     no data")
                    dpg.configure_item("m_val_%s_%d" % (a, i), color=DIM)
                    continue
                v = raw[i]
                dpg.set_value("m_bar_%s_%d" % (a, i),
                              max(0.0, min(1.0, v / 360.0)))
                since = now - m["last_change"][i] if m else 0.0
                jumps = m["jumps"][i] if m else 0
                txt = ("%7.2f deg  upd %5d  last chg %5.1f s  jumps %3d"
                       % (v, m["updates"][i] if m else 0, since, jumps))
                dpg.set_value("m_val_%s_%d" % (a, i), txt)
                col = INK
                if since > STALE_S:
                    col = BAD           # looks live, is not
                elif jumps > 0:
                    col = WARN
                dpg.configure_item("m_val_%s_%d" % (a, i), color=col)
            if raw and len(raw) >= 13:
                ax, ay, az = raw[7], raw[8], raw[9]
                gx, gy, gz = raw[10], raw[11], raw[12]
                mag = math.sqrt(ax * ax + ay * ay + az * az)
                gate = abs(mag - 1.0) < 0.15
                dpg.set_value("m_imu_%s" % a,
                              "a = %+6.3f %+6.3f %+6.3f   |a| = %5.3f g   "
                              "gate %s (1.00 +/- 0.15)\n"
                              "g = %+7.2f %+7.2f %+7.2f deg/s"
                              % (ax, ay, az, mag,
                                 "OPEN" if gate else "SHUT", gx, gy, gz))
                dpg.configure_item("m_imu_%s" % a,
                                   color=GOOD if gate else WARN)
            else:
                dpg.set_value("m_imu_%s" % a, "no IMU data")
                dpg.configure_item("m_imu_%s" % a, color=DIM)
            fsr = s.get("fsr")
            ms = s.get("ms_%s" % a)
            clutch = ("engaged" if ms and ms[0] > 0.5 else "disengaged") \
                if ms else "—"
            if fsr and len(fsr) >= 4:
                f1, f2, b1, b2 = fsr[0], fsr[1], int(fsr[2]), int(fsr[3])
                mine = f1 if a == "left" else f2
                dpg.set_value("m_fsr_%s" % a,
                              "fsr %6.0f   deadband 250 | latch close 1200 | "
                              "open 400\nbtn1 %d  btn2 %d   clutch %s"
                              % (mine, b1, b2, clutch))
            else:
                dpg.set_value("m_fsr_%s" % a,
                              "no /master_fsr_buttons   clutch %s" % clutch)
            ar = age("raw_%s" % a)
            dpg.set_value("m_ser_%s" % a,
                          "frame age %s   distinct-update ages above"
                          % ("%.2f s" % ar if ar is not None else "—"))

        # ---- robot
        js = s.get("js")
        for a in ARMS:
            names, pos = js if js else ([], [])
            for i in range(JOINTS):
                nm = "%s_joint_%d" % (a, i + 1)
                if nm in names:
                    q = pos[names.index(nm)]
                    dpg.set_value("r_bar_%s_%d" % (a, i),
                                  max(0.0, min(1.0, (q + math.pi) /
                                               (2 * math.pi))))
                    wrap = int(q // (2 * math.pi)) if i in CONTINUOUS else 0
                    dpg.set_value("r_val_%s_%d" % (a, i),
                                  "%+7.3f rad  %s"
                                  % (q, ("wrap %+d" % wrap)
                                     if i in CONTINUOUS else "limited"))
                    dpg.configure_item("r_val_%s_%d" % (a, i),
                                       color=WARN if abs(wrap) else INK)
                else:
                    dpg.set_value("r_val_%s_%d" % (a, i), "  no data")
                    dpg.configure_item("r_val_%s_%d" % (a, i), color=DIM)
            d = s.get("ik_%s" % a)
            ee = s.get("ee_%s" % a)
            if d and len(d) > 5:
                succ, fail = d[0], d[1]
                rate = 100.0 * succ / max(succ + fail, 1)
                # ZERO ATTEMPTS IS NOT ZERO SUCCESS. Printing "IK 0.0%"
                # when nothing has been asked of the solver points the
                # operator at the solver instead of the missing input.
                attempts = succ + fail
                ik_txt = ("no requests yet" if attempts == 0
                          else "%5.1f%%   ok %d  fail %d"
                          % (rate, int(succ), int(fail)))
                dpg.set_value("r_ee_%s" % a,
                              "EE %s\nIK %s"
                              % ("%+.3f %+.3f %+.3f" % ee if ee
                                 else "no transform", ik_txt))
                c = d[5]
                dpg.set_value("r_clr_%s" % a, max(0.0, min(1.0, c / 0.5)))
                # THREE HONEST STAGES, NOT FIVE CLAIMED ONES.
                #
                # This used to label the bands "tangential" and "null-space",
                # which named avoidance strategies that DO NOT EXIST in this
                # code. Nothing computes a tangential velocity and nothing
                # projects a correction into the null space. What is actually
                # implemented, and all that is implemented, is:
                #
                #   1. collision-aware IK       (avoid_collisions=True)
                #   2. redundancy re-seeding    (6 samples on joint_3)
                #   3. the hard clearance floor (hold and publish nothing)
                #
                # A display label that names a stage which was never written
                # is worse than a plain number: it tells the operator a
                # mitigation is running when nothing is.
                zone = ("HARD FLOOR -- holding" if c < 0.05 else
                        "close" if c < 0.15 else
                        "margin" if c < 0.25 else "clear")
                dpg.set_value("r_clrtxt_%s" % a,
                              "%.3f m   [%s]   floor 0.05 | stages: "
                              "collision-aware IK, 6 redundancy re-seeds, "
                              "hard floor" % (c, zone))
                dpg.configure_item("r_clrtxt_%s" % a,
                                   color=(BAD if c < 0.05 else
                                          WARN if c < 0.15 else GOOD))
            else:
                dpg.set_value("r_ee_%s" % a, "no /ik_status")
                dpg.set_value("r_clrtxt_%s" % a, "—")
            br = s.get("br_%s" % a)
            if br and len(br) > 4:
                # br[4] is -1.0 when nothing has been measured. fmt()
                # renders that as "--" so it cannot read as a measurement.
                dpg.set_value("r_real_%s" % a,
                              "bridge %s   lag %s rad (trip 0.50)   "
                              "peak %s   min clearance %s"
                              % ("ENABLED" if br[0] > 0.5 else "disabled",
                                 fmt(br[2], "%.3f"), fmt(br[3], "%.3f"),
                                 fmt(br[4], "%.3f")))
                dpg.configure_item("r_real_%s" % a,
                                   color=GOOD if br[0] > 0.5 else DIM)
            else:
                dpg.set_value("r_real_%s" % a, "not connected")
                dpg.configure_item("r_real_%s" % a, color=DIM)

        tr = s.get("trial")
        dpg.set_value("trial_txt", str(tr)[:300] if tr else "no /trial_state")

    def _verdicts(self, s, now):
        states = self.pipeline_states()
        # FIRST-OUT ANNUNCIATION, from annunciator-panel practice: when a
        # fault cascades, only the ORIGINATING stage is shown at full
        # severity. Nine red rows are not nine problems -- they are one
        # problem and eight consequences, and colouring them alike is the
        # same "everything is red so nothing is urgent" failure as the old
        # safety bar.
        first_blocked = None
        for k, _l in PIPELINE:
            if states[k][0] == "STOPPED":
                first_blocked = k
                break
        for key, label in PIPELINE:
            st, detail, rate = states[key]
            if st == "STOPPED" and first_blocked and key != first_blocked:
                st = "WAITING"
                detail = ("Waiting on an earlier stage: %s. Fix that first."
                          % dict(PIPELINE)[first_blocked])
            dpg.set_value("pst_%s" % key, st)
            dpg.configure_item("pst_%s" % key, color=FG_OF.get(st, DIM))
            dpg.set_value("prt_%s" % key,
                          ("%.0f Hz" % rate) if rate else "")
            # never truncated -- the reason is the payload
            dpg.set_value("prs_%s" % key, detail or "")
            # the status colour is applied to the ROW, so status and reason
            # are one visual unit rather than text floating beside a box
            with dpg.theme() as rt:
                with dpg.theme_component(dpg.mvAll):
                    dpg.add_theme_color(dpg.mvThemeCol_ChildBg,
                                        TINT.get(st, ROW))
            dpg.bind_item_theme("row_%s" % key, rt)

        if self.sel_stage:
            st, detail, _r = states[self.sel_stage]
            dpg.set_value("stage_detail",
                          "%s\n%s\n%s" % (self.sel_stage, st,
                                           detail or "passing data"))

        # ---- BLOCKERS, reconciled with the pipeline ----
        #
        # The first version showed eight stages BLOCKED beside "no blockers
        # reported", which is a contradiction only because the two panels
        # answer different questions and one of them said so badly.
        # /blocking comes from BlockMonitor INSIDE the follower; if the
        # follower is not running there is no publisher, and that is NOT the
        # same as nothing blocking. It now says which.
        blk = s.get("blocking")
        blocked_stages = [l for k, l in PIPELINE
                          if states[k][0] == "STOPPED"]
        # The count must MATCH what is on screen. Saying "10 blocked" while
        # one row is red and nine are greyed as consequences is a number
        # contradicting its own display.
        n_down = max(len(blocked_stages) - 1, 0)
        recon = (("Signal path: stopped at \u201c%s\u201d, with %d later "
                  "stage(s) waiting on it" % (blocked_stages[0], n_down))
                 if blocked_stages else
                 "Signal path: every stage is passing data")
        if blk is None:
            dpg.set_value(
                "blk_src",
                "The safety-check reporter is not running, so it has nothing "
                "to say. That is NOT the same as nothing being wrong -- see "
                "the signal path on the left. %s." % recon)
            dpg.configure_item("blk_src", color=WARN if blocked_stages
                               else MUTED)
            dpg.set_value("blk_txt", "")
        else:
            dpg.set_value(
                "blk_src",
                "These are the safety checks the arm controller itself is "
                "applying. They do not cover whether data is reaching it -- "
                "the signal path on the left shows that. %s." % recon)
            dpg.configure_item("blk_src",
                               color=WARN if blocked_stages else MUTED)
            lines = []
            try:
                d = json.loads(blk)
                items = d if isinstance(d, list) else d.get("blocks", [])
                items = sorted(items, key=lambda b: -float(b.get("held_s", 0)))
                for b in items:
                    held = float(b.get("held_s", 0))
                    state = ("EXPIRED" if b.get("expired") else
                             ("ACTIVE" if b.get("active") else "clear"))
                    lines.append("%-16s %-8s %7.1f s\n    %s\n    -> %s"
                                 % (b.get("name", "?"), state, held,
                                    b.get("reason", ""),
                                    b.get("recovery", "")))
            except (ValueError, TypeError, AttributeError):
                lines.append(str(blk))
            dpg.set_value("blk_txt", "\n".join(lines) or
                          "No safety check is currently stopping the arm.")

        procs = self._proc
        total = sum(proc_rss_mb(p) for p, _ in procs)
        masters = [p for p, e in procs if e.endswith("/master_pose_node")]
        dpg.set_value("proc_warn",
                      ("TWO master_pose_node INSTANCES -- they SPLIT the "
                       "serial stream. Stop one now.")
                      if len(masters) > 1 else "")
        txt = []
        for pid, exe in procs:
            txt.append("%-34s %7d %8.0f MB"
                       % (os.path.basename(exe), pid, proc_rss_mb(pid)))
        if not txt:
            txt = ["no stack processes"]
        txt.append("")
        txt.append("total %s MB    disk %s GB"
                   % (fmt(total, "%.0f"), fmt(self._disk, "%.1f")))
        dpg.set_value("proc_txt", "\n".join(txt))
        ev = ["%s %-5s %-18s %s" % (t, sev.upper(), nd[:18], m[:96])
              for t, sev, nd, m in list(self.events)[:120]]
        dpg.set_value("ev_txt", "\n".join(ev))
        dpg.set_value("ev_tail", "\n".join(ev[:8]) or "no events yet")

    # ------------------------------------------------------------- close
    def shutdown(self):
        self.on_stop_all()
        self.ros.shutdown()


def main(argv=None):
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        sys.stderr.write("No display. Terminal fallback:  ros2 run "
                         "srl_teleop teleop_gui\n")
        return 2
    c = Console()
    c.build()
    c.log_event(("info", "console", "SRL console up — Dear PyGui under WSLg"))
    try:
        while dpg.is_dearpygui_running():
            c.tick()
            dpg.render_dearpygui_frame()
    finally:
        c.shutdown()
        dpg.destroy_context()
    return 0


if __name__ == "__main__":
    sys.exit(main())
