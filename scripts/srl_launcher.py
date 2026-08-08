#!/usr/bin/env python3
"""SRL LAUNCHER -- one window. Everything else is a button.

    python3 scripts/srl_launcher.py        (or: ros2 run srl_teleop launcher)

WSLg PROVIDES THE DISPLAY. Verified on this machine: DISPLAY=:0,
WAYLAND_DISPLAY=wayland-0, /mnt/wslg present, a real 1920x1200 screen, and a
Tk window created and torn down cleanly. RViz already opens a window, so if
that works this does.

`teleop_gui` stays as the TERMINAL fallback for SSH sessions with no display.
This is the primary interface.

WHY THE REFUSAL TO START A SECOND STACK IS THE MOST IMPORTANT FEATURE
--------------------------------------------------------------------
Two `master_pose_node` instances hold the same /dev/ttyACM* and the kernel
gives each byte to exactly ONE of them, so each parses fragments. Nothing
errors. On 2026-08-06 that invalidated a full day of channel measurements --
the raw topic ran at 68 Hz with 100% of rows distinct, where one healthy
stream gives ~15 Hz of distinct updates inside 50 Hz rows. The serial fd is
now flock'd so a second reader cannot open it, and this launcher refuses
before it gets that far, by name.

WHY LIVE CONTROLS USE PARAMETER CLIENTS AND NEVER `ros2 param set`
------------------------------------------------------------------
`ros2 param set` goes through the ros2 daemon, which hangs on this box and
has reported success it had not earned. Every control here calls the node's
SetParameters service directly and READS BACK the value, echoing `old -> new`.
A failed set is reported loudly rather than silently ignored.
"""
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
    import tkinter as tk
    from tkinter import messagebox, ttk
except ImportError as e:                                     # pragma: no cover
    sys.stderr.write(
        "tkinter is not available (%s).\n"
        "Under WSL this normally means WSLg is not running. Check that\n"
        "/mnt/wslg exists and DISPLAY is set; RViz opening a window is the\n"
        "quickest confirmation. Terminal fallback:  ros2 run srl_teleop "
        "teleop_gui\n" % e)
    raise SystemExit(2)

# Processes that constitute "a stack is already running".
STACK_MARKERS = ("teleop.launch", "move_group", "ros2_control_node",
                 "master_pose_node", "ik_follower_node")
BG = "#1e1e22"
FG = "#e6e6e6"
ACC = "#4a9eff"
OKC = "#3fb950"
WARN = "#d29922"
BAD = "#f85149"


# ----------------------------------------------------------------- helpers
def running_procs(markers=STACK_MARKERS):
    """[(pid, cmd)] for anything matching. Uses an explicit PID list, never a
    pkill pattern -- `pkill -f master_pose_node` matches the shell running it
    and has killed this project's own session twice."""
    out = []
    try:
        r = subprocess.run(["ps", "-eo", "pid,cmd"], capture_output=True,
                           text=True, timeout=10)
        for line in r.stdout.splitlines()[1:]:
            line = line.strip()
            if not line:
                continue
            pid, _, cmd = line.partition(" ")
            if "srl_launcher" in cmd or "ps -eo" in cmd:
                continue
            if any(m in cmd for m in markers):
                try:
                    out.append((int(pid), cmd.strip()))
                except ValueError:
                    pass
    except Exception:                                        # noqa: BLE001
        pass
    return out


def teensy_ports():
    import glob
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def have(path):
    return os.path.exists(os.path.join(WS, path))


def models_installed():
    v = os.path.join(WS, ".percep_venv", "bin", "python")
    if not os.path.exists(v):
        return False, "perception venv (.percep_venv) not created"
    try:
        r = subprocess.run([v, "-c", "import faster_whisper, ultralytics"],
                           capture_output=True, timeout=90)
        if r.returncode != 0:
            return False, "faster-whisper / ultralytics not importable"
    except Exception as e:                                   # noqa: BLE001
        return False, str(e)[:60]
    return True, ""


def quest_server_up(port=8766):
    import socket
    s = socket.socket()
    s.settimeout(0.4)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


class Job:
    """One managed subprocess, in its own process GROUP so the whole tree can
    be stopped. Output is pumped to a queue by a reader thread -- reading it
    on the Tk thread would freeze the window."""

    def __init__(self, name, argv, cwd=WS, shell=False):
        self.name = name
        self.argv = argv
        self.q = queue.Queue()
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        self.p = subprocess.Popen(
            argv, cwd=cwd, shell=shell, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        try:
            for line in self.p.stdout:
                self.q.put(line.rstrip("\n"))
        except Exception:                                    # noqa: BLE001
            pass
        self.q.put("[%s exited with %s]" % (self.name, self.p.poll()))

    @property
    def alive(self):
        return self.p.poll() is None

    def stop(self, grace=6.0):
        """SIGINT the GROUP first. SIGINT matters: the Kortex bridge closes
        its session on SIGINT and LEAKS it on SIGKILL, and the arm permits
        exactly one session, so a leak blocks the next run."""
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


# ------------------------------------------------------------- ROS bridge
class RosLink:
    """Live status + parameter writes, on a background thread.

    Degrades to 'no ROS' rather than preventing the window from opening: the
    launcher must be usable to START a stack, which by definition means
    running before one exists.
    """

    def __init__(self):
        self.ok = False
        self.state = {}
        self.lock = threading.Lock()
        self.node = None
        self._stop = threading.Event()
        self._done = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (DurabilityPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from std_msgs.msg import Bool, Float64MultiArray, String
            from sensor_msgs.msg import JointState
        except Exception as e:                               # noqa: BLE001
            with self.lock:
                self.state["_error"] = "rclpy unavailable: %s" % e
            return
        rclpy.init(args=None)
        n = Node("srl_launcher")
        self.node = n
        self.ok = True
        tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def put(k, v):
            with self.lock:
                self.state[k] = v
                self.state["_t_" + k] = time.monotonic()

        n.create_subscription(Bool, "/estop_state",
                              lambda m: put("estop", bool(m.data)), 10)
        n.create_subscription(String, "/master_channel_state",
                              lambda m: put("channels", m.data), tl)
        n.create_subscription(String, "/blocking_summary",
                              lambda m: put("blocking", m.data), 10)
        n.create_subscription(String, "/vr_state",
                              lambda m: put("vr", m.data), 10)
        n.create_subscription(String, "/autonomy_stage",
                              lambda m: put("autonomy", m.data), 10)
        n.create_subscription(JointState, "/joint_states",
                              lambda m: put("joints", len(m.name)), 20)
        for arm in ("left", "right"):
            n.create_subscription(
                Float64MultiArray, "/ik_status_%s" % arm,
                (lambda m, a=arm: put("ik_%s" % a, list(m.data))), 10)
            n.create_subscription(
                Float64MultiArray, "/master_status_%s" % arm,
                (lambda m, a=arm: put("ms_%s" % a, list(m.data))), 10)
            n.create_subscription(
                Float64MultiArray, "/bridge_status_%s" % arm,
                (lambda m, a=arm: put("bridge_%s" % a, list(m.data))), 10)
        self.estop_pub = n.create_publisher(Bool, "/estop", 10)
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
        try:
            n.destroy_node()
            rclpy.shutdown()
        except Exception:                                    # noqa: BLE001
            pass
        self._done.set()

    def get(self, k, default=None, max_age=3.0):
        with self.lock:
            t = self.state.get("_t_" + k)
            if t is None or (time.monotonic() - t) > max_age:
                return default
            return self.state.get(k, default)

    def estop(self):
        if not self.ok:
            return False, "no ROS connection"
        try:
            from std_msgs.msg import Bool
            self.estop_pub.publish(Bool(data=True))
            return True, "published /estop true"
        except Exception as e:                               # noqa: BLE001
            return False, str(e)

    def set_param(self, node_name, param, value):
        """Direct SetParameters + read-back. NEVER `ros2 param set`, which
        goes via the daemon, hangs on this box, and has reported success it
        had not earned."""
        if not self.ok:
            return False, "no ROS connection", None
        try:
            from rcl_interfaces.srv import GetParameters, SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue
            import rclpy
            cli = self.node.create_client(SetParameters,
                                          "%s/set_parameters" % node_name)
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, "%s has no set_parameters service" % node_name, None
            getc = self.node.create_client(GetParameters,
                                           "%s/get_parameters" % node_name)
            old = None
            if getc.wait_for_service(timeout_sec=1.0):
                gr = GetParameters.Request(names=[param])
                f = getc.call_async(gr)
                t0 = time.monotonic()
                while not f.done() and time.monotonic() - t0 < 2.0:
                    time.sleep(0.01)
                if f.done() and f.result() and f.result().values:
                    v = f.result().values[0]
                    old = v.double_value if v.type == 3 else (
                        v.integer_value if v.type == 2 else
                        (v.bool_value if v.type == 1 else v.string_value))
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type, pv.bool_value = 1, value
            elif isinstance(value, int):
                pv.type, pv.integer_value = 2, value
            elif isinstance(value, float):
                pv.type, pv.double_value = 3, value
            else:
                pv.type, pv.string_value = 4, str(value)
            req = SetParameters.Request(
                parameters=[Parameter(name=param, value=pv)])
            fut = cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.01)
            if not fut.done() or not fut.result():
                return False, "set_parameters did not return", old
            r = fut.result().results[0]
            if not r.successful:
                return False, r.reason or "rejected", old
            return True, "", old
        except Exception as e:                               # noqa: BLE001
            return False, str(e)[:80], None

    def shutdown(self, timeout=3.0):
        """Stop the spin loop AND wait for it. Calling Tk's destroy() while
        an rclpy executor is still spinning aborts the process with
        'terminate called without an active exception' -- which on a launcher
        would look exactly like a crash on quit."""
        self._stop.set()
        t0 = time.monotonic()
        while self.ok and self.node is not None and \
                time.monotonic() - t0 < timeout:
            time.sleep(0.05)
            if self._done.is_set():
                break


# ------------------------------------------------------------------- GUI
class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SRL launcher")
        self.geometry("1280x860")
        self.configure(bg=BG)
        self.jobs = {}
        # Worker threads NEVER touch Tk -- not even self.after(), which also
        # raises "main thread is not in main loop" when called off-thread.
        # They post here and _tick drains it on the main thread.
        self.ui_q = queue.Queue()
        self.ros = RosLink()
        self._build()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(500, self._tick)

    # ------------------------------------------------------------- layout
    def _build(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(14, 7))
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("TLabelframe", background=BG, foreground=FG)
        style.configure("TLabelframe.Label", background=BG, foreground=ACC)

        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=(8, 0))
        tk.Button(top, text="E-STOP", bg=BAD, fg="white",
                  font=("TkDefaultFont", 16, "bold"), height=2, width=14,
                  command=self.do_estop).pack(side="left")
        self.hdr = tk.Label(top, text="starting...", bg=BG, fg=FG,
                            justify="left", anchor="w",
                            font=("TkFixedFont", 10))
        self.hdr.pack(side="left", fill="x", expand=True, padx=12)
        tk.Button(top, text="STOP ALL", bg="#8b5cf6", fg="white",
                  font=("TkDefaultFont", 11, "bold"), height=2, width=12,
                  command=self.stop_all).pack(side="right")

        body = tk.PanedWindow(self, orient="horizontal", bg=BG,
                              sashwidth=6, bd=0)
        body.pack(fill="both", expand=True, padx=8, pady=8)

        left = tk.Frame(body, bg=BG)
        body.add(left, width=640)
        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)
        self._tab_teleop(nb)
        self._tab_autonomy(nb)
        self._tab_experiments(nb)
        self._tab_calib(nb)
        self._controls(left)

        right = tk.Frame(body, bg=BG)
        body.add(right)
        self._status(right)
        self._console(right)

    def _btn(self, parent, text, cmd, enabled=True, why="", **kw):
        b = tk.Button(parent, text=text, command=cmd, bg="#2d2d33", fg=FG,
                      activebackground=ACC, relief="flat", anchor="w",
                      padx=10, pady=6, **kw)
        if not enabled:
            b.configure(state="disabled", fg="#777")
            self._tip(b, why or "unavailable")
        return b

    def _tip(self, widget, text):
        """Hover explanation. Greying a button out without saying why is the
        same failure as a blocker with no name."""
        tipwin = {"w": None}

        def enter(_e):
            if tipwin["w"]:
                return
            x = widget.winfo_rootx() + 20
            y = widget.winfo_rooty() + widget.winfo_height() + 4
            w = tk.Toplevel(widget)
            w.wm_overrideredirect(True)
            w.wm_geometry("+%d+%d" % (x, y))
            tk.Label(w, text=text, bg="#111", fg="#eee", relief="solid",
                     borderwidth=1, justify="left", padx=6, pady=4,
                     wraplength=420).pack()
            tipwin["w"] = w

        def leave(_e):
            if tipwin["w"]:
                tipwin["w"].destroy()
                tipwin["w"] = None
        widget.bind("<Enter>", enter)
        widget.bind("<Leave>", leave)

    # -------------------------------------------------------------- tabs
    def _tab_teleop(self, nb):
        f = ttk.Frame(nb)
        nb.add(f, text="Teleoperation")
        o = tk.Frame(f, bg=BG)
        o.pack(fill="x", pady=(8, 4), padx=8)
        self.v_arm = tk.StringVar(value="both")
        self.v_pos = tk.StringVar(value="spherical")
        self.v_ori = tk.StringVar(value="fixed")
        self.v_deg = tk.StringVar(value="auto")
        for lbl, var, vals in (("arm", self.v_arm, ("left", "right", "both")),
                               ("position", self.v_pos, ("spherical", "fk")),
                               ("orientation", self.v_ori, ("fixed", "tilt")),
                               ("degraded", self.v_deg, ("auto", "on", "off"))):
            tk.Label(o, text=lbl, bg=BG, fg="#9aa").pack(side="left", padx=(6, 2))
            ttk.Combobox(o, textvariable=var, values=vals, width=9,
                         state="readonly").pack(side="left")
        vr_ok = quest_server_up() or have("scripts/start_vr.sh")
        for text, fn, en, why in (
                ("Sim only  —  no master, no real arms", self.run_sim, True, ""),
                ("Mannequin teleop  —  sim + master arm", self.run_mannequin,
                 True, ""),
                ("Mannequin → real  —  full cascade, gated",
                 self.run_cascade, True, ""),
                ("VR teleop  —  Quest over adb reverse", self.run_vr, vr_ok,
                 "scripts/start_vr.sh missing")):
            self._btn(f, text, fn, en, why).pack(fill="x", padx=8, pady=3)
        tk.Label(f, text="The cascade asks for confirmation before the real "
                         "arms move.", bg=BG, fg="#888",
                 wraplength=600, justify="left").pack(anchor="w", padx=10,
                                                      pady=(6, 0))

    def _tab_autonomy(self, nb):
        f = ttk.Frame(nb)
        nb.add(f, text="Autonomy")
        ok, why = models_installed()
        rows = [
            ("Mode 3  —  orientation assist", 3, False,
             "STUB: this /compute_ik plugin ignores OrientationConstraint "
             "(measured: identical IK success with and without it). Needs a "
             "constraint-aware plugin or explicit yaw sampling."),
            ("Mode 4  —  shared autonomy", 4, True, ""),
            ("Mode 5  —  supervised auto", 5, True, ""),
            ("Mode 6  —  full autonomy, voice", 6, ok,
             why or "perception models not installed"),
        ]
        for text, mode, en, w in rows:
            self._btn(f, text, (lambda m=mode: self.run_mode(m)),
                      en, w).pack(fill="x", padx=8, pady=3)
        note = ("Mode 6 is not participant-ready: detection rate at working "
                "distance is unmeasured. Autonomy is speed-capped below "
                "teleop (0.15 vs 0.60 rad/s) because nobody is watching.")
        tk.Label(f, text=note, bg=BG, fg="#888", wraplength=600,
                 justify="left").pack(anchor="w", padx=10, pady=(8, 0))

    def _tab_experiments(self, nb):
        f = ttk.Frame(nb)
        nb.add(f, text="Experiments")
        o = tk.Frame(f, bg=BG)
        o.pack(fill="x", pady=(8, 4), padx=8)
        self.v_cond = tk.StringVar(value="direct")
        self.v_scen = tk.StringVar(value="S1")
        self.v_pid = tk.StringVar(value="P00")
        tk.Label(o, text="condition", bg=BG, fg="#9aa").pack(side="left")
        ttk.Combobox(o, textvariable=self.v_cond, width=9, state="readonly",
                     values=("direct", "assisted", "shared")).pack(side="left",
                                                                   padx=4)
        tk.Label(o, text="scenario", bg=BG, fg="#9aa").pack(side="left")
        ttk.Combobox(o, textvariable=self.v_scen, width=6, state="readonly",
                     values=("S1", "S2", "S3", "S4")).pack(side="left", padx=4)
        tk.Label(o, text="participant", bg=BG, fg="#9aa").pack(side="left")
        tk.Entry(o, textvariable=self.v_pid, width=8).pack(side="left", padx=4)
        g = tk.Frame(f, bg=BG)
        g.pack(fill="x", padx=8, pady=4)
        tasks = [("T3 rigid carry", "t3", True, ""),
                 ("T6 compliant carry", "t6", True, ""),
                 ("T7 pursuit tracking", "t7", True, ""),
                 ("T5 handover to wearer", "t5", True, ""),
                 ("T2 hold and fill", "t2", False,
                  "Feasible only with the redesigned side-handle container; "
                  "not in the 75 min session (it would displace T5).")]
        for i, (text, key, en, why) in enumerate(tasks):
            b = self._btn(g, text, (lambda k=key: self.run_task(k)), en, why)
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self._btn(f, "Run full experiment session  (P0 → P7)",
                  self.run_session).pack(fill="x", padx=8, pady=(10, 3))
        self._btn(f, "Pilot (offline, no hardware)",
                  self.run_pilot).pack(fill="x", padx=8, pady=3)

    def _tab_calib(self, nb):
        f = ttk.Frame(nb)
        nb.add(f, text="Calibration & diagnostics")
        z = tk.Frame(f, bg=BG)
        z.pack(fill="x", padx=8, pady=(8, 2))
        tk.Label(z, text="Capture zeros:", bg=BG, fg=FG).pack(side="left")
        for arm in ("left", "right"):
            self._btn(z, arm, (lambda a=arm: self.run_job(
                "zero_%s" % a, ["ros2", "run", "srl_teleop", "capture_zero",
                                "--ros-args", "-p", "arm:=%s" % arm]))
                      ).pack(side="left", padx=4)
        for text, key, argv, need in (
                ("Check channels  —  3 min repair acceptance test",
                 "channels", ["bash", "scripts/check_channels.sh"],
                 "scripts/check_channels.sh"),
                ("Trajectory capture", "capture",
                 ["ros2", "run", "srl_experiments", "record_trajectories"],
                 None),
                ("Measure workspace", "workspace",
                 ["python3", "scripts/measure_workspace.py"],
                 "scripts/measure_workspace.py"),
                ("Network check  —  arms at 192.168.1.9/.10", "net",
                 ["bash", "scripts/check_arm_network.sh"],
                 "scripts/check_arm_network.sh"),
                ("Recover  —  reset to a known-good state", "recover",
                 ["bash", "scripts/recover.sh"], "scripts/recover.sh"),
                ("Button mapping check  —  30 s, read-only", "buttons",
                 ["ros2", "run", "srl_teleop", "check_buttons"], None)):
            en = (need is None) or have(need)
            self._btn(f, text, (lambda k=key, a=argv: self.run_job(k, a)),
                      en, "%s not found" % need).pack(fill="x", padx=8, pady=3)

    # ---------------------------------------------------------- controls
    def _controls(self, parent):
        lf = ttk.Labelframe(parent, text="Live controls  (applied immediately)")
        lf.pack(fill="x", pady=(8, 0))
        self.sliders = {}
        rows = [
            ("sim speed  max_vel_rad_s", "/ik_follower_left", "max_vel_rad_s",
             0.05, 1.0, 0.6),
            ("motion scale  left", "/master_pose_node", "left_scale",
             0.1, 2.0, 1.0),
            ("motion scale  right", "/master_pose_node", "right_scale",
             0.1, 2.0, 1.0),
            ("EMA alpha", "/master_pose_node", "ema_alpha", 0.05, 1.0, 0.3),
            ("accel gate (g)", "/master_pose_node", "accel_gate_g",
             0.05, 0.5, 0.15),
            ("max_step_rad", "/ik_follower_left", "max_step_rad",
             0.02, 0.6, 0.35),
            ("lag trip (rad)", "/sim_to_real_bridge_left", "lag_trip_rad",
             0.1, 1.0, 0.5),
            ("preview delay (s)", "/sim_to_real_bridge_left",
             "preview_delay_s", 0.2, 3.0, 1.0),
        ]
        for i, (label, node, param, lo, hi, init) in enumerate(rows):
            r = tk.Frame(lf, bg=BG)
            r.pack(fill="x", padx=6, pady=1)
            tk.Label(r, text=label, bg=BG, fg=FG, width=24, anchor="w",
                     font=("TkFixedFont", 9)).pack(side="left")
            var = tk.DoubleVar(value=init)
            s = tk.Scale(r, from_=lo, to=hi, resolution=0.01,
                         orient="horizontal", variable=var, bg=BG, fg=FG,
                         highlightthickness=0, troughcolor="#333",
                         showvalue=True, length=250)
            s.pack(side="left", fill="x", expand=True)
            tk.Button(r, text="apply", bg="#2d2d33", fg=FG, relief="flat",
                      command=(lambda n=node, p=param, v=var:
                               self.apply_param(n, p, v.get()))
                      ).pack(side="left", padx=4)
            self.sliders[param] = var

    def apply_param(self, node, param, value):
        """Runs off the Tk thread: a service call that blocks the GUI is how
        the terminal console ended up appearing hung exactly when it was
        being used to find out why something was hung."""
        def work():
            ok, err, old = self.ros.set_param(node, param, float(value))
            if ok:
                self.log("[PARAM OK] %s %s: %s -> %.4f"
                         % (node, param,
                            ("%.4f" % old) if isinstance(old, float) else old,
                            value))
            else:
                self.log("[PARAM FAILED] %s %s -> %.4f : %s"
                         % (node, param, value, err))
                self.ui_q.put((
                    "error", "Parameter not applied",
                    "%s %s could not be set:\n\n%s\n\nThe value on the "
                    "robot is UNCHANGED." % (node, param, err)))
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------ status
    def _status(self, parent):
        lf = ttk.Labelframe(parent, text="Live status")
        lf.pack(fill="x")
        self.status = tk.Text(lf, height=17, bg="#151519", fg=FG,
                              font=("TkFixedFont", 10), relief="flat",
                              wrap="none")
        self.status.pack(fill="both", expand=True, padx=4, pady=4)
        for tag, col in (("ok", OKC), ("warn", WARN), ("bad", BAD),
                         ("hdr", ACC), ("dim", "#777")):
            self.status.tag_configure(tag, foreground=col)

    def _console(self, parent):
        lf = ttk.Labelframe(parent, text="Output")
        lf.pack(fill="both", expand=True, pady=(8, 0))
        wrap = tk.Frame(lf, bg=BG)
        wrap.pack(fill="both", expand=True)
        self.console = tk.Text(wrap, bg="#0d0d10", fg="#cfcfcf",
                               font=("TkFixedFont", 9), relief="flat",
                               wrap="none")
        sb = tk.Scrollbar(wrap, command=self.console.yview)
        self.console.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)

    def log(self, line):
        """THREAD-SAFE. Tk may only be touched from the thread running the
        mainloop; calling this from a worker raises "main thread is not in
        main loop" and kills the worker mid-flight. That is exactly what
        happened to apply_param: the parameter echo AND the failure dialog
        both vanished, so a failed `ros2 param` write looked like a silent
        success -- the precise failure mode this GUI exists to prevent."""
        if threading.current_thread() is not threading.main_thread():
            self.ui_q.put(("log", line))
            return
        self._log_main(line)

    def _log_main(self, line):
        self.console.insert("end", line + "\n")
        self.console.see("end")
        if float(self.console.index("end-1c").split(".")[0]) > 3000:
            self.console.delete("1.0", "500.0")

    # ------------------------------------------------------------ launch
    def preflight(self, need_teensy, need_real=False):
        """Show what failed rather than starting and dying."""
        fails = []
        procs = running_procs()
        if procs:
            fails.append(
                "A stack is ALREADY RUNNING (%d process(es), e.g. pid %d).\n"
                "     Two master_pose_node instances split the serial stream "
                "and\n     invalidated a full day of measurements. Use STOP "
                "ALL first." % (len(procs), procs[0][0]))
        if need_teensy:
            ports = teensy_ports()
            if not ports:
                fails.append(
                    "No /dev/ttyACM* or /dev/ttyUSB* present. Attach the "
                    "Teensy:\n     usbipd attach --wsl --hardware-id 16c0:0483")
        if self.ros.get("estop") is True:
            fails.append("E-STOP IS LATCHED. Reset it before starting:\n"
                         "     ros2 service call /estop_reset "
                         "std_srvs/srv/Trigger {}")
        if need_real:
            import subprocess as sp
            r = sp.run(["ping", "-c1", "-W2", "192.168.1.10"],
                       capture_output=True)
            if r.returncode != 0:
                fails.append("Left arm 192.168.1.10 is not answering. This "
                             "machine may be off the lab network.")
        return fails

    def guarded(self, name, argv, need_teensy=False, need_real=False,
                shell=False, confirm=None):
        fails = self.preflight(need_teensy, need_real)
        if fails:
            messagebox.showerror(
                "Preflight failed — not starting",
                "\n\n".join("• " + f for f in fails))
            self.log("[PREFLIGHT FAILED] %s" % name)
            for f in fails:
                self.log("   " + f.replace("\n", "\n   "))
            return
        if confirm and not messagebox.askyesno("Confirm", confirm):
            self.log("[CANCELLED] %s" % name)
            return
        self.run_job(name, argv, shell=shell)

    def run_job(self, name, argv, shell=False):
        if name in self.jobs and self.jobs[name].alive:
            messagebox.showinfo("Already running",
                                "%s is already running." % name)
            return
        self.log("\n=== START %s ===\n$ %s"
                 % (name, argv if shell else " ".join(argv)))
        try:
            self.jobs[name] = Job(name, argv, shell=shell)
        except Exception as e:                               # noqa: BLE001
            messagebox.showerror("Could not start", "%s\n\n%s" % (name, e))
            self.log("[FAILED TO START] %s: %s" % (name, e))

    # -------- teleop
    def run_sim(self):
        self.guarded("sim", ["ros2", "launch", "srl_teleop",
                             "teleop.launch.py", "gate:=false",
                             "use_rviz:=false", "serial_port:=/dev/null"])

    def run_mannequin(self):
        self.guarded("mannequin",
                     ["ros2", "launch", "srl_teleop", "teleop.launch.py",
                      "gate:=false",
                      "position_mode:=%s" % self.v_pos.get(),
                      "orientation_mode:=%s" % self.v_ori.get(),
                      "degraded_mode:=%s" % self.v_deg.get()],
                     need_teensy=True)

    def run_cascade(self):
        self.guarded("cascade",
                     ["bash", "scripts/start_real.sh",
                      "arm:=%s" % self.v_arm.get()],
                     need_teensy=True, need_real=True,
                     confirm="This drives the REAL arms.\n\nThe sim stack "
                             "must already be running, the arms will home, "
                             "and the bridge will replay sim motion onto "
                             "hardware.\n\nContinue?")

    def run_vr(self):
        self.guarded("vr", ["bash", "scripts/start_vr.sh"])

    # -------- autonomy
    def run_mode(self, mode):
        self.guarded("mode%d" % mode,
                     ["ros2", "run", "srl_autonomy", "autonomy_executive",
                      "--ros-args", "-p", "mode:=%d" % mode])

    # -------- experiments
    def run_task(self, key):
        self.run_job("task_%s" % key,
                     ["bash", "scripts/run_experiment.sh", key,
                      "--participant", self.v_pid.get(),
                      "--condition", self.v_cond.get(),
                      "--scenario", self.v_scen.get()])

    def run_session(self):
        if not messagebox.askyesno(
                "Full session",
                "Runs P0 → P7 in order (~75 min).\n\nParticipant %s.\n\n"
                "Continue?" % self.v_pid.get()):
            return
        self.run_job("session", ["bash", "scripts/run_experiment.sh",
                                 "session", "--participant",
                                 self.v_pid.get()])

    def run_pilot(self):
        self.run_job("pilot", ["python3", "scripts/pilot_bimanual.py"])

    # ------------------------------------------------------------ global
    def do_estop(self):
        ok, msg = self.ros.estop()
        self.log("[E-STOP] %s" % msg)
        if not ok:
            messagebox.showwarning(
                "E-stop not published",
                "%s\n\nThe software e-stop could not be sent. USE THE "
                "PHYSICAL E-STOP." % msg)

    def stop_all(self):
        n = 0
        for name, job in list(self.jobs.items()):
            if job.alive:
                self.log("[STOP] %s" % name)
                job.stop()
                n += 1
        left = running_procs()
        if left:
            self.log("[STOP] sweeping %d orphan(s) by explicit PID" % len(left))
            for pid, _ in left:
                try:
                    os.kill(pid, signal.SIGINT)
                except OSError:
                    pass
            time.sleep(2.0)
            for pid, _ in running_procs():
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        self.log("[STOP] %d managed job(s) stopped" % n)

    def on_close(self):
        """Stop the arm, close the Kortex session, kill the group. A leaked
        session blocks the next run because the arm permits exactly one."""
        if any(j.alive for j in self.jobs.values()) or running_procs():
            if not messagebox.askyesno(
                    "Quit",
                    "Stacks are still running.\n\nQuitting will stop the arm, "
                    "close the Kortex session and kill every managed "
                    "process.\n\nQuit?"):
                return
        self.do_estop_quiet()
        self.stop_all()
        self.ros.shutdown()
        self.destroy()

    def destroy(self):
        # Belt and braces: whatever path gets here, ROS goes down first.
        try:
            self.ros.shutdown(timeout=2.0)
        except Exception:                                    # noqa: BLE001
            pass
        super().destroy()

    def do_estop_quiet(self):
        try:
            self.ros.estop()
        except Exception:                                    # noqa: BLE001
            pass

    # -------------------------------------------------------------- tick
    def _tick(self):
        # Drain worker->UI messages first, on the main thread.
        for _ in range(100):
            try:
                kind, *rest = self.ui_q.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._log_main(rest[0])
            elif kind == "error":
                self._log_main("[ERROR] %s: %s" % (rest[0], rest[1]))
                messagebox.showerror(rest[0], rest[1])
        for job in list(self.jobs.values()):
            for _ in range(200):
                try:
                    self.log(job.q.get_nowait())
                except queue.Empty:
                    break
        self._refresh_status()
        self.after(400, self._tick)

    def _put(self, text, tag="dim"):
        self.status.insert("end", text + "\n", tag)

    def _refresh_status(self):
        s = self.status
        s.configure(state="normal")
        s.delete("1.0", "end")
        procs = running_procs()
        est = self.ros.get("estop")
        self._put("STACK", "hdr")
        if procs:
            self._put("  %d process(es) running" % len(procs), "ok")
        else:
            self._put("  nothing running", "dim")
        self._put("  ROS link: %s" % ("connected" if self.ros.ok else "none"),
                  "ok" if self.ros.ok else "warn")

        self._put("\nSAFETY", "hdr")
        if est is None:
            self._put("  e-stop: unknown (no /estop_state)", "warn")
        elif est:
            self._put("  E-STOP LATCHED", "bad")
        else:
            self._put("  e-stop clear", "ok")
        blk = self.ros.get("blocking")
        self._put("  ACTIVE BLOCKERS", "hdr")
        if blk is None:
            self._put("    (no /blocking_summary)", "dim")
        elif "none" in str(blk).lower() or str(blk).strip() in ("", "{}"):
            self._put("    none", "ok")
        else:
            self._put("    " + str(blk)[:180], "bad")

        self._put("\nCHANNELS", "hdr")
        ch = self.ros.get("channels")
        if ch:
            for part in str(ch).split("|"):
                p = part.strip()
                tag = "bad" if "DEGRADED" in p else (
                    "warn" if "frozen" in p and "frozen j-" not in p else "ok")
                self._put("  " + p[:110], tag)
        else:
            self._put("  (master not publishing)", "dim")

        self._put("\nFOLLOWERS", "hdr")
        for arm in ("left", "right"):
            d = self.ros.get("ik_%s" % arm)
            if not d:
                self._put("  %-5s no /ik_status" % arm, "dim")
                continue
            succ, fail = (d[0], d[1]) if len(d) > 1 else (0, 0)
            clr = d[5] if len(d) > 5 else float("nan")
            vel = d[8] if len(d) > 8 else float("nan")
            casc = bool(d[9]) if len(d) > 9 else False
            rate = 100.0 * succ / max(succ + fail, 1)
            self._put("  %-5s IK %5.1f%%  clearance %.3f m  max_vel %.2f%s"
                      % (arm, rate, clr, vel, "  CASCADE" if casc else ""),
                      "ok" if rate > 90 else "warn")

        self._put("\nREAL ARM", "hdr")
        any_bridge = False
        for arm in ("left", "right"):
            d = self.ros.get("bridge_%s" % arm)
            if not d:
                continue
            any_bridge = True
            en = bool(d[0])
            self._put("  %-5s bridge %s  lag %.3f rad  peak %.3f"
                      % (arm, "ENABLED" if en else "disabled",
                         d[2] if len(d) > 2 else float("nan"),
                         d[3] if len(d) > 3 else float("nan")),
                      "ok" if en else "dim")
        if not any_bridge:
            self._put("  not connected", "dim")

        vr = self.ros.get("vr")
        if vr:
            self._put("\nVR", "hdr")
            self._put("  " + str(vr)[:160], "dim")
        au = self.ros.get("autonomy")
        if au:
            self._put("\nAUTONOMY", "hdr")
            self._put("  " + str(au)[:160], "dim")
        s.configure(state="disabled")

        jobs = sum(1 for j in self.jobs.values() if j.alive)
        self.hdr.configure(
            text="jobs running: %d      stack processes: %d      "
                 "e-stop: %s\nWSLg display %s      workspace %s"
                 % (jobs, len(procs),
                    "LATCHED" if est else ("clear" if est is False else "?"),
                    os.environ.get("DISPLAY", "?"), WS))


def main(argv=None):
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        sys.stderr.write(
            "No DISPLAY and no WAYLAND_DISPLAY.\n"
            "This launcher needs a graphical session (WSLg provides one).\n"
            "Over SSH use the terminal console instead:\n"
            "    ros2 run srl_teleop teleop_gui\n")
        return 2
    app = Launcher()
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
