#!/usr/bin/env python3
"""teleop_gui.py — one terminal that answers "what is the rig doing?".

    ros2 run srl_teleop teleop_gui

Plain curses. No Qt, no browser, no X server: it works over SSH and inside
WSL, which is where this rig actually lives.

WHY IT EXISTS. The state needed to debug a run is spread over a dozen topics
and three log streams, and the historically expensive failures on this rig all
looked the same from outside — the arm does not move and nothing says why.
The BLOCKERS panel is the answer to that question and is placed first for
exactly that reason.

TOPICS ONLY — it never opens the serial port. `master_pose_node` resolves the
port once in its constructor and holds it; a second reader would either fail
or, worse, steal frames. Everything here is read from ROS.

POT COLOURING IS MODE-AWARE, and this is not cosmetic. In spherical mode only
j1/j2/j4 feed position, so a dead j5 is irrelevant. Reddening it anyway trains
you to ignore red, which is how a real dropout gets missed. A channel the
current mode does not consume can never be red here.

    green   live, and this mode uses it
    yellow  railed/clamped, or dead but UNUSED by this mode
    red     dropout on a channel this mode actually consumes

CONTROLS are applied through parameter CLIENTS, never `ros2 param set` — that
goes via the daemon, which hangs on this box and fails silently. Every change
is echoed old -> new in the event log, and a failure is reported loudly rather
than leaving you to wonder whether it took.

Keys are listed in the footer; `?` toggles the full help pane.
"""
import curses
import json
import queue
import os
import math
import sys
import threading
import time
from collections import deque

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

import tf2_ros

ARMS = ("left", "right")

# Which pot channels each position mode actually CONSUMES. Everything else is
# yellow-at-worst, never red.
MODE_CHANNELS = {
    "spherical": (1, 2, 4),
    "fk": (1, 2, 3, 4, 5),
}

# Channel health from the 2026-08-06 capture, measured over DISTINCT sensor
# updates. Used only to distinguish "known bad" (yellow) from "was fine and
# just dropped out" (red).
#
# CORRECTED: left j7 is NOT railed -- it is the cleanest channel on the left
# arm (0% dropouts, 0% implausible jumps). Its range is only 26 deg, so it is
# stiff or mechanically restricted, but it works. The old "railed" verdict
# came from the recorder oversampling a 15 Hz sensor at 50 Hz, which made
# most adjacent rows identical and the spread read as zero.
KNOWN_BAD = {
    "left": {2: "INCOHERENT 5% jumps", 3: "INCOHERENT 8% jumps",
             4: "INCOHERENT 6% jumps, 22% dropouts",
             5: "INCOHERENT 13% jumps, 40% dropouts",
             6: "INCOHERENT 14% jumps, 70% dropouts",
             1: "2.2% dropouts"},
    "right": {3: "INCOHERENT 14% jumps, 80% dropouts",
              4: "INCOHERENT 5% jumps",
              5: "DEAD 100% dropouts", 7: "DEAD 100% dropouts"},
}

C_OK, C_WARN, C_BAD, C_HEAD, C_DIM, C_HI = 1, 2, 3, 4, 5, 6


def _fmt(v, w=7, p=2):
    try:
        if v is None or (isinstance(v, float) and not math.isfinite(v)):
            return "-".rjust(w)
        return ("%*.*f" % (w, p, v))
    except (TypeError, ValueError):
        return "-".rjust(w)


class GuiNode(Node):
    """All subscriptions and all parameter clients. No curses in here."""

    def __init__(self):
        super().__init__("teleop_gui")
        self.lock = threading.Lock()
        self.raw = {a: None for a in ARMS}
        self.status = {a: None for a in ARMS}
        self.pose = {a: None for a in ARMS}
        self.ik = {a: None for a in ARMS}
        self.caps = {a: {} for a in ARMS}
        self.fsr = None
        self.estop = None
        self.deadman = {}
        self.recovery = {}
        self.blocking = {}
        self.session = {}
        self.highlevel = {a: None for a in ARMS}
        self.real_js = None
        self.homing_done = {a: None for a in ARMS}
        self.events = deque(maxlen=200)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        for a in ARMS:
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                self._set("raw", a), 20)
            self.create_subscription(
                Float64MultiArray, f"/master_status_{a}",
                self._set("status", a), 20)
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}", self._set_msg("pose", a), 20)
            self.create_subscription(
                Float64MultiArray, f"/ik_status_{a}", self._set("ik", a), 10)
            self.create_subscription(
                String, f"/master_capability_{a}", self._set_json("caps", a), 10)
            self.create_subscription(
                Float64MultiArray, f"/highlevel_status_{a}",
                self._set("highlevel", a), 10)
            self.create_subscription(
                Bool, f"/homing_done_{a}", self._set_bool("homing_done", a), 10)

        self.create_subscription(Float64MultiArray, "/master_fsr_buttons",
                                 self._set_plain("fsr"), 20)
        self.create_subscription(Bool, "/estop_state",
                                 self._set_bool_plain("estop"), 10)
        self.create_subscription(String, "/estop_deadman",
                                 self._set_json_plain("deadman"), 10)
        self.create_subscription(String, "/recovery_state",
                                 self._set_json_plain("recovery"), 10)
        self.create_subscription(String, "/real/session_state",
                                 self._set_json_plain("session"), 10)
        self.create_subscription(String, "/blocking", self._on_block, 30)
        self.create_subscription(JointState, "/real/joint_states",
                                 self._set_plain_js("real_js"), 10)

        self._param_clients = {}
        self.estop_cli = self.create_client(Trigger, "/estop")
        self.reset_cli = self.create_client(Trigger, "/estop_reset")

    # ------------------------------------------------------------ plumbing
    def _set(self, field, arm):
        def cb(m):
            with self.lock:
                getattr(self, field)[arm] = list(m.data)
        return cb

    def _set_msg(self, field, arm):
        """Store the MESSAGE, not m.data.

        PoseStamped has no `.data`; the generic list-setter raised
        AttributeError inside the executor the moment a real master pose
        arrived. It stayed hidden while nothing was publishing, which is
        exactly why the GUI test must run with the stack live.
        """
        def cb(m):
            with self.lock:
                getattr(self, field)[arm] = m
        return cb

    def _set_bool(self, field, arm):
        def cb(m):
            with self.lock:
                getattr(self, field)[arm] = bool(m.data)
        return cb

    def _set_json(self, field, arm):
        def cb(m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            with self.lock:
                getattr(self, field)[arm] = d
        return cb

    def _set_plain(self, field):
        def cb(m):
            with self.lock:
                setattr(self, field, list(m.data))
        return cb

    def _set_plain_js(self, field):
        def cb(m):
            with self.lock:
                setattr(self, field, dict(zip(m.name, m.position)))
        return cb

    def _set_bool_plain(self, field):
        def cb(m):
            with self.lock:
                setattr(self, field, bool(m.data))
        return cb

    def _set_json_plain(self, field):
        def cb(m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            with self.lock:
                setattr(self, field, d)
        return cb

    def _on_block(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        d["_rx"] = time.monotonic()
        with self.lock:
            self.blocking[d.get("unit", "?")] = d

    def ee(self, arm):
        try:
            t = self.tf_buf.lookup_transform(
                "world", f"{arm}_end_effector_link", rclpy.time.Time())
            v = t.transform.translation
            return (v.x, v.y, v.z)
        except Exception:                                        # noqa: BLE001
            return None

    def log(self, msg, bad=False):
        self.events.appendleft((time.strftime("%H:%M:%S"), msg, bad))

    # ------------------------------------------------------- param clients
    def _client(self, node_name, srv, typ):
        key = (node_name, srv)
        if key not in self._param_clients:
            self._param_clients[key] = self.create_client(
                typ, "%s/%s" % (node_name.rstrip("/"), srv))
        return self._param_clients[key]

    def get_param(self, node_name, name, timeout=2.0):
        cli = self._client(node_name, "get_parameters", GetParameters)
        if not cli.wait_for_service(timeout_sec=timeout):
            return None, "%s has no parameter service (is it running?)" % node_name
        req = GetParameters.Request()
        req.names = [name]
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        r = fut.result()
        if r is None or not r.values:
            return None, "no response from %s" % node_name
        v = r.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE:
            return v.double_value, None
        if v.type == ParameterType.PARAMETER_INTEGER:
            return v.integer_value, None
        if v.type == ParameterType.PARAMETER_BOOL:
            return v.bool_value, None
        if v.type == ParameterType.PARAMETER_STRING:
            return v.string_value, None
        return None, "unset or unsupported type on %s" % name

    def set_param(self, node_name, name, value, timeout=3.0):
        """Set a parameter and REPORT whether it took.

        Uses the parameter service directly. Shelling out to `ros2 param set`
        goes through the ros2 daemon, which hangs on this box and returns
        success it has not earned -- a fault that already cost one debugging
        session, so it is not repeated here.
        """
        old, err = self.get_param(node_name, name, timeout=timeout)
        cli = self._client(node_name, "set_parameters", SetParameters)
        if not cli.wait_for_service(timeout_sec=timeout):
            self.log("FAILED %s %s: no parameter service (node running?)"
                     % (node_name, name), bad=True)
            return False
        p = Parameter()
        p.name = name
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, value
        elif isinstance(value, int):
            pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, value
        elif isinstance(value, float):
            pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, value
        else:
            pv.type, pv.string_value = ParameterType.PARAMETER_STRING, str(value)
        p.value = pv
        req = SetParameters.Request()
        req.parameters = [p]
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        r = fut.result()
        if r is None:
            self.log("FAILED %s %s: no response in %.0fs"
                     % (node_name, name, timeout), bad=True)
            return False
        res = r.results[0]
        if not res.successful:
            self.log("REJECTED %s %s: %s"
                     % (node_name, name, res.reason or "no reason given"),
                     bad=True)
            return False
        # CONFIRM by reading back. "successful" only means the node accepted
        # the message.
        new, _ = self.get_param(node_name, name, timeout=timeout)
        self.log("%s %s: %s -> %s" % (node_name.lstrip("/"), name,
                                      _short(old), _short(new)))
        return True

    def call_trigger(self, cli, label, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            self.log("FAILED %s: service unavailable" % label, bad=True)
            return False
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        r = fut.result()
        if r is None:
            self.log("FAILED %s: no response" % label, bad=True)
            return False
        self.log("%s: %s" % (label, r.message or ("ok" if r.success else "failed")),
                 bad=not r.success)
        return r.success


def _short(v):
    if v is None:
        return "?"
    if isinstance(v, float):
        return "%.4g" % v
    return str(v)


class Gui:
    def __init__(self, stdscr, node):
        self.s = stdscr
        self.n = node
        # CONTROL ACTIONS RUN ON A WORKER THREAD.
        # Every control goes through a parameter or service client, and those
        # block: wait_for_service plus the call is seconds when the target
        # node is not running. Doing that inline froze the 5 Hz redraw for as
        # long as it took, so the console appeared hung exactly when you were
        # trying to find out why something was hung. The queue keeps the
        # display live and the event log reports each action as it lands.
        self._work = queue.Queue()
        self._worker = threading.Thread(target=self._pump, daemon=True)
        self._worker.start()
        self.help = False
        self.sel_arm = 0          # which arm the per-arm keys act on
        self.status_line = ""
        # curs_set is COSMETIC and is not supported by every terminfo entry.
        # Under TERM=dumb it raises _curses.error and took the whole GUI down
        # on launch. Nothing here depends on the cursor being hidden.
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.s.nodelay(True)
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
            curses.init_pair(C_WARN, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_BAD, curses.COLOR_RED, -1)
            curses.init_pair(C_HEAD, curses.COLOR_CYAN, -1)
            curses.init_pair(C_DIM, curses.COLOR_WHITE, -1)
            curses.init_pair(C_HI, curses.COLOR_MAGENTA, -1)

    def _pump(self):
        while True:
            fn = self._work.get()
            if fn is None:
                return
            try:
                fn()
            except Exception as e:                              # noqa: BLE001
                self.n.log("control action failed: %s" % e, bad=True)

    def submit(self, fn):
        # Bound the queue so holding a key down cannot build a backlog that
        # replays for a minute after you stop.
        if self._work.qsize() < 8:
            self._work.put(fn)
        else:
            self.n.log("control busy - key ignored", bad=True)

    # ------------------------------------------------------------ drawing
    def _a(self, pair, bold=False):
        att = curses.color_pair(pair) if curses.has_colors() else 0
        return att | (curses.A_BOLD if bold else 0)

    def put(self, y, x, text, pair=C_DIM, bold=False):
        h, w = self.s.getmaxyx()
        if y < 0 or y >= h or x >= w:
            return
        try:
            self.s.addnstr(y, x, text, max(0, w - x - 1), self._a(pair, bold))
        except curses.error:
            pass

    def mode(self):
        with self.n.lock:
            for a in ARMS:
                c = self.n.caps.get(a) or {}
                if c.get("position_mode"):
                    return c["position_mode"]
        return "spherical"

    def pot_colour(self, arm, idx, value):
        """idx is 1-based. See the module docstring: mode-aware by design."""
        used = MODE_CHANNELS.get(self.mode(), MODE_CHANNELS["spherical"])
        known = KNOWN_BAD.get(arm, {})
        dropout = (value == 0.0)
        if idx not in used:
            # Never red: this mode does not consume it.
            return C_WARN if (dropout or idx in known) else C_OK
        if dropout:
            return C_BAD
        if idx in known:
            return C_WARN
        return C_OK

    def draw(self):
        self.s.erase()
        h, w = self.s.getmaxyx()
        with self.n.lock:
            raw = dict(self.n.raw)
            status = dict(self.n.status)
            ik = dict(self.n.ik)
            fsr = list(self.n.fsr) if self.n.fsr else None
            estop = self.n.estop
            blocking = dict(self.n.blocking)
            deadman = dict(self.n.deadman)
            recovery = dict(self.n.recovery)
            session = dict(self.n.session)
            hl = dict(self.n.highlevel)
            homing = dict(self.n.homing_done)
            poses = dict(self.n.pose)

        y = 0
        md = self.mode()
        est = "E-STOP LATCHED" if estop else ("clear" if estop is not None
                                              else "no /estop_state")
        self.put(y, 0, " SRL TELEOP CONSOLE ", C_HEAD, True)
        self.put(y, 21, "mode=%s" % md, C_HI, True)
        self.put(y, 21 + 14, "arm-focus=%s" % ARMS[self.sel_arm], C_HI)
        self.put(y, 21 + 34, est, C_BAD if estop else C_OK, True)
        y += 1
        self.put(y, 0, "-" * (w - 1), C_DIM)
        y += 1

        # ---- BLOCKERS FIRST. This is the "why is nothing moving" panel.
        y = self.draw_blockers(y, blocking, w)
        y += 1

        for i, a in enumerate(ARMS):
            y = self.draw_arm(y, a, raw.get(a), status.get(a), ik.get(a),
                              poses.get(a), w, focused=(i == self.sel_arm))
            y += 1

        y = self.draw_fsr(y, fsr, w)
        y += 1
        y = self.draw_real(y, hl, session, homing, deadman, recovery, w)
        y += 1
        self.draw_events(y, w, h)
        self.draw_footer(h, w)
        self.s.refresh()

    def draw_blockers(self, y, blocking, w):
        self.put(y, 0, "BLOCKERS  (why motion is stopped)", C_HEAD, True)
        y += 1
        now = time.monotonic()
        rows = []
        for unit, d in sorted(blocking.items()):
            silent = now - d.get("_rx", 0) > 5.0
            for b in d.get("blockers", []):
                if b.get("active"):
                    rows.append((unit, b["name"], "ACTIVE",
                                 "%.1fs %s" % (b.get("held_s", 0),
                                               b.get("reason", "")), C_BAD))
                elif b.get("expired"):
                    # Expired is NOT clear: the loop asserting it stopped, so
                    # the condition is unknown. Shown as loudly as active.
                    rows.append((unit, b["name"], "EXPIRED",
                                 "unknown %.1fs - asserting unit stopped"
                                 % b.get("unknown_for_s", 0), C_BAD))
            if silent:
                rows.append((unit, "-", "UNIT SILENT",
                             "no /blocking for %.0fs" % (now - d.get("_rx", 0)),
                             C_BAD))
        if not rows:
            self.put(y, 2, "none active - nothing is blocking motion", C_OK)
            return y + 1
        for unit, name, state, detail, col in rows[:8]:
            self.put(y, 2, "%-20s %-16s %-11s %s"
                     % (unit[:20], name[:16], state, detail[:w - 55]), col, True)
            y += 1
        return y

    def draw_arm(self, y, arm, raw, status, ik, pose, w, focused):
        mark = ">" if focused else " "
        self.put(y, 0, "%s %s ARM" % (mark, arm.upper()), C_HEAD, True)
        y += 1

        # pots
        self.put(y, 2, "pots ")
        x = 7
        if raw and len(raw) >= 7:
            for i in range(7):
                col = self.pot_colour(arm, i + 1, raw[i])
                self.put(y, x, "j%d%s" % (i + 1, _fmt(raw[i], 8, 1)), col,
                         col == C_BAD)
                x += 12
        else:
            self.put(y, x, "no /master_arm_raw_%s" % arm, C_BAD, True)
        y += 1

        # imu
        if raw and len(raw) >= 13:
            ax, ay, az = raw[7], raw[8], raw[9]
            gx, gy, gz = raw[10], raw[11], raw[12]
            amag = math.sqrt(ax * ax + ay * ay + az * az)
            wmag = math.sqrt(gx * gx + gy * gy + gz * gz)
            gate = abs(amag - 1.0) <= 0.15
            self.put(y, 2, "imu  a[%s %s %s] |a|%s  gate %s"
                     % (_fmt(ax, 6), _fmt(ay, 6), _fmt(az, 6), _fmt(amag, 6),
                        "OPEN" if gate else "SHUT"),
                     C_OK if gate else C_WARN)
            y += 1
            self.put(y, 2, "     w[%s %s %s] |w|%s deg/s"
                     % (_fmt(gx, 6), _fmt(gy, 6), _fmt(gz, 6), _fmt(wmag, 6)))
            y += 1
        else:
            self.put(y, 2, "imu  no data", C_WARN)
            y += 1

        # clutch / scale / health
        if status and len(status) >= 8:
            clutch = "ENGAGED" if status[0] > 0.5 else "disengaged"
            valid = status[5] > 0.5
            self.put(y, 2, "clutch %-11s scale %s  frame %s  bad %d  age %ss"
                     % (clutch, _fmt(status[1], 5), "ok" if valid else "REJECTED",
                        int(status[6]), _fmt(status[7], 5, 2)),
                     C_OK if valid else C_BAD, not valid)
        else:
            self.put(y, 2, "clutch/scale: no /master_status_%s" % arm, C_WARN)
        y += 1

        # commanded vs actual EE
        act = self.n.ee(arm)
        cmd = None
        if pose is not None:
            cmd = (pose.pose.position.x, pose.pose.position.y,
                   pose.pose.position.z) if hasattr(pose, "pose") else None
        if isinstance(pose, list):
            cmd = None
        if cmd and act:
            err = math.dist(cmd, act)
            self.put(y, 2, "EE  cmd[%s %s %s]  act[%s %s %s]  err %s m"
                     % (_fmt(cmd[0], 6, 3), _fmt(cmd[1], 6, 3), _fmt(cmd[2], 6, 3),
                        _fmt(act[0], 6, 3), _fmt(act[1], 6, 3), _fmt(act[2], 6, 3),
                        _fmt(err, 6, 4)),
                     C_OK if err < 0.02 else C_WARN)
        elif act:
            self.put(y, 2, "EE  act[%s %s %s]  (no commanded pose)"
                     % (_fmt(act[0], 6, 3), _fmt(act[1], 6, 3), _fmt(act[2], 6, 3)),
                     C_WARN)
        else:
            self.put(y, 2, "EE  no tf2 for %s_end_effector_link" % arm, C_WARN)
        y += 1

        # ik / guard / clearance
        if ik and len(ik) >= 8:
            succ, fail, direct, slew, rej = ik[0], ik[1], ik[2], ik[3], ik[4]
            tot = succ + fail
            rate = (100.0 * succ / tot) if tot else 0.0
            clr = ik[5]
            self.put(y, 2, "IK %5.1f%% (%d/%d)  GUARD direct %d slewed %d "
                     "rejected %d  clear %s m  blocks %d  redun %d"
                     % (rate, int(succ), int(tot), int(direct), int(slew),
                        int(rej), _fmt(clr, 5, 3), int(ik[6]), int(ik[7])),
                     C_OK if rate > 90 else (C_WARN if tot else C_DIM))
        else:
            self.put(y, 2, "IK  no /ik_status_%s" % arm, C_WARN)
        y += 1
        return y

    def draw_fsr(self, y, fsr, w):
        self.put(y, 0, "GRIPPERS / BUTTONS", C_HEAD, True)
        y += 1
        if not fsr or len(fsr) < 4:
            self.put(y, 2, "no /master_fsr_buttons", C_WARN)
            return y + 1
        f1, f2, b1, b2 = fsr[0], fsr[1], fsr[2], fsr[3]
        # Same mapping as fsr_gripper_node: proportional onto the driven
        # knuckle, 0 open .. 0.8 closed, deadband 250 counts.
        def knuckle(v, closed):
            if v < 250:
                return 0.0
            return max(0.0, min(0.8, 0.8 * (v - 250.0) / max(1.0, closed - 250.0)))
        self.put(y, 2, "fsr1(left) %7.0f -> %5.3f rad     "
                       "fsr2(right) %7.0f -> %5.3f rad"
                 % (f1, knuckle(f1, 3603), f2, knuckle(f2, 3842)))
        y += 1
        # MEASURED mapping: left button -> btn2, right button -> btn1.
        self.put(y, 2, "buttons  btn1 %d (RIGHT arm)   btn2 %d (LEFT arm)"
                 % (int(b1), int(b2)))
        return y + 1

    def draw_real(self, y, hl, session, homing, deadman, recovery, w):
        self.put(y, 0, "REAL ARMS", C_HEAD, True)
        y += 1
        conn = session.get("connected")
        if session:
            self.put(y, 2, "session %-12s recoveries %s  %s"
                     % ("CONNECTED" if conn else "LOST",
                        session.get("recoveries", 0), session.get("error", "")),
                     C_OK if conn else C_BAD, not conn)
            y += 1
        else:
            self.put(y, 2, "no /real/session_state - real stack not running",
                     C_DIM)
            y += 1
        for a in ARMS:
            d = hl.get(a)
            hd = homing.get(a)
            bits = ["homed %s" % ("yes" if hd else ("no" if hd is not None
                                                    else "?"))]
            if d and len(d) >= 3:
                bits.append("rate %.1f Hz" % d[0])
                bits.append("send %.1f ms" % (d[1] * 1000.0))
                bits.append("track %.2f deg" % math.degrees(d[2]))
            self.put(y, 2, "%-6s %s" % (a, "  ".join(bits)),
                     C_OK if hd else C_DIM)
            y += 1
        if deadman:
            armed = deadman.get("armed")
            self.put(y, 2, "dead-man %-9s deadline %ss  age %s  arrival %s"
                     % ("ARMED" if armed else "off",
                        _short(deadman.get("deadline_s")),
                        _short((deadman.get("age_s") or {}).get("left")),
                        _short((deadman.get("arrival_age_s") or {}).get("left"))),
                     C_OK if armed else C_DIM)
            y += 1
        faults = (recovery or {}).get("faults") or {}
        if faults:
            self.put(y, 2, "RECOVERY FAULTS: %s" % ", ".join(sorted(faults)),
                     C_BAD, True)
            y += 1
        return y

    def draw_events(self, y, w, h):
        room = h - y - 2
        if room < 3:
            return
        self.put(y, 0, "EVENTS  (every change, old -> new)", C_HEAD, True)
        y += 1
        for t, msg, bad in list(self.n.events)[:room - 1]:
            self.put(y, 2, "%s  %s" % (t, msg), C_BAD if bad else C_DIM, bad)
            y += 1

    def draw_footer(self, h, w):
        if self.help:
            return
        keys = ("TAB arm  v/V vel  s/S scale  e/E ema  g/G gate  m/M step  "
                "p mode  o orient  c clutch  C force  x E-STOP  r reset  "
                "1-7 chan  ? help  q quit")
        self.put(h - 1, 0, keys[:w - 1], C_HEAD)

    def draw_help(self):
        self.s.erase()
        lines = [
            ("SRL TELEOP CONSOLE - keys", True),
            ("", False),
            ("  TAB      switch which arm the per-arm keys act on", False),
            ("  v / V    max_vel_rad_s  down / up   (SIM followers)", False),
            ("  b / B    max_vel_rad_s  down / up   (REAL, sim_to_real_bridge)", False),
            ("           sim and real are separate on purpose: the real cap", False),
            ("           must not inherit whatever the sim was loosened to.", False),
            ("  s / S    motion scale for the focused arm", False),
            ("  e / E    ema_alpha", False),
            ("  g / G    accel_gate_g", False),
            ("  m / M    max_step_rad", False),
            ("  l / L    lag_trip_rad          (sim_to_real_bridge)", False),
            ("  d / D    preview_delay_s       (sim_to_real_bridge)", False),
            ("  p        cycle position_mode   spherical <-> fk", False),
            ("  o        cycle orientation_mode fixed <-> tilt", False),
            ("  c        toggle force_clutch_engaged", False),
            ("  C        force clutch ENGAGED", False),
            ("  1-7      toggle that channel on the focused arm", False),
            ("  x        trigger the E-STOP     (service, latches)", False),
            ("  r        reset the E-STOP       (service, the only way out)", False),
            ("  ?        close this help", False),
            ("  q        quit", False),
            ("", False),
            ("Every parameter change is applied through a parameter CLIENT and", False),
            ("read back to confirm. `ros2 param set` is never used: it goes via", False),
            ("the daemon, which hangs on this box and reports success it has", False),
            ("not earned.", False),
            ("", False),
            ("This process never opens the serial port. master_pose_node holds", False),
            ("it; a second reader would steal frames.", False),
        ]
        for i, (t, bold) in enumerate(lines):
            self.put(i, 2, t, C_HEAD if bold else C_DIM, bold)
        self.s.refresh()

    # ------------------------------------------------------------- controls
    def bump(self, node, name, factor=None, delta=None, lo=None, hi=None):
        self.submit(lambda: self._bump(node, name, factor, delta, lo, hi))

    def _bump(self, node, name, factor=None, delta=None, lo=None, hi=None):
        cur, err = self.n.get_param(node, name)
        if cur is None:
            self.n.log("FAILED read %s %s: %s" % (node, name, err or "?"), bad=True)
            return
        new = cur * factor if factor is not None else cur + delta
        if lo is not None:
            new = max(lo, new)
        if hi is not None:
            new = min(hi, new)
        self.n.set_param(node, name, float(new))

    def _cycle_position_mode(self):
        cur, _ = self.n.get_param("/master_pose_node", "position_mode")
        self.n.set_param("/master_pose_node", "position_mode",
                         "fk" if cur == "spherical" else "spherical")

    def _cycle_orientation_mode(self):
        cur, _ = self.n.get_param("/master_pose_node", "orientation_mode")
        nxt = {"fixed": "tilt", "tilt": "anchored", "anchored": "fixed"}
        self.n.set_param("/master_pose_node", "orientation_mode",
                         nxt.get(cur, "fixed"))

    def _toggle_force_clutch(self):
        cur, _ = self.n.get_param("/master_pose_node", "force_clutch_engaged")
        self.n.set_param("/master_pose_node", "force_clutch_engaged",
                         not bool(cur))

    def _toggle_channel(self, arm, j):
        name = "%s_j%d_enabled" % (arm, j)
        cur, _ = self.n.get_param("/channel_manager", name)
        self.n.set_param("/channel_manager", name, not bool(cur))

    def followers(self):
        return ["/ik_follower_%s" % a for a in ARMS]

    def handle(self, k):
        arm = ARMS[self.sel_arm]
        if k in (ord("q"), 27):
            return False
        if k == ord("?"):
            self.help = not self.help
        elif k == 9:                                   # TAB
            self.sel_arm = 1 - self.sel_arm
        elif k in (ord("v"), ord("V")):
            f = 0.8 if k == ord("v") else 1.25
            for n in self.followers():
                self.bump(n, "max_vel_rad_s", factor=f, lo=0.01, hi=2.0)
        elif k in (ord("b"), ord("B")):
            f = 0.8 if k == ord("b") else 1.25
            for a in ARMS:
                self.bump("/sim_to_real_bridge_%s" % a, "max_vel_rad_s",
                          factor=f, lo=0.005, hi=0.5)
        elif k in (ord("s"), ord("S")):
            f = 0.9 if k == ord("s") else 1.1111
            self.bump("/master_pose_node", "%s_scale" % arm, factor=f,
                      lo=0.05, hi=5.0)
        elif k in (ord("e"), ord("E")):
            d = -0.05 if k == ord("e") else 0.05
            self.bump("/master_pose_node", "ema_alpha", delta=d, lo=0.01, hi=1.0)
        elif k in (ord("g"), ord("G")):
            d = -0.01 if k == ord("g") else 0.01
            self.bump("/master_pose_node", "accel_gate_g", delta=d,
                      lo=0.01, hi=1.0)
        elif k in (ord("m"), ord("M")):
            f = 0.8 if k == ord("m") else 1.25
            for n in self.followers():
                self.bump(n, "max_step_rad", factor=f, lo=0.01, hi=1.0)
        elif k in (ord("l"), ord("L")):
            d = -0.01 if k == ord("l") else 0.01
            for a in ARMS:
                self.bump("/sim_to_real_bridge_%s" % a, "lag_trip_rad",
                          delta=d, lo=0.01, hi=1.0)
        elif k in (ord("d"), ord("D")):
            d = -0.1 if k == ord("d") else 0.1
            for a in ARMS:
                self.bump("/sim_to_real_bridge_%s" % a, "preview_delay_s",
                          delta=d, lo=0.0, hi=5.0)
        elif k == ord("p"):
            self.submit(self._cycle_position_mode)
        elif k == ord("o"):
            self.submit(self._cycle_orientation_mode)
        elif k == ord("c"):
            self.submit(self._toggle_force_clutch)
        elif k == ord("C"):
            self.submit(lambda: self.n.set_param(
                "/master_pose_node", "force_clutch_engaged", True))
        elif k == ord("x"):
            self.submit(lambda: self.n.call_trigger(self.n.estop_cli, "E-STOP"))
        elif k == ord("r"):
            self.submit(lambda: self.n.call_trigger(self.n.reset_cli,
                                                   "E-STOP reset"))
        elif ord("1") <= k <= ord("7"):
            j = k - ord("0")
            self.submit(lambda a=arm, jj=j: self._toggle_channel(a, jj))
        return True


def _refuse(msg):
    sys.stderr.write("teleop_gui: %s\n" % msg)
    return 2


def main():
    # A CURSES PROGRAM NEEDS A REAL TERMINAL. Without one, curses fails deep
    # inside wrapper() -- cbreak() returns ERR, and then wrapper's own cleanup
    # calls nocbreak() which ALSO returns ERR and replaces the original
    # exception, so the traceback names the wrong call. Refuse up front with
    # something actionable instead.
    if not sys.stdout.isatty() or not sys.stdin.isatty():
        return _refuse(
            "stdout/stdin is not a terminal. This is a full-screen curses "
            "console; it cannot be piped, redirected or run headless.\n"
            "  run it directly:   ros2 run srl_teleop teleop_gui\n"
            "  to capture output: script -qec 'ros2 run srl_teleop teleop_gui' out.txt")
    term = os.environ.get("TERM", "")
    if term in ("", "dumb"):
        return _refuse(
            "TERM=%r cannot drive a full-screen display. Set a real terminal "
            "type, e.g.  TERM=xterm-256color ros2 run srl_teleop teleop_gui"
            % term)

    rclpy.init()
    node = GuiNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def run(stdscr):
        gui = Gui(stdscr, node)
        node.log("teleop_gui up - topics only, serial port untouched")
        period = 0.2                              # 5 Hz
        nxt = time.monotonic()
        while True:
            k = stdscr.getch()
            while k != -1:
                if not gui.handle(k):
                    return
                k = stdscr.getch()
            if gui.help:
                gui.draw_help()
            else:
                gui.draw()
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    except curses.error as e:
        # Never let a terminal-capability failure look like a crash in the
        # teleop stack. Say what it was and what to do.
        sys.stderr.write(
            "teleop_gui: terminal error: %s\n"
            "  TERM=%s, size=%s. Try a larger window or "
            "TERM=xterm-256color.\n" % (e, os.environ.get("TERM"),
                                         os.environ.get("LINES", "?")))
        return 2
    finally:
        # BOUNDED TEARDOWN. An unbounded ex.shutdown() can block forever if a
        # callback in the MultiThreadedExecutor is still running -- the
        # program then never exits on 'q', which from the outside is
        # indistinguishable from a crash and made the pty test report exit
        # 250 instead of 0. Every step is time-boxed and none may raise.
        try:
            ex.shutdown(timeout_sec=2.0)
        except Exception:                                    # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:                                    # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                    # noqa: BLE001
            pass
        # The terminal has already been restored by curses.wrapper, so there
        # is nothing left to clean up in this process. Exit immediately
        # rather than waiting on a non-daemon rclpy thread that may never
        # return -- a hung exit is a worse failure than an abrupt one here.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
