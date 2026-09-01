#!/usr/bin/env python3
"""record_trajectories.py — button-gated trajectory capture, you set the pace.

    ros2 run srl_experiments record_trajectories
    ros2 run srl_experiments record_trajectories --start-at C_left_front
    ros2 run srl_experiments record_trajectories --blocks A,B --dry-run

Terminal 1 must already be running `bash scripts/run_teleop.sh`.

PROTOCOL. It prints an instruction and waits INDEFINITELY. You press to start,
move at your own pace, press again to end. The segment is saved and it
advances. A press-and-release under `min_segment_s` DISCARDS the segment and
re-prompts, and because rows are buffered and only committed on save, a
discarded take leaves no trace in the data.

USE THE BUTTON ON THE ARM YOU ARE MOVING. Left arm, left button; right arm,
right button. That is only safe because the capture PINS THE CLUTCH for its
duration -- an arm's own button normally toggles that arm's clutch, and
gating a left sweep on the left button used to disengage it exactly when the
sweep began. See GATE_INDEX below for what that cost and how the pin removes
it. The capture refuses to start if it cannot confirm the pin.

DO NOT RUN THIS INSIDE THE LAUNCH. `ros2 launch` merges and line-prefixes all
stdout, so RViz and MoveIt bury the prompt and the prefixes break the in-place
redraw. Run it in its own terminal.

BEFORE STARTING, this checks /joint_states is alive and both arm controllers
are active. A capture with silent /tf produced 0/20440 EE rows once and the
cause was not the recorder.
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

import tf2_ros

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from capture_protocol import segments, estimate  # noqa: E402

ARMS = ("left", "right")
# A pot reading outside this band is a serial parse glitch, not a reading.
# 0.2-0.8%% of samples in the 2026-08-06 capture were outside it, in bursts up
# to 27 samples, and unwrapping them produced a "range" of 314363 deg on a
# sensor that cannot exceed 360. Rejected HERE so corrupt values never enter
# the file; the count is recorded per row so the rejection is visible.
POT_MIN_DEG, POT_MAX_DEG = 0.0, 360.0
# EACH ARM IS GATED BY ITS OWN BUTTON, AND THE CLUTCH IS PINNED SO THAT IS
# SAFE.
#
# Message layout is [fsr1, fsr2, btn1, btn2], so index 2 = btn1 and 3 = btn2.
# MEASURED: the LEFT arm's own clutch button is btn2, the RIGHT arm's is btn1.
#     left  segment -> btn2 -> index 3
#     right segment -> btn1 -> index 2
#
# WHY THIS USED TO BE THE OTHER ARM'S BUTTON. Pressing an arm's own button
# toggles that arm's CLUTCH, so gating a left sweep on the left button
# disengaged the left clutch exactly when the sweep started. In the
# 2026-08-06 capture 41 of 42 directional segments recorded with the clutch
# OUT and the sim arm stationary, which made the whole set unusable. Gating
# on the opposite arm's button avoided that -- at the price of an operator
# holding the left arm having to reach across and press with the right hand,
# for every one of 28 segments.
#
# The real fix is to stop the button touching the clutch at all.
# `master_pose_node` has `force_clutch_engaged`, which pins the clutch ON and
# ignores the buttons for clutch purposes while leaving their VALUES on
# /master_fsr_buttons for this gate to read. `pin_clutch()` sets it for the
# duration of the capture and restores it afterwards, and REFUSES TO RECORD
# if it cannot confirm the pin -- because own-arm gating without the pin is
# precisely the defect above.
#
# Belt and braces: every row carries the clutch state, and `clutch_was_out()`
# checks the recorded rows at the end of each segment. If the clutch dropped
# for any reason the segment says so and can be redone, rather than the
# operator discovering it at the analysis.
GATE_INDEX = {"left": 3, "right": 2}
#: The older mapping, still selectable with --gate opposite: it needs no
#: clutch pin, at the price of reaching across for every segment.
GATE_INDEX_OPPOSITE = {"left": 2, "right": 3}

COLUMNS = (
    ["t", "segment", "block", "arm", "rep", "direction", "speed",
     # Block G parks the arm somewhere before sweeping, and the station is
     # the independent variable the whole block exists to vary. Without it a
     # G row is indistinguishable from a C row.
     "station", "sample_index"]
    + ["l_j%d" % i for i in range(1, 8)]
    + ["r_j%d" % i for i in range(1, 8)]
    + ["l_ax", "l_ay", "l_az", "l_gx", "l_gy", "l_gz"]
    + ["r_ax", "r_ay", "r_az", "r_gx", "r_gy", "r_gz"]
    + ["fsr1", "fsr2", "btn1", "btn2"]
    + ["%s_cmd_%s" % (a, c) for a in ARMS for c in ("x", "y", "z")]
    + ["%s_cmd_q%s" % (a, c) for a in ARMS for c in ("x", "y", "z", "w")]
    + ["%s_ee_%s" % (a, c) for a in ARMS for c in ("x", "y", "z")]
    + ["%s_joint_%d" % (a, i) for a in ARMS for i in range(1, 8)]
    + ["%s_clutch" % a for a in ARMS]
    + ["%s_scale" % a for a in ARMS]
    # THE DECOMPOSITION'S OWN OUTPUTS. `/master_status_<arm>` carries the
    # elevation, azimuth and reach that master_pose_node actually computed,
    # and this recorder subscribed to that topic and threw all three away.
    # Without them an analysis has to re-derive the decomposition from the raw
    # joints, and a re-derivation that differs from the node's by a sign or a
    # datum is indistinguishable from a finding. These are the three
    # quantities the left/right defect lives in, so they are recorded as the
    # NODE produced them.
    + ["%s_elev_deg" % a for a in ARMS]
    + ["%s_azim_deg" % a for a in ARMS]
    + ["%s_reach_m" % a for a in ARMS]
    # 1 when the accelerometer was outside the quasi-static band, so gravity
    # could not be isolated and the elevation on that row is the last trusted
    # value rather than a fresh measurement. An elevation analysis that does
    # not exclude these rows is fitting held values.
    + ["%s_elev_held" % a for a in ARMS]
    + ["%s_frame_valid" % a for a in ARMS]
    + ["%s_n_dropouts" % a for a in ARMS]
    + ["%s_data_age_s" % a for a in ARMS]
    + ["position_mode", "orientation_mode"]
    + ["%s_channels_live" % a for a in ARMS]
    + ["%s_ik_success" % a for a in ARMS]
    + ["%s_ik_fail" % a for a in ARMS]
    + ["%s_guard_direct" % a for a in ARMS]
    + ["%s_guard_slewed" % a for a in ARMS]
    + ["%s_guard_rejected" % a for a in ARMS]
    + ["%s_min_clearance" % a for a in ARMS]
    # REAL ARM. Absent from the 2026-08-06 capture entirely, which is why it
    # could support no real-arm conclusion at all -- there was nothing to
    # compare the sim against.
    + ["%s_real_joint_%d" % (a, i) for a in ARMS for i in range(1, 8)]
    + ["%s_real_ee_%s" % (a, c) for a in ARMS for c in ("x", "y", "z")]
    + ["real_bridge_enabled", "real_session_connected"]
    # Provenance of the row itself.
    + ["t_mono", "master_seq_left", "master_seq_right", "n_rejected_raw"]
    + ["estop"]
)


class Capture(Node):
    def __init__(self):
        super().__init__("trajectory_capture")
        self.lock = threading.Lock()
        self.gate_index = dict(GATE_INDEX)      # overridden by --gate
        self.raw = {a: None for a in ARMS}
        self.status = {a: None for a in ARMS}
        self.pose = {a: None for a in ARMS}
        self.ik = {a: None for a in ARMS}
        self.caps = {a: {} for a in ARMS}
        self.fsr = None
        self.js = {}
        self.estop = False
        self.js_times = []

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        for a in ARMS:
            self.create_subscription(Float64MultiArray, "/master_arm_raw_%s" % a,
                                     self._raw_cb(a), 50)
            self.create_subscription(Float64MultiArray, "/master_status_%s" % a,
                                     self._lst("status", a), 50)
            self.create_subscription(PoseStamped, "/master_arm_pose_%s" % a,
                                     self._obj("pose", a), 50)
            self.create_subscription(Float64MultiArray, "/ik_status_%s" % a,
                                     self._lst("ik", a), 10)
            self.create_subscription(String, "/master_capability_%s" % a,
                                     self._js("caps", a), 10)
        self.create_subscription(Float64MultiArray, "/master_fsr_buttons",
                                 self._plain("fsr"), 50)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(Bool, "/estop_state",
                                 self._bool("estop"), 10)
        self.real_js = {}
        self.bridge_enabled = None
        self.session_connected = None
        self.create_subscription(JointState, "/real/joint_states",
                                 self._real_js, 20)
        self.create_subscription(String, "/real/session_state",
                                 self._session, 10)
        for a in ARMS:
            self.create_subscription(
                Bool, "/sim_to_real_enabled_%s" % a,
                (lambda arm: (lambda m: setattr(self, "bridge_enabled",
                                                bool(m.data))))(a), 10)
        # Sequence counters: incremented only when the master RAW payload
        # actually changes. The recorder used to run at 50 Hz against a
        # ~15 Hz sensor, so 82-88%% of adjacent rows were bit-identical and
        # every per-row statistic collapsed -- the reported noise floor of
        # 0.000 deg was that artefact, not the sensor.
        self.seq = {a: 0 for a in ARMS}
        self._last_raw = {a: None for a in ARMS}
        self.n_rejected = 0

    def _raw_cb(self, arm):
        """Store the raw frame, REJECTING implausible pot values at capture.

        A value outside [POT_MIN_DEG, POT_MAX_DEG] is a parse glitch. Writing
        it to the file and cleaning later does not work: the corruption comes
        in bursts and any statistic computed before cleaning inherits it.
        Rejected samples become empty cells and are counted in n_rejected_raw
        so the rejection is visible rather than silent.
        """
        def cb(m):
            d = list(m.data)
            rej = 0
            for i in range(min(7, len(d))):
                if not (POT_MIN_DEG <= d[i] <= POT_MAX_DEG):
                    d[i] = float("nan")
                    rej += 1
            with self.lock:
                if self._last_raw[arm] != d:
                    self.seq[arm] += 1
                    self._last_raw[arm] = list(d)
                self.raw[arm] = d
                self.n_rejected += rej
        return cb

    def _lst(self, f, a):
        def cb(m):
            with self.lock:
                getattr(self, f)[a] = list(m.data)
        return cb

    def _obj(self, f, a):
        def cb(m):
            with self.lock:
                getattr(self, f)[a] = m
        return cb

    def _js(self, f, a):
        def cb(m):
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            with self.lock:
                getattr(self, f)[a] = d
        return cb

    def _plain(self, f):
        def cb(m):
            with self.lock:
                setattr(self, f, list(m.data))
        return cb

    def _bool(self, f):
        def cb(m):
            with self.lock:
                setattr(self, f, bool(m.data))
        return cb

    def _real_js(self, m):
        with self.lock:
            self.real_js = dict(zip(m.name, m.position))

    def _session(self, m):
        try:
            self.session_connected = bool(json.loads(m.data).get("connected"))
        except (ValueError, TypeError):
            pass

    def _on_js(self, m):
        with self.lock:
            self.js = dict(zip(m.name, m.position))
            self.js_times.append(time.monotonic())
            if len(self.js_times) > 200:
                self.js_times = self.js_times[-200:]

    # ---------------------------------------------------------------- reads
    def ee(self, arm, frame_prefix=""):
        try:
            t = self.tf_buf.lookup_transform(
                "%sworld" % frame_prefix,
                "%s%s_end_effector_link" % (frame_prefix, arm),
                rclpy.time.Time())
            v = t.transform.translation
            return (v.x, v.y, v.z)
        except Exception:                                        # noqa: BLE001
            return (None, None, None)

    def button(self, arm):
        """The gate for `arm`, under whichever mapping is in force.

        `gate_index` is set once from the --gate flag, so the mapping cannot
        differ between the prompt the operator is shown and the button the
        code waits on.
        """
        with self.lock:
            f = list(self.fsr) if self.fsr else None
        if not f or len(f) < 4:
            return None
        return bool(f[self.gate_index[arm]] > 0.5)

    def joint_states_hz(self):
        with self.lock:
            ts = [t for t in self.js_times if t > time.monotonic() - 3.0]
        if len(ts) < 3:
            return 0.0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def row(self, seg, idx):
        with self.lock:
            raw = {a: (list(self.raw[a]) if self.raw[a] else None) for a in ARMS}
            st = {a: (list(self.status[a]) if self.status[a] else None)
                  for a in ARMS}
            ik = {a: (list(self.ik[a]) if self.ik[a] else None) for a in ARMS}
            caps = {a: dict(self.caps[a]) for a in ARMS}
            pose = {a: self.pose[a] for a in ARMS}
            fsr = list(self.fsr) if self.fsr else [None] * 4
            js = dict(self.js)
            rjs = dict(self.real_js)
            estop = self.estop
            seq = dict(self.seq)
            nrej = self.n_rejected
            be, sc = self.bridge_enabled, self.session_connected
        r = dict.fromkeys(COLUMNS, "")
        r.update(t="%.4f" % time.time(), segment=seg["label"],
                 block=seg["block"], arm=seg["arm"], rep=seg.get("rep", 0),
                 direction=seg.get("direction", ""), speed=seg.get("speed", ""),
                 station=seg.get("station", ""),
                 sample_index=idx, estop=int(estop),
                 t_mono="%.4f" % time.monotonic(),
                 master_seq_left=seq.get("left", 0),
                 master_seq_right=seq.get("right", 0),
                 n_rejected_raw=nrej,
                 real_bridge_enabled=("" if be is None else int(be)),
                 real_session_connected=("" if sc is None else int(sc)))
        for a, p in (("left", "l"), ("right", "r")):
            d = raw.get(a)
            if d and len(d) >= 13:
                for i in range(7):
                    r["%s_j%d" % (p, i + 1)] = "%.4f" % d[i]
                for k, v in zip(("ax", "ay", "az", "gx", "gy", "gz"), d[7:13]):
                    r["%s_%s" % (p, k)] = "%.5f" % v
        for k, v in zip(("fsr1", "fsr2", "btn1", "btn2"), fsr):
            r[k] = "" if v is None else "%.1f" % v
        for a in ARMS:
            ps = pose.get(a)
            if ps is not None:
                r["%s_cmd_x" % a] = "%.6f" % ps.pose.position.x
                r["%s_cmd_y" % a] = "%.6f" % ps.pose.position.y
                r["%s_cmd_z" % a] = "%.6f" % ps.pose.position.z
                r["%s_cmd_qx" % a] = "%.6f" % ps.pose.orientation.x
                r["%s_cmd_qy" % a] = "%.6f" % ps.pose.orientation.y
                r["%s_cmd_qz" % a] = "%.6f" % ps.pose.orientation.z
                r["%s_cmd_qw" % a] = "%.6f" % ps.pose.orientation.w
            ex, ey, ez = self.ee(a)
            if ex is not None:
                r["%s_ee_x" % a] = "%.6f" % ex
                r["%s_ee_y" % a] = "%.6f" % ey
                r["%s_ee_z" % a] = "%.6f" % ez
            for i in range(1, 8):
                n = "%s_joint_%d" % (a, i)
                if n in js:
                    r[n] = "%.6f" % js[n]
            s = st.get(a)
            if s and len(s) >= 8:
                r["%s_clutch" % a] = int(s[0] > 0.5)
                r["%s_scale" % a] = "%.4f" % s[1]
                r["%s_elev_deg" % a] = "%.4f" % s[2]
                r["%s_azim_deg" % a] = "%.4f" % s[3]
                r["%s_reach_m" % a] = "%.6f" % s[4]
                r["%s_frame_valid" % a] = int(s[5] > 0.5)
                r["%s_n_dropouts" % a] = int(s[6])
                # Index 8, and only when it is there: a scripted operator
                # publishes a shorter array, and a missing field must stay
                # EMPTY rather than becoming a zero that reads as "not held".
                if len(s) > 8:
                    r["%s_elev_held" % a] = int(s[8] > 0.5)
                r["%s_data_age_s" % a] = "%.4f" % s[7]
            k = ik.get(a)
            if k and len(k) >= 8:
                r["%s_ik_success" % a] = int(k[0])
                r["%s_ik_fail" % a] = int(k[1])
                r["%s_guard_direct" % a] = int(k[2])
                r["%s_guard_slewed" % a] = int(k[3])
                r["%s_guard_rejected" % a] = int(k[4])
                r["%s_min_clearance" % a] = "%.5f" % k[5]
            for i in range(1, 8):
                k = "%s_real_joint_%d" % (a, i)
                nm = "%s_joint_%d" % (a, i)
                if nm in rjs:
                    r[k] = "%.6f" % rjs[nm]
            rx, ry, rz = self.ee(a, frame_prefix="real_")
            if rx is not None:
                r["%s_real_ee_x" % a] = "%.6f" % rx
                r["%s_real_ee_y" % a] = "%.6f" % ry
                r["%s_real_ee_z" % a] = "%.6f" % rz
            c = caps.get(a) or {}
            ch = c.get("channels") or {}
            r["%s_channels_live" % a] = "|".join(
                sorted(k2 for k2, v2 in ch.items() if v2))
            if c.get("position_mode"):
                r["position_mode"] = c["position_mode"]
        return r


class Gate:
    """CHANGE detector on one arm's gate button. One physical press = one True.

    EDGES IN EITHER DIRECTION, NOT RISING EDGES, because the buttons are
    firmware TOGGLES. `firmware/master_arm/teensy_final.ino` publishes a
    LATCHED 0/1 that flips once per press and stays there, so the value is
    the parity of how many times the button has ever been pressed, not
    whether a finger is on it.

    Waiting for a RISING edge therefore made every prompt depend on that
    parity. If the toggle happened to be sitting at 1 -- which it is after
    any odd number of presses, including presses from a previous session --
    the first press flipped it 1->0 and STARTED NOTHING, and the operator had
    to press twice with no indication why. Worse, it was systematic at the
    end of a segment: the press that started recording left the toggle at 1,
    so "press again to end" needed two presses, every segment, forever.

    A flip in either direction is exactly one press, which is what the prompt
    promises. Each call consumes at most one.
    """

    def __init__(self, node, arm):
        self.node, self.arm = node, arm
        self.prev = node.button(arm)

    def pressed(self):
        b = self.node.button(self.arm)
        if b is None:
            return False
        # `prev is None` means the buttons had not arrived when this Gate was
        # built. Adopt the first value seen rather than counting it a press.
        if self.prev is None:
            self.prev = b
            return False
        r = bool(b != self.prev)
        self.prev = b
        return r

    def wait_press(self, timeout=None):
        t0 = time.monotonic()
        while rclpy.ok():
            if self.pressed():
                return True
            if timeout is not None and time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.005)
        return False


def segment_blockers(node, arm, need_real, need_channels):
    """Why this segment must NOT run (or must stop). Empty list = proceed.

    Checked BEFORE each segment and again DURING it. In the 2026-08-06
    capture the clutch was disengaged on the arm under test for 41 of 42
    directional segments and the e-stop was latched for all of block F, and
    nothing stopped the run -- 35 minutes recorded with the sim arm
    stationary. Every condition below is one that silently produced a file
    full of nothing.
    """
    out = []
    if node.estop:
        out.append("E-STOP LATCHED - the arm cannot follow. Clear it with\n"
                   "      ros2 service call /estop_reset std_srvs/srv/Trigger\n"
                   "      (or --reset-estop, which calls exactly that once at "
                   "start-up).\n"
                   "      If this latched the moment you pressed the gate "
                   "button, the stack\n"
                   "      predates the 2026-09-01 fix: estop_node read the "
                   "TOGGLE buttons as\n"
                   "      levels, so one button left latched on from an "
                   "earlier session made\n"
                   "      your very first gating press look like a "
                   "two-handed squeeze.\n"
                   "      Restart the stack to pick the fix up.")
    st = node.status.get(arm)
    if st is None:
        out.append("no /master_status_%s" % arm)
    elif len(st) > 0 and st[0] <= 0.5:
        out.append("CLUTCH DISENGAGED on the %s arm - the commanded pose is "
                   "frozen, so nothing will be recorded but a still arm" % arm)
    if need_real:
        if not node.bridge_enabled:
            out.append("sim->real bridge NOT enabled but real_* data wanted")
        if node.session_connected is False:
            out.append("Kortex session not connected")
    caps = (node.caps.get(arm) or {}).get("channels") or {}
    dead = [c for c in need_channels if caps and not caps.get(c, True)]
    if dead:
        out.append("channel(s) %s disabled/dead on %s" % (",".join(dead), arm))
    return out


MASTER_NODE = "/master_pose_node"


#: Clients onto the master node's own parameter services, created once and
#: reused. Built lazily against the Capture node, which is already spinning
#: under the MultiThreadedExecutor, so these calls need no executor of their
#: own.
_PARAM_CLI = {}


def _param_client(node, kind):
    key = (kind, id(node))
    cli = _PARAM_CLI.get(key)
    if cli is None:
        srv, path = ((SetParameters, MASTER_NODE + "/set_parameters")
                     if kind == "set" else
                     (GetParameters, MASTER_NODE + "/get_parameters"))
        cli = node.create_client(srv, path)
        _PARAM_CLI[key] = cli
    return cli


def _call(node, cli, req, timeout=8.0):
    """One parameter call, with a real timeout and no subprocess.

    THIS USED TO SHELL OUT TO `ros2 param`, and that is what made the
    2026-09-01 capture unrecoverable rather than merely refused. Each call
    spawned a CLI process that built its own node, joined the graph and ran
    its own discovery -- ~2 s when it worked and a hard 20 s hang when it did
    not, which is what "`ros2 param set` timed out after 20 seconds" was. The
    same call sits in the `finally:` clean-up, so a Ctrl-C landed INSIDE
    subprocess.communicate() and dumped a KeyboardInterrupt traceback over
    the exit path, leaving the clutch pinned with only a stack trace to say so.

    This process is already in the graph with a spinning executor. Asking the
    master node's own parameter services directly is the same question
    without a second discovery, and it is interruptible.
    """
    if not cli.wait_for_service(timeout_sec=timeout):
        return None, ("no %s -- is master_pose_node running? (`ros2 node "
                      "list | grep master_pose`)" % cli.srv_name)
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done():
        if time.monotonic() - t0 > timeout:
            return None, ("%s did not answer within %.0f s" % (cli.srv_name,
                                                               timeout))
        if not rclpy.ok():
            return None, "shutting down"
        time.sleep(0.01)
    return fut.result(), ""


def _param_get(node, name):
    """Read one bool parameter. Returns (value_or_None, why)."""
    cli = _param_client(node, "get")
    req = GetParameters.Request()
    req.names = [name]
    res, why = _call(node, cli, req)
    if res is None:
        return None, why
    if not res.values:
        return None, "%s does not declare %s" % (MASTER_NODE, name)
    v = res.values[0]
    if v.type != ParameterType.PARAMETER_BOOL:
        return None, "%s is type %d, not bool" % (name, v.type)
    return bool(v.bool_value), ""


def _param_set(node, name, value):
    """Set one bool parameter. Returns (ok, why)."""
    cli = _param_client(node, "set")
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_BOOL
    pv.bool_value = bool(value)
    req = SetParameters.Request()
    req.parameters = [ParameterMsg(name=name, value=pv)]
    res, why = _call(node, cli, req)
    if res is None:
        return False, why
    if not res.results or not res.results[0].successful:
        return False, (res.results[0].reason if res.results
                       else "no result returned")
    return True, ""


def pin_clutch(on, node, arms=ARMS):
    """Pin the clutch ENGAGED for the capture, and CONFIRM it took.

    Own-arm gating is only safe while the buttons cannot toggle the clutch,
    so this is a precondition and not a convenience. It returns (ok, why).

    CONFIRMED TWO WAYS, because the parameter reading back is not evidence
    that the node honours it -- `force_clutch_engaged` was copied into an
    attribute at construction and never re-read, so for a long time setting
    it reported success and changed nothing. The second check is behavioural:
    the clutch must actually READ engaged on every arm afterwards.

    AND THE TWO FAILURES ARE REPORTED SEPARATELY. This used to answer every
    behavioural failure with "probably an older build that reads
    force_clutch_engaged once at start-up", which is one specific diagnosis
    for at least two causes. On 2026-09-01 the right arm was simply not
    publishing /master_status_right -- the pre-capture check had printed
    "master raw   left" one line above -- and the operator was sent to
    rebuild and restart a stack whose build was fine. A silent arm and a
    disengaged clutch are different faults with different fixes, and an
    instrument that renders them identically is the fault this repo has hit
    seventeen times.
    """
    ok, why = _param_set(node, "force_clutch_engaged", on)
    if not ok:
        return False, "could not set force_clutch_engaged: %s" % why
    got, why = _param_get(node, "force_clutch_engaged")
    if got is None:
        return False, "could not read force_clutch_engaged back: %s" % why
    if got != bool(on):
        return False, ("the parameter did not take: %s reports "
                       "force_clutch_engaged=%s after setting it to %s"
                       % (MASTER_NODE, got, bool(on)))
    if not on:
        return True, ""
    # Behavioural check: the clutch must now read engaged on every arm.
    t0 = time.time()
    silent, out_of = list(arms), []
    while time.time() - t0 < 4.0:
        st = {a: node.status.get(a) for a in arms}
        silent = [a for a, v in st.items() if not v or len(v) == 0]
        out_of = [a for a, v in st.items() if v and len(v) > 0 and v[0] <= 0.5]
        if not silent and not out_of:
            return True, ""
        time.sleep(0.2)
    if silent:
        return False, ("%s -- not publishing, so the pin CANNOT be confirmed "
                       "on %s.\n"
                       "         THIS IS NOT A STALE BUILD, it is a SILENT "
                       "ARM. Check that master_pose_node\n"
                       "         is running for %s and that its Teensy is "
                       "attached:\n"
                       "             ros2 topic hz /master_status_%s"
                       % (", ".join("no /master_status_%s" % a for a in silent),
                          " or ".join(silent),
                          " and ".join(silent), silent[0]))
    return False, ("force_clutch_engaged is set and read back true, but the "
                   "clutch still reads DISENGAGED on %s.\n"
                   "         The running master_pose_node is an older build "
                   "that reads the parameter\n"
                   "         once at start-up. Restart the stack, or fall "
                   "back to --gate opposite." % ", ".join(out_of))


def gate_word(node, arm):
    """The button to PRESS for `arm`, in the words the operator will use.

    Derived from the same `gate_index` the code waits on, so the prompt
    cannot disagree with the button. It printed the exact inverse once, and
    an operator who trusts the prompt then presses a button that does
    nothing has no way to tell that from a dead board.
    """
    # index 3 is btn2, the LEFT arm's own button; index 2 is btn1, the
    # RIGHT arm's.
    return "LEFT" if node.gate_index[arm] == 3 else "RIGHT"


def _teardown(ex, node, thread=None):
    """Shut down in the order rclpy requires, quietly.

    Calling rclpy.shutdown() while the executor thread is still inside
    wait_for_ready_callbacks raises RCLError out of that thread, and the
    traceback lands on top of whatever message the caller was trying to
    show the operator. Stop the executor, let the thread leave, THEN
    shut the context down.
    """
    try:
        ex.shutdown()
    except Exception:                                    # noqa: BLE001
        pass
    if thread is not None and thread.is_alive():
        thread.join(timeout=2.0)
    try:
        node.destroy_node()
    except Exception:                                    # noqa: BLE001
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:                                    # noqa: BLE001
        pass


def reset_estop():
    """Clear a LATCHED e-stop. This is not a bypass and there is no bypass.

    /estop stays live throughout: this calls the node's own documented reset,
    the same one a person would type. It exists because a latch left over
    from a previous session refuses the preflight, and the fix should not be
    a command the operator has to go and find.
    """
    import subprocess
    try:
        p = subprocess.run(["ros2", "service", "call", "/estop_reset",
                            "std_srvs/srv/Trigger"],
                           capture_output=True, text=True, timeout=15)
    except Exception as e:                                # noqa: BLE001
        return False, str(e)
    if p.returncode != 0:
        return False, (p.stderr or p.stdout or "").strip()[:200]
    return "success=True" in p.stdout, p.stdout.strip()[-200:]


def clutch_was_out(rows, arm):
    """Did the clutch drop during this segment? Read from the rows written.

    The pin is a precondition; this is the check that it HELD. A segment
    recorded with the clutch out has a frozen commanded pose and is worthless
    for the gain it was recorded to measure, and that is far cheaper to find
    now than at the analysis.
    """
    col = "%s_clutch" % arm
    seen = [r.get(col) for r in rows if r.get(col) not in ("", None)]
    if not seen:
        return False, ""
    out = sum(1 for v in seen if str(v) == "0")
    if not out:
        return False, ""
    return True, ("the %s clutch was OUT for %d of %d rows -- the commanded "
                  "pose is frozen for those and the segment cannot measure a "
                  "gain" % (arm, out, len(seen)))


def stack_processes():
    """The stack's own processes, from the process table rather than the graph.

    This deliberately does NOT go through ROS. The whole point is to have one
    observation that is independent of discovery, so that "nothing is running"
    can be told apart from "something is running and I cannot see it". Those
    two look identical on every ROS topic and need opposite fixes.
    """
    import subprocess
    want = ("master_pose_node", "ik_follower_node", "robot_state_publisher",
            "move_group")
    try:
        ps = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True,
                            text=True, timeout=10).stdout
    except Exception:                                        # noqa: BLE001
        return {}
    found = {}
    for line in ps.splitlines():
        for w in want:
            if w in line and "grep" not in line:
                found.setdefault(w, []).append(line.split()[0])
    return found


def preflight(node, need_arms=ARMS):
    hz = node.joint_states_hz()
    ok = True
    print("PRE-CAPTURE CHECK")
    print("  /joint_states     %.1f Hz %s" % (hz, "" if hz > 20 else "<-- TOO LOW"))
    ok &= hz > 20
    # PER ARM, AND ONLY THE ARMS THIS RUN SWEEPS. This printed the arms it
    # found and passed as long as there was at least one, so a run whose
    # blocks sweep both arms started with one arm silent -- and the failure
    # surfaced two steps later as the clutch pin blaming a stale build. The
    # capture cannot record an arm that is not publishing, so refuse here,
    # where the reason is still legible.
    have = [a for a in ARMS if node.raw.get(a)]
    print("  master raw        %s" % (", ".join(have) if have else "NONE"))
    missing = [a for a in need_arms if a not in have]
    if not have:
        print("     no /master_arm_raw_* - is the Teensy attached and "
              "master_pose_node running?")
        ok = False
    elif missing:
        print("     MISSING: %s. This run sweeps %s, so a silent arm means"
              % (", ".join("/master_arm_raw_%s" % a for a in missing),
                 " and ".join(need_arms)))
        print("     every segment on %s would record a still arm."
              % " and ".join(missing))
        print("     Start master_pose_node for %s, or limit the run with "
              "--blocks." % " and ".join(missing))
        ok = False
    st_have = [a for a in need_arms if node.status.get(a)]
    print("  master status     %s" % (", ".join(st_have) if st_have else "NONE"))
    if len(st_have) != len(need_arms):
        print("     no /master_status_%s - the clutch state cannot be read, "
              "so the pin\n"
              "     cannot be confirmed and clutch_was_out() cannot check "
              "the recorded rows."
              % ",".join(a for a in need_arms if a not in st_have))
        ok = False
    if node.fsr is None:
        print("  buttons           NO /master_fsr_buttons - the gate cannot "
              "work; capture would hang at the first prompt")
        ok = False
    else:
        print("  buttons           present")
    ee = [a for a in ARMS if node.ee(a)[0] is not None]
    print("  tf2 EE            %s" % (", ".join(ee) if ee else "NONE"))
    if not ee:
        print("     a capture with dead /tf silently loses every EE column. "
              "Check `ros2 control list_controllers`.")
        ok = False
    if not ok:
        # WHICH FAILURE IS THIS? Everything above reads the ROS graph, so
        # every one of those lines says NONE both when the stack is down and
        # when the stack is up and discovery is broken. Measured on
        # 2026-09-01: master_pose_node had been running for seven minutes,
        # `ros2 topic list --no-daemon` returned 76 topics, and the ordinary
        # `ros2 topic list` returned ZERO -- a wedged daemon caching an empty
        # graph. The operator was told to check whether the Teensy was
        # attached, twice, and it had been attached the whole time.
        #
        # The process table is the independent observation that separates
        # them, and it costs one `ps`.
        procs = stack_processes()
        print()
        if procs:
            print("  BUT THE STACK IS RUNNING. Found: %s"
                  % ", ".join("%s (pid %s)" % (k, v[0])
                              for k, v in sorted(procs.items())))
            print("  So this is NOT a dead stack and NOT the Teensy -- the")
            print("  graph exists and this process cannot see it.")
            print()
            # DIAGNOSE, DO NOT LIST CANDIDATES. This process can read its own
            # environment, so the commonest cause is not a guess: measured on
            # 2026-09-01, a shell without this variable saw 3 topics and 0
            # messages against 92 and 291 for the same shell with it. Handing
            # the operator a numbered list of things to try, when one of them
            # is decidable here and now, is how the same twenty minutes gets
            # lost three times.
            shm = os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")
            if shm != "SHM":
                print("  >>> THIS IS THE CAUSE: FASTDDS_BUILTIN_TRANSPORTS "
                      "is %s, not SHM." % (repr(shm) if shm else "UNSET"))
                print("      UDP discovery is dead on this host, so this")
                print("      process joined no graph at all. Fix it with:")
                print()
                print("          cd ~/kortex_ws && source scripts/env.sh")
                print()
                print("      then re-run. env.sh is the one source of the")
                print("      environment this rig needs; a bare login shell")
                print("      does not have it.")
                print()
                print("  If that does not fix it, then in order:")
            else:
                print("  The transport is set correctly (SHM), so the likely")
                print("  causes, in order:")
            print()
            print("    1. ros2 daemon stop && ros2 daemon start")
            print("       A wedged daemon caches an empty graph and hands it")
            print("       to every command. Confirm with:")
            print("           ros2 topic list --no-daemon | wc -l")
            print("       If that prints a large number and plain `ros2 topic")
            print("       list` prints nothing, it is the daemon. This is the")
            print("       usual cause and it costs two seconds to rule out.")
            print()
            print("    2. Check the domain matches the stack:")
            print("       ROS_DOMAIN_ID here is %s; compare with"
                  % os.environ.get("ROS_DOMAIN_ID", "unset (= 0)"))
            print("           tr '\\0' '\\n' < /proc/%s/environ | grep "
                  "ROS_DOMAIN_ID"
                  % (sorted(procs.values())[0][0] if procs else "<pid>"))
            print()
            print("    3. Only if both fail: stop the stack, clear")
            print("       /dev/shm/fastrtps_*, and relaunch. Never clear it")
            print("       with the stack up -- that orphans the running")
            print("       stack's own segments and guarantees a restart.")
        else:
            print("  No stack process is running either. Start one first:")
            print("      bash scripts/run_teleop.sh")
            print("  in its own terminal, and wait for the controllers to")
            print("  activate before re-running this.")
    print()
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    ap.add_argument("--start-at", default="",
                    help="resume at this segment label")
    ap.add_argument("--blocks", default="",
                    help="comma-separated subset, e.g. A,B")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--reset-estop", action="store_true",
                    help="call /estop_reset once before the preflight. This "
                         "CLEARS a latch, it does not disable the e-stop: "
                         "/estop stays live and still halts the arms. There "
                         "is no switch that disables it and there should not "
                         "be.")
    ap.add_argument("--gate", choices=("own", "opposite"), default="own",
                    help="which button starts and ends a segment. 'own' (the "
                         "default) is the button on the arm you are moving, "
                         "and pins the clutch so the press cannot disengage "
                         "it. 'opposite' is the older behaviour and needs no "
                         "pin.")
    ap.add_argument("--min-segment-s", type=float, default=0.5,
                    help="a press shorter than this DISCARDS the segment")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--need-real", action="store_true",
                    help="require the sim->real bridge enabled; refuse "
                         "otherwise, so real_* columns are never silently empty")
    ap.add_argument("--need-channels", default="j1,j2,j4,accel",
                    help="channels the current mode cannot run without")
    a = ap.parse_args()
    a.need_channels = [c.strip() for c in a.need_channels.split(",") if c.strip()]

    segs = segments()
    if a.blocks:
        keep = {b.strip().upper() for b in a.blocks.split(",")}
        segs = [s for s in segs if s["block"] in keep]
    if a.start_at:
        labels = [s["label"] for s in segs]
        if a.start_at not in labels:
            print("no such segment: %s" % a.start_at)
            return 2
        segs = segs[labels.index(a.start_at):]

    n, secs = estimate(segs)
    print("TRAJECTORY CAPTURE - %d segments, estimated %.0f min" % (n, secs / 60))
    print("Each segment: press the button on the %s to start, again to end. "
          "A press under %.1fs discards and re-prompts."
          % ("ARM YOU ARE MOVING" if a.gate == "own" else "OPPOSITE arm",
             a.min_segment_s))
    print()
    if a.dry_run:
        for s in segs:
            print("  %-22s %s" % (s["label"], s["instruction"]))
        return 0

    out = Path(a.out or (Path.home() / "kortex_ws" / "recordings" /
                         "trajectory_capture" /
                         time.strftime("capture_%Y%m%d_%H%M%S")))
    master = out / "all_segments.csv"

    rclpy.init()
    node = Capture()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    time.sleep(3.0)

    # THE ARMS THIS RUN ACTUALLY SWEEPS, derived from the selected segments
    # rather than assumed to be both. `--blocks G` sweeps both; a
    # single-arm subset must not be refused for an arm it never touches.
    need_arms = tuple(a for a in ARMS if any(x["arm"] == a for x in segs))

    if not preflight(node, need_arms):
        print("PRE-CAPTURE CHECK FAILED - fix the above before recording. "
              "Nothing was written.")
        _teardown(ex, node, spin_thread)
        return 1

    # THE CLUTCH PIN. Own-arm gating is only safe while a button press
    # cannot toggle the clutch, so this is a precondition, not a nicety.
    node.gate_index = dict(GATE_INDEX if a.gate == "own"
                           else GATE_INDEX_OPPOSITE)
    if a.reset_estop:
        ok_r, why_r = reset_estop()
        print("e-stop reset" if ok_r else
              "could not reset the e-stop: %s" % why_r)
        t_r = time.time()
        while node.estop and time.time() - t_r < 3.0:
            time.sleep(0.1)
    pinned = False
    if a.gate == "own":
        ok_pin, why = pin_clutch(True, node, need_arms)
        if not ok_pin:
            print("CANNOT PIN THE CLUTCH: %s" % why)
            print()
            print("Own-arm gating without the pin is the 2026-08-06 defect:")
            print("every press would toggle the clutch of the arm being")
            print("swept, and 41 of 42 segments recorded against a frozen")
            print("arm. Refusing rather than recording that again.")
            print()
            print("Either restart the stack, or run with --gate opposite to")
            print("use the OTHER arm's button, which needs no pin.")
            _teardown(ex, node, spin_thread)
            return 1
        pinned = True
        print("clutch PINNED engaged for this capture "
              "(force_clutch_engaged=true, confirmed on both arms)")
        print("press the button on the arm you are MOVING.")
        print()

    # NOTHING ON DISK UNTIL HERE. Every refusal above returns before the
    # directory exists, so an aborted run leaves no trace at all. It used to
    # create the directory first, and a refused run left a header-only
    # all_segments.csv beside a manifest -- indistinguishable at the analysis
    # from a session that ran and recorded nothing, which is a completely
    # different fault.
    out.mkdir(parents=True, exist_ok=True)

    meta = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"),
                rate_hz=a.rate, segments=[s["label"] for s in segs],
                gate_mode=a.gate, clutch_pinned=pinned,
                # DERIVED FROM GATE_INDEX, not typed. The typed version of
                # this string said "left arm gated by btn2, right arm by
                # btn1" -- the exact inversion the constant exists to
                # prevent, and the inversion that made the 2026-08-06
                # capture unusable. Anyone reading a capture's metadata to
                # check whether that recurred would have concluded it had.
                gate_mapping="; ".join(
                    "%s arm gated by btn%d" % (a, node.gate_index[a] - 1)
                    for a in ARMS)
                + (" (MEASURED); each arm is gated by ITS OWN button, "
                   "which is safe only because the clutch is PINNED "
                   "engaged for this capture and the pin was confirmed"
                   if a.gate == "own" else
                   " (MEASURED); each arm is gated by the OPPOSITE arm's "
                   "button, because its own also toggles its clutch and no "
                   "pin is in force"),
                pre_labelling="none - every channel recorded identically, no "
                              "expected_flat flag, so the live/dead verdict "
                              "comes from this capture alone")
    (out / "manifest.json").write_text(json.dumps(meta, indent=2))

    fh = open(master, "w", newline="")
    w = csv.DictWriter(fh, fieldnames=COLUMNS)
    w.writeheader()

    dt = 1.0 / a.rate
    done = 0
    try:
        for s in segs:
            while True:
                print("-" * 72)
                print("[%d/%d] %s" % (done + 1, len(segs), s["label"]))
                print("   %s" % s["instruction"])
                blk = segment_blockers(node, s["arm"], a.need_real,
                                       a.need_channels)
                if blk:
                    print("   REFUSING TO START:")
                    for x in blk:
                        print("      - %s" % x)
                    print("   fix it, then press the %s button to retry"
                          % gate_word(node, s["arm"]))
                    g2 = Gate(node, s["arm"])
                    if not g2.wait_press():
                        raise KeyboardInterrupt
                    continue
                print("   press the %s button to START"
                      % gate_word(node, s["arm"]))
                gate = Gate(node, s["arm"])
                if not gate.wait_press():
                    raise KeyboardInterrupt
                t0 = time.monotonic()
                print("   RECORDING ... press again to end")
                buf = []
                idx = 0
                nxt = time.monotonic()
                aborted = None
                while rclpy.ok():
                    buf.append(node.row(s, idx))
                    idx += 1
                    if idx % 25 == 0:
                        blk = segment_blockers(node, s["arm"], a.need_real,
                                               a.need_channels)
                        if blk:
                            aborted = blk
                            break
                    if gate.pressed():
                        break
                    nxt += dt
                    time.sleep(max(0.0, nxt - time.monotonic()))
                if aborted:
                    print("   ABORTED mid-segment, NOT saved:")
                    for x in aborted:
                        print("      - %s" % x)
                    print("   re-prompting this segment")
                    continue
                dur = time.monotonic() - t0
                if dur < a.min_segment_s:
                    print("   DISCARDED (%.2fs < %.2fs) - nothing written, "
                          "re-prompting" % (dur, a.min_segment_s))
                    continue
                for r in buf:
                    w.writerow(r)
                fh.flush()
                seg_path = out / ("%s.csv" % s["label"])
                with open(seg_path, "w", newline="") as sf:
                    sw = csv.DictWriter(sf, fieldnames=COLUMNS)
                    sw.writeheader()
                    sw.writerows(buf)
                print("   saved %d rows, %.1fs -> %s"
                      % (len(buf), dur, seg_path.name))
                # DID THE PIN HOLD? Checked from the rows just written rather
                # than assumed from the parameter. A segment recorded with
                # the clutch out has a frozen commanded pose and cannot
                # measure the gain it exists to measure -- and finding that
                # here costs one repeat, where finding it at the analysis
                # costs the session.
                bad, why = clutch_was_out(buf, s["arm"])
                if bad:
                    print("   *** %s" % why)
                    print("   *** the file is kept, and it is MARKED. Redo "
                          "this segment with --start-at %s" % s["label"])
                done += 1
                break
    except KeyboardInterrupt:
        print("\ninterrupted - %d segment(s) saved to %s" % (done, out))
    finally:
        fh.close()
        if pinned:
            # RESTORE, on every exit path including Ctrl-C. Leaving the
            # clutch pinned engaged after a capture means the next
            # operator's button does nothing and nothing says why.
            #
            # AND A SECOND Ctrl-C MUST NOT ESCAPE THIS. The release used to
            # shell out to `ros2 param set` with a 20 s timeout, so an
            # impatient second interrupt landed inside subprocess.communicate
            # and unwound the whole clean-up: the operator got a
            # KeyboardInterrupt traceback instead of the one line telling
            # them the clutch was still pinned. Whatever happens here, say
            # what state the clutch was left in.
            try:
                okr, whyr = pin_clutch(False, node)
            except KeyboardInterrupt:
                okr, whyr = False, "interrupted before the release completed"
            except Exception as exc:                        # noqa: BLE001
                okr, whyr = False, str(exc)
            print("clutch pin released" if okr else
                  "WARNING: could not release the clutch pin: %s\n"
                  "         set force_clutch_engaged=false by hand:\n"
                  "             ros2 param set /master_pose_node "
                  "force_clutch_engaged false\n"
                  "         or restart the master node." % whyr)
        _teardown(ex, node, spin_thread)
    print("\nDONE: %d/%d segments -> %s" % (done, len(segs), out))
    print("Analyse with:")
    print("  python3 src/srl_experiments/trajectory_capture/analyse_capture.py %s"
          % master)
    return 0


if __name__ == "__main__":
    sys.exit(main())
