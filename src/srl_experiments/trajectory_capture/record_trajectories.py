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

TWO THINGS THAT ARE EASY TO GET WRONG HERE, both already paid for:

  * EVERY SEGMENT IS GATED BY THE **OPPOSITE** ARM'S BUTTON. An arm's own
    button also toggles that arm's clutch, so gating a left sweep with the
    left button disengages the clutch exactly when the sweep starts.
    Measured mapping: left button -> btn2, right button -> btn1.

  * DO NOT RUN THIS INSIDE THE LAUNCH. `ros2 launch` merges and line-prefixes
    all stdout, so RViz and MoveIt bury the prompt and the prefixes break the
    in-place redraw. Run it in its own terminal.

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
# MEASURED: left button -> btn2, right button -> btn1. The gate for an arm is
# therefore the OTHER arm's button.
# Message layout is [fsr1, fsr2, btn1, btn2], so index 2 = btn1 and 3 = btn2.
# MEASURED: the LEFT arm's own clutch button is btn2, the RIGHT arm's is btn1.
# The gate for an arm must therefore be the OTHER arm's button:
#     left  segment -> btn1 -> index 2
#     right segment -> btn2 -> index 3
#
# THIS WAS INVERTED in the 2026-08-06 capture. Every segment was gated by the
# button belonging to the arm under test, so starting and ending a sweep
# toggled that arm's own clutch -- disengaging it for the duration of the
# sweep. 41 of 42 directional segments recorded with the clutch OUT and the
# sim arm therefore stationary, which made the whole capture unusable for
# gain, repeatability and rate. Rapid button presses also tripped the
# both-buttons-within-0.4s e-stop, latching it for all of block F.
GATE_INDEX = {"left": 2, "right": 3}

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
        """The gate for `arm` is the OPPOSITE arm's button. See module docs."""
        with self.lock:
            f = list(self.fsr) if self.fsr else None
        if not f or len(f) < 4:
            return None
        return bool(f[GATE_INDEX[arm]] > 0.5)

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
    """Rising-edge detector on one arm's gate button.

    Edges, not levels: the press that STARTS a segment must not also end it,
    and the operator's finger stays down for an unknown time. Each call to
    rose() consumes at most one edge.
    """

    def __init__(self, node, arm):
        self.node, self.arm = node, arm
        self.prev = node.button(arm)

    def rose(self):
        b = self.node.button(self.arm)
        if b is None:
            return False
        r = bool(b and self.prev is False)
        self.prev = b
        return r

    def wait_rise(self, timeout=None):
        t0 = time.monotonic()
        while rclpy.ok():
            if self.rose():
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
        out.append("E-STOP LATCHED - the arm cannot follow; reset it")
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


def preflight(node):
    hz = node.joint_states_hz()
    ok = True
    print("PRE-CAPTURE CHECK")
    print("  /joint_states     %.1f Hz %s" % (hz, "" if hz > 20 else "<-- TOO LOW"))
    ok &= hz > 20
    have = [a for a in ARMS if node.raw.get(a)]
    print("  master raw        %s" % (", ".join(have) if have else "NONE"))
    if not have:
        print("     no /master_arm_raw_* - is the Teensy attached and "
              "master_pose_node running?")
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
            print("  graph exists and this process cannot see it. In order:")
            print()
            print("    1. ros2 daemon stop && ros2 daemon start")
            print("       A wedged daemon caches an empty graph and hands it")
            print("       to every command. Confirm with:")
            print("           ros2 topic list --no-daemon | wc -l")
            print("       If that prints a large number and plain `ros2 topic")
            print("       list` prints nothing, it is the daemon. This is the")
            print("       usual cause and it costs two seconds to rule out.")
            print()
            print("    2. source scripts/env.sh")
            print("       UDP discovery is dead on this host, so a shell")
            print("       without FASTDDS_BUILTIN_TRANSPORTS=SHM joins")
            print("       nothing. env.sh is the one source of that truth.")
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
    print("Each segment: press the OPPOSITE arm's button to start, again to "
          "end. A press under %.1fs discards and re-prompts." % a.min_segment_s)
    print()
    if a.dry_run:
        for s in segs:
            print("  %-22s %s" % (s["label"], s["instruction"]))
        return 0

    out = Path(a.out or (Path.home() / "kortex_ws" / "recordings" /
                         "trajectory_capture" /
                         time.strftime("capture_%Y%m%d_%H%M%S")))
    out.mkdir(parents=True, exist_ok=True)
    master = out / "all_segments.csv"

    rclpy.init()
    node = Capture()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3.0)

    if not preflight(node):
        print("PRE-CAPTURE CHECK FAILED - fix the above before recording. "
              "Nothing was written.")
        ex.shutdown()
        rclpy.shutdown()
        return 1

    meta = dict(started=time.strftime("%Y-%m-%dT%H:%M:%S"),
                rate_hz=a.rate, segments=[s["label"] for s in segs],
                # DERIVED FROM GATE_INDEX, not typed. The typed version of
                # this string said "left arm gated by btn2, right arm by
                # btn1" -- the exact inversion the constant exists to
                # prevent, and the inversion that made the 2026-08-06
                # capture unusable. Anyone reading a capture's metadata to
                # check whether that recurred would have concluded it had.
                gate_mapping="; ".join(
                    "%s arm gated by btn%d" % (a, GATE_INDEX[a] - 1)
                    for a in ARMS)
                + " (MEASURED); each arm is gated by the OPPOSITE button "
                  "because its own also toggles its clutch",
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
                          % ("RIGHT" if s["arm"] == "left" else "LEFT"))
                    g2 = Gate(node, s["arm"])
                    if not g2.wait_rise():
                        raise KeyboardInterrupt
                    continue
                print("   press the %s button to START"
                      % ("RIGHT" if s["arm"] == "left" else "LEFT"))
                gate = Gate(node, s["arm"])
                if not gate.wait_rise():
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
                    if gate.rose():
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
                done += 1
                break
    except KeyboardInterrupt:
        print("\ninterrupted - %d segment(s) saved to %s" % (done, out))
    finally:
        fh.close()
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("\nDONE: %d/%d segments -> %s" % (done, len(segs), out))
    print("Analyse with:")
    print("  python3 src/srl_experiments/trajectory_capture/analyse_capture.py %s"
          % master)
    return 0


if __name__ == "__main__":
    sys.exit(main())
