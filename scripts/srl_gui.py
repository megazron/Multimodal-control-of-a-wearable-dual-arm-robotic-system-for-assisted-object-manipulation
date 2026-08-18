#!/usr/bin/env python3
"""SRL operations GUI -- DUAL VIEW: commanded beside actual, and the gap named.

    python3 scripts/srl_gui.py                 # both panels
    python3 scripts/srl_gui.py --single-rviz   # fallback: one panel, ghosted
    python3 scripts/srl_gui.py --no-rviz       # indicators only (capture/CI)

WHY TWO PANELS. The cascade drives the real arms from the sim arms, and the
lag monitor trips at `lag_trip_rad`. Until now that threshold was a number
nobody could see: it fired, or it did not, and the operator found out from an
e-stop. Two views plus a divergence readout make the quantity the monitor acts
on continuously visible, next to the thing it describes.

  LEFT   COMMANDED -- the sim arms, driven by whichever mode is active
  RIGHT  ACTUAL    -- the real arms, from /real/joint_states and the `real_*`
                     frames in the SHARED /tf. THERE IS NO /real/tf; both real
                     launches use frame_prefix="real_" into the one tree. See
                     the note in Bus.__init__.

RVIZ EMBEDDING is X11 reparenting: QWindow.fromWinId() wrapped by
QWidget.createWindowContainer(). QX11EmbedContainer was removed in Qt5. This
is verified from pixels, not from a return code -- a container can be visible
and empty. rviz2 stays a separate process, which costs styling and a brief
flash before reparenting and buys the property that matters most here: an
RViz crash does not take down the tool used to diagnose everything else.
The two instances are told apart by DIFFING the window list around each
launch, because both present as "RViz" and matching on the title would
reparent whichever appeared first into both panels.

THE DISCIPLINE THIS GUI IS BUILT ON, restated because it is the reason for
most of the code below. EVERY INDICATOR THAT CAN SHOW "NOTHING WRONG" MUST BE
ABLE TO PROVE IT IS NOT SHOWING "NOT CHECKED". This GUI once reported
"IK BLOCKED -- 0% success" computed over ZERO attempts, which sent the
operator to the solver instead of to the missing input. So:

  * a value with no data behind it renders "--" in PURPLE (unknown), never a
    zero and never a colour that means healthy;
  * divergence has SEVEN statuses and only one of them carries a number
    (see srl_teleop/divergence.py) -- a real arm that is not publishing reads
    "NO REAL ARM", not 0.000 rad;
  * a camera past its staleness limit is not painted at all, because a frozen
    picture of a workspace cannot be told from a live picture of a workspace
    that is not moving;
  * a button whose target cannot be resolved is DISABLED with the reason on
    its tooltip, because five buttons in an earlier build exited 2 on press
    while appearing to launch;
  * SELF-TEST (bottom bar) drives synthetic bad data through the real
    formatters and asserts each one changes -- so "all clear" can be shown to
    be a reading rather than a stuck widget.

THREADING. ROS owns one thread and every subscription. The GUI reads ONE
immutable snapshot dict, rebound atomically; it never calls a service, never
waits and never locks. Every control action is queued to the ROS thread and
answered by callback. Parameter writes go through parameter CLIENTS, never
`ros2 param set`, which goes via the daemon -- the daemon hangs on this box
and has reported success it had not earned.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, JointState

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_WS, "src/srl_teleop"))
from srl_teleop import camera_relay as cr                    # noqa: E402
from srl_teleop import divergence as dv                      # noqa: E402
from srl_teleop import gui_launch_specs as gls               # noqa: E402
from srl_teleop import precision_speed as ps                 # noqa: E402
# THE STRAY-PUBLISHER SCAN, WHICH HAD NEVER RUN. `_foreign_description()`
# calls `procscan.find(...)` and this module never imported it, so the
# stack-starting preflight raised NameError -- inside a Qt slot, where PyQt
# prints a traceback to a stderr nobody reads and returns. "Sim teleop only"
# and "+ perception and shared autonomy" therefore did NOTHING when pressed,
# silently. Found by `verify_gui_buttons` on 2026-08-18, the first time that
# audit got past the modal consent dialog.
#
# What the check is FOR is HARD CONSTRAINT 3: a stray `robot_state_publisher`
# does the same damage as a second stack and is not caught by the process
# count. The guard against it has been dead for as long as it has existed.
from srl_teleop import procscan                               # noqa: E402

from PyQt5.QtCore import Qt, QTimer                          # noqa: E402
from PyQt5.QtGui import (QColor, QFont, QImage, QPalette,    # noqa: E402
                         QPixmap, QWindow)
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox,  # noqa: E402
                             QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QPushButton, QScrollArea, QSizePolicy, QSlider,
                             QSplitter, QTabWidget, QTextEdit,
                             QLineEdit,
                             QVBoxLayout, QWidget)

ARMS = ("left", "right")

# ISA-101: colour is a scarce alarm channel, so a normal state is largely
# UNCOLOURED rather than green. Saturated colour is reserved for the abnormal.
# DARK HUD. One accent hue carries the whole normal state; saturation and
# glow are spent only on the abnormal. See scripts/srl_hud.py for the full
# argument -- the short version is that a film-prop interface can afford to
# glow everywhere because nothing on it has to be diagnosed, and this one
# cannot. ISA-101 survives the aesthetic unchanged.
from srl_hud import (ReadyPanel, ACCENT, BAD, BG, LINE, MUTED, PANEL, TEXT,   # noqa: E402
                     UNKNOWN, WARN, MasterArmSchematic, RobotSchematic,
                     Strip, mono, sans)

C_BG = BG
C_TEXT = TEXT
C_MUTED = MUTED
C_BAD = BAD
C_WARN = WARN
C_OK = ACCENT              # "healthy" is the accent, not a second colour
C_UNKNOWN = UNKNOWN        # distinct from bad: not knowing is its own state

# ONE STYLESHEET, so every widget inherits the dark field rather than each
# panel painting its own. Flat borders and no radius: a hairline rule reads as
# structure, a rounded filled box reads as a button.
STYLE = """
QWidget { background: %(bg)s; color: %(text)s; }
QGroupBox { border: 1px solid %(line)s; margin-top: 14px; padding-top: 8px;
            background: %(panel)s; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px;
                   color: %(muted)s; font-size: 8pt; }
QPushButton { background: %(panel)s; border: 1px solid %(line)s;
              padding: 5px 8px; color: %(text)s; }
QPushButton:hover { border: 1px solid %(accent)s; color: %(accent)s; }
QPushButton:disabled { color: #33424d; border: 1px solid #131c24; }
QScrollArea, QScrollArea > QWidget > QWidget { background: %(bg)s; }
QSlider::groove:horizontal { height: 2px; background: %(line)s; }
QSlider::handle:horizontal { background: %(accent)s; width: 8px;
                             margin: -5px 0; }
QCheckBox { color: %(text)s; }
QSplitter::handle { background: %(line)s; }
QLabel { background: transparent; }
""" % dict(bg=BG, text=TEXT, line=LINE, panel=PANEL, muted=MUTED,
           accent=ACCENT)

# Colour per divergence band. `band()` lives in divergence.py so the GUI and
# the lag monitor cannot quietly disagree about where the threshold is.
BAND_COLOUR = {"ok": C_TEXT, "watch": C_MUTED, "near": C_WARN,
               "trip": C_BAD, "unknown": C_UNKNOWN}

RVIZ_TEMPLATE = os.path.join(
    _WS, "src/srl_experiments/config/verification_capture.rviz")


def helvetica(size=11, bold=False):
    """Three type levels, and NUMBERS ARE MONOSPACE EVERYWHERE.

    A proportional font reflows as digits change, so a value updating at
    10 Hz slides sideways and the eye chases it instead of reading it. Fixed
    advance widths hold the layout still, which is what makes a changing
    digit visible in peripheral vision -- the whole point of a status panel.
    """
    return sans(size, bold)


# ===========================================================================
#  ROS SIDE
# ===========================================================================
class Bus(Node):
    """Every subscription, service call and parameter write lives here."""

    def __init__(self):
        super().__init__("srl_gui")
        self.snap = dict(t=time.monotonic())
        self._d = {}
        self._q = []
        self.log = []
        # Divergence trackers. Two Sides per arm; see divergence.py for why
        # arrival and CHANGE are tracked separately.
        self.sim = {a: dv.Side("sim/%s" % a) for a in ARMS}
        self.real = {a: dv.Side("real/%s" % a) for a in ARMS}
        # THE TRIP THRESHOLD IS READ FROM THE BRIDGE, NOT ASSUMED. A panel
        # that colours divergence against a hardcoded 0.5 while the bridge
        # runs at 0.3 is a panel that says "fine" up to the moment the arm
        # stops. Until it is read, the header says so.
        self.lag_trip = 0.5
        self.lag_trip_src = "default -- bridge not read"
        self._raw_prev, self._raw_age = {}, {}
        # Stored channel health, and which channels degraded mode has FROZEN.
        # Frozen is drawn as its own state because eight repaired channels
        # once stayed frozen behind a stale baseline with nothing saying so.
        try:
            from srl_teleop import degraded_mode as _dg
            self._baseline = _dg.load_baseline(_dg.default_baseline_path())
            self._dg = _dg
        except Exception:                                     # noqa: BLE001
            self._baseline, self._dg = {}, None
        # ONE TF BUFFER, AND THE REASON IS A MEASUREMENT.
        #
        # The brief specifies the actual arms come from "/real/joint_states and
        # /real/tf". `/real/joint_states` exists. `/real/tf` DOES NOT -- checked
        # on a running mock stack, `ros2 topic list` shows /real/joint_states,
        # /real/<arm>_arm_controller/joint_trajectory and
        # /realmock/robot_description, and no /real/tf at all.
        #
        # Both real launches (real_arms.launch.py:83, mock_real.launch.py:57)
        # run robot_state_publisher with namespace="real" and
        # frame_prefix="real_", which puts the real arm's transforms in the
        # SHARED /tf tree under prefixed frame names -- `real_world`,
        # `real_left_end_effector_link` -- joined to `world` by a static
        # transform. So there is one tree with two robots in it, not two trees,
        # and subscribing to a topic that does not exist would have produced a
        # permanently empty ACTUAL panel that looked like an embedding failure.
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.real_desc_topic = None       # discovered, not assumed
        self._sub()
        self.create_timer(0.1, self._snapshot)
        self.create_timer(0.05, self._drain)
        self.create_timer(5.0, self._read_lag_trip)

    # ---------------------------------------------------------- subscribe
    def _sub(self):
        from rclpy.qos import qos_profile_sensor_data
        for a in ARMS:
            self.create_subscription(
                String, "/master_capability_%s" % a,
                lambda m, k="cap_%s" % a: self._set(k, m.data), 10)
            self.create_subscription(
                Float64MultiArray, "/ik_status_%s" % a,
                lambda m, k="ik_%s" % a: self._set(k, list(m.data)), 10)
            self.create_subscription(
                String, "/master_status_%s" % a,
                lambda m, k="mstat_%s" % a: self._set(k, m.data), 10)
            # RAW master line: [j1..j7 deg, ax, ay, az, gx, gy, gz]. The
            # schematic needs the RAW vector, never the validated one -- a
            # substituted value is exactly what must be visible as stale.
            self.create_subscription(
                Float64MultiArray, "/master_arm_raw_%s" % a,
                lambda m, k="raw_%s" % a: self._on_raw(k, list(m.data)), 20)
        for t, k in (("/scene/state", "scene"),
                     ("/blocking_summary", "blocking"),
                     ("/master_channel_state", "chan"),
                     ("/autonomy_decision", "autonomy"),
                     ("/vr_state", "vr"),
                     ("/gripper_reference", "grip"),
                     ("/recovery_state", "recovery"),
                     ("/trial_state", "trial"),
                     # Part 6: the go/no-go verdict and the live session.
                     ("/session/ready", "ready"),
                     ("/session/state", "sess")):
            self.create_subscription(String, t,
                                     lambda m, k=k: self._set(k, m.data), 10)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: self._set("fsr", list(m.data)), 10)
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: self._set("estop", m.data), 10)

        # THE TWO JOINT STREAMS. Both go through divergence.Side, which keeps
        # arrival and content-change apart -- /real/joint_states is known to
        # FREEZE at full rate rather than stop when the hardware component
        # goes inactive, and arrival-rate freshness cannot see that.
        self.create_subscription(JointState, "/joint_states",
                                 self._on_sim_js, 20)
        self.create_subscription(JointState, "/real/joint_states",
                                 self._on_real_js, 20)

        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, DurabilityPolicy
        static_qos = QoSProfile(depth=200,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf",
                                 lambda m: self._on_tf(m, False), 200)
        self.create_subscription(TFMessage, "/tf_static",
                                 lambda m: self._on_tf(m, True), static_qos)

        # ONE MORE SUBSCRIBER, NEVER A SECOND DEVICE OWNER. The vendor vision
        # driver owns the camera; this sits beside the detector on its topic.
        # depth=1 best-effort: a viewer wants the NEWEST frame, and a reliable
        # queue would deliver a backlog after any hiccup -- exactly the stale
        # view this relay exists to prevent.
        self.cam = {a: cr.ChannelState() for a in ARMS}
        self.cam_img = {a: None for a in ARMS}
        for a in ARMS:
            for topic in ("/%s_wrist_camera/image_raw" % a,
                          "/wrist_mounted_camera/%s/image" % a):
                self.create_subscription(
                    Image, topic, (lambda m, arm=a: self._on_image(arm, m)),
                    qos_profile_sensor_data)

    def _on_raw(self, key, data):
        """Track each channel's last DISTINCT value, not its last arrival.

        The dead j7 pot, the frozen /real/joint_states and the 0.000 noise
        floor were all the same shape: a value arriving at full rate and never
        changing. Age-since-change is the only quantity that separates a live
        channel from a republished one, so it is what the schematic draws.
        """
        now = time.monotonic()
        prev = self._raw_prev.setdefault(key, [None] * 7)
        ages = self._raw_age.setdefault(key, [None] * 7)
        for i in range(min(7, len(data))):
            if prev[i] is None or abs(data[i] - prev[i]) > 1e-9:
                prev[i] = data[i]
                ages[i] = now
        self._set(key, data)

    def _on_sim_js(self, m):
        for a in ARMS:
            self.sim[a].update(m.name, m.position)

    def _on_real_js(self, m):
        for a in ARMS:
            self.real[a].update(m.name, m.position)

    def _on_tf(self, msg, static):
        for t in msg.transforms:
            try:
                if static:
                    self.buf.set_transform_static(t, "srl_gui")
                else:
                    self.buf.set_transform(t, "srl_gui")
            except Exception:                                 # noqa: BLE001
                pass

    def ee(self, which, arm):
        """(x, y, z) of an end effector in `world`, or None.

        `which` selects the frame PREFIX, not a separate tree: the real arm
        lives in the same /tf under `real_*` frames (see the note above).

        None is returned for EVERY failure -- missing frame, broken chain,
        extrapolation. It must never fall back to a previous value: a stale
        transform is exactly the frozen-cache reading this GUI exists to make
        impossible, and here it would land inside a DISTANCE, where it looks
        like excellent tracking.
        """
        frame = ("%s_end_effector_link" % arm if which == "sim"
                 else "real_%s_end_effector_link" % arm)
        try:
            tr = self.buf.lookup_transform("world", frame, rclpy.time.Time())
        except Exception:                                     # noqa: BLE001
            return None
        v = tr.transform.translation
        return (v.x, v.y, v.z)

    def _read_lag_trip(self):
        if self.lag_trip_src.startswith("bridge"):
            return
        from rcl_interfaces.srv import GetParameters
        for node in ("/sim_to_real_bridge_left", "/sim_to_real_bridge"):
            cli = self.create_client(GetParameters, "%s/get_parameters" % node)
            if not cli.service_is_ready():
                continue
            req = GetParameters.Request()
            req.names = ["lag_trip_rad"]
            fut = cli.call_async(req)

            def done(f, node=node):
                try:
                    v = f.result().values[0].double_value
                except Exception:                             # noqa: BLE001
                    return
                if v > 0:
                    self.lag_trip = float(v)
                    self.lag_trip_src = "bridge %s" % node
                    self.note("lag_trip_rad = %.3f read from %s" % (v, node))
            fut.add_done_callback(done)
            return

    def _on_image(self, arm, msg):
        self.cam[arm].on_frame(msg.width, msg.height, msg.encoding)
        # Keep the RAW buffer; convert on the GUI thread only when it will
        # actually be painted, so ROS-thread time is not spent on frames the
        # GUI is about to discard as stale.
        self.cam_img[arm] = (msg.width, msg.height, msg.encoding,
                             bytes(msg.data), msg.step)

    def _set(self, k, v):
        self._d[k] = (v, time.monotonic())

    # ----------------------------------------------------------- snapshot
    def _snapshot(self):
        s = {k: v for k, v in self._d.items()}
        s["t"] = time.monotonic()
        s["topics"] = {t for t, _ in self.get_topic_names_and_types()}
        s["cam"] = {a: (self.cam[a].caption(), self.cam[a].state(),
                        self.cam[a].show_image(), self.cam[a].hz())
                    for a in ARMS}
        s["cam_img"] = dict(self.cam_img)
        # The sim is "moving" if any sim joint changed recently. Passed to
        # compare() so a STILL real arm beside a STILL sim arm is not called
        # frozen -- a false FROZEN costs a glance, a missed one costs the
        # belief that the arm is tracking.
        s["div"] = {}
        for a in ARMS:
            ca = self.sim[a].change_age()
            moving = (ca is not None and ca < 1.0)
            r = dv.compare(self.sim[a], self.real[a], dv.joint_names(a),
                           sim_is_moving=moving)
            # EE distance is attached even when the joint comparison failed,
            # because the two can fail independently: TF can be healthy while
            # a joint is missing from the message, and vice versa. It is only
            # DISPLAYED on a measured row, but computing it regardless keeps
            # the two failures separable in the snapshot.
            r.ee_m, r.ee_reason = dv.ee_distance(self.ee("sim", a),
                                                 self.ee("real", a))
            s["div"][a] = r
        s["lag_trip"] = self.lag_trip
        s["lag_trip_src"] = self.lag_trip_src
        s["master_schema"] = self._master_schema()
        s["robot_schema"] = self._robot_schema()
        s["log"] = list(self.log[-200:])
        self.snap = s

    # ------------------------------------------------------- schematics
    def _frozen_idx(self, arm):
        raw = (self._d.get("chan") or ("", 0))[0]
        if not raw or "DEGRADED" not in raw:
            return set()
        for part in raw.split("|"):
            if part.strip().startswith(arm):
                m = re.search(r"frozen j([0-9]+)", part)
                if m:
                    return {int(c) - 1 for c in m.group(1)}
        return set()

    def _master_schema(self):
        now = time.monotonic()
        out = {}
        fsr = (self._d.get("fsr") or (None, 0))[0]
        for arm in ARMS:
            raw = (self._d.get("raw_%s" % arm) or (None, 0))[0]
            ages = self._raw_age.get("raw_%s" % arm) or [None] * 7
            frozen = self._frozen_idx(arm)
            joints = []
            for i in range(7):
                if i in frozen:
                    health = "FROZEN"
                elif self._dg is not None and self._baseline:
                    health = self._dg.verdict(self._baseline, arm, i)
                else:
                    health = "UNKNOWN"
                joints.append(dict(
                    health=health,
                    deg=(raw[i] if raw and len(raw) > i else None),
                    age_s=(None if ages[i] is None else now - ages[i])))
            g = None
            gy = None
            if raw and len(raw) >= 10:
                g = math.sqrt(sum(raw[7 + k] ** 2 for k in range(3)))
            if raw and len(raw) >= 13:
                gy = math.sqrt(sum(raw[10 + k] ** 2 for k in range(3)))
            st = (self._d.get("mstat_%s" % arm) or (None, 0))[0]
            clutch = None
            if st is not None:
                clutch = bool(re.search(r"\bENGAGED\b", str(st))) and not \
                    re.search(r"\bDISENGAGED\b", str(st))
            cap = (self._d.get("cap_%s" % arm) or (None, 0))[0]
            rung = regain = None
            if cap:
                try:
                    cd = json.loads(cap)
                    rung = cd.get("key")
                    regain = cd.get("regain") or cd.get("next_repair")
                except Exception:                             # noqa: BLE001
                    pass
            idx = 0 if arm == "left" else 1
            out[arm] = dict(
                joints=joints, accel_g=g, gyro_dps=gy,
                fsr=(fsr[idx] if fsr and len(fsr) > idx else None),
                latched=None,
                button=(bool(fsr[2 + idx]) if fsr and len(fsr) > 2 + idx
                        else None),
                clutch=clutch, rung=rung,
                regain=(regain or ("repair l_j2 and l_j4 -> SPHERICAL"
                                   if arm == "left" else "")))
        return out

    # Gen3 limits: joints 2/4/6 are limited, 1/3/5/7 continuous.
    LIMIT = {1: 2.41, 3: 2.66, 5: 2.23}

    def _robot_schema(self):
        out = {}
        for arm in ARMS:
            side = self.sim[arm]
            joints = []
            for i in range(7):
                v = side.pos.get("%s_joint_%d" % (arm, i + 1))
                if v is None:
                    joints.append(dict(frac=None, wraps=None))
                    continue
                if i in self.LIMIT:
                    lim = self.LIMIT[i]
                    frac = min(1.0, max(0.0, (v + lim) / (2 * lim)))
                    joints.append(dict(frac=frac, wraps=None))
                else:
                    wrapped = (v + math.pi) % (2 * math.pi) - math.pi
                    joints.append(dict(
                        frac=min(1.0, max(0.0, (wrapped + math.pi)
                                          / (2 * math.pi))),
                        wraps=int(abs(v - wrapped) / (2 * math.pi) + 0.4)))
            ik = (self._d.get("ik_%s" % arm) or (None, 0))[0]
            knu = side.pos.get("%s_robotiq_85_left_knuckle_joint" % arm)
            out[arm] = dict(
                joints=joints,
                aperture_rad=knu,
                object_rad=0.42,
                clearance_m=(ik[5] if ik and len(ik) > 5 else None),
                floor_m=0.12)
        return out

    def note(self, text, bad=False):
        self.log.append((time.strftime("%H:%M:%S"), text, bad))
        (self.get_logger().error if bad else self.get_logger().info)(text)

    # ------------------------------------------------------------ actions
    def submit(self, fn):
        self._q.append(fn)

    def _drain(self):
        while self._q:
            fn = self._q.pop(0)
            try:
                fn()
            except Exception as e:                            # noqa: BLE001
                self.note("action failed: %r" % (e,), bad=True)

    def publish_once(self, msg_type, topic, value):
        p = self.create_publisher(msg_type, topic, 10)
        m = msg_type()
        m.data = value
        p.publish(m)

    def call_trigger(self, name):
        cli = self.create_client(Trigger, name)
        if not cli.service_is_ready():
            # NEVER wait_for_service HERE. Blocking the ROS thread on a
            # service that may not exist is the 4-second e-stop stall, and
            # this thread also carries the e-stop's own publish.
            self.note("%s: service not present -- nothing was sent" % name,
                      bad=True)
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.note("%s -> %s" % (name, _res(f))))

    def set_param(self, node, name, value):
        """Parameter CLIENT, read back, and LOUD on failure.

        `ros2 param set` goes through the ros2 daemon, which hangs on this box
        and has reported success it had not earned. A silent failure here
        means the operator believes a limit was applied when it was not.
        """
        cli = self.create_client(SetParameters, "%s/set_parameters" % node)
        if not cli.service_is_ready():
            self.note("%s: no set_parameters service -- %s UNCHANGED"
                      % (node, name), bad=True)
            return
        p = Parameter()
        p.name = name
        v = ParameterValue()
        if isinstance(value, bool):
            v.type = ParameterType.PARAMETER_BOOL
            v.bool_value = value
        else:
            v.type = ParameterType.PARAMETER_DOUBLE
            v.double_value = float(value)
        p.value = v
        req = SetParameters.Request()
        req.parameters = [p]
        fut = cli.call_async(req)

        def done(f):
            try:
                r = f.result().results[0]
            except Exception as e:                            # noqa: BLE001
                self.note("%s %s FAILED: %r -- value UNCHANGED"
                          % (node, name, e), bad=True)
                return
            if r.successful:
                self.note("%s %s -> %s" % (node, name, value))
            else:
                self.note("%s %s REFUSED (%s) -- value UNCHANGED"
                          % (node, name, r.reason), bad=True)
        fut.add_done_callback(done)


def _res(f):
    try:
        r = f.result()
        return "%s %s" % ("ok" if r.success else "FAILED", r.message)
    except Exception as e:                                    # noqa: BLE001
        return "error %r" % (e,)


# ===========================================================================
#  WIDGETS
# ===========================================================================
class Ind(QLabel):
    """One indicator: caption, value, and a colour that means something."""

    def __init__(self, caption):
        super().__init__()
        self.caption = caption
        self.setFont(helvetica(11))
        self.setTextFormat(Qt.RichText)
        self.value = "--"
        self.set("--", C_UNKNOWN, "no data")

    def set(self, value, colour=C_TEXT, note=""):
        self.value, self.colour, self.note = value, colour, note
        self.setText(
            "<span style='color:%s;font-size:10px'>%s</span><br>"
            "<span style='color:%s;font-size:15px;font-weight:600'>%s</span>"
            "<br><span style='color:%s;font-size:10px'>%s</span>"
            % (C_MUTED, self.caption, colour, value, C_MUTED, note))


class Dial(QWidget):
    """Precision <-> speed, one slider, with the settings it implies shown."""

    def __init__(self, on_change):
        super().__init__()
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        row.addWidget(QLabel("PRECISION"))
        self.s = QSlider(Qt.Horizontal)
        self.s.setRange(0, 100)
        self.s.setValue(100)
        self.s.sliderReleased.connect(lambda: on_change(self.s.value() / 100.0))
        row.addWidget(self.s, 1)
        row.addWidget(QLabel("SPEED"))
        v.addLayout(row)
        self.lbl = QLabel()
        self.lbl.setFont(helvetica(9))
        self.lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.lbl)
        self.s.valueChanged.connect(self._show)
        self._show(100)

    def _show(self, x):
        st = ps.settings(x / 100.0)
        self.lbl.setText(
            "scale %.2f   ema %.2f   vmax %.2f rad/s   step %.2f rad"
            % (st["scale"], st["ema_alpha"], st["max_vel_rad_s"],
               st["max_step_rad"]))


# ===========================================================================
#  MAIN WINDOW
# ===========================================================================
class Gui(QMainWindow):

    def __init__(self, bus, args):
        super().__init__()
        self.bus = bus
        self.args = args
        self.rviz = []                  # [(name, Popen)]
        self.embedded = {}              # key -> (QWindow, container)
        self.jobs = []                  # [(label, Popen)]
        self.setWindowTitle("SRL operations -- commanded | actual")
        self.setFont(helvetica(11))
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(C_BG))
        self.setPalette(pal)

        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(8, 6, 8, 6)

        self.banner = QLabel()
        self.banner.setFont(sans(11, True))
        self.banner.setAlignment(Qt.AlignCenter)
        outer.addWidget(self.banner)

        self.split = QSplitter(Qt.Horizontal)
        outer.addWidget(self.split, 1)
        self.split.addWidget(self._left_column())
        self.split.addWidget(self._schematic_column())
        self.split.addWidget(self._right_column())
        self.split.setStretchFactor(0, 0)
        self.split.setStretchFactor(1, 1)
        self.split.setStretchFactor(2, 1)
        # The left column must NOT be collapsible. Once a foreign RViz window
        # is reparented, its own natural size becomes the container's size
        # hint -- two of them ask for ~2400 px and the splitter obliges by
        # squeezing this column to ZERO. Measured: the first screenshot showed
        # RViz covering the banner, controls, cameras, indicators AND the
        # divergence readout, i.e. every panel the GUI exists to show.
        self.split.setCollapsible(0, False)
        self.split.setSizes([336, 1090, 494])

        outer.addLayout(self._bottom_bar())
        self.setCentralWidget(root)
        self.resize(1920, 1060)

        self._ft = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        QTimer.singleShot(400, self.start_rviz)
        # ALWAYS dump geometry, including with --no-rviz. The pixel proof
        # needs the panel rectangles and refuses to guess regions without
        # them; scheduling this only on the RViz paths meant the one case
        # that isolates the GUI's OWN panels could not be verified at all.
        QTimer.singleShot(3000, self._dump_geometry)
        # FRAME STATS GO TO A FILE, not to stdout.
        #
        # They were printed to stdout first, and the measurement harness got
        # back an EMPTY string every time -- so the only frame numbers Part 4
        # could report were read off the status bar in a screenshot. Stdout
        # through a pipe depends on the writer surviving long enough to be
        # drained, on nothing else holding the pipe open, and on the harness
        # not killing the process group first; a file depends on none of that.
        # The same reasoning already applies to the geometry dump.
        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self._dump_stats)
        self.stat_timer.start(2000)

    # ---------------------------------------------------------- left column
    def _left_column(self):
        host = QScrollArea()
        host.setWidgetResizable(True)
        # NO HORIZONTAL SCROLL. With one, the inner widget keeps its natural
        # width and every button is clipped at both edges -- which is how the
        # column looked before: labels sliced down the middle.
        host.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        host.setMinimumWidth(330)
        inner = QWidget()
        inner.setMaximumWidth(330)
        col = QVBoxLayout(inner)

        # THE CAMERAS GO FIRST. For a remote operator this is the only view of
        # the workspace at all: every other panel describes the ROBOT, and the
        # arms are on somebody else's back. Placed last it fell below the fold
        # on a 950 px display, which for the single most important panel is a
        # real defect rather than a cosmetic one.
        campanel = QGroupBox("Wrist cameras (subscribed, never opened)")
        campanel.setFont(helvetica(11, True))
        cl = QHBoxLayout(campanel)
        self.cam_lbl, self.cam_cap = {}, {}
        for a in ARMS:
            c = QVBoxLayout()
            nm = QLabel(a.upper())
            nm.setFont(helvetica(10, True))
            nm.setAlignment(Qt.AlignCenter)
            img = QLabel()
            img.setMinimumSize(140, 106)
            img.setAlignment(Qt.AlignCenter)
            img.setStyleSheet("background:#111;color:#bbb;border:1px solid #999")
            cap = QLabel()
            cap.setFont(helvetica(9))
            cap.setAlignment(Qt.AlignCenter)
            c.addWidget(nm)
            c.addWidget(img)
            c.addWidget(cap)
            cl.addLayout(c)
            self.cam_lbl[a], self.cam_cap[a] = img, cap
        col.addWidget(campanel)

        col.addWidget(self._controls())
        col.addWidget(self._launchers())

        self.ind = {}
        for title, keys in (
            ("Master", ("capability_left", "capability_right", "channels",
                        "clutch")),
            ("Scene", ("fingerprint", "detector_left", "detector_right")),
            ("Mode", ("mode", "autonomy", "intent")),
            ("Target", ("gripper", "clearance_left", "clearance_right")),
            ("Safety", ("estop", "blockers", "ik_left", "ik_right")),
        ):
            g = QGroupBox(title)
            g.setFont(helvetica(11, True))
            gl = QGridLayout(g)
            for i, k in enumerate(keys):
                w = Ind(k.replace("_", " "))
                self.ind[k] = w
                gl.addWidget(w, i // 2, i % 2)
            col.addWidget(g)

        col.addStretch(1)
        host.setWidget(inner)
        return host

    def _controls(self):
        g = QGroupBox("Controls")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        self.dial = Dial(self.on_dial)
        v.addWidget(self.dial)

        row = QHBoxLayout()
        self.force_clutch = QCheckBox("force clutch ENGAGED")
        self.force_clutch.stateChanged.connect(self.on_force_clutch)
        row.addWidget(self.force_clutch)
        b = QPushButton("re-base anchor")
        b.setToolTip("/master_rebase -- the next VALID frame becomes the "
                     "reference. The bridge calls this when it enables.")
        b.clicked.connect(lambda: self.bus.submit(
            lambda: self.bus.call_trigger("/master_rebase")))
        row.addWidget(b)
        v.addLayout(row)

        self.scale = {}
        for a in ARMS:
            r = QHBoxLayout()
            r.addWidget(QLabel("%s scale" % a))
            s = QSlider(Qt.Horizontal)
            s.setRange(20, 200)
            s.setValue(100)
            lab = QLabel("1.00")
            lab.setFont(helvetica(10))
            s.valueChanged.connect(lambda x, l=lab: l.setText("%.2f" % (x / 100.0)))
            s.sliderReleased.connect(
                lambda a=a, s=s: self.on_scale(a, s.value() / 100.0))
            r.addWidget(s, 1)
            r.addWidget(lab)
            v.addLayout(r)
            self.scale[a] = s

        r = QHBoxLayout()
        for lbl, fn in (("release LEFT grip", lambda: self.on_release("left")),
                        ("release RIGHT grip", lambda: self.on_release("right"))):
            b = QPushButton(lbl)
            b.clicked.connect(fn)
            r.addWidget(b)
        v.addLayout(r)
        return g

    def _launchers(self):
        g = QGroupBox("Launch")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        self.specs = gls.all_specs()
        gls.validate(self.specs)
        self.buttons = {}
        # EVERY GROUP THE MANIFEST DECLARES, NOT THREE OF THE FOUR.
        #
        # `demo` was missing, so the fifteen dance specs had NO BUTTON AT ALL
        # and the three routines could not be launched from this GUI -- which
        # is G-1 and G-2, "launch every mode" and "run every task", quietly
        # unmet. They were filmable only because `record_abc_sweep` calls
        # `Gui.on_launch(spec)` directly rather than pressing anything.
        #
        # Found by `verify_gui_buttons`, whose "every enabled launch button
        # pressed" check read 50 of 65 and named the fifteen. A manifest entry
        # with no button is the same defect as a button with no manifest entry
        # and is harder to see.
        for group, title in (("mode", "Modes"), ("task", "Tasks"),
                             ("demo", "Demonstrations (no trial data)"),
                             ("diag", "Diagnostics")):
            lab = QLabel(title)
            lab.setFont(helvetica(10, True))
            v.addWidget(lab)
            grid = QGridLayout()
            items = [s for s in self.specs if s.group == group]
            for i, s in enumerate(items):
                b = QPushButton(s.label)
                b.setFont(helvetica(9))
                b.setMinimumWidth(0)
                b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
                if s.enabled:
                    b.setToolTip(s.note)
                    b.clicked.connect(lambda _, sp=s: self.on_launch(sp))
                else:
                    # DISABLED WITH THE REASON ON IT. A button that exits 2 on
                    # press looks exactly like one that launched something
                    # invisible; a greyed button with a sentence does not.
                    b.setEnabled(False)
                    b.setToolTip(s.disabled_reason)
                    b.setText(s.label + "  (unavailable)")
                grid.addWidget(b, i, 0)
                self.buttons[s.key] = b
            v.addLayout(grid)
        b = QPushButton("stop all launched jobs")
        b.clicked.connect(self.on_stop_jobs)
        v.addWidget(b)
        return g

    # --------------------------------------------------------- right column
    def _schematic_column(self):
        """The two schematics. THE PANEL THAT MATTERS MOST goes at the top.

        Fourteen channel numbers in a table do not answer "which channels are
        driving the robot and which are frozen"; the chain drawn as roll /
        bend / roll answers it pre-attentively, and the FROZEN state is drawn
        distinctly because a channel that is healthy and unused is the exact
        failure that hid a whole pot repair behind a stale baseline file.
        """
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(6, 0, 6, 0)
        v.setSpacing(8)
        self.master_schema = MasterArmSchematic()
        self.robot_schema = RobotSchematic()
        # CAPPED, NOT STRETCHED. Letting the cards expand left a bordered box
        # two thirds empty, which reads as something failing to load rather
        # than as space. A card should be the size of its content.
        self.master_schema.setMaximumHeight(404)
        self.robot_schema.setMaximumHeight(250)
        v.addWidget(self.master_schema, 0)
        v.addWidget(self.robot_schema, 0)
        # The action log moves here, into the space the caps free, where it
        # sits directly under the panels whose controls generate it.
        # PORTED FROM THE CONSOLE. srl_gui became primary while `console`
        # still held four things it never had: rolling Charts, the Pilot
        # runner, the Experiments session controls, and the full Event log.
        # Naming a primary without moving those would have left the same split
        # in place under a different name.
        tabs = QTabWidget()
        tabs.setFont(helvetica(9))
        # 268 WHILE THE TABS WERE ALL READOUTS. The Instruct tab is a
        # DIALOGUE -- a prompt, what was understood, what the camera saw, an
        # announced intention, a confirm step and a running transcript -- and
        # at 268 px the transcript was two lines high, which for the panel
        # that says why the robot refused is not a readout at all.
        tabs.setMaximumHeight(430)

        ch = QWidget()
        cv = QVBoxLayout(ch)
        cv.setContentsMargins(4, 4, 4, 4)
        cv.setSpacing(4)
        self.strips = {
            "ee_z": Strip("end-effector height", "m"),
            "clearance": Strip("clearance to wearer", "m", floor=0.12),
            "lag": Strip("sim -> real divergence", "rad", floor=0.5),
        }
        for st in self.strips.values():
            cv.addWidget(st)
        tabs.addTab(ch, "Charts")

        se = QWidget()
        sv = QVBoxLayout(se)
        self.trial_lbl = QLabel("no /trial_state")
        self.trial_lbl.setFont(mono(9))
        self.trial_lbl.setWordWrap(True)
        sv.addWidget(QLabel("TRIAL STATE"))
        sv.addWidget(self.trial_lbl)
        row = QHBoxLayout()
        for lab, argv in (
                ("Pilot (offline)", [sys.executable,
                                     os.path.join(_WS, "scripts",
                                                  "pilot_bimanual.py")]),
                ("Session order", [os.path.join(_WS, "scripts",
                                                "run_experiment.sh"), "a",
                                   "--taskset", "clip", "--scripted"])):
            b = QPushButton(lab)
            b.setFont(helvetica(9))
            b.clicked.connect(lambda _, a=argv, l=lab: self._run_raw(l, a))
            row.addWidget(b)
        sv.addLayout(row)
        sv.addStretch(1)
        tabs.addTab(se, "Session")

        # ---------------------------------------------------- READY TO RUN
        # SCROLLABLE. The right column is short and the readiness panel needs
        # ~300 px; without a scroll area the layout compresses it and the
        # controls below get squeezed to nothing. Scrolling is the honest
        # answer to "more content than height".
        rd_inner = QWidget()
        rv = QVBoxLayout(rd_inner)
        self.ready_panel = ReadyPanel()
        self.ready_panel.setMinimumHeight(300)
        rv.addWidget(self.ready_panel)
        rd = QScrollArea()
        rd.setWidget(rd_inner)
        rd.setWidgetResizable(True)
        rd.setStyleSheet("border:none")

        self.sess_lbl = QLabel("no session")
        self.sess_lbl.setFont(mono(9))
        self.sess_lbl.setWordWrap(True)
        self.sess_lbl.setStyleSheet("color:%s" % C_MUTED)
        rv.addWidget(self.sess_lbl)

        # START / RESUME
        top = QHBoxLayout()
        self.pid = QLineEdit("P01")
        self.pid.setFont(mono(9))
        self.pid.setMaximumWidth(90)
        top.addWidget(QLabel("participant"))
        top.addWidget(self.pid)
        b = QPushButton("START SESSION")
        b.setFont(helvetica(9, True))
        b.clicked.connect(self._session_start)
        top.addWidget(b)
        b = QPushButton("RESUME SESSION")
        b.setFont(helvetica(9))
        b.clicked.connect(self._session_resume)
        top.addWidget(b)
        top.addStretch(1)
        rv.addLayout(top)

        # THE THREE RECOVERIES. One click, no diagnosis.
        row = QHBoxLayout()
        for lab, svc, col in (("REDO THIS TRIAL", "redo", C_WARN),
                              ("SKIP AND CONTINUE", "skip", C_WARN),
                              ("ABORT SESSION", "abort", C_BAD)):
            bb = QPushButton(lab)
            bb.setFont(helvetica(9, True))
            bb.setStyleSheet("color:%s" % col)
            bb.clicked.connect(lambda _, x=svc: self._session_srv(x))
            row.addWidget(bb)
        rv.addLayout(row)

        self.sess_log = QTextEdit()
        self.sess_log.setReadOnly(True)
        self.sess_log.setFont(mono(8))
        self.sess_log.setStyleSheet("color:%s;border:none" % C_MUTED)
        self.sess_log.setMaximumHeight(150)
        rv.addWidget(QLabel("SESSION LOG — readable while it is happening"))
        rv.addWidget(self.sess_log)
        tabs.addTab(rd, "Ready / Session")
        # AFTER the window is up, not during construction: something later in
        # __init__ resets the current index, so selecting here silently did
        # nothing and the capture kept showing Charts.
        self._tabs = tabs
        want = getattr(self.args, "tab", None)
        if want:
            def _pick():
                for i in range(self._tabs.count()):
                    if want.lower() in self._tabs.tabText(i).lower():
                        self._tabs.setCurrentIndex(i)
                        return
            QTimer.singleShot(900, _pick)

        self.eventlog = QTextEdit()
        self.eventlog.setReadOnly(True)
        self.eventlog.setFont(mono(8))
        self.eventlog.setStyleSheet("color:%s;border:none" % C_MUTED)
        tabs.addTab(self._instruct_tab(), "Instruct")
        tabs.addTab(self.eventlog, "Event log")

        v.addWidget(tabs, 0)
        v.addStretch(1)
        return w

    # ====================================================================
    # THE PROMPT PANEL -- MODE 06 FROM A TYPED SENTENCE, IN THE GUI
    # ====================================================================
    #
    # WHY IT IS A TAB IN THIS GUI AND NOT A SECOND WINDOW. `console` and
    # `launcher` were superseded because a session needs one place to look;
    # a prompt box in its own tool would be the third.
    #
    # WHAT IT HAS TO DO, AND THE ONE FAILURE IT IS WRITTEN AGAINST: "a prompt
    # box that looks right and sends nothing". So every element here is wired
    # to the SAME code the recorded run uses -- `t1_instruction.plan_from` for
    # the grounding, `stage_observe_and_detect.py` for the look,
    # `run_experiment.sh ... --vision --instruct` for the motion -- rather
    # than to a display-only copy. If the panel plans it, the arm can run it,
    # because they are one implementation.
    #
    # THE SEQUENCE, and it is the sequence a mode 06 run actually performs:
    #
    #   LOOK        observe pose -> detect -> classify -> home   (staged, so
    #               the follower is never fighting the look)
    #   TYPE        the instruction, grounded against what was SEEN
    #   UNDERSTOOD  verb, target, destination, how many -- shown BEFORE
    #               anything moves, which is the point of showing it
    #   ASK         a question in plain words, answerable in the same box
    #   CONFIRM     the announced intention, and a click. Mode 06 does not
    #               move until a person agrees with what it said it would do
    #   RUN         the real runner, its output tailed live
    def _instruct_tab(self):
        host = QScrollArea()
        host.setWidgetResizable(True)
        host.setStyleSheet("border:none")
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(6, 4, 6, 4)
        v.setSpacing(5)

        self._inst_pending = None      # the ASK this box is answering, if any
        self._inst_outcome = None      # the plan awaiting confirmation
        self._inst_proc = None         # the running subprocess
        self._inst_seen = []           # detections, as the camera reported
        self._inst_tail = None

        # ---- what to run it on
        row = QHBoxLayout()
        row.addWidget(QLabel("task"))
        self.inst_task = QComboBox()
        self.inst_task.addItems(["m1  (stage 1)", "m1s2  (stage 2)"])
        self.inst_task.setFont(helvetica(9))
        row.addWidget(self.inst_task)
        row.addWidget(QLabel("seed"))
        self.inst_seed = QLineEdit("0")
        self.inst_seed.setFont(mono(9))
        self.inst_seed.setMaximumWidth(46)
        row.addWidget(self.inst_seed)
        self.inst_look = QPushButton("LOOK (observe + detect)")
        self.inst_look.setFont(helvetica(9, True))
        self.inst_look.clicked.connect(self.on_instruct_look)
        row.addWidget(self.inst_look)
        row.addStretch(1)
        v.addLayout(row)

        # ---- the box itself
        row = QHBoxLayout()
        self.inst_edit = QLineEdit()
        self.inst_edit.setFont(mono(10))
        self.inst_edit.setPlaceholderText(
            "put the blue ones on the blue pad")
        self.inst_edit.returnPressed.connect(self.on_instruct_send)
        row.addWidget(self.inst_edit, 1)
        b = QPushButton("SEND")
        b.setFont(helvetica(9, True))
        b.clicked.connect(self.on_instruct_send)
        row.addWidget(b)
        # THE VOICE BUTTON USES THE EXISTING PATH, and says what that path is.
        # `voice_listener` publishes to /voice_transcript; this subscribes and
        # drops whatever arrives into the same box, so speech and typing meet
        # at the same grounding step rather than at two.
        self.inst_voice = QPushButton("VOICE")
        self.inst_voice.setFont(helvetica(9, True))
        self.inst_voice.clicked.connect(self.on_instruct_voice)
        row.addWidget(self.inst_voice)
        v.addLayout(row)

        # ---- what the parser understood, BEFORE anything moves
        g = QGroupBox("What I understood")
        g.setFont(helvetica(9, True))
        gl = QGridLayout(g)
        gl.setContentsMargins(6, 4, 6, 4)
        self.inst_parse = {}
        for i, k in enumerate(("verb", "target", "destination", "how many",
                               "which one", "read as")):
            lab = QLabel(k)
            lab.setFont(helvetica(8))
            lab.setStyleSheet("color:%s" % C_MUTED)
            val = QLabel("--")
            val.setFont(mono(9))
            val.setWordWrap(True)
            gl.addWidget(lab, i // 2, (i % 2) * 2)
            gl.addWidget(val, i // 2, (i % 2) * 2 + 1)
            self.inst_parse[k] = val
        gl.setColumnStretch(1, 1)
        gl.setColumnStretch(3, 1)
        v.addWidget(g)

        # ---- what the CAMERA saw
        g = QGroupBox("What the camera saw")
        g.setFont(helvetica(9, True))
        gv = QVBoxLayout(g)
        gv.setContentsMargins(6, 4, 6, 4)
        self.inst_seen_lbl = QLabel("no look taken yet -- press LOOK")
        self.inst_seen_lbl.setFont(mono(9))
        self.inst_seen_lbl.setWordWrap(True)
        gv.addWidget(self.inst_seen_lbl)
        v.addWidget(g)

        # ---- the announced intention and the confirm step
        g = QGroupBox("What I am going to do")
        g.setFont(helvetica(9, True))
        gv = QVBoxLayout(g)
        gv.setContentsMargins(6, 4, 6, 4)
        self.inst_intent = QLabel("--")
        self.inst_intent.setFont(sans(11, True))
        self.inst_intent.setWordWrap(True)
        gv.addWidget(self.inst_intent)
        row = QHBoxLayout()
        self.inst_go = QPushButton("CONFIRM AND RUN")
        self.inst_go.setFont(helvetica(9, True))
        self.inst_go.setStyleSheet("color:%s" % C_OK)
        self.inst_go.setEnabled(False)
        # A DISABLED BUTTON MUST CARRY ITS REASON, which is the difference
        # between "this cannot run yet" and "this did nothing".
        self.inst_go.setToolTip(
            "nothing is planned yet. Press LOOK so the camera can see the "
            "table, then type an instruction and press SEND; this arms when "
            "the instruction resolves to specific cubes and specific pads.")
        self.inst_go.clicked.connect(self.on_instruct_run)
        row.addWidget(self.inst_go)
        self.inst_cancel = QPushButton("CANCEL")
        self.inst_cancel.setFont(helvetica(9))
        self.inst_cancel.clicked.connect(self.on_instruct_cancel)
        row.addWidget(self.inst_cancel)
        self.inst_state = QLabel("IDLE")
        self.inst_state.setFont(sans(11, True))
        row.addWidget(self.inst_state, 1)
        gv.addLayout(row)
        v.addWidget(g)

        # ---- the dialogue and the run, in plain words
        self.inst_log = QTextEdit()
        self.inst_log.setReadOnly(True)
        self.inst_log.setFont(mono(8))
        self.inst_log.setStyleSheet("color:%s;border:none" % C_TEXT)
        self.inst_log.setMinimumHeight(120)
        v.addWidget(self.inst_log, 1)

        host.setWidget(w)
        self._inst_say("Type an instruction. Press LOOK first -- the "
                       "destination is chosen by the colour the CAMERA sees, "
                       "so there is nothing to plan against until it has "
                       "looked.")
        return host

    # ------------------------------------------------------------ helpers
    def log(self, text, bad=False):
        """SAY IT WHERE THE OPERATOR IS LOOKING. `Gui` had no such method.

        Eight calls to `self.log(...)` in this file -- the whole session panel:
        START SESSION, RESUME SESSION, REDO, SKIP, ABORT and the session
        publisher -- raised `AttributeError: 'Gui' object has no attribute
        'log'` on every press. PyQt SWALLOWS an exception raised inside a slot:
        it prints a traceback to stderr, which nobody is reading, and returns.
        So five buttons on the ethics-critical panel of this GUI did nothing at
        all, said nothing at all, and looked exactly like buttons that had
        worked.

        `log` was `Bus.log`, which is a LIST, and `Bus.note` is what appends to
        it. The name collided across two objects and the GUI lost.

        FOUND BY `verify_gui_buttons`, and only after the audit was taught to
        answer the modal consent dialog: the audit had been hanging on START
        SESSION since that dialog was added, so it never reached the press that
        would have shown this.
        """
        try:
            self.bus.note(text, bad=bad)
        except Exception:                                     # noqa: BLE001
            pass

    def _inst_say(self, text, bad=False):
        col = C_BAD if bad else C_TEXT
        self.inst_log.append('<span style="color:%s">%s</span>'
                             % (col, text.replace("<", "&lt;")))
        sb = self.inst_log.verticalScrollBar()
        sb.setValue(sb.maximum())
        # AND INTO THE SHARED EVENT LOG. One session, one log: an operator
        # reading the run afterwards should not have to know which panel a
        # line came from. It is also what makes every button in this panel
        # OBSERVABLE to `verify_gui_buttons`, whose rule is that a click that
        # produces silence is a failure.
        self.log(text, bad=bad)

    def _inst_set_state(self, text, colour=None):
        self.inst_state.setText(text)
        self.inst_state.setStyleSheet("color:%s" % (colour or C_TEXT))

    def _inst_task_key(self):
        return "m1s2" if self.inst_task.currentIndex() else "m1"

    def _inst_seed(self):
        try:
            return int(self.inst_seed.text().strip() or "0")
        except ValueError:
            return 0

    def _inst_import(self):
        """The SAME grounding module the run uses. Imported here rather than
        at module scope so a GUI on a machine without the experiments package
        still starts and says why this one panel is unavailable."""
        for p in (os.path.join(_WS, "src/srl_experiments/experiments/abc"),
                  os.path.join(_WS, "src/srl_autonomy"),
                  os.path.join(_WS, "config")):
            if p not in sys.path:
                sys.path.insert(0, p)
        import t1_instruction as TI
        return TI

    DETECTIONS = "/tmp/srl_t1_detections.json"

    def _inst_load_detections(self, quiet=False):
        """What the last look SAW, with the classifier's own margins."""
        try:
            d = json.load(open(self.DETECTIONS))
        except Exception as e:                                # noqa: BLE001
            if not quiet:
                self._inst_say("no detections to plan against (%s). Press "
                               "LOOK." % e, bad=True)
            return None
        want_task = self._inst_task_key().replace("m1s2", "t1s2") \
            .replace("m1", "t1")
        if d.get("task", "t1") != want_task:
            self._inst_say("the detections on disk are of %s and this is %s "
                           "-- look again."
                           % (d.get("task", "t1"), want_task), bad=True)
            return None
        if want_task == "t1s2" and int(d.get("seed", -1)) != self._inst_seed():
            self._inst_say("the detections are seed %s and this is seed %d; "
                           "stage 2 draws its cubes from the seed, so those "
                           "are two different tables."
                           % (d.get("seed"), self._inst_seed()), bad=True)
            return None
        self._inst_seen = [tuple(c) for c in d.get("cubes", [])]
        seen = (d.get("timing") or {}).get("seen") or []
        colours = ("blue", "green")
        lines = []
        for i, c in enumerate(self._inst_seen):
            ev = seen[i] if i < len(seen) else {}
            conf = ev.get("confidence_counts")
            lines.append(
                "cube %d   %-5s   x %+.3f  y %+.3f   %s"
                % (i + 1, colours[int(c[2])], c[0], c[1],
                   "confidence --" if conf is None else
                   "confidence %d counts inside the %s band"
                   % (conf, ev.get("colour", "?"))))
        self.inst_seen_lbl.setText("\n".join(lines) or "nothing seen")
        return self._inst_seen

    def _inst_show_parse(self, o):
        it = getattr(o, "intent", None)
        d = {} if it is None else it.as_dict()
        dest = d.get("destination")
        if isinstance(dest, dict):
            dest = (dest.get("colour") or dest.get("kind") or "--")
        sel = d.get("selector")
        if isinstance(sel, dict):
            sel = sel.get("which") or sel.get("word") or (
                "number %s" % sel["n"] if "n" in sel else sel.get("kind"))
        fixes = d.get("corrections") or []
        self.inst_parse["verb"].setText(str(d.get("verb") or "--"))
        self.inst_parse["target"].setText(str(d.get("target") or "--"))
        self.inst_parse["destination"].setText(str(dest or "--"))
        self.inst_parse["how many"].setText(
            "all of them" if d.get("quantity") == "all" else "one")
        self.inst_parse["which one"].setText(str(sel or "--"))
        self.inst_parse["read as"].setText(
            ", ".join("%s -> %s" % (a, b) for a, b in fixes) or "--")

    # ------------------------------------------------------------ actions
    def on_instruct_look(self):
        """Run the staged look. It is a SUBPROCESS, for the reason the script
        itself records: the look publishes to the arm controller, so it must
        not happen inside anything that is also driving the arm."""
        if self._inst_proc is not None and self._inst_proc.poll() is None:
            self._inst_say("something is already running.", bad=True)
            return
        task = self._inst_task_key().replace("m1s2", "t1s2").replace("m1", "t1")
        argv = [sys.executable,
                os.path.join(_WS, "scripts", "stage_observe_and_detect.py"),
                "--task", task, "--seed", str(self._inst_seed()),
                "--out", self.DETECTIONS]
        self._inst_set_state("LOOKING", C_WARN)
        self._inst_say("looking: %s" % " ".join(argv[1:]))
        self._inst_start(argv, self._inst_look_done)

    def _inst_look_done(self, rc):
        if rc != 0:
            self._inst_set_state("LOOK FAILED", C_BAD)
            self._inst_say("the look failed (rc=%d). Nothing will be planned "
                           "from the task file -- that is the point of "
                           "refusing." % rc, bad=True)
            return
        cubes = self._inst_load_detections()
        self._inst_set_state("SEEN %d" % len(cubes or []), C_OK)
        self._inst_say("the camera saw %d cubes." % len(cubes or []))

    def on_instruct_send(self):
        text = self.inst_edit.text().strip()
        if not text:
            # SILENCE IS A FAILURE. An empty box and a box whose instruction
            # was refused look identical from the operator's side unless the
            # empty one says so.
            self._inst_say("nothing typed. Say what to do -- for example "
                           "'put the blue ones on the blue pad'.")
            return
        self.inst_edit.clear()
        self._inst_say("> %s" % text)
        cubes = self._inst_load_detections(quiet=True)
        if not cubes:
            self._inst_say("I have not looked yet, so I do not know what is "
                           "on the table. Press LOOK.", bad=True)
            self._inst_set_state("NO DETECTIONS", C_BAD)
            return
        try:
            TI = self._inst_import()
        except Exception as e:                                # noqa: BLE001
            self._inst_say("the grounding module will not import: %s" % e,
                           bad=True)
            return
        o = TI.plan_from(text, cubes, pending=self._inst_pending)
        self._inst_show_parse(o)
        if o.kind == TI.ASK:
            # A QUESTION, IN PLAIN WORDS, ANSWERABLE IN THE SAME BOX.
            self._inst_pending = o
            self._inst_outcome = None
            self.inst_go.setEnabled(False)
            self.inst_intent.setText("nothing yet -- I asked a question")
            self._inst_set_state("ASKING", C_WARN)
            self._inst_say(o.message)
            return
        self._inst_pending = None
        if o.kind == TI.REFUSE:
            self._inst_outcome = None
            self.inst_go.setEnabled(False)
            self.inst_intent.setText("nothing -- I refused")
            self._inst_set_state("REFUSED", C_BAD)
            self._inst_say(o.message, bad=True)
            return
        self._inst_outcome = o
        self._inst_text = text
        pads = ("blue", "green")
        detail = "; ".join(
            "(%.3f, %.3f) -> %s" % (p[0], p[1],
                                    "hold it up" if p[2] is None
                                    else "%s pad" % pads[int(p[2])])
            for p in o.picks)
        self.inst_intent.setText("%s\n%s" % (o.message, detail))
        self.inst_go.setEnabled(True)
        self._inst_set_state("AWAITING CONFIRMATION", C_WARN)
        self._inst_say("%s -- %s. Press CONFIRM AND RUN." % (o.message, detail))

    def on_instruct_cancel(self):
        self._inst_pending = None
        self._inst_outcome = None
        self.inst_go.setEnabled(False)
        self.inst_intent.setText("--")
        if self._inst_proc is not None and self._inst_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._inst_proc.pid), 15)
            except Exception:                                # noqa: BLE001
                pass
            self._inst_say("stopped the running job.")
        self._inst_set_state("IDLE")
        self._inst_say("cancelled -- nothing will move.")

    def on_instruct_run(self):
        """THE REAL RUNNER, with the REAL instruction. Not a preview."""
        o = self._inst_outcome
        if o is None:
            return
        if self._inst_proc is not None and self._inst_proc.poll() is None:
            self._inst_say("something is already running.", bad=True)
            return
        key = self._inst_task_key()
        argv = [os.path.join(_WS, "scripts", "run_experiment.sh"), key,
                "--mode", "06_full_autonomy", "--taskset", "msc",
                "--participant", "PILOT", "--scripted",
                "--vision", self.DETECTIONS,
                "--instruct", self._inst_text]
        if key == "m1s2":
            argv += ["--seed", str(self._inst_seed())]
        self._inst_set_state("RUNNING", C_WARN)
        self.inst_go.setEnabled(False)
        self._inst_say("running: %s" % " ".join(argv[1:]))
        self._inst_start(argv, self._inst_run_done)

    def _inst_run_done(self, rc):
        if rc == 0:
            self._inst_set_state("DONE", C_OK)
            self._inst_say("the run finished.")
        else:
            self._inst_set_state("FAILED rc=%d" % rc, C_BAD)
            self._inst_say("the run exited %d." % rc, bad=True)

    def on_instruct_voice(self):
        """Speech, through the path that already exists.

        `voice_listener` is the only thing in this repository that turns
        speech into text, and it publishes to /voice_transcript. This starts
        it and drops whatever arrives into the same box the keyboard writes
        to, so both meet the grounding step at the same place.

        IT SAYS WHAT IT CANNOT DO. /dev/snd on this host contains only
        `timer` -- there is no capture device inside WSL -- so the microphone
        path needs `scripts/win_mic_sender.py` running on Windows. The button
        starts the listener either way and the transcript arrives when audio
        does; a button that pretended otherwise would be the thing this panel
        exists to avoid.
        """
        if getattr(self, "_voice_proc", None) is not None \
                and self._voice_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._voice_proc.pid), 15)
            except Exception:                                # noqa: BLE001
                pass
            self._voice_proc = None
            self.inst_voice.setText("VOICE")
            self._inst_say("stopped listening.")
            return
        argv = ["ros2", "run", "srl_autonomy", "voice_listener",
                "--ros-args", "-p", "source:=udp"]
        try:
            self._voice_proc = self._inst_spawn(argv)
        except Exception as e:                                # noqa: BLE001
            self._inst_say("cannot start the listener: %s" % e, bad=True)
            return
        if self._voice_proc is None:
            return
        self.inst_voice.setText("STOP VOICE")
        self._inst_say(
            "listening on /voice_transcript. There is no capture device "
            "inside WSL (/dev/snd holds only `timer`), so speech has to come "
            "from scripts/win_mic_sender.py on the Windows side; the wake "
            "word is %r." % "hey doc oc")

    # ONE PLACE THIS PANEL STARTS A PROCESS, AND IT IS THESE TWO METHODS.
    #
    # `verify_gui_buttons` presses every button in this window, and pressing
    # LOOK or VOICE for real would move the arm and open a microphone in the
    # middle of a verification run. Both go through `_inst_spawn` /
    # `_inst_start` so the verifier can replace exactly the Popen and leave
    # the whole click path -- validation, refusal, logging -- running for real.
    # It is the same treatment `on_launch` already gets, for the same reason.
    def _inst_spawn(self, argv):
        """Start a detached process. The ONE Popen the voice path uses."""
        return subprocess.Popen(argv, start_new_session=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    def _inst_start(self, argv, done):
        """Run something and tail its output into the panel, line by line."""
        try:
            p = subprocess.Popen(argv, start_new_session=True,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True,
                                 bufsize=1)
        except Exception as e:                                # noqa: BLE001
            self._inst_say("could not start it: %s" % e, bad=True)
            self._inst_set_state("FAILED", C_BAD)
            return
        self._inst_proc = p

        def pump():
            for line in iter(p.stdout.readline, ""):
                self._inst_lines.append(line.rstrip())
            p.wait()
            self._inst_lines.append(("__rc__", p.returncode))

        self._inst_lines = []
        self._inst_done_cb = done
        threading.Thread(target=pump, daemon=True).start()
        if self._inst_tail is None:
            self._inst_tail = QTimer(self)
            self._inst_tail.timeout.connect(self._inst_drain)
            self._inst_tail.start(200)

    def _inst_drain(self):
        """Move whatever the job printed into the panel, on the Qt thread.

        The reader thread must not touch widgets -- Qt is not thread safe and
        a crash here takes the whole operations GUI with it.
        """
        lines, self._inst_lines = self._inst_lines, []
        for ln in lines:
            if isinstance(ln, tuple):
                cb, self._inst_done_cb = self._inst_done_cb, None
                self._inst_proc = None
                if cb:
                    cb(ln[1])
                continue
            if not ln.strip():
                continue
            # THE LINES A PERSON NEEDS, LOUDLY; the rest quietly.
            if ln.startswith("[progress]"):
                self._inst_say(ln)
                self._inst_set_state(ln.replace("[progress] ", ""), C_OK)
            elif ln.startswith(("[instruct]", "[vision]", "REFUS", "[stage-")):
                self._inst_say(ln)
            elif "Traceback" in ln or "Error" in ln:
                self._inst_say(ln, bad=True)

    def _run_raw(self, label, argv):
        """Launch something that is not a manifest Spec (the ported console
        buttons). Same refusal-and-log discipline as on_launch."""
        try:
            p = subprocess.Popen(argv, start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        except Exception as e:                                # noqa: BLE001
            self.bus.note("LAUNCH FAILED %s: %r" % (label, e), bad=True)
            return
        self.jobs.append((label, p))
        self.bus.note("launched %s (pid %d)" % (label, p.pid))
        QTimer.singleShot(2500, lambda: self._check_job_raw(label, p))

    def _check_job_raw(self, label, p):
        rc = p.poll()
        if rc is not None and rc != 0:
            self.bus.note("%s EXITED %d within 2.5 s -- it did not run"
                          % (label, rc), bad=True)

    def _right_column(self):
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)

        self.viz_split = QSplitter(Qt.Horizontal)
        self.viz_host = {}
        self.viz_msg = {}
        # DEFAULT IS ONE PANEL WITH THE COMMANDED ARM GHOSTED OVER THE ACTUAL
        # ONE, and that is a measurement result rather than a preference. Two
        # embedded RViz windows do not clip to their containers on this
        # display stack, so they paint over each other and over every
        # indicator (screenshotted). The overlay is also the better answer to
        # the question the panels exist for: the operator sees the GAP in one
        # 3-D scene instead of comparing two viewports by eye, and it costs
        # ~294 MB less. --dual-rviz opts back in where a window manager runs.
        panels = ([("commanded", "COMMANDED  --  sim, driven by the active mode"),
                   ("actual", "ACTUAL  --  real arms (real_* frames)")]
                  if getattr(self.args, "dual_rviz", False) else
                  [("overlay",
                    "COMMANDED (solid) ghosted over ACTUAL (translucent)")])
        for key, title in panels:
            box = QGroupBox(title)
            box.setFont(helvetica(11, True))
            bl = QVBoxLayout(box)
            bl.setContentsMargins(2, 2, 2, 2)
            host = QWidget()
            host.setMinimumSize(600, 420)
            hl = QVBoxLayout(host)
            hl.setContentsMargins(0, 0, 0, 0)
            msg = QLabel("starting RViz...")
            msg.setAlignment(Qt.AlignCenter)
            msg.setFont(helvetica(12))
            hl.addWidget(msg)
            bl.addWidget(host)
            self.viz_host[key], self.viz_msg[key] = host, msg
            self.viz_split.addWidget(box)
        v.addWidget(self.viz_split, 1)
        v.addWidget(self._divergence_panel())
        return w

    def _divergence_panel(self):
        g = QGroupBox("Divergence: commanded vs actual")
        g.setFont(helvetica(11, True))
        grid = QGridLayout(g)
        self.div_head = QLabel()
        self.div_head.setFont(helvetica(10))
        grid.addWidget(self.div_head, 0, 0, 1, 10)
        self.div_max, self.div_ee, self.div_state, self.div_joint = {}, {}, {}, {}
        for r, a in enumerate(ARMS):
            grid.addWidget(QLabel(a.upper()), r + 1, 0)
            self.div_state[a] = QLabel()
            self.div_state[a].setFont(helvetica(11, True))
            grid.addWidget(self.div_state[a], r + 1, 1)
            self.div_max[a] = QLabel()
            self.div_max[a].setFont(helvetica(13, True))
            grid.addWidget(self.div_max[a], r + 1, 2)
            self.div_ee[a] = QLabel()
            self.div_ee[a].setFont(helvetica(13, True))
            grid.addWidget(self.div_ee[a], r + 1, 3)
            self.div_joint[a] = QLabel()
            self.div_joint[a].setFont(helvetica(9))
            self.div_joint[a].setStyleSheet("color:%s" % C_MUTED)
            grid.addWidget(self.div_joint[a], r + 1, 4, 1, 6)
        grid.setColumnStretch(4, 1)
        return g

    def _bottom_bar(self):
        bar = QHBoxLayout()
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setFont(helvetica(14, True))
        self.estop_btn.setStyleSheet(
            "background:%s;color:white;padding:10px;" % C_BAD)
        self.estop_btn.clicked.connect(self.on_estop)
        bar.addWidget(self.estop_btn)
        b = QPushButton("reset e-stop")
        b.clicked.connect(self.on_estop_reset)
        bar.addWidget(b)
        b = QPushButton("SELF-TEST indicators")
        b.setToolTip("Drive synthetic bad data through the real formatters "
                     "and require every indicator to change. Proves 'all "
                     "clear' is a reading and not a stuck widget.")
        b.clicked.connect(self.on_self_test)
        bar.addWidget(b)
        self.selftest_lbl = QLabel()
        self.selftest_lbl.setFont(helvetica(10, True))
        bar.addWidget(self.selftest_lbl)
        bar.addStretch(1)
        self.frame_lbl = QLabel()
        self.frame_lbl.setFont(helvetica(10))
        bar.addWidget(self.frame_lbl)
        return bar

    # ------------------------------------------------------- session (Part 6)
    def _session_pub(self, topic, data):
        """Fire and forget. The GUI NEVER blocks on the session manager: a
        wait_for_service against a node that has died is exactly how the
        e-stop once took 4 s to fire."""
        try:
            from std_msgs.msg import String as _S
            pub = self.bus.create_publisher(_S, topic, 4)
            pub.publish(_S(data=str(data)))
            self.log("-> %s %s" % (topic, data))
        except Exception as e:                                   # noqa: BLE001
            self.log("session publish failed: %s" % e, bad=True)

    def _session_start(self):
        """Consent FIRST. The session manager refuses to start without it, so
        this walks the checklist rather than pretending it is a formality."""
        from srl_experiments.session import CONSENT_STEPS
        from PyQt5.QtWidgets import QMessageBox
        for step in CONSENT_STEPS:
            r = QMessageBox.question(
                self, "Consent checkpoint",
                "%s\n\nConfirmed?" % step,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if r != QMessageBox.Yes:
                self.log("session NOT started: consent step declined (%s)"
                         % step, bad=True)
                return
        self._session_pub("/session/begin", self.pid.text().strip() or "P00")
        for step in CONSENT_STEPS:
            self._session_pub("/session/consent", step)
        self.log("session start requested for %s" % self.pid.text())

    def _session_resume(self):
        from srl_experiments.session import Session
        from PyQt5.QtWidgets import QMessageBox
        found = Session.resumable()
        if not found:
            # IN THE LOG AS WELL AS IN THE DIALOG. A dialog is dismissed and
            # gone; the event log is what an operator reads afterwards to find
            # out what was tried.
            self.log("resume: no interrupted session found")
            QMessageBox.information(self, "Resume",
                                    "No interrupted session found.")
            return
        s = found[0]
        r = QMessageBox.question(
            self, "Resume session",
            "Resume %s (%s)?\n%d of %d trials done."
            % (s.participant, s.sid, s.done_count, len(s.plan)),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if r == QMessageBox.Yes:
            self._session_pub("/session/begin", "resume:%s" % s.path)

    def _session_srv(self, name):
        """REDO / SKIP / ABORT. One click, no diagnosis required."""
        try:
            from std_srvs.srv import Trigger
            cli = self.bus.create_client(Trigger, "/session/%s" % name)
            if not cli.service_is_ready():
                self.log("/session/%s not available — is session_manager "
                         "running?" % name, bad=True)
                return
            cli.call_async(Trigger.Request())
            self.log("session: %s" % name.upper())
        except Exception as e:                                   # noqa: BLE001
            self.log("session %s failed: %s" % (name, e), bad=True)

    # ---------------------------------------------------------------- rviz
    def start_rviz(self):
        """Start RViz. NOT EMBEDDED BY DEFAULT, and that is a measurement.

        X11 reparenting via QWindow.fromWinId + createWindowContainer does
        embed -- the container is created, the window moves when told to --
        but on this display stack it is NOT CLIPPED to the container. Verified
        from screenshots: RViz rendered at its own 1595x995 over the top-left
        of the GUI, covering the banner, the wrist cameras, every indicator
        group and the divergence readout, while the Qt layout underneath was
        perfectly correct (the geometry dump put each panel exactly where it
        belonged). With two instances they also paint over each other.

        Clipping a reparented foreign window is the window manager's job and
        none is installed here, so this is reported rather than worked around.
        RViz therefore runs as its own top-level window -- which is how every
        other tool in this repository uses it -- carrying the SAME ghost
        config, so the commanded arm is still drawn over the actual one. The
        GUI's own window then reliably shows the things only it provides: the
        cameras, the controls, the launchers and the divergence readout.

        --embed-rviz opts into reparenting where a window manager makes it
        behave.
        """
        if self.args.no_rviz:
            for k in self.viz_msg:
                self.viz_msg[k].setText("RViz disabled (--no-rviz)")
            return
        if not getattr(self.args, "embed_rviz", False):
            key = list(self.viz_msg.keys())[0]
            cfg = self._rviz_config(key)
            try:
                p = subprocess.Popen(["rviz2", "-d", cfg],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                self.rviz.append((key, p))
            except FileNotFoundError:
                self.viz_msg[key].setText("rviz2 not on PATH")
                return
            for k, m in self.viz_msg.items():
                m.setText(
                    "RViz runs as a SEPARATE WINDOW (not embedded).\n\n"
                    "It carries the ghost config: the COMMANDED arm solid,\n"
                    "the ACTUAL arm translucent, in one scene.\n\n"
                    "Embedding is available with --embed-rviz, but on a host\n"
                    "with no window manager the reparented window is not\n"
                    "clipped and covers this GUI's own panels.")
                m.setStyleSheet("color:%s" % C_MUTED)
            QTimer.singleShot(2000, self._dump_geometry)
            return
        self._pending = list(self.viz_msg.keys())
        self._launch_next_rviz()

    def _launch_next_rviz(self):
        if not self._pending:
            return
        key = self._pending[0]
        cfg = self._rviz_config(key)
        # DIFF THE WINDOW LIST AROUND THE LAUNCH. Both instances present as
        # "RViz"; matching on the title would reparent whichever appeared
        # first into both panels, and the second panel would sit empty while
        # looking like an embedding failure.
        self._before = self._rviz_windows()
        # NO TF REMAP. Both robots are in the same /tf; the real one is told
        # apart by its `real_` frame prefix and by the fixed frame in the
        # generated config. Remapping to /real/tf -- which the brief names and
        # which does not exist -- would leave this panel permanently empty.
        argv = ["rviz2", "-d", cfg]
        try:
            p = subprocess.Popen(argv, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.viz_msg[key].setText("rviz2 not on PATH")
            self._pending.pop(0)
            QTimer.singleShot(200, self._launch_next_rviz)
            return
        self.rviz.append((key, p))
        self._tries = 0
        QTimer.singleShot(1500, self._try_embed)

    def _try_embed(self):
        if not self._pending:
            return
        key = self._pending[0]
        self._tries += 1
        new = self._rviz_windows() - self._before
        if not new:
            if self._tries > 30:
                # Fall back with a REASON rather than an empty panel. An empty
                # panel is indistinguishable from a working one showing
                # nothing, which is the failure this GUI exists to prevent.
                self.viz_msg[key].setText(
                    "RViz did not present an X window in 45 s.\n"
                    "Embedding needs an X11 (or XWayland) session.\n"
                    "Run rviz2 separately; every indicator here still works.")
                self._pending.pop(0)
                QTimer.singleShot(300, self._launch_next_rviz)
                return
            QTimer.singleShot(1500, self._try_embed)
            return
        wid = sorted(new)[0]
        foreign = QWindow.fromWinId(wid)
        container = QWidget.createWindowContainer(foreign, self.viz_host[key])
        # IGNORED SIZE POLICY IS LOAD-BEARING, not tidying. The container
        # inherits the foreign window's geometry as its size hint; Ignored
        # tells Qt to disregard that hint entirely and give the widget
        # whatever the layout allots. Without it the two RViz windows push
        # every other panel out of the window (verified from a screenshot,
        # not from a return code).
        container.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        container.setMinimumSize(160, 120)
        self.viz_msg[key].hide()
        self.viz_host[key].layout().addWidget(container)
        # KEEP THE FOREIGN WINDOW THE SIZE OF ITS CONTAINER, ACTIVELY.
        # Reparenting a foreign window does NOT make Qt manage its geometry:
        # measured, the embedded rviz2 kept its own 1595x995 and painted over
        # the banner, the cameras, the indicators and the divergence readout,
        # while the Qt layout underneath was perfectly correct (the geometry
        # dump put the panels exactly where they belonged). The container is
        # in the right place; the X window simply was not in the container.
        # Re-asserted every refresh rather than once, because RViz resizes
        # itself when its own docks change.
        self.embedded[key] = (foreign, container)
        self._pending.pop(0)
        # Re-assert the split AFTER embedding, because the container is what
        # disturbed it.
        self.split.setSizes([336, 1090, max(360, self.width() - 1426)])
        QTimer.singleShot(600, self._launch_next_rviz)
        QTimer.singleShot(1500, self._dump_geometry)

    def _rviz_config(self, key):
        """Write a per-panel config.

        The ACTUAL panel differs from the COMMANDED one in exactly two fields:
        the RobotModel's description topic, and the fixed frame (`real_world`,
        because every frame on that side carries the `real_` prefix).

        THE DESCRIPTION TOPIC IS DISCOVERED, NOT ASSUMED. real_arms.launch.py
        publishes /real/robot_description and mock_real.launch.py publishes
        /realmock/robot_description, so hardcoding either gives an empty panel
        against the other -- and an empty RViz panel is indistinguishable from
        a failed embedding, which is the confusion this GUI exists to remove.
        If neither is present the panel says so instead of rendering nothing.
        """
        try:
            src = open(RVIZ_TEMPLATE).read()
        except OSError:
            src = ""
        out = os.path.join(_scratch(), "srl_%s.rviz" % key)
        if key == "overlay":
            src = self._ghost_config(src)
        if key == "actual":
            topics = self.bus.snap.get("topics", set())
            desc = next((t for t in ("/real/robot_description",
                                     "/realmock/robot_description")
                         if t in topics), None)
            self.bus.real_desc_topic = desc
            if desc is None:
                self.bus.note("no real robot_description topic "
                              "(/real/... or /realmock/...) -- the ACTUAL "
                              "panel will show TF only", bad=True)
                desc = "/real/robot_description"
            src = src.replace("Value: /robot_description", "Value: %s" % desc)
            src = src.replace("Fixed Frame: world", "Fixed Frame: real_world")
        with open(out, "w") as fh:
            fh.write(src)
        return out

    def _dump_stats(self):
        """Frame-time statistics, on disk, for a harness to read.

        Writes the SAME numbers the status bar shows -- computed once in
        refresh() and shared -- so the file and the operator's view cannot
        disagree. `n` is included deliberately: a median over three samples is
        not a median, and a harness that cannot see the count would report it
        as though it were.
        """
        if not self._ft:
            return
        srt = sorted(self._ft)
        try:
            with open(os.path.join(_scratch(), "srl_gui_stats.json"), "w") as fh:
                json.dump(dict(n=len(srt),
                               last_ms=round(srt[-1], 3),
                               median_ms=round(srt[len(srt) // 2], 3),
                               p95_ms=round(srt[int(len(srt) * 0.95)], 3),
                               max_ms=round(srt[-1], 3),
                               budget_ms=100.0,
                               rss=_rss(),
                               status_bar=self.frame_lbl.text()), fh)
        except OSError:
            pass

    def _dump_geometry(self):
        """Write where each panel actually IS, in root-window pixels.

        The pixel proof needs regions to crop, and hardcoding them is a guess
        that goes stale the moment the layout changes -- the first version
        guessed, and mislabelled a perfectly good control column as empty
        because the box was in the wrong place. Asking the widgets where they
        are makes the check follow the layout instead of arguing with it.
        """
        try:
            out = {}
            for name, w in [("commanded", self.viz_host.get("commanded")),
                            ("actual", self.viz_host.get("actual")),
                            ("overlay", self.viz_host.get("overlay")),
                            ("controls", self.split.widget(0)),
                            ("divergence", self.div_head.parentWidget())]:
                if w is None:
                    continue
                tl = w.mapToGlobal(w.rect().topLeft())
                out[name] = [tl.x(), tl.y(), tl.x() + w.width(),
                             tl.y() + w.height()]
            with open(os.path.join(_scratch(), "srl_gui_geometry.json"),
                      "w") as fh:
                json.dump(out, fh)
        except Exception as e:                                # noqa: BLE001
            self.bus.note("geometry dump failed: %r" % (e,), bad=True)

    def _ghost_config(self, src):
        """Add a SECOND RobotModel for the real arm, translucent, same scene.

        This is what makes one panel answer the two-panel question. Both
        robots already live in one /tf tree -- the real one under `real_`
        frames -- so RViz's own `TF Prefix` property on RobotModel is exactly
        the right mechanism and no remapping is involved.

        If no real description is being published the ghost is NOT added, and
        the panel title says so. A translucent robot that is absent because
        nothing is publishing looks identical to one that is absent because
        the arms agree perfectly, which is the confusion this whole GUI
        exists to remove.
        """
        topics = self.bus.snap.get("topics", set())
        desc = next((t for t in ("/real/robot_description",
                                 "/realmock/robot_description")
                     if t in topics), None)
        self.bus.real_desc_topic = desc
        if desc is None:
            self.bus.note("no real robot_description -- NO GHOST is drawn. "
                          "The single robot on screen is the COMMANDED one; "
                          "it is not an agreement between two.", bad=True)
            return src
        ghost = (
            "    - Class: rviz_default_plugins/RobotModel\n"
            "      Name: RobotModelACTUAL\n"
            "      Enabled: true\n"
            "      Description Source: Topic\n"
            "      Description Topic:\n"
            "        Depth: 5\n"
            "        Durability Policy: Volatile\n"
            "        History Policy: Keep Last\n"
            "        Reliability Policy: Reliable\n"
            "        Value: %s\n"
            "      Visual Enabled: true\n"
            "      Collision Enabled: false\n"
            "      Alpha: 0.45\n"
            "      TF Prefix: real_\n"
            "      Update Interval: 0\n"
            "      Value: true\n" % desc)
        anchor = "    - Class: rviz_default_plugins/MarkerArray\n"
        if anchor in src:
            return src.replace(anchor, ghost + anchor, 1)
        return src

    @staticmethod
    def _rviz_windows():
        try:
            o = subprocess.run(["xwininfo", "-root", "-tree"],
                               capture_output=True, text=True, timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return set()
        ids = set()
        for line in o.splitlines():
            if "RViz" in line or "rviz" in line:
                m = re.search(r"(0x[0-9a-f]+)", line)
                if m:
                    ids.add(int(m.group(1), 16))
        return ids

    # ------------------------------------------------------------- actions
    def on_estop(self):
        self.bus.submit(lambda: (self.bus.publish_once(Bool, "/estop", True),
                                 self.bus.note("E-STOP published")))

    def on_estop_reset(self):
        # /estop_reset IS A SERVICE, NOT A TOPIC. Publishing a Bool at it once
        # left the e-stop latched and made six subsequent fault injections
        # report NOT HANDLED for that one reason.
        self.bus.submit(lambda: self.bus.call_trigger("/estop_reset"))

    def on_dial(self, d):
        st = ps.settings(d)
        ps.assert_safety_unconditional(st)     # raises if it moved anything safe
        def act():
            for a in ARMS:
                self.bus.set_param("/master_pose_node", "%s_scale" % a,
                                   st["scale"])
                self.bus.set_param("/ik_follower_%s" % a, "max_vel_rad_s",
                                   st["max_vel_rad_s"])
                self.bus.set_param("/ik_follower_%s" % a, "max_step_rad",
                                   st["max_step_rad"])
            self.bus.set_param("/master_pose_node", "ema_alpha",
                               st["ema_alpha"])
        self.bus.submit(act)
        for a in ARMS:
            self.scale[a].setValue(int(round(st["scale"] * 100)))

    def on_scale(self, arm, v):
        self.bus.submit(lambda: self.bus.set_param(
            "/master_pose_node", "%s_scale" % arm, v))

    def on_force_clutch(self, state):
        on = bool(state)
        self.bus.submit(lambda: self.bus.set_param(
            "/master_pose_node", "force_clutch_engaged", on))

    def on_release(self, arm):
        self.bus.submit(
            lambda: self.bus.call_trigger("/gripper_release_%s" % arm))

    def on_launch(self, spec):
        fails = self._preflight(spec)
        if fails:
            self.bus.note("REFUSED %s: %s" % (spec.label, "; ".join(fails)),
                          bad=True)
            return
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        try:
            p = subprocess.Popen(spec.argv, env=env, start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        except Exception as e:                                # noqa: BLE001
            self.bus.note("LAUNCH FAILED %s: %r" % (spec.label, e), bad=True)
            return
        self.jobs.append((spec.label, p))
        self.bus.note("launched %s (pid %d)" % (spec.label, p.pid))
        # A BUTTON THAT LAUNCHES A PROCESS WHICH IMMEDIATELY DIES MUST SAY SO.
        # Five buttons in an earlier build exited 2 on press while appearing
        # to launch, because nothing ever looked again.
        QTimer.singleShot(2500, lambda: self._check_job(spec, p))

    def _check_job(self, spec, p):
        rc = p.poll()
        if rc is not None and rc != 0:
            self.bus.note("%s EXITED %d within 2.5 s -- it did not run"
                          % (spec.label, rc), bad=True)

    def _preflight(self, spec):
        fails = []
        s = self.bus.snap
        topics = s.get("topics", set())
        stack_up = _stack_pids() > 0
        if spec.starts_stack and stack_up:
            fails.append("a stack is already running (%d procs); two "
                         "master_pose_node instances split the serial stream"
                         % _stack_pids())
        if spec.needs_stack and not stack_up:
            fails.append("no stack is running")
        if spec.needs_teensy and not _teensy():
            fails.append("no /dev/ttyACM* -- the Teensy is not attached")
        if spec.needs_real and "/real/joint_states" not in topics:
            fails.append("no /real/joint_states -- the real stack is not up")
        # A FOREIGN /robot_description PUBLISHER IS AS FATAL AS A SECOND STACK,
        # and nothing was watching for it. A bare robot_state_publisher left
        # over from figure work (the CAD master arm, which has no ros2_control
        # tag) won the topic; the stack's ros2_control_node read it, threw
        # "no 'ros2_control' tag found in the URDF" and DIED at startup. The
        # visible symptom was every robot joint reading 0.000 -- a plausible
        # posture, published by a fallback joint_state_publisher, with the
        # controller manager gone. Cost 15 minutes here and would cost a day
        # in the lab, because nothing downstream disagrees.
        if spec.starts_stack:
            stray = _foreign_description()
            if stray:
                fails.append(
                    "a robot_state_publisher outside this stack owns "
                    "/robot_description (pid %s) -- it will kill "
                    "ros2_control_node with \"no 'ros2_control' tag\" and "
                    "every joint will read 0.000. kill %s"
                    % (", ".join(str(x) for x in stray),
                       " ".join(str(x) for x in stray)))
        return fails

    def on_stop_jobs(self):
        n = 0
        for label, p in self.jobs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), 2)     # SIGINT to the group
                    n += 1
                except Exception:                             # noqa: BLE001
                    pass
        self.jobs = []
        self.bus.note("SIGINT to %d job(s)" % n)

    # ----------------------------------------------------------- self test
    def on_self_test(self):
        """PROVE EVERY INDICATOR DISTINGUISHES 'NOTHING WRONG' FROM 'NOT CHECKED'.

        This is the requirement the whole GUI is built to satisfy, made
        executable. Three synthetic snapshots are pushed through the SAME
        `refresh()` the live data uses, and each indicator is required to
        render all of them differently where it can:

            GOOD    healthy data on every topic
            BAD     data present, and abnormal
            ABSENT  nothing publishing at all

        The two assertions and why each one is separate:

          GOOD != ABSENT   the user's requirement literally. An indicator that
                           renders the same thing for "measured and fine" and
                           for "no publisher" is a lie in the calm direction,
                           and it is exactly what "IK BLOCKED, 0% over zero
                           attempts" was -- a stuck reading that pointed at
                           the solver instead of at the missing input.
          GOOD != BAD      that the formatter reacts to content at all.

        BAD vs ABSENT is reported but NOT asserted, because for some
        indicators absence IS the abnormality -- a detector that is not
        publishing has no other way to be wrong -- and demanding a third
        distinct rendering there would be demanding a distinction that does
        not exist.

        COMPARED ON (value, colour, NOTE). Dropping the note was the first
        version's mistake: "--" purple "NO ATTEMPTS -- follower up, no poses
        in" and "--" purple "no ik_status published" are different readings
        that differ only in the note, and that difference is the entire point
        of the indicator.
        """
        live = self.bus.snap
        shots = {}
        try:
            for name, snap in (("good", _good_snapshot()),
                               ("bad", _bad_snapshot()),
                               ("absent", _absent_snapshot())):
                self.bus.snap = snap
                self.refresh()
                shots[name] = {k: (w.value, w.colour, w.note)
                               for k, w in self.ind.items()}
        finally:
            self.bus.snap = live
            self.refresh()

        same_absent = sorted(k for k in shots["good"]
                             if shots["good"][k] == shots["absent"][k])
        same_bad = sorted(k for k in shots["good"]
                          if shots["good"][k] == shots["bad"][k])
        info = sum(1 for k in shots["bad"]
                   if shots["bad"][k] == shots["absent"][k])
        n = len(shots["good"])

        if same_absent or same_bad:
            msg = []
            if same_absent:
                msg.append("healthy == NOT CHECKED for %s"
                           % ", ".join(same_absent))
            if same_bad:
                msg.append("healthy == abnormal for %s" % ", ".join(same_bad))
            self.selftest_lbl.setText("SELF-TEST FAILED: " + "; ".join(msg))
            self.selftest_lbl.setStyleSheet("color:%s" % C_BAD)
            self.bus.note("indicator self-test FAILED: %s" % "; ".join(msg),
                          bad=True)
            return False

        self.selftest_lbl.setText(
            "SELF-TEST PASS: %d/%d indicators separate healthy, abnormal and "
            "NOT CHECKED (%d treat absence as the abnormality)"
            % (n, n, info))
        self.selftest_lbl.setStyleSheet("color:%s" % C_OK)
        self.bus.note("indicator self-test PASS: %d indicators, %d treat "
                      "absence as the abnormality" % (n, info))
        return True

    # ------------------------------------------------------------- refresh
    def _refresh_session(self, s):
        """READY TO RUN and the live session state.

        The panel is handed the raw dict; when there is none it renders NO
        READINESS REPORT rather than anything green -- an absent verdict is
        not a pass, which is the whole point of the tri-state.
        """
        import json as _j
        try:
            self.ready_panel.set_data(_j.loads(s["ready"])
                                      if isinstance(s.get("ready"), str)
                                      else s.get("ready"))
        except Exception:                                        # noqa: BLE001
            self.ready_panel.set_data(None)
        try:
            d = _j.loads(s["sess"]) if isinstance(s.get("sess"), str) \
                else s.get("sess")
        except Exception:                                        # noqa: BLE001
            d = None
        if not d:
            self.sess_lbl.setText("no session manager "
                                  "(ros2 run srl_experiments session_manager)")
            return
        t = d.get("trial")
        if t:
            el = d.get("elapsed") or 0.0
            self.sess_lbl.setText(
                "%s   %s   block %s   trial %d/%d   attempt %d   "
                "elapsed %5.1f s   mode %s   task %s"
                % (d.get("participant"), d.get("state"), t.get("block"),
                   (t.get("index") or 0) + 1, d.get("total", 0),
                   t.get("attempt", 1), el, t.get("mode"), t.get("task")))
        else:
            self.sess_lbl.setText("%s   %s   %d/%d trials done"
                                  % (d.get("participant"), d.get("state"),
                                     d.get("done", 0), d.get("total", 0)))
        lines = []
        for e in (d.get("log") or [])[-24:]:
            mark = {"bad": "!!", "warn": " *", "good": " +"}.get(
                e.get("level"), "  ")
            lines.append("%s %s" % (mark, e.get("text", "")))
        self.sess_log.setPlainText("\n".join(lines))
        self.sess_log.verticalScrollBar().setValue(
            self.sess_log.verticalScrollBar().maximum())

    def refresh(self):
        t0 = time.perf_counter()
        s = self.bus.snap
        now = s.get("t", time.monotonic())
        try:
            self._refresh_session({k: (s.get(k) or [None])[0]
                                   for k in ("ready", "sess")})
        except Exception as e:                                   # noqa: BLE001
            self.log("session panel: %s" % e, bad=True)

        def val(key, default=None):
            v = s.get(key)
            return default if v is None else v[0]

        # ---- capability
        for a in ARMS:
            k = "capability_%s" % a
            raw = val("cap_%s" % a)
            if raw is None:
                self.ind[k].set("--", C_UNKNOWN, "capability_node not running")
            else:
                try:
                    d = json.loads(raw)
                except Exception:                             # noqa: BLE001
                    self.ind[k].set("?", C_UNKNOWN, "unparsable")
                    continue
                col = (C_OK if d.get("key") in ("FK", "SPHERICAL")
                       else C_WARN if d.get("position_available") else C_BAD)
                cost = d.get("cost_m")
                note = (("cost %.0f mm" % (1000 * cost["mean_m"]))
                        if cost else "cost unmeasured")
                if not d.get("position_available"):
                    note = "NO POSITION"
                self.ind[k].set("%s %s" % (a[:1].upper(), d.get("key", "?")),
                                col, note)

        raw = val("chan")
        # The channel line now carries the baseline FILENAME when degraded, so
        # the operator can see WHICH stored file is freezing channels without
        # opening a log.
        self.ind["channels"].set(
            (raw or "--").split("|")[0].strip() if raw else "--",
            (C_WARN if raw and "DEGRADED" in raw else C_MUTED) if raw
            else C_UNKNOWN,
            (raw.split("|")[-1].strip()[:38] if raw else "not publishing"))

        # CLUTCH. "ENGAGED" is a SUBSTRING OF "DISENGAGED", so a containment
        # test reports a disengaged clutch as engaged -- found by the
        # indicator self-test, and the same shape as the `/blocking` substring
        # match that reported every registered blocker as active. Word
        # boundaries, and DISENGAGED is tested first.
        clutch, engaged_any = [], False
        for a in ARMS:
            m = val("mstat_%s" % a)
            u = a[0].upper()
            if m is None:
                clutch.append("%s?" % u)
            elif re.search(r"\bDISENGAGED\b", str(m)):
                clutch.append("%s-" % u)
            elif re.search(r"\bENGAGED\b", str(m)):
                clutch.append("%s+" % u)
                engaged_any = True
            else:
                clutch.append("%s?" % u)
        any_status = any(val("mstat_%s" % a) is not None for a in ARMS)
        self.ind["clutch"].set(
            " ".join(clutch),
            (C_TEXT if engaged_any else C_WARN) if any_status else C_UNKNOWN,
            "+ engaged / - disengaged" if any_status else "no /master_status_*")

        # ---- scene
        raw = val("scene")
        if raw is None:
            self.ind["fingerprint"].set("--", C_UNKNOWN, "node not running")
        else:
            try:
                d = json.loads(raw)
            except Exception:                                 # noqa: BLE001
                d = {}
            st = d.get("state", "?")
            col = {"match_skip_calibration": C_OK, "registered": C_OK,
                   "changed_reregistered": C_WARN, "sweeping": C_MUTED,
                   "sweep_invalid_no_detections": C_BAD}.get(st, C_UNKNOWN)
            self.ind["fingerprint"].set(
                str(st).replace("_", " "), col,
                "%d object(s), %d drift flag(s)"
                % (d.get("n_stored", 0), len(d.get("drift_flags", []))))

        topics = s.get("topics", set())
        for a in ARMS:
            t = "/perception/detections/%s" % a
            self.ind["detector_%s" % a].set(
                *(("present", C_OK, t) if t in topics
                  else ("absent", C_UNKNOWN, "no detector publishing")))

        # ---- mode / autonomy / intent
        # MODE, from live publishers -- with CONFLICT as a real state. The
        # project's rule is one source at a time (`autonomy_has_control`
        # suppresses the master for exactly this reason), so two sources
        # claiming the arm is a fault and not a display detail. Without this
        # the indicator had no abnormal state at all and could not be shown
        # to be reading anything.
        vr, chan = val("vr"), val("chan")
        auto = val("autonomy")
        driving = [n for n, v in (("VR", vr), ("DIRECT", chan)) if v]
        if auto:
            try:
                if json.loads(auto).get("stage") not in (None, "DIRECT", "IDLE"):
                    driving.append("AUTONOMY")
            except Exception:                                 # noqa: BLE001
                pass
        if len(driving) > 1:
            self.ind["mode"].set("CONFLICT", C_BAD,
                                 "%s all claim the arm" % "+".join(driving))
        elif driving:
            self.ind["mode"].set(driving[0], C_TEXT, "from live publishers")
        else:
            self.ind["mode"].set("--", C_UNKNOWN, "no source is publishing")
        au = val("autonomy")
        if au is None:
            self.ind["autonomy"].set("--", C_UNKNOWN, "no autonomy node")
            self.ind["intent"].set("--", C_UNKNOWN, "no intent published")
        else:
            try:
                d = json.loads(au)
            except Exception:                                 # noqa: BLE001
                d = {}
            self.ind["autonomy"].set(str(d.get("stage", "?")), C_TEXT,
                                     str(d.get("note", ""))[:34])
            p = d.get("confidence")
            self.ind["intent"].set(
                "--" if p is None else "%.2f" % p,
                C_UNKNOWN if p is None else (C_OK if p > 0.8 else C_WARN),
                "top-1 confidence")

        # ---- gripper reference (Part 3.2): startup verdict, per arm
        g = val("grip")
        if g is None:
            self.ind["gripper"].set("--", C_UNKNOWN, "fsr_gripper_node absent")
        else:
            try:
                d = json.loads(g)
            except Exception:                                 # noqa: BLE001
                d = {}
            st = d.get("startup", {})
            worst = ("no_feedback" if "no_feedback" in st.values() else
                     "open_UNCONFIRMED" if "open_UNCONFIRMED" in st.values()
                     else "holding" if "holding" in st.values()
                     else "open_confirmed" if st else "?")
            col = {"open_confirmed": C_OK, "holding": C_WARN,
                   "open_UNCONFIRMED": C_BAD, "no_feedback": C_UNKNOWN,
                   "pending": C_MUTED}.get(worst, C_UNKNOWN)
            self.ind["gripper"].set(worst.replace("_", " "), col,
                                    "open reference")

        # ---- clearance
        for a in ARMS:
            v = val("ik_%s" % a)
            k = "clearance_%s" % a
            if not v or len(v) < 8:
                self.ind[k].set("--", C_UNKNOWN, "no ik_status")
                continue
            clr = v[5]
            col = (C_UNKNOWN if clr < 0 else C_BAD if clr < 0.12
                   else C_WARN if clr < 0.20 else C_OK)
            self.ind[k].set("--" if clr < 0 else "%.3f m" % clr, col,
                            "floor 0.12 m")

        # ---- safety
        es = val("estop")
        self.ind["estop"].set(
            "LATCHED" if es else ("clear" if es is not None else "--"),
            C_BAD if es else (C_TEXT if es is not None else C_UNKNOWN),
            "/estop_state")
        bl = val("blocking")
        if bl is None:
            self.ind["blockers"].set("--", C_UNKNOWN,
                                     "blocking_aggregator not running")
        else:
            try:
                d = json.loads(bl)
                n, unk = int(d.get("n_blocking", 0)), int(d.get("state_unknown", 0))
            except Exception:                                 # noqa: BLE001
                n, unk = 0, 0
            self.ind["blockers"].set(
                "%d" % n, C_BAD if n else (C_UNKNOWN if unk else C_TEXT),
                "%d unknown" % unk)

        # ---- IK. ZERO ATTEMPTS IS NOT ZERO PER CENT. This exact conflation
        # -- "BLOCKED, 0% success" over no attempts -- pointed an operator at
        # the solver instead of at the missing input, and it is why the
        # attempt count is part of the reading rather than a footnote.
        for a in ARMS:
            v = val("ik_%s" % a)
            k = "ik_%s" % a
            if not v:
                self.ind[k].set("--", C_UNKNOWN, "no ik_status published")
                continue
            att, suc = (v[0] or 0), (v[1] or 0)
            if not att:
                self.ind[k].set("--", C_UNKNOWN,
                                "NO ATTEMPTS -- follower up, no poses in")
                continue
            pct = 100.0 * suc / att
            self.ind[k].set(
                "%.0f%%" % pct,
                C_OK if pct > 90 else C_WARN if pct > 60 else C_BAD,
                "%d attempts" % att)

        self.master_schema.set_data(s.get("master_schema"))
        self.robot_schema.set_data(s.get("robot_schema"))
        self._sync_embedded()
        self._refresh_divergence(s)
        self._refresh_cameras(s)

        # ---- banner. A narration override wins: the tutorial recorder drives
        # the banner as its caption track, and refresh() runs at 10 Hz, so
        # without this the narration is overwritten before a frame is
        # captured -- the caption was invisible in the first take.
        if getattr(self, "_narration", None):
            self.banner.setText(self._narration[0])
            self.banner.setStyleSheet(self._narration[1])
        elif es:
            # THE ONLY SLAB IN THE INTERFACE. Reserved for the one state that
            # must interrupt whatever the operator was reading.
            self.banner.setText("E-STOP LATCHED")
            self.banner.setStyleSheet(
                "color:#0b0f13;background:%s;padding:5px;letter-spacing:3px"
                % C_BAD)
        elif not topics:
            self.banner.setText("NO ROS GRAPH")
            self.banner.setStyleSheet(
                "color:%s;padding:5px;letter-spacing:3px;"
                "border-bottom:1px solid %s" % (C_UNKNOWN, LINE))
        else:
            self.banner.setText("SRL  ·  OPERATIONS")
            self.banner.setStyleSheet(
                "color:%s;padding:5px;letter-spacing:4px;"
                "border-bottom:1px solid %s" % (C_MUTED, LINE))

        # charts
        t = time.monotonic()
        ee = {a: self.bus.ee("sim", a) for a in ARMS}
        self.strips["ee_z"].push(t, ee["left"][2] if ee["left"] else None,
                                 ee["right"][2] if ee["right"] else None)
        clr = {}
        for a in ARMS:
            v = val("ik_%s" % a)
            clr[a] = (v[5] if v and len(v) > 5 and v[5] >= 0 else None)
        self.strips["clearance"].push(t, clr["left"], clr["right"])
        dv_ = s.get("div", {})
        self.strips["lag"].push(
            t,
            dv_.get("left").max_rad if (dv_.get("left") and dv_["left"].measured) else None,
            dv_.get("right").max_rad if (dv_.get("right") and dv_["right"].measured) else None)
        tr = val("trial")
        self.trial_lbl.setText(str(tr)[:300] if tr else "no /trial_state")

        self.eventlog.setPlainText("\n".join(
            "%s %s%s" % (ts, "! " if bad else "  ", txt)
            for ts, txt, bad in s.get("log", [])[-200:]))
        self.eventlog.verticalScrollBar().setValue(
            self.eventlog.verticalScrollBar().maximum())

        dt = (time.perf_counter() - t0) * 1000.0
        self._ft.append(dt)
        self._ft = self._ft[-300:]
        srt = sorted(self._ft)
        self.frame_lbl.setText(
            "frame %.2f ms   median %.2f   p95 %.2f   max %.2f   "
            "(budget 100 ms)   rss %s"
            % (dt, srt[len(srt) // 2], srt[int(len(srt) * 0.95)], srt[-1],
               _rss()))

    def _sync_embedded(self):
        """Force each foreign RViz window onto its container's rectangle.

        POSITIONED IN TOP-LEVEL COORDINATES, NOT (0, 0). Reparenting a foreign
        window does not necessarily attach it to the container's own native
        window -- measured here, `setPosition(0, 0)` put rviz2 at the top-left
        of the WHOLE GUI, covering the banner, the cameras, the indicators and
        the divergence readout, while the Qt layout underneath was correct
        (the geometry dump put every panel exactly where it belonged). So the
        target is the container's position mapped into the top-level window,
        which is right whether the foreign window ended up parented to the
        container or to the window.

        Re-asserted rather than set once, because RViz resizes itself when its
        docks change -- but only at 2 Hz and only on a mismatch: doing this
        every 10 Hz frame pushed the median frame time from 2.5 ms to 10 ms.
        """
        now = time.monotonic()
        if now - getattr(self, "_last_sync", 0.0) < 0.5:
            return
        self._last_sync = now
        from PyQt5.QtCore import QPoint
        for key, (win, cont) in list(self.embedded.items()):
            try:
                tl = cont.mapTo(self.window(), QPoint(0, 0))
                if (win.size() != cont.size()
                        or win.position() != tl):
                    win.setPosition(tl)
                    win.resize(max(1, cont.width()), max(1, cont.height()))
            except Exception:                                 # noqa: BLE001
                self.embedded.pop(key, None)

    def _refresh_divergence(self, s):
        trip = s.get("lag_trip", 0.5)
        self.div_head.setText(
            "per-joint (commanded - actual) and end-effector distance, "
            "against lag_trip_rad = %.2f  [%s]. Only a MEASURED row carries "
            "a number."
            % (trip, s.get("lag_trip_src", "default -- bridge not read")))
        for a in ARMS:
            r = s.get("div", {}).get(a)
            if r is None:
                self.div_state[a].setText("--")
                continue
            if not r.measured:
                # THE WHOLE POINT. A missing real arm reads NO REAL ARM, not
                # 0.000 rad, and it is purple rather than any colour that
                # could be mistaken for healthy.
                label = {dv.NO_SIM: "NO SIM", dv.NO_REAL: "NO REAL ARM",
                         dv.SIM_STALE: "SIM STALE", dv.REAL_STALE: "REAL STALE",
                         dv.REAL_FROZEN: "REAL FROZEN",
                         dv.PARTIAL: "PARTIAL"}.get(r.status, r.status.upper())
                col = C_BAD if r.status == dv.REAL_FROZEN else C_UNKNOWN
                self.div_state[a].setText(label)
                self.div_state[a].setStyleSheet("color:%s" % col)
                self.div_max[a].setText("--")
                self.div_max[a].setStyleSheet("color:%s" % col)
                self.div_ee[a].setText("--")
                self.div_ee[a].setStyleSheet("color:%s" % col)
                self.div_joint[a].setText(r.reason)
                continue
            b = dv.band(r.max_rad, trip)
            col = BAND_COLOUR[b]
            self.div_state[a].setText("MEASURED")
            self.div_state[a].setStyleSheet("color:%s" % C_MUTED)
            self.div_max[a].setText("%.4f rad" % r.max_rad)
            self.div_max[a].setStyleSheet("color:%s" % col)
            ee = r.ee_m
            self.div_ee[a].setText("--" if ee is None else "%.1f mm" % (1000 * ee))
            self.div_ee[a].setStyleSheet(
                "color:%s" % (C_UNKNOWN if ee is None else col))
            self.div_joint[a].setText(
                "worst %s   " % r.max_joint + "  ".join(
                    "j%d %+.3f" % (i + 1, r.per_joint.get("%s_joint_%d" % (a, i + 1), 0.0))
                    for i in range(7)))

    def _refresh_cameras(self, s):
        cams = s.get("cam", {})
        imgs = s.get("cam_img", {})
        for a in ARMS:
            cap, st, show, hz = cams.get(a, ("no data", "absent", False, 0.0))
            col = {"live": C_OK, "stale": C_WARN, "dead": C_BAD,
                   "absent": C_UNKNOWN}.get(st, C_UNKNOWN)
            self.cam_cap[a].setText(cap)
            self.cam_cap[a].setStyleSheet("color:%s" % col)
            if not show:
                # NEVER PAINT A STALE FRAME. A frozen picture of a workspace
                # cannot be told from a live picture of a workspace that is
                # not moving, and under time pressure a dimmed frame is read
                # as a frame. The panel goes to text instead.
                self.cam_lbl[a].setPixmap(QPixmap())
                self.cam_lbl[a].setText(
                    {"absent": "NO CAMERA", "dead": "NO SIGNAL",
                     "stale": "STALE"}.get(st, "NO SIGNAL"))
                self.cam_lbl[a].setStyleSheet(
                    "background:#111;color:%s;border:1px solid %s" % (col, col))
                continue
            raw = imgs.get(a)
            if raw is None:
                continue
            w, h, enc, buf, step = raw
            fmt = {"rgb8": QImage.Format_RGB888, "bgr8": QImage.Format_BGR888,
                   "mono8": QImage.Format_Grayscale8}.get(enc)
            if fmt is None:
                self.cam_lbl[a].setText("unsupported encoding %s" % enc)
                continue
            qi = QImage(buf, w, h, step, fmt)
            self.cam_lbl[a].setPixmap(QPixmap.fromImage(qi).scaled(
                self.cam_lbl[a].width(), self.cam_lbl[a].height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.cam_lbl[a].setStyleSheet(
                "background:#111;border:1px solid #999")

    def closeEvent(self, ev):
        for _, p in self.rviz:
            try:
                p.terminate()
            except Exception:                                 # noqa: BLE001
                pass
        self.on_stop_jobs()
        ev.accept()


# ===========================================================================
#  helpers
# ===========================================================================
def _scratch():
    d = os.environ.get("SRL_SCRATCH") or "/tmp"
    return d



def _foreign_description():
    """PIDs of robot_state_publishers that are NOT part of a launched stack.

    A launched one carries --params-file (the launch writes the description
    into a parameter file); a hand-started one is given a URDF path on the
    command line. That difference is what distinguishes the stack's own
    publisher from a stray, and it needs no bookkeeping to stay true.
    """
    out = []
    for pid, cmd in procscan.find("robot_state_publisher"):
        if "--params-file" in cmd:
            continue
        if ".urdf" in cmd or ".xacro" in cmd:
            out.append(pid)
    return out


def _stack_pids():
    try:
        o = subprocess.run(
            ["pgrep", "-fc", "lib/srl_teleop/master_pose_node|"
                             "lib/moveit_ros_move_group/move_group"],
            capture_output=True, text=True, timeout=5).stdout.strip()
        return int(o or 0)
    except Exception:                                         # noqa: BLE001
        return 0


def _teensy():
    import glob
    return bool(glob.glob("/dev/ttyACM*") or glob.glob("/dev/ttyUSB*"))


def _rss():
    """Total RSS of this process plus its RViz children, in MB.

    Reported because the brief asks: this is a 15 GB machine and two earlier
    runs were OOM-killed, so a second RViz is a memory decision as much as a
    layout one.
    """
    try:
        out = subprocess.run(["ps", "-o", "rss=", "-p",
                              ",".join(str(p) for p in _family())],
                             capture_output=True, text=True, timeout=5).stdout
        kb = sum(int(x) for x in out.split() if x.isdigit())
        return "%.0f MB" % (kb / 1024.0)
    except Exception:                                         # noqa: BLE001
        return "--"


def _family():
    pids = [os.getpid()]
    try:
        o = subprocess.run(["pgrep", "-P", str(os.getpid())],
                           capture_output=True, text=True, timeout=5).stdout
        pids += [int(x) for x in o.split() if x.isdigit()]
    except Exception:                                         # noqa: BLE001
        pass
    return pids


def _sides(same=True):
    """(sim, real) joint-state trackers for a synthetic snapshot."""
    names = dv.joint_names("left") + dv.joint_names("right")
    sim = dv.Side("sim")
    sim.update(names, [0.1] * 14)
    if not same:
        return sim, dv.Side("real")            # never published -> NO_REAL
    real = dv.Side("real")
    real.update(names, [0.1] * 14)
    return sim, real


def _absent_snapshot():
    """NOTHING IS PUBLISHING. The state every indicator must be able to show.

    Deliberately not "an empty dict": the GUI must survive a snapshot that
    has the right shape and no content, which is what a running GUI attached
    to a dead graph actually sees.
    """
    return {"t": time.monotonic(), "topics": set(),
            "cam": {a: ("no data", "absent", False, 0.0) for a in ARMS},
            "cam_img": {a: None for a in ARMS},
            "div": {a: dv.compare(dv.Side("sim"), dv.Side("real"),
                                  dv.joint_names(a)) for a in ARMS},
            "lag_trip": 0.5, "log": []}


def _good_snapshot():
    """Everything publishing and healthy. The calm reading."""
    t = time.monotonic()

    def w(v):
        return (v, t)
    sim, real = _sides(same=True)
    return {
        "t": t,
        "topics": {"/perception/detections/%s" % a for a in ARMS},
        "estop": w(False),
        "chan": w("FULL | 14/14 coherent | left: use j1234567"),
        "cap_left": w(json.dumps(dict(key="SPHERICAL", position_available=True,
                                      cost_m=dict(mean_m=0.0)))),
        "cap_right": w(json.dumps(dict(key="SPHERICAL", position_available=True,
                                       cost_m=dict(mean_m=0.0)))),
        "mstat_left": w("clutch ENGAGED"),
        "mstat_right": w("clutch ENGAGED"),
        "scene": w(json.dumps(dict(state="registered", n_stored=3,
                                   drift_flags=[]))),
        "autonomy": w(json.dumps(dict(stage="DIRECT", note="idle",
                                      confidence=0.95))),
        "grip": w(json.dumps(dict(startup=dict(left="open_confirmed",
                                               right="open_confirmed")))),
        "blocking": w(json.dumps(dict(n_blocking=0, state_unknown=0))),
        "ik_left": w([100.0, 100.0, 0, 0, 0, 0.35, 0, 0]),
        "ik_right": w([100.0, 100.0, 0, 0, 0, 0.35, 0, 0]),
        "cam": {a: ("live 15.0 Hz 320x240", "live", False, 15.0) for a in ARMS},
        "cam_img": {a: None for a in ARMS},
        "div": {a: dv.compare(sim, real, dv.joint_names(a)) for a in ARMS},
        "lag_trip": 0.5, "lag_trip_src": "bridge", "log": [],
    }


def _bad_snapshot():
    """Everything publishing and ABNORMAL.

    Built from the same shapes the live subscriptions produce, so it exercises
    the real formatters rather than a parallel path that could drift from
    them. `ik_left` carries ZERO ATTEMPTS on purpose -- that is the exact
    reading this GUI once mislabelled as "BLOCKED, 0% success".
    """
    t = time.monotonic()

    def w(v):
        return (v, t)
    sim, real = _sides(same=False)
    return {
        "t": t,
        "topics": set(),
        "estop": w(True),
        "chan": w("DEGRADED | 6/14 coherent | baseline=old.json STALE?"),
        "cap_left": w(json.dumps(dict(key="NONE", position_available=False))),
        "cap_right": w(json.dumps(dict(key="NONE", position_available=False))),
        "mstat_left": w("clutch DISENGAGED"),
        "mstat_right": w("clutch DISENGAGED"),
        "scene": w(json.dumps(dict(state="sweep_invalid_no_detections",
                                   n_stored=0, drift_flags=["a"]))),
        "autonomy": w(json.dumps(dict(stage="REFUSED", note="synthetic",
                                      confidence=0.10))),
        "grip": w(json.dumps(dict(startup=dict(left="open_UNCONFIRMED",
                                               right="no_feedback")))),
        "blocking": w(json.dumps(dict(n_blocking=3, state_unknown=2))),
        "ik_left": w([0.0] * 8),
        "ik_right": w([10.0, 1.0, 0, 0, 0, 0.05, 0, 0]),
        "cam": {a: ("no signal", "dead", False, 0.0) for a in ARMS},
        "cam_img": {a: None for a in ARMS},
        "div": {a: dv.compare(sim, real, dv.joint_names(a)) for a in ARMS},
        "lag_trip": 0.5, "lag_trip_src": "bridge", "log": [],
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-rviz", action="store_true",
                    help="indicators only; for capture and headless checks")
    ap.add_argument("--embed-rviz", action="store_true",
                    help="reparent RViz INTO a panel. Measured on this "
                         "display stack: the foreign window is moved but NOT "
                         "CLIPPED to its container, so it paints over the "
                         "cameras, indicators and divergence readout. Needs a "
                         "window manager; none is installed here.")
    ap.add_argument("--dual-rviz", action="store_true",
                    help="TWO RViz panels side by side. Needs a window "
                         "manager: without one, Qt does not clip a reparented "
                         "foreign window to its container and the two "
                         "instances paint over each other and over this GUI "
                         "(measured -- see docs/system/07_gui_rviz_embedding.md)")
    ap.add_argument("--single-rviz", action="store_true",
                    help="deprecated alias; the ghosted single panel is now "
                         "the default")
    ap.add_argument("--tab", default=None,
                    help="select a tab by name at startup, e.g. --tab ready")
    ap.add_argument("--self-test-exit", action="store_true",
                    help="run the indicator self-test and exit with its result")
    args, rest = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    # AN UNKNOWN FLAG MUST NOT PASS SILENTLY. parse_known_args is here so ROS
    # can have its --ros-args, but it also swallowed `--tab ready` without a
    # word: the flag did nothing, said nothing, and the capture kept showing
    # the wrong tab while a stale GUI was still running. That is the same
    # silent-acceptance class as the volatile QoS and the misplaced RViz key.
    unknown = [r for r in rest
               if r.startswith("-") and not r.startswith("--ros-arg")]
    if unknown:
        ap.error("unrecognised argument(s): %s\n"
                 "(ROS arguments after --ros-args are still accepted)"
                 % " ".join(unknown))

    rclpy.init(args=None)
    bus = Bus()
    threading.Thread(target=lambda: rclpy.spin(bus), daemon=True).start()
    app = QApplication([sys.argv[0]] + rest)
    app.setFont(helvetica(10))
    app.setStyleSheet(STYLE)
    g = Gui(bus, args)
    g.show()

    if args.self_test_exit:
        rc = {"v": 1}

        def run():
            ok = g.on_self_test()
            print(g.selftest_lbl.text())
            rc["v"] = 0 if ok else 1
            app.quit()
        QTimer.singleShot(1500, run)
        app.exec_()
        bus.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return rc["v"]

    r = app.exec_()
    bus.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return r


if __name__ == "__main__":
    sys.exit(main())
