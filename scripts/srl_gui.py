#!/usr/bin/env python3
"""SRL operations GUI -- DUAL VIEW: commanded beside actual, and the gap named.

    bash scripts/start_gui.sh                  # THE ONE COMMAND. Use this.
    python3 scripts/srl_gui.py                 # if the environment is already set
    python3 scripts/srl_gui.py --no-embed-rviz # RViz as its own window
    python3 scripts/srl_gui.py --no-rviz       # indicators only (capture/CI)

THREE THINGS ARE ON SCREEN AT ONCE, and that is the whole layout argument:

  COMMANDED   RViz, the sim, driven by whichever mode is active. Embedded
              where a window manager exists (detected, see `_have_wm`).
  ACTUAL      where the REAL arms are, drawn from the `real_*` frames in the
              shared /tf, or from /real/joint_states through the robot model
              when the real publisher is not up. The panel names which.
              See scripts/srl_arm_view.py.
  DIVERGENCE  underneath both: per-joint (commanded - actual) and end-effector
              distance, against the trip threshold READ FROM THE BRIDGE.

THE LEFT COLUMN IS GROUPED BY WHAT YOU ARE DOING, not by what the system
contains: SET UP (connect the arms, the things you set once), RUN (start a
mode, run a task, the controls you touch while it runs), STATUS (read-only).

CONNECTING THE REAL ARMS IS A PANEL, NOT A MEMORY TEST. Eight checks, each in
plain words with the fix as a button: the discovery partition, leftover shared
memory, a hung daemon, a second stack, pieces of an earlier run still going,
the Teensy moving between sockets, the sim/real home gap, and a leaked Kortex
session. The checks live in srl_teleop/real_arm_doctor.py so a test can hand
each one a deliberately broken world -- see test_real_arm_doctor.py.

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
from datetime import datetime

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
# THE SEVEN CONNECTION FAULTS, EACH WITH A BUTTON. Kept out of this
# file so they can be driven by a test with a deliberately broken
# world -- see test_real_arm_doctor.py. A panel whose checks can only
# be exercised by breaking the actual machine is a panel nobody
# exercises.
from srl_teleop import real_arm_doctor as rad                 # noqa: E402
# THE ONE BUTTON. The twelve-step VR bring-up lives in its own module for
# the same reason the connection checks do: so a test can hand it a
# deliberately broken machine and require it to say so, without binding
# a port or deleting a certificate on the machine running the test.
from srl_teleop import vr_bringup as vrb                     # noqa: E402

# THE MOTION GENERATORS, AND WHAT EACH COSTS. Measured offline by
# `scripts/measure_teleop_motion.py` on the left arm's shipped home plus
# [0.50 -0.30 0.10 0.25 -0.05 0.15 0.02] at 50 Hz; full numbers in
# recordings/baselines/teleop_motion.json. "Off the path" is how far the HAND
# strays from the curve the straight joint-space line traces, against a 30 mm
# grasp capture gate.
# THE LABELS ARE SHORT BECAUSE THE COLUMN IS NARROW. At the shipped column
# width a drop-down eats ~30 px for its arrow, and the first version's
# "Ruckig -- jerk-limited, synchronised (default)" rendered as
# "...synchronised (default" -- a closing bracket lost off the right edge,
# which is this window's own recurring defect. The detail belongs in the note
# under the box, which wraps.
MOTION_GENERATORS = [
    ("ruckig", "Ruckig -- jerk-limited (default)"),
    ("clamp", "synchronised clamp"),
    ("legacy", "legacy clamp_towards"),
]
MOTION_NOTES = [
    ("ruckig",
     "Every joint arrives on the same cycle and the hand stays ON the path "
     "it was asked for: 0.0 mm off it, peak joint speed 1.00x the limit in "
     "joint_limits.yaml."),
    ("clamp",
     "One scale factor for the whole joint vector, so it is synchronised and "
     "inside the velocity limits -- but NOT jerk-limited: 62x the "
     "jerk-limited acceleration step. This is the fallback used when ruckig "
     "is not installed. Choosing it deliberately is unusual."),
    ("legacy",
     "The per-joint clamp shipped before 2026-08-23. NOT synchronised: the "
     "hand leaves the commanded path by 51.3 mm against a 30 mm grasp gate, "
     "and peak joint speed is 12.5x the limit in joint_limits.yaml. Select "
     "it only to reproduce a recording made before that date."),
]

from PyQt5.QtCore import Qt, QTimer                          # noqa: E402
from PyQt5.QtGui import (QColor, QFont, QImage, QPalette,    # noqa: E402
                         QPixmap, QWindow)
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox,  # noqa: E402
                             QGridLayout,
                             QGroupBox, QHBoxLayout, QLabel, QMainWindow,
                             QPushButton, QScrollArea, QSizePolicy, QSlider,
                             QSpinBox,
                             QSplitter, QTabBar, QTabWidget, QTextEdit,
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
import srl_arm_view as av                                    # noqa: E402
import srl_wearer_panel as swp                               # noqa: E402

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
        # PERSISTENT PUBLISHERS, keyed by (topic, type). See publish_once.
        self._pubs = {}
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
    def _prime_estop_publisher(self):
        """Create the /estop publisher at START-UP, not at the first press.

        Even with a cached publisher, the FIRST press would still be the one
        that pays for discovery -- and that press is the e-stop. Creating it
        here means DDS has matched long before anybody needs it.

        Nothing is published. A publisher that exists is not a stop.
        """
        from std_msgs.msg import Bool as _Bool
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pubs[("/estop", "Bool")] = self.create_publisher(
            _Bool, "/estop", qos)

    def _sub(self):
        from rclpy.qos import qos_profile_sensor_data
        self._prime_estop_publisher()
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
                     ("/session/state", "sess"),
                     # THE ROBOT'S OWN VOICE. `calibrate_environment` and
                     # `pick_from_map` publish one JSON sentence per phase --
                     # "calibrating", "reaching cell 7 of 12", "grasping" --
                     # and the operator asked to be told what it is doing
                     # rather than watch an arm move for reasons nobody can
                     # see. Same topic the log carries, so the window and the
                     # terminal cannot disagree.
                     ("/robot_say", "say"),
                     ("/world_map", "worldmap")):
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
        # THE SCENE CAMERA AND THE WEARER ESTIMATE. Separate from the wrist
        # cameras because they answer a different question: the wrist cameras
        # show the OBJECT at grasp range, this one shows the PERSON the arms
        # are bolted to.
        for topic, key in (("/wearer/estimate", "wearer"),
                           ("/wearer/overlay", "wearer_overlay"),
                           ("/scene_camera/state", "scene_cam")):
            self.create_subscription(String, topic,
                                     lambda m, k=key: self._set(k, m.data), 10)
        self.scene_img = None
        self.scene_img_t = 0.0
        self.create_subscription(
            Image, "/scene_camera/image_raw", self._on_scene_image,
            qos_profile_sensor_data)

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

    def _on_scene_image(self, msg):
        # Kept RAW and converted only where it is painted, like the wrist
        # cameras -- ROS-thread time is not spent on frames the GUI is about
        # to discard as stale.
        self.scene_img = (msg.width, msg.height, bytes(msg.data), msg.step,
                          msg.encoding)
        self.scene_img_t = time.monotonic()

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
        # THE ACTUAL PANEL'S POINTS, taken on the ROS thread with everything
        # else. Computed here rather than in the widget so the widget never
        # touches TF and never blocks the paint on a lookup.
        s["skel"] = self._skeletons()
        # The raw joint dictionaries, for the connection panel's home check.
        # Copied, not shared: the panel must not see a dict mutate under it.
        s["sim_pos"] = {k: v for a in ARMS for k, v in self.sim[a].pos.items()}
        s["real_pos"] = {k: v for a in ARMS for k, v in self.real[a].pos.items()}
        s["real_age"] = {a: self.real[a].arrival_age() for a in ARMS}
        s["master_schema"] = self._master_schema()
        s["robot_schema"] = self._robot_schema()
        s["scene_img"] = self.scene_img
        s["scene_img_t"] = self.scene_img_t
        s["log"] = list(self.log[-200:])
        self.snap = s

    # --------------------------------------------------- the ACTUAL panel
    def _skeletons(self):
        """(real, sim, note, state) for the ACTUAL view.

        TWO SOURCES, IN ORDER, AND THE ONE USED IS NAMED ON SCREEN:

          1. the `real_*` frames in the SHARED /tf -- the stack's own answer,
             computed by the same publisher RViz would read. THERE IS NO
             /real/tf; both real launches use frame_prefix="real_".
          2. forward kinematics on /real/joint_states, when the joint stream
             is up but the real robot_state_publisher is not. That is a state
             this rig actually reaches, and without this branch the panel
             would read NO REAL ARM while the arm was reporting.

        THERE IS NO THIRD SOURCE. When neither works, the skeletons come back
        empty and the panel draws nothing -- it never holds the last good
        pose, because a frozen skeleton and a still arm are the same picture.
        """
        real, sim = {}, {}
        used = None
        for a in ARMS:
            sk = av.from_tf(self, a, "real_")
            if sk.ok:
                real[a] = sk
                used = used or sk.source
            else:
                real[a] = av.Skeleton(a)
            sim[a] = av.from_tf(self, a, "")
        # Fall back to FK only for arms TF could not answer for, and only
        # when that arm is actually reporting joint angles.
        for a in ARMS:
            if real[a].ok:
                continue
            q = self._joint_vector(self.real[a], a)
            if q is None:
                continue
            fk = self._fk()
            sk = av.from_fk(fk, a, q)
            if sk.ok:
                real[a] = sk
                used = used or sk.source
            if not sim[a].ok:
                qs = self._joint_vector(self.sim[a], a)
                if qs is not None and fk is not None:
                    sim[a] = av.from_fk(fk, a, qs)
        n_real = sum(1 for a in ARMS if real[a].ok)
        if n_real == 0:
            note, state = ("nothing is reporting where the real arms are",
                           "unknown")
        else:
            ages = [self.real[a].arrival_age() for a in ARMS
                    if self.real[a].arrival_age() is not None]
            age = min(ages) if ages else None
            stale = age is not None and age > 2.0
            note = "%d of 2 arms, from %s%s" % (
                n_real, used or "the arms' own report",
                "" if age is None else "  (last heard %.1f s ago)" % age)
            state = "bad" if stale else ("ok" if n_real == 2 else "bad")
        return real, sim, note, state

    @staticmethod
    def _joint_vector(side, arm):
        """Seven joint angles for `arm` from a divergence.Side, or None.

        ALL SEVEN OR NONE. Six angles and a zero draws a plausible arm in the
        wrong place, which is worse than drawing nothing.
        """
        out = []
        for i in range(7):
            v = side.pos.get("%s_joint_%d" % (arm, i + 1))
            if v is None:
                return None
            out.append(v)
        return out

    def _fk(self):
        """The URDF kinematics, built once, lazily, and never on the paint
        path. Returns None if the model cannot be built -- which is UNKNOWN,
        not a reason to draw an arm at zero."""
        if hasattr(self, "_fk_cache"):
            return self._fk_cache
        self._fk_cache = None
        try:
            import srl_fk
            self._fk_cache = srl_fk.FK()
        except Exception as e:                                # noqa: BLE001
            self.note("robot model unavailable for the actual view: %r" % (e,))
        return self._fk_cache

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
    def submit(self, fn, label=None):
        """Queue an action for the ROS thread, AND SAY SO IMMEDIATELY.

        The queue is drained by a 0.05 s timer, so under load a press could
        leave no trace for hundreds of milliseconds -- and a press with no
        trace cannot be told from a disconnected signal, which is the whole
        bug class the button audit exists for. It caught this as an
        intermittent failure on `re-base anchor`: the same button passed on
        one run and failed on the next, which is worse than failing every
        time, because it reads as a flaky test rather than a real gap.
        """
        if label:
            self.note(label)
        self._q.append(fn)

    def _drain(self):
        while self._q:
            fn = self._q.pop(0)
            try:
                fn()
            except Exception as e:                            # noqa: BLE001
                self.note("action failed: %r" % (e,), bad=True)

    def publish_once(self, msg_type, topic, value):
        """Publish one message on a PERSISTENT publisher.

        THIS USED TO CREATE A PUBLISHER, PUBLISH, AND DROP IT on the same
        three lines, and both halves of that are fragile:

          * a brand-new DDS publisher has not finished discovery with the
            subscribers that already exist, so its first message can be sent
            to nobody;
          * `p` then went out of scope and was garbage-collected, taking the
            publisher with it before any late-completing match could be used.

        Publishers are now cached per (topic, type) and live as long as this
        node, and `/estop`'s is created at start-up rather than on the first
        press -- see `_prime_estop_publisher`. Discovery happens once, when
        the window opens, so the press that matters goes out on a publisher
        that is already matched.

        HONESTLY, ABOUT THE EVIDENCE. This was changed while chasing an audit
        failure that read "e-stop reaches /estop: received []", and that
        failure turned out to be the AUDIT's own subscriber, which was never
        being spun -- a second `rclpy.spin` on the global executor raises in
        its thread and nothing was watching. So there is no measurement here
        of the old code dropping an e-stop. The change stands on the two
        properties above, which are true of the code as written; it does not
        stand on an observed failure, and this paragraph is here so nobody
        later quotes one.

        TRANSIENT_LOCAL with depth 1: a node that subscribes AFTER a stop was
        commanded still learns the system is stopped. A transient-local
        publisher is compatible with the volatile subscribers `estop_node`
        and the guards already use, so nothing downstream changes.
        """
        key = (topic, msg_type.__name__)
        pub = self._pubs.get(key)
        if pub is None:
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                                   QoSProfile, ReliabilityPolicy)
            qos = QoSProfile(depth=1,
                             history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pub = self.create_publisher(msg_type, topic, qos)
            self._pubs[key] = pub
        m = msg_type()
        m.data = value
        pub.publish(m)

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
        self.specs = None               # the launch manifest, built once
        self.buttons = {}
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
        # THE LEFT COLUMN IS MEASURED, NOT 376.
        #
        # 376 was a number that fitted the buttons at the font and DPI of the
        # day it was chosen, and on this display it clips them. Ask the
        # column what it needs instead; `_left_width()` takes the widest
        # sizeHint in it and is the single source both here and after
        # embedding, which is the other place the number was written out.
        lw = self._left_width()
        self.split.setSizes([lw, 620, max(420, 1920 - lw - 620)])

        outer.addLayout(self._bottom_bar())
        self.setCentralWidget(root)
        self.resize(1920, 1060)

        self._ft = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        # EVERY READOUT IS SELECTABLE. A QLabel is not, by default, and this
        # window is almost entirely QLabels: the parse result, what the camera
        # saw, the clearance numbers, the divergence rows, every refusal
        # reason. None of it could be copied into a note, a message or a bug
        # report -- it had to be retyped from the screen or screenshotted.
        #
        # Done here rather than at each construction site because there are
        # ~200 of them and the next one added would be missed.
        self._make_text_selectable()
        QTimer.singleShot(400, self.start_rviz)
        # And again after RViz embeds, because the panels rebuilt around it.
        QTimer.singleShot(3000, self._make_text_selectable)
        # AFTER the first layout pass, when viewports are real. Before it,
        # every width is a hint.
        QTimer.singleShot(250, self._fit_left_column)
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

    def clipped_pages(self):
        """Every scroll page in the control column whose content does not
        fit its viewport, as (name, viewport_px, needed_px).

        Horizontal scrolling is OFF in that column, so a page wider than its
        viewport is not scrollable -- it is CUT, silently, with the right-hand
        end of every sentence and button label missing. Exposed as a method
        rather than kept private so `verify_gui_buttons` can fail on it: a
        layout defect that can only be seen by looking at a screenshot gets
        looked at once.
        """
        out = []
        col = getattr(self, "_left_col", None)
        if col is None:
            return out
        for i, sa in enumerate(col.findChildren(QScrollArea)):
            inner = sa.widget()
            if inner is None:
                continue
            need = inner.minimumSizeHint().width()
            vp = sa.viewport().width()
            if need > vp:
                out.append(("scroll page %d" % i, vp, need))
        return out

    def _fit_left_column(self, tries=6):
        """Widen the control column until nothing in it is clipped.

        MEASURED, NOT CALCULATED. Three attempts at a formula for this width
        all missed -- by 20 px, then 25, then 4 -- because it depends on
        frame widths, layout margins and a scrollbar that only appears when
        needed, and each attempt modelled a different subset. Asking the
        laid-out widget how much is missing and adding exactly that cannot be
        off by a margin nobody thought of.

        ONE ADJUSTMENT PER CALL, AND IT RE-ARMS ITSELF. It used to loop with
        `QApplication.processEvents()` between passes, and that re-enters the
        Qt event loop from inside a timer callback. Run 250 ms after RViz is
        reparented, the re-entry broke the adoption: the container had the
        foreign window, and the foreign window ended up at +0+0 at the corner
        of the SCREEN instead of inside the panel -- so the COMMANDED view
        was blank while the log said "embedded ... viewable". Verified from
        the X tree: the GUI window had no child at all.

        A QTimer instead. Same convergence, no re-entrancy, and the layout
        has had a real event-loop pass to settle before the next measurement.
        """
        short = max((n - v for _, v, n in self.clipped_pages()), default=0)
        if short <= 0 or tries <= 0:
            return True
        sizes = self.split.sizes()
        if len(sizes) < 3:
            return False
        new_left = min(620, sizes[0] + short + 2)
        if new_left <= sizes[0]:
            return False                # at the ceiling; report, do not loop
        rest = sizes[1] + sizes[2] - (new_left - sizes[0])
        self.split.setSizes([new_left, sizes[1], max(300, rest - sizes[1])])
        QTimer.singleShot(60, lambda: self._fit_left_column(tries - 1))
        return False

    def focus_instruct(self):
        """Put the keyboard in the instruction box, and prove it went there.

        `activateWindow()` asks the window manager for the input focus, which
        matters when an embedded RViz currently holds it; `setFocus()` then
        places it on the line edit inside this window. Both are needed -- the
        second alone moves Qt's idea of focus while X keeps sending keys
        elsewhere.
        """
        e = getattr(self, "inst_edit", None)
        if e is None:
            return False
        self.activateWindow()
        self.raise_()
        e.setFocus(Qt.OtherFocusReason)
        got = QApplication.focusWidget() is e
        if not got:
            self.bus.note("could not put the keyboard in the instruction "
                          "box; click it once. If an embedded RViz has the "
                          "focus, clicking this window's title bar takes it "
                          "back.", bad=True)
        return got

    def _make_text_selectable(self):
        """Let the operator select and copy any text in the window.

        `Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard` on every
        QLabel. The keyboard flag is what makes Ctrl+A / Ctrl+C work once a
        label has focus; the mouse flag is what lets a drag select.

        NOT applied to QLabels that are acting as image sinks -- the camera
        views -- because giving those a text cursor makes them look
        interactive when there is nothing to select.
        """
        n = 0
        for lab in self.findChildren(QLabel):
            if lab.pixmap() is not None:
                continue
            lab.setTextInteractionFlags(Qt.TextSelectableByMouse
                                        | Qt.TextSelectableByKeyboard)
            n += 1
        return n

    def _left_width(self):
        """How wide the control column has to be for nothing to be clipped.

        ONE SOURCE. The width was written out twice -- in the constructor and
        again after RViz embeds, which is the moment the splitter is
        re-asserted -- so a fix in one place was silently undone by the
        other. The comment above the second one even says so.

        Measured from the widgets: the largest of the column's own minimum,
        its layout's minimum, and every direct child's size hint. Bounded, so
        one runaway label cannot eat the RViz panel.
        """
        col = getattr(self, "_left_col", None)
        if col is None:
            return 376
        need = 0
        # ASK QT, DO NOT RE-DERIVE. Each activity tab is a QScrollArea with
        # horizontal scrolling OFF, so anything wider than its viewport is
        # silently cut with no way to reach it. The authoritative number is
        # the one Qt already computes: the inner widget's minimumSizeHint,
        # plus the vertical scrollbar that eats into the viewport, plus the
        # frame.
        #
        # Measured offscreen on 2026-08-22, which is how the previous two
        # attempts were found wanting: the RUN page's inner widget needs
        # 378 px and its viewport was 358. Twenty pixels, and they were the
        # right-hand end of every sentence in the panel.
        #
        # Measuring BUTTONS instead (attempt two) gave 323 and clamped to the
        # old 376, because the widest thing in the page is a layout of
        # several widgets, not one button. Measuring the widget SIZE HINT
        # (attempt one) gave a huge number, because a word-wrapped label
        # reports its unwrapped width.
        for sa in col.findChildren(QScrollArea):
            inner = sa.widget()
            if inner is None:
                continue
            bar = sa.verticalScrollBar().sizeHint().width()
            need = max(need, inner.minimumSizeHint().width() + bar
                       + 2 * sa.frameWidth())
        m = col.contentsMargins()
        need += m.left() + m.right()
        # Clamped: the floor is the width that was there before, and the
        # ceiling stops one runaway label eating the RViz panel.
        return int(max(376, min(need, 620)))

    def _left_column(self):
        # ONE SCROLL LEVEL, NOT TWO. The column was a QScrollArea holding
        # everything; the three activity tabs inside it scroll on their own,
        # and a scroll area inside a scroll area gives two scrollbars, two
        # sets of margins and a content width narrow enough to slice every
        # button label down the middle -- measured from a screenshot: "release
        # RIGHT g", "PRECISION ... SP".
        host = QWidget()
        host.setMinimumWidth(362)
        self._left_col = host
        # NO HARD MAXIMUM. This was `setMaximumWidth(392)`, which capped the
        # column below what its own content needs (378 px of content in a
        # 374 px viewport, measured offscreen 2026-08-22) and made every
        # attempt to widen it through the splitter a no-op -- the splitter
        # said 428 and the viewport stayed at 374. Everything in the column
        # was then cut at the right-hand edge: sentences lost their last
        # words, "HALT BOTH ARMS" rendered as "HALT BOTH ARM".
        #
        # The ceiling that the cap was protecting -- the RViz panel must not
        # be squeezed out -- is enforced where it belongs, in `_left_width`
        # and `_fit_left_column`, both of which stop at 620 and neither of
        # which can be defeated by a widget asking for more.
        host.setMaximumWidth(620)
        col = QVBoxLayout(host)
        col.setContentsMargins(0, 0, 0, 0)

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
            # WRAPPED, NOT CLIPPED. Centred in a 140 px label the caption lost
            # BOTH ends: "AMERA -- no frame has ever a". A camera panel whose
            # caption cannot be read is a camera panel with no caption, and
            # the caption is the only thing separating "live" from "stale".
            cap.setWordWrap(True)
            c.addWidget(nm)
            c.addWidget(img)
            c.addWidget(cap)
            cl.addLayout(c)
            self.cam_lbl[a], self.cam_cap[a] = img, cap
        col.addWidget(campanel)

        # GROUPED BY WHAT YOU ARE DOING, NOT BY WHAT THE SYSTEM CONTAINS.
        #
        # Everything here was previously one long column: connect-the-arms
        # controls, per-run controls and read-only status stacked together, so
        # the two sliders you set once at the start of a session sat between
        # the buttons you press during it. Three tabs, and the rule for which
        # tab something goes in is the question it answers:
        #
        #   SET UP   things you do BEFORE a run and then stop touching
        #   RUN      what you press and watch WHILE something is running
        #   STATUS   read-only. Nothing in here changes the robot.
        #
        # Nothing is removed and no widget is renamed -- `verify_gui_buttons`
        # walks this window and presses everything it finds, and the recording
        # sweep calls Gui.on_launch directly, so both keep working.
        acttabs = QTabWidget()
        acttabs.setFont(helvetica(10))
        self.act_tabs = acttabs

        setup = QWidget()
        sv = QVBoxLayout(setup)
        sv.setContentsMargins(2, 4, 2, 2)
        sv.addWidget(self._connect_panel())
        sv.addWidget(self._setup_controls())
        sv.addWidget(self._motion_panel())
        sv.addWidget(self._diag_launchers())
        sv.addStretch(1)
        acttabs.addTab(self._scroll(setup), "SET UP")

        run = QWidget()
        rv = QVBoxLayout(run)
        rv.setContentsMargins(2, 4, 2, 2)
        # MODES FIRST, THEN THE ONE BUTTON.
        #
        # `START VR TELEOP` used to be first on the argument that it is the
        # only control a VR session needs. True, and it made the window look
        # like a VR-only tool: everything else was below the fold, and the
        # first question anybody asked of it was "where is autonomy". The
        # mode panel is four rows and answers that before anything is
        # scrolled; the VR button is immediately under it and has lost
        # nothing.
        # MAP FIRST, because that is the order of the work: measure what is
        # on the table, then plan against it. It also has to be ABOVE THE
        # FOLD -- this window has already had one defect where every mode
        # button was below it and the first question anybody asked was where
        # they had gone. Screenshotted after moving, not assumed.
        rv.addWidget(self._map_panel())
        rv.addWidget(self._modes_panel())
        rv.addWidget(self._experiments_panel())
        rv.addWidget(self._vr_panel())
        rv.addWidget(self._real_panel())
        rv.addWidget(self._vision_panel())
        rv.addWidget(self._controls())
        rv.addWidget(self._launchers())
        rv.addStretch(1)
        acttabs.addTab(self._scroll(run), "RUN")

        status = QWidget()
        stv = QVBoxLayout(status)
        stv.setContentsMargins(2, 4, 2, 2)
        self.ind = {}
        for title, keys in (
            ("Master", ("capability_left", "capability_right", "channels",
                        "clutch")),
            ("Scene", ("fingerprint", "detector_left", "detector_right")),
            ("Mode", ("mode", "autonomy", "intent")),
            ("Target", ("gripper", "clearance_left", "clearance_right")),
            ("Safety", ("estop", "blockers", "ik_left", "ik_right")),
            # WHAT IS ACTUALLY GENERATING THE MOTION, read from the
            # follower's own status topic and never from which item is
            # selected in SET UP. A follower launched before the selection
            # changed, or launched from a terminal, or one where `auto` fell
            # back because ruckig would not import, all read correctly here
            # and would all read wrongly from the combo box.
            ("Motion", ("motion_left", "motion_right")),
        ):
            g = QGroupBox(title)
            g.setFont(helvetica(11, True))
            gl = QGridLayout(g)
            for i, k in enumerate(keys):
                w = Ind(k.replace("_", " "))
                self.ind[k] = w
                gl.addWidget(w, i // 2, i % 2)
            stv.addWidget(g)
        stv.addStretch(1)
        acttabs.addTab(self._scroll(status), "STATUS")
        acttabs.setCurrentIndex(1)              # RUN, which is the usual case

        col.addWidget(acttabs, 1)
        return host

    @staticmethod
    def _scroll(widget):
        """A vertical scroll area. Horizontal scrolling is OFF on purpose.

        DO NOT set a minimum width from `widget.sizeHint()`. A word-wrapped
        QLabel reports its FULL UNWRAPPED width as its size hint, so that
        minimum stops every label in the column from wrapping and cuts them
        instead -- tried on 2026-08-22 and visibly worse than the problem it
        was aimed at. `setWidgetResizable(True)` already makes the content
        follow the viewport, which is what makes wrapping work.

        The widgets that genuinely cannot fit are the ones that cannot wrap:
        buttons. Those are handled by `_left_width()`, which measures them.
        """
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sa.setStyleSheet("border:none")
        sa.setWidget(widget)
        return sa

    def _real_panel(self):
        """THE REAL ARMS, AND THE ORDER IS THE SAFETY CASE.

        `START VR TELEOP` deliberately cannot reach a real arm -- that is a
        property it is tested for. So the real-arm path is a SEPARATE panel
        with a separate, explicit act, and the buttons are laid out in the
        order they must be pressed rather than the order they were written.

        Every button here is a thin wrapper over the same command a person
        would type. Nothing is reimplemented in the GUI, so the GUI cannot
        drift away from the procedure that was tested at the terminal.
        """
        g = QGroupBox("Real arms")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        # THE E-STOP IS FIRST AND BIGGEST, and it is not at the bottom of the
        # panel. A stop control you have to scroll to is not a stop control.
        self.real_estop_btn = QPushButton("E-STOP  --  HALT BOTH ARMS")
        self.real_estop_btn.setFont(helvetica(13, True))
        self.real_estop_btn.setMinimumHeight(46)
        self.real_estop_btn.setStyleSheet(
            "color:%s;border:2px solid %s" % (C_BAD, C_BAD))
        self.real_estop_btn.setToolTip(
            "Halts both arms NOW and LATCHES. Proven on real hardware "
            "2026-08-21: a moving arm stopped dead, 0.000 deg of travel "
            "after the trip. 'reset e-stop' is the only way out.")
        self.real_estop_btn.clicked.connect(self.on_real_estop)
        v.addWidget(self.real_estop_btn)

        self.real_head = QLabel("arms not started")
        self.real_head.setFont(helvetica(10, True))
        self.real_head.setWordWrap(True)
        self.real_head.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.real_head)

        for text, slot, tip, big in (
            ("1.  START REAL ARMS",
             self.on_real_start,
             "Opens ONE Kortex session per arm and homes both to the pose in "
             "config/home_positions_*.txt. The arm permits exactly one "
             "session, so never start this twice.", True),
            ("2.  HOME BOTH ARMS",
             self.on_real_home,
             "Drives both arms to the config home under the velocity law, "
             "with the wearer clearance floor live. Safe to press again; a "
             "homed arm simply reports it is already there.", False),
            ("3.  OBSERVER STATION",
             self.on_real_observer,
             "Opens the observer's e-stop station. It is a 5 Hz HEARTBEAT: "
             "two seconds of silence reads as the observer having WITHDRAWN, "
             "which is the state the interlock exists to catch.", False),
            ("4.  START REAL ARM TELEOP",
             self.on_real_teleop,
             "Hands the VR mapper the real arms. Refuses unless the observer "
             "is present or the 'working alone' box above is ticked. This is "
             "the only control in this window that lets VR move real metal.",
             True),
        ):
            b = QPushButton(text)
            b.setFont(helvetica(12 if big else 10, True))
            if big:
                b.setMinimumHeight(38)
                b.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
            b.setToolTip(tip)
            b.clicked.connect(slot)
            v.addWidget(b)

        row = QHBoxLayout()
        for text, slot, tip in (
            ("reset e-stop", self.on_real_estop_reset,
             "Clears the latch. Deliberate act -- nothing else clears it."),
            ("stop real arms", self.on_real_stop,
             "SIGINT, never SIGKILL: the handler zeroes speeds, calls Stop() "
             "and closes the session. Killed harder, the session leaks and "
             "the next run cannot connect."),
            ("arm status", self.on_real_status,
             "Network, session, joint feed and distance from home, per arm."),
        ):
            b = QPushButton(text)
            b.setFont(helvetica(9))
            b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)

        self.real_note = QLabel("")
        self.real_note.setFont(helvetica(9))
        self.real_note.setWordWrap(True)
        self.real_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.real_note)
        return g

    # ------------------------------------------------------ real-arm slots
    #
    # EVERY ONE OF THESE IS WRAPPED. PyQt SWALLOWS EXCEPTIONS RAISED INSIDE A
    # SLOT: the button appears to work, nothing happens, and nothing is
    # logged. That exact failure hid five dead buttons in this window until
    # the button audit found them, so a bare slot here is a defect waiting to
    # repeat. `bus.note` is the log -- there is no `self.log`.
    def _real_say(self, text, bad=False):
        try:
            self.real_note.setText(text)
            self.real_note.setStyleSheet("color:%s" % (C_BAD if bad else C_MUTED))
            self.bus.note(text, bad=bad)
        except Exception:                                     # noqa: BLE001
            pass

    def _real_guard(self, label, fn):
        try:
            fn()
        except Exception as e:                                # noqa: BLE001
            self._real_say("%s FAILED: %r" % (label, e), bad=True)

    def _svc(self, name, typ="std_srvs/srv/Trigger"):
        """Call a Trigger service IN PROCESS, and report what happened.

        THIS USED TO SHELL OUT to `ros2 service call`, and that is how the
        biggest, reddest button in this window came to lie.
        `ros2 service call` on a service that does not exist **does not
        exit** -- it waits for the service, for ever. Measured 2026-08-22: 12
        seconds and still running. `_run_raw`'s 2.5 s check looks at the
        return code, sees `None` because the process is alive, and therefore
        says NOTHING.

        So pressing `E-STOP -- HALT BOTH ARMS` with no stack up launched a
        process that hung, and the panel said "E-STOP SENT -- latched until
        reset" in red. Nothing anywhere contradicted it. The same applied to
        every other service button: HOME BOTH ARMS, stop real arms, reset
        e-stop.

        `bus.call_trigger` is the path that was already correct and already
        used by the bottom-bar reset: it checks `service_is_ready()` first --
        never `wait_for_service`, which is the 4 s e-stop stall -- refuses by
        name when the service is absent, and reports the real response.
        """
        self.bus.submit(lambda: self.bus.call_trigger(name),
                        label="calling %s" % name)

    def on_real_start(self):
        self._real_guard("start real arms", lambda: (
            self._run_raw("start_real.sh",
                          ["bash", os.path.join(_WS, "scripts/start_real.sh"),
                           "arm:=both"]),
            self.real_head.setText("starting both arms -- watch this window"),
            self._real_say(
                "Opening one Kortex session per arm, then homing. Two arms "
                "share one link, so the script drops the command rate to "
                "12 Hz per arm on purpose.")))

    def on_real_home(self):
        self._real_guard("home", lambda: (
            self._svc("/home_arm_left"), self._svc("/home_arm_right"),
            self._real_say("homing both arms to the config pose")))

    def on_real_observer(self):
        self._real_guard("observer", lambda: (
            self._run_raw("observer_estop",
                          ["x-terminal-emulator", "-e", "python3",
                           os.path.join(_WS, "scripts/observer_estop.py")]),
            self._real_say("observer station opened -- it must stay running; "
                           "2 s of silence reads as withdrawal")))

    def on_real_teleop(self):
        self._real_guard("real arm teleop", lambda: (
            self._svc("/vr/enable_real_arm"),
            self._real_say(
                "Asked for real-arm control. REFUSED means the observer is "
                "not present and the 'working alone' box is not ticked.")))

    def on_real_estop(self):
        """Halt both arms. PUBLISH first, then call the service.

        `/estop` is BOTH a topic and a service -- `estop_node` creates a Bool
        subscription and a Trigger service of the same name, and so do
        `mount_guard_node`, `participant_safety_node` and `fault_injector` on
        the topic side. The TOPIC is the robust path: a publish reaches every
        subscriber that exists and costs nothing when none do, whereas the
        service exists only while `estop_node` is up.

        This button used to call the service ONLY, through a shell-out that
        hung silently when the service was absent, and then reported "E-STOP
        SENT" regardless. It now does the thing that works first, and says
        what it actually did rather than what it intended.
        """
        def go():
            self.bus.publish_once(Bool, "/estop", True)
            self.bus.note("E-STOP published to /estop")
            self.bus.call_trigger("/estop")
        self._real_guard("e-stop", lambda: (
            self.bus.submit(go, label="E-STOP"),
            self._real_say("E-STOP published to /estop. Watch the event log "
                           "for the service result; the arms are latched "
                           "until reset.", bad=True)))

    def on_real_estop_reset(self):
        self._real_guard("reset", lambda: (
            self._svc("/estop_reset"),
            self._real_say("e-stop reset requested")))

    def on_real_stop(self):
        self._real_guard("stop", lambda: (
            self._svc("/home_abort_left"), self._svc("/home_abort_right"),
            self._real_say(
                "Homing aborted; arms stop where they stand. Close the "
                "session from the terminal running start_real.sh with "
                "Ctrl-C -- SIGINT, so the session closes cleanly.")))

    def on_real_status(self):
        """Ping both arms and look for their sessions -- OFF the Qt thread.

        This ran `go()` directly on the Qt thread through `_real_guard`, and
        `go()` makes two `ping -c 1 -W 1` calls and two `pgrep` calls. With
        both arms unreachable that is two full second-long timeouts plus
        process spawns, during which the window is FROZEN -- including the
        e-stop, which is on the same panel.
        
        Found by the button audit's own slow-press check (5 s budget), which
        had never fired before because the machine was usually idle enough
        to squeak under it. A control that freezes the window only when the
        machine is busy is worse than one that always does: it works in
        testing and stops working on a lab day.
        """
        def go():
            bits = []
            for a in ("left", "right"):
                ip = {"left": "192.168.1.10", "right": "192.168.1.9"}[a]
                up = subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                                    stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL).returncode == 0
                sess = subprocess.run(
                    ["pgrep", "-f", "kortex_highlevel_bridge.*arm:=%s" % a],
                    stdout=subprocess.DEVNULL).returncode == 0
                bits.append("%s: net %s, session %s"
                            % (a.upper(), "UP" if up else "DOWN",
                               "up" if sess else "down"))
            self._arm_status_text = "   |   ".join(bits)
            self._real_say("arm status refreshed")

        self.real_head.setText("checking both arms (up to 2 s)...")
        self._arm_status_text = None
        self.bus.submit(go, label="arm status")
        QTimer.singleShot(300, self._arm_status_render)

    def _arm_status_render(self, tries=30):
        txt = getattr(self, "_arm_status_text", None)
        if txt is None:
            if tries > 0:
                QTimer.singleShot(300,
                                  lambda: self._arm_status_render(tries - 1))
            else:
                self.real_head.setText("the arm status check did not finish "
                                       "-- see the event log")
            return
        self.real_head.setText(txt)
        self._arm_status_text = None

    def _vision_panel(self):
        """SAY WHAT YOU WANT PICKED UP. Available in EVERY mode, not just VR.

        The Instruct tab drives mode 06 from a sentence through a grammar.
        This is the other half: a sentence naming an OBJECT, resolved against
        what the cameras can actually see, with the grasp and its
        reachability shown BEFORE anything moves.

        The panel states which detection backend is live. That is not
        decoration: the colour backend knows eleven colour words and nothing
        else, and a detection that does not say how it was made invites the
        reader to assume the open-vocabulary model was running when it was
        not.
        """
        g = QGroupBox("Vision: say what to pick up")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        # WHAT IS ON THE TABLE, before naming anything.
        #
        # Until 2026-08-22 this panel could only answer "where is the thing I
        # named". Asked what was on the surface at all, the system had no
        # answer -- so a wrong name produced a move to somewhere plausible
        # and empty, which is what "it just moves here and there" looks like
        # from the outside.
        b = QPushButton("WHAT IS ON THE TABLE?")
        b.setFont(helvetica(10, True))
        b.setMinimumHeight(28)
        b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        b.setToolTip(
            "Segments the support surface from the wrist camera's depth, "
            "then every cluster standing on it, then a plane-constrained "
            "grasp for each -- and reports the ones it CANNOT pick up with "
            "the reason. Needs no camera calibration: the wrist camera's "
            "pose is forward kinematics.")
        b.clicked.connect(self.on_what_is_on_the_table)
        v.addWidget(b)
        self.table_lbl = QLabel("not looked yet")
        self.table_lbl.setWordWrap(True)
        self.table_lbl.setFont(mono(8))
        self.table_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.table_lbl)

        row = QHBoxLayout()
        self.vis_prompt = QLineEdit()
        self.vis_prompt.setFont(helvetica(11))
        self.vis_prompt.setPlaceholderText(
            "e.g. the green cube   /   pick up the red block")
        self.vis_prompt.returnPressed.connect(self.on_vision_find)
        row.addWidget(self.vis_prompt, 1)
        b = QPushButton("FIND")
        b.setFont(helvetica(11, True))
        b.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
        b.setToolTip("Looks with the selected camera, detects what you named, "
                     "builds a grasp and checks whether the arm can reach it. "
                     "Moves nothing.")
        b.clicked.connect(self.on_vision_find)
        row.addWidget(b)
        v.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("camera"))
        self.vis_cam = QComboBox()
        self.vis_cam.addItems(["gripper (RGB-D)", "scene (colour only)"])
        self.vis_cam.setToolTip(
            "The gripper camera has DEPTH, so it gives a position. The scene "
            "camera is colour only -- it can say what it sees and where in "
            "the image, but not how far away, so it cannot produce a grasp.")
        self.vis_cam.currentTextChanged.connect(
            lambda t: self.bus.note("vision camera: %s" % t))
        row2.addWidget(self.vis_cam, 1)
        row2.addWidget(QLabel("arm"))
        self.vis_arm = QComboBox()
        self.vis_arm.addItems(["left", "right"])
        self.vis_arm.currentTextChanged.connect(
            lambda t: self.bus.note("vision arm: %s" % t))
        row2.addWidget(self.vis_arm)
        v.addLayout(row2)

        self.vis_backend = QLabel("")
        self.vis_backend.setFont(helvetica(9))
        self.vis_backend.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vis_backend)

        self.vis_result = QLabel("nothing looked for yet")
        self.vis_result.setFont(helvetica(10))
        self.vis_result.setWordWrap(True)
        self.vis_result.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vis_result)

        self.vis_plan = QLabel("")
        self.vis_plan.setFont(helvetica(9))
        self.vis_plan.setWordWrap(True)
        self.vis_plan.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vis_plan)

        QTimer.singleShot(600, self._vision_backend_note)
        return g

    def _vision_backend_note(self):
        try:
            from srl_perception.prompt_detector import PromptDetector
            av = PromptDetector().available()
            if av.get("yoloworld"):
                txt = ("backend: YOLO-World (open vocabulary -- name any "
                       "object)")
            else:
                txt = ("backend: colour+depth (knows colour words only; "
                       "install ultralytics for open vocabulary)")
            self.vis_backend.setText(txt)
        except Exception as e:                                # noqa: BLE001
            self.vis_backend.setText("vision module not importable: %r" % e)

    def on_vision_find(self):
        """Look, detect, plan, and say which stage failed if one did."""
        try:
            self._vision_find()
        except Exception as e:                                # noqa: BLE001
            self.vis_result.setText("vision failed: %r" % e)
            self.vis_result.setStyleSheet("color:%s" % C_BAD)
            self.bus.note("vision failed: %r" % e, bad=True)

    def _vision_find(self):
        import numpy as np
        from srl_perception import srl_cameras as CAMS
        from srl_perception.grasp_pipeline import PlanFailure, plan_grasp
        from srl_perception.prompt_detector import PromptDetector

        prompt = self.vis_prompt.text().strip()
        if not prompt:
            # EVERY PATH LOGS, including this one. The button audit requires a
            # trace from every control it presses, and it is right to: a
            # button that sets a label and logs nothing is indistinguishable
            # from a button that did nothing, which is the exact defect that
            # hid five dead controls in this window.
            self.vis_result.setText("type what to look for first")
            self.bus.note("vision: no prompt typed -- nothing to look for")
            return
        arm = self.vis_arm.currentText()
        use_grip = self.vis_cam.currentIndex() == 0
        self.vis_result.setText("looking ...")
        self.vis_result.setStyleSheet("color:%s" % C_MUTED)
        QApplication.processEvents()

        det = PromptDetector(backend="auto", far_m=1.8)
        if not use_grip:
            # SCENE CAMERA HAS NO DEPTH, so it CANNOT produce a grasp. Saying
            # so is the point: a panel that quietly returned a 2D hit here
            # would look like it had localised the object.
            cam = CAMS.SceneCamera().open()
            try:
                bgr = cam.read()
            finally:
                cam.close()
            hits = det.detect(bgr, prompt)
            if not hits:
                self.vis_result.setText("scene camera: nothing matching %r"
                                        % prompt)
                self.bus.note("vision: scene camera found nothing matching %r"
                              % prompt)
                return
            self.vis_result.setText(
                "scene camera sees %d match(es); best %s at pixel "
                "(%.0f, %.0f)" % (len(hits), hits[0].label,
                                  hits[0].centre_uv[0], hits[0].centre_uv[1]))
            self.vis_plan.setText(
                "No grasp: the scene camera has no depth, so this is a "
                "direction and not a position. Switch to the gripper camera "
                "to get a graspable pose.")
            self.bus.note("vision: scene camera matched %d, no depth so no "
                          "grasp" % len(hits))
            return

        ip = {"left": "192.168.1.10", "right": "192.168.1.9"}[arm]
        cam = CAMS.GripperCamera(ip)
        bgr = cam.read_colour()
        depth = cam.read_depth()
        cam.close()
        cam_p, cam_R = self._camera_pose(arm)
        if cam_p is None:
            self.vis_result.setText(
                "no joint states for the %s arm, so the camera's pose is "
                "unknown and any 3D answer would be meaningless" % arm)
            self.bus.note("vision: no %s joint states, camera pose unknown"
                          % arm, bad=True)
            return
        try:
            plan = plan_grasp(bgr, depth, prompt, cam_p, cam_R,
                              CAMS.KINOVA_COLOR_K, CAMS.KINOVA_DEPTH_K,
                              detector=det)
        except PlanFailure as pf:
            self.vis_result.setText("REFUSED at the %s stage" % pf.stage)
            self.vis_result.setStyleSheet("color:%s" % C_BAD)
            self.vis_plan.setText(pf.reason)
            self.bus.note("vision refused (%s): %s" % (pf.stage, pf.reason),
                          bad=True)
            return
        c = plan["centre"]
        self.vis_result.setText(
            "FOUND %s at x=%+.3f y=%+.3f z=%+.3f m  (%.0f mm wide, %d depth "
            "points, %s)" % (plan["detection"]["label"], c[0], c[1], c[2],
                             plan["width_m"] * 1000, plan["n_points"],
                             plan["backend"]))
        self.vis_result.setStyleSheet("color:%s" % C_OK)
        ok, why = self._vision_reach(arm, plan)
        self.vis_plan.setText(
            "pregrasp %s   |   reach: %s"
            % (np.round(plan["pregrasp"], 3), why))
        self.bus.note("vision found %s at %s" % (plan["detection"]["label"],
                                                 np.round(c, 3)))

    def _camera_pose(self, arm):
        """Where the wrist camera is, from the arm's own joint states."""
        import numpy as np
        try:
            import sys
            sys.path.insert(0, os.path.join(_WS, "scripts"))
            import solve_home_pose as SHP
            # THE REAL ARM'S OWN JOINTS, from divergence.Side.pos -- the same
            # stream the ACTUAL panel draws. Not the sim's: the camera pose
            # has to be where the metal is, or a 3D answer computed from it
            # describes a robot that is not in the room.
            js = self.real[arm].pos
            names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
            if not all(n in js for n in names):
                return None, None
            q = np.array([js[n] for n in names], float)
            M = SHP.Scorer().cf[arm](q)
            T = M[SHP.IDX["camera_link"]]
            return T[:3, 3], T[:3, :3]
        except Exception:                                     # noqa: BLE001
            return None, None

    def _vision_reach(self, arm, plan):
        try:
            import sys
            import numpy as np
            sys.path.insert(0, os.path.join(_WS, "scripts"))
            sys.path.insert(0, os.path.join(_WS, "config"))
            import solve_home_pose as SHP
            import home_positions as hp
            from srl_perception.grasp_pipeline import reachable
            sc = SHP.Scorer()
            home = np.array(hp.load_home_radians(arm), float)
            ok, _q, why = reachable(sc, arm, plan["centre"], home, restarts=25)
            return ok, why
        except Exception as e:                                # noqa: BLE001
            return False, "reachability check unavailable: %r" % e


    # =================================================================
    # OPERATE -- every way the arms can be driven, in one place
    # =================================================================
    #
    # WHY THIS PANEL EXISTS. Until 2026-08-22 the RUN tab opened on
    # `START VR TELEOP` and the mode launchers were the FIFTH group down,
    # under the VR panel, the real-arm panel, the vision panel and the
    # running controls. Everything was reachable and nothing was visible:
    # asked what the window could do, the honest answer from looking at it
    # was "VR teleoperation". Shared autonomy and full autonomy were behind
    # a scroll, and full autonomy's prompt -- the whole point of mode 06 --
    # was a tab in a different column.
    #
    # The rule here is the one `docs/NEXT_SESSION_2026_08_22.md` asks for:
    # ONE PANEL PER MODE, each saying the same four things -- what it will
    # command, what it refuses and why, where it is live, and how to stop it.
    # Nothing below is a new capability. Every button runs a Spec that
    # already existed in `gui_launch_specs`; what changed is that you can
    # see them.
    # (key, title, [stack specs, in order], what it is, the caveat)
    #
    # THE SECOND FIELD IS A SEQUENCE, and that is the whole point of this
    # rewrite. A mode is not one launch: VR is the sim stack AND the VR
    # transport on top of it, and "with the real arms" is any of those
    # followed by the cascade. Those were four separate buttons in a grid
    # five panels down, in an order the operator had to know.
    MODE_ROWS = [
        ("teleop", "1  MASTER TELEOP", ["sim"],
         "The instrumented arm drives the robot. Mode 01.",
         "Needs the Teensy. 7 of 14 master channels were INCOHERENT at the "
         "last channel check, so this is the mode most likely to refuse."),
        ("vr", "2  VR / DESK TELEOP", ["sim", "vr"],
         "Controllers as 6-DOF motion capture. Mode 02. Nobody wears the "
         "headset -- it stands on a shelf as the tracking reference.",
         "START VR TELEOP below does the same thing with all twelve "
         "bring-up steps checked, and is the better button for a session."),
        ("shared", "3  SHARED AUTONOMY", ["autonomy"],
         "Perception, grasp generation and the arbiter, on top of teleop. "
         "Modes 03 and 04.",
         "Starts a stack. Refuses if one is already running -- HARD "
         "CONSTRAINT 3."),
        ("full", "4  FULL AUTONOMY", ["autonomy"],
         "Mode 06. Say it in a sentence; the parser shows you what it "
         "understood and what the camera saw BEFORE anything moves.",
         "Same stack as shared autonomy. Use TYPE WHAT YOU WANT to drive "
         "it; nothing is commanded until you press CONFIRM there."),
    ]

    # What "+ REAL" adds, after the stack is up.
    REAL_STEP = {"real": ("SIM + REAL", "real",
                          "Cascades to the physical arms. Opens one Kortex "
                          "session per arm and refuses unless homing "
                          "succeeds."),
                 "mock": ("SIM + MOCK", "real_mock",
                          "The identical sequence against "
                          "mock_real.launch.py. Nothing physical moves. "
                          "Rehearse here first.")}

    def _modes_panel(self):
        g = QGroupBox("OPERATE  --  how the arms are driven")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        v.setSpacing(3)

        intro = QLabel(
            "Pick a mode, then whether it drives the SIMULATION only or "
            "cascades to the REAL arms. They are not layers you stack: one "
            "row at a time.")
        intro.setWordWrap(True)
        intro.setStyleSheet("color:%s" % C_MUTED)
        intro.setFont(helvetica(9))
        v.addWidget(intro)

        specs = {sp.key: sp for sp in self._ensure_specs()}
        self.mode_btn = {}
        self.mode_note = {}
        for key, label, chain, what, caveat in self.MODE_ROWS:
            head = QLabel(label)
            head.setFont(helvetica(12, True))
            head.setAlignment(Qt.AlignCenter)
            v.addWidget(head)

            note = QLabel(what)
            note.setWordWrap(True)
            note.setFont(helvetica(9))
            note.setStyleSheet("color:%s" % C_TEXT)
            v.addWidget(note)

            row = QHBoxLayout()
            row.setSpacing(4)
            for tag, btxt, extra in (
                    ("sim", "SIM ONLY", None),
                    ("mock", self.REAL_STEP["mock"][0],
                     self.REAL_STEP["mock"][1]),
                    ("real", self.REAL_STEP["real"][0],
                     self.REAL_STEP["real"][1])):
                seq = list(chain) + ([extra] if extra else [])
                b = QPushButton(btxt)
                b.setFont(helvetica(9, True))
                b.setMinimumHeight(26)
                b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
                # A BUTTON MUST LOOK PRESSABLE. On this dark theme a bare
                # QPushButton renders as flat text and is indistinguishable
                # from the labels around it -- which is half the reason the
                # window read as "VR only": the other modes were there and
                # did not look like controls. Real is warned in the bad
                # colour, because it is the one that moves metal.
                edge, txt, hover = ((C_BAD, C_BAD, C_BAD) if tag == "real"
                                    else (LINE, C_TEXT, C_OK))
                b.setStyleSheet(
                    "QPushButton{border:1px solid %s;border-radius:3px;"
                    "padding:3px;color:%s}"
                    "QPushButton:hover{border:1px solid %s}"
                    "QPushButton:disabled{border:1px solid %s;color:%s}"
                    % (edge, txt, hover, LINE, C_MUTED))
                missing = [k for k in seq if k not in specs]
                if missing:
                    # A ROW WITH NO SPEC IS A DEAD BUTTON. Say so ON it.
                    b.setEnabled(False)
                    b.setToolTip("no launch spec: %s" % ", ".join(missing))
                    b.setText(btxt + " (no spec)")
                else:
                    b.setToolTip(
                        "Launches, in order: %s.%s"
                        % (" then ".join(specs[k].label for k in seq),
                           ("\n\n" + self.REAL_STEP[tag][2])
                           if tag in self.REAL_STEP else ""))
                    b.clicked.connect(
                        lambda _, s_=seq, n_="%s / %s" % (label, btxt):
                        self.start_mode(s_, n_))
                row.addWidget(b)
                self.mode_btn["%s_%s" % (key, tag)] = b
            v.addLayout(row)

            sub = QLabel(caveat)
            sub.setWordWrap(True)
            sub.setFont(helvetica(8))
            sub.setStyleSheet("color:%s" % C_MUTED)
            v.addWidget(sub)

            if key == "full":
                b = QPushButton("TYPE WHAT YOU WANT  ->  Instruct")
                b.setFont(helvetica(10, True))
                b.setMinimumHeight(28)
                b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
                b.setToolTip("Switches the middle column to the Instruct "
                             "panel and puts the keyboard in the box. "
                             "Nothing is commanded until CONFIRM.")
                b.clicked.connect(self.on_show_instruct)
                v.addWidget(b)
                self.mode_btn["__instruct__"] = b

        live = QLabel("live mode: --")
        live.setFont(helvetica(9, True))
        live.setStyleSheet("color:%s" % C_MUTED)
        self.mode_live_lbl = live
        v.addWidget(live)

        self.mode_seq_lbl = QLabel("")
        self.mode_seq_lbl.setWordWrap(True)
        self.mode_seq_lbl.setFont(helvetica(9))
        self.mode_seq_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.mode_seq_lbl)

        stop = QPushButton("STOP everything this window launched")
        stop.setFont(helvetica(10, True))
        stop.clicked.connect(self.on_stop_jobs)
        v.addWidget(stop)
        return g

    # ------------------------------------------------- the mode sequencer
    def start_mode(self, seq, name, _i=0):
        """Launch a chain of specs, WAITING for the stack between them.

        `real` and `vr` both carry `needs_stack=True`, so firing them at the
        same moment as the stack they need is a refusal every time -- which
        is what a single button that "starts everything" would have done.
        This launches step 1, waits for the stack to actually appear, then
        launches step 2, and SAYS which step it is on.

        It gives up loudly. A sequence that stalls silently is worse than a
        refusal, because the operator has no idea which half is missing.
        """
        specs = {sp.key: sp for sp in self._ensure_specs()}
        if _i == 0:
            self._seq_deadline = time.monotonic() + 90.0
            self._seq_name = name
        if _i >= len(seq):
            self._seq_say("%s: all %d step(s) launched." % (name, len(seq)))
            return
        key = seq[_i]
        sp = specs.get(key)
        if sp is None:
            self._seq_say("%s: no launch spec %r -- stopping here."
                          % (name, key), bad=True)
            return
        # Wait for the stack when this step needs one and there is not one.
        if sp.needs_stack and _stack_pids() <= 0:
            if time.monotonic() > getattr(self, "_seq_deadline", 0):
                self._seq_say(
                    "%s: step %d of %d (%s) needs a running stack and none "
                    "appeared in 90 s. Nothing further was launched -- look "
                    "at .scratch/launch_%s.log for why the stack did not "
                    "come up."
                    % (name, _i + 1, len(seq), sp.label, seq[0]), bad=True)
                return
            self._seq_say("%s: waiting for the stack before step %d of %d "
                          "(%s)..." % (name, _i + 1, len(seq), sp.label))
            QTimer.singleShot(1500,
                              lambda: self.start_mode(seq, name, _i))
            return
        self._seq_say("%s: step %d of %d -- %s"
                      % (name, _i + 1, len(seq), sp.label))
        self.on_launch(sp)
        QTimer.singleShot(2000, lambda: self.start_mode(seq, name, _i + 1))

    def _seq_say(self, text, bad=False):
        lbl = getattr(self, "mode_seq_lbl", None)
        if lbl is not None:
            lbl.setText(text)
            lbl.setStyleSheet("color:%s" % (C_BAD if bad else C_MUTED))
        self.bus.note(text, bad=bad)


    # =================================================================
    # EXPERIMENTS -- name it, run it, and keep everything it produced
    # =================================================================
    #
    # WHY. Running a trial meant a terminal, `run_experiment.sh`, and knowing
    # that `--participant`, `--session`, `--trial-index` and `--taskset`
    # exist. The GUI had task buttons that hard-coded PILOT and --scripted
    # and nothing else, so every recorded run came out under the same name
    # and the trial index was always 0. Two runs of the same task were
    # indistinguishable after the fact.
    #
    # Everything here has a DEFAULT that is the sane thing, so the panel can
    # be used without filling anything in -- but the fields exist, so a real
    # session can be named and a trial number can advance.
    TASK_CHOICES = [
        ("m1", "T1 pick and place"),
        ("m1s2", "T1 stage 2, both arms"),
        ("m0", "T0 target reaching"),
        ("m2", "T2 coordinated carry"),
        ("m3", "T3 circuit box + multimeter"),
        ("a", "A positioning"),
        ("b", "B coordinated carry"),
        ("c", "C dual pursuit"),
        ("d1", "Dance: flow"),
        ("d2", "Dance: pulse"),
        ("d3", "Dance: play"),
    ]
    MODE_CHOICES = ["06_full_autonomy", "01_master_teleop", "02_vr_teleop",
                    "03_shared_autonomy", "04_vr_shared"]

    def _experiments_panel(self):
        g = QGroupBox("EXPERIMENTS  --  run a trial and keep the data")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        v.setSpacing(3)

        intro = QLabel(
            "Every field has a working default. Press RUN TRIAL and it runs "
            "T1 under full autonomy as PILOT. Fill in what you need.")
        intro.setWordWrap(True)
        intro.setFont(helvetica(9))
        intro.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(intro)

        form = QGridLayout()
        form.setHorizontalSpacing(6)
        form.setVerticalSpacing(3)

        def _row(r, label, w, tip):
            lab = QLabel(label)
            lab.setFont(helvetica(9))
            lab.setStyleSheet("color:%s" % C_MUTED)
            w.setToolTip(tip)
            form.addWidget(lab, r, 0)
            form.addWidget(w, r, 1)

        self.exp_task = QComboBox()
        for k, lab in self.TASK_CHOICES:
            self.exp_task.addItem("%s  --  %s" % (k, lab), k)
        # WHAT THE NEXT RUN WILL BE RECORDED AS, said out loud. Changing the
        # task or the mode changes the identity of everything that follows,
        # and a silent change is how two different runs end up looking like
        # repeats of one.
        self.exp_task.currentIndexChanged.connect(
            lambda _: self.log("experiments: task -> %s"
                               % self.exp_task.currentData()))
        _row(0, "task", self.exp_task,
             "Which task to run. m1 is T1 pick and place, the one with the "
             "most recorded evidence behind it.")

        self.exp_mode = QComboBox()
        for m in self.MODE_CHOICES:
            self.exp_mode.addItem(m, m)
        self.exp_mode.currentIndexChanged.connect(
            lambda _: self.log("experiments: mode -> %s"
                               % self.exp_mode.currentData()))
        _row(1, "mode", self.exp_mode,
             "The control mode the trial is recorded under. T1 and T1S2 run "
             "under 06_full_autonomy only -- they command their own approach "
             "orientation, so a run under another mode measures a different "
             "geometry under that mode's name.")

        self.exp_participant = QLineEdit("PILOT")
        _row(2, "participant", self.exp_participant,
             "Goes into the manifest. write_manifest() REFUSES a name, "
             "email, date of birth, address or phone -- anonymity is "
             "enforced in code, not by convention. Use a code.")

        self.exp_session = QLineEdit("")
        self.exp_session.setPlaceholderText("blank = a timestamp")
        _row(3, "session", self.exp_session,
             "Groups trials together. Left blank the runner stamps the time, "
             "which is what you want unless you are resuming.")

        self.exp_trial = QSpinBox()
        self.exp_trial.setRange(0, 999)
        self.exp_trial.setValue(1)
        self.exp_trial.valueChanged.connect(
            lambda v: self.log("experiments: trial number -> %d" % v))
        _row(4, "trial no.", self.exp_trial,
             "--trial-index. It was always 0 from this window, so repeated "
             "runs of one task overwrote each other's identity.")

        self.exp_seed = QLineEdit("")
        self.exp_seed.setPlaceholderText("blank = the task's own default")
        _row(5, "seed", self.exp_seed,
             "Stage-2 layouts are drawn from this. Seed 3 gives a 3/1 split; "
             "seed 0 gives 2/2 and is indistinguishable from stage 1.")
        v.addLayout(form)

        opts = QHBoxLayout()
        self.exp_scripted = QCheckBox("scripted")
        self.exp_scripted.setChecked(True)
        self.exp_scripted.setToolTip(
            "No human operator: the runner drives the waypoints itself. This "
            "is what every recorded verification clip used. Uncheck only if "
            "somebody is actually on the master arm or the controllers.")
        self.exp_dry = QCheckBox("dry run")
        self.exp_dry.setToolTip(
            "Plan and check everything, command nothing. Always press this "
            "first on a task you have not run today.")
        for w in (self.exp_scripted, self.exp_dry):
            w.setFont(helvetica(9))
            # A TOGGLE THAT CHANGES WHAT THE NEXT RUN DOES AND SAYS NOTHING
            # is the shape this whole window is written against. `dry run`
            # in particular decides whether the arms move.
            w.stateChanged.connect(
                lambda st, x=w: self.log(
                    "experiments: %s is now %s"
                    % (x.text(), "ON" if st else "off")))
            opts.addWidget(w)
        v.addLayout(opts)

        rec = QHBoxLayout()
        self.exp_rec_csv = QCheckBox("CSV")
        self.exp_rec_csv.setChecked(True)
        self.exp_rec_csv.setToolTip(
            "full_state_recorder: master pose, commanded pose, joint states, "
            "clutch, FSR and buttons, one row per tick.")
        self.exp_rec_video = QCheckBox("video")
        self.exp_rec_video.setToolTip(
            "record_rviz.py on a virtual display. x11grab of THIS desktop "
            "records black -- XWayland pixels never reach the X root window "
            "-- so the recorder runs RViz on Xvfb instead. It is slow; leave "
            "it off unless the clip is the point.")
        for w in (self.exp_rec_csv, self.exp_rec_video):
            w.setFont(helvetica(9))
            w.stateChanged.connect(
                lambda st, x=w: self.log(
                    "experiments: recording %s is now %s"
                    % (x.text(), "ON" if st else "off")))
            rec.addWidget(w)
        v.addLayout(rec)

        for text, fn, tip in (
            ("RUN TRIAL", self.on_run_trial,
             "One trial with the settings above. The trial number advances "
             "afterwards so the next press is a different trial."),
            ("RUN FULL EXPERIMENT  (all modes)", self.on_run_experiment,
             "The same task under every mode it is allowed to run in, one "
             "after another, sharing one session name. This is the "
             "comparison the study is about."),
        ):
            b = QPushButton(text)
            b.setFont(helvetica(10, True))
            b.setMinimumHeight(28)
            b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            v.addWidget(b)
            self.buttons.setdefault("exp_" + text.split()[1].lower(), b)

        self.exp_status = QLabel("nothing run from this panel yet")
        self.exp_status.setWordWrap(True)
        self.exp_status.setFont(helvetica(9))
        self.exp_status.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.exp_status)

        b = QPushButton("open the recordings folder")
        b.setFont(helvetica(9))
        b.setToolTip("Where every trial's manifest, CSV, plots and clips "
                     "land: recordings/sessions/<session>/")
        b.clicked.connect(self.on_open_recordings)
        v.addWidget(b)
        return g

    # ------------------------------------------------------ experiment run
    def _exp_session(self):
        s = self.exp_session.text().strip()
        return s or datetime.now().strftime("%Y%m%d_%H%M%S")

    def _exp_argv(self, mode=None):
        task = self.exp_task.currentData()
        mode = mode or self.exp_mode.currentData()
        taskset = ("msc" if task.startswith("m")
                   else "demo" if task.startswith("d") else "clip")
        argv = [os.path.join(_WS, "scripts/run_experiment.sh"), task,
                "--mode", mode, "--taskset", taskset,
                "--participant", self.exp_participant.text().strip()
                or "PILOT",
                "--session", self._exp_session(),
                "--trial-index", str(self.exp_trial.value())]
        if self.exp_scripted.isChecked():
            argv.append("--scripted")
        if self.exp_dry.isChecked():
            argv.append("--dry-run")
        seed = self.exp_seed.text().strip()
        if seed:
            argv += ["--seed", seed]
        return ["bash"] + argv

    def on_run_trial(self):
        """One trial, and the recorders that go with it."""
        argv = self._exp_argv()
        if self.exp_rec_csv.isChecked():
            self._run_raw("full_state_recorder",
                          ["ros2", "run", "srl_teleop", "full_state_recorder"])
        if self.exp_rec_video.isChecked():
            self._run_raw("record_rviz",
                          ["python3", os.path.join(_WS,
                                                   "scripts/record_rviz.py"),
                           "--task", self.exp_task.currentData()])
        self._run_raw("trial %s" % self.exp_task.currentData(), argv)
        self.exp_status.setText(
            "trial %d of task %s under %s, session %s -- output in "
            ".scratch/launch_trial_%s.log and recordings/sessions/%s/"
            % (self.exp_trial.value(), self.exp_task.currentData(),
               self.exp_mode.currentData(), self._exp_session(),
               self.exp_task.currentData(), self._exp_session()))
        self.exp_status.setStyleSheet("color:%s" % C_TEXT)
        # ADVANCE THE TRIAL NUMBER. Leaving it means the next press records a
        # second trial under the first one's identity, which is exactly the
        # defect this panel exists to fix.
        self.exp_trial.setValue(self.exp_trial.value() + 1)

    def on_run_experiment(self):
        """The same task under every mode it is ALLOWED to run in.

        T1 and T1S2 are locked to 06 by the task itself -- they command their
        own approach orientation rather than the pinned anchor every teleop
        mode sends, so a run under another mode measures a different geometry
        under that mode's name. The dispatcher refuses those, and this must
        not queue a run it knows will be refused.
        """
        task = self.exp_task.currentData()
        locked = {"m1": ["06_full_autonomy"], "m1s2": ["06_full_autonomy"]}
        modes = locked.get(task, list(self.MODE_CHOICES))
        sess = self._exp_session()
        self.exp_session.setText(sess)          # pin it across the whole run
        self._exp_queue = [(m, self._exp_argv(m)) for m in modes]
        self.exp_status.setText(
            "%s under %d mode(s): %s -- session %s"
            % (task, len(modes), ", ".join(modes), sess))
        self.exp_status.setStyleSheet("color:%s" % C_TEXT)
        self._exp_next()

    def _exp_next(self):
        q = getattr(self, "_exp_queue", [])
        if not q:
            self.exp_status.setText(
                "%s -- every mode launched. Watch the event log; each run "
                "writes .scratch/launch_trial_*.log."
                % self.exp_status.text().split(" -- ")[0])
            return
        mode, argv = q.pop(0)
        self._run_raw("trial %s %s" % (self.exp_task.currentData(), mode),
                      argv)
        self.bus.note("experiment: launched %s under %s"
                      % (self.exp_task.currentData(), mode))
        # SPACED, not simultaneous. Two runs against one move_group is HARD
        # CONSTRAINT 3 one level down -- a verifier that overlapped a sweep
        # read 18 of 171 where two clean runs read 0.
        QTimer.singleShot(8000, self._exp_next)

    def on_open_recordings(self):
        d = os.path.join(_WS, "recordings/sessions")
        os.makedirs(d, exist_ok=True)
        for argv in (["xdg-open", d], ["explorer.exe", d]):
            try:
                subprocess.Popen(argv, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                self.bus.note("opened %s" % d)
                return
            except Exception:                                 # noqa: BLE001
                continue
        self.bus.note("could not open a file browser. The path is %s" % d,
                      bad=True)

    def on_show_instruct(self):
        """Bring the full-autonomy prompt to the front.

        It is in the middle column's tab strip, which is the right place for
        it -- it needs the width -- and the wrong place to DISCOVER it from.
        """
        tabs = getattr(self, "mid_tabs", None)
        idx = getattr(self, "instruct_tab_index", None)
        if tabs is None or idx is None:
            self.bus.note("the Instruct panel is not built in this window",
                          bad=True)
            return
        tabs.setCurrentIndex(idx)
        # TAKE THE KEYBOARD BACK, explicitly. Opening the panel that exists to
        # be typed into and leaving the focus wherever it was -- possibly on
        # the embedded RViz, which is a separate X client -- is the difference
        # between "type what you want" and a box that ignores you.
        self.focus_instruct()
        self.bus.note("Instruct panel: type what you want the arms to do. "
                      "Nothing moves until CONFIRM.")

    def _controls(self):
        """WHAT YOU TOUCH WHILE SOMETHING IS RUNNING, and nothing else.

        The precision dial and the two grip releases. The scale sliders, the
        anchor re-base and the clutch override moved to SET UP: they are set
        once at the start of a session, and a control you use once does not
        belong beside the one you reach for when the gripper will not let go.
        """
        g = QGroupBox("While it is running")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        self.dial = Dial(self.on_dial)
        v.addWidget(self.dial)

        r = QHBoxLayout()
        for lbl, fn in (("release LEFT grip", lambda: self.on_release("left")),
                        ("release RIGHT grip", lambda: self.on_release("right"))):
            b = QPushButton(lbl)
            b.clicked.connect(fn)
            r.addWidget(b)
        v.addLayout(r)
        return g

    def _map_panel(self):
        """MAP THE ENVIRONMENT FIRST, then plan against what was measured.

        THE GUI RULE, and the capability it exposes is the one the operator
        actually asked for: the arm should work out what is on the table
        rather than be told, and it should say what it is doing while it does
        it. Both live here -- the sweep is started from this button, the
        phase banner is the arm's own voice off `/robot_say`, and the object
        list is the map the pick is planned from.
        """
        g = QGroupBox("MAP THE ENVIRONMENT  --  measure first, then plan")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        # THE BANNER. One line, large, always the current phase.
        self.say_lbl = QLabel("idle")
        self.say_lbl.setFont(helvetica(12, True))
        self.say_lbl.setWordWrap(True)
        self.say_lbl.setMinimumHeight(34)
        self.say_lbl.setStyleSheet(
            "color:%s;border:1px solid %s;padding:4px" % (C_OK, C_MUTED))
        self.say_lbl.setToolTip(
            "What the robot says it is doing, from /robot_say. Silence here "
            "while the arm moves means the moving thing is not narrating -- "
            "which is a fact about the run, not about this label.")
        v.addWidget(self.say_lbl)

        row = QHBoxLayout()
        b = QPushButton("CALIBRATE THE ENVIRONMENT")
        b.setFont(helvetica(10, True))
        b.setMinimumHeight(28)
        b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        b.setToolTip(
            "Sweeps the arm over the workspace in straight serpentine rows, "
            "segments each frame, lifts the masks through the depth, and "
            "fuses one map: the support surface MEASURED, and every object "
            "on it with its width and whether the jaws can close on it. "
            "Takes a few minutes and narrates every cell.")
        b.clicked.connect(self.on_calibrate_environment)
        row.addWidget(b, 1)
        self.map_arm = QComboBox()
        self.map_arm.addItems(["left", "right"])
        self.map_arm.currentTextChanged.connect(
            lambda t: self.bus.note("map arm: %s" % t))
        row.addWidget(self.map_arm)
        v.addLayout(row)

        row2 = QHBoxLayout()
        b2 = QPushButton("SHOW THE MAP")
        b2.setFont(helvetica(10))
        b2.setToolTip("Reads recordings/baselines/world_map.json and lists "
                      "what it holds. Moves nothing.")
        b2.clicked.connect(self.on_show_map)
        row2.addWidget(b2, 1)
        b3 = QPushButton("PICK FROM THE MAP")
        b3.setFont(helvetica(10, True))
        b3.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
        b3.setToolTip(
            "Plans a pick for the selected object using the MEASURED centre "
            "and width -- no declared coordinates anywhere in the chain. "
            "PLANS ONLY: it prints every waypoint and commands nothing "
            "unless 'and move' is ticked.")
        b3.clicked.connect(self.on_pick_from_map)
        row2.addWidget(b3, 1)
        v.addLayout(row2)

        row3 = QHBoxLayout()
        row3.addWidget(QLabel("object"))
        self.map_obj = QSpinBox()
        self.map_obj.setRange(0, 99)
        self.map_obj.valueChanged.connect(
            lambda i: self.bus.note("map object index: %d" % i))
        row3.addWidget(self.map_obj)
        self.map_move = QCheckBox("and move")
        self.map_move.setToolTip(
            "Unticked, PICK FROM THE MAP plans and prints. Ticked, it "
            "commands the arm.")
        self.map_move.stateChanged.connect(
            lambda _s: self.bus.note(
                "pick from map will %s"
                % ("MOVE THE ARM" if self.map_move.isChecked()
                   else "plan only")))
        row3.addWidget(self.map_move)
        row3.addStretch(1)
        v.addLayout(row3)

        self.map_lbl = QLabel("no map read yet")
        self.map_lbl.setWordWrap(True)
        self.map_lbl.setFont(mono(8))
        self.map_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.map_lbl)
        return g

    def _map_say(self, text, bad=False):
        self.map_lbl.setText(str(text))
        self.map_lbl.setStyleSheet("color:%s" % (C_BAD if bad else C_MUTED))

    def on_calibrate_environment(self):
        """Start the sweep. It is a LAUNCH, not a call: it takes minutes."""
        arm = self.map_arm.currentText()
        py = os.path.join(_WS, ".venv_vision", "bin", "python")
        if not os.path.exists(py):
            # NAMED. The segmenter needs ultralytics, which lives in
            # .venv_vision on this host and not in the ROS interpreter.
            self._map_say(
                "the vision interpreter %s is missing, so the segmenter "
                "cannot run. Nothing was started." % py, bad=True)
            return
        argv = [py, "-u", os.path.join(_WS, "scripts",
                                       "calibrate_environment.py"),
                "--arm", arm]
        self.bus.note("calibrating the environment (%s arm): %s"
                      % (arm, " ".join(argv)))
        self._map_say("sweeping the %s arm over the workspace. Watch the "
                      "banner above." % arm)
        self._spawn(gls.Spec("calibrate_env", "calibrate environment",
                             "map", argv, needs_stack=True))

    def on_show_map(self):
        def go():
            try:
                sys.path.insert(0, os.path.join(_WS, "scripts"))
                import pick_from_map as PFM
                doc = PFM.load_map()
            except Exception as e:                            # noqa: BLE001
                self._map_say("no map to show: %r" % (e,), bad=True)
                return
            lines = ["surface z = %.4f m   %d object(s)   finder: %s"
                     % (doc["surface"]["z_m"], len(doc["objects"]),
                        doc.get("provenance", {})
                        .get("object_finder", "?")[:40])]
            for i, o in enumerate(doc["objects"]):
                lines.append("%2d  %s  %5.0f mm  %s"
                             % (i, " ".join("%7.3f" % v for v in o["centre"]),
                                o["width_m"] * 1000,
                                "graspable" if o.get("graspable")
                                else "NOT: " + o.get("why", "")[:40]))
            self.map_obj.setRange(0, max(0, len(doc["objects"]) - 1))
            self._map_say("\n".join(lines))
        self._map_say("reading the map...")
        self.bus.submit(go, label="show the map")

    def on_pick_from_map(self):
        arm = self.map_arm.currentText()
        idx = int(self.map_obj.value())
        move = self.map_move.isChecked()
        py = os.path.join(_WS, ".venv_vision", "bin", "python")
        argv = [py if os.path.exists(py) else sys.executable, "-u",
                os.path.join(_WS, "scripts", "pick_from_map.py"),
                "--arm", arm, "--object", str(idx)]
        if move:
            argv.append("--execute")
        # A CONTROL THAT CHANGES WHAT THE NEXT RUN DOES MUST SAY SO.
        self.bus.note("pick from map: object %d, %s arm, %s"
                      % (idx, arm, "MOVING THE ARM" if move else "plan only"))
        self._map_say("planning a pick for object %d%s"
                      % (idx, " and MOVING" if move else " (plan only)"))
        self._spawn(gls.Spec("pick_from_map", "pick from map", "map", argv,
                             needs_stack=True))

    def on_what_is_on_the_table(self):
        """Ask the wrist camera what is on the surface in front of it.

        Runs the SAME code path `scripts/pick_from_table.py` runs, so the
        window and the script cannot disagree about what is out there.
        """
        def go():
            try:
                import numpy as _np
                sys.path.insert(0, os.path.join(_WS, "scripts"))
                import pick_from_table as PFT
                from srl_perception import table_scene as _TS
            except Exception as e:                            # noqa: BLE001
                self._table_say("cannot import the scene analyser: %r" % (e,),
                                bad=True)
                return
            snap = self.bus._snapshot() if hasattr(self.bus, "_snapshot") \
                else {}
            depth = (snap or {}).get("gripper_depth")
            K = (snap or {}).get("gripper_K")
            pose = (snap or {}).get("gripper_cam_pose")
            if depth is None or K is None or pose is None:
                # THE HONEST ANSWER. A panel that invents a table when no
                # camera is attached is the defect this whole window exists
                # against.
                self._table_say(
                    "no wrist RGB-D frame has arrived, so there is nothing "
                    "to analyse. This needs the camera up and the arm's "
                    "camera_link pose in TF. Nothing was assumed.",
                    bad=True)
                return
            try:
                pts, uv = PFT.cloud_in_robot_frame(depth, K, pose,
                                                   with_pixels=True)
                sc = _TS.analyse(pts, pixel_uv=uv)
                self._table_say(_TS.describe(sc))
            except _TS.SceneRefusal as e:
                self._table_say("REFUSED: %s" % e, bad=True)

        self._table_say("looking...")
        self.bus.submit(go, label="what is on the table")

    def _table_say(self, text, bad=False):
        lbl = getattr(self, "table_lbl", None)
        if lbl is not None:
            lbl.setText(text)
            lbl.setStyleSheet("color:%s" % (C_BAD if bad else C_TEXT))
        self.bus.note("table: %s" % text.replace("\n", " | ")[:160], bad=bad)

    def _setup_controls(self):
        """Set once, before a run. Deliberately NOT beside the running ones."""
        g = QGroupBox("Set once, before a run")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        row = QHBoxLayout()
        self.force_clutch = QCheckBox("force clutch ENGAGED")
        self.force_clutch.stateChanged.connect(self.on_force_clutch)
        row.addWidget(self.force_clutch)
        b = QPushButton("re-base anchor")
        b.setToolTip("/master_rebase -- the next VALID frame becomes the "
                     "reference. The bridge calls this when it enables.")
        b.clicked.connect(lambda: self.bus.submit(
            lambda: self.bus.call_trigger("/master_rebase"),
            label="re-base anchor requested"))
        row.addWidget(b)
        v.addLayout(row)

        self.scale = {}
        for a in ARMS:
            r = QHBoxLayout()
            r.addWidget(QLabel("%s scale" % a))
            sl = QSlider(Qt.Horizontal)
            sl.setRange(20, 200)
            sl.setValue(100)
            lab = QLabel("1.00")
            lab.setFont(helvetica(10))
            sl.valueChanged.connect(
                lambda x, l=lab: l.setText("%.2f" % (x / 100.0)))
            sl.sliderReleased.connect(
                lambda a=a, sl=sl: self.on_scale(a, sl.value() / 100.0))
            r.addWidget(sl, 1)
            r.addWidget(lab)
            v.addLayout(r)
            self.scale[a] = sl
        return g

    # ====================================================================
    # HOW THE ARMS MOVE -- the motion generator
    # ====================================================================
    #
    # WHY THIS IS A CONTROL AND NOT A CONSTANT. Until 2026-08-23 the whole of
    # motion generation for teleoperation was `clamp_towards`, which clamps
    # each joint INDEPENDENTLY to `max_step_rad` per cycle. The joints
    # therefore sit at different fractions of their own travel at every
    # instant, and the hand leaves the straight line the IK solution implies
    # by up to 51.3 mm on a realistic slew -- against a 30 mm grasp gate.
    # It is now Ruckig: jerk-limited, synchronised, and the first thing in
    # this project ever to read joint_limits.yaml.
    #
    # THE OLD ONE IS STILL SELECTABLE, because a recording made before that
    # date was made with it, exactly as `orientation_policy:=exact`
    # reproduces one made before the cone. Selecting it is a deliberate act
    # and the window says what it costs.
    #
    # IT TAKES EFFECT ON THE NEXT LAUNCH AND THE PANEL SAYS SO. Every
    # parameter in `ik_follower_node` except `motion_enabled` is read once at
    # construction -- the node warns about this itself -- so a control that
    # appeared to change a running follower would be a control that changed
    # nothing. The STATUS tab reads what is ACTUALLY running, from the
    # follower's own status topic, never from which item is selected here.
    def _motion_panel(self):
        g = QGroupBox("How the arms move (next launch)")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        row = QHBoxLayout()
        row.addWidget(QLabel("generator"))
        self.motion_gen = QComboBox()
        self.motion_gen.setFont(helvetica(10))
        for key, lab in MOTION_GENERATORS:
            self.motion_gen.addItem(lab, key)
        self.motion_gen.currentIndexChanged.connect(self.on_motion_generator)
        row.addWidget(self.motion_gen, 1)
        v.addLayout(row)

        self.motion_note = QLabel()
        self.motion_note.setFont(helvetica(9))
        self.motion_note.setWordWrap(True)
        self.motion_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.motion_note)

        b = QPushButton("measure what it does to the hand")
        b.setToolTip("scripts/measure_teleop_motion.py -- runs its own "
                     "instrument self-test first, then reports how far the "
                     "end effector strays from the path it was asked for, "
                     "for every generator, offline. No stack needed.")
        b.clicked.connect(self.on_measure_motion)
        v.addWidget(b)

        self.motion_result = QLabel()
        self.motion_result.setFont(helvetica(9))
        self.motion_result.setWordWrap(True)
        self.motion_result.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.motion_result)
        self._motion_describe()
        return g

    def _motion_describe(self):
        key = self.motion_gen.currentData()
        self.motion_note.setText(dict(MOTION_NOTES)[key])
        self.motion_note.setStyleSheet(
            "color:%s" % (C_MUTED if key == "ruckig" else C_WARN))

    def on_motion_generator(self, _i=0):
        """A control that changes what the next run does SAYS so."""
        key = self.motion_gen.currentData()
        self._motion_describe()
        self.bus.note("motion generator for the NEXT launch -> %s "
                      "(running followers are unchanged; it is read at "
                      "startup)" % key, bad=(key != "ruckig"))

    def on_measure_motion(self):
        """Run it OFF the Qt thread -- it spawns an interpreter and does FK.

        The result is stashed and rendered by a timer, the same shape
        `on_check_dependencies` uses: touching a widget from a worker thread
        is how a Qt program crashes in a way that looks like a robot fault.
        """
        self._motion_rows = None
        self.motion_result.setText("measuring, offline, a few seconds...")
        self.motion_result.setStyleSheet("color:%s" % C_MUTED)

        def go():
            try:
                r = subprocess.run(
                    [sys.executable,
                     os.path.join(_WS, "scripts/measure_teleop_motion.py"),
                     "--no-write"],
                    capture_output=True, text=True, timeout=600)
            except Exception as e:                            # noqa: BLE001
                self._motion_rows = ("FAILED to run: %r" % (e,), True)
                self.bus.note("motion measurement failed: %r" % (e,), bad=True)
                return
            out = (r.stdout or "") + (r.stderr or "")
            rows = [ln.rstrip() for ln in out.splitlines()
                    if ln.strip().startswith(("clamp_towards", "ruckig",
                                              "synchronised-clamp"))]
            if r.returncode != 0 or not rows:
                self._motion_rows = ("measurement FAILED (exit %d) -- see the "
                                     "event log" % r.returncode, True)
                self.bus.note("measure_teleop_motion exited %d: %s"
                              % (r.returncode, out[-400:]), bad=True)
                return
            self._motion_rows = (
                "off-path mm / same-cycle mm / peak speed / arrives:\n"
                + "\n".join(rows), False)
            self.bus.note("motion measured -- %s" % "; ".join(
                " ".join(x.split()) for x in rows[:3]))

        self.bus.submit(go, label="motion measurement")
        QTimer.singleShot(1500, self._motion_render)

    def _motion_render(self, tries=40):
        got = getattr(self, "_motion_rows", None)
        if got is None:
            if tries > 0:
                QTimer.singleShot(500, lambda: self._motion_render(tries - 1))
            else:
                self.motion_result.setText("measurement did not finish")
                self.motion_result.setStyleSheet("color:%s" % C_BAD)
            return
        text, bad = got
        self.motion_result.setText(text)
        self.motion_result.setStyleSheet("color:%s"
                                         % (C_BAD if bad else C_MUTED))

    # ====================================================================
    # CONNECTING THE REAL ARMS -- the seven faults, named, each with a button
    # ====================================================================
    #
    # WHAT THIS PANEL IS FOR. Every connection problem this project has hit
    # was diagnosed by a person remembering something: which shell had sourced
    # what, that /dev/shm had to be cleared with the stack DOWN, that the
    # daemon hangs rather than fails, that the Teensy moves between ACM0 and
    # ACM1, that the arm permits exactly one session. Remembering is not a
    # mechanism. Each of those is now a row here: what is wrong in plain
    # words, and the fix as a button.
    #
    # THE TWO RULES THE ROWS OBEY, and they are the reason the checks live in
    # srl_teleop/real_arm_doctor.py rather than in this file:
    #
    #   * NEVER GREEN ON UNKNOWN. A check that could not run reads "could not
    #     be checked" in purple. G-3 says the readiness gate is never green on
    #     unknown; a connection panel that says "ready" because it could not
    #     look is the same defect with the arms switched on.
    #   * A CHECK THAT CANNOT FAIL ON A DELIBERATELY BROKEN INPUT IS NOT A
    #     CHECK. Every check reads the world through a Probe object, so
    #     test_real_arm_doctor.py hands each one a broken world and requires
    #     it to say so -- without unplugging a board or killing a daemon.
    #
    # NO TOPIC NAMES AND NO RAW ERRORS in what is displayed. The machinery
    # goes to the event log, where it is useful for a bug report and harmless
    # under time pressure.
    # ====================================================================
    # THE ONE BUTTON: START VR TELEOP
    # ====================================================================
    #
    # WHAT IT REPLACES. Six commands in four terminals in an order you had to
    # know, where every one could half-fail and two of them fail SILENTLY --
    # the bridge dying on a bound port while the other four nodes come up
    # fine, and the mapper started before the simulation, which gives a clutch
    # that does nothing and says nothing.
    #
    # THE ORDER IS LOAD-BEARING and is encoded in `vr_bringup.STEPS`, not
    # here: the simulation must be up before the mapper, the certificate must
    # be right before the bridge starts, the port must be free before the
    # bridge takes it.
    #
    # IT RUNS ON A WORKER. Bringing a stack up takes a minute or more, and the
    # window carrying the e-stop must not stop repainting for a minute. The
    # rows update as each step finishes.
    #
    # IT CANNOT REACH A REAL ARM, by construction and by test. A one-click
    # bring-up that could energise a robot as a side effect is the wrong
    # shape whatever it prints.
    def _vr_panel(self):
        g = QGroupBox("VR teleoperation")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        self.vr_btn = QPushButton("START VR TELEOP")
        self.vr_btn.setFont(helvetica(13, True))
        self.vr_btn.setMinimumHeight(44)
        self.vr_btn.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
        self.vr_btn.setToolTip(
            "Brings up everything a VR session needs, in order, and stops at "
            "the first thing it cannot do -- with the fix as a button. It "
            "does not touch the real arms.")
        self.vr_btn.clicked.connect(self.on_vr_start)
        v.addWidget(self.vr_btn)

        # I AM WORKING ALONE. Beside the observer check, never instead of it.
        #
        # It is UNTICKED every time this window opens -- `clear()` runs in
        # __init__ -- so it is a decision taken this session and not a setting
        # that quietly carries over. Ticking it writes an audit line with a
        # timestamp; the run afterwards is identifiable as one that had nobody
        # watching the arm.
        self.vr_alone = QCheckBox(
            "I am working alone - bypass the observer requirement")
        self.vr_alone.setFont(helvetica(9, True))
        self.vr_alone.setToolTip(
            "The observer check still runs and still reports what it finds. "
            "With this ticked, having no observer stops blocking the real "
            "arms instead of refusing them. Every use is written to "
            "recordings/observer_bypass_log.jsonl with the time. It clears "
            "itself when this window closes and expires after four hours.")
        self.vr_alone.toggled.connect(self.on_vr_alone)
        v.addWidget(self.vr_alone)

        self.vr_alone_note = QLabel("")
        self.vr_alone_note.setFont(helvetica(8))
        self.vr_alone_note.setWordWrap(True)
        self.vr_alone_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vr_alone_note)

        self.vr_head = QLabel("not started")
        self.vr_head.setFont(helvetica(11, True))
        self.vr_head.setWordWrap(True)
        self.vr_head.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vr_head)

        self.vr_rows = {}
        for key, title, _fn in vrb.STEPS:
            box = QWidget()
            bl = QVBoxLayout(box)
            bl.setContentsMargins(0, 1, 0, 1)
            bl.setSpacing(1)
            t = QLabel("     %s" % title)
            t.setFont(helvetica(9))
            t.setStyleSheet("color:%s" % C_MUTED)
            t.setWordWrap(True)
            why = QLabel("")
            why.setFont(helvetica(9))
            why.setWordWrap(True)
            why.setStyleSheet("color:%s" % C_MUTED)
            why.setVisible(False)
            fix = QPushButton("")
            fix.setFont(helvetica(9, True))
            fix.setVisible(False)
            fix.clicked.connect(lambda _, k=key: self.on_vr_fix(k))
            bl.addWidget(t)
            bl.addWidget(why)
            bl.addWidget(fix)
            v.addWidget(box)
            self.vr_rows[key] = (box, t, why, fix)

        row = QHBoxLayout()
        b = QPushButton("stop VR")
        b.setFont(helvetica(9))
        b.setToolTip("Stops everything this button started. Leaves the "
                     "simulation alone.")
        b.clicked.connect(self.on_vr_stop)
        row.addWidget(b)
        b2 = QPushButton("open the headset page")
        b2.setFont(helvetica(9))
        b2.clicked.connect(self.on_vr_show_url)
        row.addWidget(b2)
        row.addStretch(1)
        v.addLayout(row)

        self.vr_note = QLabel("")
        self.vr_note.setFont(helvetica(9))
        self.vr_note.setWordWrap(True)
        self.vr_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vr_note)

        # THE BYPASS NEVER SURVIVES A WINDOW. Clearing here is what makes
        # ticking the box a decision taken THIS session: a grant left behind
        # by yesterday's run cannot authorise today's, and the operator has to
        # say it again. The checkbox is therefore always drawn unticked, and
        # that is the true state rather than a default appearance.
        try:
            from srl_teleop import observer_bypass as _OB
            _OB.clear(who="gui", note="operations window opened")
        except Exception:                                     # noqa: BLE001
            pass

        self._vr_world = None
        self._vr_results = []
        self._vr_fixes = {}
        self._vr_busy = False
        self._vr_lock = threading.Lock()
        # A QUEUE, NOT A SLOT. The worker finishes a step roughly every second
        # and `refresh()` collects every 100 ms -- but two fast steps in a row
        # overwrote each other and the row for the first never left "...".
        # Rows that never show their outcome is precisely the failure this
        # panel exists to prevent, reproduced in the panel.
        self._vr_pending = []
        return g

    # ------------------------------------------------------------ running
    def on_vr_alone(self, on):
        """Take or drop the working-alone bypass. Deliberate, and recorded."""
        try:
            from srl_teleop import observer_bypass as OB
        except Exception as e:                                # noqa: BLE001
            self.vr_alone_note.setText("could not reach the bypass: %r" % (e,))
            return
        if on:
            rec = OB.grant(who="gui", reason="operator working alone")
            self.vr_alone_note.setText(
                "WORKING ALONE since %s. Nobody is holding an e-stop. This "
                "is in the log." % rec["granted_at_iso"])
            self.vr_alone_note.setStyleSheet("color:%s" % C_WARN)
            self.log("OBSERVER BYPASS TAKEN at %s -- no observer for this "
                     "session" % rec["granted_at_iso"], bad=True)
        else:
            OB.clear(who="gui", note="unticked")
            self.vr_alone_note.setText("Observer required.")
            self.vr_alone_note.setStyleSheet("color:%s" % C_MUTED)
            self.log("observer bypass cancelled -- an observer is required "
                     "again")

    def on_vr_start(self):
        if self._vr_busy:
            self.log("VR bring-up is already running")
            return
        self._vr_busy = True
        self.vr_btn.setEnabled(False)
        self.vr_head.setText("STARTING...")
        self.vr_head.setStyleSheet("color:%s" % C_WARN)
        self.log("VR bring-up: starting")
        self._vr_results = []
        for key, (box, t, why, fix) in self.vr_rows.items():
            why.setVisible(False)
            fix.setVisible(False)
            t.setStyleSheet("color:%s" % C_MUTED)
        if self._vr_world is None:
            self._vr_world = vrb.World(ws=_WS)

        def work():
            out = []
            for step in vrb.plan():
                with self._vr_lock:
                    self._vr_pending.append(
                        ("running", step.key, None, list(out)))
                res = step.run(self._vr_world)
                out.append((step.key, res))
                with self._vr_lock:
                    self._vr_pending.append(
                        ("step", step.key, res, list(out)))
                if res.state == vrb.FAILED:
                    break
            with self._vr_lock:
                self._vr_pending.append(("done", None, None, list(out)))
        threading.Thread(target=work, daemon=True).start()

    def _vr_collect(self):
        """Called from refresh(). Applies whatever the worker has finished."""
        if not hasattr(self, "vr_rows"):
            return
        with self._vr_lock:
            batch, self._vr_pending = self._vr_pending, []
        for pending in batch:
            self._vr_apply(pending)

    def _vr_apply(self, pending):
        kind, key, res, out = pending
        self._vr_results = out
        if kind == "running":
            box, t, why, fix = self.vr_rows[key]
            t.setText("...  %s"
                      % {k: ti for k, ti, _f in vrb.STEPS}[key])
            t.setStyleSheet("color:%s" % C_WARN)
            return
        if kind == "step":
            self._vr_render_step(key, res)
            return
        # done
        self._vr_busy = False
        self.vr_btn.setEnabled(True)
        state, head = vrb.verdict([r for _k, r in out], keyed=out)
        col = {vrb.OK: C_OK, vrb.FAILED: C_BAD,
               vrb.UNKNOWN: C_UNKNOWN}.get(state, C_UNKNOWN)
        self.vr_head.setText(head.upper())
        self.vr_head.setStyleSheet("color:%s" % col)
        self.log("VR bring-up: %s" % head, bad=(state == vrb.FAILED))
        if state == vrb.OK:
            self.vr_note.setText(
                "Put the headset on its shelf, open the address above in its "
                "own browser, accept the one warning and press ENTER VR.")

    def _vr_render_step(self, key, res):
        titles = {k: t for k, t, _f in vrb.STEPS}
        box, t, why, fix = self.vr_rows[key]
        mark = {vrb.OK: "OK  ", vrb.FAILED: "FIX ", vrb.UNKNOWN: "?   ",
                vrb.SKIPPED: "--  ",
                vrb.BYPASSED: "!!  "}.get(res.state, "?   ")
        col = {vrb.OK: C_TEXT, vrb.FAILED: C_BAD, vrb.UNKNOWN: C_UNKNOWN,
               vrb.SKIPPED: C_MUTED,
               vrb.BYPASSED: C_WARN}.get(res.state, C_UNKNOWN)
        t.setText("%s%s" % (mark, titles[key]))
        t.setFont(helvetica(9, res.state in (vrb.FAILED, vrb.UNKNOWN)))
        t.setStyleSheet("color:%s" % col)
        why.setText(res.plain)
        why.setVisible(res.state != vrb.OK)
        if res.has_fix:
            self._vr_fixes[key] = res.fix
            fix.setText(res.fix_label)
            fix.setToolTip(res.fix_note or res.detail)
            fix.setVisible(True)
        else:
            # HIDDEN AND UNBOUND. A visible button with no repair behind it is
            # the "feature present but does nothing" row of CLAUDE.md's
            # instrument table.
            fix.setVisible(False)
            self._vr_fixes.pop(key, None)
        box.setToolTip(res.detail)
        self.log("VR %s: %s -- %s" % (key, res.state, res.plain[:70]),
                 bad=(res.state == vrb.FAILED))

    def on_vr_fix(self, key):
        """Run the repair the latest diagnosis attached to this row, then
        continue from the top -- the steps are ordered, so a repair three
        steps down may have changed what the ones above see."""
        name = self._vr_fixes.get(key)
        if not name:
            self.log("nothing to fix on that row any more", bad=True)
            return
        if name.startswith("show_"):
            self._vr_show_log(name)
            return
        # `reexec` lives on this window, not in `vr_bringup` -- restarting the
        # process is something only the GUI can do. It was in the DOCTOR
        # panel's table and not this one, so step 1's fix button answered
        # "no such repair: reexec". Step 1 is the environment check, which is
        # the first thing that fails on an unsourced window, so the one
        # button an operator would reach for first was the dead one.
        if name == "reexec":
            ok, msg = self._fix_reexec()
            self.vr_note.setText(msg)
            self.log("VR fix %s: %s" % (key, msg), bad=not ok)
            return
        fn = vrb.FIXES.get(name)
        if fn is None:
            self.log("no such repair: %s" % name, bad=True)
            return
        self.vr_note.setText("working...")
        try:
            ok, msg = fn(self._vr_world or vrb.World(ws=_WS))
        except Exception as e:                                # noqa: BLE001
            ok, msg = False, "could not do it: %r" % (e,)
        self.vr_note.setText(msg)
        self.log("VR fix %s: %s" % (key, msg), bad=not ok)
        if ok and not name.endswith("_help"):
            QTimer.singleShot(400, self.on_vr_start)

    def _vr_show_log(self, name):
        which = name.replace("show_", "").replace("_log", "")
        path = os.path.join(_scratch(), "vr_bringup_%s.log" % which)
        if not os.path.exists(path):
            self.vr_note.setText("There is nothing written down for that yet.")
            return
        try:
            with open(path, "rb") as fh:
                fh.seek(0, 2)
                fh.seek(max(0, fh.tell() - 2500))
                tail = fh.read().decode("utf-8", "replace")
        except OSError as e:
            self.vr_note.setText("could not read it: %r" % (e,))
            return
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.information(self, "What it printed", tail[-2000:])
        self.log("showed the %s log" % which)

    def on_vr_stop(self):
        """Stop what the button started. NOT the simulation.

        Killing the simulation too would mean every retry paid a full stack
        restart, and the operator would stop using the button.
        """
        n = 0
        w = self._vr_world
        for name, p in list(getattr(w, "started", []) or []):
            if name == "simulation" or p is None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), 2)
                n += 1
            except Exception:                                 # noqa: BLE001
                pass
        if w is not None:
            w.started = [(nm, p) for nm, p in w.started if nm == "simulation"]
        self.vr_head.setText("STOPPED")
        self.vr_head.setStyleSheet("color:%s" % C_MUTED)
        self.log("VR: stopped %d part(s). The simulation is still up." % n)

    def on_vr_show_url(self):
        """EVERY address, not the one we guess.

        This machine has two: the interface with the default route (a NAT or
        corporate link) and the lab switch the robot is on. Handing over one
        of them sends the operator to a page the headset cannot load, and
        inside a headset that is indistinguishable from a firewall problem or
        a bad certificate.
        """
        w = self._vr_world or vrb.World(ws=_WS)
        ips = w.lan_ips()
        if not ips:
            self.vr_note.setText("This machine has no address on the network.")
            self.log("no LAN address", bad=True)
            return
        urls = ["https://%s:%d/" % (i, w.port) for i in ips]
        if len(urls) == 1:
            msg = "In the headset's own browser: %s" % urls[0]
        else:
            msg = ("In the headset's own browser, whichever is the wifi the "
                   "headset is on: " + "   or   ".join(urls)
                   + "    (the first is the network the robot is on)")
        self.vr_note.setText(msg)
        for u in urls:
            self.log("headset page: %s" % u)

    def _connect_panel(self):
        g = QGroupBox("Connect the real arms")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        self.doc_head = QLabel("NOT CHECKED YET")
        self.doc_head.setFont(helvetica(11, True))
        self.doc_head.setStyleSheet("color:%s" % C_UNKNOWN)
        self.doc_head.setWordWrap(True)
        v.addWidget(self.doc_head)

        row = QHBoxLayout()
        b = QPushButton("CHECK EVERYTHING")
        b.setFont(helvetica(10, True))
        b.setToolTip("Runs the seven connection checks. They shell out, so "
                     "they run off this window's thread -- the window stays "
                     "responsive and the e-stop stays live while they run.")
        b.clicked.connect(lambda: self.on_doctor_check(announce=True))
        row.addWidget(b)
        self.doc_auto = QCheckBox("re-check every 30 s")
        self.doc_auto.setChecked(True)
        # LOGGED, like every other control. A checkbox that changes behaviour
        # and leaves no trace is indistinguishable from a disconnected signal,
        # which is the whole bug class the button audit exists for -- and the
        # audit itself requires a trace from every control it touches.
        self.doc_auto.stateChanged.connect(
            lambda st: self.log("connection re-check every 30 s: %s"
                                % ("ON" if st else "OFF")))
        row.addWidget(self.doc_auto)
        v.addLayout(row)

        # SEVEN PERMANENT ROWS, BUILT ONCE AND UPDATED IN PLACE.
        #
        # The first version rebuilt them from each diagnosis, which is tidy
        # and wrong: a re-check lands every 30 s and would delete the button
        # the operator is reaching for, mid-reach. The button audit found it
        # by crashing on a QPushButton that had been deleted while it was
        # being walked -- the same event, with a machine doing the reaching.
        #
        # So the widgets are fixed and only their CONTENT changes. Each row's
        # fix button is connected once, to a slot that looks up whatever fix
        # the latest diagnosis put there; a row with no fix hides its button
        # rather than keeping a stale one wired to the previous run's repair.
        self.doc_rows = {}
        self._doc_fix = {}
        for key in rad.KEYS:
            box = QWidget()
            bl = QVBoxLayout(box)
            bl.setContentsMargins(0, 3, 0, 3)
            bl.setSpacing(1)
            title = QLabel("?  %s" % rad.UNCHECKED[key])
            title.setFont(helvetica(10))
            title.setStyleSheet("color:%s" % C_UNKNOWN)
            title.setWordWrap(True)
            plain = QLabel("")
            plain.setFont(helvetica(9))
            plain.setWordWrap(True)
            plain.setStyleSheet("color:%s" % C_MUTED)
            plain.setVisible(False)
            fix = QPushButton("")
            fix.setFont(helvetica(9, True))
            fix.setVisible(False)
            fix.clicked.connect(lambda _, k=key: self._doctor_fix(k))
            bl.addWidget(title)
            bl.addWidget(plain)
            bl.addWidget(fix)
            v.addWidget(box)
            self.doc_rows[key] = (box, title, plain, fix)

        self.doc_note = QLabel("")
        self.doc_note.setFont(helvetica(9))
        self.doc_note.setWordWrap(True)
        self.doc_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.doc_note)

        # THE CHECKS RUN OFF THIS THREAD. Measured: run inline they froze the
        # window for 16.7 s, because asking what is running has to WAIT to
        # find out that the answer is not coming -- that wait is the check.
        # A window carrying the e-stop must not stop repainting for sixteen
        # seconds, so a worker does the waiting and `refresh()` collects the
        # answer when it arrives.
        self.doc_checks = []
        # A TEST SEAM, and it earns its place. The button audit forces each
        # connection fault into the panel and presses its repair; without a
        # way to hold the live re-check off, the 30 s timer and the 600 ms
        # post-repair re-check land mid-loop and replace the injected rows
        # with this machine's real ones -- so the button the audit is about to
        # press is destroyed under it. That failed as "missing: daemon, home",
        # which reads like two broken repairs and was neither.
        self.doctor_frozen = False
        self._doc_result = None
        self._doc_lock = threading.Lock()
        self._doc_busy = False
        QTimer.singleShot(1200, self.on_doctor_check)
        self.doc_timer = QTimer(self)
        self.doc_timer.timeout.connect(
            lambda: self.on_doctor_check() if self.doc_auto.isChecked()
            else None)
        self.doc_timer.start(30000)
        return g

    # ---------------------------------------------------------- the fixes
    def _doctor_fix(self, key):
        """Run the repair the LATEST diagnosis attached to this row.

        Looked up rather than bound at connect time, so a row can never fire
        the previous diagnosis's repair -- which, for a row whose repair stops
        processes, is the difference between fixing the fault in front of you
        and stopping something that is now fine.
        """
        fn = self._doc_fix.get(key)
        if fn is None:
            self.log("nothing to fix on that row any more", bad=True)
            return
        fn()

    def _doctor_fixes(self):
        """The button behind each row. Every one of them says what it did.

        A fix that runs silently is a fix the operator cannot tell from a
        button that does nothing, which is the failure the whole button audit
        exists for.
        """
        def wrap(fn):
            def go():
                try:
                    ok, msg = fn()
                except Exception as e:                        # noqa: BLE001
                    ok, msg = False, "could not do it: %r" % (e,)
                self.log(msg, bad=not ok)
                self.doc_note.setText(msg)
                QTimer.singleShot(600, self.on_doctor_check)
            return go

        return {
            "reexec": wrap(self._fix_reexec),
            "clear_shm": wrap(rad.fix_clear_shm),
            "reset_daemon": wrap(rad.fix_reset_daemon),
            "kill_second_stack": wrap(rad.fix_kill_second_stack),
            "kill_stray_rsp": wrap(rad.fix_kill_stray_rsp),
            "kill_orphans": wrap(self._fix_kill_orphans),
            "teensy_help": wrap(self._fix_teensy_help),
            "teensy_repoint": wrap(self._fix_teensy_repoint),
            "home_arms": wrap(self._fix_home_arms),
            "release_kortex": wrap(self._fix_release_kortex),
        }

    def _fix_reexec(self):
        """Restart this window with the settings everything else is using.

        THE ONLY HONEST FIX FOR THIS ONE. Discovery settings are read when the
        middleware starts, so changing them in this process now would change
        nothing and report success -- the exact silent-acceptance failure this
        GUI is written against. So the window really does restart.
        """
        env = dict(os.environ)
        probe = rad.Probe()
        target = None
        for pid in probe.stack_pids():
            e = probe.env_of(pid)
            if e:
                target = e
                break
        for k in rad.DISCOVERY_VARS:
            want = (target or {}).get(k, rad.EXPECTED.get(k, ""))
            if want:
                env[k] = want
            else:
                env.pop(k, None)
        from PyQt5.QtWidgets import QMessageBox
        r = QMessageBox.question(
            self, "Restart this window",
            "This window will close and reopen with the settings the rest of "
            "the system is using.\n\nAnything it launched keeps running.\n\n"
            "Restart now?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return False, "Restart declined -- nothing changed."
        argv = [sys.executable, os.path.abspath(sys.argv[0])] + sys.argv[1:]
        try:
            subprocess.Popen(argv, env=env, start_new_session=True)
        except Exception as e:                                # noqa: BLE001
            return False, "Could not reopen the window: %r" % (e,)
        QTimer.singleShot(500, self.close)
        return True, "Reopening with matching settings."

    def _fix_kill_orphans(self):
        """Confirmed first, and it names what it will stop.

        Every other repair here undoes something that is already broken. This
        one stops running processes, and a list of names is the difference
        between "stop the leftovers" and "stop seven things I cannot see".
        """
        targets = rad.Probe().orphan_nodes()
        if not targets:
            return False, "Nothing left over to stop."
        names = "\n".join("  " + c.rsplit("/", 1)[-1][:60]
                          for _, c in targets[:12])
        from PyQt5.QtWidgets import QMessageBox
        r = QMessageBox.question(
            self, "Stop the leftovers",
            "These are still running and the run that started them has "
            "ended:\n\n%s%s\n\nStop them?"
            % (names, "\n  ..." if len(targets) > 12 else ""),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return False, "Left alone -- nothing was stopped."
        return rad.fix_kill_orphans()

    def _fix_teensy_help(self):
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.information(self, "Attaching the master arm",
                                rad.teensy_help_text())
        return True, "Shown the steps for attaching the master arm."

    def _fix_teensy_repoint(self):
        """Point the running reader at the socket the board is actually on.

        The port is not a live parameter -- the node opens the device at
        startup -- so this stops the reader and starts it again on the right
        one. Stated plainly on the button rather than pretending a parameter
        write would work.
        """
        cands = rad.Probe().tty_candidates() or []
        if not cands:
            return (False, "The board is not plugged in, so there is nothing "
                           "to point at.")
        from srl_teleop import serial_port
        try:
            port = serial_port.find_port()
        except Exception:                                     # noqa: BLE001
            port = cands[0]
        procscan.kill_all(r"lib/srl_teleop/master_pose_node", 2)
        argv = ["ros2", "run", "srl_teleop", "master_pose_node",
                "--ros-args", "-p", "serial_port:=%s" % port]
        try:
            p = subprocess.Popen(argv, env=dict(os.environ),
                                 start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.STDOUT)
        except Exception as e:                                # noqa: BLE001
            return False, "Could not restart the master arm reader: %r" % (e,)
        self.jobs.append(("master arm reader on %s" % port, p))
        return True, "Master arm reader restarted on %s." % port

    def _fix_home_arms(self):
        """Ask the homing service to drive the real arms to the stored pose.

        NEVER changes a stored value, and never commands the sim to match the
        arm: home joint angles are ground truth and the REAL arm's are the
        ones that count (HARD CONSTRAINT 1). If the gap is the 2026-08-15
        home change, this will not close it, which is the correct answer --
        the fix for that is a recapture on the arms, not a move.
        """
        from PyQt5.QtWidgets import QMessageBox
        r = QMessageBox.question(
            self, "Move the real arms",
            "The real arms will move to the stored resting pose.\n\n"
            "Check that nobody is wearing the rig and that the space around "
            "the arms is clear.\n\nMove them now?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return False, "Not moved -- nothing was commanded."
        for a in ARMS:
            self.bus.submit(
                lambda a=a: self.bus.call_trigger("/home_arm_%s" % a),
                label="home the %s arm requested" % a)
        return True, ("Asked both arms to go to the resting pose. Watch the "
                      "ACTUAL view.")

    def _fix_release_kortex(self):
        """Hand the arm's one connection back, in process, without relaunching.

        A relaunch re-homes the arm and a participant session cannot absorb
        that, so the bridge has a service that closes the stale session and
        opens a fresh one on the spot. If no bridge is running there is
        nothing to ask, and this says so rather than pretending.
        """
        procs = rad.Probe().kortex_procs()
        if not procs:
            return (False, "Nothing is holding the connection from here. If "
                           "the arm still refuses, it is holding a session "
                           "from another machine and will time it out.")
        for a in ARMS:
            self.bus.submit(
                lambda a=a: self.bus.call_trigger(
                    "/real/session_recover_%s" % a),
                label="release the %s arm connection requested" % a)
        return True, "Asked the arm to drop the old connection and reopen."

    # -------------------------------------------------------- the checking
    def on_doctor_check(self, announce=False):
        """Start the seven checks on a worker. Returns immediately.

        WHY A WORKER. Run inline these froze the window for 16.7 s, measured
        by the button audit, which is right: the daemon check finds out that
        the answer is not coming by WAITING for it, and that wait IS the
        check. A window that carries the e-stop must not stop repainting for
        sixteen seconds to run a diagnostic.

        The joint state the home check needs is read from the snapshot the ROS
        thread already published, so the worker touches no subscription and
        takes no lock the ROS thread wants.
        """
        if self._doc_busy:
            if announce:
                self.log("connection check already running")
            return self.doc_checks
        # A PRESS THAT LEAVES NO TRACE IS A PRESS THAT CANNOT BE TOLD FROM A
        # DISCONNECTED SIGNAL. The answer arrives seconds later, so the press
        # itself says that it started.
        if announce:
            self.log("checking the connection to the real arms...")
        self._doc_busy = True
        snap = self.bus.snap
        fixes = self._doctor_fixes()

        class GuiProbe(rad.Probe):
            def joint_states(self_inner):
                return (snap.get("sim_pos") or None,
                        snap.get("real_pos") or None)

        def work():
            out = rad.run_all(GuiProbe(), fixes)
            with self._doc_lock:
                self._doc_result = out
        threading.Thread(target=work, daemon=True).start()
        return self.doc_checks

    def _doctor_collect(self):
        """Called from refresh(). Applies a finished diagnosis, if there is
        one. Never blocks and never waits."""
        if self.doctor_frozen:
            return
        with self._doc_lock:
            out, self._doc_result = self._doc_result, None
        if out is None:
            return
        self._doc_busy = False
        self.doc_checks = out
        state, head = rad.verdict(out)
        col = {rad.OK: C_OK, rad.BAD: C_BAD, rad.UNKNOWN: C_UNKNOWN}[state]
        self.doc_head.setText(head.upper())
        self.doc_head.setStyleSheet("color:%s" % col)
        self._doctor_render(out)
        # ONE LINE PER CHECK RUN, always. The event log is what an operator
        # reads afterwards to find out what the rig looked like at the time,
        # and "it said it was fine" is only worth something if the moment it
        # said so is recorded.
        self.log("connection check: %s (%s)"
                 % (head, ", ".join("%s=%s" % (c.key, c.state) for c in out)),
                 bad=(state == rad.BAD))

    def doctor_wait(self, timeout_s=40.0):
        """Block until a diagnosis lands, pumping the event loop. FOR TESTS.

        The GUI itself never calls this -- that is the entire point of the
        worker. The button audit does, because it has to walk a window whose
        rows are built from a diagnosis, and walking it before the first one
        arrives audits an empty panel and reports a pass. That is the same
        defect as the modal dialog which hid five buttons for months.
        """
        from PyQt5.QtWidgets import QApplication
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            QApplication.processEvents()
            self._doctor_collect()
            if self.doc_checks:
                return self.doc_checks
            time.sleep(0.05)
        return self.doc_checks

    def _doctor_render(self, checks):
        """Update the seven rows in place. No widget is created or destroyed."""
        self._doc_fix = {}
        for c in checks:
            row = self.doc_rows.get(c.key)
            if row is None:
                continue
            box, title, plain, fix = row
            col = {rad.OK: C_TEXT, rad.BAD: C_BAD,
                   rad.UNKNOWN: C_UNKNOWN}[c.state]
            mark = {rad.OK: "OK", rad.BAD: "FIX", rad.UNKNOWN: "?"}[c.state]
            title.setText("%s  %s" % (mark, c.title))
            title.setFont(helvetica(10, c.state != rad.OK))
            title.setStyleSheet("color:%s" % col)
            plain.setText(c.plain)
            plain.setVisible(c.state != rad.OK)
            if c.has_fix:
                self._doc_fix[c.key] = c.fix
                fix.setText(c.fix_label)
                fix.setToolTip(c.fix_note or c.detail)
                fix.setVisible(True)
            else:
                # HIDDEN AND UNBOUND. A button left visible with no repair
                # behind it is the "feature present but does nothing" row of
                # CLAUDE.md's instrument table.
                fix.setVisible(False)
                fix.setText("")
            box.setToolTip(c.detail)

    def _ensure_specs(self):
        """ONE manifest, whichever panel is built first.

        The launch groups are split across two tabs now, and both need the
        validated manifest. Validating it twice would double every
        subprocess `validate()` runs, and building it in whichever panel
        happens to be constructed first is exactly the kind of ordering
        dependence that breaks silently when a tab is reordered.
        """
        if getattr(self, "specs", None) is None:
            self.specs = gls.all_specs()
            gls.validate(self.specs)
            self.buttons = {}
        return self.specs

    def _spec_group(self, layout, group, title):
        items = [x for x in self._ensure_specs() if x.group == group]
        lab = QLabel(title)
        lab.setFont(helvetica(10, True))
        layout.addWidget(lab)
        grid = QGridLayout()
        for i, sp in enumerate(items):
            b = QPushButton(sp.label)
            b.setFont(helvetica(9))
            b.setMinimumWidth(0)
            b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
            if sp.enabled:
                b.setToolTip(sp.note)
                b.clicked.connect(lambda _, x=sp: self.on_launch(x))
            else:
                # DISABLED WITH THE REASON ON IT. A button that exits 2 on
                # press looks exactly like one that launched something
                # invisible; a greyed button with a sentence does not.
                b.setEnabled(False)
                b.setToolTip(sp.disabled_reason)
                b.setText(sp.label + "  (unavailable)")
            grid.addWidget(b, i, 0)
            self.buttons[sp.key] = b
        layout.addLayout(grid)

    def _launchers(self):
        """EVERY GROUP THE MANIFEST DECLARES, NOT THREE OF THE FOUR.

        `demo` was missing once, so the fifteen dance specs had NO BUTTON AT
        ALL and the three routines could not be launched from this GUI --
        which is G-1 and G-2, "launch every mode" and "run every task",
        quietly unmet. They were filmable only because `record_abc_sweep`
        calls `Gui.on_launch(spec)` directly rather than pressing anything.
        Found by `verify_gui_buttons`, whose "every enabled launch button
        pressed" check read 50 of 65 and named the fifteen. A manifest entry
        with no button is the same defect as a button with no manifest entry
        and is harder to see.
        """
        g = QGroupBox("Start a mode, run a task")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        for group, title in (("mode", "Modes"), ("task", "Tasks"),
                             ("demo", "Demonstrations (no trial data)")):
            self._spec_group(v, group, title)
        b = QPushButton("stop all launched jobs")
        b.clicked.connect(self.on_stop_jobs)
        v.addWidget(b)
        return g

    def _diag_launchers(self):
        g = QGroupBox("Checks and repairs")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        self._spec_group(v, "diag", "Diagnostics")

        # DEPENDENCIES. Not "is numpy installed" -- every environment has
        # numpy -- but "do these modules still import TOGETHER in the venv
        # the calibration scripts run in". On 2026-08-22 installing lerobot
        # into .venv_vision pulled numpy>=2, broke the system scipy that
        # .venv_vision borrows, and took check_all.py from 4 of 4 passing to
        # 2 of 4. The pip warning scrolled past and the only way to find out
        # was to run the gate.
        lab = QLabel("Dependencies")
        lab.setFont(helvetica(10, True))
        v.addWidget(lab)
        b2 = QPushButton("HOW GOOD IS THE SYSTEM?")
        b2.setFont(helvetica(9, True))
        b2.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        b2.setToolTip(
            "The control budget: where the hand ends up against the 30 mm "
            "grasp gate, how far the wearer moves before the guard hears "
            "about it, what losing tracking costs, and how free the arms "
            "are. Reads the recorded baselines -- it measures nothing new "
            "and prints UNMEASURED where nothing has been measured.")
        b2.clicked.connect(self.on_control_budget)
        v.addWidget(b2)
        self.budget_lbl = QLabel("not checked yet")
        self.budget_lbl.setWordWrap(True)
        self.budget_lbl.setFont(mono(8))
        self.budget_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.budget_lbl)

        b = QPushButton("CHECK DEPENDENCIES")
        b.setFont(helvetica(9, True))
        b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        b.setToolTip("Imports each environment's modules together in one "
                     "interpreter, checks the command-line tools are on "
                     "PATH, and checks every srl_* package is SYMLINK "
                     "installed -- a copy install means .py edits do not "
                     "take effect and it looks like a stale process.")
        b.clicked.connect(self.on_check_dependencies)
        v.addWidget(b)
        self.dep_lbl = QLabel("not checked yet")
        self.dep_lbl.setWordWrap(True)
        self.dep_lbl.setFont(mono(8))
        self.dep_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.dep_lbl)
        return g

    def on_control_budget(self):
        """The honest answer to 'how good is this', from the baselines."""
        def go():
            try:
                sys.path.insert(0, os.path.join(_WS, "scripts"))
                import measure_control_budget as MCB
                rep = MCB.report(verbose=False)
            except Exception as e:                            # noqa: BLE001
                self._budget_say("could not compute the budget: %r" % (e,),
                                 bad=True)
                return
            unm = [r["term"] for r in rep["positioning_mm"]
                   if r["mm"] is None]
            over = rep["positioning_worst_mm"] > rep["grasp_gate_m"] * 1000
            fast = [r for r in rep.get("reaction", [])
                    if r.get("exceeds_floor")]
            pg = rep.get("posture_gap") or {}
            breach = sorted({p for a in pg.values()
                             if isinstance(a, dict)
                             for p in a.get("breaches_floor", [])})
            lines = [
                "WHERE THE HAND LANDS: %.1f mm worst case against a %.0f mm "
                "gate%s"
                % (rep["positioning_worst_mm"], rep["grasp_gate_m"] * 1000,
                   "  -- OVER BUDGET" if over else ""),
                "  and %d term(s) UNMEASURED: %s"
                % (len(unm), ", ".join(unm)) if unm else "",
                "THE WEARER: %d of %d limb motions travel further than the "
                "whole %.0f mm floor before the guard hears about it"
                % (len(fast), len(rep.get("reaction", [])),
                   rep["floor_m"] * 1000),
                "LOSING TRACKING: posture(s) that BREACH the floor while the "
                "fallback assumes clearance: %s"
                % (", ".join(breach) if breach else "none"),
            ]
            self._budget_say("\n".join(x for x in lines if x),
                             bad=bool(over or fast or breach))

        self._budget_say("composing the budget from the baselines...")
        self.bus.submit(go, label="control budget")

    def _budget_say(self, text, bad=False):
        lbl = getattr(self, "budget_lbl", None)
        if lbl is not None:
            lbl.setText(text)
            lbl.setStyleSheet("color:%s" % (C_BAD if bad else C_TEXT))
        self.bus.note("budget: %s" % text.replace("\n", " | ")[:170],
                      bad=bad)

    def on_check_dependencies(self):
        """Run it OFF the Qt thread -- it spawns four interpreters."""
        self.dep_lbl.setText("checking four environments; this takes a few "
                             "seconds...")
        self.dep_lbl.setStyleSheet("color:%s" % C_MUTED)

        def go():
            try:
                from srl_teleop import dependency_check as dc
                rep = dc.report()
            except Exception as e:                            # noqa: BLE001
                self._dep_rows = None
                self.bus.note("dependency check FAILED to run: %r" % (e,),
                              bad=True)
                return
            self._dep_rows = rep
            bad = [r for g_ in ("environments", "tools", "packages")
                   for r in rep[g_] if r["state"] != "ok"]
            self.bus.note(
                "dependencies: %s"
                % ("all four environments import cleanly, every tool is on "
                   "PATH, every srl_* package is symlink-installed"
                   if not bad else
                   "; ".join("%s: %s" % (r["env"], r["detail"][:60])
                             for r in bad)),
                bad=bool([r for r in bad if r["state"] == "bad"]))

        self.bus.submit(go, label="dependency check")
        QTimer.singleShot(1200, self._dep_render)

    def _dep_render(self, tries=40):
        rep = getattr(self, "_dep_rows", None)
        if rep is None:
            if tries > 0:
                QTimer.singleShot(500,
                                  lambda: self._dep_render(tries - 1))
                return
            self.dep_lbl.setText("the dependency check did not finish -- see "
                                 "the event log")
            self.dep_lbl.setStyleSheet("color:%s" % C_BAD)
            return
        lines = []
        worst = "ok"
        for grp in ("environments", "tools", "packages"):
            for r in rep[grp]:
                if r["state"] == "ok":
                    continue
                worst = "bad" if r["state"] == "bad" else worst
                lines.append("%s  %s -- %s%s"
                             % (r["state"].upper(), r["env"],
                                r["detail"][:70],
                                ("\n      fix: " + r["fix"][:70])
                                if r["fix"] else ""))
        if not lines:
            n = sum(len(rep[g_]) for g_ in
                    ("environments", "tools", "packages"))
            self.dep_lbl.setText(
                "all %d checks pass: 4 environments import cleanly, 6 tools "
                "on PATH, 6 srl_* packages symlink-installed" % n)
            self.dep_lbl.setStyleSheet("color:%s" % C_OK)
        else:
            self.dep_lbl.setText("\n".join(lines))
            self.dep_lbl.setStyleSheet(
                "color:%s" % (C_BAD if worst == "bad" else C_WARN))
        self._dep_rows = None

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
        # 470, up from 430. The Wearer tab is a picture PLUS a nine-row
        # table, and at 430 the table showed three rows -- so the panel whose
        # job is "which parts are measured and why not" answered the question
        # for a third of the body without scrolling.
        tabs.setMaximumHeight(470)

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
            # BOTH TAB BARS, not just this one. The left column gained three
            # activity tabs, and `--tab` -- which exists so a capture can be
            # pointed at a panel -- could not reach them, so the one panel a
            # screenshot most needs to show (the connection checks) could not
            # be screenshotted at all.
            def _pick():
                for bar in (self._tabs, getattr(self, "act_tabs", None)):
                    if bar is None:
                        continue
                    for i in range(bar.count()):
                        if want.lower() in bar.tabText(i).lower():
                            bar.setCurrentIndex(i)
                            return
            QTimer.singleShot(900, _pick)

        self.eventlog = QTextEdit()
        self.eventlog.setReadOnly(True)
        self.eventlog.setFont(mono(8))
        self.eventlog.setStyleSheet("color:%s;border:none" % C_MUTED)
        # THE WEARER TAB. It goes beside Instruct rather than in the left
        # column because it is a VIEW -- a picture with things drawn on it --
        # and the left column is controls and indicators. It is also the one
        # panel whose top line is a safety state rather than a diagnosis:
        # which body the robot is planning against.
        self.wearer_panel = swp.WearerPanel(
            dict(bg=C_BG, line=LINE, muted=C_MUTED, accent=C_OK, bad=C_BAD,
                 warn=C_WARN, unknown=C_UNKNOWN),
            on_recalibrate=self.on_scene_recalibrate)
        self.wearer_panel.set_advice_sink(lambda t: self.log("wearer: " + t))
        tabs.addTab(self._scroll(self.wearer_panel), "Wearer")
        self.mid_tabs = tabs
        self.instruct_tab_index = tabs.addTab(self._instruct_tab(),
                                              "Instruct")
        # Reaching the panel ANY way -- the OPERATE button or clicking the tab
        # -- must put the keyboard in the box. Otherwise the two routes to the
        # same panel behave differently, and the one people actually use is
        # the tab.
        tabs.currentChanged.connect(
            lambda i: self.focus_instruct()
            if i == self.instruct_tab_index else None)
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
        # CHANGING WHICH STAGE INVALIDATES WHAT WAS SEEN, AND SAYS SO.
        #
        # Found by the button audit, which presses every control and requires
        # a trace from each: this drop-down changed what CONFIRM would run and
        # left no mark anywhere. The plan on screen stayed up, still showing
        # the other stage's cubes, and the only thing standing between that
        # and a run was the seed check inside `_inst_load_detections` -- which
        # fires at plan time, i.e. after the operator has read a plan for the
        # wrong table and believed it.
        #
        # The two stages are different tables: stage 2 draws its cubes from a
        # seed. So switching clears the plan rather than warning about it.
        self.inst_task.currentTextChanged.connect(self._inst_task_changed)
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

    def _inst_task_changed(self, text):
        self._inst_pending = None
        self._inst_outcome = None
        self._inst_seen = []
        self.inst_go.setEnabled(False)
        self.inst_intent.setText("--")
        self._inst_set_state("IDLE")
        self.log("instruct: task is now %s" % text)
        self._inst_say("task changed to %s. The two stages are different "
                       "tables, so anything the camera saw before does not "
                       "describe this one -- press LOOK again." % text)

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
                    "COMMANDED  --  the sim, driven by the active mode")])
        for key, title in panels:
            box = QGroupBox(title)
            box.setFont(helvetica(11, True))
            bl = QVBoxLayout(box)
            bl.setContentsMargins(2, 2, 2, 2)
            host = QWidget()
            # 600 wide squeezed the ACTUAL panel beside it to a strip. RViz
            # renders fine at 420 and the second view needs to be readable,
            # which is the whole reason it is there.
            host.setMinimumSize(420, 420)
            hl = QVBoxLayout(host)
            hl.setContentsMargins(0, 0, 0, 0)
            msg = QLabel("starting RViz...")
            msg.setAlignment(Qt.AlignCenter)
            msg.setFont(helvetica(12))
            hl.addWidget(msg)
            bl.addWidget(host)
            self.viz_host[key], self.viz_msg[key] = host, msg
            self.viz_split.addWidget(box)
        # ------------------------------------------------ THE SECOND VIEW
        # The brief asks for two views side by side: the sim, and the REAL
        # arms' actual positions. The second one is drawn here rather than by
        # a second RViz, and that is a measurement and not a preference --
        # two embedded RViz windows are not clipped to their containers on
        # this display stack and paint over each other and over every
        # indicator (screenshotted; docs/system/07_gui_rviz_embedding.md).
        # This panel also survives an RViz crash, which matters because RViz
        # is the thing most likely to take the tool down.
        self.viz_split.addWidget(self._actual_panel())
        self.viz_split.setStretchFactor(0, 3)
        self.viz_split.setStretchFactor(self.viz_split.count() - 1, 2)
        self.viz_split.setSizes([560, 360])
        v.addWidget(self.viz_split, 1)
        v.addWidget(self._divergence_panel())
        return w

    def _actual_panel(self):
        box = QGroupBox("ACTUAL  --  where the real arms are")
        box.setFont(helvetica(11, True))
        bl = QVBoxLayout(box)
        bl.setContentsMargins(4, 2, 4, 4)
        self.arm_view = av.ArmView(dict(bg=C_BG, line=LINE, muted=C_MUTED,
                                        accent=C_OK, bad=C_BAD,
                                        unknown=C_UNKNOWN))
        self.arm_view.setMouseTracking(False)
        bl.addWidget(self.arm_view, 1)
        row = QHBoxLayout()
        self.view_pick = QComboBox()
        for name in av.VIEWS:
            self.view_pick.addItem(name)
        self.view_pick.setCurrentText(av.DEFAULT_VIEW)
        self.view_pick.currentTextChanged.connect(self.arm_view.set_view)
        self.view_pick.currentTextChanged.connect(
            lambda t: self.log("actual view: %s" % t))
        row.addWidget(self.view_pick, 1)
        # A VIEW THAT CAN BE DRAGGED NEEDS A WAY BACK. One stray drag would
        # otherwise leave the panel at an angle nobody chose, and "it looks
        # wrong" is not a bug report anybody can act on.
        b = QPushButton("reset 3-D view")
        b.setFont(helvetica(9))
        b.setToolTip("Back to the shipped isometric angles (yaw 35, pitch "
                     "22). Drag inside the panel to orbit; the axis-aligned "
                     "FRONT / SIDE / TOP views do not orbit, because being "
                     "able to read a coordinate off them is the point.")
        b.clicked.connect(lambda: (
            self.arm_view.reset_view(),
            self.log("actual view: back to the shipped 3-D angles "
                     "(yaw %.0f, pitch %.0f)"
                     % (av.DEFAULT_YAW_DEG, av.DEFAULT_PITCH_DEG))))
        row.addWidget(b)
        bl.addLayout(row)
        return box

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
        """Wrapper: a failure here is SHOWN, never swallowed by Qt."""
        try:
            return self._start_rviz()
        except Exception as e:                                # noqa: BLE001
            for key, msg in self.viz_msg.items():
                msg.setText("RViz could not start:\n%s\n\nEverything else "
                            "in this window still works, including the "
                            "ACTUAL view beside this one." % (e,))
                msg.setStyleSheet("color:%s" % C_BAD)
            self.bus.note("RViz could not start: %r" % (e,), bad=True)
            return None


    def _rviz_log(self, key):
        """Where THIS RViz's stdout and stderr go.

        NOT /dev/null, which is where they went until 2026-08-22 and which
        is how an RViz that started and immediately died presented as an
        empty panel with no reason anywhere. Measured that day: the process
        was a zombie, the panel said nothing, and reproducing the failure by
        hand was the only way to see the message -- which is precisely the
        log-hunting this window exists to abolish.
        """
        path = os.path.join(_scratch(), "rviz_%s.log" % key)
        try:
            return path, open(path, "wb")
        except OSError:
            return None, subprocess.DEVNULL

    def _rviz_died(self, key):
        """(exitcode, tail) if this panel's RViz is gone, else (None, '')."""
        for k, p in self.rviz:
            if k != key:
                continue
            rc = p.poll()
            if rc is None:
                return None, ""
            tail = ""
            path = os.path.join(_scratch(), "rviz_%s.log" % key)
            try:
                with open(path, "r", errors="replace") as f:
                    lines = [ln.rstrip() for ln in f if ln.strip()]
                tail = "\n".join(lines[-6:])
            except OSError:
                pass
            return rc, tail
        return None, ""

    def _start_rviz(self):
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

        THAT MEASUREMENT WAS ON A HOST WITH NO WINDOW MANAGER, and the
        conclusion was written as if it were a property of the technique. It
        is not: it is a property of the display. Re-measured 2026-08-20 on
        this box's real display (WSLg, which runs one), `--embed-rviz` puts
        RViz INSIDE the commanded panel and clips it -- confirmed from the
        geometry dump, which reads

            overlay      1005..1554     the embedded RViz
            actual_arms  1566..1913     the second view, beside it
            divergence   1002..1918     underneath both

        side by side and not overlapping, which is exactly what the brief
        asks for. So embedding is now the DEFAULT WHERE A WINDOW MANAGER IS
        ACTUALLY PRESENT -- detected, not assumed, and not guessed from the
        platform name: `_have_wm()` asks the X root window whether anything
        claims to be managing it. On Xvfb, where nothing does, RViz still
        opens as its own window and the panel says so.
        """
        if self.args.no_rviz:
            for k in self.viz_msg:
                self.viz_msg[k].setText("RViz disabled (--no-rviz)")
            return
        embed = getattr(self.args, "embed_rviz", False)
        if not embed and not getattr(self.args, "no_embed_rviz", False):
            embed = _have_wm()
            if embed:
                self.bus.note("a window manager is running, so RViz will be "
                              "embedded in the panel beside the actual arms")
        if not embed:
            key = list(self.viz_msg.keys())[0]
            cfg = self._rviz_config(key)
            logpath, sink = self._rviz_log(key)
            try:
                p = subprocess.Popen(["rviz2", "-d", cfg],
                                     stdout=sink, stderr=subprocess.STDOUT,
                                     preexec_fn=_die_with_parent)
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
        logpath, sink = self._rviz_log(key)
        try:
            p = subprocess.Popen(argv, stdout=sink, stderr=subprocess.STDOUT,
                                 preexec_fn=_die_with_parent)
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
            # ASK WHETHER IT IS STILL ALIVE BEFORE BLAMING X11.
            #
            # This branch used to say "RViz did not present an X window in
            # 45 s -- embedding needs an X11 session" whatever had happened.
            # On 2026-08-22 the process had exited within a second on a
            # working XWayland display, so the message was false, specific
            # and confident, and it sent the reader to the display stack
            # instead of to the two lines RViz had printed. Those lines were
            # going to /dev/null; now they go to a file and land here.
            rc, tail = self._rviz_died(key)
            if rc is not None:
                self.viz_msg[key].setText(
                    "RViz STARTED AND EXITED (code %d) after %.1f s.\n"
                    "It is not an embedding problem -- the process is gone.\n"
                    "%s\n\nFull output: %s"
                    % (rc, self._tries * 1.5,
                       tail or "(it printed nothing)",
                       os.path.join(_scratch(), "rviz_%s.log" % key)))
                self.viz_msg[key].setStyleSheet("color:%s" % C_BAD)
                self.bus.note("RViz for %s exited with code %d; see %s"
                              % (key, rc,
                                 os.path.join(_scratch(), "rviz_%s.log" % key)),
                              bad=True)
                self._pending.pop(0)
                QTimer.singleShot(300, self._launch_next_rviz)
                return
            if self._tries > 30:
                # Still running after 45 s and still no window: now it really
                # is the display.
                self.viz_msg[key].setText(
                    "RViz is RUNNING but has not presented an X window in "
                    "45 s.\nEmbedding needs an X11 (or XWayland) session.\n"
                    "Run rviz2 separately; every indicator here still works.")
                self._pending.pop(0)
                QTimer.singleShot(300, self._launch_next_rviz)
                return
            QTimer.singleShot(1500, self._try_embed)
            return
        # THE LARGEST candidate, not the lowest id. Lowest-id picked the
        # selection-owner helper for months. Largest is the viewport, and
        # `_rviz_windows` has already thrown out everything without an rviz2
        # WM_CLASS or smaller than 200x200.
        sized = []
        for w_ in new:
            g = self._window_geometry(w_)
            if g:
                sized.append((g[0] * g[1], w_, g))
        if not sized:
            QTimer.singleShot(1500, self._try_embed)
            return
        sized.sort(reverse=True)
        area, wid, geom = sized[0]
        # AND WAIT FOR IT TO STOP CHANGING SIZE. RViz lays its docks out
        # after the window exists, so a window caught mid-construction is
        # the wrong size and reparenting it then gives a panel that never
        # fills. Two consecutive polls at the same size is enough.
        last = getattr(self, "_embed_last_geom", {}).get(wid)
        self._embed_last_geom = getattr(self, "_embed_last_geom", {})
        self._embed_last_geom[wid] = (geom[0], geom[1])
        if last != (geom[0], geom[1]):
            QTimer.singleShot(1500, self._try_embed)
            return
        # AND CHECK IT, because the last version of this code could not tell
        # a viewport from a 549x819 invisible helper and reported success.
        if not geom[2] or geom[0] < 200 or geom[1] < 200:
            self.viz_msg[key].setText(
                "RViz's window is %dx%d and %s -- that is not something to "
                "draw in.\nRefusing to embed it: an embedded invisible "
                "window looks exactly\nlike a working panel showing nothing."
                % (geom[0], geom[1],
                   "viewable" if geom[2] else "NOT viewable"))
            self.viz_msg[key].setStyleSheet("color:%s" % C_BAD)
            self._pending.pop(0)
            QTimer.singleShot(300, self._launch_next_rviz)
            return
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
        # THE CONTAINER MUST NOT TAKE THE KEYBOARD.
        #
        # A reparented foreign window is a real X client. By default the
        # container is in the focus chain, and once RViz holds the X input
        # focus every key the operator presses goes to RViz -- so the Instruct
        # box, which is the entire interface to full autonomy, silently
        # accepts nothing. The window looks alive and typing does nothing.
        #
        # NoFocus keeps the container out of the tab order and stops it
        # claiming focus on click; RViz still receives mouse events, which is
        # all it needs for orbit and zoom.
        container.setFocusPolicy(Qt.NoFocus)
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
        # SAY SO. Embedding either worked or it did not, and until now the
        # only evidence either way was a screenshot -- which on this host
        # records black, so on the display the operator actually uses there
        # was no evidence at all.
        self.bus.note("embedded RViz into the %s panel (X window 0x%x, "
                      "%dx%d, viewable) -- chosen from %d candidate(s) by "
                      "area, having discarded every window without an rviz2 "
                      "WM_CLASS"
                      % (key.upper(), wid, geom[0], geom[1], len(sized)))
        self._pending.pop(0)
        # Re-assert the split AFTER embedding, because the container is what
        # disturbed it.
        # SAME NUMBERS AS THE CONSTRUCTOR. These were the OLD split (336 /
        # 1090), so embedding silently undid the widening that stopped the
        # left column slicing its own button labels in half.
        lw = self._left_width()
        self.split.setSizes([lw, 620, max(420, self.width() - lw - 620)])
        # Embedding re-asserts the splitter, which is exactly how the last
        # widening was silently undone. Re-fit after it -- but LATER, not
        # 250 ms later. The foreign window has only just been adopted and
        # resizing the splitter while that is settling is what put RViz at
        # the corner of the screen instead of in the panel.
        QTimer.singleShot(2500, self._fit_left_column)
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
                            ("actual_rviz", self.viz_host.get("actual")),
                            ("overlay", self.viz_host.get("overlay")),
                            # THE SECOND VIEW. It was not in this dump, so
                            # the pixel proof had no region for the one panel
                            # that was added to satisfy "two views side by
                            # side" -- a check with nothing to crop cannot
                            # fail, which is the shape CLAUDE.md's standing
                            # rule is about.
                            ("actual_arms", getattr(self, "arm_view", None)),
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
        """RViz's REAL top-level windows. Not everything with 'rviz' in it.

        THE DEFECT THIS REPLACES, measured 2026-08-22 on WSLg. This matched
        any line of `xwininfo -root -tree` containing "rviz", and rviz2
        creates several X windows besides the one it draws in:

            0x600106 "....rviz - RViz": ("rviz2" "rviz2")  1600x1000   <- real
            0x600004 "Qt Selection Owner for rviz2": ()     549x819    <- not
            0x600008 "rviz2": ()                            1x1        <- not
            0xa00004 (has no name): ()                    1568x914     <- child

        The caller then took `sorted(new)[0]`, the LOWEST id, which is the
        selection owner. So the GUI reparented an invisible helper window
        into the commanded panel, the geometry dump measured the container
        and read 1005..1554 exactly as designed, every check passed, and the
        panel showed nothing while the real RViz sat in its own window on
        top of the GUI. CLAUDE.md recorded embedding as working on the
        strength of it.

        That is this repository's own listed failure mode -- "feature present
        but does nothing", and "everything matches: substring matching" -- in
        the one place where the evidence is a picture nobody was looking at.

        Three filters, and a window must pass all three:
          * a WM_CLASS of rviz2. The helpers have none, which is the
            cleanest single discriminator.
          * a title that is not one of Qt's internal ones.
          * a size worth drawing in. A 1x1 window is not a viewport.
        """
        try:
            o = subprocess.run(["xwininfo", "-root", "-tree"],
                               capture_output=True, text=True,
                               timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return set()
        ids = set()
        # 0x600106 "title": ("rviz2" "rviz2")  1600x1000+38+59  +135+45
        pat = re.compile(
            r'(0x[0-9a-f]+)\s+(?:"(?P<title>[^"]*)"|\(has no name\))'
            r'\s*:\s*\((?P<cls>[^)]*)\)'
            r'(?:\s+(?P<w>\d+)x(?P<h>\d+))?')
        for line in o.splitlines():
            m = pat.search(line)
            if not m:
                continue
            cls = (m.group("cls") or "").lower()
            if "rviz" not in cls:
                continue                       # helpers carry no WM_CLASS
            title = m.group("title") or ""
            if title.startswith("Qt Selection Owner"):
                continue
            # THE MAIN WINDOW, BY TITLE. rviz2 titles its main window
            # "<config path> - RViz"; the transient windows it makes while
            # starting up do not. Measured 2026-08-22: embedding whatever
            # appeared first at 1.5 s caught a 400x287 startup window, and
            # RViz then built its real 1600x1000 one as a SEPARATE toplevel
            # -- so the panel held a stub and the real view sat outside.
            if not title.endswith("- RViz"):
                continue
            w = int(m.group("w") or 0)
            h = int(m.group("h") or 0)
            if w < 200 or h < 200:
                continue                       # 1x1 stubs are not viewports
            ids.add(int(m.group(1), 16))
        return ids

    def _window_geometry(self, wid):
        """(w, h, mapped) for one X window, or None. Used to CHECK what was
        embedded rather than to assume it."""
        try:
            o = subprocess.run(["xwininfo", "-id", hex(wid)],
                               capture_output=True, text=True,
                               timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return None
        w = re.search(r"Width:\s+(\d+)", o)
        h = re.search(r"Height:\s+(\d+)", o)
        m = re.search(r"Map State:\s+(\S+)", o)
        if not (w and h):
            return None
        return (int(w.group(1)), int(h.group(1)),
                (m.group(1) if m else "") == "IsViewable")

    # ------------------------------------------------------------- actions
    # ------------------------------------------------------------- actions
    def on_estop(self):
        self.bus.submit(lambda: (self.bus.publish_once(Bool, "/estop", True),
                                 self.bus.note("E-STOP published")))

    def on_estop_reset(self):
        # /estop_reset IS A SERVICE, NOT A TOPIC. Publishing a Bool at it once
        # left the e-stop latched and made six subsequent fault injections
        # report NOT HANDLED for that one reason.
        self.bus.submit(lambda: self.bus.call_trigger("/estop_reset"),
                        label="e-stop RESET requested")

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
        self.bus.submit(act, label="precision/speed dial -> %s" % d)
        for a in ARMS:
            self.scale[a].setValue(int(round(st["scale"] * 100)))

    def on_scale(self, arm, v):
        self.bus.submit(lambda: self.bus.set_param(
            "/master_pose_node", "%s_scale" % arm, v),
            label="%s scale -> %.2f" % (arm, v))

    def on_force_clutch(self, state):
        on = bool(state)
        self.bus.submit(lambda: self.bus.set_param(
            "/master_pose_node", "force_clutch_engaged", on),
            label="force clutch engaged -> %s" % on)

    def on_release(self, arm):
        self.bus.submit(
            lambda: self.bus.call_trigger("/gripper_release_%s" % arm),
            label="release %s grip requested" % arm)

    def on_launch(self, spec):
        """Rewrite the spec for the session's settings, preflight it, spawn it.

        THE SPAWN IS A SEPARATE METHOD ON PURPOSE. `verify_gui_buttons` has to
        stop these actually starting stacks, and it used to do that by
        replacing `on_launch` with a stub that reimplemented the preflight and
        the refusal -- so everything the real `on_launch` does BEFORE the
        Popen was invisible to the audit, and stayed invisible when this
        method grew the motion-generator rewrite below. A stub that duplicates
        production logic drifts from it silently, which is the same shape as
        two home poses. The audit now replaces `_spawn` alone and the whole of
        this method runs for real.
        """
        spec = self._launch_spec(spec)
        fails = self._preflight(spec)
        if fails:
            self.bus.note("REFUSED %s: %s" % (spec.label, "; ".join(fails)),
                          bad=True)
            return
        self._spawn(spec)

    def _launch_spec(self, spec):
        """The spec as it will actually be run, with this session's settings.

        Today that is one setting: the selected motion generator, and only
        where the launch file DECLARES the argument. `ros2 launch` fails
        outright on an argument it does not know, so appending one to a stack
        that cannot take it would not degrade -- it would kill the button.
        """
        want = (self.motion_gen.currentData()
                if getattr(self, "motion_gen", None) is not None else "ruckig")
        if getattr(spec, "motion_generator", False) and want != "ruckig":
            spec = spec.with_argv(spec.argv + ["motion_generator:=%s" % want])
            self.bus.note("launching with motion_generator:=%s -- NOT the "
                          "default" % want, bad=True)
        return spec

    def _spawn(self, spec):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        # KEEP WHAT IT SAID. This was DEVNULL on both streams, so a launched
        # job that REFUSED -- naming its reason, as every refusal in this
        # repository is written to do -- produced a bare exit code and nothing
        # else. Measured the hard way on 2026-08-18: `run_abc` exited 2 inside
        # the recording sweep and ran perfectly outside it, and the sentence
        # explaining which of its two refusals had fired was being discarded
        # at this line. A day of bisection for one file descriptor.
        #
        # One file per launch, named for the spec, in the scratch directory.
        log = None
        try:
            log = open(os.path.join(_scratch(), "launch_%s.log" % spec.key),
                       "wb")
        except Exception:                                     # noqa: BLE001
            log = None
        try:
            p = subprocess.Popen(spec.argv, env=env, start_new_session=True,
                                 stdout=(log or subprocess.DEVNULL),
                                 stderr=subprocess.STDOUT)
        except Exception as e:                                # noqa: BLE001
            self.bus.note("LAUNCH FAILED %s: %r" % (spec.label, e), bad=True)
            return
        self.jobs.append((spec.label, p))
        self.bus.note("launched %s (pid %d)%s"
                      % (spec.label, p.pid,
                         "" if log is None
                         else " -> %s" % os.path.join(
                             _scratch(), "launch_%s.log" % spec.key)))
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
        # THE SAME FACT, BESIDE THE BUTTONS THAT START IT. Read from live
        # publishers, never from which button was last pressed: "I launched
        # it" and "it is driving the arm" are different, and the gap between
        # them is where a run goes wrong.
        lbl = getattr(self, "mode_live_lbl", None)
        if lbl is not None:
            if len(driving) > 1:
                lbl.setText("live mode: CONFLICT -- %s all claim the arm"
                            % "+".join(driving))
                lbl.setStyleSheet("color:%s" % C_BAD)
            elif driving:
                lbl.setText("live mode: %s  (from live publishers)"
                            % driving[0])
                lbl.setStyleSheet("color:%s" % C_OK)
            else:
                lbl.setText("live mode: nothing is driving the arms")
                lbl.setStyleSheet("color:%s" % C_MUTED)
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

        # ---- MOTION GENERATOR, per arm. Field [15] of /ik_status_<arm>:
        # 2 ruckig, 1 the synchronised fallback, 0 legacy clamp_towards.
        # An older follower publishes a shorter array and reads UNKNOWN --
        # never green, because "the field is missing" is not "it is fine".
        for a in ARMS:
            v = val("ik_%s" % a)
            k = "motion_%s" % a
            if not v:
                self.ind[k].set("--", C_UNKNOWN, "no ik_status published")
            elif len(v) < 18:
                self.ind[k].set("?", C_UNKNOWN,
                                "follower predates the generator")
            else:
                which = int(round(v[15]))
                self.ind[k].set(
                    {2: "ruckig", 1: "sync clamp", 0: "LEGACY"}.get(
                        which, "?"),
                    C_OK if which == 2 else C_WARN,
                    "%d resync(s), %d cycle(s) still travelling"
                    % (int(v[16]), int(v[17])))

        self.master_schema.set_data(s.get("master_schema"))
        self.robot_schema.set_data(s.get("robot_schema"))
        self._sync_embedded()
        self._doctor_collect()
        self._vr_collect()
        self._refresh_wearer(s)
        self._refresh_actual(s)
        self._refresh_divergence(s)
        self._refresh_cameras(s)
        self._refresh_say(s)

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

    def _refresh_say(self, s):
        """The robot's own sentence, live, in the window.

        AND IT SAYS WHEN IT HAS GONE QUIET. A banner that keeps showing the
        last thing it heard looks identical to one watching a live run, which
        is the "data fresh but never changes" row of the instrument table. The
        publisher stamps every message, so staleness is measurable rather than
        assumed.
        """
        if not hasattr(self, "say_lbl"):
            return
        raw = (s or {}).get("say")
        if not raw:
            self.say_lbl.setText("idle -- nothing is narrating")
            self.say_lbl.setStyleSheet(
                "color:%s;border:1px solid %s;padding:4px" % (C_MUTED, C_MUTED))
            return
        try:
            d = json.loads(raw)
        except Exception:                                     # noqa: BLE001
            self.say_lbl.setText(str(raw)[:200])
            return
        age = time.time() - float(d.get("t", 0.0))
        txt = str(d.get("text", ""))
        if age > 20.0:
            self.say_lbl.setText("%s   (%.0f s ago -- nothing since)"
                                 % (txt, age))
            self.say_lbl.setStyleSheet(
                "color:%s;border:1px solid %s;padding:4px" % (C_MUTED, C_MUTED))
        else:
            self.say_lbl.setText(txt)
            self.say_lbl.setStyleSheet(
                "color:%s;border:1px solid %s;padding:4px" % (C_OK, C_OK))

    def _refresh_wearer(self, s):
        """Push the newest scene frame and wearer estimate into the panel.

        Called every refresh with WHATEVER the snapshot holds, including
        nothing -- so the panel cannot keep showing a body after the tracker
        stops. That is the same rule the camera panels obey and it matters
        more here: a held body estimate is a collision model the operator
        believes while the person has walked away.
        """
        if not hasattr(self, "wearer_panel"):
            return

        def val(key):
            v = s.get(key)
            return v[0] if isinstance(v, tuple) else None
        self.wearer_panel.update_from(
            val("wearer"), val("wearer_overlay"),
            s.get("scene_img"), s.get("scene_img_t", 0.0), val("scene_cam"))

    def on_scene_recalibrate(self):
        """Re-solve the camera's position in the robot frame, from the marker.

        Runs the same script a person would run, in a subprocess, and puts its
        output in the event log. Deliberately NOT a reimplementation: a second
        copy of the extrinsic solve is a second copy that can disagree with
        the first, and the disagreement would be invisible.
        """
        argv = [sys.executable,
                os.path.join(_WS, "scripts",
                             "calibrate_scene_camera_extrinsics.py"),
                "--live", "--marker-on-mount"]
        self.log("re-checking the camera transform: %s" % " ".join(argv[1:]))
        self._run_raw("scene camera extrinsics", argv)

    def _refresh_actual(self, s):
        """Push the newest points into the ACTUAL panel, or clear it.

        CLEARING IS THE POINT. `set_pose` is called on every refresh with
        whatever the snapshot holds, including nothing -- so the panel cannot
        keep showing a pose after the arms stop reporting, which is the
        exact failure mode ('data fresh but never changes') that this GUI's
        camera panel already refuses to have.
        """
        if not hasattr(self, "arm_view"):
            return
        real, sim, note, state = s.get(
            "skel", ({a: av.Skeleton(a) for a in ARMS},
                     {a: av.Skeleton(a) for a in ARMS},
                     "nothing is reporting where the real arms are",
                     "unknown"))
        self.arm_view.set_pose(real, sim, note, state)

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
        # TERMINATE, THEN CHECK, THEN KILL. `terminate()` alone is a request:
        # it returned immediately, nothing waited, and an RViz busy rendering
        # could outlive the window that asked it to stop.
        for _, p in self.rviz:
            try:
                p.terminate()
            except Exception:                                 # noqa: BLE001
                pass
        deadline = time.time() + 3.0
        for _, p in self.rviz:
            try:
                p.wait(timeout=max(0.05, deadline - time.time()))
            except Exception:                                 # noqa: BLE001
                try:
                    p.kill()
                except Exception:                             # noqa: BLE001
                    pass
        self.on_stop_jobs()
        ev.accept()


# ===========================================================================
#  helpers
# ===========================================================================
def _die_with_parent():
    """Ask the kernel to SIGTERM this child when its parent dies.

    `closeEvent` already terminates the RViz children, and that covers
    exactly one exit path: the operator closing the window. Every other way
    the GUI ends -- a crash inside a Qt slot, a SIGKILL, the terminal going
    away -- leaves rviz2 running, holding a window and ~200 MB, and the next
    `start_gui.sh` then opens beside the corpse of the last one. Measured
    2026-08-22: three orphaned rviz2 processes after three restarts, and the
    GUI's own RSS readout counts them.

    PR_SET_PDEATHSIG is the only mechanism that survives SIGKILL of the
    parent, because the kernel does it rather than the parent. Failure to set
    it is not fatal: it means the old behaviour, not a dead GUI.
    """
    try:
        import ctypes
        import signal as _sig
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(
            1, _sig.SIGTERM, 0, 0, 0)          # 1 == PR_SET_PDEATHSIG
    except Exception:                                         # noqa: BLE001
        pass


def _scratch():
    """The scratch directory, CREATED. It was only read.

    With SRL_SCRATCH pointing at a directory that did not exist yet, writing
    the RViz config raised FileNotFoundError inside a Qt slot -- where PyQt
    prints a traceback to a stderr nobody reads and returns -- so RViz never
    started and the panel said "starting RViz..." for ever. That is the exact
    silent-failure shape this GUI is written against, and it was in the one
    helper too small to look at.
    """
    d = os.environ.get("SRL_SCRATCH") or "/tmp"
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        return "/tmp"
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


def _have_wm():
    """Is anything actually managing windows on this display?

    THE QUESTION THAT WAS NEVER ASKED. Embedding a foreign X window is only
    clipped to its container if a window manager is running, and this project
    concluded "embedding does not work here" from a host that had none --
    then carried that conclusion onto a display that does.

    Asked of the display itself: `_NET_SUPPORTING_WM_CHECK` is set on the
    root window by every EWMH-compliant window manager and by nothing else.
    Not inferred from the platform, the session type or whether WSLg is
    mentioned in an environment variable, all of which can be true with no
    window manager running.
    """
    try:
        out = subprocess.run(["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"],
                             capture_output=True, text=True, timeout=5)
    except Exception:                                         # noqa: BLE001
        return False
    return "window id" in (out.stdout or "")


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
        # EIGHTEEN FIELDS, because [15] is the motion generator and a
        # healthy stack publishes it. A short array reads UNKNOWN, which is
        # correct for an old follower and useless as a healthy fixture: the
        # indicator self-test requires healthy and abnormal to DIFFER, and it
        # caught this row reading the same in both.
        "ik_left": w([100.0, 100.0, 0, 0, 0, 0.35, 0, 0, 0.6, 0, 0.35, 0,
                      0, 0, 0, 2.0, 0, 0]),
        "ik_right": w([100.0, 100.0, 0, 0, 0, 0.35, 0, 0, 0.6, 0, 0.35, 0,
                       0, 0, 0, 2.0, 0, 0]),
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
        # [15] = 0 is the legacy per-joint clamp, which is a real abnormal
        # state and not a missing field: a follower launched to reproduce an
        # old recording and left that way. [16] counts resyncs -- a generator
        # fighting the controller.
        "ik_left": w([0.0] * 15 + [0.0, 0.0, 0.0]),
        "ik_right": w([10.0, 1.0, 0, 0, 0, 0.05, 0, 0, 0.6, 0, 0.05, 0,
                       0, 0, 0, 0.0, 41.0, 12.0]),
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
    ap.add_argument("--no-embed-rviz", action="store_true",
                    help="keep RViz as its own top-level window even where a "
                         "window manager would let it be embedded")
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
