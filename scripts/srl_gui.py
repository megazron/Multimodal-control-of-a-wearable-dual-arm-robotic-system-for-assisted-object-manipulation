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
import signal
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

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
import srl_named_poses as _POSES                             # noqa: E402
import srl_map_objects as _MAPOBJ                            # noqa: E402

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

from PyQt5.QtCore import QEvent, QObject, Qt, QTimer                          # noqa: E402
from PyQt5.QtGui import (QColor, QFont, QGuiApplication,   # noqa: E402
                         QImage, QPalette,
                         QPixmap, QWindow)
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox,  # noqa: E402
                             QDoubleSpinBox,
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

# THE WINDOW MUST FIT THE SCREEN IT IS ON. 1920x1060 was a GUESS, hard-coded,
# and on a 1920-wide display it put the window's right edge at 1926 -- six
# pixels past the glass, plus the frame. The window rendered perfectly the
# whole time (captured at 3.0 ms/frame, every panel drawn); it was simply
# never composited onto the desktop, and WSLg gave the operator a taskbar
# icon that did nothing when clicked. `x11grab` names the same fault out
# loud: BadMatch, which is what X says about a window that is not wholly
# on-screen.
#
# Kept as a pure function of four numbers so it can be checked against a
# known answer without a display -- the screen it must fit is exactly the
# thing a headless test does not have.
WINDOW_MARGIN_PX = 40
WINDOW_MIN = (960, 600)


def fit_to_screen(avail_w, avail_h, want_w=1920, want_h=1060,
                  margin_px=WINDOW_MARGIN_PX):
    """Largest (w, h) no bigger than `want` that leaves `margin_px` of screen.

    Never returns something larger than the available area, and never
    smaller than WINDOW_MIN -- a window clamped to nothing is not a fix.
    A non-positive or unknown screen falls back to the wanted size, because
    guessing small on a screen we cannot measure hides the whole left column.
    """
    if not avail_w or not avail_h or avail_w <= 0 or avail_h <= 0:
        return int(want_w), int(want_h)
    w = min(int(want_w), max(WINDOW_MIN[0], int(avail_w) - margin_px))
    h = min(int(want_h), max(WINDOW_MIN[1], int(avail_h) - margin_px))
    return w, h

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
                     # THE LINK ITSELF: which address the headset should be
                     # pointed at, which address it actually connected FROM,
                     # and whether frames are arriving. The window used to
                     # show none of that -- the URL was behind a button that
                     # printed it once, and the headset's own address was
                     # never reported at all, so "connected" was a count of
                     # anonymous sockets.
                     ("/vr/bridge_status", "vrbridge"),
                     ("/vr/mapper_left", "vrmap_left"),
                     ("/vr/mapper_right", "vrmap_right"),
                     # SHARED AUTONOMY, WHICH WAS INVISIBLE. The arbiter
                     # publishes everything needed to judge it -- the state,
                     # the REASON it is in that state, the distance to the
                     # grasp, the top object and its probability, whether the
                     # intent is ambiguous, and who owns position versus
                     # orientation -- and nothing displayed a byte of it. So
                     # "does shared autonomy work" could only be answered by
                     # watching the arm and guessing, which is not a test.
                     ("/autonomy/arbiter_left", "arb_left"),
                     ("/autonomy/arbiter_right", "arb_right"),
                     # WHY THE ARM IS NOT MOVING, in the window. The safety
                     # node has always published its freeze and the reason on
                     # /vr/safety; the window never read it, so a frozen
                     # mapper looked identical to a broken one -- the exact
                     # confusion the 2026-08-20 real session spent an hour on.
                     ("/vr/safety", "vrsafety"),
                     # The unified vision layer's per-camera verdict.
                     ("/perception/scene/objects", "scenecv"),
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
        self.create_subscription(
            JointState, "/joint_states",
            lambda m: (self._note_sim_js(m), self._on_sim_js(m)), 20)
        self.create_subscription(
            JointState, "/real/joint_states",
            lambda m: (self._note_real_arm(m), self._on_real_js(m)), 20)

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
        # The vision layer's ANNOTATED scene frame: boxes, labels, or the
        # refusal burned into the picture. Preferred over the raw frame in
        # the always-on scene view whenever it is fresh.
        self.scene_ov_img = None
        self.scene_ov_img_t = 0.0
        self.create_subscription(
            Image, "/perception/scene/overlay/scene_usb",
            self._on_scene_overlay, qos_profile_sensor_data)

        self.cam = {a: cr.ChannelState() for a in ARMS}
        self.cam_img = {a: None for a in ARMS}
        self.cam_topic = {a: None for a in ARMS}
        # THE PANEL COULD NEVER HAVE SHOWN A FRAME.
        #
        # It subscribed to `/<arm>_wrist_camera/image_raw` and
        # `/wrist_mounted_camera/<arm>/image`, and NOTHING IN THIS REPOSITORY
        # PUBLISHES EITHER -- checked: no `create_publisher` anywhere uses
        # those names. Every camera producer and consumer here uses
        # `/<arm>_camera/color/image_raw`: the mock, `env_probe`, the
        # calibration sweep, `verify_colour_vision`, `verify_scan_view`.
        #
        # So the operator's only view of the workspace has read "NO CAMERA --
        # no frame has ever arrived" for the life of the panel, and it was
        # telling the truth about a topic nobody was ever going to write to.
        # A subscriber on a name no publisher uses is the "feature present but
        # does nothing" row of the instrument table, in the one panel a remote
        # operator depends on.
        #
        # The repo's own name goes FIRST. The two old ones are kept because no
        # camera has ever been attached to this host and the real Kinova
        # driver's topic is therefore unverified -- if it turns out to use one
        # of them, this still works. Whichever one arrives is NAMED in the
        # panel, so "there is a picture" and "the picture came from where I
        # think" stay separate claims.
        # The vision layer's ANNOTATED gripper frame goes FIRST: when
        # scene_understanding_node runs, the operator sees the detections
        # on the picture, not beside it. _on_image keys freshness per
        # topic arrival, and the panel names which topic it is showing.
        for a in ARMS:
            for topic in ("/perception/scene/overlay/gripper_%s" % a,
                          "/%s_camera/color/image_raw" % a,
                          "/%s_wrist_camera/image_raw" % a,
                          "/wrist_mounted_camera/%s/image" % a):
                self.create_subscription(
                    Image, topic,
                    (lambda m, arm=a, t=topic: self._on_image(arm, m, t)),
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

    def _on_scene_overlay(self, msg):
        self.scene_ov_img = (msg.width, msg.height, bytes(msg.data),
                             msg.step, msg.encoding)
        self.scene_ov_img_t = time.monotonic()

    def _on_image(self, arm, msg, topic=None):
        self.cam[arm].on_frame(msg.width, msg.height, msg.encoding)
        # ANNOTATED FRAMES WIN WHILE FRESH. The vision overlay runs ~2 Hz
        # against a raw stream many times faster; last-writer-wins would
        # flash a detection for one frame and paint raw over it. While an
        # overlay arrived in the last 2.5 s, raw frames feed liveness (the
        # on_frame above) but not the picture.
        is_ov = bool(topic) and "/overlay/" in topic
        now = time.monotonic()
        if is_ov:
            self._cam_ov_t = getattr(self, "_cam_ov_t", {})
            self._cam_ov_t[arm] = now
        elif now - getattr(self, "_cam_ov_t", {}).get(arm, 0.0) < 2.5:
            return
        # WHICH topic it came from, so the panel can say. Four names are
        # subscribed and only two are ever published here.
        self.cam_topic[arm] = topic
        # Keep the RAW buffer; convert on the GUI thread only when it will
        # actually be painted, so ROS-thread time is not spent on frames the
        # GUI is about to discard as stale.
        self.cam_img[arm] = (msg.width, msg.height, msg.encoding,
                             bytes(msg.data), msg.step)

    def sim_js_age(self):
        """Seconds since /joint_states last arrived, or None if never.

        THIS IS WHAT THE REAL CASCADE ACTUALLY WAITS ON. `start_real.sh`
        step 1 of 6 is "checking the sim stack" and it refuses with
        "/joint_states is not publishing" -- so the thing to wait for is the
        TOPIC, not the processes.
        """
        t = getattr(self, "_sim_js_t", 0.0)
        return None if not t else (time.time() - t)

    def _note_sim_js(self, msg):
        self._sim_js_t = time.time()

    def _note_real_arm(self, msg):
        """Stamp each arm that appears in a /real/joint_states message."""
        if not hasattr(self, "_real_arm_t"):
            self._real_arm_t = {}
        if not hasattr(self, "_real_q"):
            self._real_q = {}
        self._real_q.update(dict(zip(msg.name, msg.position)))
        names = set(msg.name)
        for arm in ("left", "right"):
            if all("%s_joint_%d" % (arm, i) in names for i in range(1, 8)):
                self._real_arm_t[arm] = time.time()

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
        s["cam_topic"] = dict(self.cam_topic)
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
        s["scene_ov_img"] = self.scene_ov_img
        s["scene_ov_img_t"] = self.scene_ov_img_t
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

    def call_trigger(self, name, then=None):
        cli = self.create_client(Trigger, name)
        if not cli.service_is_ready():
            # NEVER wait_for_service HERE. Blocking the ROS thread on a
            # service that may not exist is the 4-second e-stop stall, and
            # this thread also carries the e-stop's own publish.
            self.note("%s: service not present -- nothing was sent" % name,
                      bad=True)
            return
        fut = cli.call_async(Trigger.Request())

        def done(f):
            self.note("%s -> %s" % (name, _res(f)))
            # AND HAND IT BACK. The event log is a scrolling list; a caller
            # that needs to say "this worked" or "this was REFUSED, here is
            # why" in its own panel cannot read it. Without this every
            # service button could only report that it had ASKED -- which is
            # how a panel comes to say ARMED over a refusal.
            if then is None:
                return
            try:
                r = f.result()
                then(bool(r.success), str(r.message))
            except Exception as e:                            # noqa: BLE001
                then(False, "error %r" % (e,))

        fut.add_done_callback(done)

    def real_arms_seen(self, max_age=2.0):
        """Which arms have published joint states RECENTLY. {arm: age_s}.

        KEYED ON ARRIVAL, and deliberately per-ARM rather than per-topic:
        both bridges publish to /real/joint_states and each message carries
        only its own arm's joints, so "the topic is alive" says nothing about
        whether a PARTICULAR arm is. This session read exactly that wrong --
        a single snapshot of the topic showed the right arm and reported the
        left as MISSING while the left bridge was running perfectly at
        25.7 Hz.
        """
        now = time.time()
        seen = getattr(self, "_real_arm_t", {})
        return {a: now - t for a, t in seen.items() if now - t < max_age}

    #: How often the held setpoint is re-sent, Hz. The bridge's watchdog is
    #: `watchdog_s` (0.5 s by default) and it commands ZERO SPEED when no
    #: target has arrived within it, so a setpoint sent once is a setpoint
    #: that stops being obeyed half a second later.
    HOLD_HZ = 20.0

    def send_joint_pose(self, arm, q, secs=5.0):
        """Command one arm to a joint vector AND KEEP HOLDING IT.

        THE BRIDGE HAS A WATCHDOG AND THIS DID NOT FEED IT.
        ---------------------------------------------------
        `kortex_highlevel_bridge` lists three things that command zero speed,
        and the first is "the watchdog, if no target arrives for
        watchdog_s" -- 0.5 s. This method published the trajectory three
        times and stopped, so the arm moved for half a second, the watchdog
        fired, and it stopped wherever it had got to. The operator's symptom
        was exact: HOME and PICK POSE had to be pressed over and over to keep
        the arm there, because each press bought 0.5 s of motion.

        `execute_pick_left.Executor` never had this problem -- it republishes
        `self._hold` every cycle at 20 Hz for exactly this reason. This now
        does the same: the target is latched per arm and re-sent by a
        background thread until a NEW target replaces it.

        THE HOLD IS A SETPOINT, NOT A PUSH. The bridge closes the error
        proportionally, so re-sending the same target is what "hold this
        pose" means to it; it is not repeated commanding of new motion.
        """
        if not hasattr(self, "_jt_pub"):
            self._jt_pub = {}
        real = "/real/%s_arm_controller/joint_trajectory" % arm
        sim = "/%s_arm_controller/joint_trajectory" % arm
        for topic in (real, sim):
            if topic not in self._jt_pub:
                self._jt_pub[topic] = self.create_publisher(
                    JointTrajectory, topic, 10)
                time.sleep(0.25)
        # WHEN THE CASCADE IS RUNNING, COMMAND THE SIMULATION -- NOT /real/.
        #
        # `sim_to_real_bridge` replays the SIM's joint states onto
        # /real/<arm>_arm_controller/joint_trajectory continuously. Publishing
        # there as well makes TWO WRITERS on one arm, each overriding the
        # other several times a second, and the arm shakes on every movement.
        # That is a genuinely dangerous failure and it is what "the arms are
        # shaking with every movement" was.
        #
        # So: cascade up -> drive the sim ONCE and let the cascade relay it.
        # No cascade (a bare bridge from CONNECT) -> drive /real/ and hold it,
        # because then nothing else is feeding the bridge's watchdog.
        cascade = self._cascade_running()
        # THE SIMULATION MUST NOT OUTRUN THE METAL IT IS DRIVING.
        #
        # THE DEFECT, reported 2026-08-30: going to the PICK POSE while the
        # relay was live tripped the e-stop. It is not a bug in the pose
        # button and it is not a flaky arm -- it is arithmetic.
        # `sim_to_real_bridge` compares the DELAYED simulation against the
        # real arm and trips when they differ by more than `lag_trip_rad`.
        # The sim reaches the target in `secs`, the real arm can only travel
        # at `max_vel_rad_s`, so on a long move the arm falls behind by the
        # whole distance and the monitor fires -- correctly. It is reporting
        # exactly what is true: the arm is not keeping up.
        #
        # The cure is to ask the simulation for a move the arm can actually
        # follow. With the relay live, stretch `secs` so the sim's own speed
        # stays under the relay's cap, with margin for the ramp. Without the
        # relay there is nothing downstream to outrun and the caller's own
        # timing is kept.
        if cascade:
            secs = max(secs, self._pose_secs_for(arm, q, secs))
        order = (sim, real) if cascade else (real, sim)
        pub = chosen = None
        for topic in order:
            if self._jt_pub[topic].get_subscription_count() >= 1:
                pub, chosen = self._jt_pub[topic], topic
                break
        if pub is None:
            return (False, None)
        # THE HOLD IS FOR THE REAL BRIDGE ONLY. NEVER THE SIMULATION.
        #
        # The bridge is a SETPOINT consumer with a 0.5 s watchdog, so it needs
        # feeding. A JointTrajectoryController -- which is what the bare
        # (simulated) topic is -- is a TRAJECTORY consumer: every message is a
        # NEW trajectory, so re-sending at 20 Hz makes it re-plan fifty times
        # a second and the arm shakes. Under SIM + REAL that shake is then
        # RELAYED to the metal by sim_to_real_bridge, which is how a fix for
        # "the pose will not hold" turned into "the arms shake on every
        # movement".
        #
        # The simulated controller holds its own goal, so it needs exactly one
        # message and nothing more.
        if chosen.startswith("/real/"):
            self._start_hold(arm, pub, list(map(float, q)), secs)
        else:
            # TAKE THE TOPIC FIRST, OR THIS DOES NOTHING.
            #
            # THE DEFECT, 2026-08-29: GO HOME and GO TO PICK POSE did nothing
            # whenever teleoperation was live. Not dead -- OUTVOTED.
            # `vr_pose_mapper` runs `hold_when_idle`, publishing the arm's own
            # live pose at 20 Hz so a clutch release cannot let the dead-man
            # latch; `ik_follower` turns that into trajectories on this very
            # topic. Three publishes here against twenty a second there, and
            # the target was gone inside 50 ms.
            #
            # `/pose_move_active` asks `ik_follower` to stand down for the
            # duration. The MAPPER keeps publishing throughout, so the
            # dead-man stays fed and the reason `hold_when_idle` exists is
            # not undone -- only the arm-controller topic changes hands.
            #
            # And the target is HELD, not fired three times. A
            # JointTrajectoryController holds its own goal, but the operator
            # presses this to get somewhere, and the hold is what makes
            # "arrived" a thing this window can wait for. It stops on
            # arrival, exactly as the real path's hold does.
            self.claim_arm_topic(True)
            self._start_hold(arm, pub, list(map(float, q)), secs)
        return (True, chosen)

    #: Longest a pose move may own the arm-controller topic before the claim
    #: is released regardless. A claim that could stick would be a way to
    #: disable teleoperation from a button that says GO HOME.
    POSE_CLAIM_MAX_S = 25.0

    def claim_arm_topic(self, on):
        """Take, or release, the arm-controller topic for a pose move.

        LATCHED, because `ik_follower` may not have been listening when the
        claim went out -- it subscribes at start-up and a pose move can
        happen before or after a follower restart. TRANSIENT_LOCAL means a
        follower that joins late still learns the topic is claimed.
        """
        # NOT LATCHED, AND NOT `publish_once`. THIS DISABLED TELEOPERATION.
        #
        # `publish_once` publishes TRANSIENT_LOCAL depth 1 -- deliberately,
        # so a node that subscribes after an E-STOP still learns the system
        # is stopped. For a STOP that is exactly right. For this claim it is
        # a trap: the retained sample means an `ik_follower_node` that starts
        # later immediately receives `pose_move_active = True` and drops
        # EVERY teleop command from its first breath, silently, for as long
        # as the window lives. The operator's report on 2026-08-30 was "the
        # real arms are not moving at all" with a stack that looked perfect.
        #
        # A claim is about NOW. It must not outlive the mover that made it,
        # and a subscriber that joins late must default to "nobody is
        # claiming" rather than inherit a stale yes.
        if not hasattr(self, "_claim_pub"):
            self._claim_pub = self.create_publisher(
                Bool, "/pose_move_active", 10)          # VOLATILE, on purpose
            time.sleep(0.2)                              # let discovery settle
        self._claim_pub.publish(Bool(data=bool(on)))
        if on:
            self._claim_t = time.time()
            self.note("pose move: took /<arm>_arm_controller/joint_trajectory "
                      "-- teleop commands are dropped until it releases")
        else:
            self.note("pose move: released the arm-controller topic")

    def claim_expired(self):
        """True when a claim has been held longer than it should be."""
        t = getattr(self, "_claim_t", 0.0)
        return bool(t) and (time.time() - t) > self.POSE_CLAIM_MAX_S

    #: How much longer than the theoretical minimum a relayed pose move is
    #: given. The relay rate-limits AND step-limits, and starts from rest, so
    #: the achievable average is well under the cap.
    POSE_RELAY_MARGIN = 1.8

    def _pose_secs_for(self, arm, q, secs):
        """Seconds this move needs so the relay can keep up with it.

        Read from the RELAY'S OWN `max_vel_rad_s`, not a constant here, so
        moving that slider cannot leave this stale -- a second copy of a
        speed limit is a second thing to forget.
        """
        import numpy as _np
        try:
            names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
            cur = getattr(self, "_real_q", {}) or {}
            have = [cur.get(n) for n in names]
            if not all(v is not None for v in have):
                return secs
            d = _np.abs((_np.array(have, float) - _np.array(q, float)
                         + _np.pi) % (2 * _np.pi) - _np.pi)
            travel = float(d.max())
            vmax = float(getattr(self, "_relay_vmax", 0.0)) or 0.30
            return (travel / vmax) * self.POSE_RELAY_MARGIN
        except Exception:                                     # noqa: BLE001
            return secs

    @staticmethod
    def _cascade_running():
        """Is sim_to_real_bridge relaying the simulation onto the real arm?

        Cached for a second: this is asked on every pose command and spawning
        pgrep on the Qt thread at speed is its own problem.
        """
        import subprocess as _sp
        now = time.time()
        if now - getattr(Bus, "_casc_t", 0.0) < 1.0:
            return Bus._casc
        try:
            out = _sp.run(["pgrep", "-f", "sim_to_real_bridge"],
                          capture_output=True, text=True, timeout=3).stdout
            Bus._casc = bool([x for x in out.split() if x.strip().isdigit()])
        except Exception:                                     # noqa: BLE001
            Bus._casc = False
        Bus._casc_t = now
        return Bus._casc

    def _start_hold(self, arm, pub, q, secs):
        if not hasattr(self, "_hold"):
            self._hold = {}
            self._hold_lock = threading.Lock()
        with self._hold_lock:
            # REPLACES any previous hold for this arm. Two holds on one arm
            # would be two writers racing on the same setpoint.
            self._hold[arm] = dict(pub=pub, q=q, secs=secs, t0=time.time())
        if not getattr(self, "_hold_thread_up", False):
            self._hold_thread_up = True
            threading.Thread(target=self._hold_loop, daemon=True).start()

    #: Arrived, in degrees, worst joint. Once inside this the hold STOPS.
    HOLD_ARRIVED_DEG = 1.5
    #: A constant, gentle horizon for every re-send. NOT a shrinking one.
    HOLD_HORIZON_S = 1.0

    def _hold_loop(self):
        """Feed the setpoint until the arm ARRIVES, then stop.

        TWO BUGS LIVED HERE AND THEY ARE OPPOSITES.
        -------------------------------------------
        FIRST: nothing was re-sent at all. The bridge's watchdog commands zero
        speed if no target arrives within `watchdog_s` (0.5 s), so one send
        bought half a second of motion and the operator had to press HOME over
        and over.

        THEN, fixing that badly: the whole trajectory was re-sent at 20 Hz
        with a time_from_start that SHRANK toward 0.2 s. A
        JointTrajectoryController treats each message as a NEW TRAJECTORY, so
        the arm was re-planned fifty times a second and told to close whatever
        error remained in 0.2 s. It reached the pose and then dithered around
        it, hard -- "extremely unstable" is exactly right, and it is worse the
        closer it gets, because the horizon is shortest there.

        The resolution is that the hold is only needed WHILE MOVING. Once the
        arm is at the target, the watchdog firing is harmless: zero speed on a
        non-backdrivable servo IS holding position. So this feeds the move and
        then gets out of the way, which is what both consumers want.
        """
        import numpy as _np
        while True:
            time.sleep(1.0 / self.HOLD_HZ)
            with self._hold_lock:
                items = list(self._hold.items())
            # HAND THE TOPIC BACK THE MOMENT THE MOVE IS OVER.
            #
            # Every hold ends -- on arrival, on the ceiling, or on an e-stop
            # -- so "no holds left" is exactly "the pose move is finished".
            # Releasing here rather than at the call site means a move that
            # ends the way nobody planned still gives teleoperation back. A
            # claim that could stick would be a way to disable the master arm
            # from a button labelled GO HOME, which is a worse defect than
            # the one this whole mechanism fixes.
            if not items and getattr(self, "_claim_t", 0.0):
                self._claim_t = 0.0
                self.claim_arm_topic(False)
            elif items and self.claim_expired():
                self._claim_t = 0.0
                self.claim_arm_topic(False)
                self.note("pose move exceeded %.0f s -- the topic was handed "
                          "back while the hold was still running. The arm may "
                          "not have arrived."
                          % self.POSE_CLAIM_MAX_S, bad=True)
            for arm, h in items:
                est = self.snap.get("estop")
                if est and bool(est[0]):
                    with self._hold_lock:
                        self._hold.pop(arm, None)
                    self.note("e-stop engaged -- dropped the %s arm's held "
                              "setpoint" % arm, bad=True)
                    continue
                # ARRIVED? Then stop feeding it and say so ONCE.
                names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
                cur = getattr(self, "_real_q", {}) or {}
                have = [cur.get(n) for n in names]
                if all(v is not None for v in have):
                    d = _np.abs((_np.array(have) - _np.array(h["q"]) + _np.pi)
                                % (2 * _np.pi) - _np.pi)
                    worst = float(_np.degrees(d).max())
                    if worst < self.HOLD_ARRIVED_DEG:
                        with self._hold_lock:
                            self._hold.pop(arm, None)
                        self.note("%s arm arrived (worst %.2f deg) -- hold "
                                  "released; the controller holds position "
                                  "from here" % (arm, worst))
                        continue
                # GIVE UP FEEDING after a sane ceiling rather than for ever.
                # A setpoint re-sent indefinitely at an arm that is not moving
                # is an arm fighting something, and that should be visible.
                if time.time() - h["t0"] > max(20.0, h["secs"] * 3):
                    with self._hold_lock:
                        self._hold.pop(arm, None)
                    self.note("%s arm did not arrive within %.0f s -- hold "
                              "released. It may be blocked, e-stopped, or the "
                              "target may be unreachable."
                              % (arm, max(20.0, h["secs"] * 3)), bad=True)
                    continue
                t = JointTrajectory()
                t.joint_names = names
                pt = JointTrajectoryPoint()
                pt.positions = h["q"]
                # CONSTANT horizon. A shrinking one is what made the endgame
                # violent: the closer the arm got, the harder it was told to
                # close the remainder.
                pt.time_from_start = RosDuration(
                    sec=int(self.HOLD_HORIZON_S),
                    nanosec=int((self.HOLD_HORIZON_S % 1.0) * 1e9))
                t.points = [pt]
                try:
                    h["pub"].publish(t)
                except Exception:                             # noqa: BLE001
                    pass

    def release_hold(self, arm=None):
        """Stop holding. Used by the e-stop path and when a run takes over."""
        if not hasattr(self, "_hold"):
            return
        with self._hold_lock:
            if arm is None:
                self._hold.clear()
            else:
                self._hold.pop(arm, None)

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
        # THE ORDER OF THESE BRANCHES IS LOAD-BEARING. `bool` is a subclass of
        # `int` in Python, so an `isinstance(value, int)` test placed above the
        # bool test swallows True/False and sends them as integers -- which a
        # node declaring a BOOL parameter then REFUSES, reporting a type error
        # against a control that looks perfectly correct in the window.
        if isinstance(value, bool):
            v.type = ParameterType.PARAMETER_BOOL
            v.bool_value = value
        elif isinstance(value, str):
            # STRINGS WERE NOT SUPPORTED AND WERE NOT REFUSED EITHER: every
            # non-bool went through float(), so a string parameter raised
            # ValueError inside this method and the operator saw nothing at
            # all. `smoothing` is the first string parameter any control in
            # this window sets.
            v.type = ParameterType.PARAMETER_STRING
            v.string_value = value
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
class WheelGuard(QObject):
    """Refuse wheel events on value controls that do not have focus.

    See the note at the install site in main(). The guarded types are the
    ones whose value decides what the next run does; a wheel over any of
    them while merely scrolling the column past it is a silent edit.
    """

    # QDoubleSpinBox is NOT a subclass of QSpinBox -- both derive from
    # QAbstractSpinBox and neither from the other -- so leaving it out would
    # have left the two smoothing knobs unguarded while looking covered. They
    # sit in a scrolling column, which is exactly the situation this guard
    # exists for: a wheel over `beta` on the way past it silently retunes the
    # teleoperation.
    GUARDED = (QComboBox, QSlider, QSpinBox, QDoubleSpinBox)

    def eventFilter(self, obj, ev):
        if ev.type() == QEvent.Wheel and isinstance(obj, self.GUARDED):
            if not obj.hasFocus():
                ev.ignore()
                return True
        return False


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
        # AND MAKE IT STICK. `setSizes` is a REQUEST -- the splitter is free
        # to squeeze a pane down to its widget's minimum, and this one's
        # minimum was 362. Measured offscreen: `_left_width()` returned 414,
        # `setSizes` was given 414, and the splitter settled on 392. Every
        # button label was then cut at the right-hand edge, which is the
        # defect this method was written to remove and did not.
        #
        # Raising the column's own minimum to the measured width is the only
        # thing the splitter cannot overrule. The 620 ceiling still applies.
        self._left_col.setMinimumWidth(min(lw, 620))
        self.split.setSizes([lw, 620, max(420, 1920 - lw - 620)])
        # AND SOMETHING HAS TO START IT.
        #
        # `_fit_left_column` widens the column until `clipped_pages()` is
        # empty, re-arming itself with a timer between passes. It was called
        # from EXACTLY ONE PLACE: its own re-arm. Nothing ever kicked the
        # chain off, so the whole mechanism -- written, commented at length,
        # and correct -- had never run in the live window, and the column sat
        # at whatever the splitter first handed it.
        #
        # 400 ms, so the layout has settled and the fonts are real before the
        # first measurement. It stops on its own when nothing is clipped.
        # TWICE, AND THE SECOND ONE IS NOT BELT-AND-BRACES.
        #
        # Measured in the live window: at 400 ms the pages report a shortfall
        # of a few pixels, the loop satisfies it, converges and stops -- and
        # by 1.5 s the same page reports needing 524 px against a 380 px
        # viewport, because the tabs, the fonts and the embedded panel are
        # still settling. A convergence test that runs before the thing it
        # measures has finished changing will always converge, and always on
        # the wrong number.
        QTimer.singleShot(400, self._fit_left_column)
        QTimer.singleShot(2500, lambda: self._fit_left_column(12))
        # ...and the second pass is SKIPPED ONCE RVIZ HAS EMBEDDED. See
        # `_fit_left_column`.

        outer.addLayout(self._bottom_bar())
        self.setCentralWidget(root)
        # MEASURED, not assumed. See `fit_to_screen`.
        _scr = QGuiApplication.primaryScreen()
        _av = _scr.availableGeometry() if _scr is not None else None
        if _av is not None:
            _w, _h = fit_to_screen(_av.width(), _av.height())
            self.resize(_w, _h)
            # And put it somewhere wholly inside that area. Centring on the
            # AVAILABLE rect (not the screen rect) keeps it clear of the
            # taskbar, which is the strip WSLg is least willing to overlap.
            self.move(_av.left() + max(0, (_av.width() - _w) // 2),
                      _av.top() + max(0, (_av.height() - _h) // 2))
        else:
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
        # THE CAMERA WATCHDOG, STARTED WITHOUT BEING ASKED.
        #
        # usbipd hands each USB camera across the WSL boundary and drops them
        # on its own -- the service stops, a device re-enumerates, a hub
        # sleeps. `usb_cameras.py --fix` repairs that beautifully, ONCE, when
        # somebody notices and presses something. The operator's report on
        # 2026-08-30 was "the cameras are constantly disconnected", which is
        # the same fault on a timer with nobody watching for it, and a repair
        # you have to remember to run is not a fix for a recurring fault.
        #
        # 5 s in, so the window is drawn first: this shells out to
        # usbipd.exe across the Windows boundary and the first call is the
        # slow one.
        QTimer.singleShot(5000, self._ensure_camera_watchdog)
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
            # THE BUTTONS, NOT JUST THE PAGE'S MINIMUM.
            #
            # `minimumSizeHint()` on a QPushButton is smaller than the width
            # its own text needs -- Qt will elide rather than demand room --
            # so a page whose buttons are all being cut in half reported that
            # it fitted. `_fit_left_column` then had nothing to widen for and
            # stopped, which is why the column sat at 392 px against a
            # measured requirement of 414 and every label lost its right-hand
            # end: "SIM + REAL" as a red sliver, "REMEMBER VIEW" as "R".
            #
            # A button's `sizeHint()` IS its text width, and unlike a
            # word-wrapped QLabel's it is not an unwrapped-paragraph number,
            # which is what made an earlier attempt at this useless.
            need = inner.minimumSizeHint().width()
            for lay in inner.findChildren(QHBoxLayout):
                w = sum(lay.itemAt(k).widget().sizeHint().width()
                        for k in range(lay.count())
                        if lay.itemAt(k).widget() is not None)
                need = max(need, w + lay.spacing() * max(0, lay.count() - 1))
            for b in inner.findChildren(QPushButton):
                need = max(need, b.sizeHint().width())
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
        # NEVER RESIZE THE SPLITTER AFTER RVIZ HAS BEEN ADOPTED.
        #
        # RViz is a FOREIGN X WINDOW reparented into a container. This file
        # already records that touching the layout 250 ms after the reparent
        # broke the adoption -- the container held the window and the window
        # ended up at the corner of the SCREEN, so the COMMANDED view was
        # blank while the log said "embedded ... viewable".
        #
        # I added a second widening pass at 2500 ms without testing the
        # embedded case: every screenshot I took was on an Xvfb with no
        # window manager, where RViz is NOT embedded and there is nothing to
        # disturb. The operator runs with a window manager, where it is. That
        # is the one configuration the change had never been seen in, and it
        # is the one that matters.
        #
        # The widening still happens -- it runs before the adoption, which is
        # 3 s in -- and stops touching the splitter the moment RViz is in it.
        if getattr(self, "embedded", None):
            return True
        short = max((n - v for _, v, n in self.clipped_pages()), default=0)
        if short <= 0 or tries <= 0:
            return True
        sizes = self.split.sizes()
        if len(sizes) < 3:
            return False
        new_left = min(620, sizes[0] + short + 2)
        if new_left <= sizes[0]:
            return False                # at the ceiling; report, do not loop
        # RAISE THE COLUMN'S OWN MINIMUM, NOT JUST THE SPLITTER'S REQUEST.
        #
        # `setSizes` is a request the splitter may refuse: it will not shrink
        # a pane below its widget's minimum, and it is free to shrink THIS one
        # down to its minimum to satisfy the others. The minimum was set once,
        # in the constructor, from `_left_width()` -- and measured offscreen,
        # that call returns 392 DURING CONSTRUCTION and 414 once the layout
        # has settled and the fonts are real. So the floor was fixed at a
        # pre-layout guess and the splitter sat on it for the life of the
        # window, whatever this method asked for afterwards.
        #
        # Raising it here, where the measurement is real, is the thing the
        # splitter cannot overrule. The 620 ceiling still protects RViz.
        self._left_col.setMinimumWidth(min(new_left, 620))
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
            pad = bar + 2 * sa.frameWidth()
            need = max(need, inner.minimumSizeHint().width() + pad)
            # AND THE BUTTONS THEMSELVES, BY THEIR sizeHint.
            #
            # `minimumSizeHint()` on a QPushButton is smaller than the width
            # its own text needs -- Qt is willing to elide -- so a page full
            # of buttons reported a minimum that clipped every one of them.
            # Measured from a screenshot after switching the buttons to a
            # Preferred policy: the column grew about 25 px and "SIM + REAL"
            # was still a red sliver.
            #
            # A button's `sizeHint()` IS its text width, and unlike a
            # word-wrapped QLabel's it is not an unwrapped-paragraph number,
            # which is what made attempt one useless. Rows of side-by-side
            # buttons are handled by asking the row's layout, not the button.
            for lay in inner.findChildren(QHBoxLayout):
                w = sum(lay.itemAt(i).widget().sizeHint().width()
                        for i in range(lay.count())
                        if lay.itemAt(i).widget() is not None)
                w += lay.spacing() * max(0, lay.count() - 1)
                need = max(need, w + pad + 24)
            for b in inner.findChildren(QPushButton):
                need = max(need, b.sizeHint().width() + pad + 24)
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
        campanel = QGroupBox("GRIPPER CAMERAS  --  live")
        campanel.setFont(helvetica(11, True))
        cl = QHBoxLayout(campanel)
        self.cam_lbl, self.cam_cap = {}, {}
        for a in ARMS:
            c = QVBoxLayout()
            nm = QLabel(a.upper())
            nm.setFont(helvetica(10, True))
            nm.setAlignment(Qt.AlignCenter)
            img = QLabel()
            # A FIXED BOX, NOT A MINIMUM, AND IT IS A BUG FIX.
            #
            # This was setMinimumSize, and refresh() scales each frame to
            # `cam_lbl.width()/height()` -- the label's OWN CURRENT SIZE. A
            # label with a pixmap takes its size hint FROM that pixmap, so
            # the label sized the frame and the frame then sized the label:
            # a positive feedback loop with nothing damping it.
            #
            # The stale path made it visible. Going stale clears the pixmap
            # and writes text, so the label collapses toward its minimum;
            # the next frame is then scaled into the smaller box, and the
            # box ratchets. Measured on the rig 2026-08-25 the panel
            # resized continuously, several times a second, because the
            # Kinova colour stream was flapping live/stale at 1.6-5 Hz
            # against a fixed 0.5 s staleness window (fixed separately, in
            # camera_relay.thresholds).
            #
            # A fixed box breaks the loop at the source: the scale target no
            # longer depends on what was last painted, so a frame, a STALE
            # label and an empty panel all occupy exactly the same rectangle
            # and nothing downstream of them moves.
            img.setFixedSize(220, 165)
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
            # AND A FIXED HEIGHT ON THE CAPTION, for the same reason one line
            # up. The caption is wrapped, so its height depends on how long
            # the text is -- and the text alternates between "live 9.2 Hz
            # 1280x720 [/left_camera/color/image_raw]" and "STALE -- last
            # frame 0.52 s ago", which wrap to different numbers of lines.
            # Every one of those transitions relaid out the column. Three
            # lines is enough for the longest caption plus its topic name.
            cap.setFixedHeight(42)
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
        # THE OPERATOR'S OWN ORDER (2026-08-27): the connection row on top,
        # then ONE tab with the three modes (shared autonomy is an option
        # inside master and VR, not a section), then the data controls.
        # Everything below that is detail. The VR section lives INSIDE the
        # VR tab now -- one place per mode, no repeats.
        rv.addWidget(self._arms_panel())
        # POSE directly under the connection row (operator, 2026-08-27):
        # "put the arms somewhere known" is the most frequent press of a
        # session and belongs where the arms are.
        rv.addWidget(self._pose_panel())
        rv.addWidget(self._mode_tabs())
        rv.addWidget(self._experiments_panel())
        rv.addWidget(self._real_panel())
        rv.addWidget(self._map_panel())
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

    def _mode_tabs(self):
        """MASTER | VR | FULL AUTONOMY -- one tab each, one START each.

        The operator's spec, verbatim: connection row on top, then a tab
        with the three modes; shared autonomy is an OPTION inside master
        and VR, not a section of its own; START launches the simulation
        and everything the real cascade will need; START REAL ARMS then
        just works. Every button delegates to machinery that already
        exists and is idempotent (`on_vr_start`, `start_mode`,
        `_ensure_rviz`) -- this panel adds no new path to the metal.
        """
        g = QGroupBox("DRIVE")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        v.setContentsMargins(4, 16, 4, 4)
        v.setSpacing(4)
        self.mode_btn = {}

        def _big(text, tip, fn, col=None):
            b = QPushButton(text)
            b.setFont(helvetica(12, True))
            b.setMinimumHeight(40)
            b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
            edge = col or LINE
            b.setStyleSheet(
                "QPushButton{border:1px solid %s;border-radius:3px;"
                "padding:3px;color:%s}QPushButton:hover{border:1px solid %s}"
                % (edge, col or C_TEXT, col or C_OK))
            b.setToolTip(tip)
            b.clicked.connect(fn)
            return b

        tabs = QTabWidget()
        tabs.setFont(helvetica(10, True))
        self.drive_tabs = tabs

        # ------------------------------------------------------- MASTER
        mt = QWidget()
        mv = QVBoxLayout(mt)
        mv.setSpacing(4)
        b = _big("START MASTER TELEOP",
                 "Launches the simulation and the master-arm stack, ready "
                 "for the real arms. Needs the Teensy.",
                 self.on_start_master, C_OK)
        self.mode_btn["teleop"] = b
        mv.addWidget(b)
        # A TOGGLE THAT LOOKS LIKE ONE. This was a QCheckBox, whose
        # indicator is nearly invisible on this dark theme -- the operator
        # asked "why is there no option for shared autonomy" while the
        # option was on screen. Same defect as the flat mode buttons of
        # 2026-08-22: a control that does not look like a control is not
        # one.
        self.ma_shared = self._shared_toggle(
            "START then also launches perception, grasp generation and the "
            "arbiter on top of your control.", "master")
        self.mode_btn["shared"] = self.ma_shared
        mv.addWidget(self.ma_shared)
        # The master path runs the same adaptive smoothing as VR since
        # 2026-08-27 (measured: EMA(0.3) cost 18.7 mm of lag on a brisk
        # reach; 1-Euro 1.6 mm). The law is a live parameter; this is its
        # window control, hidden like VR's because it matters when tuning
        # feel and never during a session start.
        ma_adv = QCheckBox("advanced: smoothing")
        ma_adv.setFont(helvetica(8))
        mv.addWidget(ma_adv)
        ma_row = QWidget()
        mrl = QHBoxLayout(ma_row)
        mrl.setContentsMargins(0, 0, 0, 0)
        t = QLabel("law")
        t.setFont(helvetica(9))
        mrl.addWidget(t)
        self.ma_smooth = QComboBox()
        self.ma_smooth.setFont(helvetica(9))
        self.ma_smooth.addItems(["one_euro", "ema", "none"])
        self.ma_smooth.setToolTip(
            "one_euro: still when you are still, no lag when you move "
            "(0.83 mm / 1.59 mm measured).\n"
            "ema: the fixed filter that shipped before -- 18.7 mm of lag "
            "at 0.40 m/s. Kept so old recordings reproduce.\n"
            "none: raw.")
        self.ma_smooth.currentTextChanged.connect(
            lambda law: (self.bus.set_param("/master_pose_node",
                                            "smoothing", law),
                         self.log("master smoothing law -> %s" % law)))
        mrl.addWidget(self.ma_smooth, 1)
        ma_row.setVisible(False)
        ma_adv.toggled.connect(
            lambda on: (ma_row.setVisible(bool(on)),
                        self.log("master smoothing settings %s"
                                 % ("shown" if on else "hidden"))))
        mv.addWidget(ma_row)
        mv.addStretch(1)
        tabs.addTab(mt, "MASTER")

        # ----------------------------------------------------------- VR
        vt = QWidget()
        vv = QVBoxLayout(vt)
        vv.setSpacing(4)
        self.vr_btn = _big(
            "START VR TELEOP",
            "Launches the simulation and the whole VR chain, all twelve "
            "steps checked, stopping at the first thing it cannot do.",
            self.on_start_vr_clicked, C_OK)
        self.mode_btn["vr"] = self.vr_btn
        vv.addWidget(self.vr_btn)
        self.vr_shared = self._shared_toggle(
            "START then also launches the arbiter on top of the VR "
            "controllers: the mapper runs, autonomy owns the pose.", "VR")
        self.mode_btn["shared_vr"] = self.vr_shared
        vv.addWidget(self.vr_shared)
        vv.addWidget(self._vr_panel())
        vv.addWidget(self._feel_panel())
        vv.addWidget(self._shared_panel())
        tabs.addTab(vt, "VR")

        # ------------------------------------------------- FULL AUTONOMY
        ft = QWidget()
        fv = QVBoxLayout(ft)
        fv.setSpacing(4)
        b = _big("START FULL AUTONOMY  --  type what you want",
                 "Launches the autonomy stack and puts the keyboard in the "
                 "Instruct box. Nothing moves until you press CONFIRM "
                 "there.",
                 self.on_flow_full, C_OK)
        self.mode_btn["full"] = b
        self.mode_btn["__instruct__"] = b
        fv.addWidget(b)
        fv.addStretch(1)
        tabs.addTab(ft, "FULL AUTONOMY")

        v.addWidget(tabs)

        # THE REAL ARMS ARE ONE BUTTON, AND IT IS NOT IN HERE.
        #
        # There were FOUR ways to start the real arms in this window -- this
        # one, plus `1. START REAL ARMS`, `2. HOME BOTH ARMS` and
        # `4. START REAL ARM TELEOP` in the panel below -- and the operator
        # had to know which, and in what order, and what each would refuse.
        # On 2026-08-29 that cost a lab session: every one of them was
        # pressed, each did part of the job, and the arm never moved.
        # `MOVE THE REAL ARMS`, beside RECORD EVERYTHING, is now the only
        # one. It works out for itself which of those steps are still
        # needed and does them in order.
        r = QHBoxLayout()
        r.setSpacing(4)
        b = QPushButton("open the sim view")
        b.setFont(helvetica(9))
        b.setToolTip("RViz. It opens on its own with START; this re-opens "
                     "it if it is not there.")
        b.clicked.connect(self.on_flow_sim)
        self.mode_btn["__sim_view__"] = b
        r.addWidget(b)
        b = QPushButton("rehearse with mock arms")
        b.setFont(helvetica(9))
        b.setToolTip("The identical real-arm sequence against mock "
                     "hardware. Nothing physical moves.")
        b.clicked.connect(
            lambda: self.start_mode(["real_mock"], "REHEARSE (mock arms)"))
        self.mode_btn["__mock__"] = b
        r.addWidget(b)
        v.addLayout(r)

        self.mode_live_lbl = QLabel("live mode: --")
        self.mode_live_lbl.setFont(helvetica(9, True))
        self.mode_live_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.mode_live_lbl)
        self.mode_seq_lbl = QLabel("")
        self.mode_seq_lbl.setWordWrap(True)
        self.mode_seq_lbl.setFont(helvetica(9))
        self.mode_seq_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.mode_seq_lbl)
        stop = QPushButton("STOP everything this window launched")
        stop.setFont(helvetica(10, True))
        stop.clicked.connect(self.on_stop_jobs)
        v.addWidget(stop)
        self.flow_note = QLabel("")
        self.flow_note.setFont(helvetica(9))
        self.flow_note.setWordWrap(True)
        self.flow_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.flow_note)
        return g

    # ===================================================================
    #  FEEL  --  the knobs that decide whether teleoperation is usable
    # ===================================================================
    #: (label, node, parameter, min, max, decimals, default, what it trades)
    #
    # EVERY ONE OF THESE IS RE-READ BY THE NODE THAT OWNS IT. That is not a
    # detail, it is the whole reason this panel is allowed to exist: on
    # 2026-08-29 the real-arm gate was a parameter with no consumer, and a
    # row of sliders writing values nobody looks at again would be the same
    # defect with a nicer face. `sim_to_real_bridge` re-reads its feel
    # parameters every tick, `kortex_highlevel_bridge` has had a live
    # parameter callback all along, and `vr_pose_mapper` rebuilds its
    # filters when one of these changes.
    #
    # WHAT IS DELIBERATELY ABSENT: `lag_trip_rad`, `max_step_rad`,
    # `enable_gap_rad`. Those are the guards, not the feel. A limit that can
    # be widened from a slider while the arm is moving is not a limit, and
    # the operator reaching for "smoother" must not be able to reach a
    # safety trip by accident. They stay launch parameters.
    FEEL = [
        ("hand motion scale", "/vr_pose_mapper", "scale",
         0.10, 2.00, 2, 1.00,
         "How far the ROBOT moves per metre of YOUR hand. THE DEXTERITY "
         "KNOB: 0.3 means a 30 cm reach becomes 9 cm of robot, so fine work "
         "gets three times the hand travel to do it in. Takes effect on the "
         "next re-grip."),
        ("steadiness  (min cutoff Hz)", "/vr_pose_mapper", "min_cutoff_hz",
         0.10, 5.00, 2, 1.00,
         "The 1-Euro filter's floor. LOWER is steadier when your hand is "
         "nearly still -- it is what kills tremor -- and adds lag. Raise it "
         "if the arm feels like it is wading."),
        ("follow-through  (beta)", "/vr_pose_mapper", "beta",
         0.0, 60.0, 1, 10.0,
         "How much the filter opens up when you move FAST -- the cutoff is "
         "steadiness + this x hand speed. Too HIGH and it opens past the "
         "sampling limit and stops filtering at all while you move, which "
         "is jitter, not responsiveness. The 1-Euro author says tune it in "
         "FACTORS OF TEN, not small nudges."),
        ("hand speed limit  (m/s)", "/vr_pose_mapper", "max_speed_mps",
         0.30, 3.00, 2, 2.00,
         "THE ONE THAT DEGRADES OVER TIME. When your hand outruns this, the "
         "limiter clips the step -- and it CANNOT give the distance back "
         "while the motion continues, so the command falls further behind "
         "the longer you drive. That is the miss that grew 27, 41 then 50 mm "
         "across one run. Raise it until ordinary reaching stops triggering "
         "it; releasing and re-gripping clears what has accumulated."),
        ("wrist steadiness", "/vr_pose_mapper", "rot_min_cutoff_hz",
         0.10, 5.00, 2, 1.00,
         "The same floor for ORIENTATION. Wrist jitter is the usual reason "
         "a grasp will not line up; lower this before lowering the scale."),
        ("arm lag behind you  (s)", "/sim_to_real_bridge_%s",
         "preview_delay_s", 0.10, 2.00, 2, 0.30,
         "How far the METAL runs behind the simulation. 1.0 s is a big part "
         "of why teleoperation feels indirect -- you are steering something "
         "a second in the past. Lower it for directness; the delay is what "
         "gives the lag monitor time to catch a divergence before the arm "
         "commits to it."),
        ("arm speed cap  (rad/s)", "/sim_to_real_bridge_%s",
         "max_vel_rad_s", 0.05, 0.80, 2, 0.40,
         "The fastest the relay will replay a joint. Too low and the arm can "
         "NEVER keep up, the error accumulates and the lag monitor trips on "
         "a limit rather than a fault -- that is what 0.15 did on "
         "2026-08-25. 0.40 is 29% of the joint limit."),
        ("anti-buzz deadband  (deg)", "/kortex_highlevel_bridge_%s",
         "deadband_deg", 0.05, 1.50, 2, 0.25,
         "THE VIBRATION KNOB. Inside this band the proportional term is "
         "switched OFF. Too narrow and every joint chases encoder noise "
         "through a 12 Hz loop with a network round trip in it -- a "
         "correction that arrives late becomes an oscillation, which is the "
         "buzz. Too wide and joints park short: 1.0 deg cost 7.2 mm at the "
         "end effector. RAISE THIS FIRST if the arm vibrates."),
        ("tracking stiffness  (kp)", "/kortex_highlevel_bridge_%s", "kp",
         0.10, 1.50, 2, 0.50,
         "How hard the arm closes the gap to its setpoint. Higher tracks "
         "more tightly and, past this link's latency budget, oscillates. "
         "Raise it in small steps and watch for buzz at the wrist."),
    ]

    def _shared_panel(self):
        """WHAT THE ARBITER IS DOING, AND WHY -- the only way to test it.

        `handover_arbiter` is a DISCRETE handover: DIRECT -> ASSIST ->
        GRASPED. It never blends position; in every state the operator keeps
        100% position authority, and ASSIST servos ORIENTATION only, over
        ~0.5 s, by SLERP. It enters ASSIST only when the intent probability
        is over threshold AND the end effector is inside the distance
        threshold AND the intent is not ambiguous -- all three, never any.

        That design is defensible precisely BECAUSE it is conditional, and a
        conditional system you cannot see is indistinguishable from one that
        does nothing. Every field below is already published on
        /autonomy/arbiter_<arm>; the window simply never read it. So the
        operator's question -- "how do we test shared autonomy" -- had no
        answer that did not involve watching the arm and guessing.

        The three refusal reasons are the interesting ones, and they are
        shown by name: too far, not confident enough, or ambiguous between
        two objects.
        """
        # SHORT TITLE. A QGroupBox title is NOT word-wrapped, so it sets a
        # floor under the whole column's width -- "SHARED AUTONOMY -- what the
        # robot is doing, and why" pushed scroll page 2 to 524 px against a
        # 496 px viewport and the audit's cut-off check caught it. The
        # explanation goes in the note below, which wraps.
        g = QGroupBox("SHARED AUTONOMY")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        note = QLabel(
            "What the robot is doing, and why. "
            "Tick 'robot helps' above BEFORE pressing START. The robot never "
            "takes position -- it only turns the wrist towards a grasp, and "
            "only when it is close, confident and unambiguous. It closes the "
            "gripper NEVER; that is always your trigger.")
        note.setWordWrap(True)
        note.setFont(helvetica(9))
        note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(note)
        self.arb_lbl = {}
        for a in ARMS:
            row = QLabel("%s: arbiter not running" % a.upper())
            row.setWordWrap(True)
            row.setFont(helvetica(9))
            row.setStyleSheet("color:%s" % C_UNKNOWN)
            self.arb_lbl[a] = row
            v.addWidget(row)
        return g

    def _refresh_shared(self, s):
        """One line per arm: the state, and the reason it is not the next one."""
        if not hasattr(self, "arb_lbl"):
            return
        for a in ARMS:
            raw = s.get("arb_%s" % a)
            lbl = self.arb_lbl[a]
            if raw is None:
                lbl.setText("%s: arbiter not running -- tick 'robot helps' "
                            "before START" % a.upper())
                lbl.setStyleSheet("color:%s" % C_UNKNOWN)
                continue
            try:
                d = json.loads(raw[0])
            except Exception:                                 # noqa: BLE001
                lbl.setText("%s: arbiter said something unparsable" % a.upper())
                lbl.setStyleSheet("color:%s" % C_WARN)
                continue
            st = str(d.get("state", "?"))
            col = {"DIRECT": C_MUTED, "ASSIST": C_OK,
                   "GRASPED": C_OK}.get(st, C_WARN)
            dist = d.get("distance_m")
            top, p = d.get("top"), d.get("top_p")
            bits = ["%s: %s" % (a.upper(), st)]
            if d.get("reason"):
                bits.append(str(d["reason"]))
            if dist is not None:
                bits.append("%.0f mm from the grasp" % (float(dist) * 1000.0))
            if top:
                bits.append("wants %s%s" % (
                    top, "" if p is None else " (%.0f%%)" % (float(p) * 100.0)))
            if d.get("ambiguous"):
                bits.append("AMBIGUOUS -- will not assist between two objects")
            # WHO HAS THE ARM. Stated every line, because it is the property
            # the whole design rests on and the operator should never have to
            # remember it.
            bits.append("position: %s / wrist: %s"
                        % (d.get("position_authority", "?"),
                           d.get("orientation_authority", "?")))
            lbl.setText("   |   ".join(bits))
            lbl.setStyleSheet("color:%s" % col)

    def _feel_panel(self):
        """SLIDERS FOR THE THINGS THAT DECIDE WHETHER THIS IS USABLE.

        The operator's complaint is not a bug report, it is the real
        measure: "teleoperation is hard, not smooth, not dexterous". Every
        knob that answers that lived in a launch argument or a parameter
        name, which means it did not exist for the person in the headset.

        Each row says what it TRADES, not just what it is. A slider labelled
        `min_cutoff_hz` is a slider nobody moves.
        """
        g = QGroupBox("FEEL  --  smoothness and dexterity, live")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        note = QLabel(
            "Live, while you drive. Every one of these is re-read by the "
            "node that owns it -- nothing here is a number that gets "
            "written and forgotten. The safety trips are NOT here.")
        note.setWordWrap(True)
        note.setFont(helvetica(9))
        note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(note)
        self.feel_lbl = {}
        for (label, node, prm, lo, hi, dec, dflt, tip) in self.FEEL:
            row = QWidget()
            rl = QVBoxLayout(row)
            rl.setContentsMargins(0, 2, 0, 2)
            rl.setSpacing(1)
            cap = QLabel("%s   %.*f" % (label, dec, dflt))
            cap.setFont(helvetica(9, True))
            cap.setStyleSheet("color:%s" % C_MUTED)
            cap.setToolTip(tip)
            rl.addWidget(cap)
            sld = QSlider(Qt.Horizontal)
            steps = 200
            sld.setMinimum(0)
            sld.setMaximum(steps)
            sld.setValue(int(round((dflt - lo) / (hi - lo) * steps)))
            sld.setToolTip(tip)
            sld.valueChanged.connect(
                lambda n, _lo=lo, _hi=hi, _st=steps, _d=dec, _c=cap,
                _l=label: _c.setText(
                    "%s   %.*f" % (_l, _d, _lo + (_hi - _lo) * n / _st)))
            # ON RELEASE, NOT ON EVERY PIXEL. `valueChanged` fires for every
            # step of a drag; each one is a service call to a node driving a
            # real arm, and a hundred of them in a second is a different
            # experiment from the one the operator thinks they are running.
            sld.sliderReleased.connect(
                lambda _s=sld, _lo=lo, _hi=hi, _st=steps, _n=node, _p=prm,
                _d=dec, _l=label: self._feel_apply(
                    _n, _p, _lo + (_hi - _lo) * _s.value() / _st, _d, _l))
            rl.addWidget(sld)
            self.feel_lbl[prm] = cap
            v.addWidget(row)
        return g

    def _feel_apply(self, node, prm, value, dec, label):
        """Set one feel parameter, off the Qt thread, and say what happened.

        Through `bus.set_param`, which checks the service is ready, reads the
        value BACK and is loud when it did not take -- a slider that moved
        while the arm did not is the failure this whole session was about.
        """
        # PER-ARM NODES GET IT ON EVERY ARM THAT IS UP. Hard-coding `_left`
        # would tune one arm of a two-arm rig and leave the operator
        # comparing a smoothed arm against an unsmoothed one without being
        # told -- two conditions that look like one.
        if "%s" in node:
            arms = self._kortex_arms() if "kortex" in node \
                else self._relay_arms()
            targets = [node % a for a in arms] or []
        else:
            targets = [node]
        if not targets:
            self.log("%s: no node is running that owns %s -- UNCHANGED"
                     % (label, prm), bad=True)
            return

        # KEEP THE POSE PACING IN STEP WITH THIS SLIDER. `send_joint_pose`
        # stretches a relayed move so the simulation cannot outrun the arm,
        # and it needs the relay's speed cap to do the arithmetic. Recording
        # it here means moving the slider cannot leave that stale -- a second
        # copy of a speed limit is a second thing to forget.
        if prm == "max_vel_rad_s":
            self.bus._relay_vmax = float(value)

        def go():
            for t in targets:
                self.bus.set_param(t, prm, float(value))
        try:
            self.bus.submit(go, label="%s -> %.*f (%s)"
                            % (label, dec, value, ", ".join(targets)))
        except Exception as e:                                # noqa: BLE001
            self.log("feel: %s FAILED %r" % (label, e), bad=True)

    def _shared_toggle(self, tip, who):
        """The shared-autonomy option as a bordered, checkable button that
        SAYS its state -- OFF / ON in the label, never an indicator dot."""
        b = QPushButton("robot helps  (shared autonomy):  OFF")
        b.setCheckable(True)
        b.setFont(helvetica(10, True))
        b.setMinimumHeight(30)
        b.setToolTip(tip)
        b.setStyleSheet(
            "QPushButton{border:1px solid %s;border-radius:3px;padding:3px;"
            "color:%s}QPushButton:checked{border:2px solid %s;color:%s}"
            % (LINE, C_MUTED, C_OK, C_OK))
        b.toggled.connect(lambda on: (
            b.setText("robot helps  (shared autonomy):  %s"
                      % ("ON" if on else "OFF")),
            self.log("%s: shared autonomy %s for the next START"
                     % (who, "ON" if on else "off"))))
        return b

    def _ensure_camera_watchdog(self):
        """Keep `usb_cameras.py --watch` running for this window's lifetime.

        ONE INSTANCE, and it is checked rather than assumed: two watchdogs
        would race to detach-and-attach the same camera, which is a way to
        make a working camera fail. Started detached so it survives a busy
        Qt thread, and its output goes to the scratch directory rather than
        into the window, because it is deliberately silent when nothing is
        wrong and its log is the record of how often "nothing" was not true.
        """
        try:
            if getattr(self, "_camwatch", None) is not None \
                    and self._camwatch.poll() is None:
                return
            script = os.path.join(_WS, "scripts", "usb_cameras.py")
            if not os.path.exists(script):
                return
            try:
                out = subprocess.run(
                    ["pgrep", "-f", "usb_cameras.py --watc[h]"],
                    capture_output=True, text=True, timeout=5).stdout
                if out.strip():
                    self.log("camera watchdog already running (pid %s)"
                             % out.split()[0])
                    return
            except Exception:                                 # noqa: BLE001
                pass
            log = open(os.path.join(_scratch(), "usb_camera_watchdog.log"),
                       "ab", buffering=0)
            self._camwatch = subprocess.Popen(
                [sys.executable, script, "--watch"],
                stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            self.log("camera watchdog started (pid %d) -- it re-attaches the "
                     "USB cameras whenever usbipd drops them"
                     % self._camwatch.pid)
        except Exception as e:                                # noqa: BLE001
            self.log("camera watchdog did not start: %r" % (e,), bad=True)

    def _ensure_virtual_teensy(self):
        """A master input with no hardware: scripts/virtual_teensy.py, a pty
        replaying a REAL recorded session in the exact wire format
        master_pose_node parses. Returns the pty path, or None.

        Without this, START MASTER on a Teensy-less bench brings up a
        perfectly healthy stack whose master node retries for ~165 s and
        parks DORMANT -- a working simulation in which nothing ever moves,
        which the operator correctly reads as broken.
        """
        port_file = "/tmp/virtual_teensy_port"
        vt = getattr(self, "_vteensy", None)
        if vt is not None and vt.poll() is None and os.path.exists(port_file):
            try:
                return open(port_file).read().strip() or None
            except OSError:
                return None
        script = os.path.join(_WS, "scripts", "virtual_teensy.py")
        if not os.path.exists(script):
            return None
        try:
            os.remove(port_file)
        except OSError:
            pass
        try:
            log = open(os.path.join(_scratch(), "virtual_teensy.log"), "wb")
        except OSError:
            log = subprocess.DEVNULL
        try:
            self._vteensy = subprocess.Popen(
                [sys.executable, script, "--mode", "replay",
                 "--duration", "86400"],
                stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception as e:                                # noqa: BLE001
            self.log("virtual Teensy failed to start: %r" % (e,), bad=True)
            return None
        for _ in range(50):
            if os.path.exists(port_file):
                try:
                    p = open(port_file).read().strip()
                except OSError:
                    p = ""
                if p:
                    return p
            time.sleep(0.1)
        return None

    def _master_extra_args(self):
        """serial_port:= for the virtual Teensy when no real one exists."""
        if _teensy():
            return []
        port = self._ensure_virtual_teensy()
        if port:
            self.log("no master hardware -- the master path is driven by a "
                     "VIRTUAL TEENSY replaying a real recorded session "
                     "(%s). A DEMONSTRATION input, not trial evidence."
                     % port, bad=True)
            return ["serial_port:=%s" % port]
        self.log("no Teensy and no virtual one -- the stack will come up "
                 "and the master node will retry, then park DORMANT in "
                 "about 165 s. Nothing will drive the arms.", bad=True)
        return []

    def on_start_master(self):
        extra = self._master_extra_args()
        if self.ma_shared.isChecked():
            self._stack_extra_args = {"autonomy": extra}
            self.start_mode(["autonomy"], "SHARED AUTONOMY (master arm)")
        else:
            self._stack_extra_args = {"sim": extra}
            self.start_mode(["sim"], "MASTER TELEOP")

    def on_start_vr_clicked(self):
        """START VR TELEOP, with or without the robot helping.

        BOTH ROUTES NOW TAKE THE CHECKED BRING-UP. They did not: with the
        toggle off this called `on_vr_start`, which is `vr_bringup` -- twelve
        steps, each verified, each with a named repair. With the toggle ON it
        called `start_mode(["autonomy_nomaster", "vr"])` instead, which
        launches two specs and checks nothing. So the mode with MORE moving
        parts got the LESS careful start, and every bring-up defect this
        session found in the plain path was live and unwatched in the shared
        one. It also meant `_sim_argv`'s master:=false fix -- the one that
        stopped the stack dying -- applied only to plain VR.

        Now the bring-up is the same either way, and shared autonomy is a
        LAYER added on top of it once it has succeeded: perception plus the
        arbiter, with `teleop:=false` so it brings no second stack of its own.
        HARD CONSTRAINT 3 is the reason that argument matters -- the old path
        worked only because `start_mode` adopts what is already running, which
        is a property of the sequencer rather than of this launch.
        """
        self._vr_want_shared = bool(self.vr_shared.isChecked())
        if self._vr_want_shared:
            self.log("SHARED AUTONOMY (VR): the checked bring-up first, then "
                     "the arbiter on top of it")
        self.on_vr_start()

    def on_flow_full(self):
        self.start_mode(["autonomy_nomaster"], "FULL AUTONOMY")
        self.on_show_instruct()

    def on_flow_sim(self):
        if _stack_pids() <= 0:
            self.flow_note.setText(
                "Nothing is running yet -- press a START button in step 1 "
                "first. RViz opens on its own when the simulation is up.")
            self.log("OPEN THE SIM: no simulation running -- start step 1 "
                     "first")
            return
        self._ensure_rviz()
        self.flow_note.setText(
            "The sim is the COMMANDED view on the right. If the RViz panel "
            "is blank, this press relaunches it.")
        self.log("OPEN THE SIM: RViz ensured (embedded in COMMANDED)")

    def on_flow_real(self):
        self.start_mode(["real"], "DRIVE THE REAL ARMS")

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
        g = QGroupBox("REAL ARM SEQUENCE  --  step by step")
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

        # THE NUMBERED STEPS ARE GONE. One button does them, in order,
        # and works out which are still needed -- see `on_real_one`. What is
        # left here is the E-STOP above and the things you reach for when
        # something has already gone wrong.
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
            ("restart the relay", self.on_real_relay_restart,
             "Stops sim_to_real_bridge and starts it again on the same "
             "Kortex session. The relay is the only thing that puts the "
             "simulation onto the metal, and a code change to it only takes "
             "effect on a node restart -- symlink-install means the FILE is "
             "already current and the PROCESS is not."),
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

    def _svc(self, name, typ="std_srvs/srv/Trigger", then=None):
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
        self.bus.submit(lambda: self.bus.call_trigger(name, then=then),
                        label="calling %s" % name)

    # WHAT HAS TO BE RUNNING BEFORE A VR GRIP CAN REACH THE METAL, and how
    # each part is proved. Asked of the PROCESS TABLE, never of a flag this
    # window set: the operator's arms are frequently connected by a terminal
    # this window did not start, and a window that only believes its own
    # presses is a window that reports a healthy rig as absent.
    #
    #   kortex_highlevel_bridge_<arm>   the Kortex session. Without it there
    #                                   is no metal to command at all.
    #   sim_to_real_bridge_<arm>        THE RELAY. It reads the SIMULATED
    #                                   arm's joint states and republishes
    #                                   them on
    #                                   /real/<arm>_arm_controller/joint_trajectory,
    #                                   so whatever drives the sim -- VR, the
    #                                   master arm, a pose button -- drives
    #                                   the arm. NOTHING ELSE DOES THIS.
    @staticmethod
    def _arms_running(pattern):
        """Which of left/right have a live process matching `pattern % arm`.

        The same patterns `start_cascade.sh` matches on, so this window and
        that script cannot disagree about what is up.
        """
        found = []
        for a in ("left", "right"):
            try:
                if subprocess.run(["pgrep", "-f", pattern % a],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL,
                                  timeout=5).returncode == 0:
                    found.append(a)
            except Exception:                                 # noqa: BLE001
                pass
        return found

    def _kortex_arms(self):
        """Arms with an open Kortex session."""
        return self._arms_running("kortex_highlevel_bridge_%s")

    def _relay_arms(self):
        """Arms whose sim -> real relay is running."""
        return self._arms_running("sim_to_real_bridge_%s")

    def on_real_start(self):
        """Connect the arms -- OR RELAY ONTO ARMS ALREADY CONNECTED.

        THIS BUTTON COULD DISCONNECT THE ARMS. The arm permits exactly ONE
        Kortex session (HARD CONSTRAINT 2). `start_real.sh` opens one per
        arm, so against a rig whose bridges are already up the second session
        is refused, the launch fails, and the script's own cleanup then
        sweeps `kortex_highlevel_bridge` as a straggler on the way out --
        taking the WORKING sessions with it. `start_mode` has substituted
        `real_cascade` for exactly this reason since 2026-08-26, and this
        button went round it by shelling out to `start_real.sh` raw.

        It takes the same route now. Bridges up -> `start_cascade.sh`, which
        opens no session and starts the RELAY, which is the only thing in the
        rig that puts the simulation onto the metal.
        """
        self._real_guard("start real arms", self._real_start_go)

    def _real_start_go(self):
        live = self._kortex_arms()
        if live:
            self._run_raw(
                "start_cascade.sh",
                ["bash", os.path.join(_WS, "scripts/start_cascade.sh")] + live)
            self.real_head.setText(
                "%s connected -- starting the relay"
                % "/".join(a.upper() for a in live))
            self._real_say(
                "Kortex session already open on %s. Opening a second one is "
                "refused by the arm and would sweep the live one on the way "
                "out, so this starts sim_to_real_bridge on the open "
                "session(s) instead. NOTHING HAS MOVED AND NOTHING WILL YET: "
                "this path does no homing, and the relay will refuse while "
                "the real arm is far from the loaded home. Press "
                "2. HOME BOTH ARMS, then 4. START REAL ARM TELEOP."
                % ", ".join(live))
            return
        self._run_raw("start_real.sh",
                      ["bash", os.path.join(_WS, "scripts/start_real.sh"),
                       "arm:=both"])
        self.real_head.setText("starting both arms -- watch this window")
        self._real_say(
            "Opening one Kortex session per arm, then homing, then the "
            "sim -> real relay. Two arms share one link, so the script drops "
            "the command rate to 12 Hz per arm on purpose.")

    def on_real_home(self):
        """Home the connected arms -- AND MAKE SURE THERE IS SOMETHING TO ASK.

        THIS BUTTON WAS DEAD ON AN ADOPTED SESSION. It called `/home_arm_left`
        and `/home_arm_right` unconditionally, and those services are served
        by `real_homing_node`, which only `real_arms_highlevel.launch.py`
        started. Adopt an already-open Kortex session -- which is what every
        rig brought up outside this window is -- and the services did not
        exist, so the press answered "service not present, nothing was sent"
        and the arm stayed where it was.

        That is one half of the 2026-08-29 deadlock: the relay refuses until
        the arm is homed, and homing was unreachable, so the arm could never
        move. `start_cascade.sh` serves the homing node now (auto_home false,
        so it moves nothing on its own); this ensures it is up before asking,
        and asks only for arms that actually have a Kortex session.
        """
        self._real_guard("home", self._real_home_go)

    def _real_home_go(self):
        live = self._kortex_arms()
        if not live:
            self.real_head.setText("no Kortex session")
            self._real_say(
                "REFUSED: no kortex_highlevel_bridge is running, so there is "
                "no arm to home. Press 1. START REAL ARMS first.", bad=True)
            return
        missing = [a for a in live if a not in self._homing_arms()]
        if missing:
            self._run_raw(
                "start_cascade.sh",
                ["bash", os.path.join(_WS, "scripts/start_cascade.sh")]
                + missing)
            self._real_say(
                "The homing service was not being served on %s -- starting "
                "real_homing_node there (auto_home OFF, nothing moves yet). "
                "Asking again in 8 s." % ", ".join(missing))
            QTimer.singleShot(8000, lambda: self._real_guard(
                "home", lambda: self._real_home_ask(live)))
            return
        self._real_home_ask(live)

    def _homing_arms(self):
        """Arms whose homing service is actually being served."""
        return self._arms_running("real_homing_node_%s")

    def _real_home_ask(self, arms):
        up = self._homing_arms()
        for a in arms:
            if a in up:
                self._svc("/home_arm_%s" % a)
        dead = [a for a in arms if a not in up]
        if dead:
            self.real_head.setText(
                "NO HOMING NODE: %s" % "/".join(a.upper() for a in dead))
            self._real_say(
                "REFUSED for %s: real_homing_node_%s is not running, so "
                "/home_arm_%s is unserved and the press would go nowhere. "
                "See the event log for why start_cascade.sh could not start "
                "it." % (", ".join(dead), dead[0], dead[0]), bad=True)
            return
        self.real_head.setText(
            "HOMING %s -- watch the arm" % "/".join(a.upper() for a in arms))
        self._real_say(
            "Homing %s to the pose in config/home_positions_*.txt under the "
            "velocity law, clearance floor live. The arm MOVES now. Hand on "
            "the e-stop. The homing node's own reply is in the event log."
            % ", ".join(arms))

    def on_real_observer(self):
        self._real_guard("observer", lambda: (
            self._run_raw("observer_estop",
                          ["x-terminal-emulator", "-e", "python3",
                           os.path.join(_WS, "scripts/observer_estop.py")]),
            self._real_say("observer station opened -- it must stay running; "
                           "2 s of silence reads as withdrawal")))

    def on_real_teleop(self):
        """Hand the VR mapper the real arms -- AND ACTUALLY OPEN THE PATH.

        THIS BUTTON SET A PARAMETER NOBODY READ. It called
        `/vr/enable_real_arm` and nothing else. That service sets
        `allow_real_arm` on `vr_safety_node`, which publishes it inside the
        `/vr/safety` status string -- and grepping the whole tree for that
        parameter finds the node that declares it, the launch files that
        pass it, and NO CONSUMER. Nothing gates on it, nothing starts on it,
        nothing moves because of it. CLAUDE.md's own "feature present but
        does nothing" row: checked that a field is STORED, not that a
        consumer READS it.

        So the operator pressed the one control in the window that is
        supposed to let VR move real metal, the panel said "Asked for
        real-arm control", the service replied success, and the arms stood
        still -- with nothing anywhere contradicting it. Measured on this rig
        2026-08-29: Kortex session open on the left arm, VR chain running,
        and `sim_to_real_bridge` not running at all, so the simulation had no
        route to the arm whatever this service returned.

        The safety gate is still called and still governs whether this is
        ALLOWED. What was missing is the thing that makes it HAPPEN: the
        relay must be running on each connected arm and must be ENABLED
        (`/bridge_enable_<arm>`). This does both, per arm, and REFUSES BY
        NAME when it cannot -- naming the step to press instead.
        """
        self._real_guard("real arm teleop", self._real_teleop_go)

    def _real_teleop_go(self):
        live = self._kortex_arms()
        if not live:
            self.real_head.setText("no Kortex session")
            self._real_say(
                "REFUSED: no kortex_highlevel_bridge is running, so there is "
                "no arm for VR to drive and enabling the gate would change "
                "nothing. Press 1. START REAL ARMS first.", bad=True)
            return
        # The safety gate first: it is allowed to refuse, and if it does
        # there is no point starting a relay.
        self._svc("/vr/enable_real_arm")
        missing = [a for a in live if a not in self._relay_arms()]
        if missing:
            self._run_raw(
                "start_cascade.sh",
                ["bash", os.path.join(_WS, "scripts/start_cascade.sh")]
                + missing)
            self._real_say(
                "The sim -> real relay was NOT running on %s -- that is why "
                "the arm did not move. Starting sim_to_real_bridge there now; "
                "enabling in 8 s." % ", ".join(missing))
            QTimer.singleShot(8000, lambda: self._real_guard(
                "enable relay", lambda: self._real_enable_relays(live)))
            return
        self._real_enable_relays(live)

    def _real_enable_relays(self, arms):
        """Enable the relay on each arm, and SAY which arms are actually live.

        `sim_to_real_bridge` starts disabled unless told otherwise and can
        refuse to enable -- e-stop latched, homing still running, or the real
        arm too far from the loaded home to replay sim angles without
        commanding the difference as a jump (HARD CONSTRAINT 0). Those
        refusals arrive as the service response and are printed in the event
        log by `bus.call_trigger`, so a refusal is visible rather than being
        a silent arm.
        """
        up = self._relay_arms()
        self._relay_reply = {}
        for a in arms:
            if a in up:
                # THE RELAY IS ALLOWED TO SAY NO, AND USUALLY DOES.
                # `sim_to_real_bridge.enable()` refuses on a latched e-stop,
                # on homing still running, and -- the one that bites every
                # session -- when the real arm is further from the loaded
                # home than `enable_gap_rad`, because it replays sim angles
                # starting at home and enabling would command the difference
                # as a JUMP (HARD CONSTRAINT 0: sim home and real home
                # disagree on purpose). Measured on this rig 2026-08-29:
                # 2.729 rad of gap, relay running, `cascade_active_left`
                # FALSE, and the panel saying nothing at all.
                self._svc("/bridge_enable_%s" % a,
                          then=lambda ok, msg, a=a: self._relay_reply.__setitem__(
                              a, (ok, msg)))
        dead = [a for a in arms if a not in up]
        if dead:
            self.real_head.setText(
                "RELAY MISSING: %s" % "/".join(a.upper() for a in dead))
            self._real_say(
                "REFUSED for %s: sim_to_real_bridge_%s is not running, so "
                "nothing republishes the simulation onto "
                "/real/%s_arm_controller/joint_trajectory and the arm will "
                "not move however the gate is set. Look at the event log for "
                "why start_cascade.sh could not start it."
                % (", ".join(dead), dead[0], dead[0]), bad=True)
            return
        self.real_head.setText(
            "asking the relay: %s..." % "/".join(a.upper() for a in arms))
        self._real_say(
            "Real-arm control requested. Waiting for the relay's own answer "
            "-- it is allowed to refuse, and until it says yes nothing "
            "reaches the metal.")
        QTimer.singleShot(700, lambda: self._real_relay_verdict(arms))

    def _real_relay_verdict(self, arms, tries=12):
        """Say what the relay ACTUALLY answered. Never assume it said yes.

        THE PANEL USED TO CLAIM `VR -> REAL ARMED` THE MOMENT IT HAD ASKED.
        That is the same defect as the enable gate itself -- reporting the
        request instead of the result -- and on 2026-08-29 it hid a refusal
        the operator needed to see: `bridge_status_left` read enabled 0.0
        with a 2.729 rad gap while the window said the arms were armed.

        The refusal text is worth surfacing verbatim rather than
        summarising: `sim_to_real_bridge` names the joint, the distance and
        the runbook, and it is the only place that information exists.
        """
        pending = [a for a in arms if a not in self._relay_reply]
        if pending and tries > 0:
            QTimer.singleShot(500,
                              lambda: self._real_relay_verdict(arms, tries - 1))
            return
        good = [a for a in arms if self._relay_reply.get(a, (False,))[0]]
        bad = [(a, self._relay_reply.get(a, (False, "no reply"))[1])
               for a in arms if a not in good]
        if not bad:
            self.real_head.setText(
                "VR -> REAL ARMED on %s" % "/".join(a.upper() for a in good))
            self._real_say(
                "The relay is ENABLED on %s. Squeeze the grip: the "
                "simulation moves first and the metal follows it."
                % ", ".join(good))
            return
        self.real_head.setText(
            "RELAY REFUSED: %s" % "/".join(a.upper() for a, _ in bad))
        self._real_say(
            "%s%s"
            % ("; ".join("%s: %s" % (a, m.replace("\n", " ")) for a, m in bad),
               ("  --  If that names a distance from home, press "
                "2. HOME BOTH ARMS first: the relay replays the simulation "
                "STARTING at home, so it will not close a gap it did not "
                "command."
                if any("home" in m for _, m in bad) else "")),
            bad=True)

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

    # ------------------------------------------------- THE ONE BIG BUTTON
    def _one_say(self, text, bad=False):
        try:
            self.real_one_lbl.setText(text)
            self.real_one_lbl.setStyleSheet(
                "color:%s" % (C_BAD if bad else C_MUTED))
            self.bus.note("MOVE THE REAL ARMS: %s" % text, bad=bad)
        except Exception:                                     # noqa: BLE001
            pass

    def on_fix_cameras(self):
        """Run the camera doctor and show what it found. OFF the Qt thread.

        It pings arms, probes RTSP and subscribes to topics; inline that is
        tens of seconds with the window frozen, which is the defect already
        fixed twice in this file (`arm status`, `START SCENE CAMERA`).
        """
        try:
            self.cam_fix_lbl.setText("checking every camera, layer by "
                                     "layer -- up to a minute...")
            self.cam_fix_lbl.setStyleSheet("color:%s" % C_MUTED)
            self._cam_fix_out = None
            # ITS OWN THREAD, NOT `bus.submit`. THE AUDIT CAUGHT THIS AND IT
            # WAS THE WORST KIND OF DEFECT.
            #
            # The bus queue is drained SERIALLY by a 0.05 s timer, and the
            # E-STOP's publish goes through that same queue. This work is a
            # subprocess with a 300 s timeout -- pings, RTSP probes, topic
            # subscriptions -- so putting it on the bus queued the e-stop
            # behind it for up to five minutes. `verify_gui_buttons` failed
            # "e-stop reaches /estop" on BOTH e-stop buttons, which is
            # exactly what that check exists to catch.
            #
            # The bus is for short ROS actions. Anything that shells out
            # belongs on a thread of its own, and the result comes back
            # through the same polled label the rest of this panel uses.
            self.bus.note("camera doctor: checking every camera, layer by "
                          "layer")
            threading.Thread(target=self._fix_cameras_go,
                             daemon=True).start()
            QTimer.singleShot(1500, self._fix_cameras_render)
        except Exception as e:                                # noqa: BLE001
            self.log("camera doctor: %r" % (e,), bad=True)

    def _fix_cameras_go(self):
        try:
            r = subprocess.run(
                [sys.executable,
                 os.path.join(_WS, "scripts", "camera_doctor.py"), "--fix"],
                capture_output=True, text=True, timeout=300)
            lines = [ln.strip() for ln in (r.stdout or "").splitlines()
                     if ln.strip() and not ln.startswith("==")]
            self._cam_fix_out = lines or ["the doctor said nothing"]
            for ln in lines:
                self.bus.note("cameras: %s" % ln, bad=("BROKEN" in ln))
        except Exception as e:                                # noqa: BLE001
            self._cam_fix_out = ["camera doctor failed: %r" % (e,)]

    def _fix_cameras_render(self, tries=60):
        out = getattr(self, "_cam_fix_out", None)
        if out is None:
            if tries > 0:
                QTimer.singleShot(1000,
                                  lambda: self._fix_cameras_render(tries - 1))
            else:
                self.cam_fix_lbl.setText("the camera check did not finish "
                                         "-- see the event log")
            return
        bad = any("BROKEN" in ln for ln in out)
        self.cam_fix_lbl.setText("   |   ".join(out))
        self.cam_fix_lbl.setStyleSheet("color:%s" % (C_BAD if bad else C_OK))
        self._cam_fix_out = None

    def on_real_master(self):
        """Drive the real arms from the MASTER MANNEQUIN.

        Reuses `_real_one_go` for everything after the input, because
        everything after the input IS the same: both master_pose_node and
        vr_pose_mapper publish /master_arm_pose_<arm>, the followers consume
        that, and the relay carries the result to the metal. Duplicating the
        sequence here would be a second copy to keep in step with the first.
        """
        self._real_guard("move the real arms with the mannequin",
                         self._real_master_go)

    def _real_master_go(self):
        import glob as _glob
        # 1. THE BOARD. Without it master_pose_node has nothing to read, and
        #    it does not fail loudly -- it retries and parks, so the arm
        #    simply never moves.
        if not (_glob.glob("/dev/ttyACM*") or _glob.glob("/dev/ttyUSB*")):
            self._one_say(
                "REFUSED: no /dev/ttyACM* -- the Teensy is not attached, so "
                "the mannequin has no way to send anything. Attach it from "
                "Windows (FIX THE CAMERAS does it: usbipd owns the board and "
                "both cameras together), then press this again.", bad=True)
            return
        # 2. THE NODE. A rig brought up for VR runs `master:=false` on
        #    purpose, so this is absent rather than broken, and nothing in
        #    the window said so.
        # Asked directly rather than through `_arms_running`, which formats
        # an arm name into its pattern: master_pose_node is ONE node for both
        # arms, so there is no per-arm name to look for.
        try:
            up = subprocess.run(
                ["pgrep", "-f", "srl_teleop/master_pose_node"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5).returncode == 0
        except Exception:                                     # noqa: BLE001
            up = False
        if not up:
            self._one_say(
                "REFUSED: master_pose_node is not running. The VR modes "
                "start the stack with master:=false deliberately -- with no "
                "board on the bus that node respawns for ever and starves "
                "the controller manager. Restart the stack WITH the master "
                "arm: ros2 launch srl_teleop teleop.launch.py gate:=false "
                "master:=true use_rviz:=false", bad=True)
            return
        self._one_say(
            "Teensy attached and master_pose_node running. Bringing the real "
            "arms up on the mannequin -- same sequence as the button above.")
        self._real_one_go()

    def on_real_one(self):
        """The whole real-arm sequence, as one press, skipping what is done.

        A SEQUENCER, NOT A MACRO. It asks the process table and the nodes
        what is already true and does only the missing part, so pressing it
        twice is safe and pressing it against a half-built rig is the normal
        case rather than an error. Every stage says what it did.
        """
        self._real_guard("move the real arms", self._real_one_go)

    def _real_one_go(self):
        live = self._kortex_arms()
        if self._real_one_stage == "match" and live:
            self._real_one_stage = "idle"
            self._real_one_match(live)
            return
        if self._real_one_stage == "home" and live:
            # The operator has pressed a second time on a button that says
            # it will move the arm. That is the consent; take it and reset
            # the stage so a third press cannot re-home a moving arm.
            self._real_one_stage = "idle"
            self._real_one_home(live)
            return

        # 1. A Kortex session per arm. One, never two -- the arm permits
        #    exactly one (HARD CONSTRAINT 2), so with any already open this
        #    adopts them instead of opening more.
        if not live:
            self._one_say(
                "no Kortex session -- connecting the arms. This opens one "
                "session per arm and does NOT home them: if the arm and the "
                "simulation already agree, wherever they are, driving starts "
                "from there. Nothing moves yet.")
            # home:=false. THE ARM IS NOT DRIVEN ACROSS THE WORKSPACE JUST
            # TO START. This sequencer asks the relay whether it can enable
            # where the arm actually is, and only offers homing -- as an
            # explicit second press -- if the relay refuses on the gap. So a
            # rig sitting at the pick pose starts driving from the pick pose.
            self._run_raw("start_real.sh",
                          ["bash", os.path.join(_WS, "scripts/start_real.sh"),
                           "arm:=both", "home:=false"])
            QTimer.singleShot(25000, lambda: self._real_guard(
                "move the real arms", self._real_one_go))
            return

        # 2. The three things an ADOPTED session does not get from its
        #    launch file: the relay, the homing service, and the `real_*`
        #    frames the clearance guard measures against. Missing any one of
        #    them and the arm cannot move, each for a different reason.
        need = ([a for a in live if a not in self._relay_arms()]
                + [a for a in live if a not in self._homing_arms()])
        if need:
            self._run_raw(
                "start_cascade.sh",
                ["bash", os.path.join(_WS, "scripts/start_cascade.sh")]
                + sorted(set(need)))
            self._one_say(
                "%s connected. Starting the sim -> real relay, the homing "
                "service and the real_* frames. Nothing has moved."
                % "/".join(a.upper() for a in live))
            QTimer.singleShot(12000, lambda: self._real_guard(
                "move the real arms", self._real_one_go))
            return

        # 3. Ask the relay to enable. IT is the authority on whether the arm
        #    and the simulation agree closely enough, so rather than
        #    duplicating that comparison here, ask and read the answer.
        self._one_say(
            "%s ready. Asking the relay to enable..."
            % "/".join(a.upper() for a in live))
        self._relay_reply = {}
        for a in live:
            self._svc("/bridge_enable_%s" % a,
                      then=lambda ok, msg, a=a:
                      self._relay_reply.__setitem__(a, (ok, msg)))
        QTimer.singleShot(700, lambda: self._real_guard(
            "move the real arms", lambda: self._real_one_verdict(live)))

    def _real_one_verdict(self, arms, tries=12):
        pending = [a for a in arms if a not in self._relay_reply]
        if pending and tries > 0:
            QTimer.singleShot(500, lambda: self._real_guard(
                "move the real arms",
                lambda: self._real_one_verdict(arms, tries - 1)))
            return
        bad = [(a, self._relay_reply.get(a, (False, "no reply"))[1])
               for a in arms if not self._relay_reply.get(a, (False,))[0]]
        if not bad:
            self._real_one_stage = "idle"
            self.real_one_btn.setText("\u25b6  MOVE THE REAL ARMS")
            self._one_say(
                "ARMED on %s. The simulation now drives the metal -- squeeze "
                "the grip." % "/".join(a.upper() for a in arms))
            return
        # A refusal that names home is the one case this button can fix, and
        # fixing it MOVES THE ARM. So it asks again rather than doing it.
        if any(("home" in m) or ("gap" in m) or ("rad" in m)
               for _, m in bad):
            # MOVE THE SIMULATION TO THE ARM, NOT THE ARM TO THE SIMULATION.
            #
            # THE OPERATOR'S REPORT, 2026-08-30: "it asks for home pose by
            # default but I need the pickup pose."
            #
            # The relay refuses because the sim and the arm disagree, and
            # ANY way of making them agree satisfies it -- the gap check is
            # pose-agnostic. Homing was the expensive way round: it drives
            # real metal across the workspace, takes the arm away from
            # wherever the operator deliberately put it, and on a long move
            # trips the lag monitor on the way.
            #
            # The cheap way is the other direction. The simulation is free to
            # move; the arm is not. Matching the sim to the arm's CURRENT
            # pose closes the same gap, moves NO metal, takes about a second,
            # and leaves the arm exactly where the operator parked it -- at
            # the pick pose, the scan pose, or anywhere else.
            #
            # Homing stays available as its own button for when the arm is
            # somewhere it should not be. It is no longer the price of
            # starting.
            self._real_one_stage = "match"
            self.real_one_btn.setText(
                "\u25b6  MATCH THE SIM TO THE ARM  --  nothing moves")
            self._one_say(
                "%s  --  press again to bring the SIMULATION to where the "
                "arm actually is. No metal moves, and you keep the pose you "
                "parked in. (To move the ARM instead, use HOME.)"
                % "; ".join("%s: %s" % (a, m.replace("\n", " "))
                            for a, m in bad), bad=True)
            return
        self._real_one_stage = "idle"
        self._one_say("; ".join("%s: %s" % (a, m.replace("\n", " "))
                                for a, m in bad), bad=True)

    def _real_one_match(self, arms):
        """Drive the SIMULATION to where each real arm actually is.

        Nothing physical moves. `send_joint_pose` takes the arm-controller
        topic through `/pose_move_active` for the duration, so the mapper
        cannot fight it, and the relay's gap check then passes wherever the
        arm happens to be standing.
        """
        moved = []
        for a in arms:
            names = ["%s_joint_%d" % (a, i) for i in range(1, 8)]
            cur = getattr(self.bus, "_real_q", {}) or {}
            q = [cur.get(n) for n in names]
            if not all(v is not None for v in q):
                self._one_say(
                    "cannot read the %s arm's joints, so there is nothing to "
                    "match the simulation to. Is the Kortex bridge "
                    "publishing?" % a, bad=True)
                continue
            ok, topic = self.bus.send_joint_pose(a, [float(v) for v in q],
                                                 secs=2.0)
            if ok:
                moved.append(a)
        if not moved:
            return
        self.real_one_btn.setText("\u25b6  MOVE THE REAL ARMS")
        self._one_say(
            "matching the simulation to %s. Nothing physical is moving. "
            "Trying to arm again in 6 s." % ", ".join(moved))
        QTimer.singleShot(6000, lambda: self._real_guard(
            "move the real arms", self._real_one_go))

    def _real_one_home(self, arms):
        """Second press: home, then come back and try to arm again."""
        up = self._homing_arms()
        for a in arms:
            if a in up:
                self._svc("/home_arm_%s" % a)
        self._real_one_stage = "homing"
        self.real_one_btn.setText("\u25b6  MOVE THE REAL ARMS")
        self._one_say(
            "homing %s. The arm is moving. When it stops this will try to "
            "arm again by itself." % "/".join(a.upper() for a in arms))
        QTimer.singleShot(30000, lambda: self._real_guard(
            "move the real arms", self._real_one_go))

    def on_real_relay_restart(self):
        """Restart the sim -> real relay without touching the Kortex session.

        THE TERMINAL THIS REPLACES. Every relay problem this session ended in
        `pkill -INT -f sim_to_real_bridge_left` followed by
        `bash scripts/start_cascade.sh left`, typed at a prompt, because the
        window could start the relay and could not restart one that was
        already running -- and a relay whose retry thread had died looked
        exactly like a healthy one. THE GUI RULE: a capability reachable only
        from a terminal does not exist for the person running the session.

        SIGINT, never SIGKILL, and by explicit PID -- HARD CONSTRAINT 9. The
        Kortex session belongs to `kortex_highlevel_bridge` and is not
        touched here, so this is safe to press repeatedly: the arm keeps its
        one session throughout.
        """
        self._real_guard("restart relay", self._real_relay_restart_go)

    def _real_relay_restart_go(self):
        live = self._kortex_arms()
        if not live:
            self.real_head.setText("no Kortex session")
            self._real_say(
                "REFUSED: no kortex_highlevel_bridge is running, so there is "
                "nothing to relay onto. Press 1. START REAL ARMS first.",
                bad=True)
            return
        killed = []
        for a in live:
            try:
                out = subprocess.run(
                    ["pgrep", "-f", "sim_to_real_bridge_%s" % a],
                    capture_output=True, text=True, timeout=5).stdout
            except Exception:                                 # noqa: BLE001
                out = ""
            for pid in [int(x) for x in out.split() if x.strip().isdigit()]:
                try:
                    os.kill(pid, signal.SIGINT)
                    killed.append(pid)
                except Exception:                             # noqa: BLE001
                    pass
        self.real_head.setText(
            "restarting the relay: %s" % "/".join(a.upper() for a in live))
        self._real_say(
            "SIGINT to %s. The Kortex session is untouched. Starting a fresh "
            "relay in 3 s; press 4. START REAL ARM TELEOP after it is up."
            % (("relay pid(s) " + ", ".join(str(p) for p in killed))
               if killed else "no running relay (there was none)"))
        QTimer.singleShot(3000, lambda: self._real_guard(
            "restart relay", lambda: self._run_raw(
                "start_cascade.sh",
                ["bash", os.path.join(_WS, "scripts/start_cascade.sh")]
                + live)))

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
                # THE RELAY, NAMED. Network and session both UP with no
                # sim_to_real_bridge is a rig that looks connected and
                # cannot be driven -- exactly the state this panel used to
                # report as healthy while VR moved nothing.
                relay = subprocess.run(
                    ["pgrep", "-f", "sim_to_real_bridge_%s" % a],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL).returncode == 0
                bits.append("%s: net %s, session %s, sim->real relay %s"
                            % (a.upper(), "UP" if up else "DOWN",
                               "up" if sess else "down",
                               "UP" if relay else "MISSING"))
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
        g = QGroupBox("SAY WHAT TO PICK UP")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        # WHAT IS ON THE TABLE, before naming anything.
        #
        # Until 2026-08-22 this panel could only answer "where is the thing I
        # named". Asked what was on the surface at all, the system had no
        # answer -- so a wrong name produced a move to somewhere plausible
        # and empty, which is what "it just moves here and there" looks like
        # from the outside.
        b = QPushButton("QUICK LOOK   (one camera frame)")
        b.setFont(helvetica(10, True))
        b.setMinimumHeight(28)
        # PREFERRED, NOT IGNORED, AND THAT IS WHAT WAS CLIPPING THE LABELS.
        #
        # `_left_width()` sizes the control column from `minimumSizeHint()`,
        # and a button whose horizontal policy is `Ignored` reports almost
        # nothing for that -- so no button label ever pushed the column wide
        # enough to show itself. Measured from a screenshot: "SIM + REAL"
        # rendered as a red sliver, "REMEMBER VIEW" as "R", "3. PICK IT UP"
        # as "3. PIC". The column was asking Qt how wide it needed to be and
        # Qt was answering about widgets that had been told not to care.
        #
        # `Preferred` makes the hint reflect the text. The 620 px ceiling in
        # `_left_width` still stops one long label eating the RViz panel.
        b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        b.setToolTip(
            "A single glance from where the gripper is now -- no arm movement, "
            "no full scan. Tells you what is in front of that one camera "
            "and which of it could be picked up. For a proper answer over "
            "the whole table, use FULL SCAN.")
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
            "The gripper camera can judge DISTANCE, so it can tell the arm "
            "where to go. The fixed room camera only sees colour -- it can "
            "tell you something is there and roughly where in the picture, but not how far away, so it cannot be used to grab something.")
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

        # THE STANDING VISION LAYER, distinct from the one-shot LOOK above:
        # scene_understanding_node watches every camera continuously and
        # this line is its per-camera verdict -- including the refusals,
        # which are the part an operator needs when a camera is unplugged.
        self.vis_scene = QLabel("scene vision not running (button in "
                                "Checks and fixes)")
        self.vis_scene.setFont(mono(8))
        self.vis_scene.setWordWrap(True)
        self.vis_scene.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vis_scene)

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
    # THE MODES LIVE IN THE THREE-CLICK PANEL NOW (2026-08-27). The OPERATE
    # panel that stood here -- five prose rows x three buttons each -- was
    # absorbed into `_flow_panel`: the operator counted the sections and
    # asked for one. A mode is still a SEQUENCE of specs (VR is the sim
    # stack AND the transport; "real" is any of them plus the cascade), and
    # `start_mode` below is still the only thing that runs one.
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
            self._ensure_rviz()
            return
        key = seq[_i]
        # A MODE IS AN END STATE, NOT A LIST OF PROCESSES TO SPAWN.
        #
        # This is the defect the operator hit on 2026-08-26, twice in one
        # button row. Every step was launched unconditionally, so:
        #
        #   SIM ONLY, with a stack already up -- `sim` carries
        #   starts_stack=True and the preflight refuses it under HARD
        #   CONSTRAINT 3. Correct, and useless: the refusal scrolled past in
        #   a status line, the sequence stopped at step 1, the VR transport
        #   at step 2 never ran, and from the operator's side the button
        #   simply did nothing.
        #
        #   SIM + REAL, with the arms already connected -- `real` runs
        #   `start_real.sh`, which starts a Kortex bridge per arm. The arm
        #   permits exactly ONE session (HARD CONSTRAINT 2), so the second
        #   is refused, the launch fails, and its cleanup sweeps the WORKING
        #   bridges as orphaned stack processes. The button labelled
        #   "cascade to the real arms" DISCONNECTED them.
        #
        # Both are the same mistake: asking "have I launched this?" when the
        # question is "is this up?". Each step is now probed against the
        # process table first. Already up -> skipped and SAID SO, never
        # silently. Reachable another way -> substituted, and the
        # substitution is named. Neither hides a refusal.
        key, sub_why = self._step_substitute(key)
        sp = specs.get(key)
        if sp is None:
            self._seq_say("%s: no launch spec %r -- stopping here."
                          % (name, key), bad=True)
            return
        if sub_why:
            self._seq_say("%s: step %d of %d -- %s"
                          % (name, _i + 1, len(seq), sub_why))
        already = self._step_running(key)
        if already:
            self._seq_say("%s: step %d of %d -- %s is ALREADY RUNNING (%s), "
                          "adopting it." % (name, _i + 1, len(seq), sp.label,
                                            already))
            QTimer.singleShot(300,
                              lambda: self.start_mode(seq, name, _i + 1))
            return
        # THE REAL CASCADE NEEDS THE CONTROLLERS PUBLISHING, NOT MERELY
        # PROCESSES EXISTING.
        #
        # Measured 2026-08-26: SIM + REAL fired `start_real.sh` as soon as the
        # stack's PIDs appeared. Its step 1 of 6 checks /joint_states, which
        # `joint_state_broadcaster` had not finished spawning -- so it refused
        # with "Is the sim controller up?" and its own cleanup then SWEPT THE
        # ARM BRIDGES as orphaned stack processes. Two working Kortex sessions
        # were killed by a race between two things that were both fine.
        if getattr(sp, "needs_real", False):
            age = self.bus.sim_js_age()
            if age is None or age > 1.0:
                if time.monotonic() > getattr(self, "_seq_deadline", 0):
                    self._seq_say(
                        "%s: step %d of %d (%s) needs /joint_states and it "
                        "never started publishing. The simulation's "
                        "joint_state_broadcaster did not come up -- look in "
                        ".scratch/launch_*.log for a spawner that timed out. "
                        "NOT launching the real cascade: it would refuse and "
                        "sweep any running bridges on its way out."
                        % (name, _i + 1, len(seq), key), bad=True)
                    return
                self._seq_say("%s: step %d of %d (%s) -- waiting for "
                              "/joint_states from the simulation..."
                              % (name, _i + 1, len(seq), key))
                QTimer.singleShot(1000, lambda: self.start_mode(seq, name, _i))
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

    # What proves each step is up, as a pattern for `pgrep -f` plus the
    # plain-words name of the thing found. Probed against the PROCESS TABLE
    # and not against a flag this window set, because the operator's stack is
    # frequently one this window did not start -- `srl_bringup_all.sh` brings
    # up everything before the GUI, which is the documented order.
    STEP_PROC = {
        "sim":               ("lib/moveit_ros_move_group/move_group",
                              "move_group"),
        "sim_nomaster":      ("lib/moveit_ros_move_group/move_group",
                              "move_group"),
        "autonomy":          ("lib/moveit_ros_move_group/move_group",
                              "move_group"),
        "autonomy_nomaster": ("lib/moveit_ros_move_group/move_group",
                              "move_group"),
        "vr":                ("quest_bridge_node", "quest_bridge_node"),
        "real":              ("kortex_highlevel_bridge", "a Kortex bridge"),
        "real_cascade":      ("sim_to_real_bridge", "sim_to_real_bridge"),
    }

    def _step_running(self, key):
        """Is this step's end state already reached? The evidence, or None."""
        pat = self.STEP_PROC.get(key)
        if pat is None:
            return None
        try:
            n = subprocess.run(["pgrep", "-fc", pat[0]], capture_output=True,
                               text=True, timeout=5).stdout.strip()
            n = int(n or 0)
        except Exception:                                      # noqa: BLE001
            return None
        return ("%d x %s" % (n, pat[1])) if n > 0 else None

    def _step_substitute(self, key):
        """The step to actually run, and why, when the rig is already part-up.

        One substitution today, and it is the one that was destroying live
        Kortex sessions: `real` starts a bridge per arm, so with bridges
        already running it must become `real_cascade`, which relays onto them
        and opens nothing. Returns (key, reason-or-None).
        """
        if key == "real" and self._step_running("real"):
            return ("real_cascade",
                    "the arms are ALREADY CONNECTED, so cascading onto the "
                    "open Kortex sessions instead of opening new ones "
                    "(start_real.sh would be refused and would sweep them "
                    "on its way out)")
        return (key, None)

    def _ensure_rviz(self):
        """Redraw the COMMANDED view if this window's RViz has died.

        The COMMANDED panel is half of what the operator watches, and an
        RViz that exits leaves it blank with the mode still running -- which
        reads as "the mode did nothing". A mode sequence is exactly the
        moment to notice, because it is the moment the operator starts
        looking at that panel.

        THROUGH `start_rviz`, NOT A BARE Popen. This window EMBEDS RViz into
        its own layout where a window manager exists; a plain `rviz2` here
        would open a second, unembedded, floating window that hides the one
        the operator is using. Restarting is the existing path's job.
        """
        try:
            if _stack_pids() <= 0:
                return
            alive = [k for k, p in getattr(self, "rviz", []) if p.poll() is None]
            if alive:
                return
            if getattr(self.args, "no_rviz", False):
                return
        except Exception:                                      # noqa: BLE001
            return
        self.bus.note("the COMMANDED view had no RViz -- restarting it")
        self.start_rviz()

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
        g = QGroupBox("DATA  --  record trials, CSV, graphs")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        v.setSpacing(3)

        # The stated default must match the actual combo defaults, which
        # moved to T0 / shared autonomy on 2026-08-27.
        intro = QLabel(
            "Defaults work: RUN TRIAL records T0 under shared autonomy, "
            "CSV + video on.")
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
        # m0 pairs with the shared-autonomy default below; m1 (the old
        # default) is locked to 06 by its own spec, so defaulting to it
        # alongside mode 03 would make RUN TRIAL open on a refusal.
        i = self.exp_task.findData("m0")
        if i >= 0:
            self.exp_task.setCurrentIndex(i)
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
        # SHARED AUTONOMY IS THE DEFAULT (operator, 2026-08-27): the study's
        # condition of interest, wanted on every run unless deliberately
        # changed. The change is logged like every other, and tasks locked
        # to another mode still refuse by name at dispatch.
        i = self.exp_mode.findData("03_shared_autonomy")
        if i >= 0:
            self.exp_mode.setCurrentIndex(i)
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
            b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            v.addWidget(b)
            self.buttons.setdefault("exp_" + text.split()[1].lower(), b)

        self.exp_status = QLabel("nothing run from this panel yet")
        self.exp_status.setWordWrap(True)
        self.exp_status.setFont(helvetica(9))
        self.exp_status.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.exp_status)

        # ===================================================================
        # RECORD EVERYTHING -- one press, the whole run on disk
        # ===================================================================
        # Independent of RUN TRIAL on purpose. Most of what the operator wants
        # captured (VR clutch behaviour, master arm channels, a camera that is
        # or is not detecting) happens during SET-UP and free driving, not
        # inside a scripted trial -- so the capture must not be welded to one.
        self.rec_all_btn = QPushButton("\u25cf  RECORD EVERYTHING")
        self.rec_all_btn.setFont(helvetica(11, True))
        self.rec_all_btn.setMinimumHeight(34)
        self.rec_all_btn.setToolTip(
            "Starts `ros2 bag record -a` (EVERY topic: cameras, detections, "
            "TF, joint states, master, VR, autonomy) plus a uniform trail.csv, "
            "an events log, and the REAL arms read straight off the Kortex "
            "API. Press again to stop; graphs and a summary are written on "
            "stop. The summary NAMES any channel that never published, so a "
            "recording cannot quietly come out empty.")
        self.rec_all_btn.clicked.connect(self.on_record_all_toggle)
        self.buttons["record_everything"] = self.rec_all_btn
        v.addWidget(self.rec_all_btn)

        # NAME THE RUN BEFORE YOU RECORD IT.
        #
        # The session directory is `<timestamp>_<label>`, and the label was
        # whatever the experiment panel happened to hold -- which for a
        # free-driving session is nothing, so every run landed as
        # `20260830_051727_20260830_051726`: a timestamp twice, and no way to
        # tell one from another afterwards without opening each summary.
        # `docs/RECORDINGS.md` exists because that was already painful.
        #
        # A name typed HERE, next to the button that starts the run, is the
        # cheapest possible fix: `20260830_051727_pick_left_timid` sorts, and
        # says what it is. Empty is still allowed and still works.
        nrow = QHBoxLayout()
        nrow.setSpacing(4)
        ncap = QLabel("name this run")
        ncap.setFont(helvetica(9))
        ncap.setStyleSheet("color:%s" % C_MUTED)
        nrow.addWidget(ncap)
        self.rec_name = QLineEdit()
        self.rec_name.setFont(helvetica(9))
        self.rec_name.setPlaceholderText(
            "e.g. pick_left_timid  --  optional, but it is how you find it "
            "again")
        self.rec_name.setToolTip(
            "Goes into the folder name: recordings/sessions/"
            "<date>_<time>_<name>. Letters, numbers, - and _ are kept and "
            "anything else is dropped, so the name is always a safe "
            "directory. Set it BEFORE pressing record; renaming afterwards "
            "means renaming a directory by hand.")
        nrow.addWidget(self.rec_name, 1)
        v.addLayout(nrow)
        self.rec_all_lbl = QLabel("not recording")
        self.rec_all_lbl.setFont(helvetica(9))
        self.rec_all_lbl.setWordWrap(True)
        self.rec_all_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.rec_all_lbl)

        # ===================================================================
        # MOVE THE REAL ARMS -- the ONLY real-arm start in this window
        # ===================================================================
        # THERE WERE FOUR, AND THAT IS WHY THE ARM DID NOT MOVE.
        #
        # `START REAL ARMS` on the mode panel, then `1. START REAL ARMS`,
        # `2. HOME BOTH ARMS` and `4. START REAL ARM TELEOP` in the sequence
        # panel. Each did part of the job and refused for a different reason,
        # the order mattered and was not enforced, and a refusal at step 4
        # was cured at step 2. On 2026-08-29 every one of them was pressed,
        # several times, and the arm never moved -- because the missing piece
        # was a fifth thing none of them started.
        #
        # This is one button and it is a SEQUENCER. It asks what is already
        # true and does only what is missing: open or adopt the Kortex
        # session, bring up the relay, the homing service and the `real_*`
        # frames, put the arm at home, then enable the relay. Every stage
        # says what it did, and a refusal is reported with the refusing
        # node's own words.
        #
        # IT MOVES METAL, SO IT ASKS TWICE. When the arm needs homing the
        # button changes to say so and waits for a SECOND press. Not a modal
        # dialog: one hung the button audit for weeks, and a window that can
        # block behind a dialog is a window whose e-stop can too.
        self.real_one_btn = QPushButton("\u25b6  MOVE THE REAL ARMS")
        self.real_one_btn.setFont(helvetica(11, True))
        self.real_one_btn.setMinimumHeight(34)
        self.real_one_btn.setStyleSheet(
            "color:%s;border:1px solid %s" % (C_OK, C_OK))
        self.real_one_btn.setToolTip(
            "The whole real-arm sequence, in order, skipping whatever is "
            "already done: adopt or open the Kortex session (one per arm -- "
            "it never opens a second), start the sim -> real relay, the "
            "homing service and the real_* frames the clearance guard needs, "
            "home the arm, then enable the relay so the simulation drives "
            "the metal. Press it again at any point; it picks up where the "
            "rig actually is. It asks a second time before anything moves.")
        self.real_one_btn.clicked.connect(self.on_real_one)
        self.buttons["move_the_real_arms"] = self.real_one_btn
        self.mode_btn["__real__"] = self.real_one_btn
        v.addWidget(self.real_one_btn)
        # ONE BUTTON FOR EVERY CAMERA, because after a WSL restart NOTHING
        # starts them. The USB devices come back on their own (the watchdog
        # re-attaches them from Windows) and no camera NODE does: not the
        # scene camera, not either wrist. The operator's question on
        # 2026-08-30 was exactly "if i shutdown wsl and turn it back again
        # will the cameras still work", and the honest answer was "the
        # devices yes, the nodes no". This is the missing half.
        #
        # It runs `camera_doctor.py --fix`, which walks the layers IN ORDER
        # and stops at the first one that is wrong -- stale shared memory,
        # device not attached, device held by something else, node not
        # running, no frames at the publisher's QoS -- because a fault low
        # down makes every answer above it meaningless. It repairs the three
        # that need no human and NAMES the two that do: a wedged vision
        # module needs the arm power-cycled, and a device held by another
        # process needs that process to let go.
        self.cam_fix_btn = QPushButton("FIX THE CAMERAS")
        self.cam_fix_btn.setFont(helvetica(10, True))
        self.cam_fix_btn.setMinimumHeight(28)
        self.cam_fix_btn.setToolTip(
            "Checks the scene camera and both wrist cameras layer by layer "
            "and repairs what it can: clears stale shared memory (only with "
            "nothing running), re-attaches a USB camera from Windows, and "
            "starts a node that is not running. It will not power-cycle an "
            "arm or take a device away from another process -- it says which "
            "and why instead. Safe to press at any time.")
        self.cam_fix_btn.clicked.connect(self.on_fix_cameras)
        self.buttons["fix_the_cameras"] = self.cam_fix_btn
        v.addWidget(self.cam_fix_btn)
        self.cam_fix_lbl = QLabel("cameras not checked yet")
        self.cam_fix_lbl.setFont(helvetica(9))
        self.cam_fix_lbl.setWordWrap(True)
        self.cam_fix_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.cam_fix_lbl)

        # THE SAME SEQUENCE, DRIVEN BY THE MANNEQUIN INSTEAD OF THE HEADSET.
        #
        # Everything downstream of the input is identical -- master_pose_node
        # and vr_pose_mapper both publish /master_arm_pose_<arm>, the
        # followers turn that into joint trajectories, and the relay carries
        # those to the metal. So this button does NOT duplicate the real-arm
        # sequence; it checks the two things the mannequin path needs that
        # the VR path does not, and then hands over to exactly the same
        # sequencer.
        #
        # Those two things are the whole difference: the Teensy has to be
        # attached, and master_pose_node has to be running. The VR modes
        # start the stack with `master:=false` on purpose -- with no board on
        # the bus that node dies and respawns for ever and starves the
        # controller manager -- so a rig brought up for VR has no master node
        # at all, and the mannequin then moves nothing with no explanation.
        self.real_master_btn = QPushButton(
            "\u25b6  MOVE THE REAL ARMS WITH MASTER MANNEQUIN")
        self.real_master_btn.setFont(helvetica(11, True))
        self.real_master_btn.setMinimumHeight(34)
        self.real_master_btn.setStyleSheet(
            "color:%s;border:1px solid %s" % (C_OK, C_OK))
        self.real_master_btn.setToolTip(
            "The same real-arm sequence as the button above, for the MASTER "
            "MANNEQUIN instead of the VR controllers. It first checks the "
            "two things the mannequin needs and the headset does not: the "
            "Teensy attached at /dev/ttyACM*, and master_pose_node running. "
            "It refuses by name if either is missing rather than arming a "
            "path with no input on it.")
        self.real_master_btn.clicked.connect(self.on_real_master)
        self.buttons["move_real_arms_master"] = self.real_master_btn
        v.addWidget(self.real_master_btn)

        self.real_one_lbl = QLabel("the real arms are not started")
        self.real_one_lbl.setFont(helvetica(9))
        self.real_one_lbl.setWordWrap(True)
        self.real_one_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.real_one_lbl)
        self._real_one_stage = "idle"
        self._rec_all_proc = None
        self._rec_all_timer = QTimer(self)
        self._rec_all_timer.timeout.connect(self._rec_all_refresh)
        self._rec_all_timer.start(2000)

        b = QPushButton("open the recordings folder")
        b.setFont(helvetica(9))
        b.setToolTip("Where every trial's manifest, CSV, plots and clips "
                     "land: recordings/sessions/<session>/")
        b.clicked.connect(self.on_open_recordings)
        v.addWidget(b)

        # RESULTS, ONE PRESS. Every figure the committed recordings support,
        # regenerated into recordings/analysis/ with an index naming each
        # figure's source file and date -- and the figures it CANNOT build
        # are listed with the reason, never drawn empty.
        b2 = QPushButton("MAKE ALL GRAPHS  (results)")
        b2.setFont(helvetica(10, True))
        b2.setMinimumHeight(30)
        b2.setToolTip("Runs scripts/make_results.py: accuracy per mode, "
                      "grasp matrix, VR smoothing and protocol runs, motion "
                      "generator, sim-to-real park error, grip traces, "
                      "mode path lengths. Skips are named in index.md.")
        b2.clicked.connect(self.on_make_results)
        v.addWidget(b2)
        self.results_lbl = QLabel("no graphs made this session yet")
        self.results_lbl.setFont(helvetica(8))
        self.results_lbl.setWordWrap(True)
        self.results_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.results_lbl)
        return g

    def on_make_results(self):
        """Regenerate every figure, in the background, and say what came
        out -- counts from the script's own stdout, never assumed."""
        script = os.path.join(_WS, "scripts", "make_results.py")
        if not os.path.exists(script):
            self.results_lbl.setText("scripts/make_results.py is missing")
            self.log("MAKE ALL GRAPHS: script missing", bad=True)
            return
        self.results_lbl.setText("making graphs...")
        self.log("MAKE ALL GRAPHS: running scripts/make_results.py")

        def go():
            try:
                r = subprocess.run(
                    [sys.executable, script], cwd=_WS, timeout=300,
                    capture_output=True, text=True)
                built = sum(1 for ln in r.stdout.splitlines()
                            if ln.strip().startswith("BUILT"))
                skipped = sum(1 for ln in r.stdout.splitlines()
                              if ln.strip().startswith("SKIP"))
                msg = ("%d figure(s) in recordings/analysis/ (%d skipped, "
                       "reasons in index.md)" % (built, skipped))
                if r.returncode != 0:
                    msg = "FAILED (exit %d): %s" % (
                        r.returncode, (r.stdout + r.stderr)[-200:])
            except Exception as e:                            # noqa: BLE001
                msg = "could not run it: %r" % (e,)
            QTimer.singleShot(0, lambda: (
                self.results_lbl.setText(msg),
                self.log("MAKE ALL GRAPHS: %s" % msg)))

        threading.Thread(target=go, daemon=True).start()

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
            # NOT full_state_recorder. That is a GUIDED SNAPSHOT tool: it
            # blocks on input() waiting for ENTER on a terminal, and this
            # launches detached with stdout at DEVNULL, so every trial
            # "recorded" through this tick captured nothing and said nothing.
            self._start_record_all(
                label="trial%d_%s" % (self.exp_trial.value(),
                                      self.exp_task.currentData()),
                mode=self.exp_mode.currentData())
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

    # ------------------------------------------------ RECORD EVERYTHING
    def _record_all_script(self):
        return os.path.join(_WS, "scripts/record_all.py")

    def _record_all_python(self):
        """The kortex venv if it is there, so the REAL arms get read too.

        Falls back to the system interpreter, which records everything except
        the direct Kortex read -- and record_all.py says so in arms.jsonl
        rather than leaving the absence to be discovered later.
        """
        kv = os.path.join(_WS, ".kortex_venv/bin/python")
        return kv if os.path.exists(kv) else "python3"

    def _record_all_live(self):
        """The live session dir, or None. Read from the marker record_all
        writes, not from our own handle, so a session started from a terminal
        is still visible here."""
        marker = os.path.join(_WS, "recordings/sessions/.current")
        try:
            info = json.load(open(marker))
        except Exception:                                     # noqa: BLE001
            return None
        if not os.path.exists("/proc/%d" % int(info.get("pid", -1))):
            return None
        return info

    def _start_record_all(self, label="session", mode=None):
        if self._record_all_live():
            self.log("RECORD EVERYTHING: already recording")
            return
        argv = [self._record_all_python(), self._record_all_script(),
                "--label", label]
        if mode:
            argv += ["--mode", str(mode)]
        # SAY WHAT THE NEXT RUN IS RECORDED AS. The GUI rule: a control that
        # changes what the run does must log its new value.
        # THE ACTUAL TICKS, not a guess. Both mode tabs carry their own
        # shared-autonomy toggle; either one being on makes the next START a
        # shared run, and the recording must be labelled with the condition it
        # was made under or it is not a comparison.
        # WHICH INPUT, AND WHOSE SHARED-AUTONOMY TICK.
        #
        # This was `ma_shared.isChecked() or vr_shared.isChecked()` -- an OR
        # across two tabs. It recorded THAT shared autonomy was on and lost
        # WHICH mode it belonged to, so ticking it on the VR tab and then
        # driving with the mannequin produced a run labelled `on` that the
        # data could not contradict. For E2, whose whole design is a
        # condition per run, that is a mislabelled trial rather than a
        # missing field.
        #
        # And the input itself was never recorded at all: `vr_pose_mapper`
        # and `master_pose_node` both publish /master_arm_pose_<arm>, so the
        # `master_*` columns cannot tell them apart afterwards.
        try:
            vr_on = bool(self.vr_shared.isChecked())
            ma_on = bool(self.ma_shared.isChecked())
        except Exception:                                     # noqa: BLE001
            vr_on = ma_on = False
        mode_now = self._live_input_mode()
        shared = ("on" if (vr_on if mode_now == "vr" else ma_on) else "off")
        argv += ["--shared-autonomy", shared]
        argv += ["--input-mode", mode_now]
        argv += ["--shared-vr", "on" if vr_on else "off"]
        argv += ["--shared-master", "on" if ma_on else "off"]
        self._run_raw("record_all", argv)
        self.log("RECORD EVERYTHING started: label=%s mode=%s shared_autonomy=%s"
                 % (label, mode or "unspecified", shared))
        QTimer.singleShot(1200, self._rec_all_refresh)

    def _stop_record_all(self):
        """Stop the session, AND SAY SO THE INSTANT THE BUTTON IS PRESSED.

        THE DEFECT, reported 2026-08-30 as "the recording button is not
        stopping". It was stopping. A stop flushes the bag cache, compresses
        the mcap and renders five figures, which takes about five seconds --
        and for all five the button still read STOP RECORDING in red, because
        the state comes from the live-session marker and the marker is only
        removed at the END of that work. So the press had no visible effect,
        which from the operator's side is a dead button, and the natural
        response is to press it again.
        --
        The button now changes on the press itself and the refresh takes over
        when the session is really gone. Same rule as the e-stop label fixed
        earlier tonight: a control that acts without acknowledging cannot be
        told from one that is wired to nothing.
        """
        try:
            self._rec_stopping = time.time()
            self.rec_all_btn.setText("\u25a0  STOPPING...")
            self.rec_all_btn.setStyleSheet(
                "background:%s; color:white; font-weight:bold" % C_WARN)
            self.rec_all_btn.setEnabled(False)
            self.rec_all_lbl.setText(
                "closing the session: flushing the bag, compressing it and "
                "drawing the figures. About five seconds -- the graphs and "
                "the summary are written now, so this is worth waiting for.")
            self.rec_all_lbl.setStyleSheet("color:%s" % C_WARN)
        except Exception:                                     # noqa: BLE001
            pass
        try:
            subprocess.Popen(
                [self._record_all_python(), self._record_all_script(), "--stop"],
                start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.log("RECORD EVERYTHING stopping -- graphs and summary are "
                     "written on stop, which takes a few seconds")
        except Exception as e:                                # noqa: BLE001
            self.log("RECORD EVERYTHING stop FAILED: %r" % e, bad=True)
        # Poll through the stop rather than once at 3 s: the work takes
        # about five seconds and a single check at three is a coin flip.
        for ms in (1500, 3000, 5000, 7000, 10000, 15000):
            QTimer.singleShot(ms, self._rec_all_refresh)

    def on_record_all_toggle(self):
        if self._record_all_live():
            self._stop_record_all()
        else:
            # THE TYPED NAME WINS. The experiment panel's session id is the
            # right label when a scripted trial is running; for the free
            # driving that most recordings actually are, it is a second
            # timestamp. A name the operator typed beats both.
            lbl = ""
            try:
                lbl = self._rec_label()
            except Exception:                                 # noqa: BLE001
                pass
            if not lbl:
                try:
                    lbl = self._exp_session() or "manual"
                except Exception:                             # noqa: BLE001
                    lbl = "manual"
            self._start_record_all(label=lbl)

    def _live_input_mode(self):
        """Which input is driving RIGHT NOW: "vr", "master", or "unknown".

        From the process table and the mapper's own state, never from a tab
        being visible. The operator connects one input at a time, and the
        recording has to say which -- the pose topic cannot, because both
        publishers use it.
        """
        try:
            vr = subprocess.run(
                ["pgrep", "-f", "srl_vr_teleop/vr_pose_mapper"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=3).returncode == 0
            ma = subprocess.run(
                ["pgrep", "-f", "srl_teleop/master_pose_node"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=3).returncode == 0
        except Exception:                                     # noqa: BLE001
            return "unknown"
        if vr and not ma:
            return "vr"
        if ma and not vr:
            return "master"
        if vr and ma:
            # Both alive. The clutch is the tie-break: a VR session with the
            # grip engaged is a VR session whatever else is running.
            s = self.bus.snap.get("vrmap_left") or self.bus.snap.get("vrmap_right")
            try:
                if s and json.loads(s[0]).get("engaged"):
                    return "vr"
            except Exception:                                 # noqa: BLE001
                pass
            return "both_up"
        return "unknown"

    def _rec_label(self):
        """The operator's name for this run, made safe for a directory.

        Sanitised HERE as well as in `record_all.py`, which does the same
        thing to the same characters. Two copies of one rule is usually a
        defect, and this is the exception that earns it: the operator has to
        SEE what their name will become before they press record, and the
        recorder has to be safe when it is called from a terminal with no
        window in front of it.
        """
        if not hasattr(self, "rec_name"):
            return ""
        raw = self.rec_name.text().strip()
        # SPACES BECOME UNDERSCORES, they are not deleted. Dropping them
        # turned "E2 aggressive run 3" into "E2aggressiverun3", which is a
        # safe directory name and an unreadable one -- and the whole point of
        # the field is to be able to read it in a listing six weeks later.
        out, prev = [], ""
        for c in raw:
            c = "_" if (c.isspace() or c in "/\\.:") else c
            if not (c.isalnum() or c in "-_"):
                continue
            if c == "_" and prev == "_":
                continue
            out.append(c)
            prev = c
        return "".join(out).strip("_-")

    def _rec_pose_refresh(self, info):
        """The POSE panel's copy of the recording state."""
        # The POSE panel's duplicate is gone; this guard is what lets the
        # same refresh serve whichever record controls still exist.
        if not hasattr(self, "rec_pose_btn"):
            return
        if info:
            secs = int(time.time() - float(info.get("started", time.time())))
            self.rec_pose_btn.setText("\u25a0  STOP RECORDING")
            self.rec_pose_btn.setStyleSheet(
                "background:%s; color:white; font-weight:bold" % C_BAD)
            self.rec_pose_lbl.setText(
                "RECORDING %s  --  %d:%02d"
                % (os.path.basename(info.get("outdir", "?")),
                   secs // 60, secs % 60))
            self.rec_pose_lbl.setStyleSheet("color:%s" % C_BAD)
        else:
            self.rec_pose_btn.setText("\u25cf  RECORD EVERYTHING")
            self.rec_pose_btn.setStyleSheet("")
            self.rec_pose_lbl.setText("not recording")
            self.rec_pose_lbl.setStyleSheet("color:%s" % C_MUTED)

    def _rec_all_refresh(self):
        info = self._record_all_live()
        # THE SECOND BUTTON IS NOT A SECOND RECORDER. Both reach the same
        # session marker, so both must show the same state -- a stopped
        # button beside a running one is how an operator ends up believing
        # nothing is being captured while it is.
        self._rec_pose_refresh(info)
        if not hasattr(self, "rec_all_btn"):
            return
        # A STOP IN FLIGHT OUTRANKS THE MARKER.
        #
        # This refresh runs every 2 s, and a stop takes about five: flushing
        # the bag cache, compressing the mcap, drawing five figures. The
        # live-session marker is removed at the END of that work, so for the
        # first two seconds after the press this saw a live session and put
        # the button back to a red STOP RECORDING -- overwriting the
        # STOPPING... that the press had just set. The operator sees yellow
        # flash and then a button that looks untouched, which reads as "it
        # did not stop" when the recorder had already begun closing down.
        # Reported exactly that way on 2026-08-30.
        #
        # So while a stop is outstanding the button says so, whatever the
        # marker says. Bounded, because a stop that never finishes must not
        # leave the control stuck: after 60 s the marker is believed again
        # and the operator gets the button back.
        stopping = getattr(self, "_rec_stopping", 0.0)
        if stopping and (time.time() - stopping) > 60.0:
            self._rec_stopping = stopping = 0.0
        if not info:
            self._rec_stopping = 0.0
        elif stopping:
            self.rec_all_btn.setText("\u25a0  STOPPING...")
            self.rec_all_btn.setStyleSheet(
                "background:%s; color:white; font-weight:bold" % C_WARN)
            self.rec_all_btn.setEnabled(False)
            self.rec_all_lbl.setText(
                "closing the session (%.0f s): flushing the bag, compressing "
                "it and drawing the figures."
                % (time.time() - stopping))
            self.rec_all_lbl.setStyleSheet("color:%s" % C_WARN)
            return
        if info:
            secs = int(time.time() - float(info.get("started", time.time())))
            name = os.path.basename(info.get("outdir", "?"))
            self.rec_all_btn.setText("\u25a0  STOP RECORDING")
            self.rec_all_btn.setStyleSheet(
                "background:%s; color:white; font-weight:bold" % C_BAD)
            self.rec_all_lbl.setText(
                "RECORDING %s  --  %d:%02d  --  every topic, the trail, the "
                "events and both real arms" % (name, secs // 60, secs % 60))
            self.rec_all_lbl.setStyleSheet("color:%s" % C_BAD)
        else:
            # The session is really gone -- give the button back. Re-enabling
            # HERE rather than on a timer means it returns exactly when it is
            # safe to press again, not a fixed guess at how long a stop takes.
            self.rec_all_btn.setEnabled(True)
            self.rec_all_btn.setText("\u25cf  RECORD EVERYTHING")
            self.rec_all_btn.setStyleSheet("")
            self.rec_all_lbl.setText("not recording")
            self.rec_all_lbl.setStyleSheet("color:%s" % C_MUTED)

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
        g = QGroupBox("Controls for while it runs")
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
        g = QGroupBox("LOOK AT THE TABLE  --  what is really there")
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
        b = QPushButton("1.  FULL SCAN   (once per table)")
        b.setFont(helvetica(10, True))
        b.setMinimumHeight(28)
        b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        b.setToolTip(
            "The robot looks over the whole table and works out what is on it.\n\n"
            "It first finds the table itself -- wherever it is, whatever "
            "height -- then walks over it in straight rows at two heights "
            "and from three angles, holding the wrist still at each stop "
            "like a 3D printer touching off. It photographs each object, "
            "works out how wide it is, and says which ones the gripper can "
            "actually close on.\n\n"
            "Takes several minutes. You only need it ONCE for a given "
            "table -- after that use QUICK CHECK.")
        b.clicked.connect(self.on_calibrate_environment)
        row.addWidget(b, 1)
        self.map_arm = QComboBox()
        # BOTH FIRST, and it is the default. The map is the robot's, not an
        # arm's: the two arms cannot cross the centreline, so a one-arm map is
        # missing exactly the half its own arm can never reach -- and that
        # half is not empty, it is unmeasured.
        self.map_arm.addItems(["both arms", "left arm", "right arm"])
        # WIDE ENOUGH FOR ITS OWN LONGEST ENTRY. It rendered as "bot".
        self.map_arm.setMinimumContentsLength(9)
        self.map_arm.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.map_arm.currentTextChanged.connect(
            lambda t: self.bus.note("map arm: %s" % t))
        row.addWidget(self.map_arm)
        v.addLayout(row)

        # THE FAST PATH. The full sweep is minutes and is a once-per-table
        # thing; this is the one an operator presses between picks.
        rowr = QHBoxLayout()
        br = QPushButton("2.  QUICK CHECK   (what moved)")
        br.setFont(helvetica(10, True))
        br.setMinimumHeight(28)
        br.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        br.setMinimumWidth(0)
        br.setToolTip(
            "Spots what has changed and only re-checks that.\n\n"
            "The fixed camera above the table compares the scene with the "
            "picture it remembered, which takes no arm movement at all. The "
            "arms then visit only the few positions that can see the parts "
            "that changed.\n\n"
            "About 30 seconds, against 18 minutes for a full scan. Needs a "
            "FULL SCAN and a REMEMBER THIS VIEW first.")
        br.clicked.connect(self.on_relook)
        rowr.addWidget(br, 2)
        bref = QPushButton("REMEMBER VIEW")
        bref.setFont(helvetica(10))
        bref.setToolTip(
            "Takes a picture of the table as it is now and keeps it.\n\n"
            "Press this when the table is set up how you want it. From then on, QUICK CHECK compares against this picture to see what has moved.")
        bref.clicked.connect(self.on_set_reference)
        rowr.addWidget(bref, 1)
        v.addLayout(rowr)

        row2 = QHBoxLayout()
        b2 = QPushButton("LIST WHAT IT FOUND")
        b2.setFont(helvetica(10))
        b2.setToolTip("Shows everything the robot currently believes is on the table, "
                      "with sizes, and which ones it can pick up. Moves nothing.")
        b2.clicked.connect(self.on_show_map)
        row2.addWidget(b2, 1)
        b3 = QPushButton("3.  PICK IT UP")
        b3.setFont(helvetica(10, True))
        b3.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
        b3.setToolTip(
            "Picks up the object you chose, coming DOWN onto it from above "
            "as steeply as the arm can manage.\n\n"
            "It uses where the robot SAW the object, not a position typed "
            "into a file. Nothing moves unless you tick 'really move the arm' -- otherwise it just shows you the plan.")
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
        self.map_move = QCheckBox("really move the arm")
        self.map_move.setToolTip(
            "Leave this OFF to see what the robot would do without it doing "
            "anything. Turn it ON and the arm really moves.")
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
        arm = self.map_arm.currentText().split()[0]   # "both arms" -> "both"
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
        # A CONTROL THAT CHANGES WHAT THE NEXT RUN DOES MUST SAY SO, and this
        # one changes how long the operator is going to be standing there.
        self.bus.note("calibration sweep: %s arm(s), volume sweep, one fixed "
                      "wrist attitude per pass" % arm)
        self.bus.note("calibrating the environment (%s arm): %s"
                      % (arm, " ".join(argv)))
        self._map_say("sweeping the %s arm over the workspace. Watch the "
                      "banner above." % arm)
        self._spawn(gls.Spec("calibrate_env", "calibrate environment",
                             "map", argv, needs_stack=True))

    def on_relook(self):
        """Re-measure only what the scene camera says changed."""
        py = os.path.join(_WS, ".venv_vision", "bin", "python")
        argv = [py if os.path.exists(py) else sys.executable, "-u",
                os.path.join(_WS, "scripts", "relook.py"), "--changed-only"]
        self.bus.note("fast re-look: scene camera picks the regions, arms "
                      "visit only the viewpoints that see them")
        self._map_say("asking the scene camera what changed...")
        self._spawn(gls.Spec("relook", "fast re-look", "map", argv,
                             needs_stack=True))

    def on_set_reference(self):
        """Store the scene camera's current frame as the 'before' picture."""
        py = os.path.join(_WS, ".venv_vision", "bin", "python")
        argv = [py if os.path.exists(py) else sys.executable, "-u",
                os.path.join(_WS, "scripts", "relook.py"), "--save-reference"]
        self.bus.note("storing the scene camera reference frame")
        self._map_say("storing the scene camera's current view as the "
                      "reference. Everything after is measured as a change "
                      "from this.")
        self._spawn(gls.Spec("scene_ref", "set scene reference", "map", argv,
                             needs_stack=True))

    def on_show_map(self):
        def go():
            try:
                sys.path.insert(0, os.path.join(_WS, "scripts"))
                import pick_from_map as PFM
                doc = PFM.load_map()
            except Exception as e:                            # noqa: BLE001
                self._map_say("no map to show: %r" % (e,), bad=True)
                return
            # HOW OLD THE MAP IS, IN THE FIRST LINE.
            #
            # `load_map()` has always computed `_age_s` and this panel has
            # always thrown it away, which is the "feature present but does
            # nothing" row of CLAUDE.md's own instrument table: the field is
            # STORED and no consumer READS it. Measured 2026-08-25: the panel
            # said "6 object(s)" about a map 22.3 hours old, describing the
            # simulated T1 layout, while the real table in front of the
            # operator held two things. Nothing on screen distinguished that
            # from a map of the table you are looking at, and `PICK IT UP`
            # plans from whatever this returns.
            age_s = float(doc.get("_age_s", 0.0))
            if age_s < 3600:
                age = "%d min old" % round(age_s / 60)
            else:
                age = "%.1f HOURS OLD" % (age_s / 3600)
            stale = age_s > 1800
            lines = ["surface z = %.4f m   %d object(s)   %s   finder: %s"
                     % (doc["surface"]["z_m"], len(doc["objects"]), age,
                        doc.get("provenance", {})
                        .get("object_finder", "?")[:40])]
            if stale:
                lines.append(
                    "THIS MAP IS NOT OF THE TABLE IN FRONT OF YOU unless "
                    "nothing has moved since it was made. Run FULL SCAN.")
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
        arm = self.map_arm.currentText().split()[0]
        if arm == "both":
            # A PICK IS ONE ARM'S JOB. "both" is a scanning choice, not a
            # grasping one, and silently picking the left would be a guess.
            #
            # AND IT GOES IN THE LOG. The audit caught this: a press that only
            # repaints a label leaves no trace, and "a press must leave a
            # trace" is this window's own rule -- an operator who pressed a
            # button and saw nothing happen needs the event log to tell them
            # whether it did anything at all.
            self.bus.note("pick from map REFUSED: 'both arms' is a scanning "
                          "choice, not a grasping one -- choose left or right",
                          bad=True)
            self._map_say("choose LEFT ARM or RIGHT ARM to pick with -- "
                          "'both arms' is for scanning, not for grabbing.",
                          bad=True)
            return
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
        g = QGroupBox("Settings you change once, before starting")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        row = QHBoxLayout()
        self.force_clutch = QCheckBox("force clutch ENGAGED")
        self.force_clutch.stateChanged.connect(self.on_force_clutch)
        row.addWidget(self.force_clutch)
        b = QPushButton("re-centre on my hand")
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
        g = QGroupBox("How smoothly the arms move  (next start)")
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
    def _vr_spin(self, layout, label, lo, hi, step, val, tip, apply_fn):
        """One labelled spin box that pushes its value at a running node.

        `valueChanged` fires on every keystroke, which would send a parameter
        per digit while somebody types "100" -- three writes, two of them at
        values nobody asked for. `editingFinished` plus the arrow buttons is
        what the operator means.
        """
        row = QHBoxLayout()
        lab = QLabel(label)
        lab.setFont(helvetica(9))
        lab.setToolTip(tip)
        row.addWidget(lab, 1)
        sp = QDoubleSpinBox()
        sp.setFont(helvetica(9))
        sp.setRange(lo, hi)
        sp.setSingleStep(step)
        sp.setValue(val)
        sp.setDecimals(2)
        sp.setToolTip(tip)
        sp.setKeyboardTracking(False)
        sp.valueChanged.connect(
            lambda x, n=label: (apply_fn(float(x)),
                                self.log("%s -> %.2f" % (n, x))))
        row.addWidget(sp)
        layout.addLayout(row)
        return sp

    def on_vr_smoothing(self, name):
        self.bus.set_param("/vr_pose_mapper", "smoothing", str(name))
        self.log("VR smoothing law -> %s" % name)
        # `none` and `ema` have no adaptive cutoff, so the readout would sit
        # at a stale number from the last one_euro run. Say which it is.
        if name != "one_euro":
            self.vr_smooth_state.setText(
                "law is %s -- no adaptive cutoff to report" % name)

    #: The arms and their addresses. ONE source -- arm_link_monitor's own
    #: table -- so the window cannot disagree with the thing that pings them.
    ARM_IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}

    def _arms_panel(self):
        """Per-arm CONNECT / DISCONNECT, with the address and the live state.

        WHY THE ADDRESS IS ON THE BUTTON. "The arm is not connected" has three
        completely different causes and they need three different actions:
        the arm is off the network (power/cable), the arm answers but no
        bridge is running (press CONNECT), or a bridge is running and the
        session is dead (press DISCONNECT then CONNECT). This session spent
        real time on each of those, and the only way to tell them apart was a
        terminal. The row now says which one it is.

        DISCONNECT IS SIGINT AND ONLY SIGINT. HARD CONSTRAINT 2: the arm
        permits exactly ONE Kortex session and SIGKILL LEAKS IT -- the next
        connect then fails while the arm still pings and its API port still
        answers, which is indistinguishable from a network fault.
        """
        g = QGroupBox("ARMS  --  the physical robots")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        v.setContentsMargins(6, 18, 6, 6)
        v.setSpacing(4)
        self.arm_state_lbl = {}
        self.arm_btn = {}
        for arm in ("left", "right"):
            row = QHBoxLayout()
            row.setSpacing(6)
            lab = QLabel("%s   %s" % (arm.upper(), self.ARM_IPS[arm]))
            lab.setFont(mono(11, True))
            lab.setMinimumWidth(150)
            row.addWidget(lab)
            st = QLabel("checking...")
            st.setFont(helvetica(11, True))
            st.setMinimumWidth(120)
            st.setStyleSheet("color:%s" % C_UNKNOWN)
            self.arm_state_lbl[arm] = st
            row.addWidget(st, 1)
            for act, txt, col in (("connect", "CONNECT", C_OK),
                                  ("disconnect", "DISCONNECT", C_WARN)):
                b = QPushButton(txt)
                b.setFont(helvetica(10, True))
                b.setMinimumHeight(34)
                b.setStyleSheet("color:%s;border:1px solid %s" % (col, col))
                b.setToolTip(
                    ("Starts the high-level bridge and the wrist camera for "
                     "this arm (scripts/bringup_arm.sh). Idempotent -- "
                     "pressing it twice does not open a second session."
                     if act == "connect" else
                     "Stops this arm's bridge with SIGINT and waits for "
                     "'kortex session closed cleanly'. NEVER SIGKILL: that "
                     "leaks the one session the arm allows, and the next "
                     "connect fails while the arm still pings."))
                b.clicked.connect(
                    lambda _, a=arm, k=act: self.on_arm_link(a, k))
                row.addWidget(b)
                self.arm_btn[(arm, act)] = b
            v.addLayout(row)
        self.arm_note = QLabel("")
        self.arm_note.setFont(helvetica(9))
        self.arm_note.setWordWrap(True)
        self.arm_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.arm_note)
        return g

    def _arm_spawn(self, argv):
        """The ONE Popen the arm buttons use, so the audit can replace it."""
        return subprocess.Popen(argv, start_new_session=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    def on_arm_link(self, arm, action):
        import signal
        if action == "connect":
            sh = os.path.join(_WS, "scripts", "bringup_arm.sh")
            if not os.path.exists(sh):
                self.log("%s is missing" % sh, bad=True)
                self.arm_note.setText("bringup_arm.sh is missing")
                return
            try:
                self._arm_spawn(["bash", sh, arm])
            except Exception as e:                            # noqa: BLE001
                self.log("%s connect failed: %s" % (arm, e), bad=True)
                return
            self.log("%s arm: CONNECT requested (%s)" % (arm, self.ARM_IPS[arm]))
            self.arm_note.setText(
                "%s arm connecting to %s -- the Kortex session takes a few "
                "seconds, and the wrist camera a few more."
                % (arm, self.ARM_IPS[arm]))
            return
        # ---- disconnect: SIGINT ONLY.
        pids = self._bridge_pids(arm)
        if not pids:
            self.arm_note.setText("%s arm: no bridge is running" % arm)
            self.log("%s arm: DISCONNECT -- nothing was running" % arm)
            return
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError as e:
                self.log("%s arm: could not signal %d: %s" % (arm, pid, e),
                         bad=True)
        self.log("%s arm: SIGINT sent to %s -- waiting for a clean session "
                 "close" % (arm, pids))
        self.arm_note.setText(
            "%s arm disconnecting (SIGINT to %s). Never force-kill a bridge: "
            "it leaks the one session the arm allows."
            % (arm, ", ".join(str(p) for p in pids)))

    @staticmethod
    def _bridge_pids(arm):
        """PIDs of this arm's bridge, by the node name bringup_arm.sh gives it."""
        try:
            out = subprocess.run(
                ["pgrep", "-f", "kortex_highlevel_bridge_%s" % arm],
                capture_output=True, text=True, timeout=5).stdout
        except Exception:                                     # noqa: BLE001
            return []
        return [int(x) for x in out.split() if x.strip().isdigit()]

    def _pose_panel(self):
        """GO HOME / GO TO PICK POSE -- one press, both arms.

        THE VALUES COME FROM `srl_named_poses`, which reads the SAME files
        every other consumer reads. CLAUDE.md records home living in five
        places and drifting between them; a button carrying its own copy of
        the joint angles would be the sixth, and the one nobody would think
        to check.
        """
        g = QGroupBox("POSE  --  put the arms somewhere known")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        # ROOM FOR THE TITLE. A QGroupBox draws its title inside its own
        # frame, so the default top margin puts the first row under the
        # words. Caught by screenshotting the panel, not by the audit --
        # which pressed both buttons happily while they were unreadable.
        v.setContentsMargins(6, 18, 6, 6)
        v.setSpacing(4)

        row = QHBoxLayout()
        row.setSpacing(6)
        self.pose_btn = {}
        # THREE DISTINCT COLOURS, none of them the muted text colour.
        # SCAN POSE was drawn in C_TEXT and read as DISABLED beside the two
        # coloured buttons -- a live control that looks greyed out is one
        # nobody presses, which is the same outcome as not having it.
        for key, col in (("home", C_OK), ("pick", C_WARN),
                         ("scan", C_UNKNOWN)):
            lbl, why = _POSES.describe(key)
            b = QPushButton(lbl)
            b.setFont(helvetica(14, True))
            b.setMinimumHeight(52)
            b.setStyleSheet("color:%s;border:1px solid %s" % (col, col))
            b.setToolTip("%s\n\nSource: %s\n\nSends both arms there over 5 s "
                         "on the arm controllers. Release the VR clutch first "
                         "-- while the clutch is IN the follower is also "
                         "commanding, and the two fight."
                         % (why, _POSES.source_of(key, "left")))
            b.clicked.connect(lambda _, k=key: self.on_goto_pose(k))
            row.addWidget(b)
            self.pose_btn[key] = b
        v.addLayout(row)

        self.pose_note = QLabel("")
        self.pose_note.setFont(helvetica(9))
        self.pose_note.setWordWrap(True)
        self.pose_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.pose_note)

        # ===================================================================
        # RECORD EVERYTHING -- here, because every control mode passes here
        # ===================================================================
        # ONE RECORD BUTTON, NOT TWO. There used to be a second copy here,
        # and the comment defending it argued that recording is not specific
        # to a mode so it should be reachable from wherever you are. That is
        # true and it was still wrong: the operator's report on 2026-08-30 was
        # "so much jargon in the gui ... remove repetitive buttons", and two
        # identical buttons that drive the same recorder read as two
        # recordings until you learn otherwise. The one beside
        # MOVE THE REAL ARMS is the one that stays.
        # ------------------------------------------------- THE WHOLE JOB
        # The operator asked for the sequence as buttons: home, pick pose,
        # scan pose, scan, back, pick. Each step is its own button because
        # a sequence that can only be run whole cannot be debugged -- when
        # the scan finds nothing you want to re-run the scan, not the day.
        seq = QLabel("SCAN AND PICK  --  each step, or the whole run")
        seq.setFont(helvetica(10, True))
        seq.setStyleSheet("color:%s" % C_TEXT)
        v.addWidget(seq)

        r2 = QHBoxLayout()
        r2.setSpacing(4)
        self.seq_btn = {}
        for key, lbl, tip in (
                ("scan_run", "SCAN THE TABLE",
                 "Both arms raster the surface tool-down, serpentine, and "
                 "build the world map from the wrist depth cameras. Only the "
                 "arms that are CONNECTED are scanned with."),
                ("seq_all", "RUN THE WHOLE SEQUENCE",
                 "HOME, PICK POSE, SCAN POSE, scan, back to PICK POSE. Stops "
                 "at the first step that refuses.")):
            b = QPushButton(lbl)
            b.setFont(helvetica(10, True))
            b.setMinimumHeight(34)
            b.setToolTip(tip)
            b.clicked.connect(lambda _, k=key: self.on_sequence(k))
            r2.addWidget(b)
            self.seq_btn[key] = b
        v.addLayout(r2)

        # ------------------------------------------------- WHAT IT FOUND
        # ONE BUTTON PER OBJECT, NAMED. The window used to ask for "object 0"
        # in a spin box, with the number coming from a separate LIST button
        # that printed to a log. Nothing on screen connected that number to
        # the thing on the table, so picking the wrong one was one typo away
        # and looked identical to picking the right one.
        #
        # The names are DERIVED from the map -- the colour the camera measured
        # and the width it measured -- so they cannot drift from what the
        # robot actually believes is there.
        self.found_head = QLabel("WHAT IT FOUND  --  press one to pick it")
        self.found_head.setFont(helvetica(10, True))
        self.found_head.setStyleSheet("color:%s" % C_TEXT)
        v.addWidget(self.found_head)
        self.found_box = QWidget()
        self.found_lay = QVBoxLayout(self.found_box)
        self.found_lay.setContentsMargins(0, 0, 0, 0)
        self.found_lay.setSpacing(3)
        v.addWidget(self.found_box)
        self.found_btns = []
        # ONE TICK, AND IT GOVERNS THE SCAN AND THE PICK ALIKE.
        #
        # Both underlying scripts default to the SIMULATION: they publish to
        # the bare `/<arm>_arm_controller/joint_trajectory` while
        # `kortex_highlevel_bridge` subscribes under `/real`. So a scan and a
        # pick could both run start to finish, report success, and move
        # nothing but the picture -- which is exactly what happened. Neither
        # errors, because commanding a topic nobody real is listening to is
        # not an error.
        self.found_move = QCheckBox(
            "MOVE THE REAL ARM   (unticked = simulation only)")
        self.found_move.setFont(helvetica(10, True))
        self.found_move.setStyleSheet("color:%s" % C_BAD)
        self.found_move.setToolTip(
            "Governs BOTH the scan and the pick. Unticked, they run against "
            "the simulation and the metal does not move. Ticked, both are "
            "given --drive-real, which puts the commands where the arm "
            "bridge is listening.")
        self.found_move.toggled.connect(
            lambda on: self.log("REAL ARM motion -> %s"
                                % ("ENABLED" if on else "off (simulation)"),
                                bad=bool(on)))
        v.addWidget(self.found_move)
        self._refresh_found()

        self.seq_note = QLabel("")
        self.seq_note.setFont(helvetica(9))
        self.seq_note.setWordWrap(True)
        self.seq_note.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.seq_note)
        return g

    def _refresh_found(self):
        """Rebuild the object buttons from the map. Cheap, and idempotent."""
        for b in getattr(self, "found_btns", []):
            b.setParent(None)
        self.found_btns = []
        objs = _MAPOBJ.load()
        age = _MAPOBJ.age_s()
        if not objs:
            self.found_head.setText(
                "WHAT IT FOUND  --  nothing yet. Press SCAN THE TABLE.")
            return
        self.found_head.setText(
            "WHAT IT FOUND  --  %d object(s), map %s old. Press one to pick it."
            % (len(objs), self._age_str(age)))
        for o in objs:
            b = QPushButton("%s   %.0f mm" % (o["name"], o["width_mm"]))
            b.setFont(helvetica(10, True))
            b.setMinimumHeight(30)
            if o["graspable"]:
                b.setStyleSheet("color:%s;border:1px solid %s" % (C_OK, C_OK))
                b.setToolTip("Pick this one. Measured %.1f mm across its "
                             "narrowest axis." % o["width_mm"])
            else:
                # OFFERED BUT REFUSED, WITH THE REASON ON IT. Hiding it would
                # leave the operator wondering why the thing they can see is
                # not in the list.
                b.setEnabled(False)
                b.setStyleSheet("color:%s" % C_MUTED)
                b.setText("%s   %.0f mm  -- cannot grip"
                          % (o["name"], o["width_mm"]))
                b.setToolTip(o["why"] or "the map says this is not graspable")
            b.clicked.connect(lambda _, o=o: self.on_pick_named(o))
            self.found_lay.addWidget(b)
            self.found_btns.append(b)

    @staticmethod
    def _age_str(a):
        if a is None:
            return "?"
        if a < 90:
            return "%.0f s" % a
        if a < 5400:
            return "%.0f min" % (a / 60)
        return "%.1f h" % (a / 3600)

    def on_pick_named(self, o):
        """Pick the object the operator pressed, by name."""
        arm = self.map_arm.currentText().split()[0] if hasattr(self, "map_arm") \
            else "left"
        if arm == "both":
            arm = "left"
        move = self.found_move.isChecked()
        py = os.path.join(_WS, ".venv_vision", "bin", "python")
        argv = [py if os.path.exists(py) else sys.executable, "-u",
                os.path.join(_WS, "scripts", "pick_from_map.py"),
                "--arm", arm, "--object", str(o["index"])]
        if move:
            # --drive-real IS NOT OPTIONAL WHEN THE OPERATOR SAID "MOVE".
            # pick_from_map's own help: "Without it --execute moves the
            # SIMULATION ONLY -- the bridge subscribes under /real and never
            # sees a bare topic." Ticking "really move the arm" and watching
            # the simulation move is the worst outcome available.
            argv += ["--execute", "--drive-real"]
        self.log("pick '%s' (object %d) with the %s arm, %s"
                 % (o["name"], o["index"], arm,
                    "MOVING THE REAL ARM" if move else "plan only"))
        try:
            self._seq_spawn(argv)
        except Exception as e:                                # noqa: BLE001
            self.log("pick_from_map would not start: %s" % e, bad=True)
            return
        self.seq_note.setText(
            "picking '%s' with the %s arm -- %s"
            % (o["name"], arm,
               "MOVING" if move else "planning only, nothing will move"))

    # ------------------------------------------------------------- sequence
    #: The order the operator asked for. Each entry is (label, what it does).
    #: RUN THE WHOLE SEQUENCE stops after the scan and the return to the
    #: pick pose. THE PICK IS DELIBERATELY NOT IN IT: which object to pick is
    #: a decision, the map is what informs it, and the map does not exist
    #: until the scan has run. Picking "object 0" automatically is picking
    #: whatever the segmenter happened to list first.
    SEQ_STEPS = [("home", "pose"), ("pick", "pose"), ("scan", "pose"),
                 ("scan_run", "run"), ("pick", "pose")]

    def _seq_spawn(self, argv):
        """The ONE Popen the scan/pick buttons use.

        Separate and named so `verify_gui_buttons` can replace exactly this
        and leave the whole click path -- refusals, logging, the note line --
        running for real. Pressing SCAN for real during an audit would drive
        both arms across a table.
        """
        return subprocess.Popen(argv, start_new_session=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    def on_sequence(self, key):
        """Run one step of the scan-and-pick sequence, or all of it."""
        if key == "seq_all":
            done, stopped = [], None
            for name, kind in self.SEQ_STEPS:
                ok = (self._seq_pose(name) if kind == "pose"
                      else self._seq_run(name))
                if not ok:
                    stopped = name
                    break
                done.append(name)
            msg = "sequence: %s" % (" -> ".join(done) or "nothing ran")
            if stopped:
                msg += "   STOPPED at %s" % stopped
            self.seq_note.setText(msg)
            self.log(msg, bad=bool(stopped))
            return
        before = self.seq_note.text()
        ok = self._seq_run(key)
        # DO NOT OVERWRITE A REASON WITH A VERDICT. `_seq_run` sets a note
        # naming exactly what is missing; replacing it with "REFUSED" throws
        # away the only useful half of the message.
        if self.seq_note.text() == before:
            self.seq_note.setText("%s: %s"
                                  % (key, "started" if ok else "REFUSED"))

    def _planned_legs(self, key, arm):
        """Waypoints for this pose from `recordings/baselines/scan_move.json`.

        Returns [] when there is no plan, so an unplanned pose behaves
        exactly as before. Returns None-safe: a plan that cannot be read is
        treated as no plan and SAID, never silently ignored -- a missing plan
        that quietly becomes a direct move is how the collision happened.
        """
        if key != "scan":
            return []
        path = os.path.join(_WS, "recordings/baselines/scan_move.json")
        if not os.path.exists(path):
            self.log("no scan_move.json -- SCAN POSE would be a DIRECT move. "
                     "Run: python3 scripts/plan_scan_move.py --both "
                     "--start pick --save", bad=True)
            return []
        try:
            with open(path) as fh:
                d = json.load(fh)
        except Exception as e:                                # noqa: BLE001
            self.log("scan_move.json unreadable (%s)" % e, bad=True)
            return []
        r = d.get(arm)
        if not r or not r.get("waypoints"):
            return []
        self.log("%s arm: following the planned move via '%s' "
                 "(%d waypoint(s), worst clearance %.3f m)"
                 % (arm, r.get("via", "?"), len(r["waypoints"]),
                    r.get("worst_clearance_m", float("nan"))))
        return [list(map(float, w)) for w in r["waypoints"]]

    def _wait_arrival(self, arm, q, tol_deg=3.0, timeout_s=20.0):
        """Block until the arm is near `q`, or the timeout expires.

        Arrival is WAITED FOR, not timed: the bridge closes error
        proportionally, so a fixed sleep leaves a third of it standing --
        which on a detour waypoint means the corner is cut.
        """
        import numpy as _np
        names = ["%s_joint_%d" % (arm, i) for i in range(1, 8)]
        # IF THE ARM IS NOT REPORTING, THERE IS NOTHING TO WAIT FOR.
        #
        # Blocking the Qt thread for 20 s per waypoint against an arm that
        # publishes nothing freezes the window -- including the e-stop, which
        # is the one control that must never be blocked. It also made the
        # button audit fail: the press never returned. With no joint states
        # arrival cannot be established at all, so waiting achieves nothing
        # and the hold keeps the setpoint alive regardless.
        cur0 = getattr(self.bus, "_real_q", {}) or {}
        if not all(n in cur0 for n in names):
            self.log("%s arm: no joint states -- cannot confirm the waypoint "
                     "was reached; the held setpoint stands" % arm)
            return False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            QApplication.processEvents()
            cur = getattr(self.bus, "_real_q", {}) or {}
            have = [cur.get(n) for n in names]
            if all(v is not None for v in have):
                d = _np.abs((_np.array(have) - _np.array(q) + _np.pi)
                            % (2 * _np.pi) - _np.pi)
                if float(_np.degrees(d).max()) < tol_deg:
                    return True
            time.sleep(0.05)
        self.log("%s arm: waypoint not reached within %.0f s -- continuing "
                 "anyway would cut the corner the detour exists for; stopping "
                 "here" % (arm, timeout_s), bad=True)
        return False

    def _seq_pose(self, key):
        """One pose step of the sequence. False stops the run.

        A REFUSAL STOPS THE SEQUENCE. If HOME could not be commanded --
        e-stop engaged, or no arm connected -- then scanning and picking from
        an unknown pose is worse than not running at all.
        """
        self.on_goto_pose(key)
        txt = self.pose_note.text()
        return ("NOT sent" not in txt) and ("REFUSED" not in txt)

    def _scan_preflight(self, arms):
        """What `calibrate_environment` will WAIT 120 SECONDS for.

        THE BUTTON LOOKED DEAD AND THE SCRIPT WAS FINE.
        -----------------------------------------------
        `env_probe.wait_ready` blocks for 120 s waiting on /compute_ik,
        /joint_states for the arm, and that arm's wrist camera -- and this
        window spawned the scanner with its output going to /dev/null. So
        pressing SCAN THE TABLE started a process that sat in silence for two
        minutes and then refused where nobody could see it. Measured
        2026-08-26: the traceback was in a terminal, never in the window, and
        from the operator's side the button simply did nothing.

        Checked HERE instead, in milliseconds, and named.
        """
        missing = []
        have = {n for n, _ in self.bus.get_topic_names_and_types()}
        srv = {n for n, _ in self.bus.get_service_names_and_types()}
        if "/compute_ik" not in srv:
            missing.append("/compute_ik (MoveIt is not up -- start a mode)")
        if "/joint_states" not in have:
            missing.append("/joint_states (no controllers running)")
        for a in arms:
            if "/%s_camera/depth/image_raw" % a not in have:
                missing.append("the %s wrist camera (/%s_camera/depth/"
                               "image_raw)" % (a, a))
        return missing

    def _seq_run(self, key):
        """Start the scan, and SHOW WHAT IT SAYS.

        Output goes to .scratch/scan.log and the exit code is watched, so a
        scan that dies says so in the window. A long job that reports nothing
        is a job nobody can debug, and this one takes minutes.
        """
        script = {"scan_run": "calibrate_environment.py"}.get(key)
        if script is None:
            self.log("unknown sequence step %r" % key, bad=True)
            return False
        path = os.path.join(_WS, "scripts", script)
        if not os.path.exists(path):
            self.log("%s is missing -- cannot run %s" % (path, key), bad=True)
            return False

        # ONLY THE ARMS THAT ARE ACTUALLY CONNECTED. This passed --arm both
        # unconditionally; with one arm on the network the sweep tries to
        # drive an arm that has no bridge and nothing visible happens.
        live = [a for a in ARMS if self._bridge_pids(a)]
        if not live:
            msg = ("SCAN REFUSED: no arm has a bridge running. Press CONNECT "
                   "in the ARMS panel first.")
            self.log(msg, bad=True)
            self.seq_note.setText(msg)
            return False
        miss = self._scan_preflight(live)
        if miss:
            msg = ("SCAN REFUSED: missing %s. The scanner would have waited "
                   "120 s and then given up." % "; ".join(miss))
            self.log(msg, bad=True)
            self.seq_note.setText(msg)
            return False

        venv = os.path.join(_WS, ".venv_vision", "bin", "python")
        py = venv if os.path.exists(venv) else sys.executable
        argv = [py, "-u", path,
                "--arm", "both" if len(live) == 2 else live[0],
                "--order", "serpentine"]
        real = bool(getattr(self, "found_move", None)
                    and self.found_move.isChecked())
        if real:
            # Without --drive-real the sweep publishes to the bare topics and
            # moves the SIMULATION ONLY; the bridge subscribes under /real.
            argv.append("--drive-real")
        self.log("scanning with %s -- %s"
                 % (", ".join(live),
                    "DRIVING THE REAL ARM" if real else "SIMULATION ONLY"),
                 bad=real)
        logp = os.path.join(_WS, ".scratch", "scan.log")
        try:
            os.makedirs(os.path.dirname(logp), exist_ok=True)
            self._scan_log = open(logp, "w")
            self._scan_proc = subprocess.Popen(
                argv, start_new_session=True,
                stdout=self._scan_log, stderr=subprocess.STDOUT)
        except Exception as e:                                # noqa: BLE001
            self.log("%s would not start: %s" % (script, e), bad=True)
            return False
        self.seq_note.setText(
            "scanning with %s (%s). Full output in .scratch/scan.log"
            % (", ".join(live), "REAL ARM" if real else "simulation"))
        QTimer.singleShot(4000, self._scan_watch)
        for ms in (20000, 60000, 180000, 360000):
            QTimer.singleShot(ms, self._refresh_found)
        return True

    def _scan_watch(self):
        """Poll the scan and report when it ends, with what it last said."""
        p = getattr(self, "_scan_proc", None)
        if p is None:
            return
        if p.poll() is None:
            QTimer.singleShot(4000, self._scan_watch)
            return
        rc = p.returncode
        tail = ""
        try:
            with open(os.path.join(_WS, ".scratch", "scan.log")) as fh:
                lines = [ln.rstrip() for ln in fh
                         if ln.strip() and "RTPS_TRANSPORT_SHM" not in ln]
            tail = lines[-1][:160] if lines else ""
        except Exception:                                     # noqa: BLE001
            pass
        if rc == 0:
            self.log("scan finished. %s" % tail)
            self.seq_note.setText("scan finished -- %s" % tail)
        else:
            self.log("SCAN FAILED (exit %d): %s" % (rc, tail), bad=True)
            self.seq_note.setText(
                "SCAN FAILED (exit %d): %s    [see .scratch/scan.log]"
                % (rc, tail))
        self._scan_proc = None
        self._refresh_found()
    def on_goto_pose(self, key):
        """Command a named pose on whichever arms are actually connected.

        THE E-STOP IS CHECKED FIRST, AND IT IS A REFUSAL.
        --------------------------------------------------
        Measured 2026-08-26: HOME was pressed, the trajectory reached the
        bridge, and the bridge answered `zero speed: estop` twice a second
        while this window logged "HOME commanded on left, right". The arm
        could not move and the operator was told it had been told to. That is
        this repository's own "feature present but does nothing", in a button
        whose entire job is to move an arm. Nothing is published while the
        e-stop is engaged, and the reason names the reset button.

        ONE ARM IS A VALID SETUP.
        -------------------------
        Each arm is commanded independently and reported independently: an
        absent right arm must not stop the left from homing. What it must ALSO
        not do is quietly go somewhere else -- the previous version fell back
        per-arm to the SIMULATION topic, so with only the left bridge up, HOME
        drove the real left arm and the SIMULATED right one in the same press,
        and said "commanded on left, right".
        """
        lbl, _why = _POSES.describe(key)

        # ---- e-stop gate
        est = self.bus.snap.get("estop")
        engaged = bool(est[0]) if est else None
        if engaged:
            msg = ("%s REFUSED: the E-STOP is engaged, so the bridge answers "
                   "every command with zero speed. Press 'reset e-stop' at "
                   "the bottom of this window, then try again." % lbl)
            self.log(msg, bad=True)
            self.pose_note.setText(msg)
            return

        # OFF THE Qt THREAD, BECAUSE THIS WAITS FOR AN ARM TO ARRIVE.
        #
        # A planned pose is a SEQUENCE of waypoints, and each leg is held
        # until the arm reaches it -- `_wait_arrival`, up to 20 s apiece.
        # Run inline that is the window frozen for the whole move, e-stop
        # included, and the button audit measured exactly that: "click
        # returns: SCAN POSE, took 20.6 s -- the window was frozen for that
        # long". Pacing relayed moves to the relay's speed cap made it more
        # likely, because a move the arm can actually follow takes longer
        # than one it cannot.
        #
        # Fourth site of this defect in this file tonight -- `arm status`,
        # `START SCENE CAMERA` and the camera doctor were the others. The
        # rule is the same every time: anything that WAITS gets a thread,
        # and the result comes back through a label the refresh already
        # polls.
        self.pose_note.setText("%s: moving. Each waypoint is held until the "
                               "arm reaches it, so a planned pose takes as "
                               "long as the arm takes." % lbl)
        self._pose_out = None
        threading.Thread(target=self._goto_pose_work,
                         args=(key, lbl), daemon=True).start()
        QTimer.singleShot(700, self._goto_pose_render)

    def _goto_pose_render(self, tries=90):
        out = getattr(self, "_pose_out", None)
        if out is None:
            if tries > 0:
                QTimer.singleShot(700,
                                  lambda: self._goto_pose_render(tries - 1))
            else:
                self.pose_note.setText("the pose move did not report back "
                                       "-- see the event log")
            return
        self.pose_note.setText(out)
        self._pose_out = None

    def _goto_pose_work(self, key, lbl):
        sent, failed, sim_used = [], [], []
        for arm in ARMS:
            try:
                q = _POSES.load(key, arm)
            except _POSES.PoseError as e:
                failed.append("%s: %s" % (arm, e))
                continue
            # A PLANNED MOVE IS FOLLOWED IF ONE EXISTS.
            #
            # The scan pose is reached by a checked PATH, not by handing the
            # controller the target and letting it interpolate in joint
            # space. Measured 2026-08-26: the direct joint interpolation from
            # the pick pose to the scan pose took the LEFT hand along the
            # mannequin's forearm at 0.1897 m -- 16 of 31 samples under the
            # margin -- and it hit. Both ENDPOINTS were clear; the middle was
            # not, and only the endpoints were ever checked.
            legs = self._planned_legs(key, arm)
            if legs:
                ok, topic = False, None
                for i, wp in enumerate(legs, 1):
                    ok, topic = self.bus.send_joint_pose(
                        arm, wp, secs=6.0)
                    if not ok:
                        break
                    if i < len(legs):
                        # Each leg is HELD until the arm is there. Firing the
                        # next waypoint immediately would blend the two and
                        # cut the corner -- which is the very corner the
                        # detour exists to avoid.
                        self._wait_arrival(arm, wp)
            else:
                ok, topic = self.bus.send_joint_pose(arm, q, secs=5.0)
            if not ok:
                failed.append("%s arm: not connected -- nothing subscribes to "
                              "/real/%s_arm_controller/joint_trajectory or the "
                              "sim equivalent. Press CONNECT for this arm."
                              % (arm, arm))
                continue
            sent.append("%s -> %s" % (arm, topic))
            if not topic.startswith("/real/"):
                sim_used.append(arm)
        for one in sent:
            self.log("%s commanded: %s (5 s)" % (lbl, one))
        for f in failed:
            self.log("%s REFUSED -- %s" % (lbl, f), bad=True)

        msg = []
        if sent:
            msg.append("%s sent: %s." % (lbl, "; ".join(sent)))
        if sim_used:
            # SAY IT. A press that moves the picture and not the metal is the
            # most confusing outcome available, and it looks like success.
            msg.append("NOTE: %s went to the SIMULATION, not a real arm."
                       % " and ".join(sim_used))
        if failed:
            msg.append("NOT sent: " + " | ".join(failed))
        if sent and engaged is None:
            msg.append("(e-stop state unknown -- no /estop_state publisher; "
                       "if nothing moves, that is the first thing to check.)")
        # BACK THROUGH THE POLLED LABEL, not by touching the widget from
        # this thread. Qt widgets belong to the thread that made them; the
        # render callback runs on the Qt thread and picks this up.
        self._pose_out = "  ".join(msg)

    def _vr_panel(self):
        g = QGroupBox("session details")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)

        # The start button lives in the three-click panel at the top of the
        # tab (self.vr_btn is created there); this section carries only what
        # a running VR session needs: the ticks, the link, the freeze state
        # and the twelve step rows.

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

        # WORN OR ON THE SHELF. The reference watch exists for a headset
        # standing still as the tracking origin (desk operation): a knock
        # announces nothing, so it freezes on 20 mm / 2 deg of drift. Worn
        # on a head, that same gate freezes the moment the operator looks
        # around -- stickily -- and nothing in this window could clear it.
        # This box is the missing control: ticked = the headset moves with a
        # person, so the watch is off; unticked = shelf, watch on. It is
        # applied LIVE to a running safety node and carried into the next
        # launch's environment.
        self.vr_worn = QCheckBox("the headset is being worn (not on a shelf)")
        self.vr_worn.setFont(helvetica(9, True))
        self.vr_worn.setToolTip(
            "Untick when the headset stands on a shelf as the tracking "
            "reference -- then a knock to it freezes everything, on purpose. "
            "Tick when somebody wears it: a worn headset is SUPPOSED to "
            "move, and the freeze would fire on the first head turn.")
        self.vr_worn.toggled.connect(self.on_vr_worn)
        v.addWidget(self.vr_worn)

        # WHY THE ARM IS FROZEN, in plain words, where the operator looks.
        self.vr_freeze_lbl = QLabel("")
        self.vr_freeze_lbl.setFont(helvetica(9, True))
        self.vr_freeze_lbl.setWordWrap(True)
        self.vr_freeze_lbl.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vr_freeze_lbl)

        self.vr_head = QLabel("not started")
        self.vr_head.setFont(helvetica(11, True))
        self.vr_head.setWordWrap(True)
        self.vr_head.setStyleSheet("color:%s" % C_MUTED)
        v.addWidget(self.vr_head)

        # ------------------------------------------------------- THE LINK
        # Two addresses, and they are not the same question:
        #
        #   THIS MACHINE -- what to type into the headset's browser. Known as
        #   soon as the bridge is serving, and needed BEFORE any headset has
        #   connected, which is exactly when a pop-up that has to be clicked
        #   is least useful.
        #
        #   THE HEADSET -- who actually connected. The bridge had this the
        #   whole time and threw it away, so the window could say "1 client"
        #   and nothing more. With two headsets on the bench, or a phone left
        #   on the page from an earlier test, that message is identical
        #   whether or not the right device is on the other end.
        lk = QGroupBox("link")
        lk.setFont(helvetica(9, True))
        lkv = QVBoxLayout(lk)
        # TOP MARGIN LEAVES ROOM FOR THE TITLE. A QGroupBox draws its title
        # INSIDE its own frame, so a 4 px top margin puts the first row under
        # the word "link" -- which is what shipped for about ten minutes and
        # is invisible from a return code: the audit pressed every control in
        # here and passed 277/277 while the label was unreadable.
        lkv.setContentsMargins(6, 16, 6, 4)
        lkv.setSpacing(2)

        self.vr_url_lbl = QLabel("this machine:  (bridge not started)")
        self.vr_url_lbl.setFont(mono(10, True))
        self.vr_url_lbl.setWordWrap(True)
        self.vr_url_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.vr_url_lbl.setToolTip(
            "Open this in the headset's own browser. Every address this "
            "machine has is shown, robot network first -- the headset may be "
            "on either wifi, and the wrong one fails inside the headset where "
            "you cannot see why.")
        lkv.addWidget(self.vr_url_lbl)

        self.vr_headset_lbl = QLabel("headset:  no headset connected")
        self.vr_headset_lbl.setFont(mono(10, True))
        self.vr_headset_lbl.setWordWrap(True)
        self.vr_headset_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.vr_headset_lbl.setToolTip(
            "The address the headset connected FROM, and whether pose frames "
            "are actually arriving. An open socket is not a live headset: a "
            "suspended browser page still answers pings, so this reads the "
            "frame counter rather than the socket count.")
        lkv.addWidget(self.vr_headset_lbl)
        v.addWidget(lk)

        # ------------------------------------------------------- SMOOTHING
        # THE FEEL OF THE TELEOPERATION, where the operator can reach it.
        #
        # These set parameters on a RUNNING vr_pose_mapper, so a change takes
        # effect on the next command rather than the next session -- which is
        # the only way "smoother" can be judged, because it is judged by hand.
        # Every control logs its new value: this panel changes what the arm
        # does, and the audit rule for this window is that such a control must
        # SAY so.
        # The title is short enough not to be elided at this column width.
        # "smoothing -- how the command follows your hand" was, and a
        # QGroupBox elides its title without any indication that it did.
        sg = QGroupBox("smoothing")
        sg.setFont(helvetica(9, True))
        sg.setToolTip("How the commanded pose follows the operator's hand.")
        sv = QVBoxLayout(sg)
        sv.setContentsMargins(6, 16, 6, 4)
        sv.setSpacing(3)

        r0 = QHBoxLayout()
        lab = QLabel("law")
        lab.setFont(helvetica(9))
        r0.addWidget(lab)
        self.vr_smooth = QComboBox()
        self.vr_smooth.setFont(helvetica(9))
        self.vr_smooth.addItems(["one_euro", "ema", "none"])
        self.vr_smooth.setToolTip(
            "one_euro: cutoff rises with hand speed -- still when you are "
            "still, no lag when you move. Measured against a 1.5 mm-rms "
            "tremor and a 0.40 m/s reach: 0.51 mm still / 1.53 mm lag, "
            "against 1.74 / 3.66 for the ema this replaced.\n"
            "ema: the fixed filter that shipped before. Kept so recordings "
            "made before 2026-08-26 reproduce.\n"
            "none: raw. For seeing what the filter is actually doing.")
        self.vr_smooth.currentTextChanged.connect(self.on_vr_smoothing)
        r0.addWidget(self.vr_smooth, 1)
        sv.addLayout(r0)

        # min_cutoff and beta are the two knobs that matter and they pull in
        # opposite directions, so they are shown together rather than buried.
        self.vr_mincut = self._vr_spin(
            sv, "steadiness (min cutoff, Hz)", 0.1, 5.0, 0.1, 0.5,
            "LOWER = steadier when your hand is still. Costs lag only at low "
            "speed, where lag is cheap.",
            lambda x: self.bus.set_param("/vr_pose_mapper", "min_cutoff_hz", x))
        self.vr_beta = self._vr_spin(
            sv, "responsiveness (beta, Hz per m/s)", 0.0, 400.0, 10.0, 100.0,
            "HIGHER = less lag when you move fast, at the price of letting "
            "tremor through during fast motion, where you cannot see it. "
            "NOTE THE UNITS: this signal is in METRES, so the 1-Euro paper's "
            "pixel-scale values (~0.001-1) are about 1000x too small and "
            "leave the filter stuck as a heavy fixed low pass.",
            lambda x: self.bus.set_param("/vr_pose_mapper", "beta", x))

        self.vr_rot_smooth = QCheckBox("smooth the wrist too")
        self.vr_rot_smooth.setFont(helvetica(9))
        self.vr_rot_smooth.setChecked(True)
        self.vr_rot_smooth.setToolTip(
            "Orientation was NOT filtered at all before 2026-08-26 -- the raw "
            "controller quaternion went straight to IK while position got a "
            "low pass. The pads hang off the wrist, so hand tremor arrived "
            "there unattenuated. Untick to compare.")
        self.vr_rot_smooth.toggled.connect(
            lambda on: (self.bus.set_param("/vr_pose_mapper",
                                           "smooth_orientation", bool(on)),
                        self.log("wrist smoothing -> %s" % ("on" if on else "OFF"))))
        sv.addWidget(self.vr_rot_smooth)

        # WHAT THE FILTER IS DOING RIGHT NOW. Without this, "feels laggy" and
        # "feels jittery" are unfalsifiable -- which is exactly what the old
        # fixed alpha was.
        self.vr_smooth_state = QLabel("mapper not running")
        self.vr_smooth_state.setFont(mono(9))
        self.vr_smooth_state.setWordWrap(True)
        self.vr_smooth_state.setStyleSheet("color:%s" % C_MUTED)
        sv.addWidget(self.vr_smooth_state)
        # BEHIND A TICK, NOT IN THE FACE. The operator called the RUN tab
        # what it was: too many options. The smoothing knobs matter when
        # tuning feel and never during a session start, so they hide until
        # asked for. The live one-line state stays visible logic-side --
        # it reappears with the panel.
        self.vr_adv = QCheckBox("advanced: smoothing settings")
        self.vr_adv.setFont(helvetica(8))
        self.vr_adv.toggled.connect(
            lambda on: (sg.setVisible(bool(on)),
                        self.log("VR smoothing settings %s"
                                 % ("shown" if on else "hidden"))))
        v.addWidget(self.vr_adv)
        sg.setVisible(False)
        v.addWidget(sg)

        # THE TWELVE CHECK ROWS, HIDDEN UNTIL A BRING-UP RUNS. Idle, they
        # were twelve lines of grey text the operator read as clutter --
        # rightly, because with nothing started they say nothing. They
        # appear the moment START runs them (on_vr_start), which is the
        # moment a failed row and its fix button matter.
        self.vr_steps_box = QWidget()
        sbl = QVBoxLayout(self.vr_steps_box)
        sbl.setContentsMargins(0, 0, 0, 0)
        sbl.setSpacing(0)
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
            sbl.addWidget(box)
            self.vr_rows[key] = (box, t, why, fix)
        self.vr_steps_box.setVisible(False)
        v.addWidget(self.vr_steps_box)

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
        # THE ONLY WAY OUT OF A REFERENCE FREEZE. The freeze is sticky by
        # design (poses keep flowing, so the ordinary unfreeze is true the
        # whole time it is wrong) and until now the service existed with no
        # control anywhere in the window.
        b3 = QPushButton("accept the headset's new position")
        b3.setFont(helvetica(9))
        b3.setToolTip(
            "The headset (the tracking reference) moved and everything "
            "froze. Press this to accept where it now stands. Every pose is "
            "then in the new frame -- re-run the operator yaw calibration "
            "if the move was more than a nudge.")
        b3.clicked.connect(self.on_vr_rebase)
        row.addWidget(b3)
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

        # THE ONE-BUTTON BRING-UP MUST NOT OPEN ITS OWN RViz. This window
        # embeds one; `use_rviz:=true` puts a second top-level RViz over the
        # top of it, which is the 2026-08-27 "two RVizs" finding. That fix
        # went through every launch SPEC and missed this path, because the
        # VR button spawns `run_teleop.sh` from `vr_bringup` and not from a
        # spec.
        os.environ["SRL_VR_SIM_RVIZ"] = "false"

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
            # The one-button bring-up spawns vr_safety_node from THIS
            # process's environment; without these the tick reached the
            # script route and silently not the button.
            os.environ.update(VR_REQUIRE_OBSERVER="false",
                              VR_ALLOW_REAL_ARM="true",
                              VR_ALLOW_REAL_NO_OBSERVER="true")
            self.vr_alone_note.setText(
                "WORKING ALONE since %s. Nobody is holding an e-stop. This "
                "is in the log." % rec["granted_at_iso"])
            self.vr_alone_note.setStyleSheet("color:%s" % C_WARN)
            self.log("OBSERVER BYPASS TAKEN at %s -- no observer for this "
                     "session" % rec["granted_at_iso"], bad=True)
        else:
            OB.clear(who="gui", note="unticked")
            for k in ("VR_REQUIRE_OBSERVER", "VR_ALLOW_REAL_ARM",
                      "VR_ALLOW_REAL_NO_OBSERVER"):
                os.environ.pop(k, None)
            self.vr_alone_note.setText("Observer required.")
            self.vr_alone_note.setStyleSheet("color:%s" % C_MUTED)
            self.log("observer bypass cancelled -- an observer is required "
                     "again")

    def on_vr_worn(self, on):
        """Worn headset: the reference watch is wrong by construction.

        Applied LIVE (the safety node reads the parameter per message) and
        recorded for the next launch via _launch_env. The audit rule: a
        control that changes what the next run does must SAY so.
        """
        # set_param is loud on failure through the bus notes; if no safety
        # node is running yet, the setting still reaches the next launch:
        # spec launches read _launch_env, and the one-button bring-up
        # (vr_bringup) reads THIS process's environment at spawn time.
        os.environ["VR_WATCH_REFERENCE"] = "false" if on else "true"
        self.bus.set_param("/vr_safety_node", "watch_tracking_reference",
                           not bool(on))
        if on:
            self.log("headset marked WORN: the moved-reference freeze is "
                     "OFF, live and for the next VR start. A worn headset "
                     "is supposed to move.")
        else:
            self.log("headset marked ON THE SHELF: the moved-reference "
                     "freeze is ON, live and for the next VR start. A knock "
                     "now freezes everything, on purpose.")

    def on_vr_rebase(self):
        """Accept the tracking reference where it now stands. The result
        arrives asynchronously as a bus note; call_trigger refuses loudly
        when the service is absent."""
        self.log("accepting the moved tracking reference...")
        self.bus.call_trigger("/vr/rebase_reference")

    def on_vr_start(self):
        if self._vr_busy:
            self.log("VR bring-up is already running")
            return
        self._vr_busy = True
        self.vr_btn.setEnabled(False)
        self.vr_steps_box.setVisible(True)
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
        if kind == "fixed":
            ok, msg, name = res
            self._vr_busy = False
            self.vr_btn.setEnabled(True)
            self.vr_note.setText(msg)
            self.log("VR fix %s: %s" % (key, msg), bad=not ok)
            # Only re-run when the repair actually finished its work. A
            # failed repair leaves the diagnosis on screen instead of
            # replacing it with a fresh sweep that says the same thing.
            if ok and not name.endswith("_help"):
                QTimer.singleShot(400, self.on_vr_start)
            return
        # done
        self._vr_busy = False
        self.vr_btn.setEnabled(True)
        state, head = vrb.verdict([r for _k, r in out], keyed=out)
        # THE ARBITER GOES ON TOP, AND ONLY ONTO A STACK THAT CAME UP.
        # Starting the autonomy layer over a failed bring-up would put a
        # second thing to diagnose on top of the first, and the operator
        # would be reading arbiter states from a chain that never ran.
        if getattr(self, "_vr_want_shared", False):
            self._vr_want_shared = False
            if state == vrb.OK:
                self._run_raw(
                    "shared_autonomy",
                    ["ros2", "launch", "srl_autonomy",
                     "shared_autonomy.launch.py", "teleop:=false"])
                self.log("SHARED AUTONOMY: perception and the arbiter "
                         "launching on top of the VR chain (teleop:=false -- "
                         "no second stack). The arbiter's state is in the "
                         "SHARED AUTONOMY panel; it needs ~15 s.")
            else:
                self.log("SHARED AUTONOMY NOT STARTED: the VR bring-up did "
                         "not come up clean, and an arbiter over a broken "
                         "chain is a second fault stacked on the first. Fix "
                         "what the steps above name, then press START again.",
                         bad=True)
        col = {vrb.OK: C_OK, vrb.FAILED: C_BAD,
               vrb.UNKNOWN: C_UNKNOWN}.get(state, C_UNKNOWN)
        self.vr_head.setText(head.upper())
        self.vr_head.setStyleSheet("color:%s" % col)
        self.log("VR bring-up: %s" % head, bad=(state == vrb.FAILED))
        if state == vrb.OK:
            # The simulation is up by now (it is an early bring-up step);
            # opening its view here is what makes "click VR, see the sim"
            # one click instead of two.
            self._ensure_rviz()
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
        # OFF THE UI THREAD, AND THE SEQUENCE DOES NOT RE-RUN UNTIL IT IS
        # DONE. Restarting the simulation now WAITS for the simulation --
        # up to two minutes -- because returning early is what produced
        # "the simulation is not running" from a simulation that was
        # thirty seconds into a normal start, with RViz still grey. Two
        # minutes of a frozen window is not an improvement, so the repair
        # runs in a worker and the row says what it is doing.
        if self._vr_busy:
            self.log("VR bring-up is already running", bad=True)
            return
        self._vr_busy = True
        self.vr_btn.setEnabled(False)
        self.vr_note.setText("working... (%s -- this window is waiting for "
                             "it, not stuck)" % name.replace("_", " "))
        self.log("VR fix %s: %s -- working..." % (key, name))
        world = self._vr_world or vrb.World(ws=_WS)

        def work():
            try:
                ok, msg = fn(world)
            except Exception as e:                            # noqa: BLE001
                ok, msg = False, "could not do it: %r" % (e,)
            with self._vr_lock:
                self._vr_pending.append(("fixed", key, (ok, msg, name),
                                         list(self._vr_results)))
        threading.Thread(target=work, daemon=True).start()

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
            b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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
        g = QGroupBox("Run a task")
        g.setFont(helvetica(11, True))
        v = QVBoxLayout(g)
        # ONE RUNNER, NOT A WALL. The task and demo groups used to render
        # one button per (task x mode) -- 55 of them, 24 saying "vr"
        # somewhere -- and the operator called it what it was: unusable.
        # The specs, keys and dispatch path are unchanged; only the
        # presentation collapses to two choices and one button.
        self._task_runner(v)
        # DEMOTED, NOT DELETED, AND LAST. These launch a single stack piece
        # by hand; the session's own controls are the three-click panel at
        # the top. Kept because a stuck session sometimes needs exactly one
        # piece relaunched -- but labelled as what it is, below the work.
        self._spec_group(v, "mode", "Advanced  --  launch one piece by hand")
        # NO SECOND STOP BUTTON HERE. `STOP everything this window launched`
        # at the top of the mode panel is the same call to `on_stop_jobs`,
        # and a second one at the bottom of an Advanced group is a control
        # the operator has to decide between for no reason.
        return g

    # Plain words for the five clip-tree modes, in the clip tree's order.
    # Keyed by the short name inside the spec label's brackets, which comes
    # from the mode directory name -- so a mode the tree gains shows up
    # under its raw name rather than vanishing.
    MODE_WORDS = [
        ("master teleop", "you drive -- master arm"),
        ("vr teleop", "you drive -- VR controllers"),
        ("shared autonomy", "you drive, robot helps -- master arm"),
        ("vr shared", "you drive, robot helps -- VR"),
        ("full autonomy", "robot does it alone"),
    ]

    def _task_runner(self, layout):
        """Pick what, pick how, press RUN."""
        specs = [x for x in self._ensure_specs()
                 if x.group in ("task", "demo")]
        # label "T1 pick and place  [vr teleop]" -> ("T1 pick and place",
        # "vr teleop"). The bracket form is asserted by validate()'s
        # duplicate check working at all, but parse defensively: a label
        # with no bracket becomes its own activity with a blank mode.
        self._task_pairs = {}
        acts = []
        demo_bases = set()
        for sp in specs:
            base, _, rest = sp.label.partition("  [")
            short = rest[:-1] if rest.endswith("]") else ""
            self._task_pairs[(base, short)] = sp
            if base not in acts:
                acts.append(base)
            if sp.group == "demo":
                demo_bases.add(base)

        lab = QLabel("Tasks and demonstrations")
        lab.setFont(helvetica(10, True))
        layout.addWidget(lab)

        r1 = QHBoxLayout()
        t = QLabel("do")
        t.setFont(helvetica(9))
        r1.addWidget(t)
        self.task_pick = QComboBox()
        self.task_pick.setFont(helvetica(9))
        for a in acts:
            self.task_pick.addItem(
                a + ("   (demo -- no trial data)" if a in demo_bases else ""),
                a)
        self.task_pick.setToolTip(
            "What the robot should do. Demonstrations produce no trial "
            "data and say so on the clip.")
        r1.addWidget(self.task_pick, 1)
        layout.addLayout(r1)

        r2 = QHBoxLayout()
        t = QLabel("how")
        t.setFont(helvetica(9))
        r2.addWidget(t)
        self.mode_pick = QComboBox()
        self.mode_pick.setFont(helvetica(9))
        shorts = {s for (_, s) in self._task_pairs}
        for short, words in self.MODE_WORDS:
            if short in shorts:
                self.mode_pick.addItem(words, short)
        for short in sorted(shorts - {s for s, _ in self.MODE_WORDS}):
            if short:
                self.mode_pick.addItem(short, short)
        self.mode_pick.setToolTip(
            "Who is in control. The same waypoints are commanded under "
            "every mode; the mode is the path the command travels.")
        # Shared autonomy is the study's condition of interest and the
        # operator's requested default for every run (2026-08-27).
        i = self.mode_pick.findData("shared autonomy")
        if i >= 0:
            self.mode_pick.setCurrentIndex(i)
        r2.addWidget(self.mode_pick, 1)
        layout.addLayout(r2)

        self.task_run_note = QLabel("")
        self.task_run_note.setFont(helvetica(8))
        self.task_run_note.setWordWrap(True)
        self.task_run_note.setStyleSheet("color:%s" % C_MUTED)
        layout.addWidget(self.task_run_note)

        self.task_run_btn = QPushButton("RUN")
        self.task_run_btn.setFont(helvetica(11, True))
        self.task_run_btn.setMinimumHeight(34)
        self.task_run_btn.clicked.connect(self.on_run_selected_task)
        layout.addWidget(self.task_run_btn)

        self.task_pick.currentIndexChanged.connect(self._task_pick_changed)
        self.mode_pick.currentIndexChanged.connect(self._task_pick_changed)
        self._task_pick_changed()

    def _selected_task_spec(self):
        base = self.task_pick.currentData()
        short = self.mode_pick.currentData()
        return self._task_pairs.get((base, short))

    def _task_pick_changed(self, *_):
        """The RUN button must say, BEFORE the press, what it will refuse.

        A pair the manifest does not offer (T1 runs under full autonomy
        only) or offers disabled must grey the button with the reason on
        it -- a RUN that exits 2 on press looks exactly like one that
        launched something invisible.
        """
        sp = self._selected_task_spec()
        if sp is None:
            self.task_run_btn.setEnabled(False)
            self.task_run_note.setText(
                "%s does not run under '%s' -- pick another mode."
                % (self.task_pick.currentData(),
                   self.mode_pick.currentText()))
        elif not sp.enabled:
            self.task_run_btn.setEnabled(False)
            self.task_run_note.setText(sp.disabled_reason or "unavailable")
        else:
            self.task_run_btn.setEnabled(True)
            self.task_run_note.setText(sp.note or "")
        # The audit's own rule: a control that changes what the next run
        # does must SAY so. These two combos decide exactly that.
        self.log("RUN is set to: %s / %s%s"
                 % (self.task_pick.currentData(),
                    self.mode_pick.currentText(),
                    "" if sp is not None and sp.enabled
                    else "  (refused -- see the note)"))

    def on_run_selected_task(self):
        sp = self._selected_task_spec()
        if sp is None or not sp.enabled:
            return
        self.log("RUN: %s" % sp.label)
        self.on_launch(sp)

    def _diag_launchers(self):
        g = QGroupBox("Something is wrong?  Checks and fixes")
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
        b2.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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

        b = QPushButton("check what is installed")
        b.setFont(helvetica(9, True))
        b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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
        self.inst_look = QPushButton("LOOK AT THE TABLE")
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
        # THE BOTTOM STRIP: the scene camera, ALWAYS ON, where the
        # divergence table alone used to sit (operator, 2026-08-27). The
        # sim->real gap stays beside it in a slimmer column -- it is the
        # number the lag trip acts on and hiding it entirely would make
        # the first trip unexplainable from the window.
        strip = QHBoxLayout()
        strip.setSpacing(4)
        strip.addWidget(self._scene_cam_panel(), 3)
        strip.addWidget(self._divergence_panel(), 2)
        v.addLayout(strip)
        return w

    def _scene_cam_panel(self):
        g = QGroupBox("SCENE CAMERA  --  live, with detections")
        g.setFont(helvetica(11, True))
        bl = QVBoxLayout(g)
        bl.setContentsMargins(4, 14, 4, 2)
        self.scene_view = swp.FeedView(
            dict(bg=C_BG, line=LINE, muted=C_MUTED, accent=C_OK, bad=C_BAD,
                 warn=C_WARN, unknown=C_UNKNOWN))
        # SMALL MINIMUM, on purpose: this strip shares the window's width
        # budget with the master-arm column, and a 240 px floor here was
        # measured (screenshot, 2026-08-27) squeezing that column until its
        # dials overdrew their own labels.
        self.scene_view.setMinimumSize(160, 100)
        self.scene_view.setToolTip(
            "The room camera. When the vision layer runs, this is its "
            "ANNOTATED frame -- boxes, labels, or the refusal burned into "
            "the picture; otherwise the raw stream.")
        bl.addWidget(self.scene_view, 1)

        # THE PUBLISHER, REACHABLE. This panel SUBSCRIBED to
        # /scene_camera/image_raw and nothing in the window -- or in any
        # launch spec, mode sequence or startup script -- ever started
        # `scene_camera_node`. It is a registered entry point that only a
        # verification script ran, so the panel was empty for every operator
        # who did not know to type `ros2 run srl_perception scene_camera_node`
        # at a terminal. GUI RULE: the window is the interface; a capability
        # the window cannot reach does not exist.
        row = QHBoxLayout()
        row.setSpacing(4)
        self.scene_cam_btn = QPushButton("START SCENE CAMERA")
        self.scene_cam_btn.setFont(helvetica(9, True))
        self.scene_cam_btn.setToolTip(
            "Starts scene_camera_node, which is what publishes "
            "/scene_camera/image_raw into the view above. It probes for the "
            "node that actually DELIVERS an MJPG frame rather than taking "
            "/dev/video0 -- a RealSense presents six video nodes and several "
            "open cleanly while returning nothing for ever.")
        self.scene_cam_btn.clicked.connect(self.on_scene_camera_toggle)
        self.buttons["scene_camera"] = self.scene_cam_btn
        row.addWidget(self.scene_cam_btn, 1)
        bl.addLayout(row)
        self.scene_cam_note = QLabel("")
        self.scene_cam_note.setFont(helvetica(8))
        self.scene_cam_note.setWordWrap(True)
        self.scene_cam_note.setStyleSheet("color:%s" % C_MUTED)
        bl.addWidget(self.scene_cam_note)
        self._scene_cam_timer = QTimer(self)
        self._scene_cam_timer.timeout.connect(self._scene_cam_refresh)
        self._scene_cam_timer.start(3000)
        QTimer.singleShot(500, self._scene_cam_refresh)
        return g

    # ------------------------------------------------------- scene camera
    @staticmethod
    def _scene_cam_pids():
        try:
            out = subprocess.run(
                ["pgrep", "-f", "srl_perception/scene_camera_node"],
                capture_output=True, text=True, timeout=5).stdout
        except Exception:                                     # noqa: BLE001
            return []
        return [int(x) for x in out.split() if x.strip().isdigit()]

    def on_scene_camera_toggle(self):
        import signal
        pids = self._scene_cam_pids()
        if pids:
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGINT)
                except OSError as e:
                    self.log("scene camera: could not signal %d: %s" % (pid, e),
                             bad=True)
            self.log("scene camera: SIGINT sent to %s"
                     % ", ".join(str(p) for p in pids))
            self.scene_cam_note.setText("stopping...")
        else:
            # NO /dev/video* IS ITS OWN FAULT, AND IT IS NOT A ROS FAULT.
            # Under WSL the cameras arrive over usbip, and the service drops
            # them on its own. Saying so here costs one glob and saves the
            # operator debugging a node that cannot possibly work.
            import glob
            if not glob.glob("/dev/video*"):
                # REPAIR IT, DO NOT JUST COMPLAIN. attach and detach need NO
                # administrator rights -- measured 2026-08-29, usbipd.exe runs
                # straight from WSL -- so the window can fix this itself
                # instead of handing the operator a command to type. The stale
                # case matters most: after the usbipd service restarts,
                # Windows still believes the device is attached and a plain
                # `attach` REFUSES, so the repair has to detach first. That is
                # what usb_cameras.py --fix does.
                # OFF THE Qt THREAD. `usb_cameras.py --fix` talks to
                # usbipd.exe across the WSL boundary and is given 120 s to do
                # it. Run inline, that is up to two minutes with the WHOLE
                # WINDOW frozen -- the e-stop included, because it is drawn by
                # the same event loop. The button audit's slow-press check
                # caught it at 24.1 s on a loaded machine and passed on an
                # idle one, which is the worse failure: it works in testing
                # and stops working on a lab day. Exactly the defect already
                # fixed once for `arm status`; this is the second site.
                self.scene_cam_note.setText(
                    "no /dev/video* -- attaching the cameras from Windows...")
                self.log("scene camera: no /dev/video*, running "
                         "usb_cameras.py --fix (in the background)")
                self.bus.submit(self._scene_cam_attach,
                                label="attaching the USB cameras")
                QTimer.singleShot(2500, self._scene_cam_refresh)
                return
            self._run_raw("scene_camera_node",
                          ["ros2", "run", "srl_perception", "scene_camera_node"])
            self.log("scene camera: START requested")
            self.scene_cam_note.setText("starting...")
        QTimer.singleShot(2500, self._scene_cam_refresh)

    def _scene_cam_attach(self):
        """Attach the USB cameras, then start the node. NOT on the Qt thread.

        Runs on the bus worker, so the window -- and the e-stop on it -- stay
        live for the whole two-minute budget. The result comes back through
        the same refresh timer the rest of the panel uses, so there is one
        path that reports what happened rather than two.
        """
        import glob as _glob
        try:
            r = subprocess.run(
                [sys.executable,
                 os.path.join(_WS, "scripts", "usb_cameras.py"), "--fix"],
                capture_output=True, text=True, timeout=120)
            for ln in (r.stdout or "").strip().splitlines():
                self.bus.note("  %s" % ln.strip())
        except Exception as e:                                # noqa: BLE001
            self.bus.note("usb_cameras.py failed: %r" % (e,), bad=True)
            return
        if not _glob.glob("/dev/video*"):
            self.bus.note(
                "STILL no /dev/video*. If usbipd says its service is stopped, "
                "that needs ONE administrator command and it also explains a "
                "missing master arm: Start-Service usbipd", bad=True)
            return
        self.bus.note("cameras attached -- starting the scene camera")
        self._run_raw("scene_camera_node",
                      ["ros2", "run", "srl_perception", "scene_camera_node"])

    def _scene_cam_refresh(self):
        if not hasattr(self, "scene_cam_btn"):
            return
        pids = self._scene_cam_pids()
        if pids:
            self.scene_cam_btn.setText("STOP SCENE CAMERA")
            fresh = time.time() - getattr(self, "scene_img_t", 0.0) < 3.0
            self.scene_cam_note.setText(
                "running (pid %s) -- %s"
                % (", ".join(str(p) for p in pids),
                   "publishing frames" if fresh
                   else "NO FRAMES YET on /scene_camera/image_raw"))
            self.scene_cam_note.setStyleSheet(
                "color:%s" % (C_OK if fresh else C_WARN))
        else:
            self.scene_cam_btn.setText("START SCENE CAMERA")
            self.scene_cam_note.setText("not running -- the view above stays "
                                        "empty until this is started")
            self.scene_cam_note.setStyleSheet("color:%s" % C_MUTED)

    def _rviz_keepalive(self):
        """Restart the COMMANDED view when the stack it draws appears.

        Two cases, both measured on 2026-08-27: RViz started at window-open
        against no stack never recovers when one arrives (grey grid, dead
        fixed frame); and an RViz that died stays dead unless a mode
        sequence happens to end. Neither is the operator's job to notice.
        """
        now = time.monotonic()
        if now - getattr(self, "_rviz_kept_t", 0.0) < 5.0:
            return
        self._rviz_kept_t = now
        if getattr(self.args, "no_rviz", False):
            return
        if _stack_pids() <= 0:
            return
        procs = list(getattr(self, "rviz", []))
        alive = [p for _k, p in procs if p.poll() is None]
        if alive and not getattr(self, "_rviz_prestack", False):
            return
        # KILL THE RVIZ PROCESS, NOT ITS GROUP. THIS WAS KILLING THE WINDOW.
        #
        # THE BUG, found 2026-08-30 and mis-attributed for days. RViz is
        # spawned with `preexec_fn=_die_with_parent`, which sets a
        # parent-death signal and does NOT call setsid -- so rviz2 stays in
        # THE GUI'S OWN PROCESS GROUP. `os.killpg(os.getpgid(p.pid), 15)`
        # therefore sent SIGTERM to that group: the window signalled ITSELF,
        # and every stack it had launched with it.
        #
        # It fires exactly when `_stack_pids() > 0`, which is the moment a
        # stack comes up. That is the whole of "the GUI's ROS context dies if
        # a simulation comes up under it" -- recorded as a mysterious rclpy
        # ExternalShutdownException, blamed on transports and start order,
        # and worked around by launching the stack first. It was never a ROS
        # problem. Pressing START MASTER TELEOP made the window kill itself,
        # and the launch log shows the stack shutting down CLEANLY on SIGINT
        # a moment later because its parent had gone.
        #
        # A group kill is right for `ros2 launch` and `ros2 run`, which are
        # wrappers with children (HARD CONSTRAINT 9). rviz2 is a single
        # process and needs no group -- and taking the group here is only
        # safe if the child is in one of its own, which this one is not.
        for _k, p in procs:
            if p.poll() is None:
                try:
                    p.terminate()                     # this pid, nothing else
                    try:
                        p.wait(timeout=5)
                    except Exception:                 # noqa: BLE001
                        p.kill()
                except Exception:                     # noqa: BLE001
                    pass
        self.rviz = []
        self.log("the simulation is up -- restarting the COMMANDED view "
                 "so it can see the robot"
                 if alive else
                 "the COMMANDED view was gone and a stack is up -- "
                 "restarting it")
        self.start_rviz()

    def _refresh_scene_view(self, s):
        """Prefer the vision layer's annotated frame; fall back to raw.

        Freshness decides, not preference order: a dead vision node must
        not freeze this view while raw frames still flow.
        """
        view = getattr(self, "scene_view", None)
        if view is None:
            return
        ov, ov_t = s.get("scene_ov_img"), s.get("scene_ov_img_t", 0.0)
        raw, raw_t = s.get("scene_img"), s.get("scene_img_t", 0.0)
        now = time.monotonic()
        if ov is not None and now - ov_t < 3.0:
            view.set_frame(ov, ov_t)
            view.set_overlay(None, "detections ON", "ok")
        elif raw is not None:
            view.set_frame(raw, raw_t)
            view.set_overlay(None, "raw stream -- vision layer not "
                                   "running", "unknown")
        else:
            view.set_overlay(None, "no scene camera frame has ever "
                                   "arrived", "unknown")

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
        g = QGroupBox("SIM -> REAL gap")
        g.setFont(helvetica(11, True))
        grid = QGridLayout(g)
        self.div_head = QLabel()
        self.div_head.setFont(helvetica(10))
        # WRAPPED, and that is load-bearing: an unwrappable one-line header
        # sets this panel's MINIMUM width, and since the panel shares its
        # strip with the scene camera (2026-08-27) that minimum was taken
        # out of the master-arm column, which drew its dials over their own
        # labels. Measured from a screenshot, invisible from a return code.
        self.div_head.setWordWrap(True)
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
            self.div_joint[a].setWordWrap(True)
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
        b = QPushButton("test the indicator lights")
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
        # Remember whether a stack existed when this RViz started. Started
        # before one, it shows a grey grid and "Fixed Frame [world] does
        # not exist" FOR EVER -- the view does not recover when the robot
        # appears, and the operator reads it as "there is no rviz"
        # (measured live, 2026-08-27). _rviz_keepalive restarts it once a
        # stack is up.
        self._rviz_prestack = _stack_pids() <= 0
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

        ONE SOURCE, in `srl_teleop.rviz_windows`. The three filters and the
        measured defect they encode are documented there; the VR bring-up
        waits on the same answer, and two copies of this parser would mean
        the window the GUI embeds and the window the bring-up waits for
        could be different windows.
        """
        from srl_teleop.rviz_windows import real_windows
        return real_windows()

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
        """The bottom-bar E-STOP. LABELLED, so the press is on record AT ONCE.

        This was the one `bus.submit` in the file with no label, and it was
        the worst possible one to leave out. `submit` notes the label
        IMMEDIATELY and queues the action; without a label nothing is written
        until the 0.05 s drain timer runs, which under load is hundreds of
        milliseconds. `submit`'s own docstring names the consequence: "a
        press with no trace cannot be told from a disconnected signal, which
        is the whole bug class the button audit exists for."

        Caught by that audit on a loaded machine -- `click: E-STOP, 0 log
        entries` -- having passed on an idle one, which is the shape that
        works in testing and fails on a lab day. The publish itself was never
        broken; the RECORD of it was late, and for a stop control the record
        is half the point.
        """
        self.bus.submit(lambda: (self.bus.publish_once(Bool, "/estop", True),
                                 self.bus.note("E-STOP published")),
                        label="E-STOP pressed")

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
        # One-shot per-key extras, set by the START handlers -- today the
        # virtual Teensy's serial_port for a master run with no hardware.
        extra = getattr(self, "_stack_extra_args", {}).pop(spec.key, None)
        if extra:
            spec = spec.with_argv(spec.argv + list(extra))
            self.bus.note("launching %s with %s"
                          % (spec.label, " ".join(extra)))
        return spec

    def _launch_env(self, spec):
        """The environment this spec runs with, carrying the session's gates.

        THE VR SAFETY GATES REACH THE NODE THAT ENFORCES THEM. `start_vr_wifi`
        started `vr_safety_node` with its shut defaults and no way to change
        them, so pressing the VR button on a rig with the arms connected
        launched everything and left the mapper frozen on an interlock no
        control in this window could clear. The operator's only route was to
        kill the node and relaunch it by hand -- which is how an interlock
        stops being one: the workaround removes it entirely instead of
        recording that it was bypassed.

        The tick box is already the place that decision is made and already
        writes the audit line. This is the wire from it to the process. With
        the box clear, nothing is set and the defaults stand.
        """
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        if spec.key == "vr" and getattr(self, "vr_alone", None) is not None \
                and self.vr_alone.isChecked():
            env.update(VR_REQUIRE_OBSERVER="false",
                       VR_ALLOW_REAL_ARM="true",
                       VR_ALLOW_REAL_NO_OBSERVER="true")
            self.bus.note(
                "VR safety launched with the observer requirement BYPASSED "
                "-- you ticked 'working alone'. Logged to "
                "recordings/observer_bypass_log.jsonl.", bad=True)
        if spec.key == "vr" and getattr(self, "vr_worn", None) is not None \
                and self.vr_worn.isChecked():
            # A worn headset moves with a head; latching its first pose as
            # a fixed reference freezes the run on the first head turn.
            env.update(VR_WATCH_REFERENCE="false")
            self.bus.note(
                "VR safety launched for a WORN headset -- the "
                "moved-reference freeze is off for this session.")
        return env

    def _spawn(self, spec):
        env = self._launch_env(spec)
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
        # The virtual Teensy is not a launched spec, but it is this
        # window's process and STOP means stop.
        vt = getattr(self, "_vteensy", None)
        if vt is not None and vt.poll() is None:
            try:
                os.killpg(os.getpgid(vt.pid), 2)
                n += 1
                self.log("virtual Teensy stopped")
            except Exception:                                 # noqa: BLE001
                pass
        self._vteensy = None
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

    #: Seconds of snapshot staleness past which this window's ROS node is
    #: treated as DEAD rather than slow. `Bus._snapshot` runs on a 0.1 s
    #: timer inside the ROS executor, so `snap["t"]` stops advancing the
    #: moment that executor stops -- which makes it the liveness signal, with
    #: no new plumbing and nothing to keep in sync.
    ROS_DEAD_S = 3.0

    def ros_stale_s(self, s=None):
        """How long since the ROS side of this window last produced a
        snapshot, or None if it is keeping up.

        THE WINDOW CAN OUTLIVE ITS OWN ROS NODE, and it did on 2026-08-29.
        Qt kept painting, every panel kept its last value, and the arm panel
        read "bridge up, NO DATA" -- blaming the ARM for the window's own
        deafness while `/real/joint_states` was publishing at 11.9 Hz and
        three other nodes answered `ros2 param list` normally. Every service
        button was inert at the same time, because `bus.submit` appends to a
        queue drained by a timer that was no longer running.

        A frozen picture of a rig is worse than a blank one: it is a reading
        the operator has no reason to distrust.
        """
        s = self.bus.snap if s is None else s
        t = s.get("t")
        if not t:
            return None
        age = time.monotonic() - t
        return age if age > self.ROS_DEAD_S else None

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
        self._refresh_vr_link(s)
        self._refresh_vr_smoothing(s)
        self._refresh_vr_safety(s)
        self._refresh_shared(s)
        self._refresh_scene_cv(s)
        self._refresh_scene_view(s)
        self._rviz_keepalive()
        self._refresh_arms(s)

        # ---- banner. A narration override wins: the tutorial recorder drives
        # the banner as its caption track, and refresh() runs at 10 Hz, so
        # without this the narration is overwritten before a frame is
        # captured -- the caption was invisible in the first take.
        if getattr(self, "_narration", None):
            self.banner.setText(self._narration[0])
            self.banner.setStyleSheet(self._narration[1])
        elif self.ros_stale_s(s) is not None:
            # AHEAD OF THE E-STOP SLAB, and deliberately. With the executor
            # stopped, `es` is a value read some time ago and every other
            # reading in the window is the same -- so the honest banner is
            # not what the rig was doing, it is that this window can no
            # longer see the rig OR send to it. The physical e-stop is
            # unaffected by a dead Qt process.
            self.banner.setText(
                "THIS WINDOW'S ROS CONNECTION IS DEAD (%.0f s) -- every "
                "reading below is FROZEN and no button will send. Restart "
                "the window." % self.ros_stale_s(s))
            self.banner.setStyleSheet(
                "color:#0b0f13;background:%s;padding:5px;letter-spacing:1px"
                % C_BAD)
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

    def _refresh_arms(self, s):
        """Three different faults, told apart, once a second.

        "Not connected" is useless on its own. These are the three states
        this session actually hit, each needing a different action:

          OFF THE NETWORK   no ping. Power or cable -- no button helps.
          NO BRIDGE         arm answers, nothing is talking to it. CONNECT.
          NO DATA           a bridge is running but no joint states are
                            arriving: the session dropped (this is what
                            "Broken pipe" in the log looks like from here).
                            DISCONNECT, then CONNECT.

        Ping is cached and run off the Qt thread -- a blocking ping in
        refresh() at 10 Hz would stall the window, and this panel is not
        worth a frozen e-stop.
        """
        if not hasattr(self, "arm_state_lbl"):
            return
        now = time.time()
        if not hasattr(self, "_arm_ping"):
            self._arm_ping = {}
            self._arm_ping_t = 0.0
        if now - self._arm_ping_t > 4.0:
            self._arm_ping_t = now
            threading.Thread(target=self._ping_arms, daemon=True).start()
        js = s.get("real_js_arms") or {}
        deaf = self.ros_stale_s(s)
        for arm, lbl in self.arm_state_lbl.items():
            up = self._arm_ping.get(arm)
            pids = self._bridge_pids(arm)
            fresh = arm in (self.bus.real_arms_seen()
                            if hasattr(self.bus, "real_arms_seen") else {})
            if up is None:
                lbl.setText("checking...")
                col = C_UNKNOWN
            elif not up:
                lbl.setText("OFF THE NETWORK")
                col = C_BAD
            elif not pids:
                lbl.setText("reachable, NO BRIDGE")
                col = C_WARN
            elif deaf is not None:
                # NAME THE RIGHT SUBSYSTEM. `fresh` is false whenever this
                # window has not stamped an arrival recently -- and that is
                # true both when the ARM stopped publishing and when THIS
                # WINDOW stopped receiving. The two were rendered
                # identically, as "bridge up, NO DATA", which sent the
                # operator to the arm on 2026-08-29 while the arm was
                # publishing at 11.9 Hz and the window's own ROS node was
                # dead. Distinguish them before saying either.
                lbl.setText("WINDOW NOT RECEIVING")
                col = C_BAD
            elif not fresh:
                lbl.setText("bridge up, NO DATA")
                col = C_BAD
            else:
                lbl.setText("CONNECTED")
                col = C_OK
            lbl.setStyleSheet("color:%s" % col)

    def _ping_arms(self):
        for arm, ip in self.ARM_IPS.items():
            try:
                r = subprocess.run(["ping", "-c1", "-W1", ip],
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=4)
                self._arm_ping[arm] = (r.returncode == 0)
            except Exception:                                 # noqa: BLE001
                self._arm_ping[arm] = False

    def _refresh_vr_link(self, s):
        """The two addresses of a VR session, live.

        THE SOCKET COUNT IS NOT THE HEADSET. A suspended browser page keeps
        answering WebSocket pings, so `clients` reads 1 with nothing arriving
        -- measured 2026-08-19, when the headset came off and the bridge went
        on reporting a healthy link indefinitely. The colour here therefore
        follows `live`, which the bridge computes from the FRAME COUNTER
        against a window ending now, and the address is shown beside it so
        "connected but silent" is one glance rather than two topics.
        """
        v = s.get("vrbridge")
        raw = None if v is None else v[0]
        if raw is None:
            self.vr_url_lbl.setText("this machine:  (bridge not started)")
            self.vr_url_lbl.setStyleSheet("color:%s" % C_MUTED)
            self.vr_headset_lbl.setText("headset:  (bridge not started)")
            self.vr_headset_lbl.setStyleSheet("color:%s" % C_MUTED)
            return
        try:
            d = json.loads(raw)
        except Exception:                                     # noqa: BLE001
            self.vr_headset_lbl.setText("headset:  unparsable bridge status")
            self.vr_headset_lbl.setStyleSheet("color:%s" % C_BAD)
            return

        urls = d.get("urls") or []
        if urls:
            self.vr_url_lbl.setText("this machine:  " + "   or   ".join(urls))
            self.vr_url_lbl.setStyleSheet("color:%s" % C_OK)
        else:
            # The bridge in mock mode never serves, so it has no URL and that
            # is correct rather than broken -- say which it is.
            self.vr_url_lbl.setText(
                "this machine:  not serving a page (mock bridge, or no LAN "
                "address)")
            self.vr_url_lbl.setStyleSheet("color:%s" % C_WARN)

        ip = d.get("headset_ip")
        addrs = d.get("client_addrs") or []
        live = bool(d.get("live"))
        hz = d.get("rate_hz")
        if ip:
            extra = ("  +%d more" % (len(addrs) - 1)) if len(addrs) > 1 else ""
            if live:
                self.vr_headset_lbl.setText(
                    "headset:  %s%s   %.0f Hz  LIVE" % (ip, extra, hz or 0.0))
                self.vr_headset_lbl.setStyleSheet("color:%s" % C_OK)
            else:
                age = d.get("age_s")
                self.vr_headset_lbl.setText(
                    "headset:  %s%s   CONNECTED BUT SENDING NOTHING%s"
                    % (ip, extra,
                       "" if age is None else " (%.1f s)" % age))
                self.vr_headset_lbl.setStyleSheet("color:%s" % C_BAD)
        else:
            last = d.get("last_client")
            age = d.get("last_client_age_s")
            if last:
                self.vr_headset_lbl.setText(
                    "headset:  none now - last was %s%s"
                    % (last, "" if age is None else ", %.0f s ago" % age))
            else:
                self.vr_headset_lbl.setText(
                    "headset:  no headset has connected yet")
            self.vr_headset_lbl.setStyleSheet("color:%s" % C_MUTED)

    def _refresh_scene_cv(self, s):
        """One line per camera from the standing vision layer: what it sees
        or, just as importantly, WHY it sees nothing."""
        v = s.get("scenecv")
        raw = None if v is None else v[0]
        if raw is None:
            return                        # keep the "not running" hint
        try:
            d = json.loads(raw)
        except Exception:                                     # noqa: BLE001
            return
        rows, any_obj = [], False
        for name, ent in sorted((d.get("cameras") or {}).items()):
            if ent.get("refusal"):
                # First clause only: the full sentence is in the topic.
                rows.append("%-14s %s" % (name,
                                          ent["refusal"].split(" -- ")[0]))
            else:
                n = len(ent.get("objects") or [])
                any_obj = any_obj or n > 0
                rows.append("%-14s %d object(s)" % (name, n))
        if rows:
            self.vis_scene.setText("\n".join(rows))
            self.vis_scene.setStyleSheet(
                "color:%s" % (C_OK if any_obj else C_MUTED))

    def _refresh_vr_safety(self, s):
        """Why the arm is (not) frozen, in the window, in plain words.

        The safety node published this the whole time; the window never
        read it, so 'frozen on an interlock' and 'broken' looked identical
        from the operator's chair.
        """
        v = s.get("vrsafety")
        raw = None if v is None else v[0]
        if raw is None:
            self.vr_freeze_lbl.setText("")
            return
        try:
            d = json.loads(raw)
        except Exception:                                     # noqa: BLE001
            return
        if d.get("frozen"):
            why = d.get("reason") or "no reason given"
            self.vr_freeze_lbl.setText("FROZEN: %s" % why)
            self.vr_freeze_lbl.setStyleSheet("color:%s" % C_BAD)
        else:
            self.vr_freeze_lbl.setText("not frozen -- VR commands reach "
                                       "the follower")
            self.vr_freeze_lbl.setStyleSheet("color:%s" % C_OK)

    def _refresh_vr_smoothing(self, s):
        """What the smoother is doing, per hand, right now.

        `cutoff_hz` IS the mechanism: near min_cutoff means the filter is
        treating the hand as still and is filtering hard; high means it has
        opened up to track a fast reach. Showing it turns "feels laggy" from
        an opinion into a reading -- and `lag_m` beside it says whether the
        rate limiter is the thing costing distance, which is a different
        fault with a different fix.
        """
        rows = []
        engaged_any = False
        for hand in ("left", "right"):
            v = s.get("vrmap_%s" % hand)
            raw = None if v is None else v[0]
            if raw is None:
                continue
            try:
                d = json.loads(raw)
            except Exception:                                 # noqa: BLE001
                continue
            if not d.get("engaged"):
                rows.append("%-5s idle" % hand)
                continue
            engaged_any = True
            law = d.get("smoothing", "?")
            if law == "one_euro":
                rows.append(
                    "%-5s %.1f Hz cutoff  hand %.2f m/s  lag %.1f mm%s"
                    % (hand, d.get("cutoff_hz") or 0.0,
                       d.get("hand_speed_mps") or 0.0,
                       (d.get("lag_m") or 0.0) * 1000.0,
                       "" if d.get("rot_smoothed") else "  WRIST RAW"))
            else:
                rows.append("%-5s law=%s  lag %.1f mm"
                            % (hand, law, (d.get("lag_m") or 0.0) * 1000.0))
        if not rows:
            self.vr_smooth_state.setText("mapper not running")
            self.vr_smooth_state.setStyleSheet("color:%s" % C_MUTED)
            return
        self.vr_smooth_state.setText("\n".join(rows))
        self.vr_smooth_state.setStyleSheet(
            "color:%s" % (C_OK if engaged_any else C_MUTED))

    def _refresh_cameras(self, s):
        cams = s.get("cam", {})
        imgs = s.get("cam_img", {})
        for a in ARMS:
            cap, st, show, hz = cams.get(a, ("no data", "absent", False, 0.0))
            col = {"live": C_OK, "stale": C_WARN, "dead": C_BAD,
                   "absent": C_UNKNOWN}.get(st, C_UNKNOWN)
            # SAY WHICH TOPIC THE PICTURE CAME FROM. Three names are
            # subscribed and only one is what this repository publishes; on a
            # real Kinova it may be a fourth. "There is a picture" and "the
            # picture came from where I think" are different claims and the
            # panel should not merge them.
            tname = (s.get("cam_topic") or {}).get(a)
            if tname and st in ("live", "stale"):
                cap = "%s   [%s]" % (cap, tname)
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
    # SCROLLING PAST A CONTROL MUST NOT CHANGE WHAT THE ARMS DO.
    #
    # Qt's default is that a combo box, slider or spin box under the pointer
    # eats the wheel and CHANGES ITS VALUE, focused or not. Both of this
    # window's control columns are taller than the screen, so reaching a
    # button means scrolling past a dozen such widgets.
    #
    # Measured on the live window 2026-08-25, by scrolling to reach the real
    # arm panel and nothing else: the experiment `task` silently went from
    # `m1 -- T1 pick and place` to `d3 -- Dance: play`, and `right scale`
    # went from 1.00 to 0.82 -- the motion scale that decides how far real
    # metal travels per unit of operator input. Neither was touched, both
    # persisted, and the next run would have used them.
    #
    # The valueChanged log fires, so this is NOT the "leaves no trace" bug --
    # it is worse. The trace is real and scrolls past in a log the operator
    # is not reading, describing a change they did not make.
    #
    # Focused, the wheel still works: click the control first and it behaves
    # exactly as before. Only the drive-by is refused.
    app.installEventFilter(WheelGuard(app))
    g = Gui(bus, args)
    g.show()
    # PUT IT IN FRONT, AND SAY WHERE IT IS.
    #
    # `show()` maps the window; it does not promise the window manager will
    # put it where you are looking. Launched from a terminal it can come up
    # BEHIND that terminal, and the only evidence the operator has is a log
    # that says everything succeeded -- which reads exactly like "the window
    # is not loading".
    #
    # Raising costs nothing and removes the most likely reason somebody sees
    # no window. The printed line is the other half: if it says the window is
    # open and there is still nothing on screen, that is a different problem
    # and the operator now knows which one.
    g.raise_()
    g.activateWindow()
    # AND SAY WHETHER IT IS ACTUALLY ON THE SCREEN.
    #
    # "behind another window" was the only explanation this line offered, and
    # it was the wrong one: a window sized past the edge of the display is
    # rendered perfectly by Qt, reported IsViewable by X, and never composited
    # onto the desktop by WSLg -- the operator gets a taskbar icon that does
    # nothing when clicked. That cost a session to find, with a screenshot,
    # because nothing printed the one number that would have named it.
    _geo = g.frameGeometry()
    _scr = QGuiApplication.primaryScreen()
    _av = _scr.availableGeometry() if _scr is not None else None
    print("\nThe operations window is open, at %d x %d." % (_geo.width(),
                                                           _geo.height()),
          flush=True)
    if _av is not None and not _av.contains(_geo):
        print("  WARNING: it does NOT fit inside your screen's usable area\n"
              "  (%d x %d at %d,%d). A window past the edge can be drawn\n"
              "  correctly and still never appear -- on WSLg you get a\n"
              "  taskbar icon that will not open. Report this; it is a bug\n"
              "  in the window, not in your machine."
              % (_av.width(), _av.height(), _av.left(), _av.top()),
              flush=True)
    print("If you cannot see it, it is behind another window -- look in your\n"
          "taskbar for\n"
          "  \"SRL operations -- commanded | actual\"\n"
          "RViz takes about 3 more seconds to appear inside it.\n",
          flush=True)

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
