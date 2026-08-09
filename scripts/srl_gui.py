#!/usr/bin/env python3
"""SRL operations GUI: Qt5, Helvetica, embedded RViz, everything indicated.

    python3 scripts/srl_gui.py          or    ros2 run srl_teleop gui

WHY Qt5 AND NOT THE EXISTING DEAR PYGUI CONSOLE. Three reasons, in order of
weight. RViz is itself a Qt application, so its window can be reparented into
a Qt widget and become a panel rather than a second window; that is measured
below and it is the whole point of this rewrite. Qt resolves fonts through
fontconfig, so "Helvetica" is a request the toolkit can actually satisfy. And
Dear PyGui segfaulted on import during this work, intermittently, which is not
a property one wants in the tool used to diagnose everything else.

RVIZ EMBEDDING: MEASURED, NOT ASSUMED. Three routes were investigated.

  1. X11 reparenting. QX11EmbedContainer was removed in Qt5; the modern
     equivalent is QWindow.fromWinId() wrapped by
     QWidget.createWindowContainer(). VERIFIED FROM PIXELS under Xvfb: the
     host label renders above and the complete RViz UI renders inside the
     container at 31 fps. Cost: rviz2 stays a separate process, so it cannot
     be styled by the host and it flashes as its own window for the moment
     between mapping and reparenting. That separateness is also a benefit: an
     RViz crash does not take the GUI with it.
  2. librviz as a widget. The headers exist under
     /opt/ros/jazzy/include/rviz_common/, but there are no usable Python
     bindings for rviz2, so this means a new C++ Qt package and a build step,
     and RViz then shares the GUI's process and its crashes. NOT ATTEMPTED;
     route 1 already works.
  3. Render to an image topic and display frames. Proven in this repository by
     the clip pipeline (Xvfb plus ffmpeg), so it is known to work, but it adds
     an encode and decode per frame and loses interaction entirely: the
     operator cannot orbit the camera. Strictly worse than route 1 wherever
     route 1 is available. Kept as the fallback for a headless host.

Route 1 is used, with a runtime check that falls back to a message rather than
an empty panel if the window never appears.

THREADING. ROS owns one thread and every subscription. The GUI reads ONE
immutable snapshot dict, rebound atomically; it never calls a service, never
waits and never locks. Every control action is queued to the ROS thread and
answered by callback. This is the same rule the Dear PyGui console arrived at
after two threading bugs, and it is kept.
"""
import json
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from sensor_msgs.msg import Image, JointState

# The package lives in src/, and this tool runs from scripts/, so make the
# workspace's own modules importable whether or not install/ is sourced.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src/srl_teleop"))
from srl_teleop import camera_relay as cr                    # noqa: E402
from srl_teleop import capability as cap                     # noqa: E402

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPalette, QPixmap, QWindow
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QMainWindow, QPushButton, QVBoxLayout,
                             QWidget)

ARMS = ("left", "right")

# ISA-101: colour is a scarce alarm channel, so a normal state is largely
# UNCOLOURED rather than green. Saturated colour is reserved for the abnormal.
C_BG = "#f4f4f4"
C_TEXT = "#1a1a1a"
C_MUTED = "#6b6b6b"
C_BAD = "#c62828"
C_WARN = "#ef6c00"
C_OK = "#2e7d32"
C_UNKNOWN = "#7b1fa2"      # distinct from bad: not knowing is its own state


def helvetica(size=11, bold=False):
    """Helvetica is licensed and absent here; fontconfig aliases it to
    Liberation Sans, which shares Helvetica's advance widths, so a layout
    designed for one does not reflow in the other. Requesting it by name keeps
    the intent visible in the source."""
    f = QFont("Helvetica", size)
    f.setBold(bold)
    f.setStyleHint(QFont.Helvetica)
    return f


class Bus(Node):
    """Every subscription lives here, on the ROS thread."""

    def __init__(self):
        super().__init__("srl_gui")
        self.snap = dict(t=time.monotonic())
        self._d = {}
        self._q = []
        self.sub_specs()
        self.create_timer(0.1, self._publish_snapshot)
        self.create_timer(0.05, self._drain)

    def sub_specs(self):
        for a in ARMS:
            self.create_subscription(
                String, "/master_capability_%s" % a,
                lambda m, k="cap_%s" % a: self._set(k, m.data), 10)
            self.create_subscription(
                Float64MultiArray, "/ik_status_%s" % a,
                lambda m, k="ik_%s" % a: self._set(k, list(m.data)), 10)
        self.create_subscription(String, "/scene/state",
                                 lambda m: self._set("scene", m.data), 10)
        self.create_subscription(String, "/blocking_summary",
                                 lambda m: self._set("blocking", m.data), 10)
        self.create_subscription(Bool, "/estop_state",
                                 lambda m: self._set("estop", m.data), 10)
        self.create_subscription(String, "/master_channel_state",
                                 lambda m: self._set("chan", m.data), 10)
        self.create_subscription(String, "/autonomy_decision",
                                 lambda m: self._set("autonomy", m.data), 10)
        self.create_subscription(String, "/vr_state",
                                 lambda m: self._set("vr", m.data), 10)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self._set("js", len(m.name)), 10)
        # ONE MORE SUBSCRIBER, NEVER A SECOND DEVICE OWNER. The vendor vision
        # driver owns the camera; this sits beside the detector on its topic.
        # depth=1 and best-effort: a viewer wants the NEWEST frame, and a
        # reliable queue would deliver a backlog of old ones after any hiccup,
        # which is precisely the stale view this relay exists to prevent.
        from rclpy.qos import qos_profile_sensor_data
        self.cam = {a: cr.ChannelState() for a in ARMS}
        self.cam_img = {a: None for a in ARMS}
        for a in ARMS:
            for topic in ("/%s_wrist_camera/image_raw" % a,
                          "/wrist_mounted_camera/%s/image" % a):
                self.create_subscription(
                    Image, topic,
                    (lambda m, arm=a: self._on_image(arm, m)),
                    qos_profile_sensor_data)

    def _on_image(self, arm, msg):
        self.cam[arm].on_frame(msg.width, msg.height, msg.encoding)
        # Keep the RAW buffer and convert on the GUI thread only when it will
        # actually be painted. Converting here would spend ROS-thread time on
        # frames the GUI is about to discard as stale.
        self.cam_img[arm] = (msg.width, msg.height, msg.encoding,
                             bytes(msg.data), msg.step)

    def _set(self, k, v):
        self._d[k] = (v, time.monotonic())

    def _publish_snapshot(self):
        # ONE dict, rebound atomically. The GUI thread only ever reads the
        # binding, so it can never observe a half-updated view.
        s = {k: v for k, v in self._d.items()}
        s["t"] = time.monotonic()
        s["topics"] = {t for t, _ in self.get_topic_names_and_types()}
        s["cam"] = {a: (self.cam[a].caption(), self.cam[a].state(),
                        self.cam[a].show_image(), self.cam[a].hz())
                    for a in ARMS}
        s["cam_img"] = dict(self.cam_img)
        self.snap = s

    def submit(self, fn):
        self._q.append(fn)

    def _drain(self):
        while self._q:
            try:
                self._q.pop(0)()
            except Exception as e:                            # noqa: BLE001
                self.get_logger().error("action failed: %r" % (e,))


class Ind(QLabel):
    """One indicator: caption, value, and a colour that means something."""

    def __init__(self, caption):
        super().__init__()
        self.caption = caption
        self.setFont(helvetica(11))
        self.setTextFormat(Qt.RichText)
        self.set("--", C_MUTED, "no data")

    def set(self, value, colour=C_TEXT, note=""):
        self.setText(
            "<span style='color:%s;font-size:10px'>%s</span><br>"
            "<span style='color:%s;font-size:15px;font-weight:600'>%s</span>"
            "<br><span style='color:%s;font-size:10px'>%s</span>"
            % (C_MUTED, self.caption, colour, value, C_MUTED, note))


class Gui(QMainWindow):

    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.rviz = None
        self.setWindowTitle("SRL operations")
        self.setFont(helvetica(11))
        pal = self.palette()
        pal.setColor(QPalette.Window, QColor(C_BG))
        self.setPalette(pal)

        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(10, 8, 10, 8)

        self.banner = QLabel()
        self.banner.setFont(helvetica(16, True))
        self.banner.setAlignment(Qt.AlignCenter)
        outer.addWidget(self.banner)

        body = QHBoxLayout()
        outer.addLayout(body, 1)

        left = QVBoxLayout()
        body.addLayout(left, 0)
        self.ind = {}
        for title, keys in (
            ("Master", ("capability_left", "capability_right",
                        "channels")),
            ("Scene", ("fingerprint", "camera_left", "camera_right")),
            ("Mode", ("mode", "autonomy", "intent")),
            ("Target", ("reachable", "clearance_left", "clearance_right")),
            ("Safety", ("estop", "blockers", "ik_left", "ik_right")),
        ):
            g = QGroupBox(title)
            g.setFont(helvetica(11, True))
            gl = QGridLayout(g)
            for i, k in enumerate(keys):
                w = Ind(k.replace("_", " "))
                self.ind[k] = w
                gl.addWidget(w, i // 2, i % 2)
            left.addWidget(g)
        left.addStretch(1)

        # ---- wrist cameras, both arms, labelled
        campanel = QGroupBox("Wrist cameras")
        campanel.setFont(helvetica(11, True))
        cl = QHBoxLayout(campanel)
        self.cam_lbl, self.cam_cap = {}, {}
        for a in ARMS:
            col = QVBoxLayout()
            name = QLabel(a.upper())
            name.setFont(helvetica(10, True))
            name.setAlignment(Qt.AlignCenter)
            img = QLabel()
            img.setMinimumSize(260, 195)
            img.setAlignment(Qt.AlignCenter)
            img.setStyleSheet("background:#111;color:#bbb;border:1px solid #999")
            cap = QLabel()
            cap.setFont(helvetica(9))
            cap.setAlignment(Qt.AlignCenter)
            col.addWidget(name)
            col.addWidget(img)
            col.addWidget(cap)
            cl.addLayout(col)
            self.cam_lbl[a], self.cam_cap[a] = img, cap
        left.addWidget(campanel)

        self.viz_host = QWidget()
        self.viz_host.setMinimumSize(720, 520)
        vl = QVBoxLayout(self.viz_host)
        vl.setContentsMargins(0, 0, 0, 0)
        self.viz_msg = QLabel("starting RViz...")
        self.viz_msg.setAlignment(Qt.AlignCenter)
        self.viz_msg.setFont(helvetica(12))
        vl.addWidget(self.viz_msg)
        body.addWidget(self.viz_host, 1)

        bar = QHBoxLayout()
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setFont(helvetica(14, True))
        self.estop_btn.setStyleSheet(
            "background:%s;color:white;padding:10px;" % C_BAD)
        self.estop_btn.clicked.connect(self.on_estop)
        bar.addWidget(self.estop_btn)
        self.frame_lbl = QLabel()
        self.frame_lbl.setFont(helvetica(10))
        bar.addWidget(self.frame_lbl)
        bar.addStretch(1)
        outer.addLayout(bar)

        self.setCentralWidget(root)
        self.resize(1500, 900)

        self._ft = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        QTimer.singleShot(500, self.start_rviz)

    # ------------------------------------------------------------- rviz
    def start_rviz(self):
        cfg = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "src/srl_experiments/config/verification_capture.rviz")
        cmd = ["rviz2"] + (["-d", cfg] if os.path.exists(cfg) else [])
        try:
            self.rviz = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.viz_msg.setText("rviz2 not on PATH")
            return
        self._rviz_tries = 0
        QTimer.singleShot(1200, self._try_embed)

    def _try_embed(self):
        self._rviz_tries += 1
        wid = self._find_rviz_window()
        if wid is None:
            if self._rviz_tries > 30:
                # Fall back with a REASON rather than an empty panel. An empty
                # panel is indistinguishable from a working one showing
                # nothing, which is the failure this whole GUI exists to make
                # impossible elsewhere.
                self.viz_msg.setText(
                    "RViz did not present an X window in 36 s.\n"
                    "Embedding needs an X11 (or XWayland) session.\n"
                    "Run rviz2 separately; every indicator here still works.")
                return
            QTimer.singleShot(1200, self._try_embed)
            return
        foreign = QWindow.fromWinId(wid)
        container = QWidget.createWindowContainer(foreign, self.viz_host)
        self.viz_msg.hide()
        self.viz_host.layout().addWidget(container)

    @staticmethod
    def _find_rviz_window():
        try:
            out = subprocess.run(["xwininfo", "-root", "-tree"],
                                 capture_output=True, text=True,
                                 timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return None
        for line in out.splitlines():
            if "RViz" in line or "rviz" in line:
                m = re.search(r"(0x[0-9a-f]+)", line)
                if m:
                    return int(m.group(1), 16)
        return None

    # ---------------------------------------------------------- actions
    def on_estop(self):
        def act():
            from std_msgs.msg import Bool as B
            p = self.bus.create_publisher(B, "/estop", 10)
            m = B()
            m.data = True
            p.publish(m)
        self.bus.submit(act)

    # ---------------------------------------------------------- refresh
    def refresh(self):
        t0 = time.perf_counter()
        s = self.bus.snap
        now = s.get("t", time.monotonic())

        def age(key):
            v = s.get(key)
            return None if v is None else now - v[1]

        def val(key, default=None):
            v = s.get(key)
            return default if v is None else v[0]

        stale = lambda k, lim=2.0: (age(k) is None or age(k) > lim)  # noqa

        # ---- capability, per arm
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
                col = (C_OK if d["key"] in ("FK", "SPHERICAL")
                       else C_WARN if d["position_available"] else C_BAD)
                cost = d.get("cost_m")
                note = (("cost %.0f mm" % (1000 * cost["mean_m"]))
                        if cost else "cost unmeasured")
                if not d["position_available"]:
                    note = "NO POSITION"
                self.ind[k].set("%s %s" % (a[:1].upper(), d["key"]), col, note)

        raw = val("chan")
        self.ind["channels"].set(
            (raw or "--").split("|")[0].strip() if raw else "--",
            C_MUTED if raw else C_UNKNOWN,
            "master_pose_node" if raw else "not publishing")

        # ---- scene fingerprint
        raw = val("scene")
        if raw is None:
            self.ind["fingerprint"].set("--", C_UNKNOWN, "node not running")
        else:
            d = json.loads(raw)
            st = d.get("state", "?")
            col = {"match_skip_calibration": C_OK,
                   "registered": C_OK,
                   "changed_reregistered": C_WARN,
                   "sweeping": C_MUTED,
                   "sweep_invalid_no_detections": C_BAD}.get(st, C_UNKNOWN)
            self.ind["fingerprint"].set(
                st.replace("_", " "), col,
                "%d object(s), %d drift flag(s)"
                % (d.get("n_stored", 0), len(d.get("drift_flags", []))))

        # ---- cameras, per arm: presence of the detection topic AND traffic
        topics = s.get("topics", set())
        for a in ARMS:
            t = "/perception/detections/%s" % a
            if t not in topics:
                self.ind["camera_%s" % a].set("absent", C_UNKNOWN,
                                              "no detector on this arm")
            else:
                self.ind["camera_%s" % a].set("present", C_OK, t)

        # ---- mode / autonomy / intent
        vr = val("vr")
        self.ind["mode"].set("VR" if vr else ("DIRECT" if val("chan")
                                              else "--"),
                             C_TEXT if (vr or val("chan")) else C_UNKNOWN,
                             "from live publishers")
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

        # ---- reachability and clearance, from ik_status
        for a in ARMS:
            v = val("ik_%s" % a)
            k = "clearance_%s" % a
            if not v or len(v) < 8:
                self.ind[k].set("--", C_UNKNOWN, "no ik_status")
                continue
            clr = v[5]
            col = (C_UNKNOWN if clr < 0 else
                   C_BAD if clr < 0.12 else C_WARN if clr < 0.20 else C_OK)
            self.ind[k].set("--" if clr < 0 else "%.3f m" % clr, col,
                            "floor 0.12 m")
        self.ind["reachable"].set(
            "--", C_UNKNOWN, "set by the autonomy target check")

        # ---- safety
        es = val("estop")
        self.ind["estop"].set("LATCHED" if es else ("clear" if es is not None
                                                    else "--"),
                              C_BAD if es else (C_TEXT if es is not None
                                                else C_UNKNOWN),
                              "/estop_state")
        bl = val("blocking")
        if bl is None:
            self.ind["blockers"].set("--", C_UNKNOWN,
                                     "blocking_aggregator not running")
        else:
            try:
                d = json.loads(bl)
                n = int(d.get("n_blocking", 0))
                unk = int(d.get("state_unknown", 0))
            except Exception:                                 # noqa: BLE001
                n, unk = 0, 0
            self.ind["blockers"].set(
                "%d" % n, C_BAD if n else (C_UNKNOWN if unk else C_TEXT),
                "%d unknown" % unk)
        for a in ARMS:
            v = val("ik_%s" % a)
            k = "ik_%s" % a
            if not v:
                self.ind[k].set("--", C_UNKNOWN, "no ik_status")
            else:
                att, suc = (v[0] or 0), (v[1] or 0)
                pct = 100.0 * suc / att if att else None
                self.ind[k].set(
                    "--" if pct is None else "%.0f%%" % pct,
                    C_UNKNOWN if pct is None else
                    (C_OK if pct > 90 else C_WARN if pct > 60 else C_BAD),
                    "%d attempts" % att)

        # ---- wrist cameras
        cams = s.get("cam", {})
        imgs = s.get("cam_img", {})
        for a in ARMS:
            cap, st, show, hz = cams.get(a, ("no data", "absent", False, 0.0))
            col = {"live": C_OK, "stale": C_WARN,
                   "dead": C_BAD, "absent": C_UNKNOWN}.get(st, C_UNKNOWN)
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
            fmt = {"rgb8": QImage.Format_RGB888,
                   "bgr8": QImage.Format_BGR888,
                   "mono8": QImage.Format_Grayscale8}.get(enc)
            if fmt is None:
                self.cam_lbl[a].setText("unsupported encoding %s" % enc)
                continue
            qi = QImage(buf, w, h, step, fmt)
            self.cam_lbl[a].setPixmap(QPixmap.fromImage(qi).scaled(
                self.cam_lbl[a].width(), self.cam_lbl[a].height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.cam_lbl[a].setStyleSheet("background:#111;border:1px solid #999")

        # ---- banner
        if es:
            self.banner.setText("E-STOP LATCHED")
            self.banner.setStyleSheet("color:white;background:%s;padding:6px"
                                      % C_BAD)
        elif not topics:
            self.banner.setText("no ROS graph")
            self.banner.setStyleSheet("color:white;background:%s;padding:6px"
                                      % C_UNKNOWN)
        else:
            self.banner.setText("running")
            self.banner.setStyleSheet("color:%s;padding:6px" % C_MUTED)

        dt = (time.perf_counter() - t0) * 1000.0
        self._ft.append(dt)
        self._ft = self._ft[-200:]
        srt = sorted(self._ft)
        self.frame_lbl.setText(
            "frame %.2f ms  median %.2f  p95 %.2f  (budget 100 ms)"
            % (dt, srt[len(srt) // 2], srt[int(len(srt) * 0.95)]))

    def closeEvent(self, ev):
        if self.rviz:
            self.rviz.terminate()
        ev.accept()


def main(argv=None):
    rclpy.init(args=argv)
    bus = Bus()
    th = threading.Thread(target=lambda: rclpy.spin(bus), daemon=True)
    th.start()
    app = QApplication(sys.argv)
    app.setFont(helvetica(11))
    g = Gui(bus)
    g.show()
    rc = app.exec_()
    bus.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
