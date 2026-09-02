#!/usr/bin/env python3
"""
master_mannequin_gui.py -- MASTER MANNEQUIN GUI. One window, one button.

    python3 scripts/master_mannequin_gui.py

A SEPARATE WINDOW FROM scripts/srl_gui.py, WHICH IS NOT TOUCHED. That window
is the whole rig -- VR, autonomy, cameras, participants -- and it is what
every other procedure here refers to. This one does exactly one job: drive the
REAL arms from the master mannequin, the way that worked on 2026-09-01. Adding
a seventh way to start the real arms to srl_gui.py would have been the thing
its own comments warn about; a separate window with a single button is not.

WHAT THE BUTTON DOES. Nothing this file invents: the ordered sequence lives in
`srl_teleop.master_bringup.STEPS` and is executed by the runner below, so the
list you read on screen IS the list that runs. Six steps, each of which was
learned the expensive way -- read master_bringup's docstring for which failure
each one prevents.

WHY A BUTTON AT ALL. The sequence is six commands across three terminals with
two ROS service calls in the middle, and getting step 4 (seed the sim from the
real arms) out of order makes the bridge refuse with a number that reads like
a hardware fault. That is not something to retype at 2 a.m.

SPEED is a control here because "the arms move slow" was the first thing said
after it worked. The presets are in `master_bringup.SPEEDS` and they enforce
the one invariant that matters -- the commanding side stays slower than the
following side, or the lag monitor trips.

NOTHING MOVES UNTIL THE LAST STEP. `motion_enabled` is false through steps
1-5; the arms are homed and the bridges enabled while the teleop is still
computing into the void, and only step 6 lets it drive. STOP is always live
and calls the same /estop the rest of the rig uses.
"""
import os
import subprocess
import sys
import time
import threading

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))

from PyQt5.QtCore import Qt, QTimer, pyqtSignal                  # noqa: E402
from PyQt5.QtGui import QFont                                    # noqa: E402
from PyQt5.QtWidgets import (QApplication, QComboBox,            # noqa: E402
                             QHBoxLayout, QLabel, QLineEdit,
                             QPlainTextEdit, QPushButton,
                             QVBoxLayout, QWidget)

from srl_teleop import master_bringup as mb                      # noqa: E402

C_OK = "#1b7a34"
C_STOP = "#a11414"
C_IDLE = "#333a44"
C_WARN = "#8a6d1a"


def _scratch():
    d = os.environ.get("SRL_SCRATCH") or "/tmp"
    p = os.path.join(d, "master_mannequin_gui")
    os.makedirs(p, exist_ok=True)
    return p


class MasterMannequinWindow(QWidget):

    #: Worker threads cannot touch widgets. Everything the runner reports
    #: comes back through these, which is the only reason the window stays
    #: responsive while a 5-minute bring-up runs.
    sig_log = pyqtSignal(str)
    sig_state = pyqtSignal(str, str)          # (text, colour)
    sig_step = pyqtSignal(int, str)           # (index, verdict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MASTER MANNEQUIN GUI -- real arms from the master arm")
        self.resize(880, 640)
        self._procs = {}
        self._rec_dir = None
        self._running = False
        self._abort = False

        v = QVBoxLayout(self)
        v.setSpacing(8)

        title = QLabel("MASTER MANNEQUIN")
        f = QFont(); f.setPointSize(17); f.setBold(True)
        title.setFont(f)
        v.addWidget(title)

        sub = QLabel("Drives the REAL Kinova arms from the master mannequin "
                     "arm. Separate from the main SRL window, which is "
                     "unchanged.")
        sub.setWordWrap(True)
        sub.setStyleSheet("color:#99a;")
        v.addWidget(sub)

        # ------------------------------------------------------ THE BUTTON
        self.go = QPushButton("WORKING REAL ARM MASTER")
        gf = QFont(); gf.setPointSize(15); gf.setBold(True)
        self.go.setFont(gf)
        self.go.setMinimumHeight(74)
        self.go.setStyleSheet(
            "QPushButton{background:%s;color:white;border-radius:6px;}"
            "QPushButton:disabled{background:%s;color:#bbb;}" % (C_OK, C_IDLE))
        self.go.setToolTip(
            "Runs the six-step bring-up that worked on 2026-09-01: sim "
            "stack, master arm with the clutch pinned, real Kortex session "
            "with homing, seed the sim from the real arms, enable both "
            "bridges, then arm. Nothing moves until the last step.")
        self.go.clicked.connect(self.on_go)
        v.addWidget(self.go)

        row = QHBoxLayout()
        row.addWidget(QLabel("speed:"))
        self.speed = QComboBox()
        for name in ("slow", "normal", "fast"):
            p = mb.SPEEDS[name]
            self.speed.addItem(
                "%s  (teleop %.2f / bridge %.2f rad/s, slew %.0f mm)"
                % (name, p["teleop_vmax"], p["bridge_vmax"],
                   p["slew_m"] * 1000), name)
        self.speed.setCurrentIndex(
            [self.speed.itemData(i)
             for i in range(self.speed.count())].index(mb.DEFAULT_SPEED))
        self.speed.setToolTip(
            "The commanding side is always kept slower than the following "
            "side. Ruckig at the joint limit against a 0.40 rad/s bridge is "
            "what tripped the lag monitor at 0.502 rad on 2026-09-01.")
        row.addWidget(self.speed, 1)

        self.stop = QPushButton("STOP  (e-stop)")
        self.stop.setMinimumHeight(38)
        self.stop.setStyleSheet(
            "QPushButton{background:%s;color:white;font-weight:bold;"
            "border-radius:5px;}" % C_STOP)
        self.stop.setToolTip("Latches /estop. The same e-stop the rest of "
                             "the rig uses; clear it with RESET E-STOP.")
        self.stop.clicked.connect(self.on_stop)
        row.addWidget(self.stop)

        self.reset = QPushButton("RESET E-STOP")
        self.reset.setMinimumHeight(38)
        self.reset.clicked.connect(self.on_reset)
        row.addWidget(self.reset)
        v.addLayout(row)

        # ---------------------------------------------------- ARM IPs
        ip_row = QHBoxLayout()
        ip_row.addWidget(QLabel("left arm IP:"))
        self.left_ip = QLineEdit(mb.DEFAULT_LEFT_IP)
        self.left_ip.setMaximumWidth(150)
        ip_row.addWidget(self.left_ip)
        ip_row.addWidget(QLabel("right arm IP:"))
        self.right_ip = QLineEdit(mb.DEFAULT_RIGHT_IP)
        self.right_ip.setMaximumWidth(150)
        ip_row.addWidget(self.right_ip)

        self.ping_btn = QPushButton("CHECK ARMS")
        self.ping_btn.setToolTip(
            "Pings both addresses. A reachable arm is not a connected arm, "
            "but an unreachable one cannot be connected at all, and this "
            "says which before a five-minute bring-up finds out.")
        self.ping_btn.clicked.connect(self.on_ping)
        ip_row.addWidget(self.ping_btn)

        self.connect_btn = QPushButton("CONNECT REAL ARMS")
        self.connect_btn.setToolTip(
            "Steps 3-6 only: Kortex session, homing, seed, bridges, arm. "
            "Use when the sim stack and master arm are already up and only "
            "the real side dropped -- it does not restart what is working.")
        self.connect_btn.clicked.connect(self.on_connect_only)
        ip_row.addWidget(self.connect_btn)
        ip_row.addStretch(1)
        v.addLayout(ip_row)

        # ------------------------------------------------- extra controls
        x_row = QHBoxLayout()
        self.rec_btn = QPushButton("START RECORDING")
        self.rec_btn.setToolTip(
            "full_state_recorder: every channel this session can see, to "
            "recordings/. Press again to stop and close the file.")
        self.rec_btn.clicked.connect(self.on_record)
        x_row.addWidget(self.rec_btn)

        self.shared_btn = QPushButton("SHARED AUTONOMY")
        self.shared_btn.setToolTip(
            "Starts handover_arbiter on top of the running teleop: autonomy "
            "and the operator share the pose. Teleop must already be "
            "following.")
        self.shared_btn.clicked.connect(self.on_shared)
        x_row.addWidget(self.shared_btn)

        self.stopall_btn = QPushButton("STOP EVERYTHING")
        self.stopall_btn.setToolTip(
            "SIGINT to the whole stack, in the order that closes the Kortex "
            "session cleanly. A killed session leaks and the next connect is "
            "refused.")
        self.stopall_btn.clicked.connect(self.on_stop_all)
        x_row.addWidget(self.stopall_btn)
        x_row.addStretch(1)
        v.addLayout(x_row)

        self.state = QLabel("idle")
        self.state.setStyleSheet(
            "background:%s;color:white;padding:6px;border-radius:4px;" % C_IDLE)
        v.addWidget(self.state)

        # ------------------------------------------------------- THE STEPS
        # Rendered FROM master_bringup.STEPS, so the window cannot show a
        # sequence different from the one that runs.
        self.step_labels = []
        for i, (_k, title_, why) in enumerate(mb.STEPS):
            lab = QLabel("%d. %s -- %s" % (i + 1, title_, why))
            lab.setWordWrap(True)
            lab.setStyleSheet("color:#889;padding-left:6px;")
            v.addWidget(lab)
            self.step_labels.append(lab)

        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setStyleSheet(
            "background:#11141a;color:#cfd6e4;font-family:monospace;")
        v.addWidget(self.out, 1)

        self.sig_log.connect(self._append)
        self.sig_state.connect(self._set_state)
        self.sig_step.connect(self._set_step)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._poll_status)
        self.status_timer.start(2000)
        _ch, _why = mb.ensure_env()
        self.log(("adopted the workspace environment -- " + _why) if _ch
                 else "environment OK (scripts/env.sh settings present)")
        self._status_thread = None

    # ----------------------------------------------------------- UI slots
    def _append(self, s):
        self.out.appendPlainText(s)
        self.out.verticalScrollBar().setValue(
            self.out.verticalScrollBar().maximum())

    def _set_state(self, text, colour):
        self.state.setText(text)
        self.state.setStyleSheet(
            "background:%s;color:white;padding:6px;border-radius:4px;"
            % colour)

    def _set_step(self, idx, verdict):
        if 0 <= idx < len(self.step_labels):
            lab = self.step_labels[idx]
            base = lab.text().split("   [")[0]
            lab.setText("%s   [%s]" % (base, verdict))
            col = {"running": "#d8b13a", "ok": "#54c46e",
                   "FAILED": "#e2554f"}.get(verdict, "#889")
            lab.setStyleSheet("color:%s;padding-left:6px;" % col)

    def log(self, s):
        self.sig_log.emit("%s  %s" % (time.strftime("%H:%M:%S"), s))

    # ------------------------------------------------------------ actions
    def on_go(self):
        if self._running:
            return
        self._running = True
        self._abort = False
        self.go.setEnabled(False)
        self.go.setText("BRINGING UP ...")
        for i in range(len(self.step_labels)):
            self.sig_step.emit(i, "")
        threading.Thread(target=self._run, daemon=True).start()

    def on_stop(self):
        threading.Thread(target=self._stop_worker, daemon=True).start()

    def _stop_worker(self):
        self.log("STOP pressed -- latching /estop")
        ok, why = mb.estop()
        self.log(why)
        if ok:
            self.sig_state.emit("E-STOP LATCHED", C_STOP)

    def on_reset(self):
        threading.Thread(target=self._reset_worker, daemon=True).start()

    def _reset_worker(self):
        ok, why = mb.reset_estop()
        self.log("e-stop reset: %s" % (why or ok))

    def on_ping(self):
        threading.Thread(target=self._ping_worker, daemon=True).start()

    def _ping_worker(self):
        for side, box in (("left", self.left_ip), ("right", self.right_ip)):
            ip = box.text().strip()
            ok = mb.ping(ip)
            self.log("%s arm %s: %s" % (side, ip,
                                        "reachable" if ok else "NO REPLY"))

    def on_connect_only(self):
        """Steps 3-6 without restarting a stack that is already working.

        The full button tears down and rebuilds; when only the real side has
        dropped that costs five minutes and risks the parts that are fine.
        """
        if self._running:
            return
        self._running = True
        self.connect_btn.setEnabled(False)
        threading.Thread(target=self._run, kwargs={"from_step": 2},
                         daemon=True).start()

    def on_record(self):
        if self._procs.get("record") and self._procs["record"].poll() is None:
            # SIGINT, not terminate: ros2 bag closes and INDEXES the file on
            # SIGINT. Killed harder it leaves an unindexed bag behind.
            self._procs["record"].send_signal(2)
            try:
                self._procs["record"].wait(timeout=15)
            except Exception:                                # noqa: BLE001
                pass
            out = self._rec_dir or "?"
            self._procs.pop("record", None)
            self.rec_btn.setText("START RECORDING")
            self.log("recording STOPPED and closed -> %s" % out)
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._rec_dir = os.path.join(
            WS, "recordings", "sessions", "master_teleop_%s" % stamp)
        log = os.path.join(_scratch(), "recording.log")
        # CHECK BEFORE RECORDING, not after. A bag records nothing for a
        # topic that does not exist and barely mentions it, so a session can
        # come back missing the real arm entirely and look fine.
        present, missing = mb.check_record_topics()
        if missing:
            self.log("WARNING: %d of %d topics are NOT on the graph and will "
                     "be EMPTY in this bag:" % (len(missing),
                                                len(mb.RECORD_TOPICS)))
            for t in missing:
                self.log("    %s" % t)
            self.log("  (record anyway -- but bring the missing side up "
                     "first if you need it)")
        elif present:
            self.log("all %d topics present on the graph" % len(present))
        self._procs["record"] = mb.spawn(mb.record_command(self._rec_dir), log)
        self.rec_btn.setText("STOP RECORDING")
        self.log("recording %d topics -> %s"
                 % (len(mb.RECORD_TOPICS), self._rec_dir))

    def on_shared(self):
        """Shared autonomy ON TOP of a running teleop, never instead of it."""
        if self._procs.get("shared") and self._procs["shared"].poll() is None:
            self._procs["shared"].terminate()
            self._procs.pop("shared", None)
            self.shared_btn.setText("SHARED AUTONOMY")
            self.log("shared autonomy stopped -- operator has the pose")
            return
        log = os.path.join(_scratch(), "shared.log")
        self._procs["shared"] = mb.spawn(
            ["ros2", "run", "srl_autonomy", "handover_arbiter"], log)
        self.shared_btn.setText("STOP SHARED AUTONOMY")
        self.log("shared autonomy started (handover_arbiter) -> %s" % log)

    def on_stop_all(self):
        threading.Thread(target=self._stop_all_worker, daemon=True).start()

    def _stop_all_worker(self):
        # ORDER MATTERS: the Kortex session first and with SIGINT, because a
        # killed session leaks and the arm refuses the next connect.
        self.log("stopping everything (Kortex session first, SIGINT)")
        for pat in ("[s]tart_real.sh", "[k]ortex_highlevel_bridge",
                    "[r]os2 launch srl_teleop", "[s]rl_teleop/lib",
                    "controller_manager/[r]os2_control_node",
                    "[m]oveit_ros_move_group"):
            subprocess.run(["bash", "-lc",
                            "for P in $(pgrep -f '%s'); do kill -INT $P; done"
                            % pat], capture_output=True)
            time.sleep(1.5)
        self.log("stopped. Segments left: %d" % mb.stale_shm_count())
        self.sig_state.emit("stopped", C_IDLE)

    # --------------------------------------------------------- the runner
    def _run(self, from_step=0):
        try:
            prof = mb.speed_profile(self.speed.currentData())
        except ValueError as exc:
            self.log("REFUSED: %s" % exc)
            self._done(False)
            return
        self.log("speed: teleop %.2f rad/s < bridge %.2f rad/s, slew %.0f mm"
                 % (prof["teleop_vmax"], prof["bridge_vmax"],
                    prof["slew_m"] * 1000))
        sp = _scratch()
        env = dict(os.environ)
        env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
        env["MAX_VEL"] = "%.3f" % prof["bridge_vmax"]

        fixer = mb.AutoFixer(self.log)
        try:
            # 0 ------------------------------------------- PREFLIGHT FIXES
            # BEFORE the stack, because two of these are only safe with it
            # down: the /dev/shm sweep orphans a live stack's own segments,
            # and the Teensy power-cycle needs the serial port free.
            self.log("preflight auto-fixes")
            fixer.run(keys=("daemon",), only_if_needed=False)
            # dup_stack FIRST: a second stack partitions discovery, and the
            # SHM sweep is gated on nothing running, so the order matters.
            fixer.run(keys=("dup_stack",))
            fixer.run(keys=("shm", "teensy_absent", "teensy_imu",
                            "kortex_leak"))
            if mb.teensy_port() is None:
                raise RuntimeError(
                    "no master arm on /dev/ttyACM* and it could not be "
                    "attached -- check the Teensy is plugged in")

            # 1 -------------------------------------------------- stack
            if from_step > 0:
                self.log("skipping steps 1-%d (already up)" % from_step)
            if from_step <= 0:
                self._step(0, "simulation stack")
                t1 = os.path.join(sp, "stack.log")
                self._procs["stack"] = mb.spawn(
                    ["ros2", "launch", "srl_teleop", "teleop.launch.py",
                     "gate:=false", "follower:=master", "master:=false",
                     # THE SPEED HAS TO GO IN HERE, not be set afterwards:
                     # master_teleop_node reads both at construction.
                     "master_vmax_rad_s:=%.3f" % prof["teleop_vmax"],
                     "master_max_step_m:=%.4f" % prof["slew_m"]],
                    t1, env)
                if not mb.wait_for(
                        lambda: mb.log_says(t1, "Successful 'activate' of "
                                                "hardware 'left_Kortex"), 180):
                    raise RuntimeError(
                        "controllers did not activate; see %s" % t1)
                self._ok(0)

            # 2 ------------------------------------------------- master
            if from_step <= 1:
                self._step(1, "master arm, clutch pinned")
                m1 = os.path.join(sp, "master.log")
                self._procs["master"] = mb.spawn(
                    ["ros2", "run", "srl_teleop", "master_pose_node",
                     "--ros-args", "-r", "__node:=master_pose_node",
                     "-p", "force_clutch_engaged:=true"], m1, env)
                if not mb.wait_for(
                        lambda: mb.log_says(m1, "ENGAGED (FORCED"), 60):
                    raise RuntimeError(
                        "master node did not pin the clutch; see %s" % m1)
                self._ok(1)

            # 3 --------------------------------------------------- real
            # THE DAEMON MUST BE RESET AGAIN HERE, after the stack exists.
            # It caches the graph it saw when it started, so the reset at
            # preflight leaves it holding an EMPTY graph, and start_real.sh
            # then refuses with "DISCOVERY PROBLEM, not a missing stack" --
            # which is exactly what it did on the first run of this GUI.
            self.log("  resetting the ros2 daemon now the stack exists")
            mb.reset_daemon()
            self._step(2, "real arms (Kortex session + homing)")
            r1 = os.path.join(sp, "real.log")
            renv = dict(env)
            renv["LEFT_IP"] = self.left_ip.text().strip() or mb.DEFAULT_LEFT_IP
            renv["RIGHT_IP"] = (self.right_ip.text().strip()
                                or mb.DEFAULT_RIGHT_IP)
            self.log("  arms at %s (left) and %s (right)"
                     % (renv["LEFT_IP"], renv["RIGHT_IP"]))
            self._procs["real"] = mb.spawn(
                ["bash", "scripts/start_real.sh", "arm:=both"], r1, renv)
            if not mb.wait_for(
                    lambda: mb.log_says(r1, "REAL ARMS LIVE")
                    or mb.log_says(r1, "STOPPING"), 420):
                raise RuntimeError("real arms did not come up; see %s" % r1)
            if mb.log_says(r1, "STOPPING") and not mb.log_says(r1,
                                                               "REAL ARMS LIVE"):
                raise RuntimeError("start_real.sh refused; see %s" % r1)
            self._ok(2)

            # 4 --------------------------------------------------- seed
            # BEFORE the bridge, always. See master_bringup's docstring.
            self._step(3, "seeding the sim from the real arms")
            ok, why = mb.seed_sim_from_real()
            self.log("  %s" % why)
            if not ok:
                raise RuntimeError(why)
            self._ok(3)

            # 5 ------------------------------------------------- bridge
            self._step(4, "enabling both sim->real bridges")
            ok, why = mb.enable_bridges()
            self.log("  %s" % why)
            if not ok:
                raise RuntimeError("bridge refused: %s" % why)
            self._ok(4)

            # 6 ---------------------------------------------------- arm
            self._step(5, "arming the teleop")
            # A LATCH LEFT FROM A PREVIOUS RUN would make step 6 report
            # success while the arms sit still -- the teleop holds and every
            # commander is overridden. Clear it here, where it is one line,
            # rather than leaving the operator to find it.
            ok_e, why_e = mb.reset_estop()
            if ok_e:
                self.log("  cleared a latched e-stop: %s" % why_e)
            ok, why = mb.set_bool_param("/master_teleop_node",
                                        "motion_enabled", True)
            if not ok:
                raise RuntimeError("could not arm: %s" % why)
            self._ok(5)

            self.log("REAL ARMS LIVE -- move the master arm.")
            self.sig_state.emit("REAL ARMS LIVE -- move the master arm",
                                C_OK)
            self._done(True)
        except Exception as exc:                             # noqa: BLE001
            self.log("BRING-UP STOPPED: %s" % exc)
            self.sig_state.emit("stopped: %s" % str(exc)[:90], C_STOP)
            self._done(False)

    def _step(self, i, what):
        self.sig_step.emit(i, "running")
        self.sig_state.emit("[%d/6] %s" % (i + 1, what), C_WARN)
        self.log("[%d/6] %s" % (i + 1, what))

    def _ok(self, i):
        self.sig_step.emit(i, "ok")

    def _done(self, ok):
        self._running = False
        self.go.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self.go.setText("WORKING REAL ARM MASTER")
        if not ok:
            for i, lab in enumerate(self.step_labels):
                if "[running]" in lab.text():
                    self.sig_step.emit(i, "FAILED")

    # ---------------------------------------------------------- live status
    def _poll_status(self):
        if self._status_thread and self._status_thread.is_alive():
            return
        self._status_thread = threading.Thread(target=self._status_worker,
                                               daemon=True)
        self._status_thread.start()

    def _status_worker(self):
        """One shot read of /master_teleop/status.

        Its own rclpy CONTEXT, not just its own node. Sharing the default
        context with the bring-up thread is what failed the whole sequence at
        step 6 with "Executor is already spinning" -- after the arms were
        homed and both bridges were live.
        """
        try:
            s = mb.teleop_status()
            if s and not self._running:
                col = C_OK if "FOLLOW" in s else (
                    C_STOP if "E-STOP" in s else C_WARN)
                self.sig_state.emit(s[:150], col)
        except Exception:                                    # noqa: BLE001
            pass


def main():
    # BEFORE the window, because everything it spawns inherits os.environ and
    # a GUI started from a bare shell would hand every node UDP discovery,
    # which is dead on this host. This is why the window cannot be launched
    # wrong, rather than a line in a README asking you to remember.
    changed, why = mb.ensure_env()
    print(("ENV: " + why) if changed else ("env: " + why))
    app = QApplication(sys.argv)
    w = MasterMannequinWindow()
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
