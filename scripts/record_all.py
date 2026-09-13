#!/usr/bin/env python3
"""
record_all.py -- ONE PRESS, AND EVERYTHING FROM THE RUN IS ON DISK.

    python3 scripts/record_all.py --label my_run
    python3 scripts/record_all.py --stop            # stop the live session
    python3 scripts/record_all.py --status

WHAT THIS REPLACES, AND WHY IT HAD TO BE WRITTEN
================================================================
The GUI's DATA panel launched `full_state_recorder`, which is a GUIDED
SNAPSHOT tool: it blocks on `input()` at line 167 waiting for the operator to
press ENTER in a terminal. The GUI launches it detached with stdout and
stderr at DEVNULL and no controlling terminal, so it captured NOTHING, said
nothing, and exited zero. Every "recorded" trial since produced an empty
directory. That is docs/ENGINEERING_LOG.md's own "feature present but does nothing" row,
in the button whose entire job is to produce the evidence.

WHAT IS CAPTURED
================================================================
  bag/            `ros2 bag record -a` -- EVERY topic on the graph. Cameras,
                  detections, TF, joint states, master, VR, autonomy, e-stop.
                  This is the archive: anything not in the trail is still here.
  trail.csv       a flat per-sample table of the signals you actually plot,
                  sampled at a fixed rate so columns line up in time.
  events.jsonl    discrete things, stamped: clutch engage/refuse, e-stop,
                  detections, mode changes.
  arms.jsonl      the REAL arms read straight off the Kortex API (joint
                  degrees, torques, tool pose, gripper), which is the ONE
                  source no ROS topic carries when the bridge is not up.
  manifest.json   what was running, which arms, which mode, shared autonomy
                  on or off, git rev, and the topic list with message counts.
  figures/        graphs, generated on stop.

THE RATE IS SEPARATE FROM THE BAG ON PURPOSE. The bag holds every message at
its own rate; `trail.csv` is a uniform resample so that "master vs commanded
vs real" can be subtracted sample by sample without interpolation games. A
statistic of 0.000 everywhere is the classic oversampling artefact
(docs/ENGINEERING_LOG.md's instrument table), so the trail records, per row, whether each
source actually DELIVERED a new message since the last row -- the `*_fresh`
columns. A column that is never fresh is a DEAD channel, not a steady value.
"""
import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
SESSIONS = os.path.join(WS, "recordings", "sessions")
CURRENT = os.path.join(SESSIONS, ".current")

ARMS = ("left", "right")
ARM_IPS = {"left": "192.168.1.10", "right": "192.168.1.9"}


# ---------------------------------------------------------------- the trail
def _trail_columns():
    c = ["t", "wall"]
    for a in ARMS:
        c += ["sim_%s_j%d" % (a, i) for i in range(1, 8)]
        c += ["real_%s_j%d" % (a, i) for i in range(1, 8)]
        c += ["master_%s_%s" % (a, k) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        # THE TEENSY, RAW. The pose above is DERIVED -- seven potentiometers
        # and an IMU through a calibration that has changed during this
        # project and will change again. Recording only the derived pose means
        # a recalibration cannot be replayed against old sessions, and E1's
        # Fitts characterisation of the master arm has no input trajectory to
        # characterise. `/master_arm_raw_<arm>` carries all thirteen values
        # and the trail kept none of them.
        c += ["mraw_%s_p%d" % (a, i) for i in range(1, 8)]
        c += ["mraw_%s_%s" % (a, k)
              for k in ("ax", "ay", "az", "gx", "gy", "gz")]
        c += ["cmd_%s_%s" % (a, k) for k in ("x", "y", "z")]
        # THE END EFFECTOR, MEASURED. The pre-registration is explicit:
        # "No experiment may use commanded pose as a proxy for end-effector
        # pose. All position dependent variables must be measured from tf2 on
        # <arm>_end_effector_link." Without these columns every position DV in
        # E1-E4 is uncomputable, and the moving-master gain matrix the
        # document calls a prerequisite cannot be produced at all.
        c += ["ee_%s_%s" % (a, k)
              for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        # WAS THE RELAY EVEN ON. Without this, "the arm tracked badly" and
        # "the arm was not connected" are the same numbers: the 2026-08-30
        # session computes 1.0-1.8 rad of sim-vs-real error purely because the
        # relay was disabled and the two arms were in different poses.
        c += ["cascade_%s" % a]
        c += ["sim_%s_fresh" % a, "real_%s_fresh" % a, "master_%s_fresh" % a,
              "ee_%s_fresh" % a]
    for h in ARMS:
        c += ["vr_%s_engaged" % h, "vr_%s_grip" % h, "vr_%s_refused" % h,
              "vr_%s_disp_m" % h, "vr_%s_lag_m" % h, "vr_%s_cutoff_hz" % h,
              "vr_%s_tracked" % h, "vr_%s_fresh" % h]
        # THE CONTROLLER, RAW. Everything above is the MAPPER's opinion after
        # filtering, scaling and alignment. The controller's own pose and its
        # analogue inputs are what the operator actually did, and they are
        # what a re-tune of the 1-Euro constants has to be replayed against --
        # the filter changed twice on 2026-08-30 alone. Trigger and thumbstick
        # matter too: the trigger is the gripper and the stick sets scale, so
        # without them a grasp has no visible cause in the data.
        c += ["vrc_%s_%s" % (h, k)
              for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        c += ["vrc_%s_trig" % h, "vrc_%s_gripax" % h,
              "vrc_%s_stick_x" % h, "vrc_%s_stick_y" % h,
              "vrc_%s_btn_a" % h, "vrc_%s_btn_b" % h]
    # THE PREDICTOR, SAMPLE BY SAMPLE. E5 needs the ranked goal and its
    # probability over time -- H5.1 is accuracy at the commitment point and
    # H5.2 bins accuracy at 20/40/60/80% of the reach. The arbiter already
    # publishes all of it on /autonomy/arbiter_<arm> and nothing recorded it.
    for a in ARMS:
        c += ["pred_%s_top" % a, "pred_%s_p" % a, "pred_%s_ambiguous" % a,
              "pred_%s_dist_m" % a]
    # THE TRIAL, which is what turns a recording into data. Completion time
    # is undefined without a boundary, so H2.1 cannot be computed; Fitts'
    # index of difficulty needs the target's width and distance.
    # WHICH INPUT IS DRIVING, per sample.
    #
    # `vr_pose_mapper` and `master_pose_node` BOTH publish
    # /master_arm_pose_<arm> -- deliberately, so everything downstream is
    # input-agnostic. The consequence for the data is that `master_*` means
    # "the pose input", not "the mannequin": in the 86.8 s VR session of
    # 2026-08-30 `master_left_fresh` read 86% fresh with no Teensy attached,
    # because the VR mapper was publishing there.
    #
    # So the input cannot be recovered from those columns afterwards, and a
    # study that splits VR runs from mannequin runs on them is splitting on
    # nothing. This is derived from EVIDENCE rather than from a checkbox:
    # which input nodes are alive, and whether the VR clutch is engaged.
    c += ["input_mode", "vr_node_up", "master_node_up"]
    # The gripper command on both input paths, and the headset. A grasp with
    # no recorded command is an event with no cause.
    c += ["grip_cmd_left", "grip_cmd_right", "master_fsr",
          "hmd_x", "hmd_y", "hmd_z"]
    c += ["trial_id", "trial_phase", "goal_truth", "target_id",
          "target_w_m", "target_d_m"]
    c += ["estop", "n_detections", "autonomy_left", "autonomy_right"]
    return c


class Recorder:
    """A node that samples the graph at a fixed rate into a flat table."""

    def __init__(self, node, outdir, rate_hz):
        import rclpy  # noqa: F401
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState, Joy  # noqa: F401
        from std_msgs.msg import Bool, Float64MultiArray, String

        self.node = node
        self.outdir = outdir
        self.cols = _trail_columns()
        self.fh = open(os.path.join(outdir, "trail.csv"), "w", newline="")
        self.w = csv.DictWriter(self.fh, fieldnames=self.cols)
        self.w.writeheader()
        self.ev = open(os.path.join(outdir, "events.jsonl"), "a")
        self.t0 = time.time()
        self.n_rows = 0

        self.sim = {}
        self.real = {}
        self.master = {a: None for a in ARMS}
        self.cmd = {a: None for a in ARMS}
        self.vr = {h: None for h in ARMS}
        self.estop = None
        self.n_det = 0
        self.autonomy = {a: None for a in ARMS}
        self.pred = {a: {} for a in ARMS}          # arbiter JSON, per arm
        self.mraw = {a: None for a in ARMS}        # Teensy pots + IMU, raw
        self.vrc = {h: None for h in ARMS}         # controller pose, raw
        self.vrj = {h: None for h in ARMS}         # controller buttons/axes
        self.gripc = {h: None for h in ARMS}       # gripper command
        self.fsr = None                            # master FSR buttons
        self.hmd = None                            # headset pose
        self.cascade = {a: None for a in ARMS}     # is the relay ENABLED
        self.ee = {a: None for a in ARMS}          # measured EE pose from tf2
        # The trial the operator declared. Written by `--trial`, so a task
        # script marks its own boundaries and the trail carries them.
        self.trial = dict(trial_id="", trial_phase="", goal_truth="",
                          target_id="", target_w_m="", target_d_m="")
        # Freshness counters: incremented on ARRIVAL, compared per row. This
        # is what tells a dead channel from a genuinely constant one.
        self.seen = {}

        def bump(k):
            self.seen[k] = self.seen.get(k, 0) + 1

        node.create_subscription(JointState, "/joint_states",
                                 lambda m: (self._js(self.sim, m), bump("sim")), 50)
        node.create_subscription(JointState, "/real/joint_states",
                                 lambda m: (self._js(self.real, m), bump("real")), 50)
        for a in ARMS:
            node.create_subscription(
                PoseStamped, "/master_arm_pose_%s" % a,
                lambda m, a=a: (self.master.__setitem__(a, m), bump("master_%s" % a)), 50)
            node.create_subscription(
                PoseStamped, "/%s_arm_command" % a,
                lambda m, a=a: self.cmd.__setitem__(a, m), 20)
            node.create_subscription(
                String, "/autonomy/state_%s" % a,
                lambda m, a=a: self.autonomy.__setitem__(a, m.data[:40]), 10)
            node.create_subscription(
                String, "/vr/mapper_%s" % a,
                lambda m, h=a: (self._vr(h, m), bump("vr_%s" % h)), 20)
            node.create_subscription(
                String, "/autonomy/arbiter_%s" % a,
                lambda m, a=a: self._pred(a, m), 10)
            node.create_subscription(
                Bool, "/cascade_active_%s" % a,
                lambda m, a=a: self.cascade.__setitem__(a, bool(m.data)), 10)
            node.create_subscription(
                Float64MultiArray, "/master_arm_raw_%s" % a,
                lambda m, a=a: self.mraw.__setitem__(a, list(m.data)), 20)
            node.create_subscription(
                PoseStamped, "/vr/controller_pose_%s" % a,
                lambda m, h=a: self.vrc.__setitem__(h, m), 20)
            node.create_subscription(
                Joy, "/vr/controller_joy_%s" % a,
                lambda m, h=a: self.vrj.__setitem__(h, m), 20)
            node.create_subscription(
                Float64MultiArray, "/vr/gripper_%s" % a,
                lambda m, h=a: self.gripc.__setitem__(
                    h, (list(m.data) or [None])[0]), 10)
        node.create_subscription(Bool, "/estop_state",
                                 lambda m: self._estop(m), 10)
        node.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: setattr(self, "fsr", list(m.data)), 20)
        node.create_subscription(
            PoseStamped, "/vr/hmd_pose",
            lambda m: setattr(self, "hmd", m), 20)
        node.create_subscription(String, "/perception/objects_info",
                                 lambda m: self._det(m), 10)

        # THE END EFFECTOR, FROM tf2 AND NOTHING ELSE. This is the one
        # measurement the pre-registration names as mandatory and forbids
        # substituting: commanded pose is not a proxy for where the hand went.
        try:
            import tf2_ros
            self.tf_buf = tf2_ros.Buffer()
            self.tf_lis = tf2_ros.TransformListener(self.tf_buf, node)
        except Exception as e:                                # noqa: BLE001
            self.tf_buf = None
            print("  WARNING: no tf2 (%s) -- ee_* columns will be EMPTY and "
                  "every position DV is uncomputable from this run." % e)

        self.last_seen = dict(self.seen)
        node.create_timer(1.0 / rate_hz, self.tick)

    # -- callbacks
    def _js(self, into, m):
        for n, p in zip(m.name, m.position):
            into[n] = p

    def _vr(self, hand, m):
        try:
            d = json.loads(m.data)
        except Exception:                                     # noqa: BLE001
            return
        prev = self.vr[hand]
        self.vr[hand] = d
        # A clutch transition or a REFUSAL is an event, not just a sample.
        if prev is not None:
            if bool(prev.get("engaged")) != bool(d.get("engaged")):
                self.event("clutch", hand=hand, engaged=bool(d.get("engaged")))
            if prev.get("engage_refused") != d.get("engage_refused") \
                    and d.get("engage_refused"):
                self.event("clutch_refused", hand=hand,
                           reason=d.get("engage_refused"))

    def _estop(self, m):
        if self.estop is not None and bool(m.data) != self.estop:
            self.event("estop", tripped=bool(m.data))
        self.estop = bool(m.data)

    def _pred(self, arm, msg):
        """The arbiter's own JSON: state, top goal, its probability, and
        whether the intent was ambiguous. Kept raw-ish so the offline replay
        can re-derive anything without re-running the session."""
        try:
            self.pred[arm] = json.loads(msg.data)
        except Exception:                                     # noqa: BLE001
            self.pred[arm] = {}

    def _master_node_up(self):
        """Is the MANNEQUIN's node running? Cached for a second.

        Asked of the process table, because the topic cannot answer it: the
        VR mapper publishes the same topic, so a live /master_arm_pose_<arm>
        proves only that SOMETHING is driving.
        """
        now = time.monotonic()
        if now - getattr(self, "_mnode_t", 0.0) < 1.0:
            return self._mnode
        self._mnode_t = now
        try:
            self._mnode = subprocess.run(
                ["pgrep", "-f", "srl_teleop/master_pose_node"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=3).returncode == 0
        except Exception:                                     # noqa: BLE001
            self._mnode = False
        return self._mnode

    def _read_ee(self, arm):
        """Measured end-effector pose, or None. NEVER falls back to the
        commanded pose -- a silent substitution here would put an unmeasured
        number into a dependent variable, which is the one thing the
        pre-registration rules out by name."""
        if self.tf_buf is None:
            return None
        try:
            import rclpy.time
            t = self.tf_buf.lookup_transform(
                "world", "%s_end_effector_link" % arm, rclpy.time.Time())
            return t.transform
        except Exception:                                     # noqa: BLE001
            return None

    def _det(self, m):
        try:
            d = json.loads(m.data)
            objs = d.get("objects", d if isinstance(d, list) else [])
            self.n_det = len(objs)
            self.event("detections", n=self.n_det, objects=objs[:12])
        except Exception:                                     # noqa: BLE001
            pass

    def event(self, kind, **kw):
        kw.update(kind=kind, t=round(time.time() - self.t0, 4), wall=time.time())
        self.ev.write(json.dumps(kw) + "\n")
        self.ev.flush()

    # -- the sample
    def tick(self):
        r = {c: "" for c in self.cols}
        r["t"] = round(time.time() - self.t0, 4)
        r["wall"] = round(time.time(), 4)
        for a in ARMS:
            for i in range(1, 8):
                r["sim_%s_j%d" % (a, i)] = self.sim.get("%s_joint_%d" % (a, i), "")
                r["real_%s_j%d" % (a, i)] = self.real.get("%s_joint_%d" % (a, i), "")
            m = self.master[a]
            if m is not None:
                p, q = m.pose.position, m.pose.orientation
                for k, v in zip(("x", "y", "z", "qx", "qy", "qz", "qw"),
                                (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    r["master_%s_%s" % (a, k)] = round(float(v), 6)
            c = self.cmd[a]
            if c is not None:
                for k in ("x", "y", "z"):
                    r["cmd_%s_%s" % (a, k)] = round(
                        float(getattr(c.pose.position, k)), 6)
            for src in ("sim", "real", "master_%s" % a):
                key = src if src in ("sim", "real") else src
                col = "%s_fresh" % (a if src not in ("sim", "real") else src)
                col = {"sim": "sim_%s_fresh" % a, "real": "real_%s_fresh" % a}.get(
                    src, "master_%s_fresh" % a)
                r[col] = int(self.seen.get(key, 0) > self.last_seen.get(key, 0))
            d = self.vr[a] or {}
            r["vr_%s_engaged" % a] = int(bool(d.get("engaged")))
            r["vr_%s_grip" % a] = int(bool(d.get("grip_held")))
            r["vr_%s_refused" % a] = d.get("engage_refused") or ""
            r["vr_%s_disp_m" % a] = d.get("displacement_m", "")
            r["vr_%s_lag_m" % a] = d.get("lag_m", "")
            r["vr_%s_cutoff_hz" % a] = d.get("cutoff_hz", "")
            r["vr_%s_tracked" % a] = int(bool(d.get("hand_tracked")))
            r["vr_%s_fresh" % a] = int(
                self.seen.get("vr_%s" % a, 0) > self.last_seen.get("vr_%s" % a, 0))
            r["autonomy_%s" % a] = self.autonomy[a] or ""
            # measured end effector, from tf2
            tr = self._read_ee(a)
            if tr is not None:
                for k, v in zip(("x", "y", "z"),
                                (tr.translation.x, tr.translation.y,
                                 tr.translation.z)):
                    r["ee_%s_%s" % (a, k)] = round(float(v), 6)
                for k, v in zip(("qx", "qy", "qz", "qw"),
                                (tr.rotation.x, tr.rotation.y,
                                 tr.rotation.z, tr.rotation.w)):
                    r["ee_%s_%s" % (a, k)] = round(float(v), 6)
            r["ee_%s_fresh" % a] = int(tr is not None)
            r["cascade_%s" % a] = ("" if self.cascade[a] is None
                                   else int(self.cascade[a]))
            # the predictor, for E5
            pd = self.pred[a] or {}
            r["pred_%s_top" % a] = pd.get("top") or ""
            r["pred_%s_p" % a] = pd.get("top_p", "")
            r["pred_%s_ambiguous" % a] = ("" if pd.get("ambiguous") is None
                                          else int(bool(pd.get("ambiguous"))))
            r["pred_%s_dist_m" % a] = pd.get("distance_m", "")
            # the Teensy, raw: seven pots then the IMU
            mr = self.mraw[a]
            if mr and len(mr) >= 13:
                for i in range(7):
                    r["mraw_%s_p%d" % (a, i + 1)] = round(float(mr[i]), 4)
                for i, k in enumerate(("ax", "ay", "az", "gx", "gy", "gz")):
                    r["mraw_%s_%s" % (a, k)] = round(float(mr[7 + i]), 5)
            # the controller, raw pose then its analogue inputs
            cp = self.vrc[a]
            if cp is not None:
                p_, q_ = cp.pose.position, cp.pose.orientation
                for k, v in zip(("x", "y", "z", "qx", "qy", "qz", "qw"),
                                (p_.x, p_.y, p_.z, q_.x, q_.y, q_.z, q_.w)):
                    r["vrc_%s_%s" % (a, k)] = round(float(v), 6)
            jm = self.vrj[a]
            if jm is not None:
                ax = list(jm.axes) + [0.0] * 4
                bt = list(jm.buttons) + [0] * 2
                r["vrc_%s_trig" % a] = round(float(ax[0]), 4)
                r["vrc_%s_gripax" % a] = round(float(ax[1]), 4)
                r["vrc_%s_stick_x" % a] = round(float(ax[2]), 4)
                r["vrc_%s_stick_y" % a] = round(float(ax[3]), 4)
                r["vrc_%s_btn_a" % a] = int(bt[0])
                r["vrc_%s_btn_b" % a] = int(bt[1])
            r["grip_cmd_%s" % a] = ("" if self.gripc[a] is None
                                    else round(float(self.gripc[a]), 4))
        # WHICH INPUT, FROM EVIDENCE. A checkbox says what the operator
        # intended; this says what was actually driving. They differ exactly
        # when it matters -- a mis-set condition is invisible otherwise.
        vr_up = any((self.vr[a] or {}) for a in ARMS)
        vr_drive = any(bool((self.vr[a] or {}).get("engaged")) for a in ARMS)
        mast_up = self._master_node_up()
        r["vr_node_up"] = int(vr_up)
        r["master_node_up"] = int(mast_up)
        r["input_mode"] = ("vr" if vr_drive else
                           "master" if (mast_up and not vr_up) else
                           "both_up" if (mast_up and vr_up) else
                           "vr_idle" if vr_up else
                           "none")
        r["master_fsr"] = ("" if not self.fsr else
                           "|".join("%.3f" % float(v) for v in self.fsr[:6]))
        if self.hmd is not None:
            hp = self.hmd.pose.position
            for k, v in zip(("x", "y", "z"), (hp.x, hp.y, hp.z)):
                r["hmd_%s" % k] = round(float(v), 5)
        r["estop"] = "" if self.estop is None else int(self.estop)
        r["n_detections"] = self.n_det
        # The declared trial, carried on every row so a slice of the CSV is
        # self-describing -- you can cut on trial_id without joining anything.
        #
        # Re-read at most twice a second: `--trial` is a separate process
        # writing a file, and polling it is what lets a task script mark
        # boundaries without this node exposing a service.
        now = time.monotonic()
        if now - getattr(self, "_trial_t", 0.0) > 0.5:
            self._trial_t = now
            try:
                with open(os.path.join(self.outdir, "trial.json")) as fh:
                    self.trial.update(json.load(fh))
            except Exception:                                 # noqa: BLE001
                pass
        for k, v in self.trial.items():
            r[k] = v
        self.last_seen = dict(self.seen)
        self.w.writerow(r)
        self.n_rows += 1
        if self.n_rows % 50 == 0:
            self.fh.flush()

    def close(self):
        try:
            self.fh.flush(); self.fh.close()
        except Exception:                                     # noqa: BLE001
            pass
        try:
            self.ev.close()
        except Exception:                                     # noqa: BLE001
            pass


# ------------------------------------------------------- the real arm poller
def poll_arms(outdir, stop_flag, period_s=0.25):
    """Read the REAL arms over Kortex, in a thread, into arms.jsonl.

    WHY THIS EXISTS SEPARATELY. When the bridge is not running there is no
    /real/joint_states at all, so a session recorded during setup, homing or a
    refusal has NO record of where the metal actually was. This reads the arm
    itself. It is skipped, loudly, if the kortex venv is not the interpreter.
    """
    try:
        import kortex_api.autogen.client_stubs.BaseCyclicClientRpc as BC
        import kortex_api.autogen.messages.Session_pb2 as Session_pb2
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.RouterClient import RouterClient
        from kortex_api.SessionManager import SessionManager
    except Exception as e:                                    # noqa: BLE001
        with open(os.path.join(outdir, "arms.jsonl"), "w") as f:
            f.write(json.dumps(dict(
                skipped="kortex_api not importable in this interpreter: %s. "
                        "Run under .kortex_venv/bin/python to capture the real "
                        "arms directly." % e)) + "\n")
        return

    # HARD CONSTRAINT 2: the arm permits exactly ONE Kortex session. If the
    # bridge is up it owns them and this MUST NOT take a second one.
    if _bridge_running():
        with open(os.path.join(outdir, "arms.jsonl"), "w") as f:
            f.write(json.dumps(dict(
                skipped="kortex_highlevel_bridge is running and owns the one "
                        "permitted session per arm; /real/joint_states in the "
                        "bag and trail is the record instead.")) + "\n")
        return

    conns = {}
    for a, ip in ARM_IPS.items():
        try:
            tr = TCPTransport()
            rt = RouterClient(tr, lambda k: None)
            tr.connect(ip, 10000)
            si = Session_pb2.CreateSessionInfo()
            si.username = "admin"; si.password = "admin"
            si.session_inactivity_timeout = 60000
            si.connection_inactivity_timeout = 2000
            sm = SessionManager(rt); sm.CreateSession(si)
            conns[a] = (tr, sm, BC.BaseCyclicClient(rt))
        except Exception as e:                                # noqa: BLE001
            conns[a] = ("error", str(e), None)

    with open(os.path.join(outdir, "arms.jsonl"), "w") as f:
        t0 = time.time()
        while not stop_flag[0]:
            row = {"t": round(time.time() - t0, 3)}
            for a, c in conns.items():
                if c[0] == "error":
                    row[a] = {"error": c[1]}
                    continue
                try:
                    fb = c[2].RefreshFeedback()
                    row[a] = dict(
                        joints_deg=[round(x.position, 3) for x in fb.actuators],
                        torque_nm=[round(x.torque, 2) for x in fb.actuators],
                        tool=[round(fb.base.tool_pose_x, 4),
                              round(fb.base.tool_pose_y, 4),
                              round(fb.base.tool_pose_z, 4)])
                    try:
                        row[a]["gripper"] = round(
                            fb.interconnect.gripper_feedback.motor[0].position, 2)
                    except Exception:                         # noqa: BLE001
                        pass
                except Exception as e:                        # noqa: BLE001
                    row[a] = {"error": str(e)}
            f.write(json.dumps(row) + "\n"); f.flush()
            time.sleep(period_s)
    for a, c in conns.items():
        if c[0] != "error":
            try:
                c[1].CloseSession()
            except Exception:                                 # noqa: BLE001
                pass
            try:
                c[0].disconnect()
            except Exception:                                 # noqa: BLE001
                pass
            print("kortex session closed cleanly (%s)" % a, file=sys.stderr)


def _bridge_running():
    try:
        out = subprocess.run(["ps", "-eo", "args"], capture_output=True,
                             text=True, timeout=5).stdout
    except Exception:                                         # noqa: BLE001
        return False
    return "kortex_highlevel_bridge" in out


# ------------------------------------------------------------------ figures
def make_figures(outdir):
    """Graphs, from the trail. Written on stop so a session is self-contained."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:                                    # noqa: BLE001
        return ["matplotlib unavailable: %s" % e]
    path = os.path.join(outdir, "trail.csv")
    if not os.path.exists(path):
        return ["no trail.csv"]
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return ["trail.csv is empty -- nothing was publishing"]
    figs = os.path.join(outdir, "figures")
    os.makedirs(figs, exist_ok=True)
    made = []

    def col(name):
        out = []
        for r in rows:
            v = r.get(name, "")
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                out.append(float("nan"))
        return out

    t = col("t")

    # 1. joints, sim vs real, per arm
    for a in ARMS:
        f, axes = plt.subplots(7, 1, figsize=(10, 12), sharex=True)
        any_data = False
        for i in range(1, 8):
            s, rl = col("sim_%s_j%d" % (a, i)), col("real_%s_j%d" % (a, i))
            if any(v == v for v in s):
                axes[i - 1].plot(t, s, label="sim", lw=1.0); any_data = True
            if any(v == v for v in rl):
                axes[i - 1].plot(t, rl, label="real", lw=1.0); any_data = True
            axes[i - 1].set_ylabel("j%d" % i, fontsize=8)
            axes[i - 1].grid(alpha=.3)
        axes[0].legend(fontsize=8); axes[-1].set_xlabel("t (s)")
        f.suptitle("%s arm joints -- sim vs real" % a)
        if any_data:
            p = os.path.join(figs, "joints_%s.png" % a)
            f.savefig(p, dpi=110, bbox_inches="tight"); made.append(p)
        plt.close(f)

    # 2. the clutch: grip held vs actually engaged. The gap IS the defect.
    f, ax = plt.subplots(figsize=(11, 4))
    plotted = False
    for a in ARMS:
        g, e = col("vr_%s_grip" % a), col("vr_%s_engaged" % a)
        if any(v == v for v in g):
            ax.plot(t, g, label="%s grip held" % a, lw=1.4, ls="--"); plotted = True
        if any(v == v for v in e):
            ax.plot(t, e, label="%s ENGAGED" % a, lw=1.8); plotted = True
    ax.set_ylim(-.15, 1.3); ax.set_xlabel("t (s)"); ax.grid(alpha=.3)
    ax.legend(fontsize=8)
    ax.set_title("VR clutch -- squeezed vs engaged. Any gap is a REFUSED engage;\n"
                 "events.jsonl names the reason.")
    if plotted:
        p = os.path.join(figs, "clutch.png")
        f.savefig(p, dpi=110, bbox_inches="tight"); made.append(p)
    plt.close(f)

    # 3. master / commanded trails in space
    f, ax = plt.subplots(figsize=(11, 4))
    plotted = False
    for a in ARMS:
        for src in ("master", "cmd"):
            xs = col("%s_%s_x" % (src, a))
            ys = col("%s_%s_y" % (src, a))
            zs = col("%s_%s_z" % (src, a))
            if any(v == v for v in xs):
                ax.plot(t, xs, lw=.9, label="%s %s x" % (a, src))
                ax.plot(t, ys, lw=.9, label="%s %s y" % (a, src))
                ax.plot(t, zs, lw=.9, label="%s %s z" % (a, src))
                plotted = True
    ax.set_xlabel("t (s)"); ax.set_ylabel("m"); ax.grid(alpha=.3)
    ax.legend(fontsize=7, ncol=3); ax.set_title("master pose and commanded pose")
    if plotted:
        p = os.path.join(figs, "trails.png")
        f.savefig(p, dpi=110, bbox_inches="tight"); made.append(p)
    plt.close(f)

    # 4. FRESHNESS. A flat line at zero is a dead channel and is the single
    #    most common way a recording lies about having captured something.
    f, ax = plt.subplots(figsize=(11, 3.4))
    names = [c for c in rows[0] if c.endswith("_fresh")]
    for i, n in enumerate(names):
        v = col(n)
        ax.plot(t, [x + i * 1.2 if x == x else float("nan") for x in v], lw=.8)
        ax.text(0, i * 1.2 + .35, n.replace("_fresh", ""), fontsize=7, va="bottom")
    ax.set_yticks([]); ax.set_xlabel("t (s)")
    ax.set_title("channel freshness -- a flat line at the baseline is a DEAD channel")
    p = os.path.join(figs, "freshness.png")
    f.savefig(p, dpi=110, bbox_inches="tight"); made.append(p); plt.close(f)
    return made


# ------------------------------------------------------------------- summary
def summarise(outdir):
    s = {"session": os.path.basename(outdir)}
    tp = os.path.join(outdir, "trail.csv")
    if os.path.exists(tp):
        rows = list(csv.DictReader(open(tp)))
        s["trail_rows"] = len(rows)
        if rows:
            try:
                s["duration_s"] = round(float(rows[-1]["t"]), 2)
            except Exception:                                 # noqa: BLE001
                pass
            dead, live = [], []
            for c in rows[0]:
                if not c.endswith("_fresh"):
                    continue
                n = sum(1 for r in rows if r.get(c) == "1")
                (live if n else dead).append(c.replace("_fresh", ""))
            s["channels_live"] = live
            s["channels_DEAD"] = dead
    ep = os.path.join(outdir, "events.jsonl")
    if os.path.exists(ep):
        evs = [json.loads(l) for l in open(ep) if l.strip()]
        s["n_events"] = len(evs)
        kinds = {}
        for e in evs:
            kinds[e.get("kind")] = kinds.get(e.get("kind"), 0) + 1
        s["events_by_kind"] = kinds
        refs = {}
        for e in evs:
            if e.get("kind") == "clutch_refused":
                refs[e.get("reason")] = refs.get(e.get("reason"), 0) + 1
        if refs:
            s["clutch_refusals"] = refs
    bag = os.path.join(outdir, "bag")
    if os.path.isdir(bag):
        try:
            out = subprocess.run(["ros2", "bag", "info", bag], capture_output=True,
                                 text=True, timeout=60).stdout
            s["bag_info"] = out
            s["bag_bytes"] = sum(
                os.path.getsize(os.path.join(bag, f)) for f in os.listdir(bag)
                if os.path.isfile(os.path.join(bag, f)))
        except Exception as e:                                # noqa: BLE001
            s["bag_info"] = "unreadable: %s" % e
    ap = os.path.join(outdir, "arms.jsonl")
    if os.path.exists(ap):
        s["arm_samples"] = sum(1 for _ in open(ap))
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump(s, f, indent=2)
    return s


def _bag_stop(bagp, grace=25.0):
    """SIGINT the bag's process group, escalate, and VERIFY it is gone.

    SIGINT first and generously: it is what closes the mcap cleanly, and a
    12 GB file takes time to flush. Only then TERM, and only then KILL, each
    on the GROUP -- `ros2 bag record` is a python wrapper and signalling the
    wrapper alone leaves the recorder writing.

    Returns True if the recorder is gone. The caller prints the answer either
    way, because "I asked it to stop" is not the same fact as "it stopped".
    """
    def alive():
        try:
            os.kill(bagp.pid, 0)
            return True
        except OSError:
            return False

    for sig, wait in ((signal.SIGINT, grace),
                      (signal.SIGTERM, 10.0),
                      (signal.SIGKILL, 5.0)):
        if not alive():
            return True
        try:
            os.killpg(os.getpgid(bagp.pid), sig)
        except Exception:                                     # noqa: BLE001
            try:
                os.kill(bagp.pid, sig)
            except Exception:                                 # noqa: BLE001
                pass
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            if not alive():
                return True
            time.sleep(0.25)
    return not alive()


# ---------------------------------------------------------------------- main
def do_trial(a):
    """Write a trial marker into the live session.

    Goes to `events.jsonl` AND to a small `trial.json` the recorder polls, so
    the boundary survives in two places: the event stream keeps the exact
    timestamp, and the poll file is what puts the fields on every subsequent
    row.
    """
    if not os.path.exists(CURRENT):
        print("no live session -- start one before marking trials")
        return 1
    info = json.load(open(CURRENT))
    outdir = info["outdir"]
    rec = dict(kind="trial_%s" % a.trial, t=time.time(),
               trial_id=a.trial_id, trial_phase=a.trial_phase,
               goal_truth=a.goal, target_id=a.target_id,
               target_w_m=a.target_w, target_d_m=a.target_d,
               condition=a.condition)
    with open(os.path.join(outdir, "events.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")
    live = dict(trial_id=a.trial_id if a.trial == "start" else "",
                trial_phase=a.trial_phase if a.trial == "start" else "",
                goal_truth=a.goal if a.trial == "start" else "",
                target_id=a.target_id if a.trial == "start" else "",
                target_w_m=a.target_w if a.trial == "start" else "",
                target_d_m=a.target_d if a.trial == "start" else "")
    with open(os.path.join(outdir, "trial.json"), "w") as fh:
        json.dump(live, fh)
    print("trial %s: %s" % (a.trial, a.trial_id or "(unnamed)"))
    return 0


def do_stop():
    if not os.path.exists(CURRENT):
        print("no live session")
        return 1
    info = json.load(open(CURRENT))
    pid, outdir = info["pid"], info["outdir"]
    try:
        os.killpg(os.getpgid(pid), signal.SIGINT)
        print("stopping session %s (pid %d)" % (os.path.basename(outdir), pid))
    except Exception as e:                                    # noqa: BLE001
        print("could not signal pid %d: %s -- finalising anyway" % (pid, e))
    for _ in range(60):
        if not os.path.exists(CURRENT):
            break
        time.sleep(0.5)
    if os.path.exists(CURRENT):
        os.remove(CURRENT)
        make_figures(outdir)
        summarise(outdir)
    print("session at %s" % outdir)
    return 0


def do_status():
    if not os.path.exists(CURRENT):
        print("RECORDING: no")
        return 1
    info = json.load(open(CURRENT))
    alive = os.path.exists("/proc/%d" % info["pid"])
    print("RECORDING: %s  session=%s  pid=%d"
          % ("yes" if alive else "STALE (process gone)",
             os.path.basename(info["outdir"]), info["pid"]))
    if not alive:
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="session")
    ap.add_argument("--mode", default="unspecified",
                    help="control mode this run is recorded under")
    ap.add_argument("--shared-autonomy", default="unknown",
                    choices=["on", "off", "unknown"])
    ap.add_argument("--participant", default="")
    # THE INPUT, AND A SHARED-AUTONOMY TICK PER INPUT. Both are recorded
    # because `--shared-autonomy` alone was an OR across two tabs and lost
    # which mode it belonged to; and because the pose topic is shared by the
    # VR mapper and the mannequin, so nothing downstream can recover the
    # input after the fact.
    ap.add_argument("--input-mode", default="unknown",
                    help="vr | master | both_up | unknown")
    ap.add_argument("--shared-vr", default="unknown")
    ap.add_argument("--shared-master", default="unknown")
    ap.add_argument("--rate", type=float, default=20.0, help="trail sample Hz")
    ap.add_argument("--no-bag", action="store_true",
                    help="skip `ros2 bag record -a` (images are large)")
    ap.add_argument("--max-gb", type=float, default=20.0,
                    help="stop recording once the bag reaches this size "
                         "(default 20 GB). 0 disables the cap")
    ap.add_argument("--max-minutes", type=float, default=0.0,
                    help="stop after this many minutes (0 = no limit)")
    ap.add_argument("--full-images", action="store_true",
                    help="record RAW camera frames too. ~10 GB per minute; "
                         "the compressed topics are kept either way")
    ap.add_argument("--no-arms", action="store_true",
                    help="skip reading the real arms over Kortex")
    ap.add_argument("--stop", action="store_true")
    # THE TRIAL BOUNDARY, declarable from anywhere.
    #
    # A recording without one is not data: completion time has no definition,
    # so H2.1 cannot be computed, and Fitts' index of difficulty needs the
    # target's width and distance which nothing was recording either. This
    # writes into the LIVE session, so a task script marks its own boundaries
    # while the recorder runs:
    #
    #   record_all.py --trial start --trial-id t07 --goal red_cube \
    #                 --target-id cube_a --target-w 0.04 --target-d 0.35
    #   ... the operator does the reach ...
    #   record_all.py --trial end --trial-id t07
    #
    # The fields ride on EVERY row from then on, so a slice of the CSV is
    # self-describing -- cut on trial_id and you need to join nothing.
    ap.add_argument("--trial", choices=("start", "end"),
                    help="mark a trial boundary in the LIVE session")
    ap.add_argument("--trial-id", default="")
    ap.add_argument("--trial-phase", default="")
    ap.add_argument("--goal", default="", help="GROUND TRUTH goal for E5")
    ap.add_argument("--target-id", default="")
    ap.add_argument("--target-w", default="", help="target width, m (Fitts)")
    ap.add_argument("--target-d", default="", help="target distance, m (Fitts)")
    ap.add_argument("--condition", default="",
                    help="direct | timid | aggressive -- E2 needs three "
                         "levels, not a shared-autonomy on/off flag")
    ap.add_argument("--status", action="store_true")
    a = ap.parse_args()

    if a.trial:
        return do_trial(a)
    if a.stop:
        return do_stop()
    if a.status:
        return do_status()

    if os.path.exists(CURRENT):
        info = json.load(open(CURRENT))
        if os.path.exists("/proc/%d" % info["pid"]):
            print("ALREADY RECORDING into %s (pid %d). Stop it first: "
                  "python3 scripts/record_all.py --stop"
                  % (info["outdir"], info["pid"]))
            return 2
        os.remove(CURRENT)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # THE SAME RULE THE WINDOW SHOWS THE OPERATOR. Spaces become
    # underscores rather than vanishing: deleting them turned
    # "E2 aggressive run 3" into "E2aggressiverun3", and a name nobody can
    # read in a listing is not a name. `srl_gui._rec_label` applies this
    # identically so what is typed is what appears on disk.
    _s, _prev = [], ""
    for _c in (a.label or ""):
        _c = "_" if (_c.isspace() or _c in "/\\.:") else _c
        if not (_c.isalnum() or _c in "-_"):
            continue
        if _c == "_" and _prev == "_":
            continue
        _s.append(_c)
        _prev = _c
    safe = "".join(_s).strip("_-") or "session"
    outdir = os.path.join(SESSIONS, "%s_%s" % (stamp, safe))
    os.makedirs(outdir, exist_ok=True)
    os.makedirs(SESSIONS, exist_ok=True)

    try:
        os.setpgrp()
    except Exception:                                         # noqa: BLE001
        pass
    with open(CURRENT, "w") as f:
        json.dump(dict(pid=os.getpid(), outdir=outdir, started=time.time()), f)

    def git(*args):
        try:
            return subprocess.run(["git"] + list(args), cwd=WS, capture_output=True,
                                  text=True, timeout=10).stdout.strip()
        except Exception:                                     # noqa: BLE001
            return ""

    manifest = dict(
        label=a.label, mode=a.mode, shared_autonomy=a.shared_autonomy,
        input_mode=a.input_mode, shared_vr=a.shared_vr,
        shared_master=a.shared_master,
        participant=a.participant, started=time.time(),
        started_iso=datetime.now().isoformat(timespec="seconds"),
        git_rev=git("rev-parse", "HEAD"), git_branch=git("rev-parse", "--abbrev-ref", "HEAD"),
        git_dirty=bool(git("status", "--porcelain")),
        ros_domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
        rmw=os.environ.get("RMW_IMPLEMENTATION", ""),
        arm_ips=ARM_IPS, trail_rate_hz=a.rate,
        bag=not a.no_bag, arms_polled=not a.no_arms)
    # HARD CONSTRAINT 12: anonymity. `participant` is a code, and a name
    # shaped like one is refused rather than quietly written to disk.
    for bad in ("@", " "):
        if bad in a.participant:
            print("REFUSED: --participant must be an anonymous code, not a name "
                  "or an address (docs/ENGINEERING_LOG.md hard constraint 12).")
            os.remove(CURRENT)
            return 3
    with open(os.path.join(outdir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    print("=" * 68)
    print("RECORDING EVERYTHING into %s" % outdir)
    print("  mode=%s  shared_autonomy=%s  trail=%.0f Hz  bag=%s  arms=%s"
          % (a.mode, a.shared_autonomy, a.rate, not a.no_bag, not a.no_arms))
    print("  stop with:  python3 scripts/record_all.py --stop")
    print("=" * 68)

    bagp = None
    if not a.no_bag:
        # THE RAW IMAGE TOPICS ARE 99% OF THE BAG AND ADD NOTHING.
        #
        # Measured 2026-08-30: a 4.3 second recording produced 4.87 GB, and
        # the same session left running reached 12 GB. That is 1121 MB/s, and
        # essentially all of it is uncompressed camera frames -- one wrist
        # camera at 1280x720 bgr8 30 Hz is 83 MB/s on its own, and there are
        # two of them plus the scene camera plus depth.
        #
        # Every one of those cameras ALSO publishes `.../compressed`, which
        # is the same picture at 174 kB a frame instead of 2.7 MB. So the raw
        # topics are excluded and the compressed ones kept: the recording
        # still contains every camera, and the arithmetic changes completely.
        #
        #     two wrist cameras, raw        166 MB/s     10 GB per minute
        #     the same, compressed           10 MB/s    0.6 GB per minute
        #
        # A ten minute run goes from 100 GB -- which does not fit anywhere and
        # would have filled this disk -- to 6 GB. Nothing else is dropped:
        # joint states, TF, the master arm, VR, autonomy and every detection
        # are small and are all still recorded by `-a`.
        #
        # `--full-images` puts the raw frames back for the rare case that
        # needs pixel-exact data, and says what it will cost.
        excl = [] if a.full_images else [
            "--exclude-regex", r"^/.*/image_raw$",
            "--exclude-regex", r"^/.*/image_raw/compressedDepth$",
            "--exclude-regex", r"^/.*/image_raw/theora$",
            "--exclude-regex", r"^/scene_camera/image_raw$",
        ]
        bagp = subprocess.Popen(
            ["ros2", "bag", "record", "-a",
             "--compression-mode", "file", "--compression-format", "zstd"]
            + excl + ["-o", os.path.join(outdir, "bag")],
            stdout=open(os.path.join(outdir, "bag_record.log"), "w"),
            stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        print("  ros2 bag record -a  (pid %d)%s"
              % (bagp.pid, "  RAW IMAGES INCLUDED -- expect ~10 GB/min"
                 if a.full_images else
                 "  compressed images only (~0.6 GB/min)"))

    stop_flag = [False]
    armt = None
    if not a.no_arms:
        import threading
        armt = threading.Thread(target=poll_arms, args=(outdir, stop_flag), daemon=True)
        armt.start()

    rec = node = None
    try:
        import rclpy
        rclpy.init()
        node = rclpy.create_node("srl_record_all")
        rec = Recorder(node, outdir, a.rate)
        rec.event("session_start", **{k: manifest[k] for k in
                                      ("label", "mode", "shared_autonomy")})
        print("  trail + events running. Ctrl-C or --stop to finish.")
        if a.max_gb:
            print("  size cap %.0f GB, then it stops itself." % a.max_gb)
        if a.max_minutes:
            print("  time cap %.0f min." % a.max_minutes)
        print("")
        # A CAP, BECAUSE A RECORDER THAT NEVER STOPS FILLS THE DISK.
        #
        # On 2026-08-30 a session was left running and the bag reached 12 GB
        # while every indicator said it had stopped. Even with raw images now
        # excluded, a long session is unbounded, and "unbounded" on a 1 TB
        # disk means an afternoon. So the recorder watches its own output and
        # ENDS THE SESSION cleanly when it hits either cap -- a clean stop
        # writes the summary and the figures, which a disk-full crash does
        # not.
        _t0 = time.monotonic()
        _bagdir = os.path.join(outdir, "bag")

        def _too_big():
            if a.max_minutes and (time.monotonic() - _t0) > a.max_minutes * 60:
                return "the %.0f minute cap" % a.max_minutes
            if a.max_gb and os.path.isdir(_bagdir):
                try:
                    n = sum(os.path.getsize(os.path.join(_bagdir, f))
                            for f in os.listdir(_bagdir)
                            if os.path.isfile(os.path.join(_bagdir, f)))
                except OSError:
                    return None
                if n > a.max_gb * 1e9:
                    return "the %.0f GB cap (%.1f GB written)" % (
                        a.max_gb, n / 1e9)
            return None

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            why = _too_big()
            if why:
                print("\n  STOPPING: reached %s. The session is being closed "
                      "cleanly, so the summary and figures are still written."
                      % why)
                break
    except KeyboardInterrupt:
        print("\nstopping...")
    except Exception as e:                                    # noqa: BLE001
        print("recorder error: %r" % e)
    finally:
        stop_flag[0] = True
        if rec:
            rec.event("session_stop")
            rec.close()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:                                 # noqa: BLE001
                pass
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:                                     # noqa: BLE001
            pass
        if bagp is not None:
            # STOP IT, THEN CHECK THAT IT STOPPED.
            #
            # THE DEFECT, 2026-08-30: the operator pressed STOP RECORDING,
            # the summary and the graphs were written, the marker was
            # removed, the window said "not recording" -- and `ros2 bag
            # record` WAS STILL RUNNING. It went on writing at 58 MB/s and
            # the bag grew from 4.9 GB to 12 GB while everything on screen
            # said the session had ended. On a 1 TB disk that is a few hours
            # from full, silently.
            #
            # The old code sent SIGINT to the group, waited, and on timeout
            # called `bagp.kill()` -- which signals the WRAPPER PID only, not
            # the group, so the recorder underneath it survived. And nothing
            # ever checked: the failure had no way to be noticed.
            #
            # HARD CONSTRAINT 9 is the rule this broke -- kill the process
            # GROUP, `ros2 bag record` is a wrapper. Escalate on the group,
            # verify at each step, and SAY SO if it is still alive, because a
            # recorder nobody knows about is worse than one that refuses to
            # start.
            _bag_stop(bagp)
        if armt is not None:
            armt.join(timeout=8)
        if bagp is not None and not _bag_stop(bagp, grace=0.0):
            print("\n  WARNING: `ros2 bag record` (pid %d) IS STILL RUNNING "
                  "after SIGINT, SIGTERM and SIGKILL. It is still writing to "
                  "the bag. Kill it by hand:  kill -9 -%d"
                  % (bagp.pid, bagp.pid))
        made = make_figures(outdir)
        s = summarise(outdir)
        if os.path.exists(CURRENT):
            os.remove(CURRENT)
        print("\n" + "=" * 68)
        print("SAVED  %s" % outdir)
        print("  trail rows : %s" % s.get("trail_rows"))
        print("  duration   : %s s" % s.get("duration_s"))
        print("  events     : %s %s" % (s.get("n_events"), s.get("events_by_kind", "")))
        if s.get("channels_DEAD"):
            print("  DEAD channels (nothing ever published): %s"
                  % ", ".join(s["channels_DEAD"]))
        if s.get("clutch_refusals"):
            print("  CLUTCH REFUSALS: %s" % s["clutch_refusals"])
        print("  figures    : %d" % len(made))
        for m in made:
            print("      %s" % os.path.relpath(m, WS))
        print("=" * 68)
    return 0


if __name__ == "__main__":
    sys.exit(main())
