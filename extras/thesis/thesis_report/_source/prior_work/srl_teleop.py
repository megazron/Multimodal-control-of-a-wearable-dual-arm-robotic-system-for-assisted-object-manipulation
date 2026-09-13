"""
SRL Teleoperation — Dual Kinova Gen3 + MuJoCo Sim
==================================================
Teensy format: k1j1:0.0,k1j2:45.0,...,k2j1:0.0,...  (0-360 deg raw from pots)

Run standalone:   py -3.10 srl_teleop.py
Run via launcher: py -3.10 srl_launcher.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import serial
import threading
import copy
import xml.etree.ElementTree as ET
import os
import time

# ── Kortex (optional) ────────────────────────────────────────────────────────
try:
    import kortex_api.autogen.client_stubs.BaseClientRpc as BaseClient
    import kortex_api.autogen.messages.Base_pb2 as Base_pb2
    import kortex_api.autogen.messages.Session_pb2 as Session_pb2
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    KORTEX_AVAILABLE = True
except ImportError:
    KORTEX_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════════════════
# DEFAULTS — used when running without launcher
# Override these here or let the launcher write srl_runtime_cfg.py
# ══════════════════════════════════════════════════════════════════════════════
SERIAL_PORT  = "COM3"
BAUD_RATE    = 115200
GEN3_XML     = "mujoco_menagerie/kinova_gen3/gen3.xml"
NUM_JOINTS   = 7

# Which pots are physically connected — must match Teensy connectedK1/K2 arrays
# True  = pot is wired, use it to control that joint
# False = pot not connected, hold joint at arm's current real position
CONNECTED_K1 = [False, False, False, False, False, False, True]   # J7 connected (matches your pot)
CONNECTED_K2 = [False, False, False, False, False, False, False]  # none connected yet

# ══════════════════════════════════════════════════════════════════════════════
#  >>> ARM IP ADDRESSES — EDIT THESE TO MATCH YOUR ARMS <<<
# ══════════════════════════════════════════════════════════════════════════════
# Each Gen3 has its own IP. Default Kinova IP is 192.168.1.10
# To find/set an arm's IP: connect ethernet, open browser to 192.168.1.10,
# log in (admin/admin), go to Network settings.
# For TWO arms you MUST give them different IPs (e.g. .10 and .11)
ARM_K1_IP    = "192.168.1.10"   # <-- right arm IP
ARM_K2_IP    = "192.168.1.9"    # <-- left arm IP
ARM_PORT     = 10000            # default Kortex TCP port — don't change
ARM_USER     = "admin"          # default login
ARM_PASS     = "admin"          # default password
# ══════════════════════════════════════════════════════════════════════════════

# Control mode: "sim_only" | "both" | "real_only"
DEFAULT_MODE        = "both"

# Home pose — joint angles in degrees (mapped space, not raw 0-360)
# Arms in a neutral raised-back position, safe for shoulder mount
DEFAULT_HOME_K1_DEG = [0.0, -60.0, 0.0, 60.0, 0.0, 0.0, 0.0]
DEFAULT_HOME_K2_DEG = [0.0, 60.0, 0.0, -60.0, 0.0, 0.0, 0.0]
DEFAULT_HOME_SPEED  = 5.0   # deg/s during homing

# Safety
STRICT_HOLD         = True  # joints without a pot are LOCKED, never move (recommended)
# Velocity control gains (real arm) — proven smooth+responsive in test_kinova
VEL_KP              = 4.0   # proportional gain
VEL_MAX             = 50.0  # deg/s max joint speed
VEL_DEADBAND        = 1.0   # deg — stop when within this of target
MAX_SPEED_DEG_S     = 15.0  # cap for real arm commands
WATCHDOG_TIMEOUT_S  = 2.0   # seconds before watchdog stops real arms

# Soft joint limits (degrees, mapped space)
# No clamping during teleoperation — just warn. Set to None to disable.
# Soft limits disabled — set ENABLE_SOFT_LIMITS = True to re-enable
ENABLE_SOFT_LIMITS = False
SOFT_LIMITS_DEG = [
    (-180, 180),  # J1
    (-180, 180),  # J2
    (-180, 180),  # J3
    (-180, 180),  # J4
    (-180, 180),  # J5
    (-180, 180),  # J6
    (-180, 180),  # J7
]

# Safety zones — HEAD only causes hold, others just warn
# Set ENABLE_ZONES = False to disable entirely
ENABLE_ZONES     = False
ZONE_MARGIN      = 0.05
ZONE_HEAD_CENTRE = np.array([0.0,  0.05,  0.45])
ZONE_HEAD_RADIUS = 0.20

# ── Load launcher config if available ────────────────────────────────────────
try:
    import srl_runtime_cfg as _cfg
    _MODE      = _cfg.LAUNCH_MODE.lower()
    _HOME_K1_DEG = _cfg.HOME_ANGLES_DEG
    _HOME_K2_DEG = _cfg.HOME_ANGLES_DEG
    _HOME_SPD  = _cfg.HOMING_SPEED
    print(f"[Config] Launcher: mode={_MODE}  pose={_cfg.HOME_POSE_NAME}")
except ImportError:
    _MODE      = DEFAULT_MODE
    _HOME_K1_DEG = DEFAULT_HOME_K1_DEG
    _HOME_K2_DEG = DEFAULT_HOME_K2_DEG
    _HOME_SPD  = DEFAULT_HOME_SPEED
    print(f"[Config] Standalone: mode={_MODE}")

USE_SIM  = _MODE in ("sim_only", "both")
USE_REAL = _MODE in ("both", "real_only") and KORTEX_AVAILABLE

# ══════════════════════════════════════════════════════════════════════════════
# POT → JOINT MAPPING
# Teensy pots: 0–360 raw.  Map to joint space: raw - 180 → -180..+180 deg
# ══════════════════════════════════════════════════════════════════════════════
# Unconnected pots float between 0-25 deg raw due to ADC noise.
# Only treat a pot as active if it reads above this threshold.
POT_NOISE_FLOOR = 3.0    # raw deg — below this = treat as disconnected (pots at rail send ~0-2)

def raw_to_joint(raw_deg):
    """
    Map raw 0-360 pot value to joint angle (-180..+180).
    Returns None if raw is in noise floor (pot not connected).
    """
    if raw_deg <= POT_NOISE_FLOOR:
        return None   # below noise floor — disconnected
    angle = float(raw_deg) - 180.0
    # Treat near-rail values as floating/disconnected
    # Unconnected Teensy ADC pins read near the rail (~0 or ~360 raw)
    # Real pots sweep through middle range when moved
    if abs(angle) > 170.0:
        return None   # near rail — treat as floating/disconnected
    return float(angle)

# ══════════════════════════════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════════════════════════════
k1_deg  = np.array(_HOME_K1_DEG, dtype=float)
k2_deg  = np.array(_HOME_K2_DEG, dtype=float)
lock    = threading.Lock()
updated = threading.Event()
estop   = threading.Event()   # when set, all motion stops immediately

# ══════════════════════════════════════════════════════════════════════════════
# HUMAN XML
# ══════════════════════════════════════════════════════════════════════════════
HUMAN_XML = """
  <geom name="head"    type="sphere"   size="0.1"           pos="0 0 0.35"      rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="neck"    type="cylinder" size="0.03 0.05"     pos="0 0 0.22"      rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="torso"   type="box"      size="0.14 0.07 0.22" pos="0 0 0"        rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="hips"    type="box"      size="0.10 0.07 0.06" pos="0 0 -0.26"    rgba="0.20 0.28 0.45 1" contype="0" conaffinity="0"/>
  <geom name="l_thigh" type="cylinder" size="0.04 0.18"     pos="0.06 0 -0.48"  rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="l_shin"  type="cylinder" size="0.03 0.17"     pos="0.06 0 -0.83"  rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="l_foot"  type="box"      size="0.04 0.10 0.03" pos="0.06 0.04 -1.02" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
  <geom name="r_thigh" type="cylinder" size="0.04 0.18"     pos="-0.06 0 -0.48" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="r_shin"  type="cylinder" size="0.03 0.17"     pos="-0.06 0 -0.83" rgba="0.25 0.35 0.55 1" contype="0" conaffinity="0"/>
  <geom name="r_foot"  type="box"      size="0.04 0.10 0.03" pos="-0.06 0.04 -1.02" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="0"/>
  <geom name="l_upper" type="cylinder" size="0.025 0.10"    pos="0.17 0 0.08"   euler="0 0 20"  rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="l_fore"  type="cylinder" size="0.02 0.09"     pos="0.19 0 -0.10"  euler="0 0 10"  rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="r_upper" type="cylinder" size="0.025 0.10"    pos="-0.17 0 0.08"  euler="0 0 -20" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="r_fore"  type="cylinder" size="0.02 0.09"     pos="-0.19 0 -0.10" euler="0 0 -10" rgba="0.85 0.72 0.60 1" contype="0" conaffinity="0"/>
  <geom name="backpack" type="box"     size="0.13 0.03 0.20" pos="0 -0.11 0.02" rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>
"""

# ══════════════════════════════════════════════════════════════════════════════
# SCENE BUILDER
# ══════════════════════════════════════════════════════════════════════════════
def prefix_element(el, prefix):
    NAME  = {"name","joint","body","body1","body2","site","tendon","actuator","sensor"}
    CLASS = {"class","childclass"}
    el    = copy.deepcopy(el)
    def r(e):
        for a in list(e.attrib):
            v = e.attrib[a]
            if a in NAME and not v.startswith(prefix):
                e.attrib[a] = prefix + v
            elif a in CLASS and v not in {"free","slide","hinge","ball","weld"} and not v.startswith(prefix):
                e.attrib[a] = prefix + v
        for c in e: r(c)
    r(el); return el

def sec(root, tag, px):
    el = root.find(tag)
    return "" if el is None else ET.tostring(prefix_element(el,px), encoding="unicode")

def wbx(root, px):
    wb = root.find("worldbody")
    return "" if wb is None else "".join(ET.tostring(prefix_element(c,px), encoding="unicode") for c in wb)

def build_xml():
    tree = ET.parse(GEN3_XML)
    root = tree.getroot()
    gdir = os.path.dirname(os.path.abspath(GEN3_XML))
    comp = root.find("compiler")
    mesh = os.path.join(gdir, comp.get("meshdir","assets") if comp else "assets").replace("\\","/")
    ast_ = root.find("asset")
    astr = ET.tostring(ast_, encoding="unicode") if ast_ is not None else "<asset/>"
    return f"""
<mujoco model="srl">
  <compiler meshdir="{mesh}" autolimits="true"/>
  <option gravity="0 0 0" timestep="0.002"/>
  <visual><headlight ambient="0.7 0.7 0.7" diffuse="0.9 0.9 0.9"/></visual>
  {astr}
  {sec(root,"default","k1_")} {sec(root,"default","k2_")}
  <worldbody>
    <light pos="0 -2 4" dir="0 0.4 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.42 0.42 0.42 1"/>
    <body name="human" pos="0 0 1.05">
      {HUMAN_XML}
      <!-- K1: right shoulder. Y180 flips arm down, Z90 points outward-right -->
      <!-- K1: right shoulder -->
      <body name="k1_root" pos="0.25 -0.06 0.18" euler="0 -90 0">{wbx(root,"k1_")}</body>
      <!-- K2: left shoulder — opposite X rotation to mirror -->
      <body name="k2_root" pos="-0.25 -0.06 0.18" euler="0 90 0">{wbx(root,"k2_")}</body>
    </body>
  </worldbody>
  {sec(root,"actuator","k1_")}{sec(root,"actuator","k2_")}
  {sec(root,"contact","k1_")}{sec(root,"contact","k2_")}
  {sec(root,"equality","k1_")}{sec(root,"equality","k2_")}
</mujoco>
"""

def build_qpos_maps(model):
    k1q, k2q = {}, {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        adr  = model.jnt_qposadr[i]
        for j in range(1, 8):
            if "k1_" in name and f"joint_{j}" in name: k1q[j-1] = adr
            elif "k2_" in name and f"joint_{j}" in name: k2q[j-1] = adr
    return k1q, k2q

# ══════════════════════════════════════════════════════════════════════════════
# SAFETY
# ══════════════════════════════════════════════════════════════════════════════
def check_soft_limits(arm, angles_deg):
    pass  # disabled

ZONE_TORSO_MIN = np.array([-0.22, -0.22, -0.13])
ZONE_TORSO_MAX = np.array([ 0.22,  0.22,  0.33])
ZONE_INTER_MIN = np.array([-0.08, -0.15,  0.15])
ZONE_INTER_MAX = np.array([ 0.08,  0.15,  0.45])

def check_zones(model, data, human_id):
    """
    Returns (head_hit, warnings).
    head_hit=True  -> hold last safe position (critical)
    warnings       -> list of non-critical zone strings (logged only)
    """
    if not ENABLE_ZONES:
        return False, []
    hp       = data.xpos[human_id]
    head_hit = False
    warnings = []
    for i in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
        if "k1_" not in name and "k2_" not in name:
            continue
        rel = data.xpos[i] - hp
        # Head — critical, causes rollback
        if np.linalg.norm(rel - ZONE_HEAD_CENTRE) < ZONE_HEAD_RADIUS + ZONE_MARGIN:
            head_hit = True
            print(f"  [ZONE-HEAD] {name} in head zone — holding")
        # Torso — warn only
        elif (np.all(rel >= ZONE_TORSO_MIN - ZONE_MARGIN) and
              np.all(rel <= ZONE_TORSO_MAX + ZONE_MARGIN)):
            warnings.append(f"{name}->TORSO")
        # Inter-arm — warn only
        elif (np.all(rel >= ZONE_INTER_MIN - ZONE_MARGIN) and
              np.all(rel <= ZONE_INTER_MAX + ZONE_MARGIN)):
            warnings.append(f"{name}->INTER")
    if warnings:
        pass  # zone warnings silenced — enable for debug
    return head_hit, warnings

# ══════════════════════════════════════════════════════════════════════════════
# KORTEX ARM
# ══════════════════════════════════════════════════════════════════════════════
def angle_diff(target, current):
    """Shortest signed difference, handles wrap-around."""
    return (target - current + 180.0) % 360.0 - 180.0

def compute_speeds(target_deg, current_deg):
    """
    P-controller: returns 7 joint speeds (deg/s) moving current toward target.
    target_deg/current_deg in internal -180..+180 representation.
    """
    speeds = [0.0] * NUM_JOINTS
    for j in range(NUM_JOINTS):
        err = angle_diff(target_deg[j], current_deg[j])
        if abs(err) > VEL_DEADBAND:
            speeds[j] = float(np.clip(VEL_KP * err, -VEL_MAX, VEL_MAX))
    return speeds


class KortexArm:
    def __init__(self, name, ip):
        self.name = name; self.connected = False; self._base = None
        try:
            self._tr = TCPTransport(); self._tr.connect(ip, ARM_PORT)
            self._rt = RouterClient(self._tr, lambda e: None)
            self._ss = SessionManager(self._rt)
            info = Session_pb2.CreateSessionInfo()
            info.username = ARM_USER; info.password = ARM_PASS
            info.session_inactivity_timeout = 60000
            info.connection_inactivity_timeout = 2000
            self._ss.CreateSession(info)
            self._base = BaseClient.BaseClient(self._rt)
            try:
                m = Base_pb2.ServoingModeInformation()
                m.servoing_mode = 2  # 2 = HIGH_LEVEL_SERVOING
                self._base.SetServoingMode(m)
            except Exception:
                pass  # high-level is default on bootup
            self.connected = True
            print(f"[{name}] Connected {ip}")
        except Exception as e:
            print(f"[{name}] Failed: {e}")

    def send_speeds(self, speeds_deg_s):
        """
        Velocity control — smooth & responsive (proven in test_kinova).
        speeds_deg_s: list of 7 joint velocities. Persists until next command.
        """
        if not self.connected or self._base is None:
            return
        cmd = Base_pb2.JointSpeeds()
        for i, s in enumerate(speeds_deg_s):
            js = cmd.joint_speeds.add()
            js.joint_identifier = i
            js.value = float(s)
        try:
            self._base.SendJointSpeedsCommand(cmd)
        except Exception as e:
            print(f"[{self.name}] speed cmd error: {e}")

    def read_joint_positions_fast(self):
        """Same as read_joint_positions but silent — for the control loop."""
        if not self.connected or self._base is None:
            return None
        try:
            fb = self._base.GetMeasuredJointAngles()
            angles = np.zeros(NUM_JOINTS)
            for ja in fb.joint_angles:
                idx = ja.joint_identifier
                if 0 <= idx < NUM_JOINTS:
                    raw = float(ja.value)
                    angles[idx] = raw - 360.0 if raw > 180.0 else raw
            return angles
        except Exception:
            return None

    def read_joint_positions(self):
        """
        Read current joint angles from the real arm (degrees).
        Returns numpy array of 7 angles, or None on failure.
        Called on startup so unconnected pots hold the arm's actual position.
        """
        if not self.connected or self._base is None:
            return None
        try:
            feedback = self._base.GetMeasuredJointAngles()
            angles = np.zeros(NUM_JOINTS)
            for ja in feedback.joint_angles:
                idx = ja.joint_identifier
                if 0 <= idx < NUM_JOINTS:
                    raw = float(ja.value)   # Kortex returns 0-360
                    # Convert to -180..+180 for internal use
                    angles[idx] = raw - 360.0 if raw > 180.0 else raw
            print(f"[{self.name}] Current joints: {np.round(angles, 1).tolist()}")
            return angles
        except Exception as e:
            print(f"[{self.name}] Read joints failed: {e}")
            return None

    def stop(self):
        if self.connected and self._base:
            try:
                self._base.SendJointSpeedsCommand(Base_pb2.JointSpeeds())  # zero all
                self._base.Stop()
            except Exception: pass

    def disconnect(self):
        try: self._ss.CloseSession(); self._tr.disconnect()
        except Exception: pass

# ══════════════════════════════════════════════════════════════════════════════
# WATCHDOG
# ══════════════════════════════════════════════════════════════════════════════
class Watchdog:
    def __init__(self, arms):
        self._arms = arms; self._t = time.time(); self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def beat(self):
        with self._lock: self._t = time.time()

    def _run(self):
        while True:
            time.sleep(0.5)
            with self._lock: elapsed = time.time() - self._t
            if elapsed > WATCHDOG_TIMEOUT_S:
                for a in self._arms: a.stop()

# ══════════════════════════════════════════════════════════════════════════════
# SERIAL THREAD
# ══════════════════════════════════════════════════════════════════════════════
def serial_thread(watchdog):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[Serial] opened {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        print(f"[Serial] FAILED: {e}")
        return

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line: continue

            new_k1 = [None]*NUM_JOINTS
            new_k2 = [None]*NUM_JOINTS
            got    = False

            for token in line.split(","):
                token = token.strip().lower()
                if len(token) < 5 or token[2] != "j" or ":" not in token: continue
                arm = token[:2]
                if arm not in ("k1","k2"): continue
                try:
                    js, vs = token[3:].split(":")
                    jidx   = int(js) - 1
                    raw    = float(vs)
                except ValueError: continue
                if not (0 <= jidx < NUM_JOINTS): continue
                deg = raw_to_joint(raw)
                if deg is None:
                    deg = (_HOME_K1_DEG if arm == "k1" else _HOME_K2_DEG)[jidx]
                if arm == "k1": new_k1[jidx] = deg
                else:           new_k2[jidx] = deg
                got = True

            if not got: continue

            watchdog.beat()

            MAX_DELTA = 10.0   # max degrees change per serial packet — prevents jumps
            with lock:
                for j in range(NUM_JOINTS):
                    # STRICT_HOLD: only joints with a connected pot may move
                    if new_k1[j] is not None and (not STRICT_HOLD or CONNECTED_K1[j]):
                        delta = new_k1[j] - k1_deg[j]
                        k1_deg[j] += float(np.clip(delta, -MAX_DELTA, MAX_DELTA))
                    if new_k2[j] is not None and (not STRICT_HOLD or CONNECTED_K2[j]):
                        delta = new_k2[j] - k2_deg[j]
                        k2_deg[j] += float(np.clip(delta, -MAX_DELTA, MAX_DELTA))

            # Soft limit warnings (non-blocking) - commented to reduce spam
            # check_soft_limits("K1", k1_deg)
            # check_soft_limits("K2", k2_deg)

            updated.set()
            # Uncomment below to debug serial values:
            # print(f"  K1={np.round(k1_deg,1).tolist()}")
            # print(f"  K2={np.round(k2_deg,1).tolist()}")

        except Exception as e:
            print(f"[Serial] error: {e}"); time.sleep(0.1)

# ══════════════════════════════════════════════════════════════════════════════
# HOMING
# ══════════════════════════════════════════════════════════════════════════════
def run_homing(model, data, k1q, k2q, viewer, arm_k1=None, arm_k2=None):
    home1 = np.array(_HOME_K1_DEG, dtype=float)
    home2 = np.array(_HOME_K2_DEG, dtype=float)
    steps = 120
    dt    = 1.0 / 60.0

    cur1 = np.array([np.rad2deg(data.qpos[k1q[j]]) if j in k1q else 0.0 for j in range(NUM_JOINTS)])
    cur2 = np.array([np.rad2deg(data.qpos[k2q[j]]) if j in k2q else 0.0 for j in range(NUM_JOINTS)])

    print(f"[HOME] K1 -> {home1.tolist()}")
    print(f"[HOME] K2 -> {home2.tolist()}")

    # Homing only animates the SIM. The real arm is driven by the velocity
    # control thread toward the shared target (k1_deg/k2_deg), which we set
    # gradually here so the real arm eases to home at the controlled speed.
    for s in range(steps + 1):
        alpha = s / steps
        t1    = cur1 + alpha * (home1 - cur1)
        t2    = cur2 + alpha * (home2 - cur2)

        for j in range(NUM_JOINTS):
            if j in k1q: data.qpos[k1q[j]] = np.deg2rad(t1[j])
            if j in k2q: data.qpos[k2q[j]] = np.deg2rad(t2[j])
        mujoco.mj_forward(model, data)
        viewer.sync()

        # Update shared target — velocity thread moves the real arm to match
        with lock:
            k1_deg[:] = t1
            k2_deg[:] = t2

        time.sleep(dt)

    with lock:
        k1_deg[:] = home1
        k2_deg[:] = home2

    print("[HOME] Done — accepting commands")

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
def launch_estop_window(arms):
    """
    Always-on-top red STOP button in a separate window.
    Click STOP (or press SPACE) to immediately halt all arms.
    """
    import tkinter as tk

    def trigger_stop(event=None):
        estop.set()
        for a in arms:
            if a: a.stop()
        status.config(text="STOPPED", bg="#440000", fg="#ff6666")
        print("\n*** EMERGENCY STOP ACTIVATED ***\n")

    def reset_stop():
        estop.clear()
        status.config(text="RUNNING", bg="#004400", fg="#66ff66")
        print("[E-STOP] Cleared")

    root = tk.Tk()
    root.title("E-STOP")
    root.geometry("260x200")
    root.configure(bg="#111111")
    root.attributes("-topmost", True)

    tk.Label(root, text="EMERGENCY STOP", font=("Segoe UI", 12, "bold"),
             bg="#111111", fg="#ffffff").pack(pady=(14, 4))
    tk.Button(root, text="STOP", font=("Segoe UI", 22, "bold"),
              bg="#cc0000", fg="white", activebackground="#ff0000",
              relief="flat", width=10, height=2, cursor="hand2",
              command=trigger_stop).pack(pady=6)
    status = tk.Label(root, text="RUNNING", font=("Segoe UI", 10, "bold"),
                      bg="#004400", fg="#66ff66", width=20)
    status.pack(pady=4)
    tk.Button(root, text="Reset", font=("Segoe UI", 9),
              bg="#333333", fg="white", relief="flat",
              command=reset_stop).pack(pady=2)
    root.bind("<space>", trigger_stop)
    root.mainloop()


def velocity_control_thread(arm_k1, arm_k2):
    """
    Dedicated high-rate loop for real arm velocity control.
    Runs independently of the sim render loop for smooth, responsive motion.
    """
    print("[VEL] Velocity control thread started")
    while True:
        if estop.is_set():
            if arm_k1 and arm_k1.connected: arm_k1.send_speeds([0.0]*NUM_JOINTS)
            if arm_k2 and arm_k2.connected: arm_k2.send_speeds([0.0]*NUM_JOINTS)
            time.sleep(0.05)
            continue

        with lock:
            t1 = k1_deg.copy()
            t2 = k2_deg.copy()

        if arm_k1 and arm_k1.connected:
            cur = arm_k1.read_joint_positions_fast()
            if cur is not None:
                arm_k1.send_speeds(compute_speeds(t1, cur))
        if arm_k2 and arm_k2.connected:
            cur = arm_k2.read_joint_positions_fast()
            if cur is not None:
                arm_k2.send_speeds(compute_speeds(t2, cur))

        time.sleep(0.005)   # ~200 Hz


def main():
    print("=" * 52)
    print("  SRL Teleoperation System")
    print(f"  Mode : {_MODE.upper()}")
    print(f"  Port : {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"  Home K1: {_HOME_K1_DEG}")
    print(f"  Home K2: {_HOME_K2_DEG}")
    print(f"  Zones: {'ON' if ENABLE_ZONES else 'OFF'}")
    print("=" * 52)

    # Real arms
    arm_k1 = arm_k2 = None
    if USE_REAL:
        arm_k1 = KortexArm("K1", ARM_K1_IP)
        arm_k2 = KortexArm("K2", ARM_K2_IP)

    watchdog = Watchdog([a for a in [arm_k1, arm_k2] if a and a.connected])

    # Seed unconnected joints from real arm positions so they don't snap to zero
    if USE_REAL:
        print("Reading current arm positions to seed unconnected joints...")
        if arm_k1 and arm_k1.connected:
            pos = arm_k1.read_joint_positions()
            if pos is not None:
                with lock:
                    for j in range(NUM_JOINTS):
                        if not CONNECTED_K1[j]:
                            k1_deg[j] = pos[j]
                print("  K1 unconnected joints seeded from real arm")
        if arm_k2 and arm_k2.connected:
            pos = arm_k2.read_joint_positions()
            if pos is not None:
                with lock:
                    for j in range(NUM_JOINTS):
                        if not CONNECTED_K2[j]:
                            k2_deg[j] = pos[j]
                print("  K2 unconnected joints seeded from real arm")
    else:
        with lock:
            for j in range(NUM_JOINTS):
                if not CONNECTED_K1[j]: k1_deg[j] = _HOME_K1_DEG[j]
                if not CONNECTED_K2[j]: k2_deg[j] = _HOME_K2_DEG[j]
        print("SIM_ONLY: unconnected joints seeded from home pose")

    threading.Thread(target=serial_thread, args=(watchdog,), daemon=True).start()

    # Launch emergency stop window (always on top)
    threading.Thread(target=launch_estop_window,
                     args=([arm_k1, arm_k2],), daemon=True).start()
    print("E-STOP window launched — click STOP or press SPACE to halt")

    # Launch real-arm velocity control thread (independent of sim)
    if USE_REAL:
        threading.Thread(target=velocity_control_thread,
                         args=(arm_k1, arm_k2), daemon=True).start()

    if not USE_SIM:
        print("REAL_ONLY — Ctrl+C to exit")
        try:
            while True:
                updated.wait(timeout=1.0); updated.clear()
                if estop.is_set():
                    continue
                with lock: t1, t2 = k1_deg.copy(), k2_deg.copy()
                if arm_k1 and arm_k1.connected:
                    cur = arm_k1.read_joint_positions_fast()
                    if cur is not None: arm_k1.send_speeds(compute_speeds(t1, cur))
                if arm_k2 and arm_k2.connected:
                    cur = arm_k2.read_joint_positions_fast()
                    if cur is not None: arm_k2.send_speeds(compute_speeds(t2, cur))
        except KeyboardInterrupt: pass
        finally:
            if arm_k1: arm_k1.stop(); arm_k1.disconnect()
            if arm_k2: arm_k2.stop(); arm_k2.disconnect()
        return

    # Build sim
    print("Building scene...")
    xml   = build_xml()
    model = mujoco.MjModel.from_xml_string(xml)
    data  = mujoco.MjData(model)
    k1q, k2q = build_qpos_maps(model)
    human_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "human")

    print(f"  K1 joints : {k1q}")
    print(f"  K2 joints : {k2q}")

    if not k1q or not k2q:
        print("ERROR: joint maps empty — check GEN3_XML path"); return

    last_safe_k1 = np.array(_HOME_K1_DEG, dtype=float)
    last_safe_k2 = np.array(_HOME_K2_DEG, dtype=float)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]        = False
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_NONE

        # Homing
        run_homing(model, data, k1q, k2q, viewer, arm_k1, arm_k2)

        while viewer.is_running():
            updated.wait(timeout=0.016)
            updated.clear()

            with lock:
                t1 = k1_deg.copy()
                t2 = k2_deg.copy()

            # Write to sim
            for j in range(NUM_JOINTS):
                if j in k1q: data.qpos[k1q[j]] = np.deg2rad(t1[j])
                if j in k2q: data.qpos[k2q[j]] = np.deg2rad(t2[j])
            mujoco.mj_forward(model, data)

            # Head zone check — hold last safe if violated
            if ENABLE_ZONES:
                head_hit, _ = check_zones(model, data, human_id)
                if head_hit:
                    for j in range(NUM_JOINTS):
                        if j in k1q: data.qpos[k1q[j]] = np.deg2rad(last_safe_k1[j])
                        if j in k2q: data.qpos[k2q[j]] = np.deg2rad(last_safe_k2[j])
                    mujoco.mj_forward(model, data)
                    with lock:
                        k1_deg[:] = last_safe_k1
                        k2_deg[:] = last_safe_k2
                else:
                    last_safe_k1 = t1.copy()
                    last_safe_k2 = t2.copy()

            viewer.sync()

            # (real arm control runs in its own thread — see velocity_control_thread)

    if arm_k1: arm_k1.stop(); arm_k1.disconnect()
    if arm_k2: arm_k2.stop(); arm_k2.disconnect()
    print("Shutdown complete")

if __name__ == "__main__":
    main()