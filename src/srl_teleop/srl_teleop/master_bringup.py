#!/usr/bin/env python3
"""
master_bringup.py -- the master-mannequin -> REAL ARMS sequence, in order.

    python3 -m srl_teleop.master_bringup --self-test
    python3 -m srl_teleop.master_bringup            # run it, printing each step

THE ORDER IS THE WHOLE THING, and it is here rather than in the GUI so that
it can be tested without Qt and read without scrolling through a window.
`master_mannequin_gui.py` calls `STEPS` and does nothing the list does not say.

THIS SEQUENCE IS WHAT ACTUALLY WORKED on 2026-09-01, after an evening of it
not working. Each step below exists because skipping it produced a specific
failure that looked like something else:

  1. STACK        teleop.launch.py with follower:=master and master:=false.
                  `master:=false` because step 2 owns the master node -- two
                  master_pose_nodes fight for /dev/ttyACM0 and both then read
                  corrupt frames.
  2. MASTER       master_pose_node with force_clutch_engaged:=true.
                  THE CLUTCH IS PINNED because this rig's button channels flip
                  TOGETHER: measured in the log, both arms disengaged 2 ms
                  apart, and re-engage then deferred on the quasi-static gate
                  until another spurious flip cancelled it. Pinning takes the
                  buttons out of the clutch path entirely.
  3. REAL         scripts/start_real.sh -- Kortex HIGH-LEVEL session, velocity
                  homing, sim->real bridge. NOT the cyclic ros2_control path:
                  HARD CONSTRAINT 4, and an evening of proof.
  4. SEED         command the SIM to the REAL arms' measured joint positions.
                  THIS IS THE STEP THAT WAS MISSING. The bridge refuses to
                  enable while sim and real are more than enable_gap_rad
                  (0.30) apart, and homing only ever moves the REAL side. The
                  master's resting pose maps ~32 deg from home, so the sim sat
                  there and the gap never closed -- "REFUSED: sim is 0.557 rad
                  from the real arm", repeatedly, with both sides individually
                  where they were supposed to be. Seeding the sim FROM
                  /real/joint_states closes it by construction.
  5. BRIDGE       enable both sim->real bridges. Now the gap is ~0.
  6. ARM          master_teleop motion_enabled:=true, LAST. Arming before
                  step 5 lets the sim run away from the real arm again and
                  step 4 has to be redone.

SPEED. `SPEEDS` below pins one invariant, and it is the one that cost an
e-stop: the COMMANDING side must be slower than the FOLLOWING side.
master_teleop drives the sim through Ruckig at up to its own vmax; the bridge
replays to the real arm at MAX_VEL. When Ruckig was left at the joint limit
(1.3963 rad/s) against a 0.40 rad/s bridge, the real arm fell behind
monotonically and the lag monitor tripped at 0.502 rad on joint_5. So
`teleop_vmax` is always strictly under `bridge_vmax`, and both are under the
joint limit.
"""
import argparse
import math
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ARMS = ("left", "right")
DOF = 7

#: The joint velocity limit from joint_limits.yaml, for reference. Nothing
#: here may exceed it and nothing here comes close.
JOINT_LIMIT_RAD_S = 1.3963

#: Named speed presets. `teleop_vmax` < `bridge_vmax` is not a preference, it
#: is the condition under which the lag monitor is a safety net rather than a
#: scheduled failure -- see the module docstring.
#:
#: `slew_m` is the Cartesian rate limit inside master_teleop, in metres per
#: master sample at 50 Hz. 0.006 was the measured smoothness knee on
#: capture_20260901_151026; the sweep showed 8-20 mm cost NO extra tracking
#: lag (p95 stayed 3.65-3.71 mm) and only raised the worst single step, so
#: going faster here is cheap and it is what "the arms feel slow" is asking
#: for.
SPEEDS = {
    "slow":   dict(teleop_vmax=0.25, bridge_vmax=0.40, slew_m=0.006),
    "normal": dict(teleop_vmax=0.35, bridge_vmax=0.60, slew_m=0.010),
    "fast":   dict(teleop_vmax=0.60, bridge_vmax=0.90, slew_m=0.016),
}
DEFAULT_SPEED = "normal"


def speed_profile(name):
    """Look up a preset, and REFUSE an unknown name rather than defaulting.

    A typo silently selecting a different speed on a real arm is exactly the
    class of fault this repo keeps paying for.
    """
    if name not in SPEEDS:
        raise ValueError(
            "unknown speed %r -- expected one of %s"
            % (name, ", ".join(sorted(SPEEDS))))
    p = dict(SPEEDS[name])
    check_speeds(p)
    return p


def check_speeds(p):
    """The invariant, checked rather than trusted. Returns None or raises."""
    if not (p["teleop_vmax"] < p["bridge_vmax"]):
        raise ValueError(
            "teleop_vmax %.3f must be STRICTLY under bridge_vmax %.3f: the "
            "commanding side outrunning the following side is what trips the "
            "lag monitor (measured 0.502 rad on joint_5, 2026-09-01)"
            % (p["teleop_vmax"], p["bridge_vmax"]))
    if not (p["bridge_vmax"] < JOINT_LIMIT_RAD_S):
        raise ValueError(
            "bridge_vmax %.3f is at or over the %.4f rad/s joint limit"
            % (p["bridge_vmax"], JOINT_LIMIT_RAD_S))
    if not (0.0 < p["slew_m"] <= 0.05):
        raise ValueError(
            "slew_m %.4f m/sample is outside 0 < s <= 0.05; at 50 Hz that "
            "would be over 2.5 m/s of commanded hand speed" % p["slew_m"])


# ===========================================================================
#  AUTO-FIXES
#
#  Every entry below is a fault this session actually hit, with the repair
#  that actually worked. They are here rather than in the GUI so the list can
#  be read and tested without Qt.
#
#  THE RULES, because an auto-fix that hides a fault is worse than no
#  auto-fix:
#    * each one is applied AT MOST ONCE per bring-up (`_applied`), so a fault
#      that keeps coming back surfaces as a failure instead of an infinite
#      repair loop;
#    * each one SAYS what it did, in the log, every time;
#    * none of them touches a real arm. The arms are moved only by homing and
#      by the operator;
#    * anything that cannot be repaired safely is REPORTED and not attempted
#      -- see `leaked_kortex_session`, which needs a human at the power
#      switch and says so rather than pretending.
# ===========================================================================

#: The Teensy's USB identity. The BUSID IS NOT STABLE -- it moved from 2-9 to
#: 2-6 across a single detach/attach on 2026-09-01 -- so everything below
#: looks the device up by VID:PID and never remembers a number.
TEENSY_VIDPID = "16c0:0483"

#: The arm addresses on this rig, and real_arms_highlevel.launch.py's own
#: defaults. Exposed in the GUI because an arm that has been re-addressed is
#: otherwise a five-minute bring-up that fails at the last step.
DEFAULT_LEFT_IP = "192.168.1.10"
DEFAULT_RIGHT_IP = "192.168.1.9"


def ping(host, timeout_s=2):
    """Is the arm reachable at all? Reachable is not connected, but
    unreachable cannot be connected, and knowing which is one second."""
    try:
        r = subprocess.run(["ping", "-c", "1", "-W", str(int(timeout_s)), host],
                           capture_output=True, timeout=timeout_s + 3)
        return r.returncode == 0
    except Exception:                                        # noqa: BLE001
        return False

USBIPD = "/mnt/c/Program Files/usbipd-win/usbipd.exe"


def usbipd_available():
    return os.path.exists(USBIPD)


def teensy_busid():
    """The Teensy's CURRENT busid on the Windows side, or None.

    Parsed from `usbipd list` by VID:PID. Never cached: see TEENSY_VIDPID.
    """
    if not usbipd_available():
        return None
    try:
        out = subprocess.run([USBIPD, "list"], capture_output=True, text=True,
                             timeout=25).stdout
    except Exception:                                        # noqa: BLE001
        return None
    return parse_busid(out, TEENSY_VIDPID)


def parse_busid(listing, vidpid):
    """Pure: pull the busid for `vidpid` out of `usbipd list` output.

    Split out from the subprocess call so the self-test can drive it with
    recorded output instead of hardware.
    """
    for line in listing.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1].lower() == vidpid.lower():
            return parts[0]
    return None


def teensy_port():
    """The serial device, or None. /dev/ttyACM* varies with enumeration."""
    import glob
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pat))
        if found:
            return found[0]
    return None


def attach_teensy(timeout_s=20.0):
    """Attach the Teensy to WSL. Returns (ok, why)."""
    bus = teensy_busid()
    if bus is None:
        return False, ("no %s device on the Windows side -- is the Teensy "
                       "plugged in at all?" % TEENSY_VIDPID)
    try:
        r = subprocess.run([USBIPD, "attach", "--wsl", "--busid", bus],
                           capture_output=True, text=True, timeout=40)
    except Exception as exc:                                 # noqa: BLE001
        return False, str(exc)
    if not wait_for(lambda: teensy_port() is not None, timeout_s):
        return False, ("usbipd attach on busid %s returned but no /dev/ttyACM* "
                       "appeared: %s" % (bus, (r.stderr or r.stdout).strip()[:160]))
    return True, "attached busid %s -> %s" % (bus, teensy_port())


def detach_teensy():
    bus = teensy_busid()
    if bus is None:
        return False, "no %s device to detach" % TEENSY_VIDPID
    try:
        subprocess.run([USBIPD, "detach", "--busid", bus],
                       capture_output=True, text=True, timeout=40)
    except Exception as exc:                                 # noqa: BLE001
        return False, str(exc)
    wait_for(lambda: teensy_port() is None, 15.0)
    return True, "detached busid %s" % bus


def power_cycle_teensy():
    """Detach and reattach: the only reset available from this side.

    THIS IS WHAT REVIVED THE DEAD IMUs on 2026-09-01. Both K1IMU and K2IMU
    were sending 0.000 for accel AND gyro -- the I2C bus had not come up --
    and master_pose_node cannot publish a pose without a gravity vector, so
    nothing downstream had any input at all. A detach/attach power-cycles the
    board and the IMUs re-init on boot. The busid changes across the cycle,
    which is why the reattach looks it up again.
    """
    detach_teensy()
    time.sleep(2.0)
    return attach_teensy()


def read_master_frames(seconds=4.0):
    """Raw Teensy lines, straight off the wire. REQUIRES THE PORT FREE.

    master_pose_node claims the port exclusively, so this only runs before
    step 2. It is the only ground truth about the IMUs: the ROS topics carry
    nothing at all when the frames fail validation, which reads identically
    to a dead stack.
    """
    port = teensy_port()
    if port is None:
        return []
    try:
        import serial
    except ImportError:
        return []
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception:                                        # noqa: BLE001
        return []
    try:
        time.sleep(0.5)
        ser.reset_input_buffer()
        lines, t0 = [], time.monotonic()
        while time.monotonic() - t0 < seconds:
            ln = ser.readline().decode("ascii", "replace").strip()
            if ln:
                lines.append(ln)
        return lines
    finally:
        try:
            ser.close()
        except Exception:                                    # noqa: BLE001
            pass


def imu_is_dead(lines):
    """Pure: are BOTH IMUs reporting all zeros?

    A resting accelerometer reads ~1 g of gravity. Exactly 0.000 on every
    axis of both IMUs is not a still arm, it is a bus that never came up.
    Returns None when the frames do not carry IMU fields at all, which is a
    different fault and must not be reported as this one.
    """
    seen = 0
    for ln in lines:
        if "K1IMU:" not in ln or "K2IMU:" not in ln:
            continue
        seen += 1
        for tag in ("K1IMU:", "K2IMU:"):
            body = ln.split(tag, 1)[1]
            vals = body.split(",")[:6]
            try:
                nums = [abs(float(v)) for v in vals]
            except ValueError:
                return None
            if any(n > 1e-6 for n in nums):
                return False
    if seen == 0:
        return None
    return True


#: How many /dev/shm Fast DDS segments count as "about to run out". A healthy
#: idle rig sits near 16; this session reached 469 before every new
#: participant started failing open_and_lock_file. 250 is comfortably above
#: normal operation and well below the failure point.
SHM_WARN_AT = 250


def stale_shm_count():
    try:
        import glob
        return len(glob.glob("/dev/shm/fastrtps_*")) + \
            len(glob.glob("/dev/shm/sem.fastrtps_*"))
    except Exception:                                        # noqa: BLE001
        return 0


def stack_is_up():
    """Any Fast DDS participant of ours still running?

    Clearing /dev/shm under a LIVE stack orphans that stack's own segments and
    is how a healthy rig was made deaf on 2026-08-20. So the sweep is gated on
    this, and the gate is the point.
    """
    try:
        r = subprocess.run(
            ["pgrep", "-f",
             "opt/ros/jazzy/lib/(moveit_ros_move_group|controller_manager|"
             "robot_state_publisher)|install/srl_(teleop|vr_teleop|"
             "experiments|autonomy|perception)/lib|kortex_highlevel_bridge"],
            capture_output=True, text=True, timeout=15)
        return bool(r.stdout.strip())
    except Exception:                                        # noqa: BLE001
        return True          # unknown means "do not sweep"


def clear_stale_shm():
    """Sweep Fast DDS segments. REFUSES while the stack is up."""
    if stack_is_up():
        return False, "stack is running -- NOT sweeping /dev/shm"
    n = stale_shm_count()
    if n == 0:
        return True, "no stale segments"
    subprocess.run(["bash", "-lc",
                    "source scripts/env.sh >/dev/null 2>&1; "
                    "srl_clear_stale_shm --force"],
                   cwd=WS, capture_output=True, text=True, timeout=90)
    return True, "cleared %d stale Fast DDS segment(s)" % n


#: The variables scripts/env.sh exists to pin. A process missing these joins
#: a DIFFERENT bus and sees nothing, which is indistinguishable from a dead
#: stack -- CLAUDE.md HARD CONSTRAINT 5 and the whole reason env.sh exists.
ENV_KEYS = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
            "FASTDDS_BUILTIN_TRANSPORTS", "ROS_AUTOMATIC_DISCOVERY_RANGE")


def ensure_env():
    """Make THIS process's environment the one scripts/env.sh defines.

    Called at start-up so the window cannot be launched wrong. Everything the
    GUI spawns inherits os.environ, so a GUI started from a bare shell would
    hand every node UDP discovery -- which is dead on this host -- and the
    stack would come up healthy and invisible. Rather than telling the
    operator to remember a `source`, this reads env.sh's own output and adopts
    it, and REPORTS what it changed so a surprise is never silent.

    Returns (changed, description).
    """
    missing = [k for k in ENV_KEYS if not os.environ.get(k)]
    if not missing and os.environ.get("FASTDDS_BUILTIN_TRANSPORTS") == "SHM":
        return False, "environment already correct (%s)" % ", ".join(
            "%s=%s" % (k, os.environ[k]) for k in ENV_KEYS)
    try:
        r = subprocess.run(
            ["bash", "-lc",
             "source scripts/env.sh >/dev/null 2>&1; env -0"],
            cwd=WS, capture_output=True, timeout=60)
        got = {}
        for chunk in r.stdout.split(b"\0"):
            if b"=" in chunk:
                k, _, v = chunk.partition(b"=")
                got[k.decode()] = v.decode()
    except Exception as exc:                                 # noqa: BLE001
        return False, "could not read scripts/env.sh: %s" % exc
    changed = []
    for k in ENV_KEYS:
        if k in got and os.environ.get(k) != got[k]:
            os.environ[k] = got[k]
            changed.append("%s=%s" % (k, got[k]))
    # The workspace overlay too, or `ros2 run srl_teleop ...` finds nothing.
    for k in ("AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "PATH"):
        if k in got:
            os.environ[k] = got[k]
    if not changed:
        return False, "environment already correct"
    return True, "adopted scripts/env.sh: " + ", ".join(changed)


def running_stack_pids():
    """PIDs of an ALREADY-RUNNING srl_teleop stack, newest launch included."""
    try:
        r = subprocess.run(
            ["pgrep", "-f",
             "ros2 launch srl_teleop teleop.launch.py|"
             "install/srl_teleop/lib|"
             "opt/ros/jazzy/lib/(moveit_ros_move_group|controller_manager)"],
            capture_output=True, text=True, timeout=15)
        return [int(x) for x in r.stdout.split()]
    except Exception:                                        # noqa: BLE001
        return []


def stop_existing_stack(timeout_s=30.0):
    """SIGINT any stack already running, so exactly ONE is launched.

    HARD CONSTRAINT 3, and it is what broke this GUI's first real runs.
    Pressing the button while a stack was up launched a SECOND one; by the
    third attempt start_real.sh reported nine pids across three stacks.
    Discovery then partitions and every daemon-backed query comes back empty.
    """
    pids = running_stack_pids()
    if not pids:
        return True, "no stack was running"
    for pid in pids:
        try:
            os.kill(pid, 2)                                  # SIGINT
        except OSError:
            pass
    wait_for(lambda: not running_stack_pids(), timeout_s)
    for pid in running_stack_pids():
        try:
            os.kill(pid, 9)
        except OSError:
            pass
    wait_for(lambda: not running_stack_pids(), 8.0)
    return True, "stopped %d process(es) of an existing stack" % len(pids)


def daemon_sees_graph(timeout_s=25):
    """Does the DAEMON-BACKED query return anything? The tiebreaker.

    `ros2 node list` returning nothing is indistinguishable from an idle
    graph unless you also ask without the daemon, which is why both are here.
    """
    try:
        r = subprocess.run(
            ["bash", "-lc",
             "source scripts/env.sh >/dev/null 2>&1; "
             "timeout %d ros2 node list 2>/dev/null | wc -l" % timeout_s],
            cwd=WS, capture_output=True, text=True, timeout=timeout_s + 15)
        return int((r.stdout or "0").strip() or 0)
    except Exception:                                        # noqa: BLE001
        return 0


def reset_daemon(verify=True):
    """Hard reset of the ros2 daemon, and CHECK that it worked.

    THE PATTERN IS BRACKETED AND THE PID KILL IS EXPLICIT. The first version
    of this ran `pkill -f 'ros2cli.daemon'` inside `bash -lc ...`, and that
    command line CONTAINS the string it is matching -- so pkill killed its own
    shell and `ros2 daemon start` never ran. The daemon stayed dead, every
    daemon-backed query came back empty, and start_real.sh refused three
    times with "DISCOVERY PROBLEM, not a missing stack". The diagnosis was
    right; the repair had silently not happened.

    Measured on the rig: daemon-backed `ros2 node list` returned 0 while
    `--no-daemon` returned 30, and a correct restart returned 30 immediately.

    `verify` re-queries afterwards, because a repair that reports success
    without checking is how the above went unnoticed for three attempts.
    """
    subprocess.run(
        ["bash", "-lc",
         "source scripts/env.sh >/dev/null 2>&1; "
         "timeout 15 ros2 daemon stop >/dev/null 2>&1; "
         "for P in $(pgrep -f '[r]os2cli.daemon'); do kill -9 $P; done; "
         "for i in $(seq 1 20); do "
         "  pgrep -f '[r]os2cli.daemon' >/dev/null || break; sleep 0.5; done; "
         "timeout 30 ros2 daemon start >/dev/null 2>&1; "
         "for i in $(seq 1 20); do "
         "  pgrep -f '[r]os2cli.daemon' >/dev/null && break; sleep 0.5; done"],
        cwd=WS, capture_output=True, text=True, timeout=120)
    if not verify:
        return True, "ros2 daemon restarted"
    n = daemon_sees_graph()
    if n > 0:
        return True, "ros2 daemon restarted -- it sees %d node(s)" % n
    # Nothing running is a legitimate zero; only call it a failure when a
    # stack IS up and the daemon still cannot see it.
    if not running_stack_pids():
        return True, "ros2 daemon restarted (no stack running yet)"
    return False, ("daemon restarted but STILL sees no nodes while a stack is "
                   "running -- this is the discovery partition, not a missing "
                   "stack; try: ros2 daemon stop && ros2 daemon start")


def kortex_session_leaked():
    """Is a Kortex bridge still holding a session from a previous run?

    NOT auto-repaired. The arm permits exactly one session and a leaked one is
    cleared by closing it properly or power-cycling the arm -- neither of
    which should happen without a person deciding. Reported so the operator is
    not left reading "connection refused" and guessing.
    """
    try:
        r = subprocess.run(["pgrep", "-f", "kortex_highlevel_bridge"],
                           capture_output=True, text=True, timeout=15)
        return bool(r.stdout.strip())
    except Exception:                                        # noqa: BLE001
        return False


#: (key, description, detect, fix, auto)
#: `auto=False` means DETECT AND REPORT ONLY -- see kortex_session_leaked.
AUTOFIXES = [
    ("daemon", "wedged ros2 daemon",
     None, reset_daemon, True),
    ("shm", "stale Fast DDS segments partitioning discovery",
     lambda: (not stack_is_up()) and stale_shm_count() > 0,
     clear_stale_shm, True),
    # BEFORE ANYTHING ELSE. A second stack is not a degraded stack, it is a
    # partitioned graph, and everything downstream then reports a discovery
    # fault instead of the duplication that caused it.
    ("dup_stack", "a stack is ALREADY running (a second one partitions "
                  "discovery -- HARD CONSTRAINT 3)",
     lambda: bool(running_stack_pids()), stop_existing_stack, True),
    ("teensy_absent", "master arm (Teensy) not attached to WSL",
     lambda: teensy_port() is None, attach_teensy, True),
    ("teensy_imu", "master arm IMUs reading all zeros",
     lambda: imu_is_dead(read_master_frames(3.0)) is True,
     power_cycle_teensy, True),
    ("kortex_leak", "a Kortex session is still open from a previous run",
     kortex_session_leaked, None, False),
    # SEGMENT EXHAUSTION WITH THE STACK UP. The sweep above is gated on the
    # stack being down and, when the gate holds, it says NOTHING -- which is
    # how 469 segments accumulated overnight and every new participant then
    # failed with
    #     [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portNNNN:
    #     open_and_lock_file failed
    # while the stack itself kept running and looked fine. A silent refusal
    # is the wrong answer to a condition the operator can fix in one command,
    # so this entry exists purely to SAY SO. It is not auto-repaired for the
    # same reason the sweep is gated: clearing under a live stack orphans
    # that stack's own segments.
    ("shm_exhausted",
     ("%d Fast DDS segments with the stack UP -- new nodes will fail to open "
      "ports. Stop the stack and press the button again, or run: "
      "source scripts/env.sh && srl_clear_stale_shm" % SHM_WARN_AT),
     lambda: stack_is_up() and stale_shm_count() >= SHM_WARN_AT,
     None, False),
]


class AutoFixer:
    """Applies each fix at most once and says what it did."""

    def __init__(self, log=None):
        self.log = log or (lambda s: None)
        self._applied = set()
        self.applied = []

    def run(self, keys=None, only_if_needed=True):
        """Returns a list of (key, ok, message) for what it touched."""
        out = []
        for key, desc, detect, fix, auto in AUTOFIXES:
            if keys is not None and key not in keys:
                continue
            if key in self._applied:
                continue
            needed = True
            if only_if_needed and detect is not None:
                try:
                    needed = bool(detect())
                except Exception as exc:                     # noqa: BLE001
                    self.log("  autofix %s: detector failed (%s)" % (key, exc))
                    needed = False
            if not needed:
                continue
            if not auto or fix is None:
                self.log("  NOTE: %s -- not repaired automatically" % desc)
                out.append((key, False, desc))
                continue
            self._applied.add(key)
            self.log("  AUTO-FIX: %s" % desc)
            try:
                ok, why = fix()
            except Exception as exc:                         # noqa: BLE001
                ok, why = False, str(exc)
            self.log("    -> %s%s" % ("ok" if ok else "FAILED",
                                      (": " + why) if why else ""))
            self.applied.append((key, ok, why))
            out.append((key, ok, why))
        return out


# ---------------------------------------------------------------- the steps
#: (key, human title, why it is here). The GUI renders this list; the runner
#: below executes it. One list, so a step cannot be shown and not run.
STEPS = [
    ("stack",  "simulation stack",
     "teleop.launch.py follower:=master master:=false"),
    ("master", "master arm",
     "master_pose_node, clutch PINNED (this rig's buttons flip together)"),
    ("real",   "real arms",
     "start_real.sh -- Kortex high-level session, homing, bridge"),
    ("seed",   "seed sim from real",
     "the step whose absence made the bridge refuse for an hour"),
    ("bridge", "enable sim->real",
     "both bridges; the gap is ~0 once seeded"),
    ("arm",    "arm the teleop",
     "motion_enabled:=true, LAST"),
]


def _env():
    e = dict(os.environ)
    e.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
    return e


def spawn(cmd, log_path, env=None):
    """Start a long-lived process detached, with its output on disk."""
    fh = open(log_path, "w")
    return subprocess.Popen(cmd, cwd=WS, stdout=fh, stderr=subprocess.STDOUT,
                            env=env or _env(), start_new_session=True)


def wait_for(predicate, timeout_s, poll_s=0.25):
    """True as soon as `predicate()` is true, False on timeout."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        try:
            if predicate():
                return True
        except Exception:                                    # noqa: BLE001
            pass
        time.sleep(poll_s)
    return False


def log_says(path, needle):
    """Is `needle` in the file yet? Missing file is simply 'not yet'."""
    try:
        with open(path, "r", errors="replace") as fh:
            return needle in fh.read()
    except OSError:
        return False


# ------------------------------------------------------------- ROS helpers
# Imported lazily so `--self-test` runs with no ROS on the path.

def _ros():
    import rclpy
    from rclpy.node import Node
    return rclpy, Node


def set_bool_param(node_name, param, value, timeout_s=10.0):
    """Set one bool parameter through the node's OWN service.

    NOT `ros2 param set`: that spawns a CLI process with its own discovery,
    and on this host it hangs for its full timeout often enough that the
    2026-09-01 session lost an hour to it.
    """
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_param")
    try:
        cli = n.create_client(SetParameters, "%s/set_parameters" % node_name)
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, "no %s/set_parameters" % node_name
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param, value=pv)]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
        res = fut.result()
        if res is None:
            return False, "%s did not answer" % node_name
        ok = bool(res.results and res.results[0].successful)
        return ok, ("" if ok else (res.results[0].reason if res.results
                                   else "no result"))
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def seed_sim_from_real(timeout_s=15.0):
    """Command the SIM arms to the REAL arms' measured joint positions.

    THE STEP THAT WAS MISSING. See the module docstring: homing moves only the
    real side, so the bridge's enable gap never closed and it refused with a
    number that looked like a fault when both sides were individually fine.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_seed")
    try:
        real = {}
        n.create_subscription(
            JointState, "/real/joint_states",
            lambda m: real.update(dict(zip(m.name, m.position))), 20)
        pubs = {a: n.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 10)
            for a in ARMS}
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(n, timeout_sec=0.1)
            if all("%s_joint_1" % a in real for a in ARMS):
                break
        sent = []
        for a in ARMS:
            names = ["%s_joint_%d" % (a, i) for i in range(1, DOF + 1)]
            pos = [real.get(nm) for nm in names]
            if any(p is None for p in pos):
                continue
            t = JointTrajectory()
            t.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start = Duration(sec=4, nanosec=0)
            t.points = [pt]
            pubs[a].publish(t)
            sent.append(a)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if not sent:
            return False, "no /real/joint_states -- are the real arms up?"
        return True, "seeded %s from the real arms" % ", ".join(sent)
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def enable_bridges(timeout_s=12.0):
    """Enable both sim->real bridges. Reports each arm's own answer."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_bridge")
    try:
        out, all_ok = [], True
        for a in ARMS:
            cli = n.create_client(Trigger, "/bridge_enable_%s" % a)
            if not cli.wait_for_service(timeout_sec=timeout_s):
                out.append("%s: no /bridge_enable_%s" % (a, a))
                all_ok = False
                continue
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
            r = fut.result()
            if r is None:
                out.append("%s: no answer" % a)
                all_ok = False
            else:
                out.append("%s: %s" % (a, r.message.strip()[:120]))
                all_ok = all_ok and bool(r.success)
        return all_ok, " | ".join(out)
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def reset_estop(timeout_s=10.0):
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_estop")
    try:
        cli = n.create_client(Trigger, "/estop_reset")
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, "no /estop_reset"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "no answer")
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


# ===========================================================================
#  SELF-TEST. Pure logic only -- no ROS, no hardware, no Qt. Every check is
#  one that can fail: the speed invariant is asserted against deliberately
#  broken profiles, not just the shipped ones.
# ===========================================================================
def self_test(verbose=True):
    fails = []

    def check(name, cond, detail=""):
        if verbose:
            print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
            if not cond and detail:
                print("      %s" % detail)
        if not cond:
            fails.append(name)

    print("speed profiles")
    for name in sorted(SPEEDS):
        p = SPEEDS[name]
        check("%-6s commanding side is slower than following side" % name,
              p["teleop_vmax"] < p["bridge_vmax"],
              "teleop %.2f vs bridge %.2f" % (p["teleop_vmax"],
                                              p["bridge_vmax"]))
        check("%-6s stays under the %.3f rad/s joint limit"
              % (name, JOINT_LIMIT_RAD_S),
              p["bridge_vmax"] < JOINT_LIMIT_RAD_S)
    check("'fast' is actually faster than 'slow'",
          SPEEDS["fast"]["teleop_vmax"] > SPEEDS["slow"]["teleop_vmax"]
          and SPEEDS["fast"]["slew_m"] > SPEEDS["slow"]["slew_m"])
    check("the default profile exists", DEFAULT_SPEED in SPEEDS)

    # THE INVARIANT MUST BITE. A checker that cannot fail is not a checker.
    try:
        check_speeds(dict(teleop_vmax=0.9, bridge_vmax=0.4, slew_m=0.01))
        check("an inverted profile is REFUSED", False,
              "check_speeds accepted teleop 0.9 > bridge 0.4")
    except ValueError:
        check("an inverted profile is REFUSED", True)
    try:
        check_speeds(dict(teleop_vmax=0.5, bridge_vmax=2.0, slew_m=0.01))
        check("a profile over the joint limit is REFUSED", False)
    except ValueError:
        check("a profile over the joint limit is REFUSED", True)
    try:
        check_speeds(dict(teleop_vmax=0.2, bridge_vmax=0.4, slew_m=0.5))
        check("an absurd slew is REFUSED", False)
    except ValueError:
        check("an absurd slew is REFUSED", True)
    try:
        speed_profile("quick")
        check("an unknown speed name is REFUSED, not defaulted", False)
    except ValueError:
        check("an unknown speed name is REFUSED, not defaulted", True)

    print("step list")
    keys = [k for k, _, _ in STEPS]
    check("every step has a unique key", len(keys) == len(set(keys)))
    check("the sim is seeded BEFORE the bridge is enabled",
          keys.index("seed") < keys.index("bridge"),
          "seeding after enabling is the ordering that refused all evening")
    check("the teleop is armed LAST", keys[-1] == "arm")
    check("the master node comes up after the stack",
          keys.index("stack") < keys.index("master"))
    check("the real arms come up before the seed",
          keys.index("real") < keys.index("seed"))

    print("auto-fixes")
    LISTING = """Connected:
BUSID  VID:PID    DEVICE                                          STATE
1-4    0b95:1790  ASIX USB to Gigabit Ethernet Family Adapter      Not shared
2-6    16c0:0483  USB Serial Device (COM3)                         Shared
2-9    046d:c52b  Logitech USB Input Device                        Not shared
"""
    check("the Teensy busid is found by VID:PID, not remembered",
          parse_busid(LISTING, TEENSY_VIDPID) == "2-6",
          "got %r" % parse_busid(LISTING, TEENSY_VIDPID))
    check("a listing without the Teensy yields None",
          parse_busid(LISTING.replace("16c0:0483", "1234:5678"),
                      TEENSY_VIDPID) is None)
    # The busid MOVED on 2026-09-01; a hardcoded one is the bug this prevents.
    MOVED = LISTING.replace("2-6    16c0:0483", "2-9    16c0:0483").replace(
        "2-9    046d:c52b", "2-6    046d:c52b")
    check("and it follows the device when the busid moves",
          parse_busid(MOVED, TEENSY_VIDPID) == "2-9")

    DEAD = ("k1j1:153.9,k1j2:343.9,K1IMU:0.000,0.000,0.000,0.00,0.00,0.00,"
            "K2IMU:0.000,0.000,0.000,0.00,0.00,0.00,fsr1:7.9,btn1:0")
    ALIVE = ("k1j1:125.4,k1j2:343.7,K1IMU:-0.902,0.024,0.292,2.53,1.54,1.79,"
             "K2IMU:-0.919,0.103,0.288,-1.21,-0.18,-0.64,fsr1:7.9,btn1:0")
    check("all-zero IMUs on both arms is detected as DEAD",
          imu_is_dead([DEAD] * 5) is True)
    check("a real gravity vector is NOT called dead",
          imu_is_dead([ALIVE] * 5) is False)
    check("one live arm is enough to not call it dead",
          imu_is_dead([DEAD, ALIVE]) is False)
    check("frames without IMU fields are UNKNOWN, not dead",
          imu_is_dead(["k1j1:153.9,fsr1:7.9"]) is None,
          "a different fault must not be reported as this one")
    check("no frames at all is UNKNOWN, not dead",
          imu_is_dead([]) is None)

    keys = [k for k, _, _, _, _ in AUTOFIXES]
    check("every auto-fix has a unique key", len(keys) == len(set(keys)))
    check("the leaked Kortex session is REPORTED, never auto-repaired",
          not [a for k, _d, _det, _f, a in AUTOFIXES if k == "kortex_leak"][0],
          "it needs a person at the power switch")
    check("segment exhaustion under a LIVE stack is reported, not silent",
          "shm_exhausted" in [k for k, _d, _det, _f, _a in AUTOFIXES],
          "a silent refusal is how 469 segments accumulated overnight")
    check("and it is reported rather than swept under a live stack",
          not [a for k, _d, _det, _f, a in AUTOFIXES
               if k == "shm_exhausted"][0])
    check("the warning threshold is above normal and below the failure point",
          16 < SHM_WARN_AT < 469, "SHM_WARN_AT=%d" % SHM_WARN_AT)
    check("the SHM sweep is gated on the stack being down",
          [det for k, _d, det, _f, _a in AUTOFIXES
           if k == "shm"][0] is not None)

    class _Rec:
        def __init__(self): self.lines = []
        def __call__(self, s): self.lines.append(s)

    # AT MOST ONCE. A fix that keeps re-firing turns a fault into a loop.
    calls = {"n": 0}

    def _always():
        return True

    def _count():
        calls["n"] += 1
        return True, "fixed"
    saved = list(AUTOFIXES)
    try:
        AUTOFIXES[:] = [("t", "test fault", _always, _count, True)]
        rec = _Rec()
        f = AutoFixer(rec)
        f.run(); f.run(); f.run()
        check("an auto-fix is applied at most once per bring-up",
              calls["n"] == 1, "ran %d times" % calls["n"])
        check("and it says what it did", any("AUTO-FIX" in x for x in rec.lines))

        # A detector that raises must not take the bring-up down with it.
        def _boom():
            raise RuntimeError("detector exploded")
        AUTOFIXES[:] = [("b", "boom", _boom, _count, True)]
        rec2 = _Rec()
        AutoFixer(rec2).run()
        check("a detector that raises is survived and reported",
              any("detector failed" in x for x in rec2.lines))
    finally:
        AUTOFIXES[:] = saved

    print("helpers")
    check("log_says on a missing file is 'not yet', not an error",
          log_says("/nonexistent/nope.log", "x") is False)
    check("wait_for returns False on timeout",
          wait_for(lambda: False, 0.3, 0.05) is False)
    check("wait_for returns True as soon as the predicate holds",
          wait_for(lambda: True, 1.0, 0.05) is True)
    check("wait_for survives a predicate that raises",
          wait_for(lambda: (_ for _ in ()).throw(RuntimeError("x")),
                   0.3, 0.05) is False)

    print()
    if fails:
        print("SELF-TEST FAILED: %d" % len(fails))
        for f in fails:
            print("   - %s" % f)
        return 1
    print("SELF-TEST PASSED")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--speed", default=DEFAULT_SPEED, choices=sorted(SPEEDS))
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    p = speed_profile(a.speed)
    print("master mannequin bring-up, speed=%s" % a.speed)
    print("  teleop vmax %.2f rad/s < bridge vmax %.2f rad/s, slew %.0f mm"
          % (p["teleop_vmax"], p["bridge_vmax"], p["slew_m"] * 1000))
    for i, (k, title, why) in enumerate(STEPS, 1):
        print("  %d. %-22s %s" % (i, title, why))
    print()
    print("Run it from the GUI: python3 scripts/master_mannequin_gui.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
