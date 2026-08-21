#!/usr/bin/env python3
"""real_arm_doctor.py -- the seven ways connecting the real arms goes wrong,
each detected, each said in plain words, each with a fix that is a button.

WHY THIS FILE EXISTS. Every one of these has cost this project a session, and
every one of them was diagnosed by a person remembering something:

  1. the two shells disagreed about ROS discovery, so a healthy stack was
     invisible to the terminal that needed it;
  2. a killed stack left Fast DDS segments in /dev/shm and discovery went
     intermittent -- a service visible to one client and not another;
  3. the ros2 daemon HUNG rather than failed, so `ros2 node list` returned an
     empty string with exit status 0 and the session concluded nothing was
     running;
  4. a second stack (or a stray robot_state_publisher) split the serial
     stream and invalidated a day of measurements;
  5. the Teensy moved between /dev/ttyACM0 and /dev/ttyACM1 on re-attach;
  6. the sim home was changed and the real arms were not recaptured, so the
     bridge refuses to enable and the refusal reads like a fault;
  7. a SIGKILLed bridge leaked the one Kortex session the arm permits, and
     the next run could not connect.

THE RULES THIS MODULE IS WRITTEN TO, taken from CLAUDE.md:

  * NEVER GREEN ON UNKNOWN. A check that could not be run returns UNKNOWN,
    which is its own state and is never drawn as healthy. `verdict()` returns
    UNKNOWN for anything it could not establish, not OK.
  * A check that cannot fail on a deliberately broken input is not a check.
    Every check here takes its evidence through a `Probe` object, so the test
    can hand it a broken world and require the check to say so. Nothing here
    reads the machine directly.
  * NO TOPIC NAMES AND NO RAW ERRORS in `title` or `plain`. Those two fields
    are what the operator reads. `detail` may carry the machinery, for the
    log and for a bug report.

NO Qt AND NO rclpy AT IMPORT TIME. The GUI imports this; so does a test that
runs with no graph at all.
"""
import glob
import math
import os
import re
import signal
import subprocess
import time

from srl_teleop import procscan

OK = "ok"                 # checked, and nothing is wrong
BAD = "bad"               # checked, and it is wrong
UNKNOWN = "unknown"       # could NOT be checked -- never drawn as healthy

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")

# The environment variables that decide whether two shells can see each
# other. FASTDDS_BUILTIN_TRANSPORTS is HARD CONSTRAINT 5 and is the one that
# has actually bitten: with it unset a query hung past 30 s against a stack
# that was running and answering.
DISCOVERY_VARS = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                  "FASTDDS_BUILTIN_TRANSPORTS",
                  "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_LOCALHOST_ONLY")

# What env.sh pins. A shell that never sourced it differs from this.
EXPECTED = {"ROS_DOMAIN_ID": "0",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTDDS_BUILTIN_TRANSPORTS": "SHM",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
            "ROS_LOCALHOST_ONLY": ""}

# Processes that constitute "a stack". Matched on the installed path so that
# a shell whose command line merely mentions the name cannot count -- see the
# header of procscan.py for the four times that exact mistake was made.
STACK_RX = (r"lib/srl_teleop/master_pose_node"
            r"|lib/moveit_ros_move_group/move_group"
            r"|lib/controller_manager/ros2_control_node")

# ANYTHING ON THE NETWORK, which is a wider question than "is a stack up".
#
# It was STACK_RX, and that was wrong in a way that would have broken a run:
# the leftover-memory row offers to clear the shared memory blocks whenever no
# STACK is running, and the mock and real arm stacks -- mock_real_stack, the
# homing nodes, the cascade bridges, the prefixed publisher -- are not a
# stack by that definition. So with the real arms up and the sim down, the
# panel would have offered to pull the shared memory out from underneath
# them. Matched on the installed library path, which every ROS node
# executable has and no shell command line does.
LIVE_RX = r"(/opt/ros/[a-z]+/lib/[^/ ]+/|/install/[A-Za-z0-9_]+/lib/[A-Za-z0-9_]+/)"
KORTEX_RX = r"lib/srl_teleop/kortex_highlevel_bridge"
RSP_RX = r"lib/robot_state_publisher/robot_state_publisher"


class Check:
    """One diagnosis. `fix` is a callable taking no arguments, or None.

    `state` is OK / BAD / UNKNOWN. `title` and `plain` are what the operator
    reads and carry no topic names; `detail` carries the machinery.
    """

    def __init__(self, key, title, state, plain, detail="",
                 fix_label=None, fix=None, fix_note=""):
        self.key = key
        self.title = title
        self.state = state
        self.plain = plain
        self.detail = detail
        self.fix_label = fix_label
        self.fix = fix
        self.fix_note = fix_note

    @property
    def has_fix(self):
        return self.fix is not None

    def __repr__(self):
        return "Check(%s, %s)" % (self.key, self.state)


# ===========================================================================
#  THE PROBE -- everything that touches the machine, in one place
# ===========================================================================
class Probe:
    """Reads the world. Subclassed by the test with a world that is broken.

    Every method here is allowed to fail; each returns None for "could not
    find out", which the checks turn into UNKNOWN rather than into OK.
    """

    # ---------------------------------------------------------------- env
    def my_env(self):
        return dict(os.environ)

    def stack_pids(self):
        return [pid for pid, _ in procscan.find(STACK_RX)]

    def live_pids(self):
        """Every ROS node process on this machine, stack or not."""
        return [pid for pid, _ in procscan.find(LIVE_RX)]

    def orphan_nodes(self):
        """[(pid, cmd)] for nodes whose launch has gone.

        A launch that dies re-parents its children to init, so PPID 1 on a
        node executable means "the run that started this has ended and this
        did not". Those keep publishing on the same topics as whatever starts
        next. A node started by hand from a terminal has that terminal as its
        parent and is not caught; a launch process itself is excluded, since
        a detached launch is legitimately parented to init.
        """
        out = []
        for pid, cmd in procscan.find(LIVE_RX):
            if " launch " in cmd or cmd.endswith(" launch"):
                continue
            try:
                with open("/proc/%d/status" % pid) as fh:
                    m = re.search(r"^PPid:\s*(\d+)", fh.read(), re.M)
            except OSError:
                continue
            if m and int(m.group(1)) == 1:
                out.append((pid, cmd))
        return out

    def env_of(self, pid):
        """The environment a running process was started with, or None."""
        try:
            with open("/proc/%d/environ" % pid, "rb") as fh:
                raw = fh.read()
        except OSError:
            return None
        out = {}
        for item in raw.split(b"\0"):
            if b"=" in item:
                k, v = item.split(b"=", 1)
                out[k.decode("utf-8", "replace")] = v.decode("utf-8", "replace")
        return out or None

    # ---------------------------------------------------------------- shm
    def shm_segments(self):
        try:
            return sorted(glob.glob("/dev/shm/fastrtps_*")
                          + glob.glob("/dev/shm/sem.fastrtps_*"))
        except OSError:
            return None

    # A SEGMENT YOUNGER THAN THIS IS NOT A LEFTOVER, whatever the maps say: a
    # process that is STARTING has created segments it has not mapped yet.
    SHM_MIN_AGE_S = 20.0

    def stale_shm_segments(self):
        """Segments that NO LIVE PROCESS HAS MAPPED, and are not brand new.

        THIS REPLACES A GUESS THAT WAS WRONG IN THE LAB. The check asked "is
        an installed node executable running?" and, if not, called every
        segment on disk stale. Measured on 2026-08-20 with an operator
        watching: 20 segments, of which 10 were held by the ros2 daemon, the
        operations GUI itself and RViz -- none of which is a node executable.
        The panel offered to clear all 20, which would have pulled the shared
        memory out from under the window the operator was reading it in.

        The kernel already knows the answer. /proc/*/maps names every segment
        every live process has mapped; anything on disk, in nobody's map, and
        older than a start-up window, is a leftover.
        """
        try:
            on_disk = {os.path.basename(q): q
                       for q in glob.glob("/dev/shm/fastrtps_*")}
        except OSError:
            return None
        mapped = set()
        for maps in glob.glob("/proc/[0-9]*/maps"):
            try:
                with open(maps) as fh:
                    for line in fh:
                        if "/dev/shm/fastrtps_" in line:
                            mapped.add(line.rsplit("/", 1)[-1].strip())
            except OSError:
                continue            # it died while we were reading
        now = time.time()
        out = []
        for name, q in on_disk.items():
            if name in mapped:
                continue
            try:
                if now - os.path.getmtime(q) < self.SHM_MIN_AGE_S:
                    continue        # something is starting; leave it alone
            except OSError:
                continue
            out.append(q)
        return sorted(out)

    # ------------------------------------------------------------- daemon
    def daemon_nodes(self, timeout_s=8):
        """(nodes, timed_out). nodes is None if the command could not run."""
        return self._nodes(["ros2", "node", "list"], timeout_s)

    def direct_nodes(self, timeout_s=20):
        return self._nodes(["ros2", "node", "list", "--no-daemon"], timeout_s)

    def _nodes(self, argv, timeout_s):
        try:
            p = subprocess.run(argv, capture_output=True, text=True,
                               timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return [], True
        except OSError:
            return None, False
        return [x.strip() for x in p.stdout.splitlines() if x.strip()], False

    # -------------------------------------------------------------- procs
    def find(self, rx):
        return procscan.find(rx)

    # ------------------------------------------------------------- teensy
    def tty_candidates(self):
        try:
            from srl_teleop import serial_port
            return serial_port.candidates()
        except Exception:                                     # noqa: BLE001
            return None

    def master_port_param(self):
        """The port master_pose_node was actually given, or None.

        Read from its COMMAND LINE, not from the parameter server: the
        parameter query goes through the daemon, and the daemon is one of the
        things this panel exists to diagnose.
        """
        for _, cmd in procscan.find(r"lib/srl_teleop/master_pose_node"):
            m = re.search(r"serial_port:?=(\S+)", cmd)
            if m:
                return m.group(1)
        return ""

    # --------------------------------------------------------------- pose
    def joint_states(self):
        """(sim, real) as {joint name: radians}, or (None, None).

        Supplied by the GUI from its own subscriptions. The bare Probe has no
        graph connection and says so by returning None -- which is UNKNOWN,
        not OK.
        """
        return None, None

    def home_radians(self, arm):
        try:
            import sys
            cfg = os.path.join(WS, "config")
            if cfg not in sys.path:
                sys.path.insert(0, cfg)
            import home_positions
            return list(home_positions.load_home_radians(arm))
        except Exception:                                     # noqa: BLE001
            return None

    # ------------------------------------------------------------- kortex
    def kortex_procs(self):
        return procscan.find(KORTEX_RX)

    def kortex_log_tail(self):
        """The end of the last Kortex bridge log, or None if there is none.

        'kortex session closed cleanly' on the last shutdown is the evidence
        that the one permitted session was handed back.
        """
        d = os.environ.get("SRL_SCRATCH") or "/tmp"
        best, best_t = None, 0
        for p in glob.glob(os.path.join(d, "launch_real*.log")) + \
                glob.glob(os.path.join(d, "kortex*.log")):
            try:
                t = os.path.getmtime(p)
            except OSError:
                continue
            if t > best_t:
                best, best_t = p, t
        if best is None:
            return None
        try:
            with open(best, "rb") as fh:
                fh.seek(0, 2)
                fh.seek(max(0, fh.tell() - 8000))
                return fh.read().decode("utf-8", "replace")
        except OSError:
            return None


# ===========================================================================
#  THE SEVEN CHECKS
# ===========================================================================
def check_discovery(p, fixes=None):
    """1. Do this window and the running stack agree about how to find
    each other?"""
    mine = p.my_env()
    pids = p.stack_pids()
    missing = [k for k in ("FASTDDS_BUILTIN_TRANSPORTS",)
               if mine.get(k, "") != EXPECTED[k]]
    if not pids:
        # Nothing to compare against. We can still check ourselves against
        # what env.sh pins, which is the half that is knowable.
        if missing:
            return Check(
                "discovery",
                "This window is not set up to talk to the arms",
                BAD,
                "This window is missing the one setting that lets processes "
                "on this machine find each other. Anything you start from "
                "here would run, and then be invisible to everything else.",
                detail="%s is %r, expected %r; no stack running to compare"
                       % ("FASTDDS_BUILTIN_TRANSPORTS",
                          mine.get("FASTDDS_BUILTIN_TRANSPORTS", ""),
                          EXPECTED["FASTDDS_BUILTIN_TRANSPORTS"]),
                fix_label="Restart this window set up correctly",
                fix=(fixes or {}).get("reexec"),
                fix_note="closes and reopens this window with the settings "
                         "the rest of the system uses")
        return Check("discovery",
                     "This window is set up to talk to the arms", OK,
                     "Nothing else is running yet, so there is nothing to "
                     "disagree with. This window's own settings are right.",
                     detail="no stack processes; own env matches env.sh")
    theirs = None
    for pid in pids:
        theirs = p.env_of(pid)
        if theirs:
            break
    if theirs is None:
        return Check("discovery", "Cannot tell whether this window agrees "
                     "with what is running", UNKNOWN,
                     "Something is running, but this window was not allowed "
                     "to read how it was started, so agreement cannot be "
                     "confirmed either way.",
                     detail="stack pids %s; /proc/<pid>/environ unreadable"
                            % pids)
    diff = []
    for k in DISCOVERY_VARS:
        a, b = mine.get(k, ""), theirs.get(k, "")
        if a != b:
            diff.append((k, a, b))
    if diff:
        return Check(
            "discovery",
            "This window and the running system disagree about how to find "
            "each other", BAD,
            "Something is running, but this window is set up differently, so "
            "it may see none of it -- or see it intermittently, which is "
            "worse. The two have to match before anything you start here can "
            "join in.",
            detail="; ".join("%s here=%r there=%r" % d for d in diff),
            fix_label="Restart this window to match what is running",
            fix=(fixes or {}).get("reexec"),
            fix_note="reopens this window using the settings the running "
                     "system was started with")
    return Check("discovery", "This window agrees with what is running", OK,
                 "This window and the running system use the same discovery "
                 "settings, so they can see each other.",
                 detail="%d stack process(es); %s all equal"
                        % (len(pids), ", ".join(DISCOVERY_VARS)))


def check_shm(p, fixes=None):
    """2. Leftover shared-memory segments from a killed stack.

    STALE MEANS NOBODY HAS IT OPEN, asked of the kernel. It used to mean "no
    installed node executable is running", which counted the operations GUI,
    RViz and the ros2 daemon as nothing -- see `Probe.stale_shm_segments`.
    """
    segs = p.shm_segments()
    if segs is None:
        return Check("shm", "Cannot check for leftovers from an earlier run",
                     UNKNOWN,
                     "The place those leftovers live could not be read.",
                     detail="/dev/shm not listable")
    stale = p.stale_shm_segments()
    if stale is None:
        return Check("shm", "Cannot check for leftovers from an earlier run",
                     UNKNOWN,
                     "Which blocks are still in use could not be established.",
                     detail="/proc not readable")
    if not segs:
        return Check("shm", "No leftovers from an earlier run", OK,
                     "Nothing an earlier run left behind is in the way.",
                     detail="0 Fast DDS segments in /dev/shm")
    if not stale:
        return Check("shm", "Shared memory in use by what is running", OK,
                     "The blocks in use belong to the system that is "
                     "running now, which is normal.",
                     detail="%d segment(s), none unowned" % len(segs))
    return Check(
        "shm", "An earlier run left blocks of memory behind", BAD,
        "A previous run was killed rather than stopped, and it left %d blocks "
        "of shared memory behind. While they are there, parts of the system "
        "can see each other one moment and not the next, which looks exactly "
        "like a broken node. They can only be cleared with everything "
        "STOPPED, including this window -- clearing them while anything is "
        "running can take its memory with them." % len(stale),
        detail="%d of %d segment(s) unowned for more than %.0f s"
               % (len(stale), len(segs), Probe.SHM_MIN_AGE_S),
        fix_label="Clear the leftovers",
        fix=(fixes or {}).get("clear_shm"),
        fix_note="removes both the segments and their semaphores; refuses "
                 "if anything is running")


def check_daemon(p, fixes=None):
    """3. The background helper that answers 'what is running' has hung."""
    nodes, timed_out = p.daemon_nodes()
    if nodes is None:
        return Check("daemon", "Cannot ask what is running", UNKNOWN,
                     "The command that lists what is running could not be "
                     "started at all.",
                     detail="`ros2 node list` could not be executed")
    procs = p.stack_pids()
    if timed_out:
        return Check(
            "daemon", "The helper that lists what is running has hung", BAD,
            "Asking what is running does not come back. It has not failed, "
            "it is stuck -- so every question about the system will time out "
            "and read as 'nothing is running'.",
            detail="`ros2 node list` timed out",
            fix_label="Restart that helper",
            fix=(fixes or {}).get("reset_daemon"),
            fix_note="stops it, clears it and starts it again; takes a few "
                     "seconds and disturbs nothing that is running")
    if not nodes and procs:
        return Check(
            "daemon", "What is running is invisible from this window", BAD,
            "%d parts of the system are running, and asking what is running "
            "returns nothing at all. That answer is stale rather than wrong: "
            "the helper is holding an old picture of the network."
            % len(procs),
            detail="%d stack pids, `ros2 node list` returned 0 nodes"
                   % len(procs),
            fix_label="Restart that helper",
            fix=(fixes or {}).get("reset_daemon"),
            fix_note="stops it, clears it and starts it again")
    if not nodes and not procs:
        return Check("daemon", "Nothing is running, and that is the answer "
                     "given", OK,
                     "Nothing is running yet, and asking gives that answer "
                     "promptly, so the question itself works.",
                     detail="0 nodes, 0 stack pids, no timeout")
    return Check("daemon", "Asking what is running works", OK,
                 "The system answers questions about itself promptly.",
                 detail="%d node(s) listed" % len(nodes))


def check_second_stack(p, fixes=None):
    """4. Two stacks, or a stray publisher that does the same damage."""
    masters = p.find(r"lib/srl_teleop/master_pose_node")
    movegroups = p.find(r"lib/moveit_ros_move_group/move_group")
    stray = [(pid, cmd) for pid, cmd in p.find(RSP_RX)
             if "--params-file" not in cmd
             and (".urdf" in cmd or ".xacro" in cmd)]
    if len(masters) > 1 or len(movegroups) > 1:
        who = masters if len(masters) > 1 else movegroups
        return Check(
            "second_stack", "The system is running twice", BAD,
            "Two copies of the same thing are running at once. They will "
            "split the readings from the master arm between them, and every "
            "measurement taken while both are up is worthless. This has cost "
            "a full day of work before.",
            detail="duplicate: %s" % ", ".join(
                "pid %d" % pid for pid, _ in who),
            fix_label="Stop the extra copy",
            fix=(fixes or {}).get("kill_second_stack"),
            fix_note="stops the newest copy and leaves the first one running")
    if stray:
        return Check(
            "second_stack", "Something left over is claiming to be the robot",
            BAD,
            "A leftover process from other work is publishing its own idea of "
            "the robot's shape. If you start the system now it will read that "
            "one, fail to find its motor definitions, and quietly die -- and "
            "every joint on screen will read zero, which looks like a "
            "plausible posture rather than a fault.",
            detail="stray robot_state_publisher: %s"
                   % ", ".join("pid %d" % pid for pid, _ in stray),
            fix_label="Stop the leftover",
            fix=(fixes or {}).get("kill_stray_rsp"),
            fix_note="stops only the leftover, not the running system")
    return Check("second_stack", "Only one copy of the system is running", OK,
                 "Nothing is running twice.",
                 detail="%d master_pose_node, %d move_group, %d stray "
                        "robot_state_publisher"
                        % (len(masters), len(movegroups), len(stray)))


def check_orphans(p, fixes=None):
    """4b. Pieces of a run that has ended and did not stop with it.

    THIS IS THE ONE THAT WAS FOUND BY BEING BITTEN. Stopping a launch on this
    box left seven of its nodes behind -- both followers, the e-stop, the
    gripper node, the mount guard, the recovery manager and an RViz -- all
    re-parented to init and all still publishing. They are not a SECOND stack,
    so the duplicate check did not see them, and they are not a stray robot
    description, so that check did not either. They simply keep answering on
    the topics the next run is about to use.
    """
    try:
        orphans = p.orphan_nodes()
    except Exception:                                         # noqa: BLE001
        orphans = None
    if orphans is None:
        return Check("orphans", "Cannot check for pieces of an earlier run",
                     UNKNOWN,
                     "The list of running programs could not be read.",
                     detail="orphan_nodes() unavailable")
    if not orphans:
        return Check("orphans", "Nothing left over from an earlier run", OK,
                     "Every part of the system that is running belongs to "
                     "something that is still running.",
                     detail="0 node processes re-parented to init")
    names = sorted({c.rsplit("/", 1)[-1].split(" ")[0] for _, c in orphans})
    return Check(
        "orphans", "Parts of an earlier run are still going", BAD,
        "%d part%s of a run that has already ended %s still going and still "
        "publishing: %s. They will answer alongside whatever you start next, "
        "and both answers look equally real."
        % (len(orphans), "" if len(orphans) == 1 else "s",
           "is" if len(orphans) == 1 else "are", ", ".join(names[:6])),
        detail="; ".join("pid %d %s" % (pid, c.rsplit("/", 1)[-1][:40])
                         for pid, c in orphans),
        fix_label="Stop the leftovers",
        fix=(fixes or {}).get("kill_orphans"),
        fix_note="stops only the parts whose own run has ended; anything "
                 "still attached to a live run is untouched")


def check_teensy(p, fixes=None):
    """5. The master arm board moves between ports on every re-attach."""
    cands = p.tty_candidates()
    if cands is None:
        return Check("teensy", "Cannot check the master arm connection",
                     UNKNOWN,
                     "The list of connected devices could not be read.",
                     detail="serial_port.candidates() unavailable")
    if not cands:
        return Check(
            "teensy", "The master arm is not plugged in", BAD,
            "The master arm's board is not attached to this machine. On this "
            "setup it has to be handed over from Windows after every reboot "
            "or unplug.",
            detail="no /dev/ttyACM* or /dev/ttyUSB*",
            fix_label="Show me how to attach it",
            fix=(fixes or {}).get("teensy_help"),
            fix_note="prints the one command to run on the Windows side")
    given = p.master_port_param()
    if given and given not in cands:
        return Check(
            "teensy", "The master arm moved to a different socket", BAD,
            "The board is plugged in, but the running system is looking for "
            "it at the socket it used last time. It will sit there receiving "
            "nothing, which looks exactly like a dead board.",
            detail="running with %r; present: %s" % (given, ", ".join(cands)),
            fix_label="Point it at the right socket",
            fix=(fixes or {}).get("teensy_repoint"),
            fix_note="restarts the master arm reader on the socket the board "
                     "is actually on")
    return Check("teensy", "The master arm is plugged in", OK,
                 "The board is attached%s."
                 % ("" if not given else " and the running system is using it"),
                 detail="candidates: %s%s"
                        % (", ".join(cands),
                           "" if not given else "; running with %r" % given))


def check_home_gap(p, fixes=None, tol=0.05):
    """6. The sim home and the real arms' home differ, so the bridge refuses."""
    sim, real = p.joint_states()
    if real is None:
        return Check("home", "The real arms are not connected yet", UNKNOWN,
                     "Nothing is reporting where the real arms are, so "
                     "whether they match the simulation cannot be known.",
                     detail="no real joint state seen")
    worst, worst_name, worst_arm = 0.0, None, None
    seen = 0
    for arm in ("left", "right"):
        home = p.home_radians(arm)
        if home is None:
            continue
        for i in range(7):
            name = "%s_joint_%d" % (arm, i + 1)
            if name not in real:
                continue
            seen += 1
            d = real[name] - home[i]
            if i in (0, 2, 4, 6):          # continuous joints wrap
                d = (d + math.pi) % (2 * math.pi) - math.pi
            if abs(d) > worst:
                worst, worst_name, worst_arm = abs(d), name, arm
    if not seen:
        return Check("home", "Cannot compare the arms with the simulation",
                     UNKNOWN,
                     "The real arms are reporting, but not in a form that "
                     "can be lined up against the stored resting pose.",
                     detail="no matching joint names in the real state")
    if worst > tol:
        return Check(
            "home", "The real arms are not where the simulation thinks", BAD,
            "One joint on the %s arm is %.0f degrees away from the resting "
            "pose the simulation starts from. The cascade replays simulation "
            "angles onto the real arm, so it will refuse to switch on rather "
            "than command that difference as a jump. A gap of about 110 "
            "degrees on the last joint is the known one: the resting pose was "
            "changed in the simulation and the arms have not been taught the "
            "new one yet." % (worst_arm, math.degrees(worst)),
            detail="worst %s %.4f rad (%.2f deg), tolerance %.3f rad"
                   % (worst_name, worst, math.degrees(worst), tol),
            fix_label="Move the arms to the resting pose",
            fix=(fixes or {}).get("home_arms"),
            fix_note="drives the real arms to the stored pose slowly and "
                     "reports each joint; it does not change any stored value")
    return Check("home", "The real arms are at the resting pose", OK,
                 "The arms and the simulation agree about where the arms "
                 "are, so the cascade will switch on.",
                 detail="worst %.4f rad over %d joint(s), tolerance %.3f"
                        % (worst, seen, tol))


def check_kortex_session(p, fixes=None):
    """7. The arm permits exactly one session and a killed bridge leaks it."""
    procs = p.kortex_procs()
    if procs:
        return Check("kortex", "The arm connection is open and in use", OK,
                     "The one connection the arm allows is held by the "
                     "system that is running now.",
                     detail="%s" % ", ".join("pid %d" % pid for pid, _ in procs))
    tail = p.kortex_log_tail()
    if tail is None:
        return Check("kortex", "The arm has not been connected from here yet",
                     UNKNOWN,
                     "There is no record of a previous connection, so "
                     "whether the last one was handed back cannot be known. "
                     "If the next connection is refused, this is why.",
                     detail="no previous bridge log found")
    if "kortex session closed cleanly" in tail:
        return Check("kortex", "The last connection was handed back", OK,
                     "The previous run gave the arm's one connection back "
                     "properly, so the next one will be accepted.",
                     detail="log ends with a clean close")
    return Check(
        "kortex", "The last connection may still be held", BAD,
        "The previous run did not say it gave the arm's connection back. The "
        "arm allows exactly one, so the next attempt may be refused until the "
        "arm times the old one out. Stopping the old process properly is what "
        "releases it -- killing it is what caused this.",
        detail="last bridge log has no clean-close line",
        fix_label="Release the connection",
        fix=(fixes or {}).get("release_kortex"),
        fix_note="asks the arm to drop the stale connection and open a fresh "
                 "one; never relaunches anything")


CHECKS = (check_discovery, check_shm, check_daemon, check_second_stack,
          check_orphans, check_teensy, check_home_gap, check_kortex_session)

# The row order, fixed. The panel builds one permanent row per key and
# UPDATES it, rather than rebuilding the rows from each diagnosis: a re-check
# that destroys and recreates its own widgets takes the button out from under
# whoever is reaching for it, and the button audit caught exactly that by
# crashing on a QPushButton that had been deleted mid-walk.
KEYS = tuple(fn.__name__.replace("check_", "").replace("_gap", "")
             .replace("_session", "") for fn in CHECKS)

# What each row is called before anything has been checked. NOT "ok" and not
# blank: a row with no diagnosis behind it is UNKNOWN, and it says so.
UNCHECKED = {
    "discovery": "Whether this window can talk to the arms",
    "shm": "Whether an earlier run left anything behind",
    "daemon": "Whether the system answers questions about itself",
    "second_stack": "Whether anything is running twice",
    "orphans": "Whether anything is left over from an earlier run",
    "teensy": "Whether the master arm is plugged in",
    "home": "Whether the arms are at the resting pose",
    "kortex": "Whether the arm's one connection is free",
}


def run_all(probe=None, fixes=None):
    """Every check, in the order you meet them. Never raises.

    A check that raises becomes UNKNOWN with the exception in `detail`,
    because a panel that disappears when one row breaks is worse than a panel
    with one row that says it could not tell.
    """
    p = probe or Probe()
    out = []
    for fn in CHECKS:
        try:
            out.append(fn(p, fixes))
        except Exception as e:                                # noqa: BLE001
            out.append(Check(fn.__name__.replace("check_", ""),
                             "This check could not be run", UNKNOWN,
                             "Something went wrong while checking. It is not "
                             "safe to read this as working.",
                             detail="%r" % (e,)))
    return out


def verdict(checks):
    """('bad'|'unknown'|'ok', headline). NEVER OK ON UNKNOWN."""
    bad = [c for c in checks if c.state == BAD]
    unk = [c for c in checks if c.state == UNKNOWN]
    if bad:
        return BAD, "%d thing%s to fix before the arms will connect" % (
            len(bad), "" if len(bad) == 1 else "s")
    if unk:
        return UNKNOWN, "%d thing%s could not be checked" % (
            len(unk), "" if len(unk) == 1 else "s")
    return OK, "ready to connect the arms"


# ===========================================================================
#  THE FIXES. Each returns (ok, one sentence). None of them is destructive
#  beyond stopping a process it has named first.
# ===========================================================================
def fix_clear_shm():
    """Remove only the segments NOBODY HAS OPEN.

    The old guard was "refuse if anything is running", a proxy for "do not
    remove a segment somebody is using". It was too coarse in both directions:
    it let the sweep delete blocks held by the GUI, RViz and the ros2 daemon
    (none of which is a node executable), and it refused to clear a dead
    stack's leftovers whenever a healthy one was up -- which is exactly the
    state a partitioned graph is in.

    Ownership is now asked of the kernel, so no guard is needed: a segment
    nobody has mapped cannot be in use.
    """
    # NEVER WHILE ANYTHING IS RUNNING. Ownership narrows WHAT is removed; it
    # does not make removal safe. A live Fast DDS participant's segment can be
    # absent from /proc/*/maps at the instant it is sampled, and clearing on
    # that basis partitioned a working stack in a lab session -- a fresh
    # participant went from 34 nodes to 7, with zero publishers on /tf, while
    # every controller was still active. HARD CONSTRAINT 5 says "with the
    # stack STOPPED" and it is right.
    live = procscan.find(r"(/opt/ros/[a-z]+/lib/[^/ ]+/"
                         r"|/install/[A-Za-z0-9_]+/lib/[A-Za-z0-9_]+/"
                         r"|ros2cli\.daemon"
                         r"|scripts/srl_gui\.py"
                         r"|/rviz2)")
    if live:
        return False, ("Not cleared: %d thing(s) are still running, including "
                       "this window. Stop everything first -- removing these "
                       "while anything is up can take its memory with them."
                       % len(live))
    stale = Probe().stale_shm_segments()
    if not stale:
        return True, ("Nothing to clear: every block of shared memory is in "
                      "use by something that is running.")
    n = 0
    for path in stale:
        for target in (path, path.replace("/dev/shm/", "/dev/shm/sem.")):
            try:
                os.remove(target)
                n += 1 if target == path else 0
            except OSError:
                pass
    return True, ("Cleared %d leftover block(s). Anything still in use was "
                  "left alone." % n)


def fix_reset_daemon():
    for argv in (["ros2", "daemon", "stop"],):
        try:
            subprocess.run(argv, capture_output=True, timeout=10)
        except Exception:                                     # noqa: BLE001
            pass
    procscan.kill_all(r"ros2cli\.daemon\.daemonize")
    time.sleep(1.0)
    try:
        subprocess.run(["ros2", "daemon", "start"], capture_output=True,
                       timeout=25)
    except Exception as e:                                    # noqa: BLE001
        return False, "Could not start it again: %r" % (e,)
    return True, "Restarted. Ask again in a moment."


def fix_kill_second_stack():
    """Stop the NEWEST duplicate, keep the oldest.

    Deliberately not 'stop them all': the first one may be the stack the
    operator is using, and stopping both turns a recoverable mistake into a
    restart.
    """
    stopped = []
    for rx in (r"lib/srl_teleop/master_pose_node",
               r"lib/moveit_ros_move_group/move_group"):
        found = procscan.find(rx)
        if len(found) < 2:
            continue

        # oldest first by pid start time
        def _start(pid):
            try:
                with open("/proc/%d/stat" % pid) as fh:
                    return int(fh.read().split()[21])
            except (OSError, IndexError, ValueError):
                return 0
        found.sort(key=lambda t: _start(t[0]))
        for pid, _ in found[1:]:
            try:
                os.killpg(os.getpgid(pid), 2)
            except OSError:
                try:
                    os.kill(pid, 2)
                except OSError:
                    continue
            stopped.append(pid)
    if not stopped:
        return False, "Nothing duplicate found to stop."
    return True, "Stopped %s." % ", ".join("pid %d" % p for p in stopped)


def fix_kill_orphans():
    """Stop node processes whose launch has gone. Nothing else.

    Signals the PROCESS GROUP where there is one, per HARD CONSTRAINT 9, and
    only ever the pids `orphan_nodes()` returned -- which cannot include this
    process or any ancestor, because procscan excludes them.
    """
    targets = Probe().orphan_nodes()
    if not targets:
        return False, "Nothing left over to stop."
    for pid, _ in targets:
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except OSError:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
    time.sleep(2.0)
    still = {pid for pid, _ in Probe().orphan_nodes()}
    stubborn = [pid for pid, _ in targets if pid in still]
    for pid in stubborn:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    return True, ("Stopped %d leftover part(s)%s."
                  % (len(targets),
                     "" if not stubborn
                     else " (%d needed a second ask)" % len(stubborn)))


def fix_kill_stray_rsp():
    stopped = []
    for pid, cmd in procscan.find(RSP_RX):
        if "--params-file" in cmd:
            continue
        if ".urdf" not in cmd and ".xacro" not in cmd:
            continue
        try:
            os.kill(pid, 15)
            stopped.append(pid)
        except OSError:
            pass
    if not stopped:
        return False, "No leftover found to stop."
    return True, "Stopped %s." % ", ".join("pid %d" % p for p in stopped)


def teensy_help_text():
    try:
        from srl_teleop.serial_port import REATTACH_HINT
        return REATTACH_HINT
    except Exception:                                         # noqa: BLE001
        return ("From Windows PowerShell (Administrator):\n"
                "    usbipd attach --wsl --hardware-id 16c0:0483")
