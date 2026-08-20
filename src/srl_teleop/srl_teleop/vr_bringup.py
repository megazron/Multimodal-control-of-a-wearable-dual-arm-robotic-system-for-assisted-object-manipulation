#!/usr/bin/env python3
"""ONE BUTTON: everything VR teleoperation needs, in order, or a plain reason.

    from srl_teleop import vr_bringup
    plan = vr_bringup.plan()
    for step in plan:
        outcome = step.run(world)

WHAT THIS REPLACES. Starting VR was six commands in four terminals in an order
you had to know, and every one of them could half-fail. The order is not
arbitrary -- `vr_pose_mapper` refuses to engage without robot TF, and the
symptom of getting it wrong is a clutch that silently does nothing -- so the
order is encoded here rather than remembered.

THE RULES, and they are the same three the connection panel obeys:

  * NEVER GREEN ON UNKNOWN. A step that could not be checked reads UNKNOWN,
    which is its own state and never a pass.
  * A STEP THAT CANNOT FAIL ON A DELIBERATELY BROKEN INPUT IS NOT A STEP.
    Everything reaches the machine through `World`, so a test can hand each
    step a broken one and require it to say so -- without binding a port,
    deleting a certificate or killing a daemon on the machine running the
    test.
  * NO TOPIC NAMES AND NO TRACEBACKS in `plain`. That is what the operator
    reads. The machinery goes in `detail`, for the log and for a bug report.

WHAT IT DOES NOT DO. It never touches a real arm. Every step here is sim,
transport and safety plumbing; the real arms are a separate, later, deliberate
act with its own procedure (docs/system/VR_REAL_ARM_RUN.md). A bring-up
sequence that could power a robot as a side effect of a single click would be
the wrong shape whatever it printed.
"""
import glob
import os
import re
import subprocess
import time

OK = "ok"
FAILED = "failed"
UNKNOWN = "unknown"
RUNNING = "running"
PENDING = "pending"
SKIPPED = "skipped"

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")
PORT = int(os.environ.get("VR_PORT", "8765"))

# HOW LONG TO WAIT AFTER CLEARING SHARED MEMORY, and it is not superstition.
# Clearing removes the segments; the participants that were using them do not
# find out instantly. A stack started into a half-torn-down transport comes up
# and cannot see itself -- which looks exactly like the problem just cleared,
# so the operator clears again and goes round. Five seconds is the number the
# runbook has carried since this was first diagnosed.
SHM_SETTLE_S = 5.0


class Outcome:
    """What one step did, and what to do if it did not."""

    def __init__(self, state, plain, detail="", fix_label=None, fix=None,
                 fix_note=""):
        self.state = state
        self.plain = plain
        self.detail = detail
        self.fix_label = fix_label
        self.fix = fix
        self.fix_note = fix_note

    @property
    def ok(self):
        return self.state == OK

    @property
    def has_fix(self):
        return self.fix is not None

    def __repr__(self):
        return "Outcome(%s, %r)" % (self.state, self.plain[:40])


class Step:
    def __init__(self, key, title, fn, optional=False):
        self.key = key
        self.title = title
        self.fn = fn
        self.optional = optional

    def run(self, world):
        try:
            return self.fn(world)
        except Exception as e:                                # noqa: BLE001
            return Outcome(UNKNOWN, "This step could not be carried out. It "
                                    "is not safe to read that as working.",
                           detail="%r" % (e,))

    def __repr__(self):
        return "Step(%s)" % self.key


# ===========================================================================
#  THE WORLD -- everything that touches the machine, in one replaceable place
# ===========================================================================
class World:
    """The real machine. Subclassed by tests with a broken one."""

    def __init__(self, ws=WS, port=PORT):
        self.ws = ws
        self.port = port
        self.started = []          # [(name, Popen)]
        self.notes = []

    # ------------------------------------------------------------- system
    def env(self):
        return dict(os.environ)

    def procs(self, pattern):
        from srl_teleop import procscan
        return procscan.find(pattern)

    def shm_segments(self):
        return sorted(glob.glob("/dev/shm/fastrtps_*")
                      + glob.glob("/dev/shm/sem.fastrtps_*"))

    # A SEGMENT YOUNGER THAN THIS IS NOT A LEFTOVER, whatever the maps say.
    #
    # A process that is STARTING creates its segments before it has mapped
    # them all, so for a moment they are unowned by the /proc test -- and
    # sweeping then destroys the middleware of the thing that is coming up.
    # Measured: the sim step spawned the simulation, a daemon restart swept
    # 46 blocks a second later, and the brand-new simulation came up detached
    # from everything.
    #
    # Twenty seconds is longer than any start-up window here and far shorter
    # than the age of anything left by a previous session.
    SHM_MIN_AGE_S = 20.0

    def stale_shm_segments(self):
        """Segments that NO LIVE PROCESS HAS MAPPED. Exact, not inferred.

        THE TWO GUESSES THIS REPLACES, both of which were wrong in turn:

          * "stale if no node process is running" -- the ros2 daemon is a
            participant and is not an installed node executable, so its own
            segments read as an earlier run's leftovers. The repair restarts
            the daemon, which makes more of them. Four repairs in a row,
            measured.
          * "in use if ANY participant is running" -- then a killed stack's
            genuinely stale segments are masked the moment the daemon is up,
            and the repair that would fix the partition is never offered.

        The kernel already knows the answer: `/proc/*/maps` names every
        segment every live process has mapped. Anything on disk and in
        nobody's map is a leftover, whatever else is running.
        """
        on_disk = {os.path.basename(p): p
                   for p in glob.glob("/dev/shm/fastrtps_*")}
        mapped = set()
        for maps in glob.glob("/proc/[0-9]*/maps"):
            try:
                with open(maps) as fh:
                    for line in fh:
                        if "/dev/shm/fastrtps_" in line:
                            mapped.add(line.rsplit("/", 1)[-1].strip())
            except OSError:
                continue          # the process died while we were reading
        now = time.time()
        out = []
        for name, path in on_disk.items():
            if name in mapped:
                continue
            try:
                if now - os.path.getmtime(path) < self.SHM_MIN_AGE_S:
                    continue          # something is starting; leave it alone
            except OSError:
                continue
            out.append(path)
        return sorted(out)

    def nodes(self, timeout_s=8):
        """(list, timed_out). None if the query could not be run at all."""
        try:
            p = subprocess.run(["ros2", "node", "list"], capture_output=True,
                               text=True, timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return [], True
        except OSError:
            return None, False
        return [x.strip() for x in p.stdout.splitlines() if x.strip()], False

    def nodes_direct(self, timeout_s=25):
        """The graph WITHOUT the helper. The tie-breaker.

        `env.sh` documents this as the thing that tells "the helper is stale"
        apart from "there is genuinely nothing there", and the bring-up needs
        the same distinction for a third case: a process that is running and
        has LOST ITS MIDDLEWARE -- which happens to anything that was up when
        shared memory was cleared. That looks identical to a stale helper from
        the helper's side, and restarting the helper does not fix it.
        """
        try:
            p = subprocess.run(["ros2", "node", "list", "--no-daemon"],
                               capture_output=True, text=True,
                               timeout=timeout_s)
        except Exception:                                     # noqa: BLE001
            return None
        return [x.strip() for x in p.stdout.splitlines() if x.strip()]

    def topics(self, timeout_s=10):
        try:
            p = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                               text=True, timeout=timeout_s)
        except Exception:                                     # noqa: BLE001
            return None
        return [x.strip() for x in p.stdout.splitlines() if x.strip()]

    def port_owner(self, port):
        """(pid, cmd) holding the port, or None."""
        try:
            p = subprocess.run(["ss", "-ltnp", "sport = :%d" % port],
                               capture_output=True, text=True, timeout=8)
        except Exception:                                     # noqa: BLE001
            return None
        m = re.search(r'pid=(\d+)', p.stdout or "")
        if not m:
            return None
        pid = int(m.group(1))
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            cmd = ""
        return (pid, cmd.strip())

    def lan_ip(self):
        try:
            out = subprocess.run(["hostname", "-I"], capture_output=True,
                                 text=True, timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return None
        for tok in out.split():
            if tok and not tok.startswith("127.") and tok[0].isdigit():
                return tok
        return None

    def cert_paths(self):
        d = os.environ.get("VR_CERT_DIR") or os.path.expanduser(
            "~/.srl_vr_cert")
        return os.path.join(d, "vr.crt"), os.path.join(d, "vr.key")

    def cert_covers(self, crt, ip):
        """(exists, covers_ip, detail)."""
        if not os.path.exists(crt):
            return False, False, "no certificate at %s" % crt
        try:
            out = subprocess.run(
                ["openssl", "x509", "-in", crt, "-noout", "-ext",
                 "subjectAltName"], capture_output=True, text=True,
                timeout=8).stdout
        except Exception as e:                                # noqa: BLE001
            return True, False, "certificate unreadable: %r" % (e,)
        return True, ("IP Address:%s" % ip) in out, out.strip()

    def mapper_state(self, timeout_s=8):
        """What the controller mapping is holding, or None.

        Readable while DISENGAGED: the mapper publishes an idle heartbeat at
        2 Hz. Before that it spoke only while engaged, so "clean", "reset",
        "frozen" and "the process died" were one observation.
        """
        import json
        try:
            p = subprocess.run(
                ["ros2", "topic", "echo", "/vr/mapper_right", "--once",
                 "--field", "data"], capture_output=True, text=True,
                timeout=timeout_s)
        except Exception:                                     # noqa: BLE001
            return None
        for line in (p.stdout or "").splitlines():
            line = line.strip()
            if line.startswith("{"):
                try:
                    return json.loads(line)
                except Exception:                             # noqa: BLE001
                    return None
        return None

    def pose_beats(self, window_s=2.0):
        """How many controller poses ARRIVED in `window_s`, or None.

        COUNTED, NOT LISTED. `quest_bridge_node` creates its publishers at
        start-up, so the pose topic EXISTS the moment the bridge does --
        with or without a headset. Checking the topic list therefore said
        READY TO OPERATE on a machine with no headset in the building, which
        is CLAUDE.md's "feature present but does nothing" row: the topic was
        advertised and nothing was flowing.
        """
        try:
            p = subprocess.run(
                ["timeout", "%.1f" % (window_s + 0.5), "ros2", "topic", "echo",
                 "/vr/controller_pose_right", "--field", "header.frame_id"],
                capture_output=True, text=True, timeout=window_s + 4.0)
        except Exception:                                     # noqa: BLE001
            return None
        return sum(1 for ln in (p.stdout or "").splitlines() if ln.strip()
                   and not ln.startswith("-"))

    def observer_beats(self, window_s=2.0):
        """How many observer heartbeats arrived in `window_s`, or None.

        COUNTED, NOT LATCHED. The observer interlock is a heartbeat, so
        "present" is a rate and not a value -- a single retained True from a
        publisher that has since gone away is exactly the state this is meant
        to distinguish.
        """
        try:
            p = subprocess.run(
                ["timeout", "%.1f" % (window_s + 0.5), "ros2", "topic", "echo",
                 "/vr/observer_estop_present", "--field", "data"],
                capture_output=True, text=True, timeout=window_s + 4.0)
        except Exception:                                     # noqa: BLE001
            return None
        return sum(1 for ln in (p.stdout or "").splitlines()
                   if ln.strip().lower() == "true")

    def now(self):
        """THE CLOCK LIVES HERE TOO.

        The waiting steps had `time.time()` deadlines, which made every test
        of them take as long as the real timeout -- so the injection suite
        ran for minutes and was never going to be run often. A clock in
        `World` is the same seam as everything else in this file: the test
        advances it in `sleep()` and the waits are instant.
        """
        return time.monotonic()

    def sleep(self, s):
        time.sleep(s)

    # ------------------------------------------------------------ actions
    def spawn(self, name, argv, log=None):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        out = subprocess.DEVNULL
        if log:
            try:
                out = open(log, "wb")
            except OSError:
                out = subprocess.DEVNULL
        p = subprocess.Popen(argv, env=env, start_new_session=True,
                             stdout=out, stderr=subprocess.STDOUT)
        self.started.append((name, p))
        return p

    def alive(self, p):
        return p is not None and p.poll() is None

    def note(self, text):
        self.notes.append(text)


# ===========================================================================
#  THE STEPS
# ===========================================================================
def step_environment(w):
    """1. This window is set up to talk to everything else."""
    e = w.env()
    want = {"FASTDDS_BUILTIN_TRANSPORTS": "SHM",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
    wrong = {k: e.get(k, "") for k, v in want.items() if e.get(k, "") != v}
    if e.get("ROS_LOCALHOST_ONLY"):
        wrong["ROS_LOCALHOST_ONLY"] = e["ROS_LOCALHOST_ONLY"]
    if wrong:
        return Outcome(
            FAILED,
            "This window is not set up to talk to the rest of the system. "
            "Anything started from here would run, and then be invisible to "
            "everything else.",
            detail="; ".join("%s=%r" % kv for kv in sorted(wrong.items())),
            fix_label="Restart this window set up correctly",
            fix="reexec",
            fix_note="reopens the window with the settings everything else "
                     "uses")
    return Outcome(OK, "This window is set up correctly.",
                   detail="domain %s, transport SHM"
                          % e.get("ROS_DOMAIN_ID", "0"))


# ANYTHING THAT HOLDS A SHARED-MEMORY PARTICIPANT, and the daemon is one.
#
# This pattern matched installed node executables only, so the ros2 DAEMON --
# `python3 -c "from ros2cli.daemon.daemonize import main; main()"` -- did not
# count as running. And the leftover-memory repair restarts the daemon, which
# creates fresh segments. So: clear, restart the daemon, see 14 new segments
# with "nothing running", offer to clear again, for ever. Measured on a clean
# run: four repairs in a row, each reporting 14 blocks.
#
# A repair that recreates the condition it repairs is worse than no repair.
LIVE_RX = (r"(/opt/ros/[a-z]+/lib/[^/ ]+/"
           r"|/install/[A-Za-z0-9_]+/lib/[A-Za-z0-9_]+/"
           r"|ros2cli\.daemon)")


def step_leftover_memory(w):
    """2. Nothing an earlier run left behind is in the way.

    STALE MEANS NOBODY HAS IT OPEN, and that is asked of the kernel rather
    than inferred from the process table. Two inferences were tried and both
    were wrong in opposite directions -- see `World.stale_shm_segments`.
    """
    segs = w.stale_shm_segments()
    total = len(w.shm_segments())
    if not segs:
        return Outcome(OK, "Shared memory is in use by what is running now, "
                           "which is normal."
                       if total else "Nothing left over from an earlier run.",
                       detail="%d segment(s), none unowned" % total)
    return Outcome(
        FAILED,
        "A previous run was killed rather than stopped and left %d blocks of "
        "shared memory behind. While they are there, parts of the system see "
        "each other one moment and not the next." % len(segs),
        detail="%d stale segments, nothing running" % len(segs),
        fix_label="Clear the leftovers and wait",
        fix="clear_shm",
        fix_note="removes them, then waits five seconds before anything "
                 "starts -- the participants using them do not find out "
                 "instantly, and starting into a half-torn-down transport "
                 "looks exactly like the fault just cleared")


def step_daemon(w):
    """3. The system answers questions about itself."""
    nodes, timed_out = w.nodes()
    if nodes is None:
        return Outcome(UNKNOWN, "Questions about what is running could not be "
                                "asked at all.",
                       detail="the node-list command could not be executed")
    procs = w.procs(r"lib/moveit_ros_move_group/move_group"
                    r"|lib/srl_teleop/master_pose_node")
    if timed_out:
        return Outcome(
            FAILED,
            "Asking what is running does not come back. It has not failed, it "
            "is stuck, so every question will time out and read as 'nothing "
            "is running'.",
            detail="node list timed out",
            fix_label="Restart that helper", fix="reset_daemon",
            fix_note="a few seconds, and it disturbs nothing that is running")
    if not nodes and procs:
        # WHICH OF TWO THINGS. A stale helper and a process that has lost its
        # middleware give the same empty answer, and only one of them is
        # fixed by restarting the helper. Anything that was running when
        # shared memory was cleared is in the second state, so this is the
        # NORMAL case after the previous step's repair -- and restarting the
        # helper at it, repeatedly, is a loop.
        direct = w.nodes_direct()
        if direct:
            return Outcome(
                FAILED,
                "%d parts of the system are running and this window cannot "
                "see them. The answer is stale rather than wrong."
                % len(procs),
                detail="%d processes, 0 via the helper, %d directly"
                       % (len(procs), len(direct)),
                fix_label="Restart that helper", fix="reset_daemon")
        return Outcome(
            FAILED,
            "%d parts of an earlier run are still running but have lost "
            "their connection to everything else. They cannot be recovered; "
            "they have to be stopped so a fresh start can take their place."
            % len(procs),
            detail="%d processes, 0 nodes via the helper AND 0 directly"
                   % len(procs),
            fix_label="Stop them", fix="kill_detached",
            fix_note="stops only those processes. Anything healthy is left "
                     "alone, and the simulation is started again by the next "
                     "step")
    return Outcome(OK, "The system answers questions about itself.",
                   detail="%d node(s) listed" % len(nodes))


def step_no_second_stack(w):
    """4. Nothing is running twice."""
    dupes = {}
    for label, rx in (("the simulation", r"lib/moveit_ros_move_group/move_group"),
                      ("the master arm reader", r"lib/srl_teleop/master_pose_node"),
                      ("the VR mapper", r"lib/srl_vr_teleop/vr_pose_mapper"),
                      ("the VR bridge", r"lib/srl_vr_teleop/quest_bridge_node")):
        found = w.procs(rx)
        if len(found) > 1:
            dupes[label] = found
    stray = [(pid, c) for pid, c in
             w.procs(r"lib/robot_state_publisher/robot_state_publisher")
             if "--params-file" not in c and (".urdf" in c or ".xacro" in c)]
    if dupes:
        label = sorted(dupes)[0]
        return Outcome(
            FAILED,
            "%s is running twice. Two copies fight over the same commands, "
            "and every measurement taken while both are up is worthless."
            % label.capitalize(),
            detail="; ".join("%s: %s" % (k, ", ".join(str(p) for p, _ in v))
                             for k, v in sorted(dupes.items())),
            fix_label="Stop the extra copy", fix="kill_second_stack")
    if stray:
        return Outcome(
            FAILED,
            "Something left over from other work is publishing its own idea "
            "of the robot's shape. Starting now would read that one and stop "
            "with every joint showing zero.",
            detail="stray robot description: %s"
                   % ", ".join(str(p) for p, _ in stray),
            fix_label="Stop the leftover", fix="kill_stray_rsp")
    return Outcome(OK, "Only one copy of everything is running.",
                   detail="no duplicates, no stray robot description")


def step_sim_stack(w, wait_s=90.0):
    """5. The simulation is up. START IT if it is not.

    THE ORDER MATTERS AND IT IS NOT ARBITRARY. `vr_pose_mapper` refuses to
    engage without robot TF, and the symptom of starting it first is a clutch
    that silently does nothing -- the operator squeezes, nothing moves, and
    nothing anywhere says why. So the simulation goes up first and this step
    WAITS for it rather than assuming.
    """
    topics = w.topics()
    if topics is None:
        return Outcome(UNKNOWN, "Could not tell whether the simulation is "
                                "running.", detail="topic list unavailable")
    if "/joint_states" in topics:
        return Outcome(OK, "The simulation is running.",
                       detail="%d topics" % len(topics))

    # RUNNING, BUT INVISIBLE. Before concluding the simulation is not there,
    # ask the process table -- the two are different problems with different
    # fixes and they produce the SAME empty answer. This is not hypothetical:
    # it happened on the first real run of this sequence, immediately after
    # the stale-memory clear, and the button blamed the simulation.
    procs = w.procs(r"lib/moveit_ros_move_group/move_group")
    if procs:
        return Outcome(
            FAILED,
            "The simulation IS running, and this window cannot see it. The "
            "answer is stale rather than wrong -- the helper that lists what "
            "is running is holding an old picture.",
            detail="%d simulation process(es), /joint_states not listed"
                   % len(procs),
            fix_label="Restart that helper", fix="reset_daemon",
            fix_note="a few seconds, and it disturbs nothing that is running")
    w.spawn("simulation",
            [os.path.join(w.ws, "scripts", "run_teleop.sh"), "gate:=false"],
            log=os.path.join(_scratch(w), "vr_bringup_sim.log"))
    t0 = w.now()
    while w.now() - t0 < wait_s:
        w.sleep(3.0)
        t = w.topics() or []
        if "/joint_states" in t:
            return Outcome(OK, "The simulation started and is running.",
                           detail="up after %.0f s" % (w.now() - t0))
    if w.procs(r"lib/moveit_ros_move_group/move_group"):
        return Outcome(
            FAILED,
            "The simulation started but this window still cannot see it. The "
            "answer is stale rather than wrong.",
            detail="processes present, /joint_states not listed after %.0f s"
                   % wait_s,
            fix_label="Restart the helper that lists what is running",
            fix="reset_daemon")
    return Outcome(
        FAILED,
        "The simulation did not finish starting. Nothing else can go up until "
        "it does, because the controller mapping needs to know where the "
        "robot is.",
        detail="no /joint_states and no simulation process after %.0f s"
               % wait_s,
        fix_label="Show me why", fix="show_sim_log",
        fix_note="opens what the simulation printed while it was starting")


def step_certificate(w):
    """6. The headset will accept the page."""
    ip = w.lan_ip()
    if not ip:
        return Outcome(FAILED, "This machine has no address on the network, "
                               "so nothing can reach it.",
                       detail="no non-loopback IPv4 address")
    crt, key = w.cert_paths()
    exists, covers, detail = w.cert_covers(crt, ip)
    if exists and covers:
        return Outcome(OK, "The headset will accept this machine's page.",
                       detail="%s covers %s" % (crt, ip))
    if not exists:
        return Outcome(
            FAILED,
            "There is no security certificate, so the headset's browser will "
            "refuse to start a session at all.",
            detail=detail, fix_label="Make one", fix="make_cert",
            fix_note="takes a second and needs nothing from you")
    return Outcome(
        FAILED,
        "The certificate is for a different address than this machine now "
        "has -- the network gave it a new one. In the headset that reads as a "
        "broken certificate and is a stale one.",
        detail="want %s; %s" % (ip, detail),
        fix_label="Make a new one", fix="make_cert")


def step_port_free(w):
    """7. Nothing else is holding the port the headset connects to."""
    owner = w.port_owner(w.port)
    if owner is None:
        return Outcome(OK, "The headset's connection point is free.",
                       detail="port %d unbound" % w.port)
    pid, cmd = owner
    mine = "quest_bridge_node" in cmd
    # OUR OWN HEALTHY BRIDGE IS NOT A BLOCKED PORT. This button is pressed
    # again after every repair and while VR is already running, and a bridge
    # that is up and serving must read as "already done" rather than as an
    # obstruction -- otherwise pressing the button on a working system reports
    # a fault and invites the operator to kill the thing that was working.
    if mine and w.procs(r"lib/srl_vr_teleop/quest_bridge_node"):
        return Outcome(OK, "The connection point is held by this system's own "
                           "bridge, which is what should be holding it.",
                       detail="port %d held by pid %d (our bridge)"
                              % (w.port, pid))
    return Outcome(
        FAILED,
        ("The connection the headset uses is already held by a bridge from an "
         "earlier run." if mine else
         "The connection the headset uses is already held by something else."),
        detail="port %d held by pid %d: %s" % (w.port, pid, cmd[:90]),
        fix_label="Stop it and take the port back", fix="free_port",
        fix_note="stops that process group. Without this the bridge dies on "
                 "startup while everything else looks healthy, and you find "
                 "out inside the headset")


def step_bridge(w, wait_s=25.0):
    """8. The bridge is serving the page and the connection.

    IDEMPOTENT, and that is not a nicety. This button is pressed again after
    every repair, so a step that spawns unconditionally spawns a second copy
    every time -- and two bridges is two things fighting for one port, while
    two mappers is two publishers on the arm command topic, which is the one
    thing the VR architecture says must never happen. Found by pressing the
    button four times in a row: by the fourth, the mapper was running twice.
    """
    if w.procs(r"lib/srl_vr_teleop/quest_bridge_node") and w.port_owner(w.port):
        return Outcome(OK, "The headset can already connect to this machine.",
                       detail="a bridge is already serving port %d" % w.port)
    crt, key = w.cert_paths()
    p = w.spawn("bridge", [
        "ros2", "run", "srl_vr_teleop", "quest_bridge_node", "--ros-args",
        "-p", "port:=%d" % w.port, "-p", "certfile:=%s" % crt,
        "-p", "keyfile:=%s" % key,
        "-p", "web_dir:=%s" % os.path.join(
            w.ws, "src/srl_vr_teleop/web")],
        log=os.path.join(_scratch(w), "vr_bringup_bridge.log"))
    t0 = w.now()
    while w.now() - t0 < wait_s:
        w.sleep(2.0)
        if not w.alive(p):
            return Outcome(
                FAILED,
                "The part that talks to the headset started and then stopped.",
                detail="bridge exited during start-up",
                fix_label="Show me why", fix="show_bridge_log",
                fix_note="opens what it printed before it stopped")
        if w.port_owner(w.port):
            return Outcome(OK, "The headset can connect to this machine.",
                           detail="serving on port %d" % w.port)
    if not w.alive(p):
        return Outcome(FAILED, "The part that talks to the headset started "
                               "and then stopped.",
                       detail="bridge exited", fix_label="Show me why",
                       fix="show_bridge_log")
    return Outcome(
        UNKNOWN,
        "The part that talks to the headset is running but has not started "
        "listening yet.",
        detail="alive but port %d not bound after %.0f s" % (w.port, wait_s))


def step_mapper(w, wait_s=20.0):
    """9. Controller motion becomes robot motion.

    IDEMPOTENT for the same reason as the bridge, and with a worse failure if
    it is not: TWO `vr_pose_mapper` are two publishers on
    the arm command topic. Exactly one input may run at a time.
    """
    already = w.procs(r"lib/srl_vr_teleop/vr_pose_mapper")
    if already:
        t = w.topics() or []
        if "/vr/mapper_right" in t:
            return Outcome(OK, "Controller motion is already being turned "
                               "into robot motion.",
                           detail="the mapping was already running")
        return Outcome(
            FAILED,
            "The part that turns your hand movement into robot movement is "
            "running but not reporting. It may be wedged.",
            detail="%d mapper process(es), nothing on its state topic"
                   % len(already),
            fix_label="Stop it so it can be started cleanly",
            fix="kill_mapper")
    for name, exe in (("mapper", "vr_pose_mapper"),
                      ("gripper", "vr_gripper_node"),
                      ("safety", "vr_safety_node"),
                      ("feedback", "vr_feedback_node")):
        w.spawn(name, ["ros2", "run", "srl_vr_teleop", exe],
                log=os.path.join(_scratch(w), "vr_bringup_%s.log" % name))
    t0 = w.now()
    want = "/vr/mapper_right"
    while w.now() - t0 < wait_s:
        w.sleep(2.0)
        t = w.topics() or []
        if want in t:
            return Outcome(OK, "Controller motion will be turned into robot "
                               "motion.",
                           detail="the mapper is reporting its state")
    return Outcome(
        FAILED,
        "The part that turns your hand movement into robot movement did not "
        "come up.",
        detail="nothing on %s after %.0f s" % (want, wait_s),
        fix_label="Show me why", fix="show_mapper_log")


def step_mapper_is_clean(w):
    """10. It is not holding anything from a previous run.

    THE FAILURE THIS CATCHES cost mode 02 every grasping task: the mapper did
    not reset between runs, so a reference, an anchor and a rate limiter from
    the last session were still in it and the miss ACCUMULATED. A fresh
    process is clean by construction -- but this button is also pressed on a
    stack that was already up, and then it is not.
    """
    st = w.mapper_state()
    if st is None:
        return Outcome(UNKNOWN, "Could not read what the controller mapping "
                                "is holding.",
                       detail="no mapper state published")
    dirty = [k for k in ("engaged", "has_reference", "has_anchor",
                         "filter_primed") if st.get(k)]
    # A CLEAN SCALE IS THE MAPPER'S OWN DECLARED DEFAULT, WHICH IT PUBLISHES.
    # It is 0.5, not 1.0. Assuming 1.0 reported a freshly reset mapper as
    # dirty for ever, and the repair it offered was the reset that had just
    # produced the value it was objecting to -- a loop that looks like a
    # broken node and is a broken check.
    scale = float(st.get("scale", 0.0) or 0.0)
    clean = st.get("scale_default")
    if clean is None:
        dirty.append("the mapping does not say what a clean scale is")
    elif abs(scale - float(clean)) > 1e-6:
        dirty.append("motion scale is %.2f, not the %.2f it starts at"
                     % (scale, float(clean)))
    if not dirty:
        return Outcome(OK, "The controller mapping is starting clean.",
                       detail="no references, no anchors, scale 1.0")
    return Outcome(
        FAILED,
        "The controller mapping is still holding settings from an earlier "
        "run. Left there, the arm drifts further from your hand every time "
        "you re-grip.",
        detail="still set: %s" % ", ".join(dirty),
        fix_label="Clear it", fix="reset_mapper",
        fix_note="drops references, anchors and the motion scale. It does NOT "
                 "clear where you are sitting -- that is a measurement, not "
                 "per-run state")


def step_observer(w, wait_s=2.0):
    """11. Somebody is standing next to the wearer with the e-stop.

    NOT REQUIRED FOR A SIM SESSION, and it says so rather than refusing:
    nothing in this sequence drives a real arm, and a bring-up that will not
    finish without a second person is a bring-up nobody uses for practice.
    It is required before any real arm moves, and the runbook is where that
    is enforced.
    """
    beats = w.observer_beats(wait_s)
    if beats is None:
        return Outcome(UNKNOWN, "Could not tell whether an observer is "
                                "present.", detail="topic unreadable")
    if beats:
        return Outcome(OK, "An observer is present and holding the e-stop.",
                       detail="%d heartbeat(s) in %.0f s" % (beats, wait_s))
    return Outcome(
        SKIPPED,
        "No observer is posted. That is fine for practice -- nothing here "
        "moves a real arm -- but a real arm will refuse to be enabled until "
        "somebody is standing next to the wearer holding the e-stop.",
        detail="no heartbeat on the observer topic",
        fix_label="How do I post one", fix="observer_help",
        fix_note="one command, run by the other person, on their own terminal")


def step_ready(w, window_s=2.0):
    """12. Ready to operate.

    ARRIVAL, NOT ADVERTISEMENT. This step checked that the pose topic was in
    the topic list, and the bridge creates its publishers at start-up -- so it
    said READY TO OPERATE on a machine with no headset in the building. That
    is the failure this whole file is written against, reproduced in its own
    last step: the operator would put the headset on, squeeze the grip, and
    nothing would move, with the window still saying ready.
    """
    t = w.topics() or []
    for topic, why in (("/joint_states",
                        "the simulation is not reporting"),
                       ("/vr/mapper_right",
                        "the controller mapping is not reporting")):
        if topic not in t:
            return Outcome(FAILED, "Almost ready: %s." % why,
                           detail="%s absent" % topic,
                           fix_label="What do I do", fix="ready_help")
    beats = w.pose_beats(window_s)
    if beats is None:
        return Outcome(UNKNOWN, "Could not tell whether the headset is "
                                "sending anything.",
                       detail="pose topic unreadable")
    if beats <= 0:
        return Outcome(
            UNKNOWN,
            "Everything on this machine is ready. The headset is not sending "
            "controller positions yet -- put it on its shelf, open the page "
            "in its own browser and press ENTER VR.",
            detail="0 controller poses in %.1f s (the topic exists; nothing "
                   "is arriving on it)" % window_s,
            fix_label="How do I do that", fix="ready_help")
    return Outcome(OK, "READY TO OPERATE.",
                   detail="%d controller poses in %.1f s, mapping up, "
                          "simulation up" % (beats, window_s))


def _scratch(w):
    d = os.environ.get("SRL_SCRATCH") or os.path.join(w.ws, ".scratch")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        return "/tmp"
    return d


# The sequence. Order is load-bearing: see step_sim_stack.
STEPS = (
    ("environment", "This window can talk to the system", step_environment),
    ("leftover_memory", "Nothing left over from an earlier run",
     step_leftover_memory),
    ("daemon", "The system answers questions about itself", step_daemon),
    ("second_stack", "Nothing is running twice", step_no_second_stack),
    ("sim", "The simulation is up", step_sim_stack),
    ("certificate", "The headset will accept this machine", step_certificate),
    ("port", "The headset's connection point is free", step_port_free),
    ("bridge", "The headset can connect", step_bridge),
    ("mapper", "Your hand becomes robot motion", step_mapper),
    ("mapper_clean", "Nothing held over from a previous run",
     step_mapper_is_clean),
    ("observer", "An observer is posted", step_observer),
    ("ready", "Ready to operate", step_ready),
)


def plan():
    return [Step(k, t, f) for k, t, f in STEPS]


def keys():
    return tuple(k for k, _t, _f in STEPS)


def verdict(results, keyed=None):
    """('ok'|'failed'|'unknown', headline). NEVER OK ON UNKNOWN.

    SKIPPED does not spoil a pass: the observer step is deliberately optional
    for a sim session and says so, and a sequence that reported "not ready"
    because nobody was standing next to a robot that is not powered would be
    a sequence people learn to ignore.

    THE ONE HEADLINE WORTH SPELLING OUT is "everything here is done and the
    headset is not on". That is the normal end of a bring-up with nobody in
    the room, and "1 step could not be checked" -- true, and useless -- sends
    the operator looking for a fault that is not there.
    """
    if not results:
        return UNKNOWN, "not started"
    bad = [r for r in results if r.state == FAILED]
    unk = [r for r in results if r.state == UNKNOWN]
    if bad:
        return FAILED, "stopped: %s" % bad[0].plain.split(".")[0]
    if keyed:
        others = [r for k, r in keyed if k != "ready"]
        ready = dict(keyed).get("ready")
        if (ready is not None and ready.state == UNKNOWN
                and all(r.state in (OK, SKIPPED) for r in others)
                and len(keyed) == len(STEPS)):
            return UNKNOWN, ("everything on this machine is ready -- "
                             "connect the headset")
    if unk:
        return UNKNOWN, "%d step%s could not be checked" % (
            len(unk), "" if len(unk) == 1 else "s")
    if len(results) < len(STEPS):
        return RUNNING, "step %d of %d" % (len(results), len(STEPS))
    return OK, "READY TO OPERATE"


# ===========================================================================
#  THE FIXES. Each returns (ok, one sentence saying what it did.)
# ===========================================================================
def _sweep_unowned(w):
    """Remove every shared-memory segment no live process has mapped.

    ONE PLACE. Both repairs need it, and a second copy is a second thing that
    can be given the wrong pattern.
    """
    n = 0
    for path in w.stale_shm_segments():
        for target in (path, path.replace("/dev/shm/", "/dev/shm/sem.")):
            try:
                os.remove(target)
                n += 1 if target == path else 0
            except OSError:
                pass
    return n


def fix_reset_daemon(w):
    """Restart the helper AND CLEAN UP AFTER IT.

    THE REPAIR HAS TO BE SELF-CONTAINED. Restarting the ros2 daemon orphans
    the segments the old one held -- measured, four or five every time -- so a
    restart that does not sweep them leaves the machine in a second broken
    state and the very next check offers to clear leftovers that the last
    repair created. Two repairs chasing each other is how an operator learns
    to distrust both.
    """
    from srl_teleop import procscan
    try:
        subprocess.run(["ros2", "daemon", "stop"], capture_output=True,
                       timeout=10)
    except Exception:                                         # noqa: BLE001
        pass
    procscan.kill_all(r"ros2cli\.daemon\.daemonize")
    try:
        subprocess.run(["ros2", "daemon", "start"], capture_output=True,
                       timeout=25)
    except Exception as e:                                    # noqa: BLE001
        return False, "Could not start it again: %r" % (e,)
    # SWEEP LAST, AFTER THE NEW ONE IS UP.
    #
    # Sweeping between the kill and the start looked right and cleared
    # nothing: the old daemon had not finished dying, so its segments were
    # still mapped and therefore still "in use" -- and a moment later it was
    # gone and they were leftovers. Measured, three runs in a row: five
    # blocks, every time, immediately after a restart that reported success.
    #
    # After the start, the old one is certainly gone and the new one has its
    # own segments mapped, so this removes exactly what was orphaned.
    w.sleep(2.0)
    swept = _sweep_unowned(w)
    return True, ("Restarted it%s. Trying again."
                  % ("" if not swept
                     else ", and cleared %d block(s) it left behind" % swept))


def fix_clear_shm(w):
    """Restart the helper, clear what nobody has open, THEN wait.

    THREE THINGS, IN THIS ORDER, and getting the order wrong was worth three
    false starts on one afternoon:

      1. RESTART THE HELPER FIRST. The ros2 daemon holds segments AND a cached
         picture of the graph, and clearing invalidates both. Restarting it
         afterwards orphans its old segments -- measured: clear 33, restart,
         and 4 fresh leftovers appear immediately.
      2. CLEAR ONLY WHAT NOBODY HAS MAPPED. Asked of the kernel via
         /proc/*/maps, not inferred from the process table. Inferring it was
         wrong in both directions: "no node running" missed the daemon, and
         "anything running" masked a killed stack's leftovers.
      3. WAIT. Removing the segments does not tell the participants that were
         using them. A stack started into a half-torn-down transport comes up
         and cannot see itself, which looks exactly like the fault just
         cleared -- so the operator clears again and goes round.
    """
    ok_daemon, msg_daemon = fix_reset_daemon(w)
    w.sleep(1.0)
    n = _sweep_unowned(w)
    if not n:
        return True, ("Nothing left to clear: every block of shared memory is "
                      "in use by something that is running.")
    w.sleep(SHM_SETTLE_S)
    return True, ("Restarted the helper that lists what is running, cleared "
                  "%d leftover block(s), and waited %.0f seconds for the "
                  "transport to settle.%s"
                  % (n, SHM_SETTLE_S,
                     "" if ok_daemon else
                     " (the helper did not restart: %s)" % msg_daemon))


def fix_free_port(w):
    """Stop whatever holds the port. The GROUP, per HARD CONSTRAINT 9."""
    owner = w.port_owner(w.port)
    if owner is None:
        return False, "Nothing is holding it any more."
    pid, cmd = owner
    import signal
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(os.getpgid(pid), sig)
        except OSError:
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        w.sleep(1.5)
        if w.port_owner(w.port) is None:
            return True, "Stopped it and took the connection back."
    return False, ("It did not stop. It is pid %d: %s" % (pid, cmd[:60]))


def fix_make_cert(w):
    try:
        p = subprocess.run(
            ["bash", os.path.join(w.ws, "scripts", "make_vr_cert.sh")],
            capture_output=True, text=True, timeout=90)
    except Exception as e:                                    # noqa: BLE001
        return False, "Could not make one: %r" % (e,)
    if p.returncode != 0:
        return False, "Making one failed. %s" % (p.stderr or "")[-120:]
    return True, "Made a new certificate for this machine's address."


def fix_reset_mapper(w):
    try:
        p = subprocess.run(["ros2", "service", "call", "/vr/reset",
                            "std_srvs/srv/Trigger", "{}"],
                           capture_output=True, text=True, timeout=25)
    except Exception as e:                                    # noqa: BLE001
        return False, "Could not clear it: %r" % (e,)
    if "success=True" not in (p.stdout or "").replace(" ", ""):
        return False, ("The controller mapping did not answer. It may not be "
                       "running.")
    return True, "Cleared. It is starting from nothing again."


def fix_kill_second_stack(w):
    from srl_teleop import real_arm_doctor as rad
    return rad.fix_kill_second_stack()


def fix_kill_stray_rsp(w):
    from srl_teleop import real_arm_doctor as rad
    return rad.fix_kill_stray_rsp()


def observer_help(w):
    return True, (
        "On the OTHER person's terminal, next to the wearer:\n\n"
        "    python3 scripts/observer_estop.py\n\n"
        "They confirm they can see the wearer and reach the e-stop, then "
        "press ENTER. It beats every 30 seconds; if they walk away it stops "
        "and the arm freezes.")


def ready_help(w):
    return True, (
        "Put the headset on its shelf with the proximity sensor taped, open "
        "the address printed by this window in the headset's own browser, "
        "accept the one certificate warning, and press ENTER VR. Poses only "
        "flow inside a session.")


def fix_kill_detached(w):
    """Stop processes that are running with no connection to anything.

    They cannot be recovered: a participant whose shared memory was removed
    does not re-attach. Leaving them costs the next start a second copy of
    everything they were.
    """
    from srl_teleop import procscan
    stopped = 0
    for rx in (r"lib/moveit_ros_move_group/move_group",
               r"lib/controller_manager/ros2_control_node",
               r"lib/srl_teleop/master_pose_node",
               r"lib/robot_state_publisher/robot_state_publisher",
               r"ros2 launch srl_teleop teleop",
               r"scripts/run_teleop\.sh"):
        got, _surv = procscan.kill_all(rx, 2, grace_s=4.0)
        stopped += len(got)
    w.sleep(2.0)
    _sweep_unowned(w)
    if not stopped:
        return False, "There was nothing to stop."
    return True, ("Stopped %d detached part(s) and cleared what they left "
                  "behind. The simulation will be started fresh." % stopped)


def fix_kill_mapper(w):
    """Stop the VR nodes so the next press starts them cleanly.

    The GROUP, per HARD CONSTRAINT 9: `ros2 run` is a wrapper and killing it
    leaves the node behind holding its topics.
    """
    from srl_teleop import procscan
    stopped = 0
    for rx in (r"lib/srl_vr_teleop/vr_pose_mapper",
               r"lib/srl_vr_teleop/vr_gripper_node",
               r"lib/srl_vr_teleop/vr_safety_node",
               r"lib/srl_vr_teleop/vr_feedback_node"):
        got, _surv = procscan.kill_all(rx, 2, grace_s=4.0)
        stopped += len(got)
    w.sleep(1.5)
    if not stopped:
        return False, "There was nothing to stop."
    return True, "Stopped %d part(s). Press the button again." % stopped


FIXES = {
    "clear_shm": fix_clear_shm,
    "kill_mapper": fix_kill_mapper,
    "kill_detached": fix_kill_detached,
    "reset_daemon": fix_reset_daemon,
    "free_port": fix_free_port,
    "make_cert": fix_make_cert,
    "reset_mapper": fix_reset_mapper,
    "kill_second_stack": fix_kill_second_stack,
    "kill_stray_rsp": fix_kill_stray_rsp,
    "observer_help": observer_help,
    "ready_help": ready_help,
}
