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
import sys
import time

OK = "ok"
FAILED = "failed"
UNKNOWN = "unknown"
RUNNING = "running"
PENDING = "pending"
SKIPPED = "skipped"
# A FOURTH ANSWER TO THE OBSERVER QUESTION, and it needed to exist.
#
# OK meant "somebody is watching", SKIPPED meant "nobody is, and that is fine
# because nothing here moves a real arm", UNKNOWN meant "nothing is saying".
# There was no way to say "nobody is watching, a named person decided that at
# a known time, and the real arm is going to move anyway" -- so an operator
# alone in the lab had no supported route past the interlock and the
# unsupported ones all end in the check being switched off for good.
#
# BYPASSED is that state. It does not spoil a pass, and it is never quiet.
BYPASSED = "bypassed"

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")
PORT = int(os.environ.get("VR_PORT", "8765"))

# HOW LONG TO WAIT AFTER CLEARING SHARED MEMORY, and it is not superstition.
# Clearing removes the segments; the participants that were using them do not
# find out instantly. A stack started into a half-torn-down transport comes up
# and cannot see itself -- which looks exactly like the problem just cleared,
# so the operator clears again and goes round. Five seconds is the number the
# runbook has carried since this was first diagnosed.
SHM_SETTLE_S = 5.0

# HOW LONG THE SIMULATION IS GIVEN, and how long after it first speaks before
# it is believed. Two different numbers because they answer two different
# questions.
#
# `/joint_states` is ADVERTISED early in the launch; the controllers,
# `move_group` and RViz are still coming up behind it for tens of seconds.
# The bring-up's own step waited for the topic and then moved on, and the
# repair button did not wait at all -- it spawned the launch, returned
# "press the button again when it settles", and the window re-ran the whole
# sequence 400 ms later against a simulation that was ten seconds into a
# minute-long start. The operator got "the simulation is not running" from a
# simulation that was starting perfectly well, and RViz was still a grey
# window when the verdict was printed.
#
# Worse than the wrong words: step 5 then found no `/joint_states` AND no
# `move_group` process yet -- because `move_group` is late in the same launch
# -- and SPAWNED A SECOND STACK. That is HARD CONSTRAINT 3 reached through
# the repair button.
SIM_WAIT_S = float(os.environ.get("SRL_VR_SIM_WAIT_S", "150"))
SIM_SETTLE_S = float(os.environ.get("SRL_VR_SIM_SETTLE_S", "10"))
# And how long RViz gets after that, which is NOT a gate: see
# `_wait_for_rviz`.
RVIZ_WAIT_S = float(os.environ.get("SRL_VR_RVIZ_WAIT_S", "60"))

# WHETHER THE LAUNCH BRINGS ITS OWN RViz. The operations window EMBEDS one
# and sets this false, because `use_rviz:=true` opened a SECOND top-level
# RViz over the window -- the 2026-08-27 finding, which fixed every stack
# spec the GUI launches and missed this one, since it is spawned from
# `vr_bringup` and not from a spec. From a terminal the default stands.
def _sim_rviz():
    # READ AT CALL TIME, not at import. The window sets this when it opens,
    # and it imports this module at start-up -- a module-level constant here
    # would be read before the window had said anything.
    return os.environ.get("SRL_VR_SIM_RVIZ", "true").lower() not in (
        "0", "false", "no")


def _master_present():
    """Is there a Teensy on the bus for `master_pose_node` to open?

    Globbed rather than remembered. The board arrives and leaves with a
    `usbipd attach` and moves between /dev/ttyACM0 and ACM1 when it does, so
    anything cached is wrong by the next session.
    """
    return bool(glob.glob("/dev/ttyACM*") or glob.glob("/dev/ttyUSB*"))


def _sim_argv(w):
    """The one command that starts the simulation. ONE place, because the
    step and the repair must start the SAME thing.

    AND IT LEAVES THE MASTER ARM OUT WHEN THERE IS NO TEENSY.

    `master_pose_node` is `respawn=True`, deliberately: a board attached
    after launch then connects on its own without restarting the stack. With
    no board on the bus at all that becomes a process dying and respawning
    every five seconds for ever. `gui_launch_specs` records what that cost on
    2026-08-26 -- five live instances, load average 8-11, the controller
    manager overrunning, every spawner timing out, `joint_state_broadcaster`
    never coming up, `/joint_states` with no publisher -- and it added a
    whole `sim_nomaster` spec to avoid it.

    THIS PATH NEVER USED THAT SPEC. `START VR TELEOP` came through here and
    started the master-ful stack, on a machine with no Teensy, every time. It
    happened twice on 2026-08-29: the stack came up, ran, and died, and the
    second time it took the operations window's own ROS context with it
    because the window was the launch's parent. The operator's report was
    "the real arm is not moving", four layers down.

    The VR operator holds controllers; the master arm is not their input.
    So when there is no board this asks for the stack without it, and says
    which it chose rather than leaving that to a process listing.
    """
    argv = [os.path.join(w.ws, "scripts", "run_teleop.sh"), "gate:=false"]
    if not _master_present():
        argv.append("master:=false")
    if not _sim_rviz():
        argv.append("use_rviz:=false")
    return argv

# What a launch that is still coming up looks like in the process table,
# BEFORE any of the nodes it starts exist to be found.
SIM_LAUNCHER_RX = r"run_teleop\.sh|ros2 launch srl_teleop"


def _wait_for_rviz(w, t0, wait_s=None):
    """Give RViz the rest of its start, and never fail on it.

    NOT A GATE. `use_rviz:=false` and a headless box are both legitimate, so
    a missing window must not stop the bring-up -- but returning the instant
    the joints report leaves the operator staring at a grey COMMANDED panel
    while the window says the simulation is running, which is what "it does
    not give RViz enough time" means. So: wait for the real window, up to a
    bound, and SAY which of the two happened either way.
    """
    wait_s = RVIZ_WAIT_S if wait_s is None else wait_s
    end = w.now() + wait_s
    while w.now() < end:
        if w.rviz_windows():
            return ", RViz drawn at %.0f s" % (w.now() - t0)
        w.sleep(3.0)
    return (", no RViz window after %.0f s (fine if it was started without "
            "one)" % wait_s)


def _wait_for_sim(w, wait_s=None, settle_s=None):
    """Wait until the simulation is REPORTING, not merely listed.

    Returns `(ok, seconds, detail)`.

    LISTED IS NOT REPORTING, and reporting is not LOADED. The topic exists
    the moment the launch advertises it, the first joint update arrives
    before the controllers have all spawned, and RViz finishes last. So this
    waits for the topic, then gives the rest of the launch `settle_s`, and
    only then counts arrivals -- and a count of zero after that keeps
    waiting rather than returning a verdict on a stack that is mid-start.
    """
    wait_s = SIM_WAIT_S if wait_s is None else wait_s
    settle_s = SIM_SETTLE_S if settle_s is None else settle_s
    t0 = w.now()
    listed_at = None
    while w.now() - t0 < wait_s:
        w.sleep(3.0)
        topics = w.topics() or []
        if "/joint_states" not in topics:
            continue
        if listed_at is None:
            listed_at = w.now()
        if w.now() - listed_at < settle_s:
            continue
        beats = w.joint_beats(2.0)
        if beats:
            rviz = _wait_for_rviz(w, t0)
            return True, w.now() - t0, (
                "reporting after %.0f s, %d joint updates in 2 s%s"
                % (w.now() - t0, beats, rviz))
    if listed_at is None:
        return False, w.now() - t0, (
            "no /joint_states after %.0f s" % (w.now() - t0))
    return False, w.now() - t0, (
        "/joint_states listed after %.0f s but still nothing arriving on it "
        "at %.0f s" % (listed_at - t0, w.now() - t0))


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

    # ASK THE GRAPH, NOT THE DAEMON.
    #
    # `ros2 topic list` goes through the ros2 daemon, which caches network
    # state and on this box goes stale constantly. Using it for LIVENESS
    # checks was wrong three separate times in one lab session: it reported
    # the simulation absent while 34 nodes were running, it reported the
    # controller mapping absent while it was publishing at 2 Hz, and it made
    # the final step say "not ready" for the same reason. Each time the
    # operator was sent to look at a component that was working.
    #
    # A fresh participant is the thing that matters anyway: the question is
    # not "does the daemon remember this topic" but "can something that
    # starts now talk to it". Run in a subprocess so it cannot collide with
    # an rclpy context the caller already owns -- the GUI has one.
    # WAIT UNTIL THE ANSWER STOPS CHANGING, don't sample for a fixed 1.5 s.
    #
    # Discovery over shared memory is not instant, and a fixed short window
    # reported /joint_states ABSENT and then PRESENT one second later against
    # a simulation that was fine throughout. Declaring something missing from
    # a sample too short to see it is the same mistake as every other one
    # today, and here it would have sent the operator to restart a healthy
    # stack.
    #
    # So: spin until the topic count has been unchanged for a second, capped.
    # A settled count is evidence; an elapsed timer is not.
    _TOPIC_PROBE = (
        "import rclpy, time\n"
        "from rclpy.node import Node\n"
        "rclpy.init()\n"
        "n = Node('srl_topic_probe')\n"
        "seen, stable_since, t0 = 0, None, time.time()\n"
        "while time.time() - t0 < 8.0:\n"
        "    rclpy.spin_once(n, timeout_sec=0.1)\n"
        "    cur = len(n.get_topic_names_and_types())\n"
        "    if cur != seen:\n"
        "        seen, stable_since = cur, time.time()\n"
        "    elif stable_since and time.time() - stable_since > 1.0 "
        "and cur > 0:\n"
        "        break\n"
        "print('\\n'.join(sorted(x for x, _ in n.get_topic_names_and_types())))\n"
        "rclpy.shutdown()\n")

    # COUNTING ARRIVALS WITH `ros2 topic echo` DOES NOT WORK ON THIS BOX, AND
    # IT IS ACTIVELY HARMFUL.
    #
    # Measured 2026-08-21 against a stack that a direct rclpy subscriber read
    # at 98 Hz (784 messages in 8 s):
    #
    #     timeout 2.5 ros2 topic echo /joint_states --field header.stamp.sec
    #     -> "Terminated", rc 143, ZERO lines
    #
    # at every window from 1 to 4 seconds. The CLI never got as far as
    # delivering a message. So `joint_beats` returned 0 against a healthy
    # simulation, `step_ready` believed it, and the button reported
    # "the simulation is not saying where the robot is" about a simulation
    # that was saying it 98 times a second. The instrument was the fault --
    # docs/ENGINEERING_LOG.md's standing rule, and it caught this one on the first check.
    #
    # And the plain `timeout` is the second half: SIGTERM hard-kills a DDS
    # participant holding shared memory, which is the exact hazard
    # VR_REAL_ARM_AS_RUN.md records ("Never run `ros2 topic hz`", ten call
    # sites changed to `timeout -s INT`). Three of these ran on every press of
    # the button, between step 5 and step 12 -- which is precisely the window
    # in which the simulation went from reporting to silent.
    #
    # So: a real subscriber, in a subprocess, exiting on its own. Same shape
    # as `_TOPIC_PROBE`, which has always worked. Nothing is signalled.
    _COUNT_PROBE = (
        "import sys, time, rclpy\n"
        "from rclpy.node import Node\n"
        "from rosidl_runtime_py.utilities import get_message\n"
        "topic, window = sys.argv[1], float(sys.argv[2])\n"
        "rclpy.init()\n"
        "n = Node('srl_count_probe')\n"
        "typ, t0 = None, time.time()\n"
        "while time.time() - t0 < 5.0 and typ is None:\n"
        "    for name, types in n.get_topic_names_and_types():\n"
        "        if name == topic and types:\n"
        "            typ = types[0]\n"
        "            break\n"
        "    if typ is None:\n"
        "        rclpy.spin_once(n, timeout_sec=0.1)\n"
        "if typ is None:\n"
        "    print('NOTOPIC')\n"
        "    sys.exit(0)\n"
        "seen = [0]\n"
        "n.create_subscription(get_message(typ), topic,\n"
        "                      lambda _m: seen.__setitem__(0, seen[0] + 1), 10)\n"
        "t1 = time.time()\n"
        "while time.time() - t1 < window:\n"
        "    rclpy.spin_once(n, timeout_sec=0.05)\n"
        "print(seen[0])\n"
        "rclpy.shutdown()\n")

    def _count(self, topic, window_s):
        """Messages that ARRIVED on `topic` in `window_s`. None = unaskable.

        `NOTOPIC` is reported as 0, not None: the probe ran, waited five
        seconds for the topic to appear in the graph, and it never did. That
        is an answer.
        """
        try:
            p = subprocess.run(
                [sys.executable, "-c", self._COUNT_PROBE, topic,
                 "%.1f" % window_s],
                capture_output=True, text=True, timeout=window_s + 20.0)
        except Exception:                                     # noqa: BLE001
            return None
        out = (p.stdout or "").strip().splitlines()
        if not out:
            return None
        last = out[-1].strip()
        if last == "NOTOPIC":
            return 0
        try:
            return int(last)
        except ValueError:
            return None

    def topics(self, timeout_s=30):
        try:
            p = subprocess.run([sys.executable, "-c", self._TOPIC_PROBE],
                               capture_output=True, text=True,
                               timeout=timeout_s)
        except Exception:                                     # noqa: BLE001
            return None
        out = [x.strip() for x in p.stdout.splitlines() if x.strip()]
        return out if out else None

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

    def lan_ips(self):
        """Every IPv4 address this machine has, best candidate FIRST.

        THE FIRST ONE `hostname -I` RETURNS IS NOT NECESSARILY THE RIGHT ONE.
        On this box it is 172.26.244.255 -- the interface carrying the default
        route, a NAT or corporate link -- while the lab switch the arms are on
        is 192.168.1.25 on a different interface. Handing the operator the
        first address sends them to a page the headset cannot load, and the
        symptom inside a headset is indistinguishable from a firewall problem
        or a bad certificate.

        Ordered by "is this the network the robot is on": a 192.168.x address
        first, because that is what this lab uses, then anything else. The
        caller shows the whole list rather than trusting the order blindly.
        """
        try:
            out = subprocess.run(["hostname", "-I"], capture_output=True,
                                 text=True, timeout=6).stdout
        except Exception:                                     # noqa: BLE001
            return []
        # IPv4 ONLY. `t[0].isdigit()` let the IPv6 addresses through -- they
        # start with a hex digit -- and the certificate check then demanded
        # that the certificate carry them as IP entries, which it never does:
        # `make_vr_cert.sh` puts IPv6 in as DNS names. The certificate step
        # duly failed on a certificate that was completely correct.
        ips = [t for t in out.split()
               if t and "." in t and ":" not in t
               and t[0].isdigit() and not t.startswith("127.")]
        return sorted(ips, key=lambda t: (not t.startswith("192.168."), t))

    def lan_ip(self):
        ips = self.lan_ips()
        return ips[0] if ips else None

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

        COUNTED, NOT LISTED. `quest_bridge_node` creates its publishers
        at start-up, so the pose topic EXISTS the moment the bridge
        does -- with or without a headset. Checking the topic list
        therefore said READY TO OPERATE on a machine with no headset in
        the building.
        """
        return self._count('/vr/controller_pose_right', window_s)

    def joint_beats(self, window_s=2.0):
        """How many joint-state messages ARRIVED in `window_s`, or None.

        COUNTED, NOT LISTED. `/joint_states` is advertised the moment
        anything publishes OR SUBSCRIBES to it, so its presence in a
        topic list says nothing about whether the robot is reporting
        where it is. Measured 2026-08-20: the window said "the
        simulation is running", the panel showed all fourteen joints as
        "--", and both were reading the same topic list.

        None means the question could not be asked. It does NOT mean
        zero, and the callers must not collapse the two.
        """
        return self._count('/joint_states', window_s)

    def observer_bypass(self):
        """(True, record) if an operator has declared they are working alone.

        Read through `observer_bypass`, which is the one place that decides
        what counts as a live grant, so this cannot drift from what
        `vr_safety_node` enforces. Any error is False -- see that module.
        """
        try:
            from srl_teleop import observer_bypass
            return observer_bypass.active()
        except Exception as e:                                # noqa: BLE001
            return False, "the bypass could not be read (%r)" % (e,)

    def observer_beats(self, window_s=2.0):
        """How many observer heartbeats arrived in `window_s`, or None.

        COUNTED, NOT LATCHED. The observer interlock is a heartbeat, so
        "present" is a rate and not a value -- a single retained True
        from a publisher that has since gone away is exactly the state
        this is meant to distinguish.
        """
        return self._count('/vr/observer_estop_present', window_s)

    def rviz_windows(self):
        """How many REAL RViz windows are drawn. See `rviz_windows.py`.

        The process table cannot answer "has RViz loaded" -- the process is
        there from the first second of a start and the window it draws in is
        built last, which on WSLg is tens of seconds later. That gap is what
        the operator saw: a grey panel beside a verdict.
        """
        try:
            from srl_teleop.rviz_windows import real_windows
            return len(real_windows())
        except Exception:                                     # noqa: BLE001
            return 0

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

    THIS STEP CLEARS RATHER THAN ASKS, and that is a correction.

    It used to stop the whole bring-up and offer a button. In the lab that
    was wrong twice over. Removing a segment NOBODY HAS MAPPED is safe by
    definition -- that is the entire content of "unowned" -- so there is
    nothing for the operator to decide. And because the check has a 20 s age
    floor, the count CHANGES between attempts as segments left by processes
    that have just exited age past it: an operator pressing the button twice
    saw "2 blocks", then "10 blocks", and reasonably concluded something was
    creating them faster than they could be cleared. Nothing was. They were
    the same leftovers, arriving at the threshold.

    A fault that is always safe to repair, and whose count moves while you
    look at it, must be repaired rather than reported. So: clear, and carry
    on. It only FAILS if clearing did not work, which means something is
    genuinely wrong.
    """
    stale = w.stale_shm_segments()
    total = len(w.shm_segments())
    if not stale:
        return Outcome(OK, "Shared memory is in use by what is running now, "
                           "which is normal."
                       if total else "Nothing left over from an earlier run.",
                       detail="%d segment(s), none unowned" % total)
    if _anything_running(w):
        # NOT A FAILURE, AND NOT SOMETHING TO CLEAR. Leftovers alongside a
        # running system are only a problem for the NEXT start, and removing
        # them now can take the running system's own memory with them.
        return Outcome(
            OK,
            "There are %d blocks of memory that look left over, and they are "
            "being left alone because things are running. They will be "
            "cleared next time everything is stopped." % len(stale),
            detail="%d of %d unowned; %d process(es) running, so NOT cleared"
                   % (len(stale), total, len(_anything_running(w))))
    ok, msg = fix_clear_shm(w)
    after = w.stale_shm_segments()
    if not after:
        return Outcome(
            OK,
            "An earlier run had left %d blocks of memory behind. They have "
            "been cleared." % len(stale),
            detail=msg)
    return Outcome(
        FAILED,
        "An earlier run left %d blocks of memory behind and %d of them could "
        "not be cleared. Something still has a claim on them that this window "
        "cannot see." % (len(stale), len(after)),
        detail="%s; %d still unowned afterwards" % (msg, len(after)),
        fix_label="Show me what is holding them", fix="show_shm")


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


def step_sim_stack(w, wait_s=None):
    """5. The simulation is up. START IT if it is not.

    THE ORDER MATTERS AND IT IS NOT ARBITRARY. `vr_pose_mapper` refuses to
    engage without robot TF, and the symptom of starting it first is a clutch
    that silently does nothing -- the operator squeezes, nothing moves, and
    nothing anywhere says why. So the simulation goes up first and this step
    WAITS for it rather than assuming.
    """
    wait_s = SIM_WAIT_S if wait_s is None else wait_s
    topics = w.topics()
    if topics is None:
        return Outcome(UNKNOWN, "Could not tell whether the simulation is "
                                "running.", detail="topic list unavailable")
    if "/joint_states" in topics:
        # LISTED IS NOT REPORTING. `/joint_states` appears in the topic list
        # as soon as anything publishes OR SUBSCRIBES to it, so this test
        # passed against a stack whose controllers had not come up -- and the
        # operator got "the simulation is running" beside fourteen dashes.
        beats = w.joint_beats(2.0)
        if beats is None:
            return Outcome(UNKNOWN, "The simulation is listed, but this "
                                    "window could not read anything from it.",
                           detail="/joint_states listed; arrival count "
                                  "unavailable",
                           fix_label="Restart the helper that lists what is "
                                     "running", fix="reset_daemon")
        if beats > 0:
            return Outcome(OK, "The simulation is running and reporting where "
                               "the robot is.",
                           detail="%d topics, %d joint updates in 2 s"
                                  % (len(topics), beats))
        # SILENT, OR STILL COMING UP. The topic is advertised early in the
        # launch and the first joint update is not; between the two this
        # step used to declare the simulation broken and offer to restart
        # the very thing that was starting. If a launch is in the process
        # table, wait for it before saying anything.
        if w.procs(SIM_LAUNCHER_RX):
            ok, _secs, why = _wait_for_sim(w, wait_s)
            if ok:
                return Outcome(OK, "The simulation finished starting and is "
                                   "running.", detail=why)
            return Outcome(
                FAILED,
                "The simulation has been starting for %.0f s and is still "
                "not saying where the robot is." % wait_s,
                detail=why,
                fix_label="Show me why", fix="show_sim_log",
                fix_note="opens what the simulation printed while it was "
                         "starting")
        return Outcome(
            FAILED,
            "The simulation is running but it is not saying where the robot "
            "is. Nothing that needs to know the robot's position will work, "
            "and the panel will show every joint as a dash.",
            detail="/joint_states listed, 0 messages in 2 s",
            fix_label="Restart the simulation", fix="restart_sim",
            fix_note="stops it and starts it again; it takes about two "
                     "minutes and this window waits for it")

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
    # A LAUNCH ALREADY COMING UP IS NOT AN ABSENT ONE. `move_group` is late
    # in `run_teleop.sh`, so for the first tens of seconds of a start there
    # is no `move_group` to find and no `/joint_states` to list -- which is
    # byte for byte what a machine with no simulation looks like. Spawning
    # here put a SECOND stack on top of a starting one, which is HARD
    # CONSTRAINT 3. Ask for the launcher itself, and wait for it instead.
    starting = w.procs(SIM_LAUNCHER_RX)
    if starting:
        w.note("a simulation launch is already coming up (%s); waiting for "
               "it rather than starting a second one"
               % ", ".join(str(p) for p, _ in starting))
    else:
        w.spawn("simulation", _sim_argv(w),
                log=os.path.join(_scratch(w), "vr_bringup_sim.log"))
    ok, _secs, why = _wait_for_sim(w, wait_s)
    if ok:
        return Outcome(
            OK,
            "The simulation %s and is running."
            % ("finished starting" if starting else "started"),
            detail=why)
    if w.procs(r"lib/moveit_ros_move_group/move_group"):
        return Outcome(
            FAILED,
            "The simulation started but this window still cannot see it. The "
            "answer is stale rather than wrong.",
            detail="processes present, %s" % why,
            fix_label="Restart the helper that lists what is running",
            fix="reset_daemon")
    return Outcome(
        FAILED,
        "The simulation did not finish starting. Nothing else can go up until "
        "it does, because the controller mapping needs to know where the "
        "robot is.",
        detail="no simulation process; %s" % why,
        fix_label="Show me why", fix="show_sim_log",
        fix_note="opens what the simulation printed while it was starting")


def step_certificate(w):
    """6. The headset will accept the page."""
    ips = w.lan_ips() if hasattr(w, "lan_ips") else (
        [w.lan_ip()] if w.lan_ip() else [])
    if not ips:
        return Outcome(FAILED, "This machine has no address on the network, "
                               "so nothing can reach it.",
                       detail="no non-loopback IPv4 address")
    crt, key = w.cert_paths()
    # EVERY ADDRESS, not just the preferred one. The headset may be on either
    # network, and a certificate that covers the one we happen to prefer and
    # not the one the headset is on fails inside the headset -- where the
    # message reads as a broken certificate rather than a missing address.
    results = [w.cert_covers(crt, i) for i in ips]
    exists = any(e for e, _c, _d in results)
    covers = all(c for _e, c, _d in results)
    ip = ips[0]
    detail = "%s; addresses %s" % (results[0][2], ", ".join(ips))
    if exists and covers:
        return Outcome(OK, "The headset will accept this machine's page.",
                       detail="%s covers all of %s" % (crt, ", ".join(ips)))
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
            w.ws, "src/srl_vr_teleop/web"),
        # The link's own measured behaviour: 81 Hz median with ONE stall of
        # 1.14 s (VR_REAL_ARM_AS_RUN.md). The default 0.2 s watchdog turned
        # that stall into a dropped clutch mid-motion. Same default as
        # start_vr_wifi.sh, same env override.
        "-p", "stale_timeout_s:=%s" % os.environ.get(
            "VR_LINK_TIMEOUT", "0.6")],
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
        argv = ["ros2", "run", "srl_vr_teleop", exe]
        if exe == "vr_safety_node":
            # THE SAME GATES AS start_vr_wifi.sh, FROM THE SAME ENVIRONMENT.
            # This spawn used to be bare, so the one-button path always ran
            # the shut defaults -- the GUI's "working alone" and "headset is
            # worn" ticks reached the script route and silently not this
            # one, which is two behaviours behind one button.
            e = os.environ
            argv += ["--ros-args",
                     "-p", "require_observer_estop:=%s"
                     % e.get("VR_REQUIRE_OBSERVER", "true"),
                     "-p", "allow_real_arm:=%s"
                     % e.get("VR_ALLOW_REAL_ARM", "false"),
                     "-p", "allow_real_arm_without_observer:=%s"
                     % e.get("VR_ALLOW_REAL_NO_OBSERVER", "false"),
                     "-p", "watch_tracking_reference:=%s"
                     % e.get("VR_WATCH_REFERENCE", "true"),
                     "-p", "tracking_timeout_s:=%s"
                     % e.get("VR_TRACKING_TIMEOUT", "0.6")]
        w.spawn(name, argv,
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
    # THE CHECK IS ASKED FIRST, ALWAYS, AND ITS ANSWER IS RECORDED.
    #
    # The bypass does not skip the question. If somebody IS posted, that is
    # what this reports, bypass or no bypass -- otherwise taking the bypass
    # would hide a real observer and the operator would never learn that the
    # heartbeat was working.
    beats = w.observer_beats(wait_s)
    if beats:
        return Outcome(OK, "An observer is present and holding the e-stop.",
                       detail="%d heartbeat(s) in %.0f s" % (beats, wait_s))
    on, info = w.observer_bypass()
    if on:
        return Outcome(
            BYPASSED,
            "WORKING ALONE -- the observer requirement is bypassed for this "
            "session. Nobody is holding an e-stop. If the arm does something "
            "you did not ask for, you are the only person who can stop it.",
            detail="bypass taken by %s at %s; %s"
                   % (info.get("who", "?"), info.get("granted_at_iso", "?"),
                      "no observer heartbeat"
                      if beats == 0 else "observer topic unreadable"),
            fix_label="Cancel the bypass", fix="cancel_bypass",
            fix_note="puts the observer requirement back for this session")
    if beats is None:
        return Outcome(UNKNOWN, "Could not tell whether an observer is "
                                "present.", detail="topic unreadable")
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
    # "COULD NOT ASK" IS NOT "IT IS NOT THERE", AND THIS STEP USED TO SAY IT
    # WAS.
    #
    # `w.topics()` returns None when the probe fails or times out, and this
    # read `w.topics() or []` -- so a failed probe became an empty list, and
    # the very next line reported, confidently and by name, that THE
    # SIMULATION IS NOT REPORTING. That is how the button printed
    # "ALMOST READY: THE SIMULATION IS NOT REPORTING" in the same run that
    # step 5 had logged "the simulation is running": same predicate, one
    # working probe and one failed one, and only the second was allowed to
    # sound certain. It is docs/ENGINEERING_LOG.md's worst combination -- false, specific
    # and confident -- and it sends the operator to the wrong log.
    t = w.topics()
    if t is None:
        return Outcome(
            UNKNOWN,
            "This window could not get an answer out of the system just now, "
            "so it cannot tell you whether everything is ready. Nothing is "
            "known to be wrong.",
            detail="topic probe returned nothing",
            fix_label="Restart the helper that lists what is running",
            fix="reset_daemon")
    if "/vr/mapper_right" not in t:
        return Outcome(FAILED, "Almost ready: the controller mapping is not "
                               "reporting.",
                       detail="/vr/mapper_right absent",
                       fix_label="What do I do", fix="ready_help")
    if "/joint_states" not in t:
        return Outcome(FAILED, "Almost ready: the simulation is not "
                               "reporting.",
                       detail="/joint_states absent",
                       fix_label="What do I do", fix="ready_help")
    # And listed is still not reporting, here as in step 5.
    jb = w.joint_beats(1.0)
    if jb is not None and jb <= 0:
        return Outcome(
            FAILED,
            "Almost ready: the simulation is listed but is not saying where "
            "the robot is, so every joint reads as a dash.",
            detail="/joint_states listed, 0 messages in 1 s",
            fix_label="Restart the simulation", fix="restart_sim")
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
    # A BYPASSED OBSERVER IS SAID OUT LOUD IN THE HEADLINE. Folding it into
    # "READY TO OPERATE" would make the one run with nobody watching look
    # exactly like every other run, on the one line the operator actually
    # reads.
    alone = any(r.state == BYPASSED for r in results)
    if keyed:
        others = [r for k, r in keyed if k != "ready"]
        ready = dict(keyed).get("ready")
        if (ready is not None and ready.state == UNKNOWN
                and all(r.state in (OK, SKIPPED) for r in others)
                and len(keyed) == len(STEPS)):
            return UNKNOWN, ("everything on this machine is ready -- "
                             "connect the headset"
                             + (" (WORKING ALONE: no observer)"
                                if alone else ""))
    if unk:
        return UNKNOWN, "%d step%s could not be checked" % (
            len(unk), "" if len(unk) == 1 else "s")
    if len(results) < len(STEPS):
        return RUNNING, "step %d of %d" % (len(results), len(STEPS))
    if alone:
        return OK, "READY TO OPERATE -- WORKING ALONE, no observer posted"
    return OK, "READY TO OPERATE"


# ===========================================================================
#  THE FIXES. Each returns (ok, one sentence saying what it did.)
# ===========================================================================
def _anything_running(w):
    """Any ROS process at all, including the daemon and the GUI.

    THE GUARD THAT SHOULD NEVER HAVE BEEN REMOVED. See `_sweep_unowned`.
    """
    return w.procs(r"(/opt/ros/[a-z]+/lib/[^/ ]+/"
                   r"|/install/[A-Za-z0-9_]+/lib/[A-Za-z0-9_]+/"
                   r"|ros2cli\.daemon"
                   r"|scripts/srl_gui\.py"
                   r"|/rviz2)")


def _sweep_unowned(w):
    """Remove every shared-memory segment no live process has mapped.

    ONE PLACE. Both repairs need it, and a second copy is a second thing that
    can be given the wrong pattern.
    """
    # NEVER WHILE ANYTHING IS RUNNING. This guard was removed on the grounds
    # that "nobody has it mapped" is a better test than "nothing is running",
    # and that reasoning cost a working stack in a lab session: a fresh
    # participant went from seeing 34 nodes to seeing 7, with zero publishers
    # on /tf and /joint_states, while every controller was still active.
    #
    # A live Fast DDS participant's segment CAN be absent from /proc/*/maps at
    # the instant it is sampled, and the 20 s age floor does not protect it --
    # a stack that has been up for an hour has hour-old segments. Ownership
    # narrows what is removed; it does not make removal safe.
    #
    # docs/ENGINEERING_LOG.md HARD CONSTRAINT 5 and env.sh both said "with the stack
    # STOPPED". They were right and this file overrode them with an
    # inference.
    live = _anything_running(w)
    if live:
        return 0
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


def show_shm(w):
    """Name what is still there, since clearing did not remove it."""
    left = w.stale_shm_segments()
    return True, ("%d block(s) are still there after being cleared. Either "
                  "this window may not remove them, or something is "
                  "recreating them. The first two are:\n  %s"
                  % (len(left), "\n  ".join(os.path.basename(x)
                                             for x in left[:2])))


def fix_restart_sim(w):
    """Stop the simulation and start it again.

    For the state the button could not previously describe: the processes are
    up, the topic is advertised, and nothing is arriving. Restarting the
    daemon does not help that one -- the picture is not stale, the robot
    really is silent -- so it needs a repair of its own rather than being
    routed to `reset_daemon` because that is the repair that exists.
    """
    from srl_teleop import procscan
    stopped = 0
    for pat in (r"ros2 launch srl_teleop",
                r"lib/moveit_ros_move_group/move_group",
                r"lib/controller_manager/ros2_control_node",
                r"robot_state_publisher"):
        try:
            stopped += len(procscan.kill_all(pat) or [])
        except Exception:                                     # noqa: BLE001
            pass
    w.sleep(5.0)
    w.spawn("simulation", _sim_argv(w),
            log=os.path.join(_scratch(w), "vr_bringup_sim.log"))
    # AND WAIT FOR IT. This used to return the moment the launch was
    # spawned, with "press the button again when it settles" -- and the
    # window did not wait for the operator, it re-ran the whole sequence
    # 400 ms later. So the repair reported success, the sequence reported
    # the simulation absent, and RViz was still a grey window while both
    # sentences were on screen. A repair that returns before it has
    # repaired anything is the "feature present but does nothing" row of
    # docs/ENGINEERING_LOG.md, in the button whose whole job is to fix this.
    ok, secs, why = _wait_for_sim(w)
    if ok:
        return True, ("Stopped %d part(s), started the simulation again and "
                      "waited for it: %s." % (stopped, why))
    return False, ("Stopped %d part(s) and started the simulation again, but "
                   "after %.0f s it is still not saying where the robot is "
                   "(%s). Open what it printed while it was starting."
                   % (stopped, secs, why))


def fix_cancel_bypass(w):
    """Put the observer requirement back. Always available, never automatic."""
    try:
        from srl_teleop import observer_bypass
        had = observer_bypass.clear(who="gui", note="cancelled by operator")
    except Exception as e:                                    # noqa: BLE001
        return False, "Could not cancel it: %r" % (e,)
    if not had:
        return True, "There was no bypass in force."
    return True, ("The observer requirement is back. Press the button again; "
                  "it will now ask for an observer.")


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
    "show_shm": show_shm,
    "observer_help": observer_help,
    "cancel_bypass": fix_cancel_bypass,
    "restart_sim": fix_restart_sim,
    "ready_help": ready_help,
}
