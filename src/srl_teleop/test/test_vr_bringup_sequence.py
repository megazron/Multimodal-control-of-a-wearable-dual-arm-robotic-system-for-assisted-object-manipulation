"""Break the VR bring-up eight ways and require it to say which, in English.

WHAT THIS IS FOR. The one button runs twelve steps in an order that matters,
and every one of them can half-fail. The failure that costs a lab session is
not a step that fails -- it is a step that fails QUIETLY, or one that fails
with a topic name in it, because both send the operator to the wrong place
with a robot powered.

So each of the eight failures the brief names is INJECTED through `World`, and
each is required to:

  1. be caught -- state FAILED or UNKNOWN, never OK;
  2. say what is wrong in words a tired person can act on;
  3. offer a fix, where a fix exists;
  4. contain NO topic name, NO path fragment, NO exception text in the part
     the operator reads.

And the control: a healthy world must reach READY TO OPERATE. Without that,
a sequencer that refused everything would pass this entire file.

NOTHING HERE TOUCHES THE MACHINE. No port is bound, no daemon killed, no
certificate deleted, no shared memory cleared. That is the point of `World`.
"""
from srl_teleop import vr_bringup as V


# Words that must never reach the operator. Topic names, paths, Python.
BANNED = ("/vr/", "/joint_states", "rclpy", "Traceback", "fastrtps",
          "/dev/shm", "ros2 ", "Exception", "None", "quest_bridge_node",
          "vr_pose_mapper", ".py", "8765")


class FakeWorld(V.World):
    """A machine described by keyword arguments."""

    def __init__(self, healthy=True, **kw):
        V.World.__init__(self, ws="/ws", port=8765)
        base = dict(
            env={"FASTDDS_BUILTIN_TRANSPORTS": "SHM",
                 "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                 "ROS_DOMAIN_ID": "0"},
            procs={}, shm=[], stale_shm=[],
            nodes=(["/move_group"], False),
            topics=["/joint_states", "/vr/controller_pose_right",
                    "/vr/mapper_right"],
            port_owner=None, lan_ip="192.168.1.20",
            cert=(True, True, "IP Address:192.168.1.20"),
            mapper={"engaged": False, "has_reference": False,
                    "has_anchor": False, "filter_primed": False,
                    "scale": 0.5, "scale_default": 0.5},
            # A HEALTHY MACHINE IS ONE WHOSE ROBOT IS SAYING WHERE IT IS.
            # `joint_beats=None` models the probe failing, `0` models the
            # state the operator actually hit: topic listed, nothing on it.
            joint_beats=40,
            # Nobody has declared they are working alone, by default. The
            # bypass must never be what a test gets for not mentioning it.
            bypass=(False, "no bypass has been taken"),
            beats=10, pose_beats=30, bridge_dies=False,
            nodes_direct=["/move_group"],
            spawn_ok=True,
            # RViz IS DRAWN by default. A fake that says no
            # window exists makes every start wait out the
            # RViz bound for nothing.
            rviz_windows=1,
            sim_starts=True)
        base.update(kw)
        self.k = base
        self.slept = 0.0
        self._t = 0.0

    def env(self):
        return dict(self.k["env"])

    def procs(self, pattern):
        for key, val in self.k["procs"].items():
            if key in pattern:
                return list(val)
        return []

    def shm_segments(self):
        return list(self.k["shm"])

    def stale_shm_segments(self):
        return list(self.k["stale_shm"])

    def nodes(self, timeout_s=8):
        return self.k["nodes"]

    def nodes_direct(self, timeout_s=25):
        return self.k["nodes_direct"]

    def topics(self, timeout_s=10):
        t = self.k["topics"]
        return None if t is None else list(t)

    def joint_beats(self, window_s=2.0):
        return self.k["joint_beats"]

    def rviz_windows(self):
        return self.k["rviz_windows"]

    def observer_bypass(self):
        return self.k["bypass"]

    def port_owner(self, port):
        return self.k["port_owner"]

    def lan_ip(self):
        return self.k["lan_ip"]

    def cert_paths(self):
        return "/certs/vr.crt", "/certs/vr.key"

    def cert_covers(self, crt, ip):
        return self.k["cert"]

    def mapper_state(self, timeout_s=8):
        return self.k["mapper"]

    def observer_beats(self, window_s=2.0):
        return self.k["beats"]

    def pose_beats(self, window_s=2.0):
        return self.k["pose_beats"]

    def spawn(self, name, argv, log=None):
        # THE FAKE MODELS WHAT STARTING SOMETHING DOES, not just that it was
        # asked for. The port is FREE before the bridge starts and BOUND
        # after -- which is the whole reason `port` runs before `bridge` --
        # so a fake that leaves it free for ever makes the healthy case fail
        # for a reason the real machine would never have.
        self.started.append((name, None))
        if name == "bridge" and not self.k["bridge_dies"]:
            self.k["port_owner"] = (999, "quest bridge")
        if name == "simulation" and self.k["sim_starts"]:
            t = list(self.k["topics"] or [])
            if "/joint_states" not in t:
                t.append("/joint_states")
            self.k["topics"] = t
        return None

    def alive(self, p):
        return not self.k["bridge_dies"]

    def sleep(self, s):
        # ADVANCE THE CLOCK INSTEAD OF WAITING. `now()` reads this, so a
        # 90-second wait for the simulation costs the test nothing and the
        # TIMEOUT behaviour is still exercised.
        self.slept += s
        self._t += s

    def now(self):
        return self._t


def run_all(world):
    return [(s.key, s.run(world)) for s in V.plan()]


def outcome(world, key):
    for k, t, fn in V.STEPS:
        if k == key:
            return fn(world)
    raise KeyError(key)


# ===================================================================== 0
def test_a_healthy_machine_reaches_ready_to_operate():
    """THE CONTROL. Without it, a sequencer that refused everything would
    pass every other test in this file."""
    w = FakeWorld()
    res = run_all(w)
    bad = [(k, o.state, o.plain) for k, o in res
           if o.state not in (V.OK, V.SKIPPED)]
    assert not bad, bad
    state, head = V.verdict([o for _k, o in res])
    assert state == V.OK and "READY" in head


def test_the_steps_run_in_an_order_that_matters():
    """The simulation must be up before the controller mapping starts.

    Not cosmetic: `vr_pose_mapper` refuses to engage without robot TF, and
    the symptom of starting it first is a clutch that silently does nothing.
    """
    order = list(V.keys())
    assert order.index("sim") < order.index("mapper")
    assert order.index("certificate") < order.index("bridge")
    assert order.index("port") < order.index("bridge")
    assert order.index("bridge") < order.index("mapper")
    assert order[-1] == "ready"


# ============================================================ INJECTIONS
def test_stale_shared_memory_is_CLEARED_rather_than_reported(monkeypatch):
    """FOUND IN THE LAB. Removing a segment nobody has mapped is safe by
    definition, so there is nothing for the operator to decide -- and because
    the check has a 20 s age floor, the COUNT MOVES WHILE YOU LOOK AT IT as
    leftovers age past the threshold. An operator pressing the button twice
    saw "2 blocks" then "10 blocks" and reasonably concluded something was
    creating them faster than they could be cleared. Nothing was.

    A fault that is always safe to repair, and whose count changes between
    attempts, must be repaired rather than reported.
    """
    cleared = {}

    def fake_clear(w):
        cleared["n"] = len(w.k["stale_shm"])
        w.k["stale_shm"] = []
        return True, "cleared %d" % cleared["n"]
    monkeypatch.setattr(V, "fix_clear_shm", fake_clear)

    w = FakeWorld(shm=["/dev/shm/fastrtps_a", "/dev/shm/sem.fastrtps_a"],
                  stale_shm=["/dev/shm/fastrtps_a"], procs={})
    o = outcome(w, "leftover_memory")
    assert o.state == V.OK, (
        "the sequence stopped for something it could simply have fixed")
    assert cleared.get("n") == 1
    assert "cleared" in o.plain.lower()


def test_leftovers_that_will_not_clear_DO_stop_the_sequence(monkeypatch):
    """The other half: if clearing does not work, something is genuinely
    wrong and carrying on would start a stack into a broken transport."""
    monkeypatch.setattr(V, "fix_clear_shm",
                        lambda w: (True, "tried and failed"))
    w = FakeWorld(shm=["/dev/shm/fastrtps_a"],
                  stale_shm=["/dev/shm/fastrtps_a"], procs={})
    o = outcome(w, "leftover_memory")
    assert o.state == V.FAILED
    assert o.has_fix and o.fix == "show_shm"

def test_segments_a_live_process_has_mapped_are_not_leftovers():
    """In use, whatever else is or is not running."""
    w2 = FakeWorld(shm=["/dev/shm/fastrtps_a"], stale_shm=[])
    assert outcome(w2, "leftover_memory").state == V.OK


def test_a_freshly_created_segment_is_not_a_leftover():
    """A process that is STARTING has created segments it has not mapped yet.

    Measured: the sim step spawned the simulation, a daemon restart swept 46
    blocks a second later, and the brand-new simulation came up detached from
    everything -- the repair destroyed the middleware of the thing it was
    about to wait for. Age is the discriminator, and it is asked of the file
    rather than guessed.
    """
    import os
    import tempfile
    import time as _t

    class RealAge(V.World):
        def __init__(self, d):
            V.World.__init__(self)
            self.d = d

        def stale_shm_segments(self):
            # The real implementation, pointed at a temporary directory.
            now = _t.time()
            out = []
            for name in os.listdir(self.d):
                p = os.path.join(self.d, name)
                if now - os.path.getmtime(p) < self.SHM_MIN_AGE_S:
                    continue
                out.append(p)
            return sorted(out)

    with tempfile.TemporaryDirectory() as d:
        fresh = os.path.join(d, "fastrtps_new")
        old = os.path.join(d, "fastrtps_old")
        for p in (fresh, old):
            open(p, "w").close()
        os.utime(old, (0, _t.time() - 3600))
        got = RealAge(d).stale_shm_segments()
        assert got == [old], (
            "a segment created seconds ago is being swept: %r" % got)


def test_the_daemon_is_counted_as_something_that_is_running():
    """FOUND ON A CLEAN RUN, and my own repair caused it.

    The leftover-memory repair restarts the ros2 daemon, and the daemon
    creates shared-memory segments. If the daemon does not count as "running",
    the next check sees those fresh segments with nothing running, offers to
    clear them, restarts the daemon, and goes round -- measured, four times in
    a row, each reporting 14 blocks.

    A repair that recreates the condition it repairs is worse than no repair.
    """
    w = FakeWorld(shm=["/dev/shm/fastrtps_a"] * 14, stale_shm=[])
    o = outcome(w, "leftover_memory")
    assert o.state == V.OK, (
        "the daemon's own segments are being reported as an earlier run's "
        "leftovers, which loops for ever")
    assert "in use" in o.plain


def test_the_daemon_hanging_and_answering_stale_are_both_caught():
    hung = FakeWorld(nodes=([], True))
    assert outcome(hung, "daemon").state == V.FAILED
    assert outcome(hung, "daemon").has_fix
    # Exit status 0 with an empty answer, while things are demonstrably up.
    stale = FakeWorld(nodes=([], False),
                      procs={"move_group": [(1, "x/lib/moveit_ros_move_group/"
                                                "move_group")]})
    assert outcome(stale, "daemon").state == V.FAILED
    # Genuinely idle is not a failure.
    assert outcome(FakeWorld(nodes=([], False)), "daemon").state == V.OK


def test_a_process_that_has_lost_its_middleware_is_not_blamed_on_the_helper():
    """A third state, and it is the NORMAL one after clearing shared memory.

    A stale helper and a process that has lost its connection give the same
    empty answer, and only the first is fixed by restarting the helper.
    Anything that was running when the memory was cleared is in the second
    state -- so restarting the helper at it, over and over, is a loop. The
    tie-breaker is the query that bypasses the helper entirely.
    """
    procs = {"move_group": [(1, "x/lib/moveit_ros_move_group/move_group")]}
    stale = FakeWorld(nodes=([], False), nodes_direct=["/move_group"],
                      procs=procs)
    assert outcome(stale, "daemon").fix == "reset_daemon"

    detached = FakeWorld(nodes=([], False), nodes_direct=[], procs=procs)
    o = outcome(detached, "daemon")
    assert o.state == V.FAILED
    assert o.fix == "kill_detached", o.fix
    assert "lost their connection" in o.plain


def test_a_second_stack_is_caught():
    w = FakeWorld(procs={"move_group": [
        (1, "a/lib/moveit_ros_move_group/move_group"),
        (2, "b/lib/moveit_ros_move_group/move_group")]})
    o = outcome(w, "second_stack")
    assert o.state == V.FAILED and o.has_fix
    assert "twice" in o.plain


def test_a_stray_robot_description_is_caught_separately():
    """Not a second stack and not caught by counting: it wins the description
    topic and the controller manager dies reading it."""
    w = FakeWorld(procs={"robot_state_publisher":
                         [(9, "robot_state_publisher /tmp/master.urdf")]})
    o = outcome(w, "second_stack")
    assert o.state == V.FAILED and o.has_fix
    # A LAUNCHED one carries --params-file and is not a stray.
    w2 = FakeWorld(procs={"robot_state_publisher":
                          [(9, "robot_state_publisher --params-file p.yaml")]})
    assert outcome(w2, "second_stack").state == V.OK


def test_a_running_simulation_that_is_invisible_is_not_blamed_on_the_simulation():
    """FOUND ON THE FIRST REAL RUN, and it is the expensive kind of wrong.

    Clearing stale shared memory invalidates every participant INCLUDING the
    daemon's own cached graph. Measured: after a clear, the topic list
    returned 2 topics against a fully-running stack, and the sequence reported
    "the simulation did not finish starting" -- which is false, specific and
    confident, and sends the operator to the simulation's log instead of to
    the helper.

    The discriminator is the process table, which the daemon step already
    uses. Both cases give the same empty answer; only one of them is the
    simulation's fault.
    """
    w = FakeWorld(topics=[],
                  procs={"move_group": [(1, "x/lib/moveit_ros_move_group/"
                                            "move_group")]})
    o = outcome(w, "sim")
    assert o.state == V.FAILED
    assert o.fix == "reset_daemon", (
        "a running-but-invisible simulation must point at the helper, not at "
        "the simulation's log: %r" % o.fix)
    assert "IS running" in o.plain

    # ... and with NO simulation process AND one that will not start, it
    # really is the simulation, and the fix is its log.
    w2 = FakeWorld(topics=[], procs={}, sim_starts=False)
    o2 = outcome(w2, "sim")
    assert o2.state == V.FAILED
    assert o2.fix == "show_sim_log"


def test_clearing_shared_memory_restarts_the_helper_FIRST():
    """Order, not just presence.

    The daemon holds segments AND a cached graph. Clearing invalidates both,
    so it must be restarted -- but restarting it AFTER the clear orphans its
    old segments and the next check finds fresh leftovers. Measured: clear 33,
    restart, 4 appear immediately. Restart, settle, then clear.
    """
    import inspect
    src = inspect.getsource(V.fix_clear_shm)
    assert "fix_reset_daemon" in src, (
        "clearing shared memory invalidates the daemon's cached graph and "
        "must restart it, or the next step reports a running stack as absent")
    assert src.index("fix_reset_daemon") < src.index("_sweep_unowned"), (
        "the helper is restarted AFTER the clear, which orphans its old "
        "segments and leaves fresh leftovers for the next check")
    assert src.index("_sweep_unowned") < src.index("SHM_SETTLE_S"), (
        "the settle must come after the clear, not before")


def test_restarting_the_helper_cleans_up_after_itself():
    """A repair must not leave the machine in a second broken state.

    Restarting the ros2 daemon orphans the segments the old one held -- four
    or five every time, measured -- so a restart that does not sweep them
    means the very next check offers to clear leftovers the last repair
    created. Two repairs chasing each other is how an operator learns to
    distrust both.
    """
    import inspect
    src = inspect.getsource(V.fix_reset_daemon)
    assert "_sweep_unowned" in src, (
        "the daemon restart does not clear the segments it orphans")
    assert src.index('"start"') < src.index("_sweep_unowned"), (
        "it sweeps between the kill and the start, when the old daemon has "
        "not finished dying -- so its segments are still mapped, nothing is "
        "swept, and they become leftovers a moment later. Measured: five "
        "blocks every time, after a restart that reported success.")


def test_the_sweep_has_one_implementation():
    """Two copies is two things that can be given the wrong pattern."""
    import inspect
    src = inspect.getsource(V)
    assert src.count("def _sweep_unowned") == 1
    for fn in (V.fix_reset_daemon, V.fix_clear_shm):
        assert "os.remove" not in inspect.getsource(fn), (
            "%s removes segments itself instead of using the one sweep"
            % fn.__name__)


def test_clearing_only_touches_segments_nobody_has_mapped():
    """A blanket 'refuse while anything is running' was both too strict -- a
    healthy stack blocks clearing a dead one's leftovers, which is the state
    a partitioned graph is in -- and a proxy for the real property."""
    import inspect
    src = inspect.getsource(V._sweep_unowned)
    assert "stale_shm_segments" in src
    assert "glob.glob" not in src, (
        "the repair is removing segments by pattern again rather than by "
        "whether anybody has them open")


def test_pressing_the_button_again_does_not_start_a_second_of_anything():
    """FOUND BY PRESSING IT FOUR TIMES. Every repair retries the sequence, so
    a step that spawns unconditionally spawns another copy each time -- and by
    the fourth press the mapper was running twice.

    Two mappers is TWO PUBLISHERS ON THE ARM COMMAND TOPIC, which is the one
    thing the VR architecture says must never happen: exactly one input may
    run at a time.
    """
    w = FakeWorld(procs={"quest_bridge_node": [(1, "x/lib/srl_vr_teleop/"
                                                   "quest_bridge_node")],
                         "vr_pose_mapper": [(2, "x/lib/srl_vr_teleop/"
                                                "vr_pose_mapper")]},
                  port_owner=(1, "bridge"))
    before = len(w.started)
    assert outcome(w, "bridge").state == V.OK
    assert outcome(w, "mapper").state == V.OK
    assert len(w.started) == before, (
        "the sequence started %d more process(es) when everything was "
        "already up" % (len(w.started) - before))


def test_a_mapper_that_is_running_but_silent_is_offered_a_restart():
    """Running and not reporting is a third state, and it must not read as
    'already fine' -- that is how a wedged node survives a bring-up."""
    w = FakeWorld(procs={"vr_pose_mapper": [(2, "x/lib/srl_vr_teleop/"
                                                "vr_pose_mapper")]},
                  topics=["/joint_states"])
    o = outcome(w, "mapper")
    assert o.state == V.FAILED
    assert o.fix == "kill_mapper"


def test_the_port_already_bound_is_caught_before_the_bridge_starts():
    """The specific failure: the bridge dies on startup, the other four nodes
    come up fine, and the operator is told to open a URL nobody is serving --
    so it is discovered inside a headset on a stack that looks healthy."""
    w = FakeWorld(port_owner=(4242, "python3 .../quest_bridge_node"))
    o = outcome(w, "port")
    assert o.state == V.FAILED and o.has_fix
    assert V.keys().index("port") < V.keys().index("bridge")


def test_our_own_running_bridge_is_not_reported_as_a_blocked_port():
    """Pressing the button on a system that is already up must say 'already
    done', not report a fault and invite the operator to kill the thing that
    was working."""
    w = FakeWorld(port_owner=(7, "python3 .../quest_bridge_node --ros-args"),
                  procs={"quest_bridge_node":
                         [(7, "x/lib/srl_vr_teleop/quest_bridge_node")]})
    assert outcome(w, "port").state == V.OK
    # A holder with NO bridge process behind it is a stale binding and IS a
    # fault -- that is the case the port check was written for.
    w2 = FakeWorld(port_owner=(7, "python3 .../quest_bridge_node"), procs={})
    assert outcome(w2, "port").state == V.FAILED


def test_a_missing_certificate_is_caught_and_told_apart_from_a_stale_one():
    missing = FakeWorld(cert=(False, False, "no certificate"))
    o = outcome(missing, "certificate")
    assert o.state == V.FAILED and o.has_fix
    assert "no security certificate" in o.plain

    stale = FakeWorld(cert=(True, False, "IP Address:192.168.1.99"))
    o2 = outcome(stale, "certificate")
    assert o2.state == V.FAILED and o2.has_fix
    assert "different address" in o2.plain, (
        "a stale certificate must not read as a missing one -- in the headset "
        "it looks like a broken certificate and is not")


def test_the_bridge_dying_mid_startup_is_caught():
    w = FakeWorld(bridge_dies=True, port_owner=None)
    o = outcome(w, "bridge")
    assert o.state == V.FAILED and o.has_fix
    assert "stopped" in o.plain


def test_a_mapper_holding_state_from_a_previous_run_is_caught():
    """The failure that cost mode 02 every grasping task: a reference and a
    rate limiter survived between runs and the miss ACCUMULATED."""
    for dirty in ({"has_reference": True}, {"has_anchor": True},
                  {"filter_primed": True}, {"scale": 0.4}):
        st = dict(engaged=False, has_reference=False, has_anchor=False,
                  filter_primed=False, scale=0.5, scale_default=0.5)
        st.update(dirty)
        o = outcome(FakeWorld(mapper=st), "mapper_clean")
        assert o.state == V.FAILED, dirty
        assert o.has_fix
    assert outcome(FakeWorld(), "mapper_clean").state == V.OK


def test_the_mapper_reset_fix_says_what_it_does_NOT_clear():
    """`align_yaw_deg` is where the operator is sitting, not per-run state.
    Clearing it silently would send them back to the calibration step with no
    idea why the arm started going the wrong way."""
    st = dict(engaged=False, has_reference=True, has_anchor=False,
              filter_primed=False, scale=0.5, scale_default=0.5)
    o = outcome(FakeWorld(mapper=st), "mapper_clean")
    assert "does NOT clear where you are sitting" in o.fix_note


def test_a_clean_scale_is_the_mappers_own_default_and_not_a_guess():
    """FOUND ON A REAL RUN. The mapper's declared default is 0.5, not 1.0.

    A check that assumed 1.0 reported a freshly reset mapping as "still
    holding settings from an earlier run" -- and offered, as the repair, the
    reset that had just produced the value it objected to. That loop looks
    like a broken node and is a broken check.
    """
    clean = dict(engaged=False, has_reference=False, has_anchor=False,
                 filter_primed=False, scale=0.5, scale_default=0.5)
    assert outcome(FakeWorld(mapper=clean), "mapper_clean").state == V.OK
    moved = dict(clean, scale=1.0)
    assert outcome(FakeWorld(mapper=moved), "mapper_clean").state == V.FAILED
    # And a mapping that will not say what clean means is UNKNOWN territory,
    # not a pass.
    silent = {k: v for k, v in clean.items() if k != "scale_default"}
    assert outcome(FakeWorld(mapper=silent), "mapper_clean").state == V.FAILED


def test_no_observer_is_a_skip_and_not_a_failure():
    """A sim bring-up that will not finish without a second person standing
    next to an unpowered robot is a bring-up people learn to bypass."""
    o = outcome(FakeWorld(beats=0), "observer")
    assert o.state == V.SKIPPED
    assert o.has_fix
    res = [oo for _k, oo in run_all(FakeWorld(beats=0))]
    assert V.verdict(res)[0] == V.OK, "a skipped observer must not block ready"


def test_an_observer_present_is_reported_as_present():
    assert outcome(FakeWorld(beats=8), "observer").state == V.OK


# ============================================================== HYGIENE
def test_ready_requires_poses_TO_ARRIVE_and_not_a_topic_to_exist():
    """THE WORST FIND OF THE SESSION, and it was in the last step.

    `quest_bridge_node` creates its publishers at start-up, so the controller
    pose topic EXISTS the moment the bridge does -- with or without a headset
    in the building. Checking the topic list therefore reported READY TO
    OPERATE on a machine with no headset, and the operator would have found
    out by squeezing the grip and watching nothing happen with the window
    still saying ready.

    docs/ENGINEERING_LOG.md's instrument table calls this exactly: "feature present but does
    nothing -- checked that a field is STORED, not that a consumer READS it."
    """
    # Topic present, nothing arriving: NOT ready, and not a failure either --
    # the machine is fine and the headset is simply not on.
    w = FakeWorld(pose_beats=0)
    o = outcome(w, "ready")
    assert o.state == V.UNKNOWN, o.state
    assert "not sending controller positions" in o.plain
    assert "ENTER VR" in o.plain

    # Poses arriving: ready.
    assert outcome(FakeWorld(pose_beats=25), "ready").state == V.OK

    # And the simulation missing is a different, harder failure.
    o2 = outcome(FakeWorld(topics=["/vr/mapper_right"], pose_beats=25),
                 "ready")
    assert o2.state == V.FAILED


def test_the_headline_names_the_headset_when_that_is_all_that_is_missing():
    """"1 step could not be checked" is true and useless. With nobody in the
    room that IS the normal end of a bring-up, and a vague headline sends the
    operator looking for a fault that is not there."""
    res = [(k, o) for k, o in run_all(FakeWorld(pose_beats=0))]
    state, head = V.verdict([o for _k, o in res], keyed=res)
    assert state == V.UNKNOWN
    assert "connect the headset" in head, head
    # A genuine unknown elsewhere must NOT borrow that headline.
    res2 = [(k, o) for k, o in run_all(FakeWorld(pose_beats=0, beats=None))]
    state2, head2 = V.verdict([o for _k, o in res2], keyed=res2)
    assert "connect the headset" not in head2


def test_a_machine_with_no_headset_does_not_claim_to_be_ready():
    """End to end, the case that matters when nobody is in the lab."""
    res = [o for _k, o in run_all(FakeWorld(pose_beats=0))]
    state, head = V.verdict(res)
    assert state == V.UNKNOWN, (state, head)
    assert "READY" not in head


def test_nothing_the_operator_reads_contains_a_topic_name_or_a_traceback():
    """`plain` and `fix_label` are read under time pressure, possibly by
    somebody who has not read anything else."""
    worlds = [
        FakeWorld(),
        FakeWorld(nodes=([], True)),
        FakeWorld(procs={"move_group": [(1, "a/lib/moveit_ros_move_group/"
                                            "move_group"),
                                        (2, "b/lib/moveit_ros_move_group/"
                                            "move_group")]}),
        FakeWorld(cert=(False, False, "x")),
        FakeWorld(port_owner=(1, "x")),
        FakeWorld(bridge_dies=True),
        FakeWorld(mapper=dict(engaged=True, has_reference=True,
                              has_anchor=True, filter_primed=True, scale=2.0,
                              scale_default=0.5)),
        FakeWorld(beats=0),
        FakeWorld(topics=[]),
        FakeWorld(pose_beats=0),
    ]
    for w in worlds:
        for key, o in run_all(w):
            for field in (o.plain, o.fix_label or ""):
                for b in BANNED:
                    assert b not in field, (
                        "%s: %r contains %r" % (key, field, b))


def test_every_failure_offers_a_fix_or_explains_why_it_cannot():
    for w in (FakeWorld(nodes=([], True)),
              FakeWorld(cert=(False, False, "x")),
              FakeWorld(port_owner=(1, "x")),
              FakeWorld(bridge_dies=True),
              FakeWorld(mapper=dict(engaged=True, has_reference=True,
                                    has_anchor=False, filter_primed=False,
                                    scale=0.5, scale_default=0.5))):
        for key, o in run_all(w):
            if o.state == V.FAILED:
                assert o.has_fix, "%s fails with no fix offered" % key
                assert o.fix in V.FIXES or o.fix.startswith("show_"), o.fix


def test_a_step_that_raises_becomes_unknown_and_not_ok():
    class Boom(FakeWorld):
        def env(self):
            raise RuntimeError("no")
    o = V.plan()[0].run(Boom())
    assert o.state == V.UNKNOWN
    assert "not safe to read that as working" in o.plain


def test_verdict_is_never_ok_on_unknown():
    ok = [V.Outcome(V.OK, "x")] * len(V.STEPS)
    assert V.verdict(ok)[0] == V.OK
    unk = [V.Outcome(V.OK, "x")] * (len(V.STEPS) - 1) + [
        V.Outcome(V.UNKNOWN, "x")]
    assert V.verdict(unk)[0] == V.UNKNOWN
    bad = [V.Outcome(V.FAILED, "It stopped.")] + [V.Outcome(V.OK, "x")]
    assert V.verdict(bad)[0] == V.FAILED


def test_no_step_can_power_a_real_arm():
    """A one-click bring-up that could energise a robot as a side effect is
    the wrong shape whatever it prints. The real arms are a separate,
    deliberate act with their own procedure."""
    import inspect
    src = inspect.getsource(V)
    # NAMED PRECISELY. "kortex" alone matches the workspace directory
    # `~/kortex_ws`, which is in every path in the file -- a check that fires
    # on its own address is not a check.
    for forbidden in ("start_real.sh", "kortex_driver",
                      "kortex_highlevel_bridge", "bridge_enable_",
                      "real_homing_node", "/real/joint_states"):
        assert forbidden not in src, (
            "the bring-up sequence mentions %r -- it must not be able to "
            "reach hardware" % forbidden)


# ===========================================================================
#  THE SIMULATION IS RUNNING AND THE SIMULATION IS NOT REPORTING
#
#  Both were printed in the same run on 2026-08-20: step 5 logged
#  "VR sim: ok -- The simulation is running" and step 12 stopped at
#  "ALMOST READY: THE SIMULATION IS NOT REPORTING", with all fourteen joints
#  showing "--". They came from the same predicate -- `/joint_states` in the
#  topic list -- read twice, and only the second was allowed to sound certain.
#
#  Two separate defects, and each gets its own test.
# ===========================================================================
def test_a_topic_probe_that_fails_is_not_a_verdict_about_the_simulation():
    """`w.topics()` returning None means the question could not be asked.

    The step read `w.topics() or []`, so None became an empty list and the
    next line reported, by name, that the simulation was not reporting. False,
    specific and confident -- and it sends the operator to the simulation log,
    which is the one place the answer is not.
    """
    out = V.step_ready(FakeWorld(topics=None))
    assert out.state == V.UNKNOWN, out
    assert "not reporting" not in out.plain.lower(), out.plain
    assert "could not" in out.plain.lower(), out.plain
    assert out.has_fix


def test_a_simulation_that_is_listed_but_silent_is_caught_at_step_5():
    """Listed is not reporting.

    `/joint_states` appears in the topic list as soon as anything publishes OR
    SUBSCRIBES to it, so the old test passed against a stack whose controllers
    had not come up. That is what put "the simulation is running" beside
    fourteen dashes.
    """
    out = V.step_sim_stack(FakeWorld(joint_beats=0))
    assert out.state == V.FAILED, out
    assert "not saying where the robot is" in out.plain, out.plain
    assert out.fix == "restart_sim"


def test_a_simulation_that_is_reporting_says_so_with_a_count():
    out = V.step_sim_stack(FakeWorld(joint_beats=40))
    assert out.state == V.OK, out
    assert "40" in out.detail, out.detail


def test_step_5_does_not_guess_when_it_cannot_count_arrivals():
    """None is not zero here either, and it must not become a failure."""
    out = V.step_sim_stack(FakeWorld(joint_beats=None))
    assert out.state == V.UNKNOWN, out
    assert out.has_fix


def test_ready_also_refuses_a_listed_but_silent_simulation():
    out = V.step_ready(FakeWorld(joint_beats=0))
    assert out.state == V.FAILED, out
    assert "dash" in out.plain, out.plain


def test_a_simulation_that_is_still_coming_up_is_waited_for_not_condemned():
    """The operator's report, 2026-08-29: press RESTART SIMULATION, and the
    check says the simulation is not up while it is plainly starting.

    `/joint_states` is advertised early in the launch and the first joint
    update is not, so between the two this step declared the simulation
    broken and offered to restart the thing that was starting. If a launch
    is in the process table, wait for it.
    """
    w = FakeWorld(joint_beats=0,
                  procs={"run_teleop": [(7, "bash scripts/run_teleop.sh")]})

    # It comes good while we wait: the beats arrive on the third look.
    seen = {"n": 0}

    def beats(window_s=2.0):
        seen["n"] += 1
        return 0 if seen["n"] < 3 else 30
    w.joint_beats = beats
    out = V.step_sim_stack(w)
    assert out.state == V.OK, out
    assert w.slept > 0, "it returned a verdict without waiting at all"


def test_a_starting_simulation_is_never_given_a_SECOND_stack():
    """HARD CONSTRAINT 3, reached through the repair button.

    `move_group` is late in the launch, so for the first tens of seconds of
    a start there is no `move_group` to find and no `/joint_states` to list
    -- byte for byte what a machine with no simulation looks like. This step
    spawned on that, on top of a launch that was already coming up.
    """
    w = FakeWorld(topics=[],
                  procs={"run_teleop": [(7, "bash scripts/run_teleop.sh")]})
    # The launch that is already up comes good on its own, as it would.
    seen = {"n": 0}

    def topics(timeout_s=10):
        seen["n"] += 1
        return [] if seen["n"] < 3 else ["/joint_states"]
    w.topics = topics
    out = V.step_sim_stack(w)
    assert out.state == V.OK, out
    assert not w.started, (
        "started a second simulation on top of one that was coming up: %r"
        % (w.started,))


def test_the_simulation_is_not_believed_the_instant_it_is_listed():
    """Listed is not reporting, and reporting is not loaded. There is a
    settle window between the topic appearing and the count being taken."""
    w = FakeWorld(topics=[])
    V.step_sim_stack(w)
    assert w.slept >= V.SIM_SETTLE_S, (
        "asked for a count %.0f s after the topic appeared; the controllers "
        "and RViz are still coming up there" % w.slept)


def test_restarting_the_simulation_WAITS_for_it():
    """A repair that returns before it has repaired anything.

    `fix_restart_sim` spawned the launch and returned "press the button
    again when it settles" -- and the window did not wait for the operator,
    it re-ran the whole sequence 400 ms later. So the repair reported
    success and the sequence reported the simulation absent, both on screen
    at once, with RViz still grey.
    """
    w = FakeWorld(topics=[])
    ok, msg = V.fix_restart_sim(w)
    assert ok, msg
    assert w.slept >= V.SIM_SETTLE_S, (
        "returned %.0f s after starting a launch that takes a minute" % w.slept)
    assert "waited" in msg.lower(), msg


def test_a_restart_that_never_comes_up_reports_FAILURE():
    """The control. Without it, a repair that always returns True after a
    long sleep passes the test above."""
    w = FakeWorld(topics=[], sim_starts=False)
    ok, msg = V.fix_restart_sim(w)
    assert not ok, msg
    assert "still not saying where the robot is" in msg, msg


def test_rviz_is_waited_for_and_is_never_a_gate():
    """A missing RViz window is reported, not fatal: `use_rviz:=false` and a
    headless box are both legitimate."""
    w = FakeWorld(topics=[], rviz_windows=0)
    out = V.step_sim_stack(w)
    assert out.state == V.OK, out
    assert "no RViz window" in out.detail, out.detail
    w2 = FakeWorld(topics=[], rviz_windows=1)
    assert "RViz drawn" in V.step_sim_stack(w2).detail


def test_the_step_and_the_repair_start_THE_SAME_simulation():
    """Two spawn sites, one command. They drifted apart once already."""
    import inspect
    for fn in (V.step_sim_stack, V.fix_restart_sim):
        assert "_sim_argv" in inspect.getsource(fn), fn


def test_the_window_can_stop_the_launch_opening_a_second_rviz():
    """The 2026-08-27 "two RVizs" finding, on the path it missed."""
    import os
    w = FakeWorld()
    old = os.environ.get("SRL_VR_SIM_RVIZ")
    try:
        os.environ["SRL_VR_SIM_RVIZ"] = "false"
        assert "use_rviz:=false" in V._sim_argv(w)
        os.environ.pop("SRL_VR_SIM_RVIZ")
        assert "use_rviz:=false" not in V._sim_argv(w), (
            "a terminal run has no window to embed in and must keep its own")
    finally:
        if old is None:
            os.environ.pop("SRL_VR_SIM_RVIZ", None)
        else:
            os.environ["SRL_VR_SIM_RVIZ"] = old


def test_restarting_the_simulation_is_offered_as_its_own_repair():
    """Not routed to `reset_daemon`.

    The picture is not stale in this state -- the robot really is silent -- so
    restarting the helper that lists what is running repairs nothing and
    reports success, which is the silent-acceptance failure this file exists
    to prevent.
    """
    assert "restart_sim" in V.FIXES


# ===========================================================================
#  WORKING ALONE
# ===========================================================================
def test_no_bypass_is_the_default_everywhere():
    """A FakeWorld that says nothing about the bypass must not have one."""
    out = V.step_observer(FakeWorld(beats=0))
    assert out.state == V.SKIPPED, out
    assert "bypass" not in out.plain.lower()


def test_a_taken_bypass_reports_BYPASSED_and_names_who_and_when():
    out = V.step_observer(FakeWorld(beats=0, bypass=(
        True, {"who": "gui", "granted_at_iso": "2026-08-21T09:00:00+0100"})))
    assert out.state == V.BYPASSED, out
    assert "WORKING ALONE" in out.plain
    assert "gui" in out.detail and "09:00:00" in out.detail
    assert out.fix == "cancel_bypass", "it must be cancellable from the row"


def test_the_bypass_does_not_hide_an_observer_who_IS_posted():
    """The check runs first, always.

    If somebody is actually posted, that is what the row says -- otherwise
    ticking the box would mask a working heartbeat and the operator would
    never learn the observer path was fine.
    """
    out = V.step_observer(FakeWorld(beats=3, bypass=(True, {"who": "gui"})))
    assert out.state == V.OK, out
    assert "present" in out.plain.lower()


def test_working_alone_is_said_in_the_headline_and_not_folded_away():
    """The one line the operator reads must not make an unobserved run look
    like every other run."""
    res = [o for _k, o in run_all(FakeWorld(
        beats=0, bypass=(True, {"who": "gui", "granted_at_iso": "x"})))]
    state, head = V.verdict(res)
    assert "WORKING ALONE" in head or "no observer" in head, head


def test_a_bypassed_observer_does_not_spoil_the_run():
    keyed = run_all(FakeWorld(
        beats=0, bypass=(True, {"who": "gui", "granted_at_iso": "x"})))
    res = [o for _k, o in keyed]
    state, _head = V.verdict(res, keyed=keyed)
    assert state in (V.OK, V.UNKNOWN), state
    assert not [r for r in res if r.state == V.FAILED]


def test_the_observer_check_itself_is_unchanged():
    """The bypass ADDS a state. It must not have altered the other three."""
    assert V.step_observer(FakeWorld(beats=3)).state == V.OK
    assert V.step_observer(FakeWorld(beats=0)).state == V.SKIPPED
    assert V.step_observer(FakeWorld(beats=None)).state == V.UNKNOWN
