"""Every connection check must be able to FAIL, and must never be green on
unknown.

A CHECK THAT CANNOT FAIL ON A DELIBERATELY BROKEN INPUT IS NOT A CHECK -- the
standing rule in CLAUDE.md, and the reason this file exists rather than a
"the panel renders" smoke test. The seven checks in `real_arm_doctor` are the
seven ways connecting the real arms has actually gone wrong on this rig, and
each one is here driven three ways:

    a healthy world      -> ok
    a broken world       -> bad, WITH a fix offered
    a world it cannot see -> unknown, and NEVER ok

The third case is the one that matters most and is the easiest to lose. The
GUI once reported "IK BLOCKED -- 0% success" computed over ZERO attempts,
which sent the operator to the solver instead of to the missing input. A
connection panel that reads "ready" because it could not look is the same
defect with the arms switched on.

Everything the checks touch goes through `Probe`, so the broken world is a
subclass rather than a mutated machine: no test here creates a stale segment,
kills a daemon or unplugs a board.
"""
import math

from srl_teleop import real_arm_doctor as D


HOME = [0.0] * 7


class FakeProbe(D.Probe):
    """A world described by keyword arguments. Nothing here reads the machine."""

    def __init__(self, env=None, stack=(), envs=None, shm=None,
                 daemon=([], False), procs=None, ttys=(), port="",
                 sim=None, real=None, home=HOME, kortex=(), ktail=None,
                 live=None, orphans=()):
        self._env = dict(env if env is not None else D.EXPECTED)
        self._stack = list(stack)
        self._envs = envs or {}
        self._shm = shm
        self._daemon = daemon
        self._procs = procs or {}
        self._ttys = ttys
        self._port = port
        self._sim, self._real = sim, real
        self._home = home
        self._kortex = list(kortex)
        self._ktail = ktail
        self._live = list(stack) if live is None else list(live)
        self._orphans = list(orphans)

    def my_env(self):
        return dict(self._env)

    def stack_pids(self):
        return list(self._stack)

    def live_pids(self):
        return list(self._live)

    def orphan_nodes(self):
        return list(self._orphans)

    def env_of(self, pid):
        return self._envs.get(pid)

    def shm_segments(self):
        return self._shm

    def daemon_nodes(self, timeout_s=8):
        return self._daemon

    def find(self, rx):
        for key, val in self._procs.items():
            if key in rx:
                return list(val)
        return []

    def tty_candidates(self):
        return self._ttys

    def master_port_param(self):
        return self._port

    def joint_states(self):
        return self._sim, self._real

    def home_radians(self, arm):
        return list(self._home) if self._home is not None else None

    def kortex_procs(self):
        return list(self._kortex)

    def kortex_log_tail(self):
        return self._ktail


def _one(fn, probe, fixes=None):
    return fn(probe, fixes or {})


# --------------------------------------------------------------------- 1
def test_discovery_ok_bad_unknown():
    good = FakeProbe(env=D.EXPECTED, stack=[], shm=[])
    assert _one(D.check_discovery, good).state == D.OK

    # This window never sourced the environment file.
    bare = dict(D.EXPECTED)
    bare["FASTDDS_BUILTIN_TRANSPORTS"] = ""
    c = _one(D.check_discovery, FakeProbe(env=bare, stack=[]),
             {"reexec": lambda: None})
    assert c.state == D.BAD and c.has_fix

    # A stack is running and disagrees with this window.
    theirs = dict(D.EXPECTED, ROS_DOMAIN_ID="7")
    c = _one(D.check_discovery,
             FakeProbe(env=D.EXPECTED, stack=[101], envs={101: theirs}),
             {"reexec": lambda: None})
    assert c.state == D.BAD and "ROS_DOMAIN_ID" in c.detail

    # A stack is running and its environment cannot be read: UNKNOWN, not OK.
    c = _one(D.check_discovery, FakeProbe(env=D.EXPECTED, stack=[101],
                                          envs={}))
    assert c.state == D.UNKNOWN


# --------------------------------------------------------------------- 2
def test_shm_only_bad_when_nothing_is_running():
    segs = ["/dev/shm/fastrtps_port7002", "/dev/shm/sem.fastrtps_port7002"]
    c = _one(D.check_shm, FakeProbe(shm=segs, stack=[]),
             {"clear_shm": lambda: None})
    assert c.state == D.BAD and c.has_fix

    # The same segments with a stack up are IN USE, not stale. Offering to
    # clear them there would break a healthy run.
    c = _one(D.check_shm, FakeProbe(shm=segs, stack=[101]))
    assert c.state == D.OK and not c.has_fix

    assert _one(D.check_shm, FakeProbe(shm=[], stack=[])).state == D.OK
    assert _one(D.check_shm, FakeProbe(shm=None)).state == D.UNKNOWN


# --------------------------------------------------------------------- 3
def test_daemon_hang_and_stale_answer_are_both_caught():
    # It HUNG. This is the original failure: no error, just no answer.
    c = _one(D.check_daemon, FakeProbe(daemon=([], True)),
             {"reset_daemon": lambda: None})
    assert c.state == D.BAD and c.has_fix

    # It answered "nothing" while things are demonstrably running. Exit
    # status 0 and an empty string is indistinguishable from an idle graph
    # unless the process table is consulted, which is why it is.
    c = _one(D.check_daemon, FakeProbe(daemon=([], False), stack=[1, 2]),
             {"reset_daemon": lambda: None})
    assert c.state == D.BAD

    # Genuinely idle.
    assert _one(D.check_daemon,
                FakeProbe(daemon=([], False), stack=[])).state == D.OK
    assert _one(D.check_daemon,
                FakeProbe(daemon=(["/move_group"], False))).state == D.OK
    assert _one(D.check_daemon, FakeProbe(daemon=(None, False))).state == D.UNKNOWN


# --------------------------------------------------------------------- 4
def test_second_stack_and_stray_publisher():
    two = {"master_pose_node": [(11, "x/lib/srl_teleop/master_pose_node"),
                                (12, "x/lib/srl_teleop/master_pose_node")]}
    c = _one(D.check_second_stack, FakeProbe(procs=two),
             {"kill_second_stack": lambda: None})
    assert c.state == D.BAD and c.has_fix

    stray = {"robot_state_publisher":
             [(21, "robot_state_publisher /tmp/master.urdf")]}
    c = _one(D.check_second_stack, FakeProbe(procs=stray),
             {"kill_stray_rsp": lambda: None})
    assert c.state == D.BAD and "left over" in c.title.lower()

    # A launched publisher carries --params-file and is NOT a stray. Without
    # this distinction the check would fire on every healthy stack.
    launched = {"robot_state_publisher":
                [(22, "robot_state_publisher --params-file /tmp/p.yaml")]}
    assert _one(D.check_second_stack,
                FakeProbe(procs=launched)).state == D.OK
    assert _one(D.check_second_stack, FakeProbe(procs={})).state == D.OK


# -------------------------------------------------------------------- 4b
def test_orphan_nodes_are_caught_and_a_live_run_is_not():
    """The failure that was found by being bitten: seven nodes outlived the
    launch that started them and kept publishing, and neither the duplicate
    check nor the stray-publisher check could see them."""
    orph = [(111460, "/ws/install/srl_teleop/lib/srl_teleop/ik_follower_node"),
            (111140, "/opt/ros/jazzy/lib/rviz2/rviz2 -d cfg")]
    c = _one(D.check_orphans, FakeProbe(orphans=orph),
             {"kill_orphans": lambda: None})
    assert c.state == D.BAD and c.has_fix
    assert "ik_follower_node" in c.plain or "rviz2" in c.plain

    assert _one(D.check_orphans, FakeProbe(orphans=[])).state == D.OK


def test_shm_counts_the_real_arm_stack_as_running():
    """The mock and real arm stacks are not a SIM stack, and clearing the
    shared memory while they are up would pull it out from under them.

    Caught by inspection, not by a crash: the leftover row asked
    `stack_pids()`, which matches only master_pose_node / move_group /
    ros2_control_node, so with the arms up and the sim down the panel offered
    to clear memory that the arms were using."""
    segs = ["/dev/shm/fastrtps_x"]
    # No SIM stack, but the real arm stack is up.
    c = _one(D.check_shm, FakeProbe(shm=segs, stack=[], live=[9001]))
    assert c.state == D.OK and not c.has_fix
    # Genuinely nothing running.
    c = _one(D.check_shm, FakeProbe(shm=segs, stack=[], live=[]),
             {"clear_shm": lambda: None})
    assert c.state == D.BAD


# --------------------------------------------------------------------- 5
def test_teensy_absent_and_moved():
    c = _one(D.check_teensy, FakeProbe(ttys=[]), {"teensy_help": lambda: None})
    assert c.state == D.BAD and c.has_fix

    # Plugged in, but the running reader was pointed at the other socket --
    # the failure that looks exactly like a dead board.
    c = _one(D.check_teensy,
             FakeProbe(ttys=["/dev/ttyACM1"], port="/dev/ttyACM0"),
             {"teensy_repoint": lambda: None})
    assert c.state == D.BAD and c.has_fix

    assert _one(D.check_teensy,
                FakeProbe(ttys=["/dev/ttyACM0"], port="/dev/ttyACM0")).state == D.OK
    assert _one(D.check_teensy, FakeProbe(ttys=None)).state == D.UNKNOWN


# --------------------------------------------------------------------- 6
def test_home_gap_is_measured_and_absence_is_unknown():
    at_home = {"left_joint_%d" % (i + 1): 0.0 for i in range(7)}
    at_home.update({"right_joint_%d" % (i + 1): 0.0 for i in range(7)})
    assert _one(D.check_home_gap, FakeProbe(real=at_home)).state == D.OK

    # The known case: ~1.9 rad on the last joint, which is the 2026-08-15
    # home change and not a mis-homed arm.
    off = dict(at_home, left_joint_7=1.94)
    c = _one(D.check_home_gap, FakeProbe(real=off), {"home_arms": lambda: None})
    assert c.state == D.BAD and c.has_fix
    assert "111" in c.detail or "%.0f" % math.degrees(1.94) in c.plain

    # No real arm at all is UNKNOWN. It is emphatically not "at home".
    assert _one(D.check_home_gap, FakeProbe(real=None)).state == D.UNKNOWN
    # Reporting, but nothing lines up: still unknown.
    assert _one(D.check_home_gap,
                FakeProbe(real={"elbow": 0.0})).state == D.UNKNOWN


def test_home_gap_wraps_the_continuous_joints():
    """A continuous joint 2*pi away is AT home, not 360 degrees from it."""
    js = {"left_joint_%d" % (i + 1): 0.0 for i in range(7)}
    js.update({"right_joint_%d" % (i + 1): 0.0 for i in range(7)})
    js["left_joint_1"] = 2 * math.pi
    assert _one(D.check_home_gap, FakeProbe(real=js)).state == D.OK
    # ... while a NON-continuous joint 2*pi away is not, because it cannot
    # get there by wrapping.
    js2 = dict(js)
    js2["left_joint_1"] = 0.0
    js2["left_joint_2"] = 2 * math.pi
    assert _one(D.check_home_gap, FakeProbe(real=js2),
                {"home_arms": lambda: None}).state == D.BAD


# --------------------------------------------------------------------- 7
def test_kortex_session_leak():
    c = _one(D.check_kortex_session, FakeProbe(ktail="something\nexited 1\n"),
             {"release_kortex": lambda: None})
    assert c.state == D.BAD and c.has_fix

    assert _one(D.check_kortex_session,
                FakeProbe(ktail="kortex session closed cleanly\n")).state == D.OK
    assert _one(D.check_kortex_session,
                FakeProbe(kortex=[(9, "kortex_highlevel_bridge")])).state == D.OK
    # No record at all is UNKNOWN -- there is no evidence either way.
    assert _one(D.check_kortex_session, FakeProbe(ktail=None)).state == D.UNKNOWN


# ------------------------------------------------------------- the panel
def test_verdict_is_never_ok_on_unknown():
    ok = [D.Check("a", "t", D.OK, "p")]
    unk = ok + [D.Check("b", "t", D.UNKNOWN, "p")]
    bad = unk + [D.Check("c", "t", D.BAD, "p")]
    assert D.verdict(ok)[0] == D.OK
    assert D.verdict(unk)[0] == D.UNKNOWN
    assert D.verdict(bad)[0] == D.BAD


def test_run_all_survives_a_check_that_raises():
    class Boom(D.Probe):
        def my_env(self):
            raise RuntimeError("no")

    out = D.run_all(Boom())
    assert len(out) == len(D.CHECKS)
    assert any(c.state == D.UNKNOWN for c in out)


def test_operator_text_carries_no_topic_names_or_tracebacks():
    """`title` and `plain` are what a person reads under time pressure.

    The brief is explicit: never a topic name, never a raw error. Those go in
    `detail`, which is for the log and for a bug report.
    """
    banned = ("/joint_states", "rclpy", "Traceback", "fastrtps", "ttyACM",
              "ros2 ", "/dev/shm", "None", "Exception")
    for c in D.run_all(FakeProbe(shm=[], daemon=([], False))):
        for field in (c.title, c.plain):
            for b in banned:
                assert b not in field, "%s: %r contains %r" % (c.key, field, b)


def test_every_bad_state_offers_a_fix():
    """A named problem with no button is a problem the operator has to
    remember how to fix, which is what this panel exists to stop."""
    fixes = {k: (lambda: None) for k in
             ("reexec", "clear_shm", "reset_daemon", "kill_second_stack",
              "kill_stray_rsp", "kill_orphans", "teensy_help",
              "teensy_repoint", "home_arms", "release_kortex")}
    worlds = [
        FakeProbe(env=dict(D.EXPECTED, FASTDDS_BUILTIN_TRANSPORTS="")),
        FakeProbe(shm=["/dev/shm/fastrtps_x"], stack=[]),
        FakeProbe(daemon=([], True)),
        FakeProbe(procs={"master_pose_node": [(1, "a/lib/srl_teleop/master_pose_node"),
                                              (2, "b/lib/srl_teleop/master_pose_node")]}),
        FakeProbe(ttys=[]),
        FakeProbe(real={"left_joint_7": 1.94}),
        FakeProbe(ktail="died\n"),
        FakeProbe(orphans=[(1, "/opt/ros/jazzy/lib/rviz2/rviz2")]),
    ]
    for probe in worlds:
        for c in D.run_all(probe, fixes):
            if c.state == D.BAD:
                assert c.has_fix, "%s is BAD with no fix" % c.key
                assert c.fix_label
