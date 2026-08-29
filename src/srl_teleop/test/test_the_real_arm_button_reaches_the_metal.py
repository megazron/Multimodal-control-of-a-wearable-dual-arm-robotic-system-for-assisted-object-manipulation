"""START REAL ARM TELEOP has to open the path, not set a flag nobody reads.

THE DEFECT THIS PINS, measured on the rig 2026-08-29. The operator squeezed
the grip, the simulation followed, and the real arms stood still.

`on_real_teleop` -- step 4 of the REAL ARM SEQUENCE panel, and the only
control in the window that is supposed to let VR move real metal -- called
`/vr/enable_real_arm` and nothing else. That service sets `allow_real_arm` on
`vr_safety_node`, which republishes it inside the `/vr/safety` status string.
Grepping the tree for that parameter finds the node that declares it and the
launch files that pass it, and NO CONSUMER: nothing gates on it, nothing
starts because of it. The service returned success, the panel said "Asked for
real-arm control", and the arms could not have moved.

What actually carries the simulation to the metal is `sim_to_real_bridge`:
one per arm, republishing the SIM's joint states onto
/real/<arm>_arm_controller/joint_trajectory, which is what
`kortex_highlevel_bridge` consumes. On the rig that day it was not running at
all. Nothing in the window said so -- the arm-status line reported network UP
and session up, which was true and irrelevant.

The second defect is in step 1. `on_real_start` shelled out to
`start_real.sh` raw, which opens one Kortex session per arm. The arm permits
exactly ONE (HARD CONSTRAINT 2), so against already-connected arms the launch
fails and its own cleanup sweeps the WORKING bridges as stragglers: the button
labelled START REAL ARMS disconnects the arms. `start_mode` has substituted
the cascade for this reason since 2026-08-26 and this button went round it.

These tests exercise the real methods over a FAKED process table. Nothing here
starts a stack, opens a session or touches hardware. The last test is the
standing rule's own requirement: a check that cannot fail on a deliberately
broken input is not a check.
"""
import os
import subprocess
import sys
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))


def _slice_methods():
    """The five real-arm methods, sliced out of `scripts/srl_gui.py`.

    Sliced rather than imported: that module needs PyQt and a display, and
    this has to run in CI. The slice is delimited by the methods' OWN names,
    so a rename breaks this loudly instead of silently testing nothing.
    """
    src = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
    start = src.index("    @staticmethod\n    def _arms_running")
    end = src.index("    def on_real_estop(self):")
    assert end > start, "the real-arm slots moved -- this test slices nothing"
    return src[start:end]


class _FakeRun:
    """`subprocess.run` over a declared process table.

    Returns 0 for a pgrep whose pattern names a listed process, 1 otherwise --
    which is exactly what pgrep does, and is the only thing these methods ask
    of the process table.
    """

    #: The real module's own sentinels, because the code under test passes
    #: `subprocess.DEVNULL` and reads it off this same object.
    DEVNULL = subprocess.DEVNULL

    def __init__(self, procs):
        self.procs = list(procs)

    def run(self, argv, **kw):
        # NOT `__call__`. The code under test calls `subprocess.run(...)`, and
        # a fake that only defines `__call__` raises AttributeError inside the
        # helper's own `except Exception: pass` -- so every arm reads as
        # absent and every test passes for the wrong reason.
        assert argv[0] == "pgrep", argv
        pat = argv[-1]
        hit = any(pat in p for p in self.procs)
        return subprocess.CompletedProcess(argv, 0 if hit else 1)


class _Timer:
    """QTimer.singleShot, captured instead of fired."""

    def __init__(self):
        self.pending = []

    def singleShot(self, ms, fn):
        self.pending.append((ms, fn))

    def fire(self):
        """Run ONE round: the callbacks queued now, not the ones they queue.

        Draining re-armed callbacks too would collapse a real 500 ms retry
        loop into an instant one and exhaust every retry before the service
        it is waiting on could possibly have answered -- which is a test
        harness inventing a timeout the window does not have.
        """
        batch, self.pending = self.pending, []
        for _, fn in batch:
            fn()


class _Recorder:
    """Everything the slots talk to, recorded rather than done.

    These OVERRIDE the sliced methods -- `_Recorder` comes first in the MRO --
    so the code under test is the real code and its collaborators are not.
    """

    # --- the collaborators the slots use, all recorded ------------------
    def _run_raw(self, label, argv):
        self.launched.append((label, argv))

    def _svc(self, name, typ="std_srvs/srv/Trigger", then=None):
        self.services.append(name)
        # The relay's answer, as the real `call_trigger` hands it back. The
        # panel is only allowed to say ARMED when this said yes.
        if then is not None:
            self.replies.append((name, then))

    def _real_say(self, text, bad=False):
        self.said.append((text, bad))

    def _real_guard(self, label, fn):
        fn()

    class _Head:
        def __init__(self, outer):
            self.outer = outer

        def setText(self, t):
            self.outer.head = t

    @property
    def real_head(self):
        return _Recorder._Head(self)

    # --- convenience ----------------------------------------------------
    def scripts_launched(self):
        return [os.path.basename(a[1][1]) for a in self.launched]

    def last_said(self):
        return self.said[-1] if self.said else ("", False)


def _Panel(procs):
    """One panel over one declared process table.

    The sliced methods are compiled into a namespace of their own, so
    `subprocess` and `QTimer` inside them are THIS panel's fakes and swapping
    `_ns["subprocess"]` mid-test is how a relay is made to come up (or not).
    """
    timer = _Timer()
    ns = {"subprocess": _FakeRun(procs), "os": os, "QTimer": timer, "_WS": WS}
    exec("class _M:\n" + _slice_methods(), ns)   # noqa: S102 -- code under test
    panel = type("_Bound", (_Recorder, ns["_M"]), {})()
    panel.launched = []             # (label, argv)
    panel.services = []             # service names
    panel.said = []                 # (text, bad)
    panel.head = ""
    panel.replies = []              # (service, callback) awaiting an answer
    panel.timer = timer
    panel._ns = ns
    return panel


def _answer(panel, ok, msg):
    """Deliver the relay's reply to every pending callback, then let the
    verdict timer run."""
    for _, cb in panel.replies:
        cb(ok, msg)
    panel.replies = []
    panel.timer.fire()


BRIDGE_L = "kortex_highlevel_bridge_left"
BRIDGE_R = "kortex_highlevel_bridge_right"
RELAY_L = "sim_to_real_bridge_left"


class TestStartRealArmsCannotDisconnectThem(unittest.TestCase):

    def test_already_connected_starts_the_relay_not_a_second_session(self):
        p = _Panel([BRIDGE_L, BRIDGE_R])
        p.on_real_start()
        self.assertEqual(p.scripts_launched(), ["start_cascade.sh"],
                         "pressed against open Kortex sessions, START REAL "
                         "ARMS must relay onto them -- start_real.sh opens a "
                         "second session, is refused, and sweeps the working "
                         "bridges on its way out")
        self.assertEqual(p.launched[0][1][2:], ["left", "right"],
                         "the cascade must be told which arms are up")

    def test_nothing_connected_still_connects(self):
        p = _Panel([])
        p.on_real_start()
        self.assertEqual(p.scripts_launched(), ["start_real.sh"])
        self.assertIn("arm:=both", p.launched[0][1])


class TestRealArmTeleopOpensThePath(unittest.TestCase):

    def test_it_enables_the_relay_and_not_just_the_gate(self):
        p = _Panel([BRIDGE_L, RELAY_L])
        p.on_real_teleop()
        self.assertIn("/vr/enable_real_arm", p.services,
                      "the safety gate is still the first thing asked")
        self.assertIn("/bridge_enable_left", p.services,
                      "THE DEFECT: the gate alone sets a parameter no "
                      "consumer reads. The relay is what carries the "
                      "simulation to the metal and it has to be ENABLED")

    def test_a_missing_relay_is_started_then_enabled(self):
        p = _Panel([BRIDGE_L])                  # session open, NO relay
        p.on_real_teleop()
        self.assertEqual(p.scripts_launched(), ["start_cascade.sh"])
        self.assertEqual(p.launched[0][1][2:], ["left"])
        self.assertNotIn("/bridge_enable_left", p.services,
                         "nothing to enable until the relay exists")
        p._ns["subprocess"] = _FakeRun([BRIDGE_L, RELAY_L])   # it came up
        p.timer.fire()
        self.assertIn("/bridge_enable_left", p.services)
        self.assertNotIn("ARMED", p.head,
                         "asking is not arming -- the relay has not "
                         "answered yet")
        _answer(p, True, "bridge enabled")
        self.assertIn("ARMED", p.head)

    def test_no_session_refuses_by_name_and_opens_no_gate(self):
        p = _Panel([])
        p.on_real_teleop()
        self.assertEqual(p.services, [],
                         "with no arm to drive, opening the real-arm gate "
                         "changes nothing and hides the real reason")
        text, bad = p.last_said()
        self.assertTrue(bad)
        self.assertIn("REFUSED", text)
        self.assertIn("START REAL ARMS", text,
                      "a refusal has to name the step that fixes it")


class TestTheCheckCanFail(unittest.TestCase):
    """A relay that never comes up must be REPORTED, not assumed."""

    def test_a_relay_that_does_not_start_is_named(self):
        p = _Panel([BRIDGE_L])
        p.on_real_teleop()
        p.timer.fire()                       # relay still absent
        self.assertNotIn("/bridge_enable_left", p.services)
        text, bad = p.last_said()
        self.assertTrue(bad, "a silent arm must not read as success")
        self.assertIn("sim_to_real_bridge_left", text)
        self.assertIn("RELAY MISSING", p.head)


if __name__ == "__main__":
    unittest.main()


class TestThePanelReportsTheRelaysAnswer(unittest.TestCase):
    """Asking is not arming.

    MEASURED 2026-08-29, after the relay itself was fixed. The relay was
    running and `cascade_active_left` read FALSE: `sim_to_real_bridge`
    refuses to enable while the real arm is further from the loaded home
    than `enable_gap_rad`, because it replays sim angles STARTING at home
    and enabling would command the difference as a jump. The measured gap
    was 2.729 rad. The panel said the arms were armed.

    That is the same defect as the enable gate it replaced -- reporting the
    request instead of the result.
    """

    def test_a_refusal_is_never_reported_as_armed(self):
        p = _Panel([BRIDGE_L, RELAY_L])
        p.on_real_teleop()
        _answer(p, False, "REFUSED: real left arm is 2.729 rad from the "
                          "loaded home on joint_2")
        self.assertNotIn("ARMED", p.head)
        self.assertIn("REFUSED", p.head)
        text, bad = p.last_said()
        self.assertTrue(bad)
        self.assertIn("2.729 rad", text,
                      "the relay names the joint and the distance and it is "
                      "the only place that exists -- surface it verbatim")

    def test_a_home_refusal_names_the_step_that_fixes_it(self):
        p = _Panel([BRIDGE_L, RELAY_L])
        p.on_real_teleop()
        _answer(p, False, "REFUSED: real left arm is 2.729 rad from the "
                          "loaded home on joint_2")
        text, _ = p.last_said()
        self.assertIn("HOME BOTH ARMS", text)

    def test_a_success_is_reported_as_armed(self):
        p = _Panel([BRIDGE_L, RELAY_L])
        p.on_real_teleop()
        _answer(p, True, "bridge enabled")
        self.assertIn("ARMED", p.head)
        text, bad = p.last_said()
        self.assertFalse(bad)


HOMING_L = "real_homing_node_left"


class TestTheHomeButtonIsNotDeadOnAnAdoptedSession(unittest.TestCase):
    """`/home_arm_<arm>` is served by `real_homing_node`, which only the
    real-arm LAUNCH started. Adopt an already-open Kortex session -- what
    every rig brought up outside this window is -- and the service did not
    exist, so HOME BOTH ARMS answered "service not present, nothing was
    sent". That is one half of the 2026-08-29 deadlock: the relay refuses
    until the arm is homed, homing was unreachable, the arm could never
    move."""

    def test_it_starts_the_homing_node_when_none_is_served(self):
        p = _Panel([BRIDGE_L])                  # session open, NO homing node
        p.on_real_home()
        self.assertEqual(p.scripts_launched(), ["start_cascade.sh"])
        self.assertEqual(p.launched[0][1][2:], ["left"])
        self.assertNotIn("/home_arm_left", p.services,
                         "asking a service nobody serves is the defect")

    def test_it_asks_once_the_service_is_served(self):
        p = _Panel([BRIDGE_L])
        p.on_real_home()
        p._ns["subprocess"] = _FakeRun([BRIDGE_L, HOMING_L])
        p.timer.fire()
        self.assertIn("/home_arm_left", p.services)
        self.assertIn("HOMING", p.head)

    def test_it_only_homes_arms_that_have_a_session(self):
        p = _Panel([BRIDGE_L, HOMING_L])
        p.on_real_home()
        self.assertIn("/home_arm_left", p.services)
        self.assertNotIn("/home_arm_right", p.services,
                         "the right arm has no Kortex session -- asking is "
                         "noise that hides the real state")

    def test_no_session_refuses_by_name(self):
        p = _Panel([])
        p.on_real_home()
        self.assertEqual(p.services, [])
        text, bad = p.last_said()
        self.assertTrue(bad)
        self.assertIn("START REAL ARMS", text)

    def test_a_homing_node_that_never_starts_is_named(self):
        p = _Panel([BRIDGE_L])
        p.on_real_home()
        p.timer.fire()                       # it did not come up
        self.assertNotIn("/home_arm_left", p.services)
        text, bad = p.last_said()
        self.assertTrue(bad, "a press that goes nowhere must not read as ok")
        self.assertIn("real_homing_node_left", text)
