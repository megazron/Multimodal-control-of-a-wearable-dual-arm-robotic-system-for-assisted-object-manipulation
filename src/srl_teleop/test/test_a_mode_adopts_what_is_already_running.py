"""A mode is an END STATE, not a list of processes to spawn.

THE DEFECT THIS PINS, measured 2026-08-26 from the operator's side. The OPERATE
rows launched every step of their chain unconditionally, and on a rig that was
already part-way up that produced two failures which look nothing alike and are
the same mistake:

  "2 VR / DESK TELEOP -> SIM ONLY does nothing, no RViz."
      Step 1 is the simulation stack, which carries `starts_stack=True`. With
      a stack already running the preflight refuses it under HARD CONSTRAINT 3
      -- correctly. But the sequencer treated the refusal as the end of the
      chain, so step 2, the VR transport, never ran. The refusal itself went to
      a status line. A button that refuses invisibly and a button that is not
      wired to anything are indistinguishable from a chair.

  "SIM + REAL kills the arms."
      Step 3 is `start_real.sh`, which brings the real side up from nothing --
      one `kortex_highlevel_bridge` per arm, one Kortex session each. The arm
      permits exactly ONE (HARD CONSTRAINT 2), so against already-connected
      arms the second session is refused, the launch fails, and its cleanup
      sweeps the WORKING bridges as orphaned stack processes. The button
      labelled "cascade to the real arms" DISCONNECTS them.

The fix is one question asked before each step -- "is this up?" instead of
"have I launched it?" -- and these tests hold it. They exercise the real
methods against a faked process table; nothing here starts a stack.

The third test is the standing rule's own requirement: a check that cannot
fail on a deliberately broken input is not a check. `test_the_probe_can_say_no`
makes the process table empty and requires the adoption to STOP happening.
"""
import os
import re
import sys
import types
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))


def _gui_helpers(pgrep_counts):
    """The sequencer's three probe methods, over a FAKED process table.

    Sliced out of `scripts/srl_gui.py` rather than imported, because
    importing that module needs PyQt and a display and this must run in CI.
    The slice is delimited by the methods' own names, so a rename breaks the
    test loudly instead of silently testing nothing -- which is the failure
    mode `test_home_has_one_source` was written against.
    """
    src = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
    start = src.index("    STEP_PROC = {")
    end = src.index("    def _ensure_rviz")
    assert end > start, "the helpers moved -- this test is slicing nothing"

    class _Run:
        def __init__(self, out):
            self.stdout = out

    def fake_run(argv, **kw):
        # `pgrep -fc PATTERN`
        pat = argv[-1]
        return _Run("%d\n" % pgrep_counts.get(pat, 0))

    ns = {"subprocess": types.SimpleNamespace(run=fake_run), "os": os,
          "_WS": WS}
    exec("class G:\n" + src[start:end], ns)
    return ns["G"]()


MOVE_GROUP = "lib/moveit_ros_move_group/move_group"
KORTEX = "kortex_highlevel_bridge"
CASCADE = "sim_to_real_bridge"
QUEST = "quest_bridge_node"


class TestAModeAdoptsWhatIsRunning(unittest.TestCase):

    def test_a_running_stack_is_adopted_not_refused(self):
        """The VR row's step 1 must not stop the chain when a stack is up.

        This is the whole of "SIM ONLY does nothing": the stack was up, so
        the step was refused, so the VR transport behind it never launched.
        """
        g = _gui_helpers({MOVE_GROUP: 1})
        self.assertTrue(g._step_running("sim_nomaster"),
                        "a running move_group must be recognised as the "
                        "simulation stack, so the step is adopted rather "
                        "than relaunched into a HARD CONSTRAINT 3 refusal")
        self.assertIn("move_group", g._step_running("sim_nomaster"),
                      "the adoption must NAME its evidence -- a silent skip "
                      "is the same shape as the silent refusal it replaces")

    def test_connected_arms_never_get_a_second_kortex_session(self):
        """HARD CONSTRAINT 2, enforced at the button.

        With bridges running, `real` MUST become `real_cascade`. Anything
        else runs start_real.sh, which opens a second session per arm and
        sweeps the working ones on its way out.
        """
        g = _gui_helpers({MOVE_GROUP: 1, KORTEX: 2})
        key, why = g._step_substitute("real")
        self.assertEqual(key, "real_cascade",
                         "with Kortex bridges already up, SIM + REAL must "
                         "cascade onto them, never start_real.sh")
        self.assertTrue(why, "a substitution the operator did not ask for "
                             "must say why it happened")
        self.assertIn("ALREADY CONNECTED", why)

    def test_with_no_arms_connected_the_real_step_is_left_alone(self):
        """The substitution is conditional, not a rename.

        On a cold rig `start_real.sh` is the right thing and must still run:
        it is the only entry that BRINGS UP the real side. A substitution
        that fired unconditionally would leave no way to connect at all.
        """
        g = _gui_helpers({MOVE_GROUP: 1})
        key, why = g._step_substitute("real")
        self.assertEqual(key, "real", "with no bridge running there is "
                                      "nothing to cascade onto")
        self.assertIsNone(why)

    def test_the_probe_can_say_no(self):
        """Deliberately broken input: an EMPTY process table.

        Every adoption above must reverse. A probe that returns evidence
        from an empty table would make every step look already-satisfied and
        the modes would launch nothing at all -- which presents exactly like
        the defect this file pins.
        """
        g = _gui_helpers({})
        for key in ("sim", "sim_nomaster", "autonomy", "autonomy_nomaster",
                    "vr", "real", "real_cascade"):
            self.assertIsNone(g._step_running(key),
                              "%s reported evidence from an empty process "
                              "table" % key)
        self.assertEqual(g._step_substitute("real")[0], "real")

    def test_every_step_of_every_mode_row_can_be_probed(self):
        """No mode may contain a step whose end state cannot be checked.

        A key missing from STEP_PROC probes as None, i.e. "not running", so
        it is launched every time -- which is the old behaviour, restored
        silently for that one row. The MODE_ROWS table and the probe table
        must not be able to drift apart.
        """
        # The MODE_ROWS table became the DRIVE tab's start_mode() calls on
        # 2026-08-27; the invariant is unchanged -- every step key any
        # sequence uses must be probe-able -- so parse the call sites.
        src = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
        keys = set()
        for seq in re.findall(r"start_mode\(\s*\[([^\]]*)\]", src):
            keys.update(x.strip().strip("\"'")
                        for x in seq.split(",") if x.strip())
        self.assertTrue(keys, "parsed no steps out of any start_mode() call")
        g = _gui_helpers({})
        for k in sorted(keys) + ["real", "real_mock"]:
            self.assertIn(k, g.STEP_PROC if k != "real_mock" else
                          set(g.STEP_PROC) | {"real_mock"},
                          "mode step %r has no way to check whether it is "
                          "already running" % k)


class TestTheVrRouteIsOneThisMachineCanTake(unittest.TestCase):
    """The VR button launched a route that dies on its first line.

    `vr_connect.sh` needs adb. docs/ENGINEERING_LOG.md records "no adb" for this host under
    Blocked on the lab, so the spawned process refused and exited before
    anything drew -- indistinguishable from a dead button.
    """

    def test_the_spec_follows_adb(self):
        from srl_teleop import gui_launch_specs as g
        vr = [s for s in g.MODES if s.key == "vr"]
        self.assertEqual(len(vr), 1)
        script = os.path.basename(vr[0].argv[0])
        if g.have_adb():
            self.assertEqual(script, "vr_connect.sh")
        else:
            self.assertEqual(
                script, "start_vr_wifi.sh",
                "with no adb the USB route cannot run, so the button must "
                "take the WebXR route instead of spawning a refusal")

    def test_both_routes_exist_on_disk(self):
        for s in ("vr_connect.sh", "start_vr_wifi.sh", "start_cascade.sh"):
            self.assertTrue(
                os.access(os.path.join(WS, "scripts", s), os.X_OK),
                "%s is missing or not executable" % s)

    def test_the_cascade_script_opens_no_kortex_session(self):
        """It must RELAY onto a bridge, never start one (HARD CONSTRAINT 2).

        Asked of the EXECUTABLE LINES. The header explains at length why
        `start_real.sh` and `real.launch.py` are the wrong thing here, and a
        naive substring search over the whole file fails on the prose that
        exists to prevent the mistake -- a check that punishes the comment
        for describing the bug.
        """
        src = open(os.path.join(WS, "scripts", "start_cascade.sh")).read()
        code = "\n".join(ln for ln in src.splitlines()
                         if not ln.lstrip().startswith("#"))
        for forbidden in ("real.launch.py", "start_real.sh",
                          "kortex_highlevel_bridge --ros-args"):
            self.assertNotIn(forbidden, code,
                             "%s would open a Kortex session" % forbidden)
        self.assertIn("sim_to_real_bridge", code)
        self.assertIn("pgrep -f \"kortex_highlevel_bridge_${arm}\"", code,
                      "it must REQUIRE an existing bridge per arm")


class TestVrDoesNotStartTheMasterArm(unittest.TestCase):
    """`master_pose_node` is respawn=True and there is no Teensy.

    Five live instances starved the controller manager on 2026-08-26 until
    joint_state_broadcaster never came up, /joint_states never published, and
    the real cascade refused with "Is the sim controller up?" -- which points
    at the simulation when the cause is a missing cable. Mode 1's input IS
    the master arm so it keeps `sim`; mode 2's is a pair of controllers.
    """

    def test_the_vr_row_uses_the_nomaster_stack(self):
        # Any start_mode() sequence that brings up the VR transport must
        # ride a NO-MASTER stack: `master_pose_node` is respawn=True, so
        # with no Teensy it dies and respawns for ever -- five instances
        # starved the controller manager on 2026-08-26.
        src = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
        vr_seqs = [s for s in re.findall(r"start_mode\(\s*\[([^\]]*)\]", src)
                   if '"vr"' in s]
        self.assertTrue(vr_seqs, "no start_mode() sequence launches vr")
        for s in vr_seqs:
            self.assertIn("nomaster", s,
                          "a VR sequence rides a stack WITH the master arm: "
                          "%r" % s)

    def test_the_nomaster_stack_actually_passes_the_argument(self):
        from srl_teleop import gui_launch_specs as g
        sp = [s for s in g.MODES if s.key == "sim_nomaster"][0]
        self.assertIn("master:=false", sp.argv)
        self.assertTrue(sp.starts_stack)

    def test_mode_one_still_has_its_master_arm(self):
        from srl_teleop import gui_launch_specs as g
        sp = [s for s in g.MODES if s.key == "sim"][0]
        self.assertNotIn("master:=false", sp.argv,
                         "MASTER TELEOP without master_pose_node has no "
                         "input at all")


if __name__ == "__main__":
    unittest.main()
