"""START VR TELEOP must not start the stack that kills itself.

MEASURED 2026-08-29, twice in one session.

`master_pose_node` is `respawn=True` on purpose: a Teensy attached after
launch then connects on its own without restarting the stack. With no board
on the bus AT ALL that becomes a process dying and respawning every five
seconds for ever. `gui_launch_specs` already records the cost, from
2026-08-26 -- five live instances, load average 8-11, the controller manager
overrunning, every spawner timing out, `joint_state_broadcaster` never coming
up, `/joint_states` with no publisher -- and it added a whole `sim_nomaster`
spec so the VR modes could avoid it.

THE VR BRING-UP NEVER USED THAT SPEC. `_sim_argv` is the one place both the
simulation step and its repair start the sim, and it asked for the
master-ful stack unconditionally. On this machine `/dev/ttyACM*` does not
exist. So `START VR TELEOP` -- the single button the operator is told to
press -- started a stack that could not survive, on every press.

It died twice. The second time it took the operations window's own ROS
context with it, because the window was the launch's parent, and the window
went on drawing stale readings. The operator's report was "the real arm is
not moving", four layers down from the cause.

The VR operator holds controllers. The master arm is not their input, so
there is nothing to lose by leaving it out when there is no board to read.
"""
import os
import sys
import types
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_teleop"))

SRC = open(os.path.join(
    WS, "src", "srl_teleop", "srl_teleop", "vr_bringup.py")).read()


def _argv(devices, rviz=True):
    """`_sim_argv` over a chosen /dev, sliced out rather than imported --
    vr_bringup pulls in rclpy and a great deal else."""
    ns = {"os": os,
          "glob": types.SimpleNamespace(glob=lambda pat: list(devices)),
          "_sim_rviz": lambda: rviz}
    body = SRC[SRC.index("def _master_present():"):
               SRC.index("# What a launch that is still coming up looks like")]
    exec(body, ns)                             # noqa: S102 -- code under test
    w = types.SimpleNamespace(ws=WS)
    return ns["_sim_argv"](w)


class TestNoTeensyMeansNoMasterNode(unittest.TestCase):

    def test_it_leaves_the_master_out_when_no_board_is_present(self):
        self.assertIn("master:=false", _argv([]),
                      "with no /dev/ttyACM* the master node respawns for "
                      "ever and starves the controller manager -- the stack "
                      "this button starts must be one that can survive")

    def test_it_keeps_the_master_when_a_board_is_present(self):
        # The respawn behaviour is deliberate and must not be disabled for a
        # rig that actually has the hardware.
        self.assertNotIn("master:=false", _argv(["/dev/ttyACM0"]))

    def test_a_usb_serial_board_counts_too(self):
        self.assertNotIn("master:=false", _argv(["/dev/ttyUSB0"]))


class TestItIsStillTheSameOneCommand(unittest.TestCase):
    """`_sim_argv` exists so the step and its repair start the SAME thing.
    A conditional must not fork that."""

    def test_it_is_still_run_teleop_with_the_gate_off(self):
        argv = _argv([])
        self.assertTrue(argv[0].endswith("scripts/run_teleop.sh"))
        self.assertIn("gate:=false", argv)

    def test_rviz_is_still_honoured_independently(self):
        self.assertIn("use_rviz:=false", _argv([], rviz=False))
        self.assertNotIn("use_rviz:=false", _argv([], rviz=True))

    def test_the_board_is_looked_up_every_call(self):
        # Globbed, never cached: the board arrives with a `usbipd attach` and
        # moves between ACM0 and ACM1 when it does.
        seen = []

        def counting_glob(pat):
            seen.append(pat)
            return []

        ns = {"os": os, "glob": types.SimpleNamespace(glob=counting_glob),
              "_sim_rviz": lambda: True}
        body = SRC[SRC.index("def _master_present():"):
                   SRC.index("# What a launch that is still coming up looks like")]
        exec(body, ns)                         # noqa: S102
        w = types.SimpleNamespace(ws=WS)
        ns["_sim_argv"](w)
        ns["_sim_argv"](w)
        self.assertGreaterEqual(len(seen), 2,
                                "a cached answer is wrong by the next session")


if __name__ == "__main__":
    unittest.main()
