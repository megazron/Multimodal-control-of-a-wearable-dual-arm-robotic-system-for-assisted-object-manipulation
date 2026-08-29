"""Adopting an open Kortex session must leave the arm HOMEABLE.

THE DEADLOCK THIS PINS, measured on the rig 2026-08-29.

`start_cascade.sh` exists because `start_real.sh` cannot be run against a rig
whose bridges are already connected -- the arm permits one Kortex session
(HARD CONSTRAINT 2) and start_real's cleanup sweeps the working bridges on
its way out. So the cascade adopts the open session and starts the relay,
`sim_to_real_bridge`, which is the only thing that puts the simulation onto
the metal.

Two facts then close on each other:

  * HARD CONSTRAINT 0 -- sim home is the presentation pose and the real arms
    are still at the legacy Kortex home. An adopted session is therefore far
    from the loaded home. Measured that day: **2.729 rad**.
  * `sim_to_real_bridge.enable()` refuses past `enable_gap_rad`, because it
    replays sim angles STARTING at home and enabling would command the
    difference as a jump.

So the relay refused (`cascade_active_left` false, `bridge_status_left[0]`
0.0) and the cure is to home the arm first. But `real_homing_node` was
started only by `real_arms_highlevel.launch.py`, never by the cascade, so on
an adopted session **`/home_arm_left` did not exist** -- HOME BOTH ARMS
answered "service not present -- nothing was sent". The one step that closes
the gap was unreachable, so the relay could never enable, so the arm could
never move. Permanently, every session, with nothing in the window saying
which of the two halves was missing.

The fix is one node with `auto_home:=false`: it serves `/home_arm_<arm>` and
`/home_abort_<arm>` and MOVES NOTHING until somebody presses HOME. That
property is the reason this is safe to start on adoption at all, and it is
the first thing these tests check.
"""
import os
import re
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
SH = open(os.path.join(WS, "scripts", "start_cascade.sh")).read()


class TestTheHomingServiceIsServed(unittest.TestCase):

    def test_it_starts_the_homing_node(self):
        self.assertIn("real_homing_node", SH,
                      "adopting a session without starting real_homing_node "
                      "leaves /home_arm_<arm> unserved, and the relay's "
                      "refusal then has no cure")

    def test_it_is_per_arm_and_named(self):
        # One homing node per arm, distinct node names. "abort homing" that
        # reaches an arbitrary one of two moving arms is worse than no abort.
        self.assertIn('__node:="real_homing_node_${arm}"', SH)
        self.assertIn('-p arm:="${arm}"', SH)


class TestAdoptionNeverMovesMetalOnItsOwn(unittest.TestCase):
    """The safety property that makes this safe to start automatically."""

    def test_auto_home_is_off(self):
        self.assertIn("-p auto_home:=false", SH)

    def test_auto_home_is_never_switched_on(self):
        self.assertNotIn("auto_home:=true", SH,
                         "a script that adopts a session the operator did "
                         "not just start must not decide on its own to "
                         "drive that arm across the room")

    def test_it_still_opens_no_kortex_session(self):
        # The whole reason this script exists. It may only ever REQUIRE a
        # bridge, never start one.
        self.assertNotIn("kortex_highlevel_bridge --ros-args", SH)
        self.assertIn('pgrep -f "kortex_highlevel_bridge_${arm}"', SH)


class TestItIsReachedForTheRigThatNeedsIt(unittest.TestCase):
    """Ordering, and it is load-bearing.

    The arm this matters most for is the one whose relay is ALREADY running
    and refusing -- which is exactly the arm the relay's own
    "cascade ALREADY RUNNING" branch `continue`s past. Placed after that
    branch, the fix does nothing for the only rig that has the bug.
    """

    def test_homing_comes_before_the_relays_early_continue(self):
        # ANCHOR ON THE LAUNCH LINE, NOT THE NAME. The header explains this
        # fix at length, so a bare `SH.index("real_homing_node")` finds the
        # COMMENT at byte 1698 and passes whatever the code does -- an
        # ordering test that cannot fail, caught by its own can-fail case.
        i_home = SH.index("setsid ros2 run srl_teleop real_homing_node")
        i_skip = SH.index('pgrep -f "sim_to_real_bridge_${arm}"')
        self.assertLess(i_home, i_skip,
                        "the homing node must be started BEFORE the branch "
                        "that skips an already-running relay and continues")

    def test_the_skip_branch_still_continues(self):
        # Pin that the branch this ordering exists for is still a `continue`
        # -- if it stops being one, the ordering argument above changes and
        # somebody should re-read it rather than trust a stale test.
        tail = SH[SH.index('pgrep -f "sim_to_real_bridge_${arm}"'):]
        self.assertIn("continue", tail.split("fi")[0])


class TestTheCheckCanFail(unittest.TestCase):
    """A check that cannot fail on a deliberately broken input is not a
    check. Each of these breaks the script in the exact way it was broken
    before, and requires the assertion to notice."""

    def _assert_fails(self, broken, needle, msg):
        self.assertNotIn(needle, broken, msg)

    def test_removing_the_homing_node_is_caught(self):
        broken = re.sub(r"setsid ros2 run srl_teleop real_homing_node.*?&\n",
                        "", SH, flags=re.S)
        self._assert_fails(
            broken, "setsid ros2 run srl_teleop real_homing_node",
            "the fixture did not actually remove the launch line")
        # ...and the property the real test asserts is now absent.
        self.assertNotIn("-p auto_home:=false", broken)

    def test_moving_it_after_the_continue_is_caught(self):
        block = re.search(
            r"  # THE HOMING SERVICE.*?\n  fi\n\n", SH, flags=re.S)
        self.assertIsNotNone(block, "the homing block is not where the "
                                    "ordering test thinks it is")
        moved = SH.replace(block.group(0), "") + block.group(0)
        i_home = moved.index("setsid ros2 run srl_teleop real_homing_node")
        i_skip = moved.index('pgrep -f "sim_to_real_bridge_${arm}"')
        self.assertGreater(i_home, i_skip,
                           "the fixture did not actually move the block")


if __name__ == "__main__":
    unittest.main()
