"""GO HOME must reach the arm while teleoperation is running.

THE DEFECT, reported from the rig 2026-08-29: "our pose panel doesn't work
when teleoperation". The buttons were not dead. They were OUTVOTED.

`vr_pose_mapper` runs with `hold_when_idle` true, and that is deliberate:
whenever it is not driving a hand it publishes the arm's OWN live pose at
20 Hz, so that a clutch release cannot let /master_arm_pose_<arm> go stale
and latch the dead-man's e-stop. `ik_follower` turns that stream into
trajectories on /<arm>_arm_controller/joint_trajectory.

The pose button published its target THREE times onto that same topic. Twenty
messages a second saying "stay exactly where you are" against three saying
"go home": the target was overwritten inside 50 ms and the arm never moved.

Two writers on one topic. The fix is NOT to publish harder -- that is a race,
not a design, and whichever side won would win by accident. The mover CLAIMS
the topic on /pose_move_active, `ik_follower` stands down for the duration,
and the mapper keeps publishing throughout so the dead-man stays fed and the
reason `hold_when_idle` exists is not undone.

The last group is the one that matters for safety: the claim must always come
back. A claim that could stick would be a way to disable the master arm from
a button labelled GO HOME.
"""
import os
import re
import unittest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
GUI = open(os.path.join(WS, "scripts", "srl_gui.py")).read()
IKF = open(os.path.join(WS, "src", "srl_teleop", "srl_teleop",
                        "ik_follower_node.py")).read()
MAP = open(os.path.join(WS, "src", "srl_vr_teleop", "srl_vr_teleop",
                        "vr_pose_mapper.py")).read()


class TestTheFollowerStandsDown(unittest.TestCase):

    def test_it_subscribes_to_the_claim(self):
        self.assertIn('"/pose_move_active"', IKF,
                      "the follower is the other writer; it has to hear the "
                      "claim or the pose button is still outvoted")

    def test_the_tracking_publish_is_gated(self):
        # The 20 Hz path -- the one that actually overwrote the target.
        # Sliced to the END of the gate (`self.pub.publish`) rather than a
        # fixed byte count: the first version cut at 400 chars and started
        # failing when the comment grew, which is a test measuring its own
        # formatting instead of the code.
        i = IKF.index("        traj.points = [pt]\n        if self.pose")
        gate = IKF[i:IKF.index("        self.pub.publish(traj)", i)]
        self.assertIn("if self.pose_move_active:", gate)
        self.assertIn("return", gate,
                      "the command must be DROPPED, not queued: by the time "
                      "the move ends this target is stale")

    def test_dropping_the_command_resyncs_the_generator(self):
        """The half that `test_every_refusal_after_the_step_resyncs_the
        _generator` caught me missing.

        The generator has already taken its step by this point, so it
        believes it commanded a position that was never published -- while
        the arm is being driven somewhere else entirely by the pose move.
        Without the resync, the first command after the release is computed
        from a pose the arm was never in.
        """
        i = IKF.index("        traj.points = [pt]\n        if self.pose")
        gate = IKF[i:IKF.index("        self.pub.publish(traj)", i)]
        self.assertIn("self._refuse()", gate)
        self.assertLess(gate.index("self._refuse()"), gate.rindex("return"),
                        "resync BEFORE returning, or it never happens")

    def test_the_unwind_publish_is_gated_too(self):
        """Unwind is a second publisher on the same topic and would fight the
        move just as hard, once, at the worst possible moment.

        ANCHORED ON THE WHOLE METHOD, not on a byte window after the
        trajectory is built. The first version of this check read 400
        characters past `build_unwind_trajectory` -- which is the same
        mistake `test_the_tracking_publish_is_gated` records above, and it
        bites harder here, because the CORRECT place for this guard is not
        beside the publish at all. `startup_check` cancels its own repeating
        timer on the way past; a guard below that cancel would return with
        nothing left to re-arm it and disable tracking permanently. The guard
        therefore has to sit ABOVE the cancel, where the timer simply comes
        round again once the claim is released.
        """
        body = IKF[IKF.index("    def startup_check(self):"):
                   IKF.index("    def build_unwind_trajectory(self")]
        self.assertIn("if self.pose_move_active:", body,
                      "the unwind path does not check the claim at all")
        self.assertLess(
            body.index("self.pose_move_active"),
            body.index("self.startup_timer.cancel()"),
            "the guard is below the timer cancel, so a deferred unwind can "
            "never be retried and tracking stays disabled for ever")
        self.assertLess(
            body.index("self.pose_move_active"),
            body.index("self.pub.publish(traj)"),
            "the claim is checked after the trajectory has already gone out")


class TestTheMoverClaimsAndHolds(unittest.TestCase):

    def test_the_sim_path_claims_the_topic(self):
        body = GUI[GUI.index("        if chosen.startswith(\"/real/\"):"):
                   GUI.index("    POSE_CLAIM_MAX_S")]
        self.assertIn("self.claim_arm_topic(True)", body)

    def test_the_sim_path_holds_rather_than_firing_three_times(self):
        body = GUI[GUI.index("        if chosen.startswith(\"/real/\"):"):
                   GUI.index("    POSE_CLAIM_MAX_S")]
        self.assertIn("_start_hold", body,
                      "three publishes is what lost the argument; the target "
                      "has to be held so arrival is something to wait for")
        self.assertNotIn("for _ in range(3):", body)


class TestTheClaimAlwaysComesBack(unittest.TestCase):
    """The safety half. A stuck claim disables teleoperation silently."""

    def test_it_is_released_when_no_hold_remains(self):
        loop = GUI[GUI.index("    def _hold_loop"):
                   GUI.index("    def release_hold")]
        self.assertIn("if not items and getattr(self, \"_claim_t\", 0.0):",
                      loop)
        self.assertIn("self.claim_arm_topic(False)", loop)

    def test_it_is_released_on_a_ceiling_even_if_the_hold_persists(self):
        loop = GUI[GUI.index("    def _hold_loop"):
                   GUI.index("    def release_hold")]
        self.assertIn("claim_expired()", loop)

    def test_the_ceiling_is_bounded_and_sane(self):
        m = re.search(r"POSE_CLAIM_MAX_S = ([\d.]+)", GUI)
        self.assertIsNotNone(m)
        v = float(m.group(1))
        self.assertGreater(v, 5.0, "shorter than a real pose move")
        self.assertLess(v, 120.0, "long enough to feel like a dead master arm")

    def test_release_happens_in_the_loop_not_the_call_site(self):
        # A move can end on arrival, on the ceiling, or on an e-stop. Only
        # the loop sees all three.
        call = GUI[GUI.index("    def send_joint_pose"):
                   GUI.index("    POSE_CLAIM_MAX_S")]
        self.assertNotIn("claim_arm_topic(False)", call,
                         "releasing at the call site misses every ending "
                         "except the tidy one")


class TestTheDeadManIsNotUndone(unittest.TestCase):
    """`hold_when_idle` is why the mapper publishes at all; the claim must
    not be 'fix it by silencing the mapper'."""

    def test_the_mapper_is_untouched_by_the_claim(self):
        self.assertNotIn("pose_move_active", MAP,
                         "the mapper must keep publishing through a pose "
                         "move -- it is what feeds the dead-man. Only the "
                         "arm-controller topic changes hands.")

    def test_hold_when_idle_still_exists(self):
        self.assertIn("hold_when_idle", MAP)


if __name__ == "__main__":
    unittest.main()
