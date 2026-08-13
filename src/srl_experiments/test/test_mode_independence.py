#!/usr/bin/env python3
"""THE MODE-INDEPENDENCE PROOF. Build order item 1, before any task exists.

The claim under test: running the same TaskCommand under all five control
modes produces an IDENTICAL task-layer trace signature.  If it does not, the
five conditions are not measuring the same task and every comparison in the
study is between two different procedures.

A PASS HERE MUST NOT BE CHEAP, so four things are asserted, not one:

  1. POSITIVE   all five signatures are identical.
  2. NOT VACUOUS  the signature is non-empty and contains the phases the verb
                  is supposed to have.  "All five produced nothing" is also
                  five identical signatures, and it is the answer a broken
                  task layer gives.  The project rule -- a sweep must REFUSE
                  when tested == 0 -- applies to proofs too.
  3. THE MODES ARE REALLY DIFFERENT  the five ADAPTER logs must not all be
                  identical.  Five adapters that behave the same way are five
                  names for one mode, and identity in (1) would then be
                  trivially true and would mean nothing.
  4. THE HARNESS CAN FAIL  a deliberately mode-leaking task layer is run
                  through the same comparison and MUST be caught.  Without
                  this the whole file is a green light that has never been
                  shown capable of turning red -- the same reason the clip
                  verifier refuses to report until it has caught constructed
                  broken clips.

Degradation is part of the invariant: when a pose is unreachable, every mode
must fail at the SAME phase with the SAME status.  A mode that silently
retried would produce a different trace and would have a different completion
time for a reason that is not the mode.
"""
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from srl_experiments import mode_adapters as MA          # noqa: E402
from srl_experiments import task_actions as TA           # noqa: E402

ALL_MODES = list(MA.MODE_NAMES)


def script():
    """One of every verb, so the proof covers the whole API rather than the
    one verb that happens to be easy."""
    L = [0.35, 0.20, 1.12]
    R = [-0.35, 0.20, 1.12]
    return [
        TA.TaskCommand(TA.Verb.REACH_TARGET, "left", "L2", pose=L),
        TA.TaskCommand(TA.Verb.PICK_OBJECT, "left", "cyl_blue_1", pose=L),
        TA.TaskCommand(TA.Verb.PLACE_OBJECT, "left", "cyl_blue_1", pose=L,
                       to_pose=[0.40, 0.22, 1.12]),
        TA.TaskCommand(TA.Verb.GRASP, "right", "box", pose=R),
        TA.TaskCommand(TA.Verb.RELEASE, "right", "box", pose=R),
        TA.TaskCommand(TA.Verb.TRANSFER, "both", "tray",
                       pose=[0.0, 0.35, 1.10], to_pose=[0.0, 0.35, 1.30],
                       params={"grip_sep_m": 0.50,
                               "waypoints": [[0.0, 0.35, 1.10],
                                             [0.0, 0.35, 1.20],
                                             [0.0, 0.35, 1.30]]}),
        TA.TaskCommand(TA.Verb.RETURN, "right", "box", pose=R, to_pose=R),
        # MOVE_TO: go somewhere NAMED, with no object at all. It is the one
        # verb whose target is a place rather than a thing, and it has to be
        # here for the same reason as the rest -- the proof covers the whole
        # API or it covers the verb that happens to be easy. The pose is a
        # measured cell; see srl_autonomy.named_places.
        TA.TaskCommand(TA.Verb.MOVE_TO, "left", "left side", pose=L),
    ]


def run(mode, **kw):
    tr = MA.RecordingTransport(**kw)
    ad = MA.build(mode, tr)
    clock = iter(range(10000))
    _, sig = TA.run_script(ad, script(), clock=lambda: next(clock) * 0.01)
    return sig, ad.log(), tr


# ---------------------------------------------------------------- 1 + 2
def test_all_five_modes_produce_the_same_trace():
    sigs = {m: run(m)[0] for m in ALL_MODES}
    ref = sigs[ALL_MODES[0]]

    # (2) NOT VACUOUS -- refuse a pass on an empty or truncated trace.
    assert len(ref) > 0, "empty signature: five modes agreeing on nothing"
    verbs = {s[0] for s in ref}
    assert verbs == {v.value for v in TA.Verb}, (
        "the script did not exercise every verb: missing %s"
        % ({v.value for v in TA.Verb} - verbs))
    phases = {s[3] for s in ref}
    for needed in ("APPROACH", "DESCEND", "CLOSE", "OPEN", "LIFT",
                   "CARRY", "RETREAT", "VERIFY", "AWAIT_OPERATOR"):
        assert needed in phases, "phase %s never appeared" % needed

    # (1) POSITIVE
    for m, s in sigs.items():
        assert s == ref, (
            "mode %s produced a different task trace.\n  first divergence: %s"
            % (m, next((i, a, b) for i, (a, b) in enumerate(zip(s, ref))
                       if a != b)))


def test_the_five_modes_are_actually_different():
    """(3) If the adapter logs are all identical the modes are one mode, and
    the identity above would be trivially true."""
    logs = {m: tuple(run(m)[1]) for m in ALL_MODES}
    assert len(set(logs.values())) == len(ALL_MODES), (
        "modes with identical adapter behaviour: %s"
        % [m for m in ALL_MODES
           if list(logs.values()).count(logs[m]) > 1])

    # And the differences must be the ones the design claims.
    assert any("assist_wrist" in str(r) for r in logs["MASTER_SHARED_AUTONOMY"])
    assert not any("assist_wrist" in str(r) for r in logs["MASTER_TELEOP"])
    assert any("announce" in str(r) for r in logs["FULL_AUTONOMY"])
    assert any("no_confirm_needed" in str(r) for r in logs["VR_TELEOP"])


def test_topics_differ_where_the_design_says_they_do():
    tr = MA.RecordingTransport()
    assert MA.build("MASTER_TELEOP", tr).pose_topic == "/master_arm_pose_%s"
    assert MA.build("VR_TELEOP", tr).pose_topic == "/master_arm_pose_%s"
    assert (MA.build("FULL_AUTONOMY", tr).pose_topic
            == "/autonomy/assist_pose_%s")
    # The measured trap: a live VR mapper freezes the master-driven modes.
    assert MA.conflicting_modes("VR_TELEOP", "MASTER_TELEOP")
    assert not MA.conflicting_modes("VR_TELEOP", "FULL_AUTONOMY")


# ---------------------------------------------------------------- 4
class _LeakyLayer(TA.TaskLayer):
    """A task layer that branches on the mode -- the exact defect the design
    forbids, written deliberately so the comparison can be shown to catch it.
    It skips the standoff for VR, which is the plausible version of this bug:
    "VR is 6-DOF and accurate, it does not need the approach height"."""

    def _pick(self, out, cmd):
        if "VR" in self.a.name:
            if not self._confirm(out, cmd, "pick %s" % cmd.subject):
                return TA.Status.REFUSED
            self._move(out, cmd, cmd.arm, cmd.pose, TA.Phase.DESCEND)
            self._grip(out, cmd, cmd.arm, TA.GRIPPER_CLOSED, TA.Phase.CLOSE)
            return TA.Status.OK
        return super()._pick(out, cmd)


def test_the_proof_can_fail():
    """(4) NEGATIVE CONTROL on the harness itself."""
    sigs = {}
    for m in ALL_MODES:
        tr = MA.RecordingTransport()
        tl = _LeakyLayer(MA.build(m, tr), clock=lambda: 0.0)
        sigs[m] = tuple(s for c in script() for s in tl.execute(c).signature())
    assert len(set(sigs.values())) > 1, (
        "a task layer that branches on the mode was NOT caught -- this "
        "comparison cannot fail, so its passes mean nothing")


def test_no_mode_names_in_the_task_layer():
    """Structural guard.  The empirical proof only covers the paths the
    script walks; this covers the file."""
    src = open(os.path.join(HERE, "..", "srl_experiments",
                            "task_actions.py")).read()
    body = re.sub(r'"""[\s\S]*?"""', "", src)          # strip docstrings
    body = re.sub(r"#.*", "", body)                    # and comments
    for m in ALL_MODES + ["VR", "MASTER", "AUTONOMY", "TELEOP"]:
        assert m not in body, (
            "task_actions.py names the mode %r outside a comment -- the task "
            "layer must not know which mode drives it" % m)


# ---------------------------------------------------------------- failure
def test_failure_degrades_identically_in_every_mode():
    """An unreachable pose must fail at the SAME phase with the SAME status
    everywhere.  A mode that silently retried would post a different
    completion time for a reason that is not the mode."""
    def blocked(arm, pose):
        return pose[2] < 1.25            # the LIFT of the pick is refused

    sigs = {m: run(m, reachable=blocked)[0] for m in ALL_MODES}
    ref = sigs[ALL_MODES[0]]
    assert any(s[4] == "UNREACHABLE" for s in ref), (
        "the injected failure never fired -- this test proves nothing")
    for m, s in sigs.items():
        assert s == ref, "mode %s degraded differently" % m


def test_refusal_is_only_expressible_in_three_of_five_modes():
    """A REAL ASYMMETRY, found by this proof and kept rather than smoothed
    over, because it changes the analysis.

    The first version of this test asserted that a refusal degrades
    identically everywhere.  It does not, and it should not: refusal needs
    somebody to refuse, and in MASTER_TELEOP and VR_TELEOP the human IS the
    controller.  There is no confirmation step to decline -- an unwilling
    operator simply does not move the arm, which surfaces as a TIMEOUT, not
    as REFUSED.

    So `status == REFUSED` can only ever occur in MASTER_SHARED_AUTONOMY,
    VR_SHARED_AUTONOMY and FULL_AUTONOMY.  Consequences that must not be
    forgotten downstream:
      * a per-condition count of refusals is structurally zero in two cells
        and MUST NOT be compared across all five as though it were a
        measurement;
      * any trial-exclusion rule keyed on refusal excludes from three
        conditions only, which would bias the comparison it feeds.
    The invariant the study actually needs is the one asserted above: when
    the operator PROCEEDS, all five traces are identical.
    """
    sigs = {m: run(m, human_says=False)[0] for m in ALL_MODES}
    can_refuse = [m for m in ALL_MODES
                  if any(s[4] == "REFUSED" for s in sigs[m])]
    assert set(can_refuse) == {"MASTER_SHARED_AUTONOMY",
                               "VR_SHARED_AUTONOMY", "FULL_AUTONOMY"}, (
        "the set of modes that can express a refusal changed: %s"
        % can_refuse)

    # Within the three that CAN refuse, they must do so identically.
    ref = sigs["MASTER_SHARED_AUTONOMY"]
    for m in can_refuse:
        assert sigs[m] == ref, "mode %s refused differently" % m

    # And the two that cannot still emit the phase, so the trace SHAPE is
    # unchanged -- which is what makes the proceed-case identity possible.
    for m in ("MASTER_TELEOP", "VR_TELEOP"):
        assert any(s[3] == "AWAIT_OPERATOR" for s in sigs[m])


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
