#!/usr/bin/env python3
"""THE FIVE CONTROL MODES. This is the ONLY file allowed to name one.

    MASTER_TELEOP           mannequin master arm, robot copies, robot decides
                            nothing
    MASTER_SHARED_AUTONOMY  master drives position; autonomy supplies wrist
                            orientation on approach; the human confirms
    VR_TELEOP               Quest controller, 6-DOF, no dead channels
    VR_SHARED_AUTONOMY      VR position plus the same assistance
    FULL_AUTONOMY           spoken or typed goal; detect, plan, announce,
                            wait for confirmation, execute

Each adapter answers exactly four questions from the task layer and keeps its
own log of what it did internally.  Those internal logs are EXPECTED TO
DIFFER -- that difference is what makes the independent variable real, and
`test_mode_independence.py` asserts it, because five adapters whose logs are
identical are five names for one mode.

WHAT DIFFERS, CONCRETELY, AND WHERE IT COMES FROM
-------------------------------------------------
  entry topic        MASTER_* publish on /master_arm_pose_<arm>; VR_* go
                     through vr_pose_mapper onto the SAME topic; autonomy
                     publishes /autonomy/assist_pose_<arm>.  All three land
                     at ik_follower_node's request_ik, so the entire safety
                     stack is in every path -- that is deliberate and is why
                     the modes are comparable at all.
  orientation        teleop modes are pinned: orientation_mode is `fixed` and
                     nothing in the master measures the wrist.  Shared and
                     full autonomy supply the wrist on approach.  Measured:
                     pinned gets 86.7% of the left arm's graspable band and
                     0% of the right arm's MIRRORED band, but 5/12 in the
                     right arm's OWN band -- so the pin costs position, not
                     orientation.  Yaw-free recovers left 100%, right 11/12,
                     which is why T1's objects are cylinders.
  confirmation       teleop: the human is already driving, so there is
                     nothing to confirm and await_operator returns at once.
                     shared: the human confirms the grasp.  full autonomy:
                     the system announces intent and WAITS.
  rate               master 50 Hz; VR measured 68.5-72 Hz with a round trip
                     of 8.66 ms median.

ONE SOURCE OWNS THE GRAPH AT A TIME.  Measured: after a VR run, modes 01/04/06
all recorded 0.0000 m of travel until vr_pose_mapper was killed -- it stays
engaged and keeps publishing on the same topic mode 01 uses.  `owns_topic()`
declares what each mode holds so the conductor can enforce the rule at process
level rather than trusting it.
"""
from __future__ import annotations

from dataclasses import dataclass, field

MODE_NAMES = ("MASTER_TELEOP", "MASTER_SHARED_AUTONOMY", "VR_TELEOP",
              "VR_SHARED_AUTONOMY", "FULL_AUTONOMY")


class Transport:
    """Where a commanded pose actually goes.

    Swappable so the adapters can be exercised without a stack -- but note
    what is NOT swapped: the adapters themselves.  A proof that ran against
    reimplemented adapters would be constructing the environment where the
    bug cannot occur, which is a failure class this project keeps a list of.
    """

    def send_pose(self, topic, arm, pose, phase):
        raise NotImplementedError

    def send_gripper(self, topic, arm, opening, phase):
        raise NotImplementedError

    def ask_human(self, channel, intent):
        raise NotImplementedError


@dataclass
class RecordingTransport(Transport):
    """Records instead of publishing.  Used by the proof and by --dry-run.

    `reachable` and `grippable` are callables so a test can make a pose fail
    and check the task layer degrades the same way in every mode.
    """
    sent: list = field(default_factory=list)
    reachable: object = None
    grippable: object = None
    human_says: bool = True

    def send_pose(self, topic, arm, pose, phase):
        self.sent.append(("pose", topic, arm, tuple(pose), phase))
        if self.reachable is not None and not self.reachable(arm, pose):
            return None
        return list(pose)

    def send_gripper(self, topic, arm, opening, phase):
        self.sent.append(("grip", topic, arm, opening, phase))
        if self.grippable is not None and not self.grippable(arm, opening):
            return None
        return opening

    def ask_human(self, channel, intent):
        self.sent.append(("ask", channel, intent))
        return self.human_says


class _Base:
    """Shared plumbing.  Deliberately NOT a place to put behaviour: anything
    that differs between modes belongs in the subclass, where it is visible."""

    name = "abstract"
    pose_topic = "/master_arm_pose_%s"
    grip_topic = "/gripper_cmd_%s"
    supplies_wrist = False
    confirm_channel = None            # None == nothing to confirm
    input_rate_hz = 0.0
    # The physical channel the pose ORIGINATES from.  MASTER_TELEOP and
    # VR_TELEOP publish on the same topic with the same pinned wrist and the
    # same absent confirmation, so without this they are indistinguishable in
    # the log -- and the proof caught exactly that.  They are not the same
    # mode: one is 7 potentiometers of which half are currently incoherent at
    # 50 Hz, the other is a 6-DOF controller at ~70 Hz with no dead channels.
    # That difference IS the VR-versus-master hypothesis, so it is recorded.
    input_source = "none"

    def __init__(self, transport: Transport):
        self.t = transport
        self._log = []

    def _note(self, *row):
        self._log.append(row)

    def log(self):
        return list(self._log)

    def owns_topic(self):
        """Topics this mode publishes on.  Two modes owning the same topic
        may not run at once; the conductor enforces it."""
        return tuple(self.pose_topic % a for a in ("left", "right"))

    def command_pose(self, arm, pose, phase):
        p = list(pose)
        if self.supplies_wrist and phase in ("APPROACH", "DESCEND"):
            # Autonomy contributes the wrist on approach.  Position stays
            # 100% the operator's in every state -- that line is the safety
            # argument of the shared-autonomy design and must not move here.
            self._note("assist_wrist", arm, phase)
        self._note("pose", arm, phase, self.pose_topic % arm,
                   self.input_source)
        return self.t.send_pose(self.pose_topic % arm, arm, p, phase)

    def command_gripper(self, arm, opening, phase):
        self._note("grip", arm, phase, opening)
        return self.t.send_gripper(self.grip_topic % arm, arm, opening, phase)

    def await_operator(self, intent):
        if self.confirm_channel is None:
            # The human is already driving the arm; there is nobody else to
            # ask, and inserting a prompt here would add a step teleop does
            # not have and break the comparison.
            self._note("no_confirm_needed", intent)
            return True
        self._note("confirm", self.confirm_channel, intent)
        return self.t.ask_human(self.confirm_channel, intent)


class MasterTeleop(_Base):
    name = "MASTER_TELEOP"
    input_source = "master_arm"
    input_rate_hz = 50.0


class MasterSharedAutonomy(_Base):
    name = "MASTER_SHARED_AUTONOMY"
    input_source = "master_arm"
    pose_topic = "/autonomy/assist_pose_%s"
    supplies_wrist = True
    confirm_channel = "master_button"
    input_rate_hz = 50.0


class VrTeleop(_Base):
    name = "VR_TELEOP"
    input_source = "vr_controller"
    # vr_pose_mapper republishes onto the master topic; that is why a live VR
    # mapper silently freezes modes 01/04/06.
    pose_topic = "/master_arm_pose_%s"
    input_rate_hz = 70.0

    def owns_topic(self):
        return super().owns_topic() + ("/vr/controller_pose_left",
                                       "/vr/controller_pose_right")


class VrSharedAutonomy(VrTeleop):
    name = "VR_SHARED_AUTONOMY"
    input_source = "vr_controller"
    pose_topic = "/autonomy/assist_pose_%s"
    supplies_wrist = True
    confirm_channel = "vr_trigger"


class FullAutonomy(_Base):
    name = "FULL_AUTONOMY"
    input_source = "voice"
    pose_topic = "/autonomy/assist_pose_%s"
    supplies_wrist = True
    confirm_channel = "voice"
    input_rate_hz = 0.0

    def await_operator(self, intent):
        """Announce, THEN wait.  The 2008 ms measured from utterance to
        motion is 2000 ms of this deliberate wait; it is a safety choice and
        is reported as t_dispatch, never folded into movement time."""
        self._note("announce", intent)
        return super().await_operator(intent)


ADAPTERS = {
    "MASTER_TELEOP": MasterTeleop,
    "MASTER_SHARED_AUTONOMY": MasterSharedAutonomy,
    "VR_TELEOP": VrTeleop,
    "VR_SHARED_AUTONOMY": VrSharedAutonomy,
    "FULL_AUTONOMY": FullAutonomy,
}


def build(mode: str, transport: Transport):
    if mode not in ADAPTERS:
        raise KeyError("unknown mode %r; known: %s"
                       % (mode, ", ".join(sorted(ADAPTERS))))
    return ADAPTERS[mode](transport)


def conflicting_modes(a: str, b: str, transport=None) -> bool:
    """True if two modes may not be up at once because they publish on the
    same topic.  Measured consequence of ignoring this: after a VR run, three
    other modes recorded 0.0000 m of arm travel."""
    tr = transport or RecordingTransport()
    return bool(set(build(a, tr).owns_topic()) & set(build(b, tr).owns_topic()))
