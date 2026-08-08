#!/usr/bin/env python3
"""THE SIX OPERATING MODES. One definition, imported by every package.

Two axes: WHAT DRIVES THE ARM, and HOW MUCH THE ROBOT DECIDES.

    mode                 input              robot decides
    -------------------- ------------------ ----------------------------------
    1 DIRECT_MANNEQUIN   master arm         nothing
    2 DIRECT_VR          Quest controllers  nothing
    3 ORIENTATION_ASSIST master + vision    wrist angle on approach
    4 SHARED_AUTONOMY    master + vision    target, approach, wrist; the
                                            OPERATOR confirms the grasp
    5 SUPERVISED_AUTO    point or voice     everything after the target is
                                            designated
    6 FULL_AUTONOMY      voice only         identification, grasp, transport,
                                            placement

This lives in `srl_teleop` -- the package that depends on nothing in-repo --
precisely so `srl_autonomy`, `srl_perception`, `srl_vr_teleop` and
`srl_experiments` can all import it without inverting the dependency arrow
that the whole experimental design rests on (teleop is the BASELINE
condition; if it imported autonomy the comparison would be circular).

THE SAFETY INVARIANT
--------------------
`SAFETY_STACK` is the list of mechanisms every mode must pass through. It is
not documentation: `assert_safety_invariant()` is called by the mode manager
on every transition and raises rather than degrading. A mode that could
bypass collision-aware IK or the clearance floor would be a different robot
wearing the same name, and the highest-autonomy mode is the one where nobody
is watching.

WHY MODE 6 IS THE HIGHEST-RISK MODE IN THIS PROJECT
---------------------------------------------------
No operator in the loop, arms mounted beside a person's head, and a
perception system that can be CONFIDENTLY WRONG. In modes 1-4 a human sees
the arm and can stop it. In mode 6 the clearance floor and the e-stop are the
only things between a misdetection and the wearer.
"""
from enum import IntEnum


class Mode(IntEnum):
    DIRECT_MANNEQUIN = 1
    DIRECT_VR = 2
    ORIENTATION_ASSIST = 3
    SHARED_AUTONOMY = 4
    SUPERVISED_AUTO = 5
    FULL_AUTONOMY = 6


# The input each mode consumes. Used to decide which dead-man applies: a mode
# driven by the master needs the master dead-man, a VR mode needs the
# tracking/link freeze, and an autonomous mode needs neither but needs the
# voice-stop and the action timeout instead.
INPUT_SOURCE = {
    Mode.DIRECT_MANNEQUIN: "master",
    Mode.DIRECT_VR: "vr",
    Mode.ORIENTATION_ASSIST: "master",
    Mode.SHARED_AUTONOMY: "master",
    Mode.SUPERVISED_AUTO: "voice_or_point",
    Mode.FULL_AUTONOMY: "voice",
}

# What the ROBOT is permitted to decide. Anything not listed, it may not.
ROBOT_DECIDES = {
    Mode.DIRECT_MANNEQUIN: frozenset(),
    Mode.DIRECT_VR: frozenset(),
    Mode.ORIENTATION_ASSIST: frozenset({"wrist"}),
    Mode.SHARED_AUTONOMY: frozenset({"wrist", "target", "approach"}),
    Mode.SUPERVISED_AUTO: frozenset({"wrist", "approach", "grasp",
                                     "transport", "place"}),
    Mode.FULL_AUTONOMY: frozenset({"wrist", "target", "approach", "grasp",
                                   "transport", "place"}),
}

# EVERY mode passes through all of these. No exceptions, no per-mode opt-out.
SAFETY_STACK = (
    "collision_aware_ik",     # avoid_collisions=True on every /compute_ik
    "clearance_floor",        # measured from TF, not from the commanded pose
    "graduated_avoidance",    # redundancy re-seed before the hard floor
    "joint_wrapping",         # continuous joints take the short way round
    "estop",                  # latching, service-reset only
    "deadman",                # source-timestamp keyed
)

# Extra requirements that apply ONLY to the autonomous modes, because the
# things that make them safe in modes 1-4 (a human looking at the arm) are
# absent.
AUTONOMOUS_EXTRA = {
    Mode.SUPERVISED_AUTO: ("action_timeout", "confidence_floor",
                           "voice_stop", "decision_log"),
    Mode.FULL_AUTONOMY: ("action_timeout", "confidence_floor", "voice_stop",
                         "decision_log", "spoken_confirmation",
                         "world_model_agreement"),
}

# Position/velocity caps per mode. Autonomy runs SLOWER than teleop, not
# faster: there is no operator watching the arm, so the only thing that
# bounds a wrong motion is how long it takes to happen.
MAX_VEL_RAD_S = {
    Mode.DIRECT_MANNEQUIN: 0.60,
    Mode.DIRECT_VR: 0.60,
    Mode.ORIENTATION_ASSIST: 0.60,
    Mode.SHARED_AUTONOMY: 0.60,
    Mode.SUPERVISED_AUTO: 0.25,
    Mode.FULL_AUTONOMY: 0.15,
}

IMPLEMENTED = {
    Mode.DIRECT_MANNEQUIN: "implemented",
    Mode.DIRECT_VR: "implemented",
    Mode.ORIENTATION_ASSIST: "STUB -- orientation_mode stays 'fixed'; "
                             "OrientationConstraint is ignored by this "
                             "/compute_ik plugin (measured: identical IK "
                             "success with and without it)",
    Mode.SHARED_AUTONOMY: "implemented (handover_arbiter, intent_inference, "
                          "grasp_generator)",
    Mode.SUPERVISED_AUTO: "implemented (autonomy_executive, target given)",
    Mode.FULL_AUTONOMY: "implemented (autonomy_executive + voice_intent + "
                        "world_model); PERCEPTION MODELS NOT INSTALLED",
}


def mode_from(value):
    """Accept a name, a number, or a Mode. Raises on anything else.

    Deliberately strict: a typo'd mode name silently falling back to a
    default would choose the robot's autonomy level by accident.
    """
    if isinstance(value, Mode):
        return value
    if isinstance(value, bool):
        raise ValueError("mode must not be a bool")
    if isinstance(value, int):
        return Mode(value)
    s = str(value).strip().upper().replace("-", "_").replace(" ", "_")
    if s.isdigit():
        return Mode(int(s))
    try:
        return Mode[s]
    except KeyError:
        raise ValueError(
            "unknown mode %r. Valid: %s"
            % (value, ", ".join("%d/%s" % (m.value, m.name) for m in Mode)))


def requires_master(mode):
    return INPUT_SOURCE[mode] == "master"


def is_autonomous(mode):
    return mode in (Mode.SUPERVISED_AUTO, Mode.FULL_AUTONOMY)


def required_safety(mode):
    """Everything this mode must have active before it may command motion."""
    return tuple(SAFETY_STACK) + tuple(AUTONOMOUS_EXTRA.get(mode, ()))


def assert_safety_invariant(mode, active):
    """Raise unless every required mechanism is present in `active`.

    Called on EVERY transition. Raising is correct here: a mode that cannot
    prove its safety stack is up must not run at all, and returning False
    would leave the decision to a caller that might not check.
    """
    need = set(required_safety(mode))
    have = set(active or ())
    missing = sorted(need - have)
    if missing:
        raise RuntimeError(
            "mode %s (%d) REFUSED: safety mechanism(s) not active: %s. "
            "Every mode shares one safety stack and no mode may bypass it; "
            "mode 6 especially, where the clearance floor and the e-stop are "
            "the only things between a misdetection and the wearer."
            % (mode.name, mode.value, ", ".join(missing)))
    return True


def describe(mode):
    m = mode_from(mode)
    return ("%d %s | input: %s | robot decides: %s | max_vel %.2f rad/s | %s"
            % (m.value, m.name, INPUT_SOURCE[m],
               ", ".join(sorted(ROBOT_DECIDES[m])) or "nothing",
               MAX_VEL_RAD_S[m], IMPLEMENTED[m]))
