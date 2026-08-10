#!/usr/bin/env python3
"""The gripper's three states, and the ONE piece of state that must survive a
process death.

WHY THIS FILE EXISTS
--------------------
The Robotiq 2F-85 holds its position when the commanding process goes away. So
a node killed mid-grasp leaves the fingers part-closed, and the NEXT session
starts with the hand in an unknown place -- neither open nor holding, with
nothing in the software able to tell which.

That is the same failure this project has met four times in other clothing:
the frozen `/real/joint_states` that read as "holding", the dead j7 pot that
read as "steady", the oversampled recorder whose duplicate rows read as a
0.000 noise floor. A value that looks like a measurement but is really a
leftover.

THE THREE STATES, and why the middle one is the whole problem
------------------------------------------------------------
    open      < 0.10 rad     fingers apart
    holding   0.10..0.74     closed on SOMETHING -- the object is the stop
    free_air  >= 0.74 rad    closed on NOTHING -- travelled to its own limit

`free_air` earns its keep twice over. It is how the experiment tells a knocked
object from a fumbled grasp, and it is how startup tells "I am gripping the
participant's block" from "I am just shut".

THE MARKER IS WRITTEN WHEN THE GRIP IS TAKEN, NOT WHEN THE PROCESS EXITS
-----------------------------------------------------------------------
This is the design decision that makes the whole thing work, and it is
counter-intuitive enough to state plainly. `kill -9` runs no shutdown code. An
`atexit` hook, a signal handler and a `finally:` block are all equally useless
against it. So the evidence that the hand may be holding something cannot be
laid down at shutdown -- it has to already be on disk before the kill lands.

The latch marker is therefore created at LATCH time and removed at RELEASE
time. It is a claim about the world ("I closed on something and have not let
go"), not a record of how the process ended.

Recovering a latch needs BOTH halves and neither alone is sufficient:

    marker present + knuckle in `holding`   -> still holding. DO NOT OPEN.
    marker present + knuckle `open`         -> someone opened it by hand
    marker present + knuckle `free_air`     -> the object is GONE; the marker
                                               is stale and the hand is safe
                                               to reset
    no marker                               -> free to open and confirm

The knuckle alone cannot do it (a hand closed on nothing looks closed) and the
marker alone cannot do it (it outlives the object). Requiring both is what
stops a stale file from freezing a perfectly empty gripper forever.
"""
import os
import time

# Robotiq 2F-85 driven knuckle, radians. THIS IS THE SINGLE DEFINITION;
# `srl_experiments` imports it rather than restating it. Two copies of a
# threshold drift, and a drifted grasp threshold silently reclassifies every
# trial in a study.
OPEN_RAD = 0.10
FREE_AIR_RAD = 0.74
CLOSED_RAD = 0.80          # commanded full close; the mechanical limit

OPEN, HOLDING, FREE_AIR, UNKNOWN = "open", "holding", "free_air", "unknown"

# THE COMMAND TO OPEN, which is NOT the threshold that classifies "open".
# Kept as a separate name because conflating them is how the copies drifted:
# vr_gripper_node called 0.0 "OPEN_RAD" and record_rviz called 0.05
# "GRIP_OPEN", while OPEN_RAD here is 0.10 and decides what counts as open.
# A command and a threshold that share a name will be substituted for one
# another eventually.
CMD_OPEN_RAD = 0.0

# The knuckle's own mechanical stop, reached only when the hand closes on
# NOTHING. Above FREE_AIR_RAD by construction -- asserted below, because the
# gap between them is the entire margin that makes "closed on nothing"
# distinguishable from "closed on something".
MECH_LIMIT_RAD = 0.80
assert CMD_OPEN_RAD < OPEN_RAD < FREE_AIR_RAD <= MECH_LIMIT_RAD


def grip_for(width_mm):
    """Knuckle angle that closes on an object of this width.

    Robotiq 2F-85: 85 mm stroke across roughly 0 -> 0.8 rad of driven knuckle,
    so closing on a w-mm object leaves the knuckle short of full closure. This
    is THE definition -- `record_rviz`, `clip_tasks` and the clip scene all
    import it. It existed as three separate copies, and a drifted grasp
    threshold silently reclassifies every trial in a study.
    """
    return max(0.12, min(0.70, 0.8 * (1.0 - float(width_mm) / 85.0)))


# Fraction of grip_for() at which the fingers are accepted as being ON the
# object. Absorbs the mock's first-order tracking lag without accepting a
# gripper that has barely moved.
GRIP_REACHED_FRAC = 0.90


def holding(knuckle, width_mm=None):
    """True only when the fingers are closed ON SOMETHING.

    With `width_mm` given, the test is that the fingers have actually reached
    the object's own width rather than merely left the open position. Without
    it the band alone attached a marker the instant the knuckle passed
    OPEN_RAD, so a 40 mm block (which needs 0.42) jumped to the gripper while
    the fingers were still visibly open -- the object appeared grasped before
    it was touched.
    """
    if knuckle is None or knuckle != knuckle:
        return False
    if width_mm is not None:
        # BOTH ENDS. The lower bound says the fingers reached the object's
        # width; the UPPER bound says they did not sail past it and close on
        # air. This branch had only the lower bound, and that is a real bug
        # with a measured consequence: `mock_components` boots the knuckle at
        # 0.7929 rad, so at scene start every object in the set read as
        # GRASPED -- 0.7929 clears the 40 mm block's 0.3812 threshold easily.
        # Task A's first clip of the 2026-08-10 re-record logged GRASPED at
        # t=0.0 with the arm still at home, then a RELEASED that carried
        # 0.000 m, and the real grasp 56 s later was reported as "51.3 s
        # outside the clip". The no-width branch below always had both bounds;
        # only this one lost the upper. CLAUDE.md records the same failure
        # from the original pilot ("the mock gripper boots at 0.79 rad --
        # already past any closed threshold"), so this is a regression of a
        # known bug, not a new one.
        #
        # Safe for every object in the set: the narrowest (20 mm) needs
        # 0.5506, well under FREE_AIR_RAD.
        return (GRIP_REACHED_FRAC * grip_for(width_mm) <= knuckle
                < FREE_AIR_RAD)
    return OPEN_RAD <= knuckle < FREE_AIR_RAD

# One marker per arm, in the user's runtime dir so it does not survive a
# reboot -- a grip cannot outlive the machine, and a marker that does would be
# a latch nobody can clear.
MARKER_DIR = os.environ.get("XDG_RUNTIME_DIR") or "/tmp"


def classify(knuckle_rad):
    """rad -> open | holding | free_air | unknown. Never raises."""
    try:
        v = float(knuckle_rad)
    except (TypeError, ValueError):
        return UNKNOWN
    if v != v or v in (float("inf"), float("-inf")):   # NaN / inf
        return UNKNOWN
    if v < OPEN_RAD:
        return OPEN
    if v >= FREE_AIR_RAD:
        return FREE_AIR
    return HOLDING


def marker_path(arm):
    return os.path.join(MARKER_DIR, "srl_gripper_latched_%s" % arm)


def set_latched(arm, frac=None, note=""):
    """Record that this arm has CLOSED ON SOMETHING. Called at latch time."""
    try:
        with open(marker_path(arm), "w") as fh:
            fh.write("%f %s %s\n" % (time.time(),
                                     "-" if frac is None else "%.3f" % frac,
                                     note))
        return True
    except OSError:
        return False


def clear_latched(arm):
    """Record that this arm has LET GO. Called at release time."""
    try:
        os.remove(marker_path(arm))
    except FileNotFoundError:
        pass
    except OSError:
        return False
    return True


def was_latched(arm):
    """Did a previous run of anything take a grip on this arm and not let go?"""
    return os.path.exists(marker_path(arm))


def marker_age_s(arm):
    try:
        return max(0.0, time.time() - os.path.getmtime(marker_path(arm)))
    except OSError:
        return None


def startup_decision(arm, knuckle_rad):
    """What to do with this gripper at startup. (action, reason).

    action is one of:
        "open"    command a full open, then CONFIRM it from /joint_states
        "hold"    leave it alone -- it is holding something
        "unknown" no joint feedback; refuse to guess, say so, do not command

    THE DEFAULT IS NOT "OPEN". An unconditional open at startup is exactly how
    a latched grip drops a participant's object on the floor between trials.
    """
    st = classify(knuckle_rad)
    latched = was_latched(arm)

    if st == UNKNOWN:
        return ("unknown",
                "no knuckle feedback -- cannot tell open from holding, so "
                "commanding nothing. Check /joint_states.")
    if latched and st == HOLDING:
        return ("hold",
                "a previous run latched on an object and the knuckle is still "
                "in the holding band (%.3f rad). Opening now would drop it. "
                "Release deliberately, or clear %s."
                % (knuckle_rad, marker_path(arm)))
    if latched and st == FREE_AIR:
        return ("open",
                "stale latch marker: the knuckle is at %.3f rad, which is "
                "closed on NOTHING -- the object is already gone."
                % knuckle_rad)
    if latched and st == OPEN:
        return ("open",
                "stale latch marker: the knuckle is at %.3f rad, already open."
                % knuckle_rad)
    if st == OPEN:
        return ("open", "already open (%.3f rad); commanding a full open "
                        "anyway so the reference is asserted, not assumed."
                        % knuckle_rad)
    return ("open",
            "no latch marker, but the knuckle is at %.3f rad (%s) -- left "
            "part-closed by a previous run. Opening." % (knuckle_rad, st))
