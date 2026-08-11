#!/usr/bin/env python3
"""THE TASK LAYER. Seven verbs, and it does not know which mode drives it.

    REACH_TARGET  PICK_OBJECT  PLACE_OBJECT  GRASP  RELEASE  TRANSFER  RETURN

WHY THIS FILE EXISTS AT ALL, AND WHY IT IS BUILT FIRST
------------------------------------------------------
The independent variable is the CONTROL MODE.  For "shared autonomy beat
direct teleoperation" to mean anything, the two must have performed the SAME
TASK -- not a similar one.  If the task layer branches on mode, then each mode
runs a subtly different procedure and every comparison in the study is between
two things that were never the same.  So this module is written before any
task, and `test_mode_independence.py` proves the property before anything is
built on top.

THE INVARIANT, STATED SO IT CAN BE TESTED
-----------------------------------------
Running the same TaskCommand under any mode produces an IDENTICAL task-layer
trace signature: the same verbs, the same phases, in the same order, with the
same subjects and the same terminal status.  What differs between modes is
(a) the VALUES -- timings, achieved poses, errors -- and (b) the ADAPTER's own
internal steps, which are recorded separately and are expected to differ.

Two things follow, and both are enforced rather than hoped for:

  * nothing in this file may name a mode.  `test_no_mode_names_in_task_layer`
    greps this module's source for every mode name and fails on a hit.  A
    comparison written as `if mode == VR:` is the failure this whole design
    exists to prevent, and it is the kind that reads as harmless.
  * the task layer calls `await_operator()` at the same point in EVERY verb.
    Whether that blocks for a spoken confirmation, waits on a button, or
    returns immediately because the human is already driving the arm, is the
    ADAPTER's business.  Hoisting that decision up here would put the mode
    back into the task layer through the side door.

WHAT A "PHASE" IS FOR
---------------------
Phases are the unit the trial logger samples against, and they are what make
a completion time decomposable -- approach, descend, grasp, lift.  They are
part of the signature precisely so that a mode cannot quietly skip one: an
adapter that teleports to the object would produce a trace missing DESCEND,
and the proof would catch it.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional, Sequence


class Verb(str, Enum):
    REACH_TARGET = "REACH_TARGET"
    PICK_OBJECT = "PICK_OBJECT"
    PLACE_OBJECT = "PLACE_OBJECT"
    GRASP = "GRASP"
    RELEASE = "RELEASE"
    TRANSFER = "TRANSFER"
    RETURN = "RETURN"


class Phase(str, Enum):
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    CLOSE = "CLOSE"
    OPEN = "OPEN"
    LIFT = "LIFT"
    CARRY = "CARRY"
    SETTLE = "SETTLE"
    RETREAT = "RETREAT"
    VERIFY = "VERIFY"
    AWAIT_OPERATOR = "AWAIT_OPERATOR"


class Status(str, Enum):
    OK = "OK"
    UNREACHABLE = "UNREACHABLE"
    GRIP_FAILED = "GRIP_FAILED"
    REFUSED = "REFUSED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"


# Standoff above a grasp, and the height a lift clears to.  Task-level
# geometry, identical in every mode -- which is the point.
STANDOFF_M = 0.10
LIFT_M = 0.08
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.60          # inside the measured jaw map, not full closure


@dataclass(frozen=True)
class TaskCommand:
    """One instruction from the trial script.  Mode-free by construction:
    there is nowhere in here to put one."""
    verb: Verb
    arm: str                              # "left" | "right" | "both"
    subject: str = ""                     # target id, object id, or ""
    pose: Optional[Sequence[float]] = None
    to_pose: Optional[Sequence[float]] = None
    params: dict = field(default_factory=dict)


@dataclass
class TaskEvent:
    seq: int
    verb: str
    arm: str
    subject: str
    phase: str
    status: str
    t: float = 0.0
    detail: dict = field(default_factory=dict)

    def signature(self):
        """The MODE-INVARIANT part.  Deliberately excludes time, achieved
        pose and error -- those are the measurements, and requiring them to
        match across modes would be requiring the modes to perform
        identically, which is the hypothesis, not the invariant."""
        return (self.verb, self.arm, self.subject, self.phase, self.status)


@dataclass
class TaskResult:
    status: Status
    events: list
    detail: dict = field(default_factory=dict)

    def signature(self):
        return tuple(e.signature() for e in self.events)


class Adapter:
    """What the task layer is allowed to ask of a control mode.

    Four methods and nothing else.  Every mode difference -- which topic the
    pose goes to, whether autonomy supplies the wrist, whether a human must
    say yes -- lives behind these.
    """

    name = "abstract"

    def command_pose(self, arm, pose, phase):        # -> achieved pose | None
        raise NotImplementedError

    def command_gripper(self, arm, opening, phase):  # -> achieved | None
        raise NotImplementedError

    def await_operator(self, intent):                # -> bool
        raise NotImplementedError

    def log(self):                                   # -> list of adapter steps
        return []


class TaskLayer:
    """Executes TaskCommands against an Adapter.  Knows no modes."""

    def __init__(self, adapter: Adapter, clock: Callable[[], float] = None,
                 timeout_s: float = 120.0):
        self.a = adapter
        self.clock = clock or time.monotonic
        self.timeout_s = timeout_s
        self._seq = 0

    # ---------------------------------------------------------------- utils
    def _ev(self, out, cmd, phase, status, **detail):
        self._seq += 1
        out.append(TaskEvent(self._seq, cmd.verb.value, cmd.arm, cmd.subject,
                             phase.value, status.value, self.clock(), detail))

    def _move(self, out, cmd, arm, pose, phase):
        got = self.a.command_pose(arm, pose, phase.value)
        ok = got is not None
        self._ev(out, cmd, phase, Status.OK if ok else Status.UNREACHABLE,
                 commanded=list(pose), achieved=(list(got) if ok else None))
        return got

    def _grip(self, out, cmd, arm, opening, phase):
        got = self.a.command_gripper(arm, opening, phase.value)
        ok = got is not None
        self._ev(out, cmd, phase, Status.OK if ok else Status.GRIP_FAILED,
                 commanded=opening, achieved=got)
        return got

    def _confirm(self, out, cmd, intent):
        """The one place a mode may insert a human decision.  It is called in
        EVERY verb and in every mode, so the trace shape does not change --
        only how long it takes and what the adapter did inside."""
        ok = bool(self.a.await_operator(intent))
        self._ev(out, cmd, Phase.AWAIT_OPERATOR,
                 Status.OK if ok else Status.REFUSED, intent=intent)
        return ok

    @staticmethod
    def _above(pose, dz):
        return [pose[0], pose[1], pose[2] + dz]

    # ---------------------------------------------------------------- verbs
    def execute(self, cmd: TaskCommand) -> TaskResult:
        fn = {
            Verb.REACH_TARGET: self._reach,
            Verb.PICK_OBJECT: self._pick,
            Verb.PLACE_OBJECT: self._place,
            Verb.GRASP: self._grasp,
            Verb.RELEASE: self._release,
            Verb.TRANSFER: self._transfer,
            Verb.RETURN: self._return,
        }[cmd.verb]
        out = []
        t0 = self.clock()
        status = fn(out, cmd)
        return TaskResult(status, out,
                          {"duration_s": self.clock() - t0,
                           "adapter": self.a.name})

    def _reach(self, out, cmd):
        if not self._confirm(out, cmd, "reach %s" % cmd.subject):
            return Status.REFUSED
        if self._move(out, cmd, cmd.arm, cmd.pose, Phase.APPROACH) is None:
            return Status.UNREACHABLE
        if self._move(out, cmd, cmd.arm, cmd.pose, Phase.SETTLE) is None:
            return Status.UNREACHABLE
        self._ev(out, cmd, Phase.VERIFY, Status.OK)
        return Status.OK

    def _pick(self, out, cmd):
        if not self._confirm(out, cmd, "pick %s" % cmd.subject):
            return Status.REFUSED
        pre = self._above(cmd.pose, STANDOFF_M)
        if self._move(out, cmd, cmd.arm, pre, Phase.APPROACH) is None:
            return Status.UNREACHABLE
        if self._grip(out, cmd, cmd.arm, GRIPPER_OPEN, Phase.OPEN) is None:
            return Status.GRIP_FAILED
        if self._move(out, cmd, cmd.arm, cmd.pose, Phase.DESCEND) is None:
            return Status.UNREACHABLE
        if self._grip(out, cmd, cmd.arm, GRIPPER_CLOSED, Phase.CLOSE) is None:
            return Status.GRIP_FAILED
        if self._move(out, cmd, cmd.arm, self._above(cmd.pose, LIFT_M),
                      Phase.LIFT) is None:
            return Status.UNREACHABLE
        return Status.OK

    def _place(self, out, cmd):
        dest = cmd.to_pose if cmd.to_pose is not None else cmd.pose
        if not self._confirm(out, cmd, "place %s" % cmd.subject):
            return Status.REFUSED
        if self._move(out, cmd, cmd.arm, self._above(dest, STANDOFF_M),
                      Phase.APPROACH) is None:
            return Status.UNREACHABLE
        if self._move(out, cmd, cmd.arm, dest, Phase.DESCEND) is None:
            return Status.UNREACHABLE
        if self._grip(out, cmd, cmd.arm, GRIPPER_OPEN, Phase.OPEN) is None:
            return Status.GRIP_FAILED
        if self._move(out, cmd, cmd.arm, self._above(dest, STANDOFF_M),
                      Phase.RETREAT) is None:
            return Status.UNREACHABLE
        return Status.OK

    def _grasp(self, out, cmd):
        if not self._confirm(out, cmd, "grasp %s" % cmd.subject):
            return Status.REFUSED
        got = self._grip(out, cmd, cmd.arm, GRIPPER_CLOSED, Phase.CLOSE)
        return Status.OK if got is not None else Status.GRIP_FAILED

    def _release(self, out, cmd):
        if not self._confirm(out, cmd, "release %s" % cmd.subject):
            return Status.REFUSED
        got = self._grip(out, cmd, cmd.arm, GRIPPER_OPEN, Phase.OPEN)
        return Status.OK if got is not None else Status.GRIP_FAILED

    def _transfer(self, out, cmd):
        """BIMANUAL, and the coupling is in the CALL not in the adapter: both
        grips are commanded for the same waypoint before either advances.  An
        adapter that moves one arm at a time would still produce this trace,
        which is why T2 measures tilt CONTINUOUSLY rather than trusting the
        verb."""
        if not self._confirm(out, cmd, "transfer %s" % cmd.subject):
            return Status.REFUSED
        sep = float(cmd.params.get("grip_sep_m", 0.0))
        way = cmd.params.get("waypoints") or [cmd.pose, cmd.to_pose]
        for i, p in enumerate(way):
            phase = Phase.CARRY if 0 < i < len(way) - 1 else (
                Phase.APPROACH if i == 0 else Phase.SETTLE)
            for arm, s in (("left", +0.5), ("right", -0.5)):
                q = [p[0] + s * sep, p[1], p[2]]
                if self.a.command_pose(arm, q, phase.value) is None:
                    self._ev(out, cmd, phase, Status.UNREACHABLE, arm=arm,
                             commanded=q)
                    return Status.UNREACHABLE
            self._ev(out, cmd, phase, Status.OK, waypoint=i,
                     commanded=list(p))
        return Status.OK

    def _return(self, out, cmd):
        """PLACE back at the recorded origin, then clear.  A separate verb
        from PLACE_OBJECT because T3 scores the return leg on its own and
        because the destination is not chosen by the participant."""
        if not self._confirm(out, cmd, "return %s" % cmd.subject):
            return Status.REFUSED
        home = cmd.to_pose if cmd.to_pose is not None else cmd.pose
        if self._move(out, cmd, cmd.arm, self._above(home, STANDOFF_M),
                      Phase.APPROACH) is None:
            return Status.UNREACHABLE
        if self._move(out, cmd, cmd.arm, home, Phase.DESCEND) is None:
            return Status.UNREACHABLE
        if self._grip(out, cmd, cmd.arm, GRIPPER_OPEN, Phase.OPEN) is None:
            return Status.GRIP_FAILED
        if self._move(out, cmd, cmd.arm, self._above(home, STANDOFF_M),
                      Phase.RETREAT) is None:
            return Status.UNREACHABLE
        return Status.OK


def run_script(adapter: Adapter, commands, clock=None):
    """Execute a list of TaskCommands, returning (results, joint signature)."""
    tl = TaskLayer(adapter, clock=clock)
    results = [tl.execute(c) for c in commands]
    sig = tuple(s for r in results for s in r.signature())
    return results, sig
