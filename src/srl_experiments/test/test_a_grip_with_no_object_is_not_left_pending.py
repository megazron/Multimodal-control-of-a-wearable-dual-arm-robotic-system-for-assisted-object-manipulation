"""A commanded grip change with nothing to arrive at must be APPLIED, not held.

WHAT WAS WRONG, AND IT WAS THE WHOLE LIFE OF T2.

`run_abc` holds a commanded grip change until the finger pads reach the
object it was asked against:

    if pads is not None and obj is not None and dist(pads, obj) <= ARRIVE_TOL:
        held_grip[arm] = pend[arm]          # close, now
    else:
        late[arm] += 1                      # wait

That is right for a PICK -- closing early drives the hand shut in mid-air, and
the gate exists because an earlier version compared the arm against the wrong
frame and the hand never closed at all. It is unsatisfiable for a task that
declares NO object: `obj` is None on every tick, so the change stays pending
for the whole run.

T2 is that task. Its spec says "both grippers are already closed on the tray
and STAY closed: the task is the carry, not the grasp", it schedules
`grip_for(30)` on every waypoint, and it sets `grip_obj=None` because there is
nothing to arrive at. Measured on the 2026-08-23 re-record, all five modes:

    01_master_teleop    642 samples   0.000 .. 0.000   both arms
    02_vr_teleop       1219 samples   0.000 .. 0.000
    03_shared_autonomy  620 samples   0.000 .. 0.000
    04_vr_shared        666 samples   0.000 .. 0.000
    06_full_autonomy    697 samples   0.000 .. 0.000

CLAUDE.md records the consequence -- "T2-1 'held by BOTH grippers' is false
and has been for the life of the task" -- and attributes it to the elastic
tray hiding it. The tray was fixed on 2026-08-16 and the grippers still never
closed, because the cause is this gate.

WHAT IS CHECKED HERE. Source structure, because the loop needs a live stack
and a moving arm to exercise, and the property is about which branch exists:

  * a change with no object is applied on the spot, and says so;
  * the arrival gate is still there for tasks that DO declare an object --
    removing it would reintroduce the mid-air close it was written to stop;
  * `late[]` is not incremented for a grip that was never waiting;
  * T2 still declares no object, so this is the branch it takes.
"""
import ast
import inspect
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for _p in (os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc"),
           os.path.join(ROOT, "src", "srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SRC = open(os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc",
                        "run_abc.py")).read()


def test_a_change_with_no_object_is_applied_immediately():
    assert "if obj is None:" in SRC, (
        "the no-object branch is gone; a task that declares no grip object "
        "would hold its grip change pending for the whole run again")
    i = SRC.index("if obj is None:")
    block = SRC[i:i + 700]
    assert "held_grip[arm] = pend[arm]" in block
    assert "pend[arm] = None" in block


def test_it_says_so_rather_than_closing_silently():
    """A grip that closes without the usual arrival line would be invisible in
    the run log, which is where a clip that went wrong is read back from."""
    i = SRC.index("if obj is None:")
    block = SRC[i:i + 700]
    assert "[progress]" in block
    assert "no object declared" in block


def test_the_arrival_gate_still_exists_for_tasks_that_declare_an_object():
    """THE CONTROL. If this branch were removed the hand would close in
    mid-air on every pick, which is the failure the gate was written for."""
    assert "math.dist(pads, obj) <= ARRIVE_TOL_M" in SRC
    i = SRC.index("if obj is None:")
    assert SRC.index("math.dist(pads, obj) <= ARRIVE_TOL_M", i) > i, (
        "the arrival gate no longer follows the no-object branch")


def test_a_grip_that_was_never_waiting_is_not_counted_late():
    """`late[arm]` drives a report at the end of the run. Counting a grip
    that had nothing to wait for would make every T2 run look lagged."""
    i = SRC.index("if obj is None:")
    j = SRC.index("math.dist(pads, obj) <= ARRIVE_TOL_M", i)
    assert "late[arm] += 1" not in SRC[i:j], (
        "the no-object branch increments late[]; it was never waiting")


def test_T2_is_the_task_that_takes_this_branch():
    """And if T2 ever gains an object, this test says so rather than silently
    describing a task that no longer exists."""
    import msc_clip_tasks as MCT
    spec = MCT.TASKS["t2"]
    assert spec.get("grip_obj") is None, (
        "T2 now declares a grip object, so it takes the arrival gate and this "
        "file's premise has changed")
    n = 5
    sched = spec["grip"](n)
    assert sched["left"] == sched["right"], "both arms hold the same tray"
    assert len(set(sched["left"])) == 1, (
        "T2's schedule is meant to be CONSTANT -- already closed, stays "
        "closed -- which is why nothing ever changed and nothing ever fired")
    assert sched["left"][0] > 0.0, "and it is a CLOSED value, not open"


def test_the_first_tick_really_does_see_a_change():
    """`held_grip` starts at 0.0 and T2's schedule starts closed, so tick 0
    raises a pending change. If held_grip were seeded from the schedule there
    would be no change to apply and this fix would do nothing."""
    assert "held_grip = {arm: 0.0 for arm in ARMS}" in SRC
