#!/usr/bin/env python3
"""Launch buttons: every one must resolve, or be disabled WITH A REASON.

Five experiment buttons in an earlier build exited 2 the instant they were
pressed while appearing to launch. The defence is a shared manifest plus a
checker; these tests pin the checker's properties. The end-to-end proof --
actually running the dispatcher and actually pressing every button -- is
`scripts/verify_gui_buttons.py`.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from srl_teleop import gui_launch_specs as gls               # noqa: E402


def test_the_dispatcher_list_is_PARSED_not_duplicated():
    """A hand-copied list is how the labels and the dispatcher drifted apart.
    An EMPTY parse must not read as 'no tasks are valid'."""
    t = gls.dispatcher_tasks()
    assert t, "parsed zero tasks -- the parser, not the dispatcher, is broken"
    assert {"a", "b", "c"} <= t


def test_a_REFUSING_branch_is_not_read_as_an_accepted_task():
    """The trap this parser exists to avoid, in its current form.

    run_experiment.sh carries one `t1|t2|...|e6)` branch that echoes to stderr
    and exits 2 -- the archived task sets, kept refusable so an old command
    line gets a named refusal instead of a confusing failure. A parser that
    reads a case label as an accepted task turns that single refusal into
    fifteen offered buttons, every one of which exits 2 the moment it is
    pressed. That is precisely the defect this whole manifest was built to
    stop, so the archived tasks must be absent, not merely disabled.
    """
    t = gls.dispatcher_tasks()
    for k in ("t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8", "t9",
              "e1", "e2", "e3", "e4", "e5", "e6"):
        assert k not in t, (
            "%s comes from a branch that exits 2 -- offering it is the bug"
            % k)


def test_every_spec_resolves_or_states_why_not():
    for spec, ok, detail in gls.validate():
        assert ok or spec.disabled_reason, "%s disabled with no reason" % spec.key
        assert detail, "%s has no detail" % spec.key


def test_a_disabled_spec_is_never_enabled():
    for spec, ok, _ in gls.validate():
        assert spec.enabled == ok


def test_only_the_two_CURRENT_task_sets_are_offered():
    """One generation per set, one button per (task, mode), and NO archived
    generation on the screen beside them.

    This test used to assert A/B/C were the only set offered. That was right
    when there was one; the MSc four (m0-m3, displayed T0-T3) are now a second
    CURRENT set and are deliberately present. What the test is actually for is
    unchanged and is still enforced: t1-t9 ran a 300/310 mm span and e1-e6 are
    the superseded E-series, and offering either beside a 500 mm spec is how a
    result gets filed under the wrong geometry.

    Note the MSc set is keyed m0-m3 precisely BECAUSE t0-t3 are spoken for:
    run_experiment.sh refuses t1..t9 by name, and a dispatcher that took `t3`
    would have to choose between two different T3s.
    """
    specs = gls.all_specs()
    abc = [s for s in specs if s.key.startswith("abc_")]
    msc = [s for s in specs if s.key.startswith("msc_")]
    assert len(abc) == 15, "3 tasks x 5 modes"
    # 5 LAUNCHABLE CELLS, not 4 tasks. T1 has two STAGES: stage 1 is one arm
    # at a time, stage 2 is both arms at once from random positions, and they
    # are separate dispatcher keys (m1, m1s2) with separate session cells
    # because stage 2 is the SIMULTANEITY condition -- the same job twice at
    # once -- and a participant must do stage 1 first or they are learning the
    # task and the simultaneity together. This constant was 20 and the code
    # was right; a stale count in a test that exists to catch drift is the
    # thing it is supposed to catch.
    assert len(msc) == 25, "5 launchable cells (T1 has two stages) x 5 modes"
    # ENABLED UNLESS THE TASK ITSELF FORBIDS THAT MODE.
    #
    # This was a flat "every one of them is enabled", and it was wrong in the
    # direction that costs a run: T1 declares `modes=("06_full_autonomy",)`
    # and `run_abc` REFUSES it under anything else, so four of stage 1's five
    # buttons could only ever exit 1 -- and stage 2, which sends the same
    # non-anchor approach, had no restriction at all, so its four would have
    # SUCCEEDED and logged T1's geometry under a teleop mode's name.
    #
    # The lock is read from the task spec, not listed here, so a task that
    # gains or loses one cannot leave a stale copy in this test.
    locks = gls.task_mode_locks()
    assert locks, "no task declares a mode restriction -- the lookup broke"
    for s in abc + msc:
        # READ OFF THE ARGV, not parsed out of the key. The key is
        # "msc_m1s2_01_master_teleop" and every scheme for cutting a task out
        # of that is a guess about where the boundaries are; the command line
        # says `run_experiment.sh <task> --mode <mode>` and cannot be
        # ambiguous.
        key = s.argv[1]
        mode = s.argv[s.argv.index("--mode") + 1]
        if key in locks and mode not in locks[key]:
            assert not s.enabled, (
                "%s is offered but %s runs under %s only"
                % (s.key, key, locks[key]))
            assert "runs under" in (s.disabled_reason or ""), s.key
            continue
        assert s.enabled, "%s disabled: %s" % (s.key, s.disabled_reason)

    # AND THE LOCK MUST LEAVE SOMETHING LAUNCHABLE. A restriction naming a
    # mode that has no button would disable every button for that task and
    # read as "the task is gone" rather than "the task is mode-locked".
    for key, modes in locks.items():
        live = [s for s in msc
                if s.key.startswith("msc_%s_" % key) and s.enabled]
        assert live, "%s is locked to %s and has no enabled button" % (
            key, modes)

    tasks = [s for s in specs if s.group == "task"]
    assert len(tasks) == len(abc) + len(msc), (
        "a task button outside the two current sets is being offered: %s"
        % [s.key for s in tasks
           if not s.key.startswith(("abc_", "msc_"))])

    # The archived generations must not be reachable by ANY button.
    import re
    for s in tasks:
        arg = " ".join(s.argv)
        assert not re.search(r"run_experiment\.sh\s+(t[1-9]|e[1-6])\b", arg), (
            "%s dispatches an ARCHIVED task set: %s" % (s.key, arg))


def test_task_keys_and_labels_are_unique():
    """Two buttons that dispatch as one.

    The mode key used to be cut from the mode name's first two characters, so
    03_shared_autonomy and the legacy alias 04_shared_autonomy -- same topic,
    same command path -- produced two differently-labelled buttons sharing one
    key, and one of each pair was unreachable. Keys now carry the whole mode
    name. This is the assertion, not the comment, because the failure was
    invisible on screen: six buttons for five modes reads as a complete list.
    """
    specs = gls.all_specs()
    keys = [s.key for s in specs]
    labels = [s.label for s in specs]
    assert len(set(keys)) == len(keys), \
        sorted({k for k in keys if keys.count(k) > 1})
    assert len(set(labels)) == len(labels), \
        sorted({v for v in labels if labels.count(v) > 1})


def test_stack_starting_specs_are_marked_as_such():
    """`starts_stack` scopes the second-stack refusal. Marking an add-on as
    stack-starting makes the refusal fire when you most need the add-on."""
    by = {s.key: s for s in gls.all_specs()}
    assert by["sim"].starts_stack and by["autonomy"].starts_stack
    for k in ("diag", "blocking", "preflight",
              "abc_a_01_master_teleop", "abc_c_06_full_autonomy"):
        assert not by[k].starts_stack


def test_a_missing_script_disables_its_button(tmp_path, monkeypatch):
    """The negative control: validate() must be able to FAIL."""
    s = gls.Spec("x", "Ghost", "diag", [str(tmp_path / "nope.sh")])
    rows = gls.validate([s])
    assert rows[0][1] is False
    assert "missing script" in s.disabled_reason
