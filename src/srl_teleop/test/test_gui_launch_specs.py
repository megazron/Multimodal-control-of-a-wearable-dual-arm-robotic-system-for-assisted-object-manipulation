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


def test_tasks_A_B_C_are_the_only_task_set_offered():
    """One generation of task set, one button per (task, mode).

    A/B/C were disabled while they had a verified spec and no runner; run_abc
    now reads experiments/abc/tasks.py and the dispatcher routes a|b|c there
    and nowhere else, so they are live. What must NOT come back is the second
    and third generation alongside them: t1-t9 ran a 300/310 mm span and e1-e6
    are the superseded E-series, and offering either next to a 500 mm spec is
    how a result gets filed under the wrong geometry.
    """
    abc = [s for s in gls.all_specs() if s.key.startswith("abc_")]
    assert len(abc) == 15, "3 tasks x 5 modes"
    for s in abc:
        assert s.enabled, "%s disabled: %s" % (s.key, s.disabled_reason)
    tasks = [s for s in gls.all_specs() if s.group == "task"]
    assert len(tasks) == len(abc), "a non-A/B/C task button is being offered"


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
