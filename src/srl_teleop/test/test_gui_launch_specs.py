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
    assert {"t3", "t7", "e1"} <= t


def test_tasks_the_dispatcher_REFUSES_are_not_offered():
    """t1 is subsumed by t7 and t4 is geometrically blocked; both branches
    exist in the script and exit 2. Offering them would be the original bug."""
    t = gls.dispatcher_tasks()
    assert "t1" not in t
    assert "t4" not in t


def test_every_spec_resolves_or_states_why_not():
    for spec, ok, detail in gls.validate():
        assert ok or spec.disabled_reason, "%s disabled with no reason" % spec.key
        assert detail, "%s has no detail" % spec.key


def test_a_disabled_spec_is_never_enabled():
    for spec, ok, _ in gls.validate():
        assert spec.enabled == ok


def test_tasks_A_B_C_are_disabled_and_the_reason_names_the_trap():
    """They are specified and verified but have no runner. Wiring them to
    t3/t6/t7 would run the superseded 300/310 mm span under a 500 mm name --
    a button that launches something DIFFERENT from what it claims is worse
    than one that refuses."""
    abc = [s for s in gls.all_specs() if s.key.startswith("abc_")]
    assert len(abc) == 3
    for s in abc:
        assert not s.enabled
        assert "300/310" in s.disabled_reason


def test_stack_starting_specs_are_marked_as_such():
    """`starts_stack` scopes the second-stack refusal. Marking an add-on as
    stack-starting makes the refusal fire when you most need the add-on."""
    by = {s.key: s for s in gls.all_specs()}
    assert by["sim"].starts_stack and by["autonomy"].starts_stack
    for k in ("diag", "blocking", "preflight", "t3", "e1"):
        assert not by[k].starts_stack


def test_a_missing_script_disables_its_button(tmp_path, monkeypatch):
    """The negative control: validate() must be able to FAIL."""
    s = gls.Spec("x", "Ghost", "diag", [str(tmp_path / "nope.sh")])
    rows = gls.validate([s])
    assert rows[0][1] is False
    assert "missing script" in s.disabled_reason
