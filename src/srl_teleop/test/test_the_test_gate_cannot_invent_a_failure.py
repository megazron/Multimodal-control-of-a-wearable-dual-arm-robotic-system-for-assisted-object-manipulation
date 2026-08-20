"""The gate must not report tests that do not exist.

WHAT HAPPENED. `check_tests.py` finds failures with `^FAILED (\\S+)` over the
whole pytest output -- and the suite includes pep257 and flake8, which ECHO
THE SOURCE LINES THEY OBJECT TO. So a source file containing a line beginning
`FAILED ` at column zero puts that text into the output, and the gate reported

    REFUSING: 1 test(s) failing that are not on the allowlist:
      =

on a workspace where every real test passed. The offending line was
`FAILED = "failed"` in `vr_bringup.py`.

WHY IT MATTERS MORE THAN A MISSED FAILURE. This gate is what decides whether
the workspace is healthy. A gate that cries wolf is a gate somebody bypasses,
and then it is not a gate. The fix is to require a pytest NODE ID -- which
always carries `::` or ends `.py` -- rather than any word after `FAILED`.
"""
import os
import re

GATE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))), "scripts", "check_tests.py")


def _pattern():
    src = open(GATE).read()
    m = re.search(r'failed = set\(re\.findall\(r"([^"]+)", out, re\.M\)\)', src)
    assert m, "the failure pattern has moved -- this test cannot check it"
    return m.group(1)


def test_the_gate_ignores_source_lines_that_merely_start_with_FAILED():
    """The exact output that produced the phantom."""
    out = "\n".join([
        'FAILED = "failed"',
        "FAILED src/srl_teleop/test/test_flake8.py::test_flake8 - Assertion",
        "FAILED src/srl_teleop/test/test_pep257.py::test_pep257 - Assertion",
    ])
    hits = set(re.findall(_pattern(), out, re.M))
    assert "=" not in hits, (
        "a source line beginning 'FAILED ' is being counted as a failing test")
    assert hits == {"src/srl_teleop/test/test_flake8.py::test_flake8",
                    "src/srl_teleop/test/test_pep257.py::test_pep257"}


def test_the_gate_still_catches_a_real_failure():
    """The other half. A pattern tightened until it matches nothing would
    pass the test above and hide every failure in the workspace."""
    out = ("FAILED src/srl_perception/test/test_wearer_tracking.py::"
           "test_a_body_further_away_never_improves_clearance - Assertion")
    hits = set(re.findall(_pattern(), out, re.M))
    assert len(hits) == 1
    assert "test_a_body_further_away_never_improves_clearance" in hits.pop()


def test_a_bare_file_id_still_counts():
    """pytest prints a file-only id when a whole module fails to collect, and
    that is exactly the case the gate must not miss."""
    out = "FAILED src/srl_teleop/test/test_something.py - ImportError"
    hits = set(re.findall(_pattern(), out, re.M))
    assert hits == {"src/srl_teleop/test/test_something.py"}


def test_the_offending_line_is_still_in_the_workspace():
    """Keep this test honest: if `FAILED = "failed"` is ever renamed away, the
    regression it caused is no longer reachable and this file should be read
    as history rather than as a live guard."""
    vb = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(
        __file__))), "srl_teleop", "vr_bringup.py")
    src = open(vb).read()
    assert re.search(r"^FAILED = ", src, re.M), (
        "the line that triggered this is gone -- which is fine, but the gate "
        "must still be tightened, because the next file will do it again")
