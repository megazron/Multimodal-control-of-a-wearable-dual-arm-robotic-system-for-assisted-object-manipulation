"""A clearance guard with no data must REFUSE, not report infinity.

Measured 2026-08-21: a 176 deg sweep of the left arm ran to completion with
`0 of 9 parts checked` and `clear inf m` on every progress line. The wearer
was declared PRESENT; not one of its transforms resolved; and the guard
returned the single value that can never trip a floor comparison.

The honest answer to "how close is the arm to the person" was "I cannot see
the person". Reporting that as infinitely clear points the failure in the
most dangerous direction available.
"""
import pathlib

SRC = (pathlib.Path(__file__).resolve().parents[1] / "srl_teleop"
       / "real_homing_node.py").read_text()


def test_no_transforms_does_not_return_infinity():
    i = SRC.index("if not pts:")
    window = SRC[i:i + 1400]
    body = window.split("return", 1)[1][:80]
    assert 'float("inf")' not in body, (
        "no data still reports infinite clearance, which always passes")


def test_no_transforms_returns_something_that_cannot_pass_a_floor():
    i = SRC.index("if not pts:")
    window = SRC[i:i + 1400]
    assert 'float("nan")' in window, (
        "the no-data value must fail every comparison, not pass every one")


def test_the_execute_loop_checks_for_the_no_data_case_and_halts():
    assert "clear != clear" in SRC, "NaN is never detected, so it is ignored"
    i = SRC.index("clear != clear")
    window = SRC[i:i + 700]
    assert "HOMING HALTED" in window
    assert "self.aborted = True" in window, "it detects it and keeps going"


def test_the_halt_message_tells_the_operator_how_to_proceed():
    i = SRC.index("clear != clear")
    window = SRC[i:i + 700]
    assert "SRL_WEARER_PRESENT" in window, (
        "a refusal with no way forward gets bypassed by whatever is at hand")


def test_the_no_data_check_runs_BEFORE_the_floor_comparison():
    """Order matters: NaN < floor is False, so a floor check placed first
    would let the unmeasured case through silently."""
    assert SRC.index("clear != clear") < SRC.index("if clear < self.min_clear:")
