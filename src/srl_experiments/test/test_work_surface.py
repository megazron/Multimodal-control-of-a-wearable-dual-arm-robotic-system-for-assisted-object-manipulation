"""The work surface height has ONE owner and a 20 mm error must be NAMED."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "srl_experiments"))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from srl_experiments import work_surface as WS  # noqa: E402


def setup_function(_):
    WS.clear_measured()


def test_declared_until_something_measures_it():
    assert WS.source() == "declared"
    assert WS.table_top() == WS.DECLARED_M
    assert WS.delta_m() is None


def test_measured_wins_and_is_labelled_as_such():
    WS.set_measured(0.961, "mock depth")
    assert WS.source() == "measured"
    assert abs(WS.table_top() - 0.961) < 1e-9


def test_twenty_mm_either_way_FAILS_and_names_the_number():
    for d in (0.020, -0.020):
        WS.set_measured(WS.DECLARED_M + d)
        ok, msg = WS.check()
        assert not ok, "a %+.0f mm surface error passed silently" % (d * 1000)
        # The number must be IN the message. "surface mismatch" sends nobody
        # anywhere; "+20.0 mm" tells them what to change.
        assert "%+.1f mm" % (d * 1000) in msg
        assert "40 mm" in msg          # says why it matters: half a cube


def test_small_error_passes_but_still_reports_the_number():
    WS.set_measured(WS.DECLARED_M + 0.004)
    ok, msg = WS.check()
    assert ok
    assert "+4.0 mm" in msg


def test_unmeasured_is_a_FAILURE_when_measurement_is_required():
    ok, msg = WS.check(require_measured=True)
    assert not ok
    assert "NOT MEASURED" in msg


def test_the_check_can_fail_on_a_deliberately_broken_input():
    """A check that cannot fail is not a check."""
    WS.set_measured(WS.DECLARED_M + 0.5)
    ok, _ = WS.check()
    assert not ok


def test_the_scene_reads_the_owner_rather_than_its_own_literal():
    """clip_scene must not carry a second copy of the height."""
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    src = open(os.path.join(root, "scripts", "clip_scene.py")).read()
    # SCAN THE WHOLE MODULE, not a slice "before the first def". The first
    # version cut at the first `def` and passed only as long as no function
    # was ever defined above the table-height block. One was, and the test
    # broke without the property it guards having changed at all -- the
    # instrument was the fault, not the code.
    assert "work_surface" in src, (
        "clip_scene must READ work_surface.table_top(), not define its own "
        "table height")
    i = src.find("TABLE_TOP = 0.95")
    if i != -1:
        # A fallback literal is allowed, but only where it WARNS. A silent
        # fallback is how a measured surface reaches nothing.
        assert "warn" in src[max(0, i - 600):i], (
            "the fallback table height is silent; it must warn")
