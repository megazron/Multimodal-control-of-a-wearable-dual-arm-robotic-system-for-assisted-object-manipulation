"""The operations window must fit the screen it is opened on.

WHY THIS TEST EXISTS. The window size was hard-coded at 1920x1060. On a
1920-wide display that puts its right edge six pixels past the glass, and
the consequence was not a cosmetic clip: WSLg never composited the window
onto the desktop at all. The operator got a taskbar icon that did nothing
when clicked, while the application behind it was rendering every panel
correctly at 3.0 ms/frame. `x11grab` refused the same window with BadMatch,
which is what X says about a window that is not wholly on-screen.

The failure looked exactly like "the GUI is broken" and was nothing of the
sort, which is why the arithmetic is pinned here rather than left to a
screenshot somebody has to remember to take.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "scripts"))


def _fit():
    # Imported lazily: srl_gui pulls in PyQt5 and rclpy at module scope, and
    # a machine without them should skip rather than error the whole file.
    try:
        from srl_gui import fit_to_screen
    except Exception as exc:                       # pragma: no cover
        pytest.skip("srl_gui not importable here: %s" % exc)
    return fit_to_screen


def test_it_fits_the_screen_that_caused_the_bug():
    """1920x1200 -- this host. The window must not be 1920 wide."""
    w, h = _fit()(1920, 1200)
    assert w <= 1920 - 40, "still overhangs: %d" % w
    assert h <= 1060


def test_it_never_exceeds_the_available_area():
    """The property that matters, over a spread of real screens."""
    fit = _fit()
    for aw, ah in [(1920, 1200), (1920, 1080), (1366, 768), (1280, 1024),
                   (2560, 1440), (3840, 2160), (1024, 700)]:
        w, h = fit(aw, ah)
        assert w <= aw, "%dx%d -> width %d off-screen" % (aw, ah, w)
        assert h <= ah, "%dx%d -> height %d off-screen" % (aw, ah, h)


def test_a_big_screen_still_caps_at_the_wanted_size():
    """Fitting is a CEILING, not a stretch. 4K must not open at 4K."""
    assert _fit()(3840, 2160) == (1920, 1060)


def test_a_tiny_screen_is_clamped_rather_than_vanishing():
    """A window clamped to nothing is not a fix -- the controls must exist."""
    w, h = _fit()(320, 200)
    assert (w, h) == (960, 600)


def test_an_unknown_screen_falls_back_rather_than_guessing_small():
    """Guessing small on a screen we cannot measure hides the left column."""
    fit = _fit()
    for bad in [(0, 0), (None, None), (-1, -1), (1920, 0)]:
        assert fit(*bad) == (1920, 1060), bad


def test_the_check_fails_on_a_broken_fit():
    """A CHECK THAT CANNOT FAIL ON A DELIBERATELY BROKEN INPUT IS NOT A CHECK.

    This is the hard-coded resize the fix replaced. If someone reverts to it,
    the assertions above are the ones that must go red -- so prove here that
    they do, rather than trusting that they would.
    """
    def broken(avail_w, avail_h, want_w=1920, want_h=1060, margin_px=40):
        return int(want_w), int(want_h)          # the old behaviour

    with pytest.raises(AssertionError):
        w, _ = broken(1920, 1200)
        assert w <= 1920 - 40, "still overhangs: %d" % w
