#!/usr/bin/env python3
"""Degraded mode: freeze what is broken, and never invent a value.

The properties pinned here are the ones whose violation would be silent.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_teleop import degraded_mode as dg          # noqa: E402

BASE = {
    "l_j1": {"verdict": "INTERMITTENT"}, "l_j2": {"verdict": "INCOHERENT"},
    "l_j3": {"verdict": "INCOHERENT"}, "l_j4": {"verdict": "INCOHERENT"},
    "l_j5": {"verdict": "INCOHERENT"}, "l_j6": {"verdict": "INCOHERENT"},
    "l_j7": {"verdict": "ALIVE"},
    "r_j1": {"verdict": "ALIVE"}, "r_j2": {"verdict": "ALIVE"},
    "r_j3": {"verdict": "INCOHERENT"}, "r_j4": {"verdict": "ALIVE"},
    "r_j5": {"verdict": "DEAD"}, "r_j6": {"verdict": "ALIVE"},
    "r_j7": {"verdict": "DEAD"},
}
HEALTHY = {"%s_j%d" % (a, i + 1): {"verdict": "ALIVE"}
           for a in ("l", "r") for i in range(7)}


def test_counts_the_measured_baseline():
    assert dg.n_coherent(BASE) == 6           # l_j1, l_j7, r_j1/2/4/6


def test_intermittent_is_usable_but_incoherent_is_not():
    """An INTERMITTENT channel drops out -- a KNOWN loss the validator
    already rejects. An INCOHERENT one returns plausible wrong values, which
    nothing downstream can detect. Only the second must be frozen."""
    exc = dg.excluded_channels(BASE)
    assert 0 not in exc["left"]               # l_j1 INTERMITTENT -> kept
    assert 1 in exc["left"]                   # l_j2 INCOHERENT   -> frozen
    assert 4 in exc["right"]                  # r_j5 DEAD         -> frozen


def test_auto_engages_below_threshold_and_clears_above():
    on, _, _ = dg.decide(BASE, "auto", min_coherent=12)
    assert on is True
    off, exc, _ = dg.decide(HEALTHY, "auto", min_coherent=12)
    assert off is False
    assert exc == {"left": [], "right": []}


def test_unknown_health_is_not_treated_as_healthy():
    """An absent baseline must not read as an all-clear -- that is the same
    error as treating an EXPIRED BlockMonitor entry as clear."""
    on, exc, why = dg.decide({}, "auto")
    assert on is True
    assert "NO channel baseline" in why
    # ...but it must not freeze channels it has no evidence against either.
    assert exc == {"left": [], "right": []}


def test_off_never_freezes_anything():
    on, exc, _ = dg.decide(BASE, "off")
    assert on is False and exc == {"left": [], "right": []}


def test_left_loses_reach_and_right_does_not():
    """The measured cost. Left j2/j4 are the reach pots and both are
    incoherent, so the left commanded set collapses to a spherical shell.
    Right freezes only j3/j5/j7, none of which spherical reach depends on
    materially (j3's measured contribution is 4.5 mm)."""
    exc = dg.excluded_channels(BASE)
    assert "reach FROZEN" in dg.position_capability("left", exc["left"])
    assert "reach LIVE" in dg.position_capability("right", exc["right"])
    # azimuth survives on both -- j1 is coherent on both arms
    assert "azimuth LIVE" in dg.position_capability("left", exc["left"])


def test_banner_names_every_frozen_channel():
    on, exc, why = dg.decide(BASE, "auto")
    txt = dg.banner(BASE, on, exc, why)
    for i in exc["left"]:
        assert "j%d" % (i + 1) in txt
    assert "DEGRADED MODE ACTIVE" in txt
    assert "check_channels.sh" in txt         # the recovery path is stated


@pytest.mark.parametrize("mode", ["auto", "on", "off"])
def test_decide_is_total(mode):
    on, exc, why = dg.decide(BASE, mode)
    assert isinstance(on, bool) and why
    assert set(exc) == {"left", "right"}


# ---------------------------------------------------------------------------
# CAN A POT REPAIR ACTUALLY TAKE EFFECT?
#
# Until 2026-08-09 the answer was NO, and not for the reason anyone was
# looking at. The warning was not too quiet -- the runtime was reading a
# DIFFERENT FILE from the one the lab writes. `check_channels.sh` saves the
# new reference as channels_<today>.json and diffs against
# `ls -t channels_*.json | head -1`; `default_baseline_path()` returned the
# literal string "channels_20260806.json". So a session could repair every
# pot, re-capture, watch the script confirm 14/14, and master_pose_node would
# go on freezing eight working channels for ever.
# ---------------------------------------------------------------------------

def _write(dirpath, name, verdicts, mtime=None):
    import json
    import os
    p = os.path.join(dirpath, name)
    with open(p, "w") as fh:
        json.dump({"%s_j%d" % (a, i + 1): {"verdict": v}
                   for a in ("l", "r") for i, v in enumerate(verdicts)}, fh)
    if mtime is not None:
        os.utime(p, (mtime, mtime))
    return p


def test_the_newest_baseline_wins_so_a_repair_can_land(tmp_path):
    import os
    d = tmp_path / "recordings" / "baselines"
    d.mkdir(parents=True)
    broken = ["INTERMITTENT"] + ["INCOHERENT"] * 5 + ["ALIVE"]
    _write(str(d), "channels_20260806.json", broken, mtime=1000)
    fixed = _write(str(d), "channels_20260812.json", ["ALIVE"] * 7, mtime=2000)

    got = dg.default_baseline_path(pkg_root=str(tmp_path))
    assert got == fixed, "the runtime is still reading the OLD baseline"

    # ...and the repair now actually disengages degraded mode.
    active, _, why = dg.decide(dg.load_baseline(got), "auto")
    assert active is False, why
    assert "FULL channel set" in why

    # while the stale one still engages it, so the test is not vacuous
    old = os.path.join(str(d), "channels_20260806.json")
    active_old, _, _ = dg.decide(dg.load_baseline(old), "auto")
    assert active_old is True


def test_the_shell_script_and_the_runtime_pick_THE_SAME_file(tmp_path):
    """Two 'which baseline is current' rules in one repo is the drift class
    that caused this. `ls -t | head -1` and default_baseline_path() must
    agree, and this is what notices if either is changed alone."""
    import os
    import subprocess
    d = tmp_path / "recordings" / "baselines"
    d.mkdir(parents=True)
    _write(str(d), "channels_20260806.json", ["ALIVE"] * 7, mtime=1000)
    _write(str(d), "channels_20260812.json", ["ALIVE"] * 7, mtime=2000)
    _write(str(d), "channels_20260810.json", ["ALIVE"] * 7, mtime=1500)

    shell = subprocess.run(
        ["bash", "-c", 'ls -t "$1"/channels_*.json | head -1', "_", str(d)],
        capture_output=True, text=True).stdout.strip()
    runtime = dg.default_baseline_path(pkg_root=str(tmp_path))
    assert os.path.basename(shell) == os.path.basename(runtime)


def test_no_baseline_at_all_still_names_where_it_looked(tmp_path):
    p = dg.default_baseline_path(pkg_root=str(tmp_path))
    assert "recordings/baselines" in p.replace(os.sep, "/")


def test_the_staleness_warning_is_INSIDE_the_banner(tmp_path):
    """It used to be a separate log call, which raised NameError on every run
    and was swallowed by a bare `except` -- shipped, verified in isolation,
    and never once fired. Carrying it in the banner means it cannot go
    missing on its own."""
    broken = {"%s_j%d" % (a, i + 1): {"verdict": "INCOHERENT"}
              for a in ("l", "r") for i in range(7)}
    active, exc, why = dg.decide(broken, "auto")
    # Normalise whitespace: the banner hard-wraps to 68 columns, so a phrase
    # may legitimately straddle two lines.
    text = " ".join(dg.banner(broken, active, exc, why).split())
    assert "NOT FROM THIS ARM" in text
    assert "check_channels.sh" in text
    assert "FIRST ACTION OF EVERY LAB SESSION" in text


def test_a_healthy_baseline_produces_NO_stale_warning():
    """Guards the other direction: a warning that always fires is one nobody
    reads."""
    healthy = {"%s_j%d" % (a, i + 1): {"verdict": "ALIVE"}
               for a in ("l", "r") for i in range(7)}
    active, exc, why = dg.decide(healthy, "auto")
    assert dg.staleness_warning(healthy) is None
    assert "NOT FROM THIS ARM" not in " ".join(
        dg.banner(healthy, active, exc, why).split())
