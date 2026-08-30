#!/usr/bin/env python3
"""The shim over srl_teleop.smoothing exports what vr_smoothing always did.

The implementation MOVED on 2026-08-27 (the master path needed the same
filter, and HARD CONSTRAINT 7 means shared code can only live in srl_teleop).
A move is invisible to every VR consumer only while the old import path keeps
serving the same objects -- so the export list is pinned HERE, name by name,
against the implementation module. A dropped name would otherwise surface as
an ImportError in whichever consumer imports it next, on a lab day.
"""
from srl_teleop import smoothing as impl

from srl_vr_teleop import vr_smoothing as shim

# The public surface as of the move, from the module that shipped 2026-08-26.
# `_Passthrough` and `_self_test` are underscore-named but were reachable and
# are re-exported too: make("none") returns a _Passthrough, and the module's
# own `--self-test` entry point runs _self_test.
NAMES = (
    "TWO_PI",
    "alpha_for",
    "LowPass",
    "OneEuro",
    "OneEuroQuat",
    "LegacyEma",
    "make",
    "q_norm",
    "q_canon",
    "q_slerp",
    "q_angle",
    "_Passthrough",
    "_self_test",
)


def test_every_pinned_name_is_the_implementation_object():
    """Identity, not equality: a re-implementation that behaved 'the same'
    would still be a fork, and forks drift."""
    for name in NAMES:
        assert getattr(shim, name) is getattr(impl, name), (
            "vr_smoothing.%s is not srl_teleop.smoothing.%s -- the shim has "
            "forked or dropped it" % (name, name))


def test_make_through_the_old_path_builds_the_moved_classes():
    assert isinstance(shim.make("one_euro"), impl.OneEuro)
    assert isinstance(shim.make("ema"), impl.LegacyEma)
    assert isinstance(shim.make("none"), impl._Passthrough)
