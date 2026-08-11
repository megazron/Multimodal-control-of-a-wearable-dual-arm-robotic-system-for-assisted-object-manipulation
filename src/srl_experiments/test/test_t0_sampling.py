#!/usr/bin/env python3
"""T0 randomisation: reproducible, safe, and it REFUSES rather than degrades.

The brief's guards, carried forward as code rather than as memory:
  * a sweep must REFUSE when tested == 0
  * a control that fails means the sweep does not report
  * randomisation samples ONLY from verified safe regions and must never
    generate an unsafe configuration

Every one of those is a property of the sampler, so they are asserted here.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "experiments", "abc"))

import task0 as T0                                        # noqa: E402
from srl_experiments.task_actions import Verb             # noqa: E402


def test_same_seed_reproduces_the_trial_exactly():
    """The contract that makes a trial replayable. Without it the `seed`
    column in the trial log is decoration."""
    for seed in (0, 1, 42, 9999):
        a, _ = T0.sample_trial(seed)
        b, _ = T0.sample_trial(seed)
        assert a == b
    assert T0.sample_trial(1)[0] != T0.sample_trial(2)[0], (
        "different seeds produced the same trial -- the seed is not wired in")


def test_every_sample_is_inside_the_verified_band():
    """The band is measured with the bench in the planning scene. A sample
    outside it has never been verified reachable by anything."""
    lo_x, hi_x = T0.BAND["x"]
    lo_z, hi_z = T0.BAND["z"]
    n = 0
    for seed in range(200):
        t, _ = T0.sample_trial(seed)
        for label, p in t.items():
            n += 1
            assert lo_x - 1e-9 <= abs(p[0]) <= hi_x + 1e-9, (label, p)
            assert p[1] == T0.BAND["y"], (label, p)
            assert lo_z - 1e-9 <= p[2] <= hi_z + 1e-9, (label, p)
            assert (p[0] > 0) == label.startswith("L"), (
                "%s is on the wrong side: %s" % (label, p))
    assert n == 200 * 6, "checked %d poses, expected 1200" % n


def test_separation_is_always_honoured():
    for seed in range(200):
        t, _ = T0.sample_trial(seed)
        for arm in ("left", "right"):
            pts = [t[k] for k in T0.LABELS[arm]]
            for i in range(len(pts)):
                for j in range(i + 1, len(pts)):
                    d = sum((a - b) ** 2 for a, b in zip(pts[i], pts[j])) ** .5
                    assert d >= T0.MIN_SEPARATION_M - 1e-9, (seed, arm, d)


def test_it_REFUSES_rather_than_returning_fewer_targets():
    """The guard that found the real conflict: three spheres at 80 mm do not
    fit the 200 x 80 mm band reliably. It must raise, not return two."""
    with pytest.raises(T0.RegionExhausted):
        T0.sample_targets("left", 0, n=3, min_sep=0.25, max_draws=200)


def test_a_refusing_validator_is_never_silently_ignored():
    """A validator that rejects everything must produce a refusal, not an
    unvalidated sample. This is the 'control that fails means the sweep does
    not report' rule applied to randomisation."""
    with pytest.raises(T0.RegionExhausted):
        T0.sample_targets("left", 0, validate=lambda arm, p: False,
                          max_draws=100)

    # And a validator that accepts must let the same seed through, or the
    # test above would pass for the wrong reason.
    got, meta = T0.sample_targets("left", 0, validate=lambda arm, p: True)
    assert len(got) == T0.TARGETS_PER_ARM
    assert meta["rejected"]["validator"] == 0


def test_validator_actually_runs_on_every_candidate():
    """A validator nobody calls is the same as no validator. Counted, because
    'it was passed in' is not evidence it was used."""
    seen = []
    T0.sample_targets("left", 3, validate=lambda arm, p: (seen.append(p)
                                                          or True))
    assert len(seen) >= T0.TARGETS_PER_ARM


def test_difficulty_controls_how_many_are_visible():
    t, _ = T0.sample_trial(11)
    for d in (1, 2, 3):
        assert len(T0.visible_for(d, "left", t)) == d
        assert len(T0.visible_for(d, "right", t)) == d
    with pytest.raises(ValueError):
        T0.visible_for(4, "left", t)
    with pytest.raises(ValueError):
        T0.visible_for(0, "left", t)


def test_one_trial_is_six_reaching_events_not_one():
    """The brief is explicit: 3 left targets and 3 right, per trial."""
    t, _ = T0.sample_trial(5)
    cmds = T0.commands(t)
    assert len(cmds) == 6
    assert all(c.verb is Verb.REACH_TARGET for c in cmds)
    assert sum(1 for c in cmds if c.arm == "left") == 3
    assert sum(1 for c in cmds if c.arm == "right") == 3
    assert [c.subject for c in cmds] == ["L1", "L2", "L3", "R1", "R2", "R3"]
    # The commands carry the sampled pose, not a placeholder.
    for c in cmds:
        assert c.pose == list(t[c.subject])


def test_commands_are_identical_whatever_mode_will_run_them():
    """T0's half of the architecture principle: the command list is built
    before any mode is chosen and does not mention one."""
    t, _ = T0.sample_trial(5)
    a = T0.commands(t)
    b = T0.commands(t)
    assert [(c.verb, c.arm, c.subject, tuple(c.pose)) for c in a] == \
           [(c.verb, c.arm, c.subject, tuple(c.pose)) for c in b]
    for c in a:
        assert "MODE" not in str(c.params)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
