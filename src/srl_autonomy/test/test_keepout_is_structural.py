#!/usr/bin/env python3
"""A forbidden target must never become a disambiguation candidate.

The bug this pins: the wearer keep-out was evaluated in start_pick, AFTER
resolution. A target behind the wearer therefore became one of three
candidates and the robot asked "which one -- left, right, near or far?",
offering a position it must never reach. Asking the operator to choose an
option that will then be refused invites them to say "the far one" and trust
the answer.

Filtering after the fact is not enough: it leaves every future code path
free to forget. Exclusion now happens INSIDE WorldModel.match(), so a
forbidden object cannot be returned to any consumer.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "srl_teleop"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_autonomy.autonomy_executive import (               # noqa: E402
    behind_wearer, inside_wearer)
from srl_autonomy.world_model import WorldModel             # noqa: E402
from srl_autonomy import voice_intent as vi                 # noqa: E402


def forbidden(p):
    return behind_wearer(p) or inside_wearer(p)[0]


def det(label, xyz, c=0.9):
    return {"label": label, "position": list(xyz), "confidence": c}


ALLOWED = [+0.34, 0.30, 0.90]
BEHIND_1 = [+0.20, -0.30, 1.10]
BEHIND_2 = [-0.25, -0.40, 0.95]
IN_HEAD = [0.0, 0.0, 1.295]
IN_TORSO = [0.0, -0.06, 1.05]


def wm_with(objs):
    m = WorldModel()
    m.observe([det("green cube", p) for p in objs])
    return m


def test_every_forbidden_target_is_excluded_at_the_source():
    """FOUR forbidden positions, three distinct reasons."""
    m = wm_with([BEHIND_1, BEHIND_2, IN_HEAD, IN_TORSO])
    assert len(m.match("green cube")) == 4          # all are in the model
    assert m.match("green cube", exclude=forbidden) == []


def test_none_is_ever_offered_as_a_disambiguation_candidate():
    """The exact failure: several forbidden + one allowed must resolve to
    the allowed one UNIQUELY, never to an ambiguous question."""
    m = wm_with([BEHIND_1, BEHIND_2, IN_HEAD, ALLOWED])
    cands = m.match("green cube", exclude=forbidden)
    status, chosen, cs = vi.resolve_target("green cube", cands)
    assert status == vi.UNIQUE
    assert chosen["position"] == ALLOWED
    assert all(not forbidden(c["position"]) for c in cs)


def test_all_forbidden_is_NO_MATCH_not_a_question():
    m = wm_with([BEHIND_1, BEHIND_2, IN_HEAD, IN_TORSO])
    cands = m.match("green cube", exclude=forbidden)
    status, chosen, _ = vi.resolve_target("green cube", cands)
    assert status == vi.NO_MATCH and chosen is None


def test_the_executive_can_still_say_WHY():
    """Refusing with 'I can't see anything' when the object is really behind
    the wearer would be true but useless. count_excluded() is what lets the
    refusal name the real reason."""
    m = wm_with([BEHIND_1, BEHIND_2])
    assert m.count_excluded("green cube", forbidden) == 2
    m2 = wm_with([ALLOWED])
    assert m2.count_excluded("green cube", forbidden) == 0


def test_two_allowed_still_ambiguates_normally():
    """The fix must not suppress genuine ambiguity."""
    m = WorldModel()
    m.observe([det("green cube", [-0.30, 0.34, 0.90]),
               det("green cube", [+0.30, 0.34, 0.90])])
    cands = m.match("green cube", exclude=forbidden)
    status, _, cs = vi.resolve_target("green cube", cands)
    assert status == vi.AMBIGUOUS and len(cs) == 2


@pytest.mark.parametrize("p,why", [
    (BEHIND_1, "behind"), (BEHIND_2, "behind"),
    (IN_HEAD, "head"), (IN_TORSO, "torso")])
def test_each_forbidden_position_is_individually_rejected(p, why):
    assert forbidden(p), (p, why)
