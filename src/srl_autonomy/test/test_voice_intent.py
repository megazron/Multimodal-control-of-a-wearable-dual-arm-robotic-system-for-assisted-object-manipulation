#!/usr/bin/env python3
"""Voice grammar: what it accepts, and -- more importantly -- what it refuses."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from srl_autonomy import voice_intent as vi          # noqa: E402

W = "hey doc oc"


def p(t, **kw):
    return vi.parse(t, **kw)


@pytest.mark.parametrize("utt,verb,target", [
    ("hey doc oc grab the green cube", "grab", "green cube"),
    ("hey doc oc please pick up the red block", "grab", "red block"),
    ("hey doc oc take the blue ball", "grab", "blue ball"),
])
def test_grab(utt, verb, target):
    i = p(utt)
    assert i.verb == verb and i.target == target


def test_handover_and_place_and_stop():
    assert p("hey doc oc give it to the other arm").verb == "handover"
    assert p("hey doc oc hand it over to the other gripper").verb == "handover"
    assert p("hey doc oc put it down").verb == "place"
    assert p("stop").verb == "stop"


def test_stop_needs_no_wake_word():
    """Requiring a wake word before 'stop' puts a word between the operator
    and halting a moving arm."""
    for u in ("stop", "halt", "STOP!", "no wait stop"):
        assert p(u).verb == "stop", u


def test_wake_word_is_required_for_everything_else():
    i = p("grab the green cube")
    assert not i.ok and "wake word" in i.reason


def test_unknown_verb_is_refused_not_guessed():
    i = p("hey doc oc do the thing with the cube")
    assert not i.ok and "no known verb" in i.reason


def test_grab_without_a_target_is_refused():
    i = p("hey doc oc grab it")
    assert not i.ok and "no target" in i.reason


def test_deictic_is_flagged_not_resolved():
    """'that thing over there' cannot be grounded from voice alone -- mode 6
    has no pointing input -- so it must be marked, and the caller asks."""
    i = p("hey doc oc grab that thing over there")
    assert i.verb == "grab" and i.deictic is True


OBJS = [
    {"label": "green cube", "confidence": 0.9, "position": [-0.2, 0.3, 0.9]},
    {"label": "green cube", "confidence": 0.8, "position": [+0.2, 0.3, 0.9]},
    {"label": "red block", "confidence": 0.7, "position": [0.0, 0.3, 0.9]},
]


def test_unique_match():
    s, c, _ = vi.resolve_target("red block", OBJS)
    assert s == vi.UNIQUE and c["label"] == "red block"


def test_two_matches_is_AMBIGUOUS_and_chooses_NOTHING():
    """The failure this exists to prevent: silently taking the first of
    several, which defeats the spoken confirmation entirely."""
    s, c, cands = vi.resolve_target("green cube", OBJS)
    assert s == vi.AMBIGUOUS
    assert c is None
    assert len(cands) == 2


def test_no_match_is_no_match():
    s, c, _ = vi.resolve_target("purple bottle", OBJS)
    assert s == vi.NO_MATCH and c is None


def test_confidence_floor_excludes():
    s, _, _ = vi.resolve_target("red block", OBJS, min_confidence=0.85)
    assert s == vi.NO_MATCH


def test_spatial_disambiguation_uses_the_repo_frame():
    """+x is the wearer's RIGHT. 'left' must pick the NEGATIVE x candidate."""
    _, _, cands = vi.resolve_target("green cube", OBJS)
    left = vi.spatial_disambiguate("the left one", cands)
    right = vi.spatial_disambiguate("the right one", cands)
    assert left["position"][0] < 0
    assert right["position"][0] > 0


def test_question_names_the_choice():
    _, _, cands = vi.resolve_target("green cube", OBJS)
    q = vi.question_for("green cube", cands)
    assert "left" in q and "right" in q and "?" in q
