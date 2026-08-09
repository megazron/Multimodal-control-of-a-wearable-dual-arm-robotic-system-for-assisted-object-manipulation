#!/usr/bin/env python3
"""Known answers for free-form referring expressions.

The category these exist to pin is MISUNDERSTOOD: grounded confidently to the
wrong object. A refusal costs a repeat; a misunderstanding moves a manipulator
toward the wrong thing beside a person.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from srl_autonomy import referring as R          # noqa: E402

S = [dict(label="red block", xyz=(0.32, 0.35, 1.02), rgb=(0.8, .1, .1),
          volume=6.4e-5),
     dict(label="blue block", xyz=(-0.30, 0.35, 1.02), rgb=(.15, .3, .85),
          volume=6.4e-5),
     dict(label="blue cup", xyz=(-0.10, 0.42, 1.05), rgb=(.15, .3, .85),
          volume=2.4e-4)]


def out(t, scene=None):
    return R.ground(t, scene if scene is not None else S)["outcome"]


def tgt(t):
    return R.ground(t, S)["target"]["label"]


def test_a_phrase_that_constrains_nothing_never_grounds():
    for t in ("pick that random thing", "grab something", "take whatever",
              "get me that thing over there"):
        assert out(t) != R.GROUNDED, t


def test_but_it_grounds_when_there_is_only_one_object():
    """Vacuous is not the same as impossible: with one object present, "that
    thing" is unambiguous."""
    assert out("pick that thing", [S[0]]) == R.GROUNDED


def test_relational_references_are_refused_not_guessed():
    """Measured failure: 'the one behind the red block' grounded to the red
    block itself, the single dangerous outcome in the phrase set."""
    for t in ("the one behind the red block", "the block next to the cup",
              "the thing on top of the tray"):
        r = R.ground(t, S)
        assert r["outcome"] == R.NONE
        assert "relational" in r["reason"]


def test_verb_words_do_not_leak_into_the_noun():
    """They did, and every label then mismatched them, turning 'grab the
    leftmost object' into a refusal."""
    assert "grab" not in R.parse_reference("grab the red block").nouns
    assert "pick" not in R.parse_reference("pick up the red block").nouns


def test_superlatives_are_definite():
    assert tgt("the leftmost object") == "blue block"
    assert tgt("the biggest one") == "blue cup"


def test_a_genuine_ambiguity_asks():
    assert out("pick up the blue one") == R.ASK


def test_colour_plus_noun_disambiguates_it():
    assert tgt("pick up the blue block") == "blue block"
    assert tgt("take the blue cup") == "blue cup"


def test_absent_object_is_refused():
    assert out("pick up the purple elephant") == R.NONE


def test_empty_scene_refuses_rather_than_crashing():
    assert out("pick up the red block", []) == R.NONE


def test_unreachable_target_is_named_before_committing():
    def unreachable(p):
        return False, "past my reach"
    r = R.plan_request("pick up the red block", S, unreachable)
    assert r["outcome"] == R.NONE
    assert "can't reach" in r["say"]
    assert "announce" not in r


def test_a_reachable_target_announces_and_waits():
    r = R.plan_request("pick up the red block", S, lambda p: (True, ""))
    assert r["outcome"] == R.GROUNDED
    assert r["needs_confirmation"] is True
    assert "red block" in r["announce"]


def test_keepout_objects_are_never_offered_as_options():
    """Filtering after resolution lets the robot offer a target it will then
    refuse, which invites the operator to pick it and trust the answer."""
    def ko(p):
        return "by the wearer" if p[0] > 0.3 else None
    r = R.plan_request("pick up the blue one", S, lambda p: (True, ""), ko)
    assert "red block" not in str(r.get("candidates", []))


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
