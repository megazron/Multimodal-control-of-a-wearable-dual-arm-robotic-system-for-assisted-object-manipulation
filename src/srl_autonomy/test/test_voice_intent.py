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


# ---------------------------------------------------------------------------
# THE FOUR SENTENCE SHAPES THAT PRODUCED ALL SEVEN MISUNDERSTOOD OUTCOMES
# in the 2026-08-09 language sweep. Each one parsed to a confident, wrong
# command because the grammar matched a real verb and a real noun and threw
# away the word that reversed or qualified them. Refusing is the fix; these
# tests pin the refusal so it cannot be "simplified" back out.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("utt", [
    "hey doc oc don't grab the blue cube",
    "hey doc oc do not pick up the red block",
    "hey doc oc never grab the green ball",
    "hey doc oc I don't want you to take the blue cube",
])
def test_negation_before_the_verb_refuses(utt):
    i = vi.parse(utt)
    assert not i.ok, "NEGATED command parsed as an action: %r" % i
    assert "negat" in i.reason


def test_negation_AFTER_the_verb_is_an_exclusion_clause_not_a_refusal():
    """The positional rule is the whole point. 'grab X but not Y' is a good
    grab; refusing it would trade a real capability for no safety at all."""
    i = vi.parse("hey doc oc grab the blue cube but not the red one")
    assert i.ok and i.verb == "grab"
    assert i.target == "blue cube"


def test_relational_reference_refuses():
    i = vi.parse("hey doc oc grab the cube to the left of the red block")
    assert not i.ok
    assert "relational" in i.reason


def test_sequence_refuses_rather_than_executing_half():
    i = vi.parse("hey doc oc pick up the red block then put it down")
    assert not i.ok
    assert "one action" in i.reason


def test_two_targets_refuse_rather_than_picking_one():
    i = vi.parse("hey doc oc grab the blue cube and the green ball")
    assert not i.ok
    assert "one target" in i.reason


def test_a_bare_conjunction_is_not_two_targets():
    """'and' alone must not become a refusal -- only a second NAMED target."""
    i = vi.parse("hey doc oc go ahead and grab the blue cube")
    assert i.ok and i.target == "blue cube"


def test_stop_is_never_refused_for_being_ungrammatical():
    """The guard sits AFTER the stop check. A halt must survive any sentence,
    including ones this module would otherwise refuse."""
    for utt in ("don't move, stop",
                "stop and then let go",
                "stop next to the red block"):
        assert vi.parse(utt).verb == "stop", utt


def test_verbs_are_matched_EXACTLY_and_here_is_why():
    """MEASURED HAZARD, not a style preference.

    Fuzzy-matching the verb vocabulary at Damerau-Levenshtein 1 would make
    'top' a STOP, 'crop' a DROP, and 'let'/'yet'/'net'/'bet' all a GET or a
    SET. The verb decides the ACTION, so a near-miss there is the one failure
    this module is built to prevent -- which is why a misspelled verb refuses
    outright while a misspelled NOUN degrades to a question.
    """
    assert not vi.parse("hey doc oc grabb the blue cube").ok       # verb: refuse
    assert not vi.parse("hey doc oc top").ok                       # NOT a stop
    i = vi.parse("hey doc oc take the gren ball")                  # noun: open
    assert i.ok and i.verb == "grab" and i.target == "ball"


# ------------------------------------------------- the demonstration verbs
def test_hand_it_to_me_is_the_WEARER_not_the_other_arm():
    """Two different mechanisms, and only one of them is possible here.

    Arm-to-arm transfer is IMPOSSIBLE on this rig: 0 of 16 candidate transfer
    points reachable by both arms, 0 of 63 frontal cells, the reachable sets
    being disjoint. Handing an object TO THE WEARER is T5 and is feasible. One
    verb for both would route a possible request into an impossible mechanism.
    """
    for u in ("hand it to me", "give it to me", "pass it to me"):
        r = vi.parse("hey doc oc " + u, wake=vi.WAKE_DEFAULT)
        assert r.verb == "handover_wearer", (u, r.verb, r.reason)


def test_arm_to_arm_handover_still_parses_as_itself():
    r = vi.parse("hey doc oc hand it over to the other arm",
                 wake=vi.WAKE_DEFAULT)
    assert r.verb == "handover", (r.verb, r.reason)


def test_the_front_centre_is_REFUSED_with_the_measurement():
    """The most useful thing the demonstration can show about this platform.

    The reachable region is nothing like the region a person expects, and the
    robot must say so rather than accept and fail. 0 of 319 surveyed cells lie
    at |x| <= 0.10.
    """
    for u in ("move to the front centre", "move to the front center"):
        r = vi.parse("hey doc oc " + u, wake=vi.WAKE_DEFAULT)
        assert r.verb is None, "the front centre must NOT be accepted"
        assert "not reachable" in r.reason, r.reason
        assert "0.30" in r.reason, "the refusal must carry the measurement"


def test_a_reachable_named_place_is_accepted():
    """Or the refusal above would pass for the wrong reason -- a grammar that
    refuses everything also refuses the front centre."""
    r = vi.parse("hey doc oc move to the left side", wake=vi.WAKE_DEFAULT)
    assert r.verb == "goto" and r.target == "left side", (r.verb, r.target)


def test_an_unknown_place_is_refused_by_name():
    r = vi.parse("hey doc oc move to the moon", wake=vi.WAKE_DEFAULT)
    assert r.verb is None and "no place I know" in r.reason, r.reason
