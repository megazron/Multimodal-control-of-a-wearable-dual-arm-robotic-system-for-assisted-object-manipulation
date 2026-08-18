"""THE WAKE MATCHER, PINNED TO WHAT A REAL TRANSCRIBER ACTUALLY PRODUCED.

Every string below was written by faster-whisper `small`/int8 from Piper
audio, recorded in `recordings/baselines/wake_word.json`. None of them is
invented, which is the point: the alias list this replaced was five
hand-guessed spellings and caught 13 of 40 real transcriptions.

The test also pins the KNOWN FALSE ACCEPT rather than hiding it. A wake
matcher that lets "hey doc how long have you been running that" through is a
fact about the phrase "hey doc oc", and a test that quietly stopped asserting
it would be a test that let the safety margin drift without saying so.
"""
import os
import sys

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))

from srl_autonomy import voice_intent as vi                   # noqa: E402

# Transcriptions of "hey doc oc <instruction>", verbatim from the measurement.
HEARD = [
    "Hey Doc Ock, put the blue ones on the blue pad.",
    "Hey Duck Ock, put the blue ones on the blue pad.",
    "Hey duck-ock pick up the blue cube and put it on the blue pad.",
    "Hey duck, ock move that blue block to its color.",
    "Hey duck, ox sort the cubes by color.",
    "Hey duck, uck, move all the cubes to the color that matches.",
    "Hey duck-cock, put the GERN 1s on the GERN pad.",
]

# AND THE ONES IT STILL REFUSES, WHICH ARE PART OF THE MEASUREMENT AND NOT AN
# OVERSIGHT. Both are 4 edits from `heydococ`, one edit past the shipped
# threshold, and threshold 4 is where the false-accept count doubles. They are
# listed so the residual has a name and a size instead of being the difference
# between two percentages nobody reads.
STILL_REFUSED = [
    "Hey duckuck, put every cube on the pad of its own color.",
    "Hayduck Ock put the blue cube and the green cube on the pads.",
]


def test_exact_match_is_unchanged_and_consumes_only_itself():
    ok, rest = vi.strip_wake("hey doc oc put the blue ones on the blue pad")
    assert ok
    assert rest == "put blue ones on blue pad", rest


def test_every_measured_transcription_is_accepted():
    missed = [h for h in HEARD if not vi.strip_wake(h)[0]]
    assert not missed, (
        "%d of %d real transcriptions of the wake phrase were refused: %s"
        % (len(missed), len(HEARD), missed))


def test_the_two_that_are_still_refused_are_exactly_these_two():
    for h in STILL_REFUSED:
        ok, _r = vi.strip_wake(h)
        e, _k, _f = vi.wake_distance(h)
        assert not ok, (
            "%r is now accepted. Good, if the threshold was raised "
            "deliberately -- but wake_word.json says that costs false "
            "accepts, so re-run scripts/measure_wake_word.py." % h)
        assert e == vi.WAKE_MAX_EDITS + 1, (h, e)


def test_the_instruction_survives_the_fuzzy_match():
    """Accepting the wake word must not eat the sentence with it."""
    ok, rest = vi.strip_wake("Hey duck, ock move that blue block to its color.")
    assert ok
    assert "blue block" in rest and "move" in rest, rest


def test_the_wake_word_is_stripped_from_the_end_too():
    ok, rest = vi.strip_wake("put the blue ones on the blue pad hey duck ock")
    assert ok
    assert rest.startswith("put"), rest
    assert "duck" not in rest, rest


def test_unrelated_speech_is_refused():
    for s in ("put the kettle on would you",
              "we should stop for lunch soon",
              "the pad on the left is a bit sticky",
              "did anyone move my cube of resin",
              "i think the green one is the spare"):
        assert not vi.strip_wake(s)[0], s


def test_the_known_false_accept_is_still_the_only_one_of_its_kind():
    """`hey doc` IS the wake phrase minus two letters. Recorded, not fixed.

    This asserts the defect deliberately. If someone changes the wake phrase
    to something distinctive this test should be updated to assert the refusal
    instead -- and that edit is exactly the moment to re-run
    `scripts/measure_wake_word.py`.
    """
    ok, _rest = vi.strip_wake("Hey Doc, how long have you been running that?")
    assert ok, ("this used to be accepted at the measured threshold; if it is "
                "now refused the wake phrase or the threshold has changed and "
                "wake_word.json is stale")
    # and the one that a LOOSER threshold let through must still be refused
    assert not vi.strip_wake("He took the blue one home yesterday.")[0]


def test_distance_is_symmetric_in_where_the_phrase_sits():
    a = vi.wake_distance("hey duck ock put the blue ones on the blue pad")
    b = vi.wake_distance("put the blue ones on the blue pad hey duck ock")
    assert a[0] == b[0], (a, b)
    assert a[2] is False and b[2] is True


def test_no_wake_phrase_means_no_comparison_rather_than_a_quiet_yes():
    assert vi.strip_wake("anything at all", wake="")[0] is True
    assert vi.wake_distance("", "hey doc oc") == (None, 0, False)
