"""T1 FROM A TYPED SENTENCE, and the colour comes from the CAMERA.

`t1_instruction.plan_from()` is the join between `voice_intent.parse()` and
`vision_grasp.observe_and_detect()`. Everything here runs offline: the scene is
a list of (x, y, pad_index) tuples, which is exactly what the detector returns,
so the ground truth is CONSTRUCTED and the standing rule is satisfied. Nothing
in this file says anything about whether the camera can see a cube.

THE ONE THAT MATTERS IS THE MISLABEL. `msc_clip_tasks.T1_PAIR` declares each
cube's colour and this module must never read it. The test below lies in
T1_PAIR -- flips every declaration -- and asserts the plan does not move. A
plan that changes when the declaration changes is a plan built from the file,
and the whole perception path would be decoration.

THE OTHER ONE THAT MATTERS IS THE SINGULAR. "put the blue cube on the blue
pad" against two blue cubes must ASK. Every way of choosing one of them --
first, nearest, leftmost -- is defensible and is a guess, and a guess moves
metal next to a person.
"""
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
for _p in (os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc"),
           os.path.join(ROOT, "src", "srl_autonomy"),
           os.path.join(ROOT, "config")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import t1_instruction as TI                                  # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402

BLUE, GREEN = 0, 1
SEEN = [(0.560, 0.120, BLUE), (0.620, 0.120, GREEN),
        (0.680, 0.120, BLUE), (0.740, 0.120, GREEN)]


def pads(outcome):
    """{x -> pad}, so a plan can be compared without depending on order."""
    return {round(p[0], 3): p[2] for p in outcome.picks}


# ---------------------------------------------------------------- controls
def test_the_pads_are_blue_and_green_in_that_order():
    """Everything below indexes pads by number. If the scene's colour order
    ever changes, these tests would silently be about something else."""
    assert MCT.PLANE_COLOURS == ("blue", "green")


def test_a_plain_instruction_plans():
    o = TI.plan_from("put the blue ones on the blue pad", SEEN)
    assert o.ok, o.message
    assert pads(o) == {0.560: BLUE, 0.680: BLUE}


def test_a_different_destination_gives_a_different_plan():
    """The scorer must be able to tell two plans apart, or none of the
    assertions below can fail."""
    a = TI.plan_from("put the blue ones on the blue pad", SEEN)
    b = TI.plan_from("put the blue ones on the green pad", SEEN)
    assert a.ok and b.ok
    assert pads(a) != pads(b)


# ------------------------------------------------- the mislabel, the point
def test_the_plan_ignores_the_DECLARED_colour_entirely():
    """Flip every declaration in T1_PAIR. The plan must not move.

    This is the offline half of `scripts/verify_vision_drives_grasp.py`: that
    one runs the real camera, this one runs on every commit.
    """
    saved = dict(MCT.T1_PAIR)
    before = TI.plan_from("put every cube where it belongs", SEEN)
    assert before.ok
    try:
        for k in list(MCT.T1_PAIR):
            MCT.T1_PAIR[k] = 1 - MCT.T1_PAIR[k]
        after = TI.plan_from("put every cube where it belongs", SEEN)
    finally:
        MCT.T1_PAIR.clear()
        MCT.T1_PAIR.update(saved)
    assert after.ok
    assert pads(after) == pads(before), (
        "the plan changed when the DECLARED colours changed, so it is being "
        "built from msc_clip_tasks.T1_PAIR and not from the detection")
    # and it is the SEEN colour that was used, not some constant
    assert pads(after) == {0.560: BLUE, 0.620: GREEN,
                           0.680: BLUE, 0.740: GREEN}


def test_a_mislabelled_cube_goes_where_the_CAMERA_put_it():
    """The sharper form: one cube, declared one colour and seen as the other.

    The detection says cube 0 is GREEN. Its declaration says blue. "put it
    where it belongs" must send it to the GREEN pad.
    """
    saved = dict(MCT.T1_PAIR)
    lie = [(0.560, 0.120, GREEN)]          # the camera says green
    try:
        MCT.T1_PAIR[0] = BLUE              # the file says blue
        o = TI.plan_from("put the cube where it belongs", lie)
    finally:
        MCT.T1_PAIR.clear()
        MCT.T1_PAIR.update(saved)
    assert o.ok, o.message
    assert pads(o) == {0.560: GREEN}, (
        "a cube the camera saw as green was sent to the blue pad, which is "
        "the colour its DECLARATION carries")


def test_the_module_does_not_read_the_declared_pairing_at_all():
    """A source check beside the behavioural one. The behaviour test proves
    this run did not read T1_PAIR; this proves no branch can."""
    src = open(os.path.join(ROOT, "src", "srl_experiments", "experiments",
                            "abc", "t1_instruction.py")).read()
    body = src.split('"""', 2)[-1]         # skip the module docstring
    assert "T1_PAIR" not in body, (
        "t1_instruction must ground colour on the DETECTION only")


# ------------------------------------------------------ ask, do not choose
@pytest.mark.parametrize("utt", [
    "pick up the blue cube and put it on the blue pad",
    "move that blue block to its colour",
    "put the green cube on the green pad",
])
def test_a_singular_request_against_two_matches_ASKS(utt):
    o = TI.plan_from(utt, SEEN)
    assert o.kind == TI.ASK, (utt, o.kind, o.message)
    assert o.picks == [], "it chose something while asking which one"


def test_the_plural_form_of_the_same_sentence_does_NOT_ask():
    """The difference is in the words, not in a threshold."""
    o = TI.plan_from("put the blue ones on the blue pad", SEEN)
    assert o.ok and len(o.picks) == 2


def test_a_deictic_with_nothing_to_point_at_ASKS():
    o = TI.plan_from("put it on the blue pad", SEEN)
    assert o.kind == TI.ASK
    assert "which" in o.message.lower()


# ------------------------------------------------------------- refusals
@pytest.mark.parametrize("utt,fragment", [
    ("put the red ones on the red pad", "do not see a red cube"),
    ("put the blue ones on the red pad", "no red pad"),
    ("put the tray on the blue pad", "no tray"),
    ("do not put the blue ones on the blue pad", "negated"),
    ("put the blue cube and the green cube on the pads", "more than one target"),
    ("put the blue ones on the blue pad then go home", "more than one action"),
    ("put the blue cube on top of the green cube", "relational"),
])
def test_these_refuse_and_say_why(utt, fragment):
    o = TI.plan_from(utt, SEEN)
    assert o.kind == TI.REFUSE, (utt, o.kind, o.message)
    assert fragment in o.message, (utt, o.message)


def test_no_detections_refuses_rather_than_falling_back():
    """TASK_SPEC P-4. A blind pick that quietly reverts to the file
    coordinate is indistinguishable from a working perception path."""
    o = TI.plan_from("put the blue ones on the blue pad", [])
    assert o.kind == TI.REFUSE
    assert "not fall back" in o.message


# ---------------------------------------------- cross-colour is not a bug
def test_a_destination_that_is_not_the_cubes_colour_is_HONOURED():
    """"put the blue ones on the green pad" is a perfectly ordinary
    instruction. Colour-matching it anyway would be the code overruling the
    operator, which is the same class of error as dropping a negation."""
    o = TI.plan_from("put the blue ones on the green pad", SEEN)
    assert o.ok
    assert pads(o) == {0.560: GREEN, 0.680: GREEN}


# ------------------------------------------------------------- the path
def test_the_path_is_built_by_the_ONE_builder_and_a_subset_is_shorter():
    """A two-cube instruction must produce a two-cube path, not the four-cube
    routine with two picks ignored."""
    two = TI.build_path([(0.560, 0.120, BLUE), (0.680, 0.120, BLUE)])
    four = TI.build_path([(x, y, p) for x, y, p in SEEN])
    assert len(two["left"]) * 2 == pytest.approx(len(four["left"]), abs=8)
    assert len(two["left"]) == len(two["right"]), "the idle arm must be held"


def test_the_path_picks_where_the_CAMERA_said_and_not_where_the_file_says():
    """Shift the detection 0.25 m and the commanded pick must shift with it.
    This is TASK_SPEC control P-B, at the instruction layer."""
    # THE FIRST WAYPOINT, which is the standoff above the PICK. `max` over the
    # whole path was the first attempt and it measured the wrong thing: the
    # place point sits at x = 0.595, so for a pick at 0.560 the maximum is the
    # PAD, and the shift read 0.215 instead of 0.250.
    near = TI.build_path([(0.560, 0.120, BLUE)])["left"]
    far = TI.build_path([(0.810, 0.120, BLUE)])["left"]
    dx = far[0][0] - near[0][0]
    assert dx == pytest.approx(0.25, abs=1e-6), dx
