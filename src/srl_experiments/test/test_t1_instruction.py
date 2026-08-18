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
# THE SCENE THE CAMERA WOULD REPORT, READ FROM THE TASK RATHER THAN RESTATED.
#
# It used to be four literals in a row at y = 0.120 with the colours
# alternating, and after the 2026-08-18 rebuild that scene cannot exist:
# the blue pad is on the LEFT of the centreline and the green pad on the
# RIGHT, neither arm crosses the centreline (0 of 10 at every cross-side pose,
# measured), so a green cube at x = +0.62 is a cube this rig cannot deliver.
# `t1_task.build` now raises on exactly that, which is how the drift was
# found.
import t1_task as _T1M                                       # noqa: E402
SEEN = [(cx, cy, _T1M.T1_PAIR[i])
        for i, (cx, cy) in enumerate(_T1M.T1_CUBES)]
BLUE_SEEN = [c for c in SEEN if c[2] == BLUE]
GREEN_SEEN = [c for c in SEEN if c[2] == GREEN]


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
    assert pads(o) == {round(c[0], 3): BLUE for c in BLUE_SEEN}


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
    assert pads(after) == {round(c[0], 3): c[2] for c in SEEN}


def test_a_mislabelled_cube_goes_where_the_CAMERA_put_it():
    """The sharper form: one cube, declared one colour and seen as the other.

    The detection says cube 0 is GREEN. Its declaration says blue. "put it
    where it belongs" must send it to the GREEN pad.
    """
    saved = dict(MCT.T1_PAIR)
    # THE CAMERA SAYS GREEN, and the cube has to be on the GREEN pad's own
    # side of the centreline or no arm can deliver it -- which is a property
    # of the rig, not of this test. Take a real green cube's position.
    lie = [(GREEN_SEEN[0][0], GREEN_SEEN[0][1], GREEN)]
    try:
        MCT.T1_PAIR[0] = BLUE              # the file says blue
        o = TI.plan_from("put the cube where it belongs", lie)
    finally:
        MCT.T1_PAIR.clear()
        MCT.T1_PAIR.update(saved)
    assert o.ok, o.message
    assert pads(o) == {round(GREEN_SEEN[0][0], 3): GREEN}, (
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
    # BOTH BLUE CUBES TO THE GREEN PAD is a plan this rig cannot execute --
    # the green pad is on the other side of the wearer -- and that is a
    # question for the BUILDER, which refuses it by name. What this test is
    # about is whether the grounding layer OVERRULES the operator, and it must
    # not: the plan says green because the operator said green.
    assert pads(o) == {round(c[0], 3): GREEN for c in BLUE_SEEN}


# ------------------------------------------------------------- the path
def test_the_path_is_built_by_the_ONE_builder_and_a_subset_is_shorter():
    """A two-cube instruction must produce a two-cube path, not the four-cube
    routine with two picks ignored.

    IT IS NO LONGER "HALF AS LONG", AND THAT IS THE TWO-ARM REBUILD RATHER
    THAN A REGRESSION. The old T1 ran one arm, so a path's length was
    proportional to the cube count and `two * 2 == four` held to a few
    waypoints. The rebuilt T1 runs the arms IN TURN and both arrays are the
    same length by construction -- the idle arm is padded at its park pose --
    so the total is (left work + right work) for both arms whatever the split.
    Two blue cubes are all LEFT-arm work; four cubes are two each. The
    property that still has to hold is the one the test is named for: a
    subset is genuinely shorter, and no pick is silently dropped.
    """
    two = TI.build_path([tuple(c) for c in BLUE_SEEN])
    four = TI.build_path([(x, y, p) for x, y, p in SEEN])
    assert len(two["left"]) < len(four["left"]), (
        "a two-cube plan is not shorter than a four-cube one, so picks are "
        "being padded rather than dropped from the path")
    assert len(two["left"]) == len(two["right"]), "both arrays run together"
    assert len(four["left"]) == len(four["right"])
    # AND THE COUNT IS RIGHT, not merely smaller: two closes for two cubes.
    # REBUILD FIRST. `t1_task.grip()` returns the schedule belonging to the
    # LAST path built, which is `four` by this line -- reading it without
    # rebuilding measured the four-cube schedule against a two-cube path and
    # answered 4. That is the module's documented contract (the schedule is
    # built WITH the path it belongs to) and the test has to honour it.
    import t1_task as _T1
    two = TI.build_path([tuple(c) for c in BLUE_SEEN])
    g = _T1.grip(len(two["left"]))
    closes = sum(1 for a in ("left", "right")
                 for i in range(1, len(g[a]))
                 if g[a][i] != g[a][i - 1] and g[a][i] != 0.0)
    assert closes == 2, closes


def test_the_path_picks_where_the_CAMERA_said_and_not_where_the_file_says():
    """Shift the detection 0.25 m and the commanded pick must shift with it.
    This is TASK_SPEC control P-B, at the instruction layer."""
    # THE FIRST WAYPOINT, which is the standoff above the PICK. `max` over the
    # whole path was the first attempt and it measured the wrong thing: the
    # place point sits at x = 0.595, so for a pick at 0.560 the maximum is the
    # PAD, and the shift read 0.215 instead of 0.250.
    near = TI.build_path([(0.400, 0.450, BLUE)])["left"]
    far = TI.build_path([(0.650, 0.450, BLUE)])["left"]
    dx = far[0][0] - near[0][0]
    assert dx == pytest.approx(0.25, abs=1e-6), dx


# --------------------------------------------------------------------------
# THE 2026-08-18 ADDITIONS. Each of these pins a fault that was MEASURED, not
# a behaviour that was designed -- see the sweep's own header for the two the
# phrase set found on the day it was written.
# --------------------------------------------------------------------------
def test_a_complete_compound_moves_EVERY_cube_it_names():
    """The one MISUNDERSTOOD this project has recorded since it started counting.

    "put the blue ones on the blue pad and the green ones on the green pad" is
    two complete instructions sharing a verb. The shipped grounding layer
    planned the FIRST and dropped the second in silence -- two cubes of four,
    reported as a success. Half an instruction executed confidently is the
    dangerous outcome the whole module exists to prevent, and it was live.
    """
    o = TI.plan_from("put the blue ones on the blue pad and the green ones "
                     "on the green pad", SEEN)
    assert o.kind == TI.PLAN, o.message
    assert len(o.picks) == len(SEEN), (
        "planned %d picks for an instruction naming all %d cubes: %s"
        % (len(o.picks), len(SEEN), o.picks))
    for px, _py, pad in o.picks:
        seen = min(SEEN, key=lambda c: abs(c[0] - px))
        assert pad == seen[2], (px, pad, seen)


def test_a_compound_that_contradicts_itself_is_refused():
    o = TI.plan_from("put the blue ones on the blue pad and the blue ones on "
                     "the green pad", SEEN)
    assert o.kind == TI.REFUSE, (o.kind, o.message)


def test_politeness_does_not_become_a_second_action():
    """`when` is one Damerau edit from `then`, and the repair used to take it.

    A word that decides whether the sentence is REFUSED is as load-bearing as
    the verb, so it is not a repair target -- the same rule that already
    stopped `top` becoming `stop`.
    """
    o = TI.plan_from("could you please just put the blue ones on the blue pad "
                     "for me when you get a chance", SEEN)
    assert o.kind == TI.PLAN, o.message
    assert len(o.picks) == 2, o.picks


def test_a_selector_the_scene_cannot_settle_is_refused_by_name():
    """Not dropped. Dropping it turns a qualified instruction into an
    unqualified one, which is how the operator gets a cube they did not name."""
    o = TI.plan_from("put the biggest cube on the blue pad", SEEN)
    assert o.kind == TI.REFUSE
    assert "biggest" in o.message and "same" in o.message, o.message


def test_a_selector_the_scene_CAN_settle_plans_without_asking():
    o = TI.plan_from("put the leftmost blue cube on the blue pad", SEEN)
    assert o.kind == TI.PLAN, o.message
    assert len(o.picks) == 1
    blues = [c for c in SEEN if c[2] == 0]
    assert abs(o.picks[0][0] - max(c[0] for c in blues)) < 1e-9, o.picks


def test_a_bare_pick_is_a_pick_and_not_half_an_instruction():
    o = TI.plan_from("pick up the leftmost blue cube", SEEN)
    assert o.kind == TI.PLAN, o.message
    assert o.hold_only, o.picks
    assert o.picks[0][2] is None


def test_an_ask_is_answerable_and_two_answers_give_two_plans():
    q = TI.plan_from("pick up the blue cube", SEEN)
    assert q.kind == TI.ASK and q.question
    left = TI.answer(q, "the leftmost", SEEN)
    right = TI.answer(q, "the rightmost", SEEN)
    assert left.kind == TI.PLAN and right.kind == TI.PLAN
    assert left.picks != right.picks, (left.picks, right.picks)


def test_an_answer_that_answers_nothing_asks_again_rather_than_guessing():
    q = TI.plan_from("pick up the blue cube", SEEN)
    again = TI.answer(q, "hmm", SEEN)
    assert again.kind == TI.ASK, (again.kind, again.message)
    assert again.question is not None


def test_no_cancels_and_nothing_moves():
    q = TI.plan_from("pick up the blue cube", SEEN)
    o = TI.answer(q, "never mind", SEEN)
    assert o.kind == TI.REFUSE and not o.picks


def test_tidy_up_offers_the_reading_and_does_not_assume_it():
    q = TI.plan_from("tidy up", SEEN)
    assert q.kind == TI.ASK and q.question["kind"] == "confirm"
    yes = TI.answer(q, "yes", SEEN)
    assert yes.kind == TI.PLAN and len(yes.picks) == len(SEEN)
    no = TI.answer(q, "no", SEEN)
    assert no.kind == TI.REFUSE and not no.picks


def test_a_superlative_anchored_on_the_PERSON_is_not_a_relational_reference():
    """"the cube closest to me" names no second object to be relative to.

    It is still refused here -- every T1 cube is in one row, so `nearest`
    separates nothing -- but it must be refused for THAT reason, not as a
    relational reference, or the operator is sent to rewrite a sentence that
    was already unambiguous.
    """
    o = TI.plan_from("grab the cube closest to me", SEEN)
    assert o.kind == TI.REFUSE
    assert "relational" not in o.message.lower(), o.message
    assert "row" in o.message, o.message
