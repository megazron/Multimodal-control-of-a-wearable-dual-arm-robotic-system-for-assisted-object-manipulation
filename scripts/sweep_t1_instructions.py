#!/usr/bin/env python3
"""T1 FROM A TYPED SENTENCE: correct / asked / refused / MISUNDERSTOOD.

    python3 scripts/sweep_t1_instructions.py [--json OUT]

WHAT IS BEING MEASURED, AND WHAT IS NOT. This scores GROUNDING: given a scene
the code already knows the truth of, does a typed instruction resolve to the
cubes and pads the writer meant. It says nothing about the DETECTOR -- the
scene here is arithmetic, so the ground truth is constructed rather than
rendered, per the standing rule. Whether the camera can see four cubes at
working distance is a different measurement and lives in
`scripts/verify_look_then_grasp.py` and `scripts/verify_colour_vision.py`.

THE FOUR OUTCOMES

  CORRECT       resolved to exactly the cubes and pads the writer meant
  ASKED         resolved to more than one possibility and chose NOTHING
  REFUSED       declined, and said why
  MISUNDERSTOOD resolved CONFIDENTLY to something else -- a different cube, a
                different pad, or a dropped constraint that changes which
                object moves

ASKED and REFUSED are successes. The arm did not move and the operator was
told. MISUNDERSTOOD is the only dangerous outcome and it is reported on its
own line, because a summary that averages it away is a summary that hides it.

HOW THE CASES WERE WRITTEN, WHICH IS THE PART THAT DECIDES WHETHER THE NUMBER
MEANS ANYTHING. `scripts/sweep_language_vision.py` records the lesson: a
phrase set written alongside the grammar it tests reports zero
misunderstandings by construction. So the block marked ADVERSARIAL below was
written from the PARSER'S STRUCTURE, hunting for sentences whose plain English
meaning the code cannot represent -- negation, a second target, a second
action, relational reference, stacking, a colour with no pad, an object not in
the scene, a question rather than a command -- and the MISSPELLING block was
written to attack the repair rule itself, including a tie the repair must
refuse to break.

WHAT COUNTS AS "MEANT". Each case declares the exact set of (cube index, pad
index) pairs it intends, against the scene below. A plan that delivers the
right cubes to the wrong pad is MISUNDERSTOOD, not partially correct: the pad
IS the task.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "src/srl_autonomy"),
          os.path.join(ROOT, "config")):
    if p not in sys.path:
        sys.path.insert(0, p)

import t1_instruction as TI                                  # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_instruction_sweep.json")

# THE SCENE, AS THE CAMERA WOULD REPORT IT. Four cubes in a row, colours
# alternating, in the (x, y, pad_index) form `observe_and_detect` returns.
# Index 0 is the blue pad, 1 the green -- MCT.PLANE_COLOURS.
#
# TWO OF EACH COLOUR IS THE POINT. "the blue cube" is genuinely ambiguous in
# this scene, so a sweep that used one cube per colour would never exercise
# the singular/plural distinction and would report a higher CORRECT rate for
# an easier task.
SEEN = [(0.560, 0.120, 0),      # cube 0, BLUE
        (0.620, 0.120, 1),      # cube 1, GREEN
        (0.680, 0.120, 0),      # cube 2, BLUE
        (0.740, 0.120, 1)]      # cube 3, GREEN
BLUE, GREEN = 0, 1
ALL_TO_OWN = [(0, BLUE), (1, GREEN), (2, BLUE), (3, GREEN)]

CORRECT, ASKED, REFUSED, MISUNDERSTOOD = \
    "CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"

# (utterance, category, intended_pairs, note)
#   intended_pairs is a list of (cube_index, pad_index)
#   None            -> the only safe outcomes are ASKED or REFUSED
CASES = [
    # ---- CONTROLS. If these fail the harness is wrong, not the code.
    ("put the blue ones on the blue pad", "control",
     [(0, BLUE), (2, BLUE)], "the plainest form there is"),
    ("put the green ones on the green mat", "control",
     [(1, GREEN), (3, GREEN)], "a synonym for the pad"),
    ("put every cube on the pad of its own colour", "control",
     ALL_TO_OWN, "the task as the spec words it"),

    # ---- THE THREE PHRASINGS THE BRIEF ASKED FOR, VERBATIM
    ("pick up the blue cube and put it on the blue pad", "brief", None,
     "SINGULAR against two blue cubes -- must ask, not pick one"),
    ("put the green ones on the green mat", "brief",
     [(1, GREEN), (3, GREEN)], "plural, synonym destination"),
    ("move that blue block to its colour", "brief", None,
     "singular and deictic against two blue cubes -- must ask"),

    # ---- PARAPHRASE. Different words, same instruction.
    ("sort the cubes by colour", "paraphrase", ALL_TO_OWN, "no preposition"),
    ("put every cube where it belongs", "paraphrase", ALL_TO_OWN,
     "a place named by a relation to the object, not by a coordinate"),
    ("sort them onto their matching colours", "paraphrase", None,
     "'them' with nothing pointed at -- ask"),
    ("move all the cubes to the colour that matches", "paraphrase",
     ALL_TO_OWN, "'all' plus a match phrase"),
    ("place each block on the matching pad", "paraphrase", ALL_TO_OWN,
     "'block' for cube, 'each' for all"),
    ("could you please put the blue ones onto the blue square", "paraphrase",
     [(0, BLUE), (2, BLUE)], "politeness and a third word for the pad"),
    ("shift the green blocks over to the green target", "paraphrase",
     [(1, GREEN), (3, GREEN)], "a verb and a noun neither of which is canonical"),

    # ---- CROSS-COLOUR. The destination is NOT the cube's colour, and that
    # is a legitimate instruction that must not be quietly colour-matched.
    ("put the blue ones on the green pad", "cross-colour",
     [(0, GREEN), (2, GREEN)],
     "if this comes back as the blue pad, the code is matching colours "
     "instead of reading the sentence"),
    ("put the green cubes on the blue mat", "cross-colour",
     [(1, BLUE), (3, BLUE)], "the same, the other way round"),

    # ---- VAGUE. Under-specified rather than wrong.
    ("put it on the blue pad", "vague", None, "'it' with nothing held"),
    ("tidy up", "vague", None, "no verb this grammar knows"),
    ("put that there", "vague", None, "two deictics and no colour"),
    ("do the cube thing", "vague", None, "not an instruction"),
    ("put the cube on the pad", "vague", None,
     "which cube, and which pad -- four cubes and two pads"),

    # ---- ADVERSARIAL, written from the parser's structure
    ("do not put the blue ones on the blue pad", "negation", None,
     "negated -- acting on it is the worst outcome available"),
    ("put the blue ones on the blue pad but not the green ones", "negation",
     [(0, BLUE), (2, BLUE)],
     "'not' AFTER the verb is an exclusion clause on a target already named"),
    ("put the blue cube and the green cube on the pads", "two-targets", None,
     "two targets -- doing one of them is partial execution"),
    ("put the blue ones on the blue pad then go home", "two-actions", None,
     "two actions -- doing the first is partial execution"),
    ("put the cube behind the blue one on the blue pad", "relational", None,
     "relational reference, which this rig cannot ground"),
    ("put the blue cube on top of the green cube", "stacking", None,
     "stacking, which is not a pad and is not this task"),
    ("put the red ones on the red pad", "not-in-scene", None,
     "no red cubes and no red pad"),
    ("put the blue ones on the red pad", "not-in-scene", None,
     "the cubes exist, the pad does not"),
    ("put the tray on the blue pad", "not-in-scene", None,
     "a noun the grammar knows and the scene does not have"),
    ("which cubes are blue", "question", None, "a question, not a command"),
    ("the blue cubes go on the blue pad", "statement", None,
     "a statement of fact with no verb of instruction"),
    ("stop", "stop", None, "a halt, never a pick and place"),
    ("put the blue ones on the left pad", "unknown-place", None,
     "'left' is not a pad colour"),

    # ---- MISSPELLINGS, written to attack the repair rule
    ("put the bleu ones on the bleu pad", "misspelling",
     [(0, BLUE), (2, BLUE)], "transposition, one edit, unambiguous"),
    ("put the blue cubbes on the blue pad", "misspelling",
     [(0, BLUE), (2, BLUE)], "doubled letter in the noun"),
    ("put the gren ones on the gren pad", "misspelling", None,
     "'gren' is one edit from BOTH green and grey -- a tie, which must be "
     "asked about and never broken"),
    ("put the gerne ones on the gerne pad", "misspelling", None,
     "two edits from green -- outside the repair rule, so refuse"),
    ("pt the blue ones on the blue pad", "misspelling", None,
     "the VERB is misspelled, and verbs are never fuzzy-matched"),
    ("put the blue ones on the blue padd", "misspelling",
     [(0, BLUE), (2, BLUE)], "doubled letter in the surface noun"),
    ("sort teh cubes by colour", "misspelling", ALL_TO_OWN,
     "a filler word misspelled -- must not change the meaning"),
]


def _pairs(outcome):
    """A plan -> {(cube_index, pad_index)}, matched back to the scene by x."""
    out = set()
    for px, py, pad in outcome.picks:
        idx = min(range(len(SEEN)), key=lambda i: abs(SEEN[i][0] - px))
        out.add((idx, int(pad)))
    return out


def score(case):
    utt, cat, intended, note = case
    o = TI.plan_from(utt, SEEN)
    if o.kind == TI.ASK:
        return ASKED, o, ""
    if o.kind == TI.REFUSE:
        return REFUSED, o, ""
    got = _pairs(o)
    if intended is None:
        return MISUNDERSTOOD, o, (
            "acted on an instruction whose only safe outcomes were ask or "
            "refuse; it moved %s" % sorted(got))
    want = set(intended)
    if got == want:
        return CORRECT, o, ""
    return MISUNDERSTOOD, o, ("meant %s, planned %s"
                              % (sorted(want), sorted(got)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=OUT)
    a = ap.parse_args()

    # CONTROLS ON THE HARNESS ITSELF, run before anything is reported. A sweep
    # that cannot detect a wrong plan cannot certify a right one.
    controls = {}
    ok = TI.plan_from("put the blue ones on the blue pad", SEEN)
    controls["a_plain_instruction_plans"] = ok.ok
    controls["it_plans_the_blue_cubes"] = _pairs(ok) == {(0, BLUE), (2, BLUE)}
    wrong = TI.plan_from("put the blue ones on the green pad", SEEN)
    controls["a_different_pad_gives_a_different_plan"] = (
        wrong.ok and _pairs(wrong) != _pairs(ok))
    controls["an_empty_scene_refuses"] = (
        TI.plan_from("put the blue ones on the blue pad", []).kind == TI.REFUSE)
    # the scorer must be able to SAY misunderstood
    fake = TI.Outcome(TI.PLAN, [(0.560, 0.120, GREEN)])
    controls["the_scorer_can_report_misunderstood"] = (
        _pairs(fake) != {(0, BLUE)})
    bad = [k for k, v in controls.items() if not v]
    if bad:
        print("HARNESS CONTROL FAILED: %s -- reporting nothing" % ", ".join(bad))
        return 3

    # THE SAME 40 CASES AGAINST THE SHIPPED GRAMMAR, so the numbers below can
    # be read as a DELTA rather than as an absolute nobody can calibrate.
    # `put_on` is removed and nothing else changes, which is the same
    # before/after construction `sweep_language_vision.py` uses.
    import srl_autonomy.voice_intent as _vi
    _saved = _vi.VERB_PATTERNS
    _vi.VERB_PATTERNS = tuple(v for v in _saved if v[0] != "put_on")
    before = {CORRECT: 0, ASKED: 0, REFUSED: 0, MISUNDERSTOOD: 0}
    try:
        for case in CASES:
            before[score(case)[0]] += 1
    finally:
        _vi.VERB_PATTERNS = _saved

    rows, tally = [], {CORRECT: 0, ASKED: 0, REFUSED: 0, MISUNDERSTOOD: 0}
    by_cat = {}
    for case in CASES:
        verdict, o, why = score(case)
        tally[verdict] += 1
        by_cat.setdefault(case[1], []).append(verdict)
        rows.append(dict(utterance=case[0], category=case[1],
                         intended=case[2], note=case[3],
                         verdict=verdict, message=o.message,
                         why=why, plan=[list(p) for p in o.picks],
                         intent=None if o.intent is None else o.intent.as_dict()))

    last = None
    for r in rows:
        if r["category"] != last:
            print("\n-- %s" % r["category"].upper())
            last = r["category"]
        print("  %-56s %-14s %s"
              % (r["utterance"][:56], r["verdict"],
                 r["why"] or r["message"][:60]))

    n = len(rows)
    print("\n" + "=" * 78)
    print("TOTALS over %d phrasings" % n)
    print("=" * 78)
    print("  %-14s %8s %8s" % ("", "shipped", "now"))
    for k in (CORRECT, ASKED, REFUSED, MISUNDERSTOOD):
        print("  %-14s %8d %8d   (%.1f%%)"
              % (k, before[k], tally[k], 100.0 * tally[k] / n))
    print("\n  'shipped' is these same 40 cases with the put_on verb removed "
          "and\n  nothing else changed, so the two columns differ by the "
          "feature and\n  by nothing else.")
    print("\n  ASKED and REFUSED are SAFE -- the arm did not move and the "
          "operator was told.")
    if tally[MISUNDERSTOOD]:
        print("\n" + "!" * 78)
        print("%d MISUNDERSTOOD. These are the dangerous ones:"
              % tally[MISUNDERSTOOD])
        for r in rows:
            if r["verdict"] == MISUNDERSTOOD:
                print("   %-52s %s" % (r["utterance"][:52], r["why"]))
        print("!" * 78)
    else:
        print("\n" + "!" * 78)
        print("NO MISUNDERSTANDINGS.")
        print("!" * 78)

    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(dict(scene=[list(s) for s in SEEN],
                   pads=list(MCT.PLANE_COLOURS),
                   controls=controls, totals=tally,
                   totals_without_put_on=before, rows=rows,
                   caveat="grounding only; the scene is constructed, so this "
                          "says nothing about detection rate"),
              open(a.json, "w"), indent=1)
    print("\n  -> %s" % a.json)
    return 1 if tally[MISUNDERSTOOD] else 0


if __name__ == "__main__":
    raise SystemExit(main())
