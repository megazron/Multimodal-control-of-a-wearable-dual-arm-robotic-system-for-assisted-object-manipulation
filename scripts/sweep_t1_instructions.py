#!/usr/bin/env python3
"""T1 FROM A TYPED SENTENCE: correct / asked / refused / MISUNDERSTOOD.

    python3 scripts/sweep_t1_instructions.py [--json OUT] [--no-before]

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
misunderstandings by construction. So the blocks marked ADVERSARIAL below were
written from the PARSER'S STRUCTURE, hunting for sentences whose plain English
meaning the code cannot represent -- negation, a second target, a second
action, relational reference, stacking, a colour with no pad, an object not in
the scene, a question rather than a command, a selector no scene can settle --
and the MISSPELLING block was written to attack the repair rule itself,
including a tie the repair must refuse to break.

TWO CASES IN HERE FOUND REAL FAULTS ON THE DAY THEY WERE WRITTEN, which is the
evidence that the set is not being written to the code:

  * "put the blue ones on the blue pad and the green ones on the green pad"
    -- a complete compound. The shipped grammar planned the FIRST half and
    dropped the second in silence: two cubes moved of four, reported as a
    success. That is a MISUNDERSTOOD and it is the only one this project has
    recorded since the outcome was first counted.
  * "could you please just put the blue ones on the blue pad for me when you
    get a chance" -- politeness. `when` is one Damerau edit from `then`, the
    repair took it, and the sentence was refused as "more than one action in
    one instruction".

WHAT COUNTS AS "MEANT". Each case declares the exact set of (cube index, pad
index) pairs it intends, against the scene below. A plan that delivers the
right cubes to the wrong pad is MISUNDERSTOOD, not partially correct: the pad
IS the task. A pad index of None means a pick and a hold, which is what a bare
"pick up the blue cube" asks for and is not the same outcome as a placement.

THE ANSWER PATH IS SCORED SEPARATELY AND IT IS NOT COUNTED AS A CORRECT. A
case may carry a REPLY, and where it does, the ASK plus the reply is scored on
its own line. Rolling those into the CORRECT column would let a grammar that
asks about everything look like a grammar that understands everything.
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, "src/srl_experiments/experiments/abc"),
          os.path.join(ROOT, "src/srl_autonomy"),
          os.path.join(ROOT, "config")):
    if p not in sys.path:
        sys.path.insert(0, p)

import t1_instruction as TI                                  # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
import msc_clip_tasks as _MCT                                # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/t1_instruction_sweep.json")

# THE COMMIT THE "BEFORE" COLUMN COMES FROM. Not a stubbed-out feature flag:
# the two files are checked out of git into a temporary tree and the same
# cases are run against them in a separate interpreter, so the two columns
# differ by the code and by nothing else.
BEFORE_REF = "b782a9b"
BEFORE_FILES = (("src/srl_autonomy/srl_autonomy/voice_intent.py",
                 "srl_autonomy/voice_intent.py"),
                ("src/srl_autonomy/srl_autonomy/referring.py",
                 "srl_autonomy/referring.py"),
                ("src/srl_autonomy/srl_autonomy/__init__.py",
                 "srl_autonomy/__init__.py"),
                ("src/srl_experiments/experiments/abc/t1_instruction.py",
                 "t1_instruction.py"))

# THE SCENE, AS THE CAMERA WOULD REPORT IT, in the (x, y, pad_index) form
# `observe_and_detect` returns. Index 0 is the blue pad, 1 the green.
#
# TWO OF EACH COLOUR IS THE POINT. "the blue cube" is genuinely ambiguous in
# this scene, so a sweep that used one cube per colour would never exercise
# the singular/plural distinction and would report a higher CORRECT rate for
# an easier task.
#
# READ FROM THE TASK, not restated, so it cannot drift from the layout the run
# uses.
SEEN = [(cx, cy, _MCT.T1_PAIR[i])
        for i, (cx, cy) in enumerate(_MCT.T1_CUBES)]
BLUE, GREEN = 0, 1
BLUE_IDX = [i for i, c in enumerate(SEEN) if c[2] == BLUE]
GREEN_IDX = [i for i, c in enumerate(SEEN) if c[2] == GREEN]
BLUES = [(i, BLUE) for i in BLUE_IDX]
GREENS = [(i, GREEN) for i in GREEN_IDX]
BLUES_TO_GREEN = [(i, GREEN) for i in BLUE_IDX]
GREENS_TO_BLUE = [(i, BLUE) for i in GREEN_IDX]
ALL_TO_OWN = [(i, c[2]) for i, c in enumerate(SEEN)]

# +x is the LEFT arm's side of this layout, so "leftmost" is the largest x.
LEFTMOST_BLUE = max(BLUE_IDX, key=lambda i: SEEN[i][0])
RIGHTMOST_BLUE = min(BLUE_IDX, key=lambda i: SEEN[i][0])
LEFTMOST_GREEN = max(GREEN_IDX, key=lambda i: SEEN[i][0])
RIGHTMOST_GREEN = min(GREEN_IDX, key=lambda i: SEEN[i][0])
# "the first" counts along x from the far right, which is the order the plan
# is built in and the order the clip shows.
BY_X = sorted(range(len(SEEN)), key=lambda i: SEEN[i][0])

CORRECT, ASKED, REFUSED, MISUNDERSTOOD = \
    "CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"

HOLD = None            # a pad index of None is a pick and a hold


# (utterance, category, intended_pairs, note, reply, intended_after_reply)
def C(utt, cat, intended, note, reply=None, then=None):
    return dict(utterance=utt, category=cat, intended=intended, note=note,
                reply=reply, then=then)


CASES = [
    # ---- CONTROLS. If these fail the harness is wrong, not the code.
    C("put the blue ones on the blue pad", "control", BLUES,
      "the plainest form there is"),
    C("put the green ones on the green mat", "control", GREENS,
      "a synonym for the pad"),
    C("put every cube on the pad of its own colour", "control", ALL_TO_OWN,
      "the task as the spec words it"),

    # ---- THE SIX PHRASINGS THE BRIEF ASKED FOR, VERBATIM
    C("pick up the blue cube", "brief", None,
      "SINGULAR against two blue cubes -- must ask, not pick one",
      reply="the leftmost", then=[(LEFTMOST_BLUE, HOLD)]),
    C("pick up the green cube", "brief", None,
      "the same on the other arm",
      reply="the rightmost", then=[(RIGHTMOST_GREEN, HOLD)]),
    C("put the blue cubes on the blue pad", "brief", BLUES,
      "plural, named pad"),
    C("put the green ones where they belong", "brief", GREENS,
      "plural, destination named by a relation to the object"),
    C("move that block to its colour", "brief", None,
      "singular and deictic against four cubes -- must ask",
      reply="the blue one on the left", then=[(LEFTMOST_BLUE, BLUE)]),
    C("pick up all the cubes", "brief", None,
      "one hand, four cubes -- must ask rather than pick one of them. The "
      "reply is a NEW instruction rather than an answer, and it is itself "
      "plural-deictic, so it asks once more before it plans",
      reply=["put them on their own colours", "all of them"], then=ALL_TO_OWN),

    # ---- PARAPHRASE. Different words, same instruction.
    C("sort the cubes by colour", "paraphrase", ALL_TO_OWN, "no preposition"),
    C("put every cube where it belongs", "paraphrase", ALL_TO_OWN,
      "a place named by a relation to the object, not by a coordinate"),
    C("sort them onto their matching colours", "paraphrase", None,
      "'them' with nothing pointed at -- ask", reply="all of them",
      then=ALL_TO_OWN),
    C("move all the cubes to the colour that matches", "paraphrase",
      ALL_TO_OWN, "'all' plus a match phrase"),
    C("place each block on the matching pad", "paraphrase", ALL_TO_OWN,
      "'block' for cube, 'each' for all"),
    C("could you please put the blue ones onto the blue square", "paraphrase",
      BLUES, "politeness and a third word for the pad"),
    C("shift the green blocks over to the green target", "paraphrase",
      GREENS, "a verb and a noun neither of which is canonical"),
    C("deliver the green cubes to the green mat", "paraphrase", GREENS,
      "a verb the grammar knows and nobody uses for cubes"),
    C("stack the blue ones on the blue pad", "paraphrase", BLUES,
      "'stack' onto a PAD is a placement, not object-on-object stacking"),

    # ---- CROSS-COLOUR. The destination is NOT the cube's colour, and that
    # is a legitimate instruction that must not be quietly colour-matched.
    C("put the blue ones on the green pad", "cross-colour", BLUES_TO_GREEN,
      "if this comes back as the blue pad, the code is matching colours "
      "instead of reading the sentence"),
    C("put the green cubes on the blue mat", "cross-colour", GREENS_TO_BLUE,
      "the same, the other way round"),
    C("put all four cubes on the blue pad", "cross-colour",
      [(i, BLUE) for i in range(len(SEEN))],
      "every cube to ONE pad -- a colour-matcher would split them"),

    # ---- SELECTORS AND SUPERLATIVES, which the brief asked for by name
    C("put the leftmost blue cube on the blue pad", "superlative",
      [(LEFTMOST_BLUE, BLUE)], "a superlative that the scene can settle"),
    C("put the rightmost green cube on the green pad", "superlative",
      [(RIGHTMOST_GREEN, GREEN)], "the same on the other arm"),
    C("put the second green cube on the green mat", "ordinal",
      [(BY_X[1], GREEN)],
      "ordinals count along the row, nearest the far side first"),
    C("put the last cube on its own colour", "ordinal",
      [(BY_X[-1], SEEN[BY_X[-1]][2])], "'last' along the same order"),
    C("put either blue cube on the blue pad", "superlative",
      [(RIGHTMOST_BLUE, BLUE)],
      "'either' is permission, not ambiguity -- asking here asks a question "
      "the operator has already answered"),
    C("put the biggest cube on the blue pad", "superlative", None,
      "four identical 40 mm cubes -- a selector the scene CANNOT settle, so "
      "it must be refused by name and never dropped"),
    C("put the nearest blue cube on the blue pad", "superlative", None,
      "every cube is in one row, so 'nearest' separates nothing"),
    C("put the middle cube on the blue pad", "superlative", None,
      "there are four, so there is no middle one"),
    C("take the left blue one to the blue pad", "superlative",
      [(LEFTMOST_BLUE, BLUE)], "'the left one' without the -most"),
    C("grab the cube closest to me", "superlative", None,
      "a superlative anchored on the PERSON, not a relational reference to "
      "another object -- it must not be refused as relational"),

    # ---- COMPOUND. Two complete instructions in one sentence.
    C("put the blue ones on the blue pad and the green ones on the green pad",
      "compound", BLUES + GREENS,
      "BOTH halves complete. Planning one of them is partial execution "
      "reported as success -- the one MISUNDERSTOOD this set has found"),
    C("put the green cubes on the green pad and the blue cubes on the blue "
      "pad", "compound", GREENS + BLUES, "the same, uttered the other way up"),
    C("put the blue ones on the blue pad and the blue ones on the green pad",
      "compound", None,
      "the two halves send the same cubes to two different pads"),
    C("put the blue cube and the green cube on the pads", "two-targets", None,
      "ONE destination between two targets -- not a compound, and acting on "
      "either one is partial execution"),

    # ---- POLITENESS AND FILLER
    C("could you please just put the blue ones on the blue pad for me when "
      "you get a chance", "politeness", BLUES,
      "the whole sentence is filler except eight words. 'when' is one edit "
      "from 'then' and the repair used to take it"),
    C("hey, would you mind putting the green ones on the green pad, thanks",
      "politeness", GREENS, "filler either side of the instruction"),
    C("ok so I would like you to sort the cubes by colour please",
      "politeness", ALL_TO_OWN, "a preamble and a trailing please"),

    # ---- VAGUE. Under-specified rather than wrong.
    C("put it on the blue pad", "vague", None, "'it' with nothing held",
      reply="the leftmost blue one", then=[(LEFTMOST_BLUE, BLUE)]),
    C("tidy up", "vague", None,
      "an instruction with an obvious reading and no stated one -- offer the "
      "reading back, never assume it", reply="yes", then=ALL_TO_OWN),
    C("sort everything out", "vague", None, "the same, other words",
      reply="yes", then=ALL_TO_OWN),
    C("put that there", "vague", None, "two deictics and no colour"),
    C("do the cube thing", "vague", None, "not an instruction"),
    C("put the cube on the pad", "vague", None,
      "which cube, and which pad -- four cubes and two pads. TWO questions, "
      "so two turns, and the dialogue has to survive that",
      reply=["both blue ones", "the blue pad"], then=BLUES),
    C("just do something useful", "vague", None, "no verb this grammar knows"),

    # ---- ADVERSARIAL, written from the parser's structure
    C("do not put the blue ones on the blue pad", "negation", None,
      "negated -- acting on it is the worst outcome available"),
    C("put the blue ones on the blue pad but not the green ones", "negation",
      BLUES,
      "'not' AFTER the verb is an exclusion clause on a target already named"),
    C("never put the green cubes on the blue pad", "negation", None,
      "a different negation word in the same position"),
    C("put the blue ones on the blue pad then go home", "two-actions", None,
      "two actions -- doing the first is partial execution"),
    C("put the cube behind the blue one on the blue pad", "relational", None,
      "relational reference, which this rig cannot ground"),
    C("put the blue cube on top of the green cube", "stacking", None,
      "stacking, which is not a pad and is not this task"),
    C("put the red ones on the red pad", "not-in-scene", None,
      "no red cubes and no red pad"),
    C("put the blue ones on the red pad", "not-in-scene", None,
      "the cubes exist, the pad does not"),
    C("put the tray on the blue pad", "not-in-scene", None,
      "a noun the grammar knows and the scene does not have"),
    C("put the yellow cube where it belongs", "not-in-scene", None,
      "a colour with neither a cube nor a pad"),
    C("which cubes are blue", "question", None, "a question, not a command"),
    C("how many cubes can you see", "question", None, "another question"),
    C("the blue cubes go on the blue pad", "statement", None,
      "a statement of fact with no verb of instruction"),
    C("stop", "stop", None, "a halt, never a pick and place"),
    C("put the blue ones on the blue pad, stop", "stop", None,
      "a halt anywhere in the sentence wins"),
    C("put the blue ones on the left pad", "unknown-place", None,
      "'left' is not a pad colour"),
    C("hand the blue cube to the other arm", "not-possible", None,
      "arm-to-arm transfer, measured impossible on this rig"),
    C("give me the blue cube", "not-possible", None,
      "handing to the wearer is T5 and is not this task"),

    # ---- BARE PICK. A whole instruction, not half of one.
    C("pick up the leftmost blue cube", "bare-pick",
      [(LEFTMOST_BLUE, HOLD)],
      "a pick with no destination is complete. Adding 'and put it on its own "
      "colour' would be a silent completion"),
    C("grab the rightmost green block", "bare-pick",
      [(RIGHTMOST_GREEN, HOLD)], "the same with other words"),
    C("lift the second cube", "bare-pick", [(BY_X[1], HOLD)],
      "an ordinal with no colour"),

    # ---- MISSPELLINGS, written to attack the repair rule
    C("put the bleu ones on the bleu pad", "misspelling", BLUES,
      "transposition, one edit, unambiguous"),
    C("put the blue cubbes on the blue pad", "misspelling", BLUES,
      "doubled letter in the noun"),
    C("put the gren ones on the gren pad", "misspelling", None,
      "'gren' is one edit from BOTH green and grey -- a tie, which must be "
      "asked about and never broken", reply="green", then=GREENS),
    C("put the gerne ones on the gerne pad", "misspelling", None,
      "two edits from green -- outside the repair rule, so refuse"),
    C("pt the blue ones on the blue pad", "misspelling", None,
      "the VERB is misspelled, and verbs are never fuzzy-matched"),
    C("put the blue ones on the blue padd", "misspelling", BLUES,
      "doubled letter in the surface noun"),
    C("sort teh cubes by colour", "misspelling", ALL_TO_OWN,
      "a filler word misspelled -- must not change the meaning"),
    C("put the blue ones on the blue pad wen you are ready", "misspelling",
      BLUES,
      "'wen' is one edit from 'when' AND from 'hen'. It must not become "
      "'then', which would refuse a perfectly ordinary sentence"),
    C("put the lefmost blue cube on the blue pad", "misspelling",
      [(LEFTMOST_BLUE, BLUE)], "a misspelled SELECTOR, one edit"),
]


def _pairs(outcome):
    """A plan -> {(cube_index, pad_index)}, matched back to the scene by x."""
    out = set()
    for px, py, pad in outcome.picks:
        idx = min(range(len(SEEN)), key=lambda i: abs(SEEN[i][0] - px))
        out.add((idx, None if pad is None else int(pad)))
    return out


def score_outcome(o, intended):
    if o.kind == TI.ASK:
        return ASKED, ""
    if o.kind == TI.REFUSE:
        return REFUSED, ""
    got = _pairs(o)
    if intended is None:
        return MISUNDERSTOOD, (
            "acted on an instruction whose only safe outcomes were ask or "
            "refuse; it moved %s" % sorted(got, key=str))
    want = set(tuple(p) for p in intended)
    if got == want:
        return CORRECT, ""
    return MISUNDERSTOOD, ("meant %s, planned %s"
                           % (sorted(want, key=str), sorted(got, key=str)))


def score(case):
    o = TI.plan_from(case["utterance"], SEEN)
    verdict, why = score_outcome(o, case["intended"])
    reply = None
    if case.get("reply") is not None:
        # A REPLY MAY BE A LIST. Some questions have two halves -- "put the
        # cube on the pad" does not say which cube OR which pad -- and a
        # dialogue that can only take one turn would score those as failures
        # of understanding when they are just two questions.
        turns = case["reply"] if isinstance(case["reply"], (list, tuple)) \
            else [case["reply"]]
        if verdict != ASKED:
            reply = dict(text=" / ".join(turns), verdict="NOT ASKED",
                         why="the reply was written for an ASK and the "
                             "instruction did not ask", message="")
        else:
            o2, said = o, []
            for t in turns:
                said.append(t)
                o2 = TI.answer(o2, t, SEEN)
                if o2.kind != TI.ASK:
                    break
            v2, w2 = score_outcome(o2, case["then"])
            reply = dict(text=" / ".join(said), turns=len(said), verdict=v2,
                         why=w2, message=o2.message,
                         plan=[list(p) for p in o2.picks])
    return verdict, o, why, reply


def _before(tmp):
    """The same cases against the grammar at BEFORE_REF, in its own process."""
    pkg = os.path.join(tmp, "srl_autonomy")
    os.makedirs(pkg, exist_ok=True)
    for src, dst in BEFORE_FILES:
        try:
            blob = subprocess.check_output(
                ["git", "show", "%s:%s" % (BEFORE_REF, src)], cwd=ROOT)
        except subprocess.CalledProcessError:
            return None
        with open(os.path.join(tmp, dst), "wb") as fh:
            fh.write(blob)
    runner = os.path.join(tmp, "run_before.py")
    with open(runner, "w") as fh:
        fh.write(
            "import json, sys\n"
            # ORDER MATTERS AND IT IS THE WHOLE EXPERIMENT. The temporary tree
            # must be searched FIRST or `import t1_instruction` finds the
            # CURRENT one and the before column comes back identical to the
            # after column -- which is exactly what happened the first time
            # this was run, and is indistinguishable from "the change did
            # nothing" unless you look.
            "sys.path.insert(0, %r)\n"
            "sys.path.insert(0, %r)\n"
            "sys.path.insert(0, %r)\n"
            "WHERE = %r\n"
            "import t1_instruction as TI\n"
            # NOT `sys.path[0]`: importing the old t1_instruction pushes its
            # own paths onto the front, so by the time this line runs sys.path
            # has moved under it. Compare against the tree we wrote.
            "assert TI.__file__.startswith(WHERE), TI.__file__\n"
            "cases = json.load(open(%r))\n"
            "seen = json.load(open(%r))\n"
            "out = []\n"
            "for c in cases:\n"
            "    try:\n"
            "        o = TI.plan_from(c, [tuple(s) for s in seen])\n"
            "        out.append([o.kind, [list(p) for p in o.picks]])\n"
            "    except Exception as e:\n"
            "        out.append(['refuse', []])\n"
            "print(json.dumps(out))\n"
            % (os.path.join(ROOT, "config"),
               os.path.join(ROOT, "src/srl_experiments/experiments/abc"), tmp,
               tmp,
               os.path.join(tmp, "cases.json"), os.path.join(tmp, "seen.json")))
    json.dump([c["utterance"] for c in CASES],
              open(os.path.join(tmp, "cases.json"), "w"))
    json.dump([list(s) for s in SEEN], open(os.path.join(tmp, "seen.json"), "w"))
    try:
        raw = subprocess.check_output([sys.executable, runner], cwd=tmp,
                                      stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        return None
    got = json.loads(raw.decode())
    tally = {CORRECT: 0, ASKED: 0, REFUSED: 0, MISUNDERSTOOD: 0}
    rows = []
    for case, (kind, picks) in zip(CASES, got):
        fake = TI.Outcome({"plan": TI.PLAN, "ask": TI.ASK,
                           "refuse": TI.REFUSE}[kind],
                          [tuple(p) for p in picks])
        v, why = score_outcome(fake, case["intended"])
        tally[v] += 1
        rows.append(dict(utterance=case["utterance"], verdict=v, why=why))
    return dict(ref=BEFORE_REF, totals=tally, rows=rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=OUT)
    ap.add_argument("--no-before", action="store_true",
                    help="skip the A/B against the committed grammar")
    a = ap.parse_args()

    # CONTROLS ON THE HARNESS ITSELF, run before anything is reported. A sweep
    # that cannot detect a wrong plan cannot certify a right one.
    controls = {}
    ok = TI.plan_from("put the blue ones on the blue pad", SEEN)
    controls["a_plain_instruction_plans"] = ok.ok
    controls["it_plans_the_blue_cubes"] = _pairs(ok) == set(BLUES)
    wrong = TI.plan_from("put the blue ones on the green pad", SEEN)
    controls["a_different_pad_gives_a_different_plan"] = (
        wrong.ok and _pairs(wrong) != _pairs(ok))
    controls["an_empty_scene_refuses"] = (
        TI.plan_from("put the blue ones on the blue pad", []).kind == TI.REFUSE)
    fake = TI.Outcome(TI.PLAN, [(SEEN[0][0], SEEN[0][1], GREEN)])
    controls["the_scorer_can_report_misunderstood"] = (
        _pairs(fake) != {(BLUE_IDX[0], BLUE)})
    # AND THAT AN ANSWER CAN CHANGE THE ANSWER. A dialogue that returns the
    # same thing whatever is replied is not a dialogue.
    q = TI.plan_from("pick up the blue cube", SEEN)
    l = TI.answer(q, "the leftmost", SEEN)
    r = TI.answer(q, "the rightmost", SEEN)
    controls["two_replies_give_two_plans"] = (
        l.ok and r.ok and _pairs(l) != _pairs(r))
    bad = [k for k, v in controls.items() if not v]
    if bad:
        print("HARNESS CONTROL FAILED: %s -- reporting nothing" % ", ".join(bad))
        return 3

    before = None
    if not a.no_before:
        tmp = tempfile.mkdtemp(prefix="t1grammar_")
        before = _before(tmp)

    rows, tally = [], {CORRECT: 0, ASKED: 0, REFUSED: 0, MISUNDERSTOOD: 0}
    replies = {CORRECT: 0, ASKED: 0, REFUSED: 0, MISUNDERSTOOD: 0,
               "NOT ASKED": 0}
    by_cat = {}
    for case in CASES:
        verdict, o, why, reply = score(case)
        tally[verdict] += 1
        if reply:
            replies[reply["verdict"]] = replies.get(reply["verdict"], 0) + 1
        by_cat.setdefault(case["category"], []).append(verdict)
        rows.append(dict(utterance=case["utterance"], category=case["category"],
                         intended=case["intended"], note=case["note"],
                         verdict=verdict, message=o.message,
                         why=why, reply=reply,
                         plan=[list(p) for p in o.picks],
                         question=o.question,
                         intent=None if o.intent is None else o.intent.as_dict()))

    last = None
    for r in rows:
        if r["category"] != last:
            print("\n-- %s" % r["category"].upper())
            last = r["category"]
        print("  %-58s %-14s %s"
              % (r["utterance"][:58], r["verdict"],
                 r["why"] or r["message"][:58]))
        if r["reply"]:
            print("        reply %-46r %-8s %s"
                  % (r["reply"]["text"][:46], r["reply"]["verdict"],
                     r["reply"]["why"] or r["reply"]["message"][:50]))

    n = len(rows)
    print("\n" + "=" * 78)
    print("TOTALS over %d phrasings" % n)
    print("=" * 78)
    if before:
        print("  %-14s %10s %8s" % ("", "at " + BEFORE_REF, "now"))
        for k in (CORRECT, ASKED, REFUSED, MISUNDERSTOOD):
            print("  %-14s %10d %8d   (%.1f%%)"
                  % (k, before["totals"][k], tally[k], 100.0 * tally[k] / n))
        print("\n  'at %s' is the SAME cases run against the grammar and the "
              "grounding\n  layer as committed, checked out of git into a "
              "temporary tree and run in\n  its own interpreter. The two "
              "columns differ by the code and by nothing\n  else." % BEFORE_REF)
    else:
        for k in (CORRECT, ASKED, REFUSED, MISUNDERSTOOD):
            print("  %-14s %8d   (%.1f%%)" % (k, tally[k], 100.0 * tally[k] / n))

    nrep = sum(1 for c in CASES if c.get("reply") is not None)
    if nrep:
        turns = sum((r["reply"] or {}).get("turns", 0) for r in rows)
        print("\n  OF THE %d THAT WERE ANSWERED, IN %d REPLIES:"
              % (nrep, turns))
        for k in (CORRECT, ASKED, REFUSED, MISUNDERSTOOD, "NOT ASKED"):
            if replies.get(k):
                print("     %-14s %d" % (k, replies[k]))
        print("  These are NOT counted as CORRECT above. A grammar that asks "
              "about\n  everything must not be able to look like one that "
              "understands everything.")

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
                   controls=controls, totals=tally, replies=replies,
                   before=before, rows=rows,
                   by_category={k: {v: vs.count(v) for v in set(vs)}
                                for k, vs in by_cat.items()},
                   caveat="grounding only; the scene is constructed, so this "
                          "says nothing about detection rate"),
              open(a.json, "w"), indent=1)
    print("\n  -> %s" % a.json)
    return 1 if tally[MISUNDERSTOOD] else 0


if __name__ == "__main__":
    raise SystemExit(main())
