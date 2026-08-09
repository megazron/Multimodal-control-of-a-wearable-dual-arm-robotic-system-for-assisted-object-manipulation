#!/usr/bin/env python3
"""PART 3.1 -- THE LANGUAGE AND VISION SWEEP, INCLUDING PHRASINGS NOBODY DESIGNED FOR.

WHY A SECOND SWEEP EXISTS. `measure_freeform_language.py` already reports
17 CORRECT / 7 ASKED / 6 REFUSED / 0 MISUNDERSTOOD over 30 phrases. Every one
of those phrases was written alongside the grammar it tests, so zero
misunderstandings is the expected output of a self-test, not evidence of
safety. That is recurring bug class 5 -- a test that constructs the
environment where the bug cannot occur.

This sweep is built the other way round: from the PARSER'S STRUCTURE, hunting
for inputs whose plain English meaning the shipped code cannot represent.
It deliberately includes negation, relational reference, compounds,
superlatives, questions, misspellings and out-of-scene objects.

THE FOUR OUTCOMES, and why MISUNDERSTOOD is the only dangerous one
-----------------------------------------------------------------
  CORRECT       the robot resolved what the speaker meant
  ASKED         ambiguous; the robot asks and picks NOTHING       (safe)
  REFUSED       the robot declines and says why                   (safe)
  MISUNDERSTOOD the robot confidently resolves something the speaker did NOT
                mean -- a different verb, a different object, or a dropped
                constraint that changes WHICH object gets picked up.

ASKED and REFUSED are successes. A robot bolted to a person that says "I don't
understand" has lost nothing. One that grabs the wrong object because it
silently discarded half the sentence has.

THE SCENE IS CONSTRUCTED, NOT RENDERED. Object labels and positions are
arithmetic, so ground truth is exact -- per the standing rule, synthetic input
is only trustworthy where the truth is constructed rather than drawn. This
sweep says nothing about the DETECTOR; it measures grounding given a known
scene. Real detection at working distance remains UNMEASURED.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
from srl_autonomy import voice_intent as vi                  # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/language_vision_sweep.json")
WAKE = vi.WAKE_DEFAULT

# --------------------------------------------------------------- the scene
# +x is the wearer's RIGHT, +y is FORWARD. Two red objects on purpose (so
# "the red one" is genuinely ambiguous) and exactly one blue cube.
SCENE = [
    dict(label="red block",  confidence=0.94, position=(-0.30, 0.40, 1.10)),
    dict(label="red cube",   confidence=0.91, position=(0.30, 0.40, 1.10)),
    dict(label="blue cube",  confidence=0.93, position=(0.05, 0.55, 1.10)),
    dict(label="green ball", confidence=0.88, position=(-0.10, 0.35, 1.10)),
]
PRESENT = {o["label"] for o in SCENE}

# (utterance, category, intended_verb, intended_label_or_None, note)
#   intended_label None  -> the ONLY safe outcomes are ASKED or REFUSED
#   intended_verb  None  -> the utterance is not a command at all
W = "hey doc oc"
CASES = [
    # ---- canonical controls. If these fail, the harness is wrong, not the code
    (f"{W} grab the blue cube", "control", "grab", "blue cube", "canonical"),
    (f"{W} pick up the green ball", "control", "grab", "green ball", "canonical"),

    # ---- polite / filler / disfluency
    (f"um {W} could you please uh grab the blue cube now", "polite-filler",
     "grab", "blue cube", "every filler word is stripped"),
    (f"{W} so like maybe grab the green ball please", "polite-filler",
     "grab", "green ball", "unlisted fillers 'so like maybe'"),
    (f"{W} would you mind terribly grabbing the blue cube", "polite-filler",
     "grab", "blue cube", "gerund 'grabbing', not the bare verb"),

    # ---- NEGATION. The parser strips punctuation and has no negation rule.
    (f"{W} don't grab the blue cube", "negation", None, None,
     "NEGATED command -- must not act"),
    (f"{W} do not pick up the red block", "negation", None, None,
     "NEGATED command -- must not act"),
    (f"{W} never grab the green ball", "negation", None, None,
     "NEGATED command -- must not act"),
    (f"{W} I don't want you to take the blue cube", "negation", None, None,
     "NEGATED command -- must not act"),

    # ---- RELATIONAL. The relation names a DIFFERENT object from the target.
    (f"{W} grab the cube to the left of the red block", "relational",
     "grab", None, "relation ignored; 'red' belongs to the LANDMARK"),
    (f"{W} pick up the ball next to the blue cube", "relational",
     "grab", "green ball", "only one ball, so it may resolve by luck"),
    (f"{W} take the object behind the red cube", "relational", "grab", None,
     "relation unrepresentable"),
    (f"{W} grab the one that isn't red", "relational", "grab", None,
     "negated attribute"),

    # ---- SUPERLATIVE
    (f"{W} grab the biggest cube", "superlative", "grab", None,
     "size not in the vocabulary; two cubes -> must ask"),
    (f"{W} pick up the closest red thing", "superlative", "grab", None,
     "two red objects; 'closest' unhandled at parse"),
    (f"{W} take the leftmost object", "superlative", "grab", None,
     "no colour/noun to ground"),

    # ---- COMPOUND
    (f"{W} grab the blue cube and the green ball", "compound", "grab", None,
     "TWO targets; acting on one silently is partial execution"),
    (f"{W} pick up the red block then put it down", "compound", "grab",
     "red block", "sequence; only the first verb is representable"),
    (f"{W} grab the blue cube but not the red one", "compound", "grab",
     "blue cube", "exclusion clause"),

    # ---- MISSPELLING / ASR ERROR, split by WHICH WORD is misspelled.
    # The split is the point: this module's stated design closes the VERB and
    # opens the NOUN, so the two halves should fail differently. They do.
    (f"{W} grabb the blue cube", "misspelled-verb", "grab", "blue cube",
     "doubled consonant in the VERB"),
    (f"{W} pik up the green ball", "misspelled-verb", "grab", "green ball",
     "clipped VERB"),
    (f"{W} garb the blue cube", "misspelled-verb", "grab", "blue cube",
     "transposed VERB"),
    (f"{W} grab the blu cube", "misspelled-noun", "grab", None,
     "misspelled COLOUR -- 'cube' still matches two objects, so ASK"),
    (f"{W} pick up the green bal", "misspelled-noun", "grab", "green ball",
     "misspelled NOUN -- 'green' alone still picks out one object"),
    (f"{W} take the gren ball", "misspelled-noun", "grab", "green ball",
     "misspelled COLOUR, noun intact"),

    # ---- OBJECT NOT PRESENT
    (f"{W} grab the yellow cube", "absent", "grab", None,
     "not in the scene -- must refuse"),
    (f"{W} pick up the purple bottle", "absent", "grab", None,
     "not in the scene -- must refuse"),
    (f"{W} take the screwdriver", "absent", "grab", None,
     "noun outside the vocabulary"),

    # ---- VAGUE / DEICTIC with no pointing input
    (f"{W} grab that", "vague", "grab", None, "deictic, no pointing channel"),
    (f"{W} pick up the thing over there", "vague", "grab", None, "deictic"),
    (f"{W} get me one of those", "vague", "grab", None, "deictic plural"),
    (f"{W} grab the red one", "vague", "grab", None,
     "TWO red objects -- must ask, never pick"),

    # ---- QUESTIONS AND NON-COMMANDS
    (f"{W} can you see the blue cube", "non-command", None, None,
     "a question, not an instruction"),
    (f"{W} what colour is the cube", "non-command", None, None, "a question"),
    (f"{W} the blue cube is nice", "non-command", None, None, "a statement"),
    ("just chatting about grabbing the blue cube", "non-command", None, None,
     "NO WAKE WORD -- must be silent"),

    # ---- ARM vs SPATIAL QUALIFIER. 'left' is both.
    (f"{W} grab the left cube", "arm-vs-spatial", "grab", None,
     "'left' means WHICH CUBE, not which arm"),
    (f"{W} grab the blue cube with your left hand", "arm-vs-spatial", "grab",
     "blue cube", "'left' really does mean the arm here"),

    # ---- STOP, which bypasses the wake word by design
    ("stop", "stop", "stop", None, "must halt with no wake word"),
    (f"{W} grab the blue cube before it stops rolling", "stop", "grab",
     "blue cube", "'stops' inside a grab command"),
]


def classify(utt, want_verb, want_label):
    """Run the SHIPPED path and grade the outcome."""
    intent = vi.parse(utt, wake=WAKE, require_wake=True)
    detail = {}

    if not intent.ok:
        return "REFUSED", None, dict(reason=intent.reason)

    got_verb = intent.verb
    detail["verb"] = got_verb

    if got_verb == "stop":
        if want_verb == "stop":
            return "CORRECT", None, detail
        # A spurious stop halts a healthy arm. Disruptive, but it errs toward
        # not moving, so it is not the dangerous category.
        return "REFUSED", None, dict(detail, note="spurious STOP (fail-safe)")

    # A command was produced. Was one wanted at all?
    if want_verb is None:
        detail["resolved_intent"] = intent.as_dict()
        # It parsed a verb from something that was not a command. Whether that
        # is dangerous depends on whether it also grounds a target.
        status, chosen, cands = vi.resolve_target(intent.target, SCENE)
        if status == vi.UNIQUE:
            detail["would_grab"] = chosen["label"]
            return "MISUNDERSTOOD", chosen["label"], detail
        if status == vi.AMBIGUOUS:
            return "ASKED", None, dict(detail, n=len(cands))
        return "REFUSED", None, dict(detail, note="verb but no groundable target")

    if got_verb != want_verb:
        return "MISUNDERSTOOD", None, dict(detail, wanted_verb=want_verb)

    if intent.deictic and intent.target is None:
        return "REFUSED", None, dict(detail, note="deictic, no pointing input")

    status, chosen, cands = vi.resolve_target(intent.target, SCENE)
    detail["target_prompt"] = intent.target
    detail["status"] = status

    if status == vi.NO_MATCH:
        return "REFUSED", None, detail
    if status == vi.AMBIGUOUS:
        detail["candidates"] = [c["label"] for c in cands]
        return "ASKED", None, detail

    got = chosen["label"]
    detail["would_grab"] = got
    if want_label is None:
        # We required ASKED or REFUSED. It committed to one object instead.
        return "MISUNDERSTOOD", got, detail
    if got == want_label:
        return "CORRECT", got, detail
    return "MISUNDERSTOOD", got, dict(detail, wanted=want_label)


def self_test():
    """PROVE THE HARNESS CAN STILL REPORT A MISUNDERSTANDING.

    This exists because of the standing rule and because of what it is
    guarding. The headline of this sweep is a COUNT OF ZERO, and a count of
    zero is exactly what a harness produces when it has stopped grading --
    when `classify()` returns early, when `vi.parse` raises and is swallowed,
    when the scene empties so nothing can ever ground. "0 MISUNDERSTOOD" and
    "0 utterances actually checked" print identically.

    So: swap in a deliberately BROKEN parser -- one that ignores negation
    exactly the way the shipped code did before this pass, and always grounds
    the first object -- and require the harness to catch it. If this does not
    report the four negation cases as MISUNDERSTOOD, the real run's zero means
    nothing and the sweep refuses to print one.
    """
    real_parse, real_resolve = vi.parse, vi.resolve_target

    class Naive:
        ok, verb, target, deictic = True, "grab", "blue cube", False

        def as_dict(self):
            return dict(verb="grab", target="blue cube")

    try:
        vi.parse = lambda utt, **kw: Naive()
        vi.resolve_target = lambda desc, scene, **kw: (vi.UNIQUE, scene[0], scene)
        caught = [utt for utt, cat, wv, wl, _ in CASES if cat == "negation"
                  and classify(utt, wv, wl)[0] == "MISUNDERSTOOD"]
    finally:
        vi.parse, vi.resolve_target = real_parse, real_resolve

    n_neg = sum(1 for c in CASES if c[1] == "negation")
    ok = len(caught) == n_neg and n_neg > 0
    print("HARNESS SELF-TEST: broken parser -> %d/%d negation cases caught "
          "as MISUNDERSTOOD ... %s" % (len(caught), n_neg,
                                       "PASS" if ok else "FAIL"))
    return ok


def main():
    if not self_test():
        print("\nREFUSING TO REPORT. The harness could not detect a "
              "misunderstanding it was shown on purpose, so any count it "
              "prints -- especially a zero -- is meaningless.")
        return 2
    rows, buckets = [], {}
    for utt, cat, wv, wl, note in CASES:
        verdict, got, detail = classify(utt, wv, wl)
        buckets[verdict] = buckets.get(verdict, 0) + 1
        rows.append(dict(utterance=utt, category=cat, note=note,
                         intended_verb=wv, intended_label=wl,
                         verdict=verdict, resolved=got, detail=detail))

    print("=" * 78)
    print("LANGUAGE AND VISION SWEEP -- %d utterances, %d categories"
          % (len(CASES), len(set(c[1] for c in CASES))))
    print("scene: " + ", ".join(sorted(PRESENT)))
    print("=" * 78)
    cur = None
    for r in rows:
        if r["category"] != cur:
            cur = r["category"]
            print("\n-- %s" % cur.upper())
        mark = "   <<< MISUNDERSTOOD" if r["verdict"] == "MISUNDERSTOOD" else ""
        got = r["resolved"] or ""
        print("  %-52s %-14s %s%s"
              % (r["utterance"][:52], r["verdict"], got, mark))

    print("\n" + "=" * 78)
    print("TOTALS")
    print("=" * 78)
    for k in ("CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"):
        n = buckets.get(k, 0)
        print("  %-14s %3d   (%4.1f%%)" % (k, n, 100.0 * n / len(rows)))
    print("\n  ASKED and REFUSED are SAFE outcomes -- the robot moved nothing.")

    bad = [r for r in rows if r["verdict"] == "MISUNDERSTOOD"]
    print("\n" + "!" * 78)
    if not bad:
        print("NO MISUNDERSTANDINGS.")
    else:
        print("%d MISUNDERSTOOD -- THE DANGEROUS CATEGORY. Each of these is the"
              % len(bad))
        print("robot confidently doing something the speaker did not ask for.")
        print("!" * 78)
        for r in bad:
            print("\n  SAID    : %s" % r["utterance"])
            print("  MEANT   : %s" % (("%s %s" % (r["intended_verb"],
                                                  r["intended_label"]))
                                      if r["intended_label"]
                                      else "%s (must ask or refuse)"
                                           % r["intended_verb"]))
            print("  WOULD DO: %s %s" % (r["detail"].get("verb", "?"),
                                         r["resolved"] or "--"))
            print("  WHY     : %s" % r["note"])
    print("!" * 78)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(dict(scene=SCENE, totals=buckets, rows=rows), fh, indent=2)
    print("\n  -> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
