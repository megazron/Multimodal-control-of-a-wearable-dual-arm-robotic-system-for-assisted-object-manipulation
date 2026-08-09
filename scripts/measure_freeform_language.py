#!/usr/bin/env python3
"""PART 5: free-form language, measured on phrasings it was not designed for.

THE CATEGORY THAT MATTERS IS MISUNDERSTOOD. A refusal costs a repeat. A
misunderstanding moves a manipulator toward the wrong object next to a person.
So every phrase carries the target a competent listener would pick, and the
outcome is scored into four buckets:

  CORRECT       grounded, and to the right object
  ASKED         refused to choose; a question, not a mistake
  REFUSED       said it cannot see anything matching
  MISUNDERSTOOD grounded CONFIDENTLY to the WRONG object   <-- the dangerous one

The phrase set deliberately includes forms the grammar was never written for:
politeness, hedging, filler, relational references, superlatives, misspellings,
compound clauses, and phrases that sound specific while constraining nothing.
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_autonomy"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
from srl_autonomy import referring as R          # noqa: E402
from srl_autonomy import voice_intent as VI      # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/freeform_language.json")

# A scene with deliberate near-ties, because a set of obviously-distinct
# objects cannot exercise the ambiguity path at all.
SCENE = [
    dict(label="red block",   xyz=(0.32, 0.35, 1.02), rgb=(0.8, 0.1, 0.1),
         volume=6.4e-5),
    dict(label="blue block",  xyz=(-0.30, 0.35, 1.02), rgb=(0.15, 0.3, 0.85),
         volume=6.4e-5),
    dict(label="blue cup",    xyz=(-0.10, 0.42, 1.05), rgb=(0.15, 0.3, 0.85),
         volume=2.4e-4),
    dict(label="green cube",  xyz=(0.10, 0.30, 1.00), rgb=(0.15, 0.7, 0.2),
         volume=2.7e-5),
]

# (phrase, expected label or None, note). None means: must NOT ground.
PHRASES = [
    # --- designed for
    ("pick up the red block", "red block", "canonical"),
    ("grab the green cube", "green cube", "canonical"),
    ("take the blue cup", "blue cup", "canonical"),
    # --- politeness and filler, never designed for
    ("could you please pick up the red block for me", "red block", "polite"),
    ("um, grab the green cube I guess", "green cube", "filler"),
    ("yeah go ahead and take the blue cup now", "blue cup", "filler"),
    ("if you would be so kind as to lift the red block", "red block",
     "elaborate"),
    # --- vacuous: sound specific, constrain nothing
    ("pick that random thing", None, "VACUOUS -- must ask"),
    ("grab something", None, "VACUOUS"),
    ("just take whatever", None, "VACUOUS"),
    ("get me that thing over there", None, "VACUOUS + deictic"),
    ("pick up one of them", None, "VACUOUS"),
    # --- spatial, never designed for
    ("grab the leftmost object", "blue block", "spatial superlative"),
    ("pick up the one on the right", "red block", "spatial"),
    ("take the nearest thing", "green cube", "spatial + vacuous noun"),
    ("get the furthest one", "blue cup", "spatial superlative"),
    # --- size
    ("pick up the biggest one", "blue cup", "size superlative"),
    ("grab the smallest object", "green cube", "size superlative"),
    # --- genuinely ambiguous: two blue things
    ("pick up the blue one", None, "AMBIGUOUS -- two blue objects"),
    ("grab the blue thing", None, "AMBIGUOUS"),
    # --- compound colour+noun disambiguates
    ("pick up the blue block", "blue block", "colour + noun"),
    ("take the blue cup please", "blue cup", "colour + noun"),
    # --- nothing matching
    ("pick up the purple elephant", None, "not present -- must refuse"),
    ("grab the yellow banana", None, "not present"),
    # --- misspellings and speech-recognition noise
    ("pik up the redd blok", "red block", "misspelt"),
    ("grabb the grean cube", "green cube", "misspelt"),
    # --- compound clauses
    ("pick up the red block and put it in the bin", "red block", "compound"),
    ("take the green cube then stop", "green cube", "compound"),
    # --- relational, deliberately beyond the design
    ("the block next to the cup", None, "RELATIONAL -- not supported"),
    ("the one behind the red block", None, "RELATIONAL -- not supported"),
]


def classify(phrase, expected):
    res = R.ground(phrase, SCENE)
    got = res["outcome"]
    if got == R.GROUNDED:
        lab = res["target"]["label"]
        if expected is None:
            return "MISUNDERSTOOD", lab, res
        return ("CORRECT" if lab == expected else "MISUNDERSTOOD"), lab, res
    if got == R.ASK:
        return "ASKED", None, res
    return "REFUSED", None, res


def main():
    print("=" * 78)
    print("PART 5  FREE-FORM LANGUAGE, on phrasings it was not designed for")
    print("=" * 78)
    print("Scene: %s" % ", ".join(c["label"] for c in SCENE))
    print("\n%-46s %-14s %s" % ("phrase", "outcome", "resolved to"))
    print("-" * 78)
    buckets = {}
    rows = []
    for phrase, expected, note in PHRASES:
        verdict, lab, res = classify(phrase, expected)
        buckets[verdict] = buckets.get(verdict, 0) + 1
        rows.append(dict(phrase=phrase, expected=expected, note=note,
                         verdict=verdict, resolved=lab,
                         say=res.get("say", "")))
        mark = "  <-- DANGEROUS" if verdict == "MISUNDERSTOOD" else ""
        print("%-46s %-14s %s%s"
              % (phrase[:45], verdict, lab or res.get("say", "")[:28], mark))

    n = len(PHRASES)
    print("\nSUMMARY over %d phrases" % n)
    for k in ("CORRECT", "ASKED", "REFUSED", "MISUNDERSTOOD"):
        v = buckets.get(k, 0)
        print("  %-14s %2d  %5.1f%%%s" % (k, v, 100.0 * v / n,
                                          "   <-- the dangerous category"
                                          if k == "MISUNDERSTOOD" else ""))

    # ---- verb handling stays closed and is checked separately
    print("\nVERB HANDLING (closed set, unchanged): stop must win everywhere")
    vt = [("stop", "stop"), ("stop now", "stop"), ("halt", "stop"),
          ("pick up the red block", "grab"),
          ("grab it but stop if it slips", "stop"),
          ("put it down", "place"), ("hand it to the other arm", "handover"),
          ("dance a jig", None)]
    vbad = 0
    for text, want in vt:
        got = VI.parse(text, wake=None).verb
        ok = got == want
        vbad += 0 if ok else 1
        print("    %-34s -> %-9s %s" % (text, got, "" if ok else "MISMATCH"))
    print("    verb mismatches: %d" % vbad)

    out = dict(rows=rows, buckets=buckets, n=n, verb_mismatches=vbad)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print("\n  -> %s" % OUT)
    return 1 if buckets.get("MISUNDERSTOOD", 0) else 0


if __name__ == "__main__":
    sys.exit(main())
