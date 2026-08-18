#!/usr/bin/env python3
"""A TYPED INSTRUCTION PLUS WHAT THE CAMERA SAW, INTO A T1 PLAN.

    from t1_instruction import plan_from
    outcome = plan_from("put the blue cube on the blue pad", cubes)

WHERE THIS SITS. `voice_intent.parse()` turns the sentence into an Intent and
knows nothing about this scene. `vision_grasp.observe_and_detect()` turns a
camera frame into `[(x, y, pad_index), ...]`, where the pad index comes from
the colour the CAMERA classified. This module is the join, and it is the only
place the two meet, so there is one answer to "which cube, and where does it
go" rather than one per caller.

THE COLOUR IS THE CAMERA'S, EVERYWHERE. `T1_PAIR` -- the declared colour of
each cube -- is not read here at all. A cube declared blue and rendered green
is planned as GREEN, which is the whole point of the mislabel control in
`scripts/verify_vision_drives_grasp.py`: change the declaration and the plan
must not move; change the pixels and it must.

FOUR OUTCOMES, AND ONLY ONE OF THEM IS DANGEROUS
------------------------------------------------
  PLAN      the instruction resolved to specific cubes and specific pads
  ASK       it resolved to more than one possibility, and NOTHING is chosen
  REFUSE    it cannot be represented or cannot be grounded, and it says why
  -- and the fourth is MISUNDERSTOOD, which is not a value this module can
  return: it is what a PLAN turns out to be when it is scored against what the
  operator meant. `scripts/sweep_t1_instructions.py` is what measures it.

WHY ASK IS NOT A FAILURE. A robot bolted to a person that says "which one?"
has lost a few seconds. One that picks the wrong cube because it silently
resolved an ambiguity has moved metal near a wearer on a guess. Every branch
below that could guess asks instead.

THE SINGULAR/PLURAL DISTINCTION IS LOAD-BEARING. "put the blue cube on the
blue pad" names ONE cube; there are two blue cubes in T1's layout, so it must
ASK. "put the blue ones on the blue pad" names both and must not. That
difference lives in `Intent.quantity`, which is read off the words, and not
in a heuristic about how many things happen to match.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
for _p in (HERE, os.path.join(_ROOT, "src", "srl_autonomy")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from srl_autonomy import voice_intent as vi                  # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
import t1_task as T1M                                        # noqa: E402

PLAN, ASK, REFUSE = "plan", "ask", "refuse"

# The nouns that name a T1 cube. A "tray" or a "bottle" is a perfectly good
# parse and is not in this scene, and saying so is better than picking the
# nearest cube.
CUBE_NOUNS = ("cube", "block", "box", "object", "thing", "one")


class Outcome(object):
    """(kind, picks, message). `picks` is [(x, y, pad_index)] for the builder."""

    def __init__(self, kind, picks=None, message="", intent=None, seen=None):
        self.kind = kind
        self.picks = list(picks or [])
        self.message = message
        self.intent = intent
        self.seen = list(seen or [])

    @property
    def ok(self):
        return self.kind == PLAN

    def __repr__(self):
        return "Outcome(%s, %d picks, %r)" % (self.kind, len(self.picks),
                                              self.message)

    def as_dict(self):
        return dict(kind=self.kind, picks=[list(p) for p in self.picks],
                    message=self.message,
                    intent=None if self.intent is None else self.intent.as_dict())


def _colour_of(pad_index):
    return MCT.PLANE_COLOURS[int(pad_index)]


def _pad_for_colour(name):
    """A colour name -> the pad index that colour belongs to, or None.

    None is a REFUSAL, not a fallback. There is no red pad in this scene and
    delivering a cube to the nearest pad instead would be the silent
    substitution this repository's audit log is full of.
    """
    for i, c in enumerate(MCT.PLANE_COLOURS):
        if c == name:
            return i
    return None


def describe(cubes):
    """What the camera saw, in words, for a question or a log line."""
    if not cubes:
        return "nothing"
    counts = {}
    for c in cubes:
        counts[_colour_of(c[2])] = counts.get(_colour_of(c[2]), 0) + 1
    return ", ".join("%d %s" % (n, k) for k, n in sorted(counts.items()))


def plan_from(text, cubes, wake=vi.WAKE_DEFAULT, require_wake=False):
    """A typed instruction and a set of DETECTED cubes -> an Outcome.

    `cubes` is what `vision_grasp.observe_and_detect()` returned:
    [(x, y, pad_index)], the pad index chosen from the colour the camera
    classified. `require_wake` is False because this is TYPED -- the wake word
    exists to stop a microphone acting on overheard speech, and a keyboard has
    no such problem. Speech keeps the wake word; the flag is here so both can
    use one parser.
    """
    intent = vi.parse(text, wake=wake, require_wake=require_wake)
    seen = [(float(c[0]), float(c[1]), int(c[2])) for c in cubes or []]

    def out(kind, msg, picks=None):
        return Outcome(kind, picks, msg, intent=intent, seen=seen)

    if not intent.ok:
        # A TIE IN THE SPELLING IS A QUESTION, NOT A REFUSAL. `voice_intent`
        # writes the alternatives onto the intent rather than choosing one.
        if getattr(intent, "ambiguous_words", None):
            return out(ASK, intent.reason)
        return out(REFUSE, intent.reason)

    if intent.verb != "put_on":
        return out(REFUSE,
                   "I understood %r, and this task is putting cubes on pads. "
                   "Say where the cube should go." % intent.verb)

    if not seen:
        return out(REFUSE,
                   "I did not see any cubes, so I have nothing to plan with. "
                   "This is a detection failure and not an empty table -- I "
                   "will not fall back to the coordinates in the task file.")

    # ---- WHICH CUBES ------------------------------------------------------
    want_colour, want_noun = None, None
    for w in (intent.target or "").split():
        if w in vi.COLOURS and want_colour is None:
            want_colour = w
        elif w in vi.NOUNS and want_noun is None:
            want_noun = w

    if intent.deictic and want_colour is None:
        # "put it on the blue pad" with nothing held and nothing pointed at.
        # There is no pointing device on this rig and no held object to mean
        # "it", so the honest answer is a question.
        return out(ASK,
                   "Which cube? I can see %s, and %r does not tell me which "
                   "one." % (describe(seen), intent.raw.strip()))

    if want_noun is not None and want_noun not in CUBE_NOUNS:
        return out(REFUSE,
                   "I can see %s on the table and no %s."
                   % (describe(seen), want_noun))

    if want_colour is None:
        cands = list(seen)
    else:
        cands = [c for c in seen if _colour_of(c[2]) == want_colour]
        if not cands:
            return out(REFUSE,
                       "I do not see a %s cube. The camera sees %s."
                       % (want_colour, describe(seen)))

    if intent.quantity != "all" and len(cands) > 1:
        # THE SENTENCE SAID ONE AND THE CAMERA SEES SEVERAL. Choosing the
        # first, the nearest or the leftmost would all be defensible and all
        # be guesses. Ask.
        where = ", ".join("x=%.2f" % c[0] for c in cands)
        return out(ASK,
                   "I see %d %s cubes (%s). Which one -- or say 'the %s ones' "
                   "for all of them?"
                   % (len(cands), want_colour or "matching", where,
                      want_colour or "matching"))

    # ---- WHERE THEY GO ----------------------------------------------------
    dest = intent.destination or {}
    if dest.get("kind") == "match":
        picks = [(c[0], c[1], c[2]) for c in cands]      # its OWN seen colour
        how = "each on the pad of the colour the camera saw"
    elif dest.get("kind") == "colour":
        pad = _pad_for_colour(dest.get("colour"))
        if pad is None:
            return out(REFUSE,
                       "there is no %s pad in this scene -- the pads are %s"
                       % (dest.get("colour"), " and ".join(MCT.PLANE_COLOURS)))
        picks = [(c[0], c[1], pad) for c in cands]
        how = "on the %s pad" % MCT.PLANE_COLOURS[pad]
    else:
        return out(REFUSE, "I did not hear where to put it")

    # ORDERED BY x, NEAREST THE BODY FIRST. Not cosmetic: the pick paths are
    # commanded in list order, and a stable order makes two runs of the same
    # instruction comparable.
    picks.sort(key=lambda p: p[0])
    msg = ("%d cube%s, %s"
           % (len(picks), "" if len(picks) == 1 else "s", how))
    if intent.corrections:
        msg += " (read %s)" % ", ".join("%r as %r" % (a, b)
                                        for a, b in intent.corrections)
    return out(PLAN, msg, picks)


def build_path(picks):
    """The T1 path for a resolved plan, from the ONE builder.

    `t1_task.build(cubes=...)` accepts (x, y, pad_index) because the vision
    join needed it. Passing a SUBSET is what makes "put the blue ones on the
    blue pad" a two-cube task rather than the whole four-cube routine with two
    of them ignored.

    IT IS `t1_task`, NOT `msc_clip_tasks.t1`, SINCE THE 2026-08-17 REBUILD.
    The builder moved with the task; the pad index a pick carries now also
    decides which ARM runs it, because the two pads are on opposite sides of
    the centreline.
    """
    return T1M.build(cubes=[list(p) for p in picks])
