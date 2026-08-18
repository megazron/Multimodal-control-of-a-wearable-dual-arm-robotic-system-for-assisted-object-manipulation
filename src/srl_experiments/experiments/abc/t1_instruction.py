#!/usr/bin/env python3
"""A TYPED INSTRUCTION PLUS WHAT THE CAMERA SAW, INTO A T1 PLAN.

    from t1_instruction import plan_from, answer
    outcome = plan_from("put the blue cube on the blue pad", cubes)
    if outcome.kind == ASK:
        outcome = answer(outcome, "the left one", cubes)

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

AN ASK IS NOW ANSWERABLE, AND THAT IS THE 2026-08-18 CHANGE
-----------------------------------------------------------
Asking was already safe and it was also a dead end: the operator got a
question and had to retype the whole sentence. Every ASK now carries a
`question` -- what was asked, and the candidates it was asked about -- and
`answer()` resolves it from a reply as short as "the left one", "both",
"either", "the second", "the blue one" or "yes". The GUI's prompt box and
`scripts/instruct_t1.py` use the same two calls, so there is one dialogue and
not one per front end.

THE SINGULAR/PLURAL DISTINCTION IS LOAD-BEARING. "put the blue cube on the
blue pad" names ONE cube; there are two blue cubes in T1's layout, so it must
ASK. "put the blue ones on the blue pad" names both and must not. That
difference lives in `Intent.quantity`, which is read off the words, and not
in a heuristic about how many things happen to match.

WHAT A SELECTOR BUYS. "the leftmost blue cube" names exactly one cube, and
until this module could read selectors it asked a question the operator had
already answered. Selectors are resolved against the DETECTIONS -- what the
camera saw -- and a selector the scene cannot separate ("the biggest cube",
against four identical 40 mm cubes) is REFUSED by name rather than dropped,
because dropping it is how a qualified instruction becomes an unqualified one.

A BARE PICK IS A REAL ACTION. "pick up the blue cube" says nothing about where
it goes and it is not half an instruction -- it is a complete one, and it is
planned as a pick and a lift with the destination left alone. Inventing "and
put it on its own colour" would be exactly the silent completion this file
refuses everywhere else.

COMPOUND INSTRUCTIONS ARE RUN, NOT TRUNCATED. "put the blue ones on the blue
pad and the green ones on the green pad" is two complete instructions sharing
a verb. Measured on 2026-08-18, the shipped code planned the FIRST clause and
dropped the second in silence: two cubes moved of four, reported as a
success. `voice_intent.split_clauses` names the clauses and this module plans
each and merges them, and it is still a refusal when a clause does not stand
on its own.
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

# The verbs this task can act on, and what each one MEANS here. Anything else
# parses and is refused by name.
#   put_on   pick a cube and place it on a pad
#   grab     pick a cube up and hold it -- a complete instruction with no
#            destination, not half of one
#   tidy     "sort this out", which has an obvious reading and no stated one
ACTABLE = ("put_on", "grab", "tidy")

YES = ("yes", "yeah", "yep", "yes please", "ok", "okay", "sure", "go on",
       "do it", "correct", "right", "that s right", "thats right", "please do")
NO = ("no", "nope", "cancel", "stop", "forget it", "never mind", "nevermind",
      "abort", "not that")


class Outcome(object):
    """(kind, picks, message). `picks` is [(x, y, pad_index)] for the builder.

    `pad_index` is None for a pick-and-hold, which is what a bare "pick up the
    blue cube" resolves to.

    `question` is set on an ASK and is what makes the ASK answerable. It
    carries the candidates the question was asked ABOUT, so `answer()` narrows
    the same set the operator was shown rather than re-grounding from scratch
    against a scene that may have been re-detected in between.
    """

    def __init__(self, kind, picks=None, message="", intent=None, seen=None,
                 question=None):
        self.kind = kind
        self.picks = list(picks or [])
        self.message = message
        self.intent = intent
        self.seen = list(seen or [])
        self.question = dict(question) if question else None

    @property
    def ok(self):
        return self.kind == PLAN

    @property
    def hold_only(self):
        """True when nothing is placed -- every pick is a pick and a lift."""
        return bool(self.picks) and all(p[2] is None for p in self.picks)

    def __repr__(self):
        return "Outcome(%s, %d picks, %r)" % (self.kind, len(self.picks),
                                              self.message)

    def as_dict(self):
        return dict(kind=self.kind,
                    picks=[[p[0], p[1], p[2]] for p in self.picks],
                    message=self.message, question=self.question,
                    hold_only=self.hold_only,
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


def _where(cands):
    """Name the candidates by where they are, so a question can be answered.

    Positions, not indices: the operator is looking at a table, not at a list.
    """
    out = []
    for c in cands:
        out.append("%s at x = %+.2f" % (_colour_of(c[2]), c[0]))
    return "; ".join(out)


# --------------------------------------------------------------- selectors
def _apply_selector(sel, cands):
    """(chosen, why) or (None, reason) -- narrow a candidate set by the words.

    +x is the wearer's RIGHT-HAND side of the table in this repository's world
    frame and the LEFT arm's side of the layout, which is why "left" here means
    the largest x. That is the arm's own name for it and the name written on
    the clip; getting it backwards would hand the operator the cube on the
    other side of the table while sounding perfectly correct.

    NEAR and FAR are y, which for T1 is a single row, so they cannot separate
    anything and say so rather than choosing.
    """
    if not sel:
        return list(cands), None
    kind = sel.get("kind")
    if kind == "ungroundable":
        return None, (
            "I cannot pick one out by %r -- the camera sees %d cubes and they "
            "are all the same 40 mm cube. Say which one by where it is (the "
            "leftmost, the second) or by colour."
            % (sel.get("word"), len(cands)))
    if kind == "any":
        return list(cands[:1]), "you said it did not matter which"
    if kind == "spatial":
        w = sel.get("which")
        if w in ("left", "right"):
            s = sorted(cands, key=lambda c: c[0])
            return [s[-1] if w == "left" else s[0]], "the %s one" % w
        if w in ("near", "far"):
            ys = {round(c[1], 4) for c in cands}
            if len(ys) < 2:
                return None, (
                    "%r cannot separate them -- every cube is in the same row, "
                    "%.2f m from you. Say the leftmost, the rightmost, or a "
                    "colour." % (w, list(ys)[0] if ys else 0.0))
            s = sorted(cands, key=lambda c: c[1])
            return [s[0] if w == "near" else s[-1]], "the %s one" % w
        if w in ("inner", "outer"):
            s = sorted(cands, key=lambda c: abs(c[0]))
            return [s[0] if w == "inner" else s[-1]], "the %s one" % w
        if w == "middle":
            if len(cands) % 2 == 0:
                return None, (
                    "there is no middle one -- I can see %d, which is an even "
                    "number. Say the leftmost or the rightmost."
                    % len(cands))
            s = sorted(cands, key=lambda c: c[0])
            return [s[len(s) // 2]], "the middle one"
        return None, "I do not know what %r means here" % w
    if kind == "ordinal":
        n = int(sel.get("n", 1))
        s = sorted(cands, key=lambda c: c[0])
        idx = len(s) - 1 if n == -1 else n - 1
        if idx < 0 or idx >= len(s):
            return None, ("there is no %s one -- I can see %d"
                          % ("last" if n == -1 else "number %d" % n, len(s)))
        return [s[idx]], ("the last one" if n == -1 else "number %d" % n)
    return list(cands), None


def _cubes_matching(seen, want_colour):
    if want_colour is None:
        return list(seen)
    return [c for c in seen if _colour_of(c[2]) == want_colour]


# ------------------------------------------------------------------ plan
def plan_from(text, cubes, wake=vi.WAKE_DEFAULT, require_wake=False,
              pending=None):
    """A typed instruction and a set of DETECTED cubes -> an Outcome.

    `cubes` is what `vision_grasp.observe_and_detect()` returned:
    [(x, y, pad_index)], the pad index chosen from the colour the camera
    classified. `require_wake` is False because this is TYPED -- the wake word
    exists to stop a microphone acting on overheard speech, and a keyboard has
    no such problem. Speech keeps the wake word; the flag is here so both can
    use one parser.

    `pending` is the ASK this text is answering, if it is answering one. One
    entry point rather than two means a front end cannot forget to route the
    reply and silently start a new instruction with it.
    """
    if pending is not None and getattr(pending, "question", None):
        return answer(pending, text, cubes)

    intent = vi.parse(text, wake=wake, require_wake=require_wake)
    seen = [(float(c[0]), float(c[1]), int(c[2])) for c in cubes or []]

    def out(kind, msg, picks=None, question=None):
        return Outcome(kind, picks, msg, intent=intent, seen=seen,
                       question=question)

    # A TIE IN THE SPELLING IS A QUESTION, NOT A REFUSAL, AND IT IS ASKED
    # BEFORE ANYTHING ELSE. `voice_intent` writes the alternatives onto the
    # intent rather than choosing one.
    #
    # IT IS CHECKED WHETHER OR NOT THE SENTENCE PARSED, and that is the
    # 2026-08-18 correction. "put the gren ones on the gren pad" PARSES --
    # `ones` is a noun and `pad` is a pad -- so the tie was never reached and
    # the operator was asked "which cube?", a question about the wrong half of
    # the sentence. The word that could not be read is the thing to say.
    if getattr(intent, "ambiguous_words", None):
        tok, cands = intent.ambiguous_words[0]
        return out(ASK,
                   "I could not read %r -- did you mean %s?"
                   % (tok, " or ".join(cands)),
                   question=dict(kind="spelling", token=tok,
                                 options=list(cands), raw=text))
    if not intent.ok:
        return out(REFUSE, intent.reason)

    if intent.verb not in ACTABLE:
        return out(REFUSE,
                   "I understood %r, and this task is picking cubes up and "
                   "putting them on pads. Say which cube and where it goes."
                   % intent.verb)

    if not seen:
        return out(REFUSE,
                   "I did not see any cubes, so I have nothing to plan with. "
                   "This is a detection failure and not an empty table -- I "
                   "will not fall back to the coordinates in the task file.")

    # ---- A COMPOUND IS EVERY CLAUSE, OR IT IS NOTHING --------------------
    if len(getattr(intent, "clauses", []) or []) > 1:
        return _plan_compound(intent, seen, text, wake)

    # ---- "TIDY UP" -- offer the reading, never assume it -----------------
    if intent.verb == "tidy":
        return out(ASK,
                   "I can see %s. Do you mean put every cube on the pad of "
                   "its own colour? Say yes, or tell me what to do instead."
                   % describe(seen),
                   question=dict(kind="confirm", proposal="match_all",
                                 cands=[list(c) for c in seen], raw=text))

    return _plan_single(intent, seen, text)


def _plan_compound(intent, seen, text, wake):
    """Every clause planned separately and merged, in the order uttered."""
    picks, notes = [], []
    for cl in intent.clauses:
        o = plan_from(cl, seen, wake=wake, require_wake=False)
        if o.kind != PLAN:
            return Outcome(o.kind, [], "%s (in %r, which is one of %d parts "
                                       "of what you said)"
                           % (o.message, cl, len(intent.clauses)),
                           intent=intent, seen=seen, question=o.question)
        picks.extend(o.picks)
        notes.append(o.message)
    # THE SAME CUBE NAMED TWICE IS A CONTRADICTION, NOT A DUPLICATE. "put the
    # blue ones on the blue pad and the blue ones on the green pad" asks for
    # one cube in two places, and running the first is partial execution.
    byxy = {}
    for p in picks:
        k = (round(p[0], 4), round(p[1], 4))
        if k in byxy and byxy[k] != p[2]:
            return Outcome(REFUSE, [],
                           "the two halves of that send the same cube to two "
                           "different pads", intent=intent, seen=seen)
        byxy[k] = p[2]
    picks.sort(key=lambda p: p[0])
    return Outcome(PLAN, picks, " and ".join(notes), intent=intent, seen=seen)


def _plan_single(intent, seen, text):
    def out(kind, msg, picks=None, question=None):
        return Outcome(kind, picks, msg, intent=intent, seen=seen,
                       question=question)

    # ---- WHICH CUBES ------------------------------------------------------
    want_colour, want_noun = None, None
    for w in (intent.target or "").split():
        if w in vi.COLOURS and want_colour is None:
            want_colour = w
        elif w in vi.NOUNS and want_noun is None:
            want_noun = w

    if intent.deictic and want_colour is None and not intent.selector:
        # "put it on the blue pad" with nothing held and nothing pointed at.
        # There is no pointing device on this rig and no held object to mean
        # "it", so the honest answer is a question.
        return out(ASK,
                   "Which cube? I can see %s, and %r does not tell me which "
                   "one. Say a colour, or the leftmost, or the second."
                   % (describe(seen), intent.raw.strip()),
                   question=dict(kind="which_cube",
                                 cands=[list(c) for c in seen],
                                 verb=intent.verb,
                                 dest=intent.destination, raw=text))

    if want_noun is not None and want_noun not in CUBE_NOUNS:
        return out(REFUSE,
                   "I can see %s on the table and no %s."
                   % (describe(seen), want_noun))

    cands = _cubes_matching(seen, want_colour)
    if not cands:
        return out(REFUSE,
                   "I do not see a %s cube. The camera sees %s."
                   % (want_colour, describe(seen)))

    # ---- NARROW BY THE SELECTOR, IF THERE WAS ONE ------------------------
    chosen, why = _apply_selector(intent.selector, cands)
    if chosen is None:
        return out(REFUSE, why)
    cands, sel_note = chosen, why

    if intent.quantity != "all" and len(cands) > 1:
        # THE SENTENCE SAID ONE AND THE CAMERA SEES SEVERAL. Choosing the
        # first, the nearest or the leftmost would all be defensible and all
        # be guesses. Ask -- and the ask is answerable.
        return out(ASK,
                   "I see %d %s cubes (%s). Which one -- the leftmost, the "
                   "rightmost, or say 'both' for all of them?"
                   % (len(cands), want_colour or "matching", _where(cands)),
                   question=dict(kind="which_cube",
                                 cands=[list(c) for c in cands],
                                 verb=intent.verb,
                                 dest=intent.destination, raw=text))

    # ---- WHERE THEY GO ----------------------------------------------------
    return _finish(intent, seen, cands, intent.destination, sel_note, text)


def _finish(intent, seen, cands, dest, sel_note, text):
    """Candidates plus a destination -> a plan, a question, or a refusal."""
    def out(kind, msg, picks=None, question=None):
        return Outcome(kind, picks, msg, intent=intent, seen=seen,
                       question=question)

    dest = dest or {}
    verb = intent.verb

    if verb == "grab" and not dest:
        # A BARE PICK. It is a whole instruction, not half of one, and it is
        # planned as a pick and a lift with no place.
        if len(cands) > 1:
            return out(ASK,
                       "I can hold one cube at a time, and that names %d "
                       "(%s). Say which one, or say 'put them on their own "
                       "colours' if you want them all moved."
                       % (len(cands), _where(cands)),
                       question=dict(kind="which_cube",
                                     cands=[list(c) for c in cands],
                                     verb=verb, dest=None, raw=text))
        picks = [(cands[0][0], cands[0][1], None)]
        return out(PLAN, "1 cube, picked up and held%s"
                   % ("" if not sel_note else " (%s)" % sel_note), picks)

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
    elif dest.get("kind") == "unspecified":
        # "put the blue ones on the pad" names a place and not WHICH place.
        # There are two, so this is a question and not a refusal.
        return out(ASK,
                   "Which pad -- the %s one or the %s one? Or say 'its own "
                   "colour'." % MCT.PLANE_COLOURS,
                   question=dict(kind="which_pad",
                                 cands=[list(c) for c in cands],
                                 verb=verb, raw=text))
    else:
        return out(REFUSE, "I did not hear where to put it")

    # ORDERED BY x, so two runs of the same instruction are comparable.
    picks.sort(key=lambda p: p[0])
    msg = ("%d cube%s, %s%s"
           % (len(picks), "" if len(picks) == 1 else "s", how,
              "" if not sel_note else " (%s)" % sel_note))
    if intent.corrections:
        msg += " (read %s)" % ", ".join("%r as %r" % (a, b)
                                        for a, b in intent.corrections)
    return out(PLAN, msg, picks)


# ---------------------------------------------------------------- answers
def answer(pending, text, cubes=None):
    """A reply to an ASK -> a new Outcome. Never guesses; may ask again.

    THE REPLY IS NARROWED AGAINST THE CANDIDATES THE QUESTION NAMED, not
    re-grounded from the whole scene. If the operator was shown two blue cubes
    and says "the left one", they mean the left of those two, and re-running
    the sentence against all four would answer a different question.
    """
    q = getattr(pending, "question", None)
    seen = [tuple(c) for c in (cubes if cubes is not None else pending.seen)]
    intent = pending.intent
    raw = (text or "").strip()
    low = vi.strip_filler(vi.normalise(raw))

    def out(kind, msg, picks=None, question=None):
        return Outcome(kind, picks, msg, intent=intent, seen=seen,
                       question=question)

    if q is None:
        return out(REFUSE, "there was no question to answer")
    if low in NO or any(low == w for w in NO):
        return out(REFUSE, "cancelled -- nothing will move")

    cands = [tuple(c) for c in q.get("cands", [])] or seen

    if q["kind"] == "confirm":
        if low in YES:
            picks = sorted([(c[0], c[1], c[2]) for c in seen],
                           key=lambda p: p[0])
            return out(PLAN, "%d cubes, each on the pad of the colour the "
                             "camera saw (you confirmed it)" % len(picks),
                       picks)
        # Not a yes and not a no -- treat it as a fresh instruction rather
        # than as an answer, because that is what it is.
        return plan_from(raw, seen)

    if q["kind"] == "spelling":
        opts = q.get("options", [])
        pick = next((o for o in opts if o in low.split()), None)
        if pick is None:
            return out(ASK, "I still cannot read %r. Did you mean %s?"
                       % (q.get("token"), " or ".join(opts)), question=q)
        fixed = " ".join(pick if w == q.get("token") else w
                         for w in vi.normalise(q.get("raw", "")).split())
        return plan_from(fixed, seen)

    if q["kind"] == "which_pad":
        colour = next((w for w in low.split() if w in vi.COLOURS), None)
        if "own" in low or "match" in low or "belong" in low:
            return _finish(intent, seen, cands, dict(kind="match"), None, raw)
        if colour is None:
            return out(ASK, "Which pad -- the %s one or the %s one?"
                       % MCT.PLANE_COLOURS, question=q)
        pad = _pad_for_colour(colour)
        if pad is None:
            return out(REFUSE, "there is no %s pad in this scene -- the pads "
                               "are %s" % (colour,
                                           " and ".join(MCT.PLANE_COLOURS)))
        return _finish(intent, seen, cands, dict(kind="colour", colour=colour),
                       None, raw)

    # ---- which cube ------------------------------------------------------
    #
    # A REPLY CAN BE A NEW INSTRUCTION, and treating it as an answer would be
    # the wrong kind of stubbornness. "put them on their own colours" is a
    # complete instruction offered in place of an answer to "which one?", and
    # the operator plainly means it to be acted on rather than parsed as a
    # choice between two cubes.
    #
    # THE TEST IS THAT IT STANDS ALONE: a verb this task can act on AND a
    # destination. "the left one" has neither and stays an answer.
    fresh = vi.parse(raw, wake="", require_wake=False)
    if fresh.ok and fresh.verb in ACTABLE and (fresh.destination
                                               or fresh.clauses):
        return plan_from(raw, seen)

    toks = low.split()
    colour = next((w for w in toks if w in vi.COLOURS), None)
    narrowed = cands
    if colour is not None:
        narrowed = [c for c in cands if _colour_of(c[2]) == colour]
        if not narrowed:
            return out(ASK, "None of those is %s. I was asking about %s."
                       % (colour, _where(cands)), question=q)

    # "BOTH" AFTER THE COLOUR, NOT BEFORE IT. "both blue ones" says two things
    # and the colour is the one that narrows; taking `both` first delivered
    # all four cubes to a question that had been asked about two of them.
    if any(t in ("both", "all", "every", "each", "everything") for t in toks):
        return _finish(intent, seen, narrowed, q.get("dest"),
                       "you said all of them", raw)

    sel = vi.extract_selector(low)
    if sel is None and len(narrowed) == 1:
        return _finish(intent, seen, narrowed, q.get("dest"),
                       "the %s one" % colour if colour else None, raw)
    if sel is None:
        num = next((int(t) for t in toks if t.isdigit()), None)
        if num is not None:
            sel = dict(kind="ordinal", n=num)
    if sel is None:
        return out(ASK,
                   "I did not follow %r. I am asking which of these: %s. Say "
                   "the leftmost, the rightmost, the first, the second, a "
                   "colour, or both." % (raw, _where(cands)), question=q)

    chosen, why = _apply_selector(sel, narrowed)
    if chosen is None:
        return out(ASK, why, question=q)
    if len(chosen) > 1:
        return out(ASK, "That still leaves %d (%s). Which one?"
                   % (len(chosen), _where(chosen)), question=q)
    return _finish(intent, seen, chosen, q.get("dest"), why, raw)


def build_path(picks):
    """The T1 path for a resolved plan, from the ONE builder.

    `t1_task.build(cubes=...)` accepts (x, y, pad_index) because the vision
    join needed it. Passing a SUBSET is what makes "put the blue ones on the
    blue pad" a two-cube task rather than the whole four-cube routine with two
    of them ignored.

    A pad index of None is a PICK AND HOLD -- the builder stops after the lift
    and returns to park still holding, which is what "pick up the blue cube"
    literally asks for.

    IT IS `t1_task`, NOT `msc_clip_tasks.t1`, SINCE THE 2026-08-17 REBUILD.
    The builder moved with the task; the pad index a pick carries now also
    decides which ARM runs it, because the two pads are on opposite sides of
    the centreline.
    """
    return T1M.build(cubes=[list(p) for p in picks])
