#!/usr/bin/env python3
"""Voice -> structured command. A DETERMINISTIC GRAMMAR, not an LLM.

WHY NOT A LOCAL LLM -- and this is a recommendation, with reasons
-----------------------------------------------------------------
The command set is closed and tiny: grab, give-to-other-arm, put-down, stop,
plus a wake word. A 7B model quantised to fit alongside the perception stack
would cost 3-4 GB of the 4 GB this machine HAS (RTX A500), add 200-800 ms per
utterance, and introduce non-determinism into the one component that decides
whether a robot beside a person's head starts moving.

The failure modes differ in kind, not degree. A grammar that does not
recognise an utterance returns `unparsed` and the robot asks again. An LLM
asked to emit JSON can emit a DIFFERENT, PLAUSIBLE verb -- "put it down" ->
`{"verb":"place"}` when the object is over the wearer -- and nothing
downstream can tell that apart from a correct parse. For five verbs that is a
bad trade.

The grammar is deliberately forgiving about SURFACE form (word order,
fillers, "please") and strict about SEMANTICS. Everything it cannot map, it
refuses.

AMBIGUITY IS AN OUTCOME, NOT AN ERROR
-------------------------------------
`resolve_target()` returns one of three things and the caller must handle all
three: a unique match, AMBIGUOUS with the candidates, or NO_MATCH. It never
picks the first of several. Guessing which green object the operator meant is
exactly the failure that a spoken confirmation is supposed to catch, and
guessing silently defeats it.

MEASURED, 2026-08-09 -- `scripts/sweep_language_vision.py`, 40 phrasings in
13 categories, most of them written against the parser's STRUCTURE rather
than alongside it:

    before this pass   10 CORRECT   7 ASKED  16 REFUSED   7 MISUNDERSTOOD
    after              10 CORRECT   6 ASKED  24 REFUSED   0 MISUNDERSTOOD

("before" is the same 40 cases with `unrepresentable()` stubbed to None, so
the two rows differ by the fix and by nothing else.)

All seven dangerous outcomes were negation, relational reference, a second
target, or a second action -- see `unrepresentable()` below. The CORRECT count
did not fall, so nothing usable was traded away for them.
"""
import re

STOP_WORDS = ("stop", "halt", "freeze", "abort", "cancel", "wait")
WAKE_DEFAULT = "hey doc oc"

# Colour and shape vocabulary the detector prompt is built from. Kept
# explicit so a word the detector cannot ground is rejected at PARSE time
# rather than becoming an empty detection later.
COLOURS = ("red", "green", "blue", "yellow", "orange", "purple", "black",
           "white", "grey", "gray", "brown", "pink")
NOUNS = ("cube", "block", "box", "ball", "sphere", "cylinder", "tool",
         "bottle", "cup", "mug", "tray", "object", "thing", "one")

DEICTIC = ("that", "this", "there", "over there", "it")

# --------------------------------------------------------------------------
# THE THREE WAYS A SENTENCE MEANS SOMETHING THE GRAMMAR CANNOT REPRESENT.
#
# Measured, 2026-08-09, over the 37-phrase language sweep: these three
# accounted for ALL SEVEN MISUNDERSTOOD outcomes -- the robot confidently
# grabbing an object the speaker had just told it not to touch, or silently
# executing half a sentence. Every one of them shares a shape: the grammar
# matched a verb and a noun that were really there, and DISCARDED the word
# that reversed or qualified them.
#
# The fix is deliberately a REFUSAL, not an interpretation. Partial execution
# of a two-part instruction is the dangerous outcome: the operator hears the
# robot acknowledge and assumes the whole sentence landed.
# --------------------------------------------------------------------------

# 1. NEGATION. Only counts BEFORE the verb. "don't grab the cube" negates the
#    command; "grab the blue cube but not the red one" is an exclusion clause
#    on the target and is a perfectly good grab. Position is what tells them
#    apart, so the check is positional rather than a bare keyword search.
#    "don't" normalises to "don t", so the leading token is "don".
NEGATION_WORDS = ("not", "never", "dont", "don", "cannot", "cant", "nor",
                  "neither", "avoid", "refrain")

# 2. SEQUENCE. Two actions in one utterance. Only the first is representable,
#    and executing it alone is partial execution.
SEQUENCE_WORDS = ("then", "afterwards", "afterward", "next")

# 3. CONJUNCTION. Two targets in one utterance. "and" only -- "but" is an
#    exclusion clause and must NOT be caught here.
CONJUNCTION_WORDS = ("and", "plus", "also", "both")

# 4. RELATIONAL reference names an ANCHOR and asks for something else. The
#    single definition lives in `referring`, which measured the failure:
#    grounding "the one behind the red block" by the words present returned
#    the red block with full confidence. Imported rather than copied, because
#    two lists of the same thing drift.
try:
    from .referring import RELATIONAL
except ImportError:                                   # pragma: no cover
    from referring import RELATIONAL

VERB_PATTERNS = (
    # (verb, regex). Order matters: STOP is matched first, everywhere.
    ("stop", re.compile(r"\b(%s)\b" % "|".join(STOP_WORDS))),
    ("handover", re.compile(
        r"\b(give|hand|pass|transfer)\b.*\b(other|opposite|left|right)\b"
        r".*\b(arm|hand|gripper)\b"
        r"|\bswap\s+(arms|hands)\b"
        r"|\bhand(?:\s+it)?\s+(?:over\s+)?to\s+the\s+other\b")),
    ("place", re.compile(
        r"\b(put|set|place|drop|release)\b.*\b(down|here|there|back)\b"
        r"|\blet\s+go\b|\brelease\b")),
    # HAND IT TO THE WEARER. Distinct from the arm-to-arm `handover` above,
    # and the distinction is not pedantic: arm-to-arm transfer is IMPOSSIBLE
    # on this rig -- 0 of 16 candidate transfer points are reachable by both
    # arms and 0 of 63 frontal cells, the sets being disjoint -- while handing
    # an object TO THE WEARER is T5 and is feasible. One verb for both would
    # route a possible request into an impossible mechanism.
    ("handover_wearer", re.compile(
        r"\b(give|hand|pass|bring)\b.*\b(me|to\s+me|wearer|here)\b"
        r"|\bhand\s+it\s+(?:over\s+)?to\s+me\b")),
    # GO SOMEWHERE NAMED. The demonstration asks for "move to the front
    # centre", and the honest answer on this rig is a REFUSAL with the
    # measurement behind it -- see NAMED_PLACES.
    ("goto", re.compile(
        r"\b(move|go|reach|point)\b.*\b(to|toward|towards)\b"
        r"|\bmove\s+(?:to\s+)?the\b")),
    ("grab", re.compile(r"\b(grab|pick|take|get|grasp|fetch|lift)\b")),
)

# NAMED PLACES, and whether the arms can actually get there.
#
# MEASURED, not declared: scripts/survey_work_surface.py, a 29 x 11 grid on
# the work plane, pinned wrist, full path, N=3, controls correct.
#
#     left    34 cells   x  0.30..0.70   y 0.05..0.20
#     right   32 cells   x -0.70..-0.30  y 0.05..0.20
#     |x| <= 0.10 (the FRONT CENTRE)      0 cells
#
# So "move to the front centre" must be refused, and refused with the reason
# rather than with a shrug. It is the single most useful thing the
# demonstration can show about this platform: the reachable region is nothing
# like the region a person expects, and the robot knows it.
NAMED_PLACES = {
    "front centre": dict(reachable=False,
                         why="the front centre is not reachable by either "
                             "arm: 0 of 319 surveyed cells at |x| <= 0.10, "
                             "and the nearest reachable x is 0.30"),
    "front center": dict(reachable=False,
                         why="the front centre is not reachable by either "
                             "arm: 0 of 319 surveyed cells at |x| <= 0.10, "
                             "and the nearest reachable x is 0.30"),
    "left side": dict(reachable=True, arm="left"),
    "right side": dict(reachable=True, arm="right"),
    "home": dict(reachable=True, arm=None),
}

# The literal trigger words of the patterns above, used only to locate WHERE
# in the sentence the verb sits so negation can be scoped to it.
VERB_TRIGGERS = frozenset(
    "give hand pass transfer swap put set place drop release let "
    "grab pick take get grasp fetch lift".split())


def unrepresentable(body):
    """Why this sentence cannot be executed as one command, or None.

    Returns a reason string. Each branch refuses rather than truncating: the
    caller turns it into `Intent(reason=...)`, which the executive speaks
    aloud, so the operator learns the sentence did not land.
    """
    toks = body.split()

    # RELATIONAL first: it is a property of the phrase, not of the verb.
    rel = next((w for w in RELATIONAL if w in body), None)
    if rel:
        return ("relational reference (%r) -- I cannot pick something out by "
                "where it is relative to another object" % rel)

    verb_at = next((i for i, t in enumerate(toks) if t in VERB_TRIGGERS), None)

    if verb_at is not None:
        neg = next((t for t in toks[:verb_at] if t in NEGATION_WORDS), None)
        if neg:
            return "negated command (%r before the verb) -- I will not act" % neg

    seq = next((t for t in toks if t in SEQUENCE_WORDS), None)
    if seq:
        return ("more than one action in one instruction (%r) -- say them one "
                "at a time" % seq)

    conj = next((t for t in toks if t in CONJUNCTION_WORDS), None)
    if conj:
        # A conjunction alone is harmless ("go on and grab it"). It is only a
        # second TARGET when two different colours or two different nouns are
        # named, which is what makes acting on one of them partial execution.
        colours = {t for t in toks if t in COLOURS}
        nouns = {t for t in toks if t in NOUNS}
        if len(colours) > 1 or len(nouns) > 1:
            return ("more than one target in one instruction (%r) -- say them "
                    "one at a time" % conj)
    return None


class Intent:
    """verb, target description, destination, arm -- plus why it failed."""

    def __init__(self, verb=None, target=None, arm=None, destination=None,
                 raw="", reason="", deictic=False):
        self.verb = verb
        self.target = target
        self.arm = arm
        self.destination = destination
        self.raw = raw
        self.reason = reason
        self.deictic = deictic

    @property
    def ok(self):
        return self.verb is not None

    def __repr__(self):
        return ("Intent(verb=%r target=%r arm=%r dest=%r deictic=%s%s)"
                % (self.verb, self.target, self.arm, self.destination,
                   self.deictic,
                   "" if self.ok else " reason=%r" % self.reason))

    def as_dict(self):
        return dict(verb=self.verb, target=self.target, arm=self.arm,
                    destination=self.destination, deictic=self.deictic,
                    raw=self.raw, reason=self.reason)


def normalise(text):
    t = (text or "").lower().strip()
    t = re.sub(r"[^\w\s]", " ", t)
    t = re.sub(r"\b(please|could you|can you|now|the|a|an|um|uh)\b", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def strip_wake(text, wake=WAKE_DEFAULT):
    """(heard_wake, remainder). The wake word may be anywhere in the phrase;
    people say it before AND after the command."""
    t = normalise(text)
    w = normalise(wake)
    if not w:
        return True, t
    if w not in t:
        # tolerate the commonest mis-transcriptions of a nonsense wake phrase
        alts = [w.replace("doc oc", x) for x in
                ("doc ock", "dock oc", "doctor oc", "doc og", "docaugh")]
        for alt in alts:
            if alt in t:
                return True, re.sub(re.escape(alt), " ", t).strip()
        return False, t
    return True, re.sub(re.escape(w), " ", t).strip()


def extract_target(text):
    """(description, is_deictic). Description is a detector PROMPT."""
    words = text.split()
    colour = next((w for w in words if w in COLOURS), None)
    noun = next((w for w in words if w in NOUNS), None)
    if any(d in text for d in ("over there", "that thing", "that one")) \
            or (noun in ("thing", "one", "object") and colour is None):
        return (("%s %s" % (colour, noun)).strip() if (colour or noun)
                else None), True
    if colour and noun:
        return "%s %s" % (colour, noun), False
    if colour:
        return colour + " object", False
    if noun and noun not in ("thing", "one", "object"):
        return noun, False
    return None, False


def extract_arm(text):
    if re.search(r"\bleft\b", text) and not re.search(r"\bother\b", text):
        return "left"
    if re.search(r"\bright\b", text) and not re.search(r"\bother\b", text):
        return "right"
    return None


def parse(text, wake=WAKE_DEFAULT, require_wake=True):
    """Utterance -> Intent. Never raises; refusal is a returned reason."""
    raw = text or ""
    heard, body = strip_wake(raw, wake)

    # STOP bypasses the wake word. Requiring a wake word before "stop" would
    # put a word between the operator and halting a moving arm.
    if VERB_PATTERNS[0][1].search(normalise(raw)):
        return Intent(verb="stop", raw=raw)

    if require_wake and not heard:
        return Intent(raw=raw, reason="no wake word (%r)" % wake)
    if not body:
        return Intent(raw=raw, reason="wake word only, no command")

    # REFUSE BEFORE MATCHING, NOT AFTER. Every one of these sentences contains
    # a perfectly good verb and a perfectly good noun; the danger is precisely
    # that the grammar succeeds on them. Checking after the verb match would
    # mean the refusal competes with a confident parse instead of pre-empting
    # it. Note this sits AFTER the stop check above -- a halt is never refused
    # for being ungrammatical.
    why = unrepresentable(body)
    if why:
        return Intent(raw=raw, reason=why)

    for verb, rx in VERB_PATTERNS[1:]:
        if rx.search(body):
            target, deictic = extract_target(body)
            arm = extract_arm(body)
            if verb == "grab" and target is None:
                return Intent(raw=raw, deictic=deictic,
                              reason="heard 'grab' but no target description")
            if verb == "handover":
                return Intent(verb="handover", arm=arm, raw=raw)
            if verb == "handover_wearer":
                return Intent(verb="handover_wearer", arm=arm, raw=raw)
            if verb == "goto":
                place = next((k for k in NAMED_PLACES if k in body), None)
                if place is None:
                    return Intent(raw=raw,
                                  reason="heard 'move to' but no place I know")
                spec = NAMED_PLACES[place]
                if not spec["reachable"]:
                    return Intent(raw=raw, reason=spec["why"])
                return Intent(verb="goto", target=place,
                              arm=spec.get("arm"), raw=raw)
            if verb == "place":
                dest = "down"
                if "back" in body:
                    dest = "back"
                elif "here" in body:
                    dest = "here"
                return Intent(verb="place", destination=dest, arm=arm, raw=raw)
            return Intent(verb=verb, target=target, arm=arm, raw=raw,
                          deictic=deictic)
    return Intent(raw=raw, reason="no known verb in %r" % body)


# --------------------------------------------------------------- targets
AMBIGUOUS = "ambiguous"
NO_MATCH = "no_match"
UNIQUE = "unique"


def resolve_target(description, objects, min_confidence=0.0):
    """(status, chosen, candidates).

    `objects` is a list of dicts with at least `label` and `confidence`, and
    optionally `position` (world xyz) so a spatial qualifier can disambiguate.

    NEVER returns the first of several matches. If the operator said "the
    green cube" and there are two, the correct behaviour is to ASK -- and
    asking is only possible if this refuses to choose.
    """
    if not description:
        return NO_MATCH, None, []
    want = set(normalise(description).split())
    cands = []
    for o in objects or []:
        if float(o.get("confidence", 0.0)) < min_confidence:
            continue
        have = set(normalise(str(o.get("label", ""))).split())
        # every content word the operator said must appear in the label,
        # except the generic fallbacks which match anything
        need = {w for w in want if w not in ("object", "thing", "one")}
        if need and not need <= have:
            continue
        cands.append(o)
    if not cands:
        return NO_MATCH, None, []
    if len(cands) == 1:
        return UNIQUE, cands[0], cands
    return AMBIGUOUS, None, cands


def spatial_disambiguate(qualifier, candidates):
    """'the left one' / 'the far one' over an ambiguous set.

    +x is the wearer's RIGHT and +y is FORWARD in this repo's world frame --
    NOT the ROS x-forward convention. Getting that backwards would hand the
    operator the object on the opposite side while sounding correct.
    """
    if not candidates or not qualifier:
        return None
    q = normalise(qualifier)
    keyed = [c for c in candidates if c.get("position")]
    if not keyed:
        return None
    if "left" in q:
        return min(keyed, key=lambda c: c["position"][0])
    if "right" in q:
        return max(keyed, key=lambda c: c["position"][0])
    if "far" in q or "back" in q:
        return max(keyed, key=lambda c: c["position"][1])
    if "near" in q or "close" in q or "front" in q:
        return min(keyed, key=lambda c: c["position"][1])
    return None


def question_for(description, candidates):
    """What the robot SAYS when it will not guess."""
    n = len(candidates)
    sides = []
    for c in candidates:
        p = c.get("position")
        sides.append("left" if (p and p[0] < 0) else
                     ("right" if p else "one"))
    if n == 2 and set(sides) == {"left", "right"}:
        return ("I see two things matching %r, on the left and on the right. "
                "Which one?" % description)
    return ("I see %d things matching %r. Which one -- left, right, near or "
            "far?" % (n, description))
