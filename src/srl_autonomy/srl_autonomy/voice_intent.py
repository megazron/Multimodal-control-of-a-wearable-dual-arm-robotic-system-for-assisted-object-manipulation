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
# PICK AND PLACE FROM ONE SENTENCE, ADDED 2026-08-17.
#
# T1 is "each cube goes on the plane of its own colour", and until now the
# grammar could not say that. Measured against the shipped parser before this
# block existed:
#
#   "pick up the blue cube and put it on the blue pad"
#        -> grab / target "blue cube" / destination None
#           -- the destination was SILENTLY DROPPED, which is the dangerous
#              category: half a two-part instruction executed confidently
#   "put the green ones on the green mat"
#        -> refused, "no known verb" (there is no bare put/place rule without
#           a down/here/there/back particle, and "ones" is not a noun)
#   "move that blue block to its colour"
#        -> refused, "heard 'move to' but no place I know" (goto claimed it)
#
# So one of the three was MISUNDERSTOOD and two were refused. The verb below
# is what represents them.
#
# THE DESTINATION IS PART OF THE SENTENCE, AND THE TARGET IS EXTRACTED FROM
# THE HEAD ONLY. That split is not tidiness: `extract_target` takes the FIRST
# colour in the sentence, so on "put it on the blue pad" it would have made
# "blue" the TARGET's colour when blue describes the destination. The arm
# would then have gone looking for a blue object nobody named.
DEST_CUES = ("on", "onto", "into", "in", "to", "over", "at")

# Surfaces a cube can be put ON. Deliberately NOT added to NOUNS: the
# two-targets check counts distinct nouns, and "put the cube on the pad" would
# have become "more than one target in one instruction" -- a refusal for a
# sentence that names exactly one object and one place.
SURFACE_NOUNS = ("pad", "pads", "mat", "mats", "plane", "planes", "square",
                 "squares", "patch", "patches", "tile", "tiles", "target",
                 "targets", "spot", "spots", "zone", "zones", "area", "areas",
                 "board", "boards", "mark", "marker", "circle", "rectangle",
                 "sheet", "card", "side", "colour", "color")

# "put it where it belongs" -- the destination is a FUNCTION of what the
# camera saw, not a place. This is the phrasing T1 is actually about and the
# reason the plan cannot be built before the look.
DEST_MATCH_PHRASES = (
    "its colour", "its color", "its own colour", "its own color",
    "own colour", "own color", "matching colour", "matching color",
    "same colour", "same color", "colour that matches", "color that matches",
    "right colour", "right color", "correct colour", "correct color",
    "matching pad", "matching mat", "matching one", "its pad", "its mat",
    "own pad", "own mat", "colour match", "color match",
    "colour they match", "where they belong", "where it belongs",
    "where they go", "where it goes", "by colour", "by color",
    "colour matched pad", "pad of same colour", "plane of its own colour")

PLURAL_NOUNS = {"cubes": "cube", "blocks": "block", "boxes": "box",
                "balls": "ball", "spheres": "sphere", "cylinders": "cylinder",
                "objects": "object", "things": "thing", "ones": "one",
                "bottles": "bottle", "cups": "cup", "mugs": "mug",
                "trays": "tray", "tools": "tool"}

# "all of them", not "one of them". A singular request that matches more than
# one object must ASK; a plural one must not.
ALL_WORDS = ("all", "every", "each", "both", "everything")

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
    # PUT X ON Y. Sits above `place` and `goto` because both would otherwise
    # claim it: "put it on the blue pad" has `put`, and "move it to its
    # colour" has `move ... to`. It only fires when a destination can actually
    # be EXTRACTED (see parse()), so "put it down" still reaches `place` and
    # "move to the front centre" still reaches `goto` and its measured
    # refusal.
    ("put_on", re.compile(
        r"\b(put|place|move|set|drop|pop|shift|transfer|stack|position|"
        r"leave|return|send|deliver|sort|pick|grab|take|lift|get)\b")),
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
# THE NAMES ONLY. The GEOMETRY lives in srl_autonomy.named_places, which
# reads the survey, and this module deliberately does not import it: the
# parser is pure text and is tested without a workspace on disk.
#
# THE REFUSAL TEXT USED TO BE WRITTEN OUT HERE AND IT WENT STALE. It said
# "0 of 319 surveyed cells ... the nearest reachable x is 0.30", which was the
# bench-era survey; the current one has 409 cells and the nearest reachable x
# is 0.15. Both statements refuse the same command for the same reason, and
# one of them quoted a number that no longer described this robot. So the
# parser now says only THAT the front centre is unreachable, and the executive
# attaches the measurement when it resolves the place -- one owner for the
# number, which is the whole point of named_places.
NAMED_PLACES = {
    "front centre": dict(reachable=False,
                         why="the front centre is not reachable by either "
                             "arm; ask the executive for the survey figures"),
    "front center": dict(reachable=False,
                         why="the front centre is not reachable by either "
                             "arm; ask the executive for the survey figures"),
    "left side": dict(reachable=True, arm="left"),
    "right side": dict(reachable=True, arm="right"),
    "home": dict(reachable=True, arm=None),
}

# The literal trigger words of the patterns above, used only to locate WHERE
# in the sentence the verb sits so negation can be scoped to it.
VERB_TRIGGERS = frozenset(
    "give hand pass transfer swap put set place drop release let "
    "grab pick take get grasp fetch lift".split())


def _dest_from(tail):
    """The words after a destination cue -> a destination, or None.

    None is the important return. It is what keeps "put it down", "hand it to
    me" and "move to the front centre" on the verbs that already handle them,
    instead of being swallowed by a pick-and-place rule that would then have
    nowhere to put anything.
    """
    t = " ".join(tail)
    for ph in DEST_MATCH_PHRASES:
        if ph in t:
            return dict(kind="match", colour=None, phrase=ph)
    colour = next((w for w in tail if w in COLOURS), None)
    if colour is None:
        return None
    rest = tail[tail.index(colour) + 1:]
    # "on the blue pad" / "on the blue one" / "onto blue" are places.
    # "on the blue CUBE" is stacking one object on another, which this rig
    # cannot do and which must not be quietly turned into a pad.
    if rest and (rest[0] in NOUNS or rest[0] in PLURAL_NOUNS) \
            and rest[0] not in ("one", "ones", "object", "thing"):
        return None
    return dict(kind="colour", colour=colour, phrase=t)


def split_destination(body):
    """(head, destination). `head` is the sentence with the place clause cut
    off, so the TARGET is never read out of the DESTINATION's words."""
    toks = body.split()
    for i, t in enumerate(toks):
        if t in DEST_CUES and i + 1 < len(toks):
            d = _dest_from(toks[i + 1:])
            if d:
                return " ".join(toks[:i]), d
    # A COLOUR-MATCH DESTINATION NEEDS NO PREPOSITION. "put every cube where
    # it belongs" and "sort them by colour" name a place without an `on` or a
    # `to`, and requiring a cue refused both of them.
    for ph in DEST_MATCH_PHRASES:
        j = body.find(ph)
        if j >= 0:
            head = body[:j].rstrip()
            # trim a dangling connective the phrase was hanging off
            for w in ("where", "by", "into", "in", "onto", "on", "to", "so",
                      "that", "they", "it"):
                if head.endswith(" " + w):
                    head = head[:-(len(w) + 1)].rstrip()
            return head, dict(kind="match", colour=None, phrase=ph)
    return body, None


# --------------------------------------------------------------------------
# MISSPELLINGS, AND WHY THE REPAIR IS DELIBERATELY TIMID.
#
# A typed instruction is typed by a person under time pressure beside a robot,
# so "gren", "cubbe" and "pik up" all happen. Repairing them is worth doing
# and is also the easiest way to manufacture the one outcome that matters:
# a confident parse of a word the operator did not write.
#
# So the repair only fires when the answer is FORCED:
#   * the token is not already vocabulary;
#   * it is at least four characters (three-letter tokens are one edit from
#     half the language);
#   * exactly ONE vocabulary word is within Damerau-Levenshtein distance 1.
#     Two candidates is a tie and a tie is left alone, so the sentence goes on
#     to be refused rather than resolved by alphabetical accident.
# Every repair is RECORDED on the intent, so a caller can say "I heard green"
# and the sweep can score a repaired parse separately from a clean one.
_FUNCTION_WORDS = ("its", "it", "own", "same", "matching", "match", "that",
                   "this", "them", "they", "belongs", "belong", "where",
                   "here", "there", "back", "down", "up", "over", "onto",
                   "into", "left", "right", "other", "arm", "hand", "me",
                   "wearer", "front", "centre", "center", "side", "near",
                   "far", "correct", "colour", "color", "and", "then",
                   "not", "dont", "don", "please", "you", "your")


def _known():
    """Words that are already correct and must be left exactly as written."""
    return (set(COLOURS) | set(NOUNS) | set(PLURAL_NOUNS)
            | set(SURFACE_NOUNS) | set(DEST_CUES) | set(ALL_WORDS)
            | set(VERB_TRIGGERS) | set(STOP_WORDS) | set(_FUNCTION_WORDS))


def _repair_targets():
    """Words a typo may be repaired INTO. A STRICT SUBSET of _known().

    VERBS AND STOP WORDS ARE DELIBERATELY ABSENT, and that is a measured
    constraint rather than an oversight: at Damerau-Levenshtein 1, `top` is a
    STOP, `crop` is a DROP and `let`, `yet`, `net` and `bet` are all a GET or a
    SET. The verb decides the ACTION, so a near-miss there is the one failure
    this module exists to prevent, and
    `test_verbs_are_matched_EXACTLY_and_here_is_why` pins it. A misspelled verb
    refuses outright; a misspelled NOUN or COLOUR degrades to a question.

    THE TWO SETS MUST BE SEPARATE AND THE FIRST ATTEMPT MADE THEM ONE. Dropping
    the verbs from the vocabulary entirely meant a CORRECTLY SPELLED verb was
    no longer recognised as a word, so `grab` -- one edit from `gray` -- was
    repaired into a colour and ten tests went red at once. "Never repair a verb"
    and "never touch a verb" are different rules and both are needed.
    """
    return (set(COLOURS) | set(NOUNS) | set(PLURAL_NOUNS)
            | set(SURFACE_NOUNS) | set(DEST_CUES) | set(ALL_WORDS)
            | set(_FUNCTION_WORDS))


def _damerau(a, b):
    """Edit distance with transposition. 'bleu' -> 'blue' is ONE edit here and
    two under plain Levenshtein, and it is the commonest colour typo there is."""
    la, lb = len(a), len(b)
    d = [[0] * (lb + 1) for _ in range(la + 1)]
    for i in range(la + 1):
        d[i][0] = i
    for j in range(lb + 1):
        d[0][j] = j
    for i in range(1, la + 1):
        for j in range(1, lb + 1):
            c = 0 if a[i - 1] == b[j - 1] else 1
            d[i][j] = min(d[i - 1][j] + 1, d[i][j - 1] + 1, d[i - 1][j - 1] + c)
            if i > 1 and j > 1 and a[i - 1] == b[j - 2] and a[i - 2] == b[j - 1]:
                d[i][j] = min(d[i][j], d[i - 2][j - 2] + 1)
    return d[la][lb]


def _stem_to_known(tok, vocab):
    """'grabbing' -> 'grab', 'cubes' stays 'cubes'. Only ever returns a word
    that is already vocabulary, so it cannot invent one."""
    if tok in vocab:
        return None
    cands = []
    if tok.endswith("ing"):
        cands += [tok[:-3], tok[:-4]]           # grabbing -> grabb -> grab
    if tok.endswith("ed"):
        cands += [tok[:-2], tok[:-3]]
    if tok.endswith("s"):
        cands.append(tok[:-1])
    for c in cands:
        if len(c) >= 3 and c in vocab:
            return c
    return None


def repair(body, vocab=None):
    """(repaired_body, [(was, now), ...], [(was, [candidates]), ...]).

    The third return is the TIES -- tokens one edit from more than one
    vocabulary word. "gren" is one edit from both `green` and `grey`, and
    picking either would be a confident answer to a question the operator has
    not been asked. The caller turns a tie into a question naming the
    alternatives, which is a better outcome than a bare refusal and a much
    better one than a guess. Never invents a word.
    """
    known = _known()
    vocab = vocab or _repair_targets()
    out, fixes, ties = [], [], []
    for tok in body.split():
        if tok in known or tok.isdigit():
            out.append(tok)
            continue
        stem = _stem_to_known(tok, vocab)
        if stem:
            out.append(stem)
            fixes.append((tok, stem))
            continue
        if len(tok) < 4:
            out.append(tok)
            continue
        near = [w for w in vocab if abs(len(w) - len(tok)) <= 1
                and _damerau(tok, w) <= 1]
        if len(near) == 1:
            out.append(near[0])
            fixes.append((tok, near[0]))
        else:
            # 0 candidates -> an unknown word, left alone so the sentence is
            # refused. 2+ -> a TIE, and choosing one of a tie is exactly the
            # confident-wrong-answer this whole module is built to avoid.
            out.append(tok)
            if len(near) > 1:
                ties.append((tok, sorted(near)))
    return " ".join(out), fixes, ties


def extract_quantity(head):
    """'all' or 1. A singular request that matches several objects must ASK;
    a plural one must not, and the difference is in the words."""
    toks = head.split()
    if any(t in ALL_WORDS for t in toks):
        return "all"
    if any(t in PLURAL_NOUNS for t in toks):
        return "all"
    return 1


def unrepresentable(body, head=None):
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
        #
        # COUNTED IN THE HEAD, NOT THE WHOLE SENTENCE, when a destination was
        # split off. "pick up the blue cube and put it on the GREEN pad" names
        # one target and one place; counting the place's colour as a second
        # target refused a perfectly ordinary instruction. The head is the
        # sentence with the place clause removed, so the count is over the
        # words that actually describe objects.
        scope = (head if head is not None else body).split()
        colours = {t for t in scope if t in COLOURS}
        nouns = {t for t in scope if t in NOUNS or t in PLURAL_NOUNS}
        nouns = {PLURAL_NOUNS.get(t, t) for t in nouns}
        if len(colours) > 1 or len(nouns) > 1:
            return ("more than one target in one instruction (%r) -- say them "
                    "one at a time" % conj)
    return None


class Intent:
    """verb, target description, destination, arm -- plus why it failed."""

    def __init__(self, verb=None, target=None, arm=None, destination=None,
                 raw="", reason="", deictic=False, quantity=1,
                 corrections=None):
        self.verb = verb
        self.target = target
        self.arm = arm
        self.destination = destination
        # 1 or "all". A singular request matching several objects must ASK.
        self.quantity = quantity
        # [(was, now)] from repair(). Carried so a caller can confirm what it
        # heard, and so a sweep can score a repaired parse separately.
        self.corrections = list(corrections or [])
        # [(token, [candidates])] -- typos one edit from MORE THAN ONE
        # vocabulary word. Left unrepaired on purpose; see repair().
        self.ambiguous_words = []
        self.raw = raw
        self.reason = reason
        self.deictic = deictic

    @property
    def ok(self):
        return self.verb is not None

    def __repr__(self):
        return ("Intent(verb=%r target=%r arm=%r dest=%r qty=%r deictic=%s%s%s)"
                % (self.verb, self.target, self.arm, self.destination,
                   self.quantity, self.deictic,
                   "" if not self.corrections else " fixed=%r" % self.corrections,
                   "" if self.ok else " reason=%r" % self.reason))

    def as_dict(self):
        return dict(verb=self.verb, target=self.target, arm=self.arm,
                    destination=self.destination, deictic=self.deictic,
                    quantity=self.quantity, corrections=self.corrections,
                    ambiguous_words=self.ambiguous_words,
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
    """(description, is_deictic). Description is a detector PROMPT.

    PLURALS ARE SINGULARISED HERE. "sort the cubes onto their matching
    colours" named its target perfectly clearly and was refused with "no
    description of WHAT to put there", because `cubes` is not in NOUNS. The
    COUNT is not lost by this -- `extract_quantity` reads the plural off the
    same head -- so the two questions ("what" and "how many") are answered
    separately instead of one of them silently deciding the other.
    """
    words = [PLURAL_NOUNS.get(w, w) for w in text.split()]
    text = " ".join(words)
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


def parse(text, wake=WAKE_DEFAULT, require_wake=True, fix_spelling=True):
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

    # TYPOS, REPAIRED ONLY WHERE THE ANSWER IS FORCED. See repair(): a token
    # one edit from exactly one vocabulary word is corrected and the
    # correction is recorded; a tie is left alone so the sentence is refused
    # rather than resolved by accident.
    fixes, ties = [], []
    if fix_spelling:
        body, fixes, ties = repair(body)

    def Intent_(**kw):
        """Every intent out of this function carries the repairs and the ties.

        A TIE IS NOT A REFUSAL HERE, and that is on purpose. `gren` is one
        edit from both `green` and `grey`; the word is simply left alone, so it
        falls out of the target description and `resolve_target` asks -- which
        is the repository's one asking mechanism. Recording the tie lets a
        caller name the alternatives ("did you mean green or grey?") instead of
        asking a vaguer question, without inventing a second place that asks.
        """
        kw.setdefault("corrections", fixes)
        i = Intent(**kw)
        i.ambiguous_words = list(ties)
        return i

    # WHERE THE THING GOES, SPLIT OFF BEFORE ANYTHING READS THE TARGET.
    head, dest = split_destination(body)

    # REFUSE BEFORE MATCHING, NOT AFTER. Every one of these sentences contains
    # a perfectly good verb and a perfectly good noun; the danger is precisely
    # that the grammar succeeds on them. Checking after the verb match would
    # mean the refusal competes with a confident parse instead of pre-empting
    # it. Note this sits AFTER the stop check above -- a halt is never refused
    # for being ungrammatical.
    why = unrepresentable(body, head=head if dest else None)
    if why:
        return Intent_(raw=raw, reason=why, corrections=fixes)

    for verb, rx in VERB_PATTERNS[1:]:
        if rx.search(body):
            if verb == "put_on":
                # ONLY when a destination was actually extracted. Otherwise
                # this rule would claim "put it down" and "move to the front
                # centre" and have nowhere to put anything.
                if dest is None:
                    continue
                target, deictic = extract_target(head)
                if target is None and any(
                        d in head.split() for d in ("it", "that", "this",
                                                    "them", "these", "those")):
                    deictic = True
                if target is None and not deictic:
                    return Intent_(
                        raw=raw, corrections=fixes,
                        reason="heard a place to put something but no "
                               "description of WHAT to put there")
                return Intent_(verb="put_on", target=target,
                              arm=extract_arm(head), destination=dest,
                              quantity=extract_quantity(head),
                              deictic=deictic, raw=raw, corrections=fixes)
            target, deictic = extract_target(body)
            arm = extract_arm(body)
            if verb == "grab" and target is None:
                return Intent_(raw=raw, deictic=deictic, corrections=fixes,
                              reason="heard 'grab' but no target description")
            if verb == "handover":
                return Intent_(verb="handover", arm=arm, raw=raw,
                              corrections=fixes)
            if verb == "handover_wearer":
                return Intent_(verb="handover_wearer", arm=arm, raw=raw,
                              corrections=fixes)
            if verb == "goto":
                place = next((k for k in NAMED_PLACES if k in body), None)
                if place is None:
                    return Intent_(raw=raw, corrections=fixes,
                                  reason="heard 'move to' but no place I know")
                spec = NAMED_PLACES[place]
                if not spec["reachable"]:
                    return Intent_(raw=raw, reason=spec["why"],
                                  corrections=fixes)
                return Intent_(verb="goto", target=place,
                              arm=spec.get("arm"), raw=raw,
                              corrections=fixes)
            if verb == "place":
                _d = "down"
                if "back" in body:
                    _d = "back"
                elif "here" in body:
                    _d = "here"
                return Intent_(verb="place", destination=_d, arm=arm,
                              raw=raw, corrections=fixes)
            return Intent_(verb=verb, target=target, arm=arm, raw=raw,
                          deictic=deictic, corrections=fixes)
    if ties:
        # SAY WHAT WAS UNREADABLE. "no known verb in 'put gren cube on gren
        # pad'" is true and names the wrong thing: the verb was fine and a
        # COLOUR was the word that could not be read. A refusal that misnames
        # its own cause sends the operator to fix the wrong half of the
        # sentence.
        t, cands = ties[0]
        return Intent_(raw=raw, corrections=fixes,
                       reason="I could not read %r -- did you mean %s?"
                              % (t, " or ".join(cands)))
    # SAY THAT THE PLACE WAS MISSING, NOT THAT THE VERB WAS.
    #
    # "put the cube on the pad" and "put the blue ones on the left pad" both
    # came back as "no known verb", which is false twice over: `put` is a verb
    # this grammar knows, and what actually failed was the DESTINATION -- no
    # colour, no match phrase, so nothing to put anything on. A refusal that
    # misnames its own cause sends the operator to rewrite the half of the
    # sentence that was already right.
    if dest is None and VERB_PATTERNS[1][1].search(body):
        return Intent_(
            raw=raw, corrections=fixes,
            reason="I heard something to move but not WHERE to put it. Name "
                   "a colour ('on the blue pad') or say 'on its own colour'.")

    return Intent_(raw=raw, reason="no known verb in %r" % body,
                  corrections=fixes)


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
