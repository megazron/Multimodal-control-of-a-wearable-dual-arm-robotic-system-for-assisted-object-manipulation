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
# THE SUBSET OF SURFACE_NOUNS THAT CAN ONLY MEAN ONE OF T1'S TWO PADS.
#
# `side` and `colour` are surfaces in "on the left side" and "on its colour"
# and neither is a pad, so an unqualified destination built from them steals
# "move to the left side" from the `goto` verb and its measured refusal.
# Measured, by the two named-place tests going red the moment they were
# included.
PAD_NOUNS = ("pad", "pads", "mat", "mats", "plane", "planes", "square",
             "squares", "patch", "patches", "tile", "tiles", "spot", "spots")

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
# SELECTORS: THE WORDS THAT PICK ONE OUT OF SEVERAL, ADDED 2026-08-18.
#
# "put the leftmost blue cube on the blue pad" names exactly one cube in a
# scene with two blue ones, and the grammar had no way to say so: `leftmost`
# fell out of the target description and the sentence came back as a question
# about which blue cube was meant. Asking a question the operator has already
# answered is not dangerous, but it is the difference between a robot that
# takes instructions and one that takes three.
#
# THEY ARE RESOLVED AGAINST WHAT THE CAMERA SAW, NOT AGAINST THE TASK FILE, and
# the resolution lives in `t1_instruction` where the detections are. This
# module only says WHICH selector was uttered.
#
# THE DANGEROUS ONE IS THE SELECTOR THAT CANNOT BE RESOLVED. "the biggest cube"
# is a perfectly good English selector over a scene of four identical 40 mm
# cubes, and the only honest answers are "they are all the same size" or a
# question -- never a confident choice. So a selector carries its KIND, and a
# kind the scene cannot separate is refused by name rather than dropped.
SELECTOR_SPATIAL = {
    "leftmost": "left", "rightmost": "right", "left": "left", "right": "right",
    "nearest": "near", "closest": "near", "near": "near", "nearer": "near",
    "furthest": "far", "farthest": "far", "far": "far", "further": "far",
    "outermost": "outer", "outer": "outer", "innermost": "inner",
    "inner": "inner", "middle": "middle", "centre": "middle",
    "center": "middle",
}
SELECTOR_ORDINAL = {"first": 1, "second": 2, "third": 3, "fourth": 4,
                    "last": -1, "next": None}
# Size and shape words. They PARSE and they cannot be GROUNDED in this scene,
# which is a different thing from not being understood, and the difference is
# what the operator needs told.
SELECTOR_UNGROUNDABLE = ("biggest", "largest", "smallest", "tiniest",
                         "heaviest", "lightest", "tallest", "shortest",
                         "widest", "narrowest", "roundest", "squarest",
                         "newest", "oldest", "best", "worst", "nicest")
# "either", "any", "whichever" -- the operator has said the choice does not
# matter. That is not ambiguity, it is permission, and treating it as
# ambiguity asks a question whose answer has already been given.
SELECTOR_ANY = ("either", "any", "whichever", "whatever", "one of", "anyone")

SELECTOR_WORDS = tuple(sorted(
    set(SELECTOR_SPATIAL) | set(SELECTOR_ORDINAL)
    | set(SELECTOR_UNGROUNDABLE)
    | {w for ph in SELECTOR_ANY for w in ph.split()}))

# THE WORDS THAT MEAN "SORT IT OUT" WITHOUT SAYING HOW. A person looking at
# four cubes and two coloured pads and saying "tidy up" means something, and
# the something is obvious to a person and a GUESS to a robot. The grammar
# represents the utterance and the GROUNDING layer offers the reading back as
# a question, so the operator confirms an interpretation instead of the robot
# assuming one.
TIDY_PHRASES = ("tidy up", "tidy this up", "tidy them up", "tidy", "clean up",
                "clear up", "sort it out", "sort them out", "sort this out",
                "sort everything out", "put everything away", "reset the table",
                "do the task", "do the cube thing", "carry on", "get on with it")

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
        # A PLACE WITH NO COLOUR IS STILL A PLACE. "put the cube on the pad"
        # names a destination and not WHICH destination, and there are two of
        # them -- so it is a question for the grounding layer, not a refusal
        # here. Measured: it came back as "I heard something to move but not
        # WHERE to put it", which is false; the operator said where, they just
        # did not say which one.
        #
        # ONLY when a SURFACE noun was actually uttered. "put it down" and
        # "put it back" must still fall through to `place`, which handles
        # them, and "on the table" is not a pad.
        if any(w in PAD_NOUNS for w in tail):
            return dict(kind="unspecified", colour=None, phrase=" ".join(tail))
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
            | set(VERB_TRIGGERS) | set(STOP_WORDS) | set(_FUNCTION_WORDS)
            | set(SELECTOR_WORDS) | set(SEQUENCE_WORDS)
            | set(CONJUNCTION_WORDS) | set(NEGATION_WORDS))


# WORDS A TYPO MUST NEVER BE REPAIRED INTO, for the same reason verbs are not.
#
# The rule already in force is "never repair a verb", because the verb decides
# the ACTION and a near miss there is the failure this module exists to
# prevent. These words decide whether the sentence is REFUSED at all -- a
# negation, a second action, a second target, a relational reference -- so a
# near miss on one of them manufactures a refusal out of an ordinary sentence,
# or, worse, could clear one. `when` -> `then` is the measured case: it turned
# a polite instruction into "more than one action in one instruction".
#
# They stay in `_known()`, so a correctly spelled `then` is still recognised
# and still refuses. "Never repair into it" and "never recognise it" are
# different rules, and this file has already paid for confusing them once.
NEVER_REPAIR_INTO = frozenset(
    set(NEGATION_WORDS) | set(SEQUENCE_WORDS) | set(CONJUNCTION_WORDS)
    | {w for phrase in RELATIONAL for w in phrase.split()})


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
    return ((set(COLOURS) | set(NOUNS) | set(PLURAL_NOUNS)
             | set(SURFACE_NOUNS) | set(DEST_CUES) | set(ALL_WORDS)
             | set(_FUNCTION_WORDS) | set(SELECTOR_WORDS))
            - NEVER_REPAIR_INTO)


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
        # A TIE BETWEEN TWO SPELLINGS OF ONE WORD IS NOT A TIE. `padd` is one
        # edit from both `pad` and `pads`, and those are the same noun -- so
        # asking "did you mean pad or pads?" is asking about a distinction the
        # sentence does not have. `gren` is one edit from `green` and `grey`,
        # which are two different colours, and that one stays a tie.
        #
        # Collapse by LEMMA and only then count. Measured: without this,
        # "put the blue ones on the blue padd" went from a plan to a question
        # the moment ties were asked about before parsing.
        if len(near) > 1:
            lemmas = {PLURAL_NOUNS.get(w, w) for w in near}
            if len(lemmas) == 1:
                near = [sorted(near, key=len)[0]]
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


def extract_selector(head):
    """Which ONE of several, or None. Read off the words, never from the scene.

    Returns a dict with `kind`:

        spatial      {"kind": "spatial", "which": "left"|"right"|"near"|
                     "far"|"inner"|"outer"|"middle"}
        ordinal      {"kind": "ordinal", "n": 1 | 2 | ... | -1}
        any          {"kind": "any"}          -- "either one", the choice is free
        ungroundable {"kind": "ungroundable", "word": "biggest"}

    LEFT AND RIGHT ARE ONLY SELECTORS WHEN THEY ARE NOT ARMS. "the left one"
    picks a cube; "the left arm" names an arm and this rig has two. The test is
    the next word, which is the same test `extract_arm` needs and the reason
    both live here rather than in the caller.
    """
    toks = head.split()
    for i, t in enumerate(toks):
        nxt = toks[i + 1] if i + 1 < len(toks) else ""
        if t in ("left", "right") and nxt in ("arm", "hand", "gripper", "side"):
            continue
        if t in SELECTOR_UNGROUNDABLE:
            return dict(kind="ungroundable", word=t)
        if t in SELECTOR_SPATIAL:
            return dict(kind="spatial", which=SELECTOR_SPATIAL[t])
        if t in SELECTOR_ORDINAL and SELECTOR_ORDINAL[t] is not None:
            return dict(kind="ordinal", n=SELECTOR_ORDINAL[t])
        if t in ("either", "any", "whichever", "whatever", "anyone"):
            return dict(kind="any")
    return None


def is_tidy(body):
    """The utterance that means "sort this out" and does not say how."""
    b = " %s " % body.strip()
    for ph in TIDY_PHRASES:
        if (" %s " % ph) in b or body.strip() == ph:
            return ph
    return None


def split_clauses(body):
    """A compound instruction -> its clauses, or [body] if it is not one.

    "put the blue ones on the blue pad and the green ones on the green pad" is
    TWO complete instructions sharing a verb, and it was the most dangerous
    sentence in the set: `split_destination` cut at the FIRST destination cue,
    so the head was "put the blue ones" and the conjunction check -- which
    counts targets in the head -- saw one target and passed it. The second half
    was dropped in silence and two cubes moved. Measured, 2026-08-18.

    A clause is only split off when BOTH sides carry their own destination. A
    conjunction with one destination between them ("put the blue cube and the
    green cube on the pads") is genuinely one instruction with two targets and
    stays unrepresentable, which is what `unrepresentable` already says.

    THE VERB IS CARRIED FORWARD. English drops it in the second clause -- "and
    the green ones on the green pad" has no verb of its own -- so each clause
    is reassembled with the leading verb of the first, which is the only verb
    that was uttered.
    """
    toks = body.split()
    if not any(t in ("and", "plus") for t in toks):
        return [body]
    parts, cur = [], []
    for t in toks:
        if t in ("and", "plus"):
            parts.append(cur)
            cur = []
        else:
            cur.append(t)
    parts.append(cur)
    if len(parts) < 2 or any(not p for p in parts):
        return [body]
    verb = parts[0][0] if parts[0] and parts[0][0] in VERB_TRIGGERS else None
    out = []
    for i, p in enumerate(parts):
        cl = list(p)
        if i and verb and (not cl or cl[0] not in VERB_TRIGGERS):
            cl = [verb] + cl
        text = " ".join(cl)
        head, dest = split_destination(text)
        if dest is None or not head.strip():
            return [body]
        out.append(text)
    return out


def unrepresentable(body, head=None):
    """Why this sentence cannot be executed as one command, or None.

    Returns a reason string. Each branch refuses rather than truncating: the
    caller turns it into `Intent(reason=...)`, which the executive speaks
    aloud, so the operator learns the sentence did not land.
    """
    toks = body.split()

    # RELATIONAL first: it is a property of the phrase, not of the verb.
    #
    # UNLESS THE ANCHOR IS THE PERSON. "the cube closest to me" is a
    # SUPERLATIVE about the speaker, not a relational reference to another
    # object -- there is no second object in it to be relative to. It was
    # being refused as "relational reference ('closest to')", which sends the
    # operator to rewrite a sentence that was already unambiguous. The
    # relational refusal is for "the one behind the red block", where
    # grounding by the words present returns the red block.
    ANCHOR_IS_PERSON = ("me", "you", "us", "myself", "yourself", "wearer",
                        "operator", "here")
    for w in RELATIONAL:
        j = body.find(w)
        if j < 0:
            continue
        after = body[j + len(w):].split()
        if after and after[0] in ANCHOR_IS_PERSON:
            continue
        return ("relational reference (%r) -- I cannot pick something out by "
                "where it is relative to another object" % w)

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
                 corrections=None, selector=None, clauses=None):
        self.verb = verb
        self.target = target
        self.arm = arm
        self.destination = destination
        # 1 or "all". A singular request matching several objects must ASK.
        self.quantity = quantity
        # WHICH ONE of several, read off the words -- see extract_selector.
        # None means the sentence did not narrow the set, which is a different
        # thing from narrowing it to everything.
        self.selector = selector
        # The clauses of a compound instruction, raw text, in order. Empty for
        # an ordinary one. A caller that ignores this executes the first
        # clause and nothing else, which is the failure the field exists for.
        self.clauses = list(clauses or [])
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
                    quantity=self.quantity, selector=self.selector,
                    clauses=list(self.clauses), corrections=self.corrections,
                    ambiguous_words=self.ambiguous_words,
                    raw=self.raw, reason=self.reason)


# POLITENESS AND FILLER, STRIPPED BEFORE ANYTHING READS THE SENTENCE.
#
# A person typing at a robot beside them writes "could you please just put the
# blue ones on the blue pad for me when you get a chance". Every word of that
# except the middle eight is social, and the grammar has no business seeing
# any of it -- MEASURED, on that exact sentence: "when you get a chance"
# reached the spelling repair, `when` is one Damerau edit from `then`, and the
# instruction came back REFUSED as "more than one action in one instruction".
# A politeness that turns into a refusal is worse than no politeness handling
# at all, because the operator cannot see what they did wrong.
#
# MULTI-WORD PHRASES FIRST, then single words, because "for me" must go before
# "me" is considered and "go ahead and" must go before the bare "and" is
# counted as a conjunction.
FILLER_PHRASES = (
    "when you get a chance", "when you have a chance", "when you get time",
    "if you do not mind", "if you dont mind", "if you don t mind",
    "would you mind", "would you please", "could you please", "can you please",
    "i would like you to", "i d like you to", "i want you to",
    "i would like", "i need you to", "can i get you to",
    "go ahead and", "at some point", "right now", "for me", "thank you",
    "no rush", "as soon as you can", "when you can", "if possible",
)
FILLER_WORDS = ("please", "could", "can", "would", "now", "the", "a", "an",
                "um", "uh", "just", "kindly", "thanks", "ok", "okay", "hey",
                "hi", "so", "well", "maybe", "perhaps")


def normalise(text):
    """Punctuation and articles only. NOT the filler -- see strip_filler.

    THE WAKE MATCHER USES THIS, and that is why the filler stripping is a
    separate function rather than more words in the list below. `hey` is
    filler in a command and half the wake phrase in "hey doc oc"; stripping it
    here made the wake phrase normalise to "doc oc", which moved every
    edit-distance in `wake_distance` by two and started accepting "we should
    stop for lunch soon" as a wake utterance. Measured by the wake-word tests,
    which is exactly what they are for.
    """
    t = (text or "").lower().strip()
    t = re.sub(r"[^\w\s]", " ", t)
    t = re.sub(r"\b(please|could you|can you|now|the|a|an|um|uh)\b", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def strip_filler(t):
    """The social half of a typed instruction, removed before parsing.

    Applied to the COMMAND BODY only, after the wake word has been taken off.
    """
    t = " %s " % (t or "").strip()
    for ph in FILLER_PHRASES:
        t = t.replace(" %s " % ph, " ")
    t = re.sub(r"\b(%s)\b" % "|".join(FILLER_WORDS), " ", t)
    return re.sub(r"\s+", " ", t).strip()


# HOW FAR A TRANSCRIBED WAKE PHRASE MAY BE FROM THE WRITTEN ONE.
#
# MEASURED, NOT CHOSEN. `scripts/measure_wake_word.py` speaks the wake phrase
# through Piper, transcribes it with the same faster-whisper `small`/int8
# model `voice_listener` loads, and compares what comes back with what was
# said. Over 40 spoken instructions the transcriber wrote the wake phrase
# EXACTLY ZERO times and produced, among others:
#
#     Hey Doc Ock   Hey Duck Ock   Hey duckuck   Hey duck-ock   Hey duck, ox
#     Hey duck, uck   Hayduck Ock   Hey duck-cock   Hey duck, arc   Hey Doc
#
# Compared letters-only against `heydococ`, those sit 1 to 4 edits away, and
# the alias list this function used to carry -- five hand-guessed spellings --
# caught 13 of 40. The other 27 were refused with "no wake word" and one of
# them was a plain, correct instruction. A wake word is the first thing
# between an operator and a robot, and one that only works when a speech
# engine spells a joke the way the author did is not a wake word.
#
# THE THRESHOLD IS A TRADE AND HERE IS THE TABLE IT WAS CHOSEN FROM. Both
# sides measured on the same 95 spoken utterances -- 40 with the wake phrase,
# 55 without (the same instructions spoken bare, plus overheard chatter):
#
#     edits   wake accepted        no-wake FALSELY accepted
#       0       0 of 40    0%          0 of 55    0%
#       1      14 of 40   35%          0 of 55    0%
#       2      17 of 40   42%          1 of 55    2%
#       3      38 of 40   95%          1 of 55    2%      <-- shipped
#       4      39 of 40   98%          2 of 55    4%
#       5      39 of 40   98%          9 of 55   16%
#       6      40 of 40  100%         21 of 55   38%
#
# 3 is where the wake side saturates: it buys 21 more real wake utterances
# over threshold 2 for no extra false accept at all. Above it the false-accept
# curve turns over sharply.
#
# THE ONE FALSE ACCEPT AT 3 IS NOT A NEAR-MISS, IT IS THE PHRASE. It is
# "hey doc how long have you been running that", heard as "Hey Doc, how long
# have you been running that?" -- a person saying "hey doc", two edits from
# "hey doc oc". No threshold separates those, because they are the same words.
#
# SO THE REAL FINDING IS ABOUT THE WAKE PHRASE, NOT THE MATCHER: "hey doc oc"
# is a pun on ordinary English and cannot be made both reliable and safe. A
# wake phrase should be phonetically distinctive and not a prefix of things
# people say. `scripts/measure_wake_word.py --wake "..."` is how a replacement
# gets chosen, and it must be re-run against REAL speakers before any number
# here is called validated -- one synthetic voice is a pipeline test.
WAKE_MAX_EDITS = 3
# How many leading (or trailing) tokens may be consumed by the fuzzy match. A
# two-token wake phrase can arrive as one token ("duckuck") or as three
# ("duck", "ock", ","), so the window has to be searched rather than fixed.
WAKE_MAX_TOKENS = 4


def _edits(a, b):
    """Levenshtein distance. Small strings; the simple table is fine."""
    if a == b:
        return 0
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1,
                           prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[len(b)]


def wake_distance(text, wake=WAKE_DEFAULT, max_tokens=WAKE_MAX_TOKENS):
    """(edits, tokens_consumed, from_end) for the best wake match in `text`.

    Letters only, both sides, so the transcriber's punctuation and its
    decision to write "duckuck" as one word instead of two cannot matter.
    Returns (None, 0, False) when there is nothing to compare.
    """
    w = re.sub(r"[^a-z0-9]", "", normalise(wake))
    toks = normalise(text).split()
    if not w or not toks:
        return None, 0, False
    best = (None, 0, False)
    for from_end in (False, True):
        seq = list(reversed(toks)) if from_end else toks
        for k in range(1, min(max_tokens, len(seq)) + 1):
            part = seq[:k]
            if from_end:
                part = list(reversed(part))
            s = re.sub(r"[^a-z0-9]", "", "".join(part))
            e = _edits(s, w)
            if best[0] is None or e < best[0]:
                best = (e, k, from_end)
    return best


def strip_wake(text, wake=WAKE_DEFAULT, max_edits=WAKE_MAX_EDITS):
    """(heard_wake, remainder). The wake word may be anywhere in the phrase;
    people say it before AND after the command.

    An EXACT match wins first and consumes exactly itself, so every typed
    utterance behaves precisely as it did before this function learned to be
    tolerant. Only when there is no exact match does the fuzzy path run, and
    it consumes whole tokens from one end -- never from the middle, because a
    wake phrase matched out of the middle of a sentence would cut the
    sentence in half.
    """
    t = normalise(text)
    w = normalise(wake)
    if not w:
        return True, t
    if w in t:
        return True, re.sub(re.escape(w), " ", t).strip()
    e, k, from_end = wake_distance(t, wake)
    if e is not None and e <= max_edits:
        toks = t.split()
        rest = toks[:-k] if from_end else toks[k:]
        return True, " ".join(rest).strip()
    return False, t


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
    body = strip_filler(body)
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

    # A COMPOUND INSTRUCTION IS TWO INSTRUCTIONS AND IS SAID SO HERE.
    #
    # It is reported rather than executed: this function returns ONE intent,
    # and the clauses ride on it so a caller that knows how to run several --
    # `t1_instruction.plan_from` does -- can, while one that does not sees a
    # verb it can refuse. Silently returning the first clause is what used to
    # happen and it moved two cubes out of four.
    clauses = split_clauses(body)
    if len(clauses) > 1:
        first = parse(clauses[0], wake="", require_wake=False,
                      fix_spelling=False)
        first.clauses = list(clauses)
        first.raw = raw
        first.corrections = list(fixes)
        first.ambiguous_words = list(ties)
        return first

    # WHERE THE THING GOES, SPLIT OFF BEFORE ANYTHING READS THE TARGET.
    head, dest = split_destination(body)

    # "TIDY UP" -- an instruction with no verb this grammar knows and an
    # obvious meaning. Represented as its own verb so the grounding layer can
    # offer the reading back as a question; see TIDY_PHRASES.
    if dest is None:
        tidy = is_tidy(body)
        if tidy:
            return Intent_(verb="tidy", raw=raw, corrections=fixes,
                           quantity="all", target=None)

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
                              selector=extract_selector(head),
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
                          deictic=deictic, corrections=fixes,
                          quantity=extract_quantity(body),
                          selector=extract_selector(body))
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
