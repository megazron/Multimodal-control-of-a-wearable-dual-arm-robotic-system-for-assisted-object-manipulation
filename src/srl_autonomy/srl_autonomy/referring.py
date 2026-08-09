#!/usr/bin/env python3
"""Free-form referring expressions, grounded against the scene.

THE TENSION WITH THE RECORDED DECISION, STATED FIRST. This project chose a
deterministic grammar over a language model, and the reason still holds: a
grammar that does not recognise an utterance returns unparsed and the robot
asks again, whereas a model asked for structured output can return a
different, plausible verb that nothing downstream can distinguish from a
correct parse. Free-form input does not change that argument. What it changes
is WHERE the openness lives.

  * VERBS stay closed and deterministic. There are four, plus stop, and the
    consequence of mis-hearing one is a robot doing the wrong ACTION.
  * OBJECT REFERENCE becomes open. The consequence of mis-grounding is a robot
    doing the right action to the WRONG THING, which is bounded by
    reachability, confirmation and the keep-out, and is recoverable.

So this module opens the noun, not the verb. "Pick that random thing" parses
because the noun is unconstrained; it then fails to GROUND, which is a
different and much safer failure than parsing to something wrong.

THREE OUTCOMES, AND ONLY THE THIRD IS DANGEROUS.
  GROUNDED     one candidate clearly wins -> announce, confirm, act
  ASK          two or more are close, or nothing constrains the reference
  MISGROUNDED  one candidate wins and it is the wrong one

The whole design is arranged to convert the third into the second. A target is
committed only when the winning score exceeds the runner-up by `margin`, so a
close call becomes a question rather than a confident mistake.
"""
import re

GROUNDED, ASK, NONE, MISGROUNDED = "grounded", "ask", "none", "misgrounded"

COLOURS = {
    "red": (0.8, 0.1, 0.1), "orange": (0.9, 0.5, 0.1),
    "yellow": (0.9, 0.85, 0.1), "green": (0.15, 0.7, 0.2),
    "blue": (0.15, 0.3, 0.85), "purple": (0.5, 0.2, 0.7),
    "pink": (0.95, 0.6, 0.7), "brown": (0.45, 0.3, 0.15),
    "black": (0.05, 0.05, 0.05), "white": (0.95, 0.95, 0.95),
    "grey": (0.5, 0.5, 0.5), "gray": (0.5, 0.5, 0.5),
    "teal": (0.1, 0.6, 0.6), "tan": (0.8, 0.7, 0.5),
}

# Words that carry no constraint at all. "Thing", "one", "object", "random"
# and "whatever" are the ones that make a phrase FEEL specific while
# constraining nothing, which is exactly the case that must reach ASK.
VACUOUS = {"thing", "things", "object", "objects", "one", "item", "stuff",
           "something", "anything", "random", "whatever", "some", "any"}

SIZE_WORDS = {"big": 1, "large": 1, "biggest": 2, "largest": 2,
              "small": -1, "little": -1, "tiny": -1,
              "smallest": -2, "littlest": -2}

SPATIAL = {
    "left": ("x", -1), "leftmost": ("x", -2),
    "right": ("x", +1), "rightmost": ("x", +2),
    "near": ("y", -1), "nearest": ("y", -2), "closest": ("y", -2),
    "close": ("y", -1), "far": ("y", +1), "farthest": ("y", +2),
    "furthest": ("y", +2), "front": ("y", -1), "back": ("y", +1),
    "top": ("z", +1), "highest": ("z", +2), "upper": ("z", +1),
    "bottom": ("z", -1), "lowest": ("z", -2), "lower": ("z", -1),
}

DEICTIC = {"that", "this", "it", "there", "those", "these"}

# RELATIONAL REFERENCES ARE REFUSED, NOT GUESSED. "the one behind the red
# block" names an ANCHOR and asks for something else. Grounding it by the
# words present resolves to the anchor -- measured: it returned "red block"
# with full confidence, the single MISUNDERSTOOD case in the phrase set and
# the only genuinely dangerous outcome. Relational grounding needs pairwise
# geometry this module does not do, so the honest answer is to say so.
RELATIONAL = ("behind", "in front of", "next to", "beside", "left of",
              "right of", "on top of", "underneath", "under", "above",
              "below", "between", "nearest to", "closest to", "opposite")

# Verb and particle words must never become nouns. They leaked in and every
# candidate label then mismatched them, which turned "grab the leftmost
# object" into a refusal: the phrase was scored against the word "grab".
VERB_WORDS = {"pick", "grab", "take", "get", "fetch", "lift", "grasp",
              "put", "set", "place", "drop", "release", "give", "hand",
              "pass", "transfer", "move", "bring", "collect", "retrieve",
              "pickup", "pik", "grabb", "tak"}

STOPWORDS = {"the", "a", "an", "please", "can", "you", "could", "would",
             "me", "my", "for", "to", "up", "and", "then", "now", "just",
             "go", "ahead", "over", "of", "with", "your", "arm", "hand",
             "gripper", "robot", "there", "here", "on", "in", "at", "from"}


def normalise(text):
    return re.sub(r"[^a-z0-9 ]+", " ", (text or "").lower()).strip()


class Reference:
    """What an utterance said about WHICH object, with nothing invented."""

    def __init__(self, raw=""):
        self.raw = raw
        self.colour = None
        self.size = 0            # -2..+2 superlative scale
        self.spatial = []        # [(axis, weight)]
        self.nouns = []          # open vocabulary, may be empty
        self.deictic = False
        self.vacuous = False     # said something, constrained nothing

    @property
    def constrained(self):
        return bool(self.colour or self.size or self.spatial or self.nouns)

    def describe(self):
        bits = []
        if self.size:
            bits.append({2: "the biggest", 1: "a big", -1: "a small",
                         -2: "the smallest"}[self.size])
        if self.colour:
            bits.append(self.colour)
        bits.append(self.nouns[0] if self.nouns else "object")
        for ax, w in self.spatial:
            bits.append({("x", -1): "on the left", ("x", -2): "leftmost",
                         ("x", 1): "on the right", ("x", 2): "rightmost",
                         ("y", -1): "near", ("y", -2): "nearest",
                         ("y", 1): "far", ("y", 2): "furthest",
                         ("z", 1): "upper", ("z", 2): "highest",
                         ("z", -1): "lower", ("z", -2): "lowest"}
                        .get((ax, w), ""))
        return " ".join(b for b in bits if b)


def parse_reference(text):
    """Extract descriptors. The noun is OPEN: any leftover word is a
    candidate label, matched later against the detector's own vocabulary."""
    r = Reference(text or "")
    toks = normalise(text).split()
    for t in toks:
        if t in COLOURS and r.colour is None:
            r.colour = t
        elif t in SIZE_WORDS:
            r.size = SIZE_WORDS[t]
        elif t in SPATIAL:
            r.spatial.append(SPATIAL[t])
        elif t in DEICTIC:
            r.deictic = True
        elif t in VACUOUS:
            r.vacuous = True
        elif t not in STOPWORDS and t not in VERB_WORDS and len(t) > 2:
            r.nouns.append(t)
    return r


def _colour_distance(name, rgb):
    if rgb is None or name not in COLOURS:
        return None
    a = COLOURS[name]
    return sum((x - y) ** 2 for x, y in zip(a, rgb)) ** 0.5


def score(ref, cand, scene):
    """Score one candidate against the reference. Returns (score, why).

    Absent evidence scores ZERO rather than penalising. A detector that does
    not report colour must not make a colour-referenced phrase unresolvable;
    it should make it AMBIGUOUS, which is the safe outcome.
    """
    s, why = 0.0, []
    lab = (cand.get("label") or "").lower()

    if ref.nouns:
        hit = any(nn in lab or lab in nn for nn in ref.nouns)
        if hit:
            s += 2.0
            why.append("label matches '%s'" % ref.nouns[0])
        elif lab:
            s -= 0.5              # a named mismatch is weak evidence against

    if ref.colour:
        d = _colour_distance(ref.colour, cand.get("rgb"))
        if d is None:
            if ref.colour in lab:
                s += 1.5
                why.append("colour in the label")
        elif d < 0.35:
            s += 2.0
            why.append("colour matches")
        elif d > 0.75:
            s -= 1.0

    if ref.size and cand.get("volume") is not None:
        vols = [c.get("volume") for c in scene if c.get("volume") is not None]
        if len(vols) > 1:
            rank = sorted(vols).index(cand["volume"]) / (len(vols) - 1.0)
            want = 1.0 if ref.size > 0 else 0.0
            s += (1.5 if abs(ref.size) == 2 else 0.8) * (1 - abs(rank - want))
            why.append("size rank")

    for ax, w in ref.spatial:
        i = {"x": 0, "y": 1, "z": 2}[ax]
        vals = [c["xyz"][i] for c in scene]
        if len(vals) < 2:
            continue
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            continue
        rank = (cand["xyz"][i] - lo) / (hi - lo)
        want = 1.0 if w > 0 else 0.0
        s += (1.5 if abs(w) == 2 else 0.8) * (1 - abs(rank - want))
        why.append("position (%s)" % ax)

    return s, why


def ground(text, scene, margin=0.75, min_score=0.6):
    """Resolve a free-form phrase to one object, or refuse to.

    `scene` is a list of dicts: label, xyz, and optionally rgb, volume.
    Returns a dict with `outcome` in {GROUNDED, ASK, NONE}.
    """
    low = normalise(text)
    rel = next((w for w in RELATIONAL if w in low), None)
    if rel:
        return dict(outcome=NONE, ref=parse_reference(text),
                    reason="relational reference (%r) is not supported" % rel,
                    say=("I can only pick objects out by colour, size, name "
                         "or where they are, not by what they are %s "
                         "something else. Which one do you mean?" % rel))
    ref = parse_reference(text)
    if not scene:
        return dict(outcome=NONE, ref=ref, reason="nothing detected in the scene",
                    say="I can't see anything at the moment.")

    # A phrase that constrains NOTHING must never resolve, however confident
    # it sounds. "Pick that random thing" is the case this exists for.
    if not ref.constrained:
        if len(scene) == 1:
            c = scene[0]
            return dict(outcome=GROUNDED, ref=ref, target=c, score=1.0,
                        runner_up=None, why=["only one object present"],
                        say="the %s, the only thing I can see" % c["label"])
        return dict(outcome=ASK, ref=ref,
                    reason="the description does not narrow anything down",
                    candidates=[c["label"] for c in scene],
                    say="I can see %s. Which one?"
                        % _join([c["label"] for c in scene]))

    # A SUPERLATIVE IS DEFINITE. "leftmost", "nearest", "biggest" denote a
    # unique object by construction, so they select the extremum outright
    # rather than being scored gradedly against their neighbours. Scoring them
    # made the second-placed object land inside the ambiguity margin and
    # turned four unambiguous phrases into questions. This is semantics, not
    # threshold tuning: the tie test below still fires when two objects really
    # are level on the axis.
    sup = _superlative_pick(ref, scene)
    if sup is not None:
        pick, tie = sup
        if tie:
            return dict(outcome=ASK, ref=ref,
                        reason="two objects are level on that axis",
                        candidates=[c["label"] for c in tie],
                        say="Do you mean %s?"
                            % _join([c["label"] for c in tie], "or"))
        return dict(outcome=GROUNDED, ref=ref, target=pick, score=99.0,
                    runner_up=None, why=["superlative selects the extremum"],
                    say="the %s" % pick["label"])

    ranked = sorted(((score(ref, c, scene)[0], score(ref, c, scene)[1], c)
                     for c in scene), key=lambda t: -t[0])
    best, why, cand = ranked[0]
    second = ranked[1][0] if len(ranked) > 1 else None

    if best < min_score:
        return dict(outcome=NONE, ref=ref,
                    reason="nothing in the scene matches that description",
                    say="I can't see anything matching \"%s\"." % ref.describe())
    if second is not None and (best - second) < margin:
        tied = [c["label"] for sc, _, c in ranked if best - sc < margin]
        return dict(outcome=ASK, ref=ref, score=best, runner_up=second,
                    reason="two or more candidates score within %.2f" % margin,
                    candidates=tied,
                    say="Do you mean %s?" % _join(tied, "or"))
    return dict(outcome=GROUNDED, ref=ref, target=cand, score=best,
                runner_up=second, why=why,
                say="the %s" % cand["label"])


def _superlative_pick(ref, scene, eps=0.02):
    """Resolve a superlative directly. Returns (winner, tied_or_None) or None.

    Candidates are first filtered by any other stated constraint, so "the
    biggest blue one" ranks only the blue ones.
    """
    axis = next((a for a in ref.spatial if abs(a[1]) == 2), None)
    want_size = abs(ref.size) == 2
    if axis is None and not want_size:
        return None

    pool = scene
    if ref.colour or ref.nouns:
        narrowed = [c for c in scene if score(ref, c, scene)[0] > 0]
        if narrowed:
            pool = narrowed
    if len(pool) == 1:
        return pool[0], None

    if axis is not None:
        i = {"x": 0, "y": 1, "z": 2}[axis[0]]
        key = lambda c: c["xyz"][i]                      # noqa: E731
        tol = eps
    else:
        pool = [c for c in pool if c.get("volume") is not None]
        if len(pool) < 2:
            return (pool[0], None) if pool else None
        key = lambda c: c["volume"]                      # noqa: E731
        tol = 1e-9
    rev = (axis[1] > 0) if axis is not None else (ref.size > 0)
    ordered = sorted(pool, key=key, reverse=rev)
    best = ordered[0]
    tied = [c for c in ordered if abs(key(c) - key(best)) <= tol]
    return (best, tied if len(tied) > 1 else None)


def _join(items, word="and"):
    items = list(items)
    if len(items) == 1:
        return items[0]
    return "%s %s %s" % (", ".join(items[:-1]), word, items[-1])


# ---------------------------------------------------------------- planning
def plan_request(text, scene, reachable, keepout=None, margin=0.75):
    """Ground, then CHECK REACHABILITY BEFORE COMMITTING, then announce.

    `reachable(xyz)` -> (bool, reason). `keepout(xyz)` -> reason or None.

    ORDER IS LOAD-BEARING AND WAS GOT WRONG ONCE BEFORE. The keep-out filters
    candidates BEFORE grounding, so a forbidden object is never one of the
    options offered. Asking the operator to choose between options the robot
    would then refuse invites them to pick one and trust the answer.

    Reachability is checked AFTER grounding but BEFORE announcing, because
    announcing an intention the arm cannot carry out is the same mistake one
    stage later.
    """
    pool = scene
    excluded = []
    if keepout is not None:
        pool, excluded = [], []
        for c in scene:
            why = keepout(c["xyz"])
            (excluded if why else pool).append(c if not why else (c, why))
        pool = [c for c in pool]

    res = ground(text, pool, margin=margin)
    if res["outcome"] != GROUNDED:
        if not pool and excluded:
            res = dict(res)
            res["say"] = ("I can see %d object(s) but every one of them is "
                          "somewhere I will not reach: %s."
                          % (len(excluded),
                             _join(sorted({w for _, w in excluded}))))
        return res

    xyz = res["target"]["xyz"]
    ok, why = reachable(xyz)
    if not ok:
        return dict(outcome=NONE, ref=res["ref"], target=res["target"],
                    reason="target not reachable: %s" % why,
                    say=("I can see the %s, but I can't reach it: %s."
                         % (res["target"]["label"], why)))

    res = dict(res)
    res["announce"] = ("I will pick up the %s at %.2f, %.2f, %.2f. Say go "
                       "ahead, or stop." % (res["target"]["label"],
                                            xyz[0], xyz[1], xyz[2]))
    res["needs_confirmation"] = True
    return res
