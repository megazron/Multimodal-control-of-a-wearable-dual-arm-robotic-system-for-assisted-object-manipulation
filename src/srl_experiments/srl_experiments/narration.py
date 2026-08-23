#!/usr/bin/env python3
"""WHAT THE ROBOT IS DOING, IN WORDS, FROM ONE SOURCE.

    python3 -m srl_experiments.narration          # the self-test
    python3 -m srl_experiments.narration --all    # every phase, as spoken

PURE. No ROS, no audio, no GUI. It turns a phase and some facts into a
sentence; who displays it, who logs it and who speaks it are separate
problems, which is what lets all three say exactly the same thing.

WHY THIS EXISTS
---------------
Watching a recording, there is no way to tell what the robot believes it is
doing. It moves, and you infer. The only narration in the system was two
`[progress]` lines -- printed at a grasp and at a release, scraped off stdout
by ONE tab of the GUI -- so for the whole of a run except two instants,
nothing said anything at all. "Reaching", "calibrating", "returning home" and
"waiting for the camera" were all the same observation: the arm is moving, or
it is not.

An operator cannot supervise what they cannot read, and a reviewer cannot
judge a clip in which the robot never says what it was trying to do.

ONE VOCABULARY, THREE CONSUMERS
-------------------------------
The GUI banner, the run log and the text-to-speech all call this. If they had
their own wording they would drift, and a video whose caption disagrees with
the window beside it is worse than a video with no caption -- this repository
has already paid for exactly that with a clip captioned "multimeter" that
showed an empty gripper.

WRITTEN TO BE HEARD, NOT READ
-----------------------------
The sentences are short, start with the verb, and carry ONE number at most.
`speech()` strips what a listener cannot use -- coordinates to five decimal
places, topic names, units nobody says out loud -- while `text()` keeps them,
because a person watching the screen can use a coordinate and a person
listening cannot.

THERE IS NO AUDIO DEVICE ON THIS HOST. `/dev/snd` contains only `timer`, so
nothing here can be played out loud on this machine. Piper can SYNTHESISE it
to a file, which is how the narration reaches the recorded clips. That is a
real limit and it is stated rather than discovered later.
"""
from __future__ import annotations

import re

# The phases, in the order a run passes through them. The ORDER matters: it is
# what `is_progress` uses to tell moving-on from going-backwards, so a caller
# can highlight a repeat or a regression rather than showing every line the
# same way.
PHASES = (
    "STARTING",         # process up, nothing commanded yet
    "HOMING",           # going to the known start pose
    "CALIBRATING",      # looking at the world to build the map
    "MAPPING",          # fusing what was seen into one map
    "PLANNING",         # choosing a path through the map
    "REACHING",         # travelling to a target
    "TOUCHING",         # probing a surface at a cell
    "GRASPING",         # closing on an object
    "CARRYING",         # holding and moving
    "PLACING",          # arriving over the destination
    "RELEASING",        # opening
    "RETURNING",        # going back to the start pose
    "DONE",             # finished normally
    "WAITING",          # blocked on something outside the robot
    "REFUSED",          # stopped on purpose, with a reason
)
_ORDER = {p: i for i, p in enumerate(PHASES)}

# Present participle for each phase, so a sentence reads as something the
# robot is DOING rather than a state name shouted at the operator.
_VERB = {
    "STARTING": "starting up",
    "HOMING": "going to the home pose",
    "CALIBRATING": "calibrating: looking at the workspace",
    "MAPPING": "building the map",
    "PLANNING": "planning a path",
    "REACHING": "reaching",
    "TOUCHING": "touching the surface",
    "GRASPING": "closing on the object",
    "CARRYING": "carrying",
    "PLACING": "placing",
    "RELEASING": "letting go",
    "RETURNING": "returning home",
    "DONE": "finished",
    "WAITING": "waiting",
    "REFUSED": "stopping",
}


class NarrationError(Exception):
    """An unknown phase is a programming error, not a thing to paper over."""


def _fmt_xyz(p):
    return "(%.3f, %.3f, %.3f)" % (float(p[0]), float(p[1]), float(p[2]))


def _clauses(phase, arm=None, at=None, target=None, cell=None, of=None,
             detail=None, why=None):
    """[(words, speakable)] -- ONE construction, two renderings.

    THE SPOKEN FORM IS NOT THE WRITTEN FORM WITH BITS DELETED, and the first
    version made exactly that mistake: it regex-stripped the coordinate out of
    the finished sentence and left the preposition behind, so the robot said
    "Left arm: reaching at." A clause knows whether it can be spoken; dropping
    it takes its preposition with it.
    """
    phase = str(phase).upper()
    if phase not in _ORDER:
        raise NarrationError(
            "unknown phase %r; expected one of %s" % (phase, ", ".join(PHASES)))
    verb = _VERB[phase]
    out = [("%s arm: %s" % (arm, verb) if arm else verb, True)]
    if cell is not None and of:
        out.append(("cell %d of %d" % (int(cell), int(of)), True))
    elif cell is not None:
        out.append(("cell %d" % int(cell), True))
    if target:
        out.append(("towards %s" % target, True))
    # NOT SPEAKABLE: a coordinate to three decimals is unusable read aloud,
    # and the screen still carries it.
    if at is not None:
        out.append(("at %s" % _fmt_xyz(at), False))
    if detail:
        out.append((str(detail), True))
    if why:
        out.append(("-- %s" % why, True))
    return out


def text(phase, **kw):
    """The full sentence, for the screen and the log.

    Every argument is optional and an omitted one simply does not appear -- a
    narrator that prints "reaching None" is worse than one that prints
    "reaching".
    """
    return " ".join(w for w, _ in _clauses(phase, **kw))


# Things a listener cannot use even inside a speakable clause: topic paths and
# bracketed tags, which arrive inside a `why` written for a log.
_STRIP = (
    (re.compile(r"/[A-Za-z_][A-Za-z0-9_/]*"), ""),
    (re.compile(r"\[[^\]]*\]"), ""),
    (re.compile(r"\s+"), " "),
)


def speech(phase, **kw):
    """The same facts, said out loud.

    Built from the speakable clauses, so no clause is ever left holding a
    preposition whose object has gone.
    """
    s = " ".join(w for w, ok in _clauses(phase, **kw) if ok)
    for pat, rep in _STRIP:
        s = pat.sub(rep, s)
    s = s.replace(" --", ",").strip(" ,")
    return (s[0].upper() + s[1:] + ".") if s else ""


def is_progress(prev, phase):
    """Is this a step FORWARD through the run, or a repeat or a regression?

    Lets a caller show a repeat differently rather than printing an identical
    line forty times, which is how a narration stops being read.
    """
    if prev is None:
        return True
    a, b = _ORDER.get(str(prev).upper()), _ORDER.get(str(phase).upper())
    if a is None or b is None:
        return True
    return b > a


def line(phase, **kw):
    """The `[progress]` line the run log carries and the GUI already scrapes.

    Kept in the SAME shape the GUI's Instruct tab has parsed since before this
    module existed, so adding narration cannot break the one consumer that was
    already working.
    """
    return "[progress] %s" % text(phase, **kw)


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):                                   # noqa: C901
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    s = text("REACHING", arm="left", at=[0.42, 0.45, 1.27])
    check("a sentence names the arm, the verb and the place",
          s == "left arm: reaching at (0.420, 0.450, 1.270)", s)

    check("an omitted field does not appear",
          text("HOMING") == "going to the home pose", text("HOMING"))
    check("nothing ever reads 'None'",
          all("None" not in text(p) for p in PHASES))

    c = text("TOUCHING", arm="left", cell=7, of=20, at=[0.3, 0.3, 1.245])
    check("a scan says which cell of how many",
          "cell 7 of 20" in c, c)

    # SPEECH DROPS WHAT CANNOT BE HEARD, and keeps what can.
    sp = speech("REACHING", arm="left", at=[0.42, 0.45, 1.27])
    check("speech drops the coordinate", "0.420" not in sp, sp)
    check("speech keeps the arm and the verb",
          "left arm" in sp.lower() and "reaching" in sp.lower(), sp)
    check("speech never ends on an orphaned preposition",
          not any(sp.lower().rstrip(".").endswith(w)
                  for w in (" at", " towards", " of", " on")), sp)
    check("speech is a sentence", sp.endswith(".") and sp[0].isupper(), sp)
    sp2 = speech("TOUCHING", arm="left", cell=7, of=20, at=[0.3, 0.3, 1.2])
    check("speech keeps the count", "cell 7 of 20" in sp2, sp2)
    sp3 = speech("REFUSED", why="no plane on /camera/depth")
    check("speech drops a topic name", "/camera" not in sp3, sp3)

    # A CHECK THAT CANNOT FAIL IS NOT A CHECK: speech must differ from text
    # somewhere, or the stripping is doing nothing.
    check("speech really differs from the screen text",
          speech("REACHING", at=[0.1, 0.2, 0.3])
          != text("REACHING", at=[0.1, 0.2, 0.3]))

    # THE PHASE VOCABULARY IS CLOSED.
    try:
        text("POTTERING")
        check("an unknown phase is refused", False)
    except NarrationError as e:
        check("an unknown phase is refused by name", "POTTERING" in str(e))
    check("every phase has a verb", set(_VERB) == set(PHASES))

    check("progress is ordered",
          is_progress("HOMING", "REACHING")
          and not is_progress("REACHING", "HOMING")
          and not is_progress("REACHING", "REACHING"))
    check("the first line is always progress", is_progress(None, "STARTING"))

    check("the log line keeps the shape the GUI already parses",
          line("GRASPING", arm="left").startswith("[progress] "),
          line("GRASPING", arm="left"))

    if verbose:
        print("narration self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    if "--all" in sys.argv:
        for p in PHASES:
            print("%-12s %-52s | %s"
                  % (p, text(p, arm="left", cell=3, of=20),
                     speech(p, arm="left", cell=3, of=20)))
        sys.exit(0)
    sys.exit(0 if self_test() else 1)
