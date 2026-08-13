#!/usr/bin/env python3
"""DANCE for the demonstration mode. Both arms, timed to a beat.

    python3 src/srl_experiments/experiments/abc/choreography.py

NOT A TASK, AND THE DISTINCTION IS LOAD-BEARING. These produce no trial data,
no condition, no metric and no comparison. They exist to show the system
moving well, which is a legitimate thing to want and a dangerous thing to
confuse with evidence -- a smooth clip is not a result, and the caption on
every routine says so.

SAFETY IS NOT RELAXED FOR A DEMONSTRATION. Every waypoint of every routine is
collision-checked against the WEARER through the same `/compute_ik` with
`avoid_collisions=True` that every task uses, at the same N, and the routines
run through `ik_follower_node` with the same velocity caps, the same clearance
floor, the same step guard and the same e-stop and dead-man subscriptions. A
demo that bypassed the safety stack would be a demonstration of something this
project does not have.

=====================================================================
WHY THE FIRST VERSION WAS NOT DANCING, AND WHAT CHANGED
=====================================================================

The first version was three lists of waypoints -- alternating, mirrored, and a
circle -- emitted at a constant rate. That is a MOTION TEST. The three
differed only in which points they visited, and a viewer would have called all
three "the robot moving smoothly". Five things were missing, and each is now a
named mechanism rather than a nicer set of coordinates:

  BEAT. The runner publishes one waypoint every 0.05 s -- exactly 20 Hz -- so
  TIME IS SPACING. A move given 2 beats at 100 BPM occupies 24 waypoints; the
  same move given half a beat occupies 6 and reads as a snap. None of the
  geometry changes; the phrasing lives entirely in how many samples a move is
  allowed. `_render()` is where that conversion happens.

  EASING. Constant velocity is the single strongest "machine" cue -- real
  movement accelerates out of stillness and decelerates into it. Each keyframe
  carries its own curve: `swing` for flowing, `snap` for accents, `overshoot`
  and `bounce` for anything that should feel alive.

  ANTICIPATION. A movement that starts slightly AGAINST its direction before
  travelling reads as intentional; one that simply starts reads as a setpoint
  change. `anticipate=` backs off along the reverse of the coming move for a
  fraction of it first.

  FOLLOW-THROUGH. Arriving and stopping dead is the other machine cue.
  `overshoot` passes the target and settles back into it.

  RELATIONSHIP. Two arms can be in UNISON, CANON (the same phrase delayed),
  OPPOSITION (contrary motion) or CALL-AND-RESPONSE (one moves, the other
  answers). The old set only ever did symmetry. Each routine now commits to a
  different relationship, and that -- more than the coordinates -- is what
  makes them read as three different pieces.

THE ENVELOPE IS BIGGER THAN THE OLD ROUTINES USED. Measured with no furniture
(scripts/measure_forward_reach.py): forward reach 0.475 m at z=1.20 and
0.500 m at z=1.30, against the old routines' fixed y=0.30. These use
y 0.16..0.40 and z 1.16..1.50, every point verified at N=10.

HOW TO TELL IF IT IS STILL DANCING. `main()` prints max and min sample speed.
A constant-velocity sweep has them EQUAL. If a future edit makes them
converge, the phrasing has been lost whatever the coordinates say.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

FPS = 20.0          # the runner publishes one waypoint per 0.05 s

# ANTICIPATION HAS A FLOOR IN SECONDS, NOT A FRACTION OF THE MOVE.
#
# The wind-up used to be 20% of whatever the move's waypoint count happened
# to be. At the beats these three routines currently use that lands at
# 0.45-0.60 s and looks right, so nothing had ever gone wrong -- but nothing
# held it there either. Shorten a key from 3 beats to 1 and the same code
# renders a 0.10 s wind-up, which the capture runs at 5-13 fps: ONE FRAME.
# An anticipation nobody can see is not an anticipation, and the failure is
# silent because the geometry is unchanged and every check still passes.
#
# So the floor is stated in TIME, at the rate the clip is filmed at, and a
# move too short to carry it gets NO wind-up rather than an invisible one.
ANTICIPATE_S = 0.30
ANTICIPATE_N = int(round(ANTICIPATE_S * FPS))     # 6 waypoints
# The travelling part of the move needs to survive too. Four waypoints is
# 0.20 s, the shortest thing that reads as a move rather than a jump.
ANTICIPATE_MIN_MOVE_N = 4
# Every wind-up actually rendered, in seconds, and every key that ASKED for
# one and was too short to carry it. Read by the audit and by main(), so
# "the anticipation is 0.30 s" is a measurement of the rendered path rather
# than a claim about the constant.
_ANTI_LOG = []
_ANTI_SKIPPED = []


def anticipation_report():
    """(durations_s, skipped) for the routines rendered so far."""
    return list(_ANTI_LOG), list(_ANTI_SKIPPED)

# THE SPEED BUDGET, and it is a real constraint rather than a style choice.
# `ik_follower_node` caps max_vel_rad_s at 0.6 and slews anything past
# max_step_rad, so a commanded pose that moves faster than the arm can
# execute does NOT look fast -- it looks LATE. The first version of these
# routines peaked at 5.05 m/s, roughly fifteen times what the follower will
# deliver, and every "snap" would have arrived as a lagging drift. Choreograph
# inside the budget instead: a shorter move on the beat still reads as sharp,
# whereas a long move that gets clamped reads as broken.
V_MAX = 0.30

# THE DANCE ENVELOPE, inside the measured reachable volume with margin.
# Forward reach is 0.475 m at z=1.20 with no furniture; y stops at 0.40 here,
# 75 mm inside that, per CLAUDE.md's rule of staying at least 20 mm inside the
# last pose that passed N/N.
# MEASURED THE HARD WAY. The first version of this envelope was taken from
# the forward-reach sweep, which probed only |x| = 0.35 -- so it said nothing
# about the inboard and outboard columns the dance actually visits, and 117
# of 1455 waypoints failed IK. The corners that failed were the LOW INNER one
# (flow's own opening pose, 0.26/0.16/1.16, was unreachable) and everything
# play's overshoot threw past the box: |x| out to 0.679, z to 1.568.
#
# So the box is now conservative AND enforced. Keyframes live inside it, and
# _clamp() guarantees the rendered path -- overshoot, bounce and wind-up
# included -- cannot leave it, because those curves deliberately travel PAST
# their targets and an envelope that only bounds the keyframes bounds nothing.
X_IN, X_MID, X_OUT = 0.32, 0.41, 0.50
Y_NEAR, Y_MID, Y_FAR = 0.20, 0.27, 0.34
Z_LOW, Z_MID, Z_HIGH, Z_TOP = 1.24, 1.30, 1.36, 1.42
BOX = ((0.30, 0.52), (0.18, 0.36), (1.22, 1.44))     # |x|, y, z


# ------------------------------------------------------------------ easing
def _lin(t):
    return t


def _smooth(t):
    """Ease in and out. The default for anything flowing."""
    return t * t * (3.0 - 2.0 * t)


def _snap(t):
    """Fast out of the gate, decelerating hard. Accents and hits.

    THE EXPONENT IS THE BUDGET. (1-t)**4 starts at four times the average
    speed, so a 0.15 m accent over 0.6 s peaks at 1.0 m/s -- three times what
    the follower will execute, and it would have arrived as lag. **2 starts
    at twice average, which is as sharp as this arm can actually be.
    """
    return 1.0 - (1.0 - t) ** 2


def _swing(t):
    """Slow start, quick middle, slow end -- more extreme than smooth."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


def _overshoot(t, k=1.15):
    """Pass the target and settle back. FOLLOW-THROUGH."""
    u = t - 1.0
    return 1.0 + u * u * ((k + 1.0) * u + k)


def _bounce(t):
    """Overshoot, settle, overshoot smaller. Playful.

    The frequency is deliberately low (1.5 cycles, not 2.5): every extra
    wobble multiplies PEAK speed, and peak speed is the budget that binds.
    """
    return 1.0 - math.cos(1.5 * math.pi * t) * (1.0 - t) ** 1.6


EASE = {"lin": _lin, "smooth": _smooth, "snap": _snap, "swing": _swing,
        "overshoot": _overshoot, "bounce": _bounce}


# ------------------------------------------------------------- rendering
def _lerp(a, b, u):
    return [a[i] + (b[i] - a[i]) * u for i in range(3)]


def _beats_to_n(beats, bpm):
    return max(1, int(round(beats * (60.0 / bpm) * FPS)))


def _render(keys, bpm):
    """Keyframes -> one point per 0.05 s.

    A key is (target, beats, ease, anticipate). `beats` is TIME, and that is
    the entire mechanism by which any of this has rhythm.
    """
    out = [list(keys[0][0]) if isinstance(keys[0], tuple) else list(keys[0])]
    cur = list(out[0])
    for k in keys[1:]:
        tgt, beats, ez, anti = k
        n = _beats_to_n(beats, bpm)
        f = EASE.get(ez, _smooth)
        if anti and n >= ANTICIPATE_N + ANTICIPATE_MIN_MOVE_N:
            # WIND UP: a short move the OTHER way first. Without it a gesture
            # reads as a setpoint change rather than an intention.
            #
            # AT LEAST ANTICIPATE_N WAYPOINTS, whatever the move's length --
            # see the constant. 20% of a long move is still used when it is
            # more than the floor, because a big gesture wants a bigger
            # wind-up; the floor only stops a short one vanishing.
            na = max(ANTICIPATE_N, int(round(n * 0.20)))
            na = min(na, n - ANTICIPATE_MIN_MOVE_N)
            _ANTI_LOG.append(na / FPS)
            back = [cur[i] - (tgt[i] - cur[i]) * anti for i in range(3)]
            for j in range(1, na + 1):
                out.append(_lerp(cur, back, _smooth(j / float(na))))
            cur = list(back)
            n = max(2, n - na)
        elif anti:
            # ASKED FOR AND NOT GIVEN, and said so. Rendering a 0.10 s wind-up
            # here would satisfy the keyframe and show nothing on screen.
            _ANTI_SKIPPED.append((tuple(round(v, 3) for v in tgt), beats, n))
        for j in range(1, n + 1):
            out.append(_lerp(cur, tgt, f(j / float(n))))
        cur = list(tgt)
    return out


def _hold(p, beats, bpm):
    """Stillness. A phrase with no rests has no phrasing."""
    return [list(p)] * _beats_to_n(beats, bpm)


def _mirror(pts):
    return [[-p[0], p[1], p[2]] for p in pts]


def _canon(pts, beats, bpm):
    """The same phrase, DELAYED -- a round. The lag is the whole effect."""
    return [list(pts[0])] * _beats_to_n(beats, bpm) + [list(p) for p in pts]


def _clamp(pts):
    """Keep the RENDERED path inside the verified box.

    Overshoot, bounce and anticipation all travel past their keyframes -- that
    is the point of them -- so bounding the keyframes bounds nothing. This
    bounds the samples that are actually published.
    """
    (xl, xh), (yl, yh), (zl, zh) = BOX
    out = []
    for p in pts:
        sx = 1.0 if p[0] >= 0 else -1.0
        out.append([sx * min(xh, max(xl, abs(p[0]))),
                    min(yh, max(yl, p[1])),
                    min(zh, max(zl, p[2]))])
    return out


def _limit(pts, v_max=None):
    """Subdivide any step that would exceed the follower's speed.

    WHY NOT JUST HAND-TUNE THE BEATS. Peak speed depends on the easing curve's
    steepest derivative, not on the average, so `bounce` and `overshoot` blow
    the budget at durations where `smooth` is comfortable -- and hand-tuning
    each keyframe until it passes silently flattens exactly the curves that
    make it read as alive. This instead SUBDIVIDES the offending steps, which
    slows the fast part locally and leaves the geometry and the ordering
    untouched: the sharpest move is still the sharpest, it is simply executed
    at a speed the arm can deliver.

    The cost is honest and reported: the routine gets longer. It does NOT
    clamp, drop or smooth anything, because a demo whose peak was quietly
    removed would be a different piece of choreography than the one written.
    """
    v_max = V_MAX if v_max is None else v_max
    step_max = v_max / FPS
    out = [list(pts[0])]
    for a, b in zip(pts, pts[1:]):
        d = math.dist(a, b)
        k = max(1, int(math.ceil(d / step_max - 1e-9)))
        for j in range(1, k + 1):
            out.append(_lerp(a, b, j / float(k)))
    return out


def _pad(a, b):
    """Equal length, holding whichever finishes first at its last pose."""
    n = max(len(a), len(b))
    return (a + [list(a[-1])] * (n - len(a)),
            b + [list(b[-1])] * (n - len(b)))


# ------------------------------------------------------------- routines
def flow():
    """ADAGIO -- slow, continuous, the two arms in CANON.

    Nothing stops. Every key is `swing` or `smooth`, the phrase climbs to a
    held apex and unwinds, and the right arm runs the SAME phrase one beat
    behind. That lag is the effect: unison reads as one gesture, canon reads
    as two dancers.
    """
    bpm = 60.0
    L = _render([
        ([X_IN, Y_NEAR, Z_LOW], 0, "lin", 0),
        ([X_MID, Y_MID, Z_MID], 2.5, "swing", 0.14),     # rise, with wind-up
        ([X_OUT, Y_FAR, Z_HIGH], 2.5, "swing", 0),       # open out
        ([X_OUT, Y_MID, Z_TOP], 2.0, "smooth", 0),       # apex
    ], bpm)
    L += _hold(L[-1], 1.0, bpm)                          # HOLD the apex
    L += _render([
        (L[-1], 0, "lin", 0),
        ([X_MID, Y_FAR, Z_HIGH], 2.5, "swing", 0),       # unwind
        ([X_IN, Y_MID, Z_MID], 2.5, "swing", 0),
        ([X_IN, Y_NEAR, Z_LOW], 3.0, "smooth", 0),
    ], bpm)[1:]
    L += _hold(L[-1], 1.0, bpm)
    R = _mirror(_canon(L, 1.0, bpm))
    return dict(zip(("left", "right"),
                    _pad(_limit(_clamp(L)), _limit(_clamp(R)))))


def pulse():
    """STACCATO -- sharp, on the beat, the arms in OPPOSITION.

    Every move is a half-beat `snap` followed by a HOLD. The stillness between
    hits is what makes it rhythmic; the same path at constant speed would read
    as a scan. The arms move in CONTRARY motion -- one rising as the other
    drops -- rather than mirrored, so they oppose instead of match, and only
    resolve into unison on the final accent.
    """
    bpm = 80.0
    # A HIT IS SHORT, NOT HUGE. One beat at 100 BPM is 0.6 s, and 0.30 m/s
    # buys 0.18 m within it -- so the excursions here are ~0.12 m, which
    # lands well inside the budget and still reads as struck rather than
    # swept. The hold after each is doing as much work as the move.
    zh, zl = Z_MID + 0.05, Z_MID - 0.05
    L = [[X_MID, Y_MID, zl]]
    R = [[-X_MID, Y_MID, zh]]
    for i in range(8):
        up = (i % 2 == 0)
        y = Y_MID + (0.05 if i % 2 else -0.05)
        x = X_MID + (0.07 if i % 4 == 3 else (-0.07 if i % 4 == 1 else 0.0))
        L += _render([(L[-1], 0, "lin", 0),
                      ([x, y, zh if up else zl], 1.0, "snap", 0)], bpm)[1:]
        L += _hold(L[-1], 1.0, bpm)
        R += _render([(R[-1], 0, "lin", 0),
                      ([-x, y, zl if up else zh], 1.0, "snap", 0)], bpm)[1:]
        R += _hold(R[-1], 1.0, bpm)
    for arr, sx in ((L, 1.0), (R, -1.0)):
        arr += _render([(arr[-1], 0, "lin", 0),
                        ([sx * X_MID, Y_MID, Z_HIGH], 3.0, "overshoot", 0.3)],
                       bpm)[1:]
        arr += _hold(arr[-1], 2.0, bpm)
    return dict(zip(("left", "right"), _pad(_limit(_clamp(L)), _limit(_clamp(R)))))


def play():
    """PLAYFUL -- CALL AND RESPONSE, with wind-up and follow-through.

    The left arm makes a gesture and HOLDS; the right answers it, bigger, with
    a bounce; then they trade. Every gesture winds up against itself first and
    overshoots on arrival, which is what separates a character from a
    trajectory. The asymmetry is deliberate -- an answer that exactly matched
    the call would be the mirror routine again. The waiting arm HOLDS rather
    than drifting, because in a call-and-response the stillness is part of the
    performance.
    """
    bpm = 72.0
    L = [[X_IN, Y_NEAR, Z_MID]]
    R = [[-X_IN, Y_NEAR, Z_MID]]
    # TWO BEATS PER GESTURE. One beat at 104 BPM is 0.577 s, which at the
    # 0.30 m/s budget covers only 0.17 m -- less than these gestures travel.
    # Two beats buys 0.35 m and keeps the overshoot executable, so the
    # follow-through is seen rather than clamped away.
    calls = [(([X_MID, Y_FAR, Z_HIGH], 3.0, "overshoot", 0.24),
              ([-X_OUT, Y_FAR, Z_TOP], 3.0, "bounce", 0.30)),
             (([X_OUT, Y_MID, Z_LOW], 3.0, "overshoot", 0.24),
              ([-X_MID, Y_NEAR, Z_HIGH], 3.0, "bounce", 0.30)),
             (([X_IN, Y_FAR, Z_TOP], 3.5, "overshoot", 0.24),
              ([-X_OUT, Y_MID, Z_LOW], 3.5, "bounce", 0.30))]
    for call, resp in calls:
        seg = _render([(L[-1], 0, "lin", 0), call], bpm)[1:]
        L += seg + _hold(call[0], 1.0, bpm)
        R += _hold(R[-1], 0, bpm) * 0 + [list(R[-1])] * (
            len(seg) + _beats_to_n(1.0, bpm))
        seg = _render([(R[-1], 0, "lin", 0), resp], bpm)[1:]
        R += seg + _hold(resp[0], 0.5, bpm)
        L += [list(L[-1])] * (len(seg) + _beats_to_n(0.5, bpm))
    for arr, sx in ((L, 1.0), (R, -1.0)):
        arr += _render([(arr[-1], 0, "lin", 0),
                        ([sx * X_MID, Y_MID, Z_HIGH], 3.0, "overshoot", 0.2)],
                       bpm)[1:]
        arr += _hold(arr[-1], 1.5, bpm)
    return dict(zip(("left", "right"), _pad(_limit(_clamp(L)), _limit(_clamp(R)))))


# Old names kept so anything still referring to them resolves.
wave, mirror, sweep = flow, pulse, play

ROUTINES = {
    "flow": dict(build=flow, bpm=60,
                 character="adagio, CANON -- nothing stops; the right arm "
                           "runs the same phrase one beat behind"),
    "pulse": dict(build=pulse, bpm=80,
                  character="staccato, OPPOSITION -- half-beat snaps onto "
                            "the beat, holds between, contrary motion"),
    "play": dict(build=play, bpm=72,
                 character="CALL AND RESPONSE -- wind-up, overshoot, bounce; "
                           "the answer is bigger than the call"),
}

_DEMO_CAVEAT = ("A DEMONSTRATION IS NOT EVIDENCE. This routine produces no "
                "trial data, no condition, no metric and no comparison, and "
                "nothing in it may be cited as a result. It runs the same "
                "collision-aware IK, clearance floor, velocity caps, step "
                "guard, e-stop and dead-man as every task -- a demo does not "
                "get its own, weaker, safety stack.")

TASKS = {
    "d1": dict(name="dance: flow", scenario="D1_flow", build=flow,
               grip=lambda n: {"left": [0.0] * n, "right": [0.0] * n},
               width_mm=0, grip_obj=None, place_target=None,
               expect="ADAGIO at 60 BPM. Continuous eased motion rising to a "
                      "HELD apex and unwinding. The arms are in CANON -- the "
                      "right runs the same phrase ONE BEAT behind, so they "
                      "are never quite together.",
               caveat=_DEMO_CAVEAT),
    "d2": dict(name="dance: pulse", scenario="D2_pulse", build=pulse,
               grip=lambda n: {"left": [0.0] * n, "right": [0.0] * n},
               width_mm=0, grip_obj=None, place_target=None,
               expect="STACCATO at 80 BPM. Half-beat snaps onto the beat "
                      "with a hold between each -- the stillness is what "
                      "makes it rhythmic. The arms are in OPPOSITION, one "
                      "rising as the other drops, resolving to unison on a "
                      "final overshooting accent.",
               caveat=_DEMO_CAVEAT),
    "d3": dict(name="dance: play", scenario="D3_play", build=play,
               grip=lambda n: {"left": [0.0] * n, "right": [0.0] * n},
               width_mm=0, grip_obj=None, place_target=None,
               expect="CALL AND RESPONSE at 72 BPM. Left gestures and HOLDS, "
                      "right answers bigger with a bounce, then they trade. "
                      "Every gesture winds up against itself and overshoots "
                      "on arrival.",
               caveat=_DEMO_CAVEAT),
}
ORDER = ("d1", "d2", "d3")


def path_length(p):
    return sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1))


def speed_profile(p):
    """Per-sample speed, m/s. The evidence that this is not constant."""
    return [math.dist(p[i], p[i + 1]) * FPS for i in range(len(p) - 1)]


def still_fraction(p, eps=1e-4):
    """Fraction of samples that are HOLDS. A sweep has none."""
    v = speed_profile(p)
    return sum(1 for s in v if s < eps) / float(max(1, len(v)))


def main():
    print("DANCE ROUTINES -- waypoints at %g Hz" % FPS)
    print("%-6s %4s %5s %6s %8s %8s %8s %7s  %s"
          % ("name", "bpm", "pts", "secs", "path m", "v peak", "v mean",
             "still", "character"))
    for name in ("flow", "pulse", "play"):
        r = ROUTINES[name]
        p = r["build"]()
        assert len(p["left"]) == len(p["right"]), name
        v = speed_profile(p["left"])
        moving = [s for s in v if s > 1e-4]
        print("%-6s %4d %5d %6.1f %8.3f %8.3f %8.3f %6.0f%%  %s"
              % (name, r["bpm"], len(p["left"]), len(p["left"]) / FPS,
                 path_length(p["left"]), max(v),
                 sum(moving) / max(1, len(moving)),
                 100 * still_fraction(p["left"]), r["character"]))
    print("\nv peak / v mean is the PHRASING and 'still' is the punctuation.")
    print("A constant-velocity sweep has peak == mean and still == 0%.")
    bad = []
    for name in ("flow", "pulse", "play"):
        p = ROUTINES[name]["build"]()
        for arm in ("left", "right"):
            v = max(speed_profile(p[arm]))
            if v > V_MAX:
                bad.append("%s/%s %.3f m/s" % (name, arm, v))
    if bad:
        print("\nOVER THE %.2f m/s BUDGET -- these would be SLEWED by the "
              "follower and\nread as lag, not speed: %s"
              % (V_MAX, ", ".join(bad)))
        return 1
    print("all routines within the %.2f m/s follower budget" % V_MAX)
    return 0


if __name__ == "__main__":
    sys.exit(main())
