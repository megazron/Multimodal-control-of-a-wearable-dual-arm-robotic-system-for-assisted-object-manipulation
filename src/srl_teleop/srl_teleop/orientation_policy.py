#!/usr/bin/env python3
"""How much the wrist is allowed to differ from the commanded orientation.

ONE SOURCE. `ik_follower_node` asks this module what to try, and
`scripts/measure_orientation_cost.py` measures what this module permits. If
they were two implementations the measurement would describe a policy the
robot does not run, which is this repository's most repeated defect -- the
wearer guard that resolved 0 of 9 parts, the scene that measured stage 2 at
stage 1's approach, the reachability sweep that solved at the HOME wrist while
every task commands the ANCHOR.

WHY IT EXISTS, WITH THE NUMBER
------------------------------
Every waypoint of every mode is sent with `master_calibration.WORKSPACE_ORIENT`
written into it by `run_abc.send()`: one quaternion per arm, the same one
everywhere in the workspace. On a 7-DOF arm that is a 6-DOF constraint, and it
spends the whole redundancy -- the single extra joint whose entire purpose is
to let the elbow move out of the way while the hand stays put.

Measured 2026-08-22, offline, wearer floor 0.15 m enforced throughout, in
`recordings/baselines/orientation_cost*.json`:

    from the arm's own HOME EE (0.744, 0.339, 1.180) -- far outboard, roomy
        pinned  0.460 m mean reach over 28 walks,  6 wearer-bound
        free    0.547 m                            6 wearer-bound
        the pin costs 87 mm

    from the WORK POINT (0.400, 0.175, 1.120) -- where the tasks actually run
        pinned  0.065 m mean reach over 12 walks, 11 of 12 wearer-bound
        cone15  0.319 m                            7
        cone45  0.440 m                            5
        free    0.450 m                            4
        the pin costs 385 mm, and a factor of SEVEN

That second table is the finding. At the point the work happens the pinned
wrist does not merely shorten the reach, it makes eleven of twelve directions
unreachable, and the mechanism is not "the wearer is in the way": it is that
the pin removes the arm's freedom to pick an elbow that is NOT in the wearer.
The floor was 0.15 m under every policy and was never relaxed -- widening the
workspace by moving the floor would be a bug, not a result.

FREEING THE ROLL IS WORTH NOTHING, AND THAT IS THE DESIGN CONSTRAINT.
Measured on the same runs: letting the gripper spin about its own approach
axis while holding the axis fixed bought **0 mm** at the work point and 3 mm
at home. It is the tool AXIS that has to tilt. A "relaxation" that only frees
the roll would look like a fix, cost a session, and measure zero -- so
`SPIN` exists here to be REJECTED by name, not offered as an option.

WHAT THIS DOES NOT CHANGE
-------------------------
HARD CONSTRAINT 1 stands: `WORKSPACE_ORIENT` is not re-derived, not moved, and
not recomputed from home. Re-deriving it to match a level home costs T2 its
right arm, and that is measured. This module does not touch the constant. It
changes only how strictly the constant is INSISTED upon, it defaults to
`EXACT` -- byte-for-byte today's behaviour -- and every relaxation it grants
is reported so it can never be a silent widening.

A TASK THAT DECLARES AN APPROACH STILL GETS IT. At the instant of a grasp the
tool axis is the approach direction and the roll sets the jaw line; both are
real requirements and `EXACT` is the right policy there. The cone is for
TRANSIT, which is most of every path and all of the part nobody was able to
see until the whole-path check was written.
"""
from __future__ import annotations

import math

# Policy names. SPIN is here to be refused: see the module docstring.
EXACT = "exact"
CONE = "cone"
FREE = "free"
SPIN = "spin"

POLICIES = (EXACT, CONE, FREE)

# The default is today's behaviour, exactly. A module that widens the
# workspace the moment it is imported is a module nobody can bisect.
DEFAULT_POLICY = EXACT
DEFAULT_CONE_DEG = 0.0

# What the measurement says is worth having. Not applied anywhere by default;
# quoted so a caller choosing a number is choosing a MEASURED one.
MEASURED_CONE_DEG = 45.0

_SPIN_REFUSAL = (
    "orientation policy 'spin' is refused by name. Freeing the gripper's "
    "roll about its own approach axis while holding the axis fixed was "
    "measured at +0 mm of reach at the work point and +3 mm at home "
    "(recordings/baselines/orientation_cost*.json). It is the tool AXIS that "
    "has to tilt. Use 'cone' with a tolerance in degrees.")


class PolicyError(ValueError):
    """A policy that cannot be honoured, named rather than silently coerced."""


def normalise(policy=None, cone_deg=None):
    """(policy, cone_RADIANS). Raises rather than guessing.

    THE UNIT CHANGES ON THE WAY OUT AND THAT HAS BITTEN. This takes DEGREES
    and returns RADIANS, while `describe()` and `candidates()` -- its two
    siblings -- both take DEGREES. `describe(*normalise("cone", 15.0))`
    therefore prints "within a 0 deg cone", which is what
    `scripts/verify_t1.py` printed on 2026-08-23 while correctly solving
    inside a 15 deg one. The value was right and the sentence was wrong,
    which is the worse way round.

    `ik_follower_node` does not hit it -- it discards this radian value and
    keeps the degrees parameter -- but nothing said why. Callers that want a
    sentence should pass the DEGREES they started with.

    `cone_deg` of 0 under CONE is EXACT and is returned as EXACT, so a caller
    that sets a tolerance of zero gets the strict path and not a cone search
    that can only ever return the pinned answer more slowly.
    """
    p = (policy or DEFAULT_POLICY).strip().lower()
    if p == SPIN:
        raise PolicyError(_SPIN_REFUSAL)
    if p not in POLICIES:
        raise PolicyError(
            "unknown orientation policy %r; expected one of %s"
            % (policy, ", ".join(POLICIES)))
    if p == FREE:
        return FREE, math.pi
    c = float(DEFAULT_CONE_DEG if cone_deg is None else cone_deg)
    if c < 0.0:
        raise PolicyError("cone tolerance %.3f deg is negative" % c)
    if c > 180.0:
        raise PolicyError(
            "cone tolerance %.1f deg exceeds 180; use policy 'free' if the "
            "orientation genuinely does not matter, so the log says so" % c)
    if p == EXACT and c > 0.0:
        raise PolicyError(
            "policy 'exact' with a %.1f deg tolerance is a contradiction. "
            "Use 'cone' if the tolerance is meant, or drop it if it is not."
            % c)
    if p == CONE and c == 0.0:
        return EXACT, 0.0
    return p, math.radians(c)


# ------------------------------------------------------------------ geometry
def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        raise PolicyError("cannot normalise a zero-length axis")
    return [x / n for x in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _q_mul(a, b):
    """Hamilton product, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _q_axis_angle(axis, ang):
    x, y, z = _norm(axis)
    s = math.sin(ang * 0.5)
    return (x * s, y * s, z * s, math.cos(ang * 0.5))


def unit(q):
    """The quaternion, normalised. NOT a formality.

    `master_calibration.WORKSPACE_ORIENT` is not stored unit -- measured
    2026-08-22, the left anchor's norm is 0.99995844 and the right's is
    1.00000561. Four parts in 100 000 sounds like nothing and is not: build a
    rotation matrix from a non-unit quaternion and it is not orthonormal, so
    a "25.0 deg" tilt about it comes out 24.9996 deg and a cone that is
    supposed to be a BOUND is exceeded by a candidate that reports it is not.
    That is what caught this -- the test that requires the reported tilt to
    be the achieved tilt, run against the real anchors instead of identity.

    Nothing upstream is changed: this normalises on the way in, it does not
    rewrite the constant. HARD CONSTRAINT 1.
    """
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise PolicyError("orientation quaternion has zero norm")
    return (x / n, y / n, z / n, w / n)


def tool_axis(q):
    """The gripper's approach direction: the tool frame's z, in world.

    `grasp_frames` reads the same column (`q_matrix(q)[:, 2]`), and the pad
    offset is measured along it. Two definitions of "which way the hand
    points" is how a 13.47 mm grasp error survived for months. The one
    difference is deliberate: this normalises first, so the axis is a unit
    vector even when the stored anchor is not.
    """
    x, y, z, w = unit(q)
    return [2 * (x * z + y * w), 2 * (y * z - x * w),
            1 - 2 * (x * x + y * y)]


def angle_between(u, v):
    """Radians between two vectors, clamped so acos cannot overflow."""
    u, v = _norm(u), _norm(v)
    d = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v))))
    return math.acos(d)


def axis_deviation_deg(q_commanded, q_achieved):
    """How far the achieved tool axis tilted from the commanded one."""
    return math.degrees(angle_between(tool_axis(q_commanded),
                                      tool_axis(q_achieved)))


# ------------------------------------------------------------- the candidates
def candidates(q, policy=None, cone_deg=None, rings=3, per_ring=8):
    """Orientations to try, NEAREST FIRST, starting with the commanded one.

    Yields (quaternion, tilt_deg, label). The first is always the commanded
    orientation at 0.0 deg, so a caller that stops at the first success under
    any policy behaves exactly as it does today whenever the exact pose
    solves. Relaxation is only ever reached after the strict answer failed.

    The tilt is applied about axes PERPENDICULAR to the tool axis, which is
    what moves the approach direction. Rotating about the tool axis itself is
    the roll, and the roll was measured worth nothing -- see the docstring.
    """
    p, cone = normalise(policy, cone_deg)
    # The COMMANDED candidate is yielded verbatim, un-normalised, because it
    # must be bit-identical to what the caller asked for -- under the default
    # policy this is the only candidate and the request has to be the request.
    # Every RELAXED candidate is built from the normalised form, so the cone
    # is a true bound.
    q_raw = tuple(float(v) for v in q)
    yield q_raw, 0.0, "commanded"
    if p == EXACT or cone <= 0.0:
        return

    q = unit(q_raw)
    z = _norm(tool_axis(q))
    # Any two unit vectors perpendicular to z. Picking the smaller component
    # to seed the cross product keeps it well conditioned for every z.
    seed = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
    u = _norm(_cross(z, seed))
    v = _norm(_cross(z, u))

    for r in range(1, rings + 1):
        tilt = cone * r / rings
        for k in range(per_ring):
            phi = 2.0 * math.pi * k / per_ring
            axis = [math.cos(phi) * u[i] + math.sin(phi) * v[i]
                    for i in range(3)]
            qq = _q_mul(_q_axis_angle(axis, tilt), q)
            n = math.sqrt(sum(c * c for c in qq))
            yield (tuple(c / n for c in qq), math.degrees(tilt),
                   "tilt %.1f deg az %.0f" % (math.degrees(tilt),
                                              math.degrees(phi)))


def describe(policy=None, cone_deg=None):
    """One line an operator can read off a screen. Never a bare number."""
    p, cone = normalise(policy, cone_deg)
    if p == EXACT:
        return ("wrist PINNED to the commanded orientation (this is the "
                "default and it is what every recorded mode used)")
    if p == FREE:
        return ("wrist FREE -- position only. The gripper may point any way "
                "at all, which is wrong at a grasp and fine in transit")
    return ("wrist within a %.0f deg cone of the commanded approach axis; "
            "roll free within it" % math.degrees(cone))


# ---------------------------------------------------------------- self-test
def self_test(verbose=True):
    """Known answers, all constructed arithmetic."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-46s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    ident = (0.0, 0.0, 0.0, 1.0)

    # 1 -- the default is today's behaviour: exactly one candidate.
    c = list(candidates(ident))
    check("default yields ONLY the commanded orientation",
          len(c) == 1 and c[0][1] == 0.0)

    # 2 -- the first candidate is always the commanded one, under every policy
    for pol, deg in ((EXACT, None), (CONE, 30.0), (FREE, None)):
        first = next(iter(candidates(ident, pol, deg)))
        check("first candidate is the commanded pose under %-5s" % pol,
              first[0] == ident and first[1] == 0.0)

    # 3 -- no candidate ever exceeds the cone. The whole safety story of a
    #      tolerance is that it is a BOUND, so this is the load-bearing one.
    worst = 0.0
    for q, tilt, _ in candidates(ident, CONE, 25.0):
        d = axis_deviation_deg(ident, q)
        worst = max(worst, d)
    check("no candidate exceeds a 25 deg cone", worst <= 25.0 + 1e-6,
          "worst %.4f deg" % worst)

    # 4 -- the reported tilt IS the achieved tilt, not a label
    worst = 0.0
    for q, tilt, _ in candidates(ident, CONE, 40.0):
        worst = max(worst, abs(axis_deviation_deg(ident, q) - tilt))
    check("reported tilt matches the achieved tilt", worst < 1e-6,
          "worst mismatch %.2e deg" % worst)

    # 5 -- nearest first: tilts are non-decreasing
    tilts = [t for _, t, _ in candidates(ident, CONE, 45.0)]
    check("candidates are ordered nearest-first",
          all(tilts[i] <= tilts[i + 1] + 1e-9 for i in range(len(tilts) - 1)))

    # 6 -- SPIN is refused BY NAME, because it measured zero
    try:
        normalise(SPIN)
        check("policy 'spin' is refused", False)
    except PolicyError as e:
        check("policy 'spin' is refused by name", "+0 mm" in str(e))

    # 7 -- contradictions are refused rather than coerced
    for bad in ((EXACT, 15.0), ("nonsense", None), (CONE, -1.0),
                (CONE, 200.0)):
        try:
            normalise(*bad)
            check("refuses %r" % (bad,), False)
        except PolicyError:
            check("refuses %r" % (bad,), True)

    # 8 -- cone with a zero tolerance IS exact, not a slow exact
    p, c8 = normalise(CONE, 0.0)
    check("cone(0 deg) collapses to exact", p == EXACT and c8 == 0.0)

    # 9 -- the tool axis agrees with grasp_frames' own column, on a
    #      non-trivial rotation. Two definitions of "which way the hand
    #      points" is how a 13.47 mm grasp error survived for months.
    q9 = _q_axis_angle([0.3, -0.7, 0.5], 1.1)
    x, y, z, w = q9
    col2 = [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]
    check("tool_axis is q_matrix(q)[:, 2]",
          max(abs(a - b) for a, b in zip(tool_axis(q9), col2)) < 1e-12)

    # 10 -- a rotation ABOUT the tool axis is never produced: every candidate
    #       that is not the commanded one must move the axis.
    moved = [axis_deviation_deg(ident, q) > 1e-9
             for q, t, _ in candidates(ident, CONE, 30.0) if t > 0]
    check("every relaxed candidate moves the AXIS, not the roll",
          moved and all(moved))

    if verbose:
        print("orientation_policy self-test %s"
              % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
