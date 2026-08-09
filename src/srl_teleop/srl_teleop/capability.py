#!/usr/bin/env python3
"""THE CAPABILITY LADDER -- what mapping to run on whatever channels survive.

DESIGN PREMISE. Any body-worn master routing 14 analog channels through
rotating joints will lose channels: connectors fatigue and conductors flex
through joints that turn. A mapping that is defined only for "all channels
healthy" is a mapping that is undefined for the condition the hardware will
spend most of its life in.

So the position mapping is not chosen once. It is SELECTED, every cycle, from
whatever set of channels currently passes the health test, and the selection
is published so the operator knows which one is running and what it cost.

WHY A LADDER AND NOT A CONTINUOUS DEGRADATION. Each rung uses a structurally
different observable, not a noisier version of the same one:

  L0 FK        7 joints -> tip pose by forward kinematics. Needs every angle
                 to be ACCURATE, not merely present, because FK compounds the
                 whole chain.
  L1 SPHERICAL j1 + (j2,j4) + IMU. Direction from gravity and the shoulder
                 roll; radius from the FK MAGNITUDE, which depends on the
                 bend joints and is insensitive to roll error.
  L2 SPH_RATE  j1 + IMU + any alive unused channel. Direction as L1; radius
                 driven as a RATE off the spare channel, because with no bend
                 joint there is no radial observable at all.
  L3 SHELL     j1 + IMU. Direction as L1; radius FIXED. The commanded set
                 collapses from a shell of finite thickness to a surface.
  L4 DIR_ONLY  IMU only. Elevation survives (gravity needs no pot) but there
                 is no azimuth and no radius, so there is NO POSITION.
  L5 NONE      nothing usable.

L4 and L5 both report position UNAVAILABLE. That is the whole point of having
them as named rungs rather than as an error: a fabricated position on a
manipulator bolted to a person is the worst available outcome, and the
difference between "no position" and "a position" has to be visible from
outside the process.

This module is PURE. It holds no ROS handles and does no I/O, so the ladder
can be unit-tested against known channel sets without a stack running.
"""

# Channel roles, per arm. The alternating roll/bend pattern of the master is
# what makes these roles distinct: a roll moves the tip only through a
# downstream bend, so it carries no radial information on its own.
AZIMUTH_CH = "j1"                     # shoulder roll: the only azimuth source
BEND_CH = ("j2", "j4")                # shoulder flexion and elbow: reach
ROLL_CH = ("j3", "j5", "j7")          # rolls: no radial information
WRIST_CH = ("j6",)                    # wrist bend

L_FK, L_SPHERICAL, L_SPH_RATE, L_SHELL, L_DIR_ONLY, L_NONE = range(6)

LEVELS = {
    L_FK: dict(
        key="FK", name="full forward kinematics",
        needs="all 7 joints accurate",
        position=True, radius="measured", azimuth="j1", elevation="IMU"),
    L_SPHERICAL: dict(
        key="SPHERICAL", name="spherical, measured radius",
        needs="j1 + at least one of j2/j4",
        position=True, radius="measured", azimuth="j1", elevation="IMU"),
    L_SPH_RATE: dict(
        key="SPH_RATE", name="spherical, radius driven as a rate",
        needs="j1 + a spare alive channel",
        position=True, radius="rate", azimuth="j1", elevation="IMU"),
    L_SHELL: dict(
        key="SHELL", name="spherical, radius frozen",
        needs="j1 + IMU",
        position=True, radius="frozen", azimuth="j1", elevation="IMU"),
    L_DIR_ONLY: dict(
        key="DIR_ONLY", name="elevation only, NO POSITION",
        needs="IMU",
        position=False, radius=None, azimuth=None, elevation="IMU"),
    L_NONE: dict(
        key="NONE", name="nothing usable, NO POSITION",
        needs="-",
        position=False, radius=None, azimuth=None, elevation=None),
}

# Measured capability cost of each rung, filled from recorded hardware data by
# scripts/measure_capability_ladder.py. Values are commanded-tip error against
# the 4-pot reference, in metres, and lost angular extent in degrees.
#
# NOTHING IS HARD-CODED HERE AS A GUESS. A rung with no measurement carries
# None and the publisher says "unmeasured" rather than printing a number.
COST_M = {}          # {level_key: {"mean_m":.., "p95_m":.., "max_m":..}}
COST_DEG = {}        # {level_key: {"axes_lost":.., "note":..}}
RADIAL_EXTENT_M = {}  # {arm: metres of radius the operator actually used}


def load_costs(path=None):
    """Load measured rung costs from the baseline written by
    scripts/measure_capability_ladder.py.

    Costs are LOADED, never hard-coded, so a number printed to the operator
    can always be traced to the run that produced it. If the baseline is
    absent every rung reports "unmeasured" rather than a plausible guess.
    """
    import json
    import os
    if path is None:
        # Walk UP looking for the baseline rather than counting "..". The
        # package is symlink-installed, so its depth below the workspace root
        # differs between the source tree and install/, and a fixed count is
        # right in one and silently wrong in the other. A wrong path here does
        # not raise; it just makes every rung report "unmeasured".
        here = os.path.dirname(os.path.abspath(__file__))
        rel = "recordings/baselines/capability_ladder.json"
        path = None
        d = here
        for _ in range(8):
            cand = os.path.join(d, rel)
            if os.path.exists(cand):
                path = cand
                break
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
        if path is None:
            return False
    try:
        d = json.load(open(path))
    except Exception:                                        # noqa: BLE001
        return False
    # Charge the WORSE of the two arms: the operator is told what the rung can
    # cost them, not the average of a good arm and a bad one.
    for arm, v in d.items():
        RADIAL_EXTENT_M[arm] = v.get("radial_extent_m")
        for key, c in v.get("cost", {}).items():
            if c.get("mean_m") is None:
                continue
            cur = COST_M.get(key)
            if cur is None or c["mean_m"] > cur["mean_m"]:
                COST_M[key] = dict(c)
    COST_DEG["DIR_ONLY"] = dict(
        axes_lost=2, note="azimuth and radius both gone; elevation survives")
    COST_DEG["SHELL"] = dict(
        axes_lost=0, note="both angles intact; the radial AXIS is removed")
    return True


load_costs()


def select(healthy, imu_ok=True, allow_rate=True):
    """Pick the best rung available on `healthy`, a set of channel names.

    Returns (level, reason). The reason names the channel that decided it, so
    the operator is told WHY they are on a rung rather than only which.
    """
    h = set(healthy)
    if not imu_ok:
        # Elevation comes from gravity. Without the inertial sensor there is
        # no elevation at any rung, and azimuth alone is not a position.
        return (L_NONE if AZIMUTH_CH not in h else L_NONE,
                "no inertial measurement: elevation unavailable")

    if AZIMUTH_CH not in h:
        # Azimuth has exactly one source. Elevation still works.
        return L_DIR_ONLY, "j1 unhealthy: no azimuth source, so no position"

    bends = [c for c in BEND_CH if c in h]
    if len(h) >= 7:
        return L_FK, "all 7 channels healthy"
    if bends:
        return L_SPHERICAL, "radius from %s" % "+".join(bends)

    spare = sorted(c for c in h if c != AZIMUTH_CH)
    if spare and allow_rate:
        return L_SPH_RATE, "no bend joint; radius driven as a rate off %s" % spare[0]
    return L_SHELL, "no bend joint and no spare channel: radius frozen"


def regain(healthy, imu_ok=True):
    """What repairing each currently-unhealthy channel would buy.

    This is the question an operator actually has, and it is not answerable by
    looking at the health table: fixing j3 while j2 and j4 are both dead moves
    nothing, and the table cannot say so. Returns {channel: (new_level,
    improves)} for every channel not currently healthy.
    """
    cur, _ = select(healthy, imu_ok)
    out = {}
    for ch in (AZIMUTH_CH,) + BEND_CH + ROLL_CH + WRIST_CH:
        if ch in healthy:
            continue
        lvl, _ = select(set(healthy) | {ch}, imu_ok)
        out[ch] = (lvl, lvl < cur)          # lower index is a better rung
    return out


def describe(level):
    d = LEVELS[level]
    cost = COST_M.get(d["key"])
    if cost is None:
        c = "cost unmeasured"
    else:
        c = "cost %.3f m mean, %.3f m p95" % (cost["mean_m"], cost["p95_m"])
    return "L%d %s (%s) -- %s" % (level, d["key"], d["name"], c)


def position_available(level):
    return LEVELS[level]["position"]
