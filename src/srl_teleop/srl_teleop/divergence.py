#!/usr/bin/env python3
"""Sim-vs-real divergence: the number, and the four ways it can be a lie.

WHAT THIS IS FOR. The cascade drives the real arm from the sim arm, and the
lag monitor trips at `lag_trip_rad`. That threshold is currently a number
nobody can see: it fires, or it does not, and the operator learns which from
an e-stop. This module computes what the monitor is comparing so a GUI can
show it continuously.

WHY IT IS NOT JUST A SUBTRACTION
--------------------------------
`max |sim - real|` over seven joints is four lines of arithmetic. Every hard
part is deciding whether that number MEANS anything, and this rig supplies
four separate ways for it to be meaningless while looking perfect:

  1. THE REAL ARM IS NOT PUBLISHING AT ALL. Subtracting a missing thing gives
     0.000 rad, which renders as flawless tracking. This is the failure the
     GUI once shipped in another costume -- "IK BLOCKED, 0% success" computed
     over ZERO attempts, which pointed the operator at the solver instead of
     at the missing input.

  2. `/real/joint_states` FREEZES RATHER THAN STOPPING. Measured, 2026-07-31:
     with the hardware component inactive the broadcaster kept publishing at
     100 Hz with ONE distinct value per joint, peak-to-peak 0.000e+00,
     identical to a reading taken 40 minutes earlier. Arrival-rate freshness
     cannot see this -- the topic is at full rate. Only the CONTENT gives it
     away. This is the same shape as the dead-man that could not fire because
     master_pose_node republished stale frames at 50 Hz.

  3. PARTIAL JOINT COVERAGE. If the real side reports three joints of seven,
     "max divergence 0.02 rad" is true of the three and silent about the four
     that might be anywhere. A maximum over a subset is not a maximum.

  4. THE ARMS ARE COMPARED WHILE ONE IS STILL HOMING. Not an error, but a
     divergence measured during a commanded transit is not tracking error,
     and reporting it as one invites someone to retune a threshold against it.

So `compare()` returns a STATUS, and only one status carries a number. The
rule throughout is the project's own: a missing measurement is reported as
missing, never as a value.

FRESH, FROZEN AND ABSENT ARE THREE STATES, NOT TWO -- exactly as `blocked`,
`expired` and `clear` are three states in BlockMonitor, and for the same
reason. "Nobody is disagreeing with me" is not evidence of agreement.
"""
import time

# Statuses. Only OK carries numbers; every other one carries a reason.
OK = "ok"
NO_SIM = "no_sim"
NO_REAL = "no_real"
SIM_STALE = "sim_stale"
REAL_STALE = "real_stale"
REAL_FROZEN = "real_frozen"
PARTIAL = "partial"

# A side is STALE if nothing has ARRIVED for this long.
STALE_S = 1.0
# A side is FROZEN if messages keep arriving but the CONTENT has not changed
# for this long. Deliberately longer than STALE_S: a real arm genuinely holds
# still, and calling that frozen would cry wolf on every pause. 5 s of
# bit-identical joint values while the sim is moving is not a pause.
FROZEN_S = 5.0
# Below this, two floats are "the same value" for freeze detection. The real
# encoders dither; the frozen cache does not move at all. Measured p-p on a
# frozen stream was 0.000e+00, so any non-zero tolerance is generous.
FREEZE_EPS = 1e-9


class Side:
    """One joint-state stream, tracking arrival AND change separately.

    Keeping the two apart is the whole point. `last_arrival` answers "is
    anything publishing"; `last_change` answers "is what it publishes alive".
    A stream can be perfect on the first and dead on the second, and that
    combination is the one this rig actually produces.
    """

    def __init__(self, name):
        self.name = name
        self.pos = {}                  # joint name -> radians
        self.last_arrival = None       # monotonic
        self.last_change = None        # monotonic
        self.n_msgs = 0
        self.n_changes = 0

    def update(self, names, positions, now=None):
        now = time.monotonic() if now is None else now
        self.n_msgs += 1
        self.last_arrival = now
        new = {n: float(p) for n, p in zip(names, positions)}
        changed = any(abs(new[k] - self.pos.get(k, float("inf"))) > FREEZE_EPS
                      for k in new)
        self.pos.update(new)
        if changed or self.last_change is None:
            self.last_change = now
            self.n_changes += 1

    def arrival_age(self, now=None):
        if self.last_arrival is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_arrival

    def change_age(self, now=None):
        if self.last_change is None:
            return None
        return (time.monotonic() if now is None else now) - self.last_change


class Result:
    """A divergence reading, or the reason there is not one."""

    def __init__(self, status, reason, per_joint=None, max_rad=0.0,
                 max_joint=None, n_compared=0, n_expected=0, ee_m=None,
                 ee_reason=""):
        self.status = status
        self.reason = reason
        self.per_joint = per_joint or {}
        self.max_rad = max_rad
        self.max_joint = max_joint
        self.n_compared = n_compared
        self.n_expected = n_expected
        self.ee_m = ee_m
        self.ee_reason = ee_reason

    @property
    def measured(self):
        """True only when the numbers may be read as a measurement."""
        return self.status == OK

    def text(self):
        if not self.measured:
            return "--"
        return "%.4f rad" % self.max_rad

    def __repr__(self):
        return "Result(%s, %s, max=%.4f, n=%d/%d)" % (
            self.status, self.reason, self.max_rad, self.n_compared,
            self.n_expected)


def joint_names(arm):
    return ["%s_joint_%d" % (arm, i) for i in range(1, 8)]


def compare(sim, real, names, now=None, stale_s=STALE_S, frozen_s=FROZEN_S,
            sim_is_moving=None):
    """Compare two Sides over `names`. Returns a Result.

    `sim_is_moving` is optional and only affects the FROZEN verdict: a real
    stream that has not changed is only suspicious while the SIM is moving.
    Passing None means "unknown", and unknown is treated as moving -- the
    conservative direction, because a false FROZEN costs a glance at a label
    and a missed FROZEN costs the belief that the arm is tracking.
    """
    now = time.monotonic() if now is None else now
    n_expected = len(names)

    if sim.last_arrival is None:
        return Result(NO_SIM, "the sim arm has never published %s"
                      % _first(names), n_expected=n_expected)
    if real.last_arrival is None:
        return Result(NO_REAL,
                      "the real arm has never published. This is NOT zero "
                      "divergence -- there is nothing to compare.",
                      n_expected=n_expected)

    sa = now - sim.last_arrival
    ra = now - real.last_arrival
    if sa > stale_s:
        return Result(SIM_STALE, "sim joint states %.1f s old" % sa,
                      n_expected=n_expected)
    if ra > stale_s:
        return Result(REAL_STALE,
                      "real joint states %.1f s old -- the last comparison is "
                      "history, not a measurement" % ra,
                      n_expected=n_expected)

    rc = real.change_age(now)
    moving = True if sim_is_moving is None else bool(sim_is_moving)
    if moving and rc is not None and rc > frozen_s:
        return Result(REAL_FROZEN,
                      "real joint states ARRIVING at %d msgs but UNCHANGED "
                      "for %.1f s. A frozen cache reads as a held pose; the "
                      "hardware component may be inactive."
                      % (real.n_msgs, rc),
                      n_expected=n_expected)

    per, missing = {}, []
    for n in names:
        if n in sim.pos and n in real.pos:
            per[n] = sim.pos[n] - real.pos[n]
        else:
            missing.append(n)
    if missing:
        return Result(PARTIAL,
                      "only %d of %d joints are on both sides; missing %s. A "
                      "maximum over a subset is not a maximum."
                      % (len(per), n_expected, ", ".join(missing[:3])),
                      per_joint=per, n_compared=len(per),
                      n_expected=n_expected)

    mj = max(per, key=lambda k: abs(per[k]))
    return Result(OK, "%d joints compared" % len(per), per_joint=per,
                  max_rad=abs(per[mj]), max_joint=mj, n_compared=len(per),
                  n_expected=n_expected)


def ee_distance(p_sim, p_real):
    """(metres, reason). Either point being absent gives None, never 0.0.

    Same rule as the joint comparison, restated because this is the number an
    operator reads first and it is the easiest one to fake: the distance
    between a pose and a missing pose is not zero.
    """
    if p_sim is None and p_real is None:
        return None, "neither end effector is in TF"
    if p_sim is None:
        return None, "no sim end-effector transform"
    if p_real is None:
        return None, ("no real end-effector transform -- the real arm publishes\n"
                      "            prefixed frames (real_*) into the SHARED /tf")
    d = sum((a - b) ** 2 for a, b in zip(p_sim, p_real)) ** 0.5
    return d, "world frame"


def band(max_rad, trip_rad):
    """Where a divergence sits against the lag monitor's trip threshold.

    Returned as a name rather than a colour so the rule is testable without a
    toolkit, and so the GUI cannot quietly disagree with the monitor.
    """
    if max_rad is None:
        return "unknown"
    if trip_rad <= 0:
        return "unknown"
    f = max_rad / trip_rad
    if f >= 1.0:
        return "trip"                 # the monitor would fire
    if f >= 0.6:
        return "near"
    if f >= 0.3:
        return "watch"
    return "ok"


def _first(names):
    return names[0] if names else "(no joints)"


def self_test():
    """PROVE A ZERO CAN BE TOLD FROM A BLANK.

    The reading this module exists to produce is "0.0000 rad", and that is
    also what a broken comparison produces. So the module ships with the
    demonstration: two identical live streams must report OK with 0.0000, and
    a missing real side must report NO_REAL with NO number -- and the two must
    not render the same.
    """
    t = 1000.0
    names = joint_names("left")

    sim = Side("sim")
    sim.update(names, [0.1] * 7, now=t)
    real = Side("real")
    real.update(names, [0.1] * 7, now=t)
    a = compare(sim, real, names, now=t)

    lonely = Side("real")
    b = compare(sim, lonely, names, now=t)

    ok = (a.status == OK and a.max_rad == 0.0 and a.measured
          and b.status == NO_REAL and not b.measured and b.text() == "--")
    return ok, a, b


if __name__ == "__main__":
    good, a, b = self_test()
    print("identical streams : %r  -> %s" % (a, a.text()))
    print("real absent       : %r  -> %s" % (b, b.text()))
    print("\nA TRUE ZERO AND A BLANK ARE DISTINGUISHABLE: %s"
          % ("YES" if good else "NO"))
