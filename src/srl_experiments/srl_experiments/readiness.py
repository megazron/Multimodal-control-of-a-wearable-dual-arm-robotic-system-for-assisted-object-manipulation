#!/usr/bin/env python3
"""READY TO RUN — can a session start now, and if not, exactly what is wrong?

THE ONE RULE: NEVER GREEN ON UNKNOWN.

"Not checked" must be structurally incapable of reading as "healthy". This
project has been bitten by silent acceptance three times in one week -- a
volatile QoS that never errored, an RViz config key in the wrong place that
never errored, a mesh install rule that never errored -- and in each case
something absent looked exactly like something fine.

So the guarantee here is not "we remember to check". It is:

  * every check has one of THREE states: OK, FAIL, UNKNOWN. There is no
    boolean anywhere in this module, because a boolean cannot represent
    "nobody asked".
  * REQUIRED lists every check that must report. `evaluate()` starts from
    that list with every entry UNKNOWN and lets checks overwrite. A check
    that crashes, is skipped, or is never wired leaves UNKNOWN behind --
    it cannot leave OK behind, because it never wrote anything.
  * `ready` is `all(state is OK)` over REQUIRED. An UNKNOWN is therefore
    exactly as blocking as a FAIL, and the two are reported DIFFERENTLY so
    the operator can tell "this is broken" from "I do not know".

That last distinction is the point of the whole module. A red light tells you
to fix something; a grey light tells you the instrument is not looking.
"""
import time

OK = "ok"
FAIL = "fail"
UNKNOWN = "unknown"

# Every check that must report before a session may start. Adding a name here
# and forgetting to implement it makes the system NOT READY -- which is the
# correct direction to fail in.
REQUIRED = (
    "channels",          # master channels healthy for the current mode
    "cameras",           # both wrist cameras publishing
    "estop",             # clear AND tested this session
    "homed",             # both arms at home
    "scene",             # calibrated and matching the stored fingerprint
    "disk",              # room for the session's data
    "one_stack",         # exactly one stack running
    "recovery",          # recovery_manager alive
    "command_path",      # the mode's command path verified reaching the arm
)

# What each check means, in the words shown to the operator.
LABEL = {
    "channels": "master channels",
    "cameras": "wrist cameras",
    "estop": "e-stop clear and tested",
    "homed": "arms at home",
    "scene": "scene calibrated",
    "disk": "disk space",
    "one_stack": "exactly one stack",
    "recovery": "recovery manager",
    "command_path": "command path to arm",
}


class Check:
    """One check's verdict. `state` is OK / FAIL / UNKNOWN, never a bool."""

    __slots__ = ("name", "state", "detail", "stamp")

    def __init__(self, name, state=UNKNOWN, detail="not checked", stamp=None):
        if state not in (OK, FAIL, UNKNOWN):
            raise ValueError("state must be OK/FAIL/UNKNOWN, got %r" % state)
        self.name = name
        self.state = state
        self.detail = detail
        self.stamp = stamp

    def as_dict(self):
        return dict(name=self.name, state=self.state, detail=self.detail,
                    label=LABEL.get(self.name, self.name), stamp=self.stamp)

    def __repr__(self):
        return "<Check %s %s %r>" % (self.name, self.state, self.detail)


class Readiness:
    """The whole verdict. Construct via evaluate()."""

    def __init__(self, checks):
        # Start from REQUIRED, all UNKNOWN. Anything the caller supplies
        # overwrites; anything it forgot stays UNKNOWN. This is the structural
        # guarantee -- a missing check cannot become a pass.
        self.checks = {n: Check(n) for n in REQUIRED}
        for c in checks:
            if c.name in self.checks:
                self.checks[c.name] = c
            else:
                # An unexpected check is reported but cannot make us ready.
                self.checks[c.name] = c

    @property
    def failing(self):
        return [c for n, c in sorted(self.checks.items())
                if n in REQUIRED and c.state == FAIL]

    @property
    def unknown(self):
        return [c for n, c in sorted(self.checks.items())
                if n in REQUIRED and c.state == UNKNOWN]

    @property
    def ready(self):
        """OK only when EVERY required check said OK. Unknown blocks."""
        return all(self.checks[n].state == OK for n in REQUIRED)

    def summary(self):
        if self.ready:
            return "READY"
        bits = []
        if self.failing:
            bits.append("FAILING: " + ", ".join(
                "%s (%s)" % (LABEL.get(c.name, c.name), c.detail)
                for c in self.failing))
        if self.unknown:
            bits.append("NOT CHECKED: " + ", ".join(
                LABEL.get(c.name, c.name) for c in self.unknown))
        return "NOT READY — " + "; ".join(bits)

    def as_dict(self):
        return dict(ready=self.ready,
                    summary=self.summary(),
                    n_fail=len(self.failing),
                    n_unknown=len(self.unknown),
                    checks={n: c.as_dict() for n, c in self.checks.items()})


def evaluate(probe):
    """Run every required check through `probe`, tolerating a broken probe.

    `probe` supplies the raw facts; this decides the verdicts. A probe method
    that raises leaves the check UNKNOWN with the exception in the detail --
    it does NOT leave it OK, and it does not take the whole evaluation down
    with it. One dead sensor must not blind the other eight.
    """
    out = []
    now = time.time()

    def run(name, fn):
        try:
            state, detail = fn()
        except Exception as e:                                   # noqa: BLE001
            out.append(Check(name, UNKNOWN,
                             "check raised: %s" % str(e)[:60], now))
            return
        out.append(Check(name, state, detail, now))

    run("channels", probe.channels)
    run("cameras", probe.cameras)
    run("estop", probe.estop)
    run("homed", probe.homed)
    run("scene", probe.scene)
    run("disk", probe.disk)
    run("one_stack", probe.one_stack)
    run("recovery", probe.recovery)
    run("command_path", probe.command_path)
    return Readiness(out)
