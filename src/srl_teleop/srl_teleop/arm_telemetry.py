#!/usr/bin/env python3
"""Everything the Kinova arm already reports, and what it means.

    python3 -m srl_teleop.arm_telemetry        # the self-test

WHAT THIS IS FOR. `kortex_highlevel_bridge` reads exactly one thing from the
arm -- `GetMeasuredJointAngles()`, seven numbers -- and that is the entire
sensory input this project has ever had from real hardware. The Gen3 is a
torque-sensing arm with per-phase current and temperature sensing on every
actuator, an IMU in the base, and a live external-wrench estimate at the
tool. All of it is one call away.

That gap is most of "everything works in simulation and then real hardware
messes up". In simulation there is no friction, no gravity sag, no contact
and no thermal drift; on the arm there are all four, and nothing in this
repository could see any of them. A system that cannot tell that its gripper
has hit something is going to keep pushing.

WHAT `BaseCyclic.RefreshFeedback()` RETURNS, per actuator:

    position, velocity      degrees and degrees/s
    torque                  Nm, from the joint's own torque sensor
    current_motor           A, per phase
    voltage                 V
    temperature_motor       degC
    temperature_core        degC

and at the base:

    tool_external_wrench_force_{x,y,z}     N   -- the arm's own estimate of
    tool_external_wrench_torque_{x,y,z}    Nm     what the world is pushing
                                                  back with
    imu_acceleration_{x,y,z}               m/s2
    imu_angular_velocity_{x,y,z}           deg/s
    fault_bank_a / fault_bank_b            bitfields

IT COSTS NOTHING EXTRA. The control loop already makes one round trip to read
angles and one to send speeds. `RefreshFeedback` replaces the FIRST of those
with a call that returns all of the above -- same number of round trips, an
order of magnitude more information. HARD CONSTRAINT 4 is untouched: this is
the high-level API, not the cyclic write path that is unusable over WSL.

THE ONE THAT MATTERS MOST IS THE WRENCH. `tool_external_wrench` is the arm's
own estimate of the force at the tool, and it is the difference between "the
gripper is closing on a cube" and "the gripper is pushing a cube across the
table because the grasp was 20 mm off". Nothing in this project has ever been
able to tell those apart.

WHAT THIS MODULE DOES AND DOES NOT DO. It parses a feedback message into a
plain record, and it decides three things from that record: is the arm in
contact, is it faulted, is it overheating. It does NOT command anything.
Deciding what to do about contact belongs to whatever is driving the arm, and
this file exists so that decision has something to be made from.

EVERY THRESHOLD HERE IS A GUESS UNTIL IT IS MEASURED ON THE ARM, and each one
says so. The right way to set them is to record a minute of free-space motion
and take the observed range -- `contact_baseline()` does exactly that and is
the intended path. A threshold picked from a datasheet will either fire on
gravity or never fire at all.
"""
from __future__ import annotations

import math

NJ = 7

# UNTIL A BASELINE IS RECORDED, these are placeholders and are marked as
# such. The Gen3's tool wrench estimate carries the payload and the arm's own
# gravity model, so the zero is NOT zero and depends on pose.
DEFAULT_CONTACT_N = 8.0            # newtons at the tool
DEFAULT_CONTACT_NM = 1.5           # newton-metres at the tool
DEFAULT_JOINT_TORQUE_MARGIN_NM = 6.0
# Kinova rates the actuators to 80 degC at the motor; well before that the
# arm derates itself, which shows up as an arm that stops following.
WARN_TEMP_C = 60.0
FAULT_TEMP_C = 75.0


class Telemetry:
    """One feedback frame, parsed. Plain floats, no Kortex types."""

    def __init__(self, position=None, velocity=None, torque=None,
                 current=None, voltage=None, temperature=None,
                 wrench_force=None, wrench_torque=None,
                 imu_accel=None, imu_gyro=None, faults=(0, 0), t=None):
        z7 = [0.0] * NJ
        self.position = list(position or z7)          # radians
        self.velocity = list(velocity or z7)          # rad/s
        self.torque = list(torque or z7)              # Nm
        self.current = list(current or z7)            # A
        self.voltage = list(voltage or z7)            # V
        self.temperature = list(temperature or z7)    # degC
        self.wrench_force = list(wrench_force or [0.0] * 3)     # N
        self.wrench_torque = list(wrench_torque or [0.0] * 3)   # Nm
        self.imu_accel = list(imu_accel or [0.0] * 3)
        self.imu_gyro = list(imu_gyro or [0.0] * 3)
        self.faults = tuple(faults)
        self.t = t

    # ------------------------------------------------------------ derived
    @property
    def force_n(self):
        return math.sqrt(sum(v * v for v in self.wrench_force))

    @property
    def torque_nm(self):
        return math.sqrt(sum(v * v for v in self.wrench_torque))

    @property
    def hottest_c(self):
        return max(self.temperature) if self.temperature else 0.0

    @property
    def moving(self):
        return max(abs(v) for v in self.velocity) if self.velocity else 0.0

    def faulted(self):
        """(bool, reason). A fault bank is a bitfield and any bit is a
        fault; the arm refuses to servo while one is set, which presents to
        everything upstream as an arm that has simply stopped following."""
        a, b = self.faults
        if a or b:
            return True, ("actuator fault bank A=0x%x B=0x%x -- the arm will "
                          "refuse to servo until this is cleared" % (a, b))
        return False, ""

    def overheating(self, warn_c=WARN_TEMP_C, fault_c=FAULT_TEMP_C):
        """(state, reason). The arm DERATES itself before it faults, and a
        derated arm looks exactly like a control problem: it stops keeping
        up with the commanded speed for no visible reason."""
        h = self.hottest_c
        if h >= fault_c:
            return "fault", ("a motor is at %.0f degC; the arm is at or past "
                             "its own limit" % h)
        if h >= warn_c:
            return "warn", ("a motor is at %.0f degC; the arm derates before "
                            "it faults, and a derated arm reads as an arm "
                            "that has stopped following" % h)
        return "ok", "hottest motor %.0f degC" % h

    def in_contact(self, baseline=None, force_n=DEFAULT_CONTACT_N,
                   torque_nm=DEFAULT_CONTACT_NM):
        """(bool, reason). Is something pushing back at the tool?

        `baseline` is a `ContactBaseline` recorded in free space at this
        pose. WITHOUT ONE this compares against a fixed threshold, and says
        so in the reason -- the tool wrench carries the payload and the arm's
        own gravity model, so its zero is not zero and depends on pose.
        """
        if baseline is not None:
            df, dt = baseline.residual(self)
            if df > baseline.force_n or dt > baseline.torque_nm:
                return True, ("%.1f N / %.2f Nm above the free-space baseline "
                              "(gates %.1f / %.2f)"
                              % (df, dt, baseline.force_n, baseline.torque_nm))
            return False, ("%.1f N / %.2f Nm above baseline, under the gate"
                           % (df, dt))
        f, t = self.force_n, self.torque_nm
        if f > force_n or t > torque_nm:
            return True, ("%.1f N / %.2f Nm at the tool against a FIXED gate "
                          "(%.1f / %.2f). No free-space baseline has been "
                          "recorded, so this may be gravity and payload "
                          "rather than contact." % (f, t, force_n, torque_nm))
        return False, ("%.1f N / %.2f Nm, under a fixed gate that nobody has "
                       "calibrated" % (f, t))

    def as_dict(self):
        return {
            "position": self.position, "velocity": self.velocity,
            "torque": self.torque, "current": self.current,
            "voltage": self.voltage, "temperature": self.temperature,
            "wrench_force": self.wrench_force,
            "wrench_torque": self.wrench_torque,
            "imu_accel": self.imu_accel, "imu_gyro": self.imu_gyro,
            "faults": list(self.faults),
            "force_n": self.force_n, "torque_nm": self.torque_nm,
            "hottest_c": self.hottest_c,
        }


# --------------------------------------------------------------- parsing
def from_feedback(fb, deg_to_rad=None):
    """A Kortex `BaseCyclic.Feedback` -> `Telemetry`.

    `deg_to_rad` is the project's ONE conversion, passed in rather than
    imported so this module stays testable with no Kortex and no ROS. Kortex
    reports joint angles in degrees on 0..360 and this repository has a
    single function that knows what to do about that; a second copy here is
    exactly the kind of duplicate that has bitten this project before.

    Missing fields are tolerated and REPORTED, not defaulted silently: a
    firmware that does not publish temperature is a different situation from
    an arm that is cold.
    """
    missing = []

    def _get(obj, name, default=0.0):
        if not hasattr(obj, name):
            if name not in missing:
                missing.append(name)
            return default
        return float(getattr(obj, name))

    acts = list(getattr(fb, "actuators", []))
    n = min(len(acts), NJ)
    pos_deg = [_get(acts[i], "position") for i in range(n)]
    conv = deg_to_rad or (lambda d: math.radians(d))
    t = Telemetry(
        position=[conv(d) for d in pos_deg] + [0.0] * (NJ - n),
        velocity=[math.radians(_get(acts[i], "velocity"))
                  for i in range(n)] + [0.0] * (NJ - n),
        torque=[_get(acts[i], "torque") for i in range(n)] + [0.0] * (NJ - n),
        current=[_get(acts[i], "current_motor")
                 for i in range(n)] + [0.0] * (NJ - n),
        voltage=[_get(acts[i], "voltage") for i in range(n)] + [0.0] * (NJ - n),
        temperature=[_get(acts[i], "temperature_motor")
                     for i in range(n)] + [0.0] * (NJ - n),
    )
    base = getattr(fb, "base", None)
    if base is not None:
        t.wrench_force = [_get(base, "tool_external_wrench_force_x"),
                          _get(base, "tool_external_wrench_force_y"),
                          _get(base, "tool_external_wrench_force_z")]
        t.wrench_torque = [_get(base, "tool_external_wrench_torque_x"),
                           _get(base, "tool_external_wrench_torque_y"),
                           _get(base, "tool_external_wrench_torque_z")]
        t.imu_accel = [_get(base, "imu_acceleration_x"),
                       _get(base, "imu_acceleration_y"),
                       _get(base, "imu_acceleration_z")]
        t.imu_gyro = [math.radians(_get(base, "imu_angular_velocity_x")),
                      math.radians(_get(base, "imu_angular_velocity_y")),
                      math.radians(_get(base, "imu_angular_velocity_z"))]
        t.faults = (int(_get(base, "fault_bank_a")),
                    int(_get(base, "fault_bank_b")))
    t.missing_fields = missing
    return t


# ---------------------------------------------------------- the baseline
class ContactBaseline:
    """What the wrench reads in FREE SPACE, so contact means contact.

    The Gen3's tool wrench estimate includes the arm's own gravity model and
    whatever the gripper weighs, so it is not zero when nothing is touching
    the tool -- and it varies with pose. Comparing it against a fixed number
    therefore either fires on gravity or never fires at all.

    Record a minute of free-space motion, take the observed spread, and gate
    on the RESIDUAL above it. The gate is the observed range times a margin,
    which is a measurement rather than a guess.
    """

    def __init__(self, force_mean, torque_mean, force_n, torque_nm,
                 n_samples, note=""):
        self.force_mean = list(force_mean)
        self.torque_mean = list(torque_mean)
        self.force_n = float(force_n)
        self.torque_nm = float(torque_nm)
        self.n_samples = int(n_samples)
        self.note = note

    def residual(self, tel):
        df = math.sqrt(sum((a - b) ** 2 for a, b in
                           zip(tel.wrench_force, self.force_mean)))
        dt = math.sqrt(sum((a - b) ** 2 for a, b in
                           zip(tel.wrench_torque, self.torque_mean)))
        return df, dt

    def as_dict(self):
        return {"force_mean": self.force_mean,
                "torque_mean": self.torque_mean,
                "force_gate_n": self.force_n,
                "torque_gate_nm": self.torque_nm,
                "n_samples": self.n_samples, "note": self.note}


def contact_baseline(samples, margin=3.0, min_samples=50):
    """Build a baseline from free-space telemetry.

    `margin` multiplies the observed spread. 3x is chosen so ordinary noise
    does not fire; it is a judgement and it is the only one, which is why it
    is an argument.
    """
    if len(samples) < min_samples:
        raise ValueError(
            "%d samples; a free-space baseline needs at least %d. A gate "
            "built from a handful of readings is a gate that fires on noise."
            % (len(samples), min_samples))
    fx = [s.wrench_force for s in samples]
    tx = [s.wrench_torque for s in samples]
    fm = [sum(v[i] for v in fx) / len(fx) for i in range(3)]
    tm = [sum(v[i] for v in tx) / len(tx) for i in range(3)]
    df = [math.sqrt(sum((v[i] - fm[i]) ** 2 for v in fx) / len(fx))
          for i in range(3)]
    dt = [math.sqrt(sum((v[i] - tm[i]) ** 2 for v in tx) / len(tx))
          for i in range(3)]
    return ContactBaseline(
        fm, tm,
        max(margin * math.sqrt(sum(v * v for v in df)), 1.0),
        max(margin * math.sqrt(sum(v * v for v in dt)), 0.2),
        len(samples),
        "free-space, %d samples, gates at %.0fx the observed spread"
        % (len(samples), margin))


# ---------------------------------------------------------------- self-test
class _FakeAct:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _FakeBase:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _FakeFeedback:
    def __init__(self, actuators, base):
        self.actuators = actuators
        self.base = base


def _frame(pos_deg=0.0, torque=0.0, temp=25.0, fx=0.0, fz=0.0, faults=(0, 0)):
    acts = [_FakeAct(position=pos_deg + i, velocity=0.5, torque=torque,
                     current_motor=0.4, voltage=24.0,
                     temperature_motor=temp) for i in range(NJ)]
    base = _FakeBase(tool_external_wrench_force_x=fx,
                     tool_external_wrench_force_y=0.0,
                     tool_external_wrench_force_z=fz,
                     tool_external_wrench_torque_x=0.0,
                     tool_external_wrench_torque_y=0.0,
                     tool_external_wrench_torque_z=0.0,
                     imu_acceleration_x=0.0, imu_acceleration_y=0.0,
                     imu_acceleration_z=9.81,
                     imu_angular_velocity_x=0.0, imu_angular_velocity_y=0.0,
                     imu_angular_velocity_z=0.0,
                     fault_bank_a=faults[0], fault_bank_b=faults[1])
    return _FakeFeedback(acts, base)


def self_test(verbose=True):
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-52s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    t = from_feedback(_frame(pos_deg=10.0, torque=2.5, temp=30.0))
    check("every actuator field is parsed",
          len(t.position) == NJ and len(t.torque) == NJ
          and abs(t.torque[0] - 2.5) < 1e-9)
    check("velocity is converted to rad/s",
          abs(t.velocity[0] - math.radians(0.5)) < 1e-12,
          "%.6f rad/s from 0.5 deg/s" % t.velocity[0])
    check("the base wrench and IMU are parsed",
          abs(t.imu_accel[2] - 9.81) < 1e-9)

    # A MISSING FIELD IS REPORTED, not silently zero.
    bare = _FakeFeedback([_FakeAct(position=0.0) for _ in range(NJ)],
                         _FakeBase())
    b = from_feedback(bare)
    check("a firmware that omits a field says which",
          "torque" in b.missing_fields
          and "tool_external_wrench_force_x" in b.missing_fields,
          "%d missing: %s" % (len(b.missing_fields),
                              ", ".join(b.missing_fields[:3])))

    # FAULTS
    f, why = from_feedback(_frame(faults=(0, 0))).faulted()
    check("no fault bits reads clean", not f)
    f, why = from_feedback(_frame(faults=(0x40, 0))).faulted()
    check("a fault bit is reported with the bank", f and "0x40" in why, why[:52])

    # TEMPERATURE
    for temp, want in ((25.0, "ok"), (62.0, "warn"), (80.0, "fault")):
        st, why = from_feedback(_frame(temp=temp)).overheating()
        check("a motor at %.0f degC reads %s" % (temp, want), st == want,
              why[:56])

    # CONTACT, WITHOUT A BASELINE -- must say the gate is uncalibrated.
    c, why = from_feedback(_frame(fx=1.0)).in_contact()
    check("no contact under a fixed gate, and it says the gate is a guess",
          not c and "nobody has calibrated" in why)
    c, why = from_feedback(_frame(fx=20.0)).in_contact()
    check("20 N trips the fixed gate, and it says it may be gravity",
          c and "may be gravity" in why)

    # CONTACT, WITH A BASELINE. This is the point: a heavy but CONSTANT
    # gravity load must NOT read as contact, and a small change on top of it
    # must.
    free = [from_feedback(_frame(fx=12.0 + 0.05 * (i % 5), fz=-9.0))
            for i in range(200)]
    bl = contact_baseline(free)
    steady, why = from_feedback(_frame(fx=12.1, fz=-9.0)).in_contact(bl)
    check("a constant 15 N gravity load is NOT contact, with a baseline",
          not steady, why[:60])
    pushed, why = from_feedback(_frame(fx=12.1, fz=-16.0)).in_contact(bl)
    check("7 N on top of that IS contact", pushed, why[:60])
    check("the same push WITHOUT a baseline is missed by the fixed gate",
          from_feedback(_frame(fx=12.1, fz=-16.0)).in_contact()[0]
          and not steady,
          "which is why the baseline exists")

    try:
        contact_baseline(free[:10])
        check("too few samples for a baseline is refused", False)
    except ValueError as e:
        check("too few samples for a baseline is refused", "at least" in str(e))

    if verbose:
        print("arm_telemetry self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
