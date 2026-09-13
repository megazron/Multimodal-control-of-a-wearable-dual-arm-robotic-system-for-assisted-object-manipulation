#!/usr/bin/env python3
"""How the joints get from where they are to where the IK solution says.

    python3 -m srl_teleop.motion_generator          # the self-test
    python3 -m srl_teleop.motion_generator --limits # what it will enforce

PURE. No ROS, no robot, no display -- like `orientation_policy` and
`joint_planner`, so the self-test runs offline against constructed answers and
`scripts/measure_teleop_motion.py` measures the SAME code the follower runs.
Two implementations of "how the arm moves" is how this repository's worst
measurement bugs happened, and a motion generator nobody can test offline is
one that gets tuned on a person's back.

WHAT IT REPLACES, AND WHY
-------------------------
`ik_follower_node.clamp_towards()` was the whole of motion generation for
teleoperation. It takes the IK solution and walks toward it, clamping EACH
JOINT INDEPENDENTLY to `max_step_rad` per cycle. Three separable faults, all
measured by `scripts/measure_teleop_motion.py` on a realistic slew from the
left arm's shipped home (q0 + [0.50, -0.30, 0.10, 0.25, -0.05, 0.15, 0.02]):

1. IT IS NOT SYNCHRONISED. A joint needing 0.10 rad arrives in one cycle; a
   joint needing 0.50 rad takes two. The path through joint space is
   therefore NOT the straight line the IK solution implies, and the end
   effector traces a curve nobody asked for -- **51.3 mm** of it at the
   shipped 0.35 rad step, on a rig whose grasp capture gate is 30 mm. And it
   does not shrink with the step size: at 0.02 rad per cycle it is still
   48.4 mm, because the desynchronisation is PROPORTIONAL. It is the shape of
   the algorithm, not a tuning value.

2. IT IS NOT CONTINUOUS. Each cycle emits a position with no velocity and no
   acceleration continuity: from rest, the first cycle asks for a full
   `max_step` immediately. The arm sees a staircase.

3. IT NEVER READ THE JOINT LIMITS. `max_step_rad` x rate is the only speed
   bound there has ever been, and at the shipped 0.35 rad at 50 Hz that is
   **17.5 rad/s** against the 1.3963 / 1.2218 rad/s in
   `src/srl_moveit_config/config/joint_limits.yaml` -- **12.5x**, and the
   limits differ per joint, which nothing anywhere expressed.

WHAT THIS IS
------------
[Ruckig](https://github.com/pantor/ruckig) -- time-optimal, jerk-limited,
per-DoF limits, MIT, C++17, 19.8 us mean for 7 DoF, and it takes a non-zero
TARGET velocity, which is what a streaming teleop target actually is
(paper: https://arxiv.org/pdf/2105.04830).

`Synchronization.Phase` is requested, not `Time`. Both make every joint
ARRIVE together; only phase synchronisation keeps them on the straight line
BETWEEN the endpoints, and that line is the whole point. Measured on the case
above: phase 0.02 mm of path deviation, time 9.5 mm, clamp 51.3 mm. Ruckig
falls back to time synchronisation on its own when phase is impossible (a
current velocity that does not point at the new target -- i.e. most of
streaming teleop), and `Step.synchronisation` reports which was achieved
rather than which was asked for.

WHAT IS MEASURED HERE AND WHAT IS ASSUMED
-----------------------------------------
Said plainly, because a limit nobody can trace is a limit somebody widens:

  VELOCITY      MEASURED, in the sense that it is declared by the robot's own
                configuration. Read per joint from `joint_limits.yaml`, which
                is the file MoveIt plans against. Never widened by anything
                here; `velocity_cap_rad_s` can only lower it.

  ACCELERATION  ASSUMED. `joint_limits.yaml` says `has_acceleration_limits:
                false` and `max_acceleration: 0` for all fourteen joints, so
                there is nothing to read. It is derived as `v_max /
                accel_ramp_s` -- "reach the velocity limit in this long" --
                and `Limits.provenance` says ASSUMED for as long as that is
                true. If the yaml ever declares a real one, it is used and
                the provenance changes with it.

  JERK          ASSUMED, the same way: `a_max / jerk_ramp_s`.

The assumption is bounded, and this is why it is tolerable: the acceleration
and jerk limits shape HOW the velocity limit is approached. They cannot make
any joint exceed the velocity limit, and the velocity limit is the robot's
own. An assumed acceleration buys smoothness; it cannot buy speed.

THE FALLBACK IS SYNCHRONISED TOO
--------------------------------
If ruckig is not importable the generator does NOT quietly become
`clamp_towards` again. It uses `SynchronisedClamp`: ONE scale factor for the
whole joint vector, so the commanded path stays exactly on the straight line
and every joint still arrives on the same cycle, with the per-joint velocity
limits enforced by that same factor. What it does not give is jerk limiting,
and it says so -- `Step.backend` reads `synchronised-clamp` and
`Step.reason` names the missing package and the pip command. A fallback that
is silent about being a fallback is the "feature present but does nothing"
row of docs/ENGINEERING_LOG.md's own table.

CONTINUOUS JOINTS
-----------------
joint_1/3/5/7 are `type="continuous"` and `/joint_states` reports them
unwrapped, so they wind up (the right arm reached joint_1 = -15.6 rad this
way). The generator keeps its state UNWRAPPED -- a profile cannot be
jerk-limited across a seam that teleports -- folds each new target to the
shortest way round from that state, and wraps only the copy it hands back.
`Step.travel_rad` is the TRUE unwrapped distance, which is what the
controller has to travel and what `time_from_start` must be derived from.
"""
from __future__ import annotations

import math
import os

DOF = 7

# 0-based indices of the joints declared type="continuous" in gen3_macro.xacro:
# joint_1, joint_3, joint_5, joint_7.
#
# THIS IS THE ONE SOURCE. `ik_follower_node` imports it from here rather than
# declaring its own copy; `test_motion_generator` asserts there is exactly one
# definition in the package. Two lists of "which joints can wind up" is the
# same class of defect as two home poses.
CONTINUOUS_IDX = (0, 2, 4, 6)

TWO_PI = 2.0 * math.pi

_HERE = os.path.dirname(os.path.abspath(__file__))
# src/srl_teleop/srl_teleop -> src/srl_teleop -> src -> <ws>
_WS = os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))
JOINT_LIMITS_YAML = os.path.join(
    _WS, "src/srl_moveit_config/config/joint_limits.yaml")

# The ramps the assumed limits are built from. Named so the assumption is a
# number somebody can argue with rather than a magic constant.
DEFAULT_ACCEL_RAMP_S = 0.25     # v_max reached in this long
DEFAULT_JERK_RAMP_S = 0.10      # a_max reached in this long

MEASURED = "joint_limits.yaml"
ASSUMED = "ASSUMED"


class MotionError(Exception):
    """A refusal. Never a silent fallback, never a coerced value."""


# --------------------------------------------------------------- the limits

def wrap_pi(a):
    """Wrap an angle into (-pi, pi]."""
    return math.pi - ((math.pi - a) % TWO_PI)


def ang_diff(a, b):
    """Shortest signed distance a-b, in (-pi, pi]."""
    return wrap_pi(a - b)


def shortest_delta(target, current, continuous_idx=CONTINUOUS_IDX):
    """Per-joint move from current to target, shortest-way for the rolls."""
    return [ang_diff(t, c) if i in continuous_idx else t - c
            for i, (t, c) in enumerate(zip(target, current))]


class Limits:
    """Per-joint velocity, acceleration and jerk, each with its provenance."""

    def __init__(self, velocity, acceleration, jerk, provenance, source):
        self.velocity = list(velocity)
        self.acceleration = list(acceleration)
        self.jerk = list(jerk)
        self.provenance = dict(provenance)
        self.source = source
        for name in ("velocity", "acceleration", "jerk"):
            vals = getattr(self, name)
            if len(vals) != DOF:
                raise MotionError("%s limits: %d values, need %d"
                                  % (name, len(vals), DOF))
            for i, v in enumerate(vals):
                if not (v > 0.0) or not math.isfinite(v):
                    raise MotionError(
                        "%s limit for joint_%d is %r. A non-positive limit "
                        "cannot be planned inside; it is refused rather than "
                        "substituted." % (name, i + 1, v))

    def scaled_velocity(self, cap_rad_s):
        """A copy with every velocity limit lowered to at most `cap_rad_s`.

        LOWERED ONLY. The cascade and real_robot caps are safety measures; a
        cap that could raise a joint above the robot's own declared limit
        would be a widening dressed as a limit.
        """
        if cap_rad_s is None:
            return self
        cap = float(cap_rad_s)
        if not (cap > 0.0):
            raise MotionError("velocity cap %r is not positive" % (cap_rad_s,))
        prov = dict(self.provenance)
        if cap < max(self.velocity):
            prov["velocity"] = "%s, capped to %.4f rad/s" % (
                prov.get("velocity", MEASURED), cap)
        return Limits([min(v, cap) for v in self.velocity],
                      self.acceleration, self.jerk, prov, self.source)

    def describe(self):
        v, a, j = self.velocity, self.acceleration, self.jerk
        return ("velocity %.4f-%.4f rad/s (%s); acceleration %.3f-%.3f "
                "rad/s^2 (%s); jerk %.2f-%.2f rad/s^3 (%s)"
                % (min(v), max(v), self.provenance["velocity"],
                   min(a), max(a), self.provenance["acceleration"],
                   min(j), max(j), self.provenance["jerk"]))


def _read_yaml_limits(arm, path=None):
    """(velocity, acceleration) per joint, straight from joint_limits.yaml.

    acceleration entries are None where the file declares none, so the caller
    has to decide what to do about it rather than receiving a zero it might
    use.
    """
    import yaml
    path = path or JOINT_LIMITS_YAML
    if not os.path.exists(path):
        raise MotionError(
            "joint_limits.yaml not found at %s. This module refuses to invent "
            "a velocity limit; the file is the robot's own declaration and it "
            "is what MoveIt plans against." % path)
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    block = doc.get("joint_limits") or {}
    vel, acc = [], []
    for i in range(1, DOF + 1):
        name = "%s_joint_%d" % (arm, i)
        if name not in block:
            raise MotionError(
                "%s has no entry in %s. A joint with no declared limit is not "
                "given a default here." % (name, path))
        entry = block[name] or {}
        if not entry.get("has_velocity_limits"):
            raise MotionError(
                "%s declares has_velocity_limits: false in %s. There is "
                "nothing to enforce and nothing is assumed." % (name, path))
        v = float(entry.get("max_velocity", 0.0))
        if not (v > 0.0):
            raise MotionError("%s: max_velocity %r in %s" % (name, v, path))
        vel.append(v)
        a = entry.get("max_acceleration", 0.0)
        acc.append(float(a) if entry.get("has_acceleration_limits")
                   and float(a or 0.0) > 0.0 else None)
    return vel, acc


def limits_for(arm, accel_ramp_s=DEFAULT_ACCEL_RAMP_S,
               jerk_ramp_s=DEFAULT_JERK_RAMP_S, velocity_cap_rad_s=None,
               path=None):
    """The limits this generator will enforce for `arm`, with provenance.

    Velocity comes from `joint_limits.yaml`. Acceleration comes from the same
    file WHEN IT DECLARES ONE -- today it does not, for any of the fourteen
    joints -- and is otherwise derived from `accel_ramp_s` and labelled
    ASSUMED. Jerk is always derived, and always labelled.
    """
    if not (accel_ramp_s > 0.0) or not (jerk_ramp_s > 0.0):
        raise MotionError("ramp times must be positive: accel %r jerk %r"
                          % (accel_ramp_s, jerk_ramp_s))
    vel, acc_declared = _read_yaml_limits(arm, path)
    declared = [a for a in acc_declared if a is not None]
    if len(declared) == DOF:
        acc = list(acc_declared)
        acc_prov = MEASURED
    else:
        acc = [v / accel_ramp_s for v in vel]
        acc_prov = ("%s: v_max / %.2f s. joint_limits.yaml declares "
                    "has_acceleration_limits: false" % (ASSUMED, accel_ramp_s))
    jerk = [a / jerk_ramp_s for a in acc]
    jerk_prov = "%s: a_max / %.2f s" % (ASSUMED, jerk_ramp_s)
    lim = Limits(vel, acc, jerk,
                 {"velocity": MEASURED, "acceleration": acc_prov,
                  "jerk": jerk_prov},
                 path or JOINT_LIMITS_YAML)
    return lim.scaled_velocity(velocity_cap_rad_s)


# ---------------------------------------------------------------- one step

class Step:
    """One control cycle's command, and everything the caller must be told.

    Nothing here is optional reporting. `backend` and `synchronisation` are
    what stop a fallback being invisible; `complete` and `eta_s` are what stop
    an unreachable target being silently truncated -- `clamp_towards` returned
    a clamped position and no way at all to tell "nearly there" from "45
    seconds away".
    """

    __slots__ = ("position", "position_unwrapped", "velocity", "acceleration",
                 "travel_rad", "remaining_rad", "eta_s", "complete", "backend",
                 "synchronisation", "reason", "limited_by")

    def __init__(self, position, position_unwrapped, velocity, acceleration,
                 travel_rad, remaining_rad, eta_s, complete, backend,
                 synchronisation, reason, limited_by):
        self.position = position
        self.position_unwrapped = position_unwrapped
        self.velocity = velocity
        self.acceleration = acceleration
        self.travel_rad = travel_rad
        self.remaining_rad = remaining_rad
        self.eta_s = eta_s
        self.complete = complete
        self.backend = backend
        self.synchronisation = synchronisation
        self.reason = reason
        self.limited_by = limited_by

    def __repr__(self):
        return ("Step(%s/%s travel %.4f remaining %.4f eta %.3fs%s)"
                % (self.backend, self.synchronisation, self.travel_rad,
                   self.remaining_rad, self.eta_s,
                   "" if self.complete else " INCOMPLETE"))


def _wrap_out(q, continuous_idx=CONTINUOUS_IDX):
    return [wrap_pi(v) if i in continuous_idx else v
            for i, v in enumerate(q)]


def have_ruckig():
    """Is the jerk-limited backend importable in THIS interpreter."""
    try:
        import ruckig                                          # noqa: F401
        return True
    except Exception:                                          # noqa: BLE001
        return False


RUCKIG_INSTALL = ("pip install --user --no-deps --break-system-packages "
                  "ruckig   # no dependencies: it cannot move numpy")


class MotionGenerator:
    """Streaming joint motion for one arm, jerk-limited and synchronised.

    Usage, one arm, one instance, one control loop:

        gen = MotionGenerator("left")
        gen.resync(measured_q)                  # at start, and after a refusal
        step = gen.step(ik_solution, dt)        # every cycle
        publish(step.position)

    THE STATE IT INTEGRATES IS ITS OWN OUTPUT, not `/joint_states`. Feeding
    the measured state back every cycle would put the controller's tracking
    error inside the jerk limit and the profile would chatter. `resync()` is
    the deliberate way to snap back to reality, and the follower calls it
    whenever it REFUSES to publish -- a command that was blocked must not
    leave the generator believing the arm went there.
    """

    def __init__(self, arm, limits=None, accel_ramp_s=DEFAULT_ACCEL_RAMP_S,
                 jerk_ramp_s=DEFAULT_JERK_RAMP_S, velocity_cap_rad_s=None,
                 backend="auto", continuous_idx=CONTINUOUS_IDX,
                 limits_path=None):
        self.arm = arm
        self.continuous_idx = tuple(continuous_idx)
        self.limits = limits or limits_for(
            arm, accel_ramp_s, jerk_ramp_s, velocity_cap_rad_s, limits_path)
        if backend not in ("auto", "ruckig", "clamp"):
            raise MotionError(
                "backend %r: expected auto, ruckig or clamp. 'clamp' is the "
                "SYNCHRONISED clamp, not ik_follower_node.clamp_towards -- "
                "that one is kept importable for measurement only." % backend)
        self._want = backend
        self._ruckig = None
        self._reason = ""
        if backend in ("auto", "ruckig"):
            if have_ruckig():
                self.backend = "ruckig"
            elif backend == "ruckig":
                raise MotionError(
                    "backend 'ruckig' asked for and the package will not "
                    "import. Refusing rather than degrading silently. %s"
                    % RUCKIG_INSTALL)
            else:
                self.backend = "synchronised-clamp"
                self._reason = ("ruckig is not importable in this "
                                "interpreter, so motion is synchronised and "
                                "velocity-limited but NOT jerk-limited. %s"
                                % RUCKIG_INSTALL)
        else:
            self.backend = "synchronised-clamp"
            self._reason = "backend 'clamp' asked for explicitly"
        self.pos = [0.0] * DOF
        self.vel = [0.0] * DOF
        self.acc = [0.0] * DOF
        self.resyncs = 0
        self.replans = 0
        self.phase_steps = 0
        self.time_steps = 0
        self._inp = None
        self._out = None
        self._sync = "phase"
        self._last_target = None
        self._goal = None

    # ------------------------------------------------------------- state

    def resync(self, q):
        """Snap the internal state to `q` and stop. The deliberate discontinuity.

        Called at startup, and by the follower every time it refuses to
        publish. The alternative -- letting the profile run on while nothing
        reaches the controller -- is a generator whose idea of where the arm
        is diverges without bound, and whose next accepted command is
        therefore a jump.
        """
        q = self._check_vector(q, "resync")
        self.pos = list(q)
        self.vel = [0.0] * DOF
        self.acc = [0.0] * DOF
        self.resyncs += 1
        # AND THROW THE TRAJECTORY AWAY. Ruckig rides its own profile and
        # `update()` reuses it whenever the input has not changed, so a resync
        # that moved only `self.pos` would be overwritten by the old profile
        # on the very next cycle and the deliberate stop would not happen.
        # Rebuilt on the next step, from the new state.
        self._ruckig = None
        self._last_target = None

    def divergence(self, measured):
        """Worst per-joint |internal - measured|, shortest-way for the rolls."""
        return max(abs(d) for d in
                   shortest_delta(self.pos, measured, self.continuous_idx))

    def _check_vector(self, q, what):
        if q is None or len(q) != DOF:
            raise MotionError("%s: expected %d joint values, got %r"
                              % (what, DOF, None if q is None else len(q)))
        out = []
        for i, v in enumerate(q):
            v = float(v)
            if not math.isfinite(v):
                raise MotionError("%s: joint_%d is %r. A non-finite target is "
                                  "refused, not clamped." % (what, i + 1, v))
            out.append(v)
        return out

    # -------------------------------------------------------------- step

    def step(self, target, dt, target_velocity=None):
        """Advance one control cycle of length `dt` toward `target`.

        `target` is in the SAME convention `/compute_ik` returns: continuous
        joints may be anywhere, and the shortest way round is taken from the
        generator's own state.

        `target_velocity` is the streaming case Ruckig exists for: a teleop
        target is not a point the arm should come to rest on, it is a point
        moving at some rate. Default None means "come to rest there", which is
        the safe reading when the operator stops.
        """
        target = self._check_vector(target, "step target")
        if not (dt > 0.0) or not math.isfinite(dt):
            raise MotionError("step: dt %r must be a positive number of "
                              "seconds" % (dt,))
        # Fold the target to the shortest way round FROM THE INTERNAL STATE,
        # then work entirely in unwrapped space.
        #
        # AND CACHE IT WHILE THE TARGET IS UNCHANGED, which is not an
        # optimisation. `pos + wrap_pi(target - pos)` is not bit-identical to
        # `target`, so re-folding every cycle moved the goal by ~1e-17 every
        # cycle; Ruckig compares its input by value, saw a new target, and
        # re-planned on all 36 cycles. That is exactly the per-cycle re-plan
        # `_step_ruckig` documents as destroying phase synchronisation, and it
        # cost 0.0264 rad of path error while every other check passed.
        if self._last_target is not None and target == self._last_target:
            goal = self._goal
        else:
            delta = shortest_delta(target, self.pos, self.continuous_idx)
            goal = [p + d for p, d in zip(self.pos, delta)]
            self._last_target = list(target)
            self._goal = goal
        tvel = ([0.0] * DOF if target_velocity is None
                else self._check_vector(target_velocity, "target_velocity"))
        before = list(self.pos)
        if self.backend == "ruckig":
            sync, eta = self._step_ruckig(goal, tvel, dt)
        else:
            sync, eta = self._step_clamp(goal, tvel, dt)
        travel = max(abs(a - b) for a, b in zip(self.pos, before))
        remaining = max(abs(d) for d in
                        shortest_delta(target, self.pos, self.continuous_idx))
        worst = max(range(DOF),
                    key=lambda i: abs(self.vel[i]) / self.limits.velocity[i])
        return Step(position=_wrap_out(self.pos, self.continuous_idx),
                    position_unwrapped=list(self.pos),
                    velocity=list(self.vel), acceleration=list(self.acc),
                    travel_rad=travel, remaining_rad=remaining, eta_s=eta,
                    complete=remaining <= 1e-9, backend=self.backend,
                    synchronisation=sync, reason=self._reason,
                    limited_by="%s_joint_%d" % (self.arm, worst + 1))

    def _step_ruckig(self, goal, tvel, dt):
        """One cycle, riding Ruckig's OWN trajectory.

        THIS USES `update()`, NOT `calculate()` PER CYCLE, AND THE DIFFERENCE
        IS THE WHOLE POINT. `update()` recomputes only when the input actually
        changed, and otherwise samples the trajectory it already has. Calling
        `calculate()` every cycle re-plans from a mid-profile state, and a
        re-plan cannot reproduce the phase-synchronised profile it is halfway
        through: measured on the self-test's own case, per-cycle re-planning
        left the commanded path **0.0264 rad** off the straight joint-space
        line -- the joints' fractions of travel spread to 0.07 at cycle 10 and
        re-converged -- against **8.9e-17 rad** riding one trajectory. The
        first version of this file did it the wrong way and check 10b caught
        it, which is why that check compares against the line and not against
        the endpoints.

        A target that genuinely moves DOES force a re-plan, and a re-plan from
        a state whose velocity no longer points at the new target cannot be
        phase-synchronised. That is honest and unavoidable; it is reported
        rather than hidden, and every joint still ARRIVES together, because
        time synchronisation is what Ruckig falls back to.
        """
        from ruckig import (InputParameter, OutputParameter, Result, Ruckig,
                            Synchronization)
        if self._ruckig is None:
            self._ruckig = Ruckig(DOF, dt)
            self._inp = InputParameter(DOF)
            self._out = OutputParameter(DOF)
            self._inp.current_position = list(self.pos)
            self._inp.current_velocity = list(self.vel)
            self._inp.current_acceleration = list(self.acc)
            self._inp.synchronization = Synchronization.Phase
            self._sync = "phase"
        inp, out = self._inp, self._out
        self._ruckig.delta_time = dt
        inp.target_position = list(goal)
        inp.target_velocity = list(tvel)
        inp.target_acceleration = [0.0] * DOF
        inp.max_velocity = list(self.limits.velocity)
        inp.max_acceleration = list(self.limits.acceleration)
        inp.max_jerk = list(self.limits.jerk)
        res = self._ruckig.update(inp, out)
        if res not in (Result.Working, Result.Finished):
            # REFUSE. Do not hold silently: an input Ruckig will not accept is
            # a bug upstream, and swallowing it turns a bad IK solution into an
            # arm that has mysteriously stopped.
            raise MotionError(
                "ruckig refused the input: %s. current=%s target=%s "
                "limits: %s" % (res, [round(v, 4) for v in self.pos],
                                [round(v, 4) for v in goal],
                                self.limits.describe()))
        if out.new_calculation:
            self._sync = self._synchronisation(out.trajectory)
            self.replans += 1
        self.pos = list(out.new_position)
        self.vel = list(out.new_velocity)
        self.acc = list(out.new_acceleration)
        eta = max(0.0, out.trajectory.duration - out.time)
        out.pass_to_input(inp)
        if self._sync == "phase":
            self.phase_steps += 1
        else:
            self.time_steps += 1
        return self._sync, eta

    def _synchronisation(self, traj):
        """Which synchronisation Ruckig ACHIEVED, measured, not requested.

        Phase synchronisation means every DoF sits at the same fraction of its
        own travel at every instant, so it is checkable: sample the middle of
        the trajectory and compare the fractions. Reporting the REQUESTED
        setting instead would be the "checked that a field is STORED, not that
        a consumer READS it" row of docs/ENGINEERING_LOG.md's own table.
        """
        if traj.duration <= 0.0:
            return "phase"
        p0, _, _ = traj.at_time(0.0)
        p1, _, _ = traj.at_time(traj.duration)
        span = [b - a for a, b in zip(p0, p1)]
        big = max(range(DOF), key=lambda i: abs(span[i]))
        if abs(span[big]) < 1e-9:
            return "phase"
        pm, _, _ = traj.at_time(traj.duration * 0.5)
        ref = (pm[big] - p0[big]) / span[big]
        for i in range(DOF):
            if abs(span[i]) < 1e-9:
                continue
            if abs((pm[i] - p0[i]) / span[i] - ref) > 1e-6:
                return "time"
        return "phase"

    def _step_clamp(self, goal, tvel, dt):
        """ONE scale factor for the whole vector. Synchronised by construction.

        This is what `clamp_towards` should have been. The per-joint clamp
        moves a short joint to its target while a long one is still going,
        which is exactly how the path leaves the straight line; scaling the
        WHOLE delta by the most restrictive joint's factor cannot.
        """
        delta = [g - p for g, p in zip(goal, self.pos)]
        scale = 1.0
        for i, d in enumerate(delta):
            step_max = self.limits.velocity[i] * dt
            if abs(d) > step_max:
                scale = min(scale, step_max / abs(d))
        prev = list(self.vel)
        self.pos = [p + d * scale for p, d in zip(self.pos, delta)]
        self.vel = [d * scale / dt for d in delta]
        # THE IMPLIED ACCELERATION, NOT ZERO. Reporting zeros here would have
        # made the fallback pass the continuity check by fabricating the
        # quantity the check reads -- docs/ENGINEERING_LOG.md's "feature present but does
        # nothing" row, inside a test. This backend is genuinely not
        # jerk-limited and the number now says so: from rest it steps straight
        # to the velocity limit, which is an acceleration of v_max/dt.
        self.acc = [(v - q) / dt for v, q in zip(self.vel, prev)]
        eta = 0.0
        if scale < 1.0:
            eta = dt * ((1.0 - scale) / scale)
        return "phase", eta


# ------------------------------------------------------------- offline plan

class Plan:
    """The whole path, for tests, measurement and refusal -- never truncation."""

    def __init__(self, positions, velocities, accelerations, cycles, dt,
                 complete, remaining_rad, backend, synchronisation, reason):
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations
        self.cycles = cycles
        self.dt = dt
        self.complete = complete
        self.remaining_rad = remaining_rad
        self.backend = backend
        self.synchronisation = synchronisation
        self.reason = reason

    def arrival_cycles(self, target, tol=1e-6, continuous_idx=CONTINUOUS_IDX):
        """The cycle each joint FIRST reaches its target on, and stays there.

        The synchronisation test. `clamp_towards` returns a set with more than
        one member; a synchronised generator returns exactly one.

        THE TOLERANCE IS RELATIVE TO EACH JOINT'S OWN TRAVEL, and the first
        version was not. An absolute 1e-6 rad reported a phase-synchronised
        move as arriving on cycles [36, 36, 35, 36, 35, 36, 35]: every joint
        was at the same FRACTION of its travel throughout, but the ones moving
        0.02 rad were inside 1e-6 of the end a cycle before the one moving
        0.50 rad. That is the instrument's scale, not the robot's behaviour --
        an absolute gate on a per-joint quantity that spans 25x. It does not
        loosen the control: the per-joint clamp lands on its targets EXACTLY,
        so its arrival cycles are unchanged by any tolerance.
        """
        start = self.positions[0]
        out = []
        for j in range(DOF):
            travel = abs(ang_diff(target[j], start[j])
                         if j in continuous_idx else target[j] - start[j])
            gate = max(tol * travel, 1e-12)
            k = None
            for i in range(len(self.positions) - 1, -1, -1):
                if abs(ang_diff(self.positions[i][j], target[j])
                       if j in continuous_idx
                       else self.positions[i][j] - target[j]) > gate:
                    break
                k = i
            out.append(k)
        return out


def plan(generator, target, dt, max_seconds=30.0, target_velocity=None):
    """Run `generator` to `target` offline and report what happened.

    A HORIZON THAT IS REACHED IS A RESULT, NOT A TRUNCATION. `max_seconds` is
    a real bound -- an arm asked to travel further than it can in the horizon
    comes back with `complete=False` and the remaining distance in radians,
    and the caller decides. Returning the truncated path and calling it done
    is the defect this whole module exists to remove, one level up.
    """
    pos = [list(generator.pos)]
    vel = [list(generator.vel)]
    acc = [list(generator.acc)]
    limit = int(math.ceil(max_seconds / dt))
    sync = set()
    step = None
    for _ in range(limit):
        step = generator.step(target, dt, target_velocity)
        pos.append(list(step.position_unwrapped))
        vel.append(list(step.velocity))
        acc.append(list(step.acceleration))
        sync.add(step.synchronisation)
        if step.complete and max(abs(v) for v in step.velocity) < 1e-9:
            break
    remaining = step.remaining_rad if step is not None else 0.0
    return Plan(pos, vel, acc, len(pos) - 1, dt,
                bool(step is not None and step.complete), remaining,
                generator.backend,
                "phase" if sync == {"phase"} else "+".join(sorted(sync)),
                generator._reason)


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):                              # noqa: C901
    """Constructed answers only. No robot, no ROS, no display."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    # 1 -- the limits come from the file, per joint, and differ
    lim = limits_for("left")
    check("velocity limits read per joint from joint_limits.yaml",
          lim.velocity[:4] == [1.3963000000000001] * 4
          and lim.velocity[4:] == [1.2218] * 3,
          "%.4f x4 then %.4f x3" % (lim.velocity[0], lim.velocity[4]))
    check("acceleration is labelled ASSUMED while the file declares none",
          lim.provenance["acceleration"].startswith(ASSUMED),
          lim.provenance["acceleration"])
    check("velocity is NOT labelled assumed",
          lim.provenance["velocity"] == MEASURED)

    # 2 -- a cap can only lower
    capped = lim.scaled_velocity(0.15)
    check("a velocity cap lowers every joint", max(capped.velocity) == 0.15)
    check("a cap above the declared limit widens nothing",
          lim.scaled_velocity(99.0).velocity == lim.velocity)

    # 3 -- a non-positive limit is refused rather than substituted
    for bad in ([0.0] * DOF, [-1.0] * DOF, [float("nan")] * DOF):
        try:
            Limits(bad, [1.0] * DOF, [1.0] * DOF,
                   {"velocity": "x", "acceleration": "x", "jerk": "x"}, "x")
            check("refuses velocity %r" % bad[0], False)
        except MotionError:
            check("refuses a %r velocity limit" % bad[0], True)

    # 4 -- SYNCHRONISATION. Every joint arrives on the same cycle.
    gen = MotionGenerator("left")
    q0 = [0.0] * DOF
    tgt = [0.50, -0.30, 0.10, 0.25, -0.05, 0.15, 0.02]
    gen.resync(q0)
    p = plan(gen, tgt, 0.02)
    arr = p.arrival_cycles(tgt)
    check("all seven joints arrive on the SAME cycle (%s)" % gen.backend,
          len(set(arr)) == 1 and arr[0] is not None, "arrivals %s" % (arr,))
    check("the plan completes", p.complete and p.remaining_rad < 1e-9)

    # 5 -- the same case through the OLD per-joint clamp does NOT.
    #      A check that cannot fail on a deliberately broken input is not a
    #      check, so the broken input is run here.
    old = _clamp_towards_reference(q0, tgt, 0.35)
    check("the per-joint clamp arrives on DIFFERENT cycles (the control)",
          len(set(old)) > 1, "arrivals %s" % (old,))

    # 6 -- VELOCITY. No joint ever exceeds its own limit.
    worst = max(abs(v[j]) / lim.velocity[j]
                for v in p.velocities for j in range(DOF))
    check("no joint exceeds its own velocity limit", worst <= 1.0 + 1e-9,
          "worst %.4f of limit" % worst)

    # 7 -- CONTINUITY. Acceleration changes by at most jerk*dt per cycle.
    if gen.backend == "ruckig":
        worst_da = 0.0
        for k in range(len(p.accelerations) - 1):
            for j in range(DOF):
                da = abs(p.accelerations[k + 1][j] - p.accelerations[k][j])
                worst_da = max(worst_da, da / (lim.jerk[j] * p.dt))
        check("acceleration steps stay inside jerk*dt", worst_da <= 1.0 + 1e-6,
              "worst %.4f of jerk*dt" % worst_da)

    # 8 -- the fallback is SYNCHRONISED and NAMES ITSELF
    fb = MotionGenerator("left", backend="clamp")
    fb.resync(q0)
    pf = plan(fb, tgt, 0.02)
    check("the fallback names itself", fb.backend == "synchronised-clamp")
    arrf = pf.arrival_cycles(tgt)
    check("the fallback is synchronised too",
          len(set(arrf)) == 1 and arrf[0] is not None, "arrivals %s" % (arrf,))
    wf = max(abs(v[j]) / lim.velocity[j]
             for v in pf.velocities for j in range(DOF))
    check("the fallback honours the velocity limits", wf <= 1.0 + 1e-9,
          "worst %.4f of limit" % wf)
    # AND IT IS HONEST ABOUT WHAT IT IS NOT. The first version reported zero
    # acceleration for this backend, which made it pass the continuity check
    # by fabricating the quantity the check reads. It steps straight to the
    # velocity limit from rest; the number must say so.
    wj = max(abs(a[k + 1][j] - a[k][j]) / (lim.jerk[j] * pf.dt)
             for a in [pf.accelerations] for k in range(len(a) - 1)
             for j in range(DOF))
    check("the fallback does NOT claim to be jerk-limited", wj > 10.0,
          "%.1fx jerk*dt -- it is synchronised, not smooth" % wj)

    # 9 -- A TARGET THAT DOES NOT FIT THE HORIZON IS REPORTED, NOT TRUNCATED
    far = MotionGenerator("left")
    far.resync(q0)
    pf2 = plan(far, [3.0] * DOF, 0.02, max_seconds=0.2)
    check("a target past the horizon is INCOMPLETE, not silently done",
          not pf2.complete and pf2.remaining_rad > 1.0,
          "remaining %.3f rad after %.2f s" % (pf2.remaining_rad,
                                               pf2.cycles * pf2.dt))
    far.resync(q0)
    pf3 = plan(far, [3.0] * DOF, 0.02, max_seconds=30.0)
    check("and the same target completes given the time",
          pf3.complete, "%.2f s" % (pf3.cycles * pf3.dt))

    # 10 -- CONTINUOUS JOINTS take the short way round and come back wrapped.
    #       Run to completion: one cycle from rest moves 0.1 mrad and would
    #       have crossed nothing, which is a check that cannot fail.
    seam = MotionGenerator("left")
    seam.resync([3.10, 0, 0, 0, 0, 0, 0])
    tgt10 = [-3.10, 0, 0, 0, 0, 0, 0]
    p10 = plan(seam, tgt10, 0.02)
    end_raw = p10.positions[-1][0]
    check("a continuous joint crosses the seam the SHORT way (0.083 rad, "
          "not 6.20)", abs(end_raw - (3.10 + (TWO_PI - 6.20))) < 1e-6,
          "travelled %.4f rad to unwrapped %.4f" % (end_raw - 3.10, end_raw))
    check("and the published copy is wrapped back inside +/-pi",
          abs(wrap_pi(end_raw) - (-3.10)) < 1e-6,
          "published %.4f" % wrap_pi(end_raw))
    check("nothing else moved", max(abs(v) for v in p10.positions[-1][1:])
          < 1e-9)

    # 10b -- THE PATH IS THE STRAIGHT LINE. This is the defect, stated in
    #        joint space so the module can check it without kinematics:
    #        how far does the commanded joint vector stray from the segment
    #        q0 -> target. Phase synchronisation makes it zero; the per-joint
    #        clamp is what puts a curve there.
    def _off_line(q0v, qtv, samples):
        d = [b - a for a, b in zip(q0v, qtv)]
        dd = sum(x * x for x in d) or 1.0
        worst = 0.0
        for q in samples:
            r = [x - a for x, a in zip(q, q0v)]
            s_ = sum(x * y for x, y in zip(r, d)) / dd
            worst = max(worst, math.sqrt(sum((x - s_ * y) ** 2
                                             for x, y in zip(r, d))))
        return worst
    gen2 = MotionGenerator("left")
    gen2.resync(q0)
    p2 = plan(gen2, tgt, 0.02)
    off = _off_line(q0, tgt, p2.positions)
    check("the commanded path stays ON the straight joint-space line",
          off < 1e-6, "worst %.3e rad off it" % off)
    cur, csamples = list(q0), [list(q0)]
    for _ in range(50):
        for i in range(DOF):
            dd_ = max(-0.35, min(0.35, tgt[i] - cur[i]))
            cur[i] += dd_
        csamples.append(list(cur))
    coff = _off_line(q0, tgt, csamples)
    check("and the per-joint clamp does NOT (the control)", coff > 0.05,
          "worst %.4f rad off it" % coff)

    # 11 -- refusals, by name
    g = MotionGenerator("left")
    g.resync(q0)
    for bad, what in (([0.0] * 6, "wrong length"),
                      ([float("nan")] * DOF, "not finite")):
        try:
            g.step(bad, 0.02)
            check("refuses a %s target" % what, False)
        except MotionError:
            check("refuses a %s target" % what, True)
    for bad_dt in (0.0, -0.01, float("inf")):
        try:
            g.step(tgt, bad_dt)
            check("refuses dt=%r" % bad_dt, False)
        except MotionError:
            check("refuses dt=%r" % bad_dt, True)
    try:
        MotionGenerator("left", backend="nonsense")
        check("refuses an unknown backend", False)
    except MotionError:
        check("refuses an unknown backend", True)

    # 12 -- resync is the deliberate discontinuity and it stops the profile
    g.resync(q0)
    g.step(tgt, 0.02)
    check("stepping builds velocity", max(abs(v) for v in g.vel) > 0)
    g.resync(q0)
    check("resync stops it", max(abs(v) for v in g.vel) == 0.0
          and g.pos == q0)

    if verbose:
        print("motion_generator self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


def _clamp_towards_reference(start, target, max_step):
    """The OLD per-joint clamp, reimplemented here for the control only.

    `ik_follower_node.clamp_towards` is the real one and stays importable, but
    importing it here would drag rclpy into a module whose whole promise is
    that it has no ROS. This is eight lines and the two are pinned to each
    other by `test_motion_generator.test_the_control_matches_the_real_clamp`.
    """
    cur = list(start)
    arrival = [None] * DOF
    for k in range(1, 10000):
        for i in range(DOF):
            d = (ang_diff(target[i], cur[i]) if i in CONTINUOUS_IDX
                 else target[i] - cur[i])
            d = max(-max_step, min(max_step, d))
            cur[i] = cur[i] + d
            if arrival[i] is None and abs(target[i] - cur[i]) < 1e-12:
                arrival[i] = k
        if all(a is not None for a in arrival):
            return arrival
    return arrival


if __name__ == "__main__":
    import sys
    if "--limits" in sys.argv:
        for arm in ("left", "right"):
            print("%-6s %s" % (arm, limits_for(arm).describe()))
        print("\nbackend: %s" % ("ruckig" if have_ruckig()
                                 else "synchronised-clamp (ruckig absent)"))
        sys.exit(0)
    sys.exit(0 if self_test() else 1)
