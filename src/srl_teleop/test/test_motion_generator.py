"""The teleoperation motion generator: synchronised, continuous, and limited.

WHAT WAS WRONG. `ik_follower_node.clamp_towards()` was the whole of motion
generation for teleoperation, and it clamps EACH JOINT INDEPENDENTLY to
`max_step_rad` per control cycle. Three separable faults:

  1  NOT SYNCHRONISED. A joint needing 0.10 rad arrives in one cycle, a joint
     needing 0.50 rad takes two, so the joints sit at different fractions of
     their own travel at every instant and the hand leaves the straight line
     the IK solution implies. Measured on the left arm's shipped home +
     [0.50 -0.30 0.10 0.25 -0.05 0.15 0.02]: **51.3 mm** off the path at the
     shipped 0.35 rad step, against a 30 mm grasp capture gate. It settles at
     68.0 mm as the step shrinks rather than tending to zero, because the
     desynchronisation is proportional.

  2  NOT CONTINUOUS. From rest the first cycle commands a whole `max_step`.
     Its implied acceleration step is **1343x** the jerk-limited one.

  3  IT NEVER READ THE JOINT LIMITS. `max_step_rad` / dt at the shipped
     values is 17.5 rad/s against the 1.3963 and 1.2218 rad/s in
     `src/srl_moveit_config/config/joint_limits.yaml` -- **12.53x**, and the
     limits DIFFER per joint, which nothing anywhere expressed.

WHAT IS CHECKED HERE. Every property is asserted for the generator AND
refuted for `clamp_towards` in the same test, because a check that cannot fail
on a deliberately broken input is not a check -- and `clamp_towards` is the
deliberately broken input, still importable for exactly this reason.

No ROS, no robot, no display. `srl_teleop.motion_generator` is pure.
"""
import ast
import math
import os

import pytest

from srl_teleop.motion_generator import (
    CONTINUOUS_IDX, DOF, MotionError, MotionGenerator, limits_for, plan,
    self_test)

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
NODE = os.path.join(WS, "src/srl_teleop/srl_teleop/ik_follower_node.py")

DT = 0.02                       # ik_follower_node's declared cascade_rate_hz
MAX_STEP = 0.35                 # its shipped max_step_rad
SLEW = [0.50, -0.30, 0.10, 0.25, -0.05, 0.15, 0.02]
ZERO = [0.0] * DOF


def _generated(backend="auto", target=None, dt=DT, start=None, **kw):
    gen = MotionGenerator("left", backend=backend, **kw)
    gen.resync(start or ZERO)
    return gen, plan(gen, target or SLEW, dt)


def _clamped(target=None, max_step=MAX_STEP, start=None):
    """The REAL clamp_towards, driven exactly as the node drives it."""
    from srl_teleop.ik_follower_node import clamp_towards, wrap_continuous
    target = target or SLEW
    cur = list(start or ZERO)
    out = [list(cur)]
    for _ in range(5000):
        cur = clamp_towards(list(target), wrap_continuous(cur), max_step)
        out.append(list(cur))
        if max(abs(a - b) for a, b in zip(cur, target)) < 1e-12:
            break
    return out


def _arrivals(path, target):
    """First cycle each joint is AT its target and stays. Relative gate."""
    out = []
    for j in range(DOF):
        travel = abs(target[j] - path[0][j])
        gate = max(1e-6 * travel, 1e-12)
        k = None
        for i in range(len(path) - 1, -1, -1):
            if abs(path[i][j] - target[j]) > gate:
                break
            k = i
        out.append(k)
    return out


# ------------------------------------------------------------ the self-test

def test_module_self_test_passes():
    assert self_test(verbose=False)


# ---------------------------------------------------------- synchronisation

def test_every_joint_arrives_on_the_same_cycle():
    _, p = _generated()
    arrivals = p.arrival_cycles(SLEW)
    assert None not in arrivals, arrivals
    assert len(set(arrivals)) == 1, (
        "the joints arrive on different cycles: %s" % (arrivals,))


def test_the_old_clamp_does_not_and_that_is_the_control():
    """The same assertion, on the input it is supposed to catch."""
    arrivals = _arrivals(_clamped(), SLEW)
    assert len(set(arrivals)) > 1, (
        "clamp_towards arrived synchronised (%s) -- then this whole test file "
        "is measuring nothing" % (arrivals,))
    assert max(arrivals) == 2 and min(arrivals) == 1


def test_the_commanded_path_stays_on_the_straight_line():
    """Synchronisation, stated as the geometry it buys.

    Every joint at the same fraction of its own travel IS the straight line
    between the endpoints in joint space. Measured in joint space here so the
    test needs no kinematics; `scripts/measure_teleop_motion.py` measures the
    same property in millimetres at the hand.
    """
    def off_line(path):
        d = [t - s for t, s in zip(SLEW, ZERO)]
        dd = sum(x * x for x in d)
        worst = 0.0
        for q in path:
            r = [x - a for x, a in zip(q, ZERO)]
            s = sum(x * y for x, y in zip(r, d)) / dd
            worst = max(worst, math.sqrt(sum((x - s * y) ** 2
                                             for x, y in zip(r, d))))
        return worst

    _, p = _generated()
    assert off_line(p.positions) < 1e-9
    assert off_line(_clamped()) > 0.05      # the control


# -------------------------------------------------------------- continuity

def test_acceleration_never_steps_by_more_than_jerk_times_dt():
    gen, p = _generated()
    if gen.backend != "ruckig":
        pytest.skip("ruckig is not installed in this interpreter")
    lim = limits_for("left")
    worst = 0.0
    for a, b in zip(p.accelerations, p.accelerations[1:]):
        for j in range(DOF):
            worst = max(worst, abs(b[j] - a[j]) / (lim.jerk[j] * p.dt))
    assert worst <= 1.0 + 1e-6, "worst acceleration step %.3fx jerk*dt" % worst


def test_the_old_clamp_is_wildly_discontinuous_and_that_is_the_control():
    """Its implied acceleration, by finite difference on its own positions."""
    lim = limits_for("left")
    path = _clamped()
    vel = [[(b[j] - a[j]) / DT for j in range(DOF)]
           for a, b in zip(path, path[1:])]
    acc = [[(v2[j] - v1[j]) / DT for j in range(DOF)]
           for v1, v2 in zip([ZERO] + vel, vel)]
    worst = max(abs(b[j] - a[j]) / (lim.jerk[j] * DT)
                for a, b in zip([ZERO] + acc, acc) for j in range(DOF))
    assert worst > 100.0, (
        "clamp_towards looked jerk-limited (%.2fx) -- check the control" % worst)


# ------------------------------------------------------------ the real limits

def test_the_velocity_limits_come_from_joint_limits_yaml():
    """ONE SOURCE, and it is the file MoveIt plans against."""
    import yaml
    path = os.path.join(WS, "src/srl_moveit_config/config/joint_limits.yaml")
    doc = yaml.safe_load(open(path))["joint_limits"]
    for arm in ("left", "right"):
        lim = limits_for(arm)
        for i in range(DOF):
            assert lim.velocity[i] == doc["%s_joint_%d" % (arm, i + 1)][
                "max_velocity"]
    # And they are NOT all the same, which is the fact a single max_step could
    # not express.
    assert len(set(limits_for("left").velocity)) == 2


def test_no_joint_ever_exceeds_its_own_velocity_limit():
    lim = limits_for("left")
    for backend in ("auto", "clamp"):
        _, p = _generated(backend=backend)
        worst = max(abs(v[j]) / lim.velocity[j]
                    for v in p.velocities for j in range(DOF))
        assert worst <= 1.0 + 1e-6, "%s peaked at %.3fx" % (backend, worst)


def test_the_old_clamp_exceeds_them_by_twelve_times_and_that_is_the_control():
    lim = limits_for("left")
    path = _clamped()
    worst = max(abs(b[j] - a[j]) / DT / lim.velocity[j]
                for a, b in zip(path, path[1:]) for j in range(DOF))
    assert worst > 10.0, "clamp_towards was inside the limits (%.2fx)" % worst
    # It is exactly max_step/dt against the limit of the joint that actually
    # takes a full step -- joint_1, at 1.3963 rad/s. The joints limited to
    # 1.2218 move less than max_step on this case, so the worst FRACTION is
    # not on the slowest joint, which is the whole reason a single max_step
    # cannot express these limits.
    assert abs(worst - MAX_STEP / DT / lim.velocity[0]) < 0.01


def test_a_velocity_cap_can_only_lower():
    lim = limits_for("left")
    assert max(lim.scaled_velocity(0.15).velocity) == 0.15
    assert lim.scaled_velocity(99.0).velocity == lim.velocity


def test_the_acceleration_limit_is_labelled_assumed():
    """Because joint_limits.yaml declares none, and a limit nobody can trace
    is a limit somebody widens."""
    lim = limits_for("left")
    assert lim.provenance["velocity"] == "joint_limits.yaml"
    assert lim.provenance["acceleration"].startswith("ASSUMED")
    assert lim.provenance["jerk"].startswith("ASSUMED")


def test_a_declared_acceleration_would_be_used_instead(tmp_path):
    """The label follows the file. If the yaml ever gains a real limit it is
    read, and the provenance stops saying ASSUMED -- otherwise the honesty is
    a hardcoded string rather than a fact."""
    import yaml
    src = os.path.join(WS, "src/srl_moveit_config/config/joint_limits.yaml")
    doc = yaml.safe_load(open(src))
    for i in range(1, DOF + 1):
        e = doc["joint_limits"]["left_joint_%d" % i]
        e["has_acceleration_limits"] = True
        e["max_acceleration"] = 2.0
    p = tmp_path / "joint_limits.yaml"
    p.write_text(yaml.safe_dump(doc))
    lim = limits_for("left", path=str(p))
    assert lim.acceleration == [2.0] * DOF
    assert lim.provenance["acceleration"] == "joint_limits.yaml"


# ------------------------------------------------------------- the fallback

def test_the_fallback_names_itself_and_is_synchronised():
    gen, p = _generated(backend="clamp")
    assert gen.backend == "synchronised-clamp"
    arrivals = p.arrival_cycles(SLEW)
    assert None not in arrivals and len(set(arrivals)) == 1, arrivals
    assert "ruckig" in gen._reason or "explicitly" in gen._reason


def test_the_fallback_does_not_claim_to_be_jerk_limited():
    """It reports its REAL implied acceleration, not zeros.

    The first version returned zeros here, which made it pass the continuity
    check by fabricating the quantity the check reads -- CLAUDE.md's "feature
    present but does nothing" row, inside a test.
    """
    lim = limits_for("left")
    _, p = _generated(backend="clamp")
    worst = max(abs(b[j] - a[j]) / (lim.jerk[j] * p.dt)
                for a, b in zip(p.accelerations, p.accelerations[1:])
                for j in range(DOF))
    assert worst > 10.0, "the fallback claimed to be smooth (%.2fx)" % worst


def test_asking_for_ruckig_without_ruckig_is_refused_not_degraded():
    import srl_teleop.motion_generator as mg
    have = mg.have_ruckig
    mg.have_ruckig = lambda: False
    try:
        with pytest.raises(MotionError) as e:
            MotionGenerator("left", backend="ruckig")
        assert "pip install" in str(e.value)
        # and `auto` degrades, loudly
        g = MotionGenerator("left", backend="auto")
        assert g.backend == "synchronised-clamp"
        assert "not importable" in g._reason
    finally:
        mg.have_ruckig = have


# ------------------------------------------------------- horizon and refusal

def test_a_target_past_the_horizon_is_reported_not_truncated():
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    p = plan(gen, [3.0] * DOF, DT, max_seconds=0.2)
    assert not p.complete
    assert p.remaining_rad > 1.0
    assert p.cycles == 10


def test_and_the_same_target_completes_given_the_time():
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    p = plan(gen, [3.0] * DOF, DT, max_seconds=30.0)
    assert p.complete and p.remaining_rad < 1e-9


def test_every_step_reports_how_far_is_left():
    """`clamp_towards` returned a position and no way to tell 'nearly there'
    from 'forty seconds away'."""
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    s = gen.step([3.0] * DOF, DT)
    assert not s.complete and s.remaining_rad > 2.9 and s.eta_s > 1.0


@pytest.mark.parametrize("bad", [None, [0.0] * 6, [float("nan")] * DOF,
                                 [float("inf")] * DOF])
def test_a_bad_target_is_refused_by_name(bad):
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    with pytest.raises(MotionError):
        gen.step(bad, DT)


@pytest.mark.parametrize("dt", [0.0, -0.01, float("inf"), float("nan")])
def test_a_bad_dt_is_refused(dt):
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    with pytest.raises(MotionError):
        gen.step(SLEW, dt)


def test_an_unknown_backend_is_refused_by_name():
    with pytest.raises(MotionError) as e:
        MotionGenerator("left", backend="nonsense")
    assert "clamp_towards" in str(e.value)


# ---------------------------------------------------------- continuous joints

def test_a_continuous_joint_takes_the_short_way_and_comes_back_wrapped():
    gen = MotionGenerator("left")
    gen.resync([3.10] + [0.0] * 6)
    p = plan(gen, [-3.10] + [0.0] * 6, DT)
    end = p.positions[-1][0]
    assert abs((end - 3.10) - (2 * math.pi - 6.20)) < 1e-6, end
    from srl_teleop.motion_generator import wrap_pi
    assert abs(wrap_pi(end) + 3.10) < 1e-6


def test_resync_stops_the_profile_and_throws_the_trajectory_away():
    """A resync that moved only the position would be overwritten by the
    trajectory Ruckig is still riding, and the deliberate stop would not
    happen."""
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    for _ in range(5):
        gen.step(SLEW, DT)
    assert max(abs(v) for v in gen.vel) > 0
    moving = max(abs(v) for v in gen.vel)
    gen.resync(ZERO)
    assert gen.vel == ZERO and gen.acc == ZERO and gen.pos == ZERO
    s = gen.step(SLEW, DT)
    # ONE CYCLE FROM REST IS 0.5 * jerk * dt^2, ANALYTICALLY. Not "small":
    # a threshold picked by eye would have been satisfied by a surviving
    # trajectory too, since after five cycles it is only 0.14 rad/s.
    lim = limits_for("left")
    assert max(abs(v) for v in s.velocity) == pytest.approx(
        0.5 * lim.jerk[0] * DT ** 2, rel=1e-6), (
        "the old trajectory survived the resync")
    assert max(abs(v) for v in s.velocity) < 0.1 * moving


def test_divergence_uses_the_short_way_for_continuous_joints():
    gen = MotionGenerator("left")
    gen.resync([math.pi - 0.01] + [0.0] * 6)
    assert gen.divergence([-math.pi + 0.01] + [0.0] * 6) == pytest.approx(
        0.02, abs=1e-9)


# ------------------------------------------------------------- the streaming case

def test_a_moving_target_still_arrives_synchronised():
    """The case teleoperation actually runs in: the target moves every cycle.

    Phase synchronisation is impossible while the current velocity does not
    point at the new target, so Ruckig drops to TIME synchronisation -- which
    still lands every joint together. The generator reports which it achieved
    rather than which it asked for, and this test requires both that the drop
    happens and that arrival stays synchronised.
    """
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    tgt = list(ZERO)
    for k in range(60):
        tgt = [0.004 * k * v for v in SLEW]
        gen.step(tgt, DT)
    assert gen.time_steps > 0, "no re-plan ever happened; the target was static"
    p = plan(gen, tgt, DT)              # let it settle on the last target
    arrivals = p.arrival_cycles(tgt)
    assert None not in arrivals and len(set(arrivals)) == 1, arrivals
    lim = limits_for("left")
    worst = max(abs(v[j]) / lim.velocity[j]
                for v in p.velocities for j in range(DOF))
    assert worst <= 1.0 + 1e-6


def test_a_nonzero_target_velocity_is_accepted():
    """What a streaming teleop target actually is."""
    gen = MotionGenerator("left")
    gen.resync(ZERO)
    s = gen.step(SLEW, DT, target_velocity=[0.1] * DOF)
    assert s.remaining_rad > 0


# ------------------------------------------------- one source, and the wiring

def test_continuous_idx_has_exactly_one_definition():
    """Two lists of 'which joints can wind up' is two home poses again."""
    hits = []
    for pkg in ("srl_teleop",):
        root = os.path.join(WS, "src", pkg)
        for dirpath, _, names in os.walk(root):
            if "__pycache__" in dirpath or "/test" in dirpath:
                continue
            for n in names:
                if not n.endswith(".py"):
                    continue
                f = os.path.join(dirpath, n)
                tree = ast.parse(open(f).read())
                for node in ast.walk(tree):
                    if (isinstance(node, ast.Assign)
                            and any(getattr(t, "id", None) == "CONTINUOUS_IDX"
                                    for t in node.targets)):
                        hits.append(os.path.relpath(f, WS))
    assert hits == ["src/srl_teleop/srl_teleop/motion_generator.py"], hits
    from srl_teleop.ik_follower_node import CONTINUOUS_IDX as node_copy
    assert node_copy is CONTINUOUS_IDX


def test_the_follower_generates_motion_through_the_generator():
    """And `clamp_towards` is only reachable on the legacy branch.

    The point of the change is that BOTH branches -- a solution inside
    `max_step` and one outside it -- now go through the generator. Before, a
    solution inside max_step was published RAW: a 0.35 rad jump in one 20 ms
    cycle, so the path with 'no guard needed' was the faster one.
    """
    src = open(NODE).read()
    assert "self.gen.step(positions, dt)" in src
    assert src.count("clamp_towards(positions, current_wrapped") == 1
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "on_ik_response")
    calls = [n for n in ast.walk(fn)
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", None) == "clamp_towards"
             or (isinstance(n, ast.Call)
                 and getattr(n.func, "id", None) == "clamp_towards")]
    assert len(calls) == 1, "clamp_towards is called %d times" % len(calls)


def test_every_refusal_after_the_step_resyncs_the_generator():
    """A command that was generated and NOT published must not leave the
    generator believing the arm went there -- it would keep integrating ahead
    of a stationary arm and the next accepted cycle would start from fiction.
    """
    src = open(NODE).read()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "on_ik_response")
    body = fn.body
    # find the statement index of the generator step, then require that every
    # bare `return` after it is immediately preceded by self._refuse()
    lines = src.splitlines()
    step_line = next(i for i, ln in enumerate(lines)
                     if "self.gen.step(positions, dt)" in ln)
    bad = []
    for node in ast.walk(fn):
        if isinstance(node, ast.Return) and node.lineno > step_line + 1:
            prev = [ln.strip() for ln in lines[node.lineno - 4:node.lineno - 1]]
            if not any("_refuse()" in ln or "block(\"motion_generator\""
                       in ln for ln in prev):
                bad.append(node.lineno)
    assert not bad, ("returns after the generator step with no _refuse(): %s"
                     % bad)
    assert len(body) > 0


def test_the_node_declares_the_generator_parameters():
    src = open(NODE).read()
    for p in ("motion_generator", "accel_ramp_s", "jerk_ramp_s",
              "generator_resync_rad"):
        assert 'declare_parameter("%s"' % p in src, p
    assert 'self.declare_parameter("motion_generator", "ruckig")' in src
