"""A run starts from HOME, and until 2026-08-17 the second one did not.

WHAT WAS MEASURED. On a fresh teleop stack, reading /joint_states against
`config/home_positions_{arm}.txt` at the instant the FIRST waypoint appears on
the mode's entry topic:

    at boot                            left 0.0000   right 0.0000 rad
    run 1, first commanded waypoint    left 0.0000   right 0.0000
    between runs (nothing else ran)    left 1.2338   right 0.6248
    run 2, first commanded waypoint    left 1.2338   right 0.6248

The pose is not lost between launch and the first trial -- the URDF, the five
home carriers and the sim spawn were all measured correct. It is lost BETWEEN
trials: nothing returned the arms to home when a run ended, and nothing looked
at where they were when the next one started. `record_abc_sweep.py` staged, so
the recorded clips were fine; every run driven from the GUI or from
run_experiment.sh was not.

Note the right arm. T1 is a left-arm task and it still left the right arm
0.62 rad out, because the runner parks the idle arm and never brings it back.

WHAT THIS TEST CHECKS, and why each part can fail
-------------------------------------------------
`home_error()` is the instrument. The standing rule says an instrument that
cannot fail on a deliberately broken input is not a check, so every case here
feeds it a joint state whose distance from home is CONSTRUCTED -- arithmetic
on the loaded home values, not a rendered pose -- and asserts the number it
must return:

  * an arm exactly on home reads 0.0, and it must read 0.0 rather than falsy-
    nothing (`x or 9.9` has already cost this project a clip);
  * an arm displaced by a known amount reads that amount, and names the joint
    that was displaced -- so the reading cannot be a constant;
  * displacement is measured WRAPPED, so a continuous joint a whole turn away
    is at home and not 6.28 rad from it;
  * a joint state missing an arm returns None -- UNKNOWN -- and never a quiet
    zero, which is the shape of the silent faults this repository keeps
    finding;
  * the gate's tolerance is the bridge's 0.05 rad, so sim and real agree on
    what "at home" means.
"""
import math
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
ABC = os.path.join(ROOT, "src", "srl_experiments", "experiments", "abc")
CFG = os.path.join(ROOT, "config")
for _p in (ABC, CFG):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _run_abc():
    """Import run_abc WITHOUT a ROS graph. It imports rclpy at module level."""
    rclpy = pytest.importorskip("rclpy")
    assert rclpy is not None
    import run_abc
    return run_abc


class FakeNode(object):
    """Just enough of Runner for home_error(): a joint dict and a spin."""

    def __init__(self, js):
        self.js = js
        self.spins = 0

    def spin(self, s):
        self.spins += 1


def _home():
    import home_positions as hp
    return {arm: list(hp.load_home_radians(arm)) for arm in ("left", "right")}


def _js_at_home(offsets=None):
    """A /joint_states dict built from the loaded home, plus known offsets.

    `offsets` is {(arm, joint_index_1_based): radians}. The truth is therefore
    arithmetic on the same file the code reads, so the expected answer is
    constructed rather than measured.
    """
    home = _home()
    js = {}
    for arm, q in home.items():
        for i, v in enumerate(q, start=1):
            js["%s_joint_%d" % (arm, i)] = v + (offsets or {}).get((arm, i), 0.0)
    # the grippers and the wearer are in /joint_states too; include one so the
    # lookup cannot be relying on the dict holding arm joints alone
    js["left_finger_joint"] = 0.0
    return js


def test_exactly_home_reads_zero_not_nothing():
    ra = _run_abc()
    err = ra.home_error(FakeNode(_js_at_home()))
    assert err is not None
    for arm in ("left", "right"):
        worst, joint = err[arm]
        assert worst == pytest.approx(0.0, abs=1e-12), (arm, worst)
        assert joint.startswith(arm)


@pytest.mark.parametrize("arm,joint,delta", [
    ("left", 2, 0.30),
    ("left", 6, -1.2338),        # the value actually measured between runs
    ("right", 4, 0.6248),        # the idle arm, left parked by the runner
    ("right", 7, 0.07),
])
def test_a_known_displacement_reads_back_as_itself(arm, joint, delta):
    """The instrument must return the number that was put in, on the right
    joint. A reading that is the same whatever is displaced is the
    'every pose returns one value' row of CLAUDE.md's failure table."""
    ra = _run_abc()
    js = _js_at_home({(arm, joint): delta})
    err = ra.home_error(FakeNode(js))
    assert err is not None
    worst, name = err[arm]
    assert worst == pytest.approx(abs(delta), abs=1e-9)
    assert name == "%s_joint_%d" % (arm, joint)
    # the OTHER arm was not touched and must still read zero, so the two arms
    # are not being conflated
    other = "right" if arm == "left" else "left"
    assert err[other][0] == pytest.approx(0.0, abs=1e-12)


def test_displacement_is_measured_wrapped():
    """joint_7 is continuous. A whole turn is the same physical pose, and the
    right arm's home value is stored WRAPPED precisely because of this."""
    ra = _run_abc()
    err = ra.home_error(FakeNode(_js_at_home({("right", 7): 2 * math.pi})))
    assert err is not None
    assert err["right"][0] == pytest.approx(0.0, abs=1e-9)


def test_a_missing_arm_is_unknown_and_not_zero():
    ra = _run_abc()
    js = _js_at_home()
    for i in range(1, 8):
        js.pop("right_joint_%d" % i)
    assert ra.home_error(FakeNode(js)) is None, (
        "a joint state missing an arm must read UNKNOWN. Returning 0.0 rad "
        "would report an arm nobody can see as being exactly at home.")


def test_the_gate_agrees_with_the_bridge_on_what_at_home_means():
    """`sim_to_real_bridge.enable()` refuses on 0.05 rad. If the runner used a
    looser number, a pose the bridge calls 'not homed' would start a trial."""
    ra = _run_abc()
    assert ra.HOME_TOL_RAD == pytest.approx(0.05)


def test_require_home_is_a_no_op_when_already_home():
    """The recording sweep stages before it launches the runner, so the gate
    must cost it nothing -- no subprocess, no publisher, no move."""
    ra = _run_abc()
    calls = []
    import subprocess
    real = subprocess.run
    subprocess.run = lambda *a, **k: calls.append(a) or real(["true"])
    try:
        ok, why = ra.require_home(FakeNode(_js_at_home()))
    finally:
        subprocess.run = real
    assert ok is True
    assert why == "already home"
    assert calls == [], "staging was invoked on an arm that was already home"


def test_require_home_refuses_when_the_state_is_unknown():
    """No joint state is not evidence that the arm is at home."""
    ra = _run_abc()
    ok, why = ra.require_home(FakeNode({}))
    assert ok is False
    assert "UNKNOWN" in why
    # ...and the override is what turns that into a run, deliberately
    ok2, _ = ra.require_home(FakeNode({}), allow_unhomed=True)
    assert ok2 is True


def test_the_runner_offers_the_override_and_defaults_it_off():
    """--allow-unhomed must exist (so a deliberate off-home run is possible)
    and must be OFF by default (so an accidental one is not)."""
    src = open(os.path.join(ABC, "run_abc.py")).read()
    assert '"--allow-unhomed"' in src
    assert 'ap.add_argument("--allow-unhomed", action="store_true"' in src
    # and the gate runs BEFORE isolate(), because vr_pose_mapper holds the arms
    assert src.index("require_home(n") < src.index("if a.isolate:"), (
        "the home gate must run before --isolate starts vr_pose_mapper, "
        "which holds the arms against a joint-space trajectory")


# --------------------------------------------------------------------------
# THE SESSION MANAGER'S HOME GATE COULD NEVER RUN, AND SAID SO IN THE ONE WAY
# NOBODY LOOKS AT.
#
# `session_manager._js()` read `from srl_experiments import home_positions`.
# There is no such module -- the home SOURCE is config/home_positions_{arm}.txt
# and it is loaded by path, not as a package. So every /joint_states callback
# raised ImportError, the bare except set `home_err = None`, and the readiness
# panel reported "home reference not loaded" forever. It never went green on a
# wrong answer, which is why it survived: UNKNOWN is a legitimate reading for
# a gate with no data yet, and this gate had no data ever.
# --------------------------------------------------------------------------

def test_session_manager_loads_the_home_source_by_path():
    src = open(os.path.join(ROOT, "src", "srl_experiments", "srl_experiments",
                            "session_manager.py")).read()
    assert "from srl_experiments import home_positions" not in src, (
        "srl_experiments has no home_positions module; the source is "
        "config/home_positions_{arm}.txt and must be loaded by path")
    assert "import home_positions as hp" in src


def test_session_manager_home_check_produces_a_number():
    """Run the callback's arithmetic against a constructed joint state and
    assert it returns the displacement that was put in. Without this the fix
    above is 'the import no longer raises', which is not the same as 'the
    gate now measures something'."""
    import types

    class _M(object):
        pass

    home = _home()
    m = _M()
    m.name, m.position = [], []
    for arm, q in home.items():
        for i, v in enumerate(q, start=1):
            m.name.append("%s_joint_%d" % (arm, i))
            m.position.append(v)
    # displace one joint by a known amount
    m.position[m.name.index("left_joint_3")] += 0.25

    sys.path.insert(0, os.path.join(ROOT, "src", "srl_experiments"))
    sm = pytest.importorskip("srl_experiments.session_manager")
    node = types.SimpleNamespace(js_t=None, home_err="untouched")
    sm.SessionManager._js(node, m)
    assert node.home_err is not None, (
        "the home gate returned UNKNOWN on a joint state it can read")
    assert node.home_err == pytest.approx(0.25, abs=1e-9)
