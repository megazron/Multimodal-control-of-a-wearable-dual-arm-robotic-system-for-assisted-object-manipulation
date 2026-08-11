"""The two silent real-robot failures must not come back.

Both had the same shape: a safety mechanism that runs, reports success, and is
about something other than the real arm. Neither showed up in any check,
because every check was satisfied -- the clearance number was a number and the
halt logged a line.

These are STATIC checks on the source. That is deliberate and it is not
laziness: exercising either path needs hardware, and a test that cannot run
without an arm is a test that never runs. What can be pinned without hardware
is the property that was wrong, and that is what is pinned.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "srl_teleop")


def _read(name):
    with open(os.path.join(SRC, name)) as fh:
        return fh.read()


# ---------------------------------------------------------------- clearance
def test_clearance_uses_the_real_tree_when_the_cascade_is_active():
    """The lookup must be able to reach `real_*`, and must choose by cascade.

    THE BUG: `lookup_transform(part, f"{arm}_{link}")` -- unprefixed, the SIM
    tree -- while the real arm publishes under `real_` (both real launches run
    robot_state_publisher with frame_prefix="real_"). Under the cascade the sim
    leads the real arm by the bridge delay, so the figure described a pose the
    arm had not reached, under a name that reads as a measurement of the arm.
    """
    s = _read("ik_follower_node.py")
    assert "_clearance_from" in s, \
        "the clearance lookup is no longer factored by frame prefix"
    assert 'self._clearance_from("real_" if want_real else "")' in s, \
        "measure_clearance no longer selects the frame tree by cascade state"
    assert "want_real = bool(self.cascade_active)" in s, \
        "the frame tree is no longer chosen from the cascade signal"


def test_clearance_never_falls_back_to_the_sim_tree():
    """With the cascade active and no real TF, the answer is UNKNOWN.

    A quiet fall back to the sim tree is the original bug wearing a different
    hat: it would still report a number, and the number would still be about
    the wrong arm.
    """
    s = _read("ik_follower_node.py")
    body = s.split("def measure_clearance", 1)[1].split("\n    def ", 1)[0]
    # the only lookup in the function body goes through the prefixed helper
    assert "lookup_transform" not in body, \
        "measure_clearance looks up TF directly again -- it must go through " \
        "_clearance_from so the prefix cannot be bypassed"
    assert 'self._clear_src = "none"' in body, \
        "an unavailable real tree no longer marks the source unknown"
    assert "Not falling back to the sim tree" in body, \
        "the refusal to fall back is no longer stated where it happens"


def test_the_clearance_source_reaches_the_status_topic():
    """A clearance number without its source is the defect, not a detail."""
    s = _read("ik_follower_node.py")
    assert '{"sim": 0.0, "real": 1.0}.get(self._clear_src, -1.0)' in s, \
        "/ik_status no longer carries which arm the clearance is about"


# ------------------------------------------------------------------- e-stop
def test_the_bridge_offers_a_driver_level_halt():
    """The high-level bridge is the real path and must have a halt of its own.

    THE BUG: estop_node's halt_real_driver() targets /real/controller_manager
    and the Kortex hardware component -- correct for the ros2_control cascade,
    a complete no-op for the bridge, which has neither. The arm still stopped,
    because the bridge subscribes to /estop_state, but the second layer was
    absent while reading as present.
    """
    s = _read("kortex_highlevel_bridge.py")
    assert "_srv_halt" in s, "the bridge no longer has an emergency halt"
    assert 'create_service(Trigger, "/real/emergency_halt_%s" % self.arm' in s, \
        "the bridge halt is no longer registered, or is no longer PER ARM"
    body = s.split("def _srv_halt", 1)[1].split("\n    def ", 1)[0]
    assert "self.base.Stop()" in body, \
        "the halt no longer stops the arm at the driver"
    assert "_send_speeds_rad([0.0] * NJ)" in body, \
        "the halt no longer zeroes the commanded speeds"
    assert "CloseSession" not in body, \
        "the halt must never close the session: the arm permits exactly one " \
        "and a leaked one makes recovery need a relaunch"


def test_the_halt_is_per_arm_so_two_bridges_cannot_collide():
    """`arm:=both` runs two bridges. An unprefixed name is the collision this
    project has been bitten by five times."""
    s = _read("kortex_highlevel_bridge.py")
    assert '"/real/emergency_halt"' not in s, \
        "an UNPREFIXED /real/emergency_halt is back -- two bridges would " \
        "register the same service name and the halt would reach one arm"


def test_estop_calls_the_bridge_halt_for_both_arms():
    s = _read("estop_node.py")
    assert '"/real/emergency_halt_%s" % a' in s, \
        "estop_node no longer creates a per-arm client for the bridge halt"
    body = s.split("def halt_real_driver", 1)[1].split("\n    def ", 1)[0]
    assert "self._halt_cli" in body, \
        "halt_real_driver no longer contacts the bridge at all -- it would " \
        "again be a layer that does nothing on the path that actually runs"


def test_a_completely_unreachable_halt_is_reported_as_one_fact():
    """Four "UNAVAILABLE" lines are a list. "There is no second layer" is the
    fact a reader needs, and it must be said once and loudly."""
    s = _read("estop_node.py")
    body = s.split("def halt_real_driver", 1)[1].split("\n    def ", 1)[0]
    assert "NO DRIVER-LEVEL HALT WAS REACHABLE" in body, \
        "an entirely unreachable driver-level halt is no longer reported"
    assert re.search(r'all\(\s*"UNAVAILABLE" in d for d in done\s*\)', body), \
        "the all-unavailable condition is gone, so the report can no longer " \
        "distinguish 'some layers missing' from 'no second layer at all'"


def test_the_estop_halt_still_does_not_block():
    """Do not reintroduce wait_for_service here.

    Two wait_for_service(2.0) calls used to sit in this function, so on any
    stack without a /real controller manager an e-stop blocked the whole node
    for 4.0 s before parking the arms. Measured: decision at 0.97 s, trip
    logged at 4.98 s. Detection was never late; the halt was.
    """
    s = _read("estop_node.py")
    body = s.split("def halt_real_driver", 1)[1].split("\n    def ", 1)[0]
    # CODE ONLY. The first version of this check matched the function's own
    # DOCSTRING, which says "do not reintroduce wait_for_service() here" -- so
    # it failed on correct code because the warning against the bug contains
    # the name of the bug. The instrument was the fault, which is this
    # project's most repeated lesson and worth one more line of care.
    code = body.split('"""', 2)[-1]
    assert "wait_for_service" not in code, \
        "wait_for_service is back inside halt_real_driver -- this delays " \
        "every e-stop by the full timeout on every sim and mock stack"
    assert code.count("call_async") >= 2, \
        "the halt no longer sends asynchronously"
