"""A preflight that reads only the ROS graph cannot name its own failure.

MEASURED 2026-09-01. The operator ran the capture twice and was told both
times "no /master_arm_raw_* - is the Teensy attached and master_pose_node
running?". The Teensy was attached. master_pose_node had been running for
seven minutes. `ros2 topic list --no-daemon` returned 76 topics while the
ordinary `ros2 topic list` returned ZERO: a wedged daemon was caching an empty
graph and handing it to every client.

Every line of the preflight reads the ROS graph, so every line says NONE in
BOTH cases -- a stack that is down, and a stack that is up behind broken
discovery. They need opposite fixes, and the message named the wrong one.

`stack_processes()` is the independent observation that separates them. It
reads the process table and never touches ROS, which is the entire point: an
instrument that shares a failure mode with the thing it is diagnosing cannot
diagnose it.
"""
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments",
                                "trajectory_capture"))
SRC = open(os.path.join(WS, "src", "srl_experiments", "trajectory_capture",
                        "record_trajectories.py")).read()

import record_trajectories as rt          # noqa: E402


def test_the_probe_does_not_go_through_ros():
    """If it used ROS it would share the failure it exists to detect."""
    i = SRC.index("def stack_processes(")
    body = SRC[i:SRC.index("\ndef ", i + 10)]
    for banned in ("rclpy", "ros2 topic", "ros2 node", "get_topic_names"):
        assert banned not in body, \
            "stack_processes uses %r; it must be independent of the graph" \
            % banned
    assert '"ps"' in body or "'ps'" in body


def test_it_returns_a_mapping_and_never_raises():
    """It runs on a failure path, so it must not add a second failure."""
    got = rt.stack_processes()
    assert isinstance(got, dict)
    for k, v in got.items():
        assert isinstance(v, list) and v, k


def test_the_failure_message_branches_on_it():
    i = SRC.index("def preflight(")
    body = SRC[i:SRC.index("\ndef main(", i)]
    assert "stack_processes()" in body, \
        "the preflight failure path does not consult the process table"
    assert "BUT THE STACK IS RUNNING" in body, \
        "there is no branch for a live stack behind broken discovery"
    assert "No stack process is running either" in body, \
        "there is no branch for a genuinely dead stack"


def test_the_live_branch_names_the_daemon_first():
    """The daemon is the usual cause and the cheapest to rule out, so it must
    be step one. Suggesting the shared-memory sweep first would have the
    operator restart a healthy stack."""
    body = SRC[SRC.index("BUT THE STACK IS RUNNING"):]
    body = body[:body.index("No stack process is running either")]
    assert "ros2 daemon stop" in body
    assert "--no-daemon" in body, \
        "the message does not say how to CONFIRM it is the daemon"
    assert body.index("ros2 daemon stop") < body.index("/dev/shm"), \
        "the shared-memory sweep is suggested before the daemon restart"


def test_it_decides_the_transport_rather_than_listing_it_as_an_option():
    """The commonest cause is DECIDABLE from this process's own environment.

    Measured 2026-09-01: a shell without FASTDDS_BUILTIN_TRANSPORTS saw 3
    topics and 0 messages; the same shell with it saw 92 and 291. That is not
    a candidate to be tried in order, it is a fact the preflight can read.
    Offering it as item 2 of a numbered list is how the same twenty minutes
    was lost three times.
    """
    body = SRC[SRC.index("BUT THE STACK IS RUNNING"):]
    body = body[:body.index("No stack process is running either")]
    assert 'os.environ.get("FASTDDS_BUILTIN_TRANSPORTS")' in body, \
        "the preflight does not read its own transport setting"
    assert "THIS IS THE CAUSE" in body, \
        "there is no branch that states the transport IS the cause"
    # And it must not claim that when the transport is fine.
    assert 'shm != "SHM"' in body, \
        "the diagnosis is not conditional on the value being wrong"
    assert "set correctly" in body, \
        "there is no branch for a correctly-set transport"


def test_it_warns_against_clearing_shm_under_a_running_stack():
    body = SRC[SRC.index("BUT THE STACK IS RUNNING"):]
    body = body[:body.index("No stack process is running either")]
    assert "Never clear it" in body and "orphans" in body, \
        "clearing /dev/shm with the stack up orphans the running stack's " \
        "own segments; the message must say so"
