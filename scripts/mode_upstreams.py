#!/usr/bin/env python3
"""WHICH UPSTREAM EACH MODE NEEDS, and the isolation that enforces it.

    from mode_upstreams import MODE_ORDER, MODES, isolate

EXTRACTED 2026-08-11, AND THE EXTRACTION IS THE POINT. This lived inside
`record_abc_sweep.py`, so only the CLIP path ever ran it. The DATA path --
`run_abc.py`, which is what a study session drives -- had no equivalent, and
the consequence was measured rather than imagined:

    02_vr_teleop, data run, EE travel 0.0000 m on BOTH arms, exit FAIL

because `vr_pose_mapper` is a mode upstream that nobody had started. The clip
of that mode is fine; a trial of it records a stationary arm. Two paths that
drive the same five modes cannot each have their own idea of what a mode
needs, so there is now exactly one.

THE RULE IT ENFORCES, unchanged: after teardown and startup, the follower's
input topic must be published by NOTHING that this mode does not own. Not
"check it is up" -- count the publishers BY NODE NAME and compare against a
set written down in advance. A wrong set aborts the mode with the names in
the message, because a warning here produces a directory full of plausible,
worthless clips or a session full of plausible, worthless trials.
"""
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))
from srl_teleop import procscan                              # noqa: E402


def _log(msg):
    print("   %s" % msg, flush=True)


MODE_ORDER = ["06_full_autonomy", "01_master_teleop", "03_shared_autonomy",
              "02_vr_teleop", "04_vr_shared"]

MODES = {
    "01_master_teleop": dict(
        needs=[], follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the runner itself is the only publisher"),
    "02_vr_teleop": dict(
        needs=[("vr_pose_mapper",
                # SCALE 1.0 FOR RECORDING, not the shipped 0.5.
                #
                # vr_pose_mapper defaults to scale 0.5 on purpose -- a VR play
                # space is much larger than the robot's workspace, so an
                # operator's half-metre reach should not demand a half-metre
                # of robot. But the sweep does not feed it an operator: it
                # feeds it the task's own ROBOT-FRAME waypoints on
                # /vr/controller_pose_*, which the mapper then halves.
                #
                # Measured, and it is exactly a half: task A's block was
                # carried 0.308 m instead of 0.594 m and released 0.150 m
                # short in x and 0.115 m high, at 0.189 m from the bin -- the
                # SAME 0.189 m on two separate runs, which is what ruled out
                # lag and pointed at a constant factor. 04_vr_shared, which
                # uses the same VR transport but lets autonomy own the pose,
                # placed at 0 mm -- so the transport was never at fault.
                #
                # This is a property of the harness, not of the robot, and
                # 0.5 remains right for a real operator.
                ["ros2", "run", "srl_vr_teleop", "vr_pose_mapper",
                 "--ros-args", "-p", "scale:=1.0"])],
        follower_topic="/master_arm_pose_%s", expect_pubs=1,
        note="the MAPPER is the only publisher; the runner drives it "
             "upstream on /vr/controller_pose_*"),
    "04_shared_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself"),
    "06_full_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the runner itself, commanded by a spoken instruction"),
    "03_shared_autonomy": dict(
        needs=[], follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        note="the arbiter's topic; the master is present but not commanding"),
    "04_vr_shared": dict(
        needs=[("vr_pose_mapper",
                # SCALE 1.0 FOR RECORDING, not the shipped 0.5.
                #
                # vr_pose_mapper defaults to scale 0.5 on purpose -- a VR play
                # space is much larger than the robot's workspace, so an
                # operator's half-metre reach should not demand a half-metre
                # of robot. But the sweep does not feed it an operator: it
                # feeds it the task's own ROBOT-FRAME waypoints on
                # /vr/controller_pose_*, which the mapper then halves.
                #
                # Measured, and it is exactly a half: task A's block was
                # carried 0.308 m instead of 0.594 m and released 0.150 m
                # short in x and 0.115 m high, at 0.189 m from the bin -- the
                # SAME 0.189 m on two separate runs, which is what ruled out
                # lag and pointed at a constant factor. 04_vr_shared, which
                # uses the same VR transport but lets autonomy own the pose,
                # placed at 0 mm -- so the transport was never at fault.
                #
                # This is a property of the harness, not of the robot, and
                # 0.5 remains right for a real operator.
                ["ros2", "run", "srl_vr_teleop", "vr_pose_mapper",
                 "--ros-args", "-p", "scale:=1.0"])],
        follower_topic="/autonomy/assist_pose_%s", expect_pubs=1,
        vr_present=True,
        note="the VR transport is UP and autonomy owns the pose"),
}

# Every upstream any mode can start. Anything not in a mode's `needs` is torn
# down before that mode runs -- listed explicitly so a new upstream cannot be
# forgotten by omission.
ALL_UPSTREAMS = ["lib/srl_vr_teleop/vr_pose_mapper",
                 "lib/srl_autonomy/autonomy_executive",
                 "lib/srl_vr_teleop/quest_vendor_bridge"]



def _reset_mapper(timeout=12.0):
    """Call vr_pose_mapper's /vr/reset and REQUIRE an answer.

    Via `ros2 service call` rather than an rclpy client because this module is
    imported by two callers with different node situations (the sweep has no
    node at all) and a service client needs one.
    """
    try:
        r = subprocess.run(
            ["ros2", "service", "call", "/vr/reset", "std_srvs/srv/Trigger"],
            capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, ("/vr/reset did not answer in %.0f s -- the mapper is up "
                       "but not serving it" % timeout)
    out = (r.stdout or "") + (r.stderr or "")
    if "success=True" in out.replace(" ", ""):
        return True, "vr_pose_mapper RESET (per-run state cleared)"
    if "Unable to find service" in out or "waiting for service" in out:
        return False, ("/vr/reset not found. An old vr_pose_mapper without the "
                       "reset service is running; restart it")
    return False, "/vr/reset returned: %s" % out.strip().replace("\n", " ")[:200]


def isolate(mode, graph, started, own=()):
    """Make `mode` the only source. Returns (ok, message).

    Tears down every upstream this mode does not need, starts the ones it
    does, then COUNTS publishers on the follower's input topic and compares
    against the number this mode predicted.
    """
    spec = MODES[mode]
    need_pats = {n[0] for n in spec["needs"]}

    killed = []
    for pat in ALL_UPSTREAMS:
        if any(p in pat for p in need_pats):
            continue
        t, _ = procscan.kill_all(pat)
        if t:
            killed.append("%s x%d" % (pat.rsplit("/", 1)[-1], len(t)))
            started.pop(pat, None)
    if killed:
        _log("   torn down: %s" % ", ".join(killed))
        # THE GRAPH CACHE OUTLIVES THE PROCESS. Measured: a publisher SIGKILLed
        # 3 s earlier was still named by get_publishers_info_by_topic, so a
        # 2 s settle refused a mode that was in fact isolated. The refusal is
        # the safe direction -- a stale entry can only cause a false NO, never
        # a false YES -- but it costs a whole mode, so wait long enough for the
        # RMW to reap it rather than accept the flakiness.
        time.sleep(8.0)

    for name, argv in spec["needs"]:
        pat = "lib/srl_vr_teleop/%s" % name if "vr" in name else name
        if procscan.count(pat) == 0:
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            p = subprocess.Popen(argv, env=env, start_new_session=True,
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            started[pat] = p
            _log("   started %s (pid %d)" % (name, p.pid))
            time.sleep(9.0)

    # RESET THE MAPPER, EVERY RUN, WHETHER OR NOT WE JUST STARTED IT.
    #
    # vr_pose_mapper carries per-run state -- latched references, anchors, the
    # rate-limited filter, and the thumbstick's scale. Carried into a second
    # run it cost 02_vr_teleop every grasping task, missing by 88, 115, 156 and
    # 205 mm and GROWING. The clip path worked around it by starting a fresh
    # process per clip; the data path (`run_abc`) starts one per SESSION and
    # walked straight into it. Both call isolate(), so the reset belongs here
    # AND in the node -- the node owns the state, this owns "a run begins now".
    #
    # A freshly started mapper is already clean, so this is a no-op there; it
    # is not skipped in that case, because "we think we just started it" is
    # exactly the assumption that produced the session-lifetime bug.
    if any("vr_pose_mapper" in n for n, _a in spec["needs"]):
        ok_reset, why = _reset_mapper()
        _log("   %s" % why)
        if not ok_reset:
            return False, ("vr_pose_mapper is running but would not reset: %s. "
                           "Refusing rather than recording a run that begins "
                           "from the previous run's anchors." % why)

    # THE COUNT. Before the runner starts, the follower's input topic should
    # carry ONLY this mode's upstream -- 0 for the modes whose upstream IS the
    # runner, 1 for VR where the mapper publishes it.
    # BY NAME. The set of nodes allowed to publish the follower's input while
    # the mode is IDLE -- before the runner starts. `master_pose_node` is
    # tolerated because it is part of every launched stack and, with no Teensy,
    # publishes NOTHING: it dies on PortNotFound and respawns. Tolerating it by
    # name is honest; tolerating it by loosening a count would also tolerate a
    # leftover mapper, which is the exact failure this check exists to catch.
    allowed = {"master_pose_node"} | set(own)
    if spec["needs"]:
        allowed |= {n[0] for n in spec["needs"]}
    bad = []
    for arm in ("left", "right"):
        topic = spec["follower_topic"] % arm
        names = graph.pub_nodes(topic)
        extra = {x for x in names if x not in allowed}
        if extra:
            bad.append("%s is published by %s, which this mode does not own "
                       "(allowed: %s)"
                       % (topic, ", ".join(sorted(extra)),
                          ", ".join(sorted(allowed))))
    if bad:
        return False, "; ".join(bad)
    return True, ("isolated: no unowned publisher on the follower input (%s)"
                  % spec["note"])

def isolate_simple(mode, node, started=None):
    """isolate() for a caller that has a plain rclpy Node rather than the
    sweep's Graph helper.

    The DATA path has a node but no Graph, and rewriting Graph into run_abc
    would be a second implementation of the one thing this module exists to
    stop being duplicated.
    """
    class _G:
        @staticmethod
        def pub_nodes(topic):
            return {n for n, _ns in
                    [(i.node_name, i.node_namespace)
                     for i in node.get_publishers_info_by_topic(topic)]}

    # THE CALLER IS ALLOWED TO BE A PUBLISHER, and this asymmetry is the whole
    # reason isolate_simple exists rather than a bare call.
    #
    # In the CLIP path isolate() runs BEFORE the runner process is started, so
    # the only publishers on the follower's input are leftovers. In the DATA
    # path the runner IS the caller: its node and its publisher already exist
    # by the time it asks, so it saw ITSELF as an unowned publisher and refused
    # every non-VR mode -- "a trial recorded now would be about whichever
    # publisher happened to win", about itself.
    #
    # Allowing the caller BY NAME keeps the check strict: a leftover mapper or
    # a second runner is still caught, because neither is this node.
    return isolate(mode, _G(), {} if started is None else started,
                   own={node.get_name()})
