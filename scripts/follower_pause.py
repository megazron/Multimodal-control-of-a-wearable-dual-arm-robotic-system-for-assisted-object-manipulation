#!/usr/bin/env python3
"""PAUSE AND RESUME `ik_follower_node`. The one implementation.

    import follower_pause as FP
    paused = FP.pause(node)          # node is any live rclpy Node
    try:
        ...command a joint-space trajectory and wait for arrival...
    finally:
        FP.resume(node, paused)      # ALWAYS, on every exit path

WHY IT EXISTS AT ALL. `ik_follower_node` streams position commands to
`/<arm>_arm_controller/joint_trajectory`, and anything else that publishes a
trajectory to that controller is a second publisher on it. Once the follower
has a target it holds the arm there, so the other trajectory is overridden.
This is the project's one-source-at-a-time rule appearing at the CONTROLLER
level rather than at the process level, and it has now cost two different
things: the staging move, and the observe move that T1's vision path depends
on.

WHY IT IS A MODULE AND NOT A METHOD. It was a pair of methods on
`stage_presentation_pose.Stager`. The observe move needs exactly the same
thing, and the alternative was a second copy of a routine whose FAILURE MODES
ARE NOT SYMMETRIC: a pause that does not happen costs one badly framed clip,
and a restore that does not happen disarms an arm and costs every clip after
it. Two copies of that would drift.

`motion_enabled` is the follower's own arming parameter and it returns early
without publishing when it is false, so lowering it is a clean pause rather
than a kill.

THE PREVIOUS VALUE IS RESTORED, NEVER FORCED TRUE. HARD CONSTRAINT 8:
real_robot mode must be ARMED BY HAND, and a helper that armed motion as a
side effect would be exactly the silent re-arm that rule exists to prevent. An
arm in real_robot mode is refused outright and says so.
"""
import time

FOLLOWERS = ("/ik_follower_left", "/ik_follower_right")


def _params(node, target, names, timeout=3.0, wait=5.0):
    from rcl_interfaces.srv import GetParameters
    import rclpy
    cli = node.create_client(GetParameters, target + "/get_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(GetParameters.Request(names=list(names)))
    t = time.time()
    while not fut.done() and time.time() - t < wait:
        rclpy.spin_once(node, timeout_sec=0.02)
    return fut.result()


def _set_bool(node, target, name, value, timeout=3.0, wait=5.0):
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import SetParameters
    import rclpy
    cli = node.create_client(SetParameters, target + "/set_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    p = Parameter(name=name,
                  value=ParameterValue(type=1, bool_value=bool(value)))
    fut = cli.call_async(SetParameters.Request(parameters=[p]))
    t = time.time()
    while not fut.done() and time.time() - t < wait:
        rclpy.spin_once(node, timeout_sec=0.02)
    r = fut.result()
    return bool(r and r.results and r.results[0].successful)


def pause(node, followers=FOLLOWERS):
    """Lower `motion_enabled`, remembering what was actually changed.

    Returns {follower: note}. The note is what the caller prints: a pause that
    happened silently is one nobody can tell from a pause that did not.
    """
    out = {}
    for target in followers:
        res = _params(node, target, ["motion_enabled", "real_robot"])
        if res is None or len(res.values) < 2:
            out[target] = "not reachable; left alone"
            continue
        was, real = res.values[0].bool_value, res.values[1].bool_value
        if real:
            out[target] = "real_robot mode -- REFUSING to touch motion_enabled"
            continue
        if not was:
            out[target] = "motion_enabled already false; left alone"
            continue
        out[target] = ("paused (motion_enabled true -> false)"
                       if _set_bool(node, target, "motion_enabled", False)
                       else "COULD NOT PAUSE -- the follower may win")
    return out


def resume(node, paused, tries=10):
    """Restore exactly what was lowered, and VERIFY IT BY READ-BACK.

    A FAILED RESTORE LEAVES THE ARM DISARMED, which is the one outcome this
    must never produce. It happened once already: a clip logged "paused
    (motion_enabled true -> false), RESTORE FAILED" and every later clip in
    that mode failed, because the follower it had silenced never spoke again.
    The single set_parameters call had been believed on its return value
    alone.

    So: retry, read the value back, retry again if it is still false.
    Restoring is worth more effort than pausing because the failure modes are
    not symmetric.

    Returns True if everything that was lowered came back up.
    """
    ok = True
    for target, note in paused.items():
        if not note.startswith("paused"):
            continue
        good = False
        for _ in range(tries):
            _set_bool(node, target, "motion_enabled", True)
            res = _params(node, target, ["motion_enabled"])
            if res and res.values and res.values[0].bool_value:
                good = True
                break
            time.sleep(0.5)
        paused[target] = note + (", restored" if good else
                                 ", RESTORE FAILED -- THIS ARM IS DISARMED, "
                                 "re-arm it before recording anything else")
        ok = ok and good
    return ok
