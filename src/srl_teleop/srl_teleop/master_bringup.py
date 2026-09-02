#!/usr/bin/env python3
"""
master_bringup.py -- the master-mannequin -> REAL ARMS sequence, in order.

    python3 -m srl_teleop.master_bringup --self-test
    python3 -m srl_teleop.master_bringup            # run it, printing each step

THE ORDER IS THE WHOLE THING, and it is here rather than in the GUI so that
it can be tested without Qt and read without scrolling through a window.
`master_mannequin_gui.py` calls `STEPS` and does nothing the list does not say.

THIS SEQUENCE IS WHAT ACTUALLY WORKED on 2026-09-01, after an evening of it
not working. Each step below exists because skipping it produced a specific
failure that looked like something else:

  1. STACK        teleop.launch.py with follower:=master and master:=false.
                  `master:=false` because step 2 owns the master node -- two
                  master_pose_nodes fight for /dev/ttyACM0 and both then read
                  corrupt frames.
  2. MASTER       master_pose_node with force_clutch_engaged:=true.
                  THE CLUTCH IS PINNED because this rig's button channels flip
                  TOGETHER: measured in the log, both arms disengaged 2 ms
                  apart, and re-engage then deferred on the quasi-static gate
                  until another spurious flip cancelled it. Pinning takes the
                  buttons out of the clutch path entirely.
  3. REAL         scripts/start_real.sh -- Kortex HIGH-LEVEL session, velocity
                  homing, sim->real bridge. NOT the cyclic ros2_control path:
                  HARD CONSTRAINT 4, and an evening of proof.
  4. SEED         command the SIM to the REAL arms' measured joint positions.
                  THIS IS THE STEP THAT WAS MISSING. The bridge refuses to
                  enable while sim and real are more than enable_gap_rad
                  (0.30) apart, and homing only ever moves the REAL side. The
                  master's resting pose maps ~32 deg from home, so the sim sat
                  there and the gap never closed -- "REFUSED: sim is 0.557 rad
                  from the real arm", repeatedly, with both sides individually
                  where they were supposed to be. Seeding the sim FROM
                  /real/joint_states closes it by construction.
  5. BRIDGE       enable both sim->real bridges. Now the gap is ~0.
  6. ARM          master_teleop motion_enabled:=true, LAST. Arming before
                  step 5 lets the sim run away from the real arm again and
                  step 4 has to be redone.

SPEED. `SPEEDS` below pins one invariant, and it is the one that cost an
e-stop: the COMMANDING side must be slower than the FOLLOWING side.
master_teleop drives the sim through Ruckig at up to its own vmax; the bridge
replays to the real arm at MAX_VEL. When Ruckig was left at the joint limit
(1.3963 rad/s) against a 0.40 rad/s bridge, the real arm fell behind
monotonically and the lag monitor tripped at 0.502 rad on joint_5. So
`teleop_vmax` is always strictly under `bridge_vmax`, and both are under the
joint limit.
"""
import argparse
import math
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))

ARMS = ("left", "right")
DOF = 7

#: The joint velocity limit from joint_limits.yaml, for reference. Nothing
#: here may exceed it and nothing here comes close.
JOINT_LIMIT_RAD_S = 1.3963

#: Named speed presets. `teleop_vmax` < `bridge_vmax` is not a preference, it
#: is the condition under which the lag monitor is a safety net rather than a
#: scheduled failure -- see the module docstring.
#:
#: `slew_m` is the Cartesian rate limit inside master_teleop, in metres per
#: master sample at 50 Hz. 0.006 was the measured smoothness knee on
#: capture_20260901_151026; the sweep showed 8-20 mm cost NO extra tracking
#: lag (p95 stayed 3.65-3.71 mm) and only raised the worst single step, so
#: going faster here is cheap and it is what "the arms feel slow" is asking
#: for.
SPEEDS = {
    "slow":   dict(teleop_vmax=0.25, bridge_vmax=0.40, slew_m=0.006),
    "normal": dict(teleop_vmax=0.35, bridge_vmax=0.60, slew_m=0.010),
    "fast":   dict(teleop_vmax=0.60, bridge_vmax=0.90, slew_m=0.016),
}
DEFAULT_SPEED = "normal"


def speed_profile(name):
    """Look up a preset, and REFUSE an unknown name rather than defaulting.

    A typo silently selecting a different speed on a real arm is exactly the
    class of fault this repo keeps paying for.
    """
    if name not in SPEEDS:
        raise ValueError(
            "unknown speed %r -- expected one of %s"
            % (name, ", ".join(sorted(SPEEDS))))
    p = dict(SPEEDS[name])
    check_speeds(p)
    return p


def check_speeds(p):
    """The invariant, checked rather than trusted. Returns None or raises."""
    if not (p["teleop_vmax"] < p["bridge_vmax"]):
        raise ValueError(
            "teleop_vmax %.3f must be STRICTLY under bridge_vmax %.3f: the "
            "commanding side outrunning the following side is what trips the "
            "lag monitor (measured 0.502 rad on joint_5, 2026-09-01)"
            % (p["teleop_vmax"], p["bridge_vmax"]))
    if not (p["bridge_vmax"] < JOINT_LIMIT_RAD_S):
        raise ValueError(
            "bridge_vmax %.3f is at or over the %.4f rad/s joint limit"
            % (p["bridge_vmax"], JOINT_LIMIT_RAD_S))
    if not (0.0 < p["slew_m"] <= 0.05):
        raise ValueError(
            "slew_m %.4f m/sample is outside 0 < s <= 0.05; at 50 Hz that "
            "would be over 2.5 m/s of commanded hand speed" % p["slew_m"])


# ---------------------------------------------------------------- the steps
#: (key, human title, why it is here). The GUI renders this list; the runner
#: below executes it. One list, so a step cannot be shown and not run.
STEPS = [
    ("stack",  "simulation stack",
     "teleop.launch.py follower:=master master:=false"),
    ("master", "master arm",
     "master_pose_node, clutch PINNED (this rig's buttons flip together)"),
    ("real",   "real arms",
     "start_real.sh -- Kortex high-level session, homing, bridge"),
    ("seed",   "seed sim from real",
     "the step whose absence made the bridge refuse for an hour"),
    ("bridge", "enable sim->real",
     "both bridges; the gap is ~0 once seeded"),
    ("arm",    "arm the teleop",
     "motion_enabled:=true, LAST"),
]


def _env():
    e = dict(os.environ)
    e.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
    return e


def spawn(cmd, log_path, env=None):
    """Start a long-lived process detached, with its output on disk."""
    fh = open(log_path, "w")
    return subprocess.Popen(cmd, cwd=WS, stdout=fh, stderr=subprocess.STDOUT,
                            env=env or _env(), start_new_session=True)


def wait_for(predicate, timeout_s, poll_s=0.25):
    """True as soon as `predicate()` is true, False on timeout."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        try:
            if predicate():
                return True
        except Exception:                                    # noqa: BLE001
            pass
        time.sleep(poll_s)
    return False


def log_says(path, needle):
    """Is `needle` in the file yet? Missing file is simply 'not yet'."""
    try:
        with open(path, "r", errors="replace") as fh:
            return needle in fh.read()
    except OSError:
        return False


# ------------------------------------------------------------- ROS helpers
# Imported lazily so `--self-test` runs with no ROS on the path.

def _ros():
    import rclpy
    from rclpy.node import Node
    return rclpy, Node


def set_bool_param(node_name, param, value, timeout_s=10.0):
    """Set one bool parameter through the node's OWN service.

    NOT `ros2 param set`: that spawns a CLI process with its own discovery,
    and on this host it hangs for its full timeout often enough that the
    2026-09-01 session lost an hour to it.
    """
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_param")
    try:
        cli = n.create_client(SetParameters, "%s/set_parameters" % node_name)
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, "no %s/set_parameters" % node_name
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
        req = SetParameters.Request()
        req.parameters = [Parameter(name=param, value=pv)]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
        res = fut.result()
        if res is None:
            return False, "%s did not answer" % node_name
        ok = bool(res.results and res.results[0].successful)
        return ok, ("" if ok else (res.results[0].reason if res.results
                                   else "no result"))
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def seed_sim_from_real(timeout_s=15.0):
    """Command the SIM arms to the REAL arms' measured joint positions.

    THE STEP THAT WAS MISSING. See the module docstring: homing moves only the
    real side, so the bridge's enable gap never closed and it refused with a
    number that looked like a fault when both sides were individually fine.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_seed")
    try:
        real = {}
        n.create_subscription(
            JointState, "/real/joint_states",
            lambda m: real.update(dict(zip(m.name, m.position))), 20)
        pubs = {a: n.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 10)
            for a in ARMS}
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(n, timeout_sec=0.1)
            if all("%s_joint_1" % a in real for a in ARMS):
                break
        sent = []
        for a in ARMS:
            names = ["%s_joint_%d" % (a, i) for i in range(1, DOF + 1)]
            pos = [real.get(nm) for nm in names]
            if any(p is None for p in pos):
                continue
            t = JointTrajectory()
            t.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start = Duration(sec=4, nanosec=0)
            t.points = [pt]
            pubs[a].publish(t)
            sent.append(a)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if not sent:
            return False, "no /real/joint_states -- are the real arms up?"
        return True, "seeded %s from the real arms" % ", ".join(sent)
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def enable_bridges(timeout_s=12.0):
    """Enable both sim->real bridges. Reports each arm's own answer."""
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_bridge")
    try:
        out, all_ok = [], True
        for a in ARMS:
            cli = n.create_client(Trigger, "/bridge_enable_%s" % a)
            if not cli.wait_for_service(timeout_sec=timeout_s):
                out.append("%s: no /bridge_enable_%s" % (a, a))
                all_ok = False
                continue
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
            r = fut.result()
            if r is None:
                out.append("%s: no answer" % a)
                all_ok = False
            else:
                out.append("%s: %s" % (a, r.message.strip()[:120]))
                all_ok = all_ok and bool(r.success)
        return all_ok, " | ".join(out)
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


def reset_estop(timeout_s=10.0):
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("master_bringup_estop")
    try:
        cli = n.create_client(Trigger, "/estop_reset")
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, "no /estop_reset"
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout_s)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "no answer")
    finally:
        n.destroy_node()
        if own and rclpy.ok():
            rclpy.shutdown()


# ===========================================================================
#  SELF-TEST. Pure logic only -- no ROS, no hardware, no Qt. Every check is
#  one that can fail: the speed invariant is asserted against deliberately
#  broken profiles, not just the shipped ones.
# ===========================================================================
def self_test(verbose=True):
    fails = []

    def check(name, cond, detail=""):
        if verbose:
            print("  %-56s %s" % (name, "ok" if cond else "FAIL"))
            if not cond and detail:
                print("      %s" % detail)
        if not cond:
            fails.append(name)

    print("speed profiles")
    for name in sorted(SPEEDS):
        p = SPEEDS[name]
        check("%-6s commanding side is slower than following side" % name,
              p["teleop_vmax"] < p["bridge_vmax"],
              "teleop %.2f vs bridge %.2f" % (p["teleop_vmax"],
                                              p["bridge_vmax"]))
        check("%-6s stays under the %.3f rad/s joint limit"
              % (name, JOINT_LIMIT_RAD_S),
              p["bridge_vmax"] < JOINT_LIMIT_RAD_S)
    check("'fast' is actually faster than 'slow'",
          SPEEDS["fast"]["teleop_vmax"] > SPEEDS["slow"]["teleop_vmax"]
          and SPEEDS["fast"]["slew_m"] > SPEEDS["slow"]["slew_m"])
    check("the default profile exists", DEFAULT_SPEED in SPEEDS)

    # THE INVARIANT MUST BITE. A checker that cannot fail is not a checker.
    try:
        check_speeds(dict(teleop_vmax=0.9, bridge_vmax=0.4, slew_m=0.01))
        check("an inverted profile is REFUSED", False,
              "check_speeds accepted teleop 0.9 > bridge 0.4")
    except ValueError:
        check("an inverted profile is REFUSED", True)
    try:
        check_speeds(dict(teleop_vmax=0.5, bridge_vmax=2.0, slew_m=0.01))
        check("a profile over the joint limit is REFUSED", False)
    except ValueError:
        check("a profile over the joint limit is REFUSED", True)
    try:
        check_speeds(dict(teleop_vmax=0.2, bridge_vmax=0.4, slew_m=0.5))
        check("an absurd slew is REFUSED", False)
    except ValueError:
        check("an absurd slew is REFUSED", True)
    try:
        speed_profile("quick")
        check("an unknown speed name is REFUSED, not defaulted", False)
    except ValueError:
        check("an unknown speed name is REFUSED, not defaulted", True)

    print("step list")
    keys = [k for k, _, _ in STEPS]
    check("every step has a unique key", len(keys) == len(set(keys)))
    check("the sim is seeded BEFORE the bridge is enabled",
          keys.index("seed") < keys.index("bridge"),
          "seeding after enabling is the ordering that refused all evening")
    check("the teleop is armed LAST", keys[-1] == "arm")
    check("the master node comes up after the stack",
          keys.index("stack") < keys.index("master"))
    check("the real arms come up before the seed",
          keys.index("real") < keys.index("seed"))

    print("helpers")
    check("log_says on a missing file is 'not yet', not an error",
          log_says("/nonexistent/nope.log", "x") is False)
    check("wait_for returns False on timeout",
          wait_for(lambda: False, 0.3, 0.05) is False)
    check("wait_for returns True as soon as the predicate holds",
          wait_for(lambda: True, 1.0, 0.05) is True)
    check("wait_for survives a predicate that raises",
          wait_for(lambda: (_ for _ in ()).throw(RuntimeError("x")),
                   0.3, 0.05) is False)

    print()
    if fails:
        print("SELF-TEST FAILED: %d" % len(fails))
        for f in fails:
            print("   - %s" % f)
        return 1
    print("SELF-TEST PASSED")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--speed", default=DEFAULT_SPEED, choices=sorted(SPEEDS))
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    p = speed_profile(a.speed)
    print("master mannequin bring-up, speed=%s" % a.speed)
    print("  teleop vmax %.2f rad/s < bridge vmax %.2f rad/s, slew %.0f mm"
          % (p["teleop_vmax"], p["bridge_vmax"], p["slew_m"] * 1000))
    for i, (k, title, why) in enumerate(STEPS, 1):
        print("  %d. %-22s %s" % (i, title, why))
    print()
    print("Run it from the GUI: python3 scripts/master_mannequin_gui.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
