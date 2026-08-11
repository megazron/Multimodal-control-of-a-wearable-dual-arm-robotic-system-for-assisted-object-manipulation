#!/usr/bin/env python3
"""Launch the sim, WAIT ON AN OBSERVABLE CONDITION, run a command, tear down.

    python3 scripts/sim_session.py -- python3 scripts/verify_msc_tasks.py ...

WHY THIS EXISTS
---------------
The previous attempt chained "launch the stack" and "run the verification"
across two shell invocations, with the second blocking on

    until grep -q "READY" <the first command's task output file>

That marker never appeared -- the launching command had itself been moved to
the background and its stdout went somewhere the grep was not looking -- so
the verification sat waiting for ever and reported nothing. Two things were
wrong with it and both are fixed here:

  1. IT WAITED ON A PROXY, NOT ON THE CONDITION. A word printed by another
     process is evidence about that process, not about the robot. What the
     verification actually needs is /compute_ik answering and /joint_states
     DELIVERING DATA TO A NODE. That distinction is not pedantic: stale
     /dev/shm/fastrtps_* segments have twice produced a graph where
     `ros2 topic list` shows /joint_states and no subscriber ever receives a
     message, which is indistinguishable from a dead robot unless you
     subscribe.
  2. IT HUNG INSTEAD OF FAILING. An `until` loop with no deadline cannot
     report anything. This times out LOUDLY and exits non-zero.

Everything runs in ONE process, so there is no marker to miss.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# EVERY SHELL THIS FILE OPENS MUST SOURCE env.sh, INCLUDING THE PROBE'S.
#
# CLAUDE.md names scripts/env.sh the single source of environment truth: it
# pins ROS_DOMAIN_ID=0, RMW_IMPLEMENTATION=rmw_fastrtps_cpp and
# ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, and it actively unsets any inherited
# ROS_LOCALHOST_ONLY. teleop.launch.py applies the same settings IN-PROCESS,
# so a launched stack always has them.
#
# This SRC used to source setup.bash only, so the readiness probe ran with a
# bare login environment and none of them set. MEASURED, with a healthy stack
# up:
#
#     /proc/<move_group>/environ : ROS_DOMAIN_ID=0
#                                  ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
#                                  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#     bash -lc (the probe)       : all three UNSET
#
# The result is a graph the stack can see and a separately-started process
# cannot: three consecutive probes reported joint_state_msgs=0 with ik=True,
# while fsr_gripper_node INSIDE the stack was reading /joint_states happily.
# It also made `ros2 node list --no-daemon` time out from a fresh shell, which
# reads as "nothing is running" -- the exact misdiagnosis CLAUDE.md warns
# about, arriving by a different route than the stale daemon.
#
# SOURCING env.sh HERE WAS TRIED AND MADE IT WORSE. MEASURED, NOT REASONED:
# with env.sh sourced the probe went from
#
#     probe: TIMEOUT joint_state_msgs=0 arm_joints=0 ik=True  followers=0
# to
#     probe: TIMEOUT joint_state_msgs=0 arm_joints=0 ik=False followers=0
#
# i.e. it stopped seeing even /compute_ik. So the missing variables are NOT the
# cause, however well the theory fitted, and pinning them is not the fix. The
# change is reverted rather than left in the path on the strength of an
# argument -- an unvalidated "fix" in a readiness probe is how a broken graph
# gets certified.
#
# The environment difference is real and still worth recording for whoever
# picks this up: a launched node has ROS_DOMAIN_ID=0,
# ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET and RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# while a `bash -lc` shell has all three unset. It is just not what is
# isolating the probe.
SRC = "source /opt/ros/jazzy/setup.bash && source %s/install/setup.bash" % WS


def clear_shm():
    """With the stack STOPPED. Stale segments degrade discovery silently."""
    n = 0
    for d in ("/dev/shm",):
        for f in os.listdir(d):
            if f.startswith(("fastrtps_", "sem.fastrtps_")):
                try:
                    os.unlink(os.path.join(d, f))
                    n += 1
                except OSError:
                    pass
    return n


# MATCH ON THE COMMAND LINE, NOT ON `comm`. THIS IS NOT A STYLE CHOICE.
#
# `ps -o comm` truncates to 15 characters. The previous version compared that
# field against a set of full names, so three of the five entries COULD NEVER
# MATCH ANYTHING:
#
#     robot_state_publisher (21)  ->  comm reports  robot_state_pub
#     ros2_control_node     (17)  ->                ros2_control_no
#     ik_follower_node      (16)  ->                ik_follower_nod
#
# Only move_group, rviz2 and spawner were ever killed. Everything else leaked
# from EVERY session, and kill_stack() then reported a clean teardown because
# the same blind function decided what "clean" meant. Measured after a handful
# of runs: SIXTEEN live ik_follower_node processes, no move_group, no
# /tf publisher at all, and Fast DDS logging
# "Failed init_port fastrtps_port7002: open_and_lock_file failed".
#
# That is what failed the 06 re-record. All four tasks reported "capture gated
# on timeout" and the runner said "NO TF -- cannot tell whether it moved", so
# the sweep recorded FOUR FAILURES OF THE SCENE while the actual fault was a
# graph with sixteen followers and no robot_state_publisher in it. The scene
# changes under test were never exercised.
#
# The lesson is the project's own standing rule, applied to a teardown: a
# cleanup that decides its own success with the same broken predicate it
# cleans with cannot report a leak. Match the installed executable PATH, which
# is not truncated and which a shell command line cannot accidentally contain.
_STACK_PATTERNS = (
    "/lib/moveit_ros_move_group/move_group",
    "/lib/rviz2/rviz2",
    "/lib/controller_manager/ros2_control_node",
    "/lib/controller_manager/spawner",
    "/lib/robot_state_publisher/robot_state_publisher",
    "/lib/srl_teleop/",          # ik_follower_node, estop_node, mount_guard...
    "/lib/srl_autonomy/",
    "/lib/srl_perception/",
)

# THE LAUNCH PARENT MUST DIE FIRST, OR THE CHILDREN COME BACK.
#
# teleop.launch.py sets respawn=True on several nodes -- correct in the lab,
# where a Teensy attached after launch is then picked up automatically. It
# means a teardown that kills only the children is a teardown that kills
# nothing: the first kill_stack() after the truncation fix reported "10
# survived SIGKILL" and every one of those ten was a NEW pid, respawned in the
# grace period by ten leaked `ros2 launch` parents (six demo.launch.py, four
# teleop.launch.py) still running from earlier sessions.
#
# CLAUDE.md already records the converse -- killing a launch by its parent pid
# alone leaves orphans, which is what the child patterns above are for. Both
# halves are needed, in this order.
_LAUNCH_PATTERNS = (
    "ros2 launch srl_moveit_config",
    "ros2 launch srl_teleop",
    "scripts/run_teleop.sh",
)


def launch_pids():
    out = subprocess.run(["ps", "-eo", "pid,stat,args"], capture_output=True,
                         text=True).stdout.splitlines()
    me = os.getpid()
    pids = []
    for line in out[1:]:
        f = line.split(None, 2)
        if len(f) < 3 or f[1].startswith("Z"):
            continue
        if any(p in f[2] for p in _LAUNCH_PATTERNS) and int(f[0]) != me:
            pids.append(int(f[0]))
    return pids


def stack_pids():
    """Live stack processes. ZOMBIES DO NOT COUNT.

    A process that has been SIGKILLed but not yet reaped still appears in ps
    with STAT Z, and counting it made kill_stack() report "8 processes
    survived SIGKILL" moments before they all vanished -- so the session
    helper refused to start on a machine that was in fact clean. A refusal
    that fires on a transient is worse than no refusal, because the next
    person disables it.
    """
    out = subprocess.run(["ps", "-eo", "pid,stat,args"], capture_output=True,
                         text=True).stdout.splitlines()
    me = os.getpid()
    pids = []
    for line in out[1:]:
        f = line.split(None, 2)
        if len(f) < 3 or f[1].startswith("Z"):
            continue
        if not any(p in f[2] for p in _STACK_PATTERNS):
            continue
        pid = int(f[0])
        # Never count this process or the ps we just ran.
        if pid == me:
            continue
        pids.append(pid)
    return pids


def kill_stack(sig=signal.SIGINT, grace=4.0):
    # Parents first: a live launch respawns whatever we kill below.
    for p in launch_pids():
        for s2 in (signal.SIGINT, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(p), s2)
            except OSError:
                try:
                    os.kill(p, s2)
                except OSError:
                    pass
            time.sleep(0.4)
    for p in stack_pids():
        try:
            os.kill(p, sig)
        except OSError:
            pass
    time.sleep(grace)
    for p in stack_pids():
        try:
            os.kill(p, signal.SIGKILL)
        except OSError:
            pass
    # Give the kernel a moment to reap before deciding anything survived.
    for _ in range(6):
        time.sleep(0.75)
        if not stack_pids():
            break
    return stack_pids()


WAIT_PROBE = r'''
import sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
rclpy.init()
n = Node("sim_ready_probe")
got = {"js": 0, "names": set()}
def cb(m):
    got["js"] += 1
    got["names"].update(m.name)
n.create_subscription(JointState, "/joint_states", cb, 10)
cli = n.create_client(GetPositionIK, "/compute_ik")
need_fol = len(sys.argv) > 2 and sys.argv[2] == "1"
deadline = time.time() + float(sys.argv[1])
ready = False
while time.time() < deadline:
    rclpy.spin_once(n, timeout_sec=0.2)
    # THE CONDITION, not a proxy for it: messages actually RECEIVED, carrying
    # the arm joints, and the IK service answering.
    if (got["js"] >= 5
            and any(k.startswith("left_joint") for k in got["names"])
            and any(k.startswith("right_joint") for k in got["names"])
            and cli.service_is_ready()):
        # THE FOLLOWERS, when recording. Without them poses publish and the
        # arm does not move, which looks like a working run until the travel
        # check at the end.
        if need_fol:
            names = [x[0] for x in n.get_node_names_and_namespaces()]
            if not ({"ik_follower_left", "ik_follower_right"} <= set(names)):
                continue
        ready = True
        break
_nm = [x[0] for x in n.get_node_names_and_namespaces()]
print("READY" if ready else "TIMEOUT",
      "joint_state_msgs=%d arm_joints=%d ik=%s followers=%d"
      % (got["js"],
         sum(1 for k in got["names"] if "_joint_" in k),
         cli.service_is_ready(),
         sum(1 for f in ("ik_follower_left", "ik_follower_right")
             if f in _nm)))
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if ready else 1)
'''


def wait_ready(timeout_s, need_followers=False):
    """Block until a real node RECEIVES arm joint states and /compute_ik is
    up, or fail loudly. Returns True/False -- never hangs."""
    p = os.path.join("/tmp", "sim_ready_probe_%d.py" % os.getpid())
    with open(p, "w") as f:
        f.write(WAIT_PROBE)
    try:
        r = subprocess.run(["bash", "-lc",
                            "%s && python3 %s %d %d"
                            % (SRC, p, timeout_s, int(need_followers))],
                           capture_output=True, text=True,
                           timeout=timeout_s + 60)
        print("   probe: %s" % (r.stdout.strip() or r.stderr.strip()[:200]))
        return r.returncode == 0
    except subprocess.TimeoutExpired:
        print("   probe: TIMEOUT (the probe itself did not return)")
        return False
    finally:
        try:
            os.unlink(p)
        except OSError:
            pass


def respawn_storm(log_path, limit=8):
    """Nodes that have died more than `limit` times, as {node: deaths}.

    A GUARD FOR AN INDIRECTION THAT COST A SESSION. master_pose_node died 1726
    times in one launch because a missing Teensy raised PortNotFound and
    launch respawned it; the SYMPTOM was a starved controller manager and a
    dead joint_state_broadcaster, so the visible failure named neither the
    master arm nor the serial port. A count of deaths per node makes the cause
    say its own name.

    The source is fixed too -- the node now parks in a DORMANT state rather
    than exiting -- but this stays, because the next node to do it will not be
    that one.
    """
    import collections
    import re as _re
    deaths = collections.Counter()
    try:
        with open(log_path, "r", errors="ignore") as f:
            for line in f:
                m = _re.search(r"\[([\w.-]+)-\d+\]: process has died", line)
                if m:
                    deaths[m.group(1)] += 1
    except OSError:
        return {}
    return {k: v for k, v in deaths.items() if v > limit}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ready-timeout", type=int, default=180)
    ap.add_argument("--run-timeout", type=int, default=5400)
    ap.add_argument("--stack", default="moveit",
                    choices=["moveit", "teleop"],
                    help="moveit = demo.launch.py (MoveIt + RViz only; enough "
                         "for /compute_ik verification). teleop = "
                         "run_teleop.sh (adds the IK FOLLOWERS, without which "
                         "poses are published and no arm moves).")
    ap.add_argument("--keep-up", action="store_true",
                    help="leave the stack running afterwards")
    ap.add_argument("cmd", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    cmd = [c for c in a.cmd if c != "--"]
    if not cmd:
        sys.exit("nothing to run; usage: sim_session.py -- <command>")

    print("[sim] killing any existing stack")
    left = kill_stack()
    if left:
        print("[sim] REFUSING: %d stack processes survived SIGKILL: %s"
              % (len(left), left))
        return 2
    print("[sim] cleared %d stale /dev/shm segments" % clear_shm())

    log = os.path.join(WS, "log", "sim_session.log")
    os.makedirs(os.path.dirname(log), exist_ok=True)
    lf = open(log, "w")
    # master:=false IS NOT OPTIONAL HERE, AND IT IS NOT ABOUT TIDINESS.
    #
    # With no Teensy attached master_pose_node raises PortNotFound and exits 1,
    # and teleop.launch.py gives it respawn=True -- correct for the lab, where
    # a board attached after launch is then picked up automatically. In a
    # recording session it is a respawn storm: MEASURED 1726 deaths in one
    # launch, each re-scanning the serial ports, and a 17 MB log.
    #
    # The storm is not cosmetic. It starved the controller manager -- "Read
    # time: 21011 us" against a 10000 us budget, 15 missed cycles -- and
    # joint_state_broadcaster's spawner then timed out calling
    # /controller_manager/list_controllers three times and DIED. With no
    # broadcaster there is no /joint_states at all, which is exactly what the
    # readiness probe measured (joint_state_msgs=0 while ik=True followers=2)
    # and what both followers reported: "Waiting for /joint_states".
    #
    # No recording mode needs it. The sweep IS the publisher on the follower's
    # input topic in every mode -- that is what isolate() asserts -- so the
    # master arm is not in any recorded command path.
    #
    # WHICH STACK, and it is not a detail. demo.launch.py brings up MoveIt
    # and RViz and nothing else -- enough to answer /compute_ik, which is all
    # a reachability check needs. It does NOT start ik_follower_node, so a
    # RECORDING run against it publishes every pose correctly and no arm
    # moves: measured, "50 poses published ... left tf2 EE path 0.0000 m".
    # The runner caught it and refused ("no arm moved at least 0.010 m"),
    # which is the only reason 32 clips of a stationary arm were not filed as
    # results.
    launch = ("ros2 launch srl_moveit_config demo.launch.py"
              if a.stack == "moveit" else
              "bash %s/scripts/run_teleop.sh gate:=false master:=false"
              % WS)
    print("[sim] launching %s (%s) -> %s" % (launch.split()[-1], a.stack, log))
    proc = subprocess.Popen(
        ["bash", "-lc", "%s && exec %s" % (SRC, launch)],
        stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)

    print("[sim] waiting for arm joint states AND /compute_ik%s (max %d s)"
          % (" AND both IK followers" if a.stack == "teleop" else "",
             a.ready_timeout))
    if not wait_ready(a.ready_timeout, need_followers=(a.stack == "teleop")):
        print("[sim] STACK NEVER BECAME READY. Not running the command -- a "
              "run against a half-up graph produces numbers about the graph, "
              "not about the robot. See %s" % log)
        kill_stack()
        lf.close()
        return 3
    storm = respawn_storm(log)
    if storm:
        print("[sim] REFUSING: respawn storm -- %s. A node dying repeatedly "
              "starves the controller manager, and the failure then shows up "
              "somewhere else entirely. Fix the node or leave it out of the "
              "launch."
              % ", ".join("%s died %d times" % (k, v)
                          for k, v in sorted(storm.items())))
        kill_stack()
        lf.close()
        return 5
    print("[sim] ready")

    rc = 1
    try:
        r = subprocess.run(["bash", "-lc",
                            "%s && %s" % (SRC, " ".join(cmd))],
                           timeout=a.run_timeout)
        rc = r.returncode
    except subprocess.TimeoutExpired:
        print("[sim] the command exceeded --run-timeout %d s" % a.run_timeout)
        rc = 4
    finally:
        if not a.keep_up:
            print("[sim] tearing down")
            leftover = kill_stack()
            clear_shm()
            if leftover:
                print("[sim] WARNING: %d processes survived: %s"
                      % (len(leftover), leftover))
        lf.close()
    print("[sim] command exit %d" % rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
