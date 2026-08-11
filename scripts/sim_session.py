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


def stack_pids():
    """Live stack processes. ZOMBIES DO NOT COUNT.

    A process that has been SIGKILLed but not yet reaped still appears in ps
    with STAT Z, and counting it made kill_stack() report "8 processes
    survived SIGKILL" moments before they all vanished -- so the session
    helper refused to start on a machine that was in fact clean. A refusal
    that fires on a transient is worse than no refusal, because the next
    person disables it.
    """
    out = subprocess.run(["ps", "-eo", "pid,stat,comm"], capture_output=True,
                         text=True).stdout.splitlines()
    want = {"move_group", "rviz2", "ros2_control_node",
            "robot_state_publisher", "spawner"}
    pids = []
    for line in out[1:]:
        f = line.split()
        if len(f) >= 3 and f[-1] in want and not f[1].startswith("Z"):
            pids.append(int(f[0]))
    return pids


def kill_stack(sig=signal.SIGINT, grace=4.0):
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
        ready = True
        break
print("READY" if ready else "TIMEOUT",
      "joint_state_msgs=%d arm_joints=%d ik=%s"
      % (got["js"],
         sum(1 for k in got["names"] if "_joint_" in k),
         cli.service_is_ready()))
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if ready else 1)
'''


def wait_ready(timeout_s):
    """Block until a real node RECEIVES arm joint states and /compute_ik is
    up, or fail loudly. Returns True/False -- never hangs."""
    p = os.path.join("/tmp", "sim_ready_probe_%d.py" % os.getpid())
    with open(p, "w") as f:
        f.write(WAIT_PROBE)
    try:
        r = subprocess.run(["bash", "-lc",
                            "%s && python3 %s %d" % (SRC, p, timeout_s)],
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ready-timeout", type=int, default=180)
    ap.add_argument("--run-timeout", type=int, default=5400)
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
    print("[sim] launching demo.launch.py -> %s" % log)
    proc = subprocess.Popen(
        ["bash", "-lc",
         "%s && exec ros2 launch srl_moveit_config demo.launch.py" % SRC],
        stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)

    print("[sim] waiting for arm joint states AND /compute_ik (max %d s)"
          % a.ready_timeout)
    if not wait_ready(a.ready_timeout):
        print("[sim] STACK NEVER BECAME READY. Not running the command -- a "
              "run against a half-up graph produces numbers about the graph, "
              "not about the robot. See %s" % log)
        kill_stack()
        lf.close()
        return 3
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
