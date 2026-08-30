#!/usr/bin/env python3
"""reset_session.py -- put this machine back into a known state, in order.

    python3 scripts/reset_session.py            # say what it would do
    python3 scripts/reset_session.py --do       # do it
    python3 scripts/reset_session.py --do --keep-cameras

WHY THIS EXISTS. Every repair it runs already existed, each behind its own
button, and the machine still ended most sessions with 133 leftover blocks of
shared memory, 16 orphaned nodes, two stacks and a window on the wrong ROS
domain -- because the repairs only work in ONE ORDER and nothing enforced it:

  * shared memory must be cleared with NOTHING RUNNING (HARD CONSTRAINT 5).
    Clearing it under a live stack partitions the graph -- measured: 34 nodes
    to 7, zero publishers on /tf, every controller still active.
  * the Kortex bridges must be SIGINTed and given time, not killed, or the
    arm's one session leaks and the next run cannot connect (HARD CONSTRAINT
    2).
  * clearing shared memory invalidates the daemon's cached graph, so the
    daemon is restarted AFTER the clear, not before -- restarting it first
    orphans its own fresh segments.
  * and a camera that says Attached is not a camera that delivers, so the
    camera step probes rather than looking.

It ends by running the connection checks and printing the verdict, so the
answer to "is this machine clean" is measured rather than assumed.
"""
import argparse
import os
import subprocess
import sys
import time

_WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_WS, "src/srl_teleop"))

from srl_teleop import procscan                               # noqa: E402
from srl_teleop import real_arm_doctor as rad                 # noqa: E402

#: Stopped FIRST and with SIGINT, because a killed bridge leaks the arm's
#: one session. Everything else can be stopped in any order after them.
KORTEX_RX = r"kortex_highlevel_bridge|sim_to_real_bridge"


def say(*a):
    print(*a, flush=True)


def stop_kortex(wait_s=8.0):
    """SIGINT the arm bridges and WAIT.

    Never SIGKILL: the arm permits exactly one Kortex session and a killed
    bridge leaks it, so the next run cannot connect (HARD CONSTRAINT 2).
    """
    import signal
    found = procscan.find(KORTEX_RX)
    if not found:
        return "no arm bridge running"
    for pid, _ in found:
        try:
            os.kill(pid, signal.SIGINT)
        except OSError:
            pass
    t0 = time.time()
    while time.time() - t0 < wait_s:
        if not procscan.find(KORTEX_RX):
            return "%d arm bridge(s) closed cleanly" % len(found)
        time.sleep(0.5)
    left = procscan.find(KORTEX_RX)
    return ("%d arm bridge(s) asked to stop, %d STILL RUNNING -- do not kill "
            "them, the arm allows one session and killing leaks it"
            % (len(found), len(left)))


def stop_everything(wait_s=10.0):
    """Stop every node, launch and viewer this workspace starts.

    Explicit patterns and process GROUPS, never a broad pkill (HARD
    CONSTRAINT 9). The GUI is deliberately NOT in the list: this is run from
    it as often as from a terminal.
    """
    import signal
    pats = [r"ros2 launch srl_", r"run_teleop\.sh", r"run_autonomy\.sh",
            r"/install/[A-Za-z0-9_]+/lib/[A-Za-z0-9_]+/",
            r"lib/moveit_ros_move_group/move_group",
            r"lib/controller_manager/ros2_control_node",
            r"lib/robot_state_publisher/robot_state_publisher",
            r"/rviz2"]
    targets = {}
    for rx in pats:
        for pid, cmd in procscan.find(rx):
            targets[pid] = cmd
    if not targets:
        return "nothing was running"
    for pid in list(targets):
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except OSError:
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
    t0 = time.time()
    while time.time() - t0 < wait_s:
        if not any(procscan.find(rx) for rx in pats):
            return "stopped %d part(s)" % len(targets)
        time.sleep(0.5)
    stubborn = {pid for rx in pats for pid, _ in procscan.find(rx)}
    for pid in stubborn:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(2.0)
    left = {pid for rx in pats for pid, _ in procscan.find(rx)}
    return ("stopped %d part(s); %d needed a second ask%s"
            % (len(targets), len(stubborn),
               "" if not left else "; %d STILL UP" % len(left)))


def cameras():
    r = subprocess.run([sys.executable,
                        os.path.join(_WS, "scripts", "usb_cameras.py"),
                        "--fix"],
                       capture_output=True, text=True, timeout=300)
    tail = [ln.strip() for ln in (r.stdout or "").splitlines() if ln.strip()]
    return "; ".join(tail[-3:]) or "no output"


def verdict():
    checks = rad.run_all()
    state, head = rad.verdict(checks)
    return state, head, checks


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.
                                 RawDescriptionHelpFormatter)
    ap.add_argument("--do", action="store_true",
                    help="actually do it (without this it only reports)")
    ap.add_argument("--keep-cameras", action="store_true",
                    help="do not touch the usbip camera attachments")
    a = ap.parse_args()

    say("== BEFORE ==")
    state, head, checks = verdict()
    for c in checks:
        say("  %-14s %-8s %s" % (c.key, c.state, c.plain[:78]))
    say("  verdict: %s" % head)

    if not a.do:
        say("\nRe-run with --do to reset. It stops everything, clears the "
            "leftovers with nothing running, restarts the helper and "
            "re-attaches the cameras.")
        return 0

    say("\n== RESET ==")
    say("  1. arm bridges .... %s" % stop_kortex())
    say("  2. everything else. %s" % stop_everything())
    ok, msg = rad.fix_kill_orphans()
    say("  3. leftovers ...... %s" % msg)
    try:
        subprocess.run(["ros2", "daemon", "stop"], capture_output=True,
                       timeout=15)
    except Exception as e:                                    # noqa: BLE001
        say("  4. helper ......... could not stop it: %r" % (e,))
    else:
        say("  4. helper ......... stopped, so it cannot hold memory open")
    procscan.kill_all(r"ros2cli\.daemon\.daemonize")
    time.sleep(1.0)
    ok, msg = rad.fix_clear_shm()
    say("  5. shared memory .. %s" % msg)
    # THE FIVE-SECOND SETTLE IS NOT SUPERSTITION. Participants that were
    # using a cleared segment do not find out instantly, and a stack started
    # into a half-torn-down transport comes up unable to see itself.
    time.sleep(5.0)
    ok, msg = rad.fix_reset_daemon()
    say("  6. helper ......... %s" % msg)
    if a.keep_cameras:
        say("  7. cameras ........ left alone (--keep-cameras)")
    else:
        try:
            say("  7. cameras ........ %s" % cameras())
        except Exception as e:                                # noqa: BLE001
            say("  7. cameras ........ could not run the camera step: %r"
                % (e,))

    say("\n== AFTER ==")
    state, head, checks = verdict()
    for c in checks:
        say("  %-14s %-8s %s" % (c.key, c.state, c.plain[:78]))
    say("  verdict: %s" % head)
    # A RESET THAT LEAVES THE MACHINE BAD MUST SAY SO IN ITS EXIT CODE, or
    # a script that chains off it carries on into the same mess.
    return 0 if state != rad.BAD else 1


if __name__ == "__main__":
    sys.exit(main())
