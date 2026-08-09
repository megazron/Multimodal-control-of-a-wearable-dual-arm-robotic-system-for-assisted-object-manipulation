#!/usr/bin/env python3
"""Find processes without matching yourself. The ONLY sanctioned way here.

WHY THIS FILE EXISTS. `pgrep -f <pattern>` matches against every process's
full command line -- INCLUDING the shell that is running the pgrep, whose
command line contains the pattern as a literal string. In this project that
has produced four separate failures, each of which looked like a fact about
the system rather than a fact about the search:

  1. `pkill -f "lib/srl_teleop/estop_node"` KILLED THE SHELL RUNNING IT,
     because that shell's command line matched. Three working shells died in
     one session.
  2. A leftover-process check grepped for bare names (`move_group`,
     `master_pose_node`) and matched its OWN heredoc, which quoted those
     words -- reporting 2-3 leftover processes after a clean shutdown when
     there were none.
  3. `pkill -f "ros2 launch"` plus a grep-based kill sweep took out a
     `spawner`, deactivated `joint_state_broadcaster` and both arm
     controllers, killed `/tf`, and cost a 20440-row recording its entire
     end-effector column. That one was diagnosed as a recorder bug for a
     while.
  4. `pgrep -f "Xvfb :99"` matched the shell running it, so `ensure_xvfb()`
     believed the server was already up and never started it. EVERY GUI run
     then died with "could not connect to display :99" -- while the harness
     reported `alive: True`, because the rclpy thread outlived Qt.

The common shape: the search is part of the system it is searching, and the
answer includes the question. It has been written down as a trap four times
and has recurred four times, so writing it down is not working.

THE RULE, ENFORCED HERE RATHER THAN REMEMBERED:

  * `find()` excludes THIS process, its parent, its whole process group, and
    every ancestor -- so a shell, a wrapper script and a `bash -c` that
    mention the pattern can never be counted.
  * It reads /proc directly instead of shelling out, so there is no
    intermediate shell whose command line contains the pattern at all. That
    removes the failure mode rather than filtering it out afterwards.
  * `kill_all()` refuses to signal anything `find()` excluded, so the
    kill-your-own-shell failure is unreachable, not merely unlikely.
  * PREFER `probe` OVER `find`. Asking "is a process called X running" is
    usually a proxy for "does the thing X provides work". Where a direct
    probe exists -- can an X client open the display, does the service answer
    -- use it: a process can be running and useless, and it was exactly that
    substitution (process-name for condition) that let a dead display report
    as healthy.
"""
import json
import os
import re
import signal
import time


def _ancestors(pid=None):
    """Every PID from `pid` up to init, plus this process's group."""
    pid = os.getpid() if pid is None else pid
    out = set()
    try:
        out.add(os.getpgid(0))
    except OSError:
        pass
    seen = 0
    while pid and pid > 1 and seen < 64:
        out.add(pid)
        seen += 1
        try:
            with open("/proc/%d/status" % pid) as fh:
                m = re.search(r"^PPid:\s*(\d+)", fh.read(), re.M)
            pid = int(m.group(1)) if m else 0
        except (OSError, ValueError):
            break
    return out


def _cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            return fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def find(pattern, exclude_pids=()):
    """[(pid, cmdline)] matching `pattern`, NEVER including the caller.

    `pattern` is a regular expression matched against the full command line.
    """
    rx = re.compile(pattern)
    blocked = _ancestors() | set(exclude_pids)
    # A sibling of this process in the same group -- the other half of a
    # shell pipeline, say -- is just as much "the question" as the parent.
    try:
        mygrp = os.getpgid(0)
    except OSError:
        mygrp = None
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in blocked:
            continue
        if mygrp is not None:
            try:
                if os.getpgid(pid) == mygrp:
                    continue
            except OSError:
                pass
        cmd = _cmdline(pid)
        if cmd and rx.search(cmd):
            out.append((pid, cmd))
    return out


def count(pattern, exclude_pids=()):
    return len(find(pattern, exclude_pids))


def kill_all(pattern, sig=signal.SIGTERM, grace_s=4.0, exclude_pids=()):
    """Signal everything `find()` returns, then SIGKILL what survives.

    Returns (signalled, survivors). Cannot signal the caller or any ancestor,
    because it can only signal what find() returns and find() excludes them.
    """
    targets = find(pattern, exclude_pids)
    for pid, _ in targets:
        try:
            os.kill(pid, sig)
        except OSError:
            pass
    if not targets:
        return [], []
    end = time.monotonic() + grace_s
    while time.monotonic() < end:
        if not find(pattern, exclude_pids):
            return targets, []
        time.sleep(0.2)
    survivors = find(pattern, exclude_pids)
    for pid, _ in survivors:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    return targets, survivors


def self_test():
    """Reproduce the trap EXACTLY, then show find() is immune to it.

    The condition that matters is the caller's OWN command line containing the
    pattern. A test that searches for something the caller never mentions
    proves nothing -- that is the case which has always worked, and it is why
    this bug survived being written down four times.

    So: a child is launched through `bash -c "<script containing PATTERN>"`.
    That bash's command line therefore contains PATTERN literally. From inside
    it, `pgrep -f PATTERN` is asked the same question as `find(PATTERN)`, and
    the two answers are compared. pgrep must return its own shell; find must
    not.
    """
    import subprocess
    import sys
    marker = "SRL_TRAP_MARKER_%d" % os.getpid()
    here = os.path.dirname(os.path.abspath(__file__))
    # The marker appears in this bash command line, in the python -c source,
    # and nowhere else. Nothing that matches it is a process anyone wants.
    script = (
        "python3 -c \"import sys,json; sys.path.insert(0,'%s');"
        "import procscan;"
        "print(json.dumps({"
        "'find': [p for p,_ in procscan.find('%s')],"
        "'pgrep': __import__('subprocess').run(['pgrep','-f','%s'],"
        "capture_output=True,text=True).stdout.split()}))\""
        % (here, marker, marker))
    r = subprocess.run(["bash", "-c", script], capture_output=True, text=True,
                       timeout=60)
    try:
        got = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:                                         # noqa: BLE001
        return dict(ok=False, error="child produced no JSON: %r"
                    % (r.stdout or r.stderr)[:200])
    pgrep_hits = [int(x) for x in got["pgrep"] if str(x).isdigit()]
    find_hits = got["find"]
    # pgrep sees the shell (and/or the python) that asked the question.
    # If it does not, this machine's pgrep is not reproducing the trap and the
    # comparison would be vacuous -- so that is reported, not hidden.
    return dict(pgrep_hits=len(pgrep_hits), find_hits=len(find_hits),
                trap_reproduced=len(pgrep_hits) > 0,
                find_returned_nothing=(len(find_hits) == 0),
                ok=(len(pgrep_hits) > 0 and len(find_hits) == 0))


if __name__ == "__main__":
    import sys
    r = self_test()
    print(json.dumps(r, indent=2))
    if not r.get("trap_reproduced"):
        print("\nINCONCLUSIVE: pgrep did not match the caller here, so this "
              "run did not reproduce the trap and proves nothing.")
        sys.exit(1)
    print("\npgrep -f matched %d process(es) that were only the QUESTION"
          % r["pgrep_hits"])
    print("procscan.find() returned %d  -> immune" % r["find_hits"])
    sys.exit(0 if r["ok"] else 1)
