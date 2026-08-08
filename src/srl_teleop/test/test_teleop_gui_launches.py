#!/usr/bin/env python3
"""teleop_gui must LAUNCH. Exercises the real entry point, not an import.

WHY THIS EXISTS, AND WHY THE PREVIOUS TEST WAS WORSE THAN NOTHING.

The earlier check reported "verified: renders and exits cleanly through a
pty" while `ros2 run srl_teleop teleop_gui` crashed on launch. It passed on
broken code for three independent reasons, every one of which is a lesson:

  1. It drove the program through `script -qec`, which ALLOCATES A PTY. The
     bug was that curses dies without a real terminal -- so the harness
     constructed exactly the environment in which the bug cannot occur.
  2. It exported TERM=xterm. The other bug was curs_set() failing under
     TERM=dumb, again made unreachable by the harness.
  3. It asserted only that panel captions appeared in the output, and never
     checked the exit code or looked for a traceback. `script` returns its
     OWN status, so the child's failure was invisible.

So the rules here are: run the INSTALLED entry point; assert on the EXIT
CODE and the ABSENCE of a traceback; and test the degraded environments
explicitly rather than avoiding them.
"""
import os
import pty
import select
import shutil
import subprocess
import sys
import time

import pytest

CMD = ["ros2", "run", "srl_teleop", "teleop_gui"]
pytestmark = pytest.mark.skipif(shutil.which("ros2") is None,
                                reason="ros2 not on PATH")


def _env(**over):
    e = dict(os.environ)
    e.setdefault("ROS_DOMAIN_ID", "0")
    e.pop("ROS_LOCALHOST_ONLY", None)
    e.update(over)
    return e


def test_refuses_without_a_tty_and_says_why():
    """No pty. Must exit non-zero with an ACTIONABLE message, not a traceback.

    This is the case that actually broke: curses' cbreak() returns ERR, and
    wrapper's cleanup then calls nocbreak() which ALSO returns ERR and
    replaces the original exception, so the traceback names the wrong call.
    """
    p = subprocess.run(CMD, capture_output=True, text=True, timeout=90,
                       stdin=subprocess.DEVNULL, env=_env(TERM="xterm"))
    out = p.stdout + p.stderr
    assert "Traceback" not in out, (
        "crashed with a traceback instead of refusing cleanly:\n" + out[-1500:])
    assert p.returncode != 0
    assert "not a terminal" in out, \
        "refusal must say WHY and how to run it. Got:\n" + out[-800:]


def test_refuses_on_dumb_terminal_and_says_why():
    """TERM=dumb WITH a real tty -- otherwise the no-tty guard fires first
    and this would silently test the wrong thing."""
    out, status = _run_in_pty("dumb", 80, 24, seconds=5.0, expect_exit=True)
    assert "Traceback" not in out, out[-1500:]
    assert "TERM" in out, "must name TERM as the cause. Got:\n" + out[-600:]


def _run_in_pty(term, cols, rows, seconds=8.0, keys=b"", expect_exit=False):
    """Real pty, real entry point. Returns (raw_output, exit_status)."""
    pid, fd = pty.fork()
    if pid == 0:
        os.environ.update(_env(TERM=term, COLUMNS=str(cols), LINES=str(rows)))
        try:
            os.execvp(CMD[0], CMD)
        finally:
            os._exit(127)
    import fcntl
    import struct
    import termios
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
    buf = b""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if select.select([fd], [], [], 0.2)[0]:
            try:
                c = os.read(fd, 65536)
            except OSError:
                break
            if not c:
                break
            buf += c
    for k in keys:
        os.write(fd, bytes([k]))
        time.sleep(0.3)
        while select.select([fd], [], [], 0)[0]:
            try:
                buf += os.read(fd, 65536)
            except OSError:
                break
    if not expect_exit:
        os.write(fd, b"q")
    # Drain until the child actually exits. Closing the pty first would send
    # it SIGHUP and the exit status would report the signal, not the program.
    t0 = time.monotonic()
    status = None
    while time.monotonic() - t0 < 15.0:
        done, st = os.waitpid(pid, os.WNOHANG)
        if done:
            status = st
            break
        if select.select([fd], [], [], 0.2)[0]:
            try:
                c = os.read(fd, 65536)
            except OSError:
                break
            if c:
                buf += c
    if status is None:
        try:
            os.kill(pid, 15)
        except ProcessLookupError:
            pass
        _, status = os.waitpid(pid, 0)
    try:
        os.close(fd)
    except OSError:
        pass
    return buf.decode("utf-8", "replace"), status


def test_runs_in_a_real_terminal_and_draws():
    # Longer settle than the other cases: run inside the full suite this
    # shares the machine with everything before it, and the first frame is
    # gated on ROS discovery. 8 s was enough standalone and flaked in-suite.
    out, status = _run_in_pty("xterm-256color", 170, 50, seconds=14.0)
    assert "Traceback" not in out, out[-1500:]
    assert "SRL TELEOP CONSOLE" in out
    assert "BLOCKERS" in out, "the panel that answers 'why is nothing moving'"
    assert os.WIFEXITED(status), "did not exit cleanly on q"
    assert os.WEXITSTATUS(status) == 0, \
        "exit status %d" % os.WEXITSTATUS(status)


def test_survives_a_small_terminal():
    """A short window must not raise from addstr past the last row."""
    out, _ = _run_in_pty("xterm", 40, 10, seconds=6.0)
    assert "Traceback" not in out, out[-1500:]


def test_every_control_key_is_handled():
    """Drive each documented key. None may raise.

    The keys reach parameter clients for nodes that may not be running; the
    correct behaviour is a logged failure, never an exception.
    """
    keys = b"\tvVbBsSeEgGmMlLdDpocC1234567?"
    out, status = _run_in_pty("xterm-256color", 170, 50, seconds=6.0,
                              keys=keys)
    assert "Traceback" not in out, out[-2000:]
    assert os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
