#!/usr/bin/env python3
"""WORKING ALONE: a deliberate, logged, per-session bypass of the observer.

    from srl_teleop import observer_bypass
    observer_bypass.grant(who="gui", reason="single-operator session")
    observer_bypass.active()        -> (True, record) | (False, why)
    observer_bypass.clear()

WHAT THIS IS, AND WHAT IT IS NOT
--------------------------------
The observer interlock stays exactly as it was. Nothing here removes a check,
lowers a floor, or changes a default. This adds a SECOND, NAMED thing beside
it, so that an operator who is genuinely alone in the lab can get past it
without doing the one thing that would be worse -- editing the interlock, or
launching with `require_observer_estop:=false`, which is how a bypass becomes
permanent and unrecorded.

The distinction this file exists to preserve:

    OBSERVED    somebody who can see the arm is holding a physical e-stop
    BYPASSED    nobody is, and a named person decided that at a known time
    UNKNOWN     nothing is saying either way

The old code could express the first and the third. A run with no observer was
indistinguishable from a run where the heartbeat had not started yet, so the
honest state -- "there is no observer and I know it" -- had nowhere to live.

THREE PROPERTIES, AND EACH IS A RULE THAT WAS ASKED FOR
------------------------------------------------------
1. **It is deliberate every session.** The grant lives in scratch, not in
   config, not in a parameter file and not in git. `clear()` is called when
   the operations window opens, so a grant never survives to the next session.
   It also EXPIRES on its own after `TTL_S`, which is the backstop for the
   case the window is never closed: a machine left running overnight must not
   still be authorised in the morning.

2. **It is never the default.** `active()` returns False for a missing file,
   an unreadable file, a file from before the machine last booted, and a file
   older than the TTL. Every one of those is "no bypass", because the only
   safe direction for this particular question is towards the interlock.

3. **Every use is on the record.** `grant()` appends one line to
   `recordings/observer_bypass_log.jsonl` with a wall-clock timestamp, and
   that file is never rewritten or rotated from here. When somebody later
   asks "was anyone watching the arm during run 7", the answer is a file
   rather than a memory.

WHY A FILE AND NOT A ROS PARAMETER. Two processes have to agree about this --
the bring-up sequence and `vr_safety_node` -- and they do not share a
lifetime: the safety node is restarted by repairs, and a parameter set on the
old one is silently gone. A parameter would also be settable from any shell
with no record of who did it, which defeats the whole point of the third
property.
"""
from __future__ import annotations

import getpass
import json
import os
import socket
import time

STATE_ENV = "SRL_OBSERVER_BYPASS_FILE"
STATE_NAME = "vr_observer_bypass.json"
LOG_NAME = "observer_bypass_log.jsonl"

# THE BACKSTOP, NOT THE MECHANISM. The mechanism is that the window clears it
# on open. This is what catches the machine nobody closed.
TTL_S = 4 * 3600.0


def _ws():
    return os.environ.get("SRL_WS") or os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", ".."))


def _scratch():
    d = os.environ.get("SRL_SCRATCH") or os.path.join(_ws(), ".scratch")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        return "/tmp"
    return d


def state_path():
    return os.environ.get(STATE_ENV) or os.path.join(_scratch(), STATE_NAME)


def log_path():
    d = os.path.join(_ws(), "recordings")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        return os.path.join(_scratch(), LOG_NAME)
    return os.path.join(d, LOG_NAME)


def _boot_time():
    """Wall-clock time this machine booted, or None.

    A grant written before the last boot is not this session's grant, whatever
    its timestamp says. This is the one check that survives a crash-and-restart
    with the clock untouched.
    """
    try:
        with open("/proc/uptime") as fh:
            return time.time() - float(fh.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def grant(who="unknown", reason="", ttl_s=TTL_S):
    """Take the bypass, now, and write it down. Returns the record."""
    rec = {
        "granted_at": time.time(),
        "granted_at_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "ttl_s": float(ttl_s),
        "who": who,
        "reason": reason or "operator working alone",
        "user": _safe(getpass.getuser),
        "host": _safe(socket.gethostname),
        "pid": os.getpid(),
    }
    with open(state_path(), "w") as fh:
        json.dump(rec, fh, indent=2)
    # THE AUDIT LINE IS WRITTEN EVEN IF THE STATE FILE FAILED TO LAND. The
    # record of the decision matters more than the mechanism that carries it.
    try:
        with open(log_path(), "a") as fh:
            fh.write(json.dumps(dict(rec, event="granted")) + "\n")
    except OSError:
        pass
    return rec


def clear(who="unknown", note="session start"):
    """Drop any bypass. Safe to call when there is none.

    Logged only when there was something to clear, so the audit file records
    decisions rather than every time a window opened.
    """
    p = state_path()
    had = None
    if os.path.exists(p):
        try:
            with open(p) as fh:
                had = json.load(fh)
        except (OSError, ValueError):
            had = {"unreadable": True}
        try:
            os.remove(p)
        except OSError:
            pass
    if had is not None:
        try:
            with open(log_path(), "a") as fh:
                fh.write(json.dumps({
                    "event": "cleared", "who": who, "note": note,
                    "cleared_at": time.time(),
                    "cleared_at_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    "was": had}) + "\n")
        except OSError:
            pass
    return had is not None


def active():
    """(True, record) if a bypass is in force right now, else (False, why).

    Every failure path returns False. There is no branch here that turns an
    error into permission.
    """
    p = state_path()
    if not os.path.exists(p):
        return False, "no bypass has been taken"
    try:
        with open(p) as fh:
            rec = json.load(fh)
    except (OSError, ValueError) as e:
        return False, "the bypass record is unreadable (%s)" % (e,)
    try:
        at = float(rec["granted_at"])
        ttl = float(rec.get("ttl_s", TTL_S))
    except (KeyError, TypeError, ValueError):
        return False, "the bypass record is malformed"
    age = time.time() - at
    if age < 0:
        return False, "the bypass record is dated in the future"
    if age > ttl:
        return False, ("the bypass expired %.1f h ago -- take it again if you "
                       "are still alone" % ((age - ttl) / 3600.0))
    boot = _boot_time()
    if boot is not None and at < boot:
        return False, "the bypass was taken before this machine last started"
    return True, rec


def describe():
    """One plain sentence about the current state. For a status line."""
    on, info = active()
    if not on:
        return "Observer required (no bypass): %s." % info
    return ("OBSERVER BYPASSED -- taken by %s at %s. Nobody is holding an "
            "e-stop for this run."
            % (info.get("who", "?"), info.get("granted_at_iso", "?")))


def history(limit=20):
    """The last `limit` audit lines, newest last. [] if there are none."""
    try:
        with open(log_path()) as fh:
            lines = [ln for ln in fh.read().splitlines() if ln.strip()]
    except OSError:
        return []
    out = []
    for ln in lines[-limit:]:
        try:
            out.append(json.loads(ln))
        except ValueError:
            continue
    return out


def _safe(fn):
    try:
        return fn()
    except Exception:                                          # noqa: BLE001
        return "unknown"
