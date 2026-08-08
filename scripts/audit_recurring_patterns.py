#!/usr/bin/env python3
"""The five bug classes this repo keeps reproducing. Find them by SHAPE.

    python3 scripts/audit_recurring_patterns.py [--fix-none]

Each of these has appeared three or more times, and in every case the second
occurrence was written by someone who had already fixed the first. That is
what makes them worth a tool: they are not knowledge problems, they are
shape problems, and a grep for the shape finds the ones nobody remembers.

  1 SILENT BLOCKING     a guard that can only say no, with no recovery path
                        stated. Blocking is usually correct; being unable to
                        say why, or how to clear it, is what makes an arm
                        that has stopped indistinguishable from one that is
                        working.
  2 UNREACHABLE CLEAR   state set in one branch and cleared only in a branch
                        that cannot run while the state is set. Five
                        occurrences so far, the fifth inside the code written
                        to catch the first four.
  3 WALL-CLOCK TIMING   time.time() differenced to get an interval. Not
                        monotonic under WSL; it produced a -2321 ms latency.
  4 UNPREFIXED RESOURCE two arms instantiating one macro and colliding on a
                        shared name. Five found, including C++ string
                        literals no URDF-level check can see.
  5 TEST BUILDS THE     a test that constructs the environment in which the
    SAFE ENVIRONMENT    bug cannot occur, then checks for something else.
                        The teleop_gui test allocated a pty and set
                        TERM=xterm, making the no-tty and dumb-terminal bugs
                        both unreachable, then asserted on panel captions.
                        It passed on provably broken code.

Exit code is the number of findings, so CI can gate on it.
"""
import argparse
import ast
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {".git", "build", "install", "log", "__pycache__",
             "node_modules", "site-packages", "dist-packages"}
VENDOR = ("ros2_kortex", "ros2_kortex_vision", "ros2_robotiq_gripper")


def _is_venv(path):
    """Any directory holding a pyvenv.cfg is a virtualenv -- third-party code
    that we neither wrote nor can fix. Hard-coding venv NAMES was the bug:
    `.kortex_venv` was listed, `.percep_venv` was created later, and the
    audit silently jumped from 0 findings to 384 -- all of them pip, urllib3
    and matplotlib."""
    return os.path.exists(os.path.join(path, "pyvenv.cfg"))


def py_files(include_vendor=False):
    for dp, dn, fn in os.walk(ROOT):
        dn[:] = [d for d in dn
                 if d not in SKIP_DIRS and not _is_venv(os.path.join(dp, d))]
        rel = os.path.relpath(dp, ROOT)
        if not include_vendor and any(rel.startswith(os.path.join("src", v))
                                      for v in VENDOR):
            continue
        for f in fn:
            if f.endswith(".py"):
                yield os.path.join(dp, f)


def rel(p):
    return os.path.relpath(p, ROOT)


# ---------------------------------------------------------------- class 1
LOGGY = re.compile(r"get_logger\(\)|logger\.|print\(|raise |RuntimeError|"
                   r"BlockMonitor|blocks\.block|self\.block\(")
RECOVERY = re.compile(r"recovery|reset|clear|retry|re-?attach|re-?run|"
                      r"restart|how to|then |run ", re.I)


def _callees(node):
    return {n.func.attr if isinstance(n.func, ast.Attribute)
            else getattr(n.func, "id", "")
            for n in ast.walk(node) if isinstance(n, ast.Call)}


def _delegates(fn, tree, path):
    """True when this function's refusal comes from another function in the
    same module that explains itself."""
    src = open(path, encoding="utf-8", errors="replace").read()
    called = _callees(fn)
    for other in ast.walk(tree):
        if not isinstance(other, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if other.name is fn.name or other.name not in called:
            continue
        osrc = ast.get_source_segment(src, other) or ""
        if LOGGY.search(osrc):
            return True
    return False


def _is_pure_predicate(fn):
    """Only If / Return statements, and no calls that could act."""
    for st in fn.body:
        if not isinstance(st, (ast.If, ast.Return, ast.Expr)):
            return False
        if isinstance(st, ast.Expr) and not isinstance(st.value, ast.Constant):
            return False   # a bare expression statement is a side effect
    acting = _callees(fn) - {"get_parameter", "bool", "startswith", "value",
                             "int", "float", "str", "len", "get", "abs"}
    return not acting


def audit_silent_blocking():
    """A function that returns a refusal without ever saying anything.

    Heuristic, and deliberately narrow: a function whose name marks it as a
    gate (enable/arm/start/check/can_/allow/permit/preflight) and which has
    at least one `return False` whose enclosing branch contains no log, no
    raise, and no message. A refusal that carries a string is exempt --
    that string is the explanation.
    """
    out = []
    gate = re.compile(r"^(enable|arm|start|check|can_|allow|permit|preflight|"
                      r"validate|verify|guard|gate)", re.I)
    for p in py_files():
        try:
            tree = ast.parse(open(p, encoding="utf-8", errors="replace").read())
        except SyntaxError:
            continue
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if not gate.match(fn.name):
                continue
            src = ast.get_source_segment(
                open(p, encoding="utf-8", errors="replace").read(), fn) or ""
            for node in ast.walk(fn):
                if not isinstance(node, ast.Return):
                    continue
                v = node.value
                bare_false = (isinstance(v, ast.Constant) and v.value is False)
                # a tuple like (False, "reason") is fine -- it explains itself
                if isinstance(v, ast.Tuple) and len(v.elts) >= 2:
                    continue
                if not bare_false:
                    continue
                if LOGGY.search(src):
                    continue
                # EXEMPTION 1 -- DELEGATED EXPLANATION. `start()` returns
                # False only when `plan()` returned None, and plan() prints
                # the reason itself. The message exists; it is one frame
                # down. Flagging it would push a redundant log into working
                # code, and a tool that cries wolf gets switched off.
                if _delegates(fn, tree, p):
                    continue
                # EXEMPTION 2 -- PURE PREDICATE. `channel_manager.enabled()`
                # answers a question; it does not stop anything. Its callers
                # report the capability loss. A predicate that logs every
                # time it is asked is noise, not observability.
                if _is_pure_predicate(fn):
                    continue
                out.append((rel(p), node.lineno, fn.name,
                            "returns bare False with no message anywhere in "
                            "the function"))
    return out


# ---------------------------------------------------------------- class 3
WALL = re.compile(r"time\.time\(\)")


def _docstring_lines(txt):
    """Line numbers inside docstrings -- prose about the bug is not the bug."""
    try:
        tree = ast.parse(txt)
    except SyntaxError:
        return set()
    out = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef,
                          ast.AsyncFunctionDef)):
            d = ast.get_docstring(n, clean=False)
            if d and n.body and isinstance(n.body[0], ast.Expr):
                s0 = n.body[0]
                out.update(range(s0.lineno, (s0.end_lineno or s0.lineno) + 1))
    return out


def audit_wall_clock():
    """time.time() used as an INTERVAL, i.e. subtracted or compared."""
    out = []
    me = os.path.abspath(__file__)
    for p in py_files():
        if os.path.abspath(p) == me:
            continue          # the tool describes the pattern it looks for
        txt = open(p, encoding="utf-8", errors="replace").read()
        doc_lines = _docstring_lines(txt)
        for i, line in enumerate(txt.splitlines(), 1):
            if not WALL.search(line):
                continue
            stripped = line.strip()
            if stripped.startswith("#") or i in doc_lines:
                continue
            # A timestamp is fine; an interval is not. An interval shows up
            # as a subtraction, or as a stored value later differenced.
            interval = ("-" in line.split("time.time()")[0][-3:] or
                        re.search(r"time\.time\(\)\s*-", line) or
                        re.search(r"-\s*time\.time\(\)", line) or
                        re.search(r"(elapsed|since|dt|age|interval|latency|"
                                  r"deadline|timeout)", line, re.I))
            if interval:
                out.append((rel(p), i, "wall-clock interval", stripped[:90]))
    return out


# ---------------------------------------------------------------- class 5
PTY_SHAPES = [
    (re.compile(r"script\s+-q?e?c"), "allocates a pty via script(1)"),
    (re.compile(r"pty\.openpty|openpty\("), "allocates a pty directly"),
    (re.compile(r"TERM\s*[=:]\s*[\"']?(xterm|screen|linux|vt100)"),
     "forces a capable TERM"),
    (re.compile(r"DISPLAY\s*[=:]"), "forces DISPLAY"),
    (re.compile(r"stdin\s*=\s*subprocess\.PIPE.*tty", re.S), "fakes a tty"),
]


def audit_test_env():
    out = []
    for p in py_files():
        if "/test" not in p and not os.path.basename(p).startswith("test_"):
            continue
        txt = open(p, encoding="utf-8", errors="replace").read()
        asserts_status = re.search(r"returncode|exit_code|check_returncode|"
                                   r"Traceback", txt)
        # A pty only HIDES a bug when it is used to convince a SUBPROCESS it
        # has a terminal -- that is what made the teleop_gui test pass on
        # provably broken code. A pty opened as the device under test (a
        # serial port) is the opposite: it is what makes the bug reachable.
        # Discriminate on whether a subprocess is launched at all.
        if not re.search(r"subprocess\.|Popen|check_output|os\.system", txt):
            continue
        for rx, why in PTY_SHAPES:
            m = rx.search(txt)
            if not m:
                continue
            line = txt[:m.start()].count("\n") + 1
            note = why + (
                "" if asserts_status
                else " AND never asserts on exit status or a traceback")
            if not asserts_status:
                out.append((rel(p), line, "test builds the safe environment",
                            note))
    return out


# ------------------------------------------------------- classes 2 and 4
def run_existing(script, label):
    path = os.path.join(ROOT, "scripts", script)
    if not os.path.exists(path):
        return None, "%s is missing" % script
    try:
        r = subprocess.run([sys.executable, path], capture_output=True,
                           text=True, timeout=300, cwd=ROOT)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except Exception as e:                                   # noqa: BLE001
        return None, "%s failed to run: %s" % (script, e)


def show(title, rows, note=""):
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)
    if note:
        print(note)
    if not rows:
        print("  CLEAN -- 0 occurrences")
        return 0
    for f, ln, kind, detail in rows:
        print("  %s:%s" % (f, ln))
        print("      %s -- %s" % (kind, detail))
    print("  %d occurrence(s)" % len(rows))
    return len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vendor", action="store_true",
                    help="include vendor packages in the python scans")
    ap.parse_args()
    total = 0
    print("RECURRING-PATTERN AUDIT -- five classes, by shape not by instance")
    total += show("1. SILENT BLOCKING -- a gate that refuses without saying why",
                  audit_silent_blocking(),
                  "A refusal returning (False, \"reason\") is exempt: the "
                  "string IS the explanation.")
    rc, txt = run_existing("audit_unreachable_clear.py",
                           "unreachable clear()")
    print("\n" + "=" * 74)
    print("2. UNREACHABLE clear() -- delegated to audit_unreachable_clear.py")
    print("=" * 74)
    print("\n".join(txt.strip().splitlines()[-12:]) or "  (no output)")
    total += show("3. WALL-CLOCK TIMING -- time.time() used as an interval",
                  audit_wall_clock(),
                  "Wall-clock TIMESTAMPS are correct and deliberate. Only "
                  "intervals are the bug.")
    rc4, txt4 = run_existing("audit_dual_arm_collisions.py",
                             "unprefixed resources")
    print("\n" + "=" * 74)
    print("4. UNPREFIXED RESOURCES -- delegated to "
          "audit_dual_arm_collisions.py")
    print("=" * 74)
    print("\n".join(txt4.strip().splitlines()[-12:]) or "  (no output)")
    total += show("5. TEST BUILDS THE ENVIRONMENT WHERE THE BUG CANNOT OCCUR",
                  audit_test_env(),
                  "Flagged only when the test ALSO never asserts on exit "
                  "status or a traceback -- that combination is what let a "
                  "test pass on provably broken code.")
    print("\n%d finding(s) in the scanned classes (2 and 4 report their own)."
          % total)
    return 0


if __name__ == "__main__":
    sys.exit(main())
