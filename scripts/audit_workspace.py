#!/usr/bin/env python3
"""Audit the whole workspace for the failure classes this project actually has.

    python3 scripts/audit_workspace.py            # report only
    python3 scripts/audit_workspace.py --json     # machine-readable

REPORTS, DOES NOT FIX. Every finding names a file and a line so it can be
checked before anything is changed, because several of the classes below are
ones where the "obvious" fix is wrong.

The classes are not generic lint. Each is a bug that has actually bitten this
project, and the check exists because reading did not catch it:

  A  wall-clock used for an INTERVAL. time.time() steps backwards on host
     resync under WSL and once produced a send latency of -2321 ms.
  B  unreachable clear(): state set in one branch and cleared only where the
     guard that set it prevents the code from reaching. Five occurrences.
  C  unprefixed resources colliding across two arms, INCLUDING C++ string
     literals that no URDF check can see.
  D  a second stack: anything that can start a second master_pose_node, which
     splits the serial stream and invalidated a full day of measurements.
  E  entry points that do not resolve, and modules with a main() that are
     never registered.
  F  stale references: paths, topics and node names that moved through the
     six-package restructure, the E-to-T task rename and the mode reorg. The
     failure is always silent -- a button that runs nothing.
  G  dead code: defined and never referenced anywhere.
  H  known-answer tests that can pass on NO DATA (absence read as a value).
"""
import argparse
import ast
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {".git", "build", "install", "log", "__pycache__", "_template",
             ".percep_venv", ".kortex_venv", "node_modules", "recordings",
             # The prior-work archive is a HISTORICAL RECORD, not maintained
             # code. Auditing it reports defects in programs that were
             # superseded months ago and must not be edited: their value is
             # that they show what was actually done at the time.
             "prior_work"}
# THE AUDIT MUST NOT SCAN ITSELF. Every check here contains the pattern it
# looks for, as a string, so a self-scan reports the checker as a finding.
# The leftover-process check in this project once reported 2-3 phantom
# processes for exactly this reason: it matched its own command line.
SELF = {"audit_workspace.py", "audit_recurring_patterns.py",
        "audit_unreachable_clear.py"}
VENDOR = ("ros2_kortex", "ros2_kortex_vision", "ros2_robotiq_gripper",
          "srl_moveit_config/config")


def py_files(include_vendor=False):
    for d, dirs, files in os.walk(ROOT):
        dirs[:] = [x for x in dirs if x not in SKIP_DIRS]
        rel = os.path.relpath(d, ROOT)
        if not include_vendor and any(v in rel for v in VENDOR):
            continue
        for f in files:
            if f.endswith(".py") and f not in SELF:
                yield os.path.join(d, f)


def rel(p):
    return os.path.relpath(p, ROOT)


def read(p):
    try:
        return open(p, encoding="utf-8", errors="replace").read()
    except Exception:                                        # noqa: BLE001
        return ""



def string_literal_lines(src):
    """Line numbers that fall inside a string literal (docstrings included).

    A prefix test only catches a docstring's FIRST line. The one surviving
    false positive was a usage example on line 39 of a module docstring, which
    starts nowhere near a quote character. The AST knows where literals are.
    """
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return set()
    bad = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            end = getattr(node, "end_lineno", node.lineno) or node.lineno
            bad.update(range(node.lineno, end + 1))
    return bad


# ------------------------------------------------------------------ A: clock
def check_wallclock():
    """time.time() used in a SUBTRACTION is an interval, and is wrong here."""
    out = []
    for p in py_files():
        src = read(p)
        for i, line in enumerate(src.splitlines(), 1):
            if "time.time()" not in line:
                continue
            # A timestamp is fine. An interval is not: look for arithmetic.
            if re.search(r"time\.time\(\)\s*-|-\s*time\.time\(\)", line):
                out.append((rel(p), i, line.strip()[:88]))
    return out


# --------------------------------------------------- B: unreachable clear
def check_unreachable_clear():
    """A guard that blocks-and-returns, whose clear() is only on the path the
    guard prevents. Reported as RISK, not as a defect: the shape is correct
    when the guard is mutated elsewhere."""
    out = []
    for p in py_files():
        src = read(p)
        lines = src.splitlines()
        for i, line in enumerate(lines):
            m = re.search(r"\.block\((.+?)\)", line)
            if not m:
                continue
            nxt = " ".join(lines[i + 1:i + 3])
            if "return" not in nxt:
                continue
            key = m.group(1)
            cleared = any(".clear(%s" % key.split(",")[0].strip() in l
                          for l in lines)
            if not cleared:
                out.append((rel(p), i + 1, "block(%s) with no clear() anywhere"
                            % key[:40]))
    return out


# ------------------------------------------------- C: unprefixed resources
def check_unprefixed():
    """Resource names in C++ that are not built from a prefix. Five of these
    have bitten; the fifth was invisible to every URDF-level check."""
    out = []
    pat = re.compile(r'"(tcp/[a-z_.]+|reset_fault/?[a-z_]*|reactivate_\w+)"')
    for d, dirs, files in os.walk(os.path.join(ROOT, "src")):
        dirs[:] = [x for x in dirs if x not in SKIP_DIRS]
        for f in files:
            if not f.endswith((".cpp", ".hpp", ".h", ".cc")):
                continue
            p = os.path.join(d, f)
            src = read(p)
            for i, line in enumerate(src.splitlines(), 1):
                m = pat.search(line)
                if m and "prefix" not in line:
                    tag = ("VENDOR (patched via patches/)"
                           if "/ros2_" in rel(p) else "OURS")
                    out.append((rel(p), i, "%s  %s" % (m.group(1), tag)))
    return out


# ------------------------------------------------------- D: second stack
def check_second_stack():
    out = []
    for p in py_files():
        src = read(p)
        if "master_pose_node" not in src:
            continue
        lit = string_literal_lines(src)
        for i, line in enumerate(src.splitlines(), 1):
            if i in lit:
                continue
            stripped = line.strip()
            # A COMMENT OR A DOCSTRING EXAMPLE CANNOT START ANYTHING.
            # Both findings from the first run were documentation: a
            # comment citing a line number, and a usage example inside a
            # module docstring.
            if stripped[:1] in ("#", "*") or stripped[:3] in (
                    chr(34) * 3, chr(39) * 3):
                continue
            if re.search(r"(Popen|run|call|system|Node\()\s*\(?.*master_pose_node",
                         line) and "pgrep" not in line and "pkill" not in line:
                out.append((rel(p), i, line.strip()[:88]))
    return out


# --------------------------------------------------------- E: entry points
def check_entry_points():
    missing, unregistered = [], []
    for setup in subprocess.run(
            ["find", os.path.join(ROOT, "src"), "-name", "setup.py"],
            capture_output=True, text=True).stdout.split():
        pkg_dir = os.path.dirname(setup)
        src = read(setup)
        eps = re.findall(r"'([\w_]+)\s*=\s*([\w_.]+):(\w+)'", src)
        registered = set()
        for name, module, func in eps:
            registered.add(module.split(".")[-1])
            path = os.path.join(pkg_dir, *module.split(".")) + ".py"
            if not os.path.exists(path):
                missing.append((rel(setup), name, "%s missing" % module))
                continue
            if ("def %s(" % func) not in read(path):
                missing.append((rel(setup), name,
                                "%s has no %s()" % (module, func)))
        # modules with a main() that nobody registered
        pkg_name = os.path.basename(pkg_dir)
        inner = os.path.join(pkg_dir, pkg_name)
        if os.path.isdir(inner):
            for f in sorted(os.listdir(inner)):
                if not f.endswith(".py") or f == "__init__.py":
                    continue
                stem = f[:-3]
                if stem in registered:
                    continue
                if re.search(r"^def main\(", read(os.path.join(inner, f)),
                             re.M):
                    unregistered.append((pkg_name, stem))
    return missing, unregistered


# ------------------------------------------------------ F: stale references
def check_stale_paths():
    """Referenced repo paths that do not exist."""
    out = []
    pat = re.compile(r'["\']((?:scripts|src|config|docs|recordings)/[\w./*-]+)["\']')
    for p in list(py_files()) + [os.path.join(ROOT, "scripts", f)
                                 for f in os.listdir(os.path.join(ROOT, "scripts"))
                                 if f.endswith(".sh")]:
        src = read(p)
        for i, line in enumerate(src.splitlines(), 1):
            if line.strip().startswith("#"):
                continue
            for m in pat.finditer(line):
                # Strip sentence punctuation: a path quoted at the
                # end of a sentence carries the full stop.
                ref = m.group(1).rstrip(".,;:")
                if "*" in ref or "%" in ref or "{" in ref:
                    continue
                if not os.path.exists(os.path.join(ROOT, ref)):
                    out.append((rel(p), i, ref))
    return out


def check_stale_task_args():
    """Shell entry points invoked with task names the target does not accept.
    This is the class that made five GUI buttons do nothing."""
    out = []
    runner = os.path.join(ROOT, "scripts/run_experiment.sh")
    accepted = set()
    if os.path.exists(runner):
        accepted = set(re.findall(r"\b([et]\d)\b", read(runner)))
    if not accepted:
        return out
    for p in py_files():
        src = read(p)
        for i, line in enumerate(src.splitlines(), 1):
            if "run_experiment.sh" not in line:
                continue
            for tok in re.findall(r"\b([et]\d)\b", line):
                if tok not in accepted:
                    out.append((rel(p), i, "%s not accepted by run_experiment.sh"
                                % tok))
    return out


# ------------------------------------------------------------- G: dead code
def check_dead_code():
    """Module-level functions never referenced anywhere in the workspace."""
    defined = {}
    for p in py_files():
        try:
            tree = ast.parse(read(p))
        except SyntaxError:
            continue
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
                if node.name in ("main", "setup", "generate_launch_description"):
                    continue
                defined.setdefault(node.name, []).append((rel(p), node.lineno))
    blob = "\n".join(read(p) for p in py_files())
    out = []
    for name, sites in defined.items():
        # Count every MENTION, not every call: a function in a dispatch list
        # (FIGS = [fig_a, fig_b]) or passed as a callback is referenced without
        # parentheses, and counting calls alone reported 293 "dead" functions
        # of which most are live.
        if len(re.findall(r"\b%s\b" % re.escape(name), blob)) <= len(sites):
            for f, ln in sites:
                out.append((f, ln, name))
    return out


# ------------------------------------------- H: tests that pass on no data
def check_vacuous_tests():
    """all()/any() over a collection that can be empty, inside a check whose
    failure is a pass. This is how a known-answer test passed on zero rows."""
    out = []
    for p in py_files():
        if "/test" not in p and not os.path.basename(p).startswith(
                ("test_", "verify_", "measure_", "audit_")):
            continue
        src = read(p)
        lines = src.splitlines()
        for i, line in enumerate(lines, 1):
            m = re.search(r"\b(?:ok|passed|good|valid)\w*\s*=\s*all\(", line)
            if not m:
                continue
            window = "\n".join(lines[max(0, i - 12):i])
            if not re.search(r"if\s+not\s+\w+|len\(\w+\)\s*(==|<)\s*0|"
                             r"if\s+\w+\s*:", window):
                out.append((rel(p), i, line.strip()[:88]))
    return out


CHECKS = [
    ("A  wall-clock used as an interval", check_wallclock),
    ("B  block() with no clear() anywhere", check_unreachable_clear),
    ("C  unprefixed resource in C++", check_unprefixed),
    ("D  can start a second stack", check_second_stack),
    ("F1 referenced path does not exist", check_stale_paths),
    ("F2 task arg the runner rejects", check_stale_task_args),
    ("G  defined and never called", check_dead_code),
    ("H  check can pass on no data", check_vacuous_tests),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    results = {}
    print("=" * 78)
    print("WORKSPACE AUDIT -- report only, nothing is changed")
    print("=" * 78)
    print("scanned %d python files" % len(list(py_files())))

    for label, fn in CHECKS:
        rows = fn()
        results[label] = rows
        print("\n%s: %d" % (label, len(rows)))
        for r in rows[:14]:
            print("    %-46s %5s  %s" % (r[0], r[1], r[2]))
        if len(rows) > 14:
            print("    ... and %d more" % (len(rows) - 14))

    missing, unreg = check_entry_points()
    results["E1 entry point does not resolve"] = missing
    results["E2 module with main() not registered"] = unreg
    print("\nE1 entry point does not resolve: %d" % len(missing))
    for r in missing[:10]:
        print("    %-40s %-24s %s" % r)
    print("\nE2 module with main() not registered: %d" % len(unreg))
    for pkg, mod in unreg[:20]:
        print("    %-20s %s" % (pkg, mod))

    total = sum(len(v) for v in results.values())
    print("\n" + "=" * 78)
    print("TOTAL FINDINGS: %d" % total)
    if a.json:
        p = os.path.join(ROOT, "recordings/baselines/workspace_audit.json")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        json.dump({k: v for k, v in results.items()}, open(p, "w"), indent=2)
        print("  -> %s" % p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
