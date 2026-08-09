#!/usr/bin/env python3
"""Every module-like name a node uses must actually be imported.

WHY THIS EXISTS. `ik_follower_node` called `time.monotonic()` in the autonomy
path (lines 713 and 733) and the module had no `import time`. The name is only
evaluated when the callback RUNS, so the node imported cleanly, launched
cleanly, logged "Tracking enabled", and then died with

    NameError: name 'time' is not defined

on the first `/autonomy/assist_pose_<arm>` message it ever received. The
followers launch without `respawn`, so it stayed dead for the rest of the
session. That is the entire reason "autonomy does not move the arm" measured as
0 trajectories for two sessions.

WHY NOT A LIVE TEST. The obvious test -- bring up a node and publish a pose --
is the bug class this project keeps paying for: it needs `/compute_ik`, tf, a
controller and home files, so it constructs an elaborate environment and then
tests it. This test reads the SHIPPED SOURCE and needs no environment at all.
It is also not specific to `time` or to one file: it checks every module in
every `srl_*` package, so the next missing import is caught by the same test
rather than by a dead node in the lab.

NEGATIVE CONTROL. Verified to FAIL on the pre-fix `ik_follower_node.py`
(removing `import time` reproduces the NameError as a test failure) and to
pass after. A test nobody has watched fail proves nothing.
"""
import ast
import builtins
import os
import unittest

WS = os.path.expanduser("~/kortex_ws/src")

# Names that are legitimately bound by something this analysis cannot see:
# star-imports, conditional imports inside try/except, and module __getattr__.
# Keep this list SHORT and justified -- every entry is a hole in the check.
ALLOWED_UNRESOLVED = {
    "self",       # bound by the method signature, seen via ast.arg already
    "cls",
    "__builtins__",
}


def _pkg_modules():
    """Every .py under a src/srl_*/srl_*/ package directory."""
    out = []
    for pkg in sorted(os.listdir(WS)):
        if not pkg.startswith("srl_"):
            continue
        inner = os.path.join(WS, pkg, pkg)
        if not os.path.isdir(inner):
            continue
        for root, _dirs, files in os.walk(inner):
            for f in sorted(files):
                if f.endswith(".py"):
                    out.append(os.path.join(root, f))
    return out


def unresolved_module_names(path):
    """Names used as `<name>.<attr>` that are never imported or bound.

    This is deliberately conservative: it only flags a name when it is used in
    ATTRIBUTE position (`time.monotonic`), which is what a module reference
    looks like, and it treats any binding anywhere in the file as sufficient.
    A false positive here would be worse than a miss, because a test that cries
    wolf gets deleted.
    """
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)

    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                imported.add((a.asname or a.name).split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            for a in node.names:
                imported.add(a.asname or a.name)

    bound = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            bound.add(node.id)
        elif isinstance(node, ast.arg):
            bound.add(node.arg)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef)):
            bound.add(node.name)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bound.add(node.name)
        elif isinstance(node, ast.Global):
            bound.update(node.names)

    used = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            used.add(node.value.id)

    known = imported | bound | set(dir(builtins)) | ALLOWED_UNRESOLVED
    return sorted(n for n in used if n not in known)


class TestNoUnresolvedModuleNames(unittest.TestCase):

    def test_every_srl_module_resolves_its_module_references(self):
        offenders = {}
        for path in _pkg_modules():
            bad = unresolved_module_names(path)
            if bad:
                offenders[os.path.relpath(path, WS)] = bad
        self.assertEqual(
            offenders, {},
            "module-like names used but never imported -- these raise "
            "NameError only when the code path RUNS, which is how a node "
            "launches cleanly and dies on its first message:\n" +
            "\n".join("  %s: %s" % (k, ", ".join(v))
                      for k, v in sorted(offenders.items())))

    def test_ik_follower_imports_time(self):
        """The specific regression, named, so its absence is unmissable.

        The general test above would also catch this, but a generic failure
        message does not tell the next reader WHY the project cares. This one
        names the incident.
        """
        path = os.path.join(WS, "srl_teleop", "srl_teleop",
                            "ik_follower_node.py")
        with open(path) as fh:
            tree = ast.parse(fh.read())
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    names.add(a.name.split(".")[0])
        self.assertIn(
            "time", names,
            "ik_follower_node uses time.monotonic() in on_autonomy_pose and "
            "autonomy_is_driving. Without the import the first autonomy pose "
            "kills the follower with NameError and it does not respawn.")

    def test_the_check_actually_detects_a_missing_import(self):
        """Negative control for the CHECK ITSELF.

        A structural test that cannot fail is decoration. This constructs the
        pre-fix condition in a scratch module and asserts the analysis reports
        it -- so if a future refactor makes unresolved_module_names() blind,
        this fails rather than the whole suite quietly passing.
        """
        import tempfile
        src = ("def f():\n"
               "    return time.monotonic()\n")
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
            fh.write(src)
            tmp = fh.name
        try:
            self.assertIn("time", unresolved_module_names(tmp))
        finally:
            os.unlink(tmp)

    def test_the_check_does_not_flag_a_correct_module(self):
        """The other half of the control: no false positive on good source."""
        import tempfile
        src = ("import time\n"
               "def f():\n"
               "    return time.monotonic()\n")
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as fh:
            fh.write(src)
            tmp = fh.name
        try:
            self.assertEqual([], unresolved_module_names(tmp))
        finally:
            os.unlink(tmp)


if __name__ == "__main__":
    unittest.main()
