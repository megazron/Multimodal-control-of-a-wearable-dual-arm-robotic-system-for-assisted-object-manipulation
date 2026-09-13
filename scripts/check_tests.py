#!/usr/bin/env python3
"""Run the unit suite and refuse on any NEW failure.

    python3 scripts/check_tests.py [--quiet]

WHY THIS EXISTS. Two tests broke and went unnoticed for a whole session,
because the suite was not run: last session's T0 direction change left
test_t0_sampling asserting the OLD contract -- draws come from one 200 x 80 mm
band -- and it failed on a perfectly good FRONT_UP sample. Nothing said so
until the suite was run by hand for an unrelated reason.

TWO PRE-EXISTING FAILURES ARE ALLOWED BY NAME, and that is the whole design.
`test_flake8` and `test_pep257` have failed since long before this work -- the
package uses double quotes throughout, against the ROS style default -- and a
gate that goes red for them would be switched off within a day, which is worse
than no gate. They are listed, they are printed on every run as ALLOWED, and
ANY OTHER failure is a refusal. A gate that cries wolf gets disabled; a gate
that hides a known problem is a lie. Naming them does neither.

If one of the allowed two ever PASSES, that is also reported -- it means the
allowance is stale and should be removed, and an allowlist nobody prunes turns
into a blanket.
"""
import argparse
import os
import re
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Known-failing, by exact node id. Pre-existing and documented in docs/ENGINEERING_LOG.md.
ALLOWED = {
    "src/srl_teleop/test/test_flake8.py::test_flake8",
    "src/srl_teleop/test/test_pep257.py::test_pep257",
}


def run(paths=None, quiet=False):
    cmd = [sys.executable, "-m", "pytest", "-q",
           "--no-header", "-p", "no:cacheprovider"]
    # EVERY src/*/test, discovered rather than listed. A hardcoded pair ran
    # 288 tests where `src/*/test` runs 413, and a gate that silently covers
    # two thirds of the suite is the same class of defect it exists to catch.
    import glob as _glob
    cmd += paths or sorted(_glob.glob(os.path.join(WS, "src", "*", "test")))
    r = subprocess.run(cmd, cwd=WS, capture_output=True, text=True)
    out = r.stdout + r.stderr
    # A PYTEST NODE ID, NOT ANY LINE BEGINNING "FAILED ".
    #
    # pep257 and flake8 echo the SOURCE LINES they object to, so a source file
    # containing a line that starts with `FAILED ` at column 0 lands in this
    # output and was matched here. `src/srl_teleop/srl_teleop/vr_bringup.py`
    # has `FAILED = "failed"`, and this gate duly reported "1 test failing
    # that is not on the allowlist: =" -- a test that does not exist, on a
    # suite where every real test passed.
    #
    # A phantom failure in the gate is worse than a missed one: it is the
    # thing that decides whether the workspace is healthy, and a gate that
    # cries wolf gets bypassed. A real id always carries `::` or ends `.py`.
    failed = set(re.findall(r"^FAILED (\S+\.py(?:::\S+)?)", out, re.M))
    # pytest prints the id relative to the invocation directory
    failed = {f.split(" ")[0] for f in failed}
    # THE LAST MATCH, NOT THE FIRST. `re.search` found a "3 passed" earlier in
    # the output -- from a warnings block -- and reported 3 where the summary
    # line said 288. A count that is wrong by two orders of magnitude and
    # looks plausible is exactly the kind of number this project keeps
    # catching itself on.
    hits = re.findall(r"(\d+) passed", out)
    passed = int(hits[-1]) if hits else 0
    return failed, passed, out, r.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("paths", nargs="*")
    a = ap.parse_args()

    failed, passed, out, rc = run(a.paths or None, a.quiet)
    new = sorted(f for f in failed if f not in ALLOWED)
    stale = sorted(x for x in ALLOWED if x not in failed)

    print("[tests] %d passed, %d failed (%d allowed, %d NEW)"
          % (passed, len(failed), len(failed) - len(new), len(new)))

    # A COLLECTION ERROR IS NOT A CLEAN RUN, AND THIS GATE USED TO SAY IT WAS.
    #
    # `FAILED <id>` lines only exist for tests that RAN. When an import raises,
    # pytest prints `ERROR` and `Interrupted: 1 error during collection`, runs
    # nothing, and exits 2 -- so `failed` is empty, `passed` is 0, and this
    # printed "0 passed, 0 failed (0 allowed, 0 NEW)" and returned SUCCESS.
    # MEASURED on 2026-08-17, with `msc_clip_tasks` raising IndexError partway
    # through the T1 rebuild: the gate that stands in front of every recording
    # went green over a suite that had not executed a single test.
    #
    # This is the "a check that cannot fail is not a check" rule applied to the
    # thing that enforces it. Two independent conditions, because either alone
    # can be defeated: pytest's own exit code, and a run that executed nothing.
    if rc not in (0, 1):
        print("[tests] REFUSING: pytest exited %d -- the suite did not run to "
              "completion. Almost always a COLLECTION error: an import raised, "
              "so no test executed and 'nothing failed' means nothing was "
              "tried." % rc)
        tail = [ln for ln in out.splitlines() if ln.strip()][-25:]
        print("\n".join("[tests] | " + ln for ln in tail))
        return 2
    if passed == 0 and not failed:
        print("[tests] REFUSING: zero tests ran. An empty suite reports the "
              "same thing as a passing one and this gate is what stands in "
              "front of recording.")
        return 2
    for f in sorted(failed & ALLOWED):
        print("[tests]   allowed: %s" % f)
    if stale:
        print("[tests] STALE ALLOWANCE -- these now PASS and should be "
              "removed from ALLOWED: %s" % ", ".join(stale))
    if new:
        print("[tests] REFUSING: %d test(s) failing that are not on the "
              "allowlist:" % len(new))
        for f in new:
            print("[tests]   %s" % f)
        if not a.quiet:
            tail = [ln for ln in out.splitlines() if ln.strip()][-25:]
            print("\n".join("[tests] | " + ln for ln in tail))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
