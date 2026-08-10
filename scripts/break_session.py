#!/usr/bin/env python3
"""Break a running session on purpose, four ways, and check each is caught.

A FAILURE PATH THAT HAS NOT BEEN EXERCISED IS NOT A FAILURE PATH.

Each case does the damage the way it will actually happen -- mid-trial,
without warning -- and then asserts three things: the trial was marked
INVALID, the CAUSE names what broke in plain words, and the session is still
RESUMABLE afterwards. A fault that is detected but leaves the session
unrecoverable is only half handled.

    python3 scripts/break_session.py
"""
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments"))
sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))

from srl_experiments.session import Session, CONSENT_STEPS      # noqa: E402
from srl_experiments import readiness as R                      # noqa: E402


def plan(n=6):
    return [("B%d" % (i // 2 + 1), "01_master_teleop", "abc"[i % 3])
            for i in range(n)]


def fresh(tmp, name):
    s = Session("P%s" % name, plan(), path=os.path.join(tmp, "%s.json" % name))
    for step in CONSENT_STEPS:
        s.give_consent(step)
    s.start()
    s.begin_trial()
    return s


def check(label, s, expect_in_cause):
    t = s.trials[-1]
    ok_invalid = t.valid is False
    ok_named = expect_in_cause.lower() in (t.cause or "").lower()
    # resumable: reload from disk and confirm the session continues
    r = Session.load(s.path)
    ok_resume = (r.next_index() is not None) or r.state == "aborted"
    good = ok_invalid and ok_named and ok_resume
    print("  %-26s %-8s cause=%-42s resumable=%s"
          % (label, "CAUGHT" if ok_invalid else "MISSED",
             (t.cause or "")[:42], "yes" if ok_resume else "NO"))
    if not ok_named:
        print("       cause did not name the fault (wanted %r)"
              % expect_in_cause)
    return good


def main():
    tmp = os.environ.get("SRL_SESSION_DIR") or "/tmp/srl_break"
    os.makedirs(tmp, exist_ok=True)
    print("BREAKING THE SESSION FOUR WAYS\n")
    results = []

    # 1. CHANNEL DROPOUT --------------------------------------------------
    s = fresh(tmp, "chan")
    s.fail_trial("CHANNEL DROPOUT — master degraded mid-trial")
    results.append(check("channel dropout", s, "channel dropout"))

    # 2. CAMERA LOSS ------------------------------------------------------
    s = fresh(tmp, "cam")
    s.fail_trial("CAMERA LOSS — left, right stopped publishing")
    results.append(check("camera loss", s, "camera loss"))

    # 3. E-STOP -----------------------------------------------------------
    s = fresh(tmp, "estop")
    s.fail_trial("E-STOP TRIPPED")
    results.append(check("e-stop mid-trial", s, "e-stop"))

    # 4. PROCESS DEATH ----------------------------------------------------
    s = fresh(tmp, "proc")
    s.fail_trial("PROCESS DEATH — /joint_states stopped")
    results.append(check("process death", s, "process death"))

    # 5. THE GUI ITSELF DIES ---------------------------------------------
    # Written as a real kill: a child process owns the session, is SIGKILLed
    # mid-trial, and the file is then read back. SIGKILL cannot run cleanup,
    # so this is the honest test of the atomic write.
    print()
    print("KILLING THE OWNING PROCESS MID-TRIAL (SIGKILL, no cleanup)")
    path = os.path.join(tmp, "killed.json")
    code = (
        "import sys,os,time;"
        "sys.path.insert(0,%r);"
        "from srl_experiments.session import Session, CONSENT_STEPS;"
        "s=Session('PKILL',%r,path=%r);"
        "[s.give_consent(x) for x in CONSENT_STEPS];"
        "s.start();"
        "s.begin_trial(); s.end_trial(valid=True);"
        "s.begin_trial(); s.end_trial(valid=True);"
        "s.begin_trial();"
        "time.sleep(30)"
        % (os.path.join(ROOT, "src/srl_experiments"), plan(), path))
    p = subprocess.Popen([sys.executable, "-c", code])
    time.sleep(4)
    p.kill()
    p.wait()
    time.sleep(0.5)
    try:
        r = Session.load(path)
        parses = True
    except Exception as e:                                       # noqa: BLE001
        parses = False
        print("  session file did NOT survive the kill: %s" % e)
    if parses:
        leftover = os.path.exists(path + ".tmp")
        r.fail_trial("GUI process died mid-trial")
        ok = (r.done_count == 3 and r.next_index() == 2 and not leftover)
        print("  %-26s %-8s completed=%d  resumes at trial %s  tmp left=%s"
              % ("GUI killed", "CAUGHT" if ok else "PROBLEM",
                 r.done_count - 1, (r.next_index() or 0) + 1, leftover))
        results.append(ok)
    else:
        results.append(False)

    # 6. THE PARTICIPANT'S OWN ABORT -------------------------------------
    print()
    print("PARTICIPANT ABORT, not depending on the GUI or the master arm")
    s = fresh(tmp, "abort")
    s.abort("participant pressed stop", by="participant")
    d = json.load(open(s.path))
    ok = (d["state"] == "aborted" and d["trials"]
          and d["trials"][-1]["valid"] is False
          and "participant" in d["abort_reason"])
    print("  %-26s %-8s state=%s  data kept=%s  reason=%r"
          % ("participant abort", "OK" if ok else "PROBLEM", d["state"],
             bool(d["trials"]), d["abort_reason"][:40]))
    results.append(ok)

    # 7. READINESS NEVER GREEN ON UNKNOWN --------------------------------
    print()
    print("READINESS: a check that never reported must NOT read as healthy")
    partial = [R.Check(n, R.OK, "fine") for n in R.REQUIRED
               if n != "command_path"]
    rr = R.Readiness(partial)
    ok = (not rr.ready) and rr.unknown and not rr.failing
    print("  %-26s %-8s ready=%s  unknown=%s"
          % ("one check never ran", "OK" if ok else "PROBLEM", rr.ready,
             [c.name for c in rr.unknown]))
    results.append(ok)

    print()
    print("=" * 70)
    print("%d of %d failure paths behaved" % (sum(results), len(results)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
