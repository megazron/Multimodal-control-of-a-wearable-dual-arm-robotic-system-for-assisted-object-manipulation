#!/usr/bin/env python3
"""Press the one button for real, end to end, and require READY TO OPERATE.

    python3 scripts/verify_vr_one_button.py
    python3 scripts/verify_vr_one_button.py --keep     # leave it running

WHY THIS EXISTS ALONGSIDE THE INJECTION TESTS. `test_vr_bringup_sequence.py`
drives every step against a `World` that is a description of a machine, which
is the right way to test the DECISIONS and cannot test the WIRING: that
`run_teleop.sh` is where the sequence thinks it is, that the bridge really
binds the port it is told to, that the mapper really publishes the topic the
step waits for. Those are four seams a fake cannot see, and a subsystem that
decides correctly and is wired to nothing is the failure this repository has
met most often.

So this runs the real thing, with nothing typed, and requires the sequence to
reach the end.

IT DOES NOT NEED A HEADSET. Everything up to and including the bridge is
machine-side. The one thing a headset supplies is controller poses, so the
final step reports NOT READY without one -- and that is the correct answer,
not a failure of the button. `--expect-no-headset` makes that explicit.
"""
import argparse
import json
import os
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src/srl_teleop"))

from srl_teleop import vr_bringup as V                       # noqa: E402

RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, bool(ok), detail))
    print("  %-38s %s   %s" % (name[:38], "PASS" if ok else "FAIL",
                               detail[:70]))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", action="store_true",
                    help="leave everything running afterwards")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--fixes", type=int, default=4,
                    help="how many offered repairs to press before giving up")
    a = ap.parse_args(argv)

    print("\n== THE ONE BUTTON, FOR REAL ==")
    print("  nothing is typed after this line\n")

    # THE WHOLE LOOP, INCLUDING THE REPAIRS.
    #
    # An operator who meets a red row presses the fix under it and presses the
    # button again -- so a verification that stops at the first failure is
    # verifying half of the thing. This presses each offered fix and restarts
    # the sequence, up to `--fixes` times, which is what "one button" actually
    # means in use.
    w = V.World(ws=WS)
    t0 = time.time()
    out = []
    applied = []
    for attempt in range(1, a.fixes + 2):
        out = []
        if attempt > 1:
            print("  -- retry %d --" % (attempt - 1))
        stopped = None
        for step in V.plan():
            res = step.run(w)
            out.append((step.key, res))
            mark = {V.OK: "ok", V.FAILED: "FAILED", V.UNKNOWN: "unknown",
                    V.SKIPPED: "skipped"}.get(res.state, res.state)
            print("  %-18s %-8s %s" % (step.key, mark, res.plain[:64]))
            if res.state == V.FAILED:
                stopped = (step.key, res)
                break
            if time.time() - t0 > a.timeout:
                print("      (over the %.0f s budget)" % a.timeout)
                stopped = None
                break
        if stopped is None:
            break
        key, res = stopped
        print("      it offers: %s" % (res.fix_label or "NOTHING"))
        if any(k == key for k, _f, _o, _m in applied):
            # THE SAME STEP FAILED AGAIN AFTER ITS OWN REPAIR. Pressing it a
            # third time is how a loop looks from the inside; stopping and
            # saying so is how it looks from outside.
            print("      STOPPING: this step already had its repair pressed "
                  "and failed again. The repair is not working, or it is "
                  "recreating the fault.")
            break
        if not res.has_fix or attempt > a.fixes:
            break
        if res.fix.startswith("show_") or res.fix.endswith("_help"):
            print("      (that fix only shows information; not retrying)")
            break
        fn = V.FIXES.get(res.fix)
        if fn is None:
            print("      NO SUCH REPAIR: %s" % res.fix)
            break
        ok, msg = fn(w)
        applied.append((key, res.fix, ok, msg))
        print("      pressed it: %s" % msg[:66])
        if not ok:
            break
    print()
    if applied:
        print("  repairs pressed on the way: %s"
              % ", ".join("%s(%s)" % (k, f) for k, f, _o, _m in applied))
        print()

    states = {k: r.state for k, r in out}
    vstate, vhead = V.verdict([r for _k, r in out], keyed=out)
    print("  verdict: %s -- %s\n" % (vstate, vhead))
    machine_side = ("environment", "leftover_memory", "daemon",
                    "second_stack", "sim", "certificate", "port", "bridge",
                    "mapper", "mapper_clean")
    for k in machine_side:
        check("machine-side step reached: %s" % k, k in states,
              states.get(k, "not reached"))
    for k in machine_side:
        if k in states:
            check("machine-side step passed: %s" % k, states[k] == V.OK,
                  states[k])

    # THE HEADSET IS THE ONLY THING MISSING, and it must be the only thing.
    ready = states.get("ready")
    r = dict(out).get("ready")
    plain = r.plain if r else ""
    if ready == V.OK:
        check("READY TO OPERATE", True, "controller poses are arriving")
    else:
        # WITH NO HEADSET THIS IS THE CORRECT ANSWER, and it must be the
        # headset it stops on and nothing else. It must NOT say ready: the
        # pose topic exists the moment the bridge does, and the last step
        # once passed on that alone.
        check("stops ONLY on the headset, and does not claim ready",
              ready == V.UNKNOWN and "not sending controller positions"
              in plain,
              plain[:64] or "ready step not reached")

    # Every failure must have offered a way forward.
    for k, r in out:
        if r.state == V.FAILED:
            check("failure offers a fix: %s" % k, r.has_fix,
                  r.fix_label or "none")

    # And nothing the operator reads may carry machinery.
    banned = ("/vr/", "/joint_states", "Traceback", "rclpy", "fastrtps")
    dirty = [(k, b) for k, r in out for b in banned if b in r.plain]
    check("no topic names in what is displayed", not dirty, str(dirty)[:60])

    print()
    n_ok = sum(1 for _n, ok, _d in RESULTS if ok)
    print("=" * 72)
    print("%d checks, %d PASS, %d FAIL  (%.0f s)"
          % (len(RESULTS), n_ok, len(RESULTS) - n_ok, time.time() - t0))
    for n, ok, d in RESULTS:
        if not ok:
            print("  FAILED %s: %s" % (n, d))

    outp = os.path.join(WS, "recordings", "baselines", "vr_one_button.json")
    os.makedirs(os.path.dirname(outp), exist_ok=True)
    with open(outp, "w") as fh:
        json.dump(dict(states=states, seconds=round(time.time() - t0, 1),
                       repairs=[[k, f, o, m] for k, f, o, m in applied],
                       checks=[[n, ok, d] for n, ok, d in RESULTS]), fh,
                  indent=1)
    print("wrote %s" % outp)

    if not a.keep:
        print("\nstopping what this started (leaving the simulation)")
        for name, p in w.started:
            if name == "simulation" or p is None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), 2)
            except Exception:                                 # noqa: BLE001
                pass
        time.sleep(2)
    return 0 if n_ok == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
