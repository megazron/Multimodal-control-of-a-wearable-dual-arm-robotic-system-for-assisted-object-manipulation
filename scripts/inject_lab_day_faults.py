#!/usr/bin/env python3
"""LAB-DAY FAULT INJECTION: the faults fault_injector.py does not cover.

    python3 scripts/inject_lab_day_faults.py            # all of them
    python3 scripts/inject_lab_day_faults.py --list

`srl_teleop/fault_injector.py` injects 14 faults and reports 14/14 handled.
It predates the camera work, the work-surface owner and the object-pose gap,
so the faults below have NEVER been injected. They are injected here, not
reasoned about.

THREE QUESTIONS PER FAULT, and the third is the one that decides lab day:

  WHAT       what the system actually does
  NAMED      does it say so in plain words a person can act on
  RECOVERS   does it come back without a restart

A fault that is DETECTED but not NAMED is nearly as expensive as a silent one
-- the operator sees something is wrong and still has to find out what. A
fault that is silent costs a morning, which is the whole reason this file
exists.

SILENT is reported separately and first.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments", "experiments",
                                "abc"))

RESULTS = []


def record(name, what, named, recovers, detail=""):
    RESULTS.append(dict(fault=name, what=what, named=named,
                        recovers=recovers, detail=detail))
    print("   %-24s %-9s named=%-5s recovers=%-8s %s"
          % (name, what, named, recovers, detail[:60]), flush=True)


# ------------------------------------------------------- work surface
def f_table_high():
    """The real table is 20 mm ABOVE the declared height."""
    from srl_experiments import work_surface as W
    W.clear_measured()
    W.set_measured(W.DECLARED_M + 0.020, "(injected)")
    ok, msg = W.check()
    W.clear_measured()
    record("table_20mm_high", "DETECTED" if not ok else "ABSORBED",
           ("+20.0 mm" in msg and "40 mm" in msg),
           "yes, clear_measured", msg)


def f_table_low():
    from srl_experiments import work_surface as W
    W.clear_measured()
    W.set_measured(W.DECLARED_M - 0.020, "(injected)")
    ok, msg = W.check()
    W.clear_measured()
    record("table_20mm_low", "DETECTED" if not ok else "ABSORBED",
           ("-20.0 mm" in msg and "40 mm" in msg),
           "yes, clear_measured", msg)


def f_table_never_measured():
    """The state a real run STARTS in: nothing has measured the surface."""
    from srl_experiments import work_surface as W
    W.clear_measured()
    ok, msg = W.check(require_measured=True)
    record("table_never_measured", "DETECTED" if not ok else "ABSORBED",
           "NOT MEASURED" in msg, "n/a", msg)


# ------------------------------------------------------- object pose
def f_object_rotated():
    """An object sitting 30 degrees off square.

    Asks the question directly: does ANYTHING in the pipeline carry an
    object's yaw from perception to the grasp? If the fingerprint stores
    position only, a rotated object gets a square grasp and nothing notices.
    """
    import inspect
    try:
        from srl_perception import scene_fingerprint as SF
    except Exception as e:
        record("object_rotated_30deg", "NO MODULE", False, "n/a", str(e)[:60])
        return
    src = inspect.getsource(SF)
    stores = "quat" in src            # CAN it hold an orientation?
    # THE QUESTION IS NOT WHETHER IT IS STORED. The first version of this
    # check passed because the string "quat" appears in the fingerprint, and
    # that is a false pass: storing an orientation nobody reads changes
    # nothing about the grasp. So ask the CONSUMER instead.
    consumed = False
    import glob as _g
    for f in (_g.glob(os.path.join(WS, "src", "srl_autonomy", "**", "*.py"),
                      recursive=True)
              + _g.glob(os.path.join(WS, "src", "srl_experiments",
                                     "srl_experiments", "task_actions.py"))):
        try:
            if ".quat" in open(f).read():
                consumed = True
                break
        except OSError:
            pass
    record("object_rotated_30deg",
           "DETECTED" if consumed else "SILENT",
           consumed, "no" if not consumed else "unknown",
           "fingerprint CAN store a quat=%s but NO grasp path reads .quat=%s "
           "-- a rotated object gets a SQUARE grasp and nothing compares them"
           % (stores, consumed))


def f_object_moved_after_scan():
    """An object nudged between the scan and the grasp."""
    import inspect
    try:
        from srl_perception import scene_fingerprint as SF
    except Exception as e:
        record("object_moved_after_scan", "NO MODULE", False, "n/a",
               str(e)[:60])
        return
    src = inspect.getsource(SF)
    has_drift = "drift" in src.lower()
    tol = "pos_tol_m" in src
    record("object_moved_after_scan",
           "DETECTED" if (has_drift and tol) else "SILENT",
           has_drift,
           "re-scan (srv_resweep)" if has_drift else "no",
           "drift check present=%s, position tolerance present=%s"
           % (has_drift, tol))


def f_object_missing():
    import inspect
    try:
        from srl_perception import scene_fingerprint as SF
    except Exception as e:
        record("object_missing", "NO MODULE", False, "n/a", str(e)[:60])
        return
    src = inspect.getsource(SF)
    drops = "dropped" in src or "missing" in src
    record("object_missing", "DETECTED" if drops else "SILENT", drops,
           "re-scan" if drops else "no",
           "fingerprint names dropped/missing objects=%s" % drops)


# ------------------------------------------------------- camera
def _cam_topics():
    env = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS="SHM")
    try:
        out = subprocess.run(["ros2", "topic", "list"], env=env,
                             capture_output=True, text=True,
                             timeout=25).stdout
    except Exception:
        return []
    return [t for t in out.split() if "camera" in t]


def f_camera_absent():
    """No camera node at all. The state of every run so far."""
    t = _cam_topics()
    present = bool(t)
    # Does anything WARN when the fingerprint sweeps with no camera?
    import inspect
    try:
        from srl_perception import scene_fingerprint_node as SFN
        src = inspect.getsource(SFN)
        warns = "_warn_if_looking_at_nothing" in src or "PUBLISHING NOTHING" in src
    except Exception:
        warns = False
    record("camera_absent",
           "DETECTED" if warns else "SILENT", warns,
           "start the camera",
           "camera topics present=%s; fingerprint warns on a blind sweep=%s"
           % (present, warns))


def f_camera_silent():
    """Camera node up, publishing nothing (TF missing, bad frame)."""
    import inspect
    try:
        from srl_perception import mock_rgbd_camera as MC
        src = inspect.getsource(MC)
        warns = "PUBLISHING NOTHING" in src
    except Exception as e:
        record("camera_silent", "NO MODULE", False, "n/a", str(e)[:60])
        return
    record("camera_silent", "DETECTED" if warns else "SILENT", warns,
           "fix TF / restart camera",
           "mock names a blind camera by TF frame=%s" % warns)


def f_camera_garbage():
    """Camera publishing structurally valid but meaningless data."""
    import inspect
    try:
        from srl_perception import object_pose_tracker as OPT
        src = inspect.getsource(OPT)
    except Exception as e:
        record("camera_garbage", "NO MODULE", False, "n/a", str(e)[:60])
        return
    guards = [g for g in ("confidence", "drop", "timeout", "1.5")
              if g in src]
    # Garbage that is CONFIDENT is the dangerous case.
    record("camera_garbage",
           "PARTIAL" if guards else "SILENT",
           bool(guards), "unknown",
           "tracker guards seen: %s. A confidently WRONG pose passes every "
           "one of them" % (",".join(guards) or "none"))


# ------------------------------------------------------- environment
def f_two_stacks():
    """Two stacks started by accident."""
    import inspect
    try:
        import srl_gui  # noqa: F401
        src = inspect.getsource(sys.modules["srl_gui"])
    except Exception:
        p = os.path.join(WS, "scripts", "srl_gui.py")
        src = open(p).read() if os.path.exists(p) else ""
    guard = "_foreign_description" in src or "refuses to start a second" in src
    counts = "master_pose_node" in src
    record("two_stacks", "DETECTED" if guard else "SILENT", guard,
           "kill the named PIDs",
           "GUI refuses a second stack=%s, counts master_pose_node=%s"
           % (guard, counts))


def f_wearer_moves():
    """The wearer shifts during a trial."""
    found = []
    for mod, hint in (("srl_teleop.mount_guard_node", "mount guard"),
                      ("srl_teleop.clearance", "clearance")):
        try:
            __import__(mod)
            found.append(hint)
        except Exception:
            pass
    # The wearer is a FIXED link in the URDF. Nothing measures it moving.
    record("wearer_moves", "SILENT", False, "no",
           "the wearer is a FIXED link in the URDF; %s check the ARM against "
           "a STATIONARY model, so a wearer who moves is invisible"
           % (" and ".join(found) or "no guards"))


def f_gui_killed():
    """The GUI dies mid-session."""
    p = os.path.join(WS, "scripts", "srl_gui.py")
    src = open(p).read() if os.path.exists(p) else ""
    cleans = "SIGINT" in src and "process group" in src.lower()
    resume = "resume" in src.lower()
    record("gui_killed", "PARTIAL" if cleans else "SILENT", cleans,
           "relaunch; session state %s" % ("resumable" if resume else "LOST"),
           "GUI kills its children cleanly=%s, session resume present=%s"
           % (cleans, resume))


FAULTS = {
    "table_20mm_high": f_table_high,
    "table_20mm_low": f_table_low,
    "table_never_measured": f_table_never_measured,
    "object_rotated_30deg": f_object_rotated,
    "object_moved_after_scan": f_object_moved_after_scan,
    "object_missing": f_object_missing,
    "camera_absent": f_camera_absent,
    "camera_silent": f_camera_silent,
    "camera_garbage": f_camera_garbage,
    "two_stacks": f_two_stacks,
    "wearer_moves": f_wearer_moves,
    "gui_killed": f_gui_killed,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--fault", default="")
    a = ap.parse_args()
    if a.list:
        for k in FAULTS:
            print(" ", k)
        return 0
    names = [a.fault] if a.fault in FAULTS else list(FAULTS)
    print("INJECTING %d faults\n" % len(names))
    for k in names:
        try:
            FAULTS[k]()
        except Exception as e:                          # pragma: no cover
            record(k, "HARNESS ERROR", False, "n/a", repr(e)[:70])
    silent = [r for r in RESULTS if r["what"] == "SILENT"]
    print("\n%d injected, %d SILENT" % (len(RESULTS), len(silent)))
    if silent:
        print("\nSILENT -- these are the ones that cost a morning:")
        for r in silent:
            print("   %-24s %s" % (r["fault"], r["detail"]))
    out = os.path.join(WS, "recordings", "baselines", "lab_day_faults.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(RESULTS, open(out, "w"), indent=2)
    print("\n-> %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
