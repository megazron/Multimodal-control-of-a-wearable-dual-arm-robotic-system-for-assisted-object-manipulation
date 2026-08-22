#!/usr/bin/env python3
"""Are the things this workspace depends on actually present and consistent?

    python3 -m srl_teleop.dependency_check          # the report
    python3 -m srl_teleop.dependency_check --json

WHY THIS EXISTS, and it is a recent scar. On 2026-08-22 `pip install lerobot`
was run into `.venv_vision` to try the dataset format. lerobot requires
numpy>=2; the vision venv's scipy is the SYSTEM scipy, which requires
numpy<1.28. The install succeeded, printed a dependency-conflict warning that
scrolled past, and left `scripts/real_calibration/check_all.py` failing 2 of 4
modules with

    ImportError: cannot import name 'Inf' from 'numpy'

The calibration gate is the thing that is supposed to be run before touching
hardware. It broke, and the only way to find out was to run it.

So: a check that a set of packages IMPORT TOGETHER, per environment, and says
which environment it is talking about. Not "is numpy installed" -- every
environment has numpy -- but "does scipy still import in the venv the
calibration scripts run in".

WHAT IT WILL NOT DO. It does not install anything and it does not repair
anything. Every row is a fact and, where the fact is bad, the command that
fixes it. A checker that silently pip-installs is a checker that can break a
working environment while reporting success.

THE VENVS ARE SEPARATE ON PURPOSE and this file is where that is written
down:

    .venv_vision    numpy<2 + SYSTEM scipy + ultralytics + pyrealsense2.
                    This is what scripts/real_calibration/ runs in.
    .venv_lerobot   numpy>=2 + torch. Isolated BECAUSE of the incident above.
    .venv_pose      MediaPipe, which also pulls numpy 2.x.
    system          rclpy, PyQt5, and the GUI itself.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys

WS = os.environ.get("SRL_WS") or os.path.expanduser("~/kortex_ws")

OK, WARN, BAD = "ok", "warn", "bad"

# (name, python, [modules that must import TOGETHER], why it matters)
ENVS = [
    ("system", sys.executable,
     # ruckig is in this list because `ik_follower_node` runs in THIS
     # interpreter and this is where it looks for it. It has NO dependencies
     # at all -- one compiled .so -- so it cannot move numpy, which is why it
     # is installed here rather than in a venv the nodes could not reach:
     #     pip install --user --no-deps --break-system-packages ruckig
     # Without it the follower does not fail: `motion_generator:=auto`
     # degrades to the synchronised clamp and says so on /ik_status_<arm>.
     # It stops being jerk-limited, which is why this row exists.
     ["rclpy", "PyQt5.QtWidgets", "numpy", "yaml", "ruckig"],
     "the GUI and every ROS node. ruckig is the teleop motion generator"),
    (".venv_vision", os.path.join(WS, ".venv_vision/bin/python"),
     ["numpy", "scipy.optimize", "cv2", "torch", "ultralytics"],
     "scripts/real_calibration/ and the detector. scipy here is the SYSTEM "
     "one and needs numpy<1.28"),
    (".venv_lerobot", os.path.join(WS, ".venv_lerobot/bin/python"),
     ["numpy", "torch", "lerobot.datasets.lerobot_dataset"],
     "the LeRobotDataset exporter. Needs numpy>=2, which is why it is not "
     "in .venv_vision"),
    (".venv_pose", os.path.join(WS, ".venv_pose/bin/python"),
     ["mediapipe", "numpy"],
     "the wearer tracker"),
]

# Command-line tools, and what stops working without each.
TOOLS = [
    ("rviz2", "the COMMANDED panel and every recorded clip"),
    ("ros2", "every launch this window makes"),
    ("xwininfo", "embedding RViz -- without it the panel falls back to a "
                 "separate window"),
    ("xprop", "detecting whether a window manager is running"),
    ("ffmpeg", "recording clips"),
    ("colcon", "building the workspace"),
]

# In-repo packages. Symlink-installed, so a .py edit takes effect on node
# RESTART -- if a change seems not to apply, suspect a stale PROCESS.
SRL_PACKAGES = ["srl_teleop", "srl_perception", "srl_autonomy",
                "srl_experiments", "srl_description", "srl_moveit_config"]


def _probe(python, modules, timeout=90):
    """Import every module in ONE interpreter and report versions.

    Together, not one at a time: the failure this file exists for is a pair
    that cannot coexist, and importing them separately would pass.
    """
    if not os.path.exists(python):
        return None, "no interpreter at %s" % python
    code = (
        "import json,sys\n"
        "out={}\n"
        "for m in %r:\n"
        "    try:\n"
        "        mod=__import__(m, fromlist=['x'])\n"
        "        out[m]=getattr(mod,'__version__','present')\n"
        "    except Exception as e:\n"
        "        out[m]='FAILED: %%s' %% e\n"
        "print(json.dumps(out))\n" % (modules,))
    try:
        p = subprocess.run([python, "-c", code], capture_output=True,
                           text=True, timeout=timeout)
    except Exception as e:                                    # noqa: BLE001
        return None, "could not run %s: %s" % (python, e)
    if p.returncode != 0:
        return None, (p.stderr or "").strip().splitlines()[-1:] and \
            (p.stderr or "").strip().splitlines()[-1][:120] or "exit %d" % p.returncode
    try:
        return json.loads(p.stdout.strip().splitlines()[-1]), ""
    except Exception:                                         # noqa: BLE001
        return None, (p.stdout or "")[:120]


def check_environments():
    rows = []
    for name, python, mods, why in ENVS:
        got, err = _probe(python, mods)
        if got is None:
            # A MISSING VENV IS NOT A FAILURE. Three of the four are optional
            # and the workspace runs without them; saying "bad" would train
            # the operator to ignore this panel.
            state = BAD if name == "system" else WARN
            rows.append({"env": name, "state": state,
                         "detail": err or "not built",
                         "why": why,
                         "fix": ("" if name == "system" else
                                 "python3 -m venv %s && %s/bin/pip install ..."
                                 % (name, name))})
            continue
        bad = {m: v for m, v in got.items() if str(v).startswith("FAILED")}
        rows.append({
            "env": name,
            "state": BAD if bad else OK,
            "detail": ("; ".join("%s %s" % (m, v) for m, v in bad.items())
                       if bad else
                       ", ".join("%s %s" % (m.split(".")[0], v)
                                 for m, v in got.items())),
            "why": why,
            "fix": ("these modules do not import TOGETHER in this "
                    "environment. Do NOT pip install into it to fix one -- "
                    "that is how it broke. See docs/system/20_lerobot.md."
                    if bad else ""),
        })
    return rows


def check_tools():
    rows = []
    for tool, why in TOOLS:
        path = shutil.which(tool)
        rows.append({"env": tool, "state": OK if path else BAD,
                     "detail": path or "not on PATH",
                     "why": why,
                     "fix": "" if path else
                            "source /opt/ros/jazzy/setup.bash, or apt install"})
    return rows


def check_srl_packages():
    """Every in-repo package must be importable AND symlink-installed.

    A COPY install is the trap: `.py` edits then have no effect and the
    symptom is a change that "did not apply", which reads as a stale process.
    """
    rows = []
    for pkg in SRL_PACKAGES:
        share = os.path.join(WS, "install", pkg)
        if not os.path.isdir(share):
            rows.append({"env": pkg, "state": BAD, "detail": "not built",
                         "why": "colcon build --symlink-install",
                         "fix": "colcon build --symlink-install"})
            continue
        # Symlink-installed packages have a symlink somewhere under install/.
        linked = False
        for root, dirs, files in os.walk(share):
            for n in list(dirs) + list(files):
                if os.path.islink(os.path.join(root, n)):
                    linked = True
                    break
            if linked:
                break
        rows.append({
            "env": pkg, "state": OK if linked else WARN,
            "detail": "symlink-installed" if linked else "COPY install",
            "why": "a copy install means .py edits do not take effect and "
                   "the symptom looks like a stale process",
            "fix": "" if linked else "colcon build --symlink-install",
        })
    return rows


def report():
    out = {"workspace": WS,
           "environments": check_environments(),
           "tools": check_tools(),
           "packages": check_srl_packages()}
    bad = [r for g in out.values() if isinstance(g, list)
           for r in g if r["state"] == BAD]
    out["ok"] = not bad
    out["n_bad"] = len(bad)
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    rep = report()
    if a.json:
        print(json.dumps(rep, indent=1))
        return 0 if rep["ok"] else 1
    print("workspace %s\n" % rep["workspace"])
    for group in ("environments", "tools", "packages"):
        print("-- %s" % group)
        for r in rep[group]:
            mark = {OK: "ok  ", WARN: "warn", BAD: "BAD "}[r["state"]]
            print("  %s %-14s %s" % (mark, r["env"], r["detail"][:96]))
            if r["state"] != OK and r["fix"]:
                print("       fix: %s" % r["fix"][:96])
        print()
    print("%d problem(s)" % rep["n_bad"])
    return 0 if rep["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
