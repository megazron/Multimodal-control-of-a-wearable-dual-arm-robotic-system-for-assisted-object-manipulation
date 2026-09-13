#!/usr/bin/env python3
"""Every self-test in the calibration rig, in one command.

    python3 scripts/real_calibration/check_all.py

Run this BEFORE any session that moves an arm. Each module here refuses to be
trusted on assertion alone: every one of them must recover a known answer AND
must fail on a deliberately broken input, because a check that cannot fail is
not a check -- docs/ENGINEERING_LOG.md's standing rule, which has been the fault seventeen
times in this project and three more on 2026-08-21.

What each one proves:

  safe_motion       densification covers the path; a sweep into the wearer is
                    REFUSED; and a move whose two ENDPOINTS are both clear
                    while its middle is not is also refused -- the exact hole
                    that let a pick command a path through the mannequin.
  arm_ik            solves points known to be reachable to ~0 mm on both arms,
                    and refuses a point 5 m away while SAYING it searched.
  scene_cameras     the 180 deg mount is handled on both the image and the
                    principal point, and using the native cx on a rotated
                    image is shown to displace a ray by 69 mm at 2.5 m --
                    wrong, but not obviously wrong.
  solve_extrinsic   recovers a known camera pose from rendered depth, and
                    scores a deliberately wrong pose as wrong.
"""
from __future__ import annotations

import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

MODULES = [
    ("safe_motion", "path checking and refusal"),
    ("arm_ik", "seeded inverse kinematics"),
    ("scene_cameras", "camera geometry and the 180 deg mount"),
    ("solve_extrinsic", "camera -> robot base"),
]


def main():
    print("=" * 72)
    print("REAL CALIBRATION -- SELF-TESTS")
    print("=" * 72)
    failed = []
    for name, what in MODULES:
        print("\n--- %s : %s" % (name, what))
        try:
            mod = __import__(name)
            mod.self_test(verbose=True)
        except Exception as exc:                              # noqa: BLE001
            failed.append((name, exc))
            print("FAILED: %s" % exc)
            traceback.print_exc(limit=2)
    print("\n" + "=" * 72)
    if failed:
        print("%d of %d MODULES FAILED" % (len(failed), len(MODULES)))
        for n, e in failed:
            print("   %-16s %s" % (n, str(e)[:90]))
        print("Do not run a live session on a rig whose checks do not pass.")
        return 1
    print("ALL %d MODULES PASSED -- the checks themselves are sound."
          % len(MODULES))
    print("This says the INSTRUMENTS work. It says nothing about the arms.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
