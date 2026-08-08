#!/usr/bin/env python3
"""`ros2 run srl_teleop console` -> scripts/srl_console.py (Dear PyGui)."""
import os
import runpy
import sys


def main(argv=None):
    for root in (os.path.expanduser("~/kortex_ws"),
                 os.path.abspath(os.path.join(
                     os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..", ".."))):
        p = os.path.join(root, "scripts", "srl_console.py")
        if os.path.exists(p):
            sys.argv = [p] + list(argv or sys.argv[1:])
            runpy.run_path(p, run_name="__main__")
            return 0
    sys.stderr.write("scripts/srl_console.py not found\n")
    return 2


if __name__ == "__main__":
    sys.exit(main())
