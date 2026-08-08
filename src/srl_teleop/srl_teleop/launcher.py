#!/usr/bin/env python3
"""`ros2 run srl_teleop launcher` -> scripts/srl_launcher.py.

A thin shim so the GUI is reachable the same way as every other node in this
package, without duplicating it into the install tree where it would drift
from the copy under scripts/.
"""
import os
import runpy
import sys


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    for root in (os.path.abspath(os.path.join(here, "..", "..", "..", "..")),
                 os.path.expanduser("~/kortex_ws")):
        p = os.path.join(root, "scripts", "srl_launcher.py")
        if os.path.exists(p):
            sys.argv = [p] + list(argv or sys.argv[1:])
            runpy.run_path(p, run_name="__main__")
            return 0
    sys.stderr.write("scripts/srl_launcher.py not found.\n"
                     "Run it directly: python3 ~/kortex_ws/scripts/"
                     "srl_launcher.py\n")
    return 2


if __name__ == "__main__":
    sys.exit(main())
