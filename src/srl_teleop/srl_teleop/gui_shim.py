#!/usr/bin/env python3
"""Entry point for the Qt operations GUI, which lives in scripts/.

The GUI is kept in scripts/ because it is a tool rather than a node: it holds
no algorithm the rest of the system depends on, and keeping it out of the
package stops anything importing it by accident.
"""
import os
import runpy
import sys


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    for up in range(2, 7):
        p = os.path.join(here, *([".."] * up), "scripts", "srl_gui.py")
        p = os.path.normpath(p)
        if os.path.exists(p):
            sys.argv = [p] + list(argv or sys.argv[1:])
            runpy.run_path(p, run_name="__main__")
            return 0
    print("scripts/srl_gui.py not found next to the workspace")
    return 2


if __name__ == "__main__":
    sys.exit(main())
