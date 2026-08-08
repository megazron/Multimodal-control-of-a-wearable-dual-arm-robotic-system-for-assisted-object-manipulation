"""
home_positions.py -- load joint home positions from simple, editable text files
=================================================================================
File format (one per line, easy to open/edit in Notepad):
    joint_1: 62
    joint_2: -109
    ...
Values are in DEGREES (matching what RViz's joint panel shows, so you can
copy numbers straight from there). This module converts to radians for use
in ROS2 code.

Files live in ~/kortex_ws/config/home_positions_<arm>.txt -- from Windows,
that's reachable via the network path \\\\wsl$\\Ubuntu\\home\\<your-username>\\kortex_ws\\config\\
Just open either file in Notepad, edit the numbers, save -- no code
changes, no rebuild needed. Changes take effect next time the node starts.
"""
import math
import os
import re

CONFIG_DIR = os.path.expanduser("~/kortex_ws/config")


def load_home_degrees(arm):
    """Returns a list of 7 values in DEGREES, in joint_1..joint_7 order."""
    path = os.path.join(CONFIG_DIR, f"home_positions_{arm}.txt")
    values = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = re.match(r"joint_(\d+)\s*:\s*(-?\d+\.?\d*)", line)
            if m:
                idx = int(m.group(1))
                values[idx] = float(m.group(2))
    return [values[i] for i in range(1, 8)]


def load_home_radians(arm):
    """Same as load_home_degrees, converted to radians."""
    return [math.radians(d) for d in load_home_degrees(arm)]
