#!/usr/bin/env python3
"""RViz's REAL top-level X windows. ONE source, used by the GUI and by the
one-button VR bring-up.

THE DEFECT THIS ENCODES, measured 2026-08-22 on WSLg. The GUI matched any
line of `xwininfo -root -tree` containing "rviz", and rviz2 creates several
X windows besides the one it draws in:

    0x600106 "....rviz - RViz": ("rviz2" "rviz2")  1600x1000   <- real
    0x600004 "Qt Selection Owner for rviz2": ()     549x819    <- not
    0x600008 "rviz2": ()                            1x1        <- not
    0xa00004 (has no name): ()                    1568x914     <- child

The caller then took `sorted(new)[0]`, the LOWEST id, which is the selection
owner. So the GUI reparented an invisible helper window into the commanded
panel, the geometry dump measured the container and read 1005..1554 exactly
as designed, every check passed, and the panel showed nothing while the real
RViz sat in its own window on top of the GUI. docs/ENGINEERING_LOG.md recorded embedding as
working on the strength of it.

That is this repository's own listed failure mode -- "feature present but
does nothing", and "everything matches: substring matching" -- in the one
place where the evidence is a picture nobody was looking at.

Three filters, and a window must pass all three:
  * a WM_CLASS of rviz2. The helpers have none, which is the cleanest single
    discriminator.
  * a title that is not one of Qt's internal ones. rviz2 titles its main
    window "<config path> - RViz"; the transient windows it makes while
    starting up do not. Measured 2026-08-22: embedding whatever appeared
    first at 1.5 s caught a 400x287 startup window, and RViz then built its
    real 1600x1000 one as a SEPARATE toplevel.
  * a size worth drawing in. A 1x1 window is not a viewport.

AND THAT IS WHY THIS IS ALSO THE READY TEST. A window passing all three is
RViz having finished building its main window -- which is the thing an
operator means by "RViz has loaded", and which the process table cannot say,
because the process exists from the first second of a start.
"""
import re
import subprocess

# 0x600106 "title": ("rviz2" "rviz2")  1600x1000+38+59  +135+45
_LINE = re.compile(
    r'(0x[0-9a-f]+)\s+(?:"(?P<title>[^"]*)"|\(has no name\))'
    r'\s*:\s*\((?P<cls>[^)]*)\)'
    r'(?:\s+(?P<w>\d+)x(?P<h>\d+))?')


def parse(tree_text):
    """The three filters, over `xwininfo -root -tree` output.

    Split from the subprocess call so the filters have a known-answer test
    that does not need an X server -- the sample above IS the test input.
    """
    ids = set()
    for line in (tree_text or "").splitlines():
        m = _LINE.search(line)
        if not m:
            continue
        if "rviz" not in (m.group("cls") or "").lower():
            continue                       # helpers carry no WM_CLASS
        title = m.group("title") or ""
        if title.startswith("Qt Selection Owner"):
            continue
        if not title.endswith("- RViz"):
            continue
        w = int(m.group("w") or 0)
        h = int(m.group("h") or 0)
        if w < 200 or h < 200:
            continue                       # 1x1 stubs are not viewports
        ids.add(int(m.group(1), 16))
    return ids


def real_windows():
    """Ids of RViz windows that are actually drawn in. Empty on no X."""
    try:
        o = subprocess.run(["xwininfo", "-root", "-tree"],
                           capture_output=True, text=True, timeout=6).stdout
    except Exception:                                         # noqa: BLE001
        return set()
    return parse(o)
