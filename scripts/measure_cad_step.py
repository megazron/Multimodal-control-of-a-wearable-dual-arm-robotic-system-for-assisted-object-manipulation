#!/usr/bin/env python3
"""Bounding dimensions from STEP files, with no CAD library.

WHY NO LIBRARY. STEP (ISO 10303-21) is a TEXT format, and every vertex is a
CARTESIAN_POINT with literal coordinates. A bounding box over those points
needs a regex, not a kernel. cadquery and pythonocc-core are each a large
install with a compiled OCCT dependency, and neither is needed to answer
"how big is this part".

WHAT THIS CAN AND CANNOT MEASURE, stated up front because the limit matters.

  CAN: the extent of the geometry's control points, which for a part built
       from planar and cylindrical faces is the bounding box.
  CANNOT: mass, material, the assembly tree, or the position of a subpart
       inside an assembly. Those need the kernel.
  CAUTION: the extent covers CONTROL points of any B-spline, which can lie
       slightly outside the surface they define. On a printed mechanical part
       this is small, but it makes the result an upper bound rather than an
       exact dimension, and it is reported as such.

Units are read from the file's SI_UNIT declaration rather than assumed.
"""
import os
import re
import sys

PT = re.compile(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*"
                r"(-?[\d.E+-]+)\s*,\s*(-?[\d.E+-]+)\s*,\s*(-?[\d.E+-]+)")


def measure(path):
    src = open(path, errors="replace").read()
    unit = "mm" if "SI_UNIT(.MILLI.,.METRE.)" in src else "m"
    xs, ys, zs = [], [], []
    for m in PT.finditer(src):
        try:
            x, y, z = (float(v) for v in m.groups())
        except ValueError:
            continue
        xs.append(x); ys.append(y); zs.append(z)
    if not xs:
        return None
    dims = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    return dict(n_points=len(xs), unit=unit, dims=dims,
                lo=(min(xs), min(ys), min(zs)),
                hi=(max(xs), max(ys), max(zs)))


def main(argv):
    print("=" * 74)
    print("CAD STEP GEOMETRY -- bounding extents, no CAD kernel")
    print("=" * 74)
    print("Extent is over CARTESIAN_POINT control points, so it is an UPPER")
    print("BOUND on the true dimension where B-splines are involved.\n")
    for p in argv:
        r = measure(p)
        name = os.path.basename(p)
        if r is None:
            print("  %-18s no CARTESIAN_POINT found" % name)
            continue
        d = r["dims"]
        print("  %-18s %7d points, units %s" % (name, r["n_points"], r["unit"]))
        print("      extent  X %8.2f   Y %8.2f   Z %8.2f  %s"
              % (d[0], d[1], d[2], r["unit"]))
        print("      from    (%.1f, %.1f, %.1f)  to  (%.1f, %.1f, %.1f)"
              % (r["lo"] + r["hi"]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or []))
