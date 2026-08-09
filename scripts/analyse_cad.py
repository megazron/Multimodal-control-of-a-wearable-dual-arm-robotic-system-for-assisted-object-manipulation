#!/usr/bin/env python3
"""Decompose the STEP assemblies and check them against the URDF and the FK.

    .percep_venv/bin/python scripts/analyse_cad.py

WHY cadquery AND NOT pythonocc-core. pythonocc-core is not on PyPI at all --
it is distributed through conda -- and this workspace has no conda. cadquery
ships OCP, which is a binding to the SAME OpenCascade kernel, and it installs
with pip. So the kernel is identical; only the packaging differs.

Where a regex over CARTESIAN_POINT was enough (overall bounding extents), the
kernel is not needed and scripts/measure_cad_step.py does it with the standard
library. The kernel is needed for exactly three things, and this script does
those: separating an assembly into SOLIDS, getting each solid's own placement
rather than the whole file's extent, and reading volume.

WHAT IS BEING CHECKED, and why it matters.

  1. The master arm's DESIGNED link lengths against the values the forward
     kinematics uses (0.043, 0.037, 0.043, 0.037, 0.043, 0.036, 0.033 m).
     Those seven numbers were MEASURED off the built arm; if the CAD disagrees,
     one of the two is wrong and every commanded position inherits it.

  2. THE BACKPACK MOUNT ANGLE. The URDF's mount rpy was solved ANALYTICALLY
     from where the end effector had to end up, and has never been checked
     against the physical bracket. If the CAD disagrees it would explain the
     front-reach limitation better than anything so far.

MASS: reported only if the STEP carries material density. Volume alone is not
mass, and multiplying by an assumed density would be inventing a number.
"""
import math
import os
import sys

FK_LINKS_M = [0.043, 0.037, 0.043, 0.037, 0.043, 0.036, 0.033]
URDF_MOUNT_XYZ = (0.20, -0.20, 0.18)          # backpack-relative, applied
URDF_MOUNT_RPY_LEFT = (1.247666407, -0.232161321, 1.508064663)


def load(path):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError("could not read %s" % path)
    r.TransferRoots()
    return r.OneShape()


def solids(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    out, exp = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def bbox(s):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_Box()
    BRepBndLib.Add_s(s, b)
    xm, ym, zm, xM, yM, zM = b.Get()
    return (xM - xm, yM - ym, zM - zm), ((xm + xM) / 2, (ym + yM) / 2,
                                         (zm + zM) / 2)


def volume(s):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, g)
    return g.Mass()          # OCCT calls volume "Mass" when density is 1


def main():
    P = "/mnt/c/Users/Gausms/Desktop/MSc_Project/CAD"
    print("=" * 76)
    print("CAD DECOMPOSITION -- OCCT kernel via cadquery/OCP")
    print("=" * 76)

    out = {}
    for name in ("PotArm.step", "backpack.step"):
        path = os.path.join(P, name)
        if not os.path.exists(path):
            print("missing:", path)
            continue
        shp = load(path)
        sl = solids(shp)
        print("\n%s: %d solid(s)" % (name, len(sl)))
        rows = []
        for i, s in enumerate(sl):
            d, c = bbox(s)
            v = volume(s)
            rows.append(dict(i=i, dims=d, centre=c, vol_mm3=v))
        rows.sort(key=lambda r: -r["vol_mm3"])
        print("   %-4s %26s %26s %12s" % ("#", "bbox X,Y,Z (mm)",
                                          "centre X,Y,Z (mm)", "vol (mm^3)"))
        for r in rows[:24]:
            print("   %-4d %8.1f %8.1f %8.1f   %8.1f %8.1f %8.1f   %11.0f"
                  % (r["i"], r["dims"][0], r["dims"][1], r["dims"][2],
                     r["centre"][0], r["centre"][1], r["centre"][2],
                     r["vol_mm3"]))
        if len(rows) > 24:
            print("   ... and %d more" % (len(rows) - 24))
        tot = sum(r["vol_mm3"] for r in rows)
        print("   total solid volume %.1f cm^3" % (tot / 1000.0))
        print("   MASS: not reported. The STEP carries no density, and")
        print("         volume x an assumed density is an invented number.")
        out[name] = rows

    # ------------------------------------------------ link-length comparison
    arm = out.get("PotArm.step", [])
    print("\n" + "=" * 76)
    print("DESIGNED LINK LENGTHS vs THE VALUES THE FK USES")
    print("=" * 76)
    print("FK chain: %s mm, sum %.1f mm"
          % ([round(1000 * v) for v in FK_LINKS_M], 1000 * sum(FK_LINKS_M)))
    if len(arm) < 2:
        print("\nCANNOT DECOMPOSE: the arm STEP contains %d solid(s), so it is"
              % len(arm))
        print("a single fused body rather than an assembly of links. Per-link")
        print("lengths are NOT recoverable from it, and no comparison against")
        print("the FK values can be made. That is a property of the file, not")
        print("a disagreement between CAD and FK.")
    else:
        # FILTER TO STRUCTURAL SOLIDS. The arm STEP contains 400+ solids,
        # almost all of them screws, pins and small features whose Z spacing
        # has nothing to do with link length. Taking gaps over all of them
        # measures the fastener pattern. A link is a large solid; the
        # threshold is stated so the filter is inspectable rather than tuned
        # until the answer looks right.
        LINK_MIN_MM3 = 2000.0
        big = [r for r in arm if r["vol_mm3"] >= LINK_MIN_MM3]
        print("\nsolids >= %.0f mm^3 (link candidates): %d of %d"
              % (LINK_MIN_MM3, len(big), len(arm)))
        for r in sorted(big, key=lambda r: r["centre"][2])[:12]:
            print("   z=%7.1f  bbox %6.1f x %6.1f x %6.1f  vol %8.0f"
                  % (r["centre"][2], r["dims"][0], r["dims"][1],
                     r["dims"][2], r["vol_mm3"]))
        arm = big if len(big) >= 2 else arm
        zs = sorted(r["centre"][2] for r in arm)
        gaps = [zs[i + 1] - zs[i] for i in range(len(zs) - 1)]
        print("\nsolid centres along Z (mm): %s"
              % [round(z, 1) for z in zs][:12])
        print("consecutive gaps (mm):      %s"
              % [round(g, 1) for g in gaps][:12])
        print("\nA gap is only a LINK LENGTH if consecutive solids are")
        print("consecutive links stacked along Z. Check that against the")
        print("figure before treating any of these as a designed length.")

    # ------------------------------------------------------- mount geometry
    print("\n" + "=" * 76)
    print("BACKPACK MOUNT")
    print("=" * 76)
    bp = out.get("backpack.step", [])
    print("URDF (applied): xyz %s m, left rpy %s rad"
          % (URDF_MOUNT_XYZ, tuple(round(v, 4) for v in URDF_MOUNT_RPY_LEFT)))
    print("               left rpy in degrees: %.1f, %.1f, %.1f"
          % tuple(round(math.degrees(v), 1) for v in URDF_MOUNT_RPY_LEFT))
    if len(bp) < 2:
        print("\nCANNOT LOCATE THE ARM BASES: backpack.step contains %d"
              % len(bp))
        print("solid(s). With no separate mounting-face solid there is no")
        print("placement to read, so the URDF rotation CANNOT be checked")
        print("against the bracket from this file.")
        print("\nThis is the measurement the whole of C2 was for, and it is")
        print("NOT AVAILABLE. Reporting it as agreement or disagreement would")
        print("be inventing a result.")
    else:
        print("\n%d solids. The mounting faces are among them, but nothing in"
              % len(bp))
        print("the STEP labels which, so an angle derived by picking solids by")
        print("size would be a guess dressed as a measurement.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
