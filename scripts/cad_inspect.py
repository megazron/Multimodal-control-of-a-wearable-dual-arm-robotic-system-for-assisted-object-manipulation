#!/usr/bin/env python3
"""Read the master-arm STEP files and report what is actually in them.

WHY cadquery-ocp: pip-installable wheels, no root and no conda, and it is the
full OpenCASCADE kernel (OCCT) rather than a parser -- bounding boxes, volumes
and surface classification all need the kernel. pythonocc-core is conda-first
and its pip wheels are unreliable; headless FreeCAD needs apt (root) and about
a gigabyte for the same kernel.

THE MODEL WAS DESIGNED FOR PRINTING, NOT KINEMATICS. There are no joints, no
joint frames and one solid per printed part, so nothing here reads a frame out
of the file -- everything is inferred from geometry, and reported with the
confidence that inference earns.

    .cad_venv/bin/python scripts/cad_inspect.py
"""
import json
import os
import sys

CAD = "/mnt/c/Users/Gausms/Desktop/MSc_Project/CAD"
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "recordings/baselines/cad_inspect.json")


def load(path):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    rdr = STEPControl_Reader()
    if rdr.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError("could not read %s" % path)
    unit = Interface_Static.CVal_s("xstep.cascade.unit")
    rdr.TransferRoots()
    return rdr.OneShape(), unit


def solids(shape):
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(TopoDS.Solid_s(exp.Current()))
        exp.Next()
    return out


def bbox(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    xm, ym, zm, xx, yx, zx = b.Get()
    return (xm, ym, zm, xx, yx, zx)


def volume(shape):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    c = p.CentreOfMass()
    return p.Mass(), (c.X(), c.Y(), c.Z())


def main():
    res = {}
    for name in ("PotArm.step", "backpack.step"):
        path = os.path.join(CAD, name)
        if not os.path.exists(path):
            print("MISSING %s" % path)
            continue
        shape, unit = load(path)
        sl = solids(shape)
        xm, ym, zm, xx, yx, zx = bbox(shape)
        dims = (xx - xm, yx - ym, zx - zm)
        vol, com = volume(shape)
        print("=" * 74)
        print("%s   STEP unit: %s" % (name, unit))
        print("  solids            : %d" % len(sl))
        print("  bounding box (mm) : %.1f x %.1f x %.1f" % dims)
        print("  extent            : x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f"
              % (xm, xx, ym, yx, zm, zx))
        print("  volume            : %.1f cm^3" % (vol / 1000.0))
        print("  centre of mass    : (%.1f, %.1f, %.1f) mm" % com)
        # PLA at 1.24 g/cm^3, printed solid. STEP carries no density unless a
        # material was assigned; none is read here, so this is an ESTIMATE and
        # is labelled as one.
        print("  mass @ PLA 1.24   : %.0f g  (ESTIMATE -- no material in file)"
              % (vol / 1000.0 * 1.24))
        per = []
        for i, s in enumerate(sl):
            v, c = volume(s)
            bx = bbox(s)
            per.append(dict(index=i, volume_mm3=round(v, 1),
                            com=[round(q, 2) for q in c],
                            dims=[round(bx[3] - bx[0], 2),
                                  round(bx[4] - bx[1], 2),
                                  round(bx[5] - bx[2], 2)]))
        if len(sl) > 1:
            print("  per-solid:")
            for p in sorted(per, key=lambda q: -q["volume_mm3"])[:12]:
                print("     #%-3d %8.1f mm^3   %s" % (p["index"],
                                                      p["volume_mm3"],
                                                      p["dims"]))
        res[name] = dict(unit=unit, n_solids=len(sl),
                         dims_mm=[round(d, 2) for d in dims],
                         extent=[round(q, 2) for q in
                                 (xm, xx, ym, yx, zm, zx)],
                         volume_mm3=round(vol, 1),
                         com_mm=[round(q, 2) for q in com],
                         mass_g_pla_estimate=round(vol / 1000.0 * 1.24, 1),
                         solids=per)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("\n-> %s" % OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
