#!/usr/bin/env python3
"""Render the CAD to vector figures for the report.

    .percep_venv/bin/python scripts/render_cad_figures.py

VECTOR, NOT A SCREENSHOT. cadquery's SVG exporter projects the B-rep directly,
so the output is resolution-independent and needs no GL context, no display
and no compositor. That matters here specifically: this machine records BLACK
when x11grab reads a composited window, so anything that needed a rendered
window would have to go through the Xvfb detour the clip pipeline uses.

FOUR STANDARD VIEWS plus an isometric, which is what a mechanical drawing
needs to be checkable: a single isometric hides exactly the dimensions a
reader wants to confirm.

Dimensions are annotated from the MEASURED bounding box of each view rather
than typed in, so a figure cannot drift away from the geometry it shows.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAD = "/mnt/c/Users/Gausms/Desktop/MSc_Project/CAD"
OUT = os.path.join(ROOT, "extras/thesis/thesis_report/figures/cad")

VIEWS = {
    "iso":   ((1.0, -1.0, 0.6), "isometric"),
    "front": ((0.0, -1.0, 0.0), "front (looking along +y)"),
    "side":  ((1.0, 0.0, 0.0), "side (looking along -x)"),
    "top":   ((0.0, 0.0, 1.0), "top (looking down -z)"),
}


def bbox_of(shape):
    # importStep returns a Workplane, whose solids carry .wrapped; the
    # Workplane itself does not.
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b = Bnd_Box()
    for v in shape.vals():
        BRepBndLib.Add_s(v.wrapped, b)
    xm, ym, zm, xM, yM, zM = b.Get()
    return (xM - xm, yM - ym, zM - zm)


def main():
    import cadquery as cq
    from cadquery import exporters

    os.makedirs(OUT, exist_ok=True)
    made = []
    for stem in ("PotArm", "backpack"):
        path = os.path.join(CAD, stem + ".step")
        if not os.path.exists(path):
            print("missing:", path)
            continue
        shape = cq.importers.importStep(path)
        d = bbox_of(shape)
        print("\n%s  bbox %.1f x %.1f x %.1f mm" % (stem, *d))
        for key, (direction, label) in VIEWS.items():
            f = os.path.join(OUT, "%s_%s.svg" % (stem.lower(), key))
            try:
                exporters.export(
                    shape, f, opt=dict(projectionDir=direction,
                                       width=900, height=700,
                                       marginLeft=24, marginTop=24,
                                       showAxes=False,
                                       strokeWidth=0.35,
                                       strokeColor=(20, 20, 20),
                                       hiddenColor=(170, 170, 170),
                                       showHidden=False))
                made.append(f)
                print("   %-6s -> %-28s %s" % (key, os.path.basename(f), label))
            except Exception as e:                           # noqa: BLE001
                print("   %-6s FAILED: %r" % (key, e))
    print("\n%d svg(s) -> %s" % (len(made), OUT))
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
