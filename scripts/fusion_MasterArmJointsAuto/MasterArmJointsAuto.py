"""MasterArmJointsAuto -- Fusion add-in that does the whole job unattended.

On Fusion start-up (or when run from Scripts and Add-Ins) it:
  1. imports  C:/Users/Gausms/Desktop/MSc_Project/3dprint/Complete Master Arm no joint animation.f3z
  2. adds the revolute joints to every arm (pot shaft circles as axes, parts
     sorted into links by position, rigid groups, shared base grounded)
  3. exports C:/Users/Gausms/Downloads/MasterArmJoints/Complete Master Arm WITH JOINTS.f3z
  4. writes DONE.txt beside it so it never runs twice; delete DONE.txt to rerun.
Everything it does is logged to autorun.log in the same folder.
"""
import math
import os
import threading
import time
import traceback

import adsk.core
import adsk.fusion

SRC = r"C:\Users\Gausms\Desktop\MSc_Project\3dprint\Complete Master Arm no joint animation.f3z"
OUTDIR = r"C:\Users\Gausms\Downloads\MasterArmJoints"
OUT = os.path.join(OUTDIR, "Complete Master Arm WITH JOINTS.f3z")
LOG = os.path.join(OUTDIR, "autorun.log")
DONE = os.path.join(OUTDIR, "DONE.txt")
DESIGN_NAME = "complete master arm"      # matched case-insensitively against your cloud designs
EVT = "MasterArmJointsAutoGo"
START_DELAY_S = 40


def log(msg):
    try:
        os.makedirs(OUTDIR, exist_ok=True)
        with open(LOG, "a", encoding="utf-8") as f:
            f.write(time.strftime("%H:%M:%S ") + str(msg) + "\n")
    except Exception:
        pass

# ---- shared joint-building core (identical in the script and the add-in) ----
POT_NAMES = ("resist", "pot")
LIMIT_DEG = 150.0
OFF_AXIS_CM = 12.0       # a part further than this from every arm axis goes with the base


def _v(p): return (p.x, p.y, p.z)
def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def _mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def _dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _len(a): return math.sqrt(_dot(a, a))


def _norm(a):
    n = _len(a)
    return _mul(a, 1.0 / n) if n > 1e-12 else (0.0, 0.0, 1.0)


def _centre(bb):
    return ((bb.minPoint.x + bb.maxPoint.x) / 2, (bb.minPoint.y + bb.maxPoint.y) / 2,
            (bb.minPoint.z + bb.maxPoint.z) / 2)


def _thin_axis(bb):
    d = (bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z)
    i = min(range(3), key=lambda k: d[k])
    return tuple(1.0 if k == i else 0.0 for k in range(3))


def _pot_axis(pot):
    """(centre, axis, entity, radius) for the pot's rotation axis, from its own geometry.

    A rotary pot's shaft is a cylinder, so several circular edges share one
    normal: the direction shared by the most circular/arc edges is the axis,
    and the largest circle on that axis gives the origin. No assumption about
    how the pot is oriented in the design. Falls back to a planar face whose
    normal is that axis if the largest circle cannot be used.
    """
    circles = []   # (centre, normal, edge, radius)
    faces = []
    kinds = {}
    for body in pot.bRepBodies:
        for e in body.edges:
            g = e.geometry
            t = g.objectType.split("::")[-1]
            kinds[t] = kinds.get(t, 0) + 1
            if t in ("Circle3D", "Arc3D"):
                circles.append((_v(g.center), _norm(_v(g.normal)), e, g.radius))
        for f in body.faces:
            g = f.geometry
            if g.objectType == adsk.core.Plane.classType():
                faces.append((f, _norm(_v(g.normal))))
    if not circles:
        raise RuntimeError("%s has no circular edges (edge kinds: %s)" % (pot.fullPathName, kinds))
    # group circles by parallel normal; the biggest group is the shaft axis
    groups = []
    for c in circles:
        for grp in groups:
            if abs(_dot(grp[0][1], c[1])) > 0.98:
                grp.append(c)
                break
        else:
            groups.append([c])
    groups.sort(key=lambda grp: (-len(grp), -max(x[3] for x in grp)))
    axis_grp = groups[0]
    axis = axis_grp[0][1]
    best = max(axis_grp, key=lambda x: x[3])
    return best[0], axis, best[2], best[3]


def _cluster(points, k):
    """k-means on a short list of 3-vectors; seeds are the k mutually farthest points."""
    seeds = [points[0]]
    while len(seeds) < k:
        seeds.append(max(points, key=lambda p: min(_len(_sub(p, s)) for s in seeds)))
    lab = [0] * len(points)
    for _ in range(20):
        lab = [min(range(k), key=lambda j: _len(_sub(p, seeds[j]))) for p in points]
        for j in range(k):
            mem = [p for p, l in zip(points, lab) if l == j]
            if mem:
                seeds[j] = _mul(sum_v(mem), 1.0 / len(mem))
    return lab


def sum_v(ps):
    s = (0.0, 0.0, 0.0)
    for p in ps:
        s = _add(s, p)
    return s



def _bbox_vol(o):
    bb = o.boundingBox
    return abs((bb.maxPoint.x - bb.minPoint.x) * (bb.maxPoint.y - bb.minPoint.y) * (bb.maxPoint.z - bb.minPoint.z))


def _arm_direction(cs, others, info_centre):
    """Unit vector along the arm pointing from base to handle.

    Rule 1: if the chain is within ~45 deg of the root z axis, the base is the
    low end (the CAD was modelled arm-up, base at z = 0). Rule 2 otherwise: the
    base is the end nearer the largest component that lies near the axis (the
    base plate or mount), because the handle end carries only small parts.
    """
    far = max(((a, b) for a in cs for b in cs), key=lambda ab: _len(_sub(ab[0], ab[1])))
    d = _norm(_sub(far[1], far[0]))
    if abs(d[2]) > 0.7:
        return d if d[2] > 0 else _mul(d, -1.0), "z-up rule"
    org = far[0]
    near = []
    for o in others:
        c = info_centre(o)
        p = _dot(_sub(c, org), d)
        perp = _len(_sub(_sub(c, org), _mul(d, p)))
        if perp < 15.0:
            near.append((_bbox_vol(o), p))
    if not near:
        return d, "default (no parts near axis)"
    big_p = max(near)[1]
    mid = 0.5 * (min(_dot(_sub(c, org), d) for c in cs) + max(_dot(_sub(c, org), d) for c in cs))
    return (d if big_p < mid else _mul(d, -1.0)), "largest-part rule"


def build_joints(design, log):
    """Add rigid groups and revolute joints for every 7-pot arm in the design. Returns a report."""
    root = design.rootComponent
    made = 0
    for i in range(root.bRepBodies.count - 1, -1, -1):
        root.bRepBodies.item(i).createComponent()
        made += 1
    seen, occs = set(), []
    for o in root.allOccurrences:
        if o.childOccurrences.count:
            continue
        key = o.fullPathName
        if key in seen:
            continue
        seen.add(key)
        occs.append(o)
    log("leaf occurrences: %d" % len(occs))
    pots = [o for o in occs if any(k in o.component.name.lower() for k in POT_NAMES)]
    if not pots or len(pots) % 7:
        raise RuntimeError("expected a multiple of 7 pots (component name containing 'resist'/'pot'), found %d:\n%s"
                           % (len(pots), "\n".join(o.fullPathName for o in pots)))
    n_arms = len(pots) // 7
    info = {o.fullPathName: _pot_axis(o) for o in pots}
    for o in pots:
        c, a, _, r = info[o.fullPathName]
        log("pot %-50s centre %s axis %s r %.2f" % (o.fullPathName, tuple(round(x, 2) for x in c), tuple(round(x, 2) for x in a), r))
    # arms: by parent path if that splits them evenly, else by position
    by_parent = {}
    for o in pots:
        by_parent.setdefault(o.fullPathName.rsplit("+", 1)[0] if "+" in o.fullPathName else "", []).append(o)
    if len(by_parent) == n_arms and all(len(v) == 7 for v in by_parent.values()):
        arms = list(by_parent.values())
        log("arms split by parent component: %s" % list(by_parent.keys()))
    else:
        lab = _cluster([info[o.fullPathName][0] for o in pots], n_arms)
        arms = [[o for o, l in zip(pots, lab) if l == j] for j in range(n_arms)]
        log("arms split by position: sizes %s" % [len(a) for a in arms])
        if any(len(a) != 7 for a in arms):
            raise RuntimeError("could not split %d pots into arms of 7 (got %s)" % (len(pots), [len(a) for a in arms]))
    others = [o for o in occs if o not in pots]
    # per arm: axis direction, base end, joint order
    arm_data = []
    for arm in arms:
        cs = [info[o.fullPathName][0] for o in arm]
        d, rule = _arm_direction(cs, others, lambda o: _centre(o.boundingBox))
        org = min(cs, key=lambda c: _dot(c, d))
        log("arm direction %s by %s" % (tuple(round(x, 3) for x in d), rule))
        proj = lambda p, d=d, org=org: _dot(_sub(p, org), d)
        arm_sorted = sorted(arm, key=lambda o: proj(info[o.fullPathName][0]))
        jpos = [proj(info[o.fullPathName][0]) for o in arm_sorted]
        arm_data.append({"pots": arm_sorted, "d": d, "org": org, "jpos": jpos, "proj": proj})
        log("arm: axis %s, joints along it (cm) %s" % (tuple(round(x, 3) for x in d), [round(p, 2) for p in jpos]))
    # name arms left/right by their offset perpendicular to the first arm's axis
    if n_arms == 2:
        d0 = arm_data[0]["d"]
        mids = []
        for ad in arm_data:
            m = _mul(sum_v([info[o.fullPathName][0] for o in ad["pots"]]), 1.0 / 7)
            mids.append(m)
        sep = _sub(mids[1], mids[0])
        sep = _sub(sep, _mul(d0, _dot(sep, d0)))
        # the arm at the more negative x (in root coordinates) is 'left' only as a label
        names = ["armA", "armB"] if abs(sep[0]) < 1e-6 else (["left", "right"] if mids[0][0] < mids[1][0] else ["right", "left"])
    else:
        names = ["arm%d" % (i + 1) for i in range(n_arms)]
    # assign every other part: base if off every axis or below every first joint, else the nearest arm's link
    base = []
    links = [[[] for _ in range(8)] for _ in range(n_arms)]
    for ai, ad in enumerate(arm_data):
        for j, o in enumerate(ad["pots"]):
            links[ai][j].append(o)      # a pot belongs to the link below its joint
    for o in others:
        c = _centre(o.boundingBox)
        best = None
        for ai, ad in enumerate(arm_data):
            p = ad["proj"](c)
            perp = _len(_sub(_sub(c, ad["org"]), _mul(ad["d"], p)))
            if best is None or perp < best[0]:
                best = (perp, ai, p)
        perp, ai, p = best
        k = sum(1 for q in arm_data[ai]["jpos"] if p >= q)
        if perp > OFF_AXIS_CM or k == 0:
            base.append(o)
            log("  %-50s -> base (off-axis %.2f, along %.2f)" % (o.fullPathName, perp, p))
        else:
            links[ai][k].append(o)
            log("  %-50s -> %s link%d (off-axis %.2f, along %.2f)" % (o.fullPathName, names[ai], k, perp, p))
    # link0 of every arm is the shared base
    for ai in range(n_arms):
        base.extend(links[ai][0])
        links[ai][0] = base
    if not base:
        raise RuntimeError("no base parts found")
    for ai in range(n_arms):
        empty = [i for i, L in enumerate(links[ai]) if not L]
        if empty:
            raise RuntimeError("%s: links with no components: %s" % (names[ai], empty))
    # rigid groups
    def group(members, name):
        coll = adsk.core.ObjectCollection.create()
        for o in members:
            coll.add(o)
        try:
            rg = root.rigidGroups.add(coll, True)
            rg.name = name
        except Exception as e:
            log("rigid group %s failed: %s" % (name, e))
    group(base, "base")
    for o in base:
        try:
            o.isGrounded = True
        except Exception:
            pass
    joint_names = []
    for ai, ad in enumerate(arm_data):
        for i in range(1, 8):
            group(links[ai][i], "%s_link%d" % (names[ai], i))
        for i in range(7):
            _, _, edge, _ = info[ad["pots"][i].fullPathName]
            geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            inp = root.asBuiltJoints.createInput(links[ai][i][0], links[ai][i + 1][0], geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            jt = root.asBuiltJoints.add(inp)
            jt.name = "%s_J%d" % (names[ai], i + 1)
            try:
                lim = jt.jointMotion.rotationLimits
                lim.isMinimumValueEnabled = True
                lim.minimumValue = -math.radians(LIMIT_DEG)
                lim.isMaximumValueEnabled = True
                lim.maximumValue = math.radians(LIMIT_DEG)
            except Exception as e:
                log("limits on %s failed: %s" % (jt.name, e))
            joint_names.append(jt.name)
            log("joint %s: %s -> %s" % (jt.name, links[ai][i][0].name, links[ai][i + 1][0].name))
    # self-test: drive J4 of each arm by 60 deg and check the handle link really moves
    checks = []
    for ai in range(n_arms):
        try:
            jt = root.asBuiltJoints.itemByName("%s_J4" % names[ai])
            tip = links[ai][7][0]
            before = _centre(tip.boundingBox)
            jt.jointMotion.rotationValue = math.radians(60.0)
            adsk.doEvents()
            after = _centre(tip.boundingBox)
            jt.jointMotion.rotationValue = 0.0
            adsk.doEvents()
            moved = _len(_sub(after, before))
            checks.append("%s handle moved %.1f cm when J4 driven 60 deg%s" % (names[ai], moved, "" if moved > 2.0 else "  <-- FAILED"))
        except Exception as e:
            checks.append("%s self-test error: %s" % (names[ai], e))
    for c in checks:
        log("CHECK " + c)
    sizes = "; ".join("%s links %s" % (names[ai], [len(L) for L in links[ai]]) for ai in range(n_arms))
    log("link sizes: " + sizes)
    return "%d arm(s); %d root bodies made components; base %d parts; %s.\nJoints: %s.\nSelf-test: %s" % (
        n_arms, made, len(base), sizes, ", ".join(joint_names), " | ".join(checks))
# ---- end core ----



_keep = []


def find_design(app):
    """The newest cloud design whose name contains DESIGN_NAME, searched across every project."""
    hits = []
    for pi in range(app.data.dataProjects.count):
        proj = app.data.dataProjects.item(pi)
        stack = [proj.rootFolder]
        while stack:
            folder = stack.pop()
            for i in range(folder.dataFiles.count):
                f = folder.dataFiles.item(i)
                if f.fileExtension.lower() == "f3d" and DESIGN_NAME in f.name.lower():
                    hits.append((f.dateModified, proj.name, f))
            for i in range(folder.dataFolders.count):
                stack.append(folder.dataFolders.item(i))
    for dm, pn, f in sorted(hits, key=lambda h: h[0], reverse=True):
        log("candidate: %s / %s (v%s, modified %s)" % (pn, f.name, f.versionNumber, dm))
    return hits and sorted(hits, key=lambda h: h[0], reverse=True)[0][2]


def do_work(app, ui):
    try:
        log("=== MasterArmJointsAuto work start")
        df = find_design(app)
        if df is None:
            raise RuntimeError("no cloud design containing %r found in any project" % DESIGN_NAME)
        log("opening %s" % df.name)
        doc = app.documents.open(df, True)
        adsk.doEvents()
        for _ in range(60):          # give the design time to load
            design = adsk.fusion.Design.cast(app.activeProduct)
            if design and design.rootComponent.allOccurrences.count > 0:
                break
            time.sleep(1); adsk.doEvents()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise RuntimeError("no active design after opening %s" % df.name)
        log("open: %s, %d occurrences" % (doc.name, design.rootComponent.allOccurrences.count))
        report = build_joints(design, log)
        log(report)
        folder = df.parentFolder
        newname = "Complete Master Arm WITH JOINTS"
        doc.saveAs(newname, folder, "joints added by MasterArmJointsAuto", "")
        adsk.doEvents()
        log("saved as %s in %s" % (newname, folder.name))
        exp = design.exportManager
        eo = exp.createFusionArchiveExportOptions(OUT)
        ok = exp.execute(eo)
        log("export f3z: %s -> %s" % (ok, OUT))
        with open(DONE, "w") as f:
            f.write("done " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n" + report + "\n")
        log("=== finished")
        ui.messageBox("Master arm joints added.\n\nSaved in your project as '%s' and exported to:\n%s\n\n%s\n\nDrag a link or right-click a joint -> Animate Joint." % (newname, OUT, report))
    except Exception:
        log("FAILED:\n" + traceback.format_exc())


class _Go(adsk.core.CustomEventHandler):
    def __init__(self, app, ui):
        super().__init__()
        self.app, self.ui = app, ui

    def notify(self, args):
        do_work(self.app, self.ui)


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    if os.path.exists(DONE):
        log("DONE.txt present; not running again")
        return
    startup = isinstance(context, dict) and context.get("IsApplicationStartup", False)
    log("=== MasterArmJointsAuto loaded (startup=%s); work in %d s" % (startup, START_DELAY_S if startup else 1))
    try:
        app.unregisterCustomEvent(EVT)
    except Exception:
        pass
    ev = app.registerCustomEvent(EVT)
    h = _Go(app, ui)
    ev.add(h)
    _keep.append((ev, h))

    def fire():
        time.sleep(START_DELAY_S if startup else 1)
        app.fireCustomEvent(EVT, "")
    threading.Thread(target=fire, daemon=True).start()


def stop(context):
    try:
        adsk.core.Application.get().unregisterCustomEvent(EVT)
    except Exception:
        pass
