#!/usr/bin/env python3
"""RE-VERIFY every coordinate in every protocol at N repeats over the WHOLE
PATH, and report which figures change.

    python3 scripts/audit_scenario_reachability.py            # everything
    python3 scripts/audit_scenario_reachability.py --task t5
    python3 scripts/audit_scenario_reachability.py --repeats 9 --step 0.015

WHY THIS EXISTS
---------------
T2's coordinates passed a single IK call and failed a repeated one: at
x = +/-0.10 the same pose scored anywhere from 0/5 to 4/5 depending on
height, while x = +/-0.15 scored 5/5. TRAC-IK uses random restarts, so a pose
at the edge of the feasible set is a COIN FLIP, and one call is one flip.

Every scenario in this project was verified at k=2 -- better than one, not
enough. A pose that is 60% feasible passes a 2-call check 36% of the time,
which is often enough to get written into a protocol and rare enough to look
like bad luck when it fails on the day.

WHAT "VERIFIED" MEANS HERE
--------------------------
  * N repeats per pose (default 5), each repeat allowed the follower's own
    joint_3 redundancy re-seed and retry, exactly as ik_follower_node does;
  * the WHOLE PATH, densified to `--step` metres, not the endpoints. A
    scenario whose endpoints solve and whose transit does not is the exact
    shape that wastes a participant;
  * for the coupled tasks, BOTH grippers at every waypoint simultaneously;
  * for T2, the holding arm required solvable at its hold pose at every
    waypoint of the working arm's path.

Each pose is graded from its pass fraction:

    SOLID     N/N          usable
    MARGINAL  1..N-1 / N   NOT usable -- this is the T2 failure, by name
    FAIL      0/N          unreachable

A scenario is verified only if every pose on its path is SOLID.

N IS NOT A SUBSTITUTE FOR MARGIN
--------------------------------
No finite N proves a pose reachable; it only bounds how often a check lies.
At N=10 a 70% pose still passes 3.5% of the time and an 82% pose 15%. What
saves this is that feasibility here falls off a CLIFF rather than degrading:
measured at x=0.15, y=0.35, the left arm is 100% at z=1.34 and 82% at 1.36.
So the real defence is MARGIN -- keep every declared figure at least 20 mm
inside the last pose that passed N/N -- and N is what finds the boundary.
"""
import argparse
import math
import os
import sys

import numpy as np
import rclpy
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver, HOME_TOL_RAD          # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIM = os.path.join(ROOT, "src/srl_experiments/experiments/bimanual")
sys.path.insert(0, os.path.join(BIM, "t7_pursuit"))


# ------------------------------------------------------------------ paths
def densify(waypoints, step):
    """Polyline -> point list with no gap larger than `step`.

    The straight segment BETWEEN two reachable waypoints is not itself
    guaranteed reachable -- the arm travels it, so it must be checked. T3's
    S3 detour is 4 waypoints spanning 200 mm; checking only those 4 leaves
    150 mm of unexamined travel between each pair.
    """
    W = [np.asarray(w, float) for w in waypoints]
    out = [W[0]]
    for a, b in zip(W[:-1], W[1:]):
        d = float(np.linalg.norm(b - a))
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    # de-duplicate points closer together than a millimetre
    keep = [out[0]]
    for p in out[1:]:
        if float(np.linalg.norm(p - keep[-1])) > 1e-3:
            keep.append(p)
    return [list(map(float, p)) for p in keep]


def decimate(points, step):
    """Thin a dense trace to points at least `step` apart, order preserved."""
    keep = []
    for p in points:
        q = np.asarray(p, float)
        if not keep or float(np.linalg.norm(q - keep[-1])) >= step:
            keep.append(q)
    return [list(map(float, p)) for p in keep]


# ------------------------------------------------------------------ grading
class Grader:
    def __init__(self, node, quats, repeats, tries):
        self.n = node
        self.q = quats
        self.N = repeats
        self.tries = tries
        self.cache = {}
        self.calls = 0

    def score(self, arm, xyz):
        """Pass count out of N. Cached: paths revisit poses constantly."""
        key = (arm, tuple(round(v, 4) for v in xyz))
        if key in self.cache:
            return self.cache[key]
        # SHORT-CIRCUIT on the first failure. A pose is graded SOLID only at
        # N/N, so one failure settles it and the remaining draws would tell
        # us nothing we act on. This makes the cost of raising N fall almost
        # entirely on poses that are already passing.
        k = 0
        for _ in range(self.N):
            self.calls += 1
            if not self.n.solve(arm, xyz, self.q[arm], tries=self.tries):
                break
            k += 1
        self.cache[key] = k
        return k

    def grade(self, arm, xyz):
        k = self.score(arm, xyz)
        return ("SOLID" if k == self.N else "FAIL" if k == 0 else "MARGINAL"), k

    def pair_grade(self, a_xyz, b_xyz):
        """Both grippers at once, on the two ends of the object.

        WHICH ARM TAKES WHICH END IS NOT OBVIOUS AND MUST NOT BE ASSUMED.
        `left_*` links sit at POSITIVE x in this model -- the naming is
        viewer-perspective, not anatomical -- so the "left" arm works the +x
        end. The first version of this function assigned left to the -x end,
        and reported every T3 waypoint as unreachable. That was the audit
        being wrong, not the geometry. Both assignments are tried and the
        better is kept, exactly as the original checker did.
        """
        order = {"FAIL": 0, "MARGINAL": 1, "SOLID": 2}
        best = ("FAIL", 0)
        for la, ra in ((a_xyz, b_xyz), (b_xyz, a_xyz)):
            gl, kl = self.grade("left", la)
            gr, kr = self.grade("right", ra)
            g = gl if order[gl] <= order[gr] else gr
            k = min(kl, kr)
            if order[g] > order[best[0]] or (g == best[0] and k > best[1]):
                best = (g, k)
            if best[0] == "SOLID":
                break
        return best


def walk(gr, poses, label_of):
    """Grade a list of (arm_or_pair, xyz) and return the worst finding."""
    worst = []
    for item in poses:
        if item[0] == "pair":
            g, k = gr.pair_grade(item[1], item[2])
            where = "pair L%s R%s" % (fmt(item[1]), fmt(item[2]))
        else:
            g, k = gr.grade(item[0], item[1])
            where = "%s %s" % (item[0], fmt(item[1]))
        if g != "SOLID":
            worst.append((g, k, where, label_of(item)))
    return worst


def fmt(p):
    return "(%+.3f,%+.3f,%+.3f)" % tuple(p)


# ------------------------------------------------------------------ report
class Report:
    def __init__(self):
        self.rows = []

    def add(self, task, name, ok, npose, bad, note=""):
        self.rows.append(dict(task=task, scenario=name, verified=ok,
                              poses=npose, bad=bad, note=note))
        flag = "VERIFIED" if ok else "FAILS"
        print("    %-22s %-9s %3d poses  %s"
              % (name, flag, npose, note))
        for g, k, where, lab in bad[:6]:
            print("        %-8s %d/%s  %s  %s" % (g, k, "N", where, lab))
        if len(bad) > 6:
            print("        ... and %d more" % (len(bad) - 6))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="all",
                    help="all | t2 | t3 | t5 | t6 | t7")
    # N=10, not the 5 this audit was specified with. MEASURED reason: the
    # pose that broke T2's S3, (0.15, 0.35, 1.36), is 72-82% feasible per
    # call, and a 72% pose passes a 5/5 check 19% of the time -- it did,
    # here, on the first probe. N=10 takes that to 3.7%. Short-circuiting
    # makes the extra draws nearly free.
    ap.add_argument("--repeats", type=int, default=10)
    ap.add_argument("--step", type=float, default=0.02,
                    help="path densification, metres")
    ap.add_argument("--tries", type=int, default=6,
                    help="joint_3 redundancy re-seeds, as ik_follower_node")
    ap.add_argument("--t7-duration", type=float, default=30.0)
    ap.add_argument("--skip-home-check", action="store_true")
    ap.add_argument("--out", default=os.path.join(
        ROOT, "recordings/baselines/scenario_audit.yaml"))
    a = ap.parse_args()

    rclpy.init()
    n = Solver()
    n.spin(3.0)
    if not n.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim first")
        return 2

    print("SCENARIO REACHABILITY AUDIT   N=%d repeats, path step %.0f mm"
          % (a.repeats, 1000 * a.step))
    ok_home = True
    for arm in ("left", "right"):
        worst, j = n.home_ok(arm)
        print("  %-5s home offset %.4f rad (%.2f deg) joint_%d  [%s]"
              % (arm, worst, math.degrees(worst), j,
                 "ok" if worst <= HOME_TOL_RAD else "NOT AT HOME"))
        ok_home &= worst <= HOME_TOL_RAD
    if not ok_home and not a.skip_home_check:
        print("\n  REFUSING: this measures reach FROM HOME. Restart the sim "
              "with no followers running.")
        return 3

    quats = {arm: n.ee_quat(arm) for arm in ("left", "right")}
    if any(q is None for q in quats.values()):
        print("  no tf2 for an end effector")
        return 2
    gr = Grader(n, quats, a.repeats, a.tries)
    rep = Report()

    spec = yaml.safe_load(open(os.path.join(BIM, "scenarios_verified.yaml")))
    tasks = spec.get("tasks", {})
    want = None if a.task == "all" else a.task

    # ------------------------------------------------------------- T3 / T6
    if want in (None, "t3", "t6") and "T3_rigid" in tasks:
        print("\n  T3 / T6 coupled transport -- both grippers, whole path")
        for name, s in tasks["T3_rigid"].items():
            sep = float(s.get("sep", 0.310))
            pts = densify(s["path"], a.step)
            poses = [("pair", [p[0] - sep / 2, p[1], p[2]],
                      [p[0] + sep / 2, p[1], p[2]]) for p in pts]
            bad = walk(gr, poses, lambda i: "")
            rep.add("T3/T6", name, not bad, len(poses), bad,
                    "sep %.0f mm, %d waypoints densified to %d"
                    % (1000 * sep, len(s["path"]), len(pts)))

    # ----------------------------------------------------------------- T7
    if want in (None, "t7") and "T7_pursuit" in tasks:
        print("\n  T7 pursuit -- the ACTUAL Lissajous locus, not the centre")
        import targets as tgm
        for name, s in tasks["T7_pursuit"].items():
            _, L, R = tgm.trial_targets(s, a.t7_duration, dt=0.05)
            uni = s.get("unimanual", "")
            poses = []
            if uni != "right":
                poses += [("left", p) for p in decimate(L, a.step)]
            if uni != "left":
                poses += [("right", p) for p in decimate(R, a.step)]
            bad = walk(gr, poses, lambda i: "")
            rep.add("T7", name, not bad, len(poses), bad,
                    "%.0f s of locus, amplitude %.0f mm"
                    % (a.t7_duration, 1000 * s.get("amplitude_m", 0.08)))

    # ----------------------------------------------------------------- T5
    if want in (None, "t5") and "T5_handover" in tasks:
        print("\n  T5 handover -- approach, grasp, transit, present, retreat")
        for name, s in tasks["T5_handover"].items():
            arm = s.get("arm", "right")
            o, r = list(s["object"]), list(s["receive"])
            # The path the arm actually flies, not two endpoints.
            wps = [[o[0], o[1], o[2] + 0.08],        # approach from above
                   o,                                 # grasp
                   [o[0], o[1], o[2] + 0.08],        # lift clear
                   [(o[0] + r[0]) / 2, (o[1] + r[1]) / 2,
                    max(o[2], r[2]) + 0.08],          # transit
                   [r[0], r[1], r[2] + 0.06],        # arrive above
                   r,                                 # present / hold station
                   [r[0], r[1], r[2] + 0.08]]        # retreat
            pts = densify(wps, a.step)
            poses = [(arm, p) for p in pts]
            bad = walk(gr, poses, lambda i: "")
            rep.add("T5", name, not bad, len(poses), bad,
                    "%s arm, %d waypoints densified to %d"
                    % (arm, len(wps), len(pts)))

    # ----------------------------------------------------------------- T2
    if want in (None, "t2") and "T2_hold_fill" in tasks:
        print("\n  T2 hold and fill -- fill path with the hold held throughout")
        for name, s in tasks["T2_hold_fill"].items():
            ha, fa = s["hold_arm"], s["fill_arm"]
            pts = densify(s["fill_path"], a.step)
            poses = [(ha, s["hold"])] + [(fa, p) for p in pts]
            bad = walk(gr, poses, lambda i: "")
            rep.add("T2", name, not bad, len(poses), bad,
                    "hold %s %s, fill %s, %d poses"
                    % (ha, fmt(s["hold"]), fa, len(pts)))

    # ------------------------------------------- the numbers a HUMAN reads
    # The protocol .md tables are what someone lays the lab out from. They
    # are a separate source of truth from scenarios_verified.yaml and have
    # never been checked against a solver at all.
    print("\n  PROTOCOL .md COORDINATE TABLES (what the lab is laid out from)")
    md = protocol_points()
    md_rows = []
    for task, label, arm, xyz in md:
        if want and task != want:
            continue
        g, k = gr.grade(arm, xyz)
        md_rows.append(dict(task=task, item=label, arm=arm,
                            xyz=[float(v) for v in xyz], grade=g,
                            passes=int(k), n=a.repeats))
        print("    %-4s %-24s %-5s %s  %-8s %d/%d"
              % (task, label, arm, fmt(xyz), g, k, a.repeats))

    # ------------------------------------------------------------- summary
    print("\n  SUMMARY")
    nv = sum(1 for r in rep.rows if r["verified"])
    print("    scenarios      %d of %d verified at N=%d over the whole path"
          % (nv, len(rep.rows), a.repeats))
    bad_md = [r for r in md_rows if r["grade"] != "SOLID"]
    print("    protocol table %d of %d coordinates SOLID"
          % (len(md_rows) - len(bad_md), len(md_rows)))
    print("    IK calls       %d" % gr.calls)
    marg = sum(1 for r in rep.rows for b in r["bad"] if b[0] == "MARGINAL")
    print("    MARGINAL poses %d  (pass sometimes -- the T2 failure mode)"
          % marg)

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    yaml.safe_dump(dict(
        repeats=a.repeats, path_step_m=a.step, tries=a.tries,
        note="Graded SOLID only at N/N. MARGINAL means the pose passes "
             "sometimes, which is how a single-call figure gets written into "
             "a protocol and then fails on the day.",
        scenarios=[{k: v for k, v in r.items() if k != "bad"} |
                   {"bad": [[b[0], int(b[1]), b[2]] for b in r["bad"]]}
                   for r in rep.rows],
        protocol_table=md_rows), open(a.out, "w"), sort_keys=False)
    print("    written        %s" % a.out)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if (nv == len(rep.rows) and not bad_md) else 1


def protocol_points():
    """Every coordinate written in a protocol .md table, with the arm the
    protocol assigns to it. Transcribed by hand ONCE, here, so the audit can
    say whether the document a human reads agrees with the solver."""
    return [
        ("t2", "container handle (hold)", "right", [-0.15, 0.35, 1.10]),
        ("t2", "opening centre", "left", [0.15, 0.35, 1.20]),
        ("t2", "block A pick", "left", [0.30, 0.35, 1.15]),
        ("t2", "block B pick", "left", [0.35, 0.35, 1.15]),
        ("t2", "block C pick", "left", [0.45, 0.35, 1.15]),
        ("t2", "block D pick", "left", [0.35, 0.35, 1.25]),
        ("t3", "tray grip -x end", "right", [-0.155, 0.35, 1.10]),
        ("t3", "tray grip +x end", "left", [0.155, 0.35, 1.10]),
        ("t3", "lift top -x", "right", [-0.155, 0.35, 1.30]),
        ("t3", "lift top +x", "left", [0.155, 0.35, 1.30]),
        ("t3", "detour -x (S3)", "right", [-0.155, 0.38, 1.25]),
        ("t3", "detour +x (S3)", "left", [0.155, 0.38, 1.25]),
        ("t5", "tool cradle grasp", "right", [-0.30, 0.35, 1.05]),
        ("t5", "receive point", "right", [-0.16, 0.35, 1.05]),
        ("t5", "retreat", "right", [-0.16, 0.35, 1.13]),
        ("t7", "target centre left", "left", [0.40, 0.35, 1.10]),
        ("t7", "target centre right", "right", [-0.40, 0.35, 1.10]),
    ]


if __name__ == "__main__":
    sys.exit(main())
