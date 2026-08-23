#!/usr/bin/env python3
"""A motion planner, because this project did not have one.

    python3 -m srl_perception.joint_planner        # the self-test

WHAT WAS THERE BEFORE. Nothing planned. `ik_follower_node` solves ONE IK per
commanded pose and clamps the joint step to `max_step_rad` per cycle -- pose
streaming with a rate limiter. There is no OMPL in the command path, no
`move_group.plan()`, no Cartesian path service; the only OMPL reference in
the tree is a vendored Kinova launch file this project does not run.

For TELEOPERATION that is correct. The operator's hand is the input, there is
no goal to plan to, and the right problem is smooth safe following.

For AUTONOMY it is not. A pick is a discrete goal with a known start, and
streaming toward it in a straight joint-space line is exactly how a sweep
that clears both endpoints goes through the wearer -- defect 2 of 2026-08-21,
and `safe_motion.check_path` was written to CATCH that rather than to avoid
it. Catching it means the run stops. Avoiding it means the run works.

SO THIS IS AN RRT-CONNECT IN JOINT SPACE, and the choices are made for this
robot rather than in general:

  * JOINT SPACE, not Cartesian. The constraint that actually binds here is
    the distance from the ARM to the WEARER, and that is a function of the
    whole chain, not of the hand. A Cartesian planner would have to solve IK
    at every node anyway.
  * THE WEARER FLOOR IS THE COLLISION CHECK, through the same
    `clearance_parts(...)["moving_chain_m"]` the guard enforces. Not the
    SRDF and not `avoid_collisions`: HARD CONSTRAINT 11 records that the
    SRDF excludes the 44 proximal pairs a shoulder mount actually threatens,
    so a pose MoveIt calls valid can have the tube inside the person.
  * EVERY EDGE IS DENSIFIED AND CHECKED, at the same 0.05 rad as
    `safe_motion`. An RRT that checks only its nodes has the same defect the
    old pick had, one level up.
  * SHORTCUT THEN RESAMPLE. Raw RRT output is jagged; a jagged path on a
    wearable rig is an arm that jerks next to somebody's head.

WHAT IT IS NOT. It is not a replacement for MoveIt's planning pipeline in
general -- OMPL is better tested, has more planners, and knows about more
constraint types. It is a planner that (a) exists, (b) uses THIS project's
own clearance definition rather than one that is known to be blind to the
pairs that matter, and (c) can be tested offline with constructed answers,
which is the only kind of test this repository accepts.

WHY NOT cuRobo. It is the strongest thing in this space -- GPU trajectory
optimisation, collision-free IK at ~9000 queries/s, motion generation in
tens of milliseconds -- and it wants 2.5 GB of VRAM at batch 1024 on a
machine that has 4 GB total shared with the detector, plus a CUDA PyTorch
stack where this project deliberately runs CPU torch. It is the right answer
on a bigger machine and it is written down in docs/system/24_motion_planning.md
rather than half-adopted here.
"""
from __future__ import annotations

import math

import numpy as np

CONTINUOUS_IDX = (0, 2, 4, 6)
FLOOR_M = 0.15
STEP_RAD = 0.05            # the same densification `safe_motion` uses


class PlanRefusal(Exception):
    """No path, and a reason a person can act on."""


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def delta(a, b, continuous=CONTINUOUS_IDX):
    """b - a, wrapped on the continuous joints.

    Joints 1/3/5/7 are continuous. Differencing them naively across the
    +/-pi seam reports 288 deg for a 69.5 deg move -- defect 5 of
    2026-08-21 -- and here it would make the planner travel the long way
    round on every seam crossing.
    """
    out = np.array(b, float) - np.array(a, float)
    for i in continuous:
        out[i] = wrap(out[i])
    return out


def dist(a, b):
    return float(np.linalg.norm(delta(a, b)))


class VoxelWorld:
    """What the camera saw, as an obstacle the planner can avoid.

    THE GAP THIS FILLS. The planner's only collision check was the WEARER.
    Everything else in the world -- the table the objects are resting on, the
    objects, the frame, a person's coffee -- was invisible to it, so a path
    that went round the wearer would happily sweep the arm through the bench.
    The wearer is what the SAFETY case is about; it is not the only thing the
    arm can hit.

    A voxel set rather than a mesh or an octree: it is a dict lookup per
    query, it needs no library, and its one parameter -- the voxel size -- is
    also the resolution of the answer, which makes it honest about what it
    knows.

    POINTS MUST ALREADY BE IN THE ROBOT FRAME. On this rig that means they
    came through the WRIST camera's forward-kinematic pose. The scene camera
    would give a far better view of the workspace and its extrinsic has 0%
    coverage on real recordings, so its points cannot be placed yet -- see
    docs/system/22_grasping.md. When it can, it plugs in here and nothing
    else changes.

    WHAT IT DELIBERATELY DOES NOT DO: clear stale voxels. A voxel world built
    from one frame describes one frame. Carrying occupancy forward between
    frames is how MoveIt's octomap ends up with obstacles that are no longer
    there and plans that fail for ever against a ghost.
    """

    def __init__(self, points, voxel_m=0.03, ignore_within_m=0.0,
                 origin=None):
        self.voxel_m = float(voxel_m)
        self.cells = set()
        self.n_points = 0
        if points is None:
            return
        P = np.asarray(points, float)
        if P.size == 0:
            return
        if ignore_within_m > 0.0 and origin is not None:
            # DO NOT MAKE THE ROBOT AN OBSTACLE TO ITSELF. The wrist camera
            # sees the gripper; voxelising it puts a permanent obstacle
            # exactly where the hand is and nothing can ever be planned.
            # MoveIt's depth updater has a self-filter for the same reason.
            d = np.linalg.norm(P - np.asarray(origin, float), axis=1)
            P = P[d > ignore_within_m]
        self.n_points = len(P)
        if len(P):
            self.cells = set(map(tuple,
                                 np.floor(P / self.voxel_m).astype(np.int64)))

    def __len__(self):
        return len(self.cells)

    def occupied(self, p, margin_m=0.0):
        """Is this point inside an occupied voxel, with a margin.

        The margin is applied by testing the neighbouring cells rather than
        by inflating the set, so the same world can be queried at different
        margins -- the arm's tube radius is not the gripper's.
        """
        v = self.voxel_m
        r = int(math.ceil(margin_m / v)) if margin_m > 0 else 0
        c = np.floor(np.asarray(p, float) / v).astype(np.int64)
        if r == 0:
            return tuple(c) in self.cells
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if (c[0] + dx, c[1] + dy, c[2] + dz) in self.cells:
                        return True
        return False

    def hits(self, pts, margin_m=0.0):
        """The first occupied point, or None. Returns the point so a refusal
        can say WHERE, which is the difference between "blocked" and an
        answer somebody can act on."""
        for p in pts:
            if self.occupied(p, margin_m):
                return np.asarray(p, float)
        return None


def world_from_map(m, voxel_m=0.03, ignore_within_m=0.0, origin=None):
    """A `VoxelWorld` from a `world_model.Map`. THE SEAM THAT NEVER EXISTED.

    `VoxelWorld` has been built and tested since it was written and is
    constructed NOWHERE outside its own self-test, so the planner has never
    known that the table, the objects or anything else in the room is there.
    It avoided the wearer and swept through everything else. This is the two
    lines that connect what the robot SAW to what the planner AVOIDS.

    The points are the map's own occupancy -- the fused support surface AND
    the objects, in the robot frame. Nothing is filtered out here: deciding
    what the planner may not see would be this function guessing at somebody
    else's safety margin, and `ignore_within_m` around the gripper's own
    origin is the mechanism that already exists for the one case that needs
    it (the wrist camera seeing its own fingers).
    """
    pts = m.occupancy() if hasattr(m, "occupancy") else m
    return VoxelWorld(pts, voxel_m=voxel_m,
                      ignore_within_m=ignore_within_m, origin=origin)


class Checker:
    """Is this configuration, and this edge, safe.

    `clearance(q) -> metres` is passed in so the planner never owns a second
    opinion about what "safe" means. In this project that function is
    `Scorer.clearance_parts(arm, q)["moving_chain_m"]` -- the MOVING chain,
    because the whole chain reads the immobile mount stub's constant
    0.2202 m in every pose and a floor compared against it can never fire.
    """

    def __init__(self, clearance, lo, hi, cont, floor=FLOOR_M,
                 step=STEP_RAD, world=None, link_points=None,
                 tube_r=0.050):
        self.clearance = clearance
        self.lo = np.asarray(lo, float)
        self.hi = np.asarray(hi, float)
        self.cont = list(cont)
        self.floor = float(floor)
        self.step = float(step)
        # THE OBSERVED WORLD, optional. `link_points(q) -> [(x,y,z), ...]`
        # gives the arm's own geometry for a configuration; without it the
        # world cannot be checked and this says so rather than silently
        # ignoring the obstacles it was handed.
        self.world = world
        self.link_points = link_points
        self.tube_r = float(tube_r)
        if world is not None and link_points is None:
            raise ValueError(
                "a VoxelWorld was given but no link_points(q) to check "
                "against it. Accepting the world and not using it would be "
                "a planner that reports obstacle avoidance it is not doing.")
        self.n_config = 0
        self.n_edge = 0
        self.last_hit = None

    def in_limits(self, q):
        """Only the NON-continuous joints have limits to violate.

        `srl_fk.limits()` hands back (-pi, pi) for a continuous joint as a
        SEARCH BOX and says so in its own docstring. Enforcing that box as a
        stop rejects any path that crosses the seam -- measured on the real
        right arm, it refused a direction the arm walks into without
        complaint.
        """
        q = np.asarray(q, float)
        for i in range(len(q)):
            if self.cont[i]:
                continue
            if q[i] < self.lo[i] - 1e-9 or q[i] > self.hi[i] + 1e-9:
                return False
        return True

    def ok(self, q):
        self.n_config += 1
        if not self.in_limits(q):
            return False
        if float(self.clearance(q)) < self.floor:
            return False
        if self.world is not None and len(self.world):
            pts = self.link_points(q)
            hit = self.world.hits(pts, margin_m=self.tube_r)
            if hit is not None:
                self.last_hit = hit
                return False
        return True

    def edge_ok(self, a, b):
        """DENSIFIED. An RRT that checks only its nodes has the same defect
        the old pick had: two clear poses and a sweep through the person
        between them."""
        self.n_edge += 1
        d = delta(a, b)
        n = max(1, int(math.ceil(float(np.abs(d).max()) / self.step)))
        for k in range(1, n + 1):
            if not self.ok(np.asarray(a, float) + d * (k / n)):
                return False
        return True


class _Tree:
    def __init__(self, root):
        self.nodes = [np.asarray(root, float)]
        self.parent = [-1]

    def nearest(self, q):
        d = [dist(n, q) for n in self.nodes]
        return int(np.argmin(d))

    def add(self, parent, q):
        self.nodes.append(np.asarray(q, float))
        self.parent.append(parent)
        return len(self.nodes) - 1

    def path_to(self, i):
        out = []
        while i >= 0:
            out.append(self.nodes[i])
            i = self.parent[i]
        return out[::-1]


def plan(start, goal, checker, max_iter=3000, extend_rad=0.30, seed=0,
         goal_bias=0.10):
    """RRT-Connect from `start` to `goal`. Returns a list of waypoints.

    Refuses rather than returning a path it has not checked. The three
    refusals are different facts and are reported as such: the START is
    unsafe, the GOAL is unsafe, or no connection was found.
    """
    start = np.asarray(start, float)
    goal = np.asarray(goal, float)
    if not checker.ok(start):
        raise PlanRefusal(
            "the START configuration is already unsafe: clearance %.4f m "
            "against a %.2f m floor. Nothing can be planned from here -- the "
            "arm has to be moved out first."
            % (checker.clearance(start), checker.floor))
    if not checker.ok(goal):
        raise PlanRefusal(
            "the GOAL configuration is unsafe: clearance %.4f m against a "
            "%.2f m floor. This is not a planning failure; the goal is "
            "inside the person." % (checker.clearance(goal), checker.floor))

    # THE STRAIGHT LINE FIRST. Most moves on this rig are fine, and a planner
    # that always builds a tree turns a 60 ms move into a 2 s one.
    if checker.edge_ok(start, goal):
        return [start, goal], {"method": "straight line",
                               "configs": checker.n_config,
                               "edges": checker.n_edge}

    rng = np.random.default_rng(seed)
    ta, tb = _Tree(start), _Tree(goal)
    for it in range(max_iter):
        target = goal if (rng.random() < goal_bias) else rng.uniform(
            checker.lo, checker.hi)
        ia = ta.nearest(target)
        qa = ta.nodes[ia]
        d = delta(qa, target)
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            continue
        qn = qa + d * min(1.0, extend_rad / n)
        if not checker.edge_ok(qa, qn):
            ta, tb = tb, ta
            continue
        ia = ta.add(ia, qn)
        # CONNECT: reach for the other tree, not for a random point.
        ib = tb.nearest(qn)
        if checker.edge_ok(tb.nodes[ib], qn):
            pa = ta.path_to(ia)
            pb = tb.path_to(ib)
            path = pa + pb[::-1]
            # Whichever tree grew from `start` decides the direction.
            if dist(path[0], start) > dist(path[-1], start):
                path = path[::-1]
            path = shortcut(path, checker, rng)
            return path, {"method": "RRT-Connect, %d iterations" % (it + 1),
                          "configs": checker.n_config,
                          "edges": checker.n_edge}
        ta, tb = tb, ta
    raise PlanRefusal(
        "no path in %d iterations. The start and the goal are both safe, so "
        "either they are in separate free regions or the corridor between "
        "them is narrower than the %.2f rad extension. %d configurations and "
        "%d edges were checked."
        % (max_iter, extend_rad, checker.n_config, checker.n_edge))


def shortcut(path, checker, rng, rounds=120):
    """Cut corners the tree put in, and only where the edge checks out.

    Raw RRT output is jagged, and a jagged path on a wearable rig is an arm
    that jerks next to somebody's head. Every shortcut is verified with the
    same densified check as the original edges, so this cannot introduce a
    breach.
    """
    p = [np.asarray(q, float) for q in path]
    for _ in range(rounds):
        if len(p) <= 2:
            break
        i = int(rng.integers(0, len(p) - 2))
        j = int(rng.integers(i + 2, len(p)))
        if checker.edge_ok(p[i], p[j]):
            p = p[:i + 1] + p[j:]
    return p


def resample(path, step=0.10):
    """Even spacing, so the follower's step limiter has nothing to do.

    The follower clamps to `max_step_rad` per cycle. Handing it waypoints
    further apart than that means it invents the intermediate motion itself,
    with a straight joint-space interpolation nobody checked -- which is the
    defect this planner exists to remove, reintroduced at the last step.
    """
    if len(path) < 2:
        return [np.asarray(q, float) for q in path]
    out = [np.asarray(path[0], float)]
    for a, b in zip(path[:-1], path[1:]):
        d = delta(a, b)
        n = max(1, int(math.ceil(float(np.abs(d).max()) / step)))
        for k in range(1, n + 1):
            out.append(np.asarray(a, float) + d * (k / n))
    return out


def path_length(path):
    return float(sum(dist(a, b) for a, b in zip(path[:-1], path[1:])))


# ---------------------------------------------------------------- self-test
def self_test(verbose=True):
    """Constructed obstacles with known answers, in 7 dimensions."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-54s %s%s" % (name, "OK" if cond else "*** FAILED ***",
                                    (" -- " + detail) if detail else ""))

    lo = -np.ones(7) * 2.0
    hi = np.ones(7) * 2.0
    cont = [False] * 7

    # A SLAB the straight line must cross: everything with |q0| < 0.35 and
    # q1 < 0.5 is inside the "wearer". Going round means lifting q1.
    def clearance(q):
        q = np.asarray(q, float)
        if abs(q[0]) < 0.35 and q[1] < 0.5:
            return 0.0
        return 1.0

    ch = Checker(clearance, lo, hi, cont)
    start = np.array([-1.0, 0.0, 0, 0, 0, 0, 0.0])
    goal = np.array([1.0, 0.0, 0, 0, 0, 0, 0.0])

    check("the straight line between them is NOT safe",
          not ch.edge_ok(start, goal))
    path, info = plan(start, goal, ch, seed=1)
    check("a path is found round the obstacle", len(path) >= 2, info["method"])
    check("every edge of the returned path is safe",
          all(ch.edge_ok(a, b) for a, b in zip(path[:-1], path[1:])))
    check("the path actually goes round (q1 lifts above 0.5)",
          max(float(q[1]) for q in path) >= 0.5,
          "max q1 = %.2f" % max(float(q[1]) for q in path))
    check("it starts at the start and ends at the goal",
          dist(path[0], start) < 1e-9 and dist(path[-1], goal) < 1e-9)

    # A FREE MOVE MUST NOT BUILD A TREE.
    ch2 = Checker(lambda q: 1.0, lo, hi, cont)
    p2, i2 = plan(start, goal, ch2, seed=1)
    check("a clear move is a straight line, not a search",
          i2["method"] == "straight line" and len(p2) == 2, i2["method"])

    # REFUSALS, each naming which end is the problem.
    bad_start = np.array([0.0, 0.0, 0, 0, 0, 0, 0.0])
    try:
        plan(bad_start, goal, Checker(clearance, lo, hi, cont))
        check("an unsafe START is refused", False)
    except PlanRefusal as e:
        check("an unsafe START is refused by name", "START" in str(e),
              str(e)[:52])
    try:
        plan(start, bad_start, Checker(clearance, lo, hi, cont))
        check("an unsafe GOAL is refused by name", False)
    except PlanRefusal as e:
        check("an unsafe GOAL is refused by name",
              "GOAL" in str(e) and "not a planning failure" in str(e),
              str(e)[:52])

    # A GENUINELY SEALED goal must refuse rather than loop for ever.
    def sealed(q):
        return 0.0 if float(np.asarray(q, float)[0]) > 0.5 else 1.0
    try:
        plan(start, goal, Checker(sealed, lo, hi, cont), max_iter=250)
        check("an unreachable goal is refused", False)
    except PlanRefusal as e:
        check("an unreachable goal is refused, saying what it tried",
              "iterations" in str(e) or "GOAL" in str(e), str(e)[:52])

    # SHORTCUTTING must shorten and must not break safety.
    jag = [start,
           np.array([-1.0, 1.4, 0, 0, 0, 0, 0.0]),
           np.array([0.0, 1.4, 0, 0, 0, 0, 0.0]),
           np.array([1.0, 1.4, 0, 0, 0, 0, 0.0]),
           goal]
    ch3 = Checker(clearance, lo, hi, cont)
    sc = shortcut(jag, ch3, np.random.default_rng(2))
    check("shortcutting shortens the path",
          path_length(sc) <= path_length(jag) + 1e-9,
          "%.3f -> %.3f rad" % (path_length(jag), path_length(sc)))
    check("and every shortcut edge is still safe",
          all(ch3.edge_ok(a, b) for a, b in zip(sc[:-1], sc[1:])))

    # RESAMPLING must respect the step and preserve the ends.
    rs = resample(sc, step=0.10)
    worst = max(float(np.abs(delta(a, b)).max())
                for a, b in zip(rs[:-1], rs[1:]))
    check("resampling never exceeds the step", worst <= 0.10 + 1e-9,
          "worst %.4f rad" % worst)
    check("resampling keeps the endpoints",
          dist(rs[0], sc[0]) < 1e-9 and dist(rs[-1], sc[-1]) < 1e-9)

    # THE SEAM. A continuous joint must take the short way.
    cont2 = [True] + [False] * 6
    ch4 = Checker(lambda q: 1.0, -np.ones(7) * 4, np.ones(7) * 4, cont2)
    a = np.array([math.radians(170.0)] + [0.0] * 6)
    b = np.array([math.radians(-170.0)] + [0.0] * 6)
    p4, _ = plan(a, b, ch4)
    check("a seam crossing travels 20 degrees, not 340",
          abs(math.degrees(path_length(p4)) - 20.0) < 1e-6,
          "%.1f deg" % math.degrees(path_length(p4)))

    # AND THE CHECK THAT MATTERS: the planner must never hand back a path
    # whose MIDDLE is unsafe, which is the whole reason it densifies.
    def narrow(q):
        q = np.asarray(q, float)
        return 0.0 if (abs(q[0]) < 0.6 and abs(q[1] - 0.25) < 0.08) else 1.0
    ch5 = Checker(narrow, lo, hi, cont)
    p5, _ = plan(np.array([-1.0, 0.25, 0, 0, 0, 0, 0.0]),
                 np.array([1.0, 0.25, 0, 0, 0, 0, 0.0]), ch5, seed=3)
    dense = resample(p5, step=0.02)
    check("no sample anywhere on the final path is unsafe",
          all(ch5.ok(q) for q in dense),
          "%d samples checked" % len(dense))

    # ---- THE OBSERVED WORLD. The planner's only obstacle was the wearer,
    #      so a path that went round a person would sweep through the bench.
    #
    # A one-dimensional arm for the purpose: link_points puts a single point
    # at (q0, 0, 0), so a wall of voxels at x = 0 must block it.
    def one_d_links(q):
        return [np.array([float(np.asarray(q, float)[0]), 0.0, 0.0])]

    wall = np.array([[0.0, y * 0.01, z * 0.01]
                     for y in range(-8, 9) for z in range(-8, 9)])
    world = VoxelWorld(wall, voxel_m=0.03)
    check("a voxel world is built from points", len(world) > 0,
          "%d cells from %d points" % (len(world), world.n_points))
    check("a point in the wall is occupied", world.occupied([0.0, 0.0, 0.0]))
    check("a point well clear is not", not world.occupied([1.0, 0.0, 0.0]))
    check("the margin reaches further than the voxel",
          world.occupied([0.10, 0.0, 0.0], margin_m=0.12)
          and not world.occupied([0.10, 0.0, 0.0], margin_m=0.0),
          "0.10 m away: blocked at a 0.12 m margin, clear at 0")

    free = Checker(lambda q: 1.0, lo, hi, cont)
    blocked = Checker(lambda q: 1.0, lo, hi, cont, world=world,
                      link_points=one_d_links, tube_r=0.05)
    check("without a world the wall is invisible",
          free.edge_ok(np.array([-1.0] + [0.0] * 6),
                       np.array([1.0] + [0.0] * 6)))
    check("WITH a world the same edge is blocked",
          not blocked.edge_ok(np.array([-1.0] + [0.0] * 6),
                              np.array([1.0] + [0.0] * 6)),
          "and the hit is reported at %s"
          % (np.round(blocked.last_hit, 3) if blocked.last_hit is not None
             else "nowhere"))
    check("the refusal says WHERE it was blocked",
          blocked.last_hit is not None
          and abs(float(blocked.last_hit[0])) < 0.10,
          "hit at x = %.3f"
          % (blocked.last_hit[0] if blocked.last_hit is not None else -99))

    # A WORLD THAT CANNOT BE CHECKED MUST BE REFUSED, not ignored.
    try:
        Checker(lambda q: 1.0, lo, hi, cont, world=world)
        check("a world with no way to check it is refused", False,
              "it accepted obstacles it cannot use")
    except ValueError as e:
        check("a world with no way to check it is refused",
              "not using it" in str(e) or "reports obstacle avoidance"
              in str(e), str(e)[:56])

    # AN EMPTY WORLD IS NOT AN OBSTACLE.
    empty = Checker(lambda q: 1.0, lo, hi, cont,
                    world=VoxelWorld(np.zeros((0, 3))),
                    link_points=one_d_links)
    check("an empty world blocks nothing",
          empty.edge_ok(np.array([-1.0] + [0.0] * 6),
                        np.array([1.0] + [0.0] * 6)))

    # AND THE SELF-FILTER: the camera sees the gripper, and voxelising it
    # would put a permanent obstacle exactly where the hand is.
    near = np.array([[0.02, 0.0, 0.0], [0.5, 0.0, 0.0]])
    w2 = VoxelWorld(near, voxel_m=0.03, ignore_within_m=0.10,
                    origin=[0.0, 0.0, 0.0])
    check("points within the self-filter radius are dropped",
          w2.n_points == 1 and not w2.occupied([0.02, 0.0, 0.0])
          and w2.occupied([0.5, 0.0, 0.0]),
          "%d of 2 points kept" % w2.n_points)

    if verbose:
        print("joint_planner self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if self_test() else 1)
