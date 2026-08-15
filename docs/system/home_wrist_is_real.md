# The home wrist. Two poses now, and both are real.

**Read this before quoting any home angle, and read section 1 before going to
the lab.**

Since 2026-08-15 there are two home poses and they are both correct:

| | pose | wrist | authoritative for |
| --- | --- | --- | --- |
| **SIM home** | the presentation pose | **LEVEL, 0.0 deg** | everything in this repository |
| **REAL home** | the legacy Kortex home | **UP, +85 / +79 deg** | the physical arms, until they are recaptured |

They differ, deliberately and temporarily, by about 1.94 rad. Nothing is
broken; the recapture has not happened yet.

## 1. THE OLD HOME IS STILL GROUND TRUTH FOR THE REAL ARMS

The legacy Kortex home is what the physical arms hold:

    left   259.03  277.69  267.74  286.14  194.10   27.48   55.26
    right  303.65   77.06   98.57   58.57  317.14   36.39  154.71

That is not superseded data. It is where the metal is, and it stays ground
truth for the hardware until somebody drives the arms to the new pose and
confirms it. The old warning — never edit home angles to make geometry look
right — was correct, and it is why the 2026-08-15 change was made by
re-measuring the whole task set first rather than by editing a file.

**The gap cannot be walked into.** `sim_to_real_bridge.enable()` compares the
REAL arm against the loaded (sim) home with `require_homed`, tolerance 0.05
rad, and REFUSES. Measured: **1.9445 rad (111.4 deg) on joint_7 left, 1.9380
right** — thirty-nine times the tolerance. A second gate compares the sim's
delayed target against the real arm and refuses on the same gap. The refusal
message names the likely cause and points at the runbook, because this is the
expected state of the rig rather than an operator error.

Verified by `test_bridge_refuses_the_home_change.py`, which exercises the
gate's predicate against both poses and carries a control proving the gate can
still open. **It tests the arithmetic `enable()` performs, not the node's
lifecycle** — the node has never been stood up against real hardware, because
no arm in this project ever has.

**The recapture is the first item in `docs/NEXT_SESSION.md`**, with the target
in Kortex degrees and radians so nothing has to be re-derived on the day.

## 2. THE SIM HOME IS THE PRESENTATION POSE

Wrists level to 0.0 deg, hands in front of the chest at (±0.550, 0.360,
1.180), limb apex 118 mm below the wearer's shoulder line, wearer clearance
0.1610 m — which is the ceiling, since nothing on the arm can be further from
the wearer than the arm's own base.

Confirmed in the live stack after the change: the arms come up 0.00007 rad
from the file and the tool axis reads **-0.01 deg (left) / -0.00 (right)**,
against +30.8 / +22.1 before.

## 3. THE WRIST-UP ORIENTATION REMAINS THE GRASP APPROACH, AND THAT WAS NEVER THE ISSUE

This is the part that matters most and the part easiest to get backwards.
**The approach direction is not the resting pose.**

| figure | what it measures | from | changed 2026-08-15? |
| --- | --- | --- | --- |
| +85 / +79 deg → 0.0 | the WRIST's elevation at the SIM home | the horizontal | **yes** |
| **30.7 / 22.1 deg** | the pinned APPROACH axis, the tool direction every commanded pose holds | the shoulder | **no** |

`master_calibration.WORKSPACE_ORIENT` is read ONCE, in
`master_pose_node.__init__`, to seed ROS parameters. Nothing recomputes it
from the live home. So a level home does not force a level grasp: the arm
rests level and rotates its wrist to the pinned approach on the way to the
work.

**Measured, because this is the claim the whole change rests on**
(`measure_home_change.py`, N=10 over every distinct waypoint of each task's
own builder, wearer and furniture in scene, clearance geometric, arms staged
with arrival verified off `/joint_states`, and a known-answer control
reproducing the committed 2026-08-15 numbers on eight task/arm pairs):

| task | home OLD / anchor kept | home NEW / anchor kept | home NEW / anchor re-derived to level |
| --- | --- | --- | --- |
| T0 | 8 IK failures | 8 | 6 |
| T1 | 0 | **0** | 0 |
| T1 stage 2 | 0 | **0** | 1 |
| T2 | 0 | **0** | **4 — the right arm is lost** |
| T3 | 0 | **0** | 0 |

Keeping the approach costs nothing. Re-deriving it to match the level home
costs T2 its right arm, because the near-side approach is what reaches the
tray — 0/2 under top-down was already on record. **So `WORKSPACE_ORIENT` must
not be re-derived from home.** If anyone proposes it "for consistency", that
row is the answer.

T0's eight failures are not caused by any of this. They are present at the old
home too; that measurement was the first time anyone walked T0's own waypoint
list against collision-aware IK.

## 4. WHAT MOVED WITH IT, AND WHAT DID NOT

**Moved:** `config/home_positions_{left,right}.txt` (the source), both
`initial_positions` blocks of `srl_dual.urdf.xacro`, and the two nodes that
carried their own hardcoded copy — `pot_bridge.py` and `srl_teleop_node.py`,
both of which publish to the arm controllers and would otherwise have driven
the arms to the superseded pose. `test_home_has_one_source.py` now fails if a
sixth copy appears, if the URDF and the config drift apart, or if a continuous
joint is stored outside ±π.

**THE "UPDATE BOTH PLACES" TRAP WAS UNDERCOUNTED, AND THE COUNT MATTERED.**
The pose lived in five places, not two, and the two nobody names —
`pot_bridge.py` and `srl_teleop_node.py` — are installed executables that no
launch file references. Nothing exercised them, so nothing would have caught
the drift, and each is one `ros2 run` away from commanding an arm.
`config/home_positions_*.txt` is the opposite of ignored: five live runtime
consumers load it, including the bridge's enable gate and the target
`real_homing_node` drives the REAL arm to.

**The right arm's joint_7 is stored WRAPPED**, −94.25 deg rather than the
265.75 the pose search returned. Same physical pose — joint_7 is continuous —
but `ik_follower_node` refuses to start on real hardware with a continuous
joint outside ±π and unwinds 360 deg in sim. The Kortex value is 265.75 either
way, so the lab capture is unambiguous. The left arm's joint_5 seam margin
IMPROVED, 14.10 → 28.01 deg.

**Did not move:** no task coordinate, no clearance figure, none of the 579
region cells. They derive from the ANCHOR, and nothing computes them from home
at runtime — `PAD_OFFSET_BY_ARM` and `WORKSPACE_ORIENT` are stored constants.
The coupling to home is derivation HISTORY, not live computation.

**`WORKSPACE_CENTRE` is the one that had to follow**, because it is documented
as `offset = P_HOME` — the world point the master's rest maps to, and
`master_pose_node` seeds `pos_anchor`, `anchor_ref` and `last_pos` from it.

## If you are here because the arms look wrong in RViz

1. a stray `robot_state_publisher` owning `/robot_description` — it makes
   every joint read 0.000, which is also plausible;
2. a stale `move_group` from a previous stack;
3. the mount, the usual suspect when geometry looks wrong;
4. **whether the workspace was rebuilt** — `initial_positions` is baked at
   launch, so an unbuilt change leaves the arms at the old pose while every
   text file says otherwise.

The home angles are still the last thing to suspect, not the first.
