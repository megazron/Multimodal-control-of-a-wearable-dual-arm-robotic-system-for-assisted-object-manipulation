# How good is this system, and what still stops it being controlled

Measured 2026-08-22. `python3 scripts/measure_control_budget.py` regenerates
every number here; `recordings/baselines/control_budget.json` is the output.

Nothing below is an estimate. Where a term has never been measured it says
UNMEASURED, because a budget with a guessed term hides the thing you should
go and measure.

---

## THE SHORT ANSWER

**Three things are good, and three are not, and the three that are not are
all about a person moving.**

| | verdict |
| --- | --- |
| Can it see a table and say what is on it? | **Yes.** Any height 0.40–1.40 m to 0.1 mm; object centres to 0.9 mm; widths exact for every shape tried |
| Can it plan a safe grasp? | **Yes**, and it refuses the unsafe ones by name — including a sweep whose two endpoints are clear and whose middle goes through the wearer |
| Can the arms move freely? | **Now**, mostly. 0.065 → 0.304 m mean reach at the work point since the wrist stopped being pinned |
| Will the hand land where it was told? | **Not inside the grasp gate.** Worst case **33.4 mm against a 30 mm gate**, and one term is still unmeasured |
| Does the guard know where the wearer is? | **Not in time.** In 5 of 6 motion cases the limb travels further than the *entire* 150 mm floor before the guard hears about it |
| Is the assumed posture safe if tracking drops? | **No.** Arms held out to the sides give **93 mm** clearance at home — a 57 mm breach — and the fallback assumes 317 mm |

---

## 1. WHERE THE HAND ENDS UP: 33.4 mm against a 30 mm gate

| term | mm | kind |
| --- | --- | --- |
| declared vs measured pad offset (T0, T2, T3) | 13.45 | systematic |
| terminal joint error, uncompensated | 7.24 | systematic |
| shell bias from one view, **before** the plane | 10.5 | systematic |
| — the same, **with** the support plane | **0.0** | — |
| — joint error, **if** the overshoot is switched on | **1.88** | — |
| depth noise at 0.4–0.8 m | 2.0 | random |
| IK solve residual | 0.2 | random |
| **`camera_link` vs the physical module** | **UNMEASURED** | systematic |

RSS of the measured terms **18.6 mm**; worst case, because they add,
**33.4 mm**. The gate is 30 mm.

**Read that as: the system is over budget on paper, and two of the three
biggest terms already have fixes sitting switched off.** With the support
plane in the path (0.0 mm) and the terminal overshoot enabled (1.88 mm), the
worst case falls to about **17 mm** — comfortably inside — *provided* the
unmeasured term is small.

**The unmeasured term is the one to go and measure.** The wrist camera's pose
is forward kinematics, which is exact given the URDF, and nothing has ever
checked the URDF against the physical module. A 10 mm error there is
invisible and lands in every grasp. It is the cheapest high-value calibration
left in the project.

---

## 2. THE WEARER MOVES FASTER THAN THE GUARD CAN HEAR

Tracking latency is **62–562 ms** end to end (measured; the spread is the
`scene_hz` rate limit). Limb speeds are literature ranges and are labelled as
assumptions, not measurements.

| limb motion | latency | travel | vs the 150 mm floor |
| --- | --- | --- | --- |
| a deliberate reach | 62 ms | 50–93 mm | 62% of it |
| a deliberate reach | 562 ms | 450–843 mm | **exceeds it** |
| a quick reach or fidget | 62 ms | 93–155 mm | **exceeds it** |
| a quick reach or fidget | 562 ms | 843–1405 mm | **exceeds it** |
| a startle or flinch | 62 ms | 155–248 mm | **exceeds it** |
| a startle or flinch | 562 ms | 1405–2248 mm | **exceeds it** |

**5 of 6 cases move further than the whole floor before the guard knows.**

The floor is a *distance*, not a reaction time, and nothing in this system
reacts to a wearer who moves quickly. That is the single most important
control problem in the project, and it is not a software bug — it is a
consequence of a 150 mm margin, a ~0.5 s perception chain, and a person who
can move a metre in that time.

What would actually help, in order of honesty:

1. **Inflate the floor by the reaction budget.** The guard should keep
   `floor + latency × assumed_speed`, not a fixed 150 mm. That is a
   one-parameter change and it makes the margin mean something.
2. **Cut the latency.** 562 ms is the rate limit, not the tracker — the
   tracker itself is 28–34 ms/frame. Most of that spread is recoverable.
3. **Slow the arm near the person.** Speed is already a live parameter; the
   clearance is already computed. Nothing currently couples them.

None of these is done. They are named here so the next session does not have
to rediscover the arithmetic.

---

## 3. LOSING TRACKING SILENTLY LOOSENS THE GUARD

Clearance from the arm at HOME to the wearer, by the wearer's own arm
posture — the same five settings `SRL_WEARER_ARMS` offers, computed through
the guard's own geometry:

| posture | clearance | vs assumed |
| --- | --- | --- |
| **out** (arms held out to the sides) | **0.0933 m** | **−224 mm — BREACHES the floor** |
| behind (clasped behind the back) | 0.2779 m | −39 mm |
| **down** (the assumed model) | 0.3170 m | — |
| folded | 0.3211 m | +4 mm |
| none | 0.3211 m | +4 mm |

Identical on both arms.

`fuse()` is correct about its own rule — *the camera may only make the wearer
bigger* — and it falls back to the mannequin when the estimate is stale,
silent or gated out. **That fallback is the gap.** If the wearer has raised
their arms and tracking drops, the guard reverts to `down` and believes there
is 317 mm of clearance where there is 93 mm.

The system's current answer to this is an instruction to the wearer, recorded
in HARD CONSTRAINT 11: *do not tell a wearer to clasp their arms behind their
back or hold them out to the sides.* That is a real mitigation and it is not
a control system.

**The honest statement: markerless tracking is not a nicety here, it is load
bearing — and its failure mode is to quietly restore an assumption that is
224 mm optimistic.**

---

## 4. HOW FREE THE ARMS ARE

Mean reach over the direction walk, wearer floor enforced throughout, through
the follower's own candidate list:

| policy | at the work point (12 walks) | at home (28 walks) |
| --- | --- | --- |
| pinned (before 2026-08-22) | 0.065 m | 0.460 m, 6 wearer-bound |
| **cone 15° (the default now)** | **0.304 m** | 0.492 m, 6 wearer-bound |
| cone 45° | 0.325 m | 0.530 m |
| position only | 0.362 m | 0.549 m |

At home the arms were always reasonably free and the wrist cost 89 mm. **At
the point where the work happens it cost a factor of seven**, and the
mechanism was that a fixed 6-DOF pose on a 7-DOF arm spends the whole
redundancy — the joint whose purpose is moving the elbow out of the person.

---

## 5. THE OPERATOR SITTING ACROSS THE ROOM

Desk operation: the operator sits **facing** the wearer, holds the controllers
as motion capture, and watches the robot directly. Nobody wears the headset.
What makes that hard, in the order it bites:

1. **They are facing the wearer, so copying is a reflection.** `align_yaw_deg`
   is a yaw and never a mirror — a mirror is determinant −1 and would flip
   every *orientation* while positions still looked right. The parameter is
   measured by `calibrate_operator_yaw.py`, which refuses a motion under
   100 mm or 35° off horizontal.
2. **The repository disagrees with itself about which way is right.** Whether
   world +x is the wearer's right is an OPEN contradiction: the docs say it
   is and the arms sit the other way round. Until that is settled, an
   operator's "move it left" is ambiguous in the source.
3. **The tracking reference can be knocked and announces nothing.** The
   headset on the shelf *is* the reference. A nudge leaves poses valid, rate
   at 90 Hz, controllers tracked — and every pose after it in a rotated
   frame. `vr_safety_node` latches the first HMD pose and freezes past
   20 mm / 2°, and the freeze survives the ordinary unfreeze path.
4. **There is no force feedback of any kind.** The operator learns that the
   gripper has hit something by watching it.
5. **The arm stops 7 mm short of every commanded move**, so a deliberate
   correction of that size does nothing visible and the operator applies
   another.
6. **The operator cannot see the wearer's clearance.** It is computed, it is
   published, and until this session nothing drew it as geometry. `/viz/wearer`
   now does.

---

## WHAT WOULD MOVE THE NEEDLE MOST

Ranked by measured value, not by interest:

1. **Hand-eye the wrist camera.** The only unmeasured term in the positioning
   budget, and it lands in every grasp.
2. **Couple arm speed to measured clearance.** The two numbers exist and
   nothing multiplies them.
3. **Inflate the floor by the reaction budget** rather than keeping a fixed
   150 mm that a limb crosses in 100 ms.
4. **Enable the terminal overshoot and re-measure** — four runs settle it,
   and it is 7.2 → 1.9 mm.
5. **Re-capture the extrinsic background with the arms stowed.** The scene
   camera is coarse aiming only until then, and the fix is a recording, not
   a threshold.
6. **Get one wrist-camera recording of a real table.** Everything in
   `table_scene` is validated on constructed ground truth and nothing else.
