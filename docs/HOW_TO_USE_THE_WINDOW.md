# How to use the window

One page. No jargon. If something here does not match what you see on screen,
the window is right and this page is wrong — tell somebody.

```
bash scripts/start_gui.sh
```

That is the only command. It sources everything, refuses to open a second
window, and starts the robot's view alongside the controls.

---

## The three tabs on the left

| tab | what it is for |
| --- | --- |
| **SET UP** | things you do once at the start of a session and then leave alone |
| **RUN** | what you press while you are actually working |
| **STATUS** | read-only. Nothing here changes the robot. |

---

## The normal working order

Everything on the RUN tab is numbered in the order you use it.

### 1. FULL SCAN *(once per table)*

The robot looks over the whole table and works out what is on it.

It first finds the table itself — wherever it is, whatever height — then walks
over it in straight rows at two heights and from three angles, holding the
wrist still at each stop. It photographs each object, works out how wide it
is, and says which ones the gripper can actually close on.

**Takes several minutes.** You only need it once for a given table. After that
use QUICK CHECK.

Pick **both arms** unless you have a reason not to: the two arms cannot reach
across each other, so one arm alone leaves half the table unmeasured — and
unmeasured is not the same as empty.

### REMEMBER VIEW

Press this when the table is set up the way you want it. It takes a picture
from the fixed camera above the table and keeps it. Everything after this is
measured as a change from that picture.

### 2. QUICK CHECK *(what moved)*

Spots what has changed and only re-checks that.

The fixed camera compares the table with the picture it remembered — which
takes no arm movement at all — and the arms then visit only the few positions
that can see the parts that changed.

**About 30 seconds**, against 18 minutes for a full scan. It tells you what
appeared, what went, and what moved.

If nothing changed it says so and moves nothing.

### LIST WHAT IT FOUND

Shows everything the robot currently believes is on the table, with sizes, and
which ones it can pick up. Moves nothing.

Each line is `number — position — width — can it be picked up`. If it cannot,
the reason is written next to it, usually "too wide for the gripper".

### 3. PICK IT UP

Type the **number** of the object from the list, choose **left arm** or
**right arm**, and press it.

The arm comes **down onto the object from above**, as steeply as it can
manage, using where the robot *saw* the object — not a position typed into a
file.

**Nothing moves unless you tick "really move the arm".** Leave it off and you
get the whole plan printed, including the angle it would approach at, without
the robot doing anything.

---

## The line at the top of that panel

It says what the robot is doing, in words, as it does it — "looking at cell 7
of 24", "closing on the object", "finished". If it has gone quiet it tells you
how long ago it last said anything.

---

## Other things on the RUN tab

**QUICK LOOK** — a single glance from wherever the gripper is now. No arm
movement, no full scan. Useful for "what is right in front of this hand". For
the whole table, use FULL SCAN.

**SAY WHAT TO PICK UP** — type it in your own words, like *the green cube*.

**OPERATE** — the four ways the robot can be driven. Each row is one mode; you
pick a row, not a stack of them.

| | |
| --- | --- |
| **1 MASTER TELEOP** | you move the instrumented arm, the robot copies |
| **2 VR / DESK TELEOP** | you hold the VR controllers; nobody wears the headset |
| **3 SHARED AUTONOMY** | you drive, the robot helps |
| **4 FULL AUTONOMY** | you say what you want in a sentence |

The three buttons on each row are how far the command goes:

* **SIM ONLY** — the simulated robot moves. Nothing physical.
* **SIM + MOCK** — as above, plus pretend hardware. For rehearsing.
* **SIM + REAL** — the real arms move.

---

## If something goes wrong

**E-STOP** — bottom left, always visible, always works. Press it.

**reset e-stop** — clears it once you are ready again.

**Something is wrong? Checks and fixes** — on the SET UP tab. It names the
problem in plain words and gives you a button that fixes it.

---

## What the robot will refuse to do, and why

It is meant to refuse. A refusal always says what is wrong.

* **"the table is beyond the arms' reach"** — the table has been found, and it
  is somewhere the arms physically cannot work. Move the table or move the
  wearer. Nothing was scanned.
* **"this map was measured N minutes ago"** — you asked it to move using an
  old picture of the table. Run QUICK CHECK first.
* **"no scene reference"** — press REMEMBER VIEW first, or there is nothing to
  compare against.
* **"NOT graspable — 140 mm across"** — the object is wider than the gripper
  opens. It is not a fault.
* **an object listed as "weak"** — the robot saw something, but barely. It is
  reported rather than hidden, and not offered for picking.

---

## The two camera panels at the top

**Gripper cameras** — what each hand can see, live. Under each picture it says
`live`, the frame rate, the size, and **which topic the picture came from**.
That last part matters: three possible names are subscribed, because what a
real Kinova wrist camera publishes has never been observed on this machine.
Whichever one delivers is the one printed.

If it says **NO CAMERA — no frame has ever arrived**, nothing is publishing.
Start the cameras, or check the name in that caption against what your camera
actually publishes.

---

## What this system cannot do yet

Said plainly, because a window that hides its limits is worse than one that
does not.

* **No camera has ever been attached to this machine.** Everything above runs
  against a simulated camera. The pipeline is real; the room is not.
* **The arms cannot reach across the middle.** Left arm, left side; right arm,
  right side.
* **It cannot grasp straight down.** Steeply from above, yes — about 20° off
  vertical. Straight down does not reach.
* **A table behind the wearer has never been measured.** The arms can put a
  camera back there, but whether that is safe has not been established.
* **PICK IT UP moves the SIMULATED arm.** The real-arm bridge listens on a
  different topic (`/real/...`), so nothing here has ever commanded hardware.
  Driving the real arms needs `--drive-real` on the command line, and that
  path is untested. See `docs/system/26_will_it_work_on_the_real_robot.md`,
  which lists every blocker in the order it will bite.
