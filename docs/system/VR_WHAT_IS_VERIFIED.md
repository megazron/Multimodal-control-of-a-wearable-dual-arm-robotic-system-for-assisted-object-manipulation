# The VR path: what is verified, and what needs the hardware

Written 2026-08-20 with **no headset, no controllers and no arms in the
building**. The point is to find the surprises today rather than in the lab.

---

## 1. VERIFIABLE WITHOUT HARDWARE — and now verified

| | how | result |
| --- | --- | --- |
| the twelve-step bring-up, every decision | `test_vr_bringup_sequence.py`, injected worlds | **33 tests** |
| the bring-up, wired to the real machine, from a DIRTY machine | `verify_vr_one_button.py`, nothing typed | **22 checks, 0 failed**, 10 of 10 machine-side steps, **three repairs pressed automatically on the way** |
| every GUI button including the new one | `verify_gui_buttons.py` | **191 checks, 0 failed** |
| the transport, page and socket on one origin | `verify_vr_wifi_route.py` | previously green |
| the frame conversions, WebXR and Unity | `test_webxr_frames.py`, `test_quest_frames.py` | constructed truth |
| the operator alignment is a ROTATION, never a mirror | `test_operator_alignment_is_a_rotation.py` | pinned over ±720° |
| the mapper resets between runs | `test_vr_mapper_resets_between_runs.py` | 9 tests |
| tracking-reference freeze, and that it survives unfreeze | `test_tracking_reference_stability.py` | |
| the observer heartbeat, and that SILENCE is withdrawal | `test_observer_estop_silence.py` | 6 tests |
| passthrough shares one session path with VR | `test_passthrough_is_an_addition.py` | 6 tests |
| every measurement analyser, against constructed truth AND broken input | `vr_guided_session.py --selftest` | passes |
| the clearance floor follows the measured body, and can only grow | `test_clearance_floor_follows_the_tracked_body.py` | 8 tests |

## 2. IMPOSSIBLE WITHOUT THE HARDWARE

Nothing below can be moved forward from a desk, and none of it should be
attempted for the first time with arms powered.

| | why | first measured by |
| --- | --- | --- |
| pose rate and latency, headset to bridge | needs a headset streaming | `vr_measure_session.sh rate` |
| the operator's yaw | needs a hand and a controller | `... yaw` |
| tracking-loss freeze latency | needs a controller to cover | `... freeze` |
| clutch re-engage jump in mm | needs a hand that grips, releases, moves, re-grips | `... clutch` |
| thumbstick scaling under the hand | needs a thumb | `... scale` |
| trigger on the gripper | needs a trigger | `... gripper` |
| whether `immersive-ar` is granted, and its blend mode | needs that headset | the page reports it |
| everything on the real arms | see §3 | |

---

## 3. THE REAL ARMS: what could still surprise you, and who tells you

**Nothing in this repository has ever run against a real Kinova arm.** The
column that matters is the last one.

| # | what | does the GUI tell you? |
| --- | --- | --- |
| 1 | **the bridge refuses to enable on the home gap** | **YES, and in words.** See §4 |
| 2 | the ros2 daemon hangs | **YES** — connection panel, with the fix |
| 3 | stale shared memory | **YES** — and the repair now also restarts the daemon |
| 4 | a second stack, or a stray robot description | **YES**, named separately |
| 5 | orphaned nodes from a previous run | **YES** — it lists them before stopping them |
| 6 | a leaked Kortex session | **YES** — "the last connection may still be held", with a release button |
| 7 | the Teensy moved sockets | **YES** (irrelevant to VR) |
| 8 | arms unreachable on 192.168.3.x | **NO — you run `check_arm_network.sh`.** The GUI does not ping the arms |
| 9 | `arm:=both` aborts the hardware load | **NO — you watch the terminal.** Rehearse with `--mock` first |
| 10 | homing slower or oscillating on real joints | **NO — YOU WATCH THE ARM.** The law was tuned against a first-order mock with no gearbox friction, no 17 kg payload and no gravity |
| 11 | the lag trip firing on normal following error | **PARTLY** — the divergence readout shows the number against the threshold, but the first trip may just be the number |
| 12 | the real gripper's force, travel and latching | **NO — you watch the gripper** |
| 13 | Ctrl-C during homing | **NO — try it deliberately before you need it** |
| 14 | **which physical arm is "right"** | **NO — YOU WATCH THE ARM.** Genuinely open: the docs say world +x is the wearer's right and the arms sit the other way round. This is why the first motion is 5 cm |
| 15 | the observer walking away | **YES**, since 2026-08-20 — silence past 2 s freezes |
| 16 | the clearance floor against a real person | **PARTLY** — `/wearer/enforced` and the GUI's Wearer tab say which body is being enforced, but with no scene camera it is the mannequin and every clearance figure is about that |
| 17 | a crashed process leaking its Kortex session | **NO** — the clean path is covered, a segfault is not |

**Six of seventeen you find by watching the arm.** They are 8, 9, 10, 12, 13,
14 and half of 11. Every one of them is in Part C of
`docs/system/VR_REAL_ARM_RUN.md`, before any teleoperation, with the wearer
out of the rig.

---

## 4. THE HOME GAP: what the GUI actually says

This is the one most likely to happen, so it is worth knowing exactly what you
will see.

**The connection panel row reads:**

> **The real arms are not where the simulation thinks**
>
> One joint on the left arm is 111 degrees away from the resting pose the
> simulation starts from. The cascade replays simulation angles onto the real
> arm, so it will refuse to switch on rather than command that difference as
> a jump. A gap of about 110 degrees on the last joint is the known one: the
> resting pose was changed in the simulation and the arms have not been
> taught the new one yet.
>
> `[ Move the arms to the resting pose ]`

**It is not a number.** It names the arm, says how far in degrees, says what
the system will do about it, and says the likely cause. The raw figure
(`worst left_joint_7 1.9400 rad`) is in the tooltip, for a bug report.

**The button does not change any stored value.** It asks the homing service to
drive the arms to the stored pose, after a confirmation naming what will move.
If the gap is the 2026-08-15 home change it will not close it — that needs a
recapture, which is a decision taken in the room with the wearer out, and the
procedure is `docs/NEXT_SESSION.md` §"CAPTURE THE NEW HOME ON THE REAL ARMS".

**The refusal itself is correct behaviour and is the system working.**

---

## 5. WHAT TODAY FOUND

Six defects, all in the thing that is supposed to prevent surprises. The first
two are the ones that would have cost the lab session.

1. **`READY TO OPERATE` with no headset in the building.** The last step
   checked that the controller pose topic was in the topic list — and the
   bridge creates its publishers at start-up, so the topic exists the moment
   the bridge does. The operator would have put the headset on, squeezed the
   grip, watched nothing happen, and had a window still saying ready. Now it
   counts arrivals over a window.

2. **Clearing stale shared memory wedges the daemon, and the button blamed the
   simulation.** Clearing invalidates every participant including the
   daemon's cached graph, so the next step's topic query returned nothing
   against a fully running stack and the sequence said "the simulation did not
   finish starting". False, specific and confident — the worst combination,
   because it sends you to the wrong log. The clear now restarts the helper,
   and the simulation step consults the process table before blaming the
   simulation.

3. **Pressing the button twice started a second mapper.** Every repair retries
   the sequence, and the start steps spawned unconditionally. By the fourth
   press the mapper was running twice — two publishers on the arm command
   topic, which is the one thing the VR architecture says must never happen.

4. **The panel dropped step results.** The worker handed them over one slot at
   a time and `refresh()` collected every 100 ms, so two fast steps
   overwrote each other and the first row never left "...".

5. **A clean motion scale is 0.5, not 1.0.** The check assumed 1.0, so a
   freshly reset mapping read as dirty — and the repair it offered was the
   reset that had just produced the value it objected to. The mapper now
   publishes what a clean scale is; nothing guesses.

6. **Three repairs that recreated the fault they repaired.** Found by running
   the button repeatedly from a deliberately dirty machine, which is the only
   way any of them show up:

   * clearing shared memory restarted the daemon, and restarting the daemon
     orphans its own segments -- so the next check offered to clear again.
     Four times in a row, fourteen blocks each. The restart now sweeps after
     itself, and after the NEW daemon is up: sweeping between the kill and the
     start cleared nothing, because the old daemon had not finished dying and
     its segments were still mapped.
   * the sweep destroyed a *starting* simulation. A process that is coming up
     has created segments it has not mapped yet, so for a moment they look
     unowned. Measured: the sim step spawned the simulation, a daemon restart
     swept 46 blocks a second later, and the simulation came up detached from
     everything. Segments younger than 20 s are now left alone.
   * "stale" was inferred from the process table and was wrong in both
     directions -- "no node running" missed the ros2 daemon, "anything
     running" masked a killed stack's leftovers. It is asked of the kernel
     now: `/proc/*/maps` names every segment every live process has mapped,
     and anything on disk and in nobody's map is a leftover.

7. **A stale helper and a detached process look identical, and only one is
   fixed by restarting the helper.** Anything that was running when shared
   memory was cleared has lost its middleware and cannot re-attach -- which
   is the NORMAL state after the previous repair, so restarting the helper at
   it is a loop. The tie-breaker is the query that bypasses the helper, which
   `env.sh` already documents for exactly this.

8. **The test gate invented a failing test called `=`.** `check_tests.py`
   matched `^FAILED (\S+)` over the whole pytest output, and pep257 echoes the
   source lines it objects to — so `FAILED = "failed"` in a source file became
   a failing test. A gate that cries wolf is a gate somebody bypasses.
