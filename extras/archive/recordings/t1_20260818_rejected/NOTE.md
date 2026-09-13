# Two clips their own verifiers rejected, kept as evidence of the fault

**Neither of these is a T1 recording. Both were REFUSED by the checks that ran
immediately after them, and they are here rather than in
`recordings/verification/` so that nothing reads a failed clip as the task's
evidence.** `recordings/verification/abc_sweep_progress.json` carries the same
verdict for both: `ok=False`.

## `06_full_autonomy/T1/S1_both_arms_centre`

    run exited 2; NO HOME RECORD; NO GRASP RECORDED at all;
    PLACED 0.140 m FROM TARGET

`run_abc` exited 2 before writing its home record, so the arms never ran the
task. It has exactly two `return 2` paths -- `require_home()` refusing, and
`--isolate` refusing -- and the sweep does not pass `--isolate`.

**The same command outside the sweep completes correctly**: four closes, four
releases, every cube released over the slot of its own colour's pad, the
arrival gate reaching waypoint 0 in 2.5 s to 3.6 mm, both arms starting at
0.0000 rad from home. The reproduction for both is in `docs/NEXT_SESSION.md`.

The reason the refusal was not simply read off the log is that `Gui.on_launch`
sent the launched job's stdout AND stderr to DEVNULL. That is fixed -- each
launch now writes to its own file in the scratch directory -- so the next run
of this will name its own cause.

## `06_full_autonomy/T1S2/S2_both_arms_random`

    run exited 0; NO GRASP RECORDED at all

Recorded BEFORE `t1s2` declared `orient`, so it was commanded at the pinned
anchor while its coordinates had been solved at T1's own approach. It is the
more instructive of the two: the pads read **1.4 to 4.2 mm from every cube**,
the best numbers this task has ever produced, while the knuckle stayed 0.00 for
the whole run and nothing was grasped. `clip_scene` was drawing the cubes
through the same wrong offset the arrival gate was using, so the scene's own
measurement of "how close are the pads to the cube" agreed with itself while
both were wrong.

That is docs/ENGINEERING_LOG.md's "everything matches" row -- a detector keyed to the wrong
referent, agreeing by construction -- and it is worth keeping a clip of.
