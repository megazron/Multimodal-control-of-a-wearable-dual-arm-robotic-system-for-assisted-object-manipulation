# Superseded 2026-08-13: the dance and demo sets, before the routines were fixed

`extras/archive/recordings/mode06_GOOD_20260811/` is untouched, as always.

**Nothing in these may be quoted, and the dance clips in particular show a
routine that was never framed by a scene node.**

## What changed under them

| | change | effect |
| --- | --- | --- |
| 1 | **the dance had NO SCENE NODE.** `clip_scene` rejected `--task d1` in its own argparse `choices`, so the scene process failed to start on every dance clip: no furniture, no markers, and no `scene_events.json`. The sweep launched it, argparse refused it, and nothing downstream asked | these clips carry no travel measurement at all, and nothing in them was drawn by the scene |
| 2 | **the envelope was 0.18 x 0.14 x 0.18 m** — the arm could only nod. Rebuilt from task0's measured free-space anchors to \|x\| 0.28–0.48, y 0.24–0.42, z **1.06–1.54**; the vertical range went from 180 mm to 480 mm | the motion in these clips is a small fraction of what the arms can do |
| 3 | **the canon was invisible.** The two arms differed by 68 mm mirror-corrected at a 1-beat lag, which is a few pixels on screen — real in the data, unison in the picture. Now two beats: 125 mm mean, 238 mm max | flow reads as mirrored here, which is not what the routine is |
| 4 | **framing.** No `TASK_FOCUS` entry existed for d1/d2/d3, so the front view used the generic `_FOCUS` | the routines were framed for a task volume they do not occupy |
| 5 | **the presentation pose never applied.** The staging move lost to `ik_follower_node`, which streams to the same controller | these clips open on the home pose |

## What did NOT change

The three routines' characters and relationships: flow is adagio in canon,
pulse is staccato in opposition, play is call-and-response. Those were right;
what was wrong was that none of them could be seen properly.
