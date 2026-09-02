# Recording session, 2026-09-02

26 recording attempts, 1.2 GB of rosbag2/mcap on disk. **The bags themselves are
not in this folder** — see *What is not here* below. This is the index and the
fault report.

## Headline

| verdict | sessions |
| --- | ---: |
| OK | 8 |
| PARTIAL — cameras not recorded | 7 |
| FAULT — no messages recorded | 7 |
| FAULT — shorter than 5 s | 1 |
| FAULT — a master arm is silent | 1 |
| EMPTY DIRECTORY | 2 |

**8 of 26 attempts produced clean data. 10 produced nothing at all.**

## Faults, in the order they matter

### 1. Ten recordings contain no data

Seven bags opened, wrote a 1.4 kB header and closed with **0 messages on 0
topics**; two directories are completely empty; one ran for 0.5 s.

| session | what exists |
| --- | --- |
| `20260902_075535_session` | no files at all |
| `20260902_082255_session` | no files at all |
| `christian_multimeter_masterdirect_20260902_110444` | 0 msgs, 0.0 s |
| `christian_multimeter_mastershared_20260902_110933` | 0 msgs, 0.0 s |
| `christian_targetreachinng_20260902_111739` | 0 msgs, 0.0 s |
| `christian_targetreachinngshared_20260902_112652` | 0 msgs, 0.0 s |
| `chritstian_objectreaching_20260902_113502` | 0 msgs, 0.0 s |
| `jorge_multimeter_20260902_135758` | 0 msgs, 0.0 s |
| `jorge_multimetesharedr_20260902_140023` | 0 msgs, 0.0 s |
| `master_teleop_20260902_093707` | 316 msgs, 0.5 s, 0.25 MB |

**Every multimeter run is in that list** — Christian direct, Christian shared,
Jorge direct, Jorge shared. The multimeter task has no data at all, for either
participant, in either condition. Christian's target-reaching (both conditions)
and his object-reaching are gone too, which leaves him with three usable runs
out of eight.

A bag that opens, writes a header and closes with zero messages is the signature
of a recorder started against a bus with no publishers on the topics it asked
for — not of a disk or permissions problem. It is the same class of fault as the
2026-09-02 commit *"The RECORD button recorded nothing; use ros2 bag"*.

### 2. One recording lost an entire arm

`jorge_targetreachingshared_20260902_135048` ran 206.4 s and 129 308 messages,
but `/master_arm_pose_right`, `/master_arm_raw_right`, `/master_status_right`
and `/right_arm_controller/joint_trajectory` are all **empty**. It is a
one-armed recording of a two-armed task — 8 dead topics against 2–4 in every
other good run of the same shape.

### 3. The cameras recorded nothing in the afternoon block

In all seven 15:36–16:56 sessions every `/left_camera/*` topic and almost every
`/right_camera/*` topic was subscribed and captured **zero frames**. Those bags
are 26–116 MB of everything else, so they are usable for kinematics and useless
for anything visual.

### 4. Two topics never published in any session

`/mount_guard` is empty in **17 of 17** bags that recorded anything, and
`/estop` in 14 of 17. `/estop` silent is expected — nobody pressed it.
`/mount_guard` silent in every single session is not obviously expected and is
worth a look: it is the node that enforces the wearer clearance floor.

## What is here

* `MANIFEST.csv` / `manifest.json` — one row per attempt: duration, message
  count, live vs dead topics, size, verdict, and the full list of empty topics.
* `metadata/<session>.yaml` — each bag's own `metadata.yaml`, copied verbatim.
  This is the provenance: topic list, per-topic message counts, QoS profiles,
  start and end times.

## What is not here, and why

The `.mcap` files. 1.2 GB across 26 directories, and four single files over
GitHub's 100 MB hard limit:

* 187 MB — `jorge_targetreachingdirect_20260902_133239`
* 134 MB — `christian_mastershared_20260902_111231`
* 120 MB — `jorge_targetreachingshared_20260902_135048`
* 116 MB — `20260902_161756_20260902_161756`

`git-lfs` is not installed on this machine, and GitHub's free LFS tier is 1 GB
in any case. The bags stay in `recordings/sessions/` on the lab machine.

## Note on participant names

Seven session directories carry a participant's first name. This repository is
private, so they are not public, but `write_manifest()` refuses `name`/`email`
fields by design (CLAUDE.md hard constraint 12) and these directory names go
around that. Renaming them to `P01`/`P02` costs nothing while the bags are still
only on this machine.

## Full index

| session | s | messages | live/total topics | MB | verdict |
| --- | ---: | ---: | :---: | ---: | --- |
| `20260902_075535_session` | 0 | 0 | 0/0 | 0.0 | EMPTY DIRECTORY |
| `20260902_082255_session` | 0 | 0 | 0/0 | 0.0 | EMPTY DIRECTORY |
| `20260902_153646_20260902_153645` | 262.8 | 690601 | 74/111 | 74.45 | PARTIAL: cameras not recorded |
| `20260902_155631_20260902_155631` | 250.0 | 652323 | 71/111 | 77.57 | PARTIAL: cameras not recorded |
| `20260902_161756_20260902_161756` | 360.1 | 1028131 | 75/111 | 115.87 | PARTIAL: cameras not recorded |
| `20260902_162436_20260902_162436` | 265.7 | 707261 | 75/111 | 83.78 | PARTIAL: cameras not recorded |
| `20260902_164021_20260902_164021` | 207.5 | 521249 | 79/111 | 59.36 | PARTIAL: cameras not recorded |
| `20260902_164902_20260902_164902` | 94.8 | 201416 | 72/111 | 25.74 | PARTIAL: cameras not recorded |
| `20260902_165659_20260902_165659` | 99.3 | 258564 | 73/111 | 31.61 | PARTIAL: cameras not recorded |
| `christian_mastershared_20260902_111231` | 209.8 | 176740 | 30/32 | 134.01 | OK |
| `christian_multimeter_masterdirect_20260902_110444` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `christian_multimeter_mastershared_20260902_110933` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `christian_objecttrackingdirect_20260902_114001` | 154.3 | 128401 | 29/32 | 96.28 | OK |
| `christian_objecttrackingdirect_20260902_114249` | 145.9 | 118022 | 28/32 | 90.32 | OK |
| `christian_targetreachinng_20260902_111739` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `christian_targetreachinngshared_20260902_112652` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `chritstian_objectreaching_20260902_113502` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `jorge_multimeter_20260902_135758` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `jorge_multimetesharedr_20260902_140023` | 0.0 | 0 | 0/0 | 0.0 | FAULT: no messages recorded |
| `jorge_objecttrackingdirect_20260902_132710` | 111.0 | 88208 | 29/32 | 67.0 | OK |
| `jorge_objecttrackingshared_20260902_132941` | 71.7 | 60052 | 28/32 | 45.07 | OK |
| `jorge_targetreachingdirect_20260902_133239` | 303.9 | 245491 | 30/32 | 187.15 | OK |
| `jorge_targetreachingshared_20260902_134510` | 83.6 | 59215 | 29/32 | 49.9 | OK |
| `jorge_targetreachingshared_20260902_135048` | 206.4 | 129308 | 24/32 | 120.04 | FAULT: a master arm is silent |
| `master_teleop_20260902_093707` | 0.5 | 316 | 24/27 | 0.25 | FAULT: shorter than 5 s |
| `testa_20260902_093724` | 46.6 | 37592 | 28/32 | 28.87 | OK |
