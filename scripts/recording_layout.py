#!/usr/bin/env python3
"""Where recordings live, and why the tree has the shape it has.

    recordings/verification/<mode>/<task>/<scenario>/<condition>/

CONTROL MODE AND AUTONOMY CONDITION ARE INDEPENDENT AXES, and the old tree
conflated them. `direct`, `assisted` and `shared` are AUTONOMY CONDITIONS from
the original experimental design: how much the robot decides. A CONTROL MODE
is what drives the arm at all. "direct" under the mannequin master and
"direct" under VR are different recordings that shared one folder name, and
nothing in the path distinguished them.

Modes are NUMBERED so they sort by capability rather than alphabetically:
alphabetical order would interleave `full_autonomy` between `direct` and
`shared` and put the least capable mode in the middle.

WHAT IS ACTUALLY IN THE TREE TODAY, stated plainly because it is not what a
reader would assume from the directory names.

**No clip in this repository was recorded through a control mode.** Every one
was produced by `record_rviz.py`, which calls `/compute_ik` DIRECTLY and never
publishes `/master_arm_pose_*`, so no follower, no clutch, no anchor and no
orientation lock is in the path. Verified two ways: no clip's metadata carries
a mode field, and the recorder contains no publisher for the pose topic.

So the six mode directories are the structure GOING FORWARD and are empty
today. Existing clips go into two honestly-named buckets rather than being
assigned a mode they were never recorded in:

  00_unclassified_legacy_geometry   the retired nine-task clips (t2..t9).
      Stale twice over: superseded 310 mm geometry, and recorded before the
      single-owner gripper fix, so their traces show the object grasped,
      dropped and re-grasped in mid-air.

  00_unclassified_scripted_playback the current five-task clips (f1..f5).
      CURRENT geometry and a correct gripper, but still scripted playback
      rather than a mode recording. Putting them under
      `legacy_geometry` would be a false label; putting them under a mode
      would be a guess.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VERIFICATION = os.path.join(ROOT, "recordings/verification")

# Numbered so they sort by capability. The registry in
# srl_teleop/operating_modes.py is the authority on what each one means.
MODES = (
    "01_master_teleop",          # mannequin master arm
    "02_vr_teleop",              # Quest
    "03_orientation_assist",     # STUB: the IK plugin ignores the constraint
    "04_shared_autonomy",
    "05_supervised_autonomy",
    "06_full_autonomy",          # voice
)

# Not modes. Buckets for recordings whose mode cannot be determined, because
# guessing one is worse than admitting the axis is empty.
LEGACY = "00_unclassified_legacy_geometry"
SCRIPTED = "00_unclassified_scripted_playback"
BUCKETS = (LEGACY, SCRIPTED)

ALL_DIRS = MODES + BUCKETS


def clip_dir(mode, task, scenario, condition):
    return os.path.join(VERIFICATION, mode, task, scenario, condition)


def ensure_tree():
    """Create every mode directory, including the empty ones.

    An empty directory that exists says "this mode has no recordings yet". A
    missing one says nothing at all, and the reader cannot tell the difference
    between "not recorded" and "not a mode".
    """
    made = []
    for d in ALL_DIRS:
        p = os.path.join(VERIFICATION, d)
        if not os.path.isdir(p):
            os.makedirs(p, exist_ok=True)
            made.append(d)
        keep = os.path.join(p, ".gitkeep")
        if not os.path.exists(keep):
            open(keep, "w").write("")
    return made


def iter_clips(root=None):
    """Yield (mode, task, scenario, condition, path) for every clip present.

    Tolerates the OLD three-level layout as well, so a consumer written
    against this helper keeps working on a tree that has not been migrated.
    """
    root = root or VERIFICATION
    if not os.path.isdir(root):
        return
    for mode in sorted(os.listdir(root)):
        mpath = os.path.join(root, mode)
        if not os.path.isdir(mpath):
            continue
        if mode in ALL_DIRS:
            for task in sorted(os.listdir(mpath)):
                tp = os.path.join(mpath, task)
                if not os.path.isdir(tp):
                    continue
                for scen in sorted(os.listdir(tp)):
                    sp = os.path.join(tp, scen)
                    if not os.path.isdir(sp):
                        continue
                    for cond in sorted(os.listdir(sp)):
                        cp = os.path.join(sp, cond)
                        if os.path.isdir(cp):
                            yield mode, task, scen, cond, cp
        else:
            # Old layout: <task>/<scenario>/<condition>. Reported with mode
            # None so a caller can tell it apart rather than silently
            # treating the task name as a mode.
            for scen in sorted(os.listdir(mpath)):
                sp = os.path.join(mpath, scen)
                if not os.path.isdir(sp):
                    continue
                for cond in sorted(os.listdir(sp)):
                    cp = os.path.join(sp, cond)
                    if os.path.isdir(cp):
                        yield None, mode, scen, cond, cp


def bucket_for(task):
    """Which bucket a task's existing clips belong in. No guessing."""
    return SCRIPTED if str(task).startswith("f") else LEGACY
