# srl_vr_autonomy — shared autonomy for the VR path

Kept separate from `srl_autonomy` so the two modalities can be run, cited and
compared independently — they are conditions in one study.

**The inference is NOT duplicated.** `vr_intent_source` publishes
`/master_pointing_<arm>`, the same topic the mannequin's IMU path publishes,
so `srl_autonomy`'s `intent_inference` runs unchanged: same Bayesian update,
same forgetting factor, same three hard cases, same tests. Only the SOURCE of
the pointing vector differs, which is exactly the manipulation E6 needs.

## The design question, and how it was resolved

On the mannequin, autonomy supplies wrist ORIENTATION because the master
cannot measure it. On VR the operator already has it — supplying it would be
supplying something they have, and it would fight them.

So VR assistance supplies **precision and workload reduction** instead:
`vr_handover_arbiter` implements a funnel that blends the operator's pose
toward the validated grasp with a weight that grows as they approach, reaching
full authority only over the last 3 cm, and **yielding entirely** if the
operator starts rotating the controller.

| | mannequin | VR |
| --- | --- | --- |
| supplies | missing DOFs | precision |
| result shape | categorical (impossible → possible) | continuous improvement |
| measured by | E4 | E6 |

The gripper is never closed by the autonomy, in either path.
