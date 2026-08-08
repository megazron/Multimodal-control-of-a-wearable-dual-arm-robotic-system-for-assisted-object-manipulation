# srl_autonomy — shared autonomy

Grasp generation, intent inference and operator-confirmed handover.

**Layering, enforced:** this package depends on `srl_teleop` and
`srl_perception`. Neither depends on it. Teleoperation must remain runnable
with this package absent, because it is the study's baseline condition.

## Design commitments

- **Grasps come from a lookup table**, not a learned predictor: top-down at the
  centroid, gripper aligned to the object's minor axis. Every candidate is
  validated with `/compute_ik` **before it is offered** — an unreachable grasp
  is never shown to the operator.
- **Intent is inferred primarily from IMU POINTING DIRECTION**, not from
  commanded end-effector velocity. Pointing is measured directly and is
  drift-free; velocity inherits every potentiometer fault the master arm has.
  Velocity is a secondary cue with a smaller weight.
- **Handover is discrete and operator-confirmed. Never continuous blending.**
  The operator keeps full position control at all times; only wrist
  orientation is servoed, and only over ~0.5 s, and never by snapping.
- **The gripper closes only on an operator FSR command.** Never autonomously.

## Depends on

`rclpy`, `geometry_msgs`, `std_msgs`, `moveit_msgs`, `tf2_ros`, `srl_teleop`,
`srl_perception`.

## Run

    bash scripts/run_autonomy.sh          # teleop + perception + autonomy

## Interface

| topic | type | meaning |
| --- | --- | --- |
| `/autonomy/intent_<arm>` | `std_msgs/Float64MultiArray` | P(goal) over detected objects, published continuously for logging |
| `/autonomy/intent_ids_<arm>` | `std_msgs/String` | the object ids the distribution is over, same order |
| `/autonomy/state_<arm>` | `std_msgs/String` | `DIRECT` \| `ASSIST` \| `GRASPED` |
| `/autonomy/grasp_<arm>` | `geometry_msgs/PoseStamped` | the validated grasp being assisted toward |
| `/autonomy/cancel_<arm>` | `std_msgs/Bool` | operator cancel, also triggered by moving away |
