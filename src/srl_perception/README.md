# srl_perception — object detection and 6-DOF pose

**Deliberately boring.** The contribution of this project is the autonomy and
the human study; a flaky detector destroys user-study data, so this package
optimises for predictable failure over capability.

- **AprilTag is primary** (`apriltag_detector`). Robust, fast, full 6-DOF, and
  its failures are *obvious* — a tag is either decoded or it is not. There is
  no silent-wrong-answer mode.
- **Colour/shape is a clearly-secondary fallback** (`colour_shape_detector`),
  off unless explicitly enabled, and it is marked lower-confidence in every
  message it emits so downstream code cannot confuse the two.

## Depends on

`rclpy`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `cv_bridge`,
`tf2_ros`, OpenCV. **Does not depend on `srl_autonomy` or `srl_experiments`.**

## Run

    ros2 launch srl_perception perception.launch.py            # both wrist cameras
    ros2 launch srl_perception perception.launch.py fallback:=true

## Interface

| topic | type | meaning |
| --- | --- | --- |
| `/perception/detections` | `vision_msgs/Detection3DArray` | object id, 6-DOF world pose, confidence, stamp |
| `/perception/detections/<arm>` | `vision_msgs/Detection3DArray` | per-camera, before fusion |
| `/perception/objects` | `vision_msgs/Detection3DArray` | tracked, world frame, stale entries dropped |

## Characterisation

    ros2 run srl_perception measure_detector --help

Reports detection rate, pose accuracy against a known ground truth and
end-to-end latency. **Under 95% detection at working distance is a BLOCKER for
the user study** — see `docs/research/03_ethics_and_safety.md` and the measured
numbers in `docs/ENGINEERING_LOG.md`.
