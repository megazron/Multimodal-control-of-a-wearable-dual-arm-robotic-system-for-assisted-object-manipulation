#!/usr/bin/env bash
# calibrate.sh <zero|gyro|imu-mount|max-position>
#
# Every calibration the rig needs, in one place, with the order that matters
# spelled out. Gyro bias MUST be captured from a settled stream: reopening the
# serial port resets the Teensy and the IMU emits nonsense while settling,
# which once cost a factor of 34 in measured drift.
set -euo pipefail
source "$(dirname "$0")/env.sh"
case "${1:-}" in
  zero)         exec ros2 run srl_teleop capture_zero "${@:2}" ;;
  gyro)         echo "[calibrate] stop any master publisher first, or sample /master_arm_raw_* instead"
                exec ros2 run srl_teleop capture_gyro_bias "${@:2}" ;;
  imu-mount)    exec ros2 run srl_teleop imu_mount_calibration "${@:2}" ;;
  max-position) exec ros2 run srl_teleop max_position_recorder "${@:2}" ;;
  *) echo "usage: $0 <zero|gyro|imu-mount|max-position>" >&2; exit 2 ;;
esac
