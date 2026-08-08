#!/usr/bin/env bash
# run_teleop.sh - TELEOPERATION ONLY. No perception, no autonomy.
#
# This is the baseline condition for every experiment, and it is deliberately
# runnable with srl_perception and srl_autonomy absent from the workspace.
# If this script ever needs one of them, the layering has been broken.
set -euo pipefail
source "$(dirname "$0")/env.sh"
exec ros2 launch srl_teleop teleop.launch.py "$@"
