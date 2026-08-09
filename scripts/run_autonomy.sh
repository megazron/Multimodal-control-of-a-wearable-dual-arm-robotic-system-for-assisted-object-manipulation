#!/usr/bin/env bash
# run_autonomy.sh - teleoperation PLUS perception and shared autonomy.
#
# Starts the teleop stack first and layers perception + autonomy on top, so a
# failure in the autonomy layer leaves a working teleoperated robot rather
# than a dead one.
set -euo pipefail
source "$(dirname "$0")/env.sh"

# Clear stale Fast DDS shared-memory segments before launching. A stale
# segment degrades discovery silently -- see srl_clear_stale_shm in env.sh.
srl_clear_stale_shm || true

exec ros2 launch srl_autonomy shared_autonomy.launch.py "$@"
