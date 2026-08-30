#!/usr/bin/env bash
# Prompted WRIST-ROTATION measurement: every axis, both directions.
#
#     bash scripts/run_vr_protocol.sh
#     bash scripts/run_vr_protocol.sh --only front back      # just those
#
# Run this in its OWN terminal, with the VR stack already up. It sources
# everything and puts the arms back at HOME before the run and before every
# single segment, so each motion is measured from the same starting pose.
#
# DOMAIN 0, NOT 7. The VR stack the GUI starts lives on domain 0; the arm
# bridge and the room cameras were brought up on 7 earlier in this session.
# A run on the wrong domain sees no topics and reports "no controller poses"
# against a headset that is streaming perfectly -- HARD CONSTRAINT 5's
# failure mode, one level up.
set -uo pipefail

# HARD CONSTRAINT 10: ROS setup.bash reads unbound variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/kortex_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM

WS="$HOME/kortex_ws"
OUT="$WS/recordings/vr_teleop"
mkdir -p "$OUT"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "domain      $ROS_DOMAIN_ID"
echo "results     $OUT/wrist_$STAMP.json"
echo

exec python3 "$WS/scripts/vr_wrist_protocol.py" \
    --out "$OUT/wrist_$STAMP.json" "$@"
