#!/usr/bin/env bash
# Put the LEFT arm on the operator's ideal pick pose, then pick the green cube.
#
#   bash scripts/pick_cube_left.sh              # move and pick
#   bash scripts/pick_cube_left.sh --plan-only  # everything except the motion
#
# The RIGHT arm is not commanded by this script.  It is already on its ideal
# pose (recordings/baselines/pick_pose_both_v3.json), and the cube is a
# left-arm job.
#
# WHY THIS SCRIPT SETS THE DOMAIN ITSELF
# --------------------------------------
# The arms run on ROS_DOMAIN_ID=7, NOT the usual 0: domain 0's FastDDS shared
# memory is polluted and no NEW process can join it (minimal pub/sub: 0 of 3
# messages on domain 0, 3 of 3 on domain 7).  A run on the wrong domain
# publishes into the void and moves nothing, silently.
#
# WHY STEP 1 IS "SAFE" GOTO
# -------------------------
# A straight line in joint space says nothing about where the hand goes.  The
# first return to this pose was interpolated that way from a pose hovering low
# over the table, and it drove the gripper INTO the table.  safe_goto_left.py
# walks the whole hand through the path in FK against the measured table plane
# and refuses, or detours over the top, rather than commanding a sweep.
set -uo pipefail

# HARD CONSTRAINT 10: ROS setup.bash reads unbound variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/kortex_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM

WS="$HOME/kortex_ws"
SCRIPTS="$WS/scripts"
POSE="$WS/recordings/baselines/pick_pose_both_v3.json"
PLAN="$WS/recordings/baselines/pick_plan_left_v2.json"
export PYTHONPATH="$SCRIPTS:${PYTHONPATH:-}"

DRY=""
[ "${1:-}" = "--plan-only" ] && DRY="--dry-run"

step () { echo; echo "=============================================================="
          echo " $1"; echo "=============================================================="; }

step "1/4  LEFT arm -> ideal pick pose (path checked against the table)"
python3 "$SCRIPTS/safe_goto_left.py" --pose "$POSE" --arm-key left $DRY || {
    echo "FAILED at step 1 -- not proceeding."; exit 1; }

step "2/4  LOCATE the cube from the left gripper camera"
python3 "$SCRIPTS/locate_cube_left.py" || {
    echo "FAILED at step 2 -- cube not located (is it in view?); not proceeding."
    exit 1; }

step "3/4  PLAN the grasp"
python3 "$SCRIPTS/plan_pick_left.py" || {
    echo "FAILED at step 3 -- planning failed; not proceeding."; exit 1; }
for f in /tmp/scratch/pick_plan.json; do
    [ -f "$f" ] && cp "$f" "$PLAN"
done
echo "plan -> $PLAN"

if [ -n "$DRY" ]; then
    step "4/4  DRY RUN (nothing commanded)"
    python3 "$SCRIPTS/execute_pick_left.py" --plan "$PLAN" --dry-run
    exit 0
fi

step "4/4  EXECUTE  (Ctrl+C stops the arm within 0.5 s via the bridge watchdog)"
python3 "$SCRIPTS/execute_pick_left.py" --plan "$PLAN"
