#!/usr/bin/env bash
# start_cascade.sh -- make the REAL arms follow the SIMULATION, using the
# Kortex sessions that are ALREADY OPEN.
#
#     bash scripts/start_cascade.sh            # every arm that has a bridge
#     bash scripts/start_cascade.sh left       # just this one
#
# WHY THIS EXISTS AND start_real.sh DOES NOT DO IT.
#
# `start_real.sh` brings the real side up FROM NOTHING: it launches
# `real.launch.py`, which starts a `kortex_highlevel_bridge` per arm. Each of
# those opens a Kortex session, and THE ARM PERMITS EXACTLY ONE (HARD
# CONSTRAINT 2). Run it against a rig whose bridges are already connected --
# which is every rig brought up by `srl_bringup_all.sh` -- and the second
# session is refused, the launch fails, and its own cleanup then sweeps the
# WORKING bridges as orphans. Two live arms killed by pressing the button
# whose label says it connects them.
#
# So this script never starts a bridge. It REQUIRES one, per arm, and relays
# onto it. The relay is `sim_to_real_bridge`: it reads the simulated arm's
# joint states and republishes them to
# /real/<arm>_arm_controller/joint_trajectory, so whatever drives the SIM --
# VR, the master arm, MoveIt, a pose button -- drives the metal.
#
# WHAT IS DELIBERATELY LEFT OPEN. `require_homed` is false here. The relay
# normally refuses unless the real arm is at home, because it replays sim
# angles and any difference it does not catch is commanded as a jump (HARD
# CONSTRAINT 1). `enable_gap_rad` still holds: the sim and the real arm must
# agree to within that before the FIRST relayed command, so a large
# divergence is refused at the seam rather than driven through it.
#
# AND THAT IS WHY THIS ALSO STARTS `real_homing_node`, WITH auto_home FALSE.
#
# The two facts above are a DEADLOCK on their own, and the rig sat in it on
# 2026-08-29. HARD CONSTRAINT 0: sim home is the presentation pose and the
# real arms are still at the legacy Kortex home, so an adopted session is
# typically far from the loaded home -- measured 2.729 rad. The relay
# therefore refuses to enable (`cascade_active_left` false,
# `bridge_status_left[0]` 0.0), and the operator is told to home the arm
# first. But `real_homing_node` is started by `real_arms_highlevel.launch.py`
# and NOT by this script, so on an adopted session `/home_arm_<arm>` did not
# exist: the HOME button answered "service not present -- nothing was sent".
# The one step that closes the gap was unreachable, so the relay could never
# enable, so the arm could never move. Permanently, every session.
#
# auto_home is FALSE and that is the whole safety property: adopting a
# session must never silently move metal. This node comes up, prints its
# plan, moves NOTHING, and serves `/home_arm_<arm>` and `/home_abort_<arm>`
# so the operator's press has something to reach. The relay and the homing
# law share the trajectory topic by design -- `sim_to_real_bridge.enable()`
# refuses while `homing_active`, which is exactly how the launch file already
# runs the two together.
set -uo pipefail
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM

MAXV="${CASCADE_MAX_VEL:-0.30}"
# The homing law's own speed. `start_real.sh` passes 0.15 explicitly and
# records why: the banner used to print the BRIDGE's max_vel while nothing
# forwarded anything to the homing node at all.
HOMING_VMAX="${CASCADE_HOMING_VMAX:-0.15}"
WANT=("$@")
[ ${#WANT[@]} -eq 0 ] && WANT=(left right)

PIDS=()
STARTED=0

# ---------------------------------------------------------------- real TF
# THE `real_*` FRAMES, WITHOUT WHICH HOMING CANNOT RUN AT ALL.
#
# `real_homing_node` measures the arm against the wearer through
# `REAL_FRAME_PREFIX = "real_"`, applied to BOTH operands -- it asks for
# `real_torso`, `real_head`, `real_left_bracelet_link`. Those frames come from
# a robot_state_publisher in the `/real` namespace with `frame_prefix: real_`,
# plus a static `world -> real_world` link. `real_arms_highlevel.launch.py`
# starts both. THIS SCRIPT STARTED NEITHER.
#
# So on an adopted session every wearer lookup failed and homing halted with
#
#   WEARER CLEARANCE IS PARTIAL: ... 0 of 9 parts checked.
#   HOMING HALTED: the wearer clearance guard resolved NO transforms ...
#
# which is the guard behaving exactly as it should -- it will not move an arm
# on an unmeasured clearance -- while the operator sees a HOME button that
# does nothing and a log they are not reading. Measured 2026-08-29; it is the
# third thing this script had to inherit from the launch file, after the relay
# and the homing node itself.
#
# Nothing here commands anything. A robot_state_publisher republishes the
# joint states the Kortex bridge is ALREADY publishing, as TF.
if pgrep -f "real_robot_state_publisher" >/dev/null 2>&1; then
  printf '  real  TF publisher already running -- left alone\n'
else
  RSP_PARAMS="$(mktemp -t real_rsp.XXXXXX.yaml)"
  {
    printf '/real/real_robot_state_publisher:\n'
    printf '  ros__parameters:\n'
    printf '    frame_prefix: real_\n'
    printf '    robot_description: |\n'
    xacro "$(ros2 pkg prefix srl_description)/share/srl_description/urdf/srl_dual.urdf.xacro" \
      use_fake_hardware:=true left_use_fake_hardware:=true \
      right_use_fake_hardware:=true use_fake_gripper_hardware:=true \
      2>/dev/null | sed 's/^/      /'
  } > "$RSP_PARAMS"
  if [ -s "$RSP_PARAMS" ] && grep -q "robot" "$RSP_PARAMS"; then
    printf '  real  publishing the real_* frames (TF only -- commands nothing)\n'
    setsid ros2 run robot_state_publisher robot_state_publisher --ros-args \
      -r __node:=real_robot_state_publisher \
      -r __ns:=/real \
      --params-file "$RSP_PARAMS" \
      -r /real/joint_states:=/real/joint_states >/dev/null 2>&1 &
    PIDS+=("$!")
    setsid ros2 run tf2_ros static_transform_publisher \
      --frame-id world --child-frame-id real_world >/dev/null 2>&1 &
    PIDS+=("$!")
  else
    printf '  real  COULD NOT BUILD the robot description -- homing will halt\n' >&2
    printf '        on "wearer clearance guard resolved NO transforms".\n' >&2
  fi
fi

for arm in "${WANT[@]}"; do
  if ! pgrep -f "kortex_highlevel_bridge_${arm}" >/dev/null 2>&1; then
    printf '  %-5s no Kortex bridge -- skipped (nothing to relay onto)\n' "$arm"
    continue
  fi
  # THE HOMING SERVICE, SO THE RELAY'S REFUSAL HAS A CURE. See the header.
  # auto_home false: nothing moves until somebody presses HOME.
  #
  # BEFORE the relay's own already-running check, which `continue`s. The
  # arm this matters most for is the one whose relay is ALREADY up and
  # refusing -- put this after that branch and the button does nothing for
  # exactly the rig that needs it.
  if pgrep -f "real_homing_node_${arm}" >/dev/null 2>&1; then
    printf '  %-5s homing node ALREADY RUNNING -- left alone\n' "$arm"
  else
    printf '  %-5s serving /home_arm_%s (auto_home OFF -- nothing moves yet)\n' \
      "$arm" "$arm"
    setsid ros2 run srl_teleop real_homing_node --ros-args \
      -r __node:="real_homing_node_${arm}" \
      -p arm:="${arm}" \
      -p auto_home:=false \
      -p vmax_rad_s:="${HOMING_VMAX}" &
    PIDS+=("$!")
  fi

  if pgrep -f "sim_to_real_bridge_${arm}" >/dev/null 2>&1; then
    printf '  %-5s cascade ALREADY RUNNING -- left alone\n' "$arm"
    STARTED=$((STARTED + 1))
    continue
  fi
  printf '  %-5s Kortex bridge is up -- relaying sim -> real\n' "$arm"
  setsid ros2 run srl_teleop sim_to_real_bridge --ros-args \
    -r __node:="sim_to_real_bridge_${arm}" \
    -p arm:="${arm}" \
    -p enabled:=true \
    -p require_homed:=false \
    -p max_vel_rad_s:="${MAXV}" \
    -p rate_hz:=20.0 &
  PIDS+=("$!")
  STARTED=$((STARTED + 1))

done

if [ "$STARTED" -eq 0 ]; then
  printf '\nREFUSING: no arm has a Kortex bridge running.\n' >&2
  printf 'Connect the arms first -- ARMS -> CONNECT in the window, or\n' >&2
  printf '  bash scripts/bringup_arm.sh left\n' >&2
  exit 1
fi

# HARD CONSTRAINT 9: kill the process GROUP. `ros2 run` is a wrapper and
# killing the wrapper leaves the node behind holding its topics.
cleanup() {
  printf '\n  stopping the cascade\n'
  for p in "${PIDS[@]:-}"; do
    kill -INT -- "-$p" 2>/dev/null || kill -INT "$p" 2>/dev/null
  done
  sleep 2
  exit 0
}
trap cleanup INT TERM

printf '\n  %d cascade(s) running.\n' "$STARTED"
printf '  The relay will REFUSE to enable while the real arm is far from the\n'
printf '  loaded home -- it replays sim angles STARTING at home. Call\n'
printf '  /home_arm_<arm> (HOME BOTH ARMS in the window) first; that service\n'
printf '  is now served. Nothing has moved yet.\n'
printf '  Ctrl-C here to stop relaying (the Kortex sessions stay open).\n\n'
wait
