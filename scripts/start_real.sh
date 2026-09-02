#!/usr/bin/env bash
# start_real.sh -- terminal 3: connect the REAL arm(s), home, cascade.
#
#   arm:=left | right | both       (default: both)
#
#   bash scripts/start_real.sh
#
# Terminal 1 must already be running:
#   ros2 launch srl_teleop teleop.launch.py gate:=false
#
# WHAT IT DOES
#   1. verifies the sim stack is up (refuses otherwise)
#   2. 10 second countdown, one line per second, Ctrl-C aborts
#   3. starts the Kortex driver for each selected arm (one session each;
#      the arm permits exactly ONE, so a leaked session blocks the next run)
#   4. velocity homing to the legacy home, live per-joint progress
#   5. enables the sim->real bridge (delay 1.0 s, vmax 0.05, step 0.05,
#      lag trip 0.15 rad)
#   6. prints REAL ARMS LIVE
#   7. Ctrl-C at ANY point stops the arm and closes the Kortex session
#
# MOCK MODE. `bash scripts/start_real.sh --mock` runs the identical sequence
# against mock_real.launch.py instead of hardware. Same homing node, same
# bridge, same limits -- only the driver is swapped. That is how this script
# is tested without a robot attached.

set -u
set -o pipefail

MOCK=0
for a in "$@"; do
  case "$a" in
    --mock) MOCK=1 ;;
    arm:=*) ARM_ARG="${a#arm:=}" ;;
    home:=*) HOME_ARG="${a#home:=}" ;;
    --arm) NEXT_IS_ARM=1 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *)
      if [ "${NEXT_IS_ARM:-0}" = 1 ]; then ARM_ARG="$a"; NEXT_IS_ARM=0
      else echo "unknown argument: $a"; exit 2; fi ;;
  esac
done

WS="$HOME/kortex_ws"
# WHICH ARMS. Defaults to BOTH now that the dual-arm resource collisions are
# fixed (patches/0001-0003). `arm:=left` remains the conservative bring-up and
# is what every measurement to date used.
ARM="${ARM_ARG:-both}"
case "$ARM" in
  left|right|both) ;;
  *) echo "arm must be left, right or both (got '$ARM')"; exit 2 ;;
esac
if [ "$ARM" = both ]; then ARMS="left right"; else ARMS="$ARM"; fi
# HOMING IS NOT COMPULSORY, and forcing it is why teleoperation could only
# ever start from the home pose.
#
# `real_arms_highlevel.launch.py` has had a `home` argument since it was
# written; this script never passed it, so every route through here drove the
# arm to the config home before anything else could happen. Reported
# 2026-08-30: "that start real arms button still forces for home pose and
# doesn't let pick up pose to start teleop".
#
# Homing exists to close the gap between the simulation and the metal, and it
# is the right thing to do when they disagree. But `enable_gap_rad` is what
# actually decides whether the relay may start, and it is POSE-AGNOSTIC: if
# the arm and the sim already agree -- at the pick pose, the scan pose,
# anywhere -- there is nothing for homing to fix and driving across the
# workspace first is pure cost, and a hazard of its own.
#
# So the default stays true for a bare terminal run, where homing to a known
# pose is the safe way to start from nothing, and the window passes
# home:=false because its sequencer asks the relay first and only homes if the
# relay refuses on the gap. Override with home:=false.
HOME_REAL="${HOME_ARG:-true}"
case "$HOME_REAL" in
  true|false) ;;
  *) echo "home must be true or false (got '$HOME_REAL')"; exit 2 ;;
esac
# HOW FAR THE METAL RUNS BEHIND THE SIMULATION.
#
# 1.0 s was the shipped value and it is most of why teleoperation felt
# indirect: the operator steers something a full second in the past, so every
# correction is made against a view of the arm that is already wrong, and they
# over-correct and oscillate. Reported 2026-08-30 as "quite laggy and not
# stable".
#
# WHAT THE DELAY BUYS, so the trade is on the record: `sim_to_real_bridge`
# compares the DELAYED sim against the real arm, so the delay is the window in
# which a divergence can be seen and refused BEFORE the arm has committed to
# it. Shorter window, less warning. It does not weaken the trip itself --
# `lag_trip_rad` still e-stops on the same divergence -- it shortens the notice.
#
# 0.30 s keeps roughly four bridge cycles at 12 Hz of that window, which is
# enough for the monitor to see a real divergence build, and takes 700 ms of
# dead time out of the operator's hands. Override with PREVIEW_DELAY=<s>.
PREVIEW_DELAY="${PREVIEW_DELAY:-0.30}"
# BRIDGE REPLAY SPEED. This was 0.15 rad/s and that is BELOW WHAT THE SIM
# COMMANDS, which makes the lag monitor a scheduled failure rather than a
# safety net.
#
# Measured 2026-08-25 on both real arms. `vision_grasp` stages its moves with
# `secs=3.0`, and the observe pose is 1-2 rad from home, so the simulation
# demands 0.33-0.66 rad/s. The bridge could replay 0.15. The real arm can
# therefore NEVER catch up -- the error is not a transient, it accumulates
# monotonically -- and at 0.506 rad on joint_6 it crossed the 0.50 rad trip:
#
#   LAG MONITOR TRIPPING E-STOP: |sim_delayed - real| = 0.506 rad on joint_6
#
# The e-stop then LATCHED, `estop_node` took the arm-controller topic, and
# every trajectory after that was overridden. The workspace sweep went on
# printing "reaching cell N of 120" for 26 more cells against a halted arm,
# because nothing in that loop checks the e-stop or checks arrival.
#
# 0.40 rad/s is still 29% of the 1.3963 rad/s joint limit in
# joint_limits.yaml, and it is above the fastest thing the sim asks for, so
# the lag monitor goes back to meaning "the arm is not keeping up" instead of
# "the arm was never able to keep up".
# OVERRIDABLE with MAX_VEL=<rad/s>, which the master-mannequin GUI uses to
# run the master teleop faster than the scripted-move default above. Raising
# it is only safe while the COMMANDING side stays below it -- see
# master_bringup.SPEEDS, which pins master_teleop's vmax under this value so
# the lag monitor keeps meaning "the arm is not keeping up" rather than "the
# arm was never able to keep up".
MAX_VEL="${MAX_VEL:-0.40}"
# ARM IP ADDRESSES, overridable so the GUI can offer them. The defaults are
# real_arms_highlevel.launch.py's own and are the addresses on this rig.
LEFT_IP="${LEFT_IP:-192.168.1.10}"
RIGHT_IP="${RIGHT_IP:-192.168.1.9}"
# Homing speed, passed explicitly. The banner used to print $MAX_VEL --
# the BRIDGE parameter -- while nothing passed anything to the homing node.
HOMING_VMAX=0.15
# KORTEX COMMAND RATE, PER ARM. Every high-level command is a network round
# trip (HARD CONSTRAINT 4), and the two arms share one link.
#
# MEASURED 2026-08-20 in the lab, same box, same session:
#   one arm   21.0 Hz sustained, send latency 12-33 ms
#   two arms   8.1-12.2 Hz,      send latency SPIKING TO 519 ms
# 519 ms is past the 0.5 s bridge watchdog, so it zeroed the speed and the
# joints stopped WHILE STILL BEING COMMANDED. Homing then reported "BELOW
# VELOCITY FLOOR" -- a true statement about a symptom two layers down.
#
# 30 Hz x 2 arms x ~23 ms per send oversubscribes the link and the requests
# queue; the queue IS the 519 ms. Ask for less than the link can carry and
# the spikes go away. Override with KORTEX_RATE=<hz>.
if [ "$ARM" = "both" ]; then
  KORTEX_RATE="${KORTEX_RATE:-12.0}"
else
  KORTEX_RATE="${KORTEX_RATE:-30.0}"
fi
MAX_STEP=0.05
LAG_TRIP=0.5
COUNTDOWN=10

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; BOLD=$'\033[1m'; NC=$'\033[0m'

say()  { printf '%s\n' "$*"; }
ok()   { printf '%s\n' "${GRN}$*${NC}"; }
warn() { printf '%s\n' "${YEL}$*${NC}"; }
die()  { printf '%s\n' "${RED}$*${NC}" >&2; exit 1; }

# ---------------------------------------------------------------- cleanup
CHILD_PGID=""
CLEANING=0

cleanup() {
  # Guard against re-entry: Ctrl-C during cleanup must not restart cleanup.
  [ "$CLEANING" = "1" ] && return
  CLEANING=1
  echo
  warn "=== STOPPING ==="

  # 1. Disable the bridge FIRST so nothing new is commanded while we tear down.
  #
  # BOTH ARMS, BY NAME. These services are per arm now -- one bridge and one
  # homing node run per arm, so the old unprefixed /bridge_disable was two
  # registrations of one name and the call reached an arbitrary one of them.
  # Tearing down while the other arm is still driving a real robot is the
  # failure this loop now cannot have.
  if command -v ros2 >/dev/null 2>&1; then
    for A in left right; do
      timeout -s INT 8 ros2 service call "/bridge_disable_${A}" std_srvs/srv/Trigger {} \
        >/dev/null 2>&1 && say "  bridge disabled (${A})"
      # 2. Abort homing if it is still running; leaves the arm where it stands.
      timeout -s INT 8 ros2 service call "/home_abort_${A}" std_srvs/srv/Trigger {} \
        >/dev/null 2>&1 && say "  homing aborted (${A})"
    done
  fi

  # 3. Now close the driver. SIGINT to the whole child process group so the
  #    Kortex session is released rather than left dangling on the arm --
  #    the arm permits ONE session, and a leaked one blocks the next run.
  if [ -n "$CHILD_PGID" ] && kill -0 -"$CHILD_PGID" 2>/dev/null; then
    say "  closing Kortex session (pgid $CHILD_PGID)"
    kill -INT -"$CHILD_PGID" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 -"$CHILD_PGID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 -"$CHILD_PGID" 2>/dev/null; then
      warn "  did not exit on SIGINT, sending SIGTERM"
      kill -TERM -"$CHILD_PGID" 2>/dev/null
      sleep 2
      kill -9 -"$CHILD_PGID" 2>/dev/null
    fi
  fi

  # FALLBACK SWEEP. There is a window between `setsid ... &` and capturing
  # CHILD_PGID: a Ctrl-C landing inside it leaves the launch running with
  # nothing recorded to kill. On hardware that means an ORPHANED KORTEX
  # SESSION still holding the arm, and the arm allows only one session, so
  # the next run would fail to connect. Sweep by executable name; these are
  # ours and nothing else in the system runs them.
  # kortex_highlevel_bridge is included because it is the process that HOLDS
  # THE KORTEX SESSION. Leaving it behind is the one straggler that blocks the
  # next run outright -- the arm permits a single session. It is SIGINTed
  # first below so its handler can zero speeds and close the session.
  STRAGGLERS="$(pgrep -f 'srl_teleop/(real_homing_node|sim_to_real_bridge|mock_real_stack)|srl_teleop\.kortex_highlevel_bridge' 2>/dev/null | tr '\n' ' ')"
  if [ -n "${STRAGGLERS// /}" ]; then
    warn "  sweeping orphaned stack processes:$STRAGGLERS"
    for P in $STRAGGLERS; do kill -INT "$P" 2>/dev/null; done
    sleep 3
    for P in $STRAGGLERS; do kill -9 "$P" 2>/dev/null; done
  fi
  # And the launch process itself, if it outlived its group.
  LSTRAY="$(pgrep -f 'ros2 launch srl_teleop (real_arms|real_arms_highlevel|mock_real)\.launch\.py' 2>/dev/null | tr '\n' ' ')"
  if [ -n "${LSTRAY// /}" ]; then
    for P in $LSTRAY; do kill -INT "$P" 2>/dev/null; done
    sleep 2
    for P in $LSTRAY; do kill -9 "$P" 2>/dev/null; done
  fi

  ok "  stopped."
  exit 0
}
# EXIT as well as INT/TERM. Without EXIT, every `die` path -- including the
# bridge-enable timeout, which is the one that actually fires -- exited
# WITHOUT running cleanup, leaving the Kortex bridges alive holding their
# sessions. The arm permits exactly ONE session, so the next run then has to
# wait for the old one to time out. Observed twice today. The CLEANING guard
# makes the double-fire (INT then EXIT) harmless.
trap cleanup INT TERM EXIT

# ---------------------------------------------------------------- env
[ -d "$WS" ] || die "workspace not found: $WS"
cd "$WS" || die "cannot cd to $WS"

# All ROS environment lives in scripts/env.sh so that this script, the launch
# files and any shell you open by hand cannot drift apart. It also supplies
# the daemon-aware graph helpers used by the preflight below.
# shellcheck disable=SC1091
source "$WS/scripts/env.sh" || die "cannot source scripts/env.sh"

# ---------------------------------------------------------------- 1. sim check
say "${BOLD}[1/6] checking the sim stack${NC}"

# An empty graph has two completely different causes and they need completely
# different responses, so establish BOTH facts before saying anything:
#   - is the terminal-1 stack actually running?  (process table)
#   - can this shell see it?                     (ROS graph)
# The old check only asked the second question and reported "No ROS 2 nodes at
# all", which was true but pointed at the wrong thing entirely: the stack was
# up and a wedged ros2 daemon was hiding it.
PROCS="$(srl_stack_procs)"
# Exit status 10/11 means the daemon had to be restarted -- see scripts/env.sh
# for why that is reported through the status rather than a variable.
NODES="$(srl_ros_node_list 20)"; GRAPH_RC=$?

if [ "$GRAPH_RC" = "10" ] || [ "$GRAPH_RC" = "11" ]; then
  warn "  the ros2 daemon was not answering; restarted it"
  warn "  (a stale daemon survives WSL network changes and hides the graph)"
fi

if [ -z "$NODES" ]; then
  if [ -z "${PROCS// /}" ]; then
    # State 1: genuinely nothing running.
    die "No ROS 2 nodes, and no stack processes either -- nothing is running.
Start terminal 1 first:
    ros2 launch srl_teleop teleop.launch.py gate:=false"
  fi

  # State 2: the stack IS running but this shell cannot see it. Never report
  # this as "nothing is running". Confirm with a daemon-free query, which
  # cannot be fooled by a wedged daemon, then name discovery as the cause.
  DIRECT="$(srl_ros_node_list_direct 25)"
  if [ -n "$DIRECT" ]; then
    die "DISCOVERY PROBLEM, not a missing stack.
The terminal-1 stack IS running (pids:$PROCS) and a daemon-free query can see
it, but the daemon-backed graph is empty from this shell. The ros2 daemon is
the cause. It was already restarted once automatically and is still wrong.
Try, in this shell:
    ros2 daemon stop && ros2 daemon start
    ros2 node list --no-daemon      # ground truth, bypasses the daemon"
  fi

  die "DISCOVERY PROBLEM, not a missing stack.
The terminal-1 stack IS running (pids:$PROCS) but NO query from this shell can
see it, daemon or not. This is a discovery partition, not a dead stack.
Check that both shells agree:
    echo \"\$ROS_DOMAIN_ID / \$ROS_LOCALHOST_ONLY / \$ROS_AUTOMATIC_DISCOVERY_RANGE\"
This shell: domain ${ROS_DOMAIN_ID}, range ${ROS_AUTOMATIC_DISCOVERY_RANGE}, localhost_only ${ROS_LOCALHOST_ONLY:-<unset>}
Both terminals must source scripts/env.sh so these cannot drift."
fi

check_missing() {
  local nodes="$1" m=""
  echo "$nodes" | grep -q "/move_group"         || m="$m move_group"
  echo "$nodes" | grep -q "/controller_manager" || m="$m controller_manager"
  printf '%s' "$m"
}

MISSING="$(check_missing "$NODES")"

# A STALE daemon returns a NON-EMPTY but WRONG list, and the empty-list check
# above cannot see that. Observed: the daemon reported exactly two nodes --
# both `/real_world_link`, left over from a mock_real run that had long since
# exited -- while a daemon-free query saw 22 including controller_manager.
# The script then said "Sim stack is not running", which was false and sent
# the operator to restart a terminal that was already healthy.
#
# So: if the required nodes are missing BUT the stack processes exist, the
# daemon is the suspect, not the stack. Reset it and ask again before saying
# anything.
if [ -n "$MISSING" ] && [ -n "${PROCS// /}" ]; then
  warn "  '$MISSING' missing from the graph, but the stack IS running"
  warn "  (pids:$PROCS) -- resetting the ros2 daemon and re-checking"
  srl_ros_daemon_reset
  NODES="$(srl_ros_node_list 20)"
  MISSING="$(check_missing "$NODES")"
  [ -z "$MISSING" ] && ok "  daemon was stale; the graph is correct now"
fi

# Last resort: believe the daemon-free query over the daemon.
if [ -n "$MISSING" ] && [ -n "${PROCS// /}" ]; then
  DIRECT="$(srl_ros_node_list_direct 25)"
  if [ -z "$(check_missing "$DIRECT")" ]; then
    warn "  the daemon still disagrees with reality; using the daemon-free"
    warn "  node list as ground truth and continuing"
    NODES="$DIRECT"
    MISSING=""
  fi
fi

if [ -n "$MISSING" ]; then
  die "Sim stack is not running (missing:$MISSING).
Start terminal 1 first:
    ros2 launch srl_teleop teleop.launch.py gate:=false"
fi

# /joint_states must actually be flowing, not merely advertised.
HZ="$(timeout -s INT 15 ros2 topic hz /joint_states --window 10 2>&1 | grep -m1 'average rate' || true)"
[ -n "$HZ" ] || die "/joint_states is not publishing. Is the sim controller up?"
ok "  sim stack up -- $HZ"

if echo "$NODES" | grep -q "/master_pose_node"; then
  ok "  master_pose_node running"
else
  warn "  master_pose_node NOT running: the sim will not move, so the real"
  warn "  arm will have nothing to follow. Continuing anyway."
fi

# ---------------------------------------------------------------- Teensy port
# master_pose_node owns the port; this is REPORTING only, so that a Teensy that
# moved between ACM0 and ACM1 is visible here rather than being diagnosed as a
# dead master later. Nothing here opens the device.
say "${BOLD}      Teensy port${NC}"
FOUND=""
for DEV in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0 /dev/ttyUSB1; do
  [ -e "$DEV" ] || continue
  FOUND="$FOUND $DEV"
done
if [ -z "$FOUND" ]; then
  warn "  no /dev/ttyACM* or /dev/ttyUSB* present."
  warn "  If the Teensy is plugged in on Windows, attach it to WSL:"
  warn "      usbipd list        (find the BUSID)"
  warn "      usbipd attach --wsl --busid <BUSID>"
else
  ok "  serial devices present:$FOUND"
  # `ros2 topic hz` needs several samples before it prints anything, and at
  # the master's rate that can exceed a short timeout even when the topic is
  # perfectly healthy -- which produced a misleading "silent" warning. Count
  # actual messages instead: one is proof of life.
  GOT=0
  for _ in 1 2 3; do
    MASTER_OK=1
    for A in $ARMS; do
      timeout -s INT 6 ros2 topic echo "/master_arm_raw_$A" --once >/dev/null 2>&1 || MASTER_OK=0
    done
    if [ "$MASTER_OK" = 1 ]; then
      GOT=1; break
    fi
  done
  if [ "$GOT" = "1" ]; then
    ok "  master is publishing -- Teensy port resolved correctly"
  else
    warn "  /master_arm_raw_* is silent for at least one arm. master_pose_node autodetects the"
    warn "  port and launch respawns it, so a Teensy that moved between ACM0"
    warn "  and ACM1 recovers on its own; give it a few seconds."
    warn "  If it stays silent, attach the board to WSL: usbipd attach --wsl"
  fi
fi

# ---------------------------------------------------------------- estop check
ESTOP="$(timeout -s INT 10 ros2 topic echo /estop_state --once 2>/dev/null | head -1 || true)"
if echo "$ESTOP" | grep -q "true"; then
  die "E-STOP IS LATCHED. Reset it before connecting:
    ros2 service call /estop_reset std_srvs/srv/Trigger {}"
fi
ok "  e-stop clear"

# ---------------------------------------------------------------- 2. countdown
echo
if [ "$MOCK" = "1" ]; then
  warn "${BOLD}[2/6] MOCK MODE -- no hardware will be touched${NC}"
else
  printf '%s\n' "${BOLD}${YEL}[2/6] The REAL LEFT ARM is about to connect and MOVE.${NC}"
  printf '%s\n' "${YEL}      Keep a hand on the e-stop. Ctrl-C aborts.${NC}"
fi
for i in $(seq "$COUNTDOWN" -1 1); do
  printf '      %2d ...\n' "$i"
  sleep 1
done
say "      go."

# ---------------------------------------------------------------- 3-5. launch
echo
say "${BOLD}[3/6] starting the driver, homing and bridge${NC}"

LOG="$(mktemp -t start_real.XXXXXX.log)"
say "  log: $LOG"

if [ "$MOCK" = "1" ]; then
  # the mock has no network round trip and no rate_hz argument
  setsid ros2 launch srl_teleop mock_real.launch.py \
    arm:="$ARM" \
    preview_delay_s:="$PREVIEW_DELAY" \
    max_vel_rad_s:="$MAX_VEL" \
    homing_vmax:="$HOMING_VMAX" \
    max_step_rad:="$MAX_STEP" \
    lag_trip_rad:="$LAG_TRIP" >"$LOG" 2>&1 &
else
  setsid ros2 launch srl_teleop real_arms_highlevel.launch.py \
    arm:="$ARM" \
    left_robot_ip:="$LEFT_IP" \
    right_robot_ip:="$RIGHT_IP" \
    preview_delay_s:="$PREVIEW_DELAY" \
    max_vel_rad_s:="$MAX_VEL" \
    homing_vmax:="$HOMING_VMAX" \
    home:="$HOME_REAL" \
    rate_hz:="$KORTEX_RATE" \
    max_step_rad:="$MAX_STEP" \
    lag_trip_rad:="$LAG_TRIP" >"$LOG" 2>&1 &
fi
CHILD_PID=$!
sleep 1
CHILD_PGID="$(ps -o pgid= -p "$CHILD_PID" 2>/dev/null | tr -d ' ')"
[ -n "$CHILD_PGID" ] || CHILD_PGID="$CHILD_PID"
say "  child pgid $CHILD_PGID"

# ---------------------------------------------------------------- 4. homing
echo
say "${BOLD}[4/6] homing (velocity law, kp 0.5, vmax ${HOMING_VMAX} rad/s)${NC}"

DEADLINE=$(( $(date +%s) + 420 ))
SHOWN=0
while :; do
  if ! kill -0 "$CHILD_PID" 2>/dev/null && ! kill -0 -"$CHILD_PGID" 2>/dev/null; then
    say "--- last 30 log lines ---"; tail -30 "$LOG"
    die "The driver stack exited before homing finished. See $LOG"
  fi
  [ "$(date +%s)" -gt "$DEADLINE" ] && { tail -30 "$LOG"; die "Timed out waiting for homing. See $LOG"; }

  # stream per-joint progress as it appears.
  # NOTE: `grep -c` already prints 0 when there is no match AND exits 1, so a
  # `|| echo 0` fallback appends a SECOND zero and yields "0\n0", which is not
  # an integer and breaks the comparison below. Take the first line only.
  N="$(grep -c 'err deg' "$LOG" 2>/dev/null | head -1)"
  [ -n "$N" ] || N=0
  if [ "$N" -gt "$SHOWN" ]; then
    grep 'err deg' "$LOG" | tail -n +$((SHOWN + 1)) | sed 's/^\[[^]]*\] *//'
    SHOWN="$N"
  fi

  NARMS=$(echo $ARMS | wc -w)
  # grep -c PRINTS 0 and EXITS 1 when there is no match, so a
  # `|| echo 0` fallback yields "0\n0" and breaks the integer test
  # below. This is documented in CLAUDE.md and was reintroduced here.
  # Take the last line and default only when the output is empty.
  NDONE=$(grep -c "HOMING COMPLETE" "$LOG" 2>/dev/null | tail -1)
  NDONE=${NDONE:-0}
  # One COMPLETE per arm. With arm:=both the first arm finishing used to end
  # the wait and the bridge was enabled while the second was still moving.
  if [ "$NDONE" -ge "$NARMS" ]; then
    grep -A9 "final per-joint" "$LOG" | sed 's/^\[[^]]*\] *//' | head -9
    NWITHIN=$(grep -c "WITHIN the" "$LOG" 2>/dev/null | tail -1)
    NWITHIN=${NWITHIN:-0}
    if [ "$NWITHIN" -ge "$NARMS" ]; then
      ok "  homing complete on $NARMS arm(s), every joint within tolerance"
    else
      tail -20 "$LOG"
      die "Homing finished OUTSIDE tolerance. Not enabling the bridge."
    fi
    break
  fi
  for PAT in "HOMING TIMEOUT" "HOMING STALLED" "HOMING HALTED" \
             "HOMING OSCILLATING" "HOMING BELOW VELOCITY FLOOR" \
             "E-STOP during homing"; do
    if grep -q "$PAT" "$LOG" 2>/dev/null; then
      grep "$PAT" "$LOG" | tail -2 | sed 's/^\[[^]]*\] *//'
      die "Homing did not finish: $PAT. Arm left where it stopped."
    fi
  done
  sleep 1
done

# ---------------------------------------------------------------- 5-6. bridge
echo
say "${BOLD}[5/6] enabling the sim->real bridge${NC}"
say "      delay ${PREVIEW_DELAY}s  vmax ${MAX_VEL}  step ${MAX_STEP}  lag trip ${LAG_TRIP}"

DEADLINE=$(( $(date +%s) + 300 ))
LASTNAG=""
while :; do
  if grep -q "REAL ARMS LIVE" "$LOG" 2>/dev/null; then
    break
  fi
  if grep -q "GAVE UP" "$LOG" 2>/dev/null; then
    REASON="$(grep -o 'Last refusal:.*' "$LOG" | tail -1)"
    die "The bridge REFUSED to enable.
  ${REASON:-no reason recorded}
The arm is homed and safe; nothing is being commanded."
  fi
  if grep -q "LAG MONITOR" "$LOG" 2>/dev/null; then
    grep "LAG MONITOR" "$LOG" | tail -1 | sed 's/^\[[^]]*\] *//'
    die "The lag monitor tripped the e-stop before the bridge went live."
  fi
  # surface the current refusal so a stuck enable is explained, not silent
  NAG="$(grep -o 'waiting to enable:.*' "$LOG" | tail -1)"
  if [ -n "$NAG" ] && [ "$NAG" != "$LASTNAG" ]; then
    warn "  $NAG"
    LASTNAG="$NAG"
  fi
  [ "$(date +%s)" -gt "$DEADLINE" ] && die "Timed out waiting for the bridge. See $LOG"
  if ! kill -0 -"$CHILD_PGID" 2>/dev/null; then
    tail -30 "$LOG"; die "The stack exited while enabling the bridge. See $LOG"
  fi
  sleep 2
done

echo
printf '%s\n' "${GRN}${BOLD}======================================================${NC}"
printf '%s\n' "${GRN}${BOLD}  [6/6] REAL ARMS LIVE - move the master arm.${NC}"
printf '%s\n' "${GRN}${BOLD}======================================================${NC}"
echo
say "  The real arm follows the sim ${PREVIEW_DELAY}s behind."
say "  Lag over ${LAG_TRIP} rad trips the e-stop automatically."
say "  Ctrl-C here stops the arm and closes the Kortex session."
echo

# Stream anything notable until interrupted.
tail -f "$LOG" 2>/dev/null | grep --line-buffered -E \
  "LAG MONITOR|E-STOP|clearance|DISABLED|ERROR" &
TAILPID=$!

while kill -0 -"$CHILD_PGID" 2>/dev/null; do
  sleep 2
done
kill "$TAILPID" 2>/dev/null
warn "The real stack exited."
cleanup
