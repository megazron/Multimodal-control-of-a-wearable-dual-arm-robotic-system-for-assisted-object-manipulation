#!/usr/bin/env bash
# pick_the_cube.sh -- recover from a latched e-stop and run the pick.
#
#     bash scripts/pick_the_cube.sh [--arm left|right] [--expect N]
#
# WHY THIS EXISTS. The observe move tripped the sim->real lag monitor's e-stop
# on every attempt, and recovering from that is FIVE steps in an order that is
# load-bearing -- reset the latch, re-home whichever arm was halted mid-move,
# bring the SIM back to home too, re-enable both bridges, and only then look.
# Doing them out of order produces a refusal that names a different problem
# from the one in front of you:
#
#   real arm off home      -> "would command that difference as a jump"
#   SIM off home           -> "sim is 1.771 rad from the real arm on joint_5"
#
# Both are the same underlying state and neither message says so.
#
# THE UNDERLYING BUG IS FIXED SEPARATELY and this script does not paper over
# it: `Vision.stage()` now sizes each move from how far the joints actually
# travel (verify_colour_vision._staging_secs) so the simulation cannot
# outrun what the bridge can replay. Measured 2026-08-25, the old hardcoded
# 3.0 s asked for 0.33-0.66 rad/s against a bridge replaying 0.15, and the
# error accumulated monotonically to the 0.50 rad trip. Twice.
set -u
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
# shellcheck disable=SC1091
set +u; source "$WS/scripts/env.sh" >/dev/null 2>&1; set -u

ARM=left
EXPECT=1
while [ $# -gt 0 ]; do
  case "$1" in
    --arm) ARM="$2"; shift 2 ;;
    --expect) EXPECT="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

OUT="$WS/recordings/baselines/detected_cube_${ARM}.json"
say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
ok()  { printf '  \033[32m%s\033[0m\n' "$*"; }
bad() { printf '  \033[31m%s\033[0m\n' "$*"; }

svc() {  # svc <service> ; prints the message, returns 0 only on success=True
  local out
  out="$(timeout 30 ros2 service call "$1" std_srvs/srv/Trigger 2>&1)"
  printf '  %s\n' "$(printf '%s' "$out" | grep -o "message='[^']*'" | head -1)"
  printf '%s' "$out" | grep -q 'success=True'
}

say "[1/6] clearing the e-stop latch"
if svc /estop_reset; then ok "e-stop clear"; else bad "reset refused"; fi

say "[2/6] re-homing both real arms"
# THE ARM THAT WAS HALTED MID-MOVE IS NOT AT HOME, and the bridge refuses to
# enable against a gap it would otherwise command as a jump. Homing is safe to
# repeat: an arm already there simply reports so.
for a in left right; do
  printf '  %s: ' "$a"; svc "/home_arm_$a" >/dev/null && ok "homing started" || bad "refused"
done
sleep 25

say "[3/6] returning the SIMULATION to home"
# NOT COSMETIC. The bridge compares sim against real; the failed observe move
# left the sim at the observe pose, 101 deg away on joint_5, and enabling with
# that gap is exactly what the trip limit exists to prevent.
timeout 200 python3 scripts/stage_presentation_pose.py --home --move-s 12 \
  --timeout-s 40 2>&1 | tail -4

say "[4/6] enabling both bridges"
fail=0
for a in left right; do
  printf '  %s: ' "$a"
  svc "/bridge_enable_$a" >/dev/null && ok enabled || { bad "REFUSED"; fail=1; }
done
if [ "$fail" = 1 ]; then
  bad "a bridge refused -- the message above names the joint and the gap."
  bad "Do NOT raise the trip limit; re-run this script instead."
  exit 1
fi

say "[5/6] pausing the IK followers"
# They own /<arm>_arm_controller/joint_trajectory. A look performed while they
# are live is a second publisher on it and the follower wins, which is how the
# observe move silently did nothing. See scripts/follower_pause.py.
for a in left right; do
  timeout 15 ros2 param set "/ik_follower_$a" motion_enabled false >/dev/null 2>&1 \
    && ok "$a paused" || bad "$a: could not pause"
done

say "[6/6] looking at the table and detecting the cube"
timeout 500 "$WS/.venv_vision/bin/python" scripts/stage_observe_and_detect.py \
  --arm "$ARM" --expect "$EXPECT" --out "$OUT" 2>&1 | grep -v '^\[' | tail -20
rc=${PIPESTATUS[0]}

echo
if [ -s "$OUT" ]; then
  ok "detections written to $OUT"
  python3 -c "import json,sys; d=json.load(open('$OUT')); print('  ', json.dumps(d)[:400])"
else
  bad "no detections file -- the step above says why."
  bad "If it says the arm did not reach the observe pose, check for a NEW"
  bad "e-stop trip:  grep 'LAG MONITOR' on the start_real log."
fi
exit "$rc"
