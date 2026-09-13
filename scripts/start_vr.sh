#!/usr/bin/env bash
# Bring up the VR path with nothing configured by hand.
#
#   bash scripts/start_vr.sh            # real headset over USB
#   bash scripts/start_vr.sh --mock     # desktop mock, no headset
#
# Runs `adb reverse` on the WINDOWS side, because that is where the Quest is
# plugged in and where adb lives. Under networkingMode=mirrored the forwarded
# port lands on WSL's own 127.0.0.1 -- MEASURED, see docs/ENGINEERING_LOG.md -- so the ROS
# bridge binds locally and no relay is needed.
set -euo pipefail
set +u; source /opt/ros/jazzy/setup.bash; source "$(dirname "$0")/../install/setup.bash"; set -u

PORT=${PORT:-8766}
MOCK=0; [ "${1:-}" = "--mock" ] && MOCK=1
PS='/mnt/c/windows/System32/WindowsPowerShell/v1.0/powershell.exe'
say() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33m%s\033[0m\n' "$*"; }
die() { printf '\033[31m%s\033[0m\n' "$*"; exit 1; }

say "[1/4] observer e-stop"
if ! timeout 5 ros2 topic list 2>/dev/null | grep -q observer_estop_present; then
  warn "  /observer_estop_present has no publisher yet."
  warn "  The bridge WILL REFUSE to drive the arm until it does. That is"
  warn "  deliberate: the operator cannot see the real arm from inside the"
  warn "  headset, so a second person holding a stop is mandatory."
fi

if [ "$MOCK" = "0" ]; then
  say "[2/4] adb reverse on the Windows side"
  [ -x "$PS" ] || die "  no Windows PowerShell at $PS (interop needs the LOWERCASE /mnt/c/windows path)"
  ADB=$("$PS" -NoProfile -Command "(Get-Command adb -EA SilentlyContinue).Source" 2>/dev/null | tr -d '\r')
  [ -n "$ADB" ] || die "  adb is not installed on Windows. Install Android Platform-Tools, then re-run."
  DEV=$("$PS" -NoProfile -Command "& '$ADB' devices" 2>/dev/null | tr -d '\r' | grep -c "device$" || true)
  [ "${DEV:-0}" -ge 1 ] || die "  no Quest visible to adb. Plug in USB and allow USB debugging on the headset."
  "$PS" -NoProfile -Command "& '$ADB' reverse tcp:$PORT tcp:$PORT" >/dev/null 2>&1 \
    || die "  adb reverse failed"
  echo "  adb reverse tcp:$PORT tcp:$PORT  OK"
else
  say "[2/4] MOCK MODE -- no headset, no adb"
fi

say "[3/4] bridge + wearer view"
ros2 run srl_vr_teleop quest_vendor_bridge & BR=$!
ros2 run srl_vr_teleop wearer_view & WV=$!
trap 'kill $BR $WV 2>/dev/null || true' INT TERM EXIT
sleep 3

say "[4/4] ready"
if [ "$MOCK" = "1" ]; then
  echo "  running the desktop mock for 15 s..."
  ros2 run srl_vr_teleop quest_vendor_mock --ros-args -p duration:=15 2>/dev/null \
    || python3 "$(dirname "$0")/../src/srl_vr_teleop/srl_vr_teleop/quest_vendor_mock.py" --duration 15
else
  echo "  open the Unity client on the headset; it connects to ws://127.0.0.1:$PORT"
  echo "  Ctrl-C here to stop."
  wait $BR
fi
