#!/usr/bin/env bash
# check_channels.sh -- the wiring-repair acceptance test. ~3 minutes.
#
#   bash scripts/check_channels.sh                 # sweep, then verdict table
#   bash scripts/check_channels.sh --analyse-only <csv>
#   bash scripts/check_channels.sh --baseline recordings/baselines/channels_20260806.json
#
# Runs block A only -- 14 single-joint sweeps, one per master channel -- and
# prints the same verdict table under the same rules as the full analysis:
# DEAD / INCOHERENT / INTERMITTENT / ALIVE.
#
# It is deliberately short enough to run repeatedly with a soldering iron in
# hand: sweep a joint, re-run, see whether the verdict moved.
#
# WHAT "INCOHERENT" MEANS, because it is the verdict that matters here.
# A failing pot does not go quiet. It returns numbers that span a wide range
# but are not a trajectory -- consecutive updates unrelated, jumping tens of
# degrees in 60 ms. Range alone calls that healthy. The rule used here is the
# fraction of updates jumping more than 60 deg; above 5% the channel is not
# tracking anything.
set -u
cd "$(dirname "$0")/.." || exit 1
WS="$PWD"
BASE=""
ANALYSE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --baseline) BASE="$2"; shift 2 ;;
    --analyse-only) ANALYSE="$2"; shift 2 ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Default to the most recent baseline if one exists and none was named.
if [ -z "$BASE" ]; then
  BASE="$(ls -t "$WS"/recordings/baselines/channels_*.json 2>/dev/null | head -1)"
fi

REPORT="$WS/src/srl_experiments/trajectory_capture/channel_report.py"

if [ -n "$ANALYSE" ]; then
  CSV="$ANALYSE"
else
  # shellcheck disable=SC1091
  set +u; source "$WS/scripts/env.sh" || exit 1; set -u
  OUT="$WS/recordings/channel_checks/check_$(date +%Y%m%d_%H%M%S)"
  echo "=================================================================="
  echo " CHANNEL CHECK -- block A only, 14 sweeps, ~3 min"
  echo "=================================================================="
  echo " One sweep per channel. Sweep the named joint through its full"
  echo " range and back; hold the others still. Press the OPPOSITE arm's"
  echo " button to start and again to end, as usual."
  echo
  echo " A press under 0.5 s discards that segment and re-prompts, so a"
  echo " fumbled sweep costs nothing."
  echo
  ros2 run srl_experiments record_trajectories --blocks A --out "$OUT" || {
    echo "capture failed or was interrupted" >&2; exit 1; }
  CSV="$OUT/all_segments.csv"
fi

[ -f "$CSV" ] || { echo "no CSV at $CSV" >&2; exit 1; }

echo
if [ -n "$BASE" ] && [ -f "$BASE" ]; then
  echo "diffing against baseline: $BASE"
  python3 "$REPORT" "$CSV" --baseline "$BASE"
else
  echo "no baseline found -- printing absolute verdicts only"
  python3 "$REPORT" "$CSV"
fi

echo
echo "To make THIS run the new reference point:"
echo "  python3 $REPORT $CSV --save-baseline recordings/baselines/channels_\$(date +%Y%m%d).json"
