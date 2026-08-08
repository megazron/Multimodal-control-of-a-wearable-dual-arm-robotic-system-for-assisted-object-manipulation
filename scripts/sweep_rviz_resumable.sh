#!/usr/bin/env bash
# Resumable RViz capture sweep.
#
# The first sweep had no progress file: a restart would have re-recorded all
# 87 clips from the beginning. This one records each completed run in
# recordings/verification/.rviz_progress and skips anything already listed, so
# a kill, a context loss or an OOM costs one clip, not the whole sweep.
#
#   bash scripts/sweep_rviz_resumable.sh          # resume
#   bash scripts/sweep_rviz_resumable.sh --fresh  # start over
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; source /home/gausms/kortex_ws/install/setup.bash; set -u
cd /home/gausms/kortex_ws
PROG=recordings/verification/.rviz_progress
[ "${1:-}" = "--fresh" ] && rm -f "$PROG"
touch "$PROG"

# The work list comes from the scenarios file, so it cannot drift from what
# the recorder would actually run.
mapfile -t RUNS < <(python3 - <<'PY'
import yaml, os
BIM="src/srl_experiments/experiments/bimanual"
spec=yaml.safe_load(open(os.path.join(BIM,"scenarios_verified.yaml")))
KEY={"T2_hold_fill":"t2","T3_rigid":"t3","T5_handover":"t5","T6_compliant":"t6",
     "T7_pursuit":"t7","T8_wearer_assisted_reach":"t8","T9_wearer_motion":"t9"}
for k,t in KEY.items():
    for name in (spec["tasks"].get(k) or {}):
        for c in ("direct","assisted","shared"):
            print("%s %s %s"%(t,name,c))
PY
)
TOTAL=${#RUNS[@]}
echo "work list: $TOTAL runs"
DONE=0; SKIP=0
for r in "${RUNS[@]}"; do
  read -r T S C <<<"$r"
  KEYLINE="$T/$S/$C"
  MP4="recordings/verification/$T/$S/$C/rviz_iso.mp4"
  if grep -qxF "$KEYLINE" "$PROG" && [ -s "$MP4" ]; then
    SKIP=$((SKIP+1)); continue
  fi
  python3 -u scripts/record_rviz.py --task "$T" --scenario "$S" --condition "$C" \
    2>&1 | tail -1
  if [ -s "$MP4" ]; then
    echo "$KEYLINE" >> "$PROG"
    DONE=$((DONE+1))
  else
    echo "  !! no mp4 for $KEYLINE"
  fi
  echo "progress: $((DONE+SKIP))/$TOTAL"
done
echo "RVIZ SWEEP COMPLETE  recorded=$DONE skipped=$SKIP total=$TOTAL"
