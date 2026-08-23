#!/usr/bin/env bash
# RE-RECORD THE VERIFICATION SET, one mode-task cell per stack.
#
#   bash scripts/record_everything.sh            # resume where it stopped
#   bash scripts/record_everything.sh --fresh    # ignore the ledger
#
# WHY ONE CELL PER `sim_session` INVOCATION, and not one long sweep. Churning
# processes against a live stack degrades the DDS graph until a FRESH process
# receives nothing while every node is still up -- recorded as R-9 and the
# reason `record_t1_both_stages.sh` is written the same way. Each cell gets a
# stack that was killed, had its stale /dev/shm cleared, was relaunched and
# was waited on until /compute_ik answered and /joint_states DELIVERED.
#
# IT IS RESUMABLE BECAUSE IT WILL BE INTERRUPTED. `record_abc_sweep --resume`
# reads `recordings/verification/abc_sweep_progress.json` and skips a cell
# already recorded, so a kill costs one cell rather than the set. `--fresh`
# only stops it SKIPPING; it never deletes a clip, because ARCHIVE, NEVER
# DELETE and a half-finished re-record that has eaten the old set is the worst
# state to be in.
#
# THE ORDER IS DELIBERATE. `mode_upstreams.MODE_ORDER` puts 06 first: it needs
# the fewest upstreams, so a failure there is the recorder rather than the
# isolation. The VR modes are last for the same reason in reverse.
#
# ===========================================================================
# THE TWO SKIPS HAPPEN HERE, BEFORE A STACK IS BUILT, AND THAT IS THE POINT
# ===========================================================================
# `record_abc_sweep` makes both decisions correctly -- but it makes them
# INSIDE the stack, so every skipped cell still cost a full teardown, /dev/shm
# clear, relaunch and readiness wait. Measured on 2026-08-23: about nine
# minutes each.
#
#   * T1 AND T1S2 RUN UNDER 06 ONLY. The task declares it, and the sweep
#     prints "SKIP (t1 runs under 06_full_autonomy only)". Eight of the
#     twenty-five combinations this loop enumerates are that skip -- over an
#     hour of building stacks to be told no.
#   * A CELL ALREADY IN THE LEDGER is skipped by `--resume`, and on a resume
#     after an interruption that is most of them.
#
# The sweep still makes both calls for real; this only avoids paying for a
# stack to hear them. If the two ever disagree the sweep wins, because it is
# the one that can see the task.
set -u
cd "$(dirname "$0")/.."
LOG="${LOG:-recordings/verification/record_everything.log}"
PROG="recordings/verification/abc_sweep_progress.json"
RESUME="--resume"
[ "${1:-}" = "--fresh" ] && RESUME=""
SEED="${SEED:-3}"
INSTRUCT="${INSTRUCT:-put every cube on the pad of its own colour}"

MODES="06_full_autonomy 01_master_teleop 03_shared_autonomy 02_vr_teleop 04_vr_shared"
TASKS="t0 t1 t1s2 t2 t3"
# Tasks that declare their own mode. Kept as data so adding one is a one-word
# change rather than an `if` buried in the loop.
ONLY_06="t1 t1s2"

# THE WORK LIST, RESOLVED FIRST AND PRINTED. A loop whose real size is only
# discoverable by watching it run is a loop nobody can plan around.
CELLS=""
for MODE in $MODES; do
  for TASK in $TASKS; do
    case " $ONLY_06 " in
      *" $TASK "*) [ "$MODE" = "06_full_autonomy" ] || continue ;;
    esac
    if [ -n "$RESUME" ] && [ -f "$PROG" ]; then
      python3 - "$PROG" "$MODE" "$TASK" <<'PY' && continue
import json, sys
prog, mode, task = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    d = json.load(open(prog))
except Exception:
    sys.exit(1)
for k, v in d.items():
    m, t, _ = (k.split("/") + ["", "", ""])[:3]
    if m == mode and t == task and v.get("ok"):
        sys.exit(0)          # already recorded -- skip without a stack
sys.exit(1)
PY
    fi
    CELLS="$CELLS $MODE/$TASK"
  done
done

N=$(printf '%s\n' $CELLS | grep -c . || true)
echo "== RE-RECORDING THE VERIFICATION SET  $(date -Is)" | tee -a "$LOG"
echo "== seed $SEED   instruction: $INSTRUCT" | tee -a "$LOG"
echo "== $N cell(s) to record:$CELLS" | tee -a "$LOG"
if [ "$N" -eq 0 ]; then
  echo "== nothing to do" | tee -a "$LOG"
  exit 0
fi

n=0
for CELL in $CELLS; do
  MODE="${CELL%%/*}"; TASK="${CELL##*/}"
  n=$((n+1))
  echo "" | tee -a "$LOG"
  echo "=== [$n/$N] $MODE / $TASK  $(date -Is)" | tee -a "$LOG"
  timeout 2400 python3 scripts/sim_session.py --stack teleop --skip-tests -- \
      python3 scripts/record_abc_sweep.py \
          --taskset msc --only "$MODE" --tasks "$TASK" \
          --seed "$SEED" --instruct "$INSTRUCT" $RESUME 2>&1 | tee -a "$LOG"
  echo "=== [$n/$N] $MODE / $TASK exited ${PIPESTATUS[0]}" | tee -a "$LOG"
done
echo "== DONE  $(date -Is)" | tee -a "$LOG"
python3 scripts/status_table.py 2>&1 | tee -a "$LOG"
