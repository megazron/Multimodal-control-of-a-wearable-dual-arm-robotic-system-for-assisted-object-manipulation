#!/usr/bin/env bash
# RE-RECORD THE WHOLE VERIFICATION SET, one mode-task cell per stack.
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
set -u
cd "$(dirname "$0")/.."
LOG="${LOG:-recordings/verification/record_everything.log}"
RESUME="--resume"
[ "${1:-}" = "--fresh" ] && RESUME=""
SEED="${SEED:-3}"
INSTRUCT="${INSTRUCT:-put every cube on the pad of its own colour}"

MODES="06_full_autonomy 01_master_teleop 03_shared_autonomy 02_vr_teleop 04_vr_shared"
TASKS="t0 t1 t1s2 t2 t3"

echo "== RE-RECORDING THE VERIFICATION SET  $(date -Is)" | tee -a "$LOG"
echo "== seed $SEED   instruction: $INSTRUCT" | tee -a "$LOG"
n=0
for MODE in $MODES; do
  for TASK in $TASKS; do
    n=$((n+1))
    echo "" | tee -a "$LOG"
    echo "=== [$n/25] $MODE / $TASK  $(date -Is)" | tee -a "$LOG"
    timeout 2400 python3 scripts/sim_session.py --stack teleop --skip-tests -- \
        python3 scripts/record_abc_sweep.py \
            --taskset msc --only "$MODE" --tasks "$TASK" \
            --seed "$SEED" --instruct "$INSTRUCT" $RESUME 2>&1 | tee -a "$LOG"
    echo "=== [$n/25] $MODE / $TASK exited ${PIPESTATUS[0]}" | tee -a "$LOG"
  done
done
echo "== DONE  $(date -Is)" | tee -a "$LOG"
python3 scripts/status_table.py 2>&1 | tee -a "$LOG"
