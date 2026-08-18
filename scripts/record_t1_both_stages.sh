#!/usr/bin/env bash
# RECORD T1 AND T1S2 UNDER 06_full_autonomy, FROM A TYPED INSTRUCTION.
#
#   bash scripts/record_t1_both_stages.sh "put every cube on the pad of its own colour"
#
# ONE MODE PER INVOCATION, THROUGH sim_session.py, WHICH IS THE PROTOCOL THAT
# WORKS. It kills the old stack, clears the stale /dev/shm segments, settles,
# relaunches and runs the unit suite before anything is filed as evidence.
# Churning processes against a live stack degrades the DDS graph until a FRESH
# process receives nothing, with every node still up.
#
# TWO INVOCATIONS, NOT ONE SWEEP OVER BOTH TASKS, and that is deliberate: each
# stage needs its own mock camera and its own staged look, and a stack that has
# just recorded one task is not a stack anybody has verified for the next.
#
# DETACHED, WITH A WAITER. Polling kills runs (R-9): every `ros2` invocation
# against a live graph is another participant in the discovery it is trying to
# observe. Start it with nohup and watch the log file.
set -u
cd "$(dirname "$0")/.."
INSTRUCTION="${1:-put every cube on the pad of its own colour}"
SEED="${2:-0}"
LOG="${LOG:-/tmp/srl_t1_record.log}"

echo "== T1 BOTH STAGES, 06_full_autonomy" | tee "$LOG"
echo "== instruction: $INSTRUCTION" | tee -a "$LOG"
echo "== stage 2 seed: $SEED" | tee -a "$LOG"

for TASK in t1 t1s2; do
  echo "" | tee -a "$LOG"
  echo "=== $TASK ============================================" | tee -a "$LOG"
  python3 scripts/sim_session.py --stack teleop -- \
      python3 scripts/record_abc_sweep.py \
          --taskset msc --only 06_full_autonomy --tasks "$TASK" \
          --seed "$SEED" --instruct "$INSTRUCTION" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  echo "=== $TASK exited $rc" | tee -a "$LOG"
done
echo "== DONE" | tee -a "$LOG"
