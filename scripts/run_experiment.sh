#!/usr/bin/env bash
# run_experiment.sh <task> [args]
#
# ONE dispatcher for both experiment sets.
#
#   CURRENT   t2 t3 t5 t6 t7   the bimanual set. run_bimanual.py.
#   LEGACY    e1..e6           superseded as protocols, kept because several
#                              are the analysis backend a T-task reuses.
#
# T1 is subsumed by T7. T4 is geometrically blocked (0 of 16 transfer points
# reachable by both arms -- geometry, not wrist orientation).
#
# Before this, only e1..e6 were accepted, so every GUI button that claimed to
# start a bimanual task exited 2 and looked like it had launched.
set -euo pipefail
source "$(dirname "$0")/env.sh"

usage() {
  cat >&2 <<EOF
usage: $0 <task> [args]

  CURRENT SET (bimanual)
    t3    rigid coupled carry        PRIMARY
    t6    compliant coupled carry    the contrast with T3
    t7    bimanual pursuit           cross-arm interference
    t5    handover to the wearer
    t2    hold and fill              4 verified scenarios; outcome instrumented
    t8    wearer-assisted reach      TWO-PERSON: neither can do it alone
    t9    reach under wearer motion  TWO-PERSON: sway is the IV (<=100 mm)

    common args:  --participant P01 [--condition direct|assisted|shared]
                  [--scenario S1..S4] [--scripted] [--dry-run]

  LEGACY SET (superseded protocols, still runnable)
    e1 e2 e3 e4 e5 e6

examples:
    $0 t3 --participant P01 --condition direct --scenario S1
    $0 t7 --participant P01 --dry-run
EOF
  exit 2
}

TASK="${1:-}"; shift || true
[ -z "$TASK" ] && usage

case "$TASK" in
  t2|t3|t5|t6|t7|t8|t9)
    exec ros2 run srl_experiments run_bimanual.py --task "$TASK" "$@" ;;
  e1) SCRIPT=run_fitts.py ;;
  e2) SCRIPT=run_autonomy_level.py ;;
  e3) SCRIPT=run_divided_attention.py ;;
  e4) SCRIPT=run_dof_recovery.py ;;
  e5) SCRIPT=run_intent_inference.py ;;
  e6) SCRIPT=run_vr_vs_mannequin.py ;;
  t1) echo "t1 is SUBSUMED BY t7 (bimanual pursuit). Run: $0 t7 ..." >&2
      exit 2 ;;
  t4) echo "t4 is GEOMETRICALLY BLOCKED: 0 of 16 transfer points are" >&2
      echo "reachable by both arms. Not a wrist-orientation problem; it" >&2
      echo "needs the right arm re-parked in hardware." >&2
      exit 2 ;;
  -h|--help) usage ;;
  *) echo "unknown task: $TASK" >&2; usage ;;
esac

echo "NOTE: $TASK is from the SUPERSEDED E-set. The current set is" >&2
echo "t2/t3/t5/t6/t7 -- see docs/research/04_protocol.md." >&2
exec ros2 run srl_experiments "$SCRIPT" "$@"
