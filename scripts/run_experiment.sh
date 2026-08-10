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
usage: $0 <a|b|c> [args]

  THE TASK SET. One generation, not three.
    a   pick and place        single arm
    b   hold and place        bimanual
    c   multimeter            present and probe

    args:  --mode 01_master_teleop|02_vr_teleop|03_shared_autonomy|
                  04_vr_shared|06_full_autonomy
           [--taskset study|clip] [--scenario S..] [--participant P01]
           [--scripted] [--dry-run]

  The bimanual (t2-t9), five-task and E-series (e1-e6) sets are ARCHIVED under
  src/srl_experiments/experiments/_archive/. They carried a 300/310 mm span
  against the current 500 mm, so a trial run from them produces a plausible
  number against the wrong criterion. The data is kept; the launch path is not.
EOF
  exit 2
}

TASK="${1:-}"; shift || true
[ -z "$TASK" ] && usage

case "$TASK" in
  a|b|c|A|B|C)
    # THE CURRENT THREE-TASK SET, and the only one. Coordinates come from
    # experiments/abc/ (verified N=10 over the densified full path).
    exec python3 "$(dirname "$0")/../src/srl_experiments/experiments/abc/run_abc.py" \
        --task "$TASK" "$@" ;;
  t1|t2|t3|t4|t5|t6|t7|t8|t9|e1|e2|e3|e4|e5|e6)
    echo "$TASK is ARCHIVED. It used the superseded 300/310 mm span; the" >&2
    echo "current spec is 500 mm and the tilt threshold rescales with it." >&2
    echo "Data kept at src/srl_experiments/experiments/_archive/." >&2
    echo "Run: $0 a|b|c --mode <mode> --taskset clip" >&2
    exit 2 ;;
  -h|--help) usage ;;
  *) echo "unknown task: $TASK" >&2; usage ;;
esac

echo "NOTE: $TASK is from the SUPERSEDED E-set. The current set is" >&2
echo "t2/t3/t5/t6/t7 -- see docs/research/04_protocol.md." >&2
exec ros2 run srl_experiments "$SCRIPT" "$@"
