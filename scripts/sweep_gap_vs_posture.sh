#!/usr/bin/env bash
# WOULD THE WEARER MOVING THEIR ARMS OPEN THE GAP? ONE STACK PER POSTURE.
#
# The wearer's arm posture reaches TWO places that share no file: the URDF,
# which is what MoveIt plans and collision-checks against, and
# `mount_guard_node.WEARER`, which is what the geometric clearance floor is
# measured against. Both read `SRL_WEARER_ARMS` -- the xacro through
# `optenv` at expansion time, the guard through `wearer_posture.py` at import.
#
# SO THE STACK HAS TO BE RELAUNCHED. Setting the variable in the measuring
# process alone would move the capsule model and leave MoveIt planning against
# a wearer standing the old way, which is the "same answer with a new label"
# failure `gen_wearer_posture_xacro.py` exists to prevent. This kills the
# stack, brings it up under the posture, measures, and moves on.
#
# `behind` and `out` are included ON PURPOSE even though docs/ENGINEERING_LOG.md records
# both as breaching the floor at the HOME pose: a posture that is unusable
# should be shown to be unusable with a number, not omitted from the table.
set -u
cd "$(dirname "$0")/.."
POSTURES="${1:-down folded behind out none}"
for P in $POSTURES; do
  echo "=================================================================="
  echo "WEARER POSTURE: $P"
  echo "=================================================================="
  SRL_WEARER_ARMS="$P" timeout 2700 python3 scripts/sim_session.py \
      --stack moveit --keep-up -- \
      env SRL_WEARER_ARMS="$P" python3 -u scripts/measure_centre_gap.py \
          --approaches t1 --tag "posture_$P" \
          --out "recordings/baselines/centre_gap_$P.json" \
    2>&1 | sed -n '/CONTROLS/,$p'
done
