#!/usr/bin/env bash
# HOW FAR FORWARD DOES THE ROW HAVE TO BE BEFORE THE CENTRELINE OPENS?
#
# The wearer alone gives a hard bound that needs no kinematics: at the object
# height z = 0.97 a single tube point on the centreline is
#
#     y = 0.30   0.1403 m from the torso   -- 10 mm INSIDE the 150 mm floor
#     y = 0.32   0.1602 m                  -- clear
#
# and that is the TORSO, so it is identical for every wearer arm posture. The
# row T1 currently uses is at y = 0.300, which is 20 mm too close for the
# centreline to be admissible at all.
#
# A point is not an arm, though. This walks the whole pick path at N=10 for a
# range of row distances, with the table's near edge following the row (the
# objects always stand on the front edge, which is itself a measured
# constraint), and reports the innermost column that survives.
set -u
cd "$(dirname "$0")/.."
EDGES="${1:-0.280 0.300 0.320 0.340 0.380}"
for E in $EDGES; do
  echo "=================================================================="
  echo "TABLE NEAR EDGE $E  ->  row y = $(python3 -c "print(round($E+0.02,3))")"
  echo "=================================================================="
  # ITS OWN STACK, AT THE SHIPPED POSTURE. The posture sweep leaves the last
  # stack running under whatever wearer it finished with, and a row sweep
  # inheriting `behind` would be measuring two changes at once.
  SRL_WEARER_ARMS=down timeout 3000 python3 scripts/sim_session.py \
      --stack moveit --keep-up -- \
      env SRL_WEARER_ARMS=down python3 -u scripts/measure_centre_gap.py \
          --approaches t1 --near-y "$E" --x-max 0.45 \
          --tag "row_$E" --out "recordings/baselines/centre_gap_row_$E.json" \
    2>&1 | sed -n '/^1\. THE GAP/,$p' 
done
