#!/usr/bin/env bash
# scripts/run_wearer_tracker.sh -- start the body tracker in the ONE
# interpreter that can import both ROS and the pose model.
#
#     bash scripts/run_wearer_tracker.sh
#
# WHY A SEPARATE INTERPRETER. MediaPipe is not installed system-wide and must
# not be. Installing it pulls numpy 2.x, and the workspace's scipy and
# matplotlib are built against 1.26 -- measured: `import mediapipe` in a venv
# that had taken numpy 2.5.2 failed with "numpy.core.multiarray failed to
# import", and the same upgrade applied system-wide would have broken every
# analysis script in the repository. The working system is not worth breaking
# for a new one.
#
# So `.venv_pose` is created with --system-site-packages (so rclpy, cv_bridge
# and the rest of ROS are visible) and numpy is PINNED to the system's 1.26.4
# inside it. Measured in that venv: mediapipe, numpy, cv2, rclpy and cv_bridge
# all import, and the system interpreter is untouched -- `python3 -c "import
# numpy, cv2, scipy"` still reports 1.26.4 / 4.6.0 / 1.11.4.
set -euo pipefail

SRL_WS="${SRL_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export SRL_WS
VENV="$SRL_WS/.venv_pose"

set +u
# shellcheck disable=SC1091
source "$SRL_WS/scripts/env.sh"
set -u

if [ ! -x "$VENV/bin/python" ]; then
  cat <<TXT
The pose interpreter is not built yet. Build it once:

    python3 -m venv --system-site-packages "$VENV"
    "$VENV/bin/pip" install mediapipe
    "$VENV/bin/pip" install "numpy<2"      # AFTER mediapipe, on purpose

The second pip line is not optional and its order matters: mediapipe pulls
numpy 2.x, which then cannot coexist with the system scipy this venv can see.
TXT
  exit 1
fi

if ! "$VENV/bin/python" -c "import mediapipe" >/dev/null 2>&1; then
  echo "mediapipe does not import in $VENV -- see the note in this script."
  "$VENV/bin/python" -c "import mediapipe" 2>&1 | tail -3
  exit 1
fi

MODEL="$SRL_WS/models/mediapipe/pose_landmarker_full.task"
if [ ! -f "$MODEL" ]; then
  echo "No pose model at $MODEL. Fetch it once:"
  echo "  mkdir -p $SRL_WS/models/mediapipe && cd \$_ && curl -LO \\"
  echo "    https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"
  exit 1
fi

echo "interpreter  $VENV/bin/python"
echo "model        $MODEL"
for f in scene_camera_intrinsics scene_camera_extrinsics; do
  if [ -f "$SRL_WS/config/$f.yaml" ]; then
    echo "$f  present"
  else
    echo "$f  MISSING -- the tracker will refuse to place a body and will"
    echo "                          keep the mannequin. Run the matching"
    echo "                          scripts/calibrate_scene_camera*.py"
  fi
done
echo

exec "$VENV/bin/python" \
  "$SRL_WS/src/srl_perception/srl_perception/wearer_tracker_node.py" "$@"
