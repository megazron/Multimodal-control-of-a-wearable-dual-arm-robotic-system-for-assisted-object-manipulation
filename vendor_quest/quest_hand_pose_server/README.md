# Quest Hand Pose Server

This package is dedicated to Quest hand-pose streaming. It is named separately
from the future `controller_signal_server`.

## Files

- `quest_hand_pose_server.py`
- `start_quest_hand_pose_server.bat`
- `requirements.txt`

## Main GUI changes

1. The application and launcher are named **Quest Hand Pose Server**.
2. The 3D visualization magnifies hand positions by **3×**.
3. The local pose axes are also enlarged from `0.14` to `0.42`.
4. Movement trails have been removed.
5. Each tracked hand now shows a translation vector from the Tracking-Space
   origin to the hand-pose origin.
6. The numerical values in the left status panel remain the original,
   unscaled Quest values.

## First run

1. Connect Quest 3 by USB and allow USB debugging.
2. Double-click `start_quest_hand_pose_server.bat`.
3. Press Enter to use the default Conda environment `VR_Teleop`.
4. Select `3` once to install/update dependencies.
5. Start again and select `1` for GUI or `2` for terminal mode.

The launcher is configured for:

    D:\Anaconda\install\condabin\conda.bat

The Unity client remains:

    ws://127.0.0.1:8765

The server automatically manages:

    adb reverse tcp:8765 tcp:8765
