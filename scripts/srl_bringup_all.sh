#!/usr/bin/env bash
# THE WHOLE RIG, FROM NOTHING, IN THE ONE ORDER THAT WORKS.
#
#     bash scripts/srl_bringup_all.sh
#
# Every step here is a lesson from 2026-08-26. The order is not cosmetic:
#
#   1. STOP EVERYTHING, BRIDGES FIRST AND WITH SIGINT. The arm permits
#      exactly one Kortex session and SIGKILL LEAKS IT -- the next connect
#      then fails while the arm still pings (HARD CONSTRAINT 2).
#
#   2. CLEAR STALE SHARED MEMORY, WITH THE STACK DOWN. Twice today discovery
#      wedged: `ros2 topic list` returned 2 topics against a fully running
#      stack, 356 stale segments in /dev/shm. Nothing new could see anything
#      (HARD CONSTRAINT 5).
#
#   3. SIM BEFORE ARMS, AND WITH master:=false. move_group cannot run its
#      planning-scene monitor without /joint_states, so it never advertises
#      /compute_ik -- and the table sweep needs IK for every cell. And with
#      no Teensy, master_pose_node is respawn=True: five copies starved the
#      controller manager until joint_state_broadcaster died.
#
#   4. ONE STACK ONLY. Two controller managers were found running together;
#      that is HARD CONSTRAINT 3 and it produces a graph nobody can reason
#      about.
#
#   5. WRIST CAMERAS AFTER THE ARM, ONE LAUNCH PER ARM. Kinova allows two
#      connections per stream with a 30 s inactivity timeout, and killed
#      camera processes LEAK those connections -- three launches were found
#      running, two on the left arm, and the third could not start a stream.
#
#   6. EVERYTHING ON DOMAIN 7. Domain 0's shared memory is polluted on this
#      host; the arms live on 7 and a process on 0 sees an empty graph.
set -uo pipefail
WS="$HOME/kortex_ws"
cd "$WS"
# INHERITED, NOT PINNED -- see scripts/bringup_arm.sh for the measurement.
# A caller already on a domain (the GUI is on 0) keeps it; a bare terminal
# run still gets 7.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=SHM
S="$WS/.scratch"
mkdir -p "$S"

say () { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# ------------------------------------------------------------------ 1. stop
say "1/7  stopping everything (bridges with SIGINT -- never KILL)"
for p in $(pgrep -f 'kortex_highlevel_bridge' 2>/dev/null); do
    kill -INT "$p" 2>/dev/null
done
sleep 12
for pat in 'srl_gui[.]py' rviz2 'ros2 launch' kinova_vision ros2_control_node \
           move_group robot_state_publisher spawner ik_follower estop_node \
           fsr_gripper recovery_manager mount_guard apriltag object_pose_tracker \
           intent_inference grasp_generator handover_arbiter pointing_direction \
           sim_to_real real_homing master_pose_node srl_scene_camera_node \
           srl_realsense_node srl_object_detector srl_scene_understanding \
           quest_bridge_node vr_pose_mapper vr_safety_node vr_gripper_node \
           vr_feedback_node kortex_highlevel_bridge calibrate_environment \
           srl_pick_topdown srl_pick_cube; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 5
echo "   stopped."

# --------------------------------------------------------- 2. clear the SHM
say "2/7  clearing stale Fast DDS shared memory"
before=$(ls /dev/shm 2>/dev/null | grep -c fastrtps || true)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 2
echo "   $before segment(s) cleared, $(ls /dev/shm 2>/dev/null | grep -c fastrtps || echo 0) remain"

# ------------------------------------------------------------------ 3. sim
say "3/7  simulation + MoveIt (master:=false -- no Teensy on this rig)"
setsid nohup "$S/sim.sh" > "$S/sim.log" 2>&1 < /dev/null &
for i in $(seq 1 40); do
    sleep 3
    if grep -q 'Configured and activated joint_state_broadcaster' "$S/sim.log" 2>/dev/null; then
        echo "   joint_state_broadcaster active after ${i}0s"
        break
    fi
done
sleep 8

# -------------------------------------------------------------- 4. the arms
say "4/7  arm bridges (one Kortex session each)"
for arm in left right; do
    ip=$([ "$arm" = left ] && echo 192.168.1.10 || echo 192.168.1.9)
    if timeout 3 ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        echo "   $arm ($ip) is up -- connecting"
        bash scripts/bringup_arm.sh "$arm" 2>&1 | sed 's/^/     /' | tail -3
    else
        echo "   $arm ($ip) is OFF THE NETWORK -- skipped"
    fi
done

# ----------------------------------------------------------- 5. room cameras
say "5/7  scene cameras"
setsid nohup "$S/env.sh" python3 -u scripts/srl_scene_camera_node.py \
    > "$S/usb.log" 2>&1 < /dev/null &
sleep 10
# 848x480 @ 6 is the LOWEST-BANDWIDTH depth mode this D435i offers, and the
# only one that survives usbip here: 640x480 depth+colour enumerated fine and
# delivered nothing, 27 pipeline restarts in a row.
setsid nohup "$S/env.sh" ./.venv_vision/bin/python -u scripts/srl_realsense_node.py \
    --depth-only --width 848 --height 480 --fps 6 \
    > "$S/rs.log" 2>&1 < /dev/null &
sleep 12
echo "   usb: $(grep -c 'scene camera on' "$S/usb.log" 2>/dev/null || echo 0) started"
echo "   realsense: $(grep -c 'colour K' "$S/rs.log" 2>/dev/null || echo 0) started"

# ------------------------------------------------------------- 6. perception
say "6/7  scene understanding (table first, then what is on it)"
setsid nohup "$S/env.sh" python3 -u scripts/srl_scene_understanding.py \
    --save /tmp/scene_live.png > "$S/scene.log" 2>&1 < /dev/null &
sleep 8

# ------------------------------------------------------------------- 7. GUI
say "7/7  the operations window"
setsid nohup bash scripts/start_gui.sh > "$S/gui.log" 2>&1 < /dev/null &
sleep 30

say "STATE"
for n in ros2_control_node move_group kortex_highlevel_bridge kinova_vision \
         srl_scene_camera_node srl_realsense_node srl_scene_understanding \
         'srl_gui[.]py'; do
    printf '   %-28s %s\n' "$n" \
        "$(pgrep -cf "$n" 2>/dev/null || echo 0)"
done
echo
echo "Everything is on ROS_DOMAIN_ID=7."
echo "In the window: ARMS -> CONNECT is already done; use 5 FULL AUTONOMY ->"
echo "SIM ONLY (the bridges are already up, so SIM + REAL would open a"
echo "SECOND Kortex session, which the arm refuses)."
