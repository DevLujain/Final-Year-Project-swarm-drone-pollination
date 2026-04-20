#!/bin/bash
# ============================================================
# FULL INTEGRATION LAUNCH — Week 9
# Launches complete end-to-end pipeline:
#
#   PX4 x3 → XRCE Bridge → ROS 2 → Camera Bridge x3
#   → YOLOv8 Detector x3 → Position Estimator x3
#   → Pollination Controller x3 → Swarm Coordinator
#   → Mission Logger
#
# HOW TO USE:
#   1. Run this script: bash launch_integration.sh
#   2. Wait ~30 seconds for all components to initialize
#   3. Watch the Mission Logger terminal for live metrics
#   4. Check ~/Desktop/FYP/fyp_training/mission_log_*.csv for results
# ============================================================

MODEL_PATH="/home/joney/Desktop/FYP/ros2_ws/sunflower_best.pt"
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

echo "╔══════════════════════════════════════════════════════╗"
echo "║  FULL INTEGRATION LAUNCH — Week 9                   ║"
echo "╚══════════════════════════════════════════════════════╝"

# Kill leftovers
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 3

# ── Launch PX4 Drone 0 (spawns Gazebo world)
gnome-terminal --title="PX4 Drone 0" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 0
  bash
" &
echo "→ Drone 0 launching..."
sleep 8

# ── Launch PX4 Drone 1
gnome-terminal --title="PX4 Drone 1" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='2,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 1
  bash
" &
echo "→ Drone 1 launching..."
sleep 5

# ── Launch PX4 Drone 2
gnome-terminal --title="PX4 Drone 2" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='4,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 2
  bash
" &
echo "→ Drone 2 launching..."
sleep 5

# ── Launch XRCE Bridge
gnome-terminal --title="XRCE Bridge" -- bash -c "
  source ~/.bashrc
  MicroXRCEAgent udp4 -p 8888
  bash
" &
echo "→ XRCE Bridge launching..."
sleep 3

# ── Launch ROS 2 nodes for each drone
for DRONE_ID in 0 1 2; do

  # Camera Bridge
  gnome-terminal --title="Camera Bridge $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination camera_bridge \
      --ros-args -p drone_id:=$DRONE_ID
    bash
  " &

  # YOLOv8 Detector
  gnome-terminal --title="YOLOv8 Drone $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination yolov8_detector \
      --ros-args -p drone_id:=$DRONE_ID -p model_path:=$MODEL_PATH
    bash
  " &

  # Position Estimator
  gnome-terminal --title="Position Est $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination position_estimator \
      --ros-args -p drone_id:=$DRONE_ID
    bash
  " &

  # Pollination Controller
  gnome-terminal --title="Controller $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination pollination_controller \
      --ros-args -p drone_id:=$DRONE_ID
    bash
  " &

  sleep 2
done

# ── Launch Swarm Coordinator
gnome-terminal --title="Swarm Coordinator" -- bash -c "
  source ~/.bashrc
  source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_coordinator
  bash
" &
echo "→ Swarm coordinator launching..."
sleep 2

# ── Launch Mission Logger
gnome-terminal --title="Mission Logger ★" -- bash -c "
  source ~/.bashrc
  source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination mission_logger
  bash
" &
echo "→ Mission logger launching..."

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  ✓ All components launched!                         ║"
echo "║                                                      ║"
echo "║  Watch: Mission Logger terminal for live metrics     ║"
echo "║  Check: ~/Desktop/FYP/fyp_training/mission_log_*    ║"
echo "║                                                      ║"
echo "║  VERIFY in new terminal:                             ║"
echo "║    ros2 topic list | wc -l  (expect 40+ topics)     ║"
echo "╚══════════════════════════════════════════════════════╝"
