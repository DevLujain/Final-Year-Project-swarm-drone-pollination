#!/bin/bash
# ============================================================
# FULL SWARM LAUNCH — Week 9 Integration
#
# Launches complete pipeline:
#   3x PX4 SITL → XRCE Bridge → ROS 2 nodes per drone:
#   flower_detector_sim + pollination_controller + swarm_coordinator
#   + mission_logger
#
# HOW TO USE:
#   bash launch_swarm_full.sh
#   Wait ~20 seconds for all components to start
#   Watch Mission Logger for live pollination events
# ============================================================

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

echo "╔══════════════════════════════════════════════════════╗"
echo "║  SWARM FULL LAUNCH — Week 9                         ║"
echo "╚══════════════════════════════════════════════════════╝"

# Kill leftovers
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 8

# ── PX4 Drone 0 — spawns Gazebo world
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
echo "→ Drone 0 starting (Gazebo will open)..."
sleep 10

# ── PX4 Drone 1
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
echo "→ Drone 1 starting..."
sleep 6

# ── PX4 Drone 2
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
echo "→ Drone 2 starting..."
sleep 6

# ── XRCE Bridge
gnome-terminal --title="XRCE Bridge" -- bash -c "
  source ~/.bashrc
  MicroXRCEAgent udp4 -p 8888
  bash
" &
echo "→ XRCE Bridge starting..."
sleep 4

# ── ROS 2 nodes per drone
for DRONE_ID in 0 1 2; do

  gnome-terminal --title="Detector Drone $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    python3 $ROS2_WS/src/precision_pollination/precision_pollination/flower_detector_sim.py --ros-args -p drone_id:=$DRONE_ID
    bash
  " &

  gnome-terminal --title="Controller Drone $DRONE_ID" -- bash -c "
    source ~/.bashrc
    source $ROS2_WS/install/setup.bash
    python3 $ROS2_WS/src/precision_pollination/precision_pollination/pollination_controller.py --ros-args -p drone_id:=$DRONE_ID
    bash
  " &

  sleep 2
done

# ── Swarm Coordinator
gnome-terminal --title="Swarm Coordinator" -- bash -c "
  source ~/.bashrc
  source $ROS2_WS/install/setup.bash
  python3 $ROS2_WS/src/precision_pollination/precision_pollination/swarm_coordinator.py
  bash
" &
sleep 2

# ── Mission Logger ★
gnome-terminal --title="★ Mission Logger ★" -- bash -c "
  source ~/.bashrc
  source $ROS2_WS/install/setup.bash
  python3 $ROS2_WS/src/precision_pollination/precision_pollination/mission_logger.py
  bash
" &

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  ✓ All components launched!                         ║"
echo "║                                                      ║"
echo "║  Watch: ★ Mission Logger ★ terminal                 ║"
echo "║  Target: 9/9 flowers pollinated                     ║"
echo "║  Target: coverage >= 90%                            ║"
echo "║  Target: cycle time < 30s                           ║"
echo "╚══════════════════════════════════════════════════════╝"
