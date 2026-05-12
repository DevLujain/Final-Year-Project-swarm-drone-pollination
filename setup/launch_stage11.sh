#!/bin/bash
# ============================================================
# LAUNCH STAGE 11 — TSP-Optimised Swarm Pollination
# RUN: bash ~/Desktop/FYP/launch_stage11.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 11 — TSP-Optimised Pollination               ║"
echo "║  Nearest Neighbour + 2-opt per drone per run        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  TSP log: ~/Desktop/FYP/setup/tsp_log.csv"
echo "  Monitor: ros2 topic echo /swarm/tsp_summary"
echo ""

# Reset pollination log for clean 3-run demo
if [ -f "$FYP_DIR/setup/pollination_log.csv" ]; then
    echo "  Backing up existing pollination log..."
    cp $FYP_DIR/setup/pollination_log.csv \
       $FYP_DIR/setup/pollination_log_backup_$(date +%Y%m%d_%H%M%S).csv
    rm $FYP_DIR/setup/pollination_log.csv
    echo "  Log reset for clean run."
fi

sleep 2

# Launch TSP planner
gnome-terminal --title="TSP Planner [S11]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    echo 'TSP Planner ready — waiting for targets...'
    ros2 run precision_pollination tsp_planner
    exec bash" &
sleep 3

# Launch mission manager
ros2 run precision_pollination persistent_pollination_manager
