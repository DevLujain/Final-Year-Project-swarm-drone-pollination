#!/bin/bash
# ============================================================
# LAUNCH STAGE 12 — Health Mapper + KPI Logger
# FYP: Autonomous Swarm Drone Pollination
#
# RUN ORDER:
#   Terminal A: bash ~/Desktop/FYP/launch_stage11.sh   (first)
#   Terminal B: bash ~/Desktop/FYP/launch_stage12.sh   (second)
#
# This script does NOT relaunch Gazebo or drones.
# It only starts the two Stage 12 analysis nodes:
#   - field_health_mapper  → builds the heatmap
#   - kpi_logger           → tracks KPI targets
#
# Press Ctrl+C in each terminal to stop and save outputs.
#
# OUTPUTS (saved to OUTPUT_DIR on Ctrl+C):
#   health_map.png          — heatmap image for FYP report
#   health_map.csv          — raw grid values
#   detections_raw.csv      — every detection event
#   kpi_report.txt          — paste into Chapter 5
#   kpi_report.json         — machine-readable KPIs
#   pollination_events.csv  — timestamped pollination log
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

# ── Create timestamped output directory ──────────────────────
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=$FYP_DIR/mission_outputs/stage12_${TIMESTAMP}
mkdir -p $OUTPUT_DIR

# Also update the 'latest' symlink so it's easy to find
rm -f $FYP_DIR/mission_outputs/latest
ln -sf $OUTPUT_DIR $FYP_DIR/mission_outputs/latest

export STAGE12_OUTPUT_DIR=$OUTPUT_DIR

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 12 — Health Mapper + KPI Logger              ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  ⚠  Make sure launch_stage11.sh is already running  ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Output: $OUTPUT_DIR"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Topics expected:                                   ║"
echo "║    /drone_0/flower_detections                       ║"
echo "║    /drone_1/flower_detections                       ║"
echo "║    /drone_2/flower_detections                       ║"
echo "║    /drone_0/pollination/log  (and 1, 2)             ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Verifying topics are available..."
echo "(If you see no topics, make sure launch_stage11.sh is running)"
echo ""

# Quick check — just list topics, don't block
ros2 topic list 2>/dev/null | grep -E "flower_detections|pollination" || \
    echo "  ⚠  No matching topics yet. Start launch_stage11.sh first."

echo ""
echo "Starting Stage 12 nodes in separate terminals..."
sleep 1

# ── Field Health Mapper ───────────────────────────────────────
gnome-terminal \
    --title="🌻 Health Mapper [S12]" \
    -- bash -c "
        source /opt/ros/jazzy/setup.bash
        source $ROS2_WS/install/setup.bash
        export STAGE12_OUTPUT_DIR=$OUTPUT_DIR
        echo '🌻 Field Health Mapper — Stage 12'
        echo '   Subscribes to /drone_N/flower_detections'
        echo '   Press Ctrl+C to stop and save heatmap PNG'
        echo ''
        ros2 run precision_pollination field_health_mapper
        exec bash
    " &

sleep 2

# ── KPI Logger ────────────────────────────────────────────────
gnome-terminal \
    --title="📊 KPI Logger [S12]" \
    -- bash -c "
        source /opt/ros/jazzy/setup.bash
        source $ROS2_WS/install/setup.bash
        export STAGE12_OUTPUT_DIR=$OUTPUT_DIR
        echo '📊 KPI Logger — Stage 12'
        echo '   Subscribes to detections + pollination/log'
        echo '   Prints KPI summary every 30s'
        echo '   Press Ctrl+C to stop and save kpi_report.txt'
        echo ''
        ros2 run precision_pollination kpi_logger
        exec bash
    " &

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  BOTH STAGE 12 NODES LAUNCHED                       ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  To monitor topics manually:                        ║"
echo "║    ros2 topic echo /drone_0/flower_detections       ║"
echo "║    ros2 topic echo /drone_0/pollination/log         ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  When done: press Ctrl+C in each Stage 12 terminal  ║"
echo "║  Then check: ls $OUTPUT_DIR"
echo "║  Open map:   eog $OUTPUT_DIR/health_map.png"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
