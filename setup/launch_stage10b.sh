#!/bin/bash
# ============================================================
# LAUNCH STAGE 10b — Persistent Multi-Run Pollination
# FYP: Autonomous Swarm Drone Pollination
#
# Runs 3 automatic pollination sweeps in one session.
# Each sweep targets only newly-opened flowers.
# All results are saved to CSV and persist across sessions.
#
# WHAT TO SCREENSHOT FOR YOUR REPORT:
#   1. The RUN 1 / RUN 2 / RUN 3 banners in terminal
#   2. The live flower-by-flower pollination stream
#   3. The MISSION COMPLETE summary
#   4. ~/Desktop/FYP/setup/pollination_log.csv opened in
#      LibreOffice Calc — shows all 3 runs with timestamps
#
# RUN: bash ~/Desktop/FYP/launch_stage10b.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 10b — Persistent Multi-Run Pollination       ║"
echo "║  3 automatic sweeps — 300 flowers — CSV logging     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  Field:    300 flowers across 3 sectors"
echo "  Sweeps:   3 automatic runs (60s gap between each)"
echo "  Log:      ~/Desktop/FYP/setup/pollination_log.csv"
echo ""
echo "  Flowers pollinated in Run 1 will NOT appear in Run 2."
echo "  Flowers pollinated in Run 2 will NOT appear in Run 3."
echo "  This persists even if you restart the script."
echo ""
echo "  Starting in 3 seconds..."
sleep 3

ros2 run precision_pollination persistent_pollination_manager
