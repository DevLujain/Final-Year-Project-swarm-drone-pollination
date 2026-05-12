#!/bin/bash
# ============================================================
# LAUNCH STAGE 10 — Readiness-Filtered Swarm Pollination
# FYP: Autonomous Swarm Drone Pollination
#
# Pipeline:
#   PX4 x3 → XRCE → flower_detector_sim x3
#           → flower_readiness_filter x3   (NEW)
#           → lawnmower_sweep x3 (bloom_ready_targets)
#           → shared_visited_list
#           → readiness_logger             (NEW)
#
# RUN: bash ~/Desktop/FYP/launch_stage10.sh
# ============================================================

set -e

export GZ_IP=127.0.0.1
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
PX4_DIR=~/PX4-Autopilot
WORLD=sunflower_field_realistic

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  LAUNCH STAGE 10 — Readiness-Filtered Pollination   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

pkill -9 -f "bin/px4"               2>/dev/null || true
pkill -9 -f MicroXRCEAgent          2>/dev/null || true
pkill -9 -f "gz sim"                2>/dev/null || true
pkill -9 -f lawnmower_sweep         2>/dev/null || true
pkill -9 -f shared_visited_list     2>/dev/null || true
pkill -9 -f flower_detector_sim     2>/dev/null || true
pkill -9 -f flower_readiness_filter 2>/dev/null || true
pkill -9 -f readiness_logger        2>/dev/null || true
sleep 4

rm -f $PX4_DIR/build/px4_sitl_default/rootfs/*/px4.pid 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

WORLD_FILE=$PX4_DIR/Tools/simulation/gz/worlds/${WORLD}.sdf
[ ! -f "$WORLD_FILE" ] && WORLD=sunflower_field && \
    WORLD_FILE=$PX4_DIR/Tools/simulation/gz/worlds/sunflower_field.sdf

echo "[1] Starting Gazebo..."
GZ_IP=127.0.0.1 gz sim -s -r $WORLD_FILE &
sleep 10

echo "[2] Launching 3 drones..."
for i in 0 1 2; do
    X_POS=$((i * 2))
    gnome-terminal --title="PX4 Drone $i [S10]" -- bash -c "
        export GZ_IP=127.0.0.1
        export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models
        cd $PX4_DIR
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL_POSE='${X_POS},0,0,0,0,0' \
        PX4_GZ_MODEL=x500 PX4_GZ_WORLD=$WORLD PX4_GZ_STANDALONE=1 \
        ./build/px4_sitl_default/bin/px4 -i $i; exec bash" &
    sleep 6
done

echo "[3] Starting XRCE bridge..."
gnome-terminal --title="XRCE [S10]" -- bash -c "
    MicroXRCEAgent udp4 -p 8888; exec bash" &
sleep 10

echo "[4] Launching ROS 2 nodes..."

gnome-terminal --title="Shared Visited [S10]" -- bash -c "
    source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination shared_visited_list; exec bash" &
sleep 2

gnome-terminal --title="Readiness Logger [S10]" -- bash -c "
    source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination readiness_logger; exec bash" &
sleep 2

for DRONE in 0 1 2; do
    gnome-terminal --title="Detector $DRONE [S10]" -- bash -c "
        source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash
        ros2 run precision_pollination flower_detector_sim \
            --ros-args -p drone_id:=$DRONE; exec bash" &
    sleep 1

    gnome-terminal --title="Readiness $DRONE [S10]" -- bash -c "
        source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash
        ros2 run precision_pollination flower_readiness_filter \
            --ros-args -p drone_id:=$DRONE; exec bash" &
    sleep 1

    gnome-terminal --title="Lawnmower $DRONE [S10]" -- bash -c "
        source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash
        ros2 run precision_pollination lawnmower_sweep \
            --ros-args -p drone_id:=$DRONE \
                       -p target_topic:=/drone_${DRONE}/bloom_ready_targets; exec bash" &
    sleep 2
done

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 10 RUNNING                                    ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Monitor readiness filtering:                        ║"
echo "║    ros2 topic echo /swarm/readiness_status          ║"
echo "║                                                      ║"
echo "║  Monitor bloom-ready targets (Drone 0):             ║"
echo "║    ros2 topic echo /drone_0/bloom_ready_targets     ║"
echo "║                                                      ║"
echo "║  Efficiency vs Stage 9:                              ║"
echo "║    Stage 9:  9/9 flowers targeted every sweep       ║"
echo "║    Stage 10: 6/9 bloom-ready only  (33% fewer)     ║"
echo "╚══════════════════════════════════════════════════════╝"
