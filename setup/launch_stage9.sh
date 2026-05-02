#!/bin/bash
# ============================================================
# LAUNCH STAGE 9 — Realistic Field + Lawnmower Swarm
# FYP: Autonomous Swarm Drone Pollination
#
# Kills everything, relaunches in correct order:
#   1. Gazebo server (sunflower_field_realistic.sdf)
#   2. PX4 SITL x3 (via gnome-terminal tabs)
#   3. XRCE bridge
#   4. ROS 2 nodes: shared_visited_list + lawnmower_sweep x3
#
# RUN: bash setup/launch_stage9.sh
# ============================================================

set -e

export GZ_IP=127.0.0.1
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
PX4_DIR=~/PX4-Autopilot
WORLD=sunflower_field_realistic

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  LAUNCH STAGE 9 — Realistic Swarm Pollination       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Kill everything cleanly ───────────────────────────────
echo "[1/5] Killing all leftover processes..."
pkill -9 -f "bin/px4"         2>/dev/null || true
pkill -9 -f MicroXRCEAgent    2>/dev/null || true
pkill -9 -f "gz sim"          2>/dev/null || true
pkill -9 -f lawnmower_sweep   2>/dev/null || true
pkill -9 -f shared_visited_list 2>/dev/null || true
pkill -9 -f swarm_coordinator 2>/dev/null || true
sleep 4

# Clean PX4 lock files
rm -f $PX4_DIR/build/px4_sitl_default/rootfs/*/px4.pid 2>/dev/null || true
echo "✓ All processes killed"

# ── 2. Source ROS 2 ──────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

# ── 3. Start Gazebo server ───────────────────────────────────
echo ""
echo "[2/5] Starting Gazebo server..."
WORLD_FILE=$PX4_DIR/Tools/simulation/gz/worlds/${WORLD}.sdf

# Fall back to original world if realistic not found
if [ ! -f "$WORLD_FILE" ]; then
    echo "⚠ Realistic world not found, using sunflower_field.sdf"
    WORLD=sunflower_field
    WORLD_FILE=$PX4_DIR/Tools/simulation/gz/worlds/sunflower_field.sdf
fi

GZ_IP=127.0.0.1 gz sim -s -r $WORLD_FILE &
GZ_PID=$!
echo "✓ Gazebo server started (PID $GZ_PID)"

# Wait for Gazebo to be ready
echo "  Waiting for Gazebo world to load..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "$WORLD"; then
        echo "✓ Gazebo world ready after ${i}s"
        break
    fi
    sleep 2
    if [ $i -eq 30 ]; then
        echo "✗ Gazebo failed to load after 60s. Exiting."
        exit 1
    fi
done

# ── 4. Launch 3 PX4 drones in separate terminals ─────────────
echo ""
echo "[3/5] Launching 3 PX4 SITL drones..."

gnome-terminal --title="PX4 Drone 0 [Stage9]" -- bash -c "
    export GZ_IP=127.0.0.1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models
    cd $PX4_DIR
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \
    PX4_GZ_MODEL=x500 \
    PX4_GZ_WORLD=$WORLD \
    PX4_GZ_STANDALONE=1 \
    ./build/px4_sitl_default/bin/px4 -i 0
    exec bash
" &
sleep 6

gnome-terminal --title="PX4 Drone 1 [Stage9]" -- bash -c "
    export GZ_IP=127.0.0.1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models
    cd $PX4_DIR
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE='2,0,0,0,0,0' \
    PX4_GZ_MODEL=x500 \
    PX4_GZ_WORLD=$WORLD \
    PX4_GZ_STANDALONE=1 \
    ./build/px4_sitl_default/bin/px4 -i 1
    exec bash
" &
sleep 6

gnome-terminal --title="PX4 Drone 2 [Stage9]" -- bash -c "
    export GZ_IP=127.0.0.1
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models
    cd $PX4_DIR
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE='4,0,0,0,0,0' \
    PX4_GZ_MODEL=x500 \
    PX4_GZ_WORLD=$WORLD \
    PX4_GZ_STANDALONE=1 \
    ./build/px4_sitl_default/bin/px4 -i 2
    exec bash
" &
sleep 8
echo "✓ Drone terminals launched"

# ── 5. Start XRCE bridge ─────────────────────────────────────
echo ""
echo "[4/5] Starting XRCE bridge..."
gnome-terminal --title="XRCE Bridge [Stage9]" -- bash -c "
    echo '=== XRCE Bridge — auto-restart on crash ==='
    while true; do
        MicroXRCEAgent udp4 -p 8888
        echo '⚠ XRCE bridge exited — restarting in 3s...'
        sleep 3
    done
" &
sleep 12

# Verify topics
echo "  Verifying ROS 2 topics..."
source /opt/ros/jazzy/setup.bash
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "vehicle_local_position"; then
        echo "✓ PX4 topics publishing after ${i}x2s"
        ros2 topic list | grep vehicle_local_position
        break
    fi
    sleep 2
    echo "  Waiting for topics... ${i}/20"
    if [ $i -eq 20 ]; then
        echo "✗ Topics not found after 40s — check XRCE Bridge tab"
        exit 1
    fi
done

# ── 6. Launch ROS 2 nodes ────────────────────────────────────
echo ""
echo "[5/5] Launching ROS 2 swarm nodes..."

gnome-terminal --title="Shared Visited List [Stage9]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination shared_visited_list
    exec bash
" &
sleep 3

gnome-terminal --title="Lawnmower Drone 0 [Stage9]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination lawnmower_sweep --ros-args -p drone_id:=0
    exec bash
" &
sleep 2

gnome-terminal --title="Lawnmower Drone 1 [Stage9]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination lawnmower_sweep --ros-args -p drone_id:=1
    exec bash
" &
sleep 2

gnome-terminal --title="Lawnmower Drone 2 [Stage9]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination lawnmower_sweep --ros-args -p drone_id:=2
    exec bash
" &

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  ALL COMPONENTS LAUNCHED                             ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Terminals open:                                     ║"
echo "║  • PX4 Drone 0, 1, 2                                ║"
echo "║  • XRCE Bridge                                       ║"
echo "║  • Shared Visited List                               ║"
echo "║  • Lawnmower Drone 0, 1, 2                          ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Monitor coverage:                                   ║"
echo "║  ros2 topic echo /swarm/coverage_status             ║"
echo "╚══════════════════════════════════════════════════════╝"
