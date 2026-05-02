#!/bin/bash
# ============================================================
# Stage 8 Task 2 — 30-Minute Stress Test
# Runs full swarm pipeline continuously, logs every 30s
# Target: system stable, no crashes, coverage maintained
# ============================================================

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES=$ROS2_WS/src/precision_pollination/precision_pollination
LOG_DIR=$FYP_DIR/fyp_training/stress_test
DURATION=1800  # 30 minutes in seconds

mkdir -p $LOG_DIR

echo "╔══════════════════════════════════════════════════════╗"
echo "║  Stage 8 Task 2 — 30-Minute Stress Test             ║"
echo "║  Start: $(date)                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Duration: 30 minutes"
echo "Log: $LOG_DIR"
echo ""

# Kill any leftover nodes
pkill -f gz_drone_controller 2>/dev/null
pkill -f gz_pose_bridge 2>/dev/null
pkill -f mission_logger 2>/dev/null
pkill -f flower_detector_sim 2>/dev/null
sleep 2

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

# Launch GZ pose bridge
gnome-terminal --title="GZ Bridge [Stress]" -- bash -c "
  source $ROS2_WS/install/setup.bash
  python3 $NODES/gz_pose_bridge.py; bash" &
sleep 3

# Launch 3 drone controllers
for DRONE_ID in 0 1 2; do
  gnome-terminal --title="Drone $DRONE_ID [Stress]" -- bash -c "
    source $ROS2_WS/install/setup.bash
    python3 $NODES/gz_drone_controller.py --ros-args -p drone_id:=$DRONE_ID
    bash" &
  sleep 1
done

# Launch detectors
for DRONE_ID in 0 1 2; do
  gnome-terminal --title="Detector $DRONE_ID [Stress]" -- bash -c "
    source $ROS2_WS/install/setup.bash
    python3 $NODES/flower_detector_sim.py --ros-args -p drone_id:=$DRONE_ID
    bash" &
  sleep 1
done

# Launch mission logger with extended logging
STRESS_LOG=$LOG_DIR/stress_test_$(date +%Y%m%d_%H%M%S).csv
gnome-terminal --title="★ Stress Logger ★" -- bash -c "
  source $ROS2_WS/install/setup.bash
  python3 $NODES/mission_logger.py 2>&1 | tee $LOG_DIR/stress_output.txt
  bash" &

echo "✓ All nodes launched. Running for 30 minutes..."
echo "  Watch: ★ Stress Logger ★ terminal"
echo ""

# Monitor every 30 seconds and log node health
START=$(date +%s)
CYCLE=0

while true; do
  sleep 30
  NOW=$(date +%s)
  ELAPSED=$((NOW - START))
  CYCLE=$((CYCLE + 1))
  MINS=$((ELAPSED / 60))
  SECS=$((ELAPSED % 60))

  # Count running nodes
  NODES_RUNNING=$(pgrep -f "gz_drone_controller\|mission_logger\|gz_pose_bridge\|flower_detector_sim" | wc -l)

  echo "[$(date +%H:%M:%S)] Elapsed: ${MINS}m${SECS}s | Cycle: $CYCLE | Nodes running: $NODES_RUNNING"
  echo "$(date +%H:%M:%S),$ELAPSED,$CYCLE,$NODES_RUNNING" >> $LOG_DIR/health_log.csv

  if [ $ELAPSED -ge $DURATION ]; then
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  STRESS TEST COMPLETE — 30 minutes elapsed          ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo "  Health log: $LOG_DIR/health_log.csv"
    echo "  Mission log: $LOG_DIR/stress_output.txt"
    break
  fi
done
