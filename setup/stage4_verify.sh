#!/bin/bash
# ============================================================
# STAGE 4 — End-to-End Verification
# FYP: Swarm Drone Pollination
#
# Run this AFTER stages 1, 2, and 3 are complete.
# It checks every component is installed and working,
# then launches the full simulation stack.
#
# RUN: bash stage4_verify.sh
# TIME: ~5 minutes
# ============================================================

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 4 — System Verification & Launch             ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

PASS=0
FAIL=0

check() {
  local label="$1"
  local cmd="$2"
  if eval "$cmd" &>/dev/null; then
    echo "  ✅  $label"
    PASS=$((PASS+1))
  else
    echo "  ❌  $label  ← PROBLEM"
    FAIL=$((FAIL+1))
  fi
}

echo "[1/5] Checking installed software..."
check "Ubuntu 24.04"           "lsb_release -r | grep -q '24.04'"
check "Gazebo Harmonic"        "gz sim --version 2>&1 | grep -q 'Gazebo'"
check "PX4 cloned"             "[ -d $HOME/PX4-Autopilot ]"
check "PX4 built (gz_x500)"    "[ -f $HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]"
check "MicroXRCEAgent"         "which MicroXRCEAgent"
check "ROS 2 Jazzy"            "[ -d /opt/ros/jazzy ]"
check "ros_gz bridge"          "[ -d /opt/ros/jazzy/lib/ros_gz_bridge ]"
check "Python3"                "python3 --version"
check "ultralytics (YOLOv8)"   "python3 -c 'import ultralytics'"
check "cv2 (OpenCV)"           "python3 -c 'import cv2'"
check "ROS 2 workspace built"  "[ -d $HOME/ros2_ws/install ]"
check "pollination_controller" "[ -f $HOME/ros2_ws/src/precision_pollination/precision_pollination/pollination_controller.py ]"
check "yolov8_detector"        "[ -f $HOME/ros2_ws/src/precision_pollination/precision_pollination/yolov8_detector.py ]"

echo ""
echo "Result: $PASS passed, $FAIL failed"

if [ $FAIL -gt 0 ]; then
  echo ""
  echo "⚠️  Some checks failed. Fix the ❌ items above before continuing."
  echo "    Re-run the relevant stage script, then come back here."
  echo ""
  exit 1
fi

echo ""
echo "✅  All checks passed. Launching simulation stack..."
echo ""
echo "════════════════════════════════════════════════════════"
echo " Opening 4 terminals. Each one runs one component."
echo " Watch for errors in each window."
echo "════════════════════════════════════════════════════════"
echo ""

# ── Launch everything in separate terminals ──────────────────
# Uses gnome-terminal (Ubuntu desktop default)

echo "[2/5] Terminal 1 — PX4 SITL + Gazebo..."
gnome-terminal --title="PX4 SITL" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  echo '>>> Starting PX4 SITL with Gazebo Harmonic...'
  echo '>>> You should see a quadcopter appear in the Gazebo window'
  make px4_sitl gz_x500
  bash
" &
sleep 5

echo "[3/5] Terminal 2 — XRCE-DDS bridge (PX4 <-> ROS 2)..."
gnome-terminal --title="XRCE Bridge" -- bash -c "
  source ~/.bashrc
  echo '>>> Starting micro-XRCE-DDS agent...'
  echo '>>> This bridges PX4 topics into ROS 2'
  echo '>>> Wait for: [UDPv4AgentLinux] running in execution thread'
  sleep 5
  MicroXRCEAgent udp4 -p 8888
  bash
" &
sleep 3

echo "[4/5] Terminal 3 — YOLOv8 detector node..."
gnome-terminal --title="YOLOv8 Detector" -- bash -c "
  source ~/.bashrc
  source ~/ros2_ws/install/setup.bash
  echo '>>> Starting YOLOv8 detector node...'
  echo '>>> Subscribes to: /camera/image_raw'
  echo '>>> Publishes to:  /sunflower_targets'
  sleep 8
  ros2 run precision_pollination yolov8_detector
  bash
" &
sleep 3

echo "[5/5] Terminal 4 — Pollination state machine..."
gnome-terminal --title="State Machine" -- bash -c "
  source ~/.bashrc
  source ~/ros2_ws/install/setup.bash
  echo '>>> Starting pollination controller (state machine)...'
  echo '>>> States: IDLE -> TAKEOFF -> SEARCH -> APPROACH -> POLLINATE -> LAND'
  sleep 12
  ros2 run precision_pollination pollination_controller
  bash
" &

echo ""
echo "════════════════════════════════════════════════════════"
echo " ALL COMPONENTS LAUNCHED"
echo "════════════════════════════════════════════════════════"
echo ""
echo "VERIFY THE SYSTEM IS WORKING:"
echo ""
echo "  In a new terminal, run these checks:"
echo ""
echo "  ros2 topic list"
echo "    → You should see /fmu/out/vehicle_local_position"
echo "    → You should see /sunflower_targets"
echo "    → You should see /pollination/log"
echo ""
echo "  ros2 topic echo /fmu/out/vehicle_local_position"
echo "    → You should see position numbers updating in real time"
echo ""
echo "  ros2 topic echo /pollination/log"
echo "    → Will show pollination events once the drone finds flowers"
echo ""
echo "SCREENSHOT CHECKLIST (for your FYP1 report):"
echo "  □ Gazebo window with drone visible"
echo "  □ ros2 topic list output"
echo "  □ PX4 SITL console showing flight mode"
echo "  □ XRCE bridge showing connected topics"
echo ""
echo "If Gazebo did not open, run manually:"
echo "  cd ~/PX4-Autopilot && make px4_sitl gz_x500"
echo ""
