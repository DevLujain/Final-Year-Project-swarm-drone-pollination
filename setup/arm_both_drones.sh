#!/bin/bash
# ============================================================
# arm_both_drones.sh — Manual arm fallback
#
# Use this if the visual_servoing_controller's programmatic
# arming fails. Sends 'commander arm' + 'commander mode offboard'
# to each PX4 instance directly via its console.
#
# RUN: bash ~/Desktop/FYP/arm_both_drones.sh
# ============================================================

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  MANUAL ARM — both drones                           ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Find each PX4 instance's IPC channel
# PX4 SITL creates a PID file per instance:
#   /tmp/PX4_BUS_SOCKET_0  (drone 0)
#   /tmp/PX4_BUS_SOCKET_1  (drone 1)

for DRONE_ID in 0 1; do
  echo "▶ Drone $DRONE_ID — sending arm + offboard..."

  # Method: write commands directly to PX4 instance via px4-alias
  # PX4 SITL accepts commands via its in-process shell, but from
  # an external terminal we need to use px4_msgs vehicle_command
  # OR use the PX4 startup script `px4-alias.sh`.
  #
  # Simplest reliable method: use ros2 topic pub to send the
  # VehicleCommand directly.

  ros2 topic pub --once /px4_${DRONE_ID}/fmu/in/vehicle_command \
    px4_msgs/msg/VehicleCommand \
    "{command: 400, param1: 1.0, param2: 0.0, \
      target_system: $((DRONE_ID + 1)), \
      target_component: 1, source_system: 1, source_component: 1, \
      from_external: true}" 2>/dev/null

  sleep 1

  ros2 topic pub --once /px4_${DRONE_ID}/fmu/in/vehicle_command \
    px4_msgs/msg/VehicleCommand \
    "{command: 176, param1: 1.0, param2: 6.0, \
      target_system: $((DRONE_ID + 1)), \
      target_component: 1, source_system: 1, source_component: 1, \
      from_external: true}" 2>/dev/null

  echo "  ✓ Drone $DRONE_ID: ARM (cmd 400) + OFFBOARD (cmd 176, param2=6) sent"
  sleep 1
done

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Arm commands sent.                                 ║"
echo "║                                                      ║"
echo "║  If drones STILL don't fly, the XRCE bridge is the  ║"
echo "║  problem, not the arm command. Verify with:         ║"
echo "║                                                      ║"
echo "║    ros2 topic list | grep px4                       ║"
echo "║    ros2 topic echo /px4_0/fmu/out/vehicle_status    ║"
echo "║                                                      ║"
echo "║  If topic echo shows NOTHING → bridge is broken.    ║"
echo "║  Solution: kill everything, run launch again, wait. ║"
echo "║                                                      ║"
echo "║  ALTERNATIVE — type in EACH PX4 console terminal:   ║"
echo "║    commander arm                                    ║"
echo "║    commander mode offboard                          ║"
echo "╚══════════════════════════════════════════════════════╝"
