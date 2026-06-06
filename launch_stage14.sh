#!/bin/bash
# launch_stage14.sh — Stage 14 (no PX4, Gazebo VelocityControl)
#
# Sequence:
#   1. Generate fresh random 6-flower field (writes SDF + JSON)
#   2. Kill any leftover processes
#   3. Launch Gazebo, GZ<->ROS2 bridge, then 6 ROS 2 nodes:
#        - field_survey  per drone
#        - swarm_drone_controller per drone
#        - swarm_coordinator
#        - swarm_monitor

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
WORLD=~/PX4-Autopilot/Tools/simulation/gz/worlds/demo_stage14.sdf

echo ""
echo "============================================================"
echo "  STAGE 14 - Swarm Pollination Demo"
echo "  2 drones | 6 random flowers | peer-to-peer assignment"
echo "============================================================"
echo ""

# 1. Generate a fresh field
python3 "$FYP_DIR/generate_field.py"

# 2. Kill any leftover processes
pkill -9 -f 'gz sim'         2>/dev/null || true
pkill -9 -f gz-sim           2>/dev/null || true
pkill -9 -f gzserver         2>/dev/null || true
pkill -9 -f ruby             2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f px4              2>/dev/null || true
pkill -9 -f MicroXRCEAgent   2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 3

# 3a. Gazebo
gnome-terminal --title="Gazebo" -- bash -c "
  source ~/.bashrc
  echo '>>> Starting Gazebo with $WORLD'
  gz sim -r '$WORLD'
  bash" &
sleep 10

# 3b. ROS 2 <-> Gazebo bridge
gnome-terminal --title="GZ Bridge" -- bash -c "
  source ~/.bashrc
  echo '>>> Bridging ROS 2 <-> Gazebo'
  ros2 run ros_gz_bridge parameter_bridge \\
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \\
    /model/drone_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \\
    /model/drone_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose
  bash" &
sleep 5

# 3c. Per-drone survey + controller
for D in 0 1; do
  gnome-terminal --title="D${D} survey" -- bash -c "
    source ~/.bashrc; source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination field_survey --ros-args -p drone_id:=$D
    bash" &
  sleep 1
  gnome-terminal --title="D${D} controller" -- bash -c "
    source ~/.bashrc; source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination swarm_drone_controller --ros-args -p drone_id:=$D
    bash" &
  sleep 1
done

# 3d. Coordinator
gnome-terminal --title="Coordinator" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_coordinator
  bash" &
sleep 1

# 3e. Monitor
gnome-terminal --title="Monitor" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_monitor
  bash" &

echo ""
echo "============================================================"
echo "  Launched. 8 terminal tabs open:"
echo "    Gazebo | GZ Bridge | D0 survey | D0 controller |"
echo "    D1 survey | D1 controller | Coordinator | Monitor"
echo ""
echo "  Expected timeline:"
echo "    t=0..10s   Gazebo loads world with new random field"
echo "    t~15s      Coordinator fires GO -> both drones TAKEOFF"
echo "    t~20s      Both at 3 m cruise -> fly to survey waypoints"
echo "    t~25s      Field survey complete, negotiation runs"
echo "    t~30..80s  Each drone pollinates its 3 flowers at 2 m"
echo "               (GOTO 3m -> DESCEND 2m -> HOVER 1.5s -> ASCEND 3m)"
echo "    t~85s      RETURN to spawn, LAND"
echo "    t~95s      SWARM MISSION COMPLETE"
echo "============================================================"
