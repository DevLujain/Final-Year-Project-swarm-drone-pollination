#!/bin/bash
# ============================================================
# STAGE 2 — ROS 2 Jazzy Installation (Ubuntu 24.04)
# FYP: Swarm Drone Pollination
#
# WHY JAZZY and not Humble?
#   ROS 2 Humble was built for Ubuntu 22.04.
#   Ubuntu 24.04 uses ROS 2 Jazzy Jalopy (LTS, released May 2024).
#   Trying to install Humble on 24.04 causes dependency conflicts.
#   Jazzy has the same features as Humble — your code works identically.
#
# WHAT THIS SCRIPT DOES:
#   1. Adds the ROS 2 apt repository for Ubuntu 24.04
#   2. Installs ROS 2 Jazzy Desktop (includes rviz2, rqt, etc.)
#   3. Installs colcon (the build tool for ROS 2 packages)
#   4. Installs ros_gz bridge (connects Gazebo Harmonic to ROS 2)
#   5. Installs px4_msgs (PX4's ROS 2 message definitions)
#   6. Creates your ROS 2 workspace
#
# RUN: bash stage2_ros2_ubuntu24.sh  (do NOT use sudo for this one)
# TIME: ~15 minutes
# ============================================================

set -e

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 2 — ROS 2 Jazzy Installation                 ║"
echo "║  Ubuntu 24.04 + Gazebo Harmonic                      ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Add ROS 2 repository ───────────────────────────────────
echo "[1/6] Adding ROS 2 Jazzy repository..."

sudo apt-get install -y software-properties-common curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update -y
echo "✓ ROS 2 repository added"

# ── 2. Install ROS 2 Jazzy Desktop ───────────────────────────
# "Desktop" includes: core ROS 2, rviz2 (visualizer), rqt (GUI tools),
# demo nodes for testing, and documentation.
echo ""
echo "[2/6] Installing ROS 2 Jazzy Desktop..."
echo "  This is the large download (~700MB)..."
sudo apt-get install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool

# Initialize rosdep (dependency manager for ROS packages)
sudo rosdep init 2>/dev/null || echo "  (rosdep already initialized)"
rosdep update

echo "✓ ROS 2 Jazzy Desktop installed"

# ── 3. Install Gazebo-ROS 2 bridge ───────────────────────────
# ros_gz is the official bridge between Gazebo Harmonic and ROS 2.
# It translates Gazebo topics (like camera images, drone pose)
# into ROS 2 topics that your nodes can subscribe to.
echo ""
echo "[3/6] Installing ros_gz bridge (Gazebo Harmonic <-> ROS 2)..."
sudo apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim

echo "✓ ros_gz bridge installed"

# ── 4. Create ROS 2 workspace ─────────────────────────────────
# A "workspace" is just a folder where all your ROS 2 packages live.
# src/ = your source code
# build/ = compiled code (auto-generated)
# install/ = installed packages (auto-generated)
echo ""
echo "[4/6] Creating ROS 2 workspace at ~/ros2_ws..."

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# ── 5. Clone px4_msgs and px4_ros_com ────────────────────────
# px4_msgs: defines all the ROS 2 message types PX4 uses
#   (e.g., VehicleLocalPosition, TrajectorySetpoint, VehicleCommand)
# px4_ros_com: example code and utilities for PX4+ROS 2 integration
echo ""
echo "[5/6] Cloning PX4 ROS 2 packages..."

if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git -b release/1.15
    echo "  ✓ px4_msgs cloned"
else
    echo "  px4_msgs already exists"
fi

if [ ! -d "px4_ros_com" ]; then
    git clone https://github.com/PX4/px4_ros_com.git -b release/1.15
    echo "  ✓ px4_ros_com cloned"
else
    echo "  px4_ros_com already exists"
fi

# ── 5b. Create your FYP package ──────────────────────────────
# This creates the skeleton of your own ROS 2 package.
# You'll fill in the actual nodes in later stages.
echo ""
echo "  Creating your FYP package skeleton..."

cd ~/ros2_ws/src

if [ ! -d "precision_pollination" ]; then
    source /opt/ros/jazzy/setup.bash
    ros2 pkg create precision_pollination \
        --build-type ament_python \
        --dependencies rclpy sensor_msgs geometry_msgs std_msgs px4_msgs
    echo "  ✓ precision_pollination package created"
else
    echo "  precision_pollination package already exists"
fi

# Write a basic pollination controller node
mkdir -p ~/ros2_ws/src/precision_pollination/precision_pollination
cat > ~/ros2_ws/src/precision_pollination/precision_pollination/pollination_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
Pollination Controller Node
===========================
State machine for autonomous drone pollination.

States: IDLE -> TAKEOFF -> SEARCH -> APPROACH -> HOVER -> POLLINATE -> NEXT -> LAND

This node:
  - Subscribes to /sunflower_targets (flower positions from YOLOv8)
  - Subscribes to /fmu/out/vehicle_local_position (drone GPS/position from PX4)
  - Publishes to /fmu/in/trajectory_setpoint (movement commands to PX4)
  - Publishes to /pollination/log (record of which flowers were visited)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import json
from enum import Enum


class State(Enum):
    IDLE      = "IDLE"
    TAKEOFF   = "TAKEOFF"
    SEARCH    = "SEARCH"
    APPROACH  = "APPROACH"
    HOVER     = "HOVER"
    POLLINATE = "POLLINATE"
    NEXT      = "NEXT_TARGET"
    LAND      = "LAND"


class PollinationController(Node):
    """Main state machine node for drone pollination."""

    def __init__(self):
        super().__init__('pollination_controller')

        # PX4 requires a specific QoS profile for its topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to PX4 position output
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        # Subscribe to YOLOv8 flower detections
        self.flower_sub = self.create_subscription(
            PoseArray,
            '/sunflower_targets',
            self._flower_callback,
            10
        )

        # Publish movement commands to PX4
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )

        # Publish pollination log
        self.log_pub = self.create_publisher(String, '/pollination/log', 10)

        # State
        self.state = State.IDLE
        self.current_pos = None
        self.flower_targets = []
        self.pollination_log = []

        # Control loop — runs 10 times per second
        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(f'Pollination controller started. State: {self.state.value}')

    def _position_callback(self, msg):
        self.current_pos = msg

    def _flower_callback(self, msg):
        self.flower_targets = msg.poses
        if self.state == State.SEARCH and len(self.flower_targets) > 0:
            self.state = State.APPROACH
            self.get_logger().info(f'Flower detected! Switching to APPROACH')

    def _control_loop(self):
        """Called 10x/second — checks state and acts."""
        self.get_logger().debug(f'State: {self.state.value}')

        if self.state == State.IDLE:
            pass  # Waiting for start_mission() call

        elif self.state == State.TAKEOFF:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)  # 3m altitude (NED: negative z = up)

        elif self.state == State.SEARCH:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)  # hover while scanning

        elif self.state == State.APPROACH:
            if self.flower_targets:
                target = self.flower_targets[0]
                self._publish_setpoint(x=target.position.x, y=target.position.y, z=-0.5)

        elif self.state == State.HOVER:
            if self.flower_targets:
                target = self.flower_targets[0]
                self._publish_setpoint(x=target.position.x, y=target.position.y, z=-0.3)

        elif self.state == State.POLLINATE:
            self.get_logger().info('Simulating pollination event...')
            self.pollination_log.append({'target': str(self.flower_targets[0]) if self.flower_targets else 'unknown'})
            log_msg = String()
            log_msg.data = json.dumps(self.pollination_log[-1])
            self.log_pub.publish(log_msg)
            self.state = State.NEXT

        elif self.state == State.NEXT:
            if self.flower_targets:
                self.flower_targets.pop(0)
            self.state = State.SEARCH if self.flower_targets else State.LAND

        elif self.state == State.LAND:
            self._publish_setpoint(x=0.0, y=0.0, z=0.0)
            self.get_logger().info(f'Mission complete. Pollinated: {len(self.pollination_log)} flowers')

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)

    def start_mission(self):
        if self.state == State.IDLE:
            self.state = State.TAKEOFF
            self.get_logger().info('Mission started!')


def main(args=None):
    rclpy.init(args=args)
    node = PollinationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF

echo "  ✓ pollination_controller.py written"

# ── 6. Build workspace ────────────────────────────────────────
echo ""
echo "[6/6] Building ROS 2 workspace..."
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install 2>&1 | tail -30

# Add to bashrc so it auto-sources
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || \
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

grep -qxF 'source ~/ros2_ws/install/setup.bash' ~/.bashrc || \
    echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc

echo "✓ Workspace built and added to ~/.bashrc"

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 2 COMPLETE                                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "WHAT WAS INSTALLED:"
echo "  • ROS 2 Jazzy Desktop        — messaging framework"
echo "  • ros_gz bridge              — Gazebo <-> ROS 2"
echo "  • px4_msgs                   — PX4 message types"
echo "  • precision_pollination pkg  — your FYP package"
echo ""
echo "QUICK TEST (open a new terminal first):"
echo "  ros2 topic list              — should show /rosout"
echo "  ros2 run demo_nodes_py talker  — basic publish test"
echo ""
echo "KEY FILES:"
echo "  ~/ros2_ws/src/precision_pollination/precision_pollination/"
echo "    pollination_controller.py  — your state machine"
echo ""
echo "NEXT: Run stage3_yolov8_node.sh to build the vision node"
echo ""
