#!/bin/bash
# ============================================================
# STAGE 5 — Multi-Drone Simulation (Week 7)
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS SCRIPT DOES:
#   1. Kills any leftover PX4/Gazebo processes
#   2. Creates a sunflower field world for Gazebo
#   3. Creates a multi-drone launch script (3 drones)
#   4. Creates the swarm coordinator ROS 2 node
#   5. Rebuilds the ROS 2 workspace with all nodes
#
# RUN: bash stage5_multidrone.sh
# TIME: ~10 minutes
# ============================================================

set -e

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 5 — Multi-Drone Simulation (Week 7)          ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# ── 1. Kill all leftover processes ───────────────────────────
echo "[1/5] Cleaning up leftover processes..."
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 3
echo "✓ Processes cleaned"

# ── 2. Create sunflower field world ──────────────────────────
echo ""
echo "[2/5] Creating sunflower field Gazebo world..."

WORLDS_DIR=~/PX4-Autopilot/Tools/simulation/gz/worlds

cat > $WORLDS_DIR/sunflower_field.sdf << 'WORLDEOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="sunflower_field">

    <!-- Physics -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Plugins -->
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>

    <!-- Sun -->
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0.4 0.2 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0.1 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground plane (green = field) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.2 0.6 0.2 1</ambient>
            <diffuse>0.2 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sunflower markers — 3x3 grid representing sunflower positions -->
    <!-- Row 1 -->
    <model name="flower_0_0"><static>true</static><link name="link">
      <pose>5 5 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_0_1"><static>true</static><link name="link">
      <pose>5 10 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_0_2"><static>true</static><link name="link">
      <pose>5 15 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <!-- Row 2 -->
    <model name="flower_1_0"><static>true</static><link name="link">
      <pose>10 5 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_1_1"><static>true</static><link name="link">
      <pose>10 10 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_1_2"><static>true</static><link name="link">
      <pose>10 15 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <!-- Row 3 -->
    <model name="flower_2_0"><static>true</static><link name="link">
      <pose>15 5 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_2_1"><static>true</static><link name="link">
      <pose>15 10 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

    <model name="flower_2_2"><static>true</static><link name="link">
      <pose>15 15 0.5 0 0 0</pose>
      <visual name="stem"><geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
        <material><ambient>0.2 0.6 0.1 1</ambient><diffuse>0.2 0.6 0.1 1</diffuse></material></visual>
    </link></model>

  </world>
</sdf>
WORLDEOF

echo "✓ sunflower_field.sdf created"

# ── 3. Create multi-drone launch script ───────────────────────
echo ""
echo "[3/5] Creating multi-drone launch script..."

cat > ~/launch_swarm.sh << 'LAUNCHEOF'
#!/bin/bash
# ============================================================
# SWARM LAUNCH SCRIPT — 3 Drones in Gazebo
# 
# Run this script to launch 3 drones simultaneously.
# It opens 5 terminals automatically:
#   Terminal 1: Gazebo simulation world
#   Terminal 2: PX4 Drone 0 (sector: left)
#   Terminal 3: PX4 Drone 1 (sector: center)
#   Terminal 4: PX4 Drone 2 (sector: right)
#   Terminal 5: XRCE Bridge
# ============================================================

echo "Killing any leftover processes..."
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 3

echo "Launching Gazebo world..."
gnome-terminal --title="Gazebo World" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  gz sim -r Tools/simulation/gz/worlds/sunflower_field.sdf
  bash
" &
sleep 5

echo "Launching Drone 0 (left sector, position 0,0)..."
gnome-terminal --title="PX4 Drone 0" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 0
  bash
" &
sleep 4

echo "Launching Drone 1 (center sector, position 2,0)..."
gnome-terminal --title="PX4 Drone 1" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='2,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 1
  bash
" &
sleep 4

echo "Launching Drone 2 (right sector, position 4,0)..."
gnome-terminal --title="PX4 Drone 2" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE='4,0,0,0,0,0' \
  PX4_GZ_MODEL=x500 \
  PX4_GZ_WORLD=sunflower_field \
  ./build/px4_sitl_default/bin/px4 -i 2
  bash
" &
sleep 4

echo "Launching XRCE Bridge..."
gnome-terminal --title="XRCE Bridge" -- bash -c "
  source ~/.bashrc
  MicroXRCEAgent udp4 -p 8888
  bash
" &

echo ""
echo "✓ All components launched!"
echo ""
echo "Verify in a new terminal:"
echo "  ros2 topic list | grep fmu"
echo "  → Should show topics for /px4_0, /px4_1, /px4_2"
LAUNCHEOF

chmod +x ~/launch_swarm.sh
echo "✓ launch_swarm.sh created at ~/"

# ── 4. Create swarm coordinator node ─────────────────────────
echo ""
echo "[4/5] Creating swarm coordinator ROS 2 node..."

mkdir -p ~/ros2_ws/src/precision_pollination/precision_pollination

# Recreate pollination controller
cat > ~/ros2_ws/src/precision_pollination/precision_pollination/pollination_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
Pollination Controller Node
===========================
State machine for autonomous drone pollination.
States: IDLE -> TAKEOFF -> SEARCH -> APPROACH -> HOVER -> POLLINATE -> NEXT -> LAND
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

    def __init__(self):
        super().__init__('pollination_controller')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ns = f'/px4_{self.drone_id}'

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        self.flower_sub = self.create_subscription(
            PoseArray,
            f'/drone_{self.drone_id}/sunflower_targets',
            self._flower_callback,
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.log_pub = self.create_publisher(
            String, f'/drone_{self.drone_id}/pollination/log', 10
        )

        self.state = State.IDLE
        self.current_pos = None
        self.flower_targets = []
        self.pollination_log = []
        self.hover_count = 0

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'Drone {self.drone_id} pollination controller started. State: {self.state.value}'
        )

    def _position_callback(self, msg):
        self.current_pos = msg

    def _flower_callback(self, msg):
        self.flower_targets = msg.poses
        if self.state == State.SEARCH and len(self.flower_targets) > 0:
            self.state = State.APPROACH
            self.get_logger().info(f'Drone {self.drone_id}: Flower detected! APPROACH')

    def _control_loop(self):
        if self.state == State.IDLE:
            self.state = State.TAKEOFF

        elif self.state == State.TAKEOFF:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)
            if self.current_pos and abs(self.current_pos.z + 3.0) < 0.3:
                self.state = State.SEARCH
                self.get_logger().info(f'Drone {self.drone_id}: Takeoff complete, SEARCH')

        elif self.state == State.SEARCH:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)

        elif self.state == State.APPROACH:
            if self.flower_targets:
                t = self.flower_targets[0]
                self._publish_setpoint(x=t.position.x, y=t.position.y, z=-1.0)
                if self.current_pos:
                    dx = self.current_pos.x - t.position.x
                    dy = self.current_pos.y - t.position.y
                    if (dx**2 + dy**2)**0.5 < 0.5:
                        self.state = State.HOVER
                        self.hover_count = 0

        elif self.state == State.HOVER:
            if self.flower_targets:
                t = self.flower_targets[0]
                self._publish_setpoint(x=t.position.x, y=t.position.y, z=-0.3)
                self.hover_count += 1
                if self.hover_count > 30:  # 3 seconds at 10Hz
                    self.state = State.POLLINATE

        elif self.state == State.POLLINATE:
            self.get_logger().info(f'Drone {self.drone_id}: POLLINATING flower!')
            entry = {
                'drone_id': self.drone_id,
                'flower': str(self.flower_targets[0]) if self.flower_targets else 'unknown',
                'count': len(self.pollination_log) + 1
            }
            self.pollination_log.append(entry)
            log_msg = String()
            log_msg.data = json.dumps(entry)
            self.log_pub.publish(log_msg)
            self.state = State.NEXT

        elif self.state == State.NEXT:
            if self.flower_targets:
                self.flower_targets.pop(0)
            self.state = State.SEARCH if self.flower_targets else State.LAND

        elif self.state == State.LAND:
            self._publish_setpoint(x=0.0, y=0.0, z=0.0)
            self.get_logger().info(
                f'Drone {self.drone_id}: Mission complete. '
                f'Pollinated {len(self.pollination_log)} flowers'
            )

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)


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

# Recreate YOLOv8 detector
cat > ~/ros2_ws/src/precision_pollination/precision_pollination/yolov8_detector.py << 'PYEOF'
#!/usr/bin/env python3
"""
YOLOv8 Sunflower Detector Node
================================
Subscribes to drone camera feed, runs YOLOv8, publishes flower positions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Float32
import json
import os

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

FOCAL_LENGTH = 320.0
IMAGE_CX = 320.0
IMAGE_CY = 240.0


class YOLOv8Detector(Node):

    def __init__(self):
        super().__init__('yolov8_detector')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('drone_id', 0)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.drone_id = self.get_parameter('drone_id').value

        if YOLO_AVAILABLE:
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                self.get_logger().info(f'Loaded model: {model_path}')
            else:
                self.model = YOLO('yolov8n.pt')
                self.get_logger().warn('Custom model not found, using yolov8n.pt')
        else:
            self.model = None
            self.get_logger().error('ultralytics not installed!')

        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self.current_altitude = 3.0

        self.img_sub = self.create_subscription(
            Image,
            f'/drone_{self.drone_id}/camera/image_raw',
            self._image_callback,
            10
        )

        self.alt_sub = self.create_subscription(
            Float32,
            f'/drone_{self.drone_id}/altitude',
            lambda msg: setattr(self, 'current_altitude', msg.data),
            10
        )

        self.detections_pub = self.create_publisher(
            String, f'/drone_{self.drone_id}/yolov8/detections', 10
        )
        self.targets_pub = self.create_publisher(
            PoseArray, f'/drone_{self.drone_id}/sunflower_targets', 10
        )

        self.get_logger().info(
            f'YOLOv8 detector node started for drone {self.drone_id}. '
            f'Waiting for camera frames...'
        )

    def _image_callback(self, msg):
        if self.model is None or self.bridge is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        detections = []
        pose_array = PoseArray()
        pose_array.header = msg.header

        for result in results:
            for box in result.boxes:
                x_center = float(box.xywh[0][0])
                y_center = float(box.xywh[0][1])
                conf = float(box.conf[0])

                real_x = (x_center - IMAGE_CX) * self.current_altitude / FOCAL_LENGTH
                real_y = (y_center - IMAGE_CY) * self.current_altitude / FOCAL_LENGTH

                detections.append({
                    'px': x_center, 'py': y_center,
                    'conf': conf,
                    'real_x': real_x, 'real_y': real_y
                })

                pose = Pose()
                pose.position.x = real_x
                pose.position.y = real_y
                pose.position.z = 0.0
                pose_array.poses.append(pose)

        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detections_pub.publish(det_msg)
        self.targets_pub.publish(pose_array)

        if detections:
            self.get_logger().info(
                f'Drone {self.drone_id}: Detected {len(detections)} flower(s)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Detector()
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

# Create swarm coordinator
cat > ~/ros2_ws/src/precision_pollination/precision_pollination/swarm_coordinator.py << 'PYEOF'
#!/usr/bin/env python3
"""
Swarm Coordinator Node
======================
Divides the field into 3 sectors and assigns one sector per drone.
Monitors all drone positions and prevents double-pollination.

Sector assignment (static decomposition):
  Drone 0 → Left sector   (x: 0-7m,   y: 0-20m)
  Drone 1 → Center sector (x: 7-13m,  y: 0-20m)
  Drone 2 → Right sector  (x: 13-20m, y: 0-20m)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
import json


# Field sector definitions — divide 20x20m field into 3 columns
SECTORS = {
    0: {'x_min': 0,  'x_max': 7,  'y_min': 0, 'y_max': 20, 'name': 'LEFT'},
    1: {'x_min': 7,  'x_max': 13, 'y_min': 0, 'y_max': 20, 'name': 'CENTER'},
    2: {'x_min': 13, 'x_max': 20, 'y_min': 0, 'y_max': 20, 'name': 'RIGHT'},
}

NUM_DRONES = 3


class SwarmCoordinator(Node):

    def __init__(self):
        super().__init__('swarm_coordinator')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Track position of each drone
        self.drone_positions = {i: None for i in range(NUM_DRONES)}
        self.pollinated_flowers = set()  # de-duplication set

        # Subscribe to all drone positions
        for i in range(NUM_DRONES):
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{i}/fmu/out/vehicle_local_position',
                lambda msg, drone_id=i: self._position_callback(msg, drone_id),
                px4_qos
            )

            # Subscribe to pollination logs to track completed flowers
            self.create_subscription(
                String,
                f'/drone_{i}/pollination/log',
                self._pollination_callback,
                10
            )

        # Publish sector assignments
        self.sector_pub = self.create_publisher(String, '/swarm/sector_map', 10)

        # Status timer — prints swarm status every 5 seconds
        self.status_timer = self.create_timer(5.0, self._publish_status)

        # Publish sector assignments immediately
        self._publish_sectors()

        self.get_logger().info('Swarm coordinator started!')
        self.get_logger().info(f'Field divided into {NUM_DRONES} sectors:')
        for drone_id, sector in SECTORS.items():
            self.get_logger().info(
                f'  Drone {drone_id} → {sector["name"]} sector '
                f'(x: {sector["x_min"]}-{sector["x_max"]}m, '
                f'y: {sector["y_min"]}-{sector["y_max"]}m)'
            )

    def _position_callback(self, msg, drone_id):
        self.drone_positions[drone_id] = {
            'x': msg.x, 'y': msg.y, 'z': msg.z
        }

    def _pollination_callback(self, msg):
        try:
            data = json.loads(msg.data)
            flower_key = data.get('flower', '')
            if flower_key in self.pollinated_flowers:
                self.get_logger().warn(
                    f'Duplicate pollination detected! Flower already visited.'
                )
            else:
                self.pollinated_flowers.add(flower_key)
                self.get_logger().info(
                    f'Flower pollinated by Drone {data.get("drone_id")}. '
                    f'Total: {len(self.pollinated_flowers)}'
                )
        except Exception as e:
            self.get_logger().error(f'Log parse error: {e}')

    def _publish_sectors(self):
        sector_msg = String()
        sector_msg.data = json.dumps(SECTORS)
        self.sector_pub.publish(sector_msg)

    def _publish_status(self):
        self._publish_sectors()
        self.get_logger().info('=== SWARM STATUS ===')
        for drone_id in range(NUM_DRONES):
            pos = self.drone_positions[drone_id]
            if pos:
                sector = SECTORS[drone_id]
                in_sector = (
                    sector['x_min'] <= pos['x'] <= sector['x_max'] and
                    sector['y_min'] <= pos['y'] <= sector['y_max']
                )
                self.get_logger().info(
                    f'  Drone {drone_id}: x={pos["x"]:.1f} y={pos["y"]:.1f} '
                    f'z={pos["z"]:.1f} | In sector: {in_sector}'
                )
            else:
                self.get_logger().info(f'  Drone {drone_id}: No position data yet')
        self.get_logger().info(
            f'  Total flowers pollinated: {len(self.pollinated_flowers)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
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

echo "✓ All ROS 2 nodes written"

# ── 5. Update setup.py and rebuild ───────────────────────────
echo ""
echo "[5/5] Updating setup.py and rebuilding workspace..."

cat > ~/ros2_ws/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system — Week 7',
    entry_points={
        'console_scripts': [
            'pollination_controller = precision_pollination.pollination_controller:main',
            'yolov8_detector        = precision_pollination.yolov8_detector:main',
            'swarm_coordinator      = precision_pollination.swarm_coordinator:main',
        ],
    },
)
SETUPEOF

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -15

echo "✓ Workspace rebuilt with 3 nodes"

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 5 COMPLETE — Week 7 Ready                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "WHAT WAS CREATED:"
echo "  • sunflower_field.sdf    — Gazebo world with 9 flower markers"
echo "  • ~/launch_swarm.sh      — launches all 3 drones + bridge"
echo "  • swarm_coordinator.py   — sector assignment + de-duplication"
echo "  • pollination_controller — updated with drone_id parameter"
echo "  • yolov8_detector        — updated with drone_id parameter"
echo ""
echo "TO LAUNCH THE SWARM:"
echo "  bash ~/launch_swarm.sh"
echo ""
echo "THEN VERIFY IN A NEW TERMINAL:"
echo "  source ~/.bashrc"
echo "  ros2 topic list | grep fmu"
echo "  → Should show /px4_0, /px4_1, /px4_2 topics"
echo ""
echo "TO START THE SWARM COORDINATOR:"
echo "  ros2 run precision_pollination swarm_coordinator"
echo ""
echo "NEXT: Run stage6_training.sh to train YOLOv8 on your dataset"
echo ""
