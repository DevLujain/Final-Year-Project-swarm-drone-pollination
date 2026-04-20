#!/bin/bash
# ============================================================
# STAGE 7 — Full System Integration (Week 9)
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS SCRIPT DOES:
#   1. Verifies trained YOLOv8 model exists
#   2. Adds a downward camera to the x500 drone model
#   3. Creates the camera bridge node (Gazebo → ROS 2)
#   4. Creates the position estimator node (pixels → GPS)
#   5. Creates the mission logger node (CSV metrics)
#   6. Creates the full integration launch script
#   7. Rebuilds the ROS 2 workspace
#
# PRE-REQUISITES:
#   ✅ Stage 5 complete (3-drone swarm working)
#   ✅ Stage 6 complete (trained model at ~/Desktop/FYP/fyp_training/)
#
# RUN: bash stage7_integration.sh
# TIME: ~10 minutes setup, then run launch_integration.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=~/Desktop/FYP/ros2_ws
MODEL_PATH=$FYP_DIR/ros2_ws/sunflower_best.pt
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 7 — Full System Integration (Week 9)         ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash 2>/dev/null || true

# ── 1. Verify trained model ───────────────────────────────────
echo "[1/7] Verifying trained YOLOv8 model..."

if [ -f "$MODEL_PATH" ]; then
    SIZE=$(du -sh "$MODEL_PATH" | cut -f1)
    echo "✓ Model found: $MODEL_PATH ($SIZE)"
else
    # Try to find it in training runs
    BEST=$(find $FYP_DIR/fyp_training -name "best.pt" 2>/dev/null | head -1)
    if [ -n "$BEST" ]; then
        cp "$BEST" "$MODEL_PATH"
        echo "✓ Model copied from: $BEST"
    else
        echo "⚠ Trained model not found. Using yolov8n.pt as fallback."
        echo "  Run stage6_training.sh first for best results."
        MODEL_PATH="yolov8n.pt"
    fi
fi

# ── 2. Add camera to x500 drone model ─────────────────────────
echo ""
echo "[2/7] Adding downward camera to x500 drone model..."

X500_MODEL_DIR=~/PX4-Autopilot/Tools/simulation/gz/models/x500
X500_SDF=$X500_MODEL_DIR/model.sdf

if [ ! -f "$X500_SDF" ]; then
    echo "⚠ x500 model not found at $X500_SDF"
    echo "  Skipping camera attachment — will use x500_mono_cam_down instead"
    DRONE_MODEL="x500_mono_cam_down"
else
    # Backup original
    cp $X500_SDF ${X500_SDF}.backup 2>/dev/null || true

    # Check if camera already added
    if grep -q "down_camera" $X500_SDF; then
        echo "✓ Camera already present in x500 model"
    else
        # Add camera sensor before closing </model> tag
        python3 << 'CAMEOF'
import re

sdf_path = '/root/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf'
import os
sdf_path = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf')

with open(sdf_path, 'r') as f:
    content = f.read()

camera_xml = '''
    <link name="down_camera_link">
      <pose>0 0 -0.1 0 1.5708 0</pose>
      <sensor name="down_camera" type="camera">
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <topic>down_camera/image</topic>
      </sensor>
    </link>
'''

# Insert before last </model>
content = content.rstrip()
if '</model>' in content:
    idx = content.rfind('</model>')
    content = content[:idx] + camera_xml + '\n</model>\n'
    with open(sdf_path, 'w') as f:
        f.write(content)
    print('Camera added to x500 model')
else:
    print('Could not find </model> tag — manual edit required')
CAMEOF
        echo "✓ Downward camera added to x500 model"
    fi
    DRONE_MODEL="x500"
fi

# ── 3. Create camera bridge node ─────────────────────────────
echo ""
echo "[3/7] Creating camera bridge node..."

cat > $NODES_DIR/camera_bridge.py << 'PYEOF'
#!/usr/bin/env python3
"""
Camera Bridge Node
==================
Bridges Gazebo camera topic to ROS 2 image topic for each drone.
Gazebo: /drone_N/down_camera/image → ROS 2: /drone_N/camera/image_raw

Also publishes drone altitude for YOLOv8 position estimation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to Gazebo camera
        self.gz_cam_sub = self.create_subscription(
            Image,
            f'/drone_{self.drone_id}/down_camera/image',
            self._camera_callback,
            10
        )

        # Subscribe to PX4 altitude
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{self.drone_id}/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        # Publish ROS 2 camera topic
        self.ros_cam_pub = self.create_publisher(
            Image,
            f'/drone_{self.drone_id}/camera/image_raw',
            10
        )

        # Publish altitude for YOLOv8 position estimation
        self.alt_pub = self.create_publisher(
            Float32,
            f'/drone_{self.drone_id}/altitude',
            10
        )

        self.frame_count = 0
        self.get_logger().info(
            f'Camera bridge started for drone {self.drone_id}'
        )

    def _camera_callback(self, msg):
        self.ros_cam_pub.publish(msg)
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Drone {self.drone_id}: {self.frame_count} frames bridged'
            )

    def _position_callback(self, msg):
        alt_msg = Float32()
        alt_msg.data = abs(float(msg.z))
        self.alt_pub.publish(alt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
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

echo "✓ camera_bridge.py written"

# ── 4. Create position estimator node ────────────────────────
echo ""
echo "[4/7] Creating position estimator node..."

cat > $NODES_DIR/position_estimator.py << 'PYEOF'
#!/usr/bin/env python3
"""
Position Estimator Node
========================
Converts YOLOv8 pixel detections to real-world GPS coordinates.

Uses pinhole camera model:
  real_x = (pixel_x - cx) * altitude / focal_length
  real_y = (pixel_y - cy) * altitude / focal_length

Then adds drone's current GPS position to get absolute flower location.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String, Float32
import json
import math

# Camera intrinsics for 640x480 image with 80° FOV
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
FOV_H = math.radians(80)
FOCAL_LENGTH = (IMAGE_WIDTH / 2) / math.tan(FOV_H / 2)
CX = IMAGE_WIDTH / 2
CY = IMAGE_HEIGHT / 2


class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.drone_pos = None
        self.altitude  = 3.0

        # Subscribe to drone position
        self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{self.drone_id}/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        # Subscribe to altitude
        self.create_subscription(
            Float32,
            f'/drone_{self.drone_id}/altitude',
            lambda msg: setattr(self, 'altitude', msg.data),
            10
        )

        # Subscribe to YOLOv8 detections (pixel coordinates)
        self.create_subscription(
            String,
            f'/drone_{self.drone_id}/yolov8/detections',
            self._detections_callback,
            10
        )

        # Publish real-world flower positions
        self.flower_pub = self.create_publisher(
            PoseArray,
            f'/drone_{self.drone_id}/flower_positions',
            10
        )

        self.get_logger().info(
            f'Position estimator started for drone {self.drone_id}. '
            f'Focal length: {FOCAL_LENGTH:.1f}px'
        )

    def _position_callback(self, msg):
        self.drone_pos = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        self.altitude = abs(float(msg.z))

    def _detections_callback(self, msg):
        if self.drone_pos is None:
            return

        try:
            detections = json.loads(msg.data)
        except Exception:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        for det in detections:
            px = det.get('px', CX)
            py = det.get('py', CY)

            # Pixel → drone-relative offset
            offset_x = (px - CX) * self.altitude / FOCAL_LENGTH
            offset_y = (py - CY) * self.altitude / FOCAL_LENGTH

            # Drone-relative → world coordinates
            flower_x = self.drone_pos['x'] + offset_x
            flower_y = self.drone_pos['y'] + offset_y

            pose = Pose()
            pose.position.x = flower_x
            pose.position.y = flower_y
            pose.position.z = 0.0
            pose_array.poses.append(pose)

        if pose_array.poses:
            self.flower_pub.publish(pose_array)
            self.get_logger().info(
                f'Drone {self.drone_id}: Estimated {len(pose_array.poses)} '
                f'flower position(s) in world frame'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PositionEstimator()
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

echo "✓ position_estimator.py written"

# ── 5. Create mission logger node ────────────────────────────
echo ""
echo "[5/7] Creating mission logger node..."

cat > $NODES_DIR/mission_logger.py << 'PYEOF'
#!/usr/bin/env python3
"""
Mission Logger Node
====================
Records all pollination events and performance metrics to CSV.
Used for FYP evaluation against quantitative targets.

Metrics logged:
  - Pollination events (drone_id, flower_id, timestamp, cycle_time)
  - Detection counts per drone
  - Field coverage estimate
  - State machine transitions

Output: ~/Desktop/FYP/fyp_training/mission_log_TIMESTAMP.csv
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import csv
import os
import json
import time
from datetime import datetime


LOG_DIR = os.path.expanduser('~/Desktop/FYP/fyp_training/')


class MissionLogger(Node):

    def __init__(self):
        super().__init__('mission_logger')

        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(LOG_DIR, f'mission_log_{timestamp}.csv')

        self.csv_file = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp', 'drone_id', 'event_type',
            'flower_x', 'flower_y', 'cycle_time_s',
            'total_pollinated', 'details'
        ])

        self.mission_start = time.time()
        self.pollination_counts = {0: 0, 1: 0, 2: 0}
        self.total_pollinated = 0
        self.detection_counts = {0: 0, 1: 0, 2: 0}
        self.last_detection_time = {0: None, 1: None, 2: None}

        # Subscribe to all drone logs
        for i in range(3):
            self.create_subscription(
                String,
                f'/drone_{i}/pollination/log',
                lambda msg, drone_id=i: self._pollination_callback(msg, drone_id),
                10
            )
            self.create_subscription(
                String,
                f'/drone_{i}/yolov8/detections',
                lambda msg, drone_id=i: self._detection_callback(msg, drone_id),
                10
            )

        # Summary timer — every 30 seconds
        self.create_timer(30.0, self._log_summary)

        self.get_logger().info(f'Mission logger started. Log: {self.log_path}')

    def _pollination_callback(self, msg, drone_id):
        try:
            data = json.loads(msg.data)
            self.total_pollinated += 1
            self.pollination_counts[drone_id] += 1

            elapsed = time.time() - self.mission_start
            self.writer.writerow([
                datetime.now().isoformat(),
                drone_id,
                'POLLINATE',
                data.get('flower_x', 0),
                data.get('flower_y', 0),
                elapsed,
                self.total_pollinated,
                json.dumps(data)
            ])
            self.csv_file.flush()

            self.get_logger().info(
                f'LOGGED: Drone {drone_id} pollinated flower '
                f'#{self.pollination_counts[drone_id]}. '
                f'Total: {self.total_pollinated}'
            )
        except Exception as e:
            self.get_logger().error(f'Log error: {e}')

    def _detection_callback(self, msg, drone_id):
        try:
            detections = json.loads(msg.data)
            if detections:
                self.detection_counts[drone_id] += len(detections)
                now = time.time()

                if self.last_detection_time[drone_id]:
                    cycle_time = now - self.last_detection_time[drone_id]
                else:
                    cycle_time = 0

                self.last_detection_time[drone_id] = now

                self.writer.writerow([
                    datetime.now().isoformat(),
                    drone_id,
                    'DETECTION',
                    0, 0,
                    cycle_time,
                    self.total_pollinated,
                    f'{len(detections)} flowers detected'
                ])
                self.csv_file.flush()
        except Exception:
            pass

    def _log_summary(self):
        elapsed = time.time() - self.mission_start
        self.get_logger().info('=== MISSION SUMMARY ===')
        self.get_logger().info(f'  Elapsed: {elapsed:.0f}s')
        self.get_logger().info(f'  Total pollinated: {self.total_pollinated}/9')
        for i in range(3):
            self.get_logger().info(
                f'  Drone {i}: {self.pollination_counts[i]} pollinated, '
                f'{self.detection_counts[i]} detections'
            )
        coverage = (self.total_pollinated / 9) * 100
        self.get_logger().info(f'  Field coverage: {coverage:.1f}% (target: 90%)')

        self.writer.writerow([
            datetime.now().isoformat(), 'ALL', 'SUMMARY',
            0, 0, elapsed, self.total_pollinated,
            f'coverage={coverage:.1f}%'
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self._log_summary()
        self.csv_file.close()
        self.get_logger().info(f'Mission log saved to: {self.log_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
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

echo "✓ mission_logger.py written"

# ── 6. Create integration launch script ──────────────────────
echo ""
echo "[6/7] Creating full integration launch script..."

cat > ~/Desktop/FYP/launch_integration.sh << LAUNCHEOF
#!/bin/bash
# ============================================================
# FULL INTEGRATION LAUNCH — Week 9
# Launches complete end-to-end pipeline:
#
#   PX4 x3 → XRCE Bridge → ROS 2 → Camera Bridge x3
#   → YOLOv8 Detector x3 → Position Estimator x3
#   → Pollination Controller x3 → Swarm Coordinator
#   → Mission Logger
#
# HOW TO USE:
#   1. Run this script: bash launch_integration.sh
#   2. Wait ~30 seconds for all components to initialize
#   3. Watch the Mission Logger terminal for live metrics
#   4. Check ~/Desktop/FYP/fyp_training/mission_log_*.csv for results
# ============================================================

MODEL_PATH="$MODEL_PATH"
FYP_DIR=~/Desktop/FYP
ROS2_WS=\$FYP_DIR/ros2_ws

echo "╔══════════════════════════════════════════════════════╗"
echo "║  FULL INTEGRATION LAUNCH — Week 9                   ║"
echo "╚══════════════════════════════════════════════════════╝"

# Kill leftovers
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 3

# ── Launch PX4 Drone 0 (spawns Gazebo world)
gnome-terminal --title="PX4 Drone 0" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \\
  PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \\
  PX4_GZ_MODEL=x500 \\
  PX4_GZ_WORLD=sunflower_field \\
  ./build/px4_sitl_default/bin/px4 -i 0
  bash
" &
echo "→ Drone 0 launching..."
sleep 8

# ── Launch PX4 Drone 1
gnome-terminal --title="PX4 Drone 1" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \\
  PX4_GZ_MODEL_POSE='2,0,0,0,0,0' \\
  PX4_GZ_MODEL=x500 \\
  PX4_GZ_WORLD=sunflower_field \\
  ./build/px4_sitl_default/bin/px4 -i 1
  bash
" &
echo "→ Drone 1 launching..."
sleep 5

# ── Launch PX4 Drone 2
gnome-terminal --title="PX4 Drone 2" -- bash -c "
  source ~/.bashrc
  cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 \\
  PX4_GZ_MODEL_POSE='4,0,0,0,0,0' \\
  PX4_GZ_MODEL=x500 \\
  PX4_GZ_WORLD=sunflower_field \\
  ./build/px4_sitl_default/bin/px4 -i 2
  bash
" &
echo "→ Drone 2 launching..."
sleep 5

# ── Launch XRCE Bridge
gnome-terminal --title="XRCE Bridge" -- bash -c "
  source ~/.bashrc
  MicroXRCEAgent udp4 -p 8888
  bash
" &
echo "→ XRCE Bridge launching..."
sleep 3

# ── Launch ROS 2 nodes for each drone
for DRONE_ID in 0 1 2; do

  # Camera Bridge
  gnome-terminal --title="Camera Bridge \$DRONE_ID" -- bash -c "
    source ~/.bashrc
    source \$ROS2_WS/install/setup.bash
    ros2 run precision_pollination camera_bridge \\
      --ros-args -p drone_id:=\$DRONE_ID
    bash
  " &

  # YOLOv8 Detector
  gnome-terminal --title="YOLOv8 Drone \$DRONE_ID" -- bash -c "
    source ~/.bashrc
    source \$ROS2_WS/install/setup.bash
    ros2 run precision_pollination yolov8_detector \\
      --ros-args -p drone_id:=\$DRONE_ID -p model_path:=\$MODEL_PATH
    bash
  " &

  # Position Estimator
  gnome-terminal --title="Position Est \$DRONE_ID" -- bash -c "
    source ~/.bashrc
    source \$ROS2_WS/install/setup.bash
    ros2 run precision_pollination position_estimator \\
      --ros-args -p drone_id:=\$DRONE_ID
    bash
  " &

  # Pollination Controller
  gnome-terminal --title="Controller \$DRONE_ID" -- bash -c "
    source ~/.bashrc
    source \$ROS2_WS/install/setup.bash
    ros2 run precision_pollination pollination_controller \\
      --ros-args -p drone_id:=\$DRONE_ID
    bash
  " &

  sleep 2
done

# ── Launch Swarm Coordinator
gnome-terminal --title="Swarm Coordinator" -- bash -c "
  source ~/.bashrc
  source \$ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_coordinator
  bash
" &
echo "→ Swarm coordinator launching..."
sleep 2

# ── Launch Mission Logger
gnome-terminal --title="Mission Logger ★" -- bash -c "
  source ~/.bashrc
  source \$ROS2_WS/install/setup.bash
  ros2 run precision_pollination mission_logger
  bash
" &
echo "→ Mission logger launching..."

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  ✓ All components launched!                         ║"
echo "║                                                      ║"
echo "║  Watch: Mission Logger terminal for live metrics     ║"
echo "║  Check: ~/Desktop/FYP/fyp_training/mission_log_*    ║"
echo "║                                                      ║"
echo "║  VERIFY in new terminal:                             ║"
echo "║    ros2 topic list | wc -l  (expect 40+ topics)     ║"
echo "╚══════════════════════════════════════════════════════╝"
LAUNCHEOF

chmod +x ~/Desktop/FYP/launch_integration.sh
echo "✓ launch_integration.sh created"

# ── 7. Update setup.py and rebuild ───────────────────────────
echo ""
echo "[7/7] Updating setup.py and rebuilding workspace..."

cat > $ROS2_WS/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system — Week 9 Integration',
    entry_points={
        'console_scripts': [
            'pollination_controller=precision_pollination.pollination_controller:main',
            'yolov8_detector=precision_pollination.yolov8_detector:main',
            'swarm_coordinator=precision_pollination.swarm_coordinator:main',
            'camera_bridge=precision_pollination.camera_bridge:main',
            'position_estimator=precision_pollination.position_estimator:main',
            'mission_logger=precision_pollination.mission_logger:main',
        ],
    },
)
SETUPEOF

source /opt/ros/jazzy/setup.bash
cd $ROS2_WS

# Clean and rebuild
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null || true
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -10

# Fix executables
mkdir -p $ROS2_WS/install/precision_pollination/lib/precision_pollination

for NODE in pollination_controller yolov8_detector swarm_coordinator camera_bridge position_estimator mission_logger; do
cat > $ROS2_WS/install/precision_pollination/lib/precision_pollination/$NODE << NODEEOF
#!/usr/bin/env python3
import sys
sys.path.insert(0, '$ROS2_WS/src/precision_pollination')
from precision_pollination.$NODE import main
main()
NODEEOF
chmod +x $ROS2_WS/install/precision_pollination/lib/precision_pollination/$NODE
done

echo "✓ All 6 nodes registered"

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 7 COMPLETE — Week 9 Ready                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "NEW NODES CREATED:"
echo "  • camera_bridge.py      — Gazebo camera → ROS 2 image"
echo "  • position_estimator.py — pixels → real-world GPS"
echo "  • mission_logger.py     — CSV metrics for FYP evaluation"
echo ""
echo "FULL PIPELINE:"
echo "  PX4 → XRCE → Camera → YOLOv8 → Position → Controller → Logger"
echo ""
echo "TO LAUNCH FULL INTEGRATION:"
echo "  bash ~/Desktop/FYP/launch_integration.sh"
echo ""
echo "METRICS SAVED TO:"
echo "  ~/Desktop/FYP/fyp_training/mission_log_TIMESTAMP.csv"
echo ""
echo "FYP TARGETS TO VERIFY:"
echo "  ✓ mAP@0.5 ≥ 0.85       (from Stage 6 training)"
echo "  ✓ Cycle time < 30s     (from mission_log CSV)"
echo "  ✓ Coverage ≥ 90%       (9/9 flowers pollinated)"
echo "  ✓ 30-min stress test   (let it run for 30 min)"
echo ""
echo "NEXT: Run stage8_validation.sh for stress test + real footage"
echo ""
