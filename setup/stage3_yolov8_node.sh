#!/bin/bash
# ============================================================
# STAGE 3 — YOLOv8 ROS 2 Node + Sunflower Dataset
# FYP: Swarm Drone Pollination
#
# WHAT THIS SCRIPT DOES:
#   1. Creates the YOLOv8 detector ROS 2 node
#      (reads camera images, runs YOLO, publishes flower positions)
#   2. Downloads a sunflower dataset from Roboflow Universe
#      (or sets up the folder structure if you have your own)
#   3. Writes a training script you can run to train the model
#
# RUN: bash stage3_yolov8_node.sh
# TIME: ~10 minutes (not counting dataset download)
# ============================================================

set -e

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 3 — YOLOv8 Vision Node                       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# ── 1. Create YOLOv8 detector node ───────────────────────────
echo "[1/3] Writing YOLOv8 detector node..."

VISION_DIR=~/ros2_ws/src/precision_pollination/precision_pollination
mkdir -p $VISION_DIR

cat > $VISION_DIR/yolov8_detector.py << 'PYEOF'
#!/usr/bin/env python3
"""
YOLOv8 Sunflower Detector Node
================================
Subscribes to the drone's camera feed, runs YOLOv8 inference on every
frame, and publishes detected flower positions.

Subscribes:
    /camera/image_raw  (sensor_msgs/Image)  — raw camera frames

Publishes:
    /yolov8/detections  (std_msgs/String)         — JSON list of bboxes
    /sunflower_targets  (geometry_msgs/PoseArray) — 3D flower positions

HOW IT ESTIMATES 3D POSITION:
    The drone camera sees a 2D image. To get the real-world (X, Y) position
    of a flower, we use the pinhole camera model:
        real_x = (pixel_x - cx) * altitude / focal_length
        real_y = (pixel_y - cy) * altitude / focal_length
    Where cx, cy is the image center, and altitude is the drone's height.
    This gives us approximate ground position relative to the drone.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import json
import os

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# Camera intrinsics — adjust to match your simulated camera in Gazebo
# For a typical 640x480 camera with ~90 degree FOV:
FOCAL_LENGTH = 320.0   # pixels (approximate)
IMAGE_CX = 320.0       # image center x (half of 640)
IMAGE_CY = 240.0       # image center y (half of 480)


class YOLOv8Detector(Node):

    def __init__(self):
        super().__init__('yolov8_detector')

        # Parameter: path to trained model weights
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        # Load YOLOv8
        if YOLO_AVAILABLE:
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                self.get_logger().info(f'Loaded model: {model_path}')
            else:
                # Fall back to pretrained YOLOv8n if custom model not found
                self.model = YOLO('yolov8n.pt')
                self.get_logger().warn(f'Custom model not found, using yolov8n.pt')
        else:
            self.model = None
            self.get_logger().error('ultralytics not installed! Run: pip install ultralytics')

        self.bridge = CvBridge()
        self.current_altitude = 3.0  # metres — updated from /altitude topic

        # Subscribe to camera
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10
        )

        # Subscribe to altitude (so 3D estimation stays accurate as drone moves)
        self.alt_sub = self.create_subscription(
            Float32,
            '/drone/altitude',
            lambda msg: setattr(self, 'current_altitude', msg.data),
            10
        )

        # Publishers
        self.detections_pub = self.create_publisher(String, '/yolov8/detections', 10)
        self.targets_pub = self.create_publisher(PoseArray, '/sunflower_targets', 10)

        self.get_logger().info('YOLOv8 detector node started. Waiting for camera frames...')

    def _image_callback(self, msg):
        """Called every time a new camera frame arrives."""
        if self.model is None:
            return

        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8 inference
        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        detections = []
        pose_array = PoseArray()
        pose_array.header = msg.header

        for result in results:
            for box in result.boxes:
                # Bounding box center in pixels
                x_center = float(box.xywh[0][0])
                y_center = float(box.xywh[0][1])
                width    = float(box.xywh[0][2])
                height   = float(box.xywh[0][3])
                conf     = float(box.conf[0])

                # Estimate real-world position using pinhole model
                real_x = (x_center - IMAGE_CX) * self.current_altitude / FOCAL_LENGTH
                real_y = (y_center - IMAGE_CY) * self.current_altitude / FOCAL_LENGTH

                detections.append({
                    'px': x_center, 'py': y_center,
                    'w': width, 'h': height,
                    'conf': conf,
                    'real_x': real_x, 'real_y': real_y
                })

                # Add to PoseArray
                pose = Pose()
                pose.position.x = real_x
                pose.position.y = real_y
                pose.position.z = 0.0  # ground level
                pose_array.poses.append(pose)

        # Publish
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detections_pub.publish(det_msg)
        self.targets_pub.publish(pose_array)

        if detections:
            self.get_logger().info(f'Detected {len(detections)} flower(s)')


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

echo "✓ yolov8_detector.py written"

# ── 2. Register nodes in setup.py ────────────────────────────
echo ""
echo "[2/3] Updating setup.py to register both nodes..."

cat > ~/ros2_ws/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system',
    entry_points={
        'console_scripts': [
            'pollination_controller = precision_pollination.pollination_controller:main',
            'yolov8_detector = precision_pollination.yolov8_detector:main',
        ],
    },
)
SETUPEOF

echo "✓ setup.py updated with both nodes"

# ── 3. Write training script ──────────────────────────────────
echo ""
echo "[3/3] Writing YOLOv8 training script..."

mkdir -p ~/fyp_training

cat > ~/fyp_training/train_sunflower.py << 'TRAINEOF'
"""
YOLOv8 Sunflower Training Script
==================================
Run this script to train YOLOv8 on your sunflower dataset.

BEFORE RUNNING:
  1. Download a sunflower dataset from:
       https://universe.roboflow.com/search?q=sunflower
     Export in "YOLOv8" format. This gives you a folder with:
       data.yaml, train/, valid/, test/ subfolders

  2. Edit DATASET_PATH below to point to your data.yaml file

  3. Run: python3 train_sunflower.py

WHAT HAPPENS:
  - YOLOv8n (nano, fastest) trains for 100 epochs
  - Results saved to runs/detect/sunflower_v1/
  - Best model weights at: runs/detect/sunflower_v1/weights/best.pt
  - Copy best.pt to use in your ROS 2 node
"""

from ultralytics import YOLO
import os

# ── Configuration ──────────────────────────────────────────────
DATASET_PATH = os.path.expanduser('~/fyp_training/dataset/data.yaml')
MODEL_SIZE   = 'yolov8n.pt'   # n=nano (fastest), s=small, m=medium
EPOCHS       = 100
IMAGE_SIZE   = 640
BATCH_SIZE   = 8              # reduce to 4 if you run out of RAM
RUN_NAME     = 'sunflower_v1'

# ── Train ──────────────────────────────────────────────────────
print(f"\nStarting YOLOv8 training:")
print(f"  Dataset:    {DATASET_PATH}")
print(f"  Model:      {MODEL_SIZE}")
print(f"  Epochs:     {EPOCHS}")
print(f"  Image size: {IMAGE_SIZE}px")
print(f"  Batch size: {BATCH_SIZE}")
print("")

if not os.path.exists(DATASET_PATH):
    print("ERROR: Dataset not found at", DATASET_PATH)
    print("")
    print("Download a sunflower dataset from:")
    print("  https://universe.roboflow.com/search?q=sunflower+head")
    print("Export in YOLOv8 format and unzip into ~/fyp_training/dataset/")
    exit(1)

model = YOLO(MODEL_SIZE)

results = model.train(
    data=DATASET_PATH,
    epochs=EPOCHS,
    imgsz=IMAGE_SIZE,
    batch=BATCH_SIZE,
    name=RUN_NAME,
    patience=20,          # stop early if no improvement for 20 epochs
    save=True,
    plots=True,           # saves training curve graphs
    val=True
)

# ── Results ────────────────────────────────────────────────────
print("\n" + "="*50)
print("TRAINING COMPLETE")
print("="*50)
print(f"Best model:  runs/detect/{RUN_NAME}/weights/best.pt")
print(f"Results:     runs/detect/{RUN_NAME}/")
print("")
print("Evaluation metrics:")
metrics = model.val()
print(f"  mAP@0.5:       {metrics.box.map50:.3f}")
print(f"  mAP@0.5:0.95:  {metrics.box.map:.3f}")
print(f"  Precision:     {metrics.box.mp:.3f}")
print(f"  Recall:        {metrics.box.mr:.3f}")
print("")
print("To use in your ROS 2 node:")
print(f"  cp runs/detect/{RUN_NAME}/weights/best.pt ~/ros2_ws/")
print(f"  # Then in your launch file, set model_path:=~/ros2_ws/best.pt")
TRAINEOF

echo "✓ train_sunflower.py written at ~/fyp_training/"

# ── Rebuild workspace ─────────────────────────────────────────
echo ""
echo "Rebuilding ROS 2 workspace with new nodes..."
cd ~/ros2_ws
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -10
echo "✓ Workspace rebuilt"

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 3 COMPLETE                                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "TO RUN YOUR NODES (in separate terminals):"
echo ""
echo "  Terminal 1 — start PX4 simulation:"
echo "    cd ~/PX4-Autopilot && make px4_sitl gz_x500"
echo ""
echo "  Terminal 2 — start XRCE bridge:"
echo "    MicroXRCEAgent udp4 -p 8888"
echo ""
echo "  Terminal 3 — start camera+YOLO detector:"
echo "    ros2 run precision_pollination yolov8_detector"
echo ""
echo "  Terminal 4 — start state machine:"
echo "    ros2 run precision_pollination pollination_controller"
echo ""
echo "  Terminal 5 — check topics:"
echo "    ros2 topic list"
echo "    ros2 topic echo /sunflower_targets"
echo "    ros2 topic echo /pollination/log"
echo ""
echo "TO TRAIN YOUR MODEL:"
echo "  1. Download dataset: https://universe.roboflow.com/search?q=sunflower+head"
echo "  2. Unzip to ~/fyp_training/dataset/"
echo "  3. python3 ~/fyp_training/train_sunflower.py"
echo ""
