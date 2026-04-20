#!/usr/bin/env python3
"""
Simulated Flower Detector Node
================================
Triggers YOLOv8-equivalent detections when a drone flies
within detection_range metres of a known flower position.

Academic justification:
  Flower positions are known a priori from the Gazebo world
  file (sunflower_field.sdf). Detection confidence scores
  are drawn from the trained YOLOv8s-OBB model result
  (mAP@0.5 = 0.855, precision = 0.807, recall = 0.784).
  This approach is consistent with simulation-first
  validation methodology (Sharma et al., 2024;
  Weng et al., 2025).

Sector assignment:
  Drone 0 → LEFT   (x: 0-7m)   flowers: (2,2),(2,5),(2,8)
  Drone 1 → CENTER (x: 7-13m)  flowers: (5,2),(5,5),(5,8)
  Drone 2 → RIGHT  (x: 13-20m) flowers: (8,2),(8,5),(8,8)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
import json
import math
import time

# Known flower positions from sunflower_field.sdf (x, y)
ALL_FLOWERS = [
    (2.0, 2.0), (2.0, 5.0), (2.0, 8.0),   # Left sector
    (5.0, 2.0), (5.0, 5.0), (5.0, 8.0),   # Center sector
    (8.0, 2.0), (8.0, 5.0), (8.0, 8.0),   # Right sector
]

# Sector flower assignment per drone
SECTOR_FLOWERS = {
    0: [(2.0, 2.0), (2.0, 5.0), (2.0, 8.0)],   # Drone 0 - LEFT
    1: [(5.0, 2.0), (5.0, 5.0), (5.0, 8.0)],   # Drone 1 - CENTER
    2: [(8.0, 2.0), (8.0, 5.0), (8.0, 8.0)],   # Drone 2 - RIGHT
}

# YOLOv8s-OBB trained model confidence (mAP@0.5 = 0.855)
MODEL_CONFIDENCE = 0.855
DETECTION_RANGE  = 3.0   # metres — drone must be within this range
CHECK_RATE       = 2.0   # Hz — how often to check proximity


class FlowerDetectorSim(Node):

    def __init__(self):
        super().__init__('flower_detector_sim')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.drone_pos   = None
        self.altitude    = 0.0
        self.my_flowers  = SECTOR_FLOWERS[self.drone_id]
        self.detected    = set()   # already-detected flower indices

        # Subscribe to drone position
        self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{self.drone_id}/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        # Publish flower targets (to pollination controller)
        self.targets_pub = self.create_publisher(
            PoseArray,
            f'/drone_{self.drone_id}/sunflower_targets',
            10
        )

        # Publish detection log (to mission logger)
        self.det_log_pub = self.create_publisher(
            String,
            f'/drone_{self.drone_id}/yolov8/detections',
            10
        )

        # Proximity check timer
        self.create_timer(1.0 / CHECK_RATE, self._check_proximity)

        self.get_logger().info(
            f'Flower detector (sim) started for drone {self.drone_id}. '
            f'Monitoring {len(self.my_flowers)} flowers in sector. '
            f'Detection range: {DETECTION_RANGE}m. '
            f'Model confidence: {MODEL_CONFIDENCE} (YOLOv8s-OBB mAP@0.5=0.855)'
        )

    def _position_callback(self, msg):
        self.drone_pos = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        self.altitude  = abs(float(msg.z))

    def _check_proximity(self):
        if self.drone_pos is None:
            return

        dx_pos = self.drone_pos['x']
        dy_pos = self.drone_pos['y']

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        detections = []

        for idx, (fx, fy) in enumerate(self.my_flowers):
            dist = math.sqrt((dx_pos - fx)**2 + (dy_pos - fy)**2)

            if dist <= DETECTION_RANGE:
                pose = Pose()
                pose.position.x = fx
                pose.position.y = fy
                pose.position.z = 0.0
                pose_array.poses.append(pose)

                detections.append({
                    'flower_id': idx,
                    'flower_x': fx,
                    'flower_y': fy,
                    'distance': round(dist, 2),
                    'conf': MODEL_CONFIDENCE,
                    'px': 320.0,
                    'py': 240.0,
                    'real_x': fx,
                    'real_y': fy,
                    'simulated': True
                })

                if idx not in self.detected:
                    self.detected.add(idx)
                    self.get_logger().info(
                        f'Drone {self.drone_id}: DETECTED flower {idx} '
                        f'at ({fx},{fy}) — dist={dist:.1f}m '
                        f'conf={MODEL_CONFIDENCE}'
                    )

        if pose_array.poses:
            self.targets_pub.publish(pose_array)

        if detections:
            det_msg = String()
            det_msg.data = json.dumps(detections)
            self.det_log_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlowerDetectorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
