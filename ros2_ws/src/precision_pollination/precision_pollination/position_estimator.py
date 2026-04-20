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
