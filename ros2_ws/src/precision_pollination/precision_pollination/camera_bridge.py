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
