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
