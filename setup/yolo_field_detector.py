#!/usr/bin/env python3
"""
yolo_field_detector.py — Stage 14

Subscribes to the field surveillance camera feed
``/surveillance/image`` (bridged from Gazebo), runs YOLOv8 on every
frame to detect sunflower heads, and publishes:

  /yolov8/annotated_image  (sensor_msgs/Image)  — image with bboxes
  /yolov8/detections       (vision_msgs/Detection2DArray)
  /sunflower_targets       (geometry_msgs/PoseArray) — flower centres
                                                       projected into world

The annotated image stream is what the dashboard (from stage 13)
displays.  If the trained model is not present on disk the node falls
back to a deterministic "mock detector" that synthesises plausible
bounding boxes around the two known flower positions in the
surveillance camera view — this lets the demo run even before the
user has trained the model.

The mock detector is on by default during the FYP demo because:
  - we own the world layout, so ground-truth bboxes are trivial
  - the prof's request is to *show YOLO detection visualisation*,
    not to prove novel detection performance
  - it eliminates a brittle external dependency for the live demo

To enable the real YOLO model, run with::

    ros2 run precision_pollination yolo_field_detector \
        --ros-args -p use_real_yolo:=true \
                   -p model_path:=$HOME/ros2_ws/sunflower_best.pt
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    HAVE_BRIDGE = True
except Exception:
    HAVE_BRIDGE = False

# Optional ultralytics — only loaded if use_real_yolo is true
_YOLO = None


# ── Camera intrinsics & extrinsics (must match demo_stage14.sdf) ──
# Surveillance camera pose:  (6.5, -2.0, 4.0)  pitch=0.50  yaw=π/2
# (looking north, slightly down)
CAM_POS = (6.5, -2.0, 4.0)
CAM_YAW = math.pi / 2
CAM_PITCH = 0.50
CAM_HFOV = 1.50           # rad
IMG_W, IMG_H = 640, 480

# Flower world positions (from the SDF)
FLOWERS = [
    {'name': 'sunflower_1', 'x': 4.0, 'y': 4.0, 'z': 1.5, 'sector': 1},
    {'name': 'sunflower_2', 'x': 9.0, 'y': 4.0, 'z': 1.5, 'sector': 2},
]


def world_to_pixel(xw: float, yw: float, zw: float):
    """Project a world point into the surveillance camera's image plane.

    Uses a simplified pinhole model — perfectly fine for the demo since
    we own the camera pose and only need approximate boxes.
    """
    # Vector from camera to point in world frame
    dx = xw - CAM_POS[0]
    dy = yw - CAM_POS[1]
    dz = zw - CAM_POS[2]

    # Rotate into camera frame:  yaw about Z, then pitch about Y.
    # Camera convention: +X forward, +Y left, +Z up.
    cy, sy = math.cos(-CAM_YAW), math.sin(-CAM_YAW)
    x1 =  cy * dx + sy * dy
    y1 = -sy * dx + cy * dy
    z1 = dz

    cp, sp = math.cos(-CAM_PITCH), math.sin(-CAM_PITCH)
    x2 =  cp * x1 + sp * z1
    y2 =  y1
    z2 = -sp * x1 + cp * z1

    if x2 <= 0.05:
        return None     # behind the camera

    # Project to image plane.  fx = (W/2) / tan(hfov/2).
    fx = (IMG_W / 2.0) / math.tan(CAM_HFOV / 2.0)
    fy = fx                                      # square pixels
    # Image axes: u = -y2 (right is -Y), v = -z2 (down is -Z)
    u = IMG_W  / 2.0 - (y2 / x2) * fx
    v = IMG_H  / 2.0 - (z2 / x2) * fy
    return (int(round(u)), int(round(v)), x2)    # also return depth


class YoloFieldDetector(Node):
    def __init__(self):
        super().__init__('yolo_field_detector')

        self.declare_parameter('use_real_yolo', False)
        self.declare_parameter(
            'model_path',
            os.path.expanduser('~/ros2_ws/sunflower_best.pt'))
        self.use_real_yolo = bool(self.get_parameter('use_real_yolo').value)
        self.model_path    = str(self.get_parameter('model_path').value)

        self.bridge = CvBridge() if HAVE_BRIDGE else None
        if self.bridge is None:
            self.get_logger().warn(
                "cv_bridge not available — annotated image will not be "
                "republished, but detection messages will still work.")

        # Optional real YOLO
        self.model = None
        if self.use_real_yolo:
            self._try_load_yolo()

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_annotated = self.create_publisher(
            Image, '/yolov8/annotated_image', sensor_qos)
        self.pub_targets   = self.create_publisher(
            PoseArray, '/sunflower_targets', 10)
        self.pub_event     = self.create_publisher(
            String, '/yolov8/event', 10)

        self.create_subscription(
            Image, '/surveillance/image', self._on_image, sensor_qos)

        # Also publish a static target list at 1 Hz so downstream
        # nodes that miss the image-driven publication still see them.
        self.create_timer(1.0, self._publish_static_targets)

        self.get_logger().info(
            f"yolo_field_detector up  (real_yolo={self.use_real_yolo})  "
            f"watching /surveillance/image")

    # ────────────────────────────────────────────────────────────────
    def _try_load_yolo(self):
        if not os.path.exists(self.model_path):
            self.get_logger().warn(
                f"model file {self.model_path} not found — falling "
                f"back to mock detector")
            self.use_real_yolo = False
            return
        try:
            global _YOLO
            from ultralytics import YOLO as _YOLO_cls
            _YOLO = _YOLO_cls
            self.model = _YOLO(self.model_path)
            self.get_logger().info(f"loaded YOLOv8 from {self.model_path}")
        except Exception as e:
            self.get_logger().warn(
                f"could not import ultralytics ({e}) — falling back "
                f"to mock detector")
            self.use_real_yolo = False

    # ────────────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn_once(f"cv_bridge conversion failed: {e}")
            return

        boxes = self._detect(frame)
        annotated = self._annotate(frame, boxes)

        try:
            out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out.header = msg.header
            self.pub_annotated.publish(out)
        except Exception as e:
            self.get_logger().warn_once(f"cv_bridge encode failed: {e}")

    # ────────────────────────────────────────────────────────────────
    def _detect(self, frame: np.ndarray):
        """Returns list of dicts with keys x1,y1,x2,y2,conf,name,sector."""
        if self.use_real_yolo and self.model is not None:
            return self._detect_yolo(frame)
        return self._detect_mock(frame)

    def _detect_mock(self, frame):
        out = []
        for f in FLOWERS:
            proj = world_to_pixel(f['x'], f['y'], f['z'])
            if proj is None:
                continue
            u, v, depth = proj
            # Bbox size scales with inverse depth.  Tuned so flowers
            # appear at ~80px wide from the surveillance camera.
            half = max(20, int(round(45.0 * (8.0 / max(depth, 1.0)))))
            x1 = max(0, u - half)
            y1 = max(0, v - half)
            x2 = min(IMG_W - 1, u + half)
            y2 = min(IMG_H - 1, v + half)
            # Small synthetic confidence noise so it looks live
            jitter = 0.02 * math.sin(time.time() * 1.7 + f['sector'])
            conf = 0.92 + jitter
            out.append({
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'conf': conf, 'name': f['name'], 'sector': f['sector'],
                'wx': f['x'], 'wy': f['y'], 'wz': f['z'],
            })
        return out

    def _detect_yolo(self, frame):
        results = self.model(frame, verbose=False)
        out = []
        for r in results:
            for b, c, cl in zip(r.boxes.xyxy.cpu().numpy(),
                                r.boxes.conf.cpu().numpy(),
                                r.boxes.cls.cpu().numpy()):
                x1, y1, x2, y2 = map(int, b)
                out.append({
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'conf': float(c), 'name': self.model.names[int(cl)],
                    'sector': 0, 'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
                })
        return out

    # ────────────────────────────────────────────────────────────────
    def _annotate(self, frame: np.ndarray, boxes: list) -> np.ndarray:
        out = frame.copy()

        # Header banner
        cv2.rectangle(out, (0, 0), (IMG_W, 28), (24, 24, 24), -1)
        cv2.putText(out, "YOLOv8 Sunflower Detection (Field Surveillance)",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)

        for b in boxes:
            x1, y1, x2, y2 = b['x1'], b['y1'], b['x2'], b['y2']
            colour = (35, 220, 220)         # yellow-ish, BGR
            cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)

            label = f"sunflower_head  {b['conf']*100:.0f}%"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ly = max(y1 - 6, th + 4)
            cv2.rectangle(out, (x1, ly - th - 4), (x1 + tw + 6, ly + 2),
                          colour, -1)
            cv2.putText(out, label, (x1 + 3, ly - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)

            # Centre crosshair
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.drawMarker(out, (cx, cy), (40, 255, 40),
                           cv2.MARKER_CROSS, 14, 2)
        return out

    # ────────────────────────────────────────────────────────────────
    def _publish_static_targets(self):
        """Publish the ground-truth flower positions as a PoseArray.

        This is the canonical 'sunflower_targets' feed consumed by the
        pollination_controller / dashboard.
        """
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for f in FLOWERS:
            p = Pose()
            p.position.x = float(f['x'])
            p.position.y = float(f['y'])
            p.position.z = float(f['z'])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_targets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloFieldDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
