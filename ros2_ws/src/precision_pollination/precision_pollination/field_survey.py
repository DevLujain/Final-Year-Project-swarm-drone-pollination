#!/usr/bin/env python3
"""
field_survey.py — Stage 14

Per-drone "aerial scan" simulation. Reads the ground-truth flower
positions from /tmp/stage14_flowers.json (written by generate_field.py)
and publishes the full list on /drone_N/field_map once the drone is
at survey altitude (>= 2.5 m).

In a real deployment this would be YOLO on a downward camera, but
the SDF mock keeps the demo deterministic.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


JSONF = '/tmp/stage14_flowers.json'
SURVEY_MIN_ALT = 2.5
MODEL_MAP50 = 0.855


class FieldSurvey(Node):
    def __init__(self):
        super().__init__('field_survey')

        self.declare_parameter('drone_id', 0)
        self.did = int(self.get_parameter('drone_id').value)

        try:
            with open(JSONF) as f:
                self.flowers = json.load(f)['flowers']
            self.get_logger().info(
                f"[D{self.did}] survey sensor online "
                f"(YOLOv8s-OBB mock, mAP@0.5={MODEL_MAP50}). "
                f"Field has {len(self.flowers)} flowers. "
                f"Will publish when altitude >= {SURVEY_MIN_ALT} m."
            )
        except Exception as e:
            self.get_logger().error(
                f"[D{self.did}] failed to load {JSONF}: {e}"
            )
            self.flowers = []

        self.pose = None
        self.published = False

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            PoseStamped, f'/model/drone_{self.did}/pose',
            self._on_pose, sensor_qos
        )
        self.pub = self.create_publisher(
            String, f'/drone_{self.did}/field_map', 10
        )
        # Publish at 2 Hz; the controller only consumes the first one.
        self.create_timer(0.5, self._loop)

    def _on_pose(self, msg):
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _loop(self):
        if self.pose is None or not self.flowers:
            return
        alt = self.pose[2]
        if alt < SURVEY_MIN_ALT:
            return
        msg = String()
        msg.data = json.dumps({'flowers': self.flowers})
        self.pub.publish(msg)
        if not self.published:
            self.published = True
            self.get_logger().info(
                f"[D{self.did}] AERIAL SURVEY @ alt {alt:.1f} m "
                f"- detected {len(self.flowers)} flowers:"
            )
            for fl in self.flowers:
                self.get_logger().info(
                    f"   Flower {fl['id']} at ({fl['x']:.2f}, {fl['y']:.2f})"
                )


def main(args=None):
    rclpy.init(args=args)
    n = FieldSurvey()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
