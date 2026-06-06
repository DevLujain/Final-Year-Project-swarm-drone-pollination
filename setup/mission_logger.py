#!/usr/bin/env python3
"""
mission_logger.py — Stage 14

Logs the visual-servoing demo to a CSV file under
``~/Desktop/FYP/mission_outputs/stage14/`` for use in the FYP report.

Two files are written:

  stage14_trace.csv   — wide rows, 10 Hz: timestamp, D0_x, D0_y, D0_z,
                        D0_state, D1_x, D1_y, D1_z, D1_state
  stage14_events.csv  — sparse rows, one per event: timestamp, src, msg

Run alongside swarm_coordinator + the two visual_servoing_controllers.
"""

from __future__ import annotations

import csv
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


OUT_DIR = Path.home() / 'Desktop' / 'FYP' / 'mission_outputs' / 'stage14'


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.trace_path  = OUT_DIR / f'stage14_trace_{ts}.csv'
        self.events_path = OUT_DIR / f'stage14_events_{ts}.csv'

        self.trace_f = open(self.trace_path, 'w', newline='')
        self.trace_w = csv.writer(self.trace_f)
        self.trace_w.writerow([
            'time', 'D0_x', 'D0_y', 'D0_z', 'D0_state',
                    'D1_x', 'D1_y', 'D1_z', 'D1_state'])

        self.events_f = open(self.events_path, 'w', newline='')
        self.events_w = csv.writer(self.events_f)
        self.events_w.writerow(['time', 'source', 'event'])

        self.poses  = {0: (0.0, 0.0, 0.0), 1: (0.0, 0.0, 0.0)}
        self.states = {0: 'UNKNOWN', 1: 'UNKNOWN'}

        for i in (0, 1):
            self.create_subscription(
                PoseStamped, f'/pollination/drone_{i}/pose',
                lambda m, idx=i: self._on_pose(idx, m), 10)
            self.create_subscription(
                String, f'/pollination/drone_{i}/state',
                lambda m, idx=i: self._on_state(idx, m), 10)
            self.create_subscription(
                String, f'/pollination/drone_{i}/event',
                lambda m, idx=i: self._on_event(f'D{idx}', m), 10)

        self.create_subscription(
            String, '/pollination/swarm/event',
            lambda m: self._on_event('SWARM', m), 10)
        self.create_subscription(
            String, '/yolov8/event',
            lambda m: self._on_event('YOLO', m), 10)

        # 10 Hz trace
        self.create_timer(0.1, self._flush_trace)

        self.get_logger().info(f"writing trace to {self.trace_path}")
        self.get_logger().info(f"writing events to {self.events_path}")

    # ────────────────────────────────────────────────────────────────
    def _on_pose(self, drone_id: int, msg: PoseStamped):
        self.poses[drone_id] = (msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z)

    def _on_state(self, drone_id: int, msg: String):
        self.states[drone_id] = msg.data

    def _on_event(self, src: str, msg: String):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.events_w.writerow([f'{now:.3f}', src, msg.data])
        self.events_f.flush()

    def _flush_trace(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        p0 = self.poses[0]; p1 = self.poses[1]
        self.trace_w.writerow([f'{now:.3f}',
                               f'{p0[0]:.3f}', f'{p0[1]:.3f}', f'{p0[2]:.3f}',
                               self.states[0],
                               f'{p1[0]:.3f}', f'{p1[1]:.3f}', f'{p1[2]:.3f}',
                               self.states[1]])
        self.trace_f.flush()

    def destroy_node(self):
        try:
            self.trace_f.close()
            self.events_f.close()
        except Exception:
            pass
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
