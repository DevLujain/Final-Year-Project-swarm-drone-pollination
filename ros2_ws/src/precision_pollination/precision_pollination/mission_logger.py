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
