#!/usr/bin/env python3
"""
Shared Visited List Node
=========================
Central ROS 2 node that tracks which flowers have been
pollinated by ANY drone in the swarm.

Each drone publishes to /swarm/flower_pollinated when it
completes a pollination. This node:
  1. Maintains a set of visited flower (x,y) positions
  2. Publishes the full visited list to /swarm/visited_flowers
  3. Any drone checks this list before approaching a flower
  4. Guarantees no flower is pollinated twice

Topic interface:
  SUB  /swarm/flower_pollinated  (std_msgs/String) — JSON {x, y, drone_id}
  PUB  /swarm/visited_flowers    (std_msgs/String) — JSON list of (x,y)
  PUB  /swarm/coverage_status    (std_msgs/String) — JSON coverage stats
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time

MATCH_RADIUS = 1.5  # Two flower reports match if within 1.5m of each other


class SharedVisitedList(Node):

    def __init__(self):
        super().__init__('shared_visited_list')

        self.visited = []       # list of (x, y) — pollinated flower positions
        self.total_flowers = 0  # set externally or via param
        self.start_time = time.time()

        # Subscribe to pollination events from all drones
        self.create_subscription(
            String,
            '/swarm/flower_pollinated',
            self._on_pollinated,
            10
        )

        # Publish visited list (drones query this)
        self.visited_pub = self.create_publisher(
            String,
            '/swarm/visited_flowers',
            10
        )

        # Publish coverage status every 5 seconds
        self.coverage_pub = self.create_publisher(
            String,
            '/swarm/coverage_status',
            10
        )

        self.create_timer(2.0, self._publish_visited)
        self.create_timer(5.0, self._publish_coverage)

        self.get_logger().info(
            'Shared visited list started. '
            'Listening on /swarm/flower_pollinated'
        )

    def _on_pollinated(self, msg):
        try:
            data = json.loads(msg.data)
            fx, fy = data['x'], data['y']
            drone_id = data.get('drone_id', '?')

            # Check if already visited (within match radius)
            for (vx, vy) in self.visited:
                dist = math.sqrt((fx - vx)**2 + (fy - vy)**2)
                if dist < MATCH_RADIUS:
                    self.get_logger().warn(
                        f'Drone {drone_id} tried to pollinate ({fx},{fy}) '
                        f'but already visited (nearest: ({vx},{vy}) dist={dist:.2f}m). '
                        f'SKIPPING.'
                    )
                    return

            # New flower — add to visited
            self.visited.append((fx, fy))
            self.get_logger().info(
                f'✓ Drone {drone_id} pollinated flower at ({fx},{fy}). '
                f'Total pollinated: {len(self.visited)}'
            )

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Bad pollination message: {e}')

    def _publish_visited(self):
        msg = String()
        msg.data = json.dumps(self.visited)
        self.visited_pub.publish(msg)

    def _publish_coverage(self):
        elapsed = time.time() - self.start_time
        msg = String()
        msg.data = json.dumps({
            'visited': len(self.visited),
            'elapsed_s': round(elapsed, 1),
            'positions': self.visited
        })
        self.coverage_pub.publish(msg)
        self.get_logger().info(
            f'=== COVERAGE STATUS === '
            f'Pollinated: {len(self.visited)} | '
            f'Elapsed: {elapsed:.0f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SharedVisitedList()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
