#!/usr/bin/env python3
"""
Lawnmower Sweep Node - Stage 9
Simulates drone sweeping lawnmower pattern and detects flowers by proximity.
Uses simulated positions (no PX4 position topic dependency).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time

try:
    from .flower_positions import SECTOR_FLOWERS, ALL_FLOWERS
except ImportError:
    from flower_positions import SECTOR_FLOWERS, ALL_FLOWERS

DETECTION_RANGE = 3.0
TRACK_SPACING   = 3.0
CRUISE_ALT      = 5.0
MODEL_CONF      = 0.855

SECTOR_BOUNDS = {
    0: (0.0,  6.0),
    1: (6.0,  12.0),
    2: (12.0, 20.0),
}
Y_BOUNDS = (0.0, 25.0)


class LawnmowerSweep(Node):

    def __init__(self):
        super().__init__('lawnmower_sweep')
        self.declare_parameter('drone_id', 0)
        self.drone_id  = self.get_parameter('drone_id').value
        self.my_flowers = SECTOR_FLOWERS.get(self.drone_id, [])
        self.pollinated = set()
        self.visited    = []
        self.waypoints  = self._generate_lawnmower()
        self.wp_index   = 0
        self.drone_pos  = {'x': self.waypoints[0][0],
                           'y': self.waypoints[0][1],
                           'z': CRUISE_ALT}

        self.pollinated_pub = self.create_publisher(
            String, '/swarm/flower_pollinated', 10)

        self.create_subscription(
            String, '/swarm/visited_flowers',
            self._visited_cb, 10)

        # Move along waypoints every 1s, check proximity every 0.5s
        self.create_timer(1.0,  self._advance_waypoint)
        self.create_timer(0.5,  self._check_proximity)

        self.get_logger().info(
            f'Lawnmower sweep started for drone {self.drone_id}. '
            f'Sector: x={SECTOR_BOUNDS[self.drone_id]}. '
            f'Monitoring {len(self.my_flowers)} flowers. '
            f'Track spacing: {TRACK_SPACING}m. '
            f'Generated {len(self.waypoints)} waypoints.'
        )

    def _generate_lawnmower(self):
        x_min, x_max = SECTOR_BOUNDS[self.drone_id]
        y_min, y_max = Y_BOUNDS
        waypoints = []
        x = x_min + TRACK_SPACING / 2
        northbound = True
        while x <= x_max:
            if northbound:
                waypoints.append((x, y_min, CRUISE_ALT))
                waypoints.append((x, y_max, CRUISE_ALT))
            else:
                waypoints.append((x, y_max, CRUISE_ALT))
                waypoints.append((x, y_min, CRUISE_ALT))
            northbound = not northbound
            x += TRACK_SPACING
        return waypoints if waypoints else [(x_min, y_min, CRUISE_ALT)]

    def _advance_waypoint(self):
        """Move drone to next waypoint, interpolating position."""
        if not self.waypoints:
            return
        target = self.waypoints[self.wp_index % len(self.waypoints)]
        # Move drone position toward target
        cx, cy = self.drone_pos['x'], self.drone_pos['y']
        tx, ty = target[0], target[1]
        dist = math.sqrt((tx-cx)**2 + (ty-cy)**2)
        if dist < 1.0:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.wp_index = 0
        else:
            # Move 2m per second toward target
            step = min(2.0, dist)
            ratio = step / dist
            self.drone_pos['x'] = cx + (tx - cx) * ratio
            self.drone_pos['y'] = cy + (ty - cy) * ratio

    def _visited_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.visited = [(v[0], v[1]) for v in data]
        except Exception:
            pass

    def _is_visited(self, fx, fy):
        for (vx, vy) in self.visited:
            if math.sqrt((fx-vx)**2 + (fy-vy)**2) < 1.5:
                return True
        return False

    def _check_proximity(self):
        dx, dy = self.drone_pos['x'], self.drone_pos['y']
        for idx, (fx, fy) in enumerate(self.my_flowers):
            if idx in self.pollinated:
                continue
            if self._is_visited(fx, fy):
                self.pollinated.add(idx)
                continue
            dist = math.sqrt((dx-fx)**2 + (dy-fy)**2)
            if dist <= DETECTION_RANGE:
                self.pollinated.add(idx)
                msg = String()
                msg.data = json.dumps({
                    'x': fx, 'y': fy,
                    'drone_id': self.drone_id,
                    'confidence': MODEL_CONF,
                    'distance': round(dist, 2),
                    'timestamp': time.time()
                })
                self.pollinated_pub.publish(msg)
                self.get_logger().info(
                    f'Drone {self.drone_id}: ✓ POLLINATED flower {idx} '
                    f'at ({fx},{fy}) dist={dist:.2f}m. '
                    f'Total: {len(self.pollinated)}/{len(self.my_flowers)}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerSweep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
