#!/usr/bin/env python3
"""
swarm_monitor.py — Stage 14

Passive dashboard. Subscribes to /swarm/peer and prints a status
table for both drones every 2 seconds. Makes no control decisions.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
        self.d = {0: {}, 1: {}}
        self.t0 = time.time()
        self.create_subscription(String, '/swarm/peer', self._cb, 10)
        self.create_timer(2.0, self._dash)
        self.get_logger().info('SWARM MONITOR (passive display)')

    def _cb(self, m):
        try:
            x = json.loads(m.data)
            did = x.get('drone_id')
            if did in (0, 1):
                self.d[did] = x
        except Exception:
            pass

    def _dash(self):
        t = int(time.time() - self.t0)
        lines = ['', f'====== SWARM STATUS  T+{t}s ======']
        done = 0
        for i in (0, 1):
            x = self.d.get(i, {})
            ph = x.get('phase', '-')
            claims = x.get('claimed', [])
            if x.get('x') is not None:
                pos = f"({x['x']:5.2f}, {x['y']:5.2f}, {x['z']:5.2f})"
            else:
                pos = "      pose unknown      "
            if ph == 'DONE':
                done += 1
            lines.append(f"  Drone {i}: {ph:18s} pos {pos} claims {claims}")
        lines.append(f"  completed: {done}/2")
        lines.append('==================================')
        for l in lines:
            self.get_logger().info(l)


def main(args=None):
    rclpy.init(args=args)
    n = Monitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
