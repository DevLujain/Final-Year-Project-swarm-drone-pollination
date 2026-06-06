#!/usr/bin/env python3
"""
swarm_coordinator.py — Stage 14

Fires the global GO signal on /swarm/start after a warmup period
(so Gazebo + bridge + per-drone nodes are all ready). Then monitors
both drones' phases via /swarm/peer and announces mission complete
when both reach DONE.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


WARMUP_SEC = 5.0    # seconds to wait before firing /swarm/start


class Coordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        self.started = False
        self.completed = False
        self.phases = {0: 'UNKNOWN', 1: 'UNKNOWN'}

        self.pub_start = self.create_publisher(Bool,   '/swarm/start',            10)
        self.pub_event = self.create_publisher(String, '/pollination/swarm/event', 10)

        self.create_subscription(String, '/swarm/peer', self._on_peer, 10)

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"swarm coordinator online - "
            f"will fire /swarm/start in {WARMUP_SEC:.0f} s"
        )

    def _on_peer(self, msg):
        try:
            d = json.loads(msg.data)
            did = d.get('drone_id')
            if did in (0, 1):
                self.phases[did] = d.get('phase', 'UNKNOWN')
        except Exception:
            pass

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.started and now - self.t0 >= WARMUP_SEC:
            # Publish a few times so we don't lose it to subscriber-not-ready
            m = Bool()
            m.data = True
            for _ in range(5):
                self.pub_start.publish(m)
            self.started = True
            ev = String()
            ev.data = "*** SWARM GO - both drones launching together ***"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)

        if (self.started and not self.completed
                and self.phases[0] == 'DONE'
                and self.phases[1] == 'DONE'):
            self.completed = True
            ev = String()
            ev.data = "*** SWARM MISSION COMPLETE - both drones landed safely ***"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)


def main(args=None):
    rclpy.init(args=args)
    n = Coordinator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
