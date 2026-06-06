#!/usr/bin/env python3
"""
swarm_coordinator.py — Stage 14

Mission coordinator for the two-drone visual-servoing demo.

Responsibilities
----------------
1. After a short warm-up period (so Gazebo, the bridge, and the
   controllers all have time to subscribe), publish ``True`` once on
   ``/pollination/start`` to release both visual_servoing_controller
   nodes.
2. Subscribe to ``/pollination/drone_0/state`` and
   ``/pollination/drone_1/state``.  When both report ``DONE``, publish
   a mission-complete event on ``/pollination/swarm/event``.
3. Publish an aggregated swarm status string on
   ``/pollination/swarm/status`` at 2 Hz for the dashboard.

This node intentionally has no per-drone control logic — it only
orchestrates the swarm-level handshake.  The "cooperative" element of
the swarm is the synchronised start and the global progress monitor;
sector assignment is handled at launch time by the per-drone
parameter file.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


WARMUP_SEC = 4.0    # wait this long after the node comes up before
                    # firing the global start signal


class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        self.declare_parameter('n_drones', 2)
        self.n_drones = int(self.get_parameter('n_drones').value)

        self.states = {i: 'UNKNOWN' for i in range(self.n_drones)}
        self.started = False
        self.completed = False

        self.pub_start  = self.create_publisher(Bool,   '/pollination/start', 10)
        self.pub_event  = self.create_publisher(String, '/pollination/swarm/event', 10)
        self.pub_status = self.create_publisher(String, '/pollination/swarm/status', 10)

        for i in range(self.n_drones):
            self.create_subscription(
                String,
                f'/pollination/drone_{i}/state',
                lambda msg, idx=i: self._on_state(idx, msg),
                10,
            )

        # Timer used to fire the start signal after WARMUP_SEC
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self._on_tick)

        self.get_logger().info(
            f"swarm coordinator up — will start {self.n_drones} drones "
            f"in {WARMUP_SEC:.0f}s")

    def _on_state(self, drone_id: int, msg: String):
        self.states[drone_id] = msg.data

    def _on_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.started and now - self.t0 >= WARMUP_SEC:
            self._fire_start()
            self.started = True

        # Publish aggregated status
        m = String()
        m.data = ' | '.join(f"D{i}:{self.states[i]}" for i in range(self.n_drones))
        self.pub_status.publish(m)

        # Mission complete?
        if (self.started and not self.completed
                and all(s == 'DONE' for s in self.states.values())):
            self.completed = True
            ev = String()
            ev.data = "★ SWARM MISSION COMPLETE — both drones landed safely ★"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)

    def _fire_start(self):
        m = Bool()
        m.data = True
        # Publish a few times so we don't lose it to subscriber-not-ready
        for _ in range(5):
            self.pub_start.publish(m)
        ev = String()
        ev.data = "swarm GO — both drones armed and launching"
        self.pub_event.publish(ev)
        self.get_logger().info(ev.data)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
