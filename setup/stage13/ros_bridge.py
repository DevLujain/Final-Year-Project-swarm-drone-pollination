"""
ros_bridge.py  —  Stage 13 DaaS Dashboard
==========================================
Reads live ROS 2 topics in a background daemon thread.
Updates MissionState so Flask can serve live data to the browser.

When no swarm is running, rclpy.init() fails gracefully and the
dashboard falls back to the built-in simulation engine.

Topics read:
  /drone_N/flower_detections   JSON String — detection events
  /drone_N/pollination/log     JSON String — confirmed pollinations
  /swarm/tsp_summary           JSON String — TSP improvement %
  /swarm/readiness_status      JSON String — bloom filter %
"""

import json
import threading
import time

from mission_state import MissionState


class _ROSBridgeNode:
    """Wraps a rclpy Node subscribed to all relevant topics."""

    def __init__(self, state: MissionState, rclpy, Node, String):
        import rclpy as _rclpy
        _rclpy.init()

        # Create node via the passed-in class
        self._node  = Node('daas_bridge')
        self._state = state

        for drone_id in range(3):
            self._node.create_subscription(
                String,
                f'/drone_{drone_id}/flower_detections',
                lambda msg, d=drone_id: self._on_detection(msg, d),
                10,
            )
            self._node.create_subscription(
                String,
                f'/drone_{drone_id}/pollination/log',
                lambda msg, d=drone_id: self._on_pollination(msg, d),
                10,
            )

        self._node.create_subscription(
            String, '/swarm/tsp_summary',    self._on_tsp,   10)
        self._node.create_subscription(
            String, '/swarm/readiness_status', self._on_bloom, 10)

        state.connected_to_ros = True
        self._node.get_logger().info('DaaS bridge connected to all topics.')
        _rclpy.spin(self._node)

    def _on_detection(self, msg, drone_id):
        try:
            d = json.loads(msg.data)
            self._state.add_detection(
                float(d.get('flower_x', d.get('x', 0))),
                float(d.get('flower_y', d.get('y', 0))),
                float(d.get('confidence', d.get('conf', 0.855))),
            )
            self._state.update_drone_state(drone_id, 'APPROACH')
        except Exception:
            pass

    def _on_pollination(self, msg, drone_id):
        try:
            d = json.loads(msg.data)
            self._state.add_pollination_event(
                drone_id,
                float(d.get('flower_x', d.get('x', 0))),
                float(d.get('flower_y', d.get('y', 0))),
                d.get('flower_id', d.get('id', 'unknown')),
            )
            self._state.update_drone_state(drone_id, 'POLLINATE')
        except Exception:
            pass

    def _on_tsp(self, msg):
        try:
            d   = json.loads(msg.data)
            pct = d.get('overall_improvement_pct', d.get('improvement_pct'))
            if pct is not None:
                with self._state._lock:
                    self._state.tsp_pct = round(float(pct), 1)
        except Exception:
            pass

    def _on_bloom(self, msg):
        try:
            d   = json.loads(msg.data)
            pct = d.get('efficiency_gain_pct')
            if pct is not None:
                with self._state._lock:
                    self._state.bloom_pct = round(float(pct), 1)
        except Exception:
            pass


def start_ros_bridge(state: MissionState):
    """Entry point — called in a daemon thread by app.py."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        _ROSBridgeNode(state, rclpy, Node, String)
    except Exception as e:
        print(f'[ros_bridge] Offline mode ({e})')
        state.connected_to_ros = False
    finally:
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass
