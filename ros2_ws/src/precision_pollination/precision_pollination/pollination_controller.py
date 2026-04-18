#!/usr/bin/env python3
"""
Pollination Controller Node
===========================
State machine for autonomous drone pollination.

States: IDLE -> TAKEOFF -> SEARCH -> APPROACH -> HOVER -> POLLINATE -> NEXT -> LAND

This node:
  - Subscribes to /sunflower_targets (flower positions from YOLOv8)
  - Subscribes to /fmu/out/vehicle_local_position (drone GPS/position from PX4)
  - Publishes to /fmu/in/trajectory_setpoint (movement commands to PX4)
  - Publishes to /pollination/log (record of which flowers were visited)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import json
from enum import Enum


class State(Enum):
    IDLE      = "IDLE"
    TAKEOFF   = "TAKEOFF"
    SEARCH    = "SEARCH"
    APPROACH  = "APPROACH"
    HOVER     = "HOVER"
    POLLINATE = "POLLINATE"
    NEXT      = "NEXT_TARGET"
    LAND      = "LAND"


class PollinationController(Node):
    """Main state machine node for drone pollination."""

    def __init__(self):
        super().__init__('pollination_controller')

        # PX4 requires a specific QoS profile for its topics
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to PX4 position output
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        # Subscribe to YOLOv8 flower detections
        self.flower_sub = self.create_subscription(
            PoseArray,
            '/sunflower_targets',
            self._flower_callback,
            10
        )

        # Publish movement commands to PX4
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos
        )

        # Publish pollination log
        self.log_pub = self.create_publisher(String, '/pollination/log', 10)

        # State
        self.state = State.IDLE
        self.current_pos = None
        self.flower_targets = []
        self.pollination_log = []

        # Control loop — runs 10 times per second
        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(f'Pollination controller started. State: {self.state.value}')

    def _position_callback(self, msg):
        self.current_pos = msg

    def _flower_callback(self, msg):
        self.flower_targets = msg.poses
        if self.state == State.SEARCH and len(self.flower_targets) > 0:
            self.state = State.APPROACH
            self.get_logger().info(f'Flower detected! Switching to APPROACH')

    def _control_loop(self):
        """Called 10x/second — checks state and acts."""
        self.get_logger().debug(f'State: {self.state.value}')

        if self.state == State.IDLE:
            pass  # Waiting for start_mission() call

        elif self.state == State.TAKEOFF:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)  # 3m altitude (NED: negative z = up)

        elif self.state == State.SEARCH:
            self._publish_setpoint(x=0.0, y=0.0, z=-3.0)  # hover while scanning

        elif self.state == State.APPROACH:
            if self.flower_targets:
                target = self.flower_targets[0]
                self._publish_setpoint(x=target.position.x, y=target.position.y, z=-0.5)

        elif self.state == State.HOVER:
            if self.flower_targets:
                target = self.flower_targets[0]
                self._publish_setpoint(x=target.position.x, y=target.position.y, z=-0.3)

        elif self.state == State.POLLINATE:
            self.get_logger().info('Simulating pollination event...')
            self.pollination_log.append({'target': str(self.flower_targets[0]) if self.flower_targets else 'unknown'})
            log_msg = String()
            log_msg.data = json.dumps(self.pollination_log[-1])
            self.log_pub.publish(log_msg)
            self.state = State.NEXT

        elif self.state == State.NEXT:
            if self.flower_targets:
                self.flower_targets.pop(0)
            self.state = State.SEARCH if self.flower_targets else State.LAND

        elif self.state == State.LAND:
            self._publish_setpoint(x=0.0, y=0.0, z=0.0)
            self.get_logger().info(f'Mission complete. Pollinated: {len(self.pollination_log)} flowers')

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.setpoint_pub.publish(msg)

    def start_mission(self):
        if self.state == State.IDLE:
            self.state = State.TAKEOFF
            self.get_logger().info('Mission started!')


def main(args=None):
    rclpy.init(args=args)
    node = PollinationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
