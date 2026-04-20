#!/usr/bin/env python3
"""
Pollination Controller Node — Week 9
======================================
State machine: IDLE → TAKEOFF → SEARCH → APPROACH → HOVER → POLLINATE → NEXT → LAND

Subscribes to /drone_N/sunflower_targets from flower_detector_sim.
Publishes trajectory setpoints to PX4 via XRCE bridge.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import json
import time
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
    DONE      = "DONE"


# Search pattern — waypoints per sector
SEARCH_PATTERNS = {
    0: [(2.0, 2.0), (2.0, 5.0), (2.0, 8.0)],   # Left
    1: [(5.0, 2.0), (5.0, 5.0), (5.0, 8.0)],   # Center
    2: [(8.0, 2.0), (8.0, 5.0), (8.0, 8.0)],   # Right
}

TAKEOFF_ALT  = -3.0   # NED frame (negative = up)
APPROACH_ALT = -2.0
HOVER_ALT    = -0.5
POSITION_TOL = 0.8    # metres — position tolerance
HOVER_TICKS  = 20     # 2 seconds at 10Hz


class PollinationController(Node):

    def __init__(self):
        super().__init__('pollination_controller')

        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ns = f'/px4_{self.drone_id}'

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position',
            self._position_callback,
            px4_qos
        )

        self.flower_sub = self.create_subscription(
            PoseArray,
            f'/drone_{self.drone_id}/sunflower_targets',
            self._flower_callback,
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint',
            px4_qos
        )

        self.log_pub = self.create_publisher(
            String,
            f'/drone_{self.drone_id}/pollination/log',
            10
        )

        self.state           = State.IDLE
        self.current_pos     = None
        self.flower_targets  = []
        self.pollination_log = []
        self.hover_count     = 0
        self.search_idx      = 0
        self.search_pattern  = SEARCH_PATTERNS[self.drone_id]
        self.cycle_start     = None

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'Drone {self.drone_id} pollination controller started. '
            f'State: {self.state.value}'
        )
        self.get_logger().info(
            f'States: IDLE → TAKEOFF → SEARCH → APPROACH → HOVER → POLLINATE → LAND'
        )

    def _position_callback(self, msg):
        self.current_pos = msg

    def _flower_callback(self, msg):
        self.flower_targets = list(msg.poses)
        if self.state == State.SEARCH and len(self.flower_targets) > 0:
            self.state = State.APPROACH
            self.cycle_start = time.time()
            self.get_logger().info(
                f'Drone {self.drone_id}: Flower detected! → APPROACH'
            )

    def _at_position(self, x, y, z, tol=None):
        if self.current_pos is None:
            return False
        tol = tol or POSITION_TOL
        dx = self.current_pos.x - x
        dy = self.current_pos.y - y
        dz = self.current_pos.z - z
        return (dx**2 + dy**2 + dz**2)**0.5 < tol

    def _control_loop(self):

        if self.state == State.IDLE:
            self.get_logger().info(
                f'Drone {self.drone_id}: IDLE → TAKEOFF'
            )
            self.state = State.TAKEOFF

        elif self.state == State.TAKEOFF:
            self._publish_setpoint(x=0.0, y=0.0, z=TAKEOFF_ALT)
            if self._at_position(0.0, 0.0, TAKEOFF_ALT):
                self.state = State.SEARCH
                self.get_logger().info(
                    f'Drone {self.drone_id}: Takeoff complete → SEARCH'
                )

        elif self.state == State.SEARCH:
            # Fly search pattern within sector
            if self.search_idx < len(self.search_pattern):
                wx, wy = self.search_pattern[self.search_idx]
                self._publish_setpoint(x=wx, y=wy, z=TAKEOFF_ALT)
                if self._at_position(wx, wy, TAKEOFF_ALT):
                    self.search_idx = (self.search_idx + 1) % len(self.search_pattern)
            else:
                self.search_idx = 0

        elif self.state == State.APPROACH:
            if self.flower_targets:
                t = self.flower_targets[0]
                self._publish_setpoint(
                    x=t.position.x,
                    y=t.position.y,
                    z=APPROACH_ALT
                )
                if self._at_position(t.position.x, t.position.y, APPROACH_ALT):
                    self.state = State.HOVER
                    self.hover_count = 0
                    self.get_logger().info(
                        f'Drone {self.drone_id}: At flower → HOVER'
                    )

        elif self.state == State.HOVER:
            if self.flower_targets:
                t = self.flower_targets[0]
                self._publish_setpoint(
                    x=t.position.x,
                    y=t.position.y,
                    z=HOVER_ALT
                )
                self.hover_count += 1
                if self.hover_count >= HOVER_TICKS:
                    self.state = State.POLLINATE

        elif self.state == State.POLLINATE:
            cycle_time = time.time() - self.cycle_start if self.cycle_start else 0
            flower = self.flower_targets[0] if self.flower_targets else None

            self.get_logger().info(
                f'Drone {self.drone_id}: ✅ POLLINATED flower! '
                f'Cycle time: {cycle_time:.1f}s '
                f'(target: <30s)'
            )

            entry = {
                'drone_id': self.drone_id,
                'flower_x': flower.position.x if flower else 0,
                'flower_y': flower.position.y if flower else 0,
                'cycle_time': round(cycle_time, 2),
                'count': len(self.pollination_log) + 1,
                'timestamp': time.time()
            }
            self.pollination_log.append(entry)

            log_msg = String()
            log_msg.data = json.dumps(entry)
            self.log_pub.publish(log_msg)

            self.state = State.NEXT

        elif self.state == State.NEXT:
            if self.flower_targets:
                self.flower_targets.pop(0)
            if self.flower_targets:
                self.state = State.APPROACH
                self.cycle_start = time.time()
            else:
                self.state = State.SEARCH
                self.get_logger().info(
                    f'Drone {self.drone_id}: No more targets → SEARCH'
                )

        elif self.state == State.LAND:
            self._publish_setpoint(x=0.0, y=0.0, z=0.0)
            self.get_logger().info(
                f'Drone {self.drone_id}: Mission complete. '
                f'Pollinated {len(self.pollination_log)} flowers.'
            )
            self.state = State.DONE

        elif self.state == State.DONE:
            pass

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        self.setpoint_pub.publish(msg)


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
