#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
import json

SECTORS = {
    0: {'x_min': 0,  'x_max': 7,  'y_min': 0, 'y_max': 20, 'name': 'LEFT'},
    1: {'x_min': 7,  'x_max': 13, 'y_min': 0, 'y_max': 20, 'name': 'CENTER'},
    2: {'x_min': 13, 'x_max': 20, 'y_min': 0, 'y_max': 20, 'name': 'RIGHT'},
}

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.drone_positions = {i: None for i in range(3)}
        self.pollinated_flowers = set()
        for i in range(3):
            self.create_subscription(VehicleLocalPosition,
                f'/px4_{i}/fmu/out/vehicle_local_position',
                lambda msg, d=i: self._pos_cb(msg, d), px4_qos)
            self.create_subscription(String,
                f'/drone_{i}/pollination/log',
                self._poll_cb, 10)
        self.sector_pub = self.create_publisher(String, '/swarm/sector_map', 10)
        self.create_timer(5.0, self._status)
        self._publish_sectors()
        self.get_logger().info('Swarm coordinator started!')
        self.get_logger().info('Field divided into 3 sectors:')
        for d, s in SECTORS.items():
            self.get_logger().info(f'  Drone {d} → {s["name"]} sector (x: {s["x_min"]}-{s["x_max"]}m, y: {s["y_min"]}-{s["y_max"]}m)')

    def _pos_cb(self, msg, drone_id):
        self.drone_positions[drone_id] = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    def _poll_cb(self, msg):
        try:
            data = json.loads(msg.data)
            key = f'{data.get("flower_x")},{data.get("flower_y")}'
            if key not in self.pollinated_flowers:
                self.pollinated_flowers.add(key)
                self.get_logger().info(f'Flower pollinated! Total: {len(self.pollinated_flowers)}')
        except: pass

    def _publish_sectors(self):
        msg = String()
        msg.data = json.dumps(SECTORS)
        self.sector_pub.publish(msg)

    def _status(self):
        self._publish_sectors()
        self.get_logger().info('=== SWARM STATUS ===')
        for i in range(3):
            pos = self.drone_positions[i]
            if pos:
                s = SECTORS[i]
                in_s = s['x_min'] <= pos['x'] <= s['x_max'] and s['y_min'] <= pos['y'] <= s['y_max']
                self.get_logger().info(f'  Drone {i}: x={pos["x"]:.1f} y={pos["y"]:.1f} z={pos["z"]:.1f} | In sector: {in_s}')
            else:
                self.get_logger().info(f'  Drone {i}: No position data yet')
        self.get_logger().info(f'  Total flowers pollinated: {len(self.pollinated_flowers)}')

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
