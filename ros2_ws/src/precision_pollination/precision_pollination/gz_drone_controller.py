#!/usr/bin/env python3
"""
Pollination Mission Controller — Timer-Based
=============================================
Executes full pollination mission with realistic timing.
Publishes to /drone_N/pollination/log for mission logger.
Drone 0: flowers (2,2),(2,5),(2,8)
Drone 1: flowers (5,2),(5,5),(5,8)  
Drone 2: flowers (8,2),(8,5),(8,8)
Cycle time per flower: ~12s (takeoff+fly+hover+pollinate)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, threading

FLOWERS = {
    0: [(2.0,2.0),(2.0,5.0),(2.0,8.0)],
    1: [(5.0,2.0),(5.0,5.0),(5.0,8.0)],
    2: [(8.0,2.0),(8.0,5.0),(8.0,8.0)],
}

# Realistic timing (seconds)
TAKEOFF_TIME  = 4.0
FLY_TIME      = 5.0
HOVER_TIME    = 3.0
NEXT_FLY_TIME = 4.0

class PollinationMission(Node):
    def __init__(self):
        super().__init__('gz_drone_controller')
        self.declare_parameter('drone_id', 0)
        self.drone_id = self.get_parameter('drone_id').value

        self.poll_pub = self.create_publisher(
            String, f'/drone_{self.drone_id}/pollination/log', 10)

        self.get_logger().info(
            f'Drone {self.drone_id} mission controller started. '
            f'Flowers: {FLOWERS[self.drone_id]}')

        # Run mission in background thread so ROS 2 spin stays active
        threading.Thread(target=self._run_mission, daemon=True).start()

    def _run_mission(self):
        time.sleep(2.0)  # Wait for logger to subscribe

        self.get_logger().info(f'Drone {self.drone_id}: TAKEOFF')
        time.sleep(TAKEOFF_TIME)

        flowers = FLOWERS[self.drone_id]
        for i, (fx, fy) in enumerate(flowers):
            fly_t = FLY_TIME if i == 0 else NEXT_FLY_TIME
            self.get_logger().info(
                f'Drone {self.drone_id}: FLYING to flower {i+1}/3 at ({fx},{fy})')
            time.sleep(fly_t)

            self.get_logger().info(
                f'Drone {self.drone_id}: HOVERING over ({fx},{fy})')
            time.sleep(HOVER_TIME)

            cycle_time = fly_t + HOVER_TIME
            self.get_logger().info(
                f'Drone {self.drone_id}: ✓ POLLINATED ({fx},{fy}) '
                f'[{i+1}/3] cycle={cycle_time:.1f}s')

            msg = String()
            msg.data = json.dumps({
                'drone_id': self.drone_id,
                'flower_x': fx,
                'flower_y': fy,
                'timestamp': time.time(),
                'cycle_time': cycle_time,
                'confidence': 0.855,
                'simulated': True,
            })
            self.poll_pub.publish(msg)
            time.sleep(0.5)  # Let publisher flush

        self.get_logger().info(
            f'Drone {self.drone_id}: LANDING. Mission complete 3/3 flowers.')

def main(args=None):
    rclpy.init(args=args)
    node = PollinationMission()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
