#!/usr/bin/env python3
"""
Gazebo Pose Bridge — reads Gazebo ground truth pose,
republishes as VehicleLocalPosition for each drone.
Bypasses broken IMU/GPS sensor plugins.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import subprocess
import threading

DRONE_MODELS = {0: 'x500_0', 1: 'x500_1', 2: 'x500_2'}

class GzPoseBridge(Node):
    def __init__(self):
        super().__init__('gz_pose_bridge')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pubs = {}
        for did, mname in DRONE_MODELS.items():
            self.pubs[mname] = self.create_publisher(
                VehicleLocalPosition,
                f'/px4_{did}/fmu/out/vehicle_local_position', px4_qos)
            self.get_logger().info(f'Bridge: {mname} → /px4_{did}/fmu/out/vehicle_local_position')
        self.poses = {m: None for m in DRONE_MODELS.values()}
        self._running = True
        threading.Thread(target=self._listen, daemon=True).start()
        self.create_timer(0.1, self._publish)
        self.get_logger().info('GzPoseBridge ready!')

    def _listen(self):
        proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/sunflower_field/dynamic_pose/info'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        name, pos = None, {}
        for line in proc.stdout:
            line = line.strip()
            if line.startswith('name: "'):
                name = line.split('"')[1]
                pos = {}
            elif name in DRONE_MODELS.values():
                if line.startswith('x:'):
                    try: pos['x'] = float(line.split(':')[1])
                    except: pass
                elif line.startswith('y:'):
                    try: pos['y'] = float(line.split(':')[1])
                    except: pass
                elif line.startswith('z:'):
                    try:
                        pos['z'] = float(line.split(':')[1])
                        if len(pos) == 3:
                            self.poses[name] = dict(pos)
                    except: pass
            if not self._running: break
        proc.terminate()

    def _publish(self):
        for mname, pose in self.poses.items():
            if not pose: continue
            msg = VehicleLocalPosition()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000
            msg.x = float(pose.get('x', 0.0))
            msg.y = float(pose.get('y', 0.0))
            msg.z = float(pose.get('z', 0.0))
            msg.xy_valid = True
            msg.z_valid = True
            self.pubs[mname].publish(msg)

    def destroy_node(self):
        self._running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GzPoseBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
