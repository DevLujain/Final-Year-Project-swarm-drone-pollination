"""
ros_bridge.py — Stage 16 v13
Subscribes to ROS 2 topics, pushes into MissionState.
"""
from __future__ import annotations
import json, os, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32, String

N_DRONES = int(os.environ.get('STAGE16_N_DRONES', 2))


class Stage16Bridge(Node):

    def __init__(self, state):
        super().__init__('stage16_dashboard_bridge')
        self.state = state

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        sensor  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Int32,  '/bloom/current_day', self._on_day, latched)
        self.create_subscription(String, '/swarm/event',       self._on_event, 10)

        for i in range(N_DRONES):
            self.create_subscription(String,
                f'/survey/drone_{i}/observations',
                lambda m, d=i: self._on_obs(d, m), 10)
            self.create_subscription(Int32,
                f'/pollination/drone_{i}/pollinated',
                lambda m, d=i: self._on_pollin(d, m), 10)
            self.create_subscription(String,
                f'/pollination/drone_{i}/state',
                lambda m, d=i: self._on_state(d, m), 10)
            self.create_subscription(PoseStamped,
                f'/pollination/drone_{i}/pose',
                lambda m, d=i: self._on_pose(d, m), sensor)
            self.create_subscription(Float32,
                f'/battery/drone_{i}',
                lambda m, d=i: self._on_batt(d, m), 10)

        self.state.set_connected(True)
        self.get_logger().info(
            f'stage16_bridge v14-buds up — {N_DRONES} drones')

    def _on_day(self, m: Int32):
        self.state.set_current_day(int(m.data))

    def _on_event(self, m: String):
        try:
            env = json.loads(m.data)
        except Exception:
            return
        etype    = env.get('type', '')
        drone_id = env.get('drone_id')
        payload  = env.get('payload', {})
        self.state.add_event(etype, drone_id, payload)

        if etype == 'day_start':
            day = payload.get('day', env.get('day'))
            if day is not None:
                self.state.set_current_day(int(day))

        elif etype == 'pollination_success':
            # Use the flower_id from the event (registry id, reliable)
            fid = payload.get('flower_id')
            x   = payload.get('x')
            y   = payload.get('y')
            if fid is not None:
                self.state.mark_pollinated(int(fid), x, y)

    def _on_obs(self, did: int, m: String):
        try:
            data = json.loads(m.data)
        except Exception:
            return
        day = self.state.current_day
        for o in data.get('observations', []):
            try:
                self.state.add_observation(
                    did,
                    int(o['flower_id']),
                    float(o['x']),
                    float(o['y']),
                    day,
                    state=o.get('state', 'open'),
                )
            except (KeyError, ValueError, TypeError):
                continue
        # v13: bud census — closed flowers the drone memorized.
        for b in data.get('bud_observations', []):
            try:
                self.state.add_observation(
                    did,
                    int(b.get('bud_id', b.get('flower_id', 0))),
                    float(b['x']),
                    float(b['y']),
                    day,
                    state='closed',
                )
            except (KeyError, ValueError, TypeError):
                continue

    def _on_pollin(self, did: int, m: Int32):
        # Handled via pollination_success event (has coordinates + fid)
        pass

    def _on_state(self, did: int, m: String):
        self.state.update_drone_state(did, str(m.data))

    def _on_pose(self, did: int, m: PoseStamped):
        self.state.update_drone_pose(
            did, m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def _on_batt(self, did: int, m: Float32):
        self.state.update_battery(did, float(m.data))


def _spin(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f'[ros_bridge] spin error: {e}')


def start(state):
    rclpy.init(args=None)
    node = Stage16Bridge(state)
    t = threading.Thread(target=_spin, args=(node,), daemon=True, name='bridge_spin')
    t.start()
    return node


def stop(node):
    try: node.destroy_node()
    except Exception: pass
    try: rclpy.shutdown()
    except Exception: pass
