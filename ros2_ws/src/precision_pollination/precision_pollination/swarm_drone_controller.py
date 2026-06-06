#!/usr/bin/env python3
"""
swarm_drone_controller.py — Stage 14 (Visual Servoing, no PX4)

One instance per drone. Two-drone peer-to-peer swarm pollination
using Gazebo's VelocityControl plugin (NOT PX4).

State machine:
    INIT  -> READY  -> TAKEOFF  -> SURVEY_GOTO  -> SURVEY  -> NEGOTIATE
                                                                    |
                                                                    v
    DONE <- LAND  <- RETURN  <- (P_GOTO -> P_DESCEND -> P_HOVER -> P_ASCEND)+

Coordination:
    Each drone has a random priority (in [0, 1)) set at startup.
    Both drones broadcast their position + priority + phase + claims
    on /swarm/peer. Once a drone has its field map and the peer's
    priority, it runs the SAME greedy closest-first assignment locally
    -> both reach identical assignments without a central authority.

Frame:
    The Gazebo VelocityControl plugin interprets cmd_vel in the
    WORLD frame. We never do body-frame transforms here.
    Yaw is left at 0 throughout the mission (we only send angular.z=0).
"""
from __future__ import annotations

import json
import math
import random
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool


# ── Geometry (must match generate_field.py and the SDF) ────────
SPAWN      = {0: (1.5, 1.0), 1: (11.5, 1.0)}
SURVEY_POS = {0: (4.5, 4.0), 1: (8.5, 4.0)}
CRUISE_Z      = 3.0     # cruise altitude (m)
POLLINATE_Z   = 2.0     # pollination hover altitude (m)
GROUND_Z      = 0.10    # landed altitude (m)

# ── Controller params ──────────────────────────────────────────
WP_TOL_XY   = 0.30      # horizontal arrival tolerance (m)
WP_TOL_Z    = 0.20      # vertical arrival tolerance (m)
HOVER_TICKS = 30        # 1.5 s at 20 Hz of pollination hover
CAP         = 3         # flowers per drone

# PID gains (P + simple D)
KP_XY = 1.0
KP_Z  = 1.0
KD_XY = 0.15
KD_Z  = 0.10
VMAX_XY = 1.2           # m/s
VMAX_Z  = 0.8           # m/s

CONTROL_HZ   = 20.0
BROADCAST_HZ = 10.0


def greedy_assign(drone_positions, flowers, priorities):
    """
    Greedy closest-first assignment.

    At each step, find the (drone, flower) pair with minimum distance.
    Assign that flower to that drone. Ties (within 1 mm) broken by
    priority: higher priority drone wins; then lower flower id.

    Capped at CAP flowers per drone. With 2 drones and 6 flowers
    this gives exactly 3 each (no duplicates).
    """
    assigned = {0: [], 1: []}
    remaining = list(flowers)
    while remaining:
        cands = []
        for d in (0, 1):
            if len(assigned[d]) >= CAP:
                continue
            px, py = drone_positions[d]
            for fl in remaining:
                dist = math.hypot(px - fl['x'], py - fl['y'])
                cands.append((dist, d, fl))
        if not cands:
            break
        min_d = min(c[0] for c in cands)
        tied = [c for c in cands if c[0] - min_d < 1e-3]
        # Higher priority wins on tie; then lowest flower id for determinism
        tied.sort(key=lambda c: (-priorities[c[1]], c[2]['id']))
        _, d, fl = tied[0]
        assigned[d].append(fl)
        remaining.remove(fl)
    return assigned


class Phase(Enum):
    INIT        = "INIT"
    READY       = "READY"
    TAKEOFF     = "TAKEOFF"
    SURVEY_GOTO = "SURVEY_GOTO"
    SURVEY      = "SURVEY"
    NEGOTIATE   = "NEGOTIATE"
    P_GOTO      = "POLLINATE_GOTO"
    P_DESCEND   = "POLLINATE_DESCEND"
    P_HOVER     = "POLLINATE_HOVER"
    P_ASCEND    = "POLLINATE_ASCEND"
    RETURN      = "RETURN"
    LAND        = "LAND"
    DONE        = "DONE"


class SwarmDrone(Node):
    def __init__(self):
        super().__init__('swarm_drone_controller')

        # Parameters
        self.declare_parameter('drone_id', 0)
        self.did = int(self.get_parameter('drone_id').value)
        if self.did not in (0, 1):
            raise ValueError(f"drone_id must be 0 or 1, got {self.did}")
        self.other = 1 - self.did
        self.spawn = SPAWN[self.did]
        self.survey_pos = SURVEY_POS[self.did]
        self.priority = random.random()

        # Mission state
        self.pose = None                # (x, y, z, yaw)
        self.prev_err = (0.0, 0.0, 0.0)
        self.prev_t = None
        self.phase = Phase.INIT
        self.armed = False              # True once /swarm/start received
        self.flowers = None             # list of {id, x, y, z}
        self.peer = {}                  # latest /swarm/peer broadcast
        self.assignment = None
        self.my_flowers = []
        self.fi = 0                     # index into self.my_flowers
        self.hov_t = 0                  # tick counter while hovering
        self.done_logged = False
        self.t0 = time.time()

        # ROS interfaces
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_cmd = self.create_publisher(
            Twist, f'/model/drone_{self.did}/cmd_vel', 10)
        self.pub_state = self.create_publisher(
            String, f'/pollination/drone_{self.did}/state', 10)
        self.pub_event = self.create_publisher(
            String, f'/pollination/drone_{self.did}/event', 10)
        self.pub_pose_relay = self.create_publisher(
            PoseStamped, f'/pollination/drone_{self.did}/pose', 10)
        self.peer_pub = self.create_publisher(
            String, '/swarm/peer', 10)

        self.create_subscription(
            PoseStamped, f'/model/drone_{self.did}/pose',
            self._on_pose, sensor_qos)
        self.create_subscription(
            Bool, '/swarm/start', self._on_start, 10)
        self.create_subscription(
            String, f'/drone_{self.did}/field_map',
            self._on_field_map, 10)
        self.create_subscription(
            String, '/swarm/peer', self._on_peer, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self.create_timer(1.0 / BROADCAST_HZ, self._broadcast)

        self.get_logger().info(
            f"[D{self.did}] online. spawn={self.spawn} "
            f"survey_pos={self.survey_pos} priority={self.priority:.3f}"
        )

    # ── Subscriptions ──────────────────────────────────────────
    def _on_pose(self, msg):
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        first = self.pose is None
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(yaw),
        )
        if first:
            self.get_logger().info(
                f"[D{self.did}] pose lock at "
                f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:.2f})"
            )
        # Relay for logger/monitor
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f'drone_{self.did}'
        out.pose = msg.pose
        self.pub_pose_relay.publish(out)

    def _on_start(self, msg):
        if msg.data and not self.armed:
            self.armed = True
            self.get_logger().info(f"[D{self.did}] /swarm/start received - GO")

    def _on_field_map(self, msg):
        if self.flowers is None:
            try:
                d = json.loads(msg.data)
                self.flowers = d['flowers']
                self.get_logger().info(
                    f"[D{self.did}] field map received - "
                    f"{len(self.flowers)} flowers"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"[D{self.did}] field map parse error: {e}"
                )

    def _on_peer(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('drone_id') == self.other:
                self.peer = d
        except Exception:
            pass

    # ── Helpers ────────────────────────────────────────────────
    def _go(self, new_phase, reason=""):
        text = f"[D{self.did}] {self.phase.value} -> {new_phase.value}"
        if reason:
            text += f"  ({reason})"
        self.get_logger().info(text)
        ev = String()
        ev.data = text
        self.pub_event.publish(ev)
        self.phase = new_phase
        # Reset PID derivative tracker on phase change
        self.prev_t = None
        self.prev_err = (0.0, 0.0, 0.0)

    def _broadcast(self):
        if self.pose is None:
            x = y = z = None
        else:
            x = round(self.pose[0], 2)
            y = round(self.pose[1], 2)
            z = round(self.pose[2], 2)
        m = String()
        m.data = json.dumps({
            'drone_id': self.did,
            'x': x, 'y': y, 'z': z,
            'priority': self.priority,
            'phase': self.phase.value,
            'claimed': [int(f['id']) for f in self.my_flowers],
        })
        self.peer_pub.publish(m)

    def _publish_state(self):
        m = String()
        m.data = self.phase.value
        self.pub_state.publish(m)

    def _peer_priority(self):
        return self.peer.get('priority')

    def _peer_phase(self):
        return self.peer.get('phase')

    # ── Control output ─────────────────────────────────────────
    def _send_velocity(self, vx, vy, vz, wz):
        """Publish a Twist with EVERY field cast to float."""
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.linear.z  = float(vz)
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(wz)
        self.pub_cmd.publish(t)

    def _stop(self):
        self._send_velocity(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def _clamp(v, vmax):
        return max(-vmax, min(vmax, v))

    def _goto(self, tx, ty, tz):
        """
        PD step toward world point (tx, ty, tz). Yaw is not controlled
        (angular.z = 0). Returns True when within position tolerance.
        """
        if self.pose is None:
            return False
        x, y, z, _yaw = self.pose
        ex = float(tx) - x
        ey = float(ty) - y
        ez = float(tz) - z

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            dex = dey = dez = 0.0
        else:
            dt = max(now - self.prev_t, 1e-3)
            dex = (ex - self.prev_err[0]) / dt
            dey = (ey - self.prev_err[1]) / dt
            dez = (ez - self.prev_err[2]) / dt
        self.prev_t = now
        self.prev_err = (ex, ey, ez)

        vx = self._clamp(KP_XY * ex + KD_XY * dex, VMAX_XY)
        vy = self._clamp(KP_XY * ey + KD_XY * dey, VMAX_XY)
        vz = self._clamp(KP_Z  * ez + KD_Z  * dez, VMAX_Z)

        # Pure world-frame velocity command — no body transform
        self._send_velocity(vx, vy, vz, 0.0)

        d_xy = math.hypot(ex, ey)
        d_z = abs(ez)
        return d_xy < WP_TOL_XY and d_z < WP_TOL_Z

    # ── State machine ──────────────────────────────────────────
    def _tick(self):
        self._publish_state()

        if self.pose is None:
            return

        if self.phase == Phase.INIT:
            self._stop()
            self._go(Phase.READY, "pose lock acquired")
            return

        if self.phase == Phase.READY:
            self._stop()
            if self.armed:
                self._go(Phase.TAKEOFF, "GO signal received")
            return

        if self.phase == Phase.TAKEOFF:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, CRUISE_Z)
            if arrived:
                self._go(Phase.SURVEY_GOTO, f"at cruise alt {CRUISE_Z} m")
            return

        if self.phase == Phase.SURVEY_GOTO:
            tx, ty = self.survey_pos
            arrived = self._goto(tx, ty, CRUISE_Z)
            if arrived:
                self._go(Phase.SURVEY, "at survey waypoint")
            return

        if self.phase == Phase.SURVEY:
            # Hold position and wait for own field map + peer priority
            tx, ty = self.survey_pos
            self._goto(tx, ty, CRUISE_Z)
            if self.flowers is not None and self._peer_priority() is not None:
                self._go(Phase.NEGOTIATE, "field+peer ready")
            return

        if self.phase == Phase.NEGOTIATE:
            tx, ty = self.survey_pos
            self._goto(tx, ty, CRUISE_Z)

            priorities = {
                self.did: self.priority,
                self.other: float(self._peer_priority()),
            }
            self.assignment = greedy_assign(
                SURVEY_POS, self.flowers, priorities
            )
            mine = self.assignment[self.did]
            # Sort nearest-first from my own survey waypoint
            mine.sort(key=lambda fl: math.hypot(
                self.survey_pos[0] - fl['x'],
                self.survey_pos[1] - fl['y']
            ))
            self.my_flowers = mine
            self.fi = 0

            ids_mine  = [int(f['id']) for f in mine]
            ids_other = [int(f['id']) for f in self.assignment[self.other]]
            self.get_logger().info(
                f"\n[D{self.did}] NEGOTIATION COMPLETE\n"
                f"   I claim flowers {ids_mine}\n"
                f"   peer takes      {ids_other}\n"
                f"   (greedy closest-first; ties broken by priority)"
            )
            self._go(Phase.P_GOTO, f"start pollinating my {len(mine)} flowers")
            return

        if self.phase == Phase.P_GOTO:
            if self.fi >= len(self.my_flowers):
                self._go(Phase.RETURN, "all my flowers pollinated")
                return
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], CRUISE_Z)
            if arrived:
                self._go(Phase.P_DESCEND, f"over flower {fl['id']}")
            return

        if self.phase == Phase.P_DESCEND:
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], POLLINATE_Z)
            if arrived:
                self.hov_t = 0
                self._go(Phase.P_HOVER, f"at {POLLINATE_Z} m over flower {fl['id']}")
            return

        if self.phase == Phase.P_HOVER:
            fl = self.my_flowers[self.fi]
            self._goto(fl['x'], fl['y'], POLLINATE_Z)
            self.hov_t += 1
            if self.hov_t == 1:
                ev = String()
                ev.data = (f"[D{self.did}] *** POLLINATING flower {fl['id']} "
                           f"at ({fl['x']:.2f}, {fl['y']:.2f}) ***")
                self.get_logger().info(ev.data)
                self.pub_event.publish(ev)
            if self.hov_t >= HOVER_TICKS:
                self._go(Phase.P_ASCEND, f"flower {fl['id']} pollinated")
            return

        if self.phase == Phase.P_ASCEND:
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], CRUISE_Z)
            if arrived:
                self.fi += 1
                self._go(Phase.P_GOTO,
                         f"next ({self.fi}/{len(self.my_flowers)})")
            return

        if self.phase == Phase.RETURN:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, CRUISE_Z)
            if arrived:
                self._go(Phase.LAND, "over home pad")
            return

        if self.phase == Phase.LAND:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, GROUND_Z)
            if arrived:
                self._stop()
                self._go(Phase.DONE, "landed safely")
            return

        if self.phase == Phase.DONE:
            self._stop()
            if not self.done_logged:
                self.get_logger().info(
                    f"[D{self.did}] MISSION COMPLETE - pollinated "
                    f"{len(self.my_flowers)} flowers, landed at {self.spawn}"
                )
                self.done_logged = True
            return


def main(args=None):
    rclpy.init(args=args)
    node = SwarmDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure the drone stops on exit (all explicit floats)
        try:
            stop = Twist()
            stop.linear.x = 0.0
            stop.linear.y = 0.0
            stop.linear.z = 0.0
            stop.angular.x = 0.0
            stop.angular.y = 0.0
            stop.angular.z = 0.0
            node.pub_cmd.publish(stop)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
