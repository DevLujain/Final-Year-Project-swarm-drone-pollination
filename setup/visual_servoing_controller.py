#!/usr/bin/env python3
"""
visual_servoing_controller.py — Stage 14

One instance per drone (parameter ``drone_id``).  Implements the 4-phase
visual-servoing approach from Hulens et al. (2022), Figure 10, plus
take-off and return-to-base phases so the drone completes a full
mission and lands back at its launch pad.

State machine:

    READY ──► TAKEOFF ──► APPROACH ──► LOWER ──► ENCIRCLE
      ▲                                              │
      │                                              ▼
    DONE ◄── LAND ◄── RETURN ◄── RETREAT ◄── POLLINATE

Each phase produces a target (x, y, z, yaw) for the drone.  A
proportional-derivative (PD) controller on position generates a
``geometry_msgs/Twist`` published to ``/model/drone_N/cmd_vel`` at
20 Hz, which Gazebo's velocity-control plugin applies as a body
velocity command.

Position feedback comes from ``/model/drone_N/pose`` (bridged from
Gazebo's pose publisher plugin).

For dashboard / logger compatibility the node also publishes:
  /pollination/drone_N/state   (std_msgs/String  — current phase name)
  /pollination/drone_N/pose    (geometry_msgs/PoseStamped)
  /pollination/drone_N/target  (geometry_msgs/PoseStamped — current setpoint)
  /pollination/drone_N/event   (std_msgs/String  — phase-change events)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool


# ── Phase definitions ──────────────────────────────────────────────
READY, TAKEOFF, APPROACH, LOWER, ENCIRCLE, \
    POLLINATE, RETREAT, RETURN, LAND, DONE = range(10)

PHASE_NAMES = {
    READY:     "READY",
    TAKEOFF:   "TAKEOFF",
    APPROACH:  "APPROACH",
    LOWER:     "LOWER",
    ENCIRCLE:  "ENCIRCLE",
    POLLINATE: "POLLINATE",
    RETREAT:   "RETREAT",
    RETURN:    "RETURN",
    LAND:      "LAND",
    DONE:      "DONE",
}


# ── Geometry helpers ───────────────────────────────────────────────
def wrap_pi(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def dist3(a, b) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def dist2(a, b) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


@dataclass
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float
    pos_tol: float = 0.15     # how close before we declare "arrived"
    yaw_tol: float = 0.10
    timeout: float = 30.0     # safety: force advance after this many s


class VisualServoingController(Node):
    """ROS 2 node — one instance per drone."""

    def __init__(self):
        super().__init__('visual_servoing_controller')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('flower_x', 4.0)
        self.declare_parameter('flower_y', 4.0)
        self.declare_parameter('flower_z', 1.5)
        self.declare_parameter('base_x',   1.5)
        self.declare_parameter('base_y',   1.0)
        self.declare_parameter('cruise_z', 3.0)
        self.declare_parameter('approach_radius', 2.0)
        self.declare_parameter('encircle_radius', 1.0)
        self.declare_parameter('touch_radius', 0.30)
        self.declare_parameter('start_delay', 0.0)

        self.drone_id     = self.get_parameter('drone_id').value
        self.flower       = (float(self.get_parameter('flower_x').value),
                             float(self.get_parameter('flower_y').value),
                             float(self.get_parameter('flower_z').value))
        self.base         = (float(self.get_parameter('base_x').value),
                             float(self.get_parameter('base_y').value),
                             0.10)
        self.cruise_z         = float(self.get_parameter('cruise_z').value)
        self.approach_radius  = float(self.get_parameter('approach_radius').value)
        self.encircle_radius  = float(self.get_parameter('encircle_radius').value)
        self.touch_radius     = float(self.get_parameter('touch_radius').value)
        self.start_delay      = float(self.get_parameter('start_delay').value)

        self.ns       = f'drone_{self.drone_id}'
        self.tag      = f'[D{self.drone_id}]'

        # ── PID gains (tuned for the velocity-control plugin) ──────
        # Kinematic gains — output is body velocity, no integral term
        # needed because the plant is integrating (velocity → position).
        self.kp_xy = 1.2
        self.kp_z  = 1.1
        self.kp_yaw = 1.5
        self.kd_xy = 0.25
        self.kd_z  = 0.20
        self.vmax_xy = 1.2      # m/s
        self.vmax_z  = 0.8      # m/s
        self.vmax_yaw = 1.5     # rad/s

        # ── State ───────────────────────────────────────────────────
        self.pose       = None        # current (x, y, z, yaw)
        self.prev_err   = (0.0, 0.0, 0.0)
        self.prev_t     = None
        self.phase      = READY
        self.setpoint   = None
        self.phase_t0   = None
        self.encircle_start_yaw = None
        self.encircle_progress  = 0.0
        self.mission_start_t    = None
        self.armed       = False     # mission start trigger

        # ── ROS interfaces ─────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_cmd = self.create_publisher(
            Twist, f'/model/{self.ns}/cmd_vel', 10)

        self.pub_state  = self.create_publisher(
            String, f'/pollination/{self.ns}/state', 10)
        self.pub_event  = self.create_publisher(
            String, f'/pollination/{self.ns}/event', 10)
        self.pub_pose   = self.create_publisher(
            PoseStamped, f'/pollination/{self.ns}/pose', 10)
        self.pub_target = self.create_publisher(
            PoseStamped, f'/pollination/{self.ns}/target', 10)

        self.create_subscription(
            PoseStamped,
            f'/model/{self.ns}/pose',
            self._on_pose,
            sensor_qos,
        )

        # Mission start signal (swarm coordinator publishes True)
        self.create_subscription(
            Bool, '/pollination/start', self._on_start, 10)

        # Control loop @ 20 Hz
        self.timer = self.create_timer(0.05, self._on_tick)

        self.get_logger().info(
            f"{self.tag} READY at base {self.base[:2]}, "
            f"flower at {self.flower}, cruise z={self.cruise_z}")

    # ────────────────────────────────────────────────────────────────
    # Subscriptions
    # ────────────────────────────────────────────────────────────────
    def _on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        # Yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (msg.pose.position.x,
                     msg.pose.position.y,
                     msg.pose.position.z,
                     yaw)

        # Republish for downstream nodes (dashboard, logger)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.ns
        out.pose = msg.pose
        self.pub_pose.publish(out)

    def _on_start(self, msg: Bool):
        if msg.data and not self.armed:
            self.armed = True
            now = self.get_clock().now().nanoseconds * 1e-9
            self.mission_start_t = now + self.start_delay
            self._emit_event(
                f"mission armed; will launch in {self.start_delay:.1f}s")

    # ────────────────────────────────────────────────────────────────
    # Control tick
    # ────────────────────────────────────────────────────────────────
    def _on_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # If we haven't got a pose yet, just publish the current state
        if self.pose is None:
            self._publish_state()
            return

        # Wait for mission start
        if not self.armed:
            self._publish_state()
            self._send_velocity(0, 0, 0, 0)
            return
        if self.mission_start_t is not None and now < self.mission_start_t:
            self._publish_state()
            self._send_velocity(0, 0, 0, 0)
            return

        # Advance phase if needed
        if self.setpoint is None:
            self._enter_phase(self.phase, now)

        # Check phase-completion conditions
        self._check_advance(now)

        # Run controller toward current setpoint
        if self.phase == DONE:
            self._send_velocity(0, 0, 0, 0)
            self._publish_state()
            return

        self._run_pid(now)
        self._publish_state()

    # ────────────────────────────────────────────────────────────────
    # Phase machinery
    # ────────────────────────────────────────────────────────────────
    def _enter_phase(self, phase: int, now: float):
        self.phase = phase
        self.phase_t0 = now
        self.prev_t = None
        self.prev_err = (0.0, 0.0, 0.0)

        x, y, z, yaw = self.pose
        fx, fy, fz = self.flower

        if phase == READY:
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1], z=0.10,
                                     yaw=0.0, timeout=2.0)

        elif phase == TAKEOFF:
            # Climb vertically to cruise altitude over the base pad
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1],
                                     z=self.cruise_z, yaw=0.0,
                                     timeout=12.0)

        elif phase == APPROACH:
            # Move horizontally to a point `approach_radius` south of
            # the flower at cruise altitude, yaw toward the flower.
            tx = fx
            ty = fy - self.approach_radius
            target_yaw = math.atan2(fy - ty, fx - tx)   # = π/2 (face north)
            self.setpoint = Setpoint(x=tx, y=ty, z=self.cruise_z,
                                     yaw=target_yaw, timeout=15.0)

        elif phase == LOWER:
            # Descend to flower head height, keep xy & yaw.
            self.setpoint = Setpoint(x=fx, y=fy - self.approach_radius,
                                     z=fz, yaw=math.pi/2, timeout=8.0)

        elif phase == ENCIRCLE:
            # Half-circle from south side around to west side, ending
            # with drone facing the flower directly from the west.
            # We mark the start time and let _run_pid handle the moving
            # setpoint inside this phase.
            self.encircle_start_yaw = math.atan2(y - fy, x - fx)
            # current angular position around flower
            # target: same starting position (we just orbit a bit then
            # return to the south side so the demo is symmetric).
            tx = fx
            ty = fy - self.approach_radius
            self.setpoint = Setpoint(x=tx, y=ty, z=fz, yaw=math.pi/2,
                                     timeout=14.0)
            self._encircle_t0 = now

        elif phase == POLLINATE:
            # Final approach — close the gap to touch_radius from flower,
            # at flower height, facing it directly.
            tx = fx
            ty = fy - self.touch_radius
            self.setpoint = Setpoint(x=tx, y=ty, z=fz, yaw=math.pi/2,
                                     pos_tol=0.08, timeout=6.0)

        elif phase == RETREAT:
            tx = fx
            ty = fy - self.approach_radius
            self.setpoint = Setpoint(x=tx, y=ty, z=fz, yaw=math.pi/2,
                                     timeout=6.0)

        elif phase == RETURN:
            # Climb to cruise altitude, then fly back over the base pad.
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1],
                                     z=self.cruise_z, yaw=0.0,
                                     timeout=15.0)

        elif phase == LAND:
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1],
                                     z=0.10, yaw=0.0,
                                     pos_tol=0.10, timeout=8.0)

        elif phase == DONE:
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1],
                                     z=0.10, yaw=0.0, timeout=1.0)

        self._emit_event(f"→ {PHASE_NAMES[phase]}  "
                         f"target=({self.setpoint.x:.2f}, "
                         f"{self.setpoint.y:.2f}, {self.setpoint.z:.2f})")

    def _check_advance(self, now: float):
        if self.setpoint is None or self.phase == DONE:
            return
        x, y, z, yaw = self.pose
        d_pos = dist3((x, y, z),
                      (self.setpoint.x, self.setpoint.y, self.setpoint.z))
        d_yaw = abs(wrap_pi(self.setpoint.yaw - yaw))
        elapsed = now - self.phase_t0

        # Special handling for ENCIRCLE: progress is angular sweep
        if self.phase == ENCIRCLE:
            # Sweep from -90° (south) through -180° (west) back to -90°
            t_enc = now - self._encircle_t0
            T_ENCIRCLE = 8.0       # total seconds for the orbit
            self.encircle_progress = min(t_enc / T_ENCIRCLE, 1.0)

            if self.encircle_progress >= 1.0:
                self._advance()
            return

        arrived = (d_pos < self.setpoint.pos_tol
                   and d_yaw < self.setpoint.yaw_tol)
        timed_out = elapsed > self.setpoint.timeout

        # READY phase always advances after start_delay
        if self.phase == READY and elapsed >= 1.0:
            self._advance()
            return

        if arrived or timed_out:
            if timed_out and not arrived:
                self._emit_event(
                    f"{PHASE_NAMES[self.phase]} timeout "
                    f"(pos err {d_pos:.2f}m, yaw err {math.degrees(d_yaw):.1f}°)")
            self._advance()

    def _advance(self):
        nxt = {
            READY: TAKEOFF, TAKEOFF: APPROACH, APPROACH: LOWER,
            LOWER: ENCIRCLE, ENCIRCLE: POLLINATE, POLLINATE: RETREAT,
            RETREAT: RETURN, RETURN: LAND, LAND: DONE, DONE: DONE,
        }[self.phase]

        if self.phase == POLLINATE:
            self._emit_event("✿ POLLINATION TOUCH ✿")
        if nxt == DONE:
            self._emit_event("mission complete")

        self.phase = nxt
        self.setpoint = None    # will be (re)built on next tick

    # ────────────────────────────────────────────────────────────────
    # PID + encircle interpolation
    # ────────────────────────────────────────────────────────────────
    def _run_pid(self, now: float):
        x, y, z, yaw = self.pose

        # If we're in ENCIRCLE, the setpoint moves around the flower.
        if self.phase == ENCIRCLE:
            fx, fy, fz = self.flower
            # Start angle: -π/2 (drone south of flower).
            # Sweep through -π (west) and back to -π/2 — a half-orbit.
            start_ang = -math.pi / 2
            sweep     = -math.pi             # -π/2 → -3π/2 = -π/2 again
            ang = start_ang + sweep * self.encircle_progress
            tx = fx + self.encircle_radius * math.cos(ang)
            ty = fy + self.encircle_radius * math.sin(ang)
            tz = fz
            # Always face the flower
            tyaw = math.atan2(fy - ty, fx - tx)
            self.setpoint.x, self.setpoint.y, self.setpoint.z = tx, ty, tz
            self.setpoint.yaw = tyaw

        sp = self.setpoint
        ex = sp.x - x
        ey = sp.y - y
        ez = sp.z - z
        eyaw = wrap_pi(sp.yaw - yaw)

        # Derivative (very simple low-pass)
        if self.prev_t is None:
            dex = dey = dez = 0.0
        else:
            dt = max(now - self.prev_t, 1e-3)
            dex = (ex - self.prev_err[0]) / dt
            dey = (ey - self.prev_err[1]) / dt
            dez = (ez - self.prev_err[2]) / dt
        self.prev_t   = now
        self.prev_err = (ex, ey, ez)

        # World-frame velocity commands
        vx_w = self.kp_xy * ex + self.kd_xy * dex
        vy_w = self.kp_xy * ey + self.kd_xy * dey
        vz_w = self.kp_z  * ez + self.kd_z  * dez
        wz   = self.kp_yaw * eyaw

        # Saturate
        vx_w = self._clamp(vx_w, self.vmax_xy)
        vy_w = self._clamp(vy_w, self.vmax_xy)
        vz_w = self._clamp(vz_w, self.vmax_z)
        wz   = self._clamp(wz,   self.vmax_yaw)

        # Transform world velocity → body velocity
        # (Gazebo's velocity-control plugin treats Twist in the
        # body frame.)
        c, s = math.cos(yaw), math.sin(yaw)
        vx_b =  c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w

        self._send_velocity(vx_b, vy_b, vz_w, wz)

        # Publish current target for visualisation
        tgt = PoseStamped()
        tgt.header.stamp = self.get_clock().now().to_msg()
        tgt.header.frame_id = self.ns
        tgt.pose.position.x = sp.x
        tgt.pose.position.y = sp.y
        tgt.pose.position.z = sp.z
        tgt.pose.orientation.w = math.cos(sp.yaw / 2)
        tgt.pose.orientation.z = math.sin(sp.yaw / 2)
        self.pub_target.publish(tgt)

    @staticmethod
    def _clamp(v: float, vmax: float) -> float:
        return max(-vmax, min(vmax, v))

    def _send_velocity(self, vx: float, vy: float, vz: float, wz: float):
        t = Twist()
        t.linear.x  = vx
        t.linear.y  = vy
        t.linear.z  = vz
        t.angular.z = wz
        self.pub_cmd.publish(t)

    # ────────────────────────────────────────────────────────────────
    def _publish_state(self):
        m = String()
        m.data = PHASE_NAMES[self.phase]
        self.pub_state.publish(m)

    def _emit_event(self, msg: str):
        full = f"{self.tag} {msg}"
        self.get_logger().info(full)
        m = String()
        m.data = full
        self.pub_event.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure the drone stops if we Ctrl-C
        stop = Twist()
        node.pub_cmd.publish(stop)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
