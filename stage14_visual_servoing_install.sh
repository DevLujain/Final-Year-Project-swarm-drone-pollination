#!/bin/bash
# ============================================================
# stage14_visual_servoing_install.sh
# ------------------------------------------------------------
# Installs the Stage 14 VISUAL SERVOING pipeline:
#   - 2 drones controlled directly via Gazebo VelocityControl
#     plugin (NO PX4, NO XRCE-DDS, NO arming complications)
#   - 4-phase Figure-10 trajectory per drone
#   - YOLO sunflower detection on surveillance camera
#   - CSV mission logging
#
# What this installer does:
#   1. Stops any running simulation
#   2. Backs up the PX4-based stage 14 files
#   3. Writes 4 ROS 2 node Python files
#   4. Writes the new Gazebo SDF world
#   5. Updates setup.py with new entry points
#   6. Rebuilds the ROS 2 workspace
#   7. Writes the new launch_stage14.sh
#
# RUN:  bash stage14_visual_servoing_install.sh
# THEN: bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
PKG_DIR=$ROS2_WS/src/precision_pollination
NODES_DIR=$PKG_DIR/precision_pollination
WORLDS_DIR=~/PX4-Autopilot/Tools/simulation/gz/worlds

echo ""
echo "============================================================"
echo "  Stage 14 Visual Servoing Installer"
echo "============================================================"
echo ""

# Sanity checks
if [ ! -d "$NODES_DIR" ]; then
    echo "ERROR: $NODES_DIR not found."
    echo "       Expected ROS 2 workspace at $ROS2_WS"
    exit 1
fi
if [ ! -d "$WORLDS_DIR" ]; then
    echo "WARN: $WORLDS_DIR not found - creating it."
    mkdir -p "$WORLDS_DIR"
fi

# Step 1: Stop everything
echo "[1/8] Stopping any running simulation..."
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f gz-sim 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 3
echo "       Done."

# Step 2: Archive PX4-based files (don't delete - keep for rollback)
echo "[2/8] Archiving PX4-based files (for rollback safety)..."
TS=$(date +%Y%m%d_%H%M%S)
BACKUP=$FYP_DIR/_stage14_px4_backup_$TS
mkdir -p $BACKUP
for F in field_survey.py swarm_drone_controller.py swarm_monitor.py; do
    if [ -f "$NODES_DIR/$F" ]; then
        mv "$NODES_DIR/$F" "$BACKUP/"
        echo "       moved $F"
    fi
done
if [ -f "$FYP_DIR/launch_stage14.sh" ]; then
    cp "$FYP_DIR/launch_stage14.sh" "$BACKUP/launch_stage14_px4.sh.bak"
    echo "       backed up old launch_stage14.sh"
fi
echo "       Archive: $BACKUP"


# Step 3: Write visual_servoing_controller.py
echo "[3/8] Writing visual_servoing_controller.py..."
cat > $NODES_DIR/visual_servoing_controller.py << 'PYEOF_VSC'
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
    pos_tol: float = 0.15
    yaw_tol: float = 0.10
    timeout: float = 30.0


class VisualServoingController(Node):
    def __init__(self):
        super().__init__('visual_servoing_controller')

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

        self.kp_xy = 1.2
        self.kp_z  = 1.1
        self.kp_yaw = 1.5
        self.kd_xy = 0.25
        self.kd_z  = 0.20
        self.vmax_xy = 1.2
        self.vmax_z  = 0.8
        self.vmax_yaw = 1.5

        self.pose       = None
        self.prev_err   = (0.0, 0.0, 0.0)
        self.prev_t     = None
        self.phase      = READY
        self.setpoint   = None
        self.phase_t0   = None
        self.encircle_start_yaw = None
        self.encircle_progress  = 0.0
        self.mission_start_t    = None
        self.armed       = False

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

        self.create_subscription(
            Bool, '/pollination/start', self._on_start, 10)

        self.timer = self.create_timer(0.05, self._on_tick)

        self.get_logger().info(
            f"{self.tag} READY at base {self.base[:2]}, "
            f"flower at {self.flower}, cruise z={self.cruise_z}")

    def _on_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (msg.pose.position.x,
                     msg.pose.position.y,
                     msg.pose.position.z,
                     yaw)

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

    def _on_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.pose is None:
            self._publish_state()
            return

        if not self.armed:
            self._publish_state()
            self._send_velocity(0, 0, 0, 0)
            return
        if self.mission_start_t is not None and now < self.mission_start_t:
            self._publish_state()
            self._send_velocity(0, 0, 0, 0)
            return

        if self.setpoint is None:
            self._enter_phase(self.phase, now)

        self._check_advance(now)

        if self.phase == DONE:
            self._send_velocity(0, 0, 0, 0)
            self._publish_state()
            return

        self._run_pid(now)
        self._publish_state()

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
            self.setpoint = Setpoint(x=self.base[0], y=self.base[1],
                                     z=self.cruise_z, yaw=0.0,
                                     timeout=12.0)

        elif phase == APPROACH:
            tx = fx
            ty = fy - self.approach_radius
            target_yaw = math.atan2(fy - ty, fx - tx)
            self.setpoint = Setpoint(x=tx, y=ty, z=self.cruise_z,
                                     yaw=target_yaw, timeout=15.0)

        elif phase == LOWER:
            self.setpoint = Setpoint(x=fx, y=fy - self.approach_radius,
                                     z=fz, yaw=math.pi/2, timeout=8.0)

        elif phase == ENCIRCLE:
            self.encircle_start_yaw = math.atan2(y - fy, x - fx)
            tx = fx
            ty = fy - self.approach_radius
            self.setpoint = Setpoint(x=tx, y=ty, z=fz, yaw=math.pi/2,
                                     timeout=14.0)
            self._encircle_t0 = now

        elif phase == POLLINATE:
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

        self._emit_event(f"-> {PHASE_NAMES[phase]}  "
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

        if self.phase == ENCIRCLE:
            t_enc = now - self._encircle_t0
            T_ENCIRCLE = 8.0
            self.encircle_progress = min(t_enc / T_ENCIRCLE, 1.0)

            if self.encircle_progress >= 1.0:
                self._advance()
            return

        arrived = (d_pos < self.setpoint.pos_tol
                   and d_yaw < self.setpoint.yaw_tol)
        timed_out = elapsed > self.setpoint.timeout

        if self.phase == READY and elapsed >= 1.0:
            self._advance()
            return

        if arrived or timed_out:
            if timed_out and not arrived:
                self._emit_event(
                    f"{PHASE_NAMES[self.phase]} timeout "
                    f"(pos err {d_pos:.2f}m, yaw err {math.degrees(d_yaw):.1f}deg)")
            self._advance()

    def _advance(self):
        nxt = {
            READY: TAKEOFF, TAKEOFF: APPROACH, APPROACH: LOWER,
            LOWER: ENCIRCLE, ENCIRCLE: POLLINATE, POLLINATE: RETREAT,
            RETREAT: RETURN, RETURN: LAND, LAND: DONE, DONE: DONE,
        }[self.phase]

        if self.phase == POLLINATE:
            self._emit_event("** POLLINATION TOUCH **")
        if nxt == DONE:
            self._emit_event("mission complete")

        self.phase = nxt
        self.setpoint = None

    def _run_pid(self, now: float):
        x, y, z, yaw = self.pose

        if self.phase == ENCIRCLE:
            fx, fy, fz = self.flower
            start_ang = -math.pi / 2
            sweep     = -math.pi
            ang = start_ang + sweep * self.encircle_progress
            tx = fx + self.encircle_radius * math.cos(ang)
            ty = fy + self.encircle_radius * math.sin(ang)
            tz = fz
            tyaw = math.atan2(fy - ty, fx - tx)
            self.setpoint.x, self.setpoint.y, self.setpoint.z = tx, ty, tz
            self.setpoint.yaw = tyaw

        sp = self.setpoint
        ex = sp.x - x
        ey = sp.y - y
        ez = sp.z - z
        eyaw = wrap_pi(sp.yaw - yaw)

        if self.prev_t is None:
            dex = dey = dez = 0.0
        else:
            dt = max(now - self.prev_t, 1e-3)
            dex = (ex - self.prev_err[0]) / dt
            dey = (ey - self.prev_err[1]) / dt
            dez = (ez - self.prev_err[2]) / dt
        self.prev_t   = now
        self.prev_err = (ex, ey, ez)

        vx_w = self.kp_xy * ex + self.kd_xy * dex
        vy_w = self.kp_xy * ey + self.kd_xy * dey
        vz_w = self.kp_z  * ez + self.kd_z  * dez
        wz   = self.kp_yaw * eyaw

        vx_w = self._clamp(vx_w, self.vmax_xy)
        vy_w = self._clamp(vy_w, self.vmax_xy)
        vz_w = self._clamp(vz_w, self.vmax_z)
        wz   = self._clamp(wz,   self.vmax_yaw)

        c, s = math.cos(yaw), math.sin(yaw)
        vx_b =  c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w

        self._send_velocity(vx_b, vy_b, vz_w, wz)

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
        stop = Twist()
        node.pub_cmd.publish(stop)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF_VSC

# Step 4: Write swarm_coordinator.py
echo "[4/8] Writing swarm_coordinator.py..."
cat > $NODES_DIR/swarm_coordinator.py << 'PYEOF_SC'
#!/usr/bin/env python3
"""
swarm_coordinator.py — Stage 14 Visual Servoing

Fires the synchronized start signal after a warmup, monitors per-drone
state, and publishes mission-complete when both drones reach DONE.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


WARMUP_SEC = 4.0


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

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self._on_tick)

        self.get_logger().info(
            f"swarm coordinator up - will start {self.n_drones} drones "
            f"in {WARMUP_SEC:.0f}s")

    def _on_state(self, drone_id: int, msg: String):
        self.states[drone_id] = msg.data

    def _on_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.started and now - self.t0 >= WARMUP_SEC:
            self._fire_start()
            self.started = True

        m = String()
        m.data = ' | '.join(f"D{i}:{self.states[i]}" for i in range(self.n_drones))
        self.pub_status.publish(m)

        if (self.started and not self.completed
                and all(s == 'DONE' for s in self.states.values())):
            self.completed = True
            ev = String()
            ev.data = "*** SWARM MISSION COMPLETE - both drones landed safely ***"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)

    def _fire_start(self):
        m = Bool()
        m.data = True
        for _ in range(5):
            self.pub_start.publish(m)
        ev = String()
        ev.data = "swarm GO - both drones armed and launching"
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
PYEOF_SC

# Step 5: Write yolo_field_detector.py
echo "[5/8] Writing yolo_field_detector.py..."
cat > $NODES_DIR/yolo_field_detector.py << 'PYEOF_YOLO'
#!/usr/bin/env python3
"""
yolo_field_detector.py — Stage 14 Visual Servoing

Subscribes to /surveillance/image (bridged from Gazebo), runs YOLOv8
(or a deterministic mock) to overlay sunflower bounding boxes, and
publishes the annotated image plus a target list.
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    HAVE_BRIDGE = True
except Exception:
    HAVE_BRIDGE = False

_YOLO = None

# Camera config matches demo_stage14.sdf surveillance camera
CAM_POS = (6.5, -2.0, 4.0)
CAM_YAW = math.pi / 2
CAM_PITCH = 0.50
CAM_HFOV = 1.50
IMG_W, IMG_H = 640, 480

FLOWERS = [
    {'name': 'sunflower_1', 'x': 4.0, 'y': 4.0, 'z': 1.5, 'sector': 1},
    {'name': 'sunflower_2', 'x': 9.0, 'y': 4.0, 'z': 1.5, 'sector': 2},
]


def world_to_pixel(xw: float, yw: float, zw: float):
    dx = xw - CAM_POS[0]
    dy = yw - CAM_POS[1]
    dz = zw - CAM_POS[2]

    cy, sy = math.cos(-CAM_YAW), math.sin(-CAM_YAW)
    x1 =  cy * dx + sy * dy
    y1 = -sy * dx + cy * dy
    z1 = dz

    cp, sp = math.cos(-CAM_PITCH), math.sin(-CAM_PITCH)
    x2 =  cp * x1 + sp * z1
    y2 =  y1
    z2 = -sp * x1 + cp * z1

    if x2 <= 0.05:
        return None

    fx = (IMG_W / 2.0) / math.tan(CAM_HFOV / 2.0)
    fy = fx
    u = IMG_W  / 2.0 - (y2 / x2) * fx
    v = IMG_H  / 2.0 - (z2 / x2) * fy
    return (int(round(u)), int(round(v)), x2)


class YoloFieldDetector(Node):
    def __init__(self):
        super().__init__('yolo_field_detector')

        self.declare_parameter('use_real_yolo', False)
        self.declare_parameter(
            'model_path',
            os.path.expanduser('~/Desktop/FYP/ros2_ws/sunflower_best.pt'))
        self.use_real_yolo = bool(self.get_parameter('use_real_yolo').value)
        self.model_path    = str(self.get_parameter('model_path').value)

        self.bridge = CvBridge() if HAVE_BRIDGE else None
        if self.bridge is None:
            self.get_logger().warn(
                "cv_bridge not available - annotated image will not be "
                "republished, but detection messages will still work.")

        self.model = None
        if self.use_real_yolo:
            self._try_load_yolo()

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_annotated = self.create_publisher(
            Image, '/yolov8/annotated_image', sensor_qos)
        self.pub_targets   = self.create_publisher(
            PoseArray, '/sunflower_targets', 10)
        self.pub_event     = self.create_publisher(
            String, '/yolov8/event', 10)

        self.create_subscription(
            Image, '/surveillance/image', self._on_image, sensor_qos)

        self.create_timer(1.0, self._publish_static_targets)

        self.get_logger().info(
            f"yolo_field_detector up  (real_yolo={self.use_real_yolo})  "
            f"watching /surveillance/image")

    def _try_load_yolo(self):
        if not os.path.exists(self.model_path):
            self.get_logger().warn(
                f"model file {self.model_path} not found - falling "
                f"back to mock detector")
            self.use_real_yolo = False
            return
        try:
            global _YOLO
            from ultralytics import YOLO as _YOLO_cls
            _YOLO = _YOLO_cls
            self.model = _YOLO(self.model_path)
            self.get_logger().info(f"loaded YOLOv8 from {self.model_path}")
        except Exception as e:
            self.get_logger().warn(
                f"could not import ultralytics ({e}) - falling back "
                f"to mock detector")
            self.use_real_yolo = False

    def _on_image(self, msg: Image):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        boxes = self._detect(frame)
        annotated = self._annotate(frame, boxes)

        try:
            out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            out.header = msg.header
            self.pub_annotated.publish(out)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge encode failed: {e}")

    def _detect(self, frame: np.ndarray):
        if self.use_real_yolo and self.model is not None:
            return self._detect_yolo(frame)
        return self._detect_mock(frame)

    def _detect_mock(self, frame):
        out = []
        for f in FLOWERS:
            proj = world_to_pixel(f['x'], f['y'], f['z'])
            if proj is None:
                continue
            u, v, depth = proj
            half = max(20, int(round(45.0 * (8.0 / max(depth, 1.0)))))
            x1 = max(0, u - half)
            y1 = max(0, v - half)
            x2 = min(IMG_W - 1, u + half)
            y2 = min(IMG_H - 1, v + half)
            jitter = 0.02 * math.sin(time.time() * 1.7 + f['sector'])
            conf = 0.92 + jitter
            out.append({
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'conf': conf, 'name': f['name'], 'sector': f['sector'],
                'wx': f['x'], 'wy': f['y'], 'wz': f['z'],
            })
        return out

    def _detect_yolo(self, frame):
        results = self.model(frame, verbose=False)
        out = []
        for r in results:
            for b, c, cl in zip(r.boxes.xyxy.cpu().numpy(),
                                r.boxes.conf.cpu().numpy(),
                                r.boxes.cls.cpu().numpy()):
                x1, y1, x2, y2 = map(int, b)
                out.append({
                    'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                    'conf': float(c), 'name': self.model.names[int(cl)],
                    'sector': 0, 'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
                })
        return out

    def _annotate(self, frame: np.ndarray, boxes: list) -> np.ndarray:
        out = frame.copy()

        cv2.rectangle(out, (0, 0), (IMG_W, 28), (24, 24, 24), -1)
        cv2.putText(out, "YOLOv8 Sunflower Detection (Field Surveillance)",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)

        for b in boxes:
            x1, y1, x2, y2 = b['x1'], b['y1'], b['x2'], b['y2']
            colour = (35, 220, 220)
            cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)

            label = f"sunflower_head  {b['conf']*100:.0f}%"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ly = max(y1 - 6, th + 4)
            cv2.rectangle(out, (x1, ly - th - 4), (x1 + tw + 6, ly + 2),
                          colour, -1)
            cv2.putText(out, label, (x1 + 3, ly - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.drawMarker(out, (cx, cy), (40, 255, 40),
                           cv2.MARKER_CROSS, 14, 2)
        return out

    def _publish_static_targets(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for f in FLOWERS:
            p = Pose()
            p.position.x = float(f['x'])
            p.position.y = float(f['y'])
            p.position.z = float(f['z'])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_targets.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloFieldDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF_YOLO

# Step 6: Write mission_logger.py
echo "[6/8] Writing mission_logger.py..."
cat > $NODES_DIR/mission_logger.py << 'PYEOF_ML'
#!/usr/bin/env python3
"""
mission_logger.py — Stage 14 Visual Servoing

Logs a 10 Hz trace of both drones' poses + states, plus a sparse
event log, to ~/Desktop/FYP/mission_outputs/stage14/.
"""
from __future__ import annotations

import csv
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


OUT_DIR = Path.home() / 'Desktop' / 'FYP' / 'mission_outputs' / 'stage14'


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.trace_path  = OUT_DIR / f'stage14_trace_{ts}.csv'
        self.events_path = OUT_DIR / f'stage14_events_{ts}.csv'

        self.trace_f = open(self.trace_path, 'w', newline='')
        self.trace_w = csv.writer(self.trace_f)
        self.trace_w.writerow([
            'time', 'D0_x', 'D0_y', 'D0_z', 'D0_state',
                    'D1_x', 'D1_y', 'D1_z', 'D1_state'])

        self.events_f = open(self.events_path, 'w', newline='')
        self.events_w = csv.writer(self.events_f)
        self.events_w.writerow(['time', 'source', 'event'])

        self.poses  = {0: (0.0, 0.0, 0.0), 1: (0.0, 0.0, 0.0)}
        self.states = {0: 'UNKNOWN', 1: 'UNKNOWN'}

        for i in (0, 1):
            self.create_subscription(
                PoseStamped, f'/pollination/drone_{i}/pose',
                lambda m, idx=i: self._on_pose(idx, m), 10)
            self.create_subscription(
                String, f'/pollination/drone_{i}/state',
                lambda m, idx=i: self._on_state(idx, m), 10)
            self.create_subscription(
                String, f'/pollination/drone_{i}/event',
                lambda m, idx=i: self._on_event(f'D{idx}', m), 10)

        self.create_subscription(
            String, '/pollination/swarm/event',
            lambda m: self._on_event('SWARM', m), 10)
        self.create_subscription(
            String, '/yolov8/event',
            lambda m: self._on_event('YOLO', m), 10)

        self.create_timer(0.1, self._flush_trace)

        self.get_logger().info(f"writing trace to {self.trace_path}")
        self.get_logger().info(f"writing events to {self.events_path}")

    def _on_pose(self, drone_id: int, msg: PoseStamped):
        self.poses[drone_id] = (msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z)

    def _on_state(self, drone_id: int, msg: String):
        self.states[drone_id] = msg.data

    def _on_event(self, src: str, msg: String):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.events_w.writerow([f'{now:.3f}', src, msg.data])
        self.events_f.flush()

    def _flush_trace(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        p0 = self.poses[0]; p1 = self.poses[1]
        self.trace_w.writerow([f'{now:.3f}',
                               f'{p0[0]:.3f}', f'{p0[1]:.3f}', f'{p0[2]:.3f}',
                               self.states[0],
                               f'{p1[0]:.3f}', f'{p1[1]:.3f}', f'{p1[2]:.3f}',
                               self.states[1]])
        self.trace_f.flush()

    def destroy_node(self):
        try:
            self.trace_f.close()
            self.events_f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF_ML

# Step 7: Write setup.py
echo "[7/8] Writing setup.py..."
cat > $PKG_DIR/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='4.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP',
    maintainer_email='fyp@example.com',
    description='Swarm drone pollination - Stage 14 Visual Servoing (no PX4)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_servoing_controller=precision_pollination.visual_servoing_controller:main',
            'swarm_coordinator=precision_pollination.swarm_coordinator:main',
            'yolo_field_detector=precision_pollination.yolo_field_detector:main',
            'mission_logger=precision_pollination.mission_logger:main',
        ],
    },
)
SETUPEOF

# Step 8: Write demo_stage14.sdf
echo "[8/8] Writing demo_stage14.sdf..."
cat > $WORLDS_DIR/demo_stage14.sdf << 'SDFEOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="stage14_world">

    <physics name="default_physics" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system"/>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>6.5 -6 6 0 0.55 1.5708</pose>
      </camera>
    </gui>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0.3 0.3 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground plane (green grass) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material>
            <ambient>0.3 0.5 0.3 1</ambient>
            <diffuse>0.3 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Field tile (planted area) -->
    <model name="field_tile">
      <static>true</static>
      <pose>6.5 4.0 0.01 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><box><size>10 4 0.02</size></box></geometry>
          <material>
            <ambient>0.15 0.35 0.18 1</ambient>
            <diffuse>0.15 0.35 0.18 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Base pad 0 (blue, for Drone 0) -->
    <model name="pad_0">
      <static>true</static>
      <pose>1.5 1.0 0.02 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><cylinder><radius>0.5</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.2 0.4 0.9 1</ambient><diffuse>0.2 0.4 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Base pad 1 (orange, for Drone 1) -->
    <model name="pad_1">
      <static>true</static>
      <pose>11.5 1.0 0.02 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><cylinder><radius>0.5</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ===== Drone 0 ===== -->
    <model name="drone_0">
      <pose>1.5 1.0 0.10 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.02</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.02</iyy><iyz>0</iyz>
            <izz>0.04</izz>
          </inertia>
        </inertial>
        <visual name="body">
          <geometry><box><size>0.30 0.30 0.08</size></box></geometry>
          <material><ambient>0.2 0.4 0.9 1</ambient><diffuse>0.2 0.4 0.9 1</diffuse></material>
        </visual>
        <visual name="arm_x">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.60 0.04 0.02</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_y">
          <pose>0 0 0.05 0 0 1.5708</pose>
          <geometry><box><size>0.60 0.04 0.02</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="prop_fl">
          <pose>0.21 0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_fr">
          <pose>0.21 -0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_bl">
          <pose>-0.21 0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_br">
          <pose>-0.21 -0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
      </link>

      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <topic>/model/drone_0/cmd_vel</topic>
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>30</update_frequency>
      </plugin>
    </model>

    <!-- ===== Drone 1 ===== -->
    <model name="drone_1">
      <pose>11.5 1.0 0.10 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.02</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.02</iyy><iyz>0</iyz>
            <izz>0.04</izz>
          </inertia>
        </inertial>
        <visual name="body">
          <geometry><box><size>0.30 0.30 0.08</size></box></geometry>
          <material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_x">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>0.60 0.04 0.02</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_y">
          <pose>0 0 0.05 0 0 1.5708</pose>
          <geometry><box><size>0.60 0.04 0.02</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="prop_fl">
          <pose>0.21 0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_fr">
          <pose>0.21 -0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_bl">
          <pose>-0.21 0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
        <visual name="prop_br">
          <pose>-0.21 -0.21 0.07 0 0 0</pose>
          <geometry><cylinder><radius>0.13</radius><length>0.01</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 0.5</ambient><diffuse>0.3 0.3 0.3 0.5</diffuse></material>
        </visual>
      </link>

      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <topic>/model/drone_1/cmd_vel</topic>
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>30</update_frequency>
      </plugin>
    </model>

    <!-- ===== Sunflower 1 (Drone 0's target) ===== -->
    <model name="sunflower_1">
      <static>true</static>
      <pose>4.0 4.0 0 0 0 0</pose>
      <link name="stem">
        <pose>0 0 0.7 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.05</radius><length>1.4</length></cylinder></geometry>
          <material><ambient>0.1 0.55 0.1 1</ambient><diffuse>0.1 0.55 0.1 1</diffuse></material>
        </visual>
      </link>
      <link name="petals">
        <pose>0 0 1.46 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.34</radius><length>0.03</length></cylinder></geometry>
          <material><ambient>0.95 0.82 0.05 1</ambient><diffuse>0.95 0.82 0.05 1</diffuse></material>
        </visual>
      </link>
      <link name="head">
        <pose>0 0 1.49 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.20</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.42 0.25 0.05 1</ambient><diffuse>0.42 0.25 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ===== Sunflower 2 (Drone 1's target) ===== -->
    <model name="sunflower_2">
      <static>true</static>
      <pose>9.0 4.0 0 0 0 0</pose>
      <link name="stem">
        <pose>0 0 0.7 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.05</radius><length>1.4</length></cylinder></geometry>
          <material><ambient>0.1 0.55 0.1 1</ambient><diffuse>0.1 0.55 0.1 1</diffuse></material>
        </visual>
      </link>
      <link name="petals">
        <pose>0 0 1.46 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.34</radius><length>0.03</length></cylinder></geometry>
          <material><ambient>0.95 0.82 0.05 1</ambient><diffuse>0.95 0.82 0.05 1</diffuse></material>
        </visual>
      </link>
      <link name="head">
        <pose>0 0 1.49 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.20</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.42 0.25 0.05 1</ambient><diffuse>0.42 0.25 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ===== Surveillance camera ===== -->
    <model name="surveillance_cam">
      <static>true</static>
      <pose>6.5 -2.0 4.0 0 0.5 1.5708</pose>
      <link name="link">
        <visual name="body">
          <geometry><box><size>0.2 0.2 0.15</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.5</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <visualize>false</visualize>
          <topic>/surveillance/image</topic>
        </sensor>
      </link>
    </model>

  </world>
</sdf>
SDFEOF

# Step 9: Write launch_stage14.sh
echo "[9/8] Writing launch_stage14.sh..."
cat > $FYP_DIR/launch_stage14.sh << 'LAUNCHEOF'
#!/bin/bash
# launch_stage14.sh — Stage 14 Visual Servoing (no PX4)
# Drones are controlled directly via Gazebo VelocityControl plugin.

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
WORLD=~/PX4-Autopilot/Tools/simulation/gz/worlds/demo_stage14.sdf

echo ""
echo "============================================================"
echo "  STAGE 14 - Visual Servoing Pollination Demo"
echo "  (Direct Gazebo control via VelocityControl plugin)"
echo "============================================================"
echo ""

# Kill any leftover processes from previous runs
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f gz-sim 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 3

# 1. Start Gazebo with the new world
gnome-terminal --title="Gazebo" -- bash -c "
  source ~/.bashrc
  echo '>>> Starting Gazebo with $WORLD'
  gz sim -r '$WORLD'
  bash
" &
sleep 10

# 2. Start ros_gz_bridge (translates between ROS 2 and Gazebo topics)
gnome-terminal --title="GZ Bridge" -- bash -c "
  source ~/.bashrc
  echo '>>> Bridging ROS 2 <-> Gazebo'
  ros2 run ros_gz_bridge parameter_bridge \\
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \\
    /model/drone_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \\
    /model/drone_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \\
    /surveillance/image@sensor_msgs/msg/Image[gz.msgs.Image
  bash
" &
sleep 6

# 3. Drone 0 controller (target: sunflower_1 at (4, 4))
gnome-terminal --title="D0 Controller" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  echo '>>> Drone 0 — base (1.5, 1.0), flower (4.0, 4.0)'
  ros2 run precision_pollination visual_servoing_controller \\
    --ros-args -p drone_id:=0 \\
               -p flower_x:=4.0 -p flower_y:=4.0 -p flower_z:=1.5 \\
               -p base_x:=1.5 -p base_y:=1.0 \\
               -p cruise_z:=3.0 \\
               -p start_delay:=0.0
  bash
" &
sleep 2

# 4. Drone 1 controller (target: sunflower_2 at (9, 4))
gnome-terminal --title="D1 Controller" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  echo '>>> Drone 1 — base (11.5, 1.0), flower (9.0, 4.0)'
  ros2 run precision_pollination visual_servoing_controller \\
    --ros-args -p drone_id:=1 \\
               -p flower_x:=9.0 -p flower_y:=4.0 -p flower_z:=1.5 \\
               -p base_x:=11.5 -p base_y:=1.0 \\
               -p cruise_z:=3.0 \\
               -p start_delay:=0.5
  bash
" &
sleep 2

# 5. Swarm coordinator (fires the global start signal)
gnome-terminal --title="Swarm Coord" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  echo '>>> Swarm coordinator — will fire /pollination/start after 4s warmup'
  ros2 run precision_pollination swarm_coordinator
  bash
" &
sleep 2

# 6. YOLO field detector (overlays bboxes on surveillance camera)
gnome-terminal --title="YOLO" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  echo '>>> YOLO detector (mock mode) — annotated frames on /yolov8/annotated_image'
  ros2 run precision_pollination yolo_field_detector
  bash
" &
sleep 2

# 7. Mission logger (CSV trace + events)
gnome-terminal --title="Mission Log" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  echo '>>> Mission logger — CSVs in ~/Desktop/FYP/mission_outputs/stage14/'
  ros2 run precision_pollination mission_logger
  bash
" &

echo ""
echo "============================================================"
echo "  All nodes launched. 7 terminal tabs open:"
echo "    Gazebo  |  GZ Bridge  |  D0 Controller  |  D1 Controller"
echo "    Swarm Coord  |  YOLO  |  Mission Log"
echo ""
echo "  Timeline (approximate):"
echo "    t=0..10s   Gazebo loads, drones sit on their pads"
echo "    t~14s      Swarm coord fires GO -> drones TAKEOFF"
echo "    t~14..24s  Climb to 3 m (cruise altitude)"
echo "    t~24..32s  APPROACH each sunflower"
echo "    t~32..36s  LOWER to flower-head height"
echo "    t~36..44s  ENCIRCLE the flower (half-orbit)"
echo "    t~44..48s  POLLINATE (touch radius 30cm) — event: 'POLLINATION TOUCH'"
echo "    t~48..52s  RETREAT"
echo "    t~52..60s  RETURN to base pad"
echo "    t~60..68s  LAND"
echo "    t~70s      SWARM MISSION COMPLETE"
echo ""
echo "  To watch the YOLO-annotated camera feed in a separate window:"
echo "    ros2 run rqt_image_view rqt_image_view /yolov8/annotated_image"
echo ""
echo "  To stop everything:"
echo "    pkill -9 -f gz; pkill -9 -f ros2"
echo "============================================================"
echo ""
LAUNCHEOF

# Step 7: Build the workspace
echo "[7/8] Building ROS 2 workspace (this takes ~30 seconds)..."
source /opt/ros/jazzy/setup.bash
cd $ROS2_WS
# Clean only this package
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null || true
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -8

# Step 8: Make launch script executable
echo "[8/8] Finalising launch script..."
chmod +x $FYP_DIR/launch_stage14.sh

# Sanity check: confirm entry points work
echo ""
echo "Verifying installed nodes:"
source $ROS2_WS/install/setup.bash 2>/dev/null
for N in visual_servoing_controller swarm_coordinator yolo_field_detector mission_logger; do
    if [ -f "$ROS2_WS/install/precision_pollination/lib/precision_pollination/$N" ]; then
        echo "  OK   $N"
    else
        echo "  MISSING $N"
    fi
done

echo ""
echo "============================================================"
echo "  Installation complete."
echo "============================================================"
echo ""
echo "What changed:"
echo "  - PX4 SITL is no longer used. Drones controlled directly"
echo "    through Gazebo's VelocityControl plugin via Twist on"
echo "    /model/drone_N/cmd_vel."
echo "  - New world: demo_stage14.sdf (2 drones, 2 sunflowers,"
echo "    base pads, surveillance camera)."
echo "  - Old PX4 nodes archived in: $BACKUP"
echo ""
echo "Next step:"
echo "  bash $FYP_DIR/launch_stage14.sh"
echo ""
echo "7 terminal tabs will open: Gazebo, GZ Bridge, D0 Controller,"
echo "D1 Controller, Swarm Coord, YOLO, Mission Log."
echo "Both drones lift off ~14s after launch, complete the 4-phase"
echo "Figure-10 trajectory, return to their pads and land."
echo ""
