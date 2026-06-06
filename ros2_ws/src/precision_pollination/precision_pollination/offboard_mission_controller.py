#!/usr/bin/env python3
"""
offboard_mission_controller.py — Stage 14

One instance per drone (drone_id ∈ {0, 1, 2}).
Implements the FULL proven PX4 offboard mode sequence:

    counter < 10:  publish OffboardControlMode at 10 Hz  ← heartbeat
    counter == 10: send VEHICLE_CMD_DO_SET_MODE (OFFBOARD)
                   send VEHICLE_CMD_COMPONENT_ARM_DISARM (ARM)
    counter > 10:  publish OffboardControlMode + TrajectorySetpoint

Then runs the pollination mission state machine: takeoff → flower 1 →
descend → touch → ascend → flower 2 → descend → touch → ascend →
return-to-base → land → disarm.

Pattern derived from ARK-Electronics/ROS2_PX4_Offboard_Example
and Jaeyoung-Lim/px4-offboard (both proven working).

PX4 reports local position in the EKF2 NED frame.  Because each drone
spawned at its own base, all 3 drones use the SAME local-NED
waypoints — just the global ROS namespace differs.

NED axes:        x = North (+y in world ENU)
                 y = East  (+x in world ENU)
                 z = Down  (UP is NEGATIVE)
"""
from __future__ import annotations
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from std_msgs.msg import String, Bool
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


# Mission phases
WAIT_INIT, ARM_OFFBOARD, TAKEOFF, GOTO_F1, DESCEND_F1, POLLINATE_F1, \
    ASCEND_F1, GOTO_F2, DESCEND_F2, POLLINATE_F2, ASCEND_F2, \
    RETURN_HOME, LAND, DONE = range(14)

PHASE_NAMES = {
    WAIT_INIT:    "WAIT_INIT",     ARM_OFFBOARD: "ARM_OFFBOARD",
    TAKEOFF:      "TAKEOFF",       GOTO_F1:      "GOTO_F1",
    DESCEND_F1:   "DESCEND_F1",    POLLINATE_F1: "POLLINATE_F1",
    ASCEND_F1:    "ASCEND_F1",     GOTO_F2:      "GOTO_F2",
    DESCEND_F2:   "DESCEND_F2",    POLLINATE_F2: "POLLINATE_F2",
    ASCEND_F2:    "ASCEND_F2",     RETURN_HOME:  "RETURN_HOME",
    LAND:         "LAND",          DONE:         "DONE",
}


class OffboardMissionController(Node):
    def __init__(self):
        super().__init__('offboard_mission_controller')

        # Parameters
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('cruise_z', -3.0)     # NED, up = negative
        self.declare_parameter('flower_z', -1.7)
        self.declare_parameter('touch_seconds', 2.0)
        self.declare_parameter('pos_tol', 0.30)
        self.declare_parameter('mission_start_delay', 5.0)

        self.drone_id  = int(self.get_parameter('drone_id').value)
        self.cruise_z  = float(self.get_parameter('cruise_z').value)
        self.flower_z  = float(self.get_parameter('flower_z').value)
        self.touch_sec = float(self.get_parameter('touch_seconds').value)
        self.pos_tol   = float(self.get_parameter('pos_tol').value)
        self.start_delay = float(
            self.get_parameter('mission_start_delay').value)

        self.tag = f"[D{self.drone_id}]"
        self.ns  = f"/px4_{self.drone_id}"

        # Flower waypoints in LOCAL NED relative to drone spawn:
        # F1 is 3 m north of spawn, F2 is 4 m north + 2 m east.
        self.wp_f1 = (3.0, 0.0)   # (NED_x = north, NED_y = east)
        self.wp_f2 = (4.0, 2.0)

        # QoS — must match PX4's defaults for /fmu/in and /fmu/out
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_ocm   = self.create_publisher(
            OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_pub)
        self.pub_traj  = self.create_publisher(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_pub)
        self.pub_cmd   = self.create_publisher(
            VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', qos_pub)

        # Telemetry
        self.local_pos = VehicleLocalPosition()
        self.veh_stat  = VehicleStatus()
        self.create_subscription(
            VehicleLocalPosition,
            f'{self.ns}/fmu/out/vehicle_local_position',
            self._on_local_pos, qos_sub)
        self.create_subscription(
            VehicleStatus,
            f'{self.ns}/fmu/out/vehicle_status',
            self._on_status, qos_sub)

        # Side-channel topics (for our swarm coordinator + logger)
        self.pub_state = self.create_publisher(
            String, f'/pollination/drone_{self.drone_id}/state', 10)
        self.pub_event = self.create_publisher(
            String, f'/pollination/drone_{self.drone_id}/event', 10)
        self.create_subscription(
            Bool, '/pollination/start', self._on_start_sig, 10)

        # State
        self.phase = WAIT_INIT
        self.heartbeat_counter = 0
        self.phase_t0 = None
        self.armed = False
        self.start_signal_received = False
        self.start_t = None

        # 10 Hz timer
        self.timer = self.create_timer(0.1, self._on_tick)

        self.get_logger().info(
            f"{self.tag} initialised — namespace {self.ns}, "
            f"cruise_z={self.cruise_z}, flower_z={self.flower_z}")

    # ─── Subscriptions ────────────────────────────────────
    def _on_local_pos(self, msg):
        self.local_pos = msg

    def _on_status(self, msg):
        self.veh_stat = msg
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed = True

    def _on_start_sig(self, msg):
        if msg.data and not self.start_signal_received:
            self.start_signal_received = True
            now = self.get_clock().now().nanoseconds * 1e-9
            self.start_t = now + self.start_delay
            self._emit_event(
                f"start signal received; lift-off in {self.start_delay:.0f}s")

    # ─── Publishing helpers ───────────────────────────────
    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_offboard_heartbeat(self):
        m = OffboardControlMode()
        m.position     = True
        m.velocity     = False
        m.acceleration = False
        m.attitude     = False
        m.body_rate    = False
        m.timestamp    = self._ts()
        self.pub_ocm.publish(m)

    def _send_setpoint(self, x, y, z, yaw=1.5708):
        m = TrajectorySetpoint()
        m.position = [float(x), float(y), float(z)]
        m.yaw      = float(yaw)
        m.timestamp = self._ts()
        self.pub_traj.publish(m)

    def _send_command(self, command, **params):
        m = VehicleCommand()
        m.command = int(command)
        m.param1  = float(params.get('param1', 0.0))
        m.param2  = float(params.get('param2', 0.0))
        m.param3  = float(params.get('param3', 0.0))
        m.param4  = float(params.get('param4', 0.0))
        m.param5  = float(params.get('param5', 0.0))
        m.param6  = float(params.get('param6', 0.0))
        m.param7  = float(params.get('param7', 0.0))
        m.target_system    = self.drone_id + 1
        m.target_component = 1
        m.source_system    = 1
        m.source_component = 1
        m.from_external    = True
        m.timestamp        = self._ts()
        self.pub_cmd.publish(m)

    def _arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                           param1=1.0)
        self._emit_event("ARM command sent")

    def _engage_offboard(self):
        # param1=1 → custom main mode, param2=6 → PX4 OFFBOARD
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           param1=1.0, param2=6.0)
        self._emit_event("OFFBOARD mode requested")

    def _land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._emit_event("LAND command sent")

    # ─── Mission state machine ────────────────────────────
    def _emit_event(self, txt):
        full = f"{self.tag} {txt}"
        self.get_logger().info(full)
        msg = String()
        msg.data = full
        self.pub_event.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = PHASE_NAMES[self.phase]
        self.pub_state.publish(msg)

    def _at(self, x, y, z, tol=None):
        if tol is None: tol = self.pos_tol
        dx = self.local_pos.x - x
        dy = self.local_pos.y - y
        dz = self.local_pos.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz) < tol

    def _on_tick(self):
        # Always publish state for the swarm coordinator
        self._publish_state()

        now = self.get_clock().now().nanoseconds * 1e-9

        # === WAIT_INIT: wait until we hear from the FCU ===
        if self.phase == WAIT_INIT:
            if self.veh_stat.timestamp > 0 and self.local_pos.timestamp > 0:
                self._emit_event("FCU online; waiting for start signal")
                self.phase = ARM_OFFBOARD
                self.phase_t0 = now
                self.heartbeat_counter = 0
            return

        # === ARM_OFFBOARD: heartbeat then mode + arm ===
        if self.phase == ARM_OFFBOARD:
            # Wait for start signal + start_delay before we begin
            if not self.start_signal_received:
                return
            if self.start_t is not None and now < self.start_t:
                # Still pre-roll the heartbeat so PX4 trusts us
                self._send_offboard_heartbeat()
                return

            self._send_offboard_heartbeat()

            if self.heartbeat_counter == 10:
                self._engage_offboard()
                self._arm()

            self.heartbeat_counter += 1

            # Once armed AND in offboard nav state, advance.
            if (self.armed and
                self.veh_stat.nav_state ==
                    VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self._emit_event("ARMED in OFFBOARD — taking off")
                self.phase = TAKEOFF
                self.phase_t0 = now
            return

        # From here on, ALWAYS publish heartbeat + a setpoint
        self._send_offboard_heartbeat()

        if self.phase == TAKEOFF:
            self._send_setpoint(0.0, 0.0, self.cruise_z)
            if self._at(0.0, 0.0, self.cruise_z, tol=0.5):
                self._emit_event(f"at cruise altitude → flower 1")
                self.phase = GOTO_F1
                self.phase_t0 = now

        elif self.phase == GOTO_F1:
            x, y = self.wp_f1
            self._send_setpoint(x, y, self.cruise_z)
            if self._at(x, y, self.cruise_z, tol=0.4):
                self._emit_event("over flower 1 → descending")
                self.phase = DESCEND_F1
                self.phase_t0 = now

        elif self.phase == DESCEND_F1:
            x, y = self.wp_f1
            self._send_setpoint(x, y, self.flower_z)
            if self._at(x, y, self.flower_z, tol=0.3):
                self._emit_event("✿ POLLINATING flower 1 ✿")
                self.phase = POLLINATE_F1
                self.phase_t0 = now

        elif self.phase == POLLINATE_F1:
            x, y = self.wp_f1
            self._send_setpoint(x, y, self.flower_z)
            if now - self.phase_t0 > self.touch_sec:
                self.phase = ASCEND_F1
                self.phase_t0 = now

        elif self.phase == ASCEND_F1:
            x, y = self.wp_f1
            self._send_setpoint(x, y, self.cruise_z)
            if self._at(x, y, self.cruise_z, tol=0.4):
                self._emit_event("ascended → flower 2")
                self.phase = GOTO_F2
                self.phase_t0 = now

        elif self.phase == GOTO_F2:
            x, y = self.wp_f2
            self._send_setpoint(x, y, self.cruise_z)
            if self._at(x, y, self.cruise_z, tol=0.4):
                self._emit_event("over flower 2 → descending")
                self.phase = DESCEND_F2
                self.phase_t0 = now

        elif self.phase == DESCEND_F2:
            x, y = self.wp_f2
            self._send_setpoint(x, y, self.flower_z)
            if self._at(x, y, self.flower_z, tol=0.3):
                self._emit_event("✿ POLLINATING flower 2 ✿")
                self.phase = POLLINATE_F2
                self.phase_t0 = now

        elif self.phase == POLLINATE_F2:
            x, y = self.wp_f2
            self._send_setpoint(x, y, self.flower_z)
            if now - self.phase_t0 > self.touch_sec:
                self.phase = ASCEND_F2
                self.phase_t0 = now

        elif self.phase == ASCEND_F2:
            x, y = self.wp_f2
            self._send_setpoint(x, y, self.cruise_z)
            if self._at(x, y, self.cruise_z, tol=0.4):
                self._emit_event("ascended → returning home")
                self.phase = RETURN_HOME
                self.phase_t0 = now

        elif self.phase == RETURN_HOME:
            self._send_setpoint(0.0, 0.0, self.cruise_z)
            if self._at(0.0, 0.0, self.cruise_z, tol=0.4):
                self._emit_event("over home → landing")
                self.phase = LAND
                self.phase_t0 = now
                self._land()

        elif self.phase == LAND:
            # PX4 takes over landing; we just wait for disarm
            if (self.veh_stat.arming_state ==
                    VehicleStatus.ARMING_STATE_DISARMED):
                self._emit_event("★ MISSION COMPLETE ★")
                self.phase = DONE


def main(args=None):
    rclpy.init(args=args)
    node = OffboardMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
