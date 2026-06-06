#!/usr/bin/env python3
"""
drone_controller_stage15.py  —  Stage 15 (clean rewrite)
========================================================
ONE instance per drone. Owns the drone's full per-day mission:

  IDLE ─► TAKEOFF ─► SURVEY (lawnmower over assigned strip) ─►
        AWAIT_ROUTE ─► VISIT (per-flower: GOTO → DESCEND → BRUSH) ─►
        RTB ─► LAND ─► IDLE (ready for next day)

Identical PID-on-velocity scheme as Stage 14's visual_servoing_controller.py:
  feedback   /model/drone_N/pose      (PoseStamped from Gazebo)
  command    /model/drone_N/cmd_vel   (Twist into Gazebo VelocityControl)

INPUTS
------
  /bloom/current_day               Int32   (latched)
  /bloom/targets                   String  JSON, list of flower dicts
                                            from bloom_state_manager
  /swarm/survey_start              Bool    fires takeoff+survey
  /map_merger/routes               String  JSON {day, routes:{drone_id:[...]}}
                                            published by orchestrator
                                            after surveys complete
  /swarm/abort                     Bool    optional emergency RTB

OUTPUTS
-------
  /model/drone_N/cmd_vel               Twist  drives Gazebo VelocityControl
  /survey/drone_N/observations         String JSON, per-flower observations
  /swarm/survey_done/drone_N           Bool   true on survey-back-to-base
  /pollination/drone_N/state           String current phase name
  /pollination/drone_N/pollinated      Int32  flower_id, fires once per success
  /pollination/drone_N/at_base         Bool   true after LAND when day done
  /battery/drone_N                     Float32 0..100, naive linear drain

PARAMETERS
----------
  drone_id, n_drones, base_x, base_y, field_size_x/y,
  survey_altitude, cruise_altitude, approach_altitude,
  cruise_speed, approach_speed, contact_descent_speed,
  pistil_length, tof_contact, brush_sweep_duration,
  observation_radius, strip_pitch, battery_full_minutes

NOTES
-----
- All gains and clamps are conservative — proven on Stage 14.
- "Pose" is the model's world pose published by gz-sim-pose-publisher.
- Pollination is a strict NN+2opt route from map_merger, executed in order.
- Each phase declares its OWN velocity caps so the descend cap (0.4 m/s)
  cannot leak into TAKEOFF/CRUISE — that was a Stage 15 Pass-1 bug.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32, Float32, String


# ── Phases ─────────────────────────────────────────────────────────
IDLE, TAKEOFF, SURVEY, RETURN_FROM_SURVEY, LAND_SURVEY, \
    AWAIT_ROUTE, GOTO_FLOWER, DESCEND, BRUSH, ASCEND_GENTLE, RETREAT, \
    RTB, LAND_RTB, DAY_DONE, RTB_BATTERY, RECHARGING = range(16)

PHASE_NAMES = {
    IDLE:               'IDLE',
    TAKEOFF:            'TAKEOFF',
    SURVEY:             'SURVEY',
    RETURN_FROM_SURVEY: 'RETURN_FROM_SURVEY',
    LAND_SURVEY:        'LAND_SURVEY',
    AWAIT_ROUTE:        'AWAIT_ROUTE',
    GOTO_FLOWER:        'GOTO_FLOWER',
    DESCEND:            'DESCEND',
    BRUSH:              'BRUSH',
    ASCEND_GENTLE:      'ASCEND_GENTLE',
    RETREAT:            'RETREAT',
    RTB:                'RTB',
    LAND_RTB:           'LAND_RTB',
    DAY_DONE:           'DAY_DONE',
    RTB_BATTERY:        'RTB_BATTERY',
    RECHARGING:         'RECHARGING',
}

ACTIVE_PHASES = {TAKEOFF, SURVEY, RETURN_FROM_SURVEY, LAND_SURVEY,
                 GOTO_FLOWER, DESCEND, BRUSH, ASCEND_GENTLE, RETREAT,
                 RTB, LAND_RTB, RTB_BATTERY}


@dataclass
class PhaseLimits:
    """Per-phase velocity caps. Stage 14 used one cap for everything;
    Stage 15 Pass 1 mistakenly inherited a 0.4 m/s cap from DESCEND into
    every other phase, which made drones look stuck. Each phase now
    declares its own limits so caps cannot leak between phases."""
    vmax_xy: float
    vmax_z:  float


def clamp(v, lo, hi): return max(lo, min(hi, v))
def dist2(a, b):      return math.hypot(a[0] - b[0], a[1] - b[1])


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # ── Parameters ─────────────────────────────────────────────
        self.declare_parameter('drone_id',                  0)
        self.declare_parameter('n_drones',                  2)
        self.declare_parameter('base_x',                    7.5)
        self.declare_parameter('base_y',                   -1.0)
        self.declare_parameter('base_z',                    0.10)
        self.declare_parameter('field_size_x',             15.0)
        self.declare_parameter('field_size_y',             15.0)
        self.declare_parameter('survey_altitude_m',         4.0)
        self.declare_parameter('cruise_altitude_m',         3.0)
        self.declare_parameter('approach_altitude_m',       1.5)
        self.declare_parameter('cruise_speed_m_s',          2.0)
        self.declare_parameter('approach_speed_m_s',        0.4)
        self.declare_parameter('contact_descent_speed_m_s', 0.08)
        self.declare_parameter('pistil_length_m',           0.30)
        self.declare_parameter('tof_contact_m',             0.03)
        self.declare_parameter('brush_sweep_duration_s',    4.0)
        self.declare_parameter('observation_radius_m',      2.5)
        self.declare_parameter('strip_pitch_m',             2.0)
        self.declare_parameter('battery_full_minutes',     10.0)
        self.declare_parameter('battery_rtb_threshold_pct', 20.0)
        self.declare_parameter('battery_swap_duration_s',   2.0)

        self.did       = int(self.get_parameter('drone_id').value)
        self.n_drones  = int(self.get_parameter('n_drones').value)
        self.base_x    = float(self.get_parameter('base_x').value)
        self.base_y    = float(self.get_parameter('base_y').value)
        self.base_z    = float(self.get_parameter('base_z').value)
        self.Lx        = float(self.get_parameter('field_size_x').value)
        self.Ly        = float(self.get_parameter('field_size_y').value)
        self.survey_z  = float(self.get_parameter('survey_altitude_m').value)
        self.cruise_z  = float(self.get_parameter('cruise_altitude_m').value)
        self.approach_z = float(self.get_parameter('approach_altitude_m').value)
        self.v_cruise  = float(self.get_parameter('cruise_speed_m_s').value)
        self.v_appr    = float(self.get_parameter('approach_speed_m_s').value)
        self.v_contact = float(self.get_parameter('contact_descent_speed_m_s').value)
        self.pistil_L  = float(self.get_parameter('pistil_length_m').value)
        self.tof_c     = float(self.get_parameter('tof_contact_m').value)
        self.brush_dur = float(self.get_parameter('brush_sweep_duration_s').value)
        self.obs_r     = float(self.get_parameter('observation_radius_m').value)
        self.strip_p   = float(self.get_parameter('strip_pitch_m').value)
        self.batt_full_min   = float(self.get_parameter('battery_full_minutes').value)
        self.batt_rtb_thresh = float(self.get_parameter('battery_rtb_threshold_pct').value)
        self.batt_swap_dur   = float(self.get_parameter('battery_swap_duration_s').value)

        self.ns = f'drone_{self.did}'

        # ── Phase velocity limits (no leakage between phases) ──────
        self.limits = {
            TAKEOFF:            PhaseLimits(0.8, 1.2),
            SURVEY:             PhaseLimits(self.v_cruise, 1.0),
            RETURN_FROM_SURVEY: PhaseLimits(self.v_cruise, 1.0),
            LAND_SURVEY:        PhaseLimits(0.4, 0.8),
            GOTO_FLOWER:        PhaseLimits(self.v_cruise, 1.0),
            DESCEND:            PhaseLimits(self.v_appr, 0.24),
            BRUSH:              PhaseLimits(0.10, self.v_contact),   # slow xy for sweep
            ASCEND_GENTLE:      PhaseLimits(0.10, 0.15),             # gentle lift off
            RETREAT:            PhaseLimits(self.v_appr, 0.6),
            RTB:                PhaseLimits(self.v_cruise, 1.0),
            LAND_RTB:           PhaseLimits(0.4, 0.8),
            RTB_BATTERY:        PhaseLimits(self.v_cruise, 1.0),
            RECHARGING:         PhaseLimits(0.0, 0.0),
        }

        # ── PID gains (Stage 14 values) ────────────────────────────
        self.kp_xy = 1.2
        self.kp_z  = 1.5
        self.pos_tol = 0.40
        self.z_tol   = 0.20

        # ── Strip assignment (lawnmower band along x) ──────────────
        strip_w = self.Lx / max(1, self.n_drones)
        self.x_min = max(0.5, self.did * strip_w + 0.5)
        self.x_max = min(self.Lx - 0.5, (self.did + 1) * strip_w - 0.5)

        # ── State ─────────────────────────────────────────────────
        self.phase = IDLE
        self.pose  = None             # (x, y, z) latest
        self.day = -1
        self.targets = []             # bloom targets for this day
        self.observed_ids: set[int] = set()
        self.survey_wps: list[tuple] = []
        self.survey_wp_i = 0
        self.route: list[dict] = []   # staging buffer — replaced by _on_routes
        self.route_i = 0
        self.brush_t0 = 0.0
        self.phase_t0 = time.time()
        self.start_received = False
        self.route_received_for_day = -1
        # Active route snapshot — taken when entering GOTO_FLOWER from
        # AWAIT_ROUTE. All pollination phases (GOTO/DESCEND/BRUSH/RETREAT)
        # read from this snapshot, NOT from self.route. This insulates the
        # running mission from any mid-flight modifications to self.route
        # (which can happen if _on_day fires mid-pollination).
        self._active_route: list[dict] = []
        self._active_idx = 0
        self._active_target = None    # dict or None
        # Per-flower contact altitude (set on GOTO_FLOWER, used in BRUSH)
        self.contact_z = 0.0
        self._contact_t0 = 0.0
        self._contact_t0_set = False
        # Battery interrupt: when low, drone goes RTB_BATTERY → RECHARGING
        # → resume from self._active_idx. mission_complete tracks whether
        # the current RTB is the final one (route exhausted) or a battery
        # pit-stop (route still has work left).
        self._mission_complete = False
        self._swap_start_t = 0.0

        # Battery: 100% over battery_full_minutes of ACTIVE flight
        self.batt_pct = 100.0
        self.drain_per_s = 100.0 / max(60.0, self.batt_full_min * 60.0)
        self.last_batt_tick = time.time()

        # ── QoS ────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        # ── Publishers ─────────────────────────────────────────────
        self.pub_cmd      = self.create_publisher(Twist,   f'/model/{self.ns}/cmd_vel',           10)
        self.pub_obs      = self.create_publisher(String,  f'/survey/{self.ns}/observations',     10)
        self.pub_done     = self.create_publisher(Bool,    f'/swarm/survey_done/{self.ns}',       10)
        self.pub_state    = self.create_publisher(String,  f'/pollination/{self.ns}/state',       10)
        self.pub_pollin   = self.create_publisher(Int32,   f'/pollination/{self.ns}/pollinated',  10)
        self.pub_atbase   = self.create_publisher(Bool,    f'/pollination/{self.ns}/at_base',     10)
        self.pub_pose_out = self.create_publisher(PoseStamped, f'/pollination/{self.ns}/pose',    10)
        self.pub_batt     = self.create_publisher(Float32, f'/battery/{self.ns}',                 10)

        # ── Subscribers ────────────────────────────────────────────
        self.create_subscription(PoseStamped, f'/model/{self.ns}/pose',
                                 self._on_pose, sensor_qos)
        self.create_subscription(Bool,   '/swarm/survey_start',
                                 self._on_survey_start, 10)
        self.create_subscription(String, '/bloom/targets',
                                 self._on_targets, latched_qos)
        self.create_subscription(String, '/map_merger/routes',
                                 self._on_routes, 10)
        self.create_subscription(Int32,  '/bloom/current_day',
                                 self._on_day, latched_qos)
        self.create_subscription(Bool,   '/swarm/abort',
                                 self._on_abort, 10)

        # 20 Hz control loop (same as Stage 14)
        self.create_timer(0.05, self._loop)
        # 2 Hz state publication
        self.create_timer(0.5,  self._publish_status)

        self.get_logger().info(
            f"drone_controller[{self.did}] up [v3-survey-guard]. "
            f"strip x=[{self.x_min:.1f},{self.x_max:.1f}], "
            f"base=({self.base_x:.2f},{self.base_y:.2f}), "
            f"cruise_z={self.cruise_z}, survey_z={self.survey_z}")

    # ── Callbacks ──────────────────────────────────────────────────
    def _on_pose(self, m: PoseStamped):
        self.pose = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        # Re-publish on /pollination/* for the logger
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose = m.pose
        self.pub_pose_out.publish(out)

    def _on_day(self, m: Int32):
        new_day = int(m.data)
        if new_day == self.day:
            return
        prev_day = self.day
        self.day = new_day
        # Reset per-day flags. self.route is a staging buffer for _on_routes
        # and is safe to clear; the *active* route (self._active_route) is
        # owned by the running pollination cycle and is left alone if mid-mission.
        self.observed_ids.clear()
        self.route.clear()
        self.route_i = 0
        self.route_received_for_day = -1
        if self.phase == DAY_DONE:
            # Drone is parked — safe to reset everything including start_received
            self.start_received = False
            self.phase = IDLE
            self._active_route = []
            self._active_idx = 0
            self._active_target = None
        elif prev_day >= 0:
            # Day was advanced while drone is mid-mission. This usually means
            # the orchestrator decided too early that everyone was at base.
            # Keep start_received=True so _on_survey_start can't yank us into
            # TAKEOFF mid-pollination, and keep _active_route untouched so
            # the running cycle finishes its flowers.
            self.get_logger().warning(
                f"[D{self.did}] day advanced from {prev_day} to {new_day} "
                f"while in {PHASE_NAMES[self.phase]} (mid-mission). "
                f"Preserving active route + start_received to finish "
                f"current mission cycle.")

    def _on_targets(self, m: String):
        try:
            data = json.loads(m.data)
            self.targets = data.get('flowers', [])
        except Exception:
            pass

    def _on_survey_start(self, m: Bool):
        if not m.data or self.start_received:
            return
        # Belt-and-suspenders: even if start_received was somehow reset,
        # we refuse to switch into TAKEOFF if the drone is mid-mission.
        # An in-flight drone is one where phase is in any active flying state.
        # A stale survey_start from a prematurely-advanced day cannot
        # interrupt an active pollination cycle.
        BUSY = {TAKEOFF, SURVEY, RETURN_FROM_SURVEY, LAND_SURVEY,
                AWAIT_ROUTE, GOTO_FLOWER, DESCEND, BRUSH, ASCEND_GENTLE,
                RETREAT, RTB, LAND_RTB, RTB_BATTERY, RECHARGING}
        if self.phase in BUSY:
            self.get_logger().warning(
                f"[D{self.did}] survey_start arrived while in "
                f"{PHASE_NAMES[self.phase]} — IGNORING (mid-mission). "
                f"Will only honour next survey_start when drone is "
                f"IDLE or DAY_DONE.")
            return
        self.start_received = True
        self.observed_ids.clear()
        self._plan_lawnmower()
        self._enter(TAKEOFF)
        self.get_logger().info(
            f"[D{self.did}] survey_start — taking off, "
            f"{len(self.survey_wps)} survey waypoints")

    def _on_routes(self, m: String):
        try:
            data = json.loads(m.data)
            day = int(data.get('day', -1))
            if day == self.route_received_for_day:
                return
            routes = data.get('routes', {})
            mine = routes.get(str(self.did), [])
            self.route = list(mine)
            self.route_i = 0
            self.route_received_for_day = day
            self.get_logger().info(
                f"[D{self.did}] day {day} route received: {len(self.route)} flowers")
            # Activate the route snapshot immediately if waiting for it
            if self.phase == AWAIT_ROUTE:
                self._activate_route()
        except Exception as e:
            self.get_logger().warning(f"bad routes msg: {e}")

    def _activate_route(self):
        """Snapshot self.route into self._active_route for the running mission.
        From this moment until the mission ends (RTB), all pollination phases
        read from self._active_route — they ignore mutations to self.route."""
        self._active_route = list(self.route)   # COPY, not reference
        self._active_idx = 0
        self._mission_complete = False
        if self._active_route:
            self._active_target = self._active_route[0]
            self._enter(GOTO_FLOWER)
        else:
            self._active_target = None
            self._mission_complete = True
            self._enter(RTB)

    def _on_abort(self, m: Bool):
        if m.data and self.phase in ACTIVE_PHASES and self.phase not in (RTB, LAND_RTB):
            self.get_logger().warning(f"[D{self.did}] abort received — RTB")
            self._enter(RTB)

    # ── Survey planning ────────────────────────────────────────────
    def _plan_lawnmower(self):
        self.survey_wps.clear()
        z = self.survey_z
        n_strips = max(1, int((self.x_max - self.x_min) / self.strip_p))
        xs = [self.x_min + (i + 0.5) * (self.x_max - self.x_min) / n_strips
              for i in range(n_strips)]
        for i, x in enumerate(xs):
            if i % 2 == 0:
                self.survey_wps.append((x, 0.5,         z))
                self.survey_wps.append((x, self.Ly - 0.5, z))
            else:
                self.survey_wps.append((x, self.Ly - 0.5, z))
                self.survey_wps.append((x, 0.5,         z))
        self.survey_wp_i = 0

    # ── State helpers ──────────────────────────────────────────────
    def _enter(self, phase: int):
        if phase != self.phase:
            self.get_logger().info(
                f"[D{self.did}] {PHASE_NAMES[self.phase]} → {PHASE_NAMES[phase]}")
        self.phase = phase
        self.phase_t0 = time.time()

    def _send(self, vx: float, vy: float, vz: float):
        t = Twist()
        t.linear.x = float(vx); t.linear.y = float(vy); t.linear.z = float(vz)
        # Crucially leave angular = 0 (Stage 14 lesson: gz VelocityControl
        # is world-frame, do NOT rotate the velocity vector).
        self.pub_cmd.publish(t)

    def _goto(self, tx: float, ty: float, tz: float, lim: PhaseLimits):
        if self.pose is None:
            return
        ex = tx - self.pose[0]
        ey = ty - self.pose[1]
        ez = tz - self.pose[2]
        vx = clamp(self.kp_xy * ex, -lim.vmax_xy, lim.vmax_xy)
        vy = clamp(self.kp_xy * ey, -lim.vmax_xy, lim.vmax_xy)
        vz = clamp(self.kp_z  * ez, -lim.vmax_z,  lim.vmax_z)
        self._send(vx, vy, vz)

    def _at_xy(self, tx: float, ty: float) -> bool:
        return self.pose is not None and dist2(self.pose, (tx, ty)) < self.pos_tol

    def _at_z(self, tz: float) -> bool:
        return self.pose is not None and abs(self.pose[2] - tz) < self.z_tol

    def _at_z_tight(self, tz: float, tol: float = 0.05) -> bool:
        """Tight altitude check — drone must really be at this z, not just nearby.
        Used by BRUSH to confirm the drone actually descended to contact
        altitude rather than counting the descend_target_z (15 cm higher)
        as good enough."""
        return self.pose is not None and abs(self.pose[2] - tz) < tol

    def _battery_low(self) -> bool:
        """True if battery is below the RTB threshold and we're still mid-route."""
        return (self.batt_pct < self.batt_rtb_thresh
                and self._active_idx < len(self._active_route))

    # ── Main loop ──────────────────────────────────────────────────
    def _loop(self):
        # Battery drain (active phases only)
        now = time.time()
        dt = now - self.last_batt_tick
        self.last_batt_tick = now
        if self.phase in ACTIVE_PHASES:
            self.batt_pct = max(0.0, self.batt_pct - self.drain_per_s * dt)

        if self.pose is None:
            return

        # ───── per-phase logic ───────────────────────────────────
        if self.phase == IDLE or self.phase == DAY_DONE:
            self._send(0, 0, 0)
            return

        if self.phase == TAKEOFF:
            self._goto(self.base_x, self.base_y, self.survey_z, self.limits[TAKEOFF])
            if self._at_z(self.survey_z):
                self._enter(SURVEY)
            return

        if self.phase == SURVEY:
            if self.survey_wp_i >= len(self.survey_wps):
                self._enter(RETURN_FROM_SURVEY)
                return
            tx, ty, tz = self.survey_wps[self.survey_wp_i]
            self._goto(tx, ty, tz, self.limits[SURVEY])
            self._observe()
            if self._at_xy(tx, ty) and self._at_z(tz):
                self.survey_wp_i += 1
            if (now - self.phase_t0) > 600.0:
                self.get_logger().warning(f"[D{self.did}] survey timeout")
                self._enter(RETURN_FROM_SURVEY)
            return

        if self.phase == RETURN_FROM_SURVEY:
            self._goto(self.base_x, self.base_y, self.survey_z,
                       self.limits[RETURN_FROM_SURVEY])
            if self._at_xy(self.base_x, self.base_y):
                self._enter(LAND_SURVEY)
            return

        if self.phase == LAND_SURVEY:
            self._goto(self.base_x, self.base_y, self.base_z + 0.30,
                       self.limits[LAND_SURVEY])
            if self._at_z(self.base_z + 0.30):
                self._send(0, 0, 0)
                # Announce survey-complete
                msg = Bool(); msg.data = True
                for _ in range(3):
                    self.pub_done.publish(msg)
                self.get_logger().info(
                    f"[D{self.did}] survey done. observed {len(self.observed_ids)} flowers")
                self._enter(AWAIT_ROUTE)
            return

        if self.phase == AWAIT_ROUTE:
            # Hold position above pad until route arrives. If route is
            # already present (from a prior message) handle immediately.
            self._send(0, 0, 0)
            if self.route_received_for_day == self.day:
                if self.route:
                    self._enter(GOTO_FLOWER)
                else:
                    self._enter(RTB)
            return

        # ── Battery interrupt ─────────────────────────────────────────
        # Active flight phases check the battery; if low, divert to
        # RTB_BATTERY (resume mission after swap). Don't divert if we're
        # already heading back or recharging.
        if (self.phase in (GOTO_FLOWER, RETREAT)
                and self._battery_low()):
            self.get_logger().warning(
                f"[D{self.did}] battery {self.batt_pct:.1f}% < "
                f"{self.batt_rtb_thresh:.0f}% — pausing mission for swap. "
                f"Will resume from flower {self._active_idx + 1}/"
                f"{len(self._active_route)}")
            self._enter(RTB_BATTERY)
            return

        if self.phase == GOTO_FLOWER:
            t = self._active_target
            if t is None:
                self.get_logger().info(
                    f"[D{self.did}] GOTO_FLOWER: no active target → RTB")
                self._mission_complete = True
                self._enter(RTB)
                return
            tx, ty = float(t['x']), float(t['y'])
            self._goto(tx, ty, self.cruise_z, self.limits[GOTO_FLOWER])
            if self._at_xy(tx, ty) and self._at_z(self.cruise_z):
                # Latch this flower's contact altitude — geometry from JSON
                head_z = float(t.get('z', t.get('head_z', 1.7)))
                self.contact_z = head_z + self.pistil_L + self.tof_c
                self.get_logger().info(
                    f"[D{self.did}] over flower {int(t['flower_id'])} "
                    f"({self._active_idx + 1}/{len(self._active_route)}), "
                    f"head_z={head_z:.2f}, contact_z={self.contact_z:.2f}")
                self._enter(DESCEND)
            return

        if self.phase == DESCEND:
            t = self._active_target
            if t is None:
                self._enter(RTB)
                return
            tx, ty = float(t['x']), float(t['y'])
            # Stop 15 cm ABOVE contact_z; BRUSH will slowly close that gap.
            descend_target_z = self.contact_z + 0.15
            self._goto(tx, ty, descend_target_z, self.limits[DESCEND])
            if self._at_xy(tx, ty) and self._at_z(descend_target_z):
                self._enter(BRUSH)
            return

        if self.phase == BRUSH:
            # Real pollination motion:
            # 1) Drone slowly descends from descend_target_z to contact_z
            #    at v_contact (0.08 m/s) — about 1.9 s of descent
            # 2) Once within 5 cm of contact_z (TIGHT tolerance — not the
            #    20 cm loose z_tol that fired immediately on entry), the
            #    brush dwell timer starts
            # 3) During dwell, drone executes a 5 cm radius circular sweep
            #    over the flower for visible pollen collection
            # 4) After brush_dur seconds in contact, pollination logged
            #    and the gentle ascent begins.
            t = self._active_target
            if t is None:
                self._enter(RTB)
                return
            fx, fy = float(t['x']), float(t['y'])

            # 5 cm radius circle, completing two revolutions during brush_dur
            sweep_radius = 0.05
            n_revolutions = 2.0
            elapsed_phase = now - self.phase_t0
            theta = 2.0 * math.pi * n_revolutions * (
                elapsed_phase / max(self.brush_dur, 1e-3))
            tx = fx + sweep_radius * math.cos(theta)
            ty = fy + sweep_radius * math.sin(theta)
            self._goto(tx, ty, self.contact_z, self.limits[BRUSH])

            # TIGHT contact: must be within 5 cm of contact_z, not 20 cm
            in_contact = self._at_z_tight(self.contact_z, tol=0.05)
            if in_contact and not self._contact_t0_set:
                self._contact_t0 = now
                self._contact_t0_set = True
                self.get_logger().info(
                    f"[D{self.did}] ⤓ brush in contact with flower "
                    f"{int(t['flower_id'])} @ z={self.pose[2]:.2f}, "
                    f"starting {self.brush_dur:.1f}s sweep")
            elapsed_in_contact = (
                (now - self._contact_t0) if self._contact_t0_set else 0.0)

            if elapsed_in_contact >= self.brush_dur:
                fid = int(t['flower_id'])
                msg = Int32(); msg.data = fid
                self.pub_pollin.publish(msg)
                self.get_logger().info(
                    f"[D{self.did}] ✿ pollinated flower {fid} "
                    f"({self._active_idx + 1}/{len(self._active_route)}) "
                    f"@ contact_z={self.contact_z:.2f}, "
                    f"batt={self.batt_pct:.1f}%")
                self._active_idx += 1
                self._contact_t0_set = False
                self._enter(ASCEND_GENTLE)
            elif (now - self.phase_t0) > (self.brush_dur + 12.0):
                # Couldn't reach tight contact in 16 s — geometry off,
                # skip this flower so the rest of the route can proceed.
                self.get_logger().warning(
                    f"[D{self.did}] brush timeout on flower "
                    f"{int(t['flower_id'])} — could not reach contact_z="
                    f"{self.contact_z:.2f} (pose z={self.pose[2]:.2f}), "
                    f"skipping")
                self._active_idx += 1
                self._contact_t0_set = False
                self._enter(ASCEND_GENTLE)
            return

        if self.phase == ASCEND_GENTLE:
            # Slow vertical lift to (head_z + 0.50 m) at ~0.15 m/s so the
            # drone visibly "lifts off" gently before the normal climb.
            # The active target is the flower we just brushed.
            # Even after a brush timeout we use the same coords so the
            # drone always leaves straight up over the flower.
            t = self._active_target
            if t is None:
                self._enter(RETREAT)
                return
            tx, ty = float(t['x']), float(t['y'])
            head_z = float(t.get('z', t.get('head_z', 1.7)))
            gentle_target_z = head_z + 0.50
            self._goto(tx, ty, gentle_target_z, self.limits[ASCEND_GENTLE])
            if self._at_z(gentle_target_z):
                self._enter(RETREAT)
            return

        if self.phase == RETREAT:
            # Normal-speed climb to cruise altitude over the flower, then
            # decide: next flower or end-of-route. CRITICAL: this is the
            # transition that drives the per-flower iteration. If it picks
            # the wrong branch, the drone visits one flower then RTBs.
            t = self._active_target
            if t is not None:
                tx, ty = float(t['x']), float(t['y'])
            else:
                tx, ty = self.base_x, self.base_y
            self._goto(tx, ty, self.cruise_z, self.limits[RETREAT])
            if self._at_z(self.cruise_z):
                remaining = len(self._active_route) - self._active_idx
                if remaining <= 0:
                    self.get_logger().info(
                        f"[D{self.did}] route complete "
                        f"({self._active_idx}/{len(self._active_route)} "
                        f"flowers visited) — heading to base")
                    self._active_target = None
                    self._mission_complete = True
                    self._enter(RTB)
                else:
                    next_target = self._active_route[self._active_idx]
                    self.get_logger().info(
                        f"[D{self.did}] retreat done; advancing to flower "
                        f"{int(next_target['flower_id'])} "
                        f"({self._active_idx + 1}/{len(self._active_route)}), "
                        f"{remaining} remaining")
                    self._active_target = next_target
                    self._enter(GOTO_FLOWER)
            return

        if self.phase == RTB_BATTERY:
            # Battery RTB — like RTB but the drone will recharge, not exit
            self._goto(self.base_x, self.base_y, self.cruise_z,
                       self.limits[RTB_BATTERY])
            if self._at_xy(self.base_x, self.base_y) and self._at_z(self.cruise_z):
                # Land for swap (use LAND_RTB target but stay marked
                # battery-RTB by clearing on LAND_RTB completion)
                self._enter(LAND_RTB)
            return

        if self.phase == RTB:
            self._goto(self.base_x, self.base_y, self.cruise_z, self.limits[RTB])
            if self._at_xy(self.base_x, self.base_y):
                self._enter(LAND_RTB)
            return

        if self.phase == LAND_RTB:
            self._goto(self.base_x, self.base_y, self.base_z + 0.30,
                       self.limits[LAND_RTB])
            if self._at_z(self.base_z + 0.30):
                self._send(0, 0, 0)
                # at_base is now a level signal published every 0.5s by
                # _publish_status — no need to publish explicitly here.
                # Branch: if the active route still has flowers, this is a
                # battery pit-stop → RECHARGING. Otherwise mission complete
                # for the day → DAY_DONE.
                remaining = len(self._active_route) - self._active_idx
                if remaining > 0 and not self._mission_complete:
                    self.get_logger().info(
                        f"[D{self.did}] landed for battery swap "
                        f"({self._active_idx}/{len(self._active_route)} done, "
                        f"{remaining} remaining). Swap takes "
                        f"{self.batt_swap_dur:.1f}s.")
                    self._swap_start_t = time.time()
                    self._enter(RECHARGING)
                else:
                    self.get_logger().info(
                        f"[D{self.did}] mission for day {self.day} complete. "
                        f"At base. pollinated "
                        f"{self._active_idx}/{len(self._active_route)}")
                    self._enter(DAY_DONE)
            return

        if self.phase == RECHARGING:
            # Stationary swap — hold position, then 100% battery and resume
            # the mission from where we left off (_active_idx points at the
            # next un-pollinated flower, _active_target is that flower).
            self._send(0, 0, 0)
            elapsed_swap = time.time() - self._swap_start_t
            if elapsed_swap >= self.batt_swap_dur:
                self.batt_pct = 100.0
                # Pick up the next un-pollinated flower
                if self._active_idx < len(self._active_route):
                    next_target = self._active_route[self._active_idx]
                    self._active_target = next_target
                    self.get_logger().info(
                        f"[D{self.did}] battery swapped → 100%. Resuming "
                        f"mission at flower {int(next_target['flower_id'])} "
                        f"({self._active_idx + 1}/{len(self._active_route)})")
                    self._enter(GOTO_FLOWER)
                else:
                    # Route happened to finish exactly at the swap — done
                    self._mission_complete = True
                    self._enter(DAY_DONE)
            return

    # ── 2 Hz status & battery ─────────────────────────────────────
    def _publish_status(self):
        # State string for logger / orchestrator
        s = String(); s.data = PHASE_NAMES[self.phase]
        self.pub_state.publish(s)
        # Battery
        b = Float32(); b.data = float(self.batt_pct)
        self.pub_batt.publish(b)
        # at_base is a 2 Hz LEVEL signal: True iff drone is currently parked
        # (in IDLE or DAY_DONE). The orchestrator reads the latest level to
        # decide when all drones have finished a day — so a drone that's
        # flying is RELIABLY reported as not-at-base. The old design
        # (publish True once on land) was prone to stale-True races where
        # the orchestrator would see at_base=True from drones still in IDLE
        # at the start of the mission and advance the day prematurely.
        m = Bool()
        m.data = self.phase in (IDLE, DAY_DONE)
        self.pub_atbase.publish(m)

    # ── Simulated YOLO observer ────────────────────────────────────
    def _observe(self):
        if not self.targets or self.pose is None:
            return
        px, py, _ = self.pose
        new_obs = []
        for f in self.targets:
            fid = int(f['flower_id'])
            if fid in self.observed_ids:
                continue
            d = math.hypot(float(f['x']) - px, float(f['y']) - py)
            if d <= self.obs_r:
                self.observed_ids.add(fid)
                new_obs.append({
                    'flower_id': fid,
                    'x': float(f['x']), 'y': float(f['y']),
                    'z': float(f.get('z', f.get('head_z', 1.7))),
                    'state': f.get('state', 'prime_target'),
                    'observed_by': self.did,
                    'observed_at': round(time.time(), 3),
                })
        if new_obs:
            msg = String(); msg.data = json.dumps(
                {'drone_id': self.did, 'observations': new_obs},
                separators=(',', ':'))
            self.pub_obs.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    n = DroneController()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
