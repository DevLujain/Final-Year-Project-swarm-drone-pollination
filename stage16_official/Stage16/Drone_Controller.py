#!/usr/bin/env python3
"""
Drone_Controller.py — Stage 16 v13 (no-cheat)
==============================================
One instance per drone. The drone knows NOTHING about flowers except
what its own sensors produce:

  inputs (sensors / own perception):
    /model/drone_N/pose             Gazebo pose feedback
    /perception/drone_N/snapshot    its OWN perception node's cluster map
                                    (open flowers + bud census)
    /drone_N/tof_brush/scan         brush-tip TOF (LaserScan)

  inputs (mission control — no flower data):
    /swarm/survey_start  /bloom/current_day  /map_merger/routes  /swarm/abort

  outputs:
    /survey/drone_N/observations    ENRICHED MAP: TOF-confirmed open
                                    flowers w/ measured z + bud census
    /swarm/survey_done/drone_N, /pollination/...state/pollinated/at_base/pose,
    /battery/drone_N, /model/drone_N/cmd_vel

PER DAY:
  TAKEOFF → SURVEY (lawnmower banded to own x-strip, perception runs)
  → HEIGHT_PROBE (wait one snapshot period; for each open cluster in own
    strip not already height-known: fly over at probe_top_z, descend
    slowly watching TOF; valid return → measured_z = z − 0.325 − tof;
    no return by probe_bottom_z → PHANTOM, dropped)
  → publish enriched map → RETURN_TO_BASE → LAND_SURVEY → survey_done
  → AWAIT_ROUTE → GOTO/DESCEND/BRUSH (TOF-latched contact) per flower
  → RTB → DAY_DONE.  Battery low mid-route → RTB_BATTERY → RECHARGING → resume.

Cross-day: measured heights are remembered positionally (≤0.4 m), so
already-probed flowers are not re-probed on later days.

Boot marker: [v13-nocheat-perception]
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, Float32, String


# ── Phases ────────────────────────────────────────────────────────
(IDLE, TAKEOFF, SURVEY, HEIGHT_PROBE, RETURN_TO_BASE, LAND_SURVEY,
 AWAIT_ROUTE, GOTO_FLOWER, DESCEND, BRUSH, ASCEND_GENTLE,
 RETREAT, RTB, LAND_RTB, DAY_DONE,
 RTB_BATTERY, RECHARGING) = range(17)

PHASE_NAMES = {
    IDLE: 'IDLE', TAKEOFF: 'TAKEOFF', SURVEY: 'SURVEY',
    HEIGHT_PROBE: 'HEIGHT_PROBE',
    RETURN_TO_BASE: 'RETURN_TO_BASE', LAND_SURVEY: 'LAND_SURVEY',
    AWAIT_ROUTE: 'AWAIT_ROUTE', GOTO_FLOWER: 'GOTO_FLOWER',
    DESCEND: 'DESCEND', BRUSH: 'BRUSH', ASCEND_GENTLE: 'ASCEND_GENTLE',
    RETREAT: 'RETREAT', RTB: 'RTB', LAND_RTB: 'LAND_RTB',
    DAY_DONE: 'DAY_DONE', RTB_BATTERY: 'RTB_BATTERY',
    RECHARGING: 'RECHARGING',
}

ACTIVE_PHASES = {TAKEOFF, SURVEY, HEIGHT_PROBE, RETURN_TO_BASE, LAND_SURVEY,
                 GOTO_FLOWER, DESCEND, BRUSH, ASCEND_GENTLE,
                 RETREAT, RTB, LAND_RTB, RTB_BATTERY}

BRUSH_TIP_DZ = 0.325   # brush tip sits 0.325 m below the drone CG


@dataclass
class Lim:
    vmax_xy: float
    vmax_z:  float


def clamp(v, lo, hi): return max(lo, min(hi, v))
def dist2(a, b):      return math.hypot(a[0]-b[0], a[1]-b[1])


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('drone_id',                   0)
        self.declare_parameter('n_drones',                   2)
        self.declare_parameter('base_x',                     7.5)
        self.declare_parameter('base_y',                    -1.0)
        self.declare_parameter('base_z',                     0.10)
        self.declare_parameter('field_size_x',              15.0)
        self.declare_parameter('field_size_y',              15.0)
        self.declare_parameter('survey_altitude_m',          4.0)
        self.declare_parameter('cruise_altitude_m',          3.0)
        self.declare_parameter('approach_altitude_m',        1.5)
        self.declare_parameter('cruise_speed_m_s',           2.0)
        self.declare_parameter('approach_speed_m_s',         0.4)
        self.declare_parameter('contact_descent_speed_m_s',  0.08)
        self.declare_parameter('pistil_length_m',            0.30)
        self.declare_parameter('tof_contact_m',              0.03)
        self.declare_parameter('brush_sweep_duration_s',     4.0)
        self.declare_parameter('strip_pitch_m',              2.0)
        self.declare_parameter('strip_overlap_m',            0.6)
        self.declare_parameter('battery_full_minutes',      10.0)
        self.declare_parameter('battery_rtb_threshold_pct', 20.0)
        self.declare_parameter('battery_swap_duration_s',    2.0)
        # Height probe
        self.declare_parameter('probe_top_z_m',              2.60)
        self.declare_parameter('probe_bottom_z_m',           1.80)
        self.declare_parameter('tof_valid_max_m',             0.45)
        self.declare_parameter('probe_center_tol_m',          0.15)
        self.declare_parameter('probe_center_timeout_s',      3.0)
        self.declare_parameter('snapshot_wait_s',             6.0)
        self.declare_parameter('landmark_match_radius_m',     0.40)
        self.declare_parameter('max_altitude_m',              5.50)

        gp = lambda n: self.get_parameter(n).value
        self.did      = int(gp('drone_id'))
        self.n_drones = int(gp('n_drones'))
        self.base_x   = float(gp('base_x'))
        self.base_y   = float(gp('base_y'))
        self.base_z   = float(gp('base_z'))
        self.Lx       = float(gp('field_size_x'))
        self.Ly       = float(gp('field_size_y'))
        self.survey_z = float(gp('survey_altitude_m'))
        self.cruise_z = float(gp('cruise_altitude_m'))
        self.v_cruise = float(gp('cruise_speed_m_s'))
        self.v_appr   = float(gp('approach_speed_m_s'))
        self.v_contact= float(gp('contact_descent_speed_m_s'))
        self.pistil_L = float(gp('pistil_length_m'))
        self.tof_c    = float(gp('tof_contact_m'))
        self.brush_dur= float(gp('brush_sweep_duration_s'))
        self.strip_p  = float(gp('strip_pitch_m'))
        self.strip_ov = float(gp('strip_overlap_m'))
        self.batt_full_min   = float(gp('battery_full_minutes'))
        self.batt_rtb_thresh = float(gp('battery_rtb_threshold_pct'))
        self.batt_swap_dur   = float(gp('battery_swap_duration_s'))
        self.probe_top    = float(gp('probe_top_z_m'))
        self.probe_bottom = float(gp('probe_bottom_z_m'))
        self.tof_valid_max = float(gp('tof_valid_max_m'))
        self.probe_tol    = float(gp('probe_center_tol_m'))
        self.probe_center_to = float(gp('probe_center_timeout_s'))
        self.snap_wait    = float(gp('snapshot_wait_s'))
        self.lm_match_r   = float(gp('landmark_match_radius_m'))
        self.max_z        = float(gp('max_altitude_m'))

        self.ns = f'drone_{self.did}'

        # Per-phase velocity caps
        self.limits = {
            TAKEOFF:        Lim(0.8,  1.2),
            SURVEY:         Lim(self.v_cruise, 1.0),
            HEIGHT_PROBE:   Lim(self.v_cruise, 1.0),
            RETURN_TO_BASE: Lim(self.v_cruise, 1.0),
            LAND_SURVEY:    Lim(0.4,  0.8),
            GOTO_FLOWER:    Lim(self.v_cruise, 1.0),
            DESCEND:        Lim(self.v_appr, 0.24),
            BRUSH:          Lim(0.10, 0.20),
            ASCEND_GENTLE:  Lim(0.10, 0.15),
            RETREAT:        Lim(self.v_appr, 0.6),
            RTB:            Lim(self.v_cruise, 1.0),
            LAND_RTB:       Lim(0.4,  0.8),
            RTB_BATTERY:    Lim(self.v_cruise, 1.0),
            RECHARGING:     Lim(0.0,  0.0),
        }

        # PID
        self.kp_xy   = 1.2
        self.kp_z    = 1.5
        self.pos_tol = 0.40
        self.z_tol   = 0.20

        # Own x-strip (survey band + probe ownership)
        strip_w = self.Lx / max(1, self.n_drones)
        self.strip_x0 = self.did * strip_w
        self.strip_x1 = (self.did + 1) * strip_w

        # ── State (every var used anywhere is declared here) ──────
        self.phase   = IDLE
        self.pose    = None
        self.day     = -1
        self.phase_t0 = time.time()
        self.start_received = False
        # perception snapshot (latest)
        self.snap_obs:  list = []      # open clusters (perception)
        self.snap_buds: list = []      # bud census (perception)
        self.snap_ts = 0.0
        # TOF
        self.tof_range = None          # latest valid min range or None
        # survey
        self.survey_wps: list = []
        self.survey_wp_i = 0
        # height probe
        self.hp_route: list = []       # [{'fid','x','y'}]
        self.hp_idx = 0
        self.hp_mode = 'GOING'         # GOING | DESCENDING | CLIMBING
        self.hp_going_t0 = 0.0
        self.hp_measured: dict = {}    # fid -> measured_z (this day)
        self.hp_planned = False
        self.n_phantoms = 0
        # persistent cross-day height memory: [(x, y, z)]
        self.known_heights: list = []
        # pollination route
        self.route: list = []
        self.route_received_for_day = -1
        self._active_route: list = []
        self._active_idx = 0
        self._active_target = None
        self.contact_z = 0.0
        self._contact_t0 = 0.0
        self._contact_t0_set = False
        self._contact_hold_z = None
        self._mission_complete = False
        self._swap_start_t = 0.0

        # Battery
        self.batt_pct = 100.0
        self.drain_per_s = 100.0 / max(60.0, self.batt_full_min * 60.0)
        self.last_batt_tick = time.time()

        # QoS
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)

        # Publishers
        self.pub_cmd      = self.create_publisher(Twist,   f'/model/{self.ns}/cmd_vel',          10)
        self.pub_obs      = self.create_publisher(String,  f'/survey/{self.ns}/observations',    10)
        self.pub_done     = self.create_publisher(Bool,    f'/swarm/survey_done/{self.ns}',      10)
        self.pub_state    = self.create_publisher(String,  f'/pollination/{self.ns}/state',      10)
        self.pub_pollin   = self.create_publisher(Int32,   f'/pollination/{self.ns}/pollinated', 10)
        self.pub_atbase   = self.create_publisher(Bool,    f'/pollination/{self.ns}/at_base',    10)
        self.pub_pose_out = self.create_publisher(PoseStamped, f'/pollination/{self.ns}/pose',   10)
        self.pub_batt     = self.create_publisher(Float32, f'/battery/{self.ns}',                10)

        # Subscribers — NOTE: no /bloom/targets, no /bloom/all_flowers.
        # The only flower knowledge enters through OUR OWN perception.
        self.create_subscription(PoseStamped, f'/model/{self.ns}/pose',
                                 self._on_pose, sensor_qos)
        self.create_subscription(String, f'/perception/{self.ns}/snapshot',
                                 self._on_snapshot, 10)
        self.create_subscription(LaserScan, f'/{self.ns}/tof_brush/scan',
                                 self._on_tof, sensor_qos)
        self.create_subscription(Bool,   '/swarm/survey_start',
                                 self._on_survey_start, 10)
        self.create_subscription(String, '/map_merger/routes',
                                 self._on_routes, 10)
        self.create_subscription(Int32,  '/bloom/current_day',
                                 self._on_day, latched_qos)
        self.create_subscription(Bool,   '/swarm/abort',
                                 self._on_abort, 10)

        self.create_timer(0.05, self._loop)
        self.create_timer(0.5,  self._publish_status)
        self.create_timer(4.0,  self._stream_provisional)

        self.get_logger().info(
            f"[D{self.did}] up [v14-live-stream]. "
            f"base=({self.base_x:.2f},{self.base_y:.2f}) "
            f"strip x=[{self.strip_x0:.1f},{self.strip_x1:.1f}] "
            f"survey_z={self.survey_z} probe={self.probe_top}->{self.probe_bottom}")

    # ── Callbacks ─────────────────────────────────────────────────
    def _on_pose(self, m: PoseStamped):
        self.pose = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.pose = m.pose
        self.pub_pose_out.publish(out)

    def _on_snapshot(self, m: String):
        try:
            data = json.loads(m.data)
            self.snap_obs  = list(data.get('observations', []))
            self.snap_buds = list(data.get('bud_observations', []))
            self.snap_ts   = float(data.get('snapshot_ts', time.time()))
        except Exception:
            pass

    def _on_tof(self, m: LaserScan):
        best = None
        for r in m.ranges:
            if r is None: continue
            if math.isinf(r) or math.isnan(r): continue
            if m.range_min <= r <= min(m.range_max, self.tof_valid_max):
                if best is None or r < best:
                    best = r
        self.tof_range = best          # None when nothing in beam

    def _on_day(self, m: Int32):
        new_day = int(m.data)
        if new_day == self.day:
            return
        self.day = new_day
        self.route = []
        self.route_received_for_day = -1
        self.hp_measured = {}
        self.hp_planned = False
        if self.phase == DAY_DONE:
            self.start_received = False
            self.phase = IDLE
            self._active_route = []
            self._active_idx = 0
            self._active_target = None

    def _on_survey_start(self, m: Bool):
        if not m.data or self.start_received:
            return
        if self.phase not in (IDLE, DAY_DONE):
            return
        self.start_received = True
        self._plan_survey()
        self.hp_planned = False
        self.hp_measured = {}
        self.n_phantoms = 0
        self._enter(TAKEOFF)
        self.get_logger().info(
            f"[D{self.did}] survey_start day {self.day} → TAKEOFF, "
            f"{len(self.survey_wps)} survey waypoints "
            f"(perception will discover the flowers)")

    def _on_routes(self, m: String):
        try:
            data = json.loads(m.data)
            day  = int(data.get('day', -1))
            if day == self.route_received_for_day:
                return
            self.route = list(data.get('routes', {}).get(str(self.did), []))
            self.route_received_for_day = day
            self.get_logger().info(
                f"[D{self.did}] day {day} route: {len(self.route)} flowers")
            if self.phase == AWAIT_ROUTE:
                self._activate_route()
        except Exception as e:
            self.get_logger().warning(f"[D{self.did}] bad routes: {e}")

    def _activate_route(self):
        self._active_route = list(self.route)
        self._active_idx   = 0
        self._mission_complete = False
        if self._active_route:
            self._active_target = self._active_route[0]
            self._enter(GOTO_FLOWER)
        else:
            self._active_target = None
            self._mission_complete = True
            self._enter(RTB)

    def _on_abort(self, m: Bool):
        if m.data and self.phase in ACTIVE_PHASES \
                and self.phase not in (RTB, LAND_RTB):
            self._enter(RTB)

    # ── Survey planning (banded lawnmower, own strip + overlap) ───
    def _plan_survey(self):
        self.survey_wps = []
        z = self.survey_z
        x0 = max(0.5, self.strip_x0 - (self.strip_ov if self.did > 0 else 0.0))
        x1 = min(self.Lx - 0.5,
                 self.strip_x1 + (self.strip_ov if self.did < self.n_drones-1 else 0.0))
        n = max(1, int(round((x1 - x0) / self.strip_p)))
        xs = [x0 + (i + 0.5) * (x1 - x0) / n for i in range(n)]
        for i, x in enumerate(xs):
            if i % 2 == 0:
                self.survey_wps.append((x, 0.5,         z))
                self.survey_wps.append((x, self.Ly-0.5, z))
            else:
                self.survey_wps.append((x, self.Ly-0.5, z))
                self.survey_wps.append((x, 0.5,         z))
        self.survey_wp_i = 0

    # ── Height-probe planning ─────────────────────────────────────
    def _known_height_at(self, x, y):
        for (kx, ky, kz) in self.known_heights:
            if math.hypot(x - kx, y - ky) <= self.lm_match_r:
                return kz
        return None

    def _plan_probe(self):
        """Take the LATEST perception snapshot; probe every open cluster
        in our own x-strip whose height we don't already know."""
        self.hp_route = []
        n_known = 0
        for o in self.snap_obs:
            try:
                fid = int(o['flower_id'])
                x, y = float(o['x']), float(o['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if not (self.strip_x0 <= x < self.strip_x1):
                continue
            kz = self._known_height_at(x, y)
            if kz is not None:
                self.hp_measured[fid] = kz
                n_known += 1
                continue
            self.hp_route.append({'fid': fid, 'x': x, 'y': y})
        # nearest-first ordering from current pose
        if self.pose is not None and self.hp_route:
            ordered, cur, rem = [], (self.pose[0], self.pose[1]), list(self.hp_route)
            while rem:
                nxt = min(rem, key=lambda f: math.hypot(f['x']-cur[0], f['y']-cur[1]))
                ordered.append(nxt); cur = (nxt['x'], nxt['y']); rem.remove(nxt)
            self.hp_route = ordered
        self.hp_idx = 0
        self.hp_mode = 'GOING'
        self.hp_going_t0 = time.time()
        self.hp_planned = True
        self.get_logger().info(
            f"[D{self.did}] HEIGHT_PROBE plan: {len(self.hp_route)} new cluster(s) "
            f"to probe, {n_known} height(s) known from previous days, "
            f"{len(self.snap_buds)} bud(s) in census")

    # ── Enriched map ──────────────────────────────────────────────
    def _publish_enriched_map(self):
        """TOF-confirmed open flowers (with measured z) + bud census.
        Phantom clusters (no TOF return) are DROPPED — they never reach
        routing. Buds are display/memory only."""
        obs = []
        for o in self.snap_obs:
            try:
                fid = int(o['flower_id'])
                x, y = float(o['x']), float(o['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if not (self.strip_x0 <= x < self.strip_x1):
                continue
            mz = self.hp_measured.get(fid)
            if mz is None:
                continue   # phantom or unprobed → not trusted
            obs.append({'flower_id': fid,
                        'x': round(x, 3), 'y': round(y, 3),
                        'z': round(float(mz), 3),
                        'state': 'open',
                        'observed_by': self.did,
                        'observed_at': round(time.time(), 3)})
        buds = []
        for i, b in enumerate(self.snap_buds):
            try:
                bx, by = float(b['x']), float(b['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if not (self.strip_x0 <= bx < self.strip_x1):
                continue
            buds.append({'bud_id': 900000 + self.did*10000 + i,
                         'x': round(bx, 3), 'y': round(by, 3),
                         'state': 'closed',
                         'observed_by': self.did})
        msg = String()
        msg.data = json.dumps({'drone_id': self.did,
                               'observations': obs,
                               'bud_observations': buds},
                              separators=(',', ':'))
        for _ in range(3):
            self.pub_obs.publish(msg)
        self.get_logger().info(
            f"[D{self.did}] enriched map: {len(obs)} TOF-confirmed open, "
            f"{len(buds)} closed bud(s), {self.n_phantoms} phantom(s) dropped")

    # ── Helpers ───────────────────────────────────────────────────
    def _enter(self, phase):
        if phase != self.phase:
            self.get_logger().info(
                f"[D{self.did}] {PHASE_NAMES[self.phase]} → {PHASE_NAMES[phase]}")
        self.phase = phase
        self.phase_t0 = time.time()

    def _send(self, vx, vy, vz):
        t = Twist()
        t.linear.x = float(vx); t.linear.y = float(vy); t.linear.z = float(vz)
        self.pub_cmd.publish(t)

    def _goto(self, tx, ty, tz, lim: Lim):
        if self.pose is None: return
        vx = clamp(self.kp_xy*(tx-self.pose[0]), -lim.vmax_xy, lim.vmax_xy)
        vy = clamp(self.kp_xy*(ty-self.pose[1]), -lim.vmax_xy, lim.vmax_xy)
        vz = clamp(self.kp_z *(tz-self.pose[2]), -lim.vmax_z,  lim.vmax_z)
        self._send(vx, vy, vz)

    def _at_xy(self, tx, ty, tol=None):
        tol = self.pos_tol if tol is None else tol
        return self.pose is not None and dist2(self.pose, (tx,ty)) < tol

    def _at_z(self, tz):
        return self.pose is not None and abs(self.pose[2]-tz) < self.z_tol

    def _at_z_tight(self, tz, tol=0.05):
        return self.pose is not None and abs(self.pose[2]-tz) < tol

    def _battery_low(self):
        return (self.batt_pct < self.batt_rtb_thresh
                and self._active_idx < len(self._active_route))

    # ── Main loop ─────────────────────────────────────────────────
    def _loop(self):
        now = time.time()
        dt  = now - self.last_batt_tick
        self.last_batt_tick = now
        if self.phase in ACTIVE_PHASES:
            self.batt_pct = max(0.0, self.batt_pct - self.drain_per_s*dt)

        if self.pose is None:
            return

        # Safety ceiling
        if self.pose[2] > self.max_z:
            self._send(0.0, 0.0, -1.0)
            return

        # ── IDLE / DAY_DONE ──────────────────────────────────────
        if self.phase in (IDLE, DAY_DONE):
            self._send(0, 0, 0)
            return

        # ── TAKEOFF ──────────────────────────────────────────────
        if self.phase == TAKEOFF:
            self._goto(self.base_x, self.base_y, self.survey_z,
                       self.limits[TAKEOFF])
            if self._at_z(self.survey_z):
                self._enter(SURVEY)
            return

        # ── SURVEY (perception node watches the camera meanwhile) ─
        if self.phase == SURVEY:
            if self.survey_wp_i >= len(self.survey_wps):
                self._enter(HEIGHT_PROBE)
                return
            tx, ty, tz = self.survey_wps[self.survey_wp_i]
            self._goto(tx, ty, tz, self.limits[SURVEY])
            if self._at_xy(tx, ty) and self._at_z(tz):
                self.survey_wp_i += 1
            if (now - self.phase_t0) > 600.0:
                self.get_logger().warning(f"[D{self.did}] survey timeout")
                self._enter(HEIGHT_PROBE)
            return

        # ── HEIGHT_PROBE ─────────────────────────────────────────
        if self.phase == HEIGHT_PROBE:
            # 1) wait one snapshot period so the cluster map is fresh
            if not self.hp_planned:
                self._goto(self.pose[0], self.pose[1], self.probe_top,
                           self.limits[HEIGHT_PROBE])
                if (now - self.phase_t0) >= self.snap_wait:
                    self._plan_probe()
                return
            # 2) route exhausted → publish map, head home
            if self.hp_idx >= len(self.hp_route):
                self._publish_enriched_map()
                self._enter(RETURN_TO_BASE)
                return
            t = self.hp_route[self.hp_idx]
            tx, ty = t['x'], t['y']

            if self.hp_mode == 'GOING':
                self._goto(tx, ty, self.probe_top, self.limits[HEIGHT_PROBE])
                centered = self._at_xy(tx, ty, tol=self.probe_tol) \
                           and self._at_z(self.probe_top)
                fallback = (now - self.hp_going_t0) > self.probe_center_to \
                           and self._at_xy(tx, ty) and self._at_z(self.probe_top)
                if centered or fallback:
                    self.hp_mode = 'DESCENDING'
                return

            if self.hp_mode == 'DESCENDING':
                self._goto(tx, ty, self.probe_bottom, self.limits[HEIGHT_PROBE])
                if self.tof_range is not None:
                    mz = self.pose[2] - BRUSH_TIP_DZ - self.tof_range
                    self.hp_measured[t['fid']] = mz
                    self.known_heights.append((tx, ty, mz))
                    self.get_logger().info(
                        f"[D{self.did}] probe F{t['fid']}: z={mz:.2f} "
                        f"(tof={self.tof_range:.3f} at drone_z={self.pose[2]:.2f}) "
                        f"[{self.hp_idx+1}/{len(self.hp_route)}]")
                    self.hp_mode = 'CLIMBING'
                elif self._at_z(self.probe_bottom):
                    self.hp_measured[t['fid']] = None
                    self.n_phantoms += 1
                    self.get_logger().info(
                        f"[D{self.did}] probe F{t['fid']}: NO TOF return → "
                        f"phantom, dropped [{self.hp_idx+1}/{len(self.hp_route)}]")
                    self.hp_mode = 'CLIMBING'
                return

            if self.hp_mode == 'CLIMBING':
                self._goto(tx, ty, self.probe_top, self.limits[HEIGHT_PROBE])
                if self._at_z(self.probe_top):
                    self.hp_idx += 1
                    self.hp_mode = 'GOING'
                    self.hp_going_t0 = now
                return
            return

        # ── RETURN_TO_BASE (after survey+probe) ──────────────────
        if self.phase == RETURN_TO_BASE:
            self._goto(self.base_x, self.base_y, self.survey_z,
                       self.limits[RETURN_TO_BASE])
            if self._at_xy(self.base_x, self.base_y):
                self._enter(LAND_SURVEY)
            return

        # ── LAND_SURVEY ──────────────────────────────────────────
        if self.phase == LAND_SURVEY:
            self._goto(self.base_x, self.base_y, self.base_z+0.30,
                       self.limits[LAND_SURVEY])
            if self._at_z(self.base_z+0.30):
                self._send(0, 0, 0)
                msg = Bool(); msg.data = True
                for _ in range(3):
                    self.pub_done.publish(msg)
                n_conf = sum(1 for v in self.hp_measured.values() if v is not None)
                self.get_logger().info(
                    f"[D{self.did}] survey done — {n_conf} confirmed flower(s), "
                    f"{len(self.snap_buds)} bud(s)")
                self._enter(AWAIT_ROUTE)
            return

        # ── AWAIT_ROUTE ──────────────────────────────────────────
        if self.phase == AWAIT_ROUTE:
            self._send(0, 0, 0)
            if self.route_received_for_day == self.day:
                self._activate_route()
            return

        # ── Battery interrupt ─────────────────────────────────────
        if self.phase in (GOTO_FLOWER, RETREAT) and self._battery_low():
            self.get_logger().warning(
                f"[D{self.did}] battery low {self.batt_pct:.0f}% — RTB_BATTERY, "
                f"will resume at flower {self._active_idx+1}/{len(self._active_route)}")
            self._enter(RTB_BATTERY)
            return

        # ── GOTO_FLOWER ──────────────────────────────────────────
        if self.phase == GOTO_FLOWER:
            t = self._active_target
            if t is None:
                self._mission_complete = True
                self._enter(RTB)
                return
            tx, ty = float(t['x']), float(t['y'])
            self._goto(tx, ty, self.cruise_z, self.limits[GOTO_FLOWER])
            if self._at_xy(tx, ty) and self._at_z(self.cruise_z):
                head_z = float(t.get('z', 1.7))
                self.contact_z = head_z + self.pistil_L + self.tof_c
                self._contact_hold_z = None
                self._enter(DESCEND)
            return

        # ── DESCEND ───────────────────────────────────────────────
        if self.phase == DESCEND:
            t = self._active_target
            if t is None: self._enter(RTB); return
            tx, ty = float(t['x']), float(t['y'])
            dz = self.contact_z + 0.15
            self._goto(tx, ty, dz, self.limits[DESCEND])
            if self._at_xy(tx, ty) and self._at_z(dz):
                self._enter(BRUSH)
            return

        # ── BRUSH (TOF-latched contact) ───────────────────────────
        if self.phase == BRUSH:
            t = self._active_target
            if t is None: self._enter(RTB); return
            fx, fy = float(t['x']), float(t['y'])

            # contact latch: real TOF reading at brush distance, or
            # (fallback) tight altitude match on the measured contact_z
            if not self._contact_t0_set:
                tof_contact = (self.tof_range is not None
                               and self.tof_range <= self.tof_c + 0.015)
                z_contact = self._at_z_tight(self.contact_z, 0.05)
                if tof_contact or z_contact:
                    self._contact_t0 = now
                    self._contact_t0_set = True
                    self._contact_hold_z = self.pose[2]
                    src = 'TOF' if tof_contact else 'altitude'
                    self.get_logger().info(
                        f"[D{self.did}] ⤓ contact ({src}) on flower "
                        f"{int(t['flower_id'])} @ z={self.pose[2]:.2f}, "
                        f"{self.brush_dur:.1f}s sweep")

            hold_z = self._contact_hold_z if self._contact_hold_z is not None \
                     else self.contact_z
            elapsed_sweep = (now - self._contact_t0) if self._contact_t0_set else 0.0
            theta = 2*math.pi*2*(elapsed_sweep/max(self.brush_dur, 1e-3))
            tx = fx + 0.05*math.cos(theta)
            ty = fy + 0.05*math.sin(theta)
            self._goto(tx, ty, hold_z, self.limits[BRUSH])

            if self._contact_t0_set and elapsed_sweep >= self.brush_dur:
                fid = int(t['flower_id'])
                msg = Int32(); msg.data = fid
                self.pub_pollin.publish(msg)
                self.get_logger().info(
                    f"[D{self.did}] ✿ pollinated flower {fid} "
                    f"({self._active_idx+1}/{len(self._active_route)}) "
                    f"batt={self.batt_pct:.1f}%")
                self._active_idx += 1
                self._contact_t0_set = False
                self._contact_hold_z = None
                self._enter(ASCEND_GENTLE)
            elif (now - self.phase_t0) > (self.brush_dur + 12.0):
                self.get_logger().warning(
                    f"[D{self.did}] brush timeout on {int(t['flower_id'])} — skip")
                self._active_idx += 1
                self._contact_t0_set = False
                self._contact_hold_z = None
                self._enter(ASCEND_GENTLE)
            return

        # ── ASCEND_GENTLE ─────────────────────────────────────────
        if self.phase == ASCEND_GENTLE:
            t = self._active_target
            tx = float(t['x']) if t else self.base_x
            ty = float(t['y']) if t else self.base_y
            gz = (float(t.get('z', 1.7)) if t else 1.7) + 0.50
            self._goto(tx, ty, gz, self.limits[ASCEND_GENTLE])
            if self._at_z(gz):
                self._enter(RETREAT)
            return

        # ── RETREAT ───────────────────────────────────────────────
        if self.phase == RETREAT:
            t = self._active_target
            tx = float(t['x']) if t else self.base_x
            ty = float(t['y']) if t else self.base_y
            self._goto(tx, ty, self.cruise_z, self.limits[RETREAT])
            if self._at_z(self.cruise_z):
                remaining = len(self._active_route) - self._active_idx
                if remaining <= 0:
                    self._active_target = None
                    self._mission_complete = True
                    self._enter(RTB)
                else:
                    self._active_target = self._active_route[self._active_idx]
                    self._enter(GOTO_FLOWER)
            return

        # ── RTB_BATTERY ───────────────────────────────────────────
        if self.phase == RTB_BATTERY:
            self._goto(self.base_x, self.base_y, self.cruise_z,
                       self.limits[RTB_BATTERY])
            if self._at_xy(self.base_x, self.base_y) and self._at_z(self.cruise_z):
                self._enter(LAND_RTB)
            return

        # ── RTB ───────────────────────────────────────────────────
        if self.phase == RTB:
            self._goto(self.base_x, self.base_y, self.cruise_z,
                       self.limits[RTB])
            if self._at_xy(self.base_x, self.base_y):
                self._enter(LAND_RTB)
            return

        # ── LAND_RTB ─────────────────────────────────────────────
        if self.phase == LAND_RTB:
            self._goto(self.base_x, self.base_y, self.base_z+0.30,
                       self.limits[LAND_RTB])
            if self._at_z(self.base_z+0.30):
                self._send(0, 0, 0)
                remaining = len(self._active_route) - self._active_idx
                if remaining > 0 and not self._mission_complete:
                    self._swap_start_t = time.time()
                    self._enter(RECHARGING)
                else:
                    self.get_logger().info(
                        f"[D{self.did}] day {self.day} done. "
                        f"Pollinated {self._active_idx}/{len(self._active_route)}")
                    self._enter(DAY_DONE)
            return

        # ── RECHARGING (battery swap, then resume route) ──────────
        if self.phase == RECHARGING:
            self._send(0, 0, 0)
            if (time.time() - self._swap_start_t) >= self.batt_swap_dur:
                self.batt_pct = 100.0
                if self._active_idx < len(self._active_route):
                    self._active_target = self._active_route[self._active_idx]
                    self.get_logger().info(
                        f"[D{self.did}] battery swapped → resume flower "
                        f"{self._active_idx+1}/{len(self._active_route)}")
                    self._enter(GOTO_FLOWER)
                else:
                    self._mission_complete = True
                    self._enter(DAY_DONE)
            return

    # ── Live map streaming ("radio link") ─────────────────────────
    def _stream_provisional(self):
        """While surveying/probing, stream the current perception view
        (own strip) so the dashboard map fills in DURING flight.
        Display-only: messages are tagged provisional and the
        orchestrator discards them — routing still feeds exclusively
        on the post-TOF enriched map, which is always published last."""
        if self.phase not in (SURVEY, HEIGHT_PROBE):
            return
        obs = []
        for o in self.snap_obs:
            try:
                fid = int(o['flower_id'])
                x, y = float(o['x']), float(o['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if not (self.strip_x0 <= x < self.strip_x1):
                continue
            obs.append({'flower_id': fid,
                        'x': round(x, 3), 'y': round(y, 3),
                        'state': 'open', 'provisional': True,
                        'observed_by': self.did})
        buds = []
        for i, b in enumerate(self.snap_buds):
            try:
                bx, by = float(b['x']), float(b['y'])
            except (KeyError, ValueError, TypeError):
                continue
            if not (self.strip_x0 <= bx < self.strip_x1):
                continue
            buds.append({'bud_id': 900000 + self.did*10000 + i,
                         'x': round(bx, 3), 'y': round(by, 3),
                         'state': 'closed', 'observed_by': self.did})
        if not obs and not buds:
            return
        msg = String()
        msg.data = json.dumps({'drone_id': self.did, 'provisional': True,
                               'observations': obs,
                               'bud_observations': buds},
                              separators=(',', ':'))
        self.pub_obs.publish(msg)

    # ── 2 Hz status ───────────────────────────────────────────────
    def _publish_status(self):
        name = PHASE_NAMES[self.phase]
        if self.phase == SURVEY and self.survey_wps:
            frac = self.survey_wp_i / max(1, len(self.survey_wps))
            name = f"SURVEY {int(100*min(1.0, frac))}%"
        elif self.phase == HEIGHT_PROBE and self.hp_route:
            frac = self.hp_idx / max(1, len(self.hp_route))
            name = f"HEIGHT_PROBE {int(100*min(1.0, frac))}%"
        s = String(); s.data = name
        self.pub_state.publish(s)
        b = Float32(); b.data = float(self.batt_pct)
        self.pub_batt.publish(b)
        m = Bool(); m.data = self.phase in (IDLE, DAY_DONE)
        self.pub_atbase.publish(m)


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
