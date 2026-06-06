#!/usr/bin/env python3
"""
mission_orchestrator_stage15.py  —  Stage 15 (clean rewrite)
============================================================
Single coordinator node. Combines what Pass 1 split across four
separate nodes (bloom_state_manager + map_merger + mission_orchestrator
+ mission_logger_v15) into one place — same outputs, half the moving
parts, one source of truth for state.

WHAT IT DOES
------------
1. Loads the flower registry from /tmp/stage15_flowers.json.
2. Owns the day counter (mode.current_day → bloom_window_days).
3. For each day:
   a. Computes each flower's bloom state from (day - bloom_start)
   b. Publishes /bloom/current_day and /bloom/targets (latched)
   c. Fires /swarm/survey_start
   d. Waits for /swarm/survey_done/drone_N from every drone
   e. Aggregates observations → dedup → split spatially → NN+2opt
      → publishes /map_merger/routes
   f. Waits for /pollination/drone_N/at_base from every drone
   g. Advances to next day after `day_settle_s` pause
4. Publishes a unified event stream on /swarm/event (JSON String) for
   any consumer (dashboard, logger, ros2 topic echo).
5. Writes CSVs under ~/Desktop/FYP/mission_outputs/stage15/

JSON event envelope:
    {"schema":"1", "type": "...", "ts": 1779..., "day": 5,
     "drone_id": 0, "payload": {...}}

TYPES:
  mission_start, day_start, survey_start, survey_done, merge_complete,
  routes_assigned, pollination_success, drone_at_base, day_complete,
  mission_complete
"""

from __future__ import annotations

import csv
import json
import math
import os
import time
from collections import defaultdict
from pathlib import Path
from typing import List, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32, Float32, String


def _expand(p): return os.path.expanduser(os.path.expandvars(p))


# ── Mission phases (orchestrator-level) ───────────────────────────
INIT, DAY_PUBLISH, SURVEY_RUNNING, MERGING, POLLINATING, \
    DAY_SETTLE, MISSION_COMPLETE = range(7)


class MissionOrchestrator(Node):

    def __init__(self):
        super().__init__('mission_orchestrator')

        # ── Parameters ─────────────────────────────────────────────
        self.declare_parameter('n_drones',           2)
        self.declare_parameter('mode',               'full_cycle')
        self.declare_parameter('current_day',        5)
        self.declare_parameter('bloom_window_days',  8)
        self.declare_parameter('prime_window',       [2, 4])
        self.declare_parameter('partial_window',     [5, 7])
        self.declare_parameter('dedup_radius_m',     0.30)
        self.declare_parameter('include_partial',    True)
        self.declare_parameter('init_dwell_s',       6.0)
        self.declare_parameter('merge_dwell_s',      2.0)
        self.declare_parameter('day_settle_s',       3.0)
        self.declare_parameter('pollinate_grace_s',  5.0)
        self.declare_parameter('flowers_json',       '/tmp/stage15_flowers.json')
        self.declare_parameter('log_directory',      '~/Desktop/FYP/mission_outputs/stage15')
        self.declare_parameter('run_id_prefix',      'stage15')

        self.n_drones    = int(self.get_parameter('n_drones').value)
        self.mode        = str(self.get_parameter('mode').value)
        self.day         = int(self.get_parameter('current_day').value)
        self.day_max     = int(self.get_parameter('bloom_window_days').value)
        self.prime_win   = list(self.get_parameter('prime_window').value)
        self.partial_win = list(self.get_parameter('partial_window').value)
        self.dedup_r     = float(self.get_parameter('dedup_radius_m').value)
        self.include_part = bool(self.get_parameter('include_partial').value)
        self.init_dwell  = float(self.get_parameter('init_dwell_s').value)
        self.merge_dwell = float(self.get_parameter('merge_dwell_s').value)
        self.day_settle  = float(self.get_parameter('day_settle_s').value)
        self.pollinate_grace = float(self.get_parameter('pollinate_grace_s').value)
        self.flowers_json = str(self.get_parameter('flowers_json').value)
        log_dir = _expand(str(self.get_parameter('log_directory').value))
        self.run_id = f"{self.get_parameter('run_id_prefix').value}_{time.strftime('%Y%m%d_%H%M%S')}"

        # ── Registry ───────────────────────────────────────────────
        self.flowers: List[dict] = []
        self.bases:   List[dict] = []
        self._load_registry()

        # ── Mission state ──────────────────────────────────────────
        self.state = INIT
        self.t0 = time.time()
        self.phase_t0 = time.time()
        self.survey_done = {i: False for i in range(self.n_drones)}
        self.at_base     = {i: False for i in range(self.n_drones)}
        self.observations: Dict[int, list] = defaultdict(list)
        self.pollinated_today = 0
        self.pollinated_total = 0
        self.poses = {i: (0.0, 0.0, 0.0) for i in range(self.n_drones)}
        self.states = {i: 'IDLE' for i in range(self.n_drones)}
        self.battery = {i: 100.0 for i in range(self.n_drones)}
        self.routes_published_for_day = -1

        # ── Logging ────────────────────────────────────────────────
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        self.events_path = Path(log_dir) / f"events_{self.run_id}.csv"
        self.pollin_path = Path(log_dir) / f"pollination_{self.run_id}.csv"
        self.telem_path  = Path(log_dir) / f"telemetry_{self.run_id}.csv"
        self.summary_path = Path(log_dir) / f"mission_{self.run_id}.txt"

        self.events_f = open(self.events_path, 'w', newline='')
        self.events_w = csv.writer(self.events_f)
        self.events_w.writerow(['ts', 'day', 'drone_id', 'type', 'payload_json'])

        self.pollin_f = open(self.pollin_path, 'w', newline='')
        self.pollin_w = csv.writer(self.pollin_f)
        self.pollin_w.writerow(['run_id', 'day', 'drone_id',
                                'flower_id', 'x', 'y', 'pollinated_at'])

        self.telem_f = open(self.telem_path, 'w', newline='')
        self.telem_w = csv.writer(self.telem_f)
        header = ['ts', 'day']
        for i in range(self.n_drones):
            header += [f'D{i}_state', f'D{i}_x', f'D{i}_y', f'D{i}_z', f'D{i}_batt']
        self.telem_w.writerow(header)

        # ── QoS ───────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        # ── Publishers ────────────────────────────────────────────
        self.pub_day      = self.create_publisher(Int32,  '/bloom/current_day', latched_qos)
        self.pub_targets  = self.create_publisher(String, '/bloom/targets',     latched_qos)
        self.pub_start    = self.create_publisher(Bool,   '/swarm/survey_start', 10)
        self.pub_routes   = self.create_publisher(String, '/map_merger/routes',  10)
        self.pub_event    = self.create_publisher(String, '/swarm/event',        10)
        self.pub_abort    = self.create_publisher(Bool,   '/swarm/abort',        10)

        # ── Subscribers ───────────────────────────────────────────
        for i in range(self.n_drones):
            self.create_subscription(String, f'/survey/drone_{i}/observations',
                                     lambda m, d=i: self._on_obs(d, m), 10)
            self.create_subscription(Bool,   f'/swarm/survey_done/drone_{i}',
                                     lambda m, d=i: self._on_survey_done(d, m), 10)
            self.create_subscription(Bool,   f'/pollination/drone_{i}/at_base',
                                     lambda m, d=i: self._on_at_base(d, m), 10)
            self.create_subscription(Int32,  f'/pollination/drone_{i}/pollinated',
                                     lambda m, d=i: self._on_pollin(d, m), 10)
            self.create_subscription(String, f'/pollination/drone_{i}/state',
                                     lambda m, d=i: self._on_state(d, m), 10)
            self.create_subscription(PoseStamped, f'/pollination/drone_{i}/pose',
                                     lambda m, d=i: self._on_pose(d, m), 10)
            self.create_subscription(Float32, f'/battery/drone_{i}',
                                     lambda m, d=i: self._on_batt(d, m), 10)

        # ── Timers ────────────────────────────────────────────────
        self.create_timer(0.5, self._tick)        # state machine
        self.create_timer(0.5, self._telemetry)   # 2 Hz CSV writes

        self.get_logger().info(
            f"orchestrator up [v3-state-based-day-done]. "
            f"n_drones={self.n_drones} mode={self.mode} "
            f"day={self.day} day_max={self.day_max} flowers={len(self.flowers)}")
        self._event('mission_start', None, {
            'n_drones': self.n_drones, 'mode': self.mode,
            'start_day': self.day, 'day_max': self.day_max,
            'n_flowers': len(self.flowers)})

    # ── Registry loading ───────────────────────────────────────────
    def _load_registry(self):
        path = _expand(self.flowers_json)
        try:
            with open(path) as f:
                data = json.load(f)
            self.flowers = list(data.get('flowers', []))
            self.bases   = list(data.get('drone_bases', []))
            # Override n_drones if registry says so (single source of truth)
            if self.bases:
                self.n_drones = len(self.bases)
        except Exception as e:
            self.get_logger().error(f"couldn't load registry {path}: {e}")

    # ── Subscribers ───────────────────────────────────────────────
    def _on_obs(self, did, m):
        try:
            data = json.loads(m.data)
            self.observations[did].extend(data.get('observations', []))
        except Exception:
            pass

    def _on_survey_done(self, did, m):
        if m.data and not self.survey_done.get(did, False):
            self.survey_done[did] = True
            self._event('survey_done', did, {
                'n_observed': len(self.observations.get(did, []))})

    def _on_at_base(self, did, m):
        # Treat at_base as a level signal — only fire event on rising edge.
        was = self.at_base.get(did, False)
        self.at_base[did] = bool(m.data)
        if m.data and not was:
            self._event('drone_at_base', did, {})

    def _on_pollin(self, did, m):
        fid = int(m.data)
        self.pollinated_today += 1
        self.pollinated_total += 1
        x = y = None
        for f in self.flowers:
            if int(f['flower_id']) == fid:
                x, y = float(f['x']), float(f['y'])
                break
        ts = time.time()
        self._event('pollination_success', did, {
            'flower_id': fid, 'x': x, 'y': y,
            'cumulative_total': self.pollinated_total})
        self.pollin_w.writerow([self.run_id, self.day, did, fid, x, y,
                                time.strftime('%Y-%m-%d %H:%M:%S',
                                              time.localtime(ts))])
        self.pollin_f.flush()

    def _on_state(self, did, m):
        s = str(m.data)
        if s != self.states.get(did):
            self.states[did] = s

    def _on_pose(self, did, m: PoseStamped):
        self.poses[did] = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def _on_batt(self, did, m: Float32):
        self.battery[did] = float(m.data)

    # ── Bloom-state classifier ────────────────────────────────────
    def _state_for(self, dsi: int) -> str:
        if dsi < 0:           return 'not_ready'
        if dsi <= 1:          return 'opening'
        if self.prime_win[0] <= dsi <= self.prime_win[1]:
            return 'prime_target'
        if self.partial_win[0] <= dsi <= self.partial_win[1]:
            return 'partial_target'
        return 'spent'

    def _publish_day(self):
        # Latched day
        dm = Int32(); dm.data = self.day
        self.pub_day.publish(dm)
        # Compute targets
        counts = {'not_ready': 0, 'opening': 0, 'prime_target': 0,
                  'partial_target': 0, 'spent': 0}
        targets = []
        for f in self.flowers:
            st = self._state_for(self.day - int(f['bloom_start']))
            counts[st] += 1
            if st in ('prime_target', 'partial_target'):
                targets.append({
                    'flower_id': int(f['flower_id']),
                    'x': float(f['x']), 'y': float(f['y']),
                    'z': float(f.get('head_z', 1.7)),
                    'state': st})
        tm = String(); tm.data = json.dumps(
            {'day': self.day, 'flowers': targets}, separators=(',', ':'))
        self.pub_targets.publish(tm)
        self.get_logger().info(
            f"day {self.day}: not_ready={counts['not_ready']} "
            f"opening={counts['opening']} prime={counts['prime_target']} "
            f"partial={counts['partial_target']} spent={counts['spent']} "
            f"→ {len(targets)} targets")
        self._event('day_start', None, {
            'day': self.day, 'counts': counts, 'n_targets': len(targets)})
        return len(targets)

    # ── mTSP: dedup → spatial split → NN+2opt ────────────────────
    def _compute_routes(self) -> dict:
        all_obs = []
        for lst in self.observations.values():
            all_obs.extend(lst)
        # Dedup by flower_id
        by_id = {}
        for o in all_obs:
            fid = int(o['flower_id'])
            if fid not in by_id:
                by_id[fid] = o
        merged = list(by_id.values())
        if not self.include_part:
            merged = [m for m in merged if m.get('state') == 'prime_target']

        self.get_logger().info(
            f"merge: {len(all_obs)} observations → {len(merged)} unique")

        if not merged:
            return {str(i): [] for i in range(self.n_drones)}

        # Balanced spatial partition: sort by x, give each flower to the
        # drone with the smallest current count whose base is nearest in x.
        merged.sort(key=lambda f: (f['x'], f['y']))
        partitions = [[] for _ in range(self.n_drones)]
        base_xs = [b['base_x'] for b in self.bases] if self.bases \
                  else [0.0] * self.n_drones
        base_ys = [b['base_y'] for b in self.bases] if self.bases \
                  else [-1.0] * self.n_drones
        for f in merged:
            min_count = min(len(p) for p in partitions)
            candidates = [d for d in range(self.n_drones)
                          if len(partitions[d]) == min_count]
            best = min(candidates, key=lambda d: abs(f['x'] - base_xs[d]))
            partitions[best].append(f)

        # NN + 2-opt per drone
        routes = {}
        for did in range(self.n_drones):
            base = (base_xs[did], base_ys[did])
            ordered = self._nn_then_2opt(partitions[did], base)
            routes[str(did)] = [
                {'flower_id': int(f['flower_id']),
                 'x': float(f['x']), 'y': float(f['y']),
                 'z': float(f.get('z', f.get('head_z', 1.7)))}
                for f in ordered]
        return routes

    def _nn_then_2opt(self, flowers, base):
        if len(flowers) <= 1:
            return flowers
        rem = list(flowers)
        cur = base
        path = []
        while rem:
            nxt = min(rem, key=lambda f:
                      math.hypot(f['x'] - cur[0], f['y'] - cur[1]))
            path.append(nxt)
            cur = (nxt['x'], nxt['y'])
            rem.remove(nxt)

        def tour_len(p):
            t = math.hypot(p[0]['x'] - base[0], p[0]['y'] - base[1])
            for a, b in zip(p, p[1:]):
                t += math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            return t

        best, best_len = list(path), tour_len(path)
        for _ in range(4):
            improved = False
            for i in range(len(best) - 1):
                for j in range(i + 2, len(best)):
                    new = best[:i] + best[i:j+1][::-1] + best[j+1:]
                    nl = tour_len(new)
                    if nl + 1e-6 < best_len:
                        best, best_len = new, nl
                        improved = True
            if not improved:
                break
        return best

    # ── State machine tick ───────────────────────────────────────
    def _tick(self):
        now = time.time()

        if self.state == INIT:
            if (now - self.phase_t0) >= self.init_dwell:
                self._enter(DAY_PUBLISH)

        elif self.state == DAY_PUBLISH:
            n_targets = self._publish_day()
            self.survey_done = {i: False for i in range(self.n_drones)}
            self.observations = defaultdict(list)
            self.pollinated_today = 0
            self.routes_published_for_day = -1
            # NOTE: we do NOT reset self.at_base here. Each controller now
            # publishes at_base as a 2 Hz LEVEL signal (True iff in IDLE or
            # DAY_DONE), so the orchestrator's view of at_base is always the
            # drone's current parked state — no stale Trues to scrub.
            # NOTE: we do NOT time.sleep here either. Blocking the executor
            # would let stale at_base=True messages from still-IDLE drones
            # queue up between the reset and survey_start firing — the exact
            # race that caused day_complete to fire prematurely in the
            # earlier version. Fire survey_start immediately and let drones
            # transition out of IDLE on their own clock.
            sm = Bool(); sm.data = True
            for _ in range(3):
                self.pub_start.publish(sm)
            self._event('survey_start', None, {'day': self.day,
                                               'n_targets': n_targets})
            self._enter(SURVEY_RUNNING)

        elif self.state == SURVEY_RUNNING:
            if all(self.survey_done.values()):
                self._enter(MERGING)

        elif self.state == MERGING:
            if (now - self.phase_t0) < self.merge_dwell:
                return
            routes = self._compute_routes()
            payload = {'day': self.day, 'routes': routes}
            rm = String(); rm.data = json.dumps(payload, separators=(',', ':'))
            for _ in range(3):
                self.pub_routes.publish(rm)
            self.routes_published_for_day = self.day
            counts = {k: len(v) for k, v in routes.items()}
            self._event('merge_complete', None, {'unique_targets': sum(counts.values())})
            self._event('routes_assigned', None,
                        {'route_counts': counts, 'day': self.day})
            self.get_logger().info(f"routes assigned: {counts}")
            self._enter(POLLINATING)

        elif self.state == POLLINATING:
            # Grace period: drones need time to fly out to flowers; we
            # mustn't fire day_complete in the first few seconds after
            # entering POLLINATING (when at_base could still be stale).
            if (now - self.phase_t0) < self.pollinate_grace:
                return
            # Day is complete when EVERY drone's reported state is DAY_DONE.
            # State strings are republished every 0.5s by each controller's
            # _publish_status, so this is a reliable level signal.
            # Fallback: also accept the at_base flag (for compatibility with
            # older controller versions that publish at_base level signal).
            day_done_via_state = all(
                self.states.get(i) == 'DAY_DONE'
                for i in range(self.n_drones))
            if day_done_via_state:
                self._event('day_complete', None,
                            {'day': self.day,
                             'pollinated_today': self.pollinated_today})
                self.get_logger().info(
                    f"── day {self.day} complete — "
                    f"{self.pollinated_today} pollinated today, "
                    f"{self.pollinated_total} total ──")
                self._enter(DAY_SETTLE)

        elif self.state == DAY_SETTLE:
            if (now - self.phase_t0) < self.day_settle:
                return
            # Decide: advance, finish, or stop
            if self.mode == 'single_day':
                self._enter(MISSION_COMPLETE)
                return
            if self.day + 1 > self.day_max:
                self._enter(MISSION_COMPLETE)
                return
            self.day += 1
            self._enter(DAY_PUBLISH)

        elif self.state == MISSION_COMPLETE:
            # Idempotent — fire event once
            if not getattr(self, '_mission_complete_fired', False):
                self._mission_complete_fired = True
                self._event('mission_complete', None,
                            {'total_pollinated': self.pollinated_total,
                             'duration_s': round(time.time() - self.t0, 1)})
                self.get_logger().info(
                    f"★ MISSION COMPLETE — total pollinated "
                    f"{self.pollinated_total}")
                self._write_summary()

    def _enter(self, state: int):
        self.state = state
        self.phase_t0 = time.time()

    # ── Event publishing & logging ────────────────────────────────
    def _event(self, etype: str, drone_id, payload: dict):
        env = {
            'schema': '1',
            'type': etype,
            'ts': round(time.time(), 3),
            'day': self.day,
            'drone_id': drone_id,
            'payload': payload or {},
        }
        msg = String(); msg.data = json.dumps(env, separators=(',', ':'))
        self.pub_event.publish(msg)
        self.events_w.writerow([env['ts'], env['day'],
                                env['drone_id'] if drone_id is not None else '',
                                etype, json.dumps(payload or {},
                                                  separators=(',', ':'))])
        self.events_f.flush()

    # ── 2 Hz telemetry CSV ────────────────────────────────────────
    def _telemetry(self):
        row = [round(time.time(), 3), self.day]
        for i in range(self.n_drones):
            x, y, z = self.poses.get(i, (0, 0, 0))
            row += [self.states.get(i, '?'),
                    round(x, 3), round(y, 3), round(z, 3),
                    round(self.battery.get(i, 0.0), 1)]
        self.telem_w.writerow(row)
        # Don't flush every tick — too slow. Flush every 10 ticks.
        if not hasattr(self, '_telem_n'):
            self._telem_n = 0
        self._telem_n += 1
        if self._telem_n % 10 == 0:
            self.telem_f.flush()

    def _write_summary(self):
        try:
            with open(self.summary_path, 'w') as f:
                f.write(f"Stage 15 mission summary — run {self.run_id}\n")
                f.write(f"  mode:        {self.mode}\n")
                f.write(f"  days run:    started day; ended day {self.day}\n")
                f.write(f"  n_drones:    {self.n_drones}\n")
                f.write(f"  flowers:     {len(self.flowers)}\n")
                f.write(f"  pollinated:  {self.pollinated_total}\n")
                f.write(f"  duration:    {round(time.time() - self.t0, 1)} s\n")
                f.write(f"\nLogs:\n")
                f.write(f"  events:        {self.events_path}\n")
                f.write(f"  pollinations:  {self.pollin_path}\n")
                f.write(f"  telemetry:     {self.telem_path}\n")
        except Exception as e:
            self.get_logger().warning(f"couldn't write summary: {e}")

    def destroy_node(self):
        try:
            self.events_f.close()
            self.pollin_f.close()
            self.telem_f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    n = MissionOrchestrator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
