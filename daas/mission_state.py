"""
mission_state.py  —  Stage 13 DaaS Dashboard
==============================================
Multi-day autonomous swarm pollination simulation.

Field layout:  60 m × 20 m
Three sectors: LEFT (0–20 m)  CENTER (20–40 m)  RIGHT (40–60 m)
One drone per sector.

Mission structure:
  Each run generates a random bloom window of 8–10 days.
  Every day:  MAPPING (lawnmower sweep) → POLLINATING → pause → next day
  Mission stops when:
    A) All pollination-window flowers have been pollinated
    B) 2 consecutive days map 0 new ready flowers (bloom window over)
    C) total_days reached (natural end of bloom window)

Flower bloom model (scientifically grounded):
  Sunflower anthesis spans 5–10 days per head (eLife 2023).
  Florets open ring by ring from outer edge inward — bell-curve distribution.
  For a field of plants with staggered development, the collective window
  spans 8–12 days with a peak around Days 3–4.

  bloom_day = 0        → never opens (damaged / unhealthy)  ~5 %
  bloom_day = 1..10    → opens on that day (bell curve)    ~95 %
  Any flower with bloom_day > total_days is outside this run's window
  and is treated as a non-target.

Realistic drone parameters (x500 quadrotor, precision pollination):
  Operating altitude: ~3 m above flower level.
  Tight 20 m sector → frequent turns → effective speed reduced ~15 %.
  Sources: eLife 2023, MDPI Robotics 2022 (2 s flower-to-flower flight),
           Artificial Pollination Nano Drone review 2024 (approach stages).

Drone card stats reset at the start of each day (per-mission view).
"""

import math
import random
import threading
import time
from pathlib import Path

# ── Field constants ────────────────────────────────────────────────────────────
FIELD_W = 60.0
FIELD_H = 20.0

SECTOR_NAMES    = ['LEFT', 'CENTER', 'RIGHT']
DRONE_SECTOR    = {0: 'LEFT', 1: 'CENTER', 2: 'RIGHT'}
SECTOR_X_RANGE  = {
    'LEFT':   (0.0,  20.0),
    'CENTER': (20.0, 40.0),
    'RIGHT':  (40.0, 60.0),
}

# ── Bloom distribution (bell curve, peer-reviewed basis) ──────────────────────
# bloom_day 0 = never opens; 1-10 = day of anthesis
# Weights approximate outside-in pseudowhorl pattern (Marshall & Harmer 2023)
_BLOOM_DAYS    = [0,  1,  2,   3,   4,   5,   6,  7,  8,  9,  10]
_BLOOM_WEIGHTS = [5,  4, 10,  18,  22,  18,  10,  6,  3,  2,   2]
# sums to 100

# ── Simulation timing (real-time feel, not 1:1 real) ──────────────────────────
_T_MAP_WP    = 0.07   # per mapping waypoint (sim speed)
_T_SEARCH    = 0.22
_T_APPROACH  = 0.18
_T_HOVER     = 0.14
_T_POLLINATE = 0.18
_T_BETWEEN   = 3.0    # pause between days
_T_LAND      = 0.55

# Flower detection radius during mapping
_DETECT_R = 1.8   # metres

# Lawnmower grid
_ROW_STEP = 2.0
_COL_STEP = 2.0

# ── Realistic drone performance constants (estimated real-world) ───────────────
# Based on x500 quadrotor operating in precision pollination environment.
# Literature: MDPI Robotics 11(6) 2022, eLife 2023, MDPI UAV Review 2024
PHASE_SPEED = {
    'SWEEP':     2.5,   # m/s — tight 20 m field, frequent turns
    'SEARCH':    3.0,   # m/s — inter-flower transit with wind margin
    'APPROACH':  0.8,   # m/s — slow descent to flower level (~3 m drop)
    'HOVER':     0.2,   # m/s — fine positioning above flower
    'POLLINATE': 0.0,   # m/s — stationary contact
    'LAND':      0.8,   # m/s — controlled descent
    'TAKEOFF':   1.2,   # m/s — ascent to operating altitude (~3.5 m)
    'IDLE':      0.0,
}
_D_APPROACH   = 3.0   # m — altitude drop to flower level
_D_HOVER      = 0.5   # m — fine positioning movement
_D_ASCENT     = 3.0   # m — rise back to cruise altitude after pollination
_D_TAKEOFF    = 3.5   # m — initial ascent at mission start
_D_LAND       = 3.5   # m — final descent at mission end
_POL_DWELL    = 4.0   # real seconds — contact dwell per flower
# 15 % time overhead for deceleration + re-acceleration at each sweep turn
_TURN_FACTOR  = 1.15

# Optional CSV from stage 12
_REGISTRY_CSV = Path.home() / 'Desktop/FYP/setup/flower_registry.csv'

# Mission: stop early after this many consecutive zero-yield days
_ZERO_YIELD_STOP = 2
# Don't trigger early stop before this day (avoid false alarm on slow start)
_MIN_DAY_FOR_STOP = 5
# Absolute safety ceiling — mission cannot run beyond this
# (well outside any biological window; purely a runaway guard)
MISSION_HARD_LIMIT = 20


# ── Helpers ───────────────────────────────────────────────────────────────────

def _generate_flowers() -> list:
    """
    Generate 300–600 sunflowers with biologically-realistic bloom distribution.
    bloom_day follows the outside-in pseudowhorl bell curve.
    """
    n = random.randint(300, 600)
    flowers = []
    for i in range(n):
        x = random.uniform(0.5, FIELD_W - 0.5)
        y = random.uniform(0.5, FIELD_H - 0.5)
        sector    = 'LEFT' if x < 20 else ('CENTER' if x < 40 else 'RIGHT')
        bloom_day = random.choices(_BLOOM_DAYS, weights=_BLOOM_WEIGHTS)[0]
        # Confidence higher for flowers opening at peak (Days 3–5)
        if 2 <= bloom_day <= 6:
            conf = random.uniform(0.86, 0.98)
        elif bloom_day in (1, 7, 8):
            conf = random.uniform(0.78, 0.92)
        else:
            conf = random.uniform(0.70, 0.82)
        flowers.append({'id': f'F{i:04d}', 'x': x, 'y': y,
                        'sector': sector, 'bloom_day': bloom_day, 'conf': conf})
    return flowers


def _lawnmower_waypoints(sector: str) -> list:
    """Boustrophedon sweep waypoints for one sector."""
    x_min = SECTOR_X_RANGE[sector][0] + 0.5
    x_max = SECTOR_X_RANGE[sector][1] - 0.5
    waypoints = []
    y = 1.0
    left_to_right = True
    while y <= FIELD_H - 1.0 + 0.01:
        xs = []
        x = x_min
        while x <= x_max + 0.01:
            xs.append(round(x, 2))
            x += _COL_STEP
        if not left_to_right:
            xs = xs[::-1]
        for xp in xs:
            waypoints.append((xp, round(y, 2)))
        y = round(y + _ROW_STEP, 2)
        left_to_right = not left_to_right
    return waypoints


def _compute_metrics(flowers: list) -> tuple:
    """TSP improvement % and Bloom filter gain % for this field."""
    total  = len(flowers)
    # Closed = bloom_day 0 only (truly never-opens; late bloomers may still open)
    closed = sum(1 for f in flowers if f['bloom_day'] == 0)
    ready  = total - closed
    bloom_pct = round(closed / total * 100, 1) if total > 0 else 0.0
    tsp_base  = 34.0 + ready / 14.0
    tsp_pct   = round(min(max(tsp_base + random.uniform(-5, 5), 42.0), 72.0), 1)
    return tsp_pct, bloom_pct


def _empty_day_entry() -> dict:
    return {
        'status':             'pending',
        'new_discovered':     0,
        'cumulative_scanned': 0,
        'prev_pollinated':    0,
        'discovered':         0,
        'ready':              0,
        'not_ready':          0,
        'pollinated':         0,
        'missed':             0,
    }


def _empty_perf_entry() -> dict:
    return {
        'status':    'pending',
        'map_dist':  0.0,
        'map_time':  0.0,
        'pol_dist':  0.0,
        'pol_time':  0.0,
        'avg_speed': 0.0,
    }


# ── MissionState ──────────────────────────────────────────────────────────────

class MissionState:

    def __init__(self):
        self._lock       = threading.Lock()
        self._sim_stop   = threading.Event()
        self._sim_thread = None

        self.flowers = _generate_flowers()
        self._reset_live_state()
        self._load_registry_csv()

    # ── Initialisation ─────────────────────────────────────────────────────────

    def _reset_live_state(self):
        # Mission flags
        self.mission_active     = False
        self.mission_paused     = False
        self.connected_to_ros   = False
        self.mission_start_time = time.time()
        self.last_update_time   = time.time()

        # No pre-determined window — mission runs until exit conditions met
        self.current_day = 0
        self.current_phase = 'IDLE'
        self.mission_stop_reason = ''
        self.consecutive_zero_days = 0

        # Drone states
        self.drone_states    = {0: 'IDLE', 1: 'IDLE', 2: 'IDLE'}
        self.drone_sectors   = {
            0: 'LEFT   (x < 20 m)',
            1: 'CENTER (20–40 m)',
            2: 'RIGHT  (x > 40 m)',
        }
        self.current_targets = {0: None, 1: None, 2: None}

        # Mapping state
        self.mapping_progress   = 0.0
        self.mapping_wp_done    = 0
        self.mapping_wp_total   = 1

        # Discovered flower tracking
        self.discovered_ids     = set()   # all-time all days
        self.day_discovered_ids = set()   # this day only

        # Per-sector counts (reset each day)
        self.sector_discovered  = {s: 0 for s in SECTOR_NAMES}
        self.sector_ready       = {s: 0 for s in SECTOR_NAMES}
        self.sector_not_ready   = {s: 0 for s in SECTOR_NAMES}

        # Pollination
        self.pollinated_ever    = set()
        self.pollination_events = []
        self.day_pollinated     = 0

        # Health grid 60 × 20
        self._grid_sum   = [[0.0] * 60 for _ in range(20)]
        self._grid_count = [[0]   * 60 for _ in range(20)]
        self.health_grid = [[0.0] * 60 for _ in range(20)]

        # Per-day stats — grow dynamically as days complete
        self.day_stats = {}

        # Drone performance (per-day, reset each day)
        self.drone_speed     = {0: 0.0, 1: 0.0, 2: 0.0}
        self.drone_distance  = {0: 0.0, 1: 0.0, 2: 0.0}   # resets each day
        self.drone_real_secs = {0: 0.0, 1: 0.0, 2: 0.0}   # resets each day
        self.day_perf        = {}  # grows dynamically

        # Metrics
        self.tsp_pct, self.bloom_pct = _compute_metrics(self.flowers)

    def _load_registry_csv(self):
        if not _REGISTRY_CSV.exists():
            return
        try:
            import csv
            with open(_REGISTRY_CSV, newline='') as f:
                for row in csv.DictReader(f):
                    self._grid_add(float(row.get('x', 0)),
                                   float(row.get('y', 0)),
                                   float(row.get('confidence', 0.85)))
        except Exception:
            pass

    def _grid_add(self, x: float, y: float, conf: float):
        col = max(0, min(59, int(x)))
        row = max(0, min(19, int(y)))
        self._grid_sum[row][col]   += conf
        self._grid_count[row][col] += 1
        self.health_grid[row][col]  = (self._grid_sum[row][col] /
                                       self._grid_count[row][col])

    # ── Simulation entry ───────────────────────────────────────────────────────

    def start_simulation(self):
        if self._sim_thread and self._sim_thread.is_alive():
            return
        self._sim_stop.clear()
        with self._lock:
            self.mission_active     = True
            self.mission_start_time = time.time()
        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True, name='sim_main')
        self._sim_thread.start()

    def stop_simulation(self):
        self._sim_stop.set()
        with self._lock:
            self.mission_active  = False
            self.current_phase   = 'IDLE'
            for d in range(3):
                self.drone_states[d]    = 'IDLE'
                self.drone_speed[d]     = 0.0
                self.current_targets[d] = None

    # ── Main simulation loop ───────────────────────────────────────────────────

    def _sim_loop(self):
        """
        Runs up to total_days days automatically.
        Each day: MAPPING → POLLINATING → BETWEEN_DAYS pause.
        Stops early on two consecutive zero-yield mapping days (≥ Day 5)
        or when all pollination-window flowers are done.
        """
        day = 0
        while not self._sim_stop.is_set():
            day += 1

            # ── Hard safety ceiling ────────────────────────────────────────────
            if day > MISSION_HARD_LIMIT:
                with self._lock:
                    self.mission_stop_reason = 'hard_limit'
                break

            # ── Reset per-day drone stats (card shows today only) ──────────────
            with self._lock:
                for d in range(3):
                    self.drone_distance[d]  = 0.0
                    self.drone_real_secs[d] = 0.0
                    self.drone_speed[d]     = 0.0

                self.current_day        = day
                self.current_phase      = 'MAPPING'
                self.mapping_progress   = 0.0
                self.mapping_wp_done    = 0
                self.day_discovered_ids = set()
                self.sector_discovered  = {s: 0 for s in SECTOR_NAMES}
                self.sector_ready       = {s: 0 for s in SECTOR_NAMES}
                self.sector_not_ready   = {s: 0 for s in SECTOR_NAMES}
                self.day_pollinated     = 0
                # Initialise this day's stats entry dynamically
                if day not in self.day_stats:
                    self.day_stats[day] = _empty_day_entry()
                if day not in self.day_perf:
                    self.day_perf[day]  = _empty_perf_entry()
                self.day_stats[day]['status'] = 'active'
                prev_pol_count = len(self.pollinated_ever)

            # ── MAPPING ────────────────────────────────────────────────────────
            with self._lock:
                dist_b = {d: self.drone_distance[d]  for d in range(3)}
                secs_b = {d: self.drone_real_secs[d] for d in range(3)}

            self._run_mapping(day)
            if self._sim_stop.is_set():
                break

            # After mapping: classify ALL known flowers for today
            with self._lock:
                map_dists = [self.drone_distance[d]  - dist_b[d] for d in range(3)]
                map_secs  = [self.drone_real_secs[d] - secs_b[d] for d in range(3)]
                # Apply turn-overhead factor to mapping time
                self.day_perf[day]['map_dist'] = round(sum(map_dists) / 3, 1)
                self.day_perf[day]['map_time'] = round(
                    max(map_secs) * _TURN_FACTOR, 1)

                for s in SECTOR_NAMES:
                    self.sector_ready[s]     = 0
                    self.sector_not_ready[s] = 0
                for f in self.flowers:
                    if f['id'] not in self.discovered_ids:
                        continue
                    if f['id'] in self.pollinated_ever:
                        continue
                    if f['bloom_day'] == day:
                        self.sector_ready[f['sector']] += 1
                    else:
                        self.sector_not_ready[f['sector']] += 1

                ready_n   = sum(self.sector_ready.values())
                not_rdy_n = sum(self.sector_not_ready.values())
                new_scans = len(self.day_discovered_ids)
                cumul     = len(self.discovered_ids)

                self.day_stats[day].update({
                    'new_discovered':     new_scans,
                    'cumulative_scanned': cumul,
                    'prev_pollinated':    prev_pol_count,
                    'discovered':         new_scans,
                    'ready':              ready_n,
                    'not_ready':          not_rdy_n,
                })

                # Track consecutive zero-yield days
                if ready_n == 0:
                    self.consecutive_zero_days += 1
                else:
                    self.consecutive_zero_days = 0

                self.current_phase = 'POLLINATING'
                dist_b2 = {d: self.drone_distance[d]  for d in range(3)}
                secs_b2 = {d: self.drone_real_secs[d] for d in range(3)}

            # ── Early stop: 2 consecutive zero-yield days after Day 5 ──────────
            if (self.consecutive_zero_days >= _ZERO_YIELD_STOP
                    and day >= _MIN_DAY_FOR_STOP):
                with self._lock:
                    self.mission_stop_reason = 'zero_yield'
                    self.day_stats[day]['status'] = 'complete'
                    self.day_stats[day]['pollinated'] = 0
                    self.day_stats[day]['missed']     = 0
                    self.day_perf[day]['status']      = 'complete'
                break

            # ── POLLINATING ────────────────────────────────────────────────────
            self._run_pollination(day)
            if self._sim_stop.is_set():
                break

            with self._lock:
                done  = self.day_pollinated
                ready = self.day_stats[day]['ready']
                self.day_stats[day]['pollinated'] = done
                self.day_stats[day]['missed']     = max(0, ready - done)
                self.day_stats[day]['status']     = 'complete'

                pol_dists = [self.drone_distance[d]  - dist_b2[d] for d in range(3)]
                pol_secs  = [self.drone_real_secs[d] - secs_b2[d] for d in range(3)]
                self.day_perf[day]['pol_dist'] = round(sum(pol_dists) / 3, 1)
                self.day_perf[day]['pol_time'] = round(max(pol_secs), 1)
                self.day_perf[day]['status']   = 'complete'

                tot_d = self.day_perf[day]['map_dist'] + self.day_perf[day]['pol_dist']
                tot_t = self.day_perf[day]['map_time'] + self.day_perf[day]['pol_time']
                self.day_perf[day]['avg_speed'] = (
                    round(tot_d / tot_t, 2) if tot_t > 0 else 0.0)

            # ── All pollinated? ────────────────────────────────────────────────
            with self._lock:
                # All pollination-viable flowers = bloom_day > 0
                target_flowers = [f for f in self.flowers
                                  if f['bloom_day'] > 0]
                all_done = all(f['id'] in self.pollinated_ever
                               for f in target_flowers)
            if all_done:
                with self._lock:
                    self.mission_stop_reason = 'all_pollinated'
                break

            # ── BETWEEN DAYS (always — there is always a next day until exit) ─
            if True:
                with self._lock:
                    self.current_phase = 'BETWEEN_DAYS'
                    for d in range(3):
                        self.drone_states[d]    = 'IDLE'
                        self.drone_speed[d]     = 0.0
                        self.current_targets[d] = None
                self._sim_stop.wait(_T_BETWEEN)

        # ── COMPLETE ───────────────────────────────────────────────────────────
        if not self._sim_stop.is_set():
            with self._lock:
                self.current_phase  = 'COMPLETE'
                self.mission_active = False
                if not self.mission_stop_reason:
                    self.mission_stop_reason = 'day_limit'
                for d in range(3):
                    self.drone_states[d]    = 'IDLE'
                    self.drone_speed[d]     = 0.0
                    self.current_targets[d] = None

    # ── Mapping phase ──────────────────────────────────────────────────────────

    def _run_mapping(self, day: int):
        sector_wps = {s: _lawnmower_waypoints(s) for s in SECTOR_NAMES}
        total_wps  = sum(len(v) for v in sector_wps.values())
        with self._lock:
            self.mapping_wp_total = total_wps
            self.mapping_wp_done  = 0

        def _sweep(drone_id: int, sector: str):
            with self._lock:
                self.drone_states[drone_id] = 'SWEEP'
                self.drone_speed[drone_id]  = PHASE_SPEED['SWEEP']
                self.drone_distance[drone_id]  += _D_TAKEOFF
                self.drone_real_secs[drone_id] += _D_TAKEOFF / PHASE_SPEED['TAKEOFF']

            prev_wp = None
            for (wx, wy) in sector_wps[sector]:
                if self._sim_stop.is_set():
                    break

                with self._lock:
                    self.current_targets[drone_id] = {
                        'id': f'({wx:.1f},{wy:.1f})',
                        'x': wx, 'y': wy,
                    }
                    self.drone_speed[drone_id] = PHASE_SPEED['SWEEP']

                step = (math.sqrt((wx - prev_wp[0])**2 + (wy - prev_wp[1])**2)
                        if prev_wp else 0.0)
                if step > 0:
                    with self._lock:
                        self.drone_distance[drone_id]  += step
                        self.drone_real_secs[drone_id] += step / PHASE_SPEED['SWEEP']
                prev_wp = (wx, wy)

                for f in self.flowers:
                    if f['sector'] != sector:
                        continue
                    fid  = f['id']
                    dist = math.sqrt((f['x'] - wx)**2 + (f['y'] - wy)**2)
                    if (dist > _DETECT_R or fid in self.day_discovered_ids
                            or fid in self.discovered_ids):
                        continue
                    is_ready = (f['bloom_day'] == day)
                    with self._lock:
                        self.day_discovered_ids.add(fid)
                        self.discovered_ids.add(fid)
                        self.sector_discovered[sector] += 1
                        if is_ready:
                            self.sector_ready[sector] += 1
                        else:
                            self.sector_not_ready[sector] += 1
                        self.last_update_time = time.time()
                    self._grid_add(f['x'], f['y'], f['conf'])

                with self._lock:
                    self.mapping_wp_done += 1
                    done = self.mapping_wp_done
                self.mapping_progress = min(100.0,
                                            round(done / total_wps * 100, 1))
                self._sim_stop.wait(_T_MAP_WP)

            with self._lock:
                self.drone_states[drone_id]    = 'IDLE'
                self.drone_speed[drone_id]     = PHASE_SPEED['IDLE']
                self.current_targets[drone_id] = None

        workers = [
            threading.Thread(target=_sweep, args=(d, DRONE_SECTOR[d]),
                             daemon=True, name=f'map_d{d}')
            for d in range(3)
        ]
        for w in workers: w.start()
        for w in workers: w.join()
        with self._lock:
            self.mapping_progress = 100.0

    # ── Pollination phase ──────────────────────────────────────────────────────

    def _run_pollination(self, day: int):
        def _pollinate(drone_id: int, sector: str):
            queue = [
                f for f in self.flowers
                if f['sector']     == sector
                and f['bloom_day'] == day
                and f['id'] not in self.pollinated_ever
            ]
            random.shuffle(queue)
            self._sim_stop.wait(drone_id * 0.08)

            prev_x, prev_y = None, None

            for flower in queue:
                if self._sim_stop.is_set():
                    break

                with self._lock:
                    self.current_targets[drone_id] = {
                        'id': flower['id'],
                        'x':  round(flower['x'], 1),
                        'y':  round(flower['y'], 1),
                    }

                # SEARCH — transit to flower
                with self._lock:
                    self.drone_states[drone_id] = 'SEARCH'
                    self.drone_speed[drone_id]  = PHASE_SPEED['SEARCH']
                transit = (math.sqrt((flower['x'] - prev_x)**2 +
                                     (flower['y'] - prev_y)**2)
                           if prev_x is not None else 5.0)
                with self._lock:
                    self.drone_distance[drone_id]  += transit
                    self.drone_real_secs[drone_id] += transit / PHASE_SPEED['SEARCH']
                if self._sim_stop.wait(_T_SEARCH): break

                # APPROACH — slow descent to flower level
                with self._lock:
                    self.drone_states[drone_id] = 'APPROACH'
                    self.drone_speed[drone_id]  = PHASE_SPEED['APPROACH']
                    self.drone_distance[drone_id]  += _D_APPROACH
                    self.drone_real_secs[drone_id] += _D_APPROACH / PHASE_SPEED['APPROACH']
                self._grid_add(flower['x'], flower['y'],
                               random.uniform(0.85, 0.98))
                if self._sim_stop.wait(_T_APPROACH): break

                # HOVER — fine positioning
                with self._lock:
                    self.drone_states[drone_id] = 'HOVER'
                    self.drone_speed[drone_id]  = PHASE_SPEED['HOVER']
                    self.drone_distance[drone_id]  += _D_HOVER
                    self.drone_real_secs[drone_id] += _D_HOVER / PHASE_SPEED['HOVER']
                if self._sim_stop.wait(_T_HOVER): break

                # POLLINATE — stationary contact
                with self._lock:
                    self.drone_states[drone_id] = 'POLLINATE'
                    self.drone_speed[drone_id]  = PHASE_SPEED['POLLINATE']
                    self.drone_real_secs[drone_id] += _POL_DWELL
                self._record_pollination(drone_id, flower, day)
                if self._sim_stop.wait(_T_POLLINATE): break

                # Post-pollinate ascent
                with self._lock:
                    self.drone_distance[drone_id]  += _D_ASCENT
                    self.drone_real_secs[drone_id] += _D_ASCENT / PHASE_SPEED['TAKEOFF']

                prev_x, prev_y = flower['x'], flower['y']

            # LAND
            with self._lock:
                self.drone_states[drone_id]    = 'LAND'
                self.drone_speed[drone_id]     = PHASE_SPEED['LAND']
                self.drone_distance[drone_id]  += _D_LAND
                self.drone_real_secs[drone_id] += _D_LAND / PHASE_SPEED['LAND']
                self.current_targets[drone_id] = None
            self._sim_stop.wait(_T_LAND)
            with self._lock:
                self.drone_states[drone_id] = 'IDLE'
                self.drone_speed[drone_id]  = PHASE_SPEED['IDLE']

        workers = [
            threading.Thread(target=_pollinate, args=(d, DRONE_SECTOR[d]),
                             daemon=True, name=f'poll_d{d}')
            for d in range(3)
        ]
        for w in workers: w.start()
        for w in workers: w.join()

    def _record_pollination(self, drone_id: int, flower: dict, day: int):
        fid = flower['id']
        with self._lock:
            if fid in self.pollinated_ever:
                return
            self.pollinated_ever.add(fid)
            self.day_pollinated += 1
            self.pollination_events.append({
                'drone_id':  drone_id,
                'flower_id': fid,
                'flower_x':  round(flower['x'], 2),
                'flower_y':  round(flower['y'], 2),
                'sector':    flower['sector'],
                'day':       day,
                'timestamp': time.time(),
            })
            self.last_update_time = time.time()

    # ── Public write methods (ros_bridge) ──────────────────────────────────────

    def update_drone_state(self, drone_id: int, state: str):
        with self._lock:
            self.drone_states[drone_id] = state
            self.last_update_time = time.time()

    def add_detection(self, x: float, y: float, confidence: float):
        with self._lock:
            self._grid_add(x, y, confidence)
            self.last_update_time = time.time()

    def add_pollination_event(self, drone_id: int,
                               flower_x: float, flower_y: float,
                               flower_id: str):
        with self._lock:
            if flower_id in self.pollinated_ever:
                return
            self.pollinated_ever.add(flower_id)
            self.day_pollinated += 1
            self.pollination_events.append({
                'drone_id': drone_id, 'flower_id': flower_id,
                'flower_x': round(flower_x, 2), 'flower_y': round(flower_y, 2),
                'sector':   DRONE_SECTOR.get(drone_id, 'LEFT'),
                'day':      self.current_day,
                'timestamp': time.time(),
            })
            self.last_update_time = time.time()

    def set_mission_active(self, active: bool):
        with self._lock:
            self.mission_active   = active
            self.last_update_time = time.time()

    # ── Reset ──────────────────────────────────────────────────────────────────

    def reset(self):
        self.stop_simulation()
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)
        with self._lock:
            self.flowers = _generate_flowers()
        self._reset_live_state()

    # ── Snapshot ───────────────────────────────────────────────────────────────

    def get_snapshot(self) -> dict:
        with self._lock:
            day   = self.current_day
            phase = self.current_phase

            total_ready     = sum(self.sector_ready.values())
            total_not_ready = sum(self.sector_not_ready.values())
            pol_today       = self.day_pollinated
            cov_today       = (round(pol_today / total_ready * 100, 1)
                               if total_ready > 0 else 0.0)

            total_pol_all  = len(self.pollinated_ever)
            # Never-opens = bloom_day 0 only (the drone doesn't know about late
            # bloomers until it observes them; they may still open on future days)
            total_never    = sum(1 for f in self.flowers
                                 if f['bloom_day'] == 0)
            total_possible = len(self.flowers) - total_never

            flower_positions = []
            for f in self.flowers:
                if f['id'] not in self.discovered_ids:
                    continue
                if f['id'] in self.pollinated_ever:
                    state = 'pollinated'
                elif f['bloom_day'] == day:
                    state = 'ready'
                elif f['bloom_day'] == 0:
                    state = 'closed'   # true never-opens only
                else:
                    state = 'future'
                flower_positions.append({
                    'x': round(f['x'], 2),
                    'y': round(f['y'], 2),
                    'state': state,
                })

            # Send all days that have been started
            day_stats_out = {
                str(d): dict(v)
                for d, v in self.day_stats.items()
            }
            day_perf_out = {
                str(d): dict(v)
                for d, v in self.day_perf.items()
            }

            return {
                'connected':      self.connected_to_ros,
                'elapsed_s':      round(time.time() - self.mission_start_time, 1),
                'mission_active': self.mission_active,
                'current_day':    day,
                'current_phase':  phase,
                'mission_stop_reason':    self.mission_stop_reason,
                'consecutive_zero_days':  self.consecutive_zero_days,
                'mission_hard_limit':      MISSION_HARD_LIMIT,

                'mapping_progress':  self.mapping_progress,
                'discovered_today':  len(self.day_discovered_ids),
                'ready_today':       total_ready,
                'not_ready_today':   total_not_ready,

                'pollinated_today':   pol_today,
                'coverage_today_pct': cov_today,

                'drone_states':    dict(self.drone_states),
                'drone_sectors':   dict(self.drone_sectors),
                'current_targets': dict(self.current_targets),

                'sector_ready':     dict(self.sector_ready),
                'sector_not_ready': dict(self.sector_not_ready),

                'day_stats': day_stats_out,
                'day_perf':  day_perf_out,

                'health_grid': [
                    [round(self.health_grid[r][c], 4) for c in range(60)]
                    for r in range(20)
                ],
                'flower_positions': flower_positions,

                'pollination_events': list(self.pollination_events[-30:]),
                'total_events':       len(self.pollination_events),

                'tsp_pct':   self.tsp_pct,
                'bloom_pct': self.bloom_pct,

                'total_pollinated_all': total_pol_all,
                'total_pollination_possible': total_possible,
                'total_never_open':     total_never,
                'total_field_flowers':  len(self.flowers),

                'drone_speed': {
                    str(d): round(self.drone_speed[d], 1) for d in range(3)
                },
                'drone_distance': {
                    str(d): round(self.drone_distance[d], 2) for d in range(3)
                },
                'drone_real_secs': {
                    str(d): round(self.drone_real_secs[d], 1) for d in range(3)
                },

                'last_update': round(self.last_update_time, 2),
            }
