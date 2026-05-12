"""
mission_state.py  —  Stage 13 DaaS Dashboard
==============================================
Single shared-state object. Thread-safe.

Updated by:
  • ros_bridge.py  — live ROS 2 topics (when swarm is running)
  • _sim_loop      — built-in simulation engine (when Start pressed standalone)

Field layout
  60 m wide (x axis)  ×  20 m tall (y axis)
  Three equal columns:
    LEFT   x  0 – 20 m  → Drone 0
    CENTER x 20 – 40 m  → Drone 1
    RIGHT  x 40 – 60 m  → Drone 2

Flower generation
  Each run:  300 – 600 flowers, randomly distributed
  Bloom codes:
    0 = closed / never-opens  (skipped by bloom filter)
    1 = ready / target        (counted in total_flowers)
    2 = already open / wilted (not counted in targets)

Coverage denominator  =  bloom-1 flowers only  →  can never exceed 100 %

Simulation states per drone (in order):
  PREFLIGHT → ARMING → TAKEOFF
  then per flower:  SEARCH → APPROACH → HOVER → POLLINATE
  finally:  LAND → IDLE

No battery simulation.
"""

import random
import threading
import time
from pathlib import Path

# ── Field constants ────────────────────────────────────────────────────────────
FIELD_W = 60.0   # metres  (x)
FIELD_H = 20.0   # metres  (y)

SECTOR_NAMES = ['LEFT', 'CENTER', 'RIGHT']
DRONE_SECTOR = {0: 'LEFT', 1: 'CENTER', 2: 'RIGHT'}

# Simulation timing (seconds per phase per flower)
_T_STARTUP   = 1.0    # each startup state (PREFLIGHT / ARMING / TAKEOFF)
_T_SEARCH    = 0.6
_T_APPROACH  = 0.5
_T_HOVER     = 0.4
_T_POLLINATE = 0.5
_T_LAND      = 1.0

# Optional CSV from stage 12 (loaded if present)
_REGISTRY_CSV = Path.home() / 'Desktop/FYP/setup/flower_registry.csv'


# ── Flower generator ──────────────────────────────────────────────────────────

def _generate_flowers() -> list:
    """
    Generate a random field of 300–600 sunflowers.

    Flowers are placed randomly across the full 60×20 m area.
    x-position determines sector — so column counts differ every run.

    Returns a list of dicts:
        id      str    unique identifier  e.g. 'F0042'
        x       float  metres
        y       float  metres
        sector  str    'LEFT' | 'CENTER' | 'RIGHT'
        bloom   int    0=closed  1=ready  2=wilted
        conf    float  simulated YOLOv8 confidence  0.70–0.98
    """
    n = random.randint(300, 600)
    flowers = []
    for i in range(n):
        x = random.uniform(0.5, FIELD_W - 0.5)
        y = random.uniform(0.5, FIELD_H - 0.5)

        if x < 20.0:
            sector = 'LEFT'
        elif x < 40.0:
            sector = 'CENTER'
        else:
            sector = 'RIGHT'

        bloom = random.choices([1, 0, 2], weights=[78, 15, 7])[0]

        if bloom == 1:
            conf = random.uniform(0.84, 0.98)
        else:
            conf = random.uniform(0.70, 0.84)

        flowers.append({'id': f'F{i:04d}', 'x': x, 'y': y,
                        'sector': sector, 'bloom': bloom, 'conf': conf})
    return flowers


# ── Dynamic metrics ────────────────────────────────────────────────────────────

def _compute_metrics(flowers: list) -> tuple:
    """
    Compute TSP improvement % and Bloom filter gain % from the generated field.

    TSP: denser field → more path savings from nearest-neighbour + 2-opt.
    Bloom filter: fraction of flights saved by skipping closed flowers.

    Returns (tsp_pct, bloom_pct)  both rounded to 1 decimal.
    """
    total   = len(flowers)
    closed  = sum(1 for f in flowers if f['bloom'] == 0)
    ready   = total - closed

    # Bloom filter gain  =  fraction of total flights skipped
    bloom_pct = round((closed / total) * 100, 1) if total > 0 else 0.0

    # TSP improvement: more flowers → more savings from path optimisation
    # ready ranges ~234 – 468 (78 % of 300–600).
    # Formula maps that linearly to ~50 – 68 %, plus ±5 % run-to-run noise.
    tsp_base = 34.0 + (ready / 14.0)
    tsp_pct  = round(min(max(tsp_base + random.uniform(-5, 5), 42.0), 72.0), 1)

    return tsp_pct, bloom_pct


# ── MissionState ──────────────────────────────────────────────────────────────

class MissionState:

    def __init__(self):
        self._lock       = threading.Lock()
        self._sim_stop   = threading.Event()
        self._sim_thread = None

        # Flowers — generated fresh on every init / reset
        self.flowers        = _generate_flowers()
        self.total_flowers  = sum(1 for f in self.flowers if f['bloom'] == 1)

        # Drone states
        self.drone_states   = {0: 'IDLE', 1: 'IDLE', 2: 'IDLE'}
        self.drone_sectors  = {
            0: 'LEFT   (x < 20 m)',
            1: 'CENTER (20–40 m)',
            2: 'RIGHT  (x > 40 m)',
        }

        # Per-drone current target flower
        self.current_targets = {0: None, 1: None, 2: None}

        # Per-sector progress counters  {sector: {'pollinated': int, 'total': int}}
        self.sector_progress = self._build_sector_progress()

        # Pollination tracking
        self.pollination_events = []          # list of event dicts
        self.pollinated_ids     = set()       # set of flower id strings

        # Health grid: 60 cols × 20 rows  (one cell = 1 m²)
        self._grid_sum   = [[0.0] * 60 for _ in range(20)]
        self._grid_count = [[0]   * 60 for _ in range(20)]
        self.health_grid = [[0.0] * 60 for _ in range(20)]

        # Mission metrics
        self.tsp_pct   = None
        self.bloom_pct = None
        self._update_metrics()

        # Timing / flags
        self.mission_start_time = time.time()
        self.mission_active     = False
        self.mission_paused     = False
        self.connected_to_ros   = False
        self.last_update_time   = time.time()

        # Pre-seed health grid from initial flower confidences
        self._seed_health_grid()

        # Optionally load stage-12 CSV (non-fatal if absent)
        self._load_registry_csv()

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _build_sector_progress(self) -> dict:
        prog = {s: {'pollinated': 0, 'total': 0} for s in SECTOR_NAMES}
        for f in self.flowers:
            if f['bloom'] == 1:
                prog[f['sector']]['total'] += 1
        return prog

    def _update_metrics(self):
        tsp, bloom = _compute_metrics(self.flowers)
        self.tsp_pct   = tsp
        self.bloom_pct = bloom

    def _seed_health_grid(self):
        for f in self.flowers:
            if f['bloom'] == 0:
                continue
            self._grid_add(f['x'], f['y'], f['conf'])

    def _grid_add(self, x: float, y: float, conf: float):
        col = max(0, min(59, int(x)))
        row = max(0, min(19, int(y)))
        self._grid_sum[row][col]   += conf
        self._grid_count[row][col] += 1
        cnt = self._grid_count[row][col]
        self.health_grid[row][col] = self._grid_sum[row][col] / cnt

    def _load_registry_csv(self):
        if not _REGISTRY_CSV.exists():
            return
        try:
            import csv
            with open(_REGISTRY_CSV, newline='') as f:
                for row in csv.DictReader(f):
                    self.add_detection(
                        float(row.get('x', 0)),
                        float(row.get('y', 0)),
                        float(row.get('confidence', 0.85)),
                    )
        except Exception:
            pass

    # ── Simulation engine ──────────────────────────────────────────────────────

    def start_simulation(self):
        """Start the built-in sim engine.  Called by app.py on Start press."""
        if self._sim_thread and self._sim_thread.is_alive():
            return

        self._sim_stop.clear()
        with self._lock:
            self.mission_active     = True
            self.mission_start_time = time.time()

        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True, name='sim_loop')
        self._sim_thread.start()

    def stop_simulation(self):
        self._sim_stop.set()
        with self._lock:
            self.mission_active = False
            for d in range(3):
                self.drone_states[d]    = 'IDLE'
                self.current_targets[d] = None

    def _sim_loop(self):
        """
        Main simulation:
          Phase 1 (startup)  — all drones step through PREFLIGHT/ARMING/TAKEOFF
          Phase 2 (mission)  — 3 independent drone threads, each cycling
                               SEARCH→APPROACH→HOVER→POLLINATE per flower
          Phase 3 (land)     — drones land and go IDLE
        """
        # ── Phase 1: startup states (all drones together) ──────────────────────
        for state in ('PREFLIGHT', 'ARMING', 'TAKEOFF'):
            if self._sim_stop.is_set():
                return
            with self._lock:
                for d in range(3):
                    self.drone_states[d] = state
            self._sim_stop.wait(_T_STARTUP)

        # ── Phase 2: independent drone workers ─────────────────────────────────
        done_events = [threading.Event() for _ in range(3)]

        def _drone_worker(drone_id: int):
            sector   = DRONE_SECTOR[drone_id]
            queue    = [f for f in self.flowers
                        if f['sector'] == sector and f['bloom'] == 1]
            random.shuffle(queue)

            # Stagger drone start slightly so states aren't perfectly in sync
            self._sim_stop.wait(drone_id * 0.15)

            for flower in queue:
                if self._sim_stop.is_set():
                    break

                # Set current target
                with self._lock:
                    self.current_targets[drone_id] = {
                        'id': flower['id'],
                        'x':  round(flower['x'], 1),
                        'y':  round(flower['y'], 1),
                    }

                # SEARCH
                with self._lock:
                    self.drone_states[drone_id] = 'SEARCH'
                if self._sim_stop.wait(_T_SEARCH):
                    break

                # APPROACH  — detection fires here
                with self._lock:
                    self.drone_states[drone_id] = 'APPROACH'
                self.add_detection(flower['x'], flower['y'],
                                   random.uniform(0.84, 0.98))
                if self._sim_stop.wait(_T_APPROACH):
                    break

                # HOVER
                with self._lock:
                    self.drone_states[drone_id] = 'HOVER'
                if self._sim_stop.wait(_T_HOVER):
                    break

                # POLLINATE  — event logged here
                with self._lock:
                    self.drone_states[drone_id] = 'POLLINATE'
                self._record_pollination(drone_id, flower)
                if self._sim_stop.wait(_T_POLLINATE):
                    break

            # LAND then IDLE
            with self._lock:
                self.drone_states[drone_id]    = 'LAND'
                self.current_targets[drone_id] = None
            self._sim_stop.wait(_T_LAND)
            with self._lock:
                self.drone_states[drone_id] = 'IDLE'

            done_events[drone_id].set()

        # Launch all 3 drone workers simultaneously
        workers = [
            threading.Thread(target=_drone_worker, args=(d,),
                             daemon=True, name=f'drone_{d}')
            for d in range(3)
        ]
        for w in workers:
            w.start()
        for w in workers:
            w.join()

        # ── Phase 3: mission finished ──────────────────────────────────────────
        with self._lock:
            self.mission_active = False

    def _record_pollination(self, drone_id: int, flower: dict):
        fid = flower['id']
        with self._lock:
            if fid in self.pollinated_ids:
                return
            self.pollinated_ids.add(fid)
            sector = flower['sector']
            self.sector_progress[sector]['pollinated'] += 1
            self.pollination_events.append({
                'drone_id':  drone_id,
                'flower_id': fid,
                'flower_x':  round(flower['x'], 2),
                'flower_y':  round(flower['y'], 2),
                'sector':    sector,
                'timestamp': time.time(),
            })
            self.last_update_time = time.time()

    # ── Public write methods (used by ros_bridge) ──────────────────────────────

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
        """Called by ros_bridge for live ROS 2 pollination events."""
        with self._lock:
            if flower_id in self.pollinated_ids:
                return
            self.pollinated_ids.add(flower_id)
            sector = DRONE_SECTOR.get(drone_id, 'LEFT')
            self.sector_progress[sector]['pollinated'] += 1
            self.pollination_events.append({
                'drone_id':  drone_id,
                'flower_id': flower_id,
                'flower_x':  round(flower_x, 2),
                'flower_y':  round(flower_y, 2),
                'sector':    sector,
                'timestamp': time.time(),
            })
            self.last_update_time = time.time()

    def set_mission_active(self, active: bool):
        with self._lock:
            self.mission_active   = active
            self.last_update_time = time.time()

    # ── Reset ──────────────────────────────────────────────────────────────────

    def reset(self):
        """
        Full reset — stops simulation, generates a completely new random field.
        Press Start again after reset to begin a new mission.
        """
        self.stop_simulation()
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)

        with self._lock:
            # New random field
            self.flowers           = _generate_flowers()
            self.total_flowers     = sum(1 for f in self.flowers if f['bloom'] == 1)
            self.sector_progress   = self._build_sector_progress()

            # Reset tracking
            self.pollinated_ids     = set()
            self.pollination_events = []
            self.current_targets    = {0: None, 1: None, 2: None}
            self.drone_states       = {0: 'IDLE', 1: 'IDLE', 2: 'IDLE'}

            # Reset health grid
            self._grid_sum   = [[0.0] * 60 for _ in range(20)]
            self._grid_count = [[0]   * 60 for _ in range(20)]
            self.health_grid = [[0.0] * 60 for _ in range(20)]

            # Reset flags
            self.mission_active     = False
            self.mission_paused     = False
            self.mission_start_time = time.time()
            self.last_update_time   = time.time()

        # Recompute metrics for new field
        self._update_metrics()
        self._seed_health_grid()

    # ── Snapshot ───────────────────────────────────────────────────────────────

    def get_snapshot(self) -> dict:
        """Return all state as a JSON-safe dict.  Called by Flask every second."""
        with self._lock:
            pollinated = len(self.pollinated_ids)
            total      = self.total_flowers
            coverage   = round(pollinated / total * 100, 1) if total > 0 else 0.0
            elapsed    = round(time.time() - self.mission_start_time, 1)

            # Sector counts for field stats bar
            sector_totals = {s: 0 for s in SECTOR_NAMES}
            for f in self.flowers:
                if f['bloom'] == 1:
                    sector_totals[f['sector']] += 1

            return {
                # Connection / timing
                'connected':          self.connected_to_ros,
                'elapsed_s':          elapsed,
                'mission_active':     self.mission_active,
                'mission_paused':     self.mission_paused,

                # Coverage
                'coverage_pct':       coverage,
                'flowers_pollinated': pollinated,
                'total_flowers':      total,

                # Drones
                'drone_states':       dict(self.drone_states),
                'drone_sectors':      dict(self.drone_sectors),
                'current_targets':    dict(self.current_targets),
                'sector_progress':    {s: dict(v)
                                       for s, v in self.sector_progress.items()},

                # Events (latest 30 for log table)
                'pollination_events': list(self.pollination_events[-30:]),

                # Health grid (20 rows × 60 cols)
                'health_grid': [
                    [round(self.health_grid[r][c], 4) for c in range(60)]
                    for r in range(20)
                ],

                # Flower positions for canvas dots
                'flower_positions': [
                    {
                        'x':         round(f['x'], 2),
                        'y':         round(f['y'], 2),
                        'sector':    f['sector'],
                        'bloom':     f['bloom'],
                        'pollinated': f['id'] in self.pollinated_ids,
                    }
                    for f in self.flowers
                ],

                # Metrics
                'tsp_pct':            self.tsp_pct,
                'bloom_pct':          self.bloom_pct,
                'sector_totals':      sector_totals,
                'total_field_flowers': len(self.flowers),
                'last_update':        round(self.last_update_time, 2),
            }
