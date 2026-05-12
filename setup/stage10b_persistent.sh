#!/bin/bash
# ============================================================
# STAGE 10b — Persistent Multi-Run Pollination System
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS ADDS:
#   Realistic field of ~300 flowers across 3 drone sectors.
#   Each flower has a permanent ID, fixed position, and a
#   bloom day (1, 2, or 3). The system runs 3 automatic
#   sweeps with a gap between each, simulating three
#   separate real-world deployment days.
#
#   A flower pollinated in Run 1 is NEVER targeted again —
#   this persists across sessions via CSV on disk.
#   New flowers that opened since the last run are the
#   only targets each time.
#
# FILES CREATED:
#   ~/Desktop/FYP/setup/flower_registry.csv
#     — permanent record of all 300 flowers (never changes)
#   ~/Desktop/FYP/setup/pollination_log.csv
#     — appended every run, full history across all sessions
#
# SIMULATION FLOW (one continuous session):
#   Run 1 → ~70 Day-1 flowers pollinated → 60s gap
#   Run 2 → ~100 Day-2 flowers pollinated → 60s gap
#   Run 3 → ~90 Day-3 flowers pollinated → session ends
#   (Previously pollinated flowers excluded from every run)
#
# PRE-REQUISITES:
#   ✅ Stage 9 complete
#   ✅ Stage 10 readiness filter in place
#
# RUN: bash stage10b_persistent.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
SETUP_DIR=$FYP_DIR/setup
ROS2_WS=$FYP_DIR/ros2_ws
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination

mkdir -p $SETUP_DIR

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 10b — Persistent Multi-Run Pollination       ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Generate flower registry ───────────────────────────────
echo "[1/4] Generating flower registry (300 flowers)..."

python3 << 'PYEOF'
"""
Flower Registry Generator
==========================
Creates a realistic random sunflower field with ~300 flowers
distributed across 3 drone sectors.

Field layout (metres):
  Total field: 60m wide x 20m deep
  Sector 0 (Drone 0): x= 0–20m
  Sector 1 (Drone 1): x=20–40m
  Sector 2 (Drone 2): x=40–60m

Planting density: ~5 plants/m² with ±30% natural variation
in spacing (sunflowers are never perfectly regular in a real
field — wind, germination rate, and soil variation cause gaps).

Bloom schedule:
  Day 1: ~23% of flowers open  (early bloomers)
  Day 2: ~40% of flowers open  (peak bloom)
  Day 3: ~30% of flowers open  (late bloomers)
  Never: ~7% never open during mission window (damaged/failed)

Academic note:
  Sunflower anthesis (bloom opening) typically spans 10–14
  days across a field, with staggered opening driven by
  microclimate variation (Schneiter & Miller, 1981).
  The 3-day schedule compresses this for simulation purposes.
"""

import csv
import random
import math
import os
from pathlib import Path

random.seed(42)   # reproducible field layout

SETUP_DIR    = Path.home() / "Desktop/FYP/setup"
REGISTRY_CSV = SETUP_DIR / "flower_registry.csv"

# Field dimensions per sector (metres)
SECTOR_DEFS = {
    0: {"x_min":  0.5, "x_max": 19.5, "label": "LEFT"},
    1: {"x_min": 20.5, "x_max": 39.5, "label": "CENTER"},
    2: {"x_min": 40.5, "x_max": 59.5, "label": "RIGHT"},
}
Y_MIN, Y_MAX = 0.5, 19.5

# Target ~100 flowers per sector, with natural variation
FLOWERS_PER_SECTOR = {"LEFT": 98, "CENTER": 103, "RIGHT": 99}

# Bloom day distribution (weights)
BLOOM_WEIGHTS = {1: 0.23, 2: 0.40, 3: 0.30, 0: 0.07}
# Day 0 = never opens during mission window

def poisson_disk_sample(x_min, x_max, y_min, y_max, n_target, min_dist=0.8):
    """
    Generate n_target points with minimum spacing of min_dist metres.
    Uses rejection sampling — realistic irregular spacing.
    0.8m minimum = typical sunflower head clearance at commercial density.
    """
    points = []
    attempts = 0
    max_attempts = n_target * 50

    while len(points) < n_target and attempts < max_attempts:
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        # Check minimum distance from all existing points
        too_close = any(
            math.sqrt((x - px)**2 + (y - py)**2) < min_dist
            for px, py in points
        )
        if not too_close:
            points.append((x, y))
        attempts += 1

    return points

print("  Generating flower positions (Poisson disk sampling)...")
all_flowers = []
flower_id   = 1

for sector_id, sector in SECTOR_DEFS.items():
    label    = sector["label"]
    n_target = FLOWERS_PER_SECTOR[label]
    points   = poisson_disk_sample(
        sector["x_min"], sector["x_max"],
        Y_MIN, Y_MAX,
        n_target
    )

    day_choices = (
        [1] * int(n_target * BLOOM_WEIGHTS[1]) +
        [2] * int(n_target * BLOOM_WEIGHTS[2]) +
        [3] * int(n_target * BLOOM_WEIGHTS[3]) +
        [0] * max(1, int(n_target * BLOOM_WEIGHTS[0]))
    )
    # Pad or trim to match actual point count
    while len(day_choices) < len(points):
        day_choices.append(random.choice([1, 2, 3]))
    day_choices = day_choices[:len(points)]
    random.shuffle(day_choices)

    for (x, y), bloom_day in zip(points, day_choices):
        all_flowers.append({
            "flower_id":  f"F{flower_id:03d}",
            "sector":     label,
            "drone_id":   sector_id,
            "x":          round(x, 2),
            "y":          round(y, 2),
            "bloom_day":  bloom_day,
        })
        flower_id += 1
    print(f"    Sector {label}: {len(points)} flowers placed")

# Write registry CSV
with open(REGISTRY_CSV, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=[
        "flower_id", "sector", "drone_id", "x", "y", "bloom_day"
    ])
    writer.writeheader()
    writer.writerows(all_flowers)

# Summary
total = len(all_flowers)
by_day = {d: sum(1 for fl in all_flowers if fl["bloom_day"] == d) for d in [0,1,2,3]}
print(f"\n  Total flowers: {total}")
print(f"  Day 1 (early bloom): {by_day[1]}")
print(f"  Day 2 (peak bloom):  {by_day[2]}")
print(f"  Day 3 (late bloom):  {by_day[3]}")
print(f"  Never open:          {by_day[0]}")
print(f"\n  Registry saved: {REGISTRY_CSV}")
PYEOF

echo "✓ Flower registry generated"

# ── 2. Create the persistent pollination manager node ─────────
echo ""
echo "[2/4] Creating persistent_pollination_manager node..."

cat > $NODES_DIR/persistent_pollination_manager.py << 'PYEOF'
#!/usr/bin/env python3
"""
Persistent Pollination Manager — Stage 10b
============================================
Central node that manages the full multi-run pollination
lifecycle across all 3 drones and across sessions.

RESPONSIBILITIES:
  1. Load flower_registry.csv (300 flowers, permanent IDs)
  2. Load pollination_log.csv (history from all previous runs)
  3. Determine current run number and bloom day
  4. Publish target lists per drone (only: bloom_ready AND
     not yet pollinated in any previous run)
  5. Subscribe to pollination events → immediately append
     to pollination_log.csv with timestamp and run ID
  6. After each run completes, wait GAP_SECONDS then
     automatically start the next run with newly-opened flowers
  7. Print clear visual banners at each stage for screenshots

PUBLISHED TOPICS:
  /mission/run_status        — String (JSON current run info)
  /drone_N/persistent_targets — String (JSON flower list)

SUBSCRIBED TOPICS:
  /drone_N/pollination/log   — String (JSON pollination events)

CSV COLUMNS (pollination_log.csv):
  run_id, flower_id, sector, drone_id, x, y,
  bloom_day, pollinated_at, status

The CSV appends — never overwrites. Every session adds new
rows with a new run_id, making the full history visible.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import json
import time
import os
import random
from datetime import datetime
from pathlib import Path

# ── Configuration ──────────────────────────────────────────────
SETUP_DIR     = Path.home() / "Desktop/FYP/setup"
REGISTRY_CSV  = SETUP_DIR / "flower_registry.csv"
LOG_CSV       = SETUP_DIR / "pollination_log.csv"

TOTAL_RUNS    = 3       # Day 1, Day 2, Day 3
GAP_SECONDS   = 60      # simulated gap between deployment days
# In real life this would be days — compressed to 60s for demo

# Drone speed proxy: seconds per flower approach + pollination
POLLINATION_DELAY = 3.0  # seconds per flower in simulation

# Visual separator for terminal screenshots
SEP = "═" * 54


class PersistentPollinationManager(Node):

    def __init__(self):
        super().__init__('persistent_pollination_manager')

        self.setup_dir = SETUP_DIR
        self.setup_dir.mkdir(parents=True, exist_ok=True)

        # Load flower registry
        self.registry = self._load_registry()
        if not self.registry:
            self.get_logger().error(
                f'flower_registry.csv not found at {REGISTRY_CSV}. '
                f'Run stage10b_persistent.sh first.'
            )
            return

        # Load existing pollination history
        self.pollinated_ids = self._load_pollinated_ids()

        # Determine run number from history
        self.run_number = self._get_next_run_number()

        # Session state
        self.current_day    = 1
        self.session_done   = set()   # pollinated THIS session
        self.run_targets    = {}      # drone_id → list of flowers
        self.run_counts     = {0: 0, 1: 0, 2: 0}
        self.run_start_time = None

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/mission/run_status', 10)
        self.target_pubs = {
            i: self.create_publisher(
                String, f'/drone_{i}/persistent_targets', 10)
            for i in range(3)
        }

        # Subscribers — pollination events from all 3 drones
        for drone_id in range(3):
            self.create_subscription(
                String,
                f'/drone_{drone_id}/pollination/log',
                lambda msg, d=drone_id: self._on_pollination(msg, d),
                10
            )

        # Initialise log CSV if it doesn't exist
        self._init_log_csv()

        # Start first run after a short boot delay
        self.create_timer(5.0, self._boot)
        self._booted = False

        self.get_logger().info(
            f'Persistent pollination manager started.\n'
            f'  Registry: {len(self.registry)} flowers\n'
            f'  Previously pollinated: {len(self.pollinated_ids)}\n'
            f'  Next run: RUN_{self.run_number}'
        )

    # ── Startup ───────────────────────────────────────────────
    def _boot(self):
        if self._booted:
            return
        self._booted = True
        self._start_run(day=1)

    # ── CSV helpers ───────────────────────────────────────────
    def _load_registry(self):
        if not REGISTRY_CSV.exists():
            return []
        with open(REGISTRY_CSV, newline='') as f:
            return list(csv.DictReader(f))

    def _load_pollinated_ids(self):
        if not LOG_CSV.exists():
            return set()
        with open(LOG_CSV, newline='') as f:
            reader = csv.DictReader(f)
            return {
                row['flower_id']
                for row in reader
                if row.get('status') == 'POLLINATED'
            }

    def _get_next_run_number(self):
        if not LOG_CSV.exists():
            return 1
        with open(LOG_CSV, newline='') as f:
            reader = csv.DictReader(f)
            run_ids = {row.get('run_id', '') for row in reader}
        existing = [r for r in run_ids if r.startswith('RUN_')]
        if not existing:
            return 1
        nums = [int(r.split('_')[1]) for r in existing if r.split('_')[1].isdigit()]
        return max(nums) + 1 if nums else 1

    def _init_log_csv(self):
        if not LOG_CSV.exists():
            with open(LOG_CSV, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'run_id', 'flower_id', 'sector', 'drone_id',
                    'x', 'y', 'bloom_day', 'pollinated_at', 'status'
                ])

    def _append_log(self, run_id, flower, drone_id, status):
        with open(LOG_CSV, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                run_id,
                flower['flower_id'],
                flower['sector'],
                drone_id,
                flower['x'],
                flower['y'],
                flower['bloom_day'],
                datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                status
            ])

    # ── Run management ────────────────────────────────────────
    def _start_run(self, day: int):
        self.current_day    = day
        self.run_id         = f'RUN_{self.run_number}'
        self.run_start_time = time.time()
        self.run_counts     = {0: 0, 1: 0, 2: 0}

        # Select targets: bloom_day == day AND not previously pollinated
        all_targets = [
            fl for fl in self.registry
            if int(fl['bloom_day']) == day
            and fl['flower_id'] not in self.pollinated_ids
            and fl['flower_id'] not in self.session_done
        ]

        # Split by drone sector
        self.run_targets = {0: [], 1: [], 2: []}
        for fl in all_targets:
            self.run_targets[int(fl['drone_id'])].append(fl)

        total = len(all_targets)
        skipped = len(self.pollinated_ids) + len(self.session_done)

        # ── Visual banner ──────────────────────────────────────
        print(f'\n╔{SEP}╗')
        print(f'║  RUN {day} — DAY {day}  │  {self.run_id}' + ' ' * (54 - 22 - len(self.run_id)) + '║')
        print(f'║  {total} flowers bloom-ready today' + ' ' * (54 - 30 - len(str(total))) + '║')
        if skipped > 0:
            print(f'║  {skipped} previously pollinated → EXCLUDED' + ' ' * max(0, 54 - 38 - len(str(skipped))) + '║')
        print(f'╠{SEP}╣')
        for d in range(3):
            n = len(self.run_targets[d])
            sector = ['LEFT', 'CENTER', 'RIGHT'][d]
            print(f'║  Drone {d} ({sector:6s}): {n:3d} targets' + ' ' * max(0, 54 - 28 - len(str(n))) + '║')
        print(f'╚{SEP}╝')

        if total == 0:
            self.get_logger().info(
                f'No bloom-ready flowers for Day {day}. '
                f'Skipping to next run.'
            )
            self._finish_run()
            return

        # Publish targets to each drone
        for drone_id in range(3):
            targets = self.run_targets[drone_id]
            msg = String()
            msg.data = json.dumps({
                'run_id':   self.run_id,
                'day':      day,
                'drone_id': drone_id,
                'targets':  targets
            })
            self.target_pubs[drone_id].publish(msg)

        # Publish run status
        self._publish_status(total, skipped)

        # Simulate pollination (timer-driven since we have no real flight)
        self._simulate_pollination_run(all_targets)

    def _simulate_pollination_run(self, targets):
        """
        In simulation: process each flower with a short delay
        to mimic drone approach + pollinate time.
        Publishes each event as it happens so the terminal
        shows a live stream of pollination events.
        """
        random.shuffle(targets)

        for fl in targets:
            drone_id = int(fl['drone_id'])
            fid      = fl['flower_id']
            x        = fl['x']
            y        = fl['y']

            # Simulate flight time to flower
            time.sleep(POLLINATION_DELAY)

            # Record
            self.pollinated_ids.add(fid)
            self.session_done.add(fid)
            self.run_counts[drone_id] += 1
            self._append_log(self.run_id, fl, drone_id, 'POLLINATED')

            # Print live event
            total_this_drone = len(self.run_targets[drone_id])
            done_this_drone  = self.run_counts[drone_id]
            print(
                f'  [{fid}] Drone {drone_id} ({["LEFT","CENTER","RIGHT"][drone_id]:6s}) '
                f'→ POLLINATED  '
                f'({x:5.1f}m, {y:5.1f}m)  '
                f'[{done_this_drone}/{total_this_drone}]  ✓'
            )

            # Publish status update
            self._publish_status(len(targets), len(self.pollinated_ids))

        self._finish_run()

    def _finish_run(self):
        elapsed = time.time() - self.run_start_time if self.run_start_time else 0
        done    = sum(self.run_counts.values())

        print(f'\n╔{SEP}╗')
        print(f'║  RUN {self.current_day} COMPLETE' + ' ' * (54 - 14) + '║')
        print(f'║  Flowers pollinated this run : {done}' + ' ' * max(0, 54 - 32 - len(str(done))) + '║')
        print(f'║  Total pollinated all runs   : {len(self.pollinated_ids)}' + ' ' * max(0, 54 - 32 - len(str(len(self.pollinated_ids)))) + '║')
        print(f'║  Time elapsed                : {elapsed:.0f}s' + ' ' * max(0, 54 - 32 - len(str(int(elapsed)))) + '║')
        print(f'╠{SEP}╣')
        for d in range(3):
            sector = ['LEFT', 'CENTER', 'RIGHT'][d]
            n = self.run_counts[d]
            print(f'║  Drone {d} ({sector:6s}): {n:3d} pollinated' + ' ' * max(0, 54 - 29 - len(str(n))) + '║')
        print(f'╚{SEP}╝')

        self.run_number += 1
        next_day = self.current_day + 1

        if next_day <= TOTAL_RUNS:
            print(f'\n  ⏳  Simulating {GAP_SECONDS}s gap (= next deployment day)...')
            print(f'      New flowers will open. Previously pollinated flowers')
            print(f'      will be excluded even though they may still be open.\n')
            time.sleep(GAP_SECONDS)
            self._start_run(day=next_day)
        else:
            self._mission_complete()

    def _mission_complete(self):
        total_field  = len(self.registry)
        total_done   = len(self.pollinated_ids)
        never_open   = sum(1 for fl in self.registry if int(fl['bloom_day']) == 0)
        coverage_pct = total_done / (total_field - never_open) * 100

        print(f'\n╔{SEP}╗')
        print(f'║  MISSION COMPLETE — ALL 3 RUNS DONE' + ' ' * (54 - 37) + '║')
        print(f'╠{SEP}╣')
        print(f'║  Total flowers in field      : {total_field}' + ' ' * max(0, 54 - 32 - len(str(total_field))) + '║')
        print(f'║  Flowers that never opened   : {never_open}' + ' ' * max(0, 54 - 32 - len(str(never_open))) + '║')
        print(f'║  Total pollinated (all runs) : {total_done}' + ' ' * max(0, 54 - 32 - len(str(total_done))) + '║')
        print(f'║  Field coverage              : {coverage_pct:.1f}%' + ' ' * max(0, 54 - 35 - len(f"{coverage_pct:.1f}")) + '║')
        print(f'╠{SEP}╣')
        print(f'║  Full log saved to:' + ' ' * (54 - 20) + '║')
        print(f'║  ~/Desktop/FYP/setup/pollination_log.csv' + ' ' * (54 - 42) + '║')
        print(f'╚{SEP}╝\n')

    def _on_pollination(self, msg: String, drone_id: int):
        """
        Receives real pollination events from lawnmower_sweep
        or pollination_controller (when running with real PX4).
        In pure simulation mode _simulate_pollination_run handles this.
        """
        try:
            data = json.loads(msg.data)
            fid  = data.get('flower_id')
            if fid and fid not in self.pollinated_ids:
                flower = next(
                    (fl for fl in self.registry if fl['flower_id'] == fid),
                    None
                )
                if flower:
                    self.pollinated_ids.add(fid)
                    self.session_done.add(fid)
                    self._append_log(self.run_id, flower, drone_id, 'POLLINATED')
        except (json.JSONDecodeError, KeyError):
            pass

    def _publish_status(self, targets_today: int, excluded: int):
        status = {
            'run_id':           self.run_id,
            'day':              self.current_day,
            'targets_today':    targets_today,
            'excluded':         excluded,
            'total_pollinated': len(self.pollinated_ids),
            'timestamp':        datetime.now().isoformat()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersistentPollinationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
PYEOF

echo "✓ persistent_pollination_manager.py written"

# ── 3. Update setup.py to register new node ──────────────────
echo ""
echo "[3/4] Registering node and rebuilding workspace..."

cat > $ROS2_WS/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.10.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system — Stage 10b',
    entry_points={
        'console_scripts': [
            # Stage 5
            'pollination_controller        = precision_pollination.pollination_controller:main',
            'yolov8_detector               = precision_pollination.yolov8_detector:main',
            'swarm_coordinator             = precision_pollination.swarm_coordinator:main',
            # Stage 7
            'camera_bridge                 = precision_pollination.camera_bridge:main',
            'position_estimator            = precision_pollination.position_estimator:main',
            'mission_logger                = precision_pollination.mission_logger:main',
            # Stage 7b
            'flower_detector_sim           = precision_pollination.flower_detector_sim:main',
            # Stage 9
            'lawnmower_sweep               = precision_pollination.lawnmower_sweep:main',
            'shared_visited_list           = precision_pollination.shared_visited_list:main',
            # Stage 10
            'flower_readiness_filter       = precision_pollination.flower_readiness_filter:main',
            'readiness_logger              = precision_pollination.readiness_logger:main',
            # Stage 10b
            'persistent_pollination_manager = precision_pollination.persistent_pollination_manager:main',
        ],
    },
)
SETUPEOF

source /opt/ros/jazzy/setup.bash
cd $ROS2_WS
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null || true
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -10

# Patch executables
mkdir -p $ROS2_WS/install/precision_pollination/lib/precision_pollination

ALL_NODES=(
    pollination_controller yolov8_detector swarm_coordinator
    camera_bridge position_estimator mission_logger
    flower_detector_sim
    lawnmower_sweep shared_visited_list
    flower_readiness_filter readiness_logger
    persistent_pollination_manager
)

for NODE in "${ALL_NODES[@]}"; do
cat > $ROS2_WS/install/precision_pollination/lib/precision_pollination/$NODE << NODEEOF
#!/usr/bin/env python3
import sys
sys.path.insert(0, '$ROS2_WS/src/precision_pollination')
from precision_pollination.$NODE import main
main()
NODEEOF
chmod +x $ROS2_WS/install/precision_pollination/lib/precision_pollination/$NODE
done

echo "✓ All nodes registered (${#ALL_NODES[@]} total)"

# ── 4. Create launch script ───────────────────────────────────
echo ""
echo "[4/4] Creating launch_stage10b.sh..."

cat > $FYP_DIR/launch_stage10b.sh << 'LAUNCHEOF'
#!/bin/bash
# ============================================================
# LAUNCH STAGE 10b — Persistent Multi-Run Pollination
# FYP: Autonomous Swarm Drone Pollination
#
# Runs 3 automatic pollination sweeps in one session.
# Each sweep targets only newly-opened flowers.
# All results are saved to CSV and persist across sessions.
#
# WHAT TO SCREENSHOT FOR YOUR REPORT:
#   1. The RUN 1 / RUN 2 / RUN 3 banners in terminal
#   2. The live flower-by-flower pollination stream
#   3. The MISSION COMPLETE summary
#   4. ~/Desktop/FYP/setup/pollination_log.csv opened in
#      LibreOffice Calc — shows all 3 runs with timestamps
#
# RUN: bash ~/Desktop/FYP/launch_stage10b.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 10b — Persistent Multi-Run Pollination       ║"
echo "║  3 automatic sweeps — 300 flowers — CSV logging     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  Field:    300 flowers across 3 sectors"
echo "  Sweeps:   3 automatic runs (60s gap between each)"
echo "  Log:      ~/Desktop/FYP/setup/pollination_log.csv"
echo ""
echo "  Flowers pollinated in Run 1 will NOT appear in Run 2."
echo "  Flowers pollinated in Run 2 will NOT appear in Run 3."
echo "  This persists even if you restart the script."
echo ""
echo "  Starting in 3 seconds..."
sleep 3

ros2 run precision_pollination persistent_pollination_manager
LAUNCHEOF

chmod +x $FYP_DIR/launch_stage10b.sh
echo "✓ launch_stage10b.sh written"

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 10b COMPLETE                                  ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "FILES CREATED:"
echo "  ~/Desktop/FYP/setup/flower_registry.csv"
echo "    300 flowers: ID, sector, drone, x, y, bloom_day"
echo ""
echo "  ~/Desktop/FYP/setup/pollination_log.csv"
echo "    Appended each run — never overwritten"
echo "    Columns: run_id, flower_id, sector, drone_id,"
echo "             x, y, bloom_day, pollinated_at, status"
echo ""
echo "HOW TO RUN:"
echo "  bash ~/Desktop/FYP/launch_stage10b.sh"
echo ""
echo "WHAT YOU WILL SEE:"
echo "  ╔══ RUN 1 — DAY 1 | RUN_1 ══╗"
echo "  ║  71 flowers bloom-ready    ║"
echo "  ║  Drone 0 (LEFT):   24 targets"
echo "  [F003] Drone 0 → POLLINATED (3.2m, 7.8m) [1/24] ✓"
echo "  [F007] Drone 1 → POLLINATED (22.1m, 4.3m) [1/23] ✓"
echo "  ..."
echo "  ╔══ RUN 1 COMPLETE ══╗"
echo "  ║  71 pollinated     ║"
echo "  ⏳ 60s gap..."
echo "  ╔══ RUN 2 — DAY 2 ══╗"
echo "  ║  71 previously pollinated → EXCLUDED"
echo "  ║  98 new flowers bloom-ready"
echo "  ..."
echo ""
echo "TO RESET (start fresh, clear all history):"
echo "  rm ~/Desktop/FYP/setup/pollination_log.csv"
echo "  bash ~/Desktop/FYP/launch_stage10b.sh"
echo ""
echo "NEXT: bash stage11_tsp.sh"
echo ""
