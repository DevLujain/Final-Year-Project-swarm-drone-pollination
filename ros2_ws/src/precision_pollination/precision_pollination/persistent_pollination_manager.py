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
            x        = float(fl['x'])
            y        = float(fl['y'])

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
