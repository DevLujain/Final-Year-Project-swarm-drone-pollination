#!/usr/bin/env python3
"""
KPI Logger — Stage 12 (Realistic 300-Flower Field)
====================================================
Tracks quantitative KPIs across all 3 pollination runs (days).

The realistic scenario has ~300 flowers visited across 3 runs:
    Run 1 (Day 1): ~70 bloom-ready flowers
    Run 2 (Day 2): ~100 new flowers (Run 1 excluded)
    Run 3 (Day 3): ~90 new flowers (Runs 1+2 excluded)

This logger reads from the same CSVs as the rest of the pipeline
(flower_registry.csv + pollination_log.csv) and also listens to
live ROS 2 events. It tracks KPIs per-run and cumulatively.

KPI TARGETS (from FYP proposal):
    mAP@0.5 (detection)   ≥ 0.85    achieved: 0.957
    Bloom readiness mAP   ≥ 0.80    achieved: 0.994
    Field coverage        ≥ 90%     per 3 runs combined
    No re-pollination     = 0       across all 3 runs
    TSP improvement       > 20%     achieved: 64.7%
    Bloom filter gain     > 10%     achieved: 33.3%

OUTPUTS (saved on Ctrl+C):
    kpi_report.txt    — full report (paste into FYP Chapter 5)
    kpi_report.json   — machine-readable

HOW TO RUN:
    # Alongside launch_stage11.sh or any stage with pollination:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/FYP/ros2_ws/install/setup.bash
    ros2 run precision_pollination kpi_logger

    # Offline (reads existing CSVs only, saves report, exits):
    python3 kpi_logger.py --offline
"""

import os
import csv
import json
import time
import argparse
from datetime import datetime
from pathlib import Path

# ── KPI definitions ───────────────────────────────────────────────────────────
KPI_DETECTION_MAP50    = 0.85    # mAP@0.5 target (sunflower detection)
KPI_READINESS_MAP50    = 0.80    # mAP@0.5 target (bloom readiness classifier)
KPI_COVERAGE           = 90.0   # % of all flowers across 3 runs
KPI_REPOLLINATION      = 0      # max allowed re-pollinations
KPI_TSP_IMPROVEMENT    = 20.0   # % path reduction target
KPI_BLOOM_FILTER_GAIN  = 10.0   # % efficiency gain target

# Achieved values (from Stage 6 and Stage 10)
ACHIEVED_DETECTION_MAP50   = 0.957
ACHIEVED_DETECTION_PREC    = 0.807
ACHIEVED_DETECTION_RECALL  = 0.784
ACHIEVED_READINESS_MAP50   = 0.994
ACHIEVED_READINESS_PREC    = 0.990
ACHIEVED_READINESS_RECALL  = 0.992
ACHIEVED_TSP_IMPROVEMENT   = 64.7
ACHIEVED_BLOOM_FILTER_GAIN = 33.3

# File paths
SETUP_DIR    = Path.home() / 'Desktop/FYP/setup'
REGISTRY_CSV = SETUP_DIR / 'flower_registry.csv'
LOG_CSV      = SETUP_DIR / 'pollination_log.csv'
OUTPUT_DIR   = Path(os.environ.get(
    'STAGE12_OUTPUT_DIR',
    str(Path.home() / 'Desktop/FYP/mission_outputs/latest')
))

# Print status every N seconds
STATUS_INTERVAL = 30


class KPILogger:
    """
    Core logic — works in both ROS 2 mode and offline mode.
    """

    def __init__(self):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.start_time = time.time()

        # Load registry to know total flower count and bloom schedule
        self.flowers      = self._load_registry()
        self.total_flowers = len(self.flowers)
        self.flower_by_id = {fl['flower_id']: fl for fl in self.flowers}

        if not self.flowers:
            raise FileNotFoundError(
                f'flower_registry.csv not found at {REGISTRY_CSV}.\n'
                f'Run stage10b_persistent.sh first.'
            )

        # Track pollinated flowers per run
        # run_id (str) → set of flower_ids
        self.per_run: dict = {}
        # Track all flower_ids ever pollinated (for re-pollination detection)
        self.all_pollinated: set = set()
        # Re-pollination count (should always be 0)
        self.repollinations: int = 0
        # Count by sector
        self.sector_counts: dict = {}

        # Load existing log
        self._reload_log()

        self._print_startup()

    def _load_registry(self) -> list:
        if not REGISTRY_CSV.exists():
            return []
        with open(REGISTRY_CSV, newline='') as f:
            return list(csv.DictReader(f))

    def _reload_log(self):
        """Re-read pollination_log.csv and update all tracking structures."""
        if not LOG_CSV.exists():
            return

        new_count = 0
        with open(LOG_CSV, newline='') as f:
            for row in csv.DictReader(f):
                fid    = row.get('flower_id', '')
                run_id = row.get('run_id', '')
                status = row.get('status', '')
                sector = row.get('sector', '')

                if status != 'POLLINATED' or not fid:
                    continue

                # Check for re-pollination
                if fid in self.all_pollinated:
                    # Only count if it's in a DIFFERENT run to the first visit
                    first_run = next(
                        (r for r, ids in self.per_run.items() if fid in ids),
                        None
                    )
                    if first_run and first_run != run_id:
                        self.repollinations += 1
                        print(
                            f'  ⚠  RE-POLLINATION detected: {fid} '
                            f'first in {first_run}, again in {run_id}'
                        )
                    continue

                # New pollination — record it
                self.all_pollinated.add(fid)
                if run_id not in self.per_run:
                    self.per_run[run_id] = set()
                self.per_run[run_id].add(fid)

                # Track by sector
                fl   = self.flower_by_id.get(fid, {})
                sec  = fl.get('sector', sector)
                if sec:
                    self.sector_counts[sec] = self.sector_counts.get(sec, 0) + 1

                new_count += 1

        if new_count > 0:
            total  = self.total_flowers
            polled = len(self.all_pollinated)
            print(
                f'  Log reload: +{new_count} | '
                f'total {polled}/{total} ({polled/total*100:.1f}%)'
            )

    def add_live_event(self, flower_id: str, run_id: str,
                       drone_id: int, sector: str = ''):
        """Called by ROS 2 node when a live pollination event arrives."""
        if not flower_id:
            return

        if flower_id in self.all_pollinated:
            # Re-pollination check
            first_run = next(
                (r for r, ids in self.per_run.items() if flower_id in ids),
                None
            )
            if first_run and first_run != run_id:
                self.repollinations += 1
                print(f'  ⚠  RE-POLLINATION: {flower_id} '
                      f'(first={first_run}, now={run_id})')
            return

        self.all_pollinated.add(flower_id)
        if run_id not in self.per_run:
            self.per_run[run_id] = set()
        self.per_run[run_id].add(flower_id)

        fl  = self.flower_by_id.get(flower_id, {})
        sec = fl.get('sector', sector)
        if sec:
            self.sector_counts[sec] = self.sector_counts.get(sec, 0) + 1

        total   = self.total_flowers
        polled  = len(self.all_pollinated)
        print(
            f'  ✅ Drone {drone_id}: {flower_id} '
            f'(run={run_id}, sec={sec}) | '
            f'{polled}/{total} ({polled/total*100:.1f}%)'
        )

    # ── Status printing ───────────────────────────────────────────────────────

    def _print_startup(self):
        total  = self.total_flowers
        polled = len(self.all_pollinated)

        # Count by bloom day
        by_day = {}
        for fl in self.flowers:
            d = fl['bloom_day']
            by_day[d] = by_day.get(d, 0) + 1

        sep = '═' * 56
        print(f'\n{sep}')
        print('  KPI Logger — Stage 12 (Realistic 300-Flower Field)')
        print(f'  Output: {OUTPUT_DIR}')
        print(sep)
        print(f'  Total flowers: {total}')
        print(f'    Day 1: {by_day.get("1", 0)} | '
              f'Day 2: {by_day.get("2", 0)} | '
              f'Day 3: {by_day.get("3", 0)} | '
              f'Never: {by_day.get("0", 0)}')
        print(f'  Already pollinated from log: {polled}')
        print(f'  KPI targets: coverage ≥ {KPI_COVERAGE}% | '
              f're-pollination = 0')
        print(f'  Ctrl+C to stop and save report.')
        print(sep + '\n')

    def print_status(self):
        """Print KPI summary — called every 30 seconds by ROS 2 timer."""
        total    = self.total_flowers
        polled   = len(self.all_pollinated)
        coverage = polled / total * 100

        sep = '─' * 50
        print(f'\n{sep}')
        print(f'  KPI LIVE SUMMARY [{(time.time()-self.start_time)/60:.1f} min]')
        print(sep)

        # Per-run breakdown
        for run_id in sorted(self.per_run.keys()):
            count = len(self.per_run[run_id])
            print(f'  {run_id}: {count:3d} flowers')

        cumulative_pct = coverage
        print(f'  Cumulative: {polled}/{total} = {cumulative_pct:.1f}%  '
              f'(target ≥ {KPI_COVERAGE}%)')
        print(f'  Re-pollinations: {self.repollinations}  '
              f'(target = {KPI_REPOLLINATION})')
        print(sep)

    # ── Report generation ─────────────────────────────────────────────────────

    def save_report(self):
        """Save KPI report on shutdown."""
        self._reload_log()   # final reload

        total    = self.total_flowers
        polled   = len(self.all_pollinated)
        coverage = polled / total * 100

        # Eligible flowers (bloom_day != 0)
        eligible = sum(1 for fl in self.flowers if fl.get('bloom_day', '0') != '0')

        kpis = {
            'generated_at': datetime.now().isoformat(),
            'field': {
                'total_flowers':   total,
                'eligible_flowers': eligible,
                'never_open':      total - eligible,
                'pollinated':      polled,
                'coverage_pct':    round(coverage, 2),
            },
            'per_run': {
                run_id: {
                    'count': len(ids),
                    'pct_of_total': round(len(ids) / total * 100, 1)
                }
                for run_id, ids in sorted(self.per_run.items())
            },
            'per_sector': {
                sec: count
                for sec, count in sorted(self.sector_counts.items())
            },
            'kpis': {
                'detection_map50': {
                    'value':     ACHIEVED_DETECTION_MAP50,
                    'target':    KPI_DETECTION_MAP50,
                    'pass':      ACHIEVED_DETECTION_MAP50 >= KPI_DETECTION_MAP50,
                    'precision': ACHIEVED_DETECTION_PREC,
                    'recall':    ACHIEVED_DETECTION_RECALL,
                    'note':      'YOLOv8s-OBB (Stage 6, sunflower_best.pt)'
                },
                'readiness_map50': {
                    'value':     ACHIEVED_READINESS_MAP50,
                    'target':    KPI_READINESS_MAP50,
                    'pass':      ACHIEVED_READINESS_MAP50 >= KPI_READINESS_MAP50,
                    'precision': ACHIEVED_READINESS_PREC,
                    'recall':    ACHIEVED_READINESS_RECALL,
                    'note':      'YOLOv8s readiness classifier (Stage 10)'
                },
                'coverage_pct': {
                    'value':  round(coverage, 1),
                    'target': KPI_COVERAGE,
                    'pass':   coverage >= KPI_COVERAGE,
                },
                'repollinations': {
                    'value':  self.repollinations,
                    'target': KPI_REPOLLINATION,
                    'pass':   self.repollinations == 0,
                    'note':   'Verified by Persistent Pollination Manager'
                },
                'tsp_improvement_pct': {
                    'value':  ACHIEVED_TSP_IMPROVEMENT,
                    'target': KPI_TSP_IMPROVEMENT,
                    'pass':   ACHIEVED_TSP_IMPROVEMENT >= KPI_TSP_IMPROVEMENT,
                    'note':   'Nearest Neighbour + 2-opt (Stage 11)'
                },
                'bloom_filter_gain_pct': {
                    'value':  ACHIEVED_BLOOM_FILTER_GAIN,
                    'target': KPI_BLOOM_FILTER_GAIN,
                    'pass':   ACHIEVED_BLOOM_FILTER_GAIN >= KPI_BLOOM_FILTER_GAIN,
                    'note':   'Non-ready flowers skipped (Stage 10)'
                },
            }
        }

        # Save JSON
        json_path = OUTPUT_DIR / 'kpi_report.json'
        with open(json_path, 'w') as f:
            json.dump(kpis, f, indent=2)
        print(f'  JSON → {json_path}')

        # Save human-readable TXT
        self._save_txt(kpis)

    def _save_txt(self, kpis: dict):
        """Save formatted TXT report — paste directly into Chapter 5."""
        lines = []
        lines.append('=' * 62)
        lines.append('  STAGE 12 — KPI EVALUATION REPORT')
        lines.append('  FYP: Autonomous Swarm Drone Pollination')
        lines.append(f'  Generated: {kpis["generated_at"]}')
        lines.append('=' * 62)
        lines.append('')

        # Field stats
        f = kpis['field']
        lines.append('  FIELD STATISTICS')
        lines.append(f'  Total flowers   : {f["total_flowers"]}')
        lines.append(f'  Eligible flowers: {f["eligible_flowers"]} '
                     f'(bloom_day 1-3)')
        lines.append(f'  Never open      : {f["never_open"]} '
                     f'(damaged/failed, bloom_day=0)')
        lines.append(f'  Pollinated      : {f["pollinated"]}  '
                     f'({f["coverage_pct"]:.1f}%)')
        lines.append('')

        # Per-run
        lines.append('  PER-RUN BREAKDOWN')
        cumul = 0
        for run_id, data in sorted(kpis['per_run'].items()):
            cumul += data['count']
            pct    = cumul / kpis['field']['total_flowers'] * 100
            lines.append(
                f'  {run_id}: {data["count"]:3d} new flowers  '
                f'→ cumulative {cumul}/{kpis["field"]["total_flowers"]} '
                f'= {pct:.1f}%'
            )
        lines.append('')

        # Per-sector
        lines.append('  PER-SECTOR BREAKDOWN')
        for sec, count in sorted(kpis['per_sector'].items()):
            lines.append(f'    {sec}: {count} flowers pollinated')
        lines.append('')

        # KPI table
        lines.append(f'  {"KPI":<30} {"Value":<12} {"Target":<12} Pass?')
        lines.append('  ' + '-' * 58)

        kpi_rows = [
            ('Detection mAP@0.5',
             f'{ACHIEVED_DETECTION_MAP50:.3f}',
             f'≥ {KPI_DETECTION_MAP50}',
             ACHIEVED_DETECTION_MAP50 >= KPI_DETECTION_MAP50),
            ('Detection Precision',
             f'{ACHIEVED_DETECTION_PREC:.3f}', '—', True),
            ('Detection Recall',
             f'{ACHIEVED_DETECTION_RECALL:.3f}', '—', True),
            ('Readiness mAP@0.5',
             f'{ACHIEVED_READINESS_MAP50:.3f}',
             f'≥ {KPI_READINESS_MAP50}',
             ACHIEVED_READINESS_MAP50 >= KPI_READINESS_MAP50),
            ('Field coverage (%)',
             f'{kpis["field"]["coverage_pct"]:.1f}',
             f'≥ {KPI_COVERAGE}',
             kpis['field']['coverage_pct'] >= KPI_COVERAGE),
            ('Re-pollinations',
             str(self.repollinations),
             f'= {KPI_REPOLLINATION}',
             self.repollinations == 0),
            ('TSP improvement (%)',
             f'{ACHIEVED_TSP_IMPROVEMENT}',
             f'≥ {KPI_TSP_IMPROVEMENT}',
             ACHIEVED_TSP_IMPROVEMENT >= KPI_TSP_IMPROVEMENT),
            ('Bloom filter gain (%)',
             f'{ACHIEVED_BLOOM_FILTER_GAIN}',
             f'≥ {KPI_BLOOM_FILTER_GAIN}',
             ACHIEVED_BLOOM_FILTER_GAIN >= KPI_BLOOM_FILTER_GAIN),
        ]

        for name, val, target, passed in kpi_rows:
            tick = '✅ PASS' if passed else '❌ FAIL'
            lines.append(f'  {name:<30} {val:<12} {target:<12} {tick}')

        lines.append('=' * 62)

        text = '\n'.join(lines)
        txt_path = OUTPUT_DIR / 'kpi_report.txt'
        with open(txt_path, 'w') as f:
            f.write(text + '\n')
        print(f'  TXT  → {txt_path}')
        print('\n' + text + '\n')


# ── ROS 2 node wrapper ────────────────────────────────────────────────────────

def run_with_ros():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class KPILoggerNode(Node):

        def __init__(self):
            super().__init__('kpi_logger')
            self.logger = KPILogger()

            # Subscribe to all 3 drones' pollination logs
            for drone_id in range(3):
                self.create_subscription(
                    String,
                    f'/drone_{drone_id}/pollination/log',
                    lambda msg, d=drone_id: self._on_pollination(msg, d),
                    10
                )

            # Re-read CSV every 30 seconds
            self.create_timer(30.0, self.logger._reload_log)

            # Status print every 30 seconds
            self.create_timer(30.0, self.logger.print_status)

            self.get_logger().info('KPI Logger ready.')

        def _on_pollination(self, msg, drone_id: int):
            try:
                data      = json.loads(msg.data)
                flower_id = data.get('flower_id', data.get('id', ''))
                run_id    = data.get('run_id', 'LIVE')
                sector    = data.get('sector', '')
                self.logger.add_live_event(flower_id, run_id, drone_id, sector)
            except (json.JSONDecodeError, KeyError):
                pass

    rclpy.init()
    node = KPILoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.save_report()
        node.destroy_node()
        rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='KPI Logger — Stage 12')
    parser.add_argument(
        '--offline',
        action='store_true',
        help='Read existing CSVs only (no ROS 2), save report, then exit.'
    )
    args = parser.parse_args()

    if args.offline:
        print('\n[OFFLINE MODE] Reading CSVs and generating KPI report...\n')
        logger = KPILogger()
        logger.print_status()
        logger.save_report()
    else:
        run_with_ros()
