#!/usr/bin/env python3
"""
Field Health Mapper — Stage 12 (Realistic 300-Flower Field)
=============================================================
Builds a top-down spatial health map of the full 300-flower field.

SOURCE OF TRUTH:
    ~/Desktop/FYP/setup/flower_registry.csv
        — all 300 flower positions (x, y, sector, bloom_day)
        — written once by stage10b, never changes

    ~/Desktop/FYP/setup/pollination_log.csv
        — appended by persistent_pollination_manager during missions
        — tells us which flowers have been pollinated and in which run

NOTE ON DETECTION APPROACH:
    The realistic scenario uses persistent_pollination_manager with
    proximity-based detection — there are no raw YOLOv8 camera frames.
    So the health map uses bloom readiness status as the viability proxy
    rather than raw YOLOv8 confidence scores:

    ■ Dark grey    — bloom_day = 0 (never opens, damaged/failed)
    ■ Amber/orange — bloom_ready but not yet pollinated
    ■ Light yellow — future bloom (not yet reached bloom day)
    ■ Green        — pollinated (darkens with each run)
    ■ Light grey   — no flower in this cell

ALSO subscribes to /drone_N/pollination/log for live updates
while the mission is running, so the CSV doesn't need to be
manually refreshed.

FIELD GEOMETRY (from stage10b):
    Width:  60m  (x: 0 to 60)
    Depth:  20m  (y: 0 to 20)
    Sector LEFT   (Drone 0): x =  0–20m
    Sector CENTRE (Drone 1): x = 20–40m
    Sector RIGHT  (Drone 2): x = 40–60m

Grid: 30 cols × 10 rows = 2m × 2m per cell

OUTPUTS (saved on Ctrl+C):
    health_map.png            — full-field final heatmap
    health_map_RUN_1.png      — coverage after Run 1 only
    health_map_RUN_2.png      — cumulative after Run 2
    health_map_RUN_3.png      — cumulative after Run 3
    health_map.csv            — per-flower status table
    summary.txt               — field statistics (paste to report)

HOW TO RUN:
    # After starting launch_stage11.sh:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/FYP/ros2_ws/install/setup.bash
    ros2 run precision_pollination field_health_mapper

    # Offline mode — no ROS 2, reads CSVs and saves outputs, then exits:
    python3 field_health_mapper.py --offline
"""

import os
import sys
import csv
import json
import time
import argparse
from datetime import datetime
from pathlib import Path

# ── Field geometry (must match stage10b values exactly) ───────────────────────
FIELD_X_MAX  = 60.0
FIELD_Y_MAX  = 20.0
GRID_COLS    = 30     # 2m wide cells
GRID_ROWS    = 10     # 2m tall cells
CELL_W       = FIELD_X_MAX / GRID_COLS   # 2.0 m
CELL_H       = FIELD_Y_MAX / GRID_ROWS   # 2.0 m

# Sector boundaries in metres (from stage10b)
SECTOR_X     = [20.0, 40.0]

# File paths
SETUP_DIR    = Path.home() / 'Desktop/FYP/setup'
REGISTRY_CSV = SETUP_DIR / 'flower_registry.csv'
LOG_CSV      = SETUP_DIR / 'pollination_log.csv'
OUTPUT_DIR   = Path(os.environ.get(
    'STAGE12_OUTPUT_DIR',
    str(Path.home() / 'Desktop/FYP/mission_outputs/latest')
))


class FieldHealthMapper:
    """
    Core logic — shared between ROS 2 mode and offline mode.
    All state lives here; the ROS 2 node wrapper below calls into this.
    """

    def __init__(self):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        # Load registry (300 flowers, fixed positions)
        self.flowers = self._load_registry()
        if not self.flowers:
            raise FileNotFoundError(
                f'flower_registry.csv not found at {REGISTRY_CSV}\n'
                f'Run stage10b_persistent.sh first.'
            )
        print(f'  Loaded {len(self.flowers)} flowers from registry.')

        # flower_id → run_id (e.g. 'RUN_1')  — filled from log + live events
        self.pollinated: dict = {}

        # Load existing log
        self._reload_log()

    def _load_registry(self) -> list:
        if not REGISTRY_CSV.exists():
            return []
        with open(REGISTRY_CSV, newline='') as f:
            return list(csv.DictReader(f))

    def _reload_log(self):
        """Re-read pollination_log.csv and update self.pollinated."""
        if not LOG_CSV.exists():
            return
        new = 0
        with open(LOG_CSV, newline='') as f:
            for row in csv.DictReader(f):
                fid    = row.get('flower_id', '')
                run_id = row.get('run_id', '')
                status = row.get('status', '')
                if status == 'POLLINATED' and fid and str(fid) not in self.pollinated:
                    self.pollinated[str(fid)] = run_id
                    new += 1
        if new:
            total    = len(self.flowers)
            polled   = len(self.pollinated)
            print(f'  Log reload: +{new} new | total {polled}/{total} '
                  f'({polled/total*100:.1f}%)')

    def add_live_event(self, flower_id: str, run_id: str, drone_id: int):
        """Called by the ROS 2 node when a live pollination arrives."""
        if flower_id and str(flower_id) not in self.pollinated:
            self.pollinated[str(flower_id)] = run_id
            total   = len(self.flowers)
            polled  = len(self.pollinated)
            print(f'  [LIVE] Drone {drone_id}: pollinated {flower_id} '
                  f'(run={run_id}) | {polled}/{total} '
                  f'({polled/total*100:.1f}%)')

    def get_cell(self, x: float, y: float) -> tuple:
        """Convert world (x, y) in metres to (row, col) grid indices."""
        col = int(x / CELL_W)
        row = int(y / CELL_H)
        col = max(0, min(GRID_COLS - 1, col))
        row = max(0, min(GRID_ROWS - 1, row))
        return row, col

    def print_status(self):
        """Print live coverage summary."""
        total   = len(self.flowers)
        polled  = len(self.pollinated)
        run_counts = {}
        for rid in self.pollinated.values():
            run_counts[rid] = run_counts.get(rid, 0) + 1
        runs_str = ' | '.join(f'{r}: {c}' for r, c in sorted(run_counts.items()))
        print(f'  Coverage: {polled}/{total} ({polled/total*100:.1f}%)  {runs_str}')

    def print_startup(self):
        """Print field overview banner."""
        total  = len(self.flowers)
        by_day = {0: 0, 1: 0, 2: 0, 3: 0}
        by_sec = {}
        for fl in self.flowers:
            by_day[int(fl['bloom_day'])] += 1
            s = fl['sector']
            by_sec[s] = by_sec.get(s, 0) + 1

        sep = '═' * 56
        print(f'\n{sep}')
        print(f'  Field Health Mapper — Stage 12 (Realistic Field)')
        print(f'  Output dir: {OUTPUT_DIR}')
        print(sep)
        print(f'  Field:  {FIELD_X_MAX:.0f}m × {FIELD_Y_MAX:.0f}m')
        print(f'  Flowers: {total} total  | '
              f'Day1: {by_day[1]}  Day2: {by_day[2]}  '
              f'Day3: {by_day[3]}  Never: {by_day[0]}')
        for sec, count in sorted(by_sec.items()):
            print(f'  {sec}: {count} flowers')
        print(f'  Grid:  {GRID_COLS}×{GRID_ROWS} cells ({CELL_W:.1f}m × {CELL_H:.1f}m each)')
        print(f'  Already pollinated (from log): {len(self.pollinated)}')
        print(f'  Ctrl+C to stop and save outputs.')
        print(sep + '\n')

    # ── Output generation ─────────────────────────────────────────────────────

    def save_all_outputs(self):
        """Save CSV, summary, and PNGs for each run + final combined."""
        print('\n  Saving outputs...')
        self._reload_log()   # final reload

        all_runs = sorted(set(self.pollinated.values()))

        self._save_csv()
        self._save_summary(all_runs)

        # Per-run cumulative PNGs
        for run_id in all_runs:
            # All flowers pollinated UP TO AND INCLUDING this run
            up_to = {fid for fid, rid in self.pollinated.items()
                     if rid <= run_id}
            self._save_png(
                pollinated_set=up_to,
                filename=f'health_map_{run_id}.png',
                title=f'Coverage after {run_id}'
            )

        # Final combined PNG
        self._save_png(
            pollinated_set=set(self.pollinated.keys()),
            filename='health_map.png',
            title='Final coverage — all runs'
        )
        print(f'\n  Open map: eog {OUTPUT_DIR}/health_map.png\n')

    def _save_csv(self):
        csv_path = OUTPUT_DIR / 'health_map.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'flower_id', 'sector', 'drone_id',
                'x', 'y', 'bloom_day',
                'pollinated', 'run_id',
                'grid_row', 'grid_col'
            ])
            for fl in self.flowers:
                fid      = fl['flower_id']
                row, col = self.get_cell(float(fl['x']), float(fl['y']))
                writer.writerow([
                    fid,
                    fl['sector'],
                    fl['drone_id'],
                    fl['x'],
                    fl['y'],
                    fl['bloom_day'],
                    'YES' if fid in self.pollinated else 'NO',
                    self.pollinated.get(fid, ''),
                    row, col
                ])
        print(f'  CSV  → {csv_path}')

    def _save_summary(self, all_runs: list):
        total  = len(self.flowers)
        polled = len(self.pollinated)

        run_counts = {}
        for rid in self.pollinated.values():
            run_counts[rid] = run_counts.get(rid, 0) + 1

        sector_total = {}
        sector_polled = {}
        for fl in self.flowers:
            s = fl['sector']
            sector_total[s]  = sector_total.get(s, 0) + 1
            if fl['flower_id'] in self.pollinated:
                sector_polled[s] = sector_polled.get(s, 0) + 1

        txt_path = OUTPUT_DIR / 'summary.txt'
        lines = []
        lines.append('=' * 58)
        lines.append('  STAGE 12 — FIELD HEALTH MAP SUMMARY')
        lines.append(f'  Generated: {datetime.now().isoformat()}')
        lines.append('=' * 58)
        lines.append(f'  Total flowers : {total}')
        lines.append(f'  Pollinated    : {polled}  ({polled/total*100:.1f}%)')
        lines.append('')
        lines.append('  Per-run breakdown:')
        cumulative = 0
        for rid in sorted(run_counts.keys()):
            c = run_counts[rid]
            cumulative += c
            lines.append(
                f'    {rid}: {c:3d} new  →  '
                f'cumulative {cumulative}/{total} = '
                f'{cumulative/total*100:.1f}%'
            )
        lines.append('')
        lines.append('  Per-sector breakdown:')
        for sec in sorted(sector_total.keys()):
            t = sector_total[sec]
            p = sector_polled.get(sec, 0)
            lines.append(f'    {sec:8s}: {p:3d}/{t} ({p/t*100:.1f}%)')
        lines.append('=' * 58)

        text = '\n'.join(lines)
        with open(txt_path, 'w') as f:
            f.write(text + '\n')
        print(f'  TXT  → {txt_path}')
        print('\n' + text)

    def _save_png(self, pollinated_set: set, filename: str, title: str):
        """Generate heatmap PNG for a given set of pollinated flowers."""
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
            import numpy as np

            # Build RGBA image array
            img = np.ones((GRID_ROWS, GRID_COLS, 4), dtype=float)

            # Count per cell
            cell_counts = {}   # (row, col) → {pollinated, bloom_ready, future, never}
            for fl in self.flowers:
                r, c  = self.get_cell(float(fl['x']), float(fl['y']))
                bd    = int(fl['bloom_day'])
                fid   = f"F{int(fl['flower_id']):03d}"
                key   = (r, c)
                if key not in cell_counts:
                    cell_counts[key] = {'p': 0, 'b': 0, 'f': 0, 'n': 0}
                if fid in pollinated_set:
                    cell_counts[key]['p'] += 1
                elif bd == 0:
                    cell_counts[key]['n'] += 1
                else:
                    cell_counts[key]['b'] += 1

            # Colour each cell — flip rows so y=0 is at the bottom of image
            for (row, col), counts in cell_counts.items():
                canvas_row = GRID_ROWS - 1 - row
                total_in_cell = sum(counts.values())
                if total_in_cell == 0:
                    continue

                p_frac = counts['p'] / total_in_cell
                n_frac = counts['n'] / total_in_cell

                if p_frac > 0:
                    # Green, darkens with higher pollination fraction
                    g = 0.35 + 0.50 * p_frac
                    r_c = 0.05 + 0.20 * (1 - p_frac)
                    img[canvas_row, col] = (r_c, g, 0.10, 1.0)
                elif n_frac > 0.5:
                    img[canvas_row, col] = (0.5, 0.5, 0.5, 1.0)   # grey - never
                else:
                    img[canvas_row, col] = (0.95, 0.65, 0.10, 1.0) # amber - bloom ready

            # Cells with no flowers remain white
            all_with_flowers = set(cell_counts.keys())
            for row in range(GRID_ROWS):
                for col in range(GRID_COLS):
                    if (row, col) not in all_with_flowers:
                        canvas_row = GRID_ROWS - 1 - row
                        img[canvas_row, col] = (0.94, 0.94, 0.94, 1.0)  # light grey

            # Canvas
            fig, ax = plt.subplots(figsize=(15, 6))
            ax.imshow(
                img,
                extent=[0, FIELD_X_MAX, 0, FIELD_Y_MAX],
                origin='lower',
                aspect='auto',
                interpolation='nearest'
            )

            # Draw individual flower dots (small — 300 flowers)
            for fl in self.flowers:
                x   = float(fl['x'])
                y   = float(fl['y'])
                fid = f"F{int(fl['flower_id']):03d}"
                bd  = int(fl['bloom_day'])
                if fid in pollinated_set:
                    colour, size = '#00ff44', 18
                elif bd == 0:
                    colour, size = '#666666', 6
                else:
                    colour, size = 'white', 10
                ax.scatter(x, y, c=colour, s=size, zorder=3,
                           linewidths=0, alpha=0.85)

            # Sector lines
            for xb in SECTOR_X:
                ax.axvline(x=xb, color='cyan', linestyle='--',
                           linewidth=1.5, alpha=0.8, zorder=4)

            # Labels
            ax.text(10, FIELD_Y_MAX * 0.95, 'Drone 0  LEFT',
                    color='white', fontsize=9, ha='center',
                    fontweight='bold', zorder=5)
            ax.text(30, FIELD_Y_MAX * 0.95, 'Drone 1  CENTRE',
                    color='white', fontsize=9, ha='center',
                    fontweight='bold', zorder=5)
            ax.text(50, FIELD_Y_MAX * 0.95, 'Drone 2  RIGHT',
                    color='white', fontsize=9, ha='center',
                    fontweight='bold', zorder=5)

            ax.set_xlabel('Field X position (m)', fontsize=12)
            ax.set_ylabel('Field Y position (m)', fontsize=12)
            total  = len(self.flowers)
            polled = len(pollinated_set)
            ax.set_title(
                f'{title}  |  {polled}/{total} flowers ({polled/total*100:.1f}%)',
                fontsize=12, fontweight='bold'
            )

            legend = [
                mpatches.Patch(facecolor=(0.10, 0.85, 0.10), label='Pollinated'),
                mpatches.Patch(facecolor=(0.95, 0.65, 0.10), label='Bloom-ready (pending)'),
                mpatches.Patch(facecolor=(0.50, 0.50, 0.50), label='Never opens (damaged)'),
                mpatches.Patch(facecolor=(0.94, 0.94, 0.94), label='No flower in cell'),
            ]
            ax.legend(handles=legend, loc='lower right',
                      fontsize=9, framealpha=0.90)

            plt.tight_layout()
            out_path = OUTPUT_DIR / filename
            plt.savefig(out_path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f'  PNG  → {out_path}')

        except ImportError as e:
            print(f'  ERROR: matplotlib not installed: {e}')
            print('  Fix:  pip3 install --break-system-packages matplotlib')
        except Exception as e:
            print(f'  ERROR generating PNG: {e}')
            import traceback
            traceback.print_exc()


# ── ROS 2 node wrapper ────────────────────────────────────────────────────────

def run_with_ros():
    """Start the health mapper as a ROS 2 node."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class HealthMapperNode(Node):

        def __init__(self):
            super().__init__('field_health_mapper')
            self.mapper = FieldHealthMapper()
            self.mapper.print_startup()

            # Subscribe to live pollination events from all 3 drones
            for drone_id in range(3):
                self.create_subscription(
                    String,
                    f'/drone_{drone_id}/pollination/log',
                    lambda msg, d=drone_id: self._on_pollination(msg, d),
                    10
                )

            # Re-read CSV every 15 seconds (catches events from other nodes)
            self.create_timer(15.0, self.mapper._reload_log)

            # Print status every 60 seconds
            self.create_timer(60.0, self.mapper.print_status)

            self.get_logger().info('Field health mapper ready.')

        def _on_pollination(self, msg, drone_id: int):
            try:
                data      = json.loads(msg.data)
                flower_id = data.get('flower_id', data.get('id', ''))
                run_id    = data.get('run_id', 'LIVE')
                self.mapper.add_live_event(flower_id, run_id, drone_id)
            except (json.JSONDecodeError, KeyError):
                pass

    rclpy.init()
    node = HealthMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mapper.save_all_outputs()
        node.destroy_node()
        rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    run_with_ros()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Field Health Mapper — Stage 12')
    parser.add_argument(
        '--offline',
        action='store_true',
        help='Read CSVs only (no ROS 2). Saves outputs and exits.'
    )
    args = parser.parse_args()

    if args.offline:
        print('\n[OFFLINE MODE] Reading CSVs and generating outputs...\n')
        mapper = FieldHealthMapper()
        mapper.print_startup()
        mapper.save_all_outputs()
    else:
        run_with_ros()
