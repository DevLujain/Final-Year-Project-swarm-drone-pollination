#!/bin/bash
# ============================================================
# STAGE 11 — TSP Path Optimisation
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS ADDS:
#   Currently drones visit bloom-ready flowers in registry
#   order — which is random and not the shortest route.
#   Stage 11 computes the optimal visitation order for each
#   drone's targets before the mission starts, using:
#     1. Nearest Neighbour heuristic  (fast initial solution)
#     2. 2-opt improvement            (iterative refinement)
#
#   This is the same approach used by Li et al. (2025) for
#   durian orchard drone pollination, who reported a 26.89%
#   reduction in total path length vs. baseline ordering.
#
# WHAT GETS MEASURED (for FYP report):
#   Per drone, per run:
#     - Baseline path length  (registry/random order)
#     - TSP-optimised length  (nearest neighbour + 2-opt)
#     - % improvement
#   Saved to: ~/Desktop/FYP/setup/tsp_log.csv
#
# PIPELINE AFTER STAGE 11:
#   persistent_pollination_manager
#       -> tsp_planner (NEW) computes optimal order
#       -> persistent_pollination_manager executes TSP order
#
# PRE-REQUISITES:
#   Stage 10b complete, flower_registry.csv exists
#
# RUN: bash stage11_tsp.sh
# TIME: ~2 min (no training required)
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination
SETUP_DIR=$FYP_DIR/setup

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 11 — TSP Path Optimisation                   ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash 2>/dev/null || true

# ── 1. Verify registry ────────────────────────────────────────
echo "[1/4] Verifying flower registry..."
if [ ! -f "$SETUP_DIR/flower_registry.csv" ]; then
    echo "ERROR: flower_registry.csv not found. Run stage10b first."
    exit 1
fi
FLOWER_COUNT=$(tail -n +2 $SETUP_DIR/flower_registry.csv | wc -l)
echo "  Found $FLOWER_COUNT flowers in registry"
echo "✓ Registry verified"

# ── 2. Create tsp_planner node ────────────────────────────────
echo ""
echo "[2/4] Creating tsp_planner node..."

cat > $NODES_DIR/tsp_planner.py << 'PYEOF'
#!/usr/bin/env python3
"""
TSP Planner Node — Stage 11
=============================
Computes the shortest visitation route for each drone's
bloom-ready targets before the mission begins.

ALGORITHM:
  Step 1 — Nearest Neighbour heuristic
    Start at drone home position.
    Repeatedly visit the closest unvisited flower.
    Fast, O(n^2). Gives a reasonable initial solution.

  Step 2 — 2-opt improvement
    Try reversing every sub-segment of the route.
    If reversing [i..j] makes the route shorter, keep it.
    Repeat until no improvement is found.
    Li et al. (2025) used a similar approach for durian
    orchard drone routing, reporting 26.89% path reduction.

ACADEMIC NOTE:
  Exact TSP is NP-hard, so heuristics are standard practice
  for drone routing in agriculture (Li et al., 2025;
  Yamamoto et al., 2025). Nearest neighbour + 2-opt gives
  near-optimal results for ~100 targets per drone.

TSP LOG COLUMNS:
  run_id, drone_id, n_targets,
  baseline_distance_m, nn_distance_m,
  tsp_distance_m, improvement_pct,
  computation_time_s, timestamp
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

SETUP_DIR   = Path.home() / "Desktop/FYP/setup"
TSP_LOG_CSV = SETUP_DIR / "tsp_log.csv"

DRONE_HOMES = {
    0: (0.0,  0.0),
    1: (20.0, 0.0),
    2: (40.0, 0.0),
}

SEP = "═" * 54


def euclidean(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def total_route_distance(route, home):
    if not route:
        return 0.0
    pts = [home] + [(float(f['x']), float(f['y'])) for f in route] + [home]
    return sum(euclidean(pts[i], pts[i+1]) for i in range(len(pts) - 1))


def nearest_neighbour(flowers, home):
    unvisited = list(flowers)
    route     = []
    current   = home
    while unvisited:
        nearest = min(
            unvisited,
            key=lambda f: euclidean(current, (float(f['x']), float(f['y'])))
        )
        route.append(nearest)
        current = (float(nearest['x']), float(nearest['y']))
        unvisited.remove(nearest)
    return route


def two_opt(route, home, max_iterations=500):
    if len(route) < 4:
        return route
    best      = list(route)
    best_dist = total_route_distance(best, home)
    improved  = True
    iters     = 0
    while improved and iters < max_iterations:
        improved = False
        iters   += 1
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                new_route = best[:i] + best[i:j+1][::-1] + best[j+1:]
                new_dist  = total_route_distance(new_route, home)
                if new_dist < best_dist - 1e-6:
                    best      = new_route
                    best_dist = new_dist
                    improved  = True
    return best


class TSPPlanner(Node):

    def __init__(self):
        super().__init__('tsp_planner')
        self._init_log()
        self.current_run_id = None

        self.route_pubs = {
            i: self.create_publisher(String, f'/drone_{i}/tsp_route', 10)
            for i in range(3)
        }
        self.summary_pub = self.create_publisher(
            String, '/swarm/tsp_summary', 10)

        self.create_subscription(
            String, '/mission/run_status', self._on_run_status, 10)

        for drone_id in range(3):
            self.create_subscription(
                String,
                f'/drone_{drone_id}/persistent_targets',
                lambda msg, d=drone_id: self._on_targets(msg, d),
                10
            )

        self.get_logger().info('TSP planner ready — waiting for targets.')

    def _init_log(self):
        if not TSP_LOG_CSV.exists():
            with open(TSP_LOG_CSV, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'run_id', 'drone_id', 'n_targets',
                    'baseline_distance_m', 'nn_distance_m',
                    'tsp_distance_m', 'improvement_pct',
                    'computation_time_s', 'timestamp'
                ])

    def _on_run_status(self, msg):
        try:
            self.current_run_id = json.loads(msg.data).get('run_id')
        except Exception:
            pass

    def _on_targets(self, msg, drone_id):
        try:
            data    = json.loads(msg.data)
            targets = data.get('targets', [])
            run_id  = data.get('run_id', self.current_run_id or 'UNKNOWN')
            home    = DRONE_HOMES[drone_id]
            sector  = ['LEFT', 'CENTER', 'RIGHT'][drone_id]

            if not targets:
                self.get_logger().info(
                    f'Drone {drone_id}: no targets this run.')
                return

            t0 = time.time()

            baseline_dist = total_route_distance(targets, home)
            nn_route      = nearest_neighbour(targets, home)
            nn_dist       = total_route_distance(nn_route, home)
            tsp_route     = two_opt(nn_route, home)
            tsp_dist      = total_route_distance(tsp_route, home)

            elapsed     = time.time() - t0
            improvement = (baseline_dist - tsp_dist) / baseline_dist * 100 \
                          if baseline_dist > 0 else 0.0

            # Print banner
            print(f'\n╔{SEP}╗')
            print(f'║  TSP — Drone {drone_id} ({sector}) | {run_id}' +
                  ' ' * max(0, 54 - 20 - len(sector) - len(run_id)) + '║')
            print(f'╠{SEP}╣')
            print(f'║  Targets          : {len(targets)}' +
                  ' ' * max(0, 54 - 22 - len(str(len(targets)))) + '║')
            print(f'║  Baseline distance: {baseline_dist:.1f}m' +
                  ' ' * max(0, 54 - 24 - len(f"{baseline_dist:.1f}")) + '║')
            print(f'║  After NN         : {nn_dist:.1f}m' +
                  ' ' * max(0, 54 - 22 - len(f"{nn_dist:.1f}")) + '║')
            print(f'║  After 2-opt      : {tsp_dist:.1f}m' +
                  ' ' * max(0, 54 - 22 - len(f"{tsp_dist:.1f}")) + '║')
            print(f'║  Improvement      : {improvement:.1f}%' +
                  ' ' * max(0, 54 - 22 - len(f"{improvement:.1f}")) + '║')
            print(f'║  Compute time     : {elapsed:.3f}s' +
                  ' ' * max(0, 54 - 23 - len(f"{elapsed:.3f}")) + '║')
            print(f'╚{SEP}╝')

            # Publish optimised route
            route_msg = String()
            route_msg.data = json.dumps({
                'run_id':              run_id,
                'drone_id':            drone_id,
                'targets':             tsp_route,
                'tsp_distance_m':      round(tsp_dist, 2),
                'baseline_distance_m': round(baseline_dist, 2),
                'improvement_pct':     round(improvement, 2)
            })
            self.route_pubs[drone_id].publish(route_msg)

            # Log to CSV
            with open(TSP_LOG_CSV, 'a', newline='') as f:
                csv.writer(f).writerow([
                    run_id, drone_id, len(targets),
                    round(baseline_dist, 2), round(nn_dist, 2),
                    round(tsp_dist, 2), round(improvement, 2),
                    round(elapsed, 4),
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                ])

            # Publish summary
            summary_msg = String()
            summary_msg.data = json.dumps({
                'run_id':          run_id,
                'drone_id':        drone_id,
                'sector':          sector,
                'n_targets':       len(targets),
                'baseline_m':      round(baseline_dist, 2),
                'nn_m':            round(nn_dist, 2),
                'tsp_m':           round(tsp_dist, 2),
                'improvement_pct': round(improvement, 2),
                'compute_s':       round(elapsed, 4),
                'timestamp':       datetime.now().isoformat()
            })
            self.summary_pub.publish(summary_msg)

        except Exception as e:
            self.get_logger().error(f'TSP error drone {drone_id}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TSPPlanner()
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

echo "✓ tsp_planner.py written"

# ── 3. Update setup.py and rebuild ───────────────────────────
echo ""
echo "[3/4] Registering node and rebuilding workspace..."

cat > $ROS2_WS/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.11.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system — Stage 11',
    entry_points={
        'console_scripts': [
            'pollination_controller        = precision_pollination.pollination_controller:main',
            'yolov8_detector               = precision_pollination.yolov8_detector:main',
            'swarm_coordinator             = precision_pollination.swarm_coordinator:main',
            'camera_bridge                 = precision_pollination.camera_bridge:main',
            'position_estimator            = precision_pollination.position_estimator:main',
            'mission_logger                = precision_pollination.mission_logger:main',
            'flower_detector_sim           = precision_pollination.flower_detector_sim:main',
            'lawnmower_sweep               = precision_pollination.lawnmower_sweep:main',
            'shared_visited_list           = precision_pollination.shared_visited_list:main',
            'flower_readiness_filter       = precision_pollination.flower_readiness_filter:main',
            'readiness_logger              = precision_pollination.readiness_logger:main',
            'persistent_pollination_manager = precision_pollination.persistent_pollination_manager:main',
            'tsp_planner                   = precision_pollination.tsp_planner:main',
        ],
    },
)
SETUPEOF

source /opt/ros/jazzy/setup.bash
cd $ROS2_WS
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null || true
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -10

mkdir -p $ROS2_WS/install/precision_pollination/lib/precision_pollination

ALL_NODES=(
    pollination_controller yolov8_detector swarm_coordinator
    camera_bridge position_estimator mission_logger
    flower_detector_sim lawnmower_sweep shared_visited_list
    flower_readiness_filter readiness_logger
    persistent_pollination_manager tsp_planner
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

echo "✓ All ${#ALL_NODES[@]} nodes registered"

# ── 4. Benchmark + launch script ─────────────────────────────
echo ""
echo "[4/4] Running benchmark and creating launch script..."

python3 << 'PYEOF'
import csv, math, time
from pathlib import Path

REGISTRY_CSV = Path.home() / "Desktop/FYP/setup/flower_registry.csv"
DRONE_HOMES  = {0: (0.0, 0.0), 1: (20.0, 0.0), 2: (40.0, 0.0)}
SEP = "═" * 54

def euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def total_dist(route, home):
    if not route: return 0.0
    pts = [home] + [(float(f['x']), float(f['y'])) for f in route] + [home]
    return sum(euclidean(pts[i], pts[i+1]) for i in range(len(pts)-1))

def nearest_neighbour(flowers, home):
    unvisited = list(flowers)
    route, current = [], home
    while unvisited:
        nearest = min(unvisited,
            key=lambda f: euclidean(current, (float(f['x']), float(f['y']))))
        route.append(nearest)
        current = (float(nearest['x']), float(nearest['y']))
        unvisited.remove(nearest)
    return route

def two_opt(route, home, max_iter=500):
    if len(route) < 4: return route
    best = list(route)
    best_d = total_dist(best, home)
    improved, iters = True, 0
    while improved and iters < max_iter:
        improved = False
        iters += 1
        for i in range(1, len(best)-1):
            for j in range(i+1, len(best)):
                new = best[:i] + best[i:j+1][::-1] + best[j+1:]
                nd = total_dist(new, home)
                if nd < best_d - 1e-6:
                    best, best_d, improved = new, nd, True
    return best

if not REGISTRY_CSV.exists():
    print("  Registry not found — skipping benchmark")
    exit(0)

with open(REGISTRY_CSV, newline='') as f:
    registry = list(csv.DictReader(f))

print(f"\n{'='*56}")
print(f"  TSP BENCHMARK — Stage 11 (Day 1 bloom_ready sample)")
print(f"{'='*56}")

total_base, total_tsp = 0, 0

for drone_id in range(3):
    sector  = ['LEFT','CENTER','RIGHT'][drone_id]
    home    = DRONE_HOMES[drone_id]
    targets = [f for f in registry
               if int(f['drone_id']) == drone_id
               and int(f['bloom_day']) == 1]
    if not targets:
        continue

    t0          = time.time()
    base_d      = total_dist(targets, home)
    nn_route    = nearest_neighbour(targets, home)
    nn_d        = total_dist(nn_route, home)
    tsp_route   = two_opt(nn_route, home)
    tsp_d       = total_dist(tsp_route, home)
    elapsed     = time.time() - t0
    improvement = (base_d - tsp_d) / base_d * 100

    total_base += base_d
    total_tsp  += tsp_d

    print(f"\n╔{SEP}╗")
    print(f"║  Drone {drone_id} ({sector}){' '*(54-13-len(sector))}║")
    print(f"╠{SEP}╣")
    print(f"║  Targets          : {len(targets)}{' '*(54-22-len(str(len(targets))))  }║")
    print(f"║  Baseline distance: {base_d:.1f}m{' '*(54-24-len(f'{base_d:.1f}'))}║")
    print(f"║  After NN         : {nn_d:.1f}m{' '*(54-22-len(f'{nn_d:.1f}'))}║")
    print(f"║  After 2-opt      : {tsp_d:.1f}m{' '*(54-22-len(f'{tsp_d:.1f}'))}║")
    print(f"║  Improvement      : {improvement:.1f}%{' '*(54-22-len(f'{improvement:.1f}'))}║")
    print(f"║  Compute time     : {elapsed:.3f}s{' '*(54-23-len(f'{elapsed:.3f}'))}║")
    print(f"╚{SEP}╝")

if total_base > 0:
    overall = (total_base - total_tsp) / total_base * 100
    print(f"\n{'='*56}")
    print(f"  SWARM TOTAL — Day 1 sample")
    print(f"  Baseline : {total_base:.1f}m")
    print(f"  TSP      : {total_tsp:.1f}m")
    print(f"  Overall improvement : {overall:.1f}%")
    print(f"  (Li et al. 2025 benchmark: 26.89% for durian orchards)")
    print(f"{'='*56}\n")
PYEOF

cat > $FYP_DIR/launch_stage11.sh << 'LAUNCHEOF'
#!/bin/bash
# ============================================================
# LAUNCH STAGE 11 — TSP-Optimised Swarm Pollination
# RUN: bash ~/Desktop/FYP/launch_stage11.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 11 — TSP-Optimised Pollination               ║"
echo "║  Nearest Neighbour + 2-opt per drone per run        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  TSP log: ~/Desktop/FYP/setup/tsp_log.csv"
echo "  Monitor: ros2 topic echo /swarm/tsp_summary"
echo ""

# Reset pollination log for clean 3-run demo
if [ -f "$FYP_DIR/setup/pollination_log.csv" ]; then
    echo "  Backing up existing pollination log..."
    cp $FYP_DIR/setup/pollination_log.csv \
       $FYP_DIR/setup/pollination_log_backup_$(date +%Y%m%d_%H%M%S).csv
    rm $FYP_DIR/setup/pollination_log.csv
    echo "  Log reset for clean run."
fi

sleep 2

# Launch TSP planner
gnome-terminal --title="TSP Planner [S11]" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
    echo 'TSP Planner ready — waiting for targets...'
    ros2 run precision_pollination tsp_planner
    exec bash" &
sleep 3

# Launch mission manager
ros2 run precision_pollination persistent_pollination_manager
LAUNCHEOF

chmod +x $FYP_DIR/launch_stage11.sh
echo "✓ launch_stage11.sh written"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 11 COMPLETE                                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "ALGORITHM:"
echo "  Step 1 — Nearest Neighbour"
echo "    Start at drone home, always go to closest flower"
echo "    O(n^2) — fast for ~100 targets per drone"
echo ""
echo "  Step 2 — 2-opt improvement"
echo "    Try reversing every sub-route segment"
echo "    Keep if shorter, repeat until no gain"
echo ""
echo "NEW FILES:"
echo "  tsp_planner.py          — ROS 2 planner node"
echo "  setup/tsp_log.csv       — path length comparison log"
echo "  launch_stage11.sh       — full launcher"
echo ""
echo "TSP LOG COLUMNS:"
echo "  run_id, drone_id, n_targets,"
echo "  baseline_distance_m, nn_distance_m,"
echo "  tsp_distance_m, improvement_pct,"
echo "  computation_time_s, timestamp"
echo ""
echo "SCREENSHOTS FOR REPORT:"
echo "  1. Benchmark output above (baseline vs TSP per drone)"
echo "  2. TSP banner during launch_stage11.sh (live per drone)"
echo "  3. tsp_log.csv in LibreOffice (3 drones x 3 runs = 9 rows)"
echo ""
echo "TO RUN:"
echo "  bash ~/Desktop/FYP/launch_stage11.sh"
echo ""
echo "TO MONITOR:"
echo "  ros2 topic echo /swarm/tsp_summary"
echo ""
echo "NEXT: bash stage12_healthmap.sh"
echo ""
