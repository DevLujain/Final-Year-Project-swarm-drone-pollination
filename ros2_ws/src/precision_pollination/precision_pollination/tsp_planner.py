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
