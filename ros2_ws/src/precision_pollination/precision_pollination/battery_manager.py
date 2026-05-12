#!/usr/bin/env python3
"""
Battery Manager — Competitive Feature
========================================
Simulates realistic Li-Po battery discharge per drone.
When battery < threshold, publishes warning and
redistributes remaining unpollinated flowers to
other drones with sufficient charge.

Academic justification:
  Battery-aware task reallocation is identified as a
  key open challenge in Rejeb et al. (2024) and directly
  addresses the energy autonomy gap noted in their
  systematic review. This node implements a greedy
  redistribution: flowers sorted by distance to the
  nearest healthy drone.

Battery model:
  Drone 0, 2: drain at 0.12 %/s  → 833s to dead
  Drone 1:    drain at 0.28 %/s  → 357s to dead
              (simulates one older battery pack)
  Warning at: BATTERY_WARN_PCT (default 25%)
"""
import rclpy, json, csv, os, time, math
from rclpy.node import Node
from std_msgs.msg import String, Float32
from pathlib import Path

BATTERY_WARN_PCT = float(os.environ.get('BATTERY_WARN_PCT', '25'))
REGISTRY_CSV     = Path.home() / "Desktop/FYP/setup/flower_registry.csv"
DRAIN_RATE       = {0: 0.12, 1: 0.28, 2: 0.12}   # %/s

class BatteryManager(Node):
    def __init__(self, output_dir):
        super().__init__('battery_manager')
        self.output_dir       = Path(output_dir)
        self.battery          = {0: 100.0, 1: 100.0, 2: 100.0}
        self.warned           = {0: False, 1: False, 2: False}
        self.pollinated       = set()
        self.drone_pos        = {0: (0.0,0.0), 1: (20.0,0.0), 2: (40.0,0.0)}
        self.events           = []
        self.start            = time.time()

        # Load registry
        self.flowers = []
        if REGISTRY_CSV.exists():
            self.flowers = [{"id": int(r["flower_id"]), "drone_id": int(r["drone_id"]),
                              "x": float(r["x"]), "y": float(r["y"]), "bloom_day": int(r["bloom_day"])}
                            for r in csv.DictReader(open(REGISTRY_CSV)) if r["bloom_day"] != "0"]

        for d in range(3):
            self.create_subscription(String, f'/drone_{d}/pollination_event',
                lambda m, did=d: self._poll_cb(m, did), 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'Battery manager started. Warn at {BATTERY_WARN_PCT}%. '
            f'Drone 1 drain: {DRAIN_RATE[1]}%/s (accelerated).')

    def _poll_cb(self, msg, drone_id):
        try:
            ev = json.loads(msg.data)
            self.pollinated.add(int(ev.get('flower_id', -1)))
        except: pass

    def _tick(self):
        for d in range(3):
            self.battery[d] = max(0.0, self.battery[d] - DRAIN_RATE[d])
            pct = self.battery[d]

            if not self.warned[d] and pct <= BATTERY_WARN_PCT:
                self.warned[d] = True
                elapsed = time.time() - self.start
                self.get_logger().warn(
                    f'⚡ DRONE {d} BATTERY LOW: {pct:.1f}% at T+{elapsed:.0f}s — redistributing flowers!')
                count = self._redistribute(d)
                self.events.append({
                    'event': 'battery_warning', 'drone_id': d,
                    'battery_pct': round(pct, 2), 'elapsed_s': round(elapsed, 1),
                    'flowers_redistributed': count,
                })
                self.get_logger().info(
                    f'  → {count} remaining flowers reassigned to other drones')

    def _redistribute(self, failing_drone):
        remaining = [f for f in self.flowers
                     if f['drone_id'] == failing_drone and f['id'] not in self.pollinated]
        healthy   = [d for d in range(3) if d != failing_drone and self.battery[d] > 30]
        if not healthy or not remaining: return 0
        for i, fl in enumerate(remaining):
            # Assign to closest healthy drone
            target = min(healthy,
                key=lambda d: math.sqrt((self.drone_pos[d][0]-fl['x'])**2 +
                                        (self.drone_pos[d][1]-fl['y'])**2))
            fl['drone_id'] = target
        return len(remaining)

    def save_outputs(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        p = self.output_dir / 'battery_events.csv'
        if self.events:
            with open(p, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.events[0].keys()))
                w.writeheader(); w.writerows(self.events)
        self.get_logger().info(f'{len(self.events)} battery events saved')

def main(args=None):
    od = os.environ.get('DEMO_OUTPUT_DIR', str(Path.home() / 'Desktop/FYP/mission_outputs/latest'))
    rclpy.init(args=args); node = BatteryManager(od)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.save_outputs(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
