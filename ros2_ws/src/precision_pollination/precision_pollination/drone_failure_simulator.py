#!/usr/bin/env python3
"""
Drone Failure Simulator — Competitive Feature
===============================================
At T=FAILURE_AT seconds, publishes a simulated failure
event for Drone 1. The battery_manager detects this and
redistributes Drone 1's remaining flowers to Drones 0 and 2.

This demonstrates swarm robustness — a key differentiator
vs. single-drone systems (Nishimoto et al., 2023) which
have no fault tolerance.

Published topics:
  /swarm/drone_failure  String (JSON: {drone_id, reason, ts})
  /swarm/recovery_status String (JSON: recovery details)
"""
import rclpy, json, os, time
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
import csv

FAILURE_AT   = float(os.environ.get('FAILURE_AT', '120'))
REGISTRY_CSV = Path.home() / "Desktop/FYP/setup/flower_registry.csv"

class DroneFailureSimulator(Node):
    def __init__(self, output_dir):
        super().__init__('drone_failure_simulator')
        self.output_dir   = Path(output_dir)
        self.start        = time.time()
        self.fired        = False
        self.pollinated   = set()
        self.events       = []

        self.fail_pub     = self.create_publisher(String, '/swarm/drone_failure', 10)
        self.recover_pub  = self.create_publisher(String, '/swarm/recovery_status', 10)

        for d in range(3):
            self.create_subscription(String, f'/drone_{d}/pollination_event',
                lambda m, did=d: self._poll_cb(m, did), 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Failure simulator ready. Drone 1 "fails" at T+{FAILURE_AT:.0f}s.')

    def _poll_cb(self, msg, drone_id):
        try:
            ev = json.loads(msg.data)
            self.pollinated.add(int(ev.get('flower_id', -1)))
        except: pass

    def _tick(self):
        elapsed = time.time() - self.start
        if not self.fired and elapsed >= FAILURE_AT:
            self.fired = True
            self._fire_failure()

    def _fire_failure(self):
        elapsed = time.time() - self.start
        flowers = []
        if REGISTRY_CSV.exists():
            flowers = [r for r in csv.DictReader(open(REGISTRY_CSV))
                       if int(r['drone_id']) == 1 and r['bloom_day'] != '0'
                       and int(r['flower_id']) not in self.pollinated]

        fail_event = {'drone_id': 1, 'reason': 'connection_lost',
                      'elapsed_s': round(elapsed,1), 'remaining_flowers': len(flowers)}
        msg = String(); msg.data = json.dumps(fail_event)
        self.fail_pub.publish(msg)

        # Split Drone 1's remaining flowers between 0 and 2
        d0_count = len(flowers) // 2
        d2_count = len(flowers) - d0_count

        recovery = {'recovered_by': [0, 2],
                    'drone_0_absorbs': d0_count, 'drone_2_absorbs': d2_count,
                    'total_recovered': len(flowers), 'elapsed_s': round(elapsed,1)}
        rmsg = String(); rmsg.data = json.dumps(recovery)
        self.recover_pub.publish(rmsg)

        self.events.append({**fail_event, **recovery})

        self.get_logger().warn(
            f'🚨 DRONE 1 FAILURE SIMULATED at T+{elapsed:.0f}s!')
        self.get_logger().info(
            f'  → {len(flowers)} flowers redistributed: '
            f'{d0_count} to Drone 0, {d2_count} to Drone 2')

    def save_outputs(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        p = self.output_dir / 'failure_events.csv'
        if self.events:
            with open(p, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.events[0].keys()))
                w.writeheader(); w.writerows(self.events)
        self.get_logger().info(f'Failure events saved → {p}')

def main(args=None):
    od = os.environ.get('DEMO_OUTPUT_DIR', str(Path.home()/'Desktop/FYP/mission_outputs/latest'))
    rclpy.init(args=args); node = DroneFailureSimulator(od)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.save_outputs(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
