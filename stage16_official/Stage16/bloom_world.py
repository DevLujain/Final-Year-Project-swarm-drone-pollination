#!/usr/bin/env python3
"""
bloom_world.py  —  Stage 16 dynamic-bloom world node
====================================================
WORLD-SIMULATOR ROLE (not a drone brain). This node owns ground truth and
manifests it physically: each flower starts as a green BUD and OPENS into a
yellow sunflower on its bloom day, by raising the open-head model and
lowering the bud model with Gazebo's set_pose service.

NOT A CHEAT: the drones never read this node or the flower registry. This is
the *world* making flowers bloom over time, exactly as a real field would.
Perception is still the drones' only source of flower information.

Field_generator emits THREE models per flower:
    stem_{id}    stem + leaves, STATIC, always present
    bud_{id}     green CLOSED head — visible when closed, parked underground
                 when open
    flower_{id}  yellow OPEN head + brown disc — parked underground when
                 closed, raised to head_z when open

On each /bloom/current_day, every flower with bloom_start <= day that isn't
open yet gets opened (flower up, bud down). Idempotent and crash-safe: the
full open-set is re-derived from the day, so a missed or repeated day
self-corrects. Flowers stay open once opened (so by the last day the whole
field is yellow).
"""

from __future__ import annotations

import json
import os
import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from std_msgs.msg import Int32


def _expand(p):
    return os.path.expanduser(os.path.expandvars(p))


# Must match BLOOM_BURY_DZ in Field_generator.py — how far below its head a
# hidden flower/bud model is parked so it's out of the camera's view.
BURY_DZ = 50.0


class BloomWorld(Node):

    def __init__(self):
        super().__init__('bloom_world')

        self.declare_parameter('world_name',          'stage16_world')
        self.declare_parameter('flowers_json',        '/tmp/stage16_flowers.json')
        self.declare_parameter('current_day',         5)
        self.declare_parameter('set_pose_timeout_ms', 800)
        self.declare_parameter('swaps_per_tick',      6)

        self.world_name   = str(self.get_parameter('world_name').value)
        self.flowers_json = str(self.get_parameter('flowers_json').value)
        self.start_day    = int(self.get_parameter('current_day').value)
        self.sp_timeout   = int(self.get_parameter('set_pose_timeout_ms').value)
        self.n_per_tick   = int(self.get_parameter('swaps_per_tick').value)

        self.flowers = self._load_registry()
        self.day = -1
        self.opened: set[int] = set()
        self.queue: deque = deque()   # (kind, fid, x, y, z)

        # Flowers already bloomed by the start day are rendered OPEN by
        # Field_generator, so pre-mark them as opened — no swap needed. Only
        # blooms that happen on later days get physically swapped.
        for fl in self.flowers:
            if int(fl['bloom_start']) <= self.start_day:
                self.opened.add(int(fl['flower_id']))

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int32, '/bloom/current_day',
                                 self._on_day, latched)
        # Drain the swap queue a few at a time so we never block the executor
        # with a long burst of subprocess set_pose calls.
        self.create_timer(0.2, self._drain_queue)

        self.get_logger().info(
            f"bloom_world up [v1]. world={self.world_name} "
            f"flowers={len(self.flowers)} start_day={self.start_day} "
            f"already-open={len(self.opened)} (world-sim role — not a drone)")

    def _load_registry(self):
        path = _expand(self.flowers_json)
        try:
            with open(path) as f:
                data = json.load(f)
            return list(data.get('flowers', []))
        except Exception as e:
            self.get_logger().error(f"couldn't load registry {path}: {e}")
            return []

    def _on_day(self, m: Int32):
        day = int(m.data)
        if day == self.day:
            return
        self.day = day
        newly = []
        for fl in self.flowers:
            fid = int(fl['flower_id'])
            if fid in self.opened:
                continue
            if day >= int(fl['bloom_start']):
                self.opened.add(fid)
                hz = float(fl.get('head_z', 1.7))
                x, y = float(fl['x']), float(fl['y'])
                self.queue.append(('flower', fid, x, y, hz))            # raise open head
                self.queue.append(('bud',    fid, x, y, hz - BURY_DZ))  # lower bud
                newly.append(fid)
        if newly:
            self.get_logger().info(
                f"day {day}: opening {len(newly)} flower(s) "
                f"(total open {len(self.opened)}/{len(self.flowers)}); "
                f"{len(self.queue)} pose swaps queued")
        else:
            self.get_logger().info(
                f"day {day}: no new blooms "
                f"(total open {len(self.opened)}/{len(self.flowers)})")

    def _drain_queue(self):
        for _ in range(max(1, self.n_per_tick)):
            if not self.queue:
                return
            kind, fid, x, y, z = self.queue.popleft()
            self._set_pose(f"{kind}_{fid}", x, y, z)

    def _set_pose(self, model, x, y, z):
        svc = f'/world/{self.world_name}/set_pose'
        req = (f'name: "{model}", position: {{x: {x:.3f}, y: {y:.3f}, '
               f'z: {z:.3f}}}, orientation: {{w: 1.0}}')
        try:
            subprocess.run(
                ['gz', 'service', '-s', svc,
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', str(self.sp_timeout), '--req', req],
                timeout=2.0, capture_output=True, text=True)
        except Exception as e:
            self.get_logger().warning(f"set_pose {model} failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    n = BloomWorld()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
