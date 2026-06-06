#!/bin/bash
# ============================================================
# FIX (overwrite) — PX4 topic discovery
#
# Overwrites swarm_drone_controller.py and field_survey.py with
# corrected versions. PX4 v1.16 versions the TOPIC NAME
# (vehicle_status_v4) but keeps the message TYPE unversioned
# (VehicleStatus). These nodes import the unversioned types and
# auto-discover the real topic name at runtime.
#
# No rebuild needed (symlink-install).
#
# RUN: bash fix_overwrite_nodes.sh
# THEN relaunch:
#   pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f gz-sim
#   pkill -9 -f ruby; pkill -9 -f MicroXRCEAgent; sleep 3
#   bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e
NODES_DIR=~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination

echo "Overwriting field_survey.py and swarm_drone_controller.py..."

# ── field_survey.py ───────────────────────────────────────────
cat > $NODES_DIR/field_survey.py << 'PYEOF'
#!/usr/bin/env python3
"""field_survey.py - simulated aerial YOLO scan. When the drone is
at survey altitude, publishes the full field map (all flowers within
survey radius). Text messages only, no photos.

PX4 v1.16 versions the TOPIC NAME (e.g. vehicle_local_position_v1)
but the message TYPE stays unversioned -> import VehicleLocalPosition
and discover the real topic name (any _vN suffix) at runtime."""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition as VLP
import json, math

SURVEY_RADIUS = 10.0
SURVEY_MIN_ALT = 2.0
MODEL_MAP50 = 0.855
JSONF = '/tmp/stage14_flowers.json'


class FieldSurvey(Node):
    def __init__(self):
        super().__init__('field_survey')
        self.declare_parameter('drone_id', 0)
        self.did = self.get_parameter('drone_id').value
        with open(JSONF) as f:
            self.flowers = json.load(f)['flowers']
        self.pos = None
        self.published = False
        self._out_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pos_sub = None
        self._resolver = self.create_timer(1.0, self._resolve_px4_topics)
        self.pub = self.create_publisher(String, f'/drone_{self.did}/field_map', 10)
        self.create_timer(0.3, self._loop)
        self.get_logger().info(
            f'[D{self.did}] Survey sensor online (YOLOv8s-OBB, '
            f'mAP@0.5={MODEL_MAP50}). Will scan at >= {SURVEY_MIN_ALT}m altitude.')

    def _resolve_px4_topics(self):
        ns = f'/px4_{self.did}/fmu/out/'
        names = dict(self.get_topic_names_and_types())
        def find(base):
            if ns + base in names:
                return ns + base
            for t in names:
                if t.startswith(ns + base + '_v'):
                    return t
            return None
        if self._pos_sub is None:
            t = find('vehicle_local_position')
            if t:
                self._pos_sub = self.create_subscription(VLP, t, self._pos, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed position topic: {t}')
                self._resolver.cancel()

    def _pos(self, m):
        self.pos = m

    def _loop(self):
        if self.pos is None:
            return
        alt = -self.pos.z
        if alt < SURVEY_MIN_ALT:
            return
        seen = [fl for fl in self.flowers
                if math.hypot(self.pos.x-fl['x'], self.pos.y-fl['y']) <= SURVEY_RADIUS]
        self.pub.publish(String(data=json.dumps({'flowers': seen})))
        if not self.published and seen:
            self.published = True
            self.get_logger().info(
                f'\n[D{self.did}] AERIAL SURVEY complete @ {alt:.1f}m - '
                f'{len(seen)} flowers detected:')
            for fl in seen:
                self.get_logger().info(
                    f'   Flower {fl["id"]} at ({fl["x"]:.2f}, {fl["y"]:.2f})')


def main(args=None):
    rclpy.init(args=args)
    n = FieldSurvey()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF
echo "  field_survey.py written"

# ── swarm_drone_controller.py ─────────────────────────────────
cat > $NODES_DIR/swarm_drone_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
swarm_drone_controller.py - peer-to-peer swarm pollination.
No central coordinator: drones exchange position/priority/claims
over /swarm/peer and each runs the SAME greedy assignment locally.
Position-based motion (no dance), altitude-layered collision
avoidance, force-arm to bypass GCS preflight in SITL.

PX4 v1.16 versions the TOPIC NAME (vehicle_status_v4,
vehicle_local_position_v1) but keeps the message TYPE unversioned.
We import the unversioned types and discover the real topic name
(any _vN suffix) at runtime.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from px4_msgs.msg import (TrajectorySetpoint, VehicleCommand, OffboardControlMode)
from px4_msgs.msg import VehicleLocalPosition as VLP
from px4_msgs.msg import VehicleStatus as VS
from std_msgs.msg import String
import json, math, time, random
from enum import Enum

SPAWN      = {0: (2.0, 0.0), 1: (6.0, 0.0)}
SURVEY_POS = {0: (2.5, 3.25), 1: (5.5, 3.25)}
CRUISE_ALT = {0: -3.0, 1: -3.4}
LIFT_ALT   = -0.4
POLLINATE_ALT = -2.0
GROUND     = 0.0

WP_TOL = 0.30
ALT_TOL = 0.20
ARMED = 2
FORCE_ARM = 21196.0
PREFLIGHT_TICKS = 30
ARM_RETRY = 8
HOVER_TICKS = 30
SAFETY_HORIZ = 1.3
SAFETY_VERT = 0.6
CAP = 3


def greedy_assign(positions, flowers, priorities):
    assigned = {0: [], 1: []}
    remaining = list(flowers)
    while remaining:
        cands = []
        for d in (0, 1):
            if len(assigned[d]) < CAP:
                px, py = positions[d]
                for fl in remaining:
                    dist = math.hypot(px-fl['x'], py-fl['y'])
                    cands.append((dist, d, fl))
        if not cands:
            break
        mind = min(c[0] for c in cands)
        tied = [c for c in cands if c[0]-mind < 1e-3]
        tied.sort(key=lambda c: (-priorities[c[1]], c[2]['id']))
        _, d, fl = tied[0]
        assigned[d].append(fl)
        remaining.remove(fl)
    return assigned


class Phase(Enum):
    PREFLIGHT="PREFLIGHT"; ARMING="ARMING"; SYNC="SYNC_HOVER"
    CLIMB="CLIMB"; GOTO_SURVEY="GOTO_SURVEY"; SURVEY="SURVEY"
    NEGOTIATE="NEGOTIATE"; P_GOTO="POLLINATE_GOTO"; P_DESC="POLLINATE_DESCEND"
    P_HOVER="POLLINATE_HOVER"; P_ASC="POLLINATE_ASCEND"
    RETURN="RETURN"; LAND="LAND"; DONE="DONE"


class SwarmDrone(Node):
    def __init__(self):
        super().__init__('swarm_drone_controller')
        self.declare_parameter('drone_id', 0)
        self.did = self.get_parameter('drone_id').value
        self.other = 1 - self.did
        self.spawn = SPAWN[self.did]
        self.survey = SURVEY_POS[self.did]
        self.cruise = CRUISE_ALT[self.did]
        self.priority = random.random()

        self.phase = Phase.PREFLIGHT
        self.pos = None; self.vstatus = None
        self.flowers = None
        self.peer = {}
        self.assignment = None
        self.my_flowers = []; self.fi = 0
        self.pre_t = 0; self.arm_t = 0; self.hov_t = 0
        self.t0 = time.time()
        self.done_logged = False

        out_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        in_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        ns = f'/px4_{self.did}'

        # PX4 out-topics discovered at runtime (versioned name, unversioned type)
        self._out_qos = out_qos
        self._pos_sub = None
        self._vs_sub = None
        self._resolver = self.create_timer(1.0, self._resolve_px4_topics)

        self.create_subscription(String, f'/drone_{self.did}/field_map', self._map_cb, 10)
        self.create_subscription(String, '/swarm/peer', self._peer_cb, 10)

        self.sp = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', in_qos)
        self.ocm = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', in_qos)
        self.cmd = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', in_qos)
        self.peer_pub = self.create_publisher(String, '/swarm/peer', 10)

        self.create_timer(0.05, self._loop)
        self.create_timer(0.1, self._broadcast)
        self.get_logger().info(
            f'[D{self.did}] Swarm controller online. spawn={self.spawn} '
            f'survey={self.survey} cruise={-self.cruise:.1f}m '
            f'priority={self.priority:.3f}. Flowers UNKNOWN until survey.')

    def _resolve_px4_topics(self):
        ns = f'/px4_{self.did}/fmu/out/'
        names = dict(self.get_topic_names_and_types())
        def find(base):
            if ns + base in names:
                return ns + base
            for t in names:
                if t.startswith(ns + base + '_v'):
                    return t
            return None
        if self._pos_sub is None:
            t = find('vehicle_local_position')
            if t:
                self._pos_sub = self.create_subscription(VLP, t, self._pos_cb, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed position topic: {t}')
        if self._vs_sub is None:
            t = find('vehicle_status')
            if t:
                self._vs_sub = self.create_subscription(VS, t, self._vs_cb, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed status topic: {t}')
        if self._pos_sub and self._vs_sub:
            self._resolver.cancel()
            self.get_logger().info(f'[D{self.did}] PX4 topic discovery complete')

    def _pos_cb(self, m):
        if self.pos is None: self.get_logger().info(f'[D{self.did}] Position lock')
        self.pos = m
    def _vs_cb(self, m): self.vstatus = m
    def _map_cb(self, m):
        if self.flowers is None:
            self.flowers = json.loads(m.data)['flowers']
            self.get_logger().info(f'[D{self.did}] field map received: '
                                   f'{len(self.flowers)} flowers')
    def _peer_cb(self, m):
        try:
            d = json.loads(m.data)
            if d['drone_id'] == self.other:
                self.peer = d
        except Exception:
            pass

    def _p(self):
        return None if self.pos is None else (self.pos.x, self.pos.y, self.pos.z)
    def _d2(self, tx, ty):
        p=self._p(); return 999.0 if p is None else math.hypot(p[0]-tx, p[1]-ty)
    def _armed(self):
        return self.vstatus is not None and self.vstatus.arming_state == ARMED
    def _go(self, ph, r=''):
        self.get_logger().info(f'[D{self.did}] {self.phase.value} -> {ph.value}'
                               + (f'  ({r})' if r else '')); self.phase = ph
    def _yaw(self, tx, ty):
        p=self._p(); return 0.0 if p is None else math.atan2(ty-p[1], tx-p[0])
    def _set_ocm(self):
        o=OffboardControlMode(); o.position=True; o.velocity=False
        o.acceleration=o.attitude=o.body_rate=False
        o.timestamp=int(self.get_clock().now().nanoseconds/1000); self.ocm.publish(o)
    def _pp(self, x, y, z, yaw=0.0):
        s=TrajectorySetpoint(); s.position=[float(x),float(y),float(z)]
        s.velocity=[float('nan')]*3; s.yaw=float(yaw)
        s.timestamp=int(self.get_clock().now().nanoseconds/1000); self.sp.publish(s)
    def _send(self, c, p1=0.0, p2=0.0):
        m=VehicleCommand(); m.command=c; m.param1=float(p1); m.param2=float(p2)
        m.target_system=self.did+1; m.target_component=1
        m.source_system=1; m.source_component=1; m.from_external=True
        m.timestamp=int(self.get_clock().now().nanoseconds/1000); self.cmd.publish(m)

    def _peer_armed(self):
        return bool(self.peer.get('armed', False))
    def _peer_priority(self):
        return self.peer.get('priority', None)

    def _should_yield(self):
        p=self._p()
        if p is None or not self.peer: return False
        px, py, pz = self.peer.get('x'), self.peer.get('y'), self.peer.get('z')
        if px is None: return False
        horiz = math.hypot(p[0]-px, p[1]-py)
        vert = abs(p[2]-pz)
        if horiz < SAFETY_HORIZ and vert < SAFETY_VERT:
            pp = self._peer_priority()
            if pp is not None and self.priority < pp:
                return True
        return False

    def _broadcast(self):
        p=self._p()
        self.peer_pub.publish(String(data=json.dumps({
            'drone_id': self.did,
            'x': round(p[0],2) if p else None,
            'y': round(p[1],2) if p else None,
            'z': round(p[2],2) if p else None,
            'priority': self.priority,
            'armed': self._armed(),
            'phase': self.phase.value,
            'claimed': [fl['id'] for fl in self.my_flowers]})))

    def _loop(self):
        self._set_ocm()
        p=self._p(); sx, sy = self.spawn

        if self.phase==Phase.PREFLIGHT:
            self._pp(sx, sy, GROUND); self.pre_t += 1
            if self.pre_t>=PREFLIGHT_TICKS: self._go(Phase.ARMING,'stream ready'); self.arm_t=0

        elif self.phase==Phase.ARMING:
            self._pp(sx, sy, GROUND); self.arm_t += 1
            if self.arm_t%ARM_RETRY==2:
                self._send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.arm_t%ARM_RETRY==5:
                self._send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, FORCE_ARM)
            if self._armed():
                self._go(Phase.SYNC,'armed - lifting to hover')
            elif self.arm_t % 100 == 0:
                self.get_logger().warn(
                    f'[D{self.did}] still arming (t={self.arm_t}). '
                    f'VehicleStatus {"present" if self.vstatus else "ABSENT"}.')

        elif self.phase==Phase.SYNC:
            self._pp(sx, sy, LIFT_ALT)
            if self._peer_armed():
                self._go(Phase.CLIMB,'both armed - climbing together')

        elif self.phase==Phase.CLIMB:
            self._pp(sx, sy, self.cruise)
            if p and abs(p[2]-self.cruise)<0.4:
                self._go(Phase.GOTO_SURVEY,'at cruise')

        elif self.phase==Phase.GOTO_SURVEY:
            if self._should_yield():
                self._pp(p[0], p[1], self.cruise); return
            tx, ty = self.survey
            self._pp(tx, ty, self.cruise, self._yaw(tx, ty))
            if self._d2(tx, ty)<WP_TOL:
                self._go(Phase.SURVEY,'at survey point - reading field')

        elif self.phase==Phase.SURVEY:
            tx, ty = self.survey
            self._pp(tx, ty, self.cruise)
            if self.flowers is not None and self._peer_priority() is not None:
                self._go(Phase.NEGOTIATE,'field read + peer online')

        elif self.phase==Phase.NEGOTIATE:
            tx, ty = self.survey
            self._pp(tx, ty, self.cruise)
            pris = {self.did: self.priority, self.other: self._peer_priority()}
            self.assignment = greedy_assign(SURVEY_POS, self.flowers, pris)
            mine = self.assignment[self.did]
            mine.sort(key=lambda fl: math.hypot(self.survey[0]-fl['x'],
                                                self.survey[1]-fl['y']))
            self.my_flowers = mine; self.fi = 0
            ids = [fl['id'] for fl in mine]
            other_ids = [fl['id'] for fl in self.assignment[self.other]]
            self.get_logger().info(
                f'\n[D{self.did}] NEGOTIATION COMPLETE (peer-to-peer)\n'
                f'   I claim flowers {ids}\n'
                f'   peer takes {other_ids}\n'
                f'   (greedy closest-first, priority tie-break, no repeats)')
            self._go(Phase.P_GOTO,'starting pollination')

        elif self.phase==Phase.P_GOTO:
            if self.fi>=len(self.my_flowers):
                self._go(Phase.RETURN,'all flowers done'); return
            if self._should_yield():
                self._pp(p[0], p[1], self.cruise); return
            fl=self.my_flowers[self.fi]
            self._pp(fl['x'], fl['y'], self.cruise, self._yaw(fl['x'], fl['y']))
            if self._d2(fl['x'], fl['y'])<WP_TOL:
                self._go(Phase.P_DESC,f'over flower {fl["id"]}')

        elif self.phase==Phase.P_DESC:
            fl=self.my_flowers[self.fi]
            self._pp(fl['x'], fl['y'], POLLINATE_ALT)
            if p and abs(p[2]-POLLINATE_ALT)<ALT_TOL:
                self._go(Phase.P_HOVER,'at 2m hover'); self.hov_t=0

        elif self.phase==Phase.P_HOVER:
            fl=self.my_flowers[self.fi]
            self._pp(fl['x'], fl['y'], POLLINATE_ALT); self.hov_t += 1
            if self.hov_t==1:
                self.get_logger().info(
                    f'[D{self.did}] POLLINATING flower {fl["id"]} '
                    f'at ({fl["x"]:.2f},{fl["y"]:.2f}) - hovering 2m (no crush)')
            if self.hov_t>=HOVER_TICKS:
                self._go(Phase.P_ASC,f'flower {fl["id"]} done')

        elif self.phase==Phase.P_ASC:
            fl=self.my_flowers[self.fi]
            self._pp(fl['x'], fl['y'], self.cruise)
            if p and abs(p[2]-self.cruise)<0.4:
                self.fi += 1
                self._go(Phase.P_GOTO,f'next ({self.fi}/{len(self.my_flowers)})')

        elif self.phase==Phase.RETURN:
            if self._should_yield():
                self._pp(p[0], p[1], self.cruise); return
            self._pp(sx, sy, self.cruise, self._yaw(sx, sy))
            if self._d2(sx, sy)<WP_TOL:
                self._go(Phase.LAND,'home reached')

        elif self.phase==Phase.LAND:
            self._pp(sx, sy, GROUND)
            if p and abs(p[2])<0.20:
                self._send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self._go(Phase.DONE,'landed at spawn')

        elif self.phase==Phase.DONE:
            if not self.done_logged:
                self.get_logger().info(
                    f'[D{self.did}] MISSION COMPLETE - pollinated '
                    f'{len(self.my_flowers)} flowers, landed at {self.spawn}')
                self.done_logged=True


def main(args=None):
    rclpy.init(args=args)
    n=SwarmDrone()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
PYEOF
echo "  swarm_drone_controller.py written"

echo ""
echo "[verify] syntax..."
python3 -c "import ast; ast.parse(open('$NODES_DIR/field_survey.py').read()); print('  field_survey OK')"
python3 -c "import ast; ast.parse(open('$NODES_DIR/swarm_drone_controller.py').read()); print('  swarm_drone_controller OK')"

echo ""
echo "=============================================="
echo "  DONE - both nodes overwritten with topic"
echo "  discovery. No rebuild needed (symlink-install)."
echo "=============================================="
echo ""
echo "Relaunch:"
echo "  pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f gz-sim"
echo "  pkill -9 -f ruby; pkill -9 -f MicroXRCEAgent; sleep 3"
echo "  bash ~/Desktop/FYP/launch_stage14.sh"
echo ""
echo "Each controller should print within ~5s:"
echo "  subscribed position topic: .../vehicle_local_position_v1"
echo "  subscribed status topic:   .../vehicle_status_v4"
echo "  Position lock"
echo "then arm -> lift -> climb together."
echo ""
