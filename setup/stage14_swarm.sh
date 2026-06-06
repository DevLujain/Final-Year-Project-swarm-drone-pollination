#!/bin/bash
# ============================================================
# STAGE 14 — SWARM POLLINATION (peer-to-peer, 6 flowers)
# FYP: Autonomous Swarm Drone Pollination
#
# Mission:
#   1. Two drones arm WITHOUT dancing, lift to a stable 0.4m
#      hover, then climb to 3m TOGETHER (sync barrier).
#   2. Both fly to centre survey spots (3m apart, altitude-
#      layered) and read the whole field (all 6 flowers).
#   3. PEER-TO-PEER negotiation: each runs an identical greedy
#      closest-first assignment (3 flowers each, no repeats,
#      random tie-break via exchanged priorities) and
#      broadcasts its claim. No central coordinator.
#   4. Each pollinates its 3 flowers: 3m -> 2m hover -> 3m,
#      nearest-first. Collision avoidance: layered altitudes
#      + lower-priority drone yields if too close.
#   5. Both return to their exact spawn and land.
#
# Field: one shared area x[0,8] y[1,5.5], 6 random flowers
#        (re-rolled each launch, min 1.4m spacing).
# Camera: fixed, centred behind the field at (4,-7,7).
#
# RUN:  bash stage14_swarm.sh
# THEN: bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination
PX4_DIR=~/PX4-Autopilot
WORLDS_DIR=$PX4_DIR/Tools/simulation/gz/worlds

mkdir -p $FYP_DIR/stage14_outputs
source /opt/ros/jazzy/setup.bash

echo ""
echo "=============================================================="
echo "  STAGE 14 — Swarm Pollination (peer-to-peer, 6 flowers)"
echo "=============================================================="
echo ""

# ── 0. Clean old nodes ────────────────────────────────────────
echo "[0/6] Cleaning old Stage 14 nodes..."
rm -f $NODES_DIR/yolo_detection_sim.py \
      $NODES_DIR/visual_servoing_controller.py \
      $NODES_DIR/field_detection.py \
      $NODES_DIR/search_pollinate_controller.py \
      $NODES_DIR/swarm_coordinator_2drone.py \
      $NODES_DIR/swarm_drone_controller.py \
      $NODES_DIR/field_survey.py \
      $NODES_DIR/swarm_monitor.py
echo "cleaned"

# ── 1. Field generator ────────────────────────────────────────
echo ""
echo "[1/6] Writing generate_field.py..."

cat > $FYP_DIR/generate_field.py << 'PYEOF'
#!/usr/bin/env python3
"""Build one field with 6 RANDOM flowers (min spacing). Writes
/tmp/stage14_flowers.json (read by survey nodes only)."""
import os, json, random, re, math

DEFAULT = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf')
TARGET = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/worlds/demo_2flower.sdf')
JSONF = '/tmp/stage14_flowers.json'

# Field usable area (margins from edges)
XMIN, XMAX, YMIN, YMAX = 0.7, 7.3, 1.3, 5.2
MIN_SPACING = 1.4
N = 6

pts = []
tries = 0
while len(pts) < N and tries < 2000:
    tries += 1
    x = round(random.uniform(XMIN, XMAX), 2)
    y = round(random.uniform(YMIN, YMAX), 2)
    if all(math.hypot(x-px, y-py) >= MIN_SPACING for px, py in pts):
        pts.append((x, y))

flowers = [{"id": i+1, "x": x, "y": y, "z": 0.87} for i, (x, y) in enumerate(pts)]
with open(JSONF, 'w') as f:
    json.dump({"flowers": flowers}, f, indent=2)

with open(DEFAULT) as f:
    sdf = f.read()
sdf = re.sub(r'<world name="[^"]*"', '<world name="demo_2flower"', sdf, 1)

GUI = '''    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>4 -7 7 0 0.55 1.5708</pose>
      </camera>
    </gui>
'''
if '<gui' not in sdf:
    sdf = re.sub(r'(<world name="demo_2flower">)', r'\1\n' + GUI, sdf, 1)

def tile(name, x, y, w, h, r, g, b, a, z=0.02):
    return (f'    <model name="{name}"><static>true</static>'
            f'<pose>{x} {y} {z} 0 0 0</pose><link name="l"><visual name="v">'
            f'<geometry><box><size>{w} {h} 0.02</size></box></geometry>'
            f'<material><ambient>{r} {g} {b} {a}</ambient>'
            f'<diffuse>{r} {g} {b} {a}</diffuse></material></visual></link></model>\n')

def flower(name, x, y):
    return (f'    <model name="{name}"><static>true</static>'
            f'<pose>{x} {y} 0 0 0 0</pose>'
            f'<link name="stem"><pose>0 0 0.4 0 0 0</pose><visual name="v">'
            f'<geometry><cylinder><radius>0.05</radius><length>0.8</length></cylinder></geometry>'
            f'<material><ambient>0.1 0.55 0.1 1</ambient><diffuse>0.1 0.55 0.1 1</diffuse></material>'
            f'</visual></link>'
            f'<link name="petals"><pose>0 0 0.82 0 0 0</pose><visual name="v">'
            f'<geometry><cylinder><radius>0.34</radius><length>0.03</length></cylinder></geometry>'
            f'<material><ambient>0.95 0.82 0.05 1</ambient><diffuse>0.95 0.82 0.05 1</diffuse></material>'
            f'</visual></link>'
            f'<link name="head"><pose>0 0 0.84 0 0 0</pose><visual name="v">'
            f'<geometry><cylinder><radius>0.2</radius><length>0.05</length></cylinder></geometry>'
            f'<material><ambient>0.42 0.25 0.05 1</ambient><diffuse>0.42 0.25 0.05 1</diffuse></material>'
            f'</visual></link></model>\n')

extras = "\n    <!-- STAGE 14 swarm field -->\n"
extras += tile("field_tile", 4.0, 3.25, 8.0, 4.5, 0.15, 0.4, 0.18, 0.18, 0.01)
for fl in flowers:
    extras += flower(f"sunflower_{fl['id']}", fl['x'], fl['y'])
sdf = sdf.replace('</world>', extras + '\n  </world>')
with open(TARGET, 'w') as f:
    f.write(sdf)

print("Field generated with 6 random flowers:")
for fl in flowers:
    print(f"  Flower {fl['id']}: ({fl['x']}, {fl['y']})")
print("(positions hidden from controllers; survey nodes only)")
PYEOF
echo "generate_field.py written"

# ── 2. Per-drone survey (aerial scan -> full field map) ───────
echo ""
echo "[2/6] Writing field_survey.py..."

cat > $NODES_DIR/field_survey.py << 'PYEOF'
#!/usr/bin/env python3
"""field_survey.py - simulated aerial YOLO scan. When the drone is
at survey altitude, publishes the full field map (all flowers within
survey radius). Text messages only, no photos."""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from std_msgs.msg import String
import json, math

try:
    from px4_msgs.msg import VehicleLocalPositionV1 as VLP
    POS_TOPIC = 'vehicle_local_position_v1'
except ImportError:
    from px4_msgs.msg import VehicleLocalPosition as VLP
    POS_TOPIC = 'vehicle_local_position'

SURVEY_RADIUS = 10.0     # covers whole small field from altitude
SURVEY_MIN_ALT = 2.0     # must be this high (m) to scan
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
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        ns = f'/px4_{self.did}'
        self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', self._pos, qos)
        self.pub = self.create_publisher(String, f'/drone_{self.did}/field_map', 10)
        self.create_timer(0.3, self._loop)
        self.get_logger().info(
            f'[D{self.did}] Survey sensor online (YOLOv8s-OBB, '
            f'mAP@0.5={MODEL_MAP50}). Will scan at >= {SURVEY_MIN_ALT}m altitude.')

    def _pos(self, m): self.pos = m

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
echo "field_survey.py written"

# ── 3. Swarm drone controller (peer-to-peer) ──────────────────
echo ""
echo "[3/6] Writing swarm_drone_controller.py..."

cat > $NODES_DIR/swarm_drone_controller.py << 'PYEOF'
#!/usr/bin/env python3
"""
swarm_drone_controller.py - peer-to-peer swarm pollination.
No central coordinator: drones exchange position/priority/claims
over /swarm/peer and each runs the SAME greedy assignment locally.
Position-based motion (no dance), altitude-layered collision
avoidance, force-arm to bypass GCS preflight in SITL.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from px4_msgs.msg import (TrajectorySetpoint, VehicleCommand, OffboardControlMode)
from std_msgs.msg import String
import json, math, time, random
from enum import Enum

try:
    from px4_msgs.msg import VehicleLocalPositionV1 as VLP
    POS_TOPIC = 'vehicle_local_position_v1'
except ImportError:
    from px4_msgs.msg import VehicleLocalPosition as VLP
    POS_TOPIC = 'vehicle_local_position'
try:
    from px4_msgs.msg import VehicleStatusV4 as VS
    STATUS_TOPIC = 'vehicle_status_v4'
except ImportError:
    from px4_msgs.msg import VehicleStatus as VS
    STATUS_TOPIC = 'vehicle_status'

# ── Geometry ───────────────────────────────────────────────────
SPAWN      = {0: (2.0, 0.0), 1: (6.0, 0.0)}
SURVEY_POS = {0: (2.5, 3.25), 1: (5.5, 3.25)}   # fixed -> deterministic assignment
CRUISE_ALT = {0: -3.0, 1: -3.4}                 # altitude-layered (collision avoid)
LIFT_ALT   = -0.4        # stable low hover while syncing (no ground dance)
POLLINATE_ALT = -2.0     # hover height over flower (no crush)
GROUND     = 0.0

WP_TOL = 0.30
ALT_TOL = 0.20
ARMED = 2
FORCE_ARM = 21196.0
PREFLIGHT_TICKS = 30
ARM_RETRY = 8
HOVER_TICKS = 30         # 1.5 s pollination
SAFETY_HORIZ = 1.3       # collision-avoid horizontal radius
SAFETY_VERT = 0.6        # collision-avoid vertical band
CAP = 3                  # flowers per drone


def greedy_assign(positions, flowers, priorities):
    """Identical on both drones: closest-first, cap 3, priority tie-break."""
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
        # tie-break: higher priority drone wins, then lowest flower id
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
        self.priority = random.random()    # random -> random tie-break

        self.phase = Phase.PREFLIGHT
        self.pos = None; self.vstatus = None
        self.flowers = None                 # from own survey
        self.peer = {}                      # latest peer message
        self.assignment = None
        self.my_flowers = []; self.fi = 0
        self.pre_t = 0; self.arm_t = 0; self.hov_t = 0
        self.t0 = time.time()
        self.done_logged = False; self.arm_warned = 0

        out_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        in_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        ns = f'/px4_{self.did}'
        self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', self._pos_cb, out_qos)
        self.create_subscription(VS, f'{ns}/fmu/out/{STATUS_TOPIC}', self._vs_cb, out_qos)
        self.create_subscription(String, f'/drone_{self.did}/field_map', self._map_cb, 10)
        self.create_subscription(String, '/swarm/peer', self._peer_cb, 10)

        self.sp = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', in_qos)
        self.ocm = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', in_qos)
        self.cmd = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', in_qos)
        self.peer_pub = self.create_publisher(String, '/swarm/peer', 10)

        self.create_timer(0.05, self._loop)       # 20 Hz control
        self.create_timer(0.1, self._broadcast)    # 10 Hz peer beacon
        self.get_logger().info(
            f'[D{self.did}] Swarm controller online. spawn={self.spawn} '
            f'survey={self.survey} cruise={-self.cruise:.1f}m '
            f'priority={self.priority:.3f}. Flowers UNKNOWN until survey.')

    # ── callbacks ──
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

    # ── helpers ──
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
        """Collision avoidance: lower-priority drone holds if too close."""
        p=self._p()
        if p is None or not self.peer: return False
        px, py, pz = self.peer.get('x'), self.peer.get('y'), self.peer.get('z')
        if px is None: return False
        horiz = math.hypot(p[0]-px, p[1]-py)
        vert = abs(p[2]-pz)
        if horiz < SAFETY_HORIZ and vert < SAFETY_VERT:
            # lower priority yields
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

    # ── main loop ──
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
            # stable low hover (no ground dance) until BOTH armed
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
            # need own flower map + peer priority before negotiating
            if self.flowers is not None and self._peer_priority() is not None:
                self._go(Phase.NEGOTIATE,'field read + peer online')

        elif self.phase==Phase.NEGOTIATE:
            tx, ty = self.survey
            self._pp(tx, ty, self.cruise)
            pris = {self.did: self.priority, self.other: self._peer_priority()}
            self.assignment = greedy_assign(SURVEY_POS, self.flowers, pris)
            mine = self.assignment[self.did]
            # order nearest-first from survey position
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
echo "swarm_drone_controller.py written"

# ── 4. Passive monitor (display only, no decisions) ───────────
echo ""
echo "[4/6] Writing swarm_monitor.py..."

cat > $NODES_DIR/swarm_monitor.py << 'PYEOF'
#!/usr/bin/env python3
"""swarm_monitor.py - PASSIVE display. Subscribes to /swarm/peer and
shows status. Makes NO decisions (coordination is peer-to-peer)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time


class Monitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
        self.d = {0: {}, 1: {}}
        self.t0 = time.time()
        self.create_subscription(String, '/swarm/peer', self._cb, 10)
        self.create_timer(2.0, self._dash)
        self.get_logger().info('SWARM MONITOR (passive display - no decisions)')

    def _cb(self, m):
        try:
            x = json.loads(m.data); self.d[x['drone_id']] = x
        except Exception: pass

    def _dash(self):
        t = round(time.time()-self.t0, 0)
        L = ['', f'===== SWARM  T+{t:.0f}s =====']
        done = 0
        for i in (0, 1):
            x = self.d.get(i, {})
            ph = x.get('phase', '-'); cl = x.get('claimed', [])
            pos = (f'({x.get("x")},{x.get("y")},{x.get("z")})'
                   if x.get('x') is not None else 'unknown')
            arm = 'ARMED' if x.get('armed') else 'disarmed'
            if ph == 'DONE': done += 1
            L += [f'  Drone {i} [{arm}] {ph}',
                  f'     pos {pos}  claims {cl}']
        L += [f'  completed: {done}/2', '==========================']
        for s in L: self.get_logger().info(s)


def main(args=None):
    rclpy.init(args=args)
    n = Monitor()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF
echo "swarm_monitor.py written"

# ── 5. setup.py + build ───────────────────────────────────────
echo ""
echo "[5/6] Building package..."

cat > $ROS2_WS/src/precision_pollination/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup
package_name = 'precision_pollination'
setup(name=package_name, version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',
         ['resource/'+package_name]), ('share/'+package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True, maintainer='FYP',
    description='Swarm drone pollination - Stage 14 peer-to-peer',
    entry_points={'console_scripts': [
        'field_survey=precision_pollination.field_survey:main',
        'swarm_drone_controller=precision_pollination.swarm_drone_controller:main',
        'swarm_monitor=precision_pollination.swarm_monitor:main',
    ]})
SETUPEOF

cd $ROS2_WS
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null || true
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -4
mkdir -p $ROS2_WS/install/precision_pollination/lib/precision_pollination
for N in field_survey swarm_drone_controller swarm_monitor; do
cat > $ROS2_WS/install/precision_pollination/lib/precision_pollination/$N << EOF
#!/usr/bin/env python3
import sys
sys.path.insert(0, '$ROS2_WS/src/precision_pollination')
from precision_pollination.$N import main
main()
EOF
chmod +x $ROS2_WS/install/precision_pollination/lib/precision_pollination/$N
done
echo "built"

# ── 6. Launch + camera ────────────────────────────────────────
echo ""
echo "[6/6] Writing launch_stage14.sh + fix_camera.sh..."

cat > $FYP_DIR/fix_camera.sh << 'CAMEOF'
#!/bin/bash
# Fixed external camera centred behind the field; kill PX4 follow.
for i in 1 2 3 4 5; do
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
    --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""' 2>/dev/null || true
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 2000 \
    --req 'pose: {position: {x: 4, y: -7, z: 7}, orientation: {x: -0.1922, y: 0.1922, z: 0.6805, w: 0.6805}}' 2>/dev/null || true
  sleep 2
done
echo "Camera locked behind the field (4,-7,7). If it still tracks a"
echo "drone, drag once in the Gazebo window to detach."
CAMEOF
chmod +x $FYP_DIR/fix_camera.sh

cat > $FYP_DIR/launch_stage14.sh << 'LAUNCHEOF'
#!/bin/bash
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws

echo "STAGE 14 - Swarm Pollination (peer-to-peer, 6 flowers)"
python3 $FYP_DIR/generate_field.py

pkill -f px4 2>/dev/null||true; pkill -f gz 2>/dev/null||true
pkill -f MicroXRCEAgent 2>/dev/null||true; sleep 6

gnome-terminal --title="XRCE Bridge" -- bash -c "source ~/.bashrc; MicroXRCEAgent udp4 -p 8888; bash" &
sleep 6
gnome-terminal --title="PX4 Drone 0" -- bash -c "
  source ~/.bashrc; cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='2.0,0.0,0.1,0,0,0' \
  PX4_GZ_MODEL=x500 PX4_GZ_WORLD=demo_2flower \
  ./build/px4_sitl_default/bin/px4 -i 0; bash" &
sleep 14
gnome-terminal --title="PX4 Drone 1" -- bash -c "
  source ~/.bashrc; cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='6.0,0.0,0.1,0,0,0' \
  PX4_GZ_MODEL=x500 PX4_GZ_WORLD=demo_2flower \
  ./build/px4_sitl_default/bin/px4 -i 1; bash" &
sleep 8
for D in 0 1; do
gnome-terminal --title="Drone $D survey" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $ROS2_WS/src/precision_pollination/precision_pollination/field_survey.py \
    --ros-args -p drone_id:=$D; bash" &
sleep 1
gnome-terminal --title="Drone $D controller" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $ROS2_WS/src/precision_pollination/precision_pollination/swarm_drone_controller.py \
    --ros-args -p drone_id:=$D; bash" &
sleep 1
done
gnome-terminal --title="Swarm Monitor" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $ROS2_WS/src/precision_pollination/precision_pollination/swarm_monitor.py; bash" &

( sleep 18; bash $FYP_DIR/fix_camera.sh ) &

echo ""
echo "Launched. Both drones arm, lift together to 3m, survey the field,"
echo "negotiate peer-to-peer (3 flowers each, no repeats), pollinate at"
echo "2m, then return to spawn and land. Camera locks ~18s in."
LAUNCHEOF
chmod +x $FYP_DIR/launch_stage14.sh
echo "launch_stage14.sh + fix_camera.sh written"

echo ""
echo "=============================================================="
echo "  STAGE 14 SWARM READY"
echo "  - 6 random flowers, one field (re-rolled each launch)"
echo "  - No-dance arming, synchronized takeoff to 3m"
echo "  - Centre survey reads all 6 flowers"
echo "  - Peer-to-peer greedy 3/3 assignment (random tie-break)"
echo "  - Altitude-layered collision avoidance + yield rule"
echo "  - Pollinate at 2m (no crush), return to spawn, land"
echo "  - Fixed external camera behind the field"
echo ""
echo "  RUN:  bash ~/Desktop/FYP/launch_stage14.sh"
echo "=============================================================="
echo ""
