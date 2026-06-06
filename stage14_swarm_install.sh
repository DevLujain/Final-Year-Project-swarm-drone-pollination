#!/bin/bash
# ============================================================
# stage14_swarm_install.sh  (v2 — Visual Servoing, no PX4)
# ------------------------------------------------------------
# Installs the redesigned Stage 14 swarm pollination pipeline:
#
#   - 2 drones, controlled directly via Gazebo VelocityControl
#     plugin (no PX4 SITL, no XRCE-DDS).
#   - 6 sunflowers placed at random positions each launch.
#   - Synchronised takeoff on /swarm/start.
#   - Peer-to-peer greedy assignment (3 flowers each, no
#     duplicates, priority tie-break).
#   - Pollination at 2 m, then 3 m return cruise, then land.
#
# Fixes from the previous attempt:
#   * Every Twist field is published with float() cast - the
#     Vector3 'PyFloat_Check' assertion that crashed the
#     controllers is gone.
#   * VelocityControl runs in world frame — no body-frame
#     transform is done in the controller.
#   * 6 random flowers (not 2 hardcoded), regenerated per run.
#   * YOLO + mission_logger excluded from the launch
#     (NumPy 2.x / cv_bridge ABI issue out of scope here).
#
# What this script does:
#   1. Stops any running sim
#   2. Archives the previous Stage 14 files
#   3. Writes 5 ROS 2 node Python files + setup.py
#   4. Writes the field generator + launch_stage14.sh
#   5. Rebuilds the precision_pollination package
#
# RUN:  bash stage14_swarm_install.sh
# THEN: bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
PKG_DIR=$ROS2_WS/src/precision_pollination
NODES_DIR=$PKG_DIR/precision_pollination
WORLDS_DIR=~/PX4-Autopilot/Tools/simulation/gz/worlds

echo ""
echo "============================================================"
echo "  Stage 14 Swarm Pollination Installer (v2)"
echo "============================================================"
echo ""

if [ ! -d "$NODES_DIR" ]; then
    echo "ERROR: $NODES_DIR not found."
    exit 1
fi
mkdir -p "$WORLDS_DIR"

# Step 1: Stop everything
echo "[1/10] Stopping any running simulation..."
pkill -9 -f 'gz sim'         2>/dev/null || true
pkill -9 -f gz-sim           2>/dev/null || true
pkill -9 -f gzserver         2>/dev/null || true
pkill -9 -f ruby             2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f px4              2>/dev/null || true
pkill -9 -f MicroXRCEAgent   2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 3
echo "        Done."

# Step 2: Archive previous Stage 14 files
echo "[2/10] Archiving previous Stage 14 files..."
TS=$(date +%Y%m%d_%H%M%S)
BACKUP=$FYP_DIR/_stage14_v1_backup_$TS
mkdir -p "$BACKUP"
for F in visual_servoing_controller.py swarm_coordinator.py \
         yolo_field_detector.py mission_logger.py \
         field_survey.py swarm_drone_controller.py swarm_monitor.py; do
    if [ -f "$NODES_DIR/$F" ]; then
        mv "$NODES_DIR/$F" "$BACKUP/"
    fi
done
if [ -f "$FYP_DIR/launch_stage14.sh" ]; then
    cp "$FYP_DIR/launch_stage14.sh" "$BACKUP/launch_stage14.sh.bak"
fi
if [ -f "$FYP_DIR/generate_field.py" ]; then
    cp "$FYP_DIR/generate_field.py" "$BACKUP/generate_field.py.bak"
fi
echo "        Archive: $BACKUP"


# Step 3: Write generate_field.py
echo "[3/10] Writing generate_field.py..."
cat > $FYP_DIR/generate_field.py << 'GENEOF'
#!/usr/bin/env python3
"""
generate_field.py — Stage 14

Generates a complete Gazebo world (demo_stage14.sdf) with:
  - 2 drones (drone_0, drone_1) using VelocityControl + PosePublisher
  - 6 sunflowers at RANDOM positions inside the field rectangle,
    with a minimum 1.4 m spacing (re-rolled every launch)
  - Ground, base pads, field tile

Also writes /tmp/stage14_flowers.json so the field_survey nodes
have the same ground-truth flower list.

Run before launching Gazebo.
"""
from __future__ import annotations

import os
import json
import math
import random


# ── Field geometry ─────────────────────────────────────────────
# Flowers live inside this rectangle (metres):
FIELD_XMIN, FIELD_XMAX = 2.5, 10.5
FIELD_YMIN, FIELD_YMAX = 3.0, 7.0
MIN_SPACING = 1.4
N_FLOWERS = 6

# Drone spawn positions (matches swarm_drone_controller.SPAWN)
SPAWN = {0: (1.5, 1.0), 1: (11.5, 1.0)}

# Output paths
JSONF = '/tmp/stage14_flowers.json'
SDF_OUT = os.path.expanduser(
    '~/PX4-Autopilot/Tools/simulation/gz/worlds/demo_stage14.sdf'
)


# ── Random flower placement ────────────────────────────────────
def place_flowers(n, xmin, xmax, ymin, ymax, min_spacing, max_tries=5000):
    pts = []
    tries = 0
    while len(pts) < n and tries < max_tries:
        tries += 1
        x = round(random.uniform(xmin, xmax), 2)
        y = round(random.uniform(ymin, ymax), 2)
        if all(math.hypot(x - px, y - py) >= min_spacing for px, py, _ in pts):
            pts.append((x, y, len(pts) + 1))
    if len(pts) < n:
        raise RuntimeError(f"Could only place {len(pts)} flowers (wanted {n})")
    return pts


# ── SDF fragments ──────────────────────────────────────────────
def drone_block(name, x, y, r, g, b):
    return f"""
    <model name="{name}">
      <pose>{x} {y} 0.10 0 0 0</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.04</izz></inertia>
        </inertial>
        <visual name="body">
          <geometry><box><size>0.45 0.45 0.10</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
        <visual name="arm_x">
          <pose>0 0 0.06 0 0 0</pose>
          <geometry><box><size>0.90 0.06 0.03</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_y">
          <pose>0 0 0.06 0 0 1.5708</pose>
          <geometry><box><size>0.90 0.06 0.03</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="prop_fl">
          <pose>0.32 0.32 0.085 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.012</length></cylinder></geometry>
          <material><ambient>0.4 0.4 0.4 0.7</ambient><diffuse>0.4 0.4 0.4 0.7</diffuse></material>
        </visual>
        <visual name="prop_fr">
          <pose>0.32 -0.32 0.085 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.012</length></cylinder></geometry>
          <material><ambient>0.4 0.4 0.4 0.7</ambient><diffuse>0.4 0.4 0.4 0.7</diffuse></material>
        </visual>
        <visual name="prop_bl">
          <pose>-0.32 0.32 0.085 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.012</length></cylinder></geometry>
          <material><ambient>0.4 0.4 0.4 0.7</ambient><diffuse>0.4 0.4 0.4 0.7</diffuse></material>
        </visual>
        <visual name="prop_br">
          <pose>-0.32 -0.32 0.085 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.012</length></cylinder></geometry>
          <material><ambient>0.4 0.4 0.4 0.7</ambient><diffuse>0.4 0.4 0.4 0.7</diffuse></material>
        </visual>
      </link>

      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
        <topic>/model/{name}/cmd_vel</topic>
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <update_frequency>30</update_frequency>
      </plugin>
    </model>"""


def flower_block(name, x, y):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="stem">
        <pose>0 0 0.7 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.05</radius><length>1.4</length></cylinder></geometry>
          <material><ambient>0.1 0.55 0.1 1</ambient><diffuse>0.1 0.55 0.1 1</diffuse></material>
        </visual>
      </link>
      <link name="petals">
        <pose>0 0 1.46 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.34</radius><length>0.03</length></cylinder></geometry>
          <material><ambient>0.95 0.82 0.05 1</ambient><diffuse>0.95 0.82 0.05 1</diffuse></material>
        </visual>
      </link>
      <link name="head">
        <pose>0 0 1.49 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.20</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.42 0.25 0.05 1</ambient><diffuse>0.42 0.25 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>"""


WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="stage14_world">

    <physics name="default_physics" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>6.5 -5 6 0 0.55 1.5708</pose>
      </camera>
    </gui>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0.3 0.3 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material>
            <ambient>0.3 0.5 0.3 1</ambient>
            <diffuse>0.3 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="field_tile">
      <static>true</static>
      <pose>6.5 5.0 0.01 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><box><size>10 5 0.02</size></box></geometry>
          <material>
            <ambient>0.15 0.35 0.18 1</ambient>
            <diffuse>0.15 0.35 0.18 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="pad_0">
      <static>true</static>
      <pose>1.5 1.0 0.02 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><cylinder><radius>0.6</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.2 0.4 0.9 1</ambient><diffuse>0.2 0.4 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="pad_1">
      <static>true</static>
      <pose>11.5 1.0 0.02 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><cylinder><radius>0.6</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.9 0.5 0.1 1</ambient><diffuse>0.9 0.5 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>

{drones}
{flowers}
  </world>
</sdf>
"""


def main():
    # Re-seed every run so each launch gets a different field
    random.seed()
    flowers = place_flowers(N_FLOWERS, FIELD_XMIN, FIELD_XMAX,
                            FIELD_YMIN, FIELD_YMAX, MIN_SPACING)

    # JSON for the survey nodes
    json_data = {
        'flowers': [
            {'id': fid, 'x': x, 'y': y, 'z': 1.5}
            for x, y, fid in flowers
        ]
    }
    with open(JSONF, 'w') as f:
        json.dump(json_data, f, indent=2)

    # Build SDF
    drone_sdfs = (
        drone_block('drone_0', SPAWN[0][0], SPAWN[0][1], 0.2, 0.4, 0.9)
        + drone_block('drone_1', SPAWN[1][0], SPAWN[1][1], 0.9, 0.5, 0.1)
    )
    flower_sdfs = ''.join(
        flower_block(f'sunflower_{fid}', x, y)
        for x, y, fid in flowers
    )
    sdf = WORLD_TEMPLATE.format(drones=drone_sdfs, flowers=flower_sdfs)

    os.makedirs(os.path.dirname(SDF_OUT), exist_ok=True)
    with open(SDF_OUT, 'w') as f:
        f.write(sdf)

    print(f"Wrote {SDF_OUT}")
    print(f"Wrote {JSONF}")
    print("Field has 6 random flowers:")
    for x, y, fid in flowers:
        print(f"  Flower {fid}: ({x}, {y})")


if __name__ == '__main__':
    main()
GENEOF

# Step 4: Write swarm_drone_controller.py
echo "[4/10] Writing swarm_drone_controller.py..."
cat > $NODES_DIR/swarm_drone_controller.py << 'CTRLEOF'
#!/usr/bin/env python3
"""
swarm_drone_controller.py — Stage 14 (Visual Servoing, no PX4)

One instance per drone. Two-drone peer-to-peer swarm pollination
using Gazebo's VelocityControl plugin (NOT PX4).

State machine:
    INIT  -> READY  -> TAKEOFF  -> SURVEY_GOTO  -> SURVEY  -> NEGOTIATE
                                                                    |
                                                                    v
    DONE <- LAND  <- RETURN  <- (P_GOTO -> P_DESCEND -> P_HOVER -> P_ASCEND)+

Coordination:
    Each drone has a random priority (in [0, 1)) set at startup.
    Both drones broadcast their position + priority + phase + claims
    on /swarm/peer. Once a drone has its field map and the peer's
    priority, it runs the SAME greedy closest-first assignment locally
    -> both reach identical assignments without a central authority.

Frame:
    The Gazebo VelocityControl plugin interprets cmd_vel in the
    WORLD frame. We never do body-frame transforms here.
    Yaw is left at 0 throughout the mission (we only send angular.z=0).
"""
from __future__ import annotations

import json
import math
import random
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool


# ── Geometry (must match generate_field.py and the SDF) ────────
SPAWN      = {0: (1.5, 1.0), 1: (11.5, 1.0)}
SURVEY_POS = {0: (4.5, 4.0), 1: (8.5, 4.0)}
CRUISE_Z      = 3.0     # cruise altitude (m)
POLLINATE_Z   = 2.0     # pollination hover altitude (m)
GROUND_Z      = 0.10    # landed altitude (m)

# ── Controller params ──────────────────────────────────────────
WP_TOL_XY   = 0.30      # horizontal arrival tolerance (m)
WP_TOL_Z    = 0.20      # vertical arrival tolerance (m)
HOVER_TICKS = 30        # 1.5 s at 20 Hz of pollination hover
CAP         = 3         # flowers per drone

# PID gains (P + simple D)
KP_XY = 1.0
KP_Z  = 1.0
KD_XY = 0.15
KD_Z  = 0.10
VMAX_XY = 1.2           # m/s
VMAX_Z  = 0.8           # m/s

CONTROL_HZ   = 20.0
BROADCAST_HZ = 10.0


def greedy_assign(drone_positions, flowers, priorities):
    """
    Greedy closest-first assignment.

    At each step, find the (drone, flower) pair with minimum distance.
    Assign that flower to that drone. Ties (within 1 mm) broken by
    priority: higher priority drone wins; then lower flower id.

    Capped at CAP flowers per drone. With 2 drones and 6 flowers
    this gives exactly 3 each (no duplicates).
    """
    assigned = {0: [], 1: []}
    remaining = list(flowers)
    while remaining:
        cands = []
        for d in (0, 1):
            if len(assigned[d]) >= CAP:
                continue
            px, py = drone_positions[d]
            for fl in remaining:
                dist = math.hypot(px - fl['x'], py - fl['y'])
                cands.append((dist, d, fl))
        if not cands:
            break
        min_d = min(c[0] for c in cands)
        tied = [c for c in cands if c[0] - min_d < 1e-3]
        # Higher priority wins on tie; then lowest flower id for determinism
        tied.sort(key=lambda c: (-priorities[c[1]], c[2]['id']))
        _, d, fl = tied[0]
        assigned[d].append(fl)
        remaining.remove(fl)
    return assigned


class Phase(Enum):
    INIT        = "INIT"
    READY       = "READY"
    TAKEOFF     = "TAKEOFF"
    SURVEY_GOTO = "SURVEY_GOTO"
    SURVEY      = "SURVEY"
    NEGOTIATE   = "NEGOTIATE"
    P_GOTO      = "POLLINATE_GOTO"
    P_DESCEND   = "POLLINATE_DESCEND"
    P_HOVER     = "POLLINATE_HOVER"
    P_ASCEND    = "POLLINATE_ASCEND"
    RETURN      = "RETURN"
    LAND        = "LAND"
    DONE        = "DONE"


class SwarmDrone(Node):
    def __init__(self):
        super().__init__('swarm_drone_controller')

        # Parameters
        self.declare_parameter('drone_id', 0)
        self.did = int(self.get_parameter('drone_id').value)
        if self.did not in (0, 1):
            raise ValueError(f"drone_id must be 0 or 1, got {self.did}")
        self.other = 1 - self.did
        self.spawn = SPAWN[self.did]
        self.survey_pos = SURVEY_POS[self.did]
        self.priority = random.random()

        # Mission state
        self.pose = None                # (x, y, z, yaw)
        self.prev_err = (0.0, 0.0, 0.0)
        self.prev_t = None
        self.phase = Phase.INIT
        self.armed = False              # True once /swarm/start received
        self.flowers = None             # list of {id, x, y, z}
        self.peer = {}                  # latest /swarm/peer broadcast
        self.assignment = None
        self.my_flowers = []
        self.fi = 0                     # index into self.my_flowers
        self.hov_t = 0                  # tick counter while hovering
        self.done_logged = False
        self.t0 = time.time()

        # ROS interfaces
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_cmd = self.create_publisher(
            Twist, f'/model/drone_{self.did}/cmd_vel', 10)
        self.pub_state = self.create_publisher(
            String, f'/pollination/drone_{self.did}/state', 10)
        self.pub_event = self.create_publisher(
            String, f'/pollination/drone_{self.did}/event', 10)
        self.pub_pose_relay = self.create_publisher(
            PoseStamped, f'/pollination/drone_{self.did}/pose', 10)
        self.peer_pub = self.create_publisher(
            String, '/swarm/peer', 10)

        self.create_subscription(
            PoseStamped, f'/model/drone_{self.did}/pose',
            self._on_pose, sensor_qos)
        self.create_subscription(
            Bool, '/swarm/start', self._on_start, 10)
        self.create_subscription(
            String, f'/drone_{self.did}/field_map',
            self._on_field_map, 10)
        self.create_subscription(
            String, '/swarm/peer', self._on_peer, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self.create_timer(1.0 / BROADCAST_HZ, self._broadcast)

        self.get_logger().info(
            f"[D{self.did}] online. spawn={self.spawn} "
            f"survey_pos={self.survey_pos} priority={self.priority:.3f}"
        )

    # ── Subscriptions ──────────────────────────────────────────
    def _on_pose(self, msg):
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        first = self.pose is None
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(yaw),
        )
        if first:
            self.get_logger().info(
                f"[D{self.did}] pose lock at "
                f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:.2f})"
            )
        # Relay for logger/monitor
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = f'drone_{self.did}'
        out.pose = msg.pose
        self.pub_pose_relay.publish(out)

    def _on_start(self, msg):
        if msg.data and not self.armed:
            self.armed = True
            self.get_logger().info(f"[D{self.did}] /swarm/start received - GO")

    def _on_field_map(self, msg):
        if self.flowers is None:
            try:
                d = json.loads(msg.data)
                self.flowers = d['flowers']
                self.get_logger().info(
                    f"[D{self.did}] field map received - "
                    f"{len(self.flowers)} flowers"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"[D{self.did}] field map parse error: {e}"
                )

    def _on_peer(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('drone_id') == self.other:
                self.peer = d
        except Exception:
            pass

    # ── Helpers ────────────────────────────────────────────────
    def _go(self, new_phase, reason=""):
        text = f"[D{self.did}] {self.phase.value} -> {new_phase.value}"
        if reason:
            text += f"  ({reason})"
        self.get_logger().info(text)
        ev = String()
        ev.data = text
        self.pub_event.publish(ev)
        self.phase = new_phase
        # Reset PID derivative tracker on phase change
        self.prev_t = None
        self.prev_err = (0.0, 0.0, 0.0)

    def _broadcast(self):
        if self.pose is None:
            x = y = z = None
        else:
            x = round(self.pose[0], 2)
            y = round(self.pose[1], 2)
            z = round(self.pose[2], 2)
        m = String()
        m.data = json.dumps({
            'drone_id': self.did,
            'x': x, 'y': y, 'z': z,
            'priority': self.priority,
            'phase': self.phase.value,
            'claimed': [int(f['id']) for f in self.my_flowers],
        })
        self.peer_pub.publish(m)

    def _publish_state(self):
        m = String()
        m.data = self.phase.value
        self.pub_state.publish(m)

    def _peer_priority(self):
        return self.peer.get('priority')

    def _peer_phase(self):
        return self.peer.get('phase')

    # ── Control output ─────────────────────────────────────────
    def _send_velocity(self, vx, vy, vz, wz):
        """Publish a Twist with EVERY field cast to float."""
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.linear.z  = float(vz)
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(wz)
        self.pub_cmd.publish(t)

    def _stop(self):
        self._send_velocity(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def _clamp(v, vmax):
        return max(-vmax, min(vmax, v))

    def _goto(self, tx, ty, tz):
        """
        PD step toward world point (tx, ty, tz). Yaw is not controlled
        (angular.z = 0). Returns True when within position tolerance.
        """
        if self.pose is None:
            return False
        x, y, z, _yaw = self.pose
        ex = float(tx) - x
        ey = float(ty) - y
        ez = float(tz) - z

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            dex = dey = dez = 0.0
        else:
            dt = max(now - self.prev_t, 1e-3)
            dex = (ex - self.prev_err[0]) / dt
            dey = (ey - self.prev_err[1]) / dt
            dez = (ez - self.prev_err[2]) / dt
        self.prev_t = now
        self.prev_err = (ex, ey, ez)

        vx = self._clamp(KP_XY * ex + KD_XY * dex, VMAX_XY)
        vy = self._clamp(KP_XY * ey + KD_XY * dey, VMAX_XY)
        vz = self._clamp(KP_Z  * ez + KD_Z  * dez, VMAX_Z)

        # Pure world-frame velocity command — no body transform
        self._send_velocity(vx, vy, vz, 0.0)

        d_xy = math.hypot(ex, ey)
        d_z = abs(ez)
        return d_xy < WP_TOL_XY and d_z < WP_TOL_Z

    # ── State machine ──────────────────────────────────────────
    def _tick(self):
        self._publish_state()

        if self.pose is None:
            return

        if self.phase == Phase.INIT:
            self._stop()
            self._go(Phase.READY, "pose lock acquired")
            return

        if self.phase == Phase.READY:
            self._stop()
            if self.armed:
                self._go(Phase.TAKEOFF, "GO signal received")
            return

        if self.phase == Phase.TAKEOFF:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, CRUISE_Z)
            if arrived:
                self._go(Phase.SURVEY_GOTO, f"at cruise alt {CRUISE_Z} m")
            return

        if self.phase == Phase.SURVEY_GOTO:
            tx, ty = self.survey_pos
            arrived = self._goto(tx, ty, CRUISE_Z)
            if arrived:
                self._go(Phase.SURVEY, "at survey waypoint")
            return

        if self.phase == Phase.SURVEY:
            # Hold position and wait for own field map + peer priority
            tx, ty = self.survey_pos
            self._goto(tx, ty, CRUISE_Z)
            if self.flowers is not None and self._peer_priority() is not None:
                self._go(Phase.NEGOTIATE, "field+peer ready")
            return

        if self.phase == Phase.NEGOTIATE:
            tx, ty = self.survey_pos
            self._goto(tx, ty, CRUISE_Z)

            priorities = {
                self.did: self.priority,
                self.other: float(self._peer_priority()),
            }
            self.assignment = greedy_assign(
                SURVEY_POS, self.flowers, priorities
            )
            mine = self.assignment[self.did]
            # Sort nearest-first from my own survey waypoint
            mine.sort(key=lambda fl: math.hypot(
                self.survey_pos[0] - fl['x'],
                self.survey_pos[1] - fl['y']
            ))
            self.my_flowers = mine
            self.fi = 0

            ids_mine  = [int(f['id']) for f in mine]
            ids_other = [int(f['id']) for f in self.assignment[self.other]]
            self.get_logger().info(
                f"\n[D{self.did}] NEGOTIATION COMPLETE\n"
                f"   I claim flowers {ids_mine}\n"
                f"   peer takes      {ids_other}\n"
                f"   (greedy closest-first; ties broken by priority)"
            )
            self._go(Phase.P_GOTO, f"start pollinating my {len(mine)} flowers")
            return

        if self.phase == Phase.P_GOTO:
            if self.fi >= len(self.my_flowers):
                self._go(Phase.RETURN, "all my flowers pollinated")
                return
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], CRUISE_Z)
            if arrived:
                self._go(Phase.P_DESCEND, f"over flower {fl['id']}")
            return

        if self.phase == Phase.P_DESCEND:
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], POLLINATE_Z)
            if arrived:
                self.hov_t = 0
                self._go(Phase.P_HOVER, f"at {POLLINATE_Z} m over flower {fl['id']}")
            return

        if self.phase == Phase.P_HOVER:
            fl = self.my_flowers[self.fi]
            self._goto(fl['x'], fl['y'], POLLINATE_Z)
            self.hov_t += 1
            if self.hov_t == 1:
                ev = String()
                ev.data = (f"[D{self.did}] *** POLLINATING flower {fl['id']} "
                           f"at ({fl['x']:.2f}, {fl['y']:.2f}) ***")
                self.get_logger().info(ev.data)
                self.pub_event.publish(ev)
            if self.hov_t >= HOVER_TICKS:
                self._go(Phase.P_ASCEND, f"flower {fl['id']} pollinated")
            return

        if self.phase == Phase.P_ASCEND:
            fl = self.my_flowers[self.fi]
            arrived = self._goto(fl['x'], fl['y'], CRUISE_Z)
            if arrived:
                self.fi += 1
                self._go(Phase.P_GOTO,
                         f"next ({self.fi}/{len(self.my_flowers)})")
            return

        if self.phase == Phase.RETURN:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, CRUISE_Z)
            if arrived:
                self._go(Phase.LAND, "over home pad")
            return

        if self.phase == Phase.LAND:
            sx, sy = self.spawn
            arrived = self._goto(sx, sy, GROUND_Z)
            if arrived:
                self._stop()
                self._go(Phase.DONE, "landed safely")
            return

        if self.phase == Phase.DONE:
            self._stop()
            if not self.done_logged:
                self.get_logger().info(
                    f"[D{self.did}] MISSION COMPLETE - pollinated "
                    f"{len(self.my_flowers)} flowers, landed at {self.spawn}"
                )
                self.done_logged = True
            return


def main(args=None):
    rclpy.init(args=args)
    node = SwarmDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure the drone stops on exit (all explicit floats)
        try:
            stop = Twist()
            stop.linear.x = 0.0
            stop.linear.y = 0.0
            stop.linear.z = 0.0
            stop.angular.x = 0.0
            stop.angular.y = 0.0
            stop.angular.z = 0.0
            node.pub_cmd.publish(stop)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
CTRLEOF

# Step 5: Write field_survey.py
echo "[5/10] Writing field_survey.py..."
cat > $NODES_DIR/field_survey.py << 'SURVEYEOF'
#!/usr/bin/env python3
"""
field_survey.py — Stage 14

Per-drone "aerial scan" simulation. Reads the ground-truth flower
positions from /tmp/stage14_flowers.json (written by generate_field.py)
and publishes the full list on /drone_N/field_map once the drone is
at survey altitude (>= 2.5 m).

In a real deployment this would be YOLO on a downward camera, but
the SDF mock keeps the demo deterministic.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


JSONF = '/tmp/stage14_flowers.json'
SURVEY_MIN_ALT = 2.5
MODEL_MAP50 = 0.855


class FieldSurvey(Node):
    def __init__(self):
        super().__init__('field_survey')

        self.declare_parameter('drone_id', 0)
        self.did = int(self.get_parameter('drone_id').value)

        try:
            with open(JSONF) as f:
                self.flowers = json.load(f)['flowers']
            self.get_logger().info(
                f"[D{self.did}] survey sensor online "
                f"(YOLOv8s-OBB mock, mAP@0.5={MODEL_MAP50}). "
                f"Field has {len(self.flowers)} flowers. "
                f"Will publish when altitude >= {SURVEY_MIN_ALT} m."
            )
        except Exception as e:
            self.get_logger().error(
                f"[D{self.did}] failed to load {JSONF}: {e}"
            )
            self.flowers = []

        self.pose = None
        self.published = False

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            PoseStamped, f'/model/drone_{self.did}/pose',
            self._on_pose, sensor_qos
        )
        self.pub = self.create_publisher(
            String, f'/drone_{self.did}/field_map', 10
        )
        # Publish at 2 Hz; the controller only consumes the first one.
        self.create_timer(0.5, self._loop)

    def _on_pose(self, msg):
        self.pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _loop(self):
        if self.pose is None or not self.flowers:
            return
        alt = self.pose[2]
        if alt < SURVEY_MIN_ALT:
            return
        msg = String()
        msg.data = json.dumps({'flowers': self.flowers})
        self.pub.publish(msg)
        if not self.published:
            self.published = True
            self.get_logger().info(
                f"[D{self.did}] AERIAL SURVEY @ alt {alt:.1f} m "
                f"- detected {len(self.flowers)} flowers:"
            )
            for fl in self.flowers:
                self.get_logger().info(
                    f"   Flower {fl['id']} at ({fl['x']:.2f}, {fl['y']:.2f})"
                )


def main(args=None):
    rclpy.init(args=args)
    n = FieldSurvey()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
SURVEYEOF

# Step 6: Write swarm_coordinator.py
echo "[6/10] Writing swarm_coordinator.py..."
cat > $NODES_DIR/swarm_coordinator.py << 'COORDEOF'
#!/usr/bin/env python3
"""
swarm_coordinator.py — Stage 14

Fires the global GO signal on /swarm/start after a warmup period
(so Gazebo + bridge + per-drone nodes are all ready). Then monitors
both drones' phases via /swarm/peer and announces mission complete
when both reach DONE.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


WARMUP_SEC = 5.0    # seconds to wait before firing /swarm/start


class Coordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        self.started = False
        self.completed = False
        self.phases = {0: 'UNKNOWN', 1: 'UNKNOWN'}

        self.pub_start = self.create_publisher(Bool,   '/swarm/start',            10)
        self.pub_event = self.create_publisher(String, '/pollination/swarm/event', 10)

        self.create_subscription(String, '/swarm/peer', self._on_peer, 10)

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"swarm coordinator online - "
            f"will fire /swarm/start in {WARMUP_SEC:.0f} s"
        )

    def _on_peer(self, msg):
        try:
            d = json.loads(msg.data)
            did = d.get('drone_id')
            if did in (0, 1):
                self.phases[did] = d.get('phase', 'UNKNOWN')
        except Exception:
            pass

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if not self.started and now - self.t0 >= WARMUP_SEC:
            # Publish a few times so we don't lose it to subscriber-not-ready
            m = Bool()
            m.data = True
            for _ in range(5):
                self.pub_start.publish(m)
            self.started = True
            ev = String()
            ev.data = "*** SWARM GO - both drones launching together ***"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)

        if (self.started and not self.completed
                and self.phases[0] == 'DONE'
                and self.phases[1] == 'DONE'):
            self.completed = True
            ev = String()
            ev.data = "*** SWARM MISSION COMPLETE - both drones landed safely ***"
            self.pub_event.publish(ev)
            self.get_logger().info(ev.data)


def main(args=None):
    rclpy.init(args=args)
    n = Coordinator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
COORDEOF

# Step 7: Write swarm_monitor.py
echo "[7/10] Writing swarm_monitor.py..."
cat > $NODES_DIR/swarm_monitor.py << 'MONITOREOF'
#!/usr/bin/env python3
"""
swarm_monitor.py — Stage 14

Passive dashboard. Subscribes to /swarm/peer and prints a status
table for both drones every 2 seconds. Makes no control decisions.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
        self.d = {0: {}, 1: {}}
        self.t0 = time.time()
        self.create_subscription(String, '/swarm/peer', self._cb, 10)
        self.create_timer(2.0, self._dash)
        self.get_logger().info('SWARM MONITOR (passive display)')

    def _cb(self, m):
        try:
            x = json.loads(m.data)
            did = x.get('drone_id')
            if did in (0, 1):
                self.d[did] = x
        except Exception:
            pass

    def _dash(self):
        t = int(time.time() - self.t0)
        lines = ['', f'====== SWARM STATUS  T+{t}s ======']
        done = 0
        for i in (0, 1):
            x = self.d.get(i, {})
            ph = x.get('phase', '-')
            claims = x.get('claimed', [])
            if x.get('x') is not None:
                pos = f"({x['x']:5.2f}, {x['y']:5.2f}, {x['z']:5.2f})"
            else:
                pos = "      pose unknown      "
            if ph == 'DONE':
                done += 1
            lines.append(f"  Drone {i}: {ph:18s} pos {pos} claims {claims}")
        lines.append(f"  completed: {done}/2")
        lines.append('==================================')
        for l in lines:
            self.get_logger().info(l)


def main(args=None):
    rclpy.init(args=args)
    n = Monitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
MONITOREOF

# Step 8: Write setup.py
echo "[8/10] Writing setup.py..."
cat > $PKG_DIR/setup.py << 'SETUPEOF'
from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='5.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP',
    maintainer_email='fyp@example.com',
    description='Swarm drone pollination - Stage 14 (Gazebo direct control)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_drone_controller=precision_pollination.swarm_drone_controller:main',
            'field_survey=precision_pollination.field_survey:main',
            'swarm_coordinator=precision_pollination.swarm_coordinator:main',
            'swarm_monitor=precision_pollination.swarm_monitor:main',
        ],
    },
)
SETUPEOF

# Step 9: Write launch_stage14.sh
echo "[9/10] Writing launch_stage14.sh..."
cat > $FYP_DIR/launch_stage14.sh << 'LAUNCHEOF'
#!/bin/bash
# launch_stage14.sh — Stage 14 (no PX4, Gazebo VelocityControl)
#
# Sequence:
#   1. Generate fresh random 6-flower field (writes SDF + JSON)
#   2. Kill any leftover processes
#   3. Launch Gazebo, GZ<->ROS2 bridge, then 6 ROS 2 nodes:
#        - field_survey  per drone
#        - swarm_drone_controller per drone
#        - swarm_coordinator
#        - swarm_monitor

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
WORLD=~/PX4-Autopilot/Tools/simulation/gz/worlds/demo_stage14.sdf

echo ""
echo "============================================================"
echo "  STAGE 14 - Swarm Pollination Demo"
echo "  2 drones | 6 random flowers | peer-to-peer assignment"
echo "============================================================"
echo ""

# 1. Generate a fresh field
python3 "$FYP_DIR/generate_field.py"

# 2. Kill any leftover processes
pkill -9 -f 'gz sim'         2>/dev/null || true
pkill -9 -f gz-sim           2>/dev/null || true
pkill -9 -f gzserver         2>/dev/null || true
pkill -9 -f ruby             2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f px4              2>/dev/null || true
pkill -9 -f MicroXRCEAgent   2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 3

# 3a. Gazebo
gnome-terminal --title="Gazebo" -- bash -c "
  source ~/.bashrc
  echo '>>> Starting Gazebo with $WORLD'
  gz sim -r '$WORLD'
  bash" &
sleep 10

# 3b. ROS 2 <-> Gazebo bridge
gnome-terminal --title="GZ Bridge" -- bash -c "
  source ~/.bashrc
  echo '>>> Bridging ROS 2 <-> Gazebo'
  ros2 run ros_gz_bridge parameter_bridge \\
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \\
    /model/drone_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_0/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose \\
    /model/drone_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \\
    /model/drone_1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose
  bash" &
sleep 5

# 3c. Per-drone survey + controller
for D in 0 1; do
  gnome-terminal --title="D${D} survey" -- bash -c "
    source ~/.bashrc; source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination field_survey --ros-args -p drone_id:=$D
    bash" &
  sleep 1
  gnome-terminal --title="D${D} controller" -- bash -c "
    source ~/.bashrc; source $ROS2_WS/install/setup.bash
    ros2 run precision_pollination swarm_drone_controller --ros-args -p drone_id:=$D
    bash" &
  sleep 1
done

# 3d. Coordinator
gnome-terminal --title="Coordinator" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_coordinator
  bash" &
sleep 1

# 3e. Monitor
gnome-terminal --title="Monitor" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  ros2 run precision_pollination swarm_monitor
  bash" &

echo ""
echo "============================================================"
echo "  Launched. 8 terminal tabs open:"
echo "    Gazebo | GZ Bridge | D0 survey | D0 controller |"
echo "    D1 survey | D1 controller | Coordinator | Monitor"
echo ""
echo "  Expected timeline:"
echo "    t=0..10s   Gazebo loads world with new random field"
echo "    t~15s      Coordinator fires GO -> both drones TAKEOFF"
echo "    t~20s      Both at 3 m cruise -> fly to survey waypoints"
echo "    t~25s      Field survey complete, negotiation runs"
echo "    t~30..80s  Each drone pollinates its 3 flowers at 2 m"
echo "               (GOTO 3m -> DESCEND 2m -> HOVER 1.5s -> ASCEND 3m)"
echo "    t~85s      RETURN to spawn, LAND"
echo "    t~95s      SWARM MISSION COMPLETE"
echo "============================================================"
LAUNCHEOF

# Step 9: Rebuild the workspace
echo "[9/10] Rebuilding ROS 2 workspace..."
source /opt/ros/jazzy/setup.bash
cd "$ROS2_WS"
rm -rf build/precision_pollination install/precision_pollination 2>/dev/null
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -6

# Step 10: Verify
echo "[10/10] Verifying installed nodes..."
source "$ROS2_WS/install/setup.bash" 2>/dev/null
ALL_OK=1
for N in swarm_drone_controller field_survey swarm_coordinator swarm_monitor; do
    if [ -f "$ROS2_WS/install/precision_pollination/lib/precision_pollination/$N" ]; then
        echo "  OK    $N"
    else
        echo "  MISS  $N"
        ALL_OK=0
    fi
done

chmod +x "$FYP_DIR/launch_stage14.sh"
chmod +x "$FYP_DIR/generate_field.py"

echo ""
echo "============================================================"
if [ "$ALL_OK" = "1" ]; then
    echo "  Install complete."
    echo "============================================================"
    echo ""
    echo "Next step:"
    echo "  bash $FYP_DIR/launch_stage14.sh"
    echo ""
    echo "What happens:"
    echo "  * generate_field.py writes a fresh 6-flower SDF + JSON"
    echo "  * Gazebo opens; bridge connects ROS 2 <-> Gazebo"
    echo "  * Both drones get /swarm/start ~5 s later"
    echo "  * TAKEOFF together to 3 m, fly to survey waypoints"
    echo "  * Each surveys the field, broadcasts priority"
    echo "  * Greedy assignment (3 flowers each, no duplicates)"
    echo "  * Pollinate at 2 m, return to 3 m, fly home, land"
    echo ""
    echo "Old files (PX4-era + previous v1 attempt) are in:"
    echo "  $BACKUP"
else
    echo "  Install FAILED - some nodes did not build."
    echo "============================================================"
    exit 1
fi
