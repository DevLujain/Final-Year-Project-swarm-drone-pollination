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
