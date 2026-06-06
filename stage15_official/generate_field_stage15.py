#!/usr/bin/env python3
"""
generate_field_stage15.py  —  Stage 15 (clean rewrite)
======================================================
Generates the Gazebo world (/tmp/stage15_world.sdf) and the flower
registry (/tmp/stage15_flowers.json).

DESIGN DECISION
---------------
The SDF template here is a near-clone of demo_stage14.sdf — which has
been proven to load reliably on this machine (Ubuntu 24.04 / Wayland /
NVIDIA / Gazebo Harmonic 8.11.0). The only structural differences from
Stage 14 are:

  - 2 drones become N drones (configurable)
  - 2 flowers become N flowers (configurable)
  - Drone pads sit on a strip just south of the field at y = -1.0
  - Each flower has a `bloom_start` day attribute consumed by ROS nodes

What is deliberately NOT changed from Stage 14:

  * SDF version 1.9 (NOT 1.10 — Harmonic prefers 1.9)
  * <physics type="ignored"> (Gazebo falls back to default DART)
  * Five system plugins (Physics, Sensors w/ ogre2, SceneBroadcaster,
    UserCommands, Contact)
  * Inline <model name="ground_plane"> with a flat 50 m plane
    (NEVER an <include><uri>https://fuel.gazebosim.org/...</uri></include>
    — that HTTPS fetch hangs gz sim at startup)
  * VelocityControl plugin with NO <topic> sub-element
    (default topic /model/<name>/cmd_vel is already what we want)
  * Each drone is a kinematic body with <gravity>false</gravity>
    on its link and <self_collide>false</self_collide> on the model
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
from dataclasses import dataclass
from typing import List, Tuple

import yaml


# ────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────
def _expand(p: str) -> str:
    return os.path.expanduser(os.path.expandvars(p))


def load_config(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# Drone accent colours (RGB 0-1) — one per drone, indexed by drone_id.
DRONE_COLOURS: List[Tuple[float, float, float]] = [
    (0.10, 0.30, 0.85),   # 0: blue
    (0.90, 0.45, 0.10),   # 1: orange
    (0.20, 0.75, 0.30),   # 2: green
    (0.85, 0.20, 0.65),   # 3: magenta
    (0.25, 0.80, 0.85),   # 4: cyan
    (0.85, 0.85, 0.20),   # 5: yellow
]


@dataclass
class Flower:
    flower_id:        int
    x:                float
    y:                float
    head_z:           float
    head_r:           float
    bloom_start:      int
    head_azimuth_rad: float   # which way the head faces (yaw)
    head_tilt_rad:    float   # angle from vertical (pitch)


# ────────────────────────────────────────────────────────────────────
# Field layout
# ────────────────────────────────────────────────────────────────────
def sample_flowers(cfg: dict, rng: random.Random) -> List[Flower]:
    Lx, Ly = cfg['mission']['field_size_m']
    margin = 0.7
    min_spacing = 0.8        # min metres between any two flowers
    target = int(cfg['mission']['flower_count_target'])
    jitter = int(cfg['mission']['flower_count_jitter'])
    n_target = max(4, target + rng.randint(-jitter, jitter))

    head_z_choices = [1.5, 1.7, 1.9, 2.1]
    head_r_choices = [0.16, 0.18, 0.20, 0.22]

    bloom_mean  = float(cfg['bloom']['bloom_start_mean_day'])
    bloom_sigma = float(cfg['bloom']['bloom_start_sigma_days'])

    # Real sunflowers face roughly east at ~20° from vertical (heliotropism
    # converges east-ish at maturity). Read from config if present, else
    # use sensible defaults so older config files still work.
    fm = cfg.get('field_model', {})
    az_mean_deg   = float(fm.get('head_azimuth_deg',          90.0))   # +Y = east
    az_jitter_deg = float(fm.get('head_azimuth_jitter_deg',   15.0))
    ti_mean_deg   = float(fm.get('head_tilt_deg',             20.0))   # down from vertical
    ti_jitter_deg = float(fm.get('head_tilt_jitter_deg',      10.0))

    flowers: List[Flower] = []
    tries = 0
    while len(flowers) < n_target and tries < 5000:
        tries += 1
        x = rng.uniform(margin, Lx - margin)
        y = rng.uniform(margin, Ly - margin)
        if all(math.hypot(x - f.x, y - f.y) >= min_spacing for f in flowers):
            bs = max(0, int(round(rng.gauss(bloom_mean, bloom_sigma))))
            az_deg = az_mean_deg + rng.uniform(-az_jitter_deg, az_jitter_deg)
            ti_deg = max(0.0, ti_mean_deg + rng.uniform(-ti_jitter_deg, ti_jitter_deg))
            flowers.append(Flower(
                flower_id=len(flowers) + 1,
                x=round(x, 3), y=round(y, 3),
                head_z=rng.choice(head_z_choices),
                head_r=rng.choice(head_r_choices),
                bloom_start=bs,
                head_azimuth_rad=math.radians(az_deg),
                head_tilt_rad=math.radians(ti_deg),
            ))
    return flowers


def drone_bases(cfg: dict) -> List[Tuple[int, float, float]]:
    """Return list of (drone_id, base_x, base_y). Pads sit on a strip
    just south of the field at y = -1.0, spaced equally along x."""
    Lx, _ = cfg['mission']['field_size_m']
    n = int(cfg['swarm']['n_drones'])
    sep = float(cfg['swarm']['base_separation_m'])
    pad_y = -1.0
    # Centre the pad row on x = Lx/2
    total_w = sep * (n - 1)
    start_x = Lx / 2.0 - total_w / 2.0
    return [(i, round(start_x + i * sep, 3), pad_y) for i in range(n)]


# ────────────────────────────────────────────────────────────────────
# SDF model builders (Stage 14 style)
# ────────────────────────────────────────────────────────────────────
def flower_model(f: Flower) -> str:
    """A sunflower: stem cylinder + two leaves + a single 'bloom' link
    containing petal disc + brown head, tilted together so they look
    like a real sunflower head (~20° from vertical, facing east-ish).
    Static — no <collision> elements, so drones pass through. That's
    deliberate for Pass 1; we add collision later.

    Pose convention: roll=0, pitch=head_tilt_rad, yaw=head_azimuth_rad.
    Net effect is the disc's Z-axis points in direction
    (sin θ cos φ, sin θ sin φ, cos θ) where θ=tilt, φ=azimuth.
    """
    stem_len = f.head_z
    return f"""
    <model name="sunflower_{f.flower_id}">
      <static>true</static>
      <pose>{f.x:.3f} {f.y:.3f} 0 0 0 0</pose>

      <link name="stem">
        <pose>0 0 {stem_len/2:.3f} 0 0 0</pose>
        <visual name="v">
          <geometry><cylinder><radius>0.04</radius>
            <length>{stem_len:.3f}</length></cylinder></geometry>
          <material><ambient>0.12 0.55 0.12 1</ambient>
            <diffuse>0.12 0.55 0.12 1</diffuse></material>
        </visual>
      </link>

      <link name="leaf_l">
        <pose>-0.18 0 {stem_len*0.6:.3f} 0 0.6 0</pose>
        <visual name="v">
          <geometry><ellipsoid><radii>0.22 0.10 0.02</radii></ellipsoid></geometry>
          <material><ambient>0.15 0.6 0.18 1</ambient>
            <diffuse>0.15 0.6 0.18 1</diffuse></material>
        </visual>
      </link>

      <link name="leaf_r">
        <pose>0.18 0 {stem_len*0.7:.3f} 0 -0.6 0</pose>
        <visual name="v">
          <geometry><ellipsoid><radii>0.22 0.10 0.02</radii></ellipsoid></geometry>
          <material><ambient>0.15 0.6 0.18 1</ambient>
            <diffuse>0.15 0.6 0.18 1</diffuse></material>
        </visual>
      </link>

      <!-- Bloom: petals + head as one tilted link so they move together.
           Pitch tilts the local Z-axis; yaw orients the tilt toward
           the head's facing direction (azimuth). -->
      <link name="bloom">
        <pose>0 0 {f.head_z:.3f} 0 {f.head_tilt_rad:.4f} {f.head_azimuth_rad:.4f}</pose>
        <visual name="petals">
          <pose>0 0 0 0 0 0</pose>
          <geometry><cylinder><radius>{f.head_r*1.7:.3f}</radius>
            <length>0.04</length></cylinder></geometry>
          <material><ambient>0.98 0.85 0.06 1</ambient>
            <diffuse>0.98 0.85 0.06 1</diffuse></material>
        </visual>
        <visual name="disc">
          <pose>0 0 0.03 0 0 0</pose>
          <geometry><cylinder><radius>{f.head_r:.3f}</radius>
            <length>0.06</length></cylinder></geometry>
          <material><ambient>0.45 0.27 0.05 1</ambient>
            <diffuse>0.45 0.27 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def pad_model(name: str, x: float, y: float,
              colour: Tuple[float, float, float]) -> str:
    r, g, b = colour
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.02 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><cylinder><radius>0.45</radius>
            <length>0.04</length></cylinder></geometry>
          <material>
            <ambient>{r*0.7:.3f} {g*0.7:.3f} {b*0.7:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def drone_model(name: str, x: float, y: float, z: float,
                colour: Tuple[float, float, float]) -> str:
    """A kinematic quadrotor. Mirrors demo_stage14.sdf:
       - <gravity>false</gravity> on the link (kinematic — won't fall)
       - <self_collide>false</self_collide> on the model
       - VelocityControl plugin with NO <topic> override
       - PosePublisher publishing model pose at 50 Hz
    """
    r, g, b = colour
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <self_collide>false</self_collide>

      <link name="base_link">
        <gravity>false</gravity>

        <inertial>
          <mass>0.135</mass>
          <inertia>
            <ixx>0.003</ixx><iyy>0.003</iyy><izz>0.006</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>

        <!-- Body -->
        <visual name="body">
          <geometry><box><size>0.16 0.16 0.04</size></box></geometry>
          <material>
            <ambient>{r*0.7:.3f} {g*0.7:.3f} {b*0.7:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
        <collision name="body_c">
          <geometry><box><size>0.16 0.16 0.04</size></box></geometry>
        </collision>

        <!-- X-arms -->
        <visual name="arm_fl">
          <pose>0.12 0.12 0 0 0 0.7854</pose>
          <geometry><box><size>0.22 0.018 0.018</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_fr">
          <pose>0.12 -0.12 0 0 0 -0.7854</pose>
          <geometry><box><size>0.22 0.018 0.018</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_bl">
          <pose>-0.12 0.12 0 0 0 -0.7854</pose>
          <geometry><box><size>0.22 0.018 0.018</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="arm_br">
          <pose>-0.12 -0.12 0 0 0 0.7854</pose>
          <geometry><box><size>0.22 0.018 0.018</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>

        <!-- Propellers -->
        <visual name="prop_fl">
          <pose>0.17 0.17 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius>
            <length>0.005</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.1 0.6</ambient>
            <diffuse>0.85 0.1 0.1 0.6</diffuse></material>
        </visual>
        <visual name="prop_fr">
          <pose>0.17 -0.17 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius>
            <length>0.005</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.1 0.6</ambient>
            <diffuse>0.85 0.1 0.1 0.6</diffuse></material>
        </visual>
        <visual name="prop_bl">
          <pose>-0.17 0.17 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius>
            <length>0.005</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.1 0.6</ambient>
            <diffuse>0.85 0.1 0.1 0.6</diffuse></material>
        </visual>
        <visual name="prop_br">
          <pose>-0.17 -0.17 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.075</radius>
            <length>0.005</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.1 0.6</ambient>
            <diffuse>0.85 0.1 0.1 0.6</diffuse></material>
        </visual>

        <!-- Forward marker (yellow stripe — visible orientation) -->
        <visual name="forward_marker">
          <pose>0.085 0 0.022 0 0 0</pose>
          <geometry><box><size>0.04 0.025 0.005</size></box></geometry>
          <material><ambient>1.0 1.0 0.05 1</ambient>
            <diffuse>1.0 1.0 0.05 1</diffuse></material>
        </visual>

        <!-- Pistil rod (30 cm rigid rod hanging from CG) -->
        <visual name="pistil">
          <pose>0 0 -0.17 0 0 0</pose>
          <geometry><cylinder><radius>0.005</radius>
            <length>0.30</length></cylinder></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient>
            <diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <!-- Pistil brush tip -->
        <visual name="brush">
          <pose>0 0 -0.325 0 0 0</pose>
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material><ambient>0.95 0.80 0.20 1</ambient>
            <diffuse>1.0  0.90 0.30 1</diffuse></material>
        </visual>
      </link>

      <!-- VelocityControl plugin — accepts gz.msgs.Twist on
           the DEFAULT topic /model/{name}/cmd_vel. Do NOT add a
           <topic> sub-element; the default is exactly what we want
           and matches Stage 14. -->
      <plugin filename="gz-sim-velocity-control-system"
              name="gz::sim::systems::VelocityControl">
      </plugin>

      <!-- Pose publisher — publishes to /model/{name}/pose at 50 Hz -->
      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <static_publisher>false</static_publisher>
        <update_frequency>50</update_frequency>
      </plugin>
    </model>"""


def field_tile(Lx: float, Ly: float) -> str:
    """A soil-coloured tile placed under the flowers so the field
    is visibly distinct from the surrounding grass plane."""
    return f"""
    <model name="field_tile">
      <static>true</static>
      <pose>{Lx/2:.3f} {Ly/2:.3f} 0.005 0 0 0</pose>
      <link name="l">
        <visual name="v">
          <geometry><box><size>{Lx:.3f} {Ly:.3f} 0.01</size></box></geometry>
          <material>
            <ambient>0.30 0.20 0.10 1</ambient>
            <diffuse>0.40 0.27 0.15 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


# ────────────────────────────────────────────────────────────────────
# Full SDF (Stage 14 template, parameterised)
# ────────────────────────────────────────────────────────────────────
def build_sdf(cfg: dict, flowers: List[Flower],
              bases: List[Tuple[int, float, float]]) -> str:
    Lx, Ly = cfg['mission']['field_size_m']

    # Camera default pose: behind the field, looking at its centre.
    # Match Stage 14's "(4, -7, 7) looking N" style.
    cam_x = Lx / 2.0
    cam_y = -7.0
    cam_z = 8.0
    cam_pitch = 0.55
    cam_yaw   = 1.5708     # face +Y (north, into the field)

    flowers_xml = "\n".join(flower_model(f) for f in flowers)
    pads_xml = "\n".join(
        pad_model(f"pad_{did}", bx, by, DRONE_COLOURS[did % len(DRONE_COLOURS)])
        for did, bx, by in bases
    )
    drones_xml = "\n".join(
        drone_model(f"drone_{did}", bx, by, 0.10,
                    DRONE_COLOURS[did % len(DRONE_COLOURS)])
        for did, bx, by in bases
    )
    tile_xml = field_tile(Lx, Ly)

    return f"""<?xml version="1.0" ?>
<!--
  stage15_world.sdf — generated by generate_field_stage15.py
  Template mirrors demo_stage14.sdf, which loads reliably on this
  machine (Ubuntu 24.04 / Wayland / NVIDIA / Gazebo Harmonic 8.11.0).
-->
<sdf version="1.9">
  <world name="stage15_world">

    <!-- Physics + system plugins (identical to Stage 14) -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>{float(cfg['simulation']['real_time_factor'])}</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <gravity>0 0 -9.8</gravity>
    <atmosphere type="adiabatic"/>

    <!-- Explicit GUI camera pose. With this set, the GUI doesn't have
         to walk the entire scene graph to compute bounds at startup. -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{cam_x:.2f} {cam_y:.2f} {cam_z:.2f} 0 {cam_pitch:.2f} {cam_yaw:.4f}</pose>
      </camera>
    </gui>

    <scene>
      <grid>false</grid>
      <ambient>0.95 0.95 0.95</ambient>
      <background>0.65 0.78 0.95</background>
      <sky/>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>1.0 0.97 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.005</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- Inline ground plane — DO NOT replace this with a Fuel <include>;
         the HTTPS fetch hangs gz sim at startup. Stage 14 uses this exact
         pattern and it works. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><plane><normal>0 0 1</normal>
            <size>50 50</size></plane></geometry>
        </collision>
        <visual name="v">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal>
            <size>50 50</size></plane></geometry>
          <material>
            <ambient>0.30 0.50 0.22 1</ambient>
            <diffuse>0.30 0.50 0.22 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    {tile_xml}
    {pads_xml}
    {drones_xml}
    {flowers_xml}

  </world>
</sdf>
"""


# ────────────────────────────────────────────────────────────────────
# Outputs
# ────────────────────────────────────────────────────────────────────
def write_flowers_json(path: str, flowers: List[Flower],
                       bases: List[Tuple[int, float, float]],
                       cfg: dict):
    data = {
        'n_drones':          int(cfg['swarm']['n_drones']),
        'field_size_m':      list(cfg['mission']['field_size_m']),
        'bloom_window_days': int(cfg['mission']['bloom_window_days']),
        'drone_bases': [
            {'drone_id': did, 'base_x': bx, 'base_y': by}
            for did, bx, by in bases
        ],
        'flowers': [
            {'flower_id': f.flower_id,
             'x': f.x, 'y': f.y,
             'head_z': f.head_z, 'head_r': f.head_r,
             'head_azimuth_rad': round(f.head_azimuth_rad, 4),
             'head_tilt_rad':    round(f.head_tilt_rad, 4),
             'bloom_start': f.bloom_start}
            for f in flowers
        ],
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def write_drone_specs(path: str, bases):
    specs = [{'drone_id': did, 'base_x': bx, 'base_y': by}
             for did, bx, by in bases]
    with open(path, 'w') as f:
        json.dump(specs, f, indent=2)


# ────────────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────────────
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=os.path.join(here, 'stage15_config.yaml'))
    parser.add_argument('--world',  default='/tmp/stage15_world.sdf')
    parser.add_argument('--json',   default='/tmp/stage15_flowers.json')
    parser.add_argument('--specs',  default=os.path.join(here, 'drone_specs.json'))
    args = parser.parse_args()

    cfg = load_config(_expand(args.config))
    seed = int(cfg['simulation']['random_seed'])
    rng = random.Random(seed)

    flowers = sample_flowers(cfg, rng)
    bases   = drone_bases(cfg)

    sdf = build_sdf(cfg, flowers, bases)
    with open(_expand(args.world), 'w') as f:
        f.write(sdf)
    write_flowers_json(_expand(args.json), flowers, bases, cfg)
    write_drone_specs(_expand(args.specs), bases)

    print(f"── generate_field_stage15 ──")
    print(f"  field:    {cfg['mission']['field_size_m'][0]:.1f} × "
          f"{cfg['mission']['field_size_m'][1]:.1f} m")
    print(f"  drones:   {len(bases)}  (bases on y=-1.0 strip)")
    print(f"  flowers:  {len(flowers)}  (seed {seed})")
    print(f"  world:    {args.world}")
    print(f"  flowers:  {args.json}")


if __name__ == '__main__':
    main()
