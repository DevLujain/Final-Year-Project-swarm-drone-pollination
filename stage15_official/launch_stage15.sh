#!/usr/bin/env bash
# ============================================================
# launch_stage15.sh — Stage 15 (clean rewrite)
# ============================================================
# Usage:   ./launch_stage15.sh                  (uses default config)
#          ./launch_stage15.sh path/to/cfg.yaml
#
# Design follows Stage 14 — which has been proven to load reliably
# on this machine (Ubuntu 24.04 / Wayland / NVIDIA / Gazebo 8.11.0):
#
#   * One single `gz sim -v 2 -r WORLD.sdf` in dispatcher mode
#     (Stage 14 uses this; no fork/two-process gymnastics needed
#     once the SDF is right)
#   * Each gnome-terminal tab `source ~/.bashrc` so ROS 2 env is
#     loaded the same way the user has it interactively
#   * No PX4, no XRCE-DDS — drones are pure Gazebo kinematic models
#     driven by Python PID over /model/drone_N/cmd_vel
#
# ============================================================

set -e

# ── 0. Locate config ────────────────────────────────────────
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONFIG="${1:-$HERE/stage15_config.yaml}"
if [[ ! -f "$CONFIG" ]]; then
  echo "ERROR: config not found at $CONFIG"
  exit 1
fi

echo "────────────────────────────────────────────────────────"
echo "  Stage 15 launcher (clean)"
echo "  config: $CONFIG"
echo "────────────────────────────────────────────────────────"

# ── 1. Read YAML scalars ────────────────────────────────────
read_yaml() {
python3 - "$1" "$2" << 'PYEOF'
import sys, yaml
with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f)
path = sys.argv[2].split('.')
v = cfg
for p in path:
    v = v[p]
if isinstance(v, list):
    print(' '.join(str(x) for x in v))
else:
    print(v)
PYEOF
}

N_DRONES=$(read_yaml "$CONFIG" 'swarm.n_drones')
MODE=$(read_yaml "$CONFIG" 'mode.type')
CURRENT_DAY=$(read_yaml "$CONFIG" 'mode.current_day')
DAY_MAX=$(read_yaml "$CONFIG" 'mission.bloom_window_days')

CRUISE_SPEED=$(read_yaml "$CONFIG" 'drone.cruise_speed_m_s')
APPROACH_SPEED=$(read_yaml "$CONFIG" 'drone.approach_speed_m_s')
CONTACT_SPEED=$(read_yaml "$CONFIG" 'drone.contact_descent_speed_m_s')
PISTIL_LEN=$(read_yaml "$CONFIG" 'drone.pistil_length_m')
TOF_C=$(read_yaml "$CONFIG" 'drone.tof_contact_distance_m')
BRUSH_DUR=$(read_yaml "$CONFIG" 'drone.brush_sweep_duration_s')
BATT_MIN=$(read_yaml "$CONFIG" 'drone.battery_full_minutes')

CRUISE_Z=$(read_yaml "$CONFIG" 'mission_altitudes.cruise_m')
APPROACH_Z=$(read_yaml "$CONFIG" 'mission_altitudes.approach_m')
SURVEY_Z=$(read_yaml "$CONFIG" 'mission_altitudes.survey_m')

FIELD_X=$(read_yaml "$CONFIG" 'mission.field_size_m' | awk '{print $1}')
FIELD_Y=$(read_yaml "$CONFIG" 'mission.field_size_m' | awk '{print $2}')

LOG_DIR=$(read_yaml "$CONFIG" 'logging.log_directory')

echo "  n_drones=$N_DRONES  mode=$MODE  day=$CURRENT_DAY..$DAY_MAX"
echo "  field ${FIELD_X}x${FIELD_Y} m"

# ── 2. Generate field ───────────────────────────────────────
echo ""
echo "── Generating field & SDF…"
python3 "$HERE/generate_field_stage15.py" --config "$CONFIG"

# Read per-drone bases (written by the generator)
declare -a BASE_X BASE_Y
mapfile -t SPEC_LINES < <(python3 - "$HERE/drone_specs.json" << 'PYEOF'
import json, sys
with open(sys.argv[1]) as f:
    specs = json.load(f)
for s in specs:
    print(f"{s['drone_id']} {s['base_x']} {s['base_y']}")
PYEOF
)
for line in "${SPEC_LINES[@]}"; do
  read -r did bx by <<< "$line"
  BASE_X[$did]=$bx
  BASE_Y[$did]=$by
done

# ── 3. Clean up previous run ────────────────────────────────
echo ""
echo "── Killing leftover gz / ros2 processes…"
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f gz-sim   2>/dev/null || true
pkill -9 -f ros_gz_bridge 2>/dev/null || true
pkill -9 -f drone_controller_stage15  2>/dev/null || true
pkill -9 -f mission_orchestrator_stage15 2>/dev/null || true
ros2 daemon stop  2>/dev/null || true
sleep 2

# ── 4. Launch Gazebo (dispatcher mode — Stage 14 pattern) ──
WORLD="/tmp/stage15_world.sdf"
echo ""
echo "── Starting Gazebo (dispatcher mode)…"
gnome-terminal --tab --title="Gazebo" -- bash -c \
  "source ~/.bashrc; gz sim -v 2 -r $WORLD; exec bash" &

# Give Gazebo time to bring up server + GUI before bridging
sleep 6

# ── 5. ros_gz_bridge per drone ──────────────────────────────
BRIDGE_ARGS=""
for i in $(seq 0 $((N_DRONES - 1))); do
  BRIDGE_ARGS+=" /model/drone_${i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
  BRIDGE_ARGS+=" /model/drone_${i}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
done
echo "── Starting ros_gz_bridge…"
gnome-terminal --tab --title="gz_bridge" -- bash -c \
  "source ~/.bashrc; ros2 run ros_gz_bridge parameter_bridge $BRIDGE_ARGS; exec bash" &
sleep 3

# ── 6. Orchestrator ─────────────────────────────────────────
echo "── Starting orchestrator…"
gnome-terminal --tab --title="orch" -- bash -c \
  "source ~/.bashrc; \
   python3 $HERE/mission_orchestrator_stage15.py --ros-args \
     -p n_drones:=$N_DRONES \
     -p mode:='$MODE' \
     -p current_day:=$CURRENT_DAY \
     -p bloom_window_days:=$DAY_MAX \
     -p flowers_json:='/tmp/stage15_flowers.json' \
     -p log_directory:='$LOG_DIR' \
     -p init_dwell_s:=6.0 \
     -p merge_dwell_s:=2.0 \
     -p day_settle_s:=3.0 ; exec bash" &

# ── 7. Per-drone controllers ────────────────────────────────
for i in $(seq 0 $((N_DRONES - 1))); do
  bx=${BASE_X[$i]}
  by=${BASE_Y[$i]}
  echo "── Starting controller for drone $i (base=$bx,$by)…"
  gnome-terminal --tab --title="ctrl_$i" -- bash -c \
    "source ~/.bashrc; \
     python3 $HERE/drone_controller_stage15.py --ros-args \
       -p drone_id:=$i \
       -p n_drones:=$N_DRONES \
       -p base_x:=$bx \
       -p base_y:=$by \
       -p field_size_x:=$FIELD_X \
       -p field_size_y:=$FIELD_Y \
       -p survey_altitude_m:=$SURVEY_Z \
       -p cruise_altitude_m:=$CRUISE_Z \
       -p approach_altitude_m:=$APPROACH_Z \
       -p cruise_speed_m_s:=$CRUISE_SPEED \
       -p approach_speed_m_s:=$APPROACH_SPEED \
       -p contact_descent_speed_m_s:=$CONTACT_SPEED \
       -p pistil_length_m:=$PISTIL_LEN \
       -p tof_contact_m:=$TOF_C \
       -p brush_sweep_duration_s:=$BRUSH_DUR \
       -p battery_full_minutes:=$BATT_MIN ; exec bash" &
  sleep 0.5
done

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Stage 15 launched."
echo "  Tabs:  Gazebo, gz_bridge, orch, ctrl_0..ctrl_$((N_DRONES-1))"
echo ""
echo "  Watch the unified event stream from another terminal:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    ros2 topic echo /swarm/event"
echo ""
echo "  Logs:  $LOG_DIR"
echo "════════════════════════════════════════════════════════"
