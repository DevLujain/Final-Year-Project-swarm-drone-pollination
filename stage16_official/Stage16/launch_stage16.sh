#!/usr/bin/env bash
# launch_stage16.sh — Stage 16 v12 (clean)
set -e

HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONFIG="${1:-$HERE/stage16_config.yaml}"
[[ -f "$CONFIG" ]] || { echo "ERROR: config not found: $CONFIG"; exit 1; }

echo "────────────────────────────────────────────────────────"
echo "  Stage 16 launcher"
echo "  config: $CONFIG"
echo "────────────────────────────────────────────────────────"

read_yaml() {
python3 - "$1" "$2" << 'PYEOF'
import sys, yaml
with open(sys.argv[1]) as f: cfg = yaml.safe_load(f)
path = sys.argv[2].split('.')
v = cfg
for p in path: v = v[p]
if isinstance(v, list): print(' '.join(str(x) for x in v))
elif isinstance(v, bool): print('true' if v else 'false')
else: print(v)
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
BATT_SWAP=$(read_yaml "$CONFIG" 'drone.battery_swap_seconds')
CRUISE_Z=$(read_yaml "$CONFIG" 'mission_altitudes.cruise_m')
APPROACH_Z=$(read_yaml "$CONFIG" 'mission_altitudes.approach_m')
SURVEY_Z=$(read_yaml "$CONFIG" 'mission_altitudes.survey_m')
FIELD_X=$(read_yaml "$CONFIG" 'mission.field_size_m' | awk '{print $1}')
FIELD_Y=$(read_yaml "$CONFIG" 'mission.field_size_m' | awk '{print $2}')
P_HFOV=$(read_yaml "$CONFIG" 'perception.hfov_deg')
P_W=$(read_yaml "$CONFIG" 'perception.image_width')
P_H=$(read_yaml "$CONFIG" 'perception.image_height')
P_DEDUP=$(read_yaml "$CONFIG" 'perception.dedup_radius_m')
P_AREA_MIN=$(read_yaml "$CONFIG" 'perception.min_blob_area_px')
P_AREA_MAX=$(read_yaml "$CONFIG" 'perception.max_blob_area_px')
P_H_MIN=$(read_yaml "$CONFIG" 'perception.hsv_h_min')
P_H_MAX=$(read_yaml "$CONFIG" 'perception.hsv_h_max')
P_S_MIN=$(read_yaml "$CONFIG" 'perception.hsv_s_min')
P_V_MIN=$(read_yaml "$CONFIG" 'perception.hsv_v_min')
P_SX=$(read_yaml "$CONFIG" 'perception.image_to_world_x_sign')
P_SY=$(read_yaml "$CONFIG" 'perception.image_to_world_y_sign')
P_SWAP=$(read_yaml "$CONFIG" 'perception.image_swap_xy')
P_MIN_ALT=$(read_yaml "$CONFIG" 'perception.min_altitude_for_detect_m')
P_MIN_BROWN=$(read_yaml "$CONFIG" 'perception.min_brown_samples')
P_BH_MIN=$(read_yaml "$CONFIG" 'perception.hsv_brown_h_min')
P_BH_MAX=$(read_yaml "$CONFIG" 'perception.hsv_brown_h_max')
P_BS_MIN=$(read_yaml "$CONFIG" 'perception.hsv_brown_s_min')
P_BV_MIN=$(read_yaml "$CONFIG" 'perception.hsv_brown_v_min')
P_BV_MAX=$(read_yaml "$CONFIG" 'perception.hsv_brown_v_max')
P_GH_MIN=$(read_yaml "$CONFIG" 'perception.hsv_green_h_min')
P_GH_MAX=$(read_yaml "$CONFIG" 'perception.hsv_green_h_max')
P_GS_MIN=$(read_yaml "$CONFIG" 'perception.hsv_green_s_min')
P_GV_MIN=$(read_yaml "$CONFIG" 'perception.hsv_green_v_min')
DASH_ENABLED=$(read_yaml "$CONFIG" 'dashboard.enabled')
DASH_PORT=$(read_yaml "$CONFIG" 'dashboard.port')
DASH_HOST=$(read_yaml "$CONFIG" 'dashboard.host')
LOG_DIR=$(read_yaml "$CONFIG" 'logging.log_directory')

echo "  n_drones=$N_DRONES  mode=$MODE  day=$CURRENT_DAY..$DAY_MAX"
echo "  field ${FIELD_X}x${FIELD_Y} m"

echo ""
echo "── Generating field & SDF…"
python3 "$HERE/Field_generator.py" --config "$CONFIG"

declare -a BASE_X BASE_Y
mapfile -t SPEC_LINES < <(python3 - "$HERE/drone_specs.json" << 'PYEOF'
import json, sys
with open(sys.argv[1]) as f: specs = json.load(f)
for s in specs: print(f"{s['drone_id']} {s['base_x']} {s['base_y']}")
PYEOF
)
for line in "${SPEC_LINES[@]}"; do
  read -r did bx by <<< "$line"
  BASE_X[$did]=$bx; BASE_Y[$did]=$by
done

echo ""
echo "── Killing leftover processes…"
pkill -9 -f 'gz sim'              2>/dev/null || true
pkill -9 -f gz-sim                2>/dev/null || true
pkill -9 -f ros_gz_bridge         2>/dev/null || true
pkill -9 -f Drone_Controller.py   2>/dev/null || true
pkill -9 -f mission_orchestrator  2>/dev/null || true
pkill -9 -f perception_node.py    2>/dev/null || true
pkill -9 -f 'dashboard/app.py'    2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 2

WORLD="/tmp/stage16_world.sdf"
echo ""
echo "── Starting Gazebo…"
gnome-terminal --tab --title="Gazebo" -- bash -c \
  "source ~/.bashrc; export GZ_IP=127.0.0.1; gz sim -v 2 -r $WORLD; exec bash" &
sleep 10

BRIDGE_ARGS=""
for i in $(seq 0 $((N_DRONES - 1))); do
  BRIDGE_ARGS+=" /model/drone_${i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
  BRIDGE_ARGS+=" /model/drone_${i}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"
  BRIDGE_ARGS+=" /world/stage16_world/model/drone_${i}/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
  BRIDGE_ARGS+=" /world/stage16_world/model/drone_${i}/link/base_link/sensor/tof_brush/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
done

REMAPS=""
for i in $(seq 0 $((N_DRONES - 1))); do
  REMAPS+=" -r /world/stage16_world/model/drone_${i}/link/base_link/sensor/camera/image:=/drone_${i}/camera/image_raw"
  REMAPS+=" -r /world/stage16_world/model/drone_${i}/link/base_link/sensor/tof_brush/scan:=/drone_${i}/tof_brush/scan"
done

echo "── Starting ros_gz_bridge…"
gnome-terminal --tab --title="gz_bridge" -- bash -c \
  "source ~/.bashrc; export GZ_IP=127.0.0.1; \
   ros2 run ros_gz_bridge parameter_bridge $BRIDGE_ARGS --ros-args $REMAPS; exec bash" &
sleep 5

echo "── Starting orchestrator…"
gnome-terminal --tab --title="orch" -- bash -c \
  "source ~/.bashrc; export GZ_IP=127.0.0.1; \
   python3 $HERE/mission_orchestrator.py --ros-args \
     -p n_drones:=$N_DRONES \
     -p mode:='$MODE' \
     -p current_day:=$CURRENT_DAY \
     -p bloom_window_days:=$DAY_MAX \
     -p flowers_json:='/tmp/stage16_flowers.json' \
     -p log_directory:='$LOG_DIR' \
     -p dedup_radius_m:=$P_DEDUP \
     -p init_dwell_s:=6.0 \
     -p merge_dwell_s:=2.0 \
     -p day_settle_s:=3.0 ; exec bash" &

echo "── Starting bloom_world…"
sleep 1
gnome-terminal --tab --title="bloom" -- bash -c \
  "source ~/.bashrc; export GZ_IP=127.0.0.1; \
   python3 $HERE/bloom_world.py --ros-args \
     -p world_name:='stage16_world' \
     -p current_day:=$CURRENT_DAY \
     -p flowers_json:='/tmp/stage16_flowers.json' ; exec bash" &

sleep 3  # wait for orchestrator to publish bloom topics
for i in $(seq 0 $((N_DRONES - 1))); do
  bx=${BASE_X[$i]}; by=${BASE_Y[$i]}
  echo "── Starting controller for drone $i (base=$bx,$by)…"
  gnome-terminal --tab --title="ctrl_$i" -- bash -c \
    "source ~/.bashrc; export GZ_IP=127.0.0.1; \
     python3 $HERE/Drone_Controller.py --ros-args \
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
       -p battery_full_minutes:=$BATT_MIN \
       -p battery_swap_duration_s:=$BATT_SWAP ; exec bash" &
  sleep 0.4
done

echo ""
for i in $(seq 0 $((N_DRONES - 1))); do
  echo "── Starting perception_node for drone $i…"
  gnome-terminal --tab --title="perc_$i" -- bash -c \
    "source ~/.bashrc; export GZ_IP=127.0.0.1; \
     python3 $HERE/perception_node.py --ros-args \
       -p drone_id:=$i \
       -p world_name:='stage16_world' \
       -p camera_hfov_deg:=$P_HFOV \
       -p camera_width:=$P_W \
       -p camera_height:=$P_H \
       -p dedup_radius_m:=$P_DEDUP \
       -p min_blob_area_px:=$P_AREA_MIN \
       -p max_blob_area_px:=$P_AREA_MAX \
       -p hsv_h_min:=$P_H_MIN \
       -p hsv_h_max:=$P_H_MAX \
       -p hsv_s_min:=$P_S_MIN \
       -p hsv_v_min:=$P_V_MIN \
       -p image_to_world_x_sign:=$P_SX \
       -p image_to_world_y_sign:=$P_SY \
       -p image_swap_xy:=$P_SWAP \
       -p min_altitude_for_detect_m:=$P_MIN_ALT \
       -p min_brown_samples:=$P_MIN_BROWN \
       -p hsv_brown_h_min:=$P_BH_MIN \
       -p hsv_brown_h_max:=$P_BH_MAX \
       -p hsv_brown_s_min:=$P_BS_MIN \
       -p hsv_brown_v_min:=$P_BV_MIN \
       -p hsv_brown_v_max:=$P_BV_MAX \
       -p hsv_green_h_min:=$P_GH_MIN \
       -p hsv_green_h_max:=$P_GH_MAX \
       -p hsv_green_s_min:=$P_GS_MIN \
       -p hsv_green_v_min:=$P_GV_MIN \
       -p field_size_x_m:=$FIELD_X \
       -p field_size_y_m:=$FIELD_Y ; exec bash" &
  sleep 0.4
done

if [[ "$DASH_ENABLED" == "true" || "$DASH_ENABLED" == "True" ]]; then
  echo ""
  echo "── Starting dashboard on http://${DASH_HOST}:${DASH_PORT} …"
  gnome-terminal --tab --title="dashboard" -- bash -c \
    "source ~/.bashrc; export GZ_IP=127.0.0.1; \
     export STAGE16_N_DRONES=$N_DRONES; \
     export STAGE16_FIELD_X=$FIELD_X; \
     export STAGE16_FIELD_Y=$FIELD_Y; \
     export STAGE16_DASH_PORT=$DASH_PORT; \
     export STAGE16_DASH_HOST='$DASH_HOST'; \
     python3 $HERE/dashboard/app.py ; exec bash" &
fi

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Stage 16 launched."
echo "  Dashboard: http://${DASH_HOST}:${DASH_PORT}"
echo "  Logs: $LOG_DIR"
echo "════════════════════════════════════════════════════════"
