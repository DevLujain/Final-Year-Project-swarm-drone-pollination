#!/bin/bash
# ============================================================
# FIX px4_msgs — resolves "VehicleStatus ABSENT" / status not
# flowing back from PX4 v1.16.
#
# Cause: the workspace px4_msgs lacks the versioned message
# types (VehicleStatusV4, VehicleLocalPositionV1) that PX4
# v1.16 publishes, OR has stale definitions with a different
# wire format. Commands still go IN (arming works) but status
# never comes OUT. This syncs px4_msgs from your PX4 source
# and rebuilds it.
#
# RUN: bash fix_px4msgs.sh
# THEN re-source and relaunch:
#   source ~/Desktop/FYP/ros2_ws/install/setup.bash
#   bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e
ROS2_WS=~/Desktop/FYP/ros2_ws
PX4_DIR=~/PX4-Autopilot
PX4_MSGS=$ROS2_WS/src/px4_msgs

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash 2>/dev/null || true

echo ""
echo "=============================================="
echo "  px4_msgs sync / rebuild"
echo "=============================================="
echo ""

# ── Diagnose ──────────────────────────────────────────────────
echo "[diagnose] checking current px4_msgs types..."
HAVE_V4=0
if ros2 interface show px4_msgs/msg/VehicleStatusV4 >/dev/null 2>&1; then
    echo "  VehicleStatusV4:        PRESENT"
    HAVE_V4=1
else
    echo "  VehicleStatusV4:        MISSING"
fi
if ros2 interface show px4_msgs/msg/VehicleLocalPositionV1 >/dev/null 2>&1; then
    echo "  VehicleLocalPositionV1: PRESENT"
else
    echo "  VehicleLocalPositionV1: MISSING"
fi

if [ ! -d "$PX4_MSGS/msg" ]; then
    echo "ERROR: $PX4_MSGS/msg not found - px4_msgs package missing."
    exit 1
fi
if [ ! -d "$PX4_DIR/msg" ]; then
    echo "ERROR: $PX4_DIR/msg not found - cannot sync from PX4 source."
    exit 1
fi

# ── Sync regardless (idempotent; guarantees match with PX4) ──
echo ""
echo "[sync] copying .msg files from PX4 v1.16 source..."
BEFORE=$(ls $PX4_MSGS/msg/*.msg 2>/dev/null | wc -l)
rm -f $PX4_MSGS/msg/*.msg
cp $PX4_DIR/msg/*.msg $PX4_MSGS/msg/ 2>/dev/null || true
if [ -d "$PX4_DIR/msg/versioned" ]; then
    cp $PX4_DIR/msg/versioned/*.msg $PX4_MSGS/msg/ 2>/dev/null || true
fi
AFTER=$(ls $PX4_MSGS/msg/*.msg 2>/dev/null | wc -l)
echo "  msg files: $BEFORE -> $AFTER"

# Verify the versioned ones arrived
for V in VehicleStatusV4 VehicleLocalPositionV1; do
    if [ -f "$PX4_MSGS/msg/${V}.msg" ]; then
        echo "  ${V}.msg present in source"
    else
        echo "  WARNING: ${V}.msg not found in PX4 source - your PX4 may"
        echo "           use different names. Paste 'ls ~/PX4-Autopilot/msg'."
    fi
done

# ── Regenerate CMakeLists msg list ────────────────────────────
echo ""
echo "[cmake] regenerating CMakeLists.txt msg list..."
python3 - << 'PY'
import os, re
c = os.path.expanduser('~/Desktop/FYP/ros2_ws/src/px4_msgs/CMakeLists.txt')
d = os.path.expanduser('~/Desktop/FYP/ros2_ws/src/px4_msgs/msg')
s = open(c).read()
msgs = sorted(f for f in os.listdir(d) if f.endswith('.msg'))
blk = ('rosidl_generate_interfaces(${PROJECT_NAME}\n'
       + '\n'.join(f'    "msg/{m}"' for m in msgs)
       + '\n    DEPENDENCIES builtin_interfaces\n)')
s = re.sub(r'rosidl_generate_interfaces\([^)]*\)', blk, s, flags=re.DOTALL)
open(c, 'w').write(s)
print(f"  CMakeLists regenerated with {len(msgs)} msg files")
PY

# ── Rebuild px4_msgs ──────────────────────────────────────────
echo ""
echo "[build] rebuilding px4_msgs (2-3 min)..."
cd $ROS2_WS
rm -rf build/px4_msgs install/px4_msgs 2>/dev/null || true
colcon build --packages-select px4_msgs 2>&1 | tail -5

# ── Verify ────────────────────────────────────────────────────
echo ""
echo "[verify] checking types after rebuild..."
source $ROS2_WS/install/setup.bash 2>/dev/null || true
OK=1
for V in VehicleStatusV4 VehicleLocalPositionV1; do
    if ros2 interface show px4_msgs/msg/$V >/dev/null 2>&1; then
        echo "  px4_msgs/msg/$V  OK"
    else
        echo "  px4_msgs/msg/$V  STILL MISSING"
        OK=0
    fi
done

echo ""
if [ "$OK" = "1" ]; then
    echo "=============================================="
    echo "  DONE - px4_msgs now matches PX4 v1.16"
    echo "  Status/position will flow back to the nodes."
    echo "=============================================="
    echo ""
    echo "Now relaunch (fresh terminals pick up the new build):"
    echo "  pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f gz-sim"
    echo "  pkill -9 -f ruby; pkill -9 -f MicroXRCEAgent; sleep 3"
    echo "  bash ~/Desktop/FYP/launch_stage14.sh"
else
    echo "px4_msgs still missing versioned types. Paste the output of:"
    echo "  ls ~/PX4-Autopilot/msg"
    echo "  ls ~/PX4-Autopilot/msg/versioned 2>/dev/null"
    echo "so I can match the exact message names your PX4 build uses."
fi
echo ""
