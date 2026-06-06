#!/usr/bin/env bash
# ============================================================
# install_stage15_clean.sh
# ============================================================
# Run from inside the unpacked stage15_clean/ directory:
#   cd ~/Downloads/stage15_clean
#   chmod +x install_stage15_clean.sh
#   ./install_stage15_clean.sh
#
# What it does:
#   1. Backs up your current ~/Desktop/FYP/stage15/ to
#      ~/Desktop/FYP/_stage15_clean_backup_<timestamp>/
#   2. Wipes ~/Desktop/FYP/stage15/ and recreates it
#   3. Drops the 4 clean files in:
#         generate_field_stage15.py
#         drone_controller_stage15.py
#         mission_orchestrator_stage15.py
#         stage15_config.yaml
#   4. Drops launch_stage15.sh into ~/Desktop/FYP/
#   5. No ROS 2 package rebuild needed — controllers are run
#      directly as python3 scripts (matches Stage 14's pattern).
# ============================================================

set -e

HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
FYP_DIR="$HOME/Desktop/FYP"
STAGE15_DIR="$FYP_DIR/stage15"

if [[ ! -d "$FYP_DIR" ]]; then
  echo "ERROR: $FYP_DIR not found. Are you on the right machine?"
  exit 1
fi

# ── 1. Backup ───────────────────────────────────────────────
TS=$(date +"%Y%m%d_%H%M%S")
BACKUP="$FYP_DIR/_stage15_clean_backup_$TS"
mkdir -p "$BACKUP"

echo "── Stage 15 clean install ──────────────────────────────"
echo "Backup location: $BACKUP"

if [[ -d "$STAGE15_DIR" ]]; then
  echo "Backing up old $STAGE15_DIR …"
  cp -r "$STAGE15_DIR" "$BACKUP/stage15_old"
fi
if [[ -f "$FYP_DIR/launch_stage15.sh" ]]; then
  cp "$FYP_DIR/launch_stage15.sh" "$BACKUP/launch_stage15_old.sh"
fi

# Also note any redundant ROS 2 package nodes from earlier attempts
PKG_DIR="$FYP_DIR/ros2_ws/src/precision_pollination/precision_pollination"
if [[ -d "$PKG_DIR" ]]; then
  mkdir -p "$BACKUP/ros2_package_v15_nodes"
  for f in bloom_state_manager.py battery_monitor.py survey_node.py \
           map_merger.py visual_servoing_v15.py mission_logger_v15.py \
           swarm_coordinator_v15.py mtsp_optimizer.py; do
    if [[ -f "$PKG_DIR/$f" ]]; then
      cp "$PKG_DIR/$f" "$BACKUP/ros2_package_v15_nodes/$f"
    fi
  done
fi

# ── 2. Reset stage15 directory ──────────────────────────────
rm -rf "$STAGE15_DIR"
mkdir -p "$STAGE15_DIR"

# ── 3. Drop in clean files ──────────────────────────────────
cp "$HERE/generate_field_stage15.py"     "$STAGE15_DIR/"
cp "$HERE/drone_controller_stage15.py"   "$STAGE15_DIR/"
cp "$HERE/mission_orchestrator_stage15.py" "$STAGE15_DIR/"
cp "$HERE/stage15_config.yaml"           "$STAGE15_DIR/"

chmod +x "$STAGE15_DIR/generate_field_stage15.py"
chmod +x "$STAGE15_DIR/drone_controller_stage15.py"
chmod +x "$STAGE15_DIR/mission_orchestrator_stage15.py"

# ── 4. Drop launcher ────────────────────────────────────────
cp "$HERE/launch_stage15.sh" "$FYP_DIR/launch_stage15.sh"
# Fix the HERE reference inside the launcher to point at stage15/
sed -i 's|HERE="\$( cd "\$( dirname "\${BASH_SOURCE\[0\]}" )" && pwd )"|HERE="'"$STAGE15_DIR"'"|' \
  "$FYP_DIR/launch_stage15.sh"
chmod +x "$FYP_DIR/launch_stage15.sh"

# ── 5. Output log directory ─────────────────────────────────
mkdir -p "$FYP_DIR/mission_outputs/stage15"

# ── 6. Done ─────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  Stage 15 clean install complete"
echo "════════════════════════════════════════════════════════"
echo ""
echo "  Files installed:"
echo "    $STAGE15_DIR/generate_field_stage15.py"
echo "    $STAGE15_DIR/drone_controller_stage15.py"
echo "    $STAGE15_DIR/mission_orchestrator_stage15.py"
echo "    $STAGE15_DIR/stage15_config.yaml"
echo "    $FYP_DIR/launch_stage15.sh"
echo ""
echo "  Backup of old files: $BACKUP"
echo ""
echo "  To run:"
echo "    cd $FYP_DIR"
echo "    ./launch_stage15.sh"
echo ""
echo "  To watch the event stream from another terminal:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    ros2 topic echo /swarm/event"
echo ""
echo "  (The old precision_pollination ROS 2 package nodes are"
echo "   NOT used by this Stage 15 — they were the cause of half"
echo "   the breakage. You can delete them at your leisure, the"
echo "   backup is at $BACKUP/ros2_package_v15_nodes/ if needed.)"
echo "════════════════════════════════════════════════════════"
