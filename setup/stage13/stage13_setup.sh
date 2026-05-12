#!/bin/bash
# ============================================================
# STAGE 13 — Setup (run ONCE before first launch)
# FYP: Autonomous Swarm Drone Pollination — DaaS Dashboard
#
# What this does:
#   1. Installs Flask
#   2. Creates ~/Desktop/FYP/daas/ folder structure
#   3. Copies all dashboard files into it
#   4. Places launch_stage13.sh at ~/Desktop/FYP/
#
# Run from the folder containing this script:
#   bash stage13_setup.sh
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
DAAS_DIR=$FYP_DIR/daas
SRC="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 13 — DaaS Dashboard Setup                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# 1. Flask
echo "[1/3] Installing Flask..."
pip3 install --break-system-packages --quiet flask
echo "✓ Flask ready"

# 2. Directory structure
echo ""
echo "[2/3] Creating $DAAS_DIR..."
mkdir -p "$DAAS_DIR/templates"
mkdir -p "$DAAS_DIR/static"

# 3. Copy files
echo "[3/3] Copying files..."
cp "$SRC/app.py"                    "$DAAS_DIR/app.py"
cp "$SRC/ros_bridge.py"             "$DAAS_DIR/ros_bridge.py"
cp "$SRC/mission_state.py"          "$DAAS_DIR/mission_state.py"
cp "$SRC/templates/index.html"      "$DAAS_DIR/templates/index.html"
cp "$SRC/static/style.css"          "$DAAS_DIR/static/style.css"
cp "$SRC/static/dashboard.js"       "$DAAS_DIR/static/dashboard.js"
cp "$SRC/launch_stage13.sh"         "$FYP_DIR/launch_stage13.sh"
chmod +x "$FYP_DIR/launch_stage13.sh"

echo ""
echo "✓  app.py"
echo "✓  ros_bridge.py"
echo "✓  mission_state.py"
echo "✓  templates/index.html"
echo "✓  static/style.css"
echo "✓  static/dashboard.js"
echo "✓  launch_stage13.sh → $FYP_DIR/"
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Setup complete                                     ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  To launch:                                         ║"
echo "║    bash ~/Desktop/FYP/launch_stage13.sh             ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
