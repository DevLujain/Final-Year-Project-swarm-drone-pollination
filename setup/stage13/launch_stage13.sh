#!/bin/bash
# ============================================================
# LAUNCH STAGE 13 — DaaS Dashboard
# FYP: Autonomous Swarm Drone Pollination
#
# Optional run order:
#   Terminal A: bash ~/Desktop/FYP/launch_stage11.sh   ← swarm
#   Terminal B: bash ~/Desktop/FYP/launch_stage12.sh   ← health map
#   Terminal C: bash ~/Desktop/FYP/launch_stage13.sh   ← this
#
# Works standalone — press Start in the browser for the
# built-in simulation (no swarm required).
#
# Open: http://localhost:5000
# Stop: Ctrl+C
# ============================================================

FYP_DIR=~/Desktop/FYP
DAAS_DIR=$FYP_DIR/daas

source /opt/ros/jazzy/setup.bash
source "$FYP_DIR/ros2_ws/install/setup.bash" 2>/dev/null || true

if [ ! -f "$DAAS_DIR/app.py" ]; then
    echo "ERROR: DaaS files not found."
    echo "Run first: bash stage13_setup.sh"
    exit 1
fi

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 13 — DaaS Dashboard                         ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Open in browser:  http://localhost:5000            ║"
echo "║  Press Start in the browser to begin simulation     ║"
echo "║  Ctrl+C to stop                                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

(sleep 2 && xdg-open http://localhost:5000 2>/dev/null || true) &

cd "$DAAS_DIR"
python3 app.py
