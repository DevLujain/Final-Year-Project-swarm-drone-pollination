#!/bin/bash
# ============================================================
# STAGE 12 — Setup Script (run ONCE to install and register)
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS DOES:
#   1. Installs Python dependencies (matplotlib, scipy, pandas)
#   2. Copies the two Stage 12 nodes into your ROS 2 package
#   3. Registers them in setup.py
#   4. Rebuilds the workspace
#
# RUN: bash stage12_setup.sh
# TIME: ~2 minutes
#
# AFTER THIS: use launch_stage12.sh to run the nodes
# ============================================================

set -e

FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 12 — Setup                                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Install Python dependencies ───────────────────────────
echo "[1/4] Installing Python dependencies..."
pip3 install --break-system-packages --quiet matplotlib scipy pandas seaborn
echo "✓ matplotlib, scipy, pandas, seaborn installed"

# ── 2. Copy nodes into ROS 2 package ─────────────────────────
echo ""
echo "[2/4] Copying Stage 12 nodes into ROS 2 package..."

cp "$SCRIPT_DIR/field_health_mapper.py" "$NODES_DIR/field_health_mapper.py"
cp "$SCRIPT_DIR/kpi_logger.py"          "$NODES_DIR/kpi_logger.py"

echo "✓ field_health_mapper.py → $NODES_DIR"
echo "✓ kpi_logger.py          → $NODES_DIR"

# ── 3. Register in setup.py ───────────────────────────────────
echo ""
echo "[3/4] Checking setup.py for node registration..."

SETUP_PY="$ROS2_WS/src/precision_pollination/setup.py"

# Check if already registered, if not — add them
if ! grep -q "field_health_mapper" "$SETUP_PY"; then
    # Find the console_scripts line and add after the last entry
    # This uses Python to safely edit the file
    python3 - "$SETUP_PY" << 'PYEOF'
import re, sys

path = sys.argv[1]
with open(path, 'r') as f:
    content = f.read()

# Lines to add
new_entries = [
    "            'field_health_mapper = precision_pollination.field_health_mapper:main',",
    "            'kpi_logger = precision_pollination.kpi_logger:main',",
]

for entry in new_entries:
    node_name = entry.strip().split('=')[0].strip().strip("'")
    if node_name not in content:
        # Insert before the closing bracket of console_scripts
        content = content.replace(
            "        ],\n    },",
            f"{entry}\n        ],\n    }},"
        )
        print(f"  Registered: {node_name}")
    else:
        print(f"  Already registered: {node_name}")

with open(path, 'w') as f:
    f.write(content)
PYEOF
else
    echo "  Nodes already registered in setup.py"
fi

# ── 4. Rebuild workspace ──────────────────────────────────────
echo ""
echo "[4/4] Rebuilding ROS 2 workspace..."
source /opt/ros/jazzy/setup.bash
cd $ROS2_WS
colcon build --symlink-install --packages-select precision_pollination 2>&1 | tail -5

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 12 SETUP COMPLETE                            ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Nodes registered:                                  ║"
echo "║    ros2 run precision_pollination field_health_mapper ║"
echo "║    ros2 run precision_pollination kpi_logger         ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  TO RUN STAGE 12:                                   ║"
echo "║    1. Start mission: bash launch_stage11.sh          ║"
echo "║    2. Then run:      bash launch_stage12.sh          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
