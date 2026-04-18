#!/bin/bash
# ============================================================
# STAGE 1 — System Setup for Ubuntu 24.04 (Noble Numbat)
# FYP: Swarm Drone Pollination
#
# What this script does:
#   1. Updates the system
#   2. Installs all system-level dependencies
#   3. Installs PX4 Autopilot (latest, v1.15+)
#   4. Installs Gazebo Harmonic (the correct version for Ubuntu 24.04)
#   5. Installs micro-XRCE-DDS agent (PX4 <-> ROS 2 bridge)
#
# NOTE: Ubuntu 24.04 uses Gazebo Harmonic (gz-harmonic), NOT Gazebo Classic.
#       The old "gazebo-classic" does not work on Ubuntu 24.04.
#       PX4 v1.15+ supports Gazebo Harmonic natively.
#
# RUN: sudo bash stage1_system_ubuntu24.sh
# TIME: ~20-30 minutes
# ============================================================

set -e  # stop on any error

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 1 — Ubuntu 24.04 System Setup                ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# ── 1. Update system ──────────────────────────────────────────
echo "[1/7] Updating system packages..."
apt-get update -y
apt-get upgrade -y
apt-get install -y \
    curl wget git build-essential cmake ninja-build \
    python3 python3-pip python3-dev python3-venv \
    software-properties-common apt-transport-https \
    lsb-release gnupg2 ca-certificates \
    libssl-dev libffi-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libfuse2 libgstreamer-plugins-base1.0-dev \
    v4l-utils libc6-i386 lib32gcc-s1 lib32stdc++6

echo "✓ System packages installed"

# ── 2. Install Python tools ───────────────────────────────────
echo ""
echo "[2/7] Installing Python tools..."
pip3 install --break-system-packages \
    ultralytics \
    opencv-python \
    numpy \
    matplotlib \
    pandas \
    scikit-learn \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools \
    toml \
    future \
    jinja2 \
    jsonschema

echo "✓ Python tools installed"

# ── 3. Install Gazebo Harmonic ────────────────────────────────
# Gazebo Harmonic is the correct version for Ubuntu 24.04.
# It replaces Gazebo Classic (gazebo11) which is NOT available on 24.04.
echo ""
echo "[3/7] Installing Gazebo Harmonic..."
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt-get update -y
apt-get install -y gz-harmonic

echo "✓ Gazebo Harmonic installed"
echo "  Test later with: gz sim"

# ── 4. Clone PX4 Autopilot ────────────────────────────────────
# PX4 v1.15 is the first version with stable Gazebo Harmonic support.
# We clone the latest stable release (not main, which can be unstable).
echo ""
echo "[4/7] Cloning PX4 Autopilot (latest stable)..."

PX4_DIR="$HOME/PX4-Autopilot"

if [ -d "$PX4_DIR" ]; then
    echo "  PX4 directory already exists at $PX4_DIR — pulling latest..."
    cd "$PX4_DIR"
    git pull
else
    cd "$HOME"
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd "$PX4_DIR"
fi

# Run PX4's own dependency installer (handles Ubuntu 24.04 correctly)
echo "  Running PX4 ubuntu.sh dependency installer..."
bash Tools/setup/ubuntu.sh --no-nuttx

echo "✓ PX4 cloned and dependencies installed"

# ── 5. Build PX4 with Gazebo Harmonic ────────────────────────
# This compiles PX4 SITL and links it to Gazebo Harmonic.
# First build takes 5-15 minutes.
echo ""
echo "[5/7] Building PX4 SITL with Gazebo Harmonic..."
echo "  This will take 5-15 minutes on first run..."
cd "$PX4_DIR"
make px4_sitl gz_x500

echo "✓ PX4 SITL built successfully"
echo "  The x500 is your simulated quadcopter model"

# ── 6. Install micro-XRCE-DDS agent ──────────────────────────
# This is the bridge that connects PX4 to ROS 2.
# PX4 publishes its data (position, velocity, etc.) using the uXRCE protocol,
# and this agent translates it into ROS 2 topics.
echo ""
echo "[6/7] Installing micro-XRCE-DDS agent (PX4 <-> ROS 2 bridge)..."

cd "$HOME"

if [ -d "Micro-XRCE-DDS-Agent" ]; then
    echo "  Already cloned, rebuilding..."
else
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi

cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
make -j$(nproc)
make install
ldconfig /usr/local/lib/

echo "✓ micro-XRCE-DDS agent installed"

# ── 7. Setup environment variables ───────────────────────────
echo ""
echo "[7/7] Setting up environment variables..."

# Add PX4 tools to path if not already there
grep -qxF 'export PX4_DIR=$HOME/PX4-Autopilot' ~/.bashrc || \
    echo 'export PX4_DIR=$HOME/PX4-Autopilot' >> ~/.bashrc

grep -qxF 'export GZ_VERSION=harmonic' ~/.bashrc || \
    echo 'export GZ_VERSION=harmonic' >> ~/.bashrc

echo "✓ Environment variables added to ~/.bashrc"

# ── Done ──────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 1 COMPLETE                                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "WHAT WAS INSTALLED:"
echo "  • Gazebo Harmonic  (gz sim)               — 3D simulator"
echo "  • PX4 Autopilot    (~/PX4-Autopilot/)     — flight controller"
echo "  • PX4 SITL         (make px4_sitl gz_x500) — software simulation"
echo "  • XRCE-DDS agent   (MicroXRCEAgent)       — PX4 <-> ROS 2 bridge"
echo "  • YOLOv8 (Python)  (ultralytics)          — computer vision"
echo ""
echo "QUICK TEST (open 2 terminals after: source ~/.bashrc):"
echo ""
echo "  Terminal 1 — launch simulation:"
echo "    cd ~/PX4-Autopilot"
echo "    make px4_sitl gz_x500"
echo ""
echo "  Terminal 2 — start the ROS 2 bridge:"
echo "    MicroXRCEAgent udp4 -p 8888"
echo ""
echo "  You should see the drone appear in the Gazebo window."
echo ""
echo "NEXT: Run stage2_ros2_ubuntu24.sh to install ROS 2 Jazzy"
echo "      (Ubuntu 24.04 uses ROS 2 Jazzy, NOT Humble)"
echo ""
