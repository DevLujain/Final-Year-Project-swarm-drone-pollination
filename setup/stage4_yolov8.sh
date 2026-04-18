#!/bin/bash
# ============================================================
# STAGE 4 — YOLOv8 + Python ML stack
# This is independent — run it any time after Stage 1
# Takes ~10 minutes
# ============================================================
set -e

echo "=================================================="
echo " STAGE 4: YOLOv8 + Python stack"
echo "=================================================="

# Upgrade pip
python3 -m pip install --upgrade pip

# Core ML packages
pip3 install \
  ultralytics \
  torch \
  torchvision \
  opencv-python \
  numpy \
  matplotlib \
  pandas \
  scikit-learn \
  Pillow \
  tqdm \
  PyYAML \
  requests

echo "✓ ML packages installed"

# ROS 2 Python bindings
pip3 install \
  rclpy \
  transforms3d

# Required for porting supervisor's yolov8_ros node
pip3 install \
  sensor_msgs_py

echo "✓ ROS 2 Python packages installed"

# ── Download the Zenodo Flower Dataset ──
# This is the dataset you'll train YOLOv8 on
echo ""
echo "Downloading Zenodo Flower Detection Dataset..."
cd ~
mkdir -p ~/datasets/flower_detection
cd ~/datasets/flower_detection

# Dataset DOI: 10.5281/zenodo.7560779
# Direct download URL for the dataset zip
wget -O flower_dataset.zip \
  "https://zenodo.org/record/7560779/files/flower_detection_dataset.zip?download=1" \
  || echo "⚠ Auto-download failed. Download manually from:"
echo "  https://zenodo.org/record/7560779"
echo "  → Download 'flower_detection_dataset.zip'"
echo "  → Extract to ~/datasets/flower_detection/"

# ── Verify YOLOv8 works ──
echo ""
echo "Testing YOLOv8..."
python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
print('✓ YOLOv8n loaded successfully')
print('  Model info:')
model.info()
"

echo ""
echo "✓ Stage 4 complete."
echo ""
echo "NEXT STEP — Train YOLOv8 on the flower dataset:"
echo ""
echo "  1. Check your dataset structure:"
echo "     ~/datasets/flower_detection/"
echo "     ├── images/"
echo "     │   ├── train/   (126 images)"
echo "     │   └── val/     (40 images)"
echo "     └── labels/"
echo "         ├── train/   (YOLO format .txt files)"
echo "         └── val/"
echo ""
echo "  2. Create data.yaml (see stage5_workspace.sh)"
echo "  3. Run first training:"
echo "     yolo train model=yolov8n.pt data=~/datasets/flower_detection/data.yaml epochs=50 imgsz=640"
echo ""
echo "Then run stage5_workspace.sh"
