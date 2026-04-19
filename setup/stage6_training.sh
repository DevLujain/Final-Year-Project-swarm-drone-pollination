#!/bin/bash
# ============================================================
# STAGE 6 — YOLOv8 Training Pipeline (Week 8)
# FYP: Autonomous Swarm Drone Pollination
#
# WHAT THIS SCRIPT DOES:
#   1. Fixes the dataset data.yaml
#   2. Splits dataset into train/valid/test
#   3. Trains YOLOv8n on your sunflower dataset
#   4. Evaluates the model and prints mAP@0.5
#   5. Copies best model to ros2_ws for the detector node
#
# RUN: bash stage6_training.sh
# TIME: ~30-90 minutes depending on GPU/CPU
# ============================================================

set -e

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 6 — YOLOv8 Training Pipeline (Week 8)        ║"
echo "║  FYP: Swarm Drone Pollination                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

DATASET_DIR=~/Desktop/FYP/datasets/'My First Project.yolov8-obb'
TRAINING_DIR=~/fyp_training
OUTPUT_DIR=~/ros2_ws

# ── 1. Fix data.yaml ──────────────────────────────────────────
echo "[1/5] Fixing dataset data.yaml..."

cat > "$DATASET_DIR/data.yaml" << 'YAMLEOF'
train: train/images
val: valid/images
test: test/images

nc: 2
names:
  0: flower
  1: sunflower
YAMLEOF

echo "✓ data.yaml fixed (nc=2, flower + sunflower classes)"

# ── 2. Check dataset structure ────────────────────────────────
echo ""
echo "[2/5] Checking dataset structure..."

TRAIN_IMAGES=$(ls "$DATASET_DIR/train/images" 2>/dev/null | wc -l)
TRAIN_LABELS=$(ls "$DATASET_DIR/train/labels" 2>/dev/null | wc -l)

echo "  Train images: $TRAIN_IMAGES"
echo "  Train labels: $TRAIN_LABELS"

if [ "$TRAIN_IMAGES" -eq 0 ]; then
    echo "  ERROR: No training images found!"
    echo "  Check: $DATASET_DIR/train/images/"
    exit 1
fi

# Check if valid folder exists, create if not
if [ ! -d "$DATASET_DIR/valid/images" ]; then
    echo "  Creating valid/ split from train/ (20% validation)..."
    mkdir -p "$DATASET_DIR/valid/images"
    mkdir -p "$DATASET_DIR/valid/labels"

    # Move 20% of images to valid
    TOTAL=$TRAIN_IMAGES
    VALID_COUNT=$((TOTAL / 5))

    ls "$DATASET_DIR/train/images" | shuf | head -$VALID_COUNT | while read f; do
        mv "$DATASET_DIR/train/images/$f" "$DATASET_DIR/valid/images/"
        LABEL="${f%.*}.txt"
        [ -f "$DATASET_DIR/train/labels/$LABEL" ] && \
            mv "$DATASET_DIR/train/labels/$LABEL" "$DATASET_DIR/valid/labels/"
    done
    echo "  ✓ Validation split created ($VALID_COUNT images)"
fi

VALID_IMAGES=$(ls "$DATASET_DIR/valid/images" 2>/dev/null | wc -l)
echo "  Valid images: $VALID_IMAGES"
echo "✓ Dataset structure verified"

# ── 3. Write training script ──────────────────────────────────
echo ""
echo "[3/5] Writing and running YOLOv8 training script..."

mkdir -p $TRAINING_DIR

cat > $TRAINING_DIR/run_training.py << 'TRAINEOF'
"""
YOLOv8 Sunflower Training — Week 8
====================================
Trains YOLOv8n on the Roboflow sunflower dataset.
Target: mAP@0.5 >= 0.85
"""

from ultralytics import YOLO
from pathlib import Path
import sys
import os

# Configuration
DATASET_PATH = Path.home() / "Desktop/FYP/datasets/My First Project.yolov8-obb/data.yaml"
BASE_MODEL   = "yolov8n.pt"
EPOCHS       = 100
IMAGE_SIZE   = 640
BATCH_SIZE   = 8
RUN_NAME     = "sunflower_fyp_v1"
PATIENCE     = 20
OUTPUT_DIR   = Path.home() / "ros2_ws"

print("\n" + "="*60)
print("  YOLOV8 SUNFLOWER TRAINING — FYP WEEK 8")
print("="*60 + "\n")

if not DATASET_PATH.exists():
    print(f"ERROR: Dataset not found at {DATASET_PATH}")
    sys.exit(1)

print(f"Dataset:    {DATASET_PATH}")
print(f"Model:      {BASE_MODEL}")
print(f"Epochs:     {EPOCHS}")
print(f"Image size: {IMAGE_SIZE}px")
print(f"Batch:      {BATCH_SIZE}")
print(f"Run name:   {RUN_NAME}\n")

# Load base model
print("Loading YOLOv8n base model...")
model = YOLO(BASE_MODEL)

# Train
print("Starting training...\n")
results = model.train(
    data      = str(DATASET_PATH),
    epochs    = EPOCHS,
    imgsz     = IMAGE_SIZE,
    batch     = BATCH_SIZE,
    name      = RUN_NAME,
    patience  = PATIENCE,
    save      = True,
    plots     = True,
    val       = True,
    verbose   = True,
    device    = "cpu",       # change to '0' if you have GPU
    # Augmentation for field conditions
    flipud    = 0.3,
    fliplr    = 0.5,
    hsv_h     = 0.015,
    hsv_s     = 0.7,
    hsv_v     = 0.4,
    degrees   = 15.0,        # rotation for aerial view variation
    translate = 0.1,
    scale     = 0.5,
)

# Evaluate
print("\nRunning final evaluation...")
metrics = model.val()

map50    = metrics.box.map50
map50_95 = metrics.box.map
precision = metrics.box.mp
recall   = metrics.box.mr

print("\n" + "="*60)
print("  TRAINING RESULTS")
print("="*60)
print(f"  mAP@0.5:       {map50:.4f}  (target: >= 0.85)")
print(f"  mAP@0.5:0.95:  {map50_95:.4f}")
print(f"  Precision:     {precision:.4f}")
print(f"  Recall:        {recall:.4f}")

if map50 >= 0.85:
    print("\n  ✅ TARGET MET — mAP@0.5 >= 0.85")
elif map50 >= 0.70:
    print("\n  ⚠️  Below target. Try more epochs or yolov8s.pt")
else:
    print("\n  ❌ Low mAP. Check dataset labels.")

# Copy best model to ROS2 workspace
best = Path(f"runs/detect/{RUN_NAME}/weights/best.pt")
if best.exists():
    import shutil
    dest = OUTPUT_DIR / "sunflower_best.pt"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy(best, dest)
    print(f"\n  Model saved to: {dest}")
    print(f"  Training graphs: runs/detect/{RUN_NAME}/")
    print(f"\n  To use in ROS 2 detector:")
    print(f"    ros2 run precision_pollination yolov8_detector")
    print(f"    --ros-args -p model_path:={dest}")

print("\n" + "="*60 + "\n")
TRAINEOF

echo "✓ Training script written to $TRAINING_DIR/run_training.py"

# ── 4. Run training ───────────────────────────────────────────
echo ""
echo "[4/5] Starting YOLOv8 training..."
echo "      This will take 30-90 minutes on CPU."
echo "      Watch the mAP@0.5 column — target is > 0.85"
echo ""

cd $TRAINING_DIR
python3 run_training.py

# ── 5. Verify model saved ─────────────────────────────────────
echo ""
echo "[5/5] Verifying trained model..."

if [ -f ~/ros2_ws/sunflower_best.pt ]; then
    echo "✓ Model saved: ~/ros2_ws/sunflower_best.pt"
    ls -lh ~/ros2_ws/sunflower_best.pt
else
    echo "⚠ Model not found — check training output above"
fi

# ── Done ─────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  STAGE 6 COMPLETE — Week 8 Done                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "RESULTS SAVED TO:"
echo "  ~/fyp_training/runs/detect/sunflower_fyp_v1/"
echo "  ├── weights/best.pt      ← your trained model"
echo "  ├── results.png          ← training curves"
echo "  └── confusion_matrix.png ← class confusion"
echo ""
echo "MODEL DEPLOYED TO:"
echo "  ~/ros2_ws/sunflower_best.pt"
echo ""
echo "TO TEST THE TRAINED MODEL:"
echo "  ros2 run precision_pollination yolov8_detector \\"
echo "    --ros-args -p model_path:=~/ros2_ws/sunflower_best.pt"
echo ""
echo "NEXT: Run stage7_integration.sh for full system integration"
echo ""
