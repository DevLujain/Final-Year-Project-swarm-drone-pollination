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
