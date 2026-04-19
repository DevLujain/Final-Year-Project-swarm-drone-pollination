"""
YOLOv8 Sunflower Training Script
==================================
Run this script to train YOLOv8 on your sunflower dataset.

BEFORE RUNNING:
  1. Download a sunflower dataset from:
       https://universe.roboflow.com/search?q=sunflower
     Export in "YOLOv8" format. This gives you a folder with:
       data.yaml, train/, valid/, test/ subfolders

  2. Edit DATASET_PATH below to point to your data.yaml file

  3. Run: python3 train_sunflower.py

WHAT HAPPENS:
  - YOLOv8n (nano, fastest) trains for 100 epochs
  - Results saved to runs/detect/sunflower_v1/
  - Best model weights at: runs/detect/sunflower_v1/weights/best.pt
  - Copy best.pt to use in your ROS 2 node
"""

from ultralytics import YOLO
import os

# ── Configuration ──────────────────────────────────────────────
DATASET_PATH = os.path.expanduser('~/fyp_training/dataset/data.yaml')
MODEL_SIZE   = 'yolov8n.pt'   # n=nano (fastest), s=small, m=medium
EPOCHS       = 100
IMAGE_SIZE   = 640
BATCH_SIZE   = 8              # reduce to 4 if you run out of RAM
RUN_NAME     = 'sunflower_v1'

# ── Train ──────────────────────────────────────────────────────
print(f"\nStarting YOLOv8 training:")
print(f"  Dataset:    {DATASET_PATH}")
print(f"  Model:      {MODEL_SIZE}")
print(f"  Epochs:     {EPOCHS}")
print(f"  Image size: {IMAGE_SIZE}px")
print(f"  Batch size: {BATCH_SIZE}")
print("")

if not os.path.exists(DATASET_PATH):
    print("ERROR: Dataset not found at", DATASET_PATH)
    print("")
    print("Download a sunflower dataset from:")
    print("  https://universe.roboflow.com/search?q=sunflower+head")
    print("Export in YOLOv8 format and unzip into ~/fyp_training/dataset/")
    exit(1)

model = YOLO(MODEL_SIZE)

results = model.train(
    data=DATASET_PATH,
    epochs=EPOCHS,
    imgsz=IMAGE_SIZE,
    batch=BATCH_SIZE,
    name=RUN_NAME,
    patience=20,          # stop early if no improvement for 20 epochs
    save=True,
    plots=True,           # saves training curve graphs
    val=True
)

# ── Results ────────────────────────────────────────────────────
print("\n" + "="*50)
print("TRAINING COMPLETE")
print("="*50)
print(f"Best model:  runs/detect/{RUN_NAME}/weights/best.pt")
print(f"Results:     runs/detect/{RUN_NAME}/")
print("")
print("Evaluation metrics:")
metrics = model.val()
print(f"  mAP@0.5:       {metrics.box.map50:.3f}")
print(f"  mAP@0.5:0.95:  {metrics.box.map:.3f}")
print(f"  Precision:     {metrics.box.mp:.3f}")
print(f"  Recall:        {metrics.box.mr:.3f}")
print("")
print("To use in your ROS 2 node:")
print(f"  cp runs/detect/{RUN_NAME}/weights/best.pt ~/ros2_ws/")
print(f"  # Then in your launch file, set model_path:=~/ros2_ws/best.pt")
