"""
YOLOv8s Flower Readiness Training — Stage 10
=============================================
2-class detection model trained on the Sunflower_stages dataset.

Model choice: YOLOv8s (small) rather than Stage 6's YOLOv8n
because distinguishing bloom state requires finer texture
features than basic sunflower localisation.

Training strategy:
  - Full-image bounding boxes (each image = one sunflower)
  - Strong HSV augmentation: field lighting varies widely
  - Rotation augmentation: aerial view angle variation
  - Mosaic=0.5: helps with scale variation across growth stages
"""

from ultralytics import YOLO
from pathlib import Path
import sys
import json

DATASET_PATH = Path.home() / "Desktop/FYP/datasets/readiness_dataset/data.yaml"
BASE_MODEL   = "yolov8s.pt"
EPOCHS       = 80
IMAGE_SIZE   = 640
BATCH_SIZE   = 8
RUN_NAME     = "readiness_fyp_v1"
PATIENCE     = 20
OUTPUT_DIR   = Path.home() / "Desktop/FYP/ros2_ws"
TRAINING_DIR = Path.home() / "Desktop/FYP/fyp_training"

print("\n" + "="*65)
print("  YOLOV8s FLOWER READINESS TRAINING — FYP STAGE 10")
print("="*65)
print(f"\n  Dataset:  {DATASET_PATH}")
print(f"  Model:    {BASE_MODEL}")
print(f"  Classes:  bloom_ready, not_ready")
print(f"  Epochs:   {EPOCHS}  (early stop patience={PATIENCE})")
print()

if not DATASET_PATH.exists():
    print(f"ERROR: Dataset not found at {DATASET_PATH}")
    sys.exit(1)

train_imgs  = list((DATASET_PATH.parent / "train" / "images").glob("*.jpg"))
train_imgs += list((DATASET_PATH.parent / "train" / "images").glob("*.png"))
print(f"  Training images: {len(train_imgs)}")

import torch
device = "0" if torch.cuda.is_available() else "cpu"
print(f"  Device: {device}")
print()

model = YOLO(BASE_MODEL)
model.train(
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
    device    = device,
    flipud    = 0.3,
    fliplr    = 0.5,
    hsv_h     = 0.02,
    hsv_s     = 0.8,
    hsv_v     = 0.5,
    degrees   = 15.0,
    translate = 0.1,
    scale     = 0.5,
    mosaic    = 0.5,
)

print("\nRunning final evaluation on validation set...")
metrics   = model.val()
map50     = metrics.box.map50
map50_95  = metrics.box.map
precision = metrics.box.mp
recall    = metrics.box.mr

print("\n" + "="*65)
print("  READINESS MODEL — RESULTS")
print("="*65)
print(f"  mAP@0.5:      {map50:.4f}  (target: >= 0.80)")
print(f"  mAP@0.5:0.95: {map50_95:.4f}")
print(f"  Precision:    {precision:.4f}")
print(f"  Recall:       {recall:.4f}")

if map50 >= 0.80:
    print("\n  TARGET MET — mAP@0.5 >= 0.80")
elif map50 >= 0.65:
    print("\n  Below target. Try: more epochs or yolov8m.pt")
else:
    print("\n  Low mAP — check label quality and class balance")

best = Path(f"runs/detect/{RUN_NAME}/weights/best.pt")
if best.exists():
    import shutil
    dest = OUTPUT_DIR / "readiness_best.pt"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copy(best, dest)
    print(f"\n  Model saved: {dest}")
    print(f"  Plots:       runs/detect/{RUN_NAME}/")

results_data = {
    "stage": "Stage 10 — Flower Readiness Detection",
    "dataset": "Sunflower_stages (EarlyBloom, Healthy, MatureBud, YoungBud, Wilted)",
    "model": "YOLOv8s",
    "classes": {"0": "bloom_ready", "1": "not_ready"},
    "class_sources": {
        "bloom_ready": ["EarlyBloom", "Healthy"],
        "not_ready":   ["MatureBud", "YoungBud", "Wilted"]
    },
    "training": {
        "epochs": EPOCHS,
        "image_size": IMAGE_SIZE,
        "batch_size": BATCH_SIZE
    },
    "metrics": {
        "mAP_50":     round(float(map50), 4),
        "mAP_50_95":  round(float(map50_95), 4),
        "precision":  round(float(precision), 4),
        "recall":     round(float(recall), 4),
        "target_met": bool(map50 >= 0.80)
    }
}
results_path = Path(TRAINING_DIR) / "readiness_results.json"
results_path.parent.mkdir(parents=True, exist_ok=True)
results_path.write_text(json.dumps(results_data, indent=2))
print(f"  Results JSON: {results_path}")
print("\n" + "="*65 + "\n")
