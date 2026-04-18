#!/usr/bin/env python3
"""
=============================================================================
SUNFLOWER DETECTION — YOLOv8 TRAINING SCRIPT
FYP: Autonomous Swarm Drone Pollination System
=============================================================================

WHAT THIS SCRIPT DOES:
  This script fine-tunes a YOLOv8 model to detect sunflower heads in images.
  Fine-tuning means: we start from a model that already knows how to detect
  objects in general (trained on millions of images), and we teach it to
  specifically recognise sunflower heads using our smaller dataset.

  Think of it like this: instead of teaching someone what a "flower" is from
  scratch, you hire someone who already knows what objects look like and just
  show them 500 photos of sunflowers until they can spot one reliably.

HOW TO RUN:
  python3 train_sunflower_fyp.py

BEFORE RUNNING:
  1. Download a sunflower dataset (YOLOv8 format) from Roboflow or Zenodo
  2. Unzip it into: ~/fyp_training/dataset/
  3. Make sure the folder contains: data.yaml, train/, valid/

WHAT YOU GET AFTER TRAINING:
  - runs/detect/sunflower_fyp/weights/best.pt   ← your trained model
  - runs/detect/sunflower_fyp/results.png        ← training curve graph
  - Console output with mAP, precision, recall scores
=============================================================================
"""

# =============================================================================
# BLOCK 1 — IMPORTS
# =============================================================================
# These are Python libraries we need.
# 'ultralytics' is the company that made YOLOv8 — this is their official library.
# 'os' and 'pathlib' let us work with file paths (create folders, check if files exist).
# 'sys' lets us exit the script early if something goes wrong.
# =============================================================================

from ultralytics import YOLO   # The YOLOv8 library — does all the heavy lifting
from pathlib import Path        # Clean way to work with file/folder paths
import os                       # Basic operating system interactions
import sys                      # Lets us stop the script with sys.exit()


# =============================================================================
# BLOCK 2 — CONFIGURATION
# =============================================================================
# All the settings for your training run are defined here in one place.
# You should only need to change DATASET_PATH if your files are in a
# different location. Everything else is tuned for your FYP hardware.
# =============================================================================

# Where your dataset lives — must contain data.yaml, train/, valid/
DATASET_PATH = Path.home() / "fyp_training" / "dataset" / "data.yaml"

# Which YOLOv8 model SIZE to start from.
# 'yolov8n.pt' = nano (smallest, fastest — best for a drone's onboard computer)
# 'yolov8s.pt' = small (a bit more accurate, a bit slower)
# 'yolov8m.pt' = medium (more accurate, needs more RAM)
# For a drone running on a Raspberry Pi or Jetson Nano, stick with 'n' or 's'.
BASE_MODEL = "yolov8n.pt"

# How many times to go through the entire training dataset.
# Each full pass = 1 epoch. More epochs = more learning, but also more time.
# 100 is a good starting point. If your mAP@0.5 is still improving at epoch 100,
# you can extend to 150 or 200.
EPOCHS = 100

# The size (in pixels) each image gets resized to before training.
# 640x640 is the standard. Lower = faster but less accurate. Higher = slower.
IMAGE_SIZE = 640

# How many images to process at once before updating the model's weights.
# Larger batch = uses more RAM. 8 is safe for most laptops.
# If you get an out-of-memory error, reduce to 4.
BATCH_SIZE = 8

# A name for this training run — results will be saved under this name.
RUN_NAME = "sunflower_fyp"

# Stop early if the model hasn't improved for this many epochs in a row.
# Saves time — if mAP stops going up for 20 epochs, training is probably done.
PATIENCE = 20

# Where to save the final model weights for use in ROS 2
OUTPUT_DIR = Path.home() / "ros2_ws"


# =============================================================================
# BLOCK 3 — DATASET CHECK
# =============================================================================
# Before starting training (which takes 30-60 minutes), we check that the
# dataset actually exists. There is nothing worse than waiting an hour and
# then finding out the file path was wrong.
# =============================================================================

print("\n" + "="*65)
print("  SUNFLOWER YOLOV8 TRAINING — FYP DRONE POLLINATION")
print("="*65 + "\n")

print("Checking dataset...")

if not DATASET_PATH.exists():
    # Dataset not found — print helpful instructions and stop
    print(f"\n  ERROR: Dataset not found at:\n  {DATASET_PATH}\n")
    print("  How to fix this:\n")
    print("  OPTION A — Roboflow (easiest):")
    print("    1. Go to: https://universe.roboflow.com/search?q=sunflower+head")
    print("    2. Pick any dataset with 300+ images")
    print("    3. Click Export → Format: YOLOv8 → Download ZIP")
    print("    4. Unzip into: ~/fyp_training/dataset/")
    print("    5. Make sure data.yaml is at: ~/fyp_training/dataset/data.yaml\n")
    print("  OPTION B — Zenodo (from your literature review):")
    print("    1. Go to: https://zenodo.org/record/7560779")
    print("    2. Download the zip file")
    print("    3. Unzip into: ~/fyp_training/dataset/\n")
    sys.exit(1)

# Read data.yaml to show the user what dataset they have
with open(DATASET_PATH, 'r') as f:
    data_yaml_content = f.read()

print(f"  Dataset found: {DATASET_PATH}")
print(f"\n  Dataset contents (data.yaml):")
for line in data_yaml_content.splitlines():
    print(f"    {line}")

print(f"\n  Training settings:")
print(f"    Base model:   {BASE_MODEL}  (YOLOv8 nano — fast, drone-compatible)")
print(f"    Epochs:       {EPOCHS}")
print(f"    Image size:   {IMAGE_SIZE} x {IMAGE_SIZE} pixels")
print(f"    Batch size:   {BATCH_SIZE}")
print(f"    Run name:     {RUN_NAME}")
print(f"    Early stop:   after {PATIENCE} epochs without improvement")
print()


# =============================================================================
# BLOCK 4 — LOAD THE BASE MODEL
# =============================================================================
# Here we load YOLOv8n — the pre-trained model.
#
# What "pre-trained" means:
#   YOLOv8n was already trained by Ultralytics on the COCO dataset, which has
#   120,000 images of 80 different object types (cars, people, dogs, etc.).
#   That training took weeks on expensive GPUs. We are borrowing the result
#   of that work and just teaching it one more thing: sunflower heads.
#
# The file 'yolov8n.pt' gets automatically downloaded (~6MB) if it's not
# already on your computer.
# =============================================================================

print("Loading base model (yolov8n.pt)...")
print("  If this is your first run, it will download ~6MB automatically.\n")

model = YOLO(BASE_MODEL)
print(f"  Model loaded. Parameters: {sum(p.numel() for p in model.parameters()):,}")


# =============================================================================
# BLOCK 5 — TRAINING
# =============================================================================
# This is the main training call. It:
#   1. Reads all images from your dataset's train/ folder
#   2. For each batch of BATCH_SIZE images, makes predictions
#   3. Compares predictions to ground truth labels (the .txt annotation files)
#   4. Calculates the error (called "loss")
#   5. Adjusts the model weights slightly to reduce that error
#   6. Repeats for every batch, every epoch
#
# After each epoch, it validates on the valid/ folder and prints metrics.
# The best model (highest mAP on validation) is automatically saved.
#
# WHAT THE METRICS MEAN:
#   box_loss:  How accurately the model draws boxes around flowers
#   cls_loss:  How accurately it identifies what's in the box (sunflower vs background)
#   mAP@0.5:  Main accuracy metric. 0.85+ is good. 0.90+ is excellent.
#   Precision: Of all boxes it drew, what % were actually flowers? (avoid false positives)
#   Recall:    Of all real flowers, what % did it find? (avoid misses)
#
# For a pollination drone, RECALL is more important than precision — 
# it's better to investigate a false positive than to miss a real flower.
# =============================================================================

print("Starting training...\n")
print("  You will see a progress bar for each epoch.")
print("  Training takes approximately 30-90 minutes on a CPU.")
print("  If you have a GPU it will be 5-10x faster.\n")
print("-" * 65)

results = model.train(
    data    = str(DATASET_PATH),  # path to your data.yaml
    epochs  = EPOCHS,             # total training rounds
    imgsz   = IMAGE_SIZE,         # resize all images to this
    batch   = BATCH_SIZE,         # images per update step
    name    = RUN_NAME,           # folder name for saving results
    patience= PATIENCE,           # early stopping
    save    = True,               # save best + last model weights
    plots   = True,               # save training curve graphs
    val     = True,               # validate after each epoch
    verbose = True,               # print detailed output
    device  = "cpu",              # use 'cpu' or '0' for GPU (if available)
    # augmentation — randomly flips, rotates, adjusts brightness during training
    # This makes the model more robust to different lighting and drone angles
    flipud  = 0.3,                # 30% chance to flip image upside down
    fliplr  = 0.5,                # 50% chance to flip image left-right
    hsv_h   = 0.015,              # slight random hue shift (lighting variation)
    hsv_s   = 0.7,                # random saturation shift
    hsv_v   = 0.4,                # random brightness shift
)

print("-" * 65)


# =============================================================================
# BLOCK 6 — EVALUATE THE TRAINED MODEL
# =============================================================================
# After training, we run a final evaluation on the validation set.
# This gives us the clean, final metrics to report in our FYP.
#
# We run model.val() which is a separate evaluation pass —
# it doesn't train, just measures performance on unseen images.
# =============================================================================

print("\nRunning final evaluation on validation set...")
metrics = model.val()

map50    = metrics.box.map50    # mAP at IoU threshold 0.5
map50_95 = metrics.box.map      # mAP across IoU thresholds 0.5 to 0.95
precision= metrics.box.mp       # mean precision
recall   = metrics.box.mr       # mean recall


# =============================================================================
# BLOCK 7 — RESULTS SUMMARY
# =============================================================================
# Print a clean summary that you can screenshot for your FYP report.
# Also copies the best model weights to your ROS 2 workspace so it's
# ready to use in the detector node.
# =============================================================================

print("\n" + "="*65)
print("  TRAINING COMPLETE — RESULTS SUMMARY")
print("="*65)
print(f"\n  mAP@0.5:        {map50:.4f}  (target: > 0.85)")
print(f"  mAP@0.5:0.95:   {map50_95:.4f}")
print(f"  Precision:      {precision:.4f}  (of detected boxes, % correct)")
print(f"  Recall:         {recall:.4f}  (of real flowers, % found)")
print()

if map50 >= 0.85:
    print("  ✅  mAP@0.5 meets the FYP target (>0.85)")
elif map50 >= 0.70:
    print("  ⚠️   mAP@0.5 is acceptable but below target.")
    print("       Try: more epochs, larger dataset, or yolov8s.pt base model")
else:
    print("  ❌  mAP@0.5 is low. Check your dataset labels and try again.")

# Find the best model weights file
best_weights = Path(f"runs/detect/{RUN_NAME}/weights/best.pt")

if best_weights.exists():
    # Copy to ROS 2 workspace for use in the detector node
    dest = OUTPUT_DIR / "sunflower_best.pt"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    import shutil
    shutil.copy(best_weights, dest)

    print(f"\n  Model saved to:")
    print(f"    {best_weights}  (original)")
    print(f"    {dest}  (copied to ROS 2 workspace)")
    print(f"\n  Training graphs saved to:")
    print(f"    runs/detect/{RUN_NAME}/results.png")
    print(f"    runs/detect/{RUN_NAME}/confusion_matrix.png")
else:
    print(f"\n  ⚠️  Best weights not found at expected path: {best_weights}")

print(f"\n  TO USE IN ROS 2 DETECTOR NODE:")
print(f"    ros2 run precision_pollination yolov8_detector")
print(f"    --ros-args -p model_path:={OUTPUT_DIR}/sunflower_best.pt")

print("\n" + "="*65 + "\n")


# =============================================================================
# BLOCK 8 — QUICK INFERENCE TEST
# =============================================================================
# Optionally test the trained model on a single image to verify it works
# before deploying it to the drone simulation.
#
# Looks for any .jpg image in the validation folder and runs the detector on it.
# If a detection is made, you'll see the confidence score and box coordinates.
# =============================================================================

print("Running quick inference test on one validation image...")

# Find a sample image from the validation set
val_images_dir = DATASET_PATH.parent / "valid" / "images"
if not val_images_dir.exists():
    val_images_dir = DATASET_PATH.parent / "val" / "images"  # some datasets use 'val'

sample_images = list(val_images_dir.glob("*.jpg"))[:1] + \
                list(val_images_dir.glob("*.png"))[:1]

if sample_images:
    test_image = sample_images[0]
    print(f"  Testing on: {test_image.name}")

    test_results = model(str(test_image), conf=0.3, verbose=False)

    for r in test_results:
        n_detected = len(r.boxes)
        print(f"  Detected {n_detected} flower(s) in test image")
        for i, box in enumerate(r.boxes):
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].tolist()
            print(f"    Flower {i+1}: confidence={conf:.2f}, box={[round(x,1) for x in xyxy]}")

    if not sample_images:
        print("  (No validation images found for quick test)")
else:
    print("  (Could not find validation images for quick test — training still succeeded)")

print("\nDone. Your model is ready.\n")
