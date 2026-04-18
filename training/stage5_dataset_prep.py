#!/usr/bin/env python3
"""
=============================================================================
STAGE 5 — DATASET PREPARATION & UNIFICATION
FYP: Autonomous Swarm Drone Pollination System
=============================================================================

WHAT THIS SCRIPT DOES:
  Merges multiple sunflower/flower detection datasets from different sources
  (Roboflow, Zenodo, Kaggle) into a single unified dataset ready for YOLOv8
  training, regardless of which YOLO version the original labels were made for.

  Key operations:
    1. Scans each source dataset folder you provide
    2. Remaps all class IDs to a single unified class: 0 = sunflower_head
    3. Converts any YOLOv11/YOLOv9/YOLOv5 label format (they are all identical
       — YOLO label format has not changed, only the model architecture has)
    4. Copies images + labels into a unified train/valid/test split
    5. Writes a single data.yaml for YOLOv8 training
    6. Prints a summary so you can verify the merge before training

BEFORE RUNNING:
  Download your datasets and place them in ~/fyp_datasets/ as subfolders:
    ~/fyp_datasets/roboflow_sunflower/      ← from Roboflow export
    ~/fyp_datasets/zenodo_nab/              ← from Zenodo DOI 10.5281/zenodo.7708820
    ~/fyp_datasets/kaggle_sunflower_1/      ← from Kaggle dataset 1
    ~/fyp_datasets/kaggle_sunflower_2/      ← from Kaggle dataset 2
    ... (add as many as you have)

  Each subfolder must have the standard YOLO layout:
    images/train/   images/valid/   images/test/   (or flat images/)
    labels/train/   labels/valid/   labels/test/   (or flat labels/)
    data.yaml  (optional — script auto-detects without it)

HOW TO RUN:
  python3 stage5_dataset_prep.py

OUTPUT:
  ~/fyp_datasets/unified/
    images/train/   images/valid/   images/test/
    labels/train/   labels/valid/   labels/test/
    data.yaml       ← use this path in train_sunflower_fyp.py
    merge_report.txt

=============================================================================
"""

import os
import sys
import shutil
import random
import yaml
from pathlib import Path


# =============================================================================
# CONFIGURATION — edit these paths to match where you put your downloads
# =============================================================================

# Root folder containing all your dataset subfolders
DATASETS_ROOT = Path.home() / "fyp_datasets"

# Output folder for the unified merged dataset
OUTPUT_DIR = DATASETS_ROOT / "unified"

# Train / valid / test split ratios (must sum to 1.0)
# If a source dataset already has splits, those are respected.
# This ratio is used only for images found in a flat (unsplit) folder.
TRAIN_RATIO = 0.75
VALID_RATIO = 0.20
TEST_RATIO  = 0.05

# The single class name we are detecting. All source datasets may use
# different class names (e.g. "sunflower", "flower", "sunflower_head",
# "Sunflower") — this script remaps everything to this one name.
UNIFIED_CLASS_NAME = "sunflower_head"

# Image file extensions to look for
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# If True, skip images that already exist in the output (allows re-running
# the script to add new datasets without duplicating existing images)
SKIP_EXISTING = True

# Random seed for reproducible train/valid/test splits
RANDOM_SEED = 42


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def find_image_label_pairs(dataset_dir: Path):
    """
    Scan a dataset directory for (image_path, label_path) pairs.
    Handles both split layouts (images/train/, images/valid/) and
    flat layouts (images/).
    Returns a dict: {"train": [...], "valid": [...], "test": [...]}
    """
    dataset_dir = Path(dataset_dir)
    splits = {"train": [], "valid": [], "test": []}

    # --- Try split layout first ---
    found_split = False
    for split in ["train", "valid", "val", "test"]:
        img_dir = dataset_dir / "images" / split
        lbl_dir = dataset_dir / "labels" / split
        if not img_dir.exists():
            continue
        found_split = True
        canonical = "valid" if split == "val" else split
        for img_path in sorted(img_dir.iterdir()):
            if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
                continue
            lbl_path = lbl_dir / (img_path.stem + ".txt")
            if lbl_path.exists():
                splits[canonical].append((img_path, lbl_path))
            else:
                # Image with no label — skip (background image)
                pass

    if found_split:
        return splits

    # --- Flat layout: images/ and labels/ at root ---
    img_dir = dataset_dir / "images"
    lbl_dir = dataset_dir / "labels"
    if not img_dir.exists():
        # Some datasets put images directly in root
        img_dir = dataset_dir
        lbl_dir = dataset_dir

    all_pairs = []
    for img_path in sorted(img_dir.iterdir()):
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        lbl_path = lbl_dir / (img_path.stem + ".txt")
        if lbl_path.exists():
            all_pairs.append((img_path, lbl_path))

    if not all_pairs:
        return splits  # empty

    # Random split
    random.seed(RANDOM_SEED)
    random.shuffle(all_pairs)
    n = len(all_pairs)
    n_train = int(n * TRAIN_RATIO)
    n_valid = int(n * VALID_RATIO)
    splits["train"] = all_pairs[:n_train]
    splits["valid"] = all_pairs[n_train:n_train + n_valid]
    splits["test"]  = all_pairs[n_train + n_valid:]
    return splits


def read_data_yaml(dataset_dir: Path):
    """
    Read data.yaml from a dataset directory.
    Returns the class names list, or None if not found.
    """
    yaml_path = dataset_dir / "data.yaml"
    if not yaml_path.exists():
        # Some datasets put it one level up
        yaml_path = dataset_dir.parent / "data.yaml"
    if not yaml_path.exists():
        return None
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    names = data.get("names", None)
    if isinstance(names, dict):
        # YOLO sometimes uses {0: 'sunflower'} format
        return [names[i] for i in sorted(names.keys())]
    return names  # list format


def remap_label_file(src_label: Path, src_classes: list, dst_label: Path):
    """
    Read a YOLO label file and remap class IDs so that any class that
    looks like a sunflower/flower maps to class 0 (our unified class).
    Lines with unrecognised classes are dropped.
    Writes the remapped labels to dst_label.
    """
    FLOWER_KEYWORDS = {
        "sunflower", "sunflower_head", "flower", "fleur",
        "tournesol", "helianthus", "bloom", "blossom"
    }

    dst_label.parent.mkdir(parents=True, exist_ok=True)
    lines_out = []

    with open(src_label, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                class_id = int(parts[0])
            except ValueError:
                continue

            # Map class name to unified class 0
            if src_classes is not None:
                if class_id >= len(src_classes):
                    continue  # invalid class id, skip
                class_name = src_classes[class_id].lower().replace(" ", "_")
                # Accept anything with a flower keyword, or a single-class dataset
                if not any(kw in class_name for kw in FLOWER_KEYWORDS):
                    if len(src_classes) == 1:
                        pass  # single-class dataset — assume it's the target
                    else:
                        continue  # multi-class dataset, skip non-flower classes
            # Write with unified class id = 0
            lines_out.append("0 " + " ".join(parts[1:]))

    if lines_out:
        with open(dst_label, "w") as f:
            f.write("\n".join(lines_out) + "\n")
        return True
    return False  # no valid labels written


def copy_image(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if SKIP_EXISTING and dst.exists():
        return False
    shutil.copy2(src, dst)
    return True


def safe_name(stem: str, dataset_name: str) -> str:
    """Prefix filename with dataset name to avoid collisions."""
    return f"{dataset_name}__{stem}"


# =============================================================================
# MAIN
# =============================================================================

def main():
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║  STAGE 5 — Dataset Preparation & Unification        ║")
    print("║  FYP: Swarm Drone Pollination                        ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()

    if not DATASETS_ROOT.exists():
        print(f"ERROR: Datasets root not found: {DATASETS_ROOT}")
        print()
        print("Create it and place your datasets inside:")
        print(f"  mkdir -p {DATASETS_ROOT}")
        print(f"  # Then add subfolders: roboflow_sunflower/, zenodo_nab/, kaggle_*/")
        sys.exit(1)

    # Find all dataset subfolders (exclude the output folder)
    dataset_dirs = [
        d for d in sorted(DATASETS_ROOT.iterdir())
        if d.is_dir() and d.name != "unified"
    ]

    if not dataset_dirs:
        print(f"No dataset subfolders found in {DATASETS_ROOT}")
        print("Expected layout:")
        print(f"  {DATASETS_ROOT}/")
        print(f"    roboflow_sunflower/")
        print(f"    zenodo_nab/")
        print(f"    kaggle_sunflower_1/")
        sys.exit(1)

    print(f"Found {len(dataset_dirs)} dataset(s) to merge:")
    for d in dataset_dirs:
        print(f"  • {d.name}")
    print()

    # Create output directories
    for split in ["train", "valid", "test"]:
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Process each dataset
    total_counts = {"train": 0, "valid": 0, "test": 0}
    report_lines = ["DATASET MERGE REPORT", "=" * 60, ""]

    for dataset_dir in dataset_dirs:
        print(f"Processing: {dataset_dir.name}")

        # Read class names from this dataset's data.yaml (if present)
        src_classes = read_data_yaml(dataset_dir)
        if src_classes:
            print(f"  Classes found: {src_classes}")
        else:
            print(f"  No data.yaml found — assuming single sunflower class")
            src_classes = ["sunflower_head"]

        # Find image/label pairs by split
        splits = find_image_label_pairs(dataset_dir)
        ds_counts = {}

        for split, pairs in splits.items():
            count = 0
            for img_path, lbl_path in pairs:
                stem = safe_name(img_path.stem, dataset_dir.name)
                dst_img = OUTPUT_DIR / "images" / split / (stem + img_path.suffix)
                dst_lbl = OUTPUT_DIR / "labels" / split / (stem + ".txt")

                # Remap labels
                if remap_label_file(lbl_path, src_classes, dst_lbl):
                    copy_image(img_path, dst_img)
                    count += 1

            ds_counts[split] = count
            total_counts[split] += count

        summary = f"  train={ds_counts['train']}  valid={ds_counts['valid']}  test={ds_counts['test']}"
        print(summary)
        report_lines.append(f"Dataset: {dataset_dir.name}")
        report_lines.append(summary)
        report_lines.append("")

    # Write unified data.yaml
    data_yaml = {
        "path": str(OUTPUT_DIR),
        "train": "images/train",
        "val":   "images/valid",
        "test":  "images/test",
        "nc": 1,
        "names": [UNIFIED_CLASS_NAME],
    }
    yaml_path = OUTPUT_DIR / "data.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(data_yaml, f, default_flow_style=False, sort_keys=False)

    # Write merge report
    report_lines += [
        "=" * 60,
        "TOTALS",
        f"  train : {total_counts['train']} images",
        f"  valid : {total_counts['valid']} images",
        f"  test  : {total_counts['test']} images",
        f"  TOTAL : {sum(total_counts.values())} images",
        "",
        f"Unified data.yaml written to:",
        f"  {yaml_path}",
        "",
        "Use this path in train_sunflower_fyp.py:",
        f'  DATASET_PATH = Path("{yaml_path}")',
    ]
    report_path = OUTPUT_DIR / "merge_report.txt"
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines) + "\n")

    # Print final summary
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║  STAGE 5 COMPLETE                                    ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()
    print("UNIFIED DATASET SUMMARY:")
    print(f"  train : {total_counts['train']} images")
    print(f"  valid : {total_counts['valid']} images")
    print(f"  test  : {total_counts['test']} images")
    print(f"  TOTAL : {sum(total_counts.values())} images")
    print()
    print(f"data.yaml written to:")
    print(f"  {yaml_path}")
    print()
    print("NEXT STEP — train YOLOv8 on the unified dataset:")
    print(f"  Edit train_sunflower_fyp.py and set:")
    print(f'    DATASET_PATH = Path("{yaml_path}")')
    print(f"  Then run:")
    print(f"    python3 train_sunflower_fyp.py")
    print()
    print(f"Full merge report saved to: {report_path}")
    print()


if __name__ == "__main__":
    main()
