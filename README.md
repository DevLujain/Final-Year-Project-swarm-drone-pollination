# Autonomous Drone Swarm Pollination — FYP Setup Guide

**University Malaya · Department of Artificial Intelligence**  
**Student:** Lujain Alhumaidah | **Supervisor:** Dr. Narsimlu Kemsaram  
**Stack:** Ubuntu 24.04 · ROS 2 Jazzy · PX4 SITL v1.16 · Gazebo Harmonic · YOLOv8

---

## Table of Contents

1. [Repository Structure](#1-repository-structure)
2. [Setup Stages — Quick Reference](#2-setup-stages--quick-reference)
3. [Stage 1 — System & Simulation Setup](#3-stage-1--system--simulation-setup)
4. [Stage 2 — ROS 2 Jazzy Installation](#4-stage-2--ros-2-jazzy-installation)
5. [Stage 3 — YOLOv8 ROS 2 Vision Node](#5-stage-3--yolov8-ros-2-vision-node)
6. [Stage 4 — ML Stack Verification](#6-stage-4--ml-stack-verification)
7. [Stage 5 — Dataset Download & Unification](#7-stage-5--dataset-download--unification)
8. [Stage 6 — Training (`train_sunflower_fyp.py`)](#8-stage-6--training-train_sunflower_fyppy)
9. [Stage 7 — Full System Verification](#9-stage-7--full-system-verification)
10. [Dataset FAQ](#10-dataset-faq)
11. [GitHub Repository Organisation](#11-github-repository-organisation)
12. [Running the Full Stack](#12-running-the-full-stack)

---

## 1. Repository Structure

```
swarm-drone-pollination/          ← your GitHub repo root
│
├── README.md                     ← this file
│
├── setup/                        ← all setup scripts (run in order)
│   ├── stage1_system_ubuntu24.sh
│   ├── stage2_ros2_ubuntu24.sh
│   ├── stage3_yolov8_node.sh
│   ├── stage4_yolov8.sh
│   ├── stage5_dataset_prep.py    ← NEW: merges all your datasets
│   └── stage6_verify.sh          ← was stage4_verify.sh
│
├── training/
│   └── train_sunflower_fyp.py    ← run AFTER stage5_dataset_prep.py
│
├── ros2_ws/                      ← your ROS 2 workspace (auto-created by stage2)
│   └── src/
│       └── precision_pollination/
│           ├── pollination_controller.py
│           └── yolov8_detector.py
│
├── models/                       ← trained model weights (git-ignored — see below)
│   └── .gitkeep
│
├── datasets/                     ← dataset index only (actual data git-ignored)
│   ├── .gitignore                ← ignores images/, labels/ — only tracks data.yaml files
│   └── README_datasets.md        ← instructions for re-downloading datasets
│
├── docs/
│   └── FYP_Report_Week6_Updated.docx
│
└── .gitignore
```

> **The `datasets/` folder and `models/` folder are listed in `.gitignore`.**  
> You never push raw image data or large model weights to GitHub — only the scripts, code, and YAML config files. See [Section 11](#11-github-repository-organisation) for the full `.gitignore`.

---

## 2. Setup Stages — Quick Reference

Run these in order. Each stage must fully complete before the next.

| Stage | Script | Run as | Time | What it does |
|-------|--------|--------|------|--------------|
| **1** | `stage1_system_ubuntu24.sh` | `sudo bash` | ~25 min | OS deps, Gazebo Harmonic, PX4 SITL, uXRCE-DDS bridge |
| **2** | `stage2_ros2_ubuntu24.sh` | `bash` | ~15 min | ROS 2 Jazzy, ros_gz bridge, px4_msgs, ROS 2 workspace |
| **3** | `stage3_yolov8_node.sh` | `bash` | ~5 min | Writes YOLOv8 ROS 2 detector node and pollination controller |
| **4** | `stage4_yolov8.sh` | `bash` | ~10 min | Installs full ML Python stack, verifies YOLOv8 loads |
| **5** | `stage5_dataset_prep.py` | `python3` | ~5 min | Downloads + merges all datasets into one unified folder |
| **6** | `train_sunflower_fyp.py` | `python3` | 30–90 min | Fine-tunes YOLOv8 on unified dataset, saves `best.pt` |
| **7** | `stage6_verify.sh` | `bash` | ~5 min | Checks every component, launches full simulation stack |

**After Stage 1**, always run `source ~/.bashrc` in a fresh terminal before continuing.

---

## 3. Stage 1 — System & Simulation Setup

**Script:** `setup/stage1_system_ubuntu24.sh`  
**Run:** `sudo bash setup/stage1_system_ubuntu24.sh`

This is the longest and most critical stage. It installs every system-level dependency the project needs.

**What it installs:**

- System packages: `build-essential`, `cmake`, `ninja-build`, `git`, GStreamer libraries
- Python packages: `ultralytics` (YOLOv8), `opencv-python`, `numpy`, `pandas`, `scikit-learn`
- **Gazebo Harmonic** — the 3D physics simulator for Ubuntu 24.04. This is NOT the same as Gazebo Classic. Ubuntu 24.04 requires Gazebo Harmonic; the old `gazebo-classic` does not install on this OS version.
- **PX4 Autopilot** — cloned from GitHub and compiled for SITL (Software In The Loop) mode. The first build takes 5–15 minutes. The built model is `gz_x500` (a simulated quadcopter).
- **micro-XRCE-DDS Agent** — the bridge that translates between PX4's internal uORB message bus and ROS 2 topics. Without this, ROS 2 cannot see the simulated drone.
- Environment variables: `GZ_VERSION=harmonic` and `PX4_DIR` added to `~/.bashrc`.

**After it finishes:**
```bash
source ~/.bashrc
# Quick test — open two terminals:
# Terminal 1:
cd ~/PX4-Autopilot && make px4_sitl gz_x500
# Terminal 2:
MicroXRCEAgent udp4 -p 8888
```
You should see a quadcopter appear in the Gazebo window.

---

## 4. Stage 2 — ROS 2 Jazzy Installation

**Script:** `setup/stage2_ros2_ubuntu24.sh`  
**Run:** `bash setup/stage2_ros2_ubuntu24.sh` *(no sudo)*

**Important:** Ubuntu 24.04 uses **ROS 2 Jazzy Jalisco**, not ROS 2 Humble. Trying to install Humble on Ubuntu 24.04 causes dependency conflicts. Jazzy has identical features to Humble — all your code works exactly the same way.

**What it installs:**

- **ROS 2 Jazzy Desktop** — the full ROS 2 installation including `rviz2`, `rqt`, and demo nodes
- **colcon** — the build tool for ROS 2 packages
- **ros_gz bridge** — translates Gazebo Harmonic topics into ROS 2 topics (camera images, drone pose)
- **px4_msgs** — the ROS 2 message type definitions PX4 uses (`VehicleLocalPosition`, `TrajectorySetpoint`, etc.)
- **px4_ros_com** — example utilities for PX4 + ROS 2 integration
- Creates `~/ros2_ws/` — your ROS 2 workspace
- Creates the `precision_pollination` package skeleton with `pollination_controller.py`

**After it finishes:**
```bash
# Fresh terminal:
source ~/.bashrc
ros2 topic list     # should print /rosout
ros2 run demo_nodes_py talker   # basic publish test
```

---

## 5. Stage 3 — YOLOv8 ROS 2 Vision Node

**Script:** `setup/stage3_yolov8_node.sh`  
**Run:** `bash setup/stage3_yolov8_node.sh`

This stage writes the actual Python source code for the two main ROS 2 nodes into your workspace, then rebuilds the workspace.

**What it creates:**

- `~/ros2_ws/src/precision_pollination/precision_pollination/yolov8_detector.py`  
  The vision node: subscribes to `/camera/image_raw`, runs YOLOv8 on every frame, publishes detected flower positions as a `PoseArray` on `/sunflower_targets` and raw bounding boxes as JSON on `/yolov8/detections`.

- `~/ros2_ws/src/precision_pollination/precision_pollination/pollination_controller.py`  
  The state machine node: `IDLE → TAKEOFF → SEARCH → APPROACH → HOVER → POLLINATE → NEXT_TARGET → LAND`. Subscribes to `/sunflower_targets` and PX4 position, publishes trajectory setpoints to `/fmu/in/trajectory_setpoint`.

- `~/ros2_ws/src/precision_pollination/setup.py` — registers both nodes as runnable executables.

**After it finishes, both nodes are launchable:**
```bash
ros2 run precision_pollination yolov8_detector
ros2 run precision_pollination pollination_controller
```

---

## 6. Stage 4 — ML Stack Verification

**Script:** `setup/stage4_yolov8.sh`  
**Run:** `bash setup/stage4_yolov8.sh`  
**Can run in parallel with Stage 1** (in a second terminal) since it only touches Python packages.

This stage installs the complete Python machine learning stack and verifies that YOLOv8 loads correctly.

**What it installs:**

- `torch`, `torchvision` — PyTorch deep learning framework
- `ultralytics` — YOLOv8 library (re-installs to ensure latest version)
- `opencv-python`, `Pillow`, `tqdm`, `PyYAML`, `scikit-learn`, `matplotlib`
- `rclpy`, `sensor_msgs_py` — Python bindings for ROS 2

**Verification step:** Downloads and loads `yolov8n.pt` to confirm GPU/CPU inference works.

---

## 7. Stage 5 — Dataset Download & Unification

**Script:** `setup/stage5_dataset_prep.py`  
**Run:** `python3 setup/stage5_dataset_prep.py`  
**When:** After Stage 4, before Stage 6 (training).

This is the stage that answers your questions about mixing multiple datasets. See the [Dataset FAQ](#10-dataset-faq) section for detailed answers about Roboflow, Zenodo, and Kaggle.

### 7.1 What to Download First

Download each dataset manually and place it in `~/fyp_datasets/` as a subfolder. The script expects this layout:

```
~/fyp_datasets/
  roboflow_sunflower/         ← Roboflow export (any sunflower detection dataset)
  zenodo_nab/                 ← Zenodo 10.5281/zenodo.7708820 zip, extracted
  kaggle_sunflower_1/         ← Kaggle dataset 1, extracted
  kaggle_sunflower_2/         ← Kaggle dataset 2, extracted
  ...                         ← add as many as you like
```

Each subfolder should have the standard YOLO structure:
```
images/train/   images/valid/   (or just images/ for flat datasets)
labels/train/   labels/valid/   (or just labels/)
data.yaml       (optional — script works without it)
```

### 7.2 Downloading from Roboflow

1. Go to [https://universe.roboflow.com/search?q=sunflower+head](https://universe.roboflow.com/search?q=sunflower+head)
2. Pick any dataset with 300+ images. Recommended: search for "sunflower detection" or "sunflower head detection"
3. Click **Export Dataset** → Format: **YOLOv8** → **Download zip**
4. Unzip it:
   ```bash
   mkdir -p ~/fyp_datasets/roboflow_sunflower
   cd ~/fyp_datasets/roboflow_sunflower
   unzip ~/Downloads/your_roboflow_export.zip
   ```

**Yes, you can use multiple Roboflow datasets.** Just create a separate subfolder for each one:
```
~/fyp_datasets/roboflow_sunflower_1/
~/fyp_datasets/roboflow_sunflower_2/
```
The merge script handles all of them automatically.

**Do not clone datasets into your GitHub repo.** Download the zip, extract into `~/fyp_datasets/`, and reference the path in your scripts. The `datasets/` folder in your repo stores only YAML config files and download instructions, not the actual images.

### 7.3 Downloading from Zenodo

1. Go to: [https://zenodo.org/records/7708820](https://zenodo.org/records/7708820)
2. Download the zip file (this is the Pinheiro et al. NAB Flower Detection Dataset, 206 images)
3. Extract it:
   ```bash
   mkdir -p ~/fyp_datasets/zenodo_nab
   cd ~/fyp_datasets/zenodo_nab
   unzip ~/Downloads/flower_detection_dataset.zip
   ```

This dataset is in YOLO format already. The class may be named `flower` — the merge script remaps it to `sunflower_head` automatically.

### 7.4 Downloading from Kaggle

1. Install the Kaggle CLI (if you haven't already):
   ```bash
   pip3 install kaggle --break-system-packages
   # Go to kaggle.com → account → API → Download kaggle.json
   mkdir -p ~/.kaggle && cp ~/Downloads/kaggle.json ~/.kaggle/
   chmod 600 ~/.kaggle/kaggle.json
   ```
2. Download a dataset (replace `dataset-slug` with the actual slug from the URL):
   ```bash
   mkdir -p ~/fyp_datasets/kaggle_sunflower_1
   cd ~/fyp_datasets/kaggle_sunflower_1
   kaggle datasets download -d <owner/dataset-slug> --unzip
   ```

**About YOLOv11 labels on Kaggle:** The label file format used by YOLO has **not changed** between v5, v7, v8, v9, v10, and v11. All versions use the same text format: `class_id cx cy w h` (normalised coordinates, one line per bounding box). A label file exported for YOLOv11 is 100% compatible with YOLOv8 training. You do not need to convert anything — just make sure to export in "YOLO" format and the merge script handles the class remapping.

### 7.5 What the Script Does

Once all your dataset folders are in `~/fyp_datasets/`, run:
```bash
python3 setup/stage5_dataset_prep.py
```

It will:
1. Scan every subfolder for image + label pairs
2. Read each dataset's `data.yaml` to find the class names (e.g. `flower`, `Sunflower`, `sunflower_head`)
3. Remap all class IDs to a single unified class `0 = sunflower_head`
4. Copy images and remapped labels into `~/fyp_datasets/unified/` with prefixed filenames to avoid collisions
5. Respect existing train/valid/test splits where present; auto-split flat datasets (75/20/5)
6. Write a single `~/fyp_datasets/unified/data.yaml` ready for YOLOv8 training
7. Print a merge report showing how many images came from each source

**Output:**
```
~/fyp_datasets/unified/
  images/train/   ← all training images from all datasets
  images/valid/   ← all validation images
  images/test/    ← all test images
  labels/train/   ← remapped label files
  labels/valid/
  labels/test/
  data.yaml       ← USE THIS PATH in train_sunflower_fyp.py
  merge_report.txt
```

---

## 8. Stage 6 — Training (`train_sunflower_fyp.py`)

**Script:** `training/train_sunflower_fyp.py`  
**Run:** `python3 training/train_sunflower_fyp.py`  
**When:** After `stage5_dataset_prep.py` has produced `~/fyp_datasets/unified/data.yaml`.

### 8.1 Before Running

Open `train_sunflower_fyp.py` and confirm one line matches your output from Stage 5:
```python
DATASET_PATH = Path.home() / "fyp_datasets" / "unified" / "data.yaml"
```

### 8.2 What It Does

1. Loads `yolov8n.pt` — the pre-trained YOLOv8 nano model (downloads ~6 MB automatically on first run)
2. Fine-tunes it on your unified sunflower dataset for 100 epochs
3. Applies data augmentation: horizontal flip, brightness/contrast jitter, HSV colour shift — to make the model robust to varying lighting and drone angles
4. Validates after every epoch and saves the best model weights automatically
5. At the end, runs a final evaluation and prints `mAP@0.5`, precision, and recall
6. Copies `best.pt` to `~/ros2_ws/sunflower_best.pt` so the detector node can load it immediately

### 8.3 Expected Results

| Metric | Target | What it means |
|--------|--------|---------------|
| `mAP@0.5` | ≥ 0.85 | Detection accuracy at IoU threshold 0.5 |
| Precision | ≥ 0.85 | Of all boxes drawn, % that are real flowers |
| Recall | ≥ 0.85 | Of all real flowers, % that were detected |
| Inference | < 100 ms | Per frame on simulation camera feed |

### 8.4 After Training

```bash
# Verify the model works:
python3 -c "
from ultralytics import YOLO
model = YOLO('~/ros2_ws/sunflower_best.pt')
print('Model loaded OK')
model.info()
"

# Use in the ROS 2 detector node:
ros2 run precision_pollination yolov8_detector \
  --ros-args -p model_path:=$HOME/ros2_ws/sunflower_best.pt
```

Training results (graphs, confusion matrix, weights) are saved to:
```
runs/detect/sunflower_fyp/
  weights/best.pt
  weights/last.pt
  results.png
  confusion_matrix.png
  val_batch0_pred.jpg   ← visual check of predictions
```

---

## 9. Stage 7 — Full System Verification

**Script:** `setup/stage6_verify.sh` *(was stage4_verify.sh in old layout)*  
**Run:** `bash setup/stage6_verify.sh`  
**When:** After all setup stages AND after training is complete.

This script checks every installed component and then launches the full simulation stack in four separate terminal windows.

**Checks performed:**
- Ubuntu 24.04 confirmed
- Gazebo Harmonic binary present
- PX4 cloned and compiled
- `MicroXRCEAgent` in PATH
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- `ros_gz_bridge` installed
- `ultralytics` (YOLOv8) importable
- OpenCV importable
- ROS 2 workspace built
- Both ROS 2 nodes present

**Launches (in separate gnome-terminal windows):**
1. PX4 SITL + Gazebo Harmonic
2. uXRCE-DDS bridge
3. YOLOv8 detector node
4. Pollination state machine node

**Verification commands (run in a fifth terminal after launch):**
```bash
ros2 topic list
# Expected topics include:
#   /fmu/out/vehicle_local_position
#   /camera/image_raw
#   /sunflower_targets
#   /yolov8/detections
#   /pollination/log

ros2 topic echo /fmu/out/vehicle_local_position
# Should show position numbers updating in real time

ros2 topic echo /sunflower_targets
# Will show flower positions once the drone is airborne and detects flowers
```

---

## 10. Dataset FAQ

### Q1: Do I download datasets by cloning into the repository? Can I mix multiple datasets?

**No — do not clone or commit dataset images into your GitHub repository.**

Reasons:
- A single Roboflow sunflower dataset is typically 200–800 MB of images. Three or four datasets together exceed GitHub's 1 GB repository size limit.
- Git is not designed for binary files. Large image commits make the repo slow for everyone who clones it.

**The correct approach:**
1. Download zip files manually from Roboflow/Zenodo/Kaggle
2. Extract into `~/fyp_datasets/<dataset_name>/` on your local machine
3. Run `stage5_dataset_prep.py` to merge them
4. Only commit the `data.yaml` files and download instructions to GitHub

**Yes, you can absolutely mix multiple datasets.** `stage5_dataset_prep.py` is specifically built for this. It handles different class names, different split structures, and avoids filename collisions by prefixing each image with its source dataset name.

---

### Q2: Do I download the Zenodo zip into the repository?

**No.** Extract it into `~/fyp_datasets/zenodo_nab/` on your local machine, not inside your repo folder.

The Zenodo dataset at [https://zenodo.org/records/7708820](https://zenodo.org/records/7708820) is the NAB Flower Detection Dataset by Pinheiro et al. (206 images, 499 annotated flowers). After extraction you should have:

```
~/fyp_datasets/zenodo_nab/
  images/
    train/   (126 images)
    val/     (40 images)
  labels/
    train/
    val/
  data.yaml
```

The merge script reads this structure and incorporates it into the unified dataset. The class in this dataset may be labelled `flower` — it gets remapped to `sunflower_head` automatically.

---

### Q3: Kaggle datasets with YOLOv11 labels — do they work?

**Yes, completely.** The YOLO bounding box label format is identical across all YOLO versions (v5, v7, v8, v9, v10, v11):

```
class_id  center_x  center_y  width  height
```

All values are normalised (0.0 to 1.0), one object per line. Nothing in this format has changed between versions. "YOLOv11 format" means the *model architecture* was v11, not that the labels use a different format.

When a Kaggle dataset says only "YOLO format" without specifying a version, that is also the same format — it is universal across all versions.

**What to check on Kaggle before downloading:**
- Does the dataset have a `labels/` folder with `.txt` files?  → Compatible
- Does each `.txt` file have lines like `0 0.512 0.348 0.198 0.234`? → Compatible
- Is it in Pascal VOC XML format or COCO JSON instead? → Not directly compatible (but rare on Kaggle for YOLO-labelled datasets)

For Kaggle datasets that are NOT in YOLO format, you can convert them using the Roboflow web interface or the `roboflow` Python package.

---

### Q4: Can I add more datasets later?

Yes. The merge script is additive — just add a new subfolder to `~/fyp_datasets/`, re-run `stage5_dataset_prep.py`, and re-run training. Set `SKIP_EXISTING = True` in the script (default) to avoid re-copying files that are already in the unified folder.

---

## 11. GitHub Repository Organisation

### 11.1 What Goes in the Repo

| Include | Exclude |
|---------|---------|
| All `.sh` setup scripts | Raw image datasets (`images/`, `.jpg`, `.png`) |
| All `.py` scripts | Trained model weights (`best.pt`, `last.pt`, `.pt` files) |
| `data.yaml` config files | The unified dataset folder (`~/fyp_datasets/`) |
| `README.md` | PX4-Autopilot source (~1 GB) |
| ROS 2 node source code | `ros2_ws/build/` and `ros2_ws/install/` |
| `docs/` folder | `__pycache__/` |
| `.gitignore` | `runs/` (training output) |

### 11.2 `.gitignore`

Create this file at your repo root:

```gitignore
# ── Datasets (too large for git) ──────────────────────────────
datasets/images/
datasets/labels/
*.jpg
*.jpeg
*.png
*.bmp
*.webp

# ── Model weights ─────────────────────────────────────────────
*.pt
*.pth
*.onnx
*.tflite
models/
!models/.gitkeep

# ── Training output ───────────────────────────────────────────
runs/
wandb/

# ── ROS 2 workspace build artefacts ──────────────────────────
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/

# ── Python ────────────────────────────────────────────────────
__pycache__/
*.pyc
*.pyo
.env
venv/

# ── Dataset download folders (local machine only) ─────────────
fyp_datasets/
fyp_training/

# ── OS / IDE ──────────────────────────────────────────────────
.DS_Store
*.swp
.idea/
.vscode/
```

### 11.3 Recommended Repository Layout on GitHub

```
swarm-drone-pollination/
├── README.md
├── .gitignore
│
├── setup/
│   ├── stage1_system_ubuntu24.sh
│   ├── stage2_ros2_ubuntu24.sh
│   ├── stage3_yolov8_node.sh
│   ├── stage4_yolov8.sh
│   ├── stage5_dataset_prep.py     ← dataset merge script
│   └── stage6_verify.sh
│
├── training/
│   └── train_sunflower_fyp.py
│
├── ros2_ws/
│   └── src/
│       └── precision_pollination/
│           ├── package.xml
│           ├── setup.py
│           └── precision_pollination/
│               ├── __init__.py
│               ├── pollination_controller.py
│               └── yolov8_detector.py
│
├── datasets/
│   ├── .gitignore           ← contains: images/ labels/ *.jpg *.png
│   └── README_datasets.md   ← instructions for re-downloading
│
├── models/
│   └── .gitkeep             ← placeholder so folder exists in repo
│
└── docs/
    └── FYP_Report_Week6_Updated.docx
```

### 11.4 Setting Up the Repo from Scratch

```bash
# 1. Create the repo on GitHub (name: swarm-drone-pollination)
# 2. Clone it to your machine:
git clone https://github.com/YOUR_USERNAME/swarm-drone-pollination.git
cd swarm-drone-pollination

# 3. Create the folder structure:
mkdir -p setup training ros2_ws/src/precision_pollination/precision_pollination
mkdir -p datasets models docs
touch models/.gitkeep datasets/.gitignore

# 4. Copy your scripts into the right folders:
cp /path/to/stage1_system_ubuntu24.sh  setup/
cp /path/to/stage2_ros2_ubuntu24.sh    setup/
cp /path/to/stage3_yolov8_node.sh      setup/
cp /path/to/stage4_yolov8.sh           setup/
cp /path/to/stage5_dataset_prep.py     setup/
cp /path/to/stage4_verify.sh           setup/stage6_verify.sh
cp /path/to/train_sunflower_fyp.py     training/

# 5. Add the .gitignore (copy from Section 11.2 above)
# 6. Commit and push:
git add .
git commit -m "Initial project setup — all stages and training scripts"
git push origin main
```

### 11.5 `datasets/README_datasets.md`

Create this file so anyone who clones your repo knows how to get the data:

```markdown
# Datasets

The actual image data is NOT stored in this repository (files are too large for git).

## To reproduce the full dataset, download the following:

### 1. Roboflow Sunflower Dataset
- URL: https://universe.roboflow.com/search?q=sunflower+head
- Export format: YOLOv8
- Extract to: ~/fyp_datasets/roboflow_sunflower/

### 2. Zenodo NAB Flower Detection Dataset
- DOI: https://zenodo.org/records/7708820
- Paper: Pinheiro et al. (2023), Applied Sciences
- Extract to: ~/fyp_datasets/zenodo_nab/

### 3. Kaggle Datasets
- Search: https://www.kaggle.com/search?q=sunflower+detection+YOLO
- Extract each to: ~/fyp_datasets/kaggle_sunflower_<N>/

## After downloading all datasets, run:
python3 setup/stage5_dataset_prep.py

This will merge everything into ~/fyp_datasets/unified/ and write data.yaml.
Then run: python3 training/train_sunflower_fyp.py
```

---

## 12. Running the Full Stack

Once all stages are complete, use these commands every time you want to run the system.
Open five terminal windows and run one command in each:

```bash
# Terminal 1 — PX4 SITL + Gazebo (wait for Gazebo window to open)
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2 — uXRCE-DDS bridge (wait for "running in execution thread")
MicroXRCEAgent udp4 -p 8888

# Terminal 3 — YOLOv8 detector node
source ~/.bashrc
ros2 run precision_pollination yolov8_detector \
  --ros-args -p model_path:=$HOME/ros2_ws/sunflower_best.pt

# Terminal 4 — Pollination state machine
source ~/.bashrc
ros2 run precision_pollination pollination_controller

# Terminal 5 — Monitor topics
ros2 topic list
ros2 topic echo /sunflower_targets
ros2 topic echo /pollination/log
```

### Screenshot checklist for FYP report

Take these screenshots once the stack is running (for Chapter 4 evidence):

- [ ] Gazebo window with simulated drone visible
- [ ] `ros2 topic list` output showing all expected topics
- [ ] PX4 SITL console showing `[commander] Armed` and flight mode
- [ ] uXRCE-DDS bridge console showing connected agent
- [ ] Terminal showing `yolov8_detector` detecting flowers
- [ ] `ros2 topic echo /pollination/log` showing pollination events
- [ ] Training results: `runs/detect/sunflower_fyp/results.png`
- [ ] mAP@0.5 score printed at end of training

