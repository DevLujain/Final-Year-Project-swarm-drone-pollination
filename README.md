# Autonomous Swarm Drone Pollination System (ASDPS)

**Universiti Malaya · Faculty of Computer Science & Information Technology · Department of Artificial Intelligence**
**Student:** Lujain Alhumaidah · **Supervisor:** Dr. Narsimlu Kemsaram

A Final Year Project implementing an autonomous multi-drone system for precision sunflower pollination. A swarm of simulated quadcopters surveys a field, detects sunflowers with onboard vision, plans collision-free pollination routes, and autonomously pollinates detected flowers across a multi-day bloom cycle.

**Fixed stack:** Ubuntu 24.04 · ROS 2 Jazzy · PX4 SITL v1.16 · Gazebo Harmonic · YOLOv8

---

## Repository Structure

```
.
├── README.md                  ← this file
├── docs/                      ← setup guide, error-fix notes, report drafts
├── setup/                     ← Stage 1–14 install & fix scripts, Stage 14 nodes + world
├── launch_stage14.sh          ← Stage 14 launcher (3-drone visual-servoing demo)
├── stage15_official/          ← Stage 15: clean multi-day rewrite (4 nodes + launcher + config)
├── stage16_official/Stage16/  ← Stage 16: "no-cheat" perception build + web dashboard (latest)
├── ros2_ws/                   ← ROS 2 workspace — precision_pollination package
│   └── src/precision_pollination/
├── mission_outputs/           ← logged data from real runs (stage12 / 14 / 15 / 16)
├── fyp_training/              ← early training-phase mission logs
├── Sunflower_stages/          ← bloom-stage image classes (YoungBud → Wilted)
├── real_footage/              ← real sunflower video + YOLO detection stills
├── daas/                      ← dashboard-as-a-service prototype
├── rviz/                      ← RViz configs
└── LICENSE
```

---

## Quick Start (Stage 16 — latest)

Stage 16 is the most complete build: drones rely **only on their own sensors** (camera + ToF) — no ground-truth flower positions are fed to the controllers — with a multi-day bloom cycle, map merge, mTSP-optimised route assignment, and live web dashboard.

```bash
cd stage16_official/Stage16

# 1. Generate the field / world
python3 Field_generator.py

# 2. Launch everything (Gazebo, PX4 instances, bridges, nodes, dashboard)
bash launch_stage16.sh stage16_config.yaml

# 3. Watch the live dashboard
#    → http://localhost:5000

# 4. After the run, build the results table for the report
python3 summarize_stage16_run.py
```

Configuration (drone count, bloom days, logging directory, telemetry rate) lives in `stage16_config.yaml`.

### Stage 16 architecture

| Component | File | Role |
|---|---|---|
| Field generator | `Field_generator.py` | Builds the Gazebo world + sunflower field SDF |
| Bloom model | `bloom_world.py` | Advances flower bloom stages across mission days |
| Perception | `perception_node.py` | Per-drone camera → flower/bud cluster detection (publishes JSON observations) |
| Drone controller | `Drone_Controller.py` | Per-drone FSM: survey, ToF height enrichment, pollination approach, return-to-base |
| Orchestrator | `mission_orchestrator.py` | Day cycle, map merge, route assignment, event/telemetry/pollination CSV logging |
| Dashboard | `dashboard/` | Flask + JS live mission view (drone states, map, pollination count) |
| Summarizer | `summarize_stage16_run.py` | Turns a run's CSVs into the per-day results table |

---

## Stage Progression

| Stage | What was built | Where |
|---|---|---|
| 1–4 | System, ROS 2 Jazzy, PX4 SITL + Gazebo Harmonic, YOLOv8 ML stack | `setup/`, `docs/` |
| 5–7 | Dataset download/merge, YOLOv8 fine-tuning on sunflower data, full-stack verification | `setup/`, `fyp_training/` |
| 8–12 | Single-drone pollination FSM → first multi-drone field demo | `setup/`, `mission_outputs/stage12_*` |
| 14 | 3-drone swarm with visual-servoing approach trajectory (approach → lower → encircle → touch), YOLO field view, simulation video | `setup/`, `launch_stage14.sh`, `mission_outputs/stage14/` |
| 15 | Clean rewrite: 4 source files, multi-day bloom cycle, mTSP routing, CSV event stream | `stage15_official/` |
| 16 | "No-cheat" sensing (camera + ToF only), provisional-vs-confirmed map separation, live web dashboard | `stage16_official/Stage16/` |

Per-stage details: `setup/README_stage14.md`, `stage15_official/README.md`, and `docs/SETUP_GUIDE.md` (the full Stage 1–7 installation walkthrough).

---

## Mission Output Data

Every run logs four files into `mission_outputs/<stage>/` (path set in each stage's config under `logging.log_directory`):

| File | Contents |
|---|---|
| `events_<run>.csv` | `ts, day, drone_id, type, payload_json` — mission_start, survey_done, merge_complete, routes_assigned, pollination_success, day_complete, … |
| `pollination_<run>.csv` | One row per pollinated flower: `run_id, day, drone_id, landmark_id, x, y, pollinated_at` |
| `telemetry_<run>.csv` | 2 Hz per-drone state, x/y/z pose, and battery |
| `mission_<run>.txt` | End-of-run summary (days, drones, landmarks, pollinated total, duration) |

Raw sensor streams (camera frames, ToF scans) are processed in memory and **not** persisted; record them with `ros2 bag record` on `/drone_N/camera/image_raw` and `/drone_N/tof_brush/scan` if needed.

---

## ROS 2 Package

`ros2_ws/src/precision_pollination/` contains the workspace nodes used by the launchers: `offboard_mission_controller.py`, `swarm_drone_controller.py`, `swarm_coordinator.py`, `swarm_monitor.py`, and supporting utilities.

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Vision & Datasets

YOLOv8 is fine-tuned on merged sunflower datasets (Roboflow, Zenodo NAB, Kaggle); bloom-stage reference images are in `Sunflower_stages/` (YoungBud, MatureBud, EarlyBloom, Healthy, Wilted), and detection stills from real footage are in `real_footage/`.

---

## Reference

Visual-servoing approach trajectory based on Hulens et al. (2022), Fig. 10 (approach → lower → encircle → final touch).

## License

See [LICENSE](LICENSE).
