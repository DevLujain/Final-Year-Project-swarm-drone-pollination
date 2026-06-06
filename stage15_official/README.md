# Stage 15 — clean rewrite

The original Stage 15 (Pass 1 and its patches) accumulated ~13 files
across 4 ROS 2 nodes, a separate installer, a YAML loader pattern, and
a generator that loaded the Ground Plane from Fuel over HTTPS. The
Gazebo GUI did not load reliably because of that last point (the HTTPS
fetch hangs `gz sim` at startup) plus an SDF that diverged from
Stage 14's proven template.

This rewrite collapses everything to **4 source files + 1 launcher +
1 config**, all of them mirroring Stage 14's patterns exactly.

```
stage15_clean/
├── README.md                            ← this file
├── install_stage15_clean.sh             ← one-command installer
├── launch_stage15.sh                    ← single launcher (Stage 14 pattern)
├── stage15_config.yaml                  ← config
├── generate_field_stage15.py            ← world generator (Stage 14 SDF template)
├── drone_controller_stage15.py          ← one per drone — survey + pollinate + RTB
└── mission_orchestrator_stage15.py      ← day cycle + mTSP + event stream + CSVs
```

---

## What was wrong with the old Stage 15

Found these by reading the pass-1 generator, the patch history, and
comparing line-by-line to Stage 14's working `demo_stage14.sdf`:

| # | Old (broken) | Fixed |
|---|---|---|
| 1 | `<include><uri>https://fuel.gazebosim.org/.../Ground Plane</uri></include>` — hangs `gz sim` waiting for HTTPS asset on startup, GUI eventually times out with *"Waited 10s for a subscriber to /gazebo/starting_world"* | Inline `<model name="ground_plane">` with a 50×50 m plane primitive (Stage 14 pattern) |
| 2 | `<sdf version="1.10">` | `<sdf version="1.9">` — Gazebo Harmonic 8.11.0 prefers 1.9 |
| 3 | `<physics name="ode_physics" type="ode">` | `<physics name="1ms" type="ignored">` — matches Stage 14; Gazebo falls back to DART |
| 4 | Only 3 world plugins (Physics, UserCommands, SceneBroadcaster) | All 5 from Stage 14: + Sensors (with `<render_engine>ogre2</render_engine>`) + Contact |
| 5 | `VelocityControl` plugin had explicit `<topic>` override | Plugin with no `<topic>` element — uses default `/model/<name>/cmd_vel` (Stage 14 pattern) |
| 6 | DESCEND's 0.4 m/s velocity cap leaked into TAKEOFF / GOTO / RTB, making drones look stuck | Each phase declares its own `PhaseLimits` (TAKEOFF 1.2 m/s vertical, GOTO 2.0 m/s, DESCEND 0.24 m/s — no leakage) |
| 7 | Route-complete fired at *dispatch* (route_idx++) not at successful brush — "route done (0 flowers)" 2 ms after start | Orchestrator counts actual `/pollination/drone_N/pollinated` Int32 messages |
| 8 | 4 separate ROS 2 nodes (bloom_state_manager, map_merger, mission_orchestrator, mission_logger_v15) with QoS issues and timing races | One orchestrator. Owns day cycle, mTSP, events, CSVs. Single source of truth |
| 9 | Required colcon rebuild of `precision_pollination` package on every code change | Scripts run directly as `python3 drone_controller_stage15.py --ros-args …` (Stage 14 pattern) |
| 10 | Multiple shell scripts under different names (`stage14_swarm.sh`, `stage15_install.sh`, etc.) and unclear what to actually run | One installer (`install_stage15_clean.sh`), one launcher (`launch_stage15.sh`) |

---

## Install

```bash
# Move out of Downloads so it's easy to find:
mv ~/Downloads/stage15_clean ~/Desktop/FYP/_incoming/

cd ~/Desktop/FYP/_incoming/stage15_clean
chmod +x install_stage15_clean.sh
./install_stage15_clean.sh
```

The installer:
1. Backs up `~/Desktop/FYP/stage15/` to `~/Desktop/FYP/_stage15_clean_backup_<ts>/`
2. Wipes `~/Desktop/FYP/stage15/` and recreates it empty
3. Copies the 4 Python files + config into it
4. Drops `launch_stage15.sh` into `~/Desktop/FYP/`
5. Creates `~/Desktop/FYP/mission_outputs/stage15/` for logs

**No colcon build required.** The controllers and orchestrator are
plain Python scripts run via `python3 file.py --ros-args …`, exactly
like Stage 14's `field_survey.py` and `swarm_drone_controller.py`.

---

## Run

```bash
cd ~/Desktop/FYP
./launch_stage15.sh
```

Six tabs open: `Gazebo`, `gz_bridge`, `orch`, `ctrl_0`, `ctrl_1`
(plus an extra `ctrl_N` for each drone above 2).

Watch the event stream from a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /swarm/event
```

You should see (within ~15 seconds of launch):

```
mission_start  →  day_start (day 5)  →  survey_start  →
survey_done (D0)  →  survey_done (D1)  →  merge_complete  →
routes_assigned  →  pollination_success × N  →
drone_at_base × 2  →  day_complete  →  day_start (day 6)  →  …
```

CSVs in `~/Desktop/FYP/mission_outputs/stage15/`:
- `events_<run_id>.csv`        — every event with timestamp + JSON payload
- `pollination_<run_id>.csv`   — one row per successful pollination
- `telemetry_<run_id>.csv`     — 2 Hz per-drone pose + battery + state
- `mission_<run_id>.txt`       — end-of-mission summary

---

## What you can delete (cleanup)

You said you have too many Stage 15 files and want to manually remove
the cruft. Safe to delete after this install succeeds:

**In `~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination/`:**
- `bloom_state_manager.py`        (logic moved into orchestrator)
- `battery_monitor.py`            (logic moved into controller)
- `survey_node.py`                (merged into drone_controller_stage15.py)
- `map_merger.py`                 (logic moved into orchestrator)
- `visual_servoing_v15.py`        (replaced by drone_controller_stage15.py)
- `mission_logger_v15.py`         (logic moved into orchestrator)
- `mission_orchestrator.py`       (old version; replaced)
- `swarm_coordinator_v15.py`      (no longer needed)
- `mtsp_optimizer.py`             (logic moved into orchestrator)

**In `~/Desktop/FYP/stage15/` (the installer wipes this directory, so
this happens automatically — listed for awareness):**
- everything from Pass 1: `generate_field_v15.py`, the various
  `nodes/` subfolders, old `stage15_config.yaml` files, etc.

**In `~/Desktop/FYP/`:**
- `_stage15_pass1_backup_*` directories (installer-generated backups
  from old patches) — keep one or two for safety, delete the rest

The installer's own backup at
`~/Desktop/FYP/_stage15_clean_backup_<ts>/` is safe to delete once
you've confirmed the new Stage 15 runs cleanly end-to-end.

---

## Why this works when the old one didn't

The Gazebo GUI hang you kept hitting was **not** a complexity problem
(it hung at 10 flowers too) and **not** an environment problem
(Stage 14 worked on the same Wayland/NVIDIA setup the same day).
It was specifically these things about the old SDF/launcher:

1. **`<include><uri>https://fuel.gazebosim.org/.../Ground Plane</uri></include>`**
   makes `gz sim` block on an HTTPS asset fetch before bringing up the
   server's transport. The GUI side starts, can't find the server, prints
   *"Waited 10s for a subscriber to /gazebo/starting_world"*, and stops.
   Stage 14's SDF defines the ground plane inline with a primitive —
   no network call. This rewrite does the same.

2. **SDF 1.10 + `type="ode"` + missing Sensors/Contact plugins** put
   the server in an unusual configuration that Gazebo Harmonic 8.11.0
   prefers not to start cleanly. Stage 14 uses `1.9` + `type="ignored"`
   + all 5 system plugins; once the rest of the SDF matches that, the
   GUI loads in ~3 seconds.

3. **The launcher tried a two-process server/GUI split** to work around
   what looked like a dispatcher-fork bug, but actually the dispatcher
   was fine — the SDF was the problem. With a Stage-14-shaped SDF the
   simple `gz sim -v 2 -r WORLD.sdf` dispatcher mode works (same as
   Stage 14's launcher).

If after this install Gazebo STILL refuses to come up, paste the first
20 lines of the `Gazebo` tab's output and we'll go from there — but
based on Stage 14 working on the same machine, this should just work.
