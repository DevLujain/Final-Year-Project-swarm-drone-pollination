# Stage 14 — Visual-Servoing Pollination Demonstration

This stage is a full rewrite. The earlier stage-14 attempt used PX4 SITL
to fly the drones; after seven patches it still could not arm reliably in
the multi-vehicle SITL configuration because of stacked preflight
failures (missing sensor plugins, GCS heartbeat requirement, versioned
`px4_msgs` mismatch, XRCE-DDS topic translation). Rather than continue
patching symptoms, stage 14 swaps the *actuator backend* for the demo:

* The same Figure-10 four-phase visual-servoing algorithm (approach →
  lower → encircle → pollinate) runs in Python as a PID-driven state
  machine — exactly the control logic stages 9-13 developed.
* The PID's output is a `geometry_msgs/Twist` published to Gazebo's
  velocity-control plugin (`/model/drone_*/cmd_vel`). Gazebo applies the
  velocity directly to the drone body, so the drones physically fly the
  trajectory and you can watch it in the Gazebo window.
* Position feedback is the real Gazebo model pose, bridged into ROS 2.

What you keep from stages 1-13: YOLO model, dataset, ROS 2 package
structure, dashboard, all the prior work. What changes for stage 14
only: the drone-control backend. The professor's request — *“physically
see the drones fly over the flowers to pollinate and then they go back
to where they are”* — is satisfied.

---

## Files in this directory

| File                                | Purpose                                              |
| ----------------------------------- | ---------------------------------------------------- |
| `demo_stage14.sdf`                  | Gazebo world (2 sunflowers, 2 drones, sector tiles, surveillance camera). |
| `visual_servoing_controller.py`     | One instance per drone. Implements the 4-phase Figure-10 trajectory plus take-off / return / land via PID-on-velocity. |
| `swarm_coordinator.py`              | Fires the synchronised start signal, aggregates per-drone state, declares mission complete. |
| `yolo_field_detector.py`            | Runs YOLOv8 (or a deterministic mock) on the surveillance camera feed and publishes annotated bbox visualisation. |
| `mission_logger.py`                 | Writes a 10 Hz CSV trace of poses + states + events for the report. |
| `stage14_clean_install.sh`          | Installer — wipes the old broken stage-14 files, installs everything, registers in `setup.py`, rebuilds the workspace, and drops a `launch_stage14.sh` into `~/Desktop/FYP/`. |
| `README_stage14.md`                 | This file.                                           |

---

## How to install

1. Put all six files above into a single folder (anywhere).
2. From that folder:
   ```bash
   bash stage14_clean_install.sh
   ```
   This kills any leftover PX4/gz processes, backs up the broken
   stage-14 files into `~/Desktop/FYP/_stage14_backup_<timestamp>/`,
   installs the new world + nodes, registers them, and rebuilds the
   workspace. Takes about 90 seconds.

## How to run

```bash
bash ~/Desktop/FYP/launch_stage14.sh
```

Eight terminal tabs open:

| Tab            | What it shows                                                       |
| -------------- | ------------------------------------------------------------------- |
| Gazebo         | The 3-D world. **This is the main visual.** Both drones lift off ~4 s after launch and fly the visual-servoing trajectory. |
| GZ Bridges     | `ros_gz_bridge` translating cmd_vel + pose + camera between gz and ROS 2. |
| D0 Controller  | State-machine + PID for Drone 0 (Sector 1).                         |
| D1 Controller  | State-machine + PID for Drone 1 (Sector 2).                         |
| Swarm Coord    | Fires the global start, prints aggregated swarm status.              |
| YOLO Detector  | Runs detection on surveillance camera, prints detections.           |
| Mission Log    | Writes CSV trace.                                                   |
| Events         | Live aggregated event feed — narrates each phase change.            |

To view the YOLO-annotated camera feed:

```bash
ros2 run rqt_image_view rqt_image_view /yolov8/annotated_image
```

To stop everything:

```bash
pkill -f gz; pkill -f ros2
```

---

## What you'll see, step by step

1. **t = 0**       — Gazebo window opens. Both drones sitting on their
                     base pads (blue pad for D0, orange for D1).
2. **t ≈ 4 s**     — Swarm coordinator fires `True` on
                     `/pollination/start`. Both drones leave READY,
                     enter TAKEOFF.
3. **t ≈ 4–10 s**  — TAKEOFF: drones climb vertically to cruise
                     altitude (z = 3 m) over their pads.
4. **t ≈ 10–18 s** — APPROACH: each drone flies horizontally to a point
                     2 m south of its assigned sunflower, yawing to
                     face it (matches paper Fig. 10 step 1).
5. **t ≈ 18–24 s** — LOWER: descend to flower-head height (z = 1.5 m)
                     while keeping x, y constant — paper Fig. 10
                     step 2.
6. **t ≈ 24–32 s** — ENCIRCLE: half-orbit around the flower at radius
                     1 m, always facing the flower — paper Fig. 10
                     step 3.
7. **t ≈ 32–36 s** — POLLINATE: close to 0.3 m from the flower head
                     ("pollination touch"). Event log emits
                     `✿ POLLINATION TOUCH ✿`.
8. **t ≈ 36–40 s** — RETREAT back to 2 m.
9. **t ≈ 40–48 s** — RETURN: climb to cruise altitude, fly back over
                     base pad.
10. **t ≈ 48–54 s** — LAND on the base pad.
11. **t ≈ 54 s**    — Swarm coordinator publishes
                      `★ SWARM MISSION COMPLETE ★`.

The two drones run in parallel so the wall-clock total is roughly one
minute.

---

## Outputs you can use in the report

* **Gazebo screen recording** — record the Gazebo window while
  `launch_stage14.sh` runs. The 4-phase Figure-10 trajectory is
  visible.
* **`rqt_image_view` capture** — the surveillance camera frame with
  YOLO bboxes around the sunflower heads.
* **CSV traces** — `~/Desktop/FYP/mission_outputs/stage14/`
  contains:
  * `stage14_trace_<timestamp>.csv` — 10 Hz pose + state log
  * `stage14_events_<timestamp>.csv` — phase-change + YOLO events
* **Plotting** — `pandas.read_csv(...)` the trace and plot D0_x/y vs
  D1_x/y to show the two trajectories side by side.

---

## How to defend this in viva

> "Stage 14 demonstrates the visual-servoing control algorithm in
> isolation from the PX4 SITL multi-vehicle stack. The four-phase
> trajectory (Hulens et al. 2022, Fig. 10) is implemented as a PID
> position controller whose output is sent to Gazebo's
> velocity-control plugin, allowing us to validate the algorithm's
> phase transitions, encircle geometry, and swarm synchronisation
> without the documented PX4 multi-vehicle arming instability."

That sentence is technically accurate, owns the design choice, and
points at a known issue (PX4 multi-vehicle SITL really is fragile —
this is well-documented in PX4 issue tracker).

---

## Tuning knobs

In `launch_stage14.sh` you can change controller ROS parameters per
drone:

```
-p cruise_z:=3.0         # how high the drones fly during transit
-p approach_radius:=2.0  # how far from the flower the APPROACH phase ends
-p encircle_radius:=1.0  # encircle radius
-p touch_radius:=0.30    # how close the rod gets to the head
-p start_delay:=0.5      # stagger drones so they don't move identically
```

In `visual_servoing_controller.py` you can change the PID gains
(`kp_xy`, `kp_z`, `kp_yaw`, `kd_xy`, `kd_z`) and the velocity
clamps (`vmax_xy`, `vmax_z`, `vmax_yaw`) for tighter or smoother
motion.

---

## Backup / rollback

The installer moved the old broken files into
`~/Desktop/FYP/_stage14_backup_<timestamp>/` rather than deleting
them. If you ever want to look at the previous attempt, they're
there. To roll back: copy them back out of the backup dir, but
honestly the new approach is what works.
