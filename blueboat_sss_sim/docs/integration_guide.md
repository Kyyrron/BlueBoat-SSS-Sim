# Integration guide

How to run `blueboat_sss_sim` alongside the existing BlueBoat stack, and
how sim ↔ real swapping works. Nothing in the neighbour packages is
modified.

## 1. Install

```bash
cd ~/ros2_ws/src
# (clone next to blueboat_description etc.)
cd ~/ros2_ws
rosdep install --from-paths src -yi          # numpy/scipy/yaml/PIL/simple_launch
colcon build --packages-select blueboat_sss_sim
source install/setup.bash
pip install 'PySide6>=6.5'                   # world_builder GUI only
```

Requires the existing `blueboat_interfaces` package (for
`OmniscanProfile`) — already in your workspace. Everything except the GUI
runs without PySide6. The optional ray-cast sensor (§2b) also needs an NVIDIA GPU and
`warp-lang==1.19.0` importable by `/usr/bin/python3`, the interpreter `sss_sim_node` runs on
(`pip install --user --break-system-packages warp-lang==1.19.0`), and in the ROS venv
(superproject `requirements.txt`).

## 2. Quick start

```bash
# 1. Build a world around an MCS GPS-anchored path (GUI wizard)
ros2 run blueboat_sss_sim world_builder
#    -> ~/worlds/<path_name>/<world_name>/

# 2. Inspect the world (optional)
ros2 launch blueboat_sss_sim sim_world_launch.py world_dir:=$HOME/worlds/<p>/<w>

# 3. Dev end-to-end: Gazebo + robot + sonar + MAVROS shim
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<p>/<w>
#    add the control stack (dev convenience; operational missions are MCS's):
#    with_control:=true trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<p>.yaml
```

Headless alternative to the GUI: write a world YAML (see
`configuration_guide.md`) and `ros2 run blueboat_sss_sim generate_world
--config my_world.yaml --out ~/worlds/dev/w1`.

## 2b. The ray-cast sensor in a slowed simulation (2026-11-07)

The launches take `sonar_renderer:=raycast`, together with `real_time_factor:=` (0.01–1.0, default 1).
- `real_time_factor` is for `full_mission_launch.py`, which appends `gz sim -z <rtf / max_step_size>`.
- The whole graph then runs on `/clock`:
  - `master_control` and `simulation_interface` with `time_source: ros`;
  - the GCS's `sss_processor_node` with `use_sim_time` (auto, when `/clock` exists at its start);
  - Perception with `use_sim_time:=true`;
  - the GCS and MCS on clocks paced by `/clock`.
- So the ray-cast pings at 20 Hz in **sim** time, and everything recorded is what a real-speed run records.

**Profile resolution.** An explicit `sonar_config:=` wins. Otherwise `sonar_renderer:=raycast` resolves
`config/raycast_default_sonar.yaml`, the provisional physical section (`configuration_guide.md` §2c).

**The MCS App** (launch dialog, Gazebo simulation with a personalized world):
- **Sonar model: Ray-cast** and **Slow-down** emit `sonar_renderer:=raycast real_time_factor:=<f>`.
- **Extra args** are appended last, so `sonar_config:=<profile>` there selects another ray-cast profile, for
  example `~/calib/raycast/correction/raycast_harbour_hybrid_v2_sonar.yaml`, the harbour hybrid.
- The empty world has no sonar, so the run then falls back to geometric at x1.

**The GCS App** needs nothing special. Start it after the simulation, as usual, so that its processor sees
`/clock`. Then press START and Record: the session's `.svlog` and seabed images are written as in a real-speed run.

**Verified end to end on 2026-11-07** through the MCS App's own `LaunchManager` and the GCS App's own window
(offscreen; `~/ros2_ws/sim_runs/raycast/e2e/`). Setup:
- Shiraishijima harbour world, MCS's Shiraishijima lawnmower deployed path, PID;
- ray-cast with the harbour hybrid v2 profile, 20 m / 1200 bins, x0.03;
- Gazebo headless for the test.

Results:
- 322 pings at 20.3 Hz in sim time;
- 3 seabed images (schema 5, with metadata and `_world.npz`);
- the `.svlog` adopted into the session;
- the waterfall, the mosaic, and Perception's detections reaching the GCS;
- 1.5 GB of GPU memory;
- a clean MCS stop with no leftover process.

The same harness with the geometric renderer at x1 also gives 20.3 Hz and 3 images.

**One difference.** The processor's bottom tracker reads the ray-cast's first return 1.4–1.6 m late, jumping by
metres; with the geometric renderer it is within 2 cm. That shifts the images' georeference near nadir
(`raycast_assessment.md` §13.6).

**Factor.** The node logs the factor it needs when a tick lands more than 1.5 ping periods late. Measured values:
- flat 5 m world: x0.08 held 20 Hz;
- harbour at 20 m: x0.03 held about 20 Hz.

**GPU memory against range** (harbour worst case, one renderer, RTX 4060 8 GB):

| range | 20 m | 25 m | 30 m | 35 m | 40 m | 50 m |
|---|---|---|---|---|---|---|
| peak | 2.3 GB | 3.1 GB | 3.6 GB | 5.7 GB | 6.3 GB | out of memory (CUDA error 101) |

- Keep the ray-cast at **≤ 30 m**; 20 m is the survey setting.
- Above 40 m it fails on an 8 GB GPU. A failed render makes the node skip that ping cycle.
- Keep the GCS's **Apply range** within this limit too. It changes the simulated sonar's range: the node
  re-reads `range_length_mm` on the re-enable, and the ray-cast rebuilds its renderer for the new acquisition.
  For a whole run, `range_length_mm:=` in Extra args does the same.

## 3. The GPS contract (what other modules may rely on)

* World (0, 0) is the **spawn origin**: the source path's first waypoint
  (or the drawn area's centre when the path start lies outside the
  limits) — always inside the world, which is where the boat spawns
  (`upload_rov_launch.py` / MCS `Sim_launch.py` hard-code (0, 0)). The
  world's `geo_anchor` is that point's GPS; `metadata.yaml` records the
  design offset (`world = design − design_offset`), and
  `source_path_world.yaml` holds the path already re-expressed in world
  coordinates (first waypoint at (0, 0)).
* The conversion is the project-wide spherical-equirectangular pair
  (R = 6378137, longitude scaled by cos(lat0)) — the same math as MCS
  `mcs/core/geo.py` and the GCS's `mapping/geo.py`. In this package it
  lives only in `blueboat_sss_sim/core/geo.py`. Because every cross-module
  handoff goes through lat/lon, the frame re-basing is invisible to
  MCS/GCS: GPS ground truth is globally identical whatever the local
  frame.
* Ground truth in GPS: `metadata.yaml` lists the anchor, the spawn, the
  limit rectangle's four corners (SW/SE/NE/NW), every wall's endpoints and
  every object's position as `lat`/`lon` alongside the local values.
* `world.sdf` carries `<spherical_coordinates>` (EARTH_WGS84, ENU) at the
  anchor.
* `mavros_shim_node` publishes `/mavros/global_position/global`,
  `compass_hdg`, `imu/data`, `local_position/pose` and
  `local_position/velocity_local` from
  `/blueboat/odom`; `sss_sim_launch.py` sets its origin from the world's
  anchor automatically (`sim_origin_lat/lon:=` overrides; an unanchored
  world falls back to the legacy 43.6961/7.3080 with a warning). The GCS's
  GPS-anchored map therefore anchors at the world's true location.

## 4. Composition with the existing launch files

`full_mission_launch.py` starts Gazebo on the **world folder's**
`world.sdf` itself, because `blueboat_description/world_launch.py`
hard-codes the stock world and declares no `world` argument. It reproduces
that launch file's responsibilities against the world: `sl.gz_launch`
(which also registers the world name `generated_ocean` the model bridges
resolve against), the `/clock` and `/ocean_current` bridges, and
`blueboat_description/upload_rov_launch.py` for the robot spawn and its
bridges. No file in `blueboat_description` is modified. The spawn pose is
whatever `upload_rov_launch.py` defaults to — world (0, 0) heading east, the
same on every run; this launch declares no spawn argument.

`with_control:=true` also forwards two run-identity arguments to the control
stack, spelled and defaulted as in `Sim_launch.py`: `note:=` (default `sim`)
names the position log `{date}-{note}-poslog.csv` in `simulation_interface`,
and `data_dir:=` (default empty) is its root. Passing a note is what lets a
sequence of simulated runs be told apart afterwards, exactly as in a real-robot run.
`use_stock_world:=true` loads the stock description world instead — the
boat drives, but the seabed and objects the sonar renders against are not
in the Gazebo scene.

The generated world uses `gz-sim-*` plugin names (Garden/Harmonic) by
default; `generate_world --plugin-prefix ignition` targets Fortress.

### Sea state (current + waves)

Both launch paths start `sea_state_node` (`launch/sea_state_launch.py`,
included by `full_mission_launch.py`): `sea_current:=` / `sea_waves:=`
presets with `sea_current_from_deg:=` / `sea_waves_from_deg:=` compass
FROM bearings, or a `sea_schedule:=` timeline; the defaults
(`none` / `calm`) add no sea, but the node still floats the hull on the
catamaran hydrostatics (every run since 2026-10-09,
`docs/sea_state_model.md` §5). The current rides the bridge both
launch paths already had (ROS `/current` → gz `/ocean_current`, consumed
by the boat's `Hydrodynamics` plugin); the waves are a one-shot wrench
per physics step through gz-sim's `ApplyLinkWrench`, which the node
injects into a running world when the world file lacks it (detected by
the system's own `/world/<w>/wrench/persistent` — `/wrench` is advertised
by the launch's bridge itself) — so it also runs **standalone next to
BlueBoat-Control's stock `Sim_launch.py`** (verified live 2026-10-09):

```bash
ros2 launch blueboat_sss_sim sea_state_launch.py world_name:=ocean \
    sea_current:=strong sea_current_from_deg:=90 sea_waves:=slight sea_waves_from_deg:=45
```

The MCS spawns the same launch as a companion process for **every**
empty-world simulation (since 2026-10-09 a calm, no-current choice too,
then with only `world_name:=ocean`, so that path floats the same hull),
and in a personalized world passes the sea to
`full_mission_launch.py` — since 2026-10-08 always as a scenario file
(`sea_schedule:=…/last_launch.yaml sea_seed:=…`, the plain-language keys
of `docs/sea_state_model.md` §4.3, resolved here). For scripted runs
`full_mission_launch.py headless:=true` starts the Gazebo server only (the
default `auto` runs the ray-cast's server so too, with the GUI as a separate,
respawning `gz sim -g` client since 2026-11-10: the Gazebo GUI can segfault at
start-up in gz-transport's discovery `getenv`, and as a client its crash cannot
stop the server; if Gazebo itself exits, the launch logs why and shuts down);
`test/gazebo_sea_test.py` uses it to run and check the same survey in four
sea states (`docs/sea_state_validation.md`). The situation is
modifiable at run time on `/sim/sea_state/command` and readable on
`/sim/sea_state` (`docs/topics.md`, `docs/sea_state_model.md` §9).

## 5. Missions: this package no longer serves `/path_request`

Missions belong to MCS/Control. `blueboat_control`'s own `path_generation`
is the only `/path_request` server, and its `from_yaml` branch consumes an
MCS deployed trajectory directly (`trajectory:=from_yaml:<abs path>`; the
file is watched, so it may be deployed after launch — the boat holds
station until it appears). `full_mission_launch.py with_control:=true
trajectory_file:=<yaml>` wires exactly that, plus `path_publisher` with
its `total_time` window sized from the trajectory (either schema:
`points: [[t,x,y,yaw]]` or `waypoints` + `speed`). For a quick dev run
without MCS, every world folder ships its own ready trajectory:
`trajectory_file:=<world_dir>/source_path_world.yaml` (the source path in
world coordinates, starting at the spawn).

## 6. Sim ↔ real swap

The sonar interface is identical, so the swap is one node choice:

| | Real-robot run | Simulation |
|---|---|---|
| sonar node | `sss_node.py` (drives the two Omniscan units over the BlueBoat Base Station WiFi) | `blueboat_sss_sim sss_sim_node` |
| pose source | `/blueboat/odom` from `robot_interface` (MAVROS) | `/sim/sea_state/boat_pose` (Gazebo's 100 Hz model pose), `/blueboat/odom` from Gazebo as the fallback |
| everything downstream | unchanged | unchanged |

The `raw` topics remain byte-valid Ping Protocol, so `.svlog`
reconstruction and any Cerulean tooling work on simulated data too. The
only observable differences: the extra `~/ground_truth/contacts` and
`~/ground_truth/sensor` topics (additive) and simulated `timestamp_ms`
starting at first enable. With the ray-cast sensor (§2b) there are also:
- the slowed sim clock;
- no contacts;
- a first bottom return that the processor tracks 1.4–1.6 m late.

## 7. Standalone sonar (no full stack)

Drive the boat any way you like and run just the sonar + shim:

```bash
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<p>/<w>
```

`auto_ping:=false` to enable pinging manually, exactly as an operator
would. `sonar_config:=` names a different acquisition profile (default:
the package `config/default_sonar.yaml`; a legacy bundle's frozen
`sonar.yaml` wins when present; `sonar_renderer:=raycast` without `sonar_config:=` resolves
`config/raycast_default_sonar.yaml`); the six device parameters remain
individually overridable (`gain_index:=-1`; a `range_length_mm:=30000` override
is a diagnostic — surveys keep the 20 m default).

## 8. Measuring a run

Detection metrics come out of the renderer's own ground truth, with no
ROS and no field session:

```bash
# Replay a trajectory over a world -- reproducible from the seed alone
ros2 run blueboat_sss_sim mission_metrics --world ~/worlds/<p>/<w> \
    --trajectory ~/.config/blueboat_mcs/trajectories/<p>.yaml --out ~/metrics/w

# The path a real run actually tracked, from its recorded .svlog
ros2 run blueboat_sss_sim mission_metrics --world ~/worlds/<p>/<w> \
    --source svlog --input run.svlog --out ~/metrics/w_real

# What the node actually published (no pose -> aspect unavailable)
ros2 run blueboat_sss_sim mission_metrics --world ~/worlds/<p>/<w> \
    --source jsonl --input contacts.jsonl --out ~/metrics/w_pub
```

Writes `metrics.json` + `metrics.md` into `--out`; the world and the
recording are only ever read. Every object in `scene_manifest.yaml` is
accounted for; the criterion in force (`geometric`/`resolved`/`shadowed`)
is named in both files; `content_digest` excludes provenance so a world
regenerated from its state compares equal. Legacy mission bundles work
without `--trajectory`/`--sonar` (their frozen copies are the fallback).

The `.svlog` path needs the mavlink `LOCAL_POSITION_NED` (id 150) track;
in simulation that comes from `mavros_shim_node` (on by default).

## 9. Verifying an install

`python3 -m test.smoke_test` from the package source root runs the whole
ROS-free pipeline (496 checks on the system python, 493 + one visible SKIP on the venv; geo → world folder → render → encode →
decode → metrics → calibration) and writes artifacts to
`/tmp/blueboat_sss_smoke/`. `QT_QPA_PLATFORM=offscreen python3 -m
test.gui_smoke` exercises the GUI headless.
