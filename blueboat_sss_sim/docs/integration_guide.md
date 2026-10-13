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
runs without PySide6.

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
`full_mission_launch.py headless:=true` starts the Gazebo server only;
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
| pose source | `/blueboat/odom` from `robot_interface` (MAVROS) | `/blueboat/odom` from Gazebo |
| everything downstream | unchanged | unchanged |

The `raw` topics remain byte-valid Ping Protocol, so `.svlog`
reconstruction and any Cerulean tooling work on simulated data too. The
only observable differences: the extra `~/ground_truth/contacts` and
`~/ground_truth/sensor` topics (additive) and simulated `timestamp_ms`
starting at first enable.

## 7. Standalone sonar (no full stack)

Drive the boat any way you like and run just the sonar + shim:

```bash
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<p>/<w>
```

`auto_ping:=false` to enable pinging manually, exactly as an operator
would. `sonar_config:=` names a different acquisition profile (default:
the package `config/default_sonar.yaml`; a legacy bundle's frozen
`sonar.yaml` wins when present); the six device parameters remain
individually overridable (`range_length_mm:=30000 gain_index:=-1`).

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
ROS-free pipeline (434 checks — 431 + one visible SKIP without ROS; geo → world folder → render → encode →
decode → metrics → calibration) and writes artifacts to
`/tmp/blueboat_sss_smoke/`. `QT_QPA_PLATFORM=offscreen python3 -m
test.gui_smoke` exercises the GUI headless.
