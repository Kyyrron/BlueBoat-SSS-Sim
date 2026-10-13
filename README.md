# blueboat_sss_sim — GPS-anchored worlds + simulated Side Scan Sonar

Two things, cleanly separated:

1. **World Builder** — a PySide6 wizard that creates **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths:
   draw the world limits and reflective harbour walls on a satellite map,
   author the bathymetry (realistic harbour / river / beach / canal
   presets, or real soundings from CSV / GeoTIFF), scatter litter objects
   from a 29-class catalog (clustered, sized and buried by simple
   distributions), preview, save. Worlds land in
   `~/worlds/<path_name>/<world_name>/` and are **immutable** — every
   save, edited or not, creates a new folder.
2. **Simulated SSS** — a physically motivated Omniscan 450 simulation
   publishing a **byte-identical drop-in replacement** of the real
   `sss_node.py` interface (profiles, raw Ping-Protocol frames,
   ground-truth contacts), calibrated against the field corpus.

Purely additive — no neighbour package is modified. Missions belong to
MCS/Control: this package serves worlds and sonar data, nothing else.

## Quick start

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash

# Build a world (GUI; needs `pip install PySide6`, everything else doesn't)
ros2 run blueboat_sss_sim world_builder

# Inspect it in Gazebo alone
ros2 launch blueboat_sss_sim sim_world_launch.py world_dir:=$HOME/worlds/<path>/<world>

# Sonar + MAVROS shim over a running simulation
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<path>/<world>

# Dev end-to-end (Gazebo + robot + sonar; optional control stack driving
# the world's own ready-made trajectory — the source path, spawn-aligned)
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=$HOME/worlds/<path>/<world> \
    with_control:=true trajectory_file:=$HOME/worlds/<path>/<world>/source_path_world.yaml

# Headless world generation from a world YAML (no GUI; run from the
# package source root, blueboat_sss_sim/, where config/ lives)
ros2 run blueboat_sss_sim generate_world --config config/default_world.yaml --out ~/worlds/dev/w1
```

Offline verification without ROS (also wired as the session gate), from
`blueboat_sss_sim/`: `python3 -m test.smoke_test` (434 checks). GUI checks:
`QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`.

## GPS anchoring in one paragraph

A world's local ENU frame is its source path's design frame **re-based so
that world (0, 0) — where the boat always spawns — is the path's first
waypoint** (or the area centre when that lies outside the drawn limits):
the boat is guaranteed to start inside the world, at the survey start.
The world's `geo_anchor` is that point's GPS; `world.sdf` carries
`<spherical_coordinates>`, the manifest a `geo:` block, and
`metadata.yaml` lists the spawn, every object, wall and limit corner in
GPS (globally identical whatever the local frame — the GPS layer absorbs
the shift). `source_path_world.yaml` in each folder holds the path in
world coordinates, ready for the dev launch. The conversion is the
project-wide equirectangular pair (`blueboat_sss_sim.core.geo`, same math
as MCS `core/geo.py` and the GCS); the MAVROS shim publishes
`/mavros/global_position/global` about the world's own anchor
automatically.

## Documentation

| Doc | Contents |
|---|---|
| [docs/architecture.md](blueboat_sss_sim/docs/architecture.md) | design principles, layout, dataflow, decision records |
| [docs/integration_guide.md](blueboat_sss_sim/docs/integration_guide.md) | install, launch composition, sim↔real swap, GPS contract |
| [docs/configuration_guide.md](blueboat_sss_sim/docs/configuration_guide.md) | every YAML knob (world / depth / sonar) + world-folder formats |
| [docs/topics.md](blueboat_sss_sim/docs/topics.md) | ROS interface + raw-frame byte map |
| [docs/sonar_model.md](blueboat_sss_sim/docs/sonar_model.md) | acoustic model, noise stack, walls/pontoons, explicit assumptions |
| [docs/seabed_model.md](blueboat_sss_sim/docs/seabed_model.md) | bathymetry presets, seabed layers, materials, measured seabed from a recording; geomorphology references |
| [docs/svlog_depth_survey.md](blueboat_sss_sim/docs/svlog_depth_survey.md) | how to survey a site so a `.svlog` can become a world's depth and reflectivity |
| [docs/sea_state_model.md](blueboat_sss_sim/docs/sea_state_model.md) | current + waves: physics, presets, Gazebo coupling, references (DOI), research log |
| [docs/sea_state_validation.md](blueboat_sss_sim/docs/sea_state_validation.md) | the same survey in calm water / current / waves / both, in Gazebo: how to run it, what it shows, results |
| [docs/developer_guide.md](blueboat_sss_sim/docs/developer_guide.md) | code tour, conventions, extension recipes, testing |
| [docs/roadmap.md](blueboat_sss_sim/docs/roadmap.md) | prioritized realism upgrades |

## What's inside

| Package | Role |
|---|---|
| `core/` | shared types, geometry, **geodesy** (`geo.py`), trajectories |
| `worldgen/` | scene model, terrain texture, **authored bathymetry** (`depth/`), object catalog + placement, SDF writer, **world folders** (`world_folder.py`), map exports |
| `sonar/` | calibrated acoustic renderer, noise, Ping-Protocol encoder, walls (quays/pontoons) + multipath |
| `sea/` | ROS-free sea state: JONSWAP wave field, a seeded smooth Gauss-Markov current with weak eddies, hydrostatic wave force on the hull, presets, plain-language options and timelines |
| `builder/` | the world-builder GUI (the only PySide6 code; zero ROS) |
| `analysis/` | ground-truth detection metrics, `.svlog` reader, sim-to-real calibration |
| `ros/` | `sss_sim_node` (the drop-in sonar), `mavros_shim_node`, `sea_state_node` (current + waves into Gazebo) |
