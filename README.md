# blueboat_sss_sim — GPS-anchored worlds + simulated Side Scan Sonar

> **Frozen 2026-11-18** as a model-conditional tool: tag `sim-freeze-v1.0-2026-11-18`, code digest
> `cc29e08780aac4cd` (`python3 -m blueboat_sss_sim.core.provenance --check`). The acceptance protocol's literal
> verdict is **NOT READY**. Simulated results are valid only inside the envelope of
> [`../SIM_VALIDITY_CONDITIONS.md`](../SIM_VALIDITY_CONDITIONS.md) (superproject root), and every result
> sentence starts "In a calibrated simulation environment, …". Freeze card with digests and defect sizes:
> `~/ros2_ws/sim_runs/sim_acceptance/10_evidence/SIM_FREEZE_CARD.md`. History: [docs/HISTORY.md](docs/HISTORY.md).

Two things, cleanly separated:

1. **World Builder** — a PySide6 wizard that creates **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths:
   draw the world limits and reflective harbour walls on a satellite map,
   author the bathymetry (realistic harbour / river / beach / canal
   presets, or real soundings from CSV / GeoTIFF), scatter litter objects
   from the object catalogue (object set version 2: 11 classes since 2026-11-15; clustered, sized and buried by
   simple distributions), preview, save. Worlds land in
   `~/worlds/<path_name>/<world_name>/` and are **immutable** — every
   save, edited or not, creates a new folder. Harbour campaign worlds are built headless by `worldgen_campaign`.
2. **Simulated SSS** — a physically motivated Omniscan 450 simulation
   publishing a **byte-identical drop-in replacement** of the real
   `sss_node.py` interface (profiles, raw Ping-Protocol frames,
   ground-truth contacts), calibrated against the field corpus. The production sensor is the GPU **ray-cast**
   with the profile `raycast_harbour_fast_sonar.yaml`; the geometric renderer is legacy.

Purely additive — no neighbour package is modified. Missions belong to
MCS/Control: this package serves worlds and sonar data, nothing else.

**Step-by-step tutorial** (a new world to an analysed run, both renderers): [docs/TUTORIAL.md](docs/TUTORIAL.md).

## Quick start

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash
python3 -m blueboat_sss_sim.core.provenance --check      # must give code digest cc29e08780aac4cd

# Harbour campaign worlds, headless (from the package source root blueboat_sss_sim/)
ros2 run blueboat_sss_sim worldgen_campaign --spec config/campaign_example.yaml --seeds 1,2,3 --out ~/worlds/campaign

# Build a world by hand (GUI; needs `pip install PySide6`)
ros2 run blueboat_sss_sim world_builder

# Inspect it in Gazebo alone
ros2 launch blueboat_sss_sim sim_world_launch.py world_dir:=$HOME/worlds/<path>/<world>

# Gazebo + robot + sonar, ray-cast production sensor, MPC on the identified plant
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<path>/<world> \
    with_control:=true trajectory_file:=<deployed path yaml> sim_config:=field_equivalent \
    sonar_renderer:=raycast sonar_config:=raycast_harbour_fast_sonar.yaml real_time_factor:=0.5
```

Recording a run from the command line (no clicking): superproject `tools/sim_record/README.md`. Offline
verification without ROS, from `blueboat_sss_sim/`: `python3 -m test.smoke_test` (about 540 checks, about
260 s on the system python; visible SKIPs without the field corpus or the BlueBoat-SSS checkout). GUI checks:
`QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke`. Calibration tooling:
`python3 -m blueboat_sss_sim.calibration --help`.

## The ray-cast sensor

A 3-D ray-cast sensor (`sonar/raycast/`, on the GPU through NVIDIA Warp; `warp-lang==1.19.0` importable by
`/usr/bin/python3` and in the ROS venv). It renders a ping far more slowly than the 50 ms ping period, so a
ray-cast run slows the **whole simulation** onto one clock, and Gazebo pauses while a slow ping renders. It still
pings at 20 Hz in sim time, and the `.svlog`, the GCS sessions and the seabed images are those of a real-speed
run. In the MCS App: **Sonar model** *Ray-cast* with the harbour profile and a **Slow-down**. Profiles:
`raycast_harbour_fast` = production; `raycast_harbour_live` = slow reference; `…_flash_narrow` / `…_flash_wide`,
`…_corner_low` / `…_corner_high` = bracket ends. The harbour profiles publish ground-truth contacts (since
2026-11-11), but **a contact is never a label**: labels come from the rendered picture and the world metadata.

**Range.** Results use 20 m. The GPU working set grows with the range (RTX 4060, 8 GB, harbour worst case: 2.3 GB
at 20 m, 3.6 GB at 30 m, 5.7 GB at 35 m, 6.3 GB at 40 m, out of memory at 50 m: CUDA error 101, `ray-cast render
failed (…); cycle skipped`). Never use more than 40 m, the GCS range control included. Details:
`blueboat_sss_sim/docs/engineering/raycast_assessment.md` §13.7.

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
| [docs/TUTORIAL.md](docs/TUTORIAL.md) | the step-by-step tutorial: path, world, Gazebo, recording, results, ray-cast |
| [docs/HISTORY.md](docs/HISTORY.md) | dated decisions, the completion round, the freeze |
| [docs/architecture.md](blueboat_sss_sim/docs/architecture.md) | design principles, layout, dataflow, decision records |
| [docs/integration_guide.md](blueboat_sss_sim/docs/integration_guide.md) | install, launch composition, sim↔real swap, GPS contract |
| [docs/configuration_guide.md](blueboat_sss_sim/docs/configuration_guide.md) | every YAML knob (world / depth / sonar) + world-folder formats |
| [docs/topics.md](blueboat_sss_sim/docs/topics.md) | ROS interface + raw-frame byte map |
| [docs/sonar_model.md](blueboat_sss_sim/docs/sonar_model.md) | acoustic model, noise stack, walls/pontoons, explicit assumptions |
| [docs/seabed_model.md](blueboat_sss_sim/docs/seabed_model.md) | bathymetry presets, seabed layers, materials, measured seabed from a recording; geomorphology references |
| [docs/svlog_depth_survey.md](blueboat_sss_sim/docs/svlog_depth_survey.md) | how to survey a site so a `.svlog` can become a world's depth and reflectivity |
| [docs/sea_state_model.md](blueboat_sss_sim/docs/sea_state_model.md) | current + waves: physics, presets, Gazebo coupling, references (DOI), research log |
| [docs/sea_state_validation.md](blueboat_sss_sim/docs/sea_state_validation.md) | the same survey in calm water / current / waves / both, in Gazebo |
| [docs/developer_guide.md](blueboat_sss_sim/docs/developer_guide.md) | code tour, conventions, extension recipes, testing |
| [docs/roadmap.md](blueboat_sss_sim/docs/roadmap.md) | prioritized realism upgrades (a future simulator version) |
| [docs/engineering/raycast_design.md](blueboat_sss_sim/docs/engineering/raycast_design.md) | the ray-cast sensor: physics, paths, integration contract (§11) |
| [docs/engineering/raycast_assessment.md](blueboat_sss_sim/docs/engineering/raycast_assessment.md) | the ray-cast's calibration status, the harbour-first hybrid (§13), GPU memory (§13.7) |
| [docs/engineering/](blueboat_sss_sim/docs/engineering/) | field calibration: design, runbook, results (`calibration_results.md`) |
| [docs/literature/raycast_sonar/INDEX.md](blueboat_sss_sim/docs/literature/raycast_sonar/INDEX.md) | the ray-cast literature corpus |

The package docs under `blueboat_sss_sim/docs/` are installed by `setup.py` and are frozen with the code; some
of their status lines predate the freeze (the freeze card and the conditions file win).

## What's inside

| Package | Role |
|---|---|
| `core/` | shared types, geometry, **geodesy** (`geo.py`), trajectories, run **provenance** |
| `worldgen/` | scene model, terrain texture, **authored bathymetry** (`depth/`), object catalog + placement, campaign generator, SDF writer, **world folders** (`world_folder.py`), map exports |
| `sonar/` | calibrated acoustic renderer, noise, Ping-Protocol encoder, walls (quays/pontoons) + multipath; `sonar/raycast/` the ray-cast sensor (GPU) and its empirical correction layer |
| `calibration/` | field calibration against the `.svlog` corpus (`sss_calibration`): registry, manifests, the seabed engine, artifacts |
| `sea/` | ROS-free sea state: JONSWAP wave field, a seeded smooth Gauss-Markov current with weak eddies, hydrostatic wave force on the hull, wind, presets, plain-language options and timelines |
| `builder/` | the world-builder GUI (the only PySide6 code; zero ROS) |
| `analysis/` | ground-truth detection metrics, `.svlog` reader, sim-to-real calibration |
| `ros/` | `sss_sim_node` (the drop-in sonar), `mavros_shim_node`, `sea_state_node` (current + waves into Gazebo) |
