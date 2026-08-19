# blueboat_sss_sim — Synthetic Side Scan Sonar platform for the BlueBoat simulator

Extends the existing BlueBoat Gazebo simulator into a synthetic SSS
experimentation platform: procedural shallow-water worlds with seabed
litter, a physically motivated Omniscan 450 simulation publishing a
**byte-identical drop-in replacement** of the real `sss_node.py`
interface, autonomous survey mission generation, and automatic YOLO
training-dataset export. Purely additive — no existing package is
modified.

## Quick start

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash
ros2 run blueboat_sss_sim generate_mission \
    --config $(ros2 pkg prefix blueboat_sss_sim)/share/blueboat_sss_sim/config/default_mission.yaml \
    --out ~/runs/r1 --seed 7 --speed 0.5

ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r1

ros2 run blueboat_sss_sim export_scene_maps --bundle ~/runs/r1
```

Dataset appears in `~/runs/r1/dataset/` (Ultralytics layout, finalized on
shutdown). Offline verification without ROS: `python3 -m test.smoke_test`.

## Documentation

| Doc | Contents |
|---|---|
| [docs/architecture.md](docs/architecture.md) | design principles, layout, dataflow, decision records |
| [docs/integration_guide.md](docs/integration_guide.md) | install, launch composition, sim↔real swap, batch generation |
| [docs/configuration_guide.md](docs/configuration_guide.md) | every YAML knob (world / sonar / mission / recorder) |
| [docs/topics.md](docs/topics.md) | ROS interface + raw-frame byte map |
| [docs/sonar_model.md](docs/sonar_model.md) | acoustic model, noise stack, explicit assumptions |
| [docs/developer_guide.md](docs/developer_guide.md) | code tour, conventions, extension recipes, testing |
| [docs/roadmap.md](docs/roadmap.md) | prioritized realism upgrades (multipath, SVP, …) |

## What's inside

* `worldgen/` — seed-reproducible worlds: terrain (slope, dunes, fBm
  roughness, material patches) + 13 configurable litter classes; emits
  `world.sdf`/`seabed.stl` for Gazebo **and** `scene.npz` for the acoustic
  model from one `SceneModel`.
* `sonar/` — heightfield-draping renderer (Lambertian backscatter,
  horizon-culled shadows, beam pattern with roll coupling, TVG residual),
  full noise stack (speckle, gain drift, dropped pings), byte-exact
  Ping-Protocol encoder (`parse_frame` round-trip verified against a real
  capture: 1262 B at 600 bins).
* `mission/` — lawnmower / spiral / random / waypoint patterns; one YAML →
  one self-contained, reproducible run bundle.
* `dataset/` — waterfall tiling, highlight+shadow YOLO auto-labeling from
  per-ping ground truth, Ultralytics export with deterministic train/val
  split.
* `ros/` — `sss_sim_node` (drop-in), `dataset_recorder_node`,
  `sss_path_generation` (RequestPath-compatible mission service),
  optional `mavros_shim_node`.

License: Apache-2.0.
