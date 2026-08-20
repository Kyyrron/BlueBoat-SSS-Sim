# Architecture

`blueboat_sss_sim` turns the existing BlueBoat Gazebo simulator into a synthetic
Side Scan Sonar experimentation platform. It is a single additive ROS 2
Python package: nothing in the existing simulator (`blueboat_description`,
`blueboat_control`, `blueboat_interfaces`) is modified.

## 1. Design principles

1. **Drop-in interface fidelity.** The simulated sonar node publishes the
   exact topics, message type, QoS, parameters and control semantics of the
   real `sss_node.py` (Cerulean Omniscan 450 SS pair), including a
   byte-valid Ping-Protocol `raw` stream. The downstream processing
   pipeline runs unmodified; swapping sim/real is a launch-file choice.
2. **Gazebo owns dynamics, the sonar node owns acoustics.** The sonar is
   *not* a Gazebo plugin. Gazebo remains authoritative for hull dynamics,
   thrust, and pose (consumed via `/blueboat/odom`); a ROS 2 Python node
   renders acoustics against the same procedurally generated scene. This
   gives full control over intensity, materials, shadows and noise, keeps
   the code Gazebo-version-independent, and makes the renderer swappable.
   The accepted trade-off — the acoustic scene is static during a run — is
   irrelevant for seabed litter surveys and documented in
   `sonar_model.md §7`.
3. **Single source of truth for the world.** One `SceneModel` is generated
   per run and emitted twice: as `world.sdf` (+ `seabed.stl`) for Gazebo
   visuals/physics, and as `scene.npz` + `scene_manifest.yaml` for the
   acoustic renderer and the auto-labeler. The two views cannot diverge.
4. **ROS-free core.** All science code (`core`, `worldgen`, `sonar`,
   `dataset`, `mission`) imports no ROS and is unit-testable anywhere
   (see `test/smoke_test.py`). The `ros/` directory is a thin shell.

## 2. Package layout

```
blueboat_sss_sim/
├── blueboat_sss_sim/
│   ├── core/        types.py (Pose3D, Ping, Side, GridSpec, PlacedObject…)
│   │                geometry.py (quaternions, heading conventions, bilinear)
│   ├── worldgen/    noise.py → terrain.py → objects.py → scene.py
│   │                sdf_writer.py (world.sdf + seabed.stl)
│   │                generate.py (generate_world CLI)
│   ├── sonar/       config.py (AcquisitionParams = real node params,
│   │                           SonarModelConfig = physics/noise knobs)
│   │                acoustics.py (backscatter, beam, TVG)
│   │                noise.py (speckle, gain drift, dropouts)
│   │                renderer.py (SonarRenderer ABC + GeometricRenderer)
│   │                encoder.py (u16 quantisation + Ping-Protocol frames)
│   ├── dataset/     waterfall.py (tiling) → labeler.py (YOLO boxes)
│   │                → exporter.py (Ultralytics layout)
│   ├── mission/     patterns.py (lawnmower/spiral/random/waypoints)
│   │                generate.py (generate_mission CLI → run bundle)
│   └── ros/         sss_sim_node.py       ← replaces sss_node.py
│                    dataset_recorder_node.py
│                    sss_path_generation.py ← replaces path_generation.py
│                    mavros_shim_node.py    (optional)
├── config/          default_world / default_sonar / default_mission / materials
├── launch/          sss_sim_launch, sim_world_launch, full_mission_launch
├── docs/            this documentation set
├── msg_reference/   OmniscanProfile.msg (reference copy only)
└── test/            smoke_test.py (offline end-to-end)
```

## 3. Dataflow

```
default_mission.yaml
        │  generate_mission
        ▼
mission bundle ── world.sdf + seabed.stl ──────────► Gazebo (Fortress)
  (one dir,   ── scene.npz + manifest ──┐                │ dynamics
   one seed)  ── trajectory.yaml ──┐    │                ▼
              ── sonar.yaml ──┐    │    │          /blueboat/odom
                              │    │    │                │
                              │    ▼    ▼                ▼
                              │  sss_path_gen      sss_sim_node ──► /side_scan_sonar/{port,starboard}/profile
                              │  (/path_request)   (render+noise    /side_scan_sonar/{port,starboard}/raw
                              │        │            +encode)        /side_scan_sonar/ground_truth/contacts
                              │        ▼                                   │                │
                              │  master_control /                          ▼                ▼
                              └─ path_publisher (unmodified)        existing pipeline   dataset_recorder
                                                                    (unmodified)        → YOLO dataset
```

## 4. The run bundle

`generate_mission --config config/default_mission.yaml --out runs/r1`
produces one directory containing *everything* a run needs, reproducible
from the seed alone: `world.sdf`, `seabed.stl`, `scene.npz`,
`scene_manifest.yaml`, `trajectory.yaml`, `sonar.yaml`,
`mission_snapshot.yaml`. Launch files take `mission_dir:=runs/r1` and
nothing else.

## 5. Key interfaces (extension points)

| Interface | Where | Replace to… |
|---|---|---|
| `SonarRenderer` ABC | `sonar/renderer.py` | plug a higher-fidelity acoustic model (ray tracer, multipath) with zero changes elsewhere |
| `Material` table | `worldgen/materials.py` + `config/materials.yaml` | tune or add seabed/object acoustic responses |
| Object `CATALOG` | `worldgen/objects.py` | add litter classes (mask primitive + material + size priors) |
| `build_pattern` | `mission/patterns.py` | add survey patterns |
| `WaterfallTileConfig` / `LabelConfig` | `dataset/` | change tiling or labeling policy |
| plugin prefix (`ignition`/`gz`) | `sdf_writer.py`, mission YAML | target Gazebo Garden/Harmonic |

## 6. Why not a Gazebo sensor plugin? (decision record)

A `gpu_ray`/lidar-based approach (as sketched in the legacy
`sonar_snippets.xacro`) yields ranges, not calibrated backscatter: no
material-dependent intensity, no speckle statistics, no shadow/highlight
contrast control, and it welds the code to one Gazebo rendering backend.
Since the deliverable is *imagery statistically usable for detector
training* and the vehicle is a surface craft whose pose Gazebo already
publishes, rendering acoustics CPU-side against the ground-truth
heightfield is both more faithful and more maintainable. The
`SonarRenderer` ABC preserves the option to reintroduce a GPU/plugin
backend later behind the same interface.
