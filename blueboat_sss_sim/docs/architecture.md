# Architecture

`blueboat_sss_sim` is a single additive ROS 2 Python package with two
deliverables: a **World Builder** producing GPS-anchored Gazebo worlds,
and a **simulated Side Scan Sonar** that is a drop-in replacement for the
real Omniscan 450 pair. Nothing in the neighbour packages
(`blueboat_description`, `blueboat_control`, `blueboat_interfaces`, MCS,
GCS) is modified. Missions belong to MCS/Control — this package serves
worlds and sonar data and does not serve `/path_request`.

## 1. Design principles

1. **Drop-in interface fidelity.** The simulated sonar node publishes the
   exact topics, message type, QoS, parameters and control semantics of the
   real `sss_node.py` (Cerulean Omniscan 450 SS pair), including a
   byte-valid Ping-Protocol `raw` stream. The downstream processing
   pipeline runs unmodified; swapping sim/real is a launch-file choice.
2. **Gazebo owns dynamics, the sonar node owns acoustics.** The sonar is
   *not* a Gazebo plugin. Gazebo remains authoritative for hull dynamics,
   thrust, and pose (consumed via `/blueboat/odom`); a ROS 2 Python node
   renders acoustics against the same generated scene. The accepted
   trade-off — the acoustic scene is static during a run — is irrelevant
   for seabed litter surveys and documented in `sonar_model.md §7`.
3. **Single source of truth for the world.** One `SceneModel` is emitted
   twice: as `world.sdf` (+ `seabed.stl`) for Gazebo visuals/physics, and
   as `scene.npz` + `scene_manifest.yaml` for the acoustic renderer and
   ground-truth tooling. The two views cannot diverge.
4. **GPS anchoring is structural.** A world's local ENU frame *is* its
   source MCS path's design frame: the path's `geo_anchor` sits at world
   (0, 0). One geodesy module (`core/geo.py`, the project-wide
   equirectangular convention) feeds the manifest's `geo:` block,
   `world.sdf`'s `<spherical_coordinates>`, `metadata.yaml`'s per-object
   GPS ground truth, and the MAVROS shim's origin.
5. **ROS-free, Qt-free core.** All science code (`core`, `worldgen`,
   `sonar`, `analysis`) imports no ROS and no Qt and is testable anywhere
   (`test/smoke_test.py`). `ros/` is a thin shell; `builder/` is the only
   Qt code and imports no ROS (its `state.py`/`paths.py` are Qt-free so
   the smoke test drives the full build pipeline headless).

## 2. Package layout

```
blueboat_sss_sim/
├── blueboat_sss_sim/
│   ├── core/        types.py (Pose3D, Ping, Side, GridSpec, PlacedObject, Wall…)
│   │                geometry.py (quaternions, heading conventions, bilinear)
│   │                geo.py (equirectangular lat/lon ↔ EN, GeoAnchor, slippy)
│   │                trajectory.py (WaypointTrajectory, TimedTrajectory =
│   │                               MCS blueboat_trajectory/1, load_trajectory)
│   ├── worldgen/    noise.py → terrain.py (texture) → objects.py → scene.py
│   │                depth/   (authored bathymetry: ingest.py CSV/GeoTIFF,
│   │                          interpolate.py wall-aware fill, presets.py
│   │                          harbour/river/beach/canal + DEPTH_PRESETS,
│   │                          layers/ + compose.py / bake.py / masks.py /
│   │                          paint.py / recipes.py the layered seabed,
│   │                          svlog.py / smooth.py / coverage.py depth
│   │                          from a recording → docs/seabed_model.md)
│   │                materials.py (seabed + object materials, _ACOUSTIC)
│   │                material_map.py (baked material map)
│   │                sdf_writer.py (world.sdf + seabed.stl + spherical coords
│   │                               + wall collisions + invisible limits)
│   │                world_folder.py (~/worlds/<path>/<world>/ builder,
│   │                                 metadata.yaml, _i dedupe, library scan)
│   │                export_maps.py (gt maps + preview renderers)
│   │                generate.py (generate_world CLI)
│   │                solids.py (rotated 3-D primitives: z-buffer stamps,
│   │                           stable rest poses, settling on the seabed)
│   ├── sonar/       config.py / acoustics.py / multipath.py / noise.py /
│   │                renderer.py / facets.py (smooth-object Kirchhoff
│   │                patches, NC #17) / encoder.py
│   ├── sea/         ROS-free sea state: spectrum.py (JONSWAP/PM, dispersion)
│   │                wavefield.py / current.py (seeded, smooth Gauss-Markov
│   │                + weak eddies: a function of seed, t, x, y) / hull.py
│   │                (hydrostatic catamaran vs the Gazebo box) / presets.py
│   │                (sea_states.yaml, schedules, commands) / events.py
│   │                (Poisson wake groups) / motion.py (predict_pose,
│   │                HighRatePose, damping, simulate_3dof) / model.py /
│   │                markers.py   → docs/sea_state_model.md
│   ├── builder/     the World Builder GUI (PySide6; the ONLY Qt code):
│   │                state.py (BuilderState ↔ builder_state.yaml, Qt-free)
│   │                paths.py (MCS anchored-path discovery, Qt-free)
│   │                map/ (tiles.py fetcher, canvas.py drawing view)
│   │                pages/ (path, layout, depth, objects, place, preview)
│   │                layers/ (Screen 3's seabed layer editor)
│   │                layout.py (screen-size contract: ToolBar, ScaledImage,
│   │                split) / imaging.py
│   │                wizard.py / worker.py / forms.py / theme.py / app.py
│   ├── analysis/    svlog_reader.py (.svlog → pose track, read-only)
│   │                contacts.py (replay / svlog / jsonl → observations)
│   │                metrics.py (P(detect) vs range and aspect)
│   │                → report.py → cli.py (mission_metrics CLI)
│   │                calibration.py / calibration_cli.py (sim-to-real)
│   │                svlog_compare.py (timing/level/texture reductions)
│   │                svlog_digest.py / object_signature.py (.svlog reads)
│   │                bathymetry.py / backscatter.py / shading.py (depth,
│   │                reflectivity and relief from a recording)
│   │                sea_validation.py (a recorded Gazebo run or a field
│   │                .svlog → track / attitude / footprint / coverage /
│   │                waterfall / mosaic metrics; sss_sea_validation CLI)
│   │                wall_contrast.py (quay face / beyond / far contrasts)
│   ├── calibration/ registry.py (config/parameter_registry.yaml, drift check)
│   │                manifest.py (session manifests, whole-session splits)
│   │                quality.py (.svlog quality flags) / identifiability.py
│   │                artifact.py (versioned calibration artifacts,
│   │                effective profile) / __main__.py (sss_calibration CLI)
│   │                → docs/engineering/
│   └── ros/         sss_sim_node.py       ← replaces sss_node.py
│                    mavros_shim_node.py   (GPS/heading/IMU under MAVROS names)
│                    sea_state_node.py     (current → gz /ocean_current, wave
│                                           wrench → gz ApplyLinkWrench, status)
├── config/          default_world / shallow_water_world / enclosed_basin_world
│                    default_sonar / coverage_pass_sonar / enclosed_basin_sonar
│                    shallow_beach_sonar / fault_injection_sonar (test only)
│                    materials / materials_literature (override) /
│                    seabed_recipes / sea_states / sea_schedule_example
├── launch/          sss_sim_launch, sim_world_launch, full_mission_launch,
│                    sea_state_launch (also standalone next to Sim_launch.py)
├── docs/            this documentation set
├── msg_reference/   OmniscanProfile.msg (reference copy only)
└── test/            smoke_test.py (offline gate) · gui_smoke.py (offscreen Qt)
                     gazebo_sea_test.py (Gazebo sea gate, not in the hook)
                     fixtures/survey_lawnmower.yaml (frozen calibration context)
                     fixtures/render_baseline_v1.npz (render regression)
```

## 3. Dataflow

```
MCS anchored path (geo_anchor)          real soundings (CSV / GeoTIFF)
        │                                        │
        ▼            world_builder GUI           ▼
   Screen 1 path → Screen 2 limits+walls → Screen 3 depth+paint+texture (live previews) → Screen 4 objects (scatter) → Screen 5 hand placement → Screen 6 save
        │                                                              (or generate_world, headless)
        ▼
world folder ~/worlds/<path>/<world>/     [immutable, _i-deduped]
   ├─ world.sdf + seabed.stl ─────────────► Gazebo (Harmonic) ─► /blueboat/odom
   ├─ scene.npz + manifest (geo block) ──► sss_sim_node ─► /side_scan_sonar/{port,starboard}/{profile,raw}
   ├─ metadata.yaml (GPS ground truth)         │           /side_scan_sonar/ground_truth/contacts
   ├─ depth.npz · preview_objects.png          ├─► mavros_shim_node ─► /mavros/* (GPS about the anchor)
   └─ builder_state.yaml (Edit mode)           └─► sea_state_node ─► /current ─► gz /ocean_current (Hydrodynamics)
MCS sea choice (sea_schedule:= scenario / /sim/sea_state/command)   │            /world/<w>/wrench ─► gz ApplyLinkWrench (heave/pitch/roll)
                                                       └─► /sim/sea_state (JSON, latched) ─► MCS "SEA STATE" box
Missions: MCS deploys a trajectory → blueboat_control path_generation
(from_yaml) → master_control — unmodified, outside this package.
```

## 4. The world folder

`world_builder` (or `build_world_folder` programmatically) produces one
directory containing *everything* a run needs, reproducible from
`builder_state.yaml` + seed. Launch files take `world_dir:=…` and nothing
else. Folders are **immutable**: every save takes a fresh `_i`-suffixed
name; Edit mode loads the state, never rewrites the folder. Legacy
mission bundles still load everywhere (their frozen `sonar.yaml` /
`trajectory.yaml` are the resolver fallbacks).

## 5. Key interfaces (extension points)

| Interface | Where | Replace/extend to… |
|---|---|---|
| `SonarRenderer` ABC | `sonar/renderer.py` | plug a higher-fidelity acoustic model with zero changes elsewhere |
| `DEPTH_PRESETS` | `worldgen/depth/presets.py` | add a bathymetry pattern — fn + label + schema rows; the GUI form is generated |
| Object `CATALOG` | `worldgen/objects.py` | add litter classes (mask + material + priors + optional `fill_fraction_range` / `params_schema`); GUI, placement and metadata pick it up |
| `Material` table | `worldgen/materials.py` + `config/materials.yaml` | add seabed/object materials: a material's two-way `transmission` (0 = opaque, the default; `pp_shell` 0.11 is the flooded field box), its physical acoustic constants (`_ACOUSTIC`: density, sound speed, backing, wall — which give a smooth object its reflection coefficient, NC #17) and the literature-anchored override `config/materials_literature.yaml`. Never a per-object brightness |
| Facet layer | `sonar/facets.py` + `worldgen/solids.tessellate` | smooth solid objects scattered from their geometry (Kirchhoff patches + the incoherent residual); a new primitive needs a tessellator — `docs/sonar_model.md` §12 |
| `objects.classes` / `objects.explicit` | world YAML / `BuilderState` | per-class scattering with clustering, or verbatim placement — matched pairs, RTK clusters, and the World Builder's hand-placed objects (`BuilderState.objects.placed` pins the scatter into the same form) |
| plugin prefix (`gz` default / `ignition`) | `sdf_writer.py` | target Gazebo Fortress instead of Garden/Harmonic |
| `LayerSpec` registry | `worldgen/depth/layers/` | add a seabed layer kind — fn + schema + geometry + channels + `reference`; the menu, the property form, the compositor and the bake pick it up. A kind whose parameters invert what its blend mode means can override the blend on its own result (`LayerOutput.blend`, as a dredged basin with a floor above the bed does) |
| sonar profiles | `config/*_sonar.yaml` | one file per idea: the calibration anchor, the 30 m diagnostic profile (`coverage_pass_sonar`, historical name), the shallow-beach paths, the fault-injection knobs. A profile carries the whole model, so a legacy frozen copy renders with the constants it was frozen with |

## 6. Why not a Gazebo sensor plugin? (decision record)

A `gpu_ray`/lidar-based approach yields ranges, not calibrated
backscatter: no material-dependent intensity, no speckle statistics, no
shadow/highlight contrast control, and it welds the code to one Gazebo
rendering backend. Since the deliverable is *imagery statistically usable
for detector training* and the vehicle is a surface craft whose pose
Gazebo already publishes, rendering acoustics CPU-side against the
ground-truth heightfield is both more faithful and more maintainable. The
`SonarRenderer` ABC preserves the option of a GPU/plugin backend later.

## 7. Why the world frame is the re-based path frame (decision record)

The alternative — an arbitrary Gazebo origin plus a runtime fit, as the
MCS does for the stock world — forces every consumer to compose two
transforms and makes a world's GPS meaning depend on the session. The
world frame is therefore the path's design frame translated so that
world (0, 0) is the **spawn origin** (the path's first waypoint, else the
area centre): the boat — which always spawns at (0, 0) — is guaranteed to
start inside the drawn limits, at the survey start, and `metadata.yaml`
states absolute GPS ground truth at build time through the one
equirectangular pair all modules share. A pure "design frame verbatim"
was tried first and failed in the field: a real MCS path's waypoints can
sit hundreds of metres from its designer origin, which put the spawn (and
the origin-centred water plane) outside the world. The GPS layer makes
the translation invisible to every other module.
