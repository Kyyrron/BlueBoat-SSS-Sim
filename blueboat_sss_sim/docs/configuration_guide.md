# Configuration guide

Three YAML layers, all with realistic defaults so an empty file works:

1. **World** (`config/default_world.yaml`) — terrain, materials, objects.
2. **Sonar** (`config/default_sonar.yaml`) — acquisition + physics/noise.
3. **Mission** (`config/default_mission.yaml`) — binds a world, a pattern
   and a sonar profile into one reproducible run bundle; may override
   world sections inline.

Unknown keys in the sonar `model:` section raise immediately (typo
protection); world/mission sections ignore unknown keys.

## 1. World config

```yaml
world:
  seed: 42                # every stochastic choice derives from this
  size: [80.0, 60.0]      # m
  origin: [-40.0, -30.0]  # world xy of min corner
  resolution: 0.10        # raster cell, m — keep ≤ half a range bin
```

### terrain

| Key | Default | Effect |
|---|---|---|
| `base_depth` | 4.0 | mean depth (m); the shallow-regime dial |
| `slope.direction_deg`, `slope.grade` | 15, 0.015 | planar tilt |
| `dunes.{enabled, wavelength, amplitude, direction_deg, irregularity}` | on, 6, 0.12, 30, 0.5 | sand-wave field; irregularity 0 = pure sine |
| `roughness.{amplitude, octaves, cells}` | 0.05, 5, 12 | fBm micro-relief |
| `materials.layout` | `patches` | `uniform` (one material) or fBm-rank patches |
| `materials.composition` | sand-dominant mix | area fractions, auto-normalized |
| `materials.patch_cells` | 6 | patch spatial scale |

### objects

| Key | Default | Effect |
|---|---|---|
| `density_per_hectare` | 60 | expected object count / ha |
| `composition` | catalog defaults | relative weights per type (13 types: `tire_car`, `tire_bicycle`, `pipe_pvc`, `bottle_glass`, `bottle_plastic`, `can`, `tent_weight`, `rope`, `cylinder_metal`, `block_concrete`, `brick`, `chain`, `anchor`, `debris`) |
| `margin_m`, `min_separation_m` | 3.0, 1.0 | placement constraints (dart throwing) |
| `size_scale`, `burial_scale` | 1.0 | global multipliers on per-type priors |
| `reflectivity_jitter` | 0.15 | per-instance acoustic variation |
| `overrides.<type>` | — | per-type priors (`length_range`, `burial_range`, …) |

### materials (optional, or `config/materials.yaml`)

Per material: `reflectivity` (0–1 mean backscatter), `texture_amp`,
`texture_cells`, `micro_roughness_m`, `lambert_exp`, `color` (visual only).
Built-ins: seabed `sand/mud/gravel/rocks/seagrass`; object
`rubber/pvc/plastic/glass/metal/concrete/brickclay/rope/generic`.

## 2. Sonar config

`acquisition:` mirrors the six real ROS parameters — see
`docs/topics.md §2`; changing them here changes the launch-time defaults,
and they can still be overridden per run via ROS parameters, exactly like
on the real system. `num_results` up to 1200 (the device's 1/1200-range
cross-track resolution) is fully supported end-to-end (renderer, encoder,
raw framing, recorder).

`model:` (no hardware equivalent):

| Group | Keys | Guidance |
|---|---|---|
| Mounting | `sensor_depth_m`, `mount_x_m`, `mount_y_abs_m`, `beam_tilt_deg` (20), `vertical_aperture_deg` (50, spec), `horizontal_aperture_deg` (0.5, spec) | match the physical bracket; tilt+aperture set the usable swath |
| Timing | `max_ping_rate_hz` (20, spec-sheet cap → 50 ms at 15 m) | set 0 to disable and reproduce the field capture's 22 ms free-run |
| Acoustics | `lambert_exponent` (1.7), `absorption_db_per_m` (0.10 @450 kHz), `spreading_exponent` (2.0), `tvg_compensation` (0.90), `beam_sidelobe_floor` (0.004), `specular_strength` (30, sized to clear a +8 dB FBR threshold over dark bottoms), `specular_width_deg` (8.0), `specular_looks` (25, coherent-return CV ≈ 0.2), `pulse_smearing` (true), `alongtrack_beam_lines` (5) | raise `tvg_compensation` → flatter image; sidelobe/specular shape the first-bottom-return line (don't zero them if downstream bottom tracking must lock); `alongtrack_beam_lines: 1` = legacy infinitesimal azimuth beam |
| Multipath | `multipath_enabled` (false), `multipath_gain` (0.12) | optional shallow-water second-bottom-echo ghost |
| Calibration | `calibration_db_offset` (16), `base_scale` (110000), `gain_index_step_db` (3) | calibrated against the field capture (counts 0–62k, max_pwr_db 63.9) |
| Noise | `speckle` (true), `speckle_looks` (1 = fully-developed Exp(1); >1 = smoother multi-look Gamma), `noise_floor` (0.002), `watercolumn_noise` (0.002), `gain_drift_amp` (0 — deferred to the augmentation stage), `dropped_ping_prob` (0 — same) | zero everything for "clean physics" ablation images; keep `watercolumn_noise` well below the FBR peak |
| Sampling | `sample_step_m` (0.05, coarse upper bound only) | the renderer refines the step to ~half the slant bin automatically, so 600- and 1200-bin runs are both fully populated |

## 3. Mission config

`generate_mission --speed 1.5` overrides the pattern speed from the CLI;
the resulting `trajectory.yaml` stores `duration_s`/`length_m`, which
`full_mission_launch` reads to size `path_publisher`'s `total_time`
window to the whole mission automatically. `export_scene_maps --bundle
<dir>` renders georeferenced ground-truth maps (reflectivity, depth,
object overlay) for mosaic validation.

```yaml
mission:
  seed: 7
  randomize: false        # true → draw seed/density/pattern per bundle
  world_config: config/default_world.yaml
  sonar_profile: config/default_sonar.yaml
  gazebo_plugin_prefix: ignition   # or gz
  start: [0.0, 0.0]                # robot spawn, prepended as a transit
                                   # waypoint (null to disable)
  start_heading_deg: 0.0           # spawn heading; the lawnmower entry
                                   # corner is chosen IN FRONT of it
  pattern: lawnmower               # lawnmower | spiral | random | waypoints
  lawnmower: {bbox: [-30,-20,30,20], spacing: 8.0, speed: 1.0, heading_deg: 0}
  spiral:    {center: [0,0], r_max: 25.0, spacing: 8.0, speed: 1.0}
  random:    {bbox: [-30,-20,30,20], n_legs: 14, speed: 1.0}
  # waypoints: {points: [[...]], speed: 1.0}

world_overrides:          # section-wise merge onto world_config
  objects: {density_per_hectare: 90}
```

Choose `spacing` < 2 × usable swath (≈ `range_length_mm`·cos-projection −
nadir gap) for full coverage; the 8 m default gives generous overlap at
the 15 m / 4 m-depth defaults.

## 4. Dataset recorder parameters

The recorder is a downstream AI-stage tool and is **not** part of the
simulator launch (off by default; the sim's output stops at SSS data).
Run it separately when building datasets.

`output_dir` (required), `tile_pings` (512), `overlap_pings` (64),
`box_mode` `highlight|highlight_shadow` (default includes the shadow —
usually the stronger detector cue), `val_fraction` (0.15, deterministic
hash split), `autosave_period_s` (5), `run_name` (tile prefix).
