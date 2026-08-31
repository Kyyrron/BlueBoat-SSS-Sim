# Configuration guide

Three YAML layers, all with realistic defaults so an empty file works:

1. **World** (`config/default_world.yaml`) — terrain, materials, objects.
2. **Sonar** (`config/default_sonar.yaml`) — acquisition + physics/noise.
3. **Mission** (`config/default_mission.yaml`) — binds a world, a pattern
   and a sonar profile into one reproducible run bundle; may override
   world sections inline.

Unknown keys in the sonar `model:` section raise immediately (typo
protection); world/mission sections ignore unknown keys.

Two world/mission pairs ship. `default_world.yaml` + `default_mission.yaml`
are the 4.0 m basin; `shallow_water_world.yaml` + `shallow_water_mission.yaml`
are the same basin at 2.5 m for detection work — see §1.1.

A mission's `world_config:` / `sonar_profile:` may be absolute, or relative to
the working directory, or relative to the mission YAML's own directory (tried
in that order). The last is what lets `ros2 run … generate_mission` work
against a config in the installed share directory.

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
| `base_depth` | 4.0 | mean depth (m); the shallow-regime dial — see §1.1 |
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

### walls (optional)

Reflecting boundaries of an enclosed basin. Absent — the default — is open
water, and nothing about the scene changes.

| Key | Default | Effect |
|---|---|---|
| `name` | required | identifies the wall in ghost ground truth (`via: "wall:<name>"`) and in `world.sdf`; must be unique |
| `x0, y0, x1, y1` | required | the wall's plan segment; it extends from the seabed up to `top_z` |
| `top_z` | 0.0 | world z of the top (0 = waterline). A ray passing above it is not reflected |
| `reflectivity` | 0.5 | energy fraction returned per bounce: concrete quay ≈ 0.65, fendered pontoon ≈ 0.3 |
| `thickness` | 0.30 | Gazebo visual only |

Walls are acoustic reflectors and Gazebo visuals. They are **not** stamped
into the height raster and carry no collision, so they produce ghosts but no
direct echo and no shadow of their own (`sonar_model.md` A9), and they do not
change hull dynamics. Nothing is rendered until the sonar profile also sets
`model.wall_multipath_enabled` — the world says which walls exist, the sonar
profile says whether the path is modelled. Unknown keys raise.

### 1.2 The enclosed-basin world

`config/enclosed_basin_world.yaml` is `default_world.yaml` plus three walls
and nothing else changed; `config/enclosed_basin_sonar.yaml` is
`default_sonar.yaml` with `wall_multipath_enabled: true` and nothing else
changed; `config/enclosed_basin_mission.yaml` binds the pair with the
default's seed, pattern and object density. `walls` consumes no RNG draws, so
at the same seed the basin bundle places the *same* objects at the *same*
poses as the default and the two differ only by the boundary and the ghosts
it produces.

Use it for the enclosed-basin regime study. A full survey of the shipped
bundle yields ~200 ghost observations off the three walls, each carrying its
own ground-truth contact naming the object it images and the reflector that
made it: hard negatives for a detector, and a false-positive stimulus the
metrics count separately rather than crediting to the object.

### materials (optional, or `config/materials.yaml`)

Per material: `reflectivity` (0–1 mean backscatter), `texture_amp`,
`texture_cells`, `micro_roughness_m`, `lambert_exp`, `color` (visual only).
Built-ins: seabed `sand/mud/gravel/rocks/seagrass`; object
`rubber/pvc/plastic/glass/metal/concrete/brickclay/rope/generic`.

### 1.1 The shallow-regime world

`config/shallow_water_world.yaml` is `default_world.yaml` with
`base_depth: 2.5` and nothing else changed; `config/shallow_water_mission.yaml`
binds it with the default's seed, pattern, sonar profile and object density.
Generated depth spans 1.7–3.4 m against the default's 3.2–4.9 m.

Use it for detection work. Waterfall shadow length goes as
`h_obj · R / altitude`, so the same object seen at the same slant range casts a
markedly longer shadow from lower down — measured across 47 paired
(object, side) observations of a full lawnmower survey, every one is longer,
median **×1.50** (range ×1.04–×1.76), tracking the `h·R/altitude` prediction.
Contrast, not pixel size, is what limits object detectability at 4 m.

`base_depth` consumes no RNG draws, so at the same seed the two configs place
the *same* objects at the *same* poses with the same sizes and burial. Bundles
generated from them differ only in altitude and can be compared object by
object — which is what the smoke test's `[4]` section does.

Per root `CLAUDE.md` CM-11 this improves synthetic detectability; it does not
make synthetic imagery eligible for the headline detection claim.

## 2. Sonar config

`acquisition:` mirrors the six real ROS parameters — see
`docs/topics.md §2`. `generate_mission` freezes this section into the
bundle's `sonar.yaml`, and `sss_sim_launch.py` passes those six values to the
node explicitly, so **the bundle is what a run acquires at** — editing this
file changes future bundles, not an existing one (NC #10). Each is still
overridable per run exactly as on the real system
(`ros2 launch … gain_index:=-1 range_length_mm:=30000`); the node then warns
that the acquisition in force differs from what the bundle records.
`num_results` up to 1200 (the device's 1/1200-range cross-track resolution) is
fully supported end-to-end (renderer, encoder, raw framing, recorder).

The shipped defaults (15 m, gain 4) are the power-calibration anchor, not the
real node's defaults (20 m, gain −1 = device auto) — see `docs/topics.md §2`
for why they are deliberately not aligned. `config/coverage_pass_sonar.yaml`
is this file with `range_length_mm: 30000` and nothing else changed: the 30 m
coverage-pass setting `project_synthesis.md` §8.5 reserves, against the
default's 15 m revisit setting. Bind it from a mission with
`sonar_profile: config/coverage_pass_sonar.yaml`. At 600 bins the wider swath
gives 50 mm bins against the default's 25 mm and a 42 ms free-run period.

`model:` (no hardware equivalent):

| Group | Keys | Guidance |
|---|---|---|
| Mounting | `sensor_depth_m`, `mount_x_m`, `mount_y_abs_m`, `beam_tilt_deg` (20), `vertical_aperture_deg` (50, spec), `horizontal_aperture_deg` (0.5, spec) | match the physical bracket; tilt+aperture set the usable swath |
| Timing | `max_ping_rate_hz` (20, spec-sheet cap → 50 ms at 15 m) | set 0 to disable and reproduce the field capture's 22 ms free-run |
| Acoustics | `lambert_exponent` (1.0), `absorption_db_per_m` (0.10 @450 kHz), `spreading_exponent` (4.0), `tvg_compensation` (0.0), `beam_sidelobe_floor` (0.004), `specular_strength` (30, sized to clear a +8 dB FBR threshold over dark bottoms), `specular_width_deg` (8.0), `specular_looks` (25, coherent-return CV ≈ 0.2), `pulse_smearing` (true), `alongtrack_beam_lines` (5) | raise `tvg_compensation` → flatter image; the default 0.0 is measured (the profile stream is pre-TVG), which puts the full two-way loss — ~24 dB across the swath at 15 m / 4 m altitude — in the data, and tiles remove its smooth part for display. Sidelobe/specular shape the first-bottom-return line (don't zero them if downstream bottom tracking must lock); `alongtrack_beam_lines: 1` = legacy infinitesimal azimuth beam. Only the product `spreading_exponent · (1 − tvg_compensation)` is determined, and `tvg_compensation` and `lambert_exponent` are further confounded on a flat seabed at one altitude (0.28 dB p-p at the fitted law) — tune one at a time against a known reference, not both together |
| Multipath | `multipath_enabled` (false), `multipath_gain` (0.12) | optional shallow-water second-bottom-echo ghost, displaced +altitude in slant range |
| Wall multipath | `wall_multipath_enabled` (false), `wall_multipath_gain` (1.0), `surface_mirror_enabled` (false), `surface_reflectivity` (0.9), `ghost_beam_lines` (1) | mirror-source ghosts off the world's `walls:` — see `sonar_model.md` §10. Costs ×1.25 per ping-side with one wall in range and ×1.50 with three, ×1.00 when all are out of range; `ghost_beam_lines: 5` costs ×2.17 and buys nothing a bounce has not already smeared. `surface_mirror_enabled` adds the Lloyd-mirror image, which the vertical pattern already attenuates to the sidelobe floor |
| Calibration | `calibration_db_offset` (97.2), `max_span_db` (90) | `pwr_results` is normalised per ping onto `[min_pwr_db, max_pwr_db]`, so the counts carry no level and `calibration_db_offset` is the only level constant — fit it against the reported `max_pwr_db`, never against a count histogram. Measured to ±6 dB (the device's auto-gain regulates the real level). `max_span_db` is the device's own 90 dB clamp on that axis. See `sonar_model.md` §6 for which constants are measured, jointly determined, or undetermined |
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
  gazebo_plugin_prefix: gz         # or ignition (Fortress)
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
