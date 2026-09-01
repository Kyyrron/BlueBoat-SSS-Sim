# BlueBoat-SSS-Sim

Synthetic Side Scan Sonar simulation platform for the BlueBoat USV. A ROS 2
(ament_python) package that generates procedural shallow-water worlds and
publishes a **drop-in replacement** for the real dual Cerulean Omniscan 450 SS
interface, so the entire downstream stack runs unmodified against simulated data.

This module is **purely additive** to the BlueBoat workspace. It does not modify
the control stack, robot description, or interfaces packages.

**Naming and layout.** The package is `blueboat_sss_sim` everywhere: `package.xml`
`<name>`, `setup.py` `package_name`, the ament resource marker
`resource/blueboat_sss_sim`, `setup.cfg`'s `script_dir`
(`$base/lib/blueboat_sss_sim`), all nine `console_scripts` targets, the Python
module directory, and the `sl.node(...)` / `sl.include(...)` arguments in all
three launch files. The ROS package root is nested one level below the git root,
at `BlueBoat-SSS-Sim/blueboat_sss_sim/` — that directory is what "the package
source root" means throughout this document.

---

## 1. What this module is for

The thesis it supports investigates *aspect-aware adaptive replanning* for SSS
survey on a surface vehicle. This simulator is the environment in which the
headline policy comparison runs, so its output must be defensible as evidence,
not merely plausible-looking imagery.

Boundary of responsibility: **the simulator's output stops at SSS data**
(profiles, raw frames, ground-truth contacts). Turning that into pictures is the
visualization app's job; turning pictures into training data is the downstream AI
stage's job.

---

## 2. Architecture in one page

**The sonar is not a Gazebo plugin.** Gazebo owns hull dynamics and pose; a ROS 2
Python node owns acoustics, rendering against the same procedurally generated
scene. Consequence: the acoustic scene is static during a run (fine for seabed
litter survey), and the renderer is swappable without touching Gazebo.

```
mission YAML
   │  generate_mission
   ▼
mission bundle ── world.sdf + seabed.stl ──► Gazebo ──► /blueboat/odom
   (one dir,   ── scene.npz + manifest ──┐                    │
    one seed)  ── trajectory.yaml ──┐    │                    ▼
               ── sonar.yaml ──┐    │    └────────────► sss_sim_node
                               │    │                         │
                               │    ▼                         ├─► .../profile ─┐
                               │  sss_path_generation         └─► .../raw ─────┤
                               │  (/path_request)                              │
                               └─► master_control / path_publisher   sss_processor_node
                                   (blueboat_control, unmodified)    (adjacent package)
```

**Single source of truth:** one `SceneModel` emits both `world.sdf` (Gazebo
visuals/physics) and `scene.npz` (acoustic renderer + auto-labeler). They cannot
diverge. Never generate one without the other.

**Layering:** `core/`, `worldgen/`, `sonar/`, `dataset/`, `mission/`,
`analysis/` import no ROS and are testable standalone. `ros/` is a thin
shell. Keep it that way — the offline smoke test depends on it.

Key extension seam: `SonarRenderer` (ABC in `sonar/renderer.py`). Anything
downstream of it — noise, encoding, topics, dataset — is renderer-agnostic.

---

## 3. Interface (the contract — this is where bugs come from)

Node name `side_scan_sonar`, so private topics resolve under `/side_scan_sonar/`.

### 3.1 `sss_sim_node` — the drop-in sonar

| Topic | Type | Dir | QoS | Other side |
|---|---|---|---|---|
| `/side_scan_sonar/port/profile` | `blueboat_interfaces/OmniscanProfile` | pub | BEST_EFFORT, KEEP_LAST, 10 | `sss_processor_node` |
| `/side_scan_sonar/starboard/profile` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/port/raw` | `std_msgs/UInt8MultiArray` | pub | 〃 | `sss_processor_node` (rebuilds `.svlog`) |
| `/side_scan_sonar/starboard/raw` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/ground_truth/contacts` | `std_msgs/String` (JSON) | pub | default 10 | `dataset_recorder_node`; **simulation-only, additive** |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | default 10 | operator / launch one-shot |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | default 10 | Gazebo bridge (param `odom_topic`) |

Pinging is **off at startup**, exactly like hardware. Run-dependent parameters are
re-read on every enable.

Parameters — names and semantics identical to the real `sss_node.py`, **two
defaults deliberately differ** because the simulator is tuned to the 15 m field
capture used for calibration:

| Parameter | Sim default | Real `sss_node.py` default |
|---|---|---|
| `range_start_mm` | 0 | 0 |
| `range_length_mm` | 15000 | **20000** |
| `msec_per_ping` | 0 (free run) | 0 |
| `gain_index` | 4 | **-1** (device auto) |
| `num_results` | 600 | 600 |
| `pulse_len_percent` | 0.002 | 0.002 |

The two differing defaults are **deliberate and settled**: the sim values are
the power-calibration anchor (NC #6 — `calibration_db_offset` and the
acoustic constants are fitted at this range and fixed gain), while the real node's defaults
match neither survey setting the project uses (`project_synthesis.md` §8.5
reserves 30 m coverage / 15 m revisit). Comparability is therefore carried by
**explicitness, not by equal defaults**: the six acquisition parameters are
passed to the node from the mission bundle's frozen `sonar.yaml` by
`sss_sim_launch.py` (and forwarded by `full_mission_launch.py`), each
overridable per run as a launch argument — an empty override means "use the
bundle". The node warns at every ping enable when the acquisition in force
differs from what the bundle records. `config/coverage_pass_sonar.yaml` is the
30 m coverage-pass profile; `config/default_sonar.yaml` is the 15 m revisit
profile and the calibration anchor.

`gain_index: -1` is accepted with the real device's meaning. It is a
**command-only** sentinel — `OmniscanProfile.gain_index` is `uint16`, so the
device resolves auto-gain internally and reports a concrete index. The
simulator has no AGC and resolves `-1` to the calibrated index 4, reporting 4
and `analog_gain` 74.55. The modelled ladder is 0–7, with 4–7 measured from
the field corpus (74.55 / 142.8 / 242.025 / 464.625 — the device's own
auto-gain uses all four); any other index raises
when `AcquisitionParams` is constructed (at launch-argument resolution, not at
the first ping) rather than silently falling back.

Simulation-only: `scene_dir` (**required**), `sonar_config`, `odom_topic`,
`publish_ground_truth`, `seed`.

Node name is `side_scan_sonar` in both the simulator and the real node, so the
resolved topic namespace is identical.

Ground-truth JSON payload per ping cycle:
`{"t_sim": float, "contacts": [{"side","object_id","type","slant_range_m","extent_bins","shadow_bins","visible","ping_number","ghost","via"}]}`

`ghost`/`via` mark a multipath image: the same `object_id` down a folded
path off the named reflector — `via: "wall:<name>"` for a wall image,
`via: "surface"` for the `surface_mirror_enabled` image across z = 0, `""` on
the direct return. Consumers aggregate on `(object_id, via)` — grouping on `object_id`
alone merges an object with its own ghost. Both keys are additive and default
to direct, so a stream from a bundle generated before wall multipath existed
parses unchanged.

### 3.2 `sss_path_generation` — mission trajectory service

| Interface | Type | Dir | Other side |
|---|---|---|---|
| `/path_request` | `blueboat_interfaces/srv/RequestPath` | service server | `path_publisher.py` (`blueboat_control`) |
| `/mission/full_path` | `nav_msgs/Path` | pub, **TRANSIENT_LOCAL latched**, published once at startup | RViz (set display Durability to Transient Local) |

Serves the *same* service name as `blueboat_control`'s `path_generation.py`, and
registers under the *same node name*, `path_generation`. **Exactly one of the two
may run.** Parameters: `trajectory_file` (required), `display_log`,
`display_resolution_m` (0.5).

`path_publisher` requests a time *window* `[0, total_time]` sampled at `dt`
(its own defaults: `total_time` 1000.0, `dt` 0.1), so a mission longer than that
is truncated in RViz and in whatever tracks it. `full_mission_launch` sets
`total_time` from the bundle's stored `duration_s` rather than relying on that
default.

### 3.3 `dataset_recorder_node` — downstream tool, not part of the sim

Subscribes both `.../profile` topics plus `/side_scan_sonar/ground_truth/contacts`;
writes an Ultralytics-layout YOLO dataset. **Not started by any launch file by
default** (`with_recorder:=true` to opt in). It lives here for convenience; the
simulator's deliverable is SSS data.

Parameters: `output_dir` (required), `tile_pings` (512), `overlap_pings` (64),
`box_mode` (`highlight_shadow`), `val_fraction` (0.15), `autosave_period_s` (5),
`run_name`.

### 3.4 `mavros_shim_node` — on by default in the launches

Subscribes `/blueboat/odom`; publishes `/mavros/global_position/compass_hdg`
(`Float64`), `/mavros/imu/data` (`Imu`), `/mavros/local_position/pose`
(`PoseStamped`), and `/mavros/global_position/global` (`NavSatFix`,
BEST_EFFORT sensor-data QoS, throttled to ~5 Hz) — synthesized from the ENU
position about the node parameters `sim_origin_lat` / `sim_origin_lon`
(equirectangular inverse; defaults 43.6961 / 7.3080, matching the GCS `--sim`
origin). The NavSatFix + compass pair is what lets the BlueBoat GCS's
GPS-anchored map anchor against the simulator exactly as against the real
boat, which is why `sss_sim_launch.py` now defaults `with_mavros_shim` to
**true** (pass `with_mavros_shim:=false` to drop the MAVROS names). The sonar
interface itself still does not need it (vehicle heading is inside every
`OmniscanProfile`).

### 3.5 CLI entry points

| Command | Produces |
|---|---|
| `generate_world` | `world.sdf`, `seabed.stl`, `scene.npz`, `scene_manifest.yaml` |
| `generate_mission` | a complete run bundle (see §5) |
| `export_scene_maps` | `gt_reflectivity.png`, `gt_depth.png`, `gt_objects.png`, `gt_extent.yaml` |
| `mission_metrics` | `metrics.json`, `metrics.md` — detection metrics from ground truth |
| `sss_calibration_report` | `calibration.json`, `calibration.md` — a recording and a bundle reduced to the same per-range dB statistic, with the residual against the reduction and site floors |

### 3.6 Launch files

| File | Starts |
|---|---|
| `full_mission_launch.py` | Gazebo on the **bundle's** `world.sdf` + `/clock` and `/ocean_current` bridges + robot spawn + control stack + mission path service + sonar |
| `sss_sim_launch.py` | sonar (+ optional recorder / shim / path service), with the bundle's frozen acquisition passed explicitly |
| `sim_world_launch.py` | Gazebo alone on a generated world |

`blueboat_description`'s `world_launch.py` hard-codes the stock world and
declares no `world` argument, so `full_mission_launch.py` does the Gazebo
bring-up itself: `sl.gz_launch` on the bundle's `world.sdf` (which also
registers the world name `generated_ocean` that the model bridges resolve
against), the `/clock` and `/ocean_current` bridges, and
`blueboat_description`'s `upload_rov_launch.py` for the robot spawn and the
`/blueboat/odom`, `pose_gt`, `joint_states`, `cmd_thruster{1,2}` bridges. No
neighbour file is modified. `use_stock_world:=true` loads the stock description
world instead — the boat drives, but the seabed and objects the sonar renders
against are absent from the Gazebo scene.

---

## 4. NON-NEGOTIABLE CONSTRAINTS

Violating any of these breaks another module, the hardware swap, or the
scientific argument.

1. **Interface fidelity is absolute.** Same topic names, message types, QoS
   (BEST_EFFORT/KEEP_LAST/10), parameter names, and ping-enable semantics as the
   real `sss_node.py`. **Never define a new message type** — reuse
   `blueboat_interfaces`. Additional simulation data goes on new, additive
   topics as JSON (as `ground_truth/contacts` does).

2. **`raw` frames must be byte-valid Ping Protocol, checksum included.**
   Downstream reconstructs `.svlog` files from them for SonarView. Layout:
   `'B''R' | u16 payload_len | u16 msg_id=2198 | u8 src | u8 dst | 52-byte fixed
   payload | u16[num_results] | u16 checksum(sum of all prior bytes)`. Frame
   length = `8 + 52 + 2·num_results + 2` (1262 B at 600 bins). `parse_frame()`
   is the round-trip oracle; keep it working.

3. **Both sides must be published from the same timer tick.** This is now the
   simulator's own rule, not a requirement the downstream processor imposes:
   `sss_processor_node.py` keys rows on `ping_number` (normalised across the
   two device counters), emits one-sided rows rather than dropping them, and
   applies no arrival-time pairing tolerance, so it no longer starves on
   desynchronised sides. Keeping the two sides on one tick is what makes a
   simulated run's rows two-sided by construction and keeps the ping counters
   of the two channels in lockstep; relaxing it is this module's call
   (root `CLAUDE.md` CM-3), and nothing downstream forces it either way.

4. **The first bottom return must survive every change to the acoustic model.**
   Four coupled pieces make downstream bottom tracking lock, and the tracker
   fails silently if any is removed: the beam-pattern sidelobe floor
   (`beam_sidelobe_floor`), the specular near-nadir lobe strong enough to clear a
   +8 dB threshold over dark bottoms (`specular_strength`), the specular
   component rendered and speckled **separately** from the diffuse field with low
   CV (`specular_looks`), and pulse-length range smearing (`pulse_smearing`).
   Rule exists because a fully-speckled, unsmeared, weak nadir return made the
   processor drop 100% of pings until the boat stopped moving. Confirmed in a
   live graph against the real `sss_processor_node` (not the offline emulation):
   it bootstraps within ~10 ping pairs of ping enable and publishes continuously
   for a whole lawnmower pattern.

5. **Speckle statistics are load-bearing, not decoration.** Diffuse field:
   multiplicative `Exp(1)` (fully developed, Rayleigh amplitude) — never emit the
   clean per-bin mean. Coherent specular: `Gamma(L, 1/L)`, low CV. These are what
   make synthetic imagery statistically usable for detector training.

6. **Power calibration is anchored to the field corpus, and moves once.**
   `calibration_db_offset` (97.2 dB) is the **only** level constant: the device
   normalises `pwr_results` per ping, so the counts carry no level and the raw
   count span carries no information about it — fit against the reported
   `max_pwr_db`, never against a count histogram (`[2e]`). Changing it, or the
   fitted range law and noise floor, silently invalidates any comparison against
   real data. `docs/sonar_model.md` §6 records which constants are measured,
   which are jointly determined, and which the corpus leaves undetermined; keep
   it current whenever one moves.

7. **Never raise launch's global log level to suppress noise.** It routes all
   child process output through its own INFO loggers, so it silences every node
   including first-party ones. Filter by message content on the screen handler
   instead.

8. **Never modify the control stack or other packages.** Integration happens by
   serving identical interfaces, not by patching neighbours.

9. **The renderer's ground-sample step must stay coupled to the slant-bin size.**
   A fixed step leaves range bins unpopulated (masked by noise, fatal at 1200
   bins).

10. **Mission bundles are immutable snapshots.** Each carries its own frozen
    `sonar.yaml` and `trajectory.yaml`; editing package config does nothing to an
    existing bundle. Regenerate after any config change.

11. **Project-level:** the thesis's headline detector trains on *real* imagery
    only. Synthetic data from this module is for the policy comparison, the
    sim-to-real transfer-gap experiment, and development — never for the headline
    detection claim.

---

## 5. Data: bundles, formats, what not to overwrite

`generate_mission` produces one self-contained, seed-reproducible **run bundle**:

| File | Kind | Consumer |
|---|---|---|
| `world.sdf`, `seabed.stl` | generated | Gazebo (visuals/physics) |
| `scene.npz`, `scene_manifest.yaml` | generated, **ground truth** | renderer, labeler, `export_scene_maps` |
| `trajectory.yaml` | generated | `sss_path_generation`, `full_mission_launch` |
| `sonar.yaml` | copied snapshot | `sss_sim_node` |
| `mission_snapshot.yaml` | resolved input config | provenance |
| `_world_config.yaml` | intermediate written by `generate_mission` and not removed | none |

`trajectory.yaml` stores `name`, `speed`, `length_m`, `duration_s`, `waypoints`.
`duration_s` is what sizes `path_publisher`'s window — if it is missing (older
bundle), the launch recomputes it from waypoints and speed.

`scene.npz` holds the `height`, `reflectivity` and `material_id` rasters; the
manifest holds `version`, `seed`, `grid`, `material_names`, the resolved world
`config`, the full `objects` list (id, type, pose, size, burial, reflectivity,
material) and the `walls` list. `walls` is additive: it does **not** move the
manifest version, and a bundle generated before walls existed loads as open
water; `SceneModel.load` rejects any other version outright. This is the ground truth for every metric the thesis reports —
treat a bundle as write-once. To change something, generate a new bundle.

Dataset output (only when the recorder is explicitly enabled): Ultralytics layout
`images/{train,val}`, `labels/{train,val}`, `dataset.yaml`, with a deterministic
hash-based split, finalized on shutdown.

Tile pixels: rows are converted to **absolute dB** with the profile's own
`min_pwr_db` / `max_pwr_db` before stacking (the device normalises per ping, so
raw arrays from different pings are not commensurable), the smooth
`log10(range)` trend is removed as a viewer's TVG does, then a per-tile
percentile stretch (1.0 / 99.5) maps to uint8. No log is applied on top — the
samples are already on a dB axis. `dataset.yaml` states this as
`intensity_mapping: db` plus a `meta:` block, at both the positions the
augmentation stage reads, so a consumer never has to guess.

Rasters are stored `(ny, nx)` with origin at the min corner; `export_scene_maps`
flips vertically for north-up images and writes `gt_extent.yaml` for
georeferencing against mosaics.

---

## 6. Configuration

Three layers, all with working defaults: `config/default_world.yaml`,
`config/default_sonar.yaml`, `config/default_mission.yaml` (+ optional
`config/materials.yaml`). Unknown keys raise immediately in **both** sonar
sections — `acquisition:` and `model:` are each validated against their
dataclass fields in `sonar/config.py` — and in the world config's `walls:`
entries, which also reject a duplicate wall name. Every other world/mission
key is read by name and unknown ones are ignored. A model key that was
retired when the encoder moved to the device's per-ping normalisation
(`base_scale`, `gain_index_step_db`) raises by name saying so, so a bundle
frozen under the old encoding fails with an actionable message rather than a
bare "unknown key".

`config/coverage_pass_sonar.yaml` is `default_sonar.yaml` with
`range_length_mm: 30000` and nothing else changed — the coverage-pass setting
`project_synthesis.md` §8.5 reserves, against the default profile's 15 m
revisit setting. Bind it from a mission's `sonar_profile:`. It predates the
wall-multipath knobs and does not spell them out; they resolve to their
dataclass defaults, so the *resolved* `SonarConfig` differs from the default
profile's in `range_length_mm` alone.

A third config set ships for the enclosed-basin regime:
`config/enclosed_basin_world.yaml` is `default_world.yaml` plus three walls,
`config/enclosed_basin_sonar.yaml` is `default_sonar.yaml` with
`wall_multipath_enabled: true`, and `config/enclosed_basin_mission.yaml` binds
the pair with the default's seed, pattern and density. `walls` consumes no RNG
draws, so the same seed places the same objects at the same poses as the
default bundle and the two differ only by the boundary and its ghosts; a full
survey of it yields ~200 ghost observations off the three walls.

A second world/mission pair ships alongside the defaults:
`config/shallow_water_world.yaml` is `default_world.yaml` with
`base_depth: 2.5` and nothing else changed (generated depth 1.7–3.4 m against
the default's 3.2–4.9 m), and `config/shallow_water_mission.yaml` binds it with
the default's seed, pattern, sonar profile and object density. It is the config
for detection work: shadow length goes as `h_obj · R / altitude`, and across 47
paired (object, side) observations of a full survey every shadow is longer at
2.5 m, median ×1.50. Because `base_depth` consumes no RNG draws, the same seed
places the *same* objects at the *same* poses in both, so bundles from the two
differ only in altitude and pair object-by-object.

A mission's `world_config:` / `sonar_profile:` resolves absolute, then relative
to the working directory, then relative to the mission YAML's own directory.
The last is what makes `ros2 run … generate_mission` work against a config in
the installed share directory, where the cwd is not the package source root.

Sonar config splits into `acquisition:` (the six real device parameters) and
`model:` (simulation-only physics). Knobs whose meaning is not obvious from the
name:

- `max_ping_rate_hz` (20) — spec-sheet cap; **0 disables it**, reproducing the
  22 ms free-run per channel decoded from the field capture. The two sources
  genuinely disagree; the knob exists so either can be reproduced.
- `num_results` (600) — **1200 is verified live**, not just offline: against the
  real `sss_processor_node` on Jazzy, 1200-bin frames are 2462 B, carry
  `channel_number` 0/1 at byte 34, run at the same ~18–20 Hz per channel as 600
  bins with zero `ping_number` gaps, and the processor locks bottom within 0.4 s
  and publishes continuously (max stall 58 ms). The one measurable cost is row
  pairing: **98.7% of processed rows are two-sided at 1200 bins against 100% at
  600**, the remainder being CM-6's emit-a-one-sided-row-rather-than-drop path
  firing at the doubled payload.
- `alongtrack_beam_lines` (5) — parallel ground lines integrated across the 0.5°
  azimuth footprint, Gaussian-weighted with σ(R)=R·θ/2.355. `1` = legacy
  infinitesimal beam.
- `tvg_compensation` (0.0) / `spreading_exponent` (4.0) — the range law,
  fitted. **Measured 0**: the profile stream the device reports is pre-TVG, so
  the full two-way loss is in the data — ~40 dB per decade of slant range,
  ~24 dB across the swath at 15 m and 4 m altitude. Only the product
  `spreading_exponent · (1 - tvg_compensation)` is determined; the split is
  fixed by taking spreading at its physical two-way-intensity value. Set
  `tvg_compensation: 1.0` for a flat image. Across-track range, not material,
  sets large-scale brightness by a wide margin.
- `max_span_db` (90.0) — the device's clamp on `max_pwr_db - min_pwr_db`,
  measured (hit exactly on 8.39% of field pings, never exceeded).
- `multipath_enabled` (false) / `multipath_gain` — second-bottom-echo ghost
  displaced +altitude in slant range.
- `wall_multipath_enabled` (false) / `wall_multipath_gain` /
  `surface_mirror_enabled` (false) / `surface_reflectivity` /
  `ghost_beam_lines` (1) — mirror-source ghosts off the reflecting boundaries
  the **world** config declares under `walls:` (the world says which walls
  exist and how reflective each is; these say whether the path is modelled).
  Each wall is a finite vertical segment from the seabed to `top_z`; a ghost
  renders through the same passes as the direct path and lands at its folded
  path length, carrying its own ground-truth contact. Ghost energy joins the
  **diffuse** channel only, so the coherent FBR channel is untouched by
  construction (NC #4) and ghosts carry `Exp(1)` speckle. Sources are culled
  when the wall is beyond the receive window or the fan points away from it.
  Walls are *not* in the height raster, so they cast no direct echo or shadow
  of their own (`docs/sonar_model.md` A9). Cost per ping-side, averaged over
  both sides: x1.00 all culled, x1.25 one wall in range, x1.50 three — all
  three reproduce offline within measurement noise. The multiplier tracks how
  many mirror sources survive the two culls, not how many walls the world
  declares: on a wall-parallel leg the outboard side's sources are all culled,
  and in the shipped basin the fan cull leaves two sources per side even with
  all three wall planes in range. `ghost_beam_lines: 5` buys nothing a bounce
  has not already smeared, and it is the cost lever — but its recorded x2.17
  does **not** reproduce (an offline re-measure gives ~x1.6 at one wall and
  ~x3.4 at three) and the wall count it was taken at is not recorded, so the
  multiplier itself is UNCERTAIN. Details: `sonar/multipath.py`,
  `docs/sonar_model.md` §10.
- `gain_drift_amp`, `dropped_ping_prob` — **0 by default**; radiometric/link
  degradations belong to the downstream augmentation stage.
- `sample_step_m` — a coarse upper bound only; the renderer refines it.

Mission config: `start` (default `[0,0]`, prepended as a transit waypoint from
the spawn), `start_heading_deg` (the lawnmower entry corner is chosen in front of
it), `pattern` (`lawnmower|spiral|random|waypoints`), `gazebo_plugin_prefix`
(`gz` emits Garden/Harmonic `gz-sim-*-system` plugin names, `ignition` emits
Fortress `ignition-gazebo-*-system`). **`gz` throughout**: the shipped
`config/default_mission.yaml` and the code-level fallbacks in
`mission/generate.py`, `worldgen/generate.py` and `worldgen/sdf_writer.py`.
The development machine is ROS 2 Jazzy + Gazebo Harmonic (`gz sim` 8.11.0), so a
generated world loads its six world plugins natively with no deprecation
warnings; `BlueBoat-Control`'s Fortress-named model plugins load on the same
machine only through Harmonic's deprecated-name shim.

---

## 7. Commands

All package-relative commands below run from the package source root,
`BlueBoat-SSS-Sim/blueboat_sss_sim/`.

### Offline tools (no ROS required)

```bash
python3 -m blueboat_sss_sim.mission.generate \
    --config config/default_mission.yaml --out ~/runs/r3 --seed 7 --speed 1.0
python3 -m blueboat_sss_sim.worldgen.export_maps --bundle ~/runs/r3
python3 -m blueboat_sss_sim.worldgen.generate --config config/default_world.yaml --out ~/runs/w1
python3 -m blueboat_sss_sim.analysis.cli --bundle ~/runs/r3 --out ~/metrics/r3
python3 -m blueboat_sss_sim.analysis.calibration_cli \
    --svlog ~/ros2_ws/data/SSS_data/diffDepthCompensation.svlog \
    --bundle ~/runs/r3 --out ~/calib/r3
```

`mission_metrics` reads a bundle and writes `metrics.json` + `metrics.md`
into `--out`; the bundle and any recording it reads are never written to
(NC #10). Its default `--source replay` walks the bundle's own
`trajectory.yaml`, so its numbers are reproducible from the seed:
`content_digest` covers everything but provenance and does not move when
the bundle is regenerated. `--source svlog` measures the path a run
actually tracked (the recording must carry the mavlink `LOCAL_POSITION_NED`
track — `with_mavros_shim:=true` in simulation), and `--source jsonl` reads
a dump of the published `ground_truth/contacts` stream, which carries no
pose and so reports aspect as unavailable. Detection is a named criterion
(`geometric` / `resolved` / `shadowed`), stated in both output files.

### Build, run and launch

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash

ros2 run blueboat_sss_sim generate_mission \
    --config $(ros2 pkg prefix blueboat_sss_sim)/share/blueboat_sss_sim/config/default_mission.yaml \
    --out ~/runs/r3 --seed 7 --speed 1.0
ros2 run blueboat_sss_sim export_scene_maps --bundle ~/runs/r3

# Full mission (control stack + sonar); PID is the working controller.
# Run from the workspace root with `source env.sh` (venv + workspace).
# The venv is not optional here: blueboat_control's master_control.py imports
# ur_mpc -> acados_template + casadi, and simulation_interface.py imports the
# blueboat_control package whose __init__ imports casadi and urdf_parser_py --
# all at module scope regardless of controller_type, so without it those two
# nodes die at import and the boat never moves while the sonar still pings.
ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r3
ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r3 quiet:=false
ros2 launch blueboat_sss_sim sss_sim_launch.py mission_dir:=$HOME/runs/r3
# Per-run acquisition override (empty = the bundle's frozen value)
ros2 launch blueboat_sss_sim sss_sim_launch.py mission_dir:=$HOME/runs/r3 \
    range_length_mm:=30000 gain_index:=-1
ros2 launch blueboat_sss_sim sim_world_launch.py mission_dir:=$HOME/runs/r3

# Manual ping enable (same command as on the real system)
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

`full_mission_launch.py` includes `sss_sim_launch.py`, so the two are exercised
together. Everything in this block needs Linux + ROS 2 and has not been run on
the Windows checkout.

### Offline smoke test

```bash
python3 -m test.smoke_test        # from the package source root
```

84 checks over world generation → render → noise → encode → decode → tile →
label → export, asserting raw-frame byte parity, checksum corruption detection,
per-side `channel_number` at byte 34, FBR contrast, naive and
downstream-emulated tracker lock while moving, `max_pwr_db` against the real
corpus (`max_pwr_db` 67.85 ± 0.5 dB at the default profile — the quantity's
own spread across speckle seeds is 0.064 dB sd, so a wider band could not catch
a moved anchor), the device's per-ping normalisation invariants, speckle CV, azimuth
widening, absence of empty far-range bins, and 1200-bin operation. Section
`[2e]` covers calibration identifiability — which constants a capture can
determine and from which statistic: the level riding in `max_pwr_db` and never
in the counts, `tvg_compensation` and
`lambert_exponent` as interchangeable at one altitude, and the per-range
reduction's repeatability floor. These are properties of the parameterisation,
so they hold independent of the tuning. Section `[2f]` runs the same checks
against the **real corpus**: the device's normalisation invariants on field
pings, `ANALOG_GAIN_TABLE` against the `analog_gain` the device reports, the
water-column-to-bottom-return gap the noise floor was fitted to, and the
range law's fall against the corpus's 40–52 dB/decade. It is gated on
`$BLUEBOAT_SSS_CORPUS` (default `~/ros2_ws/data/SSS_data`) and **skips when
the recordings are absent**, so a fresh clone still reaches
`ALL CHECKS PASSED`. Section `[4]` covers the shallow-regime config: depth band,
per-ping FBR lock at ~2.2 m altitude via the same downstream detector
emulation, and paired shadow extents against the 4 m bundle. Section `[4b]`
covers the device-facing acquisition constants: `gain_index: -1` resolving and
framing byte-identically to the calibrated gain, the 0–7 ladder against the
measured `analog_gain` values, a gain step moving the reported dB and not the
counts, rejection of
out-of-ladder indices, `sss_sim_launch.py`'s own acquisition resolver binding
the bundle (and applying per-run overrides), and the coverage-pass profile
differing from the default in range alone while still locking FBR at 30 m.
Section `[4c]` covers the ground-truth metrics: the manifest partition
(every placed object is detected, ensonified-below-criterion or never
ensonified, and the three sum to the manifest), empty bins reading as
unmeasured rather than as zero, the criterion ladder nesting, the content
digest holding across a bundle regenerated from its seed, and the `.svlog`
path — clock-skew recovery, pose-track fidelity, and one corrupted packet
costing one packet.
Section `[4d]` covers wall multipath: mirror sources and the surface image
against a hand-computed reflection, the range cull and the finite-wall
crossing test, a ghost arriving at its hand-computed slant range, the feature
disabled rendering bit-identically to a wall-free world, the specular channel
never receiving ghost energy, FBR lock and speckle/bin-filling with ghosts in
the image, the shipped basin bundle's walls reaching `world.sdf`, a wall-free
manifest still loading at the same version, per-ghost ground truth, the
additive JSON schema, ghost boxes not merging with their object, and the
metrics partition holding with ghosts excluded from every rate. It writes a visual
waterfall preview to `/tmp/blueboat_sss_smoke/`. A full run takes ~36 s and ends
with `ALL CHECKS PASSED`. `test/` carries an `__init__.py` so the local package wins
over the stdlib `test` module — without it the command above resolves to the
wrong `test` and fails with `No module named test.smoke_test`.

There is no lint or type-check command configured in this package, so the smoke
test is the only automated gate. `.claude/settings.json` wires it as one.

### Session hooks

`.claude/settings.json` holds exactly two hooks, both implemented in
`.claude/tools/smoke_gate.py` (stdlib only, no ROS, no third-party imports):

| Hook | Fires on | Effect |
|---|---|---|
| `PostToolUse`, matcher `Edit\|Write\|MultiEdit` | an edit under `config/`, `sonar/` or `mission/` | Surfaces a "regenerate the bundle" reminder — bundles freeze their own copies (NC #10). A reminder, never a block. |
| `Stop` | end of a turn that changed a file the suite can observe | Runs the smoke test from the package source root. **Exit 2 blocks the stop** until it passes. |

Smoke-test scope is `*.py` under `blueboat_sss_sim/` or `test/`, plus
`*.yaml` under `config/` (the suite calls `generate_mission` against
`config/default_mission.yaml`). `docs/`, any `*.md`, `launch/`,
`msg_reference/`, `resource/`, `package.xml` and `setup.*` are out of scope —
the offline suite cannot observe them, and a gate that fires on documentation
edits gets disabled.

The Stop hook triggers on a per-session flag set by the PostToolUse hook **and**
on in-scope file mtimes, so a change made by a shell command rather than an edit
tool is still gated. It skips when `stop_hook_active` is set, so a blocked stop
cannot loop. No hook runs ROS, `colcon` or Gazebo: the gate is offline by
construction (§2 layering), which is what keeps it cheap enough to run every
turn.

Deliberately not automated: a release/packaging subagent (packaging only
modified files was a chat-delivery constraint, not a repo workflow), and a
docs-sync agent (doc updates have so far been small and colocated with the code
change).

---

## 8. Working on this module

- **Read `docs/` first** — `architecture.md`, `sonar_model.md` (physics and the
  explicit assumption list A1–A10, given in the order A1–A8, A10, A9),
  `topics.md` (raw-frame byte map), `integration_guide.md`,
  `configuration_guide.md`, `developer_guide.md`, `roadmap.md`,
  `REALISM_UPDATE_NOTES.md`.
- Frames: world is ENU, z=0 at the surface, depths negative. Internal math uses
  ENU yaw; anything hardware- or user-facing uses compass degrees. Conversions
  live only in `core/geometry.py`.
- Sides: `Side.PORT.sign = +1` (+y body), `STARBOARD = -1`; transducer heading =
  vehicle heading ∓ 90°. `Side.channel` is the device channel number — 0 port,
  1 starboard — which `PingEncoder` derives from the side and writes to byte 34
  of the raw frame and to `OmniscanProfile.channel_number`. It is the
  authoritative side identity downstream: `sss_processor_node` derives the
  `.svlog` `src_device_id` from it and the GCS reader groups port/starboard on
  it, neither trusting the topic nor the `src` tag.
- Randomness: every stochastic component takes a `numpy.random.Generator`;
  nothing touches the global RNG. World content derives entirely from
  `world.seed`.
- Adding a litter class: one `CATALOG` entry in `worldgen/objects.py` (the
  labeler discovers classes automatically). Adding a survey pattern: implement it
  in `mission/patterns.py` and register it in `build_pattern`.
- Cost budget, measured live on the Jazzy machine at the 20 Hz cap: the renderer
  alone is **2.04 ms per ping-side** at 600 bins and **2.38 ms at 1200** (5 azimuth
  lines); the whole `sss_sim_node` process, which also encodes, frames, publishes
  and emits ground truth, sits at **32% of one core** — 32.0% at 600 bins, 32.5% at
  1200. Doubling the bin count is nearly free: the node is dominated by fixed
  per-ping overhead, not by bin count. Wall multipath multiplies the renderer
  figure by **1.00** when every wall is culled, **1.25** with one wall in range
  and **1.50** with three (`ghost_beam_lines: 1`); at the process level it
  disappears into that same per-ping overhead — the node's CPU share is
  unchanged within measurement noise by enabling it.
