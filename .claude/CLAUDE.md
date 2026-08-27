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
(`$base/lib/blueboat_sss_sim`), all seven `console_scripts` targets, the Python
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

**Layering:** `core/`, `worldgen/`, `sonar/`, `dataset/`, `mission/` import no
ROS and are testable standalone. `ros/` is a thin shell. Keep it that way — the
offline smoke test depends on it.

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
| `range_length_mm` | 15000 | **30000** |
| `msec_per_ping` | 0 (free run) | 0 |
| `gain_index` | 4 | **-1** (device auto) |
| `num_results` | 600 | 600 |
| `pulse_len_percent` | 0.002 | 0.002 |

Anything launched with defaults will therefore image a 15 m swath at fixed gain
in sim and a 30 m swath at auto gain on hardware. Set both explicitly in launch
files when the two must be compared.

Simulation-only: `scene_dir` (**required**), `sonar_config`, `odom_topic`,
`publish_ground_truth`, `seed`.

Node name is `side_scan_sonar` in both the simulator and the real node, so the
resolved topic namespace is identical.

Ground-truth JSON payload per ping cycle:
`{"t_sim": float, "contacts": [{"side","object_id","type","slant_range_m","extent_bins","shadow_bins","visible","ping_number"}]}`

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

### 3.4 `mavros_shim_node` — optional

Subscribes `/blueboat/odom`; publishes `/mavros/global_position/compass_hdg`
(`Float64`), `/mavros/imu/data` (`Imu`), `/mavros/local_position/pose`
(`PoseStamped`). Only needed for tooling written against MAVROS names; the sonar
interface does not need it (vehicle heading is inside every `OmniscanProfile`).

### 3.5 CLI entry points

| Command | Produces |
|---|---|
| `generate_world` | `world.sdf`, `seabed.stl`, `scene.npz`, `scene_manifest.yaml` |
| `generate_mission` | a complete run bundle (see §5) |
| `export_scene_maps` | `gt_reflectivity.png`, `gt_depth.png`, `gt_objects.png`, `gt_extent.yaml` |

### 3.6 Launch files

| File | Starts |
|---|---|
| `full_mission_launch.py` | Gazebo + robot + control stack + mission path service + sonar |
| `sss_sim_launch.py` | sonar (+ optional recorder / shim / path service) |
| `sim_world_launch.py` | Gazebo alone on a generated world |

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

3. **Both sides must be published from the same timer tick.** The processor pairs
   port/starboard within a **50 ms** tolerance and drops unmatched pings; two
   independent per-side timers would desynchronise and starve it.

4. **The first bottom return must survive every change to the acoustic model.**
   Four coupled pieces make downstream bottom tracking lock, and the tracker
   fails silently if any is removed: the beam-pattern sidelobe floor
   (`beam_sidelobe_floor`), the specular near-nadir lobe strong enough to clear a
   +8 dB threshold over dark bottoms (`specular_strength`), the specular
   component rendered and speckled **separately** from the diffuse field with low
   CV (`specular_looks`), and pulse-length range smearing (`pulse_smearing`).
   Rule exists because a fully-speckled, unsmeared, weak nadir return made the
   processor drop 100% of pings until the boat stopped moving.

5. **Speckle statistics are load-bearing, not decoration.** Diffuse field:
   multiplicative `Exp(1)` (fully developed, Rayleigh amplitude) — never emit the
   clean per-bin mean. Coherent specular: `Gamma(L, 1/L)`, low CV. These are what
   make synthetic imagery statistically usable for detector training.

6. **Power calibration is anchored to a real capture** (`base_scale`,
   `calibration_db_offset`): simulated `max_pwr_db` ≈ 63.9 dB and counts spanning
   0–65535 match the decoded field frame. Changing these silently invalidates any
   comparison against real data.

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

`scene.npz` holds the height, reflectivity and material-ID rasters; the manifest
holds grid geometry plus the full object list (id, type, pose, size, burial,
reflectivity). This is the ground truth for every metric the thesis reports —
treat a bundle as write-once. To change something, generate a new bundle.

Dataset output (only when the recorder is explicitly enabled): Ultralytics layout
`images/{train,val}`, `labels/{train,val}`, `dataset.yaml`, with a deterministic
hash-based split, finalized on shutdown.

Rasters are stored `(ny, nx)` with origin at the min corner; `export_scene_maps`
flips vertically for north-up images and writes `gt_extent.yaml` for
georeferencing against mosaics.

---

## 6. Configuration

Three layers, all with working defaults: `config/default_world.yaml`,
`config/default_sonar.yaml`, `config/default_mission.yaml` (+ optional
`config/materials.yaml`). Unknown keys raise immediately in **both** sonar
sections — `acquisition:` and `model:` are each validated against their
dataclass fields in `sonar/config.py`; world/mission sections ignore them.

Sonar config splits into `acquisition:` (the six real device parameters) and
`model:` (simulation-only physics). Knobs whose meaning is not obvious from the
name:

- `max_ping_rate_hz` (20) — spec-sheet cap; **0 disables it**, reproducing the
  22 ms free-run per channel decoded from the field capture. The two sources
  genuinely disagree; the knob exists so either can be reproduced.
- `alongtrack_beam_lines` (5) — parallel ground lines integrated across the 0.5°
  azimuth footprint, Gaussian-weighted with σ(R)=R·θ/2.355. `1` = legacy
  infinitesimal beam.
- `tvg_compensation` (0.90) — fraction of range loss the "device" removes; 1.0
  gives a perfectly flat image.
- `multipath_enabled` (false) / `multipath_gain` — second-bottom-echo ghost
  displaced +altitude in slant range.
- `gain_drift_amp`, `dropped_ping_prob` — **0 by default**; radiometric/link
  degradations belong to the downstream augmentation stage.
- `sample_step_m` — a coarse upper bound only; the renderer refines it.

Mission config: `start` (default `[0,0]`, prepended as a transit waypoint from
the spawn), `start_heading_deg` (the lawnmower entry corner is chosen in front of
it), `pattern` (`lawnmower|spiral|random|waypoints`), `gazebo_plugin_prefix`
(`ignition` emits Fortress `ignition-gazebo-*-system` plugin names, `gz` emits
Garden/Harmonic `gz-sim-*-system`). The shipped `config/default_mission.yaml`
sets **`gz`**; the code-level fallback when the key is absent is `ignition`
(`mission/generate.py`, `worldgen/sdf_writer.py`).

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
```

### Build, run and launch

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash

ros2 run blueboat_sss_sim generate_mission \
    --config $(ros2 pkg prefix blueboat_sss_sim)/share/blueboat_sss_sim/config/default_mission.yaml \
    --out ~/runs/r3 --seed 7 --speed 1.0
ros2 run blueboat_sss_sim export_scene_maps --bundle ~/runs/r3

# Full mission (control stack + sonar); PID is the working controller.
# Run from the workspace root with the workspace sourced.
ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r3
ros2 launch blueboat_sss_sim full_mission_launch.py mission_dir:=$HOME/runs/r3 quiet:=false
ros2 launch blueboat_sss_sim sss_sim_launch.py mission_dir:=$HOME/runs/r3
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

34 checks over world generation → render → noise → encode → decode → tile →
label → export, asserting raw-frame byte parity, checksum corruption detection,
FBR contrast, naive and downstream-emulated tracker lock while moving,
`max_pwr_db` against the real capture, speckle CV, azimuth widening, absence of
empty far-range bins, and 1200-bin operation. It writes a visual waterfall
preview to `/tmp/blueboat_sss_smoke/`.

The run currently aborts in section `[3]` at `test/smoke_test.py:297`, which
imports `blueboat_sss.dataset.labeler` — a module path that no longer exists.
`TODO.md` [P0] holds it.

There is no lint/type-check command configured in this package.

---

## 8. Working on this module

- **Read `docs/` first** — `architecture.md`, `sonar_model.md` (physics and the
  explicit assumption list A1–A7), `topics.md` (raw-frame byte map),
  `integration_guide.md`, `configuration_guide.md`, `developer_guide.md`,
  `roadmap.md`.
- Frames: world is ENU, z=0 at the surface, depths negative. Internal math uses
  ENU yaw; anything hardware- or user-facing uses compass degrees. Conversions
  live only in `core/geometry.py`.
- Sides: `Side.PORT.sign = +1` (+y body), `STARBOARD = -1`; transducer heading =
  vehicle heading ∓ 90°.
- Randomness: every stochastic component takes a `numpy.random.Generator`;
  nothing touches the global RNG. World content derives entirely from
  `world.seed`.
- Adding a litter class: one `CATALOG` entry in `worldgen/objects.py` (the
  labeler discovers classes automatically). Adding a survey pattern: implement it
  in `mission/patterns.py` and register it in `build_pattern`.
- Cost budget: ~1.3 ms per ping (5 azimuth lines, 600 bins) — dual channel at the
  capped rate uses a small fraction of one core.
