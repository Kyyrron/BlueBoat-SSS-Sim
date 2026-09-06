# BlueBoat-SSS-Sim

Synthetic Side Scan Sonar platform for the BlueBoat USV. A ROS 2
(ament_python) package with two deliverables, cleanly separated:

1. the **World Builder** — a PySide6 wizard producing **GPS-anchored Gazebo
   worlds** around the MCS Pattern Designer's GPS-anchored survey paths
   (limits + reflective walls drawn on a satellite map, authored bathymetry,
   a 27-class litter catalog), saved as immutable folders under
   `~/worlds/<path_name>/<world_name>/`;
2. the **simulated sonar** — a **drop-in replacement** for the real dual
   Cerulean Omniscan 450 SS interface, so the entire downstream stack runs
   unmodified against simulated data.

This module is **purely additive** to the BlueBoat workspace. It does not
modify the control stack, robot description, interfaces packages, MCS or
GCS. Missions belong to MCS/Control: this package serves worlds and sonar
data and **no longer serves `/path_request`** (Control's own
`path_generation` is the only server; its `from_yaml` branch consumes MCS
deployed trajectories directly).

**Naming and layout.** The package is `blueboat_sss_sim` everywhere:
`package.xml` `<name>`, `setup.py` `package_name`, the ament resource
marker, `setup.cfg`'s `script_dir`, all eight `console_scripts` targets,
the Python module directory, and the `sl.node(...)` / `sl.include(...)`
arguments in all three launch files. The ROS package root is nested one
level below the git root, at `BlueBoat-SSS-Sim/blueboat_sss_sim/` — that
directory is what "the package source root" means throughout this document.

---

## 1. What this module is for

The thesis it supports investigates *aspect-aware adaptive replanning* for
SSS survey on a surface vehicle. This simulator is the environment in which
the headline policy comparison runs, so its output must be defensible as
evidence, not merely plausible-looking imagery.

Boundary of responsibility: **worlds in, SSS data out.** The World Builder
turns an MCS anchored path plus operator intent into a georeferenced Gazebo
world with ground truth; the sonar node turns that world plus vehicle odom
into profiles, raw frames and ground-truth contacts. Turning that into
pictures is the visualization app's job; missions are MCS/Control's job;
training data is the downstream AI stage's job (the in-package YOLO
recorder was deleted with the 2026-09 rework).

---

## 2. Architecture in one page

**The sonar is not a Gazebo plugin.** Gazebo owns hull dynamics and pose; a
ROS 2 Python node owns acoustics, rendering against the same generated
scene. Consequence: the acoustic scene is static during a run (fine for
seabed litter survey), and the renderer is swappable without touching
Gazebo.

```
MCS anchored path (~/.config/blueboat_mcs/trajectories/<name>.yaml, geo_anchor)
   │  world_builder GUI  (or generate_world on a world YAML, headless)
   ▼
world folder ~/worlds/<path>/<world>/          [immutable; _i-deduped]
   ├─ world.sdf + seabed.stl ──► Gazebo (spherical_coordinates, wall
   │                             collisions, invisible limit borders)
   ├─ scene.npz + scene_manifest.yaml (geo block) ──► sss_sim_node ──► profiles/raw/contacts
   ├─ depth.npz (terrain-only height)   ├─► mavros_shim_node ──► /mavros/* (GPS about the anchor)
   ├─ preview_objects.png               ├─► sea_state_node ──► /current → gz /ocean_current, /world/<w>/wrench
   ├─ metadata.yaml (GPS ground truth)  ▼                       → gz ApplyLinkWrench, /sim/sea_state (JSON) → MCS
   └─ builder_state.yaml (Edit mode)   sss_processor_node (adjacent package, unmodified)
```

**Sea state (2026-09-03):** `sea/` (ROS-free — JONSWAP wave field with
finite-depth dispersion, Fossen Gauss-Markov current, hydrostatic
Froude-Krylov force on the BlueBoat catamaran hull minus Gazebo's own box
term, presets + timelines) drives `ros/sea_state_node.py`, which
publishes the water velocity the boat's `Hydrodynamics` plugin drags
against and a one-shot wave wrench per physics step through gz-sim's
`ApplyLinkWrench` (injected at run time when a world lacks it, so the
stock `Sim_launch.py` world works too). The boat and the sonar are
unchanged: roll/heave/yaw already coupled, pitch now does (`renderer.
_ping_geometry`, `dz·tan(pitch)` along-track). **2026-09-05:** the
node now *predicts* the 20 Hz odometry to the tick with its twist and
adds hull damping (`sea/motion.py`) — the restoring spring evaluated on
a 50 ms-stale pose rang the 0.6 s heave mode at 1.6 Hz ("too
powerful, too regular, too frequent"); the field is short-crested
(12 bearings, s 3, seeded random detune), the hull also pushes
surge/sway/yaw (first-order horizontal Froude–Krylov), wake groups are
half as frequent and spectrally broad, and `simulate_3dof` closes the
loop offline as the gate (`docs/sea_state_model.md` §5.5–5.8, §9). Presets
`config/sea_states.yaml`; physics and DOIs `docs/sea_state_model.md`;
MCS chooses the situation after the world (its own code, CM-3).

**Single source of truth:** one `SceneModel` emits both `world.sdf` and
`scene.npz`. They cannot diverge. Never generate one without the other.

**GPS anchoring:** a world's local ENU frame is its source path's design
frame **translated so that world (0, 0) is the spawn origin** — the
path's first waypoint when it lies inside the drawn limits, else the
rectangle centre (`BuilderState.spawn_origin`). The boat always spawns at
world (0, 0) (`upload_rov_launch.py` / MCS `Sim_launch.py` hard-code it),
so this re-basing guarantees it starts inside the area, at the survey
start; the world's `geo_anchor` is that point's GPS (identical to the
path's own anchor when the path starts at its design origin). A real MCS
path whose waypoints sit far from its designer origin therefore still
yields a world the boat spawns inside — the GPS layer absorbs the shift,
since every conversion goes through lat/lon. `core/geo.py` carries the
project-wide equirectangular pair (byte-for-byte the MCS/GCS convention);
the manifest carries an additive `geo:` block, `world.sdf` a
`<spherical_coordinates>` element, and `metadata.yaml` every
object/wall/corner **and the spawn** in GPS (plus the design offset). The
folder also stores `source_path_world.yaml` — the source path re-expressed
in world coordinates, first waypoint at (0, 0), directly usable as the dev
launch's `trajectory_file`. `sss_sim_launch.py` feeds the MAVROS shim from
the world's anchor automatically.

**Layering:** `core/`, `worldgen/`, `sonar/`, `analysis/` import no ROS and
no Qt and are testable standalone. `ros/` is a thin ROS shell; `builder/`
is the only Qt code (and imports no ROS; `builder/state.py` and
`builder/paths.py` are Qt-free so the smoke test drives the whole build
pipeline headless). Keep it that way — the offline smoke test depends on it.

Key extension seams: `SonarRenderer` (ABC in `sonar/renderer.py`) — anything
downstream of it is renderer-agnostic; `DEPTH_PRESETS`
(`worldgen/depth/presets.py`) and `CATALOG` (`worldgen/objects.py`) — one
registry entry adds a bathymetry pattern or an object class to the GUI with
zero GUI edits.

---

## 3. Interface (the contract — this is where bugs come from)

Node name `side_scan_sonar`, so private topics resolve under
`/side_scan_sonar/`.

### 3.1 `sss_sim_node` — the drop-in sonar (unchanged by the rework)

| Topic | Type | Dir | QoS | Other side |
|---|---|---|---|---|
| `/side_scan_sonar/port/profile` | `blueboat_interfaces/OmniscanProfile` | pub | BEST_EFFORT, KEEP_LAST, 10 | `sss_processor_node` |
| `/side_scan_sonar/starboard/profile` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/port/raw` | `std_msgs/UInt8MultiArray` | pub | 〃 | `sss_processor_node` (rebuilds `.svlog`) |
| `/side_scan_sonar/starboard/raw` | 〃 | pub | 〃 | 〃 |
| `/side_scan_sonar/ground_truth/contacts` | `std_msgs/String` (JSON) | pub | default 10 | offline analysis; **simulation-only, additive** |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | default 10 | operator / launch one-shot |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | default 10 | Gazebo bridge (param `odom_topic`) |

Pinging is **off at startup**, exactly like hardware. Run-dependent
parameters are re-read on every enable.

Parameters — names and semantics identical to the real `sss_node.py`, **two
defaults deliberately differ** because the simulator is tuned to the 15 m
field capture used for calibration:

| Parameter | Sim default | Real `sss_node.py` default |
|---|---|---|
| `range_start_mm` | 0 | 0 |
| `range_length_mm` | 15000 | **20000** |
| `msec_per_ping` | 0 (free run) | 0 |
| `gain_index` | 4 | **-1** (device auto) |
| `num_results` | 600 | 600 |
| `pulse_len_percent` | 0.002 | 0.002 |

The two differing defaults are **deliberate and settled**: the sim values
are the power-calibration anchor (NC #6). Comparability is carried by
**explicitness, not by equal defaults**: `sss_sim_launch.py` passes all six
acquisition parameters to the node from the resolved sonar profile
(explicit `sonar_config:=` > a legacy bundle's frozen `sonar.yaml` >
`config/default_sonar.yaml`), each overridable per run as a launch
argument. `config/coverage_pass_sonar.yaml` is the 30 m coverage-pass
profile; `config/default_sonar.yaml` is the 15 m revisit profile and the
calibration anchor.

`gain_index: -1` is accepted with the real device's meaning (command-only
sentinel; the simulator resolves it to the calibrated index 4 and reports
4). The modelled ladder is **0–7**, 4–7 measured from the corpus; any other
index raises when `AcquisitionParams` is constructed.

Simulation-only params: `scene_dir` (**required** — a world folder or
legacy bundle), `sonar_config`, `odom_topic`, `publish_ground_truth`,
`seed`.

Ground-truth JSON payload per ping cycle:
`{"t_sim": float, "contacts": [{"side","object_id","type","slant_range_m","extent_bins","shadow_bins","visible","ping_number","ghost","via"}]}`
— `ghost`/`via` mark a multipath image (`via: "wall:<name>"` / `"surface"`,
`""` direct); consumers aggregate on `(object_id, via)`; both keys default
to direct, so pre-wall streams parse unchanged.

### 3.2 `mavros_shim_node` — on by default in the launches

Subscribes `/blueboat/odom`; publishes
`/mavros/global_position/compass_hdg` (`Float64`), `/mavros/imu/data`
(`Imu`), `/mavros/local_position/pose` (`PoseStamped`), and
`/mavros/global_position/global` (`NavSatFix`, BEST_EFFORT sensor-data QoS,
~5 Hz) — synthesized from the ENU position about `sim_origin_lat` /
`sim_origin_lon`. **`sss_sim_launch.py` fills those from the world's own
geo anchor** when the args are left empty; the legacy 43.6961 / 7.3080
default applies only to unanchored worlds, with a warning. This is what
lets the BlueBoat GCS's GPS-anchored map anchor against the simulator at
the world's true location. (History: commit f489a21 shipped this node
without its odom subscription — four advertised topics, zero messages; the
subscription is restored and is the node's entire drive train.)

### 3.3 World Builder — `world_builder` (GUI, PySide6)

Startup dialog: **Create Gazebo World** / **Edit World** (Edit lists
`~/worlds/*/*/` and reloads a world's `builder_state.yaml`; Screen 1 is
skipped and everything arrives pre-filled). Screens:

1. **GPS path** — the MCS launch dialog's list filtered to GPS-anchored
   entries (`builder/paths.py`: glob `~/.config/blueboat_mcs/trajectories/
   *.yaml` minus `*.meta.yaml`, keep `geo_anchor` present — the
   `read_geo_anchor` predicate, reimplemented locally per CM-3).
2. **World limits & walls** — satellite-tile map (Esri, the MCS/GCS
   source, cached under `~/.cache/blueboat_sss_sim/tiles/`); drag one
   axis-aligned rectangle = the limits (grid extent + four invisible
   collision-only border boxes, **never acoustic**); click wall segments =
   `Wall` entries with a **kind**: `quay` (face from the seabed, blocks
   sonar and boat) or `pontoon` (floating, `draft_m`, the sonar looks
   under it — the structure a boat can cross); each with reflectivity
   (per bounce, 0.65 concrete), `roughness_m` (Rayleigh coherence at
   450 kHz) and a Gazebo box **with collision** (`docs/sonar_model.md`
   §11, re-anchored 2026-09-03 on the corpus quay: bright face, dark zone
   behind, faint far ghosts). Every wall-dialog row carries hover help.
   Endpoints within 0.75 m of another wall's end **snap** while drawing
   or dragging and are **merged** at build time (union-find,
   `core.geometry.merge_endpoints`); selecting a wall shows a hollow
   midpoint handle — dragging it **bends** the wall (a quadratic Bézier
   through that point, stored as `ctrl` in the builder state only, and
   flattened into `<name>_s01…` chords for the engine by
   `core.geometry.wall_segments`); "Straighten wall" removes the bend.
3. **Depth & seabed** — three sources: a preset (`harbour_basin`,
   `river_channel`, `beach_profile` (Dean x^(2/3), optional berm),
   `canal`, `flat`, `sloped_plane`, `dredged_channel`, `marina`,
   `rocky_outcrop`, `sandbar_ripples`; parameter forms generated from
   `DEPTH_PRESETS`; **depth ≤ 0 is dry land** — the dry-margin presets
   accept negative minima since 2026-09-05), a file (CSV lat,lon,depth;
   geographic-CRS GeoTIFF via Pillow — projected CRS rejected with a
   message; linear interpolation inside the data, nearest-value flood
   fill outside that **never crosses a wall**), or a **sonar recording**
   (`.svlog`, 2026-09-05: the recording's first-bottom-return soundings
   at its GPS fixes — `analysis/bathymetry.py`, the processor's detector
   re-expressed, both sides fused robustly, a frozen autopilot clock
   paired by file order — fitted with a smoothing thin-plate spline,
   the edge slope continued and damped beyond the track over
   `trend_length_m`, a residual-scaled undulation between the lines;
   `worldgen/depth/svlog.py` + `smooth.py`; the track and soundings are
   drawn on the map, a quality line reports pings / bottom-return
   fraction / altitude / cells / crossovers / clock regime **and
   coverage** (2026-09-06: samples inside the limits, share of the world
   within `trend_length_m` of a sounding, with a warning below 35 % —
   `worldgen/depth/coverage.py`; a source that lands nowhere inside the
   limits is refused with both places, the distance and the compass
   point, and a near miss says to redraw the rectangle rather than to
   change the file), and an
   optional backscatter *hint* layer can seed material patches on
   request — how to sail for it: `docs/svlog_depth_survey.md`); then
   the full seabed texture schema (`TERRAIN_TEXTURE_SCHEMA`: dunes,
   roughness, material layout uniform/patches/painted), **depth and
   material brushes** on a map canvas (strokes stored in
   `DepthState.paint` / `material_paint`; a depth stroke is exact inside
   its radius and blends into the base over its own short **feather**
   with a C1 distance-field smoothstep, strokes combined by the union
   rule so overlaps leave no ridge — 2026-09-05, was a linear
   triangulation to two radii; reproduced headlessly from `depth.paint`
   / `terrain.materials.paint`), and **live previews** (height and
   reflectivity, 300 ms debounce, `DepthPreviewWorker`; the former
   "sonar pass" strip was removed on 2026-09-03 — it showed texture
   contrast on one arbitrary line and misled as a survey preview). Every
   form row — texture, depth presets, walls, objects — carries a
   one-sentence hover help, the schema tuple's 7th element.
4. **Objects** — one row per `CATALOG` class: count or per-hectare
   density, optional Gaussian clustering, size/burial distributions
   (uniform or gauss), yaw orientation, a **Resting pose** line ("let
   them fall / roll (3-D)" + randomness: `orientation.rest`, 2026-09-03 —
   each object takes a physically stable pose of its primitive and
   settles on the seabed slope, `worldgen/solids.py`), per-class
   `params_schema` extras; a global seed. Each row carries an **Info…**
   button (regular size, typical shape, rest poses with their odds, and
   a top-down footprint preview of the typical instance —
   `worldgen/objects.py`: `typical_instance`, `shape_description`,
   `rest_poses_text`, `footprint_profile`). (`objects.explicit` —
   verbatim matched-pair/cluster instances — is an engine capability
   reachable from YAML/state, not yet surfaced in the GUI.)
5. **Preview & save** — objects map + depth map (rendered by the same PIL
   code the folder stores), then name → generate. Name collisions get a
   silent `_1`, `_2`… suffix, **always** — Edit-mode re-saves included.
   Worlds are immutable.

### 3.4 CLI entry points

| Command | Produces |
|---|---|
| `world_builder` | the GUI above |
| `generate_world` | `world.sdf`, `seabed.stl`, `scene.npz`, `scene_manifest.yaml` from a world YAML (headless; supports `geo:`, `depth:`, `limits_collision`, `objects.classes/explicit`) |
| `export_scene_maps` | `gt_reflectivity.png`, `gt_depth.png`, `gt_objects.png`, `gt_extent.yaml` |
| `mission_metrics` | `metrics.json`, `metrics.md` — detection metrics from ground truth; `--world <dir> --trajectory <yaml>` (either trajectory schema; legacy bundles fall back to their frozen copies) |
| `sss_calibration_report` | `calibration.json`, `calibration.md` — a recording and a world reduced to the same per-range dB statistic (`--trajectory`/`--sonar` as above) |
| `sss_svlog_compare` | `compare.json`, `compare.md` — any number of `.svlog` files (sim or real) reduced to the same timing-law, level and texture statistics (ping period vs `max(50 ms, 2R/c + 3 ms)`, dB vs slant range, floor, span, lag-1/3/10 correlations, high-pass speckle std, counter resets). The instrument the 2026-09-02 fidelity work was judged with; run it on every model change |
| `python3 -m blueboat_sss_sim.analysis.wall_contrast <svlog…>` | quay contrast of a recording (face − before, 1.5–4 m beyond − before, 6–12 m beyond − before, band width): the numbers the wall model is anchored to (`docs/sonar_model.md` §11). Not a console script; module-run |
| `sea_state_node` (`ros/`) | not a CLI: the sea-state node the launches start (`sea_state_launch.py`) |

### 3.5 Launch files

| File | Starts |
|---|---|
| `full_mission_launch.py` | Gazebo on the **world folder's** `world.sdf` + `/clock` and `/ocean_current` bridges + robot spawn + sonar/shim; `with_control:=true trajectory_file:=<yaml>` adds the unmodified control stack with **`blueboat_control`'s own `path_generation`** on `trajectory:=from_yaml:<file>` (an MCS `.deployed/<name>.yaml` works as-is; the file is watched). Dev convenience — operational missions are MCS's. |
| `sss_sim_launch.py` | sonar + shim over a world folder, acquisition passed explicitly from the resolved profile, shim origin from the world's anchor |
| `sim_world_launch.py` | Gazebo alone on a world folder |
| `sea_state_launch.py` | `sea_state_node` + the two `/world/<w>/wrench*` bridges; included by `full_mission_launch.py` (`sea_current`, `sea_current_from_deg`, `sea_waves`, `sea_waves_from_deg`, `sea_schedule`, `sea_seed`), and **standalone next to BlueBoat-Control's `Sim_launch.py`** (`world_name:=ocean`) — what the MCS spawns as a companion for an empty-world simulation |

Module-scope, launch-free resolvers the smoke test exercises directly:
`resolve_acquisition`, `resolve_sonar_config`, `resolve_world_anchor`
(`sss_sim_launch.py`) and `trajectory_total_time`
(`full_mission_launch.py`).

---

## 4. NON-NEGOTIABLE CONSTRAINTS

Violating any of these breaks another module, the hardware swap, or the
scientific argument.

1. **Interface fidelity is absolute.** Same topic names, message types, QoS
   (BEST_EFFORT/KEEP_LAST/10), parameter names, and ping-enable semantics as
   the real `sss_node.py`. **Never define a new message type** — reuse
   `blueboat_interfaces`. Additional simulation data goes on new, additive
   topics as JSON (as `ground_truth/contacts` does).

2. **`raw` frames must be byte-valid Ping Protocol, checksum included.**
   Downstream reconstructs `.svlog` files from them for SonarView. Layout:
   `'B''R' | u16 payload_len | u16 msg_id=2198 | u8 src | u8 dst | 52-byte
   fixed payload | u16[num_results] | u16 checksum(sum of all prior bytes)`.
   Frame length = `8 + 52 + 2·num_results + 2` (1262 B at 600 bins).
   `parse_frame()` is the round-trip oracle; keep it working.

3. **Both sides must be published from the same timer tick.** The
   simulator's own rule (downstream no longer forces it): one tick keeps
   the two channels' ping counters in lockstep and every simulated row
   two-sided by construction.

4. **The first bottom return must survive every change to the acoustic
   model.** Four coupled pieces make downstream bottom tracking lock, and
   the tracker fails silently if any is removed: the beam-pattern sidelobe
   floor (`beam_sidelobe_floor`), the specular near-nadir lobe
   (`specular_strength`), the specular component rendered and speckled
   **separately** from the diffuse field with low CV (`specular_looks`),
   and pulse-length range smearing (`pulse_smearing`). The smoke test
   re-verifies FBR lock with every object family — including the
   open-frame lattice classes — in the scene.

5. **Speckle statistics are load-bearing, not decoration.** Diffuse field:
   multiplicative `Exp(1)`; coherent specular: `Gamma(L, 1/L)`, low CV.
   Since 2026-09-02 the `Exp(1)` texture comes from the renderer's coherent
   ground-scatterer field (`Ping.texture`, `coherent_scatterers: true`) so
   it is also *correlated* along range (pulse) and along track (footprint
   overlap) as the corpus measures — `Ping.power` stays the noiseless mean
   and `speckle: false` is untouched. Smoke `[2g]` gates mean, CV and both
   lag-1 correlations.

6. **Power calibration is anchored to the field corpus, and moves once.**
   `calibration_db_offset` (97.2 dB) is the **only** level constant — fit
   against the reported `max_pwr_db`, never against a count histogram, and
   **only at matched altitude**: the corpus flies at 6–9 m (FBR-derived),
   the smoke fixtures at ~3.5 m, and at matched altitude 97.2 reproduces
   the gain-4 profiles within ~1 dB (re-verified 2026-09-02, when an
   unmatched comparison had suggested a 12 dB shift). The noise floor
   (8.3e-11) and water column (1.15e-7) are now **measured separately**
   (far decile of the ≥72 m files; 1–3 m band) and the gain ladder step is
   `20·log10`; those three moved on 2026-09-02 and are not to be re-fitted
   jointly again. Changing any of them silently invalidates any comparison
   against real data. `docs/sonar_model.md` §6 records which constants are
   measured, which jointly determined, which undetermined; keep it current
   whenever one moves.

7. **Never raise launch's global log level to suppress noise.** Filter by
   message content on the screen handler instead.

8. **Never modify the control stack or other packages.** Integration
   happens by serving identical interfaces, not by patching neighbours.
   (This includes the MCS trajectory discovery: `builder/paths.py`
   reimplements the predicate rather than importing `mcs.*`.)

9. **The renderer's ground-sample step must stay coupled to the slant-bin
   size.** A fixed step leaves range bins unpopulated.

10. **World folders are immutable snapshots.** A saved world is never
    overwritten — not by Create (name dedupe), not by Edit (re-saves take a
    new `_i` folder), not by tooling (`mission_metrics` etc. write only
    into `--out`). Regenerate rather than edit; `builder_state.yaml` exists
    precisely so regeneration is a load-away. Legacy mission bundles on
    disk keep loading (their frozen `sonar.yaml`/`trajectory.yaml` are the
    resolver fallbacks) and stay equally write-once.

11. **Project-level:** the thesis's headline detector trains on *real*
    imagery only. Synthetic data from this module is for the policy
    comparison, the sim-to-real transfer-gap experiment, and development.

12. **Limits never reflect.** The world-limit borders are Gazebo
    collision-only geometry; they enter neither `scene.walls`, nor the
    rasters, nor the manifest. Only authored walls are acoustic.

13. **No return from land, and no objects on it.** A cell whose terrain
    sits at or above the transducer (a dry beach, an awash bank) scatters
    nothing back into the water (`renderer._ping_geometry`, `dz > 0`
    mask; the horizon cull hides what lies beyond the waterline), and
    random placement never lands on a cell above `DRY_HEIGHT_M`
    (−0.05 m). A fully wet world is bit-identical either way.

14. **Nothing ticked means nothing placed.** `objects.classes: {}` (the
    builder with no class enabled) yields zero objects; only a config
    *without* the key falls to the legacy random composition
    (2026-09-05 fix — an empty selection used to scatter 29 objects).

---

## 5. Data: world folders, formats, what not to overwrite

`~/worlds/<path_name>/<world_name>/` (built by
`worldgen/world_folder.build_world_folder`; resolution 0.10 m,
auto-coarsened in 0.05 m steps so no raster axis exceeds 1500 cells):

| File | Kind | Consumer |
|---|---|---|
| `world.sdf`, `seabed.stl` | generated | Gazebo (spherical coords, wall collisions, invisible `limit_*` borders) |
| `scene.npz`, `scene_manifest.yaml` | generated, **ground truth** | renderer, `export_scene_maps`, metrics |
| `depth.npz` | generated | the **terrain-only** height raster (pre-object-stamp — the Screen-3 design surface) + `origin_x/y`, `resolution` |
| `preview_objects.png` | generated | the Screen-5 objects map (objects circled + walls + limits + GPS path) |
| `depth_source.csv` / `.tif` / `.svlog` | copied | Edit mode (self-contained even if the original file is gone; a recording can be tens of MB) |
| `depth_samples.npz` | generated (`svlog` mode) | the soundings (`east, north, depth, t_ms, quality, lat, lon`) and the fitted cells (`cell_*`) |
| `reflectivity_hint.npz` | generated (`svlog` + `backscatter_layer`) | the 0–1 mid-swath backscatter hint, uncalibrated |
| `source_path_world.yaml` | generated | the source path in **world** coordinates (`blueboat_trajectory/1` points, first waypoint = (0, 0) = the spawn, no geo_anchor) — drop-in `trajectory_file` for the dev launch |
| `metadata.yaml` | generated | **the cross-module contract**: `format: blueboat_world_meta/1`, source path (+ `design_offset` — world = design − offset), `geo_anchor`, `spawn` (world (0, 0) with its GPS), limits (local + 4 GPS corners SW/SE/NE/NW), walls (local + GPS + properties), every object (local + GPS + class + sizes + burial + fill), depth summary (recording worlds: `depth.svlog_quality`, incl. a `coverage` block — how much of the world was measured rather than extrapolated), seed |
| `builder_state.yaml` | generated | **Edit mode**: `format: blueboat_world_builder_state/1`, the full wizard state incl. cached path points. GUI-internal; may churn — downstream reads `metadata.yaml`, never this |

`scene_manifest.yaml` stays **version 1**: the `geo:` block
(`{anchor: {lat0, lon0, theta_deg}, convention:
equirectangular_wgs84_r6378137}`) is additive exactly like `walls:` before
it, and `PlacedObject`'s new `fill_fraction`/`params` fields load with
defaults from old manifests. `SceneModel.load` rejects any other version
outright.

RNG determinism: a world config **with** a `depth:` section splits the
seed into two streams (`[seed, 0]` terrain, `[seed, 1]` objects), so
editing the bathymetry never moves the objects — two worlds differing only
in depth pair object-by-object. A legacy config without `depth:` keeps the
single historical stream, bit-identical to every existing bundle.

Rasters are stored `(ny, nx)` with origin at the min corner;
`export_scene_maps` flips vertically for north-up images and writes
`gt_extent.yaml` for georeferencing.

---

## 6. Configuration

World YAML (consumed by `generate_world` and produced internally by the
builder via `world_folder.state_to_world_config`):

- `world:` — `seed`, `size`, `origin`, `resolution`, and
  `limits_collision` (default false; the builder always sets true).
- `geo:` — `{lat0, lon0[, theta_deg]}`, the GPS of world (0, 0).
  `theta_deg != 0` is rejected (the world frame is the anchor's ENU frame
  by definition).
- `depth:` — optional authored bathymetry: `mode: preset|file|svlog`,
  `preset` + `params` (validated against the preset's schema, unknown
  keys raise by name), or `file` + `file_kind` (csv|geotiff, from the
  suffix when empty) + `file_values` (auto|depth|elevation; auto decides
  from the sign of the median — GEBCO-style elevations negate), or
  `file` = a `.svlog` + `svlog: {datum_offset_m, trend_length_m,
  texture_gain, smoothing, cell_m, backscatter_layer}` (unknown keys
  raise by name; `docs/configuration_guide.md` "Recording mode"). Depth
  strokes carry an optional `feather_m` (default half the radius).
  When present, `terrain:` contributes only texture
  (dunes/roughness/materials); when absent, legacy behaviour is
  bit-identical.
- `terrain:` — as before (`base_depth`/`slope` are ignored under a
  `depth:` section).
- `objects:` — legacy `density_per_hectare`/`composition` **or** the
  builder's `classes: {<type>: {enabled, amount{kind: count|density,
  value}, clusters{n, sigma_m}, size{kind: uniform|gauss, ...},
  burial{...}, material, params}}` plus `explicit: [{type, x, y, yaw,
  length, ...}]` (verbatim matched pairs / RTK clusters). Classes are
  placed in sorted name order — deterministic per seed.
- `walls:` — `name,x0,y0,x1,y1,top_z,reflectivity,thickness` plus, since
  2026-09-03, `kind` (`quay` | `pontoon`), `draft_m` (pontoon), `roughness_m`
  (unknown keys raise; names unique; old manifests load with the quay
  defaults). Walls carry a Gazebo `<collision>`; a pontoon's box floats.
- `objects.classes.<type>.orientation.rest` — `{enabled, randomness}`, the
  3-D resting pose (`docs/configuration_guide.md` "3-D resting pose");
  explicit instances take `roll_deg` / `pitch_deg`.

Sonar config splits into `acquisition:` and `model:` exactly as before;
every knob description lives in `docs/sonar_model.md` (`max_ping_rate_hz`,
`alongtrack_beam_lines`, `tvg_compensation`/`spreading_exponent`,
`max_span_db`, `multipath_*`, `wall_multipath_*`, `gain_drift_amp`,
`dropped_ping_prob`, `sample_step_m`, …). Two changes on 2026-09-01:
**transmit ringing** (`ringing_level_db: 55`, `ringing_decay_db_per_m: 22`
— the corpus-measured decaying core under the transducer, on the coherent
channel, inside the water column; 0 disables) and
**`wall_multipath_enabled: true` by default** (a strict no-op without
walls — a wall-less scene renders bit-identically). On 2026-09-03 the
wall model was re-anchored on the corpus quay (`wall_face_diffuse`,
`wall_pontoon_specular`, `wall_corner_gain`, `wall_diffuse_fraction`;
ghosts carry `reflectivity²` × the Rayleigh roughness law —
`docs/sonar_model.md` §11). Unknown keys raise in both sections; the two
retired encoder keys raise by name.

Sea state: `config/sea_states.yaml` (presets, hulls) and
`config/sea_schedule_example.yaml` (a timeline) — `docs/sea_state_model.md`.
Since 2026-09-03 the wave presets are broad (γ 1.5), longer (Tp 2–4.5 s)
and carry random **wake / swell groups** (`events:` block, `sea/events.py`,
Poisson arrivals); explicit wave fields in a keyframe or command make a
custom state (the MCS "Custom" entry). Since 2026-09-05 they are
short-crested (`spread_s` 3), the groups are half as frequent, and the
`hull:` block carries `damping: {heave_ns_per_m, roll_nms_per_rad,
pitch_nms_per_rad}` (ζ ≈ 0.4 on the catamaran, judgement values) and
`wave_force_clip: {knee_g, width_g}` (a soft clip of the excess
buoyancy above 2.5 weights). Preset Hs/Tp are unchanged.

Materials: the five seabed materials plus object materials now including
`glass`, `wood`, `eps` (air-backed foam — deliberately strong, ~0.8) and
`tinplate` (`worldgen/materials.py`).

Object catalog: 27 classes — the original 13 plus the
target-families additions (`target_families_and_surrogates.md`): F1
upright boxes (`crate_plastic`, `tin_18l`, `polytank`), F2 upright
cylinders (`drum_200l`, `pail_20l`), `fire_extinguisher`, F5
`pallet_wood`, F6 `traffic_cone` (no broadside by construction), F8
`rope_coil`/`net_ball`, F9 `bottle_glass`, and the F4 open-frame family
(`bicycle`, `mesh_crate`, `shopping_trolley`) with a **fill-fraction
lattice**: a thin solid rim plus interior cells kept with probability
`fill_fraction` (deterministic per object id), so the diluted highlight
and broken shadow *emerge* from the ordinary raster path with zero
renderer change. `DEFAULT_COMPOSITION` is deliberately unchanged — new
classes enter via `objects.classes`, never by reshuffling legacy seeds.

---

## 7. Commands

All package-relative commands run from the package source root,
`BlueBoat-SSS-Sim/blueboat_sss_sim/`.

### Offline tools (no ROS required)

```bash
python3 -m blueboat_sss_sim.builder.app                 # the GUI (PySide6)
python3 -m blueboat_sss_sim.worldgen.generate \
    --config config/default_world.yaml --out ~/worlds/dev/w1 --seed 7
python3 -m blueboat_sss_sim.worldgen.export_maps --bundle ~/worlds/dev/w1
python3 -m blueboat_sss_sim.analysis.cli --world ~/worlds/dev/w1 \
    --trajectory ~/.config/blueboat_mcs/trajectories/beach.yaml --out ~/metrics/w1
python3 -m blueboat_sss_sim.analysis.calibration_cli \
    --svlog ~/ros2_ws/data/SSS_data/diffDepthCompensation.svlog \
    --world ~/worlds/dev/w1 --trajectory <yaml> --out ~/calib/w1
```

### Build, run and launch

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash

ros2 run blueboat_sss_sim world_builder

ros2 launch blueboat_sss_sim sim_world_launch.py world_dir:=$HOME/worlds/<p>/<w>
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<p>/<w>
# Per-run acquisition override (empty = the resolved profile's value)
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<p>/<w> \
    range_length_mm:=30000 gain_index:=-1
# Dev end-to-end; PID is the working controller. Run from the workspace
# root with `source env.sh` (venv + workspace) — blueboat_control's nodes
# import casadi/acados at module scope regardless of controller_type.
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=$HOME/worlds/<p>/<w> with_control:=true \
    trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<p>.yaml

ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

### Offline smoke test

```bash
python3 -m test.smoke_test        # from the package source root
```

~210 checks over geo → world folder → render → noise → encode → decode →
metrics → calibration, in sections: [1] legacy-recipe world generation
(bit-identical to the retired default mission bundle; the survey lawnmower
is frozen in `test/fixtures/survey_lawnmower.yaml` so the calibrated
anchors keep their measured context), [2]/[2b] rendering + FBR structure,
[2c] device fidelity (the 67.85 ± 0.5 dB level anchor, the corpus ping
period and pulse laws, the persistent ping counter), [2g] speckle texture
(unit mean, CV 1, lag-1 range/track correlation, seed determinism), [2d]
beam/1200-bin, [2e] calibration identifiability, [2f] the real corpus
(gated on `$BLUEBOAT_SSS_CORPUS`, default `~/ros2_ws/data/SSS_data`, skips
when absent), [4] shallow regime, [4b] acquisition constants + the
launch's own resolvers, [4c] detector metrics + `.svlog`/jsonl sources,
[4e] the `sss_svlog_compare` reductions on the in-test log, [4d] wall
multipath + the direct wall echo/occlusion (bright line at the face range,
dark behind, cos² obliquity, wall contact, FBR lock with a quay in view,
**the shipped combination** — face +19 dB, beyond −17 dB, far −32 dB
against the wall-free seabed at equal slant —, `reflectivity²` × roughness
ghost law, pontoon: no shadow, faint edge, dark water column when
crossed),
[6] GPS anchoring (equirect inverse ≡ the shim formula, manifest
geo block, `<spherical_coordinates>`), [7] authored bathymetry (CSV plane,
in-test GeoTIFF round-trip, wall-aware fill, per-preset shapes incl.
depth ≤ 0, the limit boxes over land, the smoothing spline on the plane,
the damped trend saturating at L·slope, residual-scaled texture absent
at the soundings, the svlog config guard, the brush's C1 feather /
per-stroke `feather_m` / no ridge between overlapping strokes on a
0.1 m grid, the coverage report: full cover warns not, a world far
larger than the survey warns, a 500 m miss is refused naming distance
and compass point, a near miss says redraw, a depth file gets the same
message), [4c] also the svlog GPS + attitude envelopes on a live
clock, the frozen-clock file-order pairing, `recording_extent` and
`range_bearing`, and FBR soundings matching
the scene depth under the boat (median 3 cm) on both clock regimes, [8]
objects v2 (determinism, clustering, fill-fraction dilution raster- and
render-level, FBR lock with every new family, limits invisible +
non-acoustic, `classes: {}` places nothing while a legacy config still
does, random objects stay off dry cells, no return from a dry ridge)
+ objects v3 (a standing drum through the solid path equals
the flat disc, a lying drum is diameter-high, every stable pose of six
primitives settles with its lowest point on the slope plane, per-object
determinism, enabling rest for one class moves no other object), [9]
world folders (build, `_i` immutability, Edit round-trip, metadata GPS ↔
scene positions, launch resolvers, a recording world: source copy,
`depth_samples.npz`, survey quality and coverage in the metadata,
bit-identical rebuild, cells following the recorded seabed), [10] sea state (JONSWAP
m0 ≡ Hs²/16, PM at γ = 1, dispersion limits, wave field σ and exact
zeros, hull swap static-force-free at trim, natural periods, contouring
hull carries its weight, the horizontal slope force and its zero-slope
no-op, the excess-buoyancy clip, pose prediction and damping signs, the
**closed loop under a 20 Hz pose hold** (heave in the wave band, no
0.6 s ringing, roll/pitch in the corpus band, surge and yaw driven),
Gauss-Markov current, presets/schedules/commands, wave-group process
(rate, zeros between groups, envelope, determinism, readback), launch
resolvers, marker strings, renderer pitch shift), [9] also covers
endpoint merging (idempotent, collapsed walls dropped), curve flattening
(chained chords within 0.1 m sagitta, straight walls 1:1), a curved world
building byte-identically twice, [4d] the finite-segment mirror cull,
[11] builder schema
help (every texture / depth-preset row has a one-sentence help); the
corpus section [2f] also reduces the anchor quay with `wall_contrast`.
Artifacts in `/tmp/blueboat_sss_smoke/`. ~100 s (the Stop hook allows 180 s); runs serialize on `/tmp/blueboat_sss_smoke.lock`, so a manual run and the hook's never race in the shared artefact directory. `test/` carries an `__init__.py` so the
local package wins over the stdlib `test` module.

GUI checks (needs PySide6, not part of the gate):

```bash
QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke
```

There is no lint or type-check configured; the smoke test is the only
automated gate. `.claude/settings.json` wires it as one.

### Session hooks

`.claude/settings.json` holds exactly two hooks, both implemented in
`.claude/tools/smoke_gate.py` (stdlib only):

| Hook | Fires on | Effect |
|---|---|---|
| `PostToolUse`, matcher `Edit\|Write\|MultiEdit` | an edit under `config/`, `sonar/` or the old `mission/` paths | a "regenerate the world" reminder — folders freeze their own copies (NC #10). A reminder, never a block. |
| `Stop` | end of a turn that changed a file the suite can observe | runs the smoke test from the package source root. **Exit 2 blocks the stop** until it passes. |

Smoke-test scope is `*.py` under `blueboat_sss_sim/` or `test/`, plus
`*.yaml` under `config/`. `docs/`, `*.md`, `launch/` (edits — the suite
still imports the launch resolvers), `msg_reference/`, `resource/`,
`package.xml` and `setup.*` are out of scope. The gate is offline by
construction; no hook runs ROS, `colcon`, Gazebo or Qt.

---

## 8. Working on this module

- **Read `docs/` first** — `architecture.md`, `sonar_model.md` (physics and
  assumptions A1–A10), `topics.md` (raw-frame byte map),
  `integration_guide.md`, `configuration_guide.md`, `developer_guide.md`,
  `roadmap.md`.
- Frames: world is ENU, z = 0 at the surface, depths negative; world (0, 0)
  is the GPS anchor. Internal math uses ENU yaw; anything hardware- or
  user-facing uses compass degrees. Angle conversions live in
  `core/geometry.py`; **geodesy lives only in `core/geo.py`** (the
  project-wide equirectangular convention — never introduce a second
  formula, and never import a neighbour's copy).
- Sides: `Side.PORT.sign = +1` (+y body), `STARBOARD = -1`; transducer
  heading = vehicle heading ∓ 90°. `Side.channel` (0 port, 1 starboard) is
  written to byte 34 of the raw frame and is the authoritative side
  identity downstream.
- Randomness: every stochastic component takes a `numpy.random.Generator`;
  nothing touches the global RNG. World content derives entirely from the
  seed (split streams under `depth:` — see §5).
- Adding a litter class: one `CATALOG` entry in `worldgen/objects.py`
  (`label` for the GUI, `params_schema` for class knobs,
  `fill_fraction_range` for open-frame classes) — the objects page, the
  placement engine and the metadata pick it up automatically. Adding a
  depth pattern: one `DEPTH_PRESETS` entry in `worldgen/depth/presets.py`
  (fn + label + schema rows) — the depth page renders its form
  automatically.
- Cost budget, measured on the Jazzy machine at the 20 Hz cap: renderer
  ~3.4–4.0 ms per ping-side (600–1200 bins, coherent texture on — it was
  2.0–2.4 with the per-bin draw), i.e. ~15 % of one core for both sides at
  20 Hz; wall multipath ×1.0–1.5 on the renderer only (and only when the
  world has walls); the direct wall echo and the single-bounce band are
  negligible. `sea_state_node`: the hull integration is ~1 ms per tick at
  100 Hz (`cells_per_m: 40`); the wave field rebuild on a ramp ~100 µs.
- Walls a boat can cross are **pontoons**; a quay under the hull is a
  modelling error, not a renderer bug (2026-09-03 review). Objects that
  should lie, lean or stand at random get `orientation.rest`; the
  legacy flat stamp stays the default so no existing world moves.
- The corpus carries **GPS and attitude** in every file (id-150
  `GLOBAL_POSITION_INT` / `ATTITUDE`, 25 Hz); three processor-written
  files freeze `time_boot_ms` (`read_svlog` → `clock_frozen`, file-order
  pairing). The real boat measures roll std 0.9–1.3°, pitch 0.8–1.5°,
  ~1 s — the sea-state pass band. `worldgen.depth` must not import
  `analysis` at module level (`analysis/__init__` reaches
  `worldgen.scene`): `depth/svlog.py` imports `analysis.bathymetry`
  inside the function.
- A "X looks wrong" report on the sea state is reproduced offline first
  with `sea.simulate_3dof` (the 20 Hz pose hold is the knob that
  reproduced the 2026-09-05 ringing); a depth report with
  `python3 -m blueboat_sss_sim.analysis.bathymetry <svlog> --lat0 --lon0`.
- The superproject's session rule holds here: **never `git commit` or
  `git push`** — the user stages and commits.
