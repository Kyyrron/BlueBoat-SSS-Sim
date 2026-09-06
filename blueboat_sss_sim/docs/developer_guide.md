# Developer guide

For anyone extending the package. Read `architecture.md` first.

## 1. Conventions

* **Frames.** World = ENU, z = 0 at the water surface, depths negative in
  the heightfield. Headings: internal math uses ENU yaw (rad, CCW from
  +x); anything user/hardware-facing uses compass degrees
  (`enu_yaw_to_compass_deg`, 0 = North, CW) — conversion lives in
  `core/geometry.py` only.
* **Sides.** `Side.PORT.sign = +1` (+y in body frame), `STARBOARD = −1`;
  `Side.PORT.channel = 0`, `STARBOARD = 1` — the channel number the
  encoder writes to byte 34 of the raw frame and to
  `OmniscanProfile.channel_number`, which is what `.svlog` writers and
  readers route on (never the topic or the `src` device tag).
  Transducer heading = vehicle heading ∓ 90°.
* **Randomness.** Every stochastic component takes a
  `numpy.random.Generator`; nothing touches the global RNG. World content
  derives entirely from the seed (worlds with a `depth:` section split it
  into terrain and object streams so bathymetry edits never move objects).
* **Typing/docs.** Full type hints, module docstrings state contracts;
  `core/` and the science modules must stay importable without ROS **and
  without Qt** (only `builder/` imports PySide6, and its `state.py` /
  `paths.py` stay Qt-free so the smoke test drives builds headless).
* **Geodesy lives only in `core/geo.py`** — the project-wide
  equirectangular convention. Never add a second formula or import a
  neighbour module's copy.
* **No new messages.** The package reuses `blueboat_interfaces`. If you
  need new data channels, prefer JSON on `std_msgs/String` (as the
  ground-truth topic does) or add messages in `blueboat_interfaces`
  itself.

## 2. Code tour (read in this order)

1. `core/types.py` — every dataclass that crosses module boundaries.
2. `worldgen/scene.py` — `SceneModel`: the single source of truth;
   `generate_scene` composes `terrain.py` + `objects.py`.
3. `sonar/renderer.py` — `GeometricRenderer.render()` contains the entire
   acoustic pipeline for one ping; `acoustics.py` holds the pure formulas
   and `multipath.py` the wall/surface mirror sources (the same geometry
   and shading passes, run from a reflected transducer).
4. `sonar/encoder.py` — quantisation + Ping-Protocol framing;
   `parse_frame` is its own unit test oracle.
5. `worldgen/depth/` — authored bathymetry: `presets.py` (the ten
   environment patterns + `DEPTH_PRESETS`), `ingest.py` (CSV/GeoTIFF),
   `interpolate.py` (wall-aware nearest fill), `paint.py` (brush strokes:
   depth feathering through the same interpolator, material stamping —
   RNG-free).
6. `worldgen/world_folder.py` + `builder/state.py` — the world-folder
   build pipeline and the wizard document it consumes.
7. `analysis/` — the renderer's ground truth turned into detection
   metrics; `contacts.py` is the one record type, `metrics.py` the pure
   computation over it.
8. `ros/sss_sim_node.py` — the thin shell wiring the above to
   topics/timers; `builder/wizard.py` — the GUI shell over 5 pages.

## 3. Common extensions

### Add a litter object type
In `worldgen/objects.py`: add a `CATALOG` entry (name, material, size and
burial priors, mask primitive; optional `label` for the GUI,
`fill_fraction_range` for open-frame classes, `params_schema` for
class-specific knobs). If none of the existing primitives (box,
lying/upright cylinder, annulus, capsule, dashed line, cross, blob, cone,
coil, lattice-wrapped box) fits, add a mask function next to them. The
GUI objects page, the placement engine and `metadata.yaml` pick the class
up automatically; no other file changes.

Object yaw is not a class property: every class draws it per the
per-class `orientation` block (`random | fixed | path_relative`,
`_draw_yaw` in `worldgen/objects.py`). Keep its RNG contract when
touching `place_objects_v2` — every mode draws the historical
`rng.uniform(-π, π)` first (so `random` reproduces every existing seed
bit-for-bit and switching one class's mode never moves another class),
and only `jitter_deg > 0` adds a further `rng.normal` draw. The smoke
test's `[8]` section guards both.

### Add a depth pattern
One entry in `DEPTH_PRESETS` (`worldgen/depth/presets.py`): a pure
`fn(grid, params, rng) -> height` plus a label and the SchemaForm rows
for its parameters. The GUI depth page renders the form automatically,
and its live previews (depth, reflectivity —
`builder/worker.py: DepthPreviewWorker` over `compute_depth_preview`)
follow the form. Give every schema row its 7th element, a one-sentence
`help` (shown as the hover tooltip; the smoke test refuses a row without
one). A preset that uses `rng` draws from the terrain stream
(`[seed, 0]`), so it never moves the objects.

### Add a seabed texture knob
Add the field to `TerrainConfig` (+ its key to `_SUB_KEYS`, the
unknown-key guard) and one row to `TERRAIN_TEXTURE_SCHEMA` with a dotted
key (`dunes.wavelength`); `flatten_texture`/`unflatten_texture` and the
Screen-3 form pick it up. `DepthState.texture` stores the nested dict.

### Painting
Strokes (`worldgen/depth/paint.py`) are the one paint primitive: the map
canvas (`builder/map/canvas.py`, `Mode.DEPTH_BRUSH` / `MATERIAL_BRUSH` /
`ERASER`, `set_strokes()`/`strokes()`) collects them in the design frame,
`DepthState.paint` / `material_paint` store them, `world_folder.
depth_section` / `terrain_section` re-base them by the spawn origin into
`depth.paint` / `terrain.materials.paint`, and `build_height` /
`_material_map` apply them. Keep painting RNG-free.

### Replace the acoustic model
Subclass `SonarRenderer` (`render(side, pose, t_sim) -> RenderedPing`) and
instantiate it in `sss_sim_node._start_pinging`. Everything downstream
(noise, encoding, topics, metrics) is renderer-agnostic. Keep
ground-truth contact emission if you want the metrics to survive.

### Add a reflecting boundary
Declare it under `walls:` in the world config (`kind: quay` for a face
from the seabed, `kind: pontoon` for a floating one the sonar looks
under) — no code change. The wall travels in `scene_manifest.yaml`, so
the renderer, the labeler and the Gazebo world read one list. Face echo,
occlusion, the single-bounce band and the mirror ghosts all key on the
face band `[Wall.z_bottom, top_z]`; a ghost renders through the same
passes as the direct path and carries a ground-truth contact naming the
object it images and the reflector that made it. Check any level change
with `python3 -m blueboat_sss_sim.analysis.wall_contrast` against the
corpus numbers in `sonar_model.md` §11.

### Curved and merged walls
`core/geometry.py` owns the maths (`merge_endpoints`, `quad_bezier_polyline`,
`wall_segments`, `expand_walls`) and is Qt-free; `world_folder.
state_to_world_config` and the Screen-3 preview both call `expand_walls`,
so never expand elsewhere. The canvas draws the same Bézier
(`BuilderMapView._wall_path`) from the same `ctrl` point; the engine only
ever sees straight chords, so nothing acoustic changes. Adding a wall
property: one `_WALL_SCHEMA` row (with help), one `Wall` field.

### Give an object class 3-D rest poses
`ObjectSpec.solid()` maps the class's mask to a `worldgen/solids.Solid`
by mask name; a new primitive needs a `line_interval` branch (the
vertical-ray interval through the solid) and a `stable_poses` entry. The
stamp path (`objects._rasterize_solid`), the SDF pose and the metadata
follow automatically.

### Tune realism
All noise/physics knobs are in the sonar YAML (`configuration_guide.md
§2`); zeroing the noise group yields clean-physics images for ablation.

## 4. Testing

* `python3 -m test.smoke_test` — the offline gate (~150 checks: geo →
  world folder → render → encode → decode → metrics → calibration),
  artifacts in `/tmp/blueboat_sss_smoke/`. Run it after any change; it
  needs only numpy/scipy/yaml/PIL.
  `QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke` covers the GUI.
* Frame-level: `parse_frame(encode(...))` round-trips are the contract
  for the raw stream; extend those checks if you touch `encoder.py`.
* Metrics: `mission_metrics --world <dir> --trajectory <yaml> --out <dir>`
* Sim-vs-real comparison: `sss_svlog_compare a.svlog b.svlog … --out <dir>`
  reduces any `.svlog` (simulated or real) to the timing law, level and
  texture statistics the 2026-09-02 fidelity work was judged by
  (`compare.md` + `compare.json`); run it on every model change.
  replays deterministically — the same world, and a world regenerated from
  its `builder_state.yaml`, must print the same content digest. If a
  change moves that digest, it changed the ground truth, not just the
  code.
* In-graph: `ros2 topic hz /side_scan_sonar/port/profile` should match
  `ping_period_s()` (≈45 Hz per side at 15 m free-run… i.e. 22 ms), and
  `ros2 topic echo --once .../raw | head` should start with `[66, 82,`
  (`'B'`, `'R'`).

## 5. Performance notes

`GeometricRenderer` costs one heightfield line sample (~600 points at the
default `sample_step_m`) plus O(n) numpy ops per ping per side — a few
hundred µs; free-run dual-channel at 45 Hz uses well under one core.
`sample_step_m` and scene `resolution` are the levers if you enlarge
worlds. World generation is seconds; the STL is decimated to ≤200
vertices/axis purely for Gazebo visuals (acoustics always uses the
full-resolution raster).

## 6. Gotchas

* Ground truth is keyed by `(side, ping_number)`; if you add a second
  sonar node instance, namespace it or the keys collide downstream.
* `dropped_ping_prob` advances the encoder's counter on drops
  (hardware-faithful) — don't "fix" that.
* `world.resolution` coarser than ~half a range bin causes visible
  stair-stepping in shadows.
* Launch files follow the project's `simple_launch` style; keep new ones
  consistent.
