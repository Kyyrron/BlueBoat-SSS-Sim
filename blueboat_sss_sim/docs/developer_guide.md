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
  `transducer_heading_deg` on the wire is the constant mounting angle
  (port −90, starboard +90 — the device convention, since 2026-09-15);
  the look direction of a side is derived from the pose, `vehicle
  heading ∓ 90°`, never read back from that field.
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
   environment patterns + `DEPTH_PRESETS`), `layers/` + `masks.py` +
   `geometry.py` + `compose.py` (the seabed layer editor's kinds, masks,
   fields and compositor), `bake.py` (a layer stack -> GeoTIFF + material
   map at build time), `recipes.py`, `ingest.py` (CSV/GeoTIFF),
   `interpolate.py` (wall-aware nearest fill), `paint.py` (brush strokes:
   depth feathering through the same interpolator, material stamping —
   RNG-free).
6. `worldgen/world_folder.py` + `builder/state.py` — the world-folder
   build pipeline and the wizard document it consumes.
7. `analysis/` — the renderer's ground truth turned into detection
   metrics; `contacts.py` is the one record type, `metrics.py` the pure
   computation over it.
8. `ros/sss_sim_node.py` — the thin shell wiring the above to
   topics/timers; `builder/wizard.py` — the GUI shell over 6 pages, each
   page riding in a `PageScroll` from `builder/layout.py` (the
   screen-size contract: `ToolBar`, `ScaledImage`, `split`,
   `fit_to_screen`).

## 3. Common extensions

### Add a litter object type
In `worldgen/objects.py`: add a `CATALOG` entry (name, material, size and
burial priors, mask primitive; optional `label` for the GUI,
`fill_fraction_range` for open-frame classes, `params_schema` for
class-specific knobs). A class whose point is to reproduce a *known*
object gives its dimensions as `params_schema` rows rather than as ranges:
`length_m` / `width_m` / `height_m` (or `diameter_m` for a round
footprint) override the drawn sizes **after** the draw, so the RNG stream
is untouched and no other object moves — `plastic_box` (the 68 L NAFCO
box of the 2026-09-07 field day) and `mooring_pile` are the two examples. If none of the existing primitives (box,
lying/upright cylinder, annulus, capsule, dashed line, cross, blob, cone,
coil, lattice-wrapped box) fits, add a mask function next to them. The
GUI objects page, the hand-placement page, the placement engine and
`metadata.yaml` pick the class up automatically; no other file changes.

Object yaw is not a class property: every class draws it per the
per-class `orientation` block (`random | fixed | path_relative`,
`_draw_yaw` in `worldgen/objects.py`). Keep its RNG contract when
touching `place_objects_v2` — every mode draws the historical
`rng.uniform(-π, π)` first (so `random` reproduces every existing seed
bit-for-bit and switching one class's mode never moves another class),
and only `jitter_deg > 0` adds a further `rng.normal` draw. The smoke
test's `[8]` section guards both.

### Hand placement and pinning (Screen 5)

Two Qt-free helpers in `worldgen/objects.py` carry the semantics, so the
GUI stays thin and the offline smoke test (`[13]`) gates them:

* `placed_to_explicit(o)` — a `PlacedObject` as an `objects.explicit`
  entry that rebuilds it exactly. It pins every field the engine would
  otherwise re-draw (`PINNED_FIELDS` + `material`), which is what makes
  the entry independent of its position in the RNG stream.
* `make_explicit_instances(type_name, x, y, rng, …)` — what one map click
  drops: `count` instances about the point with Gaussian spread
  `sigma_m`, drawn through the *same* `size`/`burial`/`orientation`
  machinery the scatter screen uses, then pinned.

Two traps the helpers exist to avoid:

1. **Rasterising mutates the pose.** `_rasterize_solid` banks a rested
   object onto the local seabed slope and writes the result back into
   `o.roll`/`o.pitch` (`objects.py`, end of the function). Pin the
   *pre-settle* pose — `_generate_with_base(..., placed_out=[])` hands it
   over — or the next build banks it a second time.
2. **A tilt key is a mode switch.** `place_objects_v2` sets `rest` away
   from `canonical` as soon as `roll_deg` or `pitch_deg` is present, which
   moves the instance from the 2.5-D mask stamp to the 3-D solid path.
   Write neither key for an object that rested flat.

Pinning the scatter is a `BuilderState`-level operation, not an engine
one: `objects.placed` holds the pinned instances and
`world_folder.engine_explicit` concatenates `placed + explicit` while
`state_to_world_config` empties `classes`. The `placed_signature`
(`builder.state.scatter_signature`) records the seed and classes the pin
came from, so Screen 5 can tell the operator when Screen 4 has moved on.

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

### Add a seabed layer kind
One `register(LayerSpec(...))` in a module under `worldgen/depth/layers/`
(imported at the bottom of `layers/__init__.py`): a pure function
`fn(grid, geometry, params, rng, ctx) -> LayerOutput` returning an
absolute `height` field or a `delta` on the composite below (`ctx.height`),
optionally `material=(ids, names)` (int16, -1 = untouched), `gain_db`, a
`default_mask` (the layer's own extent) and `info`; a `label`, a `family`
(the add-layer menu group), 7-tuple schema rows *with a one-sentence help*
(the smoke test [11] refuses a row without one), the accepted `geometry`
types (`polygon | polyline | ellipse | rect | axis | radial | strokes`, or
none), the `channels` it writes, a `reference` naming `docs/seabed_model.md`
entries, a `consequence` for the sonar image, and its default `blend`.
The editor's menu, property form, compositor, bake, recipes and the
schema-help check pick it up with no GUI change. Build fields from
`worldgen/depth/geometry.py` (polygon inside/signed distance, polyline
(s, n) frame with curvature, ellipse, axis, line families, wall-confined
feathers) and draw randomness only from the ``rng`` handed in — it is the
layer's own stream, so editing one layer never moves another's texture.
A kind whose parameters can invert what its blend mode means may return
`blend="normal"` (or any mode) on its own `LayerOutput`, overriding the
configured one for that result only — `basin` does it when its design
floor lands *above* the bed, where "deepen" (a minimum) would silently do
nothing. Two shared helpers in `geometry.py` are worth knowing before
writing a new outline: `round_region` (opening then closing with a disc,
through the Euclidean distance transform — what turns a clicked polygon
into a shoreline) and `signed_distance_of` (negative inside, positive
outside), which `land` composes into a C1 shore face.

Add the geomorphology and its citation to `docs/seabed_model.md`; a
recipe that uses the kind goes into `config/seabed_recipes.yaml`
(smoke test [12] instantiates every recipe).

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

### Add a control, a tool row or a preview to a wizard page
`builder/layout.py` is the screen-size contract: a page's *layout
minimum* is what the window can be shrunk to, so a row that only grows is
a window that cannot be resized. (It happened: the depth editor's tool
row measured 1687 px wide, which put the window's minimum at 2335 px —
wider than any screen, and Qt refuses to resize a window below its layout
minimum.)

* **Tool rows use `ToolBar`**, never a bare `QHBoxLayout`. It lays out
  with `FlowLayout`, so controls wrap onto the next line and the row's
  minimum width is its *widest single control*. Group with
  `bar.separator()` and caption with `bar.label(...)`; `addStretch` /
  `addSpacing` have no meaning in a wrapping row. The bar grows to at
  most `ToolBar.MAX_ROWS` (3) rows and scrolls beyond that, so a narrow
  window costs the page a bounded strip rather than half its height.
* **Previews use `ScaledImage`** and `set_image(QImage | QPixmap |
  path)`. It keeps the source pixmap and rescales on every resize, so a
  preview rendered while the window was small stays sharp when it is
  enlarged; `clear_image(text)` puts the placeholder back.
* **Side-by-side panels use `split(...)`** (a `QSplitter`) with stretch
  factors, not a `QHBoxLayout` with fixed shares — the operator can then
  drag a panel narrow, or shut, on a small display. Give a panel a modest
  `setMinimumWidth`: it is a floor on the whole window.
* **The page itself needs nothing**: `wizard.py` wraps each one in a
  `PageScroll`, which hands the page the viewport and scrolls only below
  the page's own minimum. It deliberately does *not* use
  `setWidgetResizable(True)`: that sizes the page by `heightForWidth`,
  which any word-wrapped label inflates to the page's ideal height, so
  the window would grow a scroll bar instead of compressing the map.
* Check the result in `test/gui_smoke.py` `[g8]`: every screen's layout
  minimum must fit a 1000×600 display.

### Replace the acoustic model
Subclass `SonarRenderer` (`render(side, pose, t_sim) -> RenderedPing`) and
instantiate it in `sss_sim_node._start_pinging`. Everything downstream
(noise, encoding, topics, metrics) is renderer-agnostic. Keep
ground-truth contact emission if you want the metrics to survive.

### Add a reflecting boundary
Declare it under `walls:` in the world config (`kind: quay` for a face
from the seabed, `kind: pontoon` for a floating one the sonar looks
under) — no code change. The wall travels in `scene_manifest.yaml`, so
the renderer, the ground-truth contacts and the Gazebo world read one list. Face echo,
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
by mask name (set `rest_allowed=False` on a class fixed in place, like the
driven `mooring_pile`, so it never gets rest poses); a new primitive needs a `line_interval` branch (the
vertical-ray interval through the solid) and a `stable_poses` entry. The
stamp path (`objects._rasterize_solid`), the SDF pose and the metadata
follow automatically.

### Tune realism
All noise/physics knobs are in the sonar YAML (`configuration_guide.md
§2`); zeroing the noise group yields clean-physics images for ablation.

## 4. Testing

* `python3 -m test.smoke_test` — the offline gate (439 checks: geo →
  world folder → render → encode → decode → metrics → calibration),
  artifacts in `/tmp/blueboat_sss_smoke/`. Run it after any change; it
  needs only numpy/scipy/yaml/PIL.
  `QT_QPA_PLATFORM=offscreen python3 -m test.gui_smoke` covers the GUI,
  including `[g8]` screen adaptation — every page's layout minimum fits a
  1000×600 display, tool rows wrap, previews rescale, and a page
  compresses to the window before any scroll bar appears.
* Frame-level: `parse_frame(encode(...))` round-trips are the contract
  for the raw stream; extend those checks if you touch `encoder.py`.
* Metrics: `mission_metrics --world <dir> --trajectory <yaml> --out <dir>`
  replays deterministically — the same world, and a world regenerated from
  its `builder_state.yaml`, must print the same content digest. If a
  change moves that digest, it changed the ground truth, not just the
  code.
* Sim-vs-real comparison: `sss_svlog_compare a.svlog b.svlog … --out <dir>`
  reduces any `.svlog` (simulated or real) to the timing law, level and
  texture statistics the 2026-09-02 fidelity work was judged by
  (`compare.md` + `compare.json`); run it on every model change.
* In-graph: `ros2 topic hz /side_scan_sonar/port/profile` should match
  `ping_period_s()` — **20 Hz** at the default 20 m free-run, the device's own cap
  (`max(50 ms, 2R/c + 3 ms)`, measured on all seven corpus recordings;
  the "45 Hz / 22 ms" this line used to claim was a misreading, retired
  2026-09-02) — and
  `ros2 topic echo --once .../raw | head` should start with `[66, 82,`
  (`'B'`, `'R'`).

## 5. Performance notes

`GeometricRenderer` costs one heightfield line sample (~4000 points at
the default 20 m / 1200 bins: the step is 0.30 × the 16.7 mm slant bin)
plus O(n) numpy ops per ping per side — ~6–7 ms with the coherent texture
on (the 2026-09-19 image-formation rework roughly doubled the earlier
3.4–4.0 ms), i.e. dual-channel at the device's 20 Hz cap uses ~25–30 % of
one core, inside the 25 ms real-time budget per ping. (Since 2026-09-18 the direct pass also
accumulates the unoccluded field, one extra `bincount` per line, for the
shadow's reverberation floor.)
`sample_step_m` and scene `resolution` are the levers if you enlarge
worlds. World generation is seconds; the STL is decimated to ≤200
vertices/axis purely for Gazebo visuals (acoustics always uses the
full-resolution raster).

## 6. Gotchas

* Ground truth is keyed by `(side, ping_number)`; if you add a second
  sonar node instance, namespace it or the keys collide downstream.
* `dropped_ping_prob` advances the encoder's counter on drops
  (hardware-faithful) — don't "fix" that.
* `world.resolution` coarser than ~0.10–0.20 m quantises shadow edges and
  object outlines to the cell (brightness is resolution-independent to
  ~0.2 dB: `sonar_model.md` A6, smoke `[8]`).
* Launch files follow the project's `simple_launch` style; keep new ones
  consistent.
