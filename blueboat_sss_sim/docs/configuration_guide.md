# Configuration guide

Two YAML layers, both with realistic defaults so an empty file works:

1. **World** (`config/default_world.yaml`) — geometry, geo anchor,
   bathymetry, materials, objects, walls.
2. **Sonar** (`config/default_sonar.yaml`) — acquisition + physics/noise.

The World Builder GUI writes the world side for you (its wizard state is
`builder_state.yaml`, translated through
`worldgen/world_folder.state_to_world_config`); `generate_world` consumes
a world YAML directly. Unknown keys **raise by name** (typo protection)
in the sonar `acquisition:`/`model:` sections, in `terrain:` (and its
`slope`/`dunes`/`roughness`/`materials`), in `walls:` entries, in `depth:`
(its preset `params:`, layers, masks, `svlog:` and paint strokes) and in
an object's `orientation:` / `orientation.rest:`. They are **silently
ignored** in `world:`, at the top level of `objects:` and inside a class
entry, in `objects.explicit` entries, in `materials:` overrides and in
extra `geo:` keys — a typo there falls back to the default.

## 1. World config

```yaml
world:
  seed: 42                # every stochastic choice derives from this
  size: [80.0, 60.0]      # m
  origin: [-40.0, -30.0]  # world xy of min corner
  resolution: 0.10        # raster cell, m — 0.10 is the validated default; the renderer
                          # interpolates (a cell is ~6 range bins wide): it limits relief and
                          # outline detail, not brightness
  # Beyond the grid the sonar sees a phantom plain (deepest cell, rho 0.2) in
  # `outside: fill` worlds (hand YAML, legacy); builder worlds are `outside: edge`
  # (the edge cell repeated, manifest key `outside`). See sonar_model.md A13.
  limits_collision: false # true (builder worlds): four invisible
                          # collision-only border boxes; never acoustic

geo:                      # optional; the GPS of world (0, 0)
  lat0: 33.930824
  lon0: 130.7311812
  # theta_deg other than 0 is rejected: the world frame IS the anchor's
  # local ENU frame (identical to the MCS path design frame).
```

With `geo:` present the manifest gains an additive `geo:` block,
`world.sdf` a `<spherical_coordinates>` element, and
`sss_sim_launch.py` feeds the MAVROS shim from it automatically.

### depth (optional — authored bathymetry)

```yaml
depth:
  mode: preset            # preset | file | svlog
  preset: harbour_basin   # see the preset table below
  params: {basin_depth: 2.5, edge_shoal_m: 0.8}   # per-preset schema
  # paint:                        # optional depth brush strokes (see "Painting")
  #   - {points: [[10.0, 5.0], [18.0, 6.0]], radius_m: 3.0, value: 5.5, feather_m: 1.0, erase: false}
  # mode: file
  # file: soundings.csv           # or a geographic-CRS GeoTIFF (.tif)
  # file_kind: ""                 # csv | geotiff; "" = from the suffix
  # file_values: auto             # auto | depth | elevation
  # mode: svlog
  # file: survey.svlog            # a Cerulean recording (see "Recording mode")
  # svlog: {datum_offset_m: 0.0, trend_length_m: 15.0, texture_gain: 0.5,
  #         smoothing: 1.0, cell_m: null, backscatter_layer: false,
  #         relief: synthetic, relief_gain: 1.0, relief_max_m: 1.5}
```

Depth values are metres positive down; **zero or negative is dry land**
(a beach above the waterline). Presets whose shallow edge can be dry
accept negative minima (`flat.depth`, `beach_profile.min_depth`,
`sloped_plane.min_depth`, `rocky_outcrop.min_depth`, `canal.bank_depth`,
down to −5 m); the wet-channel presets (`river_channel`,
`dredged_channel`, `marina`, `sandbar_ripples`) keep their 0.1 m floor.
Land is handled downstream: the limit borders reach over it, the sonar
gets **no return from ground at or above the transducer** (a dry beach
scatters nothing back into the water, and the horizon cull hides what
lies beyond the waterline), and random objects are never placed on a
cell with height above −0.05 m (explicit instances stay where the
author pins them).

When `depth:` is present it produces the *base* seabed and the `terrain:`
section contributes only texture (dunes, roughness, materials —
`base_depth`/`slope` are ignored); when absent, legacy behaviour is
bit-identical. Presets (shapes and ranges grounded in `seabed_model.md`,
the harbour/river/beach/canal literature; parameters and bounds in
`worldgen/depth/presets.py`, which also feeds the GUI forms):

| Preset | Shape |
|---|---|
| `harbour_basin` | flat dredged floor, silt shoaling toward the edges, tanh dredge scarp, optional deeper channel |
| `river_channel` | asymmetric parabolic cross-section along an axis (thalweg offset toward one bank), pool–riffle undulation |
| `beach_profile` | concave-up Dean `h = A·x^(2/3)` from the shallow edge, optional longshore sandbar |
| `canal` | trapezoid: flat channel, sloped sides, floored banks |
| `flat` | one depth everywhere |
| `sloped_plane` | depth at the area centre + a grade along a direction (floored) |
| `dredged_channel` | navigation cut through a basin (tanh scarps) with a Gaussian spoil shoal hugging each side |
| `marina` | pontoon rows at a pitch, deeper tanh-edged berth pockets between them, scour pits at the piles |
| `rocky_outcrop` | seeded super-Gaussian rock heads over a base depth (positions from the terrain RNG stream — objects never move); **height only** — the heads keep the base material (for rock tone use a rocks patch or paint, or the layered editor's `outcrops` layer, which stamps rocks) |
| `sandbar_ripples` | periodic bar field with crests along a heading, optional sawtooth asymmetry |

`beach_profile` also takes `berm_enabled` (+ `berm_distance_m`, `berm_height_m`,
`berm_width_m`): a short shore-parallel ridge near the shallow edge, off by
default so existing beach worlds are bit-identical.

**Layered worlds (the World Builder's seabed editor).** The builder's
Screen 3 authors the seabed as a layer stack over one of the three
sources (`docs/seabed_model.md` §7: gradients, landforms, relief,
adjustments, material layers, a reflectivity gain, brush layers, masks,
blend modes, recipes). The stack lives only in `builder_state.yaml`
(`depth.layers`); at build time it is **baked** (`worldgen/depth/bake.py`)
into `depth_baked.tif` — a float32 north-up EPSG:4326 GeoTIFF whose pixel
lattice is the world grid — and `material_map.npz`, so the world YAML of
a layered world is an ordinary file-mode section::

```yaml
depth: {mode: file, file: /abs/path/depth_baked.tif, file_kind: geotiff, file_values: depth}
terrain:
  materials: {layout: uniform, uniform: sand, composition: {}, map: /abs/path/material_map.npz}
```

`load_geotiff` recognises a raster aligned with the grid and stamps it
losslessly (no striding, no interpolation); any other GeoTIFF follows the
historical path. `generate_world` on such a snapshot reproduces the
builder's world bit-for-bit (smoke test [12]).

File mode: CSV is `lat, lon, depth` with or without a header (columns
found by name, any order); GeoTIFF is read through Pillow
(ModelTiepoint/ModelPixelScale tags), **geographic CRS only** — projected
(UTM) files are rejected with a message. `file_values: auto` decides
depth-vs-elevation from the sign of the median (GEBCO-style elevations
negate). Interpolation is linear inside the data; outside it every cell
takes its BFS-nearest known value **without ever crossing a wall** —
"extend the furthest known depth until a wall (or the world limit) is
found". Walls partition the interpolation too: soundings on the two sides
of a quay never blend.

**Recording mode** (`mode: svlog`, 2026-09-05): `file` is a Cerulean
`.svlog` recording. The recording's first bottom return (the processor's
own detector, re-expressed in `analysis/bathymetry.py`) gives the depth
under the boat per ping, each side detected independently and fused
robustly (running median per side, agreeing sides take the larger, a
lone side is kept only near the running altitude); each sounding is
placed at the recording's `GLOBAL_POSITION_INT` fix — interpolated onto
the ping clock on a live autopilot clock, or the most recent fix in file
order when the clock is frozen (processor-written recordings) — and
converted to depth with the transducer submersion (`sensor_depth_m`) plus
`datum_offset_m` (tide / draft, positive when the water stood higher
than the reference). The soundings are decimated to 1 m cell medians
(coarser until ≤ 2500 cells; `cell_m` forces a size), fitted with a
**smoothing thin-plate spline** inside their hull (`smoothing`: scipy's
per-sample penalty, 0 = exact), continued **beyond** the hull with the
edge slope damped over `trend_length_m` (the continuation saturates at
`L × slope`; a plane stays a plane), and given a seeded undulation of
5–25 m wavelength whose RMS is `texture_gain` × the soundings' residual
about the fit, zero within 2 m of a sounding. A single straight line of
soundings (no across-track information) falls back to the nearest-value
fill. Walls partition the fit and the trend exactly as in file mode.
The folder keeps `depth_source.svlog`, `depth_samples.npz` (the raw
soundings and cells) and a `depth.svlog_quality` block in
`metadata.yaml`.

**Relief between the lines** (`relief`, 2026-09-16). `synthetic` (the
default, every older config bit-identically) is the seeded undulation
above. `shading` reads the relief the recording itself carries: every
profile bin past the bottom return is inverted through the calibrated
sonar model (`analysis/backscatter.py`), averaged over ~2 m of track and
0.25 m of range against the speckle, split into reflectivity (the linear
mean over a 20 m window along range) and slope (what the ratio deviates
from it by — Lambert inverted, `cos θᵢ = sin θ_g (I/ρ)^(1/p)`), integrated
outward from the FBR depth with the ground range re-solved once, and
detrended over `trend_length_m` along the ray so only the *relief* is
kept (`analysis/shading.py`: Cuschieri & Hebert 1990, Coiras et al. 2007,
Xie et al. 2022 — `seabed_model.md` §7.1). The measured relief replaces
the synthetic undulation where the swath was lit and fades into it over
2 m beyond. Its amplitude is uncertain by the sonar's Lambert exponent
(a factor of about 1–2, and ×0.7 measured against the simulator's own
ripples): `relief_gain` scales it, and `relief_gain: 0` **calibrates**
it by regressing the shading relief on the relief the FBR of the *other*
lines measured where they cross the swath (Ran et al. 2025, §2.2.3),
reported as `gain` / `gain_pairs` / `gain_residual_m` in the quality
block together with `coverage_fraction` (cells within `trend_length_m`
of a sounding that carry a relief sample), `rms_m`, `overlap_rms_m` (two
passes more than a minute apart disagreeing on a cell — the shading
analogue of the crossover residual), `shadow_fraction` and
`clamped_fraction` (`relief_max_m` clamps a bright object or a wall).
`none` leaves the smooth fitted surface. The builder writes `shading`
for a new world and keeps `synthetic` for a state that never chose.

**Blending the assembled relief** (`relief_smooth_m`, default **0.75**,
2026-09-18). Each pass leaves a hole at its own nadir — the water column
carries no bin — and meets its neighbours at a seam, and neither is
seabed. Rasterised raw, that prints the boat's track into the bathymetry,
which is what the 2026-09-18 report *"feels like the path of the boat is
printed and that's all"* was: the striping was the survey geometry, not
the seabed. The assembled field is therefore smoothed by a
**coverage-weighted** Gaussian (normalised convolution: the same weights
that formed each cell mean also weight the blur), which bridges the holes
and the seams without pulling the surrounding relief toward zero. A
Gaussian of σ attenuates a wavelength L by `exp(−2(πσ/L)²)`, so 0.75 m
keeps **90 %** of a 10 m ripple while removing the sub-metre track
imprint; `0` restores the raw assembly. Smoke `[7]` gates it: the nadir
holes go from 100 % to 0 % of the cells, the residual there drops 3×, the
10 m bedform keeps 0.89 of its amplitude and the field ends up closer to
the truth than the raw assembly.

The inversion itself is **cached** per `(file, mtime, size, anchor,
config)` (2026-09-18): the World Builder re-reads a selected recording on
every parameter change and every preview, and a 180 MB survey takes
minutes. It also runs in along-track chunks with a halo, so peak memory
follows the chunk rather than the survey — a 129 MB recording inverts in
1.6 GB instead of the ~10 GB that was killing the builder outright.

**Reflectivity from the recording** (`backscatter_layer`, reworked
2026-09-16). The key now stands for the builder's **`reflectivity_survey`
layer** (Screen 3, one tick): the same per-bin inversion rasterised as
Lambert's coefficient per ground cell — the recording's median rendered
as the `reference` material (sand) and every cell brighter or darker
than it as a continuous reflectivity gain, plus a coarse mud / gravel /
rock classification by the literature contrasts (Jackson & Richardson
2007 §12.1 carried to 450 kHz, cross-checked with Fonseca et al. 2025:
mud −3.5 dB below, gravel +2.5 dB and rock +5.5 dB above the median,
all editable on the layer). The folder keeps the audit record
`reflectivity_survey.npz` and a `depth.svlog_quality.backscatter` block
(pings, bins, cells, μ dB percentiles, the floor used, the angular
residual slope). The layer is an ordinary row: mask it, reorder it, edit
its thresholds, delete it or untick the box. The former disc-splat
"hint" and its `reflectivity_hint.npz` are gone. How to sail for it:
`svlog_depth_survey.md`.

**Coverage** (both external sources): a recording or a depth file places
its samples by GPS, so it can miss the world entirely. A build with no
sample inside the limits is refused with a message naming what the source
covers, what the world covers, the distance between the two centres and
the compass point (a near miss says to redraw the rectangle; a far one to
pick another file or path). A build with a *partial* overlap proceeds and
records `coverage` in the survey block: samples inside the limits, and
the share of the world within `trend_length_m` of a sounding. Below 35 %
of that share, or below half the samples inside, the build warns — the
rest of the seabed is the damped trend, not a measurement.

RNG note: a world **with** `depth:` splits the seed into separate terrain
and object streams, so changing the bathymetry never moves the objects
(two worlds differing only in depth pair object-by-object). A legacy
config without `depth:` keeps the historical single stream. The
recording mode's texture draws from the terrain stream, so it never
moves the objects either.

### Painting (depth and material strokes)

Both `depth.paint` and `terrain.materials.paint` are lists of **strokes**
(`worldgen/depth/paint.py`), the World Builder's Screen-3 brushes made
YAML-native so a painted world reproduces headlessly:

```yaml
- {points: [[x, y], ...], radius_m: 3.0, value: 5.5, feather_m: 1.5, erase: false}
```

`points` are world metres (the builder paints in the design frame and
re-bases at build time exactly like walls); `radius_m` is the brush
radius; `value` is a depth [m, positive down — zero or negative is dry
land] for `depth.paint` and a material name for `materials.paint`;
`erase: true` removes earlier paint under the disc. Strokes apply in
order, later ones winning. Unknown stroke keys and unknown material
names raise by name. `feather_m` (depth strokes only; default half the
radius) is the width of the blend back to the surrounding seabed.

A depth stroke sets the seabed to exactly `value` inside its disc and
blends back to the base over its feather with a C1 smoothstep of the
analytic distance to the stroke (a dot feathers radially, a polyline as
a smooth capsule). Strokes combine with the union rule and an eraser
cuts with its own smooth weight, so overlapping strokes of different
depth leave **no ridge** in their shared feather (the value blended
toward is a Gaussian average of the nearby paint); two touching
plateaus of different value keep their own hard step, by design. Only
the wall-regions holding paint are touched — a stroke on one side of a
quay never changes the other side — and cells beyond every feather are
untouched. Painting draws no RNG: two builds of the same strokes are
bit-identical, and a world without strokes is unchanged. (Before
2026-09-05 the feather was a linear triangulation out to two radii;
worlds built then are snapshots and keep their surface.)

### terrain (texture)

| Key | Default | Effect |
|---|---|---|
| `base_depth` | 4.0 | mean depth (m) — **ignored under a `depth:` section** |
| `slope.direction_deg`, `slope.grade` | 0, 0 (`default_world.yaml` ships 15, 0.015) | planar tilt — likewise ignored under `depth:` |
| `dunes.{enabled, wavelength, amplitude, direction_deg, irregularity}` | on, 6, 0.12, 30, 0.5 | sand-wave field |
| `roughness.{amplitude, octaves, cells}` | 0.05, 5, 12 | fBm micro-relief |
| `materials.layout` | `patches` | `uniform` (one material), fBm-rank `patches`, or `painted` (the `uniform` material as the base for `paint`) |
| `materials.composition` | sand-dominant mix | area fractions, auto-normalized (`patches` only) |
| `materials.patch_cells` | 6 | patch spatial scale |
| `materials.uniform` | `sand` | the single material for `uniform`, the base for `painted` |
| `materials.paint` | `[]` | material strokes stamped over the base layout in order (any layout); see "Painting" |
| `materials.map` | `""` | a baked `material_map.npz` (`worldgen/material_map.py`: `material_id` int16 with −1 = base layout, `material_names`, optional `gain`, its grid) stamped over the layout after the paint; its `gain` multiplies the reflectivity before the clip. Written by the builder for layered worlds; absent = no-op |

Unknown keys anywhere in `terrain:` (top level or inside `slope`, `dunes`,
`roughness`, `materials`) raise by name, like `depth:` and `walls:`. The
World Builder renders these knobs from `worldgen/terrain.py`'s
`TERRAIN_TEXTURE_SCHEMA` (its defaults are the builder's quiet seabed:
dunes off, roughness 0.03 m, one material), and stores the full nested
dict in `builder_state.yaml` under `depth.texture`; older states that
carry only `{dunes: {enabled}, roughness: {amplitude}}` load with the
schema defaults for every absent key.

### objects

Legacy random scattering **or** the builder's per-class config; a config
carrying a `classes` mapping or `explicit` entries uses the per-class
engine. **`classes: {}` means no objects at all** (the builder with
nothing ticked); only a config *without* the key falls to the legacy
random composition (2026-09-05 — an empty selection used to scatter the
legacy field). Random objects are never placed on dry cells (terrain
height above −0.05 m); explicit instances are placed verbatim — exempt
from the dry-cell rule, the `margin_m` keep-out, `min_separation_m`, the
cluster machinery and the resting-pose draw, wherever the author pinned
them.

Legacy keys: `density_per_hectare` (60), `composition` (weights over the
original 13 types — deliberately not extended, so old seeds reproduce),
`margin_m` (3), `min_separation_m` (1), `size_scale`, `burial_scale`,
`reflectivity_jitter` (0.15), `overrides.<type>`.

Per-class:

```yaml
objects:
  classes:
    drum_200l:
      enabled: true
      amount: {kind: count, value: 6}       # or {kind: density, value: 25}
      clusters: {n: 2, sigma_m: 3.0}        # omit for uniform
      size: {kind: gauss, mean: 1.0, sigma: 0.05}    # scale factor;
                                            # omit = per-spec ranges
      burial: {kind: uniform, lo: 0.0, hi: 0.2}      # omit = spec range
      material: metal                       # optional override
      orientation: {mode: path_relative, heading_deg: 90, jitter_deg: 10,
                    rest: {enabled: true, randomness: 0.6}}
                                            # omit = random, flat (see below)
      params: {fill_fraction: 0.2}          # class-specific extras
  explicit:                                 # verbatim instances (matched
    - {type: crate_plastic, x: 5.0, y: 5.0, yaw: 0.3}          # pairs,
    - {type: mesh_crate, x: -5.0, y: 5.0, fill_fraction: 0.35} # clusters)
  path_xy: [[0.0, 0.0], [20.0, 0.0]]        # survey track, world metres
                                            # (the builder fills this in)
  # path_heading_deg: 45.0                  # headless scalar fallback
```

Distributions are `uniform` or `gauss` only, by design. Classes place in
sorted name order — deterministic per seed.

**`explicit` entries and the RNG stream.** An explicit entry accepts
`type, x, y` (required) and `yaw` (**radians**, default 0), plus the
verbatim overrides `length, width, proud_height, burial, fill_fraction,
reflectivity, material`, the 3-D pose `roll_deg, pitch_deg, rest` and
`params` (the class's construction knobs, e.g. `wall_mm`, `flooded`).
*Only what the entry states is pinned*: every other field is drawn from
the shared placement stream, so an entry giving just `type, x, y` changes
size and burial whenever anything before it in the list changes. Pin all
of `worldgen.objects.PINNED_FIELDS` (what
`worldgen.objects.placed_to_explicit` writes) for an instance that must
stay put — that is what the World Builder's hand-placement screen does.

Two asymmetries worth knowing: `material` is assigned *after* the
reflectivity was computed from the class's own material, so give
`reflectivity` too when the material matters acoustically (the raster path
only — a facet object ignores reflectivity; `transmission` and the surface
roughness, which decides facet vs raster, do follow the pinned `material`
since 2026-10-21); and writing
`roll_deg`/`pitch_deg` at all sends the instance down the 3-D solid path
(`rest` stops being `canonical`), so omit both for an object that should
keep the flat 2.5-D stamp.

**Hand placement (World Builder Screen 5).** The builder writes
`explicit` from map clicks, and can additionally **pin** Screen 4's
random field so each scattered object becomes individually editable.
That lives in `BuilderState.objects` only, never in a world YAML:
`placed` (the pinned instances) and `placed_signature` (the
`seed`/`classes` fingerprint that produced them). While `placed` is
non-empty, `state_to_world_config` emits `classes: {}` and concatenates
`placed + explicit` into this `explicit` list — leaving the classes in
would place the whole field a second time. A pinned rebuild reproduces
the scattered objects to float noise (~1e-14 m), and a pinned state
rebuilds itself bit-identically.

**Orientation** (`orientation:`, per class, all keys optional):
`mode: random | fixed | path_relative`, `heading_deg` (default 0),
`jitter_deg` (default 0, gaussian σ). `random` is the default and the
historical behaviour — the yaw is the same uniform draw in the same RNG
position, so every existing seed is bit-identical. `fixed` sets the
object's long-axis heading in ENU degrees (0 = east, CCW positive).
`path_relative` measures `heading_deg` as an offset from the local survey
track — the heading of the nearest segment of `objects.path_xy`
(`[[x, y], …]` in world metres; the World Builder passes the source path
automatically and stores it in the config snapshot only when a class asks
for it), falling back to the scalar `objects.path_heading_deg`; 0 =
along-track, 90 = across-track. Both need one of the two, or generation
raises. **RNG note:** every mode consumes the same uniform variate, so
switching a class between modes never moves other classes; a `jitter_deg
> 0` adds one extra normal draw per object, which is the only place the
stream differs from `random` (objects later in the *same* class shift).
Unknown modes and unknown keys raise by name.

**3-D resting pose** (`orientation.rest`, per class, 2026-09-03):
`{enabled: false, randomness: 0.5}`. Off — the default — keeps the legacy
flat stamp (drums stand, pipes lie; every existing world bit-identical).
On, each object gets a physically **stable rest pose** of its primitive
(`worldgen/solids.py`: a box lands on one of its six faces with odds ∝
face area, a drum stands / lies / rarely inverts, a cone stands or lies
on its flank, a tyre lies flat or rarely on its tread, a bottle lies) and
is **settled** on the least-squares plane of the seabed under its footprint
(slope capped at 25°, so it leans with a bank) with its lowest point
exactly on that plane, then sunk by its burial fraction — never floating,
never over-sunk. The stamp is the z-buffer of the rotated solid (a lying
drum is diameter-high with the cylinder's footprint). `randomness` 0
keeps the canonical pose for every object (settled on the slope), 1 draws
every pose by its odds, in between that share of the objects falls over;
a settle tilt `N(0, 4°·randomness)` is added. The draw uses a
**per-object stream** seeded from the object's id and position, so
enabling it for one class moves no other object. Lines, blobs and
tangles (rope, chain, anchor, debris, coils, nets) have no rest poses
and ignore the block, and so does a class whose spec sets
`rest_allowed=False` (the driven `mooring_pile`), which keeps its upright
flat stamp. Explicit instances take `roll_deg` / `pitch_deg`
verbatim. The manifest carries `roll`, `pitch`, `rest` and `rest_z`
(additive defaults), `metadata.yaml` `roll_deg` / `pitch_deg` / `rest`,
and `world.sdf` places the same primitive at the settled pose.
The catalog holds **29 classes** (the original 13 plus the target-family additions incl. the
open-frame `bicycle`/`mesh_crate`/`shopping_trolley` with
`fill_fraction`, and the field-target `plastic_box`); see
`worldgen/objects.py` (`CATALOG`).

### walls (optional)

Reflecting boundaries (quays, pontoons). Absent — the default — is open
water.

| Key | Default | Effect |
|---|---|---|
| `name` | required | identifies the wall in ghost ground truth (`via: "wall:<name>"`) and in `world.sdf`; unique |
| `x0, y0, x1, y1` | required | the wall's plan segment |
| `kind` | `quay` | `quay`: a face from the **seabed** to `top_z` (a `top_z` below the waterline is a submerged sill); `pontoon`: a **floating** face spanning `[-draft_m, 0]` that the sonar looks under — no shadow, only rays meeting its band reflect. A line the boat drives over is a pontoon |
| `top_z` | 0.0 | world z of the top (0 = waterline) |
| `draft_m` | 0.4 | pontoon only: how deep its floats reach |
| `reflectivity` | 0.5 | energy fraction **per bounce**: concrete quay ≈ 0.65, fendered pontoon ≈ 0.3; mirror ghosts carry its square |
| `roughness_m` | 0.003 | rms face roughness: at 450 kHz a few millimetres turn most of the mirror into faint diffuse scatter (Rayleigh law, `sonar_model.md` §11) |
| `thickness` | 0.30 | Gazebo box thickness |
| `ctrl` | absent | **builder only** (`builder_state.yaml`): the through point of a curved wall — the point the wall passes through at its middle, dragged in the World Builder. Never reaches the world config |

**Merging and curves (2026-09-03).** Before a world is built, every
cluster of wall endpoints within **0.75 m** (`core.geometry.WALL_SNAP_M`)
is snapped to its centroid (union-find, idempotent), so hand-drawn
corners are water-tight for the depth interpolation barriers and the
sonar faces; the canvas also snaps to nearby endpoints while drawing or
dragging. A wall carrying `ctrl` is a quadratic Bézier through that point
and is **flattened into chords** named `<name>_s01 … _sNN` (chord-height
tolerance 0.10 m, `core.geometry.wall_segments`); a straight wall keeps
its name and count, so every existing world is byte-identical. The chords
share every property, chain exactly, reach the scene, the SDF (one box
each), the depth barriers, the manifest and the sonar alike, and
`metadata.yaml` marks each with `curve: <base name>`. `mirror_sources`
culls a chord whose nearest point is beyond the range window, so a long
curve costs only its chords in view.

Walls are acoustic reflectors and **solid** Gazebo boxes (visual +
collision — the boat cannot drive through a quay; a pontoon is a box
floating from its draft to its top). They are not stamped into the height
raster; the renderer treats them analytically — face echo, occlusion,
single-bounce band and mirror ghosts (`sonar_model.md` §11), all on by
default in the shipped sonar profiles and strict no-ops without walls.
The **world-limit borders are not walls**: collision-only, invisible, and
never acoustic.

### materials (optional, or `config/materials.yaml`)

Per material: `reflectivity`, `texture_amp`, `texture_cells`,
`micro_roughness_m`, `color`, `transmission`, `roughness_m`, and — since
2026-09-28 — the **acoustic constants** `density_kgm3`, `sound_speed_mps`,
`backing` (`solid` | `water` | `air`) and `wall_m`. Built-ins: seabed
`sand/mud/gravel/rocks/seagrass`; object
`rubber/pvc/plastic/metal/concrete/brickclay/rope/generic` plus the
target-family additions `glass/wood/eps/tinplate` and the field box's
`pp_shell`.

How an object sounds follows from these, never from a per-object gain
(NC #17, `sonar_model.md` §12): a material whose `roughness_m` is under
1.6 mm is **smooth** — on a solid primitive it scatters in the facet
layer, as coherent Kirchhoff patches scaled by the reflection coefficient
its impedance gives (`solid`: half-space; `water`: a flooded wall of
`wall_m`; `air`: `|R| = 1`) plus the incoherent residual; its
`reflectivity` is then not read. Rough materials keep the raster's
Lambert law with their `reflectivity`. A new material needs its physical
constants, with a source.

### Shipped world variants

`config/shallow_water_world.yaml` = the default at `base_depth: 2.5`
(depth 1.7–3.4 m vs 3.2–4.9 m): the detection-work config — shadow length
goes as `h·R/altitude`, measured ×1.50 median across 47 paired
observations. `config/enclosed_basin_world.yaml` = the default plus three
walls (pair with `config/enclosed_basin_sonar.yaml`). Neither knob
consumes RNG draws, so at the same seed each places the *same* objects at
the *same* poses as the default — bundles pair object-by-object, which is
what smoke sections [4] and [4d] exploit.

## 2. Sonar config

`acquisition:` mirrors the six real ROS parameters — see
`docs/topics.md §2`. World folders carry no frozen `sonar.yaml`: the
acquisition in force is the **resolved profile** (explicit
`sonar_config:=` > a legacy bundle's frozen copy > this package's
`config/default_sonar.yaml`), and `sss_sim_launch.py` passes the six
values to the node explicitly, each overridable per run exactly as on the
real system (`ros2 launch … gain_index:=-1 range_length_mm:=30000` — a 30 m
diagnostic run; every survey condition keeps the 20 m default).
`num_results` up to 1200 is fully supported end-to-end.

The shipped default profile is **20 m, gain −1 (device auto), 1200 bins,
pulse 0.00125** — the device's best range resolution and, since 2026-09-28,
exactly the real `sss_node.py`'s defaults; see `docs/topics.md §2`. A Gazebo run started from the Mission Control Station
passes none, so it gets this profile. Four more profiles ship, each the
default file with one idea changed:

| Profile | What differs | For |
|---|---|---|
| `config/default_sonar.yaml` | — | 20 m / 1200 bins: the field comparison acquisition (the model constants are the calibration anchor's) |
| `config/coverage_pass_sonar.yaml` | `range_length_mm: 30000` | a 30 m **diagnostic** profile (historical name; the project's survey range is 20 m per side for every condition since 2026-10-20, `project_synthesis.md` §8.5) |
| `config/shallow_beach_sonar.yaml` | `multipath_enabled` + `surface_mirror_enabled` | the 2026-09-07 beach regime (≈1.2 m altitude), where the bottom–surface–bottom ghost and the surface mirror land *inside* a shadow rather than beyond the swath |
| `config/fault_injection_sonar.yaml` | the four `fault_*` knobs + `dropped_ping_prob` | exercising the downstream paths a clean simulation never takes. **A test profile** — never a calibration or comparison run |

`config/enclosed_basin_sonar.yaml` is the fifth, older one (a basin
study: wall multipath, which `default_sonar.yaml` now enables too; the
surface mirror is **off** in it).

`model:` (no hardware equivalent):

| Group | Keys | Guidance |
|---|---|---|
| Mounting | `sensor_depth_m`, `mount_x_m`, `mount_y_abs_m`, `beam_tilt_deg` (20), `vertical_aperture_deg` (50, spec), `horizontal_aperture_deg` (0.5, spec) | match the physical bracket; tilt+aperture set the usable swath |
| Timing | `max_ping_rate_hz` (20, the device law measured on the corpus: 50 ms to 30 m, 2R/c + 3 ms beyond) | set 0 to disable the cap (experiment only) |
| Acoustics | `lambert_exponent` (1.0), `absorption_db_per_m` (0.10 @450 kHz), `spreading_exponent` (4.0), `tvg_compensation` (**0.30**), `specular_spreading_exponent` (**2.0**), `array_length_m` (**0.0**), `beam_sidelobe_floor` (0.004), `specular_strength` (**12**), `specular_width_deg` (8.0), `specular_looks` (25), `pulse_smearing` (true), `alongtrack_beam_lines` (5) | the range law was re-fitted on **two altitudes at once** on 2026-09-18 (beach 1.40 m + harbour 7.33 m): `tvg_compensation` 0 → 0.30, and the coherent component got its own spreading (an image source spreads once, not twice) so the first-bottom return follows the device's altitude law. Only the product `spreading_exponent · (1 − tvg_compensation)` is determined; raising the TVG flattens the image. Sidelobe/specular shape the FBR line — don't zero them if downstream bottom tracking must lock. `array_length_m` floors the along-track footprint at the aperture; 0 because the corpus says the device focuses its near field |
| Multipath | `multipath_enabled` (false), `multipath_gain` (0.12) | optional second-bottom-echo ghost, +altitude in slant range |
| Wall multipath | `wall_multipath_enabled` (true), `wall_multipath_gain` (1.0), `wall_diffuse_fraction` (0.04), `wall_corner_gain` (1.0), `surface_mirror_enabled` (false), `surface_reflectivity` (0.9), `ghost_beam_lines` (1) | mirror-source ghosts off the world's `walls:` at `reflectivity²` × the Rayleigh roughness law (+ the lumped diffuse share) and the single-bounce band just beyond a quay — `sonar_model.md` §10–11, anchored on the corpus quay. ×1.25 per ping-side with one wall in range, ×1.50 with three, ×1.00 all culled |
| Calibration | `calibration_db_offset` (**81.1**), `max_span_db` (90) | the only level constant — fit against the reported `max_pwr_db`, never a count histogram (NC #6). Re-fitted with the range law on 2026-09-18 (was 97.2, which was right at the harbour's altitude and 18 dB hot at the beach's). See `sonar_model.md` §6 |
| Noise | `speckle` (true), `speckle_looks` (1), `coherent_scatterers` (true), `scatterer_cell_m` (0.01), `scatterer_seed` (0), `filter_duration_percent` (0.0015, the receive filter: range envelope = pulse/Ts + this·N + 1 bins; the deprecated `range_filter_bins` > 0 restores the old fixed floor), `noise_floor` (**3.39e-09**, measured receiver floor), `watercolumn_noise` (**4.68e-06**, measured 1–3 m water column), `reverberation_db` (**18.0**, a shadow's floor under the lit seabed), `gain_drift_amp` (0 — augmentation-stage concern), `dropped_ping_prob` (0 — same) | zero everything for "clean physics" ablation images; `coherent_scatterers: false` restores the uncorrelated per-bin speckle. The two floor constants encode *levels* (−3.6 and +27.8 dB reported) and moved with `calibration_db_offset` on 2026-09-18 without changing them. `reverberation_db: 0` restores the pre-2026-09-18 white floor, i.e. shadows 30 dB too deep |
| Fault injection | `fault_gain_walk_pings` (0), `fault_counter_reset_pings` (0), `fault_counter_offset` (0), `fault_channel_255` (false) | **test knobs, not fidelity**: the downstream paths a clean simulation never takes (gain compensation, counter re-base, channel-255 side fallback, one-sided rows). `config/fault_injection_sonar.yaml` turns them on together |
| Wall echo | `wall_echo_enabled` (true), `wall_echo_strength` (30), `wall_echo_obliquity_floor` (0.1), `wall_face_diffuse` (0.5), `wall_pontoon_specular` (0.15) | direct face return (specular lobe + Lambert over the wet face, range law floored at the altitude) + occlusion by the face band, per wall `kind` — `sonar_model.md` §11; strict no-op without walls |
| Objects (NC #17) | `object_facets` (true), `facet_patch_m` (0.01), `sand_lambert_mu_db` (−12.0) | smooth solid objects scatter from their exact geometry (`sonar/facets.py`); `false` keeps their shadows and drops their returns (a diagnostic). The patch keeps the phase linear across it and the beam constant over it (valid from ~2 m). μ is sand's Lambert parameter at 450 kHz from the literature — the one constant tying an object's cross-section to the calibrated seabed; never fit it to an object |
| Sampling | `sample_step_m` (0.05, coarse upper bound only) | the renderer refines the step to 0.30 × the slant bin automatically (0.45 × with `coherent_scatterers: false`; floor 4 mm) |

## 2b. Sea state (current + waves)

`config/sea_states.yaml` (`format: blueboat_sea_state_presets/1`) is the
single source of truth for the presets (`current: none | weak | moderate |
strong | very_strong`, `waves: calm | rippled | smooth | slight | rough`,
each with `label`, `description`, `reference`, and for waves `gamma`,
`spread_s` and an `events: {rate_per_hour, hs_m, tp_s, duration_s,
from_spread_deg}` block of random wake / swell groups — `docs/sea_state_model.md`
§4.1b) and the two hulls the wave force uses; directions are chosen per
run. A keyframe or command may give explicit wave fields instead of a
preset (`waves: {hs_m, tp_s, gamma, from_deg, events: {...}}`) — what the
MCS "Custom" wave entry writes. A timeline is a
`config/sea_schedule_example.yaml`-shaped file
(`format: blueboat_sea_schedule/1`, `keyframes: [{t_s, current: {preset |
speed_mps…, from | from_deg}, waves: {…}}]`, `interpolation: linear |
hold`, `seed`) passed as `sea_schedule:=` or sent live on
`/sim/sea_state/command`. Physics, references and every knob:
`docs/sea_state_model.md`. The MCS reads the presets file from the
installed share directory and writes timelines under
`~/.config/blueboat_mcs/sea_states/`.

## 3. World-folder file formats

`~/worlds/<path_name>/<world_name>/` (immutable; every save takes a fresh
`_i` suffix):

- `scene.npz` + `scene_manifest.yaml` — the ground truth (rasters +
  manifest, `SceneModel.save`); `world.sdf` + `seabed.stl` — the Gazebo
  world written from the same scene (`write_world_sdf`).
- `metadata.yaml` (`format: blueboat_world_meta/1`) — **the cross-module
  contract**: `world_name`, `created`, `source_path{name,file,
  design_offset}` (world = design − offset; the offset is the spawn
  origin), `geo_anchor{lat0,lon0,theta_deg}` (the spawn's GPS),
  `spawn{x: 0, y: 0, lat, lon}`, `limits{local{x_min…y_max},
  corners_gps[SW,SE,NE,NW]}`, `walls[{name, local[[x0,y0],[x1,y1]],
  gps[[lat,lon]×2], top_z, reflectivity, thickness, kind, draft_m,
  roughness_m, curve}]` (`curve` = the builder wall a curved wall's chord
  came from), `objects[{object_id, type, x, y, lat, lon, yaw, yaw_deg,
  roll_deg, pitch_deg, rest, orientation{mode, heading_deg, jitter_deg},
  length, width,
  proud_height, burial, fill_fraction, material}]` (`orientation` is the
  class's normalised rule; explicit instances — hand-placed or pinned —
  report `fixed` at their pinned heading, which is why a fully pinned
  world reports `fixed` for every object),
  `depth{mode, preset|file, min_m, max_m, mean_m[, svlog{...},
  svlog_quality{pings, detected_fraction, altitude_fraction,
  altitude_p5/p50/p95_m, soundings, cells, cell_m, extent_m, crossover,
  clock_mode, gps_fixes, counter_offset, sensor_depth_m,
  datum_offset_m}]}`, `seabed_material`,
  `seed`. GPS values invert to world positions through the anchor with
  the shared equirectangular pair.
- `builder_state.yaml` (`format: blueboat_world_builder_state/1`) — the
  complete wizard state incl. a cached copy of the path's points (Edit
  survives the MCS file being renamed or deleted) and, since 2026-09-09,
  `depth.layers` — the seabed editor's layer stack above the source
  (`seabed_model.md` §7; absent or empty = the legacy state). GUI-internal:
  downstream modules read `metadata.yaml`, never this.
- `depth_baked.tif`, `material_map.npz` (layered worlds only) — the
  baked layer stack the world YAML references (`depth.file`,
  `terrain.materials.map`); see "Layered worlds" above.
- `depth.npz` — `height` (the terrain-only raster, before object stamps —
  the Screen-3 design surface), `origin_x`, `origin_y`, `resolution`.
- `scene_manifest.yaml` (above) — version 1, with the additive `geo:` block
  (`anchor` + `convention: equirectangular_wgs84_r6378137`).
- `preview_objects.png` — the Screen-5 objects map, rendered by
  `export_maps.render_objects_preview` (the GUI shows the same file).
- `depth_source.csv`/`.tif`/`.svlog` — a copy of the Screen-3 input
  when `mode: file` or `mode: svlog` (a recording can be tens of MB —
  the price of a self-contained, immutable folder).
- `depth_samples.npz` (`mode: svlog`) — the soundings (`east`, `north`,
  `depth`, `t_ms`, `quality`, `lat`, `lon`) and the fitted cells
  (`cell_east`, `cell_north`, `cell_depth`, `cell_count`).
- `reflectivity_survey.npz` (`mode: svlog` with the `reflectivity_survey`
  layer or the `backscatter_layer` flag) — the audit record of the
  inverted backscatter: `mu_db` (Lambert's coefficient per 1 m cell in
  the renderer's units), `count`, `spread_db`, the lattice origin and
  cell, `reference`, `gain_index_ref`, `angle_norm`.
- `source_path_world.yaml` — the source path in world coordinates
  (`blueboat_trajectory/1` points, first waypoint = (0, 0) = the spawn,
  no `geo_anchor`); a drop-in `trajectory_file` for the dev launch.
