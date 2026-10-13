# World Builder (`builder/`)

Moved verbatim from `BlueBoat-SSS-Sim/.claude/CLAUDE.md` §3.3 on 2026-09-28 so it
loads only when working in this folder. The module file keeps a summary and the
two rules that hold everywhere; section and NC references (§6, NC #10, NC #13 …)
point into that module file.

## `world_builder` (GUI, PySide6)

Start dialog: **Create Gazebo World** / **Edit World** (lists
`~/worlds/*/*/`, reloads `builder_state.yaml`, skips Screen 1, all
pre-filled).

**Display-adaptive window (2026-09-16):** opens centred in the screen's
*available* area (never under a panel), freely resizable. Tool rows wrap
(`builder/layout.ToolBar`, ≤ three rows, then scroll) instead of imposing a
minimum (the depth editor's row once forced 2335 px, wider than any screen:
unresizable); map panels in splitters (drag narrow or shut); previews
rescale (`ScaledImage`); each page rides in a `PageScroll` that compresses to
the window first and scrolls only below its own minimum (every screen fits
1000×600 — smoke `[g8]`). Screens:

1. **GPS path** — the MCS launch dialog's list, GPS-anchored entries only
   (`builder/paths.py`: glob `~/.config/blueboat_mcs/trajectories/*.yaml`
   minus `*.meta.yaml`, keep those with `geo_anchor` — the `read_geo_anchor`
   predicate, reimplemented locally per CM-3).
2. **World limits & walls** — Esri satellite tiles (the MCS/GCS source;
   cache `~/.cache/blueboat_sss_sim/tiles/`). One dragged axis-aligned
   rectangle = the limits (grid extent + four invisible collision-only
   border boxes, **never acoustic**). Clicked segments = `Wall`s of a
   **kind**: `quay` (face from the seabed; blocks sonar and boat) or
   `pontoon` (floating, `draft_m`, sonar looks under it — the structure a
   boat can cross); each with reflectivity (per bounce, 0.65 concrete),
   `roughness_m` (Rayleigh coherence at 450 kHz) and a Gazebo box **with
   collision** (`docs/sonar_model.md` §11; re-anchored 2026-09-03 on the
   corpus quay: bright face, dark zone behind, faint far ghosts). Hover help
   on every wall-dialog row. Endpoints within 0.75 m of another wall's end
   **snap** while drawing/dragging and **merge** at build (union-find,
   `core.geometry.merge_endpoints`). A selected wall's hollow midpoint handle
   **bends** it (quadratic Bézier through that point, `ctrl` in the builder
   state only, flattened to `<name>_s01…` chords for the engine by
   `core.geometry.wall_segments`); "Straighten wall" removes the bend.
3. **Depth & seabed.**
   - *Sources:* a preset (`harbour_basin`, `river_channel`, `beach_profile`
     (Dean x^(2/3), optional berm), `canal`, `flat`, `sloped_plane`,
     `dredged_channel`, `marina`, `rocky_outcrop`, `sandbar_ripples`; forms
     from `DEPTH_PRESETS`; **depth ≤ 0 is dry land**, dry-margin presets
     accept negative minima since 2026-09-05); a file (CSV lat,lon,depth;
     geographic-CRS GeoTIFF via Pillow, projected CRS rejected with a
     message; linear inside the data, nearest-value flood fill outside that
     **never crosses a wall**); or a **sonar recording** (`.svlog`,
     2026-09-05).
   - *Recording mode:* first-bottom-return soundings at the recording's GPS
     fixes (`analysis/bathymetry.py` — the processor's detector
     re-expressed, both sides fused robustly, a frozen autopilot clock paired
     by file order) → smoothing thin-plate spline, edge slope continued and
     damped beyond the track over `trend_length_m`, residual-scaled
     undulation between the lines (`worldgen/depth/svlog.py` + `smooth.py`).
     Track and soundings drawn on the map; a quality line: pings /
     bottom-return fraction / altitude / cells / crossovers / clock regime
     **and coverage** (2026-09-06: samples inside the limits, share of the
     world within `trend_length_m` of a sounding, warning below 35 % —
     `worldgen/depth/coverage.py`; a source nowhere inside the limits is
     refused naming both places, distance and compass point; a near miss
     says redraw the rectangle, not change the file). **2026-09-16:** relief
     between lines = `synthetic` (the undulation), `shading` (relief the
     recording's own brightness measures across every swath,
     `analysis/shading.py`, fused over the spline, amplitude calibrated on
     the tie lines' FBR when `relief_gain` is 0 — the new-world default) or
     `none`; one tick adds a **`reflectivity_survey` layer** (backscatter
     inverted through the calibrated model to renderer reflectivity,
     `analysis/backscatter.py`: continuous gain about the survey median +
     mud / gravel / rock classes by the literature contrasts; an ordinary
     layer row — masked, reordered, deleted like any other; the disc-splat
     "hint" it replaces could not be removed at all).
     `docs/seabed_model.md` §7.1; how to sail for both:
     `docs/svlog_depth_survey.md`.
   - *Texture, brushes, previews:* full texture schema
     (`TERRAIN_TEXTURE_SCHEMA`: dunes, roughness, material layout
     uniform/patches/painted). Map-canvas overlays are south-row-first
     rasters placed at their origin, no vertical flip (`set_overlay`,
     2026-09-16), like brush strokes, walls and soundings; preview pictures
     flip once in `imaging.raster_to_qimage`; the two are pinned to agree at
     the same design point; a hover readout under the map and both pictures
     names position, GPS, depth, material; the compositor reads a layer's NaN
     as "no change", never `NaN·0`. **Depth/material brushes**
     (`DepthState.paint` / `material_paint`): a depth stroke is exact inside
     its radius and blends into the base over its own short **feather** (C1
     distance-field smoothstep; union rule, overlaps leave no ridge —
     2026-09-05, was a linear triangulation to two radii); reproduced
     headlessly from `depth.paint` / `terrain.materials.paint`. **Live
     previews** (height + reflectivity, 300 ms debounce,
     `DepthPreviewWorker`; the "sonar pass" strip was removed 2026-09-03 —
     texture contrast on one arbitrary line misled as a survey preview).
   - *Layer editor (2026-09-09;* `builder/layers/`, engine
     `worldgen/depth/{layers,masks,geometry,compose,bake,recipes}.py`,
     literature `docs/seabed_model.md`*):* source, base brush strokes and
     texture form are the three fixed bottom rows of a **layer stack**; above,
     any number of layers — gradients (linear / smoothstep / power / damped /
     Dean from a drawn shoreline), landforms (dredged basin, channel along a
     drawn centreline in (s, n) coordinates with the thalweg on the outer
     bank, scour hole, mound, bar and rips, step, siltation from an
     entrance), relief (ripples, dunes, furrows, roughness, outcrops, pits),
     adjustments, material region / gradient / patches / belt, reflectivity
     gain, brush layers with an op — each with map-drawn geometry (click
     vertices + right-click, or drag; vertex handles after), a **mask**
     (rect, ellipse, polygon, band, distance to walls, depth range, gradient,
     mask brush, survey coverage; C1 feather never crossing a wall), opacity,
     blend (normal / add / deepen / shoal) and its own RNG stream. Recipes
     (`config/seabed_recipes.yaml`) instantiate literature-sized stacks.
     Whole-document undo/redo, composite map overlay (depth / material /
     reflectivity), section tool. The stack lives in `DepthState.layers`
     (design frame), **baked at build** (`worldgen/depth/bake.py`) into
     `depth_baked.tif` + `material_map.npz`, so the world YAML stays a plain
     file-mode world (decision 2026-09-09: no `mode: layers`; the one
     additive key is `terrain.materials.map`). No layers = serialises and
     builds exactly as before. Every form row (texture, depth presets, layer
     kinds, walls, objects) has a one-sentence hover help, the schema tuple's
     7th element.
4. **Objects** — per `CATALOG` class: count or per-hectare density, optional
   Gaussian clustering, size/burial distributions (uniform|gauss), yaw
   orientation, **Resting pose** ("let them fall / roll (3-D)" + randomness:
   `orientation.rest`, 2026-09-03 — a physically stable pose of the
   primitive, settled on the seabed slope, `worldgen/solids.py`), per-class
   `params_schema` extras; a global seed. **Info…** per row (regular size,
   typical shape, rest poses with odds, top-down footprint of the typical
   instance — `worldgen/objects.py`: `typical_instance`,
   `shape_description`, `rest_poses_text`, `footprint_profile`). Screen 4 =
   *what* and *how much*; Screen 5 = *where*.
5. **Place objects by hand** (2026-09-14) — whole catalog left (every class,
   not only Screen-4 ticks); right: tiles, limits, walls, GPS path, depth
   composite under **to-scale, yaw-turned footprints of every object the
   world will contain**. Arm a class, set one click's drop (count + spread
   σ, plus Screen 4's knobs — size/burial distributions, orientation, 3-D
   rest, class `params`, material; prefilled from that class's Screen-4
   row), left-click; stays armed until another is picked. Select → move
   (centre handle) / turn (nose handle); double-click edits **every**
   verbatim value; Delete; full undo/redo. A dry-cell click warns, never
   refuses (NC #13 exempts authored instances).
   **Pinning:** Screen-4 objects are *generated* from classes + seed, so none
   can move while they are. Editing one asks once, then **pins** the whole
   field (`placed_to_explicit`, mechanics §6) — every instance written with
   the values it was drawn with. Faithful: same objects to float noise
   (smoke `[13]`: max |dxy| 1.4e-14 m, max |dh| 3.5e-13 m); a pinned state
   rebuilds bit-identically. If Screen 4 changes after the pin, re-entering
   offers re-scatter or keep.
   **Crash fix + review (2026-09-16):** clicking an unselected object or
   dragging a scattered one's handle tore the scene down *inside* Qt's mouse
   dispatch (`_sync_items` dropped every `QGraphicsItem`, the executing one
   included) — a use-after-free segfault that lost the world. Now: a counted
   `syncing` guard; structural rebuilds requested mid-dispatch deferred to
   the event loop (`_dispatching`, `_queue_rebuild`); object items *diffed*
   in place; removed items parked in a graveyard for one turn; dead handles
   marked; drag end reported (`object_drag_finished`), where the page pushes
   one undo step per gesture or asks the pin question — never inside the
   drag. Also: a running `PlacementWorker` is never replaced (Qt aborts on a
   `QThread` destroyed mid-run); objects placed meanwhile survive its
   result; scattered footprints are selectable (the documented "drag one to
   pin the field" route); `objects.pinned` keeps the classes silent after
   every pinned object is deleted; headings wrap to ±180° (no clamp); Ctrl
   snaps a turn to 5°; heading readout while turning; nose handle ≥ 14 px
   from the centre one; Undo / Redo / Info follow their state; buttons
   follow the map's mode. Rule for every page: **never rebuild the scene or
   open a modal from a slot a canvas item event emitted** (`gui_smoke` drives real
   `QTest` sequences under `faulthandler`; they print under `[g5]`, after
   the wizard shells).
   **Edit-mode fix (2026-09-28):** the objects a state already holds
   (`placed` + `explicit`: Edit mode, a resumed autosave, a return here) are
   shown the moment the screen opens and are the page's own rows from then
   on; the `PlacementWorker` — which re-runs the whole build, tens of seconds
   on a layered or recording world (a 28 s bake measured) — only adds the
   scattered field and the depth overlay, works on a **copy** of the state,
   and cleans up its temporary bake. Before, the list stayed empty until the
   worker returned, and a Back / Next meanwhile committed that empty list:
   the edited world lost every object. Every footprint also carries a
   fixed-size **6 px ring** (a passive child item, `OBJ_MARK_PX`), since a
   0.7 m box is ~2 px wide at the zoom that fits a world. `[g7]` covers it.
6. **Preview & save** — objects + depth maps (same PIL code the folder
   stores), name → generate. Collisions get a silent `_1`, `_2`… suffix,
   **always**, Edit re-saves included. Worlds are immutable.
