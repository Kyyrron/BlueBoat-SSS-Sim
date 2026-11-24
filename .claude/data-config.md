# BlueBoat-SSS-Sim — world folders, data formats and configuration

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## 5. Data: world folders, formats, what not to overwrite

`~/worlds/<path_name>/<world_name>/`, built by
`worldgen/world_folder.build_world_folder`. Every file and its format:
`docs/configuration_guide.md` §3. What the code does not tell you:

- `scene.npz` + `scene_manifest.yaml` are the **ground truth**.
  `SceneModel.stale_rasters()` names the object rasters a world built before
  the current model lacks — logged by every loader; the cure is a rebuild
  (Edit), never a patch.
- `metadata.yaml` is **the cross-module contract** (`blueboat_world_meta/1`:
  anchor, spawn, limits, walls and every object in local + GPS). Downstream
  reads it, **never** `builder_state.yaml` (GUI-internal, may churn).
- `ground_truth_rasters.npz` (since 2026-11-15) holds `seabed_class`, `in_domain` and `survey_area` on the
  world grid; `metadata.yaml` gained the additive fields of schema version 2 (per object: role, family, state,
  `nominal_target`, `effectively_buried`, height over the seabed, face and axis directions; per world: depth
  domain, seabed classes, content digest). `docs/configuration_guide.md` §3.1.
- `depth.npz` is the **terrain-only** height raster (pre-object-stamp). The Gazebo
  visuals (`world.sdf` objects and walls, `seabed.stl`) stand on it since 2026-11-02
  (`write_world_sdf(terrain_height=...)`); before, they stood on the stamped raster, so
  every object floated on its own top over a bump (visual only, the acoustic scene was
  right). A world built earlier keeps that picture until re-saved (Edit -> new `_i`).
- `scene_manifest.yaml` stays **version 1**: the `geo:` block is additive;
  `SceneModel.load` rejects any other version.
- RNG determinism: a config **with** a `depth:` section splits the seed into
  two streams (`[seed, 0]` terrain, `[seed, 1]` objects), so editing the
  bathymetry never moves the objects. A legacy config without `depth:` keeps
  the single historical stream, bit-identical to every existing bundle.
- Rasters are `(ny, nx)`, origin at the min corner; `export_scene_maps` flips
  vertically for north-up images.

---

## 6. Configuration

World YAML schema: `docs/configuration_guide.md` §1; sonar config (every
`acquisition:` / `model:` knob): `docs/configuration_guide.md` §2, the
physics behind them `docs/sonar_model.md`; sea state:
`docs/sea_state_model.md`. **Unknown keys raise by name** in the sonar
sections, `terrain:`, `walls:`, `depth:` (presets, layers, masks, svlog,
paint) and `orientation:`; `world:`, `objects:` (top level, classes,
explicit entries), `materials:` and extra `geo:` keys ignore them —
`configuration_guide.md` lists both.
The traps:

- `geo:` `theta_deg != 0` is rejected (the world frame is the anchor's ENU
  frame by definition).
- With `depth:`, `terrain:` contributes only texture; without it, legacy
  behaviour is bit-identical.
- **An `objects.explicit` entry pins only what it states**: every other field
  is drawn from the shared RNG stream, so an entry of just `type,x,y` moves
  when anything before it changes. Screen 5 therefore writes all of
  `worldgen.objects.PINNED_FIELDS` + `material`, and
  `roll_deg`/`pitch_deg`/`rest` **only** for an object that actually rested
  (any tilt sends the instance down the 3-D solid path). `transmission` and
  `roughness_m` follow a pinned `material` (since 2026-10-21); `object_id`
  is positional and seeds rope/chain/debris/coil/lattice shapes, so deleting
  or reordering a pinned object re-draws those shapes of later objects.
- `BuilderState.objects.placed` (never in a world YAML) = Screen 4's random
  field pinned into verbatim instances; while non-empty,
  `state_to_world_config` emits `classes: {}` — keeping the classes would
  place the field twice. `placed_signature` detects a later Screen-4 change.
- Every material carries physical acoustic constants (`_ACOUSTIC` in
  `worldgen/materials.py`), never a brightness (NC #17); `transmission` is 0
  for all but `pp_shell`. `config/materials_literature.yaml` is an
  **override**, not the default, so existing worlds do not move.
- `DEFAULT_COMPOSITION` (the legacy random field of a world YAML without `objects.classes`) was
  reduced on 2026-11-15 to `tire_car`, `concrete_block`, `chain`, `debris` (the classes of it that
  survived Stage 6; the render fixture was re-baselined once for it). Otherwise it is not
  reshuffled — new classes enter via
  `objects.classes`, never by reshuffling legacy seeds.
- `wall_multipath_enabled: true` by default is a strict no-op without walls.

---
