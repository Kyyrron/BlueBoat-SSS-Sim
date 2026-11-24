# Ray-cast sensor — Phase 1 report: scene and mesh export, the CPU oracle

2026-11-02. Plan and gate: `raycast_plan.md` §Phase 1. Tolerances: `raycast_design.md` §7. Log of
the gate run: `~/ros2_ws/sim_runs/raycast/notes/rt_full_phase1.log`. Command (from the package
source root): `python3 -m test.raycast_test` (~12 min, every world).

## What was built

* `sonar/raycast/scene_mesh.py` derives the triangle scene from a `SceneModel` in memory and writes
  nothing:
  - **seabed sheet** = `height − facet_bump`, two triangles per raster quad, vertices at cell centres.
    Rough objects stay in the sheet exactly as the raster carries them, tagged with their object id;
  - **smooth solids** (`is_facet_object`): watertight meshes per catalog primitive (`solid_mesh`:
    box, cylinder, capsule, cone, annulus; chord sagitta ≤ 0.2 mm), placed by `facet_transform`, which
    mirrors `sonar.facets.object_patches`;
  - **walls**: the Gazebo box of `worldgen.sdf_writer._wall_sdf`;
  - **apron**: the scene's `outside` rule beyond the grid;
  - **sea surface**: the analytic plane z = 0, intersected exactly by `SceneOracle`.
  Per triangle: kind, object id, wall index, material, dry flag. `owner_raster` assigns rough-object
  cells from the world's terrain-only `depth.npz`.
* `sonar/raycast/bvh_cpu.py`, the CPU reference oracle:
  - a linear (Morton-ordered) BVH over a complete binary tree, built vectorised;
  - breadth-first vectorised traversal, with the leaf tests in bounded batches;
  - the **watertight** ray–triangle test of Woop, Benthin & Wald (JCGT 2013); Möller–Trumbore is
    kept only as a cross-check;
  - closest-hit and segment-occlusion queries, plus a brute-force reference.
* `test/raycast_test.py` section R1.

## Gate results (every world under `~/worlds/` + a synthetic world)

Worlds: the 6 pool worlds (`pool_{empty,1w,2w}` and their `_1` rebuilds), `plage_bathymetry2/
test_dataset_creation` (beach, 33 walls, the wavy-wall session's world candidate),
`Shiraishijima_exp` (harbour, 10 walls), `arob2027/{e2_targets_1p5m, flat_1p5m_large}`,
`sea_validation/{flat_1p5m, flat_5p0m}`. The synthetic world holds two of every catalog class, rested
at random on a 4 % slope (58 objects, 44 rested). `plage_empty` has no `scene_manifest.yaml` and is
not a world folder.

**Result: 131 passed + 2 re-run, 0 failed, 8 skipped** (no walls or no smooth objects in that world).

| Gate | Result |
|---|---|
| [M0] re-stamping `depth.npz` reproduces `height` and `facet_bump` | 0.0 difference on every world (outside the rested objects' windows, see below) |
| [M0] rough-object ownership: every raised cell owned, no other cell | 0 orphans, 0 strays everywhere (synthetic 2719 raised cells; Shiraishi 1283) |
| [M1] seabed vertices = `height − facet_bump` at every cell centre | exact. The triangle split departs from bilinear inside a quad by up to 0.66 m at stamped cliffs (99.9th percentile ≤ 55 mm) — a representation difference from the geometric renderer, reported |
| [M2] smooth-solid mesh vs `facets.object_patches` patch centres | worst **0.235 mm** (gate 1 mm) over 1.9 M patches, 145 objects |
| [M3] wall boxes vs manifest and `metadata.yaml` | exact (`plage` 33 walls, Shiraishi 10, pool 4) |
| [M4] only seabed / object / wall / apron triangles; dry flags exact | yes |
| [M5] solid volumes vs closed forms | all within 1 % (after tightening the chord tolerance from 0.5 to 0.2 mm: the capsule was −1.3 %) |
| BVH = brute force (every ray × every triangle) | **10⁵ / 10⁵ rays per world** over 5 random 2.5 m crops (each crop holds every smooth object and wall), identical triangle ids, max \|Δt\| 0 |
| full-scene closest hits | on their triangle to ≤ 2e-12 m; no closer occluder (segment test) |
| analytic surface | upward rays meet z = 0 at exactly the expected t unless geometry is closer |
| watertight = Möller–Trumbore away from edges | 2972 hits of 2·10⁵ random pairs, max difference 5.3e-15 |
| no leak through closed solids | **0 of 22 081** transversal probes through every vertex and edge midpoint; BVH = brute force on all |
| nothing written into any world folder | yes (mtimes of every file) |
| smoke test | 493 / 493 (system python), unchanged |

CPU reference throughput: 8 000–43 000 closest-hit rays/s on 1–3 M triangles, single-threaded numpy.
This is a reference, not the renderer's engine (Phase 4).

## What failed on the way, and the fixes (each found by a gate)

1. **Möller–Trumbore leaks on shared edges.** 1 of 7572 axis-aligned probe rays passed through a
   cylinder between two triangles. The fix is the watertight test. This matters because the
   renderer's scatterer and diffraction rays are structured (lattices), so edge leaks would be
   systematic, not random.
2. **Empty BVH slots were "hit" by every ray.** Their inverted boxes give an infinite slab interval.
   This exploded memory (OOM kill) without changing any answer. Empty boxes are now rejected.
3. **A vertex on a flat leaf box was rounded out of the slab test** (1 of 5186 probes). Boxes are now
   padded by 1e-9 relative; padding only adds candidates.
4. **The capsule meshed degenerate pole rings**; the poles are now fan centres.
5. Test artefacts, corrected:
   - the leak probe started outside sharp rims;
   - the M2 probe started under the floor for bottom patches;
   - the surface probe started under land or walls.

## Findings about the existing code (not changed)

* **Re-stamping a saved rested object on a slope is not idempotent.** `objects._rasterize_solid`
  composes the bank tilt into the stored `roll`/`pitch` (it writes the settled pose back), so a
  second stamp tilts it twice. Measured on the synthetic world: up to 0.89 m height difference, over
  2677 cells. Consequences for this sensor:
  - the stored raster and the stored poses are the source (as `facets.object_patches` already
    assumes);
  - no tool re-stamps saved objects;
  - M0 compares outside the rested windows only;
  - tests stamp deep copies (a stamp mutates its objects).
  No saved world is affected; worlds store the settled poses together with the raster they produced.
  Worth a line in `TODO.md` if any future tool needs to re-stamp.
* The ray-cast walls are the Gazebo boxes, so their reflecting face sits `thickness/2` (0.15 m by
  default) in front of the geometric renderer's zero-thickness line. It goes into the Phase-8
  comparison as a known geometric difference.
* NC #13: the ray-cast sensor silences dry triangles (every vertex at or above z = 0). Awash terrain
  between the transducer depth and the surface is water-covered and is not silenced. That is a
  deliberate, physical difference from the geometric renderer's "above the transducer" sampling rule.

## Gate status

**Phase 1 gate met** (2026-11-02). Phase 2 starts with the direct path (`raycast_plan.md`).
