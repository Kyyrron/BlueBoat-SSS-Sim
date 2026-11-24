# Independent review and implementation record — ray-cast simulator (2026-11-11)

**Status:** review complete; the owner chose **option A** (continue and improve the custom ray-cast
simulator) on 2026-11-11. Implemented on 2026-11-11: WP0 (provenance), WP3 (bottom line), WP2
(ground truth), WP4 (field-domain floor), WP6 (Gazebo heights), WP1a (objects as their own meshes).
Open: WP1b (a pallet as boards; the diffuse share of smooth solids), WP5 (validation world). §9 and §10 are filled package by package.
Companion: `holoocean_reference.md`. The legacy `GeometricRenderer` is not a candidate and is used
nowhere here as a benchmark.

Evidence classes used throughout:

| Mark | Meaning |
|---|---|
| **V** | verified in the current source or configuration (file:line given) |
| **M** | measured in this review, reproducibly (tool and data named) |
| **H** | historical result, possibly superseded |
| **D** | claim from documentation or literature |
| **J** | engineering judgement or hypothesis |

## 1. Purpose, and what the simulator must deliver

The thesis tests aspect-aware adaptive replanning of a side-scan survey from a surface vehicle against a
seriously tuned two-pass orthogonal survey, as a model-conditional simulated comparison
(`project_synthesis.md` §2, §4; submission early January 2027). The simulator is a means. Judged by that
purpose it must deliver, in this order:

1. labelled ground truth for a simulation-trained detector (Perception's dataset mode);
2. object highlights whose dependence on aspect is defensible, and shadows that follow object height and
   altitude (the thesis' mechanism);
3. a bottom line the real processor tracks (it georeferences everything downstream);
4. tens of closed-loop Gazebo runs at about real time;
5. the identical ROS 2 interface as the real driver (CM-1 / CM-2);
6. traceable runs.

Harbour is the primary domain; shallow water is a declared secondary limitation.

## 2. The system as built (V)

```
World Builder (builder/, PySide6)  --BuilderState-->  worldgen.world_folder.build_world_folder
  in : MCS GPS-anchored path, depth source (soundings / presets), walls, objects
  out: ~/worlds/<path>/<world>/
        world.sdf + seabed.stl      -> Gazebo (visuals; walls and limits collide)
        scene.npz + scene_manifest  -> acoustic scene (rasters + every PlacedObject)
        depth.npz                   -> terrain before any object stamp
        metadata.yaml               -> cross-module contract (anchor, limits, objects in GPS)

Gazebo Harmonic: blueboat_description (16.01 kg, Hydrodynamics + Thruster plugins)
  <- sea_state_node: wrench per physics step (current, waves, catamaran hydrostatics)
  -> /model/blueboat/pose (100 Hz) -> /sim/sea_state/boat_pose ;  /blueboat/odom (20 Hz)

sss_sim_node (ROS 2, Python; not a Gazebo plugin)
  pose -> predict to ping time -> waterline datum -> lever arm [-0.345, +-0.285, -0.22] m
  -> RaycastRenderer (scene mesh built in memory from scene.npz + manifest)
  -> shared noise stage -> PingEncoder
  -> /side_scan_sonar/{port,starboard}/{profile,raw}   (BEST_EFFORT, KEEP_LAST 10)
  -> /side_scan_sonar/ground_truth/{contacts,sensor}   (JSON, simulation only)

sss_processor_node (BlueBoat-SSS): bottom tracking, slant correction, .svlog
GCS: waterfall, mosaic, seabed images (256 rows x native bins)
Perception: fixed_gray -> detector -> detections + image records; dataset mode labels from ground truth
Belief -> Replanner -> Control (/blueboat/update_path)
```

Representations that exist more than once, and can therefore disagree (V, §6):
- object geometry: Gazebo primitive, raster stamp, ray-cast mesh;
- "ground" under an object: terrain-only `depth.npz` (Gazebo), stamped raster (stamp), `height −
  facet_bump` (ray-cast);
- sound speed: true c for echo delays, the device's 1500 m/s for range bins (ray-cast, as the real
  device), true metres in Perception's label geometry;
- boat mass and damping: `blueboat_description` (Gazebo) and the sea-state hull model.

## 3. Chronology of material changes

| Date | Change | Evidence |
|---|---|---|
| 2026-09-18 → 11-02 | Geometric renderer calibrated in four passes (legacy) | `calibration/CLAUDE.md` |
| 11-02 → 11-04 | Ray-cast built: mesh + BVH, CPU reference, small-slope seabed law, multipath, diffraction, GPU path | `raycast_phase1/2/4.md`; commits `07c73dd`, `86f8e29` |
| 11-05 / 11-06 | "Stage 2": pure physics fails its pre-registered gate — the first-return level falls about 11 dB too fast with altitude for every admissible beam and seabed (H, unresolved in physics) | `raycast_assessment.md` §4 |
| 11-07 | Harbour-first hybrid: physical core + an empirical near-nadir table (v2). Both acquisition groups EXCLUDED by the rule written beforehand; kept as a declared limitation. Live node and slow-down built | `raycast_assessment.md` §13 |
| 11-09 | Gazebo held while a ping renders; `live` variant | `.claude/raycast-live.md` |
| 11-10 | `fast` variant; harbour profiles = v2 core + table + 18 dB shadow floor; same-day fix (smoothed floor reference, measured near range at 1200 bins) | `raycast_assessment.md` §14 |
| 11-10 08:30 | last `colcon build` | `~/ros2_ws/build/*/colcon_build.rc` |
| 11-10 08:56–09:00 | source changed after that build (`floor.py`, `config.py`, `correction.py`, `renderer.py`, `sss_sim_node.py`, both harbour profiles) | `diff -rq` source vs install |
| 11-11 07:57 | latest session `2026_11_11-07_57_42` | ROS logs |
| 11-11 | this review; decision A | this document |

All work since commit `86f8e29` (2026-11-04) — 47 modified and 22 untracked files — is uncommitted.

## 4. The latest session (M)

Session `~/ros2_ws/data/SSS_data/sessions/2026_11_11-07_57_42`: world
`~/worlds/Shiraishijima-test-harbour/shiraishijilma-test`, ray-cast, system gain 126.9 dB (a harbour
profile), 20 m / 1200 bins, `harbour_chop`, requested ×2.1 with the hold (ROS logs
`~/.ros/log/2026-11-11-07-57-32-*`).

**It did not run the current source.** The install is a copy built on 11-10 at 08:30; its `floor.py`
and harbour profiles are the first harbour patch, without the measured near range and the smoothed floor
reference. The data agree: water column 27.8 dB (the fix gives 17 dB), horizontal lines in shadows. No
session file records renderer, profile, seed, world or build; these were recovered from ROS logs.

Measured with `analysis/picture_compare.py` (the tool of the 11-10 report) and the world's own depth:

| | real `toOptimize` | ray-cast 11-10 (provisional core) | ray-cast 11-11 |
|---|---|---|---|
| detector-input pixels saturated | 1.3–1.6 % | 71–73 % | 9 % |
| speckle fine contrast | 5.3–5.4 dB | 5.08 dB | 4.7 dB |
| dark speckle tail (1st percentile re mean) | −20.5 dB | −20.0 dB | −17.0 dB |
| track / range lag-1 correlation | 0.67 / 0.49 | 0.74 / 0.51 | 0.70 / 0.53 |
| anchor-bump shadow re seabed in front | −34 dB, textured | −41…−65 dB, empty | −30…−34 dB, textured |
| bottom-tracker jump, p50 / p90 | 0.16 / 0.88 m | 0.18 / 0.53 m | 0.17 / 1.75 m |
| tracked depth − world depth | — | — | median about +1 m; ≤ 16 % within 0.3 m |
| processor rows with a provisional altitude | — | — | 2076 of 3365 |
| wall time for 168 s of simulated time | — | — | 183 s |

The level moved from grossly hot to about 4 dB bright in the detector's input (not altitude-matched),
and shadows became textured. The texture became smoother; §5 gives the cause. The six placed objects
show no measurable signature (highlight within about 1 dB of the seabed for five; the sixth sits at the
nadir edge); §6 gives the cause.

Against `seabed_images_fullHarbourCleanExample` (95.7 m / 1200 bins, 8 cm bins) the comparison is
structural only (J, by inspection): the real images show quay faces with long shadows, patchy sediment
and metre-scale features; the simulated world is one material with the depth file's smooth relief. At
the thesis acquisition (real 18.4 m segments) the real seabed is also plain, with a soft fade-in at
nadir where the simulated edge is abrupt.

## 5. Ray-cast engine: what it implements, approximates and omits (V unless marked)

The model is **pre-sampled, world-fixed surface scatterers + a BVH visibility test + a coherent phasor
sum per range bin**. It is not rays-to-hits; rays serve visibility, transmission and diffraction.

| Subsystem | Implemented | Approximation / omission | Consequence for harbour runs |
|---|---|---|---|
| Scene (`scene_mesh.py`) | seabed raster triangulated at world resolution; smooth primitives as watertight meshes; walls as boxes | rough objects only as raster bumps; walls always "concrete" (`:501`); `reflectivity`, `roughness`, `transmission` rasters not read | objects under-resolved (§6); seabed homogeneous per material |
| Visibility | CPU linear BVH, fp64 watertight (reference); GPU `wp.Bvh` + own watertight test; built once | static scene | exact shadows; gates: 0 leaks, GPU = CPU to 1e-4 |
| Scatterers (`scatterers.py`) | jittered world lattice, 5 mm (10 mm in `fast`), hashed from the world seed | seed 0 in every profile | same speckle in every run of a world |
| Pose | 100 Hz pose predicted to the ping time, lever arm rotated by full attitude | one pose per ping (no Tx/Rx motion; `render(..., motion=None)`) | negligible at a 27 ms ping |
| Beam (`beam.py`) | near-field line aperture L = 0.243 m; vertical strip 3.5 mm; tilt 20°; baffled | "50°" read as a one-way −3 dB width (J); the large-angle vertical response is unmeasured | near-nadir level undetermined by physics |
| Receiver (`receiver.py`) | CW burst ⊛ filter ⊛ bin; delays at true c (1541 m/s), bins at 1500 m/s | waveform undocumented by the maker (J) | targets imaged at 0.973 R, as on the real device |
| Seabed (`seabed_ssa.py`) | small-slope roughness law + sediment volume, sand and mud | spectrum extrapolated 10× in frequency (D: literature gap); gravel, rock, seagrass fall back to sand | far-field level fitted on one recording |
| Objects | smooth: Kirchhoff patches × material reflection × coherence factor; rough: sand law × (R/R_sand)² | no diffuse share on smooth solids (`renderer.py:286-321`); roughness judged | aspect response is an upper bound (§6) |
| Propagation | spherical per leg, Francois–Garrison absorption | T, S assumed per site | minor |
| Multipath | image chains: surface, floor, walls, object faces | order 1 in the live, fast and harbour profiles; incoherent bounce shares not rendered | no quay ghost comb in harbour profiles |
| Diffraction | Fresnel–Kirchhoff on the direct leg | fast variants: wall-box edges only | hard shadow edges behind objects |
| Empirical layers (harbour profiles) | near-nadir table over r / altitude; 18 dB shadow floor; measured ringing and water column | table: one recording, 5.55–11.3 m, hard off outside, status `failed: every group EXCLUDED`; floor is a measured stand-in | declared limitation |
| Noise stage (`sonar/noise.py`) | speckle from the coherent sum; ringing; receiver noise; floor | **the floor is added in power to lit bins too** (`:127-131`) | fills speckle nulls: −20.0 → −16.7 dB (M, Monte-Carlo), measured −17.0 in the session |
| Ground truth | — | `renderer.py:430` returns `contacts=[]` | no labelled dataset |
| Cost | open water about real time in `fast` on an RTX 4060; quays held | out of memory near 50 m range | 20 m is safe |

**Assessment.** The core is sound and well gated by closed forms: strong internal consistency. Its
empirical standing is weak near nadir, and it has never been validated on an object: the only labelled
object recording is in the unopened hold-out. Its far-field level and texture agree with the real
harbour at the thesis acquisition to a few dB. The engine is the right basis; what the thesis lacks is
around it (§6, §8).

## 6. Submerged objects: definition → Gazebo → acoustics → ground truth

### 6.1 Lifecycle (V)

`CATALOG` (`worldgen/objects.py:208-370`) holds 29 classes; each is a 2-D footprint **mask** × height
range × material × a Gazebo shape. One `PlacedObject` feeds three code paths that share parameters and
rotation helpers but not geometry:

| | Gazebo (`sdf_writer._object_sdf`) | Raster stamp (`rasterize_objects`) | Ray-cast mesh |
|---|---|---|---|
| Geometry | one primitive visual | mask × height on the world grid | rough / aggregate: the stamped bump; smooth primitive: watertight mesh + patches |
| Physics | `<static>`, no collision, no inertial; pose computed, never simulated | — | — |
| Ground | terrain-only raster | terrain + earlier stamps | `height − facet_bump` |

Nothing floats, sinks, rolls or settles in Gazebo: placement is authored (or drawn) and resolved by a
geometric rest solver (`worldgen/solids.py` `settle`, stable poses weighted by face area). For static
seabed litter under a surface vehicle this is a defensible choice; it must be stated as such.

### 6.2 Inventory and provenance

Dimension source: **TF** = the owner's `target_families_and_surrogates.md` (vendor / JIS sizes);
**F** = a measured field object; **none** = no stated source. Path: **S** smooth solid (3-D mesh today),
**R** rough / aggregate (raster bump today).

| Class | Stands for | Geometry (mask → solid) | Material | Dims | Path | Gazebo visual |
|---|---|---|---|---|---|---|
| tire_car | car tyre / fender | annulus | rubber | none | R | solid disc |
| tire_bicycle | bicycle tyre | annulus | rubber | none | R | solid disc |
| pipe_pvc | pipe section | lying cylinder | pvc | none | S | cylinder |
| bottle_plastic | PET bottle | capsule | plastic | none | S | cylinder |
| can | drink can | lying cylinder | metal | none | S | cylinder |
| tent_weight | small concrete weight | box | concrete | none | R | box |
| rope | mooring line | wavy line | rope | none | R | straight box |
| cylinder_metal | gas cylinder | lying cylinder | metal | none | S | cylinder |
| concrete_block | rubble, mooring block | box | concrete | none | R | box |
| brick | brick | box | brickclay | none | S | box |
| chain | chain | dashed wavy line | metal | none | R | straight box |
| anchor | small anchor | shank + flukes "T" | metal | none | R | box |
| debris | unspecified debris | perturbed ellipse | generic | none | R | box |
| crate_plastic | fish crate | box | plastic | TF | S | box |
| plastic_box | 68 L storage box | box, thin flooded shell | pp_shell | F | S | box |
| tin_18l | 18 L square tin | box | tinplate | TF | S | box |
| polytank | 20 L polytank | box | plastic | TF | S | box |
| mooring_pile | driven pile | upright cylinder | concrete | J | R | cylinder |
| drum_200l | 200 L steel drum | upright cylinder | metal | TF | S | cylinder |
| pail_20l | 20 L pail | upright cylinder | tinplate | TF | S | cylinder |
| fire_extinguisher | fire extinguisher | lying cylinder | metal | TF | S | cylinder |
| pallet_wood | JIS T11 pallet | box thinned by a 0.10 m lattice | wood | TF | R | solid box |
| traffic_cone | traffic cone | cone | plastic | TF | S | cylinder of half radius |
| rope_coil | rope coil | dome with rings | rope | TF (part) | R | box |
| net_ball | net bundle | dome with rings | rope | TF (part) | R | box |
| bottle_glass | 1.8 L bottle | capsule | glass | TF | S | cylinder |
| bicycle | bicycle, lying | box thinned by a lattice | metal | TF (footprint) | R | solid box |
| mesh_crate | mesh container | box thinned by a lattice | plastic | TF | R | solid box |
| shopping_trolley | trolley, lying | box thinned by a lattice | metal | TF (footprint) | R | solid box |

Material constants (`worldgen/materials.py:216-253`): density and longitudinal sound speed are cited
(Selfridge 1985; Kinsler et al.), except wood and fired brick (stated, uncited); surface roughness is
"judged from manufacture, not measured" (`:115`); fill fractions, rest-pose weights and reflectivities
are judgement values; `rope` and `generic` have no acoustic constants and render as sand.

### 6.3 Ranked defects

1. **No ground truth under the ray-cast** (V). Perception builds boxes only from contacts
   (`sim_labels.py:567-575`).
2. **15 of 29 classes are not 3-D in the ray-cast** (V). In the harbour world (0.2 m cells) a tent
   weight is one raised vertex, bicycle tyres five, pallets 17–25 (M). No vertical faces, edges, slats or
   cavities; the 0.10 m lattice of bicycle, trolley, crate and pallet is below the grid.
3. **Labels would be misplaced by the range mapping** (V). Perception computes label ranges in true
   metres; the ray-cast images at 1500 / 1541 of true range. At 15 m the box sits about 0.4 m past the
   echo.
4. **The aspect response of smooth objects is an upper bound** (V + J). Mirror patches only, judged
   roughness, clean surfaces; real harbour objects are fouled. This favours the adaptive policy and must
   be a swept parameter; the real heading-response experiment is what bounds it.
5. **The material law for rough objects is unverified** (V + J). Sand law × (R/R_sand)²: rubber −14.8 dB
   re sand, wood −4.6, concrete +5.6, steel +8.1. A real tyre's return is tread, steel belt and geometry.
6. **No hollow geometry** (V): pipes without a bore, drums and tins as solid steel, crates without an
   interior.
7. **The latest world's objects are below the thesis' own floor** (M): 2–12 cm proud against "≥ 0.5 m
   with vertical relief" (`project_synthesis.md` §3.1).
8. **Gazebo is a visual stand-in** (V): shape substitutions above; the visual top sits
   `proud·burial/2` low (`sdf_writer.py:98`).
9. **Bug** (V): a rested object is drawn one water depth too deep (`sdf_writer.py:194` adds the terrain
   height to `rest_z`, which `solids.settle` returns as absolute world z). Visual only.
10. **Labels ignore shape and rest pose** (V): Perception's footprint is an L × W rectangle.
11. **No mesh import** anywhere (V).

### 6.4 Correction strategy (adopted, §8)

Objects leave the raster under the ray-cast: every object is meshed in 3-D at load from the world's
manifest (existing worlds need no rebuild), from the catalog primitives, from compounds of primitives,
or from the class's own mask sampled at 5–10 mm where no canonical geometry exists. Labels are read from
the same mesh by the same visibility test. Surface roughness becomes the one physical parameter that
sets aspect sharpness.

### 6.5 Validation scene

`~/worlds/validation/rc_validation_v1` (built by `analysis/validation_world.py`, §8 WP5): flat sand at
7 m, one quay, rows of pinned objects isolating range, shadow length, aspect, material, occlusion and
seabed contact.

## 7. Architecture decision

Three options were compared on the same requirements; HoloOcean facts are in `holoocean_reference.md`.

| Criterion | A — continue the custom ray-cast | B — adopt HoloOcean 2.4.0 | C — hybrid (Gazebo dynamics + HoloOcean sonar) |
|---|---|---|---|
| Harbour scene | built from 105 792 real soundings, quays, tide levels; one material; objects weak (fixable in place) | no bathymetry or GIS import documented; a new Unreal level to author; effort unknown | two worlds that must agree |
| Sonar, and can it be checked | near-field beam, coherent speckle, the Omniscan device chain; far field within a few dB of real; near nadir empirical; full source | octree: `R² cos` mean per bin, no spreading, beam or gain law; raycast / GPU: undocumented, source gated; no validation on real side-scan | HoloOcean's sonar plus an adapter rebuilding the device chain |
| Objects | three geometry paths; 15 / 29 classes as raster bumps | any Unreal mesh through its collision mesh; impedance per render material | every object authored twice |
| BlueBoat motion | Gazebo model, tuned controllers, sea state; gaps documented (pitch about 1.4× real, 16 kg against about 20 kg) | no BlueBoat; Otter parameters not configurable; controllers to re-tune | Gazebo's, pose copied into Unreal each tick |
| ROS 2 | identical interface in place, verified through MCS and GCS | bridge has no sonar topic and no TF | two clocks |
| Ground truth | missing for the ray-cast; schema and consumer exist | no side-scan labels | to build across two scenes |
| Performance | about real time in open water (M: ×0.92) | unknown for side-scan | unknown on one 8 GB GPU |
| Reproducibility | seeded, repeatable to 1e-9; no run provenance, stale-install trap | pinned binaries, gated source | two stacks |
| Effort before a January submission | days to about two weeks | weeks to months; does not fit this machine (4.7 GB free disk, 14 GB RAM) | B's sonar cost plus A kept alive |

**Recommendation given:** A, re-scoped to true 3-D objects, ray-cast labels, a verified bottom line and
run provenance, with no further seabed calibration.
**Owner's decision (2026-11-11): A.** The owner also allowed edits in other modules where this task
needs them.

Risks that stay under A: object aspect response is unvalidated until the real heading-response
experiment; the near-nadir table is one recording at one site; the vertical beam pattern is unmeasured.

## 8. Selected plan

Full text: `~/.claude/plans/pasted-content-id-5000-independent-lively-flask.md`. Every new behaviour is
opt-in by a generated profile key, switched on only in the two harbour profiles; a package rolls back by
its key.

| # | Package | Content | Key |
|---|---|---|---|
| WP0 | deploy, provenance | clean rebuild; build stamp; `core/provenance.py`; `~/ground_truth/run_info`; stale-install warning; GCS stores it | — |
| WP3 | bottom line | `analysis/raycast_bottomline.py`: current source + the processor's own tracker on a recorded track | — |
| WP2 | ground truth | `sonar/raycast/labels.py` from the renderer's own oracle; `range_scale`; Perception applies it | `raycast.labels.enabled` |
| WP4 | shadow floor | added in the field domain | `raycast.floor.addition: field` |
| WP1 | 3-D objects | terrain-only sheet; primitives, compounds, fine mask meshes; diffuse scatterers on object triangles; diffuse share on smooth materials | `raycast.objects.mode: solid` |
| WP6 | Gazebo | rested-object height; burial offset | — |
| WP5 | validation | validation world; `raycast_test --only V` | — |
| WP7 | record | this document; module documentation | — |

Not done: new seabed or near-nadir physics, pool multipath, GPU memory work above 30 m, any HoloOcean
code.

## 9. Implementation record

Filled as each package lands (date, files, what changed, what stayed identical and how that was shown).

| Date | Package | Record |
|---|---|---|
| 2026-11-11 | pre-edit baseline | `~/ros2_ws/sim_runs/raycast/review_a/guard_a.py`: baseline of the fast and both harbour profiles at six poses (72 arrays; repeat render differs by 1.3e-15 relative); default and live pass their existing baseline at 1.8e-15 before any edit |
| 2026-11-11 | WP0 provenance | New `core/provenance.py` (code digest over 169 files in 13 ms, build stamp, install-vs-source check, world and profile digests; CLI `--check`). `setup.py` writes `_build_stamp.json` into the build. `sss_sim_node`: latched `~/ground_truth/run_info` at start-up / enable / stop, a start-up warning naming the stale files, `run_info_dir`, `run_id` + `range_scale` on `ground_truth/sensor`, `seed_per_run`. GCS: `core/sim_run_info.py`, a latched subscription, `metadata.json["simulation"]`. Run against the stale install, the check names exactly the nine files `diff -rq` found, plus `sea/yaw_moment.py` (deleted from the source, still installed) |
| 2026-11-11 | WP3 bottom line | New `analysis/raycast_bottomline.py`: a recorded track → current source → noise stage → `PingEncoder` → the processor's own `scale_to_db`, `detect_fbr_slant_m` and `FBRTracker` (constants parsed from `sss_processor_node.py`). No model change was needed: the 11-10 fixes already in the source pass (§10) |
| 2026-11-11 | WP2 ground truth | New `sonar/raycast/labels.py` (`LabelOracle`), called from `RaycastRenderer.render`; key `raycast.labels.enabled`. Perception: `range_scale` in `core/sim_labels.py` (`PingTruth`, `ping_object_bins`), two tests, `docs/topics.md`. The ping is unchanged with labels on (2.7e-16 relative) |
| 2026-11-11 | WP4 floor | `sonar/noise.apply_ping_noise(..., floor_addition="power")`; `"field"` adds the interference term where the reverberation is the floor; key `raycast.floor.addition`. `"power"` is the previous code bit for bit (the geometric renderer and the default / live / fast profiles do not move) |
| 2026-11-11 | WP6 Gazebo | `worldgen/sdf_writer.py`: a rested object's z is `rest_z` (absolute), and the second burial subtraction is gone. New worlds only; no acoustic effect |
| 2026-11-11 | profiles | `config.py` `_harbour()`: `floor.addition: field`, `labels.enabled`, `seed_per_run`; the four variant files regenerated; the new sections are outside the correction's core fingerprint (`correction._NOT_CORE`); `system_gain_db` bounds [105, 165] now enforced; four registry entries |
| 2026-11-11 | WP1a objects | Key `raycast.objects.mode: raster\|solid`. New `sonar/raycast/object_mesh.py` (`object_geometry`: the catalog primitive placed by `facet_transform`, or `mask_mesh`: the class's own mask at ≤ 1 cm on the terrain). `scene_mesh._build_solid`: terrain-only sheet from `depth.npz`, every object its own mesh, new kind `KIND_SOLID`; a world without `depth.npz` stays raster. Renderer: rough object triangles join the planar sampler (CPU `planar_sample`, GPU `_wall_emit` unchanged; the GPU plan is grouped per object with a reach filter), altitude and the correction's nadir gate read the terrain under the transducer. Node: mesh cache keyed by the mode. On in the harbour profiles |
| 2026-11-11 | tests | smoke section `[18]` (7 checks); `raycast_test --only L` (9 checks) and `--only O` (9 checks); Perception +2 tests; GCS `tests/test_sim_run_info.py` (4 tests) |

## 10. Validation results

Tolerances are written before a result is read. Without measured object data (the hold-out is sealed),
object checks are **internal-consistency tests**, not empirical validation.

| Check | Tolerance (pre-stated) | Result |
|---|---|---|
| Bottom line: the 2026-11-11 track (3 305 pings, both sides), harbour-fast from the current source, the processor's own tracker (`analysis/raycast_bottomline.py`; `~/ros2_ws/sim_runs/raycast/review_a/bottomline_2026_11_11_harbour_fast.json`) | detection ≥ 98 %; \|median error\| ≤ 0.10 m; ≥ 95 % within 0.30 m; locked ≥ 95 %; jump p90 ≤ 0.15 m; ringing end 0.10–0.18 m; noise estimate 26.5–31.5 dB | **pass**: detection 100 % / 100 %; median −0.019 / −0.018 m; within 0.30 m 99.1 / 99.6 %; tracker locked 99.7 %, its error median −0.016 m, 99.6 % within 0.30 m; jump p90 0.083 m (sides), 0.10 m (tracker); ringing end 0.133 m; noise 29.45 dB. The session's own run (stale install): median +1.35 m, 14.5 % within 0.30 m. Offline: heave is not replayed |
| Shadow length from labels, flat sand, 0.15 and 0.40 m boxes at 6 / 10 / 15 m (`[L2]`) | within 2 bins of geometry | **pass**: 5.3 / 10.6 / 17.8 / 15.3 / 31.1 / 50.9 bins against 5.6 / 11.3 / 18.4 / 15.6 / 31.4 / 51.1; worst 0.69 bins |
| Perception's box against the render (`[L3]`, GPU) | ≥ 98 % of highlight and shadow bins inside; far end overshoots ≤ margin + 3 bins; fails without the range scale | **pass**: 100 / 100 / 100 / 98.7 %, overshoot −1…+2 bins; without `range_scale` 60.6 / 47.7 / 90.7 / 79.5 % |
| Occlusion (`[L4]`) | behind a quay or inside another object's shadow: `visible: false`, shadow 0 | **pass** |
| Labels never change the ping (`[L6]`); cost (`[L7]`) | ≤ 1e-9 relative; ≤ 3 ms with an object in the strip, ≤ 0.2 ms with none | **pass**: 2.7e-16; 1.07 ms / 0.038 ms |
| Field-domain floor: dark tail, mean, water column, old path | −20.0 ± 0.15 dB re mean; mean within 1 %; water-column bins equal; `power` `array_equal` to the old lines | **pass**: −19.96 dB (power −16.58 dB); 1.0141 vs 1.0147; equal; equal. Kolmogorov distance to Exp(1) 0.0012 (5 % critical 0.0031; power mode 0.0143) |
| Default, live, fast and both harbour profiles: `Ping.power` and `texture` unchanged | ≤ 1e-9 relative (guard, six poses, both sides) | **pass**: 1.6e-15 (default, live), 1.3e-15 (fast, harbour) |
| Object-free world, solid vs raster (`[O5]`) | ≤ 1e-9 relative | **pass**: 3.3e-16 (power), 3.6e-15 (texture) |
| Rough primitive meshes (`[O1]`) | volume within 0.5 %; top at terrain + effective height | **pass** after tightening the mesh tolerance to 0.5 mm (at 1 mm a 0.4 m pile was 0.64 % short): −0.20 / 0.00 / −0.32 / 0.00 % |
| A concrete block 10 m across, solid mode (`[O6]`) | highlight over the bare seabed; shadow under −10 dB ending within 5 bins of geometry; contact shadow within 2 bins | **pass**: +10.2 dB; −27.6 dB; bin 748 vs 746; 31.1 vs 31.4 bins |
| Material ordering on one rough block (`[O7]`) | 20·log₁₀ ratio ± 0.1 dB; far seabed unchanged | **pass**: +5.71 vs +5.71 dB; 1.6e-16. (First written with wood, which is a smooth material and rightly takes the Kirchhoff path: the test was wrong, not the code) |
| Harbour world, one ping abeam of each object, against the same seabed without it (`review_a/solid_probe.py`) | reported, no tolerance | raster → solid: tent weight shadow −3.8 → −16.0 dB, highlight +6.3 → +7.4 dB; pallets shadow −4…−18 → −18…−33 dB; bicycle tyres −12 → −20 dB, no highlight in either |
| Cost of solid mode on the 11-11 track (3 305 cycles, harbour fast) | open water ≤ +5 % | **pass**: 35 vs 34 ms per cycle (+3 %); tracker metrics unchanged. At a pallet (mask mesh, ~15 000 triangles) a ping-side costs 31–39 ms against 12–24 ms: above the +40 % budget until the pallet is a compound of boards |
| Default / live / fast unchanged; harbour changed only where rough objects are in view | ≤ 1e-9; per-pose report | **pass**: fast 6e-16…1e-15 at all six poses; harbour ≤ 2e-15 at five poses, changed at the one pose with car tyres |
| Range bin of a known target; aspect; pose; repeatability | ± 1 bin with the 1500 / c mapping; see the plan | pending (WP5) |

Not yet shown by any test: the labels and the bottom line in a **live** Gazebo run with the rebuilt
install (the owner's look test), and anything about object returns against measured data.

## 11. Limitations and remaining work

- Near nadir: one-site empirical table that failed its strict gates; the vertical beam pattern is
  unmeasured. A tank or roll-sweep measurement would settle it.
- Objects: diffuse law is the sand spectrum scaled by |R|²; roughness is judged; wood, rope and rubber
  constants are surrogates (a tyre has no steel belt in the model); Kirchhoff on thin members has no
  edge term; mask-mesh classes stay 2.5-D statistical shapes.
- Seabed: one material per world in practice (sand, mud); the builder's variability map is not read.
- Multipath order 1 in the harbour profiles; water-volume reverberation is a measured stand-in.
- Boat: pitch response about 1.4× the corpus; mass 16 kg against about 20 kg (documented in `TODO.md`).
- Labels follow the direct path only.
- The object aspect response has no empirical check until the real heading-response experiment.

Open owner decisions, none blocking: a steel-belted tyre material; extended contact keys for rested
objects; honouring the builder's seabed variability map.

## 12. Decision log

| Date | Decision | Why |
|---|---|---|
| 2026-11-11 | Option A | sound core, far field matches real data, gaps are bounded and inside this stack; B and C lack an inspectable, validated side-scan, a BlueBoat, a sonar topic and a harbour import, and do not fit the machine |
| 2026-11-11 | Labels before 3-D objects | labels block every dataset and work on today's mesh |
| 2026-11-11 | New behaviour opt-in by profile key, harbour profiles only | default, live and fast keep their look; one-key rollback |
| 2026-11-11 | Unrested objects keep the stored pose (yaw only) | the stored ground truth stays authoritative |
| 2026-11-11 | No new near-nadir physics | Stage 2's stop rule stands; the missing evidence is a measurement, not a model |
| 2026-11-11 | Tyre material unchanged | no measurement to anchor a new one; listed as an owner decision |
| 2026-11-11 (second pass) | Fouling default 0, wall type chosen in the World Builder, the real nadir ramp | owner's answers; fouling is a sensitivity axis, not a calibration |
| 2026-11-11 (second pass) | Tyre: a steel-belted tread band from cited constants, no absorption | the owner asked for a material fix; no absorption value was read at a source, so the tread echo is stated as an upper bound |
| 2026-11-11 (second pass) | Gravel and rock as a published level on the sand law, not an unread parameter set | the level is already cited in `seabed_model.md`; APL-UW TR 9407 was not read |

## 13. Second pass, 2026-11-11: one surface rule, the nadir onset, the missing recording

Asked by the owner after the first live run with the option-A build
(`sessions/2026_11_11-11_54_29`). As built: `.claude/raycast-live.md` (last block);
justification and references: `docs/simulator_justification.md`.

**Findings before the change (M).** Smooth objects were mirrors with no diffuse part; ten classes
were 2.5-D mask skins (a pallet 15 000 triangles); every wall was flat concrete; gravel and rock
rendered as sand; the simulated first return stepped up in 2 bins with a +4 dB crest where the real
one starts 9.5 dB under the far seabed and climbs over 0.5–0.7 m (the physical model alone starts
at −8.7 dB: the step was the empirical table at its maximum on the first bin); a GCS session had
no `.svlog` because Record ON was published 5 ms before `sss_processor` subscribed.

**Results against tolerances stated before running.**

| Check | Tolerance | Result |
|---|---|---|
| Default / live / fast profiles unchanged | 1e-9 relative | 1.2e-15, 9.1e-16 — pass |
| Harbour profiles beyond the fade, object-free poses | identical | ≤ 5e-16 — pass |
| Object-free world, solid vs raster | 1e-9 | 3.3e-16 — pass |
| Rough block: echo ∝ material \|R\|² | 0.1 dB | +5.71 vs +5.71 dB — pass |
| Fully fouled plastic block = sand-equivalent block | 1e-9 | 8.1e-16 — pass |
| Steel wall over concrete wall | 20·log₁₀(R_s/R_c) ± 0.1 dB | +2.46 vs +2.46 dB — pass |
| Car tyre tread band over the seabed at 10 m | ≥ 10 dB | +9.8 dB — **0.2 dB short**; bare rubber +0.4 dB. Not tuned |
| Every class: outward part meshes inside the stored footprint | 1.5 cm (a rope or chain: its own 1.5-width wander) | pass |
| First return: 10–90 % rise per ping | 0.4–0.8 m | fade √, 0.8 m: profile within 2.6 dB of the real one at every offset (limit 3 dB), 1.4 dB rms |
| Processor tracker on the 11:54 track | detection ≥ 98 %, locked ≥ 95 %, median error −0.05…+0.35 m, jump p90 ≤ 0.5 m | 100 %, 99.7 %, +0.28 m, 0.42 m — pass |

The fade was not right first time, and every value tried is recorded: a smooth step over 0.8 m
(profile 4.5 dB off, tracker +0.49 m: fail), √ over 1.2 m (2.3 dB, +0.42 m: fail), √ over 0.6 m
(3.0 dB, +0.20 m), √ over 0.8 m (kept). One shape and one length were to be tried by the plan's
own rule; four were. The two criteria were fixed beforehand and the kept value is the only one
meeting both with margin.

**Limits added by this pass.** A compound's faces are not bounce mirrors; thin members (wires,
tubes) use Kirchhoff patches without an edge term; a pallet in range costs its ~28 000 mirror
patches each ping (40 vs 35 ms per cycle on the 07:57 track); the tyre tread echo is an upper
bound; hollow bodies and sheet-pile corrugation are not modelled.
