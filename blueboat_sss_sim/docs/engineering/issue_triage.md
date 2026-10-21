# Issue triage — 2026-10-21 Cloud audit

Every finding inherited from the two supplied documents, plus the new ones
this audit turned up, classified against the **current source tree**. The
two inputs were the technical review ("Technical review: BlueBoat-SSS-Sim
for USV + side-scan sonar", sections R3.x below) and the status memo
("BlueBoat-SSS-Sim — Simulation Current Status", §32 #1–#27). Both were
treated as claims to check, not as truth.

**Status vocabulary.**

| Status | Meaning |
|---|---|
| **confirmed → fixed** | Reproduced on the pre-fix tree, fixed, and pinned by a regression check |
| **confirmed (limitation)** | True, but an intentional model approximation, not a defect |
| **obsolete** | Already fixed in the tree before this audit; re-verified |
| **reported resolved** | The memo says it is fixed; not re-verified line by line here |
| **refuted** | The code does not do what the claim says |
| **not reproduced** | Plausible, no deterministic reproduction attempted (GUI, Gazebo) |
| **inference only** | A reasoned consequence, not measured |
| **blocked** | Needs field data or a Gazebo/ROS session this environment lacks |

Severity: **P0** corrupts labels, geometry or provenance; **P1** limits
calibration or validation; **P2** quality or documentation.

## New defects confirmed and fixed in this audit

| ID | Finding | Location | Sev. | Evidence (pre-fix → post-fix) | Fix | Regression |
|---|---|---|---|---|---|---|
| F1 | A smooth (facet) object behind a quay was **rendered** while its ground truth said `visible=False` (memo #3) | `sonar/renderer.py` `render()` passed only the terrain/object horizon to `FacetLayer.render` (`sonar/facets.py`); wall blocking lived in `lit_extra`, which the label read and the facet layer did not | P0 | `cylinder_metal` 9 m out behind a quay at 6 m: extra echo **25.05 → 0** (linear, object window); label `False` both times; pontoon case unchanged (25.05, `True`) | The direct path's wall-band rule applied per patch (`multipath.segment_blocked`), passed as `occluded=` to the facet layer; gated by `wall_echo_enabled` like the seabed | smoke `[15]` checks 1–2 |
| F2 | **Land behind a quay deleted that quay's ghost** (memo #21) | `renderer._ping_geometry` for a mirror source sampled terrain *behind* the wall for the first leg, which set the horizon | P0 | `[4d]` basin, quay at y=10: ghost energy water **0.0037775**, land **0 → 0.0037775** | Pre-crossing samples of a wall ghost read height, facet bump and transmission at the point reflected across the wall line (the real transducer→face leg). Flat basins are bit-identical | smoke `[15]` check 3; all `[4d]` checks unchanged |
| F3 | **Gazebo poses put the transducers 0.111 m deeper than calibrated** (memo #10) | `ros/sss_sim_node.py` passed Gazebo base_link z raw; at rest that is `HullBoxes.gazebo_box().equilibrium_z()` = **−0.1108 m** (`sea/hull.py`; also derived by hand from the BlueBoat xacro's buoyancy box). Every offline/calibration render uses pose z = 0 = waterline | P0 | Transducer z at rest **−0.2608 → −0.1500 m** | `sea.motion.waterline_pose` + `GAZEBO_BASE_REST_Z_M`; new node parameter `base_link_rest_z_m` (default that value; `0.0` restores the old behaviour). `boat` in `ground_truth/sensor` stays the received pose; new key `base_link_rest_z_m` | smoke `[15]` check 4 |
| F4 | A hand-written explicit entry naming a `material` kept the **class default's drawn reflectivity**, and its `params` size knobs were **stored but ignored** | `worldgen/objects.place_objects_v2` explicit loop called `_make_instance` without `material_override`/`params` | P0 (labels/brightness of raster objects) | concrete `can` reflectivity **0.970 (metal band) → 0.817 (concrete band)**; `plastic_box` `params.length_m: 1.25` → length **0.721 → 1.250 m**; no other object's draw moves | Pass both into the draw (same RNG consumption). World Builder entries pin every field and are unaffected | smoke `[15]` check 6 |
| F5 | The simulator's **wire span exceeded 90 dB** on clamped pings (float32 rounding of the two endpoints) | `sonar/encoder.py` wrote `min_pwr_db`/`max_pwr_db` independently rounded | P2 (sim tell; breaks `normalisation_invariants` on sim recordings) | Synthetic clamped pings: **516/2000 → 0/2000** wider than 90 + 1e-6 dB; the attached simulated 2026-09-18 recording shows it on 628 of 13 157 pings; 0 in both attached real files | Nudge the float32 minimum up by ulps until the pair holds; counts and unclamped pings untouched | smoke `[16]` check 12 |
| F6 | The **sensor-noise seed was never recorded**; no launch passed one | node `seed` 0 → `default_rng(None)` | P0 (provenance) | — | `seed` 0 now draws a concrete entropy seed, logs it, publishes `noise_seed` on `ground_truth/sensor`; launch arg `sonar_seed` (default 0, as before) | `py_compile`; no ROS here to run the node |
| F7 | `full_mission_launch.py` could not choose a sonar profile | it did not declare/forward `sonar_config` | P1 | — | Additive `sonar_config`, `sonar_seed` args forwarded to `sss_sim_launch.py` | `py_compile` |
| F8 | `sss_calibration_report`'s simulation omitted `ambient=` in the noise call, so its shadow floors differed from a live run's | `analysis/calibration_cli.simulate_profiles` | P1 | — | Pass `ambient` like the node | none (needs a world + recording; behaviour now identical to the node's call) |
| F9 | The measured `reverberation_db` (18) lived only in the dataclass, so a frozen copy of the default profile did not freeze it | `config/default_sonar.yaml` | P2 | registry drift check reported it as implicit | Stated explicitly, same value | smoke `[16]` check 1 (no implicit keys) |
| F10 | Out-of-map seabed was **silent**: nothing told a run or a dataset that the swath saw the phantom plain (`fill`) or the repeated edge (`edge`) (memo #20, review R3.7) | `worldgen/scene.py` `sample_height`/`sample_reflectivity` | P0 (labels near the map edge) | — | Diagnostics, render unchanged: `Ping.out_of_map_from_m`; per-side `out_of_map_from_m` on `ground_truth/sensor`; one node warning per run; load log names the mode | smoke `[15]` check 5 |

## Inherited findings

### Status memo §32

| # | Claim | Status | Evidence |
|---|---|---|---|
| 1 | `brick` is a facet object | obsolete | docs corrected; `is_facet_object` (brickclay 1 mm < 1.6 mm) |
| 2 | Pinned `material` does not update roughness | obsolete (partly) → **F4** | transmission + roughness followed since before this audit (`objects.py` explicit loop); reflectivity and size knobs fixed now |
| 3 | Facet objects not culled by walls | **confirmed → fixed (F1)** | |
| 4 | GUI shows reflectivity for facet objects | obsolete | `builder/pages/place_page.py:149-154` disables it for facet objects |
| 5 | Lying-cylinder height is a cap | reported resolved (documented) | |
| 6 | `mooring_pile` can lie down | obsolete | `rest_allowed=False` (`objects.py:305`) |
| 7 | `eps` unused | confirmed (limitation, documented) | |
| 8 | `object_id` positional | confirmed (limitation, documented) | seeds rope/chain/coil/lattice shapes; reordering moves later shapes |
| 9 | Mount offset docs stale (0.20 m) | obsolete | all docs say 0.285 m / 49.5 mm |
| 10 | `sensor_depth_m` effective ~0.26 m in Gazebo | **confirmed → fixed (F3)** | |
| 11 | Two wavenumbers | obsolete | one `acoustics.wavenumber` (450 kHz); 451 127 Hz only on the wire |
| 12 | `msg_reference` "linear echo power" | obsolete | comment corrected |
| 13 | "38.5 dB" | obsolete | `config.py` says 35.5 |
| 14 | Water-column margin wording | reported resolved | |
| 15 | Sidelobe-floor wording | reported resolved | |
| 16 | Node needs odometry while the 100 Hz pose flows | **refuted** | `_ping_pose` checks the high-rate pose first; smoke `[13]` covers it (SKIP here: no `rclpy`) |
| 17 | K-distribution described as implemented | obsolete | `seabed_model.md` says not modelled |
| 18 | Material spread wording | reported resolved | |
| 19 | Walls "no collision" docs | obsolete | `sdf_writer.py` emits collision; docs agree |
| 20 | Phantom plain beyond the map | obsolete (Builder worlds use `edge`) + **F10** diagnostics | legacy `fill` kept: the render fixture is frozen on it |
| 21 | Ghost horizon over terrain behind a wall | **confirmed → fixed (F2)** | |
| 22 | Screen-3 preview RNG ≠ build RNG | not reproduced (GUI) | `builder/worker.py:135` uses `default_rng(0)` for terrain texture; P2 |
| 23 | `wall_names` vs curved-wall chord names | not reproduced | P2 |
| 24 | `rocky_outcrop` has no rock material | reported resolved (documented) | |
| 25 | Builder cannot carry `materials:` overrides | confirmed (limitation, documented) | the calibration artifact path does not depend on it |
| 26 | Raster-cell claims stale | obsolete | |
| 27 | Depth flood fill ignores wall height | confirmed, open (`.claude/TODO.md`) | P2 |

### Technical review §3

| ID | Claim | Status | Notes |
|---|---|---|---|
| R3.1 | Empirical intensity renderer, straight rays, stop-and-hop | confirmed (limitation) | Assumptions A1–A13, `docs/sonar_model.md` §7. Appropriate at 20 m / ~1 m/s; intra-ping motion is about 9 cm at the far edge at 10°/s yaw (memo, inference) |
| R3.2 | Sediment-independent Exp(1) speckle; 5.9 dB nominal material spread vs ~15 dB in the literature | confirmed (limitation) | **blocked** on classified-bottom captures; see the validation plan |
| R3.3a | Material override not atomic | → F4 | |
| R3.3b | Facet objects behind walls | → F1 | |
| R3.3c | GUI exposes ignored reflectivity | obsolete (#4) | |
| R3.3d | Visual-only Gazebo objects; cone drawn as a cylinder | confirmed (limitation) | No acoustic effect: the sonar reads `scene.npz` |
| R3.3e | Reordering objects changes shapes | confirmed (limitation, #8) | |
| R3.4 | Validation narrow relative to the parameter space | confirmed | addressed by `validation_and_experiment_plan.md` and the calibration scaffolding; data **blocked** |
| R3.5 | No AGC; `seed=0` non-reproducible; no jitter | AGC: limitation (A8; `fault_gain_walk_pings` reproduces the walk). Seed: **fixed (F6)**. Jitter: limitation | the shared node RNG interleaves port and starboard draws (N3 below) |
| R3.6 | Simulated yaw rate 4–10× too low | reported (memo §29.6, `.claude/TODO.md`); **blocked** | needs field motion logs plus Gazebo |
| R3.7 | Phantom plain; sensor-depth datum; ghost horizon | → F10, F3, F2 | |
| R3.8 | Tests are regression tests, not validation | confirmed | `[16]` adds contract tests; held-out validation needs data |
| R3.9 | Performance scaling unmeasured | inference only | facet occlusion adds one `segment_blocked` per smooth object in the beam, only when walls exist |

## Other observations (not changed)

| ID | Observation | Sev. | Recommendation |
|---|---|---|---|
| N1 | `objects.classes.<c>` ignores unknown keys: `count: 3` instead of `amount: {kind: count, value: 3}` places **nothing**, silently | P2 | Make class entries strict (raise by name) in a deliberate change; documented in `configuration_guide.md` |
| N2 | Bin-centre conventions differ across analysis tools: `i·L/N` (`calibration.py`, `svlog_digest.py`), `(i+½)·L/N` (renderer, `RangeCurve`), `i·L/(N−1)` (`bathymetry.py`) | P2 | Up to half a bin (8 mm at 1200 bins). The calibration package uses the renderer's `(i+½)·L/N`. Pick one when the real device's convention is measured on a target at known range |
| N3 | One node RNG serves both sides, so a dropped ping on one side shifts the other side's noise | P2 | Split per side (`SeedSequence.spawn`) when augmentation needs per-side reproducibility; it changes noise realisations, not statistics |
| N4 | `pose_age_ms` can be negative, and `predicted` reports the parameter rather than whether prediction was applied | P2 | Document; clamp in a later interface revision |
| N5 | The sonar node's odometry fallback has no staleness limit (the sea node uses 1.0 s) | P2 | Gazebo gate's "fresh pose" stalls (TODO) are the place to decide |
| N6 | Smoke `[4d]` ghost lab assigns objects after the scene is built (never stamped) and uses `block_concrete`, which is not a `CATALOG` key | P2 (test quality) | Fix when `[4d]` is next touched |
| N7 | Real-data observation: `object_boiteEncore` (attached) has **1 of 4236** pings with two bins tied at 65535, so "exactly one full-scale bin on every field ping" is not universal | info | The quality report warns (does not reject) below 100 % and errors below 99 % |
| N8 | Environment: PySide6 `QtGui` needs `libEGL.so.1`; this container lacked it, and the first baseline crashed at `[12]` after 381 passes | env | Installed `libegl1` (apt). A fresh container needs the same |
| N9 | Docs: the A-list in `sonar_model.md` §7 is out of order (A10, A12, A13, A9, A11) | P2 | Cosmetic; left |
