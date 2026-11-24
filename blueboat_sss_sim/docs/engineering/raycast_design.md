# Ray-cast sensor — design note (Phase 0)

Second simulated sensor, `renderer: raycast`, beside the unchanged `GeometricRenderer`. Plan and gates:
`raycast_plan.md`. Literature: `docs/literature/raycast_sonar/INDEX.md` (notes 01–10 are cited as
[n01]–[n10]). Numbers: `~/ros2_ws/sim_runs/raycast/notes/phase0_numbers.txt`. Status 2026-11-07:
Phases 0–6 built, GPU path complete, live in a slowed simulation (§11). The calibration status, including the
harbour-first hybrid, is in `raycast_assessment.md` §13. This document is the design; the 2026-11-02 Phase-0 text
below is kept as written.

## 1. What the sensor is

The sensor is a full 3-D, path-resolved, **coherent** side-scan renderer. Its scene is a triangle mesh
derived from the same `SceneModel` the geometric renderer reads. One visibility oracle (a BVH) answers
every leg of every path. Its output is the same `RenderedPing`, through the same encoder, noise stage,
topics and ground-truth schema. It aims to be more realistic, measured against the corpus, never by
looks. Its constants are physical and cited. The only fitted parameters are the system gain plus the
parameters that have no measured value, each inside literature bounds (§8).

## 2. Formulation

**Field.** For each side, ping and sample time `t_b`:

`E(t_b) = Σ_{paths p} Σ_{contributors n} A_{p,n} · s_mf(t_b − τ_{p,n}) · e^{−iω τ_{p,n}}`,
`I_b = |E(t_b)|²`,

with
`A_{p,n} = b_t(û_out; pose_t) · b_r(û_in; pose_r) · Π_legs [e^{−α ℓ ln10/20}/ℓ · D_leg] · Π_bounces R_j · a_n`.

* `b_t`, `b_r`: near-field line-aperture directivities, each in its own pose (§4.6).
* `D_leg`: diffraction factor of the leg (1 when its Fresnel disc is clear; [n06]).
* `R_j`: reflection coefficient of each specular bounce on the path. Surface `−1 · √C`
  (Eckart); seabed `V(θ)` from impedance `· √C`; walls and objects from `_ACOUSTIC` impedances
  `· √C`. The incoherent `1 − C` share goes to the diffuse channels.
* `a_n`: the contributor's scattering amplitude (§2.1).
* `τ_{p,n}`: the exact delay with the receiver's own pose at arrival (§4.6).

**Contributors.**
1. **Diffuse surface scatterers.** World-fixed points on every rough triangle (seabed, rough objects,
   wall faces, the rough part of the surface). Their positions and phases come from a counter-based hash
   of (world seed, triangle id, cell index), as `renderer._scatter_phase` already does for the geometric
   renderer, so consecutive pings re-illuminate the same scatterers. Each has
   `E|a_n|² = σ(θ_i, θ_s, φ; material) ΔA_n`, phase uniform from the hash [n01, n04].
2. **Coherent PO patches.** Every smooth surface: smooth solids, walls' coherent share, calm surface,
   smooth floor near its specular point. Amplitude from `facets.po_amplitude` / `po_bistatic`, times
   the material `R` and `√C`. Glints, aspect, edges and layover emerge [n05].
3. **Volume scatterers** in the water column at the site's `S_v` [n08], and in the sediment through
   the equivalent-interface volume term inside `σ` [n03].
4. **Edge secondary sources.** Only if the knife-edge model proves insufficient (§7, D-gate) [n06].

**Paths.** For each contributor, the set of **one-way eigenrays** from the transducer is built:
- direct;
- surface image;
- wall images (exact, planar faces);
- floor image in the local tangent plane, refined by Fermat on the mesh;
- chains up to the order cut-off (§2.2).

Every leg is ray-cast against the whole BVH (one oracle). The two-way contribution of a contributor is
the sum over ordered pairs (out eigenray *i*, back eigenray *j*). The monostatic reciprocity is that the
same set serves both ways; the back pair uses the receiver's pose. This single construction produces:
- the Lloyd mirror and its fringes;
- the shallow-water image comb, with each image's own visibility (pass-4 failure 1);
- object–seabed dihedrals [n05];
- quays hiding what is behind them for **every** path, ghosts included (B1);
- the FBR imaged as direct, mixed and surface–surface arrivals (the pool ramp).

### 2.1 Mean and realisation

`Ping.power` = the exact mean `Σ E|a|² |s_mf|²` over the diffuse contributors, plus `|Σ coherent|²`.
`Ping.texture` = `|E|² / power`. The shared `apply_ping_noise` then reproduces the realisation
(`speckle: true`, `coherent_scatterers: true`, `speckle_looks: 1`, `specular` = None).

**Monte-Carlo/quadrature error and speckle are separated by construction:**
- *speckle* is the spread of `|E|²` across **world seeds**;
- *sampling error* is the change of the same-seed `E` (and of `power`) as the contributor density and
  ray counts rise.

### 2.2 Path order cut-off

A chain is kept while its **upper-bound** contribution exceeds the noise-plus-reverberation floor of
its range bin minus 10 dB:

`bound = |b_t|_max |b_r|_max · Π|R_j|_max · (spreading, absorption of the unfolded length) · σ_max`.

So the comb in a calm pool (|R| ≈ 1 at the surface, 0.3–0.7 on the floor) is followed to high order,
and the same chain in waves (C ≈ 0) stops at first order. The rule is evaluated per scene and range;
the order actually reached is reported per ping.

## 3. Effect inventory

`I` include, `O` include-optional (off by default), `X` exclude. Each with its number or the reason a
measurement is needed. The test codes refer to §7.

### Transport and geometry

| Effect | Decision | Number / justification | Test |
|---|---|---|---|
| 3-D triangle scene derived from `SceneModel` (terrain from `depth.npz`, rough objects from `height − facet_bump − terrain`, smooth solids meshed per primitive with `facets.object_patches`' transform, walls as wet-span boxes) | I | the single source of truth; the pass-4 B1 failure needs an object-free terrain, which `depth.npz` provides | M1–M5 |
| World limits | X (NC #12) | collision-only by rule | M4 |
| Dry land silent | I (NC #13) | terrain above the transducer scatters nothing back into the water | M4, P-land |
| Pontoons passable below the draft | I | `Wall.draft_m` | M3, P-wall |
| Monostatic time-of-flight binning with the device bin width, smeared by the receiver envelope: CW pulse ⊛ receive filter of κ·τ ⊛ one bin | I | **CW evidenced, κ = 1.4 MEASURED** (X2, 2026-11-03: the corpus speckle's range correlation fits a filter of 0.8–1.5 × τ over τ = 44–208 µs, not a fixed share of the round trip, 0.14–0.36 %); pulse 3.3 cm vs bin 1.67 cm; pass 4: the boxcar kernel gives lag-2/3 of 0.26/0.10 against 0.10–0.17 / 0.03–0.10 real | R1, C2 |
| Exact visibility, every leg, one oracle | I | the four pass-4 architecture failures (hand-off §1–2) | V1–V3 |
| Beam sampled in 3-D: contributors over the whole beam solid angle, sidelobes included to −40 dB two-way | I | the shadow floor needs sidelobes; uniform aperture two-way sidelobes −26.5 dB [n09] | B1–B3 |
| Direct, surface, bottom, wall, dihedral, comb paths; higher orders by §2.2 | I | calm pool: surface C ≥ 0.94 beyond 5 m; the comb explains the pool's range shape (pass 4) | P1–P7 |
| Bistatic scattering laws for every bounce-scatter-bounce path | I | a second bounce leaves backscatter geometry (Bell97a; [n03]) | S2 |
| Refraction by a constant-gradient profile | O | mixed pool 0.02 °C/m: 0.05° at 20 m, negligible; 0.5 °C/m: +2.1/−2.7 dB at 20 m. No corpus profile exists → needs measurement [n02] | P-refr |

### Scattering and interfaces

| Effect | Decision | Number / justification | Test |
|---|---|---|---|
| Seabed roughness: small-slope approximation (reference) or composite roughness, bistatic, per material, with the spectrum **above the mesh Nyquist** only (`K_c = π/Δx`; the mesh carries the large scales) | I | JR07 §13.4–13.5; slopes never counted twice | S1, S2, S-cut |
| Sediment volume scattering (eq. 14.8), `σ₂` ∈ [1e-4, 4e-3] | I | sand: 1/e depth 3 cm, critical angle 30.5° → small except near nadir; larger on mud [n03] | S3 |
| Impedance contrast per material (`_ACOUSTIC`, Selfridge) | I | physical constants, NC #17 | S4 |
| Coherent / incoherent split by the Eckart fraction | I | σ = 1e-5 m painted floor: C ≈ 1; sand 5 mm at 4°: C ≈ 0.18 [n05] | S5 |
| Kirchhoff/PO patches for smooth bodies; glints, aspect, edges, layover emerge | I | `facets.po_*`, smoke `[14]` closed forms | O1–O3 |
| Corner paths (object–seabed, object–wall dihedral, trihedral) | I | Wil10; strong on smooth floors at low grazing | O4, O5 |
| Transmission through thin walls (layer formula per crossing angle), flooded interiors, nested contents | I | `pp_shell`'s measured 0.11 becomes a validation number | O6 |
| Elastic shell waves (Lamb/coincidence) | X (v1) | PP/PE shear speed < water's → no shear coincidence; confirm with FL77 [n05] | — |
| Sea surface: flat, `R₀ = −1`, Eckart coherent, incoherent Kirchhoff remainder | I | calm pool C 0.94–0.99; Hs 0.1 m C = 0 [n08] | P2, S6 |
| Wave-modulated surface geometry from `sea/` | O | specular points move with η(x, y, t); needs `/sim/sea_state` in the node | P-wave |
| Surface backscatter (Bragg/bubbles) | O | the beam's upper −3 dB edge is 5° above horizontal; calm ≈ 0; law pending McD93 | — |
| Seagrass as a volume scatterer | O | no primary scattering source yet [n03] | — |

### Shadow and diffraction

| Effect | Decision | Number / justification | Test |
|---|---|---|---|
| Diffraction at shadow edges: ray-traced Fresnel–Kirchhoff per leg | I | Fresnel radius 0.13 / 0.18 / 0.26 m at 5 / 10 / 20 m vs 0.1–1 m objects; pass 4: shadows 3.4 / 4.6 dB too dark | D1, D2 |
| Shadow floor from physics only (sidelobes, multipath, volume, diffraction); `reverberation_db: 0` | I | the geometric 18 dB constant floor is not used; the corpus quay's 17–25 dB is the test | D3 |
| BTM / secondary-source edge diffraction (SFV99 eqs. 1–5, 16, 19, read 2026-11-03) | O | the reference for D1 at large diffraction angles; replaces Fresnel–Kirchhoff where the two differ by > 0.5 dB | D1 |

### Water column

| Effect | Decision | Number / justification | Test |
|---|---|---|---|
| Absorption, Francois–Garrison per site (T, S, pH, z) | I | every corpus site is seawater (owner, 2026-11-03): 0.163–0.177 dB/m at the per-site OISST temperatures (26–28.5 °C, S 35); ±0.6 dB two-way at 20 m over T ± 3 °C. The geometric renderer's 0.10 is 2.5–3 dB two-way low at 20 m [n07] | A1 |
| True sound speed per site vs the device's assumed 1500 m/s (bins) | I | `sos_dmps` = 15000 in every corpus file; true c 1537–1545 m/s, so real images are range-compressed by 2.5–3.0 % (±0.6 %), 25–30 cm at 10 m. The geometric renderer uses 1500 for both [n07] | R1 |
| Spherical spreading per leg | I | exact for straight legs | A1 |
| Volume scattering `S_v` (particles, plankton), uniform per site | I | the water column before the FBR (corpus 25.3 dB reported); `S_v` fitted in bounds once a source exists | W1 |
| Boat wake / thruster bubbles | O | the ping plane is forward of the thrusters; matters in turns only; needs TVF94 and a corpus check | — |
| Suspended-sediment attenuation | O | Ric96; off without a turbidity measurement | — |
| Thermal noise before TVG (−15 + 20 log f; 82 dB re µPa² in band) | I | the reported floor rises with range [n09] | N1 |
| Thruster cavitation noise | X | no data; thrusters are behind the receive aperture's main lobe | — |

### System and platform

| Effect | Decision | Number / justification | Test |
|---|---|---|---|
| Absolute sonar equation; `SL + RS` → one fitted system gain | I | the Omniscan's `SL` and `RS` are unpublished [n09] | §8 |
| Near-field line aperture, `L` = 24.6 cm (datasheet two-way 0.5° + far field 15 m), bounded by the CAD housing (318 mm); apodisation uniform until measured | I | `2L²/λ` = 36 m > the 20 m swath; the footprint at 2–10 m is the aperture, not `Rθ` | B1, B2 |
| Vertical pattern: 3.5 mm aperture (50° one-way), tilt 20° | I | as the geometric profile (installation) | B3 |
| Coherent phasor sum → speckle, correlation; K only from scene content | I | point scatterers with a **complex circular Gaussian factor** (Brown17 eq. 2–3), so the field is Gaussian at any density; AL02 / LA99: K-like tails come from scatterers or **patches** per cell (two-component mixtures at 80 kHz) [n04] | C1–C3 |
| TVG law: `k log₁₀ r + 2αr` partially compensated, `k` fitted in [0, 40] | I | undocumented; the geometric fit says partial (0.30) | §8 |
| Gain ladder 0–7, auto → 4, receiver ceiling 67.9 dB, 16-bit per-ping normalisation | I | shared encoder, unchanged | E1 |
| Transmit ringing (55 dB, 22 dB/m) | I | measured device leakage; shared config term | E1 |
| Port/starboard crosstalk; interference between the two same-frequency units | **X** | X1, 2026-11-03: SonarView files carry assigned sync channels 0/1, our driver's carry 255 (unsynchronised). No drifting cross-talk line ≥ **+6 dB** above the local level in 6 unsynchronised pool files (the injection-test detection limit); the transducers face away from each other behind a baffle. Residual: < +6 dB not excluded | X1 |
| Full attitude + heave at the ping time; lever arm rotated by R(roll, pitch, yaw) | I | main goal; `_sensor_pose` formula | M6, T1 |
| Separate receive pose per contribution (rotation and translation during the round trip) | I | yaw 0.5 rad/s → 0.76° in 26.7 ms vs the 0.5° two-way beam; 4 cm forward vs λ 3.3 mm | T2, T3 |
| Doppler as a separate term | X | 900 Hz = 4 % of the band at the 1.5 m/s upper bound; carried implicitly by the moving receiver | T3 |
| Nonlinear acoustics, shear in sand, caustics without a gradient, current effect on c, polarisation | X | 10 W device (linear), fluid sediments at these angles, no gradient by default, current ≪ c | — |

## 4. Equations (each module)

1. **Scene** (`scene_mesh.py`). Terrain: two triangles per raster cell of `depth.npz`, vertices at cell
   centres. Rough objects: the cells where `height − facet_bump − terrain > 0`, as a separate patch mesh
   with the object id from footprint ownership. Smooth solids: a watertight mesh per primitive. Walls:
   front, back, ends and top of the wet span. Per triangle: material id, roughness, kind, object id.
2. **Scatterers** (`scatterers.py`). Density `ρ_s` per m² (Phase 5). The cell index is a world lattice
   in the triangle's plane. Position and phase come from splitmix64 / Philox of (seed, tri, i, j).
   `ΔA = 1/ρ_s`.
3. **Scattering** (`scattering.py`). `σ_rough(θ_i, θ_s, φ)` from the SSA (JR07 §13.4, App. L.2) with a
   power-law spectrum truncated below `K_c`. `σ_vol` from eq. 14.8. Tables are precomputed per material
   on a (θ_i, θ_s, φ) grid and interpolated. PO from `facets.po_bistatic`. Flat-interface `V(θ)` from
   JR07 ch. 8 (fluid) or the `_ACOUSTIC` layer formula.
4. **Propagation** (`propagation.py`). Francois–Garrison α (JR07 B.6–B.12); Mackenzie / Bilaniuk–Wong
   c; optional constant-gradient arcs (Bell97a).
5. **Paths** (`paths.py`). Images in `z = 0` and in wall planes; floor image in the tangent plane at
   the current reflection point, iterated to stationarity of the path length (Fermat; ≤ 5 iterations,
   tolerance λ/20); visibility of each leg by the BVH; validity of the reflection point inside its
   face.
6. **Beam and motion** (`beam.py`, `motion.py`).
   - Transmit pose at t₀: the node's pose after `waterline_pose`, then the lever arm
     `[mount_x, ±mount_y, −sensor_depth]` rotated by `R(roll, pitch, yaw)`.
   - Receive pose for a contribution: `predict_pose(pose, v_body, w_body, τ)`, iterating `τ` twice.
   - Near-field directivity: `b(x) = (1/L) ∫ e^{ik(|x − u ê_a| − |x|)} du`, by Fresnel integrals.
7. **Receiver** (`receiver.py`). `s_mf(t)` = the CW pulse (44.3 µs floor) autocorrelation convolved with
   the receive filter (`filter_duration_percent` × the round-trip window). Thermal noise is added before
   TVG with the device's TVG law `G(r)`. The output is linear power on the scale the encoder expects
   (`calibration_db_offset` = the fitted system gain in the ray-cast profile).
8. **Diffraction** (`diffraction.py`). `U/U₀ = (−i/(λ d_eff)) ∬ V(ξ) e^{iπ|ξ|²/(λ d_eff)} d²ξ` over a
   disc of 3 Fresnel radii, stratified samples, after a ring test.
9. **Labels** (`labels.py`). Per object, from the rendered result:
   - **echo extent**: the bins where that object's contributors exceed the local diffuse mean;
   - **visible**: any such bin;
   - **shadow_bins**: the bins whose seabed scatterers' direct legs are blocked by that object;
   - **ghosts** via path type (`via: "surface"` / `"wall:<name>"`).
   The schema is identical to the geometric renderer's (NC #15: a label cannot contradict its picture).

## 5. Scene rules

* The mesh is built in memory on load (optional cache under `~/.cache/blueboat_sss_sim/raycast/` keyed
  by a hash of the world's `scene.npz`, `depth.npz` and `scene_manifest.yaml`). Nothing is ever written
  into a world folder (NC #10).
* A legacy bundle without `depth.npz`: terrain = `height − facet_bump`, with rough objects left in it
  and labelled from footprints. Logged once as a degraded mode.
* `Wall.reflectivity` (a per-wall energy fraction) is **not used**. Walls take the concrete impedance
  (`_ACOUSTIC`) and their own `roughness_m`, for the NC #17 reason: no brightness per object.
  `analysis.wall_contrast` on the corpus quay is the test.
* Outside the map: the scene's `outside` rule, as the geometric renderer, reported in
  `out_of_map_from_m`.

## 6. Ray-count / sampling convergence (Phase 5 metric, stated now)

The quantities to converge are:
1. scatterer density `ρ_s`;
2. Fresnel-disc samples for diffraction;
3. sub-rays per scatterer for partial occlusion of its cell;
4. PO patch size (already set by the Fresnel / curvature criterion of `facets`);
5. path order (§2.2).

The reference is each quantity at ≥ 16× (density) or ≥ 4× (others) the candidate.

* **Mean error:** `ε_μ(b) = |P_b − P_b^ref| / P_b^ref`, P = `Ping.power`.
* **Statistics (Brown17's three tests, replacing a same-seed realisation error).** Refining the
  lattice draws new world-fixed scatterers, so a realisation cannot converge across densities, by
  construction. What must converge, at each density against the reference density:
  1. the mean (`ε_μ` above);
  2. the per-bin intensity PDF over homogeneous patches: KS vs Exp(1) below the 5 % critical value;
  3. the range and track lag-1–3 correlations, equal to the reference's within their sampling error.
* **Physical variability:** single-look speckle (`std/mean = 1` per bin), and the measured per-ping
  level std (0.9–1.5 dB, i.e. 0.21–0.35 in linear relative terms) for block statistics.
* **Acceptance:** at every range decile, aspect and scene type:
  - the 95th percentile of `ε_μ` ≤ 0.02 (0.09 dB, ≤ 0.1 × the smallest per-ping level std);
  - tests 2 and 3 pass.
  - Worst cases are re-run: grazing (≥ 15 m on flat sand), shadow edges, broadside glints, walls at
    30–95° incidence, the pool geometry.
  - Counts may depend on range and scene type, never on speed.

## 7. Validation ladder (tolerances stated before the work)

| Code | Test | Tolerance |
|---|---|---|
| M1 | Mesh height vs `depth.npz` at cell centres | exact (vertices are the samples); bilinear interior ≤ ½ cell × local slope |
| M2 | Smooth-solid mesh vs `facets.object_patches` patch centres | ≤ 1 mm normal distance |
| M3 | Wall wet face vs `metadata.yaml` (ends, top, draft) | ≤ 1 mm |
| M4 | No triangle from limits; dry cells flagged silent | exact |
| M5 | Mesh area and volume vs primitive closed forms | ≤ 1 % |
| M6 | Transmit pose equals `GeometricRenderer.sensor_pose` | float64 round-off |
| A1 | Absorption + spreading of a single leg vs `20 log r + αr` | ≤ 1e-9 dB |
| R1 | Point target: range profile = `|s_mf|²` centred at `2r/c` | peak position ≤ 0.1 bin; shape ≤ 1 % |
| B1 | Near-field aperture pattern vs direct numerical aperture integral | ≤ 0.05 dB in the main lobe, ≤ 0.5 dB at sidelobes |
| B2 | Far-field limit (r ≫ 2L²/λ): `sinc⁴` two-way, 0.5° two-way width | ≤ 0.01° |
| B3 | Vertical pattern and tilt vs the analytic aperture | ≤ 0.05 dB |
| S1 | Flat infinite sand: bin level vs the analytic sonar-equation integral of `σ` over the annulus (beam, TL, `s_mf`) | ≤ 0.1 dB at every range |
| S2 | Bistatic law reciprocity `σ(i, s) = σ(s, i)`; SSA vs composite on Table 13.1 sand | 1e-6; ≤ 1 dB above 10° |
| S-cut | Mesh-Nyquist cutoff: refining the mesh ×2 on a synthetic power-law terrain leaves the mean flat-sand level unchanged | ≤ 0.2 dB |
| S3 | Volume term vs eq. 14.8 | ≤ 1e-6 relative |
| S4/S5 | Specular reflection off a flat impedance interface vs `V(θ)√C` | ≤ 0.05 dB |
| S6 | Surface image amplitude vs `−√C` | ≤ 0.05 dB |
| P1 | Lloyd mirror: point target at (r, z_o) under a calm surface; fringe positions `r_n = 2 z_t z_o / ((n+½)λ)`, depth of nulls | ≤ 1 % position; nulls ≤ −20 dB |
| P2 | Ideal waveguide comb vs `waveguide.ideal_image_sum_closed` | ≤ 0.2 dB |
| P3 | Image validity: a wall image disappears when its reflection point leaves the face | exact (no ghost beyond a wall end) |
| P4 | Pool FBR ramp: nadir 0.6–0.7 m altitude in 0.9 m water → direct, mixed `(r_D + r_S)/2`, surface–surface arrivals | arrival positions ≤ 1 bin |
| P5 | Opaque object between boat and quay in 0.9 m calm water: comb tail behind the face with/without the object | the shadowed images vanish; difference reported (hand-off §1) |
| P6 | Sloping floor: comb vs the flat-floor comb | reported; the Fermat image ≤ λ/20 path error |
| P7 | B1: no object shadow beyond a quay face (wavy-wall session world) | the shadow energy behind the face ≤ the scene's floor + 1 dB |
| O1–O3 | Plate `k²A²/π`, cylinder `k a L²`, sphere `π a²` | ≤ 0.15 dB (as smoke `[14]`) |
| O4/O5 | Dihedral `8π a²b²/λ²`, trihedral `4π L⁴/(3λ²)` (to be text-checked in KST04) | ≤ 0.5 dB |
| O6 | Flooded `pp_shell` box: two-way shadow depth vs the measured 0.11 (−9.6 dB beach; −8.2 / −7.4 dB at 6.1 / 10.1 m) | reported; ≤ 2 dB is "agrees" |
| D1 | Knife edge vs `|F(v)|`, v ∈ [−2, 3] | ≤ 0.5 dB, ≤ 0.1 rad |
| D2 | Shadow-edge width ∝ `√(λR)` | log–log slope 0.5 ± 0.05 |
| D3 | Shadow floor 6–12 m behind the corpus quay vs 17–25 dB measured (`wall_contrast`) | inside the measured range |
| C1 | Homogeneous patch intensity PDF | KS vs Exp(1) ≤ sampling floor |
| C2 | Range lag-1/2/3 correlation vs the corpus | inside the corpus ranges (0.3–0.7 / 0.10–0.17 / 0.03–0.10) |
| C3 | Track lag-1 vs the corpus | inside 0.3–0.7 |
| N1 | Noise-only ping (no scene) after TVG | the floor slope equals `G(r)` |
| E1 | Encoder round trip (`parse_frame`), one full-scale bin, span ≤ 90 dB | exact |
| T1 | Rigid invariance: rotate and translate scene + boat together | float64 round-off (CPU) |
| T2 | `[10]`-type analytic roll/pitch/yaw responses | as smoke `[10]` bounds |
| T3 | Yaw-rate test: 0.5 rad/s, point target at 20 m, receive-beam misalignment loss vs analytic `b_t(0) b_r(Δψ)` | ≤ 0.1 dB |
| X1 | Crosstalk check on the corpus (measurement, Phase 7) | reported |
| NC4 | First-bottom-return envelope, `calibration.seabed.fbr_envelope` | must hold (NC #4) |
| G1 | GPU = CPU on every scene above (same scatterers, same hashes) | mean `ε_μ` ≤ 1e-4, same-seed field `|E_GPU − E_CPU|²/P` ≤ 1e-3, labels identical, leak probe 0 |

## 8. Parameters fitted in Phase 7 (each with literature bounds)

| Parameter | Bounds | Source of the bounds |
|---|---|---|
| system gain `G_sys` (`SL + RS` + receiver constants), dB | free within ±20 dB of the datasheet-free nominal; the fit must land inside | none published; reported as the identified value |
| TVG range coefficient `k` | [0, 40] (none → full spherical two-way) | device partial compensation (geometric fit 0.30 × 40) |
| roughness spectrum strength `w₂` at the mesh Nyquist, per sediment | JR07 Table 13.1 sand envelope (3.6e-7 … 1.5e-4) | JR07 |
| spectral exponent `γ₂` | [1.9, 3.9] | JR07 Table 13.1 |
| volume parameter `σ₂` | [1e-4, 4e-3] | JR07 eq. 14.9 |
| water-column `S_v`, per site | **bounds pending a primary source** (INDEX item 9) | — |
| aperture length `L` | [22.4, 31.0] cm (default 24.6) | datasheet far field (15 m) / CAD housing 318 mm (owner 2026-11-03) |
| aperture apodisation | uniform unless the corpus near-range footprint refutes it | datasheet |
| vertical large-angle response (nadir) | bounded level, to be set in the NC #4 step | none published; housing edges diffract |

Everything else is physical and fixed: impedances, F&G absorption at the site's T/S, PO, Eckart, the
installation (0.22 m, −0.345 m, tilt 20°), the 24 cm aperture.

**Split.** Whole sessions, from `~/calib/corpus_manifest.yaml`, written in `raycast_plan.md` §Phase 7
before any ray-cast render of corpus data. The hold-out named there is never loaded before the final
test. The spent pool TEST runs are never loaded. NC #4 holds for every fitted set.

## 9. GPU choice (Phase 4 method)

Candidates go in `~/ros2_ws/sim_runs/raycast/venv`: Warp (`wp.Mesh`), Numba-CUDA (own BVH traversal),
embreex (CPU). PyTorch only if both GPU candidates fail (disk 17 GB free). The benchmark records, on
the pool, harbour and wavy-wall worlds:
- closest-hit and any-hit rays per second;
- BVH build time;
- peak device memory;
- end-to-end ms per ping-side at the Phase 5 counts.

The decision table goes into `raycast_phase4.md`.

Patterns adopted from DAVE's multibeam sonar (Choi et al. 2021, doi:10.3389/frobt.2021.706646; code
`IOES-Lab/dave`, Apache-2.0; re-implemented, credited in `raycast_dave_comparison.md` §7):
a deterministic two-stage accumulation (per contributor-block writes, then a fixed-order reduction per
bin) in place of atomics, persistent device buffers across pings, and frequency-domain synthesis for the
coherent contributors (optional, Phase 5).

## 10. Determinism

CPU: same seed → bit-identical image (numpy, counter-based hashes). GPU: identical contributor sets
and phases (integer hashes). Sums are float64, or phases referenced to the bin range. With DAVE's
two-stage reduction (no atomics; Choi et al. 2021, see §9) GPU results are bitwise repeatable run to
run and equal to CPU within G1 (summation order differs from numpy's); documented, not hidden.

## 11. Integration contract (Phase 6)

* Profile `config/raycast_default_sonar.yaml`: top-level `renderer: raycast`; `acquisition:` = the real
  driver's; `model:` only for the shared encoder and noise keys, with `calibration_db_offset` = `G_sys`
  and `reverberation_db: 0`; `raycast:` strict section (site, aperture, densities, path order, switches).
  `SonarConfig.from_yaml` reads only `acquisition`/`model` (verified in `sonar/config.py`), so it is
  unchanged.
* Node: parameter `renderer` (`""` → the profile key → `geometric`), re-read on every enable; optional
  `motion=(v_body, w_body)`; the GPU stack is imported lazily, only when selected. The geometric path
  is byte-identical.
* Launch `sonar_renderer:=`; GCS selector (owner decision 2026-11-02, both paths).
* Live runs: a slower renderer lowers the ping rate (the timer cannot catch up). **Slow-down mode built
  2026-11-07 (owner order, superseding "none is built"):** `full_mission_launch.py real_time_factor:=`
  (0.01-1.0) runs the whole simulation on one clock below real time (`gz sim -z`); the control
  nodes take `time_source: ros`, the GCS processor `use_sim_time`, Perception's label wait the node
  clock, the GCS and MCS a sim-paced station clock, so a slowed run pings at the profile's period in
  sim time and records what a real-speed one does. MCS: Sonar model + Slow-down rows. The node logs
  the achieved sim-time rate and the factor that would hold the period when it falls behind.
  Implemented: node `renderer` / `raycast_backend`, launch `sonar_renderer:=`. Still open: the GCS
  replay-path selector (`sonar_source.py`).
* Profiles. MCS **Extra args** `sonar_config:=` picks a ray-cast profile other than the packaged one,
  for example the harbour hybrid (`raycast_assessment.md` §13). The node builds the correction layer from
  the profile's `raycast.correction` and checks the artifact's core fingerprint.
* Range. The GPU working set grows with the range: on an 8 GB GPU in the harbour world it peaks at
  2.3 GB at 20 m and 6.3 GB at 40 m, and runs out of memory at 50 m (CUDA error 101,
  `raycast_assessment.md` §13.7). The advice is ≤ 30 m (module README). This applies to the GCS's range
  control too, which re-ranges the node.
* Verified end to end on 2026-11-07, harbour 20 m at x0.03: the MCS `LaunchManager`, then the
  simulation, then the GCS App's START / Record and its seabed images (`integration_guide.md` §2b).

## 12. Needed measurements (none blocks Phases 1–6)

Updated 2026-11-03 with the owner's answers.
1. ~~Pool salinity and temperature~~: every site is seawater. Salinity unknown → 35 ± 3 ppt.
   Temperature unknown → per-site, per-date NOAA OISST estimates ± 3 °C (note 07). A thermometer
   reading at the next field session would replace them.
2. A temperature profile at any site (refraction).
3. ~~Transducer length~~: CAD housing 318 mm (owner), consistent with L = 24.6 cm, bounded [22.4, 31.0] cm.
4. Two-unit interference / crosstalk: owner: "check the docs and data yourself". Our driver never
   sends the sync messages (169/170), so our recordings are unsynchronised. Investigation in the
   Phase-2 report (X1).
5. The transmit waveform (CW vs FM): owner does not know. Inferred from the recordings' range
   correlation and point-target range profiles; CW is the provisional assumption.

## 13. Session-2 decisions (2026-11-03)

- **Water:** every corpus site is seawater (owner). Per-site NOAA OISST temperatures ± 3 °C, S 35 ± 3
  ppt; the default is the sites' mean, 27.3 °C (note 07).
- **Range scale:** true c for delays, the device's 1500 m/s for bins (`sos_dmps` = 15000 in the
  corpus): real images are range-compressed 2.5–3.0 %.
- **Aperture:** a physical length derived at the nominal 1500 m/s (24.6 cm), radiated at the site's
  k; bounded [22.4, 31.0] cm by the datasheet far field and the CAD housing (318 mm).
- **Receiver:**
  - CW, with a receive filter of 1.4 × τ (measured, X2);
  - device TVG `k log₁₀ r`, k ∈ [0, 40] fitted; provisional k = 12, the geometric fit's 30 %
    compensation;
  - transmit ringing: the measured shared term, in the steady channel.
- **Scatterers:** Brown17's complex circular Gaussian factor; the mean uses E|f|² = 1. The
  convergence metric is on the mean and the statistics (§6).
- **Large-angle vertical response:** `b_v = √(sinc² + f_v²)`, f_v ∈ [0, 0.3].
  - NC #4 step finding: f_v is **not** the bottleneck of the near-nadir level. On bath3 (sand, 1.37 m)
    the ray caster's first-return onset is 8–12 dB under the corpus (re the 2–4 m level) for any f_v
    in [0, 0.3].
  - The deficit is the seabed law at near-normal incidence: the SAX99 power-law spectrum extrapolated
    to 450 kHz gives a broad, weak near-specular lobe (J&R §6.5 caveat).
  - So the near-nadir profile constrains the 450 kHz roughness spectrum (w₂, γ₂, possibly an
    inner-scale roll-off), fitted in Phase 7 inside the J&R envelope.
  - The pool cannot constrain sand: its floor is painted concrete.
- **Cross-talk (X1):** excluded, detection limit +6 dB.

## 14. Session-3 decisions (2026-11-03): the bottom path and multipath

**The floor's mean field (`specular.py`).** The coherent reflection of the seabed sheet is rendered as
physical optics over the sheet's own triangles, not as an image construction:
- planar parallelogram patches, linear phase integrated exactly (`sinc`s of `facets.po_bistatic`);
- the Kirchhoff prefactor `i k/2π` in this renderer's `exp(−iωτ)` convention, so an infinite flat
  floor returns the image source `V/(2h)` in amplitude **and phase** (the smooth solids' PO now carries
  the same factor; magnitudes unchanged);
- `V(θ)` at each patch's own incidence angle (the tangent-plane rule; J&R 8.49 for sediments, the
  material's fluid half-space otherwise);
- `√C = exp(−½ Δk_z² σ_h²)`, `Δk_z = k|n·(out − inc)|` (Eckart, bistatic form);
- `σ_h` is **not a new constant**: the RMS of the same J&R power-law spectrum the diffuse law uses,
  above `K_low = max(π/r_F, π/Δx)`, `r_F = √(λR/2)` the two-way first Fresnel radius and `Δx` the
  sheet cell. The mesh carries every larger scale, so nothing is counted twice. Consequence: the
  coherent fraction depends on the world's raster resolution beyond `h ≈ 2Δx²/λ` (5.9 m at 0.1 m),
  a stated property of the composite split, not hidden;
- a smooth stationary-phase window (weight 1 inside 8 Fresnel zones, raised cosine to 0 at 30) in the
  bisector's tilt against the smoothed sheet normal, **anisotropic** for oblique specular geometry
  (a bistatic zone is 1/sin ψ wider across the plane of incidence). Numerical settings, converged in
  Phase 5;
- known approximation: the carrier-frequency Kirchhoff kernel omits the envelope-derivative term
  (≤ 1.2 % amplitude at the envelope flanks, 0 at the peak; S4/S5 residual phase 0.018 rad).

**Finding.** With the literature spectra (SAX99 sand, Key West mud) extrapolated to 450 kHz, the nadir
mean field is zero (C < 1e-13 at every altitude): the specular energy is entirely in the incoherent
near-specular lobe, which the SSA already renders (broad and weak for SAX99). A coherent first bottom
return exists only for `w₂ ≲ SAX99/30`, still inside the J&R sand envelope. So the 450 kHz spectrum
(`w₂`, `γ₂`) decides NC #4 for this sensor; it is fitted in Phase 7 (bounds below) with NC #4 held.

**Fitted-parameter bounds made concrete** (J&R Table 13.1 read 2026-11-03, `seabed_ssa.OVERRIDE_BOUNDS`;
profile key `raycast.seabed.sediments`):

| Sediment | w₂ | γ₂ | σ₂ |
|---|---|---|---|
| sand | [3.56e-7, 1.475e-4] (Panama City 84 … Panama City 93) | (2, 3.92] (SSA needs α ∈ (0, 1)) | [1e-4, 4e-3] (eq. 14.9) |
| mud | [3.56e-7, 4.1e-3] (the whole table) | (2, 3.92] | [1e-4, 4e-3] |

**Eigenray chains (`chains.py`).** The image method over the surface (exact plane), the seabed sheet
and planar faces (walls). Floor bounces are the stationary points of the path length over the sheet,
found by a damped Newton solve (analytic gradient with the smoothed slope, finite-difference Hessian;
the fixed-point tangent iteration oscillated on curved floors). One eigenray per chain: on a floor
with several specular points for one (P, X) the solve returns the one nearest the flat-floor image
point (a stated limitation; 20 of 40 chains valid on a 0.6 m / 9 m wavy floor, all 40 on a sloping
one). The near-field aperture pattern of a chain is evaluated at its departure direction **times its
unfolded length** (the image transducer radiates toward X; at the unit vector it is the 1 m pattern —
a bug found while integrating).

**Multipath (`multipath.py`).** Every contributor through every pair of kept chains; a diffuse
scatterer's pairs share its world-fixed factor (one `add_group` call per contributor chunk, so the
pairs interfere in the mean: Lloyd, comb). Sequences are kept by the §2.2 energy bound, evaluated on
geometry before any visibility ray. No physical path is counted twice:
- a chain whose last bounce is the contributor's own surface is degenerate and not built;
- the floor's mean field is the PO at the **first** floor bounce in time: out chains carry only the
  surface image, back chains any surface/floor alternation.
The incoherent share `1 − C` of each specular bounce (forward scatter, double scattering) is not
rendered (optional, off), for the floor as for the surface. The surface RMS default is the geometric
profile's judged open-water 0.02 m (surface chains then fall below the cut-off); a calm site is a
record (pool 3e-4 m, pass 4).

## 15. Fast-variant engine paths (2026-11-10)

Five knobs (`config.ENGINE_KNOBS`) whose defaults reproduce the renderer above exactly. Only the generated
`raycast_fast_sonar.yaml` turns them on, and a correction artifact's core fingerprint leaves them out at their
default. Spec, as-built notes and timings: `raycast_fast_optimize_plan.md`.

- `sampling.cull_unreachable` drops scatterers no sound can reach, on the CPU and the GPU alike:
  - above the still waterline;
  - wall points buried below the seabed;
  - quay faces turned away from the transducer.
- `paths.reselect_every` / `reselect_move_m` reuse a side's echo-path selection across pings. The kept
  chains are still evaluated every ping. GPU only.
- `diffraction.method: edge` uses the closed-form half-plane F(v) of the nearest free straight edge of a
  wall box, with the same per-leg ratio, window and two-way product as §14's sampled disc. Objects and relief
  get no factor.
- `precision: fp32` runs the float32 watertight visibility test at `seg_blocked`. GPU only.

