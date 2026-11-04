# Ray-cast sensor — plan, phase gates, data split

Owner brief: 2026-11-02 ("A second simulated sensor: full sonar ray-casting, next to the existing
renderer"). Design: `raycast_design.md`. Literature: `docs/literature/raycast_sonar/INDEX.md`.
Per-phase reports: `raycast_phaseN.md`. Experiments: `~/ros2_ws/sim_runs/raycast/`. Calibration
records: `~/calib/raycast/`.

**Each phase's exit gate is written here before the phase starts, and a phase starts only when the
previous gate is met or waived by the owner.** Status lines are appended, never rewritten.

## Rules for the whole project

* `GeometricRenderer` and everything it uses keep their behaviour. The default path stays
  byte-identical (render fixture) and no slower (timed default render before/after Phase 6).
* New code lives in `blueboat_sss_sim/sonar/raycast/` (ROS-free, Qt-free). It is imported only when
  the ray-cast renderer is selected, so the GPU stack is never needed for `geometric`.
* Interface fidelity is absolute (NC #1–#3, CM-1/CM-2): same topics, types, QoS, parameters, frame
  bytes, same-tick sides, ground-truth schema.
* Constants are physical and cited (NC #17), registered in `config/parameter_registry.yaml` (smoke
  `[16]`). The fitted ones are listed in `raycast_design.md` §8, with bounds.
* No test, scene or ray budget is shrunk to fit this PC. **On a hardware limit: stop, report what
  limits and what is needed, wait.**
* Installs: experiment stack only in `~/ros2_ws/sim_runs/raycast/venv`. At Phase 6, only the
  benchmarked GPU library, pinned, goes into `~/ros2_ws/.venv` (owner pre-approval 2026-11-02), with the
  command recorded in `raycast_phase6.md`.
* No `git commit` / `git push`; BlueBoat-Control and BlueBoat-MCS untouched. In BlueBoat-SSS only the
  GCS renderer choice (owner exception to CM-3).

## Owner decisions (2026-11-02)

1. GCS choice reaches both paths:
   - `sonar_renderer:=` when the GCS starts the simulator (bag replay);
   - otherwise, at START with *Sonar: Simulation*, the GCS sets the sim-only `renderer` parameter on
     the running `/side_scan_sonar` before ping enable. The node re-reads it on enable.
2. The hold-out is named by Claude in Phase 0 (below).
3. The GPU library may go into `~/ros2_ws/.venv` at Phase 6 (pre-approved, pinned, recorded).

## Data split for Phase 7 (written 2026-11-02, before any ray-cast render of corpus data)

From `~/calib/corpus_manifest.yaml` (whole sessions; each session's recordings stay together). Only
**real** recordings are calibration targets; `SSS_data/sessions/` (simulated GCS sessions) is
comparison material only.

| Role for the ray-cast sensor | Sessions | Recordings |
|---|---|---|
| **Calibration (fit)** | `beach_A-2026-09-07-B1` (bath3 = NC #6 anchor); `harbour_A-2026-07-22-PM-cal` (reflectionEvidenceFullharbour = NC #6 anchor); `pool_A-2026-10-28-1w-early` (= pass-3 FIT) | `beach_sept/bath{1,2,3}.svlog` (merged files excluded, as in the manifest); `reflectionEvidenceFullharbour.svlog`; the 7 pass-3 FIT pool runs |
| **Validation (never fitted)** | `harbour_A-2026-07-21`, `harbour_A-2026-07-22-AM`, `harbour_A-2026-07-22-PM-diffsv`, `pool_A-2026-10-27-SV`, `pool_A-2026-10-28-1w-line` (= pass-3 VAL) | `diffDepthCompensation`, `toOptimize-harbourScan`, `fullHarbourCleanExample`, `DiffSV-US_whenDepthCompensation`, `pool_mapping{1,2}`, the pass-3 VAL line runs |
| **New hold-out (TEST, opened once, at the end of Phase 7)** | `harbour_A-2026-07-23`; `beach_A-2026-09-07-B2` | `OneLineHarbourClean70m.svlog`, `MainReflectionEvidence+misspingsWaterfall.svlog`; all nine `beach_sept/object*.svlog` |
| **Spent, never loaded** | `pool_A-2026-10-28-2w` | `2weights_auv`, `2weights_auv_2` |

Notes:
- The hold-out is **within-site**: harbour and beach sites also appear in calibration, and passes 1–4
  opened these files with the geometric renderer. It is new only for the ray-cast sensor, which must
  not render, score or reduce it before the final test. Enforced by a refusal list in the ray-cast
  tools, as `calibration/io_guard.py` does.
- Phase 7 never uses a documented number that was measured on a hold-out file as a fitting or
  validation target. Two cases:
  - O6, the `pp_shell` shadow, was measured on `object_boiteVsDistance` (B2). It moves to the final
    test, and the pool box (FIT/VAL) is the development check.
  - D3, the quay shadow floor of 17–25 dB, is re-measured on validation harbour sessions only.

## Phases and exit gates

Tolerances: `raycast_design.md` §7 (codes in brackets).

### Phase 0 — literature + design (no code)
* *Deliverables:* `docs/literature/raycast_sonar/` (INDEX + notes 01–10), `raycast_design.md`, this
  plan.
* *Gate:* every inventory item settled with a number or marked "needs measurement"; every DOI verified
  with date and method; the list of wanted full texts sent to the owner.

### Phase 1 — scene and mesh export
* *Build:* `sonar/raycast/scene_mesh.py`, `config.py` (skeleton), `bvh_cpu.py` (numpy BVH +
  Möller–Trumbore), `test/raycast_test.py` with the M-tests.
* *Gate:*
  - [M1–M5] on every world under `~/worlds/` (pool `pool_*_1`, `plage_bathymetry2/*`, `arob2027/*`,
    `sea_validation/*`, `Shiraishijima_lawnmower/*`) and on a synthetic world exercising every catalog
    primitive, rested and flat;
  - BVH closest-hit equals brute force (every ray × every triangle) on 10⁵ random rays per world,
    over random 2.5 m crops that hold every smooth object and wall (exact hit ids, distances ≤ 1e-9 m).
    *Amended 2026-11-02, before the run:* brute force over the full 1–3 M triangles is 10¹¹ tests per
    world; crops keep it exact and affordable. The full mesh is checked by hit-on-triangle plus
    no-closer-occluder;
  - no ray leaks through a closed solid (transversal probes through every vertex and edge midpoint);
  - nothing written into any world folder (mtime check);
  - smoke test and Stop hook green.

### Phase 2 — CPU reference renderer, path by path
* *Order:* direct (beam, TOF, TL, `s_mf`, coherent sum) → surface → bottom → wall → dihedral → comb →
  diffraction → transmission/internal reflection.
* *Gate per path, before the next:*
  - direct: [A1, R1, B1–B3, S1–S5, O1–O3, C1];
  - surface: [S6, P1];
  - bottom and comb: [P2, P4, P6];
  - wall: [P3, P5, P7];
  - dihedral: [O4, O5];
  - diffraction: [D1, D2];
  - transmission: [O6 vs the pool box, reported].
  Then NC #4 [NC4]. Each failure is recorded with its number in `raycast_phase2.md`.

### Phase 3 — full 6-DoF motion
* *Gate:*
  - [M6, T1, T2, T3];
  - a recorded Gazebo wave-run pose track (`~/ros2_ws/sim_runs/sea_validation/` bags, else
    `test.gazebo_sea_test --quick`) replays with no discontinuity beyond the pose's own (ping-to-ping
    level steps explained by the pose);
  - the comparison with the geometric renderer on that track is reported effect by effect: intra-ping
    rotation, receive pose, fan tilt.

### Phase 4 — GPU path
* *Gate:*
  - [G1] on every Phase 1–3 validation scene;
  - the Phase-1 leak probe on the GPU intersection (whether Warp's `mesh_query_ray` is watertight is
    not documented; added 2026-11-02 after Phase 1 found Möller–Trumbore leaking);
  - the benchmark table (rays/s, BVH build, memory, ms per ping-side) on the pool, harbour and
    wavy-wall worlds.
  If the RTX 4060 (8 GB) or the 14 GB RAM cannot run them: **stop and report**.

### Phase 5 — sampling convergence
* *Gate:* the `raycast_design.md` §6 acceptance at every range decile, aspect and scene type, worst
  cases included. The chosen counts and their evidence go in `raycast_phase5.md`.

### Phase 6 — integration
* *Gate:*
  - node / launch / GCS working;
  - [E1] on live output;
  - labels read off the result;
  - smoke (system 493 / venv 490 + SKIP), GUI smoke, Stop hook green;
  - render fixture bit-identical, and the timed default render no slower (median of 20, within noise);
  - one recorded GCS session per choice, with the same topics, types and QoS downstream (`ros2 topic
    info -v`), the processor and GCS unmodified.

### Phase 7 — fit / validate / test on real runs
* *Gate:*
  - pre-registered gates in `~/calib/raycast/gates.yaml` (written before the first fit);
  - fits on calibration sessions only, inside bounds;
  - validation scores reported on every validation run-side;
  - NC #4 held;
  - the hold-out opened once, its scores reported whatever they are.

### Phase 8 — comparison and report
* *Gate:*
  - `analysis/raycast_compare.py` run on every case listed in the brief and in
    `raycast_handoff_from_pass4.md`;
  - `raycast_vs_geometric.md`, with the effects table (include / exclude / measured influence).

## Status log

* 2026-11-02 — Phase 0 deliverables written: the corpus (10 notes, ~75 verified references, 10 full
  texts wanted), the design note and this plan with the split. Owner informed of the wanted texts.
* 2026-11-02 — **Phase 1 gate met.** 131 checks passed + 2 re-run after a test fix, 0 failed, on 13
  worlds + a synthetic world. M2 worst 0.235 mm; BVH = brute force on 10⁵ rays per world; 0 leaks.
  Fixed on the way: watertight intersection (Möller–Trumbore leaked), empty-box traversal, box
  padding. Finding: re-stamping saved rested objects is not idempotent (existing code, unchanged).
  Report: `raycast_phase1.md`. Smoke 493/493.
* 2026-11-02 — **Phase 2 in progress.** Direct path built and gated:
  - A1, R1, B1–B3, S1 (median 0.004 dB, max 0.057 dB), C1, O1–O3, M6 pass;
  - S2–S4 pass (the J&R small-slope seabed law with the volume term, now the default).
  Open in this phase:
  - the NC #4 vertical-pattern question (nadir near the uniform aperture's first null);
  - the bounce paths, diffraction, transmission, labels.
  Quick suite 43/0, smoke 493/493. Report: `raycast_phase2.md`.
* 2026-11-03 — **Phase 2 continues (session 2).**
  - Owner answers applied: seawater everywhere; per-site OISST T; aperture bounded by the CAD
    housing.
  - X1 cross-talk excluded (+6 dB limit). X2: CW with a 1.4 × τ receive filter (measured).
  - The owner's full texts read: Brown17 Gaussian scatterers; the convergence metric is statistics.
  - S6 and P1 (Lloyd) pass.
  - NC #4: a units bug found and fixed (`shared_stage_model`). Still NOT met at high altitude; the
    cause is the missing coherent floor channel (the bottom path), next.
  - Finding: real images are range-compressed 2.5–3 % (`sos_dmps` 1500 vs c ≈ 1540).

* 2026-11-03 — **Session 3: gates written before the bottom path's coherent floor channel.**
  - Model (design §14): the mean (coherent) field of the seabed sheet near its specular points, as
    physical-optics patches (the Kirchhoff prefactor `i k/2π`, so a flat floor returns the image
    source `V/(2h)` in amplitude and phase), times the flat-interface `V(θ)` (J&R 8.49 for
    sediments) and the Eckart amplitude `√C = exp(−½ Δk_z² σ_h²)`. `σ_h` is the RMS of the same J&R
    power-law spectrum above `K_low = max(π/r_F, π/Δx)`: the two-way first Fresnel radius
    `r_F = √(λR/2)` and the mesh Nyquist (the mesh carries the larger scales; never counted twice).
    No new constant. The patches are weighted by a smooth stationary-phase window in Fresnel zones
    (a numerical setting, converged in Phase 5).
  - [S4/S5, monostatic] flat sand at altitude 0.6, 1.4, 5, 10, 20 m, isotropic unit beam: the
    coherent field vs the image source `V(90°)√C e^{−α·2h}/(2h)` deposited at `τ = 2h/c`, complex,
    bin by bin over the envelope: ≤ 0.05 dB at the peak bin, ≤ 0.05 rad in phase. Same with the
    floor RMS forced so that C = 0.1, 0.5, 0.9.
  - [S4/S5, oblique] the one-way floor-bounce eigenray (`paths.floor`) on a flat floor at five
    grazing angles: length = the image distance, factor `V(ψ)√C(ψ)`: ≤ 0.05 dB (exact geometry).
  - Then [NC4] rerun (`tools/nc4_detect.py`), reported against the geometric renderer.
* 2026-11-03 — **Session 3 results (Phase 2 continues).**
  - Bottom path built (`specular.py`, `chains.py`): **S4/S5 pass** (monostatic ≤ 0.014 dB / 0.018 rad at
    0.6–20 m, C 1…0.1; oblique bistatic PO ≤ 0.014 dB at 8–80°; floor eigenray exact to 1e-14).
    Two bugs found by the oblique gate and the integration: V evaluated at the bisector (−6.8 dB at
    30°), the near-field pattern of a chain evaluated at 1 m.
  - **P6 passes** (Fermat ≤ 0.005 mm vs λ/20 on a sloping floor; Newton replaces the oscillating
    fixed-point iteration). **P2 passes** (chains = image sum to 8e-16; −0.12 dB from the closed form at
    40 bounces). **P4 passes** (pool ramp arrivals within 0.02 bin; amplitudes within 0.16 dB, isotropic
    beam). **P3 passes** (0 mismatches). **P5 passes** (a sill removes the comb images it blocks:
    −7 to −9 dB behind the face).
  - Multipath renderer (`multipath.py`): grouped pairs, energy cut-off on geometry before visibility;
    flat sand + rough surface reduces exactly to the direct path; a calm surface adds +0.8…+4.9 dB at
    4–20 m (Lloyd).
  - Diffraction (`diffraction.py`): **D1 passes** (0.20 dB, 0.029 rad over v ∈ [−2, 3], 16 directions with
    edge-aware chord integration). **D2:** the pre-stated metric did not specify the width; the first
    one (crossing distance |U|² 0.25 → 0.05) gave slope **0.560 (outside 0.5 ± 0.05)** because it spans a
    finite stretch of the scene's nonlinear v(y) (the routine matches F(v) within 0.13–0.30 dB at 2 and
    20 m). The local width 0.25/|dI/dy| at the geometric boundary gives **0.486: pass**. Both recorded.
    Applied to the direct chain only (bounce chains keep boolean visibility).
  - **NC4 rerun** (`notes/nc4_detect_v3.log`): default spectra **10 regressions** (unchanged: no mean
    field); w₂ = SAX99/30: 2 (22 m, 40 m/600); **w₂ = SAX99/100 (4.33e-7, inside the J&R sand
    envelope): 0**. Not a regression by the criterion but worse: 0.6 m at 40 m/600 sand 0.15 vs the
    geometric 0.73 (both < 0.95). The provisional ray-cast profile will carry w₂ = literature/100
    (sand and mud), NC4-held, until the Phase-7 fit.
* 2026-11-03 — **Owner request: compare with the DAVE multibeam (Choi et al. 2021) before more code.**
  Done: `raycast_dave_comparison.md`. DAVE (`ros2` @ c1c2fba, Jetty) is a direct-path, first-hit,
  pixel-sampled multibeam model; portable are GPU patterns (deterministic two-stage reduction,
  persistent buffers) and frequency-domain synthesis for the coherent contributors. **Proposed** (owner
  decision pending, gates above unchanged): close Phase 2, then a first fit on the calibration sessions
  with a pre-registered go/no-go on the validation sessions (TEST hold-out stays sealed), before
  Phases 3–8. No ray-cast code written since the request.
* 2026-11-03 — **Owner approved the reorder** (close Phase 2 → first fit → go/no-go on the validation
  sessions; TEST hold-out sealed), with additions to the gates before freezing `gates.yaml`: absolute
  tolerances against real data per statistic; beach/harbour statistics (full-range mean intensity
  profile, speckle contrast + distribution fit, along-track correlation, object highlight/shadow
  contrast); "improves" = better than the bootstrap CI over pings, the others no worse; every fitted
  parameter reported against its literature range (w₂ flagged); CPU ms per ping-side + projected GPU
  time; a blind test of 20 mixed real/sim crops for two raters (reported at go/no-go, gated at the
  final). Done since: dead code removed (`ring_test`, `TriangleScene.areas`); survey of Stonefish and
  HoloOcean side-scan code/docs (`raycast_dave_comparison.md` §6); DAVE credited (§7, design §9/§10);
  ray-cast system gain + profile loader (`config.py`); **fit basis** (`fitbasis.py`) exact vs full
  renders (worst 0.0009 dB over 6 cases; 20 s to build per pose). Full `raycast_test` running
  (`~/ros2_ws/sim_runs/raycast/notes/s3/rt_full_s3.log`). `gates.yaml` NOT yet written; no corpus
  render yet.
* 2026-11-04 — **Gates frozen before any ray-cast corpus render:** `~/calib/raycast/gates.yaml`
  sha256 `609a806de11d1e2819171345662b9956e536493cfc3a8fe6babdbaf5a428d495` (private file, outside the repo). Contents:
  - absolute tolerances against real data for every statistic, set from real scales only
    (`~/calib/raycast/real_stats.json`: real values, no simulation);
  - "improves" defined by the bootstrap interval over pings;
  - the go rule; the parameter, compute and blind-test reports.
  Since the last line:
  - full suite under the multipath default **112 passed / 0 failed / 4 skipped** (only 6 world folders
    exist since 2026-11-03 22:20; the harbour and arob2027 worlds and the original pool worlds were
    removed outside this session);
  - ray-cast registry entries (21; registry 119, no drift);
  - `config/raycast_default_sonar.yaml` (generated);
  - an additive renderer hook in `calibration/seabed.py`;
  - `analysis/raycast_compare.py`.
* 2026-11-04 — **Owner: "make the computation GPU compatible"; check that less data would not give the
  same results; rebuild any out-of-date world that is needed.**
  - The CPU reference is the limit. Flat sand costs 9–11 s per ping-side; a pool ping-side with the
    calm-water comb was still unfinished at 44 min when stopped (lower bound, `notes/s4/cpu_timing.log`).
    The pool validation set would exceed the pre-registered 24 h compute line.
  - So Phase 4 is pulled forward, in front of the first fit's scoring:
    - `gpu_warp.py`: the visibility oracle (`WarpBVH`, a drop-in for the CPU BVH) and the near-field
      aperture integral (`near_pattern`);
    - `RaycastConfig.backend: cpu | warp`;
    - gate G1 in `test/raycast_test.py` (`gpu_checks`): the leak probe through the GPU oracle, and
      GPU = CPU renders, ε_μ ≤ 1e-4.
    The calibration fit bases (59 of 1512 built on the CPU) are kept.
  - Density convergence (§6 metric) is checked on the GPU before the fit: `tools/density_sweep.py`.
  - Worlds: the 6 remaining folders are acoustically current (no stale rasters). The harbour world
    (needed for the harbour quay statistic and the Phase-8 harbour cases) is the one to rebuild.
* 2026-11-04 — **GPU leak fixed; harbour world in; gates amendment 1 frozen.**
  - The GPU oracle leaked 375 of 22 049 edge-midpoint probes. Cause: NVRTC contracts the edge
    function `a·b − c·d` into an FMA, which is not antisymmetric. Both triangles of a shared edge
    then read the same sign (+1.7e-18 on a box diagonal), and a ray through the edge passes between
    them. Fix: `gpu_warp` is compiled with `fuse_fp: False`, so every operation is the IEEE one numpy
    performs. Hits and tie-breaks now equal the CPU reference on the failing case. A canonical
    vertex order with two expression sites did NOT fix it: each site fused its own way.
    A second cause left 56 leaks on a cone's base rim. The float64 test ran on coordinates shifted
    to the mesh's centre, which lose a sub-ulp offset that is representable near z = 0. It now runs
    on the global coordinates, with the float32 boxes alone in the shifted frame. The leak probe
    gives 0 of 22 049.
  - The owner built `~/worlds/Shiraishijima_lawnmower/Shiraishijima-empty`. The checks are tools
    outside the repo: `tools/harbour_coverage.py` and `tools/harbour_depth_check.py`.
    - Every non-hold-out harbour GPS fix lies inside the limits; six quays are within range of a
      track.
    - The bathymetry is a chart-datum survey. Its shape matches the sonar to a median 0.18–0.61 m
      once a per-recording offset is removed. That offset is the tide: 1.2–3.9 m, rising through the
      day.
    - The renderers put z = 0 at the run's still water. Each recording is therefore rendered in a
      derived world at its own water level. `tools/tide_worlds.py` builds it through the World
      Builder's headless path from the owner's `builder_state.yaml`: survey depth + h, quay tops
      unchanged. `Shiraishijima-wl3p85` and `-wl3p00` are built.
    - The rule for h and the measured values are in `~/calib/raycast/harbour_tide.yaml`. The
      hold-out's h is measured by the same rule at the final test.
  - **Gates amendment 1** (`~/calib/raycast/gates_amendment_1.yaml`, sha256
    `7c57fff820e5ee167807135ff7692de56d022e01db2c5f0abb3b347ab08fa592`, private).
    - `harbour_quay_face` becomes AVAILABLE, with the rule it lacked. It is the quay echo's excess
      over the seabed tail, with a ±3 m search for the GPS error. Two validation recordings carry it.
    - Its tolerance (3 dB) and its non-motivating role are unchanged.
    - It was written after its real values were measured (+8.36 dB and +3.23 dB) and before any
      simulated harbour render.
* 2026-11-04 — **Density convergence (§6) on the GPU: 5 mm stays.**
  - Flat sand at 1.4 m and 6 m altitude, spacings 2.5 / 5 / 7 / 10 / 20 mm (`raycast_phase4.md`).
  - 5 mm: ε_μ p95 = 0.008 / 0.006 (≤ 0.02); KS passes; range lag-1 0.50 at every spacing.
  - 10 mm fails the mean at both altitudes (0.035 / 0.023). 7 mm passes with no margin.
  - Cost: 2.5–3.3 s per flat ping-side on the GPU at 5 mm.
  - Multipath diffuse pairs folded by reciprocity, (i, j) + (j, i) → 2 (i, j): K(K+1)/2 pairs
    instead of K². `sigma_bistatic` is reciprocal to 1.2e-15 (2·10⁵ random geometries, three
    materials).
