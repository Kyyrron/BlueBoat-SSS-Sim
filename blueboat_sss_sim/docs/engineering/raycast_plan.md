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
* 2026-11-04 — **The whole per-contributor pipeline on the GPU; G1 passes; the pool validation fits
  in 24 h at full sampling** (`raycast_phase4.md`).
  - The CPU modules stay the reference, and `use_gpu_pipeline = False` is the comparison.
    `backend: warp` runs scatterers, direct, diffraction, selection, pairs, facets and the floor
    coherent channel on the device. Each stage is checked against the reference: the floor stage on
    flat, pool and calm 0.9 m scenes, pair by pair to order 4.
  - G1's first failure was a parameter inconsistency, not precision: the bounce-point lift was
    1 mm on the GPU and 0.1 mm on the CPU. It is now one registered constant
    (`raycast.paths.bounce_offset_m`). G1 then passed: quay + sill ε_μ 4.1e-6, flat 9.2e-7, leaks
    0 / 22 049.
  - BVH queries now stop at the segment's end. Hit decisions are unchanged; the proof is in the
    report. Per ping-side: flat 0.08 s, pool 0.63 s, harbour 2.46 s, against 4.2 / 968 / 15 s at
    the start of Phase 4.
  - Pool validation cost, 72 real poses: mean 0.99 s per ping-side, so ~4.2 h for p4's ~15 200
    ping-sides. The sampling is not reduced, and no gates amendment is needed.
  - Steep-cell refinement rule C was gated before adoption (slope sweep, ε_μ p95 ≤ 0.010 against
    0.02). `raycast.sampling.refine_stretch` is registered.
  - CPU reference fix: the floor coherent deposit now carries thin-shell leg transmission. It is a
    no-op without flooded shells, so the fit's flat scenes are unchanged.
  - The fit's memory: it ran out of RAM three times (12.4 GB). Bases are now held compact (power()
    bit for bit) and renderers are no longer rebuilt per evaluation.
* 2026-11-04 — **Gates amendment 2** (`~/calib/raycast/gates_amendment_2.yaml`, sha256
  `991b0036a111e4ab797c6542903724c141613b16a4729f9963be262905b38268`, private). The three pool
  statistics get what `gates.yaml` left open:
  - the session value: the median sim − real of the wall pings; the mean sim − real of the box passes;
  - the bootstrap unit: blocks of 8 wall pings per run-side, or box passes; paired across renderers;
    300 resamples, 0.95.
  - It was written before any ray-cast pool statistic was computed, and nothing frozen changes.
  - The scorer is `tools/pool_score.py`: p4_score's rules verbatim, plus readouts of the 0–1 m
    shadow band and the per-ping wall values.
  - NC #4's "sand and mud" for the ray-cast sensor: the envelope's dark bottom (reflectivity 0.22) is
    rendered as its mud sediment (`calibration.seabed._render_noisy`). The geometric path is unchanged.
* 2026-11-04 — **Gates amendment 3** (`~/calib/raycast/gates_amendment_3.yaml`, sha256
  `488776e036e32a58f0d6f88b72ad5ec375def5e72040ed2915b21a61f09bcb42`, private). It was written before any ray-cast harbour render, and nothing frozen changes.
  - `harbour_quay_face`'s pooled-verdict bootstrap: blocks of 8 selected pings per recording-side,
    paired across renderers.
  - The range kernel (range_corr_2 + range_corr_3, ONE motivating statistic in gates.yaml) is read
    conservatively. It improves only when both improve, and is worse when either is worse.
* 2026-11-04 — **Fit-basis key bug.** `calib._geometry_rc` removed the fitted keys but left their
  empty parent mappings. The fit's configuration (`set_params` adds `beam`) and the builder's
  therefore hashed the same flat scene to different keys, and every basis was built twice. Keys are
  now normalised (empty mappings pruned), and the existing bases were hard-linked to their
  normalised names (`tools/phaseA_migrate_keys.py`): all 2352 bases of the fit's first evaluation
  are present. The fit now holds one fit call's bases at a time (~11 GB); five out-of-memory kills
  are logged in `PROGRESS.md`.
* 2026-11-04 — **Gates amendment 4** (`~/calib/raycast/gates_amendment_4.yaml`, sha256
  `cf48cf383f912d18be5babefae1257a440962262dfdcac6ec2be55ec556d4df7`, private; the owner's decision the same day). The bounds of
  `raycast.seabed.sediments.sand.gamma2` move from [2.000001, 3.92] to **[2.6, 3.92]**.
  - The J&R law's I_K quadrature grows as (40/q)^(1/(γ2−2)):
    - γ2 2.6: 1.5 s per table;
    - γ2 2.5: ~1.3 GB per call;
    - γ2 2.4: 58 GB;
    - below that, not computable.
  - The fit had moved γ2 down and exhausted the host. Written before any fit result.
  - A fitted γ2 at the new bound fails `params_in_bounds` and is reported.
* 2026-11-05 — **Phase A attempt 1: NO-GO.** The fit did not converge, and every final parameter
  sits on a bound.
  - The diagnosis is in `raycast_phaseA.md`:
    - the nadir metric is degenerate, with zero sensitivity to the fitted parameters;
    - the default sea surface suppresses the shallow-water multipath.
  - Two bugs were found on the way:
    - NC4 compared the geometric envelope with itself, a false PASS;
    - `raycast_compare` passed the per-renderer detections to the point estimate.
* 2026-11-05 — **Gates amendment 5: attempt 2** (`~/calib/raycast/gates_amendment_5.yaml`, sha256
  `537952ecf9effa10b45a46d68a60e157c420e7b51bc1627334e8e3067298587a`; the owner's request). Written before any attempt-2 result.
  - The fit uses the level metric only, with a full-render multipath correction.
  - The sea-surface roughness is set per site: the pool at 3e-4 m (pass 4); the beach and harbour
    scanned on their calibration strata.
  - The pool gets its floor description, if it can be rendered within budget.
  - The validation sessions are now development evidence. The hold-out stays sealed.
  - If attempt 2 is still a NO-GO, a disclosed empirical correction layer follows.
* 2026-11-06 — **Attempt 2's validation is invalid: two engine defects** (`raycast_phaseA.md`). Both
  were found by diagnosis and fixed for the ray-cast only; the geometric path is unchanged and the
  smoke test passes.
  - The attitude replay applied the recording's altitude variation dz as the vehicle's z. This lifted
    the 0.22 m-deep transducer out of the water, and some pings came back empty. dz now moves the
    seabed.
  - The FBR altitude match fired on the far-range level of a sensor with no first-return crest and
    clamped strata to 0.2 m. It is now guarded.
  - New engine selector `level_crest`.
  - The probe and the feasibility fit show that calibration alone cannot hold the crest and the tail
    within the bounds.
* 2026-11-06 — **Gates amendment 6** (`~/calib/raycast/gates_amendment_6.yaml`, sha256
  `162871ff76b767fd52d4cff5b7b4fe510797d51a34205623c90e4c1a01519952`, private). It records the owner's
  decision: a physical near-normal contribution, with the primary literature verified first. It was
  written before any attempt-2b fit. No fit is run with the incomplete physics, and the correction
  layer is not used.
* 2026-11-06 — **Near-normal verification** (`raycast_nearnormal_verification.md`). Read NC94, NC98
  and JR07 §6.3, §6.5 and §13.1–13.5; reference checks R1–R5.
  - **The rough-facet term is NOT implemented.** It is a lower approximation of the full Kirchhoff
    result, which our SSA already reproduces: SSA/Kirchhoff 0.00 dB at 89°, and energy conserved to
    0.02 dB at 30 kHz. Adding it double-counts, and would add at most +2.6 dB.
  - The crest deficit (+9 to +23 dB in crest/tail ratio) comes from:
    - the vertical beam's large-angle response (the strip's first null at nadir, −55 dB two-way;
      the f_v ≤ 0.3 cap);
    - the 450 kHz small-scale spectrum (no coherent nadir return for any power law inside the
      bounds);
    - the TVG law.
  - Attempt 2's roughness set loses 32.8 dB of the reflected energy: it is outside the SSA's validity.
  - The water column (27.8 dB) is the shared measured constant, which does not carry over across
    acquisitions (real 16.7–29 dB on the harbour). It is not a ray-cast defect.
  - Owner decision pending.
* 2026-11-06 — **Owner redirect: a hybrid model; Stage 1 baseline; gates amendment 6a.**
  - The plan is `~/.claude/plans/bubbly-crafting-pixel.md`: preserve; one bounded beam + energy
    feasibility test; a project-wide reassessment; a physical core plus a separate correction layer.
  - The baseline archive is `~/calib/raycast/archive/2026-11-06_stage1/`: 181 files, manifest
    verified, untracked files included.
  - The hold-out audit is `~/calib/raycast/stage1/holdout_audit.md`. The ray-cast work has never
    touched the hold-out. It is not pristine, though: geometric passes 1–4 analysed it, `pp_shell`
    0.11 came from it, and the smoke test reads two of its files for decoder checks.
  - `raycast_test --quick` was stopped at 23 PASS / 0 FAIL; the remaining sections are being rerun.
  - **Gates amendment 6a** (`~/calib/raycast/gates_amendment_6a.yaml`, sha256 `c2bcc5cd4b769f681792d1c281d834f842c949bc87e53fca3dc5b852fdb11ae4`, private) was
    written before any Stage-2 computation. It records:
    - the verification outcome;
    - the owner's 11 amendments;
    - the Stage-2 procedure: one shared gain and TVG, f_v > 0.217 treated as a hypothesis, the energy
      validity check;
    - the decision rule and the stop rule.
* 2026-11-06 (late) — **Stage 2 complete: the 6a decision is FAIL** (`raycast_assessment.md` §4).
  - Test A (beam variants, seabed SAX99) and Test B (7 energy-valid seabed sets × f_v {0.217, 0.5})
    used full GPU renders, one shared gain and TVG, identical conditions, and beam and seabed swaps
    proven exact.
  - No admissible beam and no energy-valid seabed brings the harbour crests within 3 dB.
  - In every set the crest residual rises by about +11 dB from 5.7 to 16.5 m of altitude, whatever
    the beam and the seabed.
  - The whole γ2 = 2.6 row of the grid is energy-invalid, so the attempt-1 and attempt-2 solutions
    lay outside the SSA's domain.
  - The full `--quick` suite passes on the current code: 23 + 41 PASS, 0 FAIL, 2 SKIP (G1
    included). Smoke: 493 ALL PASS.
  - The correction-layer mechanism is built (`sonar/raycast/correction.py`, off by default, CL1–CL5).
  - Proposal (assessment §12): the hybrid, with gates amendment 6b written before the correction is
    fitted. Nothing fitted yet.
* 2026-11-07 — **Gates amendment 6b: the harbour-first hybrid** (`~/calib/raycast/gates_amendment_6b.yaml`,
  sha256 `66ee6c35c73fc2ae28c5d8a67ad8a91e383a40a766c9321926d9aa47b4e96c0a`, private). Written before any
  render, fit or score of the correction work. It records:
  - the owner's direction (harbour-first; shallow water a secondary limitation, never in the harbour fit)
    and decisions D1–D4: SAX99 + f_v 0.217 with gain/TVG refitted on the harbour calibration tails; the
    recorded harbour sessions' acquisition envelope, transfer failures = known limitations; enabled
    5.55–11.30 m true nadir altitude; seabed at true distance = device altitude × c/1500;
  - the data: one calibration recording (PM-cal, 16 strata, 40 m / 600 / 106.4 µs); validation PM-diffsv,
    07-21, AM (development evidence); groups 600-bin and 1200-bin;
  - the form (`u_logalt`, support u 0.85–1.35, linear in log10 alt; F1 2 or F2 6 coefficients chosen by
    leave-one-band-out CV) and an adequacy stop before validation;
  - the acceptance (P1–P5 must improve, S1–S8 must stay stable), the per-group outcome rule, and the
    out-of-domain behaviour (off and bit-identical outside the domain, with a status).
* 2026-11-07 — **Gates amendment 6c** (`~/calib/raycast/gates_amendment_6c.yaml`, sha256
  `2c12231ddb61bb2b786a121e23364bc301766935077bb868f6495aed909f3693`, private), after 6b's adequacy STOP and before
  any refit or validation score. 6b's STOP came only from the calibration bands at 14–16 m: three port-only strata
  beyond the 11.5–14.5 m data gap, with a +20 to +26 dB crest deficit, outside the enabled domain. The fit set is
  restricted to the 13 calibration strata at 5–11 m; everything else of 6b is unchanged (owner instruction quoted).
* 2026-11-07 — **Gates amendment 6d: correction v2** (`~/calib/raycast/gates_amendment_6d.yaml`, sha256 `c0a0666a74f91a720d7894d5b2994e6504a4e2549e90dc567ef029bb94984de0`,
  private). Written after v1's partial validation and before any v2 fit or score, on the owner's decision. Changes:
  - the support is widened to u 0.85–2.0 (the measured extent of the first-return excess);
  - the core gain and TVG are refitted on the tail beyond it (r ≥ 2.25 alt);
  - S1 is restated against real, S4 adds the new support end, and S5 uses the tail beyond the support.

  M1, P1–P5 and the outcome rule are unchanged. Every v2 validation result is **development evidence** only.
* 2026-11-07 — **Correction v1 and v2: final validation** (48 harbour strata, 42 in domain, expected mode; noisy on 6;
  `~/calib/raycast/correction/validation{,_v2}.json`, figures in `figures{,_v2}/`). Pooled mean |M1|:
  - v1: 600-bin 8.11 → 1.62 dB, 1200-bin 5.25 → 1.48 dB;
  - v2: 600-bin 9.99 → 1.79 dB, 1200-bin 7.25 → 1.66 dB.

  Both versions fail P2 at 9–10 m (C +3.1 to +3.7 dB) and P3 on 2–3 strata. v2 passes S1–S5 except S3 on 07-21. S6
  fails in every session for both, although C is closer to real than P on 36 of 42 window speckle statistics (v2,
  a diagnostic, not a criterion). Under 6b D, **both groups are EXCLUDED** in v1 and v2, and no criterion was
  changed. Per the owner's D2 this is a known limitation, not a no-go: the artifacts keep their domain, with
  `status: failed: every group EXCLUDED`. v2 is the recommended testable version (assessment §13.5). NC #4 for the
  hybrid is not triggered by the rule.
* 2026-11-07 — **CUDA error 101: diagnosed, left OPEN** (assessment §13.7). It is device-memory exhaustion: Warp's
  scan reports a failed temporary allocation after "error 2: out of memory". It is not a sizing or empty-array bug.
  - A 40 m harbour ping-side needs about 3.3–4.5 GB of working memory (pool high-water mark 5.4 GB with 2
    renderers), and each renderer keeps ~0.3–0.5 GB resident.
  - The OOM retry releases only the current renderer's buffers, so a deterministic demand fails all 11 retries.
  - 20 m / 1200 bins (the thesis acquisition) peaks at 1.4 GB.

  The two candidate fixes are not applied: release every cached renderer on retry, or segment the scatterer
  pipeline in range. Live-path check: the hybrid v2 profile builds and renders through the node's code path on the
  harbour world. It applies on the 9 in-domain poses, is off on the 3 above 11.3 m, and takes 0.1–1 s per
  ping-side.
* 2026-11-07 — **MCS/GCS integration verified end to end; range advice** (owner request). Run through the MCS
  `LaunchManager`: Ray-cast, x0.03, Extra args `sonar_config:=` the harbour hybrid v2, on the Shiraishijima world at
  20 m / 1200 bins, with the GCS App recording.
  - Result: 20.3 Hz in sim time, 3 seabed images, the `.svlog`, waterfall, mosaic and detections, and a clean stop.
  - Range sweep (GPU memory): 2.3 GB at 20 m up to 6.3 GB at 40 m; out of memory at 50 m. Advice: ≤ 30 m, never
    > 40 m (README).
  - Limitation: the processor's bottom tracker reads the ray-cast's first return 1.4–1.6 m late, against 2 cm for
    the geometric renderer (assessment §13.6).
  - Fix: `sss_sim_node.main`'s teardown race.
