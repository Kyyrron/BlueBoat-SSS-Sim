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

