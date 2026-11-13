# Ray-cast sensor — project-wide assessment (Stage 3, 2026-11-06)

**Status:** Stage 2 complete (2026-11-06 late); §4 holds its results and the 6a decision. **2026-11-07: the harbour-first hybrid (§13)** — gates amendments 6b, 6c and 6d; corrections v1 and v2 fitted and validated. Both acquisition groups are EXCLUDED under the pre-registered rule; the owner treats this as a known limitation, so the artifacts stay testable (§13.5). §6–9 and §12 are the 2026-11-06 proposal; where §13 differs, §13 holds.

Owner's redirect, 2026-11-06: the plan is `~/.claude/plans/bubbly-crafting-pixel.md`. The aim is the
best defensible sonar model for the project's experiments: a physical core, plus a separate,
switchable correction layer with a stated validity envelope. The procedure was pre-registered in gates
amendments 6 and 6a (`~/calib/raycast/`, private; their hashes are in `raycast_plan.md`).

## 1. What is established, and how firmly

Legend:
- **V** = verified, by a gate, a closed form or a measurement;
- **H** = hypothesis, still to be tested;
- **I** = assumption inherited from the implementation or from earlier work;
- **M** = needs a new measurement or new data.

| Claim | Class | Evidence / note |
|---|---|---|
| Scene mesh, BVH and visibility are exact (watertight, 0 leaks; GPU = CPU) | V | M1–M5, BVH = brute force, leak probe 0/22 049, G1 ε_μ 4e-6 (`raycast_phase1.md`, `raycast_phase4.md`) |
| Propagation: spreading and Francois–Garrison absorption; true c for delays, 1500 m/s for bins | V (formulas); I (site T, S) | A1, R1. T and S are per-site OISST ± 3 °C and S 35 ± 3 (owner); not measured |
| Near-field along-track aperture, L = 24.6 cm | V (the integral); I (L from the datasheet's "2-way 3 dB 0.5°") | B1–B3. L is bounded by the CAD housing |
| Small-slope seabed law: normalisation and its Kirchhoff limit | V | S1–S3; SSA = Kirchhoff to 0.00 dB at 89° (R2); 13.18 energy to −0.02 dB at 30 kHz (R3) |
| The 450 kHz roughness spectrum (w2, γ2) | **M** | No measurement at these scales (J&R §6.3/§6.5). SAX99 extrapolated 10×; the lobe spans ±60° |
| Multipath, Lloyd's mirror, the image comb | V | P1–P6, S4–S6 |
| Fresnel–Kirchhoff diffraction | V (D1); D2 only by the local-width metric | `raycast_phase2.md` |
| Speckle: Gaussian scatterers, 5 mm lattice converged | V | Density sweep (ε_μ p95 ≤ 0.008); C1 KS |
| **Vertical beam: a uniform 3.5 mm strip from "Beam Height 50°"** | **I** | The 50° is UNQUALIFIED on both vendor pages (Blue Robotics, Cerulean; read 2026-11-06), with no dB level and no one-/two-way. Reading it as a one-way −3 dB width is a provisional assumption |
| Mounting tilt 20° | **I** | Cerulean gives the 450 SS a "Vertical Beam Angle 0 deg" (no built-in tilt), so the 20° is the BlueBoat mount. It was corpus-fitted under the geometric Gaussian beam (`sonar_model.md`), not under the ray-cast's pattern |
| Large-angle vertical floor f_v (one-way amplitude, in quadrature; two-way power `(sinc² + f_v²)²`; not normalised on axis, so the system gain absorbs (1 + f_v²)²) | **H / M** | Registry `status: undetermined`, bounds [0, 0.3] are ad hoc. CPU = GPU = basis checked (`stage2/fv_agreement.json`). Blue Robotics' "full 180 degree view under the boat" supports a usable response at nadir *qualitatively* |
| f_v ≤ 0.217 (≤ the uniform strip's first sidelobe) = ordinary aperture behaviour; above that = a housing/baffle response | rule (6a) | `stage2/beam_table.json` |
| TVG law `k log10 r` | **I / M** | Undocumented device behaviour; k is fitted |
| Water column 27.8 dB (one measured constant, shared by both renderers) | **V as a discrepancy** | Real: harbour 600-bin settings 22–29 dB, 1200-bin settings 16.7–19.6 dB; beach (1–1.6 m from the transducer) 43 dB. Not a ray-cast defect |
| Real images range-compressed by 2.5–3 % (`sos_dmps` 1500 vs c ≈ 1540) | V | Corpus headers |
| Attempt-2 replay and altitude-match defects | V (found, fixed, ray-cast only) | `raycast_phaseA.md`; smoke 493 ALL PASS after the fix |

## 2. Test state since the replay fixes (2026-11-06)

- Smoke: 493 / ALL PASS (`notes/s4/smoke_s5a.log`).
- `raycast_test --quick`: 23 PASS / 0 FAIL up to [R2] wall paths (P3); stopped there at the owner's
  request.
- The remaining sections were rerun with `tools/rt_resume.py` (`notes/s5/rt_resume.log`, wall_checks
  to the end): **41 PASS / 0 FAIL / 2 SKIP**. The two skips are M2/M5 and M3: the one world holds no
  walls or smooth solids. They include P3 and P5, D1/D2, the corner and transmission checks, the fit
  basis vs full renders, **G1 (GPU = CPU)**, the PO closed forms, the seabed law and the mesh checks.
  So the whole `--quick` suite passes on the current code, which includes the replay fixes. The mesh
  crops used a fresh RNG stream (noted in the tool).
- New checks: [S-E] (13.18 energy) 2 PASS; [CL1]-[CL5] (correction layer) 5 PASS.
- Smoke after every code change of this session: 493 / ALL PASS (`notes/s5/smoke_s5b.log`).
- NC4 and the harbour quay: CUDA error 101 in attempt 2, unresolved (Stage 1.4).
- Full suite: not rerun.

## 3. Hold-out status

See `~/calib/raycast/stage1/holdout_audit.md`.
- **Unseen by the ray-cast sensor.** No render, statistic, fit or cache touched it.
- **Not pristine:**
  - within-site;
  - analysed with the geometric renderer in passes 1–4;
  - `pp_shell` 0.11 was measured on `object_boiteVsDistance`;
  - the geometric smoke test reads the two harbour hold-out files for decoder checks only.
- So final-test results are "independent of the ray-cast fit", and the B2 box-shadow comparison is not
  independent evidence.

## 4. Stage-2 result (beam + energy feasibility)

All Stage-2 files are in `~/calib/raycast/stage2/`; tools in `~/ros2_ws/sim_runs/raycast/tools/`.

### 4.0 Verifications (owner amendments 1, 2)

- **The 50° beam height.** Both vendor pages (Blue Robotics, Cerulean) state "Beam Height 50°" with no
  dB level and no one-/two-way qualifier. Only the 0.5° along-track width is qualified ("nominal
  theoretical 2-way 3 dB"). So FWHM matching is a **provisional assumption**.
- **The tilt.** Cerulean lists "Vertical Beam Angle 0 deg" for the 450 SS. Our 20° is therefore the
  BlueBoat mount, inherited from the geometric renderer's corpus fit.
- **f_v.** It is a one-way amplitude floor in quadrature; the two-way power is `(sinc² + f_v²)²`, with
  no on-axis normalisation. CPU = GPU = basis to ≤ 3e-3 dB at f_v 0.217 and 0.5 (`fv_agreement.json`).
- **Swap exactness.** Swapping the aperture or the seabed law on a built renderer gives exactly what a
  fresh renderer gives (≤ 2.4e-15 dB; `swap_check.json`, `seabed_swap_check.json`). This let all
  variants share one scene per seabed depth, under identical conditions.

### 4.1 Admissibility table (`beam_table.json`)

The H re-derivation keeps a 50° one-way −3 dB width (reading R1). The nadir column is the two-way
level at nadir, relative to the axis.

| f_v | width at fixed H | nadir, two-way | class |
|---|---|---|---|
| 0 | 51.3° | −70.9 dB | admissible |
| 0.1 | 51.7° | −40.0 dB | admissible |
| 0.217 | 53.1° | −26.9 dB | admissible (first-sidelobe level) |
| 0.3 | 54.7° | −21.5 dB | HYPOTHESIS |
| 0.5 | 61.0° | −13.4 dB | HYPOTHESIS |
| 0.7 | 71.8° | −8.9 dB | HYPOTHESIS |

Every re-derived H stays within the 20 mm housing. Under reading R3 (50° as a −6 dB width), no H
exists for f_v ≥ 0.7.

### 4.2 Energy check (J&R 13.18), 450 kHz

- **Valid (−3 / +1 dB band):**
  - J&R sands: SAX99 −2.0 dB, Falmouth −2.3, Quinault +0.3;
  - test grid: w2 ∈ {1e-6, 1e-5, 4.33e-5} × γ2 ∈ {3.04, 3.5}, plus 1.475e-4 at γ2 3.5.
- **Rejected:**
  - Panama City 98 / 93 (−4.0 / −4.2 dB): measured sands, so the band is tight at this frequency;
    they pass at ±6 dB;
  - Jacksonville (γ2 2.47): −30 dB;
  - the **whole γ2 = 2.6 row** (−3.2 to −59 dB): this is where attempts 1 and 2 ended (attempt 2:
    −32.8 dB);
  - w2 1.475e-4 at γ2 3.04: −7.7 dB.
- **Consequence:** the attempt-1 and attempt-2 fits sat outside the lowest-order SSA's domain. The
  system gain had compensated an SSA that loses most of the reflected energy.

### 4.3 Test A: beam variants, seabed fixed at SAX99 (`A.json`)

Six calibration strata; one shared gain and TVG k; identical conditions (config diff:
`beam.height_deg` and `beam.vertical_floor_amp` only). Crest and tail residuals are real − sim, in dB.

| f_v | gain | TVG k | rms | crest: beach 0.67 / 1.33 / 2.27 m | crest: harbour 5.67 / 9.47 / 16.47 m | tail: beach | tail: harbour |
|---|---|---|---|---|---|---|---|
| 0 | 137.6 | **0 (bound)** | 7.28 | +2.8 / +4.0 / +8.8 | +1.8 / +7.7 / +11.6 | +4.9 / −0.4 / −4.0 | −2.4 / −5.3 / −0.6 |
| 0.217 | 137.1 | **0 (bound)** | 6.72 | +2.4 / +3.1 / +6.6 | −0.0 / +6.4 / +10.7 | +4.9 / −0.4 / −3.9 | −2.4 / −5.3 / −0.6 |
| 0.5 (H) | 133.2 | 1.41 | 6.10 | −1.6 / −1.5 / +3.6 | −2.3 / +4.0 / +8.3 | +6.0 / +0.8 / −2.8 | −1.8 / −4.7 / −0.1 |
| 0.7 (H) | 130.5 | 2.24 | 6.18 | −2.3 / −2.6 / +2.5 | −3.6 / +2.5 / +6.6 | +6.5 / +1.3 / −2.3 | −1.6 / −4.6 / +0.1 |

(H) = hypothesis value.

- **6a decision for the beam: the admissible change FAILS** the accept rule. The best admissible value
  (0.217) leaves the harbour crest at +6.4 / +10.7 dB against the 3 dB tolerance, and TVG k sits on
  its bound.
- **The hypothesis values improve the fit but still fail the harbour crest.** They are recorded as
  evidence motivating a beam-pattern measurement, and not merged.
- **What the beam does not touch:**
  - **The crest residual grows with altitude in every variant:** about +11 dB from 5.7 to 16.5 m
    (harbour) and +4–6 dB from 0.67 to 2.27 m (beach). The simulated first return falls with
    altitude much faster than the real one (−11.3 dB per decade, image-source-like; NC #4 item 5).
  - The tail offsets (±5 dB per stratum, opposite signs) and the tail shape (rms 2–4.6 dB) barely
    move with the beam.
  - The harbour 16.47 m stratum's tail window is only ~23–29 m (rms 9.7, slope −173 dB per decade),
    so its tail statistics are unreliable and reported as such.
- **Roll evidence (supporting only).**
  - Predicted |dFBR/droll|: 0.5–1.0 dB/° at f_v ≤ 0.217; 0.3–0.4 at f_v 0.5.
  - Real, with covariates (`roll_slopes.json`): ≤ 0.3 dB/° on these strata (beach 0.67 m −0.01;
    1.33 m +0.09; 2.27 m +0.30 [0.13, 0.46]; harbour 5.67 m +0.23 [0.08, 0.39]), with a harbour roll
    spread of only ~1°.
  - It leans toward a stronger large-angle response than the uniform strip. It is not a measurement.

### 4.4 Test B: energy-valid seabed grid, beams 0.217 and 0.5 (H)

Files: `B_admissible_fv0.217.json`, `B_HYPOTHESIS_fv0.5.json`, `notes/s5/stage2_B.log`.
- Seven energy-valid (w2, γ2) sets were rendered; five invalid ones were skipped.
- One shared gain and TVG per set; identical conditions (config diff: `seabed.sediments.sand.{w2, gamma2}`
  only).

Admissible beam, f_v 0.217:

| sand (w2, γ2) | energy | gain | TVG k | rms | crest: beach 0.67 / 1.33 / 2.27 | crest: harbour 5.67 / 9.47 / 16.47 |
|---|---|---|---|---|---|---|
| 1e-6, 3.04 | −0.05 | 147.5 | 0.60 | 6.65 | −3.2 / −5.6 / +0.5 | −6.5 / −0.1 / +4.6 |
| 1e-6, 3.5 | 0.00 | 151.7 | 0 (bound) | 7.57 | −3.8 / −7.5 / −2.0 | −8.6 / −2.0 / +3.5 |
| 1e-5, 3.04 | −0.39 | 141.9 | 0 (bound) | **6.03** | +1.3 / +0.2 / +3.6 | −3.8 / +2.9 / +7.3 |
| 1e-5, 3.5 | +0.01 | 150.5 | 0 (bound) | 7.33 | −3.6 / −7.0 / −1.1 | −8.1 / −1.7 / +3.0 |
| 4.33e-5, 3.04 (SAX99) | −1.98 | 137.1 | 0 (bound) | 6.72 | +2.4 / +3.1 / +6.6 | −0.0 / +6.4 / +10.7 |
| 4.33e-5, 3.5 | +0.06 | 148.1 | 0.03 (bound) | 6.56 | −2.0 / −3.3 / +1.2 | −7.0 / −0.4 / +4.2 |
| 1.475e-4, 3.5 | +0.20 | 145.0 | 0 (bound) | 6.23 | +0.7 / −1.9 / +1.5 | −5.9 / +0.8 / +5.3 |

**Hypothesis beam, f_v 0.5:** the best rms is 6.10 (SAX99), then 6.44 (1e-5, 3.04). The pattern is the
same: harbour crest −6.4 / 0.0 / +4.2 at (1e-5, 3.04).

**The finding that matters.** In **every** set, at either beam:
- the harbour crest residual rises by **+10.7 to +11.2 dB from 5.67 m to 16.47 m** of altitude;
- the beach crest residual rises by about +3–5 dB from 0.67 m to 2.27 m.

The seabed sets and the beam move the crest up or down as a whole. None changes its altitude slope.
The tail offsets per stratum (beach 0.67 m sim too dark by +5–9 dB; harbour 9.47 m too bright by
−4 dB) and the tail shapes (rms 1–6 dB) also barely depend on the tested parameters.

### 4.5 Stage-2 decision (gates amendment 6a)

- **FAIL. No admissible change meets the accept rule.** No beam f_v ≤ 0.217 and no energy-valid seabed
  set brings all three harbour crests within the 3 dB tolerance. The shared TVG k sits on its 0 bound
  in most sets.
- **Not merged:**
  - `raycast.beam.vertical_floor_amp` stays registered (default 0, bounds [0, 0.3]);
  - the seabed spectrum stays a fitted, energy-checked parameter.
- **Unresolved limitations, recorded:**
  1. **The vertical beam's large-angle response:** unmeasured. f_v > 0.217 helps the fit and the roll
     evidence, but stays a hypothesis.
  2. **The 450 kHz small-scale spectrum:** unmeasured. The measured-sand sets that pass the energy
     check give broad lobes.
  3. **The first-return altitude law:** the simulated crest falls with altitude about 11 dB per
     altitude-tripling faster than the real one, whatever the beam and seabed. The real law is
     image-source-like (−11.3 dB per decade, NC #4 item 5); the ray-cast's near-nadir return
     behaves as an incoherent one.
- **Kept from Stage 2:**
  - the energy check, a reject-only physical validity constraint. It shows the attempt-1 and
    attempt-2 solutions (γ2 2.6) lay outside the SSA's domain;
  - the exactness tools (CPU = GPU = basis; beam and seabed swaps).
- **Stop rule in force:** no further bound change, no spectral roll-off, no new physical effect.

## 5. Compute

Measured on the GPU (Warp 1.19, RTX 4060), per ping-side:

| Scene | Time |
|---|---|
| flat | 0.06–0.08 s |
| pool | 0.6 s |
| harbour | 2.4 s |

The budget is 25 ms. The geometric renderer takes 6–7 ms.

**Simulation slowdown with one clock (owner amendment 9), audited 2026-11-06.** **Resolved 2026-11-07:** the
slow-down mode was built. The control nodes take `time_source: ros`, the processor and Perception `use_sim_time`,
and the GCS / MCS run on clocks paced by `/clock`. It was verified end to end with the ray-cast
(`integration_guide.md` §2b, §13.6). The audit below is kept as the record of what had to change:
- Every simulation launch starts its nodes with `use_sim_time:=true` (`Sim_launch.py`,
  `full_mission_launch.py`, `sss_sim_launch.py`, `sea_state_launch.py`). ROS timers and stamps
  therefore follow Gazebo's `/clock`: `master_control`'s 20 Hz loop, `simulation_interface`'s 10 Hz
  loop, the sonar's ping timer and the sea state.
- Gazebo's real-time factor below 1 slows all of them together.
- **Wall-clock uses that would break that consistency at RTF < 1** (Control is frozen and add-only,
  so changing any of them is an owner order):
  - `simulation_interface.thruster_input_stale` and the ready republish use `time.time()`. The
    0.5 s thrust watchdog would zero the thrust once `master_control`'s sim-time 20 Hz commands are
    more than 0.5 s of wall time apart (RTF ≲ 0.1).
  - `master_control`:
    - `current_time` is `time.time()`. It feeds `solve_LoS` and the manual keep-location hold, the
      path-request timeout and the monitoring stamps;
    - `arrow_rx` (the arrow timeout) is wall time too;
    - the τ governor itself is "parameter-governed, never wall-clock".
  - Perception's join and label waits use `time.monotonic()`.
  - The processor's stream-stop check is wall time on purpose (superproject §4.3). It must tolerate
    a low RTF.
- Gazebo applies the hull dynamics and the sea-state wrenches on sim time. This is consistent.

## 6.–9. Model split, and the correction-layer design (proposal for the next session)

- **Keep in the physical core (well supported, §1):**
  - geometry, visibility and multipath (P1–P6);
  - propagation and the near-field aperture;
  - the SSA law with the energy validity check;
  - the site sea surfaces;
  - the coherent floor channel.

  Nothing new is added: Stage 2's stop rule holds.
- **Physical core settings for the correction work (proposed):**
  - seabed: SAX99 sand (literature, energy-valid), or the energy-valid set with the best shared-fit
    rms (1e-5, 3.04: rms 6.03);
  - beam: f_v 0.217 (admissible, the best admissible in Test A).

  This choice must be written in gates amendment 6b before the correction is fitted. Recommendation:
  **SAX99 + f_v 0.217**. Both are literature- or rule-based, so no number is chosen by fitting the
  validation data.
- **Address with the correction layer:**
  - **The first-return (crest) level as a function of altitude.** The residual is systematic, smooth,
    the same sign across sites, and independent of the tested physics.
  - Proposed form: `u_logalt`. That is a dB table over u = r/alt ∈ [0.9, 1.4] (zero below 0.9 and
    from 1.6 on, a smooth taper between), times a few log10(alt) knots spanning the calibration
    altitudes (0.6–17 m). That is about 3 u-knots × 4 altitude knots, fitted on the calibration
    strata only.
  - Acceptance: amendment 6b (owner amendment 6).
- **Do NOT correct with this layer:**
  - **The per-stratum tail offsets (±4–9 dB).** They change sign between strata at similar
    geometry, so they are not systematic in u or altitude. A smooth correction would only average
    them. Likely causes, as hypotheses: per-recording gain or TVG state, or a seabed that is not sand
    (the harbour). They are reported as model scatter.
  - **The water column (27.8 dB vs 16.7–29 dB real).** It belongs to the shared model (it is the
    geometric renderer's constant too). It is flagged to the owner as a shared item.
  - **The harbour 16.47 m stratum's tail.** Its window is too short to judge.
- **Not worth modelling further for the application, for now:** elastic shell waves, bubbles and wake,
  refraction (no profile), surface backscatter, and a beam pattern beyond what a measurement could
  supply.
- **Evidence that would reopen the physics:**
  - a vertical beam-pattern measurement (a tank, or a controlled roll / tilt sweep over a flat
    bottom): it would settle f_v and the tilt;
  - a measured 450 kHz roughness spectrum (stereo photogrammetry at mm scale);
  - a repeated survey with logged device gain/TVG state: it would explain the tail offsets.

## 10. Calibration and validation strategy

- **Data roles.** These follow `split.yaml`, unchanged:
  - **Calibration (fit):** beach B1 (`bath1–3`), harbour PM-cal (`reflectionEvidenceFullharbour`),
    the pool 1w-early runs.
  - **Validation:** harbour 07-21, 07-22-AM, 07-22-PM-diffsv; pool 10-27-SV and 1w-line. These were
    scored in attempts 1 and 2, so they are **development evidence**, not independent validation.
  - **Hold-out:** harbour 07-23 and beach B2. They are unseen by the ray-cast and not pristine
    (§3). Open them once, at the end, after the owner's sign-off.
- **Unit of independence: the session.** Strata and pings within a session are never split
  between fit and score. Statistics are compared session by session. The bootstrap over ping blocks
  is reported but is never an acceptance criterion (owner amendment 6).
- **Four arms on identical strata and seeds:** real; physical ray-cast; corrected ray-cast; geometric
  (a reference only, not under development here).
- **Correction acceptance (gates amendment 6b), written before the correction is fitted.** It lists:
  - the metrics that must improve, each with an absolute target;
  - the metrics that must stay stable, each with an absolute margin: FBR position ± 1 bin; tail mean
    and profile rms ≤ 0.5 dB worse; K/KS, range lag-1–3 and track lag-1 ≤ 0.03 change;
  - harbour quay face where available.
- **Metrics that cannot be measured with the present data:**
  - beach object highlight and shadow, and aspect signatures: only in the sealed B2 hold-out;
  - the pool object and box statistics: deferred (cost and floor description, §4 of
    `raycast_phaseA.md`);
  - range repeatability across recordings: the harbour validation sessions only.

## 11. Application-level validation (brief §8): what can and cannot be done now

**Built:** Perception (`blueboat_perception`).
- Its dataset mode labels the simulator's seabed images from ground truth.
- The detector defaults to a dummy. The YOLO adapter exists but has no trained weights.

**Built but not yet usable for this experiment** (superproject `CLAUDE.md` §1): **Belief** and the
**Replanner** exist, but they have never run with a trained detector or in a replanner-owned
Gazebo run (built worlds cannot host one yet, Control R-3). **Evaluation** is not built.

So the paired experiment that would answer "do the adaptive-replanning conclusions survive the
sonar-model uncertainty" cannot be run yet. It is specified here so it can be run as soon as a
trained detector and a replanner-owned Gazebo run exist, with an Evaluation module to score it:
- **Scenarios:**
  - the same World-Builder worlds and object layouts;
  - the same vehicle conditions and sea states (seeded);
  - the same perception model and weights;
  - the same planners: the adaptive policy and a seriously tuned two-pass orthogonal baseline
    (CM-12).
- **Factor:** the sonar arm, in three settings: geometric, physical ray-cast, corrected ray-cast.
  All other inputs are identical.
- **Outputs:**
  - detection precision and recall;
  - localisation error;
  - aspect-estimate error;
  - coverage;
  - survey distance and duration;
  - replanning decisions (count, timing);
  - robustness across object distributions.
- **Question:** does the *ranking* of adaptive vs baseline, and its effect size, change with the sonar
  arm beyond the run-to-run spread? If not, the remaining sonar-model error does not threaten the
  thesis conclusion. If it does, the discrepancy that drives it is the one worth more modelling.
- **Preconditions:**
  - trained detector weights (CM-11: trained on simulated datasets; the arm used for training is
    itself a factor);
  - the Replanner's live path update (Control `/blueboat/update_path`, built and Gazebo-tested);
  - a slowed simulation with one clock (§5) if the ray-cast arm is to run live. Built on 2026-11-07 (§5).

## 12. Recommendation and implementation sequence

**Recommendation: the hybrid.** Keep the physical ray-cast core as it is, with the energy check.
Correct only the first-return-vs-altitude residual, through the switchable empirical layer (built,
tests CL1–CL5). Treat the tail offsets as documented model scatter. The pure-physics route has reached
its stop rule: the remaining crest discrepancy cannot be closed by any parameter that stays physical,
and the measurements that could close it (beam pattern, mm-scale spectrum) do not exist.

Next steps, in order. The machine was shut down after Stage 2.
1. Write **gates amendment 6b** before fitting anything. It records:
   - the physical-core settings (SAX99, f_v 0.217);
   - the correction form (`u_logalt`, knots);
   - the fitting data (the calibration strata only);
   - must improve: crest |residual| per calibration and validation session ≤ 3 dB (the gates' crest
     tolerance);
   - must stay stable, absolute margins: FBR position ± 1 bin; tail mean and profile rms ≤ 0.5 dB
     worse; K/KS, range lag-1–3 and track lag-1 ≤ 0.03 change; harbour quay face;
   - the validity domain: altitude 0.6–17 m, the calibration acquisitions;
   - bootstrap intervals reported, never the criterion.
2. Fit the correction artifact (a new `tools/correction_fit.py`):
   - expected-mode crest residuals on the calibration strata, from full renders or bases;
   - least squares on the dB table with a smoothness penalty, no per-stratum term;
   - write `blueboat_raycast_correction/1` YAML under `~/calib/raycast/correction/`.
3. Validate on the validation sessions (development evidence) with the four arms: real, physical,
   corrected, geometric. Show the speckle preservation on noisy rows (amendment 8). Decide by 6b.
4. Bounded CUDA error 101 investigation (`phaseA_nc4.py` alone), then NC #4 for the hybrid.
5. Owner decisions:
   - the Control wall-clock items (§5) if a slowed simulation is wanted (done on 2026-11-07, owner order);
   - the shared water-column constant;
   - a beam-pattern measurement.
6. Only then, Phases 3/5/6/7. The hold-out is opened once, after sign-off; its status is §3.


## 13. The harbour-first hybrid (2026-11-07; gates amendments 6b, 6c, 6d)

Owner's direction: the thesis application is harbour surveying, so the target is a defensible sonar model in the
harbour operating conditions. Shallow beach water is a known secondary limitation and never enters the harbour fit.
The chain is the physical core → a separate empirical correction (not physics) → the shared noise and encoder stage.
The procedures and acceptance were pre-registered before any render, in `~/calib/raycast/gates_amendment_6b.yaml`
(sha256 `66ee6c35…`, the owner's decisions D1–D4 quoted). `gates_amendment_6c.yaml` (sha256 `2c12231d…`) followed
after the adequacy stop.
`gates_amendment_6d.yaml` (sha256 `c0a0666a…`) defines v2. It was written after v1's partial validation and before
any v2 fit, so every v2 validation number is development evidence.

### 13.1 What the harbour data actually are

- **Calibration** is one recording: `reflectionEvidenceFullharbour` (PM-cal). It holds 16 strata at one acquisition
  (40 m / 600 bins / 106.4 µs, gain 4), at 5.67–16.47 m FBR altitude. No stratum lies in 11.5–14.5 m, and only port
  above 11.5 m. Its clock is frozen and its FBR detection is 78–80 %.
- **Validation** (development evidence, all scored in attempts 1–2, within-site):
  - PM-diffsv: the calibration acquisition, the same power-up 35 min later.
  - 07-21: 20 m / 600 / 66.5 µs and 35 m / 600 / 117.5 µs.
  - 07-22-AM: 1200 bins, 18–52 m, 44.3–86.5 µs. Its `fullHarbourCleanExample` gives no stratum.
  - There is **no validation stratum above ~10.4 m.**
- The recorders differ (the July `sss_processor_node` vs SonarLink), and so does the real water column (600-bin 22–29
  dB vs 1200-bin 16.7–19.6 dB). The seabed was never sampled.
- The thesis acquisition (20 m / 1200 / 44.3 µs) is not the calibration one. It lies inside the 1200-bin group's
  recorded envelope.
- Real-only calibration repeatability:
  - port − starboard crest +0.6 to +2.2 dB, tracking the tail (+1.0 to +1.6 dB, a per-device level);
  - split-half crest differences up to 3.9 dB.

  So the 3 dB crest tolerance is meaningful per session and sits at the real scatter per stratum.
- In the Shiraishijima world at the recordings' tide levels, 85–90 % of the area lies at 5.5–11.3 m nadir altitude.

### 13.2 Physical core (unchanged code)

- SAX99 sand + f_v 0.217 (Stage 2's proposal, verified against `stage2/A.json` / `B_*`), harbour sea surface 1e-3 m.
- The shared system gain and TVG k are refitted on the **harbour calibration tails only** (D1): **132.93 dB, TVG k 0**.
  TVG k sits on its lower bound, as in Stage 2.
- The seabed is placed at true distance = device altitude × c/1500 (D4; c 1541.3 m/s), so the simulated FBR lands on
  the real bin. Stage 2's placement put it 2.7 % early.
- Stage 2 reproduced to 8.9e-16 dB on its three harbour strata: the physics is unchanged.

### 13.3 The correction mechanism (`sonar/raycast/correction.py`)

- It multiplies the mean power by `10^(c/10)` over u = r / nadir altitude ∈ [0.85, 1.35] only.
  - The table is required to be 0 dB at both ends.
  - Texture, tail, far range and the water-column floors are untouched.
- **Off outside its domain.** The domain is nadir altitude 5.55–11.30 m, an acquisition inside a recorded group's
  envelope, and sand seabed under nadir. Outside it:
  - the ping is bit-identical to the physical one;
  - it carries `ping.correction_status = off:<reason>`;
  - one warning is logged per reason, and counters are kept.
- An artifact carries the fingerprint of the physical core it was fitted on. Any other core refuses to construct.
- The calibration engine applies it after the cached physical render, so physical and corrected arms share every
  render and texture.
- Tests CL1–CL11 pass; smoke 493 / 0.
- The cost is 0.052 ms per ping-side.

### 13.4 Fit

- 6b's pre-declared adequacy check **stopped** the full-span fit.
  - The three port-only strata at 14.5–16.5 m carry a +20 to +26 dB crest deficit. That exceeds the 15 dB bound and
    lies off the linear-in-log10 trend (a plateau of +12 to +15 dB at 7–11.5 m).
  - Bands 5–11 m were already within 3 dB.
  - See `~/calib/raycast/correction/adequacy_stop.md`.
- On the owner's instruction, 6c restricted the fit set to the 13 strata at 5–11 m, with nothing else changed.
- **F1** (a trapezoid in u, linear in log10 alt) was selected. F2's CV rms was 0.40 dB better, short of the 0.5 dB
  rule.
  - Coefficients: 7.76 dB at 5.56 m and 12.68 dB at 12.15 m.
  - Adequacy passed (worst band +2.7 dB in-sample, +2.9 dB CV).
  - In-sample band crest residual: physical +4.9 to +9.7 dB → corrected −0.8 to +2.7 dB.
- The domain edges are hard switches; the correction there is 7.8 dB at 5.55 m and 12.2 dB at 11.30 m.
- **v1 files:**
  - artifact `~/calib/raycast/correction/blueboat_raycast_correction_1.yaml` (disabled by default);
  - profile `~/calib/raycast/correction/raycast_harbour_hybrid_sonar.yaml` (outside the repository).

**v2 (6d).** v1's partial validation showed:
- the real first-return excess extends to u ≈ 1.8–2.0, so v1's support ended at a ~5 dB brightness edge (u 1.35);
- the core gain's tail selector overlapped that excess.

So 6d widened the support to u ∈ [0.85, 2.0]. It refitted the core gain and TVG on the tail beyond it
(r ≥ 2.25·alt; still the 16 PM-cal strata): **126.91 dB, TVG k 3.76**, off its bound. It also restated S1, S4 and S5
(S1 versus real; S4 adds [1.8, 2.4]·alt; S5's tail is beyond the support).
- **F2v2 was selected** (knots u 0.85 / 0.95 / 1.10 / 1.35 / 1.60 / 2.0, 4 free values per altitude knot). Its CV rms
  was 2.75 dB, against F1v2's 3.34.
- 3 of its 8 coefficients sit on the 15 dB bound.
- Adequacy passed: worst band +2.74 dB in-sample, +2.79 dB CV.
- In-sample band crest residual: physical +7.2 to +11.5 dB → corrected −1.3 to +2.7 dB.
- The correction at u = 1 is 13.6 dB at 5.55 m and 14.9 dB at 11.30 m. The hard switch at the domain edges is that
  large.
- **v2 files:** artifact `blueboat_raycast_correction_2.yaml`, profile `raycast_harbour_hybrid_v2_sonar.yaml` (same
  folder). **This is the recommended testable version.**

### 13.5 Validation

**Setup.**
- The four arms are real, physical (P), corrected (C) and geometric (G).
- The data are 48 strata from the three harbour validation sessions; 42 are in the domain.
- Expected mode covers every stratum. Noisy mode (120 pings, seed 7) covers 6 strata, 2 per session: the preview set.
  Noisy rows for the other in-domain strata were not rendered. They cannot change either group's outcome, because P2
  and P3 fail in expected mode.
- Files, all under `~/calib/raycast/correction/`:
  - results: `validation.json` / `validation.md` (v1) and `validation_v2.json` / `validation_v2.md` (v2);
  - figures: `figures/` and `figures_v2/`. Each stratum has the four waterfalls, the mean profiles, real − sim against
    u, and c(u). `m1_vs_altitude.png` summarises all strata.

| | v1 (6b/6c) | v2 (6d, development evidence) |
|---|---|---|
| core gain / TVG k | 132.93 dB / 0 (bound) | 126.91 dB / 3.76 |
| pooled mean \|M1\| P → C, 600-bin group | 8.11 → 1.62 dB | 9.99 → 1.79 dB |
| pooled mean \|M1\| P → C, 1200-bin group | 5.25 → 1.48 dB | 7.25 → 1.66 dB |
| session M1 C: 07-21 / AM / PM-diffsv | +3.32 / −0.73 / +0.74 dB | +3.43 / −0.79 / +0.73 dB |
| band M1 C, 5–8 m | −1.3 … +1.8 dB (pass) | −1.2 … +2.0 dB (pass) |
| band M1 C, 9 / 10 m | +3.09 / +3.36 dB (P2 fail) | +3.30 / +3.73 dB (P2 fail) |
| P3 fails | 3 strata | 2 strata (07-21 port 20 m @10.6 m: +6.45; AM stbd 26 m @6.05 m: −4.78 vs P +3.21) |
| P4, P5 | pass | pass (noisy crest 07-21 −6.2 → −1.2, AM −2.3 → −1.1, PM −4.6 → +0.5 dB) |
| S1–S5 | S1, S4, S5 fail in every session; S2 fails on PM-diffsv | all pass, except S3 on 07-21 |
| S6 (speckle, \|C − P\| ≤ 0.03) | fails in every session | fails in every session |
| outcome (6b D) | both groups EXCLUDED | both groups EXCLUDED |

- The out-of-domain strata whose poses are all off: C ≡ P bit for bit (4 strata). Two AM strata straddle 5.55 m.
  Their in-domain poses are corrected, as designed.
- **Why the rule excludes v2.**
  - P2 fails at 9–10 m: C is still +3.3 to +3.7 dB under real there, and 3 of the coefficients are on the 15 dB bound.
  - S6 fails everywhere. S6 is a stability margin (\|C − P\| ≤ 0.03 on window contrast, k, correlations), and
    raising the mean level over the physical arm's floor-dominated crest window necessarily changes those statistics.
  - Diagnostic only, not a criterion: C is closer to real than P on **36 of 42** window speckle statistics
    (v1: 30 / 42), in 5 of 6 noisy strata. The exception is 07-21 starboard 35 m at 6.9 m, where C overshoots real
    contrast (1.35 vs 1.18).
  - ENABLED-KNOWN-LIMITATION requires S6, so the rule gives EXCLUDED. Per 6b D, no threshold was changed after the
    results.
- **Owner decision (D2, 2026-11-07):** a per-group transfer failure is a known limitation, not a no-go. The artifacts
  therefore keep their domain and stay usable for testing. Their `status` records the failure
  (`failed: every group EXCLUDED`) and the per-group outcomes.
- **Not reproduced by any correction:**
  - At 9–10 m the real return rises from u ≈ 0.7, before nadir. A flat-seabed replay cannot create that.
  - The along-track speckle correlation in the crest window is 0.03–0.09 in both P and C, against 0.23–0.62 in real.
    This is a physical-core texture property. The geometric arm is 0.08–0.18.
  - In the AM (1200-bin) noisy rows, C's median FBR detection lands 1.3–2.6 m past real's. P's fails or lands later
    still.

### 13.6 Known limitations of the first version

- **Shallow water** (< 5.55 m nadir altitude, every beach stratum): correction off, physical core only. There is no
  beach validation data outside the sealed B2.
- **14.5–17.3 m:** the correction does not represent it (6c); off there.
- **One site, one calibration recording,** and the seabed "probably sand". This is no claim of validity for harbours
  in general.
- **The per-stratum tail offsets** (±2 dB with the harbour gain) and **the shared water column** are not corrected.
- **The hard switch** at the domain edges: v1 steps of 7.8 / 12.2 dB on the crest, v2 13.6 / 14.9 dB at u = 1.
- **9–11.3 m:** C is still +3.3 to +3.7 dB under real (P2), and v2's amplitude is bound-limited there.
- **Speckle in the corrected window** changes with the level (S6). It moves toward real on most statistics, but not
  within the stability margin.
- **u is referenced to the vertical nadir altitude.** On sloping seabeds the first return precedes u = 1; this was not
  validated.
- **S7 (quay face)** is pending CUDA error 101.
- **GCS bottom tracking.** The end-to-end run on 2026-11-07 (Shiraishijima, 20 m / 1200 / 44.3 µs, x0.03,
  `integration_guide.md` §2b) compared the processor's tracked bottom (`rows_data.bottom_m`) with the world's true
  transducer altitude:

  | renderer | tracked bottom vs truth | 90 % of errors within | ping-to-ping jumps |
  |---|---|---|---|
  | hybrid v2 | +1.37 to +1.59 m late (median) | 3.2–3.8 m | up to 4.8 m |
  | geometric | within 2 cm | 4 cm | none |

  - The ray-cast image has no sharp first-return line, so the tracker fires late. This is §13.5's "AM noisy FBR
    1.3–2.6 m past real" at the thesis acquisition.
  - The seabed images are produced, but their georeference near nadir is off by that altitude error.
  - NC #4 (FBR lock) was never established for the ray-cast or the hybrid.
- **Testing:** offline with the tools, or live with `sonar_renderer:=raycast sonar_config:=<the v2 profile>`
  (§11 of `raycast_design.md`; the node builds `RaycastRenderer` from the profile's `raycast:` section, correction
  included). A harbour ray-cast ping-side takes ~2.4 s, so a live run must be slowed accordingly. Harbour worlds on
  the GPU: at 20 m they run (§13.7); at 40 m they can exhaust device memory. **Verified end to end through the
  MCS and GCS Apps on 2026-11-07.**

### 13.7 CUDA error 101 (bounded investigation, 2026-11-07)

`tools/cuda101_probe.py` ran alone on the GPU. It logged Warp's mempool and the free device memory after every
render. Logs: `~/ros2_ws/sim_runs/raycast/notes/s6/cuda101_*.log`; results: `~/calib/raycast/cuda101/`.

- **Cause.** "CUDA error 101: invalid device ordinal" is Warp's `array_scan` failing to allocate its temporary
  storage after a device out-of-memory error ("error 2"). It is device-memory exhaustion. It is not a sizing or
  empty-array bug.
- **Measured** (harbour world, the recording's own poses, Warp default release threshold 0):

  | replay | renderers | resident buffers per renderer | pool high-water mark | free after |
  |---|---|---|---|---|
  | 07-21, 20 m and 35 m / 600 bins | 3 | 280–310 MB | 3.46 GB | 4.2 GB |
  | PM-diffsv, 40 m / 600 bins | 2 | 300–530 MB | **5.40 GB** (3.8 GB after the first render) | 3.1 GB |
  | thesis 20 m / 1200 bins, hybrid v2 (`live_path_check.py`) | 1 | ~480 MB | 1.41 GB | 6.0 GB |

  One 40 m harbour ping-side needs about 3.3–4.5 GB of working memory: the scatterer sheet scales with the swath
  area / h², plus wall refinement. The mempool keeps that peak reserved.
- **Why the failed runs failed.** On the 2026-11-05 and 11-06 runs, `harbour_quay` died on its second recording
  (PM-diffsv, 40 m). The OOM retry (`calib._build_retrying`) releases only the current renderer's buffers. The
  demand is deterministic, so all 11 retries failed.
- **Status: OPEN.** Under the plan's bounded rule, only a sizing or empty-array bug would have been fixed. Candidate
  fixes, not applied:
  1. Release every cached renderer's buffers before a retry in `scene_replay.render_rows` (saves ~0.3 GB per
     renderer).
  2. Process the per-ping scatterer pipeline in range segments, so the peak is bounded. This is exact, since the
     contributions add per bin. It needs G1 CPU/GPU equivalence.
- **Range sweep** (`tools/range_sweep.py`, one renderer as in the live node, hybrid v2 profile, 1200 bins; the
  PM-diffsv world's quay-facing poses with the quay inside the range, the worst case; one process per range):

  | range | 20 m | 25 m | 30 m | 35 m | 40 m | 50 m |
  |---|---|---|---|---|---|---|
  | pool high-water mark | 2.32 GB | 3.08 GB | 3.56 GB | 5.65 GB | 6.33 GB | OOM on the 4th pose (quay 26.9 m) |
  | min. free device memory | 5.25 GB | 4.93 GB | 4.65 GB | 3.37 GB | 2.79 GB | 0.04 GB |

  The advice, recorded in the module README, is to keep the ray-cast at ≤ 30 m and never above 40 m on an 8 GB
  GPU. That includes the GCS's range control, which re-ranges the simulated sonar.
- **Consequences.**
  - The thesis acquisition (20 m) runs with a large margin, offline and live.
  - 40 m harbour replays (the PM recordings) can fail on poses near the quay.
  - S7 (quay face) was not run. It cannot change the group outcomes (§13.5).

## 14. Harbour profiles: level, first return, shadow floor (2026-11-10)

Plan `~/.claude/plans/pasted-content-id-09ed-see-in-sleepy-hartmanis.md`; evidence
`~/ros2_ws/sim_runs/raycast/harbour_patch/` (`level_check.py`, `level_check.json`, the curves).
Trigger: the 2026-11-10 harbour comparison (`~/ros2_ws/sim_runs/waves_review/REPORT.md`).
On the fast profile at 20 m / 1200 / 44.3 µs it measured:
- the seabed +11 dB hot;
- the anchor shadows empty;
- the first return read 0.5–0.6 m late.

**Diagnosis.**
1. **Level.** The shipped profiles carry `PROVISIONAL_RAYCAST`, anchored only at the bath3 geometry
   (1.4 m altitude). The §13 harbour core never reached a profile:
   - `check_core` refused the fast engine keys;
   - its table was off below 5.55 m;
   - the 6b gates read "failed".
2. **Shadows.** `reverberation_db` 0 ("physics only", `raycast_design.md` §3) has no test behind it,
   because D3 never ran. In fast mode the physical fill sources are absent: no volume
   reverberation, a coherent-only surface, no object diffraction.
3. **First return.** At this acquisition the real harbour has no sharp crest: u = 1 sits −3…+4 dB re
   u = 2, and its tracker jumps 0.16 / 0.88 m (p50 / p90). What the ray-cast lacked was the crest
   *level*: the §13 core alone leaves it 5–10 dB cold.

**Patch.** No change to the default, live or fast profiles. Two generated profiles:
`config/raycast_harbour_{live,fast}_sonar.yaml` (`config.RAYCAST_VARIANTS`, `HARBOUR_V2_CORE`).
- **Core:** the §13 core.
- **Table:** the §13 table (`config/raycast_correction_harbour_v2.yaml`, a byte copy), with three
  profile-level additions:
  - `correction.altitude_hold_m` [4.5, 11.3]: below 5.55 m the 5.56 m row is applied
    (`applied:held`);
  - a linear taper to 0 at 3.5 m (`applied:taper`);
  - `accept_engine_knobs`: `check_core` substitutes the artifact's own values for
    `config.PERFORMANCE_KEYS` and compares the physics.
- **Floor:** `raycast.floor.reverberation_db` 18, the NC #15 floor.
  - Its reference is the ping's own ambient level: `floor.ambient_reference`, block medians of the
    mean over u ≥ 1.2 with a power-law fit.
  - It is fed to the noise stage only.
  - It is the geometric sensor's measured stand-in (quay 17–25 dB), not physics.

**One check, run once** (`level_check.py`): flat sand, rendered on the live renderer path, against the
toOptimize port strata at 44.3 µs. toOptimize is the AM session (validation); the guard refuses the
hold-out. Figures are real − sim [dB]:

| stratum | far seabed (tail), provisional → harbour | crest (M1), provisional → harbour | harbour fast vs full engine (max \|dB\| over the seabed) |
|---|---|---|---|
| 18 m @ 4.8 m (held) | −10.7 → −1.5 | −5.5 → −1.2 | 0.19 |
| 26 m @ 6.0 m | −11.1 → −1.2 | −7.0 → −2.0 | 0.18 |
| 26 m @ 7.5 m | −12.2 → −0.8 | −3.3 → +1.8 | 0.16 |

- The level and crest targets (±3 / ±5 dB) hold.
- The core holds under the fast engine keys (mean difference 0.00 dB).
- The first-return detector (`fbr_bin`) is not evaluable on noise-free mean curves: it finds none on
  most of them, real ones included. The tracker's behaviour is the owner's live test (GCS bottom
  line).

**Status.** A provisional harbour site calibration with an empirical layer that failed its strict
gates (§13.5: P2 at 9–10 m, P3, S6) while improving every band.
- Harbour worlds only.
- Never the sim-to-real comparison profile.
- The floor is a stand-in until D3 runs.
- Still open: ground-truth contacts.

### 14.1 v2: the owner's first run (same day)

The owner's harbour-fast run (`SSS_data/sessions/2026_11_10-08_37_26`) showed two defects:
- horizontal lines across the anchor shadow;
- a worse nadir zone.

Measured (`~/ros2_ws/sim_runs/raycast/harbour_patch/line_diag/`):

**Bottom tracker.** Tracker minus the true nadir: median +1.27 m, p90 +3.21 m. Row-to-row jumps p90
1.25 m, against 0.40 m before the patch.

**Cause of the tracking loss.** The processor's rule is: noise = the mean of 20 samples after the
ringing, plus 8 dB, held for 3 bins. On the sim it read:

| | real (1200 bins / 44.3 µs) | sim |
|---|---|---|
| ringing end | 0.138 m | 0.47 m |
| noise estimate | 28.5 dB | 41 dB |
| resulting threshold | — | 49 dB |

That threshold sat above the now correctly levelled seabed (40–46 dB).
- The shared ringing (55 dB, −22 dB/m) and water column (27.8 dB) were measured at 600 bins.
- The real 1200-bin / 44.3 µs mode rings 3× shorter, and its water column is 14.5 dB (mean of dB).
- Nothing in the correction table caused it: the recorded crest averaged 45–46 dB, against real
  46–52 dB.

**Cause of the shadow lines.** The v1 floor reference, a free per-ping fit, jumped 6–8 dB between
pings.

**Fixes.**
1. **Floor reference** (`floor.py`): fitted on the physical mean, with a Theil–Sen slope and a median
   level, both smoothed across pings.
2. **Near range** (`raycast.near_range`, `config.HARBOUR_NEAR_RANGE`): measured on the AM session at
   that acquisition, applied only there.
   - Generator: `harbour_patch/near_range/make_table.py`.
   - The table reproduces the real mean-of-dB profile to 0.01 dB.
   - The AM session therefore no longer validates this table.

**On the run's own track** (1 052 pings a side):

| measure | result |
|---|---|
| ringing end | 0.133 m |
| noise estimate | 29.4 dB |
| first bottom return − true nadir, median | −0.03 / −0.06 m |
| within 0.3 m of the true nadir | 99.7–100 % |
| ping-to-ping jumps, p50 / p90 | 0.02 / 0.07 m |
| floor change between pings, p90 | 0.09–0.25 dB |

**Still open.** The geometric renderer carries the same 600-bin near range at 1200 bins. Its sharp crest
locks the tracker anyway. Changing it touches the comparison profile (NC #6): the owner's call.
