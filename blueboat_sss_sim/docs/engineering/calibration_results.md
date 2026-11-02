# Field-calibration results — passes 2 and 3

Pass 3 (2026-11-01 → 2026-11-02, the pool corpus) is **Part II** at the end of this file;
Part I is pass 2, unchanged except where Part II says a pass-2 conclusion moved.

# Part I — pass 2 (2026-10-23 → 2026-10-30)

What the second calibration pass changed, on what evidence, and what it could
not settle. Safe summary: session labels, statistics and constants only; the
recordings, manifest, artifacts, logs and per-stratum scores stay in the
private `~/calib/pass2/` (CM-7; no position leaves it). Engine:
`blueboat_sss_sim/calibration/seabed.py` (`python3 -m blueboat_sss_sim.calibration
score | fit | scan`); procedure for the next pass:
[`validation_and_experiment_plan.md`](validation_and_experiment_plan.md) §6.

## 1. Data and roles

Two sites, seven sessions (operator's split, 2026-10-23, whole-session):
**calibration** `beach_A-2026-09-07-B1` (bath1-3; the NC #6 1.40 m anchor) and
`harbour_A-2026-07-22-PM-cal` (the 7.33 m anchor); **validation** — inspected
within-site hold-outs, never independent — `beach_A-2026-09-07-B2`,
`harbour_A-2026-07-21`, `-07-22-AM`, `-07-22-PM-diffsv` (weakest: same
power-up as the anchor), `-07-23`. No test session exists: pass 1 inspected
every one. Gain index 4 only; 323 strata (file × side × acquisition ×
altitude band), 287 of them "core" (median FBR altitude inside the file's per-side p10-p90 ± 0.1 m). The 2026-10-27/28 recordings are
**reserved, uninspected** (inventory and quality only) for the next pass.

Disclosed reuse: the pulse-length priors (k_d, k_c, the ceiling's pulse
slope) are checked against within-file slopes measured on validation
sessions; they are priors in the candidate's base profile, never artifact
values.

## 2. Method

- **Reduction** (one, both sides of every pair): `scale_to_db`, the
  downstream detector `fbr_bin`, altitude = FBR bin × L/N.
- **Forward model**: flat sand (0.55) at the stratum's FBR-matched altitude,
  the stratum's own range / bins / pulse, gain 4; **attitude replay** (the
  recorded roll, pitch and per-ping FBR altitude, FRD → FLU); `noisy` (the
  node's noise call + `PingEncoder`) and `expected` (noise-free mean
  intensity; the floors' mean `E[max(aX, bY)] = a + b − ab/(a+b)`, smoke
  `[17]`) modes.
- **Metrics** (sim − real): LEVEL = dB of the power average over the tail
  `[max(1.4 alt, 1 m), min(0.725 R, R − 1 m)]` (P1); the detector-located FBR
  crest, median of the per-ping max over `[FBR − 2, FBR + max(1 m, alt/2)]`
  (P2); power-minus-dB, per-bin std, lag-1 texture; nadir; FBR detection.
- **Equivalence**: with `--replay none` the engine reproduces the pass-1
  scorer per stratum to **0.0000 dB** (318 strata).
- **Gates**, written before any fit (hash logged, amended twice before any
  fit): a bundle is promoted only if its primary metric improves on **every**
  calibration and validation session, no session is worse than the 0.83 dB
  real-vs-real floor, the FBR survives (NC #4), and every test suite is green.

## 3. Defects found and fixed (no numeric effect on default renders unless stated)

| Defect | Fix | Test |
|---|---|---|
| 5 registry values silently truncated by an unquoted ` #` | quoted; the loader refuses an unquoted ` #` | `[16]` |
| `svlog_compare` / `calibration_cli` could write anywhere; the old guard checked only the parent folder | `calibration/io_guard.refuse_inside` (inputs' folders and common ancestor, manifest root, corpus) | `[16]` |
| quality report: period judged against one law on mixed acquisitions; no ping-gap / duplicate detection | per-setting period; counter census, `<side>_ping_gaps` / `_duplicate_pings` | `[16]` |
| `[2f]` shading gate vacuous: it asserted the bounds `calibrate_gain` clips to | `calibrate_gain` reports `gain_raw` / `clipped`; the gate pins the measured finding (tie lines do not support the amplitude) | `[2f]`, `[16]` |
| artifacts never checked `base_profile` | name + SHA-256 refused on mismatch unless re-applied deliberately (recorded) | `[16]` |
| `sonar_config:=` did not resolve a bare profile name | resolves against the package `config/` | resolver checks |
| identifiability: one acquisition for every altitude, no site nuisance | per-altitude acquisitions, per-site free levels | `[16]` (pulse exponent ≡ site level, r = −1.00) |
| walled scenes: ghost / single-bounce energy behind a quay was unspeckled | `Ping.speckle_fill` → Exp(1) there (**changes walled-scene renders**) | `[17]` |
| **pitch re-drew the speckle every ping** (track lag-1 0.76 → 0.24 under a 0.4°/ping pitch walk) | the pitch shift moves the beam centre, the lines stay on the scatterer lattice (**changes pitched renders**: wave runs) | `[10]` |
| engine: replay took consecutive stratum pings, seconds apart | strata keep file indices; noisy replay takes the most time-contiguous block | — |
| the promoted receiver ceiling's soft knee let two top bins round to 65535 (0.1 % of Gazebo profiles; the device: never, 68 948 / 68 948) | the encoder gives 65535 only to the true maximum; a ping without a tie is unchanged (found by the new Gazebo sonar-stream check) | `[17]`, Gazebo gate |

## 4. Model terms added (configurable; off = bit-identical)

`pulse_level_exponent` / `specular_pulse_exponent` (level ∝ (τ/66.5 µs)^k),
`receiver_ceiling*` (C¹ soft knee keeping one full-scale bin),
`texture_k_inv_shape` / `texture_k_cell_m` (ground-fixed Gamma field: a
K-distributed seabed), `ping_level_jitter_db`. Physics and status:
`docs/sonar_model.md` §13.

## 5. Fits and decisions, by pre-registered bundle

| Bundle | Fitted on calibration sessions | Gate | Outcome |
|---|---|---|---|
| **A** level law | prior k_d = 1 (within-file slope +0.91 [0.63, 1.15], but leave-one-session-out inconsistent: B2 +0.67, 07-22-AM +1.21); refit offset / TVG / Lambert → 79.65 dB / 0.235 / 0.536 (ridge, r = −0.82; Lambert just above its 0.5 bound) | fails: 07-21 LEVEL 2.15 → 5.17 dB; diffsv noisy LEVEL worse by 1.1 dB | provisional (candidate only) |
| **B** FBR peak | receiver ceiling 67.9 dB (beach pile-up, measured); specular_strength crest fit **1.36** → **1.75** (NC #4 envelope over seagrass); ringing refit (53.2 dB, 9.9 dB/m) **rejected** — it lost the FBR at 5-10 m | **passes**: crest better on every session, LEVEL moved ≤ 0.07 dB, detection ≥ 0.996 | **promoted 2026-10-30** |
| **C** texture | K texture `texture_k_inv_shape` **1.5** (ν ≈ 0.67) by a pre-registered rule over a {0..2.5} × jitter {0, 1.5, 2.5} grid; jitter not supported | fails: 07-22-AM crest +1.4 dB, power-minus-dB overshoots there | provisional (candidate only) |
| **D** edges, **E** walls | not attempted (D needs attitude/heave replay over the field-box pass; E an identified structure) | — | open (`.claude/TODO.md`) |

**Sensitivity of A to the pulse exponent** (pre-registered; the level triple
refitted on the calibration sessions at each k, expected-mode LEVEL):

| k_d | offset / TVG / Lambert (calibration fit) | per-stratum rms (dB) |
|---|---|---|
| 0.67 | 79.50 / 0.252 / 0.597 | 0.893 |
| 1.0 | 79.65 / 0.235 / 0.536 | 0.883 |
| 1.2 | 79.76 / 0.224 / 0.500 | 0.881 |

| Session | baseline | k = 0.67 | k = 1.0 (candidate) | k = 1.2 |
|---|---|---|---|---|
| beach_A-2026-09-07-B1 | +0.57 | +0.09 | +0.07 | +0.05 |
| beach_A-2026-09-07-B2 | +3.06 | +1.73 | +1.24 | +0.94 |
| harbour_A-2026-07-21 | -2.15 | -4.81 | -5.17 | -5.38 |
| harbour_A-2026-07-22-AM | +4.51 | +1.17 | +0.34 | -0.15 |
| harbour_A-2026-07-22-PM-cal | +0.86 | -0.65 | -0.56 | -0.51 |
| harbour_A-2026-07-22-PM-diffsv | +0.47 | -0.92 | -0.81 | -0.74 |
| harbour_A-2026-07-23 | -1.87 | -1.50 | -0.82 | -0.41 |

The calibration sessions cannot tell k apart (rms 0.893 / 0.883 / 0.881 dB).
07-21's failure is **not** k's: it stays at −4.8 to −5.4 dB for every k, and
the k = 0 control refit already read −4.2 dB. It comes from refitting the
level law at all — the 2026-07-21 harbour day reads 2-3 dB brighter than the
07-22 anchor at the same acquisition, a day-to-day site variation as large as
the pulse effect itself.

The first candidate (`cal-2026-10-30-a`: specular 2.5 + refitted ringing)
broke NC #4 on harbour validation strata at 7-21 m (detection down to 0.00).
That failure is why the FBR envelope became a selection constraint; the
candidate was rebuilt as `cal-2026-10-30-b`.

## 6. Results: baseline / promoted default / candidate v1

Each cell: **baseline** (the shipped model before pass 2) / **promoted
default** (bundle B) / **candidate v1** (`config/field_candidate_v1_sonar.yaml`:
A + B′ (specular 6.0, its own NC #4 bound) + C). Bias = sim − real, dB; core
strata, ping-weighted; attitude replay.

| Session (role) | LEVEL, expected mode | LEVEL, noisy mode | FBR crest | power-minus-dB (real) | per-bin std (real) |
|---|---|---|---|---|---|
| beach_A-2026-09-07-B1 (calibration) | +0.57 / +0.47 / +0.07 | +0.25 / +0.15 / -0.62 | +6.75 / +1.81 / +1.56 | 2.17 / 2.12 / 4.13 (4.30) | 4.94 / 4.89 / 6.53 (6.90) |
| beach_A-2026-09-07-B2 (validation) | +3.06 / +2.88 / +1.24 | +2.64 / +2.46 / +0.54 | +9.48 / +1.73 / +1.42 | 2.19 / 2.12 / 4.28 (4.10) | 4.96 / 4.89 / 6.65 (6.72) |
| harbour_A-2026-07-21 (validation) | -2.15 / -2.10 / -5.17 | -1.98 / -1.97 / -5.55 | +7.89 / -0.20 / +1.44 | 2.23 / 2.23 / 4.17 (4.38) | 4.99 / 4.98 / 6.55 (6.64) |
| harbour_A-2026-07-22-AM (validation) | +4.51 / +4.46 / +0.34 | +4.32 / +4.31 / -0.03 | +12.99 / +5.42 / +5.41 | 2.23 / 2.23 / 4.49 (3.34) | 5.00 / 5.00 / 6.83 (6.35) |
| harbour_A-2026-07-22-PM-cal (calibration) | +0.86 / +0.83 / -0.56 | +0.70 / +0.70 / -1.05 | +8.81 / +0.85 / +3.90 | 2.19 / 2.19 / 4.08 (3.89) | 4.96 / 4.96 / 6.53 (6.60) |
| harbour_A-2026-07-22-PM-diffsv (validation) | +0.47 / +0.45 / -0.81 | +0.24 / +0.24 / -1.35 | +9.90 / +1.97 / +5.10 | 2.20 / 2.20 / 4.12 (3.50) | 4.96 / 4.96 / 6.56 (6.50) |
| harbour_A-2026-07-23 (validation) | -1.87 / -1.94 / -0.82 | -2.30 / -2.30 / -1.29 | +5.41 / -1.73 / +2.71 | 2.14 / 2.14 / 3.67 (4.74) | 4.88 / 4.88 / 6.14 (7.14) |

| Acquisition (bins / pulse) | LEVEL, expected mode | FBR crest |
|---|---|---|
| 1200 bins / 119.7 us | -0.41 / -0.47 / -0.56 | +7.53 / +0.35 / +3.66 |
| 1200 bins / 44.3 us | +3.37 / +3.18 / +0.95 | +10.95 / +2.25 / +1.98 |
| 1200 bins / 46.6 us | +4.42 / +4.40 / +2.10 | +6.68 / +2.75 / +2.43 |
| 1200 bins / 51 us | +3.31 / +3.29 / +1.39 | +5.63 / +2.59 / +2.11 |
| 1200 bins / 55.4 us | +2.49 / +2.38 / +1.73 | +8.13 / +1.26 / +1.16 |
| 1200 bins / 62.1 us | +2.98 / +2.90 / +1.92 | +4.43 / +2.65 / +1.20 |
| 1200 bins / 64.3 us | +4.22 / +4.18 / +0.80 | +12.11 / +4.23 / +5.11 |
| 1200 bins / 66.5 us | +2.37 / +2.28 / +1.60 | +4.09 / +2.83 / +1.40 |
| 1200 bins / 86.5 us | -0.33 / -0.56 / -2.25 | +7.81 / +0.02 / +2.30 |
| 600 bins / 106.4 us | +0.61 / +0.58 / -0.73 | +9.53 / +1.58 / +4.69 |
| 600 bins / 117.5 us | -1.94 / -1.97 / -2.70 | +8.32 / +0.43 / +4.11 |
| 600 bins / 212.8 us | -3.52 / -3.60 / -1.13 | +3.02 / -4.07 / +1.63 |
| 600 bins / 66.5 us | +0.49 / +0.39 / -0.12 | +6.78 / +1.74 / +1.53 |

| Altitude regime | LEVEL, expected mode | FBR crest |
|---|---|---|
| high (>=3 m) | +1.37 / +1.34 / -0.77 | +10.14 / +2.35 / +4.47 |
| low (<3 m) | +1.62 / +1.49 / +0.56 | +7.90 / +1.78 / +1.50 |

Minimum simulated FBR detection over all 323 strata: baseline 1.000, promoted default 0.996, candidate v1 0.996.

Reading it:
- **FBR crest** (P2): the promoted default reduces the crest error by 3.7-8.0 dB
  on every session.
- **Seabed level** (P1): the promoted default leaves it where it was. The
  candidate halves or better the 1200-bin offsets (+3.4 → +0.95 dB at
  44.3 µs), but makes 07-21 worse.
- **Texture**: only the candidate moves power-minus-dB and per-bin std onto
  the real values, overshooting on 07-22-AM.

## 7. Verification (2026-10-30)

- smoke: system python **478 PASS**, venv **475 PASS + 1 SKIP** (no ROS), 228-259 s;
  render and encode fixtures rebaselined for the promotion; re-anchored —
  never widened — with dated notes: `[2b]` contrast (now the corpus's own
  per-ping statistic, 31.1 dB vs the corpus median 32), `[2c]` `max_pwr_db`
  anchor 66.10 → 61.20 dB and the peak law checked point by point, the
  ceiling-sensitive `[2e]` / gain-ladder checks taken below the ceiling.
- `test/calibration_fit_test.py`: all pass (round trip, recovery 82.65 / 0.401
  against truth 82.60 / 0.400, refusal, pass-1 equivalence, candidate FBR lock
  — the candidate locks on 0.71-1.00 of 10-ping windows; the device itself
  0.56-0.83 at 0.6 m on the beach files).
- GUI smoke, registry check (75 parameters, 0 drift), `colcon build`: pass;
  `calibration envelope --profile config/field_candidate_v1_sonar.yaml`: 82
  cases, 0 regressions against the promoted default.
- Gazebo gate `--quick` on `flat_5p0m` (calm / current / waves / both), now with
  a sonar-stream check (device invariants on every recorded profile, FBR found
  on >= 95 %): the **candidate passes all 29 checks**; the **promoted
  default** passes all checks, with one recurrence of the documented
  intermittent pose-delivery stall ("fresh pose", 0.17 % of pings over 60 ms
  in one *waves* run) that passed on re-run. The first runs found the
  two-full-scale-bin defect (§3), fixed before these.

## 8. Limitations

- Two sites; every validation session shares a site with a calibration
  session; nothing here is an independent test. The 2026-10-27/28 recordings
  are the first chance of one.
- The pulse-level exponent is not identified by the calibration sessions (one
  pulse each); its prior is supported on average and contradicted per session.
- The specular strength is identified by one site and one altitude band (the
  beach crest is ceiling-limited) and bounded by a synthetic envelope.
- Bottom type is the operator's statement (sand), not sampled.
- The device's port/starboard level imbalance (~1-1.5 dB) and the nadir
  ringing's plateau-then-fall shape are measured, not modelled.
- Noisy-mode LEVEL with the K texture is biased ~0.4 dB low over 120 pings
  (heavy tails, few independent cells); the expected-mode LEVEL is not.

# Part II — pass 3: the pool corpus (2026-11-01 → 2026-11-02)

Safe summary as in Part I: labels, statistics and constants only. The recordings, worlds,
registered tracks, per-run recaps and scores stay in the private `~/calib/pass3/`
(`report_2026-11-02.md` is the full report; `RUNBOOK.md` every command; CM-7: nothing under the
corpus was written, 59 manifest hashes re-verified). Engine: `calibration/scene_replay.py` renders
a recording **ping for ping** at its registered poses in a world folder (bit-identical to the
seabed engine on flat sand, smoke `[16]`); sessions with `regime.forward_model: scene` are skipped
by the flat-sand engine, so every pass-2 reproduction is unchanged.

**Verdict: not "v1 stable"; shipped as the pass-3 candidate** — the default plus one measured
installation change and four defect fixes. Three release criteria fail on the held-out runs (§II.5).

## II.1 Data, recaps, split

The first site with known geometry: an indoor pool, 18 runs (2 SonarView recordings of the empty
pool, 10-27; 16 GCS sessions of our stack, 10-28), 20 m / 1200 bins / 44.3 µs (one 10 m run),
gain 4, FBR altitude ~0.6 m over a 0.90 m painted-concrete floor, one or two pickling weights
(PE shell, cement fill) — one inside a flooded plastic box. Every run has a one-page recap
(`~/calib/pass3/runs/index.md`). Split (operator, after inspecting every run): **FIT** 7 runs
(10-28 11:25–11:56), **VALIDATION** 9 (the 10-27 pair + one line repeated six times, which gives
the pass-to-pass repeatability), **TEST** the 2 two-weight runs — an *inspected* hold-out (the
operator chose to split after phase 1), never fitted or scored before the final step.

The pool was rebuilt from the sonar, not from the stated GPS points: outline 49.52 × 19.39 m
from wall echoes, quay walls 0.20 m above water, both weights placed to a few cm from their
echoes (leave-one-run-out ≤ 0.03 m), tracks = each run's own navigation registered to the walls
(the indoor compass is off by up to ±11° per run). Floor: sand **stand-in** (no painted-concrete
law exists in the model, NC #17); the boxed weight: the box size is a photo estimate. One
unexplained floor artifact is masked everywhere.

## II.2 Defects found and fixed

- **Real recordings' per-ping `vehicle_heading_deg` is not the boat's heading** (−168…+90° off the
  autopilot ATTITUDE yaw, beach and harbour files too): `analysis/backscatter` and
  `svlog_reader.pose_track` now take ATTITUDE yaw (`profile_heading_deg`); smoke `[4c]`.
  Every earlier output of those two tools on a real recording placed profiles with the wrong
  look direction.
- **Contents of a flooded container** (`nested`, new in pass 3) keep the container's transmission
  instead of an opaque column (shadows on 27 % of box passes vs the device's 3 %); smoke `[14]`.
- New material `pe_cement` (an HDPE layer on a concrete backing, a layer-on-solid reflection,
  `sonar_model.md` §12), with its registry entry.
- Not a model defect, but disclosed: the private pool scorer's noise-stage seed was salted per
  process (phase-3 "same seeds" was untrue across invocations; effects ≤ 0.01 dB level, ≤ 0.5 dB
  per object pass); fixed before the final scores.

## II.3 Fits and decisions, by pre-registered bundle (gates written before any fit)

| bundle | outcome |
|---|---|
| **G installation** | **Promoted.** `sensor_depth_m` 0.15 → **0.22** (measured: the rear-most hull holes 0.030 m above the keel in `BlueBoat.dae`, waterline 0.216–0.241 m at 16–22 kg from `BlueBoat.stl` buoyancy, and the pool reproduces every run's FBR altitude with 0.22 m to ±0.03 m); `mount_x_m` 0.0 → **−0.345** (the drawing, vs the hull centre; operator: same holes for every corpus). The pool datum improves on every run-side (+0.07…+0.12 → −0.025…+0.033 m); beach/harbour sessions move ≤ 0.07 dB; NC #4 envelope 82 cases, 0 regressions. Render/encode fixtures rebaselined after the old installation reproduced both bit for bit. `field_candidate_v1` keeps 0.15 / 0.0 (regenerated, values identical) |
| **R ringing shape** | **Model discrepancy, not moved.** The device's ringing is a plateau then a steep fall (−10 dB at 0.09–0.14 m for 8–17 mm bins, 6–11 bins for 33–80 mm bins; plateau −1.3 dB per dB of bin width). A two-regime shape fitted on pool + field calibration profiles takes the pool's near-range rms from 20 to 5–7 dB and makes bath3 (4.7 → 7.0) and the harbour anchor (2.2 → 3.2) worse. The pool water column stays +19…+23 dB |
| **T post-echo tail** | **Refuted as a device kernel.** A wall echo is followed by a ~1 m plateau 1–3 dB under its peak; the boxed weight's tail is 11 dB lower at the same pulse; the tail exists at < 5° plan incidence and grows with range. Inferred: a surface/floor waveguide image comb of the wall's mirror image (an offline image comb with the shipped beam reproduces the growth and extent, 4–8 dB weak). The simulator is 33 dB under the device 0.5–2 m past a pool wall: a structure gap of the renderer's first-order wall multipath, plausibly also behind the harbour quay's 7–10 dB deficit (Part I §8) |
| **L level law** | **Blocked.** With the pool's third altitude, every ridge stays at \|r\| ≥ 0.9998 (offset / pulse exponent / site levels; TVG / Lambert): the pool brings its own site level |
| **O objects** | The box echo is 12.5 dB under the device relative to the local floor and has no extent (0 vs 0.9 m); no physics constant closes it; **discrepancy** (never a per-object gain, NC #17) |
| **C texture** | **Fails; pass-2's C refuted on this floor.** A ground-fixed K texture (1/ν 0.5–1.5, cell 0.05–0.5 m) brings the level spread to the device only by correlating neighbouring bins and pings, which the device does not (lag-1 track real 0.14, default 0.12, K +0.07…+0.42) |

## II.4 Baseline vs candidate on FIT + VALIDATION (sim − real, FIT / VAL)

| metric | real | pass-2 default | pass-3 candidate | repeatability |
|---|---|---|---|---|
| seabed LEVEL (expected) | — | +10.70 / +9.95 dB | same | 0.39 / 0.40 |
| level by slant 1–2 m … 12–16 m | 55.2 … 34.5 dB | +10.8 … −5.0 | same | 0.3–2.1 |
| FBR crest | 64.5 dB | +2.13 / +1.75 | same | 0.12 / 0.07 |
| FBR detection / tracker lock | 0.99 / 0.95 | 1.00 / 0.99 | 1.00 / 1.00 | 0.01 / 0.11 |
| water column 0.3–0.5 m | 25.8 dB | +22.8 / +18.9 | same | 3.5 / 1.7 |
| texture per-bin std (2–6 m) | 6.29 dB | −1.16 / −1.40 | −1.18 / −1.40 | 0.29 / 0.13 |
| wall face / 0.5–2 m beyond | +26.3 / +12.7 dB | −3.6 / −33.6 | −3.7 / −33.6 | 0.7 / 0.8 |
| box echo excess (per pass) | 20.9 dB | −12.5 | −12.5 | 1.4 |
| box shadow found | 3 % | 27 % | 21 % | — |

The pool replay uses the measured installation in both columns, so G shows only on the beach/
harbour sessions (≤ 0.07 dB) and in the datum. The pool's seabed level and its 15 dB range shape
are the painted-concrete floor against a sand stand-in [inferred]: a per-site nuisance, never fitted
into the sand constants.

## II.5 Held-out test (2 runs, 4 run-sides, scored once with the frozen candidate)

| metric | bias | release tolerance | |
|---|---|---|---|
| seabed LEVEL (expected) | +8.8 … +10.1 dB | ±0.40 dB (repeatability) | **fail** |
| FBR crest | +1.5 … +1.8 dB | ±0.12 dB | **fail** |
| FBR detection / lock | 1.00 / 1.00 (device 1.00 / 1.00) | ≥ 0.95 / ≥ device − 0.11 | pass |
| objects detected (≥ 10 dB) | box 1/8 (device 7/8), bare weight 4/7 (7/7) | all, at the right range (±0.30 m) | **fail** (range −0.07…−0.12 m where detected) |
| shadow found | 38–43 % (device 0 %) | within 10 points | **fail** |

TEST behaves like FIT/VAL (level within their +9.1…+12.7 dB spread but for one side 0.3 dB
below, crest within +1.3…+4.0), so the failures are model properties, not a held-out surprise.
The other release criteria pass: NC #4 envelope, no beach/harbour regression, smoke (system 483,
venv 480 + 1 SKIP), GUI smoke, registry (76, 0 drift), `colcon build`, every changed constant
registered with its evidence; the Gazebo gate `--quick` on `flat_5p0m` (29 / 29 checks).

## II.6 What is now measured, what is not

Measured: the transducer depth (0.22 ± 0.03 m); the ringing shape across nine acquisitions; the
10 m vs 20 m bin-width level step (−2…−3 dB at a fixed pulse; the renderer is bin-count invariant);
the wall tail and its range dependence; the pool floor's level and angular shape; a texture heavier
than Exp(1) with no extra correlation; repeatability floors on a repeated line.
Unidentified: the level law (L), the fore/aft acoustic centre (drawing −0.345 m vs the pool's
−0.10…−0.21 m from the hull centre with the GPS where the operator put it), the painted floor's law,
the box's construction, the mechanism of the per-sample texture.

**Single most valuable next experiment**: one uniform sand bottom, an altitude sweep (~1 → 8 m) at
the shipped acquisition (20 m / 1200 / 50 ms, gain 4; plus 10 m range for a second bin width),
lines in both directions, with the pool's two weights on the sand at a surveyed point. One site
level over many altitudes and two bin widths breaks the L ridge. The same objects on a known
bottom separate object physics from floor material (O). The pool's acquisition in open water
separates environment from acquisition in the near range (R). Reciprocal passes over the targets
measure the acoustic centre. The coming environment runs can add a quay or pontoon at two water
depths (the waveguide-tail prediction), a dock measurement of the transducers' depth with the GPS
antenna position noted, a same-session SonarView recording, and the bottom type at each target.

