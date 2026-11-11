# Ray-cast sensor — Phase A report: first fit and go / no-go

Phase A follows the owner-approved reorder of the plan (`raycast_plan.md`, 2026-11-03): after Phase 2
(the CPU reference) comes a first fit and a go / no-go on the validation sessions. The remaining
phases follow only after that. The test hold-out stays sealed until Phase 7.

## What was frozen before any result

- `~/calib/raycast/gates.yaml` (sha256 in `raycast_plan.md`):
  - the five fitted parameters and the engine settings;
  - the statistics, with their tolerances and their motivating role;
  - the go items and the blind test.
- `~/calib/raycast/split.yaml`: calibration, validation and hold-out sessions, the spent pool test
  runs, and the paths the tools refuse.
- Amendments, each written and hashed before the result it concerns:
  1. `harbour_quay_face` becomes available, with its rule (the harbour world, the tide datum).
  2. The pool statistics: how a session value is formed from the p4_score readouts, and the
     bootstrap unit (blocks of 8 wall pings per run-side, or box passes, paired across renderers).
  3. `harbour_quay_face`'s bootstrap unit, and the conservative reading of the range kernel: it
     improves only when both `range_corr_2` and `range_corr_3` improve, and is worse when either
     is worse.
  4. The owner's decision of 2026-11-04: the bounds of `raycast.seabed.sediments.sand.gamma2` move
     from [2.000001, 3.92] to [2.6, 3.92]. The J&R small-slope law's I_K quadrature extends to
     U = (40/q)^(1/(γ2−2)): about 1.3 GB per call at 2.5, 58 GB at 2.4, and not computable below.
     The fit's least squares had moved γ2 toward the old bound and exhausted the host.

## Pipeline (tools in `~/ros2_ws/sim_runs/raycast/tools/`)

| Step | Tool | Output |
|---|---|---|
| fit bases (calibration flat strata, expected mode, attitude replay) | `phaseA_bases.py` (one spawned worker) | `~/calib/raycast/phaseA/bases/` |
| fit (level then nadir, alternately, until no parameter moves by more than 1 % of its bound width; at most 6 rounds) | `phaseA_fit.py` via `phaseA_fit_run.sh` | `fit.json` (rounds, fitted section, parameter report) |
| flat validation (harbour validation core strata; noisy mode; 120 pings per seed; seeds 7 and 8) | `phaseA_validate.py render raycast FIT.json`, `score` | `validation.json` |
| NC #4 (FBR envelope of the fitted set vs the geometric renderer; sand and mud) | `phaseA_nc4.py` | `nc4.json` |
| harbour quay face (two validation recordings, tide-datum worlds) | `harbour_quay.py raycast FIT.json` | `~/calib/raycast/harbour_quay/` |
| pool statistics (p4_score rules; the split's pool validation runs) | `pool_score.py --renderer geometric / raycast --fit FIT.json`, `pool_verdict.py` | `pool/`, `pool_verdict.json` |
| go / no-go | `phaseA_gonogo.py`, `phaseA_report.py` | `gonogo.json`, `report.md` |
| blind test material (20 crops, key apart) | `phaseA_blind_crops.py FIT.json` (+ `blind_test.py`) | `blind/` |

`phaseA_post_fit.sh` runs every step after the fit in sequence, one GPU process at a time.

## Engineering found on the way

### The fit's memory

The fit was killed by the host's memory seven times. Each cause was found by measurement and
fixed without shrinking the fit:
1. **Every basis was built twice.** `calib._geometry_rc` removed the fitted keys but left their empty
   parent mappings. `set_params` adds `beam: {vertical_floor_amp}`, so the fit and the builder
   hashed the same flat scene to different keys. Keys are now normalised, and the stored bases were
   hard-linked to the normalised names (`phaseA_migrate_keys.py`).
2. **Bases held at full precision dtypes.** `fitbasis.compact_basis` keeps `power()` bit for bit and
   uses 2.50 → 2.03 MB per basis on a sample. The fit's 2352 bases total **7.18 GB** compact.
3. **Renderers are reference cycles.** The 63 flat renderers built for the law contexts stayed
   resident until a full collection (~6 GB). `calib` now collects whenever it drops a renderer.
4. **glibc's dynamic mmap threshold** fragmented the heap. Long jobs run with
   `MALLOC_MMAP_THRESHOLD_=131072 MALLOC_ARENA_MAX=2`.
5. **The γ2 bound** (amendment 4).

The fit now holds a steady 7.6 GB, and one evaluation of the 63 strata takes ~15 s.

### Shared GPU

Basis builds and scoring renders wait and retry on a transient device out-of-memory
(`calib._build_retrying`). Warp reports a failed scan temporary as "CUDA error 101".

## Attempt 1 (the frozen protocol, amendments 1-4): NO-GO

**Fit.** Six rounds, alternating level and nadir, did not converge: the alternation cycles through the
same three parameter sets (round 3 = round 0 nadir, round 4 = round 1, round 5 = round 2). The final
section is the round-5 nadir solution:

| parameter | value | bounds |
|---|---|---|
| system gain | 140.56 dB | [105, 165] |
| TVG k | 40.0 | [0, 40] — on the bound |
| sand w2 | 1.475e-4 | [3.56e-7, 1.475e-4] — on the bound |
| sand γ2 | 2.6 | [2.6, 3.92] — on the bound |
| vertical floor f_v | 0 | [0, 0.3] — on the bound |

`params_in_bounds` FAILS. The level metric's best (round 0) was rms 7.37 → 3.02 dB with γ2 and f_v
already on their bounds.

**Flat validation** (45 harbour strata, 3 sessions, of the final set): the ray-cast sensor is outside
tolerance on every statistic of every session, and worse than the geometric renderer on 9 of 10
(level_tail e +28.3 dB vs +1.1; range_corr_1..3 +0.56..+0.72 vs |e| ≤ 0.17; `k_log10` no worse).
Harbour quay face, first recording: real +8.36 dB, ray-cast +0.41 (e −7.95; geometric +3.91). The
rest of the chain was stopped: its outcome was decided.

### Diagnosis

1. **The nadir metric is degenerate for this sensor.** It covers slant ≤ min(0.6 m, 0.7 × altitude),
   the water column before the first bottom return. There the simulated level is the measured
   transmit ringing plus fixed floors, and +10 % on each fitted parameter changes the residual by
   exactly 0.000. Least squares with `x_scale='jac'` on a zero Jacobian takes unbounded steps: the
   parameters jumped to their bounds, and the next level step pulled them back.
2. **The range-correlation failure is saturation, not texture.** Under the final set the rows sit
   near the receiver ceiling (mean ~60 dB against 18–40 dB real). The residual about the per-bin mean
   is then a per-ping offset, which is fully correlated along range (0.99 in every stratum). With
   the start profile, the pool texture was close to real.
3. **The level fit's real gap is the sea surface.** The ray-cast default `paths.surface_rms_m` 0.02 m
   (kσ ≈ 37 at 450 kHz) leaves no coherent surface multiple. On the shallow beach strata (0.67 m
   altitude, ~0.9 m of water) the far range then lacks 10–15 dB. A 2 mm surface restores it:

   | slant | real | rms 0.02 m | rms 2 mm |
   |---|---|---|---|
   | 9 m | 38.8 | 31.9 | 39.6 |
   | 12 m | 32.9 | 22.8 | 30.2 |
   | 15 m | 30.5 | 15.3 | 29.3 |

   Without that multipath the fit bent the roughness spectrum (γ2, f_v on bounds) to raise the far
   range. Lambert's law is not the fix: on 16 strata it fits about as well as the J&R law (rms 1.05
   vs 1.08), it misses the same low grazing angles, and it is 9–20 dB too bright above 1 m altitude.
4. **The pool.**
   - The pool worlds carry a sand floor. Every floor bounce therefore follows the 450 kHz sand
     roughness rule (incoherent), and the diffuse floor is sand-bright. The real floor is painted
     concrete (pass 4: −16.4 dB re sand, smooth).
   - With the calm pool surface (3e-4 m) the wall tail behind the face rises from −33 to −12 dB, but
     the real value is +13.9 dB.
   - A calm-surface pool render costs minutes: 544 s under GPU contention, 524 bound sequences, and
     19 surface/floor chains kept with Newton solves on the textured floor.

Bugs found on the way:
- `calibration.seabed._envelope_job` never passed the ray-cast section, so NC4 compared the
  geometric envelope with itself: a false PASS.
- `analysis.raycast_compare.compare` gave the point estimate the per-renderer detection mapping.

## Attempt 2 (amendment 5)

- Level-only fit.
- Per-stratum multipath correction from full renders at the site's sea-surface roughness:
  - beach 2 mm and harbour 1 mm, chosen by a scan on their own calibration strata (mean |level
    bias| 3.25 → 2.35 dB on the beach, 4.91 → 3.78 dB on the harbour, at attempt 1's round-0 values);
  - pool 3e-4 m (pass 4).
- The validation renders carry the site surface physically.
- The pool statistics are not evaluated in attempt 2 (render cost; floor description).

**Result: INVALID as a validation** (2026-11-06). The flat-validation rows were rendered before two
engine defects were fixed:
- the attitude replay applied the altitude variation as the vehicle's z, lifting the 0.22 m-deep
  transducer out of the water (empty pings);
- the FBR altitude match fired on the far-range level and clamped strata to 0.2 m.

Both are fixed for the ray-cast only. The attempt-2 rows are archived as INVALID
(`~/calib/raycast/archive/2026-11-06_stage1/`).

The fit itself (gain 129.4, TVG k 6.65, w2 2.36e-5, γ2 2.6, f_v 0) lies outside the SSA's domain:
J&R 13.18 normal-incidence energy −32.8 dB.

The near-normal verification is `raycast_nearnormal_verification.md`. The owner's redirect to a
hybrid model and the bounded beam + energy test (Stage 2) are reported in `raycast_assessment.md`.

**Tests rerun after the fixes:**
- smoke 493 ALL PASS;
- the full `--quick` raycast suite: 64 PASS / 0 FAIL / 2 SKIP, over two runs (G1 included).
- New checks:
  - [S-E] energy validity;
  - [CL1]–[CL5] correction layer.
- Still open: NC4 and the harbour quay hit CUDA error 101 in attempt 2 (not yet investigated).
