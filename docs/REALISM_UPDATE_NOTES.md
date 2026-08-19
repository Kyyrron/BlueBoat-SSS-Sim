# Realism update notes — dual Omniscan 450 tuning request

Drop-in file replacements for `blueboat_sss_sim` (paths relative to the
package root). Rebuild and **regenerate the mission bundle** (the bundle
carries a copy of `sonar.yaml`, so old bundles keep the old model):

```bash
colcon build --packages-select blueboat_sss_sim && source install/setup.bash
ros2 run blueboat_sss_sim generate_mission --config ... --out ~/runs/r2
```

Each task was analyzed against the ground truth available in this project
(the decoded field capture from the original build session), not taken at
face value. Verdicts first, then the changes.

## Analysis verdicts

**Task 1 (power scale) — request confirmed, now calibrated.** The real
capture decoded during the original build spans `pwr_results` 0–62 000
counts with `max_pwr_db` 63.9 / `min_pwr_db` 7.0; the sim peaked at
~2 600–4 400. Fix: `base_scale` 18 000 → **110 000**, calibrated so the
flat-seabed FBR averages 15–25 k counts and speckle maxima
approach/occasionally clip 65 535 — exactly the real device's behaviour
near saturation. Verified: simulated per-ping `max_pwr_db` mean **63.9 dB
vs 63.9 dB decoded from the field frame**; counts span 5–65 535. The FBR
*structure* part of the task was already fixed in the previous update
(specular lobe + sidelobe floor) and re-verified here: 15–16 dB
gap→peak contrast, naive bootstrap locks on the exact altitude bin.

**Task 2 (20 Hz cap) — request conflicts with the team's own field
data.** The 22 ms free-run period was not a sim invention: it was decoded
from the real capture's device `timestamp_ms` deltas at 15 m (45 Hz).
The spec-sheet says ≤20 Hz. Both cannot be right, and I cannot resolve
which from here (possibilities: spec is a max-range guarantee, firmware
differences, or the spec being misread). Resolution: the cap is a
**configurable model parameter** `max_ping_rate_hz`, default **20**
(period = 50 ms at 15 m, meeting the request's acceptance), always
floored by the physical 2R/c + processing; **set 0 to disable and
reproduce the captured device behaviour**. Threaded through the node
timer and the reported `pulse_duration_sec`.

**Task 3 (0.5° azimuth beam) — confirmed, implemented.** The renderer now
integrates `alongtrack_beam_lines` (default 5) parallel ground lines
spanning the azimuth footprint, each sample Gaussian-weighted with
σ(R) = R·θ/2.355. Verified with a 30 cm point-like target: along-track
extent 30 cm → **42 cm at 13 m** (footprint ≈ 11 cm) with properly
reduced peak contrast (beam averaging), while near-nadir stays sharp.
`alongtrack_beam_lines: 1` restores the legacy delta beam.

**Task 4 (Rayleigh speckle) — request's premise is wrong; confirmed
present, not duplicated.** Multiplicative `Exp(1)` *intensity* speckle —
i.e. fully-developed speckle with Rayleigh-distributed amplitude — has
been applied to ensonified bins since v1 (`sonar/noise.py`); the per-bin
mean is never emitted clean. Verified: flat-seabed CV = σ/μ ≈ 1.0–1.2,
the Exp(1) signature. Added `speckle_looks` (multi-look Gamma(L, 1/L),
default 1) as the requested tunability knob.

**Task 5 (vertical aperture) — confirmed.** 55° → **50°** (spec beam
height). FBR behaviour unaffected (the sidelobe floor governs nadir).

**Task 6 (1/1200 bins) — exposed a real bug beyond the ask.**
`num_results` was already a run parameter and the raw framing is
size-generic, but analysis found the renderer's fixed 5 cm ground step
left **49.5% of far-range bins empty** even at 600 bins (masked by the
noise floor) and would have crippled 1200-bin mode. The step is now
coupled to the bin size (`min(sample_step_m, 0.45·bin)`): 0% empty bins
at both 600 and 1200; 1200-bin frames verified at 2 462 B. Cost:
~1.3 ms/ping (5 lines, 600 bins) — far under the dual-channel budget.

**Task 7 (shallow multipath) — implemented minimally, off by default.**
`multipath_enabled` + `multipath_gain` (0.12) add the classic
second-bottom-echo ghost: the direct response re-imaged displaced
+altitude in slant (bottom–surface–bottom path), boundary losses lumped
into the gain. Verified ghost onset at exactly 2×altitude. *Wall*
multipath stays on the roadmap.

**Notes compliance.** Radiometric/link degradations the request defers to
the downstream augmentation stage are now **off by default** in the base
model: `gain_drift_amp: 0`, `dropped_ping_prob: 0` (code retained; set
> 0 to re-enable). Roll–beam coupling stays: it is pose-driven physics,
not augmentation. Carrier frequency and dual-side layout untouched, as
required. `noise_floor` 0.004 → 0.002 to widen the dynamic range toward
the capture's `min_pwr_db` 7 dB.

## Files changed

| File | Change |
|---|---|
| `blueboat_sss/sonar/config.py` | aperture 50°, `max_ping_rate_hz`, `alongtrack_beam_lines`, `speckle_looks`, `multipath_*`, `base_scale` 110 000, noise defaults; rate-capped `ping_period_s`/`pulse_duration_s` |
| `blueboat_sss/sonar/renderer.py` | bin-coupled sampling step; K-line azimuth-beam integration; optional multipath ghost; contact indexing on the actual step |
| `blueboat_sss/sonar/noise.py` | `speckle_looks` multi-look Gamma speckle |
| `blueboat_sss/sonar/encoder.py` | `pulse_duration_sec` uses the capped period |
| `blueboat_sss/ros/sss_sim_node.py` | ping timer uses the capped period |
| `config/default_sonar.yaml` | all new keys, calibrated values, documented 20 Hz-vs-22 ms tension |
| `test/smoke_test.py` | new sections 2c (rate cap, uncapped 22 ms, power scale vs capture, speckle PDF, no empty bins) and 2d (azimuth widening, 1200-bin end-to-end); 32/32 pass |
| `docs/sonar_model.md`, `docs/configuration_guide.md` | updated model description, calibration, timing tension, multipath, high-res bins |

## Acceptance vs request

* FBR: locks within a few pings (bootstrap check on the default mission);
  power histogram anchored to the capture (`max_pwr_db` 63.9 ≈ 63.9). ✓
* Ping period 50 ms ≥ 50 ms at 15 m with the default cap; along-track
  spacing = v/PRF. ✓ (escape hatch documented)
* Point objects blur along-track ∝ range; near-nadir sharp. ✓
* Flat-seabed intensity PDF is Rayleigh-like (CV ≈ 1). ✓ (pre-existing)
* `vertical_aperture_deg` = 50. ✓
* `num_results: 1200` works end-to-end, and the sampling bug that would
  have broken it is fixed. ✓
* Optional multipath term, off by default. ✓
