# Sonar model

How the synthetic Omniscan 450 imagery is produced, what is modeled, what
is deliberately not, and which parameter controls what. Implementation:
`blueboat_sss_sim/sonar/`.

## 1. Geometry

For each ping and each side, the transducer pose is derived from
`/blueboat/odom`: mount offset (`mount_x_m`, ±`mount_y_abs_m`) rotated by
yaw, transducer at `sensor_depth_m` below the surface. The renderer casts
athwartship ground lines perpendicular to the heading (side sign:
port = +y in body frame), sampling the scene heightfield at a step
automatically coupled to the slant-bin size (`min(sample_step_m,
0.45·bin)`), so every range bin is populated at any `num_results`
(600 → 25 mm bins, 1200 → 12.5 mm bins).

**Azimuth (along-track) beam.** The Omniscan's 0.5° along-track beam is
integrated by rendering `alongtrack_beam_lines` (default 5) parallel
ground lines spanning the azimuth footprint at max range; every sample is
weighted by a Gaussian in its along-track offset with σ(R) = R·θ/2.355.
Consequences match the real beam: point-like targets smear along-track
proportionally to range (verified: a 30 cm target reads ~30 cm at close
range and ~42 cm at 13 m), sub-footprint targets lose contrast through
beam averaging, and near-nadir stays sharp. `alongtrack_beam_lines: 1`
restores the legacy infinitesimal-beam behaviour.

Per sample: slant range `R = √(y² + Δz²)`, depression angle
`δ = atan2(Δz, y)`, and local incidence angle from the along-line terrain
slope. Altitude is the water depth under the transducer; the water column
(`R <` altitude) stays empty except for additive noise, producing the nadir
gap.

## 2. Shadows — horizon culling

A sample is insonified iff its elevation angle (as seen from the
transducer) exceeds the running maximum over all nearer samples
(`np.maximum.accumulate`). This single mechanism produces geometrically
correct acoustic shadows behind proud objects, dune crests, and rocks — the
length of a shadow automatically obeys `L ≈ h·R / altitude`. The same
visibility array feeds the ground-truth contact annotations
(`visible` flag, `shadow_bins`).

## 3. Intensity model

Per insonified sample, echo power is the product of:

| Term | Formula | Config |
|---|---|---|
| Backscatter | `ρ · cos(θᵢ)^p` (Lambert-like) | material `reflectivity`, `lambert_exp` (material) or `lambert_exponent` (global) |
| Specular lobe | `S · (0.5+0.5ρ) · exp(−θₙ²/2σ²)`, θₙ = angle from the surface normal — dominates at/near nadir and produces the bright **first bottom return** that downstream FBR / bottom tracking locks onto. S (30) is sized so the FBR clears a +8 dB noise-floor threshold even over dark mud/seagrass; the field corpus does not determine it, because a harbour's near-nadir return is set by whatever lies under the boat (§6). Rendered as a **separate component** from the diffuse field because its fluctuation statistics differ (§5) | `specular_strength` (S), `specular_width_deg` (σ), `specular_looks` |
| Vertical beam pattern | Gaussian in depression angle, centered on `beam_tilt_deg` + roll toward that side, FWHM = `vertical_aperture_deg` (50°, the Omniscan 450 spec beam height), floored at `beam_sidelobe_floor` (real sidelobes; without the floor, near-nadir rays ~70° off-axis at shallow altitudes would be attenuated below the water-column noise and the FBR would vanish) | mounting section |
| Residual range response | `(10^(−α·2R/10) / R^(2k))^(1−c)` — two-way absorption `α` and spreading `k`, partially undone by TVG fraction `c` | `absorption_db_per_m`, `spreading_exponent`, `tvg_compensation` |

Samples are accumulated into the `num_results` slant-range bins with
`np.bincount` and normalized by per-bin hit counts, i.e. an unbiased mean
per bin — the discrete analogue of the intra-bin ensonified average.

**Transmit ringing.** Every real ping additionally carries the decaying
transmit-pulse core right under the transducer — measured on
`diffDepthCompensation.svlog` at ~55 dB at the transducer, through 43 dB at
0.27 m, to ~33 dB by 1 m, brighter than any seabed return in the file (this
artefact is what the GCS's 0.75 m nadir blank and the processor's
ringing-settle logic exist for). Modelled as an exponential decay specified
in the reported-dB domain at the calibration gain (`ringing_level_db`,
`ringing_decay_db_per_m`; level 0 disables) and added to the **coherent**
channel before noise, so it fluctuates with the low-CV `specular_looks`
statistic like the real ringing. It lives inside the water column at survey
altitudes: the FBR, the level anchor (§6) and NC #4 are untouched — smoke
`[2b]` asserts the level, the settle distance, and that the
processor-style bootstrap still locks at the altitude.

`tvg_compensation = 1.0` reproduces a perfectly flattened image. The
default is **0.0**: the profile stream the device reports is measurably
pre-TVG (§6), so the full two-way range loss is present in the data, and at
`spreading_exponent: 4.0` that is ~40 dB per decade of slant range —
about 24 dB across the swath at the 15 m / 4 m-altitude default. That
gradient is the device's, not a defect. Downstream viewers remove its
smooth `log10(range)` part for display before stretching, which is what a
viewer's TVG does; the wire stream is left exactly as the device would
have sent it.

## 4. Surface-vehicle attitude coupling

Roll rotates the vertical beam pattern; a rolling USV therefore brightens
one side and dims the other ping-by-ping, producing the along-track
banding characteristic of surface platforms (a core concern of the thesis
regime). Heave enters through the transducer height (altitude, slant
ranges, water column). **Pitch** (since 2026-09-03, with the sea-state
model that finally makes the hull pitch) rotates the fan plane about the
body y axis: a ground sample `dz` below the transducer is imaged
`dz·tan(pitch)` along-track from where a level fan would put it — the
symmetric port/starboard along-track distortion of a pitching platform
(Cobra, Oppenheim & Jaffe 1992), implemented as one refinement pass of the
ground line in `_ping_geometry`; zero pitch is bit-identical. The mount
lever arm is rotated by yaw only: at ±0.20 m athwartship a 10° roll moves
the transducer 3 mm, below the ground-sample step. There is no intra-ping
motion (§7, A5). The attitude itself comes from `/blueboat/odom`, i.e.
from Gazebo under the wave wrench (`docs/sea_state_model.md`).

## 5. Noise model (`sonar/noise.py`)

Applied in order, all configurable:

1. **Speckle, per component** — the diffuse field gets multiplicative
   `Exp(1)` intensity speckle: fully developed (Rayleigh amplitude), the
   textbook single-look statistic. Since 2026-09-02 that statistic comes
   from a **coherent ground-scatterer field** (`coherent_scatterers: true`,
   `renderer._texture`): every ground cell (`scatterer_cell_m`, 1 cm) carries
   a random phase fixed by a hash of its cell index and the world seed, the
   diffuse contributions of one bin are summed as phasors, the pulse /
   receive envelope is convolved on the *complex* field, and the unit-mean
   ratio of `|field|²` to its exact expectation is the multiplicative texture
   the noise stage applies (`Ping.texture`). The per-bin statistic is
   unchanged — CV 1, 5.6 dB high-pass std — but the texture is now
   **correlated**: along range by the kernel (lag-1 ≈ 0.35 for the 3-bin
   floor) and along track because consecutive pings re-illuminate the same
   cells with different footprint weights (the azimuth lines are snapped to
   a ground lattice along the heading for exactly that), which is what the
   corpus measures (lag-1 0.30–0.68 along range, 0.36–0.66 along track,
   `sss_svlog_compare`). `Ping.power` is still the noiseless mean, so every
   mean-based statistic and `speckle: false` are untouched.
   `speckle_looks: L > 1` gives multi-look `Gamma(L, 1/L)` on the legacy
   per-bin path. The **coherent specular** component gets
   `Gamma(specular_looks, 1/looks)` (CV ≈ 0.2 at the default 25): the real
   near-nadir echo is Rician with a high K-factor and fluctuates far less
   than diffuse speckle — this is what makes the first bottom return a
   stable ping-to-ping feature that bottom tracking can lock onto.
   Verified: flat-seabed CV ≈ 1; a persistence-threshold FBR detector run
   on consecutive moving pings finds 100% of 10-ping windows within a
   0.30 m band.
1b. **Pulse smearing** — each ping is convolved with the transmit-pulse
   range envelope (width `c·τ/2`, where τ follows the device law: 1.2
   sample periods at 600 bins, 1.5 at 1200, 44.3 µs floor — measured on
   every `pulse_duration_sec` in the corpus), floored at
   `range_filter_bins` (3) for the receive chain: every scatterer is a
   multi-bin feature and neighbouring bins are correlated, as at the real
   matched-filter output (`pulse_smearing: true`).
2. **Water-column noise** — volume reverberation inside the nadir gap
   (`watercolumn_noise`, **1.15 × 10⁻⁷**). **Measured** (2026-09-02): the
   1–3 m band of the gain-4 corpus pings reads 25.3 dB reported (mean of
   dB; 27.8 dB as a linear mean, which the constant is), 40 dB under the
   FBR peak, so downstream bottom tracking locks with margin.
3. **Receiver noise floor** — additive everywhere (`noise_floor`,
   **8.3 × 10⁻¹¹**). **Measured** separately: the far decile of the gain-4
   pings of the 72–126 m recordings, where nothing but noise is left, reads
   −6.1 dB reported (−3.6 dB linear mean).

   Both were previously one jointly fitted number (6.79 × 10⁻⁷, i.e.
   38.5 dB reported) chosen to put the water column 26 dB below the bottom
   return on files that never reached the receiver floor. That value hid
   the range law beyond ~20 m: the simulated far range sat flat on a floor
   ~40 dB above the device's, the per-ping span was 32 dB against the
   corpus's 63–69 dB, and a 30 m coverage-pass image looked nothing like a
   field one. With the measured constants the simulated gain-4 profile at
   **matched altitude** (the corpus flies at 6–9 m) follows the real one
   within ~1 dB from 10 to 35 m on the 40 m and 72 m recordings.
4. **Gain drift** — slow multiplicative sinusoid + random walk
   (`gain_drift_amp`, `gain_drift_period_s`). **Default 0**: radiometric
   drift/banding is deferred to the downstream augmentation stage; set
   > 0 to re-enable in the base model.
5. **Dropped pings** — Bernoulli per ping (`dropped_ping_prob`,
   **default 0**, same deferral); when enabled the ping counter still
   advances, as with the real device.

## 6. Quantization and encoding (`sonar/encoder.py`)

**`pwr_results` is not counts.** The Omniscan rescales every ping onto its
own dB axis and reports the endpoints in `min_pwr_db` / `max_pwr_db`;
downstream inverts it with the Cerulean template

```
db = min_pwr_db + (raw / 65535) · (max_pwr_db − min_pwr_db)
```

which is what `sss_processor_node` and the GCS both apply to every profile.
Three invariants follow, and the field corpus satisfies all three on
**68 948 / 68 948 pings** across seven recordings and two recorders: exactly
one bin at 65535, a minimum of exactly 0, and the span clamped at
`max_span_db` (90 dB, hit exactly on 8.39 % of pings and never exceeded).
The encoder reproduces them, and smoke `[2c]` / `[2f]` assert them on both
sides. Emitting absolute counts instead puts a linear-power vector on the
wire under a dB label, and every consumer mis-inverts it.

The reported level is

```
db = 10·log10(power) + calibration_db_offset + 20·log10(analog_gain / analog_gain[4])
```

so **receive gain moves the reported dB and leaves the counts alone**, as on
the device. The ladder is an *amplitude* gain, hence `20·log10`: the
receiver floor in the corpus steps +5.6 / +4.6 / +5.7 dB across indices
4→5→6→7, exactly 20·log10 of the table ratios (a 10·log10 step, the earlier
assumption, gives +2.8 and does not reproduce the shift). `ANALOG_GAIN_TABLE`
carries the ladder: indices **4–7 are
measured** (74.55 / 142.8 / 242.025 / 464.625 — the only values the corpus
ever reports, and its auto-gain walks all four inside every recording);
0–3 are unmeasured estimates. Indices outside 0–7 are rejected when the
parameters are read, since a silent fallback would misreport `analog_gain`
and `max_pwr_db`. `AcquisitionParams.effective_gain_index` resolves the real
device's `-1` auto-gain sentinel to the calibrated index 4, because the
profile's `gain_index` field is `uint16` and cannot carry `-1`.

Frames are byte-exact Ping Protocol
(`BR | len | 2198 | src | dst | 52-byte fixed payload | u16[n] | checksum`);
`parse_frame()` round-trips them and the smoke test verifies length parity
with the field capture (1262 B at n = 600, 2462 B at 1200). **The byte
layout did not change** when the count semantics were corrected.

### Which constants are measured, and against what

Against the **Shiraishi-jima harbour corpus** (Kasaoka, Seto Inland Sea):
7 recordings, 68 948 profile pings, range settings 18–126 m set live by the
operator, the device's own auto-gain throughout, bottom type **not
sampled**. `~/ros2_ws/data/SSS_data`, read-only (CM-7).

| Constant | Value | Status |
|---|---|---|
| `max_span_db` | 90.0 | **Measured**, directly and exactly. |
| `ANALOG_GAIN_TABLE[4..7]` | 74.55 / 142.8 / 242.025 / 464.625 | **Measured**, directly and exactly. |
| `spreading_exponent` | 4.0 | **Jointly determined** with `tvg_compensation`: only the product `spreading_exponent · (1 − tvg_compensation)` enters the curve. The split is fixed by taking spreading at its physical two-way-intensity value. |
| `tvg_compensation` | 0.0 | 〃 — which makes the reported stream pre-TVG. The corpus falls **40–52 dB per decade** of slant range at four range settings; the previous `0.90` left about **2**. |
| `noise_floor`, `watercolumn_noise` | 6.79 × 10⁻⁷ | **Jointly determined**, as one number, with each other and with the range law (§5). |
| `calibration_db_offset` | 97.2 | **Measured up to the device's auto-gain regulation.** Ping-weighted over 14 (range, altitude) cells at gain index 4; the cell spread is **11.9 dB**, because the AGC holds the real level roughly constant while the model's follows `r⁻⁴`. Treat as ±6 dB. |
| `lambert_exponent` | 1.0 | **Not determined.** The pooled residual is flat from 0.0 to 1.0, and the per-altitude preference reverses with altitude. Held at Lambert's law proper; confounded with the unsampled bottom type. |
| `ringing_level_db`, `ringing_decay_db_per_m` | 55.0, 22.0 | **Measured** on `diffDepthCompensation.svlog`: ~55 dB at the transducer decaying to ~33 dB by 1 m (22 dB over the first metre). Cross-checked by `sss_calibration_report`'s nadir-profile section (real 54.4 dB at bin 0 vs simulated 51.8; the 1 m point is floor-limited on the sim side because `noise_floor` is fitted to the Shiraishi corpus, whose water column is louder than this log's). |
| `beam_tilt_deg` | 20.0 | **Corpus-constrained** (the BlueBoat bracket tilt is unpublished): `sss_calibration_report --fit-tilt` scans 5–40° against `diffDepthCompensation.svlog` and the residual minimum lands on 20.0° (5.88 dB RMS best of 11 scored tilts, 2026-09-01). |
| `absorption_db_per_m` | 0.10 | **Not separated** from the spreading term. Held at the physical 450 kHz seawater value. |
| `specular_strength`, `specular_width_deg`, `specular_looks` | 30.0 / 8.0 / 25 | **Not determined.** The near-nadir return in a harbour is set by whatever is under the boat: the real −6 dB width varies 7× across altitude bands of one recording. Held at the values NC #4 gates. |
| `beam_tilt_deg`, `vertical_aperture_deg` | 20.0 / 50.0 | Spec sheet; widening them did not improve the fit. |

**How far the fit gets, and what stops it.** Pooled residual against the
40 m recording, over the far field past each band's own bottom return:
**14.0 dB RMS before the fit, 4.8–5.6 dB after**. The floor is not the
model — two *real* passes of the same water, at the same range setting and
altitude band, disagree by **1.8–3.1 dB RMS**. A flat-seabed model compared
against a harbour survey cannot beat that, because the far swath at 38 m
sits over ground whose depth is nothing like the depth under the boat.
The reduction's own repeatability floor is **0.67 dB RMS at 80 pings**
(smoke `[2e]`), well below both.

**What the corpus can and cannot fit** (smoke `[2e]`, properties of the
parameterisation rather than of the tuning):

* The level rides in `max_pwr_db`, **never in the counts**. The array spans
  0–65535 whatever the level is, so a count histogram carries no level
  information at all and `calibration_db_offset` is the sole level
  constant. (This replaces an earlier reading of the pair
  `base_scale` / `calibration_db_offset`, which assumed absolute counts;
  the device does not use them, and both keys are retired.)
* `tvg_compensation` and `lambert_exponent` both enter the far-field curve
  as a pure `log10(r)` slope, so on a flat seabed at one altitude they trade
  off almost exactly: at the fitted law, `tvg_compensation` 0.20 and
  `lambert_exponent` 0.10 differ by **0.28 dB peak-to-peak** across
  6.0–14.5 m, under the reduction's own floor. Separating them needs a
  capture spanning **≥ 2 altitudes** — the corpus has them (4–20 m, and
  6.7–11.4 m inside one recording), but site geometry swamps the lever.
* The per-range mean-dB curve — the statistic the fit compares — has a
  repeatability floor of **0.67 dB RMS at 80 pings** over 6.0–14.5 m.

`analysis/calibration.py` carries the reduction and
`sss_calibration_report` runs it against any recording and bundle; it
refuses to score a pairing whose altitude band or range setting does not
overlap, because that residual would measure the geometry gap.
`.claude/specs/sim-to-real-calibration-WITH_SVLOG.md` is the runbook the
fit followed, kept as the record — its Step 2 count-span plan is superseded
by the first bullet above.

**Timing.** Ping period = `max(msec_per_ping, 2R/c + 3 ms,
1/max_ping_rate_hz)` — the device law, **measured on all seven corpus
recordings** (2026-09-02): 50 ms at 18.4 m and 20 m, 54 ms at 38 m, 56 at
40, 73 at 52, 99 at 72, 110 at 80, 131 at 95.6, 170 at 125.6 m, i.e. the
spec sheet's "20 pings/s up to 30 m, thereafter limited by range and speed
of sound" with ~3 ms of processing. (An earlier comment claimed a 22 ms /
45 Hz free-run at 15 m; the corpus contradicts it and it is gone.) A 30 m
coverage pass therefore pings at 20 Hz and a 95.6 m survey at 7.6 Hz —
4000 pings in 3½ min against 2800 in 6½ min is the range setting, not a
fault. Two simulator tells remain by design: port and starboard share one
tick (NC #3) and there is no jitter. The ping counter is per side and
counts from node start, as the device counts from power-up; a ping-enable
cycle reconfigures the encoder and never restarts it. Along-track pixel
spacing = v/PRF.

## 7. Assumptions (explicit)

* **A1 — straight rays.** No refraction; sound speed constant
  (`sos_dmps` 1500 m/s). Reasonable over ≤ 30 m in shallow well-mixed water.
* **A2 — first-order reflections only.** Two multipath models
  (`wall_multipath_enabled` on by default — a strict no-op without walls;
  the second-bottom-echo `multipath_enabled` off by default), and no path
  beyond them: no second bounce, no
  composed wall-then-surface image, no reverberant tail.
  `multipath_enabled` re-images the direct response displaced +altitude in
  slant range and scaled by `multipath_gain` — the dim ghost-seabed line of
  shallow bottom–surface–bottom paths, with boundary losses and extra
  spreading lumped into the single gain. `wall_multipath_enabled` is the
  geometric model: see §10.
* **A3 — no volume scattering / water absorption inhomogeneity.**
* **A4 — static scene.** No mid-run object motion, no vegetation sway.
* **A5 — stop-and-hop pings.** No intra-ping motion blur; inter-ping
  motion (attitude, advance) is fully modeled. The finite 0.5° azimuth
  beam **is** modeled (multi-line integration, §1).
* **A6 — 2.5-D scene.** Heightfield world: no overhangs, objects
  represented as height + reflectivity stamps. Since 2026-09-03 the stamp
  of a *rested* object is the z-buffer of a rotated 3-D primitive settled
  on the local seabed plane (`worldgen/solids.py`: a drum lying on its
  side, a crate on any face, a cone on its flank), so the top surface is
  true 3-D geometry; what the raster still cannot carry is the underside
  of an overhang. Adequate for litter-scale targets; wrecks with cavities
  would need the mesh path (roadmap §3).
* **A7 — speckle correlation is geometric only.** Inter-ping and
  inter-bin speckle correlation comes from footprint overlap and the pulse
  (§5); there is no seabed micro-structure beyond the raster texture, so
  the long correlation tail the corpus shows at lag 10 on the 40 m files
  (0.2–0.5, likely real bedforms) is not reproduced.
* **A8 — no automatic gain control.** Gain is whatever `gain_index`
  selects, held fixed for the run. The real device's auto-gain
  (`gain_index: -1`) is accepted and resolves to the calibrated index 4,
  so an auto-gain run reproduces the calibration anchor rather than a
  device-chosen, scene-dependent gain. A run whose real counterpart used
  auto-gain therefore has *less* radiometric variation than the real one;
  radiometric degradations are deferred to the augmentation stage
  (`gain_drift_amp`, off by default). The real device's AGC is not
  incidental — it walks indices 4–7 inside every field recording, and it is
  why `calibration_db_offset` is determined only to ±6 dB (§6): the AGC
  holds the real reported level roughly flat against altitude while the
  model's follows the range law.
* **A10 — the bottom type behind the fitted constants is unsampled.** The
  corpus §6 fits against is a harbour survey whose sediment was never
  sampled, so `lambert_exponent` — the one constant that speaks directly
  about the seabed — is confounded with it and is held at Lambert's law
  proper rather than fitted. A capture over a known bottom would separate
  them; nothing in this corpus can.
* **A9 — closed (2026-09-02, re-anchored 2026-09-03).** Walls reflect,
  echo *and* occlude, quays and pontoons differently: §11. What remains
  assumed is the face model itself — a uniform vertical plane with one
  reflectivity and one rms roughness, no rubble slope at the foot, no
  diffraction over the top, and a lumped `wall_diffuse_fraction` for the
  incoherently scattered energy. The world-builder's *limit* borders are
  still not walls at all: collision-only, invisible, never acoustic.
* **A11 — near range is the ringing regime.** No wall face returns from
  nearer than the seabed under the boat: the range law of a face sample is
  floored at the altitude. A quay 0.3 m off the hull would otherwise blow
  up as `1/R⁴`; in the real device that region is the transmit ringing
  (§3), which the model already carries.

## 8. Fidelity positioning

The model is a KTH-style (Bore & Folkesson) heightfield-draping renderer:
the accepted standard for generating *training-grade* SSS imagery when the
goal is detector development rather than acoustic research. Everything a
detector keys on — highlight/shadow geometry, nadir gap, speckle
statistics, texture by material, range falloff, attitude banding — is
present; everything it should not key on (renderer artifacts) is avoided
by the per-bin averaging and noise stack. The `SonarRenderer` ABC is the
seam for any higher-fidelity replacement.

## 9. First-bottom-return structure (shallow-water regime)

At the 1–3 m altitudes of the shallow regime the FBR sits within the
first 40–120 of 600 bins (25 mm/bin). The rendered ping guarantees the
structure bottom-tracking bootstraps assume: a quiet water column
(noise-only, ~`noise_floor` + `watercolumn_noise` counts), a sharp bright
ramp at slant range = altitude (specular lobe × sidelobe floor), then the
Lambert-shaded seabed decay. Verified 13–16 dB gap→peak contrast across
1–4 m altitude; a simple "first bin > 3× water-column median" detector
locks on the exact FBR bin. If an FBR bootstrap tuned for deeper AUV
altitudes still fails to lock, tune its minimum-altitude window and ramp
length to this bin range rather than the renderer.

**Passing over a proud object.** `altitude` is read from the
object-inclusive raster, so over a drum the nadir gap collapses to the
drum top, the flat top images as a coherent slab at the gap edge, and the
horizon test seeded from the plateau edge shadows the seabed out to
`L ≈ h·R/alt` — geometrically right, and large only when the object is
large relative to the altitude (a 1.3 m-wide, 1.8 m-tall drum at size
scale 2 under a 4 m altitude). Structures a boat actually drives over are
pontoons, which §11 now models as floating; a real 0.9 m drum passed at
4 m altitude shortens the gap by 0.9 m and casts a few-bin shadow.

## 10. Wall / surface multipath (`sonar/multipath.py`)

On by default since 2026-09-01 (`model.wall_multipath_enabled: true`; the
surface mirror stays off). Enabling it is a strict no-op in any world
without walls — a scene with no reflectors renders bit-identically — so no
open-water result moves, and a walled world gets its physical reflections
without a special profile. The world config declares
the reflectors under `walls:` — finite vertical segments from the seabed to
`top_z`, each with its own `reflectivity` — and they travel in
`scene_manifest.yaml`, so the renderer, the labeler and the Gazebo world all
read one list.

**Image sources.** Reflecting the transducer in a plane turns the folded
path into a straight line from the mirror, so a ghost renders through the
same geometry and shading passes as the direct path and lands in the bin its
true path length earns — not at a fixed offset. The mirror set is first
order: one image per wall, plus the `z = 0` image when
`surface_mirror_enabled`.

Four details decide whether this is a model or a picture:

| Detail | Why |
|---|---|
| The look direction is **reflected**, not re-derived from a mirrored yaw | Reflection in a vertical plane flips handedness; `yaw + side.sign·90°` would pick the wrong side. The roll term, which tilts the fan toward the look direction, flips with it. |
| The surface image is evaluated at **−depression** | That path leaves the transducer *upward*. The main lobe sits 20° below horizontal, so it passes only through the sidelobe floor — which is why a near-perfect reflector does not swamp the calibrated direct field. |
| Walls are **finite** | A sample counts only if the segment from the mirror crosses that wall's plan extent, below its top. Without it, ghosts appear on the wrong side of the basin. |
| Sources are **culled** before rendering | A wall further than the receive window cannot put anything in it (the shortest folded path is the perpendicular distance to the plane), and a fan pointing away from a wall never bounces off it — on a wall-parallel leg that is the outboard side, half the sources. |

**Where the energy goes.** Ghost power joins the *diffuse* channel and never
the coherent specular one. So the first-bottom-return channel downstream
bottom tracking locks onto is untouched by this feature by construction
rather than by tuning (NC #4), and a ghost carries fully-developed `Exp(1)`
speckle rather than the low-CV coherent statistic — right, since a ghost of
the nadir return has bounced off a rough boundary and decorrelated. The
direct field's statistics are unchanged either way (NC #5). The
second-bottom-echo ghost is applied to the direct response *before* wall
ghosts are summed in, so the two models stay independent.

**Ground truth.** Every ghost carries its own contact naming the object it
images and the reflector that made it (`ghost: true`, `via:
"wall:<name>"`), so ghosts are labelled data rather than unlabelled clutter:
hard negatives for a detector, and a false-positive stimulus the metrics
count separately instead of crediting to the object.

**Cost**, measured at 600 bins, 5 azimuth lines for the direct path and
`ghost_beam_lines: 1` for ghosts: ×1.00 with every wall culled, ×1.25 with
one wall in range, ×1.50 with three. `ghost_beam_lines: 5` costs ×2.17 and
buys nothing a bounce has not already smeared — it is the lever if a basin
ever needs it.

## 11. Walls: face echo, occlusion, ghosts — quays and pontoons

Every real harbour mosaic shows the quay as a **bright band with a dark
zone behind it**, faint mirrored texture further out. Measured on the
anchor recording `fullHarbourCleanExample.svlog` with
`analysis/wall_contrast.py` (a persistent late peak over ≥ 10 pings, on
the device's own reported dB): face **+22.8 / +18.9 dB** over the seabed
1.5–4 m in front of it (port / starboard), the strip 1.5–4 m beyond
**−5.6 / −17.4 dB**, 6–12 m beyond **−20.8 / −24.8 dB**, bright band
0.7–0.9 m wide, faces at 30–35 m. Until 2026-09-03 the simulator's wall
*occluded* correctly but its own mirror ghost, at gain `0.65` on power,
repainted the occluded band with a −1.9 dB copy of the seabed — "walls do
not block" — and the face was a thin line dimmer than the seabed. The
model now has five parts, all on `core.types.Wall` segments, none in the
raster:

* **Kinds.** `kind: quay` — a face from the **seabed** to `top_z` (a
  `top_z` below the waterline is a submerged sill, looked over by rays
  above it); `kind: pontoon` — a **floating** face spanning only
  `[-draft_m, 0]`: sound passes underneath, so it occludes nothing below
  its draft, casts no quay-like shadow and reflects only rays meeting its
  band. A wall line the boat drives over is a pontoon, never a quay: the
  former rendered a crossed pontoon as a quay under the hull — a
  `1/R⁴`-boosted face slab in the water column plus a full shadow, the
  artefact reported on 2026-09-03. Every test below uses the face band
  `[z_bottom, z_top]` (`Wall.z_bottom`).
* **Hit and occlusion.** The horizontal ground ray of every azimuth line
  is intersected with every wall (`_wall_hits`); the nearest wet face is
  the one this line sees. A ground sample beyond it is culled when the
  ray's height at the crossing lies **inside the face band** — a quay
  stops everything, a pontoon only the shallow rays.
* **Face return** (`_face_echo_into`, coherent channel, own per-bin mean).
  The wet face is sampled from `z_bottom` to `z_top`; each point returns
  `reflectivity · wall_echo_strength · beam(δ) · range_law(max(R, altitude))
  · (spec · max(cos²ψ, floor) · exp(−δ²/2σ²) + wall_face_diffuse ·
  cos(θ_face)^lambert)` with `δ` the depression, `ψ` the azimuth
  obliquity and `θ_face` the angle from the face normal
  (`cos θ_face = cos δ · |cos ψ|`): a rough concrete face returns diffusely
  over its whole height on top of the specular lobe near the transducer's
  own depth; a pontoon's rounded fendered floats keep only
  `wall_pontoon_specular` (0.15) of the lobe. The range law is floored at
  the altitude (A11). The face therefore spans slant `d … √(d² + H²)` —
  0.8 m at the corpus geometry, matching the measured band.
* **Ghost energy** (`multipath.mirror_sources`, `bounce_weight`). The
  image-source path meets the face **twice** (out and back), so a ghost
  carries `reflectivity²`; on top, each bounce keeps only the coherent
  fraction `exp(−Γ²)` of the Rayleigh roughness law, `Γ = 2 k σ_h cos θ_face`
  (Eckart 1953, Urick 1983 ch. 5; `k ≈ 1900 rad/m` at 450 kHz, per-wall
  `roughness_m`, 3 mm concrete), plus the lumped share
  `wall_diffuse_fraction` (0.04) of the incoherently scattered energy that
  still reaches the mirror geometry. Near grazing (the mirrored near-wall
  seabed, just beyond the face) the mirror survives; the far mirror fades.
* **Single-bounce band.** The transducer insonifies a ground point in
  front of the wall directly and the echo returns off the face (or the
  reverse), arriving at the **mean** of the direct and folded path lengths:
  for every point in front of a quay that mean lies just beyond the foot's
  range, so the whole strip folds into the band right behind the face —
  the bright-then-dark structure of the field mosaic. Accumulated per
  mirror source as a sum of scatterers (`_Acc.num_b`, gain `reflectivity ·
  wall_corner_gain`), pulse-smeared, on the diffuse channel.
* **Ground truth** unchanged: one `object_type: "wall"` contact per
  ping-side (`wall:<name>`, face distance, face span, occluded remainder).

**Anchoring.** `wall_echo_strength` 30, `wall_corner_gain` 1.0 and
`wall_diffuse_fraction` 0.04 were set on a flat 7.2 m basin with the quay
at 30 m and 95.6 m range (the anchor file's geometry): the model reads
face **+21.8 dB**, beyond **−15.5 dB**, far **−19 dB** (TVG-flattened,
against the seabed at equal slant), within ~4 dB of both corpus sides;
`wall_face_diffuse` 0.5 and `wall_pontoon_specular` 0.15 are judgement
values (no pontoon geometry ground truth exists in the corpus). Smoke
`[4d]` renders the **shipped combination** (echo + occlusion + ghosts
together — the case the old suite never rendered), a pontoon (seabed
beyond unchanged, faint edge only, no slab when crossed), the `R²` law and
the corpus reduction (`[2f]`, gated on the corpus). Run
`python3 -m blueboat_sss_sim.analysis.wall_contrast <svlog>` on any
recording, simulated or real, to reduce it to the same three numbers.
