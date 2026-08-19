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
| Specular lobe | `S · (0.5+0.5ρ) · exp(−θₙ²/2σ²)`, θₙ = angle from the surface normal — dominates at/near nadir and produces the bright **first bottom return** that downstream FBR / bottom tracking locks onto. S (30) is sized so the FBR clears a +8 dB noise-floor threshold even over dark mud/seagrass. Rendered as a **separate component** from the diffuse field because its fluctuation statistics differ (§5) | `specular_strength` (S), `specular_width_deg` (σ), `specular_looks` |
| Vertical beam pattern | Gaussian in depression angle, centered on `beam_tilt_deg` + roll toward that side, FWHM = `vertical_aperture_deg` (50°, the Omniscan 450 spec beam height), floored at `beam_sidelobe_floor` (real sidelobes; without the floor, near-nadir rays ~70° off-axis at shallow altitudes would be attenuated below the water-column noise and the FBR would vanish) | mounting section |
| Residual range response | `(10^(−α·2R/10) / R^(2k))^(1−c)` — two-way absorption `α` and spreading `k`, partially undone by TVG fraction `c` | `absorption_db_per_m`, `spreading_exponent`, `tvg_compensation` |

Samples are accumulated into the `num_results` slant-range bins with
`np.bincount` and normalized by per-bin hit counts, i.e. an unbiased mean
per bin — the discrete analogue of the intra-bin ensonified average.

`tvg_compensation = 1.0` reproduces a perfectly flattened image;
the default `0.90` leaves the gentle range falloff visible in the real
captures.

## 4. Surface-vehicle attitude coupling

Roll rotates the vertical beam pattern; a rolling USV therefore brightens
one side and dims the other ping-by-ping, producing the along-track
banding characteristic of surface platforms (a core concern of the thesis
regime). Pitch and heave enter through the odometry pose. There is no
intra-ping motion (§7, A5).

## 5. Noise model (`sonar/noise.py`)

Applied in order, all configurable:

1. **Speckle, per component** — the diffuse field gets multiplicative
   `Exp(1)` intensity speckle: fully developed (Rayleigh amplitude), the
   textbook single-look statistic; `speckle_looks: L > 1` gives multi-look
   `Gamma(L, 1/L)`. The **coherent specular** component gets
   `Gamma(specular_looks, 1/looks)` (CV ≈ 0.2 at the default 25): the real
   near-nadir echo is Rician with a high K-factor and fluctuates far less
   than diffuse speckle — this is what makes the first bottom return a
   stable ping-to-ping feature that bottom tracking can lock onto.
   Verified: flat-seabed CV ≈ 1; a persistence-threshold FBR detector run
   on consecutive moving pings finds 100% of 10-ping windows within a
   0.30 m band.
1b. **Pulse smearing** — each ping is convolved with the transmit-pulse
   range envelope (width `c·τ/2`, ~3 bins at the 20 Hz default): every
   scatterer is a multi-bin feature and neighbouring bins are correlated,
   as at the real matched-filter output (`pulse_smearing: true`).
2. **Water-column noise** — small additive floor inside the nadir gap
   (`watercolumn_noise`). Must stay well below the first-bottom-return
   peak (default 0.002 gives 13–16 dB gap→FBR contrast at 1–4 m
   altitude) or downstream bottom tracking cannot lock.
3. **Receiver noise floor** — additive everywhere (`noise_floor`).
4. **Gain drift** — slow multiplicative sinusoid + random walk
   (`gain_drift_amp`, `gain_drift_period_s`). **Default 0**: radiometric
   drift/banding is deferred to the downstream augmentation stage; set
   > 0 to re-enable in the base model.
5. **Dropped pings** — Bernoulli per ping (`dropped_ping_prob`,
   **default 0**, same deferral); when enabled the ping counter still
   advances, as with the real device.

## 6. Quantization and encoding (`sonar/encoder.py`)

Float power → u16 `pwr_results` via
`base_scale · 10^((gain_index − 4)·gain_index_step_db/10)`, clipped to
65535, so the ROS `gain_index` parameter behaves like the real gain
ladder (`analog_gain` reported from the measured table, 74.55 at index 4).
`max/min_pwr_db = 10·log10(pwr) + calibration_db_offset` reproduces the
observed ~16 dB offset. Frames are byte-exact Ping Protocol
(`BR | len | 2198 | src | dst | 52-byte fixed payload | u16[n] | checksum`);
`parse_frame()` round-trips them and the smoke test verifies length parity
with the field capture (1262 B at n = 600).

**Power calibration.** `base_scale` (110 000) is calibrated against the
real capture: a flat-seabed FBR averages 15–25 k counts and speckle
maxima approach/occasionally clip 65 535, reproducing the captured
`pwr_results` span (0–62 k) and `max_pwr_db` (verified: sim mean
63.9 dB vs 63.9 dB decoded from the field frame).

**Timing.** Free-run period = two-way travel + 2 ms device processing;
`msec_per_ping > 0` clamps to the commanded period like the hardware.
`model.max_ping_rate_hz` (default 20, the Omniscan 450 spec-sheet cap)
additionally floors the period at 1/rate → 50 ms at 15 m. **Documented
tension:** the team's own field capture shows 22 ms (45 Hz) free-run per
channel at 15 m via the device `timestamp_ms` deltas; set
`max_ping_rate_hz: 0` to disable the cap and reproduce the captured
behaviour. Along-track pixel spacing = v/PRF either way.

## 7. Assumptions (explicit)

* **A1 — straight rays.** No refraction; sound speed constant
  (`sos_dmps` 1500 m/s). Reasonable over ≤ 30 m in shallow well-mixed water.
* **A2 — single-bounce direct path.** An optional first-order
  second-bottom-echo ghost is available (`multipath_enabled`, off by
  default): the direct response is re-imaged displaced +altitude in slant
  range and scaled by `multipath_gain`, reproducing the dim ghost-seabed
  line of shallow bottom–surface–bottom paths. Boundary reflection losses
  and extra spreading are lumped into the single gain. *Wall* multipath
  (quays, pontoons) remains roadmap §2.
* **A3 — no volume scattering / water absorption inhomogeneity.**
* **A4 — static scene.** No mid-run object motion, no vegetation sway.
* **A5 — stop-and-hop pings.** No intra-ping motion blur; inter-ping
  motion (attitude, advance) is fully modeled. The finite 0.5° azimuth
  beam **is** modeled (multi-line integration, §1).
* **A6 — 2.5-D scene.** Heightfield world: no overhangs, objects
  represented as height + reflectivity stamps. Adequate for litter-scale
  targets; wrecks with cavities would need the mesh path (roadmap §3).
* **A7 — uncorrelated speckle.** No inter-ping speckle correlation.

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
