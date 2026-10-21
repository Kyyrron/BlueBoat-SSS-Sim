# Validation and experiment plan

What to measure, in what order, and how to score it, so that each
calibrated constant is identifiable and each claim is tested on data that
did not tune it. Numbers marked *(measured)* come from `docs/sonar_model.md`
§6. No acceptance tolerance below is invented: each is derived from a
measured repeatability, or left "set from the baseline".

## 1. Split policy

- **Unit of split: the whole session** (site × day × regime), never pings
  and never passes of one outing. Adjacent pings share the seabed, the
  motion and the device state. The manifest enforces it: roles per
  session, one role per `group`, no shared paths or hashes.
- Keep **≥ 1 validation session per regime** that a result will be
  claimed for (altitude band, bottom class, gain mode, sea state), and a
  **test** set opened once, at the end.
- Stratify every metric by session, side, range band, altitude band, gain
  state, bottom class, target and aspect, and motion regime when labelled.
  Report per stratum, never only a pooled number.
- The two current calibration anchors (`bath3`, 1.40 m sand;
  `reflectionEvidenceFullharbour`, 7.33 m) are **calibration** data:
  results on them are fit quality, not validation.

## 2. Calibration order, with the experiment that identifies each stage

| Stage | Parameters (registry) | Experiment | Why it disambiguates |
|---|---|---|---|
| 1. Geometry, sync | `model.sensor_depth_m`, `mount_x_m`, `mount_y_abs_m`, `beam_tilt_deg`; (future) sonar–nav latency | Dock: tape the transducer face below the waterline at survey load; inclinometer on the bracket. Water: reciprocal passes at constant speed over a **surveyed point target** (RTK or a taped baseline) | Reciprocal headings separate an along-track lever arm (sign flips with heading) from a latency (sign flips with direction of travel, scales with speed); the depth enters the FBR altitude, independent of level |
| 2. USV dynamics | `sea.hull.*`, Control's hydrodynamics (by reference) | Calm water: step/zig-zag thrust commands with poslog + run bag; free-decay (push the hull, release) for heave/roll/pitch | Command → motion with no acoustic model involved; yaw-rate response (reported 4–10× low in sim) is identified here, not by bending the sonar |
| 3. Device | ringing, gain table 0–7, floors, `filter_duration_percent`, `max_span_db` | Deep water (bottom beyond range) at each **forced** gain index 0–7, one minute each; a pass at 600 and 1200 bins | Receiver floor per gain gives the gain steps directly; ringing decay without a bottom; bin count changes the PSF but not the level |
| 4. Seabed mean | `calibration_db_offset`, `tvg_compensation`, `lambert_exponent`, `beam_tilt_deg` (check), `specular_*` | Over one **independently classified** flat bottom (grab sample or diver), fixed gain, **≥ 3 altitudes spanning a factor ≥ 5**, reciprocal headings; repeat per bottom class | `sss_calibration identifiability` shows that one altitude cannot separate the TVG share from the angular law (correlation 0.987–0.999, condition number 3.5e4 with the offset), while 1.40 + 7.33 m can (condition number 29.5, max \|corr\| 0.92). A third altitude is margin |
| 5. Objects, labels | `sand_lambert_mu_db`, material roughness/transmission, (`smooth_lobe_width_deg`) | Reference targets of known geometry and material (a sphere or a plate of known size; the NAFCO box at **surveyed yaw**, unburied) at 3, 6 and 12 m, several aspects | Known cross-section ties µ_sand; known yaw removes the aspect/burial ambiguity that left `smooth_lobe_width_deg` undetermined; 3 vs 12 m also measures the near-field footprint (`array_length_m`) |
| 6. Texture, noise | speckle per sediment, floors, along-track correlation | Long straight lines per bottom class, constant speed | Intensity distribution and tails per class (K vs Exp), lag-1/lag-10 correlations |
| 7. Environment, augmentation | wave/current response, gain drift, drops | Logged sea states; operational logs | Only after 1–6, so augmentation does not mask model error |
| 8. Blind | — | Untouched sessions per regime | The only claim of accuracy |

## 3. Metrics

### Acoustic (per stratum; tools exist in `analysis/` unless marked new)

- **Seabed level:** dB-vs-slant past 1.4 × altitude, as bias, RMS and
  spread (`analysis.calibration.reduce_profiles`, `compare`). Report it
  against the **reduction floor 0.67 dB RMS** *(measured)* and the
  **pass-to-pass floor 1.8–3.1 dB RMS** *(measured)*. A model cannot be
  held tighter than the second.
- **First bottom return:** location error and peak level; the altitude
  law of the peak (device `66.60 − 11.28·log₁₀ alt`; sim residual +4–7 dB
  *(measured)*); tracker lock rate (smoke `[2b]` logic).
- **Pulse and PSF:**
  - effective range resolution and the FBR onset ramp;
  - edge width 10–90 % across-track (field box 1.10–1.84 m vs sim
    0.08–0.12 m *(measured)*, an open residual);
  - target contrast and footprint (`analysis.object_signature`).
- **Shadows:** length, position and depth vs the lit seabed (the 17–25 dB
  quay band *(measured)*; the 9.6 dB box shell); the label-to-image
  agreement (smoke `[8]`, `[15]`).
- **Walls and multipath:** face contrast, the beyond-face bands, ghost
  location and level (`analysis.wall_contrast`).
- **Texture:**
  - intensity quantiles, CV and tail index;
  - lag-1 correlation along range and along track (`svlog_compare`
    `texture_stats`), at the same `num_results` and pulse;
  - floors: receiver (far decile) and water column (1–3 m band);
  - ringing decay over the first metre.
- **Device behaviour:** gain steps, the normalisation invariants
  (`sss_calibration quality`), ping period and jitter vs
  `max(50 ms, 2R/c + 3 ms)`, the counter offset, and the channel-255 rate.

### USV and control

- position, heading and velocity error against the commanded path;
- cross-track error and crab angle under current;
- yaw-rate distribution and spectrum, and turn response vs command;
- roll, pitch and heave std and spectra (field roll 0.9–1.3°, pitch
  0.8–1.5° *(measured)*);
- pose age (the sim's `pose_age_ms`), and the alignment of commands,
  navigation and pings (stage 1 latency).

### Dataset utility (a separate test)

Train on synthetic data (or synthetic + augmented), test on **held-out
real** sessions. Report detector precision, recall and AP per class, range
and aspect, and run an ablation that switches simulated effects off one at
a time. Passing image-fidelity metrics does not imply utility, and the
reverse is not true either.

## 4. Acceptance gates

- **Calibration fit:** each fitted stratum's residual is within its
  measured repeatability, i.e. the pass-to-pass floor for that regime,
  re-measured on the new data.
- **Validation:** no worse on held-out sessions than the calibration
  residual plus the pass-to-pass floor. A larger gap is model discrepancy
  and is reported as such (Kennedy & O'Hagan 2001), never absorbed by
  re-fitting on the validation data.
- **Identifiability:** no reported individual value for a pair with
  |correlation| ≥ 0.95 under the actual data geometry; report the pair's
  equivalence range instead.
- **Regression:** the smoke suite stays green. Rebaseline the render
  fixture only for a deliberate model change, never to absorb a fit.
- **Thresholds** for the PSF, shadow and texture metrics are set from the
  first repeat-pass baseline of each regime, before any fit.

## 5. Session checklist (to record in the manifest)

- **Setup:** `range_length_mm`, `num_results`, pulse and gain mode;
  forced-gain segments marked.
- **Transducers:** depth below waterline at rest, measured both sides, and
  the bracket tilt.
- **Site:** bottom class with its evidence (grab sample or photo); water
  depth, tide and CTD if available; sea state, wind and current notes.
- **Targets:** surveyed position, yaw, burial and material.
- **Recordings:** the `.svlog` (sonar), the run bag and poslog (Control),
  and the GCS session; their SHA-256 values computed on the day.
- **Pass log:** heading, speed and the start and end times of each pass.
