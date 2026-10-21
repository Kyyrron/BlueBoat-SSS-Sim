# Sonar model

How the synthetic Omniscan 450 imagery is produced, what is modeled, what
is deliberately not, and which parameter controls what. Implementation:
`blueboat_sss_sim/sonar/`.

## 1. Geometry

For each ping and each side, the transducer pose is the boat's pose **at
the ping time** (Gazebo's per-step model pose on `/sim/sea_state/boat_pose`
since 2026-10-09, `/blueboat/odom` only as the fallback — odometry is not required while the 100 Hz pose flows; the last sample
propagated by its own rates, `sss_sim_node` `predict_pose`, since
2026-10-08 — §4): the
body-frame lever arm (`mount_x_m`, ±`mount_y_abs_m`, −`sensor_depth_m`)
rotated by the full attitude R(roll, pitch, yaw) (yaw only before
2026-10-08; a level pose is bit-identical either way). The renderer casts
athwartship ground lines perpendicular to the heading (side sign:
port = +y in body frame), sampling the scene heightfield at a step
automatically coupled to the slant-bin size (`max(min(sample_step_m,
f·bin), 4 mm)`, f = 0.30 with the coherent scatterer field — the default —
and 0.45 without; `renderer.py`), so every range bin is populated at any
`num_results` (at the default 20 m range: 600 → 33 mm bins, 1200 →
16.7 mm bins).

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
(`visible` flag, `shadow_bins`), which since 2026-09-18 are **read off that
array** rather than from the flat-bottom formula: an object whose shadow
ends inside its own footprint reports none, one standing in another's
shadow reports none and is not visible, and a shadow lengthened by a
down-slope or broken by a lattice is labelled as the picture shows it.

Three things make a shadow a *measurement* rather than a hole, all added on
2026-09-18 and all anchored on the corpus:

* **It is not empty.** A culled bin holds the reverberation floor —
  `reverberation_db` (18) below what the seabed at that range returns where
  it is lit, with the seabed's own `Exp(1)` speckle. The renderer carries
  that unoccluded field per bin (`Ping.unoccluded`) precisely so the floor
  can follow it. Measured behind the corpus quay: 17–25 dB.
* **Some objects are not opaque.** A material's `transmission` attenuates
  the shadow instead of zeroing it, taken from the crest that set the
  horizon, so a flooded polypropylene shell shadows the sand by 9.6 dB —
  which is what the field box did — while a steel drum still blacks it out.
* **A wall's own face is occluded too.** The face echo tests each face point
  against the nearer faces and against the terrain horizon, so a quay behind
  a pontoon is imaged (the ray passes under the floats) and a quay behind a
  quay is not; ghosts and the single-bounce corner band are culled against
  nearer walls on both real legs of the folded path (§11).

## 3. Intensity model

Per insonified sample, echo power is the product of:

| Term | Formula | Config |
|---|---|---|
| Backscatter | `ρ · cos(θᵢ)^p`, one global `p` — a **single-power (Lommel–Seeliger-like) law**, not Lambert's `cos²`. It was called "Lambert's law proper" here until 2026-09-18, which was simply wrong: `p` is fitted, and the two-altitude fit measures it at **1.0** over sand | material `reflectivity`, `lambert_exponent` (global; the per-material `lambert_exp` was dead code and is gone) |
| Specular lobe | `S · (0.5+0.5ρ) · exp(−θₙ²/2σ²)`, θₙ = angle from the surface normal — dominates at/near nadir and produces the bright **first bottom return** that downstream FBR / bottom tracking locks onto. **S is 12 since 2026-09-18** (was 30), which is what puts the reported peak's *altitude slope* on the device's; it still clears the +8 dB threshold NC #4 gates. Rendered as a **separate component** from the diffuse field because its fluctuation statistics differ (§5) **and because it spreads differently**: a coherent reflection is the field of an image source over the folded path, `specular_spreading_exponent` (2.0), against the diffuse field's two-way `r⁻⁴`. Sharing one law made the FBR fall 20–27 dB per decade of altitude where the device falls 11.3 | `specular_strength` (S), `specular_width_deg` (σ), `specular_looks`, `specular_spreading_exponent` |
| Vertical beam pattern | Gaussian in depression angle, centered on `beam_tilt_deg` + roll toward that side, FWHM = `vertical_aperture_deg` (50°, the Omniscan 450 spec beam height), floored at `beam_sidelobe_floor` (real sidelobes). At the shipped geometry the nadir ray is 70° off-axis and the Gaussian tail alone gives 0.00436 (−23.6 dB), so the floor does not bind for a downward ray at level attitude; it takes over when the fan is rolled up by more than 0.55°, when the tilt is reduced or the aperture narrowed, and for upward (surface-image) paths beyond 50.6°. Without it the FBR would sink below the water-column noise in those cases | mounting section |
| Residual range response | `(10^(−α·2R/10) / R^k)^(1−c)` — two-way absorption `α` and two-way spreading `k` (4.0 = `r⁻⁴`), partially undone by the device's TVG, fraction `c` (**0.30 since 2026-09-18**, measured on two altitudes; it was 0 on the reading that the reported stream is pre-TVG, which one altitude cannot tell apart from a steeper spreading law). The coherent channel uses the same TVG curve over its own loss | `absorption_db_per_m`, `spreading_exponent`, `tvg_compensation`, `specular_spreading_exponent` |
| Along-track footprint | Gaussian of width `max(r·θ, array_length_m)/2.355` — the beam, floored by the aperture. `array_length_m` is **0**: the corpus says the device focuses its near field (§6) | `horizontal_aperture_deg`, `array_length_m`, `alongtrack_beam_lines` |

| Surface-area element | `sqrt(1 + s²)` on the diffuse channel — a sample stands for the ground it actually covers, not for its horizontal projection. For `p = 1` this collapses with the incidence term to `ρ·(dz + y·s)/r`, which contains no slope-from-raster term at all, so an object's brightness stops being a function of `world.resolution` (measured: 7.4 dB of spread across 0.20 / 0.10 / 0.05 m before, 0.2 dB after) | `facet_area_jacobian` |
| Coherent fraction | `C = exp(−(2kσ_h·cos θᵢ)²)` (Eckart/Rayleigh — the law `sonar/multipath.bounce_weight` already applies to a wall). What stays coherent leaves in the **mirror** direction, so `(1 − C)` scales the **near-normal lobe only**: at wide angles the mirror direction is nowhere near the transducer and the Lambert tail never had a coherent share to lose. At 450 kHz `C = 0` for every seabed material (`2kσ` is 15 for mud, 38 for sand), so the seabed is bit-identical and the term speaks only about manufactured surfaces | `coherent_split`, `sonar_frequency_hz`, material `roughness_m`, `smooth_lobe_width_deg` |

### 3.1 How samples become a bin (2026-09-19)

Until 2026-09-19 a bin was the azimuth-weighted **mean** of the ground
samples that fell in it. That is wrong wherever the ground folds: at a
rising shore or an object's near wall `dr/dy → 0` and a hundred samples land
in one bin, and dividing by their count gave back exactly the brightness of
flat ground. The device integrates their energy, so the fold *brightens* —
which is why a real beach accumulates a band at the waterline where the
simulator drew a black wedge.

`image_formation: energy` replaces the mean with

```
P_b = [ Σ_i c_i · w_i · ds · J_i · f_{i,b} ] / [ bin_size_m · Σ_k w_k(r_b) ]
```

where `J_i = sqrt(1+s²)` is the surface-area element, `f_{i,b}` is the
fraction of sample *i*'s slant interval (midpoint to midpoint, taken as
min/max because `slant` is **not** monotone under layover) inside bin *b*,
and the denominator is one bin of flat ground at that range, analytic
rather than sample-counted. Four consequences, all deliberate:

1. **Flat seabed, far field is unchanged**: `Σ ds f → bin·r/y → bin` and
   `J → 1`, so the tail slope, `calibration_db_offset`, `tvg_compensation`
   and `lambert_exponent` do not move. Measured on both calibration anchors,
   the mean offset shifted 0.20 dB (`bath3`) and 0.38 dB
   (`reflectionEvidenceFullharbour`) — inside the 0.67 dB repeatability
   floor of §6.
2. **Near nadir** the residual `r/y` adds +1.55 dB at 1.4× altitude,
   +0.63 dB at 2× and +0.14 dB at 4×.
3. **Layover accumulates**, capped by `layover_gain_max_db` (20 dB): a
   vertical cliff folds an unbounded ground length into one bin, the
   device's dynamic range does not.
4. **Partial-bin shadow coverage** falls out for free, because `visible`
   became a weight.

**The coherent channel keeps the mean normalisation** (`_Acc.den_s`). An
image source's effective aperture is a Fresnel zone, not a resolution cell,
and area-weighting it put the nadir `r/y` growth on the first bottom return:
the reported peak went 8.6 dB off the corpus anchor of §6. It also uses the
**diffuse** spreading law on a surface the roughness raster calls smooth —
a 0.32 m box wall is one Fresnel zone at 8 m, not an extended interface, and
without that scoping the `r²` image-source bonus lands on every object edge
and no lobe width can undo it.

**The near-normal lobe is a rough-surface Kirchhoff peak** whose width is an
RMS *surface slope*, and the 8° it carries is sand's, fitted on the corpus.
A manufactured surface has no measured slope distribution, and lending it
sand's is what produced the glint: a stamped height step always contains one
facet exactly normal to the ray, whatever the grid spacing, and that facet
then collects the whole lobe. So on a smooth cell the lobe is suppressed
(`smooth_lobe_width_deg: 0`), and a width may be set per run once a field
capture measures one. The smooth flag is **dilated one sample along the
ray**, because `np.gradient` builds a facet from a sample's neighbours and a
facet straddling an object's edge otherwise kept the seabed's lobe.

Measured effect on the 2026-09-07 field box (`plastic_box`, 25° grazing,
through one harness): highlight **+21.6…+23.3 dB → +7.6…+9.2 dB** against
the device's measured **+2.5 dB**; the shadow's worst bin **−54 → −19 dB**
against **−13.6**. **Superseded 2026-09-28 for smooth solid objects** by the
facet layer (§12): the raster path above now serves the smooth *aggregates*
(the lattice classes: mesh crate, bicycle, trolley) and every rough object.

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

The rendered reflectivity is clipped to `[0.02, 1.0]` for a material —
and to `[0.02, REFLECTIVITY_MAX]` (4.0, +6 dB) when a baked gain map is in
force (2026-09-16, `worldgen/materials.py`): the `reflectivity_survey`
layer measures rock and gravel several dB above sand and needs the
headroom. The specular lobe's `(0.5 + 0.5ρ)` then reaches 2.5 at ρ = 4 — a
rock's nadir return *is* brighter — and a world without a gain map clips
at 1.0 exactly as before.

`tvg_compensation = 1.0` reproduces a perfectly flattened image. The
default is **0.30** (2026-09-18, was 0.0 — §6): the device compensates part
of its range loss, which only a fit on two altitudes at once could tell
apart from a steeper spreading law. The remaining 70 % of the two-way loss
is present in the data — at `spreading_exponent: 4.0` (`r⁻⁴`, ~40 dB per
decade) about 28 dB per decade of slant range. That gradient is the
device's, not a defect. Downstream viewers remove its
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
ground line in `_ping_geometry`; zero pitch is bit-identical. **The mount
lever arm follows the attitude** (2026-10-08): a 10° roll lifts one
transducer and dips the other by `mount_y·sin(roll)` ≈ 49.5 mm on the
±0.285 m mount (the earlier "3 mm" was the horizontal term only), and pitch
swings the transducer `sensor_depth·sin(pitch)` along-track — both change
the slant geometry of that ping. **Vertical datum (2026-10-21).** The
vehicle pose's z = 0 is the boat floating level at rest on the still
water, so `sensor_depth_m` is the transducer face's depth below the still
waterline — what every offline and calibration render (pose z = 0) has
always assumed. Gazebo's base_link floats at rest 0.111 m *below* that
(`sea.motion.GAZEBO_BASE_REST_Z_M`, the URDF buoyancy box's equilibrium),
and until 2026-10-21 the node fed that z straight in: in Gazebo the
transducers sat ~0.26 m deep. The node now moves every received pose onto
the waterline datum first (`base_link_rest_z_m`; `0.0` restores the old
behaviour); Gazebo recordings made before are not comparable bit for bit. **Yaw** turns the fan, so the ground a
ping images at range r moves `r·sin(Δψ)` along-track, in opposite
directions on the two sides (the differential mode; pitch is the common
mode — Lei et al. 2026, arXiv:2604.19901). Every ping is rendered at the
pose *at its own time* (Gazebo's per-step model pose, at most 10 ms old,
since 2026-10-09; the 20 Hz odometry only as a fallback; propagated with
its rates), and the pose actually used is published on
`~/ground_truth/sensor`. There is no intra-ping motion (§7, A5): over a
20 m ping's 27 ms round trip a 10°/s yaw rate turns the fan 0.27°, ~9 cm at
the far edge against a ~17 cm footprint — smearing within one ping, not
modelled. The attitude itself comes from that Gazebo pose (the odometry
only as the fallback), i.e. from Gazebo under the wave wrench; the measured consequences (back-scanned and
skipped seabed, uneven pings per target, banding) are in
`docs/sea_state_validation.md`.

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
1b. **Pulse smearing** — each ping is convolved with the range envelope
   the acquisition implies: `τ/Ts + filter_duration_percent·N + 1` bins —
   the transmit pulse (τ = max(44.3 µs, `pulse_len_percent`·2R/c), the
   device law, measured on every `pulse_duration_sec` in the corpus within
   1 µs), the receive filter (0.0015, the value the real driver pins) and
   one sample of decimation: 3 bins at 600 / 0.002, 5 at 1200 / 0.00125 and
   20 m. Every scatterer is a multi-bin feature and neighbouring bins are
   correlated, as at the real matched-filter output (`pulse_smearing:
   true`); the texture's along-range lag-1 correlation reads 0.31 / 0.50
   against the corpus's 0.29 / 0.46 at those two settings (2026-09-28;
   before, a fixed 3-bin floor gave 0.32 at 1200). A legacy profile that
   sets `range_filter_bins` keeps the old kernel.
2. **Water-column noise** — volume reverberation inside the nadir gap
   (`watercolumn_noise`, **4.68 × 10⁻⁶**). **Measured** (2026-09-02): the
   1–3 m band of the gain-4 corpus pings reads 25.3 dB reported (mean of
   dB; 27.8 dB as a linear mean, which the constant is). It is a fixed
   absolute level, so its margin under the FBR peak depends on altitude:
   about 28 dB on the corpus at 6–9 m, about 43 dB at the smoke fixture's
   3.5 m, growing as the altitude falls — enough for downstream bottom
   tracking to lock.
3. **Receiver noise floor** — additive everywhere (`noise_floor`,
   **3.39 × 10⁻⁹**). **Measured** separately: the far decile of the gain-4
   pings of the 72–126 m recordings, where nothing but noise is left, reads
   −6.1 dB reported (−3.6 dB linear mean).

   Both constants are `10^((level − calibration_db_offset)/10)`, so both
   moved by 16.1 dB when the offset was re-fitted on 2026-09-18 (97.2 →
   81.1): they were 1.15 × 10⁻⁷ and 8.3 × 10⁻¹¹ at the old offset and
   encode exactly the same reported levels.

3b. **Reverberation floor of a shadow** — `reverberation_db` (**18.0**,
   2026-09-18). Where the main lobe is blocked the receiver still hears the
   same seabed at the same range through the beam's sidelobes, and the
   water volume. The floor of a culled bin therefore follows the ping's own
   unoccluded level (`Ping.unoccluded`, carried by the renderer for this
   purpose) rather than the white receiver floor 40+ dB below it, and it
   speckles like the seabed (`Exp(1)`), not like the receiver. **Measured**
   on the corpus quay: the deep shadow 6–12 m behind the face sits 17–25 dB
   under the seabed at equal slant. Before this the simulator's shadows were
   **30 dB too deep** — the single biggest cue error the 2026-09-18 review
   found, because a detector trained on shadows learns the shadow's
   *statistics*, not its existence. `0` restores the old white floor.

   Both were previously one jointly fitted number (6.79 × 10⁻⁷, i.e.
   35.5 dB reported at the then offset of 97.2 dB) chosen to put the water column 26 dB below the bottom
   return on files that never reached the receiver floor. That value hid
   the range law beyond ~20 m: the simulated far range sat flat on a floor
   ~40 dB above the device's, the per-ping span was 32 dB against the
   corpus's 63–69 dB, and a 30 m image looked nothing like a
   field one. With the measured constants the simulated gain-4 profile at
   **matched altitude** (the corpus flies at 6–9 m) follows the real one
   within ~1 dB from 10 to 35 m on the 40 m and 72 m recordings.
4. **Gain drift** — slow multiplicative sinusoid + random walk
   (`gain_drift_amp`, `gain_drift_period_s`). **Default 0**: radiometric
   drift/banding is deferred to the (optional) downstream augmentation stage; set
   > 0 to re-enable in the base model.
5. **Dropped pings** — Bernoulli per ping (`dropped_ping_prob`,
   **default 0**, same deferral); when enabled the ping counter still
   advances, as with the real device. Each side draws independently, so a
   non-zero value produces the **one-sided rows** CM-6 is about.
6. **Fault injection** — `fault_gain_walk_pings`, `fault_counter_reset_pings`,
   `fault_counter_offset`, `fault_channel_255` (all **default off**,
   2026-09-18). Not fidelity: these are the downstream paths a clean
   simulation never takes. CM-2 promises the real stack runs unmodified
   against simulated data, and a simulator whose gain never walks, whose
   counter never restarts and which never reports channel 255 cannot
   demonstrate it — the processor's gain compensation sits at `g = 0`
   forever, its counter re-base and the channel-255 side fallback are never
   entered. Each knob reproduces something the corpus shows (all four gain
   indices inside one recording; counter offsets of 0, −1 and +60 across
   the 16 two-sided logs). `config/fault_injection_sonar.yaml` turns them on
   together; it is a **test** profile and no calibration or comparison run
   should use it.

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
sampled**; and, since 2026-09-07, the **Shirahama beach corpus**: 12
recordings over sand at **0.9–1.7 m altitude**, 20 m per side, gain 4,
including the three bathymetry runs and the object sessions. Both live under
`~/ros2_ws/data/SSS_data`, read-only (CM-7).

**The second corpus is the lever the first never had.** Every constant below
whose status changed on 2026-09-18 changed because the two altitudes together
separate what one altitude confounds: the harbour flies at 6–9 m (medians;
up to 12 m), the beach
at 1.2 m, and the range law, the level and the coherent component's own
spreading are three different functions of that. The 2026-09-18 review
measured the simulator **18 dB hot at 1.2 m** while it matched at 7 m — not a
level error but a *slope* error, which no offset can fix.

| Constant | Value | Status |
|---|---|---|
| `max_span_db` | 90.0 | **Measured**, directly and exactly. |
| `ANALOG_GAIN_TABLE[4..7]` | 74.55 / 142.8 / 242.025 / 464.625 | **Measured**, directly and exactly. |
| `spreading_exponent` | 4.0 | **Jointly determined** with `tvg_compensation`: only the product `spreading_exponent · (1 − tvg_compensation)` enters the curve. The split is fixed by taking spreading at its physical two-way-intensity value. |
| `tvg_compensation` | **0.30** | **Measured on two altitudes at once** (2026-09-18, was 0.0): `bath3` at 1.40 m (12 371 pings) and `reflectionEvidenceFullharbour` at 7.33 m (1 445 pings), fitted together over the seabed tail past 1.4 × altitude. The pair (`lambert_exponent`, `tvg_compensation`) = (1.0, 0.30) reproduces **both** absolute dB-vs-slant curves to **0.92 dB rms** (the fit's objective over the two curves together, as the 2026-09-18 review scored it — `sim_review.md` Part 7.1; the per-anchor mean error and shape rms below are different statistics); (1.0, 0.0) — the single-altitude answer — matched the harbour and ran 18 dB hot at the beach. The device *does* compensate part of its range loss. |
| `specular_spreading_exponent` | **2.0** | **Measured** (2026-09-18, new). The coherent component is the field of an *image source* and spreads once over the folded path, not twice like the diffuse field. With it the simulator's reported peak follows altitude as **72.1 − 11.5 · log₁₀ alt** dB against the device's **66.60 − 11.28 · log₁₀ alt** (19 gain-4 recordings, 0.92–12.19 m, rms 1.57 dB); sharing the diffuse `r⁻⁴` gave −20 to −27 dB per decade, i.e. a simulator right at one altitude and nowhere else. |
| `noise_floor`, `watercolumn_noise` | 3.39 × 10⁻⁹, 4.68 × 10⁻⁶ | **Measured separately** (2026-09-02) and **re-expressed at the new offset** (2026-09-18): both are `10^((level − calibration_db_offset)/10)` and the levels they encode — −3.6 dB for the receiver floor, +27.8 dB for the water column — are unchanged. |
| `reverberation_db` | **18.0** | **Measured** (2026-09-18, new): the deep shadow 6–12 m behind the corpus quay sits **17–25 dB** under the seabed at the same slant (`analysis/wall_contrast`), so a shadow's floor follows the ping's own level rather than the white receiver floor 40+ dB below it. The review measured the simulator's shadows **30 dB too deep**; they now sit at −18.9 dB against the lit seabed. |
| `calibration_db_offset` | **81.1** | **Re-fitted with the range law** (2026-09-18, was 97.2), on the same two altitudes and in the same fit: the seabed level then lands within **0.08 dB** at 1.40 m and **0.57 dB** at 7.33 m, where the old value was right at one and 18 dB out at the other. It remains the **only** level constant, and it moved once, with the law it belongs to. |
| `lambert_exponent` | 1.0 | **Measured** (2026-09-18, was "not determined"): over the two altitudes together the fit prefers 1.0 — 0.75 and 1.25 both score worse (rms 1.29 and 0.94 against 0.92) — because the two altitudes see the seabed at different grazing angles at the same slant range. Sand's angular law is therefore the single-power (Lommel–Seeliger-like) form the renderer applies, not the cos² of Lambert's law proper, and not the 1.6 the dead per-material field used to claim. |
| `array_length_m` | **0.0** | **Measured off** (2026-09-18, new). A 0.5° beam at 450 kHz implies a 0.38 m array, which — unfocused — would set the along-track footprint over the whole swath. The corpus's own along-track texture correlation (lag-1 **0.36–0.66** at 1 m/s, smoke `[2g]`) is what a ~0.1 m footprint at 12 m gives; 0.35 m would put it at 0.76. The device focuses its near field, and the `r·θ` law holds to at least 10 m. What happens inside ~3 m is **undetermined** — the knob is where a point-target capture at 3 and 12 m would put the answer. |
| `ringing_level_db`, `ringing_decay_db_per_m` | 55.0, 22.0 | **Measured** on `diffDepthCompensation.svlog`: ~55 dB at the transducer decaying to ~33 dB by 1 m (22 dB over the first metre). Cross-checked by `sss_calibration_report`'s nadir-profile section (real 54.4 dB at bin 0 vs simulated 51.8; the 1 m point is floor-limited on the sim side because `noise_floor` is fitted to the Shiraishi corpus, whose water column is louder than this log's). |
| `beam_tilt_deg` | 20.0 | **Corpus-constrained** (the BlueBoat bracket tilt is unpublished): `sss_calibration_report --fit-tilt` scans 5–40° against `diffDepthCompensation.svlog` and the residual minimum lands on 20.0° (5.88 dB RMS best of 11 scored tilts, 2026-09-01). |
| `absorption_db_per_m` | 0.10 | **Not separated** from the spreading term. Held at the physical 450 kHz seawater value. |
| `image_formation`, `facet_area_jacobian`, `layover_gain_max_db` | **energy**, **true**, 20.0 | **Derived, then verified** (2026-09-19, new). Not fitted: the equation of §3.1 is what energy conservation over a resolution cell gives, and the verification is that it leaves the calibration alone — the two-anchor mean offset moved **0.20 dB** (`bath3`) and **0.38 dB** (harbour), inside the 0.67 dB repeatability floor, so `calibration_db_offset` did not move with it. The cap is a judgement value: a vertical cliff folds an unbounded ground length into one bin and the device's dynamic range does not. |
| `coherent_split`, `sonar_frequency_hz`, material `roughness_m` | **true**, 450 000, per material | **Physical, with judged inputs** (2026-09-19, new). The law is Eckart/Rayleigh, the same one `multipath.bounce_weight` applies to a wall; the frequency is the nominal 450 kHz (the wire field `ping_hz`, 451 127 Hz observed, is reported only and never enters the physics; the model has one wavenumber, `acoustics.wavenumber`, k = 1885 rad/m, shared by this split, the object facets and the wall bounce weight). The per-material RMS heights (`_ROUGHNESS_M` in `worldgen/materials.py`) are **judged from manufacture, not measured** — 0.1 mm for an injection-moulded shell, 2 mm for form-finished concrete — and the table is stated there rather than buried because it is what decides whether a target reads as a glint or a faint patch. Every seabed material is far past `COHERENT_ROUGHNESS_LIMIT_M`, so the split is exactly a no-op for the seabed and the whole calibration is untouched. |
| `smooth_lobe_width_deg` | **0.0** (suppressed) | **Undetermined, and shipped off rather than guessed** (2026-09-19, new). The near-normal lobe's width is an RMS surface slope and the 8° it carries is *sand's*. A manufactured surface has none measured, so on a smooth cell the lobe is suppressed entirely. The field box's two grazing angles (+5.7 dB at 7°, +2.5 dB at 19°) would solve to ≈15° if they could be trusted to be one target's angular law, but that pair carries an unknown yaw and an unknown burial and a 30° yaw offset absorbs several dB — see §7 A10 and the capture item in `.claude/TODO.md` ([P2] "Seabed-dependent constants need a capture over a known bottom"). |
| `reverberation_reference` | **ambient** | **Structural** (2026-09-19, new). Referencing a shadow's floor to the culled bin's *own* would-be response fails wherever the local incidence clipped to zero — a back-facing flank, a cell over land — and the floor collapses with it. That collapse is the hard black band the sim-vs-real comparison found between a simulated object and its shadow, where the real image runs continuously; it put the worst bin at **−54 dB** against the device's **−13.6**. The ambient curve (a flat bottom at the ping's own altitude, scene median reflectivity) never collapses, and the floor is the **max** of the two, so the change can only lighten a shadow. Worst bin now **−19 dB**. |
| `volume_scattering_coeff`, `penumbra_deg`, `shore_taper_m` | **0.0** | **Implemented, shipped off, reasoned** (2026-09-19, new). Each has a defensible mechanism and no measurement that fixes its value here; see A3 for the volume term, and the capture item in `.claude/TODO.md` ([P2] "Seabed-dependent constants…"). The beach's remaining darkness past the waterline is a *world-authoring* gap — the model has no swash zone — not a model one, so a taper was not tuned to hide it. |
| `specular_strength`, `specular_width_deg`, `specular_looks` | **12.0** / 8.0 / 25 | **Strength constrained** (2026-09-18, was 30.0): with `specular_spreading_exponent` 2.0 it is what puts the reported-peak *slope* on the device's (−11.5 against −11.28 dB per decade). The level of that peak is still **+5 dB** hot — a documented residual, not a fitted value — while the seabed itself matches within 0.6 dB; the width and the look count remain **not determined** and stay at the values NC #4 gates. |
| `Material.transmission` (`pp_shell`) | 0.11 | **Measured** (2026-09-18, new): the beach box's own shadow reads 9.6 dB under the sand around it rather than at the receiver floor, and `10^(−9.6/10) = 0.11` is what a flooded 2 mm polypropylene shell passes through its walls and back. **Re-measured 2026-09-28** through the facet-layer renderer on the two clear passes (−8.2 / −7.4 dB): averaged over the unknown heading, within 0.25 dB. The walls are nearly transparent at 7–19° from their normal; the lid and floor, met 70–80° off theirs, are forward mirrors — which is what shadows a shell. Every other material transmits **0**, as every object did before. |
| `object_facets`, `facet_patch_m` | **true**, 0.01 | **Structural** (2026-09-28, new; §12). Smooth solid objects scatter from their exact geometry; the patch size is what keeps the phase linear across a patch and the beam constant over it (R ≳ 2 m). |
| `sand_lambert_mu_db` | **−12.0** | **Literature** (2026-09-28, new; §12): J&R Fig. 12.3 (−20.2 dB at 20–40 kHz) with §12.1.2's +6–8 dB/decade for sand. The one constant tying an object's cross-section to the calibrated seabed; ±5 dB site spread; never fitted to an object. |
| material `density_kgm3`, `sound_speed_mps`, `backing`, `wall_m` | per material | **Physical, cited** (2026-09-28, new; §12): Selfridge 1985, Kinsler et al. App. A10. They give each smooth object its reflection coefficient; nothing about an object's brightness is judged any more. |
| `vertical_aperture_deg` | 50.0 | Spec sheet; widening it did not improve the fit. (`beam_tilt_deg` has its own row above; it appeared twice until 2026-09-18.) |

**How far the fit gets, and what stops it (two-altitude fit, 2026-09-18).**
The seabed's absolute dB-vs-slant curve, sim against real, over the tail
past 1.4 × altitude: mean error **−0.08 dB** at the beach (1.40 m, 12 371
pings) and **+0.57 dB** in the harbour (7.33 m, 1 445 pings), with **1.5 dB
rms** of shape about that mean at each (a per-anchor statistic, not the
0.92 dB joint fit objective quoted in the `tvg_compensation` row). The reported *peak* is a separate
statistic and is still **+4 to +7 dB** hot with the right slope
(−11.5 against −11.28 dB per decade of altitude): the near-nadir return is
what the fit's tail window deliberately excludes, and closing that last
5 dB is what the near-field beam and the specular width would do — neither
is determined by this corpus.

**How far the single-altitude fit got, and what stopped it** (kept as the
record of the method that produced the 97.2 dB anchor). Pooled residual
against the 40 m recording, over the far field past each band's own bottom
return: **14.0 dB RMS before the fit, 4.8–5.6 dB after**. The floor is not the
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
  off almost exactly: against the 2026-09-18 law, `tvg_compensation` 0.20
  and `lambert_exponent` **1.45** differ by **0.14 dB peak-to-peak** across
  6.0–14.5 m, under the reduction's own floor (smoke `[2e]`, which pins the
  washing pair and re-derives it whenever the law moves — it was 0.20 vs
  0.10 against the tvg-0 law, and 0.80 vs 1.945 before that). This ridge is
  why the single-altitude fit returned a confident wrong answer for a year,
  and why the current constants were fitted on **two** altitudes 5.9 apart
  (1.40 m and 7.33 m) rather than on the 4–20 m spread of one site, where
  geometry swamps the lever.
* The per-range mean-dB curve — the statistic the fit compares — has a
  repeatability floor of **0.67 dB RMS at 80 pings** over 6.0–14.5 m.

`analysis/calibration.py` carries the reduction and
`sss_calibration_report` runs it against any recording and bundle; it
refuses to score a pairing whose altitude band or range setting does not
overlap, because that residual would measure the geometry gap.
The runbook the *first* fit followed (a `.claude/specs/` document, retired
with that directory on 2026-09-18) is superseded twice over and is gone: its
Step 2 count-span plan by the first bullet above, and its whole
single-altitude method by the 2026-09-18 two-altitude fit — `bath3` at
1.40 m together with `reflectionEvidenceFullharbour` at 7.33 m, which is
what this section now records. Nothing points at it any more.

**Timing.** Ping period = `max(msec_per_ping, 2R/c + 3 ms,
1/max_ping_rate_hz)` — the device law, **measured on all seven corpus
recordings** (2026-09-02): 50 ms at 18.4 m and 20 m, 54 ms at 38 m, 56 at
40, 73 at 52, 99 at 72, 110 at 80, 131 at 95.6, 170 at 125.6 m, i.e. the
spec sheet's "20 pings/s up to 30 m, thereafter limited by range and speed
of sound" with ~3 ms of processing. (An earlier comment claimed a 22 ms /
45 Hz free-run at 15 m; the corpus contradicts it and it is gone.) A 30 m
range therefore pings at 20 Hz (so does the survey default, 20 m) and a
95.6 m survey at 7.6 Hz —
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
* **A3 — no *separated* volume scattering.** `volume_scattering_coeff`
  exists (Bell & Linnett 1997 eq. 12's `σ_bv` beside the surface `σ_br`)
  and ships at **0**, deliberately: the Lambert coefficient the model is
  calibrated on is an empirical total over the corpus's grazing band
  (J&R par. 12.1), so it already contains whatever volume term that band
  sees, and a fitted value here would trade against
  `calibration_db_offset` and `lambert_exponent` and collapse the `[2e]`
  identifiability ridge from two parameters to three. Separating them needs
  a capture over a known bottom at several grazing angles. Water absorption
  inhomogeneity is still unmodelled.
* **A4 — static scene.** No mid-run object motion, no vegetation sway.
* **A5 — stop-and-hop pings.** No intra-ping motion blur; inter-ping
  motion (attitude, advance) is fully modeled. The finite 0.5° azimuth
  beam **is** modeled (multi-line integration, §1).
* **A6 — 2.5-D scene.** Heightfield world: no overhangs, objects
  represented as height + reflectivity (+ transmission, + roughness) stamps.
  Since 2026-09-19 the *brightness* of such a stamp no longer depends on the
  grid it is stamped on — the surface-area element makes a one-cell wall
  contribute the wall's true height (§3.1), gated at 2 dB over 0.20 / 0.10 /
  0.05 m by smoke `[8]`. What the raster still quantises is the object's
  *outline*, and its lattice members' count. Since 2026-09-03 the stamp
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
  radiometric degradations are deferred to the (optional) augmentation stage
  (`gain_drift_amp`, off by default). The real device's AGC is not
  incidental — it walks indices 4–7 inside every field recording — and
  `fault_gain_walk_pings` reproduces that walk on demand so the downstream
  gain compensation is exercised (§5). **Revised 2026-09-18:** the AGC is
  no longer the explanation for the level. The device's reported peak
  follows altitude as **66.60 − 11.28 · log₁₀ alt** dB over 19 gain-4
  recordings spanning 0.92–12.19 m (rms 1.57), which is a *law*, not a
  regulator holding a constant; what made the model's peak fall 20–27 dB
  per decade instead was giving the coherent component the diffuse field's
  spreading. With that fixed, `calibration_db_offset` is determined by the
  seabed level itself to well under a dB at both altitudes (§6), and the
  ±6 dB caveat is retired.
* **A10 — the bottom type behind the fitted constants is unsampled.** The
  harbour corpus's sediment was never sampled. **Weakened 2026-09-18:** the
  beach corpus *is* over known sand, and fitting the two together does
  determine `lambert_exponent` (1.0, against 0.75 and 1.25 both scoring
  worse, §6). What remains confounded is the *harbour* bottom, and any
  per-material angular law: the renderer applies one global exponent,
  measured over sand, and `Material.lambert_exp` — a per-material exponent
  read by nothing, which claimed 1.6 for the sand the corpus measures at
  1.0 — was deleted rather than wired. A per-material angular response is
  a real effect [J&R par. 12.1]; nothing here measures it.
* **A12 — the near-field beam is assumed focused.** The along-track
  footprint is `r · θ` at every range. An unfocused 0.38 m array (what a
  0.5° beam at 450 kHz implies) would floor it at the aperture across the
  whole swath, and the corpus's own along-track texture correlation rules
  that out at 12 m (§6, `array_length_m`). Inside ~3 m nothing measures it,
  and the knob is where the answer goes.
* **A13 — beyond the grid, the seabed is a phantom plain unless the world
  says otherwise (2026-10-21).** `SceneModel.sample_height` answers the
  deepest cell of the map outside the grid, and `sample_reflectivity` 0.2
  (`outside: fill`, the historical behaviour the render fixture is frozen
  on). A boat within the sonar range (20 m) of the world edge therefore
  sees a step down to that level, its horizon-cull shadow band, and a
  ρ = 0.2 plain (*e.g.* edge at 10 m, local 3 m, deepest cell 5 m: a dark
  band from 10 to ≈ 16.7 m). The World Builder builds `outside: edge`
  worlds, which repeat the nearest edge cell outward (the manifest key is
  additive; absent means `fill`); `generate_scene` and every world already
  on disk keep `fill`. Either way, keep the survey at least `range_max`
  inside the limits when the edge matters; an object on the edge row
  extends outward as a strip under `edge`. It is no longer silent: every
  ping reports where its centre ground line leaves the map
  (`Ping.out_of_map_from_m`, per side on `~/ground_truth/sensor`, one node
  warning per run), so a dataset can drop or flag those ranges; the render
  itself is unchanged.
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
first 60–180 of the default 1200 bins (16.7 mm/bin at 20 m). The rendered
ping guarantees the structure bottom-tracking bootstraps assume: a quiet
water column (the transmit ringing decaying from 55 dB over the first
metre, then the measured `watercolumn_noise` level, 25.3 dB reported and
28–43 dB under the FBR peak depending on altitude — §5), a sharp bright ramp
at slant range = altitude (the specular lobe on the beam pattern), then the Lambert-shaded seabed decay. Smoke `[2b]` gates
the gap→peak contrast at 24–44 dB (~31 dB with the ringing bins averaged
in; the corpus reads a median of about 32 dB, 10–90 % 25–42, on the `[2f]`
statistic — bins 10 to FBR−8, ringing tail included); a simple "first bin > 3× water-column median" detector
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
`scene_manifest.yaml`, so the renderer, the ground-truth contacts and the Gazebo world all
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
| The surface image is evaluated at **−depression** | That path leaves the transducer *upward*. The main lobe sits 20° below horizontal (FWHM 50°), so it is attenuated by 8 dB at 20° up, 17 dB at 40° up, and held at the sidelobe floor (−24 dB) beyond 50.6° — which is why a near-perfect reflector does not swamp the calibrated direct field. |
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
  (Eckart 1953, Urick 1983 ch. 5; `k = 1885 rad/m` at 450 kHz, the model's `acoustics.wavenumber`, per-wall
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
* **Occlusion of the wall features themselves** (2026-09-18). Until this
  pass the face loop stopped at the first wet hit and neither the ghosts
  nor the corner band were tested against anything: the review's wall
  census found the band drawn straight through a nearer quay in **97 %**
  of the pings that saw two walls, and a pontoon silenced the quay behind
  it. Now each face point is tested against the nearer faces' own bands —
  so a quay *behind a pontoon* is imaged, because the ray passes under the
  floats, and a quay behind a quay is not — and against the terrain
  horizon at that distance, so a bank hides what stands beyond it. The
  folded paths are culled on **both real legs**, transducer → face and
  face → ground (`multipath.segment_blocked`, `reflection_point`), and the
  corner band additionally on its own direct leg. Smoke `[4d]` gates both:
  a second quay behind the first adds **±0.0 dB** past its foot, and a quay
  behind a pontoon returns **+78 dB** over the same scene without it.
  **The ghost's own first leg** (2026-10-21): a wall ghost is rendered from
  the mirror source, so the stretch of its line before the wall plane lies
  behind the wall -- but that stretch is the mirror image of the real
  transducer → face leg. Its heights (and facet occluders and
  transmission) are now read at the reflected, near-side point; before,
  land or a high apron behind a quay set the horizon there and deleted
  the ghost outright (the harbour recipe `land` + `quay`: ghost energy 0
  instead of the water-behind value). Flat basins are unchanged; smoke
  `[15]`.
* **Smooth objects behind a wall** (2026-10-21). The facet layer is
  occluded by the walls exactly as the seabed is (the direct path's band
  rule per patch, `multipath.segment_blocked`). Before, a glint behind a
  quay was painted while its ground truth said "not visible"; smoke
  `[15]` pins image and label together (and a pontoon whose draft the ray
  passes under still lets the object be seen).
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

## 12. Objects: material physics and the facet layer (NC #17, 2026-09-28)

**Why.** The field's NAFCO box (`beach_sept/object_boiteVsDistance`, the two
passes where it shows; GCS seabed-image dB, linear means against the same
range away from the box) reads **+1.1 / +0.2 dB** near-edge at 6.1 / 10.1 m
slant (12.8° / 7.7° grazing), a footprint of about **−5 / −6.5 dB** and a
shadow of **−8.2 / −7.4 dB**: almost a shadow alone. The simulated sessions of
2026-09-28 read **+15.9 dB** and up. Three causes, each measured by rendering
the field geometry offline:

1. **A plumbing bug.** The World Builder's build path
   (`world_folder._generate_with_base`) never built the roughness raster, so
   every GUI world from 2026-09-19 to 2026-09-28 rendered its smooth objects
   with sand's specular lobe and `r²` law: **+19.8 / +24.6 dB** near-edge,
   where the raster gives +4.9 / +5.6. Both builders now share one stamping
   pass (`scene.stamp_objects`), smoke `[9]` compares every raster, and
   `SceneModel.stale_rasters()` names a world built before the model it needs.
2. **One heightfield surface for a hollow smooth shell.** Even with the
   raster, +4–5 dB remained, *independent of the box's own ρ*: bilinear
   sampling carries the sand's reflectivity onto the one-cell face ramp, whose
   layover fold does the rest; and the lid's far edge left a **−20 to −30 dB**
   band where the sand under the lid should show through.
3. **Lambert's law on a mirror.** `pp_shell`'s ρ 0.18 was the fluid-layer
   power reflection coefficient used as a Lambert coefficient. Moulded PP is
   smooth to 0.1 mm at a 3.3 mm wavelength (`2kσ = 0.38`): a mirror, for which
   Lambert's law does not hold (Coiras & Groen 2009, eq. 11 caveat; J&R 2007
   ch. 13). No open simulator does better — Stonefish, HoloOcean, DAVE,
   OceanSim all use cos- or cos²-incidence times one reflectivity per object —
   so importing one would reproduce the glint (review of 2026-09-28).

**The rule.** An object is rendered from its material and geometry, never
from a per-object gain:

* **Rough materials** (RMS height ≥ `COHERENT_ROUGHNESS_LIMIT_M`, 1.6 mm:
  concrete, rope, tyre tread, generic) and **aggregates** of many members (the
  lattice classes, rope, chain, anchor, debris, coils): the raster's Lambert
  path, unchanged and bit-identical.
* **Smooth solid primitives** (smooth material on a box, cylinder, capsule,
  cone or annulus `Solid`, proud of the seabed — `objects.is_facet_object`;
  plastic, PVC, metal, glass, wood and fired brick, which at 1 mm is just
  under the limit: coherent fraction 6·10⁻⁷ at normal incidence but 0.5 at
  77° off its normal, so smoke `[14]` expects it here):
  the **facet layer** (`sonar/facets.py`). In the raster they only occlude.

**The facet layer.** The exposed surface is tessellated into patches
(`solids.tessellate`, `facet_patch_m` 1 cm, refined on curves until the
sagitta phase error stays under 0.05 rad). Each patch returns

* a **coherent** Kirchhoff (physical-optics) amplitude
  `(k/2π) · R · √C · cosθ · a b · sinc(k a ĝ·û) sinc(k b ĝ·v̂) · e^{2ikr}`,
  weighted by the beam's amplitude pattern (vertical, riding on the roll, and
  the azimuth Gaussian), summed as complex numbers, `σ = 4π|Σ|²`;
  `C = exp(−(2kσ_h cosθ)²)` is the Eckart coherent fraction;
* an **incoherent** residual `(1 − C) · ρ_inc · cos^p θ · A` in power, with
  `ρ_inc = ρ_sand |R / R_sand|²` (Kirchhoff scaling against J&R's medium sand,
  `R_sand` = 0.370) rather than a judged number.

Each patch is weighted by the **pulse envelope**, a Gaussian whose power FWHM
is the device's range resolution, in amplitude, normalised so `Σ g² = 1`. A
hard bin gate would cut an oblique face at every bin boundary and each cut
would ring like an edge: measured, a wall 5° off broadside summed to
−217 dBsm over the face but −21 dBsm in one bin. The coherent part joins the
low-CV `specular` channel, the incoherent part the speckled `power`, after the
renderer's own pulse smear. Patches below the occluder horizon in front of
their object (terrain, nearer objects) are dropped; back faces are culled.

**Units.** The renderer's bins are the calibrated seabed's, per unit area of
the resolution cell. A cross-section is put on that scale through **one
literature constant**, sand's Lambert parameter at 450 kHz: J&R Fig. 12.3
fits −20.2 dB at 20–40 kHz and §12.1.2 gives +6–8 dB/decade for sand, so
`sand_lambert_mu_db` = −20.2 + 7 log₁₀(450/30) = **−12.0 dB** (−13.1 to −10.8
on the slope; the site spread is ±5–8 dB). At each bin
`P = rng(r) · ρ_sand sin^p ψ / (μ sin²ψ) · 4π|Σ|² / (Δ · W)`, with ψ the local
grazing angle at the object's own depth, `Δ` the bin (energy formation) and
`W = √(2π) σ_az(r)` the beam's azimuth width. Nothing is fitted to an object;
an aligned field pass over a surveyed target is the one measurement that
would pin μ.

**The raster.** A facet object's bump goes into `height` (Gazebo, altitude
and ground truth are unchanged) and into the additive `facet_bump` raster;
the seabed under it keeps its own reflectivity and no roughness is stamped.
`_ping_geometry` scatters from `z − facet_bump` (the seabed beneath) and a
seabed sample hidden by a facet object's crest keeps that object's two-way
transmission: 0 when opaque (its shadow sits on the reverberation floor like
any other), the measured value for a flooded shell (its footprint and shadow
show the seabed beneath, dimmed). A shell's transmission is stamped over its
footprint grown by one cell, or the bilinear up-ramp would set the horizon
at 0 and leave a dark seam. Ground truth reads `visible` on the occluder
surface (`occ_lit`). A line with no facet object runs exactly as before.

For a class drawn as a lying cylinder or capsule (`pipe_pvc`, `can`,
`cylinder_metal`, `fire_extinguisher`, `bottle_plastic`, `bottle_glass`),
`proud_height` and `width` are drawn independently, but the solid the facet
layer scatters from is a cylinder of diameter `width`. On the flat stamp its
top is `min(effective_height, width)` above the seabed (`facets.object_patches`:
it never floats, and is cut by the seabed when lower), while the raster bump
that occludes is `effective_height` tall: a height above the diameter
lengthens the shadow but not the echo, and a rested (3-D) object ignores
`proud_height` altogether.

**Materials** (`worldgen/materials.py`, `_ACOUSTIC`; Selfridge 1985 Table I,
Kinsler et al. App. A10): density and longitudinal speed give the impedance
and `|R|` — half-space `(Z − Zw)/(Z + Zw)` for a solid, the fluid-layer
formula for a flooded wall (`backing: water`, `wall_m`), `|R| = 1` when air is
behind a thin wall (`backing: air`). Resulting `|R|`: steel **−0.6 dB**, glass
−1.9, PVC −8.8, HDPE −14.0, rubber −23.4, a 2 mm flooded PP wall **−7.9 dB**
(transparent at a half wavelength, 3.0 mm: −33 dB). The `plastic_box` class
knobs are read per instance: `wall_mm` sets the wall, `flooded: false` makes
an air-filled box (`|R| = 1`, transmission 0). Rubber is rated at its
**tread** (3 mm) since 2026-09-28: at the moulded sidewall's 0.2 mm it became
a mirror of `R` = 0.07 — invisible — which no SSS image of a tyre shows.

**Validation — textbook, then field** (smoke `[14]`):

| Target | Model | Closed form |
|---|---|---|
| flat plate 0.10 × 0.06 m, normal incidence | 16.10 dBsm | 4πA²/λ² = 16.10 |
| its aspect at 0.5 / 1.5 / 2.5° | residuals ≤ 0.1 dB | sinc²(k a sinθ) |
| lying cylinder a = 0.1, L = 0.3 m, broadside | 12.33 dBsm | k a L² = 12.30 |
| sphere a = 0.1 m | −15.17 dBsm | π a² = −15.03 |

At the field's own geometry (1200 bins over 19.711 m, transducer 1.35 m over
the seabed at the box), over every heading (the field did not survey it):

| | near-edge 0.4 m | footprint | shadow |
|---|---|---|---|
| field, 6.1 m | +1.1 | ≈ −5 | −8.2 |
| model, 6.1 m, headings 2.5–87.5° | +0.1 … +1.3 | −5.4 (mean) | −8.0 (mean) |
| field, 10.1 m | +0.2 | ≈ −6.5 | −7.4 |
| model, 10.1 m, headings 2.5–87.5° | +0.2 … +2.9 | −4.5 (mean) | −8.3 (mean) |
| model, aligned with the track (0°) | **+15 / +24** (a glint) | | |

34 of 36 headings land within 2 dB of the field's near-edge; the two that do
not are the skirts of the two glints (2.5° from either alignment). A wall
glints only within about **1°** of broadside at 450 kHz — +4.5 dB at 1°,
+2 dB at 2° — which is why a box of unsurveyed heading reads as a shadow: the
aspect dependence the thesis rests on now *emerges* from the geometry. No
bin between the box and the end of its shadow falls below the transmitted
level (worst 5 cm: −9.4 dB). `pp_shell`'s transmission 0.11 is re-measured
through this renderer: averaged over headings it is within 0.25 dB of the two
field shadows (0.13 would be +0.26), so it stands.

**Cost.** About 1.2 ms per smooth object in the beam per ping-side (a 200 L
drum, 30 000 patches, 2.5 ms): 5.8 ms bare, 17.5 ms with ten boxes in one
beam at once — inside the 25 ms real-time budget of 20 pings/s on both sides.
Patches are cached per object (48 objects, LRU).

**Limits, recorded.** `R` is the normal-incidence value; multiple reflections
inside an object (the far wall of a shell, the dihedral with a rough seabed)
are ignored; there is no fringe (PTD) term beyond physical optics; facet
objects keep their shadows in the multipath ghosts but not their highlights;
patch validity is R ≳ 2 m; μ_sand is ±5 dB from literature. The field box's
faint +2 dB per-row maximum (its rim, buckles and lid ribs) is below what a
general model can derive and is carried, at about the right level, by the
incoherent residual. `object_facets: false` keeps every object's occlusion and
drops its scattering: a diagnostic that shows a shadow alone.

**Measuring an object in a recording.**
`python3 -m blueboat_sss_sim.analysis.object_signature <session>/seabed_images
--at X Y` reduces any GCS session, simulated or real, pass by pass to the
same near-edge / row-max / footprint / shadow / length (the point in that
session's GCS frame: local metres about its first GPS fix).
