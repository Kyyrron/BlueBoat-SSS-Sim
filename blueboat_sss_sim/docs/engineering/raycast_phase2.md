# Ray-cast sensor — Phase 2 report (IN PROGRESS): CPU reference renderer, path by path

Started 2026-11-02. Gate: `raycast_plan.md` §Phase 2. Tolerances: `raycast_design.md` §7. Command:
`python3 -m test.raycast_test [--quick]` from the package source root. Last run: 43 passed,
0 failed, 1 skipped, in 347 s (quick). Smoke test 493/493.

## Correction and additions (2026-11-03)

* **Every corpus site is seawater** (owner). The Phase-0 fresh-pool assumption is withdrawn.
  - Per-site water: NOAA OISST temperatures ± 3 °C (harbour 28.5, beach 27.5, pool 26.0 °C), S 35 ± 3
    ppt; α = 0.163–0.177 dB/m (note 07).
  - Default `raycast.water.temperature_c` is 27.3 °C (their mean).
* **Range scale:** the device reports `sos_dmps` = 15000 in every corpus file, and the true c is
  1537–1545 m/s, so real images are range-compressed 2.5–3.0 %. The ray caster reproduces it; the
  geometric renderer does not (`TODO.md` P1).
* **Aperture:**
  - derived once at the device's nominal 1500 m/s (a physical length, 24.6 cm), radiated at the
    site's k;
  - bounded [22.4, 31.0] cm by the datasheet far field and the CAD housing (318 mm, owner);
  - `raycast.beam.aperture_length_m` overrides it inside the bounds.

## Status per path

| Path | Status |
|---|---|
| **direct** | built; A1, R1, B1–B3, S1–S4, C1, O1–O3 and M6 pass. **Open: NC #4** (see below), S5 (Eckart √C on bounce legs, wanted with the bounce paths), the incoherent remainder of smooth solids, and rough walls' coherent share |
| surface | `paths.py` (direct, surface image with Eckart √C, both legs ray-cast) and the grouped coherent receiver (`BinGrid.add_group`: one scatterer's path pairs interfere). **S6 and P1 pass** (Lloyd nulls exact to 1.3e-5; two-way nulls −36/−37 dB at 23/29 m). Renderer integration (every scatterer's four pairs, bistatic law) is next |
| bottom / comb | not started |
| wall | not started (wall triangles already scatter diffusely on the direct path) |
| dihedral | not started |
| diffraction | not started |
| transmission / internal reflection | not started |

## What was built

* `sonar/raycast/propagation.py`:
  - Francois–Garrison absorption, verbatim from J&R B.6–B.12;
  - sound speed by J&R B.10;
  - per-leg spherical spreading.
* `sonar/raycast/beam.py`:
  - the along-track **near-field** line aperture (L = 24.6 cm from the datasheet's two-way 0.5°),
    a Gauss–Legendre integral over exact distances;
  - the vertical aperture (3.5 mm from 50°, far field) about the axis tilted 20°;
  - a baffle (no radiation behind the face).
* `sonar/raycast/receiver.py`:
  - envelope = CW pulse ⊛ receive filter ⊛ bin integration (a bell), centred;
  - time-to-bin mapping with the device's assumed 1500 m/s;
  - coherent and mean accumulators.
* `sonar/raycast/scatterers.py`:
  - world-fixed, **stratified** (jittered-lattice) scatterers on the seabed sheet, refined on steep
    cells;
  - a planar lattice for other triangles, clipped to the beam band;
  - splitmix64 hashing.
* `sonar/raycast/seabed_ssa.py`:
  - the J&R **small-slope** roughness law (13.11, 13.15–13.17, D.11–D.13) with A_ww (13.21–13.25);
  - fluid–fluid reflection (8.46–8.52);
  - the sediment volume term (14.6–14.9);
  - the I_K integral via a small-q series (Mellin transform of J₀), a large-q series (J₀ Taylor
    series) and quadrature, tabulated log–log.
* `sonar/raycast/scattering.py`, per material:
  - sediments (sand, mud) take the small-slope law;
  - rough manufactured materials take the sand law scaled by `|R_mat/R_sand|²`, the facet layer's
    own rule;
  - gravel, rocks and seagrass fall back to sand, with a warning (a gap).
* `sonar/raycast/renderer.py`, `RaycastRenderer(SonarRenderer)`:
  - the beam band from the actual two-way pattern;
  - the direct path over diffuse scatterers;
  - the coherent PO path of smooth solids (the facet layer's placement and tessellation, refined).
  Output: `power` = exact mean, `texture` = realisation / mean.
* `sonar/raycast/config.py`: strict `RaycastConfig` (water, beam, sampling, seabed, receiver, po).

## Gate results so far

| Gate | Result |
|---|---|
| [A1] absorption at 450 kHz, five waters | matches an independent transcription to < 6e-5 dB/m; leg TL exact |
| [B1] near-field aperture | GL = dense exact quadrature to < 1e-4 dB, 0.5–20 m, to the 3rd sidelobe |
| [B2] far field | two-way −3 dB width 0.5000° |
| [B3] vertical | −3.0103 dB at 25° |
| [R1] point target | peak at the device range r·1500/c to 0.08 bin; profile = envelope² to 1 % |
| [M6] transmit pose | = `GeometricRenderer.sensor_pose` exactly |
| [S1] flat seabed, mean vs independent quadrature | median 0.004 dB, max 0.057 dB over 1028 bins (2.5 mm lattice) |
| [C1] speckle | texture ~ Exp(1): KS D 0.0196 < 0.0293 (n = 2160), mean 1.03, CV 0.99 |
| [O1–O3] PO closed forms at 15 m | plate −0.073, cylinder −0.052, sphere −0.056 dB |
| [S2] J₀; I_K vs closed forms | J₀ error 1.7e-10. I_K: γ₂ = 3 to 2.3e-8; γ₂ = 4 to 6.5e-10 at q ≥ 0.05; series = quadrature to < 1e-7 at the switch; table to 5.7e-4 |
| [S2] small slope → perturbation | ≤ 0.009 dB at 3–8° grazing (40 and 450 kHz) |
| [S2] reciprocity | 7e-16 |
| [S3] volume term | suppressed below the 30.5° critical angle: −36.6 vs −25.2 dB |
| [S4] flat reflection | \|V\| = 1 below the critical angle (lossless); Rayleigh at normal incidence |

The 450 kHz backscatter strength of the SAX99-sand law: −34.1 / −24.0 / −16.8 / −14.7 / −17.2 /
−17.5 / −18.1 / −18.2 dB at 5 / 10 / 20 / 30 / 45 / 60 / 80 / 89°.

CPU reference cost (direct path, flat 60 m scene, 20 m range): 4.6 s per ping-side at the 5 mm
lattice; 19 s at 2.5 mm.

## What failed on the way (each found by a gate), and the fixes

1. **S1, the last half-metre: the device's sound speed was ignored in the reach.** The device maps
   time to range at 1500 m/s, so its 20 m bin is 20.24 m in true range at c = 1518 m/s. The
   contributor cut was at 20.08 m (also with a doubled envelope margin). Fixed: `r_max` covers the
   last bin in true range plus half the envelope.
2. **S1 sampling error vs lattice spacing:** rms 0.10 / 0.025 / 0.006 dB at 10 / 5 / 2.5 mm,
   falling as ~spacing² (stratification works; no bias). The provisional default is 5 mm; Phase 5
   decides.
3. **O2: the facet layer's tessellation leaves a cylinder 0.20 dB low even at 500 m.** The default
   curvature criterion (0.05 rad sagitta phase) tiles the curved surface with planar patches that act
   as a grating. The exact PO integral converges with 4× finer circumferential patches. The ray caster
   tessellates with a 16× stricter criterion (`PoConfig.curvature_refine`). **Finding about the
   geometric renderer (unchanged):** its facet layer carries the same ~0.2 dB low bias on small
   cylinders; smoke `[14]` checks at 0.5 dB, so it passes.
4. **S2, I_K at large q:** the quadrature was under-resolved (all 32 nodes over [0, 2.4] while the
   integrand lives in [0, 0.01]). Fixed by subdividing over the decay scale and adding the large-q
   series.

## Open physics question for the NC #4 step (decision pending, no guess)

With a uniform 3.5 mm vertical aperture tilted 20°, nadir is 70° off-axis, near the pattern's first
null (one-way amplitude ≈ 0.004, about −48 dB two-way). And the small-slope sand law at 450 kHz has
no strong near-nadir peak (−18 dB at 89° vs −14.7 dB at 30°). Both would weaken the first bottom
return NC #4 protects. The real transducer's response at large angles is unpublished. The geometric
renderer handles the same issue with a judged `beam_sidelobe_floor` (0.004) and the fitted
`specular_strength`. The plan for the NC4 step:
- measure the corpus's FBR-to-seabed ratio at known altitudes;
- treat the vertical pattern's large-angle level (a baffle/housing fill of the nulls) as a bounded
  parameter fitted on the calibration sessions only (`raycast_design.md` §8);
- record it, never tune it per scene.

## Not yet done in Phase 2

- Bounce paths (surface, bottom, wall, dihedral, image comb) with S5, P1–P7, O4–O5.
- Diffraction (D1, D2).
- Transmission (O6).
- The NC4 envelope.
- Labels (ground truth read off the result).
- Registry entries for the new constants (planned for Phase 6 with the integration, smoke `[16]`).

## Appendix A (2026-11-03): owner-supplied full texts, and what changed

- **Brown et al. 2017** (point-based model). Each scatterer now carries a complex circular Gaussian
  factor (|f| Rayleigh, world-fixed hash). The field is Gaussian at any density; the mean uses
  E|f|² = 1.
  - The Phase-5 metric moves from a same-seed realisation error to Brown17's statistics (mean, PDF,
    correlation), because refining the lattice draws new scatterers.
  - Gates re-run: R1, M6, S1 (median 0.004 dB, max 0.096 dB at the nadir onset), C1 (KS D 0.026 <
    0.029, CV 1.03): pass.
- **Svensson et al. 1999:** the BTM edge-source equations read (note 06); the reference for the
  diffraction step.
- **Hwang et al. 2015:** PO with GO path finding and PO at the last bounce; the corner closed forms
  follow from its effective area (O4, O5). Knott 2004 is now optional.
- **Williams et al. 2010:** validates image targets for the object–seabed interaction. Its numbers
  are low-frequency (no 450 kHz transfer).
- **Folds & Loggins 1977:** the elastic multilayer method for the transmission step.
- **Lyons & Abraham 1999:** non-Rayleigh statistics from patchiness (two-component mixtures).
- **McDaniel 1993:** reviews 3–60 kHz only, so no validated surface-backscatter law at 450 kHz; the
  term stays optional.
- **Trevorrow et al. 1994:** ship-wake M_v ≤ 0.3 m⁻¹, persisting ~7.5 min, from large vessels. An
  upper bound; the wake term stays optional pending a corpus check.

## Appendix B (2026-11-03): X1 two-unit cross-talk — excluded, with a detection limit

Record: `~/ros2_ws/sim_runs/raycast/notes/x1/X1_RESULT.md`, `tools/x1_crosstalk.py`.

- **Documents:** the Ping protocol's messages 169/170 assign each device a "channel" among the
  "number of sonars to be synchronized".
- **Corpus `channel_number`:**
  - 0/1 in every SonarLink/SonarView file (channels assigned, consistent with synchronised pinging);
  - 255 in all 15 of our driver's pool files (2026-10-28) and in reflectionEvidenceFullharbour. Our
    driver never sends 169/170, so these were unsynchronised.
- **Search in the unsynchronised files:** thin spikes (> 6 dB over a 15-bin range median) on slowly
  drifting straight lines (Hough over slope ±0.002…2 bin/ping, against a shuffled-ping null).
- **Detection limit by injection:** a 1-bin drifting line at +6 dB over the local level is recovered
  (best 1883 vs scene 1381 points at the 6 dB threshold); +3 dB is missed.
- **Result:** no consistent line. The best lines have per-file, per-side slopes, as wall geometry
  does. The candidate +0.089 bin/ping is not repeated in the other 5 files of the same power-up, nor
  with opposite sign on the other side.
- **Decision: excluded.** No cross-talk line ≥ +6 dB above the local level exists. The transducers
  face away from each other behind a baffle. A cross-talk below +6 dB over speckle is not excluded.

## Appendix C (2026-11-03): X2 transmit waveform — CW, near-matched receive filter (measured)

Record: `~/ros2_ws/sim_runs/raycast/notes/x1/X2_WAVEFORM.md`, `tools/x2_waveform.py`.

- **Method:** the speckle's range correlation at lags 1–3, with the structure floor (lags 9–12)
  removed, fitted by a pulse ⊛ boxcar-filter ⊛ bin envelope at each file's acquisition (header pulse
  length).
- **Result:**
  - the fitted filter is **0.8–1.5 × τ** over τ = 44–208 µs: calibration median 1.43, validation
    ≈ 1.3;
  - as a fixed share of the round trip it would wander 0.14–0.36 %;
  - a response that scales with the pulse is the signature of a **CW tone burst with a near-matched
    receive filter**; a compressed chirp would not scale with τ.
- **Applied:** `ReceiverConfig.filter_pulse_ratio = 1.4` (measured), replacing the driver's 0.0015 ×
  round trip.
- **Residual:** the measured lag-3 tail (0.06–0.26) is heavier than a boxcar's. The filter shape is a
  Phase-7 refinement.


## Appendix D (2026-11-03): the NC #4 step

**Large-angle vertical level and TVG vs the corpus first-return profile** (`tools/nc4_fit.py`,
calibration sessions bath3 and weight_1time):
- The joint best fit (f_v 0.02, TVG k 6) leaves an rms of 5.1 dB, with a structural misfit.
- On bath3 (sand, 1.37 m) the corpus is +3.7 dB re its 2–4 m level 5 cm past the first return. The ray
  caster is 8–12 dB lower there, for any f_v in [0, 0.3].
- **So f_v is not the bottleneck.** The deficit is the seabed law at near-normal incidence: J&R's SAX99
  power-law spectrum extrapolated to 450 kHz gives a broad, weak near-specular lobe. The roughness
  spectrum at 450 kHz is what this profile constrains (a Phase-7 fit inside the J&R envelope).
- The pool cannot constrain sand: its floor is painted concrete.

**Detection envelope, first run** (`tools/nc4_detect.py`):
- Setup: provisional system gain 126.6 dB, anchored to the calibrated geometric renderer's flat-sand
  level at the bath3 geometry (3–10 m, spread 2.4 dB); TVG k 12; 60 noisy encoded pings per case.
- Result: 0 % detection at 1–10 m (20 m/1200) and at ≥ 5 m (40 m/600), where the geometric renderer
  detects 100 %.
- **Cause: a units bug, not physics.** The shared noise stage's `noise_floor` and
  `watercolumn_noise` are linear powers in the geometric renderer's units, paired with its 81.1 dB
  offset. Under the ray caster's 126.6 dB gain they reported 45 dB too loud (water column ~66 dB
  instead of ~25 dB), so the detector's threshold sat above the seabed.
- **Fix:** `raycast.config.shared_stage_model(base, gain)` rescales them to keep their measured
  reported levels (and sets `reverberation_db` 0). Every ray-cast profile builds its shared-stage
  config through it. Rerun below.

**Detection envelope, rerun after the units fix** (`~/ros2_ws/sim_runs/raycast/notes/nc4_detect_v2.log`):
- **20 m/1200:** 100 % (sand and mud) at 0.6–3 m (mud 97 % at 3 m). Sand: 100 % at 5 m, 90 % at
  7 m, 10 % at 10 m. Mud: 43 % at 5 m, 2 % at 7 m, 0 % at 10 m. The geometric renderer: 100 % everywhere.
- **40 m/600:** 0.6 m is 45 % / 8 % (the geometric renderer 73 % / 55 %); 1–2 m is 100 %.
- **NC #4 is NOT met** at the higher altitudes.

**Which lever?** At 7 m sand / 7 m mud / 10 m sand:
- baseline 0.85 / 0.00 / 0.12;
- f_v = 0.3: 1.00 / 0.07 / 0.12;
- small-scale spectrum w₂ ÷ 10: 0.00 / 0.00 / 0.00.

Neither bounded lever fixes it, and the smoother spectrum makes it worse.

**Cause: a missing contributor.** The floor's coherent specular return is not implemented yet: the
image of the transducer in the local floor plane, with amplitude V·√C over the folded path. It is the
"bottom" path of this phase (design §2: "the floor's specular lobe → FBR imaged").
- Its coherent fraction comes from the sub-grid roughness over the Fresnel zone, from the same
  spectrum, so it is not a new parameter.
- SAX99 as extrapolated: RMS ≈ 1.3 mm over a 5 cm patch, C ≈ e⁻²⁴, so no coherent return. Only the
  incoherent nadir return remains, falling ≈ 30 dB/decade.
- With w₂ ÷ 10: RMS ≈ 0.4 mm, C ≈ 0.09. The coherent image return falls 20 dB/decade.
- The corpus first-return law, 66.60 − 11.28 log₁₀ alt reported, is ≈ −23 dB/decade before a
  partial TVG of k ≈ 12: between the two, nearer the coherent law. That is also why the geometric
  renderer needed its image-source specular lobe (NC #4 piece 5).

**Plan:**
1. Implement the bottom path's coherent floor channel (PO of the floor near the specular point, or
   the equivalent image construction with √C), with S4/S5.
2. Re-run the NC #4 envelope.
3. The 450 kHz spectrum (w₂, γ₂), f_v and TVG k are then fitted together in Phase 7 on calibration
   sessions (the near-nadir crest profile, the altitude law, and roll for the pattern).
