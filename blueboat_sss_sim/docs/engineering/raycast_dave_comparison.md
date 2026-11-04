# Ray-cast sensor vs the DAVE multibeam sonar (Choi et al. 2021): what to port, what is missing, revised plan

2026-11-03, owner request: compare `raycast_design.md` with the reference implementation **before writing
more ray-cast code**. Status of this note: comparison done; the revised plan (§5) is a **proposal awaiting
the owner's decision**. `raycast_plan.md` gates are unchanged until then.

## 1. What was read

- Repository `IOES-Lab/dave` (Apache-2.0, copyright Naval Postgraduate School). The default branch is
  `ros2` (there is no `main`). Head **`c1c2fba995e7`** (pushed 2026-10-06), read 2026-11-03 through the
  GitHub API. The six files of `gazebo/dave_gz_multibeam_sonar/multibeam_sonar/` were downloaded as data,
  never built or run: `~/ros2_ws/sim_runs/raycast/ref/dave_main_20261103/` (commit in `SOURCE_COMMIT.txt`).
- `sonar_calculation_cuda.cu` (745 lines; last changed `0f20acd`, 2025-08-26), read in full.
  `MultibeamSonarSensor.cc` (1343 lines): parameters, beam/ray setup, normals, beam corrector, image output.
- Build target at that head: `gz-sim 10`, `gz-sensors 10`, `gz-rendering 10` (**Jetty**). Our stack is
  Jetty-free: ROS 2 Jazzy + Gazebo Harmonic (`gz-sim 8`). The branch `chore/lyrical-jetty-base-20260909`
  carries the Lyrical/Jetty move; the CUDA core has not changed since 2025-08, so it is the same on
  Harmonic-era commits.
- The paper (Choi21, doi:10.3389/frobt.2021.706646) was already read in Phase 0
  (`docs/literature/raycast_sonar/01_*.md`); this note is about the code.

## 2. The DAVE pipeline, as the code does it

1. **Visibility = one rasterised first hit per pixel.** Gazebo `GpuRays` renders a depth image from the
   sensor origin: columns = beams (odd count), rows = elevation rays, every `raySkips`-th row used. No
   secondary rays: direct path only, z-buffer visibility.
2. **Normals from the depth image** by 3×3 Sobel filters, with a "focal length" guessed in the code
   (comment: "Not sure if this is the right way"). Incidence = `acos(n_z)`: the angle to the camera axis,
   not to the ray (`compute_incidence` is commented out).
3. **One point scatterer per pixel**, amplitude
   `ξ · 10^(SL/20)·1 µPa · e^{−2αr}/r² · √μ cos θ · √(r·Δaz·Δel) · 1`:
   - `ξ` complex Gaussian/√2 (Brown17), `curand` Philox keyed by the **pixel index**, seed 1234 (or
     `time(NULL)` with `blazingSonarImage`);
   - `μ` = **constant 1e-3** (`constMu`), the reflectivity image is never filled from the scene;
   - `α` = **fixed 0.0354 dB/m** (the code's comment says Francois–Garrison is "available");
   - the area term is `√(r·ΔΩ)` (area ∝ r), not the `r²ΔΩ/cos θ` footprint of a ray. Not resolved here; it
     is not a term we would port;
   - the beam pattern in the kernel is 1 (the elevation pattern is commented out).
4. **Frequency-domain synthesis**: `nFreq = ⌈B · 2R_max/c⌉` baseband bins over `[−B/2, B/2]`, each hit
   adds `amp · e^{i 2 k_f r}` with the baseband `k_f` only (the carrier phase is dropped; `sonarFreq` is
   unused in the kernel). A block reduction sums the elevation rays per (beam, frequency).
5. **Azimuth pattern applied afterwards** as an `nBeams × nBeams` matrix of `|sinc(0.884π/Δφ · sin Δφ_bb')|`
   ("beam culling correction", cuBLAS SGEMM), normalised by its root sum of squares.
6. **Batched 1-D cuFFT** over frequency → a complex range series per beam, × Δf; published as
   `marine_acoustic_msgs/ProjectedSonarImage` in dB and as a fan image.
7. A Hamming window is computed in the sensor and passed in, but **never referenced in the CUDA path** at
   this commit.
8. GPU engineering: persistent device buffers and pinned host memory, a deterministic two-stage reduction
   (per-ray-per-frequency writes, then a per-(beam, frequency) shared-memory reduction, no atomics),
   cuBLAS, cuFFT. Building it needs `nvcc` (CUDA toolkit), which this PC does not have.

## 3. Component comparison with `raycast_design.md`

| Component | DAVE (c1c2fba) | Ray-cast sensor (design + built in Phase 2) | Port? |
|---|---|---|---|
| Visibility | first hit of a raster from the sensor origin | one BVH oracle, watertight, every leg of every path (Phase 1–2 gates) | no |
| Contributors | one per pixel, moves with the sensor | world-fixed stratified lattice on the mesh (hash), refined on slopes | no (ours is required for along-track correlation C3) |
| Random factor | Brown17 complex Gaussian, keyed by pixel, per frame | Brown17 complex Gaussian, keyed by world cell | already equivalent in form; keep our keying |
| Scattering law | constant-μ Lambert, incidence vs the camera axis | J&R small-slope bistatic law + volume, per material; mean field (PO) for the coherent part | no |
| Normals | Sobel on the depth image | exact mesh normals; smoothed normals only for windows and Fermat | no |
| Beam pattern | elevation = 1; azimuth as a post-hoc beam-space matrix | 3-D near-field along-track aperture + vertical strip, per contributor (B1–B3) | no (a beam-space matrix has no meaning for one beam per side) |
| Range response | rectangular spectrum of width B (sinc in range), FFT | time domain: CW 44 µs ⊛ 1.4 τ filter ⊛ bin (measured, X2) | **idea**, see P-c |
| Broadband terms | exact per frequency (any `f`-dependent factor can be applied per bin) | carrier-frequency kernel; omits the PO envelope-derivative term (≤ 1.2 % at the envelope flanks) | **yes, as P-c** |
| Absorption, sound speed | fixed 0.0354 dB/m; one c for time and range | Francois–Garrison per site; true c for delays, 1500 m/s for bins (2.5–3 % compression) | no |
| Multipath, FBR, diffraction, transmission | none | surface/floor/wall chains, comb, mean-field FBR, Fresnel–Kirchhoff, thin-layer transmission (S4–S6, P1–P6, D1–D2, O4–O6) | no |
| Motion | per frame from the sensor pose | full attitude now; Tx/Rx poses per contribution in Phase 3 | no |
| Device chain | dB of the field; nominal SL 220 dB | shared encoder/noise stage, TVG, gain ladder, ceiling, ringing, Ping-protocol frames | no |
| GPU accumulation | deterministic two-stage reduction, persistent buffers | planned: Warp, atomics (G1 tolerance "≈ CPU, not bitwise") | **yes, as P-a/P-b** |
| Interface | Gazebo sensor plugin (Jetty), `ProjectedSonarImage` | ROS node, `OmniscanProfile` + raw frames (CM-1/CM-2), Harmonic | no |
| Tests | no closed-form or level tests in the package | validation ladder `test/raycast_test.py` | — |

## 4. What can be ported, and what DAVE lacks for side-scan

**Portable (patterns and one idea, no code copied):**
- **P-a, deterministic GPU accumulation.** Write each contributor's taps to a (contributor-block, bin)
  buffer, then reduce per bin in a fixed order. Run-to-run bitwise reproducibility on the GPU, which the
  design note conceded (§10). Phase 4.
- **P-b, persistent device buffers + pinned host memory** across pings; one allocation per scene. Phase 4.
- **P-c, frequency-domain synthesis for the coherent contributors only** (PO patches, the floor's mean
  field: thousands, not millions). Every frequency-dependent factor (the Kirchhoff `i k_f/2π`, absorption
  α(f), the aperture pattern at k_f) is then exact over the band, which removes the carrier-kernel residual
  of S4/S5. The diffuse field stays in the time domain (it is ~100× cheaper there for the Omniscan's
  short pulse: about 11 envelope taps per contributor against `nFreq = B·2R/c ≈ 600` bins at 20 m).
  Optional, decided by the Phase-5 study.
- **Not ported:** pixel-keyed sampling and random factors, constant μ, camera-axis incidence, Sobel
  normals, fixed absorption, the √(rΔΩ) area term, the beam-space corrector, the Gazebo plugin and its
  messages (Jetty API; our sonar is deliberately not a Gazebo plugin). If any DAVE code is ever copied
  verbatim, its Apache-2.0 header and NOTICE attribution go with it; the plan above copies none.

**Missing in DAVE for side-scan (everything the brief asks for beyond a direct-path first hit):**
1. Side-scan geometry: one beam per side, 0.5° two-way along track with a near-field aperture
   (`2L²/λ = 36 m` > the 20 m swath), 50° vertical tilted 20°, nadir in the vertical pattern's null.
2. A waterfall: world-fixed scatterers re-illuminated ping to ping (along-track correlation), 6-DoF
   motion with a receive pose per contribution.
3. Every path beyond the first hit: surface (Lloyd), floor, walls, the image comb, the coherent floor
   return (FBR, NC #4), shadows with diffraction, transmission through thin shells.
4. Physical seabed and water: per-material scattering and impedance, site absorption, the device's
   assumed sound speed.
5. The Omniscan receiver and encoder: CW pulse ⊛ measured filter, partial TVG, gain ladder, receiver
   ceiling, ringing, per-ping 16-bit normalisation, byte-valid Ping-protocol frames, same-tick sides.
6. Ground-truth contacts, and calibration against a field corpus (DAVE runs at a nominal SL and μ).

**Conclusion.** DAVE is a fast, direct-path, first-hit, pixel-sampled forward-looking multibeam model.
It validates the point-scatterer + coherent-sum principle our design already uses (Brown17). It offers
GPU engineering patterns and one receiver idea. It does not shorten the side-scan-specific work: about
90 % of what `raycast_design.md` lists is absent from it, and the 10 % it has (direct path, Brown17
amplitudes, coherent sum) is already built and gated here (S1, C1, R1).

## 5. Proposed revised phase plan, with a go/no-go after the first held-out fit

The risk to retire first is not engineering but value: **does the ray-cast sensor beat the geometric
renderer on held-out real data?** The current plan answers that only at Phase 7, after 6-DoF, GPU,
convergence and integration. Proposal: answer it right after Phase 2, with the sensor as it stands.

| Step | Content | Gate |
|---|---|---|
| **2-close** | P7 (B1, wavy-wall world); Phase-2 report. Labels deferred to Phase 6 (not needed for the fit) | as written in `raycast_plan.md` |
| **A, first fit** (new; was part of Phase 7) | Corpus scene replays through `calibration/scene_replay.py` with an additive `renderer=` kwarg, expected mode (exact mean power), recorded attitude at the ping time (Phase-2 pose model). **Fit on the calibration sessions only** (bath3, reflectionEvidenceFullharbour, pool FIT): `G_sys`, TVG `k`, sand `w₂`, `γ₂`, `f_v`, inside the §8/§14 bounds; `L` and `σ₂` fixed. CPU (no GPU yet); cost measured first, stop and report if it exceeds the machine | gates pre-registered in `~/calib/raycast/gates.yaml` before any corpus render |
| **GO / NO-GO** | **Held-out = the validation sessions** (never fitted): harbour 07-21, 07-22-AM, 07-22-PM-diffsv, pool 10-27-SV, pool 10-28-1w-line. The TEST hold-out (harbour 07-23, beach B2) **stays sealed** for the final test, as decided 2026-11-02. Criteria below | see below |
| 3 | 6-DoF: Tx/Rx poses per contribution (`predict_pose`), `motion=` kwarg | M6, T1–T3, wave-run replay |
| 4 | GPU (Warp in the raycast venv) with **P-a and P-b** | G1 + bitwise run-to-run repeatability + leak probe; ms per ping-side on the RTX 4060 |
| 5 | Convergence (§6), including the window settings, the diffraction directions, and **P-c** if the carrier-kernel residual matters | §6 acceptance |
| 6 | Integration (node, launch, profile, GCS, registry, docs) | as written |
| 7 | Final fit with the full sensor; the TEST hold-out opened once | as written |
| 8 | Comparison report; optionally a "DAVE-equivalent" configuration (direct path, constant-μ Lambert) as a third reference | as written |

**Go/no-go criteria (proposed numbers, to be frozen in `gates.yaml` before the fit):**
1. NC #4 held at the fitted set: 0 envelope regressions against the geometric renderer.
2. Seabed level vs slant on every validation run-side: rms no worse than the geometric renderer's on the
   same runs + 0.5 dB.
3. At least **two of four** motivating statistics better than the geometric renderer on validation:
   - (a) the near-nadir / first-return crest profile rms;
   - (b) range lag-2 / lag-3 correlation inside the corpus band (geometric 0.26 / 0.10 vs real
     0.10–0.17 / 0.03–0.10);
   - (c) the pool wall-comb tail 0.5–2 m beyond the face (geometric with its pool overlay +1.9 / +2.5 dB);
   - (d) the pool box shadow profile, 0–1 m behind the echo (geometric 3.4 / 4.6 dB too dark).
4. Every fitted parameter inside its bounds and none pinned at a bound (else flagged, counted as a fail).

- **GO:** 1, 2 and 4 hold, and 3 is met → Phases 3–8.
- **NO-GO:** a report of what failed, with numbers. The choice is the owner's: stop; continue for a named
  reason; or carry specific winning terms into a separate decision about the geometric renderer, which
  this project does not touch.

## 6. Other open-source side-scan simulators (survey, 2026-11-03)

The owner asked to finish with a better knowledge of what already exists. Searched on GitHub for code
with a side-scan sensor; sources saved as data (read, never built) in
`~/ros2_ws/sim_runs/raycast/ref/sss_survey_20261103/`.

| Simulator | What its side-scan does | Licence | What it gives this project |
|---|---|---|---|
| **DAVE** (`IOES-Lab/dave`, `ros2`) | no side-scan sensor in the tree; the multibeam above | Apache-2.0 | §4 patterns |
| **Stonefish** (`patrykcieslak/stonefish` @ master, `Library/{src,include}/…/SSS.*`, `OpenGLSSS.cpp`, `shaders/sss*.comp`) | rasterises the first hit into a range/intensity texture over vertical beam samples; range histograms per sample; a smoothstep vertical taper and a `1/sin θ` "flat-bottom" compensation; Gaussian multiplicative and distance-scaled additive noise; gain and clamp; a compute shader scrolls the waterfall | **GPL-3.0** (ideas only, no code may be copied) | confirms that a real-time SSS is a first-hit raster plus heuristic noise; nothing physical to port |
| **HoloOcean** (`byu-holoocean`; engine source not on GitHub, the docs repo is) | three back-ends: octree sampling, CPU ray cast, GPU raster; narrow along-track width (e.g. 0.25°) over a wide fan; per-material reflectivity table; Gaussian `AddSigma`/`MultSigma` noise; water density and sound speed for an impedance approximation; a documented angular-sampling rule that guarantees every range bin is populated | docs only | the sampling rule is the analogue of our NC #9 coupling (the ray caster's world lattice guarantees it by construction); no multipath, coherence or device chain documented |

**Where the field stands, as far as public code shows.** Every open side-scan or multibeam simulator found
renders the **first hit** only. None has:
- multipath with its own visibility;
- a coherent mean-field first bottom return;
- diffraction or transmission;
- a near-field aperture;
- the device's receive chain;
- calibration against field recordings.

DAVE is the only one with a coherent point-scatterer sum (Brown17). The ray-cast sensor's distinct
content is therefore genuinely new relative to public tools. Its value is not established by that: only
the go/no-go on real data (§5) can establish it.

## 7. Credits

The GPU accumulation and buffer patterns adopted for Phase 4 (P-a deterministic two-stage reduction,
P-b persistent device buffers) and the frequency-domain synthesis idea (P-c) are taken from **DAVE's
multibeam sonar** — Choi, W.-S., Olson, D. R., Davis, D., Zhang, M., Racson, A., Bingham, B., McCarrin,
M., Vogt, C., Herman, J., "Physics-based modelling and simulation of multibeam echosounder perception
for autonomous underwater manipulation", *Front. Robot. AI* 8:706646 (2021),
doi:10.3389/frobt.2021.706646; code `IOES-Lab/dave` (Apache-2.0, Naval Postgraduate School). They are
re-implemented, not copied; each module that uses them cites this note.
