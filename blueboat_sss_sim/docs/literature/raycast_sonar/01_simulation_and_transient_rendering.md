# 01 — Sonar simulation by ray tracing; transient rendering as the formal analogue

Part of the ray-cast sensor's literature corpus (`INDEX.md`). Each DOI here was resolved
through Crossref on 2026-11-02 (`INDEX.md` gives the method). The "Read" line says how much of
the source was read.

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| Bell97a | Bell & Linnett, "Simulation and analysis of synthetic sidescan sonar images", *IEE Proc. Radar Sonar Navig.* 144(4):219–226, 1997 | 10.1049/ip-rsn:19971311 | full text (local PDF) |
| Bell97b | Bell, "Application of optical ray tracing techniques to the simulation of sonar images", *Opt. Eng.* 36(6):1806–1813, 1997 | 10.1117/1.601325 | record only |
| Cer17 | Cerqueira et al., "A novel GPU-based sonar simulator for real-time applications", *Computers & Graphics* 68:66–76, 2017 | 10.1016/j.cag.2017.08.008 | record only |
| DeM15 | DeMarco, West, Howard, "A computationally-efficient 2D imaging sonar model for underwater robotics simulations in Gazebo", OCEANS 2015 | 10.23919/OCEANS.2015.7404349 | record only |
| Gwon17 | Gwon et al., "Development of a side scan sonar module for the underwater simulator", URAI 2017, pp. 662–665 | 10.1109/URAI.2017.7992789 | record only |
| Choi21 | Choi et al., "Physics-based modelling and simulation of multibeam echosounder perception for autonomous underwater manipulation", *Front. Robot. AI* 8:706646, 2021 | 10.3389/frobt.2021.706646 | full text (open access, web) |
| Pot22 | Potokar et al., "HoloOcean: Realistic Sonar Simulation", IROS 2022, pp. 8450–8456 | 10.1109/IROS47612.2022.9981119 | abstract (`literature_review.md` [1]) |
| Pot24 | Potokar et al., "HoloOcean: A full-featured marine robotics simulator…", *IEEE JOE* 49(4):1322–1336, 2024 | 10.1109/JOE.2024.3410290 | abstract (`literature_review.md` [2]) |
| Kwak15 | Kwak, Ji, Yamashita, Asama, "Development of acoustic camera-imaging simulator based on novel model", IEEE EEEIC 2015, pp. 1719–1724 | 10.1109/EEEIC.2015.7165431 | record only |
| Sac15 | Saç, Leblebicioğlu, Bozdağı Akar, "2D high-frequency forward-looking sonar simulator based on continuous surfaces approach", *Turk. J. Elec. Eng. & Comp. Sci.* 23:2289–2303, 2015 | 10.3906/elk-1305-188 | record only |
| Gue10 | Guériot & Sintes, "Forward looking sonar data simulation through tube tracing", OCEANS'10 Sydney | 10.1109/OCEANSSYD.2010.5603867 | record only |
| Pail09 | Pailhas, Petillot, Capus, Brown, "Real-time sidescan simulator and applications", OCEANS 2009-Europe | 10.1109/OCEANSE.2009.5278111 | record only |
| Coi09a | Coiras & Groen, "Simulation and 3D reconstruction of side-looking sonar images", in *Advances in Sonar Technology*, 2009 | 10.5772/39406 | record (`literature_review.md` [4]) |
| Coi09b | Coiras, Ramirez-Montesinos, Groen, "GPU-based simulation of side-looking sonar images", OCEANS 2009-Europe | 10.1109/OCEANSE.2009.5278213 | record (`literature_review.md` [5]) |
| Meng23 | Meng et al., "A high-efficiency side-scan sonar simulator for high-speed seabed mapping", *Sensors* 23(6):3083, 2023 | 10.3390/s23063083 | abstract (`literature_review.md` [3]) |
| Brown17 | Brown, Johnson, Olson, "A point-based scattering model for the incoherent component of the scattered field", *JASA* 141(3):EL210–EL215, 2017 | 10.1121/1.4976584 | record only — **full text needed** |
| Sung20 | Sung et al., "Realistic sonar image simulation using deep learning for underwater object detection", *IJCAS* 18(3):523–534, 2020 | 10.1007/s12555-019-0691-3 | abstract (local PDF) |
| Jar14 | Jarabo et al., "A framework for transient rendering", *ACM TOG* 33(6):177, 2014 | 10.1145/2661229.2661251 | record only |
| Jar17 | Jarabo, Masia, Marco, Gutierrez, "Recent advances in transient imaging: a computer graphics and vision perspective", *Visual Informatics* 1(1):65–79, 2017 | 10.1016/j.visinf.2017.01.008 | record only |
| Vel12 | Velten et al., "Recovering three-dimensional shape around a corner using ultrafast time-of-flight imaging", *Nat. Commun.* 3:745, 2012 | 10.1038/ncomms1747 | record only |

## What they establish

* **Bell97a** is the closest prior architecture to this sensor. Each ping is a set of rays leaving the
  transducer over the 3-D beam, not just in the athwartship plane. Rays are traced through horizontally
  stratified constant-gradient layers (circular arcs, radius `R = c / (g cos θ)`). They hit a
  triangulated heightfield or procedural primitives. Each ray then returns a two-way time and an
  intensity from the sonar equation `SL − DI − TL + TS`, with:
  - absorption from Francois–Garrison, per layer;
  - spreading from the ray-tube separation;
  - Jackson's **bistatic** roughness and volume law, with Kirchhoff near specular and perturbation
    elsewhere. Bell says the bistatic law is needed because of the multiple specular bounces.
  - a beam that is the product of two orthogonal line-array patterns (eq. 13–14).
  The paper's own limits: intensities add **incoherently**; the platform is stop-and-hop
  ("the transmitter does not move between transmit and receive"); there is no noise or volume
  reverberation; validation is visual plus texture statistics only.
* **Choi21** (Gazebo/GPU multibeam) makes **one point scatterer per ray hit**. Each hit gets the
  Lambert amplitude `sqrt(μ cos²α dA)/r` and a complex Gaussian random factor (after Brown17). The time
  series is then a **coherent sum** in the frequency domain: Gaussian source spectrum, two-way
  `e^{2ikr}/r²`, complex wavenumber for absorption, inverse FFT. Beam sidelobes are added afterwards as a
  matrix over beams. The paper's own limits:
  - the random factor is drawn per ray per frame, so the speckle realisation changes with the
    sampling, and Monte-Carlo noise and speckle cannot be told apart;
  - the only convergence criterion is qualitative ("ray spacing smaller than the resolved area");
  - validation is qualitative, and amplitude is explicitly not the goal.
  Cost on an RTX 2080 Ti: 1.26 s per frame at 114 elevation rays × 512 beams over 60 m.
* **Brown17** (record): the incoherent field of a rough surface can be represented by discrete point
  scatterers with random amplitudes whose mean square equals the scattering cross-section per unit area
  times the area they stand for. This is the formal basis of a point-scatterer seabed. The full text is
  needed for the density and normalisation conditions.
* **Cer17, DeM15, Kwak15, Sac15, Gue10, Gwon17, Pail09, Coi09a/b, Meng23, Pot22/24**: the GPU and
  rasterisation lineage. These simulators take range and incidence per pixel or ray from a depth buffer
  or tube tracer, then apply a Lambert/Phong-type intensity law, a beam weight and noise. Tube tracing
  (Gue10) carries the ray-tube cross-section for spreading. HoloOcean (Pot22/24) adds
  "cluster-based multipath" over an octree. None of the records reports coherent summation with
  world-fixed scatterers, a convergence study, or a field-calibrated absolute level. They are design
  comparators, not evidence.
* **Transient rendering** (Jar14, Jar17, Vel12) is the formal analogue: the path-integral formulation of
  light transport with a time dimension. A path contributes `f(x̄)` at time `t(x̄) = Σ |x_i − x_{i+1}| / c`;
  the sensor integrates over time bins. The mapping is radiance ↔ acoustic intensity, BRDF ↔ bistatic
  scattering cross-section, time bin ↔ range bin, next-event estimation ↔ the visibility ray from each
  scatterer to the receiver. Two differences matter here:
  1. at 450 kHz the field must be summed as complex amplitude (λ = 3.3 mm), not as radiance;
  2. the source and receiver coincide (monostatic) and have strong directivity.

## Equations used by the ray-cast sensor

* Two-way delay of a path with legs `ℓ_j`: `τ = Σ_j ℓ_j / c`. The range bin is
  `b = ⌊(c τ/2 − start) / Δr⌋`, at the encoder's own bin width.
* Coherent baseband field at sample time `t_b`:
  `E(t_b) = Σ_paths Σ_contributors A · s_mf(t_b − τ) · e^{−i ω τ}`.
  Here `A` carries the beam, TL, the scattering amplitude and the reflection coefficients, and `s_mf`
  is the matched/receive-filter response (note 09).
* Point-scatterer normalisation (Brown17 form, to be checked against the full text):
  `E|a_n|² = σ_bistatic(θ_i, θ_s, φ) · ΔA_n`, phase uniform on [0, 2π).

## How it maps to the renderer

`scatterers.py` (world-fixed points on rough triangles), `paths.py` (path enumeration and delay),
`receiver.py` (coherent sum and matched filter), `renderer.py`. Choi21's per-ray random factor is the
specific thing **not** copied: scatterers here are hashed to the world (`renderer._scatter_phase`
idea). Re-sampling the beam then converges to one realisation, and the convergence study can separate
Monte-Carlo error from speckle (`raycast_design.md` §6).

## Caveats

* No source in this note validates absolute level against field data; the sensor's level comes from the
  sonar equation plus one fitted system gain (NC #6 style).
* Stop-and-hop (Bell97a) is replaced here by separate transmit and receive poses (note 09).
* Bell97a interpolates Kirchhoff and perturbation; J&R 2007 (note 03) now recommend the small-slope
  approximation, or composite roughness with an explicit cutoff.

## Full-text reading, 2026-11-03 (Brown17, owner-supplied PDF)

* **Model** (eqs. 1–3): `P(ω) = S0(ω) Σ_n b_T b_R a e^{ik(R_T + R_R)}/(R_T R_R)` and
  `a = f sqrt(σ(θ_i, θ_s, φ; ω)/q_s)`:
  - `q_s` is the scatterer density per m²;
  - **`f = (X + iY)/√2` is a complex circular Gaussian**, so each scatterer's return is itself
    Gaussian;
  - the field from any pair of points is incoherent. This is reasonable when the surface pressure
    correlation length is small (Jackson & Richardson). Otherwise f must be spatially correlated.
  - They use the **small-slope approximation** for σ, as this sensor does.
* **Validation:**
  - the mean-square field vs the narrowband sonar equation over a 100-ping ensemble (agrees except at
    the signal onset);
  - normalised samples on a normal Q–Q line;
  - spatial coherence vs the van Cittert–Zernike theorem.
* **Density:** q_s = 2500 m⁻² at 10 kHz, 7.5 scatterers per wavelength, "near that used by Pouliquen".
  Multiple scattering and shadowing are excluded at low grazing (this sensor's visibility rays add
  shadowing).
* **Adopted (2026-11-03):** `scatterers.rayleigh_factor` gives |f| Rayleigh (|f|² ~ Exp(1),
  world-fixed hash) with the scatterer's phase. The mean uses E|f|² = 1, so the field is Gaussian at
  any density.
  - Consequence for the convergence metric: refining the lattice draws new scatterers, so a same-seed
    realisation does not converge across densities.
  - The Phase-5 metric is therefore on the mean and on the statistics (Brown17's own three tests),
    `raycast_design.md` §6.

