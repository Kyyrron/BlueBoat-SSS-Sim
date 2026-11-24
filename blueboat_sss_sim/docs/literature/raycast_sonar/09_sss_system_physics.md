# 09 — Side-scan sonar system physics: array, pulse, receiver, platform motion

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| OS-BR | Blue Robotics, "Cerulean Omniscan 450 SS" product page (spec table and footnotes), https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/ | — (web, no DOI) | read 2026-11-02 |
| OS-CS | Cerulean Sonar, "Omniscan 450 SS" product page, https://ceruleansonar.com/products/omniscan-450-ss ; SonarView docs (device controls, mounting) https://docs.ceruleansonar.com/c/sonarview/ | — (web) | read 2026-11-02 |
| PP-OS | Blue Robotics Ping Protocol, "Omniscan450 messages" (os_ping_params 2197, os_mono_profile 2198, sync 169/170), https://docs.bluerobotics.com/ping-protocol/pingmessage-omniscan450/ | — (web) | read 2026-11-02 (also pass 4) |
| Bell97a | Bell & Linnett 1997 — line-array directivity (eq. 13–14), TVG `40 log r + αr` caveat | 10.1049/ip-rsn:19971311 | full text |
| JR07 | Jackson & Richardson 2007, App. F (transducers: far-field condition F.1, source level F.7, receiver sensitivity F.9), App. G (scattering measurements, near-vertical), App. E (complex signals) | 10.1007/978-0-387-36945-7 | full text (local) |
| Tur60 | Turin, "An introduction to matched filters", *IRE Trans. Inf. Theory* 6(3):311–329, 1960 | 10.1109/TIT.1960.1057571 | record only |
| Lur10 | Lurton, *An Introduction to Underwater Acoustics*, 2nd ed., Springer, 2010 | 10.1007/978-3-642-13835-5 | record only (paywalled) |
| Ain10 | Ainslie, *Principles of Sonar Performance Modelling*, Springer, 2010 | 10.1007/978-3-540-87662-5 | record only |
| Mel52 | Mellen, "The thermal-noise limit in the detection of underwater acoustic signals", *JASA* 24(5):478–480, 1952 | 10.1121/1.1906924 | record only |
| Bl09 | Blondel, *The Handbook of Sidescan Sonar*, Springer/Praxis, 2009 | 10.1007/978-3-540-49886-5 | full text (local PDF) |
| JH90 | Johnson & Helferty, "The geological interpretation of side-scan sonar", *Rev. Geophys.* 28(4):357–380, 1990 | 10.1029/RG028i004p00357 | record only |
| COJ92 | Cobra, Oppenheim, Jaffe, "Geometric distortions in side-scan sonar images…", *IEEE JOE* 17(3):252–268, 1992 | 10.1109/48.153442 | full text (local PDF) |
| Wan26 | Wang et al., "A physics-informed and data-driven framework for AUV side-scan sonar imaging quality prediction under motion-induced degradation", *Ocean Eng.* 366:127967, 2026 | 10.1016/j.oceaneng.2026.127967 | abstract (local PDF) |

## Device facts (OS-BR, OS-CS, PP-OS) and what they imply

* 450 kHz; beam width **0.5°, "the nominal theoretical 2-way (transmit and receive) 3 dB beam width"**;
  beam height 50°; **"Far Field: 15 m … the range beyond which the beam width specification is valid.
  At nearer distances the effective beam width gets wider, and sidelobes get larger."**; 20 pps up to
  30 m; resolution 1/1200 of range; transducer 180 g; "50 % longer than the FS".
* **Aperture.** A uniform line aperture has a two-way pattern `sinc⁴(πL sin θ/λ)`. A two-way −3 dB
  full width of 0.5° gives **L ≈ 24 cm** and a one-way −3 dB width of 0.69°. The far field `L²/λ` is
  17.8 m, consistent with the stated 15 m; `2L²/λ` is 36 m. So **the whole 20 m swath is in the near
  field along track.** The footprint there is not `Rθ`: at 2–10 m the along-track footprint is set by
  the 24 cm aperture and Fresnel focusing, against the geometric renderer's `Rθ` = 1.7–8.7 cm
  (`array_length_m: 0` in `default_sonar.yaml`). The vertical aperture (50° one-way) is ≈ 3.5 mm,
  far field beyond 4 mm.
* **CAD check (owner, 2026-11-03).** The transducer housing (STEP model bounding box) is
  **318 × 29 × 20 mm**. The active element sits inside it, so L ≤ ~31 cm. The datasheet-derived 24.6 cm
  (two-way 0.5°) and 22.4 cm (far field 15 m) both fit, leaving ~3.5 cm per end for caps, potting and
  the cable gland. A tapered (apodised) array of the same width would need L ≈ 1.3–1.5 × 24.6 cm,
  which is longer than the housing. So the array is near-uniform, or tapered only lightly. L is held at
  24.6 cm and is a bounded parameter, **[22.4, 31.0] cm** (`raycast_design.md` §8). The 20–29 mm
  cross-section bounds the vertical aperture, consistent with the 3.5 mm strip that 50° implies. The
  housing edges diffract, which is one reason the large-angle vertical response is not a pure sinc
  (the NC #4 question).
* **Sidelobes.** Uniform aperture: first sidelobe −13.3 dB one-way, −26.5 dB two-way. The real
  apodisation is unknown; the datasheet says only that near-range sidelobes are larger.
* **Pulse.** `pulse_len_percent` × round-trip time, floored at **44.3 µs** (measured on every corpus
  recording, `AcquisitionParams.pulse_duration_s`). That is `cτ/2` = 3.3 cm, against bins of 1.67 cm at
  20 m / 1200. A receive filter `filter_duration_percent` exists (0.0015 typical; undocumented). The
  waveform type (CW tone burst vs FM) is **not documented**; a CW burst is assumed. Its matched/receive
  response is a triangle smoothed by the filter (a bell), which the pass-4 lag-2/3 correlations favour
  over a boxcar.
* **Receiver.** Gain index 0–7 or −1 (auto); every ping is normalised to 16-bit on a dB axis,
  `[min_pwr_db, max_pwr_db]` with span ≤ 90 dB. The reported-level ceiling is measured:
  67.81 + 0.91·10 log(τ/66.5 µs) (`encoder.ceiling_at`). The TVG law is **not documented**. The
  geometric model's jointly fitted `tvg_compensation: 0.30` says the device compensates part of the
  range loss. In the ray-cast sensor it is a bounded fit parameter (`raycast_design.md` §8).
* **Two units on one frequency.** The protocol has sync-channel messages (169/170); whether our two
  units synchronise, and whether port hears starboard (back lobe, surface/bottom paths), is **not
  documented. Needs a measurement**: corpus check for drifting interference bands and the cross-side
  correlation of near-range energy.

## Theory used

* **Directivity of a line array** (Bell97a eq. 14; JR07 App. F): far-field `b(θ) = sinc(πL sin θ/λ)`.
  In the near field, integrate over the aperture:
  `b(x) ∝ ∫_{−L/2}^{L/2} w(u) e^{ik(|x − u e_a| − |x|)} du`. With Fresnel's approximation this is a
  Fresnel-integral closed form for uniform `w`. The far-field condition is JR07 F.1.
* **Sonar equation, absolute chain** (JR07 F.5–F.9, G.2; Ain10). Source level `SL`, receive sensitivity
  `RS`, directivities `b_t`, `b_r`; received mean-square pressure from a scatterer
  `= s₀² b_t² b_r² σ ΔA / (r_t² r_r²) · 10^{−α(r_t + r_r)/10}`. `SL` and `RS` of the Omniscan are
  **unpublished**, so their sum enters as the one fitted system gain.
* **Matched filtering** (Tur60): output = cross-correlation of the received signal with the replica,
  range resolution ≈ `c/(2B)`.
* **Thermal noise** (Mel52): spectrum level ≈ −15 + 20 log f[kHz] dB re 1 µPa²/Hz, ≈ 38 dB at 450 kHz,
  ≈ 82 dB re µPa² in a 22.6 kHz band. It is added **before** TVG, so the reported floor rises with
  range. The corpus's measured floors (`noise_floor`, `watercolumn_noise`) are the validation.
* **Platform motion** (COJ92, Wan26, JH90): roll steers the vertical beam (the brightness modulation
  of a USV); pitch moves the beam along track; yaw fans the footprints, giving overlap/gaps; heave
  changes the altitude; speed changes the along-track sampling. All are geometric distortions of the
  image (COJ92 estimates them from line-to-line cross-correlation).

## Numbers for the motion decisions (`phase0_numbers.txt`)

| Effect | Value | Versus |
|---|---|---|
| round trip at 20 m | 26.7 ms | ping period 50 ms |
| yaw during the 20 m round trip at 0.3 / 0.5 / 1.0 rad s⁻¹ | 0.46° / 0.76° / 1.53° | two-way beam 0.5° → **include**: separate Rx pose |
| forward motion during the 20 m round trip at 1.5 m s⁻¹ | 4.0 cm | λ = 3.3 mm, footprint ≥ 24 cm → include (phase and footprint shift) |
| Doppler at 1.5 m s⁻¹ radial (upper bound; broadside radial speed ≈ 0) | 900 Hz = 4 % of the 22.6 kHz band; 0.25 rad over the pulse | **included implicitly** by per-contribution Tx/Rx positions; no separate Doppler term |

## How it maps to the renderer

- `beam.py`: the near-field line-aperture pattern (24 cm from the datasheet, apodisation an open
  parameter held at uniform); vertical tilt 20° as in `default_sonar.yaml`, vertical aperture from 50°.
- `receiver.py`: CW pulse ⊛ receive filter → `s_mf`; thermal noise before TVG; the TVG law; then the
  shared encoder (ceiling, 16-bit normalisation).
- `motion.py`: Tx at t₀, Rx at t₀ + τ per contribution.

## Caveats

* The 24.6 cm aperture is inferred from two datasheet numbers that agree (beam width and far field), and
  is bounded by the CAD housing length (318 mm). It is not measured directly.
* Transmit ringing (55 dB, 22 dB/m measured) is a device leakage, not scattering. It stays the measured
  term, reused from the shared config, and is not re-derived.
