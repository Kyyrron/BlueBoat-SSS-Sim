# 03 — Seabed scattering: roughness, volume, impedance, bistatic laws

## Sources

| Key | Reference | DOI | Read |
|---|---|---|---|
| JR07 | Jackson & Richardson, *High-Frequency Seafloor Acoustics*, Springer, 2007 — ch. 6 (roughness), 8 (fluid reflection), 13 (roughness scattering), 14 (volume), App. D, L | 10.1007/978-0-387-36945-7 | full text (local PDF; book page = PDF page − 8) |
| JWI86 | Jackson, Winebrenner, Ishimaru, "Application of the composite roughness model to high-frequency bottom backscattering", *JASA* 79(5):1410–1422, 1986 | 10.1121/1.393669 | record only |
| JB92 | Jackson & Briggs, "High-frequency bottom backscattering: roughness versus sediment volume scattering", *JASA* 92(2):962–977, 1992 | 10.1121/1.403966 | record only |
| WJ98 | Williams & Jackson, "Bistatic bottom scattering: model, experiments, and model/data comparison", *JASA* 103(1):169–181, 1998 | 10.1121/1.421109 | record; model summarised in JR07 §13.2.4 |
| Iva98 | Ivakin, "A unified approach to volume and roughness scattering", *JASA* 103(2):827–837, 1998 | 10.1121/1.421243 | record only |
| Mac61 | Mackenzie, "Bottom reverberation for 530- and 1030-cps sound in deep water", *JASA* 33(11):1498–1504, 1961 (Lambert's law) | 10.1121/1.1908482 | full text (local PDF) |
| Tho88 | Thorsos, "The validity of the Kirchhoff approximation for rough surface scattering using a Gaussian roughness spectrum", *JASA* 83(1):78–92, 1988 | 10.1121/1.396188 | record (pass 4) |
| TJ89 | Thorsos & Jackson, "The validity of the perturbation approximation…", *JASA* 86(1):261–277, 1989 | 10.1121/1.398342 | record (pass 4) |
| Sel85 | Selfridge, "Approximate material properties in isotropic materials", *IEEE Trans. Sonics Ultrason.* 32(3):381–394, 1985 | 10.1109/T-SU.1985.31608 | full text (local); already the source of `worldgen/materials.py` `_ACOUSTIC` |
| Sab02 | Sabol et al., "Evaluation of a digital echo sounder system for detection of submersed aquatic vegetation", *Estuaries* 25(1):133–141, 2002 | 10.1007/BF02696057 | record only |
| APL94 | APL-UW, *High-Frequency Ocean Environmental Acoustic Models Handbook*, TR 9407, 1994 | — (technical report, no DOI) | **not read**; cited only as JR07 cites it |

## What they establish (with JR07 equation numbers)

* **Bistatic cross-section per unit area per steradian, small-roughness perturbation (SPM).**
  `σ = k_w⁴ |A_ww|² W(ΔK)` (13.9). The spectrum is evaluated at the Bragg vector `ΔK = K_s − K_i`
  (13.6); for backscatter `ΔK = 2 k_w cos θ_i` (13.20). For a fluid sediment
  `A_ww = ½ [1 + V(θ_i)][1 + V(θ_s)] G` (13.21), with
  `G = (1 − 1/a_ρ)[cos θ_i cos θ_s cos φ_s − B] − 1 + 1/(a_p² a_ρ)` (13.22) and
  `B = sin θ_pi sin θ_ps / (a_p² a_ρ)`, `sin θ_p = (1 − a_p² cos² θ)^{1/2}` (13.23–13.25).
  `a_p` = complex sediment/water speed ratio `ν_p(1 + iδ_p)`; `a_ρ` = density ratio; `V` = the
  flat-interface reflection coefficient (ch. 8). Valid for `k h ≪ 1` on the scattering scales (TJ89).
* **Roughness spectrum.** The power law `W(K) = w₂ / K^{γ₂}` (6.8, App. D). JR07 Table 13.1 gives sand
  sites at 35–40 kHz:
  - Jacksonville φ 0.51: `a_ρ 1.956, ν_p 1.087, δ_p 0.049, γ₂ 2.47, w₂ 4.55e-6`;
  - SAX99 φ 1.28: `2.00, 1.16, 0.010, 3.04, 4.33e-5`;
  - Panama City 84 φ 2.63: `1.93, 1.115, 0.0185, 1.89, 3.56e-7`;
  - mud, Key West φ 7.0: `1.499, 1.003, 0.0153, 3.29, 4.64e-4`.
  (`w₂` in m^{4−γ₂}.)
* **High-frequency Kirchhoff (facet / geometric optics).**
  `σ = ¼ |V(θ_is)|² (Δk⁴ / Δk_z⁴) p_s(s)` (13.62), with `p_s` the bivariate Gaussian slope PDF
  evaluated at the specular slope `s = ΔK/Δk_z` (13.63–13.66). This is the near-specular (near-nadir)
  lobe.
* **Composite roughness** (JR07 §13.5, JWI86). Split the spectrum at a cutoff `K_c`: the large scales
  only tilt the surface (and shadow it), the small scales scatter by SPM at the local tilted angle.
  The result should not depend on `K_c` between the two scales. JR07 note that the **small-slope
  approximation** (§13.4, App. L.2) now covers both regimes in one expression,
  `σ ∝ ∫ J₀(u) e^{−q u^{2α}} u du` (13.15–13.17) for a power-law spectrum. It is "accurate over a wide
  range of angles", where Kirchhoff alone is too strong at moderate and small grazing angles and SPM
  alone too strong near vertical (Fig. 13.2).
* **Volume scattering.** The equivalent interface cross-section of a uniform sediment volume is
  `σ = |V_wp(θ_i)|² |V_wp(θ_s)|² σ_v / (2 k_w |a_ρ|² Im[P(θ_i) + P(θ_s)])`, with
  `P(θ) = (a_p^{−2} − cos² θ)^{1/2}` (14.7–14.8). It is parameterised by `σ₂ = σ_v / α_p`, with
  "typical values 0.0001 to 0.004" (14.9). Above a sand's critical angle it is suppressed (evanescent
  penetration).
* **Lambert's law** (Mac61): `σ = μ sin θ_i sin θ_s` (monostatic `μ sin² θ`), with μ ≈ −27 dB at
  0.5–1 kHz over deep-water bottoms. It is an empirical form; JR07 ch. 12 shows the sand data depart
  from it at high frequency.
* **Bistatic data** (WJ98): measured bistatic strengths at well-characterised sandy sites agree, within
  measurement error, with the fluid model that uses Kirchhoff near specular, SPM elsewhere, and a smooth
  interpolation between them (JR07 §13.2.4).

## Numbers at 450 kHz (`phase0_numbers.txt`)

* Bragg wavelength `λ_B = λ / (2 cos θ)` ≈ 1.7 mm at grazing incidence. The scattering scale is close to
  grain scale (medium sand: 0.25–0.5 mm grains). The power-law spectrum fitted at 10–1000 rad/m has to
  be extrapolated to ≈ 3800 rad/m, which JR07 §6.5 discusses as unvalidated. **Caveat that drives
  the fit:** `w₂` at 450 kHz is not a measured value for any corpus site → a fitted parameter inside the
  Table 13.1 envelope (`raycast_design.md` §8).
* Sediment penetration: SAX99 sand has `α_p` ≈ 141 dB/m (1/e intensity depth ≈ 3 cm at normal
  incidence) and a critical grazing angle of 30.5°. Below that, the transmitted field is evanescent at
  the λ scale, so volume scattering matters only near nadir on sand; it is larger on mud.
* Smooth hard floors (painted pool concrete; concrete `Z` from Sel85): the coherent reflection
  `|V| ≈ 0.7` and Eckart coherent fraction `C = exp(−(2kσ sin θ)²) ≈ 1` for σ = 1e-5 m
  (`pool_environment_pass4_sonar.yaml`). The backscatter is the small-scale roughness only. The
  project's measured constraint: the pool floor's diffuse level is −16.4 dB re the sand law
  (pass 4, a site overlay).

## Project-measured laws the sensor must reproduce, not import

These come from `docs/sonar_model.md` §6 and `calibration_results.md`:
- the seabed level vs slant tail at the two NC #6 anchors;
- the reported-peak law `66.60 − 11.28 log₁₀ alt` (19 gain-4 recordings);
- the FBR envelope (NC #4).

The ray-cast sensor must land on them from physics plus its bounded fitted parameters. They are its
validation targets (Phase 7), never constants copied into it.

## How it maps to the renderer

`scattering.py`:
- a per-material bistatic table `σ(θ_i, θ_s, φ)`, precomputed from the SSA (or composite roughness
  with an explicit `K_c`) plus the volume term (14.8);
- a Kirchhoff/PO channel for the coherent specular part.

**Key design consequence.** The mesh resolves slopes down to its cell size, which is the composite
model's large scale made explicit. So the scattering law uses only the spectrum above the mesh Nyquist
(`K_c = π / Δx`); tilting and shadowing come from the geometry. Slopes are never counted twice. Rough
aggregate objects take their material's law the same way. Materials are physical constants with
citations; there is never a per-object brightness (NC #17).

## Caveats and open items

* No site in the corpus has a measured roughness spectrum, grain size or sediment sound speed. The
  bottom labels are operator statements (`~/calib/corpus_manifest.yaml`). So the beach "sand" takes the
  Table 13.1 envelope, not one site's values.
* Seagrass: the literature found so far (Sab02, record only) is about detection, not a scattering law.
  It stays **exclude by default**, include-optional as a volume layer only if a world authors it. A
  primary seagrass scattering source is still missing.
* SSA vs composite: to be decided numerically, as the agreement of both on the Table 13.1 sand at
  450 kHz within 1 dB above 10° grazing. The SSA is the reference.
