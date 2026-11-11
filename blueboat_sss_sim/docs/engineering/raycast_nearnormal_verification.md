# Ray-cast sensor — near-normal incidence: literature verification (2026-11-06)

Gates amendment 6 (`~/calib/raycast/gates_amendment_6.yaml`, sha256 `162871ff…`, private) requires the
primary literature to be verified **before** any new near-normal physics is implemented. The owner asked
for four things: the exact RMS-slope formulation, its assumptions, its validity near normal incidence,
and how it combines with the existing SSA term without double counting. This note records that
verification and its numerical reference checks. Script: `~/ros2_ws/sim_runs/raycast/tools/
nearnormal_reference.py`; log: `notes/s4/nearnormal_reference.log` (CPU, closed forms, no renderer).

## 1. Sources read

* **NC94** — Novarini, J. C. & Caruthers, J. W. (1994). The partition wavenumber in acoustic
  backscattering from a two-scale rough surface described by a power-law spectrum. *IEEE J. Oceanic
  Eng.* 19(2):200–207. Full text read (owner's PDF); the equations were read from the page images.
* **NC98** — Novarini, J. C. & Caruthers, J. W. (1998). A simplified approach to backscattering from a
  rough seafloor with sediment inhomogeneities. *IEEE J. Oceanic Eng.* 23(3):157–166. Read pp. 157–160.
* **JR07** — Jackson & Richardson (2007), §13.1 (eqs. 13.9–13.18, Fig. 13.2), §13.3 (Kirchhoff and
  its high-frequency limit, eqs. 13.62–13.71, Figs. 13.12/13.14), §13.4 (SSA), §13.5
  (composite roughness, eqs. 13.72–13.83, Fig. 13.19), §6.3 and §6.5 (measured spectra and the
  validity of the power law).

## 2. What the papers say

**The rough facet model (RFM).** The near-specular term is the *facet* component m2 (NC94 eqs. 2–5;
NC98 eqs. 2–5):

* m_f = M · F, with M = |R0|² exp(−g_μ) (NC98 eq. 3; NC94 uses R0 = 1);
* g_μ = 4 σ_μ² k² cos²θ (θ from the vertical);
* F = 1 / (8π δ_f² cos⁴θ) · exp(−tan²θ / (2 δ_f²)) (NC94 eq. 5; the high-frequency Kirchhoff limit of
  Eckart, Beckmann–Spizzichino and Brekhovskikh–Lysanov 9.9.14).

The parameters:
* the microroughness variance is σ_μ² = 2π ∫_{Kc}^{Kh} W K dK;
* the facet slope variance is δ_f² = 2π ∫_{Kl}^{Kc} W K³ dK (NC94 eqs. 9–10);
* NC94's spectrum is W = a² K^(−2b), with K in **cycles/m**;
* **the partition is g_μ0 = 4 σ_μ² k² = 1 at normal incidence.** NC94 found it by fitting the RFM to
  Monte-Carlo Helmholtz–Kirchhoff simulations at 250 Hz – 2 kHz, for angles ≤ 35–43° from the vertical
  only.

**Assumptions:**
* Gaussian heights and slopes;
* an impenetrable bottom (NC94) or a fluid R0 (NC98);
* the far field of the footprint;
* exp(−g_μ) and R0 slowly varying over the slope integral;
* validity only near specular. Wide-angle scatter belongs to m1 (Lambert in NC94; Bragg and volume
  in NC98).

**Combination with the other terms (NC98 §II-B):** the perturbation (Bragg) term is **excluded** near
normal incidence. NC98 call the switch angle "an ad hoc procedure" (from φ0 to normal incidence,
Bragg theory "is suspect and excluded"). In other words, the facet term **replaces** the
small-roughness term near vertical. It is not added to it.

**Slope convention.** JR07 p. 367: "These authors use a version of (13.76) without the factor 1/2, so
their large-scale mean-square slope is twice that given by (13.77)." JR07's σ_s² (eq. 13.68) is the
slope variance of a 1-D track, the one that enters 13.70.

**JR07 on how it relates to the SSA:**
* §13.1: "the Kirchhoff and small-slope approximations for the bistatic scattering cross section
  coincide near the specular direction" (identity L.9).
* §13.4: the lowest-order SSA "produces essentially the same results as the perturbation and Kirchhoff
  approximations when each is applied to the angular region where it is the most accurate".
* §13.5: the RFM (high-frequency Kirchhoff with the cutoff prescription and |V|²/e) is an
  *approximation to the full Kirchhoff* result, and "the high-frequency approximation is consistently
  lower" (Fig. 13.19).
* "With the advent of the small-slope approximation, motivation for use of the composite-roughness and
  Kirchhoff approximations is reduced."
* Eq. 13.18 is the energy statement: near specular, coherent plus incoherent intensity equals the
  flat-interface |V|², when the lobe is narrow.

## 3. Numerical reference checks (`nearnormal_reference.py`)

| Check | Result |
|---|---|
| R1. JR07 13.75 + 13.77 reproduce Fig. 13.19's RMS slopes (20 kHz, γ2 3.25) | 0.0227 / 0.0595 / 0.1432 at c = 1500 m/s: **exact** |
| R2. Our `SSALaw.roughness` vs JR07 Kirchhoff 13.10 near vertical | 0.00 dB at 89°, +0.09 at 85°, +0.37 at 80°, +1.5 at 70°, at 30 and 450 kHz: the L.9 identity holds |
| R3. Energy at normal incidence, ∫σ dΩ vs \|V(90°)\|² | 30 kHz, Fig. 13.2 sand: **−0.02 dB** (the SSA normalisation is verified). 450 kHz: SAX99 −2.0 dB, half-energy cone 60°; attempt-2 fit **−32.8 dB**; 2b feasibility −3.2 dB |
| R4. RFM vs our SSA, backscatter, 450 kHz | RFM is 1.3 dB (SAX99) to 5 dB (feasibility) **below** the SSA at 85–90°; adding it gains at most ≈ +2.6 dB, and that energy is already counted |
| R5. Vertical beam (uniform 3.49 mm strip, tilt 20°): two-way response | grazing 90°: **−55 dB** (0.5·k·H = 3.21, nadir at sin 70°: the strip's first null); 70°: −23.6; 60°: −14.8; 45°: −5.7. With f_v = 0.3: −20.7 at 90° |
| R5. Crest/tail ratio of the flat-seabed sonar equation | changing the roughness set moves it by ≤ 4 dB; f_v 0 → 0.3 gives +1.4…+2.3 dB; a vertical response with no null gives +7.3…+10.5 dB. The real strata need +9.4 (harbour 16.5 m) to +22.7 dB (beach 2.3 m) |
| Coherent nadir channel (`specular.py`'s σ_h, JR07 13.74 above K_low = max(π/r_F, π/Δx)) | at h = 1.33 m: σ_h = 1.8 mm (SAX99), 0.9 mm (w2 1e-6); C = e^(−(2kσ_h)²) = e^−45, e^−11: no coherent return for any power-law set inside the bounds |

## 4. Conclusions

1. **The NC94/JR07 rough-facet term must not be added to our SSA.** It is a lower approximation of the
   full Kirchhoff near-specular result, which the lowest-order SSA already reproduces (R2). Adding it
   double-counts energy that the SSA conserves to 0.02 dB at 30 kHz (R3). NC98 *replace* the Bragg
   term near vertical; they never sum the two. Its size, ≤ +2.6 dB, could not close a 9–23 dB crest
   deficit in any case. Amendment 6's planned contribution is therefore **not implemented**.
2. **The crest deficit comes from three other places**, each quantified above:
   * **(a) The vertical beam's large-angle response.** The uniform-strip model, derived from the
     datasheet's 50° width, places nadir on its first null (−55 dB two-way). The real device returns
     a strong first bottom return at slant = altitude at every altitude, so its response at nadir is
     not near a null. This is the largest single factor, and the present f_v bound (0.3, an
     unpublished, ad hoc ceiling) caps the fix at about +2 dB.
   * **(b) The 450 kHz small-scale spectrum.** Extrapolating the measured power law (measured to
     about 2 cycles/cm; JR07 §6.3/§6.5, where the high-wavenumber break is "not certain") spreads the
     near-specular lobe over ±60° and removes every coherent nadir return. The corpus's FBR altitude
     law (−11.3 dB per decade; NC #4 item 5) is that of an image source, which needs σ_h ≲ 0.5 mm
     above the Fresnel scale. No primary measurement at these scales exists, so a high-wavenumber
     roll-off would be a **new fitted parameter**.
   * **(c) The TVG law** (k fitted in [0, 40]). It trades crest against tail by about 6 dB per decade
     of range (the feasibility fit used this).
3. **Attempt 2's fitted roughness was unphysical.** The SSA with w2 2.36e-5 and γ2 2.6 loses 32.8 dB
   of the reflected energy at normal incidence (R3), so the fit sat outside the approximation's
   validity and the system gain compensated. Energy conservation (JR07 13.18) can serve as a physical
   validity constraint on any fitted set, with no new constant.
4. **The water column (27.8 dB vs 16.7 dB real) is a shared-model issue, not a ray-cast one.** It comes
   from one measured constant (`watercolumn_noise`; 27.8 dB at gain 4). The geometric renderer reports
   the same 27.8 dB. The real water column depends on the acquisition:
   * harbour 600-bin settings: 22–29 dB;
   * harbour 1200-bin settings: 16.7–19.6 dB;
   * beach 2 m altitude: 43 dB, measured 1–1.6 m from the transducer, likely near-field ringing or
     surface reverberation.

   It sets the FBR detector's threshold (floor + 8 dB), and with it NC #4 detection. It is also why
   attempt 1's nadir metric had a zero Jacobian.

## 5. Decision needed (owner)

No further fit is run with the present physics (amendment 6, owner's instruction). The verified paths
are listed in `raycast_phaseA.md` and in the owner question of 2026-11-06.
